using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Threading.Tasks;
using MrBean.FallGuys;
using MrBean.FallGuys.Mechanics;
using MrBean.Modules.Blunderville;
using MrBean.Modules.FallGuysPilot.Courses;
using MrBean.Host;
using MrBean.Host.Modules;
using Dalamud.Bindings.ImGui;
using Dalamud.Game.ClientState.Conditions;

namespace MrBean.Modules.FallGuysPilot;

/// <summary>
/// Fall Guys (Blunderville, territory 1165) autopilot. Lifecycle, observation and recording
/// live here; behaviour lives in course scripts. The pilot never stops itself on a model
/// mismatch: it only yields to manual input, server qualification, or a disabled toggle.
/// </summary>
public sealed partial class FallGuysPilot : IDisposable
{
    private const ushort TerritoryDuty = 1165;
    private readonly Configuration configuration;
    private readonly PacketObservations packets;
    private readonly string directory;
    private static readonly JsonSerializerOptions Json = new() { IncludeFields = true, Converters = { new JsonStringEnumConverter() } };

    private CourseId? course;
    private CourseScript? script;
    private Actuator? actuator;
    private PilotEngine? engine;
    private PilotContext? context;
    private SessionRecorder? recorder;
    private string? recordPath;
    private uint entityId;
    private ulong objectId;
    private bool released, sawLock, qualified, controlling, yielded, eligibilityLost;
    private CourseId? interruptedCourse;
    private uint? rank;
    private string issue = "Idle.";
    private double nextProbe, nextFrame, beganAt;
    private int localHits;
    private Task statusWrite = Task.CompletedTask;

    public FallGuysPilot(Configuration configuration, ActionTracker tracker, Action<string> travel)
    {
        this.configuration = configuration;
        this.travel = travel;
        packets = new(tracker);
        directory = Path.Combine(Service.PluginInterface.GetPluginConfigDirectory(), "FallGuysPilot");
    }

    public void Update()
    {
        try { UpdateCore(); UpdateCycle(); }
        catch (Exception error)
        {
            issue = error.Message;
            actuator?.Release();
            controlling = false;
            Service.Log.Error("[FallGuysPilot] " + error);
            if (water.Manual) StopWaterDebug("執行錯誤：" + error.Message);
        }
    }

    private void UpdateCore()
    {
        double now = PilotPose.Now;
        if (water.Manual) { UpdateWaterDebug(now); return; }
        if (Service.LocalPlayer is not { } player || Service.ClientState.TerritoryType != TerritoryDuty ||
            Service.Condition[ConditionFlag.BetweenAreas] || Service.Condition[ConditionFlag.BetweenAreas51])
        {
            if (course != null) EndScene();
            if (Service.ClientState.TerritoryType != TerritoryDuty)
            { eligibilityLost = false; interruptedCourse = null; }
            return;
        }
        if (now >= nextProbe)
        {
            nextProbe = now + 1;
            var detected = CourseScene.Detect(player.Position);
            if (course != null && detected != null && detected != course) EndScene();
            if (course == null && detected is { } scene) BeginScene(scene, player.EntityId, player.GameObjectId);
        }
        if (course == null || actuator == null || context == null) return;
        if (player.GameObjectId != objectId) { EndScene(); return; }

        Drain();
        ObserveTerminal(now);
        if (!PilotPose.TryRead(actuator.Movement, out var pose)) return;
        sawLock |= pose.Has(3782);
        if (sawLock && !pose.Has(3782) || !sawLock && now - beganAt > 15) MarkReleased(now); // no lock seen: joined mid-round
        context.Pose = pose; context.Now = now;
        if (released && !qualified && !eligibilityLost && pose.Has(3708))
        {
            eligibilityLost = true;
            Write("eliminated", new { at = now, course, evidence = "current-status-3708" });
        }
        actuator.Update(now);

        bool wanted = configuration.BlundervilleEnabled && script != null && !eligibilityLost && !qualified && !yielded && !player.IsDead;
        if (actuator.CancelledByUser && controlling)
        {
            yielded = true; wanted = false;
            Write("yield", new { at = now, reason = "manual input" });
            SetLoop(false, "manual input");
        }
        if (actuator.NativeFault is { } fault) { issue = "Native callback fault: " + fault.Message; wanted = false; }
        if (wanted)
        {
            controlling = true;
            engine ??= new PilotEngine(script!, pose, context);
            ObserveRingSources(now);
            if (!released) actuator.Hold();
            else engine.Tick(context);
            issue = $"{engine.Current?.Name ?? "end"} {(engine.Current as GateStep)?.Status}";
        }
        else if (controlling)
        {
            controlling = false;
            actuator.Release();
        }
        if (now >= nextFrame)
        {
            UpdateHazardOverlay(now);
            nextFrame = now + 0.1;
            Write("frame", new
            {
                at = now, feet = pose.Feet, pose.Facing, motion = pose.Motion, pose.Grounded, pose.Speed, force = pose.Force,
                statuses = pose.Raw.Statuses.Select(s => s.Id).ToArray(), step = engine?.Current?.Name,
                gate = (engine?.Current as GateStep)?.Status, command = actuator.LastCommand, controlling,
                nudges = actuator.Nudges, jumps = actuator.Jumps,
                jump = pose.Jump, actuator.JumpDispatchAt, actuator.JumpDispatchFeet, actuator.JumpAccepted,
                actuator.JumpSpeedBefore, actuator.JumpTargetSpeed,
                arrow = pose.Raw.ArrowDirection, arrowTarget = actuator.ArrowPreference,
                actuator.ArrowEntryWrites, actuator.ArrowTurnSamples,
                actuator.ArrowTurnSeconds, actuator.ArrowTurnRadians,
                transport = pose.Transport,
                steeringClock = pose.Raw.SteeringClock,
            });
        }
    }

    private void BeginScene(CourseId scene, uint entity, ulong obj)
    {
        // A loading boundary can be a same-course checkpoint respawn. Only
        // advancing to another course without qualification implies spectating.
        bool checkpoint = interruptedCourse == scene && released;
        if (interruptedCourse is { } previous && previous != scene) eligibilityLost = true;
        interruptedCourse = null;
        course = scene; entityId = entity; objectId = obj; beganAt = PilotPose.Now;
        ++sceneSequence;
        released = sawLock = qualified = controlling = yielded = false; rank = null; localHits = 0;
        script = CourseScripts.Create(scene);
        actuator = new Actuator(); // installs hooks while the start lock holds the actor
        water.UseObservation();
        if (scene == CourseId.Hit) _ = water.Cache;
        context = new PilotContext { Act = actuator, Hazards = new HazardField(script?.Hazards ?? []), Log = Write, Water = water };
        engine = null;
        packets.Start();
        Directory.CreateDirectory(directory);
        recordPath = Path.Combine(directory, $"{scene}-{DateTime.Now:yyyyMMdd-HHmmss}.jsonl");
        recorder = new SessionRecorder(recordPath, Json);
        issue = script == null ? $"{scene}: no script yet; recording only." : eligibilityLost ? "Spectating (not qualified last round)." : "Waiting for start.";
        Write("start", new { course = scene, at = PilotPose.Now, scripted = script != null, eligibilityLost, enabled = configuration.BlundervilleEnabled,
            assembly = typeof(FallGuysPilot).Assembly.ManifestModule.ModuleVersionId });
        if (checkpoint) MarkReleased(beganAt);
    }

    private void MarkReleased(double at)
    {
        if (released) return;
        released = true;
        if (context != null) context.ReleasedAt = at;
        Write("release", new { at });
    }

    private void Drain()
    {
        while (packets.TryRead(out var observation))
        {
            Write("observation", observation);
            context!.Hazards.Observe(observation);
            switch (observation)
            {
                case MechanicEffectObservation effect when effect.Targets.Any(t => t.ObjectId == objectId && t.Effects.Any(HazardField.IsHit)):
                    ++localHits;
                    context!.LastHitAt = effect.Seconds;
                    if (effect.ActionId == 34800) context.LastWindHitAt = effect.Seconds;
                    Write("local-hit", new { at = effect.Seconds, effect.ActionId, effect.Origin, step = engine?.Current?.Name, feet = context.Pose.Feet });
                    break;
                case MechanicControlObservation { Category: 21, Arg1: 3782 } c when c.EntityId == entityId:
                case MechanicStatusObservation { StatusId: 3782, Gained: false } s when s.ObjectId == objectId:
                    MarkReleased(observation.Seconds);
                    break;
                case MechanicStatusObservation { StatusId: 1548, Gained: true } delivery
                    when course == CourseId.Crystal && delivery.ObjectId == objectId:
                    context.LastDeliveryAt = delivery.Seconds;
                    Write("crystal-delivery", new { at = delivery.Seconds, evidence = "delivery-speed-status", final = false });
                    break;
                case MechanicStatusObservation { StatusId: 3708, Gained: true } eliminated
                    when eliminated.ObjectId == objectId && released && !qualified:
                    eligibilityLost = true;
                    Write("eliminated", new { at = eliminated.Seconds, course, evidence = "status-3708" });
                    break;
                case MechanicControlObservation { Category: 109, Arg1: 0x8004002B, Arg2: 0xB, Arg3: 2, IsRecorded: false } r
                    when r.Arg4 == entityId && r.Arg5 > 0 && !qualified:
                    qualified = true; rank = r.Arg5; eligibilityLost = false;
                    Write("qualified", new { at = r.Seconds, rank, course, localHits });
                    break;
                case MechanicControlObservation { Category: 109, Arg1: 0x8004002B, Arg2: 4, IsRecorded: false } w
                    when course == CourseId.Crown && w.Arg3 == entityId && !qualified:
                    qualified = true; rank = 1;
                    Write("winner", new { at = w.Seconds, course, localHits });
                    cycleWon = true;
                    CycleWrite("winner", new { at = w.Seconds, recordPath });
                    break;
                case MechanicControlObservation { Category: 109, Arg1: 0x8004002B, Arg2: 4, IsRecorded: false } ended
                    when released && !qualified && !eligibilityLost &&
                         (course == CourseId.Crown || ended.Arg3 == 0xE0000000):
                    // In races/survival, Arg3 first names the leading player;
                    // only the later invalid-object marker ends the round.
                    // Crown ends immediately when any player wins. Own winner
                    // handling above retains its positive server evidence.
                    eligibilityLost = true; terminal = true;
                    Write("eliminated", new { at = ended.Seconds, course, evidence = "director-round-end", ended.Arg3 });
                    break;
            }
        }
    }

    private void EndScene()
    {
        Drain();
        // Qualification is independent of movement ownership. Manual takeover
        // must not let an unqualified player control the next spectator round.
        // Defer the decision until the next course identity is known.
        if (released && !qualified && Service.ClientState.TerritoryType == TerritoryDuty)
            interruptedCourse = course;
        Write("scene-end", new { at = PilotPose.Now, qualified, rank, localHits, eligibilityLost });
        if (recorder is { } finished) { finished.Complete(); Task.Run(finished.Dispose); }
        recorder = null;
        actuator?.Dispose(); actuator = null;
        packets.Stop();
        engine = null; context = null; script = null; course = null; controlling = false;
        ResetHazardOverlay();
        ringSourcePoses.Clear(); nextSourcePose = 0;
    }

    private void Write(string kind, object value)
    {
        recorder?.Write(kind, value);
        PublishObservation(kind);
    }

    public void HandleCommand(string[] args)
    {
        string command = args.FirstOrDefault() ?? "status";
        if (command == "loop")
        {
            string option = args.ElementAtOrDefault(1) ?? "status";
            if (option is "on" or "off") SetLoop(option == "on", "command");
            else if (option != "status") throw new ArgumentException("Use fallguys loop on, off or status.");
            command = "status";
        }
        if (command == "water") { HandleWaterCommand(args.Skip(1).ToArray()); return; }
        if (command == "graph") { HandleGraphCommand(args.Skip(1).ToArray()); return; }
        if (command == "geometry")
        {
            if (Service.ClientState.TerritoryType != TerritoryDuty) throw new InvalidOperationException("Enter Blunderville before capturing geometry.");
            var snapshot = PilotCollisionCapture.Capture(new(-35, -5, 108), new(35, 55, 334));
            Directory.CreateDirectory(directory);
            string path = Path.Combine(directory, "crown-collision.json");
            File.WriteAllText(path, JsonSerializer.Serialize(snapshot, Json));
            Service.Log.Information("[FallGuysPilot] Collision snapshot: " + path);
            return;
        }
        if (command == "overlay")
        {
            hazardOverlay = args.ElementAtOrDefault(1) != "off";
            command = "status";
        }
        if (command == "start")
        {
            if (water.Manual) { StopWaterDebug("返回正常流程"); water.UseObservation(); }
            configuration.BlundervilleEnabled = true; configuration.Save(); yielded = false;
        }
        else if (command == "stop") { SetLoop(false, "stop"); StopWaterDebug("已停止"); configuration.BlundervilleEnabled = false; configuration.Save(); }
        else if (command != "status") throw new ArgumentException("Use fallguys start, stop or status.");
        var status = new
        {
            enabled = configuration.BlundervilleEnabled, zone = Service.ClientState.TerritoryType, position = Service.LocalPlayer?.Position,
            loop = configuration.BlundervilleAutoLoop, cycleActive, cycleWon, completedCycles, wonCycles, cycleRecordPath,
            course, scripted = script != null, released, qualified, rank, controlling, yielded, eligibilityLost, localHits,
            step = engine?.Current?.Name, stepIndex = engine?.Index, steps = engine?.Count, issue, recordPath,
            packetFaults = packets.Faults, packetDropped = packets.Dropped, recordDropped = recorder?.Dropped ?? 0,
            assembly = typeof(FallGuysPilot).Assembly.ManifestModule.ModuleVersionId,
            overlay = new { enabled = hazardOverlay, stations = hazardDraw.Count, overlayActive, overlayUnknown, overlayProjected },
        };
        var preceding = statusWrite;
        statusWrite = Task.Run(async () =>
        {
            await preceding.ConfigureAwait(false);
            try
            {
                var text = JsonSerializer.Serialize(status, Json);
                Directory.CreateDirectory(directory);
                File.WriteAllText(Path.Combine(directory, "last-status.json"), text);
                Service.Log.Information("[FallGuysPilot] " + text);
            }
            catch (Exception failure) { Service.Log.Error("[FallGuysPilot] Status write failed: " + failure); }
        });
    }

    public void DrawSettings()
    {
        ImGui.TextUnformatted("糖豆人自動導航（Pilot）");
        bool enabled = configuration.BlundervilleEnabled;
        if (ImGui.Checkbox("啟用自動導航", ref enabled)) HandleCommand([enabled ? "start" : "stop"]);
        bool loop = configuration.BlundervilleAutoLoop;
        if (ImGui.Checkbox("自動報名、退本並繼續挑戰", ref loop)) SetLoop(loop, "settings");
        if (ImGui.Button("立即停止")) HandleCommand(["stop"]);
        ImGui.TextUnformatted($"關卡: {course?.ToString() ?? "-"}  {(qualified ? $"已晉級 #{rank}" : "")}");
        ImGui.TextWrapped(issue);
        ImGui.Checkbox("顯示目前關卡 AoE", ref hazardOverlay);
        if (hazardOverlay) ImGui.TextWrapped("紅：生效；橙：1 秒內；灰：等待；藍：時序未知。顯示已建模範圍，規劃另加角色半徑。缺少的模型不會憑空顯示。");
        if (yielded) ImGui.TextWrapped("偵測到手動操作，本回合已交還控制。");
        DrawWaterDebugSettings();
        DrawRouteSettings();
    }

    public void Dispose()
    {
        if (course != null) EndScene();
        CloseCycle();
        packets.Dispose();
        StopWaterDebug("已卸載");
        statusWrite.GetAwaiter().GetResult();
    }
}
