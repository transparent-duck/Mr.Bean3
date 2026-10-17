using System.Globalization;
using System.Security.Cryptography;
using MrBean.Modules.FallGuysPilot;
using MrBean.Host;
using Dalamud.Plugin.Services;

namespace Mr.Bean3;

internal sealed class UsageCollector : IDisposable
{
    private readonly UsageOutbox outbox;
    private readonly string version = typeof(Plugin).Assembly.GetName().Version!.ToString();
    private readonly string mapRevision;
    private RunSummaryBuilder? run;
    private bool partialDuty;

    public UsageCollector(string directory, Uri endpoint, IPluginLog log)
    {
        using var snapshot = typeof(Plugin).Assembly.GetManifestResourceStream("MrBean3.PilotSnapshot")
            ?? throw new InvalidOperationException("Pilot snapshot manifest is missing.");
        mapRevision = Convert.ToHexStringLower(SHA256.HashData(snapshot));
        outbox = new UsageOutbox(Path.Combine(directory, "telemetry-outbox"), endpoint,
            message => log.Warning("[Mr.Bean3 telemetry] {Message}", message));
    }

    public void BeginSession(bool joinedDuty) => partialDuty = joinedDuty;

    public void BeforeUpdate(uint territory)
    {
        if (territory != 1165 || run != null || !Service.PlayerState.IsLoaded) return;
        var state = Service.PlayerState;
        if (state.ContentId == 0 || !state.HomeWorld.IsValid || !state.CurrentWorld.IsValid) return;
        static WorldIdentity World(Lumina.Excel.Sheets.World world)
            => new(world.DataCenter.RowId, world.DataCenter.Value.Name.ToString(), world.RowId, world.Name.ToString());
        var character = new CharacterIdentity(state.ContentId.ToString(CultureInfo.InvariantCulture), state.CharacterName,
            World(state.HomeWorld.Value), World(state.CurrentWorld.Value));
        run = new RunSummaryBuilder(character, version, mapRevision, partialDuty, DateTimeOffset.UtcNow);
        partialDuty = false;
        outbox.Enqueue(run.Initial);
    }

    public void Observe(PilotObservation observation)
    {
        if (run is not { Finished: false } active) return;
        if (active.Observe(observation) is { } result) outbox.Enqueue(active.Finish(result, observation.OccurredAt));
    }

    public void AfterUpdate(uint territory, bool zoning)
    {
        if (territory == 1165 || zoning || run == null) return;
        Finish("unknown");
    }

    public void Finish(string outcome)
    {
        if (run is { Finished: false } active) outbox.Enqueue(active.Finish(outcome, DateTimeOffset.UtcNow));
        run = null;
    }

    public void Dispose() => outbox.Dispose();
}
