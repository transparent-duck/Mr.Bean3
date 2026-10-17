using System.Text.Json;
using MrBean.Modules.FallGuysPilot;

namespace Mr.Bean3;

internal sealed class LocalStatistics
{
    private sealed record SavedTotals(long TotalRuns = 0, long Wins = 0, long Failures = 0,
        long Interrupted = 0, bool PendingDuty = false);

    private readonly string path;
    private readonly Action<Exception> report;
    private SavedTotals totals;
    private bool session, dutySeen, finished, partialDuty, dirty;
    private long nextSave;

    public long TotalRuns => totals.TotalRuns;
    public long Wins => totals.Wins;
    public long Failures => totals.Failures;
    public long Interrupted => totals.Interrupted;
    public double WinRate => Wins + Failures == 0 ? 0 : (double)Wins / (Wins + Failures);

    public LocalStatistics(string directory, Action<Exception> report)
    {
        path = Path.Combine(directory, "statistics.json");
        this.report = report;
        totals = Load();
        if (totals.PendingDuty)
        {
            totals = totals with { Interrupted = totals.Interrupted + 1, PendingDuty = false };
            Save();
        }
    }

    public void BeginSession(bool alreadyInDuty)
    {
        Stop();
        session = true;
        partialDuty = alreadyInDuty;
    }

    public void UpdateTerritory(uint territory, bool zoning)
    {
        if (!session || zoning) return;
        if (territory == 1165)
        {
            if (dutySeen) return;
            dutySeen = true;
            finished = false;
            totals = totals with { TotalRuns = totals.TotalRuns + 1, PendingDuty = true };
            Save();
        }
        else if (dutySeen)
        {
            Complete("interrupted");
            dutySeen = false;
            partialDuty = false;
        }
    }

    public void Observe(PilotObservation observation)
    {
        if (!session || !dutySeen || finished) return;
        if (observation.ManualIntervention || observation.Kind == "yield") Complete("interrupted");
        else if (observation.Kind == "winner") Complete("won");
        else if (observation.Kind == "eliminated") Complete("failed");
    }

    private void Complete(string outcome)
    {
        if (!dutySeen || finished) return;
        finished = true;
        if (partialDuty) outcome = "interrupted";
        totals = totals with
        {
            Wins = totals.Wins + (outcome == "won" ? 1 : 0),
            Failures = totals.Failures + (outcome == "failed" ? 1 : 0),
            Interrupted = totals.Interrupted + (outcome == "interrupted" ? 1 : 0),
            PendingDuty = false,
        };
        Save();
    }

    public void Stop()
    {
        if (session) Complete("interrupted");
        session = dutySeen = partialDuty = false;
    }

    private SavedTotals Load()
    {
        foreach (var candidate in new[] { path, path + ".bak" })
        {
            if (!File.Exists(candidate)) continue;
            try
            {
                var value = JsonSerializer.Deserialize<SavedTotals>(File.ReadAllText(candidate));
                if (value == null || value.TotalRuns < 0 || value.Wins < 0 || value.Failures < 0 || value.Interrupted < 0 ||
                    (decimal)value.Wins + value.Failures + value.Interrupted + (value.PendingDuty ? 1 : 0) != value.TotalRuns)
                    throw new JsonException("Invalid statistics totals.");
                return value;
            }
            catch (Exception error) when (error is IOException or UnauthorizedAccessException or JsonException)
            { report(error); }
        }
        return new();
    }

    private void Save()
    {
        dirty = true;
        FlushPending(true);
    }

    public void FlushPending(bool force = false)
    {
        if (!dirty || !force && Environment.TickCount64 < nextSave) return;
        nextSave = Environment.TickCount64 + 5000;
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            File.WriteAllText(path + ".tmp", JsonSerializer.Serialize(totals));
            File.Move(path + ".tmp", path, true);
            File.Copy(path, path + ".bak", true);
            dirty = false;
        }
        catch (Exception error) when (error is IOException or UnauthorizedAccessException)
        { report(error); }
    }
}
