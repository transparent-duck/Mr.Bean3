using System.Net;
using System.Net.Http;
using System.Text;
using System.Text.Json;
using System.Threading.Channels;

namespace Mr.Bean3;

// Framework callbacks only enqueue immutable summaries. Disk and HTTP run off-thread.
internal sealed class UsageOutbox : IDisposable
{
    internal static readonly JsonSerializerOptions Json = new(JsonSerializerDefaults.Web);
    private readonly string directory;
    private readonly Uri endpoint;
    private readonly Action<string> report;
    private readonly HttpClient http;
    private readonly CancellationTokenSource cancel = new();
    private readonly Channel<RunSummary> pending = Channel.CreateBounded<RunSummary>(new BoundedChannelOptions(256)
        { SingleReader = true, SingleWriter = true, FullMode = BoundedChannelFullMode.Wait });
    private readonly SemaphoreSlim wake = new(0, 1);
    private readonly Task persister, uploader;
    private bool disposed;

    public UsageOutbox(string directory, Uri endpoint, Action<string> report, HttpMessageHandler? handler = null)
    {
        if (endpoint.Scheme != "https") throw new ArgumentException("Telemetry requires HTTPS.", nameof(endpoint));
        this.directory = directory;
        this.endpoint = endpoint;
        this.report = report;
        http = new HttpClient(handler ?? new HttpClientHandler { AllowAutoRedirect = false })
            { Timeout = TimeSpan.FromSeconds(10), MaxResponseContentBufferSize = 4096 };
        persister = Task.Run(Persist);
        uploader = Task.Run(Upload);
    }

    public void Enqueue(RunSummary summary)
    {
        if (!pending.Writer.TryWrite(summary)) report("Summary queue is full or closed; one update was not queued.");
    }

    private async Task Persist()
    {
        await foreach (var summary in pending.Reader.ReadAllAsync())
        {
            try
            {
                Directory.CreateDirectory(directory);
                string path = Path.Combine(directory, $"{summary.RunId}.{summary.Revision}.json");
                string temporary = path + ".tmp";
                await File.WriteAllTextAsync(temporary, JsonSerializer.Serialize(summary, Json), new UTF8Encoding(false)).ConfigureAwait(false);
                File.Move(temporary, path, true);
                if (wake.CurrentCount == 0) wake.Release();
            }
            catch (Exception error) { report("Unable to persist summary: " + error.GetType().Name); }
        }
    }

    private async Task Upload()
    {
        var token = cancel.Token;
        var delay = TimeSpan.Zero;
        while (!token.IsCancellationRequested)
        {
            try
            {
                if (delay > TimeSpan.Zero) await Task.Delay(delay, token).ConfigureAwait(false);
                delay = TimeSpan.FromSeconds(30);
                if (!Directory.Exists(directory)) { await wake.WaitAsync(TimeSpan.FromSeconds(30), token).ConfigureAwait(false); continue; }
                var paths = Directory.EnumerateFiles(directory, "*.json").Order(StringComparer.Ordinal).Take(64).ToArray();
                foreach (string path in paths)
                {
                    token.ThrowIfCancellationRequested();
                    if (new FileInfo(path).Length > 32768) { Quarantine(path); continue; }
                    string body = await File.ReadAllTextAsync(path, token).ConfigureAwait(false);
                    using var content = new StringContent(body, Encoding.UTF8, "application/json");
                    using var response = await http.PostAsync(endpoint, content, token).ConfigureAwait(false);
                    if (response.IsSuccessStatusCode)
                    {
                        using var ack = JsonDocument.Parse(await response.Content.ReadAsStringAsync(token).ConfigureAwait(false));
                        using var sent = JsonDocument.Parse(body);
                        if (ack.RootElement.TryGetProperty("accepted", out var accepted) && accepted.ValueKind == JsonValueKind.True &&
                            ack.RootElement.GetProperty("runId").GetString() == sent.RootElement.GetProperty("runId").GetString() &&
                            ack.RootElement.GetProperty("revision").GetInt32() == sent.RootElement.GetProperty("revision").GetInt32())
                            File.Delete(path);
                        else throw new InvalidDataException("Unexpected telemetry acknowledgement.");
                    }
                    else if (response.StatusCode is HttpStatusCode.BadRequest or HttpStatusCode.RequestEntityTooLarge or HttpStatusCode.Conflict)
                    {
                        Quarantine(path);
                        report("Server rejected a summary; retained locally for inspection.");
                    }
                    else
                    {
                        delay = response.Headers.RetryAfter?.Delta ?? TimeSpan.FromMinutes(2);
                        delay = TimeSpan.FromSeconds(Math.Clamp(delay.TotalSeconds, 30, 900));
                        break;
                    }
                }
                if (paths.Length == 64) delay = TimeSpan.FromSeconds(Math.Max(1, delay.TotalSeconds));
                if (paths.Length == 0)
                {
                    await wake.WaitAsync(TimeSpan.FromSeconds(30), token).ConfigureAwait(false);
                    delay = TimeSpan.Zero;
                }
            }
            catch (OperationCanceledException) when (token.IsCancellationRequested) { break; }
            catch (Exception error)
            {
                report("Upload deferred: " + error.GetType().Name);
                delay = TimeSpan.FromMinutes(2);
            }
        }
    }

    private void Quarantine(string path)
    {
        string rejected = Path.Combine(directory, "rejected");
        Directory.CreateDirectory(rejected);
        File.Move(path, Path.Combine(rejected, Path.GetFileName(path)), true);
    }

    public void Dispose()
    {
        if (disposed) return;
        disposed = true;
        pending.Writer.TryComplete();
        cancel.Cancel();
        try { Task.WhenAll(persister, uploader).GetAwaiter().GetResult(); }
        finally { http.Dispose(); cancel.Dispose(); wake.Dispose(); }
    }
}
