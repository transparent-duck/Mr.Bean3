using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Channels;
using System.Threading.Tasks;

namespace MrBean.Modules.Blunderville;

// Only owned scalar/immutable snapshots enter this queue. No native pointer or
// live game object may be read by the writer. Completion drains the finite queue
// and joins the owned task before the session releases its recorder.
internal sealed class SessionRecorder : IDisposable
{
    private sealed record Entry(string? Kind,object Value);
    private readonly Channel<Entry> pending=Channel.CreateBounded<Entry>(new BoundedChannelOptions(8192)
        {SingleReader=true,SingleWriter=true,FullMode=BoundedChannelFullMode.Wait});
    private readonly Task writer;
    private Exception? error;
    private long dropped;
    private static long serializeTicks,ioTicks;
    public static long SerializeTicks=>Interlocked.Read(ref serializeTicks);
    public static long IoTicks=>Interlocked.Read(ref ioTicks);
    public string? Fault=>Volatile.Read(ref error)?.Message;
    public long Dropped=>Interlocked.Read(ref dropped);
    public bool Completed=>writer.IsCompleted;
    public void Complete()=>pending.Writer.TryComplete();
    public SessionRecorder(string path,JsonSerializerOptions json)
        =>writer=Task.Run(()=>Run(path,json));
    public void Write(string kind,object value)=>Enqueue(new(kind,value));
    public void WriteRaw(string line)=>Enqueue(new(null,line));
    private void Enqueue(Entry entry)
    {
        if(!pending.Writer.TryWrite(entry))Interlocked.Increment(ref dropped);
    }
    private async Task Run(string path,JsonSerializerOptions json)
    {
        try
        {
            await using var file=new FileStream(path,FileMode.Create,FileAccess.Write,FileShare.Read,
                65536,FileOptions.Asynchronous);
            await using var output=new StreamWriter(file,new UTF8Encoding(false),65536);
            long flushed=Stopwatch.GetTimestamp();
            await foreach(var entry in pending.Reader.ReadAllAsync())
            {
                long started=Stopwatch.GetTimestamp();
                string line=entry.Kind==null?(string)entry.Value:
                    JsonSerializer.Serialize(new{kind=entry.Kind,value=entry.Value},json);
                Interlocked.Add(ref serializeTicks,Stopwatch.GetTimestamp()-started);
                started=Stopwatch.GetTimestamp();
                await output.WriteLineAsync(line);
                if(Stopwatch.GetElapsedTime(flushed).TotalSeconds>=.5)
                {await output.FlushAsync();flushed=Stopwatch.GetTimestamp();}
                Interlocked.Add(ref ioTicks,Stopwatch.GetTimestamp()-started);
            }
            await output.FlushAsync();
        }
        catch(Exception failure)
        {
            Volatile.Write(ref error,failure);
            pending.Writer.TryComplete();
            while(pending.Reader.TryRead(out _))Interlocked.Increment(ref dropped);
        }
    }
    public void Dispose()
    {
        Complete();
        writer.GetAwaiter().GetResult();
    }
}
