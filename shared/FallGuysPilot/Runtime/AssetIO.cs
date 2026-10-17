using System.IO.Compression;
using System.Text.Json;

namespace MrBean.FallGuys;

public static class AssetIO
{
    private static readonly JsonSerializerOptions json=new(){IncludeFields=true};
    public static CompiledCourse Read(Stream source)
    {
        using var zip=new GZipStream(source,CompressionMode.Decompress,leaveOpen:true);
        return new(JsonSerializer.Deserialize<CourseAsset>(zip,json)??throw new InvalidDataException("Missing compiled course."));
    }
    public static void Write(Stream destination,CourseAsset asset)
    {
        using var zip=new GZipStream(destination,CompressionLevel.SmallestSize,leaveOpen:true);
        JsonSerializer.Serialize(zip,asset,json);
    }
}
