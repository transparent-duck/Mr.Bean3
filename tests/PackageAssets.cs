using System.IO.Compression;
using System.Reflection.Metadata;
using System.Reflection.PortableExecutable;
using System.Text.Json;
using MrBean.Modules.FallGuysPilot.Water;

internal static class PackageAssets
{
    public static void Check(string path)
    {
        using var archive = ZipFile.OpenRead(path);
        using var source = archive.GetEntry("Mr.Bean3.dll")!.Open();
        using var bytes = new MemoryStream();
        source.CopyTo(bytes);
        bytes.Position = 0;
        using var pe = new PEReader(bytes);
        var metadata = pe.GetMetadataReader();
        var resources = new Dictionary<string, byte[]>();
        int rva = pe.PEHeaders.CorHeader!.ResourcesDirectory.RelativeVirtualAddress;
        foreach (var handle in metadata.ManifestResources)
        {
            var resource = metadata.GetManifestResource(handle);
            if (!resource.Implementation.IsNil) continue;
            string name = metadata.GetString(resource.Name);
            var reader = pe.GetSectionData(rva + (int)resource.Offset).GetReader();
            resources.Add(name, reader.ReadBytes(reader.ReadInt32()));
        }
        using var manifest = JsonDocument.Parse(resources["MrBean3.PilotSnapshot"]);
        if (!resources.ContainsKey("MrBean.Pilot.HitWater")) throw new InvalidDataException("Missing map: MrBean.Pilot.HitWater");
        int maps = 0;
        foreach (var file in manifest.RootElement.GetProperty("files").EnumerateArray())
        {
            string relative = file.GetProperty("path").GetString()!;
            if (!relative.StartsWith("Assets/")) continue;
            string name = Path.GetFileName(relative) switch
            {
                "hit-water.bin.gz" => "MrBean.Pilot.HitWater",
                "hit-ground.json.gz" => "MrBean.Pilot.HitGround",
                var asset => "MrBean.Pilot." + asset,
            };
            if (!resources.TryGetValue(name, out var content)) throw new InvalidDataException("Missing map: " + name);
            using var stream = new MemoryStream(content);
            if (name == "MrBean.Pilot.HitWater")
            {
                var water = WaterRoutes.Read(stream);
                if (water.Nodes.Length == 0 || water.Edges.Length == 0 || water.Goals.Length == 0)
                    throw new InvalidDataException("Empty water routes.");
            }
            else
            {
                using var gzip = new GZipStream(stream, CompressionMode.Decompress);
                using var json = JsonDocument.Parse(gzip);
                if (json.RootElement.ValueKind is not (JsonValueKind.Object or JsonValueKind.Array))
                    throw new InvalidDataException("Invalid map: " + name);
            }
            ++maps;
        }
        if (maps == 0) throw new InvalidDataException("Package has no maps.");
        Console.WriteLine($"Packaged maps loaded: {maps}; water routes loaded with production reader.");
    }
}
