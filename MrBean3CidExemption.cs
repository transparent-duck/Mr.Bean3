using System.Buffers.Binary;
using System.Security.Cryptography;
using MrBean.Host;

namespace Mr.Bean3;

/// <summary>
/// Content-id exemption table. Entries store PBKDF2 digests rather than ids, so
/// the shipped artifact never contains an exempt id; checking one id costs a
/// single slow KDF evaluation while enumerating the id space is out of reach.
/// </summary>
internal static class MrBean3CidExemption
{
    private const int Iterations = 500_000;
    private const int HashLength = 32;
    private const int MaxCachedVerdicts = 64;

    // PBKDF2-HMAC-SHA256, Iterations, per-entry 16-byte salt, 32-byte digest of
    // the content id as 8-byte big-endian.
    private static readonly (byte[] Salt, byte[] Hash)[] Entries =
    {
        (Convert.FromHexString("09EB0D6FECB55A279D46402D0B821362"),
         Convert.FromHexString("9DB819C3D61E946FC4549F70A5E93833FF754177602FAC2BA96BAF1E79AF19F1")),
    };

    private static readonly Dictionary<ulong, bool> Verdicts = new();
    private static readonly object Sync = new();

    internal static bool IsExemptCurrentContent() => IsExempt(Service.PlayerState.ContentId);

    internal static bool IsExempt(ulong contentId)
    {
        if (contentId == 0)
            return false;

        lock (Sync)
        {
            if (Verdicts.TryGetValue(contentId, out var cached))
                return cached;
            if (Verdicts.Count >= MaxCachedVerdicts)
                Verdicts.Clear();

            var result = Verify(contentId);
            Verdicts[contentId] = result;
            return result;
        }
    }

    private static bool Verify(ulong contentId)
    {
        Span<byte> idBytes = stackalloc byte[8];
        BinaryPrimitives.WriteUInt64BigEndian(idBytes, contentId);
        foreach (var (salt, hash) in Entries)
        {
            var derived = Rfc2898DeriveBytes.Pbkdf2(
                idBytes, salt, Iterations, HashAlgorithmName.SHA256, HashLength);
            if (CryptographicOperations.FixedTimeEquals(derived, hash))
                return true;
        }

        return false;
    }
}

/// <summary>
/// Gate wrapper that bypasses the inner gate and the build-profile region
/// restriction while the current content id verifies against the exemption
/// table. Every other context delegates unchanged.
/// </summary>
internal sealed class MrBean3CidExemptGate : IMrBean3AccessGate
{
    private readonly IMrBean3AccessGate inner;

    internal MrBean3CidExemptGate(IMrBean3AccessGate inner)
    {
        ArgumentNullException.ThrowIfNull(inner);
        this.inner = inner;
    }

    private static bool Exempt => MrBean3CidExemption.IsExemptCurrentContent();

    public MrBean3AccessDecision Current =>
        Exempt ? MrBean3AccessDecision.Allowed() : inner.Current;

    public string StartDenialNotice => Exempt ? string.Empty : inner.StartDenialNotice;

    public bool IsRegionRestrictionExempt => Exempt;

    public bool TryAuthorize(out string error)
    {
        if (Exempt)
        {
            error = string.Empty;
            return true;
        }

        return inner.TryAuthorize(out error);
    }
}
