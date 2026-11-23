namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // The owner supplies the actual identity, gait and ground decision for this
    // step. The observer substitutes only the candidate shoe geometry.
    internal static class OrdinaryFootEntry
    {
        internal const int RequiredGeometrySamples = 2;
        internal static bool TooDeep(float sole) => sole < -FootContactGeometryLimits.OrdinaryPenetrationMeters;

        internal struct Sample
        {
            internal int Local, Floor;
            internal bool Confirmed, IdentityHeld, LegacyReady, NativeInside;
            internal float NativeSole;

            internal bool GeometryReady(bool latched, bool inside, float sole) =>
                !latched && inside && LegacyReady && sole <= FootContactGeometryLimits.RecoveryMaxGapMeters;
            internal bool CanEnter(int samples) => Confirmed && IdentityHeld && samples >= RequiredGeometrySamples;
        }

        internal enum Decision { Waiting, Miss, Ordinary }
        internal struct Trail
        {
            internal int StrongStart, DepthLatch, ReadySamples, StrongSamples;
            private int _floor;

            internal Decision Observe(in Sample s, bool inside, float sole)
            {
                if (s.Floor == 0 || _floor != s.Floor) this = default;
                if (s.Floor == 0) return Decision.Waiting;
                _floor = s.Floor;
                if (StrongSamples++ == 0) StrongStart = s.Local;
                if (DepthLatch == 0 && TooDeep(sole)) DepthLatch = s.Local;
                ReadySamples = s.GeometryReady(DepthLatch != 0, inside, sole) ? ReadySamples + 1 : 0;
                if (s.Confirmed && DepthLatch != 0) return Decision.Miss;
                return s.CanEnter(ReadySamples) ? Decision.Ordinary : Decision.Waiting;
            }
        }
    }
}
