using System;
using System.Globalization;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    // Render and physics clocks remain distinct. One owner captures Unity time at entry;
    // controller helpers consume the held values, never substitute render dt for solve dt.
    internal struct SourceStepContext
    {
        internal int Epoch, Sequence;
        internal float Time, Dt;
        internal bool Valid => Epoch > 0 && Sequence > 0 && Finite(Time) && Finite(Dt) && Dt >= 0f;
        internal bool Matches(SourceStepContext other) => Valid && other.Valid && Epoch == other.Epoch &&
            Sequence == other.Sequence && Time == other.Time && Dt == other.Dt;
        private static bool Finite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
    }
    internal struct PhysicsStepContext
    {
        internal int Epoch, Sequence, SourceSequence;
        internal float FixedTime, Dt, UnscaledTime;
        internal bool Valid => Epoch > 0 && Sequence > 0 && SourceSequence >= 0 && Finite(FixedTime) && Finite(Dt) && Dt > 0f && Finite(UnscaledTime);
        internal bool Matches(PhysicsStepContext other) => Valid && other.Valid && Epoch == other.Epoch &&
            Sequence == other.Sequence && SourceSequence == other.SourceSequence && FixedTime == other.FixedTime &&
            Dt == other.Dt && UnscaledTime == other.UnscaledTime;
        private static bool Finite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        internal string Status() => "runtimeClock(epoch-fixed-source-time-dt)=" + Epoch + "|" + Sequence + "|" + SourceSequence + "|" +
            FixedTime.ToString("R", CultureInfo.InvariantCulture) + "|" + Dt.ToString("R", CultureInfo.InvariantCulture);
    }
    internal sealed class RuntimeStepClock
    {
        private int _epoch = 1, _sourceSequence, _fixedSequence;
        internal SourceStepContext LastSource { get; private set; }
        internal PhysicsStepContext LastFixed { get; private set; }
        internal SourceStepContext CaptureSource(float time, float dt)
        {
            var next = new SourceStepContext { Epoch = _epoch, Sequence = _sourceSequence + 1, Time = time, Dt = dt };
            if (!next.Valid || (LastSource.Valid && time < LastSource.Time))
                throw new InvalidOperationException("RUNTIME_SOURCE_CLOCK_INVALID_OR_BACKWARD");
            _sourceSequence++; LastSource = next; return next;
        }
        internal PhysicsStepContext CaptureFixed(float fixedTime, float dt, float unscaledTime)
        {
            PhysicsStepContext next = new PhysicsStepContext { Epoch = _epoch, Sequence = _fixedSequence + 1,
                SourceSequence = _sourceSequence, FixedTime = fixedTime, Dt = dt, UnscaledTime = unscaledTime };
            // The integer sequence is the identity. Long-lived Unity float timestamps
            // may round adjacent steps to the same value; equal time is not a reused step.
            if (!next.Valid || (LastFixed.Valid && fixedTime < LastFixed.FixedTime))
                throw new InvalidOperationException("RUNTIME_FIXED_CLOCK_INVALID_OR_BACKWARD");
            _fixedSequence++; LastFixed = next; return next;
        }
        // Trace start/stop must NOT reset the physical clock or a controller derivative.
        internal void InvalidateBinding()
        {
            _epoch++; _sourceSequence = _fixedSequence = 0;
            LastSource = new SourceStepContext(); LastFixed = new PhysicsStepContext();
        }
    }
}
