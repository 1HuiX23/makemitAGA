using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // These are the existing game-native gait boundaries. Both the live decision and
    // the transport-subtracted comparison evaluate exactly the same inequalities.
    internal readonly struct FootGaitEvidence
    {
        [Flags] internal enum Trigger { None = 0, NoHit = 1, Clearance = 2, Normal = 4, Tangential = 8 }
        internal const float StanceHeight = .030f, SwingHeight = .055f;
        internal const float StanceTangent = .35f, SwingTangent = .65f, StanceNormal = .22f, SwingNormal = .30f;
        internal readonly int Hits, Surface;
        internal readonly bool Same, Stance;
        internal readonly Trigger Swing;
        internal readonly float Clearance, Tangent, SignedNormal;
        internal readonly Vector3 Normal;

        internal FootGaitEvidence(int hits, bool same, int surface, float clearance, Vector3 normal, float vn, float tangent)
        {
            Hits = hits; Same = same; Surface = surface; Clearance = clearance; Normal = normal;
            SignedNormal = vn; Tangent = tangent;
            float abs = Mathf.Abs(vn);
            Stance = hits == 5 && same && clearance <= StanceHeight && abs <= StanceNormal && tangent <= StanceTangent;
            Swing = (hits == 0 ? Trigger.NoHit : 0) | (clearance >= SwingHeight ? Trigger.Clearance : 0) |
                    (abs >= SwingNormal ? Trigger.Normal : 0) | (tangent >= SwingTangent ? Trigger.Tangential : 0);
        }

        internal FootGaitEvidence WithVelocity(Vector3 velocity)
        {
            float vn = Vector3.Dot(velocity, Normal);
            return new FootGaitEvidence(Hits, Same, Surface, Clearance, Normal, vn, (velocity - Normal * vn).magnitude);
        }
    }

    // Distinguish motion of the character root from motion of the foot within it.
    // The rotation contribution transports the OLD world lever, with no angular derivative.
    internal sealed class FootTransitionSource
    {
        internal struct RenderPose
        {
            internal bool Valid;
            internal Vector3 Target, Foot, TargetSole, Sole;
            internal static RenderPose Capture(Vector3 target, Quaternion sourceRotation, Vector3 foot, Quaternion rotation,
                Vector3 ankleOffset, Vector3 bottomOffset)
            {
                if (!RightFootTorqueForecast.ValidRotation(rotation) || !RightFootTorqueForecast.ValidRotation(sourceRotation) ||
                    !HipPointPairMobility.V(target) || !HipPointPairMobility.V(foot) || !HipPointPairMobility.V(ankleOffset) ||
                    !HipPointPairMobility.V(bottomOffset)) return default;
                return new RenderPose { Valid=true, Target=target+sourceRotation*ankleOffset, Foot=foot+rotation*ankleOffset,
                    TargetSole=target+sourceRotation*bottomOffset, Sole=foot+rotation*bottomOffset };
            }
        }
        internal struct Sample
        {
            internal SourceStepContext Clock;
            internal int Root, Continuity;
            internal RenderPose Visual;
            internal Vector3 Position, Translation, Turn, Relative, World;
            internal bool Valid;
        }
        internal struct Heading
        {
            internal Vector3 Axis;
            internal bool FromRoot;
            internal float Time;
            internal bool Recent(float time) => Axis.sqrMagnitude > .99f && time >= Time && time-Time <= .65f;
        }
        internal Sample Current { get; private set; }
        internal Heading Travel { get; private set; }
        internal int BrakeSequence { get; private set; }
        internal int UnseenBrakes { get; private set; }
        private Vector3 _previousRoot;
        private Quaternion _previousRotation;
        private bool _armed;
        private int _continuity;
        internal const int RenderCapacity = 32;
        private readonly Sample[] _history = new Sample[RenderCapacity];
        internal bool TryRender(in Sample end, int sequence, out Sample sample)
        {
            sample = sequence > 0 ? _history[sequence%RenderCapacity] : default;
            return sample.Valid && sample.Visual.Valid && sample.Clock.Epoch == end.Clock.Epoch && sample.Clock.Sequence == sequence &&
                sample.Root == end.Root && sample.Continuity == end.Continuity;
        }

        // On an exactly level face take its center. On a tilted box take the lowest
        // material point along each axis. Its velocity includes rotation about the ankle.
        internal static Vector3 LowestLever(Quaternion rotation, Vector3 center, Vector3 half)
        {
            Vector3 x = rotation * Vector3.right, y = rotation * Vector3.up, z = rotation * Vector3.forward;
            return rotation * center - x * (Math.Sign(x.y)*half.x) -
                y * (Math.Sign(y.y)*half.y) - z * (Math.Sign(z.y)*half.z);
        }
        internal static Vector3 Planar(Vector3 v) => new Vector3(v.x, 0f, v.z);
        internal void Clear()
        {
            Current = default; Travel = default; _armed = false; ConsumeBrake();
        }
        internal void ConsumeBrake() => BrakeSequence = 0;
        internal void DropBrake()
        {
            if (BrakeSequence != 0) UnseenBrakes++;
            ConsumeBrake();
        }

        internal void Observe(SourceStepContext clock, int root, Vector3 p, Quaternion r, Vector3 foot, RenderPose visual = default)
        {
            var old = Current;
            if (!clock.Valid || root == 0 || !HipPointPairMobility.V(p) || !RightFootTorqueForecast.ValidRotation(r) ||
                !HipPointPairMobility.V(foot))
            {
                DropBrake(); Clear(); return;
            }
            // Preview or another read of the same Render packet must not move the derivative.
            if (clock.Matches(old.Clock) && root == old.Root) return;
            var next = new Sample { Clock = clock, Root = root, Position = foot, Visual = visual };
            float dt = clock.Time - old.Clock.Time;
            if (old.Root == root && old.Clock.Epoch == clock.Epoch && old.Clock.Sequence + 1 == clock.Sequence &&
                dt > .0001f && dt < .25f)
            {
                next.World = (foot - old.Position) / dt;
                next.Translation = (p - _previousRoot) / dt;
                Vector3 lever = old.Position - _previousRoot;
                next.Turn = ((r * Quaternion.Inverse(_previousRotation)) * lever - lever) / dt;
                next.Relative = next.World - next.Translation - next.Turn;
                next.Valid = HipPointPairMobility.V(next.World) && HipPointPairMobility.V(next.Relative);
            }
            if (!next.Valid)
            {
                DropBrake(); Travel = default; _armed = false; _continuity++;
            }
            else
            {
                Vector3 motion = Planar(next.World);
                float speed = motion.magnitude;
                if (speed > FootGaitEvidence.StanceTangent) DropBrake();
                if (speed >= FootGaitEvidence.SwingTangent) _armed = true;
                // A slow final step can enter the torso STOP phase without reaching the
                // swing threshold. Retain its direction when no faster heading exists.
                if (speed > FootGaitEvidence.StanceTangent ||
                    (!Travel.Recent(clock.Time) && speed > FootContactGeometryLimits.RecoveryMaxPlanarSpeed))
                {
                    Vector3 rootTravel = Planar(next.Translation);
                    bool fromRoot = rootTravel.magnitude >= .05f;
                    if (fromRoot || !Travel.FromRoot || !Travel.Recent(clock.Time))
                        Travel = new Heading { Axis = (fromRoot ? rootTravel : motion).normalized, FromRoot = fromRoot, Time = clock.Time };
                }
                if (speed <= FootGaitEvidence.StanceTangent && _armed)
                {
                    if (Travel.Recent(clock.Time)) BrakeSequence = clock.Sequence;
                    _armed = false;
                }
            }
            next.Continuity = _continuity;
            _history[clock.Sequence%RenderCapacity] = next;
            Current = next; _previousRoot = p; _previousRotation = r;
        }
    }
}
