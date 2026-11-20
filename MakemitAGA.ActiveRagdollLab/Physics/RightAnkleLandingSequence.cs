using System;
using System.Globalization;
using System.Text;
using UnityEngine;
using MakemitAGA.ActiveRagdollLab.Model;
using Forecast = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast;
using Band = MakemitAGA.ActiveRagdollLab.Physics.RightFootTorqueForecast.HeightBand;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Each lane keeps its own horizontal COM position/velocity difference for a
    // whole uninterrupted pre-contact segment. The lower leg and the source are
    // conditioned on the observed trajectory. A terminal torque is scored for
    // one interval only and never propagated into any lane.
    internal sealed class RightAnkleLandingSequence
    {
        internal const int Count = 3;
        internal static float Fraction(int i) => (1 << i) * .25f;
        internal struct Tether
        {
            internal Vector3 Raw;
            internal float SelectedCap, BaseCap, Kp, Kd, HardGap;
        }
        [Flags]
        internal enum Mark : ushort
        {
            None=0, Issued=1, Measured=2, Clipped=4, Outside=8, HardGap=16, GapPass=32,
            SoleAndGap=64, HeldAndGap=128, TorqueGain=256, TorqueCross=512, Invalid=1024, TorqueTested=2048
        }
        internal struct LaneRead
        {
            internal Mark Flags;
            internal float Gap, Xz, TorqueGain, TorqueGapBound;
            internal Forecast.Status TorqueState;
            internal Forecast.Reject TorqueRejections;
            internal bool Has(Mark m) => (Flags & m) == m;
        }
        internal struct Input
        {
            internal Vector3 AnkleDelta, AppliedA;
            internal float Dt, Drag, Cap;
            internal bool PairAvailable;
        }
        internal struct Readback
        {
            internal int Origin, Depth, FromShared, FromSource;
            internal Input Command;
            internal float EndX, EndZ;
            internal float Gap, Xz, SignedY, Box, GoalFoot, GoalCom;
            internal LaneRead Quarter, Half, Full;
            internal bool OriginalTorqueCross;
            internal bool GapPass => Gap <= FootContactGeometryLimits.RecoveryMaxAnchorGapMeters;
            internal bool SoleAndGap => GapPass && InBand(Box);
            internal LaneRead Lane(int i) => i==0 ? Quarter : i==1 ? Half : Full;
        }
        private struct Lane
        {
            internal Vector3 Offset, Velocity;
            internal LaneRead Read;
            internal float TrialHeight;
            internal bool Live;
        }
        private readonly Lane[] _lanes = new Lane[Count];
        private readonly int[] _issued = new int[Count], _measured = new int[Count], _clipped = new int[Count],
            _outside = new int[Count], _hard = new int[Count];
        private int _origin, _depth, _segments, _qualified, _missing, _saturated, _paired, _multi, _broken;
        private bool _pending;
        private float _hardGap;
        private Input _input;
        private static Vector3 Flat(Vector3 v) => new Vector3(v.x,0f,v.z);
        private static bool InBand(float v) => RightFootTorqueForecast.Band(v)==Band.InBand;
        private static bool Inside(in RightKneePivot.Frame b, Vector3 foot, Quaternion r) =>
            FootBoxGeometry.Inside(foot,r,b.Center,b.Half,b.FloorCenter,b.FloorHalf);

        internal void Cut()
        {
            if (_pending) _broken++;
            _pending=false; _origin=0; _depth=0;
        }
        internal bool CouldUsePair(in Forecast.Pose p)
        {
            if (_origin==0) return false;
            for (int i=0;i<Count;i++)
                if (_lanes[i].Live && (p.Box.FootAnkle-p.Box.LowerAnkle+_lanes[i].Offset).magnitude
                    <=FootContactGeometryLimits.RecoveryMaxAnchorGapMeters) return true;
            return false;
        }
        internal void Issue(int local, in Forecast.Pose p, in Forecast.Motion m, in Forecast.Result original,
            in Forecast.Pair pair, in Tether t)
        {
            _qualified++;
            if (local<=0 || !original.Predicted || !HipPointPairMobility.V(t.Raw) || !HipPointPairMobility.V(m.AppliedLinearA) ||
                !PelvisParentFrame.F(t.SelectedCap) || !PelvisParentFrame.F(t.BaseCap) || !PelvisParentFrame.F(t.Kp) ||
                !PelvisParentFrame.F(t.Kd) || !PelvisParentFrame.F(t.HardGap) || t.SelectedCap<=0 || t.BaseCap<=0 ||
                t.Kp<=0 || t.Kd<0 || t.HardGap<=FootContactGeometryLimits.RecoveryMaxAnchorGapMeters)
            { _missing++; Cut(); return; }
            float cap=Math.Min(t.SelectedCap,t.BaseCap);
            // A clamped baseline has coupled X/Y/Z. Never attribute that redistribution
            // to a horizontal spring, or borrow a higher transient capture allowance.
            if ((t.Raw-m.AppliedLinearA).sqrMagnitude>1e-10f || m.AppliedLinearA.magnitude>cap+1e-5f)
            { _saturated++; Cut(); return; }
            if (_pending || (_origin!=0 && _origin+_depth!=local)) Cut();
            if (_origin==0)
            {
                _segments++; _origin=local;
                for (int i=0;i<Count;i++) _lanes[i]=new Lane { Live=true };
            }
            _depth++; _pending=true; _hardGap=t.HardGap;
            _input=new Input { AnkleDelta=p.Box.FootAnkle-p.Box.LowerAnkle, AppliedA=m.AppliedLinearA,
                Dt=p.Box.Dt, Drag=m.LinearDrag, Cap=cap, PairAvailable=pair.LowerMass>0f };
            float dt=p.Box.Dt, drag=Math.Max(0f,1f-m.LinearDrag*dt);
            float budget=(float)Math.Sqrt(Math.Max(0f,cap*cap-m.AppliedLinearA.y*m.AppliedLinearA.y));
            for (int i=0;i<Count;i++)
            {
                ref var lane=ref _lanes[i];
                lane.Read=default; lane.TrialHeight=original.Box;
                if (!lane.Live) continue;
                var offset=lane.Offset;
                var error=p.Box.LowerAnkle-p.Box.FootAnkle-offset;
                if (error.magnitude>=_hardGap) { Stop(ref lane, i, Mark.HardGap); continue; }
                if (!Inside(p.Box,p.Box.FootAnkle+offset,p.Box.FootR)) { Stop(ref lane,i,Mark.Outside); continue; }
                // Subtract the original PD response to the lane's accumulated offsets
                // before applying the extra position gain to its own remaining error.
                var raw=Flat(t.Raw) - offset*t.Kp - lane.Velocity*t.Kd + Flat(error)*(Fraction(i)*t.Kp);
                var command=Vector3.ClampMagnitude(raw,budget);
                var velocity=(lane.Velocity+(command-Flat(m.AppliedLinearA))*dt)*drag;
                var next=offset+velocity*dt;
                if (!HipPointPairMobility.V(raw) || !HipPointPairMobility.V(velocity) || !HipPointPairMobility.V(next))
                { Stop(ref lane,i,Mark.Invalid); continue; }
                if (!Inside(p.Box,original.Foot+next,original.R)) { Stop(ref lane,i,Mark.Outside); continue; }
                lane.Offset=next; lane.Velocity=velocity; lane.Read.Flags=Mark.Issued; _issued[i]++;
                if (raw.sqrMagnitude>budget*budget) { lane.Read.Flags|=Mark.Clipped; _clipped[i]++; }
                if (!original.Opportunity || error.magnitude>FootContactGeometryLimits.RecoveryMaxAnchorGapMeters || pair.LowerMass<=0f) continue;
                var state=p; state.Box.FootAnkle+=offset; state.Com+=offset; state.Gap=error.magnitude;
                var trial=new Forecast.Result { State=Forecast.Status.MissingDrive, Com=original.Com+next,
                    Foot=original.Foot+next, R=original.R, Box=original.Box, TrialBox=original.Box };
                Forecast.Compare(state,m,pair,ref trial);
                lane.Read.Flags|=Mark.TorqueTested;
                lane.Read.TorqueState=trial.State; lane.Read.TorqueRejections=trial.Rejected;
                if (trial.State!=Forecast.Status.Favorable) continue;
                lane.Read.TorqueGain=trial.TrialBox-original.Box; lane.Read.TorqueGapBound=trial.ExtraGapBound;
                lane.TrialHeight=trial.TrialBox;
            }
        }
        private void Stop(ref Lane lane,int index,Mark reason)
        {
            lane.Live=false; lane.Read.Flags|=reason;
            if (reason==Mark.Outside) _outside[index]++;
            if (reason==Mark.HardGap) _hard[index]++;
        }
        internal Readback Read(in Forecast.Pose actual, in Forecast.Result original)
        {
            if (!_pending) return default;
            _pending=false; _paired++; if (_depth>=2) _multi++;
            var delta=actual.Box.FootAnkle-actual.Box.LowerAnkle;
            var n=new Readback { Origin=_origin, Depth=_depth, Command=_input, EndX=delta.x, EndZ=delta.z, Gap=delta.magnitude, Xz=Flat(delta).magnitude,
                SignedY=delta.y, Box=actual.Box.Box, GoalFoot=original.GoalAtFoot, GoalCom=original.GoalAtCom };
            n.OriginalTorqueCross=original.EntersBand && RightFootTorqueForecast.Band(n.Box)==Band.Below &&
                InBand(n.Box+original.TrialBox-original.Box) && n.Gap+original.ExtraGapBound<=FootContactGeometryLimits.RecoveryMaxAnchorGapMeters;
            bool held=InBand(n.GoalFoot) && InBand(n.GoalCom);
            for (int i=0;i<Count;i++)
            {
                ref var lane=ref _lanes[i];
                if (!lane.Live) continue;
                var shifted=delta+lane.Offset;
                lane.Read.Gap=shifted.magnitude; lane.Read.Xz=Flat(shifted).magnitude;
                if (lane.Read.Gap>=_hardGap) { Stop(ref lane,i,Mark.HardGap); continue; }
                if (!Inside(actual.Box,actual.Box.FootAnkle+lane.Offset,actual.Box.FootR)) { Stop(ref lane,i,Mark.Outside); continue; }
                _measured[i]++; lane.Read.Flags|=Mark.Measured;
                if (lane.Read.Gap>FootContactGeometryLimits.RecoveryMaxAnchorGapMeters) continue;
                lane.Read.Flags|=Mark.GapPass;
                if (InBand(n.Box)) lane.Read.Flags|=Mark.SoleAndGap;
                if (held) lane.Read.Flags|=Mark.HeldAndGap;
                if (lane.Read.TorqueGain<=0f || lane.Read.Gap+lane.Read.TorqueGapBound>FootContactGeometryLimits.RecoveryMaxAnchorGapMeters) continue;
                lane.Read.Flags|=Mark.TorqueGain;
                // Require agreement between the model's absolute height and its gain
                // relative to the next measured box. Never credit a naturally good foot.
                if (RightFootTorqueForecast.Band(n.Box)==Band.Below && InBand(lane.TrialHeight) && InBand(n.Box+lane.Read.TorqueGain))
                    lane.Read.Flags|=Mark.TorqueCross;
            }
            n.Quarter=_lanes[0].Read; n.Half=_lanes[1].Read; n.Full=_lanes[2].Read;
            return n;
        }
        internal void Append(StringBuilder b)
        {
            b.Append("\nRIGHT_ANKLE_SEQUENCE_V041530/qualified-segments-paired-multi-broken=").Append(_qualified).Append('|')
                .Append(_segments).Append('|').Append(_paired).Append('|').Append(_multi).Append('|').Append(_broken)
                .Append("/missing-commandOrCap-saturatedOrAboveBase=").Append(_missing).Append('|').Append(_saturated)
                .Append("/lower=OBSERVED/terminalTorqueSteps=1/feedbackToControl=False");
            for (int i=0;i<Count;i++) b.Append("/gain").Append(Fraction(i).ToString(CultureInfo.InvariantCulture))
                .Append("(issued-measured-clipped-outside-hardGap)=").Append(_issued[i]).Append('|').Append(_measured[i])
                .Append('|').Append(_clipped[i]).Append('|').Append(_outside[i]).Append('|').Append(_hard[i]);
        }
    }
}
