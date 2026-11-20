using System;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;
using Forecast = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed class RightFootLandingWindow
    {
        internal enum End : byte { Read, TraceStop, Active, Fault, Clock, Surface, BodyOrBox, Geometry }
        internal struct Readback
        {
            internal int From;
            internal Forecast.Status State;
            internal float Baseline, Trial, Actual, Gap;
            internal RightAnkleLandingSequence.Readback Preparation;
        }
        private struct Row
        {
            internal PhysicsStepContext Step;
            internal SourceStepContext Source;
            internal int Local, Support, FootBody, LowerBody;
            internal Audit.Phase Gait;
            internal RightThighWindow.Signal Thigh;
            internal Forecast.Pose Pose;
            internal Forecast.Motion Motion;
            internal Forecast.Result Result;
            internal Readback Next;
            internal Vector3 NextCom, NextFoot;
            internal Forecast.Pair Pair;
            internal float SourceAge;
            internal End End;
        }
        private const int PerState = 12;
        private const int States = 1 + (int)Forecast.Status.Favorable;
        private readonly Row[] _details = new Row[PerState*States];
        private readonly int[] _states = new int[States], _stored = new int[States];
        private readonly int[] _ends = new int[1+(int)End.Geometry];
        private readonly RightAnkleLandingSequence _sequence = new RightAnkleLandingSequence();
        internal bool CouldUseSequencePair(in Forecast.Pose p) => _sequence.CouldUsePair(p);
        private Row _row;
        private PhysicsStepContext _lastStep;
        private SourceStepContext _lastSource;
        private int _lastLocal, _screened, _skippedGeometry, _skippedClock, _started, _predicted, _measured;
        private int _trialEnters, _readEnters, _wrongBand, _gainAboveError, _favorableRead, _nextOverGap;
        private double _boxError, _comError, _footError, _maximumError;
        private bool _awaitCommand, _awaitRead;
        internal bool Pending => _awaitCommand || _awaitRead;
        internal Forecast.Pose Pose => _row.Pose;
        internal int Local => _row.Local;
        internal SourceStepContext Source => _row.Source;
        internal bool Bodies(int foot, int lower) => foot != 0 && lower != 0 && foot == _row.FootBody && lower == _row.LowerBody;
        internal bool Requested(in PhysicsStepContext step) => _awaitCommand && _row.Step.Matches(step);

        internal static bool BeforeContact(in Sample s) => !s.WasActive && !s.NowActive && !s.StrongLanding &&
            !s.FirstMiss && !s.MissInProgress && !s.OrdinaryEnabled && !s.RecoveryEnabled &&
            (s.Gait == Audit.Phase.Swing || s.Gait == Audit.Phase.Approach ||
             (s.Gait == Audit.Phase.Stance && s.SourceMotion != Audit.SourceKind.Idle));
        internal static bool Physical(in Sample s) => s.EarlyFoot == EarlyLandingWindow.Foot.Right &&
            s.ClockValid && !s.WasActive && s.HeightValid && s.AnkleHeightsValid && s.Identity && s.FivePoint &&
            s.FullFootprint && !s.SelectedBoxFallback && PelvisParentFrame.F(s.SourceAge) && s.SourceAge >= 0f &&
            s.SourceAge <= FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds;

        internal void Cut(End why)
        {
            _sequence.Cut();
            if (_awaitCommand) Complete(new Forecast.Result { State=why==End.Fault ? Forecast.Status.Fault : Forecast.Status.MissingController }, default);
            if (_awaitRead)
            {
                _ends[(int)why]++; _row.End=why; Save(); _awaitRead=false;
            }
        }
        // PRE_N+1 first closes the issued prediction even when this sample is the strong
        // edge or enables contact. No trial can be started by reading its own outcome.
        internal Readback Observe(in Sample s, in Forecast.Pose p, int footBody, int lowerBody, bool nativePose)
        {
            bool wants = BeforeContact(s);
            if (wants) _screened++;
            bool forward = s.ClockValid && (!_lastStep.Valid || s.Step.Epoch > _lastStep.Epoch ||
                (s.Step.Epoch == _lastStep.Epoch && s.Fixed > _lastLocal && s.Step.Sequence > _lastStep.Sequence &&
                 s.Step.FixedTime >= _lastStep.FixedTime && s.Step.UnscaledTime >= _lastStep.UnscaledTime &&
                 s.Source.Sequence >= _lastSource.Sequence && s.Source.Time >= _lastSource.Time &&
                 (s.Source.Sequence != _lastSource.Sequence || s.Source.Matches(_lastSource))));
            if (!forward)
            {
                if (wants) _skippedClock++;
                Cut(End.Clock);
                return default;
            }
            _lastLocal=s.Fixed; _lastStep=s.Step; _lastSource=s.Source;
            bool valid = nativePose && footBody!=0 && lowerBody!=0 && footBody!=lowerBody && Physical(s) && p.Box.Dt==s.Step.Dt &&
                RightFootTorqueForecast.ValidGeometry(p.Box) && HipPointPairMobility.V(p.Com) &&
                RightKneePivot.Near(p.Box.Box,s.Min) && RightKneePivot.Near(p.Box.LowerAnkle.y,s.AnkleHeights.y) &&
                RightKneePivot.Near(p.Box.FootAnkle.y,s.AnkleHeights.z) && RightKneePivot.Near(p.Gap,(p.Box.FootAnkle-p.Box.LowerAnkle).magnitude);
            Readback next = default;
            if (_awaitCommand) Complete(default, default);
            if (_awaitRead)
            {
                if (s.Fixed != _row.Local+1 || s.Step.Sequence != _row.Step.Sequence+1 || s.Step.Epoch != _row.Step.Epoch) Cut(End.Clock);
                else if (!RightAnkleWindow.Adjacent(_row.Local,_row.Step,_row.Source,_row.Support,_row.Pose.Box.FloorCenter,_row.Pose.Box.FloorHalf,s)) Cut(End.Surface);
                else if (!valid) Cut(End.Geometry);
                else if (!Bodies(footBody,lowerBody) || !RightThighTracking.Same(p.Box.Center,_row.Pose.Box.Center) ||
                    !RightThighTracking.Same(p.Box.Half,_row.Pose.Box.Half) ||
                    !RightThighTracking.Same(Quaternion.Inverse(p.Box.FootR)*(p.Box.FootAnkle-p.Com),
                        Quaternion.Inverse(_row.Pose.Box.FootR)*(_row.Pose.Box.FootAnkle-_row.Pose.Com))) Cut(End.BodyOrBox);
                else
                {
                    if (LandingWindowOutcomes.StartsNewSwing(s.Gait,_row.Gait)) _sequence.Cut();
                    next=Read(p);
                }
            }
            if (wants)
            {
                if (!valid || !s.TrackValid || !s.BoxReferencesValid || !Forecast.Valid(p)) _skippedGeometry++;
                else
                {
                    _started++; _awaitCommand=true;
                    _row = new Row { Step=s.Step, Source=s.Source, Local=s.Fixed, Support=s.SupportId,
                        Gait=s.Gait, Thigh=s.RightThigh, SourceAge=s.SourceAge, FootBody=footBody, LowerBody=lowerBody, Pose=p };
                }
            }
            if (!_awaitCommand) _sequence.Cut();
            return next;
        }
        internal void Complete(in Forecast.Result result, in Forecast.Motion motion, in Forecast.Pair pair = default)
        {
            if (!_awaitCommand) return;
            _awaitCommand=false; _row.Result=result; _row.Motion=motion; _row.Pair=pair;
            if (result.Predicted && result.State!=Forecast.Status.OutsideFootprint)
                _sequence.Issue(_row.Local,_row.Pose,motion,result,pair,motion.Tether);
            else _sequence.Cut();
            if (result.EntersBand) _trialEnters++;
            _states[(int)result.State]++;
            if (result.Predicted) { _predicted++; _awaitRead=true; }
            else Save();
        }
        private Readback Read(in Forecast.Pose actual)
        {
            var r=_row.Result;
            var n=new Readback { From=_row.Local, State=r.State, Baseline=r.Box, Trial=r.TrialBox, Actual=actual.Box.Box, Gap=actual.Gap };
            n.Preparation=_sequence.Read(actual,r);
            if (n.Preparation.Depth>0)
            {
                n.Preparation.FromShared=_row.Step.Sequence;
                n.Preparation.FromSource=_row.Source.Sequence;
            }
            double err=Math.Abs(n.Actual-n.Baseline);
            _boxError+=err; _maximumError=Math.Max(_maximumError,err);
            _comError+=(r.Com-actual.Com).magnitude; _footError+=(r.Foot-actual.Box.FootAnkle).magnitude;
            if (RightFootTorqueForecast.Band(n.Actual)!=RightFootTorqueForecast.Band(n.Baseline)) _wrongBand++;
            if (actual.Gap>FootContactGeometryLimits.RecoveryMaxAnchorGapMeters) _nextOverGap++;
            if (r.State==Forecast.Status.Favorable) { _favorableRead++; if (r.TrialBox-r.Box > err) _gainAboveError++; }
            if (r.EntersBand) _readEnters++;
            _row.NextCom=actual.Com; _row.NextFoot=actual.Box.FootAnkle;
            _measured++; _ends[(int)End.Read]++; _row.End=End.Read; _row.Next=n; Save(); _awaitRead=false;
            return n;
        }
        private void Save()
        {
            int k=(int)_row.Result.State;
            if (_stored[k]<PerState) _details[k*PerState+_stored[k]++]=_row;
        }
        private static string N(double v) => double.IsFinite(v) ? v.ToString("G8",CultureInfo.InvariantCulture) : "NA";
        private static string Mm(float v) => N(1000d*v);
        private static string V(Vector3 v) => N(v.x)+"|"+N(v.y)+"|"+N(v.z);
        private static string Q(Quaternion q) => N(q.x)+"|"+N(q.y)+"|"+N(q.z)+"|"+N(q.w);
        internal string Summary()
        {
            Cut(End.TraceStop);
            var b=new StringBuilder();
            b.Append("\nRIGHT_FOOT_LANDING_V041520/screened-started-clockRejected-geometryRejected=").Append(_screened).Append('|').Append(_started)
                .Append('|').Append(_skippedClock).Append('|').Append(_skippedGeometry)
                .Append("/predicted-read=").Append(_predicted).Append('|').Append(_measured)
                .Append("/boxMae-max-com3dMae-foot3dMaeMm=").Append(_measured==0?"NA|NA|NA|NA":
                    N(1000*_boxError/_measured)+"|"+N(1000*_maximumError)+"|"+N(1000*_comError/_measured)+"|"+N(1000*_footError/_measured))
                .Append("/bandMismatch-nextOver5=").Append(_wrongBand).Append('|').Append(_nextOverGap)
                .Append("/trialRead-gainAboveBaselineError=").Append(_favorableRead).Append('|').Append(_gainAboveError)
                .Append("/trialEntersBand-all-read=").Append(_trialEnters).Append('|').Append(_readEnters)
                .Append("/detailPerState=").Append(PerState).Append("/feedbackToControl=False");
            for(int i=0;i<States;i++) b.Append('/').Append((Forecast.Status)i).Append('=').Append(_states[i]).Append('|').Append(_stored[i]);
            for(int i=0;i<_ends.Length;i++) b.Append("/end").Append((End)i).Append('=').Append(_ends[i]);
            for(int i=0;i<States;i++) for(int j=0;j<_stored[i];j++) Append(b,_details[i*PerState+j]);
            _sequence.Append(b);
            return b.ToString();
        }
        private static void Append(StringBuilder b, in Row row)
        {
            var p=row.Pose; var r=row.Result;
            b.Append("\nRIGHT_FOOT_LANDING_STEP_V041520/epoch-support=").Append(row.Step.Epoch).Append('|').Append(row.Support)
                .Append("/local-shared-source=").Append(row.Local).Append('|').Append(row.Step.Sequence).Append('|').Append(row.Source.Sequence)
                .Append("/phase-thigh=").Append(row.Gait).Append('|').Append(row.Thigh).Append("/state=").Append(r.State);
            if (!r.Predicted) return;
            var m=row.Motion;
            b.Append("/ageMs=").Append(Mm(row.SourceAge))
                .Append("/footN-lowerN=").Append(V(p.Box.FootAnkle)).Append(',').Append(V(p.Box.LowerAnkle))
                .Append("/center-half-floorCenter-floorHalf=").Append(V(p.Box.Center)).Append(',').Append(V(p.Box.Half)).Append(',').Append(V(p.Box.FloorCenter)).Append(',').Append(V(p.Box.FloorHalf))
                .Append("/w-principal-inertia=").Append(V(p.W)).Append(',').Append(Q(m.Angular.WorldPrincipal)).Append(',').Append(V(m.Angular.Inertia))
                .Append("/maxW-tauTransport-drive-support=").Append(N(m.Angular.MaxW)).Append(',').Append(V(m.Angular.Transport)).Append(',').Append(V(m.Angular.Drive)).Append(',').Append(V(m.Angular.Support))
                .Append("/predictedCom-foot=").Append(V(r.Com)).Append(',').Append(V(r.Foot))
                .Append("/dt=").Append(N(row.Step.Dt)).Append("/com-v-issuedLinearA=").Append(V(p.Com)).Append(',').Append(V(p.Velocity)).Append(',').Append(V(m.AppliedLinearA))
                .Append("/drag-linear-angular=").Append(N(m.LinearDrag)).Append('|').Append(N(m.Angular.Drag))
                .Append("/footR-goalR=").Append(Q(p.Box.FootR)).Append(',').Append(Q(p.Goal))
                .Append("/gap-present-heldMm=").Append(Mm(p.Gap)).Append('|').Append(Mm(p.Box.Box)).Append('|').Append(Mm(p.PresentGoal))
                .Append("/base-goalAtFoot-goalAtCom-trialMm=").Append(Mm(r.Box)).Append('|').Append(Mm(r.GoalAtFoot)).Append('|').Append(Mm(r.GoalAtCom)).Append('|').Append(Mm(r.TrialBox))
                .Append("/level-relativeIncrement=").Append(r.Level).Append('|').Append(N(r.RelativeIncrement))
                .Append("/ankleDeltaNm=").Append(V(r.DeltaTorque)).Append("/extraGapBoundMm=").Append(Mm(r.ExtraGapBound))
                .Append("/tried-reject-bestSafeGainMm=").Append(r.Tried).Append('|').Append(r.Rejected).Append('|').Append(Mm(r.BestSafeGain))
                .Append("/next=").Append(row.End);
            if(r.Tried>0) b.Append("/lowerPrincipal-inertia-realizedRelativeA=").Append(Q(row.Pair.LowerPrincipal)).Append(',').Append(V(row.Pair.LowerInertia))
                .Append(',').Append(V(row.Pair.RealizedRelativeA)).Append("/lowerLever=").Append(N(row.Pair.LowerLever));
            if (row.End==End.Read) b.Append("/realCom-foot=").Append(V(row.NextCom)).Append(',').Append(V(row.NextFoot)).Append("/realBox-realGapMm=").Append(Mm(row.Next.Actual)).Append('|').Append(Mm(row.Next.Gap));
        }
    }
}
