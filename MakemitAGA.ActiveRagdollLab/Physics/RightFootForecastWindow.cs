using System;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;
using Forecast = MakemitAGA.ActiveRagdollLab.Physics.RightFootTorqueForecast;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed class RightFootForecastWindow
    {
        internal const double Horizon = .20;
        internal const int Capacity = 256;
        private const double TimeTolerance = 1e-6;
        internal struct Request
        {
            internal PhysicsStepContext Step;
            internal SourceStepContext Source;
            internal int Local, Support, Body;
            internal RightKneePivot.Frame Pose;
            internal Vector3 FootW;
        }
        internal enum Interruption : byte { TraceStop, ActiveAtEntry, Fault, InvalidSample, ClockOrSurface, BodyOrShape }
        private enum End : byte { Unavailable, Awaiting, Measured, Interrupted }
        private struct Row
        {
            internal int Epoch, Local, Shared, Source, Support, First, Latest, NextSource;
            internal Phase Phase, NextPhase;
            internal float Dt, Age, Gap, Floor, Box, Goal, NextGoal, NextLower, NextFoot, NextBox;
            internal Vector3 Reanchored, OrientationError, VelocityError;
            internal Forecast.Result Forecast;
            internal Quaternion Rotation, NextRotation;
            internal Vector3 Center, Half, NextW;
            internal float FootY;
            internal bool NextStrong, NextActive;
            internal End End;
            internal Interruption InterruptedBy;
        }
        private struct Errors
        {
            internal double X, Y, Z;
            internal void Add(float a, float b, float c) { X += Math.Abs(a); Y += Math.Abs(b); Z += Math.Abs(c); }
            internal string Means(int n, double factor) => n == 0 ? "NA" : N(X*factor/n)+"|"+N(Y*factor/n)+"|"+N(Z*factor/n);
        }

        private readonly Row[] _rows = new Row[Capacity];
        private readonly int[] _states = new int[(int)Forecast.Status.Fault+1];
        private Request _previous;
        private Row _row;
        private int _first, _latest, _seen, _origins, _episodes, _total, _saved, _predicted, _measured, _interrupted, _overGap;
        private double _elapsed;
        private bool _waiting, _pending;
        private Errors _box, _anchors, _rotationOnly, _angles, _velocities;
        internal bool Active => _first != 0;
        internal Request Current => _previous;
        internal bool Requested(in PhysicsStepContext step) => _waiting && _previous.Step.Matches(step);

        private static bool Qualified(in Sample s) =>
            s.EarlyFoot == EarlyLandingWindow.Foot.Right && s.ClockValid && !s.WasActive && s.HeightValid &&
            s.Identity && s.FivePoint && s.FullFootprint && !s.SelectedBoxFallback && s.AnkleHeightsValid &&
            PelvisParentFrame.V(s.AnkleHeights) && PelvisParentFrame.F(s.SourceAge) && s.SourceAge >= 0f &&
            s.SourceAge <= FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds;
        private static float Goal(in Sample s) => s.TrackValid && s.BoxReferencesValid ? s.ControlAtActualAnkle : float.NaN;
        internal void Cut(Interruption reason = Interruption.TraceStop)
        {
            if (_waiting) Complete(default);
            if (_pending) { _interrupted++; _row.End = End.Interrupted; _row.InterruptedBy = reason; Save(); }
            _first = _latest = 0; _elapsed = 0;
        }
        private void Save()
        {
            if (_saved < Capacity) _rows[_saved++] = _row;
            _pending = false;
        }
        internal void Complete(in Forecast.Result result)
        {
            if (!_waiting) return;
            _waiting = false; _states[(int)result.State]++; _row.Forecast = result;
            if (result.State == Forecast.Status.Predicted) { _predicted++; _pending = true; _row.End = End.Awaiting; }
            else Save();
        }
        internal void Observe(in Sample s, in RightKneePivot.Frame p, int body, Vector3 w)
        {
            bool onset = s.RightThigh == RightThighWindow.Signal.Onset;
            if (!Active && !onset) return;
            if (onset) _seen++;
            bool usable = body != 0 && Qualified(s) && Forecast.ValidGeometry(p) && PelvisParentFrame.V(w) &&
                RightKneePivot.Near(p.Box, s.Min) && RightKneePivot.Near(p.FootAnkle.y, s.AnkleHeights.z) &&
                RightKneePivot.Near(p.LowerAnkle.y, s.AnkleHeights.y) &&
                (p.FloorCenter-s.FloorCenter).sqrMagnitude <= 1e-12f && (p.FloorHalf-s.FloorHalf).sqrMagnitude <= 1e-12f;
            if (Active)
            {
                bool forward = s.Step.Epoch != _previous.Step.Epoch ||
                    (s.Fixed > _previous.Local && s.Step.Sequence > _previous.Step.Sequence &&
                    s.Source.Sequence >= _previous.Source.Sequence && s.Source.Time >= _previous.Source.Time &&
                    (s.Source.Sequence != _previous.Source.Sequence || s.Source.Matches(_previous.Source)) &&
                    s.Step.FixedTime >= _previous.Step.FixedTime && s.Step.UnscaledTime >= _previous.Step.UnscaledTime);
                bool adjacent = usable && body == _previous.Body &&
                    RightAnkleWindow.Adjacent(_previous.Local, _previous.Step, _previous.Source, _previous.Support,
                        _previous.Pose.FloorCenter, _previous.Pose.FloorHalf, s) &&
                    (p.Center-_previous.Pose.Center).sqrMagnitude <= 1e-12f && (p.Half-_previous.Pose.Half).sqrMagnitude <= 1e-12f;
                if (!adjacent)
                {
                    var reason = !usable ? Interruption.InvalidSample :
                        body != _previous.Body || (p.Center-_previous.Pose.Center).sqrMagnitude > 1e-12f ||
                        (p.Half-_previous.Pose.Half).sqrMagnitude > 1e-12f ? Interruption.BodyOrShape : Interruption.ClockOrSurface;
                    Cut(reason);
                    if (!forward) return;
                }
                else
                {
                    if (_waiting) Complete(default);
                    if (_pending) Read(s, p, w);
                }
            }
            bool openPhase = s.Gait == Phase.Swing || s.Gait == Phase.Approach;
            if (!usable || !openPhase || s.NowActive || s.StrongLanding || s.FirstMiss || s.MissInProgress ||
                !PelvisParentFrame.F(s.AnchorGap) || s.AnchorGap < 0f) { Cut(); return; }
            if (onset)
            {
                _origins++;
                if (!Active) { _episodes++; _first = s.Fixed; }
                _latest = s.Fixed; _elapsed = 0;
            }
            if (!Active || _elapsed + s.Step.Dt > Horizon + TimeTolerance) { Cut(); return; }
            _elapsed += s.Step.Dt;
            _previous = new Request { Step = s.Step, Source = s.Source, Local = s.Fixed, Support = s.SupportId,
                Body = body, Pose = p, FootW = w };
            _row = new Row { Epoch = s.Step.Epoch, Local = s.Fixed, Shared = s.Step.Sequence, Source = s.Source.Sequence,
                Support = s.SupportId, First = _first, Latest = _latest, Phase = s.Gait, Dt = s.Step.Dt, Age = s.SourceAge,
                Center = p.Center, Half = p.Half, Rotation = p.FootR, FootY = p.FootAnkle.y,
                Gap = s.AnchorGap, Floor = p.FloorCenter.y + p.FloorHalf.y, Box = s.Min, Goal = Goal(s) };
            _total++; _waiting = true;
            if (s.AnchorGap > FootContactGeometryLimits.RecoveryMaxAnchorGapMeters) _overGap++;
        }
        private static float Degrees(Quaternion actual, Quaternion forecast) =>
            FootRelativeReference.Log(actual * Quaternion.Inverse(forecast)).magnitude * Mathf.Rad2Deg;
        private void Read(in Sample s, in RightKneePivot.Frame p, Vector3 w)
        {
            var f = _row.Forecast;
            float floor = p.FloorCenter.y + p.FloorHalf.y;
            _row.NextLower = p.LowerAnkle.y - floor; _row.NextFoot = p.FootAnkle.y - floor;
            _row.NextRotation = p.FootR; _row.NextW = w; _row.NextStrong = s.StrongLanding; _row.NextActive = s.NowActive;
            _row.NextBox = s.Min; _row.NextGoal = Goal(s); _row.NextPhase = s.Gait; _row.NextSource = s.Source.Sequence;
            _row.Reanchored = new Vector3(Forecast.Clearance(p, p.FootAnkle, f.Coast.R),
                Forecast.Clearance(p, p.FootAnkle, f.Free.R), Forecast.Clearance(p, p.FootAnkle, f.Issued.R));
            _row.OrientationError = new Vector3(Degrees(p.FootR, f.Coast.R), Degrees(p.FootR, f.Free.R), Degrees(p.FootR, f.Issued.R));
            _row.VelocityError = new Vector3((w-f.Coast.W).magnitude, (w-f.Free.W).magnitude, (w-f.Issued.W).magnitude);
            _box.Add(f.Coast.Box-s.Min, f.Free.Box-s.Min, f.Issued.Box-s.Min);
            _rotationOnly.Add(_row.Reanchored.x-s.Min, _row.Reanchored.y-s.Min, _row.Reanchored.z-s.Min);
            _anchors.Add(f.LowerY-p.LowerAnkle.y, f.PredictedFoot.y-p.FootAnkle.y,
                (f.PredictedFoot.y-f.LowerY)-(p.FootAnkle.y-p.LowerAnkle.y));
            _angles.Add(_row.OrientationError.x, _row.OrientationError.y, _row.OrientationError.z);
            _velocities.Add(_row.VelocityError.x, _row.VelocityError.y, _row.VelocityError.z);
            _row.End = End.Measured; _measured++; Save();
        }
        private static string N(double v) => double.IsFinite(v) ? v.ToString("G7", CultureInfo.InvariantCulture) : "NA";
        private static string M(float v) => N(1000d*v);
        private static string V(Vector3 v, float scale = 1f) => N(v.x*scale)+"|"+N(v.y*scale)+"|"+N(v.z*scale);
        private static string Q(Quaternion q) => N(q.x)+"|"+N(q.y)+"|"+N(q.z)+"|"+N(q.w);
        internal string Summary()
        {
            Cut();
            var b = new StringBuilder();
            b.Append("\nRIGHT_FOOT_TORQUE_V041510/origins-episodes=").Append(_origins).Append('|').Append(_episodes)
                .Append("/steps-predicted-measured-interrupted=").Append(_total).Append('|').Append(_predicted).Append('|').Append(_measured).Append('|').Append(_interrupted)
                .Append("/onsetsSeen-ineligible=").Append(_seen).Append('|').Append(_seen-_origins)
                .Append("/saved-omitted=").Append(_saved).Append('|').Append(_total-_saved).Append("/over5mm=").Append(_overGap);
            for (int i=0;i<_states.Length;i++) b.Append('/').Append((Forecast.Status)i).Append('=').Append(_states[i]);
            b.Append("/lanes=coast|freeDamped|issuedDamped/boxMaeMm=").Append(_box.Means(_measured,1000))
                .Append("/fixedActualAnkleBoxMaeMm=").Append(_rotationOnly.Means(_measured,1000))
                .Append("/lower-foot-gapMaeMm=").Append(_anchors.Means(_measured,1000))
                .Append("/rotationMaeDeg=").Append(_angles.Means(_measured,1)).Append("/wErrorRadps=").Append(_velocities.Means(_measured,1));
            for (int i=0;i<_saved;i++) Append(b, _rows[i]);
            return b.ToString();
        }
        private static void Append(StringBuilder b, in Row r)
        {
            b.Append("\nRIGHT_FOOT_TORQUE_STEP_V041510/epoch-support=").Append(r.Epoch).Append('|').Append(r.Support)
                .Append("/first-latestOrigin=").Append(r.First).Append('|').Append(r.Latest)
                .Append("/local-shared-source=").Append(r.Local).Append('|').Append(r.Shared).Append('|').Append(r.Source)
                .Append("/phase=").Append(r.Phase).Append("/dt-ageMs=").Append(M(r.Dt)).Append('|').Append(M(r.Age))
                .Append("/gap-actual-goalMm=").Append(M(r.Gap)).Append('|').Append(M(r.Box)).Append('|').Append(M(r.Goal))
                .Append("/status=").Append(r.Forecast.State).Append("/next=").Append(r.End);
            if (r.End == End.Interrupted) b.Append("/interruptedBy=").Append(r.InterruptedBy);
            var f = r.Forecast;
            if (f.State != Forecast.Status.Predicted) return;
            var motion = f.Motion;
            b.Append("/footR-w=").Append(Q(r.Rotation)).Append(',').Append(V(motion.W))
                .Append("/principalR-inertia=").Append(Q(motion.WorldPrincipal)).Append(',').Append(V(motion.Inertia))
                .Append("/drag-maxW=").Append(N(motion.Drag)).Append('|').Append(N(motion.MaxW))
                .Append("/footYmm-centerHalfMm=").Append(M(r.FootY-r.Floor)).Append(',').Append(V(r.Center,1000)).Append(',').Append(V(r.Half,1000))
                .Append("/transport-drive-supportNmXYZ=").Append(V(motion.Transport)).Append(',').Append(V(motion.Drive)).Append(',').Append(V(motion.Support))
                .Append("/forecastLower-footMm=").Append(M(f.LowerY-r.Floor)).Append('|').Append(M(f.PredictedFoot.y-r.Floor))
                .Append("/boxMm=").Append(M(f.Coast.Box)).Append('|').Append(M(f.Free.Box)).Append('|').Append(M(f.Issued.Box))
                .Append("/inside=").Append(f.Coast.Inside?1:0).Append('|').Append(f.Free.Inside?1:0).Append('|').Append(f.Issued.Inside?1:0);
            if (r.End != End.Measured) return;
            b.Append("/nextSource-phase=").Append(r.NextSource).Append('|').Append(r.NextPhase)
                .Append("/nextLower-foot-box-goalMm=").Append(M(r.NextLower)).Append('|').Append(M(r.NextFoot)).Append('|').Append(M(r.NextBox)).Append('|').Append(M(r.NextGoal))
                .Append("/nextStrong-active=").Append(r.NextStrong?1:0).Append('|').Append(r.NextActive?1:0)
                .Append("/nextR-w=").Append(Q(r.NextRotation)).Append(',').Append(V(r.NextW))
                .Append("/fixedActualAnkleBoxMm=").Append(V(r.Reanchored,1000f))
                .Append("/angleErrorDeg=").Append(V(r.OrientationError)).Append("/wErrorRadps=").Append(V(r.VelocityError));
        }
    }
}
