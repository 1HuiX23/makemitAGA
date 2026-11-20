using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;
using Probe = MakemitAGA.ActiveRagdollLab.Physics.RightThighTracking;
using Signal = MakemitAGA.ActiveRagdollLab.Physics.RightThighWindow.Signal;

namespace ARL.Contracts
{
    internal static class RightThighTrackingContracts
    {
        internal static void Run()
        {
            Check.Run("v150 thigh and ankle must be low on two consecutive qualified physics steps", Gate);
            Check.Run("v150 replays both submitted hips, charges the joint actuator and chooses the smallest joint gain", Allocation);
            Check.Run("v150 relative supplements preserve gyro and parent transport at nonzero angular velocity", Ownership);
            Check.Run("v150 rotation, hip torque and the distal joint may produce opposite height changes", Tradeoffs);
            Check.Run("v150 original readback seals across changes in surface, clock or contact and counts overflow", Readbacks);
            Check.Run("v150 ordinary and missed landings retain pre-edge origins only; inactive A is silent", Events);
            Check.Run("v150 observation has no per-step allocations after setup", Cost);
        }
        private static RightKneePivot.Frame Pose(bool common = true)
        {
            var p = RightKneePivotContracts.Pose(.0003f, 12f, .0004f, -.012f);
            if (common) { p.LowerAnkle.z -= .5f; p.FootAnkle.z -= .5f; }
            return p;
        }
        private static Audit.Sample Sample(int n, in RightKneePivot.Frame p)
        {
            var s = SwingKneeContracts.Sample(n, p.Dt, n * p.Dt);
            s.EarlyFoot = EarlyLandingWindow.Foot.Right; s.SourcePhase = "STEADY"; s.SwingMeasured = false; s.ParentHeightMeasured = true;
            s.Min = p.Box; s.FloorCenter = p.FloorCenter; s.FloorHalf = p.FloorHalf; s.AnchorGap = .008f;
            s.KneeHeights = new Vector3(p.GoalKneeY, p.GoalKneeY, p.LowerKnee.y);
            s.AnkleHeights = new Vector3(p.RawAnkleY, p.LowerAnkle.y, p.FootAnkle.y);
            return s;
        }
        private static Probe.Motion Motion() => new Probe.Motion { HipMaxA = 40, ReferenceMaxA = 28, ReferenceMaxW = 5,
            UpperMaxW = 7, LowerMaxW = 7, FootMaxW = 7, LowerW = Vector3.zero, FootW = Vector3.zero,
            KneeRelativeA = Vector3.zero, AnkleGapV = Vector3.zero, KneeGapV = Vector3.zero };
        private static void Pair(in RightKneePivot.Frame p, out HipAnchorPairInput l, out HipAnchorPairInput r,
            out HipAnchorPairReport.Command lc, out HipAnchorPairReport.Command rc, float torque = 0, float inertia = .07f)
        {
            var step = PelvisDynamicReceiverIsolation.LoggedStep(); step.Dt = p.Dt;
            var baseline = PelvisDynamicReceiverIsolation.Baseline(step);
            baseline.Acceleration = baseline.ReceiverTorque = Vector3.zero;
            baseline.UpwardRetentionSelected = baseline.BoundedTradeSelected = true;
            var principal = Quaternion.AngleAxis(13f, Vector3.forward);
            var tensor = new Vector3(inertia, inertia*.9f, inertia*1.1f);
            l = HipAllocationContracts.Input(baseline, true, Vector3.zero, Vector3.zero, Vector3.zero, Vector3.right * -.10f);
            r = HipAllocationContracts.Input(baseline, false, torque * Vector3.right, Vector3.zero, Vector3.zero, Vector3.right * .10f, 1f, tensor, principal);
            l.Seed.ChildPoint += p.UpperHip; l.Seed.ChildCom += p.UpperHip; l.Seed.ParentPoint += p.UpperHip; l.Seed.ParentCom += p.UpperHip;
            r.Seed.ChildPoint += p.UpperHip; r.Seed.ChildCom += p.UpperHip; r.Seed.ParentPoint += p.UpperHip; r.Seed.ParentCom += p.UpperHip;
            Check.That(HipBilateralForceSolver.TryPlan(l, r, out lc, out rc), "real baseline solved");
        }
        private static void Gate()
        {
            var p = Pose(); var a = Sample(100,p); var b = Sample(101,p);
            var w = new RightThighWindow();
            // A repeated render sample on distinct physics steps is valid, including when the observational gap is 8 mm.
            b.Source = a.Source; b.Step.SourceSequence = a.Source.Sequence;
            Check.That(w.Observe(a,p) == Signal.First && w.Observe(b,p) == Signal.Onset && w.Requested(b.Step), "held source and same face");
            var foreign = b.Step; foreign.Epoch++;
            Check.That(!w.Requested(foreign) && w.Current.Local == 101, "wrong-step owner cannot claim request");
            w.Complete(default); Check.That(!w.Requested(b.Step), "single completion");
            Check.That((Audit.Screen(b) & Audit.Rejection.Gap) != 0, "physical ankle gate still rejects this gap");
            for (int i = 0; i < 13; i++)
            {
                w = new RightThighWindow(); a = Sample(4,p); b = Sample(5,p); var q = p;
                switch (i)
                {
                    case 0: b.Fixed++; break;
                    case 1: b.Step.Sequence++; break;
                    case 2: b.Step.Epoch++; b.Source.Epoch++; break;
                    case 3: b.FloorCenter.y += .02f; q.FloorCenter.y += .02f; break;
                    case 4: b.SupportId++; break;
                    case 5: b.Source = a.Source; b.Step.SourceSequence = b.Source.Sequence; b.Source.Dt *= 2; break;
                    case 6: b.AnkleHeights.x = b.AnkleHeights.y + .00197f; break;
                    case 7: b.KneeHeights.y = b.KneeHeights.z + .0005f; break;
                    case 8: q = RightKneePivotContracts.Pose(.03f, 2f, .001f); b = Sample(5,q); break;
                    case 9: b.SourcePhase = "TURN"; break;
                    case 10: b.NowActive = true; break;
                    case 11: b.FivePoint = false; break;
                    case 12: b.Fixed = a.Fixed; b.Step = a.Step; b.Source = a.Source; break;
                }
                w.Observe(a,p);
                Check.That(w.Observe(b,q) != Signal.Onset && !w.Requested(b.Step), "broken pair " + i);
            }
        }
        private static void Allocation()
        {
            foreach (float hz in new[] { 30f, 60f, 120f })
            {
                var p = Pose(); p.Dt = 1f/hz;
                Pair(p, out var l, out var r, out var lc, out var rc);
                var m = Motion(); var alt = Probe.Compare(p,m,l,r,lc,rc);
                Check.That(alt.State == Probe.Status.Improved, "same state gives three simultaneous gains at " + hz + ": " + alt.State);
                Check.That(alt.Gain.x >= .0001f && alt.Gain.y >= .0001f && alt.Gain.z >= .0001f && alt.Trial.Angle < alt.Baseline.Angle, "all signed gains and upper closes");
                Check.That(alt.Tried <= 3 && (hz != 60f || alt.Fraction == .25f), "first qualifying fraction");
                Check.That(alt.TrialRequestA <= m.HipMaxA && alt.TrialAllocatedA <= m.HipMaxA && alt.TrialNextW <= alt.MaxSpeed &&
                    alt.TrialTorque <= rc.UpperActuatorLimitNm && alt.ReceiverReactionDelta > 0f, "actual envelope and shared receiver");
                Check.Near(alt.Baseline.Heights.x, p.LowerKnee.y, .000002f, "zero baseline knee");
                Check.Near(alt.Baseline.Heights.y, p.LowerAnkle.y, .000002f, "zero baseline ankle");
                Check.Near(alt.Baseline.Heights.z, p.Box, .000002f, "zero baseline eight corners");
                Check.That(r.PureTorque.sqrMagnitude == 0f && rc.UpperActuatorTorque.sqrMagnitude == 0f, "value-copy original unchanged");
                var broken = lc; broken.UpperActuatorTorque.z += .02f;
                Check.That(Probe.Compare(p,m,l,r,broken,rc).State == Probe.Status.BaselineMismatch, "left receipt is essential");
                broken = rc; broken.AllocatedHipAlpha.x += .1f;
                Check.That(Probe.Compare(p,m,l,r,lc,broken).State == Probe.Status.BaselineMismatch, "right allocated alpha is essential");
                broken = rc; broken.Fixed++;
                Check.That(Probe.Compare(p,m,l,r,lc,broken).State == Probe.Status.BaselineMismatch, "wrong local stamp");
            }
        }
        private static void Ownership()
        {
            var rng = new Random(150);
            float R() => (float)rng.NextDouble()*2f-1f;
            int accepted=0, rejected=0;
            for (int i=0; i<128; i++)
            {
                var p=Pose(); Pair(p,out var l,out var r,out _,out _);
                r.ChildW=new Vector3(R(),R(),R());
                r.ChildInertia=new Vector3(.18f,.31f,.06f);
                r.ChildPrincipal=Quaternion.AngleAxis(48f,new Vector3(1f,2f,1f).normalized);
                Vector3 Torque(Vector3 v) => HipAngularRequest.Tensor(r.ChildInertia,r.ChildPrincipal,v);
                var rel=new Vector3(8f*R(),8f*R(),8f*R()); var carry=new Vector3(2f,1f,-3f);
                r.DesiredAlpha=rel+carry;
                r.PureTorque=Torque(r.DesiredAlpha)+Vector3.Cross(r.ChildW,Torque(r.ChildW));
                Check.That(HipAngularRequest.TryCreate(r.ChildInertia,r.ChildPrincipal,r.ChildW,r.PureTorque,
                    r.DesiredAlpha,rel,carry,r.DesiredAlpha,40f,out r.Angular), "nonzero transport and gyro fixture");
                float limit=HipBilateralForceSolver.ActuatorLimit(r.PureTorque);
                var delta=new Vector3(R(),R(),R())*(i%2==0?2f:48f);
                if (!Probe.TryStageRelative(r,delta,40f,limit,out var trial))
                {
                    rejected++; Check.That((r.DesiredAlpha+delta).magnitude>40f-.0002f ||
                        (r.PureTorque+Torque(delta)).magnitude>limit-.0002f,"only a physical boundary rejects this fixture");
                    continue;
                }
                accepted++;
                Check.That(Probe.Same(trial.Angular.RelativeAlpha,r.Angular.RelativeAlpha+delta) &&
                    Probe.Same(trial.Angular.TransportAlpha,r.Angular.TransportAlpha) &&
                    Probe.Same(trial.Angular.GyroTorque,r.Angular.GyroTorque) &&
                    Probe.Same(trial.Angular.TransportTorque,r.Angular.TransportTorque), "only relative ownership changes");
                Check.That(trial.DesiredAlpha.magnitude<=40.0001f && trial.PureTorque.magnitude<=limit+.0001f,"original rather than trial-created envelope");
                Check.That(HipBilateralForceSolver.TryPlan(l,trial,out var lc,out var rc) && rc.UpperActuatorTorque.magnitude<=limit+.0001f &&
                    lc.UpperActuatorTorque.magnitude<=HipBilateralForceSolver.ActuatorLimit(l.PureTorque)+.0001f,"actual allocated torque on both sides");
            }
            Check.That(accepted>50&&rejected>15,"both legal and exhausted requests reached the real allocator");
        }
        private static void Tradeoffs()
        {
            var p = Pose(false); Pair(p, out var l, out var r, out var lc, out var rc);
            var m = Motion(); var alt = Probe.Compare(p,m,l,r,lc,rc);
            Check.That(alt.State == Probe.Status.Tradeoff && alt.Gain.x > 0 && alt.Gain.y < 0 && alt.Gain.z < 0, "knee rises while ankle falls");
            p = Pose();
            p.RawAnkleY = p.LowerAnkle.y - .004f;
            alt = Probe.Compare(p,m,l,r,lc,rc);
            Check.That(alt.State == Probe.Status.Tradeoff && alt.Gain.z > 0f && alt.Gain.y < 0f, "shallower sole with larger tracking error");
            p = Pose(); m.ReferenceMaxW = .001f; alt = Probe.Compare(p,m,l,r,lc,rc);
            Check.That(alt.State == Probe.Status.BelowGain && alt.TrialNextW <= m.ReferenceMaxW + .000001f, "small speed remainder is respected");
            m = Motion(); m.ReferenceMaxA = .001f; alt = Probe.Compare(p,m,l,r,lc,rc);
            Check.That(alt.State != Probe.Status.Improved && alt.AddedA <= .001f, "no invented angular allowance");
            m = Motion(); m.UpperMaxW = float.NaN;
            Check.That(!Probe.Compare(p,m,l,r,lc,rc).HasBase, "missing native ceiling");
            Pair(p, out l, out r, out lc, out rc, 9f, 1f); m=Motion();
            alt = Probe.Compare(p,m,l,r,lc,rc);
            Check.That(rc.UpperActuatorLimitNm >= 9f && alt.State == Probe.Status.Budget && !alt.HasTrial, "legacy envelope already above 8 Nm cannot be enlarged");
            p = Pose(); Pair(p, out l, out r, out lc, out rc);
            m.KneeGapV = .1f * Vector3.up; m.AnkleGapV = -.03f * Vector3.up;
            alt = Probe.Compare(p,m,l,r,lc,rc);
            Check.Near(alt.Baseline.Heights.x-p.LowerKnee.y,.1f*p.Dt,.000001f,"native knee velocity transport");
            Check.Near(alt.Baseline.Heights.z-p.Box,.07f*p.Dt,.000001f,"two independent anchor gap velocities");
        }
        private static void Readbacks()
        {
            var p = Pose(); var w = new RightThighWindow(); var a = Sample(1,p); var b = Sample(2,p);
            for (int change = 0; change < 11; change++)
            {
                w = new RightThighWindow(); w.Observe(a,p); w.Observe(b,p);
                w.Complete(new Probe.Result { State = Probe.Status.BelowGain, HasBase = true,
                    Baseline = new Probe.Forecast { Heights = new Vector3(p.LowerKnee.y,p.LowerAnkle.y,p.Box) } });
                var c = Sample(3,p);
                switch(change)
                {
                    case 0: c.NowActive = true; break;
                    case 1: c.Fixed++; break;
                    case 2: c.Source.Time = -1; break;
                    case 3: c.SupportId++; break;
                    case 4: c.FloorHalf.x += .01f; break;
                    case 5: c.AnkleHeightsValid = false; break;
                    case 6: c.KneeHeightsValid = false; break;
                    case 7: c.Min = float.NaN; break;
                    case 8: c.WasActive = true; break;
                    case 9: w.Cut(); break;
                    case 10: c.SourceAge = .11f; break;
                }
                w.Observe(c,p); string text = w.Summary();
                Check.That(text.Contains(change==0 ? "/next=Measured/nextLocal-source=3|3" : "/next=Censored"), "exact N+1 only " + change);
                Check.That(text.Contains(change==0 ? "/baselineMAEMm(knee-ankle-box)=0|0|0" : "/baselineCalibrationN=0"), "calibration denominator");
                Check.That(text == w.Summary(), "stopping twice is idempotent");
            }
            w = new RightThighWindow();
            for (int j=0; j < RightThighWindow.Capacity+3; j++)
            {
                int n=j*4+1; w.Observe(Sample(n,p),p); w.Observe(Sample(n+1,p),p); w.Complete(default);
                Check.That(w.Observe(Sample(n+2,p),p)==Signal.Held, "sustained episode never repeats");
                w.Cut();
            }
            var sum=w.Summary();
            Check.That(sum.Contains("/saved-dropped=128|3") && sum.Contains("/next(measured-censored)=131|0") && sum.Contains("/Missing=131"), "full counts past detail capacity");
            w = new RightThighWindow(); w.Observe(a,p); w.Observe(b,p);
            Check.That(w.Summary().Contains("/Missing=1") && w.Summary().Contains("/next=Censored"), "stop before N commits");
        }
        private static void Events()
        {
            foreach (bool ordinary in new[]{ false,true })
                foreach (bool low in new[]{ false,true })
                {
                    var p = Pose(); var h = new LandingEventHistory(); var w = new RightThighWindow();
                    for (int i=1;i<=8;i++)
                    {
                        var s=Sample(i,p);
                        if (!low) s.AnkleHeights.x=s.AnkleHeights.y;
                        if (i>=6) { s.Strong=true; s.Gait=Audit.Phase.Approach; }
                        if (i==8) { s.OrdinaryEnabled=ordinary; s.FirstMiss=!ordinary; s.DepthLatchFixed=6; }
                        s.RightThigh=w.Observe(s,p); h.Add(s,i/60d,Audit.Screen(s),p);
                        if (i==8) { h.Capture(s,!ordinary); if (!ordinary) h.EndMiss(LandingEventHistory.MissEnd.HoldEnded); }
                    }
                    var str=new StringBuilder(); h.Append(str); string log=str.ToString();
                    Check.That(log.Contains("RIGHT_THIGH_EVENT_V041500") && log.Contains(low ? "/onsets-held=1|3/first-lastOnsetLocal=2|2" : "/onsets-held=0|0/first-lastOnsetLocal=0|0"), "edge attribution " + ordinary + "/" + low);
                }
            var inactive = new LandingEventHistory(); var a = Sample(1,Pose());
            inactive.Add(a,0d,Audit.Screen(a)); inactive.Capture(a,false);
            var sb=new StringBuilder(); inactive.Append(sb);
            Check.That(!sb.ToString().Contains("RIGHT_THIGH_EVENT"), "A chain cannot synthesize B metadata");
        }
        private static void Cost()
        {
            var p=Pose(); var w=new RightThighWindow(); var m=Motion(); Pair(p,out var l,out var r,out var lc,out var rc);
            for(int n=1;n<=50;n++) { w.Observe(Sample(n,p),p); Probe.Compare(p,m,l,r,lc,rc); }
            long before=GC.GetAllocatedBytesForCurrentThread();
            for(int n=51;n<=350;n++) { w.Observe(Sample(n,p),p); Probe.Compare(p,m,l,r,lc,rc); }
            long used=GC.GetAllocatedBytesForCurrentThread()-before;
            Check.That(used==0, "ongoing observer and bilateral comparison allocate " + used + " bytes");
        }
    }
}
