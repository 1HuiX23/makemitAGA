using System;
using System.Globalization;
using System.Linq;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;

namespace ARL.Contracts
{
    internal static class LandingEventHistoryContracts
    {
        internal static void Run()
        {
            Check.Run("v144 half-second evidence uses Fixed time and held source identities at 20/30/60/120 Hz", Horizon);
            Check.Run("v144 mixed Fixed intervals retain the correct real half-second across held sources", MixedIntervals);
            Check.Run("v144 event samples end on real entry and preserve the signed joint and box heights", Heights);
            Check.Run("v144 acquisition loss, plane changes, stop and recovery retain outcomes and censorship", Boundaries);
            Check.Run("v144 physical sampling cap and independent detail caps disclose all omitted events", Capacity);
            Check.Run("v144 continuous qualification across new swing starts a new same-foot window", Swing);
            Check.Run("v144 compact history allocates nothing between events", Allocation);
            Check.Run("v144 knee allocation report retains publication values while physical step allocates no text", Knee);
            Check.Run("v144 unscaled Fixed regression is rejected before publication or consumption", Clock);
        }

        private static Audit.Sample Sample(int i, float dt = 1f / 60)
        {
            var s = LandingOpportunityContracts.S(i);
            s.Step.Dt = dt; s.Step.FixedTime = s.Step.UnscaledTime = i * dt;
            int source = (i+1)/2;
            s.Source.Sequence = s.Step.SourceSequence = source; s.Source.Time = source * dt * 2;
            s.SourcePhase = "TURN"; s.BoxReferencesValid = true; s.AnkleHeightsValid = s.KneeHeightsValid = true;
            s.KneeHeights = new Vector3(.51f,.50f,.49f);
            s.AnkleHeights = new Vector3(.101f,.086f,.084f);
            return s;
        }

        private static string[] Rows(string text) => text.Split('\n').Where(l => l.StartsWith("H144,", StringComparison.Ordinal)).ToArray();
        private static double Get(string row, int column) => double.Parse(row.Split(',')[column], CultureInfo.InvariantCulture);

        private static void Horizon()
        {
            foreach (int hz in new[] {20, 30, 60, 120})
            {
                var a = new Audit();
                for (int i = 1; i <= 2*hz; i++)
                {
                    var s = Sample(i, 1f/hz);
                    if (i == 2*hz) s.NowActive = s.OrdinaryEnabled = true;
                    a.Observe(s, true);
                }
                a.Seal(); string result=a.Summary(); var rows=Rows(result);
                Check.That(rows.Length == hz/2+1 && result.Contains("/Ordinary=1|1|0|1|0") &&
                    result.Contains("/coverage=FULL"), "inclusive half second per event");
                Check.That(Math.Abs(Get(rows[0],3)+500)<.01 && Get(rows[^1],3)==0 &&
                    rows.Select(r=>Get(r,2)).Distinct().Count() < rows.Length, "held packets stay visible as repeated IDs");
            }
        }

        private static void MixedIntervals()
        {
            var audit = new Audit();
            float[] dts = { .01f, .025f, .016f, .05f, .008f };
            var times = new double[89];
            double sum = 0d;
            int source = 0;
            float sourceTime = 0f;
            for (int i = 1; i <= times.Length; i++)
            {
                float dt = dts[(i - 1) % dts.Length];
                var sample = Sample(i, dt);
                times[i - 1] = sum += dt;
                sample.Step.FixedTime = sample.Step.UnscaledTime = (float)sum;
                if (i % 3 == 1) { source++; sourceTime = (float)sum; }
                sample.Source.Sequence = sample.Step.SourceSequence = source;
                sample.Source.Time = sourceTime;
                sample.Source.Dt = .04f;
                if (i == times.Length) sample.OrdinaryEnabled = sample.NowActive = true;
                audit.Observe(sample, true);
            }
            audit.Seal();
            string summary = audit.Summary();
            var rows = Rows(summary);
            var within = times.Where(t => sum - t <= .500001d).ToArray();
            Check.That(summary.Contains("coverage=FULL") && rows.Length == within.Length, "mixed cadence coverage");
            for (int i = 0; i < rows.Length; i++)
                Check.That(Math.Abs(Get(rows[i], 3) - (within[i] - sum) * 1000d) < .001d, "each recorded offset follows its real Fixed duration");
        }

        private static void Heights()
        {
            var a = new Audit(); var s = Sample(1); s.FloorCenter.y+=.04f;
            a.Observe(s, true); s=Sample(2); s.FloorCenter.y+=.04f;
            s.FirstMiss=s.MissInProgress=true; s.Strong=true; s.Gait=Audit.Phase.Stance;
            s.Min=-.016f; s.RawMin=-.0013f; s.ControlMin=-.014f; a.Observe(s,true);
            s=Sample(3); s.FloorCenter.y+=.04f; a.Observe(s,true); a.Seal();
            string result=a.Summary(); var rows=Rows(result);
            Check.That(result.Contains("kind=FirstMiss/outcome=HoldEnded") && rows.Length==2 && Get(rows[^1],1)==102, "end on first miss, later outcome does not move the endpoint");
            var last=rows[^1];
            foreach (var pair in new[]{(8,-16d),(9,-1.3),(10,-14d),(17,945d),(18,470d),(19,460d),(20,450d),(21,61d),(22,46d),(23,44d)})
                Check.That(Math.Abs(Get(last,pair.Item1)-pair.Item2)<.01, "signed support-relative value at column "+pair.Item1);
            Check.That(last.Split(',')[5]=="TURN" && (int)Get(last,6)%8==7 && (int)Get(last,7)!=0, "phase, original witnesses and rejection preserved");
            a.Reset(); s=Sample(1); s.TrackingValid=false; s.AnkleHeightsValid=false;
            s.NowActive=s.OrdinaryEnabled=true; a.Observe(s,true); a.Seal();
            var partial=Rows(a.Summary()).Single().Split(',');
            Check.That(partial[9]!="NA" && partial[10]!="NA" && partial[21]=="NA","independent raw/control box measurements survive missing ankle tracking");
            a.Reset(); s=Sample(2); s.GeometryValid=s.ParentValid=s.TrackingValid=false; s.NowActive=s.OrdinaryEnabled=true;
            a.Observe(s,true); a.Seal(); var missing=Rows(a.Summary()).Single().Split(',');
            Check.That(missing[8]=="NA" && missing[17]=="NA" && missing[23]=="NA", "missing values never become plausible zeros");
        }

        private static void Boundaries()
        {
            for (int change=0; change<5; change++)
            {
                var a=new Audit(); var s=Sample(1); a.Observe(s,true);
                s=Sample(2); s.FirstMiss=s.MissInProgress=true; a.Observe(s,true);
                s=Sample(3); s.MissInProgress=s.NowActive=s.RecoveryEnabled=true;
                if(change==0)s.SupportId++;
                if(change==1)s.FloorCenter.y+=.001f;
                if(change==2)a.Fault();
                if(change==3){a.Seal();a.Observe(s,true);}
                else a.Observe(s,true);
                a.Seal();
                string outcome=change<3?"Discontinuity":change==3?"TraceEnded":"Recovered";
                Check.That(a.Summary().Contains("kind=FirstMiss/outcome="+outcome) && Rows(a.Summary()).Length==2, "origin conserved at "+outcome);
            }
            var truncated=new Audit(); truncated.Observe(Sample(1),true); var t=Sample(3); t.NowActive=t.OrdinaryEnabled=true;
            truncated.Observe(t,true); truncated.Seal(); Check.That(Rows(truncated.Summary()).Length==1 && truncated.Summary().Contains("LEFT_CENSORED"), "no borrowed sample across missing Fixed");
            truncated.Reset(); t=Sample(1);t.NowActive=t.OrdinaryEnabled=true;truncated.Observe(t);
            Check.That(!truncated.Summary().Contains("LANDING_HISTORY_EVENT_V041440") && truncated.Summary().Contains("sampledInactive=False"), "reference arm and restart");
        }

        private static void Capacity()
        {
            var a=new Audit();
            for(int i=1;i<=700;i++){var s=Sample(i,.001f);if(i==700)s.FirstMiss=s.MissInProgress=true;a.Observe(s,true);}
            a.Seal();
            Check.That(Rows(a.Summary()).Length==LandingEventHistory.RingCapacity && a.Summary().Contains("/FirstMiss=1|1|0|0|1") && a.Summary().Contains("/coverage=CAPACITY"),"explicit high-rate truncation");
            a.Reset();
            for(int i=1;i<=145;i++){var s=Sample(i);if(i%2!=0)s.NowActive=s.OrdinaryEnabled=true;else{s.FirstMiss=s.MissInProgress=true;}a.Observe(s,true);}
            a.Seal();string result=a.Summary();
            Check.That(result.Contains("/Ordinary=73|64|9|") && result.Contains("/FirstMiss=72|64|8|") &&
                result.Split("LANDING_HISTORY_EVENT_V041440/").Length-1==128 && result.Contains("/HoldEnded=72|"),"independent event quotas and full denominator");
        }

        private static void Swing()
        {
            var a=new Audit(); var s=Sample(1);s.Gait=Audit.Phase.Stance;a.Observe(s,true);
            s=Sample(2);s.Gait=Audit.Phase.Lift;a.Observe(s,true);s=Sample(3);s.Min=-.004f;s.FirstMiss=s.MissInProgress=true;
            a.Observe(s,true);a.Seal();
            Check.That(a.Summary().Contains("/firstMiss=1|1|0|0"),"new cycle keeps its own early qualifying point");
            a.Reset();s=Sample(1);s.Gait=Audit.Phase.Stance;a.Observe(s,true);s=Sample(2);s.Gait=Audit.Phase.Swing;s.NowActive=s.OrdinaryEnabled=true;a.Observe(s,true);
            Check.That(a.Summary().Contains("/ordinary=1|0|1|0"),"old cycle never supplies false advance");
        }

        private static void Allocation()
        {
            var a=new Audit();for(int i=1;i<200;i++)a.Observe(Sample(i),true);
            long n=GC.GetAllocatedBytesForCurrentThread();
            for(int i=200;i<1200;i++)a.Observe(Sample(i),true);
            Check.That(GC.GetAllocatedBytesForCurrentThread()==n,"no managed allocation per ordinary history tick");
        }

        private static void Knee()
        {
            var a=new KneeFlexionFeedbackHeadroomAllocationIsolation();var r=new SingleLegChainReport();
            Check.That(r.KneeFlexionAllocationLastEvidence==null && r.KneeFlexionAllocationKey==null,"unpublished");
            a.SetEnabled(true,r); Check.That(r.KneeFlexionAllocationLastEvidence=="<none>","reset snapshot");
            r.TraceKneeClampSamples=7; r.TraceKneeRotationMeanDegrees=1.5f; r.TraceKneeRotationMaxDegrees=3f;
            float v=a.Compose(5,-2,4,r,out var ff,out var raw,out bool clamped);
            r.TraceKneeClampSamples=99;
            Check.That(v==4 && ff==0 && raw==5 && clamped && r.KneeFlexionAllocationKey.EndsWith("knee(clamp/mean/max)=7/1.5/3") &&
                r.KneeFlexionAllocationLastEvidence=="step=1/opposing=True/pd=5/ffIn=-2/ffEff=0/rawA=3/allocatedA=5/appliedA=4/maxA=4/clamp=True","full frozen scalar text and physical result");
            a.Compose(float.NaN,1,4,r,out _,out _,out _);
            Check.That(r.KneeFlexionAllocationLastEvidence=="invalid=True/step=2" && r.KneeFlexionAllocationKey.Contains("B_INVALID_NON_FINITE_ALLOCATION_INPUT"),"invalid receipt");
            long begin=GC.GetAllocatedBytesForCurrentThread();
            for(int i=0;i<1000;i++)a.Compose(1,2,5,r,out _,out _,out _);
            Check.That(GC.GetAllocatedBytesForCurrentThread()==begin,"physical knee allocation publishes scalars without strings");
        }

        private static void Clock()
        {
            var clock=new RuntimeStepClock(); var first=clock.CaptureFixed(2,.02f,3);
            Check.Throws<InvalidOperationException>(()=>clock.CaptureFixed(2.02f,.02f,2.99f),"unscaled clock regression");
            var equal=clock.CaptureFixed(2,.02f,3);
            var cursor=new StepContextCursor(); cursor.AcceptFixed(first);
            var bad=equal;bad.UnscaledTime=2.9f;
            Check.Throws<InvalidOperationException>(()=>cursor.AcceptFixed(bad),"malformed delivered stamp");
            cursor.AcceptFixed(equal); Check.That(equal.Sequence==2 && cursor.Fixed.Matches(equal),"rejection cannot consume the next identity; rounded equal values work");
        }
    }
}
