using System;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace ARL.Contracts
{
    internal static class HipContactPreparationContracts
    {
        internal static void Run()
        {
            Check.Run("logged shallow geometry separates preparation from ordinary contact", Shallow);
            Check.Run("settled preparation requires actual source and physical time", Cadence);
            Check.Run("settled preparation rejects unsafe bilateral motion and geometry", Unsafe);
            Check.Run("settled preparation rejects discontinuity and does not extend timeout", Lifetime);
        }
        private static HipContactPreparationSample Sample(float min = -.0027f) => new HipContactPreparationSample
        {
            NativeOff = true, IdentityQualified = true,
            Geometry = new InitialGeometryPreflight { Valid = true, Calibrated = true, FootprintsInside = true,
                RawMin = -.0026f, ControlMin = min, ActualMin = min },
            Motion = new FootContactMotion { Valid = true, FullFootprint = true, Min = min, PredictedMin = min }
        };
        private static PhysicsStepContext Step(int n, int seq)
        { var s = RuntimeContracts.Step(n); s.SourceSequence = seq; return s; }
        private static void Feed(HipContactPreparationWindow w, int n, HipContactPreparationSample left,
            HipContactPreparationSample right, int hz = 60, string unavailable = null)
        {
            int seq = (n - 1) * hz / 60 + 1;
            w.Observe(Step(n, seq), new SourceStepContext { Epoch = 1, Sequence = seq,
                Time = (seq - 1) / (float)hz, Dt = 1f / hz }, left, right, unavailable);
        }
        private static void Shallow()
        {
            foreach (float min in new[] { -.002687f, -.002594f, -.002647f, -.002729f, -.003f })
            {
                var p = Sample(min);
                Check.That(p.Geometry.PreparationIssue == null && p.Issue == null, "observed R1 endpoint can enter unloaded preparation");
                Check.That(p.Geometry.Issue != null && p.Geometry.PreparationRoute == "SHALLOW_RECOVERY_REQUIRED",
                    "preparation pass does not become ordinary contact permission");
            }
            Check.That(Sample(-.002f).Geometry.Issue == null, "ordinary boundary remains exactly 2 mm");
            Check.That(Sample(-.00301f).Issue != null, "preparation never broadens beyond 3 mm");
            Check.That(FootContactGeometryLimits.OrdinaryPenetrationMeters == .002f &&
                FootContactGeometryLimits.RecoveryPenetrationMeters == .003f, "runtime geometry limits unchanged");
        }
        private static void Cadence()
        {
            foreach (int hz in new[] { 20, 40, 60 })
            {
                var w = new HipContactPreparationWindow(); w.Begin(Step(1,1));
                for (int n=2; n<=10; n++) Feed(w,n,Sample(),Sample(),hz);
                Check.That(w.Pending && !w.Qualified, "less than 150 ms after the first actual sample cannot qualify");
                int k=11; while (w.Pending && k<25) { Feed(w,k,Sample(),Sample(),hz); k++; }
                Check.That(w.Qualified && w.StableSeconds + .000001f >= .15f, "quiet native samples qualify at all three source rates");
                int seq=(k-2)*hz/60+1;
                Check.That(w.QualifiedFor(Step(k-1,seq)), "receipt belongs to the exact qualified step");
                Check.That(!w.QualifiedFor(Step(k,seq)), "receipt cannot authorize a later step");
            }
            var held = new HipContactPreparationWindow(); held.Begin(Step(1,1));
            var source = new SourceStepContext { Epoch=1,Sequence=1,Time=0,Dt=1f/60f };
            for (int n=2;n<30;n++) held.Observe(Step(n,1),source,Sample(),Sample(),null);
            Check.That(held.Pending && !held.Qualified && held.StableSeconds==0f, "Fixed catchup cannot turn a held source into settled duration");
            var same = new HipContactPreparationWindow(); same.Begin(Step(1,1)); Feed(same,1,Sample(),Sample());
            Check.That(!same.Pending && !same.Qualified, "initializer pose in the join step is not a settled native solve");
        }
        private static void Unsafe()
        {
            var baseline = Sample();
            for (int fault=0;fault<13;fault++)
            {
                var bad=baseline;
                switch(fault)
                {
                    case 0: bad.NativeOff=false; break;
                    case 1: bad.IdentityQualified=false; break;
                    case 2: bad.Motion.Valid=false; break;
                    case 3: bad.Geometry.ActualMin=-.00363f; break;
                    case 4: bad.Motion.PredictedMin=-.00301f; break;
                    case 5: bad.Motion.CornerNormalSpeed=.03001f; break;
                    case 6: bad.Motion.PlanarSpeed=.05001f; break;
                    case 7: bad.Motion.AngularSpeed=.50001f; break;
                    case 8: bad.Motion.AnchorGap=.00501f; break;
                    case 9: bad.Motion.SourceAge=.10001f; break;
                    case 10: bad.Motion.FullFootprint=false; break;
                    case 11: bad.Motion.Min=float.NaN; break;
                    case 12: bad.Motion.AngularSpeed=float.PositiveInfinity; break;
                }
                Check.That(bad.Issue!=null,"unsafe sample has a diagnostic: "+fault);
                foreach(bool primary in new[]{true,false})
                {
                    var w=new HipContactPreparationWindow(); w.Begin(Step(1,1));
                    for(int n=2;n<35;n++) Feed(w,n,primary?bad:baseline,primary?baseline:bad);
                    Check.That(w.Pending && !w.Qualified && w.StableSeconds==0f,"either unsafe foot blocks readiness: "+fault);
                }
            }
            var boundary=baseline;
            boundary.Motion.CornerNormalSpeed=.03f; boundary.Motion.PlanarSpeed=.05f;
            boundary.Motion.AngularSpeed=.5f; boundary.Motion.AnchorGap=.005f; boundary.Motion.SourceAge=.1f;
            Check.That(boundary.Issue==null,"existing inclusive recovery quietness boundaries are retained");
        }
        private static void Lifetime()
        {
            var p=Sample();
            var w=new HipContactPreparationWindow(); w.Begin(Step(1,1)); Feed(w,2,p,p); Feed(w,4,p,p);
            Check.That(!w.Pending && w.Reason=="CLOCK_OR_SOURCE_DISCONTINUITY","skipped solve cancels the preparation");
            w.Begin(Step(1,1)); Feed(w,2,p,p); Feed(w,2,p,p);
            Check.That(!w.Pending && !w.Qualified,"duplicate solve is not counted twice");
            w.Begin(Step(1,1)); Feed(w,2,p,p); var changed=Step(3,3); changed.Epoch=2;
            w.Observe(changed,new SourceStepContext{Epoch=2,Sequence=3,Time=.03f,Dt=.02f},p,p,null);
            Check.That(!w.Pending && !w.Qualified,"binding change invalidates readiness");
            w.Begin(Step(1,1)); Feed(w,2,p,p);
            var jumped=Step(3,3); jumped.UnscaledTime=.3f;
            w.Observe(jumped,new SourceStepContext{Epoch=1,Sequence=3,Time=.3f,Dt=.28f},p,p,null);
            Check.That(w.Pending && !w.Qualified && w.StableSeconds==0f && w.Reason=="WAIT_CONTIGUOUS_FRESH_SOURCE",
                "render gap discards the current quiet hold without manufacturing elapsed source samples");
            w.Begin(Step(1,1));
            for(int n=2;n<1000 && w.Pending;n++) Feed(w,n,p,p,60,n%4==0?"SUPPORT_UNAVAILABLE":null);
            Check.That(!w.Qualified && w.Reason.StartsWith("TIMEOUT:") && w.Elapsed<15.1f,
                "repeated hold resets do not restart the independent 15 second deadline");
            w.Begin(Step(1,1)); Feed(w,2,p,p); w.Cancel("USER_STOP"); Feed(w,3,p,p);
            Check.That(!w.Pending && !w.Qualified && w.Reason=="USER_STOP","stop prevents later samples from reviving readiness");
            w.Reset(); Check.That(w.Elapsed==0f && w.PeakStableSeconds==0f && w.Reason=="NOT_REQUESTED","reset discards prior duration and receipt");
        }
    }
}
