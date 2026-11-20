using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;
using Signal = MakemitAGA.ActiveRagdollLab.Physics.SwingLandingWindow.Signal;
using Status = MakemitAGA.ActiveRagdollLab.Physics.SwingKneeCandidate.Status;

namespace ARL.Contracts
{
    internal static class SwingKneeContracts
    {
        internal static void Run()
        {
            Check.Run("v145 prospective swing window uses measured time, new sources and signed geometry", Window);
            Check.Run("v145 swing window drops broken and stale support and allocates nothing in the hot path", Breaks);
            Check.Run("v145 bounded knee comparison closes the measured pose and independently raises both geometries", Geometry);
            Check.Run("v145 event exposure ends at the authoritative depth latch, independently of later confirmation", Events);
        }
        internal static Audit.Sample Sample(int n,float dt,float t)
        {
            var source = new SourceStepContext { Epoch=2, Sequence=n, Time=t, Dt=dt };
            return new Audit.Sample { Step=new PhysicsStepContext { Epoch=2, Sequence=n, SourceSequence=n, FixedTime=t, UnscaledTime=t, Dt=dt },
                Fixed=n, Source=source, SupportId=3, FloorHalf=new Vector3(2,.1f,2), Gait=Audit.Phase.Swing, SourceMotion=Audit.SourceKind.Motion,
                SourcePhase="TURN", Identity=true, FivePoint=true, GeometryValid=true, MotionValid=true, FullFootprint=true, BoxReferencesValid=true,
                RawMin=-.01f+t*.08f, Min=-.004f-t*.10f, SourceAge=.01f, AnchorGap=.002f, KneeHeightsValid=true, AnkleHeightsValid=true,
                KneeHeights=new Vector3(1,1,.996f-t*.06f), AnkleHeights=new Vector3(.1f,.096f-t*.08f,.095f-t*.08f), SwingMeasured=true };
        }
        private static void Window()
        {
            foreach (int hz in new[]{20,30,60,120,240})
            {
                var w=new SwingLandingWindow(); float dt=1f/hz;
                for (int i=0; i<=Math.Ceiling(.05f/dt)+1; i++)
                {
                    var r=w.Observe(Sample(i+1,dt,i*dt));
                    Check.That(r==(i*dt+1e-6f>=.05f ? Signal.Diverging : Signal.Low),"horizon "+hz+"/"+i);
                }
            }
            var window=new SwingLandingWindow(); var a=Sample(1,.015f,0f); window.Observe(a);
            float t=0; int n=1;
            foreach (float d in new[]{.010f,.014f,.018f,.017f})
            {
                t+=d; var b=Sample(++n,d,t); b.Source=a.Source; b.Step.SourceSequence=a.Source.Sequence;
                Check.That(window.Observe(b)==Signal.Low,"a held animation cannot make a trend");
            }
            var fresh=Sample(++n,.016f,t+.016f); Check.That(window.Observe(fresh)==Signal.Diverging,"new source in causal interval");
            window.Reset(); for(n=0;n<4;n++){var s=Sample(n+1,.02f,n*.02f); s.RawMin=-.002f-n*.002f;
                var sig=window.Observe(s); if(n==3)Check.That(sig==Signal.Worsening,"a lowering source must not classify as divergence");}
            window.Reset(); for(n=0;n<4;n++){var s=Sample(n+1,.02f,n*.02f);s.AnkleHeights.y+=.04f;
                Check.That(window.Observe(s)==Signal.None,"high ankle alone excludes signal");}
        }
        private static void Breaks()
        {
            for(int kind=0;kind<13;kind++)
            {
                var w=new SwingLandingWindow(); for(int i=1;i<=4;i++)w.Observe(Sample(i,.02f,i*.02f));
                var changed=Sample(5,.02f,.1f);
                switch(kind){case 0: changed.FivePoint=false;break; case 1: changed.Identity=false;break; case 2: changed.SelectedBoxFallback=true;break;
                case 3: changed.Gait=Audit.Phase.Approach;break; case 4: changed.NowActive=true;break; case 5: changed.SourceAge=.11f;break;
                case 6: changed.AnchorGap=.006f;break; case 7: changed.KneeHeightsValid=false;break; case 8: changed.AnkleHeights.y=float.NaN;break;
                case 9: changed.SupportId=44;break; case 10: changed.Fixed=4;break; case 11: changed.FloorCenter.x=.1f;break; case 12: changed.Source.Time=0f;break;}
                w.Observe(changed); Check.That(w.Observe(Sample(6,.02f,.12f))!=Signal.Diverging,"did not bridge gap mode "+kind);
            }
            var window=new SwingLandingWindow(); for(int i=1;i<30;i++)window.Observe(Sample(i,.01f,i*.01f));
            long bytes=GC.GetAllocatedBytesForCurrentThread(); for(int i=30;i<2000;i++)window.Observe(Sample(i,.01f,i*.01f));
            Check.That(bytes==GC.GetAllocatedBytesForCurrentThread(),"zero ordinary-step managed allocation");
        }
        private static float Minimum(Vector3 ankle,Quaternion q,KneeGroundTarget.Geometry g)
        {
            float min=float.PositiveInfinity;
            for(int x=-1;x<=1;x+=2)for(int y=-1;y<=1;y+=2)for(int z=-1;z<=1;z+=2)
                min=Mathf.Min(min,(ankle+q*(g.Center+new Vector3(x*g.Half.x,y*g.Half.y,z*g.Half.z))).y-g.FloorCenter.y-g.FloorHalf.y);
            return min;
        }
        internal static SwingKneeCandidate.Pose FixturePose(LandingPreparationFixtures.Fixture f)
        {
            var g=f.Packet.Geometry; g.Upper=f.Hip.Rotation;
            var q=g.Upper*Quaternion.AngleAxis(2f,Vector3.forward); var lower=q*f.Knee.R*Quaternion.AngleAxis(1.5f,Vector3.right);
            var actualFoot=f.Packet.Sample.PlannedFoot*Quaternion.AngleAxis(-.8f,Vector3.forward);
            var knee=g.Hip+q*g.UpperSpan+new Vector3(.001f,-.004f,-.002f);
            var ankle=knee+lower*g.LowerSpan; var gap=new Vector3(.001f,.002f,.0008f);
            return new SwingKneeCandidate.Pose { Reference=g, PhysicalKnee=knee, AnkleGap=gap, Axis=f.Packet.Input.Axis,
                AllowExtension=true, HipW=f.Hip.W, PhysicalUpper=q, PhysicalLower=lower,
                PhysicalFootRelative=actualFoot, SelectedFoot=f.Packet.Sample.PlannedFoot,
                ObservedBox=Minimum(ankle+gap,lower*actualFoot,g), LowerAnkleY=ankle.y };
        }
        private static void Geometry()
        {
            var rand=new Random(145); int favorable=0,checkedGates=0;
            for(int i=0;i<500;i++)
            {
                var f=LandingPreparationFixtures.Make(rand);var k=f.Knee;var p=FixturePose(f);var g=p.Reference;
                var r=SwingKneeCandidate.Evaluate(p,k,k.Step,k.Source);
                if (r.Reason == Status.NoLegal && r.WithinGeometry > 0)
                    Check.That(r.LaneIndex > 0 && r.Detail().Contains("candidate=CLOSEST_REJECTED") &&
                        (Mathf.Min(r.ReferenceGain,Mathf.Min(r.PhysicalGain,r.AnkleGain))<KneeGroundTarget.MinGain ||
                        r.CoastGain<0f || r.PhysicalCoastGain<0f), "a near miss retains its measured margin");
                if(r.Reason!=Status.Favorable)continue;
                favorable++; float dt=k.Step.Dt;
                var lane=LandingPreparationMath.MakeLane(k.PreviousR,k.PreviousW,k.InputR,k.InputW,k.R,k.W,dt,k.MaxW,k.MaxA,p.Axis,true);
                Check.That(LandingPreparationMath.TryLane(lane,g,r.LaneIndex,out var cand,out var w),"uses existing legal R/W lane");
                Check.That(r.CandidateW<=r.LimitW+.0001f && r.CandidateA<=r.LimitA+.0002f,"actual sampled angular budgets");
                var referenceFoot=p.SelectedFoot; var lower=g.Upper*cand;
                var ankle=g.Hip+g.Upper*g.UpperSpan+lower*g.LowerSpan;
                var oldLower=g.Upper*k.R;var oldAnkle=g.Hip+g.Upper*g.UpperSpan+oldLower*g.LowerSpan;
                float reference=Minimum(ankle,lower*referenceFoot,g)-Minimum(oldAnkle,oldLower*referenceFoot,g);
                var delta=p.PhysicalUpper*cand*Quaternion.Inverse(k.R)*Quaternion.Inverse(p.PhysicalUpper);
                var actualLower=delta*p.PhysicalLower;var actualAnkle=p.PhysicalKnee+actualLower*g.LowerSpan;
                float gain=Minimum(actualAnkle+p.AnkleGap,actualLower*p.PhysicalFootRelative,g)-p.ObservedBox;
                Check.Near(reference,r.ReferenceGain,.000002f,"eight reference corners");Check.Near(gain,r.PhysicalGain,.000002f,"eight measured corners");
                Check.Near(actualAnkle.y-p.LowerAnkleY,r.AnkleGain,.000002f,"preserved knee and ankle gaps");
                var rc=FootRelativeReference.Advance(cand,w,dt);var bc=FootRelativeReference.Advance(k.R,k.W,dt);
                var uc=FootRelativeReference.Advance(g.Upper,p.HipW,dt);
                float RefC(Quaternion q) => Minimum(g.Hip+uc*g.UpperSpan+uc*q*g.LowerSpan,uc*q*referenceFoot,g);
                float BodyC(Quaternion q){var l=p.PhysicalUpper*q*Quaternion.Inverse(k.R)*Quaternion.Inverse(p.PhysicalUpper)*p.PhysicalLower;
                    return Minimum(p.PhysicalKnee+l*g.LowerSpan+p.AnkleGap,l*p.PhysicalFootRelative,g);}
                Check.Near(RefC(rc)-RefC(bc),r.CoastGain,.000002f,"reference coast independently reconstructed");
                Check.Near(BodyC(rc)-BodyC(bc),r.PhysicalCoastGain,.000002f,"physical substitution coast independently reconstructed");
                Check.That(r.PhysicalGain>=.0001f && r.ReferenceGain>=.0001f && r.AnkleGain>=.0001f && r.CoastGain>=0f && r.PhysicalCoastGain>=0f,"minimum useful gain");
                if(checkedGates++==0){var bad=p;bad.ObservedBox+=.001f;Check.That(SwingKneeCandidate.Evaluate(bad,k,k.Step,k.Source).Geometry==SwingKneeCandidate.GeometryIssue.MeasuredClosure,"reject wrong floor or corner");
                    bad=p;bad.LowerAnkleY+=.001f;Check.That(SwingKneeCandidate.Evaluate(bad,k,k.Step,k.Source).Geometry==SwingKneeCandidate.GeometryIssue.MeasuredClosure,"reject wrong ankle");
                    bad=p;bad.Axis=Vector3.zero;Check.That(SwingKneeCandidate.Evaluate(bad,k,k.Step,k.Source).Reason==Status.Budget,"missing axis");
                    var old=k;old.Stale=true;Check.That(SwingKneeCandidate.Evaluate(p,old,k.Step,k.Source).Reason==Status.Protected,"stale");old=k;old.GroundTrajectory.Applied=true;
                    Check.That(SwingKneeCandidate.Evaluate(p,old,k.Step,k.Source).Reason==Status.Protected,"previous correction already applied");
                    old=k;old.Step.Sequence--;Check.That(SwingKneeCandidate.Evaluate(p,old,k.Step,k.Source).Reason==Status.Unavailable,"wrong step");
                    bad=p;bad.Reference.FloorHalf.x=.0001f;Check.That(SwingKneeCandidate.Evaluate(bad,k,k.Step,k.Source).Reason==Status.Geometry,"footprint guard");}
            }
            Console.WriteLine("  v145 synthetic distinct-pose favorable="+favorable+"/500"); Check.That(favorable>10,"exercised real feasible knee directions");
        }
        private static void Events()
        {
            var history=new LandingEventHistory();
            for(int i=1;i<=8;i++){
                var s=Sample(i,.02f,i*.02f);s.Gait=i<4?Audit.Phase.Swing:Audit.Phase.Approach; s.Strong=i>=4;
                if(i==2 || i==6){s.SwingSignal=Signal.Diverging;s.SwingCandidate.Reason=Status.Favorable;}
                s.FirstMiss=i==8;s.DepthLatchFixed=i==8?4:0;history.Add(s,i*.02,Audit.Screen(s));
                if(i==8){history.Capture(s,true);history.EndMiss(LandingEventHistory.MissEnd.HoldEnded);}}
            var b=new System.Text.StringBuilder();history.Append(b);string text=b.ToString();
            Check.That(text.Contains("anchorLocal=4") && text.Contains("beforeSteps(low-trend-diverging-favorable)=1|1|1|1") && text.Contains("leadToAnchorMs=40/") && !text.Contains("SWING_KNEE_V041450/local=6/"),"no later evidence credited");
            history.Reset();var v=Sample(50,.02f,4f);v.FirstMiss=true;v.DepthLatchFixed=20;history.Add(v,4d,Audit.Screen(v));history.Capture(v,true);b.Clear();history.Append(b);
            Check.That(b.ToString().Contains("anchorCaptured=False/beforeCounts=NA"),"explicit missing depth latch");
        }
    }
}
