using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.EarlyLandingWindow.Foot;
using Signal = MakemitAGA.ActiveRagdollLab.Physics.EarlyLandingWindow.Signal;

namespace ARL.Contracts
{
    internal static class EarlyLandingContracts
    {
        internal static void Run()
        {
            Check.Run("v146 left trials start in lift, follow only contiguous initial swing on the Fixed clock", Left);
            Check.Run("v146 left windows drop broken clocks, changed supports, missed holds and invalid input", Breaks);
            Check.Run("v146 right steady ankle and near-plane screens have independent, causal denominators", Right);
            Check.Run("v146 protected handoffs use the committed hip path while stale and mixed sources stay unqualified", HipBasis);
            Check.Run("v146 pre-event truncation, separate side anchors and censored transitions", Events);
        }

        private static Audit.Sample Sample(int n, Audit.Phase phase, float dt = .02f)
        {
            var s = SwingKneeContracts.Sample(n,dt,n*dt);
            s.Gait = phase; s.SourcePhase = "STEADY"; s.Min = -.001f; s.RawMin = -.007f;
            s.AnkleHeights = new Vector3(.125f,.120f,.121f); s.SwingMeasured = false;
            return s;
        }

        private static void Left()
        {
            var w = new EarlyLandingWindow();
            foreach (float dt in new[] { 1f/30f, 1f/60f, .01f })
            {
                w.Reset();
                Check.That(w.Observe(Sample(1,Audit.Phase.Swing,dt),Foot.Left)==Signal.None,"a negative swing cannot start itself");
                var first=Sample(2,Audit.Phase.Lift,dt);
                Check.That(w.Observe(first,Foot.Left)==Signal.LeftStart && w.Origin==2,"lift starts from current evidence");
                var next=Sample(3,Audit.Phase.Lift,dt); next.RawMin=.02f; next.Source=first.Source; next.Step.SourceSequence=first.Source.Sequence;
                Check.That(w.Observe(next,Foot.Left)==Signal.LeftContinue && w.Origin==2,"same animation can continue on a new Fixed");
                for (int j=0;j<9;j++)
                {
                    var sig=w.Observe(Sample(4+j,Audit.Phase.Swing,dt),Foot.Left);
                    Check.That(sig==(j*dt<=.05f+1e-6f ? Signal.LeftSwing : Signal.None),"duration "+dt+"/"+j);
                }
            }
            w.Reset();
            for(int i=1;i<=32;i++)
            {
                var sig=w.Observe(Sample(i,Audit.Phase.Lift),Foot.Left);
                Check.That(sig==(i==1?Signal.LeftStart:i<=26?Signal.LeftContinue:Signal.LeftLift),"stuck lift has one bounded origin");
            }
            var near=Sample(33,Audit.Phase.Lift); w.Cut(); near.RawMin=-.002f;
            Check.That(w.Observe(near,Foot.Left)==Signal.LeftLift,"raw boundary is strict");
            near=Sample(34,Audit.Phase.Lift); near.Min=.002f;
            Check.That(w.Observe(near,Foot.Left)==Signal.LeftStart,"actual near boundary is inclusive");
        }

        private static void Breaks()
        {
            for(int mode=0;mode<16;mode++)
            {
                var w=new EarlyLandingWindow(); var first=Sample(1,Audit.Phase.Lift); w.Observe(first,Foot.Left);
                var s=Sample(2,Audit.Phase.Swing);
                switch(mode)
                {
                    case 0: s.FivePoint=false; break;
                    case 1: s.Identity=false; break;
                    case 2: s.FloorCenter.x=.01f; break;
                    case 3: s.SupportId++; break;
                    case 4: s.Fixed=1; break;
                    case 5: s.SourceAge=.101f; break;
                    case 6: s.AnchorGap=.0051f; break;
                    case 7: s.WasActive=true; break;
                    case 8: s.NowActive=true; break;
                    case 9: s.MissInProgress=true; break;
                    case 10: s.SelectedBoxFallback=true; break;
                    case 11: s.Step.Sequence++; break;
                    case 12: s.Source.Time=0; break;
                    case 13: s.RawMin=float.NaN; break;
                    case 14: s.Gait=Audit.Phase.Approach; break;
                    case 15: s.Source=first.Source; s.Source.Dt+=.001f; s.Step.SourceSequence=s.Source.Sequence; break;
                }
                Check.That(w.Observe(s,Foot.Left)==Signal.None && w.Observe(Sample(3,Audit.Phase.Swing),Foot.Left)==Signal.None,"cut "+mode);
            }
            var a=new EarlyLandingWindow(); var left=Sample(1,Audit.Phase.Lift); a.Observe(left,Foot.Left);
            Check.That(a.Observe(left,Foot.Left)==Signal.None && a.Observe(left,Foot.Left)==Signal.None,"high water survives rejection");
            Check.That(EarlyLandingWindow.Side("LEFT")==Foot.Left && EarlyLandingWindow.Side("RIGHT")==Foot.Right && EarlyLandingWindow.Side(null)==Foot.Unknown,"side identity");
        }

        private static void Right()
        {
            var w=new EarlyLandingWindow();
            var s=Sample(1,Audit.Phase.Swing); s.RawMin=-.002f;
            Check.That(w.Observe(s,Foot.Right)==Signal.RightNearLow && w.Origin==0,"one current sample, inclusive box boundary");
            s=Sample(2,Audit.Phase.Swing); s.AnkleHeights.y=.126f;
            Check.That(w.Observe(s,Foot.Right)==Signal.RightSteady,"ordinary ankle denominator");
            s=Sample(3,Audit.Phase.Swing); s.RawMin=.0021f;
            Check.That(w.Observe(s,Foot.Right)==Signal.RightLow,"broad only");
            s=Sample(4,Audit.Phase.Swing); s.SourcePhase="TURN";
            Check.That(w.Observe(s,Foot.Right)==Signal.None,"restrict source to steady");
            s=Sample(5,Audit.Phase.Swing); s.AnkleHeightsValid=false;
            Check.That(w.Observe(s,Foot.Right)==Signal.None,"missing ankle is unknown");
            Check.That(w.Observe(Sample(6,Audit.Phase.Lift),Foot.Right)==Signal.None,"right never uses left onset");
            Check.That(w.Observe(Sample(7,Audit.Phase.Lift),Foot.Unknown)==Signal.None,"unknown foot cannot arm");
            for(int i=8;i<80;i++) w.Observe(Sample(i,Audit.Phase.Swing),Foot.Right);
            long start=GC.GetAllocatedBytesForCurrentThread();
            for(int i=80;i<4000;i++) w.Observe(Sample(i,Audit.Phase.Swing),Foot.Right);
            Check.That(start==GC.GetAllocatedBytesForCurrentThread(),"zero hot-path managed allocation");
        }

        private static void HipBasis()
        {
            var sample=Sample(9,Audit.Phase.Lift);
            var h=new HipNormalReferenceState { Step=sample.Step, Valid=true, Selected=true, Applied=true, ReceiverParentReference=true };
            var p=new HipGoalPathEvidence { Step=sample.Step, Source=sample.Source, Fixed=9, Prepared=true, Committed=true, NormalApplied=true,
                RawUpper=Quaternion.identity, PhysicalParent=Quaternion.identity, ControlParent=Quaternion.identity, RawInControl=Quaternion.identity,
                AfterSource=Quaternion.identity, AfterGovernor=Quaternion.identity, AfterComposite=Quaternion.identity, Final=Quaternion.identity, PhysicalUpper=Quaternion.identity };
            SwingKneeCandidate.HipPlacement Basis(bool handoff=false) =>
                SwingKneeCandidate.EarlyHip(p,h,sample.Step,sample.Source,9,0,handoff);
            Check.That(Basis()==SwingKneeCandidate.HipPlacement.ReceiverReference,"ordinary receiver anchor");
            h.Applied=p.NormalApplied=h.ReceiverParentReference=false; h.Reason="PROTECTED_BYPASS";
            Check.That(Basis(true)==SwingKneeCandidate.HipPlacement.SettledParent,"protected handoff with its own committed rotation");
            h.Reason="STALE_BYPASS"; Check.That(Basis()==SwingKneeCandidate.HipPlacement.Unsupported,"stale bypass");
            h.Reason="PROTECTED_BYPASS"; h.SampleAgePrediction=true;
            Check.That(Basis()==SwingKneeCandidate.HipPlacement.Unsupported,"predicted pose");
            h.SampleAgePrediction=false; p.NormalApplied=true;
            Check.That(Basis()==SwingKneeCandidate.HipPlacement.Missing,"mismatched owner readback");
            p.NormalApplied=false; p.Fixed++;
            Check.That(Basis()==SwingKneeCandidate.HipPlacement.Missing,"different Fixed receipt");
            p.Fixed--; p.Source.Sequence++;
            Check.That(Basis()==SwingKneeCandidate.HipPlacement.Missing,"different source receipt");
        }

        private static string Event(Foot side, bool miss, int first)
        {
            var w=new EarlyLandingWindow(); var history=new LandingEventHistory();
            for(int i=first;i<=10;i++)
            {
                var phase=i==2?Audit.Phase.Lift:i>=6?Audit.Phase.Approach:i==1?Audit.Phase.Stance:Audit.Phase.Swing;
                var s=Sample(i,phase); s.EarlyFoot=side; s.Strong=i>=6;
                if(i==3 || i==4)s.RawMin=.001f;
                s.FirstMiss=miss && i==10; s.DepthLatchFixed=miss?8:0;
                s.NowActive=s.OrdinaryEnabled=!miss&&i==10;
                s.EarlySignal=w.Observe(s,side); s.EarlyOrigin=w.Origin;
                if(EarlyLandingWindow.LeftTrial(s.EarlySignal))s.SwingCandidate.Reason=SwingKneeCandidate.Status.NoRoom;
                history.Add(s,i*.02d,Audit.Screen(s));
                if(i==10){history.Capture(s,miss); if(miss)history.EndMiss(LandingEventHistory.MissEnd.Recovered);}
            }
            var b=new StringBuilder(); history.Append(b); return b.ToString();
        }

        private static void Events()
        {
            var left=Event(Foot.Left,true,1); var right=Event(Foot.Right,true,1);
            Check.That(left.Contains("/outcome=Recovered") && left.Contains("/anchor=DEPTH_LATCH/depthLocal=8/strongStartLocal=6/strongEdge=Swing|STEADY|Approach|STEADY/anchorCaptured=True/anchorLocal=8"),"left origin aligned to true latch");
            Check.That(left.Contains("/leftStarts-trials-favorable=1|4|0/") && left.Contains("/firstLeadMs(leftOrigin-rightLow-rightNearLow)=120|NA|NA"),"four left trials belong to one event");
            Check.That(right.Contains("/anchor=STRONG_EXACT_START/depthLocal=8/strongStartLocal=6/strongEdge=Swing|STEADY|Approach|STEADY/anchorCaptured=True/anchorLocal=6") &&
                right.Contains("/rightLow-nearLow=3|2/firstLeadMs(leftOrigin-rightLow-rightNearLow)=NA|60|60"),"right anchor precedes depth");
            Check.That(Event(Foot.Right,false,1).Contains("side=Right/kind=Ordinary/outcome=Enabled"),"ordinary control included");
            Check.That(Event(Foot.Right,true,6).Contains("strongStartLocal=NA/anchorCaptured=False"),"missing preceding sample is censored");
            Check.That(Event(Foot.Left,true,9).Contains("/anchorCaptured=False"),"latch outside half-second evidence is unknown");
        }
    }
}
