using System;
using System.Text;
using UnityEngine;
using MakemitAGA.ActiveRagdollLab.Physics;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;
using F = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast;
using Seq = MakemitAGA.ActiveRagdollLab.Physics.RightAnkleLandingSequence;
using Mark = MakemitAGA.ActiveRagdollLab.Physics.RightAnkleLandingSequence.Mark;
using Fix = ARL.Contracts.RightFootLandingContracts;

namespace ARL.Contracts
{
    internal static class RightAnkleSequenceContracts
    {
        internal static void Run()
        {
            Check.Run("v153 three independent trajectories match explicit variable-dt PD integration", Trajectories);
            Check.Run("v153 joint gap and signed heights include lost ordinary alignment and terminal torque", Heights);
            Check.Run("v153 planar cap, footprint, missing receipts and original hard gap", Budgets);
            Check.Run("v153 adjacent clocks, source hold, swing reset and strong-edge closure", Continuity);
            Check.Run("v153 ordinary, recovery and final events have comparable prospective denominators", Events);
            Check.Run("v153 multi-step hot path allocates no managed memory per sample", Cost);
        }
        internal static F.Motion Motion()
        {
            var m=Fix.Motion(); m.LinearDrag=.25f;
            m.Tether=new Seq.Tether { SelectedCap=60, BaseCap=60, Kp=400, Kd=40, HardGap=.030f };
            return m;
        }
        private static F.Pose Gap(float x,float y=0,float z=0,bool ordinary=false)
        {
            var p=Fix.Pose(ordinary); var d=new Vector3(x,y,z); p.Box.LowerAnkle=p.Box.FootAnkle-d; p.Gap=d.magnitude;
            return p;
        }
        private static Seq.Readback Tick(Seq q,int i,in F.Pose p,in F.Motion m,in F.Pose next,bool torque=true)
        {
            var r=F.Predict(p,m); var pair=torque?Fix.Pair():default;
            F.Compare(p,m,pair,ref r); q.Issue(i,p,m,r,pair,m.Tether); return q.Read(next,r);
        }
        private static void Trajectories()
        {
            var q=new Seq();var m=Motion(); var pos=new double[3];var v=new double[3];
            for (int i=1;i<=12;i++)
            {
                var p=Gap(.0061f+(i%3)*.0004f,.0014f); p.Box.Dt=i%2==0?.021f:.012f;
                var n=Gap(.0061f+((i+1)%3)*.0004f,.0014f);
                var read=Tick(q,i,p,m,n);Check.That(read.Depth==i && read.Origin==1,"one unbroken cohort");
                for (int k=0;k<3;k++)
                {
                    // Independent absolute virtual acceleration under a fixed external
                    // feedforward cancelling the original spring in this test fixture.
                    double dt=p.Box.Dt, error=p.Box.LowerAnkle.x-p.Box.FootAnkle.x;
                    double spring=400*(1+Seq.Fraction(k))*(error-pos[k]);
                    double acceleration=spring-40*v[k]-400*error;
                    v[k]=(v[k]+acceleration*dt)*(1-.25*dt);pos[k]+=v[k]*dt;
                    var lane=read.Lane(k); double x=n.Box.FootAnkle.x-n.Box.LowerAnkle.x+pos[k];
                    Check.Near(lane.Xz,(float)Math.Abs(x),2e-7f,"unreset shadow velocity");
                    Check.Near(lane.Gap,(float)Math.Sqrt(x*x+read.SignedY*read.SignedY),2e-7f,"full 3D gate");
                }
            }
            // Changing the endpoint changes only the evaluation of this interval.
            var a=new Seq();var b=new Seq();var x0=Gap(.0065f);var n1=Gap(.002f);var n2=Gap(.009f);
            var r1=Tick(a,5,x0,m,n1);var r2=Tick(b,5,x0,m,n2);
            Check.Near(r1.Full.Xz-r1.Xz,r2.Full.Xz-r2.Xz,1e-7f,"no endpoint input into issued command");
        }
        private static void Heights()
        {
            var m=Motion();var q=new Seq();var p=Gap(.0062f);
            float lift=-.002045f-p.Box.Box;
            p.Box.FootAnkle.y+=lift;p.Box.LowerAnkle.y+=lift;p.Com.y+=lift;p.Box.Box+=lift;p.PresentGoal+=lift;
            bool heldOnly=false,cross=false;
            for (int i=1;i<=9;i++)
            {
                var a=Tick(q,i,p,m,p);
                heldOnly|=a.Full.Has(Mark.HeldAndGap);
                cross|=a.Full.Has(Mark.TorqueCross) && !a.OriginalTorqueCross;
                Check.That(!a.Full.Has(Mark.SoleAndGap) && !a.SoleAndGap,"horizontal adjustment never lifts a box");
            }
            Check.That(heldOnly && cross,"accumulated clearance admits an independently bounded one-step ankle correction");
            q.Cut();p=Gap(.0062f,0,0,true);Seq.Readback n=default;
            for (int i=1;i<=8;i++) n=Tick(q,i,p,m,p);
            Check.That(n.Full.Has(Mark.SoleAndGap) && !n.Full.Has(Mark.HeldAndGap) && !n.Full.Has(Mark.TorqueGain),"ordinary sole safe when held goal below");
            var opposite=Gap(-.0044f,0,0,true);n=Tick(q,9,p,m,opposite);
            Check.That(n.SoleAndGap && n.Full.Has(Mark.Measured) && !n.Full.Has(Mark.SoleAndGap),"detect loss of an originally aligned ordinary landing");
            q.Cut();p=Gap(.002f,.0052f);p.Goal=p.Box.FootR;p.PresentGoal=p.Box.Box;
            for(int i=1;i<=10;i++)n=Tick(q,i,p,m,p);
            Check.That(!n.Full.Has(Mark.GapPass) && !n.Full.Has(Mark.HeldAndGap) && !n.Full.Has(Mark.TorqueGain),"vertical gap alone exceeds gate; held target and physical box both low");
            q.Cut();p=Gap(.006f);for(int i=1;i<=10;i++)n=Tick(q,i,p,m,p,false);
            Check.That(n.Full.Has(Mark.HeldAndGap) && !n.Full.Has(Mark.TorqueGain),"missing paired actuator receipt cannot authorize a height gain");
        }
        private static void Budgets()
        {
            var p=Gap(.008f);var m=Motion();var q=new Seq();
            m.Tether.Raw=m.AppliedLinearA=new Vector3(0f,59.999f,0f);
            var n=Tick(q,1,p,m,p);
            float maxHorizontal=(float)Math.Sqrt(60*60f-m.AppliedLinearA.y*m.AppliedLinearA.y);
            Check.That(n.Full.Has(Mark.Clipped) && !n.Full.Has(Mark.GapPass),"horizontal command clipped around preserved real Y");
            Check.Near(n.Xz-n.Full.Xz,maxHorizontal*p.Box.Dt*p.Box.Dt*(1f-m.LinearDrag*p.Box.Dt),2e-7f,"selected sphere retains exact vertical acceleration");
            m.Tether.Raw.x=100;Check.That(Tick(q,2,p,m,p).Depth==0,"actually saturated baseline breaks the whole sequence");
            m=Motion();Check.That(Tick(q,3,p,m,p).Depth==1,"restart from actual after cap cut");
            m.Tether.Kp=float.NaN;Check.That(Tick(q,4,p,m,p).Depth==0,"unavailable owner");
            m=Motion();p=Gap(.031f);n=Tick(q,5,p,m,p);Check.That(n.Full.Has(Mark.HardGap) && !n.Full.Has(Mark.Issued),"original hard gate respected for each lane");
            q.Cut();p=Gap(.006f);var other=p;other.Box.FloorHalf.x=.001f;
            n=Tick(q,6,p,m,other);Check.That(n.Full.Has(Mark.Outside) && !n.Full.Has(Mark.Measured),"observed shifted footprint must fit");
            n=Tick(q,7,p,m,p);Check.That(!n.Full.Has(Mark.Issued),"failed lane cannot resume with stale velocity");
        }
        private static void Continuity()
        {
            for(int fault=0;fault<10;fault++)
            {
                var w=new RightFootLandingWindow();var p=Gap(.006f);var m=Motion();var r=F.Predict(p,m);
                var s=Fix.Sample(1,p);s.Gait=Audit.Phase.Approach;
                w.Observe(s,p,1,2,true);w.Complete(r,m,Fix.Pair());
                var t=Fix.Sample(2,p);t.Gait=Audit.Phase.Approach;
                int foot=1;
                switch(fault)
                {
                    case 1:t.SupportId++;break;
                    case 2:t.SourceAge=.101f;break;
                    case 3:t.FivePoint=false;break;
                    case 4:foot++;break;
                    case 5:t.Source=s.Source;t.Step.SourceSequence=s.Source.Sequence;break;
                    case 6:t.Gait=Audit.Phase.Swing;break;
                    case 7:t.Fixed++;t.Step.Sequence++;break;
                    case 8:t.Strong=true;t.NowActive=true;break;
                    case 9:t.Step.Epoch++;t.Source.Epoch++;break;
                }
                var n=w.Observe(t,p,foot,2,true);
                Check.That((n.Preparation.Depth==1)==(fault==0||fault==5||fault==8),"broken receipt "+fault);
                if(fault==8) Check.That(!w.Pending,"strong edge can close only");
                if(fault==6){w.Complete(r,m,Fix.Pair());var u=Fix.Sample(3,p);var v=w.Observe(u,p,1,2,true);Check.That(v.Preparation.Origin==2 && v.Preparation.Depth==1,"new swing starts cold");}
            }
        }
        private static void Events()
        {
            foreach(string outcome in new[]{"Enabled","Recovered","HoldEnded"})
            {
                var w=new RightFootLandingWindow();var history=new LandingEventHistory();var p=Gap(.0062f,0,0,true);var m=Motion();var r=F.Predict(p,m);
                for(int i=1;i<=10;i++)
                {
                    var s=Fix.Sample(i,p);if(i>=8){s.Gait=Audit.Phase.Approach;s.Strong=true;}
                    if(i==10){s.OrdinaryEnabled=outcome=="Enabled";s.FirstMiss=!s.OrdinaryEnabled;s.DepthLatchFixed=8;}
                    s.RightLandingRead=w.Observe(s,p,1,2,true);history.Add(s,i/60d,Audit.Screen(s),default);
                    if(i==10){history.Capture(s,s.FirstMiss);if(s.FirstMiss) history.EndMiss(outcome=="Recovered"?LandingEventHistory.MissEnd.Recovered:LandingEventHistory.MissEnd.HoldEnded);}
                    if(w.Requested(s.Step))w.Complete(r,m,Fix.Pair());
                }
                var b=new StringBuilder();history.Append(b);var lines=b.ToString().Split('\n');
                string line=Array.Find(lines,l=>l.StartsWith("RIGHT_ANKLE_SEQUENCE_EVENT_"));
                Check.That(line!=null && line.Contains("/outcome="+outcome) && line.Contains("/anchorLocal=8/window-paired-multi-maxDepth-originBeforeHistory=7|7|6|7|0"),"same denominators for "+outcome);
                Check.That(Array.FindAll(lines,l=>l.StartsWith("RIGHT_ANKLE_SEQUENCE_POINT_")).Length==7,"strong edge closes exactly one preceding trial");
            }
        }
        private static void Cost()
        {
            var p=Gap(.006f);var m=Motion();var q=new Seq();
            for(int i=1;i<35;i++) Tick(q,i,p,m,p);
            long allocation=GC.GetAllocatedBytesForCurrentThread();
            for(int i=35;i<200;i++) Tick(q,i,p,m,p);
            Check.That(allocation==GC.GetAllocatedBytesForCurrentThread(),"no new allocations at physics frequency");
            var b=new StringBuilder();q.Append(b);Check.That(b.ToString().Contains("199|1|199|198|0"),"complete aggregate independent of bounded event storage");
        }
    }
}
