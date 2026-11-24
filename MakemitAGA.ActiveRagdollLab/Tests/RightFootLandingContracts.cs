using System;
using System.Text;
using UnityEngine;
using MakemitAGA.ActiveRagdollLab.Physics;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;
using F = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast;
using Angular = MakemitAGA.ActiveRagdollLab.Physics.RightFootTorqueForecast;
using Window = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingWindow;

namespace ARL.Contracts
{
    internal static class RightFootLandingContracts
    {
        internal static void Run()
        {
            Check.Run("v152 asymmetric COM prediction uses issued acceleration, drag and all eight corners", Translation);
            Check.Run("v152 foot-only onset, in-band protection and minimum original-budget correction", Candidates);
            Check.Run("v152 only the adjacent qualified physical step closes the command", Continuity);
            Check.Run("v152 ordinary, recovered and missed events join the preceding foot prediction", Events);
            Check.Run("v152 bounded step detail retains aggregate counts without per-step allocations", Cost);
        }
        // Pose fixtures with the signed heights seen at the 151 B edges. Other body
        // parameters are constructed test inputs; these are not captured native states.
        internal static F.Pose Pose(bool ordinary=false)
        {
            float actual=ordinary?.004047f:-.014930f, goal=ordinary?-.005028f:.001483f;
            var p=new RightKneePivot.Frame { Dt=1f/60f, Half=new Vector3(.043f,.025f,.17f),
                FloorHalf=new Vector3(.8f,.1f,.8f), FloorCenter=new Vector3(0,-.1f,0) };
            p.FootAnkle=new Vector3(.05f,Math.Max(actual,goal)+p.Half.y,.12f);
            p.LowerAnkle=p.FootAnkle-Vector3.right*.001f;
            float lo=0f,hi=25f;
            for(int i=0;i<24;i++)
            {
                float mid=(lo+hi)*.5f;
                if (Angular.Clearance(p,p.FootAnkle,Quaternion.AngleAxis(mid,Vector3.right)) < Math.Min(actual,goal)) hi=mid;
                else lo=mid;
            }
            var tilted=Quaternion.AngleAxis((lo+hi)*.5f,Vector3.right);
            p.FootR=ordinary?Quaternion.identity:tilted; p.Box=Angular.Clearance(p,p.FootAnkle,p.FootR);
            return new F.Pose { Box=p, Goal=ordinary?tilted:Quaternion.identity, PresentGoal=goal, Gap=.001f, Com=p.FootAnkle };
        }
        internal static F.Motion Motion() => new F.Motion { Angular=new Angular.BodyMotion { Mass=.5f,
            Inertia=new Vector3(.012f,.019f,.017f), WorldPrincipal=Quaternion.identity, MaxW=7f } };
        internal static F.Pair Pair() => new F.Pair { LowerMass=1f, LowerLever=.16f, LowerInertia=new Vector3(.09f,.12f,.08f),
            LowerPrincipal=Quaternion.identity };
        internal static Audit.Sample Sample(int n,in F.Pose p)
        {
            var s=SwingKneeContracts.Sample(n,p.Box.Dt,n*p.Box.Dt);
            s.EarlyFoot=EarlyLandingWindow.Foot.Right; s.ParentHeightMeasured=true; s.RightLandingObserved=true;
            s.FloorCenter=p.Box.FloorCenter; s.FloorHalf=p.Box.FloorHalf; s.Min=p.Box.Box; s.TrackingValid=true;
            s.AnkleHeights=new Vector3(.04f,p.Box.LowerAnkle.y,p.Box.FootAnkle.y);
            s.AnchorGap=p.Gap; s.AnkleDeltaY=p.Box.FootAnkle.y-p.Box.LowerAnkle.y; s.ControlMin=p.PresentGoal-s.AnkleDeltaY;
            return s;
        }
        private static float Corners(in F.Pose p, Vector3 com, Quaternion r)
        {
            float min=float.PositiveInfinity;
            for(int i=0;i<8;i++)
            {
                var h=p.Box.Half;
                var corner=new Vector3((i&1)==0?h.x:-h.x,(i&2)==0?h.y:-h.y,(i&4)==0?h.z:-h.z);
                var start=p.Box.FootAnkle+p.Box.FootR*(p.Box.Center+corner);
                var world=com+r*(Quaternion.Inverse(p.Box.FootR)*(start-p.Com));
                min=Math.Min(min,world.y-p.Box.FloorCenter.y-p.Box.FloorHalf.y);
            }
            return min;
        }
        private static void Translation()
        {
            var p=Pose(); var m=Motion();
            p.Com += new Vector3(.017f,.032f,.078f); p.Box.Center=new Vector3(.005f,-.012f,.023f);
            p.Box.Box=Angular.Clearance(p.Box,p.Box.FootAnkle,p.Box.FootR);
            p.PresentGoal=Angular.Clearance(p.Box,p.Box.FootAnkle,p.Goal);
            m.Angular.WorldPrincipal=Quaternion.AngleAxis(42f,new Vector3(.3f,-.1f,.6f).normalized);
            p.W=m.Angular.W=new Vector3(.04f,.34f,-.09f); m.Angular.Transport=new Vector3(.09f,-.2f,.12f);
            m.Angular.Support=new Vector3(.025f,.13f,-.04f); m.Angular.Drive=new Vector3(.01f,-.015f,-.018f);
            p.Velocity=new Vector3(-.3f,.06f,-.17f); m.AppliedLinearA=new Vector3(.74f,8.15f,-3.08f);
            foreach(float dt in new[]{.008f,.01666667f,.033f,.06f})
            foreach(float drag in new[]{0f,.5f,2f,155f})
            {
                p.Box.Dt=dt; m.LinearDrag=drag;m.Angular.Drag=1.2f;
                var r=F.Predict(p,m); var v=(p.Velocity+m.AppliedLinearA*dt)*Math.Max(0f,1f-drag*dt);
                var com=p.Com+v*dt;
                Check.That(r.Predicted,"physical forecast with finite receipt");
                Check.Near((com-r.Com).magnitude,0,1e-6f,"semi-implicit damped physical COM");
                Check.Near(r.Box,Corners(p,com,r.R),2e-6f,"baseline world-space corners");
                Check.Near(r.GoalAtCom,Corners(p,com,p.Goal),2e-6f,"held target about projected COM");
                Check.That(Math.Abs(r.GoalAtFoot-r.GoalAtCom)>.002f,"moving ankle lever survives");
            }
            p.Box.Dt=1f/60f; m.LinearDrag=-1;Check.That(!F.Predict(p,m).Predicted,"negative drag");
            m.LinearDrag=0;m.AppliedLinearA.x=float.NaN;Check.That(!F.Predict(p,m).Predicted,"missing committed accel");
            m.AppliedLinearA=default;p.PresentGoal+=.001f;Check.That(!F.Predict(p,m).Predicted,"wrong held quaternion closure");
        }
        private static void Candidates()
        {
            var p=Pose(); var m=Motion(); var pair=Pair(); var r=F.Predict(p,m);
            Check.Near(r.Box,-.014930f,1e-6f,"2808 geometry"); Check.Near(r.GoalAtFoot,.001483f,1e-6f,"held goal");
            Check.That(r.State==F.Status.MissingDrive,"foot geometry opens opportunity without thigh signal");
            F.Compare(p,m,pair,ref r);
            Check.That(r.State==F.Status.Favorable && r.Level==2 && r.RelativeIncrement==3.75f,"smallest feasible tested level");
            var lowerReaction=HipPointPairMobility.InverseTensor(pair.LowerInertia,pair.LowerPrincipal,-r.DeltaTorque);
            var footReaction=HipPointPairMobility.InverseTensor(m.Angular.Inertia,m.Angular.WorldPrincipal,r.DeltaTorque);
            Check.Near((footReaction-lowerReaction).magnitude,r.RelativeIncrement,1e-5f,"equal and opposite pair");
            Check.That(r.TrialBox-r.Box>=KneeGroundTarget.MinGain && r.TrialBox<-.002f && r.ExtraGapBound>0f,"gain still below band");
            var tilted=pair; tilted.LowerPrincipal=Quaternion.AngleAxis(39f,new Vector3(.6f,.4f,.1f).normalized);
            var spun=m; spun.Angular.WorldPrincipal=Quaternion.AngleAxis(54f,new Vector3(.2f,.65f,.3f).normalized);
            var anis=F.Predict(p,spun); F.Compare(p,spun,tilted,ref anis);
            var rel=HipPointPairMobility.InverseTensor(spun.Angular.Inertia,spun.Angular.WorldPrincipal,anis.DeltaTorque)-
                HipPointPairMobility.InverseTensor(tilted.LowerInertia,tilted.LowerPrincipal,-anis.DeltaTorque);
            Check.That(anis.State==F.Status.Favorable && (rel+Vector3.right*anis.RelativeIncrement).magnitude<2e-5f,
                "rotated asymmetric inertia preserves relative correction direction");
            var edge=p; edge.Box.FloorCenter.y+=edge.Box.Box+.00208f;
            edge.Box.Box=Angular.Clearance(edge.Box,edge.Box.FootAnkle,edge.Box.FootR);
            edge.PresentGoal=Angular.Clearance(edge.Box,edge.Box.FootAnkle,edge.Goal);
            var crossed=F.Predict(edge,m);F.Compare(edge,m,pair,ref crossed);
            Check.That(crossed.EntersBand && crossed.Tried==2 && crossed.BestSafeGain>=.0001f &&
                (crossed.Rejected&F.Reject.Subthreshold)!=0,"partial gain distinguished from newly entering the band");
            var ordinary=Pose(true); var q=F.Predict(ordinary,m);F.Compare(ordinary,m,pair,ref q);
            Check.That(q.State==F.Status.ProtectInBand && q.Level==0 && q.TrialBox==q.Box && q.GoalAtCom<-.002f,"5559 ordinary control protected");
            ordinary.Velocity=Vector3.up*-1f;q=F.Predict(ordinary,m);
            Check.That(q.State==F.Status.ProtectInBand,"already good foot protected even when forecast falls");
            var saturated=pair;saturated.RealizedRelativeA=-Vector3.right*30f;q=F.Predict(p,m);F.Compare(p,m,saturated,ref q);
            Check.That(q.State==F.Status.NoBudget,"relative actuator cap");
            var heavy=m;heavy.Angular.Inertia=new Vector3(2f,2f,2f);var heavyLower=pair;heavyLower.LowerInertia=new Vector3(2f,2f,2f);
            q=F.Predict(p,heavy);F.Compare(p,heavy,heavyLower,ref q);Check.That(q.State==F.Status.NoBudget,"torque cap");
            p.Gap=.0055f;p.Box.LowerAnkle=p.Box.FootAnkle-Vector3.right*p.Gap;q=F.Predict(p,m);F.Compare(p,m,pair,ref q);
            Check.That(q.State==F.Status.OverGap && q.Level==0,"over-five samples remain calibration only");
            p.Gap=.004999f;p.Box.LowerAnkle=p.Box.FootAnkle-Vector3.right*p.Gap;q=F.Predict(p,m);F.Compare(p,m,pair,ref q);
            Check.That(q.State==F.Status.NoGain && (q.Rejected&F.Reject.AnkleGap)!=0 && q.BestSafeGain==0f,"allowance includes lower opposite reaction");
            p=Pose();p.Goal=p.Box.FootR;p.PresentGoal=p.Box.Box;q=F.Predict(p,m);Check.That(q.State==F.Status.GoalOutsideBand,"both too low");
            p=Pose(); p.Box.FloorHalf.x=.005f;q=F.Predict(p,m);Check.That(q.State==F.Status.OutsideFootprint,"projected corners outside support");
        }
        private static void Continuity()
        {
            var p=Pose();var m=Motion();var r=F.Predict(p,m);
            for(int fault=0;fault<=14;fault++)
            {
                var w=new Window();var s=Sample(1,p);s.Gait=Audit.Phase.Stance;s.Strong=false;
                w.Observe(s,p,12,22,true);
                Check.That(w.Requested(s.Step),"weak moving stance is independent of thigh");
                if(fault!=10)w.Complete(r,m);
                var next=Sample(2,p);next.Gait=Audit.Phase.Approach;next.Strong=true;next.NowActive=true;
                int id=12, lowerId=22;var other=p;
                switch(fault)
                {
                    case 1:next.Fixed++;break;
                    case 2:next.Step.Epoch++;next.Source.Epoch++;break;
                    case 3:next.SupportId++;break;
                    case 4:id++;break;
                    case 5:next.FivePoint=false;break;
                    case 6:next.Identity=false;break;
                    case 7:next.SelectedBoxFallback=true;break;
                    case 8:next.SourceAge=.101f;break;
                    case 9:other.Com.x+=.002f;break;
                    case 11:lowerId++;break;
                    case 12:next.Source=s.Source;next.Step.SourceSequence=s.Source.Sequence;break;
                    case 13:other.Box.Dt*=2;break;
                    case 14:lowerId=id;break;
                }
                var read=w.Observe(next,other,id,lowerId,true);bool valid=fault==0||fault==12;
                Check.That((read.From==1)==valid && !w.Pending,"adjacent pre-contact receipt "+fault);
                if(valid)Check.Near(read.Actual,p.Box.Box,1e-7f,"read actual, not a trial");
                Check.That(w.Summary().Contains(valid?"/predicted-read=1|1":fault==10?"/predicted-read=0|0":"/predicted-read=1|0"),"run accounting "+fault);
            }
            var dupe=new Window();var t=Sample(3,p);dupe.Observe(t,p,1,2,true);dupe.Complete(r,m);dupe.Observe(t,p,1,2,true);
            Check.That(!dupe.Pending && dupe.Summary().Contains("/endClock=1"),"duplicate cannot rearm");
            var unknown=Sample(1,p);unknown.Gait=Audit.Phase.Stance;unknown.SourceMotion=Audit.SourceKind.Unknown;
            Check.That(Window.BeforeContact(unknown),"weak stance remains visible if leg receipts are missing");
            var lifted=Sample(1,p);lifted.Gait=Audit.Phase.Lift;Check.That(!Window.BeforeContact(lifted),"release is not a new landing");
            var miss=Sample(1,p);miss.MissInProgress=true;Check.That(!Window.BeforeContact(miss),"late miss excluded");
        }
        private static void Events()
        {
            foreach(string outcome in new[]{"Enabled","Recovered","HoldEnded"})
            {
                var p=Pose(outcome=="Enabled");var w=new Window();var history=new LandingEventHistory();var m=Motion();
                for(int i=1;i<=7;i++)
                {
                    var s=Sample(i,p);
                    if(i>=5){s.Gait=Audit.Phase.Approach;s.Strong=true;}
                    if(i==7){s.OrdinaryEnabled=outcome=="Enabled";s.FirstMiss=!s.OrdinaryEnabled;s.DepthLatchFixed=5;}
                    s.RightLandingRead=w.Observe(s,p,2,3,true);
                    history.Add(s,i/60d,Audit.Screen(s),default);
                    if(i==7){history.Capture(s,s.FirstMiss);if(s.FirstMiss)history.EndMiss(outcome=="Recovered"?LandingEventHistory.MissEnd.Recovered:LandingEventHistory.MissEnd.HoldEnded);}
                    if(w.Requested(s.Step)){var r=F.Predict(p,m);F.Compare(p,m,Pair(),ref r);w.Complete(r,m);}
                }
                var b=new StringBuilder();history.Append(b);
                var line=Array.Find(b.ToString().Split('\n'),q=>q.StartsWith("RIGHT_FOOT_LANDING_EVENT_V041520"));
                Check.That(line!=null && line.Contains("/outcome="+outcome) && line.Contains("/anchorLocal=5/coveredIntervals-read=4|4") &&
                    line.Contains(outcome=="Enabled"?"/geometryOpportunities-over5-protected-goalOutside=0|0|4|0":"/first-lastTrial=1|4/favorable-gainAboveError=4|4"),
                    "untriggered ordinary and missing events align to first strong, including N-1");
            }
        }
        private static void Cost()
        {
            var w=new Window();var p=Pose(true);var m=Motion();var r=F.Predict(p,m);
            void Steps(int first,int end){for(int i=first;i<end;i++){var s=Sample(i,p);w.Observe(s,p,1,2,true);w.Complete(r,m);}}
            Steps(1,64);long bytes=GC.GetAllocatedBytesForCurrentThread();Steps(64,264);
            Check.That(GC.GetAllocatedBytesForCurrentThread()==bytes,"no allocations in live forecast window");
            var summary=w.Summary();
            Check.That(summary.Contains("/ProtectInBand=263|12") && summary.Contains("/predicted-read=263|262") &&
                summary.Contains("/endTraceStop=1") && summary==w.Summary(),"bounded details, exact denominators, idempotent stop");
        }
    }
}
