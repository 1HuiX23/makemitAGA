using System;
using UnityEngine;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using Fixture = ARL.Contracts.LeftFootLeadContracts;
using Clock = ARL.Contracts.LeftFootReleaseContracts;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseForecast;
using Lead = MakemitAGA.ActiveRagdollLab.Physics.LeftFootLeadSequence;
using Motion = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast.Motion;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;
using Prep = MakemitAGA.ActiveRagdollLab.Physics.LeftLegHeightPreparation;
using Shin = MakemitAGA.ActiveRagdollLab.Physics.LeftShinForecast;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseTrace;

namespace ARL.Contracts
{
    internal static class LeftLegPreparationContracts
    {
        private const float Mm = .001f;
        private static Vector3 V(float x, float y, float z) => new Vector3(x,y,z);
        private static void Near(Vector3 a, Vector3 b, string label) => Check.Near((a-b).magnitude,0f,.00001f,label);
        internal static void Run()
        {
            Check.Run("167 measured 166 three-step knee and paired knee/foot trajectories", Logged);
            Check.Run("167 onset, natural rising controls, bounds and rejected perturbations", Guards);
            Check.Run("167 paired foot may leave the floor without censoring its knee-only control", PairedCut);
            Check.Run("167 variable dt and both tethers with delayed measured parent acceleration", Feedback);
            Check.Run("167 overlapping starts keep separate histories and the same verified real landing", Overlap);
            Check.Run("167 cuts, qualified readback on contact and unperturbed rising comparisons", Interruptions);
        }

        private sealed class Case
        {
            internal int N;
            internal Phase Phase;
            internal Foot.Frame Frame;
            internal Foot.Result Foot;
            internal Shin.Result Shin;
            internal Motion Motion;
            internal Prep.Step Predict(Prep.Group group=Prep.Group.Descent, Prep.Step prev=null) =>
                Prep.Predict(N,group,Phase,Frame,Foot,Shin,Motion,prev);
        }

        private static void Anchors(ref Foot.Frame f, int n, Vector3 gap)
        {
            var t = Clock.Tick(n).Step;
            f.Anchors = LegMeasuredAnchors.Capture(true,t,n,t,n,Vector3.up,Vector3.up*.5f,Vector3.up*.5f+gap);
        }

        private static Prep.KneeMotion Knee(Vector3 raw) => new Prep.KneeMotion {
            Raw=raw, Applied=Vector3.ClampMagnitude(raw,60), Kp=400, Kd=40, Drag=.25f, Cap=60, HardGap=.03f
        };

        private static Case Make(int n, Phase phase, float h, Vector3 initialGap, Vector3 initialKnee,
            float dy, Vector3 endGap, Vector3 endKnee, float sole, float ffDY, Vector3 kneeRaw, Vector3 footRaw, float dt=1f/60f)
        {
            var f=Fixture.Frame(n,h*Mm,initialGap*Mm,dt);
            Anchors(ref f,n,initialKnee*Mm);
            var m=Fixture.MotionFor(footRaw); m.MeasuredLowerTransport=true;
            var end=f.Pose.Box.FootAnkle+Vector3.up*(dy*Mm);
            var lo=end-endGap*Mm;
            var k=new Shin.Result { State=Shin.Status.Predicted, Hip=f.Anchors.UpperHip, UpperKnee=f.Anchors.UpperKnee,
                LowerKnee=f.Anchors.UpperKnee+endKnee*Mm, LowerAnkle=lo, FeedforwardCom=Vector3.up*(ffDY*Mm), Knee=Knee(kneeRaw) };
            return new Case { N=n, Phase=phase, Frame=f, Motion=m, Shin=k,
                Foot=Foot.CompareEndpoint(f,phase,m,lo,end,Quaternion.identity,sole*Mm,false) };
        }

        private static Case Example(int n=1) => Make(n,Phase.Lift,-.5f,Vector3.zero,V(2,0,0),
            -.2f,V(3,1,0),V(2,-1,0),-.7f,-.5f,V(1,-8,2),Vector3.zero,.02f);

        private static Foot.Frame Observe(int n, float sole, Vector3 gap, Vector3 knee)
        {
            var f=Fixture.Frame(n,sole*Mm,gap*Mm);
            Anchors(ref f,n,knee*Mm);
            return f;
        }

        private static void Logged()
        {
            // Reconstruct the uncapped native knee vectors from 165-format P/D/FF
            // displacements in the 166 B log. The next spring gives the end knee
            // direction for the first two steps. For the last, only radius/Y survived.
            var fgap=V(.29230118f,-.027552247f,.46336651f);
            var k0=V(-.105381f,1.6028283f,2.1780728f);
            var k1=V(-.2565384f,.2493858f,2.8085706f);
            var k2=V(.1754761f,.6019472f,3.4523006f);
            var g1=V(-.62775612f,2.6417673f,4.6615601f);
            var g2=V(-.21195412f,-.67618489f,3.3320189f);
            var g3=V(.2849102f,.25615096f,3.4153461f);
            var a=Make(3376,Phase.Lift,-.1755241f,fgap,k0,-.2935976f,V(-.62799454f,2.6419015f,4.6615601f),k1,
                -.87563759f,-1.5142512f,V(1.703509f,-9.968410f,-12.54788f),V(-1.8018434f,-.36510608f,-2.6706147f));
            var b=Make(3377,Phase.Lift,-.87566733f,g1,k1,-5.121581f,V(-.2117157f,-.67610294f,3.3321381f),k2,
                -6.1596713f,.32805875f,V(1.073523f,4.3461266f,-6.734289f),V(5.0877953f,-17.825342f,-27.146734f));
            var first=a.Predict();
            Prep.Readback(first,b.Frame,.03f,.03f);
            Check.That(first.Read && first.Decision==Prep.Action.Prepare && !first.TrialPass && first.Combined.Pass,"paired foot closes the first gap");
            Check.Near(first.SeedRise/Mm,.7571256f,.0001f,"half of the available rise protects the ankle");
            Check.Near(first.TrialKnee/Mm,2.83127f,.00012f,"knee sphere at seed");
            Check.Near(first.Combined.Ankle/Mm,4.99892f,.0001f,"combined first step");
            var second=b.Predict(prev:first);
            foreach(float angle in new[] {0f,90f,185f})
            {
                var k3=Quaternion.AngleAxis(angle,Vector3.up)*V(4.0627356f,.06127346f,0);
                var c=Make(3378,Phase.Swing,-6.1596694f,g2,k2,-1.6867518f,V(.2849102f,.25619566f,3.4153461f),
                    k3,-8.0668411f,-.43560407f,V(-1.1707835f,-2.6443976f,-8.572461f),V(-.82913196f,12.42496f,-5.852317f));
                Prep.Readback(second,c.Frame,.03f,.03f);
                Check.Near(second.TrialSole/Mm,-4.81928f,.0002f,"knee-only second sole");
                Check.Near(second.Combined.Sole/Mm,-2.67425f,.0002f,"combined second sole");
                Check.Near(second.Combined.Ankle/Mm,3.49381f,.0002f,"combined second ankle");
                Check.That(second.TrialKnee>second.ActualKnee+.07f*Mm && !second.Combined.Pass,"second knee distance increased");
                var third=c.Predict(prev:second);
                Prep.Readback(third,Observe(3379,-8.0668507f,g3,k3),.03f,.03f);
                Check.Near(third.TrialSole/Mm,-6.80341f,.0002f,"knee-only third sole");
                Check.Near(third.Combined.Sole/Mm,-4.18369f,.00025f,"third-step return under native feedback");
                Check.Near(third.Combined.Ankle/Mm,4.47262f,.00025f,"combined ankle within 5 mm");
                Check.Near(third.BestFixedPose/Mm,-3.323f,.00012f,"optimistic unmodified lower/sole endpoint bound");
                Check.That(!third.Combined.Pass && !third.TrialPass && third.Combined.Vertical==null,"third sole still too low for any terminal knee heading");
            }
        }

        private static void Guards()
        {
            var a=Example();
            Check.That(Prep.Classify(a.Phase,a.Frame,a.Foot,a.Shin)==Prep.Group.Descent &&
                !Lead.Requests(a.Frame,a.Foot,a.Shin),"earlier than old 5 mm prerequisite");
            Check.That(a.Predict().Issued,"clean native receipts");
            foreach(string cut in new[] {"source geometry","unloaded","orientation","witness","start gap","swing","hard knee",
                "no FF","no down","no knee room","warmup","forged cap","exceptional foot","drag","zero cap"})
            {
                var c=Example();
                switch(cut)
                {
                    case "source geometry": c.Frame.Anchors=default; break;
                    case "unloaded": c.Shin.State=Shin.Status.Loaded; break;
                    case "orientation": c.Foot.FootRotation=default; break;
                    case "witness": c.Frame.FivePoint=false; break;
                    case "start gap": c.Frame.Pose.Box.LowerAnkle.x+=.006f; c.Frame.Pose.Gap=.006f; break;
                    case "swing": c.Phase=Phase.Swing; break;
                    case "hard knee": Anchors(ref c.Frame,c.N,V(.0301f,0f,0f)); break;
                    case "no FF": c.Shin.FeedforwardCom=Vector3.zero; break;
                    case "no down": c.Shin.Knee=Knee(Vector3.up); break;
                    case "no knee room": c.Shin.LowerKnee=c.Shin.UpperKnee+Vector3.up*.002f; break;
                    case "warmup": c.Motion.MeasuredLowerTransport=false; break;
                    case "forged cap": c.Shin.Knee.Raw.x+=.2f; break;
                    case "exceptional foot": c.Motion.Tether.SelectedCap=80; c.Motion.Tether.Raw=c.Motion.AppliedLinearA=Vector3.up*62; break;
                    case "drag": c.Motion.LinearDrag=float.NaN; break;
                    case "zero cap": c.Shin.Knee.Cap=0; break;
                }
                Check.That(!c.Predict().Issued,cut);
            }
            var capped=Example();
            capped.Shin.Knee=Knee(V(100,-90,100));
            var s=capped.Predict();
            Check.That(s.Issued && s.KneeA.magnitude<=60f && s.KneeA.y>=capped.Shin.Knee.Applied.y && s.KneeA.y<=0f,"saturated real vector preserved within cap");
            s=Example().Predict();
            var missing=Observe(2,0,V(1,0,0),V(2,0,0)); missing.FivePoint=false;
            Prep.Readback(s,missing,.03f,.03f);
            Check.That(!s.Read,"no inferred contact at missing floor witness");
            var hard=Observe(2,0,V(40,0,0),V(2,0,0));
            Prep.Readback(s,hard,.03f,.03f);
            Check.That(s.ReadHard && !s.TrialPass && !Example(2).Predict(prev:s).Issued,"hard gap aborts paired comparison");
        }

        private static void PairedCut()
        {
            var a=Make(1,Phase.Lift,-.5f,Vector3.zero,V(2,0,0),0,V(0,1,6.2f),V(2,-1,0),
                -.5f,-.4f,V(1,-8,2),Vector3.zero);
            // The solved pair stayed close to the same edge. Its forecast had
            // placed both ankle points 2 mm farther inside this floor.
            a.Frame.Pose.Box.FloorCenter.z=.8798f;
            a.Foot.FootPoint.z+=.002f;
            a.Shin.LowerAnkle.z+=.002f;
            var first=a.Predict();
            Check.That(first.Issued && first.Combined.Horizontal==Lead.Action.Horizontal && first.Combined.Body.Position.z<-.0005f,
                "the paired foot moved toward the lower ankle");
            var b=Example(2);
            b.Frame.Pose.Box.FloorCenter=a.Frame.Pose.Box.FloorCenter;
            Prep.Readback(first,b.Frame,.03f,.03f);
            Check.That(first.Read && first.Inside && first.Combined.End=="ReadFootprint" && !first.Combined.Pass,"paired branch footprint is independent");
            var next=b.Predict(prev:first);
            Check.That(next.Issued && next.Combined==null,"the knee-only comparison still runs");
            Check.That(!b.Predict(Prep.Group.RisingControl,first).Issued,"cannot convert an active descent trial into a natural control");
        }

        private static void Feedback()
        {
            var a=Example(); var first=a.Predict();
            var b=Example(2); b.Frame.Pose.Box.Dt=.013f;
            Prep.Readback(first,b.Frame,.03f,.03f);
            var next=b.Predict(prev:first);
            Check.That(next.Issued && next.Depth==2,"variable dt continuation");
            var dt=b.Frame.Pose.Box.Dt; var k=b.Shin.Knee; var m=b.Motion;
            var expectedK=Vector3.ClampMagnitude(k.Raw-k.Kp*first.Lower.Position-k.Kd*first.Lower.Velocity,k.Cap);
            var expectedFF=first.Lower.Velocity/dt;
            var expectedF=Vector3.ClampMagnitude(m.Tether.Raw+m.Tether.Kp*first.Lower.Position+
                m.Tether.Kd*first.Lower.Velocity+expectedFF,m.Tether.BaseCap);
            Near(next.KneeA,expectedK,"upper position and velocity are the shared hip command");
            Near(next.ParentDeltaA,expectedFF,"this tick's denominator on lower measured velocity");
            Near(next.FootA,expectedF,"the changed lower state reaches both foot PD and transport");
            var c=Example(3); c.Frame.Pose.Box.Dt=.026f;
            Prep.Readback(next,c.Frame,.03f,.03f);
            var third=c.Predict(prev:next);
            Near(third.ParentDeltaA,(next.Lower.Velocity-first.Lower.Velocity)/.026f,"consecutive endpoint velocity difference");
            Check.That(!Example(5).Predict(prev:third).Issued,"no missing read / nonadjacent propagation");
            Prep.Readback(third,c.Frame,.03f,.03f);
            Check.That(!Example(4).Predict(prev:third).Issued,"no fourth step");
        }

        private static void Submit(Trace tr, int n, Phase phase, bool release=false, bool rising=false, float ff=-.5f)
        {
            var t=Clock.Tick(n,phase,release);
            var c=Example(n);
            if (rising) c.Frame.Pose.Velocity.y=.2f;
            tr.Observe(t,c.Frame,true,c.Motion);
            var row=tr.PendingShin(t.Step,t.Source,n);
            Check.That(row!=null,"native continuation at "+n);
            c.Shin.LowerAnkle=row.Command.FootPoint-(rising?Vector3.zero:V(.003f,.001f,0));
            c.Shin.FeedforwardCom=Vector3.up*(ff*Mm);
            tr.CompleteShin(row,c.Shin);
        }

        private static void Overlap()
        {
            var tr=new Trace();
            Submit(tr,10,Phase.Lift,true,ff:0); // A sub-resolution origin cannot suppress the next two.
            Submit(tr,11,Phase.Lift); Submit(tr,12,Phase.Lift);
            Submit(tr,13,Phase.Swing); Submit(tr,14,Phase.Swing);
            tr.Observe(Clock.Tick(15,Phase.Swing),Example(15).Frame,true,Example(15).Motion);
            Check.That(!tr.Sampling && tr.Tracking && tr.WatchingReleaseContact,"only the real landing link remains");
            var miss=Clock.Tick(16,Phase.Stance);miss.Miss=miss.MissInProgress=true;tr.Observe(miss,default,false,default);
            tr.Observe(Fixture.Contact(17,true),default,false,default);
            string s=tr.Summary();
            Check.That(s.Contains("descent-pulses-risingControls=3|2|0/descentRead1-2-3=2|2|2"),"origins are not releases");
            Check.That(s.Contains("origin-depth-from-to=11|3|13|14") && s.Contains("origin-depth-from-to=12|3|14|15"),"separate three-solve trajectories");
            Check.That(s.Contains("/end=BelowResolution/firstLanding-local=FirstMiss|16/firstMissOutcome=Recovered/actualFirstFormal-local=Recovery|17"),"verified real recovery for shared release");
            Check.That(s.Contains("LEFT_FOOT_LEAD_V041660/requests-started=0|0") && s.Contains("/actualFormalOrdinary-recovery=0|0"),"a broader leg window cannot increment the old foot trial");
        }

        private static void Interruptions()
        {
            foreach(string reason in new[] {"Stale","Floor","Clock","ChangedBodyOrBox","ReadFloorWitness","MissingAdjacentReceipt","Loaded","NewLift","TraceEnded"})
            {
                var tr=new Trace();Submit(tr,10,Phase.Lift,true);
                var next=Clock.Tick(11,reason=="NewLift"?Phase.Swing:Phase.Lift);
                var c=Example(11);
                if (reason=="Stale") next.Age=.11f;
                if (reason=="Floor") next.Support++;
                if (reason=="Clock") next.Step.Sequence++;
                if (reason=="ChangedBodyOrBox") c.Frame.LowerId++;
                if (reason=="ReadFloorWitness") c.Frame.FivePoint=false;
                if (reason=="TraceEnded") tr.Seal();
                else tr.Observe(next,c.Frame,reason!="Loaded",c.Motion);
                if (reason=="MissingAdjacentReceipt") tr.Observe(Clock.Tick(12,Phase.Swing),c.Frame,true,c.Motion);
                if (reason=="NewLift") tr.Observe(Clock.Tick(12),c.Frame,true,c.Motion);
                tr.Seal();Check.That(tr.Summary().Contains("/end="+reason+"/"),"window cut "+reason);
            }
            var rising=new Trace();
            Submit(rising,20,Phase.Lift,true,rising:true); Submit(rising,21,Phase.Lift,rising:true);
            Submit(rising,22,Phase.Swing,rising:true);
            rising.Observe(Fixture.Contact(23,false),Example(23).Frame,false,default);
            string text=rising.Summary();
            Check.That(text.Contains("descent-pulses-risingControls=0|0|1/descentRead1-2-3=0|0|0/risingRead1-2-3=1|1|1") &&
                text.Contains("/group=RisingControl/end=ThreeSteps/firstLanding-local=Ordinary|23"),"actual outcome of natural rising control");
            var c0=Make(1,Phase.Lift,-5,Vector3.zero,V(2,0,0),3,V(1,.4f,0),V(2,-1,0),-2,0,V(1,0,0),V(0,2,3));
            var s=c0.Predict(Prep.Group.RisingControl);
            Check.That(s.Decision==Prep.Action.Control && s.Lower.Position==Vector3.zero && s.FootBody.Position==Vector3.zero && s.Combined==null,
                "protect natural rises including below-band starts");
        }
    }
}
