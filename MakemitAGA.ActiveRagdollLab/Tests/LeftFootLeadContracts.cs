using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;
using Fixture = ARL.Contracts.LeftFootReleaseContracts;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseForecast;
using Lead = MakemitAGA.ActiveRagdollLab.Physics.LeftFootLeadSequence;
using Motion = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast.Motion;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;
using Shin = MakemitAGA.ActiveRagdollLab.Physics.LeftShinForecast;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseTrace;

namespace ARL.Contracts
{
    internal static class LeftFootLeadContracts
    {
        private const float Mm = .001f;

        internal static void Run()
        {
            Check.Run("166 two 165 B horizontal misses, measured successor and conditional vertical limit", LoggedPairs);
            Check.Run("166 bounded horizontal ray preserves perpendicular components and never borrows cap", Budget);
            Check.Run("166 seed requires a future gap, descending lower, five points and a valid receipt", Guards);
            Check.Run("166 adjacent solves retain native PD on offset position and velocity with variable dt", Carry);
            Check.Run("166 three unloaded intervals and verified real ordinary or recovered contact", Formal);
            Check.Run("166 floor, body, clock, load, receipt and first-miss continuity", Cuts);
        }

        private static Foot.Frame Frame(int n, float sole, Vector3 gap, float dt = .02f)
        {
            var f = Fixture.Frame(sole);
            f.UpperId = 33;
            f.Pose.Box.Dt = dt;
            f.Pose.Box.LowerAnkle = f.Pose.Box.FootAnkle - gap;
            f.Pose.Gap = gap.magnitude;
            var clock = Fixture.Tick(n).Step;
            clock.Dt = dt;
            f.Anchors = LegMeasuredAnchors.Capture(true,clock,n,clock,n,Vector3.up,
                Vector3.up * .5f,Vector3.up * .5f + Vector3.right * .002f);
            return f;
        }

        private static Motion MotionFor(Vector3 raw, float cap = 60f)
        {
            var m = Fixture.Command();
            m.Tether.Kp = 400f;
            m.Tether.Kd = 40f;
            m.Tether.HardGap = .03f;
            m.Tether.BaseCap = m.Tether.SelectedCap = cap;
            m.Tether.Raw = raw;
            m.AppliedLinearA = Vector3.ClampMagnitude(raw,cap);
            m.LinearDrag = .25f;
            return m;
        }

        private static Shin.Result Lower(in Foot.Frame f, Vector3 ankle) => new Shin.Result {
            State = Shin.Status.Predicted, LowerAnkle = ankle, Hip = f.Anchors.UpperHip,
            UpperKnee = f.Anchors.UpperKnee, LowerKnee = f.Anchors.LowerKnee
        };

        private static Lead.Step WithEndpoint(int n, in Foot.Frame f, in Motion m, Phase phase,
            Vector3 endGap, float footDY, float endSole, Lead.Step previous = null)
        {
            var end = f.Pose.Box.FootAnkle + Vector3.up * footDY;
            var shin = Lower(f,end - endGap);
            var baseline = Foot.CompareEndpoint(f,phase,m,shin.LowerAnkle,end,Quaternion.identity,endSole,false);
            return Lead.Predict(n,phase,f,baseline,shin,m,previous);
        }

        private static void LoggedPairs()
        {
            // The 165 B log contains issued Y, both issued/actual 3D ankle gaps,
            // sole heights and shin displacement. Its raw horizontal force is unknown.
            // Run the recorded geometry with nonbinding XZ headroom in both solves.
            var rows = new[] {
                (n:3454, sole:-.00634969f, startGap:1.1787826f, lowerDY:-1.7424077f, footDY:-.418611f,
                    pred:new Vector3(.4119873f,1.2695715f,5.4913759f), measured:new Vector3(.41174889f,1.2695044f,5.4914951f),
                    endSole:-.52123165f, observed:-.52118504f, y:-1.3724953f, need:.670675f,
                    nextPred:new Vector3(.47230721f,-.29597431f,4.7332048f), nextActual:new Vector3(.47206879f,-.29604137f,4.7329664f),
                    nextLowerDY:-1.3663024f, nextSole:-3.5744267f, nextObserved:-3.5744586f, nextY:-9.1490021f,
                    expectedSole:-2.00898f),
                (n:5047, sole:-.00399992f, startGap:1.1098645f, lowerDY:-2.040118f, footDY:-.467472f,
                    pred:new Vector3(-1.2583733f,1.5835986f,-5.7237148f), measured:new Vector3(-1.2578964f,1.5836209f,-5.7239532f),
                    endSole:-.6498704f, observed:-.64987797f, y:-1.344831f, need:1.117815f,
                    nextPred:new Vector3(-1.260519f,-.45066327f,-4.9302578f), nextActual:new Vector3(-1.2607574f,-.45081973f,-4.9300194f),
                    nextLowerDY:-1.4781207f, nextSole:-4.4115858f, nextObserved:-4.4115686f, nextY:-10.956996f,
                    expectedSole:-2.37728f)
            };
            foreach (var r in rows)
            {
                const float dt = 1f / 60f;
                float startY = r.pred.y - (r.footDY - r.lowerDY);
                float startX = Mathf.Sqrt(r.startGap*r.startGap - startY*startY);
                var f = Frame(r.n,r.sole * Mm,new Vector3(startX,startY,0f) * Mm,dt);
                var m = MotionFor(Vector3.up * r.y);
                var first = WithEndpoint(r.n,f,m,Phase.Lift,r.pred * Mm,r.footDY * Mm,r.endSole * Mm);
                Check.That(first.Decision == Lead.Action.Horizontal && first.Depth == 1,"advance foot on measured heading");
                Check.Near(first.Needed / Mm,r.need,.00005f,"exact horizontal displacement to 5 mm sphere");
                Check.Near(first.TrialGap / Mm,4.999f,.0001f,"one micrometre arithmetic margin");
                Check.Near(first.Applied.y,r.y,1e-6f,"unchanged vertical input at the seed");
                var next = Frame(r.n+1,r.observed * Mm,r.measured * Mm,dt);
                Lead.Readback(first,next);
                Check.That(first.Read && first.TrialPass && !first.ActualPass,"only trial fits early actual ankle and sole");
                float footDY = r.nextLowerDY + r.nextPred.y - r.measured.y;
                var second = WithEndpoint(r.n+1,next,MotionFor(Vector3.up*r.nextY),Phase.Lift,
                    r.nextPred*Mm,footDY*Mm,r.nextSole*Mm,first);
                Lead.Readback(second,Frame(r.n+2,r.nextObserved*Mm,r.nextActual*Mm,dt));
                Check.That(second.Read && second.Vertical == Foot.Decision.Trial && second.ConditionedGap <= 5f*Mm,
                    "lead opens room for the vertical comparison at the next step");
                Check.Near(second.ConditionedSole/Mm,r.expectedSole,.0001f,"measured endpoint plus allowed rise");
                Check.That(!second.TrialPass,"strict minus-two threshold even when missed by under .01 mm");
            }
        }

        private static void Budget()
        {
            var f = Frame(1,0,Vector3.zero);
            var end = new Vector3(.008f,.001f,0);
            foreach (float x in new[] {-58f,-35f,35f,60f})
            {
                float y = x == 60f ? 0f : -9f;
                var m = MotionFor(new Vector3(x,y,2f));
                var s = WithEndpoint(1,f,m,Phase.Lift,end,0f,0f);
                Check.That(s.Issued && s.Applied.magnitude <= 60.00001f && s.Applied.x < m.AppliedLinearA.x,"bounded root");
                Check.Near(s.Applied.y,m.AppliedLinearA.y,0f,"Y invariant");
                Check.Near(s.Applied.z,m.AppliedLinearA.z,0f,"component perpendicular to horizontal request invariant");
                if (x == -58f) Check.That(s.Decision == Lead.Action.Limited && s.TrialGap > .005f && s.Added < s.Needed,
                    "partial actuation is never reported as clearing a gap");
            }
            var full = MotionFor(-Vector3.right*60f);
            Check.That(WithEndpoint(1,f,full,Phase.Lift,end,0f,0f).Decision == Lead.Action.NoRoom,"saturated in request direction");
            var exception = MotionFor(-Vector3.right*59f,80f);
            exception.Tether.BaseCap = 60f;
            var bounded = WithEndpoint(1,f,exception,Phase.Lift,end,0f,0f);
            Check.That(bounded.Decision == Lead.Action.Limited && bounded.Cap == 60f && bounded.Applied.magnitude <= 60.00001f,
                "no borrowing from the exceptional capture cap");
            Check.That(WithEndpoint(1,f,MotionFor(Vector3.zero),Phase.Lift,new Vector3(.003f,.0051f,0),0f,0f).Decision ==
                Lead.Action.VerticalGap,"horizontal translation alone cannot fix vertical excess");
        }

        private static void Guards()
        {
            foreach (string issue in new[] {"startGap","safeEnd","normalLift","alreadyLow","highEnd","phase","floor","dt",
                "drag","hard","cap","kp","raw","actual","footprint"})
            {
                var f = Frame(1,issue=="alreadyLow" ? -.003f : -.0005f,
                    issue=="startGap" ? Vector3.right*.0051f : Vector3.zero);
                var gap = new Vector3(issue=="safeEnd" ? .004f : .006f,.001f,0);
                var m = MotionFor(Vector3.zero);
                float dy = issue=="normalLift" ? .004f : 0f;
                var phase = issue=="phase" ? Phase.Swing : Phase.Lift;
                if (issue=="floor") f.FivePoint = false;
                if (issue=="dt") f.Pose.Box.Dt = float.NaN;
                if (issue=="drag") m.LinearDrag = 100f;
                if (issue=="hard") m.Tether.HardGap = .004f;
                if (issue=="cap") m.Tether.SelectedCap = 0;
                if (issue=="kp") m.Tether.Kp = 0;
                if (issue=="raw") m.Tether.Raw.x = .001f;
                if (issue=="actual") m.AppliedLinearA.x = float.NaN;
                if (issue=="footprint") f.Pose.Box.FloorHalf.x = f.Pose.Box.Half.x;
                var s = WithEndpoint(1,f,m,phase,gap,dy,issue=="highEnd" ? .026f : 0f);
                Check.That(!s.Issued,"reject "+issue);
            }
        }

        private static void Carry()
        {
            var f = Frame(1,-.0005f,Vector3.zero);
            var m = MotionFor(new Vector3(3f,-2f,5f));
            var g = new Vector3(.006f,.001f,0f);
            var first = WithEndpoint(1,f,m,Phase.Lift,g,0f,-.0005f);
            Check.That(!WithEndpoint(2,f,m,Phase.Lift,g,0f,0f,first).Issued,"must measure the adjacent successor first");
            f = Frame(2,-.0005f,g,.017f);
            Lead.Readback(first,f);
            var nextGap = new Vector3(.004f,0,0);
            var second = WithEndpoint(2,f,m,Phase.Lift,nextGap,-.004f,-.0045f,first);
            Check.That(second.Issued && second.Vertical == Foot.Decision.Trial && second.Rise > 0,"bounded second rise");
            float dt = f.Pose.Box.Dt;
            var raw = m.Tether.Raw - m.Tether.Kp*first.Offset - m.Tether.Kd*first.Velocity;
            Check.Near(second.Applied.x,raw.x,1e-5f,"horizontal PD contains both feedback terms");
            Check.Near(second.Applied.z,raw.z,1e-5f,"orthogonal component follows real tether");
            var v = (first.Velocity + (second.Applied-m.AppliedLinearA)*dt) * (1-m.LinearDrag*dt);
            Check.Near((second.Velocity-v).magnitude,0f,1e-6f,"current interval drag");
            Check.Near((second.Offset-first.Offset-v*dt).magnitude,0f,1e-7f,"variable timestep integration");
            f = Frame(3,-.0045f,nextGap,.018f);
            Lead.Readback(second,f);
            var third = WithEndpoint(3,f,m,Phase.Swing,nextGap,0f,-.004f,second);
            var expected = Vector3.ClampMagnitude(m.Tether.Raw - m.Tether.Kp*second.Offset - m.Tether.Kd*second.Velocity,60f);
            Check.That(third.Issued && third.Vertical==null && third.Applied.y < m.AppliedLinearA.y && third.Origin==1,
                "third interval uses native spring and damper on vertical offset as well");
            Check.Near((third.Applied-expected).magnitude,0,1e-5f,"no extra force at depth three");
            Lead.Readback(third,f);
            Check.That(!WithEndpoint(4,f,m,Phase.Swing,nextGap,0f,-.004f,third).Issued,"three intervals maximum");
        }

        private static Trace Start()
        {
            var tr = new Trace();
            var t = Fixture.Tick(1,Phase.Lift,true);
            var f = Frame(1,-.0005f,Vector3.zero);
            tr.Observe(t,f,true,MotionFor(Vector3.zero));
            Complete(tr,t,f,new Vector3(.006f,.001f,0));
            Check.That(tr.WatchingLeadFormal,"prospectively armed");
            return tr;
        }

        private static void Complete(Trace tr, in Trace.Tick t, in Foot.Frame f, Vector3 endGap)
        {
            var r = tr.PendingShin(t.Step,t.Source,t.Local);
            Check.That(r!=null,"fresh uncommitted row");
            tr.CompleteShin(r,Lower(f,r.Command.FootPoint-endGap));
            Check.That(tr.PendingShin(t.Step,t.Source,t.Local)==null,"receipt consumed exactly once");
        }

        private static Trace ThreeSteps()
        {
            var tr = Start();
            var t = Fixture.Tick(2);
            var f = Frame(2,-.0005f,new Vector3(.006f,.001f,0));
            tr.Observe(t,f,true,MotionFor(-10f*Vector3.up));
            Complete(tr,t,f,new Vector3(.004f,0f,0));
            t = Fixture.Tick(3,Phase.Swing);
            f = Frame(3,-.00448f,new Vector3(.004f,0,0));
            tr.Observe(t,f,true,MotionFor(Vector3.zero));
            Check.That(tr.Sampling,"keep final read even in first swing");
            Complete(tr,t,f,new Vector3(.004f,0,0));
            tr.Observe(Fixture.Tick(4,Phase.Swing),f,true,MotionFor(Vector3.zero));
            Check.That(!tr.Sampling && tr.Tracking && tr.Summary().Contains("/sequenceEnd=ThreeSteps") &&
                tr.Summary().Contains("/pairedAt1-2-3=1|1|1/verticalTrials=1"),"three adjacent unloaded solves then light event tracking");
            return tr;
        }

        private static Trace.Tick Contact(int local, bool recovery)
        {
            var t = Fixture.Tick(local,Phase.Stance);
            t.Active = true; t.Ordinary = !recovery; t.Recovery = recovery;
            t.FormalFoot = 11; t.FormalLower = 22; t.FormalUpper = 33;
            return t;
        }

        private static void Formal()
        {
            var tr = ThreeSteps();
            tr.Observe(Contact(5,false),default,false,default);
            Check.That(!tr.Tracking && tr.Summary().Contains("firstLanding-local=Ordinary|5/firstMissOutcome=NA/actualFirstFormal-local=Ordinary|5") &&
                tr.Summary().Contains("/actualFormalOrdinary-recovery=1|0/plantWrites=0"),"original ordinary entry and monitored body match");
            foreach (bool retained in new[] {true,false})
            {
                tr = ThreeSteps();
                var t = Fixture.Tick(5,Phase.Stance);
                t.Miss = t.MissInProgress = true;
                tr.Observe(t,default,false,default);
                t = Fixture.Tick(6,Phase.Stance); t.MissInProgress = retained;
                tr.Observe(t,default,false,default);
                Check.That(tr.WatchingLeadFormal && !tr.Sampling,"wait without capturing new commands");
                tr.Observe(Contact(7,retained),default,false,default);
                string valid = retained ? "Recovered/actualFirstFormal-local=Recovery|7" : "HoldEnded/actualFirstFormal-local=Ordinary|7";
                Check.That(tr.Summary().Contains("firstLanding-local=FirstMiss|5/firstMissOutcome="+valid),"first miss survives later contact");
            }
        }

        private static void Cuts()
        {
            foreach (string cause in new[] {"Clock","Floor","Stale","ChangedBodyOrBox","ReadFloorWitness","ReadFootprint","ReadHardGap","Loaded",
                "MissingCommand","MissingAdjacentReceipt","MissingPose","TraceEnded"})
            {
                var tr = Start();
                var t = Fixture.Tick(2);
                var f = Frame(2,-.0005f,new Vector3(.006f,.001f,0));
                var m = MotionFor(Vector3.zero);
                if (cause=="Clock") { t.Step.Epoch++; t.Source.Epoch++; }
                if (cause=="Floor") t.Support++;
                if (cause=="Stale") t.Age = .101f;
                if (cause=="ChangedBodyOrBox") f.UpperId++;
                if (cause=="ReadFloorWitness") f.FivePoint = false;
                if (cause=="ReadFootprint")
                {
                    f.Pose.Box.FootAnkle.x = f.Pose.Box.LowerAnkle.x = -.9595f;
                    f.Pose.Box.LowerAnkle.x -= .006f;
                    f.Pose.Com.x = f.Pose.Box.FootAnkle.x;
                }
                if (cause=="ReadHardGap") f = Frame(2,-.0005f,Vector3.right*.032f);
                if (cause=="MissingCommand") m = default;
                if (cause=="MissingPose") f = default;
                if (cause=="TraceEnded") tr.Seal();
                else tr.Observe(t,f,cause!="Loaded",m);
                if (cause=="MissingAdjacentReceipt") tr.Observe(Fixture.Tick(3,Phase.Swing),f,true,m);
                tr.Seal();
                Check.That(tr.Summary().Contains("/sequenceEnd="+cause+"/"),"interrupted lead: "+cause);
            }
            foreach (string cause in new[] {"foot","lower","upper","support","missing hold","new lift","new swing","horizon"})
            {
                var tr = ThreeSteps();
                var t = Fixture.Tick(5,Phase.Stance); t.Miss=t.MissInProgress=true;
                tr.Observe(t,default,false,default);
                t = Contact(6,true);
                if (cause=="foot") t.FormalFoot++;
                if (cause=="lower") t.FormalLower++;
                if (cause=="upper") t.FormalUpper++;
                if (cause=="support") t.EventSupport++;
                if (cause=="missing hold") { t.Recovery=t.Active=false; }
                if (cause=="new lift") t=Fixture.Tick(6,Phase.Lift);
                if (cause=="new swing") t=Fixture.Tick(6,Phase.Swing);
                if (cause=="horizon") { t=Fixture.Tick(6,Phase.Stance); t.MissInProgress=true; }
                tr.Observe(t,default,false,default);
                if (cause=="horizon")
                    for (int i=7;i<110;i++) tr.Observe(Fixture.Tick(i,Phase.Stance),default,false,default);
                string s = tr.Summary();
                if (cause=="missing hold") Check.That(s.Contains("firstMissOutcome=HoldEnded"),"closed latch");
                else if (cause=="new lift") Check.That(s.Contains("actualFirstFormal-local=NewLift|0"),"next gait cycle cannot claim contact");
                else if (cause=="new swing") Check.That(s.Contains("actualFirstFormal-local=NewSwing|0"),"direct stance to swing also starts a new cycle");
                else if (cause=="horizon") Check.That(s.Contains("actualFirstFormal-local=Horizon|0"),"bounded event tracking");
                else Check.That(s.Contains("firstMissOutcome=UnverifiedRecovery/actualFirstFormal-local=UnverifiedFormal|0") &&
                    s.Contains("actualFormalOrdinary-recovery=0|0"),"invalid formal receipt: "+cause);
            }
            var immediate = Start();
            immediate.Observe(Contact(2,false),Frame(2,-.0005f,new Vector3(.006f,.001f,0)),false,default);
            Check.That(immediate.Summary().Contains("/pairedAt1-2-3=1|0|0") &&
                immediate.Summary().Contains("actualFirstFormal-local=Ordinary|2"),"settle last free solve before the upcoming contact");
        }
    }
}
