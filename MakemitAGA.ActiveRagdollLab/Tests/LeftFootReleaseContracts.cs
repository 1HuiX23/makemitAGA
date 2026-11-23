using System;
using UnityEngine;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using F = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseForecast;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseTrace;
using Motion = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast.Motion;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;

namespace ARL.Contracts
{
    internal static class LeftFootReleaseContracts
    {
        private const float Dt = .02f;
        internal static void Run()
        {
            Check.Run("164 selectors and the disabled native-gravity receipt", Selectors);
            Check.Run("164 issued vs bounded vertical tether at the identical state", Budget);
            Check.Run("164 normals, floor, pure common descent, drag, gap and source guards", Boundaries);
            Check.Run("164 prospective decision, actual successor and ordinary/first-miss joins", Events);
            Check.Run("164 discontinuities never settle a stale solve, re-release and bounded details", Cuts);
            Check.Run("164 knee candidates retain measured hip, both joint anchors and committed knee limits", Knee);
            Check.Run("164 measured displacement budget replay of ten 163 left swing onsets", LoggedGeometry);
            Check.Run("164 measured hip/knee points survive compact active capture", MeasuredAnchors);
        }

        internal static F.Frame Frame(float height = -.0005f)
        {
            var foot = new Vector3(0,.025f + height,0);
            var box = new RightKneePivot.Frame { Dt = Dt, FootR = Quaternion.identity, FootAnkle = foot, LowerAnkle = foot,
                Half = new Vector3(.04f,.025f,.12f), FloorHalf = new Vector3(1,.1f,1), FloorCenter = new Vector3(0,-.1f,0) };
            box.Box = RightFootTorqueForecast.Clearance(box, foot, box.FootR);
            return new F.Frame { Pose = new RightFootLandingForecast.Pose { Box = box, Com = foot, Goal = box.FootR, PresentGoal = box.Box },
                FootId = 11, LowerId = 22, FivePoint = true, HipY = 1f, KneeY = .5f, SourceAnkleY = .025f };
        }
        internal static Motion Command(float y = -10f)
        {
            var m = RightFootLandingContracts.Motion();
            m.AppliedLinearA = new Vector3(0,y,0);
            m.Tether = new RightAnkleLandingSequence.Tether { Raw = m.AppliedLinearA, BaseCap = 60f, SelectedCap = 60f };
            return m;
        }
        internal static Trace.Tick Tick(int i, Phase gait = Phase.Lift, bool release = false)
        {
            var s = SwingKneeContracts.Sample(i,Dt,i*Dt);
            return new Trace.Tick { Step = s.Step, Source = s.Source, Local = i, Support = 7, ReleasedSupport = release ? 7 : 0,
                EventSupport = 7, Age = .01f, Gait = gait, Released = release, Motion = "TURN", ReleaseReason = "RELEASE" };
        }
        private static void Selectors()
        {
            var timing = new LandingObservationTiming();
            foreach (var arm in new[] { PelvisDynamicReceiverIsolation.DynamicHipArmA, PelvisDynamicReceiverIsolation.DynamicHipArmB, "legacy", null })
            {
                timing.Begin(arm);
                Check.That(timing.LeftReleaseProbe == PelvisDynamicReceiverIsolation.IsDynamicHipArm(arm), "both formal arms observe the left release");
                Check.That(RightFootPrecontact.ForArm(arm) == (timing.Running ? RightFootPrecontact.Mode.PreviousStepExpiryReview :
                    RightFootPrecontact.Mode.Off), "163 B right baseline");
                timing.Seal(); Check.That(!timing.LeftReleaseProbe, "sealed");
            }
            Check.That(F.UnloadedFoot(false,false,false,false,0,false), "native off receipt has scheduledFixed zero");
            for (int i=0; i<6; i++) Check.That(!F.UnloadedFoot(i==0,i==1,i==2,i==3,i==4 ? 17 : 0,i==5), "no active or partial readback");
        }
        private static void Budget()
        {
            var f = Frame(); var m = Command(); m.AppliedLinearA.x = 2; m.Tether.Raw = m.AppliedLinearA;
            var r = F.Compare(f, Phase.Lift, true, m);
            Check.That(r.State == F.Decision.Trial, "descent admitted");
            Check.Near(r.Sole,-.0045f,1e-6f,"original next sole");
            Check.Near(r.Rise,.004f,1e-6f,"cancel only original downward acceleration");
            Check.Near(r.TrialY,0,1e-5f,"no upward push");
            var trial = m; trial.AppliedLinearA.y = r.TrialY;
            var baseline = RightFootLandingForecast.Predict(f.Pose,m);
            var alternative = RightFootLandingForecast.Predict(f.Pose,trial);
            Check.Near(alternative.Box-baseline.Box,r.Rise,1e-6f,"same forecast using actual full commands");
            Check.Near(alternative.Foot.x,baseline.Foot.x,1e-7f,"horizontal unchanged");
            Check.Near(m.AppliedLinearA.y,-10,0,"no controller write");
            var n=Frame(r.Sole);
            Check.That(F.ReadTrial(r,n,out var sole,out var gap) && sole>-.002f && gap<=.005f, "conditioned threshold and gap");
            n=Frame(.001f); Check.That(!F.ReadTrial(r,n,out _,out _),"ordinary rise gets no credit");
            n=Frame(-.018f); Check.That(!F.ReadTrial(r,n,out _,out _),"forecast benefit does not certify observed too-deep endpoint");
            n=Frame(r.Sole); n.Pose.Box.LowerAnkle.x = .006f; n.Pose.Gap=.006f;
            Check.That(!F.ReadTrial(r,n,out _,out gap) && gap>.005f, "actual gap rejects candidate");
            f.LowerVelocity=-Vector3.up*.08f; r=F.Compare(f,Phase.Lift,true,m);
            Check.Near(r.Rise,.0024f,1e-6f,"common lower motion retained"); Check.That(r.TrialY<0,"partial suppression");
        }

        private static void Boundaries()
        {
            var f=Frame(); var m=Command();
            Check.That(F.CanCompareKnee(f,Phase.Lift) && !F.CanCompareKnee(f,Phase.Swing), "knee compares at lift, including one preceding a natural rise");
            Check.That(!F.CanCompareKnee(Frame(-.003f),Phase.Lift),"already below ordinary threshold");
            Check.That(F.Compare(f,Phase.Lift,false,m).State==F.Decision.Loaded,"load");
            Check.That(F.Compare(f,Phase.Swing,true,m).State==F.Decision.OtherPhase,"normal swing protected");
            f.FivePoint=false; Check.That(!F.CanCompareKnee(f,Phase.Lift), "missing same-floor knee witness"); Check.That(F.Compare(f,Phase.Lift,true,m).State==F.Decision.NoFloorWitness,"physical sample still measurable");
            f=Frame(.015f); Check.That(F.Compare(f,Phase.Lift,true,m).State==F.Decision.Clear,"clearance preserved");
            f=Frame(-.004f); Check.That(F.Compare(f,Phase.Lift,true,m).State==F.Decision.OutsideStartBand,"new deep placement excluded");
            f=Frame(); f.LowerVelocity=-Vector3.up; Check.That(F.Compare(f,Phase.Lift,true,m).State==F.Decision.CommonDescent,"parent leads descent");
            f=Frame(); f.Pose.Box.LowerAnkle.x=.0051f; f.Pose.Gap=.0051f;
            Check.That(F.Compare(f,Phase.Lift,true,m).State==F.Decision.StartGap,"old gap already over 5 mm");
            f=Frame(); f.LowerVelocity.x=1; Check.That(F.Compare(f,Phase.Lift,true,m).State==F.Decision.Gap,"no vertical solution to lateral gap");
            f=Frame(); f.Pose.Velocity.y=-.25f; m=Command(1);
            Check.That(F.Compare(f,Phase.Lift,true,m).State==F.Decision.NoDownwardTether,"downward motion despite braking");
            f=Frame(); m=Command(-70); m.Tether.SelectedCap=80;
            Check.That(F.Compare(f,Phase.Lift,true,m).State==F.Decision.ProtectedBudget,"exceptional cap");
            m=Command(); m.Tether.SelectedCap=float.PositiveInfinity;
            Check.That(F.Compare(f,Phase.Lift,true,m).State==F.Decision.ProtectedBudget,"nonfinite cap");
            m=Command(); m.Tether.Raw.x=2;
            Check.That(F.Compare(f,Phase.Lift,true,m).State==F.Decision.ProtectedBudget,"actual raw clamp closure");
            m=Command(); m.LinearDrag=5; var r=F.Compare(f,Phase.Lift,true,m);
            Check.Near(r.Rise,.0036f,1e-6f,"use same drag as original solver forecast");
            f.SourceAnkleY=float.NaN; Check.That(F.Compare(f,Phase.Lift,true,m).State==F.Decision.MissingPose,"invalid pose");
        }

        private static void Events()
        {
            var tr=new Trace(); var f=Frame(); var m=Command();
            f.KneeExamined=true; f.Knee.Reason=SwingKneeCandidate.Status.NoRoom;
            tr.Observe(Tick(10,Phase.Lift,true),f,true,m);
            var n=Frame(-.0045f); n.KneeY-=.002f;
            tr.Observe(Tick(11,Phase.Swing),n,true,m);
            Check.That(!tr.Sampling && tr.Tracking,"end expensive reads at first swing");
            for (int i=12; i<70; i++) tr.Observe(Tick(i,Phase.Swing),default,false,default);
            var t=Tick(70,Phase.Stance); t.Active=t.Ordinary=true;
            tr.Observe(t,default,false,default);
            string txt=tr.Summary();
            Check.That(txt.Contains("settled-modeled-trials-pairedTrials-conditionedCross=1|1|1|1|1|1") &&
                txt.Contains("kneeExamined-favorable=1|0") && txt.Contains("/kneePose=NoRoom") &&
                txt.Contains("/exit=FirstSwing/firstLanding-local=Ordinary|70") && txt.Contains("from-to=10|11"),"all reads linked");
            tr.Observe(Tick(71,Phase.Lift,true),f,true,m); tr.Observe(Tick(72,Phase.Swing),n,true,m);
            t=Tick(73,Phase.Stance); t.Miss=true;
            tr.Observe(t,default,false,default); t=Tick(74,Phase.Stance);t.Recovery=t.Active=true;tr.Observe(t,default,false,default);
            Check.That(tr.Summary().Contains("firstLanding-local=FirstMiss|73") && !tr.Tracking,"later recovery cannot replace first miss");
            tr=new Trace();tr.Observe(Tick(1,Phase.Lift,true),f,true,m);
            t=Tick(2,Phase.Stance);t.Active=t.Ordinary=true;t.EventSupport=0;
            tr.Observe(t,n,false,default);
            Check.That(tr.Summary().Contains("/exit=Formal/firstLanding-local=UnverifiedFloor|2"),"read last free solve before upcoming new contact");
        }

        private static void Cuts()
        {
            foreach (string cause in new[] {"Clock","Skipped","Epoch","SourceReuse","Floor","Stale","ChangedBodyOrBox","Moved","MissingPose","Gait"})
            {
                var tr=new Trace();var f=Frame();var t=Tick(8,Phase.Lift,true);tr.Observe(t,f,true,Command());
                var n=Tick(9,Phase.Swing);
                switch(cause)
                {
                    case "Clock":n.Local++;break;
                    case "Skipped":n.Step.Sequence++;break;
                    case "Epoch":n.Step.Epoch++;n.Source.Epoch++;break;
                    case "SourceReuse":n.Source=t.Source;n.Step.SourceSequence=n.Source.Sequence;n.Source.Time+=.001f;break;
                    case "Floor":n.Support++;break;
                    case "Stale":n.Age=.101f;break;
                    case "ChangedBodyOrBox":f.LowerId++;break;
                    case "Moved":f.Pose.Box.FloorCenter.x+=.1f;break;
                    case "MissingPose":f=default;break;
                    case "Gait":n.Gait=Phase.Unknown;break;
                }
                tr.Observe(n,f,true,Command());tr.Seal();tr.Seal();
                Check.That(tr.Summary().Contains("settled-modeled-trials-pairedTrials-conditionedCross=1|0|1|1|0|0"),cause);
            }
            var bounded=new Trace();
            bounded.Observe(Tick(1,Phase.Lift,true),Frame(),true,Command());
            for (int i=2;i<=115;i++) bounded.Observe(Tick(i),Frame(),true,Command());
            Check.That(!bounded.Tracking && bounded.Summary().Contains("/exit=WindowLimit/firstLanding-local=Horizon|0"),"rate-independent read and linkage bounds");
            bounded=new Trace();bounded.Observe(Tick(1,Phase.Lift,true),Frame(),true,Command());
            bounded.Observe(Tick(2,Phase.Swing),Frame(),true,Command());bounded.Observe(Tick(3,Phase.Lift),Frame(),true,Command());
            Check.That(!bounded.Tracking && bounded.Summary().Contains("firstLanding-local=NewLift|0"),"new lift cannot claim first landing");
            bounded=new Trace();bounded.Observe(Tick(1,Phase.Lift,true),Frame(),true,Command());
            var sameSource=Tick(2,Phase.Swing);sameSource.Source=Tick(1).Source;sameSource.Step.SourceSequence=sameSource.Source.Sequence;
            bounded.Observe(sameSource,Frame(),true,Command());bounded.Seal();
            Check.That(bounded.Summary().Contains("from-to=1|2/gait=Lift|Swing/source=1|1"),"held source with adjacent physics is valid");
            var many=new Trace();
            for (int i=1; i<=70; i++) many.Observe(Tick(i,Phase.Lift,true),Frame(),true,Command());
            many.Seal();
            Check.That(many.Summary().Contains("releases-saved-omitted=70|64|6") && many.Summary().Contains("from-to=1|0"),"censor and conserve after detail cap");
        }

        private static void Knee()
        {
            var random=new Random(164);
            int favorable=0;
            for (int i=0;i<120;i++)
            {
                var fixture=LandingPreparationFixtures.Make(random);var p=SwingKneeContracts.FixturePose(fixture);var k=fixture.Knee;
                var g=p.Reference;var lower=p.PhysicalKnee+p.PhysicalLower*g.LowerSpan;
                var box=new RightKneePivot.Frame { Box=p.ObservedBox, LowerAnkle=lower, FootAnkle=lower+p.AnkleGap,
                    FootR=p.PhysicalLower*p.PhysicalFootRelative, Center=g.Center, Half=g.Half, FloorCenter=g.FloorCenter, FloorHalf=g.FloorHalf };
                var anchors=LegMeasuredAnchors.Capture(true,k.Step,12,k.Step,12,g.Hip,g.Hip+p.PhysicalUpper*g.UpperSpan,p.PhysicalKnee);
                var original=SwingKneeCandidate.Evaluate(p,k,k.Step,k.Source,measureRoom:true);
                var actual=F.CompareKnee(box,anchors,p,k,k.Step,k.Source);
                Check.That(actual.Reason==original.Reason,"same original legal reference lanes");
                if (actual.Reason!=SwingKneeCandidate.Status.Favorable) continue;
                favorable++;
                Check.Near(actual.AnkleGain,original.AnkleGain,.00001f,"measured lower pivot");
                Check.That(actual.CandidateW<=actual.LimitW+.0001f && actual.CandidateA<=actual.LimitA+.0001f,"original W/A limits");
                var stale=k;stale.Stale=true;
                Check.That(F.CompareKnee(box,anchors,p,stale,k.Step,k.Source).Reason==SwingKneeCandidate.Status.Protected,"protected owner");
            }
            Check.That(favorable>0,"captures actual favorable fixtures as well as rejections");
        }

        private static void LoggedGeometry()
        {
            // A then B. All distances copied/derived from adjacent 163 H144 columns in mm.
            var edges = new (int From, float StartGap, float Sole, float Lower, float Foot, float Gap, float GapY)[]
            {
                (2581, 4.202320f, -5.692504f, -2.776690f, -5.401360f, 4.401283f, 0.229390f),
                (3348, 4.998274f, -6.588019f, -1.753270f, -5.976740f, 3.209788f, -1.209720f),
                (3985, 3.385254f, -5.095452f, -2.624390f, -4.553240f, 2.935321f, 0.460610f),
                (4597, 4.956978f, -5.674921f, -2.776680f, -5.322240f, 4.244863f, 0.119550f),
                (5319, 4.581924f, -6.074779f, -3.051050f, -5.681410f, 4.447616f, 0.350390f),
                (2579, 3.788728f, -4.963472f, -2.699970f, -4.631720f, 4.592763f, 0.581510f),
                (3430, 4.025027f, -2.235450f, -1.197930f, -2.026140f, 4.170606f, 0.131220f),
                (4176, 3.937129f, -4.128449f, -2.458390f, -3.771290f, 4.410291f, 0.536390f),
                (4937, 5.734837f, -4.832394f, -1.086710f, -3.267950f, 6.421669f, -0.976060f),
                (5582, 4.673718f, -3.703102f, -2.027330f, -3.176430f, 4.882893f, 0.199530f),
            };
            foreach (var e in edges)
            {
                bool fits = F.MeasuredRoom(e.StartGap*.001f,e.Sole*.001f,e.Lower*.001f,e.Foot*.001f,
                    e.Gap*.001f,e.GapY*.001f,out float need,out float excess,out float gapAtDepth);
                Check.That(fits == (e.From == 3430), "held-XZ and measured excess at " + e.From);
                Check.That(need > 0f && excess > 0f && float.IsFinite(gapAtDepth), "finite measured geometry");
            }
        }

        private static void MeasuredAnchors()
        {
            var t=Tick(4);
            var leg=LegMeasuredAnchors.Capture(true,t.Step,t.Local,t.Step,t.Local,Vector3.up,Vector3.up*.5f,Vector3.up*.49f);
            Check.That(leg.Valid, "measured points available without reference input");
            Check.Near(leg.LowerKnee.y,.49f,1e-6f,"lower-side physical knee");
            Check.That(!LegMeasuredAnchors.Capture(true,t.Step,t.Local,t.Step,5,Vector3.up,Vector3.up,Vector3.up).Valid,"wrong local");
            Check.That(!LegMeasuredAnchors.Capture(false,t.Step,t.Local,t.Step,t.Local,Vector3.up,Vector3.up,Vector3.up).Valid,"incomplete leg control");
            Check.That(!LegMeasuredAnchors.Capture(true,t.Step,t.Local,Tick(5).Step,t.Local,Vector3.up,Vector3.up,Vector3.up).Valid,"wrong shared step");
            Check.That(!LegMeasuredAnchors.Capture(true,t.Step,t.Local,t.Step,t.Local,Vector3.up,Vector3.up,Vector3.up*float.NaN).Valid,"bad anchor");
        }
    }
}
