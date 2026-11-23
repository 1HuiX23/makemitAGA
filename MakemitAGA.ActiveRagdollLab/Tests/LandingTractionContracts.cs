using System;
using System.Globalization;
using System.IO;
using System.Linq;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LandingFootTraction;
using Trial = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;

namespace ARL.Contracts
{
    internal static class LandingTractionContracts
    {
        internal static void Run()
        {
            Check.Run("171 replay of four real 170 B releases reproduces 66 choices and keeps braking room", Recorded);
            Check.Run("171 joint rise and planar closure on the 3389 geometry use one bounded vector command", Traction);
            Check.Run("171 translated foot keeps its private spring, velocity and drag across consecutive commands", Continuation);
            Check.Run("171 rolling one- and two-solve proposals terminate at the same first native opportunity", TraceWindows);
            Check.Run("171 previous braking witnesses and held ground input have independent clocks and units", ReceiptContext);
            Check.Run("171 hip socket, selected rotations, material binding and follow error close the native shoe path", Heights);
        }

        private static float F(string s) => float.Parse(s,CultureInfo.InvariantCulture);
        private static void Recorded()
        {
            using var reader = new StreamReader(typeof(LandingTractionContracts).Assembly.GetManifestResourceStream("ARL.Contracts.TurnLanding170"));
            var offsets = new Trial.Offset[3]; var stopped = new bool[3];
            int rows = 0, origin = 0, choices = 0, endpoints = 0, reserveRead = 0, newThird = 0;
            reader.ReadLine(); string line;
            while ((line = reader.ReadLine()) != null)
            {
                var cells = line.Split('\t');
                int start = int.Parse(cells[0]), depth = int.Parse(cells[1]);
                if (start != origin) { Array.Clear(offsets,0,3); Array.Clear(stopped,0,3); origin = start; }
                var s = TurnLandingContracts.DecodeIssue(cells);
                rows++;
                for (int lane = 0; lane < 3; lane++)
                {
                    if (stopped[lane])
                    {
                        if (lane < 2) Check.That(cells[15+2*lane] == "NA", "a dead lane stays unavailable");
                        continue;
                    }
                    var p = Trial.Select(s.Pose,s.Issue,s.Budget,s.Dt,s.HipY,offsets[lane],lane != 0,lane == 2);
                    if (lane < 2) { Check.That(p.Result.ToString() == cells[15+2*lane], "historical decision"); choices++; }
                    if (!p.Issued) { stopped[lane] = true; continue; }
                    offsets[lane] = p.Offset;
                    if (cells[9] == "NA") { Check.That(lane == 2 && start == 5853 && depth == 6, "only a sixth forecast exists"); continue; }
                    var read = TurnLandingContracts.ReadAim(cells[9],s);
                    var actual = Trial.Evaluate(s.Pose,default,read); var alt = Trial.Evaluate(s.Pose,p.Offset,read);
                    bool tracks = alt.Safe && alt.MaintainsHeight(actual) && alt.Tracks(actual);
                    bool pass = p.Trial.GeometricPass && alt.GeometricPass && tracks;
                    if (lane < 2)
                    { Check.Near(1000f*alt.Sole,F(cells[16+2*lane]),.002f,"historical adjacent height"); endpoints++; }
                    else
                    {
                        reserveRead++;
                        if (depth == 3 && pass) newThird++;
                        if (start == 3389 && depth == 12) Check.That(!pass && alt.Sole < -.00496f && alt.Gap > .00513f, "3389 still needs foot traction");
                        if (start == 4148 && depth == 12) Check.That(pass && alt.Sole > -.00190f, "shallow first opportunity preserved");
                        if (start == 5853 && (depth == 4 || depth == 5)) Check.That(tracks && !pass, "both new adjacent reads exist and remain below the band");
                    }
                    if (!tracks) stopped[lane] = true;
                }
            }
            Check.That(rows == 37 && choices == 66 && endpoints == 61 && reserveRead == 35 && newThird == 4, "same-state denominators");
        }

        private static Foot.Input Receipt(float cap) => new Foot.Input {
            Tether = new RightAnkleLandingSequence.Tether { BaseCap = cap, SelectedCap = cap, HardGap = .03f, Kp = 100f, Kd = 10f } };
        private static Trial.Aim Aim(Trial.Pose p, Vector3 offset) => new Trial.Aim {
            Travel = Vector3.forward, Target = p.Foot+offset, TargetSole = p.Foot+p.Rotation*(p.Center-Vector3.up*p.Half.y)+offset };
        private static Trial.Plan Baseline(Trial.Pose p, Trial.Aim aim)
        {
            var m = Trial.Evaluate(p,default,aim);
            return new Trial.Plan { Result = Trial.Choice.Baseline, Base = m, Trial = m };
        }

        private static void Traction()
        {
            var g = new Vector3(-.00179434f,-.00138169f,-.00460696f);
            var d = Foot.GeometryRequest(g,-.00496204f,out float rise,out bool canReach);
            Check.Near(rise,.00296204f,.0000001f,"pure height shortfall before inset");
            Check.Near(FootTransitionSource.Planar(d).magnitude,.00020038f,.000003f,"minimum concurrent planar reduction");
            Check.That(canReach && (g+d).magnitude < .005f,"same 5 mm sphere");
            var pose = TurnHandoffContracts.Pose(-.00496204f); pose.Lower = pose.Foot-g;
            var aim = Aim(pose,FootTransitionSource.Planar(d)*2f);
            var leg = Baseline(pose,aim);
            var cmd = Receipt(30f);
            const float dt = 1f/60f;
            var full = Foot.Predict(pose,leg,aim,cmd,dt,g,default,false);
            Check.That(full.Status == Foot.State.Predicted && full.Forecast.GeometricPass && full.Applied.magnitude <= 30f,"enough synthetic budget for both components");
            Foot.Settle(ref full,pose,default,aim,leg.Base,.03f);
            Check.That(full.SafeRead && full.Pass && full.Status == Foot.State.Read,"both endpoints checked");
            var small = Foot.Predict(pose,leg,aim,Receipt(1f),dt,g,default,false);
            Check.That(small.Status == Foot.State.Predicted && !small.Forecast.GeometricPass && small.Fraction > 0 && small.Fraction < 1f,"smaller budget records partial traction");
            Check.Near(small.Applied.magnitude,1f,.00001f,"one spherical cap on the entire command");
            var tiltedAim = Aim(pose,-FootTransitionSource.Planar(d)*15f);
            Foot.Settle(ref small,pose,default,tiltedAim,Trial.Evaluate(pose,default,tiltedAim),.03f);
            Check.That(!small.Pass,"unsafe or insufficient shifted result earns no pass");
            var tangent = Receipt(15f);
            tangent.Applied = tangent.Tether.Raw = Vector3.right * 14f;
            var biased = Foot.Predict(pose,leg,aim,tangent,dt,g,default,false);
            Check.That(biased.Status == Foot.State.Predicted && biased.Fraction < 1f && biased.Applied.y > 0f &&
                biased.Applied.magnitude <= 15.00001f,"large transverse native command shares the same three-dimensional cap");
            var clamped = Receipt(15f); clamped.Tether.SelectedCap = 1f; clamped.Tether.Raw = Vector3.up*4f; clamped.Applied = Vector3.up;
            var restricted = Foot.Predict(pose,leg,aim,clamped,dt,g,default,false);
            Check.That(clamped.Valid && restricted.Status == Foot.State.Predicted && restricted.Applied.magnitude <= 1.00001f &&
                !restricted.Forecast.GeometricPass,"a smaller selected cap and saturated native command cannot be overridden");
            var no = Foot.GeometryRequest(new Vector3(.00534f,-.00155f,0),-.01361f,out rise,out canReach);
            Check.That(!canReach && no.y < rise,"unchanged shin and shoe cannot clear the native deep case");
            cmd.Tether.Raw = Vector3.up * 2f;
            Check.That(!cmd.Valid && Foot.Predict(pose,leg,aim,cmd,dt,g,default,false).Status == Foot.State.MissingInput,"mismatched original receipt");
            Check.That(Foot.Predict(pose,leg,aim,Receipt(30f),dt,new Vector3(.031f,0,0),default,false).Status == Foot.State.StartBounds,"current hard gap enforced");
        }

        private static void Continuation()
        {
            var p = TurnHandoffContracts.Pose(.01f);
            var aim = Aim(p,Vector3.up*.01f); var leg = Baseline(p,aim);
            var cmd = Receipt(15f); cmd.Drag = .2f;
            var first = new Foot.Move { Status = Foot.State.Read, SafeRead = true,
                Body = new LeftLegHeightPreparation.Translation { Position = Vector3.up*.001f, Velocity = Vector3.up*.01f } };
            const float dt = .02f;
            var two = Foot.Predict(p,leg,aim,cmd,dt,p.Foot-p.Lower,first,true);
            float force = -.001f*100f-.01f*10f;
            float expectedV = (.01f+force*dt)*(1f-.2f*dt);
            Check.That(two.Status == Foot.State.Predicted && two.RequestedShift.magnitude == 0f,"unforced in-band continuation");
            Check.Near(two.Applied.y,force,1e-6f,"PD sees prior candidate state");
            Check.Near(two.Body.Velocity.y,expectedV,1e-7f,"native drag");
            Check.Near(two.Body.Position.y,.001f+expectedV*dt,1e-7f,"two-step endpoint");
            first.SafeRead = false;
            Check.That(Foot.Predict(p,leg,aim,cmd,dt,p.Foot-p.Lower,first,true).Status == Foot.State.MissingPrevious,"no restart after invalid read");
            var one = Foot.Predict(p,leg,aim,cmd,dt,p.Foot-p.Lower,default,false);
            Check.That(one.Status == Foot.State.Predicted && one.Body.Position.magnitude == 0,"independent one-step control");
        }

        private static void TraceWindows()
        {
            foreach (bool reverse in new[]{false,true})
            foreach (bool ordinary in new[]{false,true})
            foreach (bool missing in new[]{false,true})
                Window(reverse,ordinary,missing,true);
            Window(false,false,false,false);
        }

        private static void Window(bool reverse,bool ordinary,bool missing,bool turn)
        {
            var sim = new FootTransitionContracts.Sim { Reverse = reverse };
            sim.Frame(0,0,0);
            sim.Frame(0,0,0,(ref Trace.Tick l,ref Trace.Tick r) => { l.Motion = r.Motion = "IDLE"; });
            var p = TurnHandoffContracts.Pose(.002f);
            for (int n = 3; n <= 7; n++)
            {
                bool wants = sim.Trace.WantsLeftHeight;
                PhysicsStepContext step = default;
                sim.Frame(.02f,.03f,-.02f,(ref Trace.Tick l,ref Trace.Tick r) =>
                {
                    l.Motion = r.Motion = turn ? "TURN" : "IDLE";
                    TurnHandoffContracts.SetFree(ref l); TurnHandoffContracts.SetFree(ref r);
                    if (n == 3) { TurnHandoffContracts.SetRelease(ref l); TurnHandoffContracts.SetRelease(ref r); }
                    if (n == 6) { l.Miss = !ordinary; l.PendingMiss = !ordinary; l.Active = l.Ordinary = ordinary; l.EventFloor=l.Floor; }
                    if (n == 7) { l.Was = ordinary; l.Active=true; l.Recovery=!ordinary; l.EventFloor=l.Floor; }
                    TurnHandoffContracts.SetPose(ref l,p,0); step = l.Step;
                },held:n==5);
                Check.That(wants == (n>3 && n<=6),"native height request through first for both torso labels");
                bool found = sim.Trace.TryHandoff(0,step,out var t,out var standard,out var why,out var early,out var braked, out _);
                Check.That(found == (turn && n<=6),"same native first only");
                if (!found) continue;
                foreach (var w in new[]{standard,early,braked})
                {
                    bool issue = w.Visit(t,p,30,31,why);
                    Check.That(issue == (n<6),"settle predecessor before first");
                    if (issue) w.Issue(t,p,TurnHandoffContracts.Budget(),missing && n==4 ? default : Receipt(30f));
                }
            }
            string log=sim.Report();
            Check.That(log.Contains("TURN_HEIGHT_V041710/start-first=3|6/turnTrial="+turn), "paired release attribution includes IDLE-Lift control");
            Check.That(log.Contains("H171,6,") && !log.Contains("H171,7,"),"only one native first");
            if (!turn) { Check.That(!log.Contains("TURN_TRACTION_LINK_V041710"),"torsolabel control has no turn candidates"); return; }
            Check.That(log.Contains("TURN_TRACTION_LINK_V041710/start-first=3|6/early-braked-one-two=Pass|Pass|Pass|"+
                (missing ? "Unavailable" : "Pass")), "rolling window denominator and missing first command");
            Check.That(log.Contains("/brakedToTwo="+(missing ? "0|0|0|0|0" : "1|1|1|0|0")), "paired 2-step counts");
        }

        private static void ReceiptContext()
        {
            var pose = TurnHandoffContracts.Pose();
            var observed = TurnHandoffContracts.Tick(10,pose).Step;
            var input = TurnHandoffContracts.Tick(11,pose).Step;
            var t = TurnHandoffContracts.Tick(12,pose);
            var response = new PelvisNormalResponseEvidence { Valid=true, Input=input, Settled=t.Step, Fixed=11, PelvisId=41, ChestId=42, Q=.7f,
                RecoveryCaptured=true, RecoverySelected=true, RecoveryEligible=true, RecoveryExtraY=1.5f };
            var plan = new PelvisComReferencePlanner.ComPlan { ArrivalEnabled=true, SupportBrakingSelected=true, SupportBrakingExtraY=.012f,
                SupportInput=new SupportReferenceContext { ForStep=input, ObservedStep=observed, SupportId=3, StanceMask=3, ApproachMask=2 } };
            var ground = new FootGroundReference.GroundSample { Source=t.Source.Clock, Reason="CONTROLLED_SUPPORT_UNAVAILABLE" };
            LandingHeightBreakdown Read(PelvisNormalResponseEvidence p, FootGroundReference.GroundSample g) =>
                LandingHeightBreakdown.Capture(t,default,default,p,g,default,default,default,0f);
            response.CaptureSupportBraking(plan,input);
            var h=Read(response,ground);
            Check.That(h.Response && h.Braking && h.BrakingSelected && h.WitnessMask==3 && h.ApproachMask==2 &&
                h.WitnessSupportId==3 && h.Recovery && h.RecoveryEligible,"previous solve's witnesses and selections");
            Check.Near(h.BrakingExtraV,.012f,1e-7f,"reference velocity delta in m/s");
            Check.Near(h.RecoveryExtra,1.5f,1e-7f,"receiver acceleration addition in m/s²");
            Check.That(h.Ground && !h.GroundGeometry && !h.GroundSelected && h.GroundReason==ground.Reason,"known unavailable source geometry");
            ground.Source.Sequence--; h=Read(response,ground);
            Check.That(!h.Ground && h.Braking,"previous render is not a current ground receipt");
            plan.SupportInput=default; plan.SupportBrakingSelected=false; plan.SupportBrakingExtraY=0f;
            response.CaptureSupportBraking(plan,input); h=Read(response,default);
            Check.That(h.Braking && !h.BrakingSelected && h.WitnessMask==0 && h.WitnessSupportId==0,"observed absence differs from missing receipt");
            response.CaptureSupportBraking(plan,observed);
            Check.That(!Read(response,default).Braking,"a mismatched plan cannot retain a previous support receipt");
            response.CaptureSupportBraking(plan,input);
            response.Settled=observed;
            Check.That(!Read(response,default).Braking,"stale response loses its attached context");
        }

        private static void Heights()
        {
            var pose = TurnHandoffContracts.Pose(); var t = TurnHandoffContracts.Tick(12,pose);
            t.EntryReferenceValid = true;
            var identity=Quaternion.identity; var rawHip=Vector3.up*.94f;
            var upstream = new LandingParentPlacementInput {
                Prepared=true, Step=t.Step, Source=t.Source.Clock, RawP=rawHip, ReferenceP=rawHip+Vector3.up*.0015f,
                ActualP=rawHip-Vector3.up*.0065f, RawR=identity, ReferenceR=identity, ActualR=identity };
            var input = new LegLandingReferenceInput {
                Prepared=true, Step=t.Step, Source=t.Source.Clock, Fixed=t.Local, RawHip=rawHip, Parent=upstream, ParentHip=upstream.ActualPoint,
                UpperHip=upstream.ActualPoint, Upper=identity, UpperGoal=Quaternion.AngleAxis(-8,Vector3.right), KneeRelativeGoal=Quaternion.AngleAxis(12,Vector3.right),
                RawLower=identity, UpperHipToKnee=(-Vector3.up)*.37f, LowerKneeAnchor=Vector3.zero };
            var rawFoot=Quaternion.AngleAxis(-12,Vector3.right); var relative=Quaternion.AngleAxis(9,Vector3.right);
            var span=(-Vector3.up)*.43f;
            t.Target = rawHip+input.UpperHipToKnee+span+Vector3.up*.0009f;
            t.Lower=t.Target+(-Vector3.up)*.008f;
            var path=FootLandingReferenceEvidence.Build(t.Step,t.Source.Clock,t.Local,input,t.Target,t.Lower,identity,span,identity,relative);
            var center=new Vector3(0,-.04f,.055f); var half=new Vector3(.04f,.02f,.11f); const float top=.02f;
            path.SupportTop=top;
            t.SourceMin = FootBoxGeometry.Minimum(t.Target,rawFoot,center,half)-top;
            t.ControlMin = FootBoxGeometry.Minimum(t.Lower,relative,center,half)-top;
            var h=LandingHeightBreakdown.Capture(t,input,path,default,default,rawFoot,center,half,top);
            Check.That(h.Complete && h.Parent && !h.Recovery,"independent validity");
            Check.Near(h.Reference,.0015f,2e-7f,"socket reference can be above raw");
            Check.Near(h.ResponseY,-.008f,2e-7f,"native receiver below its reference");
            Check.Near(h.Binding,-.0009f,2e-7f,"rig binding separate from rotation");
            var selected=input.UpperGoal*input.KneeRelativeGoal*relative;
            float aligned=FootBoxGeometry.Minimum(path.Goal+rawHip-input.ParentHip,selected,center,half)-top;
            Check.Near(h.Aligned,aligned,2e-7f,"raw hip with all selected joint rotations");
            Check.Near(h.Rotation,aligned-(t.SourceMin+h.Binding),2e-7f,"pure rotation after matching the hip");
            Check.Near(h.Binding+h.Reference+h.ResponseY+h.Rotation+h.Tracking,t.ControlMin-t.SourceMin,2e-7f,"ordered minima sum to control-minus-raw");
            Check.That(Mathf.Abs(h.Tracking)>.0001f,"native tracking stays separate");
            input.Prepared=false;
            var compact=LandingHeightBreakdown.Capture(t,input,path,default,default,rawFoot,center,half,top);
            Check.That(compact.Parent && !compact.Complete,"compact receipt cannot imply a rotation budget");
            input.Prepared=true; var wrong=path; wrong.Fixed--;
            Check.That(!LandingHeightBreakdown.Capture(t,input,wrong,default,default,rawFoot,center,half,top).Complete,"reject crossed solve");
            var alternate = path; alternate.EntryFootRelative = identity;
            Check.That(!LandingHeightBreakdown.Capture(t,input,alternate,default,default,rawFoot,center,half,top).Complete,"reject different control shoe at entry");
            t.SourceMin += .003f;
            Check.That(!LandingHeightBreakdown.Capture(t,input,path,default,default,rawFoot,center,half,top).Complete,"reject different source shoe geometry");
        }
    }
}
