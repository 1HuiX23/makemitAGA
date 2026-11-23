using System;
using System.Globalization;
using System.IO;
using System.Linq;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Command = MakemitAGA.ActiveRagdollLab.Model.HipAnchorPairReport.Command;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LandingFootTraction;
using Rise = MakemitAGA.ActiveRagdollLab.Physics.LandingRiseAllocation;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;
using Trial = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;

namespace ARL.Contracts
{
    internal static class LandingRiseContracts
    {
        internal static void Run()
        {
            Check.Run("172 twelve native 171 receiver inputs locate the first causal rise request", RecordedReceiver);
            Check.Run("172 both legs and torso share each private allocation and reject mismatched identities", Allocations);
            Check.Run("172 socket acceleration includes the parent lever and angular response", Socket);
            Check.Run("172 next-step receipt, first miss or ordinary, duplicate callback and support identity", () => Window(false));
            Check.Run("176 report joins the two real observation paths in either callback order and censors missing pelvis responses", () => Window(true));
            Check.Run("176 an already applied pelvis pulse supplies a measured pose without a second height credit", () => Window(true,true));
            Check.Run("172 real 171 rolling foot commands preserve original outcomes and evaluate both margins", RecordedFeet);
            Check.Run("172 unreachable quarter-millimetre margin cannot score or relax ordinary thresholds", Margins);
        }

        private static float F(string s) => float.Parse(s,CultureInfo.InvariantCulture);
        private static Vector3 V(string s)
        {
            var a = s.Split(',').Select(F).ToArray(); return new Vector3(a[0],a[1],a[2]);
        }
        private static StreamReader Fixture(string name) =>
            new StreamReader(typeof(LandingRiseContracts).Assembly.GetManifestResourceStream("ARL.Contracts."+name));

        private static void RecordedReceiver()
        {
            using var f = Fixture("LandingReceiver171");
            f.ReadLine(); string line;
            int rows=0, first=0, support=0, without=0, with=0;
            while ((line=f.ReadLine()) != null)
            {
                var c=line.Split('\t'); int local=int.Parse(c[1]), mask=int.Parse(c[2]);
                var input=new Rise.Input { Step=PelvisDynamicReceiverIsolation.LoggedStep(), Error=F(c[4]), RefV=F(c[5]), PreV=F(c[6]), RefA=F(c[8]) };
                input.Step.Dt=F(c[3]); var baseline=V(c[9]);
                float extra=Rise.LeadRequest(input,baseline,out bool ahead)-baseline.y;
                Check.Near(extra,F(c[12]),.000004f,"current receipt only, original 3D request");
                Check.Near((F(c[7])-input.PreV)/input.Step.Dt,F(c[11]),.00002f,"native velocity response");
                Check.Near(F(c[10]),F(c[11]),.00002f,"observed native allocation");
                if (mask!=0 && support==0) support=local;
                if (ahead && extra > 0f)
                {
                    if (first==0) first=local;
                    if (mask==0) without++; else with++;
                }
                rows++;
            }
            Check.That(rows==12 && first==4140 && support==4147 && without==7 && with==2,"prior to and following the return of a witness");
        }

        private static PelvisHipAttitudeCommand Torso(in Command l,in Command r)
        {
            Check.That(PelvisHipAttitudeCommand.TryPlanWithReaction(l.ParentInput,-l.Force-r.Force,l.ParentTorque+r.ParentTorque,
                l.BilateralAngularReaction,out var t) && !t.Clamped,"real torso completion");
            t.CommonAcceleration=l.CommonAcceleration; t.MotionScale=l.MotionScale; t.UpperAngularScale=l.UpperAngularScale;
            t.UpwardExtraAcceleration=l.UpwardExtraAcceleration; t.Committed=true;
            Check.That(t.Matches(l.ParentInput),"actual receipt joined"); return t;
        }

        private static void Allocations()
        {
            var step=PelvisDynamicReceiverIsolation.LoggedStep();
            var i=new Rise.Input { Step=step, Error=.002f, RefV=.15f, PreV=-.02f, RefA=3f };
            int solved=0,limited=0,retained=0;
            foreach (float x in new[]{0f,4f,8f})
            foreach (float y in new[]{1f,3f,6f})
            foreach (float angle in new[]{0f,4f,8f,18f})
            {
                var p=PelvisDynamicReceiverIsolation.Baseline(step);
                p.Acceleration=new Vector3(x,y,0f); p.UpwardRetentionSelected=p.BoundedTradeSelected=true;
                var l=HipAllocationContracts.Input(p,true,Vector3.forward*angle,Vector3.right*2f,Vector3.up*.2f,-Vector3.right*.1f);
                var r=HipAllocationContracts.Input(p,false,Vector3.forward*angle,Vector3.up,Vector3.up*.15f,Vector3.right*.1f);
                Check.That(HipBilateralForceSolver.TryPlan(l,r,out var lc,out var rc),"native solve");
                var torso=Torso(lc,rc);
                var alt=Rise.Compare(step,6,true,i,l,r,lc,rc,torso);
                Check.That(HipBilateralForceSolver.TryPlan(r,l,out var primaryR,out var secondaryL),"right-primary baseline");
                var reverse=Rise.Compare(step,6,true,i,l,r,secondaryL,primaryR,Torso(primaryR,secondaryL),false);
                Check.That(reverse.Status==Rise.State.Compared,"replay actual primary order");
                Check.That(alt.Status==Rise.State.Compared && alt.Native.Valid && alt.RequestOnly.Valid && alt.EarlyRetention.Valid,"three independent allocations");
                Check.That(alt.RequestOnly.Request>=y && alt.RequestOnly.Request==alt.EarlyRetention.Request && alt.EarlyRetention.System.y>=alt.RequestOnly.System.y-.0001f,
                    "same request for the retention-only comparison");
                Check.Near(alt.RequestOnly.Q,alt.EarlyRetention.Q,0f,"retention uses the original q of the changed request");
                Check.Near(alt.RequestOnly.System.x,alt.EarlyRetention.System.x,.000001f,"tangent unchanged at that q");
                Check.That(l.Seed.ParentInput.SameAs(p) && r.Seed.ParentInput.SameAs(p) && torso.Matches(p),"no changes to the actual inputs");
                if (alt.RequestOnly.Q<1f) limited++;
                if (alt.EarlyRetention.System.y>alt.RequestOnly.System.y+.00001f) retained++;
                // Replay the planned private request through the same physical precommit predicates.
                var altFrame=p; altFrame.Acceleration.y=alt.EarlyRetention.Request; altFrame.UpwardRetentionEligible=true;
                var ll=l; var rr=r; ll.Seed.ParentInput=rr.Seed.ParentInput=altFrame;
                Check.That(HipBilateralForceSolver.TryPlan(ll,rr,out var al,out var ar) &&
                    HipAnchorTranslationPair.CheckActualPrecommit(ll,al) && HipAnchorTranslationPair.CheckActualPrecommit(rr,ar),"both original force/torque budgets");
                Check.Near(alt.EarlyRetention.System.y,Torso(al,ar).TargetSystemAcceleration.y,.000001f,"full bilateral result");
                if (solved++!=0) continue;

                var stale=i; stale.Step.Sequence++;
                Check.That(!Rise.Compare(step,6,true,stale,l,r,lc,rc,torso).Native.Valid,"foreign input clock");
                Check.That(Rise.Compare(step,6,false,i,l,r,lc,rc,torso).Status==Rise.State.ContactOrGeometry,"active peer or missing footprint");
                stale=i; stale.RefA=float.NaN;
                Check.That(!Rise.Compare(step,6,true,stale,l,r,lc,rc,torso).Native.Valid,"nonfinite reference");
                var different=rc; different.UpperActuatorTorque.z+=.01f;
                Check.That(Rise.Compare(step,6,true,i,l,r,lc,different,torso).Status==Rise.State.BaselineMismatch,"right torque mismatch");
                stale=i; stale.Support=new SupportReferenceContext {ForStep=step,ObservedStep=PelvisDynamicReceiverIsolation.LoggedStep(-1),SupportId=88,StanceMask=1,ApproachMask=1};
                Check.That(Rise.Compare(step,6,true,stale,l,r,lc,rc,torso).Status==Rise.State.WrongSupport,"wrong floor");
                stale.Support.SupportId=6; p.UpwardRetentionEligible=true;
                ll=l; rr=r; ll.Seed.ParentInput=rr.Seed.ParentInput=p;
                Check.That(HipBilateralForceSolver.TryPlan(ll,rr,out al,out ar),"witness native");
                alt=Rise.Compare(step,6,true,stale,ll,rr,al,ar,Torso(al,ar));
                Check.That(alt.Status==Rise.State.Compared && alt.EarlyRetention.System.y==alt.RequestOnly.System.y,"no double retention after the witness returns");
                stale=i; stale.Error=-.08f; Check.That(Rise.Compare(step,6,true,stale,l,r,lc,rc,torso).Status==Rise.State.NoDeficit,"still leading in height");
                Check.Near(Rise.LeadRequest(i,Vector3.right*12f,out _),0f,0f,"horizontal saturation leaves no controllable Y");
                stale=i; stale.RefA=0f; stale.PreV=.149f; stale.RefV=.15f;
                Check.Near(Rise.LeadRequest(stale,Vector3.up*3f,out _),3f,0f,"one-step velocity budget already used");
            }
            Check.That(solved==36 && limited>0 && retained>0,"limited q and genuine retained force covered");
        }

        private static void Socket()
        {
            var step=PelvisDynamicReceiverIsolation.LoggedStep();
            var p=PelvisDynamicReceiverIsolation.Baseline(step); p.ReceiverTorque=Vector3.forward; p.ParentW=Vector3.up;
            p.UpwardRetentionSelected=p.BoundedTradeSelected=true;
            var l=HipAllocationContracts.Input(p,true,Vector3.zero,Vector3.zero,Vector3.zero,Vector3.right*.1f);
            var r=HipAllocationContracts.Input(p,false,Vector3.zero,Vector3.zero,Vector3.zero,-Vector3.right*.1f);
            Check.That(HipBilateralForceSolver.TryPlan(l,r,out var lc,out var rc),"native socket model");
            var i=new Rise.Input {Step=step};
            var result=Rise.Compare(step,6,true,i,l,r,lc,rc,Torso(lc,rc));
            Check.Near(result.Native.SocketA-result.Native.System.y,.1f,.000005f,"left hip rises under angular acceleration with zero vertical COM force");
            var mirrored=Rise.Compare(step,6,true,i,r,l,rc,lc,Torso(rc,lc));
            Check.Near(mirrored.Native.SocketA-mirrored.Native.System.y,-.1f,.000005f,"the opposite socket actually descends");
        }

        private static void Window(bool strict, bool pulse = false)
        {
            foreach (bool reversed in new[]{false,true})
            foreach (bool ordinary in new[]{false,true})
            foreach (bool missing in new[]{false,true})
            {
                if (pulse && (missing || !ordinary)) continue;
                var sim=new FootTransitionContracts.Sim { Reverse=reversed };
                sim.Frame(0,0,0);
                sim.Frame(0,0,0,(ref Trace.Tick l,ref Trace.Tick r) => { if (strict) l.Motion=r.Motion="IDLE"; });
                var pose=TurnHandoffContracts.Pose(ordinary ? .001f : -.003f);
                PelvisNormalResponseEvidence next=default;
                for (int n=3;n<=6;n++)
                {
                    PhysicsStepContext step=default; int floor=0;
                    sim.Frame(.02f,.03f,-.02f,(ref Trace.Tick l,ref Trace.Tick r)=>{
                        TurnHandoffContracts.SetFree(ref l); TurnHandoffContracts.SetFree(ref r);
                        if (n==3) { TurnHandoffContracts.SetRelease(ref l); TurnHandoffContracts.SetRelease(ref r); }
                        if (n==5) { l.Ordinary=l.Active=ordinary; l.Miss=l.PendingMiss=!ordinary; l.EventFloor=l.Floor; }
                        if (n==6) { l.Was=ordinary; l.Active=true; l.EventFloor=l.Floor; l.Recovery=!ordinary; }
                        if (strict)
                        {
                            l.Motion=r.Motion="TURN"; l.Source.World=Vector3.forward;
                            TurnHandoffContracts.SetPose(ref l,pose,0f);
                            l.Entry=new OrdinaryFootEntry.Sample {Local=n,Floor=n>=4 ? l.Floor : 0,NativeSole=l.BoxMin,
                                NativeInside=l.Inside,LegacyReady=n>=4,Confirmed=n==5,IdentityHeld=n==5};
                            if (l.Miss) l.NativeDepthLatch=4;
                            l.Height=new LandingHeightBreakdown {Parent=true,Reference=-n*.001f,ResponseY=n*.0003f};
                        }
                        step=l.Step; floor=l.Floor;
                    },held:n==4);
                    bool wants=sim.Trace.WantsRiseStep(step);
                    Check.That(wants==(n<=5),"paired release including the terminal incoming solve");
                    if (!wants) continue;
                    if (strict)
                    {
                        Check.That(sim.Trace.TryHandoff(0,step,out var tick,out _,out var why,out _,out _,out var combined),
                            "strict release streams publish both clocks");
                        bool issue=combined.Visit(tick,pose,201,203,why);
                        Check.That(issue==(n<5),"geometry retained through native first");
                        if (issue) combined.Issue(tick,pose,TurnHandoffContracts.Budget());
                    }
                    var p=PelvisDynamicReceiverIsolation.Baseline(step);
                    p.Acceleration=Vector3.up*2f; p.UpwardRetentionSelected=p.BoundedTradeSelected=true;
                    var l=HipAllocationContracts.Input(p,true,Vector3.zero,Vector3.zero,Vector3.zero,Vector3.zero);
                    var r=HipAllocationContracts.Input(p,false,Vector3.zero,Vector3.zero,Vector3.zero,Vector3.zero);
                    l.Seed.Fixed=r.Seed.Fixed=n+20;
                    // Constant N->N+1 tangential-free fixture; original socket still descends at N=4.
                    l.Seed.ParentV.y=n==3 ? -2.01f*step.Dt : n==4 ? -.01f*step.Dt : 1.99f*step.Dt;
                    var i=new Rise.Input {Step=step,RefV=.2f,RefA=2f,Error=.003f,PreV=l.Seed.ParentV.y};
                    if (n==4) i.Support=new SupportReferenceContext {ForStep=step,ObservedStep=next.Input,SupportId=floor,StanceMask=1,ApproachMask=1};
                    p.UpwardRetentionEligible=n==4; l.Seed.ParentInput=r.Seed.ParentInput=p;
                    Check.That(HipBilateralForceSolver.TryPlan(l,r,out var lc,out var rc),"native event command");
                    var torso=Torso(lc,rc); next.Settled=step;
                    if (missing && n==(strict ? 4 : 5)) next.ChestId++;
                    sim.Trace.ObserveRise(step,i,l,r,lc,rc,torso,next,pulseApplied:pulse && n==3);
                    Check.That(!sim.Trace.WantsRiseStep(step),"duplicate does not settle or issue");
                    sim.Trace.ObserveRise(step,i,l,r,lc,rc,torso,next);
                    next=new PelvisNormalResponseEvidence {Valid=true,Input=step,Fixed=n+201,PelvisId=p.BodyId,ChestId=p.ChestBodyId,
                        ErrorY=i.Error,VelocityErrorY=i.VelocityError,Feedforward=i.RefA,Request=2f,Allocated=torso.TargetSystemAcceleration.y,
                        UpwardSelected=true,UpwardEligible=p.UpwardRetentionEligible,UpwardExtra=lc.UpwardExtraAcceleration,
                        Q=lc.MotionScale,Observed=torso.TargetSystemAcceleration.y,PreV=i.PreV,PostV=i.PreV+torso.TargetSystemAcceleration.y*step.Dt};
                }
                var log=sim.Report();
                Check.That(log.Contains("TURN_RISE_LINK_V041720/start-first=3|5/issues-matched-compared-omitted-faults=2|"+
                    (missing ? "1|1" : pulse ? "2|1" : "2|2")+"|0|0"),"matched denominator on closing frame");
                if (strict)
                {
                    bool hasBudget = !missing && !pulse;
                    Check.That(log.Contains("TURN_HIP_HEIGHT_V041760/start-firstStrong=3|4") && log.Contains("H176,3,4," + (pulse ? "PulseApplied" : "Compared") + ",") &&
                        !log.Contains("H176,4,5,") && log.Contains("/geometry-compared=1|" + (hasBudget ? 1 : 0) +
                        "/firstStrongGeometry-budget=True|" + hasBudget), "only incoming strong observation; no credit for an unmatched response");
                    string h = log.Split('\n').Single(s=>s.StartsWith("H176,"));
                    Check.That(h.EndsWith(",-3,-4,1.2") && h.Contains(",Paired,") &&
                        (hasBudget ? !h.Contains(",NA,") : h.Contains(",NA,NA,NA,NA,NA,")), "joined reference and independently available geometry");
                }
                Check.That(log.Contains("R172,3,4,") && log.Contains("R172,4,5,") && !log.Contains("R172,5,"),"only causal issues");
                Check.That(log.Contains(missing ? ",ResponseMissing," : pulse ? "firstIssuedCross-requestOnlyEarlier-" :
                    "firstNativeCross-requestOnlyEarlier-earlyRetentionEarlier-noWitness-withWitness=5|4|4|4|0"),
                    "real and conditional zero-crossings and missing receipt");
            }
        }

        private static void RecordedFeet()
        {
            using var file=Fixture("TurnLanding171"); file.ReadLine(); string line;
            int current=0,rows=0,firsts=0,oldMargin=0,newMargin=0,censored=0;
            Trial.Offset previous=default; Foot.Move oldOne=default,bufferedOne=default;
            while ((line=file.ReadLine())!=null)
            {
                var c=line.Split('\t'); int start=int.Parse(c[0]),depth=int.Parse(c[1]);
                if (start!=current) { current=start; previous=default; oldOne=bufferedOne=default; }
                var s=TurnLandingContracts.DecodeIssue(c);
                var leg=Trial.Select(s.Pose,s.Issue,s.Budget,s.Dt,s.HipY,previous,true,true);
                Check.That(leg.Result.ToString()==c[17],"replay measured leg choice"); previous=leg.Offset;
                var cap=c[16].Split('|').Select(F).ToArray(); var a=c[15].Split('|');
                var input=new Foot.Input { Drag=cap[4], Applied=V(a[1]), Tether=new RightAnkleLandingSequence.Tether {
                    Raw=V(a[0]),BaseCap=cap[0],SelectedCap=cap[1],Kp=cap[2],Kd=cap[3],HardGap=cap[5]*.001f } };
                Check.That(input.Valid && F(c[24])<.001f,"real 3-D command and sub-micrometre nominal endpoint error");
                var read=TurnLandingContracts.ReadAim(c[9],s); var native=Trial.Evaluate(s.Pose,default,read);
                bool horizontal=Trial.Evaluate(s.Pose,leg.Offset,read).Tracks(native);
                Check.That(horizontal==bool.Parse(c[18]),"historical tracking stop");
                var gap=V(c[23]);
                Foot.Move Run(bool two,bool buffered,Foot.Move last)
                {
                    var m=Foot.Predict(s.Pose,leg,s.Issue,input,s.Dt,gap,last,two,buffered);
                    if (horizontal) Foot.Settle(ref m,s.Pose,leg.Offset,read,native,input.Tether.HardGap);
                    return m;
                }
                var old=Run(true,false,oldOne); var alt=Run(true,true,bufferedOne);
                oldOne=Run(false,false,default); bufferedOne=Run(false,true,default); rows++;
                Check.That(old.Status.ToString()==c[19],"original two-step receipt");
                if (c[20]!="NA")
                {
                    var nativeForecast=c[20].Split('|');
                    Check.Near(old.Forecast.Sole*1000f,F(nativeForecast[0]),.001f,"original two-step box minimum");
                    Check.Near(old.Forecast.Gap*1000f,F(nativeForecast[1]),.001f,"original two-step ankle gap");
                    Check.Near(old.Body.Velocity.y,F(c[21]),.00001f,"real baseline terminal delta velocity");
                }
                if (!horizontal) { censored++; Check.That(start==4136 && start+depth<int.Parse(c[22]),"missing fourth landing stays censored"); }
                if (start+depth!=int.Parse(c[22])) continue;
                firsts++; if (old.MarginPass) oldMargin++; if (alt.MarginPass) newMargin++;
                Check.That(old.Pass && alt.Pass && alt.Status==Foot.State.Read,"same first opportunity and original band");
                Console.WriteLine(FormattableString.Invariant($"172 replay first={start+depth} bufferedWorstMm={alt.WorstMargin*1000f:F5} oldWorstMm={old.WorstMargin*1000f:F5} bufferedDeltaVy={alt.Body.Velocity.y:F6}"));
            }
            Check.That(rows==45 && firsts==3 && censored==1 && oldMargin==0 && newMargin==3,"paired quarter-millimetre landing margins");
        }

        private static void Margins()
        {
            var gap=new Vector3(.004f,-.00129f,0f);
            var request=Foot.GeometryRequest(gap,-.00765f,out _,out bool possible,true);
            Check.That(possible && (gap+request).magnitude < Trial.GapLimit-Foot.LandingMargin && -.00765f+request.y >= -.002f+Foot.LandingMargin,
                "a quarter millimetre fits in the narrow historical geometry");
            var highGap=new Vector3(0f,.0053f,0f);
            var descend=Foot.GeometryRequest(highGap,.001f,out _,out possible,true);
            Check.That(possible && descend.y<0f && .001f+descend.y>=-.00175f &&
                (highGap+descend).magnitude <= .00475f,"shoe has sufficient surplus height to lower inside the sphere");
            var impossible=Foot.GeometryRequest(gap,-.0084f,out float needed,out possible,true);
            Check.That(!possible && impossible.y<needed,"unreachable reports the available vertical slice");
            var m=new Trial.Measure {Inside=true,BendLegal=true,Sole=-.00175f,Gap=.00475f};
            var move=new Foot.Move {Pass=true,Forecast=m,Settled=m};
            move.Settled.Gap=.00476f; Check.That(!move.MarginPass && move.Pass,"the actual endpoint restricts margin");
            move.Settled=m; move.Forecast.Sole=-.00176f; Check.That(!move.MarginPass,"forecast and read are both required");
            move.Forecast=m; move.Forecast.Sole=move.Settled.Sole=-.0017f; move.Forecast.Gap=move.Settled.Gap=.0047f; move.Pass=false;
            Check.That(!move.MarginPass,"geometric margin cannot override the original safety predicate");
        }
    }
}
