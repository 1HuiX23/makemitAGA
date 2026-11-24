using System;
using System.Globalization;
using System.IO;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Tick = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace.Tick;
using Rise = MakemitAGA.ActiveRagdollLab.Physics.LandingRiseAllocation;
using State = MakemitAGA.ActiveRagdollLab.Physics.TurnHeightContinuation.State;
using Kind = MakemitAGA.ActiveRagdollLab.Runtime.SourceRotationCoverage.Kind;

namespace ARL.Contracts
{
    internal static class TurnHeightContracts
    {
        internal static void Run()
        {
            Check.Run("180 source-only grade, held Render, dropouts, restart and bounded receipts", Coverage);
            Check.Run("180 matched reference, recursive continuation, partial budget and lifecycle", Height);
            Check.Run("180 source grade frozen at native rejection and causal issue/read through both feet", Trace);
            Check.Run("180 contradictory height and ankle evidence at archived physical landings", Anchors);
        }

        private static PhysicsStepContext Step(int n) => new PhysicsStepContext { Epoch=4, Sequence=100+n, SourceSequence=50+n,
            Dt=.02f, FixedTime=4+n*.02f, UnscaledTime=4+n*.02f };
        private static SourceStepContext Source(PhysicsStepContext s) => new SourceStepContext {
            Epoch=s.Epoch, Sequence=s.SourceSequence, Time=s.UnscaledTime, Dt=s.Dt };

        private static void Coverage()
        {
            var audit=new SourceRotationCoverage(); audit.Reset();
            Kind Observe(int n,float yaw)
            {
                var s=Step(n); var source=Source(s);
                audit.Observe(source,Quaternion.AngleAxis(yaw,Vector3.up),20f);
                return audit.AtOpportunity(source,s,.01f);
            }
            for(int n=0;n<=12;n++) Check.That(Observe(n,n)==Kind.Unknown,"need 250 ms of usable source");
            Check.That(Observe(13,13)==Kind.Continuous && Observe(14,42)==Kind.Jump,"continuous rotation then genuine source jump");
            Check.That(Observe(14,100)==Kind.Jump && Observe(15,42)==Kind.Jump,"duplicate source never creates a new increment");
            for(int n=16;n<=26;n++) Observe(n,42);
            var held=Step(26);
            Check.That(audit.AtOpportunity(Source(held),held,.06f)==Kind.Continuous,"a held sample cannot extend an old jump beyond 250 ms");
            Check.That(Observe(27,42)==Kind.Continuous,"old jump leaves the grading interval");
            var s=Step(27);
            Check.That(audit.AtOpportunity(Source(s),s,.101f)==Kind.Unknown,"stale held source");
            Check.That(Observe(29,42)==Kind.Unknown,"missing Render sequence");
            var bad=Step(30); var c=Source(bad); c.Dt=.09f; audit.Observe(c,Quaternion.identity,20f);
            Check.That(audit.AtOpportunity(c,bad,0f)==Kind.Unknown,"long source interval");
            var log=new StringBuilder(); audit.Append(log);
            Check.That(log.ToString().Contains("/worldAxis=0|1|0") && log.ToString().Contains("kind=Jump/thresholdDeg=20/peakAt-deg-dt=64|") && log.ToString().Contains("S180/epoch-first-last=4|50|50/kind=Unknown"),"canonical source id with peak and explicit start");
            audit.Reset();
            for(int n=0;n<SourceRotationCoverage.Capacity*2+12;n++) Observe(n,(n/2)%2==0 ? 0f : 40f);
            Check.That(audit.AtOpportunity(Source(Step(522)),Step(522),0f)==Kind.Unknown,"unretained tail cannot qualify");
            // Same 720 degrees/second raw turn sampled at two rates. A smooth 24 degree
            // increment at 30 Hz must not become a source-exception candidate.
            foreach (int fps in new[]{30,60})
            {
                audit.Reset();
                for (int n=0;n<45;n++)
                {
                    var frame=Step(n); var sample=Source(frame);
                    sample.Time=n/(float)fps; sample.Dt=1f/fps;
                    audit.Observe(sample,Quaternion.AngleAxis(720f*sample.Time,Vector3.up),20f);
                    if (n>=20) Check.That(audit.AtOpportunity(sample,frame,0f)==Kind.Continuous,"smooth turn at "+fps+" fps");
                }
                var reversal=Step(45); var last=Source(reversal); last.Time=45f/fps; last.Dt=1f/fps;
                audit.Observe(last,Quaternion.AngleAxis(720f*44f/fps-28f,Vector3.up),20f);
                Check.That(audit.AtOpportunity(last,reversal,0f)==Kind.Jump,"signed trend captures an abrupt reversal");
            }
            foreach (float sign in new[]{1f,-1f})
            {
                audit.Reset();
                var src=Source(Step(0)); src.Time=0f;
                for (int n=0;n<16;n++)
                {
                    src.Sequence++; src.Time+=.02f; src.Dt=.02f;
                    audit.Observe(src,Quaternion.identity,20f);
                }
                src.Sequence++; src.Time+=.04f; src.Dt=.04f;
                audit.Observe(src,new Quaternion(0f,sign,0f,0f),20f);
                src.Sequence++; src.Time+=.02f; src.Dt=.02f;
                audit.Observe(src,Quaternion.AngleAxis(270f,Vector3.up),20f);
                var rendered=new StringBuilder(); audit.Append(rendered);
                Check.That(rendered.ToString().Contains("/smoothOverBudget=1"),"half-frame extrapolation is invariant to the 180 degree quaternion sign");
            }
            audit.Reset(); Check.That(Observe(0,0)==Kind.Unknown,"fresh run has no previous evidence");
            audit.Reset();
            for (int n=0; n<12; n++)
            {
                var slow=Step(n);
                var sample=Source(slow); sample.Dt=1f/30f; sample.Time=n*sample.Dt;
                audit.Observe(sample,Quaternion.AngleAxis(n*2.5f,Vector3.up),2f);
                if (n>=8) Check.That(audit.AtOpportunity(sample,slow,0f)==Kind.Continuous,
                    "a low profile angle still needs to exceed the physical reference speed");
            }
        }

        private sealed class Sequence
        {
            internal TurnHeightContinuation Window;
            internal string Arm = PelvisDynamicReceiverIsolation.DynamicHipArmB;
            internal TurnHeightContinuation.Result Run(int n,string change=null,bool rightFirst=false)
            {
                var s=Step(n); var src=Source(s);
                if(change=="held") { src=Source(Step(n-1)); s.SourceSequence=src.Sequence; }
                var t=new Tick { Step=s,Local=n+10,Floor=6,PriorFloor=6,Inside=true,Body=7,Was=n==0,
                    Source=new FootTransitionSource.Sample {Valid=true,Root=100,Continuity=1,Clock=src},
                    Phase=FootLandingOpportunityAudit.Phase.Lift, Motion="TURN",
                    Gait=new FootGaitEvidence(5,true,6,0f,Vector3.up,0f,0f) };
                var r=t; r.Body=8;
                if(change=="surface") r.Floor=9;
                if(change=="body") r.Body=9;
                if(change=="stop") t.Motion="STOP";
                if(change=="rightStop") r.Motion="STOP";
                if(change=="loaded") r.Active=true;
                if(change=="stale") t.Age=.2f;
                var support=default(SupportReferenceContext);
                if(n==0 || change=="witness") support=new SupportReferenceContext { ForStep=s,ObservedStep=Step(n-1),SupportId=6,StanceMask=n==0 ? 3 : 1 };
                var p=Vector3.up; var v=Vector3.zero;
                var i=new ParentHeightProbe.Input {Step=s,Arm=Arm,ReceiverReady=true,
                    Position=p,Goal=p,Velocity=v,GoalV=v,Support=support,SampledPelvisY=1.01f,FloorY=1.01f,ActualCom=p,
                    PositionLimit=.30f,CommittedPosition=p,CommittedV=v};
                if(change=="lower") { i.SampledPelvisY=.99f; i.FloorY=.992f; }
                if(change=="badBase") i.CommittedV=Vector3.right;
                if(change=="badRequest") i.PositionError=Vector3.right*.08f;
                if(change=="limit") i.PositionLimit=.0001f;
                var f=PelvisDynamicReceiverIsolation.Baseline(s);
                f.Acceleration=Vector3.zero; f.UpwardRetentionSelected=f.BoundedTradeSelected=true;
                var left=HipAllocationContracts.Input(f,true,default,default,default,default);
                var right=HipAllocationContracts.Input(f,false,default,default,default,default);
                Check.That(Rise.PlanPair(left,right,!rightFirst,out var lc,out var rc) && Rise.TryTorso(lc,rc,out _),"native commands exist");
                Rise.TryTorso(lc,rc,out var torso);
                return Window.Compare(10,t,r,i,new Rise.Input { Step=s,Support=support },left,right,lc,rc,!rightFirst,change=="pulse",Rise.Model(lc,rc,torso));
            }
        }

        private static void Height()
        {
            foreach(bool rightFirst in new[]{false,true})
            {
                var q=new Sequence(); Check.That(q.Run(0).State==State.Seed,"both previous witnesses seed once");
                var one=q.Run(1,"held",rightFirst); var two=q.Run(2,null,rightFirst);
                Check.That(one.State==State.Compared && two.State==State.Compared && one.ReferenceY>0 &&
                    two.ReferenceY>two.InstantY && one.Native.Valid && one.Trial.Valid && one.Trial.System.y>=one.Native.System.y,
                    "independent one-step and propagated references share native physical inputs");
                Check.That(one.ReferenceXZ==0f && !float.IsNaN(one.Trial.RightForce.magnitude),"on-axis case and peer reaction");
                var pulse=q.Run(3,"pulse"); Check.That(pulse.State==State.Pulse && !pulse.Trial.Valid,"separate already-applied pulse");
                var lower=q.Run(4,"lower"); Check.Near(lower.Floor,.992f,.00001f,"current raw lowering and current rotated offset");
                for(int n=5;n<=10;n++) Check.That(q.Run(n).State==State.Compared,"reference may continue for at most 200 ms");
                Check.That(q.Run(11).State==State.TimeLimit && q.Run(12).State==State.None,"expired window cannot restart");
            }
            foreach(var pair in new[]{("surface",State.Clock),("body",State.Clock),("stop",State.MotionChanged),("rightStop",State.MotionChanged),
                ("stale",State.Clock),("loaded",State.Clock),("badBase",State.BaselineMismatch),
                ("badRequest",State.BaselineMismatch),("limit",State.PositionGate),("witness",State.WitnessReturned)})
            {
                var q=new Sequence(); q.Run(0);
                Check.That(q.Run(1,pair.Item1).State==pair.Item2 && q.Run(2).State==State.None,pair.Item1);
            }
            var armA = new Sequence { Arm = PelvisDynamicReceiverIsolation.DynamicHipArmA };
            var armB = new Sequence();
            for (int n = 0; n <= 11; n++)
            {
                var a = armA.Run(n); var b = armB.Run(n);
                Check.That(a.State == b.State && a.ReferenceY == b.ReferenceY && a.Trial.Valid == b.Trial.Valid &&
                    a.Trial.System.Equals(b.Trial.System), "both formal arms retain the 181 B continuation at " + n);
            }
            var gap=new Sequence(); gap.Run(0); Check.That(gap.Run(2).State==State.Clock,"gap in fixed clock");
            var missing=new Sequence(); Check.That(missing.Run(1).State==State.Missing,"no late seed");
            var v=new Vector3(.5f,-.02f,-.6f);
            var native=PelvisComReferencePlanner.Build(Vector3.zero,v,Vector3.right,Vector3.zero,.02f,true,false,default,default,v,true,enhancedNormalFeedback:true);
            var shaped=PelvisComReferencePlanner.Build(Vector3.zero,v,Vector3.right,Vector3.zero,.02f,true,false,default,default,v,true,enhancedNormalFeedback:true,transitionFloorY:.01f);
            Check.That(!shaped.SupportFloorSelected && !shaped.SupportBrakingSelected && shaped.SupportInput.StanceMask==0 &&
                shaped.Acceleration.magnitude<=6.001f && shaped.Velocity.magnitude<=3.0001f &&
                shaped.Velocity.x==native.Velocity.x && shaped.Velocity.z==native.Velocity.z,"no phantom support or extra vector allowance");
        }

        private static void Trace()
        {
            var timing=LandingObservationTiming.Run;
            timing.Begin(PelvisDynamicReceiverIsolation.DynamicHipArmB);
            var history=new LandingEventHistory(); history.Reset();
            for (int n=0; n<18; ++n)
            {
                var step=Step(n); var src=Source(step);
                timing.SourceRotation.Observe(src,Quaternion.AngleAxis(n<15?0f:30f,Vector3.up),20f);
                var point=new FootLandingOpportunityAudit.Sample { Step=step,Source=src,Fixed=n+1,SupportId=6,FootSide="LEFT",SourcePhase="TURN",
                    SourceAge=.01f,DepthLatchFixed=15 };
                history.Add(point,n*.02,0);
                if (n==15) history.Capture(point,true);
            }
            history.EndMiss(LandingEventHistory.MissEnd.HoldEnded);
            var record=new StringBuilder(); history.Append(record);
            Check.That(record.ToString().Contains("/sourceRotation250ms=Jump/sourceMotion=TURN/rotationBoundaryLocal=15/rotationAtBoundary=Continuous"),
                "a jump after a native depth latch cannot explain the rejection");
            timing.Seal();

            foreach (bool missing in new[]{false,true})
            {
                var sim=new FootTransitionContracts.Sim();
                sim.Frame(0,0,0);
                PhysicsStepContext previous=default;
                sim.Frame(0,0,0,(ref Tick l,ref Tick r)=>{ l.Motion=r.Motion="IDLE"; previous=l.Step; });
                var pose=TurnHandoffContracts.Pose(.001f);
                PelvisNormalResponseEvidence settled=default;
                for (int n=3;n<=5;++n)
                {
                    PhysicsStepContext step=default;
                    sim.Frame(.02f,.03f,-.02f,(ref Tick l,ref Tick r)=>
                    {
                        TurnHandoffContracts.SetFree(ref l); TurnHandoffContracts.SetFree(ref r);
                        if (n==3) { TurnHandoffContracts.SetRelease(ref l); TurnHandoffContracts.SetRelease(ref r); }
                        if (n==5) { l.Active=l.Ordinary=true; l.EventFloor=l.Floor; }
                        l.Motion=r.Motion="TURN"; l.Source.World=Vector3.forward;
                        TurnHandoffContracts.SetPose(ref l,pose,0f);
                        l.Entry=new OrdinaryFootEntry.Sample {Local=n,Floor=n>=4?l.Floor:0,NativeSole=l.BoxMin,
                            NativeInside=l.Inside,LegacyReady=n>=4,Confirmed=n==5,IdentityHeld=n==5};
                        step=l.Step;
                    },held:n==4);
                    Check.That(sim.Trace.WantsRiseStep(step),"real strict release coordinator including terminal settlement");
                    var support=n==3 ? new SupportReferenceContext {ForStep=step,ObservedStep=previous,StanceMask=3,SupportId=6} : default;
                    var height=new ParentHeightProbe.Input {Arm=PelvisDynamicReceiverIsolation.DynamicHipArmB,Step=step,ReceiverReady=true,
                        Position=Vector3.up,Goal=Vector3.up,CommittedPosition=Vector3.up,ActualCom=Vector3.up,PositionLimit=.3f,
                        Support=support,SampledPelvisY=1.01f,FloorY=1.01f};
                    var frame=PelvisDynamicReceiverIsolation.Baseline(step);
                    frame.Acceleration=Vector3.zero; frame.UpwardRetentionSelected=frame.BoundedTradeSelected=true;
                    var left=HipAllocationContracts.Input(frame,true,default,default,default,default);
                    var right=HipAllocationContracts.Input(frame,false,default,default,default,default);
                    left.Seed.Fixed=right.Seed.Fixed=n+20;
                    Check.That(Rise.PlanPair(left,right,true,out var lc,out var rc),"native issued commands");
                    Check.That(Rise.TryTorso(lc,rc,out var torso),"native torso");
                    torso.Committed=true;
                    settled.Settled=step;
                    if (missing && n==5) settled.ChestId++;
                    var rise=new Rise.Input { Step=step,Support=support };
                    sim.Trace.ObserveRise(step,rise,left,right,lc,rc,torso,settled,height:height);
                    settled=new PelvisNormalResponseEvidence {Valid=true,Input=step,Fixed=n+201,PelvisId=frame.BodyId,
                        ChestId=frame.ChestBodyId,Request=0f,Allocated=torso.TargetSystemAcceleration.y,UpwardSelected=true,
                        UpwardExtra=lc.UpwardExtraAcceleration,Q=lc.MotionScale,Observed=torso.TargetSystemAcceleration.y};
                    previous=step;
                }
                string log=sim.Report();
                Check.That(log.Contains("H180_INPUT/start-issue=3|4/epoch-fixed-source=3|4|3") && log.Contains("/committedPosition=0|1|0") &&
                    log.Contains("H180/start-issue-read=3|3|4/state=Seed/read=Matched"),"release seed and round-trip command inputs");
                Check.That(log.Contains("H180/start-issue-read=3|4|5/state=Compared/read="+(missing?"ResponseMissing":"Matched")) &&
                    log.Contains("/compared-nextMatched=1|"+(missing?"0":"1")+"/model="),"only the original next physical solve counts");
                Check.That(log.Contains("/actualSole-gapMm=")==!missing,"no shoe geometry credited to a mismatched solve");
            }
        }

        private static void Anchors()
        {
            using var reader=new StreamReader(typeof(TurnHeightContracts).Assembly.GetManifestResourceStream("ARL.Contracts.TurnHeightAnchors179"));
            reader.ReadLine(); int good=0,miss=0,latest=0,noWitness=0,issued=0; string line;
            while ((line=reader.ReadLine())!=null)
            {
                var a=line.Split('\t'); float F(int i)=>float.Parse(a[i],CultureInfo.InvariantCulture);
                bool passed=a[4]=="Ordinary"; if (passed) good++; else miss++;
                Check.That(F(3)-F(2)==3f && F(10)<4.75f,"ankle has room at first strong before confirmed entry");
                Check.That(passed==(F(9)>=-2f) && passed==(F(11)>=-2f && F(12)<=5f),"frozen native results, common translation alone preserves ankle gap");
                if(a[0]=="179") { latest++; noWitness+=int.Parse(a[5]); issued+=int.Parse(a[6]); }
            }
            Check.That(good==1 && miss==6 && latest==5 && noWitness==48 && issued==63,"historical success control and failed releases");
        }
    }
}
