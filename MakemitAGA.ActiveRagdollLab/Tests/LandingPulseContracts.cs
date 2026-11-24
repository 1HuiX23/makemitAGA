using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Rise = MakemitAGA.ActiveRagdollLab.Physics.LandingRiseAllocation;
using Command = MakemitAGA.ActiveRagdollLab.Model.HipAnchorPairReport.Command;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LandingPulseWindow.Foot;

namespace ARL.Contracts
{
    internal static class LandingPulseContracts
    {
        internal static void Run()
        {
            Check.Run("173 issue-only replay of 62 commands and five real 172 releases", Recorded);
            Check.Run("173 both native feet, first opportunity, source continuity and single-use event", Window);
            PelvisDynamicReceiverIsolation.RunLandingPulseContracts();
        }
        private static float Number(string value) => value == "NA" ? float.NaN : float.Parse(value,CultureInfo.InvariantCulture);

        private static void Recorded()
        {
            using var data = new StreamReader(typeof(LandingPulseContracts).Assembly.GetManifestResourceStream("ARL.Contracts.LandingPulse172"));
            data.ReadLine(); string row;
            var releases = new HashSet<string>();
            var issues = new Dictionary<string,string>(); int count=0;
            while ((row=data.ReadLine()) != null)
            {
                var c=row.Split('\t'); count++; releases.Add(c[0]);
                if (issues.ContainsKey(c[0]) || c[2]!="Compared" || c[3]!="0") continue;
                var selected=Rise.Choose(Number(c[4]),Number(c[5]),Number(c[6]),Number(c[7]));
                if (selected!=Rise.Choice.None) issues.Add(c[0],c[1]+"|"+selected);
            }
            Check.That(count==62 && releases.Count==5 && issues.Count==3 && !issues.ContainsKey("2569") && !issues.ContainsKey("4843"),"two controls correctly stay native");
            Check.That(issues["3418"]=="3426|Retention" && issues["4172"]=="4179|Request" && issues["5727"]=="5732|Request","172 issue-time selections");
            Check.That(Rise.Choose(.001f,-1f,2f,3f)==Rise.Choice.None && Rise.Choose(-1f,.001f,2f,3f)==Rise.Choice.None &&
                Rise.Choose(0f,0f,0f,0f)==Rise.Choice.None && Rise.Choose(-1f,0f,0f,.001f)==Rise.Choice.Retention &&
                Rise.Choose(-1f,0f,float.NaN,1f)==Rise.Choice.None,"strict positive crossings and usable request-only alternative");
        }
        private static Foot Tick(int n,bool left)
        {
            var step=PelvisDynamicReceiverIsolation.LoggedStep(n);
            step.SourceSequence=200+n;
            return new Foot { Step=step, Source=new SourceStepContext { Epoch=step.Epoch, Sequence=step.SourceSequence,
                Time=n/60f, Dt=1f/60f }, SourceContinuity=1, Age=.01f, Root=9, Body=left?20:21, Floor=6, Local=n };
        }
        private static bool Observe(LandingPulseWindow window, string phase, Foot l, Foot r)
        {
            var cmd=new Command { Step=l.Step, Fixed=l.Local, ChildId=31, ParentId=7, JointSolved=true, ParentV=.01f*Vector3.up };
            cmd.ParentInput.ChestBodyId=8;
            var rc=cmd; rc.ChildId=32;
            return window.Advance(l.Step,phase,l,r,cmd,rc);
        }
        private static void Window()
        {
            for (int fault=-1;fault<13;fault++)
            {
                var lines=new List<string>(); var w=new LandingPulseWindow(lines.Add);
                var l=Tick(1,true); var r=Tick(1,false);
                l.Active=r.Active=true; Observe(w,"IDLE",l,r);
                l=Tick(2,true); r=Tick(2,false);
                l.Was=r.Was=l.LiftRelease=r.LiftRelease=true;
                if (fault==0) r.LiftRelease=false;
                if (fault==1) l.SourceContinuity++;
                if (fault==2) r.Floor++;
                if (fault==3) l.Age=.5f;
                if (fault==4) r.Root++;
                if (fault==5) r.Body=l.Body;
                if (fault==6) l.Step.Epoch++;
                if (fault==7) l.Local++;
                if (fault==8) r.Precontact=true;
                if (fault==9) { r.Active=true; r.LiftRelease=false; }
                if (fault==10) l.Miss=true;
                if (fault==11) l.Source.Time-=1f;
                if (fault==12) w.Reset("Stop");
                bool free=Observe(w,"TURN",l,r);
                Check.That(free==(fault==-1),"strict release and current foot gates "+fault);
                if (!free) continue;
                var candidate=new Rise.Allocation { Kind=Rise.Choice.Request };
                w.RecordIssue(l.Step,candidate,-.02f,true); w.RecordIssue(l.Step,candidate,-.02f,true);
                Check.That(!Observe(w,"TURN",l,r) && lines.Count(s=>s.StartsWith("ISSUE"))==1,"no duplicate within solve");
                l=Tick(3,true);r=Tick(3,false);l.Miss=l.PendingMiss=true;
                l.HasEntryGeometry=true;l.Sole=-.003f;l.Gap=.005f;
                Check.That(!Observe(w,"TURN",l,r) && lines.Any(s=>s.Contains("READ/start-from-to=2|2|3")) &&
                    lines.Any(s=>s.Contains("FIRST/start-issue-first=2|2|3/entry=Miss/sole-gapMm=-3|5")),"adjacent read even on the first miss");
                l=Tick(4,true);r=Tick(4,false);l.Active=l.Recovery=true;
                Check.That(!Observe(w,"TURN",l,r) && lines.Any(s=>s.Contains("2|2|3|4/result=Recovered")),"actual recovery joins original release");
                w.Reset("Stop"); Check.That(lines.Count(s=>s.StartsWith("END"))==1,"stop after closed event");
            }
            var ordinary=new List<string>(); var native=new LandingPulseWindow(ordinary.Add);
            var before=Tick(1,true);var peer=Tick(1,false);before.Active=peer.Active=true;
            Observe(native,"IDLE",before,peer);
            before=Tick(2,true);peer=Tick(2,false);before.Was=peer.Was=before.LiftRelease=peer.LiftRelease=true;
            Observe(native,"TURN",before,peer);
            before=Tick(3,true);peer=Tick(3,false);before.Active=before.Ordinary=true;
            Observe(native,"TURN",before,peer);
            Check.That(ordinary.Any(s=>s.Contains("2|0|3|3/result=Ordinary")),"a first ordinary entry closes a no-issue control");
        }
    }
}

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class PelvisDynamicReceiverIsolation
    {
        internal static void RunLandingPulseContracts()
        {
            ARL.Contracts.Check.Run("173 both Hip owners and receiver apply only the selected net wrench, with trace on and off", ReleaseWrites);
            ARL.Contracts.Check.Run("173 reject each partial native write, cancel both Hip pairs and finalize each receipt once", ReleaseFailures);
        }

        private static void PulseFixture(bool retain, bool primaryLeft, bool trace, out PelvisDynamicReceiverIsolation receiver,
            out HipAnchorTranslationPair left, out HipAnchorTranslationPair right, out Rise.Allocation choice)
        {
            var step=LoggedStep();
            var frame=Baseline(step); frame.Acceleration=4f*Vector3.up;
            frame.UpwardRetentionSelected=frame.BoundedTradeSelected=true;
            var i=new Rise.Input { Step=step, Error=.002f, RefV=.15f, PreV=-.02f, RefA=3f };
            var li=ARL.Contracts.HipAllocationContracts.Input(frame,true,6f*Vector3.up,Vector3.zero,Vector3.up*.2f,-Vector3.right*.1f);
            var ri=ARL.Contracts.HipAllocationContracts.Input(frame,false,6f*Vector3.up,Vector3.zero,Vector3.up*.2f,Vector3.right*.1f);
            ARL.Contracts.Check.That(HipBilateralForceSolver.TryPlan(li,ri,out var lc,out var rc),"baseline solve");
            ARL.Contracts.Check.That(Rise.TryTorso(lc,rc,out var torso),"native torso");
            torso.Committed=true;
            var plans=Rise.Compare(step,6,true,i,li,ri,lc,rc,torso);
            float low=retain?plans.RequestOnly.SocketA:plans.Native.SocketA;
            float high=retain?plans.EarlyRetention.SocketA:plans.RequestOnly.SocketA;
            ARL.Contracts.Check.That(plans.Status==Rise.State.Compared && high>low && high>0f,"fixture brackets a crossing");
            var pre=-step.Dt*(low+high)*.5f*Vector3.up;
            li.Seed.ParentV=ri.Seed.ParentV=pre;
            receiver=new PelvisDynamicReceiverIsolation(); receiver.Produce(step,trace); receiver._hipCommand=frame; receiver.LandingRiseInput=i;
            receiver._pelvis.velocity=pre;
            receiver._pelvis.AddForce(receiver._pelvis.mass*frame.Acceleration,ForceMode.Force);
            receiver._chest.AddForce(receiver._chest.mass*frame.Acceleration,ForceMode.Force);
            receiver._pelvis.AddTorque(frame.ReceiverTorque,ForceMode.Force);
            left=HipAnchorTranslationPair.Staged(li,lc,receiver._pelvis,trace);
            right=HipAnchorTranslationPair.Staged(ri,rc,receiver._pelvis,trace);
            var primary=primaryLeft?left:right;var secondary=primaryLeft?right:left;
            ARL.Contracts.Check.That(HipAnchorTranslationPair.CommitBilateral(step,primary,secondary,true) &&
                receiver.CompleteHipTranslationReaction(step,primary.Report,secondary.Report,true),"original actuators completed before foot gate");
            ARL.Contracts.Check.That(Rise.TryChoose(step,6,i,li,ri,left.Report.Current,right.Report.Current,receiver._hipAttitudeCommand,primaryLeft,out choice) &&
                choice.Kind==(retain?Rise.Choice.Retention:Rise.Choice.Request),"single selected plan through both real Hip inputs");
            var foreign=i; foreign.Support=new SupportReferenceContext { ForStep=step, ObservedStep=LoggedStep(-1), StanceMask=1,SupportId=6 };
            ARL.Contracts.Check.That(!Rise.TryChoose(step,6,foreign,li,ri,left.Report.Current,right.Report.Current,receiver._hipAttitudeCommand,primaryLeft,out _),
                "prior support witness keeps native plan");
        }

        private static void NearVector(Vector3 actual,Vector3 expected,string reason) =>
            ARL.Contracts.Check.Near((actual-expected).magnitude,0f,.00008f,reason);

        private static void ReleaseWrites()
        {
            foreach (var arm in new[] { DynamicHipArmA, DynamicHipArmB, "LEGACY" })
            foreach (bool trace in new[]{true,false})
            foreach (bool leftFirst in new[]{true,false})
            foreach (bool retain in new[]{true,false})
            {
                PulseFixture(retain,leftFirst,trace,out var p,out var l,out var r,out var next);
                var step=next.Left.Step; var native=l.Report.Current;
                p._control.Arm = arm;
                bool selected = arm != "LEGACY";
                ARL.Contracts.Check.That(p.ApplyLandingPulse(step,6,l,r,leftFirst,next)==selected,"both formal arms use the verified pulse");
                var lc=selected?next.Left:native;
                var rc=selected?next.Right:r.Report.Current;
                var target=p._hipAttitudeCommand;
                NearVector(l.TestUpper.ForceSum,lc.Force,"left point force");
                NearVector(r.TestUpper.ForceSum,rc.Force,"right point force");
                NearVector(l.TestUpper.TorqueSum,lc.ChildTorque+lc.UpperActuatorTorque,"left total torque");
                NearVector(r.TestUpper.TorqueSum,rc.ChildTorque+rc.UpperActuatorTorque,"right total torque");
                NearVector(p._pelvis.ForceSum,p._pelvis.mass*target.CommonAcceleration-lc.Force-rc.Force,"pelvis total force");
                NearVector(p._chest.ForceSum,p._chest.mass*target.CommonAcceleration,"chest total force");
                NearVector(p._pelvis.TorqueSum,lc.ParentTorque+rc.ParentTorque+target.AppliedTorque,"pelvis total torque");
                ARL.Contracts.Check.That(target.Matches(p._hipCommand) && target.Baseline.SameAs(lc.ParentInput),"next-step feedback uses final request and allocation");
                if (selected) ARL.Contracts.Check.That(!p.ApplyLandingPulse(step,6,l,r,leftFirst,next),"duplicate pulse rejected before finalizing");
                l.FinishFootDecision(step); r.FinishFootDecision(step); p.FinishFootDecision(step);
                l.FinishFootDecision(step); r.FinishFootDecision(step); p.FinishFootDecision(step);
                ARL.Contracts.Check.That(p.Report.HipReactionRecords==(trace?1:0) && l.Report.Writes==(trace?1:0) && r.Report.Writes==(trace?1:0),"one evidence row per owner");
                if (trace)
                {
                    var row=p.Report.Ring[p._pending];
                    NearVector(row.HipLeftJ,-(leftFirst?lc.Force:rc.Force)*step.Dt,"primary final input receipt");
                    NearVector(row.HipRightJ,-(leftFirst?rc.Force:lc.Force)*step.Dt,"secondary final input receipt");
                    NearVector(row.ChestJ,p._chest.mass*target.CommonAcceleration*step.Dt,"chest final receipt");
                    ARL.Contracts.Check.That(row.HipAttitude.Matches(lc.ParentInput) && row.HipEvidenceFinalized,"trace finalized");
                }
                ARL.Contracts.Check.That(l.CancelQueued(step) && r.CancelQueued(step) && l.CancelQueued(step),"cancel selected pairs once");
                NearVector(l.TestUpper.ForceSum,Vector3.zero,"left selected cancellation"); NearVector(r.TestUpper.TorqueSum,Vector3.zero,"right selected cancellation");
                NearVector(p._pelvis.ForceSum,p._pelvis.mass*target.CommonAcceleration,"only receiver force remains after cancelling both pairs");
            }
        }

        private static void ReleaseFailures()
        {
            for (int fail=1;fail<=9;fail++)
            {
                PulseFixture(true,true,true,out var p,out var l,out var r,out var next);
                var bodies=new[]{l.TestUpper,r.TestUpper,p._pelvis,p._chest};
                var oldF=bodies.Select(x=>x.ForceSum).ToArray();var oldT=bodies.Select(x=>x.TorqueSum).ToArray();
                int calls=0;
                foreach (var body in bodies) body.BeforeWrite=()=> { if (++calls==fail) throw new InvalidOperationException("fault"); };
                var native=p._hipCommand;
                ARL.Contracts.Check.That(!p.ApplyLandingPulse(native.Step,6,l,r,true,next) && calls==2*fail-1 &&
                    p.HipCompletionIssue=="RISE_PULSE_WRITE_FAILED_UNDO_FAILURES_0","reversed only the returned calls "+fail);
                for (int k=0;k<bodies.Length;k++)
                {
                    NearVector(bodies[k].ForceSum,oldF[k],"restored force "+k);
                    NearVector(bodies[k].TorqueSum,oldT[k],"restored torque "+k);
                }
                ARL.Contracts.Check.That(l.Report.Current.ParentInput.SameAs(native) && r.Report.Current.ParentInput.SameAs(native) &&
                    p._hipAttitudeCommand.Matches(native) && p.Report.HipReactionRecords==0,"failed candidate never published");
            }
            PulseFixture(true,true,true,out var broken,out var left,out var right,out var plan);
            int attempted=0;
            foreach (var body in new[]{left.TestUpper,right.TestUpper,broken._pelvis,broken._chest})
                body.BeforeWrite=()=> { attempted++; if (attempted==9 || attempted==11) throw new InvalidOperationException("native/undo"); };
            ARL.Contracts.Check.That(!broken.ApplyLandingPulse(plan.Left.Step,6,left,right,true,plan) && attempted==17 &&
                !broken.ReadyForRun && broken.HipCompletionIssue=="RISE_PULSE_WRITE_FAILED_UNDO_FAILURES_1",
                "all remaining undos attempted after failed rollback and receiver marked unsafe");
        }
    }
}
