using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;
using Entry = MakemitAGA.ActiveRagdollLab.Physics.OrdinaryFootEntry;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LandingFootTraction;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;
using Trial = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;

namespace ARL.Contracts
{
    internal static class ContinuousLandingContracts
    {
        internal static void Run()
        {
            Check.Run("174 issued knee transport and revised hip wrench have independent receipts", Angular);
            Check.Run("174 ordinary contact preserves the strong-interval latch, geometry hold and exact support", Ordinary);
            Check.Run("174 a single carried foot path is scored before confirmation and stops at its first hypothetical contact", Path);
            Check.Run("174 wrong entry receipts, interrupted observation and an unread endpoint stay ungraded", Boundaries);
            Check.Run("174 both 173 runs replay only captured unloaded solves", Recorded);
        }

        private static void Angular()
        {
            var issued = Vector3.right * 11f;
            var relative = Vector3.up * -2f;
            var final = issued + Vector3.forward * 3f;
            var receipt = new KneeAngularReceipt(issued,true);
            Check.That(receipt.Matches(true,issued,issued,relative,issued+relative), "unmodified solve closes");
            Check.That(!receipt.Matches(true,final,final,relative,issued+relative), "unexpected revision rejected");
            receipt.HipReallocated = true;
            Check.That(receipt.Matches(true,final,final,relative,issued+relative) && receipt.Parent.x==11f, "late pulse retains original parent");
            Check.That(!receipt.Matches(true,final,final,relative,final+relative), "revised parent cannot stand in for issued knee");
            Check.That(!receipt.Matches(true,final,issued,relative,issued+relative), "wrong final hip rejected");
            Check.That(!receipt.Matches(false,final,final,relative,issued+relative), "mode mismatch rejected");
            var off = new KneeAngularReceipt(issued,false) { HipReallocated=true };
            Check.That(off.Matches(false,final,final,relative,relative) && off.Parent.sqrMagnitude==0,"disabled transport stays zero");
            Check.That(!default(KneeAngularReceipt).Matches(false,Vector3.zero,Vector3.zero,Vector3.zero,Vector3.zero),"no fabricated zero receipt");

            var frame = new RightKneePivot.Frame { Dt=.02f, UpperHip=Vector3.up, UpperKnee=Vector3.up*.5f,
                LowerKnee=Vector3.up*.5f, LowerAnkle=Vector3.right*.1f, FootAnkle=Vector3.right*.1f };
            var m = new RightThighTracking.Motion { UpperMaxW=18f, LowerMaxW=18f, KneeAngular=receipt, KneeRelativeA=relative };
            var cmd = new HipAnchorPairReport.Command { AllocatedHipAlpha=final, ChildMass=2f };
            RightThighTracking.ProjectAnchors(frame,m,default,cmd,out var knee,out var ankle,out _,out var turn);
            var lowerTurn = FootRelativeReference.Advance(Quaternion.identity,(issued+relative)*frame.Dt,frame.Dt);
            var upperTurn = FootRelativeReference.Advance(Quaternion.identity,final*frame.Dt,frame.Dt);
            Check.Near((ankle-knee-lowerTurn*(frame.LowerAnkle-frame.LowerKnee)).magnitude,0f,1e-6f,"lower uses the earlier vector");
            Check.Near((turn*Vector3.up-upperTurn*Vector3.up).magnitude,0f,1e-6f,"upper uses the revised vector");
        }

        private static void Ordinary()
        {
            var trail = new Entry.Trail();
            var s = new Entry.Sample { Local=10, Floor=4, LegacyReady=true };
            Check.That(trail.Observe(s,true,-.002f)==Entry.Decision.Waiting && trail.DepthLatch==0,"inclusive ordinary depth");
            s.Local++;
            Check.That(trail.Observe(s,true,-.00201f)==Entry.Decision.Waiting && trail.DepthLatch==11,"pre-confirmation depth saved");
            s.Local++; s.IdentityHeld=s.Confirmed=true;
            Check.That(trail.Observe(s,true,.001f)==Entry.Decision.Miss && trail.ReadySamples==0,"subsequent good height cannot clear this miss");
            s.Local++; s.Floor=9;
            Check.That(trail.Observe(s,true,0f)==Entry.Decision.Waiting && trail.StrongStart==13,"changed floor starts at one");
            s.Local++; s.LegacyReady=false;
            Check.That(trail.Observe(s,true,0f)==Entry.Decision.Waiting && trail.ReadySamples==0,"owner ground veto resets geometry hold");
            s.Local++; s.LegacyReady=true;
            Check.That(trail.Observe(s,true,.025f)==Entry.Decision.Waiting,"ceiling inclusive; two samples required");
            s.Local++; s.IdentityHeld=false;
            Check.That(trail.Observe(s,true,0f)==Entry.Decision.Waiting,"identity hold required");
            s.Local++; s.IdentityHeld=true;
            Check.That(trail.Observe(s,true,0f)==Entry.Decision.Ordinary,"all ordinary gates");
            s.Local++; s.Floor=0;
            Check.That(trail.Observe(s,true,-.03f)==Entry.Decision.Waiting && trail.DepthLatch==0 && trail.StrongSamples==0,"deep swing outside eligible interval");
        }

        internal static Trace.Tick Tick(int local, Trial.Pose p, int strong, int first, int nativeLatch = 0)
        {
            var t=TurnHandoffContracts.Tick(local,p);
            t.Entry=new Entry.Sample { Local=local, Floor=local>=strong ? t.Floor : 0, LegacyReady=local>=strong,
                NativeInside=t.Inside, NativeSole=t.BoxMin,
                Confirmed=local>=first, IdentityHeld=local>=first };
            if (local==first)
            {
                t.EventFloor=t.Floor; t.NativeDepthLatch=nativeLatch;
                t.Miss=nativeLatch!=0; t.Active=t.Ordinary=!t.Miss;
            }
            return t;
        }

        private static string Report(TurnHandoffWindow window, Trace.Tick first, int strong)
        {
            var b = new StringBuilder();
            window.AppendContinuous(b,first,first.Miss ? "HoldEnded" : "Ordinary",0,strong);
            return b.ToString();
        }

        private static Foot.Input Input(float cap=60f) => new Foot.Input {
            Tether=new RightAnkleLandingSequence.Tether { BaseCap=cap,SelectedCap=cap,HardGap=.03f,Kp=100f,Kd=10f } };

        private static void Path()
        {
            const int start=10, strong=13, first=16;
            var p=TurnHandoffContracts.Pose(-.0024f);
            var window=new TurnHandoffWindow(start,Vector3.forward,TurnHandoffWindow.Policy.LeftBraked);
            Trace.Tick t=default;
            for (int n=start;n<=first;n++)
            {
                t=Tick(n,p,strong,first,strong);
                if (window.Visit(t,p,22,23,null))
                    window.Issue(t,p,TurnHandoffContracts.Budget(),Input());
            }
            var log=Report(window,t,strong);
            Check.That(log.Contains("/sequenceEnd=WouldEnter/stopLocal=16") && log.Contains("strongStart-nativeLatch-trialLatch=13|13|0") &&
                log.Contains("/strongSteps-marginPass-readySamples=4|4|4"), "complete trial ordinary while real shoe misses");
            Check.That(log.Contains("/safeReads=6") && log.Split("C174/").Length==7, "first confirmed sample settles before stopping");
            var rejected=new TurnHandoffWindow(start,Vector3.forward,TurnHandoffWindow.Policy.LeftBraked);
            for (int n=start;n<=first;n++)
            {
                t=Tick(n,p,strong,first,strong);
                if (rejected.Visit(t,p,22,23,null))
                    rejected.Issue(t,p,TurnHandoffContracts.Budget(),n==start+1 ? default : Input());
            }
            var loss=Report(rejected,t,strong);
            Check.That(loss.Contains("/sequenceEnd=MissingInput/stopLocal=11/safeReads=1") && loss.Split("C174/").Length==2,
                "later valid receipts cannot reseed a broken chain");
            var bad=new TurnHandoffWindow(start,Vector3.forward,TurnHandoffWindow.Policy.LeftBraked);
            for (int n=start;n<=first;n++)
            {
                t=Tick(n,p,strong,first,strong+1);
                if (bad.Visit(t,p,22,23,null)) bad.Issue(t,p,TurnHandoffContracts.Budget(),Input());
            }
            Check.That(Report(bad,t,strong).Contains("/sequenceEnd=NativeDisagreement"), "measured latch must reproduce native decision");
        }

        private static void Boundaries()
        {
            const int start=3;
            var pose=TurnHandoffContracts.Pose();
            foreach (string reason in new[] { "WrongStep", "WrongFloor", "WrongSole", "OtherEntry", "AlreadyActive", "Seal" })
            {
                var window=new TurnHandoffWindow(start,Vector3.forward,TurnHandoffWindow.Policy.LeftBraked);
                var t=Tick(start,pose,4,7);
                Check.That(window.Visit(t,pose,22,23,null),"start bounded trial");
                window.Issue(t,pose,TurnHandoffContracts.Budget(),Input());
                if (reason=="Seal") window.Stop("TraceEnded");
                else
                {
                    t=Tick(start+1,pose,4,7);
                    if (reason=="WrongStep") t.Entry.Local--;
                    if (reason=="WrongFloor") t.Entry.Floor++;
                    if (reason=="WrongSole") t.Entry.NativeSole+=.003f;
                    if (reason=="AlreadyActive") t.Active=true;
                    window.Visit(t,pose,22,23,reason=="OtherEntry" ? reason : null);
                }
                var log=Report(window,default,0);
                string expected=reason switch {
                    "Seal" => "TraceEnded", "OtherEntry" => reason, "WrongSole" => "EntryGeometryMismatch", _ => "EntryReceipt"
                };
                Check.That(log.Contains("/sequenceEnd="+expected) && !log.Contains("WouldEnter"),"no admission for "+reason);
                if (reason=="Seal") Check.That(log.Contains("/issue-read=3|0/status=Predicted"),"queued forecast has no read");
            }
        }

        private static float F(string v) => float.Parse(v,CultureInfo.InvariantCulture);
        private static Vector3 V(string v)
        {
            var a=v.Split(',').Select(F).ToArray();
            return new Vector3(a[0],a[1],a[2]);
        }

        // Each row retains the real issue-time angular/linear budgets and two distinct aims.
        // Paired nominal endpoints substitute for unprinted rigid poses (under 2 micrometres).
        // The 173 compact log did not export the owner's legacy-geometry/identity-hold booleans.
        private static void Recorded()
        {
            using var reader=new StreamReader(typeof(ContinuousLandingContracts).Assembly.GetManifestResourceStream("ARL.Contracts.ContinuousLanding173"));
            reader.ReadLine();
            var events=new List<string[]>();
            int rows=0, groups=0, fourStepWindows=0;
            string line;
            while ((line=reader.ReadLine())!=null)
            {
                var c=line.Split('\t');
                if (events.Count>0 && events[0][0]!=c[0]) { Replay(events,ref fourStepWindows); events.Clear(); groups++; }
                events.Add(c); rows++;
            }
            if (events.Count>0) { Replay(events,ref fourStepWindows); groups++; }
            Check.That(rows==103 && groups==10 && fourStepWindows==5,"all printed rows including the five complete independent comparison windows");
        }

        private static void Replay(List<string[]> rows, ref int fourStepWindows)
        {
            int start=int.Parse(rows[0][0]), strong=int.Parse(rows[0][20]), first=int.Parse(rows[0][21]), independent=0;
            var arm=rows[0][23];
            var init=TurnLandingContracts.DecodeIssue(rows[0]);
            var window=new TurnHandoffWindow(start,init.Issue.Travel,TurnHandoffWindow.Policy.LeftBraked);
            var current=init.Pose; current.Hip.y=init.HipY;
            current.Lower=current.Foot-V(rows[0][17]);
            Trace.Tick t=Tick(start,current,strong,first);
            t.Step.Dt=init.Dt; t.Target=init.Issue.Target; t.SourceSole=init.Issue.TargetSole;
            Check.That(window.Visit(t,current,22,23,null),"issue release");
            foreach (var c in rows)
            {
                var s=TurnLandingContracts.DecodeIssue(c); int depth=int.Parse(c[1]),to=start+depth;
                var raw=c[15].Split('|'); var limits=c[16].Split('|').Select(F).ToArray();
                var command=new Foot.Input { Applied=V(raw[1]), Drag=limits[4], Tether=new RightAnkleLandingSequence.Tether {
                    Raw=V(raw[0]),BaseCap=limits[0],SelectedCap=limits[1],Kp=limits[2],Kd=limits[3],HardGap=limits[5]*.001f } };
                Check.That((t.Target-s.Issue.Target).magnitude<.000002f && (t.SourceSole-s.Issue.TargetSole).magnitude<.000002f,"next aim arrives only at read");
                window.Issue(t,s.Pose,s.Budget,command);
                t=Tick(to,s.Pose,strong,first,strong);
                var aim=TurnLandingContracts.ReadAim(c[9],s);
                t.Target=aim.Target; t.SourceSole=aim.TargetSole; t.Step.Dt=s.Dt;
                Check.Near(t.BoxMin*1000f,F(c[22]),.002f,"captured native box");
                t.BoxMin=t.Entry.NativeSole=F(c[22])*.001f;
                window.Visit(t,s.Pose,22,23,null);
                bool twoKnown=window.TryTwoStepMargin(to,true,out var two);
                Check.That((twoKnown ? two.MarginPass.ToString() : "NA")==c[24],"reproduce independently seeded comparison");
                if (to>=strong && twoKnown && two.MarginPass) independent++;
            }
            if (independent==4) fourStepWindows++;
            // Old blackouts have a known physical first miss; its missing intermediate
            // commands are intentionally absent from this replay.
            var native=Tick(first,init.Pose,strong,first,strong);
            if (window.End==null) window.Stop("EndOfRecordedInputs");
            var log=Report(window,native,strong);
            var summary=log.TrimStart().Split('\n')[0];
            Check.That(!summary.Contains("NativeDisagreement") && !summary.Contains("EntryReceipt"), "eligible native inputs reproduce when covered");
            if (t.Local==first)
            {
                Check.That(summary.Contains("/sequenceEnd=WouldEnter/stopLocal="+first) &&
                    summary.Contains("/strongSteps-marginPass-readySamples=4|4|4"),"same continuous path covers all four real endpoints");
                var end=log.TrimEnd().Split('\n').Last();
                string velocity=end.Split("/footDeltaVMps=")[1].Split('/')[0];
                var carried = V(velocity);
                Check.That(window.TryTwoStepMargin(first,true,out var two) &&
                    (two.Body.Velocity-carried).magnitude>.00001f,"continuous state differs from independently reseeded two-step velocity");
                if (arm=="B" && start==2507)
                {
                    Check.Near(two.Body.Velocity.y,.04394f,.00002f,"recorded two-step vertical increment");
                    Check.Near(carried.y,-.02628f,.00002f,"continuous vertical increment changes sign");
                }
            }
            else Check.That(!summary.Contains("sequenceEnd=WouldEnter"),"censored run cannot earn an admission");
        }
    }
}
