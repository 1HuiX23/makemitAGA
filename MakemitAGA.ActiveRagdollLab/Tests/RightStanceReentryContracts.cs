using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;

namespace ARL.Contracts
{
    internal static partial class FootTransitionContracts
    {
        private const string Reentry = "RIGHT_STANCE_REENTRY_V041830", StrongExit = "RELEASE_ON_STRONG_STANCE_EXIT";
        private static void RunRightReentryContracts()
        {
            Check.Run("183 historical steady misses and ordinary turn match first observed depths, native latch and peer", CapturedReentries);
            Check.Run("183 tilted source and selected poses project the same eight actual corners", ReentryBoxes);
            Check.Run("183 release ownership, missing reads, exact depth threshold and phase changes", ReleaseEligibility);
            Check.Run("183 paired callback order, duplicate or stale halves and conflicting held source clocks", ReentryClocks);
            Check.Run("183 missing stance entry or delayed resolution is bounded with explicit coverage", ReentryCoverage);
        }

        private static Trace.Tick Receipt(int n, int shared, int source, bool right = true)
        {
            return new Trace.Tick {
                Local = n, Body = right ? 12 : 11, Floor = 42, PriorFloor = 42, EventFloor = 42,
                Step = new PhysicsStepContext { Epoch=3, Sequence=shared, SourceSequence=source, FixedTime=shared/60f, UnscaledTime=shared/60f, Dt=1f/60f },
                Source = new FootTransitionSource.Sample { Root=7, Continuity=1, Valid=true,
                    Clock=new SourceStepContext { Epoch=3, Sequence=source, Time=source/60f, Dt=1f/60f } },
                Gait = GaitReceipt(true), Fresh=true, Inside=true, Phase=Phase.Stance, Motion="STEADY",
                Was=true, Active=true, QualifiedIdentity=true, RightReference=Trace.Reference.Entry
            };
        }
        private static FootGaitEvidence GaitReceipt(bool strong) => new FootGaitEvidence(5,true,42,0f,Vector3.up,0f,strong?.1f:.4f);
        private static void SendPair(Trace trace, in Trace.Tick right, bool peer = true)
        {
            var left=Receipt(right.Local,right.Step.Sequence,right.Source.Clock.Sequence,false);
            left.Step=right.Step; left.Source.Clock=right.Source.Clock;
            left.Floor=left.PriorFloor=left.EventFloor=right.Floor; left.Body=right.Body-1;
            left.Active=left.Was=peer;
            if(!peer) left.Phase=Phase.Swing;
            if((right.Local&1)==0) { trace.Submit(0,left); trace.Submit(1,right); }
            else { trace.Submit(1,right); trace.Submit(0,left); }
        }
        private static Trace.Tick Released(int n)
        {
            var t=Receipt(n,n,n); t.Active=false; t.ReleaseReason=StrongExit; t.Gait=GaitReceipt(false); return t;
        }
        private static Trace.Tick Inactive(int n)
        {
            var t=Receipt(n,n,n); t.Was=t.Active=false; return t;
        }
        private static string Finish(Trace trace) { trace.Seal(); return trace.Summary(); }

        private static void CapturedReentries()
        {
            using var data=typeof(FootTransitionContracts).Assembly.GetManifestResourceStream("ARL.Contracts.RightStanceReentry182");
            using var reader=new StreamReader(data);
            var groups=new Dictionary<int,Trace>();
            string line;
            while((line=reader.ReadLine())!=null)
            {
                if(line.StartsWith("#")) continue;
                var c=line.Split('\t');
                int I(int i) => int.Parse(c[i],CultureInfo.InvariantCulture);
                float Mm(int i) => float.Parse(c[i],CultureInfo.InvariantCulture)*.001f;
                int origin=I(1);
                if(!groups.TryGetValue(origin,out var trace)) groups.Add(origin,trace=new Trace());
                var t=Receipt(I(4),I(3),I(5));
                t.Step.Epoch=t.Source.Clock.Epoch=I(2); t.Body=I(7);
                t.Floor=t.PriorFloor=t.EventFloor=I(6);
                t.Gait=new FootGaitEvidence(5,true,t.Floor,0f,Vector3.up,0f,I(9)==1?.1f:.4f);
                t.Motion=c[8]; t.BoxMin=t.RightEntryMin=Mm(12); t.Foot=Vector3.right*Mm(13);
                t.RightReference=c[10]=="NA" ? Trace.Reference.Unavailable : Trace.Reference.Entry;
                if(t.RightReference!=Trace.Reference.Unavailable) { t.SourceMin=Mm(10); t.ControlMin=Mm(11); }
                t.Was=c[0] is "Before" or "Release"; t.Active=c[0] is "Before" or "Ordinary";
                t.ReleaseReason=c[0]=="Release" ? StrongExit : null;
                t.Ordinary=c[0]=="Ordinary"; t.Miss=t.PendingMiss=c[0]=="Miss";
                t.NativeDepthLatch=t.Miss?t.Local:0;
                SendPair(trace,t,I(14)!=0);
            }
            var expected=new[] {
                (2692, "2704|HoldEnded|2705", "2704|0|2704", "2693|AlreadyBelowAtFirstRead", "2700|Crossed", "2699|Crossed", "13|0|0|0|1"),
                (4394, "4406|HoldEnded|4407", "4406|0|4406", "4395|AlreadyBelowAtFirstRead", "4400|Crossed", "4400|Crossed", "13|0|0|0|1"),
                (3228, "3239|Ordinary|3239", "3236|0|0", "0|NoBelowObserved", "0|NoBelowObserved", "0|NoBelowObserved", "12|12|0|0|1") };
            foreach(var (origin,outcome,gait,raw,goal,actual,peer) in expected)
            {
                var log=Finish(groups[origin]); var e=Records(log,Reentry).Single();
                Check.That(Val(e,"first-outcome-resolved")==outcome && Val(e,"firstStrong-firstOffStance-depthLatch")==gait,"native owner and first strong "+origin);
                Check.That(Val(e,"firstBelowRaw")==raw && Val(e,"firstBelowGoal")==goal && Val(e,"firstBelowActual")==actual,"separate observed crossing clocks "+origin);
                Check.That(Val(e,"observedSteps-peerFormal-peerPre-peerPreviousHit-missingRef")==peer &&
                    Val(e,"cohort")=="StanceThroughFirst" && Val(e,"middleRowsOmitted")=="0","paired exposure and complete pre-first rows");
                var release=Records(log,"R183").Single(r=>Val(r,"at")=="Release");
                Check.That(Val(release,"raw-goal-actualMm").StartsWith("NA|NA|"),"older run has no release pose receipts");
            }
        }
        private static float MinCorner(Vector3 ankle, Quaternion r, Vector3 center, Vector3 half, float plane)
        {
            float min=float.PositiveInfinity;
            for(int mask=0;mask<8;mask++)
            {
                var corner=center+new Vector3((mask&1)!=0?half.x:-half.x,(mask&2)!=0?half.y:-half.y,(mask&4)!=0?half.z:-half.z);
                min=Math.Min(min,(ankle+r*corner).y-plane);
            }
            return min;
        }
        private static void ReentryBoxes()
        {
            var center=new Vector3(.016f,-.037f,.058f); var half=new Vector3(.043f,.019f,.114f);
            var target=new Vector3(1,.22f,-2); var lower=target+new Vector3(.003f,-.012f,.2f);
            const float plane=.13f;
            foreach(float angle in new[]{0f,5f,28f,85f,103f,165f,180f})
            {
                var r=Quaternion.AngleAxis(angle,new Vector3(2,1,-1)); var g=Quaternion.AngleAxis(25-angle,Vector3.right);
                Check.That(Trace.TryRightMinima(target,r,lower,g,center,half,plane,out float raw,out float goal),"valid tilted shoe");
                Check.Near(raw,MinCorner(target,r,center,half,plane),.000001f,"source eight vertices");
                Check.Near(goal,MinCorner(lower,g,center,half,plane),.000001f,"control eight vertices");
            }
            var trace = new Trace();
            var a=Receipt(1,1,1); a.Target=target; a.Lower=lower; a.Foot=lower+Vector3.up*.002f;
            a.RightReference=Trace.Reference.SelectedPlane;
            a.RightPose=new Trace.SelectedPose { Source=Quaternion.identity, Control=Quaternion.AngleAxis(12f,Vector3.right),
                Center=center, Half=half, PlaneTop=plane };
            a.BoxMin=MinCorner(a.Foot,Quaternion.AngleAxis(18f,Vector3.right),center,half,plane);
            SendPair(trace,a); SendPair(trace,Released(2));
            var row=Records(Finish(trace),"R183").Single(l=>Val(l,"at")=="Before");
            var v=Val(row,"raw-goal-actualMm").Split('|').Select(Num).ToArray();
            var sc=Val(row,"sourceToGoalAnkleY-rotationMm").Split('|').Select(Num).ToArray();
            var ca=Val(row,"goalToActualAnkleY-rotationMm").Split('|').Select(Num).ToArray();
            Check.That(Val(row,"ref")=="SelectedPlane","former formal step kept its held source and control pose");
            Check.Near(v[0],MinCorner(target,a.RightPose.Source,center,half,plane)*1000f,.0001f,"release baseline raw");
            Check.Near(v[1],MinCorner(lower,a.RightPose.Control,center,half,plane)*1000f,.0001f,"release baseline control");
            Check.Near(sc[0],-12f,.0001f,"lower vs source signed ankle height");
            Check.Near(ca[0],2f,.0001f,"rigid foot vs lower signed ankle height");
            Check.Near(sc[0]+sc[1],v[1]-v[0],.0001f,"source to control equals translation plus rotation");
            Check.Near(ca[0]+ca[1],v[2]-v[1],.0001f,"control to measured equals translation plus rotation");
            Check.That(!Trace.TryRightMinima(target,default,lower,Quaternion.identity,center,half,plane,out var x,out var y) &&
                float.IsNaN(x) && float.IsNaN(y),"invalid orientation cannot manufacture zero clearance");
        }
        private static void ReleaseEligibility()
        {
            foreach(var reason in new[]{StrongExit,"RELEASE_ON_IDENTITY_LOSS","RELEASE_ON_TRANSITION_TO_SWING"})
            foreach(var priorFormal in new[]{true,false})
            {
                var t=new Trace(); var before=Receipt(1,1,1); before.Active=priorFormal;
                SendPair(t,before); var lost=Released(2); lost.Was=priorFormal; lost.ReleaseReason=reason; SendPair(t,lost);
                Check.That(Records(Finish(t),Reentry).Length==(priorFormal && reason==StrongExit?1:0),"genuine release cohort");
            }
            var trace=new Trace(); var pre=Receipt(1,1,1); pre.SourceMin=-.002f; pre.ControlMin=-.003f;
            SendPair(trace,pre); var release=Released(2); release.RightReference=Trace.Reference.Unavailable; SendPair(trace,release);
            var s=Inactive(3); s.SourceMin=-.003f; s.ControlMin=-.004f; s.BoxMin=s.RightEntryMin=-.002f; s.Phase=Phase.Lift; SendPair(trace,s);
            s=Inactive(4); s.Active=s.Ordinary=true; s.BoxMin=s.RightEntryMin=-.001f; SendPair(trace,s);
            var e=Records(Finish(trace),Reentry).Single();
            Check.That(Val(e,"firstBelowRaw")=="3|BelowAfterGap" && Val(e,"firstBelowGoal")=="1|AlreadyBelowAtFirstRead" &&
                Val(e,"firstBelowActual")=="0|NoBelowObserved","bounds, missing reference and baseline are distinct");
            Check.That(Val(e,"cohort")=="LeftStanceBeforeFirst" && Val(e,"firstStrong-firstOffStance-depthLatch")=="0|3|0" && Val(e,"observedThrough")=="FirstOffStance","departure not pooled");
        }
        private static void ReentryClocks()
        {
            var trace=new Trace(); SendPair(trace,Receipt(1,1,1)); var r=Released(2); var l=Receipt(2,2,2,false);
            trace.Submit(1,r); trace.Submit(1,r); trace.Submit(9,r); trace.Submit(0,l);
            trace.NoteContact(0,l.Step,l.Local,l.Floor,false);
            var next=Inactive(3); next.Active=next.Ordinary=true;
            var nextLeft=Receipt(3,3,3,false);
            trace.Submit(0,nextLeft); trace.Submit(0,l); trace.Submit(1,Receipt(1,1,1)); trace.Submit(1,next);
            var log=Finish(trace);
            Check.That(log.Contains("/duplicate-stale-invalidSide=2|1|1/") &&
                Val(Records(log,Reentry).Single(),"observedSteps-peerFormal-peerPre-peerPreviousHit-missingRef")=="2|2|0|1|0",
                "duplicate and stale halves preserve the pending counterpart and exactly one preceding native callback");
            var render = new FootTransitionSource();
            var pose = FootTransitionSource.RenderPose.Capture(Vector3.zero,Quaternion.identity,Vector3.zero,Quaternion.identity,Vector3.zero,Vector3.zero);
            render.Observe(Clock(1),12,Vector3.zero,Quaternion.identity,Vector3.zero,pose);
            render.Observe(Clock(2),12,Vector3.zero,Quaternion.identity,Vector3.zero,pose);
            var captured = render.Current;
            Check.That(render.TryRender(captured,2,out _),"live source packet");
            render.Clear();
            Check.That(!render.TryRender(captured,2,out _),"pre-fault source history invalidated immediately");
            foreach(int mode in new[]{0,1,2,3})
            {
                var t=new Trace(); SendPair(t,Receipt(1,1,1)); SendPair(t,Released(2)); var b=Inactive(3);
                if(mode==0) { b.Step.SourceSequence=2; b.Source.Clock.Sequence=2; }
                if(mode==1) b.Source.Clock.Time=0f;
                if(mode==2) b.Step.UnscaledTime=0f;
                b.Active=b.Ordinary=true;
                if(mode==3)
                {
                    var a=Receipt(3,3,3,false); t.Submit(0,a); b.Step.Dt*=2f; t.Submit(1,b);
                    t.Submit(0,a); t.Submit(1,b);
                }
                else SendPair(t,b);
                var result=Records(Finish(t),Reentry).Single();
                Check.That(Val(result,"cohort")=="Censored" && Val(result,"first-outcome-resolved")=="0|NoNewEntry|0" &&
                    Val(result,"end")==(mode==3?"ClockConflict":"Discontinuity"),"time or source mismatch cannot heal a landing "+mode);
            }
        }
        private static void ReentryCoverage()
        {
            var t=new Trace(); SendPair(t,Receipt(1,1,1)); SendPair(t,Released(2));
            for(int n=3;n<=42;n++)
            {
                var s=Inactive(n);
                if(n==40) { s.Miss=s.PendingMiss=true; s.NativeDepthLatch=36; }
                if(n==41) s.PendingMiss=true;
                if(n==42) s.Active=s.Recovery=true;
                if(n==36) s.BoxMin=s.RightEntryMin=-.004f;
                SendPair(t,s);
            }
            var log=Finish(t); var e=Records(log,Reentry).Single();
            Check.That(Val(e,"firstBelowActual")=="36|Crossed" && Val(e,"middleRowsOmitted")=="6" &&
                Val(e,"first-outcome-resolved")=="40|Recovered|42","first decision, strict crossing and resolution survive row limit");
            var counted = new Trace();
            SendPair(counted,Receipt(1,1,1));
            for(int i=0;i<Trace.MaxPerSideKind+3;i++)
            {
                int n=i*2+2; SendPair(counted,Released(n)); var ordinary=Inactive(n+1); ordinary.Ordinary=ordinary.Active=true; SendPair(counted,ordinary);
            }
            Check.That(Finish(counted).Contains("/eligible-recorded-omitted=51|48|3/"),"omitted eligible releases stay in the denominator");
            var rows=Records(log,"R183");
            Check.That(rows.Any(l=>Val(l,"at")=="Resolution") && rows.Any(l=>Val(l,"at")=="LastThroughFirst") && rows.Length==35,"bounded details and separate resolution");
        }
    }
}
