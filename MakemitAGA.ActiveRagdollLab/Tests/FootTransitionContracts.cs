using System;
using System.Globalization;
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
        private const float Dt = .02f;
        private static readonly Vector3 Axis = Vector3.forward;
        private static SourceStepContext Clock(int i) => new SourceStepContext {Epoch=3, Sequence=i, Time=i*Dt, Dt=Dt};
        private static void Near(Vector3 a, Vector3 b, string msg) => Check.Near((a-b).magnitude,0f,.0001f,msg);
        internal static void Run()
        {
            RunRightReentryContracts();
            Check.Run("168 frozen gait bounds, simultaneous causes and 167 confirmation samples", Gait);
            Check.Run("168 adjacent Render rigid transport, rotation and rejected derivatives", SourceMotion);
            Check.Run("168 material-point velocity and all eight box vertices", SoleBox);
            Check.Run("168 exact same-step pair and released-side comparison in either callback order", Releases);
            Check.Run("168 stationary target, proxy return, signed travel and support on either side", Stops);
            Check.Run("168 catching up, lateral motion, rotating shoes and return caused by the target", Alternatives);
            Check.Run("168 stale clocks, unmatched feet, support identity and missing opportunities", Cuts);
            Check.Run("168 quiet held samples, skipped Render edges and gentle torso stop", SampleAndHold);
            Check.Run("168 visible overshoot between Fixed steps and independent held samples", RenderOvershoot);
            Check.Run("168 native ENTER/STAY attaches only to the preceding matching support interval", Callbacks);
            Check.Run("168 per-side capture limits, run isolation and sealed partial pairs", Limits);
        }
        private static void Gait()
        {
            foreach(int hits in new[]{0,1,4,5}) foreach(bool same in new[]{true,false})
            foreach(float h in new[]{0f,.02999f,.030f,.03001f,.05499f,.055f,.05501f})
            foreach(float vn in new[]{-.3f,-.22f,0f,.21999f,.22f,.22001f,.29999f,.30f,.30001f})
            foreach(float vt in new[]{0f,.34999f,.35f,.35001f,.64999f,.65f,.65001f,2f})
            {
                var g = new FootGaitEvidence(hits,same,41,h,Vector3.up,vn,vt);
                Check.That(g.Stance==(hits==5 && same && h<=.03f && Math.Abs(vn)<=.22f && vt<=.35f),"stance");
                int flags=(hits==0 ? 1 : 0) | (h>=.055f ? 2 : 0) | (Math.Abs(vn)>=.30f ? 4 : 0) | (vt>=.65f ? 8 : 0);
                Check.That((int)g.Swing==flags && (!g.Stance || flags==0),"swing causes and disjoint bounds");
            }
            // Actual 167 confirmation steps. Release-step inputs were absent from the old log.
            var left=new FootGaitEvidence(5,true,-12046,.004157856f,Vector3.up,.037832f,2.008467f);
            var right=new FootGaitEvidence(5,true,-12046,.005384669f,Vector3.up,.343818f,1.645422f);
            Check.That((int)left.Swing==8 && (int)right.Swing==12,"2510 left, 5210 right");
            var unchanged=new FootGaitEvidence(0,false,0,.06f,Vector3.up,0,2f).WithVelocity(Vector3.zero);
            Check.That((int)unchanged.Swing==3 && !unchanged.Stance,"remove transport, keep independent causes");
        }
        private static void SourceMotion()
        {
            var s=new FootTransitionSource(); var p=new Vector3(1,2,3); var foot=p+Axis;
            s.Observe(Clock(1),61,p,Quaternion.identity,foot);
            var turn=Quaternion.AngleAxis(18,Vector3.up);
            var d=new Vector3(.01f,0,.02f); var dr=turn*Axis-Axis; var local=Vector3.right*.004f;
            var final=foot+d+dr+local;
            s.Observe(Clock(2),61,p+d,turn,final);
            var m=s.Current; Check.That(m.Valid,"two clocks");
            Near(m.Translation,d/Dt,"root travel"); Near(m.Turn,dr/Dt,"rotated old lever"); Near(m.Relative,local/Dt,"foot relative");
            Near(m.World,m.Translation+m.Turn+m.Relative,"conservation");
            s.Observe(Clock(2),61,p,Quaternion.identity,Vector3.zero);
            Near(s.Current.Position,final,"duplicate preserved");
            s.Observe(Clock(3),62,p,turn,final); Check.That(!s.Current.Valid,"rebound Transform");
            s.Observe(Clock(4),62,p,turn,final); Check.That(s.Current.Valid && s.Current.Continuity!=m.Continuity,"new derivative");
            var bad=Clock(5); bad.Time=Clock(4).Time;
            s.Observe(bad,62,p,turn,final); Check.That(!s.Current.Valid,"zero elapsed");
            bad=Clock(6); bad.Time=.5f;
            s.Observe(bad,62,p,turn,final); Check.That(!s.Current.Valid,"long Render gap");
            bad=Clock(7);bad.Time=.75f;
            s.Observe(bad,62,p,turn,final);Check.That(!s.Current.Valid,"exact binary quarter-second boundary");
            s.Observe(Clock(8),62,p,default,final); Check.That(!s.Current.Valid && s.Current.Root==0,"bad pose");
        }
        private static void SoleBox()
        {
            var center=new Vector3(.01f,-.07f,.055f); var half=new Vector3(.042f,.022f,.11f);
            foreach(float angle in new[]{0,3,45,89,105,180,-19})
            {
                var r=Quaternion.AngleAxis(angle,new Vector3(1,3,2));
                var lever=FootTransitionSource.LowestLever(r,center,half);
                float min=float.PositiveInfinity;
                for(int i=0;i<8;i++)
                {
                    var corner=center+new Vector3((i&1)==0?half.x:-half.x,(i&2)==0?half.y:-half.y,(i&4)==0?half.z:-half.z);
                    min=Math.Min(min,(r*corner).y);
                }
                Check.Near(lever.y,min,.000001f,"eight-corner minimum");
            }
            var flat=FootTransitionSource.LowestLever(Quaternion.identity,center,half);
            Near(flat,center-Vector3.up*half.y,"flat face center");
            var axisW=Vector3.right*4f;
            Check.That(FootTransitionSource.Planar(Vector3.Cross(axisW,flat)).magnitude>.3f,"rotation moves sole with stationary ankle");
        }

        internal sealed class Sim
        {
            internal readonly Trace Trace = new Trace();
            internal int Solve, Render;
            internal bool Reverse;
            internal delegate void Edit(ref Trace.Tick left,ref Trace.Tick right);
            internal void RenderOnly(float root,float left,float right,float leftLead=0,float rightLead=0,bool visible=true)
            {
                Render++;
                for(int side=0;side<2;side++)
                {
                    Vector3 target=Vector3.right*(side==0 ? -.1f:.1f)+Axis*(side==0 ? left:right);
                    Vector3 proxy=target+Axis*(side==0 ? leftLead:rightLead);
                    var visual=FootTransitionSource.RenderPose.Capture(target,Quaternion.identity,proxy,Quaternion.identity,
                        Vector3.zero,Vector3.forward*.05f-Vector3.up*.07f);
                    Trace.Source(side).Observe(Clock(Render),42,Axis*root,Quaternion.identity,target,visible ? visual : default);
                }
            }
            internal void Frame(float root,float left,float right,Edit edit=null,bool held=false)
            {
                Solve++;
                if(!held) RenderOnly(root,left,right);
                var a=Make(0,held); var b=Make(1,held);
                if(edit!=null) edit(ref a,ref b);
                if(Reverse) { Trace.Submit(1,b); Trace.Submit(0,a); }
                else { Trace.Submit(0,a); Trace.Submit(1,b); }
            }
            private Trace.Tick Make(int side,bool held)
            {
                var c=Trace.Source(side).Current;
                var t=new Trace.Tick {Step=new PhysicsStepContext {Epoch=3,Sequence=Solve,SourceSequence=Render,FixedTime=Solve*Dt,
                    UnscaledTime=Solve*Dt,Dt=Dt}, Source=c, Local=Solve, Body=71+side, Floor=6, PriorFloor=6, EventFloor=6,
                    Fresh=!held,Held=held?1:0,Age=held?Dt:0, Was=true,Active=true,Inside=true,
                    Target=c.Position,Foot=c.Position,Lower=c.Position,Sole=c.Position-Vector3.up*.07f,SourceSole=c.Position-Vector3.up*.07f,
                    Phase=Phase.Stance,Motion="STEADY",HipValid=true,HipHeight=.002f,BoxMin=.001f,Velocity=c.World,CornerVelocity=Vector3.zero};
                t.Gait=new FootGaitEvidence(5,true,6,.005f,Vector3.up,0f,FootTransitionSource.Planar(c.World).magnitude);
                return t;
            }
            internal string Report() {Trace.Seal(); return Trace.Summary();}
        }

        private static string[] Records(string log,string prefix) => log.Split('\n').Where(l=>l.StartsWith(prefix+"/",StringComparison.Ordinal)).ToArray();
        private static string Val(string row,string key) => row.Split('/').Single(f=>f.StartsWith(key+"=",StringComparison.Ordinal)).Substring(key.Length+1);
        private static float Num(string v) => float.Parse(v,CultureInfo.InvariantCulture);
        private static string Event(string log,string side,string kind) => Records(log,"FOOT_TRANSITION_EVENT_V041680")
            .Single(e=>Val(e,"side")==side && Val(e,"kind")==kind);
        private static void Release(ref Trace.Tick t)
        { t.Was=true; t.Active=false; t.Phase=Phase.Lift; t.ReleaseReason="RELEASE_ON_GAIT_EXIT"; t.EventFloor=0; }
        private static void Free(ref Trace.Tick t)
        { t.Was=false;t.Active=false;t.EventFloor=0;t.Phase=Phase.Swing; }
        private static void Releases()
        {
            foreach(bool reverse in new[]{true,false})
            {
                var s=new Sim{Reverse=reverse};
                s.Frame(0,0,0); s.Frame(.02f,.02f,.02f);
                s.Frame(.04f,.04f,.04f,(ref Trace.Tick a,ref Trace.Tick b)=>{Release(ref a);Release(ref b);});
                s.Frame(.06f,.06f,.06f,(ref Trace.Tick a,ref Trace.Tick b)=>{Free(ref a);Free(ref b);b.Precontact=true;b.EventFloor=6;});
                s.Frame(.08f,.08f,.08f,(ref Trace.Tick a,ref Trace.Tick b)=>{
                    Free(ref a);a.Miss=a.PendingMiss=true;a.EventFloor=6;a.HipHeight=-.003f;b.Was=false;b.Ordinary=true;});
                s.Frame(.10f,.1f,.1f,(ref Trace.Tick a,ref Trace.Tick b)=>{Free(ref a);a.PendingMiss=true;});
                s.Frame(.12f,.12f,.12f,(ref Trace.Tick a,ref Trace.Tick b)=>{a.Was=false;a.Recovery=true;});
                s.Frame(.14f,.14f,.14f,(ref Trace.Tick a,ref Trace.Tick b)=>Release(ref b));
                s.Frame(.16f,.16f,.16f,(ref Trace.Tick a,ref Trace.Tick b)=>{Free(ref b);b.Miss=b.PendingMiss=true;b.EventFloor=6;});
                s.Frame(.18f,.18f,.18f,(ref Trace.Tick a,ref Trace.Tick b)=>Free(ref b));
                string log=s.Report(); string left=Event(log,"LEFT","PairRelease"), right=Event(log,"RIGHT","PairRelease");
                Check.That(Val(left,"first-outcome-resolved")=="5|Recovered|7" && Val(right,"first-outcome-resolved")=="5|Ordinary|5","true matching opportunities");
                Check.That(Val(left,"otherFormal-otherPreThroughFirst")=="1|1","opposite formal and pre each counted before first");
                Check.That(Val(Event(log,"RIGHT","SingleRelease"),"first-outcome-resolved")=="9|HoldEnded|10","single-foot control");
                Check.That(Records(log,"FOOT_TRANSITION_COUNTS_V041680").All(row=>Val(row,"pairedLift-worldTanOnly-relNoSwing-relStance-nearQuiet")=="1|1|1|1|1"),"same-state transport and quietness");
                Check.That(log.Contains("at=hipCrossBelow/local-shared-source=5|5|5"),"adjacent observed hip crossing");
            }
        }

        private static string StopRun(int side,float direction,float[] target,float[] ankle,float[] sole,float x=0f)
        {
            var s=new Sim{Reverse=side==1};
            for(int j=0;j<target.Length;j++)
            {
                int n=j; float p=direction*target[n];
                s.Frame(p,side==0?p:0f,side==1?p:0f,(ref Trace.Tick l,ref Trace.Tick r)=>{
                    ref var t=ref (side==0 ? ref l:ref r);
                    Vector3 lateral=n>=3 ? Vector3.right*x:Vector3.zero;
                    t.Foot+=Axis*(direction*ankle[n])+lateral; t.Sole+=Axis*(direction*sole[n])+lateral;
                    if(n>=3) { t.Was=n==3;t.Active=false;t.EventFloor=0;t.Phase=Phase.Approach; }
                    l.Motion=r.Motion=n==3?"DECEL":n>=4?"STOP":"STEADY";
                });
            }
            return s.Report();
        }
        private static readonly float[] Stopped={0,.02f,.04f,.04f,.04f,.04f,.04f};
        private static void Stops()
        {
            foreach(int side in new[]{0,1}) foreach(float direction in new[]{-1f,1f})
            {
                var offset=new[]{0f,0,0,.004f,.013f,.010f,.0005f};
                string log=StopRun(side,direction,Stopped,offset,offset); var e=Event(log,side==0?"LEFT":"RIGHT","Brake");
                var a=Val(e,"ankle-base-quiet-peakMm-peakAt-returnAt-proxyDelta-targetDeltaMm").Split('|');
                Check.Near(Num(a[2]),13f,.001f,"true signed overshoot in either travel direction");
                Check.That(a[3]=="5" && a[4]=="7" && Val(e,"quiet")=="4","stop, peak, return steps");
                Check.Near(Num(a[5]),-12.5f,.001f,"actual proxy moved back");Check.Near(Num(a[6]),0,.001f,"animation stationary");
                Check.That(Val(e,"ankleLead-closure-soleLead-closure")=="NewForwardLead|ProxyBack|NewForwardLead|ProxyBack","forward and back");
                Check.That(Val(e,"first-outcome-resolved")=="0|NoNewEntry|0" && Val(e,"steps-otherFormal-otherPre")=="4|4|0","stop does not imply miss");
                Check.That(Val(e,"axis-kind-xyz").StartsWith("RootTravel|"),"world travel axis");
                var track=Val(e,"nearIntervals-pathMm-netAlongMm-formalIntervals-formalPathMm").Split('|');
                Check.That(Num(track[1])>20 && track[3]=="1","fixed material displacement and preceding formal authority");
            }
        }
        private static void Alternatives()
        {
            var lag=new[]{0f,-.010f,-.012f,-.009f,-.004f,-.001f,0f};
            var zero=new float[7];
            var e=Event(StopRun(0,1,Stopped,lag,zero,.035f),"LEFT","Brake");
            Check.That(Val(e,"ankleLead-closure-soleLead-closure")=="NoForwardLead|Unobserved|NoForwardLead|Unobserved","catch-up from behind, even with lateral displacement");
            var rotation=new[]{0f,0,0,.004f,.012f,.011f,0};
            e=Event(StopRun(1,1,Stopped,zero,rotation),"RIGHT","Brake");
            Check.That(Val(e,"ankleLead-closure-soleLead-closure")=="NoForwardLead|Unobserved|NewForwardLead|ProxyBack","shoe can lead with ankle aligned");
            var slow=new[]{0f,.02f,.04f,.04f,.042f,.044f,.046f};
            var stationaryProxy=new[]{0f,0,0,.006f,.004f,.002f,0};
            e=Event(StopRun(1,1,slow,stationaryProxy,stationaryProxy),"RIGHT","Brake");
            Check.That(Val(e,"ankleLead-closure-soleLead-closure").Contains("NewForwardLead|TargetCaughtUp"),"stationary proxy, target closes separation");
            var ahead=new[]{0f,.012f,.012f,.012f,.012f,.005f,0};
            e=Event(StopRun(1,1,Stopped,ahead,ahead),"RIGHT","Brake");
            Check.That(Val(e,"ankleLead-closure-soleLead-closure").StartsWith("AlreadyAhead|ProxyBack"),"lead established before brake");
        }
        private static void Cuts()
        {
            foreach(int reason in Enumerable.Range(0,10))
            {
                var s=new Sim();s.Frame(0,0,0);s.Frame(.02f,.02f,.02f);
                s.Frame(.04f,.04f,.04f,(ref Trace.Tick a,ref Trace.Tick b)=>{Release(ref a);Release(ref b);});
                s.Frame(.06f,.06f,.06f,(ref Trace.Tick a,ref Trace.Tick b)=>{
                    Free(ref a);a.Miss=a.PendingMiss=true;a.EventFloor=6;Free(ref b);
                    switch(reason)
                    {
                        case 0: b.Age=.1001f;break; case 1:b.Floor=10;break;case 2:b.Source.Root=18;break;
                        case 3:b.Body=a.Body;break;case 4:b.Local++;break;case 5:a.EventFloor=23;break;
                        case 6:b.Step.SourceSequence++;break;case 7:b.Source.Clock.Dt=.016f;break;
                        case 8:b.Source.Continuity++;break;case 9:a.Phase=Phase.Unknown;break;
                    }
                });
                var e=Event(s.Report(),"LEFT","PairRelease");
                Check.That(Val(e,"first-outcome-resolved")=="0|NoNewEntry|0","bad evidence never turns into a landing #"+reason);
                Check.That(!Val(e,"end-local").StartsWith("LandingResolved"),"cut unresolved");
            }
        }
        private static void SampleAndHold()
        {
            var s=new Sim();s.Frame(0,0,0);s.Frame(.02f,.02f,0);s.Frame(.04f,.04f,0);
            s.Frame(.04f,.04f,0);s.Frame(.04f,.04f,0,held:true);s.Frame(.04f,.04f,0,held:true);
            var e=Event(s.Report(),"LEFT","Brake");Check.That(Val(e,"maxQuietSources")=="1","one Render quiet across three solves");
            var r=new FootTransitionSource();
            r.Observe(Clock(1),2,Vector3.zero,Quaternion.identity,Vector3.zero);
            r.Observe(Clock(2),2,Vector3.zero,Quaternion.identity,Axis*.03f);
            r.Observe(Clock(3),2,Vector3.zero,Quaternion.identity,Axis*.03f);
            Check.That(r.BrakeSequence==3 && !r.Travel.FromRoot,"independent foot travel if root is stationary");
            r.Observe(Clock(4),2,Vector3.zero,Quaternion.identity,Axis*.06f);
            Check.That(r.BrakeSequence==0 && r.UnseenBrakes==1,"brake and restart between solves");
            r.Observe(Clock(5),2,Vector3.zero,Quaternion.identity,Axis*.06f);
            r.Observe(Clock(8),2,Vector3.zero,Quaternion.identity,Axis*.06f);
            Check.That(!r.Current.Valid && r.BrakeSequence==0,"source sequence break");
            var gentle=new Sim();gentle.Frame(0,0,0);gentle.Frame(.004f,0,.004f);gentle.Frame(.008f,0,.008f);
            gentle.Frame(.008f,0,.008f,(ref Trace.Tick l,ref Trace.Tick t)=>l.Motion=t.Motion="STOP");
            e=Event(gentle.Report(),"RIGHT","Brake");
            Check.That(Val(e,"trigger-source-footBrakeSource-torsoEdge")=="STOP|4|0|STOP" && Val(e,"quiet")=="4","slow torso stop with no swing threshold crossed");
        }
        private static void RenderOvershoot()
        {
            foreach(bool right in new[]{false,true})
            {
                var s=new Sim{Reverse=right};
                s.Frame(0,0,0); s.Frame(.02f,right?0:.02f,right?.02f:0); s.Frame(.04f,right?0:.04f,right?.04f:0);
                float l=right?0f:.04f, r=right?.04f:0f;
                s.RenderOnly(.04f,l,r,right?0:.004f,right?.004f:0);
                s.RenderOnly(.04f,l,r,right?0:.019f,right?.019f:0);
                s.Frame(.04f,l,r);s.Frame(.04f,l,r,held:true);
                var e=Event(s.Report(),right?"RIGHT":"LEFT","Brake");
                Check.That(Val(e,"renderSamples-quietSource-end")=="3|4|WindowEnd","two intermediate renders, no duplicate for held sample");
                Check.That(Val(e,"ankleLead-closure-soleLead-closure").StartsWith("NoForwardLead"),"proxy aligned at both sampled physics boundaries");
                Check.That(Val(e,"renderAnkleLead-closure-soleLead-closure").StartsWith("NewForwardLead|ProxyBack"),"visible pre-physics overshoot detected");
                var peak=Val(e,"renderAnkle-base-quiet-peakMm-sourceAt-returnSource-proxyDelta-targetDeltaMm").Split('|');
                Check.Near(Num(peak[2]),19f,.001f,"between-Fixed signed peak");
                Check.That(peak[3]=="5" && peak[4]=="6","render clock peak and return");
            }
            var ring=new Sim();
            for(int i=0;i<FootTransitionSource.RenderCapacity+5;i++) ring.RenderOnly(0,0,0);
            var history=ring.Trace.Source(0);
            Check.That(!history.TryRender(history.Current,2,out _) && history.TryRender(history.Current,ring.Render,out _),"overwritten slots never alias");
            var q=Quaternion.AngleAxis(65,Vector3.right);
            var offset=Vector3.forward*.15f;
            var v=FootTransitionSource.RenderPose.Capture(Vector3.zero,Quaternion.identity,Vector3.zero,q,Vector3.zero,offset);
            Check.That(v.Valid && v.Foot==v.Target && (v.Sole-v.TargetSole).magnitude>.05f,"rotation-only difference from paired visible transforms");
        }
        private static void Callbacks()
        {
            var s=new Sim();
            s.Frame(0,0,0);s.Frame(.02f,.02f,.02f);
            Trace.Tick previous=default;
            s.Frame(.04f,.04f,.04f,(ref Trace.Tick a,ref Trace.Tick b)=>{previous=a;});
            s.Trace.NoteContact(0,previous.Step,previous.Local,6,false);
            s.Trace.NoteContact(0,previous.Step,previous.Local,6,false);
            s.Trace.NoteContact(1,previous.Step,previous.Local,7,false);
            s.Frame(.04f,.04f,.04f,(ref Trace.Tick a,ref Trace.Tick b)=>{
                Release(ref a);a.Foot+=Axis*.008f;a.Sole+=Axis*.008f;
                a.Precontact=true;a.EventFloor=6;previous=a;
            });
            s.Trace.NoteContact(0,previous.Step,previous.Local,6,true);
            s.Trace.NoteContact(0,previous.Step,previous.Local-1,6,true);
            s.Frame(.04f,.04f,.04f,(ref Trace.Tick a,ref Trace.Tick b)=>Free(ref a));
            s.Frame(.04f,.04f,.04f,(ref Trace.Tick a,ref Trace.Tick b)=>Free(ref a));
            var log=s.Report();
            var path=Val(Event(log,"LEFT","Brake"),"callbackFormalIntervals-pathMm-preIntervals-pathMm").Split('|');
            Check.That(path[0]=="1" && path[2]=="1","one interval per exact stamp");
            Check.Near(Num(path[1]),8f,.001f,"formal physical interval");
            Check.Near(Num(path[3]),8f,.001f,"precontact tracked separately");
            Check.That(Val(Records(log,"FOOT_TRANSITION_V041680")[0],"unmatchedCallbacks")=="2","wrong surface and local excluded");
        }
        private static void Limits()
        {
            var s=new Sim();s.Frame(0,0,0);s.Frame(0,0,0);
            for(int i=0;i<Trace.MaxPerSideKind+3;i++)
            {
                s.Frame(0,0,0,(ref Trace.Tick a,ref Trace.Tick b)=>Release(ref a));
                s.Frame(0,0,0,(ref Trace.Tick a,ref Trace.Tick b)=>{a.Was=false;a.Ordinary=true;});
            }
            s.Trace.Submit(0,Receipt(s.Solve+1,s.Solve+1,s.Render,false));var log=s.Report();
            Check.That(Records(log,"FOOT_TRANSITION_EVENT_V041680").Length==Trace.MaxPerSideKind,"bounded rows");
            Check.That(Val(Records(log,"FOOT_TRANSITION_COUNTS_V041680")[0],"seen-skipped(pair,single,brake)")=="0|0,51|3,0|0","dropped counted");
            Check.That(Val(Records(log,"FOOT_TRANSITION_V041680")[0],"pairs-cuts-rejected-missing-faults-sealedPartial").EndsWith("|1"),"partial sealed");
            s.Trace.Seal();s.Trace.Fault();Check.That(s.Trace.Summary()==log,"sealed idempotent");
            var timing=new LandingObservationTiming();timing.Begin(PelvisDynamicReceiverIsolation.DynamicHipArmB);var first=timing.Transitions;
            Check.That(first!=null,"selected B");timing.Seal();Check.That(timing.Transitions==null && timing.Summary().Contains("FOOT_TRANSITION_V041680"),"sealed report");
            timing.Begin(PelvisDynamicReceiverIsolation.DynamicHipArmA);Check.That(timing.Transitions!=null && timing.Transitions!=first && timing.Summary().Contains("FOOT_TRANSITION_V041680"),"A has an independent matching observation window");
            timing.Begin(PelvisDynamicReceiverIsolation.DynamicHipArmB);Check.That(timing.Transitions!=first,"new run owner");
        }
    }
}
