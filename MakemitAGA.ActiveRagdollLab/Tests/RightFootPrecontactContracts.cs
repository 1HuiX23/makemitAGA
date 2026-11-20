using System;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Window = MakemitAGA.ActiveRagdollLab.Physics.RightFootPrecontact;

namespace ARL.Contracts
{
    internal static partial class RightFootPrecontactContracts
    {
        internal static void Run()
        {
            Check.Run("Precontact selects formal right A/B only", Selectors);
            RunExpiryContracts();
            RunRisingContracts();
            RunPreviousStepExpiryContracts();
            Check.Run("Precontact requires two downward physical samples and current five-point floor", Candidate);
            Check.Run("Precontact checks both clock domains and support continuity", Continuity);
            Check.Run("Precontact expires by physics time and rearms on a new descent", TimeAndRearm);
            Check.Run("Precontact releases on lift, stale input, geometry loss and the hard gap", Release);
            Check.Run("Precontact separates signed native reaction, net rotation and missing reads", Measurements);
            Check.Run("Right formal entry preserves first reaction in both arms", FormalImpacts);
            Check.Run("Precontact links original stance outcomes by foot and keeps trace resets separate", Links);
        }

        private static Window.Sample S(int n, float h, float dt = .016f) => new Window.Sample
        {
            Local = n, Support = 42, Step = new PhysicsStepContext { Epoch = 3, Sequence = n+20, SourceSequence = n,
                FixedTime = n*dt, UnscaledTime = n*dt, Dt = dt },
            Source = new SourceStepContext { Epoch = 3, Sequence = n, Time = n*dt, Dt = dt },
            ExactFloor = true, PhaseKnown = true, Swing = true, Lift = false, Gait = "SWING",
            FloorCenter = Vector3.zero, FloorHalf = new Vector3(1,1,1), W = new Vector3(0,1,0),
            Min = h, Predicted = h-.002f, CornerV = -.12f, AnkleY = .1f+h, Age = .015f, Gap = .004f
        };
        private static void Selectors()
        {
            var invalid = (Window.Mode)99;
            var window = new Window(); window.Select(invalid);
            Check.That(window.Selected == Window.Mode.Off && Window.ForFoot(invalid, "RIGHT") == Window.Mode.Off, "unregistered selector");
            foreach (string side in new[] {"LEFT", null, "UNKNOWN"})
                foreach (bool b in new[] {true,false}) Check.That(Window.ForFoot(b ? Window.Mode.PreviousStepExpiryReview : Window.Mode.NearGroundHold8mm,side)==Window.Mode.Off,"peer foot");
            Check.That(Window.ForFoot(Window.Mode.FallingForecastRise5mm,"RIGHT")==Window.Mode.Off, "retired physical arm is replay-only");
            Check.That(Window.ForFoot(Window.Mode.Off,"RIGHT")==Window.Mode.Off && Window.ForFoot(Window.Mode.NearGroundHold8mm,"RIGHT")==Window.Mode.NearGroundHold8mm &&
                Window.ForFoot(Window.Mode.PreviousStepExpiryReview,"RIGHT")==Window.Mode.PreviousStepExpiryReview,"formal arms");
        }
        private static void Candidate()
        {
            var a=S(1,.025f); var b=S(2,.001f); b.Gap=.005f; b.Predicted=-.002f;
            Check.That(Window.Candidate(a,b),"inclusive boundaries; descending swing permitted");
            a.Gap=.007f; Check.That(Window.Candidate(a,b),"5 mm is a decision boundary");
            foreach (string issue in new[]{"floor","higher","ankle","velocity","gap","predicted","negative","age","unknown","lift","nan"})
            {
                var x=b;
                switch(issue)
                {
                    case "floor": x.ExactFloor=false;break;
                    case "higher": x.Min=a.Min;break;
                    case "ankle": x.AnkleY=a.AnkleY;break;
                    case "velocity": x.CornerV=0;break;
                    case "gap": x.Gap=.00501f;break;
                    case "predicted": x.Predicted=-.00201f;break;
                    case "negative": x.Min=-.0001f;break;
                    case "age": x.Age=.101f;break;
                    case "unknown": x.PhaseKnown=false;break;
                    case "lift": x.Lift=true;break;
                    default: x.AnkleY=float.NaN;break;
                }
                Check.That(!Window.Candidate(a,x),issue);
            }
            var fastFirst = S(1, .045f);
            Check.That(!Window.Candidate(fastFirst, b), "first approach sample still capped at 25 mm");
            var widerGap = b; widerGap.Gap = .010f;
            Check.That(!Window.Candidate(a, widerGap), "opening still requires a 5 mm ankle gap");
            a.ExactFloor=false; Check.That(!Window.Candidate(a,b),"raw exact floor also required on the first sample");
        }
        private static void Continuity()
        {
            var a=S(5,.013f); var b=S(6,.010f);
            b.Step.SourceSequence=a.Source.Sequence; b.Source=a.Source;
            Check.That(Window.Candidate(a,b),"two Fixed updates may hold one fresh render packet");
            var c=b;c.Source.Time+=.001f;Check.That(!Window.Candidate(a,c),"held source stamp changed");
            c=b;c.Step.Sequence++;Check.That(!Window.Candidate(a,c),"missing shared Fixed");
            c=b;c.Local++;Check.That(!Window.Candidate(a,c),"missing foot Fixed");
            c=b;c.Step.Epoch++;c.Source.Epoch++;Check.That(!Window.Candidate(a,c),"rebind");
            c=b;c.Support++;Check.That(!Window.Candidate(a,c),"wrong selected ID");
            c=b;c.FloorCenter=Vector3.up;Check.That(!Window.Candidate(a,c),"same ID moved plane");
            c=b;c.Step.SourceSequence++;Check.That(!Window.Candidate(a,c),"foreign source");
        }
        private static Window Open(Window.Mode mode, float dt=.016f, Action<string> log=null)
        {
            var w=new Window(log); w.Select(mode);
            var a=S(1,.016f,dt);var b=S(2,.012f,dt);
            w.Evaluate(a,false,out bool first);w.Evaluate(b,false,out bool second);
            Check.That(!first && second,"opens following the second observation");
            w.Open();w.CountSolve();
            return w;
        }
        private static void TimeAndRearm()
        {
            foreach(var mode in new[]{Window.Mode.NearGroundHold8mm,Window.Mode.FallingForecastRise5mm})
            {
                var w=Open(mode,.05f);
                for(int i=3;i<=4;i++)
                {
                    var s=S(i,.011f,.05f);s.Gap=.007f;
                    Check.That(w.Evaluate(s,false,out bool open)==Window.End.None && !open,"existing window can settle above 5 mm");
                    w.CountSolve();
                }
                Check.That(w.Evaluate(S(5,.010f,.05f),false,out _)==Window.End.Timeout,"only three 50 ms solves");
                w.Close(Window.End.Timeout,5);
                w.Evaluate(S(6,.009f,.05f),false,out bool again);Check.That(!again,"same descent locked");
                w.Evaluate(S(7,.030f,.05f),false,out _);w.Evaluate(S(8,.020f,.05f),false,out _);
                w.Evaluate(S(9,.014f,.05f),false,out bool next);Check.That(next,"new descent with two observations");
            }
        }
        private static void Release()
        {
            foreach(var reason in new[]{Window.End.Floor,Window.End.Lift,Window.End.Rising,Window.End.Gap,Window.End.Depth,Window.End.Stale,Window.End.Clock,Window.End.Miss})
            {
                var w=Open(Window.Mode.FallingForecastRise5mm); var c=S(3,.011f);
                switch(reason)
                {
                    case Window.End.Floor:c.ExactFloor=false;break;
                    case Window.End.Lift:c.Lift=true;break;
                    case Window.End.Rising:c.Min=.015f;c.AnkleY=.13f;c.CornerV=-.01f;break;
                    case Window.End.Gap:c.Gap=.030f;break;
                    case Window.End.Depth:c.Min=-.0031f;break;
                    case Window.End.Stale:c.Age=float.NaN;break;
                    case Window.End.Clock:c.Local+=1;break;
                }
                Check.That(w.Evaluate(c,reason==Window.End.Miss,out _)==reason,reason.ToString());
            }
            var next=Open(Window.Mode.FallingForecastRise5mm);var lift=S(3,.011f);lift.Lift=true;lift.Swing=false;
            var exit=next.Evaluate(lift,false,out _);next.Close(exit,3);
            next.Evaluate(S(4,.015f),false,out bool one);next.Evaluate(S(5,.010f),false,out bool two);
            Check.That(!one && two,"a single lift sample permits a new two-step descent");
        }
        private static void Measurements()
        {
            var w=Open(Window.Mode.FallingForecastRise5mm);Check.That(w.Physical && w.CheckUnloaded(2,false),"checked pre-solve");
            w.Callback(1,42,4,2);w.Callback(2,41,4,2);w.Callback(2,42,0,1.25f);w.Callback(2,42,-.5f,2.25f);
            w.Callback(2,42,float.NaN,float.NaN);
            w.Evaluate(S(3,.004f),false,out _);w.CountSolve();Check.That(!w.CheckUnloaded(3,true),"unexpected load fails closed");
            var rising=S(4,.006f);rising.CornerV=.15f; w.Evaluate(rising,false,out _);
            w.Close(Window.End.Rising,4);
            string log=w.Summary();
            Check.That(log.Contains("firstCallbackLocal-signedNormalNs-deltaWRadSec=2|0|1.25") && log.Contains("peakAbsNormalNs-deltaWRadSec=0.5|2.25") &&
                log.Contains("firstNonzeroLocal-signedNormalNs-deltaWRadSec=2|-0.5|2.25") && log.Contains("callbacks-missingImpulse-missingW=3|1|1") && log.Contains("low-riseFromLow-peakGapMm=4|2|4"),"signed zero, net kick and rebound");
            var a=Open(Window.Mode.FallingForecastRise5mm);a.Callback(2,42,2,1);
            Check.That(a.Physical && a.Summary().Contains("firstNonzeroLocal-signedNormalNs-deltaWRadSec=2|2|1"),"control arm also measures native impulse");
        }
        private static void FormalImpacts()
        {
            foreach (var mode in new[]{Window.Mode.NearGroundHold8mm,Window.Mode.FallingForecastRise5mm})
            {
                var w=Open(mode);w.Evaluate(S(3,.002f),false,out _);w.BeginFormal(false);
                w.Close(Window.End.Handoff,3);
                w.FormalCallback(2,42,.8f,.5f);w.FormalCallback(3,41,.8f,.5f);
                w.FormalCallback(3,42,0,.5f);w.FormalCallback(4,42,-.2f,.7f);w.FormalCallback(5,42,.3f,.6f);
                Check.That(w.Summary().Contains("priorWindowLocal=2/firstCallbackLocal-signedNormalNs-deltaWRadSec=3|0|0.5/firstNonzeroLocal-signedNormalNs-deltaWRadSec=4|-0.2|0.7"),"join and compare A/B actual formal callbacks");
                w.EndFormal();Check.That(!w.NeedsFormalImpact(42),"close on release");
            }
        }
        private static void Links()
        {
            var w=Open(Window.Mode.FallingForecastRise5mm);w.ResetTrace();
            Check.That(w.Physical && w.LinkedOrigin==2 && w.Summary().Contains("end-local=TraceEnded|2"),"snapshot censors in-flight episode without releasing it");
            w.Close(Window.End.Handoff,3);Check.That(w.LinkedOrigin==2 && !w.Physical,"retain origin through formal handoff");
            var l=new LandingWindowOutcomes();
            var e=new FootLandingOpportunityAudit.Sample{Step=S(3,.01f).Step,Source=S(3,.01f).Source,Fixed=3,SupportId=42,
                GeometryValid=true,Min=.001f,TrackingValid=true,MotionValid=true,NowActive=true,OrdinaryEnabled=true,
                PrecontactOrigin=w.LinkedOrigin,FootSide="RIGHT"};
            l.Observe(e,false,true,false,true);
            Check.That(l.Summary().Contains("/side=RIGHT/rightPrecontactOriginLocal=2"),"ordinary event keeps its origin");
            w.AfterFormalEvent();Check.That(w.LinkedOrigin==0,"never reuse a link across formal landings");
        }
    }
}
