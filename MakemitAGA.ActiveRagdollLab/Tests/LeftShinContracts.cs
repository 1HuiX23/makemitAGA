using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;
using Shin = MakemitAGA.ActiveRagdollLab.Physics.LeftShinForecast;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseTrace;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseForecast;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;
using Fixture = ARL.Contracts.LeftFootReleaseContracts;

namespace ARL.Contracts
{
    internal static class LeftShinContracts
    {
        private const float Dt = .02f;

        internal static void Run()
        {
            Check.Run("165 committed point force, angular units and the delayed knee COM write", Linear);
            Check.Run("165 ankle reaction in the lower principal frame, damping and speed limit", Angular);
            Check.Run("165 invalid bodies, clocks and uncommitted tether inputs", Invalid);
            Check.Run("165 lower receipts and successors are joined once within the same solve and bodies", Pairing);
            Check.Run("165 delayed normal lift uses a disjoint bounded capture segment", DelayedLift);
            Check.Run("165 five measured 164 B swing endpoints constrain the vertical tether", MeasuredEndpoints);
        }

        private static Shin.Body Body(Vector3 com) => new Shin.Body {
            Com = com, V = Vector3.zero, W = Vector3.zero, Principal = Quaternion.identity,
            Inertia = new Vector3(1,2,3), Mass = 2f, Drag = 0f, AngularDrag = 0f, MaxW = 18f
        };
        private static LegMeasuredAnchors Anchors()
        {
            var s = Fixture.Tick(1).Step;
            return LegMeasuredAnchors.Capture(true,s,1,s,1,new Vector3(.10f,.9f,0),
                new Vector3(0,.55f,.01f),new Vector3(.001f,.549f,.012f));
        }
        private static void Near(Vector3 actual, Vector3 expected, string label) =>
            Check.Near((actual-expected).magnitude,0f,3e-6f,label);

        private static void Linear()
        {
            var a = Anchors();
            var ankle = new Vector3(.01f,.08f,.02f);
            var upper = Body(new Vector3(0,.9f,0)); var lower = Body(new Vector3(0,.31f,0));
            upper.V = new Vector3(-.3f,0,.22f);
            lower.V = new Vector3(.15f,-.04f,.1f);
            lower.Drag = 1.5f;
            var c = new Shin.Commands {
                HipForce = new Vector3(0,-10,0), HipPoint = a.UpperHip, UpperTorque = Vector3.forward,
                CommonA = new Vector3(1,0,1), SpringA = new Vector3(40,0,0), DamperA = new Vector3(0,-30,0),
                ParentFeedforwardA = new Vector3(40,-30,0), KneeTetherA = new Vector3(48,-36,0), TetherCap = 60f
            };
            var r = Shin.Predict(Dt,a,ankle,upper,lower,c);
            Check.That(r.State == Shin.Status.Predicted,"valid committed window");
            // The point force moment cancels the actual actuator torque on Upper.
            var up = (upper.V + (c.CommonA + c.HipForce / upper.Mass) * Dt) * Dt;
            Near(r.Hip-a.UpperHip,up,"upper hip at a lever");
            Near(r.UpperKnee-a.UpperKnee,up,"no duplicated hip moment");
            var expected = (lower.V + (c.CommonA + c.KneeTetherA) * Dt) * (Dt*(1f-lower.Drag*Dt));
            Near(r.LowerAnkle-ankle,expected,"lower uses only its own issued COM force");
            Near(r.LowerKnee-a.LowerKnee,expected,"two anchors on the same translating body");
            Near(r.CoastCom+r.CommonCom+r.SpringCom+r.DamperCom+r.FeedforwardCom+r.Turn,expected,"additive displacement");
            Near(r.DamperCom,c.DamperA*(.6f*Dt*Dt*(1f-lower.Drag*Dt)),"the common cap also scales the damper");
            c.UpperTorque = Vector3.zero;
            var turn = Shin.Predict(Dt,a,ankle,upper,lower,c);
            var delta = Quaternion.AngleAxis(-Dt*Dt/3f*Mathf.Rad2Deg,Vector3.forward);
            Near(turn.Hip,upper.Com+up+delta*(a.UpperHip-upper.Com),"upper point force rotates about COM");
            Near(turn.LowerAnkle,r.LowerAnkle,"hip force is not added to the shin");
        }

        private static void Angular()
        {
            var a = Anchors(); var upper = Body(a.UpperHip); var lower = Body(a.LowerKnee);
            lower.Principal = Quaternion.AngleAxis(90,Vector3.forward);
            lower.Inertia = new Vector3(1,3,2); lower.W = Vector3.right; lower.AngularDrag = 2f;
            var ankle = a.LowerKnee + new Vector3(0,-.41f,.09f);
            var c = new Shin.Commands {
                TetherCap = 60, HipTransportAlpha = Vector3.right, KneeRelativeAlpha = Vector3.right*3f,
                AnkleReactionNm = Vector3.right*-6f
            };
            var r = Shin.Predict(Dt,a,ankle,upper,lower,c);
            Near(r.AnkleReactionAlpha,Vector3.right*-2f,"opposite foot torque uses rotated lower inertia");
            float speed = (1f+(1f+3f-2f)*Dt)*(1f-2f*Dt);
            var rot = Quaternion.AngleAxis(speed*Dt*Mathf.Rad2Deg,Vector3.right);
            Near(r.LowerAnkle,lower.Com+rot*(ankle-lower.Com),"direct knee angular acceleration is inertia independent");
            Near(r.LowerKnee,a.LowerKnee,"fixed lower COM at knee");
            Near(r.Turn,r.LowerAnkle-ankle,"ankle rotation contribution");
            lower.MaxW = .5f;
            r = Shin.Predict(Dt,a,ankle,upper,lower,c);
            rot = Quaternion.AngleAxis(.5f*Dt*Mathf.Rad2Deg,Vector3.right);
            Near(r.LowerAnkle,lower.Com+rot*(ankle-lower.Com),"native speed limit");
            lower.AngularDrag = 100f;
            Near(Shin.Predict(Dt,a,ankle,upper,lower,c).LowerAnkle,ankle,"nonnegative damping");
        }

        private static void Invalid()
        {
            var a = Anchors(); var body = Body(Vector3.zero); var c = new Shin.Commands { TetherCap = 60 };
            Check.That(Shin.Predict(0,a,Vector3.zero,body,body,c).State == Shin.Status.InvalidInput,"zero dt");
            Check.That(Shin.Predict(Dt,default,Vector3.zero,body,body,c).State == Shin.Status.InvalidInput,"missing producer");
            c.KneeTetherA = Vector3.up;
            Check.That(Shin.Predict(Dt,a,Vector3.zero,body,body,c).State == Shin.Status.InvalidInput,"provisional or wrong vector");
            c.KneeTetherA = Vector3.zero; c.TetherCap = float.NaN;
            Check.That(Shin.Predict(Dt,a,Vector3.zero,body,body,c).State == Shin.Status.InvalidInput,"missing cap");
            c.TetherCap = 60; var b = body; b.Principal = new Quaternion(0,0,0,2);
            Check.That(Shin.Predict(Dt,a,Vector3.zero,body,b,c).State == Shin.Status.InvalidInput,"non-unit principal frame");
            b = body; b.Inertia.z = 0;
            Check.That(Shin.Predict(Dt,a,Vector3.zero,b,body,c).State == Shin.Status.InvalidInput,"bad upper inertia");
            c.HipTransportAlpha.y = float.NaN;
            Check.That(Shin.Predict(Dt,a,Vector3.zero,body,body,c).State == Shin.Status.InvalidInput,"nonfinite direct acceleration");
        }

        private static Foot.Frame Frame(float height = -.0005f)
        {
            var f = Fixture.Frame(height); f.Anchors = Anchors(); f.UpperId = 33;
            return f;
        }
        private static Shin.Result Complete(Trace trace, Trace.Tick t, Foot.Frame frame, Vector3 offset)
        {
            var row = trace.PendingShin(t.Step,t.Source,t.Local);
            Check.That(row != null,"late receipt requested");
            var r = new Shin.Result { State = Shin.Status.Predicted,
                Hip = frame.Anchors.UpperHip, UpperKnee = frame.Anchors.UpperKnee, LowerKnee = frame.Anchors.LowerKnee,
                LowerAnkle = frame.Pose.Box.LowerAnkle + offset };
            trace.CompleteShin(row,r);
            Check.That(trace.PendingShin(t.Step,t.Source,t.Local) == null,"single use receipt"); return r;
        }

        private static void Pairing()
        {
            var trace = new Trace(); var f = Frame(); var m = Fixture.Command();
            var t = Fixture.Tick(10,Phase.Lift,true); trace.Observe(t,f,true,m);
            var stale = t.Step; stale.Dt += .001f; var altered = t.Source; altered.Time += .01f;
            Check.That(trace.PendingShin(stale,t.Source,10) == null && trace.PendingShin(t.Step,t.Source,11) == null &&
                trace.PendingShin(t.Step,altered,10) == null,"exact shared/local/source identity");
            var first = trace.PendingShin(t.Step,t.Source,10);
            Complete(trace,t,f,Vector3.right*.006f);
            Check.That(first.Command.State == Foot.Decision.Trial && first.Corrected.State == Foot.Decision.Gap,"real lateral difference");
            trace.CompleteShin(first,default);
            var t1 = Fixture.Tick(11); trace.Observe(t1,f,true,m);
            var second = trace.PendingShin(t1.Step,t1.Source,t1.Local);
            trace.CompleteShin(first,default);
            Check.That(!second.ShinCompleted && first.ShinPaired,"old receipt cannot supply the new interval");
            Complete(trace,t1,f,Vector3.zero);
            var n = Frame(-.0045f); trace.Observe(Fixture.Tick(12,Phase.Swing),n,true,m);
            var landing = Fixture.Tick(13,Phase.Stance); landing.Miss = true;
            trace.Observe(landing,default,false,default);
            string txt = trace.Summary();
            Check.That(txt.Contains("issued-predicted-paired=2|2|2") && txt.Contains("revisedTrials-paired-conditionedCross=1|1|1"),"paired revised summary");
            Check.That(txt.Contains("role=BeforeSwingEntry") && txt.Contains("role=SwingEntry") && txt.Contains("firstLanding-local=FirstMiss|13"),"prospective rows with retrospective event labels");
            Check.That(txt.Contains("lowerResidualMm") && txt.Contains("actualChainYmm"),"both readback decompositions");
            foreach (string mismatch in new[] {"upper","anchors","lower","floor","seal","missing"})
            {
                trace = new Trace(); trace.Observe(t,f,true,m); n = Frame();
                if (mismatch != "missing") Complete(trace,t,f,Vector3.zero);
                if (mismatch == "upper") n.UpperId++;
                if (mismatch == "anchors") n.Anchors = default;
                if (mismatch == "lower") n.LowerId++;
                t1 = Fixture.Tick(11,Phase.Swing); if (mismatch == "floor") t1.Support++;
                if (mismatch == "seal") trace.Seal();
                trace.Observe(t1,n,false,default);
                Check.That(trace.Summary().Contains("issued-predicted-paired=1|"+(mismatch=="missing"?"0":"1")+"|0"),"no invented successor: "+mismatch);
            }
        }

        private static void DelayedLift()
        {
            var tr = new Trace(); var f = Frame(); var m = Fixture.Command(3f);
            for (int i=1; i<=21; i++) {
                var t = Fixture.Tick(i,Phase.Stance,i==1);
                tr.Observe(t,tr.Sampling || i==1 ? f : default,true,m);
            }
            Check.That(!tr.Sampling && tr.NeedsFrame(Phase.Lift) && !tr.NeedsFrame(Phase.Stance),"cheap wait for first actual lift");
            for (int i=22; i<=24; i++) {
                var t = Fixture.Tick(i,i==24?Phase.Swing:Phase.Lift);
                tr.Observe(t,f,true,m); if (i<24) Complete(tr,t,f,Vector3.up*.001f);
            }
            var hit=Fixture.Tick(26,Phase.Stance); hit.Active=hit.Ordinary=true;
            tr.Observe(Fixture.Tick(25,Phase.Approach),default,false,default); tr.Observe(hit,default,false,default);
            string txt=tr.Summary();
            Check.That(txt.Contains("/resumedAtLift=22") && txt.Contains("firstLanding-local=Ordinary|26") &&
                txt.Contains("from-to=22|23/role=BeforeSwingEntry") && !txt.Contains("from-to=21|22"),"normal control without bridging unsampled stance");
            foreach (bool changeUpper in new[] {false, true})
            {
                tr = new Trace();
                for (int i = 1; i <= 12; i++) tr.Observe(Fixture.Tick(i,Phase.Stance,i == 1),f,true,m);
                var changed = f;
                if (changeUpper) changed.UpperId++; else changed.FootId++;
                tr.Observe(Fixture.Tick(13),changed,true,m);
                Check.That(!tr.Tracking,"changed foot or upper during the pause");
            }
        }

        // Measured lower endpoint supplied as a retrospective bound. The old log has
        // gap lengths and vertical components, but no horizontal heading or leg force receipt.
        private static void MeasuredEndpoints()
        {
            float[,] rows = {
                // pre sole, pre gap, pre signed gap Y, next sole/gap/Y, foot dY, issued A.y, bounded rise (mm)
                {-.253650f,4.006224f,.73389f,-2.525308f,4.602832f,-.24334f,-2.129443f,-6.793206f,.977226f},
                {-.241695f,4.594671f,2.66141f,-5.424957f,4.411200f,.51296f,-5.081773f,-17.760658f,1.896281f},
                {-.116784f,3.758998f,1.53791f,-3.395957f,4.597916f,.46037f,-3.141113f,-10.961384f,1.077540f},
                {-.442708f,4.603902f,2.73255f,-5.742298f,4.807495f,.21320f,-5.140409f,-17.673925f,1.177286f},
                {-.138622f,4.841497f,3.11338f,-6.109193f,4.364538f,.29586f,-5.868502f,-20.611794f,2.161444f}
            };
            foreach (var horizontal in new[] {Vector3.right,Vector3.forward,new Vector3(.6f,0,.8f)})
            {
                int crosses=0;
                for (int i=0;i<rows.GetLength(0);i++)
                {
                    var f=Frame(rows[i,0]*.001f); var box=f.Pose.Box; box.Dt=1f/60f;
                    float startGap=rows[i,1]*.001f, startY=rows[i,2]*.001f, endGap=rows[i,4]*.001f, endY=rows[i,5]*.001f;
                    box.LowerAnkle=box.FootAnkle-horizontal*MathF.Sqrt(startGap*startGap-startY*startY)-Vector3.up*startY;
                    f.Pose.Box=box; f.Pose.Gap=startGap;
                    var foot=box.FootAnkle+Vector3.up*(rows[i,6]*.001f);
                    var lower=foot-horizontal*MathF.Sqrt(endGap*endGap-endY*endY)-Vector3.up*endY;
                    var m=Fixture.Command(rows[i,7]); m.LinearDrag=.25f;
                    var original=new Foot.Result { Predicted=true,FootPoint=foot,Sole=rows[i,3]*.001f,State=Foot.Decision.Trial };
                    var r=Foot.WithIssuedLower(f,Phase.Lift,m,original,lower);
                    Check.That(r.State==Foot.Decision.Trial && r.TrialGap <= .005001f,"bounded corrected candidate");
                    Check.Near(r.Rise*1000f,rows[i,8],.00012f,"recorded 164 endpoint at heading "+i);
                    if (r.Sole+r.Rise >= -.002f) crosses++;
                }
                Check.That(crosses==1,"one of five reaches the ordinary depth in hindsight");
            }
        }
    }
}
