using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;
using Forecast = MakemitAGA.ActiveRagdollLab.Physics.RightFootTorqueForecast;
using Signal = MakemitAGA.ActiveRagdollLab.Physics.RightThighWindow.Signal;

namespace ARL.Contracts
{
    internal static class RightFootTorqueContracts
    {
        internal static void Run()
        {
            Check.Run("v151 same-state three-axis foot inertia, damping, cap and independent eight corners", Inertia);
            Check.Run("v151 consecutive physics ticks share an onset interval and isolate real ankle translation", Continuity);
            Check.Run("v151 broken identity, receipt, geometry or clock cannot calibrate a forecast", Interruptions);
            Check.Run("v151 first strong edges preserve goals with opposite signs and untriggered controls", Events);
            Check.Run("v151 duration follows physics time, bounded rows keep every aggregate and allocate no hot-path memory", Cost);
        }
        private static RightKneePivot.Frame Pose() => RightKneePivotContracts.Pose(.001f, 5f, .0005f);
        private static Audit.Sample Sample(int n, in RightKneePivot.Frame p, bool onset = false)
        {
            var s = SwingKneeContracts.Sample(n, p.Dt, n*p.Dt);
            s.EarlyFoot = EarlyLandingWindow.Foot.Right; s.RightThigh = onset ? Signal.Onset : Signal.None;
            s.FloorCenter=p.FloorCenter; s.FloorHalf=p.FloorHalf; s.Min=p.Box; s.TrackingValid=true;
            s.AnkleHeights = new Vector3(p.RawAnkleY,p.LowerAnkle.y,p.FootAnkle.y);
            s.AnkleDeltaY = p.FootAnkle.y-p.LowerAnkle.y; s.ControlMin = .003f; s.ParentHeightMeasured = true;
            return s;
        }
        private static Forecast.BodyMotion Motion() => new Forecast.BodyMotion { Mass=2f, Inertia=new Vector3(.19f,.036f,.088f),
            WorldPrincipal=Quaternion.AngleAxis(51f,new Vector3(.4f,-.3f,.6f).normalized), W=new Vector3(.76f,-.52f,.85f),
            Drag=2.5f, MaxW=7f, Transport=new Vector3(.8f,-.1f,.29f),Drive=new Vector3(-.17f,.21f,.34f),Support=new Vector3(.09f,-.48f,.31f) };
        private static float Min8(in RightKneePivot.Frame p, Vector3 foot, Quaternion r)
        {
            float min=float.PositiveInfinity;
            for(int i=0;i<8;i++)
            {
                var v=new Vector3((i&1)==0?p.Half.x:-p.Half.x,(i&2)==0?p.Half.y:-p.Half.y,(i&4)==0?p.Half.z:-p.Half.z);
                min=Math.Min(min,(foot+r*(p.Center+v)).y-p.FloorCenter.y-p.FloorHalf.y);
            }
            return min;
        }
        private static void Inertia()
        {
            var p=Pose(); var foot=p.FootAnkle+new Vector3(.03f,-.002f,.06f); var m=Motion();
            var tau=m.Transport+m.Drive+m.Support;
            var x=m.WorldPrincipal*Vector3.right; var y=m.WorldPrincipal*Vector3.up; var z=m.WorldPrincipal*Vector3.forward;
            var a=x*(Vector3.Dot(x,tau)/m.Inertia.x)+y*(Vector3.Dot(y,tau)/m.Inertia.y)+z*(Vector3.Dot(z,tau)/m.Inertia.z);
            foreach(float limit in new[]{.75f,7f})
            {
                m.MaxW=limit; var f=Forecast.Predict(p,p.LowerAnkle,foot,m);
                Check.That(f.State==Forecast.Status.Predicted,"receipted command");
                var w=Vector3.ClampMagnitude((m.W+a*p.Dt)*(1-m.Drag*p.Dt),limit);
                Check.Near((f.Issued.W-w).magnitude,0f,.000005f,"principal-axis expansion plus drag and cap");
                var r=Quaternion.AngleAxis(w.magnitude*p.Dt*Mathf.Rad2Deg,w.normalized)*p.FootR;
                Check.Near(f.Issued.Box,Min8(p,foot,r),.000001f,"arbitrary tilted eight corners");
                Check.Near(f.Coast.Box,Min8(p,foot,FootRelativeReference.Advance(p.FootR,Vector3.ClampMagnitude(m.W,limit),p.Dt)),.000001f,"original V150 coast");
                Check.That((f.Issued.W-f.Free.W).magnitude>.01f, "issued torque independent of damping");
            }
            m.Transport=m.Drive=m.Support=Vector3.zero;
            var clean=Forecast.Predict(p,p.LowerAnkle,foot,m);
            Check.That(clean.State==Forecast.Status.Predicted && clean.Issued.Box==clean.Free.Box,"valid zero command");
            m.Drag=99f; clean=Forecast.Predict(p,p.LowerAnkle,foot,m);
            Check.That(clean.Issued.W.sqrMagnitude==0 && clean.Free.W.sqrMagnitude==0,"large damping cannot reverse direction");
            m.Inertia.y=0; Check.That(Forecast.Predict(p,p.LowerAnkle,foot,m).State==Forecast.Status.InvalidMotion,"singular inertia");
            m=Motion();m.WorldPrincipal=default; Check.That(Forecast.Predict(p,p.LowerAnkle,foot,m).State==Forecast.Status.InvalidMotion,"missing principal frame");
            m=Motion();p.Box+=.0005f; Check.That(Forecast.Predict(p,p.LowerAnkle,foot,m).State==Forecast.Status.Geometry,"independent physical box closure");
        }
        private static void Continuity()
        {
            var p=Pose();var m=Motion();var t=new RightFootForecastWindow();
            var s=Sample(1,p,true);var held=s.Source;
            var f=Forecast.Predict(p,p.LowerAnkle,p.FootAnkle,m);
            t.Observe(s,p,7,m.W);t.Complete(f);
            // Same render packet; only the physical ankle translated. Issued rotation is exact at that ankle.
            var q=p;q.FootR=f.Issued.R;q.FootAnkle.y+=.004f;q.LowerAnkle.y+=.003f;q.Box=Min8(q,q.FootAnkle,q.FootR);
            s=Sample(2,q,true);s.Source=held;s.Step.SourceSequence=held.Sequence;s.Gait=Audit.Phase.Approach;
            s.AnchorGap=.0061f;t.Observe(s,q,7,f.Issued.W);
            Check.That(t.Requested(s.Step),"overlap does not retire forecast when previous thigh gate clears");
            t.Complete(Forecast.Predict(q,q.LowerAnkle,q.FootAnkle,m));
            s=Sample(3,q);s.Gait=Audit.Phase.Approach;s.Strong=true;s.NowActive=true;t.Observe(s,q,7,m.W);
            string log=t.Summary();Check.That(!t.Active && !t.Requested(s.Step) && log==t.Summary(),"terminal strong edge read once");
            Check.That(log.Contains("/origins-episodes=2|1") && log.Contains("/steps-predicted-measured-interrupted=2|2|2|0") &&
                log.Contains("/over5mm=1") && log.Contains("/first-latestOrigin=1|2") && log.Contains("/nextSource-phase=1|Approach"),"unique physical samples, held source and 5mm flagged");
            var lines=log.Split('\n');var first=Array.Find(lines,v=>v.Contains("/local-shared-source=1|1|1"));
            float Value(string tag,int at)=>float.Parse(first.Split(tag)[1].Split('/')[0].Split('|')[at],System.Globalization.CultureInfo.InvariantCulture);
            Check.Near(Value("/fixedActualAnkleBoxMm=",2),Value("/nextLower-foot-box-goalMm=",2),.00015f,"fixed actual ankle removes translation");
            Check.Near(Value("/nextLower-foot-box-goalMm=",2)-Value("/boxMm=",2),4f,.00015f,"full prediction retains foot translation error");
        }
        private static void Interruptions()
        {
            for(int i=0;i<16;i++)
            {
                var p=Pose();var m=Motion();var t=new RightFootForecastWindow();var s=Sample(1,p,true);
                t.Observe(s,p,8,m.W);if(i!=13)t.Complete(Forecast.Predict(p,p.LowerAnkle,p.FootAnkle,m));
                var q=p;var next=Sample(2,p);int id=8;
                switch(i)
                {
                    case 0:next.Fixed++;break;case 1:next.Step.Epoch++;next.Source.Epoch++;break;case 2:next.Step.SourceSequence++;break;
                    case 3:next.Source.Time=-1;break;case 4:next.FloorCenter.z+=.02f;q.FloorCenter=next.FloorCenter;break;
                    case 5:next.SupportId++;break;case 6:id++;break;case 7:next.SelectedBoxFallback=true;break;
                    case 8:next.FivePoint=false;break;case 9:next.Min=float.NaN;break;case 10:next.WasActive=true;break;
                    case 11:q.Center.x+=.01f;break;case 12:next.SourceAge=.101f;break;case 13:break;
                    case 14:next.FullFootprint=false;break;case 15:next.Identity=false;break;
                }
                next.Gait=Audit.Phase.Stance;next.Strong=true;t.Observe(next,q,id,m.W);string log=t.Summary();
                Check.That(log.Contains(i==13?"/MissingController=1":"/steps-predicted-measured-interrupted=1|1|0|1") &&
                    log.Contains("/boxMaeMm=NA"),"unavailable measurements excluded "+i);
            }
        }
        private static void Events()
        {
            foreach(var outcome in new[]{"Enabled","HoldEnded","Recovered"})
            foreach(bool onset in new[]{false,true})
            foreach(float target in new[]{-.011f,.004f,.026f})
            foreach(bool fresh in new[]{true,false})
            {
                var p=Pose();var history=new LandingEventHistory();
                for(int i=1;i<=7;i++)
                {
                    var s=Sample(i,p,onset && i==2);s.SwingMeasured=false;
                    s.ControlMin=target-s.AnkleDeltaY; s.SourceAge=fresh?.1f:.1027832f;
                    if(i>=5){s.Gait=Audit.Phase.Approach;s.Strong=true;}
                    if(i==7){s.OrdinaryEnabled=outcome=="Enabled";s.FirstMiss=!s.OrdinaryEnabled;s.DepthLatchFixed=5;}
                    history.Add(s,i/60d,Audit.Screen(s),p);
                    if(i==7){history.Capture(s,s.FirstMiss);if(s.FirstMiss)history.EndMiss(outcome=="Recovered"?LandingEventHistory.MissEnd.Recovered:LandingEventHistory.MissEnd.HoldEnded);}
                }
                var b=new StringBuilder();history.Append(b);
                var line=Array.Find(b.ToString().Split('\n'),v=>v.StartsWith("RIGHT_FOOT_POSE_EVENT_V041510"));
                Check.That(line!=null && line.Contains("/outcome="+outcome) && line.Contains("/anchorLocal=5") &&
                    line.Contains(onset?"/first-latestOrigin=2|2":"/first-latestOrigin=0|0") &&
                    line.Contains("/geometryComparable=True/sourceFresh="+fresh) &&
                    line.Contains(fresh ? "/actual-goalBand=Below|"+Forecast.Band(target) : "/actual-goalBand=Missing|Missing"), "actual and held goal at identical ankle for "+outcome);
            }
            Check.That(Forecast.Band(-.002f)==Forecast.HeightBand.InBand && Forecast.Band(.025f)==Forecast.HeightBand.InBand &&
                Forecast.Band(float.NaN)==Forecast.HeightBand.Missing, "vertical boundaries are inclusive");
        }
        private static void Cost()
        {
            var m=Motion();var p=Pose();
            foreach(int hz in new[]{25,50,60,100,200})
            {
                var t=new RightFootForecastWindow();p.Dt=1f/hz;
                for(int i=1;i<=hz;i++){var s=Sample(i,p,i==1);t.Observe(s,p,5,m.W);if(t.Requested(s.Step))t.Complete(Forecast.Predict(p,p.LowerAnkle,p.FootAnkle,m));}
                int count=hz/5;Check.That(t.Summary().Contains("/steps-predicted-measured-interrupted="+count+"|"+count+"|"+count+"|0"),"wall length "+hz);
            }
            var window=new RightFootForecastWindow();p.Dt=.01f;
            void Tick(int i){var s=Sample(i,p,i%10==1);window.Observe(s,p,9,m.W);window.Complete(Forecast.Predict(p,p.LowerAnkle,p.FootAnkle,m));}
            for(int i=1;i<=80;i++)Tick(i);
            long before=GC.GetAllocatedBytesForCurrentThread();for(int i=81;i<=300;i++)Tick(i);long used=GC.GetAllocatedBytesForCurrentThread()-before;
            Check.That(used==0,"allocated "+used+" bytes in sample/forecast/readback");
            string text=window.Summary();Check.That(text.Contains("/saved-omitted=256|44") && text.Contains("/steps-predicted-measured-interrupted=300|300|299|1"),"bounded rows, aggregate all intervals and last pending");
        }
    }
}
