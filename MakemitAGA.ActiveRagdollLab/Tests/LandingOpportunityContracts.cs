using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Audit=MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;
namespace ARL.Contracts
{
    internal static class LandingOpportunityContracts
    {
        internal static void Run()
        {
            Check.Run("v142 opportunity screen preserves all independent exclusions", Screens);
            Check.Run("v142 actual ankle translation isolates Foot rotation without closing gaps", Tracking);
            Check.Run("v142 eight actual corner velocities, exact ties and invalid inputs", Corners);
            Check.Run("v142 held source streak duration, boundary censoring and terminal seal", Streaks);
            Check.Run("v142 source epochs, duplicate/backward and rounded timestamp continuity", Clocks);
            Check.Run("v142 first-strong/crossing evidence includes unadopted and unqualified rows", Events);
            Check.Run("v142 missing geometry and optional parent have honest separate denominators", Denominators);
            Check.Run("v142 opportunity accumulation has no hot-path allocations", Allocation);
        }
        internal static Audit.Sample S(int seq=1)=>new Audit.Sample
        {
            Step=new PhysicsStepContext{Epoch=7,Sequence=seq,SourceSequence=1,FixedTime=seq/60f,UnscaledTime=seq/60f,Dt=1f/60f},
            Source=new SourceStepContext{Epoch=7,Sequence=1,Time=0f,Dt=1f/60f},Fixed=100+seq,SupportId=42,
            Gait=Audit.Phase.Swing,SourceMotion=Audit.SourceKind.Motion,Identity=true,FivePoint=true,
            GeometryValid=true,MotionValid=true,TrackingValid=true,ParentValid=true,FullFootprint=true,
            FloorCenter=new Vector3(0,-.1f,0),FloorHalf=new Vector3(5,.1f,5),
            Min=.01f,PredictedMin=.009f,CornerVelocity=-.06f,AnchorGap=.003f,SourceAge=.01f,
            RawMin=.012f,ControlMin=.013f,AnkleDeltaY=-.002f,RelativeErrorDegrees=2f,
            RawHipY=1f,ReferenceHipY=1.002f,ActualHipY=.985f
        };
        private static void Screens()
        {
            var s=S();Check.That(Audit.Screen(s)==Audit.Rejection.None&&!s.StrongLanding,"phase-independent opportunity grants no stance authority");
            var reasons=new[]{Audit.Rejection.Geometry,Audit.Rejection.Footprint,Audit.Rejection.Motion,Audit.Rejection.Age,Audit.Rejection.Gap,
                Audit.Rejection.FivePoint,Audit.Rejection.Identity,Audit.Rejection.Height,Audit.Rejection.Descending,Audit.Rejection.Projection};
            for(int i=0;i<reasons.Length;i++)
            {
                s=S();switch(i){case 0:s.GeometryValid=false;break;case 1:s.FullFootprint=false;break;case 2:s.MotionValid=false;break;
                    case 3:s.SourceAge=.101f;break;case 4:s.AnchorGap=.0051f;break;case 5:s.FivePoint=false;break;case 6:s.Identity=false;s.SelectedBoxFallback=true;break;
                    case 7:s.Min=.026f;break;case 8:s.CornerVelocity=.001f;break;case 9:s.PredictedMin=-.0021f;break;}
                Check.That((Audit.Screen(s)&reasons[i])!=0,"independent reject "+reasons[i]);
            }
            s=S();s.SourceAge=.1f;s.AnchorGap=.005f;s.Min=.001f;s.PredictedMin=-.002f;
            Check.That(Audit.Screen(s)==Audit.Rejection.None,"inclusive age/gap/height/projection boundaries");
            s.CornerVelocity=-.006f;Check.That((Audit.Screen(s)&Audit.Rejection.Descending)!=0,"descending threshold is strict");
            s=S();s.CornerVelocity=float.NaN;s.SourceAge=float.NaN;Check.That(!s.PositiveDescending&&Audit.Screen(s)!=Audit.Rejection.None,"NaN cannot become an opportunity");
            Check.That(Audit.ClassifySource("RAPID_CONTINUOUS_TURN")==Audit.SourceKind.Unknown,"recovery/event labels cannot impersonate source phase");
            Check.That(Audit.ClassifySource("TURN")==Audit.SourceKind.Motion&&Audit.ClassifySource("IDLE")==Audit.SourceKind.Idle,"source classes explicit");
        }
        private static float Height(Vector3 ankle,Quaternion q)
        {
            float min=float.PositiveInfinity;
            for(int x=-1;x<=1;x+=2)for(int y=-1;y<=1;y+=2)for(int z=-1;z<=1;z+=2)
                min=MathF.Min(min,(ankle+q*(new Vector3(.01f,-.08f,.05f)+new Vector3(x*.0475f,y*.0275f,z*.1172f))).y);
            return min;
        }
        private static void Tracking()
        {
            for(int a=-20;a<=20;a++)
            {
                var s=S();var lowerAnkle=new Vector3(.2f,.115f,.1f);var footAnkle=lowerAnkle+new Vector3(.004f,-.003f,.001f);
                var control=Quaternion.AngleAxis(a,Vector3.right);var actual=Quaternion.AngleAxis(a+4f,Vector3.right);
                s.ControlMin=Height(lowerAnkle,control);s.Min=Height(footAnkle,actual);s.AnkleDeltaY=footAnkle.y-lowerAnkle.y;
                Check.Near(s.ControlAtActualAnkle,Height(footAnkle,control),1e-6f,"translation of minimum commutes with all corners");
                Check.Near(s.FootRotationDelta,Height(footAnkle,actual)-Height(footAnkle,control),1e-6f,"actual ankle, not closed lower anchor");
            }
        }
        private static void Corners()
        {
            FootLowestCornerMotion c=default;
            c.Add(-.01f,.2f);c.Add(-.01f,-.3f);for(int i=2;i<8;i++)c.Add(.02f,-5f);
            Check.That(c.Valid,"eight finite actual corners");Check.Near(c.Velocity,-.3f,0f,"minimum tied-bottom velocity, not top or COM");
            c.Add(-.02f,0);Check.That(!c.Valid,"exactly eight corners");
            c=default;for(int i=0;i<8;i++)c.Add(i==4?float.NaN:0,0);Check.That(!c.Valid,"nonfinite corner makes motion missing");
            c=default;for(int i=0;i<8;i++)c.Add(i==4?-.03f:0,i==4?.08f:-2f);Check.Near(c.Velocity,.08f,0,"unique lowest corner may be moving up");
        }
        private static void Streaks()
        {
            var a=new Audit();a.Observe(S(1));
            Check.That(a.Summary().Contains("maxSamples-maxSeconds=1|0/"),"first good sample does not borrow an unseen interval");
            for(int i=2;i<=7;i++)a.Observe(S(i));
            string t=a.Summary();Check.That(t.Contains("kind=PHASE_INDEPENDENT/started-completed-censored-open=1|0|0|1"),"held Render is a continuous physical opportunity");
            Check.That(t.Contains("atLeast2-atLeast50ms-atLeast100ms=1|1|1"),"first sample adds zero, six intervals reach 100ms");
            a.Seal();string sealedText=a.Summary();a.Seal();a.Observe(S(8));a.Fault();Check.That(a.Summary()==sealedText,"sealed trace is immutable and seal idempotent");
            Check.That(sealedText.Contains("started-completed-censored-open=1|0|1|0"),"stop censors current opportunity");
            a.Reset();for(int i=1;i<=3;i++)a.Observe(S(i));a.Observe(S(5));a.Observe(S(6));
            Check.That(a.Summary().Contains("started-completed-censored-open=2|0|1|1"),"missing Fixed censors, cannot borrow prior interval");
            var active=S(7);active.WasActive=true;active.NowActive=true;a.Observe(active);
            Check.That(a.Summary().Contains("started-completed-censored-open=2|0|2|0"),"active contact ends inactive streak without pretending physical outcome");
            a.Reset();a.Observe(S(1));var moved=S(2);moved.FloorCenter.y+=.001f;a.Observe(moved);
            Check.That(a.Summary().Contains("started-completed-censored-open=2|0|1|1"),"same collider moved plane cannot join opportunity");
        }
        private static void Clocks()
        {
            var a=new Audit();a.Observe(S());a.Observe(S());a.Observe(S());a.Observe(S(2));
            Check.That(a.Summary().Contains("observed-active-inactive=2|0|2"),"repeated duplicate never grows count");
            Check.That(a.Summary().Contains("invalidClock-duplicateOrBackward-breaks-faults=0|2|0|0"),"duplicate explicitly separated from missing geometry");
            a.Reset();var one=S(1);one.Step.FixedTime=100000f;a.Observe(one);var two=S(2);two.Step.FixedTime=100000f;a.Observe(two);
            Check.That(a.Summary().Contains("atLeast2-atLeast50ms-atLeast100ms=1|0|0"),"rounded equal float FixedTime still has consecutive integer clock");
            a.Reset();a.Observe(S(1));var foreign=S(2);foreign.Source.Sequence=3;a.Observe(foreign);a.Observe(S(3));
            Check.That(a.Summary().Contains("invalidClock-duplicateOrBackward-breaks-faults=1|0|0|0"),"different source does not share step identity");
            a.Reset();a.Observe(S(1));var rebound=S(2);rebound.Step.Epoch=rebound.Source.Epoch=8;a.Observe(rebound);
            Check.That(a.Summary().Contains("started-completed-censored-open=2|0|1|1"),"binding epoch cannot borrow an earlier window");
        }
        private static void Events()
        {
            var a=new Audit();var s=S(1);s.Min=.0025f;s.Identity=false;s.Strong=false;a.Observe(s);
            s=S(2);s.Min=-.00004f;s.Identity=false;a.Observe(s);
            s=S(3);s.Gait=Audit.Phase.Approach;s.Strong=true;s.Min=-.00435f;a.Observe(s);
            var t=a.Summary();Check.That(t.Contains("crossings-total-withoutStrong-withoutIdentity=1|1|1"),"crossing before existing authorization is retained");
            Check.That(t.Contains("firstStrong(known-above-shallow-deep-geometryMissing-leftCensored)=1|0|0|1|0|0"),"first strong evidence is already below ordinary entry");
            Check.That(t.Contains("adopted-unadopted=0|3")&&t.Contains("selection=CROSS_PREVIOUS"),"unadopted preparation still has evidence");
            a.Reset();s=S();s.Gait=Audit.Phase.Approach;s.Strong=true;a.Observe(s);
            Check.That(a.Summary().Contains("firstStrong(known-above-shallow-deep-geometryMissing-leftCensored)=0|0|0|0|0|1"),"trace beginning is not a known first-strong edge");
        }
        private static void Denominators()
        {
            var a=new Audit();var adopted=S(1);adopted.Adopted=true;a.Observe(adopted);var missing=S(2);missing.GeometryValid=false;missing.ParentValid=false;missing.TrackingValid=false;missing.SourceMotion=Audit.SourceKind.Unknown;a.Observe(missing);
            string t=a.Summary();Check.That(t.Contains("adopted-unadopted=1|1"),"adopted and unadopted share the complete observation denominator");Check.That(t.Contains("group=ALL/inactive-valid-missing-motion=2|1|1|1"),"missing not filled as zero");
            Check.That(t.Contains("trackingValid=1")&&t.Contains("parentValid=1"),"optional domains have independent denominators");
            Check.That(t.Contains("group=SOURCE_UNKNOWN/inactive-valid-missing-motion=1|0|1|0"),"unknown remains explicit");
            string parentKey="parentMeanMm(rawToReference-referenceToPhysical)=";
            int at=t.IndexOf(parentKey,StringComparison.Ordinal)+parentKey.Length;
            string[] parent=t.Substring(at).Split('/','\n')[0].Split('|');
            Check.Near(float.Parse(parent[0],CultureInfo.InvariantCulture),2f,.0001f,"source to reference has independent meaning");
            Check.Near(float.Parse(parent[1],CultureInfo.InvariantCulture),-17f,.0001f,"physical parent sag is separated from reference placement");
            a.Reset();Check.That(a.Summary().Contains("minMeanMm=NA/trackingValid=0"),"empty means are NA");
        }
        private static void Allocation()
        {
            var a=new Audit();for(int i=1;i<=100;i++)a.Observe(S(i));
            long start=GC.GetAllocatedBytesForCurrentThread();for(int i=101;i<=4100;i++)a.Observe(S(i));
            Check.That(GC.GetAllocatedBytesForCurrentThread()==start,"no formatting or allocation per sample");
        }
    }
}
