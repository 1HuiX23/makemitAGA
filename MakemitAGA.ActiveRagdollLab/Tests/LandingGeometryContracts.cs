using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class LandingGeometryContracts
    {
        internal static void Run()
        {
            Check.Run("v141 eight-corner decomposition isolates positions, rotations and all anchor gaps", Decomposition);
            Check.Run("v141 same-state deltas preserve observed errors and world separations", Probes);
            Check.Run("v141 geometry source/fixed/foot/shape qualification and finite-footprint masks", Rejection);
            Check.Run("v141 all-response denominators, disjoint kinds and optional geometry loss", Aggregation);
            Check.Run("v141 geometry and response accumulation allocate no hot-path bytes", Allocation);
        }
        private struct Case
        {
            internal LandingPreparationCommand C;
            internal LegLandingReferenceInput I;
            internal FootFixedStepEvidence B;
            internal Quaternion Lower, Foot;
            internal LandingPreparationGeometry.Sample Build() => LandingPreparationGeometry.Build(C,I,B,Lower,Foot,C.PublishedFoot);
        }
        private static Case Fixture()
        {
            var sourceFixture = HipLandingWitnessFixtures.All()[0];
            Quaternion k = Quaternion.AngleAxis(20f,Vector3.right);
            var c = new LandingPreparationCommand { Step=sourceFixture.Knee.Step,Source=sourceFixture.Knee.Source,
                HipApplied=true,Approach=true,SupportId=21,GeometryPrepared=true,
                RenderPhysicalHip=new Vector3(.1f,.9f,.2f),UpperSpan=new Vector3(0f,-.42f,0f),LowerSpan=new Vector3(0f,-.4f,0f),
                HipBaseline=Quaternion.AngleAxis(-.2f,Vector3.forward),HipSelected=Quaternion.identity,
                KneeBaseline=Quaternion.AngleAxis(20.3f,Vector3.right),KneeSelected=k,PublishedFoot=Quaternion.Inverse(k),
                Center=new Vector3(.01f,-.025f,.04f),Half=new Vector3(.05f,.025f,.12f),
                FloorCenter=new Vector3(0f,-.05f,0f),FloorHalf=new Vector3(5f,.05f,5f) };
            var input = new LegLandingReferenceInput { Prepared=true,Step=c.Step,Source=c.Source,Fixed=501,
                UpperGoal=c.HipSelected,KneeRelativeGoal=k,Upper=Quaternion.identity,
                ParentHip=c.RenderPhysicalHip+Vector3.up*.02f,UpperHip=c.RenderPhysicalHip+Vector3.up*.017f,
                UpperHipToKnee=c.UpperSpan,LowerKneeAnchor=Vector3.zero };
            Vector3 lowerAnkle=input.UpperHip+input.Upper*c.UpperSpan+k*c.LowerSpan+Vector3.up*.004f;
            var before=new FootFixedStepEvidence { FixedSequence=501,Valid=true,PreparationPoseValid=true,PreparationSupportId=21,
                LowerAnchorWorld=lowerAnkle,FootAnchorWorld=lowerAnkle-Vector3.up*.005f,AnchorErrorMeters=.005f,
                FootAnkleVelocityWorld=Vector3.up*.23f };
            return new Case { C=c,I=input,B=before,Lower=k,Foot=Quaternion.identity };
        }
        private static float Corners(Vector3 ankle,Quaternion q,LandingPreparationCommand c)
        {
            float min=float.PositiveInfinity;
            for(int x=-1;x<=1;x+=2)for(int y=-1;y<=1;y+=2)for(int z=-1;z<=1;z+=2)
                min=MathF.Min(min,(ankle+q*(c.Center+new Vector3(x*c.Half.x,y*c.Half.y,z*c.Half.z))).y-c.FloorCenter.y-c.FloorHalf.y);
            return min;
        }
        private static void Decomposition()
        {
            var f=Fixture();var s=f.Build();Check.That(s.Valid&&s.ProbeValid,"base state is qualified");
            Check.Near(s.ParentDelta,.02f,.000002f,"Render hip to actual parent hip");
            Check.Near(s.HipGapDelta,-.003f,.000002f,"separate hip anchor separation");
            Check.Near(s.KneeGapDelta,.004f,.000002f,"separate knee anchor separation");
            Check.Near(s.AnkleGapDelta,-.005f,.000002f,"separate ankle anchor separation");
            Check.Near(s.HipRotationDelta,0f,.000002f,"no invented hip rotation contribution");
            Check.Near(s.KneeRotationDelta,0f,.000002f,"no invented knee rotation contribution");
            Check.Near(s.FootRotationDelta,0f,.000002f,"no invented foot rotation contribution");
            Check.Near(s.CornerNormalSpeed,.23f,.000002f,"ankle point speed copied, not COM speed");
            f.B.FootAngularVelocityWorld=new Vector3(0f,0f,2f);s=f.Build();
            Check.Near(s.CornerNormalSpeed,.23f+2f*(f.C.Center.x-f.C.Half.x),.000002f,"tied bottom corners select minimum instantaneous speed");
            var random=new Random(14101);
            for(int n=0;n<300;n++)
            {
                f=Fixture();f.I.Upper=Q(random);f.Lower=f.I.Upper*Q(random);f.Foot=Q(random);
                f.B.LowerAnchorWorld=f.I.UpperHip+f.I.Upper*f.C.UpperSpan+f.Lower*f.C.LowerSpan+new Vector3(.002f,-.003f,.004f);
                f.B.FootAnchorWorld=f.B.LowerAnchorWorld+new Vector3(-.003f,.006f,.001f);
                s=f.Build();Check.That(s.Valid,"random qualified geometry");
                Quaternion goalFoot=f.C.HipSelected*f.C.KneeSelected*f.C.PublishedFoot;
                Vector3 goal=f.C.RenderPhysicalHip+f.C.HipSelected*f.C.UpperSpan+(f.C.HipSelected*f.C.KneeSelected)*f.C.LowerSpan;
                Check.Near(s.Reference,Corners(goal,goalFoot,f.C),.000002f,"independent reference corners");
                Check.Near(s.Actual,Corners(f.B.FootAnchorWorld,f.Foot,f.C),.000002f,"independent actual corners");
                Check.Near(s.Actual-s.Reference,s.ParentDelta+s.HipRotationDelta+s.KneeRotationDelta+s.HipGapDelta+s.KneeGapDelta+s.AnkleGapDelta+s.FootRotationDelta,.000005f,"full ordered telescope");
                Check.Near(s.FootRotationDelta,Corners(f.B.FootAnchorWorld,f.Foot,f.C)-Corners(f.B.FootAnchorWorld,f.Lower*f.C.PublishedFoot,f.C),.000002f,"orientation replaces the minimum corner, not one fixed old corner");
            }
        }
        private static Quaternion Q(Random r) => Quaternion.AngleAxis((float)(r.NextDouble()*30-15),new Vector3(.3f,.5f,.7f).normalized)*
            Quaternion.AngleAxis((float)(r.NextDouble()*20-10),Vector3.right);
        private static void Probes()
        {
            var random=new Random(14102);bool mixed=false;
            for(int n=0;n<300;n++)
            {
                var f=Fixture();f.C.KneeApplied=true;f.C.HipBaseline=Q(random);f.C.KneeBaseline=Q(random);
                f.I.Upper=Q(random);f.Lower=f.I.Upper*Q(random);f.Foot=Q(random);
                // Arbitrary observed gaps are intentional. A probe must NOT silently
                // close them or move the current actual hip to the planned hip.
                f.B.LowerAnchorWorld+=new Vector3(.02f,-.03f,.01f);f.B.FootAnchorWorld+=new Vector3(-.01f,-.018f,.022f);
                var s=f.Build();Check.That(s.Valid&&s.ProbeValid,"same-state baseline reconstructs actual state");
                Quaternion u=f.I.Upper, k=Quaternion.Inverse(u)*f.Lower;
                Quaternion hu=u*Quaternion.Inverse(f.C.HipBaseline)*f.C.HipSelected;
                Quaternion kk=k*Quaternion.Inverse(f.C.KneeBaseline)*f.C.KneeSelected;
                Quaternion localFoot=Quaternion.Inverse(f.Lower)*f.Foot;
                foreach(int mode in new[]{1,2,3})
                {
                    Quaternion cu=mode==2?u:hu,ck=mode==1?k:kk;
                    // Independent endpoint-displacement formulation: no virtual
                    // absolute anchor reconstruction or summed-debt implementation.
                    Vector3 candidate=f.B.FootAnchorWorld+(cu*f.C.UpperSpan-u*f.C.UpperSpan)+((cu*ck)*f.C.LowerSpan-f.Lower*f.C.LowerSpan);
                    float expected=Corners(candidate,cu*ck*localFoot,f.C);
                    float measured=mode==1?s.HipProbe:mode==2?s.KneeProbe:s.BothProbe;
                    Check.Near(measured,expected,.000003f,"noncommuting relative error mapping mode "+mode);
                }
                if((s.HipProbe-s.Actual)*(s.KneeProbe-s.Actual)<0f)mixed=true;
            }
            Check.That(mixed,"counterexamples include opposing Hip/Knee directions");
            var identity=Fixture();identity.C.HipBaseline=identity.C.HipSelected;identity.C.KneeBaseline=identity.C.KneeSelected;
            var z=identity.Build();Check.Near(z.HipProbe,z.Actual,.000002f,"no Hip increment is identity");
            Check.Near(z.KneeProbe,z.Actual,.000002f,"no Knee increment is identity");
            Check.Near(z.BothProbe,z.Actual,.000002f,"no combined increment is identity");
        }
        private static void Rejection()
        {
            for(int n=0;n<13;n++)
            {
                var f=Fixture();Quaternion consumed=f.C.PublishedFoot;
                if(n==0)f.I.Source.Sequence++;
                if(n==1)f.I.Step.Sequence++;
                if(n==2)f.I.Fixed++;
                if(n==3)f.B.PreparationSupportId++;
                if(n==4)f.C.GeometryPrepared=false;
                if(n==5)f.C.UpperSpan.x+=.001f;
                if(n==6)consumed=Quaternion.identity;
                if(n==7)f.I.KneeRelativeGoal=Quaternion.identity;
                if(n==8)f.C.HipBaseline=default;
                if(n==9)f.B.LowerAnchorWorld.x=float.NaN;
                if(n==10)f.C.FloorHalf.x=0;
                if(n==11)f.B.FootAnchorWorld.x=100;
                if(n==12)f.Foot=new Quaternion(0f,0f,0f,2f);
                Check.That(!LandingPreparationGeometry.Build(f.C,f.I,f.B,f.Lower,f.Foot,consumed).Valid,"geometry guard "+n);
            }
            var outside=Fixture();outside.C.RenderPhysicalHip.x=100f;var s=outside.Build();
            Check.That(s.Valid&&(s.FootprintMask&1)==0&&(s.FootprintMask&128)!=0,"intermediate plane extension explicitly marked, actual stays inside");
        }
        private static LandingPreparationResponse.Sample Response(Case f,float dy)
        {
            var next=f.C.Step;next.Sequence++;next.FixedTime+=next.Dt;next.UnscaledTime+=next.Dt;
            var after=f.B;after.FixedSequence++;after.FootAnchorWorld.y+=dy;
            return LandingPreparationResponse.Measure(f.C,f.B,f.Foot,next,after,f.Foot,f.Build());
        }
        private static void Aggregation()
        {
            var r=new LandingPreparationResponse();var f=Fixture();var a=Response(f,.002f);r.Observe(f.C,true,a);
            f.C.HipApplied=false;f.C.KneeApplied=true;f.C.Approach=false;var b=Response(f,-.002f);r.Observe(f.C,true,b);
            f.C.HipApplied=true;f.C.Approach=true;var c=Response(f,0f);c.Geometry=default;r.Observe(f.C,true,c);
            string summary=r.Summary();
            Check.That(summary.Contains("group=ALL/resolved=3/actualUp-down-flat=1|1|1"),"all three actual directions");
            Check.That(summary.Contains("group=HIP_ONLY/resolved=1")&&summary.Contains("group=KNEE_ONLY/resolved=1")&&summary.Contains("group=BOTH/resolved=1"),"disjoint union, shared Hip+Knee counted once");
            Check.That(summary.Contains("group=APPROACH/resolved=2"),"approach is a labelled subset");
            Check.That(summary.Contains("geometry(valid-missing-allFootprintsInside-refBelowActualAbove)=2|1|2|0"),"missing geometry cannot become a zero-height sample");
            Check.That(summary.Contains("MAX_REFERENCE_ACTUAL_GAP")&&summary.Contains("diagnosticFeedback=False"),"bounded explanatory witness");
            var foreign=c;foreign.Command.Source.Sequence++;r.Observe(f.C,true,foreign);
            Check.That(r.Summary().Contains("commands-resolved-censored-unavailable=4|3|0|1"),"foreign-source receipt rejected even at same Fixed");
            var next=f.C.Step;next.Sequence++;var after=f.B;after.FixedSequence++;
            var stale=f.Build();stale.Step.Sequence++;
            var qualified=LandingPreparationResponse.Measure(f.C,f.B,f.Foot,next,after,f.Foot,stale);
            Check.That(qualified.Valid&&!qualified.Geometry.Valid,"optional decomposition loss leaves response valid");
            r.Reset();Check.That(r.Summary().Contains("group=ALL/resolved=0/actualUp-down-flat=0|0|0")&&!r.Summary().Contains("MAX_REFERENCE_ACTUAL_GAP"),"restart clears all bounded evidence");
        }
        private static void Allocation()
        {
            var f=Fixture();var r=new LandingPreparationResponse();var response=Response(f,.001f);
            for(int i=0;i<100;i++){f.Build();r.Observe(f.C,true,response);}
            long initial=GC.GetAllocatedBytesForCurrentThread();
            for(int i=0;i<2000;i++){var s=f.Build();if(!s.Valid)throw new Exception("lost geometry");r.Observe(f.C,true,response);}
            Check.That(GC.GetAllocatedBytesForCurrentThread()==initial,"pure geometry and totals have no hot-path allocation");
        }
    }
}
