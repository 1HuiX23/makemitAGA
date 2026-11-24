using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;
using Status = MakemitAGA.ActiveRagdollLab.Physics.SwingKneeCandidate.Status;

namespace ARL.Contracts
{
    internal static class SwingKneeContracts
    {
        internal static void Run()
        {
            Check.Run("v145 bounded knee comparison closes the measured pose and independently raises both geometries", Geometry);
        }
        internal static Audit.Sample Sample(int n,float dt,float t)
        {
            var source = new SourceStepContext { Epoch=2, Sequence=n, Time=t, Dt=dt };
            return new Audit.Sample { Step=new PhysicsStepContext { Epoch=2, Sequence=n, SourceSequence=n, FixedTime=t, UnscaledTime=t, Dt=dt },
                Fixed=n, Source=source, SupportId=3, FloorHalf=new Vector3(2,.1f,2), Gait=Audit.Phase.Swing, SourceMotion=Audit.SourceKind.Motion,
                SourcePhase="TURN", Identity=true, FivePoint=true, GeometryValid=true, MotionValid=true, FullFootprint=true, BoxReferencesValid=true,
                RawMin=-.01f+t*.08f, Min=-.004f-t*.10f, SourceAge=.01f, AnchorGap=.002f, KneeHeightsValid=true, AnkleHeightsValid=true,
                KneeHeights=new Vector3(1,1,.996f-t*.06f), AnkleHeights=new Vector3(.1f,.096f-t*.08f,.095f-t*.08f) };
        }
        private static float Minimum(Vector3 ankle,Quaternion q,KneeGroundTarget.Geometry g)
        {
            float min=float.PositiveInfinity;
            for(int x=-1;x<=1;x+=2)for(int y=-1;y<=1;y+=2)for(int z=-1;z<=1;z+=2)
                min=Mathf.Min(min,(ankle+q*(g.Center+new Vector3(x*g.Half.x,y*g.Half.y,z*g.Half.z))).y-g.FloorCenter.y-g.FloorHalf.y);
            return min;
        }
        internal static SwingKneeCandidate.Pose FixturePose(LandingPreparationFixtures.Fixture f)
        {
            var g=f.Packet.Geometry; g.Upper=f.Hip.Rotation;
            var q=g.Upper*Quaternion.AngleAxis(2f,Vector3.forward); var lower=q*f.Knee.R*Quaternion.AngleAxis(1.5f,Vector3.right);
            var actualFoot=f.Packet.Sample.PlannedFoot*Quaternion.AngleAxis(-.8f,Vector3.forward);
            var knee=g.Hip+q*g.UpperSpan+new Vector3(.001f,-.004f,-.002f);
            var ankle=knee+lower*g.LowerSpan; var gap=new Vector3(.001f,.002f,.0008f);
            return new SwingKneeCandidate.Pose { Reference=g, PhysicalKnee=knee, AnkleGap=gap, Axis=f.Packet.Input.Axis,
                AllowExtension=true, HipW=f.Hip.W, PhysicalUpper=q, PhysicalLower=lower,
                PhysicalFootRelative=actualFoot, SelectedFoot=f.Packet.Sample.PlannedFoot,
                ObservedBox=Minimum(ankle+gap,lower*actualFoot,g), LowerAnkleY=ankle.y };
        }
        private static void Geometry()
        {
            var rand=new Random(145); int favorable=0,checkedGates=0;
            for(int i=0;i<500;i++)
            {
                var f=LandingPreparationFixtures.Make(rand);var k=f.Knee;var p=FixturePose(f);var g=p.Reference;
                var r=SwingKneeCandidate.Evaluate(p,k,k.Step,k.Source);
                if (r.Reason == Status.NoLegal && r.WithinGeometry > 0)
                    Check.That(r.LaneIndex > 0 && r.Detail().Contains("candidate=CLOSEST_REJECTED") &&
                        (Mathf.Min(r.ReferenceGain,Mathf.Min(r.PhysicalGain,r.AnkleGain))<KneeGroundTarget.MinGain ||
                        r.CoastGain<0f || r.PhysicalCoastGain<0f), "a near miss retains its measured margin");
                if(r.Reason!=Status.Favorable)continue;
                favorable++; float dt=k.Step.Dt;
                var lane=LandingPreparationMath.MakeLane(k.PreviousR,k.PreviousW,k.InputR,k.InputW,k.R,k.W,dt,k.MaxW,k.MaxA,p.Axis,true);
                Check.That(LandingPreparationMath.TryLane(lane,g,r.LaneIndex,out var cand,out var w),"uses existing legal R/W lane");
                Check.That(r.CandidateW<=r.LimitW+.0001f && r.CandidateA<=r.LimitA+.0002f,"actual sampled angular budgets");
                var referenceFoot=p.SelectedFoot; var lower=g.Upper*cand;
                var ankle=g.Hip+g.Upper*g.UpperSpan+lower*g.LowerSpan;
                var oldLower=g.Upper*k.R;var oldAnkle=g.Hip+g.Upper*g.UpperSpan+oldLower*g.LowerSpan;
                float reference=Minimum(ankle,lower*referenceFoot,g)-Minimum(oldAnkle,oldLower*referenceFoot,g);
                var delta=p.PhysicalUpper*cand*Quaternion.Inverse(k.R)*Quaternion.Inverse(p.PhysicalUpper);
                var actualLower=delta*p.PhysicalLower;var actualAnkle=p.PhysicalKnee+actualLower*g.LowerSpan;
                float gain=Minimum(actualAnkle+p.AnkleGap,actualLower*p.PhysicalFootRelative,g)-p.ObservedBox;
                Check.Near(reference,r.ReferenceGain,.000002f,"eight reference corners");Check.Near(gain,r.PhysicalGain,.000002f,"eight measured corners");
                Check.Near(actualAnkle.y-p.LowerAnkleY,r.AnkleGain,.000002f,"preserved knee and ankle gaps");
                var rc=FootRelativeReference.Advance(cand,w,dt);var bc=FootRelativeReference.Advance(k.R,k.W,dt);
                var uc=FootRelativeReference.Advance(g.Upper,p.HipW,dt);
                float RefC(Quaternion q) => Minimum(g.Hip+uc*g.UpperSpan+uc*q*g.LowerSpan,uc*q*referenceFoot,g);
                float BodyC(Quaternion q){var l=p.PhysicalUpper*q*Quaternion.Inverse(k.R)*Quaternion.Inverse(p.PhysicalUpper)*p.PhysicalLower;
                    return Minimum(p.PhysicalKnee+l*g.LowerSpan+p.AnkleGap,l*p.PhysicalFootRelative,g);}
                Check.Near(RefC(rc)-RefC(bc),r.CoastGain,.000002f,"reference coast independently reconstructed");
                Check.Near(BodyC(rc)-BodyC(bc),r.PhysicalCoastGain,.000002f,"physical substitution coast independently reconstructed");
                Check.That(r.PhysicalGain>=.0001f && r.ReferenceGain>=.0001f && r.AnkleGain>=.0001f && r.CoastGain>=0f && r.PhysicalCoastGain>=0f,"minimum useful gain");
                if(checkedGates++==0){var bad=p;bad.ObservedBox+=.001f;Check.That(SwingKneeCandidate.Evaluate(bad,k,k.Step,k.Source).Geometry==SwingKneeCandidate.GeometryIssue.MeasuredClosure,"reject wrong floor or corner");
                    bad=p;bad.LowerAnkleY+=.001f;Check.That(SwingKneeCandidate.Evaluate(bad,k,k.Step,k.Source).Geometry==SwingKneeCandidate.GeometryIssue.MeasuredClosure,"reject wrong ankle");
                    bad=p;bad.Axis=Vector3.zero;Check.That(SwingKneeCandidate.Evaluate(bad,k,k.Step,k.Source).Reason==Status.Budget,"missing axis");
                    var old=k;old.Stale=true;Check.That(SwingKneeCandidate.Evaluate(p,old,k.Step,k.Source).Reason==Status.Protected,"stale");old=k;old.GroundTrajectory.Applied=true;
                    Check.That(SwingKneeCandidate.Evaluate(p,old,k.Step,k.Source).Reason==Status.Protected,"previous correction already applied");
                    old=k;old.Step.Sequence--;Check.That(SwingKneeCandidate.Evaluate(p,old,k.Step,k.Source).Reason==Status.Unavailable,"wrong step");
                    bad=p;bad.Reference.FloorHalf.x=.0001f;Check.That(SwingKneeCandidate.Evaluate(bad,k,k.Step,k.Source).Reason==Status.Geometry,"footprint guard");}
            }
            Console.WriteLine("  v145 synthetic distinct-pose favorable="+favorable+"/500"); Check.That(favorable>10,"exercised real feasible knee directions");
        }
    }
}
