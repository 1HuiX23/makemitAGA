using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
namespace ARL.Contracts
{
    // These production DTOs are assigned by native adapters excluded from this
    // managed project. Exercise their actual consumer contracts, rather than
    // deleting live fields or silencing CS0649 globally.
    internal static class LandingEvidenceContracts
    {
        internal static void Run()
        {
            Check.Run("landing receipts keep source, receiver target and physical parent distinct", Parent);
            Check.Run("landing input clock and optional upstream paths fail independently", Optional);
            Check.Run("landing Knee receipt separates requested, allocated and transported commands", Knee);
        }
        internal static LegLandingReferenceInput Input()
        {
            var source=new SourceStepContext{Epoch=7,Sequence=42,Time=1.2f,Dt=.02f};
            var step=new PhysicsStepContext{Epoch=7,Sequence=81,SourceSequence=42,FixedTime=1.25f,UnscaledTime=1.25f,Dt=.02f};
            var parent=new LandingParentPlacementInput{Prepared=true,Step=step,Source=source,
                RawP=new Vector3(.1f,1f,.2f),ReferenceP=new Vector3(.12f,1.01f,.2f),ActualP=new Vector3(.12f,.99f,.2f),
                LocalAnchor=new Vector3(.15f,0,0),RawR=Quaternion.AngleAxis(30,Vector3.forward),
                ReferenceR=Quaternion.AngleAxis(10,Vector3.forward),ActualR=Quaternion.AngleAxis(-5,Vector3.forward)};
            var goal=Quaternion.AngleAxis(8,Vector3.right);
            var path=new HipGoalPathEvidence{Step=step,Source=source,Fixed=18,Modes=13,Prepared=true,Committed=true,NormalApplied=true,
                RawUpper=Quaternion.AngleAxis(2,Vector3.right),PhysicalParent=parent.ActualR,ControlParent=parent.ReferenceR,
                RawInControl=Quaternion.AngleAxis(3,Vector3.right),AfterSource=Quaternion.AngleAxis(4,Vector3.right),
                AfterGovernor=Quaternion.AngleAxis(5,Vector3.right),AfterComposite=Quaternion.AngleAxis(6,Vector3.right),
                Final=goal,PhysicalUpper=Quaternion.AngleAxis(9,Vector3.right)};
            var reference=new KneeReferenceState{Valid=true,Selected=true,Step=step,Source=source,
                PreviousR=Quaternion.identity,InputR=Quaternion.AngleAxis(20,Vector3.right),R=Quaternion.AngleAxis(19,Vector3.right),
                PreviousW=Vector3.zero,InputW=new Vector3(.1f,0,0),W=new Vector3(.08f,0,0),A=new Vector3(4,0,0),MaxW=3,MaxA=20};
            var knee=new LandingKneeControlInput{Prepared=true,Step=step,Source=source,Fixed=18,
                FeedforwardValid=true,FeedforwardActive=true,FeedforwardClamped=false,IndependentGate=true,
                FlexionClamped=false,SoftClamped=true,ParentTransportEnabled=true,FeedforwardReason="ACTIVE_NORMAL_WINDOW",SoftPhase="BASELINE",
                ErrorAbsDegrees=2,TargetW=.1f,BodyW=.04f,VelocityError=.06f,P=1,D=2,Feedback=3,
                InputFF=8,SelectedFF=8,AllocatedFF=5,FinalFlexion=8,MaxFlexionA=40,
                SoftRawA=12,SoftAppliedA=10,SoftMaxA=10,RelativeA=new Vector3(1,2,3),ParentA=new Vector3(.1f,.2f,.3f),
                TotalA=new Vector3(1.1f,2.2f,3.3f),ReferenceSelected=true,PlannedSourceRecoverySelected=true,FeedforwardStateBits=4,
                Reference=reference,RawSourceA=new Vector3(9,0,0)};
            Vector3 actualHip=parent.ActualP+parent.ActualR*parent.LocalAnchor;
            return new LegLandingReferenceInput{Prepared=true,Step=step,Source=source,Fixed=18,
                UpperGoal=goal,KneeRelativeGoal=reference.R,Upper=path.PhysicalUpper,
                ParentHip=actualHip,UpperHip=actualHip+new Vector3(.001f,-.003f,0),UpperHipToKnee=new Vector3(0,-.44f,0),LowerKneeAnchor=Vector3.zero,
                RawHip=parent.RawP+parent.RawR*parent.LocalAnchor+Vector3.up*.001f,
                RawLower=path.RawUpper*Quaternion.AngleAxis(15,Vector3.right),RawKneeRelative=Quaternion.AngleAxis(15,Vector3.right),
                HipPath=path,Normal=new HipNormalReferenceState{Valid=true,Step=step,InputR=Quaternion.AngleAxis(7,Vector3.right),Applied=true,ReceiverParentReference=true},
                Parent=parent,Knee=knee};
        }
        internal static FootLandingReferenceEvidence.LandingPath Build(LegLandingReferenceInput i)
        {
            var p=FootLandingReferenceEvidence.Build(i.Step,i.Source,i.Fixed,i,new Vector3(.1f,.12f,.2f),new Vector3(.1f,.09f,.2f),
                i.Upper*Quaternion.AngleAxis(20,Vector3.right),new Vector3(0,-.47f,0));
            p.SupportTop=.04f;return p;
        }
        private static string Value(string text,string key)
        {
            int p=text.IndexOf(key+"=",StringComparison.Ordinal);Check.That(p>=0,"receipt field present "+key);p+=key.Length+1;
            int end=text.IndexOf('/',p);return end<0?text.Substring(p):text.Substring(p,end-p);
        }
        private static void Parent()
        {
            var i=Input();var p=Build(i);Check.That(p.Valid&&p.ParentPlacementValid&&p.UpstreamValid&&p.KneeControlValid,"all independent receipts qualified");
            Check.Near((p.ParentReference-p.ParentBoundRaw).y,.01f+.15f*(MathF.Sin(10*MathF.PI/180)-MathF.Sin(30*MathF.PI/180)),1e-6f,"reference pose effect includes parent rotation at offset anchor");
            Check.Near((p.ParentResponseP-p.ParentReference).y,-.02f,1e-6f,"physical origin lag is separate from reference change");
            Check.Near((p.ParentActual-p.ParentResponseP).y,.15f*(MathF.Sin(-5*MathF.PI/180)-MathF.Sin(10*MathF.PI/180)),1e-6f,"physical rotation at offset anchor kept separate");
            Check.Near((p.ParentActual-p.ParentRaw-p.ParentTotal).magnitude,0,1e-6f,"physical parent endpoint matches recorded raw origin debt");
            string text=FootLandingReferenceEvidence.SampleText(p);string[] heights=Value(text,"heightMm(raw-goal-upper-lower-hip-actual)").Split('|');
            Check.Near(float.Parse(heights[5],CultureInfo.InvariantCulture),50,1e-4f,"height relative to support, not absolute world Y");
            Check.That(i.HipPath.Details().Contains("local-source-modes-normal=18|42|13|True"),"goal-path metadata remains live diagnostic content");
        }
        private static void Optional()
        {
            var i=Input();i.Parent.Source.Sequence++;var p=Build(i);
            Check.That(p.Valid&&p.UpstreamValid&&p.KneeControlValid&&!p.ParentPlacementValid,"stale optional parent does not invent a zero or drop endpoint");
            i=Input();i.HipPath.Committed=false;p=Build(i);
            Check.That(p.Valid&&p.ParentPlacementValid&&!p.UpstreamValid,"uncommitted upstream target is missing, not consumed");
            i=Input();i.HipPath.Final=default;p=Build(i);Check.That(!p.UpstreamValid&&p.Valid,"invalid final goal does not masquerade as committed path");
            i=Input();i.Knee.Fixed++;p=Build(i);Check.That(p.Valid&&!p.KneeControlValid&&p.ParentPlacementValid,"different local Fixed cannot borrow Knee receipt");
            i=Input();var step=i.Step;step.Sequence++;p=FootLandingReferenceEvidence.Build(step,i.Source,i.Fixed,i,Vector3.zero,Vector3.zero,Quaternion.identity,-Vector3.up);
            Check.That(!p.Valid,"foreign shared Fixed rejects original endpoint path");
        }
        private static void Knee()
        {
            var i=Input();string text=FootLandingReferenceEvidence.SampleText(Build(i));
            Check.That(Value(text,"kneeFlexionA(P-D-feedback-allocatedFF-final-limit)")=="1|2|3|5|8|40","request and allocated FF are not conflated");
            Check.That(Value(text,"kneeFF(input-selected-reason)")=="8|8|ACTIVE_NORMAL_WINDOW","requested FF is retained separately");
            Check.That(Value(text,"kneeReceiptResidual(feedback-allocation-transport)")=="0|0|(0,0,0)","published command has closed receipt");
            Check.That(Value(text,"kneeFFPolicyV041180(selected-stateBits)")=="True|4"&&Value(text,"kneeClamp(flexion-soft)")=="False-True","gate and clamp provenance retained");
            i.Knee.AllocatedFF=4;text=FootLandingReferenceEvidence.SampleText(Build(i));Check.That(Value(text,"kneeReceiptResidual(feedback-allocation-transport)").StartsWith("0|1|"),"inconsistent allocation remains visible rather than repaired");
            i=Input();i.Knee.FeedforwardValid=false;i.Knee.InputFF=float.NaN;i.Knee.SelectedFF=float.NaN;i.Knee.FeedforwardReason=null;
            var p=Build(i);Check.That(p.KneeControlValid&&Value(FootLandingReferenceEvidence.SampleText(p),"kneeFF(input-selected-reason)")=="NA","missing FF does not discard independent PD receipt");
            i.Knee.FeedforwardValid=true;Check.That(!Build(i).KneeControlValid,"claimed valid FF requires finite values and reason");
        }
    }
}
