using System;
using ARL.Contracts;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private struct ProgressFixture
        {
            internal KneeGroundTarget.Geometry Geometry;
            internal KneeFootGroundInput Input;
            internal Quaternion PreviousFoot, PreviousGoal, NativeLower;
            internal Vector3 PreviousW, NativeAnkle;
            internal float Dt;
        }
        private static float Next(Random r, float lo, float hi) => lo + (hi - lo) * (float)r.NextDouble();
        private static KneeReferenceState ReferenceAt(PhysicsStepContext step, Quaternion rotation, Vector3 w = default)
        {
            var source = Source(step.SourceSequence, step.UnscaledTime);
            return new KneeReferenceState { Step = step, Source = source, Valid = true, Selected = true,
                R = rotation, PreviousR = rotation, InputR = rotation, W = w, PreviousW = w,
                MaxW = 40f / 9f, MaxA = 20f };
        }
        private static ProgressFixture RandomProgress(Random random, int hz)
        {
            float dt = 1f / hz, rawAngle = Next(random, -22f, 22f), currentAngle = rawAngle + Next(random, -12f, 12f);
            var raw = Quaternion.AngleAxis(rawAngle, Vector3.right);
            var current = Quaternion.AngleAxis(currentAngle, Vector3.right);
            var foot = Quaternion.AngleAxis(Next(random, -15f, 15f), Vector3.right);
            var g = new KneeGroundTarget.Geometry { Hip = new Vector3(0f, Next(random, .995f, 1.045f), 0f),
                Upper = Quaternion.AngleAxis(Next(random, -8f, 8f), Vector3.right),
                UpperSpan = new Vector3(0f, -.45f, 0f), LowerSpan = new Vector3(0f, -.45f, 0f),
                FootRelative = foot, FootBindRelative = Quaternion.identity,
                Center = new Vector3(0f, -.1f, .04f), Half = new Vector3(.05f, .025f, .1f),
                FloorCenter = new Vector3(0f, -.05f, 0f), FloorHalf = new Vector3(3f, .05f, 3f) };
            var completed = Step(100, 200, 1f);
            var input = new KneeFootGroundInput { Source = Source(201, 1f + dt, dt), CompletedStep = completed,
                Enabled = true, Apply = true, AllowExtension = true, ParentId = 11, UpperId = 12, LowerId = 13,
                Hip = g.Hip, Upper = g.Upper, UpperSpan = g.UpperSpan, Axis = Vector3.right, RawKnee = raw,
                CompletedKneeReference = ReferenceAt(completed, current) };
            var joint = KneeGroundTarget.Evaluate(g, raw, input.Axis, true);
            return new ProgressFixture { Geometry = g, Input = input, Dt = input.Source.Time - 1f,
                PreviousFoot = FootRelativeReference.Advance(foot, Vector3.right, Next(random, -.1f, .1f)),
                PreviousGoal = FootRelativeReference.Advance(KneeGroundTarget.SelectedFoot(g, joint.Foot), Vector3.right, Next(random, -.025f, .025f)),
                PreviousW = new Vector3(Next(random, -1.5f, 1.5f), Next(random, -.15f, .15f), 0f),
                NativeLower = g.Upper * current, NativeAnkle = KneeGroundTarget.Ankle(g, current) };
        }
        private static FootAnkleProxyFoundation ProgressPort(ProgressFixture fixture, bool selected)
        {
            var f = new FootAnkleProxyFoundation(fixture.Geometry, fixture.Input);
            f._groundReference.SelectKneeProgress(selected);
            f._relativeReference.SelectGroundTrajectory(true);
            var previous = fixture.Input.CompletedKneeReference.Source;
            f._relativeReference.Seed(previous, fixture.PreviousFoot, fixture.PreviousW);
            f._groundReference.SeedInput(previous, fixture.PreviousGoal, Vector3.zero);
            f._lowerBody.rotation = fixture.NativeLower;
            f._lowerBody.position = fixture.NativeAnkle - fixture.NativeLower * f._lowerAnkleAnchorLocal;
            return f;
        }
        private void PublishProgress(ProgressFixture f, bool selected)
        {
            var input = f.Input; input.PreferKneeProgress = selected;
            ObserveFootReference(input.Source, f.Geometry.FootRelative, Vector3.zero, f.Dt, input);
        }
        private static ProgressFixture ProgressExample(int hz = 60)
        {
            var rng = new Random(41350 + hz);
            for (int i = 0; i < 12000; i++)
            {
                var f = RandomProgress(rng, hz); var port = ProgressPort(f, true); port.PublishProgress(f, true);
                if (port._groundReference.CurrentKneeProgress.Applied && port._groundReference.Finite) return f;
            }
            throw new InvalidOperationException("No improving production waypoint fixture at " + hz + " Hz");
        }
        private static float CornerMinimum(KneeGroundTarget.Geometry g, Vector3 ankle, Quaternion lower, Quaternion foot)
        {
            float min = float.PositiveInfinity;
            for (int x = -1; x <= 1; x += 2) for (int y = -1; y <= 1; y += 2) for (int z = -1; z <= 1; z += 2)
                min = MathF.Min(min, (ankle + lower * foot * (g.Center + new Vector3(g.Half.x*x,g.Half.y*y,g.Half.z*z))).y - g.FloorCenter.y - g.FloorHalf.y);
            return min;
        }
        internal static void RunKneeProgressContracts()
        {
            Check.Run("real Foot publication preserves completed preparation provenance without granting control authority", PreparationProvenance);
            Check.Run("v135 production waypoint improves both checked domains inside original budgets", ProgressPublication);
            Check.Run("moving native geometry retains exact fallback and rejects harmful final previews", ProgressMovingNative);
            Check.Run("completed Knee identity is required and its absence keeps exact old Foot output", ProgressIdentity);
            Check.Run("progress goal cannot buy a worse native-Lower goal or final step", ProgressNativeGuards);
            Check.Run("progress previews do not publish and actual publication rechecks RWA", ProgressPreviewIntegrity);
            Check.Run("progress held Render trace reset and stop preserve independent ownership", ProgressLifecycle);
            Check.Run("progress production references remain continuous at 20 40 60 Hz", ProgressSequences);
            Check.Run("progress candidate hot path creates no managed garbage", ProgressAllocation);
        }
        private static void PreparationProvenance()
        {
            var fixture = ProgressExample(); var foot = ProgressPort(fixture, true); foot.PublishProgress(fixture, true);
            var packet = foot._kneeFootGroundPacket;
            Check.That(packet.PreparationSupport.Equals(foot._previousReferenceSupport), "complete source-phase witness copied by actual Render publisher");
            var step = Step(101, fixture.Input.Source.Sequence, fixture.Input.Source.Time);
            bool before = packet.Matches(step, fixture.Input.Source, 11, 12, 13, 21);
            packet.PreparationSupport = default;
            Check.That(before && packet.Matches(step, fixture.Input.Source, 11, 12, 13, 21), "diagnostic provenance is never a new control or contact gate");
        }

        private static void ProgressPublication()
        {
            int applied = 0;
            foreach (int hz in new[] {20,40,60})
            {
                var fixture = ProgressExample(hz); var a = ProgressPort(fixture, false); var b = ProgressPort(fixture, true);
                a.PublishProgress(fixture, false); b.PublishProgress(fixture, true);
                var pa = a._kneeFootGroundPacket; var pb = b._kneeFootGroundPacket;
                var result = b._groundReference.CurrentKneeProgress;
                Check.That(a._groundReference.CurrentKneeProgress.Better && !a._groundReference.CurrentKneeProgress.Applied && result.Applied, "same candidate evaluated in A, adopted only in B");
                Check.That(a._groundReference.Finite && b._groundReference.Finite && pa.Published && pb.Published, "both owners publish once");
                Check.That(pa.Sample.Goal.Equals(pb.Sample.Goal) && pa.Sample.Foot.Equals(pb.Sample.Foot), "ideal Knee and ideal Foot endpoint remain intact");
                Check.That(pb.Sample.KneeProgress.Applied && pb.Sample.PlannedFoot.Equals(b._relativeReference.Current.Rotation), "Knee consumes actual selected Foot publication");
                var step = Step(101, fixture.Input.Source.Sequence, fixture.Input.Source.Time);
                Check.That(b.Read(step, fixture.Input.Source, out _), "selected packet passes real consumer identity checks");
                var old = a._relativeReference.Current; var next = b._relativeReference.Current; var g = fixture.Geometry;
                var k = fixture.Input.CompletedKneeReference.R; var ankle = KneeGroundTarget.Ankle(g,k); var lower = g.Upper*k;
                float h0=CornerMinimum(g,ankle,lower,old.Rotation),h1=CornerMinimum(g,ankle,lower,next.Rotation);
                Check.That(h1-h0 >= .0000999f, "independent full Box current-Knee step gain");
                Check.That(CornerMinimum(g,fixture.NativeAnkle,fixture.NativeLower,next.Rotation) >= CornerMinimum(g,fixture.NativeAnkle,fixture.NativeLower,old.Rotation)-1e-7f, "independent native step does not worsen");
                Check.That(next.W.magnitude <= MathF.Max(3.75f,fixture.PreviousW.magnitude)+1e-4f && next.A.magnitude <= 20.002f, "original Foot W/A balls");
                Check.Near((next.A-(next.W-fixture.PreviousW)/fixture.Dt).magnitude,0f,1e-6f,"A derives from final W and actual Render dt");
                Check.That(FootRelativeReference.Log(next.Rotation*Quaternion.Inverse(FootRelativeReference.Advance(fixture.PreviousFoot,next.W,fixture.Dt))).magnitude < 1e-6f,"R integrates final W exactly once");
                Check.That(FootGroundReference.GoalWithinBounds(lower,g.FootRelative,b._groundReference.Current.Goal,g.FootBindRelative) &&
                    FootGroundReference.GoalWithinBounds(fixture.NativeLower,g.FootRelative,b._groundReference.Current.Goal,g.FootBindRelative),"waypoint remains inside both geometric Foot bounds");
                applied++;
            }
            Console.WriteLine("v135 real production waypoint improving rates="+applied);
        }
        private static void ProgressMovingNative()
        {
            var rng=new Random(13507); int improved=0,nativeStepRejected=0, fallback=0;
            foreach(int hz in new[]{20,40,60}) for(int n=0;n<600;n++) {
                var f=RandomProgress(rng,hz);
                f.NativeLower=Quaternion.AngleAxis(Next(rng,-15f,15f),Vector3.right)*f.NativeLower;
                f.NativeAnkle.y+=Next(rng,-.004f,.004f);
                var a=ProgressPort(f,false);var b=ProgressPort(f,true);
                a.PublishProgress(f,false);b.PublishProgress(f,true);
                var r=b._groundReference.CurrentKneeProgress;
                Check.That(a._relativeReference.Finite && b._relativeReference.Finite && a._groundReference.Finite && b._groundReference.Finite,"native drift never corrupts either owner");
                if(r.Reason==FootKneeProgressTarget.Status.NativeStepRegression)nativeStepRejected++;
                if(r.Applied) {
                    improved++;
                    Check.That(r.StepCandidateMin-r.StepBaselineMin >= .0001f && r.NativeStepCandidateMin>=r.NativeStepBaselineMin,"both step-domain acceptance conditions hold");
                } else {
                    fallback++;
                    Check.That(a._relativeReference.Current.Rotation.Equals(b._relativeReference.Current.Rotation) &&
                        a._relativeReference.Current.W.Equals(b._relativeReference.Current.W) &&
                        a._relativeReference.Current.A.Equals(b._relativeReference.Current.A),"rejected preview preserves exact common baseline");
                }
            }
            Console.WriteLine("v135 moving-native improved="+improved+" nativeStepRejected="+nativeStepRejected+" fallback="+fallback);
            Check.That(improved>0 && fallback>0 && nativeStepRejected>0,"independent native poses exercise adoption, exact fallback and native final-step refusal");
        }

        private static void ProgressIdentity()
        {
            var fixture = ProgressExample();
            for (int fault = 0; fault < 13; fault++)
            {
                var bad = fixture; var reference = bad.Input.CompletedKneeReference;
                switch (fault) {
                    case 0: reference.Valid=false;break; case 1: reference.Selected=false;break;
                    case 2: reference.Stale=true;break; case 3: reference.Step.Sequence--;break;
                    case 4: reference.Source.Sequence--;break; case 5: reference.Source.Epoch++;break;
                    case 6: reference.Source.Time-=.2f;break; case 7: reference.R=default;break;
                    case 8: reference.W.x=float.NaN;break; case 9: reference.A.y=float.PositiveInfinity;break;
                    case 10: reference.MaxA=0f;break; case 11: reference.Step.UnscaledTime-=.001f;break;
                    case 12: reference.Source.Time=bad.Input.Source.Time+.01f;break;
                }
                bad.Input.CompletedKneeReference=reference;
                var b=ProgressPort(fixture,true);var control=ProgressPort(fixture,false);
                b.PublishProgress(bad,true);control.PublishProgress(fixture,false);
                Check.That(!b._groundReference.CurrentKneeProgress.Applied && b._groundReference.Finite,"optional invalid reference falls back "+fault);
                Check.That(b._relativeReference.Current.Rotation.Equals(control._relativeReference.Current.Rotation) &&
                    b._relativeReference.Current.W.Equals(control._relativeReference.Current.W) &&
                    b._relativeReference.Current.A.Equals(control._relativeReference.Current.A),"fallback physical RWA equals common baseline "+fault);
            }
            var mismatched=ProgressPort(fixture,true);mismatched.PublishProgress(fixture,false);
            Check.That(!mismatched._kneeFootGroundPacket.Published && !mismatched._groundReference.CurrentKneeProgress.Applied,"producer selector and packet policy cannot disagree");
        }
        private static void ProgressNativeGuards()
        {
            var fixture=ProgressExample();var basePort=ProgressPort(fixture,false);basePort.PublishProgress(fixture,false);
            var packet=basePort._kneeFootGroundPacket; var g=packet.Geometry;
            var native = new FootGroundTrajectory.Input { Source=packet.Input.Source,GeometryStep=packet.Input.CompletedStep,
                ParentId=11,UpperId=12,LowerId=13,SupportId=21,Geometry=g,KneeGoal=packet.Sample.Goal,
                NativeAnkle=fixture.NativeAnkle,NativeLower=fixture.NativeLower };
            int refused=0;
            for(int angle=-25;angle<=25;angle++) {
                var shifted=native;shifted.NativeLower=Quaternion.AngleAxis(angle,Vector3.right)*native.NativeLower;
                var r=FootKneeProgressTarget.Evaluate(packet,shifted,packet.Sample.Foot,out _);
                if(r.Reason==FootKneeProgressTarget.Status.NativeGoalRegression)refused++;
            }
            Check.That(refused>0,"actual geometry counterexamples reject a native-worsening goal");
            var a = basePort._relativeReference.Current; var candidate=a;
            var result=FootKneeProgressTarget.Evaluate(packet,native,packet.Sample.Foot,out _);
            FootKneeProgressTarget.AssessPreview(ref result,packet,native,a,candidate);
            Check.That(!result.Better && result.Reason==FootKneeProgressTarget.Status.NoStepGain,"an attractive endpoint alone cannot authorize an unchanged step");
        }
        private static void ProgressPreviewIntegrity()
        {
            var fixture=ProgressExample();
            for(int fault=0;fault<5;fault++) {
                var b=ProgressPort(fixture,true);b.PublishProgress(fixture,true);
                var corrupted=b._relativeReference.Current;
                if(fault==0)corrupted.W.x+=.001f;
                if(fault==1)corrupted.A.y+=.001f;
                if(fault==2)corrupted.Rotation=Quaternion.AngleAxis(.01f,Vector3.up)*corrupted.Rotation;
                if(fault==3)corrupted.Dt+=.0001f;
                if(fault==4)corrupted.Source.Sequence++;
                b._groundReference.ValidateKneeProgressPublication(fixture.Input.Source,corrupted);
                Check.That(!b._groundReference.Finite && b._groundReference.Issue=="KNEE_PROGRESS_FINAL_PUBLICATION_MISMATCH","final publication cannot differ from preview "+fault);
                var step=Step(101,fixture.Input.Source.Sequence,fixture.Input.Source.Time);
                Check.That(!b.Read(step,fixture.Input.Source,out _),"invalid publication cannot authorize a Knee packet "+fault);
            }
        }
        private static void ProgressLifecycle()
        {
            var fixture=ProgressExample();var loud=ProgressPort(fixture,true);var quiet=ProgressPort(fixture,true);quiet._traceEnabled=false;
            loud.PublishProgress(fixture,true);quiet.PublishProgress(fixture,true);
            Check.That(loud._relativeReference.Current.Equals(quiet._relativeReference.Current),"trace does not choose the waypoint");
            var packet=loud._kneeFootGroundPacket;var before=loud._relativeReference.Current;var progress=loud._groundReference.CurrentKneeProgress;
            string summary=loud._groundReference.GetDetails();loud._lowerBody.position.y+=1f;loud.PublishProgress(fixture,true);
            Check.That(loud._kneeFootGroundPacket.Equals(packet) && loud._relativeReference.Current.Equals(before) && loud._groundReference.GetDetails()==summary,"held source integrates and counts nothing despite changed live geometry");
            loud._groundReference.ResetTrace();loud._relativeReference.ResetTrace();
            Check.That(loud._groundReference.KneeProgressSelected && loud._groundReference.CurrentKneeProgress.Equals(progress) && loud._relativeReference.Current.Equals(before),"trace reset preserves live input and publication contract");
            loud._groundReference.ValidateKneeProgressPublication(fixture.Input.Source,before);
            Check.That(loud._groundReference.Finite,"reset does not erase expected final publication");
            loud._groundReference.DisablePreserveTrace();loud.ClearKneeFootGroundPacket();
            Check.That(!loud._groundReference.KneeProgressSelected && !loud.Read(Step(101,201,fixture.Input.Source.Time),fixture.Input.Source,out _),"stop removes selection and packet");
            loud._groundReference.Select(true,true,true);
            Check.That(!loud._groundReference.KneeProgressSelected && !loud._groundReference.CurrentKneeProgress.Applied,"restart requires an explicit new selector");
        }
        private static void ProgressSequences()
        {
            int adopted=0;
            foreach(int hz in new[]{20,40,60}) foreach(bool selected in new[]{false,true}) {
                var f=ProgressExample(hz);var port=ProgressPort(f,selected);var knee=new KneeRelativeReference();
                knee.Reset(true,f.Input.CompletedKneeReference.R,Vector3.zero);
                var source=Source(200,1f);var initial=Step(100,200,1f);
                Check.That(knee.Step(false,initial,source,f.Input.CompletedKneeReference.R,Vector3.zero,0f,.1f,40f/9f,20f,5f),"seed a real completed Knee reference");
                int lastRender=0;
                for(int n=1;n<=180;n++) {
                    int render=n*hz/60;float time=1f+n/60f;
                    if(render>lastRender) {
                        source=Source(200+render,time,time-source.Time);f.Input.Source=source;f.Dt=source.Dt;
                        f.Input.CompletedStep=knee.Current.Step;f.Input.CompletedKneeReference=knee.Current;
                        f.Input.RawKnee=FootRelativeReference.Advance(f.Input.RawKnee,Vector3.right, .0003f*MathF.Sin(n*.13f));
                        port._report.RuntimeContext=port._previousReferenceSupport.Step=knee.Current.Step;
                        // Inject observations; this is a reference-contract sequence, NOT a native integration.
                        port._lowerBody.rotation=f.Geometry.Upper*knee.Current.R;
                        port._lowerBody.position=f.Geometry.Hip+f.Geometry.Upper*f.Geometry.UpperSpan;
                        var old=port._relativeReference.Current;port.PublishProgress(f,selected);var now=port._relativeReference.Current;
                        Check.That(port._groundReference.Finite && port._relativeReference.Finite,"real Foot owners stay finite");
                        Check.That(now.A.magnitude<=20.002f && now.W.magnitude<=MathF.Max(3.75f,old.W.magnitude)+1e-4f,"recursive Foot budgets");
                        Check.Near((now.A-(now.W-old.W)/source.Dt).magnitude,0f,1e-5f,"actual Render derivative");
                        if(port._groundReference.CurrentKneeProgress.Applied)adopted++;
                        if(n>=90&&n<90+60/hz){port._groundReference.ResetTrace();port._relativeReference.ResetTrace();}
                        lastRender=render;
                    }
                    var step=Step(100+n,source.Sequence,time);bool read=port.Read(step,source,out var packet);
                    var goal=read&&packet.Sample.Applied?packet.Sample.Goal:f.Input.RawKnee;
                    Check.That(knee.Step(true,step,source,goal,Vector3.zero,time-source.Time,.1f,40f/9f,20f,5f,read?packet:default,true),"real Knee consumes the published Foot in its own Fixed clock");
                    Check.That(knee.Current.A.magnitude<=20.002f,"original Knee acceleration bound");
                }
            }
            Console.WriteLine("v135 recursive actual adopted inputs="+adopted);
            Check.That(adopted>0,"recursive sequence includes actual new-policy execution");
        }
        private static void ProgressAllocation()
        {
            var f=ProgressExample();var port=ProgressPort(f,false);port.PublishProgress(f,false);var packet=port._kneeFootGroundPacket;
            var planner=ProgressPort(f,true);var input=new FootGroundTrajectory.Input {Source=f.Input.Source,GeometryStep=f.Input.CompletedStep,
                ParentId=11,UpperId=12,LowerId=13,SupportId=21,Geometry=f.Geometry,KneeGoal=packet.Sample.Goal,NativeAnkle=f.NativeAnkle,NativeLower=f.NativeLower};
            packet.Input.PreferKneeProgress=true;
            for(int i=0;i<50;i++){var c=packet.Sample.Foot;var g=input;planner._groundReference.ChooseKneeProgress(false,f.Input.Source,f.Geometry.FootRelative,Vector3.zero,f.Dt,packet,planner._relativeReference,ref c,ref g);}
            long start=GC.GetAllocatedBytesForCurrentThread();
            for(int i=0;i<500;i++){var c=packet.Sample.Foot;var g=input;planner._groundReference.ChooseKneeProgress(false,f.Input.Source,f.Geometry.FootRelative,Vector3.zero,f.Dt,packet,planner._relativeReference,ref c,ref g);}
            Check.That(GC.GetAllocatedBytesForCurrentThread()==start,"waypoint selection and both previews allocate zero");
        }
    }
}
