using System;
using ARL.Contracts;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private static FootAnkleProxyFoundation FootTrajectoryFixture(FootGroundTrajectoryContracts.Fixture fixture, bool selected)
        {
            var f = Create(); var input = fixture.Input; var g = input.Geometry;
            f._report.RuntimeContext = input.GeometryStep;
            f._previousReferenceSupport.Step = input.GeometryStep;
            f._bindSourceRelative = g.FootBindRelative;
            f._lowerBody.rotation = input.NativeLower;
            f._lowerBody.position = input.NativeAnkle - input.NativeLower * f._lowerAnkleAnchorLocal;
            f._collider.center = g.Center; f._collider.size = g.Half * 2f;
            var previousSource = input.Source; previousSource.Sequence--; previousSource.Time -= fixture.Dt;
            f._relativeReference.Seed(previousSource, fixture.Previous, fixture.W);
            f._relativeReference.SelectGroundTrajectory(selected);
            f._groundReference.SeedInput(previousSource,
                FootRelativeReference.Advance(fixture.Goal, -fixture.NominalW, fixture.Dt), Vector3.zero);
            return f;
        }
        private void PublishTrajectoryFixture(FootGroundTrajectoryContracts.Fixture fixture)
        {
            var g = fixture.Input.Geometry;
            var input = Input(fixture.Input.Source); input.AllowExtension = true;
            input.Hip = g.Hip; input.Upper = g.Upper; input.UpperSpan = g.UpperSpan;
            input.RawKnee = fixture.Input.KneeGoal;
            ObserveFootReference(fixture.Input.Source, g.FootRelative, Vector3.zero, fixture.Dt, input);
        }
        internal static void RunFootTrajectoryPublicationContracts()
        {
            Check.Run("real Foot producer applies B before immutable Knee publication at 20/40/60 Hz", () =>
            {
                int improved = 0; var rng = new Random(13402);
                foreach (int hz in new[] { 20, 40, 60 })
                {
                    bool found = false;
                    for (int i = 0; i < 1000 && !found; i++)
                    {
                        var fixture = FootGroundTrajectoryContracts.RandomFixture(rng, hz);
                        fixture.Dt = fixture.Input.Source.Time - 1f;
                        var g = fixture.Input.Geometry;
                        var candidate = KneeGroundTarget.Evaluate(g, fixture.Input.KneeGoal, Vector3.right, true);
                        if (!candidate.Valid || !candidate.Foot.Feasible) continue;
                        fixture.Goal = KneeGroundTarget.SelectedFoot(g, candidate.Foot);
                        var a = FootTrajectoryFixture(fixture, false); var b = FootTrajectoryFixture(fixture, true);
                        a.PublishTrajectoryFixture(fixture); b.PublishTrajectoryFixture(fixture);
                        Check.That(a._relativeReference.Finite && b._relativeReference.Finite, "real references remain finite");
                        var sa = a._relativeReference.Current; var sb = b._relativeReference.Current;
                        if (!sb.GroundTrajectory.Applied) continue;
                        found = true; improved++;
                        Check.That(sa.GroundTrajectory.Better && !sa.GroundTrajectory.Applied && sb.GroundTrajectory.Applied,
                            "both arms evaluate, only B executes");
                        var step = Step(101, fixture.Input.Source.Sequence, fixture.Input.Source.Time);
                        bool readA = a.Read(step, fixture.Input.Source, out var pa);
                        bool readB = b.Read(step, fixture.Input.Source, out var pb);
                        Check.That(readA && readB, "both real packets valid");
                        Check.That(pa.Sample.AllowExtension && pb.Sample.AllowExtension && pa.Sample.Goal.Equals(pb.Sample.Goal), "same bounded-extension input in both arms");
                        Check.That(pb.Sample.PlannedFoot.Equals(sb.Rotation) && pb.Sample.FootTrajectory.Applied, "Knee reads selected Foot R, not discarded baseline");
                        Check.That(!sa.Rotation.Equals(sb.Rotation) && sb.A.magnitude <= 20.0002f, "bounded choice reaches real publication");
                        Check.That(FootRelativeReference.Log(sb.Rotation * Quaternion.Inverse(FootRelativeReference.Advance(fixture.Previous, sb.W, fixture.Dt))).magnitude < 1e-6f,
                            "published R closes with selected W");
                        b._lowerBody.position.y += .5f; b.PublishTrajectoryFixture(fixture);
                        Check.That(b._kneeFootGroundPacket.Equals(pb) && b._relativeReference.Current.Equals(sb), "held Render does not recapture native geometry");
                        b._relativeReference.ResetTrace(); b._groundReference.ResetTrace();
                        Check.That(b.Read(step, fixture.Input.Source, out var held) && held.Equals(pb), "trace reset cannot erase immutable pair");
                        b._relativeReference.DisableGroundTrajectoryPreserveTrace(); b.ClearKneeFootGroundPacket();
                        Check.That(!b._relativeReference.GroundTrajectorySelected && !b.Read(step, fixture.Input.Source, out _), "stop removes live choice and pair");
                    }
                    Check.That(found, "actual production improvement at Hz=" + hz);
                }
                Console.WriteLine("actual Foot publication improved rates=" + improved);
            });
        }
    }
}
