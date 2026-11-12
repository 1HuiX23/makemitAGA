using System;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class KneeGroundContracts
    {
        internal static void Run()
        {
            Check.Run("full Box support function agrees with all eight corners", Corners);
            Check.Run("joint Knee Foot candidate clears full Box with separate bounded rotations", Candidate);
            Check.Run("Knee geometry invalid, clear and outside fallback", Fallback);
            Check.Run("joint Foot compensation unlocks a Knee candidate rejected by held Foot geometry", CoupledCounterexample);
            Check.Run("Foot-only clearance never spends unnecessary Knee correction", FootOnly);
            Check.Run("joint candidates respect independent angular bounds over varied chain geometry", BoundedPairs);
        }
        private static KneeGroundTarget.Geometry Geometry() => new KneeGroundTarget.Geometry
        {
            Hip = new Vector3(0f, .9f, 0f), Upper = Quaternion.AngleAxis(30f, Vector3.right),
            UpperSpan = new Vector3(0f, -.45f, 0f), LowerSpan = new Vector3(0f, -.45f, 0f),
            FootRelative = Quaternion.AngleAxis(30f, Vector3.right),
            FootBindRelative = Quaternion.AngleAxis(30f, Vector3.right),
            Center = new Vector3(0f, -.1f, 0f), Half = new Vector3(.05f, .025f, .08f),
            FloorCenter = new Vector3(0f, -.05f, 0f), FloorHalf = new Vector3(3f, .05f, 3f)
        };
        private static void Corners()
        {
            var random = new Random(4125);
            for (int i = 0; i < 200; i++)
            {
                var q = Quaternion.AngleAxis((float)random.NextDouble() * 360f,
                    new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()));
                var ankle = new Vector3(.1f, .2f, .3f); var center = new Vector3(.03f, -.12f, .02f);
                var half = new Vector3(.05f, .025f, .12f); float min = float.PositiveInfinity;
                for (int x = -1; x <= 1; x += 2)
                for (int y = -1; y <= 1; y += 2)
                for (int z = -1; z <= 1; z += 2)
                    min = MathF.Min(min, (ankle + q * (center + new Vector3(x * half.x, y * half.y, z * half.z))).y);
                Check.Near(FootBoxGeometry.Minimum(ankle, q, center, half), min, 1e-6f, "eight-corner oracle");
            }
        }
        private static void Candidate()
        {
            var g = Geometry(); var raw = Quaternion.AngleAxis(-60f, Vector3.right);
            var a = KneeGroundTarget.Evaluate(g, raw, Vector3.right);
            Check.That(a.Valid && a.Candidate && !a.Applied, "geometry is only a candidate until the selected arm consumes it");
            Check.That(a.GoalMin >= 0f && a.GoalMin > a.RawMin && a.GoalAnkleY > a.RawAnkleY, "both ankle and full Box improve");
            Check.That(MathF.Abs(a.Correction) <= 5f && a.GoalBend > a.RawBend && a.GoalBend <= 150f, "only bounded added flexion");
            Check.That(FootGroundReference.GoalWithinBounds(g.Upper * a.Goal, g.FootRelative,
                KneeGroundTarget.SelectedFoot(g, a.Foot), g.FootBindRelative), "the paired Foot goal stays inside original correction and ROM");
            Check.That(a.GoalMin > a.FootOnlyMin, "Knee must improve over the same-state Foot-only candidate");
            var b = KneeGroundTarget.Evaluate(g, raw, new Vector3(-1f, 0f, 0f));
            Check.Near(Quaternion.Angle(a.Goal, b.Goal), 0f, .001f, "calibration sign cannot reverse the selected physical pose");
            g.Hip = new Vector3(0f, .86f, 0f); b = KneeGroundTarget.Evaluate(g, raw, Vector3.right);
            Check.That(b.Valid && b.Candidate && b.GoalMin < 0f && b.GoalMin > b.RawMin, "partial correction never claims clearance");
            Check.Near(KneeGroundTarget.Minimum(g, b.Goal, KneeGroundTarget.SelectedFoot(g, b.Foot)), b.GoalMin, 1e-6f,
                "published geometry belongs to both candidate rotations");
        }
        private static void Fallback()
        {
            var raw = Quaternion.AngleAxis(-60f, Vector3.right); var g = Geometry();
            Check.That(!KneeGroundTarget.Evaluate(default, raw, Vector3.right).Valid, "absent geometry is not zero clearance");
            Check.That(!KneeGroundTarget.Evaluate(g, raw, Vector3.zero).Valid, "absent calibration axis cannot select correction");
            g.Hip = new Vector3(0f, 1.2f, 0f);
            Check.That(!KneeGroundTarget.Evaluate(g, raw, Vector3.right).Candidate, "clear input is unchanged");
            g = Geometry(); g.FloorHalf = new Vector3(.01f, .05f, .01f);
            Check.That(!KneeGroundTarget.Evaluate(g, raw, Vector3.right).Candidate, "no correction outside the owned support");
            g = Geometry(); raw = Quaternion.AngleAxis(-.1f, Vector3.right);
            g.FootRelative = Quaternion.AngleAxis(-29.9f, Vector3.right);
            g.FootBindRelative = g.FootRelative;
            g.Hip = new Vector3(0f, .899f, 0f);
            Check.That(!KneeGroundTarget.Evaluate(g, raw, Vector3.right).Candidate,
                "cannot gain height by crossing a nearly straight knee to the opposite branch");
            g = Geometry(); g.Hip = new Vector3(float.NaN, .9f, 0f);
            Check.That(!KneeGroundTarget.Evaluate(g, raw, Vector3.right).Valid, "nonfinite native geometry is unavailable");
        }
        private static void CoupledCounterexample()
        {
            var g = Geometry(); g.Upper = Quaternion.identity;
            g.Hip = new Vector3(0f, 1.01516f, 0f); g.Half.z = .2f;
            var raw = Quaternion.AngleAxis(-10f, Vector3.right);
            g.FootRelative = g.FootBindRelative = Quaternion.AngleAxis(10f, Vector3.right);
            float rawMin = KneeGroundTarget.Minimum(g, raw);
            for (int n = 1; n <= 16; n++)
                Check.That(KneeGroundTarget.Minimum(g, raw * Quaternion.AngleAxis(-5f * n / 16f, Vector3.right)) < rawMin,
                    "holding Foot relative pose makes every tested Knee bend lower a Box corner");
            var s = KneeGroundTarget.Evaluate(g, raw, Vector3.right);
            Check.That(s.Candidate && s.GoalMin >= 0f && s.GoalAnkleY > s.RawAnkleY,
                "same bounded Knee range clears when the Foot target is evaluated with it");
            Check.That(s.Foot.CorrectionDegrees > 0f && s.Foot.CorrectionDegrees <= 15.001f, "counterrotation is explicit and bounded");
        }
        private static void FootOnly()
        {
            var g = Geometry(); g.Hip.y = .906f;
            g.FootRelative = g.FootBindRelative = Quaternion.AngleAxis(45f, Vector3.right);
            var s = KneeGroundTarget.Evaluate(g, Quaternion.AngleAxis(-60f, Vector3.right), Vector3.right);
            Check.That(s.Valid && s.Foot.Feasible && s.GoalMin >= 0f && !s.Candidate && s.Correction == 0f,
                "Foot-only solution keeps the raw Knee input");
        }
        private static void BoundedPairs()
        {
            var random = new Random(4126); int candidates = 0;
            for (int n = 0; n < 300; n++)
            {
                var g = Geometry(); float upper = (float)random.NextDouble() * 40f;
                float knee = -15f - (float)random.NextDouble() * 75f;
                g.Upper = Quaternion.AngleAxis(upper, Vector3.right);
                var raw = Quaternion.AngleAxis(knee, Vector3.right);
                g.FootRelative = g.FootBindRelative = Quaternion.AngleAxis(-upper - knee, Vector3.right);
                g.Hip.y += -.002f - (float)random.NextDouble() * .04f - KneeGroundTarget.Minimum(g, raw);
                var s = KneeGroundTarget.Evaluate(g, raw, Vector3.right);
                Check.That(s.Valid, "generated geometry remains finite");
                if (!s.Candidate) continue;
                candidates++;
                var foot = KneeGroundTarget.SelectedFoot(g, s.Foot);
                Check.That(MathF.Abs(s.Correction) <= 5f && s.GoalBend > s.RawBend && s.GoalBend <= 150f, "same bounded bend branch");
                Check.That(FootGroundReference.GoalWithinBounds(g.Upper * s.Goal, g.FootRelative, foot, g.FootBindRelative), "Foot correction ROM and upright cone retained");
                Check.That(s.GoalMin - s.FootOnlyMin >= .000099f && s.GoalAnkleY - s.RawAnkleY >= .000099f, "joint gain over Foot-only and ankle gain are both real");
                Check.Near(KneeGroundTarget.Minimum(g, s.Goal, foot), s.GoalMin, .000001f, "candidate clearance uses the published pair");
            }
            Check.That(candidates > 100, "bounds tested on a substantial set of actual candidates");
        }
    }
}
