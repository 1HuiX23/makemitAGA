using System;
using System.Linq;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;
using Pivot = MakemitAGA.ActiveRagdollLab.Physics.RightKneePivot;

namespace ARL.Contracts
{
    internal static class RightKneePivotContracts
    {
        internal static void Run()
        {
            Check.Run("v149 signed pivot components close through both physical knee anchors in a world frame", Parts);
            Check.Run("v149 dominant thigh rotates the distal limb and the full box at fixed joint gaps", Rotation);
            Check.Run("v149 isolated translation, previous travel, normal clearance and footprint losses", Bounds);
            Check.Run("v149 warning and ordinary comparison windows respect source, cycle and surface continuity", Windows);
            Check.Run("v149 knee velocity prediction is causal and can be separately censored", Forecast);
            Check.Run("v149 per-swing ring is bounded and allocates nothing after initialization", Allocation);
        }
        private const float Tolerance = .000003f;
        private static float MinCorner(in Pivot.Frame p, Vector3 ankle, Quaternion rotation)
        {
            float height = float.PositiveInfinity;
            for (int x = -1; x <= 1; x += 2)
                for (int y = -1; y <= 1; y += 2)
                    for (int z = -1; z <= 1; z += 2)
                    {
                        var local = p.Center + Vector3.Scale(p.Half, new Vector3(x, y, z));
                        height = Math.Min(height, (ankle + rotation * local).y - (p.FloorCenter.y + p.FloorHalf.y));
                    }
            return height;
        }
        internal static Pivot.Frame Pose(float hip, float angle, float gap, float box = -.006f)
        {
            var p = new Pivot.Frame
            {
                State = Pivot.Capture.Ready,
                Dt = 1f / 60f,
                UpperHip = new Vector3(.3f, 1.2f, -.28f),
                UpperR = Quaternion.AngleAxis(20f, Vector3.right),
                GoalR = Quaternion.AngleAxis(20f + angle, Vector3.right),
                FootR = Quaternion.AngleAxis(11f, Vector3.right),
                Center = new Vector3(.015f, -.065f, .045f),
                Half = new Vector3(.05f, .026f, .12f),
                FloorHalf = new Vector3(2f, .1f, 2f)
            };
            Vector3 span = Vector3.up * -.44f;
            p.ReferenceHipY = p.UpperHip.y + hip;
            p.ParentHipY = p.UpperHip.y - .0015f;
            p.UpperKnee = p.UpperHip + p.UpperR * span;
            p.LowerKnee = p.UpperKnee + new Vector3(.001f, -gap, -.002f);
            p.LowerAnkle = p.LowerKnee + new Vector3(.011f, -.43f, .28f);
            p.FootAnkle = p.LowerAnkle + new Vector3(.001f, .003f, .001f);
            p.GoalKneeY = p.ReferenceHipY + (p.GoalR * span).y;
            p.RawAnkleY = p.LowerAnkle.y + .01f;
            p.FloorCenter.y = MinCorner(p, p.FootAnkle, p.FootR) - box;
            p.Box = box;
            return p;
        }
        private static Pivot.Split PartsOf(in Pivot.Frame p)
        {
            Check.That(Pivot.Decompose(p, out var parts) == Pivot.Capture.Ready, "finite and independently closed");
            Check.Near(parts.Total, parts.Hip + parts.Thigh + parts.KneeGap, Tolerance, "additive identity");
            return parts;
        }
        private static void Parts()
        {
            foreach (float h in new[] { -.004f, .004f, .024f })
                foreach (float degrees in new[] { -5f, 0f, 7f })
                    foreach (float gap in new[] { -.005f, .004f, .016f })
                    {
                        var p = Pose(h, degrees, gap);
                        var a = PartsOf(p);
                        float lever = .44f * (Mathf.Cos(20f * Mathf.Deg2Rad) - Mathf.Cos((20f + degrees) * Mathf.Deg2Rad));
                        Check.Near(a.Total, h + gap + lever, Tolerance, "independent geometry");
                        Check.Near(a.Receiver, h + .0015f, Tolerance, "reference to pelvis");
                        Check.Near(a.HipJoint, -.0015f, Tolerance, "pelvis to upper-side hip");
                        Check.Near(a.Thigh, lever, Tolerance, "upper span vertical change");
                        Check.Near(a.KneeGap, gap, Tolerance, "upper to lower anchor");
                    }
            var ties = Pose(.004f, 0f, .004f);
            var tied = PartsOf(ties);
            Check.That(tied.Dominant == Pivot.Part.Tied && !Pivot.Compare(ties, tied, default, false).Measured, "ties are explicit");
            Check.That(PartsOf(Pose(-.03f, 4f, .002f)).Dominant == Pivot.Part.NoDeficit, "offset parts can cancel");
            var invalid = ties;
            invalid.GoalKneeY += .002f;
            Check.That(Pivot.Decompose(invalid, out _) == Pivot.Capture.GeometryMismatch, "goal independent of decomposition");
            invalid = ties;
            invalid.Box += .002f;
            Check.That(Pivot.Decompose(invalid, out _) == Pivot.Capture.GeometryMismatch, "all eight real corners close");
            invalid = ties;
            invalid.UpperR = default;
            Check.That(Pivot.Decompose(invalid, out _) == Pivot.Capture.GeometryMismatch, "missing physical orientation");
            invalid = ties;
            invalid.ReferenceHipY = float.NaN;
            Check.That(Pivot.Decompose(invalid, out _) == Pivot.Capture.GeometryMismatch, "nonfinite hip cannot win");
            invalid = ties;
            invalid.State = Pivot.Capture.MissingAnchors;
            Check.That(Pivot.Decompose(invalid, out _) == Pivot.Capture.MissingAnchors, "missing anchors are not zero");
        }
        private static void Rotation()
        {
            var p = Pose(.0004f, 7f, .001f);
            var s = PartsOf(p);
            Check.That(s.Dominant == Pivot.Part.Thigh, "thigh wins");
            var t = Pivot.Compare(p, s, default, false);
            var turn = Quaternion.AngleAxis(7f, Vector3.right);
            var knee = p.UpperHip + turn * (p.UpperKnee - p.UpperHip) + (p.LowerKnee - p.UpperKnee);
            var ankle = knee + turn * (p.LowerAnkle - p.LowerKnee);
            var foot = ankle + (p.FootAnkle - p.LowerAnkle);
            Check.Near(t.KneeY, s.Thigh, Tolerance, "full restores precisely thigh component");
            Check.Near(t.AnkleY, ankle.y - p.LowerAnkle.y, Tolerance, "child follows rotation");
            Check.Near(t.Box, MinCorner(p, foot, turn * p.FootR), Tolerance, "rotated eight-corner ground minimum");
            Check.That(t.Measured && t.Inside && t.KneeGain > 0f && t.AnkleGain < 0f && t.DepthGain < 0f, "raising knee can worsen foot");
            var prev = p;
            prev.UpperR = Quaternion.AngleAxis(19f, Vector3.right);
            prev.Dt = 2f * p.Dt;
            var one = Pivot.Compare(p, s, prev, true);
            Check.Near(one.Angle, .5f, .0004f, "original angular travel scaled to new dt");
            prev.UpperR = p.UpperR;
            one = Pivot.Compare(p, s, prev, true);
            Check.That(one.Measured && one.Angle == 0f, "zero incoming angular travel stays zero");
        }
        private static void Bounds()
        {
            var p = Pose(.008f, 0f, .001f, .022f);
            var s = PartsOf(p);
            Check.That(s.Dominant == Pivot.Part.Hip, "hip wins including child anchor displacement");
            var t = Pivot.Compare(p, s, default, false);
            Check.Near(t.KneeY, .008f, Tolerance, "vertical upper hip correction");
            Check.Near(t.AnkleY, .008f, Tolerance, "distal follows translation");
            Check.Near(t.Box, .030f, Tolerance, "compares ordinary clearance");
            Check.That(t.WasInBand && !t.InBand && t.Inside && t.DepthGain == 0f, "ordinary can leave the height band");
            var before = p;
            before.UpperHip.y -= .0015f;
            before.Dt = p.Dt / 2;
            t = Pivot.Compare(p, s, before, true);
            Check.Near(t.KneeY, .003f, Tolerance, "recent hip travel");
            p = Pose(-.001f, 0f, .008f);
            s = PartsOf(p);
            Check.That(s.Dominant == Pivot.Part.KneeGap, "passive joint gap wins");
            before = p;
            before.LowerKnee.y -= .002f;
            before.UpperHip.y -= .05f;
            t = Pivot.Compare(p, s, before, true);
            Check.Near(t.AnkleY, .002f, Tolerance, "only gap travel sets this scale");
            Check.That(!Pivot.Compare(p, s, default, true).Measured, "missing previous step cannot be treated as zero");
            var unsafePose = Pose(0f, 7f, 0f);
            // Place the left edge of the floor just behind the original rear corner.
            var rear = unsafePose.FootAnkle + unsafePose.FootR * (unsafePose.Center - new Vector3(0f, unsafePose.Half.y, unsafePose.Half.z));
            unsafePose.FloorCenter.z = rear.z + .5f - .001f;
            unsafePose.FloorHalf.z = .5f;
            s = PartsOf(unsafePose);
            t = Pivot.Compare(unsafePose, s, default, false);
            Check.That(t.Measured && !t.Inside, "lost corner remains visible as an adverse result");
        }
        private static Audit.Sample Sample(int i)
        {
            var s = SwingKneeContracts.Sample(i, 1f / 60f, i / 60f);
           
            s.ParentHeightMeasured = true;
            s.EarlyFoot = EarlyLandingWindow.Foot.Right;
            s.SourcePhase = "STEADY";
            s.RawMin = 0f;
            s.KneeHeights = new Vector3(.49f, .49f, .485f);
            s.AnkleHeights = new Vector3(.11f, .101f, .103f);
            s.Min = -.006f;
            s.AnchorGap = i <= 4 ? .007f : .003f;
            return s;
        }
        private static string Event(bool ordinary, bool warn, bool cut, bool heldSource = false, bool edgeWarning = false)
        {
            var h = new FootLandingOpportunityAudit();
            var detector = new EarlyLandingWindow();
            for (int i = 1; i <= 10; i++)
            {
                var s = Sample(i);
                if (heldSource && i == 6)
                {
                    var old = Sample(i - 1);
                    s.Source = old.Source;
                    s.Step.SourceSequence = old.Source.Sequence;
                }
                if (!warn || (edgeWarning && i < 8)) s.RawMin = .008f;
                if (i == 6 && warn) s.AnkleHeights.y = s.AnkleHeights.x;
                if (i == 7 && cut) s.SupportId = 15;
                if (i >= 9)
                {
                    s.Strong = true;
                    s.Gait = Audit.Phase.Approach;
                }
                s.EarlySignal = detector.Observe(s, s.EarlyFoot);
                s.RightWarningOrigin = detector.RightWarningOrigin;
                s.OrdinaryEnabled = i == 10 && ordinary;
                s.NowActive = s.OrdinaryEnabled;
                s.FirstMiss = i == 10 && !ordinary;
                s.MissInProgress = s.FirstMiss;
                s.DepthLatchFixed = s.FirstMiss ? 9 : 0;
                var frame = RightAnkleWindow.InSwingContext(s) ? Pose(.004f, 0f, .001f) : new Pivot.Frame { State = Pivot.Capture.OutsideSwing };
                h.Observe(s, true, frame);
            }
            if (!ordinary)
            {
                var s = Sample(11);
                h.Observe(s, true, new Pivot.Frame { State = Pivot.Capture.OutsideSwing });
            }
            h.Seal();
            return h.Summary();
        }
        private static void Windows()
        {
            string normal = Event(true, false, false, true);
            Check.That(normal.Contains("/outcome=Enabled/align=Edge/edgeLocal=9/centerLocal=8") && normal.Contains("/coverage=Full/stored=5"), "ordinary without a warning keeps the same five positions");
            Check.That(normal.Contains("center-offset=8|-4/local-source=4|4") && normal.Contains("/gapWithin5mm=False/state=Ready"), "over-gap history visible");
            Check.That(normal.Contains("center-offset=8|-2/local-source=6|5"), "a held render source is legal");
            string missed = Event(false, true, false);
            Check.That(missed.Contains("/outcome=HoldEnded/align=Warning/edgeLocal=9/centerLocal=5") && missed.Contains("/onset=5/leadMs="), "cancelled warning stays linked");
            Check.That(missed.Contains("/align=Warning/edgeLocal=9/centerLocal=7") && missed.Contains("/align=Edge/edgeLocal=9/centerLocal=8"), "later warning before edge and independent control window");
            string coincident = Event(false, true, false, edgeWarning: true);
            var sameCenter = coincident.Split('\n').Where(line => line.StartsWith("RIGHT_PIVOT_WINDOW_")).ToArray();
            Check.That(sameCenter.Length == 1 && sameCenter[0].Contains("/align=WarningAndEdge/edgeLocal=9/centerLocal=8"),
                "the same warning and edge endpoint are stored once");
            string cut = Event(true, false, true);
            Check.That(cut.Contains("/centerLocal=8/leadToEdgeMs=") && cut.Contains("/coverage=Partial/stored=1") &&
                !cut.Contains("center-offset=8|-1") && cut.Contains("/recent=NA"), "no motion or prior frames across a changed support");
        }
        private static void Forecast()
        {
            var w = new RightAnkleWindow();
            var s = Sample(5);
            s.EarlySignal = EarlyLandingWindow.Signal.RightNearLow;
            s.RightWarningOrigin = 5;
            var r = new RightAnkleProbe.Result
            {
                BaseMeasured = true,
                PivotForecast = true,
                PredictedKneeShift = .001f,
                Baseline = new RightAnkleProbe.Heights { Ankle = .100f, Box = -.007f }
            };
            w.Observe(s, r);
            var n = Sample(6);
            n.Gait = Audit.Phase.Approach;
            n.Min = -.006f;
            n.KneeHeights.z += .0005f;
            w.Observe(n, default);
            string text = w.Summary();
            Check.That(text.Contains("/pointVelocityForecastMm=1") && text.Contains("/actualMinusMovingBaseMm(ankle-box)=") && text.Contains("/actualKneeShiftMm="), "causal original prediction gets next same-surface readback");
            string[] fields = text.Split('/');
            var residual = fields.First(x => x.StartsWith("actualMinusMovingBaseMm")).Split('=')[1].Split('|');
            Check.Near(float.Parse(residual[0], System.Globalization.CultureInfo.InvariantCulture), 0f, .0001f, "corrected ankle residual");
            Check.Near(float.Parse(residual[1], System.Globalization.CultureInfo.InvariantCulture), 0f, .0001f, "corrected box residual");
            var withoutPrediction = new RightAnkleWindow();
            withoutPrediction.Observe(s, default);
            withoutPrediction.Observe(n, default);
            string observed = withoutPrediction.Summary();
            Check.That(observed.Contains("/actualKneeShiftMm=") && !observed.Contains("/actualMinusMovingBaseMm"),
                "an independently measured pivot displacement survives a missing forecast");

            w = new RightAnkleWindow();
            w.Observe(s, r);
            n.FivePoint = false;
            w.Observe(n, default);
            text = w.Summary();
            Check.That(text.Contains("/pointVelocityForecastMm=1") && !text.Contains("/actualMinusMovingBaseMm"), "censored outcome preserves prospective prediction");
        }
        private static void Allocation()
        {
            var h = new LandingEventHistory();
            var pose = Pose(.002f, 0f, .001f);
            var s = Sample(5);
            h.Add(s, 0, Audit.Screen(s), pose);
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 1; i <= 2000; i++)
            {
                s.Fixed = i;
                h.Add(s, i / 120d, Audit.Screen(s), pose);
            }
            Check.That(GC.GetAllocatedBytesForCurrentThread() == before, "no managed heap traffic on ordinary swing samples");
        }
    }
}
