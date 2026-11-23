using System;
using System.Linq;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Trial = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;

namespace ARL.Contracts
{
    internal static class TurnHandoffContracts
    {
        private const float Dt = 1f / 60f;
        internal static void Run()
        {
            Check.Run("169 original five starts, recorded three solves, ideal height and independent 5 mm gap", Historic);
            Check.Run("169 calibrated flexion-plane geometry, two gaps, offset shoe and world heading", Geometry);
            Check.Run("169 directional native command and angular speed headroom with carried offsets", Budgets);
            Check.Run("169 separate issued pose and measured endpoint, changed target, shape and body", Readback);
            Check.Run("169 repeated and cut starts require an actual IDLE to TURN pair in either order", StartSelection);
            Check.Run("169 right stop waits across precontact and pairs the peak with eventual ordinary landing", RightStop);
            Check.Run("169 right peak inside the three solves matches the 5327, 5332, 5334, 5340 timeline", RecordedStopTiming);
            Check.Run("169 moving animation target can invalidate a forecast horizontal gain", TargetChange);
            Check.Run("169 precontact ending in formal contact or lost opposite support is reported", NoFreeControl);
        }

        internal static Trial.Pose Pose(float height = 0f, float z = 0f)
        {
            var origin = new Vector3(.1f, height, z);
            var upperKnee = origin + new Vector3(0, .53f, .17f);
            var foot = origin + new Vector3(0, .10f, 0);
            return new Trial.Pose
            {
                Hip = origin + Vector3.up * .95f,
                UpperKnee = upperKnee,
                LowerKnee = upperKnee + Vector3.right * .001f,
                Lower = foot - Vector3.right * .001f,
                Foot = foot,
                Rotation = Quaternion.identity,
                Axis = Vector3.right,
                AllowExtension = true,
                Center = new Vector3(0, -.08f, .045f),
                Half = new Vector3(.04f, .02f, .09f),
                FloorCenter = Vector3.up * -.05f,
                FloorHalf = new Vector3(3, .05f, 3)
            };
        }

        private static Trial.Aim Aim(Trial.Pose p, float lead) => new Trial.Aim
        {
            Target = p.Foot - Vector3.forward * lead,
            TargetSole = p.Foot + p.Rotation * (p.Center - Vector3.up * p.Half.y) - Vector3.forward * lead,
            Travel = Vector3.forward
        };
        internal static Trial.Budget Budget() => new Trial.Budget
        {
            Axis = Vector3.right,
            HipCap = 40,
            KneeCap = 40,
            HipA = Vector3.zero,
            KneeA = 0,
            UpperW = Vector3.zero,
            LowerW = Vector3.zero,
            FootW = Vector3.zero,
            UpperMaxW = 18,
            LowerMaxW = 18,
            FootMaxW = 18
        };
        private static void Near(Vector3 a, Vector3 b, string message) => Check.Near((a - b).magnitude, 0f, .000002f, message);

        private static void Historic()
        {
            // 168 B's matched LEFT_RELEASE/LEFT_SHIN endpoints, in millimetres.
            // Only the normal target and preserved ankle gap are replayable from these columns.
            float[,] hip ={{-1.742363f,-2.1048784f,-2.0768642f},{-1.666069f,-2.0358562f,-1.8844604f},
                {-1.1686087f,-1.6388893f,-1.7875433f},{-.95498562f,-.97036362f,-1.3874769f},{-1.2638569f,-1.324892f,-1.8464327f}};
            float[,] sole ={{-.17448172f,-.21241485f,-5.6870241f},{-.088738218f,-.41595089f,-5.9889526f},
                {-.13896209f,-.1985193f,-4.0771441f},{-.12874392f,-.6721397f,-4.4033794f},{-.15149824f,-.3620981f,-4.3557539f}};
            float[,] gaps ={{.53996837f,4.4564862f,4.7406926f},{.83224213f,4.8300838f,4.5395241f},{.5094744f,3.9536664f,4.2536311f},
                {1.1632755f,6.1339445f,5.3877039f},{.52798337f,3.2881048f,4.6909575f}};
            int[] pass = { 0, 0, 0 };
            for (int lane = 0; lane < 5; lane++)
            {
                float h = 0;
                for (int n = 0; n < 3; n++)
                {
                    h += hip[lane, n] / 1000;
                    float gain = Trial.WantedRise(0, h, sole[lane, n] / 1000);
                    float level = sole[lane, n] / 1000 + gain;
                    Check.That(gain > 0f && gain <= -h && level >= -.002f && level <= .000001f, "normal target fits recorded upper hip descent");
                    if (gaps[lane, n] / 1000 <= Trial.GapLimit) pass[n]++;
                }
            }
            Check.That(pass.SequenceEqual(new[] { 5, 4, 4 }), "changing height cannot remove the 5114 and 5115 solve ankle excess");
            Check.Near(Trial.WantedRise(0, .003f, -.012f), 0, 0, "ascending hip gives zero");
            Check.Near(Trial.WantedRise(0, -.003f, .012f), 0, 0, "clear nominal shoe gives zero");
        }

        private static void Geometry()
        {
            var p = Pose(-.006f);
            p.Foot = p.Lower + new Vector3(.003f, -.001f, .004f);
            p.Rotation = Quaternion.AngleAxis(24f, Vector3.forward);
            var restricted = p; restricted.AllowExtension = false;
            var positive = new Trial.Offset { Knee = .02f }; var negative = new Trial.Offset { Knee = -.02f };
            var goal = Aim(p, 0f);
            Check.That(Trial.Evaluate(p, positive, goal).BendLegal && Trial.Evaluate(p, negative, goal).BendLegal &&
                Trial.Evaluate(restricted, positive, goal).BendLegal != Trial.Evaluate(restricted, negative, goal).BendLegal, "existing extension selector");
            foreach (float h in new[] { -2f, 0f, 2f }) foreach (float k in new[] { -3f, 0f, 3f })
                {
                    var o = new Trial.Offset { Hip = h * Mathf.Deg2Rad, Knee = k * Mathf.Deg2Rad };
                    var a = Aim(p, .12f); var m = Trial.Evaluate(p, o, a);
                    var hr = Quaternion.AngleAxis(h, Vector3.right); var kr = Quaternion.AngleAxis(k, Vector3.right);
                    var up = p.Hip + hr * (p.UpperKnee - p.Hip);
                    var knee = up + (p.LowerKnee - p.UpperKnee);
                    var lower = knee + hr * kr * (p.Lower - p.LowerKnee);
                    var foot = lower + (p.Foot - p.Lower);
                    Near(m.Lower, lower, "two segments around original hip"); Near(m.Foot, foot, "actual gap retained in 3D");
                    Check.Near(m.Gap, (p.Foot - p.Lower).magnitude, .000001f, "ankle gap");
                    Check.Near(m.KneeGap, (p.LowerKnee - p.UpperKnee).magnitude, .000001f, "knee gap");
                    float min = float.PositiveInfinity;
                    for (int i = 0; i < 8; i++)
                    {
                        var c = p.Center + new Vector3((i & 1) == 0 ? p.Half.x : -p.Half.x, (i & 2) == 0 ? p.Half.y : -p.Half.y, (i & 4) == 0 ? p.Half.z : -p.Half.z);
                        min = Mathf.Min(min, (foot + hr * kr * p.Rotation * c).y);
                    }
                    Check.Near(m.Sole, min, .000002f, "rotating full shoe");
                    Check.Near(m.AnkleAlong, foot.z - a.Target.z, .000002f, "signed travel");
                    a.Travel = -a.Travel;
                    Check.Near(Trial.Evaluate(p, o, a).SoleAlong, -m.SoleAlong, .000002f, "reversed travel sign");
                }
        }

        private static void Budgets()
        {
            var p = Pose(-.0055f); var b = Budget(); var a = Aim(p, -.045f);
            int changed = 0;
            foreach (float lead in new[] { -.12f, .12f }) foreach (float native in new[] { -40f, -22f, 0f, 17f, 40f })
                {
                    a = Aim(p, lead); b.HipA = Vector3.right * native; b.KneeA = -native;
                    Trial.Offset state = default;
                    for (int n = 0; n < 3; n++)
                    {
                        var result = Trial.Select(p, a, b, Dt, p.Hip.y + .006f, state);
                        if (!result.Issued) { Check.That(result.Result == Trial.Choice.NoContinuation && n > 0, "stopped with full history"); break; }
                        var o = result.Offset; if (o.Changed) changed++;
                        Check.Near((o.Hip - state.Hip) / Dt, o.HipRate, .00005f, "hip rate continuity");
                        Check.Near((o.KneeRate - state.KneeRate) / Dt, result.KneeDeltaA, .0001f, "knee acceleration continuity");
                        Check.That((b.HipA + b.Axis * result.HipDeltaA).magnitude <= b.HipCap + .0001f &&
                            Mathf.Abs(b.KneeA + result.KneeDeltaA) <= b.KneeCap + .0001f, "directional native caps");
                        Check.That(result.Trial.Tracks(result.Base) && result.Trial.Safe, "signed and full horizontal errors, footprint");
                        if (o.Changed) Check.That(result.Trial.Sole >= result.Base.Sole - Trial.Precision, "lowest corner");
                        state = o;
                    }
                }
            Check.That(changed > 0, "genuine upstream angle changes");
            b = Budget(); b.UpperMaxW = b.LowerMaxW = b.FootMaxW = .001f;
            var limited = Trial.Select(p, a, b, Dt, p.Hip.y + .006f, default);
            Check.That(limited.Result == Trial.Choice.Baseline, "native speeds can exhaust the search");
            b.UpperW = Vector3.up; Check.That(Trial.Select(p, a, b, Dt, p.Hip.y, default).Result == Trial.Choice.MissingBudget, "invalid nominal even at rest");
            b = Budget(); p.Axis = Vector3.zero; Check.That(Trial.Select(p, a, b, Dt, p.Hip.y, default).Result == Trial.Choice.MissingGeometry, "unknown calibration");
            p = Pose(); p.FloorHalf.z = .01f; Check.That(Trial.Select(p, a, b, Dt, p.Hip.y, default).Result == Trial.Choice.Outside, "whole footprint");
            p = Pose(); a = Aim(p, .145039f); var held = Trial.Select(p, a, b, Dt, p.Hip.y, default);
            Check.That(held.Result == Trial.Choice.Baseline && !held.Offset.Changed && held.Tested == 0, "clear right-stop reference");
            Check.Near(held.Trial.AnkleAlong, .145039f, .000001f, "signed lead preserved for neutral control");
        }

        internal static Trace.Tick Tick(int local, Trial.Pose p, float lead = 0f)
        {
            var t = new Trace.Tick
            {
                Local = local,
                Body = 28,
                Floor = 3,
                Gait = new FootGaitEvidence(5, true, 3, 0, Vector3.up, 0, 0),
                Inside = true,
                Step = new PhysicsStepContext { Epoch = 1, Sequence = local, Dt = Dt, FixedTime = local * Dt, UnscaledTime = local * Dt, SourceSequence = local },
                Source = new FootTransitionSource.Sample { Clock = new SourceStepContext { Epoch = 1, Sequence = local, Dt = Dt, Time = local * Dt } }
            };
            SetPose(ref t, p, lead); return t;
        }
        internal static void SetPose(ref Trace.Tick t, Trial.Pose p, float lead)
        {
            var aim = Aim(p, lead); var m = Trial.Evaluate(p, default, aim);
            t.Foot = p.Foot; t.Lower = p.Lower; t.Target = aim.Target; t.SourceSole = aim.TargetSole;
            t.Sole = p.Foot + p.Rotation * (p.Center - Vector3.up * p.Half.y); t.BoxMin = m.Sole;
        }
        private static string Report(TurnHandoffWindow w, int peak = 0, float lead = 0f)
        { var b = new StringBuilder(); w.Append(b, 1, w.Start, "Ordinary", 40, 40, "NewForwardLead", peak, lead); return b.ToString(); }

        private static void Readback()
        {
            var p = Pose(); var t = Tick(10, p, .04f); var pending = new TurnHandoffWindow(10, Vector3.forward); var b = Budget();
            Check.That(pending.Visit(t, p, 30, 31, null), "accept origin");
            var next = Pose(-.004f); pending.Issue(t, next, b); pending.Issue(t, next, b);
            Check.That(!pending.Pending(t.Local), "duplicate ignored");
            var moved = next; moved.Hip.x += .001f;
            t = Tick(11, moved, -.08f);
            Check.That(!pending.Visit(t, moved, 30, 31, null) && pending.End == "BaselineMismatch", "nominal drift is visible even with same shoe");
            string text = Report(pending);
            Check.That(text.Contains("/read=True") && text.Contains("/nominalAgrees-horizontal-basePass-trialPass=False|") &&
                text.Split("TURN_HANDOFF_STEP").Length == 2, "mismatched forecast cannot pass");
            foreach (string reason in new[] { "BodyOrBoxChanged", "PoseMismatch", "MissingGeometry", "ContactOrBounds", "MissingCompletion" })
            {
                var w = new TurnHandoffWindow(1, Vector3.forward); var first = Tick(1, p);
                Check.That(w.Visit(first, p, 30, 31, null), "first sample"); w.Issue(first, p, b);
                var m = p; var successor = Tick(2, p);
                int upper = 30;
                if (reason == "BodyOrBoxChanged") upper = 49;
                if (reason == "PoseMismatch") successor.BoxMin += .015f;
                if (reason == "MissingGeometry") m.Axis = Vector3.zero;
                if (reason == "ContactOrBounds") successor.Precontact = true;
                if (reason == "MissingCompletion") successor.Local++;
                Check.That(!w.Visit(successor, m, upper, 31, null) && w.End == reason, reason);
                if (reason == "ContactOrBounds") Check.That(Report(w).Contains("/read=True"), "settle the free solve before the precontact FOR_N");
            }
        }

        internal static void SetFree(ref Trace.Tick t)
        { t.Was = false; t.Active = false; t.Phase = Phase.Swing; t.EventFloor = 0; }
        internal static void SetRelease(ref Trace.Tick t)
        { t.Was = true; t.Active = false; t.Phase = Phase.Lift; t.EventFloor = 0; t.ReleaseReason = "RELEASE_ON_TRANSITION_TO_SWING"; }

        private static void StartSelection()
        {
            foreach (bool reverse in new[] { true, false }) foreach (string mode in new[] { "Pair", "Single", "WrongMotion", "WrongReason" })
                {
                    var sim = new FootTransitionContracts.Sim { Reverse = reverse };
                    sim.Frame(0, 0, 0); sim.Frame(0, 0, 0, (ref Trace.Tick l, ref Trace.Tick r) => { l.Motion = r.Motion = "IDLE"; });
                    PhysicsStepContext step = default;
                    sim.Frame(.02f, -.035f, .027f, (ref Trace.Tick l, ref Trace.Tick r) =>
                    {
                        SetRelease(ref l); if (mode != "Single") SetRelease(ref r); l.Motion = r.Motion = mode == "WrongMotion" ? "ACCEL" : "TURN";
                        if (mode == "WrongReason") l.ReleaseReason = "OTHER"; step = l.Step;
                    });
                    bool found = sim.Trace.TryHandoff(0, step, out _, out _, out _, out _, out _, out _);
                    Check.That(found == (mode == "Pair"), "exact bilateral origin " + mode);
                    Check.That(!sim.Trace.TryHandoff(1, step, out _, out _, out _, out _, out _, out _), "never use right turn as braking control");
                    if (found)
                    {
                        sim.Frame(.03f, -.055f, .035f, (ref Trace.Tick l, ref Trace.Tick r) => { SetFree(ref l); SetFree(ref r); });
                        Check.That(sim.Report().Contains("/end=MissingCompletion/model=CONDITIONED_POSE"), "first physics receipt cannot be skipped");
                    }
                }
        }

        private static FootTransitionContracts.Sim BeforeStop()
        {
            var sim = new FootTransitionContracts.Sim();
            sim.Frame(0, 0, 0); sim.Frame(0, 0, 0);
            sim.Frame(.02f, 0, .025f, (ref Trace.Tick l, ref Trace.Tick r) => SetRelease(ref r));
            return sim;
        }

        private static void RightStop()
        {
            var sim = BeforeStop(); var b = Budget();
            // Sequence shape of 168 B: native precontact covers the quiet frame,
            // a free interval follows, the ankle peaks, and ordinary contact is last.
            var leads = new[] { .07581f, .088f, .096f, .11f, .12f, .13f, .145039f, .139f, .125f };
            for (int i = 0; i < leads.Length; i++)
            {
                int n = 4 + i;
                var p = Pose(.014f, leads[i]); PhysicsStepContext step = default;
                sim.Frame(.02f, 0, .025f, (ref Trace.Tick l, ref Trace.Tick r) =>
                {
                    SetFree(ref r); l.Motion = r.Motion = "STOP"; SetPose(ref r, p, leads[i]);
                    if (i < 2) { r.Precontact = true; r.EventFloor = r.Floor; }
                    if (i == leads.Length - 1) { r.Active = r.Ordinary = true; r.EventFloor = r.Floor; r.Phase = Phase.Stance; }
                    step = r.Step;
                });
                bool pending = sim.Trace.TryHandoff(1, step, out var tick, out var w, out var prohibit, out _, out _, out _);
                var peak = sim.Trace.RightPeak(step);
                Check.That(pending == (n >= 6 && n <= 9), "free interval for right stop " + n);
                bool issue = pending && w.Visit(tick, p, 8, 9, prohibit);
                peak?.ReadPeak(tick, p, 8, 9);
                if (issue) w.Issue(tick, Pose(.014f, leads[i + 1]), b);
            }
            var log = sim.Report();
            Check.That(log.Contains("quiet-precontactSolves-start=4|2|6/result=Selected"), "precontact exposure retained");
            Check.That(log.Contains("/end=ThreeReadbacks") && log.Contains("/first-outcome-resolved=12|Ordinary|12"), "terminal native link");
            Check.That(log.Contains("/rightStopLead-peakAt-mm=NewForwardLead|10|145.039") &&
                log.Contains("/peakComparison=HeldThirdPose/peakUnderContact=False"), "only the actual signed peak is conditioned");
            Check.That(log.Contains("/peakBounds-tracking=True|True"), "same geometry at measured peak");
        }

        private static void RecordedStopTiming()
        {
            // Local 4 maps to actual 5327. Five selected precontact solves expire at local 9.
            var sim = BeforeStop(); var b = Budget();
            float[] lead = { .07581f, .093f, .104f, .115f, .123f, .134f, .141f, .1450392f, .139f, .13f, .129f, .127f, .125f, .119423f };
            for (int i = 0; i < lead.Length; i++)
            {
                var p = Pose(.02f, lead[i]); PhysicsStepContext step = default;
                sim.Frame(.02f, 0, .025f, (ref Trace.Tick l, ref Trace.Tick r) =>
                {
                    SetFree(ref r); l.Motion = r.Motion = "STOP"; SetPose(ref r, p, lead[i]);
                    if (i < 5 || (i >= 10 && i < 13)) { r.Precontact = true; r.EventFloor = r.Floor; }
                    if (i == 13) { r.Ordinary = r.Active = true; r.EventFloor = r.Floor; r.Phase = Phase.Stance; }
                    step = r.Step;
                });
                bool pending = sim.Trace.TryHandoff(1, step, out var tick, out var w, out var why, out _, out _, out _);
                var peak = sim.Trace.RightPeak(step);
                bool issue = pending && w.Visit(tick, p, 8, 9, why);
                peak?.ReadPeak(tick, p, 8, 9);
                if (issue) w.Issue(tick, Pose(.02f, lead[i + 1]), b);
            }
            string log = sim.Report();
            Check.That(log.Contains("quiet-precontactSolves-start=4|5|9") &&
                log.Contains("/first-outcome-resolved=17|Ordinary|17"), "original chronology");
            Check.That(log.Contains("NewForwardLead|11|145.039") && log.Contains("peakComparison=PairedStep"), "second free solve is the true peak");
            Check.That(log.Contains("side=RIGHT/retainedStarted-complete=1|1/rightQuietEligible=1") &&
                log.Contains("step3(issued-read-calibrated-basePass-trialPass-new-lost-improved)=1|1|1|1|1|0|0|0"), "neutral comparison counted without gains");
        }

        private static void TargetChange()
        {
            var p = Pose(); var next = Pose(-.006f); var b = Budget();
            bool tested = false;
            foreach (float lead in new[] { -.09f, .09f })
            {
                var t = Tick(1, p, lead);
                var aim = Aim(p, lead);
                var cmd = Trial.Select(next, aim, b, Dt, p.Hip.y, default);
                if (!cmd.Issued || Mathf.Abs(cmd.Trial.AnkleAlong - cmd.Base.AnkleAlong) < .0002f) continue;
                var w = new TurnHandoffWindow(1, Vector3.forward);
                Check.That(w.Visit(t, p, 30, 31, null), "origin"); w.Issue(t, next, b);
                t = Tick(2, next, -lead);
                Check.That(!w.Visit(t, next, 30, 31, null) && w.End == "ReadbackGeometry", "true target reverses the planar comparison");
                Check.That(Report(w).Contains("nominalAgrees-horizontal-basePass-trialPass=True|False"), "conditioned failure is visible");
                tested = true; break;
            }
            Check.That(tested, "nonzero old-horizontal alternative tested");
        }

        private static void NoFreeControl()
        {
            foreach (bool loseLeft in new[] { false, true })
            {
                var sim = BeforeStop(); sim.Frame(.02f, 0, .025f, (ref Trace.Tick l, ref Trace.Tick r) =>
                {
                    SetFree(ref r); r.Precontact = true; r.EventFloor = r.Floor; l.Motion = r.Motion = "STOP";
                });
                sim.Frame(.02f, 0, .025f, (ref Trace.Tick l, ref Trace.Tick r) =>
                {
                    SetFree(ref r);
                    if (loseLeft) { SetRelease(ref l); r.Precontact = true; r.EventFloor = r.Floor; }
                    else { r.Ordinary = r.Active = true; r.EventFloor = r.Floor; }
                });
                Check.That(sim.Report().Contains("quiet-precontactSolves-start=4|1|0/result=SupportOrEntry"), "censored quiet control");
            }
        }
    }
}
