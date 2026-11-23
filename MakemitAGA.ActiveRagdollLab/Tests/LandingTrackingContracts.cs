using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Entry = MakemitAGA.ActiveRagdollLab.Physics.OrdinaryFootEntry;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LandingFootTraction;
using Height = MakemitAGA.ActiveRagdollLab.Physics.LandingHipHeightBudget;
using Leg = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;
using Search = MakemitAGA.ActiveRagdollLab.Physics.CombinedLandingSearch;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;

namespace ARL.Contracts
{
    internal static class LandingTrackingContracts
    {
        internal static void Run()
        {
            Check.Run("176 fifty original 175 B issued commands and measured poses through all four native misses", Recorded);
            Check.Run("176 relaxed height covers rotating box corners and rigid common translation", HeightGeometry);
            Check.Run("176 socket height alternatives use signed velocities, real dt and independent solves", HeightChoices);
        }

        private static int I(string s) => int.Parse(s, CultureInfo.InvariantCulture);
        private static float F(string s) => float.Parse(s, CultureInfo.InvariantCulture);
        private static Vector3 V(string s)
        {
            var v = s.Split(','); return new Vector3(F(v[0]), F(v[1]), F(v[2]));
        }

        private static void Recorded()
        {
            using var file = new StreamReader(typeof(LandingTrackingContracts).Assembly.GetManifestResourceStream("ARL.Contracts.LandingTracking175"));
            file.ReadLine();
            var events = new List<string[]>();
            string line;
            while ((line = file.ReadLine()) != null) events.Add(line.Split('\t'));
            var releases = events.GroupBy(r => I(r[0])).ToArray();
            int shoes = 0, oldShoes = 0;
            foreach (var release in releases)
            {
                if (Replay(release.ToArray())) shoes++;
                if (release.First()[26] == "WouldEnter") oldShoes++;
            }
            Check.That(events.Count == 50 && releases.Length == 4 && oldShoes == 2 && shoes == 3, "separate old and new shoe outcomes");
        }

        // The selected paths see the logged PRE_N aim only. The actual pose, refreshed target
        // and owner-provided gait/identity flags are decoded as the next physical read.
        private static bool Replay(string[][] rows)
        {
            int start = I(rows[0][0]), first = I(rows[0][23]), strong = I(rows[0][27]);
            var initial = TurnLandingContracts.DecodeIssue(rows[0]);
            var pose = initial.Pose;
            pose.Hip.y = initial.HipY;
            pose.Lower = pose.Foot - V(rows[0][17]);
            var tick = ContinuousLandingContracts.Tick(start, pose, strong, first);
            SetClock(ref tick,I(rows[0][28]),I(rows[0][24].Split('|')[0]),initial.Dt);
            tick.Target = initial.Issue.Target; tick.SourceSole = initial.Issue.TargetSole;
            var window = new TurnHandoffWindow(start,initial.Issue.Travel,TurnHandoffWindow.Policy.LeftCombined, usePreview: false);
            Check.That(window.Visit(tick,pose,22,23,null), "release pose and source");
            var search = new Search(usePreview: false);
            Search.Step previous = null;
            foreach (var row in rows)
            {
                var s = TurnLandingContracts.DecodeIssue(row);
                int to = start + I(row[1]), from = to-1;
                var clocks = row[24].Split('|').Select(I).ToArray();
                Check.That(tick.Step.Sequence == I(row[28]) && tick.Step.SourceSequence == clocks[0] &&
                    (tick.Target-s.Issue.Target).magnitude < 2e-6f && (tick.SourceSole-s.Issue.TargetSole).magnitude < 2e-6f,
                    "issue shares source, local clock and original aim");
                var cmd = CombinedLandingContracts.Command(row, false);
                var gap = V(row[17]); var issueClock = tick.Step;
                window.Issue(tick,s.Pose,s.Budget,cmd);
                var step = search.Issue(from,s.Dt,s.HipY,s.Pose,s.Issue,s.Budget,cmd,gap,previous);
                var real = s.Pose; var bones = row[18].Split('|').Select(V).ToArray();
                real.Hip = bones[0]; real.UpperKnee = bones[1]; real.LowerKnee = bones[2]; real.Lower = bones[3]; real.Foot = bones[4];
                real.Axis = V(row[19]); var q = row[20].Split(',').Select(F).ToArray(); real.Rotation = new Quaternion(q[0],q[1],q[2],q[3]);
                var aim = TurnLandingContracts.ReadAim(row[9],s);
                var e = row[21].Split('|');
                var receipt = new Entry.Sample { Local = I(e[0]), Floor = I(e[1]), Confirmed = bool.Parse(e[2]),
                    LegacyReady = bool.Parse(e[3]), IdentityHeld = bool.Parse(e[4]), NativeInside = bool.Parse(e[5]),
                    NativeSole = row[22] == "NA" ? 0f : F(row[22])*.001f };
                Check.That(receipt.Local == to && step.Count <= Search.Width, "bounded issued set meets original owner receipt");
                var native = Leg.Evaluate(real,default,aim);
                search.Read(to,step,real,aim,native,receipt,to == first);
                tick = ContinuousLandingContracts.Tick(to,real,strong,first,strong);
                SetClock(ref tick,issueClock.Sequence+1,clocks[1],s.Dt);
                tick.Target = aim.Target; tick.SourceSole = aim.TargetSole; tick.Entry = receipt;
                tick.Entry.Floor = receipt.Floor == 0 ? 0 : tick.Floor;
                Check.That(window.Visit(tick,real,22,23,null) == (to < first), "native ordinary owner matches independently of search");
                Check.That(window.TryHeightPair(from,to,issueClock,tick.Step,22,tick.Body,tick.Floor,out var forecast,out var endpoint) &&
                    (forecast.Hip-s.Pose.Hip).sqrMagnitude == 0f && (endpoint.Hip-real.Hip).sqrMagnitude == 0f,
                    "paired clocks and real identities expose the correct two geometries");
                var wrongClock = tick.Step; wrongClock.Epoch++;
                Check.That(!window.TryHeightPair(from,to,issueClock,wrongClock,22,tick.Body,tick.Floor,out _,out _) &&
                    !window.TryHeightPair(from,to,issueClock,tick.Step,23,tick.Body,tick.Floor,out _,out _) &&
                    !window.TryHeightPair(from,to,issueClock,tick.Step,22,tick.Body,tick.Floor+1,out _,out _), "no cross-event height comparison");
                double ceiling = Height.Ceiling(endpoint)*1000.0;
                Check.That(ceiling >= F(row[38])-.0002 && ceiling <= F(row[39])+.0002,
                    "analytic box ceiling agrees with independent interval bounds at " + to + " value=" + ceiling);
                CheckShoes(to,step,real,aim,native,cmd,row[25]);
                CheckHipBudget(to,row,s.Dt,ceiling);
                previous = step;
            }
            return CheckFirst(start,strong,first,search,window,tick);
        }

        private static void CheckShoes(int to, Search.Step step, in Leg.Pose real, in Leg.Aim aim, in Leg.Measure native, in Foot.Input cmd, string oldCounts)
        {
            for (int i = 0; i < step.Count; i++)
            {
                var path = step.Paths[i];
                if (!path.SafeRead) continue;
                var leg = Leg.Evaluate(real,path.Leg.Offset,aim);
                var final = Leg.TranslateShoe(real,leg,aim,path.Move.Body.Position);
                Check.That(path.HasHorizontalRead && leg.Safe && leg.MaintainsHeight(native) &&
                    final.Tracks(native) && final.Safe && final.MaintainsHeight(native) && final.Gap < cmd.Tether.HardGap &&
                    (!path.Protected || leg.Tracks(native)), "safe leg, final shoe and unchanged protected path");
                Check.Near(path.PostTraction.AnkleXZ,final.AnkleXZ-native.AnkleXZ,1e-7f,"final shoe at next target");
                if (to == 4169)
                {
                    float before = Max(path.PreTraction)*1000f, after = Max(path.PostTraction)*1000f;
                    Check.That(!path.Protected && before > 1.01f && before < 1.06f && after > .05f && after < .09f &&
                        path.ReadLower.XZ > .00019f && path.ReadLower.XZ < .00021f && path.ReadLower.Along < -.0047f &&
                        path.FirstLowerExcess > 0, "traction moves the shoe; the shank point has its own planar excess");
                }
            }
            if (to == 4169)
                Check.That(step.SafeReads == 4 && I(oldCounts.Split('|')[4]) == 0, "all four re-read against the same 4169 target");
        }

        private static void CheckHipBudget(int to, string[] row, float dt, double ceiling)
        {
            if (to < 2548 || to > 2550) return;
            Check.That(row[29] == "Compared" && row[30] == "Matched", "original paired pelvis allocation");
            var budget = Height.Compare(dt,F(row[31]),F(row[32]),F(row[33]),F(row[34]));
            Check.That(budget.Valid && budget.Best == Height.Choice.Retention, "largest compared single-command gain");
            double needed = Math.Max(0,Height.Target*1000.0-ceiling), rem = budget.Remaining(ceiling*.001)*1000.0;
            if (to == 2550)
            {
                Check.Near((float)needed,.22542f,.0003f,"first strong minimum hip height shortfall mm");
                Check.Near((float)(budget.BestGain*1000.0),.099f,.0002f,"available in this matched incoming command mm");
                Check.Near((float)rem,.12642f,.0004f,"single command leaves a positive minimum shortfall mm");
                Check.Near(F(row[36]),-15.07825f,.00002f,"reference hip relative to animation mm");
                Check.Near(F(row[37]),-1.41728f,.00002f,"actual relative to reference mm");
            }
            if (to == 2549) Check.Near((float)rem,.292f,.001f,"preceding command also leaves a height deficit");
            if (to == 2548) Check.That(needed == 0.0 && budget.BestGain*1000.0 > .51 && rem == 0.0, "earlier room belongs only to its own solve");
        }

        private static bool CheckFirst(int start, int strong, int first, Search search, TurnHandoffWindow window, Trace.Tick tick)
        {
            var pathAtFirst = search.LastRead;
            Check.That(window.End == "FirstOpportunity" && search.EndLocal == first && search.LastReadLocal == first &&
                pathAtFirst.Entry.StrongSamples == 4, "all windows reach the native first, including the depth-latched trial");
            var text = new StringBuilder();
            window.AppendCombined(text,tick,"HoldEnded",0,strong,null);
            string log = text.ToString();
            Check.That(log.Contains("/readGate=") && log.Contains("/preShoeLossMm=") && log.Contains("/postShoeLossMm=") &&
                log.Contains("/lowerReadLossXZ-alongMm="), "separate fresh candidate receipts are printable");
            if (start == 4159)
            {
                Check.That(pathAtFirst.MarginSamples == 4 && pathAtFirst.FirstLowerExcess != 0 && pathAtFirst.FirstPreExcess != 0 &&
                    log.Contains("/retainedBufferedShoe-preAlso-lowerAlso-bothAlso=4|0|0|0"), "four shoe-only certificates, no retained whole-chain certificate");
                Check.Near(1000*pathAtFirst.Move.Settled.Sole,-1.749f,.002f,"first sole mm");
                Check.Near(1000*pathAtFirst.Move.Settled.Gap,3.510f,.002f,"first ankle gap mm");
            }
            if (start == 3339 || start == 5355)
                Check.That(log.Contains("/retainedBufferedShoe-preAlso-lowerAlso-bothAlso=4|4|4|4"), "every retained shoe on the original successes keeps both other tracking checks");
            if (start == 2540)
                Check.That(log.Contains("/retainedBufferedShoe-preAlso-lowerAlso-bothAlso=0|0|0|0"), "complete failed release yields a measured zero");
            bool ok = start != 2540;
            Check.That(search.Outcome == (ok ? "WouldEnter" : "DepthLatched") && pathAtFirst.MarginSamples == (ok ? 4 : 1) &&
                log.Contains("/allMarginsAtNativeFirst=" + ok), "original physical decision and continuous shoe-only result");
            return ok;
        }

        private static void SetClock(ref Trace.Tick tick, int shared, int source, float dt)
        {
            tick.Step.Sequence = shared; tick.Step.SourceSequence = source; tick.Step.Dt = dt;
            tick.Step.FixedTime = tick.Step.UnscaledTime = shared*dt;
            tick.Source.Clock.Sequence = source; tick.Source.Clock.Time = source*dt;
        }

        private static float Max(Search.ShoeLoss d) =>
            Math.Max(Math.Max(d.AnkleXZ,d.SoleXZ),Math.Max(d.AnkleAlong,d.SoleAlong));

        private static void HeightGeometry()
        {
            Check.That(double.IsNaN(Height.Ceiling(default)), "missing geometry has no numeric ceiling");
            for (int i = 0; i < 12; i++)
            {
                var p = TurnHandoffContracts.Pose(-.005f);
                p.Axis = new Vector3((i%3)-1, (i%5)-1.5f, 1f).normalized;
                p.Rotation = Quaternion.AngleAxis(17f*i,new Vector3(1f,.5f,-.7f).normalized);
                double cap = Height.Ceiling(p), sampled = double.NegativeInfinity;
                var aim = new Leg.Aim { Target = p.Foot, TargetSole = p.Foot, Travel = Vector3.forward };
                for (int h = -20; h <= 20; h++)
                for (int k = -20; k <= 20; k++)
                {
                    var angle = new Leg.Offset { Hip = h*.25f*Mathf.Deg2Rad, Knee = k*.25f*Mathf.Deg2Rad };
                    var m = Leg.Evaluate(p,angle,aim);
                    double relaxed = m.Sole - (p.Foot.y-p.Lower.y) + Leg.GapLimit-Foot.LandingMargin;
                    sampled = Math.Max(sampled,relaxed);
                }
                Check.That(cap >= sampled-3e-7 && cap-sampled < .0004, "tight upper bound over a grid independent of the analytic branches");
                var dy = Vector3.up*.003f;
                p.Hip += dy; p.UpperKnee += dy; p.LowerKnee += dy; p.Lower += dy; p.Foot += dy;
                Check.Near((float)(Height.Ceiling(p)-cap),.003f,3e-7f,"common vertical translation moves the full box equally");
            }
        }

        private static void HeightChoices()
        {
            var tie = Height.Compare(.02f,-.1f,-.08f,-.08f,-.12f);
            Check.That(tie.Valid && tie.Best == Height.Choice.Request, "request wins equal prediction without depending on measured response");
            Check.Near((float)tie.BestGain,.0004f,1e-8f,"one-step equivalent displacement");
            Check.Near((float)tie.NativeResidual,.0004f,1e-8f,"native residual is separate");
            var native = Height.Compare(.02f,-.1f,-.2f,-.12f,-.1f);
            Check.That(native.Valid && native.Best == Height.Choice.Native && native.BestGain == 0 &&
                native.RequestGain < 0 && native.RetentionGain < 0, "a slower alternative does not invent room");
            Check.That(!Height.Compare(.02f,.1f,float.NaN,0f,.1f).Valid &&
                !Height.Compare(0f,.1f,0f,0f,.1f).Valid, "unavailable allocation and invalid dt never contribute zero gain as evidence");
        }
    }
}
