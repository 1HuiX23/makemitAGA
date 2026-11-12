using System;
using System.IO;
using System.Text.RegularExpressions;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class LoadTraceContracts
    {
        internal static void Run()
        {
            Check.Run("real Knee trace reset preserves load receipt, duplicate guard and fault", KneeReset);
            Check.Run("load trace-reset source boundaries exclude live state writes", SourceBoundaries);
        }
        private static void KneeReset()
        {
            foreach (bool b in new[] { false, true })
            {
                var r = new KneeSupportNormalReactionIsolation(); r.Select(b);
                var upper = new Rigidbody { mass = 2f }; var lower = new Rigidbody { mass = 3f };
                Check.That(r.Step(true, 11, 11, true, true, true, upper, lower, Vector3.up, Vector3.up, 1f/60f, true), "real reaction produced");
                Vector3 force = r.Report.CurrentUpperForceWorld;
                r.ResetTrace();
                Check.That(r.Report.CurrentFixed == 11 && r.Report.CurrentSourceFixed == 11 && r.Report.CurrentApplied == b &&
                    r.Report.CurrentEligible && r.Report.CurrentNormalConstraint && r.Report.CurrentLowerMass == 3f &&
                    PelvisChestPointResponse.Same(r.Report.CurrentUpperForceWorld, force), "trace reset retains the actual command");
                Check.That(r.Report.Samples == 0 && r.Report.Writes == 0 && upper.ForceCalls == (b ? 1 : 0), "statistics clear without new native writes");
                Check.That(!r.Step(true, 11, 11, true, true, true, upper, lower, Vector3.up, Vector3.up, 1f/60f, true),
                    "trace reset cannot allow a duplicate physical command");
                string fault = r.Report.InvalidReason; r.ResetTrace();
                Check.That(!r.Finite && r.Report.InvalidReason == fault, "trace reset cannot heal a live fault");
                r.Select(b); Check.That(r.Finite && r.Report.CurrentFixed == 0, "explicit lifecycle resets fault and clock");
                Check.That(r.Step(true, 1, 1, true, true, true, upper, lower, Vector3.up, Vector3.up, 1f/60f, true), "new lifecycle can restart at one");
            }
        }

        // Architectural guard for the other native owners. Inspect the actual source
        // method body (not a copied implementation). This does not execute native ports
        // or certify the whole class; the linked Knee runtime check above covers behavior.
        private static void SourceBoundaries()
        {
            string root = FindRoot(Directory.GetCurrentDirectory()) ?? FindRoot(AppContext.BaseDirectory);
            Check.That(root != null, "run from the source tree, or keep the test output under its source tree");
            string[] owners = { "KneeSupportNormalReactionIsolation", "LowerStanceNativeGravitySupportLoadIsolation",
                "FootStanceNativeGravitySupportLoadIsolation", "LowerSelfWeightSupportNormalLoadTransferIsolation",
                "FootLowerSupportNormalReciprocalLoadTransferIsolation", "UpperSelfWeightSupportNormalLoadTransferIsolation" };
            foreach (string owner in owners)
            {
                string source = File.ReadAllText(Path.Combine(root, "Physics", owner + ".cs"));
                // Strip comments and literals before counting braces or checking writes.
                string code = Regex.Replace(source, "@\"(?:\"\"|[^\"])*\"|\"(?:\\\\.|[^\"\\\\])*\"|//[^\\r\\n]*|/\\*[\\s\\S]*?\\*/", " ");
                var match = Regex.Match(code, @"\bvoid\s+ResetTrace\s*\([^)]*\)\s*\{");
                Check.That(match.Success, owner + " trace entry present");
                int start = match.Index + match.Length, end = start, depth = 1;
                for (; end < code.Length && depth > 0; end++) { if (code[end] == '{') depth++; else if (code[end] == '}') depth--; }
                Check.That(depth == 0, owner + " trace body bounded");
                string body = code.Substring(start, end - start - 1);
                Check.That(!Regex.IsMatch(body, @"\b(?:Quiesce|ClearCurrent)\s*\("), owner + " cannot invoke live clear from trace reset");
                Check.That(!Regex.IsMatch(body, @"\b(?:_current\w*|Current\w*|_finite|_invalidReason|Finite|InvalidReason|_lastScheduledFixed)\s*=(?!=)"),
                    owner + " cannot overwrite a load receipt, clock or fault from trace reset");
            }
        }
        private static string FindRoot(string path)
        {
            for (var p = new DirectoryInfo(path); p != null; p = p.Parent)
                if (File.Exists(Path.Combine(p.FullName, "MakemitAGA.ActiveRagdollLab.csproj"))) return p.FullName;
            return null;
        }
    }
}
