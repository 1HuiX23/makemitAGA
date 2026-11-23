using System;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using MakemitAGA.ActiveRagdollLab.Configuration;

namespace ARL.Contracts
{
    internal static class Maintenance170Contracts
    {
        internal static void Run()
        {
            Check.Run("170 full template publication preserves concurrent user edits and cleans failed drafts", ProfilePublication);
            Check.Run("170 missing preceding render preserves the signed peak without claiming a new lead", MissingBaseline);
        }

        private static void ProfilePublication()
        {
            string dir = Path.Combine(Path.GetTempPath(), "ARL_Profile_" + Guid.NewGuid().ToString("N"));
            string path = Path.Combine(dir, "profile.txt");
            try
            {
                string contents = "中文\r\nline2\r\n";
                Check.That(ProfileTemplateFile.CreateIfMissing(path, () => contents), "publish complete template");
                Check.That(File.ReadAllText(path) == contents, "encoded template contents");
                Check.That(!ProfileTemplateFile.CreateIfMissing(path, () => throw new Exception("unexpected")), "pre-existing file is untouched");
                File.Delete(path);
                Check.Throws<InvalidOperationException>(() => ProfileTemplateFile.CreateIfMissing(path, () => throw new InvalidOperationException("template")), "generation failure");
                Check.That(!Directory.EnumerateFileSystemEntries(dir).Any(), "no empty or partial destination, no temp file");

                using var waiting = new ManualResetEventSlim();
                using var resume = new ManualResetEventSlim();
                var contender = Task.Run(() => ProfileTemplateFile.CreateIfMissing(path, () =>
                {
                    waiting.Set();
                    if (!resume.Wait(TimeSpan.FromSeconds(10))) throw new TimeoutException();
                    return contents;
                }));
                try
                {
                    Check.That(waiting.Wait(TimeSpan.FromSeconds(10)), "in-progress template");
                    Check.That(!File.Exists(path), "reader never sees unfinished template");
                    File.WriteAllText(path, "user settings");
                }
                finally { resume.Set(); }
                Check.That(!contender.GetAwaiter().GetResult() && File.ReadAllText(path) == "user settings", "late creator cannot overwrite user settings");
                Check.That(Directory.EnumerateFiles(dir).Count() == 1, "loser's temp removed");
            }
            finally { if (Directory.Exists(dir)) Directory.Delete(dir, true); }
        }

        private static void MissingBaseline()
        {
            var s = new FootTransitionContracts.Sim();
            s.Frame(0,0,0); s.Frame(.02f,.02f,.02f);
            s.RenderOnly(.04f,.04f,.04f, visible:false);
            s.RenderOnly(.04f,.04f,.04f,.010f,.010f);
            s.RenderOnly(.04f,.04f,.04f,.020f,.020f);
            s.Frame(.04f,.04f,.04f);
            string line = s.Report().Split('\n').Single(l => l.StartsWith("FOOT_TRANSITION_EVENT_V041680/side=LEFT/kind=Brake/"));
            Check.That(line.Contains("renderAnkleLead-closure-soleLead-closure=UnknownBaseline|"), "unknown previous render");
            Check.That(line.Contains("renderAnkle-base-quiet-peakMm-sourceAt-returnSource-proxyDelta-targetDeltaMm=NA|10|20|"), "signed quiet/peak retained");
        }
    }
}
