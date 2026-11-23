using System;
using System.IO;

namespace MakemitAGA.ActiveRagdollLab.Configuration
{
    internal static class ProfileTemplateFile
    {
        // Two game instances may share BepInEx/config. Publish only a completed
        // template and let a user file or another successful creator win the race.
        internal static bool CreateIfMissing(string path, Func<string> getTemplate)
        {
            if (File.Exists(path)) return false;
            string directory = Path.GetDirectoryName(Path.GetFullPath(path));
            Directory.CreateDirectory(directory);
            string temporary = Path.Combine(directory, Path.GetRandomFileName());
            bool ownsTemporary = false;
            try
            {
                using (var file = new FileStream(temporary, FileMode.CreateNew, FileAccess.Write, FileShare.None))
                {
                    ownsTemporary = true;
                    using var writer = new StreamWriter(file);
                    writer.Write(getTemplate());
                }
                try { File.Move(temporary, path); }
                catch (IOException) when (File.Exists(path)) { return false; }
                ownsTemporary = false;
                return true;
            }
            finally
            {
                if (ownsTemporary) File.Delete(temporary);
            }
        }
    }
}
