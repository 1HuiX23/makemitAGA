using System;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    public enum ProbePreference
    {
        Auto,
        Cpu,
        MeshData,
        Bake,
        Gpu,
        Collider
    }

    public static class ProbePreferenceParser
    {
        public static bool TryParse(string text, out ProbePreference value)
        {
            value = ProbePreference.Auto;
            string normalized = (text ?? string.Empty).Trim().ToLowerInvariant();
            switch (normalized)
            {
                case "":
                case "auto": value = ProbePreference.Auto; return true;
                case "cpu": value = ProbePreference.Cpu; return true;
                case "meshdata": value = ProbePreference.MeshData; return true;
                case "bake": value = ProbePreference.Bake; return true;
                case "gpu": value = ProbePreference.Gpu; return true;
                case "collider": value = ProbePreference.Collider; return true;
                default: return false;
            }
        }
    }
}
