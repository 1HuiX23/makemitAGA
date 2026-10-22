using System.Globalization;
using System.Text;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed class KneeSafetyContextReport
    {
        public string Mode;
        public bool TraceEnabled;
        public int Samples;
        public float PrimaryPeakAnchorMeters;
        public float SecondaryPeakAnchorMeters;
        public bool AbortCaptured;
        public string AbortSide;
        public string AbortReason;
        public int AbortFixedSequence;
        public float AbortAnchorMeters;
        public float SafetyLimitMeters;
        public string RingContext;
        public string Key;

        public string GetConsoleSummary()
        {
            return "ARL Knee safety context | mode=" + (Mode ?? "<none>") +
                   " | trace=" + TraceEnabled + " | samples=" + Samples +
                   " | peak(L/R)=" + F(PrimaryPeakAnchorMeters) + "/" + F(SecondaryPeakAnchorMeters) +
                   " | abort=" + AbortCaptured + "/" + (AbortSide ?? "<none>") +
                   " | fixed=" + AbortFixedSequence + " | anchor=" + F(AbortAnchorMeters) +
                   " | limit=" + F(SafetyLimitMeters);
        }

        public string GetFullText()
        {
            StringBuilder sb = new StringBuilder(4096);
            sb.AppendLine("MakemitAGA Active Ragdoll Lab 0.4.2.1 - Minimal Knee Hard-Safety Context Recorder");
            sb.AppendLine(GetConsoleSummary());
            sb.AppendLine("abortReason=" + (AbortReason ?? "<none>"));
            sb.AppendLine("key=" + (Key ?? "<none>"));
            sb.AppendLine("ring(last settled FixedSteps; diagnostic only):");
            sb.AppendLine(RingContext ?? "<none>");
            sb.AppendLine("NOTE: v0.3.10 frame-stall/debt correlation study is closed. This recorder keeps only a small fixed ring for naturally occurring future Knee hard-gate events. The 30mm gate remains hard fail-close in v0.3.14; no soft recovery behavior is implemented here.");
            return sb.ToString();
        }

        private static string F(float v) => float.IsNaN(v) || float.IsInfinity(v) ? "nan" : v.ToString("0.######", CultureInfo.InvariantCulture);
    }
}
