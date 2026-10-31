using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Exact cached goals at their existing call sites; no new estimator or native getter.
    // All poses below are world rotations. RawInControl changes ONLY the parent basis;
    // each later pose adds its named existing stage. Angles between stages are not additive.
    internal struct HipGoalPathEvidence
    {
        internal PhysicsStepContext Step;
        internal SourceStepContext Source;
        internal int Fixed, Modes;
        internal bool Prepared, Committed, NormalApplied;
        internal Quaternion RawUpper, PhysicalParent, ControlParent, RawInControl;
        internal Quaternion AfterSource, AfterGovernor, AfterComposite, Final, PhysicalUpper;
        internal bool Valid => Prepared && Committed && Step.Valid && Fixed > 0 && Source.Valid &&
            Source.Epoch == Step.Epoch && Source.Sequence == Step.SourceSequence &&
            Q(RawUpper) && Q(PhysicalParent) && Q(ControlParent) && Q(RawInControl) && Q(AfterSource) &&
            Q(AfterGovernor) && Q(AfterComposite) && Q(Final) && Q(PhysicalUpper);
        internal string Details() => !Valid ? "NA_NOT_CAPTURED_OR_INVALID" :
            "READ_ONLY_WORLD_R/runtime=" + Step.Status() + "/local-source-modes-normal=" + Fixed + "|" + Source.Sequence + "|" + Modes + "|" + NormalApplied +
            "/rawUpper-physicalParent-controlParent=" + R(RawUpper) + "|" + R(PhysicalParent) + "|" + R(ControlParent) +
            "/rawInControl-afterSource-afterGovernor-afterComposite-final-physicalUpper=" + R(RawInControl) + "|" + R(AfterSource) + "|" +
            R(AfterGovernor) + "|" + R(AfterComposite) + "|" + R(Final) + "|" + R(PhysicalUpper);
        private static bool Q(Quaternion q) => PelvisParentFrame.Q(q);
        private static string N(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
        private static string R(Quaternion q) => "(" + N(q.x) + "," + N(q.y) + "," + N(q.z) + "," + N(q.w) + ")";
    }
}
