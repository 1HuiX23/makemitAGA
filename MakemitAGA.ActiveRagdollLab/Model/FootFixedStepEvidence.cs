using System.Globalization;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    /// <summary>Read-only pre-lifecycle sample. No controller or eligibility rule consumes these fields.</summary>
    internal struct FootFixedStepEvidence
    {
        public int FixedSequence;
        public bool Valid;
        // V140 optional diagnostic position; never changes the original Valid gate.
        internal bool PreparationPoseValid;
        internal int PreparationSupportId;
        internal Vector3 FootAnchorWorld;
        internal Vector3 LowerAnchorWorld; // V141 copies the same pre-lifecycle ankle readback.
        public bool ContactActiveBeforeLifecycle;
        public float AnchorErrorMeters;
        public float TrackingErrorDegrees;
        public Vector3 LowerAnkleVelocityWorld;
        public Vector3 FootAnkleVelocityWorld;
        public Vector3 LowerAngularVelocityWorld;
        public Vector3 FootAngularVelocityWorld;

        public string GetSummary()
        {
            return "fixed=" + FixedSequence + "/valid=" + Valid + "/contactBefore=" + ContactActiveBeforeLifecycle +
                   "/anchorMm=" + F(AnchorErrorMeters * 1000f) + "/trackingDeg=" + F(TrackingErrorDegrees) +
                   "/ankleV(lower/foot)=" + V(LowerAnkleVelocityWorld) + "/" + V(FootAnkleVelocityWorld) +
                   "/W(lower/foot)=" + V(LowerAngularVelocityWorld) + "/" + V(FootAngularVelocityWorld);
        }

        private static string F(float value) => value.ToString("0.######", CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
