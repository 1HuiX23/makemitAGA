using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.3 Foot-only collision callback bridge (v0.4.1 identity gate + downstream support-normal mechanical reaction observer).
    /// This component is intentionally separate from RagdollProxyContactTracker so Stage-E Foot callback evidence
    /// cannot be mixed into the legacy Pelvis/Chest contact counters.  It forwards the Collision object without interpreting it. v0.4.1 identity validation runs first; only after that gate accepts the exact authorized pair may the downstream v0.4.3 observer read direct Collision.impulse/relativeVelocity and Rigidbody response witnesses.
    /// </summary>
    public sealed class FootContactShadowTracker : MonoBehaviour
    {
        private string _side;
        private bool _initialized;

        public FootContactShadowTracker(IntPtr pointer)
            : base(pointer)
        {
        }

        public void Initialize(string side)
        {
            _side = string.IsNullOrWhiteSpace(side) ? "UNKNOWN" : side;
            _initialized = true;
        }

        public void OnCollisionEnter(Collision collision)
        {
            if (_initialized)
                ActiveRagdollLabRuntime.NotifyFootContactShadow(_side, "ENTER", collision);
        }

        public void OnCollisionStay(Collision collision)
        {
            if (_initialized)
                ActiveRagdollLabRuntime.NotifyFootContactShadow(_side, "STAY", collision);
        }

        public void OnCollisionExit(Collision collision)
        {
            if (_initialized)
                ActiveRagdollLabRuntime.NotifyFootContactShadow(_side, "EXIT", collision);
        }
    }
}
