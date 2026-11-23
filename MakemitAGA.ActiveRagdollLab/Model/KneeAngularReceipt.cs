using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Captured immediately after Lower receives its torque. A later Hip allocation
    // changes Upper's wrench but cannot retroactively change this transport component.
    internal struct KneeAngularReceipt
    {
        private readonly Vector3 _hipAtIssue;
        private readonly bool _captured, _transport;
        internal bool HipReallocated;
        internal Vector3 Parent => _transport ? _hipAtIssue : Vector3.zero;

        internal KneeAngularReceipt(Vector3 hipAtIssue, bool transport)
        {
            _hipAtIssue = hipAtIssue;
            _transport = transport;
            _captured = true;
            HipReallocated = false;
        }

        internal bool Matches(bool transport, Vector3 finalHip, Vector3 appliedHip, Vector3 relative, Vector3 total) =>
            _captured && _transport == transport && HipPointPairMobility.V(_hipAtIssue) &&
            RightThighTracking.Same(finalHip, appliedHip) &&
            (HipReallocated || RightThighTracking.Same(finalHip, _hipAtIssue)) &&
            RightThighTracking.Same(total, Parent + relative);
    }
}
