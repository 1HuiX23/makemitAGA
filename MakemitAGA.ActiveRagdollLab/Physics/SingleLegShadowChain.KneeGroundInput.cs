using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        private KneeFootGroundInput CaptureKneeFootGroundInput()
        {
            if (!DynamicParentActive || _dynamicJoinPending || !_kneeReference.Selected ||
                !PelvisRunScopes.HasHipCoupling(_dynamicScope) || !_physicalParent.Matches(_runtimeStep) ||
                _pelvisBody == null || _upperBody == null || _lowerBody == null) return default;
            try
            {
                Vector3 parentScale = _pelvisBody.transform.lossyScale, upperScale = _upperBody.transform.lossyScale;
                if (!IsFinite(parentScale) || !IsFinite(upperScale) || parentScale.x <= 0f || parentScale.y <= 0f ||
                    parentScale.z <= 0f || upperScale.x <= 0f || upperScale.y <= 0f || upperScale.z <= 0f) return default;
                return new KneeFootGroundInput
                {
                    Source = _runtimeSource, CompletedStep = _runtimeStep,
                    Enabled = true, Apply = _kneeGroundTargetSelected,
                    ParentId = _pelvisBody.GetInstanceID(), UpperId = _upperBody.GetInstanceID(), LowerId = _lowerBody.GetInstanceID(),
                    Hip = _pelvisBody.position + _pelvisBody.rotation * Vector3.Scale(_rawHipConnectedAnchorLocal, parentScale),
                    // Native readback from this Render snapshot; do not reuse a trace
                    // Hip target or pretend last Fixed's command is a new source pose.
                    Upper = _upperBody.rotation, UpperSpan = Vector3.Scale(_kneeParentAnchorUpperLocal, upperScale),
                    LowerKneeAnchor = _kneeChildAnchorLowerLocal,
                    RawKnee = _sampledTargetKneeRelative, Axis = _kneeFlexionAxisLowerLocal
                };
            }
            catch { return default; } // Optional input cannot grant contact or replace native safety checks.
        }
    }
}
