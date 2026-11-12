using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class BilateralLegCoordinator
    {
        private readonly HipJoinPreparationWindow _hipJoinWindow = new HipJoinPreparationWindow();
        private readonly HipContactPreparationWindow _hipContactWindow = new HipContactPreparationWindow();
        private int _hipPreparationSupportId;
        private Vector3 _hipPreparationCenter, _hipPreparationHalf;
        internal bool HipJoinPreparationPending => _hipJoinWindow.Pending;
        internal string HipPreparationDependency => _hipJoinWindow.Pending ? "STABLE_HIP_JOIN_GEOMETRY" :
            _hipContactWindow.Pending ? "SETTLED_HIP_CONTACT_PREPARATION" : null;
        internal string HipJoinPreparationSummary() => _hipJoinWindow.Summary() + "\n" + _hipContactWindow.Summary();
        internal void CancelHipJoinPreparation(string reason)
        { _hipJoinWindow.Cancel(reason); _hipContactWindow.Cancel(reason); }
        private void ResetHipPreparation()
        {
            _hipJoinWindow.Reset(); _hipContactWindow.Reset(); _hipPreparationSupportId = 0;
            _hipPreparationCenter = _hipPreparationHalf = Vector3.zero;
        }
        internal bool AdvanceHipJoinPreparation(PhysicsStepContext step)
        {
            if (!DynamicJoinReady || !_contextCursor.Fixed.Matches(step) ||
                !_torso.TryGetLegParentCommand(step, out LegParentCommandFrame command) || !command.PhysicalParent.Matches(step))
                throw new RunPreparationRejectedException("HIP_JOIN_PREVIEW_CONTRACT");
            if (_hipJoinWindow.Reason == "NOT_REQUESTED")
            { _hipJoinWindow.Begin(step); _initialGeometryStatus = "WAITING_STABLE_JOIN"; }
            HipJoinPose left = default, right = default;
            _primary.RefreshFootSupportProbe(); _secondary.RefreshFootSupportProbe();
            bool support = FootSupportSurfaceIsolation.TryPreviewEntryBox(_primary.CurrentFootGround, _secondary.CurrentFootGround,
                _primary.FootContactShadowCollider, _secondary.FootContactShadowCollider, PrimaryFootReport, SecondaryFootReport,
                out int id, out Vector3 center, out Vector3 half, out string reason);
            if (support)
            {
                left = _primary.ProjectDynamicJoin(step, command.PhysicalParent, center, half);
                right = _secondary.ProjectDynamicJoin(step, command.PhysicalParent, center, half);
            }
            _hipJoinWindow.Observe(step, _contextCursor.Source, id, center, half, left, right, support ? null : reason);
            if (_hipJoinWindow.QualifiedFor(step))
            {
                _hipPreparationSupportId = id; _hipPreparationCenter = center; _hipPreparationHalf = half;
                return true;
            }
            if (!_hipJoinWindow.Pending)
            {
                _initialGeometryStatus = "REJECTED";
                throw new RunPreparationRejectedException("HIP_JOIN_PREPARATION:" + _hipJoinWindow.Reason);
            }
            return false;
        }

        internal void BeginHipContactPreparation(PhysicsStepContext step)
        {
            if (!_hipJoinWindow.QualifiedFor(step) || !DynamicParentsActive || !FreeSpaceReady || TraceEnabled)
                throw new RunPreparationRejectedException("HIP_CONTACT_PREPARATION_JOIN_CONTRACT");
            // Read the actual initializer writes once before any free-space solve.
            var left = _primary.CaptureInitialGeometry(step, _hipPreparationCenter, _hipPreparationHalf);
            var right = _secondary.CaptureInitialGeometry(step, _hipPreparationCenter, _hipPreparationHalf);
            Plugin.Logger?.LogInfo("[ARL][HIP_JOIN_NATIVE_CHECK] domain=CONTACT_AND_LOAD_OFF | " +
                left.Summary("PRIMARY") + "\n" + right.Summary("SECONDARY"));
            if (left.PreparationIssue != null || right.PreparationIssue != null)
                throw new RunPreparationRejectedException("HIP_JOIN_NATIVE_GEOMETRY:" +
                    (left.PreparationIssue ?? "PASS") + ";" + (right.PreparationIssue ?? "PASS"));
            _initialGeometryStatus = "WAITING_SETTLED_NATIVE_PREPARATION";
            _hipContactWindow.Begin(step);
        }

        internal bool AdvanceHipContactPreparation(PhysicsStepContext step)
        {
            if (!DynamicParentsActive || !FreeSpaceReady || TraceEnabled || !_contextCursor.Fixed.Matches(step))
                throw new RunPreparationRejectedException("HIP_CONTACT_PREPARATION_NATIVE_CONTRACT");
            bool support = FootSupportSurfaceIsolation.TryPreviewEntryBox(_primary.CurrentFootGround, _secondary.CurrentFootGround,
                _primary.FootContactShadowCollider, _secondary.FootContactShadowCollider, PrimaryFootReport, SecondaryFootReport,
                out int id, out Vector3 center, out Vector3 half, out string reason);
            if (support && (id != _hipPreparationSupportId || !_hipJoinWindow.MatchesSupport(id, center, half)))
            {
                _initialGeometryStatus = "REJECTED";
                _hipContactWindow.Cancel("SUPPORT_CHANGED_DURING_PREPARATION");
                throw new RunPreparationRejectedException("HIP_CONTACT_PREPARATION:SUPPORT_CHANGED_DURING_PREPARATION");
            }
            var left = support ? _primary.CaptureContactPreparation(step, id, center, half) : default;
            var right = support ? _secondary.CaptureContactPreparation(step, id, center, half) : default;
            _hipContactWindow.Observe(step, _contextCursor.Source, left, right, support ? null : reason);
            if (_hipContactWindow.QualifiedFor(step)) return true;
            if (!_hipContactWindow.Pending)
            {
                _initialGeometryStatus = "REJECTED";
                throw new RunPreparationRejectedException("HIP_CONTACT_PREPARATION:" + _hipContactWindow.Reason);
            }
            return false;
        }
    }
}
