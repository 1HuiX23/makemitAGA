using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootStanceOnlyAuthorizedContactLifecycle
    {
        internal readonly RightFootPrecontact Precontact = new RightFootPrecontact(LogEvent);
        internal bool AwaitingFormalCallback { get; private set; }
        internal void ObserveFormalCallback() => AwaitingFormalCallback = false;
        internal void ContactDisabled(Model.FootAnkleFoundationReport report)
        {
            Precontact.Close(RightFootPrecontact.End.Stop, 0);
            Precontact.BreakContinuity();
            Precontact.EndFormal();
            AwaitingFormalCallback = false;
            PrecontactEventOrigin = 0;
            if (report != null) report.PrecontactColliderActive = false;
        }

        private RightFootPrecontact.End _deferredClose;
        internal int PrecontactEventOrigin { get; private set; }

        private bool UpdatePrecontact(int local, string gait, Collider ground, FootQualifiedContactShadow shadow,
            BoxCollider foot, FootSupportSurfaceIsolation surface, Model.FootAnkleFoundationReport report,
            in EntryContext context, ref bool pending)
        {
            PrecontactEventOrigin = 0;
            _deferredClose = RightFootPrecontact.End.None;
            if (Precontact.Selected == RightFootPrecontact.Mode.Off) return true;
            if (shadow.Active && !Precontact.Physical)
            {
                Precontact.BreakContinuity();
                return true;
            }
            var g = _entryGeometry;
            var s = new RightFootPrecontact.Sample
            {
                Step = report.RuntimeContext, Source = report.RuntimeSourceContext, Local = local, Support = g.SupportId,
                ExactFloor = g.Valid && g.MotionValid && g.LowestMotionValid && g.FullFootprint && !g.DiagnosticSelectedBox &&
                    context.FixedSequence == local && context.FootAnchorYValid && surface != null && surface.MainRoomBox &&
                    ground != null && ground == surface.SelectedCollider && g.SupportId == surface.SelectedId &&
                    report.GroundProbeHitCount == 5 && report.GroundFootprintSameSurface && !report.GroundColliderHasAttachedRigidbody &&
                    report.GroundColliderInstanceId == g.SupportId && report.GroundFootprintMinNormalDotReferenceUp >= .90f &&
                    report.GroundFootprintMinNormalDotReferenceUp <= 1.0001f,
                Lift = gait == FootGameNativeLocomotionPhaseObserver.StateToSwing,
                Swing = gait == FootGameNativeLocomotionPhaseObserver.StateSwing,
                PhaseKnown = context.CurrentGait && (gait == FootGameNativeLocomotionPhaseObserver.StateToStance || gait == FootGameNativeLocomotionPhaseObserver.StateStance ||
                    gait == FootGameNativeLocomotionPhaseObserver.StateSwing || gait == FootGameNativeLocomotionPhaseObserver.StateToSwing),
                FloorCenter = g.FloorCenter, FloorHalf = g.FloorHalf, W = context.AngularVelocity,
                Min = g.MinClearance, Predicted = g.PredictedMinClearance, CornerV = g.LowestCornerNormalVelocity,
                AnkleY = context.FootAnchorY, Gap = g.AnchorGap, Age = g.SourceAge, Gait = gait,
                ControlMin = g.ReferencesValid ? g.ControlMinClearance : float.NaN
            };
            var end = Precontact.Evaluate(s, _entryWindowMissed, out pending);
            if (end == RightFootPrecontact.End.None) return true;
            if (end == RightFootPrecontact.End.Timeout || end == RightFootPrecontact.End.ExtensionLimit ||
                end == RightFootPrecontact.End.Rising || end == RightFootPrecontact.End.NearGroundExit)
            {
                // A valid formal handoff on this boundary needs no off/on cycle before the same solve.
                _deferredClose = end;
                return true;
            }
            if (!ClosePrecontact(end, local, shadow, report)) return false;
            if (end == RightFootPrecontact.End.Fault)
            {
                Invalidate("RIGHT_PRECONTACT_UNLOADED_RECEIPT_MISSING", report);
                return false;
            }
            return true;
        }

        private bool ClosePrecontact(RightFootPrecontact.End reason, int local, FootQualifiedContactShadow shadow,
            Model.FootAnkleFoundationReport report)
        {
            if (Precontact.Physical && !shadow.DisableColliderPreservePreparation(report, out string error))
            {
                Invalidate("RIGHT_PRECONTACT_RELEASE_FAILED:" + Clean(error), report);
                return false;
            }
            Precontact.Close(reason, local);
            return true;
        }

        private bool FinishPrecontact(bool pending, int local, Collider ground, FootQualifiedContactShadow shadow,
            BoxCollider foot, Transform sourceRoot, Model.FootAnkleFoundationReport report, FootStanceContactFrictionIsolation friction)
        {
            if (Precontact.Selected == RightFootPrecontact.Mode.Off) return true;
            if (!shadow.Active) Precontact.EndFormal();
            if (OrdinaryEnabledThisStep || RecoveryEnabledThisStep)
            {
                Precontact.BeginFormal(RecoveryEnabledThisStep);
                Precontact.Close(RightFootPrecontact.End.Handoff, local);
            }
            else if (FirstMissThisStep || _entryWindowMissed)
            {
                if (Precontact.Tracking && !ClosePrecontact(RightFootPrecontact.End.Miss, local, shadow, report)) return false;
            }
            else if (_deferredClose != RightFootPrecontact.End.None && !Precontact.TryKeepAtBoundary(_deferredClose))
            {
                if (!ClosePrecontact(_deferredClose, local, shadow, report)) return false;
            }
            else if (pending && !shadow.Active)
            {
                if (!ValidatePrecontactFriction(friction, ground, foot, report)) return false;
                if (!EnableSupport(ground.GetInstanceID(), ground, sourceRoot, shadow, foot, report)) return false;
                Precontact.Open();
            }
            if (Precontact.Tracking)
            {
                if (Precontact.Physical && !pending && !ValidatePrecontactFriction(friction, ground, foot, report)) return false;
                Precontact.CountSolve();
            }
            PrecontactEventOrigin = Precontact.LinkedOrigin;
            if (FirstMissThisStep || OrdinaryEnabledThisStep || RecoveryEnabledThisStep) Precontact.AfterFormalEvent();
            return true;
        }

        private bool ValidatePrecontactFriction(FootStanceContactFrictionIsolation friction, Collider ground, BoxCollider foot,
            Model.FootAnkleFoundationReport report)
        {
            string error = null;
            if (friction != null && friction.CheckPrecontactPair(foot, ground, report, out error)) return true;
            Invalidate("RIGHT_PRECONTACT_FRICTION:" + Clean(error), report);
            return false;
        }
    }
}
