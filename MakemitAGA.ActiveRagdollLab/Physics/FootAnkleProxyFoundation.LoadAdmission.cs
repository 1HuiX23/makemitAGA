using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private readonly StanceLoadAdmission _loadAdmission = new StanceLoadAdmission();
        internal bool StanceLoadAdmissionSelected => _loadAdmission.Selected;
        private bool BaseAuthorizedStanceLoadEligible => _contactShadow.Active &&
            _report.ContactShadowCurrentContactActive && CurrentAuthorizedSupportNormalValid() && _stanceContactFrictionIsolation.ZeroFriction;

        private bool UpdateStanceLoadAdmission(PhysicsStepContext step, int local, float anchorGap, float sourceAge)
        {
            bool eligible = BaseAuthorizedStanceLoadEligible;
            int support = _contactShadow.AuthorizedColliderId;
            FootStanceOnlyAuthorizedContactLifecycle.EntryGeometry geometry = default;
            // Only pending B admission needs geometry. Reuse the existing eight-corner
            // primitive, with current body motion and the owned support; no raycast,
            // reference-path reconstruction, source Transform read or new observer.
            if (eligible && _loadAdmission.NeedsGeometry(support))
            {
                var context = new FootStanceOnlyAuthorizedContactLifecycle.EntryContext
                {
                    FixedSequence = local, Center = _body.worldCenterOfMass,
                    Velocity = _body.velocity, AngularVelocity = _body.angularVelocity,
                    AnchorGap = anchorGap, SourceAge = sourceAge
                };
                context.MotionValid = IsFinite(context.Center) && IsFinite(context.Velocity) && IsFinite(context.AngularVelocity);
                geometry = FootStanceOnlyAuthorizedContactLifecycle.CaptureEntryGeometry(
                    _collider, _supportSurface, support, context, step.Dt);
            }
            if (_loadAdmission.Step(_traceEnabled, step, local, support, eligible, geometry)) return true;
            _report.Finite = false; _report.Detail = _loadAdmission.Issue;
            return false;
        }
    }
}
