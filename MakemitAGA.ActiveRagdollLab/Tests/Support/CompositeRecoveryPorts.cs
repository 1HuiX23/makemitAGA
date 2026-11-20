// Data-only boundary types for the linked real composite recovery owner.
namespace MakemitAGA.ActiveRagdollLab.Model { internal sealed partial class SingleLegChainReport {
        public int TraceKneeClampSamples;
        public float TraceKneeRotationMeanDegrees, TraceKneeRotationMaxDegrees;
        public string HipCompositeReferenceShockAbsorberMode;
        public string HipCompositeReferenceShockAbsorberArm;
        public bool HipCompositeReferenceShockAbsorberEnabled;
        public bool HipCompositeReferenceShockAbsorberActive;
        public int HipCompositeReferenceShockAbsorberFixedSteps;
        public int HipCompositeReferenceShockAbsorberSourceSteps;
        public int HipCompositeReferenceShockAbsorberRecoverySteps;
        public int HipCompositeReferenceShockAbsorberActivations;
        public int HipCompositeReferenceShockAbsorberReleases;
        public int HipCompositeReferenceShockAbsorberSpeedLimitedSteps;
        public int HipCompositeReferenceShockAbsorberAccelerationLimitedSteps;
        public int HipCompositeReferenceShockAbsorberInheritedOverSpeedSteps;
        public int HipCompositeReferenceShockAbsorberInputSpikeSteps;
        public float HipCompositeReferenceShockAbsorberCurrentInputGapDegrees;
        public float HipCompositeReferenceShockAbsorberCurrentInputStepDegrees;
        public float HipCompositeReferenceShockAbsorberCurrentInputAngularSpeed;
        public float HipCompositeReferenceShockAbsorberCurrentInputAngularAcceleration;
        public float HipCompositeReferenceShockAbsorberCurrentOutputStepDegrees;
        public float HipCompositeReferenceShockAbsorberCurrentOutputAngularSpeed;
        public float HipCompositeReferenceShockAbsorberCurrentOutputAngularAcceleration;
        public float HipCompositeReferenceShockAbsorberCurrentMaxAngularSpeed;
        public float HipCompositeReferenceShockAbsorberCurrentMaxAngularAcceleration;
        public float HipCompositeReferenceShockAbsorberCurrentLocalCompensationDegrees;
        public bool HipCompositeReferenceShockAbsorberCurrentUpstreamReferenceActive;
        public float HipCompositeReferenceShockAbsorberPeakInputGapDegrees;
        public float HipCompositeReferenceShockAbsorberPeakInputStepDegrees;
        public float HipCompositeReferenceShockAbsorberPeakInputAngularSpeed;
        public float HipCompositeReferenceShockAbsorberPeakInputAngularAcceleration;
        public float HipCompositeReferenceShockAbsorberPeakOutputStepDegrees;
        public float HipCompositeReferenceShockAbsorberPeakOutputAngularSpeed;
        public float HipCompositeReferenceShockAbsorberPeakOutputAngularAcceleration;
        public float HipCompositeReferenceShockAbsorberPeakOutputAccelerationRatio;
        public float HipCompositeReferenceShockAbsorberPeakLocalCompensationDegrees;
        public bool HipCompositeReferenceShockAbsorberSpeedContractSatisfied;
        public bool HipCompositeReferenceShockAbsorberAccelerationContractSatisfied;
        public string HipCompositeReferenceShockAbsorberLastEvidence;
        public string HipCompositeReferenceShockAbsorberAbortEvidence;
        public string HipCompositeReferenceShockAbsorberKey;
} }
namespace MakemitAGA.ActiveRagdollLab.Configuration { internal sealed class ActiveRagdollLabProfile {
public float LegHipControlFrameRecoveryExitAngleDegrees = 1f;
public float LegHipControlFrameRecoveryExitAngularSpeed = 1f;
public float LegHipControlFrameRecoveryMaxAngularAcceleration = 1f;
public float LegHipControlFrameRecoveryMaxAngularSpeed = 1f;
public float LegHipControlFrameRecoveryStableHoldSeconds = 1f;
public float LegHipControlFrameSourceMaxAngularAcceleration = 1f;
public float LegHipControlFrameSourceMaxAngularSpeed = 1f;
} }
