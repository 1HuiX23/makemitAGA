using System.Globalization;
using System.Text;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // V04460 reuses the Upper input ledger. No settled Pelvis momentum or native Hip reaction is inferred.
    internal sealed partial class HipPelvisLoadPathReport
    {
        internal bool Selected, Collecting, Finite = true, ReceiverCaptured;
        internal string Issue;
        internal int Samples, LeftLoaded, RightLoaded, LeftReleased, RightReleased, BothLoaded, NeitherLoaded;
        internal int LastFixed, AbortedStepsSkipped;
        internal Receiver Pelvis, Chest;
        internal struct Receiver
        {
            internal int BodyId;
            internal bool Kinematic, Gravity;
            internal float Mass;
            internal Vector3 ComLocal, Inertia;
            internal Quaternion InertiaRotation;
        }
        internal struct InputFrame
        {
            internal int Fixed;
            internal bool Idle, LeftLoaded, RightLoaded;
            internal float Dt;
            internal Vector3 PelvisCom, LeftUpperCom, RightUpperCom;
            internal Vector3 LeftHipJ, RightHipJ, LeftCommonJ, RightCommonJ, ComMirrorMomentJ;
            internal HipCommandWrench LeftWrench, RightWrench;
        }
        internal sealed class Bucket
        {
            internal int Samples;
            internal double Seconds, HipX, HipY, HipZ, CommonX, CommonY, CommonZ, HipAbsSum, IndividualAbsSum;
            internal float PeakHipForce, PeakIndividualHipForce, PeakComMirrorMoment;
            internal void Reset()
            {
                Samples = 0; Seconds = HipX = HipY = HipZ = CommonX = CommonY = CommonZ = HipAbsSum = IndividualAbsSum = 0d;
                PeakHipForce = PeakIndividualHipForce = PeakComMirrorMoment = 0f;
            }
            internal void Add(InputFrame f)
            {
                Vector3 hip = f.LeftHipJ + f.RightHipJ, common = f.LeftCommonJ + f.RightCommonJ;
                Samples++; Seconds += f.Dt; HipX += hip.x; HipY += hip.y; HipZ += hip.z;
                CommonX += common.x; CommonY += common.y; CommonZ += common.z; HipAbsSum += hip.magnitude;
                IndividualAbsSum += (double)f.LeftHipJ.magnitude + f.RightHipJ.magnitude;
                PeakHipForce = Mathf.Max(PeakHipForce, hip.magnitude / f.Dt);
                PeakIndividualHipForce = Mathf.Max(PeakIndividualHipForce, Mathf.Max(f.LeftHipJ.magnitude, f.RightHipJ.magnitude) / f.Dt);
                PeakComMirrorMoment = Mathf.Max(PeakComMirrorMoment, f.ComMirrorMomentJ.magnitude / f.Dt);
            }
            internal string Summary() => "samples-seconds=" + Samples + "-" + D(Seconds) +
                "/actualHipUpperSumNs=(" + D(HipX) + "," + D(HipY) + "," + D(HipZ) + ")" +
                "/externalUpperCommonSumNs=(" + D(CommonX) + "," + D(CommonY) + "," + D(CommonZ) + ")" +
                "/hipMeanForceN=(" + D(Seconds > 0d ? HipX / Seconds : 0d) + "," + D(Seconds > 0d ? HipY / Seconds : 0d) + "," + D(Seconds > 0d ? HipZ / Seconds : 0d) + ")" +
                "/hipAbsMean-peakForceN=" + D(Seconds > 0d ? HipAbsSum / Seconds : 0d) + "-" + F(PeakHipForce) +
                "/individualAbsSumMean-peakSideForceN=" + D(Seconds > 0d ? IndividualAbsSum / Seconds : 0d) + "-" + F(PeakIndividualHipForce) +
                "/unexecutedComMirrorPeakMomentNm=" + F(PeakComMirrorMoment);
        }
        internal readonly Bucket Idle = new Bucket(), Motion = new Bucket();
        internal readonly InputFrame[] Ring = new InputFrame[12];
        internal int Next, Count;
        internal bool EvidenceReady => (!DynamicMode || DynamicExposureReady) && Selected && Finite && ReceiverCaptured &&
            LeftLoaded >= 50 && RightLoaded >= 50 && LeftReleased >= 30 && RightReleased >= 30 &&
            Idle.Samples >= 50 && Motion.Samples >= 30;

        internal string GetSummary() => DynamicMode ? GetDynamicSummary() : "mode=READ_ONLY_HIP_PELVIS_LOAD_PATH_V04460/selected-collecting-finite=" +
            Selected + "-" + Collecting + "-" + Finite + "/issue=" + (Issue ?? "<none>") +
            "/receiverCaptured=" + ReceiverCaptured + "/inputEvidenceReady=" + EvidenceReady +
            "/samples-leftLoaded-rightLoaded-leftReleased-rightReleased-both-neither=" + Samples + "-" + LeftLoaded + "-" + RightLoaded + "-" + LeftReleased + "-" + RightReleased + "-" + BothLoaded + "-" + NeitherLoaded +
            "/lastFixed-abortedStepsSkipped=" + LastFixed + "-" + AbortedStepsSkipped +
            "/pelvis={" + ReceiverSummary(Pelvis) + "}/chest={" + ReceiverSummary(Chest) + "}" +
            "/nativeHipJoint=ABSENT_IN_VALIDATED_FROZEN_STACK/hipPelvisForceWritePath=NONE_IN_FROZEN_CONTROLLER" +
            "/hipControlParent=INERTIALIZED_MECHANICAL_PARENT_REFERENCE_NOT_DYNAMIC_RECEIVER" +
            "/pelvisPoseOwner=SOURCE_POSE_BRIDGE_MOVE_POSITION_ROTATION/chestJoint=CHARACTERJOINT_CONNECTED_TO_PELVIS" +
            "/dynamicReceiverQualified=False/nativeMassProperties=TRACE_START_READBACK_ONLY_NOT_MIGRATED" +
            "/idle={" + Idle.Summary() + "}/motion={" + Motion.Summary() + "}";

        internal string GetDetails()
        {
            if (DynamicMode) return GetDynamicDetails();
            var sb = new StringBuilder(GetSummary());
            for (int k = 0; k < Count; k++)
            {
                InputFrame f = Ring[(Next - Count + k + Ring.Length) % Ring.Length];
                Vector3 hip = f.LeftHipJ + f.RightHipJ;
                sb.Append("\nhipPelvisRow=").Append(f.Fixed).Append("/phase=").Append(f.Idle ? "IDLE" : "MOTION_OR_SOURCE_TURN")
                    .Append("/loaded(left-right)=").Append(f.LeftLoaded).Append('-').Append(f.RightLoaded).Append("/dt=").Append(F(f.Dt))
                    .Append("/preSolveCom(pelvis-leftUpper-rightUpper)=").Append(V(f.PelvisCom)).Append('|').Append(V(f.LeftUpperCom)).Append('|').Append(V(f.RightUpperCom))
                    .Append("/actualHipUpperJ(left-right-sum)=").Append(V(f.LeftHipJ)).Append('|').Append(V(f.RightHipJ)).Append('|').Append(V(hip))
                    .Append("/externalUpperCommonJ(left-right)=").Append(V(f.LeftCommonJ)).Append('|').Append(V(f.RightCommonJ))
                    .Append("/unexecutedPelvisHipComMirrorJ=").Append(V(-hip))
                    .Append("/unexecutedComMirrorOrbitalMomentJ=").Append(V(f.ComMirrorMomentJ));
            }
            return sb + "\nV04460 records committed pre-solve Hip/Upper common inputs; Common is external and is not included in the hypothetical Hip mirror. The COM mirror and its orbital moment are algebraic proposals, never applied or measured reactions. Native Chest joint effects, Hip angular torque, Lower common transport and whole-body balance are outside this ledger. Kinematic Pelvis mass/inertia readback does not qualify a dynamic receiver.";
        }
        private static string ReceiverSummary(Receiver r) => "bodyId=" + r.BodyId + "/kinematic-gravity=" + r.Kinematic + "-" + r.Gravity +
            "/mass=" + F(r.Mass) + "/localCOM=" + V(r.ComLocal) + "/inertia=" + V(r.Inertia) +
            "/inertiaRotation=(" + F(r.InertiaRotation.x) + "," + F(r.InertiaRotation.y) + "," + F(r.InertiaRotation.z) + "," + F(r.InertiaRotation.w) + ")";
        private static string F(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
        private static string D(double x) => x.ToString("G17", CultureInfo.InvariantCulture);
        private static string V(Vector3 x) => "(" + F(x.x) + "," + F(x.y) + "," + F(x.z) + ")";
    }
}
