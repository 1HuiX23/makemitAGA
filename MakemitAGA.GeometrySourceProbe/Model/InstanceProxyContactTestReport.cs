using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    internal sealed class InstanceProxyContactCaseReport
    {
        public int Sequence;
        public int ProxyIndex;
        public string InstanceKey;
        public string SampleKind;
        public bool Started;
        public bool Completed;
        public bool Passed;
        public bool TimedOut;
        public float ElapsedSeconds;

        public Vector3 SpawnPosition;
        public Vector3 InitialVelocity;
        public Vector3 ExpectedSurfacePoint;
        public Vector3 ExpectedNormal;
        public Vector3 ProbeCenterAtEvent;

        // 这些“接触点/法线”由托管 OBB 最近表面算法计算，不来自
        // Collision.GetContact / ContactPoint，避免 MiSide IL2CPP 桥接异常。
        public Vector3 ContactPoint;
        public Vector3 ContactNormal;
        public bool CollisionEventObserved;
        public bool TargetColliderMatched;
        public bool ContactPointValid;
        public bool ContactNormalValid;
        public bool SurfacePointWithinTolerance;
        public bool NormalAlignmentPassed;
        public float SurfacePointError;
        public float SurfacePointTolerance;
        public float CenterToSurfaceDistance;
        public float NormalAlignment;
        public float NormalAlignmentThreshold;
        public int ContactCount;
        public string ContactGeometrySource;
        public string GeometryDetail;
        public string Detail;

        public string ToLogLine()
        {
            return
                "sequence=" + Sequence +
                " | proxy=" + ProxyIndex +
                " | key=" + Clean(InstanceKey) +
                " | sample=" + Clean(SampleKind) +
                " | completed=" + Completed +
                " | passed=" + Passed +
                " | timedOut=" + TimedOut +
                " | elapsed=" + ElapsedSeconds.ToString("F3") + "s" +
                " | eventObserved=" + CollisionEventObserved +
                " | targetMatched=" + TargetColliderMatched +
                " | spawn=" + SpawnPosition.ToString("G9") +
                " | velocity=" + InitialVelocity.ToString("G9") +
                " | expectedPoint=" + ExpectedSurfacePoint.ToString("G9") +
                " | expectedNormal=" + ExpectedNormal.ToString("G9") +
                " | probeCenter=" + ProbeCenterAtEvent.ToString("G9") +
                " | surfacePoint=" + ContactPoint.ToString("G9") +
                " | pointValid=" + ContactPointValid +
                " | pointError=" + SurfacePointError.ToString("F5") +
                " | pointTolerance=" + SurfacePointTolerance.ToString("F5") +
                " | pointGate=" + SurfacePointWithinTolerance +
                " | surfaceNormal=" + ContactNormal.ToString("G9") +
                " | normalValid=" + ContactNormalValid +
                " | normalAlignment=" + NormalAlignment.ToString("F5") +
                " | normalThreshold=" + NormalAlignmentThreshold.ToString("F3") +
                " | normalGate=" + NormalAlignmentPassed +
                " | centerToSurface=" + CenterToSurfaceDistance.ToString("F5") +
                " | matchingEvents=" + ContactCount +
                " | geometrySource=" + Clean(ContactGeometrySource) +
                " | geometryDetail=" + Clean(GeometryDetail) +
                " | detail=" + Clean(Detail);
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }

    internal sealed class InstanceProxyContactTestReport
    {
        public bool Started;
        public bool Running;
        public bool Completed;
        public bool Cancelled;
        public bool Success;
        public string TargetName;
        public int ProxyCount;
        public int PlannedCases;
        public int StartedCases;
        public int CompletedCases;
        public int EventObservedCases;
        public int TargetMatchedCases;
        public int SurfacePointValidCases;
        public int SurfaceNormalValidCases;
        public int SurfacePointGatePassedCases;
        public int NormalGatePassedCases;
        public int PassedCases;
        public int FailedCases;
        public int TimedOutCases;
        public bool ProxyCollidersWereEnabled;
        public bool ProxyCollidersRestored;
        public int TemporaryRigidbodiesCreated;
        public int TemporaryRigidbodiesRemaining;
        public float ElapsedSeconds;
        public string Detail;
        public readonly List<InstanceProxyContactCaseReport> Cases =
            new List<InstanceProxyContactCaseReport>();

        public string ToStatusText()
        {
            return
                "GSP instance proxy contact test" +
                " | running=" + Running +
                " | completed=" + Completed +
                " | cancelled=" + Cancelled +
                " | success=" + Success +
                " | target=" + Clean(TargetName) +
                " | proxies=" + ProxyCount +
                " | cases=" + CompletedCases + "/" + PlannedCases +
                " | events=" + EventObservedCases +
                " | targetMatches=" + TargetMatchedCases +
                " | surfacePoints=" + SurfacePointValidCases +
                " | surfaceNormals=" + SurfaceNormalValidCases +
                " | pointGate=" + SurfacePointGatePassedCases +
                " | normalGate=" + NormalGatePassedCases +
                " | passed=" + PassedCases +
                " | failed=" + FailedCases +
                " | timedOut=" + TimedOutCases +
                " | collidersBefore=" + ProxyCollidersWereEnabled +
                " | collidersRestored=" + ProxyCollidersRestored +
                " | temporaryRigidbodies=" + TemporaryRigidbodiesRemaining +
                " | elapsed=" + ElapsedSeconds.ToString("F2") + "s" +
                " | geometry=managed-obb-closest-surface" +
                " | detail=" + Clean(Detail);
        }

        public string ToDetailedText()
        {
            StringBuilder sb = new StringBuilder(ToStatusText());
            for (int i = 0; i < Cases.Count; i++)
            {
                InstanceProxyContactCaseReport item = Cases[i];
                sb.Append("\ncase[").Append(i + 1).Append("]=")
                  .Append(item != null ? item.ToLogLine() : "<null>");
            }
            return sb.ToString();
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }
}
