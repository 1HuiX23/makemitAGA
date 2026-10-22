using System;
using System.Collections.Generic;
using System.Text;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    internal sealed class HouseFloorProviderItemReport
    {
        public string RoomName;
        public string FloorObjectName;
        public string RoomPath;
        public string TargetPath;
        public bool TargetIdentityValid;
        public bool SourceCurrentlyActive;
        public bool SourceActiveAtCapture;
        public bool CachedPersistent;
        public bool CaptureWaiting;
        public bool CaptureFailed;
        public int CaptureAttempts;
        public int TargetLayer = -1;
        public int ProviderLayer = -1;
        public bool LayerIdentityValid;
        public bool AuditCompleted;
        public bool AuditSuccess;
        public int VertexCount;
        public int IndexCount;
        public int TriangleCount;
        public int RawBoundaryEdges;
        public int WeldedBoundaryEdges;
        public int WeldedComponents;
        public int NonManifoldEdges;
        public int OrientationConflicts;
        public int DuplicateTriangles;
        public string WorldBounds;
        public float MinY;
        public float MaxY;
        public bool MeshCreated;
        public bool ColliderCreated;
        public bool ColliderEnabled;
        public int ProviderRigidbodies;
        public int SourceNativeColliders;
        public int SourceNativeEnabledColliders;
        public int SourceNativeRigidbodies;
        public string Detail;

        public string ToCompactText()
        {
            return Clean(RoomName) + "/" + Clean(FloorObjectName) +
                "{state=" + CaptureState() +
                ",attempts=" + CaptureAttempts +
                ",sourceActive=" + SourceCurrentlyActive +
                ",activeAtCapture=" + SourceActiveAtCapture +
                ",cached=" + CachedPersistent +
                ",identity=" + TargetIdentityValid +
                ",layer=" + TargetLayer + "→" + ProviderLayer + "/" + LayerIdentityValid +
                ",v/i/t=" + VertexCount + "/" + IndexCount + "/" + TriangleCount +
                ",topo=b:" + WeldedBoundaryEdges + ",nm:" + NonManifoldEdges +
                ",oc:" + OrientationConflicts + ",dup:" + DuplicateTriangles +
                ",y=" + MinY.ToString("F4") + ".." + MaxY.ToString("F4") +
                ",bounds=" + Clean(WorldBounds) +
                ",collider=" + ColliderCreated + "/" + ColliderEnabled +
                ",rb=" + ProviderRigidbodies +
                ",nativeC=" + SourceNativeColliders + "/" + SourceNativeEnabledColliders +
                ",detail=" + Clean(Detail) + "}";
        }

        private string CaptureState()
        {
            if (CachedPersistent && ColliderCreated) return "CAPTURED";
            if (CaptureFailed) return "FAILED";
            if (!CaptureWaiting && CaptureAttempts > 0 && !AuditCompleted) return "CAPTURING";
            if (CaptureWaiting) return "WAITING_ACTIVE";
            if (AuditCompleted && AuditSuccess) return "AUDITED";
            return "UNSEEN";
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value)) return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }

    internal sealed class HouseFloorProviderReport
    {
        public bool Started;
        public bool Running;
        public bool Completed;
        public bool Success;
        public bool Operational;
        public bool WatcherActive;
        public bool DesiredVisual;
        public bool DesiredCollider;
        public string SceneName;
        public string Stage;
        public int ExpectedFloors;
        public int ResolvedFloors;
        public int AuditedFloors;
        public int BuiltFloors;
        public int EnabledFloors;
        public int ActiveSourceFloors;
        public int WaitingFloors;
        public int FailedFloors;
        public int CurrentIndex = -1;
        public string CurrentRoom;
        public string Detail;
        public readonly List<HouseFloorProviderItemReport> Floors =
            new List<HouseFloorProviderItemReport>();

        public string ToStatusText()
        {
            StringBuilder sb = new StringBuilder();
            sb.Append("GSP house-floor provider v0.2.3")
              .Append(" | running=").Append(Running)
              .Append(" | completed=").Append(Completed)
              .Append(" | success=").Append(Success)
              .Append(" | operational=").Append(Operational)
              .Append(" | watcher=").Append(WatcherActive)
              .Append(" | stage=").Append(Clean(Stage))
              .Append(" | scene=").Append(Clean(SceneName))
              .Append(" | captured=").Append(BuiltFloors).Append('/').Append(ExpectedFloors)
              .Append(" | activeSource=").Append(ActiveSourceFloors).Append('/').Append(ExpectedFloors)
              .Append(" | waiting=").Append(WaitingFloors)
              .Append(" | failed=").Append(FailedFloors)
              .Append(" | enabled=").Append(EnabledFloors).Append('/').Append(BuiltFloors)
              .Append(" | desiredVisual=").Append(DesiredVisual)
              .Append(" | desiredCollider=").Append(DesiredCollider)
              .Append(" | current=").Append(CurrentIndex).Append(':').Append(Clean(CurrentRoom))
              .Append(" | sourceRoomActivationIsNotSupportTruth=True")
              .Append(" | cachedFloorsPersistAcrossRoomDeactivation=True")
              .Append(" | dllCoupling=NONE_UNITY_PHYSICS_ONLY")
              .Append(" | detail=").Append(Clean(Detail));
            for (int i = 0; i < Floors.Count; i++)
                sb.Append(" | floor[").Append(i).Append("]=").Append(Floors[i].ToCompactText());
            return sb.ToString();
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value)) return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }
}
