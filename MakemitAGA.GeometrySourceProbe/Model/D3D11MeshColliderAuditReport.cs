namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// v0.1.22：独立 D3D11 提取 Mesh 的静态非凸 MeshCollider 烹饪审计结果。
    /// </summary>
    public sealed class D3D11MeshColliderAuditReport
    {
        public bool Completed;
        public bool Success;
        public string TargetName;
        public string TargetPath;
        public string PreviewObjectName;
        public string PreviewMeshName;
        public int VertexCount;
        public int IndexCount;
        public int TriangleCount;
        public bool PreviewExists;
        public bool ColliderCreated;
        public bool ColliderEnabled;
        public bool SharedMeshAssigned;
        public bool SharedMeshMatchesPreviewMesh;
        public bool Convex;
        public bool IsTrigger;
        public bool AttachedRigidbodyPresent;
        public bool PhysicsSyncCalled;
        public string SourceBounds;
        public string PreviewBounds;
        public string ColliderBounds;
        public bool ColliderMatchesPreviewBounds;
        public bool ColliderMatchesSourceBounds;
        public string CookingPath;
        public string Detail;

        public string ToText()
        {
            return
                "GSP D3D11 static MeshCollider audit" +
                " | completed=" + Completed +
                " | success=" + Success +
                " | target=" + (TargetName ?? "<none>") +
                " | path=" + (TargetPath ?? "<none>") +
                " | object=" + (PreviewObjectName ?? "<none>") +
                " | mesh=" + (PreviewMeshName ?? "<none>") +
                " | vertices=" + VertexCount +
                " | indices=" + IndexCount +
                " | triangles=" + TriangleCount +
                " | previewExists=" + PreviewExists +
                " | collider=created:" + ColliderCreated +
                    ",enabled:" + ColliderEnabled +
                    ",sharedMesh:" + SharedMeshAssigned +
                    ",sameMesh:" + SharedMeshMatchesPreviewMesh +
                    ",convex:" + Convex +
                    ",trigger:" + IsTrigger +
                    ",rigidbody:" + AttachedRigidbodyPresent +
                " | physicsSync=" + PhysicsSyncCalled +
                " | sourceBounds=" + (SourceBounds ?? "<none>") +
                " | previewBounds=" + (PreviewBounds ?? "<none>") +
                " | colliderBounds=" + (ColliderBounds ?? "<none>") +
                " | matches=preview:" + ColliderMatchesPreviewBounds +
                    ",source:" + ColliderMatchesSourceBounds +
                " | cookingPath=" + (CookingPath ?? "<none>") +
                " | detail=" + (Detail ?? "<none>");
        }
    }
}
