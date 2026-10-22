using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// 一个由空间实例代理草案实际创建出来的静态盒代理。
    /// 本报告只记录 GSP 自己创建的对象；不会把游戏原对象的 Collider 计入。
    /// </summary>
    internal sealed class SpatialInstanceProxyEntryReport
    {
        public int Index;
        public string InstanceKey;
        public string ParentInstanceKey;
        public int CompoundPartIndex;
        public string CompoundPartKind;
        public string ProxyType;
        public string ObjectName;
        public Vector3 Center;
        public Vector3 Size;
        public Vector3 EulerDegrees;
        public bool GameObjectCreated;
        public bool RendererCreated;
        public bool ColliderCreated;
        public bool ColliderEnabled;
        public bool RigidbodyPresent;
        public bool FixedToWorld;
        public bool Pickable;
        public int Layer;
        public string Detail;

        public string ToLogLine()
        {
            return
                "index=" + Index +
                " | key=" + Clean(InstanceKey) +
                " | parentKey=" + Clean(ParentInstanceKey) +
                " | compoundPart=" + CompoundPartIndex + ":" + Clean(CompoundPartKind) +
                " | proxyType=" + Clean(ProxyType) +
                " | object=" + Clean(ObjectName) +
                " | center=" + Center.ToString("G9") +
                " | size=" + Size.ToString("G9") +
                " | euler=" + EulerDegrees.ToString("G9") +
                " | created=gameObject:" + GameObjectCreated +
                ",renderer:" + RendererCreated +
                ",collider:" + ColliderCreated +
                " | colliderEnabled=" + ColliderEnabled +
                " | rigidbodyPresent=" + RigidbodyPresent +
                " | profile=fixedToWorld:" + FixedToWorld +
                ",pickable:" + Pickable +
                " | layer=" + Layer +
                " | detail=" + Clean(Detail);
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }

    /// <summary>
    /// 实例单盒/复合盒运行时创建、可视化、Collider 启停与回滚状态。
    /// </summary>
    internal sealed class SpatialInstanceProxyRuntimeReport
    {
        public bool Started;
        public bool Completed;
        public bool Success;
        public string TargetName;
        public string TargetPath;
        public string RootName;
        public int Candidates;
        public int Created;
        public int CompoundInstances;
        public int CompoundPartsCreated;
        public int SkippedNoDraft;
        public int SkippedNoProfile;
        public int SkippedDisabled;
        public int SkippedUnsupportedProxy;
        public int SkippedNotFixedToWorld;
        public int SkippedPickable;
        public int SkippedCompoundInvalid;
        public int Colliders;
        public int Renderers;
        public int Rigidbodies;
        public bool VisualVisible;
        public bool CollidersEnabled;
        public string Detail;
        public readonly List<SpatialInstanceProxyEntryReport> Entries =
            new List<SpatialInstanceProxyEntryReport>();

        public string ToStatusText()
        {
            return
                "GSP instance proxy runtime" +
                " | completed=" + Completed +
                " | success=" + Success +
                " | target=" + Clean(TargetName) +
                " | candidates=" + Candidates +
                " | created=" + Created +
                " | compoundInstances=" + CompoundInstances +
                " | compoundParts=" + CompoundPartsCreated +
                " | colliders=" + Colliders +
                " | renderers=" + Renderers +
                " | rigidbodies=" + Rigidbodies +
                " | visual=" + VisualVisible +
                " | colliderEnabled=" + CollidersEnabled +
                " | skipped=noDraft:" + SkippedNoDraft +
                ",noProfile:" + SkippedNoProfile +
                ",disabled:" + SkippedDisabled +
                ",unsupportedProxy:" + SkippedUnsupportedProxy +
                ",notFixed:" + SkippedNotFixedToWorld +
                ",pickable:" + SkippedPickable +
                ",compoundInvalid:" + SkippedCompoundInvalid +
                " | root=" + Clean(RootName) +
                " | detail=" + Clean(Detail);
        }

        public string ToDetailedText()
        {
            StringBuilder sb = new StringBuilder(ToStatusText());
            for (int i = 0; i < Entries.Count; i++)
            {
                SpatialInstanceProxyEntryReport item = Entries[i];
                sb.Append("\nproxy[").Append(i + 1).Append("]=")
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
