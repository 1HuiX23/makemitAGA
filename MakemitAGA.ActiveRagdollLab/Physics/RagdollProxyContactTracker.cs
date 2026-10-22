using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// IL2CPP 碰撞回调桥。只转发 Collider 身份，不读取 ContactPoint、contacts、contactCount 或 GetContact。
    /// </summary>
    public sealed class RagdollProxyContactTracker : MonoBehaviour
    {
        private string _bodyLabel;
        private bool _initialized;

        public RagdollProxyContactTracker(IntPtr pointer)
            : base(pointer)
        {
        }

        public void Initialize(string bodyLabel)
        {
            _bodyLabel = string.IsNullOrWhiteSpace(bodyLabel) ? "UNKNOWN" : bodyLabel;
            _initialized = true;
        }

        public void OnCollisionEnter(Collision collision)
        {
            if (_initialized)
                ActiveRagdollLabRuntime.NotifyProxyCollision(_bodyLabel, "ENTER", collision);
        }

        public void OnCollisionStay(Collision collision)
        {
            if (_initialized)
                ActiveRagdollLabRuntime.NotifyProxyCollision(_bodyLabel, "STAY", collision);
        }

        public void OnCollisionExit(Collision collision)
        {
            if (_initialized)
                ActiveRagdollLabRuntime.NotifyProxyCollision(_bodyLabel, "EXIT", collision);
        }
    }
}
