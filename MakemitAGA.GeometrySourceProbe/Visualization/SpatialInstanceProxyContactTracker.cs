using System;
using UnityEngine;

namespace MakemitAGA.GeometrySourceProbe.Visualization
{
    /// <summary>
    /// 挂在 v0.1.23 受控接触测试球上的 IL2CPP 回调桥。
    /// 只转发真实 OnCollisionEnter；测试器会再次校验碰撞对象是否为当前目标代理。
    /// </summary>
    public sealed class SpatialInstanceProxyContactTracker : MonoBehaviour
    {
        private int _probeId;
        private bool _initialized;

        public SpatialInstanceProxyContactTracker(IntPtr pointer)
            : base(pointer)
        {
        }

        public void Initialize(int probeId)
        {
            _probeId = probeId;
            _initialized = probeId > 0;
        }

        public void OnCollisionEnter(Collision collision)
        {
            if (_initialized)
                InstanceProxyContactTest.NotifyCollisionEnter(_probeId, collision);
        }
    }
}
