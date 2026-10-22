using System;
using Il2CppInterop.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// Unity IL2CPP 原生/注入组件的统一创建入口。
    ///
    /// 不调用 GameObject.AddComponent&lt;T&gt;()，避免目标游戏没有保留对应
    /// AOT 泛型实例时触发 MethodInfoStoreGeneric_AddComponent 初始化异常。
    /// 组件由非泛型 AddComponent(Il2CppSystem.Type) 创建，再通过 Il2CppInterop
    /// 的 TryCast 转换为具体代理类型。
    /// </summary>
    internal static class Il2CppComponentFactory
    {
        public static T Add<T>(GameObject owner, string phase)
            where T : Component
        {
            if (owner == null)
                throw new ArgumentNullException(nameof(owner), "component owner is null at phase " + phase);

            Component raw;
            try
            {
                raw = owner.AddComponent(Il2CppType.Of<T>());
            }
            catch (Exception e)
            {
                throw new InvalidOperationException(
                    "non-generic AddComponent failed at " + phase + " for " + typeof(T).FullName,
                    e);
            }

            if (raw == null)
                throw new InvalidOperationException(
                    "non-generic AddComponent returned null at " + phase + " for " + typeof(T).FullName);

            T typed;
            try
            {
                typed = raw.TryCast<T>();
            }
            catch (Exception e)
            {
                throw new InvalidOperationException(
                    "Il2Cpp TryCast failed at " + phase + " for " + typeof(T).FullName,
                    e);
            }

            if (typed == null)
                throw new InvalidOperationException(
                    "Il2Cpp TryCast returned null at " + phase + " for " + typeof(T).FullName);

            return typed;
        }

        public static string DescribeException(Exception exception)
        {
            if (exception == null)
                return "<none>";

            System.Text.StringBuilder sb = new System.Text.StringBuilder(256);
            Exception current = exception;
            int depth = 0;
            while (current != null && depth < 6)
            {
                if (depth > 0)
                    sb.Append(" -> ");
                sb.Append(current.GetType().Name);
                sb.Append(':');
                sb.Append(current.Message ?? "<no-message>");
                current = current.InnerException;
                depth++;
            }
            return sb.ToString();
        }
    }
}
