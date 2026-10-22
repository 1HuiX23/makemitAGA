using System;
using System.Collections.Generic;
using UnityEngine;

namespace MakemitAGA.GeometrySourceProbe.Acquisition
{
    /// <summary>
    /// MiSide 四房间的长期地板语义注册表。
    ///
    /// v0.2.3 生命周期修订：
    /// - 玩家所在房间决定场景对象激活；非玩家房间通常会整体失活；
    /// - 因此 Registry 不再要求四个房间同时 active，也不把“房间未激活”判成 provider failure；
    /// - 每个房间只在其真实 active 窗口做唯一绑定与几何捕获；捕获完成后由 GSP-owned 地板代理独立持有；
    /// - 源房间后续 SetActive(false) 不会删除或关闭已经捕获的 GSP 地板；scene-change 仍会整体清理；
    /// - 不引用 ARL。ARL 未来只通过 Unity Physics 消费 GSP-owned floor Collider。
    /// </summary>
    internal static class HouseFloorRegistry
    {
        public const string HousePath = "World/House/HouseGame Tamagotchi/House";

        internal sealed class FloorBinding
        {
            public string RoomName;
            public string FloorObjectName;
            public string RoomPath;
            public string TargetPath;
            public GameObject Target;
            public int MatchesInRoom;
            public bool SourceActive;
        }

        private sealed class FloorSemantic
        {
            public string RoomName;
            public string FloorObjectName;
        }

        private static readonly FloorSemantic[] Semantics =
        {
            new FloorSemantic { RoomName = "Bedroom", FloorObjectName = "FloorBedroom" },
            new FloorSemantic { RoomName = "Kitchen", FloorObjectName = "FloorKitchen" },
            new FloorSemantic { RoomName = "Main", FloorObjectName = "RoomMain" },
            new FloorSemantic { RoomName = "Toilet", FloorObjectName = "FloorToilet" }
        };

        public static int ExpectedFloorCount => Semantics.Length;

        /// <summary>
        /// 返回四个长期语义槽位，但不要求当前 active。
        /// 这些槽位用于 progressive capture watcher。
        /// </summary>
        public static List<FloorBinding> CreateExpectedBindings()
        {
            List<FloorBinding> result = new List<FloorBinding>(Semantics.Length);
            for (int i = 0; i < Semantics.Length; i++)
            {
                FloorSemantic semantic = Semantics[i];
                result.Add(new FloorBinding
                {
                    RoomName = semantic.RoomName,
                    FloorObjectName = semantic.FloorObjectName,
                    RoomPath = HousePath + "/" + semantic.RoomName,
                    TargetPath = null,
                    Target = null,
                    MatchesInRoom = 0,
                    SourceActive = false
                });
            }
            return result;
        }

        /// <summary>
        /// 仅解析“当前 active 的该房间地板”。未激活不是错误，只返回 false + waiting detail。
        /// 真正的唯一性错误仍 fail-close：如果 active 房间存在但其地板同名重复，则返回明确错误。
        /// </summary>
        public static bool TryResolveActive(
            FloorBinding expected,
            out FloorBinding resolved,
            out string detail,
            out bool uniquenessFailure)
        {
            resolved = null;
            detail = null;
            uniquenessFailure = false;
            if (expected == null)
            {
                detail = "binding-null";
                uniquenessFailure = true;
                return false;
            }

            GameObject room = TargetFinder.FindUniqueExactPath(
                expected.RoomPath,
                out int roomMatches,
                out string roomError);
            if (room == null || roomMatches != 1)
            {
                if (roomMatches > 1)
                {
                    uniquenessFailure = true;
                    detail = "房间完整路径出现多个 active 匹配：room=" + expected.RoomName + "，detail=" + roomError;
                }
                else
                {
                    detail = "WAITING_ROOM_INACTIVE:" + expected.RoomName;
                }
                return false;
            }

            GameObject floor = FindUniqueDescendantExactNameActive(
                room.transform,
                expected.FloorObjectName,
                out int floorMatches,
                out string floorError);
            if (floor == null || floorMatches != 1)
            {
                if (floorMatches > 1)
                {
                    uniquenessFailure = true;
                    detail = "活动房间地板出现重复：room=" + expected.RoomName +
                        "，floor=" + expected.FloorObjectName + "，matches=" + floorMatches +
                        "，detail=" + floorError;
                }
                else
                {
                    detail = "WAITING_FLOOR_INACTIVE_OR_NOT_PRESENT:" + expected.RoomName + "/" + expected.FloorObjectName;
                }
                return false;
            }

            resolved = new FloorBinding
            {
                RoomName = expected.RoomName,
                FloorObjectName = expected.FloorObjectName,
                RoomPath = expected.RoomPath,
                TargetPath = TargetFinder.GetPath(floor.transform),
                Target = floor,
                MatchesInRoom = floorMatches,
                SourceActive = floor.activeInHierarchy
            };
            detail = "ACTIVE_UNIQUE";
            return true;
        }

        /// <summary>
        /// 保留主项目 GetFloorForObject 的语义：沿祖先向上找到房间，再解析该房间当前 active 的 floor。
        /// 注意：这只是 source 场景诊断，不是 future ARL support truth；米塔可能位于失活房间。
        /// </summary>
        public static GameObject GetFloorForObject(GameObject obj)
        {
            if (obj == null)
                return null;

            Transform current = obj.transform;
            int guard = 0;
            while (current != null && guard++ < 128)
            {
                string name = current.name ?? string.Empty;
                for (int i = 0; i < Semantics.Length; i++)
                {
                    FloorSemantic semantic = Semantics[i];
                    if (name.IndexOf(semantic.RoomName, StringComparison.OrdinalIgnoreCase) < 0)
                        continue;

                    GameObject floor = FindUniqueDescendantExactNameActive(
                        current,
                        semantic.FloorObjectName,
                        out int matches,
                        out string ignoredError);
                    return matches == 1 ? floor : null;
                }
                current = current.parent;
            }
            return null;
        }

        private static GameObject FindUniqueDescendantExactNameActive(
            Transform root,
            string exactName,
            out int matches,
            out string error)
        {
            matches = 0;
            error = null;
            if (root == null)
            {
                error = "room-root-null";
                return null;
            }
            if (string.IsNullOrWhiteSpace(exactName))
            {
                error = "floor-name-empty";
                return null;
            }

            GameObject matched = null;
            Stack<Transform> stack = new Stack<Transform>();
            stack.Push(root);
            int guard = 0;
            while (stack.Count > 0 && guard++ < 8192)
            {
                Transform current = stack.Pop();
                if (current == null)
                    continue;

                GameObject go = current.gameObject;
                if (go != null &&
                    go.activeInHierarchy &&
                    !(go.name ?? string.Empty).StartsWith("GSP_", StringComparison.Ordinal) &&
                    string.Equals(go.name, exactName, StringComparison.OrdinalIgnoreCase))
                {
                    matches++;
                    matched = go;
                }

                int childCount = 0;
                try { childCount = current.childCount; }
                catch { }
                for (int i = 0; i < childCount; i++)
                {
                    try
                    {
                        Transform child = current.GetChild(i);
                        if (child != null)
                            stack.Push(child);
                    }
                    catch { }
                }
            }

            if (guard >= 8192)
            {
                error = "room-descendant-traversal-guard-exceeded";
                return null;
            }
            if (matches == 1)
                return matched;
            if (matches == 0)
                error = "没有在当前活动房间子树找到名称完整等于 “" + exactName + "” 的活动对象。";
            else
                error = "当前活动房间子树内出现 " + matches + " 个同名活动对象：“" + exactName + "”。";
            return null;
        }
    }
}
