using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using BepInEx;
using BepInEx.Unity.IL2CPP.Utils.Collections;
using HarmonyLib;
using Il2CppInterop.Runtime;
using Il2CppInterop.Runtime.Injection;
using Il2CppInterop.Runtime.Runtime;
using RootMotion.FinalIK;
using UnityEngine;
using UnityEngine.AI;

namespace MakemitAGA.Mita_self.Mita_tools
{
    internal static class MitaSitOwnershipPatches
    {
        [HarmonyPatch(typeof(MitaPerson), "MagnetToTarget")]
        [HarmonyPrefix]
        private static bool BlockMagnetWhenOwned(MitaPerson __instance)
        {
            return !Mita_sit.ShouldBlockNativeMagnet(__instance);
        }

        [HarmonyPatch(typeof(MitaPerson), "AiWalkToTarget", new Type[] { typeof(Transform) })]
        [HarmonyPrefix]
        private static bool BlockExternalWalkWhenOwned(MitaPerson __instance)
        {
            return !Mita_sit.ShouldBlockExternalAiWalk(__instance);
        }
    }

    public class Mita_sit : MonoBehaviour
    {
        // IL2CPP 环境下，插件自定义 MonoBehaviour 必须提供 IntPtr 构造函数。
        // Unity 从 native 侧创建组件 wrapper 时会调用这个构造函数。
        public Mita_sit(IntPtr ptr) : base(ptr) { }

        // 是否创建脚点/盆骨调试球。主项目默认关闭；需要排查 IK 时再改成 true。
        private const bool EnableDebugMarkers = false;

        // 是否向 BepInEx 控制台输出详细过程日志。主项目默认关闭，避免刷屏。
        private const bool EnableVerboseLog = false;

        private static Mita_sit _instance;
        private static bool _il2CppTypeRegistered;

        /// <summary>
        /// 把 Mita_sit 注册为 IL2CPP 可创建的自定义 MonoBehaviour。
        /// 不能直接对未注册的插件类型调用 GameObject.AddComponent&lt;T&gt;，否则 IL2CPP 泛型缓存可能在加载阶段抛空引用。
        /// </summary>
        public static bool EnsureIl2CppTypeRegistered()
        {
            if (_il2CppTypeRegistered) return true;

            try
            {
                ClassInjector.RegisterTypeInIl2Cpp<Mita_sit>();
                _il2CppTypeRegistered = true;
                return true;
            }
            catch (Exception e)
            {
                // 某些热重载/重复加载场景会提示已经注册。只要类型已经存在，就允许继续。
                string msg = e.ToString();
                if (msg.IndexOf("already", StringComparison.OrdinalIgnoreCase) >= 0 ||
                    msg.IndexOf("registered", StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    _il2CppTypeRegistered = true;
                    return true;
                }

                Plugin.Logger?.LogError("Mita_sit IL2CPP 类型注册失败: " + e);
                return false;
            }
        }

        /// <summary>
        /// 使用非泛型 AddComponent(Type) 创建 Mita_sit。
        /// 这样可以绕开 IL2CPP 下 GameObject.AddComponent&lt;T&gt; 对插件自定义类型的泛型 MethodInfoStore 初始化问题。
        /// </summary>
        private static Mita_sit AddMitaSitComponent(GameObject host)
        {
            if (host == null) return null;
            if (!EnsureIl2CppTypeRegistered()) return null;

            Component component = host.AddComponent(Il2CppType.Of<Mita_sit>());
            return component != null ? component.TryCast<Mita_sit>() : null;
        }

        private static Mita_sit GetMitaSitComponent(GameObject host)
        {
            if (host == null) return null;
            if (!EnsureIl2CppTypeRegistered()) return null;

            Component component = host.GetComponent(Il2CppType.Of<Mita_sit>());
            return component != null ? component.TryCast<Mita_sit>() : null;
        }

        /// <summary>
        /// 确保主线程上存在一个 Mita_sit 组件。
        /// 控制台第一次执行 sit(...) 时会懒加载。
        /// </summary>
        public static Mita_sit EnsureInstance()
        {
            if (_instance != null) return _instance;
            if (!EnsureIl2CppTypeRegistered()) return null;

            try
            {
                if (Plugin.Runner != null)
                {
                    _instance = GetMitaSitComponent(Plugin.Runner.gameObject);
                    if (_instance == null) _instance = AddMitaSitComponent(Plugin.Runner.gameObject);
                    return _instance;
                }
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogError("Mita_sit 挂载到 Plugin.Runner 失败: " + e);
            }

            try
            {
                // 理论上 Plugin.Runner 会在 Plugin.Load 中创建；这里是兜底，避免早期调用直接失败。
                GameObject runner = new GameObject("Mita_sit_Runner");
                UnityEngine.Object.DontDestroyOnLoad(runner);
                _instance = AddMitaSitComponent(runner);
                return _instance;
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogError("Mita_sit 兜底 Runner 创建失败: " + e);
                return null;
            }
        }

        /// <summary>
        /// 控制台/AI tool 的主入口：按名称寻找座椅/物体并执行入座。
        /// 例：sit(TestChair_High)
        /// </summary>
        public static void Sit(string objectNamePart)
        {
            Mita_sit inst = EnsureInstance();
            if (inst == null)
            {
                ConsoleMain.ConsolePrintGame("<color=red>Mita_sit 初始化失败。</color>");
                return;
            }

            if (string.IsNullOrWhiteSpace(objectNamePart))
            {
                ConsoleMain.ConsolePrintGame("用法: sit(TestChair_High)");
                return;
            }

            inst.StartSitSequenceByName(objectNamePart);
        }

        /// <summary>
        /// 让米塔从当前坐姿恢复站立，并把原生跟随交还给游戏。
        /// 可供 come 指令或之后的动作调度器调用。
        /// </summary>
        public static void UnlockAndResume()
        {
            Mita_sit inst = EnsureInstance();
            if (inst != null) inst.UnlockMita();
        }

        public static bool HasActiveSession
        {
            get
            {
                Mita_sit inst = _instance;
                return inst != null && (inst._isSittingLocked || inst._hasBackup || s_actionControlActive);
            }
        }
        // ============================================================================================
        // 可调参数。后续如果迁回主项目，建议改成 ConfigEntry 或动作参数。
        // ============================================================================================
        private const string BundleFileName = "mita_actions";

        // 兼容你们可能打包过的不同 Controller 名称。
        private static readonly string[] ControllerAssetNames = new string[]
        {
            "MitaPrefab",
            "MitaPrefab.controller",
            "Override_Sit",
            "Override_Sit.controller"
        };

        private const string StartSitStateName = "Mita StartSit";
        private const string WalkStateName = "Base Layer.Walk";

        private const float WalkStopDistance = 0.28f;
        private const float WalkTimeout = 10.0f;
        private const float NativeControlPulseInterval = 0.22f;
        private const float NativeRepathInterval = 0.85f;
        private const float ManualApproachSpeed = 1.45f;
        private const float ManualApproachTimeout = 7.0f;
        private const float ManualApproachStopDistance = 0.08f;
        private const float AlignDuration = 0.35f;
        private const float SitDownDuration = 0.95f;
        private const float StandUpDuration = 0.82f;

        // 入座时不要直接把 root 瞬移到座面内侧。
        // 流程改为：先走到座椅前方 -> 原地转身 -> 播放坐下动画时再平滑滑入座面。
        private const float SitRootSlideDelayRatio = 0.08f;

        // 起身时优先倒采样 StartSit 动画，让姿态先从坐姿回到站姿，再移动到安全站立点。
        // 注意：这里故意让 root 移动晚一些，避免“坐姿整体飞起来”，也避免脚部提前释放导致穿地。
        private const float StandRootMoveDelayRatio = 0.58f;

        // 起身时的脚底防穿地策略：先保持脚部 IK，再在接近站稳时释放。
        // 这比一开始就把 foot weight 归零更像真实起身：脚先踩住支撑面，身体再起来。
        private const float StandFootLockHoldRatio = 0.55f;
        private const float StandFootReleaseEndRatio = 0.94f;
        private const float FootGroundGuardMargin = 0.012f;

        // 悬空脚没有真实支撑面，不能像支撑脚一样锁太久，否则起身时会像“脚被空气钉住”。
        // 因此悬空脚在起身早期就开始释放，支撑脚才使用完整的防穿地锁定策略。
        private const float UnsupportedStandFootReleaseStartRatio = 0.18f;
        private const float UnsupportedStandFootReleaseEndRatio = 0.48f;

        // 纯程序层面的“坐下生动度”补偿。幅度不要太大，否则会像弹簧。
        private const float SitSettleBounceAmplitude = 0.018f;
        private const float SitSettleStartRatio = 0.72f;

        // 走路停靠点和最终坐姿根节点必须分开。
        // 旧版把根节点锁在 cube 前沿外侧，视觉上会“坐太靠前”；
        // 现在先走到前方安全点，再把最终坐姿根节点压回座面前沿内侧。
        private const float ApproachExtraOffset = 0.34f;
        private const float SeatedRootInsetFromFrontEdge = 0.14f;

        // 脚的默认前伸距离。没有支撑面时，这个点会变成自然垂脚的 XZ 位置。
        private const float FootForwardOffset = 0.45f;
        private const float FootSideOffset = 0.15f;
        private const float AnkleHeightOffset = 0.13f;

        // 座面到盆骨的经验补偿。太小会陷进椅子，太大像半蹲。
        private const float SeatPelvisPadding = 0.15f;

        // 射线设置。
        private const float RaycastHeight = 2.0f;
        private const float RaycastDistance = 6.0f;

        // 复杂场景安全参数：
        // 1. 座面只从目标物体自身 Collider 上取，且用多个采样点取较低的可坐表面，降低扶手/靠背误判。
        // 2. 脚点射线从接近膝盖/小腿高度开始，不再从头顶打，避免桌面、扶手遮挡。
        private const float SeatProbeRayHeightAboveBounds = 0.45f;
        private const float SeatProbeSideSampleOffset = 0.18f;
        private const float FootProbeRayHeightAboveSeat = 0.22f;
        private const float FootProbeMaxSupportAboveSeat = 0.16f;

        // 动作控制权保险丝。即使协程因异常/切场景中断，静态 Harmony 锁也不会永久卡死。
        private const float ActionControlTimeout = 45.0f;

        // FinalIK 膝盖 Bend Goal。降低坐下/起身时腿太直导致的反关节、内八翻转风险。
        private const float KneeBendGoalForwardOffset = 0.55f;
        private const float KneeBendGoalSideOffset = 0.04f;
        private const float KneeBendGoalUpOffset = 0.03f;

        // 没有脚部支撑时，让脚自然垂到座面下方。不要强行找地板.
        private const float DanglingFootDropFromSeat = 0.48f;
        private const float UnsupportedFootWeight = 0.55f;
        private const float SupportedFootWeight = 1.0f;

        // ============================================================================================
        // AssetBundle ICall
        // ============================================================================================
        private delegate IntPtr LoadFromFileDelegate(IntPtr path, uint crc, ulong offset);
        private delegate IntPtr LoadAssetDelegate(IntPtr bundlePtr, IntPtr assetName, IntPtr typePtr);

        private static LoadFromFileDelegate _loadFromFileFunc;
        private static LoadAssetDelegate _loadAssetFunc;

        private IntPtr _bundlePtr = IntPtr.Zero;
        private RuntimeAnimatorController _sitController;
        private bool _isLoaded;

        // ============================================================================================
        // 当前 Mita 引用
        // ============================================================================================
        private Coroutine _actionRoutine;
        private GameObject _mitaAvatar;
        private MitaPerson _mitaScript;
        private Animator _animator;
        private NavMeshAgent _navAgent;
        private FullBodyBipedIK _fbbik;

        // ============================================================================================
        // 动作控制权。
        // sit(...) 执行期间需要临时阻断原生跟随/磁吸，否则米塔可能走向座椅途中又被游戏拉回玩家身边。
        // 本模块自己发起的 AiWalkToTarget 会通过 s_allowInternalNativeWalk 临时放行。
        // ============================================================================================
        private static bool s_actionControlActive;
        private static bool s_allowInternalNativeWalk;
        private static MitaPerson s_controlledMita;
        private static Mita_sit s_controlOwner;
        private static float s_actionControlExpiresAt;

        private bool _lastWalkArrived;

        // ============================================================================================
        // 原始状态备份
        // ============================================================================================
        private bool _hasBackup;

        private RuntimeAnimatorController _originalController;
        private bool _originalApplyRootMotion;
        private AnimatorCullingMode _originalCullingMode;
        private float _originalAnimatorSpeed;

        private bool _navHadAgent;
        private bool _navOriginalEnabled;
        private bool _navOriginalStopped;
        private bool _navOriginalUpdatePosition;
        private bool _navOriginalUpdateRotation;

        private MonoBehaviour _animFuncScript;
        private MonoBehaviour _unitStepScript;
        private bool _animFuncWasEnabled;
        private bool _unitStepWasEnabled;

        private float _originalSolverWeight;

        private Transform _originalBodyTarget;
        private Transform _originalLHandTarget;
        private Transform _originalRHandTarget;
        private Transform _originalLFootTarget;
        private Transform _originalRFootTarget;

        private float _originalBodyWeight;
        private float _originalLHandWeight;
        private float _originalRHandWeight;
        private float _originalLFootWeight;
        private float _originalRFootWeight;

        // ============================================================================================
        // 坐姿实时锁定状态
        // ============================================================================================
        private bool _isSittingLocked;
        private float _currentRootY;
        private Vector3 _lockedRootPos;
        private Quaternion _lockedRootRot;

        // 起身时使用的安全站立点。
        // 关键点：不能在“坐姿下沉后的 rootY”直接 Rebind/Play Walk，否则腿会先穿地再被原生逻辑拉回。
        private bool _hasStandExitPose;
        private Vector3 _standExitRootPos;
        private Quaternion _standExitRootRot;

        private Vector3 _leftFootTarget;
        private Vector3 _rightFootTarget;
        private float _leftFootIkWeight;
        private float _rightFootIkWeight;
        private float _leftFootTargetWeight;
        private float _rightFootTargetWeight;

        // 每只脚独立记录是否真的找到了承重面。
        // 四种情况会分别处理：双脚支撑、左支撑右悬空、右支撑左悬空、双脚悬空。
        private bool _leftFootHasSupport;
        private bool _rightFootHasSupport;
        private bool _leftFootGuardEnabled;
        private bool _rightFootGuardEnabled;

        // 脚部防穿地基准线。这里记录的是“脚/踝 IK 目标最低线”，不是强行要求必须是地板。
        // 只有真正有支撑面的脚才启用 guard；悬空脚只保留自然垂脚目标，不参与 root lift。
        private bool _hasFootGroundLines;
        private float _leftFootGroundLineY;
        private float _rightFootGroundLineY;

        // ============================================================================================
        // 调试对象
        // ============================================================================================
        private GameObject _debugSphereL;
        private GameObject _debugSphereR;
        private GameObject _debugSpherePelvis;

        // FinalIK / 原生 Character_Look 在 IL2CPP 环境下有些脚本会假定 effector.target 永远不为空。
        // v1.16 把 target 直接置 null，在部分场景可能触发每帧 NullReferenceException。
        // v1.16.1 改成隐藏锚点：既切断原生旧 target 的拉扯，又不给其它脚本留下 null target。
        private Transform _bodyEffectorAnchor;
        private Transform _leftHandEffectorAnchor;
        private Transform _rightHandEffectorAnchor;
        private Transform _leftFootEffectorAnchor;
        private Transform _rightFootEffectorAnchor;

        private Transform _leftKneeBendAnchor;
        private Transform _rightKneeBendAnchor;
        private Transform _originalLeftLegBendGoal;
        private Transform _originalRightLegBendGoal;
        private float _originalLeftLegBendWeight;
        private float _originalRightLegBendWeight;
        private bool _hasOriginalLeftLegBendWeight;
        private bool _hasOriginalRightLegBendWeight;

        private void Start()
        {
            _instance = this;
            InitializeICalls();
        }

        private void OnDisable()
        {
            ForceRestoreIfNeeded();
        }

        private void OnDestroy()
        {
            ForceRestoreIfNeeded();
        }

        private void Update()
        {
            // 主项目中不监听测试快捷键。
            // 控制台命令由 DialoguePatches.InterceptCommand 分发到本组件。
            CheckActionControlWatchdog();
        }

        // ============================================================================================
        // LateUpdate：最终覆盖层。
        // Animator / 原生脚本 / LookAtIK 都跑完后，再锁根节点和脚部 IK。
        // ============================================================================================
        private void LateUpdate()
        {
            if (!_isSittingLocked || _mitaAvatar == null || _fbbik == null) return;

            try
            {
                _mitaAvatar.transform.position = new Vector3(_lockedRootPos.x, _currentRootY, _lockedRootPos.z);
                _mitaAvatar.transform.rotation = _lockedRootRot;

                UpdateLegBendAnchors();

                _fbbik.solver.IKPositionWeight = 1f;

                // Body/Hands 不参与坐姿约束，避免跟原生 target 或 Look IK 抢控制权。
                _fbbik.solver.bodyEffector.positionWeight = 0f;
                _fbbik.solver.leftHandEffector.positionWeight = 0f;
                _fbbik.solver.rightHandEffector.positionWeight = 0f;

                // 同步隐藏锚点。不要让 FinalIK 或原生 Look 脚本看到 null target。
                if (_leftFootEffectorAnchor != null) _leftFootEffectorAnchor.position = _leftFootTarget;
                if (_rightFootEffectorAnchor != null) _rightFootEffectorAnchor.position = _rightFootTarget;

                _fbbik.solver.leftFootEffector.position = _leftFootTarget;
                _fbbik.solver.leftFootEffector.positionWeight = _leftFootIkWeight;
                _fbbik.solver.rightFootEffector.position = _rightFootTarget;
                _fbbik.solver.rightFootEffector.positionWeight = _rightFootIkWeight;
            }
            catch (Exception e)
            {
                LogDebug("LateUpdate lock skipped: " + e.GetType().Name + " / " + e.Message);
            }
        }

        /// <summary>
        /// 外部入口：坐到指定物体。后续迁回主项目时，可以把它拆成 sit(target) tool 的执行端。
        /// </summary>
        public void StartSitSequence(GameObject chair)
        {
            if (chair == null)
            {
                LogWarn("StartSitSequence failed: target chair is null. Create or pass a valid GameObject first.");
                return;
            }

            if (!_isLoaded && !LoadBundleViaICall())
            {
                LogWarn("StartSitSequence failed: cannot load sit controller from AssetBundle.");
                return;
            }

            if (!ResolveMitaReferences())
            {
                LogWarn("StartSitSequence failed: cannot resolve Mita references.");
                return;
            }

            AcquireActionControl();
            StopCurrentActionRoutine();

            if (_isSittingLocked)
                _actionRoutine = StartCoroutine(TransitionToNewSit(chair).WrapToIl2Cpp());
            else
                _actionRoutine = StartCoroutine(SitRoutine(chair.transform).WrapToIl2Cpp());
        }

        /// <summary>
        /// 调试入口：按名称模糊寻找物体并尝试入座。可在 UnityExplorer / 控制台调用。
        /// </summary>
        public void StartSitSequenceByName(string objectNamePart)
        {
            GameObject target = FindObjectByNamePart(objectNamePart);
            StartSitSequence(target);
        }

        public void UnlockMita()
        {
            if (!_isSittingLocked && !_hasBackup && !s_actionControlActive) return;

            StopCurrentActionRoutine();

            if (_mitaAvatar == null)
            {
                ForceRestoreIfNeeded();
                return;
            }

            // 如果还处在“走向座椅/等待到达”的阶段，不能走起身动画。
            // 否则 _lockedRootPos 还没有被坐姿锁初始化，强制起身会把她拉到旧锁点/默认点，表现成瞬移。
            if (!_isSittingLocked)
            {
                StopNativeWalking();
                RestoreOriginalState();
                HideDebugSpheres();
                ResetSitRuntimeFlags();
                ReleaseActionControl();
                TryResumeNativeFollowPlayer();
                LogInfo("Mita action cancelled before sit lock.");
                return;
            }

            _actionRoutine = StartCoroutine(UnlockRoutine(true).WrapToIl2Cpp());
        }

        private IEnumerator TransitionToNewSit(GameObject newChair)
        {
            yield return UnlockRoutine(false);
            yield return new WaitForSeconds(0.25f);
            yield return SitRoutine(newChair.transform);
        }

        // ============================================================================================
        // 主流程：定位 -> 走到停靠点 -> 对齐 -> 接管动画/IK -> 环境测量 -> 坐下锁定
        // ============================================================================================
        private IEnumerator SitRoutine(Transform target)
        {
            if (target == null) yield break;
            if (!ResolveMitaReferences()) yield break;

            AcquireActionControl();
            CaptureOriginalState();

            SeatProbe seat = BuildSeatProbe(target);
            Vector3 finalSitRootPos = seat.SitRootPosition;
            Quaternion finalSitRootRot = seat.DockRotation;

            // NavMesh 只负责把角色带到座椅前方的安全站立点；最终坐姿 root 不再提前对齐。
            // 之前“靠近后瞬移”的原因，就是这里把角色从停靠点快速 Lerp 到座面内侧。
            Vector3 approachPoint = seat.ApproachPosition;
            approachPoint.y = _mitaAvatar.transform.position.y;
            NavMeshHit navHit;
            if (NavMesh.SamplePosition(approachPoint, out navHit, 1.2f, -1))
                approachPoint = navHit.position;

            yield return WalkToDockPoint(approachPoint);

            if (!_lastWalkArrived)
            {
                LogWarn("Native walk did not reach dock point cleanly. Switching to guarded manual approach to avoid final teleport.");
                yield return ManualApproachToDockPoint(approachPoint);
            }

            if (!_lastWalkArrived)
            {
                LogWarn("Sit cancelled: Mita could not reach the dock point. This prevents a forced snap to the chair.");
                StopNativeWalking();
                RestoreOriginalState();
                ResetSitRuntimeFlags();
                ReleaseActionControl();
                yield break;
            }

            // 实际停下的位置才是起身应返回的安全点。
            // 不强制把她先滑到 approachPoint，避免又出现“走完后被拉一小段”的违和感。
            Vector3 standExitNow = _mitaAvatar.transform.position;
            _standExitRootPos = new Vector3(standExitNow.x, standExitNow.y, standExitNow.z);
            _standExitRootRot = finalSitRootRot;
            _hasStandExitPose = true;

            // 只原地转身，不在这个阶段移动到座面内侧。
            // 位置移动放到坐下动画期间完成，看起来更像“走到旁边 -> 转身 -> 坐下”。
            yield return AlignRotationOnly(finalSitRootRot, AlignDuration);

            HijackAnimationAndIK();

            float seatY = ProbeSeatY(target, seat);
            float desiredPelvisY = seatY + SeatPelvisPadding;
            float currentPelvisY = GetCurrentPelvisY();
            float heightOffset = desiredPelvisY - currentPelvisY;

            // 脚点：优先任何“合理支撑面”，不是只找地板；如果没有支撑面，允许悬空垂脚。
            float footRayY = seatY + FootProbeRayHeightAboveSeat;
            Vector3 leftRay = finalSitRootPos + (finalSitRootRot * new Vector3(-FootSideOffset, 0f, FootForwardOffset));
            Vector3 rightRay = finalSitRootPos + (finalSitRootRot * new Vector3(FootSideOffset, 0f, FootForwardOffset));
            leftRay.y = footRayY;
            rightRay.y = footRayY;

            FootProbeResult leftFoot = ProbeFootTarget(leftRay, true, target.gameObject, seatY);
            FootProbeResult rightFoot = ProbeFootTarget(rightRay, false, target.gameObject, seatY);

            _leftFootTarget = leftFoot.Position;
            _rightFootTarget = rightFoot.Position;
            _leftFootHasSupport = leftFoot.HasSupport;
            _rightFootHasSupport = rightFoot.HasSupport;
            _leftFootGuardEnabled = leftFoot.HasSupport;
            _rightFootGuardEnabled = rightFoot.HasSupport;
            _leftFootTargetWeight = leftFoot.HasSupport ? SupportedFootWeight : UnsupportedFootWeight;
            _rightFootTargetWeight = rightFoot.HasSupport ? SupportedFootWeight : UnsupportedFootWeight;

            // 记录起身阶段使用的脚部最低线。
            // 注意：只有真正打到承重面的脚才启用防穿地 guard。
            // 悬空脚没有“地面标准线”，否则会把空气当成地板，起身时反而导致腿被硬拉。
            _leftFootGroundLineY = _leftFootTarget.y;
            _rightFootGroundLineY = _rightFootTarget.y;
            _hasFootGroundLines = _leftFootGuardEnabled || _rightFootGuardEnabled;

            UpdateDebugSpheres(_leftFootTarget, _rightFootTarget, new Vector3(finalSitRootPos.x, desiredPelvisY, finalSitRootPos.z));

            Vector3 rootSlideStart = _mitaAvatar.transform.position;
            Vector3 rootSlideEnd = new Vector3(finalSitRootPos.x, rootSlideStart.y, finalSitRootPos.z);

            _lockedRootPos = rootSlideStart;
            _lockedRootRot = finalSitRootRot;
            _isSittingLocked = true;
            _currentRootY = rootSlideStart.y;
            _leftFootIkWeight = 0f;
            _rightFootIkWeight = 0f;

            try
            {
                _animator.CrossFade(StartSitStateName, 0.18f);
            }
            catch (Exception e)
            {
                LogWarn("CrossFade failed. State name may be different: " + e.Message);
            }

            float startY = _currentRootY;
            float targetY = startY + heightOffset;
            float timer = 0f;

            while (timer < SitDownDuration)
            {
                timer += Time.deltaTime;
                float t = Mathf.Clamp01(timer / SitDownDuration);

                // 支撑脚更快锁死；悬空脚只给中等权重，避免硬拉出诡异姿势。
                float footLock = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(t * 3.4f));
                _leftFootIkWeight = footLock * _leftFootTargetWeight;
                _rightFootIkWeight = footLock * _rightFootTargetWeight;

                // 坐下动画开始后再把 root 从停靠点平滑滑入座面，不再提前瞬移。
                float slideT = Mathf.Clamp01((t - SitRootSlideDelayRatio) / (1f - SitRootSlideDelayRatio));
                slideT = Mathf.SmoothStep(0f, 1f, slideT);
                _lockedRootPos = Vector3.Lerp(rootSlideStart, rootSlideEnd, slideT);
                _currentRootY = EvaluateSitRootY(startY, targetY, t);
                yield return null;
            }

            _lockedRootPos = rootSlideEnd;
            _currentRootY = targetY;
            _leftFootIkWeight = _leftFootTargetWeight;
            _rightFootIkWeight = _rightFootTargetWeight;

            LogInfo("Sit locked. seatY=" + seatY.ToString("F3")
                + ", rootY=" + _currentRootY.ToString("F3")
                + ", footCase=" + GetFootCaseLabel()
                + ", leftSupport=" + leftFoot.HasSupport
                + ", rightSupport=" + rightFoot.HasSupport);
        }

        private IEnumerator WalkToDockPoint(Vector3 dockPoint)
        {
            _lastWalkArrived = false;
            if (_mitaScript == null || _mitaAvatar == null) yield break;

            GameObject walkMarker = new GameObject("Mita_sit_WalkMarker");
            walkMarker.transform.position = dockPoint;

            try
            {
                PulseMovementOwnership(false);
                NativeAiWalkToTarget(walkMarker.transform);
            }
            catch (Exception e)
            {
                LogWarn("AiWalkToTarget failed, fallback to guarded manual approach: " + e.Message);
            }

            float timeout = WalkTimeout;
            float pulseTimer = 0f;
            float repathTimer = NativeRepathInterval;

            while (timeout > 0f)
            {
                if (_mitaAvatar == null) break;

                Vector2 a = new Vector2(_mitaAvatar.transform.position.x, _mitaAvatar.transform.position.z);
                Vector2 b = new Vector2(dockPoint.x, dockPoint.z);
                float dist = Vector2.Distance(a, b);
                if (dist < WalkStopDistance)
                {
                    _lastWalkArrived = true;
                    break;
                }

                pulseTimer -= Time.deltaTime;
                repathTimer -= Time.deltaTime;

                if (pulseTimer <= 0f)
                {
                    PulseMovementOwnership(false);
                    pulseTimer = NativeControlPulseInterval;
                }

                // 有时原生逻辑会吃掉路径，或者刚好被自动跟随覆盖。控制权锁挡住外部请求后，
                // 这里周期性重发一次“内部寻路”，确保不是等到 timeout 后再突然滑到座椅。
                if (repathTimer <= 0f)
                {
                    bool needRepath = true;
                    try
                    {
                        if (_navAgent != null && _navAgent.enabled && _navAgent.isOnNavMesh)
                            needRepath = !_navAgent.hasPath || _navAgent.pathPending || _navAgent.remainingDistance > dist + 0.4f;
                    }
                    catch { needRepath = true; }

                    if (needRepath && walkMarker != null)
                        NativeAiWalkToTarget(walkMarker.transform);

                    repathTimer = NativeRepathInterval;
                }

                timeout -= Time.deltaTime;
                yield return null;
            }

            if (walkMarker != null) Destroy(walkMarker);
            StopNativeWalking();

            if (!_lastWalkArrived)
                LogWarn("Native walk timed out or was interrupted before reaching dock point.");
        }

        private IEnumerator ManualApproachToDockPoint(Vector3 dockPoint)
        {
            if (_mitaAvatar == null) yield break;

            StopNativeWalking();
            TryPlayOriginalWalkPose();

            float timeout = ManualApproachTimeout;
            while (timeout > 0f)
            {
                if (_mitaAvatar == null) break;

                Vector3 current = _mitaAvatar.transform.position;
                Vector3 target = new Vector3(dockPoint.x, current.y, dockPoint.z);
                Vector3 flatDelta = target - current;
                flatDelta.y = 0f;

                if (flatDelta.magnitude < ManualApproachStopDistance)
                {
                    _lastWalkArrived = true;
                    break;
                }

                PulseMovementOwnership(true);

                Vector3 next = Vector3.MoveTowards(current, target, ManualApproachSpeed * Time.deltaTime);
                _mitaAvatar.transform.position = next;

                if (flatDelta.sqrMagnitude > 0.0004f)
                {
                    Quaternion look = Quaternion.LookRotation(flatDelta.normalized, Vector3.up);
                    _mitaAvatar.transform.rotation = Quaternion.Slerp(_mitaAvatar.transform.rotation, look, Mathf.Clamp01(Time.deltaTime * 7.5f));
                }

                timeout -= Time.deltaTime;
                yield return null;
            }

            StopNativeWalking();

            if (!_lastWalkArrived)
                LogWarn("Manual approach also failed to reach dock point within timeout.");
        }

        private IEnumerator AlignRotationOnly(Quaternion rootRot, float duration)
        {
            if (_mitaAvatar == null) yield break;

            Vector3 p0 = _mitaAvatar.transform.position;
            Quaternion r0 = _mitaAvatar.transform.rotation;

            float timer = 0f;
            while (timer < duration)
            {
                timer += Time.deltaTime;
                float t = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(timer / duration));

                // 只转身，不平移。入座 root 的 XZ 移动交给 SitDownDuration 阶段。
                _mitaAvatar.transform.position = p0;
                _mitaAvatar.transform.rotation = Quaternion.Slerp(r0, rootRot, t);
                yield return null;
            }

            _mitaAvatar.transform.position = p0;
            _mitaAvatar.transform.rotation = rootRot;
        }

        // ============================================================================================
        // 解锁与还原
        // ============================================================================================
        private IEnumerator UnlockRoutine(bool resumeFollowPlayer)
        {
            float startL = _leftFootIkWeight;
            float startR = _rightFootIkWeight;

            Vector3 startRootPos = _mitaAvatar != null
                ? new Vector3(_lockedRootPos.x, _currentRootY, _lockedRootPos.z)
                : _lockedRootPos;

            Quaternion startRootRot = _lockedRootRot;
            Vector3 safeStandPos = GetSafeStandExitRootPosition();
            Quaternion safeStandRot = _hasStandExitPose ? _standExitRootRot : _lockedRootRot;

            // 旧流程看起来像“保持坐姿飞起来”，因为 root 已经在移动，但动画仍停在坐姿末帧。
            // 这里优先把 StartSit 状态按 1 -> 0 倒采样；如果失败，再切回原 Controller 的 Walk 姿态作为兜底。
            bool canReverseSitPose = TrySampleSitState(1f);
            if (!canReverseSitPose)
                PreviewOriginalStandingPose();

            float timer = 0f;
            while (timer < StandUpDuration)
            {
                timer += Time.deltaTime;
                float rawT = Mathf.Clamp01(timer / StandUpDuration);
                float t = Mathf.SmoothStep(0f, 1f, rawT);

                if (canReverseSitPose)
                {
                    // 坐下动画反向采样：姿态从“坐姿末帧”逐渐回到“站姿起点”。
                    // 这不是完整物理起身，但比坐姿整体上浮自然很多。
                    TrySampleSitState(1f - t);
                }

                // 脚部不要一开始就释放。
                // 先让脚作为“支点”踩住，再让身体站起来；最后才释放给原生动画。
                _leftFootIkWeight = EvaluateStandFootWeight(startL, rawT, true);
                _rightFootIkWeight = EvaluateStandFootWeight(startR, rawT, false);

                // 姿态先开始起身，再移动 root 回安全站立点，避免“坐着飞起来”。
                float moveT = Mathf.Clamp01((rawT - StandRootMoveDelayRatio) / (1f - StandRootMoveDelayRatio));
                moveT = Mathf.SmoothStep(0f, 1f, moveT);

                Vector3 p = Vector3.Lerp(startRootPos, safeStandPos, moveT);

                // 以脚的最低线作为保护线。
                // 如果反向采样的姿态导致脚骨低于锁定脚点，就临时把 root 整体抬高一点。
                // 这个抬高会被限制在很小范围内，避免再次出现“整个人飞起来”。
                p.y += GetFootGroundGuardLift();

                _lockedRootPos = new Vector3(p.x, _lockedRootPos.y, p.z);
                _currentRootY = p.y;
                _lockedRootRot = Quaternion.Slerp(startRootRot, safeStandRot, moveT);

                yield return null;
            }

            _leftFootIkWeight = 0f;
            _rightFootIkWeight = 0f;
            _lockedRootPos = safeStandPos;
            _currentRootY = safeStandPos.y;
            _lockedRootRot = safeStandRot;

            if (_mitaAvatar != null)
            {
                _mitaAvatar.transform.position = safeStandPos;
                _mitaAvatar.transform.rotation = safeStandRot;
            }

            if (_animator != null)
            {
                try { _animator.speed = _originalAnimatorSpeed; }
                catch { }
            }

            _isSittingLocked = false;

            RestoreOriginalState();
            HideDebugSpheres();
            ResetSitRuntimeFlags();

            // 强制洗状态机，降低还原后滑行概率。
            if (_animator != null)
            {
                try
                {
                    _animator.applyRootMotion = true;
                    _animator.Play(WalkStateName, 0, 0f);
                    _animator.Update(0f);
                }
                catch (Exception e)
                {
                    LogDebug("Walk reset skipped: " + e.Message);
                }
            }

            yield return new WaitForSeconds(0.1f);

            if (resumeFollowPlayer)
            {
                ReleaseActionControl();
                TryResumeNativeFollowPlayer();
            }

            LogInfo("Mita restored.");
        }


        private bool TrySampleSitState(float normalizedTime)
        {
            if (_animator == null) return false;

            try
            {
                _animator.enabled = true;
                _animator.applyRootMotion = false;
                _animator.cullingMode = AnimatorCullingMode.AlwaysAnimate;
                _animator.speed = 0f;
                _animator.Play(StartSitStateName, 0, Mathf.Clamp01(normalizedTime));
                _animator.Update(0f);
                return true;
            }
            catch (Exception e)
            {
                LogDebug("Reverse sit sampling skipped: " + e.Message);
                return false;
            }
        }

        private void PreviewOriginalStandingPose()
        {
            if (_animator == null || _originalController == null) return;

            try
            {
                _animator.runtimeAnimatorController = _originalController;
                _animator.applyRootMotion = false;
                _animator.cullingMode = AnimatorCullingMode.AlwaysAnimate;
                _animator.speed = _originalAnimatorSpeed;
                _animator.enabled = true;
                _animator.CrossFade(WalkStateName, 0.10f);
                _animator.Update(0f);
            }
            catch (Exception e)
            {
                LogDebug("PreviewOriginalStandingPose skipped: " + e.Message);
            }
        }

        private float EvaluateSitRootY(float startY, float targetY, float t)
        {
            float y = Mathf.SmoothStep(startY, targetY, t);

            // 代码层面的轻微“落座缓冲”：接近坐稳时略微过冲，再回弹到目标高度。
            // 这不能替代高质量动画，但可以减少线性下沉的机械感。
            float settleT = Mathf.Clamp01((t - SitSettleStartRatio) / (1f - SitSettleStartRatio));
            if (settleT > 0f && settleT < 1f)
            {
                float direction = targetY >= startY ? 1f : -1f;
                float damp = 1f - settleT;
                float bounce = Mathf.Sin(settleT * Mathf.PI) * SitSettleBounceAmplitude * damp * direction;
                y += bounce;
            }

            return y;
        }

        private float EvaluateStandFootWeight(float startWeight, float rawT, bool leftFoot)
        {
            if (startWeight <= 0.001f) return 0f;

            bool hasSupport = leftFoot ? _leftFootHasSupport : _rightFootHasSupport;

            // 悬空脚：没有真实承重面，所以不能使用“踩住地面”的逻辑。
            // 让它在起身早期就释放给反向坐起/站立动画，避免脚像被空气钉住，或者把 root 无意义地顶起来。
            if (!hasSupport)
            {
                if (rawT <= UnsupportedStandFootReleaseStartRatio)
                    return startWeight;

                float dangleReleaseT = Mathf.Clamp01((rawT - UnsupportedStandFootReleaseStartRatio) /
                                                      (UnsupportedStandFootReleaseEndRatio - UnsupportedStandFootReleaseStartRatio));
                dangleReleaseT = Mathf.SmoothStep(0f, 1f, dangleReleaseT);
                return Mathf.Lerp(startWeight, 0f, dangleReleaseT);
            }

            // 支撑脚：起身前半段保持脚部支撑，不让脚跟着反向动画穿过支撑面。
            if (rawT < StandFootLockHoldRatio)
                return startWeight;

            // 后半段平滑释放给原生站立/走路动画。
            float releaseT = Mathf.Clamp01((rawT - StandFootLockHoldRatio) / (StandFootReleaseEndRatio - StandFootLockHoldRatio));
            releaseT = Mathf.SmoothStep(0f, 1f, releaseT);
            float w = Mathf.Lerp(startWeight, 0f, releaseT);

            // 如果当前脚骨仍然低于保护线，延迟释放。
            // 这就是“以脚高度作为标准线”的核心：动画可以起身，但不能让支撑脚先穿进地面/支撑面。
            bool guardEnabled = leftFoot ? _leftFootGuardEnabled : _rightFootGuardEnabled;
            if (guardEnabled && _hasFootGroundLines && rawT < StandFootReleaseEndRatio)
            {
                Vector3 footPos = GetCurrentFootPosition(leftFoot);
                float lineY = leftFoot ? _leftFootGroundLineY : _rightFootGroundLineY;
                if (footPos != Vector3.zero && footPos.y < lineY + FootGroundGuardMargin)
                    w = Mathf.Max(w, startWeight * 0.45f);
            }

            return w;
        }

        private float GetFootGroundGuardLift()
        {
            if (!_hasFootGroundLines) return 0f;

            float lift = 0f;

            // 只有有真实承重面的脚才参与 root lift。
            // 单脚悬空时，悬空脚不会把全身顶起来；双脚悬空时，这里自然返回 0。
            if (_leftFootGuardEnabled)
            {
                Vector3 left = GetCurrentFootPosition(true);
                if (left != Vector3.zero)
                    lift = Mathf.Max(lift, (_leftFootGroundLineY + FootGroundGuardMargin) - left.y);
            }

            if (_rightFootGuardEnabled)
            {
                Vector3 right = GetCurrentFootPosition(false);
                if (right != Vector3.zero)
                    lift = Mathf.Max(lift, (_rightFootGroundLineY + FootGroundGuardMargin) - right.y);
            }

            // 保护性抬高只处理瞬时穿地，不承担真正起身。限制幅度避免又变成“飞起来”。
            return Mathf.Clamp(lift, 0f, 0.18f);
        }

        private string GetFootCaseLabel()
        {
            if (_leftFootHasSupport && _rightFootHasSupport) return "BothSupported";
            if (_leftFootHasSupport && !_rightFootHasSupport) return "LeftSupported_RightDangling";
            if (!_leftFootHasSupport && _rightFootHasSupport) return "RightSupported_LeftDangling";
            return "BothDangling";
        }

        private void ForceRestoreIfNeeded()
        {
            try
            {
                StopCurrentActionRoutine();
                _isSittingLocked = false;
                _leftFootIkWeight = 0f;
                _rightFootIkWeight = 0f;
                RestoreOriginalState();
                HideDebugSpheres();
                ResetSitRuntimeFlags();
                ReleaseActionControl();
            }
            catch { }
        }

        // ============================================================================================
        // 接管与备份
        // ============================================================================================
        internal static bool ShouldBlockNativeMagnet(MitaPerson instance)
        {
            if (!IsStaticActionControlAlive()) return false;
            return s_actionControlActive && IsControlledMita(instance);
        }

        internal static bool ShouldBlockExternalAiWalk(MitaPerson instance)
        {
            if (!IsStaticActionControlAlive()) return false;
            if (!s_actionControlActive) return false;
            if (s_allowInternalNativeWalk) return false;
            return IsControlledMita(instance);
        }

        private static bool IsStaticActionControlAlive()
        {
            if (!s_actionControlActive) return false;

            if (s_actionControlExpiresAt > 0f && Time.realtimeSinceStartup > s_actionControlExpiresAt)
            {
                ClearStaticActionControl();
                return false;
            }

            return true;
        }

        private static void ClearStaticActionControl()
        {
            s_actionControlActive = false;
            s_allowInternalNativeWalk = false;
            s_controlledMita = null;
            s_controlOwner = null;
            s_actionControlExpiresAt = 0f;
        }

        private void CheckActionControlWatchdog()
        {
            if (!s_actionControlActive) return;
            if (s_actionControlExpiresAt <= 0f || Time.realtimeSinceStartup <= s_actionControlExpiresAt) return;

            LogWarn("Action control watchdog expired. Force releasing movement ownership lock.");

            if (s_controlOwner == this)
                ForceRestoreIfNeeded();
            else
                ClearStaticActionControl();
        }

        private static bool IsControlledMita(MitaPerson instance)
        {
            if (instance == null) return false;
            if (s_controlledMita == null) return true;

            try
            {
                if (instance == s_controlledMita) return true;
                if (instance.gameObject != null && s_controlledMita.gameObject != null)
                    return instance.gameObject == s_controlledMita.gameObject;
            }
            catch { }

            // MiSide 主场景通常只有一个 MitaPerson。比较失败时宁可保守阻断，避免她被原生 Follow 抢回。
            return true;
        }

        private void AcquireActionControl()
        {
            s_controlledMita = _mitaScript;
            s_controlOwner = this;
            s_actionControlActive = true;
            s_allowInternalNativeWalk = false;
            s_actionControlExpiresAt = Time.realtimeSinceStartup + ActionControlTimeout;
            PulseMovementOwnership(false);
        }

        private void ReleaseActionControl()
        {
            if (s_controlOwner == this || s_controlledMita == _mitaScript || _mitaScript == null || s_controlledMita == null)
                ClearStaticActionControl();
        }

        private void NativeAiWalkToTarget(Transform target)
        {
            if (_mitaScript == null || target == null) return;

            try
            {
                s_allowInternalNativeWalk = true;
                _mitaScript.MagnetOff();
                _mitaScript.AiWalkToTarget(target);
            }
            finally
            {
                s_allowInternalNativeWalk = false;
            }
        }

        private void PulseMovementOwnership(bool hardStopAgent)
        {
            if (s_actionControlActive && s_controlOwner == this)
                s_actionControlExpiresAt = Time.realtimeSinceStartup + ActionControlTimeout;

            try { if (_mitaScript != null) _mitaScript.MagnetOff(); } catch { }

            if (!hardStopAgent) return;

            try
            {
                if (_navAgent != null && _navAgent.enabled && _navAgent.isOnNavMesh)
                {
                    _navAgent.isStopped = true;
                    _navAgent.ResetPath();
                    _navAgent.velocity = Vector3.zero;
                }
            }
            catch { }
        }

        private void TryResumeNativeFollowPlayer()
        {
            if (_mitaScript == null || Camera.main == null) return;

            try
            {
                _mitaScript.MagnetOff();
                _mitaScript.AiWalkToTarget(Camera.main.transform);
            }
            catch (Exception e)
            {
                LogDebug("Resume follow skipped: " + e.Message);
            }
        }

        private void ResetSitRuntimeFlags()
        {
            _hasStandExitPose = false;
            _hasFootGroundLines = false;
            _leftFootHasSupport = false;
            _rightFootHasSupport = false;
            _leftFootGuardEnabled = false;
            _rightFootGuardEnabled = false;
            _lastWalkArrived = false;
            _originalLeftLegBendGoal = null;
            _originalRightLegBendGoal = null;
            _hasOriginalLeftLegBendWeight = false;
            _hasOriginalRightLegBendWeight = false;
        }

        private bool ResolveMitaReferences()
        {
            _mitaAvatar = GameObject.Find("MitaPerson Mita");
            _mitaScript = UnityEngine.Object.FindObjectOfType<MitaPerson>();

            if (_mitaAvatar == null && _mitaScript != null) _mitaAvatar = _mitaScript.gameObject;
            if (_mitaAvatar == null) return false;

            _animator = _mitaAvatar.GetComponent<Animator>();
            if (_animator == null) _animator = _mitaAvatar.GetComponentInChildren<Animator>();

            _fbbik = _mitaAvatar.GetComponent<FullBodyBipedIK>();
            if (_fbbik == null) _fbbik = _mitaAvatar.GetComponentInChildren<FullBodyBipedIK>();

            _navAgent = _mitaScript != null ? _mitaScript.GetComponent<NavMeshAgent>() : _mitaAvatar.GetComponent<NavMeshAgent>();

            return _animator != null && _fbbik != null && _mitaScript != null;
        }

        private void CaptureOriginalState()
        {
            if (_hasBackup) return;

            if (_animator != null)
            {
                _originalController = _animator.runtimeAnimatorController;
                _originalApplyRootMotion = _animator.applyRootMotion;
                _originalCullingMode = _animator.cullingMode;
                _originalAnimatorSpeed = _animator.speed;
            }

            _navHadAgent = _navAgent != null;
            if (_navHadAgent)
            {
                _navOriginalEnabled = _navAgent.enabled;
                try { _navOriginalStopped = _navAgent.isStopped; }
                catch { _navOriginalStopped = false; }
                try { _navOriginalUpdatePosition = _navAgent.updatePosition; }
                catch { _navOriginalUpdatePosition = true; }
                try { _navOriginalUpdateRotation = _navAgent.updateRotation; }
                catch { _navOriginalUpdateRotation = true; }
            }

            _animFuncScript = FindScriptByName(_mitaAvatar, "Animator_FunctionsOverride");
            _unitStepScript = FindScriptByName(_mitaAvatar, "Animator_UnitStep");
            _animFuncWasEnabled = _animFuncScript != null && _animFuncScript.enabled;
            _unitStepWasEnabled = _unitStepScript != null && _unitStepScript.enabled;

            if (_fbbik != null)
            {
                _originalSolverWeight = _fbbik.solver.IKPositionWeight;

                _originalBodyTarget = _fbbik.solver.bodyEffector.target;
                _originalLHandTarget = _fbbik.solver.leftHandEffector.target;
                _originalRHandTarget = _fbbik.solver.rightHandEffector.target;
                _originalLFootTarget = _fbbik.solver.leftFootEffector.target;
                _originalRFootTarget = _fbbik.solver.rightFootEffector.target;

                _originalBodyWeight = _fbbik.solver.bodyEffector.positionWeight;
                _originalLHandWeight = _fbbik.solver.leftHandEffector.positionWeight;
                _originalRHandWeight = _fbbik.solver.rightHandEffector.positionWeight;
                _originalLFootWeight = _fbbik.solver.leftFootEffector.positionWeight;
                _originalRFootWeight = _fbbik.solver.rightFootEffector.positionWeight;

                CaptureOriginalLegBendState();
            }

            _hasBackup = true;
        }

        private void HijackAnimationAndIK()
        {
            LockNavAgentForManualPose();

            if (_animFuncScript != null) _animFuncScript.enabled = false;
            if (_unitStepScript != null) _unitStepScript.enabled = false;
            SetCharacterLookEnabled(_mitaAvatar, false);

            if (_animator != null)
            {
                RuntimeAnimatorController controllerToUse = _sitController;

                // 用 OverrideController 包一层，减少 Avatar/Controller 直接替换的不确定性。
                if (!(_sitController is AnimatorOverrideController))
                    controllerToUse = new AnimatorOverrideController(_sitController);

                _animator.runtimeAnimatorController = controllerToUse;
                _animator.applyRootMotion = false;
                _animator.cullingMode = AnimatorCullingMode.AlwaysAnimate;
                _animator.enabled = true;
                _animator.Update(0f);
            }

            if (_fbbik != null)
            {
                // 关键：不要把 target 直接设成 null。
                // 一些原生脚本 / FinalIK 内部路径在 IL2CPP 下会假定 target 存在，null 会导致 Unity 控制台每帧刷 NRE。
                // 用隐藏锚点替代旧 target：既切断旧目标拉扯，又保持引用非空。
                Vector3 root = _mitaAvatar != null ? _mitaAvatar.transform.position : Vector3.zero;
                _fbbik.solver.bodyEffector.target = EnsureEffectorAnchor(ref _bodyEffectorAnchor, "MAT_BodyEffectorAnchor", GetBodyEffectorPosition(root + Vector3.up * 0.9f));
                _fbbik.solver.leftHandEffector.target = EnsureEffectorAnchor(ref _leftHandEffectorAnchor, "MAT_LeftHandEffectorAnchor", GetLeftHandEffectorPosition(root + Vector3.up * 0.9f - Vector3.right * 0.25f));
                _fbbik.solver.rightHandEffector.target = EnsureEffectorAnchor(ref _rightHandEffectorAnchor, "MAT_RightHandEffectorAnchor", GetRightHandEffectorPosition(root + Vector3.up * 0.9f + Vector3.right * 0.25f));
                _fbbik.solver.leftFootEffector.target = EnsureEffectorAnchor(ref _leftFootEffectorAnchor, "MAT_LeftFootEffectorAnchor", GetCurrentFootPosition(true));
                _fbbik.solver.rightFootEffector.target = EnsureEffectorAnchor(ref _rightFootEffectorAnchor, "MAT_RightFootEffectorAnchor", GetCurrentFootPosition(false));

                _fbbik.solver.bodyEffector.positionWeight = 0f;
                _fbbik.solver.leftHandEffector.positionWeight = 0f;
                _fbbik.solver.rightHandEffector.positionWeight = 0f;

                ApplyLegBendGoals();
            }
        }

        private void RestoreOriginalState()
        {
            if (!_hasBackup) return;

            if (_animator != null)
            {
                if (_originalController != null) _animator.runtimeAnimatorController = _originalController;
                _animator.applyRootMotion = _originalApplyRootMotion;
                _animator.cullingMode = _originalCullingMode;
                _animator.speed = _originalAnimatorSpeed;
                _animator.enabled = true;

                try
                {
                    _animator.Rebind();
                    _animator.Update(0f);
                }
                catch { }
            }

            if (_animFuncScript != null) _animFuncScript.enabled = _animFuncWasEnabled;
            if (_unitStepScript != null) _unitStepScript.enabled = _unitStepWasEnabled;
            if (_mitaAvatar != null) SetCharacterLookEnabled(_mitaAvatar, true);

            if (_fbbik != null)
            {
                _fbbik.solver.IKPositionWeight = _originalSolverWeight;

                _fbbik.solver.bodyEffector.target = _originalBodyTarget;
                _fbbik.solver.leftHandEffector.target = _originalLHandTarget;
                _fbbik.solver.rightHandEffector.target = _originalRHandTarget;
                _fbbik.solver.leftFootEffector.target = _originalLFootTarget;
                _fbbik.solver.rightFootEffector.target = _originalRFootTarget;

                RestoreLegBendGoals();

                _fbbik.solver.bodyEffector.positionWeight = _originalBodyWeight;
                _fbbik.solver.leftHandEffector.positionWeight = _originalLHandWeight;
                _fbbik.solver.rightHandEffector.positionWeight = _originalRHandWeight;
                _fbbik.solver.leftFootEffector.positionWeight = _originalLFootWeight;
                _fbbik.solver.rightFootEffector.positionWeight = _originalRFootWeight;
            }

            if (_navHadAgent && _navAgent != null)
            {
                try
                {
                    _navAgent.updatePosition = _navOriginalUpdatePosition;
                    _navAgent.updateRotation = _navOriginalUpdateRotation;
                    _navAgent.enabled = _navOriginalEnabled;
                    if (_navAgent.enabled && _navAgent.isOnNavMesh) _navAgent.isStopped = _navOriginalStopped;
                }
                catch { }
            }

            DestroyEffectorAnchors();
            _hasBackup = false;
        }

        private void StopNativeWalking()
        {
            try { if (_mitaScript != null) _mitaScript.AiShraplyStop(); } catch { }

            try
            {
                if (_navAgent != null && _navAgent.enabled && _navAgent.isOnNavMesh)
                {
                    _navAgent.isStopped = true;
                    _navAgent.ResetPath();
                    _navAgent.velocity = Vector3.zero;
                }
            }
            catch { }
        }

        private void LockNavAgentForManualPose()
        {
            try
            {
                if (_navAgent == null) return;

                if (_navAgent.enabled && _navAgent.isOnNavMesh)
                {
                    _navAgent.isStopped = true;
                    _navAgent.ResetPath();
                    _navAgent.velocity = Vector3.zero;
                }

                // Animator/FinalIK 进入手动锁定后，NavMeshAgent 不能再写 transform。
                // 否则会出现“我们锁坐姿，agent 又把 root 往玩家/路径点拉”的抢权现象。
                _navAgent.updatePosition = false;
                _navAgent.updateRotation = false;
            }
            catch { }
        }

        private void TryPlayOriginalWalkPose()
        {
            if (_animator == null) return;

            try
            {
                if (_originalController != null)
                    _animator.runtimeAnimatorController = _originalController;

                _animator.applyRootMotion = false;
                _animator.cullingMode = AnimatorCullingMode.AlwaysAnimate;
                _animator.enabled = true;
                _animator.CrossFade(WalkStateName, 0.08f);
                _animator.Update(0f);
            }
            catch (Exception e)
            {
                LogDebug("TryPlayOriginalWalkPose skipped: " + e.Message);
            }
        }

        private void StopCurrentActionRoutine()
        {
            if (_actionRoutine != null)
            {
                try { StopCoroutine(_actionRoutine); } catch { }
                _actionRoutine = null;
            }
        }

        // ============================================================================================
        // 环境测量：座面 / 停靠点 / 脚点
        // ============================================================================================
        private struct SeatProbe
        {
            public Bounds Bounds;
            public bool HasBounds;
            public Vector3 ApproachPosition;
            public Vector3 SitRootPosition;
            public Quaternion DockRotation;
            public float HalfDepth;
        }

        private struct FootProbeResult
        {
            public Vector3 Position;
            public bool HasSupport;
        }

        private SeatProbe BuildSeatProbe(Transform target)
        {
            SeatProbe probe = new SeatProbe();
            probe.Bounds = CalculateBounds(target.gameObject, out probe.HasBounds);

            Vector3 forward = Flatten(target.forward);
            if (forward.sqrMagnitude < 0.0001f) forward = Flatten(_mitaAvatar.transform.forward);
            if (forward.sqrMagnitude < 0.0001f) forward = Vector3.forward;
            forward.Normalize();

            Vector3 center = probe.HasBounds ? probe.Bounds.center : target.position;
            float halfDepth = probe.HasBounds ? ProjectedHalfSize(probe.Bounds, forward) : Mathf.Abs(target.lossyScale.z) * 0.5f;
            halfDepth = Mathf.Max(0.12f, halfDepth);

            Vector3 frontEdge = center + forward * halfDepth;

            probe.HalfDepth = halfDepth;
            probe.ApproachPosition = frontEdge + forward * ApproachExtraOffset;
            probe.SitRootPosition = frontEdge - forward * SeatedRootInsetFromFrontEdge;
            probe.DockRotation = Quaternion.LookRotation(forward, Vector3.up);
            return probe;
        }

        private float ProbeSeatY(Transform target, SeatProbe probe)
        {
            // 座面只允许来自目标物体自身/子物体 Collider。
            // 使用最终坐姿 root 附近的多点采样，并选择“较低的可坐上表面”，避免沙发扶手/靠背被当成座面。
            bool hit;
            Vector3 seatPoint = RaycastBestSeatSurface(target.gameObject, probe, out hit);
            if (hit) return seatPoint.y;

            // 兜底：仍然只打目标自身。
            Vector3 rayOrigin = probe.HasBounds
                ? new Vector3(probe.SitRootPosition.x, probe.Bounds.max.y + SeatProbeRayHeightAboveBounds, probe.SitRootPosition.z)
                : target.position + Vector3.up * RaycastHeight;

            Vector3 p = RaycastHighest(rayOrigin, _mitaAvatar, null, target.gameObject, true, out hit);
            if (hit) return p.y;

            if (probe.HasBounds) return probe.Bounds.max.y;
            return target.position.y;
        }

        private Vector3 GetSafeStandExitRootPosition()
        {
            Vector3 pos = _hasStandExitPose
                ? _standExitRootPos
                : new Vector3(_lockedRootPos.x, _mitaAvatar != null ? _mitaAvatar.transform.position.y : _currentRootY, _lockedRootPos.z);

            // 优先相信 NavMesh：角色恢复原生行走前，root 应回到可站立的 NavMesh 高度。
            NavMeshHit navHit;
            if (NavMesh.SamplePosition(pos, out navHit, 1.6f, -1))
                return navHit.position;

            // 没有 NavMesh 时，用向下射线找一个普通承重面。
            bool hit;
            Vector3 rayOrigin = pos + Vector3.up * RaycastHeight;
            Vector3 surface = RaycastHighest(rayOrigin, _mitaAvatar, null, null, false, out hit);
            if (hit)
                return new Vector3(pos.x, surface.y, pos.z);

            return pos;
        }

        private FootProbeResult ProbeFootTarget(Vector3 rayOrigin, bool leftFoot, GameObject seatTarget, float seatY)
        {
            FootProbeResult result = new FootProbeResult();
            bool found;
            Vector3 support = RaycastBestFootSupport(rayOrigin, _mitaAvatar, seatY, out found);

            if (found)
            {
                result.HasSupport = true;
                result.Position = support + Vector3.up * AnkleHeightOffset;
                return result;
            }

            // 没有任何可承重面：允许自然垂脚，而不是把脚硬塞到地面或目标物体上。
            // XZ 使用理想脚点，Y 取“当前脚高”和“座面下垂高度”中更自然的低值。
            Vector3 currentFoot = GetCurrentFootPosition(leftFoot);
            float dangleY = seatY - DanglingFootDropFromSeat;
            float fallbackY = currentFoot != Vector3.zero ? Mathf.Min(currentFoot.y, dangleY) : dangleY;

            result.HasSupport = false;
            result.Position = new Vector3(rayOrigin.x, fallbackY + AnkleHeightOffset, rayOrigin.z);
            return result;
        }

        // 脚部支撑面选择：不区分地板/床/椅子/道具，只要法线和位置合理就可以。
        private Vector3 RaycastBestFootSupport(Vector3 rayOrigin, GameObject avatar, float seatY, out bool found)
        {
            found = false;
            RaycastHit[] hits = Physics.RaycastAll(rayOrigin, Vector3.down, RaycastDistance);

            Vector3 bestPoint = rayOrigin + Vector3.down;
            float bestScore = float.NegativeInfinity;

            foreach (RaycastHit h in hits)
            {
                if (h.collider == null) continue;
                if (h.collider.isTrigger) continue;

                Transform ht = h.collider.transform;
                GameObject go = h.collider.gameObject;
                if (avatar != null && (go == avatar || ht.IsChildOf(avatar.transform))) continue;

                // 只接受大致向上的面。这样可以避免脚点打到柜子侧面、椅腿侧面。
                if (h.normal.y < 0.35f) continue;

                // 脚部支撑通常不会高过座面太多。
                // 这条过滤能避免桌面、扶手、靠背顶面被误认为脚点；
                // 同时仍允许床面、脚凳、平台等接近座面的承重面。
                if (h.point.y > seatY + FootProbeMaxSupportAboveSeat) continue;

                Vector2 hitXZ = new Vector2(h.point.x, h.point.z);
                Vector2 rayXZ = new Vector2(rayOrigin.x, rayOrigin.z);
                float xzDistance = Vector2.Distance(hitXZ, rayXZ);

                // 分数：越高越好，法线越向上越好，越接近理想 XZ 越好。
                float score = h.point.y * 1.0f + h.normal.y * 0.35f - xzDistance * 0.20f;
                if (score > bestScore)
                {
                    bestScore = score;
                    bestPoint = h.point;
                    found = true;
                }
            }

            return bestPoint;
        }

        private Vector3 RaycastBestSeatSurface(GameObject seatTarget, SeatProbe probe, out bool found)
        {
            found = false;
            if (seatTarget == null) return Vector3.zero;

            float originY = probe.HasBounds
                ? probe.Bounds.max.y + SeatProbeRayHeightAboveBounds
                : seatTarget.transform.position.y + RaycastHeight;

            Vector3 side = probe.DockRotation * Vector3.right;
            Vector3 baseOrigin = new Vector3(probe.SitRootPosition.x, originY, probe.SitRootPosition.z);

            List<Vector3> candidates = new List<Vector3>();
            Vector3 p;
            bool hit;

            p = RaycastSeatSurfaceAt(baseOrigin, seatTarget, out hit);
            if (hit) candidates.Add(p);

            p = RaycastSeatSurfaceAt(baseOrigin + side * SeatProbeSideSampleOffset, seatTarget, out hit);
            if (hit) candidates.Add(p);

            p = RaycastSeatSurfaceAt(baseOrigin - side * SeatProbeSideSampleOffset, seatTarget, out hit);
            if (hit) candidates.Add(p);

            if (candidates.Count <= 0)
                return Vector3.zero;

            // 复杂家具常见问题：左右采样可能打到扶手，中心采样打到座垫。
            // 取较低的可坐上表面比取最高点更安全；最高点很容易是扶手/靠背。
            Vector3 best = candidates[0];
            for (int i = 1; i < candidates.Count; i++)
            {
                if (candidates[i].y < best.y)
                    best = candidates[i];
            }

            found = true;
            return best;
        }

        private Vector3 RaycastSeatSurfaceAt(Vector3 rayOrigin, GameObject seatTarget, out bool found)
        {
            found = false;
            RaycastHit[] hits = Physics.RaycastAll(rayOrigin, Vector3.down, RaycastDistance);
            Vector3 best = Vector3.zero;
            float bestY = float.NegativeInfinity;

            foreach (RaycastHit h in hits)
            {
                if (h.collider == null) continue;
                if (h.collider.isTrigger) continue;

                Transform ht = h.collider.transform;
                GameObject go = h.collider.gameObject;

                if (_mitaAvatar != null && (go == _mitaAvatar || ht.IsChildOf(_mitaAvatar.transform))) continue;
                if (go != seatTarget && !ht.IsChildOf(seatTarget.transform)) continue;

                // 只认上表面。靠背、椅腿、侧边不能成为座面。
                if (h.normal.y < 0.35f) continue;

                if (h.point.y > bestY)
                {
                    bestY = h.point.y;
                    best = h.point;
                    found = true;
                }
            }

            return best;
        }

        private Vector3 RaycastHighest(Vector3 rayOrigin, GameObject avatar, GameObject ignoreTarget, GameObject includeOnlyTarget, bool includeOnlyMode, out bool hitSomething)
        {
            hitSomething = false;
            RaycastHit[] hits = Physics.RaycastAll(rayOrigin, Vector3.down, RaycastDistance);
            float highY = float.NegativeInfinity;
            Vector3 best = rayOrigin + Vector3.down;

            foreach (RaycastHit h in hits)
            {
                if (h.collider == null) continue;
                if (h.collider.isTrigger) continue;

                Transform ht = h.collider.transform;
                GameObject go = h.collider.gameObject;

                if (avatar != null && (go == avatar || ht.IsChildOf(avatar.transform))) continue;
                if (ignoreTarget != null && (go == ignoreTarget || ht.IsChildOf(ignoreTarget.transform))) continue;

                if (includeOnlyMode)
                {
                    if (includeOnlyTarget == null) continue;
                    if (go != includeOnlyTarget && !ht.IsChildOf(includeOnlyTarget.transform)) continue;
                }

                if (h.point.y > highY)
                {
                    highY = h.point.y;
                    best = h.point;
                    hitSomething = true;
                }
            }

            return best;
        }

        private Bounds CalculateBounds(GameObject go, out bool hasBounds)
        {
            Bounds result = new Bounds(go.transform.position, Vector3.zero);
            hasBounds = false;

            var colliders = go.GetComponentsInChildren<Collider>(true);
            foreach (var c in colliders)
            {
                if (c == null || c.isTrigger) continue;
                if (!hasBounds) { result = c.bounds; hasBounds = true; }
                else result.Encapsulate(c.bounds);
            }

            var renderers = go.GetComponentsInChildren<Renderer>(true);
            foreach (var r in renderers)
            {
                if (r == null) continue;
                if (!hasBounds) { result = r.bounds; hasBounds = true; }
                else result.Encapsulate(r.bounds);
            }

            return result;
        }

        private float ProjectedHalfSize(Bounds bounds, Vector3 axis)
        {
            axis.Normalize();
            Vector3 c = bounds.center;
            Vector3 e = bounds.extents;
            float min = float.PositiveInfinity;
            float max = float.NegativeInfinity;

            for (int ix = -1; ix <= 1; ix += 2)
            for (int iy = -1; iy <= 1; iy += 2)
            for (int iz = -1; iz <= 1; iz += 2)
            {
                Vector3 corner = c + new Vector3(e.x * ix, e.y * iy, e.z * iz);
                float d = Vector3.Dot(corner, axis);
                if (d < min) min = d;
                if (d > max) max = d;
            }

            return Mathf.Abs(max - min) * 0.5f;
        }

        private Vector3 Flatten(Vector3 v)
        {
            v.y = 0f;
            return v;
        }

        private float GetCurrentPelvisY()
        {
            try
            {
                if (_fbbik != null && _fbbik.references != null && _fbbik.references.pelvis != null)
                    return _fbbik.references.pelvis.position.y;
            }
            catch { }

            return _mitaAvatar != null ? _mitaAvatar.transform.position.y + 0.9f : 0.9f;
        }

        private Vector3 GetCurrentFootPosition(bool left)
        {
            try
            {
                if (_fbbik != null && _fbbik.references != null)
                {
                    Transform foot = left ? _fbbik.references.leftFoot : _fbbik.references.rightFoot;
                    if (foot != null) return foot.position;
                }
            }
            catch { }

            return Vector3.zero;
        }

        private Transform EnsureEffectorAnchor(ref Transform anchor, string name, Vector3 position)
        {
            if (anchor == null)
            {
                GameObject go = new GameObject(name);
                go.hideFlags = HideFlags.HideInHierarchy;
                anchor = go.transform;
            }

            Transform parent = _mitaAvatar != null ? _mitaAvatar.transform : transform;
            if (anchor.parent != parent)
            {
                try { anchor.SetParent(parent, true); } catch { }
            }

            if (position == Vector3.zero && _mitaAvatar != null)
                position = _mitaAvatar.transform.position;

            anchor.position = position;
            anchor.rotation = _mitaAvatar != null ? _mitaAvatar.transform.rotation : Quaternion.identity;
            return anchor;
        }

        private void CaptureOriginalLegBendState()
        {
            _originalLeftLegBendGoal = GetLegBendGoal(true);
            _originalRightLegBendGoal = GetLegBendGoal(false);

            _hasOriginalLeftLegBendWeight = TryGetLegBendWeight(true, out _originalLeftLegBendWeight);
            _hasOriginalRightLegBendWeight = TryGetLegBendWeight(false, out _originalRightLegBendWeight);
        }

        private void ApplyLegBendGoals()
        {
            if (_fbbik == null || _mitaAvatar == null) return;

            UpdateLegBendAnchors();

            if (_leftKneeBendAnchor != null)
            {
                SetLegBendGoal(true, _leftKneeBendAnchor);
                SetLegBendWeight(true, 1f);
            }

            if (_rightKneeBendAnchor != null)
            {
                SetLegBendGoal(false, _rightKneeBendAnchor);
                SetLegBendWeight(false, 1f);
            }
        }

        private void RestoreLegBendGoals()
        {
            if (_fbbik == null) return;

            SetLegBendGoal(true, _originalLeftLegBendGoal);
            SetLegBendGoal(false, _originalRightLegBendGoal);

            if (_hasOriginalLeftLegBendWeight)
                SetLegBendWeight(true, _originalLeftLegBendWeight);
            if (_hasOriginalRightLegBendWeight)
                SetLegBendWeight(false, _originalRightLegBendWeight);
        }

        private void UpdateLegBendAnchors()
        {
            if (_mitaAvatar == null) return;

            Vector3 forward = Flatten(_mitaAvatar.transform.forward);
            if (forward.sqrMagnitude < 0.0001f) forward = Vector3.forward;
            forward.Normalize();

            Vector3 side = Flatten(_mitaAvatar.transform.right);
            if (side.sqrMagnitude < 0.0001f) side = Vector3.right;
            side.Normalize();

            Vector3 leftKnee = GetCurrentKneePosition(true);
            Vector3 rightKnee = GetCurrentKneePosition(false);

            Vector3 leftGoal = leftKnee + forward * KneeBendGoalForwardOffset - side * KneeBendGoalSideOffset + Vector3.up * KneeBendGoalUpOffset;
            Vector3 rightGoal = rightKnee + forward * KneeBendGoalForwardOffset + side * KneeBendGoalSideOffset + Vector3.up * KneeBendGoalUpOffset;

            EnsureEffectorAnchor(ref _leftKneeBendAnchor, "MAT_LeftKneeBendAnchor", leftGoal);
            EnsureEffectorAnchor(ref _rightKneeBendAnchor, "MAT_RightKneeBendAnchor", rightGoal);
        }

        private Vector3 GetCurrentKneePosition(bool left)
        {
            try
            {
                if (_fbbik != null && _fbbik.references != null)
                {
                    object calfObj = GetMemberValue(_fbbik.references, left ? "leftCalf" : "rightCalf");
                    Transform calf = calfObj as Transform;
                    if (calf != null) return calf.position;
                }
            }
            catch { }

            Vector3 root = _mitaAvatar != null ? _mitaAvatar.transform.position : Vector3.zero;
            Vector3 side = _mitaAvatar != null ? _mitaAvatar.transform.right : Vector3.right;
            return root + Vector3.up * 0.48f + side * (left ? -0.13f : 0.13f);
        }

        private object GetLegChain(bool left)
        {
            try
            {
                if (_fbbik == null || _fbbik.solver == null) return null;
                return GetMemberValue(_fbbik.solver, left ? "leftLegChain" : "rightLegChain");
            }
            catch { return null; }
        }

        private object GetLegBendConstraint(bool left)
        {
            object chain = GetLegChain(left);
            if (chain == null) return null;
            return GetMemberValue(chain, "bendConstraint");
        }

        private Transform GetLegBendGoal(bool left)
        {
            object bend = GetLegBendConstraint(left);
            if (bend == null) return null;
            object value = GetMemberValue(bend, "bendGoal");
            return value as Transform;
        }

        private void SetLegBendGoal(bool left, Transform goal)
        {
            object bend = GetLegBendConstraint(left);
            if (bend == null) return;
            SetMemberValue(bend, "bendGoal", goal);
        }

        private bool TryGetLegBendWeight(bool left, out float weight)
        {
            weight = 0f;
            object bend = GetLegBendConstraint(left);
            if (bend == null) return false;
            object value = GetMemberValue(bend, "weight");
            if (value == null) return false;

            try
            {
                weight = Convert.ToSingle(value);
                return true;
            }
            catch { return false; }
        }

        private void SetLegBendWeight(bool left, float weight)
        {
            object bend = GetLegBendConstraint(left);
            if (bend == null) return;
            SetMemberValue(bend, "weight", weight);
        }

        private object GetMemberValue(object obj, string memberName)
        {
            if (obj == null) return null;

            try
            {
                Type t = obj.GetType();
                FieldInfo f = t.GetField(memberName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                if (f != null) return f.GetValue(obj);

                PropertyInfo p = t.GetProperty(memberName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                if (p != null) return p.GetValue(obj, null);
            }
            catch { }

            return null;
        }

        private void SetMemberValue(object obj, string memberName, object value)
        {
            if (obj == null) return;

            try
            {
                Type t = obj.GetType();
                FieldInfo f = t.GetField(memberName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                if (f != null)
                {
                    f.SetValue(obj, value);
                    return;
                }

                PropertyInfo p = t.GetProperty(memberName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                if (p != null)
                    p.SetValue(obj, value, null);
            }
            catch { }
        }

        private Vector3 GetBodyEffectorPosition(Vector3 fallback)
        {
            try { return _fbbik.solver.bodyEffector.position; }
            catch { return fallback; }
        }

        private Vector3 GetLeftHandEffectorPosition(Vector3 fallback)
        {
            try { return _fbbik.solver.leftHandEffector.position; }
            catch { return fallback; }
        }

        private Vector3 GetRightHandEffectorPosition(Vector3 fallback)
        {
            try { return _fbbik.solver.rightHandEffector.position; }
            catch { return fallback; }
        }

        private void DestroyEffectorAnchors()
        {
            DestroyAnchor(ref _bodyEffectorAnchor);
            DestroyAnchor(ref _leftHandEffectorAnchor);
            DestroyAnchor(ref _rightHandEffectorAnchor);
            DestroyAnchor(ref _leftFootEffectorAnchor);
            DestroyAnchor(ref _rightFootEffectorAnchor);
            DestroyAnchor(ref _leftKneeBendAnchor);
            DestroyAnchor(ref _rightKneeBendAnchor);
        }

        private void DestroyAnchor(ref Transform anchor)
        {
            if (anchor == null) return;
            try
            {
                GameObject go = anchor.gameObject;
                anchor = null;
                if (go != null) Destroy(go);
            }
            catch
            {
                anchor = null;
            }
        }

        // ============================================================================================
        // AssetBundle 加载
        // ============================================================================================
        private void InitializeICalls()
        {
            if (_loadFromFileFunc != null && _loadAssetFunc != null) return;

            _loadFromFileFunc = IL2CPP.ResolveICall<LoadFromFileDelegate>("UnityEngine.AssetBundle::LoadFromFile_Internal(System.String,System.UInt32,System.UInt64)");
            _loadAssetFunc = IL2CPP.ResolveICall<LoadAssetDelegate>("UnityEngine.AssetBundle::LoadAsset_Internal(System.String,System.Type)");
        }

        private bool LoadBundleViaICall()
        {
            InitializeICalls();

            if (_sitController != null)
            {
                _isLoaded = true;
                return true;
            }

            string abPath = Path.Combine(Paths.PluginPath, BundleFileName);
            if (!File.Exists(abPath))
            {
                LogWarn("AssetBundle not found: " + abPath);
                return false;
            }

            IntPtr pathPtr = IL2CPP.ManagedStringToIl2Cpp(abPath);
            _bundlePtr = _loadFromFileFunc(pathPtr, 0, 0);
            if (_bundlePtr == IntPtr.Zero)
            {
                LogWarn("LoadFromFile_Internal returned zero pointer.");
                return false;
            }

            IntPtr typePtr = Il2CppType.Of<RuntimeAnimatorController>().Pointer;

            foreach (string assetName in ControllerAssetNames)
            {
                IntPtr namePtr = IL2CPP.ManagedStringToIl2Cpp(assetName);
                IntPtr assetPtr = _loadAssetFunc(_bundlePtr, namePtr, typePtr);
                if (assetPtr == IntPtr.Zero) continue;

                _sitController = Il2CppObjectPool.Get<RuntimeAnimatorController>(assetPtr);
                if (_sitController != null)
                {
                    _isLoaded = true;
                    LogInfo("Sit controller loaded: " + assetName + " / " + _sitController.name);
                    return true;
                }
            }

            LogWarn("No RuntimeAnimatorController found in bundle. Tried: " + string.Join(", ", ControllerAssetNames));
            return false;
        }

        // ============================================================================================
        // 原生脚本 / Character_Look 处理
        // ============================================================================================
        private MonoBehaviour FindScriptByName(GameObject go, string scriptName)
        {
            if (go == null) return null;

            var scripts = go.GetComponentsInChildren<MonoBehaviour>(true);
            foreach (var s in scripts)
            {
                if (s == null) continue;
                try
                {
                    if (s.GetIl2CppType().Name == scriptName) return s;
                }
                catch { }
            }

            return null;
        }

        private void SetCharacterLookEnabled(GameObject avatar, bool enable)
        {
            if (avatar == null) return;

            var components = avatar.GetComponentsInChildren<MonoBehaviour>(true);
            foreach (var c in components)
            {
                if (c == null) continue;

                string typeName = string.Empty;
                try { typeName = c.GetIl2CppType().Name; } catch { }
                if (string.IsNullOrEmpty(typeName) || !typeName.Contains("Character_Look")) continue;

                SetBoolMember(c, "activeBodyIK", enable);
                SetBoolMember(c, "canRotateBody", enable);

                // 不直接禁用整个组件。以后可以保留头/眼 Look，只关身体参与。
                try { c.enabled = true; } catch { }
            }
        }

        private void SetBoolMember(MonoBehaviour script, string memberName, bool value)
        {
            if (script == null) return;

            try
            {
                Type t = script.GetType();
                PropertyInfo p = t.GetProperty(memberName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                if (p != null)
                {
                    p.SetValue(script, value, null);
                    return;
                }

                FieldInfo f = t.GetField(memberName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                if (f != null)
                {
                    f.SetValue(script, value);
                    return;
                }
            }
            catch { }
        }

        // ============================================================================================
        // 调试工具
        // ============================================================================================
        private GameObject FindObjectByNamePart(string namePart)
        {
            if (string.IsNullOrEmpty(namePart)) return null;

            string needle = namePart.Trim().ToLowerInvariant();

            // IL2CPP 版本的 UnityEngine.Resources 通常没有泛型重载。
            // 这里必须使用非泛型 FindObjectsOfTypeAll(Type)，并把返回的 UnityEngine.Object 显式转成 GameObject。
            var all = Resources.FindObjectsOfTypeAll(Il2CppType.Of<GameObject>());
            foreach (UnityEngine.Object obj in all)
            {
                if (obj == null) continue;

                GameObject go = obj.TryCast<GameObject>();
                if (go == null || go.name == null) continue;

                if (go.name.ToLowerInvariant().Contains(needle)) return go;
            }

            return null;
        }


        private void UpdateDebugSpheres(Vector3 left, Vector3 right, Vector3 pelvis)
        {
            // 调试三球默认不创建。需要排查脚点/盆骨高度时，把 EnableDebugMarkers 改为 true。
            if (!EnableDebugMarkers) return;

            if (_debugSphereL == null)
            {
                _debugSphereL = CreateSphere(Color.green, "D_LeftFoot");
                _debugSphereR = CreateSphere(Color.red, "D_RightFoot");
                _debugSpherePelvis = CreateSphere(Color.blue, "D_Pelvis");
            }

            _debugSphereL.SetActive(true);
            _debugSphereR.SetActive(true);
            _debugSpherePelvis.SetActive(true);

            _debugSphereL.transform.position = left;
            _debugSphereR.transform.position = right;
            _debugSpherePelvis.transform.position = pelvis;
        }

        private GameObject CreateSphere(Color color, string name)
        {
            GameObject s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            s.name = name;
            s.transform.localScale = Vector3.one * 0.08f;

            Renderer r = s.GetComponent<Renderer>();
            if (r != null) r.material.color = color;

            Collider c = s.GetComponent<Collider>();
            if (c != null) Destroy(c);

            return s;
        }

        private void HideDebugSpheres()
        {
            if (_debugSphereL != null) _debugSphereL.SetActive(false);
            if (_debugSphereR != null) _debugSphereR.SetActive(false);
            if (_debugSpherePelvis != null) _debugSpherePelvis.SetActive(false);
        }

        private void LogInfo(string msg)
        {
            if (EnableVerboseLog && Plugin.Logger != null)
                Plugin.Logger.LogInfo("[Mita_sit] " + msg);
        }

        private void LogWarn(string msg)
        {
            if (EnableVerboseLog && Plugin.Logger != null)
                Plugin.Logger.LogWarning("[Mita_sit] " + msg);
        }

        private void LogDebug(string msg)
        {
            if (EnableVerboseLog && Plugin.Logger != null)
                Plugin.Logger.LogDebug("[Mita_sit] " + msg);
        }
    }
}
