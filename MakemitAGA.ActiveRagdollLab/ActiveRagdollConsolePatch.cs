using System;
using System.Text;
using HarmonyLib;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab
{
    /// <summary>
    /// V04870: distinct Knee normal coupling scope; current commands share the existing session owners.
    /// Free-space and the unloaded/Foot-only comparison scopes remain distinct.
    /// V04430/V04450 and contact damping remain available in the legacy loaded regression.
    /// V04210 self-weight transfer and all controller/hard-gate settings remain frozen. Joined load/angular/boundary evidence remains read-only;
    /// V04270 velocity stays original; V04310 position12 retains both captured originals for cleanup.
    /// V04280/V04290 retain 12-frame read-only rings; V04250 accounts for torque impulse. ContactPoint APIs remain retired.
    /// </summary>
    public static class ActiveRagdollConsolePatch
    {
        [HarmonyPatch(typeof(ConsoleCommandsGame), "Command")]
        [HarmonyPrefix]
        [HarmonyPriority(Priority.First)]
        public static bool InterceptCommand(string code)
        {
            string command = (code ?? string.Empty).Trim();
            if (!command.StartsWith("arl_", StringComparison.OrdinalIgnoreCase))
                return true;

            try
            {
                if (TryHandle(command))
                    return false;
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogError("[ARL] Command failed: " + e);
                PrintGame("<color=red>ARL 命令失败：</color> " + e.GetType().Name + " / " + e.Message);
                return false;
            }

            PrintGame("<color=yellow>未知 ARL 命令。</color> 输入 arl_help 查看当前维护版命令。\n");
            return false;
        }

        private static bool TryHandle(string command)
        {
            if (IsBare(command, "arl_help"))
            {
                PrintHelp();
                return true;
            }
            if (IsBare(command, "arl_pelvis_test_setup"))
            {
                RunTestSetupBundle(true); return true;
            }
            if (IsBare(command, "arl_test_setup"))
            {
                RunTestSetupBundle();
                return true;
            }
            if (IsBare(command, "arl_test_status"))
            {
                RunTestStatusBundle();
                return true;
            }
            if (IsBare(command, "arl_test_report"))
            {
                RunTestStatusBundle(true);
                return true;
            }
            if (IsBare(command, "arl_test_cleanup"))
            {
                RunTestCleanupBundle();
                return true;
            }
            if (IsBare(command, "arl_pelvis_upper_load_test"))
            {
                PrintGame(ActiveRagdollLabRuntime.StartPelvisUpperLoadTest()); return true;
            }
            if (TryParseCall(command, "arl_pelvis_upper_load_test", out string[] pelvisUpperLoadArgs))
            {
                PrintGame(pelvisUpperLoadArgs.Length == 1 ? ActiveRagdollLabRuntime.StartPelvisUpperLoadTest(pelvisUpperLoadArgs[0]) : ActiveRagdollLabRuntime.StartPelvisUpperLoadTest());
                return true;
            }
            if (IsBare(command, "arl_pelvis_knee_test"))
            {
                PrintGame(ActiveRagdollLabRuntime.StartPelvisKneeTest()); return true;
            }
            if (TryParseCall(command, "arl_pelvis_knee_test", out string[] pelvisKneeArgs))
            {
                PrintGame(pelvisKneeArgs.Length == 1 ? ActiveRagdollLabRuntime.StartPelvisKneeTest(pelvisKneeArgs[0]) : ActiveRagdollLabRuntime.StartPelvisKneeTest());
                return true;
            }
            if (IsBare(command, "arl_pelvis_lower_load_test"))
            {
                PrintGame(ActiveRagdollLabRuntime.StartPelvisLowerLoadTest()); return true;
            }
            if (TryParseCall(command, "arl_pelvis_lower_load_test", out string[] pelvisLowerLoadArgs))
            {
                PrintGame(pelvisLowerLoadArgs.Length == 1 ? ActiveRagdollLabRuntime.StartPelvisLowerLoadTest(pelvisLowerLoadArgs[0]) : ActiveRagdollLabRuntime.StartPelvisLowerLoadTest());
                return true;
            }
            if (IsBare(command, "arl_pelvis_foot_load_test"))
            {
                PrintGame(ActiveRagdollLabRuntime.StartPelvisFootLoadTest()); return true;
            }
            if (TryParseCall(command, "arl_pelvis_foot_load_test", out string[] pelvisFootLoadArgs))
            {
                PrintGame(pelvisFootLoadArgs.Length == 1 ? ActiveRagdollLabRuntime.StartPelvisFootLoadTest(pelvisFootLoadArgs[0]) : ActiveRagdollLabRuntime.StartPelvisFootLoadTest());
                return true;
            }
            if (IsBare(command, "arl_pelvis_contact_test"))
            {
                PrintGame(ActiveRagdollLabRuntime.StartPelvisContactTest()); return true;
            }
            if (TryParseCall(command, "arl_pelvis_contact_test", out string[] pelvisContactArgs))
            {
                PrintGame(pelvisContactArgs.Length == 1 ? ActiveRagdollLabRuntime.StartPelvisContactTest(pelvisContactArgs[0]) : ActiveRagdollLabRuntime.StartPelvisContactTest());
                return true;
            }
            if (IsBare(command, "arl_pelvis_leg_test"))
            {
                PrintGame(ActiveRagdollLabRuntime.StartPelvisLegTest()); return true;
            }
            if (TryParseCall(command, "arl_pelvis_leg_test", out string[] pelvisLegArgs))
            {
                PrintGame(pelvisLegArgs.Length == 1 ? ActiveRagdollLabRuntime.StartPelvisLegTest(pelvisLegArgs[0]) : ActiveRagdollLabRuntime.StartPelvisLegTest());
                return true;
            }
            if (IsBare(command, "arl_pelvis_test"))
            {
                PrintGame(ActiveRagdollLabRuntime.StartPelvisTest()); return true;
            }
            if (TryParseCall(command, "arl_pelvis_test", out string[] pelvisTestArgs))
            {
                PrintGame(pelvisTestArgs.Length == 1 ? ActiveRagdollLabRuntime.StartPelvisTest(pelvisTestArgs[0]) : ActiveRagdollLabRuntime.StartPelvisTest());
                return true;
            }
            if (IsBare(command, "arl_ankle_test"))
            {
                PrintGame(ActiveRagdollLabRuntime.StartAnkleTest());
                return true;
            }
            if (TryParseCall(command, "arl_ankle_test", out string[] ankleTestArgs))
            {
                // Runtime owns the valid arm set. A second whitelist here previously rejected A.
                if (ankleTestArgs.Length != 1)
                {
                    string usage = ActiveRagdollLabRuntime.StartAnkleTest();
                    Plugin.Logger?.LogWarning("[ARL][ANKLE_TEST_BUNDLE][NOT_STARTED] started=False | reason=ARGUMENT_COUNT | " + usage);
                    PrintGame(usage);
                }
                else
                    PrintGame(ActiveRagdollLabRuntime.StartAnkleTest(ankleTestArgs[0]));
                return true;
            }
            if (IsBare(command, "arl_profile_template"))
            {
                PrintGame(ActiveRagdollLabRuntime.EnsureProfileTemplate());
                return true;
            }
            if (IsBare(command, "arl_profile_reload"))
            {
                PrintGame(ActiveRagdollLabRuntime.ReloadProfile());
                return true;
            }
            if (IsBare(command, "arl_profile_status"))
            {
                PrintGame(ActiveRagdollLabRuntime.GetProfileStatus());
                return true;
            }
            if (IsBare(command, "arl_audit"))
            {
                PrintGame(ActiveRagdollLabRuntime.Audit(null));
                return true;
            }
            if (TryParseCall(command, "arl_audit", out string[] auditArgs))
            {
                if (auditArgs.Length != 1 || string.IsNullOrWhiteSpace(auditArgs[0]))
                    PrintGame("用法：arl_audit 或 arl_audit(对象完整名称)");
                else
                    PrintGame(ActiveRagdollLabRuntime.Audit(auditArgs[0]));
                return true;
            }
            if (IsBare(command, "arl_audit_report"))
            {
                PrintGame(ActiveRagdollLabRuntime.WriteAuditReportAndGetSummary());
                return true;
            }
            if (IsBare(command, "arl_build"))
            {
                PrintGame(ActiveRagdollLabRuntime.Build(null));
                return true;
            }
            if (TryParseCall(command, "arl_build", out string[] buildArgs))
            {
                if (buildArgs.Length != 1 || string.IsNullOrWhiteSpace(buildArgs[0]))
                    PrintGame("用法：arl_build 或 arl_build(对象完整名称)");
                else
                    PrintGame(ActiveRagdollLabRuntime.Build(buildArgs[0]));
                return true;
            }

            if (TryParseOnOffCall(command, "arl_visual", out bool visual, out bool visualValid))
            {
                if (visualValid) PrintGame(ActiveRagdollLabRuntime.SetVisual(visual));
                return true;
            }
            if (TryParseOnOffCall(command, "arl_colliders", out bool colliders, out bool collidersValid))
            {
                if (collidersValid) PrintGame(ActiveRagdollLabRuntime.SetColliders(colliders));
                return true;
            }
            if (TryParseOnOffCall(command, "arl_gravity", out bool gravity, out bool gravityValid))
            {
                if (gravityValid) PrintGame(ActiveRagdollLabRuntime.SetGravity(gravity));
                return true;
            }
            if (TryParseOnOffCall(command, "arl_drive", out bool drive, out bool driveValid))
            {
                if (driveValid) PrintGame(ActiveRagdollLabRuntime.SetDrive(drive));
                return true;
            }

            if (TryParseOnOffCall(command, "arl_knee_axis_calibrate", out bool calibrate, out bool calibrateValid))
            {
                if (calibrateValid) PrintGame(ActiveRagdollLabRuntime.SetKneeAxisCalibration(calibrate));
                return true;
            }
            if (IsBare(command, "arl_knee_axis_status"))
            {
                PrintGame(ActiveRagdollLabRuntime.GetKneeAxisCalibrationStatus());
                return true;
            }
            if (IsBare(command, "arl_knee_axis_apply"))
            {
                PrintGame(ActiveRagdollLabRuntime.ApplyCalibratedKneeAxis());
                return true;
            }
            if (IsBare(command, "arl_knee_axis_reset"))
            {
                PrintGame(ActiveRagdollLabRuntime.ResetKneeAxisToInitial());
                return true;
            }

            if (TryParseOnOffCall(command, "arl_hip_stack", out bool stack, out bool stackValid))
            {
                if (stackValid) PrintGame(ActiveRagdollLabRuntime.SetHipExperimentStack(stack));
                return true;
            }
            if (IsBare(command, "arl_hip_stack_status"))
            {
                PrintGame(ActiveRagdollLabRuntime.GetHipExperimentStackStatus());
                return true;
            }
            if (IsBare(command, "arl_leg_freeze_status"))
            {
                PrintGame(ActiveRagdollLabRuntime.GetSingleLegFreezeStatus());
                return true;
            }

            if (IsBare(command, "arl_bilateral_status"))
            {
                PrintGame(ActiveRagdollLabRuntime.GetBilateralStatus());
                return true;
            }

            if (IsBare(command, "arl_foot_requal_status"))
            {
                PrintGame(ActiveRagdollLabRuntime.GetFootSoleRequalificationStatus());
                return true;
            }

            if (IsBare(command, "arl_foot_status"))
            {
                PrintGame(ActiveRagdollLabRuntime.GetFootAnkleStatus());
                return true;
            }

            if (IsBare(command, "arl_knee_safety_status"))
            {
                PrintGame(ActiveRagdollLabRuntime.GetKneeSafetyContextStatus());
                return true;
            }

            if (TryParseOnOffCall(command, "arl_trace", out bool trace, out bool traceValid))
            {
                if (traceValid)
                {
                    string result = ActiveRagdollLabRuntime.SetTrace(trace);
                    Plugin.Logger?.LogInfo("[ARL][COMMAND_RESULT][TRACE] " + result);
                    bool stateMatches = ActiveRagdollLabRuntime.TraceActive == trace;
                    if (!stateMatches)
                    {
                        PrintGame("<color=red>ARL trace STOP：请求未生效。</color> " + result);
                    }
                    else
                    {
                        PrintGame(trace
                            ? "ARL trace 已开启；采集 torso / bilateral leg / Foot 执行器、sole-frame、safety 与 freeze diagnostics；足部实际参考及前馈输入以 FOOT_REFERENCE_V04730 的 arm/traceArm/traceFFInput 为准，完整状态见 BepInEx。"
                            : "ARL trace 已关闭；完整 final 状态见 BepInEx。需要汇总时执行 arl_test_status。");
                    }
                }
                return true;
            }
            if (TryParseOnOffCall(command, "arl_leg_drive", out bool legDrive, out bool legDriveValid))
            {
                if (legDriveValid) PrintGame(ActiveRagdollLabRuntime.SetLegDrive(legDrive));
                return true;
            }
            if (IsBare(command, "arl_leg_status"))
            {
                PrintGame(ActiveRagdollLabRuntime.GetLegStatus());
                return true;
            }
            if (IsBare(command, "arl_reset_pose"))
            {
                PrintGame(ActiveRagdollLabRuntime.ResetPose());
                return true;
            }
            if (IsBare(command, "arl_rollback"))
            {
                PrintGame(ActiveRagdollLabRuntime.Rollback("console"));
                return true;
            }

            return false;
        }

        private static void LogTestBundleItem(string bundle, string step, string result)
        {
            Plugin.Logger?.LogInfo("[ARL][TEST_BUNDLE][" + bundle + "][" + step + "]\n" + (result ?? "<null>"));
        }

        private static void RunTestSetupBundle(bool torsoOnly = false)
        {
            string profile = ActiveRagdollLabRuntime.GetProfileStatus();
            LogTestBundleItem("SETUP", "PROFILE", profile);
            string audit = ActiveRagdollLabRuntime.Audit(null);
            LogTestBundleItem("SETUP", "AUDIT", audit);
            if (!ActiveRagdollLabRuntime.AuditReady)
            {
                PrintGame("<color=red>arl_test_setup STOP：骨骼审计未通过。</color> 完整原因已写入 BepInEx 日志；不要继续 PCA/drive 测试。");
                return;
            }

            string build = torsoOnly ? ActiveRagdollLabRuntime.BuildPelvisTestFoundation() : ActiveRagdollLabRuntime.Build(null);
            LogTestBundleItem("SETUP", "BUILD", build);
            if (!ActiveRagdollLabRuntime.IsBuilt)
            {
                PrintGame("<color=red>arl_test_setup STOP：ARL build 未完成。</color> 完整原因已写入 BepInEx 日志；不要继续测试。");
                return;
            }

            bool setupSafe = true;
            RunBundleStep("SETUP", "COLLIDERS_OFF", () => ActiveRagdollLabRuntime.SetColliders(false), ref setupSafe);
            RunBundleStep("SETUP", "GRAVITY_OFF", () => ActiveRagdollLabRuntime.SetGravity(false), ref setupSafe);
            RunBundleStep("SETUP", "TRACE_OFF", () => ActiveRagdollLabRuntime.SetTrace(false), ref setupSafe);
            if (!setupSafe)
            {
                bool cleanupSafe = true;
                RunBundleStep("SETUP", "ROLLBACK_AFTER_SAFE_STATE_FAILURE", () => ActiveRagdollLabRuntime.Rollback("setup-safe-state-failed"), ref cleanupSafe);
                PrintGame("<color=red>arl_test_setup STOP：post-build 安全状态步骤失败，已尝试整体 rollback。</color> 请把完整 BepInEx 日志发回，不要继续 PCA/drive。");
                return;
            }
            PrintGame(torsoOnly
                ? "<color=#9fe7ff><b>arl_pelvis_test_setup 完成</b></color>：仅创建Pelvis与Chest及两体CharacterJoint，无腿/足刚体或关节。双侧Hip源引用已解析；下一步arl_pelvis_test(b)，无需PCA/drive/Hip stack。"
                : "<color=#9fe7ff><b>arl_test_setup 完成</b></color>：profile / audit / build / colliders OFF / gravity OFF / trace OFF 已一次执行。" + ActiveRagdollLabRuntime.StartPelvisKneeTest());
        }

        private static void RunTestStatusBundle(bool detailed = false)
        {
            Plugin.Logger?.LogInfo((detailed ? "[ARL][TEST_REPORT_SNAPSHOT]\n" : "[ARL][TEST_STATUS_SUMMARY]\n") + ActiveRagdollLabRuntime.GetMaintenanceStatusSnapshot(detailed));
            PrintGame(ActiveRagdollLabRuntime.GetAutomaticStopGameStatus());
            if (ActiveRagdollLabRuntime.PelvisStatusIsCurrentScope)
                PrintGame(ActiveRagdollLabRuntime.GetPelvisTestStatus(false).Split('\n')[0]);
            else
                PrintGame(ActiveRagdollLabRuntime.GetAnkleTestStatus(false).Split('\n')[0]);
            PrintGame(detailed ? "<color=#9fe7ff>完整状态已写入 BepInEx 日志与组件报告。</color>" : "<color=#9fe7ff>状态摘要已记录。</color> 完整结果在停止时保存；需要再次展开时执行 arl_test_report。");
        }

        private static void RunTestCleanupBundle()
        {
            bool cleanupSafe = true;
            if (ActiveRagdollLabRuntime.PelvisTestActive)
                RunBundleStep("CLEANUP", "FINALIZE_PELVIS_TEST", () => ActiveRagdollLabRuntime.StopPelvisTest(), ref cleanupSafe);
            if (ActiveRagdollLabRuntime.AnkleTestActive)
                RunBundleStep("CLEANUP", "FINALIZE_ANKLE_TEST", () => ActiveRagdollLabRuntime.StopAnkleTest(), ref cleanupSafe);
            if (ActiveRagdollLabRuntime.TraceActive)
                RunBundleStep("CLEANUP", "TRACE_OFF_DEFENSIVE", () => ActiveRagdollLabRuntime.SetTrace(false), ref cleanupSafe);
            RunBundleStep("CLEANUP", "LEG_DRIVE_OFF", () => ActiveRagdollLabRuntime.SetLegDrive(false), ref cleanupSafe);
            RunBundleStep("CLEANUP", "DRIVE_OFF", () => ActiveRagdollLabRuntime.SetDrive(false), ref cleanupSafe);
            RunBundleStep("CLEANUP", "ROLLBACK", () => ActiveRagdollLabRuntime.Rollback("test-bundle"), ref cleanupSafe);
            cleanupSafe = cleanupSafe && ActiveRagdollLabRuntime.LastRollbackSucceeded;
            PrintGame(cleanupSafe
                ? "<color=#9fe7ff><b>arl_test_cleanup 完成</b></color>：trace（如仍开启）/ leg drive / torso drive 已关闭并 rollback。"
                : "<color=red><b>arl_test_cleanup STOP：至少一个 teardown 步骤或 rollback 失败。</b></color> 已继续尝试全部清理步骤；请保留现场并发送完整 BepInEx 日志。");
        }

        private static string RunBundleStep(string bundle, string step, Func<string> action, ref bool allSucceeded)
        {
            try
            {
                string result = action != null ? action() : "<null-action>";
                LogTestBundleItem(bundle, step, result);
                return result;
            }
            catch (Exception e)
            {
                allSucceeded = false;
                string result = "EXCEPTION " + e.GetType().Name + " / " + e.Message;
                Plugin.Logger?.LogError("[ARL][TEST_BUNDLE][" + bundle + "][" + step + "] " + result);
                return result;
            }
        }

        private static bool IsBare(string command, string verb)
        {
            return command.Equals(verb, StringComparison.OrdinalIgnoreCase) ||
                   command.Equals(verb + "()", StringComparison.OrdinalIgnoreCase);
        }

        private static void PrintHelp()
        {
            PrintGame("<b>MakemitAGA Active Ragdoll Lab v0.4.89</b>\n" +
                "本轮：独立膝部法向耦合，保留Foot/Lower自重；Upper/Torso重力及Hip反力关闭。\n" +
                "arl_test_setup / arl_pelvis_upper_load_test(b|stop) / arl_test_status / arl_test_cleanup\n" +
                "arl_test_report（仅需重新展开完整诊断时使用）\n" +
                ActiveRagdollLabRuntime.StartPelvisKneeTest() + "\n小腿自重回归arl_pelvis_lower_load_test(b|stop)，Foot自重对照arl_pelvis_foot_load_test(b|stop)，无载荷接触对照arl_pelvis_contact_test(b|stop)，无接触对照arl_pelvis_leg_test(a|b|stop)。\n\n" +
                "骨盆独立范围保留arl_pelvis_test_setup与arl_pelvis_test(b|stop)；两个会话互斥。\n" +
                "arl_profile_template / arl_profile_reload / arl_profile_status\n" +
                "arl_audit[(对象完整名称)] / arl_audit_report / arl_build[(对象完整名称)]\n" +
                "arl_visual(on/off) / arl_colliders(on/off) / arl_gravity(on/off) / arl_drive(on/off)\n" +
                "arl_trace(on/off) / arl_leg_drive(on/off) / arl_leg_status\n" +
                "arl_knee_axis_calibrate(on/off) / arl_knee_axis_status / arl_knee_axis_apply / arl_knee_axis_reset\n" +
                "arl_hip_stack(on/off) / arl_hip_stack_status\n" +
                "arl_bilateral_status / arl_foot_status / arl_knee_safety_status / arl_leg_freeze_status\n" +
                "arl_reset_pose / arl_rollback\n" +
                "任何自动停止立即收尾、保存完整日志；STARTED前的pending并非正式run。");
        }

        private static bool TryParseOnOffCall(string command, string verb, out bool value, out bool valid)
        {
            value = false;
            valid = false;
            if (!TryParseCall(command, verb, out string[] args))
                return false;
            if (args.Length != 1 || !TryParseOnOff(args[0], out value))
            {
                PrintGame("用法：" + verb + "(on/off)");
                return true;
            }
            valid = true;
            return true;
        }

        private static bool TryParseOnOff(string text, out bool value)
        {
            value = false;
            string input = (text ?? string.Empty).Trim();
            if (input.Equals("on", StringComparison.OrdinalIgnoreCase) || input.Equals("true", StringComparison.OrdinalIgnoreCase) || input == "1")
            {
                value = true;
                return true;
            }
            if (input.Equals("off", StringComparison.OrdinalIgnoreCase) || input.Equals("false", StringComparison.OrdinalIgnoreCase) || input == "0")
            {
                value = false;
                return true;
            }
            return false;
        }

        private static bool TryParseCall(string command, string verb, out string[] args)
        {
            args = Array.Empty<string>();
            if (!command.StartsWith(verb, StringComparison.OrdinalIgnoreCase))
                return false;
            string tail = command.Substring(verb.Length).Trim();
            if (tail.Length < 2 || tail[0] != '(' || tail[tail.Length - 1] != ')')
                return false;
            string inner = tail.Substring(1, tail.Length - 2);
            if (string.IsNullOrWhiteSpace(inner))
                return true;
            args = inner.Split(',');
            for (int i = 0; i < args.Length; i++)
                args[i] = args[i].Trim();
            return true;
        }

        private const int MaxGameConsoleCharacters = 6000;
        private const int SafePrefixCharacters = 5200;

        internal static void PrintGame(string text) => TryPrintGame(text);

        internal static bool TryPrintGame(string text)
        {
            string value = text ?? string.Empty;
            if (value.Length > MaxGameConsoleCharacters)
            {
                Plugin.Logger?.LogWarning("[ARL][GameConsole] output-over-budget | chars=" + value.Length + " | budget=" + MaxGameConsoleCharacters);
                Plugin.Logger?.LogInfo("[ARL][GameConsole][FULL_OUTPUT]\n" + value);
                value = value.Substring(0, Math.Min(SafePrefixCharacters, value.Length)) +
                        "\n<color=yellow>游戏内输出已截断；完整内容见 BepInEx 日志或报告文件。</color>";
            }
            try { ConsoleMain.ConsolePrintGame(value); return true; }
            catch (Exception e)
            {
                Plugin.Logger?.LogWarning("[ARL][GAME_CONSOLE_DELIVERY_FAILED] " + e.GetType().Name + ":" + e.Message);
                Plugin.Logger?.LogInfo("[ARL][GameConsole] " + value);
                return false;
            }
        }
    }
}
