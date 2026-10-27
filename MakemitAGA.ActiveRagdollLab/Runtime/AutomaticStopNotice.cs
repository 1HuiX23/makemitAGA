using System;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    /// <summary>
    /// One bounded notification for the first cause in a drive session. This is UI/history state,
    /// never a physics gate. Keep the last event through stop/cleanup; successful drive re-entry
    /// starts a new deduplication scope without erasing the previous event. Native console calls
    /// are deferred until LateUpdate, after the safety owners have performed their shutdown.
    /// </summary>
    internal sealed class AutomaticStopNotice
    {
        private const int MaximumAttempts = 3;
        private int _epoch, _recordedEpoch = -1, _sequence, _attempts;
        private string _owner, _reason, _scene, _arm;
        private string _sessionStopCommand = "arl_drive(off)", _eventStopCommand = "arl_drive(off)";
        private int _runId;
        private bool _pending, _consoleCallReturned;
        private float _nextAttemptTime;

        internal bool Pending => _pending;
        internal bool HasNotice => _sequence > 0;

        internal void BeginDriveSession(string stopCommand)
        {
            _epoch++;
            _sessionStopCommand = string.IsNullOrWhiteSpace(stopCommand) ? "arl_drive(off)" : stopCommand;
            _pending = false; // Never replay an old alert as if the newly started run just stopped.
        }

        internal bool Record(string owner, string reason, string scene, int runId, string arm)
        {
            if (HasNotice && _recordedEpoch == _epoch) return false;
            _recordedEpoch = _epoch;
            _sequence++;
            _owner = Clean(owner); _reason = Clean(reason); _scene = Clean(scene); _arm = Clean(arm);
            _runId = runId; _eventStopCommand = _sessionStopCommand;
            _attempts = 0; _nextAttemptTime = 0f; _consoleCallReturned = false; _pending = true;
            return true;
        }

        // A successful return only proves the console API returned; it cannot prove the UI was visible.
        // Three attempts at most. Explicit status always renders the retained event again.
        internal bool TryDeliver(float now, Func<string, bool> print)
        {
            if (!_pending || now < _nextAttemptTime) return false;
            _attempts++;
            try { _consoleCallReturned = print != null && print(GameText()); }
            catch { _consoleCallReturned = false; }
            _pending = !_consoleCallReturned && _attempts < MaximumAttempts;
            _nextAttemptTime = now + 0.5f;
            return true;
        }

        internal string Status()
        {
            if (!HasNotice) return "[LAST_AUTOMATIC_STOP] present=False";
            return "[LAST_AUTOMATIC_STOP] present=True | notice=" + _sequence +
                " | scope=" + (_recordedEpoch == _epoch ? "LAST_STOP_IN_CURRENT_DRIVE_SESSION" : "PREVIOUS_DRIVE_SESSION") +
                " | sourceRevision=" + Plugin.SourceRevision + " | owner=" + _owner + " | reason=" + _reason +
                " | scene=" + _scene + " | runId=" + _runId + " | arm=" + _arm +
                " | consoleAttempts=" + _attempts + " | consoleCallReturned=" + _consoleCallReturned + " | retryPending=" + _pending;
        }

        internal string GameText()
        {
            if (!HasNotice) return "<color=#9fe7ff>最近自动停机：无记录。</color>";
            string history = _recordedEpoch == _epoch ? "" : "（历史记录；之后已重新启动驱动）";
            string guidance = _recordedEpoch == _epoch
                ? "请停止本轮动作，执行 " + _eventStopCommand + " → arl_test_cleanup → arl_test_status，并保存 output.log。"
                : "这是上一驱动阶段的停机记录，不代表当前驱动已停止；当前运行状态见同次 arl_test_status。";
            return "<color=#ff4d4f><b>【ARL 自动停机：" + Category(_reason) + "】</b></color>" + history +
                "\n" + Explanation(_reason) + "\n来源：" + ForGame(_owner, 100) +
                "；实验：" + ForGame(_arm, 100) + "；记录编号：" + _sequence +
                "\n原因：" + ForGame(_reason, 650) +
                "\n" + guidance;
        }

        private static string Category(string reason)
        {
            if (reason.Contains("ORIGINAL_SUPPORT_") || reason.Contains("OWNED_BOX_CONTRACT_CHANGED")) return "支撑环境中断";
            if (reason.StartsWith("SCENE_CHANGE:", StringComparison.Ordinal)) return "场景切换";
            if (reason.Contains("OVER_") || reason.Contains("SAFETY_LIMIT")) return "物理越界";
            if (reason.Contains("EXCEPTION")) return "运行异常";
            if (reason.Contains("ISOLATION") || reason.Contains("SUPPORT_SCOPE")) return "接触隔离异常";
            return "控制保护";
        }

        private static string Explanation(string reason)
        {
            if (reason.Contains("ANKLE_TRANSLATION_TETHER_OVER_30MM")) return "足部与小腿连接点的距离超过30毫米，驱动已触发保护性关闭。";
            if (reason.Contains("INACTIVE_HIERARCHY")) return "原游戏地面所在对象或父对象已停用，本轮支撑面比较中断。";
            if (reason.Contains("COLLIDER_DISABLED")) return "原游戏地面碰撞体已禁用，本轮支撑面比较中断。";
            if (reason.Contains("ORIGINAL_SUPPORT_")) return "原游戏地面的状态不再满足实验要求；具体对象和状态已记录到日志。";
            return "检测到保护性停机。代理贴回骨骼表示进入预览跟随，不能当作物理运行仍然正常。";
        }

        private static string Clean(string text) => string.IsNullOrWhiteSpace(text) ? "<none>" : text.Replace('\r', ' ').Replace('\n', ' ');
        private static string ForGame(string text, int limit)
        {
            string value = text.Length > limit ? text.Substring(0, limit) + "…（完整原因见日志）" : text;
            return value.Replace("<", "＜").Replace(">", "＞");
        }
    }
}
