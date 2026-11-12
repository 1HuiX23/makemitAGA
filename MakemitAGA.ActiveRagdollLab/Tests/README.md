# 长期合同测试

在源码根目录执行：

```sh
dotnet run --project Tests/ARL.Contracts.csproj --configuration Release
```

使用与插件目标相符的 .NET 6 SDK / runtime。此工程没有第三方 NuGet 包，也不读取 `LocalPaths.props`，不需要 MiSide、BepInEx 或 Unity 安装路径。它是普通的可执行测试工程：成功返回退出码 `0`，任一测试组失败返回 `1`；可以直接将上述命令作为 CI 的一个步骤。这里不使用 `dotnet test`，因为没有引入测试发现器或测试运行器包。

`ARL.Contracts.csproj` 通过 `Compile Include` 链接实际生产 `.cs`，不会复制生产算法。新增测试时优先直接链接小型、职责明确的状态/数学实现。不要为测试再实现一套控制器，也不要把整套 Unity 模拟层扩展成新的维护负担。

## 覆盖范围

| 合同 | 检查目的 |
| --- | --- |
| `RuntimeStepClock` / `StepContextCursor` | source 与 Fixed 身份、重复/跳步拒绝、无效采样不消耗序号、绑定 epoch、浮点时间相等仍可代表不同步 |
| `RunSession` | 准备/预热/运行/停止的合法顺序、重复停止、清理失败阻止重启、重试不改写结果、失败准备保留上次正式证据；物理所有者首因与后续停机触发分开，清理不覆盖首因，新尝试不继承旧首因 |
| `AutomaticStopNotice` | 首因保留、通知次数和间隔、回调异常、新会话保留历史但不重放 |
| 两个 test coordinator | 通过端口注入准备、捕获、恢复、写报告失败；验证先捕获后恢复、超时/时钟停止和 completed/rejected 语义 |
| `FoundationPoseWindow` / `InitialGeometryPreflight` | 真正连续稳定的源姿态、累计慢漂移/足部旋转、卡顿/重绑/重复身份、绝对超时与取消；122记录中的深穿透拒绝、B浅层初态保留，以及Hip绑定、悬空、足印、源龄与无效值的独立拒绝 |
| `HipJoinPreparationWindow` / `HipJoinPose` | 接入前双侧几何和真实源时间窗口；20/40/60Hz源、held不冒充新样本、持续不合格超时、支持面/时钟变更、取消及重置；准备等待独立于两拍passive预热，不认证实际接触或物理解算 |
| `HipContactPreparationWindow` / `FootContactMotion` | R1日志中约2.6～2.7mm浅层重叠只取得准备资格；接入后真实样本双时钟至少150ms、20/40/60Hz源、双侧运动/预计角点/身份/足印拒绝、时钟中断和绝对15秒超时；不会代替正式接触的强支撑确认与载荷准入 |
| `SingleLegShadowChain.KneeReference` / `KneeRelativeReference` / `SingleLegAngularAccelerationFeedforward` | 直接链接腿控制器的参考与前馈入口；A/B 接入第一拍沿用 held source、无接触准备到正式 Hip run 的 R/W/A 连续性、20/40/60Hz 源、trace 只清统计、停用/重启；选择/原生边界/接入时钟错误、过期 FF 收据与既有保护拒绝；两臂共用联合地面候选和旧 raw 路径保留 |
| `BilateralLegCoordinator.DynamicContactBoundary` | 真实单腿 Knee 选择接入真实双腿最终读回；A/B 共同输入均通过，双侧各选择项缺失、支持面/足部配对/作用域错误、未知接收器及子模块故障仍拒绝；旧接触作用域保持联合输入关闭，停用后的状态不能复用 |
| `PelvisParentFrame` | 完整父级样本身份、Hip 映射、COM 偏移下的挂点速度及无效旋转拒绝；避免用全零默认结构代替真实父级输入 |
| `PelvisDynamicReceiverIsolation.HipCompletion` | 真实输入记录→双侧反作用完成→下一物理步结算；准备期不同长度、trace 开关、重复/过期/错 epoch/错 source/错 dt、同一局部序号的伪匹配、预算与分配拒绝、额外Y回执跨提交传播及丢失/伪造/超额拒绝、逐次原生写入失败和回滚继续、环形记录回绕；旧骨盆独立分支不要求 Hip 输入 |
| `HipPointResponseAudit.RotationWindow` / `HipAttitudeBudgetSample` | 腿部与接收器各自计数起点不同仍能拼接同一步；旋转峰值前后窗口、历史峰值保留、姿态预算及基线扭矩拼接、过期或跨 epoch 的证据拒绝；诊断缺失不取消命令 |
| `HipBilateralForceSolver` / `HipSystemAllocation` / `HipAnchorTranslationPair.Validation` | 直接运行实际求解与写力前校验；原统一q及保留的127同q正Y候选，九球预算、XZ/净角请求/反作用不变、反馈闭合、不可行fallback和边界舍入回退；当前支撑/参考身份准入，合成多几何输入，不代表真实接触收益 |
| `HipBilateralForceSolver.AllocationAudit` / `HipAllocationAuditContext` / `HipAllocationAuditReport` | 直接执行同输入只读候选；上下文缺失/损坏不改变物理命令，候选1精确复现127 B，候选2保持XZ/完整点反馈并显式减少净Upper角请求，独立检查九项预算和闭合；q=1跳过额外求解、fallback不伪造upper cut，错回执/重复/跨步/trace与重置、摘要容量和无逐拍堆分配。合成输入不认证未来角跟随或接触收益 |
| `HipReferenceTrajectoryMath` | 退化输入回退、解析旋转、速度/停止距离/单步上限、有限输出、姿态归一化、等价四元数符号 |
| `TargetRenderSampleTiming` | 观察和消费提交分离、提交点前退出不消费源采样、时钟读取失败保留已发布包、held 深度、驱动重置保留上次年龄/新鲜度直到下一次发布 |
| `SupportNormalReference` / `SupportReferenceContext` | 原向量预算内单向候选修正、不可达残差；法向优先分配、横向速度增量/制动保留及速度/加速度双球边界；确认/过渡见证选择、混合曝光、前一完成步与支撑身份、卡顿/过期/重绑拒绝；不模拟接触或载荷 |
| `KneeGroundTarget` / `FootGroundReference` / `FootBoxGeometry` | 同一腿链中单独弯膝降低角点而联合足部输入可清地的反例；Foot-only优先、完整八角Box、轴符号不变性、同屈曲分支、Knee/Foot/ROM边界和固定种子多姿态检查。普通接触2mm与无接触准备/浅恢复3mm分开；纯值检查不认证实际接触 |
| 真实Foot Render输入 / `KneeFootGroundInput` | Foot消费后才发布联合包；保留的独立输入回退分支与原Foot R/W/A逐分量完全相同（当前两臂均选择联合输入）；同source不重算、held Fixed不推进Foot、trace不清live；三刚体/支持/epoch/时间匹配，失效与越界精确回退；20/40/60Hz真实源和原Knee Fixed连续执行 |
| `PelvisComFeedback` / `PelvisComReferencePlanner` / `PelvisNormalResponseEvidence` | 法向独立增益、名义阻尼比例、半隐式参考灵敏度与缺口逆映射、原两球限幅和真实制动保护；N−1输入到N已解响应的完整身份/时间/非有限拒绝。独立纯值测试不模拟PhysX |
| `FootControlEvidenceState` | 观察/计算/完成序号分离、统计口径保留、停止快照冻结、值拷贝隔离、重复封存与下一轮重置 |
| `KneeSupportNormalReactionIsolation` / 载荷 trace 边界 | 直接执行膝部反作用与重置，检查当前命令、重复提交保护、故障锁存和显式重新选择；其余载荷所有者检查实际 `ResetTrace` 源码不得清除当前收据/物理步/故障或调用清理入口。后者是源码边界检查，不是原生行为测试 |

## 与游戏工程的边界

游戏 `.csproj` 明确排除了 `Tests/**/*.cs`。此处的替代 `Plugin` 只给会话文本一个测试标识；数学适配层只提供纯数学辅助所需的向量/四元数操作，并使用 `System.Numerics`。Foot 的测试报告接收端只储存发布的值，不模拟真实报告的格式与资格规则。上述替代类型只属于测试程序集，不发布为游戏插件，也不证明 Unity 的 API、IL2CPP 对象寿命或 PhysX 求解语义正确。

`LegKneeReferencePort` 只替代参考入口周围的姿态、阶段、原生状态读回和报告接收端；真实的准备边界、Render原生快照、Knee参考及前馈实现直接参与测试。`FootReferencePort` 链接真实的Render输入、Foot规划器消费、联合包发布/读取，端口仅提供native几何及身份。纯数学直接共用生产PD限值，不另抄数值。测试会注入已满足静稳条件的物理观察值，因此不覆盖整条 `FixedUpdate` 的原生操作，也不证明真实地面准备一定能通过。不要把这些端口扩展成 Rigidbody/接触仿真器。仅在测试项目出现的 `CS0649` 可能源于未链接生产写入者，应先补足有意义的输入/消费测试，不能据此删除生产字段。

`HipPairValidationPort` 只提供写力前的原生快照，直接调用生产 `CanCommit`，不复制校验式或模拟积分。

`DynamicContactBoundaryContracts` 将最终汇总校验直接链接到真实的单腿 Knee 选择；周围的支撑面、接触选择项、碰撞体身份和单腿原生故障结果为注入观察值。测试覆盖 `PrepareDynamicContact` 调用的最终读回条件，不模拟完整准备过程或原生接触；用旧的 B-only 校验会复现 A 的拒绝启动。

`HipCompletionPorts` 只提供完成入口周围的身份、状态观察、报告接收端和可注入异常的原生调用边界。输入记录、完成校验、分配数学、回滚顺序和下一步结算直接链接生产实现；测试只知道哪些调用正常返回，不能推断真实 API 抛异常前是否已产生副作用。旋转窗口也直接链接生产实现，窗口周围的历史帧与格式接收端是测试输入。此端口不模拟力学积分，不认证完整 `FixedUpdate`、接触、原生参数读回或启停过程。

物理参数、原生 `Destroy` 的延迟完成、碰撞忽略读回、接触回调顺序和实际 A/B 运行仍须使用真实游戏编译和实机日志复测。维护时先运行本工程，再正常构建游戏插件；涉及原生生命周期或控制步序的变化还应做相应的故障注入/实机检查。

## 保持测试可维护

状态/数学模块优先通过内部合同断言，例如“不在准备失败时丢失上一轮结果”。腿控制器的局部回归使用同类的测试 partial 接入真实私有入口，仅在端口内连接必要状态，不复制算法或依赖源文件行号；改动该入口时应同步检查端口是否仍对应真实调用顺序。随机数学输入采用固定种子，失败可以复现。测试输出包含组名和断言失败位置，测试返回非零时不应继续打包。

不要将 `bin/`、`obj/`、测试 DLL 或本机路径文件放入源码交付包。当前版本 `0.4.128`；测试项目不改变游戏插件的版本或 `SourceRevision`。
