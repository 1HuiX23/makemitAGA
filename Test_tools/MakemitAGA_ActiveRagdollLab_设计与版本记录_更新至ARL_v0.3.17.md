# MakemitAGA Active Ragdoll Lab：设计与版本记录（压缩长期记忆版）

> 当前版本：**ARL v0.3.17**  
> 当前阶段：**Bilateral / Full-Body Foundation，Gravity/Colliders 仍 OFF**  
> 文档策略：0.1.x 与 0.2.x 不再逐版本流水账展开，而按两个独立历史章节压缩；0.3.x 及以后继续逐版保留关键因果、实机结果与决定。旧完整历史由备份 ZIP / 旧 `.md` 保留。

---

# 0. 当前恢复入口

## 0.1 当前环境与长期边界

- Unity 2021.3.35f1，Windows IL2CPP，BepInEx Unity.IL2CPP 6.x，.NET 6。
- ARL 的长期目标不是做一个单独“能跟动画摆动”的代理，而是逐步建立可进入 **双腿 → 全身 → Gravity/Contact → Balance → ML-Agents / 高层动作** 的稳定物理身体。
- 不恢复 ConfigurableJoint / FixedJoint 路线。历史上已经确认在当前 IL2CPP/运行环境下不值得继续依赖该方向。**注意：这不等于整个 ARL 已彻底不用 CharacterJoint。** 当前执行 `arl_hip_stack(on)` 后的 frozen Single-Leg production Hip/Knee 都没有 CharacterJoint；但冻结 Two-Body torso 的 Pelvis↔Chest 仍使用 Unity 原生 `UnityEngine.CharacterJoint`。我们自定义的是外层 relative-PD、parent feedforward、Pose-Bridge、recovery/safety，而不是自定义 CharacterJoint 本体。
- Source hierarchy 只读；实验层不修改原角色骨骼层级或 Animator 输出。
- 现阶段 Gravity/Colliders OFF。任何“看起来像接触/支撑”的现象都不能在这一阶段解释成真实 ground reaction。
- 生产物理遵循：**先 observer / counterfactual，后窄 physical A/B；不因一次 run 的视觉印象直接改参数。**

## 0.2 当前冻结 Single-Leg 基线（v0.2.122）

除非后续双腿/全身阶段给出新的可复现因果证据，否则以下内容视为冻结：

- True Healthy Normal parent transport：**v0.2.88 A / OFF**。
- strict mechanical-parent transport：v0.2.48 ON。
- dedicated post-frame recovery support：ON。
- Hip actuator：v0.2.81 inertia/gyro torque，ForceMode.Force。
- Upper/Lower angularDrag：3。
- Hip PD：Kp/Kd = 55/12；MaxA baseline/source/recovery = 40/80/80。
- Knee PD：45/9/40。
- Knee soft off-axis：30/8；MaxA baseline/source/recovery = 20/40/40。
- Normal Angular Acceleration FF：gain 1，bound 20，production ON。
- Production Knee target-W：`TargetVelocityBlend=0.50` filtered/held reference。
- frozen stack激活后的 Hip 与 Knee CharacterJoint 均 absent；translation-only tether validity gate = **30 mm fail-closed**。
- Explicit Uniform Box mass/COM/inertia foundation validated。
- Knee flexion axis使用 source-only/data-driven PCA，必须 Applied + Ready。
- v0.2.122 freeze 不是“数学最优证明”，而是“现有证据下可作为下一阶段稳定基线”。

## 0.3 当前 Bilateral 基线（v0.3.0）

- `SingleLegShadowChain` 是唯一 per-side production controller；Left/Right 不分叉参数或算法。
- 两份相同 frozen controller 共享同一个 Two-Body Pelvis parent。
- v0.3.0 实机：Fixed/Render/Fresh/Held/Drive lifecycle parity 全部 0 mismatch；两侧各自通过 v0.2.122 freeze；`BILATERAL_FROZEN_REPLICATION_PASS`。
- Bilateral coordinator 负责 build/PCA/stack/trace/drive/reset/rollback 与 fail-closed 生命周期，不包含新的 PD 数学。
- 如果一侧 abort，另一侧必须在同一个 coordinator 外层 FixedUpdate 后关闭；不能允许“半个身体继续动态运行”。

## 0.4 v0.3.1 已关闭结论

- Hip common-energy 高值主要集中在共同 Source motion；Normal common/differential 约各半。
- Knee 没有 persistent common locking。
- Normal-only cross-side Hip command → 对侧 Hip error-delta 的 lag1/lag2 相关很弱，不支持 material cross-side tracking contamination。
- 当前 no-Hip-joint + upper-only Hip tether + child-only Knee torque 没有真正 equal-and-opposite leg→Pelvis reaction path，因此 Pelvis correlation 只能解释为 shared-parent/source context，不能写成“腿在驱动 Pelvis”。
- v0.3.1 live coupling observer 已退休。

## 0.5 v0.3.2 已获得的 reaction feasibility 结论

首轮 v0.3.2 在末尾发生一次 Right Knee tether fail-closed，但在此之前已有足够 reaction-CF population：

- CF samples = 5102，pre-abort valid = **5101**。
- phase：Source 1618 / Recovery 224 / active Normal 2758 / Idle 501。
- Hip angular-only predicted Pelvis `|Δα|`：mean ≈ 1.342 rad/s²，max ≈ 7.511；Normal mean ≈ 1.359，max ≈ 7.075。
- **v0.3.5 历史纠错：**上述 angular `Δα` 当时使用 production kinematic Pelvis 的 `Rigidbody.inertiaTensor`；v0.3.4 实机暴露该 tensor 为默认式 `(1,1,1)`，并不代表当前 Pelvis box 的真实 dynamic inertia。因此 v0.3.2/v0.3.3 的 angular-acceleration 绝对量只能作为 legacy/default-tensor witness，不能再作为 future dynamic Pelvis 的物理量级依据；tether linear force/`Δa` 与左右 force cancellation 不受此惯量纠错影响。
- full anchor-transport total `|Δα|`：mean ≈ 2.374，P95 ≈ 5.358，P99 ≈ 7.201，max ≈ 40.983 rad/s²。
- virtual tether linear `|Δa|`：mean ≈ 3.085，P95 ≈ 7.884，P99 ≈ 11.279，max ≈ 104.413 m/s²。
- total reaction cancellation：ALL≈0.279；Source≈0.223；Recovery≈0.278；Normal≈0.309，不能依赖左右腿天然抵消来保证 full reaction 安全。
- total-torque common share：ALL≈0.441；Source≈0.648；Recovery≈0.466；Normal≈0.383。

**决定：**

- full raw tether reaction write-back：**NOT AUTHORIZED**。
- Hip angular-only reaction 的量级相对温和，允许继续作为独立候选路径研究，但尚未自动授权 physical A/B。
- tether force / anchor moment 必须先做 budget / selective attenuation；不能一次性闭合到 Pelvis。
- v0.3.2 的“invalid=1”实际上对应末尾 safety-abort sample；从 v0.3.3 起必须把 **calculation-invalid** 与 **safety-censored** 分开，避免一拍真实安全事件把前面数千拍有效 CF 误判成“数据无效”。

## 0.6 v0.3.2 Knee safety 事件

- side：RIGHT。
- reason：`KNEE_TRANSLATION_ONLY_TETHER_INVALID_ANCHOR`。
- current≈34.002 mm > 30 mm frozen gate。
- phase：NORMAL_TRANSITION；当时不是 SourceTurn，也不是 Recovery。
- abort sample appliedA≈48.87 m/s²，没有卡在60 m/s² final clamp；所以不能简单解释为“因为 clamp 不够”。
- Left 同 run Knee anchor peak≈25.991 mm；Right≈34.002 mm。
- Two-Body 同 run仍 `ALL_GATES_PASS`，30/30 recovery PASS。

**长期处理原则：**

- 30 mm gate 不因单次事件放宽。
- 先保留 pre-abort temporal context，判断是累积 positional debt、transition packet/held history、FF/velocity mismatch还是控制 authority 结构性不足。
- foundation FAIL 与 observer dataset 是否可评分必须分开表示。

## 0.7 v0.3.3 已关闭结论

v0.3.3 第二轮实机为完整 safety-clean 决策 run，reaction-budget 问题已经足够关闭：

- samples/valid/calculationInvalid/safetyCensored = **7135/7135/0/0**。
- phase：Source 2189 / Recovery 637 / active Normal 2995 / Idle 1314。
- Bilateral foundation：clock mismatch `0/0/0/0/0`，failClosed=0，freeze=True/True，aborted=False/False，`BILATERAL_FROZEN_REPLICATION_PASS`。
- Two-Body：39 source events，2 TRUE discontinuity；37/37 evaluable recovery PASS，0 fail，`ALL_GATES_PASS`。
- Normal Hip angular-only：legacy/default-tensor witness P95≈4.987，P99≈6.262，max≈9.604 rad/s²；**v0.3.5 起不再把这些 angular-acceleration 绝对值当作 physical Pelvis 量级**。
- Normal angular P99：tether gain 0.25≈5.864（0.936× angular-only）；0.50≈5.971（0.954×）；1.00≈8.278（1.322×）。这些相对lane结果仍可作为default-tensor下的方向/cancellation witness，但不能替代corrected physical inertia复核。
- Normal tether linear P99：0.25≈3.455，0.50≈6.910，1.00≈13.821 m/s²。
- tether anchor moment 在 ALL/Source/Recovery/Normal 中 dominant samples=4882/1327/414/2254；Normal 约75%，说明 full mechanical closure 的主要难点不是 Hip angular actuator，而是 tether force + lever-arm transport。
- Knee safety 本 run 未 abort；Left/Right Knee peak anchor≈23.623/28.059 mm。Right 仍更靠近30mm gate，因此低频 tail-risk 继续保留观察，但不改 gate。

**决定：**

- full tether reaction：继续 **NOT AUTHORIZED**。
- 25%/50% tether lane 的 angular P99 没有高于 angular-only，不代表可以直接 physical ON，因为 linear momentum injection 仍随 gain 近似线性增加。
- Hip angular-only reaction 当时由两轮 CF 支持进入物理研究阶段；**v0.3.5 发现 dynamic inertia foundation 未验证后，该授权降级为“必须在 analytical physical inertia shadow 上重新复核”**。
- v0.3.3 live reaction-budget observer 退休；不继续扩展更多虚拟 gain lane。
- 下一步必须先建立“真正能接收 Hip angular reaction、但不破坏冻结 production Pelvis”的动态 receiver / authority-isolation sandbox，再谈 production Pelvis dynamic 化。

## 0.8 v0.3.4–v0.3.15 Dynamic Pelvis / Two-Body Mass Foundation 结论与 v0.3.16 当前入口

- v0.3.4 建立 isolated PhysX Pelvis A/B sandbox，但 copied production kinematic tensor 为 `(1,1,1)`，因此当时约 `0.54°` reaction penalty 只能保留为 default-inertia witness。
- v0.3.5 实机确认 analytical Pelvis inertia≈`(0.027136,0.053456,0.042573)`，copied/analytical ratio≈`36.85/18.71/23.49`；source-only L0/A tracking几乎一致，说明约13° tail不是copied inertia导致。
- v0.3.6 已关闭：Normal-Transition angular-acceleration FF因果收益SUPPORTED，但exact derivative尚未production-ready；同时发现production Pelvis/Chest default-like COM/tensor。
- v0.3.7 full explicit mass-property migration在冻结Two-Body controller下REJECTED；v0.3.8 factorization确认**explicit inertia是primary regression owner，COM-only≈production**。
- v0.3.9 corrected-inertia F继续稳定复现rotation regression：Normal P99约`6.269→17.622°`，Steady约`2.454→3.833°`，Transition约`8.589→19.233°`；best target-W→body-W lag仍为0，因此不是简单额外固定帧延迟。Two-Body角向执行仍是`AddTorque(..., ForceMode.Acceleration)`，问题继续指向CharacterJoint/effective constraint response、COM lever与linear parent-FF的闭环耦合。
- v0.3.9同run曾出现Right Knee `NORMAL_TRANSITION`三拍debt avalanche：约`3mm→20.85→29.60→36.20mm`并触发30mm hard gate；但v0.3.10多轮自然复测表明，严重frame stall不是该Knee abort的充分条件。
- v0.3.10至少获得两轮强stall但无abort的负对照：一轮约`169ms render / 234ms wall-gap / 10 Fixed catch-up / 159ms source age / held 9`，另一轮约`232ms / 389ms / 14 / 219ms / held 13`，Knee仍安全通过。最新完整run约`7240/7240 valid`，Left/Right peak Knee anchor约`21.851/21.212mm`，bilateral freeze PASS。
- v0.3.10 correlation重复显示：Knee positive-debt对renderDt只有弱相关（约0.21），对wall-gap/source-age/fixed-index接近0；whole-proxy detachment对render/source-age/fixed-index关联明显更强（最新约0.35/0.47/0.32）。因此**frame stall可解释六代理整体瞬时脱离，但不是Knee 30mm fail的主要直接owner**。

**v0.3.11 已关闭：**

- 实机dataset=`5232/5232 valid`，Normal=`2493`（Steady/Transition=`2189/304`），F drive-loss=0，dataset可评分。
- Normal rotation P99 P/F≈`7.482/17.201°`；corrected-inertia regression再次稳定复现。
- CharacterJoint solver angular-A proxy P99 P/F≈`6.972/22.575 rad/s²`，在corrected plant中显著放大。
- `r(COM→joint anchor)×currentForce` 在P/F有效reaction样本中均100%大于native `currentTorque`，因此direct native torque不是primary owner，force-moment coupling成为主要机械候选。
- 对`measuredA - previousCommand`加入统一sign solver witness后，P99 residual closure improvement约P=`11.0%`、F=`35.0%`；P/F均选择`MEASURED_MINUS_COMMAND_MINUS_SOLVER`。说明solver reaction是material contributor，但仍不足以解释全部corrected residual。
- Knee hard safety PASS：peak约Left/Right=`21.762/23.414mm`，abort=False；bilateral clock mismatch全0。Full freeze未评估仅因为本轮27个Source event均为rapid-continuous、TRUE discontinuity=0。

**v0.3.12 已关闭 / STRONG PASS：**

- `5248/5248 valid`；Normal rotation P99 P/F0/F1≈`6.365/17.693/6.880°`。
- gain=1 previous-settled `-r×Joint.currentForce` cancellation使F1/F0≈`0.3889`，追回约95.5%的F0相对production excess tail；cancellation P99≈`4.488Nm`、guard≈`10.146Nm`、clamp=0。
- solver-A反而从F0≈`21.559`升至F1≈`38.222 rad/s²`，说明cancellation改变的是constraint closed-loop balance，不是让CharacterJoint reaction消失。
- 结论：`r×currentForce`为STRONGLY SUPPORTED physical causal contributor；但previous-step cancellation只作为causal probe，不授权production。

**v0.3.13 已关闭：**

- 最终clean dataset=`5195/5195 valid`，Normal=`2533`（Steady/Transition=`2203/330`），F0/F1 drive-loss=`0/0`。
- Normal rotation P99 P/F0/F1≈`7.142/15.064/31.842°`，F1/F0≈`2.114`；Steady F0/F1≈`4.554/5.708°`，Transition≈`19.195/42.628°`。
- Parent FF OFF后force moment P99从约`2.409`升至`3.261`，solver-A从约`28.654`升至`36.908 rad/s²`，Chest position P99从约`68.664`升至`94.511mm`。
- 同一日志安全门前独立segment=`3656/3656 valid`也给出F1/F0 rotation ratio≈`2.082`，方向重复一致。
- 结论：**ParentMotionFeedforward不是有害`r×currentForce`的上游source；相反它是corrected-inertia CharacterJoint topology里的稳定/load-sharing transport path。Parent FF保持ON。**
- 同run曾发生一次Left Knee轻微30mm越界（约`30.692mm`，SOURCE_TURN，fresh source，非stall）；这进一步支持Knee hard-gate事件属于低频state-dependent debt，而不是单一电脑卡顿owner。当前hard fail-close不变。

**v0.3.14–v0.3.15 已关闭 / v0.3.16 当前入口：**

- P继续是真正production legacy torso，完全不修改。
- F0/F1都使用相同corrected explicit COM+uniform-box inertia、Parent FF ON、relative-PD、Pose Bridge、source sampling、kinematic Pelvis/dynamic Chest。
- F0 native CharacterJoint继续使用calibrated source/spine anchor。
- F1唯一机械变化：把**Chest侧Unity CharacterJoint native anchor移到Chest centerOfMass**，并重算connectedAnchor使两侧native anchor在build时位于同一世界点；因此Chest COM→native joint anchor lever应接近0。
- 重要隔离：旧`_virtualChestAnchorLocal/_virtualPelvisAnchorLocal`继续保留给source/control/Parent-FF transport语义，F1不改这些reference anchor。
- 同时记录source/control anchor error与native constraint-anchor error、Chest COM lever、joint force/native torque/`r×force`/solver-A，以及rotation/position/drive-loss。
- 若F1在lever≈0且native anchor closure健康时显著降低forceMoment和rotation，则lever-arm channel获得更强结构性causal支持；若rotation不改善，则v0.3.12 cancellation的收益不能简单归因于几何lever本身。
- 当前30mm Knee仍是硬fail-close；future soft recovery gate继续只保留为full-body后期resilience roadmap。
- v0.3.14实机`5409/5409` samples有效；Normal rotation P99 P/F0/F1≈`8.114/17.042/8.028°`，F1/F0≈`0.471`；Chest COM lever P99约`83.093mm→0`，`r×currentForce` P99约`1.729Nm→0`。因此Chest侧lever-arm causal owner已 **CLOSED / STRONG PASS**。
- v0.3.15不改变上述P/F0/F1物理结构，只读记录同一previous-settled reaction快照中的Chest/Pelvis lever、两侧body-local moment与corrected Pelvis inertia-only angular-A；production Pelvis保持kinematic。
- v0.3.15 clean run=`5108` samples，Normal=`2575`，invalid=0，fault=False，corrected parity=True，productionPelvisKinematic=True，scorable=True。F0/F1 joint-force P99约`65.469/67.885N`，只增加约3.69%；Chest lever约`82.790/0.000119mm`、Chest moment约`2.049806/≈0Nm`，但Pelvis lever约`177.969/247.069mm`、parent moment约`7.796/12.144Nm`（F1/F0≈`1.558`），parent inertia-only A约`270.275/438.273rad/s²`（F1/F0≈`1.622`）。
- **v0.3.15 CLOSED / STRONG LEVER-TRANSFER EVIDENCE。** Chest-COM native anchor是当前kinematic-parent topology的强workaround，但会把更大的body-local rotational burden转移到connected Pelvis侧；不能把它直接称为full-body topology solution。
- v0.3.16删除旧P/F0/F1 live replicas与parent witness的运行时接线，进入两个完全隔离的dynamic Pelvis/dynamic Chest rotational receiver A/B。两lane都使用显式box COM/inertia；Pelvis authority恢复v0.3.5/v0.3.6的inertia+gyro P+D内核，不恢复full Hip reaction injection，也不启用target-A feedforward。

---

---

# 1. 当前整体设计

## 1.1 层级结构

当前系统可理解为：

`Source Animation / Render Sampling`
→ `Render→Fixed Pose/W Bridge`
→ `Two-Body Pelvis/Chest Foundation`
→ `Bilateral Coordinator`
→ `Left Frozen Single-Leg Controller + Right Frozen Single-Leg Controller`
→ `v0.3.0 Bilateral Parity + v0.3.16 Isolated Fully-Dynamic Pelvis/Chest Anchor A/B`

当前 production 身体仍不是完整 Newtonian closed-body：腿部 actuator/tether 的 equal-and-opposite parent reaction 尚未回灌 production shared Pelvis。Two-Body torso 仍由 **Unity 原生 CharacterJoint + ARL自定义relative-PD/parent-FF/Pose-Bridge** 构成；Single-Leg build阶段仍会短暂创建Unity CharacterJoint用于既有初始化/隔离路径，但执行 `arl_hip_stack(on)` 后的冻结production Hip/Knee topology为NO_HIP_CHARACTERJOINT / NO_KNEE_CHARACTERJOINT。v0.3.7 已证明full explicit COM+inertia不能直接迁移到现有冻结Two-Body controller；v0.3.8进一步确认inertia是primary owner、COM-only基本无害。v0.3.11–v0.3.14逐步把corrected-plant回归定位到CharacterJoint线性约束力经Chest COM lever形成的`r×currentForce`；v0.3.15又确认Chest-COM alignment会把更大的rotational burden转移到kinematic connected Pelvis侧。v0.3.16只在与production完全断开的双动态接收器中检验这一几何变化，production Two-Body和Single-Leg source/physics write均冻结。

## 1.2 Two-Body Foundation 的职责

- 提供 Pelvis/Chest 代理、父体 pose/velocity bridge 与 source-turn/recovery语义。Pelvis↔Chest 约束仍是 **UnityEngine.CharacterJoint**，不是ARL自定义joint；ARL自定义的是围绕它运行的relative-PD torque、parent feedforward、Pose-Bridge和freeze/recovery。
- 负责 Normal/Source/Recovery freeze validation 与 fail-fast。
- Pelvis 当前仍由 Pose-Bridge 主导且 production body 为 kinematic。future dynamic receiver 必须显式设置并验证 mass、COM、inertiaTensor 与 inertiaTensorRotation，不能再复制/依赖 kinematic Rigidbody 的默认式 mass-property readback。v0.3.6 的只读 audit 已确认 production Pelvis/Chest 都不匹配 uniform-box foundation；v0.3.7 仍不直接修改 production，而是先在完整 Two-Body shadow topology 上做 causal requalification。

## 1.3 Single-Leg Frozen Controller 的职责

Hip：

- no CharacterJoint；upper-only translation tether维护Hip anchor几何。
- relative PD + source/recovery reference stack + strict/mechanical-parent transport。
- v0.2.81 inertia/gyro actuator负责实际 angular torque。
- 现有 tether 不给 parent equal/opposite force，因此不是完整动量守恒约束。

Knee：

- no CharacterJoint；Lower-COM translation tether维护Knee anchor位置。
- flexion-axis PD + soft off-axis PD。
- Knee torque child-only；没有 equal/opposite Upper torque。
- Knee parent angular transport与anchor acceleration FF为已验证production组件。
- 30mm translation-tether validity 是硬 safety contract。

## 1.4 Bilateral Coordinator 的职责

- 不复制 RightLeg 控制器；只实例化同一个 `SingleLegShadowChain` 两次。
- 两侧PCA、stack、trace、drive采用对称preflight。
- 任何一侧 fail-fast都传播为 bilateral fail-close。
- v0.3.0 parity observer永远只负责生命周期/冻结基础，不被后续reaction observer替代。

## 1.5 v0.3.4 Dynamic Reaction Receiver 的职责

v0.3.4 不把 production Two-Body Pelvis 从 kinematic 改成 dynamic。原因是那会同时改变 Chest CharacterJoint connected body、两条腿共同 mechanical-parent reference 与 Pose-Bridge authority，无法隔离 reaction 自身因果。

因此新建两个**完全断开 production chain 的 collider-less dynamic Pelvis shadow receiver**：

- A：`SOURCE_AUTHORITY_ONLY`。
- B：`SOURCE_AUTHORITY + -(τHipPrimary + τHipSecondary)`。
- 两个 receiver 都复制 production Pelvis 的 mass、inertiaTensor、inertiaTensorRotation、angularDrag 与 maxAngularVelocity。
- position 三轴冻结，只开放 angular DOF；没有 Collider、Joint、Gravity，也不作为 Chest/Leg parent。
- source authority 复用冻结 Two-Body 的 `AngularSpring=55 / AngularDamper=12 / MaxAngularAcceleration=40`，仅作为 diagnostic receiver law，不新增 production tuning knobs。
- source target rotation 使用冻结 Pelvis 已经计算好的 Pose-Bridge command；target angular velocity 使用同一 FixedStep 的 command transport angular velocity。
- source authority acceleration转换为真实 inertia+gyro physical torque后，通过 `AddTorque(..., ForceMode.Force)`施加到A/B；B再额外施加exact bilateral Hip reaction torque。
- production Pelvis仍`isKinematic=True`，production Two-Body/Single-Leg方法不消费任何 receiver state。

主要观察：

- A/B source tracking rotation error。
- `B error - A error` 的 reaction penalty。
- A/B相对rotation与angular-velocity divergence。
- exact Hip reaction torque 与 B source-authority torque 的倍率、方向alignment/opposition。
- A/B source-authority clamp exposure，尤其active Normal。
- receiver自身non-finite/reset必须与production fail-fast分离；shadow sandbox失败不得关闭生产身体，但会让该receiver dataset不可评分。

这里的目的仍不是自动授权production reaction，而是第一次得到**同一个Unity PhysX时钟、相同source authority、唯一差异为Hip reaction torque**的真实物理causal A/B。

## 1.6 v0.3.6 Transition Feedforward / Physics-Foundation Audit 的职责（已关闭）

v0.3.6 的 A0/A1 只研究一个问题：`NORMAL_TRANSITION` 是否缺少 moving-target angular-acceleration feedforward。首轮实机已经给出足够强的因果结果：

- post-restart clean run 中 Normal P99：A0≈12.249°，A1≈8.616°。
- Transition Fresh P99：A0≈13.233°，A1≈8.295°；Transition Held：13.230°→8.794°。
- Steady Fresh 基本不变：5.455°→5.517°。
- targetA P99/max≈42.866/93.677 rad/s²；A1 Transition clamp=19，说明 exact derivative 有明显收益，但未经conditioning还不能直接进入production。
- 因此“Transition需要angular-acceleration FF”的 causal hypothesis 关闭为 SUPPORTED；v0.3.6 live observer 已退休。

同一轮 `PHYSICS_FOUNDATION_AUDIT` 则发现更上游的问题：

- production Pelvis：COM actual `(0,0,0)` vs expected `(0,0.025981,0)`；I actual `(1,1,1)` vs expected约`(0.027141,0.053466,0.042581)`。
- production Chest：COM actual `(0,0,0)` vs expected `(0,-0.051962,0)`；I actual `(1,1,1)` vs expected约`(0.081844,0.11108,0.126774)`。
- Pelvis当前kinematic；Chest在drive ON后dynamic，因此Chest是现役production mass-property foundation风险。

## 1.7 v0.3.8 Two-Body COM / Inertia Factorization 的职责（已关闭）

v0.3.8 用 P/C/I/F 同run 2×2 factorization正式关闭了v0.3.7的owner问题：

- P production legacy：Normal/Steady/Transition P99≈`8.589/2.650/9.545°`。
- C COM-only：≈`8.365/2.617/9.515°`，与P几乎一致。
- I inertia-only：≈`11.917/6.602/20.677°`，Steady与Transition都显著恶化。
- F full explicit：≈`9.791/4.147/19.009°`，仍明显差于P，但比I-only更好。
- Normal angular clamp P/C/I/F=`0/0/2/2`；C/I/F drive-loss=`0/0/0`。

**关闭结论：** explicit inertia是主要regression owner；COM不是primary owner，并且在corrected-inertia plant上有部分mitigation。不能把问题写成“CharacterJoint anchor坏了”，也不能用MaxA=40不足解释全部退化。

## 1.8 v0.3.9 Corrected-Inertia Two-Body Controller Response Attribution 的职责（已关闭）

v0.3.9使用P production legacy + F explicit COM/inertia单replica，确认corrected-inertia regression稳定存在，并且不是简单target-W→body-W多一拍delay。Two-Body angular actuator仍是`ForceMode.Acceleration`，所以不能把问题粗略归因为“torque没有乘inertia”。该版提供P/D/raw/applied command与lag1 measured response基础，随后因Knee稀有安全事件暂时让路给v0.3.10。

## 1.9 v0.3.10 Frame-Stall / Knee Debt Attribution 的职责（已关闭）

v0.3.10为了验证用户观察到的“偶发卡顿时六个proxy一起脱离”是否同时拥有Knee 30mm abort，临时删除额外torso replica，只运行六个production proxy并记录renderDt、wall-gap、Fixed catch-up、source age/held、six-body center error与Knee debt。

多轮自然实机结果形成强负对照：200–400ms级stall、十余拍Fixed catch-up和>200ms stale source可以发生，但Knee仍完整恢复；Knee debt与timing相关弱，而whole-proxy detachment与source age/render/fixed-index关联更明显。

**关闭结论：** frame stall是视觉whole-proxy temporary detachment的可信触发因素，但不是Knee hard abort的充分/主要直接owner；历史abort归类为rare state-dependent multi-factor debt avalanche。v0.3.10 live observer退休。

## 1.10 v0.3.11 Corrected-Inertia CharacterJoint Solver Coupling Attribution 的历史职责（已关闭）

该版本当时只保留P/F两条torso plant：

- **P**：真正production legacy Two-Body；COM/inertia、PD、Parent FF、CharacterJoint全部冻结不改。
- **F**：一个与production断开的collider-less corrected plant；explicit `Box.center + uniform-box inertia + identity inertiaTensorRotation`，其余topology/control与P相同。

v0.3.11新增的只读tap：

- angular P、D、raw/applied command与headroom；控制公式不变。
- FixedUpdate入口的`CharacterJoint.currentForce/currentTorque`，对应刚刚settle的上一物理拍。
- Chest COM→joint anchor lever、`r×currentForce`、native torque、combined moment。
- 使用每个lane自己的`Rigidbody.inertiaTensor/inertiaTensorRotation`把solver moment换算为inertia-only angular-A proxy；该值不含gyro，属于witness而不是physics truth。
- 上一拍angular command→本拍measured relative-A residual，分别测试`residual-solver`与`residual+solver`，不预设Unity joint reaction API的符号方向。
- solver-A/command ratio、solver与command alignment/opposition、force-moment/native-torque ratio与dominance share。
- joint force与`mass*parentFF acceleration`的绝对alignment/ratio，用来判断linear parent-FF是否通过CharacterJoint lever形成主要angular coupling；API reaction sign不预设。
- Steady/Transition分别统计closure improvement，避免Transition tail掩盖Steady plant mismatch。

**决策边界：**

- 若F solver-A显著高于P，且加入某一统一solver sign后`measuredA-command` residual明显下降，则CharacterJoint/effective-inertia coupling获得直接支持。
- 若`r×F`长期主导native torque，则下一步优先拆COM lever / linear parent-FF / anchor constraint coupling，而不是先调rotational PD。
- 若native torque主导且solver closure显著，则再研究torso joint topology/effective inertia；仍不直接移除production CharacterJoint。
- 若solver witness不能解释residual，则回到reference/parent-transport/parent-FF时序，不用PD tuning掩盖owner未知。

## 1.11 Knee hard-safety context 的职责

v0.3.10 live timing observer退休后，只保留一个12拍固定ring：

- 每拍记录Left/Right Knee anchor、raw/applied tether acceleration、parent-anchor FF、clamp、phase/fixed sequence。
- 只有trace finalize/status/abort时构建字符串；FixedUpdate无动态分配、无额外Rigidbody、无控制输出。
- 30mm仍是当前冻结硬安全契约；未来软恢复安全门不属于当前版本。

---

---

# 2. 未来宏观规划

## 2.1 当前阶段 D：Bilateral → Full-Body

顺序原则：

1. 双腿 frozen replication（v0.3.0）——完成。
2. shared-parent/common-differential attribution（v0.3.1）——完成/退休。
3. missing reaction path feasibility（v0.3.2）——完成，full raw tether reaction未授权。
4. selective reaction budget + Knee tail context（v0.3.3）——完成/退休；Hip angular-only进入physical-study候选。
5. **corrected-inertia source-authority closure（v0.3.5）——已关闭：Transition owns tail；full local-Pelvis Hip reaction rejected。**
6. **Normal-Transition target-angular-acceleration FF（v0.3.6）——causal hypothesis SUPPORTED；live observer retired。**
7. **Two-Body full explicit COM/inertia causal requalification（v0.3.7）——完成/REJECTED。** Full explicit把Normal P99约`7.93→18.83°`，但anchor closure仍健康；不能直接production迁移。
8. **Two-Body COM vs inertia factorization（v0.3.8）——完成。** Inertia=primary owner；COM-only≈legacy，Full较I-only有部分mitigation。
9. **Corrected-inertia controller response attribution（v0.3.9）——完成。** regression稳定复现，非简单固定帧delay。
10. **Frame-stall / Knee debt attribution（v0.3.10）——完成。** stall支持whole-proxy detachment，但不是Knee abort的充分/主要直接owner。
11. **CharacterJoint solver coupling attribution（v0.3.11）——已关闭。** 用currentForce/currentTorque、COM lever moment和command→measured-A closure定位corrected-inertia plant mismatch。
12. 根据v0.3.11 owner再做最窄controller/constraint requalification；之后才返回bounded/conditioned Transition FF。
13. 然后重新设计reaction allocation/effective-inertia receiver；full tether reaction继续不授权。
14. 再加入Foot/Ankle代理（初期Gravity/Colliders OFF），随后进入Contact / stance-swing / ground reaction / balance，最后扩展torso/arms/full-body。


## 2.2 关于未来 Foot Proxy 与当前小腿误差

目前肉眼观察认为误差更集中在小腿，这个现象在现有结构下是**合理但尚未单独因果证明**的：Lower Leg 现在上端由Knee translation tether约束，角向由child-only Knee torque控制，但下端Foot没有独立Rigidbody/ankle constraint/contact boundary，因此Lower是一个“只有proximal约束、distal端自由”的物理段。

加入双脚代理预计会有三类帮助：

- **Distal endpoint geometry**：Foot/Ankle提供小腿下端的位置/姿态边界，能减少Lower围绕Knee anchor的自由漂移与几何欠约束。
- **更好的状态可观测性**：Knee误差不再只靠Upper↔Lower anchor和flexion angle判断；可以同时看ankle/foot pose debt，区分“膝盖本身没跟上”与“整根小腿作为刚体偏了”。
- **未来ground support路径**：进入contact后，脚是ground reaction、friction、stance/swing、balance的真正入口；没有foot proxy就无法建立完整下肢支撑链。

但Foot Proxy **不能简单做成另一个硬position tether**。必须避免：

- Knee 30mm硬安全tether + ankle硬tether + kinematic foot lock同时抢同一条Lower刚体，形成过约束。
- source动画与ground contact不一致时把所有冲突都塞进Knee，造成更大的anchor spike。
- Foot在swing phase也被地面/位置硬锁，破坏步态。
- 在reaction path尚未预算完成前直接把足底冲击全传回dynamic Pelvis。

更合理的未来方案：

- Foot/Ankle先作为**独立刚体 + 有限compliance的ankle angular/translation contract**加入，初期仍Gravity/Colliders OFF做链几何验证。
- 先观察Hip/Knee/Ankle三段误差如何重分配，再决定是否减弱现有Knee tether或改成更物理的constraint。
- Contact阶段采用stance/swing gating；stance允许有限足底支撑，swing不锁地。
- ground reaction进入Pelvis前继续使用reaction budget / authority allocation，避免一步跳到全闭环。

因此：**双脚代理大概率对小腿约束有帮助，但真正价值是给Lower Leg增加正确的distal boundary，而不是用“更硬”直接压误差。**

## 2.3 后续阶段 E/F

- Stage E：Gravity / Colliders / Contact / GSP融合；必须先有清楚的reaction path与foot support语义。
- Stage F：ML-Agents / 高层语义动作。RL不应替代底层物理契约；先让身体具有稳定、可观测、可失败归因的低层动力学，再让策略网络学习高层协调。

## 2.4 未来 Soft Safety Recovery Gate（远期，不在当前主线）

当full-body、Foot/Contact、reaction allocation与基础恢复路径都跑通后，可以把当前“一旦越硬门立即退出”的部分安全策略升级为分级恢复，但前提是先证明恢复本身不会把错误继续放大。目标不是取消硬安全，而是在**有限且状态仍finite的可恢复越界**时增加一个短时间recovery envelope：

- 第一级：检测到可恢复的anchor/pose excursion后，不立即销毁整个代理；进入受限recovery状态。
- recovery期间可以暂时降低危险自由度/限制新增debt，并优先偿还position/rotation debt；具体authority、source hold/conditioning、contact策略届时重新做因果验证，当前不预先冻结实现。
- 若在严格短窗口内回到安全带，则重新进入normal controller，并保留完整事件记录。
- 若出现non-finite、持续runaway、重复越界、恢复超时或其它不可恢复条件，仍升级为当前hard fail-close。
- 软门不能通过简单把30mm阈值调大实现；必须有明确的entry/exit/timeout/hysteresis与“恢复失败→硬退出”语义。

当前v0.3.16继续保持30mm Knee hard gate不变。这个方向只作为**后期鲁棒性/产品化安全层**记录，避免现在把稀有tail-risk与主物理闭环开发混在一起。

---

# 3. 历史版本记录

## 3.1 v0.1.x 系列总结——Two-Body、时序桥与安全语义基础

### 为什么需要0.1.x

ARL最初的问题不是“PD参数是多少”，而是Unity IL2CPP环境下能否稳定创建代理、能否找到可用Joint/物理API、Render动画与FixedUpdate物理是否同相位，以及source瞬时转向时日志中的大误差到底属于controller failure还是sampling discontinuity。0.1.x的任务是把这些基础语义建立起来，否则后面任何Kp/Kd、joint、feedforward实验都会被时序和平台问题污染。

### 0.1.0–0.1.3：平台与最小动态闭环

- 建立Active Ragdoll Lab最小插件、目标骨审计、代理Rigidbody构建与报告。
- IL2CPP中泛型组件创建能力受限，形成非泛型 component factory / staged build diagnostics；以后遇到AddComponent/interop异常，先判定平台能力，不把它误当控制问题。
- ConfigurableJoint路线确认不值得继续后，先用jointless PD验证最小“target pose → rigidbody torque → physical response”链。
- 对移动source增加速度前馈并去掉不必要的Rigidbody interpolation拖尾，确认render动画与physics target必须显式桥接。

### 0.1.4–0.1.8：CharacterJoint、Relative PD与Parent Motion

- 尝试CharacterJoint + PD Torque用于Pelvis/Chest；宽限位用来区分“控制问题”与“joint limit提前接管”。
- 建立relative joint-space PD，而不是用world pose直接追source；parent motion feedforward开始成为正式设计要素。
- 把steady与transition分开统计，并检查PhysX solver后的真实运动，避免只看施加command。
- TURN过程中发现parent transport与FixedUpdate phase alignment很关键；形成“父体运动必须纳入child target语义”的长期原则。

### 0.1.9–0.1.11：Render→Physics Bridge

- Render sample-and-hold angular velocity bridge解决“FixedUpdate内看不到动画真实速度”的问题。
- Pose Bridge进一步把render pose通过predict/correct方式运输到physics clock。
- Parent feedforward必须与Pose Bridge使用同一个command reference；不能一个模块看raw source，另一个看bridged pose，否则会制造内部相位矛盾。

### 0.1.12–0.1.16：Source Turn、Freeze Validation与Fail-Fast

- 引入Source Turn Discontinuity attribution，把rapid continuous turn与真正kinematic discontinuity分开。
- Freeze validation不只看平均误差，而是要求Normal、Source survival、Recovery三个阶段都满足既有契约。
- Recovery必须从source event close后评分；新的source event会censor旧recovery，不能把未完成窗口算成failure。
- compact trace、failure attribution与drive fail-fast成为长期安全框架。
- Stable-hold与异常捕获被加入，避免“数值坏了但controller继续跑”的silent corruption。

### 0.1.x 最终留下什么

- Two-Body foundation。
- Render→Fixed sample/hold velocity + Pose Bridge。
- Pose-Bridge-consistent parent feedforward。
- SourceTurn semantic + recovery/freeze policy。
- fail-fast / report-finalization / compact trace模式。

### 以后必须避开什么

- 不再回到“只看FixedUpdate瞬时source transform”判断动画速度。
- 不用单一平均误差评价source-turn robustness。
- 不把新source event导致的censored recovery当failure。
- 不因为IL2CPP组件/API缺失就用高层控制参数补偿平台问题。
- 不让observer、日志开关包住production lifecycle/state estimator。

---

## 3.2 v0.2.x 系列总结——Single-Leg 从拓扑、惯量、运输、恢复到Precision Freeze

### 为什么需要0.2.x

Two-Body已经能稳定解释source/physics时序，但全身动作最终依赖腿。Single-Leg的难点集中在：Knee hinge轴并不适合手写、真实inertia改变PD含义、CharacterJoint solver reaction与ARL自有控制混在一起、Hip parent运动会通过多条路径进入Lower、极端source turn会让reference比物理能力走得更快、Normal阶段Knee还有group-delay与velocity-reference问题。因此0.2.x采用“每次只改一个机械变量/参考变量”的长因果链，最终收敛到v0.2.122 frozen baseline。

### A. v0.2.0–v0.2.17：Single-Leg基础、Knee PCA、A-FF与真实惯量

**核心工作**

- 建立Upper/Lower/Foot目标解析、Single-Leg proxy与初始Hip/Knee chain。
- v0.2.2用source-only PCA学习Knee flexion axis；不再依赖左右腿手写符号。后续双腿实机证明左右candidate自然形成镜像分量，说明PCA吸收了局部坐标镜像。
- 逐层加入Hip/Knee parent transport、phase timing、relative pose/W bridge。
- Angular Acceleration FF在Normal窗口被验证有价值，最终保留gain1/bound20。
- v0.2.10–0.2.16围绕“internal reaction / inertia weighting / COM / upper-vs-lower inertia”做系统隔离。
- v0.2.12–0.2.17最终建立 Explicit Uniform Box mass/COM/inertia foundation，停止依赖Unity默认unit inertia语义。

**为什么这样做**

真实惯量改变同样torque对应的angular acceleration。如果不先冻结mass property，后续“PD变差/变好”无法区分是controller还是刚体参数变化。

**必须避开**

- 不用unit inertia实验结论直接推导real-inertia production。
- Knee轴不为Right side手工加负号；优先data-driven axis。
- A-FF只能在明确Normal healthy窗口使用；Recovery/Source期间不能偷偷继续注入。

### B. v0.2.18–v0.2.29：Constraint因果、Soft Knee与Recovery Authority

**核心工作**

- Hip principal-axis、frame/coupling、single-axis solver response与Knee lower-chain causality逐步拆开。
- 检查Knee constraint backend、CharacterJoint limit engagement，确认一部分误差来自solver/constraint而非PD本身。
- 建立Soft Knee off-axis control，最终production为30/8、MaxA 20/40/40。
- v0.2.25开始Production Knee soft-constraint migration；随后通过Recovery attribution与phase-aware authority验证source/recovery窗口。
- Whole-leg/Hip recovery开始与source-turn语义绑定，Hip authority形成baseline/source/recovery分层。

**为什么这样做**

单纯提高Knee Kp会同时影响flexion与off-axis，且CharacterJoint native reaction会掩盖真正的控制效果。必须先分轴、分phase、分solver ownership。

**必须避开**

- 不把joint limit接管误写成PD成功。
- 不用Recovery参数改善Normal误差；phase authority必须分开。
- 不因source-turn peak大就全局提高MaxA。

### C. v0.2.30–v0.2.44：Hip Control Frame、Reference Governor与Feasibility

**核心工作**

- Source turn下Hip出现reference跑得比physical body快的问题，于是建立control-frame inertialization、bounded lag、whole-leg transport attribution与body-aware Governor。
- 经stacked-source、tracking-health、Normal A-FF等多轮隔离后，确认问题不是一个简单“多一点前馈”就能解决。
- strict W/A control-frame integrator、sample-hold/composite clock、local Render→Fixed bridge被逐层验证。
- v0.2.44形成Physical-Recovery-Continuous Feasibility-Limited Hip Reference Governor：reference必须服从当前body stopping/authority，而不是无限追source。

**为什么这样做**

极端source turn会产生一个逻辑上可定义、但物理上不可执行的target trajectory。控制器若盲目追这个target，只会制造持续lag/debt。

**必须避开**

- 不把“target math更接近source”当作“物理更可执行”。
- 不关闭acceleration bound追求视觉瞬时跟随。
- 不把reference governor与actual actuator authority割裂分析。

### D. v0.2.45–v0.2.61：Mechanical Parent、Anchor Transport与Constraint Reaction

**核心工作**

- 物理Hip parent reaction / braking attribution表明parent frame本身是关键变量。
- 引入inertialized mechanical-parent proxy，并逐步验证strict angular contract、parent angular-acceleration transport、anchor linear transport。
- v0.2.49强确认Hip anchor linear acceleration / joint reaction的重要性；v0.2.50初版transport执行不完整，v0.2.51修复RAW→STRICT activation continuity后才真正评分。
- 继续检查CharacterJoint preprocessing、native solver reaction、constraint force/application point，逐步降低“某个native joint flag就是主因”的可能性。
- v0.2.61确认recovery bandwidth与source stacking相关且state-dependent。

**为什么这样做**

Hip并不是孤立转子；parent的角/线运动、anchor几何、solver reaction会同时影响Upper。只有把这些transport路径逐一显式化，才能知道PD真正控制了什么。

**必须避开**

- “observer看到parentA”不等于production真的把它transport进去了；必须检查执行路径。
- 不把native solver preprocessing的一次差异当根因。
- 机械parent与真实Pelvis的语义必须区分；proxy不是等同于全身动量闭环。

### E. v0.2.62–v0.2.79：Recovery Phase-Space、去Joint Serial Chain与Knee Transport

**核心工作**

- Recovery control-effort、origin、signed phase-plane、3D radial/tangential、vector capture等observer/A-B用来研究“误差大时应该刹车还是追target”。
- 多轮结果说明单纯momentum-aware braking只能部分改善，不能作为普适production替代。
- Source-reference shock absorber、final composite world-reference absorber逐步吸收不可执行reference jump。
- v0.2.73/0.2.74移除Hip CharacterJoint，保留translation-only tether；最终采用**upper-only Hip tether**，让Lower重新通过Upper→Knee形成serial chain，而不是Hip tether同时直接拖Upper+Lower。
- v0.2.75加入Knee parent angular transport；v0.2.76移除Knee CharacterJoint并用Lower-COM translation tether保持Knee anchor。
- v0.2.77 Knee parent-anchor acceleration FF PASS；v0.2.78 feedback-priority allocation PASS；v0.2.79 Hip positional anchor acceleration FF PASS。

**为什么这样做**

Native joint reaction与ARL torque混在一起时，很难判断Lower误差到底来自solver还是controller。去Joint后虽然失去自然equal/opposite reaction，但换来了更可解释的serial-chain控制基础，为后续reaction-path研究留下清晰边界。

**必须避开**

- translation tether只解决位置，不代表完整机械joint。
- upper-only/child-only force或torque不守恒，不能提前声称“全身闭环”。
- 30mm tether safety是因果隔离阶段的重要保护，不应因为偶发tail event直接放宽。

### F. v0.2.80–v0.2.89：Settled Catch-Up、Inertia/Gyro Actuator与Healthy-Normal Parent Transport

**核心工作**

- v0.2.80观察source target已经settled但physical Hip仍落后的catch-up窗口。
- v0.2.81将Hip actuator改为inertia/gyro-aware torque；实现层PASS，但“它会自动消除所有tracking问题”的主假设REJECT，因此只保留作为更物理的actuator实现。
- Upper angularDrag隔离后production仍保留drag=3。
- Parent-relative FixedStep closure、Healthy-Normal upcoming transport、timing/authority/scope observer用来判断Normal parent transport是否应一直开。
- v0.2.87确认post-frame physical recovery确实需要dedicated parent support。
- v0.2.88最终冻结：**True Healthy Normal transport OFF，Dedicated Recovery support ON**。

**为什么这样做**

Normal与Recovery对parent transport的需求不同。把Recovery中有价值的support长期施加到Healthy Normal，会制造持续平均误差。

**必须避开**

- 不把“Recovery需要”推成“Normal也必须开”。
- 不因为inertia/gyro actuator更物理就继续无限调它；tracking残差还有reference/transport/phase来源。

### G. v0.2.90–v0.2.101：Non-Healthy / TRUE Shock / Governor 闭合

**核心工作与关键结论**

- v0.2.90定位Non-Healthy opposed response；v0.2.91 strict-phase parent-step CF显示upper residual以drag-proxy dominant为主。
- v0.2.92建立source discontinuity severity语义；v0.2.93区分packetization与TRUE kinematic discontinuity。
- v0.2.94–0.2.97追踪TRUE shock temporal debt、world-target counterfactual、impulse vs backlog、severe vs contained。
- v0.2.98首次暴露severity join bug；0.2.98.1只做分类hotfix。
- v0.2.99：severe pure TRUE晚期Governor主要受lead cap限制，但release-eligible=0。
- v0.2.100证明`ACCEL_CLAMP_MASKS_LEAD_RELAXATION`是真实interaction。
- v0.2.100.1发现observer/timing probe本身会造成持续tracking退化，成为重要维护教训。
- v0.2.101 required-kinematics：NO_LEAD+NO_ACCEL轨迹平均约需5.2×、peak约6.9×现有full Hip acceleration authority；stopping可行但加速不可行。

**最终决定**

- Severe TRUE Governor研究 CLOSED。
- 不提高Source/Recovery MaxA追不可执行target；不关acceleration clamp；不做Governor-relaxation physical A/B。
- v0.2.100 interaction是真实数学现象，但不代表物理应该照着counterfactual轨迹执行。

**必须避开**

- observer overhead可以成为真实物理干扰；只读不等于“零影响”。
- classification/join bug会让后续统计全错，语义层必须先于数值结论。
- CF要区分“数学更接近source”和“所需kinematics在当前刚体authority内可执行”。

### H. v0.2.102–v0.2.122：Normal/Mild Precision、Knee Phase/Filter路线与Single-Leg Freeze

**进入原因**

Severe TRUE已关闭后，继续围绕极端事件调Governor价值很低，转而处理用户实际长期看到的Normal/Mild持续误差，尤其Knee。

**主要因果链**

- v0.2.102清理heavy severe observers，进入Normal/Mild precision；v0.2.102.1修复observer退休时留下的orphan guard，明确`FollowSourceKinematicPreview()`与`UpdateBodyRelativeAngularVelocities()`属于生命周期/production输入，不能被trace开关包住。
- v0.2.103确认Knee phase与Fresh/Held/source-age有关。
- v0.2.104 W-only phase-lead physical B REJECT。
- v0.2.105 command→body acceleration lag1；production targetW→bodyW lag1，建立closed-loop基准。
- v0.2.106 opposing-D dominance physical isolation REJECT。
- v0.2.107 raw Render pose/W几乎自洽闭合，filtered/held bridge不是“错误”，而是带group delay。
- v0.2.108–0.2.112依次研究pose-consistent velocity bridge、event conditioning、same-direction pose-debt bridge、paired pose/W contract、FixedStep timestamp alignment；多个数学方案可行但physical版本不改善或引入新的D distortion，因此不写回。
- v0.2.113最终phase budget：true source-W→production target-W约lag3；target-W→bodyW约lag1；true source-W→bodyW约lag4；sourcePose→bodyPose约lag4。
- v0.2.114把target-W generation group delay拆到raw→filtered/held，raw Render-W质量高。
- v0.2.115 full raw-W physical B REJECT。
- v0.2.116发现P-aligned raw-W innovation最干净；v0.2.117 gate persistence PASS，允许窄A/B。
- v0.2.118 P-aligned selective raw-W physical B仍REJECT：nominal lag0改善但best-lag不改善且aligned residual变差。
- v0.2.119/0.2.119.1 broad continuous alpha Pareto：A060虽Fresh lag1，但ALL dominant-D风险>3%，拒绝。
- v0.2.120 transition band `.50/.52/.54/.56/.58/.60`：A054 risk PASS但lag2；A056到lag1同时introduced-D≈3.25% FAIL，crossing集中在.54–.56。
- v0.2.121 A055两次独立run都risk PASS但lag2；A056在三份证据中introduced-D都>3%。因此`FIRST_ORDER_CONTINUOUS_FILTER_TUNING=CLOSED`，production保留alpha=.50。
- v0.2.122删除Pareto runtime observer，组合既有production/safety契约形成Single-Leg freeze certification。

**v0.2.122实机冻结结果**

- `SINGLE_LEG_FREEZE_READY=True / ALL_EXISTING_FREEZE_CONTRACTS_PASS`。
- trace samples≈9704；Hip mean/max≈3.887°/64.590°；Knee≈3.612°/24.579°；KneeOff≈0.438°/6.592°。
- peak Hip/Knee anchor≈12.562/25.741mm；abort=False。
- Two-Body `ALL_GATES_PASS`；source events=46、TRUE discontinuity=2；43/43 evaluable recovery PASS。

**最终决定**

- Single-Leg正式FROZEN，阶段C结束。
- 不继续扫Knee alpha；run-to-run best-lag离散变化不能作为无限调filter的理由。
- future bilateral/full-body若出现新证据，只重开与新证据直接相关的最小变量。

### 0.2.x 最重要的长期教训

1. **物理拓扑先于调参**：Joint、tether、parent、mass/inertia不清楚时，Kp/Kd优化没有稳定意义。
2. **reference可执行性先于source拟合度**：target再“正确”也不能要求body执行数倍于authority的kinematics。
3. **Normal/Source/Recovery不能混成一个控制状态**。
4. **Raw数据更快不等于physical更好**：raw-W多次A/B已经证明phase改善可能伴随D distortion或tracking恶化。
5. **observer必须有生命周期**：问题闭合后就退休；否则性能、控制流和认知负担都会累计。
6. **safety reason必须与同一sample数据一致**：先修报告顺序，不先放宽gate。
7. **删除observer要做orphan-guard audit**：历史v0.2.102.1已经证明仅做symbol scan不够。

---

## 3.3 v0.3.0 — Bilateral Leg Foundation / Frozen Single-Leg Replication（PASS / FROZEN）

### 为什么做

Single-Leg已经冻结，下一步先验证同一份controller能否在同一Pelvis下实例化两次，而不是立刻加contact/gravity。

### 改动

- 两份完全相同`SingleLegShadowChain`；primary来自`singleLegSide`，secondary自动opposite。
- `BilateralLegCoordinator`统一生命周期；不新建Right专用参数。
- `BilateralLegCouplingDiagnostics`只读检查Fixed/Render/Fresh/Held/Drive exact parity。
- 双侧PCA/stack/drive采用原子preflight与fail-closed。

### 实机结果

- clockSamples=7653；mismatch=`0/0/0/0/0`；failClosed=0。
- Left/Right leg samples=7299/7299。
- Hip best lag=1/1；Knee=2/2。
- freeze=True/True；abort=False/False。
- peak Hip/Knee anchor：Left≈15.285/26.726mm；Right≈15.353/26.320mm。
- Two-Body source events=38，TRUE discontinuity=1，36/36 evaluable recovery PASS。

### 决定

`BILATERAL_FROZEN_REPLICATION_PASS`。不分叉Right controller，不调左右参数。

---

## 3.4 v0.3.1 — Shared-Pelvis Common/Differential Coupling Attribution（PASS / CLOSED / Observer Retired）

### 为什么做

v0.3.0证明两份controller能一起稳定运行，但还不知道是否一侧通过shared parent污染另一侧tracking。

### 实机结果

- inherited parity保持全0 mismatch；freeze=True/True；abort=False/False。
- observer valid=7354/7354。
- Hip common share ALL/Source/Recovery/Normal≈0.812/0.926/0.653/0.501。
- Knee common share≈0.473/0.432/0.509/0.476。
- simultaneous Hip clamp 5次，全部Source且common-dominant；Normal为0。
- Normal-only cross：P→S lag2 corr≈0.028；S→P≈0.107。
- Pelvis common correlation≈0.417 > differential≈0.100，但当前拓扑不存在direct leg→Pelvis equal/opposite torque路径。

### 决定

没有material Normal cross-side contamination证据。v0.3.1 observer退休；不要继续为“coupling”调Hip/Knee参数。

---

## 3.5 v0.3.2 — Bilateral Virtual Reaction-Path Counterfactual / Feasibility（SCORABLE / Full Raw Reaction NOT AUTHORIZED）

### 为什么做

v0.3.1显示两腿controller本身没有明显跨侧污染，真正缺口是当前腿部作用没有动量守恒地闭合回shared Pelvis。Contact并不会自动解决这件事，因此先计算如果未来闭合reaction，Pelvis会受到多大增量。

### 改动

- 只读读取actual Hip physical torque与upper-only Hip tether applied acceleration。
- angular CF：`-τHip`。
- tether CF：`-m_upper*a_tether`；把其parent reaction虚拟运输到Pelvis/Hip anchor得到`r×F`。
- 同时保留angular-only与anchor-transport-total视角。
- future dynamic Pelvis使用当前mass/inertia预测`Δα/Δω`与`Δa/Δv`。
- v0.3.1 live coupling observer退休。

### 实机结果

- pre-abort reaction population充分：valid=5101。
- angular-only max≈7.51 rad/s²；Normal max≈7.07。
- full anchor-total P99≈7.20，max≈40.98 rad/s²。
- linear P99≈11.28，max≈104.41 m/s²。
- cancellation偏弱，尤其Source≈0.223。
- 同run末尾Right Knee在NORMAL_TRANSITION触发34.002mm > 30mm safety gate；Left peak≈25.991mm。
- Two-Body仍30/30 recovery PASS。

### 决定

- Full raw tether reaction不允许直接physical write-back。
- Angular-only可以继续作为独立reaction path候选。
- Safety abort不证明v0.3.2 observer造成回归，但暴露了Right Knee低频tail-risk。
- 下一版必须同时解决reaction budget与abort temporal context。

---

## 3.6 v0.3.3 — Reaction Budget / Selective Path Feasibility + Knee Tether Abort Context（PASS / CLOSED / Observer Retired）

### 为什么做

v0.3.2已经说明“是否要reaction”不是二元ON/OFF：Hip angular-only相对温和，而tether linear/anchor path有很大的tail。与此同时一次Right Knee 34.002mm safety event要求我们保留Lower positional closure的pre-abort历史。

### 实现

- production Single-Leg、v0.3.0 bilateral parity/fail-close、30mm Knee gate、Gravity/Colliders全部不变。
- Hip angular保持1.0，虚拟tether force+anchor moment以0.25/0.50/1.00三条lane计算。
- `valid/calculationInvalid/safetyCensored`分离，避免真实safety sample污染数学有效性语义。
- 固定ring保存Knee pre-abort context。

### 关键实机结果

- 7135/7135 valid，0 calculation-invalid，0 safety-censored；完整safety-clean。
- Bilateral：7608 clock samples，mismatch=0/0/0/0/0，failClosed=0，freeze=True/True，abort=False/False。
- Two-Body：39 source events，2 TRUE discontinuities，37/37 evaluable recovery PASS。
- Normal angular-only P95/P99/max≈4.987/6.262/9.604 rad/s²。
- Normal lane P99：25%=5.864（0.936×），50%=5.971（0.954×），100%=8.278（1.322×）。
- Normal linear P99：25%=3.455，50%=6.910，100%=13.821 m/s²。
- tether moment dominant：ALL 4882/7135；Normal 2254/2995。
- Knee未abort；Left/Right peak≈23.623/28.059mm。

### 决定

- Full tether reaction继续NOT AUTHORIZED。
- 25/50% lane不能因为angular P99较低就直接物理开启；linear momentum仍显著。
- Hip angular-only physical study JUSTIFIED。
- 不再扩展reaction budget gain lane；live reaction-budget observer退休。
- 下一步先做isolated dynamic receiver，不直接改production Pelvis。

---

## 3.7 v0.3.4 — Shared-Pelvis Dynamic Reaction Receiver / Authority-Isolation Foundation（首轮实机完成 / Sandbox PASS / Reaction 结论转为 PROVISIONAL）

### 为什么做

v0.3.3 的 CF 仍不是实际 PhysX dynamics，而 production Pelvis 又是 kinematic。为避免一次性改变 Two-Body、Chest joint 与双腿 parent，v0.3.4 建立两个完全隔离的 dynamic shadow Pelvis：A=source authority only，B=A+exact bilateral Hip angular reaction。

### 实机结果

- 7428 samples，7427 valid，invalid/reset/safetyCensored=0/0/0；bilateral 与 Two-Body frozen foundation 均 PASS。
- A source-only Normal error P99≈13.125°，B≈13.463°；逐 sample positive penalty P99≈0.541°，Normal A/B delta rotation P99≈0.785°，delta-W P99≈0.197 rad/s。
- A/B Normal authority clamp=0/0，整run clamp=51/51；表面上 Hip angular reaction 没有增加 source-authority saturation。
- 但 build/status 同时暴露 receiver 从 production kinematic Pelvis 复制到的 inertia 为 **`(1,1,1)`**。

### v0.3.5 对 v0.3.4 的历史纠错

- 当前 Pelvis proxy 约为 8kg、`~0.227×0.110×0.169m` 的箱体；解析均匀箱体 inertia 量级应约 `Ixx~0.027 / Iyy~0.054 / Izz~0.043 kg·m²`，与 `(1,1,1)` 相差约 19–37×。
- production Pelvis 是 kinematic 且 collider 默认关闭；其 default-like tensor 从未经过“future dynamic Pelvis mass-property foundation”验证。
- source-authority law 使用 inertia-aware torque composition，所以 **A 的 source-only tracking 不一定因 tensor 大小而变化**；但 fixed external Hip reaction torque 的真实 angular acceleration 会直接受 inertia 影响。因此 v0.3.4 的“reaction penalty 很小”只能保留为 **copied/default-inertia witness**，不能继续当作 physical Pelvis reaction feasibility closure。

### 决定

- v0.3.4 的 isolated PhysX sandbox 架构本身 PASS。
- source-only A `Normal P99≈13.1°` 仍是需要解释的 authority/phase 现象。
- Hip angular reaction 的 production/physical feasibility **重新打开**，必须先在 explicit physical Pelvis inertia 上复核。
- production Pelvis 继续 kinematic；禁止因 v0.3.4 小 penalty 直接做 reaction write-back。

---

## 3.8 v0.3.5 — Dynamic Pelvis Source-Authority Tracking Attribution / Inertia Foundation（首轮实机完成 / CLOSED）

### 为什么做

v0.3.4 的A source-only Normal P99约13°，同时shadow copied kinematic Pelvis tensor暴露为 `(1,1,1)`。如果直接调Kp/Kd或相信v0.3.4 reaction penalty，会混淆“错误dynamic mass-property foundation”与“真正source-authority phase问题”。因此v0.3.5用L0/A/B同钟shadow先纠正inertia基础并重新评分reaction。

### 关键实机结果

- dataset：7054 samples，7053 valid，invalid/reset/safetyCensored=0/0/0；Bilateral与Two-Body freeze全部PASS。
- copied inertia=`(1,1,1)`；analytical uniform-box=`(0.027136,0.053456,0.042573)`；ratio≈`36.85/18.71/23.49`。
- source-only Normal P99：L0≈`12.51272°`，A≈`12.51276°`，ratio≈1.000003。说明source-authority inertia compensation正常；约13°tail不是copied inertia导致。
- A Normal phase：Steady Fresh/Held P99≈`4.043/4.050°`；Transition Fresh/Held≈`14.225/14.250°`。Normal Transition是主要owner，Fresh/Held不是。
- A Normal raw-A P99/max≈`16.997/22.174 rad/s²`，MaxA=40，clamp=0；**MaxA shortage REJECTED**。
- lag1 command→measured-A：alignment≈0.586，gain mean≈1.053；响应幅值总体闭合，但方向/transition tracking仍需进一步拆解。
- corrected-inertia full Hip reaction B：A Normal P99≈`12.513°`，B≈`40.952°`；positive penalty P99≈`40.191°`。

### 决定

- v0.3.4“小reaction penalty”结论正式SUPERSEDED；它只代表default-like高惯量witness。
- full 100% Hip angular reaction **NOT AUTHORIZED** for isolated local-Pelvis receiver。未来connected whole-body effective inertia可能不同，因此此结论不能偷换成“全身永远不能equal-and-opposite”；reaction allocation必须以后在更真实connected-body receiver上重新研究。
- source-only的主要问题不是MaxA、Fresh/Held或inertia，而是Normal Transition moving-target tracking。
- v0.3.5 L0和100% reaction B live observer退休。下一步只研究Transition source kinematic feedforward。

---

## 3.9 v0.3.6 — Dynamic Pelvis Normal-Transition Kinematic Feedforward Causal A/B + Physics-Foundation Audit（实机完成 / CLOSED）

### 为什么做

v0.3.5 已把source-only high tail定位到Normal Transition，而且raw authority远低于MaxA；因此v0.3.6不调Kp/Kd/MaxA，而用同一analytical-inertia shadow建立A0=`P+D`与A1=`P+D+d(targetW)/dt` causal A/B。同时全库审计继续检查dynamic Rigidbody是否存在default-like COM/inertia foundation。

### 关键实机结果

- 日志前段曾有一次Right Knee 30mm fail-closed：31.553mm>30mm；对应TRUE kinematic discontinuity，raw rotation≈46.81°、raw W≈24.27rad/s、raw A≈729.83rad/s²。安全门行为正确，不放宽gate。
- 重启/重建后的主run：bilateral clock mismatch `0/0/0/0/0`，failClosed=0，freeze=True/True，abort=False/False；Two-Body `ALL_GATES_PASS`。
- A/B Normal P99：A0≈12.249°，A1≈8.616°。
- Transition Fresh：13.233°→8.295°；Transition Held：13.230°→8.794°。
- Steady Fresh：5.455°→5.517°，基本不变。
- targetA P99/max≈42.866/93.677rad/s²；actual FF increment P99≈32.429；Transition clamp A0/A1=0/19。
- benefit/penalty P99≈5.727/2.862°：exact derivative有明确净收益，但tail中存在overshoot/phase cost，因此不能直接production ON。
- foundation audit：production Pelvis与Chest均读回COM=0、I=(1,1,1)，与各自Box.center/uniform-box inertia明显不匹配；Chest在drive ON后dynamic，所以这是现役Two-Body foundation issue。

### 决定

- `NORMAL_TRANSITION` moving-target angular-acceleration FF causal hypothesis：**SUPPORTED / CLOSED**。
- exact unconditioned derivative：**NOT PRODUCTION-READY**；以后需要bounded/conditioned版本。
- v0.3.6 live Transition-FF observer/report退休。
- production torso mass properties不在v0.3.6直接修改；优先进入v0.3.7 full-topology mass-property causal requalification。

---

## 3.10 v0.3.7 — Two-Body Explicit Mass-Property Foundation / Dynamic Chest Causal Requalification（已关闭 / REJECTED）

### 为什么做

v0.3.6确认production dynamic Chest仍使用default-like COM=0、I=(1,1,1)。由于这已经成为历史冻结controller的plant参数，不能直接“修BUG”，因此v0.3.7建立两套断开的完整Two-Body replica：A保持legacy implicit mass properties；B只把Pelvis/Chest改成Box.center + analytical uniform-box inertia。

### 关键实机结果

- dataset：6781/6781 valid，Normal=2962，Steady=2080，Transition=882。
- production bilateral、Two-Body freeze、左右Single-Leg freeze均PASS；Knee safety无abort。
- Normal rotation P99 A/B≈`7.926/18.826°`。
- Normal-Steady P99≈`2.792/5.482°`；Normal-Transition≈`8.969/20.769°`。
- Normal Chest-position P99≈`62.704/88.154mm`。
- Normal CharacterJoint anchor P99≈`24/23µm`，几乎不变；drive-loss=`0/0`。
- Normal angular clamp A/B=`0/5`。

### 解释与决定

- Full explicit mass-property migration在当前冻结controller下 **REJECTED**；不能直接写回production。
- anchor closure不恶化，说明不是CharacterJoint几何/anchor calibration失效；主要回归来自dynamic Chest rotational plant与既有authority law不匹配。
- `(1,1,1)`虽然非物理，但已经成为历史controller共同收敛的隐式plant参数；以后必须按plant migration处理，而不是普通字段修正。
- 不能立即调Kp/Kd/MaxA；先进入v0.3.8 factorization拆COM与inertia owner。

---

## 3.11 v0.3.8 — Two-Body COM vs Inertia Foundation Factorization（已关闭）

### 实机结果

- samples/valid/invalid=`5332/5332/0`；Normal=`2542`，Steady=`1935`，Transition=`607`。
- Normal rotation P99 P/C/I/F≈`8.589/8.365/11.917/9.791°`。
- Steady P99≈`2.650/2.617/6.602/4.147°`。
- Transition P99≈`9.545/9.515/20.677/19.009°`。
- Normal angular clamp P/C/I/F=`0/0/2/2`；drive-loss C/I/F=`0/0/0`。
- 本run production无abort、failClosed=0、bilateral clock mismatch=`0/0/0/0/0`。完整Two-Body freeze未ready仅因为25个source event中TRUE discontinuity=0，属于coverage不足，不是regression。

### 结论

- **COM-only≈P：COM不是primary regression owner。**
- **I显著坏于P：explicit inertia是primary owner。**
- F仍明显坏于P但好于I-only：COM correction在corrected inertia下部分mitigate rotation debt，plant存在material non-additive interaction。
- 不能将v0.3.7退化归因于CharacterJoint anchor closure，也不能把少量clamp当作唯一root cause。
- v0.3.8科学问题关闭，不因缺少TRUE discontinuity重跑。

### CharacterJoint架构澄清

- Single-Leg在build初始化阶段仍会创建Unity CharacterJoint；执行`arl_hip_stack(on)`后的当前冻结production Hip/Knee均无CharacterJoint，使用ARL-owned tether/PD/actuator。
- Two-Body Pelvis↔Chest仍是Unity原生`UnityEngine.CharacterJoint`；ARL自定义的是relative-PD、parent-FF、Pose-Bridge、source/recovery/freeze。
- v0.3.8没有证据要求立刻移除torso CharacterJoint；先做corrected-inertia controller/constraint response attribution。

## 3.12 v0.3.9 — Corrected-Inertia Two-Body Controller Requalification / Response Attribution（已关闭）

### 实机结果

- dataset：samples/valid/invalid=`5692/5692/0`；Normal=`2579`，Steady=`2175`，Transition=`404`。
- Normal rotation P99 P/F≈`6.269/17.622°`；Steady≈`2.454/3.833°`；Transition≈`8.589/19.233°`。
- target-W error P/F≈`0.782/1.368 rad/s`。
- lag1 command→measured relative-A alignment P/F≈`0.490/0.433`；gain≈`4.125/2.979`；target-W→body-W best lag=`0/0`。
- F drive-loss=0，dataset完整可评分。corrected-inertia regression再次复现，但不是简单的额外固定帧延迟。

### 同run Knee safety事件

- Right，fixed=`4621`，phase=`NORMAL_TRANSITION`，`36.199mm > 30mm`。
- fixed 4612–4618 anchor约`2–4mm`；随后约`20.85 → 29.60 → 36.20mm`三拍快速增长。
- 4619 raw tether demand约`115.07m/s²`、final applied=`60m/s²`；4621 raw≈`66.73`、applied=`60`、parent-FF≈`35.72m/s²`。
- 这支持“局部debt injection rate短时超过repayment rate”，不支持“持续运动时间越久就必然无限漂移”。

### 决定

- v0.3.9 corrected-inertia response attribution科学问题关闭；额外F replica退休，避免继续占PhysX预算。
- corrected COM+inertia仍未production-authorized；Two-Body controller adaptation暂缓，先解决当前更直接的Knee safety tail与可能的frame-stall触发关系。
- 不提高Knee 60m/s² authority，不放宽30mm gate。

## 3.13 v0.3.10 — Knee Positional Debt / Frame-Stall Correlation Attribution（已关闭）

### 为什么做

v0.3.9出现Right Knee三拍`3mm→20.85→29.60→36.20mm` hard abort，同时用户观察到偶发卡顿时六个proxy会整体脱离再恢复。v0.3.10专门验证stall是否是Knee 30mm failure的直接owner。

### 多轮实机结果

- 第一轮自然强stall但无abort：Render max约`169ms`、wall-gap约`234ms`、Fixed catch-up max=`10`、source age约`159ms`、held depth=`9`；Left/Right Knee peak约`25.173/24.954mm`。
- 后续更强负对照：Render max约`232ms`、wall-gap约`389ms`、Fixed catch-up max=`14`、source age约`219ms`、held depth=`13`；仍`7240/7240 valid`、Knee peak约`21.851/21.212mm`、bilateral freeze PASS。
- 第二轮最大单拍positive Knee debt仍可到约`15mm`，但随后controller成功偿还，没有形成连续avalanche。
- 最新Knee debt correlation：renderDt约`0.21`，wall-gap/source-age/fixed-index接近0；whole-proxy detachment对render/source-age/fixed-index约`0.35/0.47/0.32`，明显更高。
- 用户随后多次按相同流程复测仍未重新捕获原始hard abort。

### 决定

- **Frame stall支持“六proxy整体瞬时脱离”的视觉owner，但不是Knee 30mm abort的充分或主要直接owner。**
- 历史Knee abort归类为`RARE_STATE_DEPENDENT_MULTI_FACTOR_DEBT_AVALANCHE`；可能需要timing disturbance、gait/transition phase、parent-anchor acceleration、existing debt与60m/s² repayment budget连续几拍共同不利。
- 不继续人工追求复现；不提高Knee 60m/s² authority，不放宽30mm hard gate。
- heavy timing/debt observer退休，只留minimal Knee safety ring。
- v0.3.10 CLOSED。

## 3.14 v0.3.11 — Corrected-Inertia CharacterJoint Solver Reaction / Effective Dynamics Attribution（已关闭）

### 为什么做

v0.3.7–v0.3.9已经稳定确认corrected inertia会让Two-Body Steady/Transition rotation tracking恶化，而COM-only并非primary owner；Two-Body角向执行又是`ForceMode.Acceleration`，因此需要直接观察Unity CharacterJoint solver reaction是否在较小真实inertia下形成了额外effective angular response。

### 当前实现

- P=untouched production legacy torso；F=一个isolated explicit COM+uniform-box inertia完整Two-Body replica。
- F仍使用相同kinematic Pelvis、Unity built-in CharacterJoint、dynamic Chest、frozen relative-PD、linear parent-FF与Pose Bridge；Gravity/Colliders/reaction OFF。
- FixedUpdate入口读取`CharacterJoint.currentForce/currentTorque`，与刚settle的上一物理拍measured motion同相位。
- 记录native torque、`r(COM→anchor)×force`、combined moment，并通过各lane真实inertia换算inertia-only solver angular-A proxy。
- observer保存上一拍PD angular command，在下一拍计算`measured relative-A - previous command`，同时评估`-solver`和`+solver`两种closure符号；不假定Unity API sign。
- 统计force-moment/native-torque dominance、solver-A/command ratio、solver/command opposition、joint force vs parent-FF equivalent force abs-alignment，以及Steady/Transition closure improvement。
- production controller/mass properties/CharacterJoint settings完全不变；Knee hard safety仅保留12拍minimal ring。

### 首轮要回答

1. F的solver angular-A proxy是否相对P显著放大。
2. solver proxy是否能显著解释F的`measuredA-command` residual；哪一个统一sign更符合数据。
3. `r×currentForce`还是native `currentTorque`主导combined moment。
4. joint force是否与linear parent-FF形成稳定强constraint coupling；正负方向交给整体±solver closure判定。
5. Steady与Transition是否由同一solver mechanism拥有；若只有Transition强，则再回到v0.3.6 moving-target acceleration语义。

### 决策边界

- 不因单次`currentForce/currentTorque`峰值直接修改production。
- solver-A proxy忽略gyro，只是归因witness；只有长期P/F差异 + closure改善共同成立才支持CharacterJoint/effective-inertia owner。
- 若solver coupling成立，下一版再设计最窄causal isolation/requalification；若不成立，回到parent-FF/reference/transport链，不盲调Kp/Kd/MaxA。

### 实机结果 / 决定

- `5232/5232 valid`，Normal=`2493`（Steady/Transition=`2189/304`），F drive-loss=0。
- Normal rotation P99 P/F=`7.482/17.201°`；solver angular-A proxy P99=`6.972/22.575 rad/s²`。
- force-moment dominant share P/F=`1/1`：`r×currentForce`全面大于native `currentTorque`。
- best solver sign P/F均为`MEASURED_MINUS_COMMAND_MINUS_SOLVER`；P99 closure improvement约`0.110/0.350`。
- 结论：CharacterJoint force-moment coupling是corrected-inertia regression的**material contributor**，native currentTorque不是primary owner；但35% closure不足以宣称CharacterJoint已是完整root cause。
- v0.3.11 CLOSED；下一步必须做physical cancellation A/B，而不是继续增加read-only observer。

## 3.15 v0.3.12 — Corrected-Inertia CharacterJoint Force-Moment Causal Cancellation A/B（已关闭 / STRONG PASS）

### 为什么做

v0.3.11已经证明`r(COM→joint anchor)×Joint.currentForce`在corrected-inertia torso中显著放大，并解释material share的response residual；v0.3.12进一步验证它是否真能作为physical causal lever，而不只是相关witness。

### 实现

- P=untouched production legacy torso。
- F0=isolated explicit COM+uniform-box inertia完整Two-Body frozen baseline。
- F1=与F0完全相同的第二个corrected replica；唯一差异是manager在F1完成冻结`TwoBodyShadowProxy.FixedUpdate()`后，对F1 Chest追加上一拍settled `r×Joint.currentForce`的equal-opposite `ForceMode.Force` torque。
- cancellation gain=1；torque guard=`max(inertiaTensor) × MaxAngularAcceleration × 2`，仅作为isolated candidate异常边界。
- production、F0、bilateral legs均不接收cancellation。

### 实机结果

- dataset=`5248/5248 valid`，invalid=0；Normal=`2263`，Steady/Transition=`1971/292`。
- Normal rotation P99 P/F0/F1≈`6.365/17.693/6.880°`；F1/F0≈`0.3889`，即相对F0下降约61.1%。
- F0相对production P新增的Normal P99 regression约`11.328°`，F1仅残余约`0.515°`；按该scalar tail计算，约95.5%的F0 excess被intervention追回。
- F1 cancellation torque P99≈`4.488Nm`，guard≈`10.146Nm`，Normal clamp=0；不是靠guard截断得到的假改善。
- solver angular-A witness F0/F1≈`21.559/38.222 rad/s²`：F1 tracking改善时native solver reaction反而增大，说明intervention改变的是闭环constraint balance，而不是“让CharacterJoint不再出力”。
- F0/F1 drive-loss=`0/0`；Knee hard safety PASS，Left/Right Knee peak约`22.167/22.853mm`。
- production Two-Body Normal tracking/recovery仍健康；本run TRUE discontinuity=0，因此full freeze recertification只因coverage不足而NOT EVALUABLE。

### 决定

- **CharacterJoint linear constraint force经COM→anchor lever形成的`r×currentForce` rotational channel，升级为STRONGLY SUPPORTED physical causal contributor。**
- v0.3.12 cancellation是causal probe，不是production controller授权；上一拍solver reaction反向回灌会形成隐含delay feedback，full-body/contact阶段风险更大。
- 不继续做cancellation gain sweep；下一步应追它的上游激发源。第一优先变量是当前linear ParentMotionFeedforward，因为它直接推动Chest COM，同时CharacterJoint必须维护anchor closure。
- v0.3.12 CLOSED / STRONG PASS。

## 3.16 v0.3.13 — Corrected-Inertia CharacterJoint Force-Moment Source Isolation / Parent-FF Causal A/B（已关闭 / HYPOTHESIS REJECTED）

### 为什么做

v0.3.12已证明`r×currentForce`是强物理causal lever，但previous-step cancellation不能产品化。v0.3.13测试linear ParentMotionFeedforward是否是持续制造CharacterJoint constraint force / force moment的主要上游激发源。

### 实现

- P=untouched production legacy torso，Parent FF保持冻结ON。
- F0/F1均为相同corrected explicit COM+uniform-box inertia完整Two-Body；F0 Parent FF ON，F1 Parent FF OFF。
- 其它CharacterJoint geometry、relative-PD、Pose Bridge、source sampling、kinematic Pelvis/dynamic Chest、Gravity/Colliders一致。
- v0.3.12 cancellation完全删除，manager无额外force/torque write。

### 实机结果

- clean dataset=`5195/5195 valid`，Normal=`2533`，Steady/Transition=`2203/330`。
- Normal rotation P99 P/F0/F1≈`7.142/15.064/31.842°`，F1/F0≈`2.114`。
- Steady F0/F1≈`4.554/5.708°`；Transition≈`19.195/42.628°`。
- force moment P99 F0/F1≈`2.409/3.261`；solver-A≈`28.654/36.908 rad/s²`。
- Chest position P99 F0/F1≈`68.664/94.511mm`；drive-loss=`0/0`。
- 同日志第一次安全门前独立segment=`3656/3656 valid`，Normal P99 P/F0/F1≈`8.320/17.925/37.329°`，F1/F0≈`2.082`；方向和幅度重复。
- 中途Left Knee约`30.692mm` hard gate发生于SOURCE_TURN、fresh source（age约3.19ms，held=0），不是stall-only事件；episode内曾出现raw tether A约`98.96m/s²`并有2拍60m/s² clamp，支持“先注入debt、稍后轻微越界”的解释。后半段clean rerun无abort。

### 决定

- **ParentMotionFeedforward作为有害force-moment上游source的假说REJECTED。**
- Parent FF OFF反而增加Chest position error、CharacterJoint force moment、solver rotational influence与rotation tail，因此Parent FF当前应视为**稳定/load-sharing transport authority**并保持ON。
- 更基础的owner继续指向CharacterJoint positional closure通过非零Chest COM→anchor lever形成的rotational channel。
- 不删Parent FF，不调Kp/Kd/MaxA，不迁移production inertia。
- v0.3.13 CLOSED。

## 3.17 v0.3.14 — Corrected-Inertia CharacterJoint COM-Aligned Anchor / Lever-Arm Causal A/B（已关闭 / STRONG PASS）

### 为什么做

v0.3.12证明`r×currentForce`是强causal contributor，v0.3.13又排除Parent FF为有害上游source。下一步直接测试最机械的变量：**Chest COM到native CharacterJoint constraint anchor的lever是否本身拥有corrected-inertia regression。**

### 实现

- P=untouched production legacy torso。
- F0=isolated corrected explicit COM+uniform-box inertia baseline，Parent FF ON，native CharacterJoint anchor仍为calibrated source/spine point。
- F1与F0完全相同，唯一机械变化是Chest native CharacterJoint anchor=`Chest.centerOfMass`；connectedAnchor重新计算到同一初始世界点，避免build瞬间native anchor错位。
- **source/control virtual anchor与native CharacterJoint anchor彻底分离。** F1不修改旧`_virtualChestAnchorLocal/_virtualPelvisAnchorLocal`，因此Parent FF、source/reference transport与既有diagnostic reference仍使用冻结语义。
- build-time同时验证mass-property parity、Parent FF ON/ON、F0 anchor mode、F1 COM-aligned mode、Unity CharacterJoint anchor/connectedAnchor实际readback，以及F1初始COM lever/native anchor closure接近0。
- 新report同时保存source/control anchor P99、native constraint-anchor P99、Chest COM lever P99、joint force/native torque/force moment/solver-A、rotation/position/target-W error、clamp与drive-loss。

### 判定边界

- 若F1 lever≈0、native constraint anchor保持健康，同时`r×currentForce`显著下降并让rotation接近production/F0明显改善，则**lever-arm geometry是强结构性causal owner**；后续应设计更物理一致的COM-aligned translation transport或jointless torso topology。
- 若F1 forceMoment下降但rotation不改善，则v0.3.12 cancellation收益还包含其它solver-feedback/constraint dynamics，不能把几何lever单独当root cause。
- 若F1 source/control anchor或Chest position严重恶化，即使rotation改善也只能视为causal probe，不能直接production迁移。
- 若F1 drive-loss或native constraint anchor不闭合，则本run不评分。
- 当前30mm Knee hard fail-close保持不变；future soft recovery只在full-body resilience阶段实现。

### 实机结果与决定

- dataset=`5409/5409 valid`；Normal=`2418`，Steady/Transition=`2043/375`。
- Normal rotation P99 P/F0/F1≈`8.114/17.042/8.028°`，F1/F0≈`0.471`；Steady F0/F1≈`4.007/2.251°`，Transition≈`20.117/8.991°`。
- Chest COM→native anchor lever P99 F0/F1≈`83.093/0mm`；`r×currentForce` P99≈`1.729/0.000001Nm`；solver angular-A proxy≈`22.510/0.003rad/s²`；Chest position P99≈`50.992/40.366mm`。
- **决定：v0.3.14 CLOSED / STRONG PASS。** 在当前`kinematic Pelvis -> CharacterJoint -> dynamic Chest`拓扑中，CharacterJoint线性约束力通过非零Chest COM lever形成的`r×currentForce`是corrected-inertia rotation regression的强结构causal owner；结论不能直接外推到future fully-dynamic whole body。

## 3.18 v0.3.15 — Parent-Side Reaction / Lever-Transfer Attribution（已关闭 / STRONG LEVER-TRANSFER EVIDENCE）

### 为什么做

v0.3.14消除了F1 Chest侧力臂，但同一CharacterJoint仍在同一world constraint point对connected Pelvis施加equal-and-opposite internal force。当前Pelvis是kinematic，父侧可能存在的旋转载荷被动力学条件隐藏；因此本版只读估算`rPelvis × (-currentForce)`及corrected Pelvis inertia-only angular-A，判断COM-aligned CharacterJoint是完整拓扑候选，还是仅把lever burden从Chest转移到了父侧。

### 实现与首轮测试前审计修正

- 完整复用v0.3.14 P/F0/F1 physical replicas；production torso/legs、Parent FF、Kp/Kd/MaxA、Pose Bridge/source/reference、Gravity/Colliders与30mm Knee hard fail-close均不变。
- 新增`TwoBodyParentSideLeverTransferAttribution.cs`，继续复用`arl_twobody_anchor_status`，不新增命令，也不包含Rigidbody/Joint physical write。
- `currentForce/currentTorque`、Chest/Pelvis COM lever与native constraint closure统一在`CaptureSolvedCharacterJointReaction()`内形成同一个previous-settled snapshot；observer不再把上一拍reaction与post-FixedUpdate live Transform geometry配对。
- F0/F1 corrected foundation的build parity与status/finalize readback统一覆盖Chest/Pelvis的`centerOfMass`、`inertiaTensor`、`inertiaTensorRotation`、proxy box size和box center。
- Runtime拆分production torso、observer replicas与bilateral production legs的异常边界：production torso失败时本拍停止以避免腿读取stale parent；observer失败只fail-close F0/F1实验，production bilateral legs独立继续。
- parent witness只有在`correctedPelvisParity=True`、`productionPelvisKinematic=True`、`invalid=0`、reaction样本充足且`fault=False`时才允许`scorable=True`；每次trace重置deterministic reservoir序列。

### 预注册判断

- 若F1保持Chest moment≈0，但Pelvis-side moment / inertia-only A明显大或相对F0增加：v0.3.14结论保持，但COM-aligned CharacterJoint只能视为kinematic-parent-specific workaround / lever-transfer candidate；下一步做isolated fully-dynamic Pelvis/Chest或COM-centered force transport / jointless translation closure。
- 若F1父侧burden也bounded：COM-aligned CharacterJoint可升级为corrected-mass-property topology candidate，但仍只进入isolated dynamic Pelvis/Chest physical A/B，不能直接迁入production。
- Chest/Pelvis moment是同一internal force pair相对不同COM的body-local rotational burden，**禁止相加并称作净外力矩**；production Pelvis仍为kinematic，因此parent angular-A只是counterfactual feasibility witness。

### 实机结果与决定

- clean dataset=`5108` samples；Normal=`2575`，invalid=0，observer fault=False，correctedPelvisParity=True，productionPelvisKinematic=True，scorable=True。
- v0.3.14 core同run复现：Normal rotation P/F0/F1≈`8.007/18.428/8.095°`，F1/F0≈`0.439`；因此parent witness建立在重复成功的Chest-side causal A/B上。
- joint-force P99 F0/F1≈`65.469/67.885N`，F1仅高约3.69%；不是force magnitude本身发生数量级变化。
- Chest lever P99约`82.790/0.000119mm`，Chest moment P99约`2.049806/≈0Nm`；Chest侧lever/moment继续按预期塌缩。
- Pelvis lever P99约`177.969/247.069mm`；parent moment P99约`7.796/12.144Nm`，F1/F0≈`1.558`；corrected Pelvis inertia-only A P99约`270.275/438.273rad/s²`，F1/F0≈`1.622`。
- 同run bilateral clock mismatch全0、failClosed=0、Knee peak约`22.997/25.130mm`，abort=False/False；没有TRUE discontinuity，因此不顺带宣称新的full-freeze recertification。
- **决定：v0.3.15 CLOSED / STRONG LEVER-TRANSFER EVIDENCE。** Chest-COM native CharacterJoint anchor在kinematic-parent topology中是有效workaround，但不是已验证的full-body topology；下一步只能进入isolated fully-dynamic Pelvis/Chest physical A/B，不能直接修改production。

## 3.19 v0.3.16 — Isolated Fully-Dynamic Pelvis/Chest Rotational Receiver Foundation（已关闭 / FOUNDATION STRONG PASS）

### 为什么做

v0.3.15证明Chest-COM native anchor会把更大的body-local force moment转移到当时被kinematic条件隐藏的Pelvis侧。必须让connected Pelvis真正进入dynamic rotation，才能判断这种lever transfer在真实receiver上是否转化为Pelvis rotation regression。

### 关键变化与实现边界

- 建立独立F0/F1 dynamic Pelvis + dynamic Chest + native `CharacterJoint` sandbox；Pelvis只`FreezePosition`，因此本阶段是**fully-dynamic rotational receiver**，不是完整translation/contact whole body。
- 两lane显式统一box COM/inertia、mass、drag、joint limits、controller与两拍passive activation；production Two-Body/Single-Leg完全不接收sandbox feedback。
- 早期实机连续出现“切dynamic后一拍米级爆炸”。分层隔离证明controller不是first-blowup owner，`enablePreprocessing`也不是充分原因。
- 最终定位到run-start `Rigidbody.position/rotation`与对应`Transform`状态域可发生米级分裂，而旧anchor preflight又混用了Transform-domain与Rigidbody-domain，因此会在损坏初态上假通过。
- foundation最终固定为：kinematic reset同时同步Rigidbody+Transform；rebind/safety/anchor/COM geometry统一使用Rigidbody pose domain；dynamic toggle前后均有Rigidbody↔Transform hard gate；仍保留两拍无controller passive PhysX solve。

### 最终实机证据与决定

- 修复后run-start四个body的Rigidbody↔Transform position/rotation delta均为0；两个passive physics solve均健康，anchor closure保持亚毫米，随后正常进入controller stage。
- fully-dynamic dataset=`5352`，Normal=`2467`，invalid=`0`，reaction-valid F0/F1=`2467/2467`，sandbox fault=False，`scorable=True`。
- preprocessing ON/OFF在Pelvis rotation、Chest-relative rotation、anchor、Pelvis moment与authority上几乎重合；该solver开关对当前receiver没有material effect，后续固定ON以减少变量。
- 同run production Knee peak约`19.976/22.129mm`，abort=False/False；bilateral clock mismatch全0、failClosed=0。Two-Body freeze未重新认证仅因没有TRUE discontinuity，不是tracking regression。
- **决定：v0.3.16 FOUNDATION STRONG PASS。** v0.3.16.2–v0.3.16.4 first-solve explosion的共同实现根因是Rigidbody/Transform state-domain desynchronization + mixed-domain geometry preflight；fully-dynamic rotational receiver foundation现已可用于真正native-anchor causal A/B。

## 3.20 v0.3.17 — Fully-Dynamic Native CharacterJoint Anchor Causal A/B（当前 / 待实机）

### 为什么做

v0.3.16已经把dynamic receiver foundation与状态域问题闭合，因此现在可以重新回答v0.3.15留下的核心物理问题：Chest-COM anchor在kinematic Pelvis时能消除Chest侧lever moment、但会增加Pelvis侧hidden burden；当Pelvis能够真实旋转响应后，这种burden transfer究竟改善、无效还是恶化整体动态表现。

### 唯一A/B变量与冻结边界

- F0=`CALIBRATED_SOURCE_JOINT`：Chest/Pelvis native anchor保持source/spine calibrated world point。
- F1=`CHEST_COM`：只把Chest local native anchor设为显式`centerOfMass`，再以同一world point重算Pelvis `connectedAnchor`，保证初始closure≈0。
- 两lane `enablePreprocessing=True/True`；mass/COM/inertia、joint axis/limits、Pelvis/Chest controller、authority、activation lifecycle、Gravity/Colliders与state-domain hardening全部一致。
- 继续保留两拍passive dynamic solve；任何Rigidbody↔Transform sync、anchor semantics、foundation parity或150mm runtime geometry gate失败都只fail-close sandbox。
- production Two-Body、Single-Leg、30mm Knee hard gate、PCA、bilateral fail-close与所有frozen control参数保持不变。

### 预注册判定

- 只有foundationParity=True、四body dynamicObserved=True、productionPelvisKinematic=True、invalid Normal=0、fault=False，且paired Normal/reaction-valid样本≥128时才评分。
- 预注册material gate：F0 Chest-moment P99必须≥`0.02Nm`形成足够excitation；F1/F0 Chest lever与Chest force-moment ratio均≤`0.25`才算明确塌缩；F1 anchor P99必须≤`1mm`；F1/F0 Chest-relative rotation、dynamic Pelvis rotation与Pelvis-side force-moment ratio均≤`1.10`才判SUPPORT。
- 若Chest moment已塌缩但Pelvis rotation或Pelvis-side moment>F0的`1.10x`，则拒绝Chest-COM作为dynamic-parent topology candidate；若child tracking/anchor bound先失败，则按constraint/child regression拒绝。
- 无论结果如何，本版仍不直接授权production migration：Pelvis translation仍被冻结，Chest controller仍是child-only。

### 同版维护清理

- 退休已经完成历史因果任务、且输出不再回流controller/reference governor/tether safety/PCA/bilateral fail-close的旧Single-Leg只读observer；历史证据由旧ZIP与本长期记录保留。
- 清理原则是“删observer，不重写frozen controller”：保留`SingleLegFreezeReadinessCertification`、Hip/Knee translation tether safety、PCA、mass-property foundation、Hip/Knee authority与governor、mechanical parent、bilateral clock/fail-close及current experiment所需模块。
- 活动C#文件由62降至50；C#源码体积约减少22.6%，不以进一步压缩为目标，避免为少量空间收益改写稳定report/control schema。


---

# 4. 当前活动源码与退休模块

## 4.1 当前必须保留

**Production / frozen control**

- Two-Body foundation、Render→Physics Pose Bridge、Parent FF、Source/Recovery freeze/fail-fast。
- Explicit Uniform Box mass/COM/inertia与Knee PCA。
- Hip strict/control-frame/reference Governor、source/composite shock absorber。
- no-Hip/no-Knee CharacterJoint serial-chain translation tether。
- Knee parent angular transport、Hip/Knee anchor acceleration FF、feedback-priority allocation。
- v0.2.81 Hip inertia/gyro actuator、drag=3、Hip 40/80/80、Knee/soft-Knee frozen parameters。

**当前 experiment / read-only / safety**

- `SingleLegFreezeReadinessCertification.cs`。
- `BilateralLegCoordinator.cs`。
- `BilateralLegCouplingDiagnostics.cs` + `BilateralLegReport.cs`（v0.3.0 parity/foundation，仍活动）。
- `SingleLegBilateralReadOnlySnapshot.cs`。
- `TwoBodyFullyDynamicAnchorIsolationAttribution.cs`（v0.3.17当前fully-dynamic native-anchor A/B；F0=calibrated source/spine，F1=Chest-COM；两lane preprocessing ON；继承v0.3.16 state-domain hardening，只读production Pose-Bridge command/report，不向production或Single-Leg反馈）。
- `KneeSafetyContextRecorder.cs` + `KneeSafetyContextReport.cs`（minimal 12-step hard-safety ring；不再做timing correlation）。

## 4.2 已退休，不应回灌活动路径

- **v0.3.17 maintenance retirement — historical Single-Leg read-only attribution package**：`SingleLegAttributionDiagnostics`、`SingleLegAngularDynamicDiagnostics`、Knee mild/closed-loop旧observer、soft-Knee recovery、whole-leg/body-recovery、Hip settled catch-up / physical reaction / constrained-body、v0.2.92 severity与v0.2.93 packetization live observers已从runtime/source tree退休。它们的输出不参与当前actuator、reference governor、translation tether safety、PCA、freeze readiness或bilateral fail-close；需要复核历史结论时从对应旧ZIP恢复最小必要逻辑，不在主干常驻。
- v0.3.15 `TwoBodyParentSideLeverTransferAttribution.cs`：parent-side same-phase只读witness已关闭；已证明Chest-COM alignment把更大的body-local moment转移到kinematic Pelvis侧，runtime observer由v0.3.16真实dynamic receiver取代。
- v0.3.14 `TwoBodyComAlignedAnchorIsolationAttribution.cs` + `TwoBodyComAlignedAnchorIsolationReport.cs`：kinematic-Pelvis/dynamic-Chest anchor A/B已STRONG PASS并由v0.3.15完成parent-side解释；旧live replicas不再运行。需要历史复核时从v0.3.14/v0.3.15 ZIP恢复，不回灌current runtime。
- v0.3.13 `TwoBodyParentFeedforwardIsolationAttribution.cs` + `TwoBodyParentFeedforwardIsolationReport.cs`：Parent FF source-isolation已关闭；OFF使rotation/position/forceMoment进一步恶化，Parent FF应保持ON，v0.3.14转向native CharacterJoint COM→anchor lever isolation。
- v0.3.12 `TwoBodyForceMomentCancellationAttribution.cs` + `TwoBodyForceMomentCancellationReport.cs`：physical cancellation A/B已STRONG PASS并关闭；证明`r×currentForce`为强causal contributor，但cancellation不是production方案，v0.3.13转向Parent-FF上游source isolation。
- v0.3.11 `TwoBodyJointSolverCouplingAttribution.cs` + `TwoBodyJointSolverCouplingReport.cs`：solver reaction attribution已完成；force-moment material contributor已获得实机证据，后续已由v0.3.12 physical cancellation和v0.3.13 source isolation取代。
- v0.3.10 `KneeDebtFrameStallAttribution.cs` + `KneeDebtFrameStallReport.cs`：stall与Knee abort关系已关闭；只保留minimal safety ring，不再长期携带timing correlation。
- v0.3.9 `TwoBodyControllerResponseAttribution.cs` + `TwoBodyControllerResponseAttributionReport.cs`：基础P/F response问题已关闭；v0.3.11只恢复最小必要P/F replica思想并改为solver-reaction attribution，不恢复旧observer。
- v0.3.8之前的旧`KneeTetherSafetyContextRecorder.cs` + `KneeTetherSafetyContextReport.cs`历史实现：不恢复；当前主线使用重新精简的`KneeSafetyContext*`。

- v0.3.8 `TwoBodyMassPropertyFactorizationAttribution.cs` + `TwoBodyMassPropertyFactorizationReport.cs`：COM/inertia owner已关闭；inertia=primary owner，live三路replica已删除。

- v0.3.7 `TwoBodyMassPropertyRequalificationAttribution.cs` + `TwoBodyMassPropertyRequalificationReport.cs`：full explicit COM+inertia在冻结Two-Body controller下REJECTED；已由v0.3.8 factorization取代。
- v0.3.6 `SharedPelvisTransitionFeedforwardAttribution.cs` + `SharedPelvisTransitionFeedforwardReport.cs`：Transition acceleration-FF causal hypothesis已SUPPORTED/CLOSED；exact derivative尚未production-ready，runtime observer已删除。
- v0.3.5 `SharedPelvisDynamicAuthorityAttribution.cs` + `SharedPelvisDynamicAuthorityAttributionReport.cs`：inertia/source/reaction问题已关闭，L0与100% reaction B不再live运行。
- v0.3.4 `SharedPelvisDynamicReactionReceiverAuthorityIsolation.cs` + `SharedPelvisDynamicReactionReceiverReport.cs`：sandbox结构已验证，但reaction小penalty结论因default-like `(1,1,1)` inertia在v0.3.5被降级为legacy witness。
- v0.3.3 `BilateralReactionBudgetSelectivePathAttribution.cs` + `BilateralReactionBudgetReport.cs`：budget gain扫描已关闭；其中angular-acceleration绝对量同样受v0.3.5 inertia历史纠错约束。
- v0.3.2 `BilateralReactionPathCounterfactualAttribution.cs` + `BilateralReactionPathCounterfactualReport.cs`：被v0.3.3 selective budget取代，只保留closure。
- v0.3.1 `BilateralSharedPelvisCouplingAttribution.cs` + `BilateralLegCouplingReport.cs`：问题已关闭。
- v0.2.119–0.2.121 Knee continuous filter Pareto observer：first-order filter tuning已关闭。
- v0.2.115–0.2.118 raw-W/P-aligned raw-W系列physical/observer：问题已闭合，production回到filtered/held alpha=.50。
- v0.2.107–0.2.114 pose/W bridge、timestamp、phase-budget、target-W generation等closed-question observers：只保留immutable closure。
- v0.2.83/85/87/89/90/91、v0.2.94–0.2.101 heavy Hip parent/TRUE-shock/Governor observers：不再live运行。
- `ObserverTimingIntegrityDiagnostics.cs`：只保留“observer overhead可造成回归”的维护教训。

历史问题若因新证据重开，应从对应旧ZIP恢复**最小必要逻辑**，不要一次性复活全部历史observer。

## 4.3 Console / logging 当前策略

Console只保留当前setup/build/PCA/stack/trace/leg-drive/freeze/bilateral/current-experiment/report/reset/rollback入口。

当前实验关键入口（v0.3.17）：

- `arl_bilateral_status`
- `arl_twobody_dynamic_status`
- `arl_knee_safety_status`
- `arl_leg_freeze_status`

**不再兼容早期实验命令。** 已退休command不会redirect、不会保留同名wrapper，也不会为旧版本maintain比较逻辑；输入旧命令统一按unknown处理。若历史问题真正因新证据重开，从对应旧ZIP恢复最小必要模块，而不是在主干长期携带兼容层。

## 4.4 长期维护风险

- **Dynamic sandbox必须坚持单一physics pose domain。** kinematic reset/rebind时Rigidbody与Transform必须同步；native anchor、COM、lever、closure与safety geometry统一从Rigidbody pose计算，Transform只作独立诊断。dynamic toggle前后都要保留state-domain hard gate，禁止再次混用两个世界做preflight。
- **任何将来会变成dynamic并接收真实force/torque的Rigidbody，都必须显式定义并验证 mass、centerOfMass、inertiaTensor、inertiaTensorRotation；不能把Unity/BepInEx运行时default-like readback当作已验证物性。**
- Collider geometry与mass-property basis必须使用同一local frame/center。设置analytical tensor却遗漏COM同样属于foundation不完整，即便某一轮pure-torque实验暂时不敏感。
- 冻结production中发现可疑默认值时，先做read-only audit并获得实机证据；不要借“清BUG”名义同时改变已冻结控制和实验变量。
- Safety reason与summary必须来自同一sample或明确写采样差异。
- `AbortDrive`是安全语义，普通重构不能替换成一般disable。
- 删除observer后必须做type/method reference scan + control-flow orphan-guard audit。
- Read-only observer要避免每FixedUpdate排序、字符串拼接、LINQ、大对象分配；统计只做O(1) accumulation，重计算放status/finalize。
- Safety-censored与calculation-invalid不能混为一谈；前者代表真实系统失败，后者代表observer数据数学/有限性失败。
- **Kinematic/default Rigidbody inertia 不得直接当成 future dynamic mass-property foundation。** Collider关闭或未显式重建mass properties时，`inertiaTensor=(1,1,1)`之类的readback必须先按几何/质量验证；任何固定外力矩 reaction-CF 都对该tensor高度敏感。
- **性能/调度stall必须用时间轴证据判断。** 肉眼看到全部proxy瞬时脱离时，至少同时保存Render `unscaledDeltaTime`、FixedUpdate catch-up depth、FixedStep到上一LateUpdate的`realtimeSinceStartup` wall-gap、source age/held与多body error；不能仅凭一拍大误差就归因于CPU/GPU，也不能忽略机器卡顿对Render→Fixed sample bridge的真实影响。

---

# 5. 文档维护规则

本文件以后只保留：

1. 当前恢复入口与冻结决定。
2. 当前系统整体设计与物理边界。
3. 宏观路线与进入下一阶段的条件。
4. **0.1.x / 0.2.x 压缩历史章节**，只在有真正新历史纠错时修改，不再恢复逐版本流水账。
5. **0.3.x及以后逐版精简记录**：为什么做、相对上版改变什么、关键实机证据、最终决定。
6. 当前活动源码/退休模块与必须长期记住的维护风险。

不再保存：

- 旧版本命令执行步骤。
- 每版重复编译/静态QA流水账。
- 已被A/B或后续observer取代的中间猜测。
- 同一个阈值/参数在几十个版本里重复解释。
- 纯开发日记、行号级修改说明。

**压缩不等于丢掉因果。** 对历史主干必须保留“为什么当时要做、最终证据支持/否定什么、以后必须避开什么”。完整逐版细节由旧 `.md` 与对应版本 ZIP 备份承担。
