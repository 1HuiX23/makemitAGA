# MakemitAGA 碰撞体重建模块：现状总结、设计原则与未来开发大纲

> 文档性质：碰撞体重建模块的阶段性总设计说明  
> 涵盖项目：Spatial Reconstruction Lab（SRL）与 Geometry Source Probe（GSP）  
> 当前基础：GSP v0.2.0 固定被动碰撞体基础冻结版（保留对象无关几何读取、实例审计、静态非凸源网格代理、简单盒/复合盒代理、环境观测与完整回滚；删除全部小球、落球、水平截面和数据驱动通道计划验证代码）  
> 目标读者：后续继续维护 MakemitAGA 的开发者，以及需要理解项目整体方向的非计算机专业读者

---

## 一、这个模块最终要解决什么问题

MakemitAGA 后续希望让 Active Ragdoll 角色能够在 MiSide 的真实房间中：

- 站立、移动和保持平衡；
- 坐到床、椅子等物体上；
- 用手接触桌面、墙面、柜子；
- 拿起杯子、书本、遥控器、游戏手柄等小物体；
- 根据 VLM 给出的世界空间目标，把手腕、手掌或其他身体部位移动到正确位置；
- 让 ML-Agents 根据周围表面、距离、法线和接触状态做稳定控制。

要做到这些，游戏中的视觉模型不能只“看得见”，还必须转化为物理系统能够稳定使用的环境表示。

这里所说的“碰撞体重建”，并不只是简单地给每个物体添加一个 MeshCollider。它实际包含四项工作：

1. **找到真实几何数据**：从游戏运行时拿到顶点和三角形；
2. **理解几何属于什么**：实体、图片、玻璃、阴影、装饰还是多个物体的合集；
3. **生成适合物理的代理**：静态网格、盒子、胶囊、凸包或多个组合碰撞体；
4. **维护物理属性**：质量、摩擦、弹性、是否固定、是否可以拿取。

因此，这个模块最终不是一个单独的“扫描器”，而是一套：

```text
几何获取
→ 语义分类
→ 拓扑审计
→ 现实实例拆分
→ 碰撞代理生成
→ 物理参数配置
→ 接触验证
→ Agent 观测输出
```

的完整基础设施。

---

## 二、必须区分的四种“网格”

项目早期遇到的很多问题，都来自把不同用途的网格当成同一种东西。未来设计必须明确区分以下四层。

### 2.1 游戏原始渲染网格

这是游戏为了显示画面而制作的模型。

它可能包含：

- UV 接缝产生的重复顶点；
- 法线硬边产生的重复顶点；
- 材质拆分；
- 只做了一面的薄片；
- 没有背面的墙板；
- 透明玻璃；
- 图片画面；
- 阴影平面；
- 特效面；
- 静态合批后已经烘焙成世界坐标的几何；
- 一个 Unity 节点下分散在房间不同位置的多个现实物体。

它的目标是“看起来正确”，不是“物理上封闭”。

### 2.2 高保真观察几何

观察几何用于：

- 最近表面点查询；
- 表面法线；
- 到环境的距离；
- VLM 目标点有效性检查；
- ML-Agents 的局部环境观测；
- Active Ragdoll 的接触预判。

观察几何可以保留：

- 桌面凹槽；
- 床面细节；
- 物体外形；
- 有意义的显示屏或画面位置。

但应排除：

- 阴影平面；
- 发光特效；
- 没有真实空间意义的覆盖层；
- 错位或未通过世界坐标验证的部件。

观察几何不一定需要水密，也不一定直接参与 Unity 物理。

### 2.3 碰撞候选几何

碰撞候选几何只保留有实体意义的部分，例如：

- 相框主体；
- 数字钟外壳；
- 桌面和桌腿；
- 床体；
- 杯子的杯壁和底部；
- 椅子的座面、靠背和扶手。

它会进入：

- 空间焊接；
- 连通组件拆分；
- 现实物体实例聚类；
- 代理类型选择；
- 碰撞体生成前审计。

碰撞候选几何仍不等于最终碰撞体。

### 2.4 最终物理代理

最终物理代理是 Unity Physics 真正使用的表示，例如：

- 静态非凸 MeshCollider；
- BoxCollider；
- 多个 BoxCollider；
- CapsuleCollider；
- SphereCollider；
- 单个凸 MeshCollider；
- 多凸包组合；
- 为软物体准备的简化胶囊或关节组合。

最终代理必须优先考虑：

- 稳定；
- 低复杂度；
- 不产生尖刺；
- 不穿透；
- 不让角色被细小视觉缝隙卡住；
- 能够快速参与 Active Ragdoll 的大量接触计算。

最终物理代理可以比视觉网格简单得多。

---

## 三、我们已经做过的主要尝试

## 3.1 SRL 早期：十四相机扫描与体素融合

Spatial Reconstruction Lab 最早采用围绕目标物体布置多个虚拟相机，从多个方向取得深度信息，再进行体素或表面融合。

主要优点：

- 不依赖原始 Mesh 是否可读；
- 理论上能从视觉深度恢复物体外形；
- 能生成新的独立网格；
- 可以对完全未知对象使用同一套扫描入口。

实际遇到的问题：

- 桌腿、椅子轮、把手等细小结构容易缺失；
- 被其他表面遮挡的区域无法可靠恢复；
- 开放边界难以稳定封闭；
- 网格出现阶梯状；
- 直角被平滑成圆角；
- 薄面容易消失；
- 相互接近的部件容易粘连；
- 对复杂椅子、桌子等物体，需要大量特殊修补逻辑；
- 顶点数可能快速增长并触发 Unity Mesh 限制。

这一路线证明了“仅依靠有限视角重建所有真实场景物体”成本非常高。

## 3.2 SRL：切片扫描

为了解决遮挡区域和小物体，项目加入了方向切片扫描，并逐步发展为 X、Y、Z 三轴切片融合。

取得的进展：

- 桌面凹槽比十四相机结果更清晰；
- 桌腿逐步从完全缺失变为可以识别；
- 对小而复杂物体保留了纯切片接口；
- 切片主导路线在部分真实物体上比十四相机更稳定。

暴露的问题：

- 桌腿与主体之间出现缝隙；
- 合并后产生尖刺；
- 桌腿顶部可能高于桌面；
- 三轴中只有一轴有数据时，薄片很难判断；
- 大规模体素和表面合并仍会产生高顶点数；
- 修复规则容易逐物体堆积，维护成本高。

## 3.3 SRL：十四相机与切片混合

项目曾使用十四相机负责平滑主体，切片负责无覆盖区域。

实际经验：

- 对规则柜子、床、箱子等物体可以得到不错结果；
- 对转椅和复杂桌子，混合路径仍会产生粘连、补平、尖刺和粗糙轮廓；
- 两种来源的数据质量和尺度不同，融合边界容易出问题；
- 修复一个物体的参数不一定适用于其他物体。

因此后期方向转为：

> 切片作为扫描重建的主要方案；十四相机更多承担诊断或补充，而不再假设它能独立解决所有物体。

## 3.4 SRL：严格拓扑门槛

项目建立了较严格的最终网格门槛：

```text
boundary = 0
nonManifold = 0
orientationConflict = 0
components = 1
watertight = True
colliderSafe = True
```

这个门槛的价值在于防止“肉眼看起来封住了”却存在严重拓扑问题。

同时也暴露了一个重要事实：

- “水密”不等于“视觉正确”；
- “通过拓扑门槛”不等于“适合动态碰撞”；
- 一个被强行封闭的桌子可能失去凹槽；
- 一个被平滑封闭的椅子可能把扶手下方填满；
- 一个水密网格仍可能有尖刺或不合理厚度。

因此未来验收不能只看一个 `watertight=True`。

## 3.5 SRL：代理碰撞体与动态接触测试

项目尝试过切片代理碰撞体，以及半透明刚体与默认碰撞体之间的受控接触比较。

目标是验证：

- 网格是否真的能阻挡刚体；
- 接触是否稳定；
- 代理是否比高细节结果更少尖刺；
- 默认碰撞体与 SRL 代理之间有什么差异。

早期动态接触测试出现过：

- 协程和 IL2CPP API 编译问题；
- PhysicMaterial 构造差异；
- 卡顿；
- 全部未通过；
- 测试输出难以维护。

这推动了后续 GSP 中更小、更可控的逐步验证。

---

## 四、GSP 路线：直接取得游戏原始几何

SRL 的困难促使项目建立独立测试项目 Geometry Source Probe。

GSP 的核心思想是：

> 如果游戏 GPU 已经拥有正确的渲染几何，就优先安全地把这份几何读取出来，而不是重新从深度图猜测一次。

## 4.1 安全边界的形成

早期测试曾尝试修改：

```text
mesh.vertexBufferTarget
mesh.indexBufferTarget
```

这在 MiSide 的 IL2CPP 运行环境中触发了原生崩溃。

由此确立了永久安全规则：

- 不再写入这两个 target；
- 不修改源 Mesh；
- 不释放游戏拥有的原始 COM buffer；
- 只释放 GSP 自己取得或创建的 COM 引用；
- 所有 GPU 路线都采用只读、可回滚、逐步扩大范围的测试方式。

## 4.2 CPU、MeshData 和异步读回尝试

GSP 先调查了：

- MeshFilter CPU 路径；
- SkinnedMeshRenderer.BakeMesh；
- 现有 MeshCollider；
- MeshData interop；
- GraphicsBuffer.GetData；
- AsyncGPUReadback；
- IL2CPP icall 和 MethodInfo。

实际结果：

- 大量静态合批 Mesh 为 `isReadable=false`；
- MeshData 在当前 interop 中缺少可用成员；
- AsyncGPUReadback 路线不可用或被裁剪；
- GraphicsBuffer 内部读取曾得到全零；
- 需要更底层但仍只读的 D3D11 staging 路线。

## 4.3 D3D11 staging 只读路线

GSP 后续逐步验证：

1. 取得 GPU 原生 buffer 指针；
2. 读取 D3D11 buffer 描述符；
3. 创建 GSP 自己的 staging buffer；
4. 只读回三个索引；
5. 读回索引对应的三个顶点；
6. 扩展到完整 routed SubMesh；
7. 验证位置、索引、面积、bounds 和哈希；
8. 保证所有 target 和资源所有权保持不变。

以 Bed 为代表的成功结果证明：

- 静态合批 Position 已经是世界坐标；
- 指定 routed SubMesh 可以完整读取；
- 读取后的 bounds 与 Renderer bounds 一致；
- 没有越界索引、重复索引三角形或零面积三角形；
- staging 和 COM 清理正常。

这成为当前精确几何获取的基础。

## 4.4 独立 Mesh 预览

GSP v0.1.17 将读回几何创建为独立、可删除、默认无碰撞的 Unity Mesh 预览。

验证了：

- 读取结果肉眼与原物体重合；
- 原物体可以隐藏和恢复；
- 临时对象能在场景切换和退出时清理。

同时，这类“创建 Unity Mesh 副本”的路径也是当前 65000 顶点错误需要重点审计的区域。未来不能再默认把大型聚合结果做成一个单独 Mesh。

## 4.5 静态非凸 MeshCollider

GSP v0.1.18 在提取副本上建立静态非凸 MeshCollider，确认：

```text
created = True
enabled = True
sharedMesh = True
sameMesh = True
convex = False
trigger = False
rigidbody = False
bounds match = True
```

这证明：

> 一个不一定数学水密的渲染表面，仍然可以作为有效静态碰撞表面。

这是未来设计中的重要分界：静态环境并不都需要被修成封闭实体。

## 4.6 真实刚体球接触

GSP v0.1.19～0.1.21 先后加入：

- 自然落球；
- 从相机方向发射测试球；
- 最多保留三个球并回收最旧对象；
- 临时禁用源碰撞体，避免误撞；
- IL2CPP 注册的真实 `OnCollisionEnter/Stay/Exit`；
- 接触冲量；
- Unity 原始 ContactPoint.normal 审计；
- 原始法线为零时，从最近三角形推导法线；
- 延迟自动汇总；
- ClosestPoint 诊断降噪；
- 测试结束后恢复原 Collider。

真实日志证明测试球确实撞到了 GSP 提取的 MeshCollider。

也发现 Unity 在该环境中有时返回零 ContactPoint.normal，因此未来观察接口不能盲信单一 Unity 字段，需要保留：

- 原始接触法线；
- 推导法线；
- 法线来源；
- 冲量是否有效；
- 接触新鲜度。

## 4.7 观察基础 DTO

GSP v0.1.22 建立了不直接依赖 ML-Agents 程序集的环境观察基础，包括：

- 世界 bounds；
- 顶点、索引、三角形数量；
- 几何和碰撞准备状态；
- 世界点归一化；
- 最近三角形法线查询；
- 当前或历史接触摘要；
- 未来消费者：Active Ragdoll、ML-Agents、VLM。

这使碰撞体重建不再只是 Unity Collider 创建模块，而成为 Agent 环境感知层的一部分。

---

## 五、从单物体读取到整屋普查

## 5.1 四房间几何普查

第一章四房间普查沿着：

```text
World/House/HouseGame Tamagotchi/House
```

扫描：

```text
Bedroom
Kitchen
Main
Toilet
```

共处理 270 个房间直接物体根节点。

其中符合单 Renderer、单路由读取条件的 186 个对象全部读回成功。

这说明 D3D11 精确读取不只适用于 Bed。

同时普查发现：

- 大量对象是多 Renderer 或多 SubMesh；
- 很多 Unity 根节点不等于一个现实物体；
- 部分节点只有层级或语义信息，没有网格；
- 可拿取对象不能只靠名称关键词判断；
- 原始索引拓扑会把 UV 接缝和硬边误算成大量开放边界。

## 5.2 空间焊接审计

游戏模型常在同一个空间位置保存多个顶点索引。

所以未来拓扑判断采用两套结果：

```text
Raw topology
Welded audit topology
```

空间焊接只作用于审计副本，不修改原渲染网格。

DigitalClock 的例子：

```text
原始：328 顶点，43 个组件，310 条边界
焊接：150 顶点，3 个组件，34 条边界
```

这证明绝大多数原始断裂只是渲染接缝。

但焊接后仍有开放边界，说明它仍不适合作为未经处理的动态实体 MeshCollider。

---

## 六、多部件聚合设计

## 6.1 为什么不能把所有 Renderer 直接合并

一个相框节点可能包含：

- 相框实体；
- 图片；
- 玻璃或高光；
- 阴影特效。

一个数字钟可能包含：

- 外壳；
- 屏幕覆盖面；
- 发光或闪烁平面。

如果全部合并：

- 阴影会变成假墙；
- 图片可能变成额外薄碰撞板；
- Agent 最近表面查询会命中无实体意义的特效；
- 水密审计会被大量薄片污染。

## 6.2 当前部件角色

现有中间设计为每条 Renderer/SubMesh 路由分配角色：

```text
PHYSICAL_CORE
PHYSICAL_DETAIL
VISUAL_ONLY
SHADOW_EFFECT_EXCLUDE
VISUAL_EFFECT_EXCLUDE
CHILD_OBJECT_EXCLUDE
UNKNOWN_REVIEW
SKINNED_DEFER
```

### PHYSICAL_CORE

物体主体，可进入观察和碰撞候选。

### PHYSICAL_DETAIL

有物理意义但不是主要体积的部件，例如把手、按钮或支架。

### VISUAL_ONLY

对视觉观察有价值，但不应参与碰撞，例如显示画面或透明覆盖面。

### SHADOW_EFFECT_EXCLUDE

已确认的阴影特效，读取前排除。

### VISUAL_EFFECT_EXCLUDE

其他纯特效面，读取前排除。

### CHILD_OBJECT_EXCLUDE

确认是独立现实物体、但在 Unity 层级中挂在当前父对象下面的子物体。

它会：

- 在读取前从父对象观察与碰撞聚合中排除；
- 不污染父对象的静态碰撞代理；
- 保留为未来可独立扫描、配置或改造成动态可拿物体的候选。

`Bedroom TumbonRoll` 下的 `Bag` 与 `SchoolBag` 已验证采用该角色。

### UNKNOWN_REVIEW

自动规则无法可靠判断，需要白名单或 Unity Explorer 人工确认。

### SKINNED_DEFER

会随骨骼变形的网格，不能直接按静态路线处理。

## 6.3 双聚合通道

同一对象分别生成：

```text
Observation aggregate
Collision candidate aggregate
```

观察通道可包含视觉表面；碰撞通道只包含实体。

这两套结果共享来源追踪，但用途不同。

---

## 七、世界坐标统一

当前场景中存在两类 Position：

```text
静态合批路由：Position 已经是世界坐标
普通 Renderer：Position 仍是局部坐标
```

如果错误地对静态合批顶点再次 Transform，会把物体移动到错误位置；如果不转换普通局部网格，它会停留在原点附近。

现有流程对每条路由同时检查：

```text
raw positions
localToWorld transformed positions
```

然后与 Renderer 世界 bounds 对比。

只有一种候选明确匹配时才允许进入聚合：

```text
raw/world-baked
renderer-local-to-world
```

两种都不可信的路由会被拒绝，不能污染环境观察。

DigitalClock 已经验证：

- 主体是 raw/world-baked；
- Blink 是 localToWorld；
- 两者合并后的观察 bounds 与目标 Renderer bounds 一致。

---

## 八、现实空间实例拆分

## 8.1 Unity 根节点不等于现实物体

`BedroomPictures 1` 是最明确的案例。

Unity 层级上它是一个对象根节点，但实际包含三个分散在卧室不同墙面的相框。

因此白名单不能只建立在 GameObject 根节点级别。

## 8.2 当前实例拆分步骤

```text
世界坐标聚合三角形
→ 空间焊接
→ 按共享边拆连通组件
→ 按 AABB 距离与尺寸自适应归并
→ 生成现实空间实例候选
```

当前结果已经验证：

```text
DigitalClock：3 个实体组件 → 1 个现实数字钟
BedroomPictures 1：5 个主体组件 → 3 个现实相框
```

这两个案例分别证明：

- 互不连通但空间贴近的部件可以属于同一个物体；
- 同一 Unity 路由中相距很远的几何应拆成多个物体。

## 8.3 稳定 instanceKey

每个空间实例会生成草案标识：

```text
SI-COLLISION-...
SI-OBSERVATION-...
```

它综合：

- 目标路径；
- 通道；
- 量化后的世界 bounds；
- 来源路由。

目前在同一场景和相同几何条件下，两次扫描的相框 instanceKey 保持稳定，已经可以用于实例级配置。

未来仍需考虑：

- 场景更新；
- 物体被移动；
- 动态对象位置变化；
- 游戏版本改变模型；
- 浮点误差导致标识变化。

因此正式系统可能需要“持久语义 ID + 几何指纹 + 当前 Transform”三层标识。

---

## 九、白名单与物理配置

## 9.1 为什么白名单是必要的

纯自动算法无法可靠知道：

- Blink 是屏幕还是阴影；
- 杯口应该保留还是补死；
- 一个物体是否允许拿起；
- 相框是否固定在墙上；
- 枕头应该是硬体还是软体近似；
- 某个小组件是按钮还是垃圾碎片。

因此白名单不是临时补丁，而是未来模块的正式组成部分。

## 9.2 部件级白名单

当前配置文件：

```text
MakemitAGA.GeometrySourceProbe.ObjectPhysicsWhitelist.txt
```

负责：

- 部件角色；
- 是否进入观察；
- 是否进入碰撞候选；
- 对象级质量、摩擦和材质元数据。

## 9.3 实例级白名单

当前配置文件：

```text
MakemitAGA.GeometrySourceProbe.SpatialInstancePhysicsWhitelist.txt
```

按照：

```text
对象路径 + collision instanceKey
```

分别维护：

```text
enabled
proxyType
fixedToWorld
pickable
massKg
staticFriction
dynamicFriction
bounciness
materialTag
minimumThicknessMeters
notes
```

三个相框已经分别匹配三条配置。

## 9.4 未来正式配置需要增加的字段

建议后续增加：

```text
semanticId
sceneId
objectCategory
proxyGenerationMode
proxyQualityLevel
centerOfMassMode
centerOfMassOverride
inertiaMode
linearDrag
angularDrag
collisionLayer
contactMaterial
breakable
attachable
holdPoseHints
graspPoints
interactionTags
observationPriority
manualVerificationState
sourceGameVersion
geometryFingerprint
```

---

## 十、当前碰撞代理策略

未来不能用一种碰撞体处理所有物体，应按类别选择。

| 物体类型 | 推荐代理 | 是否要求水密 | 示例 |
|---|---|---:|---|
| 大型固定表面 | 静态非凸 MeshCollider 或分块表面 | 否 | 地板、墙、床面 |
| 规则固定家具 | Box/多个 Box | 否 | 柜子、桌子主体 |
| 薄固定装饰 | 定向薄 Box | 否 | 相框、镜子 |
| 小型规则刚体 | Box、Sphere、Capsule 或单凸包 | 是/代理自身封闭 | 书、遥控器、球 |
| 凹形可拿物体 | 多凸包或组合基础碰撞体 | 每个代理封闭 | 杯子、茶壶、手柄 |
| 带把手容器 | 杯体代理 + 把手代理 | 不直接封杯口 | 杯子、桶、茶壶 |
| 多孔装饰 | 简化实体代理 | 否 | 篮子、藤编盒 |
| 软物体 | 胶囊/盒组合或未来软体近似 | 否 | 枕头、毛巾 |
| 动画角色部件 | 胶囊、盒、球和关节链 | 是 | Active Ragdoll 身体 |

---

## 十一、PCA 定向薄盒草案

针对相框等薄而旋转的物体，当前中间测试使用 PCA 估计三条主轴，再在主轴空间计算紧贴几何的盒子。

输出包括：

```text
center
rawSize
inflated size
euler
rawVolume
inflatedVolume
AABB volume
ratioToAabb
thicknessInflation
```

三个相框测试中：

- 都识别出约 1.5～2.0 厘米的真实薄轴；
- 加入少量接触余量后约为 1.85～2.29 厘米；
- 第三个斜放相框的定向盒体积远小于世界 AABB，证明主轴识别有效。

目前它只是数学草案，还没有创建 BoxCollider。

正式应用前仍需解决：

- PCA 轴符号和轴顺序稳定；
- 接近正方形时两个长轴可能交换；
- 完全对称物体的旋转不唯一；
- 最小厚度不能吞掉真实小结构；
- 固定物体和可拿物体需要不同接触余量；
- 代理是否穿入墙面；
- 代理中心是否需要沿法线偏移。

---

## 十二、65000 顶点错误与大型网格策略

项目曾多次遇到：

```text
ArgumentException: Mesh can not have more than 65000 vertices
```

### 12.1 已确认的主要误判来源

相框阶段的日志最终证明，最近一次 65000 错误并不是碰撞几何本身超限，而是把约数万字符的完整报告一次性输出到游戏控制台后，Unity UI 文本生成了超过 65000 顶点的文字 Mesh。

因此永久规则是：

```text
完整报告写入 BepInEx/config 文件
游戏内控制台只输出短摘要
PrintGame 保留统一字符预算
超预算内容只进入 BepInEx 日志和磁盘报告
```

后续看到同一异常时，必须先区分：

```text
几何 Mesh 顶点/索引超限
还是
UI 文字 Mesh 超限
```

不得再次把数万字符报告直接送进游戏内控制台。

### 12.2 几何 Mesh 仍然需要分块安全门

UI 根因被确认，不代表大型几何可以无限合并。未来架构仍不能依赖“整屋单 Mesh”或“多个现实物体共用一个动态 Mesh”。

建议硬规则：

- 不创建整屋单 Mesh；
- 不把多个现实物体合成一个动态 Mesh；
- 预览与碰撞代理默认按现实实例或空间块创建；
- 可能超过 65535 顶点时，在写入顶点前明确选择 `IndexFormat.UInt32`；
- 静态非凸 MeshCollider 按对象实例创建；
- 动态物体优先使用凸包、多个凸包或基础 Collider；
- 每次 Unity Mesh 创建记录用途、对象、顶点、索引、indexFormat 和所有权；
- 所有 Mesh、Collider、Material 与临时 GameObject 都必须支持完整回滚。

### 12.3 当前已验证的静态源网格规模

`Bedroom TumbonRoll` 的静态源网格代理为：

```text
vertices = 4304
indices = 10476
triangles = 3492
indexFormat = UInt16
```

它远低于 65535 顶点，并已通过 Mesh 创建、非凸 MeshCollider cooking、真实接触、空腔穿越与回滚验证。

---

## 十三、当前推荐的正式模块结构

未来可以把碰撞体重建拆成以下模块。

## 13.1 GeometrySourceAcquisition

职责：

- CPU 可读 Mesh；
- Skinned Bake；
- 现有 Collider 几何；
- D3D11 routed SubMesh staging 读回；
- 世界坐标模式判断；
- 源资源所有权审计。

输出：

```text
GeometryRouteSnapshot
```

包含：

- 来源对象；
- Renderer；
- route；
- 材质和 Shader；
- 顶点和索引；
- 坐标模式；
- 世界 bounds；
- 数据哈希；
- 读取安全状态。

## 13.2 GeometrySemanticFilter

职责：

- 自动部件角色；
- 部件白名单覆盖；
- 阴影和特效排除；
- 视觉/物理通道选择；
- Skinned 延迟处理。

输出：

```text
ResolvedGeometryPart
```

## 13.3 GeometryAudit

职责：

- 索引越界；
- 重复索引；
- 零面积三角形；
- 原始拓扑；
- 空间焊接拓扑；
- 边界环；
- 非流形边；
- 朝向冲突；
- 重复三角形；
- 组件和面积。

输出：

```text
TopologyAuditReport
```

## 13.4 SpatialInstanceResolver

职责：

- 连通组件拆分；
- AABB/OBB 距离；
- 自适应空间归并；
- 观察实例与碰撞实例关联；
- 稳定实例标识；
- 人工复核队列。

输出：

```text
SpatialObjectInstance
```

## 13.5 CollisionProxyPlanner

职责：

根据实例特征和白名单选择：

```text
STATIC_MESH_SURFACE
BOX
ORIENTED_THIN_BOX
BOX_COMPOUND
CAPSULE
SPHERE
CONVEX_HULL
CONVEX_DECOMPOSITION
CONTAINER_COMPOUND
SOFT_PROXY
IGNORE_COLLISION
MANUAL_REQUIRED
```

输出：

```text
CollisionProxyPlan
```

## 13.6 CollisionProxyBuilder

职责：

- 实际创建 Collider；
- 设置父对象和局部变换；
- 设置 PhysicMaterial；
- 动态对象创建 Rigidbody；
- 设置质量、重心和惯性；
- 保存所有权；
- 支持完整回滚。

这一层必须与 Planner 分离。Planner 可以先离线审计，Builder 才会真正修改 Unity 场景。

## 13.7 PhysicsProfileRegistry

职责：

- 读取部件级和实例级白名单；
- 参数校验；
- 版本迁移；
- 场景和游戏版本匹配；
- 默认材质模板；
- 人工标定记录。

## 13.8 CollisionValidationLab

职责：

- 静态落球；
- 视角射球；
- 水平射球；
- 数据驱动的固定被动碰撞验证计划；
- 非目标 Collider 隔离；
- 计划轴与指定对侧出口检查；
- 开口、空腔和通道穿越检查；
- 滑动摩擦测试；
- 倾斜平面测试；
- 旋转和翻滚稳定性；
- 穿透检测；
- 接触法线审计；
- 代理与原 Collider 对比；
- Active Ragdoll 手、脚、身体接触测试；
- 自动结果汇总与完整回滚。

v0.1.29 新增：

```text
PassiveColliderValidationPlanRegistry
PassiveColliderValidationPlan
PassiveColliderValidationStep
```

计划文件把以下内容从对象特化代码移到配置数据：

```text
目标对象匹配
步骤顺序
步骤名称
发射轴
高度模式
高度值
横向偏移
人工备注
```

## 13.9 EnvironmentObservationAdapter

职责：

把高保真观察几何和实时接触整理成：

- Agent 相对位置；
- 最近表面点；
- 法线；
- 有符号或无符号距离；
- 表面类别；
- 接触冲量；
- 接触对象 ID；
- 当前/历史接触；
- 代理类型；
- 是否可拿取；
- VLM 语义标签。

---

## 十四、建议的数据流

```text
Scene hierarchy
    ↓
对象与 Renderer 普查
    ↓
逐路由只读几何获取
    ↓
世界坐标规范化
    ↓
部件自动角色 + 白名单覆盖
    ↓
Observation aggregate / Collision candidate aggregate
    ↓
空间焊接审计副本
    ↓
连通组件拆分
    ↓
现实空间实例解析
    ↓
实例级白名单
    ↓
代理规划
    ↓
代理草案可视化与报告
    ↓
安全 Builder 创建 Collider/Rigidbody
    ↓
接触实验和严格验证
    ↓
写入长期注册表
    ↓
提供给 Active Ragdoll / ML-Agents / VLM
```

---

## 十五、碰撞代理的验证门槛

未来每个最终代理至少应通过以下五类检查。

## 15.1 结构检查

- 代理数量合理；
- 没有空代理；
- 没有 NaN/Infinity；
- 尺寸大于安全最小值；
- 不超过场景合理尺度；
- 没有超过 Mesh 顶点或索引限制。

## 15.2 几何覆盖检查

- 代理覆盖主要实体区域；
- 不包含阴影和特效；
- 不严重超出视觉物体；
- 不堵住杯口、椅子空隙等有意开口；
- 对已知空腔或通道，使用数据驱动计划从指定入口沿计划轴穿到指定对侧出口；
- 侧面绕出、外部 Collider 干扰、超时和目标误碰必须与真正通过分开记录；
- 不穿进相邻墙面或其他实例；
- 细节代理与主体代理无明显断层。

## 15.3 Unity 物理检查

- Collider 创建成功；
- sharedMesh 或基础参数正确；
- convex 状态符合静态/动态用途；
- 动态物体没有非凸 MeshCollider + Rigidbody 的非法组合；
- PhysicMaterial 应用正确；
- Rigidbody 质量、重心和惯性有效；
- Physics.SyncTransforms 后 bounds 正确。

## 15.4 动态接触检查

- 自然落球能稳定停留；
- 侧面射球能正确反弹或滑动；
- 不穿透；
- 不被微小尖刺异常弹飞；
- 接触法线方向合理；
- 冲量存在；
- 多次重复结果稳定；
- 清理后原 Collider 和 Renderer 全部恢复。

## 15.5 Agent 使用检查

- 最近表面查询与真实可接触面一致；
- 阴影不成为表面；
- 代理碰撞与观察几何关联正确；
- 接触对象 ID 稳定；
- 睡眠 Rigidbody 的有效速度应视为零；
- 历史接触和当前接触明确区分；
- 观察向量有明确单位、范围和新鲜度。

---

## 十六、静态环境与动态可拿物体必须分开

## 16.1 静态环境

例如：

- 地板；
- 墙；
- 床；
- 大型柜子；
- 固定桌面；
- 固定相框。

优先目标：

- 提供稳定接触表面；
- 不需要计算内部体积；
- 不需要 Rigidbody；
- 可以使用开放的静态表面 MeshCollider；
- 也可以用盒子或分块代理提高稳定性。

## 16.2 动态可拿物体

例如：

- 杯子；
- 茶壶；
- 书；
- 遥控器；
- 游戏手柄；
- 小黄鸭；
- 小盒子。

必须额外满足：

- 动态代理必须是凸或组合凸；
- 有合理体积；
- 有质量和惯性；
- 有抓取位置；
- 不使用复杂非凸视觉网格直接驱动 Rigidbody；
- 有明确的语义实例 ID；
- 能独立移动，而不是带动同根节点的其他现实物体。

`BedroomPictures 1` 的三个相框证明：创建 Rigidbody 前必须先拆分现实实例，否则一个节点下三个物体会一起移动。

---

## 十七、与 Active Ragdoll 的接口

碰撞体重建模块将为 Active Ragdoll 提供两种信息。

### 17.1 Unity Physics 接触

由实际 Collider 负责：

- 脚与地面；
- 身体与床；
- 手与桌面；
- 手指与可拿物体；
- 物体与环境。

### 17.2 几何预观察

在真正碰撞之前提供：

- 手腕到目标表面的距离；
- 手掌前方的法线；
- 脚底下方地面高度；
- 膝盖附近障碍物；
- 目标物体中心、朝向和尺寸；
- 是否可拿取；
- 推荐抓取点。

这样 ML-Agents 不需要只靠碰撞发生之后才知道环境结构。

---

## 十八、与 VLM 的接口

VLM 不直接控制 Unity Collider，但可以提供：

```text
语义任务
目标对象
目标实例
目标身体部位
期望世界空间点
期望朝向
交互类型
```

例如：

```text
把右手移动到杯子把手
```

碰撞体重建模块需要把“杯子”解析为：

- 正确场景实例；
- 观察几何；
- 碰撞代理；
- 把手部件；
- 抓取点；
- 质量和摩擦；
- 当前是否被其他物体遮挡。

因此未来白名单也可以逐步变成“物理 + 语义注册表”。

---

## 十九、未来开发阶段建议

## 阶段 A：固定被动碰撞主线收尾

当前状态：

- v0.1.23：相框定向盒真实接触；
- v0.1.24：独立子物体排除、复合盒审计草案；
- v0.1.25：七个复合盒实际创建，但视觉拟合被否决；
- v0.1.26：静态非凸源网格被动碰撞体；
- v0.1.27：空格射球真实接触、小球尺寸、历史统计和 bounds 快照；
- v0.1.28：水平/自动通道、非目标 Collider 隔离、严格对侧出口，两个层间空腔 2/2 通过；
- v0.1.29：把验证迁移为 `PassiveColliderValidationPlan` 数据驱动执行器。

v0.1.29 通过条件：

- 项目在 MiSide interop 环境编译成功；
- 运行时计划注册表自动创建并读取；
- `BedroomTumbonRoll.LayerGaps` 匹配成功；
- 两个步骤仍为严格 PASS；
- 日志出现 `validationPlanStrictSuccess=True`；
- 不再由 `StaticSourceMeshBallTest` 直接读取 `TRAY_BOX`。

## 阶段 B：v0.1.30 特化测试代码退场

目标：

- 删除 v0.1.29 中一次性的 `Bedroom TumbonRoll` 计划迁移适配器；
- 删除测试执行器中的固定“下—中层/中—上层”假设和对象名称判断；
- 逐步移除仅为旧报告兼容保留的 `LOWER_TO_MIDDLE`、`MIDDLE_TO_UPPER` 专名字段；
- 保留已写入 config 的验证计划数据；
- 保留全部通用改进。

必须保留：

```text
静态非凸源网格代理
CHILD_OBJECT_EXCLUDE
非目标 Collider 隔离
严格计划轴与指定对侧出口
任意数量步骤的顺序执行
结果与生命周期分离
固定 bounds 快照
碰撞对象路径诊断
状态恢复与完整回滚
```

## 阶段 C：更多固定物体的计划化验收

选择代表对象：

- 床：上表面承托和床下开放区；
- 桌子：桌面承托、桌腿之间通道、桌下空腔；
- 柜子：外壳阻挡与门洞/凹槽；
- 相框：正面、侧面、边缘；
- 房间墙面与地板：大范围静态接触。

验证计划应优先由配置、空间实例与人工确认产生，而不是继续增加对象名称分支。

## 阶段 D：通用代理选择器

目标：

- 规则固定家具：Box 或 BoxCompound；
- 复杂固定家具：STATIC_NONCONVEX_MESH；
- 薄固定装饰：ORIENTED_THIN_BOX；
- 小型规则刚体：Box、Sphere、Capsule 或单凸包；
- 凹形可拿物体：多凸包或组合基础碰撞体；
- 中空容器：保留开口的组合代理；
- 软物体：简化盒、胶囊或未来软体近似。

## 阶段 E：动态可拿物体

代表对象：

- Gamepad；
- BathroomSinkCup；
- Kitchen Teapot；
- BathDuck；
- 书、遥控器、小盒子；
- Bag 与 SchoolBag。

目标：

- 独立现实实例；
- 封闭凸代理或多个凸代理；
- 质量、重心、惯性和摩擦；
- 抓取点与交互标签；
- Rigidbody 合法组合；
- 动态接触、翻滚、落地和抓取验证。

## 阶段 F：全房间注册表

目标：

- 从四房间普查生成实例待办清单；
- 自动建议代理类型；
- 白名单记录人工语义与物理参数；
- 验证计划记录关键实体面和有意开口；
- 支持场景版本、几何指纹与人工验收状态；
- 输出可复核的房间级物理注册表。

## 阶段 G：Active Ragdoll、ML-Agents 与 VLM 接入

目标：

- Agent 局部环境观察；
- 脚底、手掌和身体附近的最近表面查询；
- 接触对象语义 ID；
- 当前/历史接触与新鲜度；
- 可拿物体状态；
- VLM 世界目标与空间实例注册表连接；
- 代理验证状态进入训练和运行时安全门。

---

## 二十、当前仍未解决的问题

1. v0.1.29 数据驱动验证计划尚未在用户的 MiSide interop 环境完成编译与运行确认；
2. v0.1.30 尚未删除 `Bedroom TumbonRoll` 一次性计划迁移适配器和旧专名兼容字段；
3. 验证计划目前依赖人工或已验证样本提供步骤，尚未实现从几何自动提出可靠通道；
4. 计划目前主要描述水平 X/Z 穿越，任意三维方向、曲线路径和多段路径尚未支持；
5. SkinnedMeshRenderer 的批量读取和当前姿势烘焙尚未统一；
6. 实例聚类对大型、细长、互相接近物体仍需更多测试；
7. instanceKey 在动态移动和游戏版本变化下的稳定性尚未验证；
8. PCA 对高度对称物体的轴顺序仍可能不稳定；
9. 通用凸包与多凸包生成路线尚未正式实现；
10. 杯口、篮孔、把手孔等有意开口的自动识别尚未实现；
11. 软物体代理尚未设计完成；
12. 质量、摩擦、重心和惯性尚未经过系统标定；
13. 观察几何的空间查询加速结构尚未建立；
14. 当前接触与历史接触的新鲜度语义仍需完善；
15. 全房间代理与游戏原 Collider 的长期冲突管理尚未完成；
16. 动态代理的存档、场景重载、配置迁移和自动回滚仍需完整测试。

---

## 二十一、长期维护原则

### 原则 1：永远保留来源

每个代理必须能追溯到：

```text
场景
对象路径
Renderer
SubMesh route
空间实例
白名单规则
生成算法
```

### 原则 2：自动结果不是语义真相

自动分类和聚类是候选，Unity Explorer、VLM 或人工白名单可以覆盖。

### 原则 3：观察和碰撞永远分离

不能因为一个表面对视觉有用，就默认让它参与物理。

### 原则 4：静态与动态策略分离

静态开放表面可以有效；动态物体必须使用稳定的封闭凸代理或组合代理。

### 原则 5：不为水密而水密

有意开口、物体空腔和真实功能结构不能为了通过门槛而被封死。

### 原则 6：优先简单稳定的代理

高细节视觉网格不是最好的实时碰撞体。

### 原则 7：所有 Unity 场景修改都必须可清理

每个临时 Mesh、Collider、Rigidbody、Material 和 GameObject 都必须有明确所有权和回滚路径。

### 原则 8：每次扩大范围都先做小样本测试

先一个索引，再一个三角形，再一个 SubMesh，再一个对象，再一个房间，不直接跳到整屋创建。

### 原则 9：错误必须带阶段和上下文

所有后续 Mesh 创建、Collider cooking 和物理失败都应记录：

```text
对象
实例
代理类型
顶点数
indexFormat
当前阶段
最近命令
stackTrace
```

### 原则 10：文档与白名单是系统的一部分

人工确认结果不能只留在聊天或截图里，应进入可版本管理的配置和说明文档。

---

## 二十二、当前阶段的总体结论

项目已经从最初的“重新扫描并猜测整个物体表面”，发展到：

```text
从 GPU 精确读取原始路由几何
→ 区分世界坐标和局部坐标
→ 排除阴影、特效和独立子物体
→ 空间焊接消除渲染接缝假边界
→ 拆分连通组件
→ 从 Unity 管理节点恢复现实空间实例
→ 选择定向盒、复合盒或静态非凸源网格
→ 实际创建可回滚 Collider
→ 用真实碰撞回调验证实体接触
→ 用隔离式严格通道验证有意空腔
→ 把验证步骤迁移为数据驱动计划
```

`Bedroom TumbonRoll` 已经提供了一个完整的固定复杂家具案例：

```text
七个粗略复合盒：运行机制通过，但几何拟合被视觉测试否决
静态非凸源网格：创建、bounds、cooking、启停与回滚通过
实体表面：真实 OnCollisionEnter 通过
下层—中层空腔：严格隔离 PASS
中层—上层空腔：严格隔离 PASS
外部 Collider 干扰：0
状态恢复：通过
```

这证明未来核心不应是：

> 把每个模型强行修成一个水密 MeshCollider。

而应是：

> 对每个现实物体保留高保真的观察表面，根据静态/动态用途选择稳定代理，并用可配置、可隔离、可回滚的验证计划同时检查实体面和有意开口。

GSP v0.1.29 的重点不是继续增加收纳架特判，而是把已经验证有效的实验机制变成通用基础设施；v0.1.30 再删除最后的一次性迁移代码。

---

## 二十三、GSP v0.1.23 正式主线更新

### 23.1 为什么从 v0.1.22 继续迭代

v0.1.22 之后的中间构建已经验证了完整的主线闭环：

```text
原始路由几何读取
→ 语义排除
→ 空间实例拆分
→ 定向盒草案
→ 静态 BoxCollider 实际创建
→ 可视化与碰撞启停
→ 完整回滚
```

这些能力已经直接服务于碰撞体重建，不再只是探索性诊断。因此 v0.1.23 恢复正式版本迭代。

### 23.2 受控接触验证成为代理验收门

v0.1.23 为每个实例代理安排三项真实物理测试：

```text
FRONT_CENTER：正面中心
SIDE_CENTER：侧面中心
FRONT_EDGE：正面边缘
```

每次只创建一个无重力小球和一个临时 Rigidbody，并忽略当前目标代理之外的 Collider。通过条件必须是：

```text
测试球收到 OnCollisionEnter
且 collision.collider 与目标 BoxCollider 完全相同
```

这样可以验证：

- 代理位置是否真实可碰撞；
- 旋转是否正确；
- 薄轴是否有效；
- 正面、侧面与边缘是否都能进入 Unity 物理解算；
- Collider 启停和恢复是否存在残留。

### 23.3 静态粗代理与高细节代理正式分层

第三个相框包含悬挂绳。PCA 定向盒把相框和绳子整体包入一个盒子，产生少量空白区域的保守碰撞。

当前正式定义为：

```text
STATIC_COARSE_PROXY
```

适用于：

- 固定在墙上的相框；
- 不可拿取的装饰物；
- 对角色只需要稳定阻挡，而不需要手指级精确接触的物体。

未来只有在物体需要拿取、穿过空洞、抓住把手或执行精细操作时，才升级为：

```text
INTERACTIVE_DETAIL_PROXY
```

并使用组件拆分、组合盒、凸包或多凸包。

### 23.4 项目瘦身后的模块边界

v0.1.23 删除了已经被完整 D3D11 staging 路线取代的阶段性实验：

- AsyncGPUReadback 桥和 icall 枚举；
- 单元素 GraphicsBuffer 读取；
- 首三角形异步读取；
- 原生指针、D3D11 描述符和首三角形 staging 边界命令；
- 专用于定位游戏控制台 65000 文字 Mesh 问题的错误捕获模块；
- 硬编码的临时样本规则安装器。

仍保留：

- 完整 D3D11 子网格读取；
- Mesh 预览与静态 MeshCollider 审计；
- 真实球碰撞回调；
- 中立环境观察 DTO；
- 四房间几何普查；
- 多部件、空间实例、白名单与代理运行时。

这使项目从“版本历史实验集合”进一步转变为“可长期维护的碰撞重建模块”。

### 23.5 65000 顶点问题的最终处理

专用测试逻辑已删除，但源码保留长期避坑注释和两个永久安全门：

```text
完整报告写入磁盘文件
游戏内 PrintGame 只允许有限长度短摘要
```

以后出现“Mesh can not have more than 65000 vertices”时，必须先区分：

```text
几何 Mesh 超限
还是
UI 文字 Mesh 超限
```

不得再次为了诊断而把数万字符报告直接输出到游戏内控制台。

### 23.6 v0.1.23 之后的设计主线

```text
固定物体定向盒接触验收
→ 房间级代理注册表
→ 组合盒与通用代理选择器
→ 可拿取物体凸代理
→ 质量、摩擦和 Rigidbody 参数应用
→ Active Ragdoll / ML-Agents 环境接触接口
```

从这一版本开始，新增功能应优先回答三个问题：

1. 这个代理是否稳定地代表现实物体？
2. 它是否通过真实接触和回滚测试？
3. 它是否能为 Agent 提供一致的空间与物理语义？

---

## 二十四、GSP v0.1.24：独立子物体排除与复合盒草案

### 24.1 `CHILD_OBJECT_EXCLUDE`

`Bedroom TumbonRoll` 的 Unity 根节点下包含：

```text
Bedroom TumbonRoll
├─ Bag
└─ SchoolBag
```

实际语义上，两个包是独立现实物体，不应被焊进收纳架主体。

v0.1.24 增加：

```text
CHILD_OBJECT_EXCLUDE
```

并验证：

```text
收纳架主体：PHYSICAL_CORE
Bag：CHILD_OBJECT_EXCLUDE
SchoolBag：CHILD_OBJECT_EXCLUDE
```

两个包在 GPU readback 前就被排除，因此不会污染父对象观察几何、碰撞几何和 bounds。

### 24.2 七部件复合盒审计草案

收纳架被暂时描述为：

```text
3 × TRAY_BOX
4 × FRAME_VERTICAL_RAIL
```

该版本只输出数学草案，不创建 Unity Collider。

### 24.3 日志瘦身

游戏内只输出关键组件；完整实例报告写入：

```text
BepInEx/config/MakemitAGA.GeometrySourceProbe.MultipartInstanceReport.txt
```

---

## 二十五、GSP v0.1.25：复合盒实际创建与视觉否决

v0.1.25 将七个草案实际创建为：

```text
7 × BoxCollider
7 × 半透明盒预览
Rigidbody = 0
Collider 默认关闭
支持显示、启停和回滚
```

运行机制全部通过，但截图显示：

- 三个托盘盒填满了真实托盘空间；
- 四根黄色立柱位于世界 AABB 角落；
- 立柱与真实白色管架明显错位；
- 空间覆盖过于保守。

因此结论是：

> 复合盒运行时框架可保留为粗略回退，但不能作为该复杂固定家具的默认高保真碰撞表示。

---

## 二十六、GSP v0.1.26：静态非凸源网格被动碰撞体

v0.1.26 新增：

```text
STATIC_NONCONVEX_MESH
```

它从最近一次成功碰撞聚合快照创建 GSP 自己拥有的 Mesh：

```text
vertices = 4304
indices = 10476
triangles = 3492
convex = False
isTrigger = False
Rigidbody = 0
Collider 默认关闭
```

永久规则：

- 不把源 Renderer Mesh 直接交给新 Collider；
- 用托管顶点与索引创建 GSP-owned Mesh；
- 世界坐标几何避免重复 Transform；
- 静态复杂家具不创建 Rigidbody；
- 开放边界不自动判失败，但必须通过实际接触和空腔测试。

构建、可视化、Collider 启停、bounds 匹配和回滚均通过。

---

## 二十七、GSP v0.1.27：真实接触、小球与报告热修复

### 27.1 空格射球真实接触

测试球通过真实 `OnCollisionEnter` 命中 GSP 静态 MeshCollider。

不使用已知不稳定的：

```text
Collision.GetContact
Collision.contacts
Collision.contactCount
ContactPoint
```

表面点和法线由托管三角网格计算。

### 27.2 球尺寸调整

原始测试球半径：

```text
0.085m
```

层间空腔测试改为：

```text
默认 0.040m
实际常用 0.035m
允许 0.020～0.120m
```

质量按体积近似缩放，避免小球仍保留过大的旧质量。

### 27.3 结果与生命周期分离

旧版球被回收时会把：

```text
TARGET_CONTACT
```

覆盖成：

```text
RECYCLED
```

热修复后分为：

```text
classification
lifecycle
```

回收或会话停止不再破坏已经完成的物理结果。

### 27.4 bounds 快照

测试启动、Collider 仍启用时保存：

```text
targetBoundsAtBegin
expandedBoundsAtBegin
```

最终报告不再从已关闭 Collider 读取零尺寸 bounds。

---

## 二十八、GSP v0.1.28：水平、引导与严格隔离空腔验证

### 28.1 为什么自由瞄准不足

自由相机射球的初速度带有很大负 Y 分量，即使瞄准层间空白，也可能在进入收纳架后撞到托盘。

因此加入：

```text
VIEW
HORIZONTAL
CAVITY_GUIDED
```

### 28.2 自动层间通道

根据已确认的三层托盘高度，测试：

```text
LOWER_TO_MIDDLE_GAP
MIDDLE_TO_UPPER_GAP
```

最初测试证明球能进入并离开目标 AABB，但附近箱子、墙面和地板会改变轨迹，且从侧面离开也可能被旧分类误判为通过。

### 28.3 严格隔离热修复

每颗引导球执行：

```text
Physics.IgnoreCollision(ballCollider, nonTargetCollider, true)
```

严格 PASS 必须满足：

```text
进入目标扩展 bounds
没有命中目标 MeshCollider
没有外部 Collider 事件
沿计划轴运动
到达指定对侧出口平面
实际出口轴与计划轴一致
横向偏移通过门槛
```

### 28.4 最终实测结果

`LogOutput(20260806-114550).log` 中：

```text
spawned = 2
guidedShots = 2
guidedPasses = 2
guidedContacts = 0
guidedDeviations = 0
timeout = 0
unclassified = 0
ignoredColliderPairs = 82
ignoreCollisionFailures = 0
otherColliderEvents = 0
LOWER_TO_MIDDLE = PASS
MIDDLE_TO_UPPER = PASS
guidedStrictSuccess = True
sourceCollidersRestored = True
proxyColliderRestored = True
```

两颗球均：

```text
expectedExitReached = True
strictAxisGate = True
plannedAxis = +Z
actualExitAxis = +Z
lateralDeviation = 0
externalInterference = False
```

因此收纳架的两个主要层间空腔已经获得自动、隔离、严格、可复核的物理通过证据。

---

## 二十九、GSP v0.1.29：PassiveColliderValidationPlan 通用化

### 29.1 版本目标

v0.1.28 的执行器仍知道：

- 一共有两条通道；
- 它们叫下—中层与中—上层；
- 高度来自 `TRAY_BOX` 中心；
- 第一个完成后固定启动第二个。

这些逻辑只适合当前收纳架，不应继续进入其他家具。

v0.1.29 将执行数据改为：

```text
PassiveColliderValidationPlan
└─ PassiveColliderValidationStep[]
```

### 29.2 配置文件

运行时配置：

```text
BepInEx/config/MakemitAGA.GeometrySourceProbe.PassiveColliderValidationPlans.txt
```

格式：

```text
planId|targetPattern|enabled|sequence|stepId|axis|heightMode|heightValue|lateralOffset|notes
```

支持：

```text
axis:
AUTO
+X
-X
+Z
-Z

heightMode:
BOUNDS_FRACTION
ABSOLUTE_WORLD_Y
CENTER_OFFSET_Y
```

同一计划可以包含任意数量步骤，按 `sequence` 顺序执行。

### 29.3 通用执行器保留的能力

```text
非目标 Collider 隔离
严格对侧出口
计划轴检查
横向偏移诊断
任意数量步骤顺序执行
结果与生命周期分离
bounds 固定快照
Collider 路径记录
源/GSP Collider 状态恢复
完整回滚
```

`StaticSourceMeshBallTest` 不再读取：

```text
TRAY_BOX
三个托盘中心
固定 0.351 / 0.691 常量
```

### 29.4 v0.1.29 迁移适配器

为了让升级后第一次运行仍能复现已经通过的收纳架测试，v0.1.29 只保留一个集中、明确标记的一次性迁移函数：

```text
EnsureV0129VerifiedMigrationBaseline
```

它把已验证计划写入 config：

```text
BedroomTumbonRoll.LayerGaps
LOWER_TO_MIDDLE_GAP
MIDDLE_TO_UPPER_GAP
```

该特化只存在于配置迁移层，不进入通用物理执行器。

### 29.5 v0.1.30 必须删除的内容

```text
EnsureV0129VerifiedMigrationBaseline
Bedroom TumbonRoll 对象名称迁移判断
旧 LOWER_TO_MIDDLE / MIDDLE_TO_UPPER 专名兼容输出
只为收纳架准备的测试说明和分支
```

不得删除：

```text
PassiveColliderValidationPlanRegistry
通用计划文件格式
严格隔离
严格出口
顺序批处理
历史统计
bounds 快照
诊断与回滚
```

### 29.6 实测验收结果

v0.1.29 已在用户的 Visual Studio + MiSide IL2CPP 环境完成两轮运行测试。

第一轮标准回归：

```text
validationPlan = BedroomTumbonRoll.LayerGaps
validationPlanSource = file:MakemitAGA.GeometrySourceProbe.PassiveColliderValidationPlans.txt
plannedSteps = 2
completedPlanSteps = 2
LOWER_TO_MIDDLE_GAP = PASS
MIDDLE_TO_UPPER_GAP = PASS
validationPlanStrictSuccess = True
ignoreCollisionFailures = 0
otherColliderEvents = 0
sourceCollidersRestored = True
proxyColliderRestored = True
```

第二轮修改配置中的 `stepId` 与 `sequence` 后，执行顺序变为：

```text
V129_UPPER_CONFIG_TEST
V129_LOWER_CONFIG_TEST
```

最终两步继续严格通过。这证明步骤名称、执行顺序、高度比例和结果汇总确实来自配置文件，而不是源码中的固定下层/上层分支。

因此 v0.1.29 的数据驱动通用化已经通过验收，可以进入对象特化退场阶段。

---

## 三十、GSP v0.1.30：对象特化测试代码正式退场

### 30.1 版本目标

v0.1.30 不再继续围绕已完成验收的单个收纳架累积源码分支。该物体的计划、部件排除与实例物理参数已经进入用户配置文件，后续源码只保留可用于其他物体的通用机制。

核心原则：

```text
对象差异进入 config
通用执行逻辑留在源码
已验证改进永久保留
一次性迁移与专名输出全部删除
```

### 30.2 v0.1.30 实际完成的退场范围

v0.1.30 已经删除并实机验证：

```text
EnsureV0129VerifiedMigrationBaseline
固定对象名称的验证计划迁移判断
固定两步骤迁移数据
LOWER_TO_MIDDLE / MIDDLE_TO_UPPER 专名摘要字段
旧 cavity_guided / ball_guided 命令别名
CAVITY_GUIDED / GUIDED_CHANNEL_* 运行分类
```

因此，严格空腔/通道测试已经改为只消费 `PassiveColliderValidationPlan`。但后续全代码库检查发现，v0.1.30 当时对“所有对象特化均已退场”的描述过于绝对：部件和实例白名单模块仍保留早期样本对象的空配置自动恢复，实例草案仍会按两个样本名称猜测固定状态和物理参数，空间实例分析器也仍输出收纳架式复合草案标签。这些残留统一由 v0.1.31 清理。

### 30.3 保留并通用化的验证能力

以下能力来自 v0.1.26～v0.1.29 的实测结果，继续保留：

```text
STATIC_NONCONVEX_MESH 静态非凸源网格代理
GSP 自有 Mesh/Collider 所有权
无 Rigidbody 的固定被动碰撞安全门
非目标 Collider 逐球隔离
严格指定对侧出口
计划轴与实际出口轴比较
横向偏移诊断
任意数量计划步骤顺序执行
结果分类与球生命周期分离
目标 bounds 启用时固定快照
Collider 对象名称和完整路径诊断
测试结束状态恢复
场景切换、回滚和卸载清理
游戏控制台输出长度安全门
```

### 30.4 新的正式入口与术语

正式计划执行命令只保留：

```text
gsp_instance_static_mesh_plan_validate(半径米)
```

运行模式统一为：

```text
PLAN_VALIDATION
```

严格通过/偏离分类统一为：

```text
PLAN_STEP_PASS
PLAN_STEP_DEVIATED
```

普通自由瞄准与水平发射仍保留，用于实体命中和诊断：

```text
VIEW
HORIZONTAL
```

### 30.5 配置成为唯一对象差异来源

通用计划文件继续使用：

```text
planId|targetPattern|enabled|sequence|stepId|axis|heightMode|heightValue|lateralOffset|notes
```

部件语义、实例代理类型和物理参数分别由原有两个白名单配置维护。源码不会依据后续测试对象名称自动选择：

```text
STATIC_NONCONVEX_MESH
COMPOUND_BOX
fixedToWorld
pickable
massKg
materialTag
```

这意味着后续测试桌子、柜子、床、相框或其他固定物体时，不需要再为每个对象新增 C# 分支，只需新增或调整数据行。

### 30.6 实机日志暴露的剩余边界

v0.1.30 的计划验证本身成功，但日志仍出现：

```text
VERTICAL_RACK_TRAYS_PLUS_FOUR_RAILS_AUDIT_ONLY
TRAY_BOX
FRAME_VERTICAL_RAIL
```

这部分没有参与 `STATIC_NONCONVEX_MESH` 的实际创建，也没有决定验证计划的步骤和高度；然而它仍把一种几何结构写成了“收纳架、托盘、四根立柱”的专用语义。结合全代码库扫描后，项目决定不再把它解释为已经通用，而是在 v0.1.31 中改写成纯几何特征驱动的“水平层板 + 覆盖框架角柱”审计。

### 30.7 升级与测试注意事项

从 v0.1.29 升级时应保留现有：

```text
MakemitAGA.GeometrySourceProbe.ObjectPhysicsWhitelist.txt
MakemitAGA.GeometrySourceProbe.SpatialInstancePhysicsWhitelist.txt
MakemitAGA.GeometrySourceProbe.PassiveColliderValidationPlans.txt
```

v0.1.30 不会删除或重写这些文件。此前已经通过验证的计划仍应由 `gsp_passive_validation_plan_reload` 读出，并使用正式命令运行。

本版源码已经完成静态结构、版本号、命令接线、对象专名残留和危险 API 检查；最终编译和运行仍以用户的 MiSide interop 环境为准。

---

## 三十一、GSP v0.1.31：全代码库对象特化清扫与通用复合审计

### 31.1 为什么需要再扫描整个项目

v0.1.30 的实机结果证明通用计划执行器没有回退：外部配置中的两个自定义步骤按指定顺序执行，均得到 `PLAN_STEP_PASS`，目标碰撞、外部干扰、偏离和超时都为零。

但日志同时显示，验证执行器之外仍存在旧样本语义。进一步检查整个源码树后确认，残留不只在复合盒审计：

```text
ObjectPhysicsWhitelist
  空配置时仍可能自动恢复早期相框部件规则

SpatialInstancePhysicsWhitelist
  空配置时仍可能自动恢复早期相框实例规则
  seed_current 仍会按样本对象名称猜测 fixed/pickable/质量/摩擦/材质

SpatialInstanceAnalyzer
  仍以 VERTICAL_RACK / TRAY_BOX / FRAME_VERTICAL_RAIL 输出复合草案

Runtime / Console / example files
  仍有 Bed、DigitalClock、BedroomPictures 等测试对象专名或过期说明

StaticSourceMeshBallTest / Report
  外部执行模式已经叫 PLAN_VALIDATION，但内部和报告仍残留 guidedChannel、guidedBatch 等旧术语
```

这些代码不一定会立即造成错误，但会使未来对象接入时出现三个风险：

1. 清空配置后，源码悄悄恢复某个历史样本规则；
2. 自动草案根据对象名称猜测物理属性，把测试经验误当成通用真相；
3. 日志和内部字段同时使用“计划”与“引导通道”两套术语，增加维护误解。

### 31.2 部件与实例配置彻底只读用户数据

v0.1.31 删除：

```text
EnsureVerifiedSemanticBaselines
EnsureVerifiedInstanceBaselines
AppendVerifiedBlock 对应的样本恢复路径
verified-baseline-restored 日志
```

新的规则是：

```text
配置文件不存在
→ 只创建带注释的空模板

配置文件存在但没有有效规则
→ 保持 rules=0
→ 不注入任何对象、部件、代理或物理参数

用户已有配置
→ 原样读取
→ 不删除、不重写、不迁移
```

这不会删除当前游戏目录中已经验证过的收纳架、Bag、SchoolBag 或相框规则。它只保证源码不再负责制造这些对象差异。

### 31.3 实例草案不再按对象名称猜物理属性

`gsp_instance_profile_seed_current` 仍会根据审计结果写出：

```text
精确 targetPath
精确 instanceKey
碰撞通道
几何推导的 proxyType
PCA 最短轴推导的 minimumThicknessMeters
```

但以下字段统一保持未标定：

```text
fixedToWorld=auto
pickable=auto
massKg=<空>
staticFriction=<空>
dynamicFriction=<空>
bounciness=<空>
materialTag=unclassified
```

原因是固定状态、是否可拿取、质量和材质都不是单靠名字或网格尺寸能够可靠确定的事实，必须来自人工配置、语义注册表或后续真实物理标定。

### 31.4 收纳架式复合草案改为通用层状结构草案

旧方法：

```text
TryBuildVerticalRackCompoundDraft
TRAY_BOX
FRAME_VERTICAL_RAIL
VERTICAL_RACK_TRAYS_PLUS_FOUR_RAILS_AUDIT_ONLY
```

新方法：

```text
TryBuildLayeredStructureCompoundDraft
HORIZONTAL_SLAB_BOX
ENCLOSING_FRAME_CORNER_POST
LAYERED_HORIZONTAL_SLABS_PLUS_CORNER_POSTS_AUDIT_ONLY
```

它只检查世界空间几何特征：

```text
至少两个闭合水平浅组件
每个层板覆盖实例主要 X/Z 范围
存在一个覆盖主要范围且高度较大的闭合组件
```

层板分别生成薄盒草案；高组件只在四角生成角柱草案，避免单一大盒封死内部开放空间。整个过程：

- 不读取对象名、路径、材质名或白名单备注；
- 不创建 Collider 或 Rigidbody；
- 始终标记 `NeedsReview=True`；
- 只作为 `COMPOUND_BOX` 的审计候选；
- 不影响配置选择的 `STATIC_NONCONVEX_MESH`。

当前边界仍需明确：该启发式假设世界 Y 轴为竖直方向，并且层板近似水平、框架近似轴对齐。旋转家具、倾斜层板或非矩形框架仍需 PCA/OBB、凸分解或人工配置，不能由这条启发式自动定案。

### 31.5 计划验证术语统一

v0.1.31 将旧内部和报告术语统一为：

```text
PlanStep
PlanHeight
PlanCalibration
PlanStepIndex
PlanShots
PlanPasses
PlanContacts
PlanDeviations
PlanBatch
```

日志字段相应改为：

```text
strictPlanExit
planBatch
planStep
planHeight
planStepIndex
```

物理行为没有改变：仍然是非目标 Collider 隔离、指定对侧平面、轴向门槛、横向偏移、顺序执行和完整状态恢复。

### 31.6 全代码库静态检查范围

本次不是只搜索几个专名，而是对整个项目进行以下检查：

```text
45 个 C# 文件逐文件字符串/字符/注释/括号状态检查
GeometrySourceProbe.csproj XML 解析
63 个游戏控制台到 Runtime 的方法调用引用检查
版本号、程序集版本和 build 标签一致性
全部源码与 example 配置中的历史样本对象专名搜索
旧 TRAY_BOX / FRAME_VERTICAL_RAIL / VERTICAL_RACK 标签搜索
旧 CAVITY_GUIDED / GUIDED_CHANNEL_* / 固定两层步骤搜索
部件/实例白名单自动基线恢复函数搜索
按 TargetName 猜测物理参数的分支搜索
vertexBufferTarget / indexBufferTarget 写入搜索
Collision.GetContact / contacts / contactCount 实际调用搜索
AsyncGPUReadback 实际调用搜索
UnityEngine.RenderingModule 硬引用搜索
所有 GSP 新建 Mesh 的 indexFormat 设置顺序检查
空间实例草案示例字段数量检查
```

静态检查结果全部通过。仍无法在当前生成环境中完成针对 MiSide interop 的最终编译，因此用户 Visual Studio 编译和游戏运行日志仍是最后门槛。

### 31.7 v0.1.31 推荐回归

保留现有三个配置文件，不要删除：

```text
MakemitAGA.GeometrySourceProbe.ObjectPhysicsWhitelist.txt
MakemitAGA.GeometrySourceProbe.SpatialInstancePhysicsWhitelist.txt
MakemitAGA.GeometrySourceProbe.PassiveColliderValidationPlans.txt
```

执行：

```text
gsp_passive_validation_plan_reload
gsp_physics_whitelist_reload
gsp_instance_profile_reload

gsp_instance_audit(已配置的固定复杂对象)
gsp_instance_report

gsp_instance_static_mesh_build
gsp_instance_static_mesh_plan_validate(0.035)
```

预期复合审计日志使用：

```text
HORIZONTAL_SLAB_BOX
ENCLOSING_FRAME_CORNER_POST
LAYERED_HORIZONTAL_SLABS_PLUS_CORNER_POSTS_AUDIT_ONLY
```

不应再出现：

```text
TRAY_BOX
FRAME_VERTICAL_RAIL
VERTICAL_RACK_TRAYS_PLUS_FOUR_RAILS_AUDIT_ONLY
```

计划验证应继续满足：

```text
class=PLAN_STEP_PASS
planShots=计划步骤数
planPasses=计划步骤数
planContacts=0
planDeviations=0
validationPlanStrictSuccess=True
ignoreCollisionFailures=0
otherColliderEvents=0
sourceCollidersRestored=True
proxyColliderRestored=True
```

### 31.8 当前总体结论

截至 v0.1.31，源码边界正式变为：

```text
对象差异、语义确认和物理参数
→ 只存在于用户配置/长期注册表

几何获取、拓扑审计、实例拆分、代理规划和验证执行
→ 只依赖通用数据结构与几何特征

历史样本测试结果
→ 保留为设计经验和配置数据
→ 不再成为源码自动分支
```

这使后续增加新物体时不再需要复制“某对象专用 C# 测试代码”。新的固定物体应先通过通用审计和配置计划；可拿取物体则进入独立的凸代理、质量、惯性和抓取验证主线。

---

## 三十二、GSP v0.1.32（历史归档；相关发射测试已在 v0.2.0 删除）：目标相对水平截面与通用发射校准

### 32.1 本版解决的问题

在新对象 ChairPC 的第一次普通水平测试中，目标碰撞体已经成功创建，VIEW 模式也取得多次真实接触；但 HORIZONTAL 模式的四颗球全部为 `BOUNDS_MISS`。

原因不是网格或 Collider 失败，而是旧 HORIZONTAL 直接使用相机世界高度：

```text
目标最高点约为 1.114m
相机水平球高度约为 1.700m
速度 Y=0
重力=False
```

球会永久保持在目标上方，因此无法进入目标 bounds。这个问题并不局限于椅子：只要相机高度高于或低于被测物体，普通水平发射都可能失去意义。

### 32.2 新的对象无关输入

正式命令扩展为：

```text
gsp_instance_static_mesh_ball_horizontal
gsp_instance_static_mesh_ball_horizontal(半径)
gsp_instance_static_mesh_ball_horizontal(半径,高度比例)
gsp_instance_static_mesh_ball_horizontal(半径,高度比例,横向比例)
```

默认值：

```text
球半径=0.035m
高度比例=0.50
横向比例=0.00
```

参数含义：

```text
高度比例 0~1
0   = 目标 bounds 最低处
0.5 = 目标 bounds 中部
1   = 目标 bounds 最高处

横向比例 -0.5~0.5
0    = 当前水平截面的中央
负值 = 横向一侧
正值 = 横向另一侧
```

最终世界坐标会根据球半径和 0.01m 安全余量自动夹紧，避免球心被生成到 bounds 边缘之外或目标薄截面内部。

### 32.3 相机只决定“从哪一侧进入”

旧实现：

```text
生成位置 = 相机位置 + 相机水平 forward × 0.65
生成高度 = 相机世界 Y
发射方向 = 相机水平 forward
```

新实现：

```text
生成高度 = targetBounds.minY + 高度比例 × targetBounds.sizeY
横向位置 = targetBounds.center + 横向比例 × 对应横向尺寸
发射方向 = 相机到目标中心的水平向量
再吸附到主世界轴 ±X / ±Z
生成位置 = 目标 AABB 近侧平面之外
```

因此相机不再决定绝对高度，也不要求准星精确指向物体；相机只用于判断从目标的哪一个近侧面射入。

这种方式对以下对象同样适用：

- 椅子、桌子、柜子、床架；
- 架子、开放式箱体、机器外壳；
- 高度远低于相机的小物体；
- 高度高于相机的大型固定物体。

### 32.4 新的运行时诊断字段

HORIZONTAL 每次发射会新增：

```text
[HORIZONTAL_SLICE]
requestedHeightFraction
resolvedHeightFraction
height
requestedLateralFraction
resolvedLateralFraction
axis
spawn
velocity
```

逐球报告新增：

```text
launchHeight
launchHeightFraction
launchLateralFraction
launchCalibration
```

其中 `launchCalibration` 示例：

```text
TARGET_BOUNDS_HORIZONTAL:
requestedHeightFraction=0.3:
resolvedHeightFraction=0.3:
requestedLateralFraction=0:
resolvedLateralFraction=0:
axis=+X
```

这样可以明确区分：

- 用户要求的截面；
- 经过球半径安全夹紧后的实际截面；
- 实际使用的主水平轴；
- 最终生成位置和速度。

### 32.5 不改变的安全边界

本版只改变普通 HORIZONTAL 的发射校准，不改变：

```text
STATIC_NONCONVEX_MESH 创建条件
无 Rigidbody 的固定物体边界
源 Collider 暂时禁用与恢复
非目标 Collider 隔离
最多三个运行时球
结果与生命周期分离
bounds 固定快照
PLAN_VALIDATION 严格对侧出口
完整关闭与回滚
```

VIEW 模式仍保留完整相机 forward，用于自由瞄准实体表面；PLAN_VALIDATION 仍完全由外部计划文件驱动。

### 32.6 推荐通用测试截面

对一个新固定物体，可以依次测试：

```text
gsp_instance_static_mesh_ball_horizontal(0.035,0.15,0)
gsp_instance_static_mesh_ball_horizontal(0.035,0.30,0)
gsp_instance_static_mesh_ball_horizontal(0.035,0.50,0)
gsp_instance_static_mesh_ball_horizontal(0.035,0.75,0)
gsp_instance_static_mesh_ball_horizontal(0.035,0.90,0)
```

每次启动后按空格发射，测试结束执行：

```text
gsp_instance_static_mesh_ball_status
gsp_instance_static_mesh_ball_report
gsp_instance_static_mesh_ball_off
```

解释原则：

```text
TARGET_CONTACT
= 该截面命中了真实网格表面

AABB_PASS_NO_CONTACT
= 该截面进入并离开整体 bounds，但没有命中网格；属于开放空间候选

BOUNDS_MISS
= 理论目标相对发射器不应频繁出现；需要检查轴、bounds 或参数夹紧

EXTERNAL_INTERFERENCE
= 非目标 Collider 隔离不完整，样本无效
```

普通 HORIZONTAL 仍不是严格空腔证明。需要正式证明一条指定通道时，应把确认后的高度、轴和横向偏移写入 `PassiveColliderValidationPlan`，再使用：

```text
gsp_instance_static_mesh_plan_validate(0.035)
```

### 32.7 当前总体结论

v0.1.32 将普通水平测试从“依赖玩家相机高度的临时射球工具”提升为：

```text
基于目标 bounds
可配置高度截面
可配置横向截面
自动选择近侧主轴
从 AABB 外部稳定射入
可在报告中完整复核
```

实现不读取任何对象名称，也没有 ChairPC 专用分支。ChairPC 只是暴露问题的第二类结构样本；改进本身适用于所有已创建静态源网格代理的固定物体。

---

## 三十三、GSP v0.2.0：固定被动碰撞体基础冻结

### 33.1 为什么在这里冻结 GSP

置物架和 ChairPC 已经证明，GSP 的核心成果不是某一种小球发射方法，而是：

```text
从运行时 Renderer 取得真实路由子网格
→ 统一静态合批与普通 Renderer 的世界坐标
→ 区分实体、视觉层、特效和独立子对象
→ 焊接审计副本并拆分组件/现实实例
→ 保存碰撞聚合快照
→ 创建 GSP 自有静态非凸 MeshCollider
→ 不修改源网格、不创建 Rigidbody
→ 可视化、Collider 启停、场景切换与完整回滚
```

ChairPC 已经在第二类复杂物体上完成：

```text
4103 vertices
10638 indices
3546 triangles
STATIC_NONCONVEX_MESH
convex=false
rigidbody=false
rendererMatch=true
colliderMatch=true
```

手动接触测试也已经证明动态凸形测试体能与该静态非凸代理发生真实碰撞，并且至少存在可穿过的开放区域。继续增加不同高度、不同横向比例的小球轨迹，不会继续提高被动碰撞体的核心能力，反而会扩大状态机、报告字段、配置文件和维护负担。

因此 v0.2.0 将 GSP 定位为：

> **稳定、可回滚、对象无关的固定被动环境碰撞体提供者。**

### 33.2 v0.2.0 删除的功能

源码中正式删除：

```text
D3D11PhysicsBallTest
D3D11BallCollisionTracker
StaticSourceMeshBallTest
StaticSourceMeshBallCollisionTracker
D3D11PhysicsBallTestReport
StaticSourceMeshBallTestReport
PassiveColliderValidationPlan
PassiveColliderValidationPlanRegistry
PassiveColliderValidationPlans.example.txt
```

同时删除以下运行入口和状态更新：

```text
相机自由小球发射
自然落球
目标相对水平截面发射
静态源网格小球状态/报告/关闭
数据驱动通道计划加载、重载与严格验证
小球 Update / FixedUpdate
小球结果写入环境观测 DTO
```

退役命令包括：

```text
gsp_instance_static_mesh_ball_launcher
gsp_instance_static_mesh_ball_horizontal
gsp_instance_static_mesh_plan_validate
gsp_instance_static_mesh_ball_status
gsp_instance_static_mesh_ball_report
gsp_instance_static_mesh_ball_off
以及旧 D3D11 小球/落球命令
```

已有游戏目录中的：

```text
MakemitAGA.GeometrySourceProbe.PassiveColliderValidationPlans.txt
```

从 v0.2.0 起不再读取。它可以作为历史测试记录备份，也可以由用户手动移出 config；GSP 不再自动修改它。

### 33.3 v0.2.0 保留的核心能力

仍然保留：

```text
D3D11 staging 只读几何获取
完整路由子网格验证
普通 Renderer / 静态合批坐标判定
部件角色白名单
空间实例级物理配置
组件与现实实例拆分
对象无关分层结构复合草案
单盒/复合盒代理
静态非凸源网格代理
环境观测 DTO
OBJ/文本报告导出
半透明预览
Collider 显式启停
统一控制台字符预算安全门
场景切换、插件退出和手动完整回滚
```

还保留简单盒/复合盒的受控接触测试：

```text
gsp_instance_proxy_contact_test
```

它不是已删除的小球发射系统，而是对简单代理的独立受控刚体接触与托管 OBB 表面点/法线验证。它继续服务于盒代理回归，不参与静态源网格小球或通道计划。

### 33.4 当前正式输出档位

#### A. 简单固定物体

```text
ORIENTED_THIN_BOX
ORIENTED_BOX
COMPOUND_BOX / 通用复合盒草案
```

适合相框、薄板、规则柜体和可由少量盒覆盖的固定结构。

#### B. 复杂固定物体

```text
STATIC_NONCONVEX_MESH
MeshCollider.convex=false
Rigidbody=false
fixedToWorld=true
pickable=false
```

适合当前阶段的床、桌、椅、置物架和其他固定环境。它保留原三角网格的凹陷与开放空间，但只能作为静态环境，不用于可移动 Rigidbody。

### 33.5 与 Active Ragdoll Lab 的分工

从 v0.2.0 起，碰撞有效性的主要消费者由小球切换为真实身体代理：

```text
GSP v0.2.0
负责环境几何与固定被动 Collider
        ↓ Unity Physics
Active Ragdoll Lab
负责骨盆、躯干、四肢 Rigidbody / Collider / Joint
        ↓
未来 ML-Agents
负责平衡、动作修正和环境交互策略
```

新的验证重点不再是“某一颗球是否穿过某条线”，而是：

```text
身体是否能被座面稳定支撑
背部是否能靠在靠背上
手臂是否能接触桌面和扶手
多刚体同时接触是否抖动
关节驱动持续施力时是否穿透或弹飞
碰撞结束和场景切换后是否完整恢复
```

这比继续扩展小球工具更接近最终目标。

### 33.6 为什么暂时不开发拿取和动态复杂物体

可拿取复杂物体需要同时解决：

```text
动态 Rigidbody
多个凸 Collider 或凸分解
质量与惯量
摩擦与抓握约束
手部 Active Ragdoll
Agent 控制
拿起、放下和失手状态机
```

若在 Active Ragdoll 基础尚未稳定时同时开发这些内容，一旦出现抖动、穿透或爆炸，很难判断问题来自：

```text
环境 Collider
角色关节
手部控制
物体凸分解
质量参数
抓握约束
ML 策略
```

因此当前门槛明确为：

> **先让自定义 Active Ragdoll 与 GSP v0.2.0 的固定被动碰撞体稳定运行，再启动可拿取物体、动态刚体和 CoACD/V-HACD 凸分解。**

这不是放弃拿取功能，而是把它放到依赖条件成熟之后。

### 33.7 v0.2.0 推荐回归流程

以 ChairPC 或另一个已写入实例配置的固定复杂物体为例：

```text
gsp_instance_profile_reload
gsp_physics_whitelist_reload

gsp_instance_audit(ChairPC)
gsp_instance_static_mesh_build
gsp_instance_static_mesh_status
gsp_instance_static_mesh_visual(on)
gsp_instance_static_mesh_collider(on)
```

预期状态：

```text
success=True
profile=True:STATIC_NONCONVEX_MESH
fixed=True
pickable=False
convex=False
trigger=False
rigidbody=False
rendererMatch=True
colliderMatch=True
```

随后由 ARL 打开身体 Collider 并执行真实接触测试。结束后：

```text
arl_colliders(off)
arl_drive(off)
arl_rollback

gsp_instance_static_mesh_collider(off)
gsp_instance_static_mesh_rollback
```

### 33.8 冻结边界

GSP v0.2.0 暂停新增：

- 静态源网格小球与通道测试；
- 可拿取复杂物体；
- 动态非凸 MeshCollider；
- 自动凸分解；
- 质量、惯量和抓握系统；
- ML-Agents 控制逻辑。

只有在以下条件通过后，才重新开启动态物体路线：

```text
ARL 全身代理可稳定创建
关节驱动与回滚稳定
与至少两类 GSP 固定环境物体持续接触通过
基础坐、靠、跌落或支撑测试通过
ML-Agents 能在固定环境上稳定读取接触并输出受限动作
```

届时新阶段应单独定义为：

```text
动态复杂物体代理
→ CoACD / V-HACD 后端接口
→ 多凸包 Compound Collider
→ Rigidbody 质量与惯量
→ 手部抓握与 Agent 交互
```

### 33.9 v0.2.0 阶段结论

项目当前主线调整为：

```text
GSP v0.2.0：冻结的固定被动环境碰撞基础
ARL v0.1.x+：自定义 Active Ragdoll 物理消费者
未来 ML-Agents：控制与学习层
动态拿取：在前两层稳定后重新启动
```

这种拆分减少了同时变化的模块数量，也让后续故障归因更加清楚。GSP 不是结束，而是完成了当前阶段应承担的底层职责。

