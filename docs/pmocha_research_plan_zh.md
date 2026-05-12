# P-MOCHA 中文研究计划

## 0. 核心判断

这个新方向不应该被包装成：

```text
MOCHA + RAST
```

这种说法会让贡献看起来像是把 RAST 的风险走廊前端接到 MOCHA 优化器上，创新点会偏工程拼接。

更强的定位应该是：

> 感知不确定性不仅会改变局部碰撞风险，还会改变安全拓扑和最佳观测策略。因此，我们将动态风险、感知不确定性和遮挡建模为持久多同伦风险结构，并在 MOCHA 的低维轨迹变量中联合优化运动安全性和未来可观测性。

暂定名称：

**P-MOCHA: Perception-aware Multi-Homotopy Trajectory Optimization under Dynamic Uncertainty and Occlusion**

中文可称为：

**感知不确定性驱动的多同伦主动观测轨迹优化。**

核心问题不是：

> 怎么把感知结果接到规划器上？

而是：

> 在动态不确定环境中，机器人应该只是躲开高风险区域，还是应该主动选择一条能更早暴露未来风险、更好降低感知不确定性的轨迹？

我们要回答的是后者。

## 1. 当前代码基础研判

当前工作区里有两类非常相关的代码：

- `dynus/`
- `RAST_corridor_planning/`

这两部分都可以复用，但复用方式不同。

### 1.1 DYNUS

路径：`dynus/`

DYNUS 是一个动态未知环境中的不确定性感知轨迹规划器。它已经包含很多和 P-MOCHA 接近的基础模块：

- 通过 `dynTraj` 管理动态障碍物轨迹。
- 在 `DYNUS::computeRisk` 中使用 unknown cells 计算环境不确定性。
- 在 `DYNUS::computeRisk` 中使用动态障碍物距离和数量计算动态风险。
- 通过 `DYNUS::getSafeCorridor` 构建安全走廊。
- 通过 `DYNUS::replan` 实现完整重规划流程。
- 通过 `DYNUS::generateGlobalPath` 和 `DYNUS::pushPath` 生成并调整全局路径。
- 通过 `DYNUS::generateLocalTrajectory` 做局部轨迹优化。
- 通过 `DYNUS::findFrontiers`、`DYNUS::extractFrontierPoints` 和 `DYNUS::computeAllFrontierPoints` 提取 frontier。
- 通过 `YawSolver` 做观测方向优化，其中已经包含 collision likelihood、time-since-observed、proximity、yaw change 等代价。

关键判断：

> DYNUS 已经有 uncertainty、frontier、dynamic obstacle、yaw observation 的雏形，但这些模块还没有被统一成一个“可优化的感知-规划联合目标”。

也就是说，DYNUS 的感知目前主要服务于：

- 地图更新；
- frontier 探索；
- 风险估计；
- yaw 选择；
- 动态障碍物避让。

但它还没有做到：

> 让轨迹本身主动选择能降低未来不确定性的拓扑和视角。

这正是 P-MOCHA 可以切入的地方。

### 1.2 RAST Corridor Planning

路径：`RAST_corridor_planning/`

RAST 相关代码里已经有：

- 通过 `future_risk_global` 接收未来风险图。
- 通过 `risk_aware_kinodynamic_a_star.h` 做 risk-aware kinodynamic A*。
- 通过风险阈值检查 motion primitive 是否安全。
- 通过 `findCorridors` 扩展风险安全走廊。
- 通过 mini-snap 在 corridor 内优化轨迹。

RAST 的 pipeline 可以抽象为：

```text
future risk map -> risk-aware kinodynamic A* -> one corridor chain -> polynomial optimization
```

这个 pipeline 很有价值，但不能直接作为新论文主线。

它可以作为：

- baseline；
- risk map/corridor 构造参考；
- 动态风险实验对照；
- 单走廊方法的对比对象。

但我们不应该把它作为最终框架。

关键原因：

> RAST 是 risk-aware，但不是 perception-improving。它会根据不确定风险生成安全走廊，但轨迹不会主动为了改善未来感知而移动。

### 1.3 对新工作的启发

当前代码已经足够支持一个分阶段实验路线：

- 用 DYNUS 复用地图、frontier、动态障碍物、yaw、replanning 基础设施。
- 用 RAST 复用风险地图和走廊生成思想。
- 新增一个多候选拓扑层，用来生成和评价多条风险/可见性同伦路径。
- 先做 candidate-level active observation scoring，再改优化器。

最重要的代码策略是：

> 先证明“主动观测会改变拓扑选择并降低隐藏动态风险”，再把这个目标嵌入 MOCHA 低维优化。

不要一开始就重写优化器。

## 2. 相关论文定位

### 2.1 RAST

论文：

**RAST: Risk-Aware Spatio-Temporal Safety Corridors for MAV Navigation in Dynamic Uncertain Environments**

RA-L 2023。

它的核心思想是：

- 使用 particle-based dynamic map 表示环境不确定性。
- 预测未来风险。
- 构建 risk-aware spatio-temporal safety corridors。
- 在 corridor 内优化 MAV 的动态可行轨迹。

我们借鉴它的部分：

- 时空风险图 `R(x,t)`；
- 动态障碍物不确定性；
- 风险安全走廊；
- 在不同不确定性水平下做大量实验。

我们不应照搬的部分：

- 单一 corridor；
- 单一路径参考；
- 风险感知但被动的规划方式；
- MAV 场景绑定。

我们的区别应该写成：

> RAST 将不确定性转化为一个安全走廊；P-MOCHA 将不确定性转化为多个持久风险拓扑候选，并联合优化运动安全和未来可观测性。

### 2.2 Occlusion-Aware MPC

代表方向：

**OA-MPC: Occlusion-Aware MPC for Guaranteed Safe Robot Navigation with Unseen Dynamic Obstacles**

核心思想：

- 显式考虑遮挡区域中可能存在的隐藏动态障碍物；
- 用 reachable set 表示隐藏障碍物可能到达的位置；
- 在 MPC 中保证安全；
- 使用 terminal stopping constraints 保证递归可行性。

我们借鉴它的部分：

- 遮挡区域不是普通 unknown space；
- 遮挡后方可能存在会和机器人发生交互的动态 agent；
- 安全性应该在障碍物出现之前就被考虑。

我们的区别：

- 我们不只做最坏情况安全约束；
- 我们希望轨迹主动减少遮挡诱导的不确定性；
- 我们把这个问题放入多同伦选择和 MOCHA 低维轨迹优化中。

### 2.3 Alternate Perspective / Visibility Cost Map

代表方向：

**Estimating Visibility from Alternate Perspectives for Motion Planning with Occlusions**

核心思想：

- 估计从候选未来位置能看到什么；
- 构建 visibility/information cost map；
- 让规划器提前暴露遮挡区域。

我们借鉴它的部分：

- 可以在机器人到达之前估计未来视角的信息收益；
- 轨迹可以因为“更早看见风险区域”而发生偏移；
- 可见性本身可以作为规划代价。

我们的区别：

- 我们不是只构建一个 grid cost map；
- 我们要让 visibility 影响 homotopy selection；
- 最终还要让 visibility 进入连续时间轨迹优化。

### 2.4 Active Perception / Observability-Aware Planning

相关方向包括：

- active perception with NeRF；
- observability-aware trajectory optimization；
- viewpoint planning；
- next-best-view；
- target tracking with FoV / line-of-sight constraints。

这些方法的共同问题是：

- 很多是先选 viewpoint，再交给 planner；
- 很多没有把动态障碍物风险、车辆动力学和连续时间轨迹优化放在一起；
- 很多没有利用 MOCHA 这种低维重参数化结构。

我们的区别：

> 同一组低维变量应该同时决定运动可行性和未来可观测性。

这句话是新论文和 MOCHA 连接最紧的地方。

## 3. 数学建模

### 3.1 状态、轨迹和低维变量

机器人状态写成：

```text
q(t) = [p(t), psi(t)]
```

其中：

- `p(t)` 是位置；
- `psi(t)` 是 yaw 或传感器朝向。

MOCHA 的核心优势是把原本高维 waypoint 优化降到低维参数：

```text
z = [lambda_1, ..., lambda_M, tau_1, ..., tau_N]
```

或者等价的低维轨迹参数。

因此，我们希望所有感知相关代价最终都能写成：

```text
J(z) = integral cost(q(t; z), t) dt
```

这样感知不再是外部前端，而是 MOCHA 低维优化的一部分。

### 3.2 面向规划的感知表示

感知模块不应该只输出 occupancy、detection 或 segmentation。

它应该输出一个 planner-facing belief field：

```text
B(x, t) = { R(x, t), U(x, t), O(x, t), S(x, t) }
```

其中：

- `R(x,t)`：动态碰撞风险；
- `U(x,t)`：感知/预测不确定性；
- `O(x,t)`：遮挡或不可观测区域风险；
- `S(x,t)`：语义或任务重要性，后续可选。

最小版本：

```text
B_0(x,t) = { R(x,t), U(x,t) }
```

推荐的第一版论文系统：

```text
B_1(x,t) = { R(x,t), U(x,t), O(x,t), V(q,x,t) }
```

其中 `V(q,x,t)` 表示从机器人状态 `q` 到位置或区域 `x` 的可见性。

### 3.3 风险和不确定性代价

给定轨迹 `gamma`，风险代价定义为：

```text
J_risk(gamma) = integral_0^T phi(R(p(t), t) - R_safe) dt
```

其中 `phi` 可以用 smooth hinge：

```text
phi(s) = log(1 + exp(beta s)) / beta
```

不确定性代价：

```text
J_unc(gamma) = integral_0^T U(p(t), t) dt
```

这两项对应被动风险规避：

- 避开高风险区域；
- 避开高不确定性区域。

但这还不够。

如果只有这两项，方法仍然更像 risk-aware planning，而不是 active perception planning。

### 3.4 Occlusion Shadow Risk

定义遮挡或未知区域：

```text
Omega_occ(t)
```

对其中每个未知或遮挡单元 `u`，定义隐藏动态障碍物概率：

```text
P_hidden(u, t)
```

遮挡阴影风险可以定义为：

```text
J_shadow(gamma) =
integral_0^T sum_{u in Omega_occ(t)} P_hidden(u,t) * A(u, p(t), t) dt
```

其中 `A(u,p,t)` 表示从未知区域 `u` 出现的隐藏 agent 是否可能影响机器人未来运动。

最小实现可以用：

```text
A(u,p,t) = exp(-d(u, p)^2 / sigma_a^2)
```

更强实现可以用 reachable set：

```text
A(u,p,t) = 1 if Reach(u, t -> t + Delta) intersects robot swept volume
```

这项的直觉是：

> unknown space 不是均匀危险的。只有那些可能藏着动态障碍物并且会影响未来路径的 unknown space 才真正危险。

### 3.5 可见性和主动观测收益

定义从机器人状态 `q(t)` 到点或区域 `x` 的可见性：

```text
V(q(t), x, t) =
I_FOV(q(t), x) * I_LOS(q(t), x) * Q_dist(d(q(t),x)) * Q_angle(theta(q(t),x))
```

其中：

- `I_FOV`：是否在传感器视场内；
- `I_LOS`：line-of-sight 是否被遮挡；
- `Q_dist`：距离是否合适；
- `Q_angle`：视角是否合适。

硬指示函数可以软化：

```text
I_FOV -> sigmoid(theta_max - abs(theta))
I_LOS -> exp(-k * accumulated_occupancy_along_ray)
Q_dist -> exp(-(d - d_best)^2 / sigma_d^2)
```

主动观测收益：

```text
G_obs(gamma) =
integral_0^T sum_{u in Omega_interest(t)} W(u,t) * V(q(t), u, t) dt
```

其中 `Omega_interest` 可以包括：

- 遮挡边界；
- frontier；
- 预测动态障碍物；
- 高不确定性区域；
- 可能丢失的目标。

主动感知损失可以写成：

```text
J_obs(gamma) = -G_obs(gamma)
```

或者：

```text
J_obs(gamma) =
integral_0^T sum_u W(u,t) * (1 - V(q(t),u,t)) dt
```

这是整个新方向里最关键的感知-规划耦合项。

### 3.6 Persistent Risk Homotopy

单一风险阈值不稳定：

```text
R(x,t) > rho
```

感知噪声稍微变化，就可能让某个通道从 free 变成 blocked，或者反过来。

因此应该使用多个风险阈值：

```text
rho_1 < rho_2 < ... < rho_L
```

在每个阈值下构建风险障碍集合：

```text
O_rho_l(t) = { x | R(x,t) + eta U(x,t) > rho_l }
```

对路径 `gamma`，计算多阈值拓扑签名：

```text
H_risk(gamma) = [ h_{rho_1}(gamma), ..., h_{rho_L}(gamma) ]
```

路径的拓扑稳定性可以用前后帧签名一致性衡量：

```text
Stability(gamma) =
consistency(H_risk(gamma; B_t), H_risk(gamma; B_{t-1}))
```

第一版不需要做完整代数拓扑。

实用实现可以这样做：

- 标记路径从高风险连通区域左侧还是右侧经过；
- 标记路径是否经过某些 occlusion/frontier component；
- 标记路径在动态障碍物之前还是之后穿过交互区域；
- 标记路径能观察到哪些关键遮挡区域。

这样可以形成：

```text
H_perception(gamma) = [ H_geo, H_risk, H_occ, H_vis ]
```

这个比传统 homotopy 更强，因为：

> 两条几何上类似的路径，可能在可见性和遮挡暴露能力上完全不同。

### 3.7 最终目标函数

对每个候选同伦走廊 `k`，优化：

```text
min_z J_k(z)
```

其中：

```text
J_k =
w_s J_smooth
+ w_t J_time
+ w_d J_dyn
+ w_c J_collision
+ w_r J_risk
+ w_u J_unc
+ w_o J_shadow
+ w_v J_obs
+ w_h J_corridor
```

约束包括：

- 动力学约束；
- 碰撞安全距离；
- corridor 约束；
- 全向底盘或阿克曼底盘平台约束。

关键设计原则：

> 不要一次性把所有项放进优化器。先做候选路径层面的 scoring，再逐步进入连续优化。

## 4. 数学上强的地方和风险点

### 4.1 强的地方

这个方向数学上是成立的，因为：

- RAST 已经证明了时空风险图 `R(x,t)` 有意义；
- DYNUS 已经有 unknown cell 和动态障碍物风险；
- frontier extraction 可以自然近似 `Omega_interest`；
- yaw solver 已经有 time-since-observed 和动态目标观测逻辑；
- MOCHA 的低维参数化天然适合把感知代价投影进去；
- 多同伦候选可以避免单一路径局部最优。

最重要的是：

> 感知不确定性不仅改变代价值，还改变拓扑选择。

这个观点是论文最有价值的数学动机。

### 4.2 风险点

第一个风险：

动态不确定场里的严格 homotopy 可能过于复杂。

应对方式：

> 先用 persistent signature 作为候选路径标签和稳定性指标，不要一开始承诺完整拓扑理论保证。

第二个风险：

visibility 和 line-of-sight 通常是非光滑的。

应对方式：

> 先做离散候选路径 scoring，再用 soft visibility approximation 进入优化器。

第三个风险：

主动观测可能和运动效率冲突，导致机器人过度绕路。

应对方式：

> 只奖励 safety-critical uncertainty 的观测收益，而不是奖励所有 unknown space。

也就是说，机器人不需要为了看清所有地方而移动，只需要为了看清会影响未来安全和任务成功的区域而移动。

## 5. 代码集成路线

### 5.1 Phase 0：复现实验基线

目标：

在加新模块前，先建立可靠 baseline。

使用：

- 原始 DYNUS；
- RAST corridor planner；
- 原始 MOCHA，如果已有可运行版本。

指标：

- success rate；
- collision rate；
- planning latency；
- dynamic obstacle near-miss distance；
- path length/time；
- replan count。

产出：

> 一套可以重复运行的动态遮挡场景和评估脚本。

### 5.2 Phase 1：Risk Field Adapter

目标：

建立统一的 planner-facing risk interface：

```cpp
struct PerceptionRiskField {
  double risk(const Eigen::Vector3d& x, double t) const;
  double uncertainty(const Eigen::Vector3d& x, double t) const;
  double occlusion(const Eigen::Vector3d& x, double t) const;
};
```

第一版：

- 封装 DYNUS 的 unknown cells；
- 封装 dynamic obstacles；
- 生成简单 `R(x,t)` 和 `U(x,t)`。

第二版：

- 接入 RAST 风格 future risk map。

第三版：

- 加入 occlusion shadow risk。

接入点：

- `DYNUS::computeRisk`；
- `DYNUS::generateGlobalPath`；
- `DYNUS::generateLocalTrajectory`；
- 后续 MOCHA nonlinear optimizer。

### 5.3 Phase 2：多同伦候选路径生成

目标：

不再只生成一条 guide path，而是生成多条候选：

```text
gamma_1, gamma_2, ..., gamma_K
```

最小实现：

- 用不同风险阈值运行 graph search；
- 用不同动态障碍物 inflation 运行 graph search；
- 根据空间差异和风险签名保留 K 条不同路径。

更好实现：

- 用 `H_risk` 和 `H_occ` 给候选路径打标签；
- 每种标签保留一到两条代表路径。

接入点：

- DYNUS 当前 global planner 只输出一个 `global_path`；
- 可以包装成 `std::vector<GlobalCandidate>`；
- 每个 candidate 保存 path、risk score、uncertainty score、visibility score 和 corridor。

### 5.4 Phase 3：候选路径级主动观测评分

目标：

在改优化器之前，先证明主动观测会改变拓扑选择。

对每条候选路径 `gamma_k` 计算：

```text
Score(gamma_k) =
J_motion
+ w_r J_risk
+ w_u J_unc
+ w_o J_shadow
- w_g G_obs
```

第一版用采样点即可。

可见性近似：

- 是否在 FoV 内；
- 是否在传感器 range 内；
- 是否 line-of-sight clear；
- 能看到多少 frontier、occlusion boundary 或 dynamic obstacle。

接入点：

- `DYNUS::computeAllFrontierPoints` 已经能提供 frontier points 和 directions；
- `YawSolver` 已经在考虑观测哪个动态障碍物；
- 这一步可以完全不动 Gurobi/MOCHA。

这是第一个真正应该做的实验模块。

原因：

> 它成本最低，但可以直接验证论文最核心的假设：主动观测会改变路径选择并改善安全性。

### 5.5 Phase 4：Occlusion Shadow Field

目标：

把 unknown/occluded region 从普通 unknown cost 升级为 future dynamic risk。

最小版本：

- 找到路径附近 frontier cells；
- 给被障碍物遮挡的 unknown 区域更高风险；
- 风险随距离和时间衰减。

更强版本：

- 从 occlusion boundary 向外传播 hidden-agent reachable set；
- 惩罚和机器人 swept volume 相交的区域。

接入点：

- DYNUS octomap unknown cells；
- `computeAllFrontierPoints` 给出的 frontier directions；
- `dynTraj` 动态障碍物预测结构。

### 5.6 Phase 5：低维优化集成

目标：

从 candidate scoring 进入真正的轨迹优化。

第一步集成：

- 保持 corridor hard constraints 不变；
- 在候选走廊内做优化；
- 把感知分数作为 candidate selection 或 soft reference。

第二步集成：

加入 sampled differentiable costs：

```text
sum_i R(p(t_i),t_i)
sum_i U(p(t_i),t_i)
sum_i O(p(t_i),t_i)
sum_i (1 - V(q(t_i), x_i, t_i))
```

注意：

当前 DYNUS 的 Gurobi solver 主要是二次目标和线性/二次约束。任意非线性的 risk/visibility field 不适合硬塞进这个 solver。

因此有两个路线：

1. 在 DYNUS 中先做 candidate-level scoring，保持 Gurobi 优化不变。
2. 在 MOCHA 的 nonlinear optimizer 中加入真正的可微感知代价。

推荐：

> 不要强行把复杂非线性感知代价塞进当前 DYNUS Gurobi solver。DYNUS 用来做基础设施和 baseline，完整的 perception-aware cost 放进 MOCHA。

### 5.7 Phase 6：闭环重规划

目标：

形成双层重规划：

```text
low-frequency: update risk topology and candidate guides
high-frequency: optimize within selected topology using MOCHA
```

低频层：

- 更新 risk field；
- 生成 persistent homotopy candidates；
- 选择当前拓扑。

高频层：

- 在当前拓扑内用 MOCHA 快速重优化；
- 做 safety check；
- 如果风险拓扑突变，触发 fallback。

这正好对应论文故事：

> 拓扑低频稳定更新，轨迹高频低维优化。

## 6. 实验计划

### 6.1 实验 A：静态遮挡 Toy World

目的：

先在没有动态障碍物复杂性的情况下验证主动观测。

场景：

- 走廊拐角；
- 静态遮挡物；
- 目标在遮挡区域附近或后方。

对比：

- shortest/smoothest path；
- risk-only path；
- active observation path。

指标：

- time-to-first-observation；
- occlusion exposure ratio；
- path length；
- planning time。

预期结果：

主动观测路径会稍微绕一点，但能更早看到遮挡区域。

### 6.2 实验 B：隐藏动态障碍物

目的：

证明遮挡不是普通 unknown，而是安全关键风险。

场景：

- 行人或移动障碍物从墙角、车辆、货架或人群后方出现；
- 机器人需要到达目标点，同时避免碰撞。

对比：

- DYNUS/MOCHA baseline；
- RAST-like risk corridor；
- P-MOCHA without active observation；
- P-MOCHA full。

指标：

- collision rate；
- near-miss distance；
- emergency replan count；
- time-to-first-observation of hidden obstacle；
- success rate。

预期结果：

risk-only planner 要么过于保守，要么反应太晚；P-MOCHA 应该能更早暴露隐藏风险，减少急刹和紧急重规划。

### 6.3 实验 C：多同伦选择

目的：

证明感知会改变拓扑选择。

场景：

- 绕过一个障碍物或货架有左右两条路；
- 两条路几何上都可行；
- 其中一条路对关键遮挡区域可见性更差。

对比：

- geometry-only homotopy；
- risk-only homotopy；
- risk + visibility homotopy。

指标：

- selected homotopy class；
- risk signature stability；
- occlusion exposure ratio；
- collision/success rate。

预期结果：

active observation 可能选择更长但更安全的路径，因为它能更早看到关键区域。

### 6.4 实验 D：感知噪声鲁棒性

目的：

证明 persistent risk topology 的价值。

场景：

- 同一地图加入 risk map noise；
- 动态障碍物预测 covariance 改变。

对比：

- single risk threshold corridor；
- multi-threshold persistent risk homotopy。

指标：

- topology switches；
- replan jitter；
- success rate；
- computation time。

预期结果：

multi-threshold persistent signature 能减少拓扑抖动和重规划不稳定。

### 6.5 实验 E：平台对比

目的：

回到 MOCHA 原始优势：全向和阿克曼统一。

对比：

- omnidirectional chassis；
- Ackermann 或 yaw-constrained chassis。

指标：

- active observation success；
- extra path length for observability；
- planning latency；
- trajectory feasibility。

预期结果：

同一个 perception-aware risk topology 可以服务不同平台，平台差异由 MOCHA 的低维动力学优化处理。

## 7. Ablation 设计

必须能单独支撑每个贡献。

推荐 ablation：

- Full P-MOCHA；
- without multi-homotopy candidates；
- without persistent risk thresholds；
- without occlusion shadow risk；
- without active observation gain；
- without visibility-aware yaw；
- risk-only RAST-like corridor；
- original MOCHA/DYNUS baseline。

最关键的两个 ablation：

1. Full method vs risk-only。
2. Persistent multi-homotopy vs single threshold/single corridor。

如果这两个对比明显，论文故事就站得住。

## 8. 推荐实现顺序

建议按这个顺序做：

1. 搭建静态遮挡和隐藏动态障碍物场景。
2. 新增 risk/uncertainty/occlusion field adapter。
3. 新增 candidate path scoring，不改优化器。
4. 生成多条 candidate guides/corridors。
5. 新增 visibility/frontier/occlusion exposure metrics。
6. 跑第一组 ablation：risk-only vs active observation candidate selection。
7. 加入 persistent multi-threshold labels。
8. 把选中的 candidates 接入 MOCHA 低维优化。
9. 如果 nonlinear optimizer 支持，再加入可微感知代价。
10. 做完整闭环重规划实验。

这个顺序的核心是：

> 先证明主动观测有行为收益，再投入复杂优化器改造。

## 9. 近期代码任务

### 9.1 第一个里程碑：轨迹感知评分器

新增一个独立模块：

```cpp
struct TrajectoryPerceptionScore {
  double risk = 0.0;
  double uncertainty = 0.0;
  double occlusion_shadow = 0.0;
  double visibility_gain = 0.0;
  double total = 0.0;
};
```

输入：

```text
sampled path or PieceWisePol
current octomap
dynamic obstacle predictions
frontier/occlusion points
sensor model
```

输出：

```text
score and debug metrics
```

这一步的目标不是优化，而是评估：

> 给定一条轨迹，它的风险、不确定性、遮挡暴露能力和主动观测收益是多少？

### 9.2 第二个里程碑：多同伦候选包装

定义：

```cpp
struct PerceptionHomotopyCandidate {
  vec_Vecf<3> path;
  std::vector<LinearConstraint3D> corridor;
  TrajectoryPerceptionScore score;
  std::string risk_signature;
  std::string visibility_signature;
};
```

这一步的目标是把多条路径作为一等对象，而不是只保留一条 `global_path`。

### 9.3 第三个里程碑：候选选择器

修改全局规划选择逻辑：

```text
generate K candidates -> score each -> choose best -> pass to existing local optimizer
```

这一步完成后，即使不改 MOCHA/Gurobi，也可以做第一版实验。

## 10. 论文贡献草案

### 贡献 1：Persistent Perception-Risk Homotopy

从动态风险和感知不确定性构建多阈值风险拓扑签名，在感知噪声下生成稳定的多同伦 guide。

解决问题：

> 单风险阈值和单 corridor 容易因为感知噪声发生拓扑抖动。

### 贡献 2：Occlusion-Aware Active Observation Objective

将遮挡未知区域建模为未来交互风险，并定义 visibility/information gain 代价，使机器人更早暴露 safety-critical uncertainty。

解决问题：

> 传统动态避障主要躲已知或已预测障碍物，不主动减少遮挡诱导的不确定性。

### 贡献 3：Low-Dimensional Perception-Aware MOCHA Optimization

把风险、不确定性、遮挡和可观测性代价嵌入 MOCHA 低维轨迹参数空间，在保持实时性的同时耦合运动可行性和未来可观测性。

解决问题：

> active perception 和连续时间轨迹优化、平台动力学约束之间长期割裂。

## 11. 最大风险与 fallback

最大风险是：

> 太早追求所有东西都可微、都进优化器。

fallback 路线：

1. 第一版只做 multi-homotopy candidate generation + active observation scoring，再用 MOCHA 优化选中的 candidate。
2. 第二版把 perception cost 放进 MOCHA nonlinear optimizer。
3. 第三版做完整 closed-loop persistent topology update 和 high-frequency low-dimensional re-optimization。

这样即使优化器集成比较慢，也能先得到可发表的系统原型和实验结果。

## 12. 当前最推荐的下一步

下一步不要直接改 MOCHA，也不要先写复杂可微 visibility。

应该先做：

> 多同伦候选路径生成 + 主动观测评分 + 遮挡动态障碍实验。

第一组最有说服力的结果应该展示：

- 至少两条可行 homotopy path；
- 最短或 risk-only path 的未来可见性更差；
- active observation path 能更早看到隐藏风险；
- hidden dynamic obstacle 场景下 collision/emergency-replan rate 降低。

只要这个结果成立，后面的数学故事和代码路线都会非常清楚。

## 13. 最终一句话

P-MOCHA 的本质不是：

```text
用 RAST 给 MOCHA 做前端
```

而是：

```text
用感知不确定性生成多同伦风险拓扑，再用 MOCHA 低维优化主动选择既安全又更可观测的轨迹
```

这条路线最能同时继承：

- RAST 的动态风险思想；
- MOCHA 的低维实时优化优势；
- 感知-规划交叉中的主动观测创新。
