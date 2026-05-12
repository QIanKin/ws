# 新论文核心

不要做：
RAST corridor + MOCHA optimizer

要做：
RAST risk map → 多同伦风险拓扑 → MOCHA 可微风险优化 → 主动感知重规划

---

# 需要接入 RAST 的点

1. 时空风险图
   R(x,t)

2. 不确定性图
   U(x,t)

3. 风险走廊思想
   low-risk spatio-temporal corridor

---

# 需要改的点

## 1. 单路径 corridor → 多同伦 risk corridor

RAST:
一条参考路径 → 一个 corridor

你:
风险图 → 拓扑图 → K 条同伦不同 guide → K 个 risk corridor

解决：
避免单路径选错拓扑、陷入局部风险区域。

---

## 2. 单风险阈值 → 持久风险拓扑

RAST:
R(x,t) > ρ 就当障碍物

你:
使用多个风险阈值：
ρ1, ρ2, ..., ρL

每个阈值下计算 homotopy signature：
H(γ) = [hρ1(γ), hρ2(γ), ..., hρL(γ)]

解决：
降低感知噪声导致的拓扑不稳定。

---

## 3. corridor 硬约束 → MOCHA 可微风险代价

RAST:
corridor constraint + QP

你:
把风险直接放进 MOCHA 目标函数：

J = Jsmooth + Jtime + Jdyn + Jrisk + Junc + Jcorridor

其中：

Jrisk = ∫ φ(R(p(t),t) - Rsafe) dt

Junc = ∫ U(p(t),t) dt

解决：
不是简单限制在 corridor 内，而是在低维变量 λ, τ 上连续优化风险、时间、平滑性和动力学约束。

---

## 4. 被动避障 → 主动感知

加入可见性代价：

Jview = ∫∫ U(x,t) · 1[x 不在传感器视野内] dxdt

解决：
有限 FOV、遮挡、高动态障碍物突然出现的问题。

---

## 5. 每帧重建 corridor → 双层重规划

低频：
更新风险拓扑和多同伦 guide

高频：
MOCHA 在当前拓扑类内快速优化 λ, τ

解决：
兼顾拓扑鲁棒性和实时性。

---

# 最终贡献

1. Persistent Risk Homotopy
   用多风险阈值生成更稳定的多同伦 guide。

2. Risk-Aware MOCHA Optimization
   把 RAST 的风险、不确定性、corridor 转成 MOCHA 的可微代价。

3. Perception-Active Replanning
   让轨迹主动选择能降低未来不确定性的路径和视角。

---

# 一句话

不是把 RAST 的 QP 换成 MOCHA，
而是把 RAST 的单路径风险走廊升级成：
多同伦 + 持久风险拓扑 + 可微低维优化 + 主动感知。