本文基于以下实现文件整理：
- `src/shared/particle_dynamics/solid_dynamics/constraint_dynamics.h`
- `src/shared/particle_dynamics/solid_dynamics/constraint_dynamics.cpp`
- `src/shared/particle_dynamics/solid_dynamics/constraint_dynamics.hpp`

目标：总结各类的使用场景、用法和数学物理原理。

# 1. 总体定位

`constraint_dynamics` 提供的是“约束/驱动型”动力学操作，核心特点是：

- 直接修改粒子位置、速度或加速度（强约束）。
- 常用于边界条件、刚体驱动、与多体动力学（SimBody）耦合。
- 不同类的共同入口通常是 `setupDynamics()` 和 `update(index_i, dt)`。

这些类大多继承自 `MotionConstraint<...>`（或约简基类），运行时会对目标粒子集逐粒子执行 `update`。

# 2. 各类详解

## 2.1 `SpringConstrain`

### 使用场景
- 希望某个 `BodyPartByParticle` 区域通过“弹簧”拉回初始位置。
- 适合做软约束、缓和固定，而不是刚性固定。

### 用法
- 构造：`SpringConstrain(body_part, stiffness)`。
- `stiffness` 是标量，会扩展到各坐标方向。
- 每步 `update` 中根据位移修正速度。

### 数学物理原理
对第 `i` 个粒子，位移：
$$
\Delta x_i = x_i - x_{i0}
$$
类中实际实现为加速度：
$$
a_i = -\frac{K\,\Delta x_i}{m_i}
$$

其中 `K` 为对角刚度（各方向），`m_i` 是粒子质量。

速度显式更新：
$$
v_i \leftarrow v_i + dt\,a_i
$$

本质是离散的胡克定律恢复力 $F=-Kx$，再除以质量得到加速度。

### 注意点
- 只改速度，不直接改位置。
- 无显式阻尼项，刚度过大时可能引入振荡，需要结合时间步控制。

---

## 2.2 `PositionSolidBody`

### 使用场景
- 在时间区间 `[start_time, end_time]` 内，把整个刚体从初始中心移动到给定目标中心。
- 典型于“准静态位移加载”边界条件。
- 注释明确指出：不用于弹性体形变求解本体。

### 用法
- 构造：`PositionSolidBody(sph_body, start_time, end_time, pos_end_center)`。
- 初始化时由包围盒中心得到初始中心 `pos_0_center_`，并计算整体平移量 `translation_`。
- 在时间窗口内每个粒子执行位移推进，并将速度置零。

### 数学物理原理
单粒子目标位置：
$$
x_i^{\mathrm{final}} = x_{i0} + \mathrm{translation}
$$
每步位移采用剩余时间归一化：
$$
\Delta x_i = \frac{\bigl(x_i^{\mathrm{final}} - x_i\bigr)\,dt}{\mathrm{end\_time} - t}
$$
更新：
$$
x_i \leftarrow x_i + \Delta x_i,\qquad v_i \leftarrow 0
$$

该写法等价于让粒子在剩余时间内“匀速追踪”目标点，保证在 $t\to\mathrm{end\_time}$ 时收敛到目标（在数值稳定前提下）。

### 注意点
- 时间窗口外不生效。
- 每步清零速度，意味着此驱动优先级高于自由动力学速度演化。

---

## 2.3 `PositionScaleSolidBody`

### 使用场景
- 在给定时间内，以初始中心为参考对整体进行尺度缩放（放大/缩小）。
- 也是准静态的几何驱动边界条件。

### 用法
- 构造：`PositionScaleSolidBody(sph_body, start_time, end_time, end_scale)`。
- 初始中心 `pos_0_center_` 由包围盒中心确定。
- 每步把粒子推进到缩放目标，并清零速度。

### 数学物理原理
目标位置：
$$
x_i^{\mathrm{final}} = c_0 + s_{\mathrm{end}}\bigl(x_{i0} - c_0\bigr)
$$
其中 `c0` 为初始中心，`s_end` 为目标尺度。
单步推进：
$$
\Delta x_i = \frac{\bigl(x_i^{\mathrm{final}} - x_i\bigr)\,dt}{\mathrm{end\_time} - t}
$$
更新：
$$
x_i \leftarrow x_i + \Delta x_i,\qquad v_i \leftarrow 0
$$

本质上是对参考构型的各粒子做同尺度仿射映射，并在时间域平滑施加。

### 注意点
- 与 `PositionSolidBody` 一样属于强制运动学驱动。
- 仅适合“按预设几何轨迹走”，不反映真实受力后的自然形变路径。

---

## 2.4 `PositionTranslate<DynamicsIdentifier>`

别名：`TranslateSolidBody`（对整刚体）、`TranslateSolidBodyPart`（对粒子子域）
### 使用场景
- 给定总平移向量 `translation`，在指定时间窗内完成平移。
- 与 `PositionSolidBody` 相比，这里直接给“位移向量”，不依赖目标中心。

### 用法
- 构造：`PositionTranslate(identifier, start_time, end_time, translation)`。
- 时间窗内更新位置，速度清零。

### 数学物理原理
目标位置同样是：
$$
x_i^{\mathrm{final}} = x_{i0} + \mathrm{translation}
$$
单步位移：
$$
\Delta x_i = \frac{\bigl(x_i^{\mathrm{final}} - x_i\bigr)\,dt}{\mathrm{end\_time} - t}
$$
代码里实际用了：
$$
x_i \leftarrow x_i + 0.5\,\Delta x_i
$$

这里 `0.5` 的注释是“因为该操作会执行两次”，所以单次只施加一半，避免重复推进导致过冲。

### 注意点
- 若你的调度流程没有“双执行”，这个 `0.5` 会导致总位移不足，需要检查调用链。
- 模板参数可用于 body 或 body part，适配性比 `PositionSolidBody` 更好。

---

## 2.5 `ConstrainSolidBodyMassCenter`

### 使用场景
- 约束刚体整体质心平动速度，常用于去除刚体漂移或固定某些方向的整体运动。
- 例如只允许 x 方向漂移、抑制 y/z 方向净动量。

### 用法
- 构造：`ConstrainSolidBodyMassCenter(sph_body, constrain_direction)`。
- `constrain_direction` 是分量掩码，默认全 1（各方向都约束）。
- `setupDynamics` 中计算整体动量对应的速度修正；`update` 对每个粒子减去该修正。

### 数学物理原理
总质量：
$$
M = \sum_i m_i
$$
总动量（由 `QuantityMoment<Vecd, SPHBody>("Velocity")` 计算）：
$$
P = \sum_i m_i v_i
$$
质心速度：
$$
V_{\mathrm{cm}} = \frac{P}{M}
$$
方向掩码矩阵 $C=\mathrm{diag}(\mathrm{constrain\_direction})$，修正速度：
$$
C = \mathrm{diag}(\mathrm{constrain\_direction}),\qquad V_{\mathrm{corr}} = C\,V_{\mathrm{cm}}
$$
对所有粒子：
$$
v_i \leftarrow v_i - V_{\mathrm{corr}}
$$

这会把被约束方向上的整体平动模式从速度场中投影掉。

### 注意点
- 该操作保持相对速度分布，但移除整体平动分量。
- 若与外载荷并用，可能改变系统真实动量守恒路径，需要明确物理意图。

---

## 2.6 `ConstraintBySimBody<DynamicsIdentifier>`

别名：`ConstraintBodyBySimBody`、`ConstraintBodyPartBySimBody`
### 使用场景
- SPH 粒子（整体或局部）严格跟随 SimBody 多体系统里某个 mobilized body 的位姿与运动学量。
- 用于 SPH 与刚体/机构的强耦合运动学约束。

### 用法
- 构造需要四个对象：
	`ConstraintBySimBody(identifier, MBsystem, mobod, integ)`。
- 每步在 `setupDynamics` 中从积分器拿状态并 `realize` 到 `Acceleration` 阶段。
- 对每个粒子，在 `update` 中把其“初始局部站点”映射到地面系，写回：
	位置、速度、加速度、法向。

### 数学物理原理
令粒子初始位置 `x_i0` 作为刚体局部坐标下站点（station）。

对刚体当前状态 `(R(t), r_o(t), omega(t), alpha(t), v_o(t), a_o(t))`：
$$
x_i = r_o + R\,x_{i0}
$$
$$
v_i = v_o + \omega \times (R\,x_{i0})
$$
$$
a_i = a_o + \alpha \times (R\,x_{i0}) + \omega \times \bigl(\omega \times (R\,x_{i0})\bigr)
$$
法向也按刚体旋转更新（代码中通过 `findStationLocationVelocityAndAccelerationInGround` 一并获得）。

这是一种典型刚体运动学映射，确保受约束粒子与 SimBody 描述完全一致。

### 注意点
- 这是强约束，不是柔性耦合。
- 正确性依赖 SimBody 状态已推进且 `realize` 到足够阶段。

---

## 2.7 `TotalForceForSimBody<DynamicsIdentifier>`

别名：`TotalForceOnBodyForSimBody`、`TotalForceOnBodyPartForSimBody`
### 使用场景
- 将 SPH 侧粒子受力汇总为可施加给 SimBody 的合力-合力矩（wrench / spatial force）。
- 常与 `ConstraintBySimBody` 配套，形成双向耦合：
	SimBody 给运动学约束，SPH 回传动力学反力。

### 用法
- 构造：`TotalForceForSimBody(identifier, MBsystem, mobod, integ)`。
- 归约前在 `setupDynamics` 读取当前刚体原点位置。
- `reduce` 对每个粒子返回其对刚体原点贡献的 `SpatialVec`，再求和。

### 数学物理原理
粒子总力：
$$
f_i = \mathrm{force}_i + \mathrm{force\_prior}_i
$$
相对刚体原点位矢：
$$
r_i = x_i - x_{\mathrm{origin}}
$$
力矩：
$$
τ_i = r_i \times f_i
$$
归约输出：
$$
\mathrm{SpatialVec}(\tau_i, f_i)
$$
总和后得到：
$$
τ = \sum_i τ_i,\qquad F = \sum_i f_i
$$

这是从离散粒子力到刚体广义外力的标准映射。

### 注意点
- `force_prior` 也被计入总力，表示历史/先验力项会影响反馈给 SimBody 的结果。
- 力矩参考点是当前 mobilized body 原点，改参考点会改变力矩值。

# 3. 典型组合方式

## 3.1 纯运动学驱动（不与 SimBody 耦合）
- 小变形/准静态边界加载可选：
	`PositionSolidBody`、`PositionScaleSolidBody`、`PositionTranslate`。
- 若要软回弹可用 `SpringConstrain`。

## 3.2 与 SimBody 双向耦合
- 运动学约束：`ConstraintBySimBody`（SimBody -> SPH）。
- 动力学回传：`TotalForceForSimBody`（SPH -> SimBody）。

## 3.3 去整体漂移
- 使用 `ConstrainSolidBodyMassCenter` 去除指定方向质心平动速度。

# 4. 工程使用建议

- 时间窗类约束都依赖 `PhysicalTime`，确保系统时间变量正确注册并推进。
- 多个约束并用时，注意执行顺序：
	先强制位置再改速度，或先动力学再投影约束，会导致不同结果。
- `PositionTranslate` 的 `0.5` 与调度策略绑定，改动前先确认调用频次。
- 若关注真实动力响应，避免长期使用“每步速度清零”的强运动学驱动。
