
# relax_stepping：类继承关系与适用范围速查

对应实现文件：

- `src/shared/particle_dynamics/relax_dynamics/relax_stepping.h`
- `src/shared/particle_dynamics/relax_dynamics/relax_stepping.hpp`
- `src/shared/particle_dynamics/relax_dynamics/relax_stepping.cpp`

本模块的目标：在“粒子松弛（particle relaxation）”过程中，通过迭代更新粒子位置，使分布满足**零阶一致性**并贴合几何/边界。

---

## 1. 核心数据（变量注册/读写）

`RelaxationResidual<Base, ...>` 在构造中绑定/注册了关键粒子变量（见 `.hpp`）：

- 读取：`Position`、`VolumetricMeasure`
- 注册写入：`ZeroOrderResidual`（零阶残差，Vecd）、`ParticleKineticEnergy`（Real）
- 依赖：`SPHAdaptation`、`Kernel`

**注意**：`VolumetricMeasure` 表示粒子体积度量（在 2D/3D 通常是面积/体积）；`ZeroOrderResidual` 是本模块中所有“位移更新”的驱动力。

---

## 2. 类/模板总体结构

该文件的主要结构可以分为三层：

1) **残差计算器**：`RelaxationResidual<...>` 的不同特化（Inner/Contact/LevelSet/Implicit）。

2) **单步推进器（Step）**：`RelaxationStep<TResidual>` / `RelaxationStepImplicit<TResidual>`，组织一次松弛迭代包含的子步骤。

3) **辅助 Dynamics**：`RelaxationScaling`、`PositionRelaxation`、`UpdateSmoothingLengthRatioByShape`。

另外 `Explicit` / `Implicit` 在本文件中作为**标签类型**（tag type）存在：

- `Inner<Implicit>`、`Inner<LevelSetCorrection, Implicit>` 通过 `Implicit` 标签选择“隐式版本残差/更新”。

---

## 3. 继承关系图（从基类到具体特化）

### 3.1 RelaxationResidual 族

```
RelaxationResidual<Base, DataDelegationType>
	: LocalDynamics
	+ DataDelegationType            (DataDelegateInner / DataDelegateContact)
			|
			+-- RelaxationResidual<Inner<>>
			|     : RelaxationResidual<Base, DataDelegateInner>
			|
			+-- RelaxationResidual<Inner<LevelSetCorrection>>
			|     : RelaxationResidual<Inner<>>
			|
			+-- RelaxationResidual<Inner<Implicit>>
			|     : RelaxationResidual<Inner<>>
			|
			+-- RelaxationResidual<Inner<LevelSetCorrection, Implicit>>
			|     : RelaxationResidual<Inner<Implicit>>
			|
			+-- RelaxationResidual<Contact<>>
						: RelaxationResidual<Base, DataDelegateContact>
```

### 3.2 Step 族

```
RelaxationStep<RelaxationResidualType>
	: BaseDynamics<void>
	- InteractionDynamics<RelaxationResidualType> relaxation_residual_
	- ReduceDynamics<RelaxationScaling>           relaxation_scaling_
	- SimpleDynamics<PositionRelaxation>         position_relaxation_
	- SimpleDynamics<ShapeSurfaceBounding>       surface_bounding_

RelaxationStepImplicit<RelaxationResidualType>
	: BaseDynamics<void>
	- InteractionSplit<RelaxationResidualType>   relaxation_residual_
	- ReduceDynamics<RelaxationScaling>          relaxation_scaling_
	- SimpleDynamics<PositionRelaxation>         position_relaxation_ (成员存在，但 exec 中不调用)
	- SimpleDynamics<ShapeSurfaceBounding>       surface_bounding_
```

**关键差异**：

- `RelaxationStep`（显式风格）先 `relaxation_residual_.exec()` 再用 `PositionRelaxation` 显式更新位置。
- `RelaxationStepImplicit`（隐式风格）把“位置更新”合并到 `RelaxationResidual<Inner<Implicit...>>` 内部（`updateStates` 会直接改 `pos_`），因此 `exec` 不再调用 `position_relaxation_`。

---

## 4. 各个类的职责与适用范围

### 4.1 `RelaxationResidual<Base, DataDelegationType>`（共同基类）

职责：

- 统一完成变量绑定、Kernel/Adaptation 访问、数据委托（inner/contact configuration 的访问）。
- 不直接提供 `interaction()`，由具体特化实现。

适用：所有松弛残差实现的“骨架”。

---

### 4.2 `RelaxationResidual<Inner<>>`（最基本的 inner 残差）

职责：

- 对每个粒子 i，遍历 `inner_configuration_[i]`，累加零阶残差：
	- 本质上是对邻域核梯度项的离散求和（见 `.cpp`）。
- `residual_[i]` 输出到 `ZeroOrderResidual`。
- 持有 `relax_shape_`：默认用 `sph_body_->getInitialShape()`；也可通过 `sub_shape_name` 指定 `ComplexShape` 的子形状。

适用：

- 单一 body 的“内部一致性”松弛。
- 不考虑边界 level set 修正，也不做隐式耦合更新。

---

### 4.3 `RelaxationResidual<Inner<LevelSetCorrection>>`（inner + level set 边界修正）

继承：`RelaxationResidual<Inner<>>`

职责：

- 先调用基础 `Inner<>` 残差。
- 再叠加 level set 的核梯度积分项修正：
	- `residual_[i] -= 2 * level_set_shape_.computeKernelGradientIntegral(...)`
- 通过 `DynamicCast<LevelSetShape>(..., getRelaxShape())` 获取 `level_set_shape_`。

适用：

- 当松弛目标形状是 `LevelSetShape`（或可被动态转换为 LevelSetShape）时，用于更稳健的边界/表面处理。

注意事项：

- 如果 `getRelaxShape()` 不是 `LevelSetShape`，动态转换会失败（通常意味着配置/形状选择不对）。

---

### 4.4 `RelaxationResidual<Inner<Implicit>>`（隐式版本 inner 松弛）

继承：`RelaxationResidual<Inner<>>`

职责：

- 计算一个局部线性/二次形式的误差与参数矩阵：
	- `computeErrorAndParameters()` 构造 `error_`、`a_`、`c_`。
- `updateStates()` 解一个小型线性系统并同时更新：
	- `pos_[i] += a_ * k`
	- 邻居 `pos_[j] -= b * k`
- 输出残差：`residual_[i] = -error / dt^2`，并记录 `ParticleKineticEnergy = |residual|`。

适用：

- 想要更“耦合/隐式”的位置调整（一次 interaction 会同时动到 i 和其邻居 j），减少显式更新可能带来的不稳定。
- 通常配合 `RelaxationStepImplicit` 使用（见 4.8）。

注意事项：

- 该实现包含 `parameter_l.inverse()`；当局部矩阵病态时可能数值敏感。

---

### 4.5 `RelaxationResidual<Inner<LevelSetCorrection, Implicit>>`（隐式 + level set 修正）

继承：`RelaxationResidual<Inner<Implicit>>`

职责：

- 在隐式 `computeErrorAndParameters()` 的基础上，加入 level set 的积分/梯度/二阶梯度项，并引入 `overlap`（核积分）做耦合系数：
	- `error_ += ... * (1 + overlap)`
	- `a_    -= ... * (1 + overlap)`

适用：

- 既希望隐式耦合更新，又希望边界用 level set 更准确/更稳定。

---

### 4.6 `RelaxationResidual<Contact<>>`（contact 残差）

继承：`RelaxationResidual<Base, DataDelegateContact>`

职责：

- 遍历每个接触体 k 的 contact neighborhood，累加来自外部 body 的残差贡献。
- 为每个接触体注册其 `VolumetricMeasure`（`contact_Vol_`），用于体积权重。

适用：

- 多 body 之间需要保持间距/避免互相穿插的松弛。
- 通常不会单独使用，而是与 `Inner<>` 组合成 Complex 交互（见 4.9）。

---

### 4.7 `RelaxationScaling`（步长/尺度估计）

继承：`LocalDynamicsReduce<ReduceMax>`

职责：

- `reduce()` 返回 `|residual[i]|`。
- 通过 ReduceMax 得到全局最大残差后，输出一个缩放：
	- `scaling = 0.0625 * h_ref / (maxResidual + TinyReal)`

适用：

- 显式 step：把“残差幅度”转成稳定的位移尺度。
- 隐式 step：用于计算 `sqrt(scaling)` 并截断到 0.01（见 `.hpp` 的 `RelaxationStepImplicit::exec`）。

---

### 4.8 `PositionRelaxation`（显式位置更新器）

继承：`LocalDynamics`

职责：

- 位置显式更新：
	- `pos[i] += residual[i] * scaling * 0.5 / SmoothingLengthRatio(i)`

适用：

- 仅在 `RelaxationStep`（显式风格）中使用。
- 隐式步中通常不使用（因为隐式 residual 已直接更新 pos）。

---

### 4.9 `UpdateSmoothingLengthRatioByShape`（按形状自适应粒子尺度/体积）

继承：`LocalDynamics`

职责：

- 根据 `AdaptiveByShape::getLocalSpacing(target_shape, pos[i])` 更新：
	- `SmoothingLengthRatio = reference_spacing / local_spacing`
	- `VolumetricMeasure = local_spacing^Dimensions`

适用：

- 当目标形状不同区域需要不同粒子分辨率（局部加密/稀疏）时。
- 它不在 `RelaxationStep` 内部自动调用，通常由上层流程在松弛迭代前/迭代中按需调用。

---

## 5. Step 执行流程与“什么时候用哪种 Step”

### 5.1 `RelaxationStep<TResidual>`（显式松弛一步）

执行流程（`.hpp`）：

1. `real_body_.updateCellLinkedList()`
2. 对所有 body relation：`updateConfiguration()`
3. `relaxation_residual_.exec()`（计算 `ZeroOrderResidual`）
4. `scaling = relaxation_scaling_.exec()`（取 max |residual| 计算尺度）
5. `position_relaxation_.exec(scaling)`（显式更新 `pos`）
6. `surface_bounding_.exec()`（几何边界约束/投影/限制）

适用：

- 想要“逻辑简单、行为直观”的松弛迭代。
- 通常作为默认选择；若发现显式更新收敛慢/不稳，再考虑隐式。

### 5.2 `RelaxationStepImplicit<TResidual>`（隐式松弛一步）

执行流程（`.hpp`）：

1. `scaling = min(sqrt(relaxation_scaling_.exec()), 0.01)`
2. `relaxation_residual_.exec(scaling)`（隐式 residual 内部会直接改 `pos`）
3. `real_body_.updateCellLinkedList()`
4. 对所有 body relation：`updateConfiguration()`
5. `surface_bounding_.exec()`

适用：

- 使用 `RelaxationResidual<Inner<Implicit...>>` 时的推荐 step。
- 希望每步更新更“耦合”，减少显式算法可能出现的局部振荡。

---

## 6. 常用类型别名（可直接选用的组合）

本文件尾部提供了常用别名：

- `RelaxationStepInner`：`RelaxationStep<RelaxationResidual<Inner<>>>`
	- 单 body、无 level set、显式。

- `RelaxationStepLevelSetCorrectionInner`：`RelaxationStep<RelaxationResidual<Inner<LevelSetCorrection>>>`
	- 单 body、有 level set 边界修正、显式。

- `RelaxationStepComplex`：`RelaxationStep<ComplexInteraction<RelaxationResidual<Inner<>, Contact<>>>>`
	- inner + contact，多 body 接触参与、显式。

- `RelaxationStepLevelSetCorrectionComplex`：`RelaxationStep<ComplexInteraction<RelaxationResidual<Inner<LevelSetCorrection>, Contact<>>>>`
	- inner（带 level set） + contact，多 body、显式。

- `RelaxationStepInnerImplicit`：`RelaxationStepImplicit<RelaxationResidual<Inner<Implicit>>>`
	- 单 body、隐式。

- `RelaxationStepLevelSetCorrectionInnerImplicit`：`RelaxationStepImplicit<RelaxationResidual<Inner<LevelSetCorrection, Implicit>>>`
	- 单 body、隐式 + level set 修正。

---

## 7. 选型建议（经验规则）

- 只做单体几何贴合、快速得到均匀分布：优先 `RelaxationStepInner`。
- 边界附近出现“贴不住/穿透/分布失真”：用 `RelaxationStepLevelSetCorrectionInner`（前提：shape 是 LevelSet）。
- 多 body 之间需要避免互相侵入：用 `RelaxationStepComplex` 或其 level set 版本。
- 显式更新出现不稳定或收敛慢：尝试 `RelaxationStepInnerImplicit`（或带 level set 的隐式版本）。

圆柱壁面松弛收敛速度：`RelaxationStepLevelSetCorrectionInnerImplicit` > `RelaxationStepLevelSetCorrectionInner` > `RelaxationStepInnerImplicit` > `RelaxationStepInner`。
