类：SPHAdaptation（及其局部加密派生类）

> 位置：`src/shared/adaptations/adaptation.h(.hpp/.cpp)`

# 概览

`SPHAdaptation` 是 SPHinXsys 中“数值分辨率/自适应策略”的基类（per-body）：

- 统一定义粒子分辨率相关的**全局标尺**：参考粒子间距 `spacing_ref_`、参考光滑长度 `h_ref_`、最小间距 `spacing_min_` 等。
- 持有并管理核函数 `Kernel`（默认 `KernelWendlandC2`），并提供与核相关的派生量：截止半径、晶格参考数密度 `sigma0_ref_`。
- 为网格相关结构提供创建入口：`createCellLinkedList()`（单层/多层）与 `createLevelSet()`（单层/多层）。
- 对“局部加密”的场景，派生类 `ParticleWithLocalRefinement` 在 `BaseParticles` 上注册自适应变量（如 `SmoothingLengthRatio`、`ParticleMeshLevel`），并驱动**多层 cell linked list** 与**自适应邻域搜索**。

这套设计的核心思想是：

1. **body 级别**持有一个 `SPHAdaptation` 对象（见 `SPHBody::sph_adaptation_`），负责“该 body 的分辨率策略”。
2. **粒子级别**（可选）通过在 `BaseParticles` 注册变量来表达局部变化，并在邻域搜索/网格结构中使用这些变量。

# 关键约定与尺度定义（重要）

## 1) 分辨率引用关系：system resolution vs body resolution

构造参数里会出现 `system_refinement_ratio`：

- `resolution_ref`：系统参考分辨率（通常来自 `SPHSystem::ReferenceResolution()`）。
- `system_refinement_ratio_`：系统分辨率相对 body 分辨率的比例。
- `spacing_ref_ = resolution_ref / system_refinement_ratio_`：该 body 的参考粒子间距。

直观理解：`system_refinement_ratio_ > 1` 时，body 的粒子间距更小（更细），反之更粗。

## 2) 光滑长度与间距的比值

`h_spacing_ratio_` 定义“参考光滑长度与粒子间距的比值”：

- `h_ref_ = h_spacing_ratio_ * spacing_ref_`

这会影响：

- 核函数 `Kernel` 的光滑长度与截止半径（`CutOffRadius()`）。
- 参考晶格数密度 `sigma0_ref_`（通过 `computeLatticeNumberDensity()` 对规则晶格求和得到）。

## 3) 局部加密与最小间距

在基类中 `local_refinement_level_` 默认是 0，因此：

- `spacing_min_ = spacing_ref_ / 2^{local_refinement_level_}`（默认等于 `spacing_ref_`）
- `h_ratio_max_ = spacing_ref_ / spacing_min_`（默认 1）
- `MinimumSmoothingLength() = h_ref_ / h_ratio_max_`

当使用 `ParticleWithLocalRefinement` 时，`local_refinement_level_` 由构造参数指定，从而确定系统允许的“最细粒子”尺度。

# 类与继承结构

本文件中与 adaptation 相关的类层次：

- `SPHAdaptation`：单分辨率基类（也作为默认实现）。
- `ParticleWithLocalRefinement : SPHAdaptation`：为粒子注册局部变量，支持多层 cell linked list / level set。
- `ParticleRefinementByShape : ParticleWithLocalRefinement`：抽象类，按距离/形状度量给出局部 spacing。
	- `ParticleRefinementNearSurface`：靠近 surface 更细。
	- `ParticleRefinementWithinShape`：shape 内更细，外部平滑过渡。

# 成员变量

以下按 `SPHAdaptation` 与 `ParticleWithLocalRefinement` 分别说明。

## 1) SPHAdaptation（核心尺度 + 核函数）

| 变量名 | 含义 |
| --- | --- |
| `h_spacing_ratio_` | 参考核光滑长度与粒子间距的比值 $$h/\Delta x$$（默认 1.3）。 |
| `system_refinement_ratio_` | system resolution 相对 body resolution 的比例（默认 1.0）。 |
| `local_refinement_level_` | 局部加密层级（默认 0）。 |
| `spacing_ref_` | body 的参考粒子间距：`resolution_ref / system_refinement_ratio_`。 |
| `h_ref_` | 参考核光滑长度：`h_spacing_ratio_ * spacing_ref_`。 |
| `kernel_ptr_` | 核函数对象的唯一指针（默认 `KernelWendlandC2(h_ref_)`）。 |
| `sigma0_ref_` | 参考晶格数密度（由核与 `spacing_ref_` 决定）。用公式描述是$$\sum_j W_{ij}^{0}$$，其中$$W_{ij}^{0}$$为初始时刻（严格来说是精确按照`Lattice`排布）的核函数值。 |
| `spacing_min_` | 最小粒子间距（由 `local_refinement_level_` 决定）。 |
| `Vol_min_` | 最小体积度量：`pow(spacing_min_, Dimensions)`。 |
| `h_ratio_max_` | `spacing_ref_/spacing_min_`，也可理解为“参考光滑长度相对最小光滑长度”的比例上界。 |

## 2) ParticleWithLocalRefinement（粒子级自适应变量句柄）

| 变量名 | 含义 |
| --- | --- |
| `Real *h_ratio_` | 每粒子的 `SmoothingLengthRatio` 数据指针：$$h_\mathrm{ref}/h_i$$（注意：此处语义是“参考光滑长度相对当前光滑长度的比例”）。 |
| `int *level_` | 每粒子的 `ParticleMeshLevel`：其应落在哪一层 cell linked list 网格。 |
| `finest_spacing_bound_` | 最细粒子间距的界（`spacing_min_ + Eps`）。 |
| `coarsest_spacing_bound_` | 最粗粒子间距的界（`spacing_ref_ - Eps`）。 |

# 成员函数

## 1) 构造与基础访问器（SPHAdaptation）

| 函数名 | 含义 |
| --- | --- |
| `SPHAdaptation(resolution_ref, h_spacing_ratio, system_refinement_ratio)` | 计算 `spacing_ref_ / h_ref_`，构造默认核 `KernelWendlandC2`，并计算 `sigma0_ref_`、`spacing_min_` 等。 |
| `SPHAdaptation(SPHSystem&, ...)` | 便捷构造：使用 `sph_system.ReferenceResolution()` 作为 `resolution_ref`。 |
| `LocalRefinementLevel()` | 返回 `local_refinement_level_`。 |
| `ReferenceSpacing()` / `MinimumSpacing()` | 参考/最小粒子间距。 |
| `ReferenceSmoothingLength()` / `MinimumSmoothingLength()` | 参考/最小核光滑长度。 |
| `getKernel()` | 返回核函数指针。 |
| `LatticeNumberDensity()` | 返回 `sigma0_ref_`。 |

### 关于 `sigma0_ref_`：computeLatticeNumberDensity

`computeLatticeNumberDensity(Vec2d/Vec3d)` 会在规则晶格点上遍历一个“覆盖 cutoff radius 的立方/方形窗口”，对落在 cutoff 内的点累加 `kernel_ptr_->W(distance, particle_location)`。

它是一个 **reference normalization** 量，常用于把离散数密度与连续数密度的尺度对应起来。

## 2) 自适应比率与核重置

| 函数名 | 含义 |
| --- | --- |
| `resetAdaptationRatios(h_spacing_ratio, new_system_refinement_ratio)` | 重设 `h_spacing_ratio_ / system_refinement_ratio_` 并同步更新 `spacing_ref_ / h_ref_ / kernel smoothing length / sigma0_ref_ / spacing_min_ / Vol_min_ / h_ratio_max_`。 |
| `resetKernel<KernelType>(args...)` | 重新创建核函数对象并刷新 `sigma0_ref_`。 |
| `NumberDensityScaleFactor(smoothing_length_ratio)` | 返回 `pow(smoothing_length_ratio, Dimensions)`；用于随光滑长度缩放时的数密度尺度因子。 |
| `SmoothingLengthRatio(i)` | 基类默认返回 1.0；派生类可返回粒子局部比率。 |
| `SmoothingLengthByLevel(level)` | 返回 `h_ref_ / 2^{level}`；用于多层网格/level set 的尺度构造。 |

## 3) 适配变量初始化：initializeAdaptationVariables

| 函数名 | 含义 |
| --- | --- |
| `initializeAdaptationVariables(BaseParticles&)` | 基类为空实现；为派生类提供 hook：在粒子生成后注册/初始化自适应变量。 |

该函数的调用链在 `SPHBody::generateParticles()` 中：

- 粒子几何生成完成
- `particles->initializeBasicParticleVariables()`
- `sph_adaptation_->initializeAdaptationVariables(*particles)`

因此它是“每个 body 的 particle variables”与 adaptation 绑定的关键时机。

## 4) 网格/邻域结构创建

| 函数名 | 含义 |
| --- | --- |
| `createCellLinkedList(domain_bounds, base_particles)` | 默认创建单层 `CellLinkedList(domain_bounds, kernel.CutOffRadius(), ...)`。 |
| `createRefinedCellLinkedList(level, ...)` | 创建“更细”的 cell linked list：grid spacing = `CutOffRadius()/2^{level}`。 |
| `createLevelSet(shape, refinement_ratio)` | 默认估算 mesh levels，先构造一个“coarser”的 `MultilevelLevelSet` 来拿到最细层 spacing，再返回最细层的 `MultilevelLevelSet`。 |
| `createBackGroundMesh<MeshType>(sph_body, args...)` | 背景网格构造：domain = system bounds，grid spacing = `kernel.CutOffRadius()`，buffer = 2。 |

### 说明：createLevelSet的level估算

实现中使用：

`total_levels = (int)log10(MinimumDimension(bounds)/ReferenceSpacing()) + 2`

这不是 $$\log_2$$，而是 $$\log_{10}$$，它更像一个经验估算（与 cell linked list 的层数并不严格对应）。理解时以“给 level set 足够层级覆盖从 coarse 到 fine”即可。

## 5) 局部加密派生类：ParticleWithLocalRefinement

### 5.1 initializeAdaptationVariables

`ParticleWithLocalRefinement::initializeAdaptationVariables(BaseParticles&)` 会：

1. 调用基类 hook。
2. 注册 state variable：
	 - `Real SmoothingLengthRatio`（数据指针 `h_ratio_`）
	 - `int ParticleMeshLevel`（数据指针 `level_`）
3. 将 `SmoothingLengthRatio` 加入 evolving variables（即参与 restart/reload、以及粒子交换/拷贝等）。

其中 `SmoothingLengthRatio` 的默认初始化采用 lambda：

`ReferenceSpacing() / base_particles.ParticleSpacing(i)`

而 `ParticleSpacing(i)` 在 `BaseParticles` 中默认来自体积度量：

`ParticleSpacing = pow(Vol_[i], 1/Dimensions)`

因此：如果你通过几何生成/重载改变了 `Vol_`，`SmoothingLengthRatio` 的初值会随之改变。

### 5.2 createCellLinkedList / createLevelSet

该类会创建多层结构：

- `createCellLinkedList()` -> `MultilevelCellLinkedList(..., total_levels = local_refinement_level + 1, ...)`
- `createLevelSet()` -> `MultilevelLevelSet(..., total_levels = local_refinement_level + 1, ...)`

### 5.3 ParticleMeshLevel 的用途

`MultilevelCellLinkedList` 在插入粒子时，会根据每粒子的 cutoff radius 选择网格层级，并写入 `level_[i]`。

此外，邻域搜索里有“split inner adaptive”的建邻策略，会利用 `ParticleMeshLevel` 做去重/方向性约束（只对某一侧建立邻居）。

## 6) 按 shape 距离决定 spacing：ParticleRefinementByShape

| 函数名 | 含义 |
| --- | --- |
| `getLocalSpacing(shape, position)` | 纯虚函数：由具体策略给出 position 的目标 spacing。 |
| `smoothedSpacing(measure, transition_thickness)` | 将 measure（如到 surface 的距离）通过核函数 1D 权重平滑映射到 `[finest_spacing_bound_, coarsest_spacing_bound_]`。 |

### 6.1 ParticleRefinementNearSurface

- `phi = abs(shape.findSignedDistance(position))`
- `getLocalSpacing = smoothedSpacing(phi, spacing_ref_)`

即：越靠近 surface（phi 越小），spacing 越接近最细；远离 surface 平滑过渡到最粗。

### 6.2 ParticleRefinementWithinShape

- `phi = shape.findSignedDistance(position)`
- 若 `phi < 0`（shape 内部）：直接最细 `finest_spacing_bound_`
- 否则（shape 外部）：`smoothedSpacing(phi, 2*spacing_ref_)`

即：shape 内强制最细，shape 外用更厚的过渡带平滑变粗。

# 变量系统的关联：adaptation 与 BaseParticles

adaptation 本身不直接“持有粒子数组”，但它通过 `initializeAdaptationVariables()` 将关键变量注册到 `BaseParticles`，并在后续模块中被使用。

典型变量名：

- `SmoothingLengthRatio`：被邻域搜索（`NeighborBuilderInnerAdaptive` 等）读取，用于：
	- cutoff radius：`kernel.CutOffRadius(h_ratio_min)`
	- 权重：`kernel.W(i_h_ratio, ...)` 与导数 `kernel.dW(h_ratio_min, ...)`
- `ParticleMeshLevel`：被多层 cell linked list 与“split inner adaptive”建邻读取。

# 注意事项 / 可能的“坑”

1. **`AdaptiveSmoothingLength(BaseParticles&)` 仅在头文件声明**：在当前仓库版本中未找到其实现（全仓库搜索只有声明）。因此文档中不建议依赖它；若你在上游版本见到实现，应以对应版本源码为准。
2. **局部 refinement 需要粒子变量存在**：若某些模块假定存在 `SmoothingLengthRatio`（例如自适应邻域搜索），则必须使用 `ParticleWithLocalRefinement` 或手动注册同名变量，否则 `getVariableDataByName` 会直接报错退出。
3. **`SmoothingLengthRatio` 的语义要统一**：该变量在代码中被当作 $$h_\mathrm{ref}/h_i$$ 使用；同时 cutoff 与权重接口以“ratio”传入核函数。自定义核或自定义更新策略时要避免把它当成 $$h_i/h_\mathrm{ref}$$。

# 常见用法（调用范式）

## 1) 默认单分辨率（不做局部 refinement）

- `SPHBody` 构造时默认创建 `SPHAdaptation`（见 `SPHBody` 构造函数）。
- 你可以通过 `SPHBody::defineAdaptationRatios(h_spacing_ratio, system_refinement_ratio)` 调整标尺。

## 2) 使用局部加密 adaptation

在 body 上定义 adaptation（示意；具体参数取决于你的应用）：

```cpp
// 例如：使用局部加密
body.defineAdaptation<ParticleWithLocalRefinement>(
		/*h_spacing_ratio=*/1.3,
		/*system_refinement_ratio=*/1.0,
		/*local_refinement_level=*/2);
```

之后 `generateParticles()` 会自动调用 `initializeAdaptationVariables()` 注册：

- `SmoothingLengthRatio`
- `ParticleMeshLevel`

并在需要时创建 `MultilevelCellLinkedList`。

## 3) 形状驱动的加密策略（NearSurface / WithinShape）

这两个类提供“根据 signed distance 返回目标 spacing”的策略函数，但它们本文件中并未直接把 spacing 写回粒子（通常在 relax / generator / 自定义流程里使用）。

你可以在粒子生成/松弛步骤中依据 `getLocalSpacing()` 设定体积、或更新 `SmoothingLengthRatio`。

