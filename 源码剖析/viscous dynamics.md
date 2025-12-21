# 基类

`ViscousForce<DataDelegationType>`：继承自`ForcePrior`+`DataDelegationType`（数据委托，决定它是`inner`还是`contact`）。在构造里会从粒子上拿到 Density/Mass/VolumetricMeasure/Velocity，并注册状态变量`ViscousForce`（写入`viscous_force_`）。

# 两个关键“策略参数”

## 粘度策略ViscosityType

- `FixedViscosity`：继承`PairGeomAverageFixed<Real>`，用两边材料`ReferenceViscosity()`做几何平均（常粘度/牛顿流体典型用法）
- `VariableViscosity`：继承`PairGeomAverageVariable<Real>`，从粒子变量`VariableViscosity`取值并几何平均（非牛顿/时空变化粘度）

## 核修正策略KernelCorrectionType

决定内/接触项里用什么“核修正矩阵/系数”。定义在`particle_functors.h`。

- `NoKernelCorrection`：恒为 1（不做核修正）
- `LinearGradientCorrection`：返回`B_[i]`（"LinearGradientCorrectionMatrix"）
- `LinearGradientCorrectionWithBulkScope`：关键用于open boundary/buffer 场景。在`operator()(j,i)`中，若`j`不属于bulk，则用`B_[i]`替代`B_[j]`，避免边界/缓冲粒子带来的不稳定核修正传播到主体

# ViscousForce的主要变体（继承关系 + 适用范围）

模板`ViscousForce<Inner<>, ViscosityType, KernelCorrectionType>`
$$
\boldsymbol{f}_\mathrm{vis}=2 \sum_j V_iV_j \bar{\mu} \frac{\boldsymbol{v}_{i}-\boldsymbol{v}_{j}}{r_{i j}+0.01h} \frac{\partial W_{i j}}{\partial r_{i j}}
$$

- 继承：`ViscousForce<DataDelegateInner>`（也就是inner数据委托）
- 构造参数：`BaseInnerRelation&`
- 适用范围：同一流体体内（single-phase/same body）的粘性力
- 特点：用速度差的“导数”形式` (v_i - v_j)/(r_ij + 0.01 h)`，并叠加核修正与体积权重，最后写`viscous_force_[i] = ...`
- 实例`using ViscousForceInner = ViscousForce<Inner<>, FixedViscosity, NoKernelCorrection>;`。单相/体内，常粘度、无核修正。

模板`ViscousForce<Inner<AngularConservative>, ViscosityType, KernelCorrectionType>`

- 继承：`ViscousForce<DataDelegateInner>`
- 构造参数：`BaseInnerRelation&`
- 适用范围：同一流体体内，但你更关心角动量守恒/涡量类问题时
- 特点：用 Monaghan 2005 风格的“角动量一致”形式（代码里注释提到对Taylor–Green Vortex更准确），力方向更贴合 `e_ij` 的投影形式

模板`ViscousForce<Contact<Wall>, ViscosityType, KernelCorrectionType>`
$$
\boldsymbol{f}_\mathrm{vis}=2 \sum_j V_iV_j^\mathrm{wall} \bar{\mu} \frac{\boldsymbol{v}_{i}-\boldsymbol{v}_{j}^\mathrm{wall}}{r_{i j}+0.01h} \frac{\partial W_{i j}}{\partial r_{i j}}
$$

- 继承：`BaseViscousForceWithWall`，其本质是`InteractionWithWall<ViscousForce>`，而后者继承于`ViscousForce<DataDelegateContact>`。
- 构造参数：`BaseContactRelation&`（壁面接触关系）
- 适用范围：流体-壁面（Wall）粘性作用
- 特点：使用`wall_vel_ave_`（壁面速度平均）与`wall_Vol_`（壁面粒子体积度量）来算剪切。计算中有系数`2.0 * (...)`，并且是 `viscous_force_[i] += ...`（通常配合 inner一起用：先inner得到体内粘性，再叠加壁面贡献）。粘度使用`mu_(index_i, index_i)`（壁面不一定有同类材料/变量，等价于“用流体自身粘度与壁面作用”）

模板`ViscousForce<Contact<Wall, AngularConservative>, ViscosityType, KernelCorrectionType>`

- 继承/构造/适用范围与`ViscousForce<Contact<Wall>, ViscosityType, KernelCorrectionType>`类似，但使用角动量一致的接触版本
- 适用范围：流体-壁面且希望角动量一致（或更稳定的涡量相关表现）

模板`ViscousForce<Contact<>, ViscosityType, KernelCorrectionType>`（多体接触/多相接触）

- 继承：`ViscousForce<DataDelegateContact>`
- 构造参数：`BaseContactRelation&`（一般接触关系，通常是流体-流体/相-相）
- 适用范围：不同`SPHBody`之间的粘性相互作用（典型：多相/多域耦合的流体-流体粘性）
- 特点：为每个接触体`k`缓存一套：`contact_mu_[k]`、`contact_kernel_corrections_[k]`、`contact_vel_[k]`、`wall_Vol_[k]`（名字叫`wall_Vol_`，但这里其实是“对方接触体的VolumetricMeasure”）。交互时双层循环：先遍历接触体`k`，再遍历邻居`j`。

# 复合类型

- `using ViscousForceWithWall = ComplexInteraction<ViscousForce<Inner<>, Contact<Wall>>, FixedViscosity, NoKernelCorrection>;`。体内+壁面、常粘度、无核修正（最常见的“流体粘性+无滑移壁面”组合）。
- `using ViscousForceWithWallCorrection = ComplexInteraction<ViscousForce<Inner<>, Contact<Wall>>, FixedViscosity, LinearGradientCorrection>;`。体内+壁面、常粘度、线性梯度修正矩阵，用于提升近壁/不规则粒子分布下的精度。
- `using MultiPhaseViscousForceWithWall = ComplexInteraction<ViscousForce<Inner<>, Contact<>, Contact<Wall>>, FixedViscosity, NoKernelCorrection>;`。体内+多相（相间粘性）+ 壁面、常粘度、无核修正。
- `using NonNewtonianViscousForceWithWall=ComplexInteraction<ViscousForce<Inner<FormulationType...>, Contact<Wall, FormulationType...>>, VariableViscosity, NoKernelCorrection>;`。非牛顿（粒子上有 `"VariableViscosity"`），并且把同一套Formulation tag（比如空或者`AngularConservative`）同时用于体内与壁面。