本文总结 SPHinXsys 中固体弹性动力学模块 `elastic_dynamics` 的主要算法类、它们之间的继承/组合关系、典型调用流程与对应的数学含义。

参考实现位置：

- `src/shared/particle_dynamics/solid_dynamics/elastic_dynamics.h/.cpp`
- `src/for_2D_build/particle_dynamics/solid_dynamics/solid_dynamics_2d.cpp`（2D 法向旋转）
- `src/for_3D_build/particle_dynamics/solid_dynamics/solid_dynamics_3d.cpp`（3D 法向旋转+极分解）

---

# 1. 模块目标与基本变量

## 1.1 目标（弱可压弹性固体显式时间推进）

该模块实现“弱可压（weakly compressible）弹性固体”的显式SPH时间积分框架，核心是：

- 用变形梯度 $$\mathbf F$$ 描述大变形（拉格朗日形式）。
- 用两步 Verlet/Leapfrog 风格的应力松弛（stress relaxation）推进 $$\mathbf x,\mathbf v,\mathbf F$$。
- 用“声学时间步长”约束显式稳定性（由声速、速度与加速度给出）。

## 1.2 关键粒子状态/辅助量（按源码变量名）

常见状态变量（每粒子）：

- 位置：`Position` $$\mathbf x$$
- 速度：`Velocity` $$\mathbf v$$
- 受力：`Force` $$\mathbf f$$（本模块计算的内力/应力散度贡献等）
- 先验外力：`ForcePrior` $$\mathbf f^{prior}$$（重力/流固耦合等在别处累积）
- 质量：`Mass` $$m$$
- 密度：`Density` $$\rho$$
- 体积度量：`VolumetricMeasure` $$V$$（可视为粒子参考体积或体积权重）
- 变形梯度：`DeformationGradient` $$\mathbf F$$
- 变形梯度变化率：`DeformationRate` $$\dot{\mathbf F}$$（注意，它不是变形率张量——速度梯度的对称部分）
- 梯度修正矩阵：`LinearGradientCorrectionMatrix` $$\mathbf B$$（reproducing / 线性一致性修正）

自由表面相关：

- 法向：`NormalDirection` $$\mathbf n$$
- 初始法向：`InitialNormalDirection` $$\mathbf n_0$$
- 有符号距离：`SignedDistance` $$\phi$$
- 初始有符号距离：`InitialSignedDistance` $$\phi_0$$

材料参数来自 `ElasticSolid`：参考密度 $$\rho_0$$、参考声速 $$c_0$$、剪切模量 $$\mu$$ 等，并提供应力本构与数值阻尼算子。

---

# 2. 类之间的关系（继承 + 数据委托）

## 2.1 继承/组合关系总览

```
LocalDynamics
 ├─ ElasticDynamicsInitialCondition
 ├─ UpdateElasticNormalDirection
 ├─ AcousticTimeStep : LocalDynamicsReduce<ReduceMin>
 └─ (with DataDelegateInner)
		 ├─ DeformationGradientBySummation
		 ├─ BaseElasticIntegration
		 │   ├─ BaseIntegration1stHalf
		 │   │   ├─ Integration1stHalf
		 │   │   │   ├─ Integration1stHalfPK2
		 │   │   │   ├─ Integration1stHalfCauchy
		 │   │   │   ├─ Integration1stHalfKirchhoff
		 │   │   │   └─ Integration1stHalfPK2RightCauchy
		 │   │   └─ DecomposedIntegration1stHalf
		 │   └─ Integration2ndHalf
		 └─ (通过 BaseInnerRelation 提供 inner_configuration_ 邻域)
```

核心点：

- `LocalDynamics`：对单个 `SPHBody` 的局部遍历算子（每粒子独立 update/interaction）。
- `DataDelegateInner`：通过 `BaseInnerRelation` 提供“固体内部相互作用邻域”（inner neighbors）与相关数组访问。
- `LocalDynamicsReduce<ReduceMin>`：对所有粒子做归约（求全局最小时间步）。

---

# 3. 各类职责、输入输出与适用范围

下面按“做什么/依赖什么/产出什么/什么时候用”来总结。

## 3.1 `ElasticDynamicsInitialCondition`

**作用**：为弹性固体设置初始条件（框架类，通常由具体案例继承后在其构造/自定义函数中写入初值）。

**数据**：读取 `Position`，注册/写入 `Velocity`。

**适用**：几乎所有固体算例都需要先设置初始速度场（以及可能的位移/初始应变等，但这里暴露的是速度/位置指针）。

---

## 3.2 `UpdateElasticNormalDirection`

**作用**：更新自由表面法向与到表面的距离（signed distance）。

**关键步骤**：

1) 通过极分解得到旋转部分 $$\mathbf R$$，并旋转初始法向：

$$
\mathbf F = \mathbf R\,\mathbf U,\quad \mathbf R^T\mathbf R = \mathbf I,
$$
$$
\mathbf n = \mathbf R\,\mathbf n_0.
$$

2) 用Nanson关系/面积映射思想更新距离（源码实现采用 $$\mathbf F^{-T}\mathbf n_0$$ 的范数作为缩放）：

$$
ilde{\mathbf n} = \mathbf F^{-T}\,\mathbf n_0,\quad
\phi = \frac{\phi_0}{\|\tilde{\mathbf n}\| + \varepsilon}.
$$

其中 $$\varepsilon$$ 为数值小量（代码里是 `SqrtEps`）。

**2D/3D实现差异**：

- 3D：调用 Higham 等的 3×3 极分解算法得到 $$\mathbf R$$。
- 2D：利用 2×2 极分解的闭式旋转构造（本质上取 $$\mathbf R \propto \begin{bmatrix}F_{00}+F_{11}&F_{01}-F_{10}\\F_{10}-F_{01}&F_{00}+F_{11}\end{bmatrix}$$ 并归一化）。

**适用**：需要自由表面几何量（法向、距离）用于边界条件、表面力或后处理时。

---

## 3.3 `AcousticTimeStep`

**作用**：计算显式时间步长（全局取最小）。

**源码给出的粒子级时间步**（再对所有粒子做 `ReduceMin`）：

设

$$
\mathbf a_i = \frac{\mathbf f_i + \mathbf f^{prior}_i}{m_i},\quad
\|\mathbf a_i\| = \text{acceleration\_norm},
$$

则

$$
\Delta t_\mathrm{ac} = \mathrm{CFL}\cdot \min_i\left(
\sqrt{\frac{h_{\min}}{\|\mathbf a_i\| + \epsilon}},
\frac{h_{\min}}{c_0 + \|\mathbf v_i\|}
\right).
$$

这里 $$h_{\min}$$ 是最小平滑长度（`MinimumSmoothingLength()`），$$c_0$$ 是参考声速（`ReferenceSoundSpeed()`）。

**物理/数值含义**：

- 第一项类似“加速度控制”（避免粒子在一步内被加速度拉飞）。
- 第二项类似“CFL 声速/对流限制”（声学波传播 + 速度对流）。

**适用**：显式弱可压弹性固体几乎必用。若出现 $$\Delta t$$ 异常小，常见原因是 $$J=\det(\mathbf F)$$ 过小/变负导致应力或阻尼暴涨（源码注释也提示了）。

---

## 3.4 `DeformationGradientBySummation`

**作用**：通过 SPH 求和（而非积分）直接重构变形梯度 $$\mathbf F$$。

**公式（对应 `interaction()`）**：

令邻域核梯度向量（含体积权重）

$$
\nabla W_{ij}V_j \equiv \left(\frac{\partial W}{\partial r}\right)_{ij}\,V_j\,\mathbf e_{ij},
$$

则

$$
\mathbf F_i = \left[-\sum_{j\in\mathcal N(i)} (\mathbf x_i-\mathbf x_j)\otimes (\nabla W_{ij}V_j)^T\right]\,\mathbf B_i.
$$

其中 $$\mathbf B_i$$ 是线性梯度修正矩阵，用于提升一致性/近自由表面的收敛。

**适用**：

- 初始化或纠偏：当你希望从当前构型直接重构 $$\mathbf F$$（例如某些重映射/重初始化步骤）。
- 与 `Integration2ndHalf` 的 $$\dot{\mathbf F}$$ 积分方案互为两种更新路径：一个是“微分推进”，一个是“代数重构”。

---

## 3.5 `BaseElasticIntegration`

**作用**：第一/第二半步积分共享基类，统一拿到常用数组指针与邻域。

**提供**：`Vol_`, `pos_`, `vel_`, `force_`, `B_`, `F_`, `dF_dt_`。

**适用**：不是直接用的算子，而是后续所有积分类的基座。

---

## 3.6 `BaseIntegration1stHalf`

**作用**：第一半步“速度更新”的公共部分。

**更新公式（对应 `update()`）**：
$$
\mathbf v_i^{n+1} = \mathbf v_i^{n} + \Delta t\,\frac{\mathbf f_i + \mathbf f_i^\mathrm{prior}}{m_i}.
$$

**并持有**：`ElasticSolid` 材料、$$\rho_0$$、$$1/\rho_0$$、`Density`、`Mass`、`ForcePrior`、参考平滑长度 $$h$$。

**适用**：任何“第一半步应力松弛”变体都会继承它。

---

## 3.7 `Integration1stHalf`（通用一阶半步：应力散度 + 数值耗散）

**作用**：计算第一半步的内部力`Force`（应力散度项 + 颗粒对数值阻尼），再由 `BaseIntegration1stHalf::update()` 完成速度推进。

**关键内部量**：

- `StressPK1OnParticle`：记作 $$\mathbf P_i\mathbf B_i^{T}$$（源码变量 `stress_PK1_B_`，具体在各initialization中填充）。
- 数值耗散系数 `numerical_dissipation_factor_`（默认 0.25）。
- 核函数 $$W$$ 在 $$r=0$$ 的值 $$W_0$$（用于归一化权重）。

**颗粒对“应变率”标量（源码 `strain_rate`）**：
$$
\dot\varepsilon_{ij}=\left(\frac{d}{r_{ij}}\right)^2 (\mathbf x_i-\mathbf x_j)\cdot(\mathbf v_i-\mathbf v_j),
$$

其中 $$d$$ 为维数 `Dimensions`。

**内力**：
$$
\mathbf f_i=\sum_{j\in\mathcal N(i)} \frac{m_i}{\rho_0}
\left(\nabla W_{ij} V_j\right)
\left[\mathbf P_i\mathbf B_i^T + \mathbf P_j\mathbf B_j^T + \alpha w_{ij}\mathbf P^{damp}_{ij}\right]\mathbf e_{ij},
$$

其中 $$\alpha$$ 对应 `numerical_dissipation_factor_`，$$w_{ij}=W_{ij}/W_0$$；
数值阻尼项在代码里形如
$$
\mathbf P^{damp}_{ij} = \tfrac12(\mathbf F_i+\mathbf F_j)\,\mathcal D(\dot\varepsilon_{ij},h),
$$

$$\mathcal D(\cdot)$$ 由材料 `ElasticSolid::PairNumericalDamping()` 给出。

**适用**：这是最“标准”的第一半步形式。后面的PK2/Cauchy/Kirchhoff只是改变“如何在半步构型上构造 $$\mathbf P$$”。

---

## 3.8 `Integration1stHalfPK2` / `Integration1stHalfCauchy` / `Integration1stHalfKirchhoff`

三者都继承 `Integration1stHalf`，区别主要在 `initialization()`：如何从半步构型得到应力并写入 `stress_PK1_B_`。

它们共有的半步构型推进：

$$
\mathbf x_i^{n+1/2} = \mathbf x_i^n + \tfrac12\Delta t\,\mathbf v_i^n,
$$
$$
\mathbf F_i^{n+1/2} = \mathbf F_i^n + \tfrac12\Delta t\,\dot{\mathbf F}_i^n,
$$
$$
J_i^{n+1/2} = \det(\mathbf F_i^{n+1/2}),\quad
\rho_i^{n+1/2} = \frac{\rho_0}{J_i^{n+1/2}}.
$$

然后构造第一 Piola–Kirchhoff 应力（或等价形式）并乘以修正矩阵：

$$
\mathrm{stress\_PK1\_B}_i \leftarrow \mathbf P_i^{n+1/2}\,(\cdot)\,\mathbf B_i^{(\cdot)}.
$$

具体差异：

#### (a) `Integration1stHalfPK2`

**意图**：以第二 Piola–Kirchhoff（PK2）本构为基础得到 PK1。

典型关系为 $$\mathbf P = \mathbf F\,\mathbf S$$（$$\mathbf S$$ 为 PK2）。源码通过 `ElasticSolid::StressPK1(F,i)` 直接返回 PK1（注释说明其来自 PK2）。

并使用转置的修正矩阵（源码：`* B_.transpose()`）：

$$
\mathrm{stress\_PK1\_B}_i = \mathbf P_i\,\mathbf B_i^T.
$$

#### (b) `Integration1stHalfCauchy`

用Cauchy应力 $$\boldsymbol\sigma$$（由 Almansi 应变给出）转换到PK1：

Almansi 应变（欧拉应变）在代码中：

$$
\mathbf e = \tfrac12\left(\mathbf I - (\mathbf F\mathbf F^T)^{-1}\right).
$$

再用

$$
\mathbf P = J\,\boldsymbol\sigma(\mathbf e)\,\mathbf F^{-T},
$$

并乘修正矩阵（源码：`* B_`）。

#### (c) `Integration1stHalfKirchhoff`

以 Kirchhoff 应力 $$\boldsymbol\tau = J\boldsymbol\sigma$$ 为基础，分解为体积项与偏应变项：

体积比 $$J=\det(\mathbf F)$$，并构造等体积化的左 Cauchy–Green 张量：

$$
\bar{\mathbf b} = J^{-2/d}\,\mathbf F\mathbf F^T,
$$
$$
\bar{\mathbf b}^{dev} = \bar{\mathbf b} - \tfrac1d\,\mathrm{tr}(\bar{\mathbf b})\,\mathbf I.
$$

然后

$$
\mathbf P = (\boldsymbol\tau_{vol}(J) + \boldsymbol\tau_{dev}(\bar{\mathbf b}^{dev}))\,\mathbf F^{-T},
$$

再乘以修正矩阵 $$\mathbf B_i$$（源码为 `... * inverse_F_T * B_`）。

**适用选择建议（经验）**：

- 已有材料模型主要定义在 PK2/参考构型：倾向用 `Integration1stHalfPK2`。
- 想在欧拉量（如 Cauchy 应力）上做控制/耦合：用 `Integration1stHalfCauchy`。
- 想显式区分体积/剪切贡献、并与等体积化偏应变配合：用 `Integration1stHalfKirchhoff`。

---

## 3.9 `DecomposedIntegration1stHalf`（分解应力 + 预剪切力）

**作用**：另一种第一半步形式：

- 以“粒子应力 + 非均匀材料项”分解方式构造 `stress_on_particle_`。
- 通过颗粒对形式引入“预剪切力”以减轻虚假剪切应力/变形。
- 对“二阶拉普拉斯式 pairwise”与“一阶梯度式 particlewise”的不匹配，用 `correction_factor_ = 1.07` 做经验修正。

**初始化要点（源码结构）**：

- 同样推进到半步 $$\mathbf x^{n+1/2},\mathbf F^{n+1/2}$$ 并计算 $$J,\rho$$。
- 计算 $$J^{-2/d}$$（源码变量 `J_to_minus_2_over_dimension_` 实际保存 $$ (1/J^2)^{1/d} = J^{-2/d}$$）。
- 计算 $$\mathbf F^{-T}$$。
- 构造 `stress_on_particle_`：包含体积 Kirchhoff 项、剪切相关修正项，以及左 Cauchy 形式的数值阻尼项（`NumericalDampingLeftCauchy`）。

**相互作用力（源码 `interaction()`）**：除“粒子应力项”外，还显式加上颗粒对剪切力

$$
\mathbf f^\mathrm{shear}_{ij} \propto \mu\,(J_i^{-2/d}+J_j^{-2/d})\,\frac{\mathbf x_i-\mathbf x_j}{r_{ij}}.
$$

**适用**：

- 材料非均匀或剪切主导问题中，想提高稳定性/抑制虚假模式。
- 需要注意：它引入了经验修正系数（1.07），对高精度对比时需要校准/敏感性分析。

---

## 3.10 `Integration1stHalfPK2RightCauchy`（PK2 + 右 Cauchy 数值阻尼）

**作用**：在 `Integration1stHalfPK2` 的基础上，在 `initialization()` 里额外叠加“右 Cauchy”形式的阻尼应力。

**特点**：

- 额外注册 `SmoothingLengthRatio`（`h_ratio_`），阻尼计算使用 $$h/h_{ratio,i}$$。
- 其 `interaction()` 不包含 `Integration1stHalf` 里的 pair numerical damping（即只用 $$\mathbf P_i + \mathbf P_j$$ 的弹性贡献）；阻尼已经被加进了 `stress_PK1_B_`。

阻尼应力叠加（按源码结构）：

$$
\mathbf P \leftarrow \mathbf P + \tfrac12\alpha\,\mathbf F\,\boldsymbol\sigma^{damp}_{RC},
$$

其中 $$\boldsymbol\sigma^{damp}_{RC}$$ 由 `ElasticSolid::NumericalDampingRightCauchy()` 给出，$$\alpha$$ 为数值耗散因子。

**适用**：

- 当你希望把阻尼完全“并入粒子应力”，而不是在相互作用里做 pair damping。
- 或者你希望对阻尼模型做更材料一致的控制（右 Cauchy 形式）。

---

## 3.11 `Integration2ndHalf`（第二半步：更新位置与 $$\mathbf F$$）

**作用**：完成 Verlet 的第二半步：

1) `initialization`用更新后的速度把位置推进到整步：

$$
\mathbf x_i^{n+1} = \mathbf x_i^{n+1/2} + \tfrac12\Delta t\,\mathbf v_i^{n+1}.
$$

2) `interaction`通过速度梯度计算 $$\dot{\mathbf F}$$：

$$
\dot{\mathbf F}_i^{n+1}=\left[-\sum_{j\in\mathcal N(i)} (\mathbf v_i^{n+1}-\mathbf v_j^{n+1})\otimes(\nabla W_{ij}V_j)^T\right]\,\mathbf B_i.
$$

3) `update`再把 $$\mathbf F$$ 推进到整步：

$$
\mathbf F_i^{n+1} = \mathbf F_i^{n+1/2} + \tfrac12\Delta t\,\dot{\mathbf F}_i^{n+1}.
$$

**适用**：所有基于“$$\mathbf F$$ 随速度场演化”的显式固体求解流程都需要它。

---

# 4. 典型时间推进流程（把类串起来）

下面给出一个常见的单步 $$t^n\to t^{n+1}$$ 调度（外层控制器/时间循环通常在案例中实现，这里给出“概念上的串联”）：

1) （可选）初始化：`ElasticDynamicsInitialCondition` 写入初始 $$\mathbf v$$ 等。

2) 计算时间步：`AcousticTimeStep`

$$
\Delta t = \min_i \Delta t_i.
$$

3) 第一半步（应力松弛 / 内力计算）：选择下列之一

- `Integration1stHalfPK2` 或 `Integration1stHalfCauchy` 或 `Integration1stHalfKirchhoff`
- `DecomposedIntegration1stHalf`
- `Integration1stHalfPK2RightCauchy`

典型调用顺序（同一算子实例，对每粒子）：

- `initialization(i, dt)`：推进 $$\mathbf x,\mathbf F$$ 到半步，更新 $$\rho$$，并构造应力。
- `interaction(i, dt)`：邻域求和得到 `Force`（内力）。
- `update(i, dt)`（来自 `BaseIntegration1stHalf`）：更新 $$\mathbf v$$ 到整步。

4) 第二半步（更新位置与 $$\mathbf F$$）：`Integration2ndHalf`

- `initialization(i, dt)`：用 $$\mathbf v^{n+1}$$ 推进 $$\mathbf x$$ 半步到整步。
- `interaction(i, dt)`：由速度梯度计算 $$\dot{\mathbf F}$$。
- `update(i, dt)`：推进 $$\mathbf F$$ 半步到整步。

5) （可选）几何量更新：`UpdateElasticNormalDirection`（用新 $$\mathbf F$$ 更新 $$\mathbf n,\phi$$）。

6) （可选/替代）若采用代数重构：`DeformationGradientBySummation` 可在合适时机重算 $$\mathbf F$$。

---

# 5. 适用范围与选择建议（工程视角）

## 5.1 适用范围

- **弱可压弹性固体**：以参考声速 $$c_0$$ 控制体积响应，显式方案通常需要较小 $$\Delta t$$。
- **显式动力学**：适合瞬态冲击、波传播、快速动力响应；对准静态问题可能效率较低（时间步受声学约束）。
- **大变形**：以 $$\mathbf F$$ 为核心状态量，可描述大转动/大伸长（但需注意 $$J=\det(\mathbf F)$$ 的物理/数值合理性）。

## 5.2 方案选择建议

- 想要“默认稳妥”的路径：`Integration1stHalfPK2` + `Integration2ndHalf` + `AcousticTimeStep`。
- 自由表面/边界更敏感：关注修正矩阵 $$\mathbf B$$ 的使用（PK2 初始化中乘了 $$\mathbf B^T$$；Kirchhoff/Cauchy 里乘 $$\mathbf B$$）。
- 想要更强阻尼或不同阻尼形式：
	- pair damping：`Integration1stHalf`（通过 `PairNumericalDamping`）
	- stress-based damping：`Integration1stHalfPK2RightCauchy` 或 `DecomposedIntegration1stHalf`
- 如果时间步突然极小：优先检查 $$J$$ 是否接近 0 或为负、材料参数是否过硬、以及 CFL 是否需要降低。

---

# 6. 一句话总结

`elastic_dynamics` 把“材料本构（`ElasticSolid`）+ 邻域 SPH 离散（`DataDelegateInner`）+ 两步显式积分（`Integration1stHalf/2ndHalf`）+ 声学稳定步长（`AcousticTimeStep`）”封装成一组可组合的局部动力学算子；你主要通过选择不同的 `Integration1stHalf*` 变体来切换应力度量与阻尼策略。

