# 基类

模板`DensitySummation<Base, DataDelegationType>`：所有`DensitySummation`的基类。是`LocalDynamics`和`DataDelegationType`的公有继承。它不实现具体`interaction/update`，只负责：通过`DataDelegate`拿到粒子/邻域配置等数据入口绑定/注册需要的粒子变量指针与常量。

# Inner分支（同一流体体内的相互作用）

按照下式更新密度：
$$
\rho_i=\rho_0\frac{\sum_j W_{ij}}{\sum_j W_{ij}^{0}}
$$
其中$$W_{ij}^{0}$$为初始时刻（严格来说是精确按照`Lattice`排布）的核函数值。

实例`DensitySummation<Inner<Base>>`：所有`Inner`版本的基类。只是把委托类型固定为`DataDelegateInner`。

- 实例`DensitySummation<Inner<>>`：别名`DensitySummationInner`。实现了`interaction()`+`update()`。是最常用的 inner密度求和。
- 实例`DensitySummation<Inner<Adaptive>>`。实现了`interaction()`+`update()`（带变光滑长度/自适应修正）。

# Contact分支（与接触体/多体接触相互作用）

在Inner分支基础上添加了一项，用下标f表示流体：
$$
\begin{aligned}
\langle\rho_i\rangle&=\rho_{f0}\frac{\sum_j W_{ij}}{\sum_j W_{ij}^{0}}+\frac{{\rho_{f0}}^2}{m_i}\frac{\sum_k W_{ik}m_k/\rho_{k0}}{\sum_j W_{ij}^0}\\
&=\frac{\rho_{f0}}{\sum_j W_{ij}^0}\left(\sum_j W_{ij}+\frac{\rho_{f0}}{m_i}\sum_k \frac{W_{ik}m_k}{\rho_{k0}}\right)
\end{aligned}
$$
这里下标$$j$$表示某个流体粒子，下标$$k$$表示某个接触体粒子。假设只有一种接触体，那么$$\rho_{k0}$$和$$m_k$$是常数，我们有
$$
\begin{aligned}
\langle\rho_i\rangle&=\frac{\rho_{f0}}{\sum_j W_{ij}^0}\left(\sum_j W_{ij}+\frac{\rho_{f0}m_k}{m_i\rho_{k0}}\sum_k W_{ik}\right)\\
&=\frac{\rho_{f0}}{\sum_j W_{ij}^0}\left(\sum_j W_{ij}+\frac{V_{k0}}{V_{j0}}\sum_k W_{ik}\right)
\end{aligned}
$$

所以接触体的density summation实际上是乘了一个体积的权重。如果所有粒子都是相同间距，那么Contact分支和Inner分支就一模一样了。**流体粒子的density summation与壁面粒子的密度实际上是无关的，只与体积有关**。所以即使壁面是密度很大的材料，也不会使得流体在做完density summation后密度偏高。

实例`DensitySummation<Contact<Base>>`：所有`contact`版本的基类。把委托类型固定为`DataDelegateContact`。缓存接触体信息：`contact_inv_rho0_`、`contact_mass_`。提供`ContactSummation(index_i)`：对所有contact邻域做一遍密度求和。

注意，所有`Contact`分支均没有实现`update()`成员函数，只实现了`interaction()`成员函数。这是因为它在这套设计里被当作“对 `rho_sum_`的增量修正项（correction term）”，而不是一个“完整的密度更新算法”。最终把`rho_sum_`写回`rho_`、以及由`rho_`派生出`Vol_`（Vol = mass / rho）这类状态落盘动作，应当由`Inner`（或其`FreeSurface`/`NearSurface`装饰版）来统一做一次，避免重复/冲突。

- 实例`DensitySummation<Contact<>>`：实现`interaction()`：把`contact`求和“加到”`rho_sum_` 上。
- 实例`DensitySummation<Contact<Adaptive>>`。实现`interaction()`：与`Contact<>`类似，但对自适应做`NumberDensityScaleFactor(h_ratio)`修正。

# FreeSurface/NearSurface叠加分支（只改update规则）

`SummationType`可以是空或者`Adaptive`。

模板`DensitySummation<Inner<FreeSurface, SummationType...>>`：继承自`DensitySummation<Inner<SummationType...>>`。只实现`update()：rho = max(rho_sum, rho0)`（避免自由面处不合理低密度）。

- 实例`using DensitySummationFreeSurfaceInner = DensitySummation<Inner<FreeSurface>>;`。这里`SummationType`是空。
  $$
  \langle\rho_i\rangle=\max\left\{\rho_0\frac{\sum_j W_{ij}}{\sum_j W_{ij}^{0}}, \rho_0\right\}
  $$

`FreeStream`与`NotNearSurface`：两个“NearSurfaceType policy functor”

- `NotNearSurface`：近表面就直接返回`rho`（相当于保持原值，不用`rho_sum`）。
- `FreeStream`：如果`rho_sum < rho`，按比例把缺口补一点（更“温和”的自由流修正）——`rho_sum + SMAX(Real(0), (rho - rho_sum)) * rho0 / rho`。

模板`DensitySummation<Inner<NearSurfaceType, SummationType...>>`：`NearSurfaceType`指的是`FreeStream`和`NotNearSurface`。只实现`update()`：如果“靠近自由面”则用`near_surface_rho_(rho_sum,rho0,rho)`，否则用`rho_sum`
通过粒子变量`Indicator`+内邻域判断`isNearFreeSurface(i)`只要邻居里有`Indicator==1`就认为near surface）

- 实例`using DensitySummationInnerNotNearSurface = DensitySummation<Inner<NotNearSurface>>;`

- 实例`using DensitySummationInnerFreeStream = DensitySummation<Inner<FreeStream>>;`
  $$
  \rho_i^\mathrm{sum}=\rho_0\frac{\sum_j W_{ij}}{\sum_j W_{ij}^{0}}
  $$
  
  $$
  \langle\rho_i\rangle=\begin{cases}
  \rho_i^\mathrm{sum}+\max\{0,\rho_i-\rho_i^\mathrm{sum}\}\rho_0/\rho_i,& i\text{ has free surface neighbor(s)};\\
  \rho_i^\mathrm{sum},& \text{else}.
  \end{cases}
  $$
  
  等价于
  $$
  \langle\rho_i\rangle=\begin{cases}
	\rho_i^\mathrm{sum},& i\text{ has no free surface neighbor or   }\rho_i < \rho_i^\mathrm{sum};\\
  \rho_i^\mathrm{sum}(1-\rho_0/\rho_i)+\rho_0,& \text{else}.\\
  
  \end{cases}
  $$
  对于表面粒子或者近表面粒子，一般会有

  1. $$\rho_0>\rho_i$$，于是$$\langle\rho_i\rangle<\rho_0$$。
  2. $$\rho_i > \rho_i^\mathrm{sum}$$，于是$$\langle\rho_i\rangle >   \rho_i^\mathrm{sum}$$。

  也即$$\rho_i^\mathrm{sum} < \langle\rho_i\rangle < \rho_0$$。所以我们可以认为`FreeStream`类型的density summation将边界粒子的$$\langle\rho_i\rangle$$拉高了一些，但没有像`FreeSurface`类型那样直接拉高到参考密度，因此这是一种更软的处理。

# 复合类型（Inner+Contact的组合）

复合类型的基类：

```cpp
template <class InnerInteractionType, class... ContactInteractionTypes>
using BaseDensitySummationComplex = ComplexInteraction<DensitySummation<InnerInteractionType, ContactInteractionTypes...>>;
```

- `using DensitySummationComplex = BaseDensitySummationComplex<Inner<>, Contact<>>;`

- `using DensitySummationComplexAdaptive = BaseDensitySummationComplex<Inner<Adaptive>, Contact<Adaptive>>;`
- `using DensitySummationComplexFreeSurface = BaseDensitySummationComplex<Inner<FreeSurface>, Contact<>>;`
- `using DensitySummationFreeSurfaceComplexAdaptive = BaseDensitySummationComplex<Inner<FreeSurface, Adaptive>, Contact<Adaptive>>;`
- `using DensitySummationFreeStreamComplex = BaseDensitySummationComplex<Inner<FreeStream>, Contact<>>;`
- `using DensitySummationFreeStreamComplexAdaptive = BaseDensitySummationComplex<Inner<FreeStream, Adaptive>, Contact<Adaptive>>;`
- `using DensitySummationNotNearSurfaceComplex = BaseDensitySummationComplex<Inner<NotNearSurface>, Contact<>>;`

# 针对压力边界的版本

设置压力边界时，一般会同步设置`BidirectionalBuffer`。density summation默认针对所有流体粒子。但是在buffer区域中的粒子使用target pressure更新密度：
$$
\rho_i=p_\mathrm{b}/c^2+\rho_0
$$
所以需要对默认的density summation做一个改进——识别粒子是否为内部流动区域的粒子，只有当条件满足时，才做density summation。见`tests\extra_source_and_tests\extra_src\shared\pressure_boundary\density_correciton.h`：

```cpp
    void DensitySummationPressure<Inner<>>::update(size_t index_i, Real dt = 0.0)
    {
        if (buffer_indicator_[index_i] == 0)
            assignDensity(index_i);
    };
```

其中`assignDensity(index_i)`做的是`rho_[index_i] = rho_sum_[index_i];`。
