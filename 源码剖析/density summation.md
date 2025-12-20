# 基类

模板`DensitySummation<Base, DataDelegationType>`：所有`DensitySummation`的基类。是`LocalDynamics`和`DataDelegationType`的公有继承。它不实现具体`interaction/update`，只负责：通过`DataDelegate`拿到粒子/邻域配置等数据入口绑定/注册需要的粒子变量指针与常量。

# Inner分支（同一流体体内的相互作用）

实例`DensitySummation<Inner<Base>>`：所有`Inner`版本的基类。只是把委托类型固定为`DataDelegateInner`。

- 实例`DensitySummation<Inner<>>`：别名`DensitySummationInner`。实现了`interaction()`+`update()`。是最常用的 inner密度求和。
- 实例`DensitySummation<Inner<Adaptive>>`。实现了`interaction()`+`update()`（带变光滑长度/自适应修正）。

# Contact分支（与接触体/多体接触相互作用）

实例`DensitySummation<Contact<Base>>`：所有`contact`版本的基类。把委托类型固定为`DataDelegateContact`。缓存接触体信息：`contact_inv_rho0_`、`contact_mass_`。提供`ContactSummation(index_i)`：对所有contact邻域做一遍密度求和。

注意，所有`Contact`分支均没有实现`update()`成员函数，只实现了`interaction()`成员函数。这是因为它在这套设计里被当作“对 `rho_sum_`的增量修正项（correction term）”，而不是一个“完整的密度更新算法”。最终把`rho_sum_`写回`rho_`、以及由`rho_`派生出`Vol_`（Vol = mass / rho）这类状态落盘动作，应当由`Inner`（或其`FreeSurface`/`NearSurface`装饰版）来统一做一次，避免重复/冲突。

- 实例`DensitySummation<Contact<>>`：实现`interaction()`：把`contact`求和“加到”`rho_sum_` 上。
- 实例`DensitySummation<Contact<Adaptive>>`。实现`interaction()`：与`Contact<>`类似，但对自适应做`NumberDensityScaleFactor(h_ratio)`修正。

# FreeSurface/NearSurface叠加分支（只改update规则）

模板`DensitySummation<Inner<FreeSurface, SummationType...>>`：继承自`DensitySummation<Inner<SummationType...>>`。只实现`update()：rho = max(rho_sum, rho0)`（避免自由面处不合理低密度）。

- 实例`using DensitySummationFreeSurfaceInner = DensitySummation<Inner<FreeSurface>>;`。这里`SummationType`是空。

`FreeStream`与`NotNearSurface`：两个“NearSurfaceType policy functor”

- `NotNearSurface`：近表面就直接返回`rho`（相当于保持原值，不用`rho_sum`）
- `FreeStream`：如果`rho_sum < rho`，按比例把缺口补一点（更“温和”的自由流修正）

模板`DensitySummation<Inner<NearSurfaceType, SummationType...>>`：`NearSurfaceType`指的是`FreeStream`和`NotNearSurface`。只实现`update()`：如果“靠近自由面”则用`near_surface_rho_(rho_sum,rho0,rho)`，否则用`rho_sum`
通过粒子变量`Indicator`+内邻域判断`isNearFreeSurface(i)`只要邻居里有`Indicator==1`就认为near surface）

- 实例`using DensitySummationInnerNotNearSurface = DensitySummation<Inner<NotNearSurface>>;`
- 实例`using DensitySummationInnerFreeStream = DensitySummation<Inner<FreeStream>>;`

# 复合类型（Inner+Contact 的组合）

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

