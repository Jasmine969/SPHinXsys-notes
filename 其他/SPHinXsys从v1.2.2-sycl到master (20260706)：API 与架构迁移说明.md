> **版本基线**
>
> - 旧版本：`v.1.2.2-sycl`，commit `e98b9c4`，2026-01-21
> - 新版本：本文核对的 `master@2342419`，2026-07-06
>
> `master` 是持续更新的开发分支，因此本文描述的是上述 `master` 快照。后续版本仍可能继续修改 API。

本文主要面向已经使用 `v.1.2.2-sycl` 编写 SPHinXsys 用户案例、扩展源码或学习笔记的用户。

从 `v.1.2.2-sycl` 升级到当前 `master` 后，旧代码可能出现大量编译错误。实际迁移表明，其中最常见的问题来自以下几类变化：

1. 材料系统从 `BaseMaterial / Closure` 重构为 **Matter Material + Material Properties**；
2. `AlignedBox` 系列统一重命名为 `OrientedBox`；
3. 一系列 getter 和基础类型进行了命名统一；
4. `IOEnvironment` 从 `SPHSystem` 中解耦；
5. `SingularVariable` 更名为 `SingleVariable`，并引入新的数据视图体系；
6. `BodyPart` 与 relation masking 的职责重新划分；
7. SYCL/CK 异构计算、restart、材料模型等继续扩展。

因此，这次升级不能只理解为“改几个函数名”。尤其是**材料系统、变量系统和 BodyPart 机制已经发生了架构级变化**。

------

# 1. 常用 API 迁移速查表

| `v.1.2.2-sycl`                       | 当前 `master`                                                | 说明                                                         |
| ------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| `defineMaterial<T>()`                | `defineMatterMaterial<T>()`                                  | 仅针对主体物质材料                                           |
| `defineClosure<Base, Property...>()` | `defineMatterMaterial<Base>()` + `addMaterialProperty<Property>()` | 不是简单重命名，而是材料组合方式改变                         |
| `getBaseMaterial()`                  | `getMatterMaterial()`                                        | 获取主体材料                                                 |
| `Closure` 中除主体 `BaseModel` 外组合的 `AuxiliaryModels...`（例如 `Viscosity`） | `getMaterialProperty<T>()` / `collectMaterialProperties<T>()` | 新版将这些附加模型作为独立的 `MaterialProperty` 管理 |
| `*AlignedBox*`                       | `*OrientedBox*`                                              | 语义基本延续。`*`为通配符，包括`ByCell`、`ByParticle`、`Part`等。 |
| `getName()`                          | `Name()`                                                     | `SPHBody`、`Shape`、`BodyPart` 等均有此变化                  |
| `sph_system.getIOEnvironment()`      | `IO::getEnvironment()`                                       | IO 环境从 `SPHSystem` 解耦                                   |
| `*SingularVariable*`                 | `*SingleVariable*`                                           | 单值变量。`*`为通配符，包括`<T>`、`get`、`Name`等。          |
| `ShapeBooleanOps`                    | `GeometricOps`                                               | 几何布尔运算枚举                                             |
| `BaseIdentifier`                     | `RangeIdentifier`                                            | 部分模板/源码分析需要更新                                    |
| `initializeBasicParticleVariables()` | `initializeBasicDiscreteVariables()`                         | BaseParticles 初始化接口变化                                 |

此外，一些原先返回指针的 object factory API 现在返回引用。例如当前 `SPHBody::generateParticles()`、`defineMatterMaterial()` 和 `addMaterialProperty()` 均采用引用返回值。旧版 `SPHBody` 则使用 `BaseMaterial*`、`defineMaterial()`、`defineClosure()` 和返回 `ParticleType*` 的 `generateParticles()`。

------

# 2. 最大的架构变化：材料系统重构

## 2.1 旧版本：一个 Body 对应一个 `BaseMaterial` 或 `Closure`

在 `v.1.2.2-sycl` 中，`SPHBody` 内部主要保存一个：

```cpp
UniquePtrKeeper<BaseMaterial> base_material_keeper_;
BaseMaterial *base_material_;
```

普通材料使用：

```cpp
body.defineMaterial<MaterialType>(...);
```

需要组合多个材料模型时，则使用：

```cpp
body.defineClosure<BaseModel, AuxiliaryModel1, AuxiliaryModel2>(...);
```

例如旧代码：

```cpp
water_block.defineClosure<WeaklyCompressibleFluid, Viscosity>(
    ConstructArgs(rho0_f, c_f), mu_f);
```

旧版 `SPHBody` 的确直接提供了 `defineMaterial()` 和 `defineClosure()`。

## 2.2 当前版本：Matter Material 与 Material Property 分离

当前 `master` 中，一个 Body 可以拥有：

- 一个主要的 **Matter Material**；
- 零个或多个独立的 **Material Properties**。

因此上面的代码改为：

```cpp
water_block.defineMatterMaterial<WeaklyCompressibleFluid>(rho0_f, c_f);
water_block.addMaterialProperty<Viscosity>(mu_f);
```

当前 `SPHBody` 分别使用 `matter_keeper_` 和 `material_properties_keeper_` 管理这些对象，并提供：

```cpp
defineMatterMaterial<T>()
addMaterialProperty<T>()
getMatterMaterial()
getMaterialProperty<T>()
collectMaterialProperties<T>()
```

当前官方流体案例也采用：

```cpp
water_block.defineMatterMaterial<WeaklyCompressibleFluid>(rho0_f, c_f);
water_block.addMaterialProperty<Viscosity>(mu_f);
```

### 这意味着什么？

这不是：

```text
defineClosure → 换了一个名字
```

而是：

```text
旧：
Body
└── 一个 BaseMaterial
    └── Closure<BaseModel, AuxiliaryModels...>

新：
Body
├── 一个 MatterMaterial
└── 多个独立 MaterialProperty
```

这种设计让同一个 Body 可以更自然地组合多个材料属性，而不必把所有模型编译期封装进一个 `Closure` 类型。

因此，旧笔记中以下概念需要整体重写：

- “一个 SPHBody 拥有一个 BaseMaterial”；
- `base_material_keeper_`；
- `base_material_`；
- `defineMaterial()`；
- `defineClosure()`；
- `getBaseMaterial()`；
- “Closure 是 SPHBody 组合材料模型的核心机制”。

当前更准确的描述应是：

> `SPHBody` 分别管理主体物质材料 `MatterMaterial` 和可附加的多个材料属性。主体材料描述该 Body 是什么物质，例如 Fluid 或 Solid；附加材料属性则描述黏性等可以组合的物理属性。

------

# 3. `AlignedBox` 全面迁移到 `OrientedBox`

旧版本广泛使用：

```cpp
AlignedBox
AlignedBoxByCell
AlignedBoxByParticle
AlignedBoxPart
getAlignedBox()
```

当前版本对应为：

```cpp
OrientedBox
OrientedBoxByCell
OrientedBoxByParticle
OrientedBoxPart
getOrientedBox()
```

例如旧代码：

```cpp
AlignedBox left_emitter_shape(
    xAxis,
    Transform(Vec2d(left_bidirectional_translation)),
    bidirectional_buffer_halfsize);

AlignedBoxByCell left_emitter(
    water_block,
    left_emitter_shape);
```

迁移后：

```cpp
OrientedBox left_emitter_shape(
    xAxis,
    Transform(Vec2d(left_bidirectional_translation)),
    bidirectional_buffer_halfsize);

OrientedBoxByCell left_emitter(
    water_block,
    left_emitter_shape);
```

当前官方压力边界案例直接使用：

```cpp
OrientedBoxByCell left_emitter(
    water_block,
    OrientedBox(
        xAxis,
        Transform(Vec2d(left_bidirectional_translation)),
        bidirectional_buffer_halfsize));
```

`OrientedBox` 仍然保留：

```cpp
HalfSize()
checkInBounds()
checkUpperBound()
checkLowerBound()
```

等接口，因此大多数旧代码只是类型名和 getter 发生变化，而局部坐标系、参考轴以及 upper/lower bound 的基本语义仍然延续。

例如旧的速度入口：

```cpp
AlignedBox &aligned_box_;

aligned_box_(boundary_condition.getAlignedBox());

if (aligned_box_.checkInBounds(position))
{
    ...
}
```

现在应写为：

```cpp
OrientedBox &oriented_box_;

oriented_box_(boundary_condition.getOrientedBox());

if (oriented_box_.checkInBounds(position))
{
    ...
}
```

当前官方 FSI 案例就是这种写法。

需要特别注意：

> 编译器可能提示 `Eigen::AlignedBox`，但它与 SPHinXsys 旧版的 `AlignedBox` 不是同一个概念，不能用 `Eigen::AlignedBox` 替代。

------

# 4. `getName()` 统一迁移到 `Name()`

旧版本 `SPHBody` 和 `Shape` 使用：

```cpp
object.getName()
```

当前版本使用：

```cpp
object.Name()
```

例如粒子 reload：

旧：

```cpp
wall_boundary.generateParticles<BaseParticles, Reload>(
    wall_boundary.getName());
```

新：

```cpp
wall_boundary.generateParticles<BaseParticles, Reload>(
    wall_boundary.Name());
```

旧版 `SPHBody` 明确定义了 `getName()`，当前版本则定义 `Name()`。

`Shape` 也发生了相同变化：

```diff
- shape.getName()
+ shape.Name()
```

因此旧笔记中的 `getName()` 应重点检查。

不过，不建议对整个 C++ 项目中的所有 `.getName()` 盲目全局替换，因为第三方库或用户自定义类仍可能合法地使用 `getName()`。

------

# 5. `IOEnvironment` 从 `SPHSystem` 中解耦

旧版本中，`SPHSystem` 自己拥有 `IOEnvironment`，因此可以写：

```cpp
sph_system.getIOEnvironment().resetOutputFolder("LocalDirection");
```

旧版 `SPHSystem` 中确实包含 `io_keeper_`、`io_environment_` 和 `getIOEnvironment()`。

当前版本中应改为：

```cpp
IO::getEnvironment().resetOutputFolder("LocalDirection");
```

或完整写法：

```cpp
SPH::IO::getEnvironment().resetOutputFolder("LocalDirection");
```

当前 `IOEnvironment` 通过：

```cpp
namespace IO
{
IOEnvironment &initEnvironment();
IOEnvironment &getEnvironment();
}
```

统一访问。

因此其架构关系从：

```text
SPHSystem
└── IOEnvironment
```

变成了更接近：

```text
SPH::IO
└── 全局 IOEnvironment
```

这意味着旧笔记中“IOEnvironment 是 SPHSystem 的成员资源”的解释已经过时。

------

# 6. `SingularVariable` 更名为 `SingleVariable`，变量系统进一步重构

旧笔记中使用：

```cpp
SingularVariable<T>
DiscreteVariable<T>
```

当前版本变为：

```cpp
SingleVariable<T>
DiscreteVariable<T>
```

相关 API 也相应变化：

```diff
- addUniqueSingularVariable()
+ addUniqueSingleVariable()

- registerSingularVariable()
+ registerSingleVariable()

- getSingularVariableByName()
+ getSingleVariableByName()
```

旧版 `BaseParticles` 使用 `SingularVariable`，并通过构造函数接收 `BaseMaterial*`。

当前 `BaseParticles` 已改为 `SingleVariable`，构造函数只接收 `SPHBody&`，不再直接持有 `BaseMaterial` 接口。

因此旧笔记中的核心概念：

> `SingularVariable<T>` 表示整个对象只有一个值，`DiscreteVariable<T>` 表示每个粒子或离散点都有一个值。

**概念本身仍然成立**，但应更新为：

> `SingleVariable<T>` 表示单值状态，`DiscreteVariable<T>` 表示按离散实体存储的数据。

此外，当前变量体系还引入了：

```cpp
DataView<T>
EntryView<T>
MultiEntryView<T>
Quantity
```

这意味着当前变量系统不再只是简单的：

```text
SingleVariable
vs.
DiscreteVariable
```

而是进一步加入了面向 CPU/设备端统一数据访问的 view 层。

对于涉及 SYCL、CK 或底层变量存储的源码笔记，应重新理解：

```text
Quantity
├── SingleVariable
└── DiscreteVariable
        ↓
   DataView / EntryView / MultiEntryView
```

------

# 7. `BodyPart` 的基本概念仍成立，但 mask 职责发生变化

旧版中：

> `BodyPart` 自己维护 `dv_body_part_id_`，tag 粒子时直接给粒子写 part ID。

这一点已经不再准确。

当前 `BodyPartByParticle::tagParticles()` 主要构建：

```cpp
body_part_particles_
dv_particle_list_
sv_range_size_
```

而不再直接在这里写 `dv_body_part_id_`。

与 relation 邻域筛选有关的 target mask 逻辑现在进一步由：

```cpp
TargetParticleMask<..., BodyPartByParticle>
```

等机制负责。

因此旧版应从：

```text
BodyPart 自己维护成员身份 ID
```

调整为：

```text
BodyPart 负责描述粒子/Cell 集合；
relation 更新和邻居搜索阶段通过专门的 TargetParticleMask
处理针对 BodyPart 的目标粒子筛选。
```

# 8. 几何 API：`ShapeBooleanOps` → `GeometricOps`

旧版本：

```cpp
ShapeBooleanOps::add
ShapeBooleanOps::sub
```

当前版本：

```cpp
GeometricOps::add
GeometricOps::sub
```

旧版枚举名为 `ShapeBooleanOps`。

当前版本统一为 `GeometricOps`。

因此例如：

```diff
- multi_polygon_.addPolygon(shape, ShapeBooleanOps::add);
+ multi_polygon_.addPolygon(shape, GeometricOps::add);
```

这一点需要检查旧的几何创建笔记，因为其中可能同时混用了新旧名称。

------

# 9. Factory API 越来越倾向于返回引用

旧版本中：

```cpp
MaterialType *defineMaterial(...);
Closure<...> *defineClosure(...);
ParticleType *generateParticles(...);
LevelSetShape *defineBodyLevelSetShape(...);
```

当前版本中相应 API 更倾向于：

```cpp
MaterialType &defineMatterMaterial(...);
MaterialType &addMaterialProperty(...);
ParticleType &generateParticles(...);
LevelSetShape &defineBodyLevelSetShape(...);
```

所有权仍然主要由内部 keeper 管理，但用户侧 API 越来越倾向于返回**非拥有型引用**，从类型层面减少对所有权的误解。

# 10. Header 依赖更加模块化

旧版 `base_body.h` 直接包含了很多重量级头文件，包括：

```cpp
adaptation.h
all_geometries.h
base_material.h
base_particles.h
closure_wrapper.h
...
```

当前 `base_body.h` 大量使用 forward declaration，仅保留较少的直接 include。

因此升级后，自定义扩展源码可能出现一种新的编译错误：

```text
某个类型原来“顺便”被其他头文件 include 进来，
升级后变成 incomplete type 或 type not declared。
```

这种情况下不一定是 API 被删除了，也可能只是**旧代码依赖了传递式 include**。

建议自定义源码显式 include 自己实际使用的类型，而不要依赖其他头文件间接包含。

# 11. `master` 相比 `v.1.2.2-sycl` 的其他重要改变

## 11.1 材料系统支持更灵活的组合

这是本次升级最重要的架构变化之一。

新的：

```cpp
defineMatterMaterial<T>()
addMaterialProperty<T>()
getMaterialProperty<T>()
collectMaterialProperties<T>()
```

让“物质本体”和“附加物性”在类型和所有权上得到明确区分。

这比旧的 `Closure<Base, Auxiliary...>` 更适合一个 Body 拥有多个可独立组合的物理属性。

------

## 11.2 新增 `WeaklyCompressibleMixture`

当前版本已经包含：

```cpp
WeaklyCompressibleMixture
```

它支持：

- 多组分 species；
- 每种组分的参考密度；
- mass fraction；
- 局部参考密度；
- 面向 computing kernel 的 EOS 接口。

这表明当前材料系统已经开始支持更复杂的多组分流体模型，而不仅仅是单一的 `WeaklyCompressibleFluid`。

## 11.3 Restart 与 IO 机制继续增强

当前版本的 restart 系统增加了更完善的 summary/状态管理，`IOEnvironment` 对 output、restart 和 reload 文件夹的管理也更加独立。

对于复杂的 GPU/SYCL 案例，某些不能从当前几何重新推导的状态变量需要显式标记为 evolving variable，才能保证 restart 后状态连续。

因此，以后编写支持 restart 的用户案例时，应更加明确地区分：

```text
可从当前状态重新计算的变量
vs.
必须写入 restart 并恢复的演化状态
```

------

## 11.4 变量与设备数据访问体系更加成熟

当前版本增加了：

```cpp
DataView
EntryView
MultiEntryView
```

并逐步将底层数据访问从单纯的裸数组指针提升为更统一的数据 view 抽象。

这对普通 CPU 用户案例影响可能不明显，但对于：

- SYCL；
- CK；
- GPU offload；
- 多 entry 变量；
- 自定义 computing kernel；

是重要的底层架构变化。

------

## 11.5 BodyPart 与 relation masking 更模块化

当前版本将 BodyPart 的“区域定义”和 relation 邻域搜索中的“target particle masking”进一步分离。

这使得：

```text
BodyPart
```

不再需要自己承担所有粒子成员标记与邻域筛选职责，而是可以由 relation/update machinery 根据 RangeIdentifier 等信息构造相应 mask。

对于普通用户案例，这种变化通常不会直接出现；但对于阅读源码、写自定义 relation 或 computing kernel 的用户，这是需要重新学习的部分。

------

## 11.6 新增更多按变量选择 BodyPart 的能力

当前 `BodyPart` 体系已经提供：

```cpp
VariableRangeTagCriteria<T>
BodyPartByRealVar
BodyPartByIntVar
```

用于按照某个粒子变量的取值范围定义粒子子集。

相比只通过几何区域定义 BodyPart，这为：

- 按材料 ID；
- 按 indicator；
- 按某个连续状态变量范围；

选择粒子提供了更直接的机制。


# 总结

从 `v.1.2.2-sycl` 到当前 `master`，最值得注意的变化可以概括为：

```text
BaseMaterial / Closure
    ↓
MatterMaterial + multiple MaterialProperties
AlignedBox
    ↓
OrientedBox
SPHSystem-owned IOEnvironment
    ↓
SPH::IO global environment
SingularVariable
    ↓
SingleVariable + richer DataView infrastructure
```

以及：

```text
BodyPart directly handling more membership/mask details
    ↓
BodyPart definition and relation masking becoming more modular
```
