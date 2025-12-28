
类：SPHBody / RealBody

> 位置：`src/shared/bodies/base_body.h(.cpp)`

# 概览

`SPHBody` 是 SPHinXsys 里“Body（物体）”的基础类，主要职责是把：

- **几何（Shape）**：物体的初始形状与边界盒
- **数值自适应策略（SPHAdaptation）**：分辨率参考、核函数、cell linked list 的创建策略等
- **材料（BaseMaterial / Closure）**：参考密度、局部材料参数初始化等
- **粒子容器（BaseParticles 及其派生类）**：粒子状态变量与 IO

绑定在一起，并提供一套“定义-生成-访问”的对象工厂式接口。

`RealBody` 是 `SPHBody` 的一个重要派生类：

- **只有 RealBody 才具有 cell linked list**（用于邻域搜索/配置构建）。
- 通过惰性创建（lazy create）的方式在首次访问时创建 cell linked list。

# 关键约定 / 生命周期（建议按这个顺序理解）

1. 创建 `SPHBody` 时需要一个 `Shape`（或提供 `SharedPtr<Shape>` / 仅给名字则使用默认形状）。构造函数里会把 body 注册到 `SPHSystem`。
2. 通常在生成粒子前会：
	- `defineMaterial<...>()`（或 `defineClosure<...>()`）
	- 需要的话：`defineAdaptationRatios(...)` / `defineAdaptation<...>()` / `defineBodyLevelSetShape(...)`
3. 调用 `generateParticles<ParticleType>(...)`：
	- 创建粒子对象
	- 粒子生成器写入几何变量
	- `particles->initializeBasicParticleVariables()` 注册基础状态
	- `sph_adaptation_->initializeAdaptationVariables(*particles)`
	- `base_material_->setLocalParameters(...)`
4. 若是 `RealBody`，在需要建立邻域关系时，通过 `getCellLinkedList()` 创建/获取 cell linked list。

> 注意：在粒子尚未生成前，调用 `getBaseParticles()` / `getBaseMaterial()` 会直接报错并 `exit(1)`。

# 成员变量（SPHBody）

## 1) 指针/对象所有权（内部 keeper）

| 成员 | 含义 |
| --- | --- |
| `shape_ptr_keeper_` | 保存/管理 shape 的 shared 指针（当 body 用 `SharedPtr<Shape>` 构造时会把它放进来，确保生命周期）。 |
| `sph_adaptation_ptr_keeper_` | `SPHAdaptation` 的所有权管理（unique）。 |
| `base_particles_ptr_keeper_` | `BaseParticles`（及其派生粒子类型）的所有权管理（unique）。 |
| `base_material_ptr_keeper_` | `BaseMaterial` / `Closure` 的所有权管理（unique）。 |

## 2) 运行时关键引用/缓存

| 成员 | 含义 |
| --- | --- |
| `sph_system_` | 所属 `SPHSystem` 引用。构造时 `sph_system_.addSPHBody(this)`。 |
| `body_name_` | body 的名字。 |
| `newly_updated_` | “本轮是否 newly updated” 标记（通常用于配置更新/调度逻辑）。 |
| `base_particles_` | 粒子容器指针（构造时为空，`BaseParticles` 构造时会 `assignBaseParticles(this)` 回填）。 |
| `is_bound_set_` | 是否显式设置过 `bound_`。 |
| `bound_` | body 的 bounding box（可手动设置）。 |
| `initial_shape_` | 初始体几何（volumetric geometry），用于界定 body。 |
| `sph_adaptation_` | 数值自适应策略指针。默认在构造时创建 `SPHAdaptation(sph_system.ReferenceResolution())`。 |
| `base_material_` | 材料指针。默认在构造时创建 `BaseMaterial()`，也可能被 `defineMaterial` / `defineClosure` 替换。 |
| `body_relations_` | 所有“以该 body 为中心订阅的关系”（`SPHRelation*`），由 `SPHRelation::subscribeToBody()` 追加。 |

# 成员函数（SPHBody）

## 1) 基础访问

| 函数 | 含义 |
| --- | --- |
| `getName()` | 返回 `body_name_`。 |
| `getSPHSystem()` | 返回 `sph_system_`。 |
| `getInitialShape()` | 返回 `*initial_shape_`。 |
| `assignBaseParticles(BaseParticles*)` | 由粒子构造时回填 `base_particles_`（DataDelegate 常用）。 |
| `getSPHAdaptation()` | 返回 `*sph_adaptation_`。 |
| `getBaseParticles()` | 若 `base_particles_==nullptr` 则报错并 `exit(1)`；否则返回引用。 |
| `getBaseMaterial()` | 若 `base_material_==nullptr` 则报错并 `exit(1)`；否则返回引用。 |
| `getBodyRelations()` | 返回 `body_relations_` 引用（供 relation 订阅/遍历）。 |

## 2) 循环范围

| 函数 | 含义 |
| --- | --- |
| `LoopRange()` | 返回 `[0, TotalRealParticles())` 的 `IndexRange`（典型并行循环范围）。 |
| `SizeOfLoopRange()` | 返回 `TotalRealParticles()`。 |

## 3) 分辨率/更新标记

| 函数 | 含义 |
| --- | --- |
| `getSPHBodyResolutionRef()` | 返回 `sph_adaptation_->ReferenceSpacing()`。 |
| `setNewlyUpdated()` / `setNotNewlyUpdated()` / `checkNewlyUpdated()` | 设置/查询 `newly_updated_`。 |

## 4) 边界盒

| 函数 | 含义 |
| --- | --- |
| `setSPHBodyBounds(bound)` | 手动设置 `bound_`，并置 `is_bound_set_=true`。 |
| `getSPHBodyBounds()` | 若 `is_bound_set_` 则返回 `bound_`；否则返回 `initial_shape_->getBounds()`。 |
| `getSPHSystemBounds()` | 返回 `sph_system_.getSystemDomainBounds()`（系统域边界）。 |

## 5) 粒子筛选 Mask（用于配置/邻域构建的策略注入）

| 类型 | 含义 |
| --- | --- |
| `SourceParticleMask` | 对 source 粒子默认“全通过”的 mask（`operator()` 恒返回 true）。 |
| `TargetParticleMask<TargetCriterion>` | 继承 `TargetCriterion` 的包装，作为 target 粒子筛选条件的适配器。 |

## 6) 工厂接口（定义 adaptation / shape / material）

| 函数 | 含义 |
| --- | --- |
| `defineAdaptationRatios(h_spacing_ratio, new_system_refinement_ratio)` | 调整 `SPHAdaptation` 内部的 ratio（转发到 `resetAdaptationRatios`）。 |
| `defineAdaptation<AdaptationType>(args...)` | 用指定类型创建 `sph_adaptation_`（替换默认的 `SPHAdaptation`）。 |
| `defineComponentLevelSetShape(shape_name, args...)` | 对 `ComplexShape` 组件定义 level set；需要 `initial_shape_` 可 `DynamicCast` 到 `ComplexShape`。 |
| `defineBodyLevelSetShape(args...)` | 把 `initial_shape_` 替换为 `LevelSetShape`（并返回该指针）。提供普通版本和带执行策略版本。 |
| `defineMaterial<MaterialType>(args...)` | 创建材料并回填 `base_material_`。 |
| `defineClosure<BaseModel, AuxiliaryModels...>(args...)` | 创建 closure（也属于 material keeper 管理），并回填 `base_material_` 指向 closure。 |

## 7) 粒子生成（关键入口）

| 函数 | 含义 |
| --- | --- |
| `generateParticles<ParticleType, Parameters...>(args...)` | 创建粒子对象 + 粒子生成器写几何变量；随后调用 `initializeBasicParticleVariables()`、`initializeAdaptationVariables()`、`setLocalParameters()`。返回生成的粒子指针。 |
| `generateParticlesWithReserve<ParticleType, Parameters...>(reserve, args...)` | 带 reserve/buffer（或 ghost）的一种便捷封装，内部还是走 `generateParticles`。 |

# RealBody：带 cell linked list 的 body

> 位置同上：`base_body.h(.cpp)`

## 成员变量

| 成员 | 含义 |
| --- | --- |
| `cell_linked_list_ptr_` | `BaseCellLinkedList` 的 unique 指针。 |
| `cell_linked_list_created_` | 是否已创建 cell linked list。 |

## 成员函数

| 函数 | 含义 |
| --- | --- |
| `getCellLinkedList()` | 惰性创建：首次调用时通过 `sph_adaptation_->createCellLinkedList(getSPHSystemBounds(), *base_particles_)` 创建，并设置名字为 `getName() + "CellLinkedList"`。 |
| `updateCellLinkedList()` | 调用 `getCellLinkedList().UpdateCellLists(*base_particles_)` 更新链表。 |
| `ListedParticleMask` | 目前别名为 `SPHBody::SourceParticleMask`（默认全粒子）。 |

