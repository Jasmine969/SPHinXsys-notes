
类：SPHRelation / BaseInnerRelation / BaseContactRelation

> 位置：`src/shared/body_relations/base_body_relation.h(.cpp)`

# 概览

`BaseBodyRelation` 这一层主要定义“Body 之间/内部的拓扑关系”的抽象基类与一些通用工具：

- **Relation 是以某个 body 为中心（centered）** 的：每个 relation 都持有 `sph_body_` 与其 `base_particles_`。
- Relation 的核心产物是 **ParticleConfiguration**：本质上是 `StdVec<Neighborhood>`，每个粒子一个 `Neighborhood`。
- 内部联系（inner）：`inner_configuration_`。
- 接触关系（contact）：对每个接触 body 维护一份 `contact_configuration_[k]`。

这些 configuration 通常由更具体的 relation（例如 inner/contact/complex 等派生类）在 `updateConfiguration()` 中填充；本文件提供基类框架与“清空 current_size_”的通用逻辑。

# SearchDepth：邻域搜索深度的小工具（cell linked list 相关）

> 这些 functor 的常见用途：在基于 cell linked list 的邻域构建中，为每个粒子给出需要搜索的“cell 层数/深度”。

| 类型 | 用途 | 核心逻辑 |
| --- | --- | --- |
| `SearchDepthSingleResolution` | 单分辨率最简单情况 | 恒返回 1。 |
| `SearchDepthContact` | 跨 body contact 搜索深度（基于目标 mesh） | `1 + floor(kernel->CutOffRadius() / target_mesh.GridSpacing())`。 |
| `SearchDepthAdaptive` | 变光滑长度（adaptive smoothing length），面向目标 mesh | 读取粒子变量 `SmoothingLengthRatio`：`1 + floor(kernel->CutOffRadius(h_ratio[i]) / grid_spacing)`。 |
| `SearchDepthAdaptiveContact` | contact 构建时的 adaptive 深度（用 adaptation 的 ratio 计算 cutoff） | `1 + floor(kernel.CutOffRadius(adaptation.SmoothingLengthRatio(i)) / grid_spacing)`。 |

> 注意：`SearchDepthAdaptive` 会直接从 `sph_body.getBaseParticles()` 里取 `Real* h_ratio_ = getVariableDataByName<Real>("SmoothingLengthRatio")`；如果该变量没注册，会触发 `getVariableDataByName` 的报错退出。

# BodyPartsToRealBodies：将 body part 列表转换为 real body 列表

| 函数 | 含义 |
| --- | --- |
| `BodyPartsToRealBodies(BodyPartVector)` | 对每个 `BodyPart` 取 `getSPHBody()` 并 `DynamicCast<RealBody>`，返回 `RealBodyVector`。 |

# SPHRelation：所有 relation 的抽象基类

## 成员变量

| 成员 | 含义 |
| --- | --- |
| `sph_body_` | 关系中心 body 的引用。 |
| `base_particles_` | `sph_body_.getBaseParticles()` 的引用（构造时获取）。 |

## 成员函数

| 函数 | 含义 |
| --- | --- |
| `SPHRelation(SPHBody&)` | 构造：绑定 `sph_body_` 并获取 `base_particles_`。 |
| `getSPHBody()` | 返回中心 body。 |
| `subscribeToBody()` | 将 `this` 放入 `sph_body_.getBodyRelations()`，便于系统统一调度/更新。 |
| `updateConfiguration()` | 纯虚函数：派生类负责建立/更新邻域配置。 |

# BaseInnerRelation：body 内部联系

> 该类以 `RealBody` 为中心，维护每个粒子的 inner neighbor 列表。

## 成员变量

| 成员 | 含义 |
| --- | --- |
| `real_body_` | 指向中心 `RealBody` 的指针（便于访问 cell linked list 等 RealBody 特有数据）。 |
| `inner_configuration_` | `ParticleConfiguration`：长度为 `base_particles_.ParticlesBound()`，元素类型为 `Neighborhood`。 |

## 成员函数

| 函数 | 含义 |
| --- | --- |
| `BaseInnerRelation(RealBody&)` | 构造：`subscribeToBody()`；并 `inner_configuration_.resize(ParticlesBound(), Neighborhood())`。 |
| `resetNeighborhoodCurrentSize()` | 将 `[0, TotalRealParticles())` 上每个粒子的 `inner_configuration_[i].current_size_` 置 0（并行）。 |

# BaseContactRelation：body 与其接触 bodies 的关系

> “接触”指中心 body 与多个其他 real body 的相互作用邻域；对每个 contact body 都维护一份配置。

## 成员变量

| 成员 | 含义 |
| --- | --- |
| `contact_bodies_` | 接触的 `RealBody` 列表。 |
| `contact_particles_` | 与 `contact_bodies_[k]` 对应的粒子指针 `&contact_body->getBaseParticles()`。 |
| `contact_adaptations_` | 与 `contact_bodies_[k]` 对应的 adaptation 指针 `&contact_body->getSPHAdaptation()`。 |
| `contact_configuration_` | `StdVec<ParticleConfiguration>`：对每个 contact body 一份配置；每份长度为 `ParticlesBound()`。 |

## 成员函数

| 函数 | 含义 |
| --- | --- |
| `BaseContactRelation(SPHBody&, RealBodyVector)` | 构造：`subscribeToBody()`；`contact_configuration_.resize(contact_bodies_.size())`；并对每个 contact body：缓存 particles/adaptation 指针，并把 `contact_configuration_[k]` resize 到 `ParticlesBound()`。 |
| `BaseContactRelation(SPHBody&, BodyPartVector)` | 便捷构造：内部先 `BodyPartsToRealBodies` 再走上面的构造函数。 |
| `resetNeighborhoodCurrentSize()` | 对每个 contact body `k`：将 `[0, TotalRealParticles())` 的 `contact_configuration_[k][i].current_size_` 置 0（并行）。 |
| `getContactBodies()` / `getContactParticles()` / `getContactAdaptations()` | 返回对应缓存（便于派生类构建/更新配置）。 |

# 读代码时的一个“抓手”

如果你后面要继续读更具体的 relation（例如 `inner_body_relation.*`、`contact_body_relation.*`、`complex_body_relation.*`），可以把它们理解为：

- 基于 cell linked list / kernel cutoff 等规则，填充 `Neighborhood` 内部的邻居索引、距离、核函数权重等数据。
- 基类这里提供的 `resetNeighborhoodCurrentSize()` 相当于“每次重新构建配置前把邻域计数清零”。

