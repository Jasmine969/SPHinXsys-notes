
# BodyPart 体系总结（含继承关系/成员/用法）

本文基于 `src/shared/bodies/base_body_part.h/.cpp` 对 BodyPart 家族做一份“按类索引”的总结，并在末尾保留并完善你原来的 by-cell / by-particle 对比结论。

## 继承关系（概览）

```text
BodyPart
├─ BodyPartByID
├─ BodyPartByParticle
│  ├─ BodyRegionByParticle
│  ├─ BodySurface
│  ├─ BodySurfaceLayer
│  ├─ AlignedBoxByParticle   (同时继承 AlignedBoxPart)
│  └─ (其它基于粒子的子类)
├─ BodyPartByCell
│  ├─ BodyRegionByCell
│  ├─ NearShapeSurface
│  ├─ AlignedBoxByCell       (同时继承 AlignedBoxPart)
│  └─ (其它基于 cell 的子类)
└─ AlignedBoxPart            (一个“混入”类：保存 aligned_box_)
```

> 说明：`AlignedBoxByParticle/AlignedBoxByCell` 是多重继承：分别继承 `BodyPartByParticle/BodyPartByCell` + `AlignedBoxPart`。

## 通用基类：BodyPart

### 职责

- 为一个 `SPHBody` 定义“body 的某一部分（part）”的公共属性。
- 维护该 part 的 `part_id_`，并在粒子上建立一个“属于哪个 part”的标记变量（`dv_body_part_id_`）。
- 提供一个通用的 mask：`TargetParticleMask`（用于“只对属于该 part 的粒子生效”的过滤）。

### 关键成员变量（核心状态）

- `SPHBody &sph_body_`：所属 body。
- `BaseParticles &base_particles_`：所属粒子容器。
- `int part_id_`：该 part 的唯一 id（通过 `base_particles_.getNewBodyPartID()` 获取）。
- `std::string part_name_`：默认名字（`bodyName + "Part" + id`）。
- `std::optional<std::string> alias_`：可选别名（很多派生类会把 shape名/固定名称写到 alias）。
- `SPHAdaptation &sph_adaptation_`：便于取 spacing 等信息。
- `SingularVariable<UnsignedInt> *sv_range_size_`：记录loop range的规模（粒子数或 cell 数，取决于子类）。
- `DiscreteVariable<int> *dv_body_part_id_`：每个粒子一个int，存它被标记成哪个`part_id_`。
- `Vecd *pos_`：粒子位置指针（来自`Position`）。
- `UniquePtrsKeeper<Entity> unique_variable_ptrs_`：用于创建 `dv_particle_list_ / dv_cell_list_ / sv_range_size_` 等“只属于该 part 的变量”。

### 关键成员函数

- `BodyPart(SPHBody &sph_body)`：
	- 分配 `part_id_`。
	- 注册 `dv_body_part_id_ = registerStateVariable<int>(part_name_ + "ID")`。
	- 取 `pos_ = getVariableDataByName<Vecd>("Position")`。
- `std::string getName() const`：优先返回 `alias_`，否则返回 `part_name_`。
- `BaseCellLinkedList &getCellLinkedList()`：对 `sph_body_` 做 `DynamicCast<RealBody>` 并返回其 cell linked list（因此：不是 `RealBody` 会在这里失败）。
- `SingularVariable<UnsignedInt> *svRangeSize()`：返回 range size。

### 内嵌模板：TargetParticleMask

```cpp
template <typename TargetCriterion>
class TargetParticleMask : public TargetCriterion
{
	// operator()(target_index, ...) 先判断 body_part_id_[target_index] == part_id_
	// 再调用 TargetCriterion::operator()
};
```

用途直觉：把任意“邻居判据/筛选条件”包装成“只对属于该 part 的粒子生效”的判据。

> 备注：仓库里也存在其它标识类（比如 `SPHBody`、`BodyPartition`）自带同名 `TargetParticleMask`；使用时到底实例化哪个版本，取决于上层算法里 `Identifier` 的具体类型。

## BodyPartByID

### 职责

- 表示“整个 body 作为一个 part”的最简单版本：loop range 就是全体 real particles。

### 关键点

- `typedef BodyPartByID BaseIdentifier;`：用于把它作为 dynamics 的“标识类型（Identifier）”。
- 构造时把 `sv_range_size_ = base_particles_.svTotalRealParticles()`，也就是直接用全体粒子数。

## BodyPartByParticle

### 职责

- 用“显式粒子索引列表”表示一个 body part。
- 构造后会通过 `tagParticles(...)` 建立粒子列表，并生成对应的 loop range。

### 关键成员变量

- `IndexVector body_part_particles_`：该 part 的粒子索引集合。
- `DiscreteVariable<UnsignedInt> *dv_particle_list_`：把 `body_part_particles_` 固化成一个离散变量（用于 device/offload 等）。
- `BoundingBox body_part_bounds_` + `bool body_part_bounds_set_`：可选的包围盒。

### 关键成员函数

- `IndexVector &LoopRange()`：返回 `body_part_particles_`。
- `size_t SizeOfLoopRange()`：返回粒子数。
- `void setBodyPartBounds(BoundingBox bbox)` / `BoundingBox getBodyPartBounds()`。
- `void tagParticles(TaggingParticleMethod &method)`：
	- 遍历 `[0, TotalRealParticles)`，对满足判据的粒子：
		- `dv_body_part_id_->setValue(i, part_id_)`（写标记）
		- `body_part_particles_.push_back(i)`（收集索引）
	- 生成 `dv_particle_list_`（长度 = 粒子数，内容 = 粒子索引）。
	- 生成 `sv_range_size_`（记录粒子数）。

### 重要语义

`BodyPartByParticle` 的粒子集合通常是“构造时刻的快照”：后续粒子移动后，是否仍在区域内不影响该列表（除非你重新构建/重新 tag）。

## BodyPartByCell

### 职责

- 用“一组 cell 列表”表示空间区域，粒子集合来自这些 cell 当前包含的粒子。
- 适合流动/注入/删除等会频繁改变粒子分布的场景：cell-linked-list 更新后，这个区域内“当前有哪些粒子”会随之变化。

### 关键成员变量

- `ConcurrentCellLists body_part_cells_`：该 part 的 cell 集合（每个 cell 对应一个粒子索引数组/并发容器）。
- `BaseCellLinkedList &cell_linked_list_`：来源于`RealBody`。
- `DiscreteVariable<UnsignedInt> *dv_cell_list_`：保存该 part 选中的 cell 索引。
- `DiscreteVariable<UnsignedInt> *dv_particle_index_`、`DiscreteVariable<UnsignedInt> *dv_cell_offset_`：cell linked list 的索引结构（用于按 cell 取粒子）。

### 关键成员函数

- `ConcurrentCellLists &LoopRange()`：返回 cell 列表（遍历时通常是“先 cell，再 cell 内粒子”）。
- `size_t SizeOfLoopRange()`：累加每个 cell 内粒子数。
- `void tagCells(TaggingCellMethod &method)`：
	- 调用 `cell_linked_list_.tagBodyPartByCell(...)` 筛选出 cell，并填充 `body_part_cells_` 与 `cell_indexes`。
	- 遍历 `body_part_cells_` 内所有粒子 index：对每个粒子写 `dv_body_part_id_->setValue(particle, part_id_)`。
	- 生成 `dv_cell_list_` 与 `sv_range_size_`（此处 size 是 cell 数）。

## 注意点

`BodyPartByCell`标记的是cell而非particle。cell是比较粗糙的，它的边界不一定和区域边界重合，所以`BodyPartByCell`管理的粒子**可能不在预期的区域之内**。在使用`BodyPartByCell`的时候建议严格检查粒子是否在预期区域之内。

例如`InflowVelocityCondition<...>`使用`AlignedBoxByCell`来遍历粒子，以下是其`update`函数：

```cpp
    void update(size_t index_i, Real dt = 0.0)
    {
        if (aligned_box_.checkContain(pos_[index_i])) // 严格检查粒子是否在aligned box内
        {
			// 设置vel_[index_i]为给定速度
        }
    };
```

再如`BidirectionalBuffer`的`TagBufferParticles`使用`AlignedBoxByCell`来遍历粒子，以下是其`update`函数：

```cpp
        virtual void update(size_t index_i, Real dt = 0.0)
        {
            if (aligned_box_.checkContain(pos_[index_i])) // 严格检查粒子是否在aligned box内
            {
                buffer_indicator_[index_i] = part_id_;
            }
        };
```

`BidirectionalBuffer`的`Injection`类使用`AlignedBoxByCell`来遍历粒子，以下是其`update`函数：

```cpp
        void update(size_t index_i, Real dt = 0.0)
        {
            ...
				if (aligned_box_.checkUpperBound(pos_[index_i], upper_bound_fringe_) &&
                    buffer_indicator_[index_i] == part_id_ && // 严格检查粒子是否在buffer区域内
                    index_i < particles_->TotalRealParticles())
                ...
        }
```

## 具体子类（按用途）

### BodyRegionByParticle

- 父类：`BodyPartByParticle`
- 额外成员：
	- `Shape &body_part_shape_`
	- `SharedPtrKeeper<Shape> shape_ptr_keeper_`（当通过 SharedPtr 构造时，用于延长 shape 生命周期）
- 构造行为：
	- `alias_ = shape.getName()`
	- `tagParticles( bind(tagByContain) )`
- 判据：`tagByContain(i) -> shape.checkContain(pos_[i])`

### BodySurface

- 父类：`BodyPartByParticle`
- 额外成员：`Real particle_spacing_min_`（取自 adaptation 的 MinimumSpacing）
- 构造行为：
	- `alias_ = "BodySurface"`
	- `tagParticles( bind(tagNearSurface) )`
- 判据：
	- `phi = sph_body_.getInitialShape().findSignedDistance(pos_[i])`
	- `abs(phi) < particle_spacing_min_`

### BodySurfaceLayer

- 父类：`BodyPartByParticle`
- 额外成员：`Real thickness_threshold_ = ReferenceSpacing * layer_thickness`
- 构造行为：
	- `alias_ = "InnerLayers"`
	- `tagParticles( bind(tagSurfaceLayer) )`
- 判据：
	- `distance = abs( findSignedDistance(pos_[i]) ) < thickness_threshold_`

### BodyRegionByCell

- 父类：`BodyPartByCell`
- 额外成员：
	- `Shape &body_part_shape_`
	- `SharedPtrKeeper<Shape> shape_ptr_keeper_`
- 构造行为：
	- `alias_ = shape.getName()`
	- `tagCells( bind(checkNotFar) )`
- 判据：`checkNotFar(cell_pos, threshold) -> shape.checkNotFar(cell_pos, threshold)`

### NearShapeSurface

- 父类：`BodyPartByCell`
- 额外成员：
	- `LevelSetShape &level_set_shape_`
	- `UniquePtrKeeper<LevelSetShape> level_set_shape_keeper_`（当需要从 shape 构造 level set 时持有）
- 典型构造方式：
	- `NearShapeSurface(real_body, SharedPtr<Shape>)`：内部创建 `LevelSetShape(real_body, *shape, true)`
	- `NearShapeSurface(real_body)`：从 `real_body.getInitialShape()` dynamic cast 到 `LevelSetShape`
	- `NearShapeSurface(real_body, sub_shape_name)`：从 ComplexShape 的子 shape dynamic cast 到 `LevelSetShape`
- 构造行为：`tagCells( bind(checkNearSurface) )`
- 判据：`checkNearSurface(cell_pos, threshold) -> level_set_shape_.checkNearSurface(cell_pos, threshold)`

## AlignedBox*（混入类 + 两个实现）

### AlignedBoxPart

- 这是一个“存 aligned box 的混入类”，不直接参与粒子/cell 标记。
- 成员：
	- `UniquePtrKeeper<SingularVariable<AlignedBox>> sv_aligned_box_keeper_`
	- `AlignedBox &aligned_box_`（引用到上面 `SingularVariable` 的数据）
- 函数：
	- `SingularVariable<AlignedBox> *svAlignedBox()`
	- `AlignedBox &getAlignedBox()`

### AlignedBoxByParticle

- 父类：`BodyPartByParticle, AlignedBoxPart`
- 构造：`AlignedBoxByParticle(real_body, aligned_box)` 后 `tagParticles(bind(tagByContain))`
- 判据：`aligned_box_.checkContain(pos_[i])`

### AlignedBoxByCell

- 父类：`BodyPartByCell, AlignedBoxPart`
- 构造：`AlignedBoxByCell(real_body, aligned_box)` 后 `tagCells(bind(checkNotFar))`
- 判据：`aligned_box_.checkNotFar(cell_pos, threshold)`

---

# BodyPartByCell 和 BodyPartByParticle 的区别

主要有两种 body part：一种 by cell，一种 by particle。

结论一句话：`BodyPartByParticle` 存的是“一串粒子索引”；`BodyPartByCell` 存的是“一串 cell（每个 cell 里又有若干粒子索引）”。因此它们的遍历方式、适用场景和更新代价都不同。

## 数据结构上的区别

### `BodyPartByParticle`

主要成员：`IndexVector body_part_particles_`
`LoopRange()` 直接返回这串粒子索引（见 `base_body_part.h`:95-126）。
语义：这个 body part 是“按粒子粒度精确选出来的一组粒子”。

### `BodyPartByCell`

主要成员：`ConcurrentCellLists body_part_cells_`（一组 cell，每个 cell 是 `ConcurrentIndexVector*`，内部是一串粒子索引）
`LoopRange()` 返回 cell 列表；真正遍历时会“先遍历 cell，再遍历 cell 内粒子”（见 `particle_iterators.h`:107-144）。
语义：这个 body part 是“按 cell 粒度粗筛出来的一片区域”，粒子集合来自这些 cell 当前包含的粒子。

## 构建/标记（tagging）方式的区别

`BodyPartByParticle::tagParticles(...)`
用一个 bool(size_t particle_index) 判据把粒子一个个挑出来，填进 `body_part_particles_`。
`BodyPartByCell::tagCells(...)`
用一个`bool(Vecd cell_position, Real threshold)`判据把 cell 挑出来，填进`body_part_cells_`（见`base_body_part.cpp`:83-111）。
之后遍历时使用的是这些 cell 里“当前”的粒子索引列表。

## 使用上的区别（什么时候用哪个）

### 用 `BodyPartByParticle`（精确粒子集合）

- 需要对“某一组特定粒子”施加操作（例如固定边界粒子层、固定观测点、固定结构节点）。
- 你希望集合非常精确，不想包含“附近但不在区域内”的粒子。
- 你能接受它更像“构造时刻的快照”，或者你愿意在需要时重建该集合。

### 用 `BodyPartByCell`（区域/缓冲区，基于 cell linked list）

- 用于边界条件、缓冲区、局部计算域等“空间区域”问题（例如入口/出口 buffer、阻尼区）。
- 粒子每步都在流动、排序、注入/删除时，按 cell 存储更自然：只要选中区域 cell，里面有哪些粒子会随 cell linked list 更新自动变化。

## 性能/维护成本的直觉

`BodyPartByParticle`：遍历开销是$O(N_\mathrm{part})$，但如果粒子集合需要频繁重建（粒子流动导致集合变化），重建成本可能高。
`BodyPartByCell`：遍历开销大约是 $O(N_\mathrm{cells~in~region}+N_\mathrm{particles~in~those~cells})$；区域固定时，cell 集合通常相对稳定，而且能很好地跟随 cell linked list 的更新来“自动跟踪”区域内粒子。

# AlignedBoxByParticle和AlignedBoxByCell

`AlignedBoxByParticle`：

- 记录的是被构造时`AlignedBox`区域的粒子ID。所以后续遍历时，无论这些粒子是否跑出了`AlignedBox`，都会把这些粒子遍历到。
- 用于`EmitterInflowInjection`、`EmitterInflowCondition`。

`AlignedBoxByCell`：

- 记录的是cell的ID。后续遍历时，无论粒子初始时是否属于这个cell，只要检测到粒子当前在cell里，就会被遍历到。
- 用于`InflowVelocityCondition<...>`、`DisposerOutflowDeletion`