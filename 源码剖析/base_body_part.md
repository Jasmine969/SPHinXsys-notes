# BodyPartByCell和BodyPartByParticle的区别

主要有两种body part，一种by cell，一种by particle。

结论一句话：`BodyPartByParticle`存的是“一串粒子索引”；`BodyPartByCell`存的是“一串 cell（每个 cell 里又有若干粒子索引）”。因此它们的遍历方式、适用场景和更新代价都不同。

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

## 用`BodyPartByParticle`（精确粒子集合）：

需要对“某一组特定粒子”施加操作（例如某个固定边界粒子层、固定观测点、固定结构节点）。
你希望集合非常精确，不想包含“附近但不在区域内”的粒子。

## 用 `BodyPartByCell`（区域/缓冲区，基于 cell linked list）：

用于边界条件、缓冲区、局部计算域等“空间区域”问题（例如入口/出口 buffer、阻尼区）。
粒子每步都在流动、排序、注入/删除时，按 cell 存储更自然：你只要选中区域 cell，里面有哪些粒子会随 cell linked list 更新自动变化。
典型例子就是 `AlignedBoxByCell` 给 `InflowVelocityCondition` 提供遍历范围：见 `dynamics_algorithms.h`:113-121 里 `identifier_.LoopRange()` 的用法。

## 性能/维护成本的直觉

`BodyPartByParticle`：遍历开销是$O(N_\mathrm{part})$，但如果粒子集合需要频繁重建（粒子流动导致集合变化），重建成本可能高。
`BodyPartByCell`：遍历开销大约是 $O(N_\mathrm{cells~in~region}+N_\mathrm{particles~in~those~cells})$；区域固定时，cell 集合通常相对稳定，而且能很好地跟随 cell linked list 的更新来“自动跟踪”区域内粒子。