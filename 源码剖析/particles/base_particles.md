类：BaseParticles

> 位置：`src/shared/particles/base_particles.h(.hpp/.cpp)`

# 概览

`BaseParticles` 是 SPHinXsys 中“粒子数据容器”的基类：

- 负责管理一个 `SPHBody` 的粒子数据（几何量、物理量、以及各类方法运行时注册的局部变量）。
- 以“连续内存段”的方式组织不同类型粒子（real / buffer(reserved) / ghost），并提供在边界处进行“real⇄buffer”切换的一组操作。
- 提供变量注册/查询、输出到文件（restart/reload XML）等通用基础能力。

# 粒子分组与索引约定（重要）

通常约定：

- real particles：索引区间 `[0, TotalRealParticles())`，参与动力学更新。
- buffer / reserved particles：紧跟在 real 之后的预留粒子（用于注入/删除等操作）。
- ghost particles：运行时在末尾临时追加（用于边界条件等），通过 `allocateGhostParticles()` 扩展容量。

其中：

- `TotalRealParticles()` 是当前 real 粒子数（运行时变化）。
- `ParticlesBound()` 是当前粒子数组“容量上限”（包含 real + buffer + ghost），可能在模拟过程中增长。

# 成员变量

## 1) 全局计数/容量

| 变量名 | 含义 |
| --- | --- |
| `sv_total_real_particles_` | **真实粒子数**（real particles 的数量），类型为 `SingularVariable<UnsignedInt>*`，通过 `TotalRealParticles()` 访问其值。 |
| `particles_bound_` | **当前粒子数组容量上限**（bound/capacity）。在实现中会随 buffer/ghost 分配而增长。通过 `ParticlesBound()` 访问。 |

## 2) 粒子 ID 映射（排序/重排相关）

| 变量名 | 含义 |
| --- | --- |
| `original_id_` | 每个粒子“初始生成时”的 ID（`OriginalID`）。通常用于追踪粒子来源。 |
| `sorted_id_` | **从 original id 到当前索引的映射**（`SortedID`）：`sorted_id_[oid] = current_index`。在粒子交换/重排后用于快速定位。Ghost 粒子的 `sorted_id_` 会被设置为其对应 real 粒子的索引。 |

> 备注：在 `switchToBufferParticle()` 中会通过交换末尾 real 粒子与目标粒子，保持 real 区间连续，并更新 `original_id_ / sorted_id_`。

## 3) 基础几何/物理状态变量（常用指针句柄）

| 变量名 | 含义 |
| --- | --- |
| `dv_pos_` | 离散变量 `Position` 对应的 `DiscreteVariable<Vecd>*`（变量对象）。 |
| `Vol_` | `VolumetricMeasure` 的数据指针（体积/面积/长度度量，取决于粒子类型）。 |
| `rho_` | `Density`（密度）的数据指针。 |
| `mass_` | `Mass`（质量）的数据指针。 |

## 4) 关联对象/IO

| 变量名 | 含义 |
| --- | --- |
| `sph_body_` | 所属 `SPHBody` 引用。构造时会 `sph_body.assignBaseParticles(this)`。 |
| `body_name_` | body 名称缓存（来自 `sph_body_.getName()`）。 |
| `base_material_` | 关联材料 `BaseMaterial` 引用。 |
| `restart_xml_parser_` | restart（继续计算）用 XML 读写器。 |
| `reload_xml_parser_` | reload（从外部粒子文件重载初值）用 XML 读写器。 |

## 5) 变量集合（用于“哪些变量会演化/哪些变量要输出”）

| 变量名 | 含义 | 例子 |
| --- | --- | --- |
| `evolving_variables_` | 属于discrete variable。满足以下条件：1.需要在粒子排序时与粒子一起交换。2. 需要写入reload文件和restart文件。 | `"OriginalID"`, `"Position"`, `"VolumeMeasure"`。<br />对于fluid：`"Velocity"`, `"Mass"`, `"ForcePrior"`, `"Force"`, `"DensityChangeRate"`, `"Density"`, `"Pressure"`。 |
| `variables_to_write_` | 属于discrete variable。需要写入输出的一组离散变量（变量对象集合）。 | `"OriginalId"`, `"SortedID"`。<br />对于fluid：`"Velocity"`。 |
| `all_discrete_variables_` | 当前粒子上已注册的所有离散变量集合（内部用于查找/管理）。 | 所有`evolving_variables_`和`all_state_data_`都是。 |
| `all_singular_variables_` | 当前粒子上已注册的所有单值变量集合（内部用于查找/管理）。 | `sv_total_real_particles_`, `"PhbysicalTime"`,`damping_rate_`。 |
| `all_state_data_` | 属于discrete variable。“状态变量数据指针”汇总（不包含粒子 ID 那类`UnsignedInt`状态）。用于统一拷贝粒子状态等操作。 | `"Position"`, `"VolumeMeasure"`, `"Density"`, `"Mass"`。<br />对于fluid：`"Velocity"`, `"DensityChangeRate"`, `"ForcePrior"`, `"Force"`。 |

## 6) BodyPart（按粒子子集划分）

| 变量名 | 含义 |
| --- | --- |
| `total_body_parts_` | body part计数器，用于分配新body part ID。 |
| `body_parts_by_particle_` | 以粒子子集定义的body part列表。 |

# 成员函数

## 1) 基础信息

| 函数名 | 含义 |
| --- | --- |
| `BaseParticles(SPHBody&, BaseMaterial*)` | 构造：绑定body/material，注册 `TotalRealParticles` 单值变量。 |
| `getSPHBody()` / `getBaseMaterial()` | 访问所属 body / material。 |
| `getSPHAdaptation()` | 转发到 `sph_body_.getSPHAdaptation()`。 |
| `printBodyName()` | 打印 body 名称。 |
| `printAllDiscreteVariableNames(std::ostream&)` | 打印已注册离散变量名（按类型分组，便于确认模板参数）。 |

## 2) 粒子容量/数量管理

| 函数名 | 含义 |
| --- | --- |
| `svTotalRealParticles()` / `TotalRealParticles()` | 获取真实粒子数量变量/数值。 |
| `ParticlesBound()` | 获取粒子容量上限。 |
| `initializeAllParticlesBounds(size_t)` | 初始化 real 粒子数与容量：`TotalRealParticles = particles_bound_ = number_of_particles`。 |
| `initializeAllParticlesBoundsFromReloadXml()` | 从 reload XML 的粒子节点数量初始化容量与 real 粒子数。 |
| `increaseParticlesBounds(size_t)` | 扩容：`particles_bound_ += extra_size`。 |

## 3) 基础变量注册（几何/物理）

| 函数名 | 含义 |
| --- | --- |
| `initializeBasicParticleVariables()` | 注册/初始化基础变量：`Position`、`VolumetricMeasure`、`Density`、`Mass`、`OriginalID`、`SortedID` 等。 |
| `registerPositionAndVolumetricMeasure(StdVec<Vecd>&, StdVec<Real>&)` | 直接从外部数组挂接`Position/VolumetricMeasure`。 |
| `registerPositionAndVolumetricMeasureFromReload()` | 从 reload XML 中读取并注册 `Position/VolumetricMeasure`。 |
| `dvParticlePosition()` | 获取 `Position` 变量对象指针。 |

## 4) 粒子复制与三类粒子管理（real / buffer / ghost）

| 函数名 | 含义 |
| --- | --- |
| `copyFromAnotherParticle(i, j)` | 将粒子 `j` 的状态拷贝到粒子 `i`（基于 `all_state_data_` 批量拷贝）。 |
| `allocateGhostParticles(ghost_size)` | 在末尾追加 ghost 粒子容量，返回 ghost 区间起始索引（旧的 `particles_bound_`）。 |
| `updateGhostParticle(ghost_index, index)` | 用 real 粒子 `index` 的状态更新 ghost 粒子，并令 `sorted_id_[ghost_index] = index`。 |
| `switchToBufferParticle(index)` | 将某个 real 粒子“删除/转为 buffer”：与末尾 real 粒子交换数据并更新 ID 映射，然后 `TotalRealParticles--`。 |
| `createRealParticleFrom(index)` | 从某个粒子拷贝状态，在第一个 buffer 位置生成一个新 real 粒子（`TotalRealParticles++`），返回其 `original_id`。 |

## 5) 变量系统（模板接口，按需注册/查询/写出）

这一组接口是 `BaseParticles` 最核心的“变量注册系统”。理解它们时建议先记住两点：

1. **DiscreteVariable**：按“名字 + 类型”管理的一段粒子数组（本质上是 `T[data_size]` 的封装）。
2. **State variable**：是“会参与粒子状态拷贝/重启 IO 的变量子集”，注册时会把其数据指针加入 `all_state_data_`。

> 重要限制：`UnsignedInt` 被显式禁止作为 **state variable**（`static_assert`）。因此像 `OriginalID/SortedID` 这类 ID 变量应使用 `registerDiscreteVariable*` / `registerDiscreteVariableData*`。

### 5.1 registerDiscreteVariable*：注册“离散变量”（不自动纳入状态拷贝/重启）

| 接口 | 返回 | data_size | 是否加入 `all_state_data_` | 行为要点 |
| --- | --- | --- | --- | --- |
| `registerDiscreteVariable<DataType>(name, data_size, init...)` | `DiscreteVariable<DataType>*` | 调用者指定 | 否 | 若同名同类型变量已存在则直接复用；否则创建并用 `init...` 初始化。**仅在首次创建时初始化**。 |
| `registerDiscreteVariableData<DataType>(name, data_size, init...)` | `DataType*` | 调用者指定 | 否 | 与上面相同，但直接返回数据指针（`variable->Data()`）。 |

适用场景：

- **不是粒子状态**（不需要被 `copyFromAnotherParticle()` 拷贝、也不需要写入 restart/reload）的变量。
- 或者像 `UnsignedInt` 这类 **不能作为 state variable 的离散数组**。

> 细节：复用已存在变量时不会重新初始化（避免覆盖已有模拟状态）。

### 5.2 registerStateVariable*：注册“状态变量”（会参与状态拷贝/重启）

| 接口 | 返回 | data_size | 是否加入 `all_state_data_` | 行为要点 |
| --- | --- | --- | --- | --- |
| `registerStateVariable<DataType>(name, init...)` | `DiscreteVariable<DataType>*` | 固定为 `particles_bound_` | 是 | 内部调用 `registerDiscreteVariable(name, particles_bound_, init...)`，然后把 `DataType*` 加入 `all_state_data_`，用于粒子状态批量拷贝/重启读写。 |
| `registerStateVariableData<DataType>(name, init...)` | `DataType*` | 固定为 `particles_bound_` | 是 | 同上，但返回数据指针。 |

适用场景：

- 会随动力学更新、需要在粒子拷贝/交换（real/buffer/ghost）时同步的变量。
- 需要写入 restart/reload 的变量（通常配合 `addEvolvingVariable()` / `addVariableToWrite()` 使用）。

### 5.3 registerStateVariableFrom*：从已存在变量“拷贝初始化”一个新变量

| 接口 | 返回 | 说明 |
| --- | --- | --- |
| `registerStateVariableFrom<DataType>(new_name, old_name)` | `DiscreteVariable<DataType>*` | 查找 `old_name`，若不存在会打印错误并 `exit(1)`；若存在，则创建/复用 `new_name` 并用 `old_name` 的数据逐粒子拷贝初始化。 |
| `registerStateVariableDataFrom<DataType>(new_name, old_name)` | `DataType*` | 同上，但返回数据指针。 |

典型用途：

- 需要保留一份“初值/参考值”（比如把 `Velocity` 复制一份到 `Velocity0`）。

### 5.4 registerStateVariableFrom(StdVec<...>)：从外部数组写入（常用于几何生成后的挂接/拷贝）

| 接口 | 返回 | 说明 |
| --- | --- | --- |
| `registerStateVariableFrom<DataType>(name, geometric_data)` | `DiscreteVariable<DataType>*` | 创建/复用变量后，把 `geometric_data[i]` 拷贝到 `data_field[i]`（仅写入 `0..geometric_data.size()`）。其余位置保留默认初始化值。 |
| `registerStateVariableDataFrom<DataType>(name, geometric_data)` | `DataType*` | 同上，但返回数据指针。 |

### 5.5 registerStateVariableFromReload*：从 reload XML 中读取

| 接口 | 返回 | 前置条件 | 说明 |
| --- | --- | --- | --- |
| `registerStateVariableFromReload<DataType>(name)` | `DiscreteVariable<DataType>*` | 需要先 `readReloadXmlFile(path)` 载入 XML | 遍历 `reload_xml_parser_.first_element_` 下的每个 `particle` 节点，读取属性 `name` 到 `data_field[index]`。 |
| `registerStateVariableDataFromReload<DataType>(name)` | `DataType*` | 同上 | 同上，但返回数据指针。 |

### 5.6 批量注册/查询：registerStateVariables / getVariablesByName

| 接口 | 返回 | 行为 |
| --- | --- | --- |
| `registerStateVariables<DataType>(names, suffix)` | `StdVec<DiscreteVariable<DataType>*>` | 对每个 `name` 注册 `name + suffix` 的 state variable 并返回变量指针列表。 |
| `getVariablesByName<DataType>(names, suffix)` | `StdVec<DiscreteVariable<DataType>*>` | 对每个 `name` 查找 `name + suffix`，不存在会报错并 `exit(1)`。 |

### 5.7 getVariableByName / getVariableDataByName 的“强制存在”语义

- `getVariableByName<DataType>(name)`：若变量未注册，会打印错误 + body 名称，然后 `exit(1)`。
- `getVariableDataByName<DataType>(name)`：若变量存在但 `Data()` 尚未分配，也会 `exit(1)`。

因此它们适合“这里必须存在，否则就是配置/流程错误”的场景。

### 5.8 与 addEvolvingVariable / addVariableToWrite 的关系（常见调用范式）

注册 ≠ 自动输出/自动参与 evolving 列表。

- `addEvolvingVariable<DataType>("VarName")`：把变量加入 `evolving_variables_`，并把其数据指针加入 `evolving_variables_data_`（便于批量操作）。
- `addVariableToWrite<DataType>("VarName")`：把变量加入输出列表。

另外：`addVariableToList` 会检查 `variable->getDataSize() >= particles_bound_`，否则报错退出；因此只有“粒子维度”的数组才可加入这些列表。

#### 示例 1：新增一个会演化并输出的状态变量

```cpp
// 方式 A：拿到数据指针
Vecd *vel = particles.registerStateVariableData<Vecd>("Velocity", Vecd::Zero());
particles.addEvolvingVariable<Vecd>("Velocity");
particles.addVariableToWrite<Vecd>("Velocity");

// 方式 B：拿到变量对象（有时用于传递/数组变量）
auto *dv_vel = particles.registerStateVariable<Vecd>("Velocity", Vecd::Zero());
particles.addEvolvingVariable<Vecd>(dv_vel);
particles.addVariableToWrite<Vecd>(dv_vel);
```

#### 示例 2：从已有变量复制一份（用于保存参考态）

```cpp
particles.registerStateVariableFrom<Vecd>("Velocity0", "Velocity");
// Velocity0 也是 state variable，会被纳入 all_state_data_，也可按需加入写出列表
particles.addVariableToWrite<Vecd>("Velocity0");
```

#### 示例 3：从 reload XML 读取额外变量

```cpp
particles.readReloadXmlFile(path);
particles.registerStateVariableFromReload<Real>("Temperature");
particles.addEvolvingVariable<Real>("Temperature");
```

### 5.9 补充：addUnique* 与 register* 的区别（避免误用）

- `addUniqueDiscreteVariable* / addUniqueStateVariableData`：创建的变量存放在 `unique_variable_ptrs_`，**不进入** `all_discrete_variables_`；因此通常也**不能**通过 `getVariableByName` 这种“全局注册表”去查找。
- 它们更像“只给某个算法/阶段临时用的私有变量”，默认也不参与状态拷贝/重启 IO。

## 6) XML（restart / reload）

| 函数名 | 含义 |
| --- | --- |
| `resizeXmlDocForParticles(XmlParser&)` | 保证 XML 中 particle 节点数量 ≥ `TotalRealParticles()`。 |
| `writeParticlesToXmlForRestart(path)` / `readParticlesFromXmlForRestart(path)` | restart 读写：面向继续计算（通常包含 evolving variables）。 |
| `writeParticlesToXmlForReload(path)` / `readReloadXmlFile(path)` | reload 读写：面向重载初始粒子数据。 |
| `reloadExtraVariable<DataType>(name)` | 从 reload XML 额外读取一个变量并注册到粒子上（链式返回 `this`）。 |

## 7) BodyPart 管理

| 函数名 | 含义 |
| --- | --- |
| `getNewBodyPartID()` | 生成新的 body part ID（内部计数器加一并返回）。 |
| `addBodyPartByParticle(...)` / `getBodyPartsByParticle()` | 管理按粒子子集定义的 body part 列表。 |