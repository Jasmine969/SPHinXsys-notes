# ExecutionPolicy

Dynamics 系列类（`SimpleDynamics`, `ReduceDynamics`, `InteractionDynamics`, `InteractionWithUpdate`, `InteractionWithInitialization`, `Dynamics1Level` 等）的第一个模板参数通常是 `LocalDynamicsType`，第二个模板参数是 `ExecutionPolicy`。

默认情况下库内很多类型会使用 `ParallelPolicy`（或在开启 SYCL 时使用 device 版本的并行 policy），即默认采用并行模拟。

## Host 侧（CPU）ExecutionPolicy

这些类型定义在 `SPH::execution` 命名空间（见 `src/shared/particle_dynamics/execution/execution_policy.h`）：

- `SequencedPolicy`：串行（顺序）执行。调试时最常用。
- `UnsequencedPolicy`：允许“无序（unsequenced）”执行语义，适合没有循环依赖/没有竞态的场景（概念上类似标准库的 `std::execution::unseq`）。
- `ParallelPolicy`：并行执行（host 侧）。
- `ParallelUnsequencedPolicy`：并行 + 无序语义（概念上类似 `std::execution::par_unseq`）。

库里也提供了便捷常量（同样在 `SPH::execution` 下）：

- `seq` = `SequencedPolicy{}`
- `unseq` = `UnsequencedPolicy{}`
- `par_host` = `ParallelPolicy{}`
- `par_unseq` = `ParallelUnsequencedPolicy{}`

## Device 侧（SYCL/GPU）ExecutionPolicy

SPHinXsys 用一个轻量 wrapper `DeviceExecution<PolicyType>` 表示“在 device 上用某种语义执行”。

- `DeviceExecution<PolicyType>`：policy wrapper（继承自 `PolicyType`）。
- `ParallelDevicePolicy` = `DeviceExecution<ParallelPolicy>`
- `SequencedDevicePolicy` = `DeviceExecution<SequencedPolicy>`

对应的便捷常量：

- `par_device` = `ParallelDevicePolicy{}`
- `seq_device` = `SequencedDevicePolicy{}`

## MainExecutionPolicy（工程默认主执行策略）

`MainExecutionPolicy` 会根据编译宏 `SPHINXSYS_USE_SYCL` 自动选择：

- 若 `SPHINXSYS_USE_SYCL` 为真：`MainExecutionPolicy = ParallelDevicePolicy`
- 否则：`MainExecutionPolicy = ParallelPolicy`

并且库里提供了一个同名语义的常量 `par_ck`，类型随上述宏变化：

- SYCL 开启：`par_ck` 是 `ParallelDevicePolicy{}`
- SYCL 关闭：`par_ck` 是 `ParallelPolicy{}`

## 使用示例

### 1) 调试时切换为串行（你已经用过）

```cpp
SimpleDynamics<ComputeLocalTubeDirections, SequencedPolicy>
	compute_local_directions(wall_boundary);
```

### 2) 强制使用 host 并行（在你显式写 template 参数时）

```cpp
SimpleDynamics<ComputeLocalTubeDirections, ParallelPolicy>
	compute_local_directions(wall_boundary);
```

### 3) 在 SYCL 版本下，显式指定 device policy（如果某处允许这么用）

```cpp
using namespace SPH::execution;
SimpleDynamics<ComputeLocalTubeDirections, ParallelDevicePolicy>
	compute_local_directions(wall_boundary);
```

## 备注：Unsequenced / ParallelUnsequenced 什么时候用？

`UnsequencedPolicy` / `ParallelUnsequencedPolicy` 一般只适用于“每次迭代互不依赖、不会写同一份数据导致竞态”的计算（例如纯读取、或写入互不重叠的输出）。如果你的局部动力学里存在邻域相互作用、共享累加等写冲突，盲目切到 unseq/par_unseq 可能会引入难以复现的错误。

