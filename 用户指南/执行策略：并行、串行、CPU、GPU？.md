`src/shared/particle_dynamics/execution/execution_policy.h` 中的执行策略是用于模板分派的类型标签。策略名称描述的是期望的执行语义；某个策略是否可用，以及实际使用哪个后端，取决于相应算法是否为该策略提供了重载。

# 策略对比

| 策略         | 执行位置与后端                              | 并行度              | 当前源码支持                                                 | 单步调试                                                     |
| ------------ | ------------------------------------------- | ------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| `seq`        | Host CPU；普通 C++ 循环                     | 串行                | `particle_for`、`particle_reduce` 等 host 算法已实现 `SequencedPolicy` 重载 | 可以使用普通 C++ 调试器逐行单步，执行顺序确定                |
| `unseq`      | 设计上为 host SIMD 或允许乱序的执行语义预留 | 未指定              | 当前未提供通用的 `particle_for` / `particle_reduce` 重载；不能把它视为已实现的 SIMD 后端 | 不适用；若未来实现向量化，单步顺序也不应被假定               |
| `par_host`   | Host CPU + oneTBB                           | 多线程              | `ParallelPolicy` 的迭代器和归约已实现，分别调用 `tbb::parallel_for` 和 `tbb::parallel_reduce` | 可以调试，但任务分块和线程调度不固定；应避免依赖断点命中的线程或迭代顺序 |
| `par_unseq`  | 设计上为“线程级并行 + SIMD/乱序”语义预留    | 未指定              | 当前未提供通用迭代器重载                                     | 不适用；未来实现后也不应假定确定的迭代顺序                   |
| `par_device` | SYCL device，例如 GPU                       | 多个 work-item 并行 | SYCL 路径使用 `sycl::queue::submit` 和 `sycl::parallel_for`；同时有 device 端归约实现 | 需要编译器和目标设备支持的 SYCL device-debug 工具链；普通 host 调试器通常只会停在 kernel 提交和等待处 |
| `seq_device` | SYCL device 的单个 work-item                | 单 work-item        | 当前为 `LoopRangeCK<SequencedDevicePolicy, ...>` 提供 `sycl::single_task` 实现；不是通用 `IndexRange` 的 host 串行替代 | 普通 host 调试器通常不能逐步进入 device lambda；只有具备 device-debug 支持时才可能调试 kernel 内部 |

## `par_host` 与 TBB 的关系

`par_host` 是 SPHinXsys 的抽象执行策略，不是另一种独立的 CPU 并行后端。它的类型是 `ParallelPolicy`；在 host 粒子迭代器中，`particle_for` 会把`IndexRange` 交给 `tbb::parallel_for`。因此它代表的是：由 SPHinXsys 模板接口选择的 **CPU + oneTBB** 执行路径。

`SPHSystem` 构造函数接收的 `number_of_threads` 会传给`tbb::global_control::max_allowed_parallelism`，从而限制 TBB 的最大并行度。粒子迭代器还传入 `tbb::affinity_partitioner`，以倾向保持后续调用中的任务与线程亲和性，改善缓存局部性；这不会保证迭代的全局执行顺序。

## `par_device`、`seq_device` 与调试边界

二者都是 `DeviceExecution` 类型：`par_device` 包装 `ParallelPolicy`，`seq_device` 包装 `SequencedPolicy`。它们的差异发生在 device kernel 内：前者用多个 work-item 执行，后者以 `single_task` 只启动一个 work-item。

单 work-item 不等于 host 调用栈中的普通函数。`seq_device` 仍会经历 device kernel 的编译、提交和执行；当前实现随后调用 `wait_and_throw()` 等待完成。故排查一般动力学逻辑时应优先用 `seq`。`seq_device` 更适合保持 device 数据访问与接口不变的前提下执行或定位不适合并行化的 device-side 小任务。

## `par_ck`：按构建配置选择主后端

`MainExecutionPolicy` 与 `par_ck` 由 `SPHINXSYS_USE_SYCL` 在编译期选择：

| 构建配置               | `MainExecutionPolicy`  | `par_ck`                                    |
| ---------------------- | ---------------------- | ------------------------------------------- |
| `SPHINXSYS_USE_SYCL=0` | `ParallelPolicy`       | `ParallelPolicy`，即 host + TBB             |
| `SPHINXSYS_USE_SYCL=1` | `ParallelDevicePolicy` | `ParallelDevicePolicy`，即 SYCL device 并行 |

这使依赖 computing kernel（CK）的同一套动力学模板能在非 SYCL 构建时走TBB，在 SYCL 构建时走 device kernel，而调用侧不必为两种后端分别重写算法。

## 源码定位

- 策略类型、`par_ck` 与 `MainExecutionPolicy`：`src/shared/particle_dynamics/execution/execution_policy.h`
- Host 端 `seq` / `par_host` 粒子迭代器：`src/shared/particle_dynamics/particle_iterators.h`
- CK 的 host 端 `seq` / `par_host` 迭代器：`src/shared/shared_ck/particle_dynamics/particle_iterators_ck.h`
- SYCL 的 `par_device` / `seq_device` 迭代器：`src/src_sycl/shared/particle_dynamics/particle_iterators_sycl.h`
- TBB 范围与 affinity partitioner：`src/shared/common/sphinxsys_tbb.h`
- TBB 最大并行度设置：`src/shared/sphinxsys_system/sph_system.cpp`