# 1. 以GravityForceCK\<Gravity\>为例

前七课已经分别介绍了 SYCL 的执行模型，以及 SPHinXsys 中的 `par_host`、`par_ck`、`DiscreteVariable`、`DelegatedData()` 和 `UpdateKernel`。本课不再逐个解释这些概念，而是沿着一条真实源码调用链，把它们重新连接起来。

本课选择二维溃坝案例（`tests\tests_sycl\2d_examples\test_2d_dambreak_sycl\dambreak_sycl.cpp`）中的重力 Dynamics：

```cpp
Gravity gravity(Vecd(0.0, -gravity_g));

auto &constant_gravity =
    main_methods.addStateDynamics<
        GravityForceCK<Gravity>>(
            water_block, gravity);
```

选择它有三个原因：

- 数学逻辑简单，每个粒子只计算自身重力；
- 它同时使用 `DiscreteVariable` 和 `SingleVariable`；
- 它既能走 Host 路径，也能走 SYCL Device 路径，并且存在传统 CPU 版本可供对照。

对于粒子 \(i\)，最核心的计算是：

\[
\boldsymbol{F}_{g,i}
=
m_i\boldsymbol{a}_g
\left(
\boldsymbol{x}_i,t
\right)
\]

其中，\(m_i\) 是粒子质量，\(\boldsymbol{x}_i\) 是粒子位置，\(t\) 是当前物理时间，\(\boldsymbol{a}_g\) 是重力模型给出的加速度。

但 `GravityForceCK` 不只是把 \(m_i\boldsymbol{a}_g\) 写进一个数组。它还要维护总的 `ForcePrior`。在 SPHinXsys 中，`ForcePrior` 表示压力或应力之外的其他粒子力，例如重力、外载荷和驱动力。因此，本课会同时追踪 `GravityForceCK` 和它继承的 `ForcePriorCK`。

本文所引用的源码主要涉及以下文件：

```text
tests/tests_sycl/2d_examples/test_2d_dambreak_sycl/dambreak_sycl.cpp
│
├── src/shared/shared_ck/particle_dynamics/general_dynamics/
│   ├── force_prior_ck.h
│   ├── force_prior_ck.hpp
│   └── force_prior_ck.cpp
│
├── src/shared/shared_ck/particle_dynamics/
│   ├── simple_algorithms_ck.h
│   ├── loop_range.h
│   └── particle_iterators_ck.h
│
├── src/src_sycl/shared/particle_dynamics/
│   ├── particle_iterators_sycl.h
│   └── implementation_sycl.h
│
└── src/shared/particle_dynamics/
    ├── execution/implementation.h
    └── general_dynamics/
        ├── force_prior.h
        └── force_prior.hpp
```

# 2. 从用户案例入口确定真实类型

SYCL 溃坝案例首先创建两个方法容器：

```cpp
SPHSolver sph_solver(sph_system);

auto &main_methods =
    sph_solver.addParticleMethodContainer(par_ck);

auto &host_methods =
    sph_solver.addParticleMethodContainer(par_host);
```

启用 `SPHINXSYS_USE_SYCL` 时，`par_ck` 的类型是 `ParallelDevicePolicy`；`par_host` 的类型始终是 `ParallelPolicy`。因此，这两个容器在当前 SYCL 构建中大致是：

```text
main_methods
└── ParticleMethodContainer<ParallelDevicePolicy>

host_methods
└── ParticleMethodContainer<ParallelPolicy>
```

随后调用：

```cpp
auto &constant_gravity =
    main_methods.addStateDynamics<
        GravityForceCK<Gravity>>(
            water_block, gravity);
```

`ParticleMethodContainer::addStateDynamics()` 并不是直接创建一个 `GravityForceCK<Gravity>`，而是创建：

```cpp
StateDynamics<
    ExecutionPolicy,
    GravityForceCK<Gravity>>
```

由于这里的容器使用 `par_ck`，在 SYCL 构建中，实际类型可以展开为：

```cpp
StateDynamics<
    ParallelDevicePolicy,
    GravityForceCK<Gravity>>
```

关系可以画成：

```text
main_methods
│
│ addStateDynamics<GravityForceCK<Gravity>>
▼
StateDynamics<
    ParallelDevicePolicy,
    GravityForceCK<Gravity>>
│
├── Execution Policy：ParallelDevicePolicy
└── Update Type：GravityForceCK<Gravity>
```

`GravityForceCK<Gravity>` 描述“一个粒子应该怎样更新重力”；`StateDynamics` 则负责“如何让全部粒子执行这个更新”。

案例在进入时间循环前调用：

```cpp
constant_gravity.exec();
```

这里使用的是恒定重力，因此只需初始化一次。若使用随位置或时间变化的重力模型，则应根据物理模型在相应时间步重新执行。

同一个 CK Dynamics 也可以加入 `host_methods`：

```cpp
auto &constant_gravity =
    host_methods.addStateDynamics<
        GravityForceCK<Gravity>>(
            water_block, gravity);
```

此时它会被实例化为：

```cpp
StateDynamics<
    ParallelPolicy,
    GravityForceCK<Gravity>>
```

逐粒子计算公式没有变化，变化的是执行后端。

# 3. 外层 Dynamics 负责取得框架对象

`GravityForceCK` 的声明可以简化为：

```cpp
template <class GravityType>
class GravityForceCK
    : public LocalDynamics,
      public ForcePriorCK
{
  public:
    template <typename... Args>
    GravityForceCK(
        SPHBody &sph_body,
        Args &&...args);

    class UpdateKernel
        : public ForcePriorCK::UpdateKernel
    {
      public:
        template <
            class ExecutionPolicy,
            class EncloserType>
        UpdateKernel(
            const ExecutionPolicy &ex_policy,
            EncloserType &encloser);

        void update(
            size_t index_i,
            Real dt = 0.0);

      protected:
        GravityType gravity_;
        Real *physical_time_;
        Vecd *pos_;
        Real *mass_;
    };

  protected:
    const GravityType gravity_;
    SingleVariable<Real> *sv_physical_time_;
    DiscreteVariable<Vecd> *dv_pos_;
    DiscreteVariable<Real> *dv_mass_;
};
```

这个类分成外层对象和内层 `UpdateKernel` 两部分。

外层 `GravityForceCK` 保存的是框架层对象：

| 成员 | 含义 |
|---|---|
| `gravity_` | 重力模型 |
| `sv_physical_time_` | 全局物理时间变量 |
| `dv_pos_` | 所有粒子的位置变量 |
| `dv_mass_` | 所有粒子的质量变量 |

构造函数为：

```cpp
template <class GravityType>
template <typename... Args>
GravityForceCK<GravityType>::GravityForceCK(
    SPHBody &sph_body, Args &&...args)
    : LocalDynamics(sph_body),
      ForcePriorCK(
          this->particles_, "GravityForceCK"),
      gravity_(std::forward<Args>(args)...),
      sv_physical_time_(
          &sph_system_->svPhysicalTime()),
      dv_pos_(
          particles_->getVariableByName<Vecd>("Position")),
      dv_mass_(
          particles_->getVariableByName<Real>("Mass"))
{
}
```

`LocalDynamics(sph_body)` 让 Dynamics 获得 `SPHSystem`、`SPHBody`、`BaseParticles` 和循环范围等基础信息。

`ForcePriorCK(this->particles_, "GravityForceCK")` 则取得或注册三个粒子变量：

- `ForcePrior`：所有非压力、非应力作用力的总和；
- `GravityForceCK`：当前重力贡献；
- `PreviousGravityForceCK`：上一次记录的重力贡献。

`ForcePriorCK` 的核心构造逻辑为：

```cpp
ForcePriorCK::ForcePriorCK(
    BaseParticles *particles,
    const std::string &force_name)
    : ForcePriorCK(
          particles,
          particles->registerStateVariable<Vecd>(force_name))
{
}
```

进一步调用的构造函数会注册 `ForcePrior` 和 `Previous...` 变量，并将相关变量标记为需要随模拟状态演化的数据。

这里有一个重要设计区别：

- 外层 Dynamics 保存 `DiscreteVariable<Vecd>*` 等变量对象；
- 内层 `UpdateKernel` 保存 `Vecd*`、`Real*` 等实际计算指针。

外层对象适合在 Host 上组织框架资源，内层对象适合交给 Host 或 Device 上的逐粒子计算。

# 4. `UpdateKernel` 如何取得可访问数据

`ForcePriorCK::UpdateKernel` 先取得三个力数组：

```cpp
template <
    class ExecutionPolicy,
    class EncloserType>
ForcePriorCK::UpdateKernel::UpdateKernel(
    const ExecutionPolicy &ex_policy,
    EncloserType &encloser)
    : force_prior_(
          encloser.dv_force_prior_
              ->DelegatedData(ex_policy)),
      current_force_(
          encloser.dv_current_force_
              ->DelegatedData(ex_policy)),
      previous_force_(
          encloser.dv_previous_force_
              ->DelegatedData(ex_policy))
{
}
```

派生的 `GravityForceCK::UpdateKernel` 再取得重力计算需要的数据：

```cpp
template <class GravityType>
template <
    class ExecutionPolicy,
    class EncloserType>
GravityForceCK<GravityType>::
    UpdateKernel::UpdateKernel(
        const ExecutionPolicy &ex_policy,
        EncloserType &encloser)
    : ForcePriorCK::UpdateKernel(
          ex_policy, encloser),
      gravity_(encloser.gravity_),
      physical_time_(
          encloser.sv_physical_time_
              ->DelegatedData(ex_policy)),
      pos_(
          encloser.dv_pos_
              ->DelegatedData(ex_policy)),
      mass_(
          encloser.dv_mass_
              ->DelegatedData(ex_policy))
{
}
```

`DelegatedData(ex_policy)` 的作用不是改变数据类型。无论 Host 还是 Device 路径，最后得到的仍是普通指针，例如 `Vecd*` 或 `Real*`。它改变的是指针所指向的数据位置。

```text
DelegatedData(ParallelPolicy)
└── 返回 Host 数据地址

DelegatedData(ParallelDevicePolicy)
└── 返回 Device 可访问的数据地址
```

因此，同一个构造函数可以服务两种执行路径：

```text
外层 GravityForceCK
│
├── dv_pos_
├── dv_mass_
├── sv_physical_time_
└── 其他变量对象
        │
        │ DelegatedData(ex_policy)
        ▼
GravityForceCK::UpdateKernel
│
├── Vecd *pos_
├── Real *mass_
├── Real *physical_time_
└── Vecd *force...
```

当前实现中，`DiscreteVariable` 的 Device 路径使用 Device-only Allocation，`SingleVariable` 的 Device 路径使用 Shared USM。`UpdateKernel` 不需要知道这些管理细节，只需通过统一接口取得可用指针。

`gravity_` 没有通过指针访问，而是从外层对象复制到 `UpdateKernel` 中：

```cpp
gravity_(encloser.gravity_)
```

这要求具体 `GravityType` 能够作为 Device-compatible 的值对象被复制和调用。若用户自定义的重力类型内部包含 Host-only 指针、虚函数对象或不受支持的标准库资源，即使 `GravityForceCK` 本身支持 Device，该具体模板实例也可能无法通过 SYCL Device 编译。

# 5. `update(index_i, dt)` 只处理一个粒子

真正的物理计算位于：

```cpp
template <class GravityType>
void GravityForceCK<GravityType>::
    UpdateKernel::update(
        size_t index_i,
        Real dt)
{
    this->current_force_[index_i] =
        mass_[index_i] *
        gravity_.InducedAcceleration(
            pos_[index_i], *physical_time_);

    ForcePriorCK::UpdateKernel::update(
        index_i, dt);
}
```

第一步计算当前重力：

```cpp
current_force_[index_i] =
    mass_[index_i] *
    gravity_.InducedAcceleration(
        pos_[index_i],
        *physical_time_);
```

第二步调用基类更新总的 `ForcePrior`：

```cpp
force_prior_[index_i] +=
    current_force_[index_i] -
    previous_force_[index_i];

previous_force_[index_i] =
    current_force_[index_i];
```

为什么不是直接写成：

```cpp
force_prior_[index_i] +=
    current_force_[index_i];
```

因为 `ForcePrior` 可能同时包含多个外力贡献，而且同一个 Dynamics 可能重复执行。框架需要替换该 Dynamics 上一次留下的贡献，而不是每执行一次就把同一份力继续累加。

假设最初：

```text
ForcePrior = 0
PreviousGravityForce = 0
```

第一次执行后：

```text
CurrentGravityForce = m g

ForcePrior
= 0 + (m g - 0)
= m g

PreviousGravityForce = m g
```

第二次在重力不变时执行：

```text
ForcePrior
= m g + (m g - m g)
= m g
```

所以重力不会被重复加两次。

若重力从 \(\boldsymbol{g}_1\) 变成 \(\boldsymbol{g}_2\)，则：

\[
\boldsymbol{F}_{\mathrm{prior}}^{\,new}
=
\boldsymbol{F}_{\mathrm{prior}}^{\,old}
+
m\boldsymbol{g}_2
-
m\boldsymbol{g}_1
\]

这相当于先移除旧贡献，再加入新贡献。

`update()` 没有包含粒子循环。它只描述一个 `index_i` 应该做什么。也正因为如此，它可以被 TBB 的 CPU 任务或 SYCL Work-item 重复调用。

参数 `dt` 在这个 Dynamics 中没有直接使用，但保留统一的 `update(index_i, dt)` 接口，使 `StateDynamics` 可以用相同方式调用不同的 Update Type。

# 6. 从 `exec()` 追踪到 SYCL `parallel_for`

当用户调用：

```cpp
constant_gravity.exec();
```

实际进入的是：

```cpp
StateDynamics<
    ParallelDevicePolicy,
    GravityForceCK<Gravity>>::exec();
```

`StateDynamics::exec()` 的核心代码为：

```cpp
void exec(Real dt = 0.0) override
{
    this->setUpdated(
        this->identifier_->getSPHBody());

    this->setupDynamics(dt);

    UpdateKernel *update_kernel =
        kernel_implementation_
            .getComputingKernel();

    particle_for(
        LoopRangeCK<
            ExecutionPolicy,
            RangeIdentifier>(
                *this->identifier_),
        [=](size_t i)
        {
            update_kernel->update(i, dt);
        });

    finish_dynamics_();
}
```

整个过程可以分成四步。

## 6.1 准备 Computing Kernel

`getComputingKernel()` 会先在 Host 上构造 `UpdateKernel`，由 `kernel_keeper_` 持有。构造过程中，所有 `DelegatedData(ex_policy)` 都已经根据执行策略选好了数据地址。

当 Execution Policy 是 Device Policy 时，`Implementation` 会：

1. 在 Device 上为一个 `UpdateKernel` 对象分配空间；
2. 创建并由 `kernel_keeper_` 持有一个 Host 侧 `UpdateKernel`；
3. 将 Host 侧对象的内容复制到 Device；
4. 返回 Device 上的 `UpdateKernel*`。

因此，传给 SYCL Kernel 的 `update_kernel` 不是指向普通 Host 对象的指针，而是 Device 可访问的 Computing Kernel 指针。

## 6.2 确定循环范围

`GravityForceCK` 继承 `LocalDynamics`，其默认 `RangeIdentifier` 是 `SPHBody`。对应的 `LoopRangeCK` 会从：

```cpp
svTotalRealParticles()
```

取得真实粒子总数。

对于完整 `SPHBody`，`computeUnit(f, i)`只是直接执行`f(i)`。

因此，这里的逻辑循环编号就是实际粒子编号：

```text
逻辑编号 i
└── index_i = i
```

对于 `BodyPartByParticle` 等其他 Range，`LoopRangeCK` 还会通过粒子列表把逻辑编号映射为实际粒子编号。

## 6.3 提交 SYCL Kernel

Device 版本的 `particle_for` 最终调用：

```cpp
auto &sycl_queue =
    execution_instance.getQueue();

const size_t loop_bound =
    loop_range.LoopBound();

sycl_queue.submit(
    [&](sycl::handler &cgh)
    {
        cgh.parallel_for(
            execution_instance.getUniformNdRange(loop_bound),
            [=](sycl::nd_item<1> item)
            {
                if (item.get_global_id(0) < loop_bound)
                {
                    loop_range.computeUnit(
                        unary_func,
                        item.get_global_id(0));
                }
            });
    }).wait_and_throw();
```

`getUniformNdRange()` 会把全局范围向上补齐为局部范围的整数倍。例如真实粒子数为 1000、局部大小为 64 时，全局范围会补齐到 1024。因此 Kernel 中必须检查：

```cpp
item.get_global_id(0) < loop_bound
```

避免补出的 Work-items 访问越界。

对于合法 Work-item：

```text
item.get_global_id(0)
        │
        ▼
LoopRangeCK::computeUnit(...)
        │
        ▼
StateDynamics 中的 unary_func(i)
        │
        ▼
update_kernel->update(i, dt)
```

所以 `index_i` 的完整来源是：

```text
SYCL Work-item 的 global id
        │
        ▼
LoopRangeCK 的逻辑循环编号
        │
        ▼
实际粒子编号 index_i
```

## 6.4 等待任务完成

当前 `particle_for` 在提交后立即调用：

```cpp
wait_and_throw();
```

因此，`constant_gravity.exec()` 返回时，这一次逐粒子重力更新已经完成，并且相应的 SYCL 异步错误已经被处理。

这使不同 Dynamics 之间的顺序关系容易理解，但也意味着当前这条执行路径不会把 Event 暴露给上层，让多个 `exec()` 自由重叠执行。

完整调用链可以总结为：

```text
constant_gravity.exec()
│
▼
StateDynamics::exec()
│
├── setupDynamics(dt)
│
├── getComputingKernel()
│   ├── 构造 UpdateKernel
│   ├── DelegatedData(ex_policy)
│   └── Device 路径：复制 Kernel 对象到 Device
│
├── LoopRangeCK
│   └── 取得真实粒子总数
│
└── particle_for()
    │
    ▼
SYCL queue.submit()
    │
    ▼
parallel_for(nd_range)
    │
    ▼
Work-item global id
    │
    ▼
update_kernel->update(index_i, dt)
    │
    ▼
wait_and_throw()
```

# 7. 与传统 CPU 版本逐项对照

传统 CPU 溃坝案例使用：

```cpp
Gravity gravity(
    Vecd(0.0, -gravity_g));

SimpleDynamics<
    GravityForce<Gravity>>
constant_gravity(
    water_block, gravity);
```

传统 `GravityForce` 的结构可以简化为：

```cpp
template <class GravityType>
class GravityForce : public ForcePrior
{
  protected:
    const GravityType gravity_;
    Vecd *pos_;
    Real *mass_;
    Real *physical_time_;

  public:
    GravityForce(
        SPHBody &sph_body,
        const GravityType &gravity);

    void update(
        size_t index_i,
        Real dt = 0.0);
};
```

它在外层 Dynamics 中直接保存原始数据指针，`SimpleDynamics::exec()` 再通过 CPU `particle_for` 调用：

```cpp
this->update(i, dt);
```

CK 版本则把结构拆成：

```text
GravityForceCK 外层
├── 框架变量对象
└── 重力模型
        │
        ▼
GravityForceCK::UpdateKernel
├── 原始数据指针
└── update(index_i, dt)
```

三条执行路线可以对照如下：

| 路线 | 用户层类型 | 单粒子计算位置 | 并行执行 |
|---|---|---|---|
| 传统 CPU | `SimpleDynamics<GravityForce<Gravity>>` | 外层 `GravityForce::update()` | CPU/TBB |
| Host CK | `StateDynamics<ParallelPolicy, GravityForceCK<Gravity>>` | `GravityForceCK::UpdateKernel::update()` | CPU/TBB |
| Device CK | `StateDynamics<ParallelDevicePolicy, GravityForceCK<Gravity>>` | 同一个 `UpdateKernel::update()` | SYCL `parallel_for` |

传统 CPU 版本和 CK 版本的数学核心几乎相同：

```cpp
current_force_[index_i] =
    mass_[index_i] *
    gravity_.InducedAcceleration(
        pos_[index_i],
        *physical_time_);
```

主要差异不在物理公式，而在数据和执行方式：

| 传统 CPU Dynamics | CK Dynamics |
|---|---|
| 外层类直接保存 Host 数据指针 | 外层类保存变量对象 |
| `update()` 属于外层类 | `update()` 属于嵌套 `UpdateKernel` |
| 不需要 `DelegatedData()` | 通过 Execution Policy 选择数据地址 |
| 不创建独立 Computing Kernel 对象 | Computing Kernel 可复制到 Device |
| 主要面向 CPU | 同一结构可用于 Host 或 Device |

若要验证同一个 CK Dynamics 的 Host 路径，只需将它加入 `host_methods`：

```cpp
auto &constant_gravity =
    host_methods.addStateDynamics<
        GravityForceCK<Gravity>>(
            water_block, gravity);
```

这不是换回传统 `GravityForce`，而是让同一个 `GravityForceCK` 的 `UpdateKernel` 由 TBB 在 CPU 上执行。

进行 Host 与 Device 对比时，不要在同一次模拟中先后执行两个 `constant_gravity` 对象，否则二者会共同修改同一组状态变量。更稳妥的做法是分别编译或运行两个版本，再比较输出。

# 8. 常见问题定位

## 8.1 Host CK 可以编译，Device CK编译失败，

优先检查 `UpdateKernel` 及其成员是否包含：

- 虚函数对象；
- Host-only 指针；
- `std::vector` 等动态容器；
- 文件和日志操作；
- 未验证可在 Device 上调用的第三方函数；
- 无法复制到 Device 的自定义类型。

## 8.2 程序能编译但运行时报内存错误

检查

- 是否通过 `DelegatedData(ex_policy)` 取得数据；
- 指针指向的变量是否仍然存活；
- 数组是否按粒子数量正确分配；
- `index_i` 是否可能越界；
- Host 和 Device 数据是否在需要时完成同步。

## 8.3 Host 与 Device 数值不同

应先缩小到少量粒子，并逐步检查：

1. 初始变量是否一致；
2. `UpdateKernel` 得到的成员值是否一致；
3. 单个粒子的 `update()` 是否一致；
5. 前后 Dynamics 的执行顺序是否一致。

## 8.4 Work-group 大小、寄存器或 `nd_range` 错误

应把问题与物理公式分开分析。`update()` 数学正确并不代表某个局部大小一定适合目标设备。

## 9. 本课总结

阅读一个 SPHinXsys SYCL Dynamics 时，可以固定按照下面的顺序追踪：

```text
用户案例中的 add...Dynamics
        │
        ▼
Execution Policy 和包装类型
        │
        ▼
外层 Dynamics 构造函数
        │
        ▼
变量对象的注册与取得
        │
        ▼
UpdateKernel 构造函数
        │
        ▼
DelegatedData(ex_policy)
        │
        ▼
update(index_i, dt)
        │
        ▼
StateDynamics::exec()
        │
        ▼
particle_for()
        │
        ▼
TBB 或 SYCL parallel_for
```

对于 `GravityForceCK<Gravity>`：

- 外层类负责取得粒子变量、物理时间和重力模型；
- `UpdateKernel` 把这些资源转换为计算所需的指针和值对象；
- `update()` 只处理一个粒子；
- `StateDynamics` 组织全部粒子的执行；
- `particle_for` 根据 Execution Policy 选择 TBB 或 SYCL；
- SYCL 路径把 Work-item 的全局编号映射为 `index_i`；
- `wait_and_throw()` 保证 `exec()` 返回前计算已经完成。

掌握这条阅读链后，再阅读 Interaction Dynamics、Reduction、邻居循环和 `ConstituteKernel` 时，核心方法仍然相同：先找单粒子或单邻域的计算对象，再向外追踪数据来源，最后向下追踪执行后端。
