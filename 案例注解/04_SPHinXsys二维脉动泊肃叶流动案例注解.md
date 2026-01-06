在`test_2d_pulsatile_poiseuille_flow`案例中，我们将学到

- 双向buffer（`BidirectionalBuffer`）的使用
- 压力边界的指定
- 指定压力边界时如何进行density summation
- Riemann solver的取舍

注意区分本文的两个概念：buffer区域的粒子和buffer particle。前者指的是位于buffer区域（某个aligned box）的粒子，属于real particle。后者指的是内存空间中预留的空位置，属性没有任何意义，不属于real particle。

# TargetPressure

用于指定压力。这个类是一个函数对象，具有以下签名：

```cpp
Real TargetPressure::operator()(Real p, Real current_time);
```

在本例中，作者在管道左端和右端分别设置了一个`TargetPressure`：

```cpp
struct LeftInflowPressure
{
    template <class BoundaryConditionType>
    LeftInflowPressure(BoundaryConditionType &boundary_condition) {}

    Real operator()(Real p, Real physical_time)
    {
        /*pulsatile pressure*/
        Real pressure = Inlet_pressure * cos(physical_time);
        /*constant pressure*/
        //        Real pressure = Inlet_pressure;
        return pressure;
    }
};

struct RightInflowPressure
{
    template <class BoundaryConditionType>
    RightInflowPressure(BoundaryConditionType &boundary_condition) {}

    Real operator()(Real p, Real physical_time)
    {
        /*constant pressure*/
        Real pressure = Outlet_pressure;
        return pressure;
    }
};
```

左端是$$p_\mathrm{in}\cos(t)$$，右端是$$p_\mathrm{out}$$。

# 流入/流出双向buffer

因为流体在压力边界上可能流入也可能流出，因此这里需要一个双向的buffer（而非仅仅emitter或仅仅disposer）。`BidirectionalBuffer`实现了这一功能。它一共做三件事情：

1. 每一步对buffer区域的粒子进行动态标记
2. 流体从buffer的lower bound流出时，删除之（real particle转为buffer particle）
3. 流体从buffer的upper bound流出时，注入之（buffer particle转为real particle）

对于底层是如何构造和更新的，见[fluid_boundary](../源码剖析/particle_dynamics/fluid_dynamics/fluid_boundary.md)。

## 定义

```cpp
    AlignedBox left_emitter_shape(xAxis, Transform(Vec2d(left_bidirectional_translation)), bidirectional_buffer_halfsize);
    AlignedBoxByCell left_emitter(water_block, left_emitter_shape);
    fluid_dynamics::BidirectionalBuffer<LeftInflowPressure> left_bidirection_buffer(left_emitter, in_outlet_particle_buffer);
```

当我们定义一个`BidirectionalBuffer`时，需要提供三个东西：

1. `TargetPressure`类，用于在注入时将periodic bounding回lower bound的粒子压力设为指定压力。这里`TargetPressure`是`LeftInflowPressure`。
2. `AlignedBoxByCell`对象，用于确定buffer区域。
3. `ParticleBuffer<Base>`（或其派生类）对象，用于在流体注入时`checkEnoughBuffer`。

注意，这个类还会用到`previous_surface_indicator_`，因此它必须在定义`FreeSurfaceIndication<Inner<SpatialTemporal>>`或`*SpatialTemporalFreeSurfaceIndicationComplex`之后方能定义。

对于出口附近的双向buffer，需要特别注意：**要将aligned box翻转180度（Pi）**。这是因为出口处的正向速度代表流出（删除），反向速度代表流入（注入），与入口处的刚好相反。

```cpp
    AlignedBox right_emitter_shape(xAxis, Transform(Rotation2d(Pi), Vec2d(right_bidirectional_translation)), bidirectional_buffer_halfsize);
    AlignedBoxByCell right_emitter(water_block, right_emitter_shape);
    fluid_dynamics::BidirectionalBuffer<RightInflowPressure> right_bidirection_buffer(right_emitter, in_outlet_particle_buffer);
```

## 执行

```cpp
    sph_system.initializeSystemCellLinkedLists();
    sph_system.initializeSystemConfigurations();
    boundary_indicator.exec();
    left_bidirection_buffer.tag_buffer_particles.exec(); // 1
    right_bidirection_buffer.tag_buffer_particles.exec();
	...
    while (physical_time < end_time)
    {
        ...
        while (integration_time < Output_Time)
        {
            ...
            left_bidirection_buffer.injection.exec(); // 2
            right_bidirection_buffer.injection.exec();
            left_bidirection_buffer.deletion.exec(); // 3
            right_bidirection_buffer.deletion.exec();

            water_block.updateCellLinkedList();
            water_block_complex.updateConfiguration();
            interval_updating_configuration += TickCount::now() - time_instance;
            boundary_indicator.exec();
            left_bidirection_buffer.tag_buffer_particles.exec(); // 4
            right_bidirection_buffer.tag_buffer_particles.exec();
            ...
        }
    }
```

1. 初始化邻居表后先标记一次buffer区域的粒子；
2. 在中层循环中进行粒子注入；
3. 在中层循环中进行粒子删除；
4. 在中层循环中、更新邻居表之后，进行buffer区域粒子的重新标记。

# 压力边界

底层的具体实现见[fluid_boundary](../源码剖析/particle_dynamics/fluid_dynamics/fluid_boundary.md)。用户只需要知道压力边界主要做的事是在动量方程中加入一项$$2 p_\mathrm{b} \sum_j m_j/(\rho_i \rho_j) \nabla W_{i j}$$，由此我们能看出需要提供三个东西：

1. 目标压力$$p_\mathrm{b}$$，由`TargetPressure`作为模板参数提供。这里是`LeftInflowPressure`和`RightInflowPressure`。
2. 邻居粒子的核梯度之和$$\sum_j (m_j/\rho_j)\nabla W_{ij}$$，`PressureCondition`会在底层调用之。这里由`kernel_summation`提供。因此`kernel_summation.exec()`必须放在`...inflow_pressure_condition.exec(dt)`之前。
3. 这一项是加到动量方程中的，因此`pressure_relaxation.exec(dt)`必须放在`...inflow_pressure_condition.exec(dt)`之前。

```cpp
	InteractionDynamics<NablaWVComplex> kernel_summation(water_block_inner, water_block_contact);
	...
    SimpleDynamics<fluid_dynamics::PressureCondition<LeftInflowPressure>> left_inflow_pressure_condition(left_emitter);
    SimpleDynamics<fluid_dynamics::PressureCondition<RightInflowPressure>> right_inflow_pressure_condition(right_emitter);
	...
    while (physical_time < end_time)
    {
        ...
        while (integration_time < Output_Time)
        {
            ...
            while (relaxation_time < Dt)
            {
                ...
                pressure_relaxation.exec(dt);
                kernel_summation.exec();
                left_inflow_pressure_condition.exec(dt);
                right_inflow_pressure_condition.exec(dt);
                density_relaxation.exec(dt);
				...
            }
            ...
        }
    }
```

# density summation

buffer区域的粒子已经在`BidirectionalBuffer`中通过给定的压力指定了密度。就不用对这些粒子再做density summation了（何况本来支撑域也不完整）。只需要对内部流动区域的粒子做density summation。通过`DensitySummationPressureComplex`实现这一点。

```cpp
    InteractionWithUpdate<fluid_dynamics::DensitySummationPressureComplex> update_fluid_density(water_block_inner, water_block_contact);
	...
    while (physical_time < end_time)
    {
        Real integration_time = 0.0;
        /** Integrate time (loop) until the next output time. */
        while (integration_time < Output_Time)
        {
            ...
            update_fluid_density.exec();
            ...
        }
        ...
    }
```

# Riemann solver

在之前的案例中，density relaxation都没有采用Riemann solver。但是本例中density relaxation采用了Riemann solver。

在 SPHinXsys 这套弱可压（WCSPH）框架里，“要不要用 Riemann solver”本质上是在取舍两件事：

- **稳定性 / 抑制数值噪声（尤其是边界附近、强瞬变、开口边界）**
- **数值耗散（Riemann会更“粘”，更耗散，速度/剪切层会更容易被抹平）**

##  Riemann  VS NoRiemann大概在做什么

- `Integration...WithWallRiemann`（你提到的 `AcousticRiemannSolver`）  
  
  用“声学近似的 Riemann 解算器”在粒子对（含壁面接触）上构造界面状态（压力/速度等），相当于给通量加了更强的**上风/耗散**机制。常见收益：
  
  - 更稳：密度/压力振荡更不容易炸
  - 对**开口边界**（inlet/outlet、buffer、pressure BC）通常更友好，反射/噪声更小
  - 对瞬态强加速、局部强压缩（虽然 WCSPH 理论上低马赫）更鲁棒
  
- `Integration...WithWallNoRiemann`  
  
  不走 Riemann 通量构造，通常更接近“传统 SPH 压力梯度/连续方程离散”的那种风格，**耗散更低**、结果更“锐”，但也更吃参数（时间步长、人工粘性/修正、壁面处理质量）。

一句话：**Riemann 更稳更耗散；NoRiemann 更不耗散但更容易起噪/不稳。**

---

## 为什么不同例子会选不同的2nd half

- **pulsatile_poiseuille_flow 用 `Integration2ndHalfWithWallRiemann`** 
  这个案例里有明显的 **压力边界 + 双向 buffer（injection/deletion）**，而且是**脉动**工况（时间变化边界驱动）。这类“开口边界 + 非定常驱动”特别容易在边界/缓冲区附近出现密度/压力的数值波反射和噪声；用 Riemann（声学）通常能更稳、更不容易炸，所以会倾向选 `...Riemann`。

- **poiseuille_flow / channel_flow_shell 用 `Integration2ndHalfWithWallNoRiemann`** 
  这俩属于典型“粘性主导/低马赫”的内部流或周期/封闭性质更强的设置（尤其 channel_flow_shell 还是周期 x）。这类工况更在意速度剖面、剪切层的准确性，**不希望额外耗散把剖面抹平**，所以常见选择是 `NoRiemann`，前提是整体数值足够稳定。

## 选择准则

- 目标是“更稳先跑通/边界更干净”：选 `...Riemann`
- 目标是“粘性流剖面/剪切细节更准、耗散更小”：选 `...NoRiemann`

