在`test_2d_pulsatile_poiseuille_flow`案例中，我们将学到

- 双向buffer（`BidirectionalBuffer`）的使用
- 压力边界的指定

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

