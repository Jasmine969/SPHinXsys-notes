上一个学习案例是溃坝。在`test_2d_channel_flow_fluid_shell`案例中将学习到的知识点有：

- 自己写一个`ParticleGenerator`
- 流入速度设定
- `defineMaterial`和`defineClosure`的区别
- 周期性边界
- 流壳耦合（壁面只有单层粒子）

# 命令行参数

这个案例与二维溃坝的一大区别在于使用了google test，没有用回归测试。所以命令行参数是google test的参数。这里省略。

# 全局参数

```cpp
#include "sphinxsys.h"
#include <gtest/gtest.h>
using namespace SPH;

//----------------------------------------------------------------------
//	Basic geometry parameters and numerical setup.
//----------------------------------------------------------------------
const Real DL = 10.0; /**< Channel length. */
const Real DH = 2.0;  /**< Channel height. */
//----------------------------------------------------------------------
//	Global parameters on the fluid properties
//----------------------------------------------------------------------
const Real rho0_f = 1.0;                  /**< Density. */
const Real U_f = 1.0;                     /**< Characteristic velocity. */
const Real c_f = 10.0 * U_f;              /**< Speed of sound. */
const Real Re = 100.0;                    /**< Reynolds number. */
const Real mu_f = rho0_f * U_f * DH / Re; /**< Dynamics viscosity. */
```

# 几何定义

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251216194154.png)

```cpp
//	Define case dependent geometries
//----------------------------------------------------------------------
namespace SPH
{
class WaterBlock : public MultiPolygonShape
{
  public:
    class FluidAxialObserver;
    explicit WaterBlock(const std::vector<Vecd> &shape, const std::string &shape_name) : MultiPolygonShape(shape_name)
    {
        multi_polygon_.addAPolygon(shape, ShapeBooleanOps::add);
    }
};
    
void channel_flow_shell(...
	...
    /** create a water block shape */
    auto createWaterBlockShape = [&]()
    {
        // geometry
        std::vector<Vecd> water_block_shape;
        water_block_shape.push_back(Vecd(-DL_sponge, 0.0));
        water_block_shape.push_back(Vecd(-DL_sponge, DH));
        water_block_shape.push_back(Vecd(DL, DH));
        water_block_shape.push_back(Vecd(DL, 0.0));
        water_block_shape.push_back(Vecd(-DL_sponge, 0.0));

        return water_block_shape;
    };
	...
}
```

这里定义了水体的形状类，它继承于多边形类`MultiPolygonShape`，底层是用boost geometry实现的。用户需要为构造函数传入两个参数：储存多个矢量的vector（多边形的顶点集）和形状名称。`channel_flow_shell`函数中的`createWaterBlockShape`就生成了所需要的vector。

```cpp
/** Particle generator and constraint boundary for shell baffle. */
class WallBoundary;
template <>
class ParticleGenerator<SurfaceParticles, WallBoundary> : public ParticleGenerator<SurfaceParticles>
{
    Real DL_sponge_;
    Real BW_;
    Real resolution_ref_;
    Real wall_thickness_;

  public:
    explicit ParticleGenerator(SPHBody &sph_body, SurfaceParticles &surface_particles,
                               Real resolution_ref, Real wall_thickness)
        : ParticleGenerator<SurfaceParticles>(sph_body, surface_particles),
          DL_sponge_(20 * resolution_ref), BW_(4 * resolution_ref),
          resolution_ref_(resolution_ref), wall_thickness_(wall_thickness) {};
    void prepareGeometricData() override
    {
        auto particle_number_mid_surface = int((DL + DL_sponge_ + 2 * BW_) / resolution_ref_);
        for (int i = 0; i < particle_number_mid_surface; i++)
        {
            Real x = -DL_sponge_ - BW_ + (Real(i) + 0.5) * resolution_ref_;
            // upper wall
            Real y1 = DH + 0.5 * resolution_ref_;
            addPositionAndVolumetricMeasure(Vecd(x, y1), resolution_ref_);
            Vec2d normal_direction_1 = Vec2d(0, 1.0);
            addSurfaceProperties(normal_direction_1, wall_thickness_);
            // lower wall
            Real y2 = -0.5 * resolution_ref_; // lower wall
            addPositionAndVolumetricMeasure(Vecd(x, y2), resolution_ref_);
            Vec2d normal_direction_2 = Vec2d(0, -1.0);
            addSurfaceProperties(normal_direction_2, wall_thickness_);
        }
    }
};
```

生成shell粒子一般有两种方法，第一种如同溃坝案例中一样：先定义shell的几何，然后用`generateParticles<SurfaceParticles, Lattice>()`生成粒子，见`ball_shell_collision.cpp`。第二种就是这里的方法：不定义几何（即采用默认几何），自己规定粒子位置和体积，因为是shell，我们还需要规定其表面性质，包括法方向和厚度。

具体而言，这里写了一个特化的`ParticleGenerator<SurfaceParticles, Style>`模板，它继承于`ParticleGenerator<SurfaceParticles>`。`WallBoundary`只有声明，没有定义，但是不需要定义，因为它的作用只是将此特化的`ParticleGenerator`和`src/`中的`ParticleGenerator`区分开来。`resolution_ref_`是粒子间距$\Delta L$；`DL_sponge`是施加流入条件的宽度（$20\Delta L$）；`BW`是壁面比水体多出的宽度（$4\Delta L$）；`wall_thickness_`是壁面（壳层）厚度。

我们将`ParticleGenerator`的`prepareGeometricData()`进行了覆盖。在函数体中，首先统计了有多少对壁面颗粒（一个上壁面+一个下壁面颗粒称为一对）。对于每一对颗粒，它的横坐标是`-DL_sponge_ - BW_ + (Real(i) + 0.5) * resolution_ref_`。这里之所以要给`Real(i)`加上0.5，是为了和流体粒子对齐，流体粒子将用lattcie生成，它们是在网格中心的。`y1`和`y2`分别将上壁面向上平移$\Delta L/2$和下壁面向下平移$\Delta L/2$。`addPositionAndVolumetricMeasure`的作用是添加位置信息和体积信息，二维模拟的shell是一维的，所以体积就是$\Delta L$。`addSurfaceProperties`的作用是添加壁面法向和厚度信息。

# 流入速度

流入速度显然是针对入口而言的，我们只需对入口处的粒子设置速度即可，这个入口正是几何中x负半轴的部分，称之为流入缓冲区（inflow buffer）。但是`water_block`是一个整体，如何告诉程序取其中的一部分呢？这就涉及body part的概念。body part是body的一部分。它可以让我们只模拟给定body的指定区域的粒子。这在程序中体现为`AlignedBoxByCell`。以下为流入速度设定的全部代码：

```cpp
//----------------------------------------------------------------------
//	Inflow velocity
//----------------------------------------------------------------------
struct InflowVelocity
{
    Real u_ref_, t_ref_;
    AlignedBox &aligned_box_;
    Vecd halfsize_;

    template <class BoundaryConditionType>
    InflowVelocity(BoundaryConditionType &boundary_condition)
        : u_ref_(U_f), t_ref_(2.0),
          aligned_box_(boundary_condition.getAlignedBox()),
          halfsize_(aligned_box_.HalfSize()) {}

    Vecd operator()(Vecd &position, Vecd &velocity, Real current_time)
    {
        Vecd target_velocity = velocity;
        Real u_ave = current_time < t_ref_ ? 0.5 * u_ref_ * (1.0 - cos(Pi * current_time / t_ref_)) : u_ref_;
        if (aligned_box_.checkInBounds(position))
        {
            target_velocity[0] = 1.5 * u_ave * (1.0 - position[1] * position[1] / halfsize_[1] / halfsize_[1]);
        }
        return target_velocity;
    }
};

void channel_flow_shell(...) {
    ...
    /** Inflow boundary condition. */
    AlignedBoxByCell inflow_buffer(
        water_block, AlignedBox(xAxis, Transform(Vec2d(buffer_translation)), buffer_halfsize));
    SimpleDynamics<fluid_dynamics::InflowVelocityCondition<InflowVelocity>> parabolic_inflow(inflow_buffer);
    ...
    while (...) {
        while (...) {
            while (...) {
                ...
                parabolic_inflow.exec();
                ...
            }
        }
    }
}
```

程序设计了一个类`InflowVelocity`。它的构造函数需要一个`BoundaryConditionType`对象`boundary_condition`作为参数，从`boundary_condition`中，先获取`aligned_box_`，再获取后者的`halfsize`。在main函数调用中可以看出，此aligned box就是`AlignedBox(xAxis, Transform(Vec2d(buffer_translation)), buffer_halfsize)`，它就是inflow buffer。`AlignedBox`类与`GeometricBox`类很像，只不过`AlignedBox`多了一个对齐轴（alignment axis），所有“上/下边界、周期映射、近边界判断”等操作都沿着这个对齐轴来做。

`InflowVelocity`的函数调用运算符被重载了，因此它实际上会生成一个函数对象。它有三个形参：位置、速度、当前时间。`u_ave`是随时间变化的平均速度，形式如下：
$$
\bar{u}=\begin{cases}
0.5u_\mathrm{ref}[1-\cos(\pi t/t_\mathrm{ref})], &t<t_\mathrm{ref};\\
u_\mathrm{ref}, &t\geq t_\mathrm{ref}.
\end{cases}
$$
![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251216201809.png)

这个函数使得平均速度从0平滑过渡到最大值。

接着检测给定点是否在aligned box（inflow buffer）内，如果是的话，则该位置非水平方向速度为原值（`Vecd target_velocity = velocity;`），水平速度为
$$
u_x=1.5\bar{u}\left[1-\frac{y^2}{\mathrm{(DH/2)}^2}\right].
$$
这是典型的抛物线分布，说明将入口流动设定为充分发展的流动。

定义完`InflowVelocity`类后，用`water_block`和aligned box定义了一个`AlignedBoxByCell`对象`inflow_buffer`。它用来初始化`SimpleDynamics<fluid_dynamics::InflowVelocityCondition<InflowVelocity>>`对象。使用`AlignedBoxByCell`的目的是允许程序在遍历时只遍历inflow buffer的粒子，而非体系所有粒子，以提高计算效率。使用`SimpleDynamics`是因为流入速度不涉及颗粒交互。只需要update即可。

## `SimpleDynamics`是如何构造的

让我们分析一下`SimpleDynamics<fluid_dynamics::InflowVelocityCondition<InflowVelocity>> parabolic_inflow(inflow_buffer);`这行定义会在底层做些什么。这里模板参数是嵌套的。

### 1. 开始构造`SimpleDynamics`

首先会调用最外层`SimpleDynamics`的构造函数：

```cpp
template <class LocalDynamicsType, class ExecutionPolicy = ParallelPolicy>
class SimpleDynamics : public LocalDynamicsType, public BaseDynamics<void>
{
  public:
    template <class DynamicsIdentifier, typename... Args>
    SimpleDynamics(DynamicsIdentifier &identifier, Args &&...args)
        : LocalDynamicsType(identifier, std::forward<Args>(args)...),
          BaseDynamics<void>()
    {
        static_assert(!has_initialize<LocalDynamicsType>::value &&
                          !has_interaction<LocalDynamicsType>::value,
                      "LocalDynamicsType does not fulfill SimpleDynamics requirements");
    };
    ...
}
```

`LocalDynamicsType`是用户显式指定的`InflowVelocityCondition<InflowVelocity>`，`ExecutionPolicy`保持默认值。`SimpleDynamics`继承于`LocalDynamicsType`（`InflowVelocityCondition`）和`BaseDynamics<void>`。于是此类分别获得了`InflowVelocityCondition<InflowVelocity>`的`update`、`setupDynamics`成员函数和`BaseDynamics<void>`的`exec`、`setUpdated`成员函数。

`inflow_buffer`作为参数传入`SimpleDynamics`的构造函数，因此`DynamicsIdentifier`被推断为`AlignedBoxByCell`，`args`为空参数包。

然后进入成员初始化列表。首先会调用基类构造函数。因为多重继承于两个基类，调用基类构造函数应按照派生列表中基类的出现顺序。因此，先调用`InflowVelocityCondition<InflowVelocity>`的构造函数。

### 2. 开始构造`InflowVelocityCondition<InflowVelocity>`

`fluid_dynamics::InflowVelocityCondition`模板的构造函数如下：

```cpp
template <typename TargetVelocity>
class InflowVelocityCondition : public BaseFlowBoundaryCondition
{
  public:
    /** default parameter indicates prescribe velocity */
    explicit InflowVelocityCondition(AlignedBoxByCell &aligned_box_part, Real relaxation_rate = 1.0)
        : BaseFlowBoundaryCondition(aligned_box_part),
          relaxation_rate_(relaxation_rate), aligned_box_(aligned_box_part.getAlignedBox()),
          transform_(aligned_box_.getTransform()), halfsize_(aligned_box_.HalfSize()),
          target_velocity(*this),
          physical_time_(sph_system_.getSystemVariableDataByName<Real>("PhysicalTime")) {};
    ...
  protected:
    Real relaxation_rate_;
    AlignedBox &aligned_box_;
    Transform &transform_;
    Vecd halfsize_;
    TargetVelocity target_velocity;
    Real *physical_time_;
}
```

类具有一个模板参数`TargetVelocity`类型的成员`target_velocity`。在这里，我们显式指定了`TargetVelocity`为`InflowVelocity`。在初始化列表中，`inflow_buffer`被传入了基类构造函数。

### 3. 开始构造`BaseFlowBoundaryCondition`

用传入的`inflow_buffer`构造`BaseFlowBoundaryCondition`。`BaseFlowBoundaryCondition`继承于`BaseLocalDynamics<BodyPartByCell>`。在构造`BaseLocalDynamics<BodyPartByCell>`时会传入`inflow_buffer`：

``` cpp
BaseFlowBoundaryCondition::BaseFlowBoundaryCondition(BodyPartByCell &body_part)
    : BaseLocalDynamics<BodyPartByCell>(body_part), ... {};
template <class DynamicsIdentifier>
class BaseLocalDynamics
{
  public:
    explicit BaseLocalDynamics(DynamicsIdentifier &identifier)
        : identifier_(identifier), sph_system_(identifier.getSPHSystem()),
          sph_body_(identifier.getSPHBody()),
          sph_adaptation_(&sph_body_.getSPHAdaptation()),
          particles_(&sph_body_.getBaseParticles()),
          logger_(Log::get()) {};
    ...
}
```

`DynamicsIdentifier`被推断为`BodyPartByCell`类型，而也就是说`identifier_`是一个绑定到`BodyPartByCell`类型的`AlignedBoxByCell`对象。`sph_body_`被初始化为`inflow_buffer`的`sph_body_`。从`inflow_buffer`定义中可以获知它就是`water_block`。

### 4. `BaseFlowBoundaryCondition`构造完毕

然后初始化`relaxation_rate`（默认），用传入的`inflow_buffer`的成员初始化`aligned_box_`、`transform_`、`halfsize_`，接着用自身初始化`target_velocity`。此时`InflowVelocity`的构造函数被调用。

### 5. 开始构造`InflowVelocity`

`InflowVelocity`的构造函数接受一个`BoundaryConditionType`的引用类型，显然它就是部分初始化的`InflowVelocityCondition`。这不会造成递归构造，因为是传引用调用，而非传值调用。

### 6. `InflowVelocity`构造完毕

当`target_velocity`初始化完毕后，获取了系统的物理时间。这个物理时间也就是`InflowVelocity`中要用的`current_time`。

### 7. `InflowVelocityCondition<InflowVelocity>`构造完毕

回到`SimpleDynamics`的构造函数，此时应该构造第二个基类。

### 8. 开始构造`BaseDynamics<void>`

### 9. `BaseDynamics<void>`构造完毕

此时`SimpleDynamics`的成员初始化列表任务完成，进入构造函数体。检查`InflowVelocityCondition<InflowVelocity>`是否具有`initialize`和`interaction`类，如果有，编译报错。因为`InflowVelocityCondition<InflowVelocity>`有的是`update`成员函数，所以不会报错。

### 10. `SimpleDynamics`构造完毕

## parabolic_inflow是如何执行的

### 1. 调用`SimpleDynamics<…>.exec()`

在模拟时，每隔一段时间会调用一次`parabolic_inflow.exec()`，也即调用以下函数：

```cpp
template <class LocalDynamicsType, class ExecutionPolicy = ParallelPolicy>
class SimpleDynamics : public LocalDynamicsType, public BaseDynamics<void>
{
    ...
	virtual void exec(Real dt = 0.0) override
    {
        this->setUpdated(this->identifier_.getSPHBody());
        this->setupDynamics(dt);
        particle_for(ExecutionPolicy(),
                     this->identifier_.LoopRange(),
                     [&](size_t i)
                     { this->update(i, dt); });
    };
}
```

`dt`采用默认值`0.0`。函数体中，先调用`this->setUpdated`，这来自`BaseDynamics<void>`。参数`this->identifier_.getSPHBody()`被解析为`water_block`。

### 2. 调用`BaseDynamics<void>::setUpdated()`

```cpp
template <class ReturnType = void>
class BaseDynamics
{
    ...
    template <class IdentifierType>
	void setUpdated(IdentifierType &identifier)
    {
        identifier.setNewlyUpdated();
        is_newly_updated_ = true;
    };
}
```

首先调用了`water_block`的`setNewlyUpdated()`，然后将自己的`is_newly_updated_`设置为`true`。

### 3. 退出`BaseDynamics<void>::setUpdated()`

然后调用`this->setupDynamics(dt);`。因为`InflowVelocityCondition<...>`和`BaseFlowBoundaryCondition`都没有定义自己的`setupDynamics`，因此调用基类的成员函数。

### 4. 调用`BaseLocalDynamics::setupDynamics(dt)`

```cpp
virtual void setupDynamics(Real dt = 0.0) {};
```

什么也不做。

### 5. 退出`BaseLocalDynamics::setupDynamics(dt)`

重点在于`particle_for`这一段。首先`ExecutionPolicy`采用的是默认的`ParallelPolicy`。`this->identifier_.LoopRange()`调用`BodyPartByCell`对象的`LoopRange()`方法，它返回的是`ConcurrentCellLists`对象。

```cpp
        particle_for(ExecutionPolicy(),
                     this->identifier_.LoopRange(),
                     [&](size_t i)
                     { this->update(i, dt); });
```

参数`local_dynamics_function`是一个lambda函数对象`[&](size_t i) { this->update(i, dt); })`。`[&]`以引用捕获方式捕获了`exec`函数体中所有变量，包括`dt`和`this`指针。它接受粒子索引`i`作为参数，在函数体中调用`InflowVelocityCondition<InflowVelocity>`的`update`成员函数。不用看`particle_for`源码我们就能猜出来，遍历`inflow_buffer`的每个粒子，对其调用`this->update`函数，采取并行模式。从这里可以看出来，使用`AlignedBoxByCell`对象的好处在于不用遍历`water_block`每个粒子。

# 体系定义

```cpp
void channel_flow_shell(...)
{
    Real DL_sponge = resolution_ref * 20.0; /**< Sponge region to impose inflow condition. */
    Real BW = resolution_ref * 4.0;         /**< Boundary width, determined by specific layer of boundary particles. */

    /** Domain bounds of the system. */
    BoundingBox system_domain_bounds(Vec2d(-DL_sponge - BW, -wall_thickness), Vec2d(DL + BW, DH + wall_thickness));
    //----------------------------------------------------------------------
    //	Build up the environment of a SPHSystem with global controls.
    //----------------------------------------------------------------------
    SPHSystem sph_system(system_domain_bounds, resolution_ref);
    ...
}
```

# 创建SPHBody

```cpp
void channel_flow_shell(...)
{
    ...
	//	Creating body, materials and particles.
    //----------------------------------------------------------------------
    FluidBody water_block(sph_system, makeShared<WaterBlock>(createWaterBlockShape(), "WaterBody"));
    water_block.defineClosure<WeaklyCompressibleFluid, Viscosity>(ConstructArgs(rho0_f, c_f), mu_f);
    water_block.generateParticles<BaseParticles, Lattice>();

    SolidBody wall_boundary(sph_system, makeShared<DefaultShape>("Wall"));
    wall_boundary.defineMaterial<Solid>();
    wall_boundary.generateParticles<SurfaceParticles, WallBoundary>(resolution_ref, wall_thickness);
    ...
}
```

在二维溃坝案例中，我们使用`water_block.defineMaterial<WeaklyCompressibleFluid>(rho0_f, c_f);`来定义材料属性。这里却没有用`defineMaterial`，而是用了`defineClosure`。这是因为除了`WeaklyCompressibleFluid`还有`Viscosity`。`defineMaterial`和`defineClosure`的核心区别是：一个只创建“单一材料模型(`BaseMaterial`派生类)”，另一个创建“物理闭包（`Closure`）”，把多个材料/本构/附加模型组合成一个整体材料对象。在 SPHinXsys 里二者最终都会把`SPHBody::base_material_`指向你创建的对象，但对象类型和用途不同。`WeaklyCompressibleFluid`需要两个参数：密度和声速，`Viscosity`需要一个参数：`mu_f`。为了让编译器知道哪些参数是传给`WeaklyCompressibleFluid`的，我们需要用`ConstructArgs`吧`rho_f`和`c_f`打包。

# 定义关系

```cpp
void channel_flow_shell(...)
{
    ...
	//	Define body relation map.
    //	The contact map gives the topological connections between the bodies.
    //	Basically the the range of bodies to build neighbor particle lists.
    //----------------------------------------------------------------------
    InnerRelation water_block_inner(water_block);
    InnerRelation shell_boundary_inner(wall_boundary);
    ShellInnerRelationWithContactKernel shell_curvature_inner(wall_boundary, water_block);
    // shell normal should point from fluid to shell
    // normal corrector set to false if shell normal is already pointing from fluid to shell
    ContactRelationFromShellToFluid water_block_contact(water_block, {&wall_boundary}, {false});
    ContactRelation fluid_axial_observer_contact(fluid_axial_observer, {&water_block});
    ContactRelation fluid_radial_observer_contact(fluid_radial_observer, {&water_block});
    //----------------------------------------------------------------------
    // Combined relations built from basic relations
    // and only used for update configuration.
    //----------------------------------------------------------------------
    ComplexRelation water_block_complex(water_block_inner, water_block_contact);
    ...
}
```

`shell_boundary_inner`似乎是个多余的关系，后面并没有调用。`shell_curvature_inner`后面被调用了，但是壁面是固定的，所以曲率恒为零，所以这个关系其实也多余。我尝试把相关代码注释掉，发现结果没有变化。

壳体与流体的关系有两种：

- 一种是`ContactRelationFromShellToFluid`。它“主体”是流体，更新的是每个流体粒子的contact neighbor list。在邻域构建时，会把接触体当作shell，使用shell的NormalDirection/Thickness/Average1stPrincipleCurvature等信息，对核函数权重做“沿法向dummy外推”的修正。因此，`ContactRelationFromShellToFluid`构造函数的第一个参数是主体`water_block`，第二个参数是接触体vector`{&wall_boundary}`，第三个参数指定每一个接触体的法向是否要修正。接触体的法向应当从流体指向壁面，如果需要修正，这里输入`true`，告诉程序把壁面法向反向。这里，因为我们已经在`ParticleGenerator<SurfaceParticles, WallBoundary>`中正确定义了方向，所以这里填`false`。
- 另一种是`ContactRelationFromFluidToShell`，刚好反过来。“主体”是壳体：更新的是每个壳粒子的 contact neighbor list。用途是壳体侧要从流体取信息/受力（例如壳体受流体压力、FSI耦合时的力学更新等）。

这里显然我们应该用`ContactRelationFromShellToFluid`。

# 定义数值方法

# 观测数据与模拟验证

```cpp
StdVec<Vecd> createFluidAxialObservationPoints(Real resolution_ref)
{
    StdVec<Vecd> observation_points;
    /** A line of measuring points at the entrance of the channel. */
    size_t number_observation_points = 51;
    Real range_of_measure = DL - resolution_ref * 4.0;
    Real start_of_measure = resolution_ref * 2.0;
    Real y_position = 0.5 * DH;
    /** the measuring locations */
    for (size_t i = 0; i < number_observation_points; ++i)
    {
        Vec2d point_coordinate(range_of_measure * (Real)i / (Real)(number_observation_points - 1) + start_of_measure, y_position);
        observation_points.push_back(point_coordinate);
    }
    return observation_points;
};

StdVec<Vecd> createFluidRadialObservationPoints(Real resolution_ref)
{
    StdVec<Vecd> observation_points;
    /** A line of measuring points at the entrance of the channel. */
    size_t number_observation_points = 21;
    Real range_of_measure = DH - resolution_ref * 4.0;
    Real start_of_measure = resolution_ref * 2.0;
    Real x_position = 0.5 * DL;
    /** the measuring locations */
    for (size_t i = 0; i < number_observation_points; ++i)
    {
        Vec2d point_coordinate(x_position, range_of_measure * (Real)i / (Real)(number_observation_points - 1) + start_of_measure);
        observation_points.push_back(point_coordinate);
    }
    return observation_points;
};

void channel_flow_shell(const Real resolution_ref, const Real wall_thickness) {
    ...
    ObserverBody fluid_axial_observer(sph_system, "FluidAxialObserver");
    fluid_axial_observer.generateParticles<ObserverParticles>(createFluidAxialObservationPoints(resolution_ref));

    ObserverBody fluid_radial_observer(sph_system, "FluidRadialObserver");
    fluid_radial_observer.generateParticles<ObserverParticles>(createFluidRadialObservationPoints(resolution_ref));
    ...
    ContactRelation fluid_axial_observer_contact(fluid_axial_observer, {&water_block});
    ContactRelation fluid_radial_observer_contact(fluid_radial_observer, {&water_block});
    ...
    ObservedQuantityRecording<Vecd> write_fluid_axial_velocity("Velocity", fluid_axial_observer_contact);
    ObservedQuantityRecording<Vecd> write_fluid_radial_velocity("Velocity", fluid_radial_observer_contact);
    ...
    while (...) {
        ...
        write_fluid_axial_velocity.writeToFile(number_of_iterations);
        write_fluid_radial_velocity.writeToFile(number_of_iterations);
        fluid_axial_observer_contact.updateConfiguration();
        fluid_radial_observer_contact.updateConfiguration();
    }
    ...
    /**
     * @brief 	Gtest start from here.
     */
    /* Define analytical solution of the inflow velocity.*/
    std::function<Vec2d(Vec2d)> inflow_velocity = [&](Vec2d pos)
    {
        Real y = 2 * pos[1] / DH - 1;
        return Vec2d(1.5 * U_f * (1 - y * y), 0);
    };
    /* Compare all simulation to the analytical solution. */
    // Axial direction.
    BaseParticles &fluid_axial_particles = fluid_axial_observer.getBaseParticles();
    Vecd *pos_axial = fluid_axial_particles.ParticlePositions();
    Vecd *vel_axial = fluid_axial_particles.getVariableDataByName<Vecd>("Velocity");
    for (size_t i = 0; i < fluid_axial_particles.TotalRealParticles(); i++)
    {
        EXPECT_NEAR(inflow_velocity(pos_axial[i])[1], vel_axial[i][1], U_f * 5e-2);
    }
    // Radial direction
    BaseParticles &fluid_radial_particles = fluid_radial_observer.getBaseParticles();
    Vecd *pos_radial = fluid_radial_particles.ParticlePositions();
    Vecd *vel_radial = fluid_radial_particles.getVariableDataByName<Vecd>("Velocity");
    for (size_t i = 0; i < fluid_radial_particles.TotalRealParticles(); i++)
    {
        EXPECT_NEAR(inflow_velocity(pos_radial[i])[1], vel_radial[i][1], U_f * 5e-2);
    }
}
```

生成`ObserverBody`的粒子需要传入观测点。我们需要首先定义这些观测点的位置，放在标准库vector中（`StdVec`）。轴向观测点均布位于x轴正半轴的水体的对称线上。径向观测点均布于$x=\mathrm{DL}/2$处的垂直线上。`createFluidAxialObservationPoints(resolution_ref)`返回的是储存所有观测点坐标的`StdVec`，它用来生成观测粒子。

在每次写入观测点数据之前，需要更新其邻居表。

模拟结束时，用google test验证模拟结果。首先定义了解析解。这里使用lambda函数的原因是函数体中不能定义新的普通函数。为了获取观测点的速度，我们首先通过`getBaseParticles()`获取观测点粒子。通过`BaseParticles`的`ParticlePositions`获取颗粒位置，它应该和`createFluidAxialObservationPoints`返回的没有区别，然后通过`BaseParticles`的`getVariableDataByName<Vecd>("Velocity")`获取粒子速度。接着将每一个观测点的速度与理论值进行比较，规定容差为$0.05U_f$。

这个案例使用Google test做验证，main函数中的`RUN_ALL_TESTS()`会运行所有`TEST`中的测试。`TEST`第一个参数是测试的函数名，第二个参数是本轮测试的名字。

```cpp
TEST(channel_flow_shell, thickness_10x)
{ // for CI
    const Real resolution_ref = 0.05;
    const Real wall_thickness = 10 * resolution_ref;
    channel_flow_shell(resolution_ref, wall_thickness);
}

//----------------------------------------------------------------------
//	Main program starts here.
//----------------------------------------------------------------------
int main(int ac, char *av[])
{
    testing::InitGoogleTest(&ac, av);
    return RUN_ALL_TESTS();
}
```

