在`test_3d_poiseuille_flow_shell`案例中，我们将学到

- 三维模型的建立与模拟
- 流入流出边界
- particle_bound_与buffer particle的概念
- `AlignedBoxByParticle`和`AlignedBoxByCell`的区别
- 多分辨率粒子分布（不同body采用不同分辨率，但同一body分辨率相同）
- 自定义光滑长度

这个教程中，我们不区分分辨率（resolution）与粒子间距（particle spacing）的概念，认为两者等价。

# body创建

这是壁面与流体的几何的左端局部视图。壁面的粒子间距（$$\Delta L_\mathrm{s}$$）是流体的（$$\Delta L_\mathrm{f}$$）一半。壁面比流体的边界多延伸出$$4\Delta L_\mathrm{f}$$的距离（程序中称为`wall_thickness`，记为$$\delta_\mathrm{s}$$），用来放置入口和出口的buffer。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251224100332.png)

流体网格的左边界为$$y=0$$的位置。流体粒子和壁面粒子都被放置在网格的中心。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251224100023.png)

于是我们可以计算出每个壁面粒子（序号为$$j$$，从零开始）的y坐标为：
$$
y_j=\delta_\mathrm{s}+j\Delta L_\mathrm{s}+\Delta L_\mathrm{s}/2
$$

## 壁面几何与body

圆管壁面是一个单层的柱面，y轴是圆管轴线方向，每个纵截面是一个有65个粒子的环，轴线上一共有116个环，这样总共有7540个壁面粒子。

```cpp
//	Basic geometry parameters and numerical setup.
//----------------------------------------------------------------------
const Real scale = 0.001;
const Real diameter = 6.35 * scale;
const Real fluid_radius = 0.5 * diameter;
const Real full_length = 10 * fluid_radius;

class ShellBoundary;
template <>
class ParticleGenerator<SurfaceParticles, ShellBoundary> : public ParticleGenerator<SurfaceParticles>
{
    Real resolution_shell_;
    Real wall_thickness_;
    Real shell_thickness_;

  public:
    explicit ParticleGenerator(SPHBody &sph_body, SurfaceParticles &surface_particles,
                               Real resolution_shell, Real wall_thickness, Real shell_thickness)
        : ParticleGenerator<SurfaceParticles>(sph_body, surface_particles),
          resolution_shell_(resolution_shell),
          wall_thickness_(wall_thickness), shell_thickness_(shell_thickness) {};
    void prepareGeometricData() override
    {
        const Real radius_mid_surface = fluid_radius + resolution_shell_ * 0.5;
        const auto particle_number_mid_surface =
            int(2.0 * radius_mid_surface * Pi / resolution_shell_);
        const auto particle_number_height =
            int((full_length + 2.0 * wall_thickness_) / resolution_shell_);
        for (int i = 0; i < particle_number_mid_surface; i++)
        {
            for (int j = 0; j < particle_number_height; j++)
            {
                Real theta = (i + 0.5) * 2 * Pi / (Real)particle_number_mid_surface;
                Real x = radius_mid_surface * cos(theta);
                Real y = -wall_thickness_ + (full_length + 2 * wall_thickness_) * j / (Real)particle_number_height + 0.5 * resolution_shell_;
                Real z = radius_mid_surface * sin(theta);
                addPositionAndVolumetricMeasure(Vec3d(x, y, z),
                                                resolution_shell_ * resolution_shell_);
                Vec3d n_0 = Vec3d(x / radius_mid_surface, 0.0, z / radius_mid_surface);
                addSurfaceProperties(n_0, shell_thickness_);
            }
        }
    }
};
```

壁面几何中有两个thickness需要区分。一个是`wall_thickness`，这是壁面比流体在y方向多延伸出的长度；另一个是`shell_thickness`，这才是壁面的厚度。

上面的`prepareGeometricData()`规定了每个壁面粒子的位置、体积、法方向和壁面厚度。x和z坐标由圆的参数方程给出，注意半径比预期的多了$$\Delta L_\mathrm{s}/2$$，以避免和流体粒子重叠。y坐标的公式已经在上面给出，只不过程序里将最后一项做了等量代换（其实没必要，还更麻烦了）。对于壳体，粒子的体积是$$(\Delta L_\mathrm{s})^2$$，法向量是外法向（流体指向壁面）。

```cpp
void poiseuille_flow(const Real resolution_ref, const Real resolution_shell, const Real shell_thickness) {
    const Real wall_thickness = resolution_ref * 4.0;
    ...
    SolidBody shell_boundary(system, makeShared<DefaultShape>("Shell"));
    shell_boundary.defineAdaptation<SPH::SPHAdaptation>(1.15, resolution_ref / resolution_shell);
	shell_boundary.defineMaterial<LinearElasticSolid>(1, 1e3, 0.45);
    shell_boundary.generateParticles<SurfaceParticles, ShellBoundary>(resolution_shell, wall_thickness, shell_thickness);
    ...
}

TEST(poiseuille_flow, 10_particles)
{ // for CI
    const int number_of_particles = 10;
    const Real resolution_ref = diameter / number_of_particles;
    const Real resolution_shell = 0.5 * resolution_ref;
    const Real shell_thickness = 0.5 * resolution_shell;
    poiseuille_flow(resolution_ref, resolution_shell, shell_thickness);
}
```

注意shell具有自己的分辨率，是全局分辨率的一半。这就涉及到**多分辨率**的问题。因为shell的粒子间距是全局的一半，如果shell也采用全局光滑长度进行平滑，势必会过度平滑，造成失真（见下图）。因此，我们有必要相应减小shell的光滑长度，这通过`defineAdaptation<SPH::SPHAdaptation>(...)`来实现。括号中传入的

- 第一个参数是光滑长度与粒子间距之比（$$h/\Delta L$$），默认值是1.3，设为1.15则有$$h=1.15\Delta L$$；
- 第二个参数是全局分辨率与本body分辨率之比，默认指是1。这里是按照定义给的：`resolution_ref`是全局分辨率，`resolution_shell`是shell的分辨率。

`defineMaterial`将材料设置为线弹性材料，似是没有必要（见下图），因为这个案例中壁面是固定不动的。我认为改成`Solid`也可以。

在下图中，我模拟了几个case。纵坐标是轴向观测点`Velocity[15][1]`的数据，横坐标是物理时间。original-0、original-1、original-2表示用原始程序跑三次出的结果（每次跑结果都不一样，跑三次可以看出大致趋势）；Solid是将线弹性材料改为`Solid`并不传递任何参数的结果。Solid-NoAdapt在Solid基础上进一步取消了Adaptation。Solid-NoAdapt_Single_resoltion在Solid-NoAdapt的基础上将流体粒子的间距减小为shell粒子的间距，这样就是单一分辨率了。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251223200622.png)

我们可以看到，将线弹性材料改为`Solid`没有任何影响。多分辨率场景中去掉Adaptation使得结果振荡且偏离原值，但改为单一分辨率后又回复至原值且稳定（看稳态结果）。这说明在多分辨率场景中有必要使用Adaptation修改默认设置。

## 流体几何与body

```cpp
    const int SimTK_resolution = 20;
    const Vec3d translation_fluid(0., full_length * 0.5, 0.);
	...
	auto water_block_shape = makeShared<ComplexShape>("WaterBody");
    water_block_shape->add<TriangleMeshShapeCylinder>(SimTK::UnitVec3(0., 1., 0.), fluid_radius,
                                                      full_length * 0.5, SimTK_resolution,
                                                      translation_fluid);

    //----------------------------------------------------------------------
    //  Build up -- a SPHSystem --
    //----------------------------------------------------------------------
    SPHSystem system(system_domain_bounds, resolution_ref);
    //----------------------------------------------------------------------
    //	Creating bodies with corresponding materials and particles.
    //----------------------------------------------------------------------
    FluidBody water_block(system, water_block_shape);
    water_block.defineClosure<WeaklyCompressibleFluid, Viscosity>(ConstructArgs(rho0_f, c_f), mu_f);
    ParticleBuffer<ReserveSizeFactor> inlet_particle_buffer(0.5);
    water_block.generateParticlesWithReserve<BaseParticles, Lattice>(inlet_particle_buffer);
```

使用`TriangleMeshShapeCylinder`创建了流体的几何。五个参数依次为：轴线方向矢量、流体半径、半长度、网格精度和圆柱中心点坐标。关于SimTK的网格精度，在[几何创建](../几何创建.md)中已有介绍，在此不赘。这样一来，流体的几何位于$$0\leq y \leq 10\Delta L_\mathrm{f}$$的区间内。

使用`inlet_particle_buffer`进行了粒子生成。`generateParticlesWithReserve<BaseParticles, Lattice>(inlet_particle_buffer)`其实与`generateParticles<BaseParticles, Lattice>()`差不多，区别是将`particles_bound_`递增了`buffer_size`（因为用户传入的是0.5，所以递增real particle数目的一半），并且设置`inlet_particle_buffer`的`is_particles_reserved_`属性为`true`。

**理解particle_bound_与buffer particle**

`particles_bound_`的概念类似于标准库vector的capacity，而real particle数目类似于标准库vector的size。递增之后，就会**多分配一段大小为`buffer_size`个粒子信息所需的内存**。我们称这些额外内存段上的粒子为buffer particle。buffer particle不是真正的particle，它们的粒子属性没有意义，不会参与interaction，更不会写入VTP文件。buffer particle的意义是为inflow的粒子提前分配内存空间（空房间），不然在并行模拟时动态分配内存开销是很大的。

# 入口流速

入口流速设定为
$$
\boldsymbol{v}=(0,2U_f(1-r^2/R^2),0)
$$
其中$$r(x,z)=x^2+z^2$$。

```cpp
//----------------------------------------------------------------------
//	Inflow velocity
//----------------------------------------------------------------------
struct InflowVelocity
{
    Real u_ref_, t_ref_;
    AlignedBox &aligned_box_;
    Vec3d halfsize_;

    template <class BoundaryConditionType>
    InflowVelocity(BoundaryConditionType &boundary_condition)
        : u_ref_(U_f), t_ref_(1.0),
          aligned_box_(boundary_condition.getAlignedBox()),
          halfsize_(aligned_box_.HalfSize()) {}

    Vec3d operator()(Vec3d &position, Vec3d &velocity, Real current_time)
    {
        Vec3d target_velocity = Vec3d(0, 0, 0);
        target_velocity[1] = SMAX(2.0 * U_f *
                                      (1.0 - (position[0] * position[0] + position[2] * position[2]) /
                                                 fluid_radius / fluid_radius),
                                  0.0);
        return target_velocity;
    }
};
} // namespace SPH

void poiseuille_flow(const Real resolution_ref, const Real resolution_shell, const Real shell_thickness) {
    const Real inflow_length = resolution_ref * 10.0; // Inflow region
    ...
    const Vec3d emitter_buffer_halfsize(fluid_radius, inflow_length * 0.5, fluid_radius);
    const Vec3d emitter_buffer_translation(0., inflow_length * 0.5 - 2 * resolution_ref, 0.);
	...
    AlignedBoxByCell emitter_buffer(water_block, AlignedBox(yAxis, Transform(Vec3d(emitter_buffer_translation)), emitter_buffer_halfsize));
    SimpleDynamics<fluid_dynamics::InflowVelocityCondition<InflowVelocity>> emitter_buffer_inflow_condition(emitter_buffer);
    ...
}
```

从`AlignedBox`的设定来看，入口速度的盒子长度为$$10\Delta L_\mathrm{f}$$，将其平移后，所有$$-2\Delta L_\mathrm{f}\leq y \leq 8\Delta L_\mathrm{f}$$区间内的粒子速度被设置为给定的入口流速。

# 流入边界

## 定义

流入盒子定义在$$-R\leq x\leq R, 0\leq y \leq 4\Delta L_\mathrm{f}, -R\leq z\leq R$$区间内。它在构造时会从给定的`emitter`（`AlignedBoxByParticle`类型）中读取body信息、粒子信息和获取aligned box，然后将给定的`inlet_particle_buffer`记录为自己的buffer成员，最后确保传入的`inflow_particle_buffer`的`is_particles_reserved_`属性为`true`。

```cpp
void poiseuille_flow(...) {
	const Vec3d emitter_halfsize(fluid_radius, resolution_ref * 2, fluid_radius);
    const Vec3d emitter_translation(0., resolution_ref * 2, 0.);
    ...
    AlignedBoxByParticle emitter(water_block, AlignedBox(yAxis, Transform(Vec3d(emitter_translation)), emitter_halfsize));
    SimpleDynamics<fluid_dynamics::EmitterInflowInjection> emitter_inflow_injection(emitter, inlet_particle_buffer);
    ...
}
```

注意这里使用的是emitter是`AlignedBoxByParticle`，而非入口流速指定那里使用的`AlignedBoxByCell`。这两个的区别在于`AlignedBoxByParticle`对象记录的是被构造时`AlignedBox`区域的粒子ID。所以后续遍历时，无论这些粒子是否跑出了`AlignedBox`，都会把这些粒子遍历到。而`AlignedBoxByCell`记录的是cell（CLL中的cell）的ID。后续遍历时，无论粒子初始时是否属于这个cell，只要检测到某个粒子当前在cell里，就会被遍历到。

这里给`EmitterInflowInjection`传入的必须是`AlignedBoxByParticle`类型的对象，为什么呢？且看下面流入边界是如何执行的。

## 执行

看下面这张图。假设上方是初始粒子分布。注意这里有个容易误解的地方：**图上显示的所有粒子，包括emitter区域的粒子，都是real particle**。不要误认为emitter区域的是buffer particle。如前所述，buffer particle的属性是没有意义的，只是起到预留内存空间的作用。根据案例设定，emitter右边界在$$4\Delta L_\mathrm{f}$$的位置。当时间向前推进，粒子向右移动，程序检测到有一个粒子越出了emitter的右边界，这时，会依次执行：

1. 在这个粒子原位拷贝出一个粒子，使用的是buffer particle的内存。换言之，我们把一个buffer particle转换为了real particle，这一新的particle具有和原粒子一模一样的属性（除了ID）。结果是real particle增加了一个，buffer particle减少了一个。
2. 把旧的particle按周期性边界映射回左边界附近（蛇形箭头），用公式描述就是$$y'=y-4\Delta L_\mathrm{f}$$。映射回去的粒子属性设为默认属性。这样我们就保证了emitter永远有足够的粒子来注入内部的bulk flow。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251226213505.png)

理解了过程，下面来看代码。在内循环结束、更新CLL和邻居表之前执行：

```cpp
void poiseuille_flow(...) {
    ...
	while (physical_time < end_time)
    {
		...
        while (integration_time < Output_Time)
        {
            ... // 内循环
            emitter_inflow_injection.exec();
            ...
        }
        ...
    }
    ...
}
```

`emitter_inflow_injection.exec()`会为每一个粒子调用`EmitterInflowInjection::update`：

```cpp
void EmitterInflowInjection::update(size_t original_index_i, Real dt)
{
    size_t sorted_index_i = sorted_id_[original_index_i];
    if (aligned_box_.checkUpperBound(pos_[sorted_index_i]))
    {
        mutex_switch_to_real_.lock();
        buffer_.checkEnoughBuffer(*particles_);
        particles_->createRealParticleFrom(sorted_index_i);
        mutex_switch_to_real_.unlock();

        /** Periodic bounding. */
        pos_[sorted_index_i] = aligned_box_.getUpperPeriodic(pos_[sorted_index_i]);
        rho_[sorted_index_i] = fluid_.ReferenceDensity();
        p_[sorted_index_i] = fluid_.getPressure(rho_[sorted_index_i]);
    }
}
```

首先需获取sortedID，因为属性是按照sortedID存储的。然后用`checkUpperBound`检查当前粒子是否越出了右边界。如果是，并且buffer尚有足够内存，则原位拷贝一个新的粒子。启用互斥锁的原因是这类边界注入通常可能在并行循环里跑，切换粒子状态、修改粒子数量/容器时必须保证线程安全。

创建完毕后，把旧的粒子映射回左边。其密度设为默认密度，压力按照默认密度计算得出。

# 流出边界

## 定义

流出盒子定义在$$-1.1R\leq x\leq 1.1R, L-4\Delta L_\mathrm{f}\leq y\leq L, -1.1R\leq z\leq 1.1R$$区间内。它在构造时会从给定的`disposer`（`AlignedBoxByParticle`类型）中读取body信息、粒子信息和获取aligned box。

```cpp
void poiseuille_flow(...) {
    const Vec3d disposer_halfsize(fluid_radius * 1.1, resolution_ref * 2, fluid_radius * 1.1);
    const Vec3d disposer_translation(0., full_length - disposer_halfsize[1], 0.);
    ...
    AlignedBoxByCell disposer(water_block, AlignedBox(yAxis, Transform(Vec3d(disposer_translation)), disposer_halfsize));
    SimpleDynamics<fluid_dynamics::DisposerOutflowDeletion> disposer_outflow_deletion(disposer);
    ...
}
```

## 执行

```cpp
void poiseuille_flow(...) {
    ...
	while (physical_time < end_time)
    {
		...
        while (integration_time < Output_Time)
        {
            ... // 内循环
            disposer_outflow_deletion.exec();
            ...
        }
        ...
    }
    ...
}
```

`disposer_outflow_deletion.exec()`会为每一个粒子调用`DisposerOutflowDeletion::update`：

```cpp
void DisposerOutflowDeletion::update(size_t index_i, Real dt)
{
    mutex_switch_to_buffer_.lock();
    while (aligned_box_.checkUpperBound(pos_[index_i]) && index_i < particles_->TotalRealParticles())
    {
        particles_->switchToBufferParticle(index_i);
    }
    mutex_switch_to_buffer_.unlock();
}
```

在互斥锁保护下，检查当前粒子是否超出了disposer边界，以及当前粒子索引是否小于real particle的数目。如果是，则把这个粒子转为buffer particle。

## 深入探究

容易引起困惑的是

1. `AlignedBoxByCell`只遍历box内部的粒子，为什么越界粒子也会被遍历到？

2. 为什么这里使用的是while循环，而不是简单的if语句？

3. 按理说update的粒子都是real particle，为什么还要再检查`index_i < particles_->TotalRealParticles()`，不是多余吗？

第一个问题容易回答：CLL是在流出边界条件执行完毕后方才执行，所以此时CLL是上一时间步的CLL，这个越界的粒子仍然能够被遍历到。

对于后面两个问题，我们需要深入源码看看。我们首先假设一共有5000个real particle。遍历sortedID为#0-4998粒子（后面提到的ID默认都是sortedID，存储顺序也是按sortedID）时都没有检测到有粒子跑出disposer，遍历到#4999粒子时，检测到#4999越界。另外，因为4999<5000，所以while的条件满足，进入循环。进入`particles_->switchToBufferParticle(index_i)`看看：

```cpp
void BaseParticles::switchToBufferParticle(size_t index)
{
    size_t last_real_particle_index = TotalRealParticles() - 1;
    if (index < last_real_particle_index)
    {
        copyFromAnotherParticle(index, last_real_particle_index);
        // update original and sorted_id as well
        std::swap(original_id_[index], original_id_[last_real_particle_index]);
        sorted_id_[original_id_[index]] = index;
    }
    decrementTotalRealParticles();
}
```

传入的实参`index`的值是4999。`TotalRealParticles()`的值是5000，于是`last_real_particle_index`值是4999。在if判断中，`index`和`last_real_particle_index`的值相等，if条件不满足，跳过。最后`decrementTotalRealParticles()`会将`TotalRealParticles()`的值减去1。此时#4999粒子已不在real particles的范围内，后续就不会参与interaction和update了。这也是为什么这个函数名叫做`switchToBufferParticle`。

现在退出`BaseParticles::switchToBufferParticle`。再次来到while的判断，因为`particles_->TotalRealParticles()`已经变成4999了，所以while条件不满足，退出循环。

好像while循环和`index_i < particles_->TotalRealParticles()`判断并没有起作用？这是因为我们选定的越界粒子太特殊了。实际上越界的不一定恰为#4999号粒子。这次我们假设#4000和#4999都越界了，那么在遍历到#4000号时便会使得while条件满足，传入`BaseParticles::switchToBufferParticle`的`index`就是4000。`last_real_particle_index`还是4999。此时4000<4999，if条件满足，进入内部作用域。`copyFromAnotherParticle(index, last_real_particle_index)`会把#4999粒子的属性拷贝给#4000粒子（安全的，因为#4000粒子即将被剔除real particle，它原本的属性已经无需保留）。然后把#4000粒子和#4999粒子交换。结果是越界粒子变成了#4999，原#4999粒子现在变成了#4000。现在就可以安全地执行`decrementTotalRealParticles()`了。

退出`BaseParticles::switchToBufferParticle`。再次来到while的判断。注意此时`index_i`（#4000）粒子已经具备了原#4999粒子的属性，而原#4999粒子是越界的。如果这里不是while循环而是if判断，那么这个越界的粒子就再也不会被处理了，因为再也不会遍历到#4000了。幸好这里是while。于是再次进入`BaseParticles::switchToBufferParticle`，交换当前的#4000（原#4999）和当前的#4998，把原#4999粒子也删掉，`TotalRealParticles()`变成了4998。再次进入while条件判断，原#4998是不越界的，因此while条件不满足。现在我们回答了第二个问题（为什么要while而非if）。

注意，当我们删除粒子时，disposer的loop range并没有更改。因此，尽管real particle的数目已经变成了4998，while循环依旧会遍历到4998和4999。`index_i < particles_->TotalRealParticles()`的判断，避免了这里把buffer particle也删了（非预期行为）。现在我们回答了第二个问题（为什么要额外判断`index_i < particles_->TotalRealParticles()`）。

补充一下，储存real particles的内存空间是连续的，储存buffer particles的内存空间也是连续的，并且buffer particle紧跟在real particle后面。因此每次删除粒子都是删最后一个real particle，而非直接把越界粒子删掉。这说明了交换粒子的必要性。
