在`test_3d_poiseuille_flow_shell`案例中，我们将学到

- 三维模型的建立与模拟
- 流入流出边界
- 多分辨率粒子分布（不同body采用不同分辨率，但同一body分辨率相同）
- 自定义光滑长度

这个教程中，我们不区分分辨率（resolution）与粒子间距（particle spacing）的概念，认为两者等价。

# body创建

这是壁面与流体的几何的左端局部视图。壁面的粒子间距（$\Delta L_\mathrm{s}$）是流体的（$\Delta L_\mathrm{f}$）一半。壁面比流体的边界多延伸出$4\Delta L_\mathrm{f}$的距离（程序中称为`wall_thickness`，记为$\delta_\mathrm{s}$），用来放置入口和出口的buffer。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251224100332.png)

流体网格的左边界为$y=0$的位置。流体粒子和壁面粒子都被放置在网格的中心。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251224100023.png)

于是我们可以计算出每个壁面粒子（序号为$j$，从零开始）的y坐标为：
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

上面的`prepareGeometricData()`规定了每个壁面粒子的位置、体积、法方向和壁面厚度。x和z坐标由圆的参数方程给出，注意半径比预期的多了$\Delta L_\mathrm{s}/2$，以避免和流体粒子重叠。y坐标的公式已经在上面给出，只不过程序里将最后一项做了等量代换（其实没必要，还更麻烦了）。对于壳体，粒子的体积是$(\Delta L_\mathrm{s})^2$，法向量是外法向（流体指向壁面）。

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

- 第一个参数是光滑长度与粒子间距之比（$h/\Delta L$），默认值是1.3，设为1.15则有$h=1.15\Delta L$；
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

使用`TriangleMeshShapeCylinder`创建了流体的几何。五个参数依次为：轴线方向矢量、流体半径、半长度、网格精度和圆柱中心点坐标。关于SimTK的网格精度，在[几何创建](../几何创建.md)中已有介绍，在此不赘。这样一来，流体的几何位于$0\leq y \leq 10\Delta L_\mathrm{f}$的区间内。

# 入口流速

入口流速设定为
$$
\boldsymbol{v}=(0,2U_f(1-r^2/R^2),0)
$$
其中$r(x,z)=x^2+z^2$。

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

从`AlignedBox`的设定来看，入口速度的盒子长度为$10\Delta L_\mathrm{f}$，将其平移后，所有$-2\Delta L_\mathrm{f}\leq y \leq 8\Delta L_\mathrm{f}$区间内的粒子速度被设置为给定的入口流速。

# 流入流出边界

## 定义

流入盒子定义在$-R\leq x\leq R, -4\Delta L_\mathrm{f}\leq y \leq 0, -R\leq z\leq R$区间内，也即

```cpp
void poiseuille_flow(...) {
	const Vec3d emitter_halfsize(fluid_radius, resolution_ref * 2, fluid_radius);
    const Vec3d emitter_translation(0., resolution_ref * 2, 0.);
	...
    const Vec3d disposer_halfsize(fluid_radius * 1.1, resolution_ref * 2, fluid_radius * 1.1);
    const Vec3d disposer_translation(0., full_length - disposer_halfsize[1], 0.);
    ...
    AlignedBoxByParticle emitter(water_block, AlignedBox(yAxis, Transform(Vec3d(emitter_translation)), emitter_halfsize));
    SimpleDynamics<fluid_dynamics::EmitterInflowInjection> emitter_inflow_injection(emitter, inlet_particle_buffer);
    ...
    AlignedBoxByCell disposer(water_block, AlignedBox(yAxis, Transform(Vec3d(disposer_translation)), disposer_halfsize));
    SimpleDynamics<fluid_dynamics::DisposerOutflowDeletion> disposer_outflow_deletion(disposer);
    ...
}
```

## 执行

在内循环结束、更新CLL和邻居表之前执行：

```cpp
void poiseuille_flow(...) {
    ...
	while (physical_time < end_time)
    {
		...
        while (integration_time < Output_Time)
        {
            ...
            /** Water block configuration and periodic condition. */
            emitter_inflow_injection.exec();
            disposer_outflow_deletion.exec();
            ...
        }
        ...
    }
    ...
}
```

