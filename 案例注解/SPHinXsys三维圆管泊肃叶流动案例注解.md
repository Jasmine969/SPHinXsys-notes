在`test_3d_poiseuille_flow_shell`案例中，我们将学到

- 三维模型的建立与模拟
- 流入流出边界
- 多分辨率粒子分布（不同body采用不同分辨率，但同一body分辨率相同）
- 自定义光滑长度

这个教程中，我们不区分分辨率（resolution）与粒子间距（particle spacing）的概念，认为两者等价。

# body创建

## 壁面几何与body

这是壁面几何。y轴是圆管轴线方向，单层壁面，每个纵截面是一个有65个粒子的环，轴线上一共有116个环，这样总共有7540个壁面粒子。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251223141953.png)

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

fff

```cpp
void poiseuille_flow(const Real resolution_ref, const Real resolution_shell, const Real shell_thickness) {
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

测试案例中，规定shell具有自己的分辨率，是全局分辨率的一半。这就涉及到**多分辨率**的问题。因为shell的粒子间距是全局的一半，如果shell也采用全局光滑长度进行平滑，势必会过度平滑，造成失真（见下图）。因此，我们有必要相应减小shell的光滑长度，这通过`defineAdaptation<SPH::SPHAdaptation>(...)`来实现。括号中传入的

- 第一个参数是光滑长度与粒子间距之比（$h/\Delta L$），默认值是1.3，设为1.15则有$h=1.15\Delta L$；
- 第二个参数是全局分辨率与本body分辨率之比，默认指是1。这里是按照定义给的：`resolution_ref`是全局分辨率，`resolution_shell`是shell的分辨率。

`defineMaterial`将材料设置为线弹性材料，似是没有必要（见下图），因为这个案例中壁面是固定不动的。我认为改成`Solid`也可以。

在下图中，我模拟了几个case。纵坐标是轴向观测点`Velocity[15][1]`的数据，横坐标是物理时间。original-0、original-1、original-2表示用原始程序跑三次出的结果（每次跑结果都不一样，跑三次可以看出大致趋势）；Solid是将线弹性材料改为`Solid`并不传递任何参数的结果。Solid-NoAdapt在Solid基础上进一步取消了Adaptation。Solid-NoAdapt_Single_resoltion在Solid-NoAdapt的基础上将流体粒子的间距减小为shell粒子的间距，这样就是单一分辨率了。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251223200622.png)

我们可以看到，将线弹性材料改为`Solid`没有任何影响。多分辨率场景中去掉Adaptation使得结果振荡且偏离原值，但改为单一分辨率后又回复至原值且稳定（看稳态结果）。这说明在多分辨率场景中有必要使用Adaptation修改默认设置。



