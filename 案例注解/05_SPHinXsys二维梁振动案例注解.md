在`test_2d_oscillating_beam`案例中，我们将学到固体力学问题求解的基本流程，包括：

- 固体材料模型的定义
- 固体初始条件的定义
- 固体动力学的求解
- 固体运动约束

以下是模型的几何。红色是固定的基座，蓝色是可变形的梁。给定梁的初始速度分布，求解梁在之后的运动。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20260130192656.png)

下图是模型在某时刻的PK1应力分布：

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20260130192900.png)

# 固体材料定义

```cpp
int main(...)
{
    ...
	//	Creating body, materials and particles.
    //----------------------------------------------------------------------
    SolidBody beam_body(sph_system, makeShared<Beam>("BeamBody"));
    beam_body.defineMatterMaterial<SaintVenantKirchhoffSolid>(rho0_s, Youngs_modulus, poisson);
    beam_body.generateParticles<BaseParticles, Lattice>();
    ...
}
```

`SaintVenantKirchhoffSolid`继承于`LinearElasticSolid`，二者区别是`LinearElasticSolid`只适用于小应变，而`SaintVenantKirchhoffSolid`适用于大应变。在[elastic_solid源码剖析](..\源码剖析\physical_closure\materials\elastic_solid.md)中有详细解释。

构造函数输入参考密度、杨氏模量、泊松比$$(\rho_0,E,\nu)$$，代码计算体积模量$$K_0$$、剪切模量$$G_0$$和第一Lame系数$$\lambda_0$$：

$$
K_0=\frac{E}{3(1-2\nu)}
$$

$$
G_0=\frac{E}{2(1+\nu)}
$$

$$
\lambda_0=\frac{\nu E}{(1+\nu)(1-2\nu)}
$$

由上述属性，在每个时间步计算PK2应力：
$$
\mathbf S=\lambda_0\,\mathrm{tr}(\mathbf E)\mathbf I+2G_0\mathbf E
$$

# 初始条件

```cpp
//----------------------------------------------------------------------
//	application dependent initial condition
//----------------------------------------------------------------------
class BeamInitialCondition
    : public solid_dynamics::ElasticDynamicsInitialCondition
{
  public:
    explicit BeamInitialCondition(SPHBody &sph_body)
        : solid_dynamics::ElasticDynamicsInitialCondition(sph_body),
          elastic_solid_(DynamicCast<ElasticSolid>(this, sph_body_->getMatterMaterial())) {};

    void update(size_t index_i, Real dt)
    {
        /** initial velocity profile */
        Real x = pos_[index_i][0] / PL;
        if (x > 0.0)
        {
            vel_[index_i][1] = vf * elastic_solid_.ReferenceSoundSpeed() *
                               (M * (cos(kl * x) - cosh(kl * x)) - N * (sin(kl * x) - sinh(kl * x))) / Q;
        }
    };

  protected:
    ElasticSolid &elastic_solid_;
};

int main(...)
{
    ...
    SimpleDynamics<BeamInitialCondition> beam_initial_velocity(beam_body);
    beam_initial_velocity.exec();
    // 开始求解
    ...
}
```

`solid_dynamics::ElasticDynamicsInitialCondition`其实就是个空壳子，和`fluid_dynamics::FluidInitialCondition`除了名字不一样其他都一样。用户需要自定义`update`函数。它继承于`LocalDynamics`，当用户在构造时传入`sph_body`时，就会赋给`LocalDynamics`的`sph_body_`成员。在速度场的定义中需要用到固体参考声速。`getMatterMaterial()`返回主体材料的基类引用，而该基类没有定义`ReferenceSoundSpeed`，因此要通过`DynamicCast`取得实际的`ElasticSolid`引用，再调用其接口。

因为`BeamInitialCondition`只定义了`update`函数，所以只需用`SimpleDynamics`接管这个动力学。在求解开始之前执行一次即可。

# 固体动力学

```cpp
int main(...)
{
	...
    InteractionWithUpdate<LinearGradientCorrectionMatrixInner> beam_corrected_configuration(beam_body_inner);

    Dynamics1Level<solid_dynamics::Integration1stHalfPK2> stress_relaxation_first_half(beam_body_inner);
    Dynamics1Level<solid_dynamics::Integration2ndHalf> stress_relaxation_second_half(beam_body_inner);
    ReduceDynamics<solid_dynamics::AcousticTimeStep> computing_time_step_size(beam_body);

    //	Setup computing and initial conditions.
    //----------------------------------------------------------------------
    sph_system.initializeSystemCellLinkedLists();
    sph_system.initializeSystemConfigurations();
    beam_initial_velocity.exec();
    beam_corrected_configuration.exec();
    ...
    Real dt = 0.0;
    // 开始求解
    // computation loop starts
    while (physical_time < end_time)
    {
        Real integration_time = 0.0;
        // integrate time (loop) until the next output time
        while (integration_time < output_interval)
        {

            Real relaxation_time = 0.0;
            while (relaxation_time < Dt)
            {
                stress_relaxation_first_half.exec(dt);
                constraint_beam_base.exec();
                stress_relaxation_second_half.exec(dt);
				...
                dt = computing_time_step_size.exec();
                ...
            }
        }
        ...
    }
    ...
}
```

## 修正矩阵

`beam_corrected_configuration`是修正矩阵，用于确保一阶一致性，详见[全拉格朗日SPH入门](../算法/全拉格朗日SPH入门.md)。因为它只依赖于参考构形，所以只需要在模拟开始前计算一次。

两个最关键的动力学是`Integration1stHalf...`和`Integration2ndHalf`这两个积分动力学。在[elastic_dynamics](../源码剖析/particle_dynamics/solid_dynamics/elastic_dynamics.md)中有详细解释。

## Integration1stHalfPK2

`Integration1stHalfPK2`分为三个阶段（对应三个成员函数）：`initialization`、`interaction`和`update`。

- `initialization`阶段，将位置、变形梯度、密度更新半步：
$$
	\mathbf x_i^{n+1/2} = \mathbf x_i^n + \tfrac12\Delta t\,\mathbf 	v_i^n,
$$

$$
\mathbf F_i^{n+1/2} = \mathbf F_i^n + \tfrac12\Delta t\,\dot{\mathbf F}_i^n,
$$

$$
	J_i^{n+1/2} = \det(\mathbf F_i^{n+1/2}),\quad
	\rho_i^{n+1/2} = \frac{\rho_0}{J_i^{n+1/2}}.
$$

然后构造第一PK应力并乘以修正矩阵，结果储存在`stress_PK1_B_`中：
$$
	\mathrm{stress\_PK1\_B}_i \leftarrow  \mathbf P_i^{n+1/2}\mathbf B_i^T,
$$

其中第一PK应力是在材料模型中用第二PK应力算出来的，而第二PK应力又是用最新的变形梯度$$\mathbf F_i^{n+1/2}$$算出来的。

- `interaction`阶段只做一件事——计算内力：

  **内力**：
  $$
  \mathbf f_i=\sum_{j\in\mathcal N(i)} \frac{m_i}{\rho_0}
  \left(\nabla W_{ij} V_j\right)
  \left[\mathbf P_i\mathbf B_i^T + \mathbf P_j\mathbf B_j^T + \alpha w_{ij}\mathbf P^\mathrm{damp}_{ij}\right]\mathbf e_{ij},
  $$

  其中方括号中的第三项是数值阻尼项，$$\alpha$$对应 `numerical_dissipation_factor_`，$$w_{ij}=W_{ij}/W(r=0)$$，$$\mathbf P^\mathrm{damp}_{ij}$$是
  $$
  \mathbf P^\mathrm{damp}_{ij} = \tfrac12(\mathbf F_i+\mathbf F_j)\,\mathcal D(\dot\varepsilon_{ij},h),
  $$

  $$\mathcal D(\cdot)$$ 由材料 `ElasticSolid::PairNumericalDamping()` 给出，$$h$$是光滑长度，$$\dot\varepsilon_{ij}$$是应变率，在`interaction`中先计算好：
  $$
  \dot\varepsilon_{ij}=\left(\frac{d}{r_{ij}}\right)^2 (\mathbf x_i-\mathbf x_j)\cdot(\mathbf v_i-\mathbf v_j),
  $$
  其中$$d$$是维数。

- `update`阶段使用计算的合力更新一步速度：
  $$
  \mathbf v_i^{n+1} = \mathbf v_i^{n} + \Delta t\,\frac{\mathbf f_i + \mathbf f_i^\mathrm{prior}}{m_i}.
  $$
  
## Integration2ndHalf

`Integration2ndHalf`也分为三个阶段（对应三个成员函数）：`initialization`、`interaction`和`update`。

- `initialization`阶段将位置推进至$n+1$步：
  $$
  \mathbf x_i^{n+1} = \mathbf x_i^{n+1/2} + \tfrac12\Delta t\,\mathbf v_i^{n+1}.
  $$

- `interaction`阶段更新变形梯度的变化率：
  $$
  \dot{\mathbf F}_i^{n+1}=\left[-\sum_{j\in\mathcal N(i)} (\mathbf v_i^{n+1}-\mathbf v_j^{n+1})\otimes(\nabla W_{ij}V_j)^T\right]\,\mathbf B_i.
  $$

- `update`阶段把变形梯度推进到$n+1$时间步：
  $$
  \mathbf F_i^{n+1} = \mathbf F_i^{n+1/2} + \tfrac12\Delta t\,\dot{\mathbf F}_i^{n+1}.
  $$

可以看到，SPHinXsys先计算变形梯度变化率，再用其推进变形梯度的更新。在[全拉格朗日SPH入门](../算法/全拉格朗日SPH入门.md)中，我们提到变形梯度可以按下面方式计算：
$$
\langle\mathbf{F}\rangle_i=\langle\nabla_0\mathbf x\rangle_i=\left[\sum_{j\in\mathcal N(i)} V_{0j}(\mathbf{x}_j - \mathbf{x}_i)\otimes\nabla_0 W_{ij}\right]\mathbf B_{0i}
$$
那么为什么不按照这个式子（位置重构式）直接更新变形梯度，而是大费周章地先求变化率再更新变形梯度呢？

实际上位置重构式在SPHinXsys中也有实现，就是`solid_dynamics::DeformationGradientBySummation`。它对粒子无序、邻域变化更敏感；每步重算会让 变形梯度出现“跳动”，再通过本构关系放大到应力和力里，容易引入高频噪声或能量漂移。相比之下，用变形梯度变化率去推进通常更“平滑”，因为它在时间上是连续推进的；并且跟Verlet算法用变化率去推进这套模式也是匹配的。

位置重构式更适合做类似density summation的修正，见`test_2d_stretching`案例。

## 积分时间步长

固体动力学积分的时间步长使用`solid_dynamics::AcousticTimeStep`，它按照下式计算时间步长：
$$
\Delta t_\mathrm{ac} = \mathrm{CFL}\cdot \min_i\left(
\sqrt{\frac{h_{\min}}{\|\mathbf a_i\| + \epsilon}},
\frac{h_{\min}}{c_0 + \|\mathbf v_i\|}
\right).
$$
我个人认为**代码在书写顺序上有点问题**。按理说，应该先求时间步，再执行积分。可是这里顺序反过来了。同时，`dt`被初始化为零，这就导致第一次进行时间积分的时候，时间步长是零，也就是说第一次积分相当于啥也没做。应该把顺序调过来才对。

# 固体运动约束

```cpp
    BodyRegionByParticle beam_base(beam_body, makeShared<MultiPolygonShape>(createBeamConstrainShape()));
    SimpleDynamics<FixBodyPartConstraint> constraint_beam_base(beam_base);
```

我在[指定物体运动或变形](../用户指南/指定物体运动或变形.md)中简要提过固体运动约束。具体来说，我们先定义一个基座的body part；然后用`FixBodyPartConstraint`定义运动约束，它的`update`函数会将位置置为初始位置，速度置零：

```cpp
    void update(size_t index_i, Real dt = 0.0)
    {
        this->pos_[index_i] = this->pos0_[index_i];
        this->vel_[index_i] = Vecd::Zero();
    };
```

因为`FixBodyPartConstraint`只有一个`update`函数，所有我们用`SimpleDynamics`接管它即可。

