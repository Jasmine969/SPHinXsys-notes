在`test_2d_oscillating_beam`案例中，我们将学到固体力学问题求解的基本流程。

# 固体定义

```cpp
int main(...)
{
    ...
	//	Creating body, materials and particles.
    //----------------------------------------------------------------------
    SolidBody beam_body(sph_system, makeShared<Beam>("BeamBody"));
    beam_body.defineMaterial<SaintVenantKirchhoffSolid>(rho0_s, Youngs_modulus, poisson);
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
          elastic_solid_(DynamicCast<ElasticSolid>(this, sph_body_->getBaseMaterial())) {};

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

`solid_dynamics::ElasticDynamicsInitialCondition`其实就是个空壳子，和`fluid_dynamics::FluidInitialCondition`除了名字不一样其他都一样。用户需要自定义`update`函数。它继承于`LocalDynamics`，当用户在构造时传入`sph_body`时，就会赋给`LocalDynamics`的`sph_body_`成员，相当于接管了这个body。在速度场的定义中，我们要用到固体的参考声速。但是无法直接从`sph_body_`获取，因为`getBaseMaterial()`返回的是基类`BaseMaterial`的对象，而`BaseMaterial`并未定义`ReferenceSoundSpeed`方法。因此我们需要将返回的`BaseMaterial`对象经`DynamicCast`转换为`ElasticSolid`对象，后者定义了`ReferenceSoundSpeed`方法。

因为`BeamInitialCondition`只定义了`update`函数，所以只需用`SimpleDynamics`接管这个动力学。在求解开始之前执行一次即可。

# 固体动力学

```cpp
int main(...)
{
	...
    InteractionWithUpdate<LinearGradientCorrectionMatrixInner> beam_corrected_configuration(beam_body_inner);

    Dynamics1Level<solid_dynamics::Integration1stHalfPK2> stress_relaxation_first_half(beam_body_inner);
    Dynamics1Level<solid_dynamics::Integration2ndHalf> stress_relaxation_second_half(beam_body_inner);

    //	Setup computing and initial conditions.
    //----------------------------------------------------------------------
    sph_system.initializeSystemCellLinkedLists();
    sph_system.initializeSystemConfigurations();
    beam_initial_velocity.exec();
    beam_corrected_configuration.exec();
    ...
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
            }
        }
        ...
    }
    ...
}
```

