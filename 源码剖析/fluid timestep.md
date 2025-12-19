版本是GitHub master分支。

# AcousticTimeStep

在fluid_time_step.h中可以看到，这个类采用的归约操作是全局最大化：`LocalDynamicsReduce<ReduceMax>`。

查看fluid_time_step.cpp

```cpp
Real AcousticTimeStep::reduce(size_t index_i, Real dt)
{
    return fluid_.getSoundSpeed(p_[index_i], rho_[index_i]) + vel_[index_i].norm();
}
```

可以看到被归约的变量是$c+|v|_i$，也即$c+|v|_i$。

```cpp
Real AcousticTimeStep::outputResult(Real reduced_value)
{
    // since the particle does not change its configuration in pressure relaxation step
    // I chose a time-step size according to Eulerian method
    return acousticCFL_ * h_min_ / (reduced_value + TinyReal);
}
```

最终的结果是
$$
\Delta t_\mathrm{ac}=\mathrm{CFL_{ac}}\frac{ h_\mathrm{min}}{c+|v|_\mathrm{max}}
$$

# SurfaceTensionTimeStep

继承于`AcousticTimeStep`，用于需要考虑表面张力的case。最终结果是
$$
\Delta t_\mathrm{sf}=\mathrm{CFL_{ac}}\min\left\{\frac{h_\mathrm{min}}{c+|v|_\mathrm{max}},\sqrt{\frac{2\pi\sigma h_\mathrm{min}}{\rho_i}}\right\}
$$

# AdvectionTimeStep

最终结果是
$$
\Delta t_\mathrm{ad}=\mathrm{CFL_{ad}}\min\left\{\frac{h_\mathrm{min}}{|v|_\mathrm{max}},\sqrt{\frac{h_\mathrm{min}}{4|a|_\mathrm{max}}},\frac{h_\mathrm{min}}{|v|_\mathrm{ref}}\right\}
$$
这里$|v|_\mathrm{ref}$是特征速度，由用户给定，溃坝案例中是$\sqrt{2gH_\mathrm{max}}$。

# AdvectionViscousTimeStep

继承于`AdvectionTimeStep`，用于需要考虑物理粘度的case。最终结果是
$$
\Delta t_\mathrm{ad}=\mathrm{CFL_{ad}}\min\left\{\frac{h_\mathrm{min}}{|v|_\mathrm{max}},\sqrt{\frac{h_\mathrm{min}}{4|a|_\mathrm{max}}},\frac{h_\mathrm{min}}{|v|_\mathrm{ref}},\frac{h_\mathrm{min}^2}{\nu}\right\}
$$
这里$\nu$是运动粘度。

