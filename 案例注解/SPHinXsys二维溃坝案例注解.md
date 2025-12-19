官方教程的案例已经过时，与最新版的SPHinXsys不匹配了。因此我写了最新版的教程系列，欢迎关注。本次讲第一个案例——二维溃坝（test_2d_dambreak）。

# 命令行参数

这些参数定义在`sph_system.cpp`中：

```cpp
        desc.add_options()("help", "produce help message");
        desc.add_options()("relax", po::value<bool>(), "Particle relaxation.");
        desc.add_options()("reload", po::value<bool>(), "Particle reload from input file.");
        desc.add_options()("regression", po::value<bool>(), "Regression test.");
        desc.add_options()("state_recording", po::value<bool>(), "State recording in output folder.");
        desc.add_options()("restart_step", po::value<int>(), "Run form a restart file.");
        desc.add_options()("log_level", po::value<int>(), "Output log level (0-6). "
                                                          "0: trace, 1: debug, 2: info, 3: warning, 4: error, 5: critical, 6: off");
```

## relax

指定是否要粒子松弛，在复杂几何中使用，用于生成贴体的粒子初始分布（不进行模拟）。如果需要，参数为`true/yes/on/1`；否则为`false/no/off/0`，不区分大小写。

默认值：`false`。

## reload

指定是否要从输入文件中重新加载松弛的粒子。如果终端提示`Error in load file: Error=XML_ERROR_FILE_NOT_FOUND ErrorID=3 (0x3)`，先运行`--relax`，再运行`reload`。

默认值：`false`。

## regression

指定是否要生成回归测试的数据集。

默认值：`false`。

## state_recording

指定是否要输出粒子状态（`output`目录中的VTP文件）。

默认值：`true`。

## restart_step

用某一时间步保存的粒子信息续算。用户需要提供从哪一步（整数）开始续算。

默认值：`0`。

## log_level

日志级别，应为0-6的整数。具体含义见`--help`输出和spdlog文档。

默认值：2（info）。

# 全局参数

```cpp
#include "sphinxsys.h" //SPHinXsys Library.
using namespace SPH;   // Namespace cite here.
//----------------------------------------------------------------------
//	Basic geometry parameters and numerical setup.
//----------------------------------------------------------------------
Real DL = 5.366;                    /**< Water tank length. */
Real DH = 5.366;                    /**< Water tank height. */
Real LL = 2.0;                      /**< Water column length. */
Real LH = 1.0;                      /**< Water column height. */
Real particle_spacing_ref = 0.025;  /**< Initial reference particle spacing. */
Real BW = particle_spacing_ref * 3; /**< Thickness of tank wall. */
//----------------------------------------------------------------------
//	Material parameters.
//----------------------------------------------------------------------
Real rho0_f = 1.0;                       /**< Reference density of fluid. */
Real gravity_g = 1.0;                    /**< Gravity. */
Real U_ref = 2.0 * sqrt(gravity_g * LH); /**< Characteristic velocity. */
Real c_f = 10.0 * U_ref;                 /**< Reference sound speed. */
```

# 壁面几何

矩形几何创建时默认以原点为中心。通过Transform命令，把左下角平移到原点位置。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251205152139.png)创建壁面采取了大矩形减去小矩形的方式。

```cpp
class WallBoundary : public ComplexShape
{
  public:
    explicit WallBoundary(const std::string &shape_name) : ComplexShape(shape_name)
    {
        add<GeometricShapeBox>(Transform(outer_wall_translation), outer_wall_halfsize);
        subtract<GeometricShapeBox>(Transform(inner_wall_translation), inner_wall_halfsize);
    }
};
```

注意，这里原点O不在流体域，也不在固体域，而是在流体与固体中间的位置。这个原因到`generateParticles`时会讲。

`add`和`subtract`这两行代码看着挺难理解的。我以`add`为例，讲讲它在底层在做什么。`WallBoundary`继承于`ComplexShape`，后者又继承于`BinaryShape`，`BinaryShape`具有`add`成员函数。`add`成员函数将在其`sub_shape_ptrs_keeper_`（储存`unique_ptr<Shape*>`的vector）添加一个指向`GeometricShapeBox`对象的`Shape`指针，然后在`sub_shapes_and_ops_`（储存`pair<Shape *, ShapeBooleanOps>`的vector）中添加一个`pair`：`(上面创建的Shape指针,add)`。创建指向`GeometricShapeBox`对象的`Shape`指针时，使用的参数为`Transform(outer_wall_translation)`和`outer_wall_halfsize`，其中前者调用了构造函数`explicit BaseTransform::BaseTransform(const VecType &translation)`表示平移变换，后者表示盒子一半尺寸，将传入`GeometricBox`的构造函数。

# main函数

## 定义系统

```cpp
    // 定义二维盒子边界，使用左下角（坐标最小的）和右上角（坐标最大的）
    BoundingBox system_domain_bounds(Vec2d(-BW, -BW), Vec2d(DL + BW, DH + BW));
    // 定义系统，使用定义好的系统边界与粒子间距
    SPHSystem sph_system(system_domain_bounds, particle_spacing_ref);
```

## 定义body

在SPHinXsys中，定义一个body有以下5种方式：

```cpp
SPHBody(SPHSystem &sph_system, Shape &shape, const std::string &name);
SPHBody(SPHSystem &sph_system, Shape &shape);
SPHBody(SPHSystem &sph_system, const std::string &name);
SPHBody(SPHSystem &sph_system, SharedPtr<Shape> shape_ptr, const std::string &name);
SPHBody(SPHSystem &sph_system, SharedPtr<Shape> shape_ptr);
```

定义water_block时使用的是第二个，定义SolidBody对象时使用的是第五个。当没有传入名字时，默认使用shape的名字（见base_body.cpp的20行和32行）。

```cpp
	GeometricShapeBox initial_water_block(Transform(water_block_translation), water_block_halfsize, "WaterBody");
	FluidBody water_block(sph_system, initial_water_block);
	// 定义材料，rho0f和c_f是初始化WeaklyCompressibleFluid的参数
    water_block.defineMaterial<WeaklyCompressibleFluid>(rho0_f, c_f);
	// 生成颗粒，模板参数中第一个是颗粒类型，第二个是创建风格
    water_block.generateParticles<BaseParticles, Lattice>();

    SolidBody wall_boundary(sph_system, makeShared<WallBoundary>("WallBoundary"));
    wall_boundary.defineMaterial<Solid>();
    wall_boundary.generateParticles<BaseParticles, Lattice>();
```

注生成`water_block`时，我们传入的是在栈上定义的`initial_water_block`。但是在生成`wall_boundary`时，我们传入的却是shared pointer。为什么这里使用了不同的构造方式呢？按照[从源码到范式：SPHinXsys 的RAII所有权设计与指针策略](../源码剖析/从源码到范式：SPHinXsys 的RAII所有权设计与指针策略.md)一文所说，这里其实传入shared pointer是最合适的，符合RAII原则。需要注意的是，以下定义body的写法是不允许的：

```cpp
SolidBody wall_boundary(sph_system, WallBoundary("WallBoundary"));
```

虽然看着简单，只需要一行代码就可以创建，但是这行代码会在编译期报错。理由是`WallBoundary("WallBoundary")`是一个临时的对象，更准确来说是个右值，我们不能把一个左值引用（`Shape &shape`）绑定到一个右值上。因此，上面的代码应该改为下面这样，以保证`wall_boundary_`存活至main函数结束：

```cpp
WallBoundary wall_boundary_("WallBoundary");
SolidBody wall_boundary(sph_system, wall_boundary_);
```

我们使用`generateParticles<BaseParticles, Lattice>()`来生成颗粒。它会调用`ParticleGenerator<BaseParticles, Lattice>`的构造函数。`Lattice`是基于网格生成粒子。网格线与边界线是重合的，而粒子被放在网格的中心位置。所以，粒子位置总是会比指定边界多或少半个粒子间距。而这**恰好避免了壁面与流体的粒子重叠**。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251216121448.png)

定义观测点，大概是为了回归测试的。

```cpp
    ObserverBody fluid_observer(sph_system, "FluidObserver");
    StdVec<Vecd> observation_location = {Vecd(DL, 0.2)};
    fluid_observer.generateParticles<ObserverParticles>(observation_location);
```

## 定义关系

类似于LAMMPS中的`pair_coeff I J`，指定哪两类粒子之间要建立邻居表。`ComplexRelation`不是为了建立邻居表，而是为了更新配置。

```cpp
    //----------------------------------------------------------------------
    //	Define body relation map.
    //	The contact map gives the topological connections between the bodies.
    //	Basically the the range of bodies to build neighbor particle lists.
    //  Generally, we first define all the inner relations, then the contact relations.
    //----------------------------------------------------------------------
    InnerRelation water_block_inner(water_block);
    ContactRelation water_wall_contact(water_block, {&wall_boundary});
    ContactRelation fluid_observer_contact(fluid_observer, {&water_block});
    // Combined relations built from basic relations
    // which is only used for update configuration.
    //----------------------------------------------------------------------
    ComplexRelation water_wall_complex(water_block_inner, water_wall_contact);
```

注意，这里的关系不是双向的（对称的），而是单向的。例如当前的`water_wall_contact`表示的是水体要和什么body建立邻居表，主体是水体，这个`water_wall_contact`只能用于水体的积分。我们不能把`water_wall_contact`定义为以下形式：

```cpp
ContactRelation water_wall_contact(wall_boundary, {&water_block});
```

上面这种关系以壁面为主体，它在壁面可变形的场景中是有用的，但是在这里没用。

## 规定数值方法

`SimpleDynamics`是一个不考虑粒子间相互作用的简单粒子动力学算法模板类。`NormalDirectionFromBodyShape` 是局部动力学类，用于从物体形状计算粒子的法向方向。这确保了墙体边界的每个粒子都有正确的法向量，这对于后续的流固耦合计算（如Riemann求解器）至关重要，用于正确处理流体与墙体的相互作用。

```cpp
    //----------------------------------------------------------------------
    // Define the numerical methods used in the simulation.
    // Note that there may be data dependence on the sequence of constructions.
    // Generally, the geometric models or simple objects without data dependencies,
    // such as gravity, should be initiated first.
    // Then the major physical particle dynamics model should be introduced.
    // Finally, the auxiliary models such as time step estimator, initial condition,
    // boundary condition and other constraints should be defined.
    //----------------------------------------------------------------------
    Gravity gravity(Vecd(0.0, -gravity_g));
    SimpleDynamics<GravityForce<Gravity>> constant_gravity(water_block, gravity);
    SimpleDynamics<NormalDirectionFromBodyShape> wall_boundary_normal_direction(wall_boundary);
```

内壁面法向量朝内，外壁面法向量朝外，对于奇数层壁面，中间层的法向量是朝内的，如图所示：

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251215102145.png)

`Dynamics1Level`是最复杂的粒子动力学算法，通常是流体动力学或固体动力学算法的具体实现，包含完整的三步骤初始化`initialization(i, dt)`、粒子间相互作用`interaction(i, dt)`、状态更新`update(i, dt)`。

```cpp
    Dynamics1Level<fluid_dynamics::Integration1stHalfWithWallRiemann> fluid_pressure_relaxation(water_block_inner, water_wall_contact);
    Dynamics1Level<fluid_dynamics::Integration2ndHalfWithWallRiemann> fluid_density_relaxation(water_block_inner, water_wall_contact);
```

关于fluid_integration相关类的详细介绍见[此](./fluid integration梳理.md)。`Integration1stHalfWithWallRiemann`是一个复合类型，既有内部交互，也有流体与壁面交互，使用Riemann solver实现物理场的稳定化，没有核函数修正。与`2ndHalf`系列相比，它执行对动量方程的离散实现。执行如下操作（`src\shared\particle_dynamics\fluid_dynamics\fluid_integration.hpp`）：

```cpp
// 1. 流体内部initialization() - 更新半步密度、压力、位置
rho_[index_i] += drho_dt_[index_i] * dt * 0.5;
p_[index_i] = fluid_.getPressure(rho_[index_i]);
pos_[index_i] += vel_[index_i] * dt * 0.5;

// 2. 流体内部interaction() - 搜索邻居表，计算压力梯度力
force -= (p_[index_i] * correction_(index_j) + p_[index_j] * correction_(index_i)) * dW_ijV_j * e_ij;
rho_dissipation += riemann_solver_.DissipativeUJump(p_[index_i] - p_[index_j]) * dW_ijV_j;

// 3. 流体与壁面interaction() - 搜索邻居表，计算压力梯度力
force -= (p_[index_i] + p_j_in_wall) * correction_(index_i) * dW_ijV_j * e_ij;
rho_dissipation += riemann_solver_.DissipativeUJump(p_[index_i] - p_j_in_wall) * dW_ijV_j;

// 4. update() - 更新一步速度
vel_[index_i] += (force_prior_[index_i] + force_[index_i]) / mass_[index_i] * dt;
```

`Integration2ndHalfWithWallRiemann`是一个复合类型，既有内部交互，也有流体与壁面交互，使用Riemann solver实现物理场的稳定化，没有核函数修正。与`1stHalf`系列相比，它执行对连续性方程的离散实现。执行如下操作（`src\shared\particle_dynamics\fluid_dynamics\fluid_integration.hpp`）：

```cpp
// 1. 流体内部initialization() - 更新半步位置
pos_[index_i] += vel_[index_i] * dt * 0.5;

// 2. 流体内部interaction() - 搜索邻居表，计算速度散度引起的密度变化
Real u_jump = (vel_[index_i] - vel_[index_j]).dot(e_ij);
density_change_rate += u_jump * dW_ijV_j;
p_dissipation += riemann_solver_.DissipativePJump(u_jump) * dW_ijV_j * e_ij;

// 3. 流体与壁面interaction() - 搜索邻居表，计算速度散度引起的密度变化
density_change_rate += (vel_[index_i] - vel_j_in_wall).dot(e_ij) * dW_ijV_j;
Real u_jump = 2.0 * (vel_[index_i] - vel_ave_k[index_j]).dot(n_k[index_j]);
p_dissipation += riemann_solver_.DissipativePJump(u_jump) * dW_ijV_j * n_k[index_j];

// 4. 流体内部update() - 更新一步密度
rho_[index_i] += drho_dt_[index_i] * dt * 0.5;
```

`InteractionWithUpdate`没有initialization，只有interaction和update。

```cpp
    InteractionWithUpdate<fluid_dynamics::DensitySummationComplexFreeSurface> fluid_density_by_summation(water_block_inner, water_wall_contact);
```

`ReduceDynamics`一般用来求全局的归约值（最大、最小、求和等）这两个`ReduceDynamics`用来计算Dual-criteria timestepping中的对流时间步长和声学时间步长

```cpp
    ReduceDynamics<fluid_dynamics::AdvectionTimeStep> fluid_advection_time_step(water_block, U_ref);
    ReduceDynamics<fluid_dynamics::AcousticTimeStep> fluid_acoustic_time_step(water_block);
```

它们分别是（v1.2.1版本尚未更正，以下是master分支的更正结果）
$$
\Delta t_\mathrm{ac}=\mathrm{CFL_{ac}}\frac{ h_\mathrm{min}}{c+|v|_\mathrm{max}}
$$

$$
\Delta t_\mathrm{ad}=\mathrm{CFL_{ad}}\min\left\{\frac{h_\mathrm{min}}{|v|_\mathrm{max}},\sqrt{\frac{h_\mathrm{min}}{4|a|_\mathrm{max}}},\frac{h_\mathrm{min}}{|v|_\mathrm{ref}}\right\}
$$

## 粒子排序

```cpp
    //	Define the configuration related particles dynamics.
    //----------------------------------------------------------------------
    ParticleSorting particle_sorting(water_block);
```

这对于后面建立邻居表是有用的。

## IO方法与回归测试

```cpp
    //	Define the methods for I/O operations, observations
    //	and regression tests of the simulation.
    //----------------------------------------------------------------------
    BodyStatesRecordingToVtp body_states_recording(sph_system);
    body_states_recording.addToWrite<Vecd>(wall_boundary, "NormalDirection");
    RestartIO restart_io(sph_system);
    RegressionTestDynamicTimeWarping<ReducedQuantityRecording<TotalMechanicalEnergy>> write_water_mechanical_energy(water_block, gravity);
    RegressionTestDynamicTimeWarping<ObservedQuantityRecording<Real>> write_recorded_water_pressure("Pressure", fluid_observer_contact);
```

我们在`BodyStatesRecordingToVtp`构造函数中传入了整个体系，这样它会为每一个`SPHBody`输出一个VTP文件。

`body_states_recording.addToWrite<Vecd>(wall_boundary, "NormalDirection");`这一行的作用是在`wall_boundary`的VTP文件中添加一个`NormalDirection`的变量。壁面颗粒默认只有`OriginalID`和`SortedID`两个变量输出；而流体颗粒除了这两个ID，还有`Velocity`输出（见fluid_integration.h第46行）。以下为输出其他变量的示例（整数使用`int`类型，浮点数使用`Real`类型，矢量使用`Vecd`类型，二阶张量使用`Matd`类型）：

```cpp
    body_states_recording.addToWrite<Real>(water_block, "Pressure");
    body_states_recording.addToWrite<Real>(water_block, "Density");
    body_states_recording.addToWrite<Vecd>(water_block, "Force");
    body_states_recording.addToWrite<Vecd>(water_block, "ForcePrior");
    body_states_recording.addToWrite<Real>(water_block, "Mass");
```

## 初始化

初始化邻居表。SPHinXsys中的邻居表是用Cell Linked List（CLL）构建的。因此，我们要先初始化CLL（`initializeSystemCellLinkedLists`），然后才能初始化邻居表（`initializeSystemConfigurations`）

```cpp
    sph_system.initializeSystemCellLinkedLists();
    sph_system.initializeSystemConfigurations();
```

其他初始化，包括壁面法向和重力，它们不需要在模拟时更新，只需要初始化即可。

```cpp
    wall_boundary_normal_direction.exec();
    constant_gravity.exec();
```

## 加载续算文件

如果需要续算，需要准备以下文件：

- 每个`SPHBody`对象一个XML文件，命名为`<BodyName>_rst_<step>.xml`；
- `Restart_time_<step>.data`，储存该时间步的物理时间。

这些文件会在上一次模拟时自动保存。

```cpp
    //	Load restart file if necessary.
    //----------------------------------------------------------------------
    Real &physical_time = *sph_system.getSystemVariableDataByName<Real>("PhysicalTime");
    if (sph_system.RestartStep() != 0)
    {
        physical_time = restart_io.readRestartFiles(sph_system.RestartStep());
        water_block.updateCellLinkedList();
        water_wall_complex.updateConfiguration();
        fluid_observer_contact.updateConfiguration();
    }
```

首先获取了系统物理时间的引用。接着`restart_io.readRestartFiles(sph_system.RestartStep())`修改了系统所有`SPHBody`的状态（在内部依次读取每个`SPHBody`对象的XML文件），然后返回了续算时间步的物理时间给`physical_time`。因为后者是绑定到系统的物理时间的，所以这一操作也就修改了系统的物理时间。下面两行与初始化类似，先更新了CLL，然后更新了邻居表。注意，更新CLL只针对了`water_block`，而没有针对`wall_boundary`。这是因为壁面是固定不动的，如果壁面是运动的，那么也要更新其CLL。

## 时间推进控制

```cpp
    //----------------------------------------------------------------------
    //	Setup for time-stepping control
    //----------------------------------------------------------------------
    size_t number_of_iterations = sph_system.RestartStep();
    int screen_output_interval = 100;
    int observation_sample_interval = screen_output_interval * 2;
    int restart_output_interval = screen_output_interval * 10;
    Real end_time = 20;
    Real output_interval = 0.1;
```

`number_of_iterations`表示当前已经运行的步数。`screen_output_interval`表示每隔多久在屏幕输出一次。`observation_sample_interval`控制观测数据的采样间隔。`restart_output_interval`控制多久保存一次restart文件。`end_time`表示算到什么时候结束（物理时间）。`output_interval`表示每隔多久（物理时间）输出一次粒子状态。

## 计时器

```cpp
    //----------------------------------------------------------------------
    //	Statistics for CPU time
    //----------------------------------------------------------------------
    TickCount t1 = TickCount::now();
    TimeInterval interval;
    TimeInterval interval_computing_time_step;
    TimeInterval interval_computing_fluid_pressure_relaxation;
    TimeInterval interval_updating_configuration;
    TickCount time_instance;
```

`TickCount`和`TimeInterval`是tbb中定义的类型，前者表示时刻，后者表示时间间隔。它们的具体含义见后续代码。

## 初始输出

```cpp
    //----------------------------------------------------------------------
    //	First output before the main loop.
    //----------------------------------------------------------------------
    body_states_recording.writeToFile();
    write_water_mechanical_energy.writeToFile(number_of_iterations);
    write_recorded_water_pressure.writeToFile(number_of_iterations);
```

在主循环开始前，保存零时刻的状态。`body_states_recording.writeToFile`函数不需要任何参数；`write_water_mechanical_energy.writeToFile`需要传入当前时间步（默认零时间步）。

## 主循环

循环分为三层。外层循环把整个模拟分成了多个output周期，周期长度就是`output_interval`。中层和内层循环由Dual-criteria time stepping控制，具体来说， 中层循环的时间步长为对流时间步长（$\Delta t_\mathrm{ad}$），内层循环的时间步长为声学时间步长（$\Delta t_\mathrm{ac}$）。内层循环的周期长度是对流时间步长。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251218221548.png)

### 外循环

```cpp
    while (physical_time < end_time)
    {
        Real integration_time = 0.0;
        /** Integrate time (loop) until the next output time. */
        while (integration_time < output_interval)
        {
            // middle loop
            /** outer loop for dual-time criteria time-stepping. */
            ...
        }
        body_states_recording.writeToFile();
        TickCount t2 = TickCount::now();
        TickCount t3 = TickCount::now();
        interval += t3 - t2;
    }
```

`integration_time`记录了每一轮output已经进行了多久，当时间到达`output_interval`时，跳出中层循环，保存粒子状态，然后进入下一轮output。

感觉这里`TickCount t2 = TickCount::now();`应该是调到上面一行，以忽略IO的时间消耗，作者可能是笔误：

```cpp
        TickCount t2 = TickCount::now();
		body_states_recording.writeToFile();
        TickCount t3 = TickCount::now();
```

### 中循环

```cpp
        while (integration_time < output_interval)
        {
            /** outer loop for dual-time criteria time-stepping. */
            time_instance = TickCount::now();
            Real advection_dt = fluid_advection_time_step.exec();
            fluid_density_by_summation.exec();
            interval_computing_time_step += TickCount::now() - time_instance;
```

首先计算了对流时间步长。然后执行了密度求和。这一过程的用时记录在`interval_computing_time_step`。

```cpp
            time_instance = TickCount::now();
            Real relaxation_time = 0.0;
            Real acoustic_dt = 0.0;
            while (relaxation_time < advection_dt)
            {
                /** inner loop for dual-time criteria time-stepping.  */
                ...
            }
            interval_computing_fluid_pressure_relaxation += TickCount::now() - time_instance;
```

然后进行了按声学时间步长推进的内循环。`relaxation_time`记录了每一轮中层循环进行了多久。

```cpp
            /** screen output, write body observables and restart files  */
            if (number_of_iterations % screen_output_interval == 0)
            {
                std::cout << std::fixed << std::setprecision(9) << "N=" << number_of_iterations << "	Time = "
                          << physical_time
                          << "	advection_dt = " << advection_dt << "	acoustic_dt = " << acoustic_dt << "\n";

                if (number_of_iterations % observation_sample_interval == 0 && number_of_iterations != sph_system.RestartStep())
                {
                    write_water_mechanical_energy.writeToFile(number_of_iterations);
                    write_recorded_water_pressure.writeToFile(number_of_iterations);
                }
                if (number_of_iterations % restart_output_interval == 0)
                    restart_io.writeToFile(number_of_iterations);
            }
            number_of_iterations++;


```

接着在每隔一段时间屏幕上输出相关信息、将观测数据写入文件、保存续算文件。

```cpp
            /** Update cell linked list and configuration. */
            time_instance = TickCount::now();
            if (number_of_iterations % 100 == 0 && number_of_iterations != 1)
            {
                particle_sorting.exec();
            }
            water_block.updateCellLinkedList();
            water_wall_complex.updateConfiguration();
            fluid_observer_contact.updateConfiguration();
            interval_updating_configuration += TickCount::now() - time_instance;
        }
```

最后，每隔100步执行一次例子排序。更新流体的CLL，更新邻居表。这一过程的用时记录在`interval_updating_configuration`。

### 内循环

```cpp
            while (relaxation_time < advection_dt)
            {
                /** inner loop for dual-time criteria time-stepping.  */
                acoustic_dt = fluid_acoustic_time_step.exec();
                fluid_pressure_relaxation.exec(acoustic_dt);
                fluid_density_relaxation.exec(acoustic_dt);
                relaxation_time += acoustic_dt;
                integration_time += acoustic_dt;
                physical_time += acoustic_dt;
            }
```

内循环先计算了声学时间步长。然后分别进行了动量方程和连续性方程的更新。最后更新了中层循环的时间、外循环的时间和物理时间.

## 打印各部分用时

```cpp
    TickCount t4 = TickCount::now();

    TimeInterval tt;
    tt = t4 - t1 - interval;
    std::cout << "Total wall time for computation: " << tt.seconds()
              << " seconds." << std::endl;
    std::cout << std::fixed << std::setprecision(9) << "interval_computing_time_step ="
              << interval_computing_time_step.seconds() << "\n";
    std::cout << std::fixed << std::setprecision(9) << "interval_computing_fluid_pressure_relaxation = "
              << interval_computing_fluid_pressure_relaxation.seconds() << "\n";
    std::cout << std::fixed << std::setprecision(9) << "interval_updating_configuration = "
              << interval_updating_configuration.seconds() << "\n";
```

主循环结束后，打印总用时与各部分用时。

## 回归测试

```cpp
    if (sph_system.GenerateRegressionData())
    {
        write_water_mechanical_energy.generateDataBase(1.0e-3);
        write_recorded_water_pressure.generateDataBase(1.0e-3);
    }
    else if (sph_system.RestartStep() == 0)
    {
        write_water_mechanical_energy.testResult();
        write_recorded_water_pressure.testResult();
    }
    return 0;
};
```

如果用户指定了`--regression yes`，则生成回归测试数据集；如果没有，并且是从零时刻开始算的，则进行回归测试。

