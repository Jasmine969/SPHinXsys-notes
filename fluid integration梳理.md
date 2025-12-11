# 概述

这两个文件共同定义并实现弱可压流体（WCSPH）的两阶段时间推进算子：第一半步（动量/压力力）与第二半步（连续性/密度）。
`fluid_integration.h` 声明类与模板类型，`fluid_integration.hpp` 提供具体实现。

# 核心类与关系

- `LocalDynamics`（来自 `base_local_dynamics.h`）

  - 所有局部动力学的基类，持有`SPHBody`与粒子变量访问接口。
  
- `FluidInitialCondition : LocalDynamics`
  
  - 用于初始化流体粒子的 Position、Velocity。
  
- `ContinuumVolumeUpdate : LocalDynamics`
  
  - 根据质量和密度更新体积 Vol = Mass / rho。
  
- `BaseIntegration<DataDelegationType> : LocalDynamics, DataDelegationType`
  
  - 通用基类，绑定关系（`Inner/Contact`），抓取/注册通用状态：
    - `Vol, rho, mass, p, drho_dt, pos, vel, force, force_prior`
    - 引用材料 Fluid。
  
- 模板`Integration1stHalf`。第一半步（动量方程、压力梯度力、人工耗散），更新速度；同时半步更新位置与压力。可以指定是否要修正核函数。
  
  - 模板`Integration1stHalf<Inner<>, RiemannSolverType, KernelCorrectionType>`。使用核修正 `correction_` 与 `RiemannSolver` 计算内邻域压力力。包括
    - 实例`using Integration1stHalfInnerNoRiemann = Integration1stHalf<Inner<>, NoRiemannSolver, NoKernelCorrection>;`
    - 实例`using Integration1stHalfInnerRiemann = Integration1stHalf<Inner<>, AcousticRiemannSolver, NoKernelCorrection>;`
    - 实例`using Integration1stHalfCorrectionInnerRiemann = Integration1stHalf<Inner<>, AcousticRiemannSolver, LinearGradientCorrection>;`
  - 模板`Integration1stHalf<Contact<Wall>, RiemannSolverType, KernelCorrectionType>`。处理墙接触，基于墙面平均加速度与镜像压力 `p_j_in_wall`。
  - `Integration1stHalf<Contact<>, RiemannSolverType, KernelCorrectionType>`。处理与其它流体体的接触，逐接触体保存各自的 `KernelCorrection` 与 `RiemannSolver`。
  - 壁面+流体复合模板`using Integration1stHalfWithWall = ComplexInteraction<Integration1stHalf<Inner<>, Contact<Wall>>, RiemannSolverType, KernelCorrectionType>;`。有内部交互，也有流体与壁面交互。
    - 实例`using Integration1stHalfWithWallNoRiemann = Integration1stHalfWithWall<NoRiemannSolver, NoKernelCorrection>;`。
    - 实例`using Integration1stHalfWithWallRiemann = Integration1stHalfWithWall<AcousticRiemannSolver, NoKernelCorrection>;`。
    - 实例`using Integration1stHalfCorrectionWithWallRiemann = Integration1stHalfWithWall<AcousticRiemannSolver, LinearGradientCorrection>;`。
    - 实例`using Integration1stHalfCorrectionForOpenBoundaryFlowWithWallRiemann = Integration1stHalfWithWall<AcousticRiemannSolver, LinearGradientCorrectionWithBulkScope>;`。
  
  - 壁面+流体+多相实例`MultiPhaseIntegration1stHalfWithWallRiemann = ComplexInteraction<Integration1stHalf<Inner<>, Contact<>, Contact<Wall>>, AcousticRiemannSolver, NoKernelCorrection>`。有内部交互，也有流体与壁面交互，还包括多相之间的交互。
  
- 模板`Integration2ndHalf`。第二半步（连续性方程，速度散度引起的密度变化），更新密度；同时半步更新位置。不修正核函数。

  - 模板`Integration2ndHalf<Inner<>, RiemannSolverType>`。根据速度跳跃`u_jump`累积`drho_dt`与压力耗散力。
    - 实例`using Integration2ndHalfInnerRiemann = Integration2ndHalf<Inner<>, AcousticRiemannSolver>;`
    - 实例`using Integration2ndHalfInnerNoRiemann = Integration2ndHalf<Inner<>, NoRiemannSolver>;`
    - 实例`using Integration2ndHalfInnerDissipativeRiemann = Integration2ndHalf<Inner<>, DissipativeRiemannSolver>;`
  - 模板`Integration2ndHalf<Contact<Wall>, RiemannSolverType>`。使用墙面平均速度与法向，镜像速度参与散度与耗散计算。
  - 模板`Integration2ndHalf<Contact<>, ...>`。与其它流体体的平均速度与速度跳跃计算`drho_dt`与耗散。
  - 壁面+流体复合模板`using Integration2ndHalfWithWall = ComplexInteraction<Integration2ndHalf<Inner<>, Contact<Wall>>, RiemannSolverType>;`。
    - 实例`using Integration2ndHalfWithWallNoRiemann = Integration2ndHalfWithWall<NoRiemannSolver>;`
    - 实例`using Integration2ndHalfWithWallRiemann = Integration2ndHalfWithWall<AcousticRiemannSolver>;`
  - 壁面+流体+多相实例`using MultiPhaseIntegration2ndHalfWithWallRiemann = ComplexInteraction<Integration2ndHalf<Inner<>, Contact<>, Contact<Wall>>, AcousticRiemannSolver>;`。
