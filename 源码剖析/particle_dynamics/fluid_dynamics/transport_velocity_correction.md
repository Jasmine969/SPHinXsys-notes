# 基类

模板`TransportVelocityCorrection<Base, DataDelegationType, KernelCorrectionType, ParticleScope>`。继承于`LocalDynamics`+` DataDelegationType`。

# Inner特化：对同一fluid body内部邻域做修正

模板`TransportVelocityCorrection<Inner<ResolutionType, LimiterType>, CommonControlTypes...>`。继承基类（固定用`DataDelegateInner`）。

- `using TransportVelocityCorrectionInner=TransportVelocityCorrection<Inner<SingleResolution, LimiterType>, NoKernelCorrection, ParticleScope>;`

# Contact特化：接触边界 wall，只需要 wall 的体积度量

模板`TransportVelocityCorrection<Contact<Boundary>, CommonControlTypes...>`。从每个接触body取`wall_Vol_`（"VolumetricMeasure"）并累加对`kernel_gradient_integral_`的贡献。

# Contact 特化 2：接触“其他相/其他流体体”，每个接触体有自己的 kernel correction

模板`TransportVelocityCorrection<Contact<>, KernelCorrectionType, ...>`。为每个contact body创建`contact_kernel_corrections_`与 `contact_Vol_`，用于多相/多流体间的TVC修正项。

# 复合类型

| 别名                                                         | 实质                                                         | 关系                                           | ResolutionType       | LimiterType       | KernelCorrectionType                    | ParticleScope |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ---------------------------------------------- | -------------------- | ----------------- | --------------------------------------- | ------------- |
| `BaseTransportVelocityCorrectionComplex`                     | `ComplexInteraction<TransportVelocityCorrection<Inner<ResolutionType, LimiterType>, Contact<Boundary>>, CommonControlTypes...>` | `Inner<...>`, `Contact<Boundary>`              | ?                    | ?                 | ?                                       | ?             |
| `TransportVelocityCorrectionComplex`                         | `BaseTransportVelocityCorrectionComplex<SingleResolution, NoLimiter, NoKernelCorrection, ParticleScope>` | `Inner<...>`, `Contact<Boundary>`              | `SingleResolution`   | `NoLimiter`       | `NoKernelCorrection`                    | ?             |
| `TransportVelocityCorrectionCorrectedComplex`                | `BaseTransportVelocityCorrectionComplex<SingleResolution, NoLimiter, LinearGradientCorrection, ParticleScope>` | `Inner<...>`, `Contact<Boundary>`              | `SingleResolution`   | `NoLimiter`       | `LinearGradientCorrection`              | ?             |
| `TransportVelocityCorrectionCorrectedForOpenBoundaryFlowComplex` | `BaseTransportVelocityCorrectionComplex<SingleResolution, NoLimiter, LinearGradientCorrectionWithBulkScope, ParticleScope>` | `Inner<...>`, `Contact<Boundary>`              | `SingleResolution`   | `NoLimiter`       | `LinearGradientCorrectionWithBulkScope` | ?             |
| `TransportVelocityLimitedCorrectionCorrectedForOpenBoundaryFlowComplex` | `BaseTransportVelocityCorrectionComplex<SingleResolution, TruncatedLinear, LinearGradientCorrectionWithBulkScope, ParticleScope>` | `Inner<...>`, `Contact<Boundary>`              | `SingleResolution`   | `TruncatedLinear` | `LinearGradientCorrectionWithBulkScope` | ?             |
| `TransportVelocityLimitedCorrectionComplex`                  | `BaseTransportVelocityCorrectionComplex<SingleResolution, TruncatedLinear, NoKernelCorrection, ParticleScope>` | `Inner<...>`, `Contact<Boundary>`              | `SingleResolution`   | `TruncatedLinear` | `NoKernelCorrection`                    | ?             |
| `TransportVelocityLimitedCorrectionCorrectedComplex`         | `BaseTransportVelocityCorrectionComplex<SingleResolution, TruncatedLinear, LinearGradientCorrection, ParticleScope>` | `Inner<...>`, `Contact<Boundary>`              | `SingleResolution`   | `TruncatedLinear` | `LinearGradientCorrection`              | ?             |
| `TransportVelocityCorrectionComplexAdaptive`                 | `BaseTransportVelocityCorrectionComplex<AdaptiveResolution, NoLimiter, NoKernelCorrection, ParticleScope>` | `Inner<...>`, `Contact<Boundary>`              | `AdaptiveResolution` | `NoLimiter`       | `NoKernelCorrection`                    | ?             |
| `BaseMultiPhaseTransportVelocityCorrectionComplex`           | `ComplexInteraction<TransportVelocityCorrection<Inner<ResolutionType, NoLimiter>, Contact<>, Contact<Boundary>>, CommonControlTypes...>` | `Inner<...>`, `Contact<>`, `Contact<Boundary>` | ?                    | `NoLimiter`       | ?                                       | ?             |
| `MultiPhaseTransportVelocityCorrectionComplex`               | `BaseMultiPhaseTransportVelocityCorrectionComplex<SingleResolution, NoKernelCorrection, ParticleScope>` | `Inner<...>`, `Contact<>`, `Contact<Boundary>` | `SingleResolution`   | `NoLimiter`       | `NoKernelCorrection`                    | ?             |

# 四类“控制维度”：决定变体适用范围

## 粒子范围ParticleScope

决定哪些粒子参与修正（“适用范围”最直接的开关）。定义在`particle_functors.h`。

- `AllParticles`：全体粒子都修正
- `BulkParticles`：`IndicatedParticles<0>`，只修正`Indicator==0`的“体内/主体”粒子（常用于排除`buffer/open boundary`粒子）
- `IndicatedParticles<k> / NotIndicatedParticles<k>`：按`Indicator`筛选。代码里所有 interaction/update 都先判断 within_scope_(index_i)，不在范围内就跳过

## 核梯度修正KernelCorrectionType

决定内/接触项里用什么“核修正矩阵/系数”。定义在`particle_functors.h`。

- `NoKernelCorrection`：恒为 1（不做核修正）
- `LinearGradientCorrection`：返回`B_[i]`（"LinearGradientCorrectionMatrix"）
- `LinearGradientCorrectionWithBulkScope`：关键用于open boundary/buffer 场景：在`operator()(j,i)`中，若`j`不属于bulk，则用`B_[i]`替代`B_[j]`，避免边界/缓冲粒子带来的不稳定核修正传播到主体

## 限制修正幅度LimiterType

更稳，但更“保守”。定义在`common_functors.h`。

- `NoLimiter`：不限制（返回 1）
- `TruncatedLinear`：根据measure（这里是`kernel_gradient_integral`的平方范数）给一个截断线性比例，抑制过大位移修正

## 分辨率ResolutionType

- `SingleResolution`: $h$比例恒为 1
- `AdaptiveResolution`: 比例来自粒子变量
