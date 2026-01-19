
# elastic_solid材料模型梳理

对应源码：`src/shared/physical_closure/materials/elastic_solid.h`与`src/shared/physical_closure/materials/elastic_solid.cpp`。

## 1.类关系总览

![](..\..\..\static\DerivedClasses-material.png)

另外：`complex_solid.h/.hpp`里有`CompositeSolid`、`ActiveMuscle`等会“组合/叠加”这些`ElasticSolid`派生类的`StressPK2`。

## 2.符号与张量（与实现一致）

- $$F$$：`deformation`，变形梯度。
- $$J=\det(F)$$：体积比。
- $$C=F^TF$$：RightCauchy-Green张量。
- $$b=FF^T$$：LeftCauchy-Green张量。
- $$E=\frac12(C-I)$$：Green-Lagrange应变。
- $$e=\frac12\left(I-(FF^T)^{-1}\right)$$：EulerianAlmansi应变（代码里通过$$F$$构造）。
- 应力度量：
  - PK1：$$P$$，一阶Piola-Kirchhoff应力。
  - PK2：$$S$$，二阶Piola-Kirchhoff应力。
  - Cauchy：$$\sigma$$。
  - Kirchhoff：$$\tau=J\,\sigma$$。

在固体动力学里常用的换算（见`elastic_dynamics.cpp`）：

- 由PK2到PK1：$$P=F\,S$$。
- 由Cauchy到PK1：$$P=J\,\sigma\,F^{-T}$$。
- Kirchhoff分解形式：$$P=(\tau)F^{-T}$$，其中$$\tau=\tau_{vol}I+\tau_{dev}$$。

## 3.ElasticSolid（抽象基类）

### 3.1材料参数与波速

成员（均为reference/初始常量意义）：

- $$\rho_0$$：密度（来自`Solid`基类中的`rho0_`）。
- $$E_0,G_0,K_0,\nu$$：杨氏模量、剪切模量、体积模量、泊松比（对应`E0_`、`G0_`、`K0_`、`nu_`）。
- $$c_0,c_{t0},c_{s0}$$：声速/拉伸波速/剪切波速（对应`c0_`、`ct0_`、`cs0_`）。

代码中`setSoundSpeeds()`：

$$
c_0=\sqrt{\frac{K_0}{\rho_0}},\quad c_{t0}=\sqrt{\frac{E_0}{\rho_0}},\quad c_{s0}=\sqrt{\frac{G_0}{\rho_0}}
$$

这些波速主要用于：时间步控制、接触刚度`setContactStiffness(c0_)`、以及数值耗散。

### 3.2统一接口（子类必须给出具体本构）

- `StressPK1(F,i)`、`StressPK2(F,i)`、`StressCauchy(e,i)`
- `VolumetricKirchhoff(J)`
- `getRelevantStressMeasureName()`：后处理选择的应力度量名称（如`PK2`或`Cauchy`）

### 3.3Kirchhoff应力的“体积+偏”分解

- `DeviatoricKirchhoff(deviatoric_be)`：默认实现非常简单

$$
	au_{dev}=G_0\,\text{deviatoric\_be}
$$

其中`deviatoric_be`来自动力学模块对归一化$$b$$的去迹部分：

$$
\bar b = J^{-2/d} b,\quad \text{deviatoric\_be}=\bar b-\frac{\mathrm{tr}(\bar b)}{d}I
$$

- `VolumetricKirchhoff(J)`：由子类定义$$\tau_{vol}$$。

### 3.4数值耗散

- `PairNumericalDamping(dE_dt_ij,h)`：

$$
ext{damping}=\frac12\rho_0 c_0\,\dot E_{ij}\,h
$$

- `NumericalDampingRightCauchy`/`NumericalDampingLeftCauchy`：用$$\dot F$$构造应变率并分离法向/剪切部分，返回一个“应力型”阻尼项，比例系数包含$$\rho_0,c_0,c_{s0}$$。

## 4.LinearElasticSolid（各向同性线弹性）

### 4.1 参数换算

构造函数输入$$(\rho_0,E,\nu)$$，代码计算：

$$
K_0=\frac{E}{3(1-2\nu)},\quad G_0=\frac{E}{2(1+\nu)},\quad \lambda_0=\frac{\nu E}{(1+\nu)(1-2\nu)}
$$

其中$$\lambda_0$$是第一Lame系数。

### 4.2本构公式（实现细节很重要）

- `StressPK2(F)`：

代码使用“对称化的F减I”作为小应变近似：

$$
\varepsilon=\frac12(F^T+F)-I
$$

$$
S=\lambda_0\,\mathrm{tr}(\varepsilon)I+2G_0\varepsilon
$$

- `StressPK1(F)`：$$P=FS$$。
- `StressCauchy(almansi)`：同样的线弹性形式

$$
\sigma=\lambda_0\,\mathrm{tr}(e)I+2G_0 e
$$

- `VolumetricKirchhoff(J)`：

$$
	au_{vol}=K_0 J(J-1)
$$

### 4.3 适用范围

- 适合小变形、小转动（因为应变用$$\frac12(F^T+F)-I$$，并不是严格的有限变形形式）。
- 如果存在较大旋转或大拉伸，建议用SVK/Neo-Hookean等超弹性。

`getRelevantStressMeasureName()`返回`PK2`，默认更偏向在Lagrangian框架做后处理。

## 5.Saint-Venant-Kirchhoff Solid（SVK超弹性）

与LinearElasticSolid同参数，但使用有限变形应变：

$$
E=\frac12(F^TF-I)
$$

$$
S=\lambda_0\,\mathrm{tr}(E)I+2G_0E
$$

适用范围：

- 允许较大位移/转动（几何非线性），但材料本构仍是“线弹性形式”，在大应变下可能出现不物理响应（例如压缩/拉伸非对称等）。

## 6.NeoHookeanSolid（可压缩Neo-Hookean）

### 6.1 `StressPK2(F)`

代码实现（并注明允许$$\det(F)$$为负，参考Smith2018的StableNeo-Hookean思路）：

令$$C=F^TF,J=\det(F)$$，则

$$
S=G_0 I+\left(\lambda_0(J-1)-G_0\right)J\,C^{-1}
$$

### 6.2`StressCauchy(almansi)`

代码从Almansi应变反解$$B$$：

$$
B=\left(-2e+I\right)^{-1},\quad J=\sqrt{\det(B)}
$$

$$
\sigma=\frac12K_0\left(J-\frac1J\right)I+G_0\,J^{-2/d-1}\left(B-\frac{\mathrm{tr}(B)}{d}I\right)
$$

### 6.3`VolumetricKirchhoff(J)`

$$
au_{vol}=\frac12K_0(J^2-1)
$$

### 6.4 适用范围

- 典型大变形弹性体/软组织背景材料。
- `getRelevantStressMeasureName()`返回`Cauchy`，更适合Eulerian意义的后处理。

## 7.NeoHookeanSolidIncompressible（不可压缩偏）

### 7.1`StressPK2(F)`

令$$C=F^TF$$，$$I_1=\mathrm{tr}(C)$$，$$I_3=\det(C)$$，则

$$
S=G_0 I_3^{-1/3}\left(I-\frac13 I_1 C^{-1}\right)
$$

### 7.2`VolumetricKirchhoff(J)`

仍使用

$$
	au_{vol}=\frac12K_0(J^2-1)
$$

### 7.3重要限制

- `StressCauchy()`在cpp里标记TODO并返回空矩阵，因此不能用于Cauchy应力积分路径。
- 头文件注释写明：Currentlyonlyworkswith`DecomposedIntegration1stHalf`,notwith`Integration1stHalf`。

## 8.OrthotropicSolid（正交各向异性，仅3D）

构造输入：3个正交主方向$$a_i$$、对应$$E_i,G_i,\nu_i$$（i=1..3）。

实现要点：

- 继承`LinearElasticSolid`时，为了“时间步/声速估计”，父类用$$\max(E_i)$$和$$\max(\nu_i)$$近似。
- 派生类内部会构造$$A_i=a_i\otimes a_i$$，以及各向异性系数`Mu_[i]`与`Lambda_(i,j)`。

`StressPK2(F)`使用有限变形应变$$E=\frac12(F^TF-I)$$，然后做双重求和组装：

$$
S=\sum_{i=1}^{d}\mu_i\left(A_iE+EA_i+\frac12\sum_{j=1}^{d}\Lambda_{ij}\left(\langle A_i,E\rangle A_j+\langle A_j,E\rangle A_i\right)\right)
$$

其中$$\langle A,E\rangle$$对应代码里的`CalculateBiDotProduct(A_[i],strain)`。

VolumetricKirchhoff(J)沿用线弹性的

$$
	au_{vol}=K_0J(J-1)
$$

适用范围：

- 仅3D。
- 适合木材、复合材料、纤维增强材料等“正交各向异性”弹性。

## 9.FeneNeoHookeanSolid（FENE有限延展）

在Neo-Hookean基础上引入有限延展参数`j1_m_`（默认1.0）。

令$$C=F^TF$$，$$\text{strain}=\frac12(C-I)$$，$$J=\det(F)$$，则

$$
S=\frac{G_0}{1-\frac{2\,\mathrm{tr}(\text{strain})}{j1_m}}I+\left(\lambda_0(J-1)-G_0\right)J\,C^{-1}
$$

适用范围：

- 适合链网络/橡胶类材料在大拉伸下“逐渐变硬”的效应。
- 需要避免$$1-2\,\mathrm{tr}(\text{strain})/j1_m\to0$$导致奇异。

`getRelevantStressMeasureName()`返回`Cauchy`。

## 10.Muscle（全局正交各向异性肌肉）

### 10.1 参数与方向

- 输入：$$\rho_0$$、`bulk_modulus`（体积模量意义）、参考纤维方向$$f_0$$、片层方向$$s_0$$、以及4组材料参数`a0_[k]`、`b0_[k]`。
- 内部构造：$$f_0\otimes f_0$$、$$s_0\otimes s_0$$、$$f_0\otimes s_0+s_0\otimes f_0$$。

### 10.2 由bulk_modulus与a0,b0反推(E,nu)

代码里把背景剪切模量取为$$G=a0$$（仅background）：

$$
\nu=\frac12\frac{3K-2G}{3K+G},\quad E=3K(1-2\nu)
$$

### 10.3 StressPK2(F)

令$$C=F^TF,J=\det(F)$$，并定义不变量（实现完全一致）：

$$
I_{ff}-1=f_0^TCf_0-1
$$

$$
I_{ss}-1=s_0^TCs_0-1
$$

$$
I_{fs}=f_0^TCs_0
$$

$$
I_1-1=\mathrm{tr}(C)-d
$$

则

$$
\begin{aligned}
S&=a_0^{(0)}e^{b_0^{(0)}(I_1-1)}I+\left(\lambda_0(J-1)-a_0^{(0)}\right)J\,C^{-1}\\
&\quad+2a_0^{(1)}(I_{ff}-1)e^{b_0^{(1)}(I_{ff}-1)^2}(f_0\otimes f_0)\\
&\quad+2a_0^{(2)}(I_{ss}-1)e^{b_0^{(2)}(I_{ss}-1)^2}(s_0\otimes s_0)\\
&\quad+a_0^{(3)}I_{fs}e^{b_0^{(3)}I_{fs}^2}(f_0\otimes s_0+s_0\otimes f_0)
\end{aligned}
$$

VolumetricKirchhoff(J)：

$$
	au_{vol}=K_0J(J-1)
$$

适用范围：

- 适合具有纤维增强的软组织/肌肉，被动各向异性响应。
- 这里的方向是“全局常量”（所有粒子相同）。

`getRelevantStressMeasureName()`返回`Cauchy`。

## 11.LocallyOrthotropicMuscle（局部纤维/片层方向）

与`Muscle`同本构，但纤维方向`local_f0_[i]`与片层方向`local_s0_[i]`随粒子变化。

源码行为：

- `registerLocalParameters()`注册粒子状态变量`Fiber`与`Sheet`。
- `initializeLocalParameters()`派生出`FiberFiberTensor`、`SheetSheetTensor`、`FiberSheetTensor`。
- `StressPK2(F,i)`把`Muscle`里的$$f_0,s_0$$替换为`local_f0_[i]`、`local_s0_[i]`及其张量积。

适用范围：

- 适合纤维场空间变化（例如心肌纤维螺旋分布）。

## 12.与固体动力学积分格式的对齐（怎么选模型更稳）

`elastic_dynamics.cpp`里提供多条应力路径：

- `Integration1stHalfPK2`：直接用`StressPK2(F)`，再转PK1。
- `Integration1stHalfKirchhoff`：用`VolumetricKirchhoff(J)`+`DeviatoricKirchhoff(deviatoric_b)`组装Kirchhoff应力。
- `Integration1stHalfCauchy`：用`StressCauchy(almansi)`得到$$\sigma$$，再转PK1。
- `DecomposedIntegration1stHalf`：显式使用`VolumetricKirchhoff(J)`并做一项与`ShearModulus()`相关的修正，再加LeftCauchy数值耗散。

因此选择建议：

- 如果材料类没有实现`StressCauchy`(例如`NeoHookeanSolidIncompressible`目前TODO)，就不要走Cauchy路径。
- 如果希望用Kirchhoff分解形式，需要`VolumetricKirchhoff(J)`合理且`DeviatoricKirchhoff`足够表达材料偏应力；当前`ElasticSolid`默认的`DeviatoricKirchhoff`是$$G_0\,\text{deviatoric\_be}$$，更接近各向同性剪切。
- 各向异性材料(`OrthotropicSolid`/`Muscle`/`LocallyOrthotropicMuscle`)主要通过`StressPK2`提供完整偏应力，通常更适合走PK2路径。

## 13.适用范围速查

- `LinearElasticSolid`：小应变、小转动；快但在大变形下不可靠。
- `SaintVenantKirchhoffSolid`：有限变形几何；中等应变可用，但极大应变可能不物理。
- `NeoHookeanSolid`：大变形弹性体常用；实现含稳定化处理；同时支持Cauchy后处理。
- `NeoHookeanSolidIncompressible`：不可压缩偏；当前不支持`StressCauchy`；按注释更适合`DecomposedIntegration1stHalf`。
- `OrthotropicSolid`：3D正交各向异性；适合方向性弹性材料。
- `FeneNeoHookeanSolid`：大拉伸有限延展效应；注意接近奇异的参数区。
- `Muscle`/`LocallyOrthotropicMuscle`：软组织/肌肉被动各向异性；后者方向可随空间变化。

