# TLSPH的基本思想

TL框架中，一切积分、核函数梯度、邻域几何关系都在**初始构形**（参考构形 $$\Omega_0$$）上表达。对TLSPH来说，

- 粒子间参考距离$$ \mathbf{X}_{ij} = \mathbf{X}_i - \mathbf{X}_j$$**不随时间变**
- 核函数$$W(\mathbf{X}_{ij},h_0)$$与其梯度 $$\nabla_0 W$$**不随时间变**（除非自适应$$h$$）

与更新拉格朗日框架下的SPH相比，TLSPH可以避免邻域扭曲导致的核梯度噪声，并且更适合大变形固体。

# TLSPH的粒子变量

每个粒子$$i$$常存：

- 参考位置：$$\mathbf{X}_i$$
- 当前位置：$$\mathbf{x}_i$$，速度 $$\mathbf{v}_i$$
- 质量$$m_i$$、参考密度$$\rho_{0i}$$、参考体积$$V_{0i}=m_i/\rho_{0i}$$
- 变形梯度$$\mathbf{F}_i$$（或位移梯度$$\nabla_0 \mathbf{u}_i$$）
- 应力：$$\mathbf{P}_i$$或$$\mathbf{S}_i$$

邻域由参考距离决定：
$$
\mathbf{X}_{ij}=\mathbf{X}_i-\mathbf{X}_j,\quad r_{ij}=\|\mathbf{X}_{ij}\|.
$$

# TLSPH对矢量场的梯度近似

## 无修正版本

对任意矢量场$$\mathbf{v}(\mathbf{X})$$，其参考构型的梯度离散为（推导过程类似[SPH梯度近似的强形式和弱形式 - 知乎](https://zhuanlan.zhihu.com/p/1994692537596798873)中强形式A2）：
$$
\boxed{\langle\nabla_0 \mathbf{v}\rangle_i=\sum_{j} V_{0j}(\mathbf{v}_j - \mathbf{v}_i)\otimes\nabla_0 W_{ij}}\tag{1}
$$
> 重点：此处全部是 $$\nabla_0$$（对参考坐标求导），这就是TL的核心。

## 梯度近似的一阶一致性修正

由于模拟过程中粒子分布不均和边界截断的问题，SPH的粒子梯度近似可能与实际梯度相差较大。实践中，我们想着**至少重构出线性（一阶）部分**，也即保证梯度近似的一阶一致性。假设此标量场为
$$
\mathbf{v}(\mathbf X)=a+\mathbf M\mathbf X+O(\mathbf X^2).
$$
其梯度为
$$
\nabla_0\mathbf{v}=\mathbf M.
$$
如果用一个修正矩阵$$\mathbf B_0$$乘上梯度近似式(1)，就能够恰好得到$$\mathbf M$$，那可就太好了，完美近似了线性部分：
$$
\left[\sum_{j} V_{0j}\,(\mathbf{v}_j - \mathbf{v}_i)\otimes\nabla_0 W_{ij}\right]\mathbf B_{0i}=\mathbf M
$$
将$$\mathbf{v}(\mathbf X)$$的线性部分代入：
$$
\mathbf M\left[\sum_{j} V_{0j}\,(\mathbf X_j - \mathbf X_i)\otimes\nabla_0 W_{ij}\right]\mathbf B_{0i}=\mathbf M
$$

记$$\mathbf A_{0i}=\sum_{j} V_{0j}\,(\mathbf X_j - \mathbf X_i)\otimes\nabla_0 W_{ij}$$。得到$$\mathbf B_{0i}$$的表达式：
$$
\boxed{\mathbf B_{0i}=\mathbf A_0^{-1}=\left[\sum_{j} V_{0j}(\mathbf X_j - \mathbf X_i)\otimes\nabla_0 W_{ij}\right]^{-1}}\tag{2}
$$
因此，我们对于任何一个矢量场$$\mathbf{v}(\mathbf{X})$$，只要用式(1)初步算出近似的梯度张量后右乘一个$$\mathbf B_0$$，就能够确保一阶一致性：
$$
\boxed{\langle\nabla_0 \mathbf{v}\rangle_i=
\left[\sum_{j} V_{0j}(\mathbf{v}_j - \mathbf{v}_i)\otimes\nabla_0 W_{ij}\right]\mathbf B_{0i}
}\tag{3}
$$
前提是$$\mathbf B_{0i}$$是可逆的。好在$$\mathbf B_{0i}$$完全依赖于初始构型，因此只需在模拟开始时计算一次。实践中，一般来说，粒子均匀分布就能够存在逆矩阵。为了避免奇异矩阵的影响，SPHinXsys采取了一种稳健的做法。首先，计算$$\mathbf A_{0i}$$的伪逆而不是直接求逆：
$$
\mathbf B_{0i}^{\dagger}=(\mathbf A_{0i}^T\mathbf A_{0i}+\varepsilon\mathbf I)^{-1}\mathbf A_{0i}^T.
$$
然后通过$$\mathbf A_{0i}$$的行列式判断病态程度——当$$|\mathbf A_{0i}| < \alpha$$时，需对其进行修正，使其偏向恒等矩阵$$\mathbf I$$；否则使用$$\mathbf B_{0i}^{\dagger}$$进行后续修正：
$$
\mathbf B_{0i}=\begin{cases}
\dfrac{|\mathbf A_{0i}|}{\alpha}\mathbf B_{0i}^{\dagger}+\dfrac{\alpha-|\mathbf A_{0i}|}{\alpha}\mathbf I,&|\mathbf A_{0i}| < \alpha;\\
\mathbf B_{0i}^{\dagger},& |\mathbf A_{0i}| \geq\alpha.
\end{cases}
$$

## 变形梯度$$\mathbf{F}$$的近似

变形梯度的连续定义是
$$
\mathbf{F} = \frac{\partial \mathbf{x}}{\partial \mathbf{X}}=\nabla_0\mathbf x
$$

也就是将$$\mathbf v(\mathbf X)=\mathbf x$$代入式(3)：
$$
\langle\mathbf{F}\rangle_i=\langle\nabla_0\mathbf x\rangle_i=\left[\sum_{j} V_{0j}(\mathbf{x}_j - \mathbf{x}_i)\otimes\nabla_0 W_{ij}\right]\mathbf B_{0i}
$$

因为$$\mathbf F=\nabla_0\mathbf u+\mathbf I$$（$$\mathbf u$$是位移矢量），所以变形梯度离散形式也常写为
$$
\langle\mathbf{F}\rangle_i=\langle\nabla_0\mathbf u\rangle_i+\mathbf I
=\left[\sum_{j} V_{0j}(\mathbf{u}_j - \mathbf{u}_i)\otimes\nabla_0 W_{ij}\right]\mathbf B_{0i}+\mathbf I
$$

# 动量方程的离散

动量方程的连续形式为：
$$
\rho_0 \ddot{\mathbf{x}} = \nabla_0\cdot \mathbf{P} + \rho_0\mathbf{b}
$$

参考[SPH梯度近似的强形式和弱形式 - 知乎](https://zhuanlan.zhihu.com/p/1994692537596798873)中的弱形式B2，我们可以将$$\nabla_0\cdot \mathbf{P}$$离散成
$$
\langle\nabla_0\cdot\mathbf P\rangle_i=\sum_j V_j(\mathbf P_i+\mathbf P_j)\cdot\nabla_0 W_{ij}
$$
有核梯度修正的版本为：
$$
\langle\nabla_0\cdot\mathbf P\rangle_i=\sum_j V_j(\mathbf P_i\mathbf B_{0i}+\mathbf P_j\mathbf B_{0j})\cdot\nabla_0 W_{ij}
$$
因此完整的动量方程离散为：
$$
\boxed{\rho_{0i} \ddot{\mathbf{x}}_i = \sum_j V_j(\mathbf P_i\mathbf B_{0i}+\mathbf P_j\mathbf B_{0j})\cdot\nabla_0 W_{ij} + \rho_{0i}\mathbf{b}_i.}
$$

# 时间步长控制

见Ye等（2025）：

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20260116130438.png)

# 参考文献

Ye, M. *et al.* DRLinSPH: an open-source platform using deep reinforcement learning and SPHinXsys for fluid-structure-interaction problems. *Engineering Applications of Computational Fluid Mechanics* **19**, 2460677 (2025).
