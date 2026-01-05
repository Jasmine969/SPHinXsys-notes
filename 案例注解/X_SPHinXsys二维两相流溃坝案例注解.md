User: 液体上面是有空气粒子的，为什么这里还要使用free surface类型的density summation？为什么不使用`InteractionWithUpdate<fluid_dynamics::BaseDensitySummationComplex<Inner<>, Contact<>, Contact<>>>
        update_water_density_by_summation(water_inner, water_air_contact, water_wall_contact);`？



这里的 `FreeSurface` 不是“上面没有空气粒子才用”，而是“**在密度求和得到的 $\rho_{\text{sum}}$ 可能因为支撑域缺失而偏低时，采用自由液面策略避免 $\rho$ 掉到参考密度以下**”。

**1) `DensitySummationComplexFreeSurface` 到底做了什么**
- 在 density_summation.h 里有别名：  
  `DensitySummationComplexFreeSurface = BaseDensitySummationComplex<Inner<FreeSurface>, Contact<>>`
- `Inner<FreeSurface>` 的关键行为在 density_summation.hpp：  
  它的 `update()` 直接做了
  $$ \rho_i \leftarrow \max(\rho_{\text{sum},i}, \rho_0) $$
  也就是**把密度下限钳在 $\rho_0$**。
- 这很重要，因为弱可压缩 EOS 往往是 $p \propto (\rho-\rho_0)$；若 $\rho<\rho_0$ 就会出现负压/拉伸不稳定风险（自由液面问题的典型来源）。

**2) 为什么 water 的 density summation 仍然用 “free surface”**
看你例子里 water 的更新是：
- `update_water_density_by_summation(water_inner, water_wall_contact);`  
  它**没有把 `water_air_contact` 算进去**（相界面方向等价于“缺邻居”）。

这通常是有意为之：在多相里，**水-气界面并不希望在“水的密度重建”阶段把气相当成填充水相核支撑域的邻居**，否则会把界面处理得更像“可混相的连续介质”，容易带来：
- 界面密度/体积的数值扩散（相间“掺混”感更强）
- 由于气相 $\rho_0$ 和质量尺度很不同，引入额外噪声（`ContactSummation` 会用到接触体的 `mass` 和 `rho0_k`，见 density_summation.cpp）
- 失去 “$\rho \ge \rho_0$” 的保护（若你用非 free-surface 的 inner 更新，就会允许 $\rho$ 掉下去）

换句话说：**虽然几何上水上方确实有气粒子，但在这个算例里，水相的“密度求和重建”仍把界面当作自由液面来稳定处理**；水-气相互作用主要由后面的 `MultiPhaseIntegration*WithWallRiemann` 压力/密度松弛步骤来负责。

**3) 为什么 air 的 density summation 用 `BaseDensitySummationComplex<Inner<>, Contact<>, Contact<>>`**
你现在的 air 更新：
- `update_air_density_by_summation(air_inner, air_water_contact, air_wall_contact);`

空气相通常不希望做 “$\rho \ge \rho_{0,a}$” 这种钳制（会抹掉它应有的可压缩波动/稀疏区），所以用普通 `Inner<>`。同时空气在靠近水和墙时确实需要把两种接触都计入密度重建（否则空气在界面/壁面附近也会严重缺邻居）。

**4) 你给的替代写法为什么不建议（以及还差一点写对）**
你贴的这行从语义上是“用 air 的类型去更新 water”（变量名/参数也对不上）：
- `update_air_density_by_summation(water_inner, water_air_contact, water_wall_contact);`

如果你的真实意图是“water 也把 air_contact 算进去”，正确的类型至少应是类似：
- `BaseDensitySummationComplex<Inner<...>, Contact<>, Contact<>> update_water_density_by_summation(water_inner, water_air_contact, water_wall_contact);`

但这么做会让 water 的 $\rho_{\text{sum}}$ 受到气相粒子的质量与 $\rho_{0,a}$ 的影响（见 density_summation.cpp），并且如果你用的是 `Inner<>` 而不是 `Inner<FreeSurface>`，还会失去自由液面下限钳制——这就是这个算例当前设计想避免的点。

做了两次试验，可以看出，还是原有代码最优。变体密度场更加振荡，并且出现了更显著的相间渗入。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20260104213905.png)