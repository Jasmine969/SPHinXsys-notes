# SingularVariable和DiscreteVariable有什么区别？

 **本质区别：一个是“单值(标量)变量”，一个是“按粒子/网格点离散的数组变量”。**

`SingularVariable<T>` 表示**整个对象只有一份的量**（全局/本体级别的单个值）。它内部持有一个 `T`（通过 `data_` 指针管理），典型用法是时间步长、计数器、总粒子数之类的“只有一个”的状态。例如 `BaseParticles` 里用 `SingularVariable<UnsignedInt>` 存 `TotalRealParticles`（真实粒子数量），因为这个量对整个粒子集合只有一个值，不是每个粒子一个值。实现上它提供 `setValue/getValue/incrementValue`，并支持把这个单值“委托/共享”到设备侧（`DeviceSharedSingularVariable`），这样 GPU/设备执行时也能读写同一个计数或状态（见 sphinxsys_variable.h）。

`DiscreteVariable<T>` 表示**离散化后的场变量**：对每个粒子（或离散点）都有一个 `T`，因此它内部是 `new T[data_size]` 的连续数组（`data_field_`）。典型用法是每粒子的 `Position(Vec d)`、`Density(Real)`、`Mass(Real)`、`VolumetricMeasure(Real)` 等。它提供 `Data()` 返回数组指针、`setValue(index, ...)`/`getValue(index)` 访问单个元素，并支持设备侧的数据副本/同步与扩容（`DelegatedOnDevice()`, `synchronizeWithDevice()`, `reallocateData(...)`），见 sphinxsys_variable.h。

**一句话总结**
- `SingularVariable<T>`：**1 个值**（global/singleton state），设备侧通常是“共享/委托同一份数据”。
- `DiscreteVariable<T>`：**N 个值**（per-particle/per-node array），设备侧通常需要“分配数组 + 同步/重分配”。

源码中`sv_`开头的变量都是`SingularVariable`类型，`dv_`开头的变量都是`DiscreteVariable`类型。