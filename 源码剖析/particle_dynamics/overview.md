我们能看到很多dynamics（例如`fluid_dynamics`的`BaseIntegration`和`DensitySummation<Base, DataDelegationType>`）会同时继承`LocalDynamics`和`DataDelegationType`。

这种写法的核心意图是把**“做什么”（dynamics的算法框架）**和**“能访问到哪些数据”（数据委托DataDelegate）**解耦，然后用模板把两者组合起来：
- `LocalDynamics`：给派生类提供对**本体（body）及其粒子系统**的统一访问方式与生命周期（例如initialize / update / interaction这一类框架约定）。
- `DataDelegationType`：给派生类提供**邻域/拓扑相关的数据视图**（邻居表、接触对等），决定这段dynamics能“看见”哪些邻居。

`LocalDynamics`是`BaseLocalDynamics<SPHBody>`的别名。它通常为派生类提供（或间接提供）：
- `body_`：当前dynamics作用的`SPHBody`
- `adaptation_`：该body的自适应/分辨率相关对象（影响smoothing length、kernel等）
- `particles_`：该body的粒子容器（用于访问粒子变量、数量等）
（具体成员名以源码为准，但语义上就是“拿到本体+其粒子+其自适应配置”。）

`DataDelegationType`是模板参数，常见取值包括`DataDelegateInner`或`DataDelegateContact`：
- `DataDelegateInner`：提供**同一body内部**的邻居关系（inner neighbors）。典型用途是内部相互作用：压力、黏性、密度求和等。
- `DataDelegateContact`：提供**跨body的接触**邻居关系（contact neighbors）。除邻居表外，还会额外暴露所有接触体（contact bodies）及其粒子信息，以便在interaction中读取“对方body的粒子变量”。

读代码时的快速判断方法：
- 模板参数/继承里出现`DataDelegateInner`→这段dynamics只依赖本体内部邻域。
- 出现`DataDelegateContact`或`contact_configuration_`之类字段→这段dynamics会遍历接触体，并访问对方粒子数据。