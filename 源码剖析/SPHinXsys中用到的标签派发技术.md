SPHinXsys源码有这样一些类：

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20251219142051.png)

它们在`sphinxsys_containers.h`定义或声明：

```cpp
class Base;  // Indicating base class
struct Fixed // Indicating with fixed adaptation
{
    static inline const bool is_adaptive = false;
    static inline const bool is_fixed = true;
    static inline const bool is_dynamic = false;
};
struct Adaptive // Indicating with adaptive resolution
{
    static inline const bool is_adaptive = true;
    static inline const bool is_fixed = false;
    static inline const bool is_dynamic = true;
};
template <typename... InnerParameters>
class Inner; /**< Inner interaction: interaction within a body*/

template <typename... ContactParameters>
class Contact; /**< Contact interaction: interaction between a body with one or several another bodies */
```

这些只是起了一个标签的作用，这种技术称作“标签派发（tag dispatch）”。

# 什么是tag dispatch

Tag dispatch指的是：用一些“只用于编译期分类的类型（tag types）”作为模板参数，让编译器在编译期选择不同的实现分支（模板特化/重载），从而避免运行时 if/else 或虚函数分派。
在SPHinXsys里，`Inner<...>`、`Contact<...>`、`Base`、`Adaptive`、`FreeSurface`等就是典型tag：它们的价值不在于提供接口，而在于“让模板系统选中某个版本”。

# 为什么 SPHinXsys 喜欢这样写

- 性能：分支在编译期决定，运行时没有虚函数开销，也更容易被内联优化。
- 组合性强：`Inner<FreeSurface, Adaptive>`这种“叠加标签”的写法天然支持“把策略组合起来”，比写一堆互相交织的`if`更可控。
- 接口稳定：上层调用经常只写一个别名（例如`DensitySummationComplex`），底层换实现不需要改调用点。
- 可读性（对库作者）：用标签把“算法维度”显式编码出来（`Inner` vs `Contact`、`Base` vs `Adaptive`、是否`FreeSurface`…），比靠注释约定更可靠。