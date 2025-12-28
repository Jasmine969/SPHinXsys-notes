读了两周SPHinXsys源码，感觉C++水平达到了前所未有的高度。这次讲讲SPHinXsys的所有权设计（ownership design）。

> SPHinXsys aims to follow the concept of resource acquisition is initialization (RAII) strictly. Therefore, the ownership of all objects have been clarified. Basically, there are three types of ownerships.
>
> First, the SPHsystem, bodies, body relations, gravities and particle-dynamics methods are owned by the application, i.e. the main function. This requires that these objects are defined in the main function. Second, the particles and materials are owned by SPHbody; Third, though the geometries, are owned by SPHBody; the shapes owned by complex shape or body part during the simulation, these objects are temporally owned by base and derived constructors. Therefore, we need to use shared pointer for them.
>
> One important choice in SPHinXsys is that the ownership is only clarified at the construction phase. After that, observers of raw pointers are assigned and used during the simulation.

上面是[官网](https://www.sphinxsys.org/html/introduction.html#ownership-design-in-sphinxsys)的说明。SPHinXsys严格遵循RAII原则。RAII（Resource Acquisition Is Initialization）的直观理解是：把资源的生命周期绑定到对象生命周期——在构造阶段“拿到/绑定资源”，在析构阶段“释放资源”。智能指针是RAII在内存管理上的经典体现。RAII的价值不只是“避免内存泄漏”，更重要的是：哪怕中途 return/异常抛出，析构仍然会发生，从而把资源回收路径变得可靠、可维护。

# 应用层（main）拥有：系统/Body/Relation/Dynamics

首先，`SPHSystem`、`SPHBody`、`SPHRelation`、重力、粒子动力学方法等对象是归应用所有的，也即归main函数所有。因此这些对象是在main函数中定义的。

这句话背后其实是一个“生命周期约束”：仿真阶段大量模块之间只保存 raw 指针/引用（observer），所以必须保证这些 owner（由 main 管）在仿真过程中一直存活。

# SPHBody拥有：粒子与材料（以及一堆类似“只属于它”的对象）

其次，粒子和材料归`SPHBody`所有。不仅如此，neighbor builder归relation所有，time stepper归solver所有……凡此种种，不胜枚举。这也是为什么我们能在源码中看到大量的`unique_ptr`。它表达“唯一所有权（exclusive ownership）”，语义清晰、开销也最低。例如material：

```cpp
class SPHBody
{
  private:
    UniquePtrKeeper<BaseMaterial> base_material_ptr_keeper_;
  public:
    BaseMaterial &getBaseMaterial();
	template <class MaterialType = BaseMaterial, typename... Args>
    MaterialType *defineMaterial(Args &&...args)
    {
        MaterialType *material = base_material_ptr_keeper_.createPtr<MaterialType>(std::forward<Args>(args)...);
        base_material_ = material;
        return material;
    };
    ...
}
```

`material`的生存周期是跟着body走的：

- 生存周期由`base_material_ptr_keeper_`（内部是`unique_ptr`）管理
- 使用时采用原始指针（`base_material_`）。别的对象需要用到material，都是从`base_material_`访问（`getBaseMaterial`），不会再去涉及`base_material_ptr_keeper_`（而且也访问不到，因为它是私有成员）。

因此“智能指针只在构建时起作用”这句话可以更精确地说：智能指针负责所有权和析构；仿真阶段主要走原始指针路径以降低开销并简化数据访问。这也对应官网那句：

> ownership is only clarified at the construction phase… observers of raw pointers are assigned and used during the simulation.

# 为什么Shape偏偏用shared_ptr，而粒子/材料用unique_ptr？

另外，注意到源码中除了`unique_ptr`，还使用了`shared_ptr`：

```cpp
class SPHBody
{
  private:
    SharedPtrKeeper<Shape> shape_ptr_keeper_;
    UniquePtrKeeper<SPHAdaptation> sph_adaptation_ptr_keeper_;
    UniquePtrKeeper<BaseParticles> base_particles_ptr_keeper_;
    UniquePtrKeeper<BaseMaterial> base_material_ptr_keeper_;
    ...
}
```

为什么只有shape用的是shared pointer管理，其余用的都是unique pointer呢？直觉版答案：particles/material 往往是“一个 body 独占”的资源，用`unique_ptr`（唯一所有权）最合适、最省成本；而Shape更可能出现“多个对象共同持有/共同依赖其生命周期”的场景，用`shared_ptr`更稳健。

更落地一点：一个shape可能被多个对象共有，比如body、body part。看body part的源码：

```cpp
class BodyRegionByParticle : public BodyPartByParticle
{
  private:
    SharedPtrKeeper<Shape> shape_ptr_keeper_;
  
  public:
    BodyRegionByParticle(SPHBody &sph_body, Shape &body_part_shape);
    BodyRegionByParticle(SPHBody &sph_body, SharedPtr<Shape> shape_ptr);
    ...
}
```

可见`BodyRegionByParticle`的shape也是由shared pointer管理的，并且在初始化时也接受`shared_ptr`对象。如果用`unique_ptr`，就会遇到“到底谁拥有它？”、“怎么把所有权在body/body-part之间转移？”等麻烦；用`shared_ptr`则可以清晰表达：你们都可以是owner，还有一个owner存活，这个shape就不会被析构。

# 以二维溃坝为例：为什么壁面用shared_ptr，water用栈对象？

在二维溃坝案例中，定义`FluidBody`和`SolidBody`时调用了不同的构造函数：

```cpp
int main() {
    ...
    GeometricShapeBox initial_water_block(Transform(water_block_translation), water_block_halfsize, "WaterBody");
	FluidBody water_block(sph_system, initial_water_block);
	SolidBody wall_boundary(sph_system, makeShared<WallBoundary>("WallBoundary"));
    ...
}
```

创建`FluidBody`时传入的是`GeometricShapeBox`对象`initial_water_block`，而创建`SolidBody`时传入的是`WallBoundary`的`shared_ptr`。更准确地说：两者都能工作，但表达的“所有权归属”不同——

- `water_block`这里：body只“观察”这个 shape（保存`Shape*`），shape的生命周期由main的栈对象保证；确保它不会在`water_block`用完之前销毁，是用户（main函数）的责任。
- `wall_boundary`这里：shape由`shared_ptr`交给body的keeper托管，保证“形状与body同在”。body不需要了，shape也随之销毁。

从“抗重构/抗误用”的角度看，`makeShared`这种写法更稳：以后哪怕把一段建模代码抽到函数里、或添加新的body-part复用同一个shape，也更不容易因为作用域变化引入悬空指针问题。

# 小结

- Owner vs Observer：Owner 负责对象生命周期（析构/释放）；Observer只保存原始指针/引用用来访问，绝不负责释放。
- 构造期 vs 仿真期：SPHinXsys把“所有权澄清”集中在构造期完成；进入仿真期后主要靠原始指针互相引用，追求低开销与简单数据通路。
- `unique_ptr`何时用：资源天然“一对一归属”（body的particles/material、relation的neighbor builder、solver的time stepper等）→ 选`unique_ptr`表达唯一所有权。
- `shared_ptr`何时用：资源可能被多个模块共同持有/复用（典型是`Shape`：body 与 body-part、复杂几何组合与派生）→ 选`shared_ptr`表达共享所有权与安全保活。
- 最重要的约束：既然仿真期大量用observer raw pointer，那么就必须保证owner的生命周期更长（通常靠main的作用域/声明顺序来保证）。
