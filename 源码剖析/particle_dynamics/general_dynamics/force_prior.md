```cpp
template <class DynamicsIdentifier>
void BaseForcePrior<DynamicsIdentifier>::update(size_t index_i, Real dt)
{
    force_prior_[index_i] += current_force_[index_i] - previous_force_[index_i];
    previous_force_[index_i] = current_force_[index_i];
}
```

这里看起来和`force_prior_[index_i] = current_force_[index_i]`效果是等价的。但其实作者这么写是有用意的。并不是只有`BaseForcePrior<DynamicsIdentifier>`的派生类能修改`force_prior_`，其他类也可能会修改`force_prior_`。比如`tests\3d_examples\test_3d_taylor_bar\taylor_bar.h`的`DynamicContactForceWithWall`就会在`interaction`中动态改变`force_prior_`。如果这里改为`force_prior_[index_i] = current_force_[index_i]`的话，那么`DynamicContactForceWithWall`对`force_prior_`的贡献就会被抹去了（除非把`DynamicContactForceWithWall::interaction`放后面执行）。

