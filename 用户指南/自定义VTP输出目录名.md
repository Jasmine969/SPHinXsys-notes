默认的VTP输出目录是`output`，怎么改成自定义的名字呢？

一行代码搞定：

```cpp
IO::getEnvironment().resetOutputFolder("NewFolder");
```

SPHinXsys还提供了一个接口，可以在现有的输出目录名后面加上下划线`_`+指定后缀。例如，我们现在已经把目录名改成了`NewFolder`。下面

```cpp
IO::getEnvironment().appendOutputFolder("case1");
```

现在的目录名会变成`NewFolder_case1`。
