在`tests/`提供的案例（如二维溃坝）中，尽管用户明确指定了每隔0.1秒输出一次粒子信息到VTP文件，可是得到的VTP文件的物理时间仍然不是0.1的倍数。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20260104220329.png)

这是因为SPHinXsys模拟时采用可变时间步长。判断是否要输出到文件时，只需要满足物理时间超过`output_interval`即可，并没有要求物理时间恰好等于`output_interval`。

为了在精确的0.1, 0.2, 0.3, …秒输出，我们需要限制一下时间步长。

双标准时间步推进一般采用下列程序：

```cpp
while (physical_time < end_time) // 外层循环
{
 	...
    while (integration_time < output_interval) // 中层循环
    {
        Real advection_dt = fluid_advection_time_step.exec();
        ... // 按对流时间步长推进
        while (relaxation_time < advection_dt) // 内层循环
        {
            acoustic_dt = fluid_acoustic_time_step.exec();
            ... // 按声速时间步长推进
            relaxation_time += acoustic_dt;
            integration_time += acoustic_dt;
            physical_time += acoustic_dt;
        }
    }
    ... // 写入VTP文件
}
```

一旦`integration_time >= output_interval`，就会输出。而这个`integration_time`的累加是由`acoustic_dt`推进的。所以我们只要控制好`acoustic_dt`就能够让某个时间点恰有`integration_time == output_interval`。因此，我们要盯着当前的`integration_time`距离`output_interval`还有多远。假如下一个`acoustic_dt`会使得`integration_time > output_interval`，就说明`acoustic_dt`迈大了，要把步子收一收。也就是说

```cpp
            acoustic_dt = fluid_acoustic_time_step.exec();
            acoustic_dt = SMIN(acoustic_dt, output_interval - integration_time); // revision 1
```

如此可以保证`integration_time`会精确落在`output_interval`的位置。

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20260105103935.png)

但是这还没完。如果最后一个步子迈小了，很有可能导致在`relaxation_time < advection_dt`时，便已经满足了`integration_time == output_interval`。此时已经无法再向前推进，于是内层循环永远无法退出：

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20260105104414.png)

为了解决这个问题，我们需要在内层循环的判断条件再加一条：只有当`integration_time < output_interval`时，才能进入内层while循环。一旦`integration_time`达到`output_interval`，便退出内层循环。

```cpp
		while (relaxation_time < advection_dt && integration_time < output_interval) // revision 2
```

完整的修改如下：

```cpp
        while (relaxation_time < advection_dt && integration_time < output_interval) // revision 2
        {
            /** inner loop for dual-time criteria time-stepping.  */
            acoustic_dt = fluid_acoustic_time_step.exec();
            acoustic_dt = SMIN(acoustic_dt, output_interval - integration_time); // revision 1
            fluid_pressure_relaxation.exec(acoustic_dt);
            fluid_density_relaxation.exec(acoustic_dt);
            relaxation_time += acoustic_dt;
            integration_time += acoustic_dt;
            physical_time += acoustic_dt;
        }
```

经过这次修改，VTP文件可以在0.1, 0.2, 0.3, …秒输出了：

![](https://fengimages-1310812903.cos.ap-shanghai.myqcloud.com/20260105104858.png)