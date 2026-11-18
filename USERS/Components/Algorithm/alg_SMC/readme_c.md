# C 版本用法

`slidingmodec.c/.h` 是`SMC` 类的纯 C 实现，它使用结构体保存控制器状态，使用函数模拟构造函数和成员函数，因此每个控制轴都可以拥有独立实例。

```c
#include "slidingmodec.h"

static SMC yaw_smc;

void Control_Init(void)
{
    SMC_Init(&yaw_smc, 20.0f, 120.0f, 0.0f, 0.001f,
             21.0f, 27.0f, 25000.0f, 0.8f, 0.5f);
}

void Control_Loop(float angle_now, float angle_vel, float target_angle)
{
    yaw_smc.target = target_angle;
    SMC_Tick(&yaw_smc, angle_now, angle_vel);
    /* 将 yaw_smc.u 映射到电机驱动协议。 */
}
```

`SMC_Reset()` 只清除运行时历史量并保留控制参数，可在重新启用控制器时调用。
