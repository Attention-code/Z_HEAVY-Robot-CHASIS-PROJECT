# Step 1：麦轮底盘调试指南

> 前置状态：遥控器链路和通道读取已经测试通过。本步骤不重复遥控器接收测试，重点验证四个底盘电机的 CAN 反馈、驱动方向和麦轮运动学。

## 1. 控制链路

```text
I6X 遥控器
  -> Ix6_Remote 解码（USART 接收）
  -> ROBOT/robot_adapter.c：通道映射、死区、失联保护
  -> ROBOT/robot_task.c：安全门控与周期调度
  -> app_chassis_update()：速度命令处理
  -> alg_mecanum_inverse()：车体速度 -> 四轮目标角速度
  -> module_dji_motor_bus_update()：M3508 速度 PID -> CAN 电流命令
  -> robot_port.c / BSP CAN -> FDCAN1 -> 四个 M3508

四个 M3508 反馈
  -> FDCAN1 FIFO0 中断只置接收标志
  -> CAN_Task / robot_task_can_receive()
  -> DJI CAN 分发器（ID 0x201~0x204）
  -> module_dji_motor_bus_handle_feedback()
  -> 电机在线状态与速度反馈
```

遥控器在线才允许控制输出；电机收到反馈并在线后，运行任务会尝试使能电机。遥控丢失会禁用底盘输出。调试时仍应准备实体急停/断电手段，不能把软件保护当作唯一安全措施。

## 2. 涉及文件

| 文件 | 用途 |
|---|---|
| [robot_adapter.c](../ROBOT/robot_adapter.c) | 遥控通道到车体速度的映射、死区和失联处理 |
| [robot_task.c](../ROBOT/robot_task.c) | CAN 反馈分发、控制周期、安全门控和电机总线刷新 |
| [robot_config.c](../ROBOT/robot_config.c) / [robot_config.h](../ROBOT/robot_config.h) | FDCAN、电机实例、CAN ID、麦轮几何参数、方向符号和 PID 配置 |
| [app_chassis.c](../USERS/Application/app/app_chassis/app_chassis.c) / [app_chassis.h](../USERS/Application/app/app_chassis/app_chassis.h) | 禁用/使能轮电机，调用逆运动学并下发四轮目标 |
| [alg_mecanum.c](../USERS/Components/Algorithm/alg_mecanum/alg_mecanum.c) / [alg_mecanum.h](../USERS/Components/Algorithm/alg_mecanum/alg_mecanum.h) | 麦轮运动学与轮速限幅 |
| [module_dji_motor.c](../USERS/Components/module/module_dji_motor/module_dji_motor.c) / [module_dji_motor.h](../USERS/Components/module/module_dji_motor/module_dji_motor.h) | M3508 反馈解析、速度环和发送组打包 |
| [cod-fdcan.c](../USERS/BSP/bsp_fdcan/cod-fdcan.c) | FDCAN 接收通知、FIFO 配置和错误状态 |
| [CAN_Task.c](../USERS/Application/Task/Src/CAN_Task.c) / [Control_Task.c](../USERS/Application/Task/Src/Control_Task.c) | CAN 接收任务和约 1 kHz 控制任务入口 |
| [main.c](../Core/Src/main.c) | 外设初始化后调用 `robot_init()` |

## 3. 当前代码中的底盘配置

- 电机：4 × M3508，接在 FDCAN1。
- 电机顺序与 ID：`robot.wheel[0..3]` 对应左前、右前、左后、右后；DJI 接收 ID 依次为 `0x201`、`0x202`、`0x203`、`0x204`，发送组为 `0x200`。
- 控制模式：四轮均为速度模式。当前轮电机 `direction_sign` 都是 `+1`；麦轮配置里的 `direction_sign` 为 `{+1, -1, +1, -1}`，用于补偿轮子安装方向。该配置需和实物方向核对。
- 运动学参数：轮半径 `0.076 m`、半轴距 `0.19 m`、半轮距 `0.21 m`、X 型辊子、最高轮角速度 `60 rad/s`。应以实测尺寸和辊子朝向为准。
- 遥控映射（均有负号）：左摇杆前后（CH3）控制 `vx`，右摇杆左右（CH1）控制 `vy`，左摇杆左右（CH4）控制 `wz`。代码给出的最大线速度为 `2.5 m/s`、最大角速度为 `6 rad/s`；首次上电应架空底盘，必要时先降低 [robot_adapter.c](../ROBOT/robot_adapter.c) 中的限幅。
- FDCAN1 的电机反馈走 FIFO0；CAN 任务将 ID `0x201~0x206` 分发，底盘只使用前四个 ID。FDCAN2 是 DM 电机总线，不属于本步骤的麦轮验证范围。

## 4. 调试前安全检查

1. 将底盘稳固架空，四个轮子均能自由转动，车体不会因轮子转动而移动；清空轮子周围区域。
2. 断开或确保抬升、拨弹等非底盘机构不会动作。遥控右拨杆处于非上位，避免触发拨弹单发。
3. 核对四个 M3508 的供电、CANH/CANL、共地、终端电阻和接插件；确认电机控制器/电调已上电。
4. 确认左前、右前、左后、右后电机线序与 `robot.wheel[0..3]` 一致，并确认 FDCAN1 的波特率与电机总线设置匹配。
5. 确认实体急停或电源切断方式可用。首次测试保持低幅度、短时操作；发现异常立即断电。

## 5. 调试步骤与验收标准

### 5.1 启动与反馈

1. 烧录并启动程序，通过调试器确认 `robot.initialized == true`（`main.c` 当前未处理 `robot_init()` 的返回值）；若未初始化成功，先排查配置，不要继续驱动测试。
2. 打开工程现有日志或调试器。遥控已经验证通过，本步骤只需确认控制周期内遥控在线状态仍有效，不必重新做遥控接收排查。
3. 在调试器中检查四个底盘电机反馈状态（通过 `module_motor_get_feedback(module_dji_motor_as_base(&robot.wheel[i]))` 查看 `is_online` 等字段）。四个电机都应持续在线，速度反馈应随手动转动对应轮子而变化。
4. 若某个轮子没有反馈，先停止测试；按该电机的 ID、FDCAN1 接线、供电和反馈数据逐项排查。确认只有底盘电机使用 ID `0x201~0x204`，没有同总线地址冲突。

**通过条件：** 四路电机反馈都能稳定更新，且控制板没有持续的 CAN 错误或电机离线状态。

### 5.2 低速方向点动

1. 确认架空稳定后，以尽可能小的摇杆幅度短促点动；每次只测试一个方向，松杆后等待车轮停下。
2. 逐项测试前进/后退、左移/右移、逆时针/顺时针旋转。注意坐标约定：`+x` 向前、`+y` 向左、`+wz` 从上方看逆时针为正。
3. 观察四轮是否按 X 型麦轮组合转动。前进/后退时四轮应共同产生纵向分量；横移时左右侧轮的方向组合应不同于纵向；原地旋转时左右侧轮组应反向。
4. 若某方向相反或某个轮子方向不符，立即停止并记录“摇杆方向—期望车体方向—实际四轮转向”。检查对应物理轮位、索引、`direction_sign` 和辊子 X/O 布局。一次只改一项，重新架空验证；不要靠改摇杆映射掩盖单个电机接反。
5. 若点动幅度偏大，可暂时降低遥控限幅。完成标定后再逐步增加，不要一开始用满杆验证。

**通过条件：** 三类运动方向正确；每种运动中四轮均参与且方向组合符合实物 X 型布局；松杆后命令回零且车轮停止。

### 5.3 落地低速验证

仅在架空测试全部通过后进行。将底盘放到平整、空旷地面，先使用低限幅，依次验证短距离前进/后退、横移和小角度旋转。每项之间回中并确认停车；出现跑偏、异响、明显抖动、单轮不动或电机/驱动器过热，立即停止并回到架空排查。

**通过条件：** 地面上的运动方向与架空验证一致，四轮输出协调，停车可控，电机反馈持续在线且无异常发热。

## 6. 常见现象定位

| 现象 | 优先检查 |
|---|---|
| 四轮都不动 | 遥控在线及安全门控、`robot_init()` 是否成功、四个反馈是否在线、驱动器使能/供电、FDCAN1 是否实际收发 |
| 单轮不动或反馈离线 | 电机供电与 CAN 接线、该电机接收 ID、轮序映射、驱动器状态 |
| 前进正常但横移/旋转错误 | `ALG_MECANUM_ROLLER_X` 与实物辊子布局、轮子索引顺序、`direction_sign` |
| 某轮方向相反 | 先核实该轮实际位置与索引，再核实方向符号；确认修改的是唯一原因 |
| 方向正确但运动量/跑偏异常 | 实测轮半径、半轴距/半轮距、轮胎打滑、M3508 速度 PID 与机械阻力；一次调整一个参数 |
| 遥控失联后仍有输出 | 立即断电并检查 `robot_adapter_update()`、安全门控及当前固件是否为本工程版本 |
| 松杆后仍有转动或抖动 | 检查遥控回中死区、速度 PID 参数、反馈是否在线及机械卡滞；在定位前不要落地运行 |

## 7. 本步骤记录

调试完成后记录固件版本、四个电机的实际轮位/ID、实测轮半径与轴距、最终方向符号、速度 PID 参数、架空测试结果和地面测试结果。任何临时降低的速度限幅都应记录，并在安全条件下确认最终配置。
