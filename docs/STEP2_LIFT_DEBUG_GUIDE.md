# Step 2：抬升机构调试指南

> 前置状态：Step 1 麦轮底盘已通过（遥控链路在线、四个底盘电机反馈与方向正常）。本步骤只验证抬升：2×M3508 角度闭环 + 2×DM J4310 位置闭环。
>
> 与 Step 1 最大的区别：**抬升目前没有遥控映射，也没有按键触发**。[robot_adapter.c](../ROBOT/robot_adapter.c) 只产生底盘速度与拨弹请求，抬升目标只能通过 `robot_lift_set_target_rad()` 写入，而该函数当前**没有任何调用者**。因此本步骤的目标值必须由调试器注入，不能靠推摇杆。

## 1. 控制链路

```text
调试器：call robot_lift_set_target_rad(x)   （或直接写 s_lift_target_rad / s_lift_target_valid）
  -> s_lift_target_rad / s_lift_target_valid
  -> robot_lift_update(dt)：目标 × k_lift_*_direction
       ├─ M3508 角度模式：目标角 - 反馈角 → 角度环（Kp 20，输出 ±30 rad/s 速度目标）
       │     -> 速度环（Kp 2.0 / Ki 0.05，输出 ±16384 电流命令）
       │     -> 电流环（Kp 1.0）-> 0x1FF 打包下发
       └─ J4310 位置速度模式：position=目标，velocity 固定 1.0 rad/s -> 主 ID 0x01/0x02 逐台下发

M3508 抬升反馈：FDCAN1 FIFO0 -> CAN_Task -> 0x205/0x206 -> module_dji_motor_bus_handle_feedback
J4310 抬升反馈：FDCAN2 FIFO1 -> CAN_Task -> 0x11/0x12   -> module_dm_motor_bus_handle_feedback
```

输出总门在 `app_safety`：上电默认关闭，控制周期内按 `robot.remote.online` 打开。**遥控器必须开机并在线，否则四台抬升电机每周期都会被失能**（DJI 组发 0 电流，DM 组直接发 DISABLE 帧）。这意味着 Step 1 的遥控链路是本步骤的硬前提。

## 2. 涉及文件

| 文件 | 用途 |
|---|---|
| [robot_config.c](../ROBOT/robot_config.c) / [robot_config.h](../ROBOT/robot_config.h) | 抬升电机实例、CAN ID、方向符号、角度环 PID、DM 限制参数、`robot_lift_update()` |
| [robot_task.c](../ROBOT/robot_task.c) | 1 kHz 控制入口，调用 `robot_lift_update()` / `robot_try_enable_motors()` |
| [robot_adapter.c](../ROBOT/robot_adapter.c) | 遥控 → 命令映射；**当前不含抬升输入** |
| [module_dji_motor.c](../USERS/Components/module/module_dji_motor/module_dji_motor.c) | M3508 角度/速度/电流三环、编码器多圈累计、0x1FF 打包 |
| [module_dm_motor.c](../USERS/Components/module/module_dm_motor/module_dm_motor.c) / [module_dm_motor_bus.c](../USERS/Components/module/module_dm_motor/module_dm_motor_bus.c) | J4310 位置速度模式编码、使能/失能命令、故障码与温度解析 |
| [module_motor.c](../USERS/Components/module/module_motor/module_motor.c) | 电机基类状态机、反馈在线标志、输出总门判断 |
| [app_safety.c](../USERS/Application/app/app_safety/app_safety.c) | 遥控监控器与整机输出门 |
| [Control_Task.c](../USERS/Application/Task/Src/Control_Task.c) | 控制周期（默认 1 ms，超过 20 ms 时回落到 1 ms） |

## 3. 当前代码中的抬升配置

- **M3508 抬升**：ID `5`、`6`，接 FDCAN1，反馈 ID `0x205`、`0x206`，发送组 `0x1FF`（槽位 0/1）。控制模式为角度模式：编码器上电清零（`MODULE_DJI_POSITION_BOOT_RELATIVE`），`target_angle_rad` 以**上电位置为 0**。
- **角度环**：`Kp 20 / Ki 0 / Kd 0`，输出为速度目标，限幅 `±30 rad/s`。速度环沿用底盘参数 `Kp 2.0 / Ki 0.05`，电流命令限幅 `±16384`。误差 0.1 rad 时角度环即已饱和，所以目标必须小步给。
- **J4310 抬升**：主 ID `0x01`、`0x02`，反馈 ID `0x11`、`0x12`，接 FDCAN2，位置速度模式。目标速度在 [robot_config.c](../ROBOT/robot_config.c#L353-L371) 中固定为 `1.0 rad/s`。
- **DM 限制参数**：`±12.5 rad`、`±30 rad/s`、`±10 Nm`，必须与电机固件 PMAX/VMAX/TMAX 一致。目标超出 `±12.5 rad` 时 `module_dm_motor_set_position_velocity_target()` 返回 `OUT_OF_RANGE`，而 `robot_lift_update()` **忽略返回值**——表现为该组静默不动。
- **方向符号**：`k_lift_dji_direction = {1, -1}`、`k_lift_dm_direction = {1, -1}`，只作用于下发目标。注意 M3508 的 `direction_sign` 同时作用于反馈，而 J4310 结构体里没有方向字段，**DM 反馈位置始终是原始符号**，对比"目标-反馈"时要按机械同向判断，不能只看数值符号。
- **零位参考不一致**：M3508 的 0 是上电位置，J4310 的 0 是绝对编码器零点。四台电机共用同一个标量目标，因此两组零位必须在机械上对齐后，同一目标才有相同含义。
- **使能**：`robot_try_enable_motors()` 在反馈 `is_online` 后自动使能，不依赖遥控；J4310 需先收到一帧反馈才能使能。
- **反馈"在线"是粘滞的**：`module_motor_set_feedback_timeout()` / `module_motor_update_feedback_time()` 当前无人调用，`is_online` 一旦置 true 不会再变 false。**判断链路是否真的在更新，要看 `feedback.update_count` 是否持续增长**，不能只看 `is_online`。

## 4. 调试前安全检查

1. 抬升机构断电后可能因自重下落，且没有行程开关。先把机构**机械支撑或锁定在最低位**，确认断电也不会砸落、不会夹手；测试全程手不进入剪叉/导轨夹点。
2. 确认四台抬升电机的安装方向：给一个正目标时两组必须推动机构**同向**。方向未确认前，目标幅值不超过 `0.05 rad`。
3. 在达妙调试助手中逐台核对 J4310：主 ID `0x01/0x02`、反馈 ID `0x11/0x12`、CAN 1 Mbps、控制模式与代码一致（位置速度）、PMAX/VMAX/TMAX 与 `k_dm4310_limits` 一致、通信丢失保护时间不要过短。
4. 核对 M3508 ID `5/6`、FDCAN1 接线与 `0x1FF` 发送组；确认总线上没有第二个设备占用 `0x205/0x206`。
5. 遥控器开机并确认在线（输出门的唯一前提），右拨杆保持非上位；准备实体急停或直接断电手段。
6. 上电前把机构停在"两组零位都成立"的位置：M3508 上电位置即其 0；J4310 必须在绝对 0 附近，否则上电后它会直接向绝对 0 位置运动。

## 5. 调试步骤与验收标准

### 5.1 上电与反馈

1. 烧录并启动，确认 `robot.initialized == true`。
2. 遥控开机，确认 `robot.remote.online == true`、`robot.safety.output_allowed == true`（输出门关闭时抬升不会响应任何目标）。
3. 逐台检查反馈：
   - `robot.lift_dji[i].super.feedback`：`position_rad`、`velocity_rad_per_s`、`current_a`、`motor_temperature_c`、`update_count`；
   - `robot.lift_dm[i].super.feedback`：同上，另加 `module_dm_motor_get_fault()`、`module_dm_motor_get_mos_temperature_c()`。
4. 观察上电瞬间：M3508 目标默认为 0，即锁定上电位置，不应有明显动作；J4310 目标默认为绝对 0 且速度目标为 0，若当前位置远离 0 可能出现动作。**一旦出现非预期动作立即断电**，先回去做零点标定，不要继续给目标。
5. 若 J4310 报 `fault >= 8`，电机已进入 FAULT。该状态会被 `robot_try_enable_motors()` 拒绝对待，且工程内没有清故障调用，只能断电重启或后续补 `module_motor_clear_fault()`。

**通过条件：** 四台电机 `update_count` 持续增长、无 FAULT、温度正常，且上电过程机构无意外动作。

### 5.2 零位与方向确认（单步极小目标）

1. 确认机构已支撑、周围无人后，用调试器写入目标：`call robot_lift_set_target_rad(0.05f)`。若该调用在 Cortex-Debug 下不可用，改为在变量窗口写 `s_lift_target_rad = 0.05`、`s_lift_target_valid = true`。
2. 等待 2 s 以上，观察两组机械是否同向、是否有异响或互相顶住。
3. 记录每台电机的 `position_rad` 变化量：M3508 应与目标同号（其 `direction_sign` 为 +1）；J4310 中 `k_lift_dm_direction = -1` 的那台，**反馈符号与目标符号相反是正常的**，以机械运动方向为准。
4. 再写 `-0.05`、`0`，确认能回位且松手后保持不动。
5. 若某组反向：只改对应的一项 `k_lift_dji_direction[i]` 或 `k_lift_dm_direction[i]`，重新编译验证。**一次只改一项**，不要靠改目标符号或交换电机线掩盖问题。

**通过条件：** 正目标时两组同向，负目标反向，0 目标保持；无卡滞、无异响、无单组不动。

### 5.3 小阶跃闭环

1. 按 `0.05 → 0.1 → 0.2 rad` 逐级给目标，每级稳定后再加，步间至少 2 s。
2. 每级记录：到位时间、超调量、稳态误差、保持 5 s 内的电流波动。M3508 角度环 `Kp=20` 且无微分，出现持续抖动优先怀疑机械间隙与编码器量化，不要直接加 Kd。
3. J4310 的速度目标固定 1 rad/s，响应明显慢于 M3508。若两组机械刚性连接，小目标下就可能出现互相拉扯：先排查零位与方向，**不要用加大目标的方式"顶过去"**。
4. 若目标超过 `±12.5 rad`，J4310 会静默不动而 M3508 继续动，两组立刻撕裂。行程上限必须按 `±12.5 rad` 预留余量。

**通过条件：** 每级目标两组都能到位、保持稳定、无过冲振荡，M3508 与 J4310 不对顶。

### 5.4 目标回零与断电行为

1. 把目标写回 `0`，确认机构停在上电/零点位置。
2. 关闭遥控器，验证 100 ms 内 `output_allowed` 变为 false：M3508 组电流归零（**机构处于自由状态，会因自重下落**），J4310 组收到 DISABLE。
3. 重新开遥控，确认自动使能恢复。J4310 若在失联期间进入 FAULT，需要断电重启。

**通过条件：** 失联后 100 ms 内全部抬升输出消失；恢复遥控后能重新使能且不冲击。

### 5.5 带载与行程验证

仅在前述步骤全部通过后进行。把机构支撑移除，先带轻载在行程中段做 ±0.1 rad 往返，再逐步扩大行程；两端必须留机械余量（无行程开关）。出现异响、单组掉力、电机或 MOS 温度快速上升、机构下沉，立即断电并回到 5.2 排查。

**通过条件：** 带载往返到位、保持不下沉、电流与温度在合理范围、两组动作协调。

## 6. 常见现象定位

| 现象 | 优先检查 |
|---|---|
| 四台抬升都不动 | 遥控是否在线（输出门）；`robot.initialized`；是否真的写过目标（`s_lift_target_valid`）；四路反馈 `update_count` 是否在涨 |
| 只有 M3508 动或只有 J4310 动 | 该组 `control_mode` 是否与代码一致（J4310 必须是位置速度模式）；目标是否超出 `±12.5 rad` 导致 DM 静默拒绝；DM 是否处于 FAULT |
| 上电瞬间机构自己动 | J4310 当前位置远离绝对 0；先在达妙工具里把机械零位设为零点，并确认上电位置与 0 对齐 |
| 给了目标但一组反向 | 对应 `k_lift_*_direction` 符号；M3508 还要核对其 `direction_sign` 与轮序/安装 |
| 两组互相顶、异响、抖动 | 零位参考不一致（M3508 上电相对 / J4310 绝对）、方向符号错、机械刚性耦合下速度目标差异过大 |
| 跑一下就掉力 | J4310 固件通信丢失保护时间过短或 CAN 丢帧；`Control_Task` 被拖慢；`transmit_error_count` 增长 |
| 到位后持续抖动 | 角度环 `Kp=20` 偏大、机械间隙、编码器量化；先用中间位置小目标复现 |
| 报故障后再也使能不上 | J4310 故障锁存（`fault` 非 0），工程内无清故障调用，需断电重启；M3508 超过 80 ℃ 会进 FAULT |
| 遥控失联后抬升下落 | 属正常断电行为，机构无抱闸；必须靠机械支撑，不能依赖软件保持 |
| `is_online` 一直是 true 但机构不响应 | 该标志不会自动转 false；改看 `update_count` / `elapsed_time_since_update_ms` |

## 7. 本步骤记录

调试完成后记录：固件版本、四台抬升电机的 ID 与安装位置、两组零位的标定方式、最终方向符号、角度环参数、J4310 的 PMAX/VMAX/TMAX 与实际固件值、每一级目标的到位时间与稳态误差、带载测试结果、失联与断电行为。临时使用过的目标值、临时写入的变量都应在记录中注明，并把机构最终停在安全位置。
