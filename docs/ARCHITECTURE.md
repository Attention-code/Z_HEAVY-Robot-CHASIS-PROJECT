# COD-H7-Template 工程文档

> 目标机型：STM32H723XG + FreeRTOS
> 硬件配置：4×M3508 麦轮底盘 + 2×M3508 抬升 + 2×DM J4310 抬升 + 1×DM J4310 拨弹盘（共 9 个电机）

---

## 1. 整体分层

工程按「硬件 → 抽象 → 组件 → 应用 → 项目适配」五层组织，依赖方向单向向下，不允许反向引用。

```
┌──────────────────────────────────────────────────────────────┐
│  Core/                     CubeMX 生成：时钟、外设、中断、任务创建 │
├──────────────────────────────────────────────────────────────┤
│  ROBOT/                    项目适配层：整车实例、参数总表、遥控制导 │
│      robot_config  robot_adapter  robot_port  robot_task       │
├──────────────────────────────────────────────────────────────┤
│  USERS/Application/        任务层 + App 层                       │
│      Task/*   app/app_chassis   app/app_safety   app/app_shooter│
├──────────────────────────────────────────────────────────────┤
│  USERS/Components/         组件层：算法库 + 设备驱动              │
│      Algorithm/alg_*        module/module_*   module/<设备>     │
├──────────────────────────────────────────────────────────────┤
│  USERS/BSP/                板级支持：外设封装 + 硬件抽象           │
│      旧 COD 驱动（bsp_adc/gpio/mcu/pwm/spi/tick/uart/fdcan）     │
│      新 ECF 抽象（bsp_common/can/log/dwt/watchdog）             │
└──────────────────────────────────────────────────────────────┘
```

工程当前处于**新旧两代代码交接态**：`USERS/BSP/bsp_adc` 等 8 个目录是旧 COD 扁平驱动，`bsp_can` 等 5 个是新 ECF 抽象。二者职责不同，不是重复实现（详见 4.1）。

---

## 2. 目录结构

```
COD-H7-Template-main/
├── CMakeLists.txt              用户源码收集 + include 路径
├── CMakePresets.json           Debug / Release 预设
├── STM32H723XG_FLASH.ld        链接脚本
├── Z_HEAVY_Robot_CHASIS.ioc    CubeMX 工程
│
├── Core/                       CubeMX 生成代码（勿手改，用户代码放 USER CODE 区）
│   ├── Inc/                    main.h / FreeRTOSConfig.h / 各外设句柄声明
│   └── Src/                    main.c / freertos.c / stm32h7xx_it.c / fdcan.c / ...
│
├── Drivers/                    ST HAL 与 CMSIS（原样保留）
├── Middlewares/                FreeRTOS / USB Device / ARM DSP 库
├── USB_DEVICE/                 USB CDC 虚拟串口
├── SystemView/                 SEGGER 调试（当前未接入构建）
│
├── ROBOT/                      项目适配层
│   ├── robot_config.c/.h       整车对象 robot_t + 全部参数（CAN ID、PID、机构尺寸）
│   ├── robot_adapter.c/.h      遥控器通道 → 底盘/拨弹命令
│   ├── robot_port.c/.h         HAL FDCAN → bsp_can_driver_ops_t 操作集
│   └── robot_task.c/.h         周期调度：CAN 收帧 / 控制环路
│
└── USERS/
    ├── Application/
    │   ├── Task/Inc/           CAN_Task.h  Control_Task.h  Detect_Task.h
    │   │                       INS_Task.h  Config.h
    │   ├── Task/Src/           对应 4 个任务实现
    │   └── app/
    │       ├── app_types.h     跨模块命令/反馈结构体与枚举
    │       ├── app_chassis/    底盘 App（麦轮解算 + 闭环）
    │       ├── app_safety/     安全门控（监控器超时 → 关输出）
    │       └── app_shooter/    射击器 App（拨弹）
    │
    ├── BSP/
    │   ├── bsp_adc/            板载 ADC 初始化
    │   ├── bsp_gpio/           GPIO 初始化 + 引脚下标定义
    │   ├── bsp_mcu/            MCU_Init()：外设初始化总入口
    │   ├── bsp_pwm/            TIM PWM 启动 + 占空比设置（BMI088 加热）
    │   ├── bsp_spi/            SPI 读写（BMI088）
    │   ├── bsp_tick/           Delay_us / Delay_ms
    │   ├── bsp_uart/           USART1 DMA 双缓冲 + UART5 SBUS + UART7 打印
    │   ├── bsp_fdcan/          FDCAN1/2/3 初始化 + 中断回调（只置标志）
    │   ├── bsp_common/         bsp_status_t / 事件 / 错误记录
    │   ├── bsp_can/            CAN 硬件无关抽象 + 帧分发器
    │   ├── bsp_log/            BSP_LOG_INFO/WARNING/ERROR 宏
    │   ├── bsp_dwt/            DWT 周期计数（当前未接线）
    │   └── bsp_watchdog/       看门狗抽象（当前接 NULL）
    │
    └── Components/
        ├── Algorithm/
        │   ├── alg_crc/        裁判系统 CRC8/CRC16
        │   ├── alg_kalman/     矩阵卡尔曼（QuaternionEKF 的底层）
        │   ├── alg_lpf/        二阶低通滤波器
        │   ├── alg_quaternion/ 四元数姿态解算 EKF
        │   ├── alg_math/       标量/向量/矩阵/四元数/统计工具
        │   ├── alg_pid/        PID 核心 + 角度环 + 串级
        │   ├── alg_mecanum/    麦轮正逆运动学
        │   └── alg_chassis/    底盘运动学封装 + 轮速监控
        └── module/
            ├── bmi088/         BMI088 六轴 IMU 驱动 + 寄存器表
            ├── ix6_remote/     I6X SBUS 遥控器解析
            ├── image_transmission/ 图传链路（自定义控制器/键鼠）
            ├── referee_system/ 裁判系统协议解析
            ├── module_motor/   电机基类（虚表 + 反馈 + PID 配置）
            ├── module_dji_motor/   DJI M2006/M3508/GM6020 CAN 驱动
            ├── module_dm_motor/    DM J4310 CAN 驱动 + 多机总线
            └── module_shooter/ 发射机构状态机（送弹/堵转/回退）
```

---

## 3. 启动时序

```
Reset
 └─ main()
     ├─ HAL_Init() / SystemClock_Config() / MPU_Config() / Cache 使能
     ├─ MX_*_Init()                 CubeMX 外设初始化（FDCAN1/2/3、UART5、SPI2、TIM3、ADC…）
     ├─ MCU_Init()                  [bsp_mcu.c] 板级初始化总入口
     │    ├─ BSP_PWM_Init()         加热 PWM
     │    ├─ BSP_GPIO_Init()        引脚
     │    ├─ BSP_FDCAN_Init()       [bsp_fdcan] 三路 FDCAN 滤波器 + Start
     │    ├─ BSP_USART_Init()       [bsp_uart] USART1 DMA + UART5 SBUS + UART7 打印
     │    ├─ BSP_ADC_Init()
     │    ├─ MX_USB_DEVICE_Init()
     │    └─ BMI088_Init()          [bmi088] IMU 自检
     ├─ robot_init()                [robot_config.c] 整车对象构建（见 5.1）
     └─ MX_FREERTOS_Init()          [freertos.c] 创建 4 个任务
          └─ osKernelStart()
```

---

## 4. 各层文件职能

### 4.1 BSP 层

| 文件 | 职能 | 被谁调用 |
|---|---|---|
| `bsp_mcu/bsp_mcu.c` | `MCU_Init()`：把板级外设按依赖顺序初始化一次 | `main.c` |
| `bsp_gpio/bsp_gpio.c` | GPIO 初始化 + 电池/开关量引脚读取 | `bsp_mcu`、`Bmi088` |
| `bsp_pwm/bsp_pwm.c` | 启动 TIM3 PWM，设置 BMI088 加热占空比 | `bsp_mcu`、`INS_Task` |
| `bsp_spi/bsp_spi.c` | `BMI088_Read_Write_Byte()` SPI 单字节收发 | `Bmi088.c` |
| `bsp_tick/bsp_tick.c` | `Delay_us` / `Delay_ms` 忙等延时 | `Bmi088.c` |
| `bsp_adc/bsp_adc.c` | ADC 校准与启动 | `bsp_mcu` |
| `bsp_uart/bsp_uart.c` | **三条串口链路**：USART1 DMA 双缓冲（裁判系统 / 图传二选一，由 `USART1_RX_Switch` 切换）、UART5 逐字节收 SBUS 帧、UART7 调试打印 | `bsp_mcu` |
| `bsp_fdcan/cod-fdcan.c` | **三路 FDCAN 的 HAL 落地**：`BSP_FDCAN_Init()` 配滤波器并 Start；`HAL_FDCAN_RxFifo0/1Callback` 只置 `FDCANx_RxPending` 标志，不在中断取帧 | `bsp_mcu`、`CAN_Task`、`robot_*` |
| `bsp_can/bsp_can.c` | 硬件无关 CAN 接口：`bsp_can_t` 对象 + `bsp_can_driver_ops_t` 操作集 | `robot_config.c` |
| `bsp_can/bsp_can_dispatcher.c` | 帧分发器：按 `identifier/mask` 查路由表回调，在任务上下文消费 `RxPending` | `robot_task.c`、`robot_config.c` |
| `bsp_common/bsp_common.c` | `bsp_status_t` 状态码、事件类型、错误记录 | 全 BSP |
| `bsp_log/bsp_log.c` | `BSP_LOG_INFO/WARNING/ERROR` 宏实现，`BSP_LOG_DISABLED=1` 可整体剔除 | `app_safety.c` |
| `bsp_dwt/bsp_dwt.c` | DWT 周期计数换算（`bsp_dwt_*`） | **当前无调用者**，预留 |
| `bsp_watchdog/bsp_watchdog.c` | 看门狗抽象（init/refresh/get_reset_detected） | `app_safety.c`（`watchdog` 传 NULL，暂不生效） |

**为什么 `bsp_fdcan` 和 `bsp_can` 都要存在**

二者是**同一协议栈的上下两层**，不是重复封装：

```
cod-fdcan.c   → HAL 驱动落地（滤波器配置、Start、中断只置标志）
      ↓ robot_port.c 把 HAL 实现成 driver_ops
bsp_can.c     → 硬件无关对象接口（transmit / receive）
      ↓
bsp_can_dispatcher.c → 按 CAN ID 路由到业务回调
      ↓
module_dji_motor / module_dm_motor → 电机协议解析
```

删掉 `cod-fdcan.c` 会失去 `BSP_FDCAN_Init()`（`MCU_Init()` 依赖）和 HAL 弱回调实现（否则链接失败）。

### 4.2 Components / Algorithm

| 文件 | 职能 | 关键调用者 |
|---|---|---|
| `alg_quaternion/Quaternion.c` | `QuaternionEKF_Update()` 四元数姿态融合 | `INS_Task` |
| `alg_kalman/Kalman_Filter.c` | 通用矩阵卡尔曼滤波 | `Quaternion.c` |
| `alg_lpf/LPF.c` | `LowPassFilter2p_Update()` 二阶低通 | `INS_Task` |
| `alg_pid/alg_pid_core.c` | PID 核心（抗积分饱和、微分滤波） | `INS_Task` 温控 |
| `alg_pid/alg_pid_angle.c` | 角度环（自动跨 ±π 最短路径） | 预留 |
| `alg_pid/alg_pid_cascade.c` | 串级 PID（位置→速度→电流） | 预留 |
| `alg_math/alg_math_*.c` | 标量/向量/矩阵/四元数/统计工具 | `app_chassis` |
| `alg_mecanum/alg_mecanum.c` | 麦轮逆运动学：底盘三轴速度 → 四轮转速；正运动学做里程计 | `robot_config`、`app_chassis` |
| `alg_chassis/alg_chassis_motion.c` | 底盘运动学封装（速度限幅、坐标系变换） | `app_chassis` |
| `alg_chassis/alg_chassis_wheel_monitor.c` | 轮速异常监控 | `app_chassis` |
| `alg_crc/CRC.c` | CRC8 / CRC16 | `Referee_System.c`、`Image_Transmission.c` |

### 4.3 Components / module

| 文件 | 职能 | 关键调用者 |
|---|---|---|
| `module_motor/module_motor.c` | **电机基类**：状态机（DISABLED/ENABLED/FAULT）、反馈超时、虚表分发 | `module_dji_motor`、`module_dm_motor` 向上转型 |
| `module_dji_motor/module_dji_motor.c` | M3508/M2006/GM6020：反馈帧解析、按发送组（0x1FF/0x200…）打包 4 路电流、位置/速度/角度三环 | `robot_config` |
| `module_dm_motor/module_dm_motor.c` | DM J4310：MIT / 力位混合模式、使能/失能/参数读写 | `robot_config` |
| `module_dm_motor/module_dm_motor_bus.c` | DM 多机总线：主 ID↔反馈 ID 映射、时隙发送 | `robot_config`、`robot_task` |
| `module_shooter/module_shooter.c` | 发射状态机：待发队列、送弹、堵转检测、回退重试 | `robot_task`、`robot_adapter` |
| `bmi088/Bmi088.c` | BMI088 初始化、自检、加速度/陀螺仪读取、温控 | `INS_Task`、`bsp_mcu` |
| `ix6_remote/Ix6_Remote.c` | `sbus_to_i6x()`：25 字节 SBUS 帧 → 通道值/拨杆/失控标志 | `bsp_uart.c` |
| `referee_system/Referee_System.c` | 裁判系统 0x0001~0x0308 各 cmd_id 解析 | `bsp_uart.c`（`USART1_RX_Switch=0`） |
| `image_transmission/Image_Transmission.c` | 图传自定义控制器 / 键鼠数据解析 | `bsp_uart.c`（`USART1_RX_Switch=1`） |

### 4.4 Application / App

| 文件 | 职能 |
|---|---|
| `app_types.h` | 跨模块共享：`app_remote_input_t`、`app_chassis_command_t`、`app_shooter_command_t`、`app_switch_t` 等 |
| `app_chassis/app_chassis.c` | `app_chassis_update()`：命令 → 麦轮逆解 → 四轮角速度目标 |
| `app_safety/app_safety.c` | 监控器注册与超时判定；任一必需监控器掉线立即置 `output_allowed=false`，还可周期喂看门狗 |
| `app_shooter/app_shooter.c` | 拨弹单发/连发命令 → `module_shooter` |

### 4.5 Application / Task

| 任务 | 优先级 | 周期 | 职责 |
|---|---|---|---|
| `INS_Task` | High | 1 ms | BMI088 采样 → 低通 → 四元数 EKF → 发布 `INS_Info`；BMI088 加热温控 |
| `Control_Task` | AboveNormal | 1 ms | 计算 `dt` → `robot_task_control(dt)`（唯一控制入口） |
| `CAN_Task` | Normal | 1 ms | FDCAN2 Bus-Off 恢复 → `robot_task_can_receive()` |
| `Detect_Task` | BelowNormal | 1 ms | 当前为空循环（历史遗留，清理时保留任务创建） |

`Config.h`：物理常量（重力、π 换算、弹道系数、装甲板尺寸），仅 `INS_Task` 使用。

### 4.6 ROBOT 适配层

| 文件 | 职能 |
|---|---|
| `robot_config.c/.h` | 全局 `robot_t robot`；所有 CAN ID、PID 增益、麦轮尺寸、DM 限制集中在此；`robot_init()` 完成 12 步构建；`robot_lift_update()` 抬升控制；`robot_try_enable_motors()` 反馈在线后自动使能；`robot_enter_safe_state()` 安全态 |
| `robot_adapter.c` | 读 `i6x_ctrl` → 死区/归一化 → 填 `robot.chassis_cmd`；右拨杆上升沿触发单发；100 ms 无新帧判失联 |
| `robot_port.c` | 把 `HAL_FDCAN_*` 包装成 `bsp_can_driver_ops_t`（transmit / receive / configure_filter…） |
| `robot_task.c` | `robot_task_can_receive()` 跑两个分发器；`robot_task_control()` 按「适配器 → 安全门控 → 底盘 → 抬升 → 拨弹 → 使能 → 总线刷新」顺序执行 |

---

## 5. 关键协作关系

### 5.1 `robot_init()` 构建顺序（robot_config.c）

```
① bsp_can_init(can_dji→hfdcan1, can_dm→hfdcan2)      CAN 对象
② module_dji_motor_bus_init / module_dm_motor_bus_init  电机总线
③ 4×M3508  底盘   ID 1~4   发送组 0x200  速度模式
④ 2×M3508  抬升   ID 5~6   发送组 0x1FF  角度模式
⑤ 2×J4310  抬升   主 0x01/0x02  反馈 0x11/0x12
⑥ 1×J4310  拨弹   主 0x03       反馈 0x13
⑦ alg_mecanum_init   麦轮运动学
⑧ app_chassis_init   底盘 App
⑨ module_shooter_init 拨弹状态机（本板仅拨弹电机，无摩擦轮）
⑩ bsp_can_dispatcher_init ×2 + 注册 6 条 DJI 路由（0x201~0x206）+ 3 条 DM 路由（0x11~0x13）
⑪ app_safety_init + rc_monitor（100 ms 超时，required）+ 失联回调
⑫ 命令默认值：全部 disabled / NO_FORCE
```

### 5.2 CAN 收帧链路

```
FDCAN 硬件
  └─ HAL_FDCAN_RxFifo0Callback / RxFifo1Callback   [cod-fdcan.c]
       └─ 只置 FDCAN1_RxPending / FDCAN2_RxPending（中断上下文，不取帧）
            └─ CAN_Task（1 ms 轮询）
                 └─ robot_task_can_receive()            [robot_task.c]
                      ├─ bsp_can_dispatcher_process(dji_dispatcher)
                      │    └─ bsp_can_receive → robot_port.receive → HAL_FDCAN_GetRxMessage
                      │    └─ 路由匹配 0x201~0x206
                      │    └─ dji_feedback_callback → module_dji_motor_bus_handle_feedback
                      ├─ bsp_can_dispatcher_process(dm_dispatcher)  → 0x11~0x13
                      └─ 清 RxPending
```

### 5.3 控制链路（1 kHz）

```
Control_Task  [Control_Task.c]
  └─ robot_task_control(dt)   [robot_task.c]
       ├─ robot_adapter_update()      遥控 → chassis_cmd / shooter 单发请求
       ├─ app_safety_process(now_ms)  各监控器超时判定
       ├─ app_safety_set_output_enabled(remote.online)
       ├─ output_allowed == false → 强制 NO_FORCE
       ├─ app_chassis_update(&chassis, &chassis_cmd, dt)   麦轮逆解 → 四轮目标
       ├─ robot_lift_update(dt)       抬升目标角 → 2×M3508 角度环 + 2×J4310 位速
       ├─ module_shooter_update(dt)   拨弹状态机
       ├─ robot_try_enable_motors()   反馈在线则自动使能
       ├─ module_dji_motor_bus_update(dt)  打包 0x200 / 0x1FF 下发
       └─ module_dm_motor_bus_update(dt)   J4310 逐台下发
```

### 5.4 遥控链路

```
I6X 接收机 ──SBUS── UART5
  └─ HAL_UART_RxCpltCallback（逐字节）
       └─ ProcessSbusByte()            [bsp_uart.c]
            ├─ 帧头 0x0F 对齐
            ├─ 收满 I6X_FRAME_LENGTH(25) 字节
            └─ sbus_to_i6x(&i6x_ctrl, frame)   [Ix6_Remote.c]
                 └─ 更新 ch[0..3] / s[0..1] / failsafe / frame_cnt
                      └─ robot_adapter_update()  读 get_i6x_point()
                           ├─ frame_cnt 变化 → 刷新存活时间
                           ├─ 100 ms 无变化 → online=false → 底盘 NO_FORCE
                           ├─ ch 归一化 → velocity_x/y + angular
                           └─ 右拨杆上升沿 → module_shooter_request_shots(1)
```

### 5.5 姿态链路

```
INS_Task (1 ms)
  ├─ BMI088_Info_Update()                     [Bmi088.c]
  ├─ LowPassFilter2p_Update() ×3              [LPF.c] 加速度低通
  ├─ QuaternionEKF_Update()                   [Quaternion.c] → Kalman_Filter.c
  ├─ INS_Info 更新（欧拉角 / 角速度）
  └─ BMI088_Temp_Control() → alg_pid → Heat_Power_Control()  加热闭环
```

### 5.6 串口资源分配

| 外设 | 用途 | 工作方式 |
|---|---|---|
| USART1 | 裁判系统 **或** 图传（编译期 `USART1_RX_Switch` 二选一） | DMA 双缓冲 + IDLE 中断 |
| UART5 | I6X SBUS 遥控 | 逐字节 `HAL_UART_Receive_IT` |
| UART7 | 调试打印 | 阻塞发送 |

> UART5 已关闭 DMA（`CLEAR_BIT(USART_CR3_DMAR)`），因此不存在 DMA 与中断双路径竞争，SBUS 只有 `ProcessSbusByte` 一个解析入口。

---

## 6. CAN ID 映射表

| 总线 | 设备 | 数量 | 反馈 ID | 控制发送 ID |
|---|---|---|---|---|
| FDCAN1 | 麦轮 M3508 | 4 | 0x201–0x204 | 0x200（4 路电流打包） |
| FDCAN1 | 抬升 M3508 | 2 | 0x205–0x206 | 0x1FF（4 组中 2 路有效） |
| FDCAN2 | 抬升 J4310 | 2 | 0x11 / 0x12 | 主 ID 0x01 / 0x02 |
| FDCAN2 | 拨弹 J4310 | 1 | 0x13 | 主 ID 0x03 |
| FDCAN3 | — | — | 已初始化，未挂业务 | — |

规模宏集中在 `robot_config.h`：`ROBOT_WHEEL_COUNT` / `ROBOT_LIFT_DJI_COUNT` / `ROBOT_LIFT_DM_COUNT` / `ROBOT_DM_MOTOR_COUNT` / `ROBOT_DJI_ROUTE_COUNT` / `ROBOT_DM_ROUTE_COUNT`。改机型只需改这组宏 + `robot_init()` 里的对应段落。

---

## 7. 编译与烧录

```powershell
cmake --preset Debug                 # 配置（首次或增删文件后）
cmake --build build/Debug            # 编译
```

- 工具链：`cmake/gcc-arm-none-eabi.cmake`，`arm-none-eabi-gcc 14.3.1`
- 源文件收集：`CMakeLists.txt` 用 `file(GLOB_RECURSE ... CONFIGURE_DEPENDS)` 递归扫描 `USERS/Application`、`USERS/BSP`、`USERS/Components`，新增 `.c` 无需改 CMake，但**移动文件后必须重新 `--preset`**
- 产物：`build/Debug/Z_HEAVY_Robot_CHASIS.elf` / `.map`
- 当前占用：DTCMRAM 98872 B (75.43%)、FLASH 156096 B (14.89%)

---

## 8. 目录整理记录

本轮整理把旧 COD 的「`Inc/` + `Src/` 两级分离」统一为「一个模块一个自包含目录」，删除 6 个空目录、移动 33 个文件：

| 原路径 | 新路径 |
|---|---|
| `USERS/BSP/Inc+Src/bsp_{adc,gpio,mcu,pwm,spi,tick,uart}` | `USERS/BSP/bsp_{adc,gpio,mcu,pwm,spi,tick,uart}/` |
| `USERS/BSP/Inc+Src/cod-fdcan.*` | `USERS/BSP/bsp_fdcan/` |
| `Algorithm/Inc+Src/{CRC,Kalman_Filter,LPF,Quaternion}` | `Algorithm/alg_{crc,kalman,lpf,quaternion}/` |
| `module/Inc+Src/{Bmi088,Bmi088_Reg,Image_Transmission,Ix6_Remote,Referee_System}` | `module/{bmi088,image_transmission,ix6_remote,referee_system}/` |

`CMakeLists.txt` 的 include 路径同步替换，`cmake --build` 113/113 通过，FLASH 占用与整理前完全一致（156096 B），说明整理未引入任何代码变化。

前一轮已删除的重复/死代码：旧 `bsp_dwt.c/h`、`Remote_Control.c/h`（与 `Ix6_Remote` 重复解析同一 SBUS 帧）、`MiniPC.c/h`、`Ramp.c/h`、`RLS.c/h`。

**保留但当前未接线**（编译通过、被 `--gc-sections` 丢弃、不占 FLASH）：

- `bsp_dwt/` —— BSP 层计时 API，待控制环需要高精度 dt 时接入
- `App` 层部分 `alg_math` / `alg_pid_angle` / `alg_pid_cascade` —— 库 API 预留
- `bsp_watchdog/` —— `app_safety_init(&robot.safety, NULL)` 传 NULL，需实现 IWDG 后端并传入实例才生效
- `app_shooter.c` —— 拨弹直接由 `robot_task` 调 `module_shooter_update`，未走 `app_shooter` 封装
- `Detect_Task` —— 空循环，任务仍创建（保留一份栈）

---

## 9. 实车标定清单

工程里所有需要按实物修改的参数都集中在这几个位置，改完只需重新编译，**不需要动任何逻辑代码**。

### 9.0 标定顺序（务必按序）

```
① 电机 ID 与接线核对 → ② 电机方向 → ③ 麦轮几何尺寸 → ④ 电机 PID
→ ⑤ DM 限制参数 → ⑥ 遥控通道与方向 → ⑦ IMU 与温控 → ⑧ 安全阈值
→ ⑨ 分阶段验车
```

每一步都单独验证，不要跳步。第 ④ 步之前所有电机必须能在**离地状态**正常正反转。

### 9.1 电机 ID 与接线核对

| 检查项 | 应为何值 | 位置 |
|---|---|---|
| 4 个麦轮 M3508 | ID 1 / 2 / 3 / 4，全部接 FDCAN1 | `robot_config.c` ③ 段 |
| 2 个抬升 M3508 | ID 5 / 6，接 FDCAN1 | `robot_config.c` ④ 段 |
| 2 个抬升 J4310 | 主 ID 0x01 / 0x02，反馈 0x11 / 0x12，接 FDCAN2 | `robot_config.c` ⑤ 段 |
| 1 个拨弹 J4310 | 主 ID 0x03，反馈 0x13，接 FDCAN2 | `robot_config.c` ⑥ 段 |

> 若改动力总成数量，必须同步改 [robot_config.h](file:///d:/STM32work/trae/COD-H7-Template-main/ROBOT/robot_config.h#L27-L32) 的 `ROBOT_*_COUNT` 宏，否则数组越界或路由不足。
> J4310 的「主 ID」是收命令的 ID，「反馈 ID」是它回传用的 ID，两者必须在达妙上位机里配好且与代码一致。

**验证方法**：给单台电机发 0 电流以外的极小目标，看是否只有目标电机响应。若两台同时动，说明 ID 冲突。

### 9.2 电机安装方向

| 参数 | 当前值 | 说明 |
|---|---|---|
| `k_lift_dji_direction[2]` | `{1.0f, -1.0f}` | 两台抬升 M3508 的镜像安装补偿 |
| `k_lift_dm_direction[2]` | `{1.0f, -1.0f}` | 两台抬升 J4310 的镜像安装补偿 |
| `mecanum_config.direction_sign[4]` | `{1,-1,1,-1}` | 四轮安装方向 |
| `make_dji_config().direction_sign` | `1.0f` | 单台电机方向，默认全正 |

**验证方法**：给抬升一个正目标角，两台电机必须**同向**推动机构；若一台反了，把对应 `direction_*` 取反。

### 9.3 麦轮几何尺寸（必须实测）

| 参数 | 当前默认 | 含义 | 实测方法 |
|---|---|---|---|
| `wheel_radius_m` | `0.076f` | 轮半径（含辊子压缩后的有效值） | 卷尺量直径 ÷ 2；更准的做法是推车 1 m 数编码器圈数反推 |
| `half_wheelbase_m` | `0.19f` | 前后轴距的一半 | 前后轮心距 ÷ 2 |
| `half_track_width_m` | `0.21f` | 左右轮距的一半 | 左右轮心距 ÷ 2 |
| `maximum_wheel_angular_velocity_rad_per_s` | `60.0f` | 单轮限速 | 按电机额定转速与减速比核算 |
| `roller_arrangement` | `ALG_MECANUM_ROLLER_X` | 辊子排布方式（X 型 / O 型） | 俯视看辊子轴线是否呈 X |

参数在 `robot_config.c` ⑦ 段 `mecanum_config`。

**验证方法**：摇杆推正前方，车应直行不偏转；推正左方，车应纯横移。若前进变斜行、或横移带自转，先检查 `roller_arrangement`，再校尺寸。

> 尺寸偏差最典型的症状：车能走但**里程计漂移**严重、或四轮转速差异明显导致打滑。

### 9.4 PID 整定顺序

三组参数都在 `robot_config.c` 顶部 `k_*_pid` 常量。**整定顺序为 内环 → 外环**：

| 顺序 | 参数 | 当前默认 | 整定目标 |
|---|---|---|---|
| ① | `k_wheel_velocity_pid`（速度环，输出电流） | Kp 2.0 / Ki 0.05 / Kd 0 | 阶跃响应无超调、无静差 |
| ② | `k_motor_current_pid`（电流环） | Kp 1.0 / Ki 0 / Kd 0 | 电流跟踪快、无振荡 |
| ③ | `k_lift_angle_pid`（角度环，输出速度） | Kp 20 / Ki 0 / Kd 0 | 到位无过冲、无持续抖动 |

整定要点：

1. **先只调 Kp**，加到刚开始等幅振荡，再回退到 60~70%
2. **再加 Ki** 消除静差，注意 `integral_min/max` 已按输出量程配好，不要随意放大
3. **Kd 建议保持 0**：CAN 反馈有量化噪声，微分项容易放大抖动；确需加时务必开 `derivative_filter_cutoff_hz`
4. `output_min/max = ±16384` 是 M3508 电流命令满量程，不要突破

**验证方法**：把车架空，发阶跃速度目标，用 `UART7` 打印实测转速看响应曲线。落地后复测一次，负载会改变实际增益。

### 9.5 DM J4310 限制参数（**必须与电机固件一致**）

`k_dm4310_limits` 在 `robot_config.c` ⑤ 段前方：

| 参数 | 当前默认 | 来源 |
|---|---|---|
| `position_min/max_rad` | ±12.5 | 电机固件 PMAX 寄存器 |
| `velocity_min/max_rad_per_s` | ±30.0 | VMAX |
| `torque_min/max_nm` | ±10.0 | TMAX |
| `proportional_gain_min/max` | 0 ~ 500 | KP 范围 |
| `derivative_gain_min/max` | 0 ~ 5 | KD 范围 |

> **这组值填错不会编译报错，但会让 J4310 收到的命令被限幅成错误值甚至进入保护**。必须先在达妙调试助手里读出每台电机的 PMAX/VMAX/TMAX，或统一烧成同一套参数，再同步到代码。

同时确认 `make_dm_config()` 里 `transmit_timeout_ms = 2U`：J4310 若 2 ms 内收不到下一帧命令会自动失能。控制周期 1 ms 能满足，但如果 `Control_Task` 被高优先级任务拖慢，就会掉使能——这是排查「DM 电机莫名其妙掉力」的第一嫌疑点。

### 9.6 遥控通道映射

宏定义在 [robot_adapter.c](file:///d:/STM32work/trae/COD-H7-Template-main/ROBOT/robot_adapter.c#L12-L16)：

| 宏 | 当前值 | 调整场景 |
|---|---|---|
| `ROBOT_RC_DEADBAND` | `30` | 摇杆回中有漂移就加大 |
| `ROBOT_RC_CHANNEL_SCALE` | `660.0f` | I6X 满量程约 ±660，换遥控器要重测 |
| `ROBOT_MAX_LINEAR_MPS` | `2.5f` | 底盘实际最高速度 |
| `ROBOT_MAX_ANGULAR_RPS` | `6.0f` | 底盘实际最高角速度 |
| `ROBOT_RC_TIMEOUT_MS` | `100U` | 失联判定；与 `rc_monitor` 的 `timeout_ms` 保持一致 |

通道映射在 `robot_adapter_update()` 第 96~98 行：

```c
velocity_x = -normalize_channel(rc->ch[2]) * ROBOT_MAX_LINEAR_MPS;   // 前后
velocity_y = -normalize_channel(rc->ch[0]) * ROBOT_MAX_LINEAR_MPS;   // 左右
angular    = -normalize_channel(rc->ch[3]) * ROBOT_MAX_ANGULAR_RPS;  // 旋转
```

**验证方法**：推摇杆前进，若车后退，把 `ch[2]` 前的负号去掉；左右/旋转同理。**每次只改一个符号**。

拨弹触发在 `robot_adapter.c` 第 104~108 行：右拨杆 `UP` 的**上升沿**发 1 发。若想改连发，改这里调 `module_shooter_request_shots` 的数量或改 `automatic_fire_enabled`。

### 9.7 IMU 与温控

| 参数 | 当前值 | 位置 | 说明 |
|---|---|---|---|
| `INS_LPF2p_Alpha[3]` | `1.9294 / -0.9318 / 0.00233` | `INS_Task.c` | 加速度二阶低通系数，对应截止频率约 30 Hz。震动大时降低截止频率 |
| `QuaternionEKF_Init` 的 `Process_Noise1` | `10.f` | `INS_Task.c:179` | 过程噪声 |
| `Process_Noise2` | `0.001f` | 同上 | 过程噪声 |
| `Measure_Noise` | `1000000.f` | 同上 | 测量噪声。**偏大**，若姿态收敛慢可试着降到 1000~10000 |
| 温控目标 | `40.f` °C | `INS_Task.c:192` | BMI088 恒温点 |
| 温控 PID | Kp 1200 / Ki 20 / 输出 0~2000 | `INS_Task.c` `TempCtrl_PID_Config` | 加热 PWM 占空比 |
| `maximum_temperature_c` | `80.0f` | `robot_config.c` `make_dji_config` | 电机过温阈值，触发 FAULT |

**BMI088 静态标定**：整车上电静置 30 s，读陀螺仪零偏（`BMI088_Info.Gyro`），若静止时零偏超过 ±0.01 rad/s，需在 `Bmi088.c` 初始化里写入 `GYRO_OFFSET`。加速度计零偏会影响上电初始 Pitch/Roll，静置时应接近 0（水平放置）。

### 9.8 安全阈值

| 参数 | 当前值 | 位置 | 说明 |
|---|---|---|---|
| `rc_monitor.timeout_ms` | `100U` | `robot_config.c` ⑪ 段 | 遥控失联判定 |
| `rc_monitor.required` | `true` | 同上 | 必需监控器，掉线直接关输出 |
| `app_shooter` 堵转电流 | `jam_current_threshold_a = 3.0f` | `robot_config.c` ⑨ 段 | 拨弹卡弹电流阈值 |
| 堵转确认时间 | `jam_confirmation_time_s = 0.3f` | 同上 | 持续多久判定堵转 |
| 最大重试次数 | `maximum_jam_retries = 3U` | 同上 | 回退重试上限 |

`app_safety_set_output_enabled(&robot.safety, robot.remote.online)` 是总开关：遥控一掉线，底盘强制 `NO_FORCE`、`enabled = false`，这是**唯一**的正常停车路径，不要绕过它做「屏蔽遥控」的调试，否则会失去失控保护。

### 9.9 分阶段验车

| 阶段 | 操作 | 通过标准 |
|---|---|---|
| 1. 静态 | 上电，不接电机动力线 | `UART7` 打印 `UART7 READY`；无 HardFault |
| 2. 通信 | 接电机信号线，车架空 | `CAN_Task` 收到反馈，`module_motor` 反馈 `is_online = true` |
| 3. 单机 | 遥控轻推，逐台确认转向 | 每个电机单独响应，方向正确 |
| 4. 底盘 | 落地，低速推摇杆 | 前进/横移/旋转均正确，无自转、无跑偏 |
| 5. 抬升 | 下发小角度目标 | 两台同步、到位不抖、不撞限位 |
| 6. 拨弹 | 右拨杆上拨 | 每次上拨发 1 发，卡弹能回退重试 |
| 7. 失联 | 关遥控器 | 100 ms 内全部电机失能，底盘停转 |

阶段 3 之后每一步都建议把车架空先测一遍，再落地复测。
