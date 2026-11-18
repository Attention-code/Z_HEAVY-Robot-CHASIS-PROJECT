# 调试日志（RTT）与实时波形使用说明

本工程调试输出走 **SEGGER RTT**：代码把数据写进目标 RAM 的环形缓冲区，电脑通过 **OpenOCD + DAPLink** 走 SWD 读取，不占用 UART，也不需要 SWO 引脚。

- **文本日志**：RTT 通道 0，在 VS Code 终端 / TCP 端口查看（第一~四章）
- **实时波形**：RTT 通道 1/2，由 cortex-debug 的 `graph` 解码器解析为 float 并绘制曲线（第五章）

已验证环境：OpenOCD 0.12.0、cortex-debug 1.12.1；`_SEGGER_RTT` 符号位于 DTCM（无 cache 一致性问题）。

## 一、硬件连接

DAPLink 一端 USB 接电脑，另一端 SWD 排线接板子：

| DAPLink | 板子 |
|---|---|
| 3V3 | 3V3（仅参考电平，不要靠它给整板供电） |
| GND | GND |
| SWDIO | SWDIO / DIO |
| SWCLK | SWCLK / CLK |
| NRST（可选） | NRST |

板子单独供电。先接线后上电，断电顺序相反。

## 二、日常调试（推荐：VS Code）

1. 接好 DAPLink，板子上电。
2. 按 `F5`，选择配置 `OpenOCD: Debug`：自动编译（`preLaunchTask`）→ 烧录 → 停在 `main`。
3. **再按一次 `F5`** 全速运行，RTT Console（Channel 0）开始实时输出日志；同时自动弹出 **"RTT 波形"** 图表面板（配置见第五章）。
4. 调试结束按 `Shift+F5`，不要直接拔线。

说明：
- 不需要先单独烧录，F5 会自己编译并烧录；用 `Ctrl+Shift+B` 再烧一遍也无妨。
- 停在 `main` 时没有日志属正常，必须继续运行。
- `rttConfig` 中 `address: "auto"`，会自动从 ELF 的 `_SEGGER_RTT` 符号取地址，重新编译后地址变了也不用改配置。
- DAPLink 只有一个，同一时刻只能有一个 OpenOCD 占用它（VS Code 调试与命令行方式不能同时开）。

## 三、只看日志、不要断点（命令行）

固件已烧录、板子已运行的前提下，开两个 PowerShell 窗口：

```powershell
# 窗口 1：挂 RTT
openocd -f openocd.cfg -c "init; rtt setup 0x20000000 0x20000; rtt start; rtt server start 9090 0"
```

```powershell
# 窗口 2：接收日志
ncat localhost 9090
```

说明：
- `rtt setup 0x20000000 0x20000` 会在 DTCM 范围内搜索 `SEGGER RTT` 控制块，不用每次重新查地址。
- 也可以按固定地址：`arm-none-eabi-nm build/Debug/COD_H7_Template.elf | Select-String _SEGGER_RTT`，再用 `rtt setup <地址> 0x100`。
- 若窗口 2 无输出且固件没跑，在 `init` 后加上 `reset run`。
- 接收端也可用 PuTTY：`Raw` + `localhost` + `9090`。
- 退出：OpenOCD 窗口按 `Ctrl+C`。

## 四、代码里怎么打日志

包含头文件，直接用宏（定义见 `bsp_log.h`）：

```c
#include "bsp_log.h"

BSP_LOG_INFO("frame_cnt = %lu\r\n", (unsigned long)rc->frame_cnt);
BSP_LOG_WARNING("rc timeout\r\n");
BSP_LOG_ERROR("can tx failed, err=%d\r\n", err);
```

| 宏 | 前缀 | 用途 |
|---|---|---|
| `BSP_LOG_INFO` | `[I] ` | 普通状态、变量值 |
| `BSP_LOG_WARNING` | `[W] ` | 异常但系统仍可运行 |
| `BSP_LOG_ERROR` | `[E] ` | 明确错误 |

- 三个宏都会自动换行；`bsp_log_printf()` 为不带级别、不自动换行的裸版本。
- 初始化在 `bsp_mcu.c` 的 `MCU_Init()` 第一行 `bsp_log_init()`，全工程只需一次。
- 单条日志上限 `BSP_LOG_BUFFER_SIZE = 128` 字节，超长会截断。
- 不要在 CAN / 控制中断里高频打印，调试信息放在任务中输出。
- 编译期定义 `BSP_LOG_DISABLED=1` 可剔除全部日志调用。
- `%f` 依赖 printf 库的浮点支持，保险起见用 `%d` 打印放大后的整数。

## 五、实时波形查看（RTT Graph）

### 5.1 原理

RTT 的一个控制块里可以有多个独立的环形缓冲（通道）。本工程把通道分工如下：

| 通道 | 内容 | 格式 | 查看方式 |
|---|---|---|---|
| 0 | 文本日志 | ASCII 文本 | VS Code 终端 / TCP 9090 |
| 1 | 波形曲线 1（Plot0） | 4 字节小端 float | cortex-debug 图表面板 |
| 2 | 波形曲线 2（Plot1） | 4 字节小端 float | cortex-debug 图表面板 |

**波形为什么单独用通道**：`graph` 解码器要求通道里是**纯二进制 float 流**——每 4 字节算一个采样点。如果和文本混在一个通道，解码器会把 ASCII 字符也当成 float 解析，曲线立刻变成乱码。

完整数据通路（以通道 1 为例）：

```text
robot_task_control()（1kHz 调度，robot_task.c）
   │  每 10ms 节流一次（100Hz）
   ▼
bsp_log_plot(0, x)                 bsp_log_plot(1, y)
   │  4 字节 float（小端）            │
   ▼                                ▼
RTT Up Buffer 1 "Plot0"           Up Buffer 2 "Plot1"
   （512B 环形缓冲，位于 DTCM 的 _SEGGER_RTT 控制块内）
   └────────────────┬───────────────┘
                    │  OpenOCD 每 10ms 经 SWD 读一次 RAM
                    │  （launch.json 中 polling_interval: 10）
                    ▼
   cortex-debug rttConfig：graph 解码器（encoding: "float"）
                    ▼
   "底盘左前轮 目标/反馈轮速" 实时面板（样式由 graphConfig 定义）
```

关键点：

- **一个通道 = 一条曲线**。同一通道里连续发送的多个 float 只是同一曲线上的连续点；想要两条曲线就各用各的通道。
- 读的是 RAM 而非外设，所以**不断点、不停核**即可持续采样，比 SWO/UART 侵入性更小；代价是依赖调试器连接，拔掉 DAPLink 就没有波形。
- CPU 停在断点时 OpenOCD 读不到新数据，波形会"冻结"，继续运行后恢复。

### 5.2 主机端配置（launch.json）

配置位于 `.vscode/launch.json` 的 `OpenOCD: Debug` 配置项，分两块：

**① `rttConfig`：声明"从哪个通道读、怎么解析"**

```jsonc
"rttConfig": {
    "enabled": true,
    "address": "auto",          // 从 ELF 的 _SEGGER_RTT 符号自动找控制块
    "polling_interval": 10,     // OpenOCD 轮询周期（ms），越小越顺滑、总线越忙
    "decoders": [
        { "port": 0, "type": "console", "label": "RTT Channel 0" },   // 文本日志
        {
            "port": 1,              // RTT 通道 1
            "type": "graph",        // 图形解码器：数据不进终端，直接进图
            "encoding": "float",    // 按 4 字节小端 float 解析
            "graphId": "plot0",     // 曲线 ID，供 graphConfig 引用
            "label": "Plot 0"
        },
        { "port": 2, "type": "graph", "encoding": "float", "graphId": "plot1", "label": "Plot 1" }
    ]
}
```

| 字段 | 说明 |
|---|---|
| `port` | RTT 通道号，与固件侧 `bsp_log_plot()` 的通道对应 |
| `type` | `graph` 为绘图解码器；另有 `console`（文本）、`binary`（十六进制终端）、`advanced`（自定义 JS 解码器） |
| `encoding` | 解析格式：`float` / `signed` / `unsigned` / `Q16.16`，默认 `unsigned`，本工程用 `float` |
| `graphId` | 任意自定义字符串，是曲线在 `graphConfig` 里的引用名，必须唯一 |
| `scale` | 可选，解码值乘以该系数后再画（如 ADC 原始值转电压） |

**② `graphConfig`：声明"图表长什么样"**

```jsonc
"graphConfig": [
    {
        "type": "realtime",                 // 实时滚动图
        "label": "底盘左前轮 目标/反馈轮速 (rad/s)",  // 面板标题
        "timespan": 10,                     // 横轴保留最近 10 秒
        "minimum": -25.0,                   // 纵轴下限，按实际量程改
        "maximum": 25.0,                    // 纵轴上限
        "plots": [                          // 数组：把若干曲线画到同一张图里对比
            { "graphId": "plot0", "label": "FL target",   "color": "#1c8c5a" },
            { "graphId": "plot1", "label": "FL feedback", "color": "#c24344" }
        ]
    }
]
```

典型用法是把"目标值"和"反馈值"两条曲线放进同一张图（两个 `plots` 项），直观对比 PID 跟随效果——当前配置正是如此：**曲线 0 = 左前轮目标轮速，曲线 1 = 左前轮反馈轮速**。

### 5.3 固件端使用（bsp_log_plot）

固件侧只有一个 API（`bsp_log.h` / `bsp_log.c`）：

```c
bsp_log_plot(0, target_speed);    // 曲线 1 → RTT 通道 1
bsp_log_plot(1, feedback_speed);  // 曲线 2 → RTT 通道 2
```

- 参数 `curve` 为曲线索引：`0` → 通道 1（Plot0），`1` → 通道 2（Plot1）；传错会被忽略。
- 每个通道各 512 字节环形缓冲，`NO_BLOCK_SKIP` 模式：主机来不及读时**丢弃新数据但不阻塞**，绝不会卡住控制循环。
- 通道缓冲由 `bsp_log_plot_init()` 配置，在 `bsp_log_init()` 和早于 main 的构造函数里都会执行，保证调试器一连接控制块就绪（这也是 `address: "auto"` 能找到控制块的前提）。

**采样率怎么定**：当前 100Hz（1kHz 任务里每 10ms 发一次），100Hz × 4B ≈ 400 B/s，远低于缓冲与轮询能力。经验上限约 1kHz/通道；再高图表会自动抽稀丢点，波形反而失真。节流模板见 `robot_task.c` 中的实际实现：

```c
static uint32_t last_plot_tick;
...
if ((uint32_t)(now_ms - last_plot_tick) >= ROBOT_PLOT_INTERVAL_MS) {  // 10ms → 100Hz
    last_plot_tick = now_ms;
    bsp_log_plot(0, value);
}
```

**当前波形内容**：`robot_task.c` 的 `robot_task_control()` 中，曲线 0 发送左前轮目标轮速 `robot.wheel[0].target_velocity_rad_per_s`，曲线 1 发送左前轮反馈轮速（`module_motor_get_feedback()`）。想看其他轮或其他变量，改这两个调用和 `launch.json` 的标签即可。

### 5.4 使用步骤

1. 编译烧录新固件（`Ctrl+Shift+B` 或 `OpenOCD: flash` 任务）。
2. 按 `F5` 启动 `OpenOCD: Debug`，再按一次 `F5` 全速运行。
3. cortex-debug 会自动打开 **"RTT 波形"** 图表面板（编辑器标签页；没看到就找侧边栏 `CORTEX-DEBUG GRAPHS` 视图）。
4. 同时 TERMINAL 里通道 0 文本日志照常输出——**日志和波形在同一个调试会话里并行查看**。
5. 曲线不对时先查两处：固件是否真的在发（变量是否更新）、`graphConfig` 的纵轴范围是否匹配数据量程。

## 六、遥控通道实时打印

已在 `robot_adapter.c` 的 `robot_adapter_update()` 中加入定时打印，默认 100 ms 一条（宏 `ROBOT_RC_LOG_INTERVAL_MS`）：

```
[I] RC CH1= -12 CH2=   0 CH3= 640 CH4=   0 | SWA= 1 SWB=-1 SWC= 0 SWD=-1 | online=1 cnt=12345
```

| 字段 | 含义 |
|---|---|
| CH1 | 右摇杆 左右 |
| CH2 | 右摇杆 前后 |
| CH3 | 左摇杆 前后 |
| CH4 | 左摇杆 左右 |
| SWA/SWB/SWC/SWD | 拨杆档位：`1`=上、`0`=中、`-1`=下（仅 SWC 为三档） |
| online | `1` 链路正常，`0` 失联 |
| cnt | 合法 SBUS 帧计数，不增长说明没收到帧 |

排查顺序：先看 `cnt` 是否增长 → 再看摇杆值是否跟手 → 最后核对拨杆档位（SWC 置中应为 `0`）。

## 七、常见问题

- **F5 报找不到 CMSIS-DAP**：检查 USB 线/口，确认驱动为 WinUSB（必要时用 Zadig）。
- **有调试但没日志**：确认程序已继续运行（不是停在 `main`），且日志代码确实被执行。
- **RTT 窗口空白**：缓冲是实时的，主机连接前打印的内容不会补发；可加 `reset run` 重跑。
- **波形面板没出现**：核对 `rttConfig` 解码器与 `graphConfig` 中 `graphId` 拼写一致（`plot0`/`plot1`），并确认用的是 `type: "graph"` 而非 `console`。
- **波形一直是平线或 0**：常见三种——固件实际没执行 `bsp_log_plot`（变量没更新/写错曲线索引）、发送的通道与解码器 `port` 不一致、纵轴 `minimum`/`maximum` 设置过大把曲线压成了直线。
- **波形毛刺、跳变**：数据率超出图表处理能力被自动抽稀，降低发送频率（参考 5.3 节节流写法）。
- **断点期间波形不动**：OpenOCD 靠 SWD 读 RAM，CPU 挂起时无新数据，属于正常现象，继续运行即恢复。
- **J-Link RTT Viewer 连不上**：它只支持 J-Link，DAPLink 请用上面的 VS Code 或 OpenOCD TCP 方式。
- **日志太多/被截断**：调大 `SEGGER_RTT_Conf.h` 中的 `BUFFER_SIZE_UP`（默认 1024）。
