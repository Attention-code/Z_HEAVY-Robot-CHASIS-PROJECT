# bsp_led — WS2812 心跳灯

通过 SPI6 驱动一颗 WS2812（全彩 RGB），作为**任务存活心跳指示灯**。各被监控任务在主循环内"打卡"，`Detect_Task` 汇总检查后刷灯；看灯的颜色/状态即可判断板子上任务的存活情况。

## 硬件接线

| WS2812 引脚 | MCU 引脚 | 说明 |
|-------------|----------|------|
| DIN | **PA7**（SPI6_MOSI） | 唯一需要接的信号线 |
| VCC | 5V | 整条亮需考虑电流 |
| GND | GND | 必须共地 |

SCK（PA5）无需接线。SPI6 由 `LED_SPI_Init()` 在代码内直接初始化，**不需要在 CubeMX 里配置**。

## 时序原理

时钟链：HSE 24MHz → SYSCLK 320MHz → APB4 40MHz → SPI6 8 分频 = **5MHz**。

本编码 1 SPI 字节 = 1 个 WS2812 位：

| WS2812 位 | 字节模式 | 高电平时长 |
|-----------|----------|-----------|
| 0 码 | `0x60`（2 位高） | 0.4µs |
| 1 码 | `0x78`（4 位高） | 0.8µs |

恰好落在 WS2812B 标准时序（T0H=0.4µs / T1H=0.8µs）上。

## 心跳机制

```text
INS_Task ─────┐
Control_Task ─┼─ 每圈循环 LED_TaskKick() 打卡（记录当前 tick）
CAN_Task ─────┘
                  ↓
Detect_Task ── 每 100ms 调 LED_Heartbeat() 检查打卡并刷灯
```

## 灯语速查

| 状态 | 表现 | 含义 |
|------|------|------|
| 正常 | 彩虹渐变 + 呼吸亮度 | INS / Control / CAN 全部存活 |
| 异常 | 红灯 500ms 快闪 | 有任务 >200ms 未打卡（卡死/阻塞） |
| 冻结 | 灯停在某帧不动 | Detect_Task 自身或调度器异常 |
| 熄灭 | 完全不亮 | 主循环没跑起来 |

## 用法

```c
// 1. 被监控任务：主循环内打卡（如 Control_Task.c）
LED_TaskKick(LED_TASK_CONTROL);

// 2. Detect_Task：周期驱动心跳灯
for (;;) {
    LED_Heartbeat();
    osDelay(100);
}

// 3. 直接指定颜色（底层接口）
WS2812_Ctrl(0, 60, 0);          // 绿色
WS2812_Ctrl(0, 0, 0);           // 熄灭
```

## API 速查

| 函数 | 功能 |
|------|------|
| `WS2812_Ctrl(r, g, b)` | 底层刷灯：发送一帧 RGB 颜色（首次调用自动初始化 SPI6） |
| `LED_TaskKick(id)` | 任务打卡：汇报本任务存活，`id` 取 `LED_TaskID_t` |
| `LED_Heartbeat()` | 心跳检查 + 刷灯，由 Detect_Task 周期调用 |
| `HSV_to_RGB(h, s, v)` | HSV → RGB 转换，供渐变效果使用 |
| `LED_Loop()` | 彩虹流水灯（独立效果，当前未接入主流程） |

## 可调参数

宏定义在 [led.cpp](led.cpp) 顶部：

| 宏 | 默认值 | 含义 |
|----|--------|------|
| `LED_KICK_TIMEOUT_TICKS` | 200 | 打卡超时阈值（ms），超过判死 |
| `LED_BLINK_PERIOD_TICKS` | 500 | 异常红灯闪烁周期（ms） |
| `LED_HUE_SPEED` | 90.0 | 彩虹色相旋转速度（度/秒），4s 一圈 |
| `LED_BREATH_PERIOD_S` | 2.0 | 呼吸周期（秒） |
| `LED_BREATH_V_MAX` | 0.30 | 呼吸最高亮度（0~1，建议 ≤0.3） |
| `LED_BREATH_V_MIN` | 0.05 | 呼吸最低亮度 |

## 新增监控任务

1. 在 [led.hpp](led.hpp) 的 `LED_TaskID_t` 枚举中追加一项（注意 `LED_TASK_COUNT` 自动跟随）；
2. 在新任务的主循环内调用 `LED_TaskKick(LED_TASK_XXX)`；
3. 无需改动 Detect_Task / 心跳逻辑。
