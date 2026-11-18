// =============================== 引入头文件 ===============================
#include "led.hpp"

#include <math.h>

#include "cmsis_os.h"

// =============================== 宏定义区 ===============================
#define WS2812_LowLevel 0xC0   // 0码
#define WS2812_HighLevel 0xF0  // 1码

/* 心跳参数：任务 1kHz 打卡，超过 TIMEOUT 判死；灯 500ms 闪一次 */
#define LED_KICK_TIMEOUT_TICKS 200u  // 打卡超时阈值（tick，1tick=1ms）
#define LED_BLINK_PERIOD_TICKS 500u  // 闪烁周期（tick）

/* 呼吸渐变参数 */
#define LED_HUE_SPEED 90.0f       // 色相旋转速度（度/秒），4s 转完一圈彩虹
#define LED_BREATH_PERIOD_S 2.0f  // 呼吸周期（秒）
#define LED_BREATH_V_MAX 0.30f    // 呼吸最高亮度
#define LED_BREATH_V_MIN 0.05f    // 呼吸最低亮度
#define LED_PI 3.14159265f

// =============================== 变量区 ===============================
/* 各任务最近一次打卡时刻（tick） */
static volatile uint32_t task_kick_tick[LED_TASK_COUNT] = {0};

SPI_HandleTypeDef WS2812_SPI_UNIT;

// =============================== 函数实现 ===============================
/**
 * @brief  初始化 WS2812 所用的 SPI6 硬件
 * @note   时钟链：SPI6 内核时钟显式挂到 PCLK4，分频按运行时实际频率计算，
 *         目标 SPI 比特率 5MHz。
 *         本编码 1 字节 = 1 个 WS2812 位：2 位高电平 = 0 码（T0H=0.4us），
 *         4 位高电平 = 1 码（T1H=0.8us），恰好符合 WS2812B 时序。
 *         接线：WS2812 DIN -> PA7（SPI6_MOSI），SCK 无需接线，共地。
 */
static void LED_SPI_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI6_CLK_ENABLE();

  /* 显式配置 SPI6 内核时钟源为 PCLK4（不依赖复位默认值） */
  RCC_PeriphCLKInitTypeDef spi6_clk = {0};
  spi6_clk.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
  spi6_clk.Spi6ClockSelection = RCC_SPI6CLKSOURCE_D3PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&spi6_clk) != HAL_OK) {
    return;
  }

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;  // PA5=SCK, PA7=MOSI
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPI6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  WS2812_SPI_UNIT.Instance = SPI6;
  WS2812_SPI_UNIT.Init.Mode = SPI_MODE_MASTER;
  WS2812_SPI_UNIT.Init.Direction = SPI_DIRECTION_2LINES;
  WS2812_SPI_UNIT.Init.DataSize = SPI_DATASIZE_8BIT;
  WS2812_SPI_UNIT.Init.CLKPolarity = SPI_POLARITY_LOW;
  WS2812_SPI_UNIT.Init.CLKPhase = SPI_PHASE_1EDGE;
  WS2812_SPI_UNIT.Init.NSS = SPI_NSS_SOFT;
  /* 按实际内核频率选择分频（MBR），目标 5MHz，取不超过目标的最大频率档。
   * 内核 20MHz -> /4=5MHz；40MHz -> /8=5MHz，以此类推。 */
  uint32_t kernel_hz = HAL_RCCEx_GetD3PCLK1Freq();  // = PCLK4
  uint32_t mbr = 0;
  while (mbr < 7U && (kernel_hz >> (mbr + 1U)) > 5000000U) {
    mbr++;
  }
  WS2812_SPI_UNIT.Init.BaudRatePrescaler = mbr << SPI_CFG1_MBR_Pos;
  WS2812_SPI_UNIT.Init.FirstBit = SPI_FIRSTBIT_MSB;
  WS2812_SPI_UNIT.Init.TIMode = SPI_TIMODE_DISABLE;
  WS2812_SPI_UNIT.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  if (HAL_SPI_Init(&WS2812_SPI_UNIT) != HAL_OK) {
    return;
  }
  __HAL_SPI_ENABLE(&WS2812_SPI_UNIT);
}

/**
 * @brief  控制 WS2812 灯带颜色输出
 * @param  r: 红色分量（0~255）
 * @param  g: 绿色分量（0~255）
 * @param  b: 蓝色分量（0~255）
 * @note   通过 SPI 发送 RGB 颜色数据到 WS2812 灯带，附加空闲位用于刷新
 */
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
  static bool spi_ready = false;
  if (!spi_ready) {
    LED_SPI_Init();
    spi_ready = true;
  }

  uint8_t txbuf[24];
  uint8_t res = 0;
  for (int i = 0; i < 8; i++) {
    txbuf[7 - i] = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
  }
  /* 先发 24 字节有效编码（G-R-B 顺序，WS2812 数据格式），
   * 再补 100 字节 0 保持 MOSI 低电平，凑足 >50us 的锁存（RESET）时间 */
  HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
  for (int i = 0; i < 100; i++) {
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
  }
}

/**
 * @brief  HSV 色彩空间转换为 RGB 色彩
 * @param  h: 色相（Hue，0~360）
 * @param  s: 饱和度（Saturation，0~1）
 * @param  v: 明度（Value，0~1）
 * @retval RGB 结构体（r/g/b 分量 0~255）
 * @note   用于实现亮度恒定的渐变颜色效果
 */
RGB HSV_to_RGB(float h, float s, float v)
{
  float r, g, b;
  int i;
  float f, p, q, t;

  if (s == 0.0f) {
    r = g = b = v;
  }
  else {
    h /= 60.0f;
    i = (int)h;
    f = h - i;
    p = v * (1.0f - s);
    q = v * (1.0f - s * f);
    t = v * (1.0f - s * (1.0f - f));

    switch (i % 6) {
      case 0:
        r = v;
        g = t;
        b = p;
        break;
      case 1:
        r = q;
        g = v;
        b = p;
        break;
      case 2:
        r = p;
        g = v;
        b = t;
        break;
      case 3:
        r = p;
        g = q;
        b = v;
        break;
      case 4:
        r = t;
        g = p;
        b = v;
        break;
      case 5:
        r = v;
        g = p;
        b = q;
        break;
    }
  }

  RGB rgb = {.r = (uint8_t)(r * 255), .g = (uint8_t)(g * 255), .b = (uint8_t)(b * 255)};
  return rgb;
}

/**
 * @brief  LED 渐变控制循环（流水灯逻辑）
 * @note   每次调用推进色相变化，产生平滑渐变效果
 */
void LED_Loop()
{
  static float hue = 0.0f;

  // hue 取值范围 0~360，用于控制颜色
  hue += 2.0f;
  if (hue >= 360.0f) hue -= 360.0f;

  // 固定饱和度 1.0、亮度 0.3（可调节亮度强度）
  RGB color = HSV_to_RGB(hue, 1.0f, 0.3f);
  WS2812_Ctrl(color.r, color.g, color.b);
}

/**
 * @brief  任务打卡：汇报本任务存活
 * @param  id: 任务编号（LED_TaskID_t）
 * @note   各被监控任务在主循环内调用，记录当前系统 tick
 */
void LED_TaskKick(LED_TaskID_t id)
{
  if (id < LED_TASK_COUNT) {
    task_kick_tick[id] = osKernelSysTick();
  }
}

/**
 * @brief  LED 心跳：根据各任务打卡情况刷灯
 * @note   由 Detect_Task 周期调用（建议 ≥100ms 一次）
 *         - 全部任务存活：彩虹渐变 + 呼吸亮度
 *         - 有任务超时：红灯 500ms 周期闪烁
 *         - LED 完全冻结：Detect_Task 自身或调度器异常
 */
void LED_Heartbeat(void)
{
  uint32_t now = osKernelSysTick();

  // 检查所有任务打卡是否超时
  bool all_alive = true;
  for (int i = 0; i < LED_TASK_COUNT; i++) {
    if ((now - task_kick_tick[i]) > LED_KICK_TIMEOUT_TICKS) {
      all_alive = false;
      break;
    }
  }

  if (all_alive) {
    // ---- 正常态：彩虹渐变 + 呼吸亮度 ----
    static float hue = 0.0f;
    static uint32_t last_tick = 0;

    // 计算距上次调用的时长（首次调用按 0 处理，并限幅防止卡顿跳变）
    float dt = (last_tick == 0) ? 0.0f : (float)(now - last_tick) * 0.001f;
    if (dt > 0.5f) dt = 0.5f;
    last_tick = now;

    hue += LED_HUE_SPEED * dt;  // 色相持续旋转，红橙黄绿青蓝紫循环
    if (hue >= 360.0f) hue -= 360.0f;

    // 正弦呼吸：亮度在 V_MIN ~ V_MAX 之间平滑往复
    float breath = 0.5f + 0.5f * sinf(2.0f * LED_PI * (float)now * 0.001f / LED_BREATH_PERIOD_S);
    float v = LED_BREATH_V_MIN + (LED_BREATH_V_MAX - LED_BREATH_V_MIN) * breath;

    RGB color = HSV_to_RGB(hue, 1.0f, v);
    WS2812_Ctrl(color.r, color.g, color.b);
  }
  else {
    // ---- 异常态：红灯 500ms 闪烁 ----
    static uint32_t last_toggle_tick = 0;
    static bool blink_on = false;
    static bool led_updated = false;

    if (now - last_toggle_tick >= LED_BLINK_PERIOD_TICKS) {
      last_toggle_tick = now;
      blink_on = !blink_on;
    }

    bool on = blink_on;
    if (on != led_updated) {
      led_updated = on;
      if (on) {
        WS2812_Ctrl(60, 0, 0);  // 红
      }
      else {
        WS2812_Ctrl(0, 0, 0);  // 灭
      }
    }
  }
}