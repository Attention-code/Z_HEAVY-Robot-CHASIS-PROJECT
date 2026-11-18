//
// Author: Breezeee
// Date: 25-5-28
//

#ifndef LED_HPP
#define LED_HPP

// =============================== 调用库 ===============================
#include "main.h"
// =============================== 变量区 ===============================
/* WS2812 所用 SPI6：MOSI=PA7（DIN 接此脚），SCK=PA5（无需接线） */
extern SPI_HandleTypeDef WS2812_SPI_UNIT;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB;

// =============================== 任务心跳定义 ===============================
/* 需要监控的任务列表，各任务在主循环中调用 LED_TaskKick() 打卡 */
typedef enum {
    LED_TASK_INS = 0,
    LED_TASK_CONTROL,
    LED_TASK_CAN,
    LED_TASK_COUNT
} LED_TaskID_t;
// =============================== 函数声明 ===============================
#ifdef __cplusplus
extern "C" {
#endif

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
void LED_Loop();
RGB HSV_to_RGB(float h, float s, float v);

void LED_TaskKick(LED_TaskID_t id);     /* 任务循环内调用：汇报本任务存活 */
void LED_Heartbeat(void);               /* Detect_Task 周期调用：检查打卡并刷灯 */
#ifdef __cplusplus
}
#endif
// =============================== 类声明 ===============================


#endif // LED_HPP
