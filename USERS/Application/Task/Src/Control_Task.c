/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : 控制任务（ECF App + 电机总线刷新）
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Control_Task.h"
#include "cmsis_os.h"
#include "robot_config.h"
#include "robot_task.h"
#include "led.hpp"

#define ROBOT_CONTROL_DT_DEFAULT_S (0.001f)
#define ROBOT_CONTROL_DT_MAX_S     (0.02f)

/**
  * @brief  控制任务：1kHz 调度
  */
void Control_Task(void const *argument)
{
    TickType_t last_tick = osKernelSysTick();

    (void)argument;

    for (;;) {
        TickType_t now_tick = osKernelSysTick();
        uint32_t tick_delta = (uint32_t)(now_tick - last_tick);
        float delta_time_s = (float)tick_delta * 0.001f;

        last_tick = now_tick;

        if ((delta_time_s <= 0.0f) || (delta_time_s > ROBOT_CONTROL_DT_MAX_S)) {
            delta_time_s = ROBOT_CONTROL_DT_DEFAULT_S;
        }

        robot_task_control(delta_time_s);

        /* 汇报本任务存活 */
        LED_TaskKick(LED_TASK_CONTROL);

        osDelay(1);
    }
}
