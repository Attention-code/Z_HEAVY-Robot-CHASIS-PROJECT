/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN 接收任务（ECF 帧分发）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "fdcan.h"
#include "cod-fdcan.h"
#include "robot_config.h"
#include "robot_task.h"
#include "led.hpp"

/**
  * @brief  CAN 任务：把 FDCAN 反馈帧分发到 ECF 电机总线
  * @note   中断里只置 RxPending 标志，实际取帧在此任务上下文完成。
  */
void CAN_Task(void const *argument)
{
    (void)argument;

    for (;;) {
        /* FDCAN2 总线恢复 */
        if (FDCAN2_BusOff != 0U) {
            (void)HAL_FDCAN_Stop(&hfdcan2);
            if (HAL_FDCAN_Start(&hfdcan2) == HAL_OK) {
                FDCAN2_BusOff = 0U;
            }
        }

        robot_task_can_receive();

        /* 汇报本任务存活 */
        LED_TaskKick(LED_TASK_CAN);

        osDelay(1);
    }
}
