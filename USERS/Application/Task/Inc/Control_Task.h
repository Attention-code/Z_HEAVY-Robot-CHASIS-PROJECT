/**
  ******************************************************************************
  * @file           : Control_Task.h
  * @brief          : Control task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* 控制逻辑已迁移到 ROBOT/robot_task.c，本头文件仅保留任务入口声明 */
void Control_Task(void const *argument);

#endif // CONTROL_TASK_H
