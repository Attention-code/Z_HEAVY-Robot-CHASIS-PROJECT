/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.h
  * @brief          : The header file of bsp_can.c 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to extern the functions and structure
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
/* 注意：守卫名不能是 BSP_CAN_H，否则会与 ECF 的 bsp_can.h 冲突 */
#ifndef COD_FDCAN_H
#define COD_FDCAN_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"

/* Externs ------------------------------------------------------------------*/
extern volatile uint8_t FDCAN2_BusOff;

/* RX 中断只置位标志，实际取帧在任务上下文由 ECF bsp_can_dispatcher 完成 */
extern volatile uint8_t FDCAN1_RxPending;
extern volatile uint8_t FDCAN2_RxPending;
extern volatile uint8_t FDCAN3_RxPending;

extern void BSP_FDCAN_Init(void);

#ifdef __cplusplus
}
#endif

#endif