/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.h
  * @brief          : The header file of bsp_can.h 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to extern the functions and structure
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef BSP_UART_H
#define BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include------------------------------------------------------------------*/
#include "stm32h7xx.h"

/**
 * @brief 遥控链路调试信息（UART7 打印用）
 */
typedef struct
{
  int16_t  Stick_CH2;             /*!< SBUS ch2（ch[1]），右摇杆上下 */
  int16_t  Stick_CH3;             /*!< SBUS ch3（ch[2]），左摇杆上下（当前控制用通道） */
  int8_t   SWC;                   /*!< 拨杆 s[0] */
  uint32_t Frame_Count;           /*!< 合法 SBUS 帧计数 */
  uint32_t RxByte_Count;          /*!< UART5 收到的原始字节数 */
  uint32_t RxError_Count;         /*!< UART5 接收错误次数 */
  uint32_t Frame_Bad_Count;       /*!< 帧尾非法的帧数 */
  uint8_t  LastEndByte;           /*!< 最近一帧的帧尾字节 */
  int32_t  Target_Velocity_Centi; /*!< 目标速度 * 100 */
  uint32_t CanError_Count;        /*!< FDCAN2 发送失败次数 */
} Remote_Debug_Info_Typedef;

extern void BSP_USART_Init(void);
	   		 
extern void USART_Vofa_Justfloat_Transmit(float SendValue1,float SendValue2,float SendValue3);
extern void USART_Remote_Debug_Transmit(const Remote_Debug_Info_Typedef *Info);
#endif