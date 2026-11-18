/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Detect_Task.h
  * @brief          : Detect task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Detect_Task.h"
#include "Control_Task.h"
#include "bsp_gpio.h"
#include "led.hpp"

/**
  * @note turn on:  800
	*       turn off: 4150
	*/

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the StartDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
//  TickType_t systick = 0;
	

  /* Infinite loop */
  for(;;)
  {
    /* LED 心跳：绿灯闪=任务全部存活，红灯闪=有任务卡死，不闪=Detect_Task 异常 */
    LED_Heartbeat();

    osDelay(100);
  }
  /* USER CODE END Detect_Task */
}

		
		
		
		
		
		
		
	