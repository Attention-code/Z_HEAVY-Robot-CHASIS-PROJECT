/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the fdcan filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "cod-fdcan.h"

/* RX 中断只置位标志，取帧交给 ECF 的 bsp_can_dispatcher（任务上下文） */
volatile uint8_t FDCAN2_BusOff;
volatile uint8_t FDCAN1_RxPending;
volatile uint8_t FDCAN2_RxPending;
volatile uint8_t FDCAN3_RxPending;

/**
  * @brief  Configures the FDCAN Filter. 
            FDCAN1:CLASSIC_CAN  FDCAN2:FDCAN  FDCAN3:CLASSIC_CAN
  * @param  None
  * @retval None
  */
void BSP_FDCAN_Init(void){

  FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID; // ����ID����ѡ�� ��׼ID
  FDCAN1_FilterConfig.FilterIndex = 0;           //��ǰFDCAN��������ţ��������ö�����������˲�ͬ��ID ��������0��1��2....
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK; //������Maskģʽ �غ�������ID1��ID2������
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//ѡ���ĸ�FIFO�����գ�����CubeMX����������FIFO1�͸ĳ�FDCAN_FILTER_TO_RXFIFO1
  FDCAN1_FilterConfig.FilterID1 = 0x00000000; // ������У�ֻҪID2����0x00000000�Ͳ�����˵��κ�ID
  FDCAN1_FilterConfig.FilterID2 = 0x00000000; //��������
  
  HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig); //���������õ�CAN1
		
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); //����CAN1��ȫ�ֹ��ˣ����ǿ���������
 
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//��FIFO0���������ݽ����ж�
  
  HAL_FDCAN_Start(&hfdcan1);//ʹ��CAN1
 	
	
	FDCAN_FilterTypeDef FDCAN2_FilterConfig;//FDCAN2过滤器结构体
	
  FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;//过滤器过滤ID类型
  FDCAN2_FilterConfig.FilterIndex = 0;//过滤器编号0
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;//过滤器类型：经典位屏蔽过滤器
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//过滤后进入FIFO1区
  FDCAN2_FilterConfig.FilterID1 = 0x00000000; //消息ID过滤器
  FDCAN2_FilterConfig.FilterID2 = 0x00000000; //过滤器屏蔽 每个位均设置0，即不过滤任何ID
  
	HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig);//根据过滤器结构体中指定的参数配置FDCAN接收过滤器

  //配置 FDCAN 全局过滤器
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  
  HAL_FDCAN_ActivateNotification(&hfdcan2,
      FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_BUS_OFF,
      0);//使能接收和 Bus-Off 中断

  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2);//使能FDCAN发送延时补偿
 
  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2,14,14);//FDCAN发送延时补偿时间设置

  HAL_FDCAN_Start(&hfdcan2);//FDCAN开始工作
	
	
	FDCAN_FilterTypeDef FDCAN3_FilterConfig;
	
	FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN3_FilterConfig.FilterIndex = 0;
  FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN3_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN3_FilterConfig.FilterID2 = 0x00000000; 
  
	HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_FDCAN_Start(&hfdcan3);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if (hfdcan == &hfdcan2 && (ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U) {
    FDCAN2_BusOff = 1U;
  }
}

/**
  * @brief  Rx FIFO 0 callback（FDCAN1 / FDCAN3）。
  * @note   只置位标志，不在此处读取 FIFO；取帧由 ECF 的 bsp_can_dispatcher 在任务上下文完成。
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  (void)RxFifo0ITs;

  if (hfdcan == &hfdcan1) {
    FDCAN1_RxPending = 1U;
  } else if (hfdcan == &hfdcan3) {
    FDCAN3_RxPending = 1U;
  }
}

/**
  * @brief  Rx FIFO 1 callback（FDCAN2）。
  * @note   同上，只置位标志。
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  (void)RxFifo1ITs;

  if (hfdcan == &hfdcan2) {
    FDCAN2_RxPending = 1U;
  }
}