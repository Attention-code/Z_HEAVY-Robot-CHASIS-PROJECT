/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions 
  * @author         : GrassFan Wang
  * @date           : 2025/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to init the  BSP_USART_Init functions
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_uart.h"
#include "usart.h"
#include "Ix6_Remote.h"
#include "Referee_System.h"
#include "Image_Transmission.h"
#include <stdio.h>

static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t );

static uint8_t sbus_byte;
static uint8_t sbus_frame[I6X_FRAME_LENGTH];
static uint8_t sbus_frame_index;

static void StartSbusReceive(void){
	sbus_frame_index = 0U;
	CLEAR_BIT(huart5.Instance->CR3, USART_CR3_DMAR);
	__HAL_UART_CLEAR_OREFLAG(&huart5);
	__HAL_UART_CLEAR_NEFLAG(&huart5);
	__HAL_UART_CLEAR_FEFLAG(&huart5);
	__HAL_UART_CLEAR_PEFLAG(&huart5);
	HAL_UART_Receive_IT(&huart5, &sbus_byte, 1U);
}

static void ProcessSbusByte(uint8_t byte){
	if (sbus_frame_index == 0U) {
		if (byte != 0x0FU) {
			return;
		}
	}

	sbus_frame[sbus_frame_index++] = byte;

	if (sbus_frame_index == I6X_FRAME_LENGTH) {
		sbus_frame_index = 0U;
		/* 帧尾校验与统计放在 sbus_to_i6x() 内，便于把接收机实际帧尾打出来 */
		sbus_to_i6x(&i6x_ctrl, sbus_frame);
	}
}

PLL2_ClocksTypeDef PLL2_ClockFreq;

#define USART1_RX_Switch  1  //Referee_System 0  Image_Transmission 1
/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
void BSP_USART_Init(void){
   
	 HAL_RCCEx_GetPLL2ClockFreq(&PLL2_ClockFreq);// Get PLL2 P Q R  Clock Frequency
	 uint32_t USART1_ClockFreq = PLL2_ClockFreq.PLL2_Q_Frequency;//USART1 use PLL2Q Clock Frequency
	
	#if USART1_RX_Switch
	
	 USART1->CR1 &= ~USART_CR1_UE; //Disable USART1
   USART1->BRR = (uint32_t)(USART1_ClockFreq/921600);// Set baudrate 921600
	 USART1->CR1 |= USART_CR1_UE;// Enable USART1
	 USART_RxDMA_MultiBuffer_Init(&huart1,(uint32_t *)Image_Trans_MultiRx_Buff[0],(uint32_t *)Image_Trans_MultiRx_Buff[1],IMAGE_TRANS_RX_LENGTH);
	
	#else 
   
	 USART1->CR1 &= ~USART_CR1_UE;
   USART1->BRR = (uint32_t)(USART1_ClockFreq/115200);// Set baudrate 115200
	 USART1->CR1 |= USART_CR1_UE;
	 USART_RxDMA_MultiBuffer_Init(&huart1,(uint32_t *)Referee_System_Info_MultiRx_Buf[0],(uint32_t *)Referee_System_Info_MultiRx_Buf[1],REFEREE_RXFRAME_LENGTH);
	
	#endif
	
	/* SBUS is received byte by byte to avoid DMA idle/restart frame loss. */
	 __HAL_UART_DISABLE_IT(&huart5, UART_IT_IDLE);
	 __HAL_UART_CLEAR_IDLEFLAG(&huart5);
	 HAL_UART_AbortReceive(&huart5);
	 StartSbusReceive();

	 const uint8_t uart7_ready[] = "UART7 READY\r\n";
	 HAL_UART_Transmit(&huart7, (uint8_t *)uart7_ready, sizeof(uart7_ready) - 1U, 20U);

}

/**
  * @brief  Init the multi_buffer DMA Transfer with interrupt enabled.
  * @param  huart       pointer to a UART_HandleTypeDef structure that contains
  *                     the configuration information for the specified USART Stream.  
  * @param  SrcAddress pointer to The source memory Buffer address
  * @param  DstAddress pointer to The destination memory Buffer address
  * @param  SecondMemAddress pointer to The second memory Buffer address in case of multi buffer Transfer  
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval none
  */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){

 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

 huart->RxXferSize    = DataLength * 2;

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 
		
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

  /* Configure the destination memory Buffer address */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);	
	
}



/**
  * @brief  USER USART1 Reception Event Callback.(Referee_System)
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */



static void USER_USART1_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

#if USART1_RX_Switch
	
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET){
		
					/* Disable DMA */
					__HAL_DMA_DISABLE(huart->hdmarx);
          
				  /* Switch Memory 0 to Memory 1*/
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
					
				  /* Juge whether size is equal to the length of the received data */
					if(Size > 10)
					{
					
						/* Memory 0 data update to remote_ctrl*/
						Image_Transmission_Info_Update(Image_Trans_MultiRx_Buff[0]);
					
					}
					
					/* Reset the receive count */
				  __HAL_DMA_SET_COUNTER(huart->hdmarx,IMAGE_TRANS_RX_LENGTH*2);

			}
			/* Current memory buffer used is Memory 1 */
			else{
					/* Disable DMA */
					__HAL_DMA_DISABLE(huart->hdmarx);
				 
				  /* Switch Memory 1 to Memory 0*/
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
				
					if(Size > 10)
					{
						/* Memory 1 to data update to remote_ctrl*/
						Image_Transmission_Info_Update(Image_Trans_MultiRx_Buff[1]);
					}
				  /* Reset the receive count */
					__HAL_DMA_SET_COUNTER(huart->hdmarx,IMAGE_TRANS_RX_LENGTH*2);
			}
#else
  
		if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET){
		
					__HAL_DMA_DISABLE(huart->hdmarx);
          
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
					
					if(Size > 10 )
					{
						Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[0]);
						memset(Referee_System_Info_MultiRx_Buf[0],0,REFEREE_RXFRAME_LENGTH);
					}
					__HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH*2);
			}
			/* Current memory buffer used is Memory 1 */
			else{
				
					__HAL_DMA_DISABLE(huart->hdmarx);
				 
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
				
					if(Size > 10)
					{
						Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[1]);
						memset(Referee_System_Info_MultiRx_Buf[1],0,REFEREE_RXFRAME_LENGTH);

					}
					__HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH*2);

			}
 

#endif 
}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	 if(huart == &huart1){
	 
      USER_USART1_RxHandler(huart,Size);
		
	 }
	
   huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
  /* Enalbe IDLE interrupt */
   __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
   SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
  /* Enable DMA */
   __HAL_DMA_ENABLE(huart->hdmarx);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart5) {
		i6x_ctrl.rx_byte_cnt++;
		ProcessSbusByte(sbus_byte);
		HAL_UART_Receive_IT(&huart5, &sbus_byte, 1U);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if (huart == &huart5) {
		i6x_ctrl.rx_error_cnt++;
		HAL_UART_AbortReceive(&huart5);
		StartSbusReceive();
	}
}

void USART_Vofa_Justfloat_Transmit(float SendValue1,float SendValue2,float SendValue3){
 
    __attribute__((section (".AXI_SRAM")))  static uint8_t Rx_Buf[16];

		uint8_t *SendValue1_Pointer,*SendValue2_Pointer,*SendValue3_Pointer;

		SendValue1_Pointer = (uint8_t *)&SendValue1;
		SendValue2_Pointer = (uint8_t *)&SendValue2;
		SendValue3_Pointer = (uint8_t *)&SendValue3;


		Rx_Buf[0] =  *SendValue1_Pointer;
		Rx_Buf[1] =  *(SendValue1_Pointer + 1);
		Rx_Buf[2] =  *(SendValue1_Pointer + 2);
		Rx_Buf[3] =  *(SendValue1_Pointer + 3);
		Rx_Buf[4] =  *SendValue2_Pointer;
		Rx_Buf[5] =  *(SendValue2_Pointer + 1);
		Rx_Buf[6] =  *(SendValue2_Pointer + 2);
		Rx_Buf[7] =  *(SendValue2_Pointer + 3);
		Rx_Buf[8] =  *SendValue3_Pointer;
		Rx_Buf[9] =  *(SendValue3_Pointer + 1);
		Rx_Buf[10] = *(SendValue3_Pointer + 2);
		Rx_Buf[11] = *(SendValue3_Pointer + 3);
		Rx_Buf[12] =  0x00;
		Rx_Buf[13] =  0x00;
		Rx_Buf[14] =  0x80;
		Rx_Buf[15] =  0x7F;
		HAL_UART_Transmit_DMA(&huart7,Rx_Buf,sizeof(Rx_Buf));
	

}

void USART_Remote_Debug_Transmit(const Remote_Debug_Info_Typedef *Info){
	char message[192];
	int32_t target_magnitude = (Info->Target_Velocity_Centi < 0) ?
								 -Info->Target_Velocity_Centi : Info->Target_Velocity_Centi;
	char target_sign = (Info->Target_Velocity_Centi < 0) ? '-' : ' ';
	int length = snprintf(message, sizeof(message),
						  "RC ch2=%d ch3=%d SWC=%d fr=%lu rxb=%lu err=%lu bad=%lu end=0x%02X target=%c%ld.%02ld CANerr=%lu\r\n",
						  (int)Info->Stick_CH2, (int)Info->Stick_CH3, (int)Info->SWC,
						  (unsigned long)Info->Frame_Count, (unsigned long)Info->RxByte_Count,
						  (unsigned long)Info->RxError_Count, (unsigned long)Info->Frame_Bad_Count,
						  (unsigned int)Info->LastEndByte,
						  target_sign, (long)(target_magnitude / 100),
						  (long)(target_magnitude % 100),
						  (unsigned long)Info->CanError_Count);

	if (length > 0 && length < (int)sizeof(message)) {
		HAL_UART_Transmit(&huart7, (uint8_t *)message, (uint16_t)length, 20U);
	}
}

