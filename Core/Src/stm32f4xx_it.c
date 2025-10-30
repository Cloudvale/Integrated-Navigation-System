/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "usarts.h"
#include "portmacro.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gnss_solve.h"
#include "imu_data.h"
#include "led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TickType_t previousTimestamp = 0;
int f_gnss,f_imu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern RTC_HandleTypeDef hrtc;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;
extern SD_HandleTypeDef hsd;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
	
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC wake-up interrupt through EXTI line 22.
  */
void RTC_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_WKUP_IRQn 0 */

  /* USER CODE END RTC_WKUP_IRQn 0 */
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_WKUP_IRQn 1 */

  /* USER CODE END RTC_WKUP_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
//	 uint32_t temp;
//	
//	if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!= RESET)       //������ܵ���һ֡����
//	{ 
//		__HAL_UART_CLEAR_IDLEFLAG(&huart3);                         //�����־λ
//		//temp = huart1.Instance->SR;                               //���״̬�Ĵ���SR,��ȡSR�Ĵ�������ʵ�����SR�Ĵ����Ĺ���
//		//temp = huart1.Instance->DR;                               //��ȡ���ݼĴ����е�����
//		  HAL_UART_DMAStop(&huart3);                                  //DMAֹͣ����
//		  temp  =__HAL_DMA_GET_COUNTER(&hdma_usart3_rx);              // ��ȡDMA��δ��������ݸ���   
//		  rx_imu_len=140-temp;                                        //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
////			if(rx_imu_len>0)
////			{
////				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8,GPIO_PIN_RESET);       //RS485_RE=0,�������ģʽ; RS485_RE = 1,���뷢��ģʽ
////			}
////		
//		recv_imu_end_flag = 1;	                                        // ������ɱ�־λ��1	
//		 
//		if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
//		{
//			static BaseType_t xHigherPriorityTaskWoken;
//			vTaskNotifyGiveFromISR(IMU_task_local_handler, &xHigherPriorityTaskWoken);
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//		}
//	}

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

	uint32_t temp;

	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!= RESET)											  	/* ������ܵ���һ֡���� */
	{ 
		
//		if(((USART1->SR)>>5)&0x01)                                				          /* ���Ϊ�����ж� */
//	 {
		
//		  if(main_pro_ok)
//			{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);                                         /* �����־λ */
			/* ����������溯������һ�������� */
			//temp = huart1.Instance->SR;  																							/* ���״̬�Ĵ���SR,��ȡSR�Ĵ�������ʵ�����SR�Ĵ����Ĺ��� */
			//temp = huart1.Instance->DR; 																						  /* ��ȡ���ݼĴ����е����� */
			HAL_UART_DMAStop(&huart1); 																									/* ֹͣDMA�Ĵ��� */    
			temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);													  /* ��ȡDMA��δ��������ݸ���	*/   
			//temp  = hdma_usart1_rx.Instance->NDTR;																	  /* ��ȡNDTR�Ĵ��� ��ȡDMA��δ��������ݸ��� */
			/*	���������Ǿ��Ч	*/
			gnss_info_data.rx_len =  BUFFER_SIZE - temp; 																							/* �ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ��� */

	
//			if(interval>0x100)                                                         /* ��Ч�������100msʱ */
//			{
//						recv_end_flag = 0;                         
//			}
//			else 

		  f_gnss = queue_en(&q_gnss, gnss_info_data.rx_buffer,150);
	   	HAL_UART_Receive_DMA(&huart1,gnss_info_data.rx_buffer,BUFFER_SIZE);                         /* GNSS���´�DMA���� */		
//		  gnss_info_data.recv_end_flag=1;

		
		}
//	HAL_UART_IRQHandler(&huart1);
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
//   
//  HAL_UART_IRQHandler(&huart1);

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	 uint32_t temp;
	
	if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!= RESET)       //������ܵ���һ֡����
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);                         //�����־λ
		//temp = huart1.Instance->SR;                               //���״̬�Ĵ���SR,��ȡSR�Ĵ�������ʵ�����SR�Ĵ����Ĺ���
		//temp = huart1.Instance->DR;                               //��ȡ���ݼĴ����е�����
		  HAL_UART_DMAStop(&huart3);                                  //DMAֹͣ����
		  temp  =__HAL_DMA_GET_COUNTER(&hdma_usart3_rx);              // ��ȡDMA��δ��������ݸ���   
		  imu_info_data.rx_imu_len=140-temp;                                        //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���

		 if(imu_info_data.rx_imu_buff[0]!=0x59)
			imu_rec_failed=1;
		 else
		 {
			imu_rec_failed=0;
		 f_imu = queue_en(&q_imu, imu_info_data.rx_imu_buff,51);
		 }
//		 test_cnt_DMA++;
		 HAL_UART_Receive_DMA(&huart3,imu_info_data.rx_imu_buff,IMU_READ_DATA);  
//		 imu_info_data.recv_imu_end_flag=1;                                                               /* �����־λ���� */
	}
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles SDIO global interrupt.
  */
void SDIO_IRQHandler(void)
{
  /* USER CODE BEGIN SDIO_IRQn 0 */

  /* USER CODE END SDIO_IRQn 0 */
  HAL_SD_IRQHandler(&hsd);
  /* USER CODE BEGIN SDIO_IRQn 1 */

  /* USER CODE END SDIO_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
//void TIM6_DAC_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

//  /* USER CODE END TIM6_DAC_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim6);
//  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

//  /* USER CODE END TIM6_DAC_IRQn 1 */
//}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */


  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_sdio_rx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles Ethernet global interrupt.
  */
//void ETH_IRQHandler(void)
//{
//  /* USER CODE BEGIN ETH_IRQn 0 */

//  /* USER CODE END ETH_IRQn 0 */
//  HAL_ETH_IRQHandler(&heth);
//  /* USER CODE BEGIN ETH_IRQn 1 */

//  /* USER CODE END ETH_IRQn 1 */
//}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_sdio_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */


//void USART6_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART6_IRQn 0 */

//  /* USER CODE END USART6_IRQn 0 */
//  HAL_UART_IRQHandler(&huart6);
//  /* USER CODE BEGIN USART6_IRQn 1 */

//  /* USER CODE END USART6_IRQn 1 */
//}

//void DMA2_Stream6_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

//  /* USER CODE END DMA2_Stream6_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_usart6_tx);
//  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

//  /* USER CODE END DMA2_Stream6_IRQn 1 */
//}

//void DMA2_Stream1_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

//  /* USER CODE END DMA2_Stream1_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_usart6_rx);
//  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

//  /* USER CODE END DMA2_Stream1_IRQn 1 */
//}



/* USER CODE END 1 */
