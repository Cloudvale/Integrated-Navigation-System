/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "../USER/src/IMU/imu_data.h"
#include "../USER/src/GNSS/gnss_solve.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
// #define BUFFER_SIZE  700  
//extern  volatile uint32_t rx_len ;  //����һ֡���ݵĳ���
//extern volatile uint8_t recv_end_flag; //һ֡���ݽ�����ɱ�־
//extern uint8_t rx_buffer[BUFFER_SIZE];  //�������ݻ�������
//extern uint8_t rx_buffer_f[BUFFER_SIZE];
extern TaskHandle_t GNSS_task_local_handler; /* GNSS������ */
extern TaskHandle_t IMU_task_local_handler;  /* IMU������ */
extern TaskHandle_t SD_task_local_handler;   /* SD�������� */
//extern uint8_t gnss_parse_end_flag;

//extern uint8_t recv_youxiao_flag;
/* IMU���������� */
//extern volatile uint32_t rx_imu_len;
//extern volatile uint32_t recv_imu_end_flag;
//extern uint8_t rx_imu_buff[140];
//extern uint8_t rx_imu_buff_s[140];
//extern uint8_t imu_parse_end_flag;

//extern volatile uint32_t tx_send_data_len;
//extern volatile uint32_t rx_imu_send_len;

/* ���ⷢ������ */
//extern uint8_t tx_gnss_buffer[BUFFER_SIZE];  //gnss�������ݻ�������
//extern uint8_t tx_imu_buffer[140];  //�������ݻ�������

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

