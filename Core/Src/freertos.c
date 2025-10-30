/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*--------GNSS_include-----*/
//#include "info.h"
//#include "parser.h"
//#include "gmath.h"
#include "gnss_solve.h"
/*---------IMU_include------*/
#include "analysis_data.h"
#include "imu_data.h"
/*---------SDIO_include-----*/
#include "sd_data.h"
#include "sdio_sdcard.h"
/*----------led_include-----*/
#include "led.h"
/*---------sram_include-----*/
#include "sram.h"
/*---------usarts_include---*/
#include "usart.h"
#include "usarts.h"
/*---------lcd_include------*/
#include "lcd.h"
/*---------key_include------*/
#include "key.h"
/*---------IMU_include------*/
#include "malloc.h"
#include "exfuns.h"
//#include "ff.h"
#include "fattester.h"
#include "string.h"
#include "timers.h"
#include "lwip_comm.h"
#include "lwipopts.h"
#include "lwip_demo.h"
#include "lwip/udp.h"
#include  "btim.h"
#include "delay.h"
#include "lwip_data.h"
#include "fatfs.h"
#include "ffconf.h"
#include "navimain.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define READ_NUM  160
//#define READ_OFF  0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/*---------GNSS_Variables------*/
  volatile uint8_t flgssss=0;
	
	uint64_t test_cnt_TASK = 0;
	
	uint8_t kal_cal_flag=0;
	
	uint8_t flag = 0;
	uint32_t count[2] = {0};
	
	uint32_t ret = 0;
	
	volatile double lat = 0;
	volatile double lon = 0;
	volatile double elv = 0;
#define BLOCK_START_ADDR         0     /* Block start address      */
#define NUM_OF_BLOCKS            1   /* Total number of blocks   */
#define BUFFER_WORDS_SIZE        ((BLOCKSIZE * NUM_OF_BLOCKS) >> 2) /* Total data size in bytes */

//FATFS fs;                       /* FatFs �ļ�ϵͳ���� */
//FIL file;                       /* �ļ����� */
FRESULT f_res;                  /* �ļ�������� */
UINT fnum;                      /* �ļ��ɹ���д���� */
BYTE ReadBuffer[1024] = {0};    /* �������� */
BYTE WriteBuffer[300] = {0};            /* д������ */

/*---------SD_Variables------*/ 

/*---------SD_Variables------*/ 

/* USER CODE END Variables */
osThreadId MainTaskHandle;
osThreadId GNssTaskHandle;
osThreadId IMU_TaskHandle;
osTimerId myTimer01Handle;
osTimerId myTimer02Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/**
 * @breif       ����UI
 * @param       mode :  bit0:0,������;1,����ǰ�벿��UI
 *                      bit1:0,������;1,���غ�벿��UI
 * @retval      ��
 */



/* USER CODE END FunctionPrototypes */

void main_process(void const * argument);
void GNssProcess(void const * argument);
void imu_process(void const * argument);
void Callback01(void const * argument);
void Callback02(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* definition and creation of myTimer02 */
  osTimerDef(myTimer02, Callback02);
  myTimer02Handle = osTimerCreate(osTimer(myTimer02), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
	
	/* definition and creation of myTimer02 */
   /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MainTask */
  osThreadDef(MainTask, main_process, osPriorityRealtime, 0, 500);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* definition and creation of GNssTask */
  osThreadDef(GNssTask, GNssProcess, osPriorityHigh, 0, 500);
  GNssTaskHandle = osThreadCreate(osThread(GNssTask), NULL);

  /* definition and creation of IMU_Task */
  osThreadDef(IMU_Task, imu_process, osPriorityHigh, 0, 1800);
  IMU_TaskHandle = osThreadCreate(osThread(IMU_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_main_process */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_main_process */
void main_process(void const * argument)
{
  /* USER CODE BEGIN main_process */
	uint32_t total;
	uint32_t frees;
	uint8_t t = 0;
	uint8_t res = 0;
	
//  sys_stm32_clock_init(336, 8, 2, 7); /* ����ʱ��,168Mhz */
	led_init();                                                                              /* ��ʼ��LED */
	sram_init();                                                                             /* ��ʼ���ⲿSRAM */
	usart_init(115200);                                                                      /* ����2��ʼ��Ϊ115200 */
//  btim_timx_int_init(10000 - 1, 8400 - 1); /* 84 000 000 / 84 00 = 10 000 10Khz�ļ���Ƶ�ʣ�����10K��Ϊ1s */
	
	my_mem_init(SRAMIN);                                                                     /* ��ʼ���ڲ�SRAM�ڴ�� */
	my_mem_init(SRAMEX);                                                                     /* ��ʼ���ⲿSRAM�ڴ�� */
	my_mem_init(SRAMCCM);                                                               		 /* ��ʼ��CCM�ڴ�� */
	  	
	 while (sd_init())                                                                       /* ��ⲻ��SD�� */
	{
		lcd_show_string(30, 130, 300, 32, 32, "SD Card Error!", RED);                          /* ����ѭ�����жϣ�  MX_SDIO_SD_Init()�����жϣ���δ��Ч */
//		HAL_Delay(500);
		lcd_show_string(30, 130, 300, 32, 32, "Please Check! ", RED);
//		HAL_Delay(500);
		LED0_TOGGLE();                                                                         
	}
	  lcd_fill(30, 130, 240+30, 130 + 32, WHITE); 
	
	
/*************************************sd����FATFS-BEGIN����**************************************************************************************************/		
//	exfuns_init();                                                                           /* Ϊfatfs��ر��������ڴ� */
//  f_mount(fs[0], "0:", 1);                                                                 /* ����SD�� */
//	res = f_mount(fs[1], "1:", 1);                                                           /* ����FLASH */

	BYTE work[_MAX_SS];
	
	retSD = f_mount(&SDFatFS,SDPath,1);
	switch (retSD)
	{
		case FR_NO_FILESYSTEM:
		{
			retSD = f_mkfs(SDPath,FM_EXFAT,512,work,sizeof work);
		  if(retSD!=FR_OK)
				while(1);
		}
		 break;
		case FR_OK :
			break;
		default :
			while(1);
		
	}
//	HAL_Delay(20);
//	retSD = f_open(&SDFile,"134.txt",FA_CREATE_ALWAYS|FA_WRITE|FA_OPEN_APPEND);
	
//	char numss[25] = "11111111111111111111\n";
//	uint32_t num_rr;
	
//	retSD = f_write(&SDFile,(char *)numss,strlen((char *)numss),&num_rr);
//  if (res == 0X0D)                                                                         /* FLASH����,FAT�ļ�ϵͳ����,���¸�ʽ��FLASH */
//  {
//		
//		 lcd_show_string(30, 130, 300, 32, 32, "Flash Disk Formatting...", RED);                 /* ��ʽ��FLASH */
//		 res = f_mkfs("1:", 0, 512,0, _MAX_SS);                                                    /* ��ʽ��FLASH,1:,�̷�;0,ʹ��Ĭ�ϸ�ʽ������ */

//		 if (res == 0)
//		 {
////			 f_setlabel((const TCHAR *)"1:ALIENTEK");                                         /* ����Flash���̵�����Ϊ��ALIENTEK */
//			 lcd_show_string(30, 130, 300, 32, 32, "Flash Disk Format Finish", RED);          /* ��ʽ����� */
//		 }
//		 else 
//			 lcd_show_string(30, 130, 300, 32, 32, "Flash Disk Format Error ", RED);       /* ��ʽ��ʧ�� */

////		 HAL_Delay(1000);
//	 }

//   lcd_fill(30, 150, 240, 150 + 16, WHITE);                                               /* �����ʾ */

//   while (exfuns_get_free("0", &total, &frees))                                           /* �õ�SD������������ʣ������ */
//    {
//	 	  lcd_show_string(30, 150, 200, 16, 16, "SD Card Fatfs Error!", RED);               /* sd���ļ���ʼ���� */
//	 	  HAL_Delay(20);
//		  lcd_fill(30, 150, 240, 150 + 16, WHITE);                                           /* �����ʾ */
//		  HAL_Delay(20);
////		  LED0_TOGGLE();                                                                     /* LED0��˸ */
//    }
/*************************************sd����FATFS-END����*****************************************************************************************************/		
                                                                          
	
	lcd_show_init();                                                                        /* lcd������ʾ��ʼ�� */

	
/***************************************LWIP-BEGIN����***********************************************************************************************/
		
//	lwip_test_ui(1);                    /* ����ǰ�벿��UI */

//  lcd_show_string(300, 110, 200, 16, 16, "lwIP Init !!", BLUE);
  

#if   LWIP_SEND_START  		                                                        /* �������ݷ��ͺ궨��  1Ϊ�������ͣ�0Ϊ�ر� */
  while (lwip_comm_init() != 0)                                       				    /* lwip����ʧ�� */
  {
    lcd_show_string(30, 130, 300, 32, 32, "lwIP Init failed!!", BLUE);
    HAL_Delay(20);
    lcd_fill(30, 130, 200 + 30, 130 + 32, WHITE);
    lcd_show_string(30, 130, 200, 32, 32, "Retrying...       ", BLUE);           /* ��ʾ���¼��� */
    HAL_Delay(20);
//    LED1_TOGGLE();
    }

    while (!ethernet_read_phy(PHY_SR))                                            /* ���MCU��PHYоƬ�Ƿ�ͨ�ųɹ� */
    {
 //       		printf("MCU��PHYоƬͨ��ʧ�ܣ������·����Դ�룡������\r\n");
		lcd_show_string(30, 150, 300, 32, 32, "MCU to PHY failed��������", BLUE);           /* MCU��PHYоƬͨ��ʧ�ܣ������·����Դ�룡������ */
		
    }
    
		
#if LWIP_DHCP                                                                     //
//    lcd_show_string(6, 130, 200, 16, 16, "DHCP IP configing... ", BLUE);           /* ��ʼDHCP */
    
    while ((g_lwipdev.dhcpstatus != 2) && (g_lwipdev.dhcpstatus != 0XFF))          /* �ȴ�DHCP��ȡ�ɹ�/��ʱ��� */
    {
        lwip_periodic_handle();                                                    /* lwip��ѯ���� */
        HAL_Delay(10);
    }	
#endif	
		
		lwip_demo();                                                                   /* lwIP������� */
    
#endif

		
	key_time_init_selcet();                                                      //����ʱ��Ľ����Լ���ʼ���ж�
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //ʹ��IDLE�ж�
	  HAL_UART_Receive_DMA(&huart1,gnss_info_data.rx_buffer,BUFFER_SIZE);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); //ʹ��IDLE�ж�
	  HAL_UART_Receive_DMA(&huart3,imu_info_data.rx_imu_buff,IMU_READ_DATA);  //DMA���պ������˾�һ��Ҫ�ӣ����ӽ��ղ�����һ�δ�������ʵ���ݣ��ǿյģ��Ҵ�ʱ���յ������ݳ���Ϊ�����������ݳ���
		
	LED0(1);
		
	vTaskSuspend(NULL);  // ������������
/******************************************LWIP-END����*********************************************************************************************/		
  /* Infinite loop */
		
		
  for(;;)
  {	 
		/* ���ݴ��� */                                                                      		/* ���ݴ���ʼ */		
    osDelay(1);
  }
  /* USER CODE END main_process */
}

/* USER CODE BEGIN Header_GNssProcess */
/**
* @brief Function implementing the GNssTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GNssProcess */
void GNssProcess(void const * argument)
{
  /* USER CODE BEGIN GNssProcess */
	osDelay(2000);                                                                /* ��ʱ����main_porcess���̳�ʼ������ */
//	vTaskSuspend(NULL);  // ������������
	gnss_zero_INFO(&gnss_info_data);
	nmea_zero_INFO(&info);                                                       /* info��Ϣ��ʼ�� */
  nmea_parser_init(&parser);																										/* parser��������ʼ�� */
	osTimerStart(myTimer02Handle,2000);

	 uint8_t show_flag=0;
	 uint8_t show_key=0;
//  /* Infinite loop */
  for(;;)                                                                       /* GNSS_Task��ѭ�� */
  {

//		gnss_info_data.recv_end_flag=0;
		
		int gnss_len = queue_len(q_gnss);
		if(gnss_len != 0)                                                           /* ���ձ�־λΪ1ʱ����ʼ�������ݴ��� */
		{
			 /* GNSS�忨���ݴ��� */
			memcpy(gnss_info_data.rx_buffer_f,gnss_info_data.rx_buffer,BUFFER_SIZE);                                 	/* copy�����ݻ����� */
			while(gnss_info_data.read_end_off<BUFFER_SIZE)                                                               	/* ���ݽ��պ�ѭ������ */ 
			 {
				memcpy(gnss_info_data.rx_parse,gnss_info_data.rx_buffer_f+gnss_info_data.read_end_off,READ_NUM);      	    /* ���ݴ��뻺���� */
				queue_de(&q_gnss, e_gnss,GNSS_BUFFER_SIZE);
				nmea_parse(&parser, (char *)e_gnss, READ_NUM, &info); 				 /* ���ݿ�ʼ���н��� */
				
				gnss_info_data.read_end_off+=READ_NUM;                                                                      /* ���ݼ��� */
			 }
					
			  gnss_info_data.read_end_off=READ_OFF;                                                    	/* ������������ */
			  gnss_info_data.rx_len = 0;                                                             		/* ������� */
			  gnss_info_data.recv_end_flag = READ_OFF;                                               		/* ������ս�����־λ */
				if(info.lat!=0)
			  {
					gnss_btim_flag=1;
					gnss_info_data.gnss_parse_end_flag=1;                                                  		/* ������ɱ�־ */
			  }
				if(info.sig==1)
				kal_cal_flag=1;
				

			 /* 2�������ں�1��������ǰ */
			 
				/* gnss������ʾ */
//				show_gnss_data();
//         KalUpdate_d(&Kal, struct CEarth* eth, struct CVect3* vn, struct Quat* qnb, struct CMAT3* Cnb, struct CVect3* att);//�����������

        /* ���к������ */
				lat = info.lat;
				lon = info.lon;
				elv = info.elv;
				 if(lat > 40)
				 {
					while(1);
				 }
//				sd_open_send();
        /* end */
		}

//						if(ret >= 100)
//				{
//					show_imu_data();
//					ret = 0;
//				}
  }
  /* USER CODE END GNssProcess */
}

/* USER CODE BEGIN Header_imu_process */
/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_process */
void imu_process(void const * argument)
{
  /* USER CODE BEGIN imu_process */
	 double t_1=0.005;       //��ʼʱ��
	double tk0=0;       //����ʱ��
	double T=10;        //����׼ʱ��
	int n=2;            //������
	
	struct CVect3 vn0={0,0,0};
	struct CVect3 pos0={30.52*d2r,114.31*d2r,36.0};
  

	
//	uint32_t g_outputs_info=0;
//  uint32_t g_outputs_infos=0;

	uint8_t only_oness=0;
	uint8_t double_ins=0;
	uint8_t num_add_accumulates=0;
	
//	uint8_t test_sd=0;
//	
//	struct Kalman Kal;
//	struct CVect3 Satt = {0.0192203,0.0192203,5*0.0192203};
//	struct CVect3 Svn={0.5,0.5,0.5};
//	struct CVect3 Spos={1,1,1};
//	struct CVect3 Sgyro={2.423889E-5,2.423889E-5,2.423889E-5};
//	struct CVect3 Sacc={0.15E-3*9.8,0.15E-3*9.8,0.15E-3*9.8};
//	struct CVect3 SWg={0.037021,0.037021,0.037021};
//	struct CVect3 SWa={0.02632868,0.02632868,0.02632868};
//	struct CVect3 gnss_pos={0,0,0};
	
  
		

	
//	char rximu_buff[50]={0};

	osDelay(5000);                                                        /* ��ʱ����main_porcess���̳�ʼ������ */
//	ins_init_param(&ins_info_init,1,0.005,0,10);                             //����׼���г�ʼ��������������и�ֵ
	sd_scan_files();                                                       /* ����sd�����ﴢ���ɨ��ͳ�ʼ�� */
	imu_zero_INFO(&imu_info_data);
	
////  /* Infinite loop */
  for(;;)
  {

 LED0(0);
		int imu_len = queue_len(q_imu);
		if(imu_len != 0)                                                                                /* �����־λ�ж� */
		{
		LED0(1);
			/* IMU���������ݴ��� */
			imu_info_data.recv_imu_end_flag=0;
			
				queue_de(&q_imu, e_imu,IMU_BUFFER_SIZE);  
				analysis_data(e_imu,IMU_RECV_DATA,&g_output_info);   			/* imu���ݽ��� */
				imu_info_data.rx_imu_len=0; 
 			  if(gnss_btim_flag == 1 && flag == 0)			
				{
					Ininavimain(lat, lon, elv);
					flag = 1;
				}
				if(flag == 1)
				{
//					 if(lat > 40)
//				 {
//					while(1);
//				 }
					navimain(g_output_info.angle_rate.x, -g_output_info.angle_rate.z, g_output_info.angle_rate.y, g_output_info.accel.x, -g_output_info.accel.z, g_output_info.accel.y, lat, lon, elv, &gnss_btim_flag);
					
				}
				sd_open_send();

				/* �˹�д������ ����Ҫ���Ǻ���Ļ�����*/
				
				/* �������ⷢ�͵�������һ�ε����� ǰ���6��imu��������һʱ�̣������6����imu��������һʱ�� ���ߵ����������Ҳ����һʱ��*/
						if (res == 0)                                                                      /* ��־λ�ж� */
						{    
							lwip_udp_senddata(udppcb);                                                       /* ��ʼͨ���������ⷢ������ */
//								lwip_periodic_handle();                                                        /* LWIP��ѯ���� */
						}						
			 }
					
		}				

	
 
  /* USER CODE END imu_process */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/* Callback02 function */
void Callback02(void const * argument)
{
  /* USER CODE BEGIN Callback02 */
//		if(gnss_info_data.gnss_parse_end_flag || imu_info_data.imu_parse_end_flag)
//		show_imu_data();                                                                   /* lcd��ʾ����ˢ�� */
  /* USER CODE END Callback02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */



/* USER CODE END Application */
