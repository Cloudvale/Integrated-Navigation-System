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

//FATFS fs;                       /* FatFs 文件系统对象 */
//FIL file;                       /* 文件对象 */
FRESULT f_res;                  /* 文件操作结果 */
UINT fnum;                      /* 文件成功读写数量 */
BYTE ReadBuffer[1024] = {0};    /* 读缓冲区 */
BYTE WriteBuffer[300] = {0};            /* 写缓冲区 */

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
 * @breif       加载UI
 * @param       mode :  bit0:0,不加载;1,加载前半部分UI
 *                      bit1:0,不加载;1,加载后半部分UI
 * @retval      无
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
	
//  sys_stm32_clock_init(336, 8, 2, 7); /* 设置时钟,168Mhz */
	led_init();                                                                              /* 初始化LED */
	sram_init();                                                                             /* 初始化外部SRAM */
	usart_init(115200);                                                                      /* 串口2初始化为115200 */
//  btim_timx_int_init(10000 - 1, 8400 - 1); /* 84 000 000 / 84 00 = 10 000 10Khz的计数频率，计数10K次为1s */
	
	my_mem_init(SRAMIN);                                                                     /* 初始化内部SRAM内存池 */
	my_mem_init(SRAMEX);                                                                     /* 初始化外部SRAM内存池 */
	my_mem_init(SRAMCCM);                                                               		 /* 初始化CCM内存池 */
	  	
	 while (sd_init())                                                                       /* 检测不到SD卡 */
	{
		lcd_show_string(30, 130, 300, 32, 32, "SD Card Error!", RED);                          /* 进入循环并判断，  MX_SDIO_SD_Init()已做判断，并未有效 */
//		HAL_Delay(500);
		lcd_show_string(30, 130, 300, 32, 32, "Please Check! ", RED);
//		HAL_Delay(500);
		LED0_TOGGLE();                                                                         
	}
	  lcd_fill(30, 130, 240+30, 130 + 32, WHITE); 
	
	
/*************************************sd卡的FATFS-BEGIN设置**************************************************************************************************/		
//	exfuns_init();                                                                           /* 为fatfs相关变量申请内存 */
//  f_mount(fs[0], "0:", 1);                                                                 /* 挂载SD卡 */
//	res = f_mount(fs[1], "1:", 1);                                                           /* 挂载FLASH */

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
//  if (res == 0X0D)                                                                         /* FLASH磁盘,FAT文件系统错误,重新格式化FLASH */
//  {
//		
//		 lcd_show_string(30, 130, 300, 32, 32, "Flash Disk Formatting...", RED);                 /* 格式化FLASH */
//		 res = f_mkfs("1:", 0, 512,0, _MAX_SS);                                                    /* 格式化FLASH,1:,盘符;0,使用默认格式化参数 */

//		 if (res == 0)
//		 {
////			 f_setlabel((const TCHAR *)"1:ALIENTEK");                                         /* 设置Flash磁盘的名字为：ALIENTEK */
//			 lcd_show_string(30, 130, 300, 32, 32, "Flash Disk Format Finish", RED);          /* 格式化完成 */
//		 }
//		 else 
//			 lcd_show_string(30, 130, 300, 32, 32, "Flash Disk Format Error ", RED);       /* 格式化失败 */

////		 HAL_Delay(1000);
//	 }

//   lcd_fill(30, 150, 240, 150 + 16, WHITE);                                               /* 清除显示 */

//   while (exfuns_get_free("0", &total, &frees))                                           /* 得到SD卡的总容量和剩余容量 */
//    {
//	 	  lcd_show_string(30, 150, 200, 16, 16, "SD Card Fatfs Error!", RED);               /* sd卡文件初始错误 */
//	 	  HAL_Delay(20);
//		  lcd_fill(30, 150, 240, 150 + 16, WHITE);                                           /* 清除显示 */
//		  HAL_Delay(20);
////		  LED0_TOGGLE();                                                                     /* LED0闪烁 */
//    }
/*************************************sd卡的FATFS-END设置*****************************************************************************************************/		
                                                                          
	
	lcd_show_init();                                                                        /* lcd数据显示初始化 */

	
/***************************************LWIP-BEGIN设置***********************************************************************************************/
		
//	lwip_test_ui(1);                    /* 加载前半部分UI */

//  lcd_show_string(300, 110, 200, 16, 16, "lwIP Init !!", BLUE);
  

#if   LWIP_SEND_START  		                                                        /* 网口数据发送宏定义  1为开启发送，0为关闭 */
  while (lwip_comm_init() != 0)                                       				    /* lwip加载失败 */
  {
    lcd_show_string(30, 130, 300, 32, 32, "lwIP Init failed!!", BLUE);
    HAL_Delay(20);
    lcd_fill(30, 130, 200 + 30, 130 + 32, WHITE);
    lcd_show_string(30, 130, 200, 32, 32, "Retrying...       ", BLUE);           /* 显示重新加载 */
    HAL_Delay(20);
//    LED1_TOGGLE();
    }

    while (!ethernet_read_phy(PHY_SR))                                            /* 检查MCU与PHY芯片是否通信成功 */
    {
 //       		printf("MCU与PHY芯片通信失败，请检查电路或者源码！！！！\r\n");
		lcd_show_string(30, 150, 300, 32, 32, "MCU to PHY failed！！！！", BLUE);           /* MCU与PHY芯片通信失败，请检查电路或者源码！！！！ */
		
    }
    
		
#if LWIP_DHCP                                                                     //
//    lcd_show_string(6, 130, 200, 16, 16, "DHCP IP configing... ", BLUE);           /* 开始DHCP */
    
    while ((g_lwipdev.dhcpstatus != 2) && (g_lwipdev.dhcpstatus != 0XFF))          /* 等待DHCP获取成功/超时溢出 */
    {
        lwip_periodic_handle();                                                    /* lwip轮询查找 */
        HAL_Delay(10);
    }	
#endif	
		
		lwip_demo();                                                                   /* lwIP程序入口 */
    
#endif

		
	key_time_init_selcet();                                                      //进行时间的矫正以及初始化判断
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能IDLE中断
	  HAL_UART_Receive_DMA(&huart1,gnss_info_data.rx_buffer,BUFFER_SIZE);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); //使能IDLE中断
	  HAL_UART_Receive_DMA(&huart3,imu_info_data.rx_imu_buff,IMU_READ_DATA);  //DMA接收函数，此句一定要加，不加接收不到第一次传进来的实数据，是空的，且此时接收到的数据长度为缓存器的数据长度
		
	LED0(1);
		
	vTaskSuspend(NULL);  // 挂起任务自身
/******************************************LWIP-END设置*********************************************************************************************/		
  /* Infinite loop */
		
		
  for(;;)
  {	 
		/* 数据处理 */                                                                      		/* 数据处理开始 */		
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
	osDelay(2000);                                                                /* 延时，让main_porcess进程初始化结束 */
//	vTaskSuspend(NULL);  // 挂起任务自身
	gnss_zero_INFO(&gnss_info_data);
	nmea_zero_INFO(&info);                                                       /* info信息初始化 */
  nmea_parser_init(&parser);																										/* parser解析器初始化 */
	osTimerStart(myTimer02Handle,2000);

	 uint8_t show_flag=0;
	 uint8_t show_key=0;
//  /* Infinite loop */
  for(;;)                                                                       /* GNSS_Task主循环 */
  {

//		gnss_info_data.recv_end_flag=0;
		
		int gnss_len = queue_len(q_gnss);
		if(gnss_len != 0)                                                           /* 接收标志位为1时，开始进行数据处理 */
		{
			 /* GNSS板卡数据处理 */
			memcpy(gnss_info_data.rx_buffer_f,gnss_info_data.rx_buffer,BUFFER_SIZE);                                 	/* copy到数据缓冲区 */
			while(gnss_info_data.read_end_off<BUFFER_SIZE)                                                               	/* 数据接收后循环处理 */ 
			 {
				memcpy(gnss_info_data.rx_parse,gnss_info_data.rx_buffer_f+gnss_info_data.read_end_off,READ_NUM);      	    /* 数据存入缓冲区 */
				queue_de(&q_gnss, e_gnss,GNSS_BUFFER_SIZE);
				nmea_parse(&parser, (char *)e_gnss, READ_NUM, &info); 				 /* 数据开始进行解析 */
				
				gnss_info_data.read_end_off+=READ_NUM;                                                                      /* 数据加载 */
			 }
					
			  gnss_info_data.read_end_off=READ_OFF;                                                    	/* 加载数字清零 */
			  gnss_info_data.rx_len = 0;                                                             		/* 清除计数 */
			  gnss_info_data.recv_end_flag = READ_OFF;                                               		/* 清除接收结束标志位 */
				if(info.lat!=0)
			  {
					gnss_btim_flag=1;
					gnss_info_data.gnss_parse_end_flag=1;                                                  		/* 处理完成标志 */
			  }
				if(info.sig==1)
				kal_cal_flag=1;
				

			 /* 2号天线在后，1号天线在前 */
			 
				/* gnss数据显示 */
//				show_gnss_data();
//         KalUpdate_d(&Kal, struct CEarth* eth, struct CVect3* vn, struct Quat* qnb, struct CMAT3* Cnb, struct CVect3* att);//导航量测更新

        /* 进行后验估计 */
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
	 double t_1=0.005;       //开始时间
	double tk0=0;       //采样时间
	double T=10;        //初对准时间
	int n=2;            //子样数
	
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

	osDelay(5000);                                                        /* 延时，让main_porcess进程初始化结束 */
//	ins_init_param(&ins_info_init,1,0.005,0,10);                             //初对准进行初始化，各项参数进行赋值
	sd_scan_files();                                                       /* 进行sd卡的里储存的扫描和初始化 */
	imu_zero_INFO(&imu_info_data);
	
////  /* Infinite loop */
  for(;;)
  {

 LED0(0);
		int imu_len = queue_len(q_imu);
		if(imu_len != 0)                                                                                /* 传输标志位判断 */
		{
		LED0(1);
			/* IMU传感器数据处理 */
			imu_info_data.recv_imu_end_flag=0;
			
				queue_de(&q_imu, e_imu,IMU_BUFFER_SIZE);  
				analysis_data(e_imu,IMU_RECV_DATA,&g_output_info);   			/* imu数据解析 */
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

				/* 人工写入数据 输入要的是航向的弧度制*/
				
				/* 网口向外发送的则是上一次的数据 前面的6个imu数据是上一时刻，后面的6干扰imu数据是下一时刻 ，惯导结算的数据也是上一时刻*/
						if (res == 0)                                                                      /* 标志位判断 */
						{    
							lwip_udp_senddata(udppcb);                                                       /* 开始通过网口向外发送数据 */
//								lwip_periodic_handle();                                                        /* LWIP轮询任务 */
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
//		show_imu_data();                                                                   /* lcd显示界面刷新 */
  /* USER CODE END Callback02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */



/* USER CODE END Application */
