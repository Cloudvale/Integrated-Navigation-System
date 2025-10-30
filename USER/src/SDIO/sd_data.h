#ifndef  __SD_DATA_H__
#define  __SD_DATA_H__

//#include "../USER/src/FATFS/exfuns/fattester.h"
//#include "../USER/src/FATFS/source/ff.h"
#include "../USER/src/GNSS/gnss_solve.h"
#include "../USER/src/IMU/imu_data.h"
#include "../USER/src/LCD/lcd.h"
#include "../USER/src/RTC/rtcs.h"
#include "../USER/src/INS/INS.h"
#include "../USER/src/KEY/key.h"
#include "../USER/src/TIMER/btim.h"
#include "stdint.h"
#include "stdio.h"
#include "ff.h"

#define  SIZE_SD_STORE      100*1024*1024
#define LWIP_SEND_START      1


extern char sd_send_imu_gnss[512];            /* sd������洢ʱ���ַ������� */
//extern char sd_send_imu[360];                 /* ����-sd������洢ʱ���ַ������� */

//extern _m_use_fattester use_fattester;        /* FATFSʹ�ýṹ�� */
extern int32_t files_num;                     /* sd����txt�ļ����� */
extern char sd_send_title[200];
extern uint32_t num;                          /* sd���ڱ�����txt�ļ��е����ݴ洢��� */


extern uint32_t total;                        /* sd����ʼ������ */
extern uint32_t frees;                        /* sd����ʼ������ */
	
extern UINT brr, bww;	
extern 	uint32_t ret;
extern struct Result Re1;
extern nmeaINFO info;
extern struct GNSSDATA GNSS;
extern volatile double lat;
extern volatile double lon;
extern volatile double elv;
//	uint8_t t = 0;


uint8_t sd_scan_files();                      /* sd�����ļ�ɨ�� */
void sd_open_send();                          /* sd���������� */
void lcd_show_init();                         /* lcd���ݵĳ�ʼ����imu��gnss�ȣ� */

#endif