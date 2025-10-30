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


extern char sd_send_imu_gnss[512];            /* sd卡向外存储时的字符串变量 */
//extern char sd_send_imu[360];                 /* 备用-sd卡向外存储时的字符串变量 */

//extern _m_use_fattester use_fattester;        /* FATFS使用结构体 */
extern int32_t files_num;                     /* sd卡内txt文件数量 */
extern char sd_send_title[200];
extern uint32_t num;                          /* sd卡内保存至txt文件中的数据存储序号 */


extern uint32_t total;                        /* sd卡初始化参数 */
extern uint32_t frees;                        /* sd卡初始化参数 */
	
extern UINT brr, bww;	
extern 	uint32_t ret;
extern struct Result Re1;
extern nmeaINFO info;
extern struct GNSSDATA GNSS;
extern volatile double lat;
extern volatile double lon;
extern volatile double elv;
//	uint8_t t = 0;


uint8_t sd_scan_files();                      /* sd卡内文件扫描 */
void sd_open_send();                          /* sd卡保存数据 */
void lcd_show_init();                         /* lcd数据的初始化（imu，gnss等） */

#endif