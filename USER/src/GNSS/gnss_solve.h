#ifndef __GNSS_SLOVE_H__
#define __GNSS_SLOVE_H__

#include "../USER/include/info.h"
#include "../USER/include/parser.h"
#include "../USER/include/gmath.h"
#include "../USER/src/LCD/lcd.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"

/******************************************************************************************/

#define READ_NUM  200
#define READ_OFF  0
#define BUFFER_SIZE  420

/******************************************************************************************/

typedef struct _GnssINFO{
	
  nmeaINFO info;                     /* GNSS解析数据存储信息区 */
	nmeaPOS dpos;                      /* 经纬度数据--度转弧度 */
	nmeaPARSER parser;                 /* GNSS数据解析器 */
	
	uint8_t rx_buffer[BUFFER_SIZE];  	 /* 接收数据数组 */
	uint8_t rx_buffer_f[BUFFER_SIZE];  /* 数据接收缓冲区数组 */
	uint8_t rx_parse[READ_NUM];                 /* 数据划块接收 */  
	
	uint8_t gnss_parse_end_flag;       /* gnss数据解析完成标志 */
	uint8_t recv_end_flag; 	           /* 一帧数据接收完成标志 */
	uint32_t rx_len;  				         /* 接收一帧数据的长度 */
	uint32_t read_end_off;             /* 读取结束清零标志 */
	
}gnssINFO;


typedef struct _Show_data_str{
	
	char show_str_PDOP[10];         /* PDOP数据转字符串发送数组 */
	char show_str_HDOP[10];					/* HDOP数据转字符串发送数组 */
	char show_str_VDOP[10];					/* VDOP数据转字符串发送数组 */
	char show_str_TDOP[10];					/* TDOP数据转字符串发送数组 */
	char show_str_GDOP[10];					/* GDOP数据转字符串发送数组 */
	char show_str_Lon[14];					/* Lon数据转字符串发送数组 */
	char show_str_Lat[14];					/* Lat数据转字符串发送数组 */
	char show_str_Alt[10];					/* Alt数据转字符串发送数组 */
	char show_str_V_e[10];					/* V_e数据转字符串发送数组 */
	char show_str_V_n[10];					/* V_n数据转字符串发送数组 */
	char show_str_V_u[10];					/* V_u数据转字符串发送数组 */
	char show_str_heading[10];			/* heading数据转字符串发送数组 */
	char show_str_sig[2];           /* sig数据转字符串发送数组 */
	char show_str_ok;	              /* OK字符更新标志位 */

}show_gnss_str;


/******************************************************************************************/
extern gnssINFO gnss_info_data;
extern show_gnss_str gnss_lcd_show;
extern nmeaINFO info;                     /* GNSS解析数据存储信息区 */
extern nmeaPARSER parser;                 /* GNSS数据解析器 */

void show_gnss_data(void);            /* 显示gnss解析出来的数据 */
void gnss_zero_INFO(gnssINFO *info);  /* 对gnss参数进行初始化 */

#endif