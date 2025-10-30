#include "gnss_solve.h"



/* USER CODE BEGIN PV */

gnssINFO gnss_info_data;
show_gnss_str gnss_lcd_show;
nmeaINFO info;                     /* GNSS解析数据存储信息区 */
nmeaPARSER parser;                 /* GNSS数据解析器 */
/* USER CODE END PV */


void show_gnss_data(void)
{
	if(!gnss_lcd_show.show_str_ok) 
			{
				gnss_lcd_show.show_str_ok++;
				lcd_show_string(10+12*9,70,50,30,24," OK",BLUE);                           		/* “ok”显示，减轻刷新频率 */
			}
				sprintf(gnss_lcd_show.show_str_PDOP,"%.1lf",gnss_info_data.info.PDOP);
				sprintf(gnss_lcd_show.show_str_HDOP,"%.1lf",gnss_info_data.info.HDOP);
				sprintf(gnss_lcd_show.show_str_VDOP,"%.1lf",gnss_info_data.info.VDOP);
				sprintf(gnss_lcd_show.show_str_TDOP,"%.4lf",gnss_info_data.info.TDOP);
				sprintf(gnss_lcd_show.show_str_GDOP,"%.4lf",gnss_info_data.info.GDOP);
				sprintf(gnss_lcd_show.show_str_Lon,"%.3lf",gnss_info_data.info.lon);
				sprintf(gnss_lcd_show.show_str_Lat,"%.3lf",gnss_info_data.info.lat);
				sprintf(gnss_lcd_show.show_str_Alt,"%.3lf",gnss_info_data.info.Height);
				sprintf(gnss_lcd_show.show_str_V_e,"%.3lf",gnss_info_data.info.EastVel);
				sprintf(gnss_lcd_show.show_str_V_n,"%.3lf",gnss_info_data.info.NorthVel);
				sprintf(gnss_lcd_show.show_str_V_u,"%.3lf",gnss_info_data.info.UpVel);
			
				lcd_show_string(550+12*5,70,50,30,24,gnss_lcd_show.show_str_PDOP,BLUE);	
				lcd_show_string(550+12*5,100,50,30,24,gnss_lcd_show.show_str_HDOP,BLUE);
				lcd_show_string(550+12*5,130,50,30,24,gnss_lcd_show.show_str_VDOP,BLUE);
				lcd_show_string(550+12*5,160,200,30,24,gnss_lcd_show.show_str_TDOP,BLUE);
				lcd_show_string(550+12*5,190,200,30,24,gnss_lcd_show.show_str_GDOP,BLUE);
				lcd_show_string(550+12*4,220,300,30,24,gnss_lcd_show.show_str_Lon,BLUE);
				lcd_show_string(550+12*4,250,300,30,24,gnss_lcd_show.show_str_Lat,BLUE);
				lcd_show_string(550+12*4,280,200,30,24,gnss_lcd_show.show_str_Alt,BLUE);
				lcd_show_string(550+12*4,310,100,30,24,gnss_lcd_show.show_str_V_e,BLUE);
				lcd_show_string(550+12*4,340,100,30,24,gnss_lcd_show.show_str_V_n,BLUE);
				lcd_show_string(550+12*4,370,100,30,24,gnss_lcd_show.show_str_V_u,BLUE);
}

void gnss_zero_INFO(gnssINFO *info)
{
	memset(&gnss_info_data,0,sizeof(gnssINFO));             //对所有数据进行初始化
	memset(&gnss_lcd_show,0,sizeof(show_gnss_str));             //对显示的数据进行初始化
}