#include "sd_data.h"
#include "string.h"
#include "sdio.h"
#include "usart.h"
#include "led.h"
#include "fatfs.h"


char files_same_str[10];                  /* 储存当某个txt文件大于设定容量时，续接上的txt文件的末尾序号 */
char send_names[50];                      /* 存储文件保存时rtc时间命名，包含尾部txt文件 */
char send_names_before[50];               /* 存储文件保存时rtc时间命名，不包含尾部txt文件 */

char sd_send_imu_gnss[512];               /* sd卡向外存储时的字符串变量 */
//char sd_send_imu[360];                    /* 备用-sd卡向外存储时的字符串变量*/

//_m_use_fattester use_fattester;           /* FATFS使用结构体 */

uint32_t num;                             /* sd卡内保存至txt文件中的数据存储序号 */
int32_t files_num;                        /* sd卡内txt文件数量 */

uint32_t total;                           /* sd卡初始化参数 */
uint32_t frees;                           /* sd卡初始化参数 */

int files_same_num=0;	                    /* 相同时间txt保存文件续接数量 */


UINT brr, bww;

uint32_t num_rr;

char sd_send_title[200]="% num  PDOP  HDOP  VDOP  TDOP  GDOP  Lon        Lat    Heig    Evel  Nvel  Uvel  heading    ac_x   ac_y   ac_z   ar_x   ar_y   ar_z   sig \n";

int timerr = 0;

uint8_t sd_scan_files()
{
	  uint8_t result;
	

		f_open(&SDFile,"0:\125.txt",FA_CREATE_ALWAYS|FA_WRITE|FA_OPEN_APPEND); 
//		files_num=use_mf_scan_files("0:");                                                      /* 扫描sd卡内部文件，并与设置的文件进行遍历 */
//		if(files_num>0)                                                                        /* 如果结果不为1，代表没有设置的文件，并重新创建并打开 */
//		{
//			rtc_get_time(&hour, &min, &sec, &ampm);
//			rtc_get_date(&year, &month, &date, &week);
//			sprintf(send_names,"0:/%d_20%02d-%02d-%02d-%02d-%02d-%02d",files_num,year,month,date,hour,min,sec);
//			memcpy(send_names_before,send_names,sizeof(send_names));
//			strcat(send_names,".txt");
//			

//			num=0;		
//		}

	return result;
}

void sd_open_send()
{
	uint8_t result;
	uint32_t num_bit;
	

	
//	sprintf(sd_send_imu_gnss,"%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%d,%d,%f,%u,%.6lf,%.6lf,%.4lf,%.6lf,%.6lf,%.6lf,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf\n",\
//			ins.eth.pos.i*r2d,ins.eth.pos.j*r2d,ins.eth.pos.k,ins.eth.vn.i,ins.eth.vn.j,ins.eth.vn.k,ins.att.i*r2d,ins.att.j*r2d,ins.att.k*r2d,g_output_info.accel.x,g_output_info.accel.y,g_output_info.accel.z,g_output_info.angle_rate.x,g_output_info.angle_rate.y,g_output_info.angle_rate.z,0,0,0,0,0,0,0.0,g_output_info.data_ready_timestamp,info.lon,info.lat,info.Height,info.EastVel,info.NorthVel,info.UpVel,0.0,0.0,0.0,0.0,0.0,0.0,info.PDOP,info.HDOP,info.TDOP,info.VDOP,info.GDOP,info.heading);

	
//	sprintf(sd_send_imu_gnss,"0x55,0x55,0xAA,0XAA,0X12,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%d,%d,%d,%d\n",\
//      g_output_info.accel.x,g_output_info.accel.y,g_output_info.accel.z,\
//      g_output_info.angle_rate.x,g_output_info.angle_rate.y,g_output_info.angle_rate.z,\
//	    g_output_info.sample_timestamp,g_output_info.data_ready_timestamp,headers.tid,imu_rec_failed);
	
//	if(lat > 40)
//				 {
//					while(1);
//				 }
	
	sprintf(sd_send_imu_gnss,"%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.5lf,%.5lf,%.5lf,%d,%d,%d\n",\
			g_output_info.accel.x, -g_output_info.accel.z, g_output_info.accel.y,\
			g_output_info.angle_rate.x, -g_output_info.angle_rate.z, g_output_info.angle_rate.y,\
			Re1.Lati, Re1.Longi, Re1.High, Re1.pitch, Re1.roll, Re1.yaw, Re1.VE, Re1.VN, Re1.VU,\
			lat, lon, elv,\
	    g_output_info.sample_timestamp,g_output_info.data_ready_timestamp,headers.tid);

//	timerr = (info.utc.hour*3600 + info.utc.min*60+info.utc.sec)*100+info.utc.hsec;
//	
//	sprintf(sd_send_imu_gnss,"%.5lf,%.5lf,%.5lf,%d,%d,%d\n",\
//			lat, lon, elv,\
//	    g_output_info.sample_timestamp,g_output_info.data_ready_timestamp,timerr);
		result = f_write(&SDFile,sd_send_imu_gnss,strlen(sd_send_imu_gnss),&num_rr);
		ret++;

	
	 
	 if(key_scan(0)==KEY0_PRES)
	 { 
		imu_btim_flag=0;
    gnss_btim_flag=0;
		LED0(1);                                                                      /* 保存结束，关闭信号灯 */
		LED1(1);                                                                      /* 保存结束，关闭信号灯 */
		f_close(&SDFile);                                                   				/* 关闭文件 */
	 }
}


void lcd_show_init()
{
	lcd_show_string(550,30,200,40,32,"GPS Follows:",BLACK);
	lcd_show_string(550,70,200,30,24,"PDOP:",BLACK);
	lcd_show_string(550,100,200,30,24,"HDOP:",BLACK);
	lcd_show_string(550,130,200,30,24,"VDOP:",BLACK);
	lcd_show_string(550,160,200,30,24,"TDOP:",BLACK);
	lcd_show_string(550,190,200,30,24,"GDOP:",BLACK);
	lcd_show_string(550,220,200,30,24,"Lon:",BLACK);
	lcd_show_string(550,250,200,30,24,"Lat:",BLACK);
	lcd_show_string(550,280,200,30,24,"Alt:",BLACK);
	lcd_show_string(550,310,200,30,24,"V-e:",BLACK);
	lcd_show_string(550,340,200,30,24,"V-n:",BLACK);
	lcd_show_string(550,370,200,30,24,"V-u:",BLACK);
	lcd_show_string(550,400,200,30,24,"heading:",BLACK);
	lcd_show_string(550,430,200,30,24,"SIG:",BLACK);
	
	lcd_draw_line(540,0,540,500,BLACK);
	
	lcd_show_string(10,30,300,40,32,"Status Follows:",BLACK);
	lcd_show_string(10,70,200,30,24,"GNSS:",BLACK);
	lcd_show_string(10,100,200,30,24,"IMU:",BLACK);
	lcd_show_string(10,130,200,30,24,"LWIP:",BLACK);
	lcd_draw_hline(0,157,540,BLACK);
	
	lcd_show_string(10,160,300,40,32,"Combined State:",BLACK);
	lcd_show_string(10,200,200,30,24,"Speed:",BLACK);
	lcd_show_string(10,230,200,30,24,"Position:",BLACK);
	lcd_show_string(10,260,200,30,24,"Posture:",BLACK);

	
	
	lcd_draw_hline(0,330,540,BLACK);
	lcd_show_string(10,340,300,40,32,"IMU Follows:",BLACK);
	lcd_show_string(10,380,300,30,24,"ac.x:",BLACK);
	lcd_show_string(10,410,300,30,24,"ac.y:",BLACK);
	lcd_show_string(10,440,300,30,24,"ac.z:",BLACK);
	
	lcd_show_string(300,380,300,30,24,"ang_r.x:",BLACK);
	lcd_show_string(300,410,300,30,24,"ang_r.y:",BLACK);
	lcd_show_string(300,440,300,30,24,"ang_r.z:",BLACK);
	
	
	lcd_show_string(10+12*5,70,200,30,24,"Init",BLUE);
	lcd_show_string(10+12*4,100,200,30,24,"Init",BLUE);
	lcd_show_string(10+12*5,130,200,30,24,"Init",BLUE);
}