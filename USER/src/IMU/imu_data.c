#include "imu_data.h"

/* IMU传感器数据 */
protocol_info_t g_output_info = {0};   /* IMU数据解析接收器 */
protocol_info_t g_output_info_store = {0};   /* IMU数据解析接收器备份 */
imuINFO imu_info_data;
imuINFO imu_info_data_store;
show_imu_str show_imu_data_str;
//ins_init_data ins_info_init;
ins_sov_param ins_params_sov;

uint8_t time_acc; 

uint8_t hour, min, sec, ampm;
uint8_t year, month, date, week;
uint8_t tbuf[100];



double g_output_info_ac_x;
double g_output_info_ac_y;
double g_output_info_ac_z;

double g_output_info_an_x;
double g_output_info_an_y;
double g_output_info_an_z;



uint8_t ones_test;
int imu_rec_failed=0;



void show_imu_data(void)
{

	
			/* imu数据接收显示 */
			sprintf(show_imu_data_str.show_imu_str_ac_x,"%.3lf",g_output_info.accel.x);
			sprintf(show_imu_data_str.show_imu_str_ac_y,"%.3lf",g_output_info.accel.y);
			sprintf(show_imu_data_str.show_imu_str_ac_z,"%.3lf",g_output_info.accel.z);			
			sprintf(show_imu_data_str.show_imu_str_ang_r_x,"%.3lf",g_output_info.angle_rate.x);		
			sprintf(show_imu_data_str.show_imu_str_ang_r_y,"%.3lf",g_output_info.angle_rate.y);
			sprintf(show_imu_data_str.show_imu_str_ang_r_z,"%.3lf",g_output_info.angle_rate.z);
			
			if(!show_imu_data_str.imu_ok_show)                                                                    /*“ok”显示，减轻刷新频率  */
			{
				show_imu_data_str.imu_ok_show++;
				lcd_show_string(10+12*8,100,100,30,24," OK",BLUE);
			}
			
			lcd_show_string(10+12*5,380,100,30,24,show_imu_data_str.show_imu_str_ac_x,BLUE);
			lcd_show_string(10+12*5,410,100,30,24,show_imu_data_str.show_imu_str_ac_y,BLUE);
			lcd_show_string(10+12*5,440,100,30,24,show_imu_data_str.show_imu_str_ac_z,BLUE);	
			lcd_show_string(300+12*6,380,100,30,24,show_imu_data_str.show_imu_str_ang_r_x,BLUE);
			lcd_show_string(300+12*6,410,100,30,24,show_imu_data_str.show_imu_str_ang_r_y,BLUE);
			lcd_show_string(300+12*6,440,100,30,24,show_imu_data_str.show_imu_str_ang_r_z,BLUE);
			
			
			if(!gnss_lcd_show.show_str_ok) 
			{
				gnss_lcd_show.show_str_ok++;
				lcd_show_string(10+12*9,70,50,30,24," OK",BLUE);                           		/* “ok”显示，减轻刷新频率 */
			}
//			sprintf(gnss_lcd_show.show_str_PDOP,"%.1lf",gnss_info_data.info.PDOP);
//			sprintf(gnss_lcd_show.show_str_HDOP,"%.1lf",gnss_info_data.info.HDOP);
//			sprintf(gnss_lcd_show.show_str_VDOP,"%.1lf",gnss_info_data.info.VDOP);
//			sprintf(gnss_lcd_show.show_str_TDOP,"%.4lf",gnss_info_data.info.TDOP);
//			sprintf(gnss_lcd_show.show_str_GDOP,"%.4lf",gnss_info_data.info.GDOP);
//			sprintf(gnss_lcd_show.show_str_Lon,"%.3lf",gnss_info_data.info.lon);
//			sprintf(gnss_lcd_show.show_str_Lat,"%.3lf",gnss_info_data.info.lat);
//			sprintf(gnss_lcd_show.show_str_Alt,"%.3lf",gnss_info_data.info.Height);
//			sprintf(gnss_lcd_show.show_str_V_e,"%.3lf",gnss_info_data.info.EastVel);
//			sprintf(gnss_lcd_show.show_str_V_n,"%.3lf",gnss_info_data.info.NorthVel);
//			sprintf(gnss_lcd_show.show_str_V_u,"%.3lf",gnss_info_data.info.UpVel);
//			sprintf(gnss_lcd_show.show_str_sig,"%d",gnss_info_data.info.sig);
//			sprintf(gnss_lcd_show.show_str_heading,"%.4lf",gnss_info_data.info.heading);
		
//			lcd_show_string(550+12*5,70,50,30,24,gnss_lcd_show.show_str_PDOP,BLUE);	
//			lcd_show_string(550+12*5,100,50,30,24,gnss_lcd_show.show_str_HDOP,BLUE);
//			lcd_show_string(550+12*5,130,50,30,24,gnss_lcd_show.show_str_VDOP,BLUE);
//			lcd_show_string(550+12*5,160,200,30,24,gnss_lcd_show.show_str_TDOP,BLUE);
//			lcd_show_string(550+12*5,190,200,30,24,gnss_lcd_show.show_str_GDOP,BLUE);
//			lcd_show_string(550+12*4,220,300,30,24,gnss_lcd_show.show_str_Lon,BLUE);
//			lcd_show_string(550+12*4,250,300,30,24,gnss_lcd_show.show_str_Lat,BLUE);
//			lcd_show_string(550+12*4,280,200,30,24,gnss_lcd_show.show_str_Alt,BLUE);
//			lcd_show_string(550+12*4,310,100,30,24,gnss_lcd_show.show_str_V_e,BLUE);
//			lcd_show_string(550+12*4,340,100,30,24,gnss_lcd_show.show_str_V_n,BLUE);
//			lcd_show_string(550+12*4,370,100,30,24,gnss_lcd_show.show_str_V_u,BLUE);
//			lcd_show_string(550+12*8,400,100,30,24,gnss_lcd_show.show_str_heading,BLUE);
//			lcd_show_string(550+12*4,430,100,30,24,gnss_lcd_show.show_str_sig,BLUE);
			
			
//			rtc_get_time(&hour, &min, &sec, &ampm);
//			rtc_get_date(&year, &month, &date, &week);	
//			sprintf((char *)tbuf, "Date:20%02d-%02d-%02d,Time:%02d:%02d:%02d\n", year, month, date, hour, min, sec);
//			lcd_show_string(10, 290, 400, 30, 24, (char*)tbuf, BLACK);

			
}

void ins_init_param(ins_init_data *info,int nss, double t_1,double tkss0,double T)
{
//	info->n=nss;
//	info->t_1=t_1;
//	info->tk0=tkss0;
//	info->T=T;
//	info->pos0.i=0;
//	info->pos0.j=0;
//	info->pos0.k=0;
//	info->vn0.i=0;
//	info->vn0.j=0;
//	info->vn0.k=0;
//	memset(&info->vn0,0,sizeof(struct CVect3));
//	memset(&info->pos0,0,sizeof(struct CVect3));
}

void imu_zero_INFO(imuINFO *info)
{
	memset(&imu_info_data,0,sizeof(imuINFO));             //对所有数据进行初始化
	memset(&show_imu_data_str,0,sizeof(show_imu_str));             //对所有数据进行初始化
	memset(&ins_params_sov,0,sizeof(ins_sov_param));
}

