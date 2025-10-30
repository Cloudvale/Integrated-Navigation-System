#ifndef  __IMU_DATA_H__
#define  __IMU_DATA_H__

#include "../USER/src/IMU/imu_data.h"
#include "../USER/include/analysis_data.h"
#include "../USER/src/LCD/lcd.h"
#include "../USER/src/GNSS/gnss_solve.h"
#include "../USER/src/RTC/rtcs.h"
#include "../USER/src/INS/INS.h"

#include "stdint.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"

#define IMU_READ_DATA  51
#define IMU_RECV_DATA  51


typedef struct _ImuINFO{
	
	protocol_info_t g_output_info;               /* imu解析数据储存包 */

	uint8_t imu_parse_end_flag;                  /* imu数据解析完成标志位 */
	uint8_t rx_imu_buff[IMU_RECV_DATA];          /* imuDMA数据接收数组 */
	uint8_t rx_imu_buff_s[IMU_RECV_DATA];        /* imuDMA数据接收缓存区数组 */
	uint16_t num_add_accumulate;                  /* 先验计算标志位，100ms，20次进行一次计算 */

	uint32_t rx_imu_len;                         /* 接收数组长度 */
	uint32_t recv_imu_end_flag;                  /* imu数据接收标志位 */
	uint32_t data_tid;                           /* imu数据中TID标志位 */
	
	uint8_t time_acc;
	uint8_t countss;


}imuINFO;


typedef struct _ins_solve_param{
	
 /* 各项INS数据 */
//	struct STICA ica;


}ins_sov_param;



typedef struct _Show_imu_str{
	
	char show_imu_str_ac_x[20];
	char show_imu_str_ac_y[20];
	char show_imu_str_ac_z[20];
	char show_imu_str_ang_r_x[20];
	char show_imu_str_ang_r_y[20];
	char show_imu_str_ang_r_z[20];
	int imu_ok_show;
  uint8_t main_pro_ok;

}show_imu_str;



typedef struct _Ins_Init_data{

//	double t_1;       //开始时间
//	double tk0;       //采样时间
//	double T ;        //初对准时间
//	int n;            //子样数
	
//	struct CVect3 vn0;
//	struct CVect3 pos0;
//	struct Kalman Kal;
//	struct CVect3 pos0 = { 31.4667 * d2r,114.4 * d2r,6.6356 };

} ins_init_data;


extern double g_output_info_ac_x;
extern double g_output_info_ac_y;
extern double g_output_info_ac_z;

extern double g_output_info_an_x;
extern double g_output_info_an_y;
extern double g_output_info_an_z;

extern imuINFO imu_info_data;
extern imuINFO imu_info_data_store;
extern show_imu_str show_imu_data_str;
//extern ins_init_data ins_info_init;
extern ins_sov_param ins_params_sov;
extern protocol_info_t g_output_info;   /* IMU数据解析接收器 */
extern protocol_info_t g_output_info_store;   /* IMU数据解析接收器备份 */

extern uint8_t time_acc; 

extern int imu_rec_failed;
extern struct CVect3 ans;

//extern struct Kalman Kal;
//extern struct CVect3 Satt;
//extern struct CVect3 Svn;
//extern struct CVect3 Spos;
//extern struct CVect3 Sgyro;
//extern struct CVect3 Sacc;
//extern struct CVect3 SWg;
//extern struct CVect3 SWa;

extern struct Result Res;
//extern double pwm[1][3];
//extern double pvm[1][3];
extern struct CSINS ins;
extern struct STICA ica;

extern uint8_t hour, min, sec, ampm;
extern uint8_t year, month, date, week;
extern uint8_t tbuf[100];

extern uint8_t ones_test;

void show_imu_data(void);
void ins_init_param(ins_init_data *info,int n, double t_1,double tk0,double T);
void imu_zero_INFO(imuINFO *info);


#endif