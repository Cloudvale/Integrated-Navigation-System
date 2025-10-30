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
	
  nmeaINFO info;                     /* GNSS�������ݴ洢��Ϣ�� */
	nmeaPOS dpos;                      /* ��γ������--��ת���� */
	nmeaPARSER parser;                 /* GNSS���ݽ����� */
	
	uint8_t rx_buffer[BUFFER_SIZE];  	 /* ������������ */
	uint8_t rx_buffer_f[BUFFER_SIZE];  /* ���ݽ��ջ��������� */
	uint8_t rx_parse[READ_NUM];                 /* ���ݻ������ */  
	
	uint8_t gnss_parse_end_flag;       /* gnss���ݽ�����ɱ�־ */
	uint8_t recv_end_flag; 	           /* һ֡���ݽ�����ɱ�־ */
	uint32_t rx_len;  				         /* ����һ֡���ݵĳ��� */
	uint32_t read_end_off;             /* ��ȡ���������־ */
	
}gnssINFO;


typedef struct _Show_data_str{
	
	char show_str_PDOP[10];         /* PDOP����ת�ַ����������� */
	char show_str_HDOP[10];					/* HDOP����ת�ַ����������� */
	char show_str_VDOP[10];					/* VDOP����ת�ַ����������� */
	char show_str_TDOP[10];					/* TDOP����ת�ַ����������� */
	char show_str_GDOP[10];					/* GDOP����ת�ַ����������� */
	char show_str_Lon[14];					/* Lon����ת�ַ����������� */
	char show_str_Lat[14];					/* Lat����ת�ַ����������� */
	char show_str_Alt[10];					/* Alt����ת�ַ����������� */
	char show_str_V_e[10];					/* V_e����ת�ַ����������� */
	char show_str_V_n[10];					/* V_n����ת�ַ����������� */
	char show_str_V_u[10];					/* V_u����ת�ַ����������� */
	char show_str_heading[10];			/* heading����ת�ַ����������� */
	char show_str_sig[2];           /* sig����ת�ַ����������� */
	char show_str_ok;	              /* OK�ַ����±�־λ */

}show_gnss_str;


/******************************************************************************************/
extern gnssINFO gnss_info_data;
extern show_gnss_str gnss_lcd_show;
extern nmeaINFO info;                     /* GNSS�������ݴ洢��Ϣ�� */
extern nmeaPARSER parser;                 /* GNSS���ݽ����� */

void show_gnss_data(void);            /* ��ʾgnss�������������� */
void gnss_zero_INFO(gnssINFO *info);  /* ��gnss�������г�ʼ�� */

#endif