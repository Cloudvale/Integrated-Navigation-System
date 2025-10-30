#ifndef __LWIP_DATA_H__
#define __LWIP_DATA_H__
#include "stdint.h"


typedef struct _udp_send_data
	{
		uint8_t udp_header;
		double  PDOPS;
		double  HDOPS;
		double  VDOPS;
		double  TDOPS;
		double  GDOPS;
		double  Lons;
		double  Lats;
		double  Alts;
		double  V_es;
		double  V_ns;
		double  V_us;
		/*总共 */
		
		double  ac_x;
		double  ac_y;
		double  ac_z;
		double  ang_r_x;
		double  ang_r_y;
		double  ang_r_z;
		/*总共*/
		
	}udp_send_out;

//extern udp_send udp_data;
	
extern char send_udp_str[400];
extern char send_udp_str_2[500];
extern struct Result Re1;
void lwip_send_data();	
void num_assign(); 

#endif