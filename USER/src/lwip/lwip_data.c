#include "lwip_data.h"
#include "lwip_demo.h"
#include "gnss_solve.h"
#include "imu_data.h"
#include "stdint.h"
#include "stdio.h"
#include "navimain.h"

udp_send_out udp_data;
char send_udp_str[400];
char send_udp_str_2[500];

void lwip_send_data()
{
//	    if (res == 0)                                                                      /* 标志位判断 */
//    {    
//      lwip_udp_senddata(udppcb);                                                       /* 开始通过网口向外发送数据 */


//        if (g_lwip_send_flag & 1 << 6)                                                 /* 是否收到数据 */
//        {
//            lcd_fill(5, 230, lcddev.width - 1, lcddev.height - 1, WHITE);              /* 清上一次数据 */
//            /* 显示接收到的数据 */
//            lcd_show_string(6, 230, lcddev.width - 2, lcddev.height - 230, 16, (char *)g_lwip_demo_recvbuf, g_point_color); 
//            g_lwip_send_flag &= ~(1 << 6);                                             /* 标记数据已经被处理了 */
//            
//        }
//        lwip_periodic_handle();                                                        /* LWIP轮询任务 */
//        HAL_Delay(2);
//        t++;
//        if (t == 200)
//        {
//            t = 0;
//            LED0_TOGGLE();
//        }
//    }
}



void num_assign()
{
//	udp_data.udp_header=0x59;
//	udp_data.PDOPS=info.PDOP;
//	udp_data.HDOPS=info.HDOP;
//	udp_data.VDOPS=info.TDOP;
//	udp_data.TDOPS=info.VDOP;
//	udp_data.GDOPS=info.GDOP;
//	udp_data.Lons=info.lon;
//	udp_data.Lats=info.lat;
//	udp_data.Alts=info.Height;
//	udp_data.V_es=info.EastVel;
//	udp_data.V_ns=info.NorthVel;
//	udp_data.V_us=info.UpVel;
//	
//	udp_data.ac_x=g_output_info.accel.x;
//	udp_data.ac_y=g_output_info.accel.y;
//	udp_data.ac_z=g_output_info.accel.z;
//	udp_data.ang_r_x=g_output_info.angle_rate.x;
//	udp_data.ang_r_y=g_output_info.angle_rate.y;
//	udp_data.ang_r_z=g_output_info.angle_rate.z;
	/*暂时弃用*/

	
			Re1.Lati  = eth.pos.i*r2d;
			Re1.Longi = eth.pos.j*r2d;
			Re1.High  = (float)eth.pos.k;
			Re1.VE    = (float)eth.vn.i;
			Re1.VN    = (float)eth.vn.j;
			Re1.VU    = (float)eth.vn.k;
			Re1.pitch = (float)ins.att.i*r2d;
			Re1.roll  = (float)ins.att.j*r2d;
			Re1.yaw   = (float)ins.att.k*r2d;
			Re1.fx    = (float)ins.fb.i;
			Re1.fy    = (float)ins.fb.j;
			Re1.fz    = (float)ins.fb.k;
			Re1.gx    = (float)ins.wib.i*r2d;
			Re1.gy    = (float)ins.wib.j*r2d;
			Re1.gz    = (float)ins.wib.k*r2d;
			
	
	
     	sprintf(send_udp_str_2,"%d,%d,%d,%d,%d,%.8lf,%.8lf,%.8lf,%.8lf",\
			0x55,0x55,0xAA,0xAA,0x12,Re1.Lati,Re1.Longi,Re1.High,Re1.VE);
					
//			 	sprintf(send_udp_str_2,"%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.3lf,%.3lf,%.3lf,%.3lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.8lf,%.8lf,%.8lf,%.8lf\n",\
//			ins.eth.vn.i,ins.eth.vn.j,ins.eth.vn.k,ins.att.i*r2d,ins.att.j*r2d,ins.att.k*r2d,g_output_info.accel.x,g_output_info.accel.y,g_output_info.accel.z,g_output_info.angle_rate.x,g_output_info.angle_rate.y,g_output_info.angle_rate.z,info.lon,info.lat,info.Height,info.EastVel,info.NorthVel,info.UpVel,g_output_info_ac_x,g_output_info_ac_y,g_output_info_ac_z,g_output_info_an_x,g_output_info_an_y,g_output_info_an_z,ins.eth.wnin.i,ins.eth.wnin.j,ins.eth.wnin.k,ins.qnb.q0,ins.qnb.q1,ins.qnb.q2,ins.qnb.q3);
							
//			 	sprintf(send_udp_str_2,"%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.3lf,%.3lf,%.3lf,%.3lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.13lf,%.8lf,%.8lf,%.8lf,%.8lf,%u\n",\
			ins.eth.vn.i,ins.eth.vn.j,ins.eth.vn.k,ins.att.i*r2d,ins.att.j*r2d,ins.att.k*r2d,g_output_info.accel.x,g_output_info.accel.y,g_output_info.accel.z,g_output_info.angle_rate.x,g_output_info.angle_rate.y,g_output_info.angle_rate.z,info.lon,info.lat,info.Height,info.EastVel,info.NorthVel,info.UpVel,g_output_info_ac_x,g_output_info_ac_y,g_output_info_ac_z,g_output_info_an_x,g_output_info_an_y,g_output_info_an_z,ins.eth.wnin.i,ins.eth.wnin.j,ins.eth.wnin.k,ins.qnb.q0,ins.qnb.q1,ins.qnb.q2,ins.qnb.q3,g_output_info.sample_timestamp);


//			strcat(send_udp_str,send_udp_str_2);		
//	sprintf(send_udp_str,"# %.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.6lf,%.6lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%d,%d,%d,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf\n",\
//					info.PDOP,info.HDOP,info.TDOP,info.VDOP,info.GDOP,info.lon,info.lat,info.Height,info.EastVel,\
//					info.NorthVel,info.UpVel,info.heading,g_output_info.accel.x,g_output_info.accel.y,g_output_info.accel.z,\
//					g_output_info.angle_rate.x,g_output_info.angle_rate.y,g_output_info.angle_rate.z,g_output_info.sample_timestamp,g_output_info.data_ready_timestamp,headers.tid,ins.eth.vn.i,\
//			ins.eth.vn.j,ins.eth.vn.k,ins.eth.pos.i,ins.eth.pos.j,ins.eth.pos.k,ins.att.i,ins.att.j,ins.att.k);
//			
//	sprintf(send_udp_str,"#%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%d,%d,%d,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf,%.8lf\n",\
//      g_output_info.accel.x,g_output_info.accel.y,g_output_info.accel.z,\
//      g_output_info.angle_rate.x,g_output_info.angle_rate.y,g_output_info.angle_rate.z,g_output_info.sample_timestamp,g_output_info.data_ready_timestamp,headers.tid,ins.eth.vn.i,\
//			ins.eth.vn.j,ins.eth.vn.k,ins.eth.pos.i,ins.eth.pos.j,ins.eth.pos.k,ins.att.i,ins.att.j,ins.att.k);
//	
	
}