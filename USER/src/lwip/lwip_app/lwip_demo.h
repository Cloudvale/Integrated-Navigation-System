/**
 ****************************************************************************************************
 * @file        lwip_demo.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-04
 * @brief       lwIP RAW�ӿ�UDPʵ��
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ̽���� F407������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */
 
#ifndef _LWIP_DEMO_H
#define _LWIP_DEMO_H
#include "sys.h"
#include "stdint.h"


#define LWIP_SEND_DATA              0X80    /* ���������ݷ��� */
extern uint8_t lwip_send_flag;              /* UDP���ݷ��ͱ�־λ */

extern uint8_t g_lwip_demo_recvbuf[1000]; 
extern uint8_t g_lwip_send_flag;

extern uint8_t res;
extern uint8_t t;
extern struct udp_pcb *udppcb;
void lwip_demo(void);

#endif /* _CLIENT_H */
