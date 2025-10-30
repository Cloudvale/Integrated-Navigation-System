/**
 ****************************************************************************************************
 * @file        sdio_sdcard.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-11-05
 * @brief       SD�� ��������
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
 * �޸�˵��
 * V1.0 20211105
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#include "string.h"
#include "usarts.h"
#include "sdio_sdcard.h"
#include "sdio.h"


extern SD_HandleTypeDef hsd;            /* SD����� */
extern HAL_SD_CardInfoTypeDef g_sd_card_info_handle; /* SD����Ϣ�ṹ�� */

/**
 * @brief       ��ʼ��SD��
 * @param       ��
 * @retval      ����ֵ:0 ��ʼ����ȷ������ֵ����ʼ������
 */

uint8_t sd_init(void)
{ 
    uint8_t SD_Error;

    /* ��ʼ��ʱ��ʱ�Ӳ��ܴ���400KHZ */
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;                       /* ������ */
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;                  /* ��ʹ��bypassģʽ��ֱ����HCLK���з�Ƶ�õ�SDIO_CK */
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;           /* ����ʱ���ر�ʱ�ӵ�Դ */
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;                               /* 1λ������ */
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE; /* �ر�Ӳ������ */
    hsd.Init.ClockDiv = SDIO_TRANSF_CLK_DIV;                           /* SD����ʱ��Ƶ�����25MHZ */

    SD_Error = HAL_SD_Init(&hsd);
    if (SD_Error != HAL_OK)
    {
        return 1;
    }
//		HAL_MoreSD_MspInit(&hsd);                                                                /* SD���ٴγ�ʼ�� */
	
/* for test code */

//    HAL_SD_GetCardInfo(&hsd, &g_sd_card_info_handle);                  /* ��ȡSD����Ϣ */

//    SD_Error = HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B); /* ʹ��4bit������ģʽ */
//    if (SD_Error != HAL_OK)
//    {
//        return 2;
//    }
    
    return 0;
}


/**
 * @brief       SDMMC�ײ�������ʱ��ʹ�ܣ��������ã�DMA����
                �˺����ᱻHAL_SD_Init()����
 * @param       hsd:SD�����
 * @retval      ��
 */
void HAL_MoreSD_MspInit(SD_HandleTypeDef *hsd)
{
    if (hsd->Instance == SDIO)
    {
        DMA_HandleTypeDef TxDMAHandler, RxDMAHandler;
        GPIO_InitTypeDef gpio_init_struct;

        __HAL_RCC_SDIO_CLK_ENABLE();    /* ʹ��SDIOʱ�� */

        SD_D0_GPIO_CLK_ENABLE();        /* D0����IOʱ��ʹ�� */
        SD_D1_GPIO_CLK_ENABLE();        /* D1����IOʱ��ʹ�� */
        SD_D2_GPIO_CLK_ENABLE();        /* D2����IOʱ��ʹ�� */
        SD_D3_GPIO_CLK_ENABLE();        /* D3����IOʱ��ʹ�� */
        SD_CLK_GPIO_CLK_ENABLE();       /* CLK����IOʱ��ʹ�� */
        SD_CMD_GPIO_CLK_ENABLE();       /* CMD����IOʱ��ʹ�� */

        /* SD_D0����ģʽ���� */
        gpio_init_struct.Pin = SD_D0_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;            /* ���츴�� */
        gpio_init_struct.Pull = GPIO_PULLUP;                /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;      /* ���� */
        gpio_init_struct.Alternate = GPIO_AF12_SDIO;        /* ����ΪSDIO */
        HAL_GPIO_Init(SD_D0_GPIO_PORT, &gpio_init_struct);  /* ��ʼ�� */
        /* SD_D1����ģʽ���� */
        gpio_init_struct.Pin = SD_D1_GPIO_PIN;
        HAL_GPIO_Init(SD_D1_GPIO_PORT, &gpio_init_struct);  /* ��ʼ�� */
        /* SD_D2����ģʽ���� */
        gpio_init_struct.Pin = SD_D2_GPIO_PIN;
        HAL_GPIO_Init(SD_D2_GPIO_PORT, &gpio_init_struct);  /* ��ʼ�� */
        /* SD_D3����ģʽ���� */
        gpio_init_struct.Pin = SD_D3_GPIO_PIN;
        HAL_GPIO_Init(SD_D3_GPIO_PORT, &gpio_init_struct); /* ��ʼ�� */
        /* SD_CLK����ģʽ���� */
        gpio_init_struct.Pin = SD_CLK_GPIO_PIN;
        HAL_GPIO_Init(SD_CLK_GPIO_PORT, &gpio_init_struct); /* ��ʼ�� */
        /* SD_CMD����ģʽ���� */
        gpio_init_struct.Pin = SD_CMD_GPIO_PIN;
        HAL_GPIO_Init(SD_CMD_GPIO_PORT, &gpio_init_struct); /* ��ʼ�� */
    }
}

//void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
//{
//    if (hsd->Instance == SDIO)
//    {
//        DMA_HandleTypeDef TxDMAHandler, RxDMAHandler;
//        GPIO_InitTypeDef gpio_init_struct;

//        __HAL_RCC_SDIO_CLK_ENABLE();    /* ʹ��SDIOʱ�� */

//        SD_D0_GPIO_CLK_ENABLE();        /* D0����IOʱ��ʹ�� */
//        SD_D1_GPIO_CLK_ENABLE();        /* D1����IOʱ��ʹ�� */
//        SD_D2_GPIO_CLK_ENABLE();        /* D2����IOʱ��ʹ�� */
//        SD_D3_GPIO_CLK_ENABLE();        /* D3����IOʱ��ʹ�� */
//        SD_CLK_GPIO_CLK_ENABLE();       /* CLK����IOʱ��ʹ�� */
//        SD_CMD_GPIO_CLK_ENABLE();       /* CMD����IOʱ��ʹ�� */

//        /* SD_D0����ģʽ���� */
//        gpio_init_struct.Pin = SD_D0_GPIO_PIN;
//        gpio_init_struct.Mode = GPIO_MODE_AF_PP;            /* ���츴�� */
//        gpio_init_struct.Pull = GPIO_PULLUP;                /* ���� */
//        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;      /* ���� */
//        gpio_init_struct.Alternate = GPIO_AF12_SDIO;        /* ����ΪSDIO */
//        HAL_GPIO_Init(SD_D0_GPIO_PORT, &gpio_init_struct);  /* ��ʼ�� */
//        /* SD_D1����ģʽ���� */
//        gpio_init_struct.Pin = SD_D1_GPIO_PIN;
//        HAL_GPIO_Init(SD_D1_GPIO_PORT, &gpio_init_struct);  /* ��ʼ�� */
//        /* SD_D2����ģʽ���� */
//        gpio_init_struct.Pin = SD_D2_GPIO_PIN;
//        HAL_GPIO_Init(SD_D2_GPIO_PORT, &gpio_init_struct);  /* ��ʼ�� */
//        /* SD_D3����ģʽ���� */
//        gpio_init_struct.Pin = SD_D3_GPIO_PIN;
//        HAL_GPIO_Init(SD_D3_GPIO_PORT, &gpio_init_struct); /* ��ʼ�� */
//        /* SD_CLK����ģʽ���� */
//        gpio_init_struct.Pin = SD_CLK_GPIO_PIN;
//        HAL_GPIO_Init(SD_CLK_GPIO_PORT, &gpio_init_struct); /* ��ʼ�� */
//        /* SD_CMD����ģʽ���� */
//        gpio_init_struct.Pin = SD_CMD_GPIO_PIN;
//        HAL_GPIO_Init(SD_CMD_GPIO_PORT, &gpio_init_struct); /* ��ʼ�� */
//    }
//}

/**
 * @brief       ��ȡ����Ϣ����
 * @param       cardinfo:SD����Ϣ���
 * @retval      ����ֵ:��ȡ����Ϣ״ֵ̬
 */
uint8_t get_sd_card_info(HAL_SD_CardInfoTypeDef *cardinfo)
{
    uint8_t sta;
    
    sta = HAL_SD_GetCardInfo(&hsd, cardinfo);
    
    return sta;
}

/**
 * @brief       �ж�SD���Ƿ���Դ���(��д)����
 * @param       ��
 * @retval      ����ֵ:SD_TRANSFER_OK      ������ɣ����Լ�����һ�δ���
                       SD_TRANSFER_BUSY SD ����æ�������Խ�����һ�δ���
 */
uint8_t get_sd_card_state(void)
{
    return ((HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

/**
 * @brief       ��SD��(fatfs/usb����)
 * @param       pbuf  : ���ݻ�����
 * @param       saddr : ������ַ
 * @param       cnt   : ��������
 * @retval      0, ����;  ����, �������(���SD_Error����);
 */
uint8_t sd_read_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt)
{
    uint8_t sta = HAL_OK;
    uint32_t timeout = SD_TIMEOUT;
    long long lsector = saddr;
    __disable_irq();                                                                       /* �ر����ж�(POLLINGģʽ,�Ͻ��жϴ��SDIO��д����!!!) */
    sta = HAL_SD_ReadBlocks(&hsd, (uint8_t *)pbuf, lsector, cnt,SD_TIMEOUT); /* ���sector�Ķ����� */

//	  sta = HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t *)pbuf, lsector, cnt);
    /* �ȴ�SD������ */
    while (get_sd_card_state() != SD_TRANSFER_OK)
    {
        if (timeout-- == 0)
        {
            sta = SD_TRANSFER_BUSY;
        }
    }
    __enable_irq(); /* �������ж� */
    
    return sta;
}

/**
 * @brief       дSD��(fatfs/usb����)
 * @param       pbuf  : ���ݻ�����
 * @param       saddr : ������ַ
 * @param       cnt   : ��������
 * @retval      0, ����;  ����, �������(���SD_Error����);
 */
uint8_t sd_write_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt)
{
    uint8_t sta = HAL_OK;
    uint32_t timeout = SD_TIMEOUT;
    long long lsector = saddr;
    __disable_irq();                                                                        /* �ر����ж�(POLLINGģʽ,�Ͻ��жϴ��SDIO��д����!!!) */
    sta = HAL_SD_WriteBlocks(&hsd, (uint8_t *)pbuf, lsector, cnt, SD_TIMEOUT); /* ���sector��д���� */

    /* �ȴ�SD��д�� */
    while (get_sd_card_state() != SD_TRANSFER_OK)
    {
        if (timeout-- == 0)
        {
            sta = SD_TRANSFER_BUSY;
        }
    }
    __enable_irq();     /* �������ж� */
    
    return sta;
}
