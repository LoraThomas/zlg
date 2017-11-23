/****************************************Copyright (c)****************************************************
**                            Guangzhou ZLGMCU Development Co., LTD
**
**                                 http://www.zlgmcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           main.c
** Last modified Date:  2011-02-04
** Last Version:        V1.0
** Descriptions:        The main() function example template
**
**--------------------------------------------------------------------------------------------------------
** Created by:          Li Baihua
** Created date:        2010-09-10
** Version:             V1.00
** Descriptions:        ����ģ�壬����û�Ӧ�ó���
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         yanghongyu
** Modified date:       2012-10-08
** Version:             
** Descriptions:        
**
*********************************************************************************************************/
#include "LPC11xx.h"                                                    /* LPC11xx����Ĵ���            */

#include "uart.h" 

/*********************************************************************************************************
  �궨��
*********************************************************************************************************/
#define    UART_BPS       115200                                        /* ����ͨ�Ų�����               */
/*********************************************************************************************************
  ȫ�ֱ���
*********************************************************************************************************/   
uint16_t             uiByteNum = 0;
extern     uint8_t   ucTxbuf[300]; 
extern     uint16_t  uiMs;
/*********************************************************************************************************
** Function name:        uartInit
** Descriptions:        ���ڳ�ʼ��������Ϊ8λ����λ��1λֹͣλ������żУ�飬������Ϊ115200
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void uartInit (void)
{
    uint16_t usFdiv;
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);                             /* ʹ��IOCONʱ��               */
    LPC_IOCON->PIO1_6 |= 0x01;                                          /* ��P1.6 1.7����ΪRXD��TXD    */
    LPC_IOCON->PIO1_7 |= 0x01;

    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);                               /* ��UART���ܲ���ʱ��         */
    LPC_SYSCON->UARTCLKDIV       = 0x01;                                /* UARTʱ�ӷ�Ƶ                 */

    LPC_UART->LCR  = 0x83;                                              /* �������ò�����               */
    usFdiv = (SystemFrequency/LPC_SYSCON->UARTCLKDIV/16)/UART_BPS;      /* ���ò�����                   */
    LPC_UART->DLM  = usFdiv / 256;
    LPC_UART->DLL  = usFdiv % 256;
    LPC_UART->LCR  = 0x03;                                              /* ����������                   */
    LPC_UART->FCR  = 0x87;                                              /* ʹ��FIFO������8���ֽڴ�����  */
    
    NVIC_EnableIRQ(UART_IRQn);                                          /* ʹ��UART�жϣ����������ȼ�   */
    NVIC_SetPriority(UART_IRQn, 1);

    LPC_UART->IER  = 0x01;                                              /* ʹ�ܽ����ж�                 */
}
/*********************************************************************************************************
** Function name:       Uart_En_Rx_IRQ
** Descriptions:        ʹ�ܴ����ж�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Uart_En_Rx_IRQ (void)
{
    LPC_UART->IER  = 0x01;                                              /* ʹ�ܽ����ж�                 */
}
/*********************************************************************************************************
** Function name:       Uart_Clear_Rx_IRQ
** Descriptions:        ʧ�ܴ����ж�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Uart_Clear_Rx_IRQ (void)
{
    LPC_UART->IER  = 0x00;                                              /* ʧ�ܽ����ж�                 */
}
/*********************************************************************************************************
** Function name:       uartSendByte
** Descriptions:        �򴮿ڷ����ӽ����ݣ����ȴ����ݷ�����ɣ�ʹ�ò�ѯ��ʽ
** input parameters:    ucDat:   Ҫ���͵�����
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void uartSendByte (uint8_t ucDat)
{
    LPC_UART->THR = ucDat;                                              /*  д������                    */
    while ((LPC_UART->LSR & 0x40) == 0);                                /*  �ȴ����ݷ������            */
}
/*********************************************************************************************************
** Function name:       uartSendStr
** Descriptions:        �򴮿ڷ����ַ���
** input parameters:    pucStr:  Ҫ���͵��ַ���ָ��
**                      ulNum:   Ҫ���͵����ݸ���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void uartSendStr (char *pucStr)
{
    while (1){
        if (*pucStr == '\0') break;                                     /*  �������������˳�            */
        uartSendByte (*pucStr++);
    }
}
/*********************************************************************************************************
* Function Name:        UART_IRQHandler
* Description:          UART�жϷ�����
* Input:                ��
* Output:               ��
* Return:               ��
*********************************************************************************************************/
void UART_IRQHandler (void)
{
 
	unsigned char  ucNum    = 0;

	while ((LPC_UART->IIR & 0x01) == 0){                                /*  �ж��Ƿ����жϹ���          */
		uiMs  =   150000 / UART_BPS + 2;                                /*  �趨��ʱ���봮�������й�    */
		ucNum = 0;
		switch (LPC_UART->IIR & 0x0E){                                  /*  �ж��жϱ�־                */
	
			case 0x04:                                                  /*  ���������ж�                */
				for (ucNum = 0; ucNum < 8; ucNum++){                    /*  ��������8���ֽ�             */
					ucTxbuf[uiByteNum] = LPC_UART->RBR;
					uiByteNum++;
					if(uiByteNum > PACK_LENGTH) {
						 uiByteNum--; 
					} 
				}
				break;
			
			case 0x0C:                                                  /*  �ַ���ʱ�ж�                */
				while ((LPC_UART->LSR & 0x01) == 0x01){                 /*  �ж������Ƿ�������        */
					ucTxbuf[uiByteNum] = LPC_UART->RBR;  
					uiByteNum++;
					if(uiByteNum > PACK_LENGTH) {
						uiByteNum--; 
					} 

				}
				break;
					
			default:
				break;
		}
	} 
}
/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
