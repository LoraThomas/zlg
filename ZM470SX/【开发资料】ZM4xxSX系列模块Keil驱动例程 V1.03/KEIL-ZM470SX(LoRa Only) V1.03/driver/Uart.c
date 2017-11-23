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
** Descriptions:        整理模板，添加用户应用程序
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         yanghongyu
** Modified date:       2012-10-08
** Version:             
** Descriptions:        
**
*********************************************************************************************************/
#include "LPC11xx.h"                                                    /* LPC11xx外设寄存器            */

#include "uart.h" 

/*********************************************************************************************************
  宏定义
*********************************************************************************************************/
#define    UART_BPS       115200                                        /* 串口通信波特率               */
/*********************************************************************************************************
  全局变量
*********************************************************************************************************/   
uint16_t             uiByteNum = 0;
extern     uint8_t   ucTxbuf[300]; 
extern     uint16_t  uiMs;
/*********************************************************************************************************
** Function name:        uartInit
** Descriptions:        串口初始化，设置为8位数据位，1位停止位，无奇偶校验，波特率为115200
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void uartInit (void)
{
    uint16_t usFdiv;
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);                             /* 使能IOCON时钟               */
    LPC_IOCON->PIO1_6 |= 0x01;                                          /* 将P1.6 1.7配置为RXD和TXD    */
    LPC_IOCON->PIO1_7 |= 0x01;

    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);                               /* 打开UART功能部件时钟         */
    LPC_SYSCON->UARTCLKDIV       = 0x01;                                /* UART时钟分频                 */

    LPC_UART->LCR  = 0x83;                                              /* 允许设置波特率               */
    usFdiv = (SystemFrequency/LPC_SYSCON->UARTCLKDIV/16)/UART_BPS;      /* 设置波特率                   */
    LPC_UART->DLM  = usFdiv / 256;
    LPC_UART->DLL  = usFdiv % 256;
    LPC_UART->LCR  = 0x03;                                              /* 锁定波特率                   */
    LPC_UART->FCR  = 0x87;                                              /* 使能FIFO，设置8个字节触发点  */
    
    NVIC_EnableIRQ(UART_IRQn);                                          /* 使能UART中断，并配置优先级   */
    NVIC_SetPriority(UART_IRQn, 1);

    LPC_UART->IER  = 0x01;                                              /* 使能接收中断                 */
}
/*********************************************************************************************************
** Function name:       Uart_En_Rx_IRQ
** Descriptions:        使能串口中断
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Uart_En_Rx_IRQ (void)
{
    LPC_UART->IER  = 0x01;                                              /* 使能接收中断                 */
}
/*********************************************************************************************************
** Function name:       Uart_Clear_Rx_IRQ
** Descriptions:        失能串口中断
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Uart_Clear_Rx_IRQ (void)
{
    LPC_UART->IER  = 0x00;                                              /* 失能接收中断                 */
}
/*********************************************************************************************************
** Function name:       uartSendByte
** Descriptions:        向串口发送子节数据，并等待数据发送完成，使用查询方式
** input parameters:    ucDat:   要发送的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void uartSendByte (uint8_t ucDat)
{
    LPC_UART->THR = ucDat;                                              /*  写入数据                    */
    while ((LPC_UART->LSR & 0x40) == 0);                                /*  等待数据发送完毕            */
}
/*********************************************************************************************************
** Function name:       uartSendStr
** Descriptions:        向串口发送字符串
** input parameters:    pucStr:  要发送的字符串指针
**                      ulNum:   要发送的数据个数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void uartSendStr (char *pucStr)
{
    while (1){
        if (*pucStr == '\0') break;                                     /*  遇到结束符，退出            */
        uartSendByte (*pucStr++);
    }
}
/*********************************************************************************************************
* Function Name:        UART_IRQHandler
* Description:          UART中断服务函数
* Input:                无
* Output:               无
* Return:               无
*********************************************************************************************************/
void UART_IRQHandler (void)
{
 
	unsigned char  ucNum    = 0;

	while ((LPC_UART->IIR & 0x01) == 0){                                /*  判断是否有中断挂起          */
		uiMs  =   150000 / UART_BPS + 2;                                /*  设定超时，与串口速率有关    */
		ucNum = 0;
		switch (LPC_UART->IIR & 0x0E){                                  /*  判断中断标志                */
	
			case 0x04:                                                  /*  接收数据中断                */
				for (ucNum = 0; ucNum < 8; ucNum++){                    /*  连续接收8个字节             */
					ucTxbuf[uiByteNum] = LPC_UART->RBR;
					uiByteNum++;
					if(uiByteNum > PACK_LENGTH) {
						 uiByteNum--; 
					} 
				}
				break;
			
			case 0x0C:                                                  /*  字符超时中断                */
				while ((LPC_UART->LSR & 0x01) == 0x01){                 /*  判断数据是否接收完毕        */
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
