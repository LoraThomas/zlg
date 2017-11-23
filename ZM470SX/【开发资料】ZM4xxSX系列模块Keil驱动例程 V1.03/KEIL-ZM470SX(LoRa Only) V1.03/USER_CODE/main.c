/****************************************Copyright (c)****************************************************
**                            Guangzhou ZLGMCU Development Co., LTD
**
**                                 http://www.zlgmcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           main.c
** Last modified Date:  2015-8-13
** Last Version:        V1.0
** Descriptions:         
**
**--------------------------------------------------------------------------------------------------------
** Created by:          yanghongyu
** Created date:        2015-6-06
** Version:             V1.00
** Descriptions:         
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         heyuanfeng
** Modified date:       2017-5-2
** Version:             V1.03
** Descriptions:        修正LoRa初始化函数不能重复调用的Bug
                        删除FSK函数，只保留LORA函数
*********************************************************************************************************/
#include "LPC11xx.h"                                                    /* LPC11xx外设寄存器            */
#include "uart.h" 

#include "stdint.h"
#include "stdio.h"

#include "platform.h"
#include "radio.h"
#include "sx1276-LoRa.h"
#include "sx1276-Hal.h"
#include "sx1276.h"


/*********************************************************************************************************
  宏定义
*********************************************************************************************************/
#define LED_INIT()    LPC_GPIO2->DIR  |=  0XFF0                         /* LED初始化                    */


void SX1276LoRaSetSpreadingFactor( uint8_t factor );
tRadioDriver *Radio = 0;

extern uint16_t   uiByteNum;									        /*  串口接收数据个数            */

uint8_t   ucRxbuf[PACK_LENGTH]  = {0};                                  /*  接收数据缓冲区              */
uint8_t   ucTxbuf[PACK_LENGTH] = {0};                                   /*  发送数据缓冲区兼串口缓冲    */
uint16_t  uiMs          = 0; 

static uint8_t    ucsleep  = 0;
static uint8_t    led  = 0;
static uint8_t    UPwr = 20;                                             /*  发射功率,最小5dBm,最大20dBm */
char GcRcvBuf[100] = {0};
/*********************************************************************************************************
** Function name:        LED_Show
** Descriptions:         
** input parameters:     ucled 
** output parameters:    
*********************************************************************************************************/
void LED_Show(unsigned char ucled)
{

	unsigned int temp = 0;
		
	 /* 先调整IO顺序,再赋值 */
	temp |= (ucled & (1ul << 7)) >> 5;   /* LED7,DIO6 */
	temp |= (ucled & (1ul << 6)) >> 3;   /* LED6,DIO7 */
	temp |= (ucled & (1ul << 5)) >> 1;   /* LED5,DIO8 */	
	temp |= (ucled & (1ul << 4)) >> 4;   /* LED4,DIO4 */
	temp |= (ucled & (1ul << 3)) >> 2;   /* LED3,DIO5 */	
	temp |= (ucled & (1ul << 2)) << 3;   /* LED2,DIO9 */
	temp |= (ucled & (1ul << 1)) << 5;   /* LED1,DIO10 */	
	temp |= (ucled & (1ul << 0)) << 7;   /* LED0,DIO11 */

	LPC_GPIO2->DATA &= 0x00f;
	LPC_GPIO2->DATA |= ~(temp << 4);
}
/*********************************************************************************************************
** Function name:        dellayms
** Descriptions:         
** input parameters:     uiNum: 
** output parameters:    
*********************************************************************************************************/
void delayms(unsigned int uiNum)
{
	unsigned int i = 0;
	unsigned int j = 0;
	while(i < uiNum) {
		i++;
		for(j = 0; j < 4369; j++);
	}
}
/*********************************************************************************************************
** Function name:       KEY2_INT
** Descriptions:        按键S2对应的中断初始化函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void KEY2_INT (void)
{
	LPC_GPIO1->DIR &= ~ (1ul << 2);                                     /*  设置P1.2为输入              */
	LPC_GPIO1->IS   = 0x00;                                             /*  P1.2为边沿中断              */
	LPC_GPIO1->IEV &= (1ul << 2);                                       /*  下降沿中断                  */
	LPC_GPIO1->IE  |= (1ul << 2);                                       /*  P1.2中断不屏蔽              */		
	NVIC_EnableIRQ(EINT1_IRQn);                                         /* 设置中断并使能               */
	NVIC_SetPriority(EINT1_IRQn, 2);	
}
/*********************************************************************************************************
** Function name:       PIOINT1_IRQHandler
** Descriptions:        按键S2对应的中断服务函数，发送一包数据
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void PIOINT1_IRQHandler  (void)
{
	uint16_t  i      = 0;	
	
	delayms(10); 
	while(!(LPC_GPIO1->DATA & (1ul << 2)));															/*  等待按键松开                */

	for(i = 0; i < 50; i++) {
		ucTxbuf[i] = i;
	}
	uiByteNum = 10; 
	uiMs = 2;                                                           /* 使能发送条件                 */

	LPC_GPIO1->IC |= (1ul << 2);                                        /*  清除中断标志                */

}
/*********************************************************************************************************
** Function name:       KEY3_INT
** Descriptions:        外中断初始化函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void KEY3_INT (void)
{
	LPC_GPIO0->DIR &= ~ (1ul << 7);                                     /*  设置P0.7为输入              */
	LPC_GPIO0->IS   = 0x00;                                             /*  P0.7为边沿中断              */
	LPC_GPIO0->IEV &= ~(1ul << 7);                                      /*  下降沿中断                  */
	LPC_GPIO0->IE  |= (1ul << 7);                                       /*  P0.7中断不屏蔽              */		
	NVIC_EnableIRQ(EINT0_IRQn);                                         /* 设置中断并使能               */
	NVIC_SetPriority(EINT0_IRQn, 2);	
	LPC_GPIO0->IC |= (1ul << 7);                                        /*  清除中断标志                */
}
/*********************************************************************************************************
** Function name:       PIOINT0_IRQHandler
** Descriptions:        S3中断服务函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void PIOINT0_IRQHandler  (void)
{
	UPwr++;
	if (UPwr > 20) {
		UPwr = 5;
	}	
	while(!(LPC_GPIO0->DATA & (1ul << 7)));								
	delayms(10); 
	LPC_GPIO0->IC |= (1ul << 7);                                       
	//LED_Show(UPwr);

	SX1276TxPower(UPwr);
	sprintf(GcRcvBuf,"\r\nPower:%d",UPwr);uartSendStr(GcRcvBuf); 
}
/*********************************************************************************************************
** Function name:       DisableIRQ
** Descriptions:        关闭中断
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void DisableIRQ (void)
{
	NVIC_DisableIRQ(EINT0_IRQn);			      
	NVIC_DisableIRQ(EINT1_IRQn);
}
/*********************************************************************************************************
** Function name:       EnableIRQ
** Descriptions:        恢复中断
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void EnableIRQ (void)
{
	NVIC_EnableIRQ(EINT0_IRQn);                                         
	NVIC_EnableIRQ(EINT1_IRQn);  	
}
/*********************************************************************************************************
** Function name:       main
** Descriptions:        应用函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
int main(void)
{
	double crssi;
	uint8_t  UPwr;	
	uint8_t   j      = 0;
	uint16_t  i      = 0;		
	uint16_t  temp      = 0x5f;	
	uint8_t   tempbuf[PACK_LENGTH]  = {0};
	uint32_t  Freq;
	//系统初始化；晶体11.0592MHz
	SystemInit();                                                       			
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6);  	
	//初始化LED
	LED_INIT();                                                         	
	//初始化UART
	uartInit();                                                         		 
	//初始按钮S3中断;对应功率
	KEY3_INT();			                                               
	//初始按钮S2中断 
	KEY2_INT();														    
	//复位并检测SPI
	SX1276Reset();
	if(SX1276CheckSPI())
		LED_Show(0x18);//SPI正常
	else{
		LED_Show(0xff);//SPI错误
		while(1) ;//进入死循环
	}
	//初始化模块
	SX1276Init();   
	//初始化完成后再设置频率470Mhz
	Freq = 470000000;
	UPwr = 20;
	SX1276FreqSet(Freq);
	//设置发射功率为20dBM
	SX1276TxPower(UPwr);
	//进入待机状态
	SX1276SetOpMode(RFLR_OPMODE_STANDBY);
	//进入接收状态
	SX1276RxStateEnter(); 
	sprintf(GcRcvBuf,"\r\nInitialization finished.\r\nFreq:%dhz Power:%2ddBm\n",Freq,UPwr);uartSendStr(GcRcvBuf); 
	while (1){
	//接收 
	//如果DIO0输出高电平，则接收到数据
		if (1 == SX1276Dio0State()){                               	
			//把RF芯片FIFO中的数据读出来
			//SX1276RxDataRead( &ucRxbuf[0], &j );   不使用CRC32校验
			SX1276RxDataReadCRC32( &ucRxbuf[0], &j );//使用CRC32校验
			//进入接收状态
			SX1276RxStateEnter(); 				
			if(j > 0){
				crssi = GetPackLoRaRSSI();				
				for (temp = 0; temp < j; temp++) {
					//把数据从串发送出去
					uartSendByte(ucRxbuf[temp]);                   			
				}
				//收到数据计数加1
				led++;                                                 
				led = led % 128; 
				LED_Show(led);
			}			
		}
		//发送	
		//若缓冲区满或者串口缓冲区接收超时则把缓冲区数据发送出去
		if(uiByteNum >= PACK_LENGTH || uiMs == 1){
			//失能中断,防止SPI时序混乱
			DisableIRQ();                                                
			//失能UART中断
			NVIC_DisableIRQ(UART_IRQn);                         
			//把串口缓冲区的数据读出来
			for(i = 0; i < uiByteNum; i++){
				tempbuf[i] = ucTxbuf[i];                                 
			}
			//缓冲区数据标志清零
			uiByteNum = 0;
			//使能UART中断
			NVIC_EnableIRQ(UART_IRQn);                             		
			LED_Show(0x80);	
			//发送数据出去
			//SX1276TxData(&tempbuf[0], i);            不使用CRC32校验
			if(SX1276TxDataCRC32(&tempbuf[0], i)!=0){  //使用CRC32校验
				//发送数据失败
				delayms(100);
				LED_Show(0xff);	
				delayms(100);
				LED_Show(0x80);	
			}
			//进入接收状态
			SX1276RxStateEnter();                   
			//恢复按键中断
			EnableIRQ();                                 
			LED_Show(led);
			uiMs = 0;    
			//休眠
			if(ucsleep == 1){
				//进入休眠,电流1.2uA
				SX1276SetOpMode(RFLR_OPMODE_SLEEP);                    
				ucsleep = 0;
			}	
		}
		if(uiByteNum > 0){
			//误差很小的1ms延时
			delayms(1); 	                                           
			if(uiMs > 0){
				//等待串口接收延时减1
				uiMs--;                                                  
			}   
		}	
	}
}

/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
