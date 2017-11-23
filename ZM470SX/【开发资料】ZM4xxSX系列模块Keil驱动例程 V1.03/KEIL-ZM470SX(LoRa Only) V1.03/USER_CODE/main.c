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
** Descriptions:        ����LoRa��ʼ�����������ظ����õ�Bug
                        ɾ��FSK������ֻ����LORA����
*********************************************************************************************************/
#include "LPC11xx.h"                                                    /* LPC11xx����Ĵ���            */
#include "uart.h" 

#include "stdint.h"
#include "stdio.h"

#include "platform.h"
#include "radio.h"
#include "sx1276-LoRa.h"
#include "sx1276-Hal.h"
#include "sx1276.h"


/*********************************************************************************************************
  �궨��
*********************************************************************************************************/
#define LED_INIT()    LPC_GPIO2->DIR  |=  0XFF0                         /* LED��ʼ��                    */


void SX1276LoRaSetSpreadingFactor( uint8_t factor );
tRadioDriver *Radio = 0;

extern uint16_t   uiByteNum;									        /*  ���ڽ������ݸ���            */

uint8_t   ucRxbuf[PACK_LENGTH]  = {0};                                  /*  �������ݻ�����              */
uint8_t   ucTxbuf[PACK_LENGTH] = {0};                                   /*  �������ݻ������洮�ڻ���    */
uint16_t  uiMs          = 0; 

static uint8_t    ucsleep  = 0;
static uint8_t    led  = 0;
static uint8_t    UPwr = 20;                                             /*  ���书��,��С5dBm,���20dBm */
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
		
	 /* �ȵ���IO˳��,�ٸ�ֵ */
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
** Descriptions:        ����S2��Ӧ���жϳ�ʼ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void KEY2_INT (void)
{
	LPC_GPIO1->DIR &= ~ (1ul << 2);                                     /*  ����P1.2Ϊ����              */
	LPC_GPIO1->IS   = 0x00;                                             /*  P1.2Ϊ�����ж�              */
	LPC_GPIO1->IEV &= (1ul << 2);                                       /*  �½����ж�                  */
	LPC_GPIO1->IE  |= (1ul << 2);                                       /*  P1.2�жϲ�����              */		
	NVIC_EnableIRQ(EINT1_IRQn);                                         /* �����жϲ�ʹ��               */
	NVIC_SetPriority(EINT1_IRQn, 2);	
}
/*********************************************************************************************************
** Function name:       PIOINT1_IRQHandler
** Descriptions:        ����S2��Ӧ���жϷ�����������һ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void PIOINT1_IRQHandler  (void)
{
	uint16_t  i      = 0;	
	
	delayms(10); 
	while(!(LPC_GPIO1->DATA & (1ul << 2)));															/*  �ȴ������ɿ�                */

	for(i = 0; i < 50; i++) {
		ucTxbuf[i] = i;
	}
	uiByteNum = 10; 
	uiMs = 2;                                                           /* ʹ�ܷ�������                 */

	LPC_GPIO1->IC |= (1ul << 2);                                        /*  ����жϱ�־                */

}
/*********************************************************************************************************
** Function name:       KEY3_INT
** Descriptions:        ���жϳ�ʼ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void KEY3_INT (void)
{
	LPC_GPIO0->DIR &= ~ (1ul << 7);                                     /*  ����P0.7Ϊ����              */
	LPC_GPIO0->IS   = 0x00;                                             /*  P0.7Ϊ�����ж�              */
	LPC_GPIO0->IEV &= ~(1ul << 7);                                      /*  �½����ж�                  */
	LPC_GPIO0->IE  |= (1ul << 7);                                       /*  P0.7�жϲ�����              */		
	NVIC_EnableIRQ(EINT0_IRQn);                                         /* �����жϲ�ʹ��               */
	NVIC_SetPriority(EINT0_IRQn, 2);	
	LPC_GPIO0->IC |= (1ul << 7);                                        /*  ����жϱ�־                */
}
/*********************************************************************************************************
** Function name:       PIOINT0_IRQHandler
** Descriptions:        S3�жϷ�����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        �ر��ж�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void DisableIRQ (void)
{
	NVIC_DisableIRQ(EINT0_IRQn);			      
	NVIC_DisableIRQ(EINT1_IRQn);
}
/*********************************************************************************************************
** Function name:       EnableIRQ
** Descriptions:        �ָ��ж�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void EnableIRQ (void)
{
	NVIC_EnableIRQ(EINT0_IRQn);                                         
	NVIC_EnableIRQ(EINT1_IRQn);  	
}
/*********************************************************************************************************
** Function name:       main
** Descriptions:        Ӧ�ú���
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
	//ϵͳ��ʼ��������11.0592MHz
	SystemInit();                                                       			
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6);  	
	//��ʼ��LED
	LED_INIT();                                                         	
	//��ʼ��UART
	uartInit();                                                         		 
	//��ʼ��ťS3�ж�;��Ӧ����
	KEY3_INT();			                                               
	//��ʼ��ťS2�ж� 
	KEY2_INT();														    
	//��λ�����SPI
	SX1276Reset();
	if(SX1276CheckSPI())
		LED_Show(0x18);//SPI����
	else{
		LED_Show(0xff);//SPI����
		while(1) ;//������ѭ��
	}
	//��ʼ��ģ��
	SX1276Init();   
	//��ʼ����ɺ�������Ƶ��470Mhz
	Freq = 470000000;
	UPwr = 20;
	SX1276FreqSet(Freq);
	//���÷��书��Ϊ20dBM
	SX1276TxPower(UPwr);
	//�������״̬
	SX1276SetOpMode(RFLR_OPMODE_STANDBY);
	//�������״̬
	SX1276RxStateEnter(); 
	sprintf(GcRcvBuf,"\r\nInitialization finished.\r\nFreq:%dhz Power:%2ddBm\n",Freq,UPwr);uartSendStr(GcRcvBuf); 
	while (1){
	//���� 
	//���DIO0����ߵ�ƽ������յ�����
		if (1 == SX1276Dio0State()){                               	
			//��RFоƬFIFO�е����ݶ�����
			//SX1276RxDataRead( &ucRxbuf[0], &j );   ��ʹ��CRC32У��
			SX1276RxDataReadCRC32( &ucRxbuf[0], &j );//ʹ��CRC32У��
			//�������״̬
			SX1276RxStateEnter(); 				
			if(j > 0){
				crssi = GetPackLoRaRSSI();				
				for (temp = 0; temp < j; temp++) {
					//�����ݴӴ����ͳ�ȥ
					uartSendByte(ucRxbuf[temp]);                   			
				}
				//�յ����ݼ�����1
				led++;                                                 
				led = led % 128; 
				LED_Show(led);
			}			
		}
		//����	
		//�������������ߴ��ڻ��������ճ�ʱ��ѻ��������ݷ��ͳ�ȥ
		if(uiByteNum >= PACK_LENGTH || uiMs == 1){
			//ʧ���ж�,��ֹSPIʱ�����
			DisableIRQ();                                                
			//ʧ��UART�ж�
			NVIC_DisableIRQ(UART_IRQn);                         
			//�Ѵ��ڻ����������ݶ�����
			for(i = 0; i < uiByteNum; i++){
				tempbuf[i] = ucTxbuf[i];                                 
			}
			//���������ݱ�־����
			uiByteNum = 0;
			//ʹ��UART�ж�
			NVIC_EnableIRQ(UART_IRQn);                             		
			LED_Show(0x80);	
			//�������ݳ�ȥ
			//SX1276TxData(&tempbuf[0], i);            ��ʹ��CRC32У��
			if(SX1276TxDataCRC32(&tempbuf[0], i)!=0){  //ʹ��CRC32У��
				//��������ʧ��
				delayms(100);
				LED_Show(0xff);	
				delayms(100);
				LED_Show(0x80);	
			}
			//�������״̬
			SX1276RxStateEnter();                   
			//�ָ������ж�
			EnableIRQ();                                 
			LED_Show(led);
			uiMs = 0;    
			//����
			if(ucsleep == 1){
				//��������,����1.2uA
				SX1276SetOpMode(RFLR_OPMODE_SLEEP);                    
				ucsleep = 0;
			}	
		}
		if(uiByteNum > 0){
			//����С��1ms��ʱ
			delayms(1); 	                                           
			if(uiMs > 0){
				//�ȴ����ڽ�����ʱ��1
				uiMs--;                                                  
			}   
		}	
	}
}

/*********************************************************************************************************
  End Of File
*********************************************************************************************************/
