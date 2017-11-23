/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx1276-LoRa.c
 * \brief      SX1276 RF chip driver mode LoRa
 *
 * \version    2.0.0 
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#include <string.h>

#include "platform.h"

#if defined( USE_SX1276_RADIO )

#include "radio.h"

#include "sx1276-Hal.h"
#include "sx1276.h"

#include "sx1276-LoRaMisc.h"
#include "sx1276-LoRa.h"

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -155.0
#define RSSI_OFFSET_HF                              -150.0

#define NOISE_ABSOLUTE_ZERO                         -174.0

#define NOISE_FIGURE_LF                                4.0
#define NOISE_FIGURE_HF                                6.0 

/*!
 * Precomputed signal bandwidth log values
 * Used to compute the Packet RSSI value.
 */
const double SignalBwLog[] =
{
    3.8927900303521316335038277369285,  // 7.8 kHz
    4.0177301567005500940384239336392,  // 10.4 kHz
    4.193820026016112828717566631653,   // 15.6 kHz
    4.31875866931372901183597627752391, // 20.8 kHz
    4.4948500216800940239313055263775,  // 31.2 kHz
    4.6197891057238405255051280399961,  // 41.6 kHz
    4.795880017344075219145044421102,   // 62.5 kHz
    5.0969100130080564143587833158265,  // 125 kHz
    5.397940008672037609572522210551,   // 250 kHz
    5.6989700043360188047862611052755   // 500 kHz
};

const double RssiOffsetLF[] =
{   // These values need to be specify in the Lab
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
};

const double RssiOffsetHF[] =
{   // These values need to be specify in the Lab
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
};

/*!
 * Frequency hopping frequencies table
 */
const int32_t HoppingFrequencies[] =
{
    916500000,
    923500000,
    906500000,
    917500000,
    917500000,
    909000000,
    903000000,
    916000000,
    912500000,
    926000000,
    925000000,
    909500000,
    913000000,
    918500000,
    918500000,
    902500000,
    911500000,
    926500000,
    902500000,
    922000000,
    924000000,
    903500000,
    913000000,
    922000000,
    926000000,
    910000000,
    920000000,
    922500000,
    911000000,
    922000000,
    909500000,
    926000000,
    922000000,
    918000000,
    925500000,
    908000000,
    917500000,
    926500000,
    908500000,
    916000000,
    905500000,
    916000000,
    903000000,
    905000000,
    915000000,
    913000000,
    907000000,
    910000000,
    926500000,
    925500000,
    911000000,
};

// Default settings
tLoRaSettings LoRaSettings =
{
    472500000,        /* 载波频率 */
    20,               /* 发射功率 */
    9,                /* 带宽  [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                       5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved] */
    10,               /* 扩频因数 [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips] */ 
    2,                // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    true,             /* CRC 校验 [0: OFF, 1: ON] */ 
    false,            // ImplicitHeaderOn [0: OFF, 1: ON] /* 包头模式 */
    0,                // RxSingleOn [0: Continuous, 1 Single] /* 固定为设置0 */
    0,                // FreqHopOn [0: OFF, 1: ON]  /* 固定为设置0 */
    4,                // HopPeriod Hops every frequency hopping period symbols
    100,              // TxPacketTimeout
    100,              // RxPacketTimeout
    128,              // PayloadLength (used for implicit header mode)
};

/*!
 * SX1276 LoRa registers variable
 */
tSX1276LR* SX1276LR;

/*!
 * Local RF buffer for communication support
 */
static uint8_t RFBuffer[RF_BUFFER_SIZE];

/*!
 * RF state machine variable
 */
static uint8_t RFLRState = RFLR_STATE_IDLE;

/*!
 * Rx management support variables
 */
static uint16_t RxPacketSize = 0;
static int8_t RxPacketSnrEstimate;
static double RxPacketRssiValue;
static uint8_t RxGain = 1;
static uint32_t RxTimeoutTimer = 0;
/*!
 * PacketTimeout Stores the Rx window time value for packet reception
 */
static uint32_t PacketTimeout;

/*!
 * Tx management support variables
 */
static uint16_t TxPacketSize = 0;

void SX1276LoRaInit( void )
{
    RFLRState = RFLR_STATE_IDLE;

    SX1276LoRaSetDefaults( );
    
    SX1276ReadBuffer( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );
    
    SX1276LR->RegLna = RFLR_LNA_GAIN_G1;

    SX1276WriteBuffer( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );

    // set the RF settings 
    SX1276LoRaSetRFFrequency( LoRaSettings.RFFrequency );
	
	
    SX1276LoRaSetSpreadingFactor( LoRaSettings.SpreadingFactor ); // SF6 only operates in implicit header mode.
    SX1276LoRaSetErrorCoding( LoRaSettings.ErrorCoding );	 	/* code rate 编码率 */
    SX1276LoRaSetPacketCrcOn( LoRaSettings.CrcOn );
    SX1276LoRaSetSignalBandwidth( LoRaSettings.SignalBw );

    SX1276LoRaSetImplicitHeaderOn( LoRaSettings.ImplicitHeaderOn );
    SX1276LoRaSetSymbTimeout( 0x3FF );
    SX1276LoRaSetPayloadLength( LoRaSettings.PayloadLength );
    SX1276LoRaSetLowDatarateOptimize( true );

#if( ( MODULE_SX1276RF1IAS == 1 ) || ( MODULE_SX1276RF1KAS == 1 ) )
    if( LoRaSettings.RFFrequency > 860000000 )
    {
        SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_RFO );
        SX1276LoRaSetPa20dBm( false );
        LoRaSettings.Power = 14;
        SX1276LoRaSetRFPower( LoRaSettings.Power );
    }
    else
    {
		SX1276Write( REG_LR_OCP, 0x3f );	/* 默认参数PA最在电流为100mA,当输出20dBm时需要120mA,所以必须配置0x0b存器*/
        SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST );
        SX1276LoRaSetPa20dBm( true );
//        LoRaSettings.Power = 20;
        SX1276LoRaSetRFPower( LoRaSettings.Power );
    } 

#endif
	
    SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
}

void SX1276LoRaSetDefaults( void )
{
    // REMARK: See SX1276 datasheet for modified default values.

    SX1276Read( REG_LR_VERSION, &SX1276LR->RegVersion );
}

void SX1276LoRaReset( void )
{
    SX1276SetReset( RADIO_RESET_ON );
    
    // Wait 1ms 
	dellayxm(1);
    SX1276SetReset( RADIO_RESET_OFF );  
    // Wait 6ms
	dellayxm(6);
}

void SX1276LoRaSetOpMode( uint8_t opMode )
{
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = true;
    bool antennaSwitchTxOn = false;

    opModePrev = SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;

    if( opMode != opModePrev )
    {
        if( opMode == RFLR_OPMODE_TRANSMITTER )
        {
            antennaSwitchTxOn = true;
        }
        else
        {
            antennaSwitchTxOn = false;
        }
        if( antennaSwitchTxOn != antennaSwitchTxOnPrev )
        {
            antennaSwitchTxOnPrev = antennaSwitchTxOn;
//            RXTX( antennaSwitchTxOn ); // Antenna switch control
        }
        SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_MASK ) | opMode;

        SX1276Write( REG_LR_OPMODE, SX1276LR->RegOpMode );        
    }
}

uint8_t SX1276LoRaGetOpMode( void )
{
    SX1276Read( REG_LR_OPMODE, &SX1276LR->RegOpMode );
    
    return SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;
}

uint8_t SX1276LoRaReadRxGain( void )
{
    SX1276Read( REG_LR_LNA, &SX1276LR->RegLna );
    return( SX1276LR->RegLna >> 5 ) & 0x07;
}

uint8_t SX1276LoRaGetPacketRxGain( void )
{
    return RxGain;
}

int8_t SX1276LoRaGetPacketSnr( void )
{
    return RxPacketSnrEstimate;
}

double SX1276LoRaGetPacketRssi( void )
{
    return RxPacketRssiValue;
}

void SX1276LoRaStartRx( void )
{
    SX1276LoRaSetRFState( RFLR_STATE_RX_INIT );
}

void SX1276LoRaGetRxPacket( void *buffer, uint16_t *size )
{
    *size = RxPacketSize;
    RxPacketSize = 0;
    memcpy( ( void * )buffer, ( void * )RFBuffer, ( size_t )*size );
}

void SX1276LoRaSetTxPacket( const void *buffer, uint16_t size )
{
    if( LoRaSettings.FreqHopOn == false )
    {
        TxPacketSize = size;
    }
    else
    {
        TxPacketSize = 255;
    }
    memcpy( ( void * )RFBuffer, buffer, ( size_t )TxPacketSize ); 

    RFLRState = RFLR_STATE_TX_INIT;
}

uint8_t SX1276LoRaGetRFState( void )
{
    return RFLRState;
}

void SX1276LoRaSetRFState( uint8_t state )
{
    RFLRState = state;
}

/*!
 * \brief Process the LoRa modem Rx and Tx state machines depending on the
 *        SX1276 operating mode.
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY, 
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
uint32_t SX1276LoRaProcess( void )
{
    uint32_t result = RF_BUSY;
    
    switch( RFLRState )
    {
    case RFLR_STATE_IDLE:       break;

    case RFLR_STATE_RX_INIT:
        
        SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );				  /*  操作寄存器只能在在Standby,sleep or FSTX 模式       */ 

        SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    //RFLR_IRQFLAGS_RXDONE |
                                    //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );			 /*  设置需要屏蔽的中断,被注释掉的中断是打开的        */ 

        if( LoRaSettings.FreqHopOn == true )
        {
            SX1276LR->RegHopPeriod = LoRaSettings.HopPeriod;

            SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
            SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
        }
        else
        {
            SX1276LR->RegHopPeriod = 255;
        }
        
        SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );			//0x24
                
                                    // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
        SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CadDetected               ModeReady
        SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_10 | RFLR_DIOMAPPING2_DIO5_00;
	/* 在ZM470SX-M上,DIO4接到射频开关的6脚,进入接收状态后此引脚需要拉高,
	   所以在LoRa模式必须设为:RFLR_DIOMAPPING2_DIO4_10, */       

		SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );			/*  配置5个IO的功能   */ 
    
        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {

            SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
        else // Rx continuous mode
        {
            SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;		  /* 内容:0x00 */
            SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );   /* SPI访问FIFO的地址 */
            
            SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );
        }
        
        memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );

        PacketTimeout = LoRaSettings.RxPacketTimeout;
        RxTimeoutTimer = GET_TICK_COUNT( );
        RFLRState = RFLR_STATE_RX_RUNNING;
        break;
    case RFLR_STATE_RX_RUNNING:
        
        if( DIO0 == 1 ) // RxDone
        {
            RxTimeoutTimer = GET_TICK_COUNT( );
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
                SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );
            RFLRState = RFLR_STATE_RX_DONE;
        }
        if( DIO2 == 1 ) // FHSS Changed Channel
        {
            RxTimeoutTimer = GET_TICK_COUNT( );
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
                SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
            // Debug
            RxGain = SX1276LoRaReadRxGain( );
        }

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            if( ( GET_TICK_COUNT( ) - RxTimeoutTimer ) > PacketTimeout )
            {
                RFLRState = RFLR_STATE_RX_TIMEOUT;
            }
        }
        break;
    case RFLR_STATE_RX_DONE:
        SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
        if( ( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )	/*  CRC错误   */ 
        {
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );
            
            if( LoRaSettings.RxSingleOn == true ) // Rx single mode
            {
                RFLRState = RFLR_STATE_RX_INIT;
            }
            else
            {
                RFLRState = RFLR_STATE_RX_RUNNING;
            }
            break;
        }
        
        {
            uint8_t rxSnrEstimate;
            SX1276Read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );	   	/*  信噪比  */ 
            if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
                RxPacketSnrEstimate = -RxPacketSnrEstimate;
            }
            else
            {
                // Divide by 4
                RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
            }
        }
        
        if( LoRaSettings.RFFrequency < 860000000 )  // LF  /* 获取包信号强度 */
        {    
            if( RxPacketSnrEstimate < 0 )
            {
                RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_LF + ( double )RxPacketSnrEstimate;
            }
            else
            {    
                SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
                RxPacketRssiValue = RssiOffsetLF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegPktRssiValue;
            }
        }
        else                                        // HF
        {    
            if( RxPacketSnrEstimate < 0 )
            {
                RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_HF + ( double )RxPacketSnrEstimate;
            }
            else
            {    
                SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
                RxPacketRssiValue = RssiOffsetHF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegPktRssiValue;
            }
        }

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;
            SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1276LR->RegPayloadLength;
                SX1276ReadFifo( RFBuffer, SX1276LR->RegPayloadLength );
            }
            else
            {
                SX1276Read( REG_LR_NBRXBYTES, &SX1276LR->RegNbRxBytes );
                RxPacketSize = SX1276LR->RegNbRxBytes;
                SX1276ReadFifo( RFBuffer, SX1276LR->RegNbRxBytes );
            }
        }
        else // Rx continuous mode
        {
            SX1276Read( REG_LR_FIFORXCURRENTADDR, &SX1276LR->RegFifoRxCurrentAddr );  /* 读取最后接收一包数据首指针 */

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1276LR->RegPayloadLength;
                SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
                SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );	/* 写入FIFO的访问地址 */
                SX1276ReadFifo( RFBuffer, SX1276LR->RegPayloadLength );
            }
            else
            {
                SX1276Read( REG_LR_NBRXBYTES, &SX1276LR->RegNbRxBytes );	 	/* 读取包长度 */
                RxPacketSize = SX1276LR->RegNbRxBytes;
                SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
                SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );
                SX1276ReadFifo( RFBuffer, SX1276LR->RegNbRxBytes );
            }
        }
        
        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            RFLRState = RFLR_STATE_RX_INIT;
        }
        else // Rx continuous mode
        {
            RFLRState = RFLR_STATE_RX_RUNNING;
        }
        result = RF_RX_DONE;
        break;
    case RFLR_STATE_RX_TIMEOUT:
        RFLRState = RFLR_STATE_RX_INIT;
        result = RF_RX_TIMEOUT;
        break;
    case RFLR_STATE_TX_INIT:

        SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        if( LoRaSettings.FreqHopOn == true )
        {
            SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        //RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED;
            SX1276LR->RegHopPeriod = LoRaSettings.HopPeriod;

            SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
            SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
        }
        else
        {
            SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        //RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED;
            SX1276LR->RegHopPeriod = 0;
        }
        SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );
        SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );

        // Initializes the payload size
        SX1276LR->RegPayloadLength = TxPacketSize;
        SX1276Write( REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength ); /* 在implicit模式(隐式包头),必须写入FIFO长度,0x80*/
        
        SX1276LR->RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
        SX1276Write( REG_LR_FIFOTXBASEADDR, SX1276LR->RegFifoTxBaseAddr );

        SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoTxBaseAddr;
        SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );
        
        // Write payload buffer to LORA modem
        SX1276WriteFifo( RFBuffer, SX1276LR->RegPayloadLength );
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
        SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
        SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
		/* 在ZM470SX-M上,DIO4接到射频开关的6脚,进入发送状态后此引脚需要拉低,
	   所以在LoRa模式必须设为:RFLR_DIOMAPPING2_DIO4_00, */       

        SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

        SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );

        RFLRState = RFLR_STATE_TX_RUNNING;
        break;
    case RFLR_STATE_TX_RUNNING:
        if( DIO0 == 1 ) // TxDone
        {
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );
            RFLRState = RFLR_STATE_TX_DONE;   
        }
        if( DIO2 == 1 ) // FHSS Changed Channel
        {
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
                SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
        }
        break;
    case RFLR_STATE_TX_DONE:
        // optimize the power consumption by switching off the transmitter as soon as the packet has been sent
        SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        RFLRState = RFLR_STATE_IDLE;
        result = RF_TX_DONE;
        break;
    case RFLR_STATE_CAD_INIT:    
        SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
    
        SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    //RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; // |
                                    //RFLR_IRQFLAGS_CADDETECTED;
        SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );
           
                                    // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
        SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CAD Detected              ModeReady
        SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );
            
        SX1276LoRaSetOpMode( RFLR_OPMODE_CAD );
        RFLRState = RFLR_STATE_CAD_RUNNING;
        break;
    case RFLR_STATE_CAD_RUNNING:
        if( DIO3 == 1 ) //CAD Done interrupt
        { 
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE  );
            if( DIO4 == 1 ) // CAD Detected interrupt
            {
                // Clear Irq
                SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED  );
                // CAD detected, we have a LoRa preamble
                RFLRState = RFLR_STATE_RX_INIT;
                result = RF_CHANNEL_ACTIVITY_DETECTED;
            } 
            else
            {    
                // The device goes in Standby Mode automatically    
                RFLRState = RFLR_STATE_IDLE;
                result = RF_CHANNEL_EMPTY;
            }
        }   
        break;
    
    default:
        break;
    } 
    return result;
}
/*********************************************************************************************************
** Function name:       GetLoRaSNR
** Descriptions:        获取调用本函数前最后接到那包数据的信噪比
** input parameters:    无
** output parameters:   无
** Returned value:      信噪比，有正有负
*********************************************************************************************************/
int8_t GetLoRaSNR (void)
{  
	uint8_t rxSnrEstimate;	
	/************* 取最后一包数据的信噪比 ************/        
	SX1276Read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );	   	            /*  信噪比                      */ 
	if( rxSnrEstimate & 0x80 ) { // The SNR sign bit is 1               /*  信噪比小于0的情况，负数     */ 	
		RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;  	/*  Invert and divide by 4      */
		RxPacketSnrEstimate = -RxPacketSnrEstimate;
	} else{                                                             /*  信噪比大于0的情况，正数     */ 
		RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;		    /*  Divide by 4                 */
	}
	return RxPacketSnrEstimate;
}
/*********************************************************************************************************
** Function name:       GetPackLoRaRSSI
** Descriptions:        获取调用本函数前最后接到那包数据的信号强度
** input parameters:    无
** output parameters:   无
** Returned value:      信号强度，有正有负
*********************************************************************************************************/
double GetPackLoRaRSSI(void)
{  
	if( GetLoRaSNR() < 0 ) {                                            /*  信号被噪声淹没              */ 
		RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + \
		                    NOISE_FIGURE_LF + ( double )RxPacketSnrEstimate;
		/* 功率：P=-174（dBm） + BW(dB) + NF(dB) + SNR(dB); 
		在信号被噪声淹没的情况下用此公式反推RSSI，前三项是热噪声功率,最后一项是信噪比 */ 	
	} else {                                                            /*  信号强于噪声                */ 
		SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
		RxPacketRssiValue = RssiOffsetLF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegPktRssiValue;
	}
	return RxPacketRssiValue;
}
/*********************************************************************************************************
** Function name:       SX1276LoRaReadRssi
** Descriptions:        读取当前信号强度,返回的是调用这个函数那一刻的信号强度
                        需要在接收状态下调用才有用。 
** input parameters:    无
** output parameters:   无
*********************************************************************************************************/
double SX1276LoRaReadRssi( void )
{
    SX1276Read( REG_LR_RSSIVALUE, &SX1276LR->RegRssiValue );               /* 寄存器0x1B，当前信号强度的值 */
	return RssiOffsetHF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegRssiValue;	
	/* 此处只能测量输入到天线接口处的功率；包括信号和噪声。
       与包信号强度不一样，因为包信号强度可通过SNR来还原噪声之下的信号强度 */ 
}
/*********************************************************************************************************
** Function name:       LoRaRxStateEnter
** Descriptions:        进入接收状态
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void LoRaRxStateEnter (void)
{
       
	SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );				  /*  操作寄存器只能在在Standby,sleep or FSTX 模式       */ 

	SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    //RFLR_IRQFLAGS_RXDONE |    /*  只打开接收完成这个中断     */ 
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
	SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask ); /*  设置需要屏蔽的中断,被注释掉的中断是打开的        */ 

    SX1276LR->RegHopPeriod = 255;
	SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );			//0x24
                
                                    /*  DIO0:RxDone */
    SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_11 \
	                         | RFLR_DIOMAPPING1_DIO2_11 | RFLR_DIOMAPPING1_DIO3_11;
	SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_10 | RFLR_DIOMAPPING2_DIO5_11;
	/* 在ZM470SX-M上,DIO4接到射频开关的6脚,进入接收状态后此引脚需要拉高,
	   所以在LoRa模式必须设为:RFLR_DIOMAPPING2_DIO4_10, */
	 
    SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );			/*  配置5个IO的功能   */ 
    
	if( LoRaSettings.RxSingleOn == true ) { // Rx single mode
	
		SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
	}else{ // Rx continuous mode
        
		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;		  /* 内容:0x00 */
		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );   /* SPI访问FIFO的地址 */	
		SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );           /* 进入接收状态 */
	}
        
        memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );

        PacketTimeout = LoRaSettings.RxPacketTimeout;
        RxTimeoutTimer = GET_TICK_COUNT( );
  //      RFLRState = RFLR_STATE_RX_RUNNING;
}
/*********************************************************************************************************
** Function name:       LoRaRxDataRead
** Descriptions:        读取接收到数据
** input parameters:    pbuf:数据缓冲区指针 
                        size:数据字节长度指针  
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void LoRaRxDataRead (uint8_t *pbuf, uint8_t *size )
{
	
	if( DIO0 == 1 ) {                                                   /* RxDone                       */
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );          /* 执行写操作以清除接收中断标志 */
	}	
	
    SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );              /*  记取中断状态标志            */
	if( ( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )/* CRC错误 */ 
	{
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );	/* 执行写操作以清除接收中断标志 */	
		*size = 0;
		return;
	}     
	
	GetLoRaSNR();                  /* 读取最后一包数据信噪比;如果不关心这SNR参数可把此函数删除      */   	     
	GetPackLoRaRSSI();             /* 读取最后一包数据信号强度;如果不关心这RSSI参数可把此函数删除   */   

	/************* 取数据 ************/
	SX1276Read( REG_LR_FIFORXCURRENTADDR, &SX1276LR->RegFifoRxCurrentAddr );  /* 读取最后接收一包数据首指针 */
	if( LoRaSettings.ImplicitHeaderOn == true ) {
		RxPacketSize = SX1276LR->RegPayloadLength;
		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );	/* 写入FIFO的访问地址 */
		SX1276ReadFifo( RFBuffer, SX1276LR->RegPayloadLength );
	} else {
		SX1276Read( REG_LR_NBRXBYTES, size );	 	/* 读取包长度 */
//		RxPacketSize = SX1276LR->RegNbRxBytes;
		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );  /* 写入FIFO的访问地址 */
		SX1276ReadFifo( pbuf, *size);
	}
}	

/*********************************************************************************************************
** Function name:       LoRaTxData
** Descriptions:        发送数据
** input parameters:    pbuf:数据缓冲区指针 
                        size:数据字节数  
** output parameters:   无
** Returned value:      成功:0; 失败:其它
*********************************************************************************************************/
uint8_t LoRaTxData (uint8_t *pbuf, uint8_t size ,uint8_t *pcrcbuf ,uint8_t crcflag)
{
     uint32_t i,j; 
	
     if (0 == pbuf || 0 == size) {
		return 1;
	 }		 
    SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
//    SX1276LoRaSetPreambleLength(65530);
	SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
								RFLR_IRQFLAGS_RXDONE |
								RFLR_IRQFLAGS_PAYLOADCRCERROR |
								RFLR_IRQFLAGS_VALIDHEADER |
								//RFLR_IRQFLAGS_TXDONE |                /*  只打开发送完成这个中断      */ 
								RFLR_IRQFLAGS_CADDONE |
								RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
								RFLR_IRQFLAGS_CADDETECTED;
	SX1276LR->RegHopPeriod = 0;
	SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );
	SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );

	// Initializes the payload size
	SX1276LR->RegPayloadLength = size; //TxPacketSize;
	SX1276Write( REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength +4); /* 在implicit模式(隐式包头),必须写入FIFO长度,0x80*/

	SX1276LR->RegFifoTxBaseAddr = 0x00;                                 /* Full buffer used for Tx      */
	SX1276Write( REG_LR_FIFOTXBASEADDR, SX1276LR->RegFifoTxBaseAddr );  /* 写入发送的首地址             */ 

	SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoTxBaseAddr;
	SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );        /* 写入FIFO的访问地址           */
	SX1276WriteFifo( pbuf, SX1276LR->RegPayloadLength );	            /* 写入数疽要发送的数据         */ 
	//SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );        /* 写入FIFO的访问地址           */
	if(crcflag==1)
	{
		if(size<(256-4))
			SX1276WriteFifo( pcrcbuf, 4 );
		else
			return 1;
	}
    /*  DIO0:TxDone */        
	SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_11 | 
	                           RFLR_DIOMAPPING1_DIO2_11 | RFLR_DIOMAPPING1_DIO3_11;
	SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_11;
		/* 在ZM470SX-M上,DIO4接到射频开关的6脚,进入发送状态后此引脚需要拉低,
	   所以在LoRa模式必须设为:RFLR_DIOMAPPING2_DIO4_00, */
	
	SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

    SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );                     /* 进入发送状态                 */   

	i = 1000;
	while ( DIO0 == 0 && 0 != i) {                  /* 等待发送完成或者发送失败超时,完成后:DIO0为高电平 */
        i--;	
		for(j = 0; j < 4369; j++);	                                    /* 约1ms                        */
	}
	
	if (0 == i) {                                                       /* 如果发送失败                 */
		return 2;
	} else {                                                            /* 如果发送成功                 */
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );          /* 清除发送完成中断             */
		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
		return 0;
	}
//	    SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );                     /* 进入发送状态                 */   
}//
/*********************************************************************************************************
** Function name:       LoRaTxPower
** Descriptions:        设置发射功率
** input parameters:    pwr: 功率范围5 ~ 20
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void LoRaTxPower (uint8_t pwr)
{  
	SX1276Write( REG_LR_OCP, 0x3f );	/* 默认参数PA最在电流为100mA,当输出20dBm时需要120mA,所以必须配置0x0b存器*/
	SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST );
	SX1276LoRaSetPa20dBm( true );
	SX1276LoRaSetRFPower( pwr);
}
/*********************************************************************************************************
** Function name:       LoRaFreqSet
** Descriptions:        设置载波频率
** input parameters:    420000000~500000000
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void LoRaFreqSet (uint32_t freq )
{  
    SX1276LoRaSetRFFrequency( freq );
}

#endif // USE_SX1276_RADIO
