/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ModbusOverSerial.c                                         */
/* Author      : Stefano Martino                                            */
/*               Hu Xiaokai                                                 */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

//#include <XE167F.H>
//#include <intrins.h>
#include <string.h>

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"

#ifndef _RD
#include "drive\AxM-E-Defines.h"
#ifndef _INFINEON_
#define TMR_100NS           uwSysTimers100ns
#else
#define TMR_100NS           CC2_T8
#endif // _infineon_
#else
#include "RemDisp-Defines.h"
#define TMR_100NS           CC2_T8
#endif // _rd

#include "core\SerialPorts.h"
#include "ModbusOverSerial.h"

#ifdef _AXX_SYSAPP
#include "system\SystemStatus.h"
#include "common\TaskScheduler.h"
#include "system\SysAppGlobals.h"
#include "system\SysAppInterruptLevels.h"
#else
#include "system\BootBlockGlobals.h"
#include "system\BootBlockInterruptLevels.h"
#endif

#include "ModbusOverSerialDefs.h"

/////////////////////////////////////////////////////////////////////////////
//

//#define TODO_MODBUS_OVERSER_ERROR_MANAGEMENT


/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
    UBYTE   ubReceivedSlaveAddress;
    UWORD   uwRxToTxTimer;
} MODBUS_OVERSER_CONTEXT ;

/////////////////////////////////////////////////////////////////////////////
//
// Modbus use two timers: Timer1.5 for silent detection between received bytes
//                        Timer3.5 for silent detection between received frames
//
// Remark from "MODBUS over serial line specification and implementation guide V1.0" :
//   The implementation of RTU reception driver may imply the management of a lot of interruptions due
//   to the t1.5 and t3.5 timers. With high communication baud rates, this leads to a heavy CPU load.
//   Consequently these two timers must be strictly respected when the baud rate is equal or lower than
//   19200 Bps. For baud rates greater than 19200 Bps, fixed values for the 2 timers should be used: it
//   is recommended to use a value of 750uS for the inter-character time-out (t1.5) and a value of 
//   1.750mS for inter-frame delay (t3.5).
//
// Note :
//   To avoid two interrupt services, we use only hardware Timer3.5. The Timer1.5 is implemented as 
//   ticks difference between Timer3.5 and Timer1.5 and is named Timer2.0
//

// serial port 0
static UWORD uwPort0Timer35Count;
static UWORD uwPort0Timer35Timer;
static UWORD uwPort0Timer35Ticks;

static UWORD uwPort0Timer20Count;
static UWORD uwPort0Timer20Timer;
static UWORD uwPort0Timer20Ticks;

static BOOL  bPort0WaitFirstByte;
static BOOL  bPort0FrameReceived;

static MODBUS_OVERSER_DRV_PARAMS stPort0DriverParams;
static UWORD uwPort0DriverParamRxToTxDelay;

static UWORD TTC_TIMER2_0;

// serial port 1
static UWORD uwPort1Timer35Count;
static UWORD uwPort1Timer35Timer;
static UWORD uwPort1Timer35Ticks;

static UWORD uwPort1Timer20Count;
static UWORD uwPort1Timer20Timer;
static UWORD uwPort1Timer20Ticks;

static BOOL  bPort1WaitFirstByte;
static BOOL  bPort1FrameReceived;

static MODBUS_OVERSER_DRV_PARAMS stPort1DriverParams;
static UWORD uwPort1DriverParamRxToTxDelay;

static UWORD TTC_TIMER2_1;


/////////////////////////////////////////////////////////////////////////////
//

static const UBYTE ubTabCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const UBYTE ubTabCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

/////////////////////////////////////////////////////////////////////////////
//

static const ULONG ulMinBaudRateTable[]=
{
    SERIALPORTS_CBR_1200,
    SERIALPORTS_CBR_2400,
    SERIALPORTS_CBR_4800,
    SERIALPORTS_CBR_9600,
    SERIALPORTS_CBR_14400,
    SERIALPORTS_CBR_19200,
    SERIALPORTS_CBR_38400,
    SERIALPORTS_CBR_56000,
    SERIALPORTS_CBR_57600,
    SERIALPORTS_CBR_115200,
    SERIALPORTS_CBR_128000,
    SERIALPORTS_CBR_230400,
    SERIALPORTS_CBR_256000,
    SERIALPORTS_CBR_460800,
    SERIALPORTS_CBR_512000,
};

/////////////////////////////////////////////////////////////////////////////
//

UWORD ModBusOverSerCRC16( UWORD uwStartCrc, HPUBYTE hpubFrame, UWORD uwLength )
{
  UBYTE  ubCRCHi = (UBYTE)(uwStartCrc >> 8);
  UBYTE  ubCRCLo = (UBYTE)(uwStartCrc & 0xFF);

  while ( uwLength-- ) {
    SWORD swIndex = ubCRCLo ^ *( hpubFrame++ );
    ubCRCLo = ubCRCHi ^ ubTabCRCHi[ swIndex ];
    ubCRCHi = ubTabCRCLo[ swIndex ];
  }                             
  return ubCRCHi << 8 | ubCRCLo;
}


/////////////////////////////////////////////////////////////////////////////
//

static void ModBusOverSerPort0RxCbk( void );
static void CCU_Handler_0(void);
static void ModBusOverSerPort1RxCbk( void );
static void CCU_Handler_1(void);

/////////////////////////////////////////////////////////////////////////////
//

SWORD ModbusOverSerOpenPort( UWORD uwPortNumber, MODBUS_OVERSER_DRIVER_SETUP  * hpDriverSetup )
{
  ULONG ulT15, ulT35; 
  SWORD sw;

  // Check Port Number
  if ( uwPortNumber != 0 && uwPortNumber != 1 )
    return SERIALPORTS_IE_BADID;

  // Check if Driver Enabled
  if ( !hpDriverSetup->stDrvParams.flags.b.ubDriverEnable )
    return -10;

  // Check Data Bits (RTU Transmission Mode require 8 data bits)
  if ( hpDriverSetup->stPortSetup.uwDataBits != SERIALPORTS_DATABITS_8 )
    return SERIALPORTS_IE_DATABITS;

  // Check Parity Mode / Stop Bits Combination (RTU Transmission Mode Remark : the use of no parity requires 2 stop bits)
//  if ( uwParityMode == SERIALPORTS_NOPARITY && uwStopBits != SERIALPORTS_TWOSTOPBITS )
//    return SERIALPORTS_IE_PARITYMODE;

  if( uwPortNumber == 0)
  {
    // Save for ModbusOverSerRecvPDU and ModbusOverSerSendPDU
    stPort0DriverParams.flags.b.ubDriverEnable   = hpDriverSetup->stDrvParams.flags.b.ubDriverEnable;
    stPort0DriverParams.flags.b.ubBroadcAnswer   = hpDriverSetup->stDrvParams.flags.b.ubBroadcAnswer;
    stPort0DriverParams.ubSlaveAddress           = hpDriverSetup->stDrvParams.ubSlaveAddress;
    if(hpDriverSetup->uwRxToTxDelay>0)
      uwPort0DriverParamRxToTxDelay = hpDriverSetup->uwRxToTxDelay/125+1;
    else
      uwPort0DriverParamRxToTxDelay = 0;

    //
    bPort0WaitFirstByte = TRUE; 
    bPort0FrameReceived = FALSE;

    ///  -----------------------------------------------------------------------
    ///  Based on TTC2 that runs at 100 nS compute Timer Constants
    ///  -----------------------------------------------------------------------
    if ( hpDriverSetup->stPortSetup.ulBaudRate <= 19200 ) {
      ulT15 = (ULONG)( (ULONG)( (ULONG)( 1000000000 / hpDriverSetup->stPortSetup.ulBaudRate ) *
                    (UWORD)( 15 * ( 1 + hpDriverSetup->stPortSetup.uwDataBits + hpDriverSetup->stPortSetup.uwStopBits ) ) )
                  / 10 ) / 100;

      ulT35 = (ULONG)( (ULONG)( (ULONG)( 1000000000 / hpDriverSetup->stPortSetup.ulBaudRate ) *
                    (UWORD)( 35 * ( 1 + hpDriverSetup->stPortSetup.uwDataBits + hpDriverSetup->stPortSetup.uwStopBits ) ) )
                  / 10 ) / 100;
    } else {
      ulT15 =  7500;
      ulT35 = 17500;
    }
    //
    uwPort0Timer20Count  = (UWORD)( ( ulT35 - ulT15 ) / 65536L );
    uwPort0Timer20Ticks  = (UWORD)( ( ulT35 - ulT15 ) % 65536L );
    //
    uwPort0Timer35Count  = (UWORD)( ulT35 / 65536L );
    uwPort0Timer35Ticks  = (UWORD)( ulT35 % 65536L );

    // Open Serial Port
    if ( ( sw = SerialPortOpen( uwPortNumber, hpDriverSetup->stPortSetup.ulBaudRate, 
                                              hpDriverSetup->stPortSetup.uwDataBits, 
                                              hpDriverSetup->stPortSetup.uwParityMode, 
                                              hpDriverSetup->stPortSetup.uwStopBits, 
                                              hpDriverSetup->stPortSetup.uwPortMode, 
                                              hpDriverSetup->stPortSetup.uwWriteDelay, ModBusOverSerPort0RxCbk, NULL ) ) != 0 )
      return sw;

    Timer_CCSet(TIMER_CCU0_ID, SYSAPPIL_MODBUSOVERSERIAL, CCU_Handler_0);
  }

  if( uwPortNumber == 1)
  {
    // Save for ModbusOverSerRecvPDU and ModbusOverSerSendPDU
    stPort1DriverParams.flags.b.ubDriverEnable   = hpDriverSetup->stDrvParams.flags.b.ubDriverEnable;
    stPort1DriverParams.flags.b.ubBroadcAnswer   = hpDriverSetup->stDrvParams.flags.b.ubBroadcAnswer;
    stPort1DriverParams.ubSlaveAddress           = hpDriverSetup->stDrvParams.ubSlaveAddress;
    if(hpDriverSetup->uwRxToTxDelay>0)
      uwPort1DriverParamRxToTxDelay = hpDriverSetup->uwRxToTxDelay/125+1;
    else
      uwPort1DriverParamRxToTxDelay = 0;

    //
    bPort1WaitFirstByte = TRUE; 
    bPort1FrameReceived = FALSE;

    ///  -----------------------------------------------------------------------
    ///  Based on TTC2 that runs at 100 nS compute Timer Constants
    ///  -----------------------------------------------------------------------
    if ( hpDriverSetup->stPortSetup.ulBaudRate <= 19200 ) {
      ulT15 = (ULONG)( (ULONG)( (ULONG)( 1000000000 / hpDriverSetup->stPortSetup.ulBaudRate ) *
                    (UWORD)( 15 * ( 1 + hpDriverSetup->stPortSetup.uwDataBits + hpDriverSetup->stPortSetup.uwStopBits ) ) )
                  / 10 ) / 100;

      ulT35 = (ULONG)( (ULONG)( (ULONG)( 1000000000 / hpDriverSetup->stPortSetup.ulBaudRate ) *
                    (UWORD)( 35 * ( 1 + hpDriverSetup->stPortSetup.uwDataBits + hpDriverSetup->stPortSetup.uwStopBits ) ) )
                  / 10 ) / 100;
    } else {
      ulT15 =  7500;
      ulT35 = 17500;
    }
    //
    uwPort1Timer20Count  = (UWORD)( ( ulT35 - ulT15 ) / 65536L );
    uwPort1Timer20Ticks  = (UWORD)( ( ulT35 - ulT15 ) % 65536L );
    //
    uwPort1Timer35Count  = (UWORD)( ulT35 / 65536L );
    uwPort1Timer35Ticks  = (UWORD)( ulT35 % 65536L );

    // Open Serial Port
    if ( ( sw = SerialPortOpen( uwPortNumber, hpDriverSetup->stPortSetup.ulBaudRate, 
                                              hpDriverSetup->stPortSetup.uwDataBits, 
                                              hpDriverSetup->stPortSetup.uwParityMode, 
                                              hpDriverSetup->stPortSetup.uwStopBits, 
                                              hpDriverSetup->stPortSetup.uwPortMode, 
                                              hpDriverSetup->stPortSetup.uwWriteDelay, ModBusOverSerPort1RxCbk, NULL ) ) != 0 )
      return sw;

    Timer_CCSet(TIMER_CCU1_ID, SYSAPPIL_MODBUSOVERSERIAL, CCU_Handler_1);
  }

  // Discard received bytes
  SerialPortRxFlush( uwPortNumber );

  //
  SerialPortReceiverOn( uwPortNumber );

  return 0;
}


/////////////////////////////////////////////////////////////////////////////
//

SWORD ModbusOverSerClosePort( UWORD uwPortNumber )
{
  // Check Port Number
  if ( uwPortNumber != 0 && uwPortNumber != 1 )
    return SERIALPORTS_IE_BADID;

  // disable timer irq
  Timer_CCStop( uwPortNumber==0 ? TIMER_CCU0_ID : TIMER_CCU1_ID );

  // and close serial port
  SerialPortClose( uwPortNumber );

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
//

SWORD ModbusOverSerMinimalParamsEnc( MODBUS_OVERSER_DRIVER_SETUP  * hpDriverSetupSrc, UWORD  * hpuwMinParamDst )
{
  UWORD uwCnt;

  if ( hpDriverSetupSrc->stPortSetup.uwDataBits != SERIALPORTS_DATABITS_8 )
    return SERIALPORTS_IE_DATABITS;

  for(uwCnt=0; uwCnt < sizeof(ulMinBaudRateTable)/sizeof(ULONG);uwCnt++)
    if(ulMinBaudRateTable[uwCnt] == hpDriverSetupSrc->stPortSetup.ulBaudRate)
      break;
  
  if(uwCnt >= sizeof(ulMinBaudRateTable)/sizeof(ULONG))
    return SERIALPORTS_IE_BAUDRATE;

  uwCnt <<= 8;

  uwCnt  |= hpDriverSetupSrc->stDrvParams.ubSlaveAddress & 0x007F;
  uwCnt  |= (hpDriverSetupSrc->stPortSetup.uwStopBits==SERIALPORTS_TWOSTOPBITS?0x0080:0x0000);
  uwCnt  |= (hpDriverSetupSrc->stPortSetup.uwParityMode&0x0003)<<14;

  *hpuwMinParamDst = uwCnt;

  return 0;
}

SWORD ModbusOverSerMinimalParamsDec( UWORD  * hpuwMinParamSrc, MODBUS_OVERSER_DRIVER_SETUP  * hpDriverSetupDst )
{
  hpDriverSetupDst->stPortSetup.ulBaudRate     = ulMinBaudRateTable[(*hpuwMinParamSrc>>8)&0x000F];
  hpDriverSetupDst->stDrvParams.ubSlaveAddress = (UBYTE)(*hpuwMinParamSrc & 0x007F);
  hpDriverSetupDst->stPortSetup.uwStopBits     = (*hpuwMinParamSrc&0x0080?SERIALPORTS_TWOSTOPBITS:SERIALPORTS_ONESTOPBIT);
  hpDriverSetupDst->stPortSetup.uwParityMode   = (*hpuwMinParamSrc>>14)&0x0003;

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
//

SWORD ModbusOverSerRecvPDU( UWORD uwPortNumber, HPUBYTE hpubFrameBuffer, HPUWORD hpuwFrameLength, HPVOID sContext )
{
  UBYTE ubFrameCRC[ MODBUS_SER_PDU_SIZE_CRC ];
  UBYTE ubAddress[ MODBUS_SER_PDU_OFFS_PDU ];
  SWORD swFrameLength;
  SWORD swModBusPDULength;
  UBYTE ubSlaveAddress;

  // Check Port Number
  if ( uwPortNumber != 0 && uwPortNumber != 1 )
    return SERIALPORTS_IE_BADID;

  // Check if Driver Enabled
  if ((uwPortNumber == 0 && !stPort0DriverParams.flags.b.ubDriverEnable) || 
      (uwPortNumber == 1 && !stPort1DriverParams.flags.b.ubDriverEnable) )
    return -10;

  if ( (uwPortNumber==0 && !bPort0FrameReceived) || (uwPortNumber==1 && !bPort1FrameReceived) )
    return -1;

  swFrameLength = SerialPortRxStatus( uwPortNumber );

  // If Frame
  if ( swFrameLength >= MODBUS_SER_PDU_SIZE_MIN ) {
    // Read Address
    if ( SerialPortRead( uwPortNumber, ubAddress, MODBUS_SER_PDU_OFFS_PDU ) == MODBUS_SER_PDU_OFFS_PDU )
    {
      // Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus size of address field and CRC checksum.
      swModBusPDULength = swFrameLength - MODBUS_SER_PDU_OFFS_PDU - MODBUS_SER_PDU_SIZE_CRC;
      // Read Modbus PDU
      if ( SerialPortRead( uwPortNumber, hpubFrameBuffer, swModBusPDULength ) == swModBusPDULength )
        // Read Received CRC
        if ( SerialPortRead( uwPortNumber, ubFrameCRC, MODBUS_SER_PDU_SIZE_CRC ) == MODBUS_SER_PDU_SIZE_CRC )
        {
          UWORD uwReceivedCRC;
          UWORD uwFrameCRC16;

          // Calculate CRC    
          uwFrameCRC16 = ModBusOverSerCRC16( MODBUSOVERSER_CRC_STARTVALUE, ubAddress,       MODBUS_SER_PDU_OFFS_PDU );
          uwFrameCRC16 = ModBusOverSerCRC16( uwFrameCRC16,                 hpubFrameBuffer, swModBusPDULength       );

          // Check Received Frame CRC
          uwReceivedCRC  = (UWORD)ubFrameCRC[ 0 ]     ;
          uwReceivedCRC |= (UWORD)ubFrameCRC[ 1 ] << 8;

          // Check Slave Address
          ubSlaveAddress = uwPortNumber==0 ? stPort0DriverParams.ubSlaveAddress : stPort1DriverParams.ubSlaveAddress;

          if ( uwFrameCRC16 == uwReceivedCRC ) {
            // Check received address field. Only valid  frames are passed to the upper layer.
            if ( ( ubAddress[ MODBUS_SER_PDU_OFFS_ADR ] == MODBUS_OVERSER_ADDRESS_BROADCAST ) ||
                 ( ubAddress[ MODBUS_SER_PDU_OFFS_ADR ] >= MODBUS_OVERSER_ADDRESS_MIN_VALUE &&
                   ubAddress[ MODBUS_SER_PDU_OFFS_ADR ] <= MODBUS_OVERSER_ADDRESS_MAX_VALUE &&
                   ubAddress[ MODBUS_SER_PDU_OFFS_ADR ] == ubSlaveAddress) ) {
              // Get received address for send PDU
              ((MODBUS_OVERSER_CONTEXT  *)sContext)->ubReceivedSlaveAddress = ubAddress[ MODBUS_SER_PDU_OFFS_ADR ] ;
              // Set Modbus-PDU length
              *hpuwFrameLength = swModBusPDULength;
              //
              if( uwPortNumber == 0 )
              {
                  bPort0WaitFirstByte = TRUE; bPort0FrameReceived = FALSE;
#ifdef _AXX_SYSAPP
                  // Start timer for delay between RX and TX
                  ((MODBUS_OVERSER_CONTEXT *)sContext)->uwRxToTxTimer = timer_settimeout(uwSysTimers125us, uwPort0DriverParamRxToTxDelay);
#endif
              }
              if( uwPortNumber == 1 )
              {
                  bPort1WaitFirstByte = TRUE; bPort1FrameReceived = FALSE;
#ifdef _AXX_SYSAPP
                  // Start timer for delay between RX and TX
                  ((MODBUS_OVERSER_CONTEXT *)sContext)->uwRxToTxTimer = timer_settimeout(uwSysTimers125us, uwPort1DriverParamRxToTxDelay);
#endif
              }
              // Return no errors
              return 1;
          }
        }
      }
    }
  }

  // Discard received frame
  SerialPortRxFlush( uwPortNumber );

  if( uwPortNumber == 0 )
  {
      //
      bPort0WaitFirstByte = TRUE; 
      bPort0FrameReceived = FALSE;
  }
  
  if( uwPortNumber == 1 )
  {
      //
      bPort1WaitFirstByte = TRUE; 
      bPort1FrameReceived = FALSE;
  }

  // Activate Modbus Over Serial Error
#ifdef TODO_MODBUS_OVERSER_ERROR_MANAGEMENT
  ulSystemErrorBits |= MODBUS_OVERSER_ERROR_INVALID_FRAME_LENGTH;
#endif

  return -1;
}

/////////////////////////////////////////////////////////////////////////////
//

SWORD ModbusOverSerSendPDU( UWORD uwPortNumber, HPUBYTE hpubFrameBuffer, UWORD uwFrameLength, HPVOID sContext )
{
  UBYTE ubFraming[ 2 ];
  UWORD uwFrameCRC16;

  // Check Port Number
  if ( uwPortNumber != 0 && uwPortNumber != 1 )
    return SERIALPORTS_IE_BADID;

  // Check if Driver Enabled
  if ((uwPortNumber == 0 && !stPort0DriverParams.flags.b.ubDriverEnable) || 
      (uwPortNumber == 1 && !stPort1DriverParams.flags.b.ubDriverEnable) )
    return -10;

  // If the request was not sent to the broadcast address we return a reply.
  if ( ((MODBUS_OVERSER_CONTEXT  *)sContext)->ubReceivedSlaveAddress == MODBUS_OVERSER_ADDRESS_BROADCAST &&
      ((uwPortNumber==0 && !stPort0DriverParams.flags.b.ubBroadcAnswer) || (uwPortNumber==1 && !stPort1DriverParams.flags.b.ubBroadcAnswer) ))
    return 0;

  // Modbus Over Serial Line Slave Address, take from incoming PDU as due to a bug of ModBus Cockpit DLL it
  // process only answer PDU from drive with same sender address
  ubFraming[ 0 ] = ((MODBUS_OVERSER_CONTEXT  *)sContext)->ubReceivedSlaveAddress ;
  uwFrameCRC16 = ModBusOverSerCRC16( MODBUSOVERSER_CRC_STARTVALUE, ubFraming, 1 );
  SerialPortWrite( uwPortNumber, ubFraming, 1, SERIALPORTS_BF_QUEUEONLY );

  // Now queue the Modbus PDU into the Modbus Over Serial Line PDU.
  uwFrameCRC16 = ModBusOverSerCRC16( uwFrameCRC16, hpubFrameBuffer, uwFrameLength );
  SerialPortWrite( uwPortNumber, hpubFrameBuffer, uwFrameLength, SERIALPORTS_BF_QUEUEONLY );

  // Add CRC16 checksum for Modbus Over Serial Line PDU.
  ubFraming[ 0 ] = (UBYTE)( uwFrameCRC16 & 0x00FF );
  ubFraming[ 1 ] = (UBYTE)( uwFrameCRC16 >> 8 );

#ifdef _AXX_SYSAPP
  // wait for rx to tx delay is elapsed
  while(!timer_istimedout(uwSysTimers125us, ((MODBUS_OVERSER_CONTEXT  *)sContext)->uwRxToTxTimer));
#endif

  // Queue CRC and send
  SerialPortWrite( uwPortNumber, ubFraming, 2, SERIALPORTS_BF_QUEUEANDSEND );

  // Notify successfully packet processed
#ifdef _AXX_SYSAPP
  bSysStatPnlMgrPacketReceived=TRUE;
#endif

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
//

static void ModBusOverSerPort0RxCbk( void )
{
  // Overrun
  if ( bPort0FrameReceived ) {
    // Activate Modbus Over Serial Error
#ifdef TODO_MODBUS_OVERSER_ERROR_MANAGEMENT
    ulSystemErrorBits |= MODBUS_OVERSER_ERROR_FRAME_OVERRUN;
#endif
    return;
  }

  // If first byte received start Modbus Timer3.5
  if ( bPort0WaitFirstByte ) {
    // Load Modbus Timer2.0
    uwPort0Timer20Timer = uwPort0Timer20Count;
    // Load Modbus Timer3.5
    uwPort0Timer35Timer = uwPort0Timer35Count;

    if ( uwPort0Timer35Timer )
      TTC_TIMER2_0 = TIMER_CCU0_COUNT;
    else
      TTC_TIMER2_0 = TIMER_CCU0_COUNT + uwPort0Timer35Ticks;

    // Enable Modbus Timer3.5
    Timer_CCStart(TIMER_CCU0_ID, TTC_TIMER2_0);

    bPort0WaitFirstByte = FALSE;
  } else {
    // Check if Modbus Timer1.5 has elapsed
    if ( ( uwPort0Timer20Timer == 0 ) && ( (UWORD)(TTC_TIMER2_0 - TIMER_CCU0_COUNT) < uwPort0Timer20Ticks ) )
    {
      // Disable Modbus Timer3.5
      Timer_CCStop( TIMER_CCU0_ID );
      //
      bPort0WaitFirstByte = TRUE; bPort0FrameReceived = FALSE;

    // Activate Modbus Over Serial Error
#ifdef TODO_MODBUS_OVERSER_ERROR_MANAGEMENT
      ulSystemErrorBits |= MODBUS_OVERSER_ERROR_TIMER_15_EXPIRED;
#endif

    }
    else
    {
      // Reoad Modbus Timer2.0
      uwPort0Timer20Timer = uwPort0Timer20Count;
      // Reload Modbus Timer3.5
      uwPort0Timer35Timer = uwPort0Timer35Count;
      if ( uwPort0Timer35Timer )
        TTC_TIMER2_0 = TIMER_CCU0_COUNT;
      else
        TTC_TIMER2_0 = TIMER_CCU0_COUNT + uwPort0Timer35Ticks;

      Timer_CCStart(TIMER_CCU0_ID, TTC_TIMER2_0);
    }
  }
}

static void ModBusOverSerPort1RxCbk( void )
{
  // Overrun
  if ( bPort1FrameReceived ) {
    // Activate Modbus Over Serial Error
#ifdef TODO_MODBUS_OVERSER_ERROR_MANAGEMENT
    ulSystemErrorBits |= MODBUS_OVERSER_ERROR_FRAME_OVERRUN;
#endif
    return;
  }

  // If first byte received start Modbus Timer3.5
  if ( bPort1WaitFirstByte ) {
    // Load Modbus Timer2.0
    uwPort1Timer20Timer = uwPort1Timer20Count;
    // Load Modbus Timer3.5
    uwPort1Timer35Timer = uwPort1Timer35Count;

    if ( uwPort1Timer35Timer )
      TTC_TIMER2_1 = TIMER_CCU1_COUNT;
    else
      TTC_TIMER2_1 = TIMER_CCU1_COUNT + uwPort1Timer35Ticks;

    // Enable Modbus Timer3.5
    Timer_CCStart(TIMER_CCU1_ID, TTC_TIMER2_1);

    bPort1WaitFirstByte = FALSE;
  } else {
    // Check if Modbus Timer1.5 has elapsed
    if ( ( uwPort1Timer20Timer == 0 ) && ( (UWORD)(TTC_TIMER2_1 - TIMER_CCU1_COUNT) < uwPort1Timer20Ticks ) )
    {
      // Disable Modbus Timer3.5
      Timer_CCStop( TIMER_CCU1_ID );
      //
      bPort1WaitFirstByte = TRUE; bPort1FrameReceived = FALSE;

    // Activate Modbus Over Serial Error
#ifdef TODO_MODBUS_OVERSER_ERROR_MANAGEMENT
      ulSystemErrorBits |= MODBUS_OVERSER_ERROR_TIMER_15_EXPIRED;
#endif

    }
    else
    {
      // Reoad Modbus Timer2.0
      uwPort1Timer20Timer = uwPort1Timer20Count;
      // Reload Modbus Timer3.5
      uwPort1Timer35Timer = uwPort1Timer35Count;
      if ( uwPort1Timer35Timer )
        TTC_TIMER2_1 = TIMER_CCU1_COUNT;
      else
        TTC_TIMER2_1 = TIMER_CCU1_COUNT + uwPort1Timer35Ticks;

      Timer_CCStart(TIMER_CCU1_ID, TTC_TIMER2_1);
    }
  }
}

//****************************************************************************
// TTC2 interrupt handler
static void CCU_Handler_0( void )
{
  if(Timer_CCStatus(TIMER_CCU0_ID))
  {
    // Reload Timer TTC_TIMER2 if not elapsed
    if ( uwPort0Timer35Timer ) {
      uwPort0Timer35Timer--;
      if ( uwPort0Timer20Timer )
        uwPort0Timer20Timer--;
    }
    else if ( uwPort0Timer35Count ) {
      TTC_TIMER2_0 = TIMER_CCU0_COUNT + uwPort0Timer35Ticks;
      Timer_CCStart(TIMER_CCU0_ID, TTC_TIMER2_0);
    }
    else {
      // Disable Modbus Timer3.5
      Timer_CCStop(TIMER_CCU0_ID);
      // Signal Frame Received
      bPort0FrameReceived = TRUE;
    }
  }
}

static void CCU_Handler_1( void )
{
  if(Timer_CCStatus(TIMER_CCU1_ID))
  {
    // Reload Timer TTC_TIMER2 if not elapsed
    if ( uwPort1Timer35Timer ) {
      uwPort1Timer35Timer--;
      if ( uwPort1Timer20Timer )
        uwPort1Timer20Timer--;
    }
    else if ( uwPort1Timer35Count ) {
      TTC_TIMER2_1 = TIMER_CCU1_COUNT + uwPort1Timer35Ticks;
      Timer_CCStart(TIMER_CCU1_ID, TTC_TIMER2_1);
    }
    else {
      // Disable Modbus Timer3.5
      Timer_CCStop(TIMER_CCU1_ID);
      // Signal Frame Received
      bPort1FrameReceived = TRUE;
    }
  }
}
