/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ModbusOverSerial.h                                         */
/* Author      : Stefano Martino                                            */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _MODBUS_OVER_SERIAL_H
 #define _MODBUS_OVER_SERIAL_H

/////////////////////////////////////////////////////////////////////////////
//

#define MODBUS_OVERSER_ERROR_INVALID_FRAME_LENGTH   0x0001
#define MODBUS_OVERSER_ERROR_FRAME_OVERRUN          0x0002
#define MODBUS_OVERSER_ERROR_TIMER_15_EXPIRED       0x0004


/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
  union {
      struct {
        UWORD ubDriverEnable : 1; // Enable Driver
        UWORD ubBroadcAnswer : 1; // Force Broadcast Answers (false for standar Modbus)
      } b ;
      UWORD w ;
  } flags ;

  UWORD ubSlaveAddress;

} MODBUS_OVERSER_DRV_PARAMS;

/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
  ULONG ulBaudRate;
  UWORD uwDataBits;
  UWORD uwParityMode;
  UWORD uwStopBits;
  UWORD uwPortMode;
  ULONG uwWriteDelay;               // [usec]

} MODBUS_OVERSER_PORT_SETUP;



/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
  MODBUS_OVERSER_DRV_PARAMS   stDrvParams;
  MODBUS_OVERSER_PORT_SETUP   stPortSetup;
  ULONG                       uwRxToTxDelay;        // [usec]
} MODBUS_OVERSER_DRIVER_SETUP;

/////////////////////////////////////////////////////////////////////////////
//

SWORD ModbusOverSerOpenPort( UWORD uwPortNumber, MODBUS_OVERSER_DRIVER_SETUP  * hpDriverSetup );
SWORD ModbusOverSerClosePort( UWORD uwPortNumber );

SWORD ModbusOverSerRecvPDU( UWORD uwPortNumber, HPUBYTE hpubFrameBuffer, HPUWORD hpuwFrameLength, HPVOID sContext );
SWORD ModbusOverSerSendPDU( UWORD uwPortNumber, HPUBYTE hpubFrameBuffer, UWORD   uwFrameLength  , HPVOID sContext );

SWORD ModbusOverSerMinimalParamsEnc( MODBUS_OVERSER_DRIVER_SETUP  * hpDriverSetupSrc, UWORD  * hpuwMinParamDst );
SWORD ModbusOverSerMinimalParamsDec( UWORD  * hpuwMinParamSrc, MODBUS_OVERSER_DRIVER_SETUP  * hpDriverSetupDst );

UWORD ModBusOverSerCRC16( UWORD uwStartCrc, HPUBYTE hpubFrame, UWORD uwLength );

#endif // _MODBUS_OVER_SERIAL_H

