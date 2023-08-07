/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ModbusAppProtocol.h                                        */
/* Author      : Stefano Martino                                            */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _MODBUS_APP_PROTOCOL_H
 #define _MODBUS_APP_PROTOCOL_H

#include "ModbusOverSerial.h"
//#include "ModbusOverEthernet.h"
#ifdef _AXX_SYSAPP
//#include "system\SysAppConfig.h"
#endif

/////////////////////////////////////////////////////////////////////////////
//

#ifdef CFG_EN_MODBUSOVERCAN
#include "ModbusOverCAN.h"
#endif

/////////////////////////////////////////////////////////////////////////////
//

#ifndef _AXX_BOOTBLOCK 
#include "ModbusOverEth.h"
#endif

/////////////////////////////////////////////////////////////////////////////
//

#define MODBUS_OVER_SERIAL_0    0x0001
#define MODBUS_OVER_SERIAL_1    0x0002
#define MODBUS_OVER_USB_PORT    0x0004
#define MODBUS_OVER_ETHERNET    0x0008
#ifdef CFG_EN_MODBUSOVERCAN
#define MODBUS_OVER_CAN         0x0010
#endif

/////////////////////////////////////////////////////////////////////////////
//

#define MODBUS_READ_COILS                   0x01
#define MODBUS_READ_INPUTS                  0x02
#define MODBUS_READ_HOLDING_REGISTERS       0x03
#define MODBUS_READ_INPUT_REGISTERS         0x04
#define MODBUS_WRITE_SINGLE_COIL            0x05
#define MODBUS_WRITE_SINGLE_REGISTER        0x06
#define MODBUS_WRITE_MULTIPLE_COILS         0x0f
#define MODBUS_WRITE_MULTIPLE_REGISTERS     0x10

#define MODBUS_PROGRAM_CONTROLLER           0x0d

/////////////////////////////////////////////////////////////////////////////
//

#define MODBUS_EXCEPTION_SUCCESSFULLY           0x00
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION       0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS   0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE     0x03
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE   0x04

/////////////////////////////////////////////////////////////////////////////
//

#define MODBUS_MAXIMUM_DRIVERS    2


/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
  union {
      struct {
        BOOL uwZeroBaseAddress; // Zero Base Register Addressing (false for standard Modbus)
        BOOL uwDisableModbusOverCAN; // Disable modbus over CAN (free aux CAN port)
      } b ;
      ULONG w ;
  } flags ;

} MODBUS_PROTOCOL_OPTION;

typedef struct
{
  UWORD   uwDriverMask;
  HPVOID  hpvSetupPrms;

} MODBUS_PROTOCOL_DRIVER;


typedef struct
{
  UWORD   uwGeneric[4];
} MODBUS_PROTOCOL_CONTEXT;

/////////////////////////////////////////////////////////////////////////////
//

SWORD ModbusAppProtInit( MODBUS_PROTOCOL_OPTION  * hpProtocolOptions, MODBUS_PROTOCOL_DRIVER  * hpProtocolDrivers );
UBYTE ModbusAppProtStop( void );
UBYTE ModbusAppProtHandleRequests();

#endif // _MODBUS_APP_PROTOCOL_H

