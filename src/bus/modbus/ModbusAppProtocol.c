/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ModbusAppProtocol.c                                        */
/* Author      : Stefano Martino                                            */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

//#include <XE167F.H>
//#include <intrins.h>
#include <stdlib.h>

#include "common\CommonDefines.h"

#ifndef _RD
#include "drive\AxM-E-Defines.h"
#else
#include "RemDisp-Defines.h"
#endif

#include "ModbusAppProtocol.h"

#ifdef _AXX_BOOTBLOCK
#include "common\ParameterSupport.h"
#endif

#include "PacketCommand.h"

#ifdef _AXX_SYSAPP
#include "ModBusComDB.h"
#include "ModBusCommandMgr.h"
#include "system\SysAppGlobals.h"
#endif

#ifdef CFG_EN_MODBUSOVERUSB
#include "ModbusOverUsb.h"
#endif

/////////////////////////////////////////////////////////////////////////////
//

#define MODBUS_APP_PDU_SIZE_MIN     1     // Only Function Code.
#define MODBUS_APP_PDU_SIZE_MAX     256   // Maximum size of a PDU.
#define MODBUS_APP_PDU_OFFS_FNC     0     // Offset of function code in PDU.
#define MODBUS_APP_PDU_OFFS_DAT     1     // Offset of data fields in PDU.

/////////////////////////////////////////////////////////////////////////////
// Read Holding Registers

#define MODBUS_FUNC_RHR_OFFS_ADR    ( MODBUS_APP_PDU_OFFS_DAT + 0 )
#define MODBUS_FUNC_RHR_OFFS_NUM    ( MODBUS_APP_PDU_OFFS_DAT + 2 )
#define MODBUS_FUNC_RHR_SIZE_FNC    ( 4 )
#ifdef _AXX_BOOTBLOCK
#define MODBUS_FUNC_RHR_RCNT_MAX    ( 2 )
#endif
#ifdef _AXX_SYSAPP
#define MODBUS_FUNC_RHR_RCNT_MAX    ( 120 )
#define MODBUS_FUNC_RC_RCNT_MAX     ( 1920 )
#endif

/////////////////////////////////////////////////////////////////////////////
// Write Multiple Registers

#define MODBUS_FUNC_WMR_OFFS_ADR    ( MODBUS_APP_PDU_OFFS_DAT + 0 )
#define MODBUS_FUNC_WMR_OFFS_NUM    ( MODBUS_APP_PDU_OFFS_DAT + 2 )
#define MODBUS_FUNC_WMR_OFFS_BYT    ( MODBUS_APP_PDU_OFFS_DAT + 4 )
#define MODBUS_FUNC_WMR_OFFS_VAL    ( MODBUS_APP_PDU_OFFS_DAT + 5 )
#define MODBUS_FUNC_WMR_SIZE_MIN    ( 5 )
#ifdef _AXX_BOOTBLOCK
#define MODBUS_FUNC_WMR_RCNT_MAX    ( 2 )
#endif
#ifdef _AXX_SYSAPP
#define MODBUS_FUNC_WMR_RCNT_MAX    ( 120 )
#define MODBUS_FUNC_WC_RCNT_MAX     ( 1920 )
#endif


/////////////////////////////////////////////////////////////////////////////
// Write Single Register

#ifdef _AXX_SYSAPP
#define MODBUS_FUNC_WSR_OFFS_ADR    ( MODBUS_APP_PDU_OFFS_DAT + 0 )
#define MODBUS_FUNC_WSR_OFFS_VAL    ( MODBUS_APP_PDU_OFFS_DAT + 2 )
#define MODBUS_FUNC_WSR_SIZE_MIN    ( 4 )
#endif


/////////////////////////////////////////////////////////////////////////////
// Program Controller

#define MODBUS_FUNC_PRC_OFFS_CNT    ( MODBUS_APP_PDU_OFFS_DAT + 0 )
#define MODBUS_FUNC_PRC_OFFS_CMD    ( MODBUS_APP_PDU_OFFS_DAT + 2 )
#define MODBUS_FUNC_PRC_SIZE_MIN    ( 3 )

#define MODBUS_REPL_PRC_SIZE_MIN    ( 3 )
#define MODBUS_REPL_PRC_OFFS_FNC    ( MODBUS_APP_PDU_OFFS_FNC + 0 )
#define MODBUS_REPL_PRC_OFFS_CNT    ( MODBUS_APP_PDU_OFFS_DAT + 0 )
#define MODBUS_REPL_PRC_OFFS_DAT    ( MODBUS_APP_PDU_OFFS_DAT + 2 )



/////////////////////////////////////////////////////////////////////////////
//

static SWORD ModbusAppProtProcessPDU( UWORD uwDriverIdentifier, HPUBYTE hpubFrameBuffer, UWORD uwFrameLength, HPUBYTE hpubReplyBuffer, HPVOID sContext, BOOL bZeroBaseAddress );
static UBYTE ModbusAppProtReadHoldingRegisters( UBYTE ubFCode, HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bZeroBaseAddress );
static UBYTE ModbusAppProtWriteMultipleRegisters( UBYTE ubFCode, HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bZeroBaseAddress );
static UBYTE ModbusAppProtProgramController( HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength );

#ifdef _AXX_SYSAPP
static UBYTE ModbusAppProtReadCoils( UBYTE ubFCode, HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bZeroBaseAddress );
static UBYTE ModbusAppProtWriteSingleRegister( UBYTE ubFCode, HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bZeroBaseAddress );
static UBYTE ModbusAppProtWriteMultipleCoils( UBYTE ubFCode, HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bZeroBaseAddress );
static UBYTE ModbusAppProtWriteSingleCoil( UBYTE ubFCode, HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bZeroBaseAddress );
#endif


/////////////////////////////////////////////////////////////////////////////
//

static UWORD                   uwActiveDriverMask = 0x0000;
static MODBUS_PROTOCOL_OPTION  stModbusProtocolOption;

#ifdef CFG_EN_MODBUSOVERCAN
static UBYTE  ubModbusOverCanFrameBuffer[ MODBUS_APP_PDU_SIZE_MAX ];
static UBYTE  ubModbusOverCanReplyBuffer[ MODBUS_APP_PDU_SIZE_MAX ];
#endif

/////////////////////////////////////////////////////////////////////////////
//

SWORD ModbusAppProtInit( MODBUS_PROTOCOL_OPTION  * hpProtocolOptions, MODBUS_PROTOCOL_DRIVER  * hpProtocolDrivers )
{
  UWORD uwDriver;
  SWORD swRetVal;
  UWORD uwLocDriverMask;

  // Create Driver Mask according on configured protocols
  for ( uwActiveDriverMask = 0x0000, uwDriver = 0; uwDriver < MODBUS_MAXIMUM_DRIVERS; uwDriver++ ) {
    if ( hpProtocolDrivers[ uwDriver ].uwDriverMask ) {    
      uwActiveDriverMask |= hpProtocolDrivers[ uwDriver ].uwDriverMask;
    }
  }

#ifdef CFG_EN_MODBUSOVERCAN
  // Check if controlboard hw rev comply with external remote display
  if ( sGlbControlBoardParameters.sProductInfo.uwProductRev < 200 )
    uwActiveDriverMask &= ~MODBUS_OVER_CAN;

  // Check disabling of external remote display
  if ( hpProtocolOptions->flags.b.uwDisableModbusOverCAN )
    uwActiveDriverMask &= ~MODBUS_OVER_CAN;
#endif

  // Copy MODBUS_PROTOCOL_OPTION
  stModbusProtocolOption = *hpProtocolOptions;

  // Initialize Single Driver Instances
  for ( uwDriver = 0; uwDriver < MODBUS_MAXIMUM_DRIVERS; uwDriver++ ) {
    // select active protocols
    uwLocDriverMask = hpProtocolDrivers[ uwDriver ].uwDriverMask & uwActiveDriverMask;

    // MODBUS_OVER_SERIAL_0
    if ( uwLocDriverMask & MODBUS_OVER_SERIAL_0 ) {    
      MODBUS_OVERSER_DRIVER_SETUP  * hpDriverSetup = (MODBUS_OVERSER_DRIVER_SETUP  *)hpProtocolDrivers[ uwDriver ].hpvSetupPrms;
      swRetVal = ModbusOverSerOpenPort( 0, hpDriverSetup );
      if(swRetVal)
        return swRetVal;
    }

    // MODBUS_OVER_SERIAL_1
    if ( uwLocDriverMask & MODBUS_OVER_SERIAL_1 ) {    
      MODBUS_OVERSER_DRIVER_SETUP  * hpDriverSetup = (MODBUS_OVERSER_DRIVER_SETUP  *)hpProtocolDrivers[ uwDriver ].hpvSetupPrms;
      swRetVal = ModbusOverSerOpenPort( 1, hpDriverSetup );
      if(swRetVal)
        return swRetVal;
    }

    // MODBUS_OVER_CAN
#ifdef CFG_EN_MODBUSOVERCAN
    if ( uwLocDriverMask & MODBUS_OVER_CAN ) {
      swRetVal = ModbusOverCANOpenPort( (MODBUSOVERCAN_PARAMS  *)hpProtocolDrivers[ uwDriver ].hpvSetupPrms,
                                                       ubModbusOverCanFrameBuffer, sizeof(ubModbusOverCanFrameBuffer),
                                                       ubModbusOverCanReplyBuffer, sizeof(ubModbusOverCanReplyBuffer));
      if(swRetVal)
        return swRetVal;
    }
#endif

    // MODBUS_OVER_USB
#ifdef CFG_EN_MODBUSOVERUSB
    if ( uwLocDriverMask & MODBUS_OVER_USB_PORT ) {
      MODBUS_OVERUSB_DRIVER_SETUP  * hpDriverSetup = (MODBUS_OVERUSB_DRIVER_SETUP  *)hpProtocolDrivers[ uwDriver ].hpvSetupPrms;
      swRetVal = ModbusOverUsbOpenPort( hpDriverSetup );
      if(swRetVal)
        return swRetVal;
    }
#endif

    // MODBUS_OVER_ETH
#ifdef CFG_EN_MODBUSOVERETH
#ifndef _AXX_BOOTBLOCK 
    if ( uwLocDriverMask & MODBUS_OVER_ETHERNET )
    {
      MODBUS_OVERETH_DRIVER_SETUP  * hpDriverSetup = (MODBUS_OVERETH_DRIVER_SETUP  *)hpProtocolDrivers[ uwDriver ].hpvSetupPrms;
      swRetVal = ModbusOverEthOpenPort( hpDriverSetup );
      if(swRetVal)
        return swRetVal;
    }
#endif
#endif
  }

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
//

UBYTE ModbusAppProtStop( void )
{
  // Stop for MODBUS_OVER_SERIAL_0
  if ( uwActiveDriverMask & MODBUS_OVER_SERIAL_0 )
  {
    ModbusOverSerClosePort( 0 );
  }

  // Stop for MODBUS_OVER_SERIAL_1
  if ( uwActiveDriverMask & MODBUS_OVER_SERIAL_1 )
    ModbusOverSerClosePort( 1 );

  // Stop for MODBUS_OVER_CAN
#ifdef CFG_EN_MODBUSOVERCAN
  if ( uwActiveDriverMask & MODBUS_OVER_CAN )
    ModbusOverCANClosePort();
#endif

  // Stop for MODBUS_OVER_USB
#ifdef CFG_EN_MODBUSOVERUSB
//  if ( wActiveDriverMask & MODBUS_OVER_USB_PORT )
#endif

  // Stop for MODBUS_OVER_ETH
#ifdef CFG_EN_MODBUSOVERETH
#ifndef _AXX_BOOTBLOCK 
  if ( uwActiveDriverMask & MODBUS_OVER_ETHERNET )
    ModbusOverEthClosePort();
#endif
#endif

  uwActiveDriverMask = 0;

  return FALSE;
}

/////////////////////////////////////////////////////////////////////////////
//

UBYTE ModbusAppProtHandleRequests()
{
  static UBYTE  ubFrameBuffer[ MODBUS_APP_PDU_SIZE_MAX ];
  static UBYTE  ubReplyBuffer[ MODBUS_APP_PDU_SIZE_MAX ];
  UWORD uwFrameLength;
  MODBUS_PROTOCOL_CONTEXT sContext;

  // Handle Request for MODBUS_OVER_SERIAL_0
  if ( uwActiveDriverMask & MODBUS_OVER_SERIAL_0 )
    if ( ModbusOverSerRecvPDU( 0, ubFrameBuffer, &uwFrameLength, &sContext ) > 0 )
      ModbusAppProtProcessPDU( MODBUS_OVER_SERIAL_0, ubFrameBuffer, uwFrameLength, ubReplyBuffer, &sContext, stModbusProtocolOption.flags.b.uwZeroBaseAddress );

  // Handle Request for MODBUS_OVER_SERIAL_1
  if ( uwActiveDriverMask & MODBUS_OVER_SERIAL_1 )
    if ( ModbusOverSerRecvPDU( 1, ubFrameBuffer, &uwFrameLength, &sContext ) > 0 )
      ModbusAppProtProcessPDU( MODBUS_OVER_SERIAL_1, ubFrameBuffer, uwFrameLength, ubReplyBuffer, &sContext, stModbusProtocolOption.flags.b.uwZeroBaseAddress );

  // Handle Request for MODBUS_OVER_CAN
#ifdef CFG_EN_MODBUSOVERCAN
  if ( uwActiveDriverMask & MODBUS_OVER_CAN )
  {
    UBYTE  * ubModbusOverCanBuf;
    if ( ModbusOverCANRecvPDU( &ubModbusOverCanBuf, &uwFrameLength, &sContext ) > 0 )
      ModbusAppProtProcessPDU( MODBUS_OVER_CAN, ubModbusOverCanBuf, uwFrameLength, ubReplyBuffer, &sContext, TRUE );
  }
#endif

#ifdef CFG_EN_MODBUSOVERETH
#ifndef _AXX_BOOTBLOCK
  if ( uwActiveDriverMask & MODBUS_OVER_ETHERNET )
  {
    if((ModbusOverEthRecvPDU(ubFrameBuffer, &uwFrameLength, &sContext ) > 0))
      ModbusAppProtProcessPDU( MODBUS_OVER_ETHERNET, ubFrameBuffer, uwFrameLength, ubReplyBuffer, &sContext, TRUE );
  }
#endif
#endif

#ifdef CFG_EN_MODBUSOVERUSB
  if ( uwActiveDriverMask & MODBUS_OVER_USB_PORT )
  {
    if((ModbusOverUsbRecvPDU(ubFrameBuffer, &uwFrameLength, &sContext ) > 0))
      ModbusAppProtProcessPDU( MODBUS_OVER_USB_PORT, ubFrameBuffer, uwFrameLength, ubReplyBuffer, &sContext, TRUE );
  }
#endif

  return FALSE;
}

/////////////////////////////////////////////////////////////////////////////
//

static SWORD ModbusAppProtProcessPDU( UWORD uwDriverIdentifier, HPUBYTE hpubFrameBuffer, UWORD uwFrameLength, HPUBYTE hpubReplyBuffer, HPVOID sContext, BOOL bZeroBaseAddress )
{
  UBYTE  ubFunctionCode;
  UBYTE  ubExceptionCode;
  UWORD  uwReplyLength = 0;

  ubFunctionCode  = hpubFrameBuffer[ MODBUS_APP_PDU_OFFS_FNC ];
  ubExceptionCode = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;

  switch ( ubFunctionCode ) {
    case MODBUS_READ_HOLDING_REGISTERS :
#ifdef _AXX_SYSAPP
    case MODBUS_READ_INPUT_REGISTERS :
#endif
      ubExceptionCode = ModbusAppProtReadHoldingRegisters( ubFunctionCode, hpubFrameBuffer, uwFrameLength, hpubReplyBuffer, &uwReplyLength, bZeroBaseAddress );
      break;

    case MODBUS_WRITE_MULTIPLE_REGISTERS :
      ubExceptionCode = ModbusAppProtWriteMultipleRegisters( ubFunctionCode, hpubFrameBuffer, uwFrameLength, hpubReplyBuffer, &uwReplyLength, bZeroBaseAddress );
      break;

    case MODBUS_PROGRAM_CONTROLLER :
      ubExceptionCode = ModbusAppProtProgramController( hpubFrameBuffer, uwFrameLength, hpubReplyBuffer, &uwReplyLength );
      break;

#ifdef _AXX_SYSAPP
    case MODBUS_READ_COILS :
    case MODBUS_READ_INPUTS :
      ubExceptionCode = ModbusAppProtReadCoils( ubFunctionCode, hpubFrameBuffer, uwFrameLength, hpubReplyBuffer, &uwReplyLength, bZeroBaseAddress );
      break;

    case MODBUS_WRITE_SINGLE_REGISTER :
      ubExceptionCode = ModbusAppProtWriteSingleRegister( ubFunctionCode, hpubFrameBuffer, uwFrameLength, hpubReplyBuffer, &uwReplyLength, bZeroBaseAddress );
      break;

    case MODBUS_WRITE_MULTIPLE_COILS :
      ubExceptionCode = ModbusAppProtWriteMultipleCoils( ubFunctionCode, hpubFrameBuffer, uwFrameLength, hpubReplyBuffer, &uwReplyLength, bZeroBaseAddress );
      break;

    case MODBUS_WRITE_SINGLE_COIL :
      ubExceptionCode = ModbusAppProtWriteSingleCoil( ubFunctionCode, hpubFrameBuffer, uwFrameLength, hpubReplyBuffer, &uwReplyLength, bZeroBaseAddress );
      break;
#endif

    default :
      break;
  }

  // An exception occured. Build an error frame.
  if ( ubExceptionCode != MODBUS_EXCEPTION_SUCCESSFULLY ) {
    hpubReplyBuffer[ uwReplyLength++ ] = ubFunctionCode | 0x80;
    hpubReplyBuffer[ uwReplyLength++ ] = ubExceptionCode;
  }

  // Handle Response for MODBUS_OVER_SERIAL_0
  if ( uwDriverIdentifier & MODBUS_OVER_SERIAL_0 )
    ModbusOverSerSendPDU( 0, hpubReplyBuffer, uwReplyLength, sContext );

  // Handle Response for MODBUS_OVER_SERIAL_1
  if ( uwDriverIdentifier & MODBUS_OVER_SERIAL_1 )
    ModbusOverSerSendPDU( 1, hpubReplyBuffer, uwReplyLength, sContext );

#ifdef CFG_EN_MODBUSOVERUSB
  // Handle Response for MODBUS_OVER_USB
  if ( uwDriverIdentifier & MODBUS_OVER_USB_PORT )
    ModbusOverUsbSendPDU( hpubReplyBuffer, uwReplyLength, sContext );
#endif

  // Handle Request for MODBUS_OVER_CAN
#ifdef CFG_EN_MODBUSOVERCAN
  if ( uwDriverIdentifier & MODBUS_OVER_CAN )
    ModbusOverCANSendPDU( hpubReplyBuffer, uwReplyLength, sContext );
#endif

#ifdef CFG_EN_MODBUSOVERETH
#ifndef _AXX_BOOTBLOCK
  if ( uwDriverIdentifier & MODBUS_OVER_ETHERNET )
    ModbusOverEthSendPDU( hpubReplyBuffer, uwReplyLength, sContext );
#endif
#endif

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
//

static UBYTE ModbusAppProtReadHoldingRegisters( UBYTE ubFCode, HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bZeroBaseAddress )
{
  // Check received frame length
  if ( uwRecvLength == MODBUS_APP_PDU_SIZE_MIN + MODBUS_FUNC_RHR_SIZE_FNC ) {
    // Retrieve requested register address
    UWORD uwRegAddress = hpubRecvBuffer[ MODBUS_FUNC_RHR_OFFS_ADR ] << 8 |
                         hpubRecvBuffer[ MODBUS_FUNC_RHR_OFFS_ADR + 1 ];
    // Retrieve requested register numbers
    UWORD uwRegNumbers = hpubRecvBuffer[ MODBUS_FUNC_RHR_OFFS_NUM ] << 8 |
                         hpubRecvBuffer[ MODBUS_FUNC_RHR_OFFS_NUM + 1 ];

    // Zero Base Register Addressing (JBUS compatibility)
    if ( !bZeroBaseAddress )
      uwRegAddress++;

    // Check if the number of registers to read is valid. If not return illegal data value exception.
    if ( ( uwRegNumbers >= 1 ) && ( uwRegNumbers <= MODBUS_FUNC_RHR_RCNT_MAX ) ) {

#ifdef _AXX_BOOTBLOCK
      PARAMETER_ELEMENT  * pElem; UWORD uwReplyLength = 0;

      // Search in the parameters database
      if ( ( pElem = ParameterFindElement( uwRegAddress ) ) == NULL )
        return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

      // Check Read Attribute
      if ( !( pElem->ubFlgs & PARAM_FLAGS_RD ) )
        return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

      // First byte contains the function code.
      hpubReplyBuffer[ uwReplyLength++ ] = ubFCode;

      // Second byte contains the number of bytes in the response
      hpubReplyBuffer[ uwReplyLength++ ] = (UBYTE)( uwRegNumbers * 2 );

      // Read Modbus WORD
      if ( uwRegNumbers == 1 ) {
        UWORD uwValue = ParameterDataReadAsUWORD( pElem, uwRegAddress - pElem->uwIndx );
        hpubReplyBuffer[ uwReplyLength++ ] = HIBYTE( uwValue );
        hpubReplyBuffer[ uwReplyLength++ ] = LOBYTE( uwValue );
      // Read Modbus DWORD
      } else if ( uwRegNumbers == 2 ) {
        ULONG ulValue = ParameterDataReadAsULONG( pElem, uwRegAddress - pElem->uwIndx );
        hpubReplyBuffer[ uwReplyLength++ ] = HIBYTE( LOWORD( ulValue ) );
        hpubReplyBuffer[ uwReplyLength++ ] = LOBYTE( LOWORD( ulValue ) );
        hpubReplyBuffer[ uwReplyLength++ ] = HIBYTE( HIWORD( ulValue ) );
        hpubReplyBuffer[ uwReplyLength++ ] = LOBYTE( HIWORD( ulValue ) );
      } else {
        return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
      }

      //
      *hpuwReplyLength = uwReplyLength;
      //
      return MODBUS_EXCEPTION_SUCCESSFULLY;
#endif

#ifdef _AXX_SYSAPP
      return ModBusComDBEntryRead(ubFCode, uwRegAddress, uwRegNumbers, hpubReplyBuffer, hpuwReplyLength);
#endif
    } else {
      return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
  } else {
    // Can't be a valid request because the length is incorrect.
    return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
  }

}



/////////////////////////////////////////////////////////////////////////////
//

static UBYTE ModbusAppProtWriteMultipleRegisters( UBYTE ubFCode, HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bZeroBaseAddress )
{
  // Check received frame length
  if ( uwRecvLength >= MODBUS_APP_PDU_SIZE_MIN + MODBUS_FUNC_WMR_SIZE_MIN ) {
    UWORD uwOrigRegAddress;
    // Retrieve requested register address
    UWORD uwRegAddress = hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_ADR ] << 8 | hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_ADR + 1 ];
    // Retrieve requested register numbers
    UWORD uwRegNumbers = hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_NUM ] << 8 | hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_NUM + 1 ];
    // Retrieve byte count
    UBYTE ubByteCount  = hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_BYT ];

    // Zero Base Register Addressing (JBUS compatibility)
    uwOrigRegAddress = uwRegAddress;
    if ( !bZeroBaseAddress )
      uwRegAddress++;

    // Check if the number of registers to write is valid. If not return illegal data value exception.
    if ( ( uwRegNumbers >= 1 ) && ( uwRegNumbers <= MODBUS_FUNC_WMR_RCNT_MAX ) && ( ubByteCount == (UBYTE)( 2 * uwRegNumbers ) ) ) {

#ifdef _AXX_BOOTBLOCK
      PARAMETER_ELEMENT  * pElem;
      UWORD uwReplyLength = 0;

      // Search in the parameters database
      if ( ( pElem = ParameterFindElement( uwRegAddress ) ) == NULL )
        return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

      // Check Write Attribute
      if ( !( pElem->ubFlgs & PARAM_FLAGS_WR ) )
        return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;


      // Write Modbus WORD
      if ( uwRegNumbers == 1 ) {
        UWORD uwValue;

        ((UBYTE *)&uwValue)[ 1 ] = hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_VAL + 0 ];
        ((UBYTE *)&uwValue)[ 0 ] = hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_VAL + 1 ];

        ParameterDataWriteAsUWORD( pElem, uwRegAddress - pElem->uwIndx, uwValue );
      // Write Modbus DWORD
      } else if ( uwRegNumbers == 2 ) {
        ULONG ulValue;

        ((UBYTE *)&ulValue)[ 1 ] = hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_VAL + 0 ];
        ((UBYTE *)&ulValue)[ 0 ] = hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_VAL + 1 ];
        ((UBYTE *)&ulValue)[ 3 ] = hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_VAL + 2 ];
        ((UBYTE *)&ulValue)[ 2 ] = hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_VAL + 3 ];

        ParameterDataWriteAsULONG( pElem, uwRegAddress - pElem->uwIndx, ulValue );
      } else {
        return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
      }

      // First byte contains the function code.
      hpubReplyBuffer[ uwReplyLength++ ] = ubFCode;

      // Second word contains the register address
      hpubReplyBuffer[ uwReplyLength++ ] = HIBYTE( uwOrigRegAddress );
      hpubReplyBuffer[ uwReplyLength++ ] = LOBYTE( uwOrigRegAddress );

      // Third word contains the register numbers
      hpubReplyBuffer[ uwReplyLength++ ] = HIBYTE( uwRegNumbers );
      hpubReplyBuffer[ uwReplyLength++ ] = LOBYTE( uwRegNumbers );

      //
      *hpuwReplyLength = uwReplyLength;
      //
      return MODBUS_EXCEPTION_SUCCESSFULLY;
#endif

#ifdef _AXX_SYSAPP
      return ModBusComDBEntryWrite(ubFCode, uwRegAddress, uwRegNumbers, uwOrigRegAddress, &hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_VAL], hpubReplyBuffer, hpuwReplyLength, FALSE);
#endif
    } else {
      return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
  } else {
    // Can't be a valid request because the length is incorrect.
    return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
  }
}



/////////////////////////////////////////////////////////////////////////////
//

static UBYTE ModbusAppProtProgramController( HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength )
{
  // Check received frame length
  if ( uwRecvLength >= MODBUS_APP_PDU_SIZE_MIN + MODBUS_FUNC_PRC_SIZE_MIN ) {
    // Retrieve requested register numbers
    UWORD uwByteCount = hpubRecvBuffer[ MODBUS_FUNC_PRC_OFFS_CNT ] << 8 | hpubRecvBuffer[ MODBUS_FUNC_PRC_OFFS_CNT + 1 ];

    // Check if the number bytes is valid. If not return illegal data value exception.
    if ( uwByteCount < MODBUS_APP_PDU_SIZE_MAX - MODBUS_APP_PDU_SIZE_MIN ) {
      //
      SWORD swDataLength = ExecutePacketCommand( hpubRecvBuffer + MODBUS_FUNC_PRC_OFFS_CMD, hpubReplyBuffer + MODBUS_REPL_PRC_OFFS_DAT );

      //
      if ( swDataLength < 1 || hpubReplyBuffer[ MODBUS_REPL_PRC_OFFS_DAT + swDataLength - 1 ] != PACKET_COMMAND_ANS_READY )
        return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

      // First byte contains the function code.
      hpubReplyBuffer[ MODBUS_REPL_PRC_OFFS_FNC ] = MODBUS_PROGRAM_CONTROLLER;

      // Second word contains the data lenght
      hpubReplyBuffer[ MODBUS_REPL_PRC_OFFS_CNT + 0 ] = HIBYTE( swDataLength );
      hpubReplyBuffer[ MODBUS_REPL_PRC_OFFS_CNT + 1 ] = LOBYTE( swDataLength );

      //
      *hpuwReplyLength = MODBUS_REPL_PRC_SIZE_MIN + swDataLength;
      //
      return MODBUS_EXCEPTION_SUCCESSFULLY;
    } else {
      return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
  } else {
    // Can't be a valid request because the length is incorrect.
    return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
  }
}

/////////////////////////////////////////////////////////////////////////////
//

#ifdef _AXX_SYSAPP

static UBYTE ModbusAppProtReadCoils( UBYTE ubFCode, HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bZeroBaseAddress )
{
  // Check received frame length
  if ( uwRecvLength == MODBUS_APP_PDU_SIZE_MIN + MODBUS_FUNC_RHR_SIZE_FNC ) {
    // Retrieve requested register address
    UWORD uwRegAddress = hpubRecvBuffer[ MODBUS_FUNC_RHR_OFFS_ADR ] << 8 |
                         hpubRecvBuffer[ MODBUS_FUNC_RHR_OFFS_ADR + 1 ];
    // Retrieve requested register numbers
    UWORD uwRegNumbers = hpubRecvBuffer[ MODBUS_FUNC_RHR_OFFS_NUM ] << 8 |
                         hpubRecvBuffer[ MODBUS_FUNC_RHR_OFFS_NUM + 1 ];

    // Zero Base Register Addressing (JBUS compatibility)
    if ( !bZeroBaseAddress )
      uwRegAddress++;

    // Check if the number of coils to read is valid. If not return illegal data value exception.
    if ( ( uwRegNumbers >= 1 ) && ( uwRegNumbers <= MODBUS_FUNC_RC_RCNT_MAX ) ) {
      return ModBusComDBCoilRead(ubFCode, uwRegAddress, uwRegNumbers, hpubReplyBuffer, hpuwReplyLength);
    } else {
      return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
  } else {
    // Can't be a valid request because the length is incorrect.
    return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
  }

}

/////////////////////////////////////////////////////////////////////////////
//

static UBYTE ModbusAppProtWriteSingleRegister( UBYTE ubFCode, HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bZeroBaseAddress )
{
  // Check received frame length
  if ( uwRecvLength >= MODBUS_APP_PDU_SIZE_MIN + MODBUS_FUNC_WSR_SIZE_MIN ) {
    UWORD uwOrigRegAddress;
    // Retrieve requested register address
    UWORD uwRegAddress = hpubRecvBuffer[ MODBUS_FUNC_WSR_OFFS_ADR ] << 8 | hpubRecvBuffer[ MODBUS_FUNC_WSR_OFFS_ADR + 1 ];

    // Zero Base Register Addressing (JBUS compatibility)
    uwOrigRegAddress = uwRegAddress;
    if ( !bZeroBaseAddress )
      uwRegAddress++;

    // Pass request to upper handler
    return ModBusComDBEntryWrite(ubFCode, uwRegAddress, 1, uwOrigRegAddress, &hpubRecvBuffer[ MODBUS_FUNC_WSR_OFFS_VAL], hpubReplyBuffer, hpuwReplyLength, TRUE);
  } else {
    // Can't be a valid request because the length is incorrect.
    return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
  }
}

/////////////////////////////////////////////////////////////////////////////
//

static UBYTE ModbusAppProtWriteMultipleCoils( UBYTE ubFCode, HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bZeroBaseAddress )
{
  // Check received frame length
  if ( uwRecvLength >= MODBUS_APP_PDU_SIZE_MIN + MODBUS_FUNC_WMR_SIZE_MIN ) {
    UWORD uwOrigRegAddress;
    // Retrieve requested register address
    UWORD uwRegAddress = hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_ADR ] << 8 | hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_ADR + 1 ];
    // Retrieve requested register numbers
    UWORD uwRegNumbers = hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_NUM ] << 8 | hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_NUM + 1 ];
    // Retrieve byte count
    UBYTE ubByteCount  = hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_BYT ];

    // Zero Base Register Addressing (JBUS compatibility)
    uwOrigRegAddress = uwRegAddress;
    if ( !bZeroBaseAddress )
      uwRegAddress++;

    // Check if the number of coils to write is valid. If not return illegal data value exception.
    if ( ( uwRegNumbers >= 1 ) && ( uwRegNumbers <= MODBUS_FUNC_WC_RCNT_MAX ) && ubByteCount == (UBYTE)( (uwRegNumbers+7)/8 )) {
      return ModBusComDBCoilWrite(ubFCode, uwRegAddress, uwRegNumbers, uwOrigRegAddress, &hpubRecvBuffer[ MODBUS_FUNC_WMR_OFFS_VAL], hpubReplyBuffer, hpuwReplyLength, FALSE);
    } else {
      return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
  } else {
    // Can't be a valid request because the length is incorrect.
    return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
  }
}

/////////////////////////////////////////////////////////////////////////////
//

static UBYTE ModbusAppProtWriteSingleCoil( UBYTE ubFCode, HPUBYTE hpubRecvBuffer, UWORD uwRecvLength, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bZeroBaseAddress )
{
  // Check received frame length
  if ( uwRecvLength >= MODBUS_APP_PDU_SIZE_MIN + MODBUS_FUNC_WSR_SIZE_MIN ) {
    UWORD uwOrigRegAddress;
    // Retrieve requested register address
    UWORD uwRegAddress = hpubRecvBuffer[ MODBUS_FUNC_WSR_OFFS_ADR ] << 8 | hpubRecvBuffer[ MODBUS_FUNC_WSR_OFFS_ADR + 1 ];

    // Zero Base Register Addressing (JBUS compatibility)
    uwOrigRegAddress = uwRegAddress;
    if ( !bZeroBaseAddress )
      uwRegAddress++;

    // Pass request to upper handler
    return ModBusComDBCoilWrite(ubFCode, uwRegAddress, 1, uwOrigRegAddress, &hpubRecvBuffer[ MODBUS_FUNC_WSR_OFFS_VAL], hpubReplyBuffer, hpuwReplyLength, TRUE);
  } else {
    // Can't be a valid request because the length is incorrect.
    return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
  }
}

#endif
