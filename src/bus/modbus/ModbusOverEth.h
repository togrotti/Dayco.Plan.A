/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ModbusOverEth.h                                            */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _MODBUS_OVER_TCPIP_H
#define _MODBUS_OVER_TCPIP_H

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "EnetProtocol.h"

/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
  union {
      struct {
        UWORD ubDriverEnable : 1; // Enable Driver
      } b ;
      UWORD w ;
  } flags ;
  
  UBYTE ubEthPortSel;
  
} MODBUS_OVERETH_DRV_PARAMS;


/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
  MODBUS_OVERETH_DRV_PARAMS stDrvParams;
  ENET_PROT_SETUP           stProtSetup;
  
} MODBUS_OVERETH_DRIVER_SETUP;

/////////////////////////////////////////////////////////////////////////////
//

SWORD ModbusOverEthOpenPort( MODBUS_OVERETH_DRIVER_SETUP  * hpDriverSetup);
SWORD ModbusOverEthRecvPDU( UBYTE  * hpubFrameBuffer, HPUWORD hpuwFrameLength, HPVOID sContext );
void  ModbusOverEthSendPDU( HPUBYTE hpubFrameBuffer, UWORD uwFrameLength, HPVOID sContext );
SWORD ModbusOverEthClosePort(void);

#endif
