/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright $)AB) 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ECATHw.c                                                   */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : ECAT hardware access                                       */
/*                                                                          */
/****************************************************************************/

#ifndef _ECATHW_H
#define _ECATHW_H

#include "esc.h"
#include "ECATHwCommon.h"
#include "common\CommonTypedef.h"

//***************************************************************************
// ESC Access macros

#define ESC_UINT8(reg)      (*((UBYTE volatile  *)(FPGA_ETHERNET_BASE_ADDRESS + (reg))))
#define ESC_UINT16(reg)     (*((UWORD volatile  *)(FPGA_ETHERNET_BASE_ADDRESS + (reg))))

//***************************************************************************
// ESC Globals

extern volatile BOOL bAlEventEnabled;
extern volatile UALEVENT EscAlEvent;
extern volatile UALCONTROL EscAlControl;


//***************************************************************************
// ESC Globals

void HW_EscReadAccess( UINT8  *pData, UINT16 Address, UINT16 Len );
void HW_EscWriteAccess( UINT8  *pData, UINT16 Address, UINT16 Len );
void HW_ResetIntMask(UINT16 intMask);
void HW_SetIntMask(UINT16 intMask);
void HW_SetAlStatus(UINT8 alStatus, UINT16 alStatusCode);
UINT8 HW_CopyToSendMailbox(TMBX MBXMEM *pMbx);
void HW_CheckAndCopyMailbox(void);
void HW_DisableSyncManChannel(UINT8 channel);
void HW_EnableSyncManChannel(UINT8 channel);
TSYNCMAN * HW_GetSyncMan(UINT8 channel);
UINT16 HW_GetAlEvent(void);
UINT8 HW_EepromAccess(UINT32 address, UINT16 size, UINT16 MBXMEM *pData, UINT8 access);
BOOL HW_MIIMRead(UINT8 ubPhyAddr, UINT8 ubRegAddr, UINT16 * hpuRegRdata);
BOOL HW_MIIMWrite(UINT8 ubPhyAddr, UINT8 ubRegAddr, UINT16 uwRegData);
void HW_DiagRefresh(ECATCM_DIAG * psDiag);
BOOL HW_SetPHYLeds();
void HW_Init(void);
void HW_Main(void);

#endif
