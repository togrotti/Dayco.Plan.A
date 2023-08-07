/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CanOpenCOECommon.h                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : CanOpen/COE Common                                         */
/*                                                                          */
/****************************************************************************/

#ifndef _CANOPENCOECOMMON_H
#define _CANOPENCOECOMMON_H

//#include "DefineExternals.h"
#include "common\CommonParamDB.h"

//***************************************************************************
// Globals

extern const CHARS sCanOpenCOEManufacturerName[];
extern const CHARS sCanOpenCOEDeviceNameBase[];
extern const CHARS sCanOpenCOESWVersion[];

//***************************************************************************
// Global functions

BOOL CanOpenCOE_Init(UWORD);
BOOL CanOpenCOE_GetAlarmEntry(ULONG * pulRefTime, ULONG * pulAlarm, UWORD * puwAlarmSubCode, UBYTE * pubAlarmClass, ULONG * pulAlarmStat);

#ifdef _APP_DEBUG
BOOL CanOpenCOE_CheckDBTable(void);
#endif

#ifdef _APP_XC
extern BOOL sCanOpenCOEReqReflash;
#endif

//***************************************************************************
// CanOpen Hooks

UWORD CanOpenCOE_ParamSave(COMMONPARAMDB_ENTRY  * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext);
UWORD CanOpenCOE_ParamRestore(COMMONPARAMDB_ENTRY  * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext);
#ifdef _APP_XC
UWORD CanOpenCOE_FwDownload(COMMONPARAMDB_ENTRY  * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext);
#endif

#endif
