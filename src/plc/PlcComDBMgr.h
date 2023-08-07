/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2012, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : PlcComDBMgr.h                                              */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Common DB parameter manager for PLC user parameters        */
/*                                                                          */
/****************************************************************************/

#ifndef _PLCCOMDBMGR_H
#define _PLCCOMDBMGR_H

#include "common\ParamStorageManagement.h"

//***************************************************************************
// Defines

#define PLCCOMDBMGR_IEC_PAR_WS      1030
#define PLCCOMDBMGR_IEC_PAR_BS      1031
#define PLCCOMDBMGR_IEC_PAR_WI      1040
#define PLCCOMDBMGR_IEC_PAR_BI      1041

//***************************************************************************
// Globals

extern UBYTE ubPlcComDbErrCode;
extern UWORD uwPlcComParDefCRC;
extern PARMGM_PTRSOURCE sPlcComParCurrCheck;
extern PARMGM_PTRSOURCE sPlcComParStorCheck;
extern ULONG ulPlcComParCodeStart;
extern ULONG ulPlcComParCodeSize;

//***************************************************************************
// Global functions

UBYTE PlcComDBAppSet(ULONG ulAddress, ULONG ulRecNum);
UBYTE PlcComDBAppUnset(void);
SWORD PlcComDBAppRestParChk(HPVOID pvSrc, UWORD uwSize);
UBYTE PlcComDBAppRemapPar(UWORD * puwWord, UWORD uwWordSz, UWORD * puwBool, UWORD uwBoolSz);

#endif
