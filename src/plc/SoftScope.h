/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SoftScope.h                                                */
/* Author      : Fabio Terrile,Hu Xiaokai                                   */
/*               Axel                                                       */
/*                                                                          */
/* Description : Configuration and integration of SoftScope runtime         */
/*                                                                          */
/****************************************************************************/

#ifndef _SOFTSCOPE_H
#define _SOFTSCOPE_H

//#include "CanOpenDs301.h"
#include "common\CommonParamDB.h"

//***************************************************************************
// Defines

#define ALSSC_RUNTIME_ADV                   (1)

#define SOFTSCOPE_REQ_MEMORY                (0)
#define SOFTSCOPE_SAMPLES_NUMBER            4096//8192

//****************************************************************************
// Globals

extern ULONG  ulScoMemory[ SOFTSCOPE_SAMPLES_NUMBER ];

extern UWORD uwScoCommand;            
extern ULONG ulScoData;               

extern UWORD uwScoTriggerRequiredMode;
extern ULONG ulScoAcquisitionId;      
extern UWORD uwScoTriggerStatus;      
extern ULONG ulScoSampleCount;        
extern ULONG ulScoTriggerPosition;    
extern ULONG ulScoAcquiredDataId;     
#if ALSSC_RUNTIME_ADV
extern ULONG ulScoAbsTimeRef;
extern ULONG ulScoRTBuffSwitch;
#endif

//****************************************************************************
// Global functions

BOOL SoftScopeInit(UWORD t);

BOOL SoftScopeRequestMemory(ULONG);
BOOL SoftScopeReqMemSave(void);
BOOL SoftScopeReqMemLoad(void);
UWORD SoftScopeReqMemParHook(COMMONPARAMDB_ENTRY  *p, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext);
#if ALSSC_RUNTIME_ADV
UWORD SoftScopeDataSwitch(COMMONPARAMDB_ENTRY  *pElem, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext);
#endif

#endif
