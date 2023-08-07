/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : CanOpenCommandMgr.h                                        */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : CanOpen specific command manager                           */
/*                                                                          */
/****************************************************************************/

#ifndef _CANOPENCOMMANDMGR_H
#define _CANOPENCOMMANDMGR_H

#include "common\DefineExternals.h"
#include "common\CommonParamDB.h"
#include "common\CommonController.h"
#include "drive\AxM-E-Defines.h"
//***************************************************************************
// Defines

#define CANOPENCM_INIT_CORE                 0
#define CANOPENCM_INIT_SYNCEVENT            1
#define CANOPENCM_INIT_SYNCPDOEVENT         2

//***************************************************************************
// Data structures

typedef struct
{
    SWORD   swCanController;
    ULONG   ulDisableCanAlarmMask;
    union
    {
        struct
        {
           BOOL bReSyncEnable; //UWORD bReSyncEnable:1;          // enable fieldbus resync
           BOOL bSyncEvOnPeriodicCob; //UWORD bSyncEvOnPeriodicCob:1;   // enable SYNC event from periodic sending
           BOOL bDelayedInputs; //UWORD bDelayedInputs:1;         // process Tx PDO/Inputs one rt-cycle delayed
           BOOL bDisableNmtResetNode; //UWORD bDisableNmtResetNode:1;   // disable NMT Protocol Command: Reset_Node
        }__attribute__((aligned(4)))f;
//        UWORD w;
    } flags;
}CANOPENCM_PARAM;

typedef struct
{
    const COMCTRL_HANDLERS *psCtrlHandlers  ;
} CANOPENCM_IN ;

typedef struct
{
    UBYTE   ubNMTStatus;
} CANOPENCM_OUT;

//***************************************************************************
// Globals

extern CANOPENCM_PARAM tCanOpenCMParams;
extern const CANOPENCM_PARAM tCanOpenCMDefParams;

extern CANOPENCM_IN sCanOpenCMIn;
extern CANOPENCM_OUT sCanOpenCMOut;

extern BOOL bCanOpenCMFaultClear;

//***************************************************************************
// Global functions

BOOL CanOpenCM_Init(UWORD);
void CanOpenCM_GetHwOpt(UWORD, ULONG *, ULONG *);

void CanOpenCM_CanFaultSignal(ULONG);

#endif
