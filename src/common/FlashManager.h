/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : FlashManager.h                                             */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Flash storage manager and sequencer                        */
/*                                                                          */
/****************************************************************************/

#ifndef _FLASHMANAGER_H
#define _FLASHMANAGER_H

#include "CommonDefines.h"

//****************************************************************************
// Return values

#define FLASHMGR_R_OK               0
#define FLASHMGR_R_LOCKFAILED       1
#define FLASHMGR_R_FLASHERROR       2

//****************************************************************************
// Manager functions

    // Initialize
u32 FlashMgrInit(void);

    // Write Data
UWORD FlashMgrWriteData(HPVOID hpvDest, const HPVOID hpvSrc, ULONG ulSize);

#ifdef _AXX_SYSAPP
    // lock manager
UWORD FlashMgrBegin(UWORD uwTimeOut);

    // Flush buffer and unlock manager
UWORD FlashMgrEnd(HPVOID * ppNextValidLocation);
#endif

#endif
