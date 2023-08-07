/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SyncManager.h                                              */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Main PWM and related linked RT re-synchronization manager  */
/*                                                                          */
/****************************************************************************/

#ifndef _SYNCMANAGER_H
#define _SYNCMANAGER_H

#include "common\CommonDefines.h"

//****************************************************************************
// Data structures

typedef struct
{
    UWORD       uwTSFilter;         // K filter for timestamp (0=no filter)
    SLONG       slReSyncDelta;      // [nsec] time shift for sync point
    SWORD       swFiltKp;           // Kp PLL
    SWORD       swFiltKd;           // Kd PLL
    UWORD       uwPeakNDiscard;     // number of consecutive discardable sample
    ULONG       ulPeakThreshold;    // threshold for peak detection
} SYNCMGR_PARAMS;

typedef struct
{
    ULONG       ulSyncTime;         // [nsec] sync time period
    ULONG       ulSyncMax;          // [nsec] max sync time period
    ULONG       ulSyncMin;          // [nsec] min sync time period
    SWORD       swCorrection;       // istantaneous value used for correction
    SBYTE       bValid;             // sync valid and system synchronized
    SLONG       slLastSamplePoint;  // last fieldbus sample point, averaged,
                                    // 16LSB [0x0000:0xFFFF=0:125usec]
                                    // 16MSB n*125usec
    SLONG       slSyncInstValue;    // instant sync period
                                    // 16LSB [0x0000:0xFFFF=0:125usec]
                                    // 16MSB n*125usec
} SYNCMGR_OUT;

//****************************************************************************
// General global variables

extern SYNCMGR_PARAMS sSyncMgrParam;
extern const SYNCMGR_PARAMS  sSyncMgrDefParam;
extern SYNCMGR_OUT sSyncMgrDiagnosticOut;
#ifdef _APP_XC
extern UWORD uwSyncMgrAdjRTScaling;
#endif

//****************************************************************************
// Global functions

BOOL SyncMgrInit(UWORD uwOption);
BOOL SyncMgrAdjustRTPeriod(UWORD uwPeriod);

#endif
