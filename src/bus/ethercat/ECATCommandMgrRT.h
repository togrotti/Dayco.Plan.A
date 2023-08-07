/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ECATCommandMgrRT.h                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : ECAT time-optimized real time module                       */
/*                                                                          */
/****************************************************************************/

#ifndef _ECATCOMMANDMGRRT_H
#define _ECATCOMMANDMGRRT_H

#include "ECATHwCommon.h"
#include "system\SysAppConfig.h"
#include "common\CommonParamDB.h"

//***************************************************************************
// PDO Fast Entry data types for time optimized runtime

#define ECATCM_RT_SELECTOR_8BIT                     0
#define ECATCM_RT_SELECTOR_EVEN16                   1
#define ECATCM_RT_SELECTOR_ODD16                    2
#define ECATCM_RT_SELECTOR_EVEN32                   3
#define ECATCM_RT_SELECTOR_ODD32                    4
#define ECATCM_RT_SELECTOR_64BIT                    5
#define ECATCM_RT_SELECTOR_8BIT_UM                  6
#define ECATCM_RT_SELECTOR_EVEN16_UM                7
#define ECATCM_RT_SELECTOR_ODD16_UM                 8
#define ECATCM_RT_SELECTOR_EVEN32_UM                9
#define ECATCM_RT_SELECTOR_ODD32_UM                 10
#define ECATCM_RT_SELECTOR_64BIT_UM                 11

//***************************************************************************
// Data structures

typedef struct
{
    void *          pvDataAddress;
    UWORD           uwSelector;
    UWORD ( *   fpfUmConv )( BOOL, void *, void * );
} ECATMGR_RT_PDOFASTENTRY_ELEMENT;

//***************************************************************************
// Globals

extern BOOL bEcatCMRTReSyncEnable;
extern BOOL bEcatCMRTPdoError;
extern BOOL bEcatCMRTDelayTxPdo;

extern ECATMGR_RT_PDOFASTENTRY_ELEMENT *   ptEcatCMRTPdoRxElemList;
extern UWORD                               uwEcatCMRTPdoRxWordBufSize;
extern ECATMGR_RT_PDOFASTENTRY_ELEMENT *   ptEcatCMRTPdoTxElemList;
extern UWORD                               uwEcatCMRTPdoTxWordBufSize;

//***************************************************************************
// Global functions

BOOL EcatCM_RT_RxDecode(ECATMGR_RT_PDOFASTENTRY_ELEMENT * psElem, HPVOID hpvSrcData, UWORD uwEscWordSz);
BOOL EcatCM_RT_TxEncode(ECATMGR_RT_PDOFASTENTRY_ELEMENT * psElem, HPVOID hpvSrcData, UWORD uwEscWordSz);
BOOL EcatCM_RT_SyncEvent(void);
BOOL EcatCM_RT_SyncPdoProcessing(void);

#endif
