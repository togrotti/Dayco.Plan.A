/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : CanOpenPdoSyncMgr.h                                        */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : CanOpen PDO and SYNC manager, time-optimized for           */
/*               the platform                                               */
/*                                                                          */
/****************************************************************************/

#ifndef _CANOPENPDOSYNCMGR_H
#define _CANOPENPDOSYNCMGR_H

#include "common\CommonDefines.h"
#include "system\SysAppGlobals.h"
#include "system\SysAppConfig.h"
#include "MultiCANController.h"
#include "CanOpenDs301.h"

//***************************************************************************
// Defines

#define CANOPENPSM_PAR_RX                             1
#define CANOPENPSM_PAR_TX                             2

#define CANOPENPSM_NSUBINDX_RX                        2
#define CANOPENPSM_NSUBINDX_TX                        6

#define CANOPENPSM_PDOERR_INVALIDCOBID               -1
#define CANOPENPSM_PDOERR_RTRNOTVALID                -2
#define CANOPENPSM_PDOERR_INVALIDTXTYPE              -3
#define CANOPENPSM_PDOERR_INVALIDMAPCOUNT            -4
#define CANOPENPSM_PDOERR_OUTOFMEMORY                -5
#define CANOPENPSM_PDOERR_INVALIDSYNCSTARTVALUE      -6
#define CANOPENPSM_PDOERR_INTERNALERROR              -7
#define CANOPENPSM_PDOERR_PDOLENGTHEXCEED            -8
#define CANOPENPSM_PDOERR_HOOKPROCESSINGFAIL         -9
#define CANOPENPSM_PDOERR_DUPLICATEDCOBID           -10

#define CANOPENPSM_PE_MASK(x)                       (0x10000<<(-(x)-1))

//***************************************************************************
// PDO Fast Entry data types for time optimized runtime

#define CANOPENPSM_SELECTOR_8BIT                    0
#define CANOPENPSM_SELECTOR_EVEN16                  1
#define CANOPENPSM_SELECTOR_ODD16                   2
#define CANOPENPSM_SELECTOR_EVEN32                  3
#define CANOPENPSM_SELECTOR_ODD32                   4
#define CANOPENPSM_SELECTOR_64BIT                   5
#define CANOPENPSM_SELECTOR_DISPLACEMENT            6
#define CANOPENPSM_SELECTOR_8BIT_UM                 7
#define CANOPENPSM_SELECTOR_EVEN16_UM               8
#define CANOPENPSM_SELECTOR_ODD16_UM                9
#define CANOPENPSM_SELECTOR_EVEN32_UM               10
#define CANOPENPSM_SELECTOR_ODD32_UM                11
#define CANOPENPSM_SELECTOR_64BIT_UM                12

//#define _atomic_(0)
//#define _endatomic_()
typedef struct
{
    UWORD           uwSelector;
    void *          pvDataAddress;
    UWORD (* fpfUmConv )( BOOL, void *, void * );
} CANOPENPSM_PDOFASTENTRY_ELEMENT;

typedef struct
{
    UBYTE           ubCobLen;
    UWORD           uwNElements;
    CANOPENPSM_PDOFASTENTRY_ELEMENT * psElements;
    CANDRV_PCOB     psCob;
    UWORD *         puwCounter;
} CANOPENPSM_PDOFASTENTRY_RX;

typedef struct
{
    UWORD           uwCmpDat[4];
    struct
    {
        UBYTE       bCfgSyncCnt:1;
        UBYTE       bCfgCompare:1;

        UBYTE       bStTxTrigger:1;
        UBYTE       bStRTRTriggered:1;
    } f;
    UBYTE           ubNSync;
    UBYTE           ubSyncCnt;
    UWORD           uwNElements;
    CANOPENPSM_PDOFASTENTRY_ELEMENT * psElements;
    CANDRV_PCOB     psCob;
    CANDRV_PCOB     psRTRCob;
} CANOPENPSM_PDOFASTENTRY_TX;

//***************************************************************************
// Globals

extern CANOPENPSM_PDOFASTENTRY_RX ** psCanOpenPSMSyncRxLst;
extern CANOPENPSM_PDOFASTENTRY_TX ** psCanOpenPSMSyncTxLst;
extern BOOL bCanOpenPSMPdoEnabled;
extern BOOL bCanOpenPSMReSyncEnable;
extern BOOL bCanOpenPSMSyncEvOnPeriodicCob;
extern BOOL bCanOpenPSMFirstSyncRecv;
extern BOOL bCanOpenPSMDelayTxPdo;
extern UWORD                         uwCanOpenPSMSyncTiming;
extern UWORD                         uwCanOpenPSMCanController;
extern CANOPENPSM_PDOFASTENTRY_TX ** psCanOpenPSMSyncCobTxP;

extern UWORD                         uwCanOpenPSMRxPdoCnt[CFG_DS301_PDO_RX_TOT];

//***************************************************************************
// Global functions

#ifdef _APP_DEBUG
BOOL CanOpenPSMCheckDBTable(void);
#endif

BOOL CanOpenPSM_Init(UWORD, BOOL, BOOL, BOOL);
void CanOpenPSM_Loop(void);

//***************************************************************************
// Global functions from time optimized module

void CanOpenPSM_RT_RxDecode(CANOPENPSM_PDOFASTENTRY_RX *);
void CanOpenPSM_RT_TxEncode(CANOPENPSM_PDOFASTENTRY_TX *);
void CanOpenPSM_RT_CobSyncEvent(CANDRV_COB *);
void CanOpenPSM_RT_SyncSentEvent(UWORD);
BOOL CanOpenPSM_RT_SyncEvent(void);
BOOL CanOpenPSM_RT_SyncPdoProcessing(void);

//***************************************************************************
// Create/destroy/setdefaultcobid PDO events

BOOL CanOpenPSM_CreatePdoEvent(UBYTE);
void CanOpenPSM_DestroyPdoEvent(UBYTE);
void CanOpenPSM_SetDefaultCobID(UBYTE,UBYTE);

//***************************************************************************
// DB Hooks

UWORD CanOpenPSM_PdoParam(COMMONPARAMDB_ENTRY * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext);
UWORD CanOpenPSM_PdoMap(COMMONPARAMDB_ENTRY * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext);

#endif
