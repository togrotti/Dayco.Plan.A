/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ECATCommandMgr.h                                           */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : ECAT specific command manager                              */
/*                                                                          */
/****************************************************************************/

#ifndef _ECATCOMMANDMGR_H
#define _ECATCOMMANDMGR_H

#include "ECATHwCommon.h"
#include "ecat_def.h"
#include "system\SysAppConfig.h"
#include "common\CommonParamDB.h"
#include "common\CommonController.h"
#include "bus\canopen\CanOpenDs301.h"

//***************************************************************************
// Defines

#define ECATCM_INIT_ALWAYS                  8
#define ECATCM_INIT_CORE                    0
#define ECATCM_INIT_SYNCEVENT               1
#define ECATCM_INIT_SYNCPDOEVENT            2

#define ECATCM_NO_OF_SYNCMANAGER            4
#define ECATCM_MAXNPDO                      (CFG_DS301_PDO_TX_TOT>CFG_DS301_PDO_RX_TOT?CFG_DS301_PDO_TX_TOT:CFG_DS301_PDO_RX_TOT)

#define ECATCM_PAR_RX                       1
#define ECATCM_PAR_TX                       2

//***************************************************************************
// Data structures

typedef struct
{
    union
    {
        struct
        {
          BOOL bEnableModule;  //UWORD bEnableModule:1;          // enable ECAT module
          BOOL bReSyncEnable;  //UWORD bReSyncEnable:1;          // enable pwm fieldbus resync
          BOOL bDelayedInputs;  //UWORD bDelayedInputs:1;         // process Tx PDO/Inputs one rt-cycle delayed
          BOOL bDisableAlarms;  //UWORD bDisableAlarms:1;         // disable emcy and fault signaling triggering
        } f;
//        UWORD w;
    } flags;
    UWORD       uwConfStatAlias;            // ESC specific parameters managed
    SWORD       swExecDelay;                // by master
    SWORD       swPort0Delay;
    SWORD       swPort1Delay;
#ifdef _INFINEON_
} ECATCM_PARAM;
#else
} __attribute__((aligned(4))) ECATCM_PARAM;
#endif

typedef union
{
    UNSIGNED32                      dMap;
    struct
    {
        UNSIGNED8                   bLength;
        UNSIGNED8                   bSubIndex;
        UNSIGNED16                  wIndex;
    } f;
} ECATCM_PDO_MAP_ENTRY;

typedef struct
{
    UBYTE       NoMapped;                               // numero oggetti mappati
    ECATCM_PDO_MAP_ENTRY Map[CFG_DS301_MAXDATAOBJECT];  // oggetti mappato
} ECATCM_PDO_MAPPING;

typedef struct
{
    UBYTE       NoMapped;                   // numero oggetti mappati
    UWORD       Map[ECATCM_MAXNPDO];        // PDO mappato
#ifdef _INFINEON_
} ECATCM_SYNCMGR_PDOMAPPING;
#else
} __attribute__((aligned(4))) ECATCM_SYNCMGR_PDOMAPPING;
#endif

typedef struct
{
    ULONG       u32ShiftTime;
    UWORD       u16SyncTypesSupported;
    ULONG       u32MinCycleTime;
    ULONG       u32CalcAndCopyTime;
    ULONG       u32DelayTime;
    ULONG       u32Sync0CycleTime;
    ULONG       u32CycleTimeTooSmall;
    ULONG       u32SMEventMissed;
    CHARS       bSyncError;
} ECATCM_TSYNCMANDIAG;

typedef struct
{
    UWORD       u16SyncType;
    ULONG       u32CycleTime;
} ECATCM_TSYNCMANPAR;

typedef struct
{
    UWORD       smEventMissedCounter;
    UWORD       shiftTooShortCounter;
    UWORD       cycleExceededCounter;
    UWORD       syncFailedCounter;
} ECATCM_TCYCLEDIAG;

typedef struct
{
    const COMCTRL_HANDLERS *psCtrlHandlers  ;
} ECATCM_IN;

//***************************************************************************
// Globals

extern ECATCM_PARAM tEcatCMParam;
extern const ECATCM_PARAM  tEcatCMDefParam;
extern const UBYTE tEcatCMSyncMgrCommType[ECATCM_NO_OF_SYNCMANAGER];
extern ECATCM_PDO_MAPPING  tEcatCMTxPdoMap[CFG_DS301_PDO_TX_TOT];
extern const ECATCM_PDO_MAPPING  tEcatCMDefTxPdoMap[CFG_DS301_PDO_TX_TOT];
extern ECATCM_PDO_MAPPING  tEcatCMRxPdoMap[CFG_DS301_PDO_RX_TOT];
extern const ECATCM_PDO_MAPPING  tEcatCMDefRxPdoMap[CFG_DS301_PDO_RX_TOT];
extern ECATCM_SYNCMGR_PDOMAPPING  tEcatCMOutSyncPdoMap;
extern const ECATCM_SYNCMGR_PDOMAPPING  tEcatCMDefOutSyncPdoMap;
extern ECATCM_SYNCMGR_PDOMAPPING  tEcatCMInSyncPdoMap;
extern const ECATCM_SYNCMGR_PDOMAPPING  tEcatCMDefInSyncPdoMap;
extern ECATCM_TSYNCMANDIAG tEcatCMSyncManOutDiag;
extern ECATCM_TSYNCMANDIAG tEcatCMSyncManInDiag;
extern ECATCM_TSYNCMANPAR tEcatCMSMOutParam;
extern ECATCM_TSYNCMANPAR tEcatCMSMInParam;
extern ECATCM_TCYCLEDIAG tEcatCMCycleDiag;
extern ECATCM_IN sEcatCMIn;
extern ECATCM_DIAG sEcatDiag;
extern UWORD uwEcatEmcyErrCode;
extern UBYTE ubEcatEmcyErrReg;

//***************************************************************************
// Global functions

BOOL EcatCM_Init(UWORD);
BOOL EcatCM_IsALStateMachineOP(void);
void EcatCM_GetHwOpt(UWORD, ULONG *, ULONG *);

//***************************************************************************
// DB Hooks

UWORD EcatCM_PdoMap(COMMONPARAMDB_ENTRY  * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext);
UWORD EcatCM_SyncMgrPdoMap(COMMONPARAMDB_ENTRY  * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext);

#endif
