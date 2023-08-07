/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ECATCommandMgrRT.c                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : ECAT time-optimized real time module                       */
/*                                                                          */
/****************************************************************************/

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "bus\SyncManager.h"
#include "common\TaskScheduler.h"
#include "ECATCommandMgr.h"
#include "ECATCommandMgrRT.h"
#include "fpga\FpgaHandler.h"
#include "ECATHw.h"
#include "ecatslv.h"
//#include <intrins.h>
#include <string.h>

#ifdef _INFINEON_
//***************************************************************************
// Avoids warning C113: nonstandard extension
// optimize for speed

#pragma warning disable = 113
#pragma ot(7,speed)
#else
/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)
#endif

//***************************************************************************
// Globals

BOOL bEcatCMRTReSyncEnable=FALSE;
BOOL bEcatCMRTPdoError=FALSE;
BOOL bEcatCMRTDelayTxPdo=FALSE;

ECATMGR_RT_PDOFASTENTRY_ELEMENT *   ptEcatCMRTPdoRxElemList;
UWORD                               uwEcatCMRTPdoRxWordBufSize;
ECATMGR_RT_PDOFASTENTRY_ELEMENT *   ptEcatCMRTPdoTxElemList;
UWORD                               uwEcatCMRTPdoTxWordBufSize;

//***************************************************************************
// Locals

static UBYTE pubProcBuffer[MAX_PD_OUTPUT_SIZE>MAX_PD_INPUT_SIZE?MAX_PD_OUTPUT_SIZE:MAX_PD_INPUT_SIZE];

static UWORD uwPrev1msTimer;

static void EcatCM_RT_TxPdoProcessing(void);

//***************************************************************************
// RX PDO decoder

BOOL EcatCM_RT_RxDecode(ECATMGR_RT_PDOFASTENTRY_ELEMENT * psElem, HPVOID hpvSrcData, UWORD uwEscWordSz)
{
    union
    {
        UBYTE b[COMMONPARAMDB_BASETYPE_MAXSIZE];
        UWORD w[COMMONPARAMDB_BASETYPE_MAXSIZE/2];
        ULONG l[COMMONPARAMDB_BASETYPE_MAXSIZE/4];
        UQWRD q;
    } ud;
    register UBYTE * pubDataIn=pubProcBuffer;

        // optimized buffer transfer from ESC
#ifdef _INFINEON_
    while(uwEscWordSz-->0)
        *((UWORD *)pubDataIn)++=*((UWORD *)hpvSrcData)++;
#else
    while(uwEscWordSz-->0)
    {
        *((UWORD *)pubDataIn)=*((UWORD *)hpvSrcData);
        pubDataIn+=2;hpvSrcData+=2;
    }
#endif

    pubDataIn=pubProcBuffer;
    while(psElem->pvDataAddress!=NULL)
    {
        register UBYTE * pubDataOut=(UBYTE *)psElem->pvDataAddress;

        switch(psElem->uwSelector)
        {
            case ECATCM_RT_SELECTOR_8BIT:
                *pubDataOut=*pubDataIn++;
                break;

            case ECATCM_RT_SELECTOR_EVEN16:
                *((UWORD *)pubDataOut)=*((UWORD *)pubDataIn); pubDataIn+=sizeof(UWORD);
                break;

            case ECATCM_RT_SELECTOR_ODD16:
                ud.b[0]=*pubDataIn++;
                ud.b[1]=*pubDataIn++;
//                _atomic_(0);
                *(UWORD *)pubDataOut=ud.w[0];
//                _endatomic_();
                break;

            case ECATCM_RT_SELECTOR_EVEN32:
                ud.w[0]=*((UWORD *)pubDataIn); pubDataIn+=sizeof(UWORD);
                ud.w[1]=*((UWORD *)pubDataIn); pubDataIn+=sizeof(UWORD);
//                _atomic_(0);
                *(ULONG *)pubDataOut=ud.l[0];
//                _endatomic_();
                break;

            case ECATCM_RT_SELECTOR_ODD32:
                ud.b[0]=*pubDataIn++;
                ud.b[1]=*pubDataIn++;
                ud.b[2]=*pubDataIn++;
                ud.b[3]=*pubDataIn++;
//                _atomic_(0);
                *(ULONG *)pubDataOut=ud.l[0];
//                _endatomic_();
                break;

            case ECATCM_RT_SELECTOR_64BIT:
                atomic_write(pubDataOut, pubDataIn, sizeof(UQWRD));
                pubDataIn=&pubDataIn[sizeof(UQWRD)];
                break;

            case ECATCM_RT_SELECTOR_8BIT_UM:
                ud.b[0]=*pubDataIn++;
                if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE,psElem->pvDataAddress,ud.b))
                    return FALSE;
                break;

            case ECATCM_RT_SELECTOR_EVEN16_UM:
                ud.w[0]=*((UWORD *)pubDataIn); pubDataIn+=sizeof(UWORD);
                if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE,psElem->pvDataAddress,ud.b))
                    return FALSE;
                break;

            case ECATCM_RT_SELECTOR_ODD16_UM:
                ud.b[0]=*pubDataIn++;
                ud.b[1]=*pubDataIn++;
                if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE,psElem->pvDataAddress,ud.b))
                    return FALSE;
                break;

            case ECATCM_RT_SELECTOR_EVEN32_UM:
                ud.w[0]=*((UWORD *)pubDataIn); pubDataIn+=sizeof(UWORD);
                ud.w[1]=*((UWORD *)pubDataIn); pubDataIn+=sizeof(UWORD);
                if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE,psElem->pvDataAddress,ud.b))
                    return FALSE;
                break;

            case ECATCM_RT_SELECTOR_ODD32_UM:
                ud.b[0]=*pubDataIn++;
                ud.b[1]=*pubDataIn++;
                ud.b[2]=*pubDataIn++;
                ud.b[3]=*pubDataIn++;
                if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE,psElem->pvDataAddress,ud.b))
                    return FALSE;
                break;

            case ECATCM_RT_SELECTOR_64BIT_UM:
                atomic_read(&ud.q, pubDataIn, sizeof(UQWRD));
                pubDataIn=&pubDataIn[sizeof(UQWRD)];
                if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE,psElem->pvDataAddress,ud.b))
                    return FALSE;
                break;
        }

        psElem++;
    }

    return TRUE;
}

//***************************************************************************
// TX PDO encoder

BOOL EcatCM_RT_TxEncode(ECATMGR_RT_PDOFASTENTRY_ELEMENT * psElem, HPVOID hpvDstData, UWORD uwEscWordSz)
{
    union
    {
        UBYTE b[COMMONPARAMDB_BASETYPE_MAXSIZE];
        UWORD w[COMMONPARAMDB_BASETYPE_MAXSIZE/2];
        ULONG l[COMMONPARAMDB_BASETYPE_MAXSIZE/4];
        UQWRD q;
    } ud;
    register UBYTE * pubDataOut=pubProcBuffer;

    while(psElem->pvDataAddress!=NULL)
    {
        register UBYTE * pubDataIn=(UBYTE *)psElem->pvDataAddress;

        switch(psElem->uwSelector)
        {
            case ECATCM_RT_SELECTOR_8BIT:
                *pubDataOut++=*pubDataIn;
                break;

            case ECATCM_RT_SELECTOR_EVEN16:
                *((UWORD *)pubDataOut)=*((UWORD *)pubDataIn); pubDataOut+=sizeof(UWORD);
                break;

            case ECATCM_RT_SELECTOR_ODD16:
//                _atomic_(0);
                ud.w[0]=*(UWORD *)pubDataIn;
//                _endatomic_();
                *pubDataOut++=ud.b[0];
                *pubDataOut++=ud.b[1];
                break;

            case ECATCM_RT_SELECTOR_EVEN32:
//                _atomic_(0);
                ud.l[0]=*(ULONG *)pubDataIn;
//                _endatomic_();
                *((UWORD *)pubDataOut)=ud.w[0]; pubDataOut+=sizeof(UWORD);
                *((UWORD *)pubDataOut)=ud.w[1]; pubDataOut+=sizeof(UWORD);
                break;

            case ECATCM_RT_SELECTOR_ODD32:
//                _atomic_(0);
                ud.l[0]=*(ULONG *)pubDataIn;
//                _endatomic_();
                *pubDataOut++=ud.b[0];
                *pubDataOut++=ud.b[1];
                *pubDataOut++=ud.b[2];
                *pubDataOut++=ud.b[3];
                break;

            case ECATCM_RT_SELECTOR_64BIT:
                atomic_read(pubDataOut, pubDataIn, sizeof(UQWRD));
                pubDataOut=&pubDataOut[sizeof(UQWRD)];
                break;

            case ECATCM_RT_SELECTOR_8BIT_UM:
                if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_READ,ud.b,psElem->pvDataAddress))
                    return FALSE;
                *pubDataOut++=ud.b[0];
                break;

            case ECATCM_RT_SELECTOR_EVEN16_UM:
                if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_READ,ud.b,psElem->pvDataAddress))
                    return FALSE;
                *((UWORD *)pubDataOut)=ud.w[0]; pubDataOut+=sizeof(UWORD);
                break;

            case ECATCM_RT_SELECTOR_ODD16_UM:
                if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_READ,ud.b,psElem->pvDataAddress))
                    return FALSE;
                *pubDataOut++=ud.b[0];
                *pubDataOut++=ud.b[1];
                break;

            case ECATCM_RT_SELECTOR_EVEN32_UM:
                if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_READ,ud.b,psElem->pvDataAddress))
                    return FALSE;
                *((UWORD *)pubDataOut)=ud.w[0]; pubDataOut+=sizeof(UWORD);
                *((UWORD *)pubDataOut)=ud.w[1]; pubDataOut+=sizeof(UWORD);
                break;

            case ECATCM_RT_SELECTOR_ODD32_UM:
                if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_READ,ud.b,psElem->pvDataAddress))
                    return FALSE;
                *pubDataOut++=ud.b[0];
                *pubDataOut++=ud.b[1];
                *pubDataOut++=ud.b[2];
                *pubDataOut++=ud.b[3];
                break;

            case ECATCM_RT_SELECTOR_64BIT_UM:
                if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_READ,ud.b,psElem->pvDataAddress))
                    return FALSE;
                atomic_write(pubDataOut, &ud.q, sizeof(UQWRD));
                pubDataOut=&pubDataOut[sizeof(UQWRD)];
                break;
        }

        psElem++;
    }

        // optimized buffer transfer to ESC
    pubDataOut=pubProcBuffer;
#ifdef _INFINEON_
    while(uwEscWordSz-->0)
        *((UWORD huge *)hpvDstData)++=*((UWORD *)pubDataOut)++;
#else
    while(uwEscWordSz-->0)
    {
        *((UWORD *)hpvDstData)=*((UWORD *)pubDataOut);
        hpvDstData+=2;pubDataOut+=2;
    }
#endif

    return TRUE;
}

//***************************************************************************
// real time task for sync event and command manager

BOOL EcatCM_RT_SyncEvent(void)
{
    UWORD uwDummy;
    BOOL bSyncCobReceived;
//    BOOL bPrevFieldbusSyncing;

        // get irq latch status
    bSyncCobReceived=FPGA_ECATREGS_STAT_IRQLATCH;

        // if pwm resync enabled, trigger resync calculation in the
        // rt cycle after the fbus sync cycle, in order to save time
    if(bEcatCMRTReSyncEnable)
        bSysStatFieldbusTSReSync=bSysStatEtherCATSync;

        // delayed tx cob processing after sync cycle
    if(bSysStatEtherCATSync && bEcatCMRTDelayTxPdo)
        EcatCM_RT_TxPdoProcessing();

        // reset syncing status
    bSysStatEtherCATSync=bSysStatFieldbusSyncing=FALSE;

        // just set true for one real time task the fieldbus sync flag
    if(bSyncCobReceived)
    {
            // get AL event
    	EscAlEvent.Word[0]=ESC_UINT16(ESC_ADDR_ALEVENT);

            // if output event or, if output disabled, check
            // for input event, if SYNC0 always true
        if( (EscAlEvent.Word[0]&(PROCESS_OUTPUT_EVENT|SYNC0_EVENT)) ||
            (nPdOutputSize==0 && (EscAlEvent.Word[0]&(PROCESS_INPUT_EVENT|SYNC0_EVENT))) )
        {
                // setup first received valid and reset wdt if output received
            if((EscAlEvent.Word[0]&PROCESS_OUTPUT_EVENT) || nPdOutputSize==0)
            {
                bEcatFirstOutputsReceived=TRUE;
                u16WdCounter=0;
                // signal fieldbus syncing only if data is arrived
                // or if output is unavailable
                bSysStatFieldbusSyncing=TRUE;
            }
            // sync error if no output data available
            else
                sEcatDiag.uwSyncOutputMissingCnt++;

                // signal ecat resyncing
            bSysStatEtherCATSync=TRUE;
        }
    }

        // if pwm resync enabled get timer timestamp
    if(bEcatCMRTReSyncEnable && bSysStatEtherCATSync)
    {
        union
        {
            UWORD w[2];
            ULONG l;
        } fbus,freerun;
        UWORD uwActualReload=SYNCMGR_FULLRELOAD_GET();

            // get right snapshot timer
        if(bDcSyncActive)
            fbus.l=RGAD_ECAT_SPSYNC0;
        else
            fbus.l=RGAD_ECAT_SPIRQ;

            // get actual freerun
        freerun.l=RGAD_ECAT_FREERUNTMR;

#ifdef _APP_XC
            // correct hi-res part to match local timer
        fbus.w[0]=_sint16_mpy_16_q14(fbus.w[0],uwSyncMgrAdjRTScaling);
#endif
            // split and correct timings
        if(fbus.w[0]>=(0xffff-uwActualReload))
            uwSysFieldbusSampleTS=0xffff;
        else
            uwSysFieldbusSampleTS=fbus.w[0]+uwActualReload;
        uwSysFieldbusSampleRT=fbus.w[1]-freerun.w[1]+uwSysFieldbusRTTimer;

            // setup statistics
        tEcatCMSyncManOutDiag.u32CalcAndCopyTime=(0x00010000l-(sSyncMgrDiagnosticOut.slLastSamplePoint&0x0000ffffl))*2;    // ~ 1/65536*125*1000
        tEcatCMSyncManInDiag.u32CalcAndCopyTime=1000000000/REALTIME_TASK_FREQ;
    }

        // if not syncing and 1st output received then service wdt every 1ms
    if(!bSysStatFieldbusSyncing && bEcatFirstOutputsReceived)
        if(uwPrev1msTimer!=uwSysTimers1ms)
        {
            uwPrev1msTimer=uwSysTimers1ms;
            ECAT_CheckWatchdog();
        }

        // if DC sync enabled sync received or 1st output still not received, reset irq mask here
    if(bDcSyncActive && ((!bEcatFirstOutputsReceived) | bSysStatEtherCATSync))
//        memcpy(&uwDummy, (HPUWORD)&(ESC_UINT16(ESC_ADDR_SYNC_STATUS)), 2);
        uwDummy = ESC_UINT16(ESC_ADDR_SYNC_STATUS);

    return TRUE;
}

//***************************************************************************
// Realtime task PDO manager activated on fieldbus sync

BOOL EcatCM_RT_SyncPdoProcessing(void)
{
    UWORD uwDummy;

        // process rx cob if event and enabled
    if(EscAlEvent.Word[0]&PROCESS_OUTPUT_EVENT)
    {
        if(bEcatOutputUpdateRunning)
            bEcatCMRTPdoError|=!EcatCM_RT_RxDecode(ptEcatCMRTPdoRxElemList,(HPVOID)(FPGA_ETHERNET_BASE_ADDRESS+nEscAddrOutputData),uwEcatCMRTPdoRxWordBufSize);

            // if disabled read first and last byte of buffer in order to reset event mask
        else
        {
            memcpy(&uwDummy, (HPVOID)(FPGA_ETHERNET_BASE_ADDRESS+nEscAddrOutputData), 1);
            memcpy(&uwDummy, (HPVOID)(FPGA_ETHERNET_BASE_ADDRESS+nEscAddrOutputData+nPdOutputSize-1), 1);
        }
    }

        // standard tx cob processing
    if(!bEcatCMRTDelayTxPdo)
        EcatCM_RT_TxPdoProcessing();

    return TRUE;
}

//***************************************************************************
// Tx COB handling (EtherCAT Inputs)

static void EcatCM_RT_TxPdoProcessing(void)
{
        // process tx cob if enabled and if DC sync event or if output or input event if DC disabled
    if( bEcatInputUpdateRunning && ((bDcSyncActive && (EscAlEvent.Word[0]&DC_EVENT_MASK)) ||
                                   (!bDcSyncActive && (EscAlEvent.Word[0]&(PROCESS_OUTPUT_EVENT|PROCESS_INPUT_EVENT)))) )
        bEcatCMRTPdoError|=!EcatCM_RT_TxEncode(ptEcatCMRTPdoTxElemList,(HPVOID)(FPGA_ETHERNET_BASE_ADDRESS+nEscAddrInputData),uwEcatCMRTPdoTxWordBufSize);

        // otherwise if here without sending input then increment SM missing counter
    else
        tEcatCMCycleDiag.smEventMissedCounter++;
}
