/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : CanOpenPdoSyncMgrRT.c                                      */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : CanOpen PDO and SYNC manager, time-optimized for           */
/*               the platform                                               */
/*                                                                          */
/****************************************************************************/


#include "system\SysAppConfig.h"
#if CFG_CANDRV_DS301
#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "common\CommonParamDB.h"
#include "CanOpenPdoSyncMgr.h"
#include "CanOpenCommandMgr.h"
#include "system\SysLogManagement.h"
#include "system\SystemAlarms.h"
#include "CanOpenDs301.h"
#include "common\TaskScheduler.h"
//#include <intrins.h>

//***************************************************************************
// Avoids warning C113: nonstandard extension
// optimize for speed

//#pragma warning disable = 113
//#pragma ot(7,speed)

//***************************************************************************
// Locals
#pragma GCC optimize (2)

static BOOL bSyncCobReceived=FALSE;
static BOOL bSyncSent=FALSE;
static BOOL bSyncTriggerTS=FALSE;
static BOOL bSyncTriggerSendCob=FALSE;
static UWORD uwSyncPrevTS;

static void CanOpenPSM_RT_SyncTxCobProcessing(void);
static void CanOpenPSM_RT_PostSyncTxCob(CANDRV_COB * psCob);

//***************************************************************************
// RX PDO decoder

void CanOpenPSM_RT_RxDecode(CANOPENPSM_PDOFASTENTRY_RX * psPdoDef)
{
    CANDRV_PCOB pCob;
    union
    {
        UBYTE b[COMMONPARAMDB_BASETYPE_MAXSIZE];
        UWORD w[COMMONPARAMDB_BASETYPE_MAXSIZE/2];
        ULONG l[COMMONPARAMDB_BASETYPE_MAXSIZE/4];
        UQWRD q;
    } ud;

    pCob=psPdoDef->psCob;

    if(pCob->flags.bNewRxCob)//if(pCob->flags&CANDRV_F_NEWCOB)
    {
        if(pCob->len!=psPdoDef->ubCobLen)
            CanOpenCM_CanFaultSignal((ULONG)DS301_COMMERR_PDOLENGHTERROR);
        else
        {
            CANOPENPSM_PDOFASTENTRY_ELEMENT * psElem;
            UWORD uwCt;
            register UBYTE * pubDataIn;
            //UWORD * puwData;

            pubDataIn=(UBYTE *)pCob->d;
            //puwData = pCob->d;

            for(uwCt=0,psElem=psPdoDef->psElements;uwCt<psPdoDef->uwNElements;uwCt++,psElem++)
            {
                register UBYTE * pubDataOut;
                pubDataOut=(UBYTE *)psElem->pvDataAddress;
                switch(psElem->uwSelector)
                {
                    case CANOPENPSM_SELECTOR_8BIT:
                        *pubDataOut=*pubDataIn++;
                        break;

                    case CANOPENPSM_SELECTOR_EVEN16:
//                        _atomic_(0);
                        *((UWORD *)pubDataOut)= pCob->d[uwCt];//*((UWORD *)pubDataIn)++;
//                        _endatomic_();
                        break;

                    case CANOPENPSM_SELECTOR_ODD16:
                        ud.b[0]=*pubDataIn++;
                        ud.b[1]=*pubDataIn++;
//                        _atomic_(0);
                        *(UWORD *)pubDataOut=ud.w[0];
//                        _endatomic_();
                        break;

                    case CANOPENPSM_SELECTOR_EVEN32:
                        //_atomic_(0);
                        ((UWORD *)pubDataOut)[0]=pCob->d[uwCt*2];  //*((UWORD *)pubDataIn)++;
                        ((UWORD *)pubDataOut)[1]=pCob->d[uwCt*2 + 1];//*((UWORD *)pubDataIn)++;
                        //_endatomic_();
                        break;

                    case CANOPENPSM_SELECTOR_ODD32:
                        ud.b[0]=*pubDataIn++;
                        ud.b[1]=*pubDataIn++;
                        ud.b[2]=*pubDataIn++;
                        ud.b[3]=*pubDataIn++;
//                        _atomic_(0);
                        *(ULONG *)pubDataOut=ud.l[0];
//                        _endatomic_();
                        break;

                    case CANOPENPSM_SELECTOR_64BIT:
                        atomic_write(pubDataOut, pubDataIn, sizeof(UQWRD));
                        break;

                    case CANOPENPSM_SELECTOR_DISPLACEMENT:
                        pubDataIn=(UBYTE *)((UWORD)pubDataIn+(UWORD)pubDataOut);
                        break;

                    case CANOPENPSM_SELECTOR_8BIT_UM:
                        if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE,psElem->pvDataAddress,pubDataIn++))
                            goto hookerror;
                        break;

                    case CANOPENPSM_SELECTOR_EVEN16_UM:
                        ud.w[0]= pCob->d[uwCt];  //*((UWORD *)pubDataIn)++;
                        if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE,psElem->pvDataAddress,ud.b))
                            goto hookerror;
                        break;

                    case CANOPENPSM_SELECTOR_ODD16_UM:
                        ud.b[0]=*pubDataIn++;
                        ud.b[1]=*pubDataIn++;
                        if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE,psElem->pvDataAddress,ud.b))
                            goto hookerror;
                        break;

                    case CANOPENPSM_SELECTOR_EVEN32_UM:
                        ud.w[0]=pCob->d[uwCt*2];//*((UWORD *)pubDataIn)++;
                        ud.w[1]=pCob->d[uwCt*2 +1];//*((UWORD *)pubDataIn)++;
                        if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE,psElem->pvDataAddress,ud.b))
                            goto hookerror;
                        break;

                    case CANOPENPSM_SELECTOR_ODD32_UM:
                        ud.b[0]=*pubDataIn++;
                        ud.b[1]=*pubDataIn++;
                        ud.b[2]=*pubDataIn++;
                        ud.b[3]=*pubDataIn++;
                        if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE,psElem->pvDataAddress,ud.b))
                            goto hookerror;
                        break;

                    case CANOPENPSM_SELECTOR_64BIT_UM:
                        if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE,psElem->pvDataAddress,pubDataIn))
                            goto hookerror;
                        break;
                }
            }

            if(pCob->flags.bOverRun)//if(pCob->flags&CANDRV_F_OVERRUN)
                CanOpenCM_CanFaultSignal((ULONG)DS301_COMMERR_CANOVERRUN);

            (*(psPdoDef->puwCounter))++;
            pCob->flags.bNewRxCob = FALSE;
            pCob->flags.bOverRun = FALSE;//fast_atomic_clear_bits(pCob->flags, CANDRV_F_NEWCOB|CANDRV_F_OVERRUN);
        }
    }

    return;

 hookerror:       
    CanOpenCM_CanFaultSignal(CANOPENPSM_PE_MASK(CANOPENPSM_PDOERR_HOOKPROCESSINGFAIL));

    return;
}

//***************************************************************************
// TX PDO encoder

void CanOpenPSM_RT_TxEncode(CANOPENPSM_PDOFASTENTRY_TX * psPdoDef)
{
    union
    {
        UBYTE b[COMMONPARAMDB_BASETYPE_MAXSIZE];
        UWORD w[COMMONPARAMDB_BASETYPE_MAXSIZE/2];
        ULONG l[COMMONPARAMDB_BASETYPE_MAXSIZE/4];
        UQWRD q;
    } ud;

    register CANDRV_PCOB pCob=psPdoDef->psCob;

        // check if yet has to be transmit
    if((pCob->flags.bTxOK)==FALSE)//if((pCob->flags&CANDRV_F_TXOK)==0)
    {
        CanOpenCM_CanFaultSignal((ULONG)DS301_COMMERR_CAN_TXOVERRUN);
        return;
    }

        // by default take RTR triggered flag
    if(psPdoDef->f.bStRTRTriggered)
        psPdoDef->f.bStTxTrigger=TRUE;

    {
            // if sync cyclic 1-240
        if(psPdoDef->f.bCfgSyncCnt)
                // decrement and check # sync counter
            if(--psPdoDef->ubSyncCnt == 0)
            {
                    // reload # sync counter
                psPdoDef->ubSyncCnt=psPdoDef->ubNSync;

                psPdoDef->f.bStTxTrigger=TRUE;
            }
    }

        // if no compare option and not yet triggered then exit
    if(!psPdoDef->f.bCfgCompare && !psPdoDef->f.bStTxTrigger)
        return;

    {
        CANOPENPSM_PDOFASTENTRY_ELEMENT * psElem;
        UWORD uwCt;
        register UBYTE * pubDataOut;
    
            // if sync acyclic or async event then prepare data in
            // local buffer for late comparison, if just triggered
            // then escape the comparison
        if(!psPdoDef->f.bStTxTrigger)
            pubDataOut=(UBYTE *)psPdoDef->uwCmpDat;
        else
            pubDataOut=(UBYTE *)pCob->d;

        for(uwCt=0,psElem=psPdoDef->psElements;uwCt<psPdoDef->uwNElements;uwCt++,psElem++)
        {
            register UBYTE * pubDataIn;
            pubDataIn=(UBYTE *)psElem->pvDataAddress;
            switch(psElem->uwSelector)
            {
                case CANOPENPSM_SELECTOR_8BIT:
                    *pubDataOut++=*pubDataIn;
                    break;
    
                case CANOPENPSM_SELECTOR_EVEN16:
//                    _atomic_(0);
                    *((UWORD *)pubDataOut) =*((UWORD *)pubDataIn);
                    pubDataOut++;
                    pubDataOut++;
//                    _endatomic_();
                    break;
    
                case CANOPENPSM_SELECTOR_ODD16:
//                    _atomic_(0);
                    ud.w[0]=*(UWORD *)pubDataIn;
//                    _endatomic_();
                    *pubDataOut++=ud.b[0];
                    *pubDataOut++=ud.b[1];
                    break;
    
                case CANOPENPSM_SELECTOR_EVEN32:
//                    _atomic_(0);
                    ((UWORD *)pubDataOut)[0]=((UWORD *)pubDataIn)[0];
                    ((UWORD *)pubDataOut)[1]=((UWORD *)pubDataIn)[1];
//                    _endatomic_();
                    pubDataOut=(UBYTE *)((UWORD)pubDataOut+sizeof(ULONG));
                    break;
    
                case CANOPENPSM_SELECTOR_ODD32:
//                    _atomic_(0);
                    ud.l[0]=*(ULONG *)pubDataIn;
//                    _endatomic_();
                    *pubDataOut++=ud.b[0];
                    *pubDataOut++=ud.b[1];
                    *pubDataOut++=ud.b[2];
                    *pubDataOut++=ud.b[3];
                    break;
    
                case CANOPENPSM_SELECTOR_64BIT:
                    atomic_read(pubDataOut, pubDataIn, sizeof(UQWRD));
                    break;
    
                case CANOPENPSM_SELECTOR_DISPLACEMENT:
                    pubDataOut=(UBYTE *)((UWORD)pubDataOut+(UWORD)pubDataIn);
                    break;
    
                case CANOPENPSM_SELECTOR_8BIT_UM:
                    if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_READ,pubDataOut++,psElem->pvDataAddress))
                        goto hookerror;
                    break;
    
                case CANOPENPSM_SELECTOR_EVEN16_UM:
                    if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_READ,ud.b,psElem->pvDataAddress))
                        goto hookerror;
                    *((UWORD *)pubDataOut)=ud.w[0];
                    pubDataOut++;
                    pubDataOut++;
                    break;
    
                case CANOPENPSM_SELECTOR_ODD16_UM:
                    if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_READ,ud.b,psElem->pvDataAddress))
                        goto hookerror;
                    *pubDataOut++=ud.b[0];
                    *pubDataOut++=ud.b[1];
                    break;
    
                case CANOPENPSM_SELECTOR_EVEN32_UM:
                    if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_READ,ud.b,psElem->pvDataAddress))
                        goto hookerror;
                    *((UWORD *)pubDataOut)=ud.w[0];
                    pubDataOut++;
                    pubDataOut++;
                    *((UWORD *)pubDataOut)=ud.w[1];
                    pubDataOut++;
                    pubDataOut++;
                    break;
    
                case CANOPENPSM_SELECTOR_ODD32_UM:
                    if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_READ,ud.b,psElem->pvDataAddress))
                        goto hookerror;
                    *pubDataOut++=ud.b[0];
                    *pubDataOut++=ud.b[1];
                    *pubDataOut++=ud.b[2];
                    *pubDataOut++=ud.b[3];
                    break;

                case CANOPENPSM_SELECTOR_64BIT_UM:
                    if((*psElem->fpfUmConv)(COMMONPARAMDB_UCFLAG_READ,pubDataOut,psElem->pvDataAddress))
                        goto hookerror;
                    break;
            }
        }
    }

        // if not yet triggered
    if(!psPdoDef->f.bStTxTrigger)
    {
        register UWORD * puwSrc=&psPdoDef->uwCmpDat[0];
        register UWORD * puwDst=&pCob->d[0];

        if( puwDst[0]!=puwSrc[0] || puwDst[1]!=puwSrc[1] || \
            puwDst[2]!=puwSrc[2] || puwDst[3]!=puwSrc[3] )
        {
            *puwDst++ = *puwSrc++;
            *puwDst++ = *puwSrc++;
            *puwDst++ = *puwSrc++;
            *puwDst++ = *puwSrc++;

            psPdoDef->f.bStTxTrigger=TRUE;
        }
    }

        // if triggered prepare COB for CAN driver
    if(psPdoDef->f.bStTxTrigger)
    	memset(&(pCob->flags),0,sizeof(pCob->flags));//pCob->flags=CANDRV_F_DEFAULT;

    psPdoDef->f.bStRTRTriggered=FALSE;
    return;

 hookerror:       
    CanOpenCM_CanFaultSignal(CANOPENPSM_PE_MASK(CANOPENPSM_PDOERR_HOOKPROCESSINGFAIL));

    psPdoDef->f.bStTxTrigger=psPdoDef->f.bStRTRTriggered=FALSE;
    return;
}

//***************************************************************************
// sync event from sync sending

void CanOpenPSM_RT_SyncSentEvent(UWORD uwNode)
{
    bSyncSent = (uwCanOpenPSMCanController==uwNode) & bCanOpenPSMSyncEvOnPeriodicCob;
}

//***************************************************************************
// cob sync event

void CanOpenPSM_RT_CobSyncEvent(CANDRV_COB * pCob)
{
    if(bCanOpenPSMReSyncEnable)
    {
        uwSysFieldbusSampleTS=uwSysFieldbusTSTimer;
        uwSysFieldbusSampleRT=uwSysFieldbusRTTimer;
    }

    pCob->flags.bNewRxCob = FALSE;//fast_atomic_clear_bits(pCob->flags, CANDRV_F_NEWCOB);
    bSyncCobReceived=TRUE;
}

//***************************************************************************
// real time task for sync event and command manager

BOOL CanOpenPSM_RT_SyncEvent(void)
{
        // if pwm resync enabled, trigger resync calculation in the
        // rt cycle after the fbus sync cycle, in order to save time
    if(bCanOpenPSMReSyncEnable)
        bSysStatFieldbusTSReSync=bSyncTriggerTS;

        // if send TX cob trigger the first TX PDO
    if(bSyncTriggerSendCob)
    {
        CanOpenPSM_RT_PostSyncTxCob(NULL);
        bSyncTriggerSendCob=FALSE;
    }

        // delayed tx cob processing after sync cycle
    if(bSysStatCANOpenSync && bCanOpenPSMPdoEnabled && bCanOpenPSMDelayTxPdo)
        CanOpenPSM_RT_SyncTxCobProcessing();

        // just set true for one real time task the fieldbus sync flag
        // resync calculation only triggered for real sync, disabled
        // when sync sending enabled
    if(bSyncCobReceived || bSyncSent)
    {
        bSysStatCANOpenSync=bSysStatFieldbusSyncing=TRUE;
        bSyncTriggerTS=bSyncCobReceived;
        bSyncCobReceived=bSyncSent=FALSE;
    }
    else
        bSysStatCANOpenSync=bSysStatFieldbusSyncing=bSyncTriggerTS=FALSE;

        // check alarm clearing flag
    bCanOpenCMFaultClear|=bSysStatAlarmsReset;

    return TRUE;
}

//***************************************************************************
// Realtime task PDO manager activated on fieldbus sync

BOOL CanOpenPSM_RT_SyncPdoProcessing(void)
{
    if(bCanOpenPSMPdoEnabled)
    {
        CANOPENPSM_PDOFASTENTRY_RX ** psRxP=psCanOpenPSMSyncRxLst;

            // process rx cob
        while(*psRxP)
            CanOpenPSM_RT_RxDecode(*psRxP++);

            // standard tx cob processing
        if(!bCanOpenPSMDelayTxPdo)
            CanOpenPSM_RT_SyncTxCobProcessing();
    }

        // first SYNC flag, in order to start SYNC timing check
    bCanOpenPSMFirstSyncRecv=TRUE;

        // calc time between syncs
    uwCanOpenPSMSyncTiming=uwSysTimers125us-uwSyncPrevTS;
    uwSyncPrevTS=uwSysTimers125us;

    return TRUE;
}

//***************************************************************************
// Processing of Tx PDOs

static void CanOpenPSM_RT_SyncTxCobProcessing(void)
{
    CANOPENPSM_PDOFASTENTRY_TX ** psTxP=psCanOpenPSMSyncTxLst;

        // process tx cob
    while(*psTxP)
        CanOpenPSM_RT_TxEncode(*psTxP++);

        // then trigger event for the 1st tx cob
    psCanOpenPSMSyncCobTxP=psCanOpenPSMSyncTxLst;
    bSyncTriggerSendCob=TRUE;
}

//***************************************************************************
// Event for tx pdo marked for tx after sync event

static void CanOpenPSM_RT_PostSyncTxCob(CANDRV_COB *cob)
{
        // if enabled
    if(psCanOpenPSMSyncCobTxP)
    {
            // scan all sync tx cob and send the ones selected for tx
        while(*psCanOpenPSMSyncCobTxP)
        {
            if((*psCanOpenPSMSyncCobTxP)->f.bStTxTrigger)
            {
                    // send one cob per time
                candrv_sendcobwcb(uwCanOpenPSMCanController, (*psCanOpenPSMSyncCobTxP)->psCob, &CanOpenPSM_RT_PostSyncTxCob);
                (*psCanOpenPSMSyncCobTxP)->f.bStTxTrigger=FALSE;
                psCanOpenPSMSyncCobTxP++;

                return;
            }
            psCanOpenPSMSyncCobTxP++;
        }
        psCanOpenPSMSyncCobTxP=NULL;
    }
}

#endif
