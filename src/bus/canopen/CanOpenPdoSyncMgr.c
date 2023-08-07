/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : CanOpenPdoSyncMgr.c                                        */
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
#include "system\SysAppGlobals.h"
#include "system\SystemStatus.h"
#include "CanOpenComDB.h"
#include "CanOpenDs301Externals.h"
#include "CanOpenDs301.h"
#include "CanOpenPdoSyncMgr.h"
#include "CanOpenCommandMgr.h"
#include "common\TaskScheduler.h"
#include "system\Os.h"
#include "bus\SyncManager.h"
//#include <intrins.h>
#include <math.h>
#include <string.h>

#pragma GCC optimize (2)
//***************************************************************************
// Avoids warning C37: nonstandard extension - unnamed parameter

//#pragma warning disable = 37

//***************************************************************************
// Macros

#define SYNC_TIMEOUTOFSYNC      4	// # of consecutive out of sync
                                    // before triggering alarm

//***************************************************************************
// PDO parameters

DS301_PDO_PARAM DS301_MEMQ_PARAM tDs301PdoParamTx[CFG_DS301_PDO_TX_TOT];

const DS301_PDO_PARAM DS301_MEMQ_PARAM tDs301PdoDefParamTx[CFG_DS301_PDO_TX_TOT]=
{
    {DS301_CREATEPDOCOBIDENTRY(0,1,DS301_COBID_PDO_TX+0x100*0+CFG_DS301_DEFNODEID), DS301_PDO_TYPE_ASYNCHRONOUS_STD, 1, 0, 0,
                {0x60410010}},
    {DS301_CREATEPDOCOBIDENTRY(0,1,DS301_COBID_PDO_TX+0x100*1+CFG_DS301_DEFNODEID), DS301_PDO_TYPE_ACYCLIC         , 2, 0, 0,
                {0x60410010, 0x60610008}},
    {DS301_CREATEPDOCOBIDENTRY(0,1,DS301_COBID_PDO_TX+0x100*2+CFG_DS301_DEFNODEID), DS301_PDO_TYPE_ACYCLIC         , 2, 0, 0,
                {0x60410010, 0x60640020}},
    {DS301_CREATEPDOCOBIDENTRY(0,1,DS301_COBID_PDO_TX+0x100*3+CFG_DS301_DEFNODEID), DS301_PDO_TYPE_ACYCLIC         , 2, 0, 0,
                {0x60410010, 0x606c0020}},
    {DS301_CREATEPDOCOBIDENTRY(1,1,0),                                              DS301_PDO_TYPE_ASYNCHRONOUS_STD, 0, 0, 0,
                {0}},
    {DS301_CREATEPDOCOBIDENTRY(1,1,0),                                              DS301_PDO_TYPE_ASYNCHRONOUS_STD, 0, 0, 0,
                {0}},
    {DS301_CREATEPDOCOBIDENTRY(1,1,0),                                              DS301_PDO_TYPE_ASYNCHRONOUS_STD, 0, 0, 0,
                {0}},
    {DS301_CREATEPDOCOBIDENTRY(1,1,0),                                              DS301_PDO_TYPE_ASYNCHRONOUS_STD, 0, 0, 0,
                {0}},
};

DS301_PDO_PARAM DS301_MEMQ_PARAM tDs301PdoParamRx[CFG_DS301_PDO_RX_TOT];
    
const DS301_PDO_PARAM DS301_MEMQ_PARAM tDs301PdoDefParamRx[CFG_DS301_PDO_RX_TOT]=
{
    {DS301_CREATEPDOCOBIDENTRY(0,1,DS301_COBID_PDO_RX+0x100*0+CFG_DS301_DEFNODEID), DS301_PDO_TYPE_ASYNCHRONOUS_STD, 1, 0, 0,
                {0x60400010}},
    {DS301_CREATEPDOCOBIDENTRY(0,1,DS301_COBID_PDO_RX+0x100*1+CFG_DS301_DEFNODEID), DS301_PDO_TYPE_ASYNCHRONOUS_STD, 2, 0, 0,
                {0x60400010, 0x60600008}},
    {DS301_CREATEPDOCOBIDENTRY(0,1,DS301_COBID_PDO_RX+0x100*2+CFG_DS301_DEFNODEID), DS301_PDO_TYPE_ASYNCHRONOUS_STD, 2, 0, 0,
                {0x60400010, 0x607a0020}},
    {DS301_CREATEPDOCOBIDENTRY(0,1,DS301_COBID_PDO_RX+0x100*3+CFG_DS301_DEFNODEID), DS301_PDO_TYPE_ASYNCHRONOUS_STD, 2, 0, 0,
                {0x60400010, 0x60ff0020}},
    {DS301_CREATEPDOCOBIDENTRY(1,1,0),                                              DS301_PDO_TYPE_ASYNCHRONOUS_STD, 0, 0, 0,
                {0}},
    {DS301_CREATEPDOCOBIDENTRY(1,1,0),                                              DS301_PDO_TYPE_ASYNCHRONOUS_STD, 0, 0, 0,
                {0}},
    {DS301_CREATEPDOCOBIDENTRY(1,1,0),                                              DS301_PDO_TYPE_ASYNCHRONOUS_STD, 0, 0, 0,
                {0}},
    {DS301_CREATEPDOCOBIDENTRY(1,1,0),                                              DS301_PDO_TYPE_ASYNCHRONOUS_STD, 0, 0, 0,
                {0}},
};

//***************************************************************************
// Globals

CANOPENPSM_PDOFASTENTRY_RX **       psCanOpenPSMSyncRxLst;
CANOPENPSM_PDOFASTENTRY_TX **       psCanOpenPSMSyncTxLst;
BOOL                       bCanOpenPSMPdoEnabled;
BOOL                       bCanOpenPSMReSyncEnable;
BOOL                       bCanOpenPSMSyncEvOnPeriodicCob;
BOOL                       bCanOpenPSMFirstSyncRecv=FALSE;
BOOL                       bCanOpenPSMDelayTxPdo=FALSE;
UWORD                               uwCanOpenPSMSyncTiming;
UWORD                               uwCanOpenPSMCanController;
CANOPENPSM_PDOFASTENTRY_TX **       psCanOpenPSMSyncCobTxP=NULL;

UWORD                               uwCanOpenPSMRxPdoCnt[CFG_DS301_PDO_RX_TOT];

//***************************************************************************
// Defines

#define	ODC_INTEGER8										0x0002
#define	ODC_INTEGER16										0x0003
#define	ODC_INTEGER32										0x0004
#define	ODC_UNSIGNED8										0x0005
#define	ODC_UNSIGNED16										0x0006
#define	ODC_UNSIGNED32										0x0007

#define CHKCOMP_PDO_DISABLED                                0
#define CHKCOMP_PDO_ENABLED                                 1

#define PDO_WRKELSIZE                                       ((CFG_DS301_PDO_RX_TOT+CFG_DS301_PDO_TX_TOT)*CFG_DS301_PDO_AVG_ELEM_PER_PDO)
#define PDO_WRKRXLISTSIZE                                   (CFG_DS301_PDO_RX_TOT+2)
#define PDO_WRKTXLISTSIZE                                   (CFG_DS301_PDO_TX_TOT+2)

//***************************************************************************
// Locals

static BOOL bSlowTaskPdoProcessing;
static UWORD uwSyncTimingValidCounter=SYNC_TIMEOUTOFSYNC;

static CANOPENPSM_PDOFASTENTRY_RX sPdoWrkRx[CFG_DS301_PDO_RX_TOT];
static CANDRV_COB sPdoRx[CFG_DS301_PDO_RX_TOT];

static CANOPENPSM_PDOFASTENTRY_TX sPdoWrkTx[CFG_DS301_PDO_TX_TOT];
static CANDRV_COB sPdoTx[CFG_DS301_PDO_TX_TOT];
static CANDRV_COB sPdoRTR[CFG_DS301_PDO_TX_TOT];

static CANOPENPSM_PDOFASTENTRY_ELEMENT sPdoWrkEl[PDO_WRKELSIZE];

static CANOPENPSM_PDOFASTENTRY_RX * psPdoRxList[PDO_WRKRXLISTSIZE];
static CANOPENPSM_PDOFASTENTRY_TX * psPdoTxList[PDO_WRKTXLISTSIZE];

//***************************************************************************
// Local functions prototypes

void cobRTRrequestevent(CANDRV_COB *);

static SWORD checkandcompilepdo(UWORD uwPdoNum, BOOL bSync, BOOL bIsRxPdo, CANOPENPSM_PDOFASTENTRY_ELEMENT * psMapEl, UWORD * puwElArraySize);

//****************************************************************************
// Init handler

BOOL CanOpenPSM_Init(UWORD uwOption, BOOL bReSyncEnable, BOOL bSyncEvOnPeriodicCob, BOOL bDelayTxPdo)
{
    switch(uwOption)
    {
        case CANOPENCM_INIT_CORE:
            bCanOpenPSMPdoEnabled=FALSE;
            bSlowTaskPdoProcessing=FALSE;
            bCanOpenPSMReSyncEnable=bReSyncEnable;
            bSysStatSyncMgrRequired|=bReSyncEnable;
            bCanOpenPSMSyncEvOnPeriodicCob=bSyncEvOnPeriodicCob;
            bCanOpenPSMDelayTxPdo=bDelayTxPdo;
            uwCanOpenPSMCanController=-1;
            break;

        case CANOPENCM_INIT_SYNCEVENT:
                // install callback for sync pdo management when using sync sending
            candrv_instcallbackperiodiccob(&CanOpenPSM_RT_SyncSentEvent);
            break;

        case CANOPENCM_INIT_SYNCPDOEVENT:
                // install realtime task
            if(!TaskSched_AddRTTask(&CanOpenPSM_RT_SyncPdoProcessing, TASKSCHEDULER_FLAG_NONE, \
                    SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_FIELDBUS_SYNCING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_CANOPEN_SYNC), \
                    SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_FIELDBUS_SYNCING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_CANOPEN_SYNC)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_RESETTING), 0))
                return FALSE;

            break;

        default:
            assert(FALSE);
    }

    return TRUE;
}

//***************************************************************************
// Create PDO event

BOOL CanOpenPSM_CreatePdoEvent(UBYTE ubCanCtrl)
{
    UWORD uwPdoNum;
    UWORD uwListPos,uwDupCobIdPos;
    UWORD uwLeftSize=PDO_WRKELSIZE;
    SWORD swRetVal;
    UWORD uwCobId[PDO_WRKRXLISTSIZE+PDO_WRKTXLISTSIZE];

        // save selected can controller
    uwCanOpenPSMCanController=ubCanCtrl;

        // lock PDO configuration change
    bSysStatPdoMgrLockConfig=TRUE;

        // erase cobid list for late checking
    for(uwPdoNum=0;uwPdoNum<PDO_WRKRXLISTSIZE+PDO_WRKTXLISTSIZE;uwPdoNum++)
        uwCobId[uwPdoNum]=0x000;

        // erase rx counter
    for(uwPdoNum=0;uwPdoNum<CFG_DS301_PDO_RX_TOT;uwPdoNum++)
        uwCanOpenPSMRxPdoCnt[uwPdoNum]=0;

        // create RX list for async pdo
    uwListPos=uwDupCobIdPos=0;
    for(uwPdoNum=0;uwPdoNum<CFG_DS301_PDO_RX_TOT;uwPdoNum++)
    {
        swRetVal=checkandcompilepdo(uwPdoNum, FALSE, TRUE, &sPdoWrkEl[PDO_WRKELSIZE-uwLeftSize], &uwLeftSize);

            // if enabled add to the list
        if(swRetVal==CHKCOMP_PDO_ENABLED)
        {
            uwCobId[uwDupCobIdPos++]=DS301_GETCANIDS(sPdoWrkRx[uwPdoNum].psCob->id);
            psPdoRxList[uwListPos++]=&sPdoWrkRx[uwPdoNum];
        }

        if(swRetVal<0)
            goto processingerror;
    }

        // write terminator
    psPdoRxList[uwListPos++]=NULL;
    
        // then this element is the begin for the RX list for sync pdo
    psCanOpenPSMSyncRxLst=&psPdoRxList[uwListPos];

        // create RX list for sync pdo
    for(uwPdoNum=0;uwPdoNum<CFG_DS301_PDO_RX_TOT;uwPdoNum++)
    {
        swRetVal=checkandcompilepdo(uwPdoNum, TRUE, TRUE, &sPdoWrkEl[PDO_WRKELSIZE-uwLeftSize], &uwLeftSize);

            // if enabled add to the list
        if(swRetVal==CHKCOMP_PDO_ENABLED)
        {
            uwCobId[uwDupCobIdPos++]=DS301_GETCANIDS(sPdoWrkRx[uwPdoNum].psCob->id);
            psPdoRxList[uwListPos++]=&sPdoWrkRx[uwPdoNum];
        }

        if(swRetVal<0)
            goto processingerror;
    }

        // write terminator
    psPdoRxList[uwListPos++]=NULL;

        // create TX list for async pdo
    uwListPos=0;
    for(uwPdoNum=0;uwPdoNum<CFG_DS301_PDO_TX_TOT;uwPdoNum++)
    {
        swRetVal=checkandcompilepdo(uwPdoNum, FALSE, FALSE, &sPdoWrkEl[PDO_WRKELSIZE-uwLeftSize], &uwLeftSize);

            // if enabled add to the list
        if(swRetVal==CHKCOMP_PDO_ENABLED)
        {
            uwCobId[uwDupCobIdPos++]=DS301_GETCANIDS(sPdoWrkTx[uwPdoNum].psCob->id);
            psPdoTxList[uwListPos++]=&sPdoWrkTx[uwPdoNum];
        }

        if(swRetVal<0)
            goto processingerror;
    }

        // write terminator
    psPdoTxList[uwListPos++]=NULL;
    
        // then this element is the begin for the TX list for sync pdo
    psCanOpenPSMSyncTxLst=&psPdoTxList[uwListPos];

        // create TX list for sync pdo
    for(uwPdoNum=0;uwPdoNum<CFG_DS301_PDO_TX_TOT;uwPdoNum++)
    {
        swRetVal=checkandcompilepdo(uwPdoNum, TRUE, FALSE, &sPdoWrkEl[PDO_WRKELSIZE-uwLeftSize], &uwLeftSize);

            // if enabled add to the list
        if(swRetVal==CHKCOMP_PDO_ENABLED)
        {
            uwCobId[uwDupCobIdPos++]=DS301_GETCANIDS(sPdoWrkTx[uwPdoNum].psCob->id);
            psPdoTxList[uwListPos++]=&sPdoWrkTx[uwPdoNum];
        }

        if(swRetVal<0)
            goto processingerror;
    }

        // write terminator
    psPdoTxList[uwListPos]=NULL;

        // scan all RX/TX lists to check for duplicated cob-id
    for(uwPdoNum=0;uwPdoNum<PDO_WRKRXLISTSIZE+PDO_WRKTXLISTSIZE;uwPdoNum++)
        if(uwCobId[uwPdoNum]>0)
            for(uwLeftSize=0;uwLeftSize<PDO_WRKRXLISTSIZE+PDO_WRKTXLISTSIZE;uwLeftSize++)
                if(uwPdoNum!=uwLeftSize && uwCobId[uwPdoNum]==uwCobId[uwLeftSize])
                {
                    swRetVal=CANOPENPSM_PDOERR_DUPLICATEDCOBID;
                    goto processingerror;
                }

        // now scan all RX/RTR lists and add all enabled PDO to can controller rx list

        // sync RX
    for(uwPdoNum=0;psCanOpenPSMSyncRxLst[uwPdoNum];uwPdoNum++)
        candrv_addrxcob(uwCanOpenPSMCanController, psCanOpenPSMSyncRxLst[uwPdoNum]->psCob, NULL);
    
        // async RX
    for(uwPdoNum=0;psPdoRxList[uwPdoNum];uwPdoNum++)
        candrv_addrxcob(uwCanOpenPSMCanController, psPdoRxList[uwPdoNum]->psCob, NULL);
    
        // sync RTR TX
    for(uwPdoNum=0;psCanOpenPSMSyncTxLst[uwPdoNum];uwPdoNum++)
        if(psCanOpenPSMSyncTxLst[uwPdoNum]->psRTRCob!=NULL)
            candrv_addrxcob(uwCanOpenPSMCanController, psCanOpenPSMSyncTxLst[uwPdoNum]->psRTRCob, &cobRTRrequestevent);

        // async RTR TX
    for(uwPdoNum=0;psPdoTxList[uwPdoNum];uwPdoNum++)
        if(psPdoTxList[uwPdoNum]->psRTRCob!=NULL)
            candrv_addrxcob(uwCanOpenPSMCanController, psPdoTxList[uwPdoNum]->psRTRCob, &cobRTRrequestevent);

        // unlock PDO configuration change
    bSysStatPdoMgrLockConfig=FALSE;

        // and enable PDO processing
    bCanOpenPSMPdoEnabled=TRUE;

    return TRUE;

processingerror:
    bSysStatPdoMgrLockConfig=FALSE;
    CanOpenCM_CanFaultSignal(CANOPENPSM_PE_MASK(swRetVal));
    return FALSE;
}

//***************************************************************************
// Destroy PDO event

void CanOpenPSM_DestroyPdoEvent(UBYTE by)
{
    UWORD uwPdoNum;

        // if enabled
    if(bCanOpenPSMPdoEnabled)
    {
            // stop processing
        bCanOpenPSMPdoEnabled=FALSE;
        psCanOpenPSMSyncCobTxP=NULL;

            // and wait if slow task is still processing pdo
        while(bSlowTaskPdoProcessing)
            Os_Sleep(0);

            // now remove all rx/rtr cob that was subscribed
            // sync RX
        for(uwPdoNum=0;psCanOpenPSMSyncRxLst[uwPdoNum];uwPdoNum++)
            candrv_deleterxcob(uwCanOpenPSMCanController, psCanOpenPSMSyncRxLst[uwPdoNum]->psCob);
        
            // async RX
        for(uwPdoNum=0;psPdoRxList[uwPdoNum];uwPdoNum++)
            candrv_deleterxcob(uwCanOpenPSMCanController, psPdoRxList[uwPdoNum]->psCob);
        
            // sync RTR TX
        for(uwPdoNum=0;psCanOpenPSMSyncTxLst[uwPdoNum];uwPdoNum++)
            if(psCanOpenPSMSyncTxLst[uwPdoNum]->psRTRCob!=NULL)
                candrv_deleterxcob(uwCanOpenPSMCanController, psCanOpenPSMSyncTxLst[uwPdoNum]->psRTRCob);
    
            // async RTR TX
        for(uwPdoNum=0;psPdoTxList[uwPdoNum];uwPdoNum++)
            if(psPdoTxList[uwPdoNum]->psRTRCob!=NULL)
                candrv_deleterxcob(uwCanOpenPSMCanController, psPdoTxList[uwPdoNum]->psRTRCob);
    }
}

//***************************************************************************
// Set default cobid for PDO's when nodeid changed

void CanOpenPSM_SetDefaultCobID(UBYTE ubPrev, UBYTE ubNew)
{
    UWORD ct;

	if(ubPrev==DS301_LSS_UNCONFIGUREDNODEID)
		for(ct=0;ct<DS301_PDO_TX_TOT;ct++)
			if(ct<4)
				tDs301PdoParamTx[ct].tCobId=DS301_CREATEPDOCOBIDENTRY(1,1,DS301_COBID_PDO_TX+0x100*ct+ubNew);
			else
				tDs301PdoParamTx[ct].tCobId=DS301_CREATEPDOCOBIDENTRY(1,1,0);
				
	else
		for(ct=0;ct<(4<DS301_PDO_TX_TOT?DS301_PDO_TX_TOT:4);ct++)
			if(DS301_GETCANID(tDs301PdoParamTx[ct].tCobId)==DS301_COBID_PDO_TX+0x100*ct+ubPrev)
				tDs301PdoParamTx[ct].tCobId=(tDs301PdoParamTx[ct].tCobId&~DS301_COBIDMASK)+DS301_COBID_PDO_TX+0x100*ct+ubNew;
	
	if(ubPrev==DS301_LSS_UNCONFIGUREDNODEID)
		for(ct=0;ct<DS301_PDO_RX_TOT;ct++)
			if(ct<4)
				tDs301PdoParamRx[ct].tCobId=DS301_CREATEPDOCOBIDENTRY(1,1,DS301_COBID_PDO_RX+0x100*ct+ubNew);
			else
				tDs301PdoParamRx[ct].tCobId=DS301_CREATEPDOCOBIDENTRY(1,1,0);
				
	else
		for(ct=0;ct<(4<DS301_PDO_RX_TOT?DS301_PDO_RX_TOT:4);ct++)
			if(DS301_GETCANID(tDs301PdoParamRx[ct].tCobId)==DS301_COBID_PDO_RX+0x100*ct+ubPrev)
				tDs301PdoParamRx[ct].tCobId=(tDs301PdoParamRx[ct].tCobId&~DS301_COBIDMASK)+DS301_COBID_PDO_RX+0x100*ct+ubNew;
}

//***************************************************************************
// check configuration and compile rx PDO for fast processing

static SWORD checkandcompilepdo(UWORD uwPdoNum, BOOL bSync, BOOL bIsRxPdo, CANOPENPSM_PDOFASTENTRY_ELEMENT * psMapEl, UWORD * puwElArraySize)
{
    DS301_PDO_PARAM DS301_MEMQ_PARAM * psParam;
    UWORD uwCt;
    UWORD uwSize;
    BOOL bIsSync;

        // select parameters
    if(bIsRxPdo)
        psParam=&tDs301PdoParamRx[uwPdoNum];
    else
        psParam=&tDs301PdoParamTx[uwPdoNum];

        // if not enabled
	if(!DS301_COBIDVALID(psParam->tCobId))
        return CHKCOMP_PDO_DISABLED;

        // check for valid cobid
	if(DS301_COBID29BITID(psParam->tCobId))
		return CANOPENPSM_PDOERR_INVALIDCOBID;;

//$TEST$ - tappullo x ZF, rimettere controllo validita' COBID
//    if(DS301_GETCANID(psParam->tCobId)<DS301_COBID_PDO_ID_LOW || DS301_GETCANID(psParam->tCobId)>DS301_COBID_PDO_ID_HIGH)
//		return CANOPENPSM_PDOERR_INVALIDCOBID;

        // check for RTR appliance, only for TXPDO, as from DS301V4.2 such bit for RXPDO is marked as 'do not care'
    if(!bIsRxPdo)
    {
            // if RTR only tx type RTR flag have to be enabled
        if((psParam->bType==DS301_PDO_TYPE_SYNCHRONOUS_RTR || psParam->bType==DS301_PDO_TYPE_ASYNCHRONOUS_RTR) && \
            !DS301_COBIDRTR(psParam->tCobId) )
            return CANOPENPSM_PDOERR_RTRNOTVALID;
    }

        // check for valid rx type
	if(bIsRxPdo)
	{
		if(!(psParam->bType<=DS301_PDO_TYPE_CYCLIC_UBOUND || psParam->bType==DS301_PDO_TYPE_ASYNCHRONOUS_STD))
			return CANOPENPSM_PDOERR_INVALIDTXTYPE;

        bIsSync=(psParam->bType<=DS301_PDO_TYPE_CYCLIC_UBOUND);
	}
	else
	{
		if(!(psParam->bType<=DS301_PDO_TYPE_CYCLIC_UBOUND || psParam->bType>=DS301_PDO_TYPE_SYNCHRONOUS_RTR))
			return CANOPENPSM_PDOERR_INVALIDTXTYPE;

        bIsSync=(psParam->bType<=DS301_PDO_TYPE_CYCLIC_UBOUND) || (psParam->bType==DS301_PDO_TYPE_SYNCHRONOUS_RTR);
	}

        // check if mapping only sync objects or only async objects
    if(bSync!=bIsSync)
        return CHKCOMP_PDO_DISABLED;

        // check for valid SYNC start value
    if(!bIsRxPdo && (psParam->bType>=DS301_PDO_TYPE_CYCLIC_LBOUND || psParam->bType<=DS301_PDO_TYPE_CYCLIC_UBOUND) && \
        psParam->bSyncStartValue>psParam->bType)
        return CANOPENPSM_PDOERR_INVALIDSYNCSTARTVALUE;

        // check for valid mapping
    if(psParam->bNoMapped<1 || psParam->bNoMapped>DS301_PDO_MAXDATAOBJECT)
        return CANOPENPSM_PDOERR_INVALIDMAPCOUNT;

        // check for element map availability
    if(psParam->bNoMapped>*puwElArraySize)
        return CANOPENPSM_PDOERR_OUTOFMEMORY;

        // begin mapping elements
    uwSize=0;
    for(uwCt=0;uwCt<psParam->bNoMapped;uwCt++)
    {
        DS301_PDO_MAP sMapElement=psParam->tMap[uwCt];
        const CANOPENCOMDB_ENTRY * ptEntry;
        UWORD uwDummy;

            // check for valid object (must be not zero)
        if(sMapElement.dMap==0l)
            return CANOPENPSM_PDOERR_INVALIDMAPCOUNT;
        
            // index mapped in object dictionary
        if(sMapElement.f.wIndex>=DS301_PDO_VALIDSDOINDEX)
        {
                // find element
            if(CanOpenComDBEntrySearch(sMapElement.f.wIndex,sMapElement.f.bSubIndex,CANOPENCOMDB_F_DS301_VALID,&ptEntry)!=DS301_SDOOK)
                return CANOPENPSM_PDOERR_INTERNALERROR;
        
                // check if mappable in PDO
            if(!(ptEntry->uwFlags&CANOPENCOMDB_F_PDOMAPPABLE))
                return CANOPENPSM_PDOERR_INTERNALERROR;
        
                // if array and the selected element is zero then error
            if(ptEntry->hpsComDBEntry->uwNElements>1 && sMapElement.f.bSubIndex==0)
                return CANOPENPSM_PDOERR_INTERNALERROR;
        
                // if RX check for object writable
            if(bIsRxPdo)
            {
                if(ptEntry->uwFlags&CANOPENCOMDB_F_WRDENYWHENOPER)
                    return CANOPENPSM_PDOERR_INTERNALERROR;
        
                if(!(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_WR))
                    return CANOPENPSM_PDOERR_INTERNALERROR;
            }
                // otherwise if TX check for object readable
            else
                if(!(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_RD))
                    return CANOPENPSM_PDOERR_INTERNALERROR;

                // map element, if subindex not zero map array element, but if found element is not
                // an array then all subindex are mapped with multiple entry, then address of base
                // element should be choose
            if(sMapElement.f.bSubIndex>0 && ptEntry->hpsComDBEntry->uwNElements>1)
                psMapEl[uwCt].pvDataAddress=(void *)ComParDBEntryGetPointer(ptEntry->hpsComDBEntry, sMapElement.f.bSubIndex-1, &uwDummy);
            else
                psMapEl[uwCt].pvDataAddress=(void *)ComParDBEntryGetPointer(ptEntry->hpsComDBEntry, 0, &uwDummy);

            switch(ptEntry->hpsComDBEntry->ubType)
            {
                case COMMONPARAMDB_TYPE_UBYTE:
                case COMMONPARAMDB_TYPE_SBYTE:
                    if(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_UMCONV)
                    {
                        psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_8BIT_UM;
                        psMapEl[uwCt].fpfUmConv=ptEntry->hpsComDBEntry->uf.fpfUmConv;
                    }
                    else
                    {
                        psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_8BIT;
                        psMapEl[uwCt].fpfUmConv=NULL;
                    }
                    uwSize+=sizeof(UBYTE);
                    break;

                case COMMONPARAMDB_TYPE_UWORD:
                case COMMONPARAMDB_TYPE_SWORD:
                    if(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_UMCONV)
                    {
                        if(uwSize&1)
                            psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_ODD16_UM;
                        else
                            psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_EVEN16_UM;
                        psMapEl[uwCt].fpfUmConv=ptEntry->hpsComDBEntry->uf.fpfUmConv;
                    }
                    else
                    {
                        if(uwSize&1)
                            psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_ODD16;
                        else
                            psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_EVEN16;
                        psMapEl[uwCt].fpfUmConv=NULL;
                    }
                    uwSize+=sizeof(UWORD);
                    break;

                case COMMONPARAMDB_TYPE_ULONG:
                case COMMONPARAMDB_TYPE_SLONG:
                case COMMONPARAMDB_TYPE_FLOAT:
                    if(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_UMCONV)
                    {
                        if(uwSize&1)
                            psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_ODD32_UM;
                        else
                            psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_EVEN32_UM;
                        psMapEl[uwCt].fpfUmConv=ptEntry->hpsComDBEntry->uf.fpfUmConv;
                    }
                    else
                    {
                        if(uwSize&1)
                            psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_ODD32;
                        else
                            psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_EVEN32;
                        psMapEl[uwCt].fpfUmConv=NULL;
                    }
                    uwSize+=sizeof(ULONG);
                    break;

                case COMMONPARAMDB_TYPE_UQWRD:
                case COMMONPARAMDB_TYPE_SQWRD:
                case COMMONPARAMDB_TYPE_DOUBL:
                    if(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_UMCONV)
                    {
                        psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_64BIT_UM;
                        psMapEl[uwCt].fpfUmConv=ptEntry->hpsComDBEntry->uf.fpfUmConv;
                    }
                    else
                    {
                        psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_64BIT;
                        psMapEl[uwCt].fpfUmConv=NULL;
                    }
                    uwSize+=sizeof(UQWRD);
                    break;

                default:
                    return CANOPENPSM_PDOERR_INTERNALERROR;
            }
        }
            // index mapped for system objects
        else
        {
            UWORD uwObjLen=0;

        	if(!bIsRxPdo)
        		return CANOPENPSM_PDOERR_INTERNALERROR;
        
                // dummy objects, just for RX PDO
        	if(sMapElement.f.wIndex<DS301_PDO_DUMMY_LBOUND || sMapElement.f.wIndex>DS301_PDO_DUMMY_HBOUND)
        		return CANOPENPSM_PDOERR_INTERNALERROR;
        
        	if(sMapElement.f.bSubIndex>0)
        		return CANOPENPSM_PDOERR_INTERNALERROR;
        
        	if(sMapElement.f.wIndex==ODC_INTEGER8  || sMapElement.f.wIndex==ODC_UNSIGNED8)
        		uwObjLen=sizeof(UBYTE);
        
        	if(sMapElement.f.wIndex==ODC_INTEGER16 || sMapElement.f.wIndex==ODC_UNSIGNED16)
        		uwObjLen=sizeof(UWORD);
        
        	if(sMapElement.f.wIndex==ODC_INTEGER32 || sMapElement.f.wIndex==ODC_UNSIGNED32)
        		uwObjLen=sizeof(ULONG);

            psMapEl[uwCt].pvDataAddress=(void *)uwObjLen;
            psMapEl[uwCt].uwSelector=CANOPENPSM_SELECTOR_DISPLACEMENT;
            psMapEl[uwCt].fpfUmConv=NULL;

            uwSize+=uwObjLen;
        }
    }

        // check overall mapping size
    if(uwSize>sizeof(sPdoWrkRx[uwPdoNum].psCob->d))
        return CANOPENPSM_PDOERR_PDOLENGTHEXCEED;

        // finalize mapping for RX PDO
    if(bIsRxPdo)
    {
            // prepare mapping
        sPdoWrkRx[uwPdoNum].ubCobLen=(UBYTE)uwSize;
        sPdoWrkRx[uwPdoNum].uwNElements=psParam->bNoMapped;
        sPdoWrkRx[uwPdoNum].psElements=psMapEl;
        sPdoWrkRx[uwPdoNum].psCob=&sPdoRx[uwPdoNum];
        sPdoWrkRx[uwPdoNum].puwCounter=&uwCanOpenPSMRxPdoCnt[uwPdoNum];

            // and rx cob
        sPdoRx[uwPdoNum].id=DS301_GETCANIDS(psParam->tCobId);
        memset(&(sPdoRx[uwPdoNum].flags),0,sizeof(sPdoRx[uwPdoNum].flags));//sPdoRx[uwPdoNum].flags=CANDRV_F_DEFAULT;
        if(bIsSync)
            sPdoRx[uwPdoNum].flags.bRxHiPriority;//|=CANDRV_F_RXHIPRIOR;
        sPdoRx[uwPdoNum].len=uwSize;
    }
        // finalize mapping for TX PDO
    else
    {
            // prepare mapping
        sPdoWrkTx[uwPdoNum].uwNElements=psParam->bNoMapped;
        sPdoWrkTx[uwPdoNum].psElements=psMapEl;
        sPdoWrkTx[uwPdoNum].psCob=&sPdoTx[uwPdoNum];

        if( psParam->bType==DS301_PDO_TYPE_ACYCLIC || \
            psParam->bType==DS301_PDO_TYPE_ASYNCHRONOUS_MANUF || \
            psParam->bType==DS301_PDO_TYPE_ASYNCHRONOUS_STD )
            sPdoWrkTx[uwPdoNum].f.bCfgCompare=TRUE;
        else
            sPdoWrkTx[uwPdoNum].f.bCfgCompare=FALSE;

            // if sync cyclic
        if( psParam->bType>=DS301_PDO_TYPE_CYCLIC_LBOUND && \
            psParam->bType<=DS301_PDO_TYPE_CYCLIC_UBOUND )
        {
            sPdoWrkTx[uwPdoNum].f.bCfgSyncCnt=TRUE;
            sPdoWrkTx[uwPdoNum].ubNSync=psParam->bType;
            if(psParam->bSyncStartValue==0)
                sPdoWrkTx[uwPdoNum].ubSyncCnt=1;
            else
                sPdoWrkTx[uwPdoNum].ubSyncCnt=psParam->bSyncStartValue;
        }
        else
            sPdoWrkTx[uwPdoNum].f.bCfgSyncCnt=FALSE;

            // if acyclic or async trigger immediate tx
        if( psParam->bType==DS301_PDO_TYPE_ACYCLIC || \
            psParam->bType==DS301_PDO_TYPE_ASYNCHRONOUS_MANUF || \
            psParam->bType==DS301_PDO_TYPE_ASYNCHRONOUS_STD )
            sPdoWrkTx[uwPdoNum].f.bStRTRTriggered=TRUE;
        else
            sPdoWrkTx[uwPdoNum].f.bStRTRTriggered=FALSE;

        sPdoWrkTx[uwPdoNum].f.bStTxTrigger=FALSE;

            // and tx cob
        sPdoTx[uwPdoNum].id=DS301_GETCANIDS(psParam->tCobId);
        memset(&(sPdoTx[uwPdoNum].flags),0,sizeof(sPdoTx[uwPdoNum].flags));
        sPdoTx[uwPdoNum].flags.bTxOK=TRUE;//sPdoTx[uwPdoNum].flags=CANDRV_F_DEFAULT|CANDRV_F_TXOK;
        sPdoTx[uwPdoNum].len=uwSize;

            // if sync ignore inhibit time
        if(bIsSync)
            sPdoTx[uwPdoNum].inhibit=0;
        else
            sPdoTx[uwPdoNum].inhibit=psParam->wInhibitTime;

            // check if RTR apply
        if(DS301_COBIDRTR(psParam->tCobId))
        {
            sPdoRTR[uwPdoNum].id=DS301_GETCANIDS(psParam->tCobId);
            memset(&sPdoRTR[uwPdoNum].flags,0,sizeof(sPdoRTR[uwPdoNum].flags));
            sPdoRTR[uwPdoNum].flags.bRTR = TRUE;//sPdoRTR[uwPdoNum].flags=CANDRV_F_DEFAULT|CANDRV_F_RTR;
            if(bIsSync)
            	sPdoRTR[uwPdoNum].flags.bRxHiPriority = TRUE;//sPdoRTR[uwPdoNum].flags|=CANDRV_F_RXHIPRIOR;
            sPdoRTR[uwPdoNum].len=0;
            sPdoRTR[uwPdoNum].usrdata=(ULONG)&sPdoWrkTx[uwPdoNum];

            sPdoWrkTx[uwPdoNum].psRTRCob=&sPdoRTR[uwPdoNum];
        }
        else
        {
        	memset(&sPdoRTR[uwPdoNum].flags,0,sizeof(sPdoRTR[uwPdoNum].flags));//sPdoRTR[uwPdoNum].flags=CANDRV_F_DEFAULT;
            sPdoWrkTx[uwPdoNum].psRTRCob=NULL;
        }
    }

        // update no. of elements left
    *puwElArraySize-=psParam->bNoMapped;

    return CHKCOMP_PDO_ENABLED;
}

//***************************************************************************
// Hook for param management

UWORD CanOpenPSM_PdoParam(COMMONPARAMDB_ENTRY * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext)
{
    DS301_PDO_PARAM * sPar;
    BOOL bIsRxPdo;
    const uwElSize[]=
    {
        sizeof(UBYTE),
        sizeof(sPar->tCobId),
        sizeof(sPar->bType),
        sizeof(sPar->wInhibitTime),
        sizeof(UBYTE),
        sizeof(UWORD),
        sizeof(sPar->bSyncStartValue),
    };

        // get pointer to PDO param structure
    sPar=(DS301_PDO_PARAM *)hpsEntry->hpvData;

        // if abort do nothing
    if(uwFlags&COMMONPARAMDB_CBFLAG_ABORT)
        return COMMONPARAMDB_CH_OK;

        // get PDO type
    bIsRxPdo=(hpsEntry->swMoreType==CANOPENPSM_PAR_RX);

        // check element access
    if(bIsRxPdo)
    {
        if(uwElement>CANOPENPSM_NSUBINDX_RX)
            return COMMONPARAMDB_CH_INVALID_ELEMENT;
    }
    else
        if(uwElement>CANOPENPSM_NSUBINDX_TX)
            return COMMONPARAMDB_CH_INVALID_ELEMENT;

        // if data read
    if(uwFlags&COMMONPARAMDB_CBFLAG_RD)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
                // data size if requested
            if(uwFlags&COMMONPARAMDB_CBFLAG_SIZEINQUIRY)
            {
				*((HPULONG)hpvBuffer)=uwElSize[uwElement];
                *puwBufSize=sizeof(ULONG);
            }
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            UWORD uwCt;

            if(uwFlags&COMMONPARAMDB_CBFLAG_NODSCHECK)
            {
                if(*puwBufSize<uwElSize[uwElement])
                    return COMMONPARAMDB_CH_WRONGLENGTH;
            }
            else
                if(*puwBufSize!=uwElSize[uwElement])
                    return COMMONPARAMDB_CH_WRONGLENGTH;

            switch(uwElement)
            {
				case 0:     // no. of subindex
                    if(bIsRxPdo)
                        *((HPUBYTE)hpvBuffer)=CANOPENPSM_NSUBINDX_RX;
                    else
                        *((HPUBYTE)hpvBuffer)=CANOPENPSM_NSUBINDX_TX;
                    break;

				case 1:     // cob-id
                    atomic_read(hpvBuffer, &sPar->tCobId, sizeof(sPar->tCobId));
					break;
			
				case 2:     // tx type
					*((HPUBYTE)hpvBuffer)=sPar->bType;
					break;
			
				case 3:     // inhibit time
                    memcpy(hpvBuffer, &(sPar->wInhibitTime), sizeof(UWORD));
					break;

				case 4:     // reserved
					*((HPUBYTE)hpvBuffer)=0;
					break;

				case 5:     // reserved
					((HPUBYTE)hpvBuffer)[0]=0;
					((HPUBYTE)hpvBuffer)[1]=0;
					break;

				case 6:     // sync start value
					*((HPUBYTE)hpvBuffer)=sPar->bSyncStartValue;
					break;
            }

            for(uwCt=uwElSize[uwElement];uwCt<*puwBufSize;uwCt++)
                ((HPUBYTE)hpvBuffer)[uwCt]=0;

            *puwBufSize=uwElSize[uwElement];
        }
    }

        // if data write
    if(uwFlags&COMMONPARAMDB_CBFLAG_WR)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
                // check for write access
            switch(uwElement)
            {
				case 0:     // no. of subindex
				case 4:     // reserved
				case 5:     // reserved
                    return COMMONPARAMDB_CH_NO_WRITE_ACCESS;
            }

                // also accept zero length for clients that
                // do not send data length
            if(*((HPULONG)hpvBuffer)!=0l)
                if(uwFlags&COMMONPARAMDB_CBFLAG_NODSCHECK)
                {
                    if(*((HPULONG)hpvBuffer)<(ULONG)uwElSize[uwElement])
                        return COMMONPARAMDB_CH_WRONGLENGTH;
                }
                else
                    if(*((HPULONG)hpvBuffer)!=(ULONG)uwElSize[uwElement])
                        return COMMONPARAMDB_CH_WRONGLENGTH;
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            DS301_COBIDENTRY cobid;

            switch(uwElement)
            {
				case 1:     // cob-id
                    if(*puwBufSize<sizeof(sPar->tCobId))
                        return COMMONPARAMDB_CH_WRONGLENGTH;

                    memcpy(&cobid, hpvBuffer, sizeof(ULONG));

    				if(DS301_COBIDVALID(cobid))
    				{
    					if(DS301_COBID29BITID(cobid))
    						return COMMONPARAMDB_CH_INVALID_DATA;
    
    					if(!ds301_check_cob_id(cobid))
    						return COMMONPARAMDB_CH_INVALID_DATA;
    				}
                    
                    atomic_write(&sPar->tCobId, hpvBuffer, sizeof(sPar->tCobId));
					break;
			
				case 2:     // tx type
                    if(*puwBufSize<sizeof(sPar->bType))
                        return COMMONPARAMDB_CH_WRONGLENGTH;

    				if(bIsRxPdo)
    				{
    					if(!(*((HPUBYTE)hpvBuffer)<=DS301_PDO_TYPE_CYCLIC_UBOUND || *((HPUBYTE)hpvBuffer)==DS301_PDO_TYPE_ASYNCHRONOUS_STD))
    						return COMMONPARAMDB_CH_INVALID_DATA;
    				}
    				else
    				{
    					if(!(*((HPUBYTE)hpvBuffer)<=DS301_PDO_TYPE_CYCLIC_UBOUND || *((HPUBYTE)hpvBuffer)>=DS301_PDO_TYPE_SYNCHRONOUS_RTR))
    						return COMMONPARAMDB_CH_INVALID_DATA;
    				}

					sPar->bType=*((HPUBYTE)hpvBuffer);
					break;
			
				case 3:     // inhibit time
                    if(*puwBufSize<sizeof(sPar->wInhibitTime))
                        return COMMONPARAMDB_CH_WRONGLENGTH;

                    memcpy(&(sPar->wInhibitTime), hpvBuffer, sizeof(UWORD));
					break;

				case 6:     // sync start value
                    if(*puwBufSize<sizeof(sPar->bSyncStartValue))
                        return COMMONPARAMDB_CH_WRONGLENGTH;

					sPar->bSyncStartValue=*((HPUBYTE)hpvBuffer);
					break;
            }
        }
    }

    return COMMONPARAMDB_CH_OK;
}


//***************************************************************************
// Hook for object mapping management

UWORD CanOpenPSM_PdoMap(COMMONPARAMDB_ENTRY * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext)
{
    DS301_PDO_PARAM * sPar;
    BOOL bIsRxPdo;
    UWORD uwSize;

        // get pointer to PDO param structure
    sPar=(DS301_PDO_PARAM *)hpsEntry->hpvData;

        // if abort do nothing
    if(uwFlags&COMMONPARAMDB_CBFLAG_ABORT)
        return COMMONPARAMDB_CH_OK;

        // get PDO type
    bIsRxPdo=(hpsEntry->swMoreType==CANOPENPSM_PAR_RX);

        // check element access
    if(uwElement>DS301_PDO_MAXDATAOBJECT)
        return COMMONPARAMDB_CH_INVALID_ELEMENT;

        // calculate element size
    uwSize=(uwElement?sizeof(DS301_PDO_MAP):sizeof(UBYTE));

        // if data read
    if(uwFlags&COMMONPARAMDB_CBFLAG_RD)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
                // data size if requested
            if(uwFlags&COMMONPARAMDB_CBFLAG_SIZEINQUIRY)
            {
                *((HPULONG)hpvBuffer)=uwSize;
                *puwBufSize=sizeof(ULONG);
            }
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            UWORD uwCt;

            if(uwFlags&COMMONPARAMDB_CBFLAG_NODSCHECK)
            {
                if(*puwBufSize<uwSize)
                    return COMMONPARAMDB_CH_WRONGLENGTH;
            }
            else
                if(*puwBufSize!=uwSize)
                    return COMMONPARAMDB_CH_WRONGLENGTH;

                // element
            if(uwElement)
                memcpy(hpvBuffer, &(sPar->tMap[uwElement-1]), sizeof(DS301_PDO_MAP));

                // no. of elements
            else
                *((HPUBYTE)hpvBuffer)=sPar->bNoMapped;

            for(uwCt=uwSize;uwCt<*puwBufSize;uwCt++)
                ((HPUBYTE)hpvBuffer)[uwCt]=0;
            *puwBufSize=uwSize;

        }
    }

        // if data write
    if(uwFlags&COMMONPARAMDB_CBFLAG_WR)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
                // accept zero length for clients that
                // do not send data length
            if(*((HPULONG)hpvBuffer)!=0l)
                if(uwFlags&COMMONPARAMDB_CBFLAG_NODSCHECK)
                {
                    if(*((HPULONG)hpvBuffer)<uwSize)
                        return COMMONPARAMDB_CH_WRONGLENGTH;
                }
                else
                    if(*((HPULONG)hpvBuffer)!=uwSize)
                        return COMMONPARAMDB_CH_WRONGLENGTH;
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            if(*puwBufSize<uwSize)
                return COMMONPARAMDB_CH_WRONGLENGTH;

                // map write
            if(uwElement)
            {
                DS301_PDO_MAP sMapElement;
                const CANOPENCOMDB_ENTRY * ptEntry;
                UBYTE ubObjLen=0;

                    // get map element
                memcpy(&sMapElement, hpvBuffer, sizeof(DS301_PDO_MAP));

                    // accept zero as null object
                if(sMapElement.dMap!=0l)
                {
                        // index mapped in object dictionary
                    if(sMapElement.f.wIndex>=DS301_PDO_VALIDSDOINDEX)
                    {
                        ULONG ulRetVal;
    
                            // find element
                        ulRetVal=CanOpenComDBEntrySearch(sMapElement.f.wIndex,sMapElement.f.bSubIndex,CANOPENCOMDB_F_DS301_VALID,&ptEntry);
                        if(ulRetVal!=DS301_SDOOK)
                            if(ulRetVal==DS301_SDOABORT_INVALIDSUBINDEX)
                                return COMMONPARAMDB_CH_INVALID_ELEMENT;
                            else
                                return COMMONPARAMDB_CH_ELEMENT_NOTFOUND;
    
                            // check if mappable in PDO
                        if(!(ptEntry->uwFlags&CANOPENCOMDB_F_PDOMAPPABLE))
                            return COMMONPARAMDB_CH_NOTMAPPABLE;
    
                            // if array and the selected element is zero then error
                        if(ptEntry->hpsComDBEntry->uwNElements>1 && sMapElement.f.bSubIndex==0)
                            return COMMONPARAMDB_CH_NOTMAPPABLE;
    
                            // if RX check for object writable
                        if(bIsRxPdo)
                        {
                            if(ptEntry->uwFlags&CANOPENCOMDB_F_WRDENYWHENOPER)
                                return COMMONPARAMDB_CH_NO_WRITE_ACCESS;
    
                            if(!(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_WR))
                                return COMMONPARAMDB_CH_NO_WRITE_ACCESS;
                        }
                            // otherwise if TX check for object readable
                        else
                            if(!(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_RD))
                                return COMMONPARAMDB_CH_NO_READ_ACCESS;
    
                        ubObjLen=(UBYTE)ComParDBEntryGetEntrySize(ptEntry->hpsComDBEntry);
                    }
                        // index mapped for system objects
                    else
                    {
        				if(!bIsRxPdo)
        					return COMMONPARAMDB_CH_ELEMENT_NOTFOUND;
        
                            // dummy objects, just for RX PDO
        				if(sMapElement.f.wIndex<DS301_PDO_DUMMY_LBOUND || sMapElement.f.wIndex>DS301_PDO_DUMMY_HBOUND)
        					return COMMONPARAMDB_CH_ELEMENT_NOTFOUND;
        
        				if(sMapElement.f.bSubIndex>0)
        					return COMMONPARAMDB_CH_INVALID_ELEMENT;
        
        				if(sMapElement.f.wIndex==ODC_INTEGER8  || sMapElement.f.wIndex==ODC_UNSIGNED8)
        					ubObjLen=sizeof(UBYTE);
        
        				if(sMapElement.f.wIndex==ODC_INTEGER16 || sMapElement.f.wIndex==ODC_UNSIGNED16)
        					ubObjLen=sizeof(UWORD);
        
        				if(sMapElement.f.wIndex==ODC_INTEGER32 || sMapElement.f.wIndex==ODC_UNSIGNED32)
        					ubObjLen=sizeof(ULONG);
                    }

                        // check for length
                    if(ubObjLen==0 || (ubObjLen*8)!=sMapElement.f.bLength)
                        return COMMONPARAMDB_CH_WRONGLENGTH;
                }

                atomic_write(&sPar->tMap[uwElement-1], &sMapElement, sizeof(sMapElement));
            }
                // write no. of elements
            else
            {
                if(*((HPUBYTE)hpvBuffer)>DS301_PDO_MAXDATAOBJECT)
                    return COMMONPARAMDB_CH_WRONGLENGTH;

                sPar->bNoMapped=*((HPUBYTE)hpvBuffer);
            }
        }
    }

    return COMMONPARAMDB_CH_OK;
}

//***************************************************************************
// Slow task loop

void CanOpenPSM_Loop(void)
{
    static BOOL bPrevSyncMgrValid=FALSE;
    ULONG ulTmp;

    if(bCanOpenPSMPdoEnabled)
    {
        CANOPENPSM_PDOFASTENTRY_RX ** psRxP;
        CANOPENPSM_PDOFASTENTRY_TX ** psTxP;

            // local lock against pdo destroying
        bSlowTaskPdoProcessing=TRUE;

            // process all async pdo
        psRxP=psPdoRxList;
        psTxP=psPdoTxList;

            // process rx cob
        while(*psRxP)
            CanOpenPSM_RT_RxDecode(*psRxP++);

            // process tx cob
        while(*psTxP)
        {
                // if successfully tx'ed and not yet inhibited
        	if(  (*psTxP)->psCob->flags.bTxOK && (!((*psTxP)->psCob->flags.bInhibited)))//if(  (*psTxP)->psCob->flags&CANDRV_F_TXOK && (!((*psTxP)->psCob->flags&CANDRV_F_INHIBITED)))
            {
                CanOpenPSM_RT_TxEncode(*psTxP);

                if((*psTxP)->f.bStTxTrigger)
                {
                	memset(&((*psTxP)->psCob->flags),0,sizeof((*psTxP)->psCob->flags));//(*psTxP)->psCob->flags=CANDRV_F_DEFAULT;
                    candrv_sendcob(uwCanOpenPSMCanController, (*psTxP)->psCob);
                    (*psTxP)->f.bStTxTrigger=FALSE;
                }
            }
            
            psTxP++;
        }

            // unlock
        bSlowTaskPdoProcessing=FALSE;
    }

        // check SYNC validity, after first received
    if(bCanOpenPSMFirstSyncRecv)
    {
            // if resync is enabled then use diagnostic from syncmanager
        if(bCanOpenPSMReSyncEnable)
        {
                // if transition from valid to invalid
            if(!sSyncMgrDiagnosticOut.bValid && bPrevSyncMgrValid)
                CanOpenCM_CanFaultSignal((ULONG)DS301_COMMERR_SYNCERROR);

                // check comm cycle period, if valid
            if(sSyncMgrDiagnosticOut.bValid && tDs301Param.dCommCyclePeriod>0l)
            {
                    // get last sync time
                atomic_read(&ulTmp, &sSyncMgrDiagnosticOut.ulSyncTime, sizeof(ulTmp));

                    // if out of commcycle by 10%
                if(labs((SLONG)(ulTmp/1000-tDs301Param.dCommCyclePeriod)) > tDs301Param.dCommCyclePeriod/10)
                    if(uwSyncTimingValidCounter>0)
                        uwSyncTimingValidCounter--;
                    else
                        CanOpenCM_CanFaultSignal((ULONG)DS301_COMMERR_SYNCERROR);
                else
                    if(uwSyncTimingValidCounter<SYNC_TIMEOUTOFSYNC)
                        uwSyncTimingValidCounter++;
            }

                // update for transition detection
            bPrevSyncMgrValid=sSyncMgrDiagnosticOut.bValid;
        }
            // otherwise make it's own calculation
        else if(tDs301Param.dCommCyclePeriod>0l)
        {
            FLOAT flCommcycle=(FLOAT)tDs301Param.dCommCyclePeriod/1e6;

            if(fabs((FLOAT)uwCanOpenPSMSyncTiming/REALTIME_TASK_FREQ-flCommcycle) > flCommcycle/10.0)
                if(uwSyncTimingValidCounter>0)
                    uwSyncTimingValidCounter--;
                else
                    CanOpenCM_CanFaultSignal((ULONG)DS301_COMMERR_SYNCERROR);
            else
                if(uwSyncTimingValidCounter<SYNC_TIMEOUTOFSYNC)
                    uwSyncTimingValidCounter++;
        }
    }
}

//***************************************************************************
// cob rtr request event

void cobRTRrequestevent(CANDRV_COB * pCob)
{
    CANOPENPSM_PDOFASTENTRY_TX * psTxP;

    pCob->flags.bNewRxCob=FALSE;//fast_atomic_clear_bits(pCob->flags, CANDRV_F_NEWCOB);

        // just set flag only if enabled
    if(bCanOpenPSMPdoEnabled)
    {
        psTxP=(CANOPENPSM_PDOFASTENTRY_TX *)(UWORD)pCob->usrdata;
    
//        _atomic_(0);
        psTxP->f.bStRTRTriggered=TRUE;
//        _endatomic_();
    }
}

#endif
