/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CanOpenComDB.c                                             */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : CanOpen parameter DB interface for Common DB handling      */
/*                                                                          */
/****************************************************************************/

//#include <intrins.h>
#include <stdlib.h>
#include <string.h>

#include "CanOpenComDB.h"
#include "CanOpenComDBInfo.h"
#include "common\CommonUtility.h"
#include "CanOpenCOECommon.h"
#if CFG_CANDRV_DS301
#include "CanOpenPdoSyncMgr.h"
#endif
#include "bus\ethercat\ECATCommandMgr.h"
#include "system\SystemStatus.h"

#ifndef _INFINEON_
/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#if defined(_CRS_DBG)
#if CRS_DBGDSK
#pragma GCC optimize (0) // crs_dbg
#else
#pragma GCC optimize (2)
#endif
#else
#pragma GCC optimize (2)
#endif
#endif

//***************************************************************************
// Globals

const CANOPENCOMDB_ENTRY  * hpsCanOpenDynParamTable=NULL;
UWORD uwCanOpenDynParamCount=0;

//***************************************************************************
// Local structures

typedef struct
{
    const CANOPENCOMDB_ENTRY  * ptEntry;
    union
    {
        UWORD                       uwCHBuf[(COMMONPARAMDB_BASETYPE_MAXSIZE)/sizeof(UWORD)];
        ULONG                       ulGeneric;
    } u;
    SBYTE                           sbExtReq;
    UBYTE                           ubRecvSize;
} SDO_CONTEXT;

typedef struct
{
    UWORD                           uwDBSel;
    UWORD                           uwDBMask;
    UWORD                           uwExtDBSel;
    UWORD                           uwExtDBMask;
} INFO_SELMASKS;

//***************************************************************************
// Locals

static const INFO_SELMASKS tOdInfoTr[]=
{
    {0,0,                                               CANOPENCOMDB_F_DEFAULT,CANOPENCOMDB_F_HIDDEN},
    {COMMONPARAMDB_FLAG_WR,COMMONPARAMDB_FLAG_WR,       CANOPENCOMDB_F_PDOMAPPABLE,CANOPENCOMDB_F_PDOMAPPABLE|CANOPENCOMDB_F_HIDDEN},
    {COMMONPARAMDB_FLAG_RD,COMMONPARAMDB_FLAG_RD,       CANOPENCOMDB_F_PDOMAPPABLE,CANOPENCOMDB_F_PDOMAPPABLE|CANOPENCOMDB_F_HIDDEN},
    {0,0,                                               CANOPENCOMDB_F_PARAM,CANOPENCOMDB_F_PARAM|CANOPENCOMDB_F_HIDDEN},
    {0,0,                                               CANOPENCOMDB_F_PARAM,CANOPENCOMDB_F_PARAM|CANOPENCOMDB_F_HIDDEN},
};

//***************************************************************************
// Local prototypes;

static BOOL objdictenummatch(const CANOPENCOMDB_ENTRY  * hpDbEntry,INFO_SELMASKS * pMask,UWORD * pSkip);

//***************************************************************************
// Table checking, just for debug purpose

#ifdef _APP_DEBUG
BOOL CanOpenComDBCheckTable(void)
{
    UWORD uwIndxCount;

#if defined (_CRS_DBG)
   static volatile UWORD uwCrsDbgCounter = 0 ;
   static volatile UWORD uwCrsDbgCanOpenParamCount = 0 ;
   static volatile UWORD uwCrsDbgCanOpenParamInfoCount = 0 ;

   uwCrsDbgCanOpenParamCount = uwCanOpenParamCount ;
   uwCrsDbgCanOpenParamInfoCount = uwCanOpenParamInfoCount ;
#endif

        // Check Index Contiguity and Duplicates, only between adjacent elements that belong to same db (DS301 and COE)
    for ( uwIndxCount = 0; uwIndxCount < uwCanOpenParamCount-1; uwIndxCount++ )
        if ( (hpsCanOpenParamTable[ uwIndxCount ].uwFlags & hpsCanOpenParamTable[ uwIndxCount + 1 ].uwFlags & (CANOPENCOMDB_F_DS301_VALID|CANOPENCOMDB_F_ECATCOE_VALID)) )
        {
            assert ( hpsCanOpenParamTable[ uwIndxCount ].uwIndex <= hpsCanOpenParamTable[ uwIndxCount + 1 ].uwIndex );

            if ( hpsCanOpenParamTable[ uwIndxCount ].uwIndex == hpsCanOpenParamTable[ uwIndxCount + 1 ].uwIndex )
                assert ( hpsCanOpenParamTable[ uwIndxCount ].uwSubIndex < hpsCanOpenParamTable[ uwIndxCount + 1 ].uwSubIndex );
        }

    for ( uwIndxCount = 0; uwIndxCount < uwCanOpenParamCount; uwIndxCount++ )
    {
            // Check for PDO mappable type
        if(hpsCanOpenParamTable[ uwIndxCount ].uwFlags & CANOPENCOMDB_F_PDOMAPPABLE)
        {
#if defined (_CRS_DBG)
        	if( (hpsCanOpenParamTable[ uwIndxCount ].uwFlags & CANOPENCOMDB_F_WRDENYWHENOPER) == 0 )
        	{
#if defined(_DEBUG_TRACES)
                xil_printf("CanOpenComDB::CanOpenComDBCheckTable::hpsCanOpenParamTable[ %d ].uwFlags: %04X \r\n", uwIndxCount, hpsCanOpenParamTable[ uwIndxCount ].uwFlags);
#endif // _debug_traces
                uwCrsDbgCounter = uwIndxCount ;
                assert(TRUE) ;
        	}

        	if ( (hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->ubFlags & (COMMONPARAMDB_FLAG_HOOK|COMMONPARAMDB_FLAG_VALIDATE|COMMONPARAMDB_FLAG_RESETREQ|COMMONPARAMDB_FLAG_WRLOCKREQ)) == 0)
        	{
#if defined(_DEBUG_TRACES)
                xil_printf("CanOpenComDB::CanOpenComDBCheckTable::hpsCanOpenParamTable[ %d ].hpsComDBEntry->ubFlags: %04X\r\n", uwIndxCount, hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->ubFlags);
#endif  // _debug_traces
                uwCrsDbgCounter = uwIndxCount ;
                assert(TRUE) ;
        	}
#else // _crs_dbg
            assert ( (hpsCanOpenParamTable[ uwIndxCount ].uwFlags & CANOPENCOMDB_F_WRDENYWHENOPER) == 0 );
            assert ( (hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->ubFlags & \
                    (COMMONPARAMDB_FLAG_HOOK|COMMONPARAMDB_FLAG_VALIDATE|COMMONPARAMDB_FLAG_RESETREQ|COMMONPARAMDB_FLAG_WRLOCKREQ)) == 0);
#endif // _crs_dbg


        }

#if defined (_CRS_DBG)
        	if ((hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK) || hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->uwNElements<=0x00FE )
        	{
#if defined(_DEBUG_TRACES)
                xil_printf("CanOpenComDB::CanOpenComDBCheckTable::hpsCanOpenParamTable[ %d ].hpsComDBEntry->ubFlags: %04X\r\n", uwIndxCount, hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->ubFlags);
                xil_printf("CanOpenComDB::CanOpenComDBCheckTable::hpsCanOpenParamTable[ %d ].hpsComDBEntry->uwNElements: %04X\r\n", uwIndxCount, hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->uwNElements);
#endif // _debug_traces
                uwCrsDbgCounter = uwIndxCount ;
                assert(TRUE) ;
        	}
#else // _crs_dbg
            // Check for array size, as subindex is limited up to 0xFE, ignore for hooks
        assert ((hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK) || \
                hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->uwNElements<=0x00FE );
#endif // _crs_dbg

            // Check for STRING type, only READ is supported
        if(hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->ubType==COMMONPARAMDB_TYPE_STRING)
        {
#if defined (_CRS_DBG)
        	if ((hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_WR) == 0)
        	{
#if defined(_DEBUG_TRACES)
                xil_printf("CanOpenComDB::CanOpenComDBCheckTable::hpsCanOpenParamTable[ %d ].hpsComDBEntry->ubFlags: %04X\r\n", uwIndxCount, hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->ubFlags);
#endif // _debug_traces
                uwCrsDbgCounter = uwIndxCount ;
                assert(TRUE) ;
        	}
#else // _crs_dbg
            assert ((hpsCanOpenParamTable[ uwIndxCount ].hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_WR)==0 );
#endif // _crs_dbg
        }
    }

        // Check Index Contiguity and Duplicates in the info table
    for ( uwIndxCount = 0; uwIndxCount < uwCanOpenParamInfoCount-1; uwIndxCount++ )
    {
        assert ( hpsCanOpenParamInfo[ uwIndxCount ].uwIndex <= hpsCanOpenParamInfo[ uwIndxCount + 1 ].uwIndex );

        if ( hpsCanOpenParamInfo[ uwIndxCount ].uwIndex == hpsCanOpenParamInfo[ uwIndxCount + 1 ].uwIndex )
            assert ( hpsCanOpenParamInfo[ uwIndxCount ].uwSubIndex < hpsCanOpenParamInfo[ uwIndxCount + 1 ].uwSubIndex );
    }

    return CanOpenCOE_CheckDBTable();
}
#endif

//***************************************************************************
// Parameter Table search

ULONG CanOpenComDBEntrySearch(UWORD uwIndex, UWORD uwSubIndex, UWORD uwDBSel, const CANOPENCOMDB_ENTRY  * * ptEntry)
{
    const CANOPENCOMDB_ENTRY  * hpDbEntry=hpsCanOpenDynParamTable;
    UWORD uwCnt=uwCanOpenDynParamCount;

        // first search is done in the dynamic parameter table, here
        // index/subindex pairs are checked as there's no continuity as system table
    if(hpDbEntry)
        while(uwCnt>0)
        {
            if(hpDbEntry->uwIndex==uwIndex && hpDbEntry->uwSubIndex==uwSubIndex && (hpDbEntry->uwFlags&uwDBSel))
                break;
    
            uwCnt--;
            hpDbEntry++;
        }
    else
            // if not defined force system db search
        uwCnt=0;

        // second search is done in the system db table
    if(uwCnt==0)
    {
        hpDbEntry=hpsCanOpenParamTable;
        uwCnt=uwCanOpenParamCount;
    
        while(uwCnt>0)
        {
            if(hpDbEntry->uwIndex==uwIndex && (hpDbEntry->uwFlags&uwDBSel))
                break;
    
            uwCnt--;
            hpDbEntry++;
        }
    }

    if(uwCnt==0)
        return DS301_SDOABORT_OBJECTNOTFOUND;

        // if hook found at subindex 0 then return base element as hook
        // could also manage subindex
        // if requested subindex 0 also return here
    if((hpDbEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK) || (uwSubIndex==0))
    {
        *ptEntry=hpDbEntry;
        return DS301_SDOOK;
    }

        // if here subindex>0 was requested, then if entry has only one
        // element is not an array, then goes further in the canopendb to seek
        // matching index and subindex pairs in case of RECORD element
    if(hpDbEntry->hpsComDBEntry->uwNElements==1)
    {
        while(uwCnt>0)
        {
            if(hpDbEntry->uwIndex!=uwIndex)
                break;

            if(hpDbEntry->uwSubIndex==uwSubIndex)
                break;
    
            uwCnt--;
            hpDbEntry++;
        }

        if(uwCnt==0)
            return DS301_SDOABORT_INVALIDSUBINDEX;

        if(hpDbEntry->uwIndex!=uwIndex || hpDbEntry->uwSubIndex!=uwSubIndex)
            return DS301_SDOABORT_INVALIDSUBINDEX;
    }
        // if here then is an array
    else
        if(uwSubIndex > hpDbEntry->hpsComDBEntry->uwNElements)
            return DS301_SDOABORT_INVALIDSUBINDEX;

    *ptEntry=hpDbEntry;
    return DS301_SDOOK;
}

//***************************************************************************
// SDO Transaction callback

ULONG CanOpenComDBSdoDispatcher(DS301_SDOTRANSACTION * ptSdoStatus, UWORD uwDBSel)
{
    SDO_CONTEXT * ptCntx;
    ULONG destsize,ulretval;
    UWORD retval,bufsize;

    assert(sizeof(SDO_CONTEXT)<=sizeof(ptSdoStatus->ubContext));
    ptCntx=(SDO_CONTEXT *)ptSdoStatus->ubContext;
    retval=COMMONPARAMDB_CH_OK;

    if(ptSdoStatus->f.b.bInit)
    {
        ulretval=CanOpenComDBEntrySearch(ptSdoStatus->uwIndex,(UWORD)ptSdoStatus->ubSubIndex,uwDBSel,&ptCntx->ptEntry);

        if(ulretval!=DS301_SDOOK)
            return ulretval;

        ptCntx->sbExtReq=(ComParDBEntryCheckForCommonHandling(ptCntx->ptEntry->hpsComDBEntry)!=COMMONPARAMDB_CH_OK);

        if(!ptCntx->sbExtReq)
        {
            destsize=ComParDBEntryGetEntrySize(ptCntx->ptEntry->hpsComDBEntry);
            if(destsize==0)
                destsize=1;
        }
        else if(ptCntx->ptEntry->hpsComDBEntry->ubType==COMMONPARAMDB_TYPE_STRING)
            destsize=strlen(ptCntx->ptEntry->hpsComDBEntry->hpvData);
        else if(ptCntx->ptEntry->hpsComDBEntry->ubType==COMMONPARAMDB_TYPE_BINARY)
            destsize=(ULONG)ptCntx->ptEntry->hpsComDBEntry->swMoreType;
        else
            destsize=0l;

        if(ptSdoStatus->f.b.bDownload)
        {
            if(!ptCntx->sbExtReq && ptCntx->ptEntry->hpsComDBEntry->uwNElements>1 && ptSdoStatus->ubSubIndex==0)
                return DS301_SDOABORT_DENYWRITE;

            if((ptCntx->ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_WR)==0)
                return DS301_SDOABORT_DENYWRITE;

            if(ptCntx->ptEntry->hpsComDBEntry->uWrDeny&ulSystemStatus)
                return DS301_SDOABORT_DATACANNOTSTOREDFORSTATE;

            if(ptCntx->ptEntry->uwFlags&CANOPENCOMDB_F_WRDENYWHENOPER)
            {
#if CFG_CANDRV_DS301
            	if(uwDBSel&CANOPENCOMDB_F_DS301_VALID)
                    if(ds301_get_nmt_state()==DS301_NMT_STATUS_OPERATIONAL)
                        return DS301_SDOABORT_DATACANNOTSTOREDFORSTATE;
#endif
                if(uwDBSel&CANOPENCOMDB_F_ECATCOE_VALID)
                    if(EcatCM_IsALStateMachineOP())
                        return DS301_SDOABORT_DATACANNOTSTOREDFORSTATE;
            }
            
            if(destsize>0 && ptSdoStatus->ulDataSize>0 && ptSdoStatus->ulDataSize!=destsize)
                return DS301_SDOABORT_PARAMLENGHTUNMATCHED;

            if(!ptCntx->sbExtReq)
            {
                if(ptCntx->ptEntry->hpsComDBEntry->uwNElements>1)
                    if(ptSdoStatus->ubSubIndex==0)
                        return DS301_SDOABORT_DENYWRITE;

                ptCntx->ubRecvSize=0;
            }
            else if(ptCntx->ptEntry->hpsComDBEntry->ubType==COMMONPARAMDB_TYPE_BINARY)
                ptCntx->u.ulGeneric=0l;
            else if(ptCntx->ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK)
            {
                destsize=ptSdoStatus->ulDataSize;
                bufsize=sizeof(destsize);
                retval=(ptCntx->ptEntry->hpsComDBEntry->uf.fpfHook)(ptCntx->ptEntry->hpsComDBEntry, COMMONPARAMDB_CBFLAG_INIT|COMMONPARAMDB_CBFLAG_WR, \
                            ptSdoStatus->ubSubIndex, &destsize, &bufsize, &ptCntx->u.ulGeneric);
                if(retval!=COMMONPARAMDB_CH_OK)
                    goto translateaborts;
            }
            else
                assert(FALSE);
        }
        else
        {
            if((ptCntx->ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_RD)==0)
                return DS301_SDOABORT_DENYREAD;

            if(!ptCntx->sbExtReq)
            {
                bufsize=sizeof(ptCntx->u.uwCHBuf);
                if(ptCntx->ptEntry->hpsComDBEntry->uwNElements>1)
                {
                    if(ptSdoStatus->ubSubIndex==0)
                    {
                        destsize=1;
                        ptCntx->u.uwCHBuf[0]=(UBYTE)ptCntx->ptEntry->hpsComDBEntry->uwNElements;
                        retval=COMMONPARAMDB_CH_OK;
                    }
                    else
                        retval=ComParDBEntryRead(ptCntx->ptEntry->hpsComDBEntry, ptSdoStatus->ubSubIndex-1, ptCntx->u.uwCHBuf, &bufsize);
                }
                else
                    retval=ComParDBEntryRead(ptCntx->ptEntry->hpsComDBEntry, 0, ptCntx->u.uwCHBuf, &bufsize);

                if(retval!=COMMONPARAMDB_CH_OK)
                    return DS301_SDOABORT_GENERALPARAMINCOMPATIBILITY;

                ptCntx->ubRecvSize=0;
            }
            else if(ptCntx->ptEntry->hpsComDBEntry->ubType==COMMONPARAMDB_TYPE_BINARY || ptCntx->ptEntry->hpsComDBEntry->ubType==COMMONPARAMDB_TYPE_STRING)
                ptCntx->u.ulGeneric=0l;
            else if(ptCntx->ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK)
            {
                bufsize=sizeof(destsize);
                retval=(ptCntx->ptEntry->hpsComDBEntry->uf.fpfHook)(ptCntx->ptEntry->hpsComDBEntry, COMMONPARAMDB_CBFLAG_INIT|COMMONPARAMDB_CBFLAG_RD|COMMONPARAMDB_CBFLAG_SIZEINQUIRY, \
                            ptSdoStatus->ubSubIndex, &destsize, &bufsize, &ptCntx->u.ulGeneric);
                if(retval!=COMMONPARAMDB_CH_OK)
                    goto translateaborts;
            }
            else
                assert(FALSE);

            ptSdoStatus->ulDataSize=destsize;
        }
    }

    if(ptSdoStatus->f.b.bData)
    {
        if(!ptCntx->sbExtReq)
            if(ptSdoStatus->f.b.bDownload)
            {
                memcpy(&((UBYTE *)ptCntx->u.uwCHBuf)[ptCntx->ubRecvSize], ptSdoStatus->pubDataBuffer, (UWORD)ptSdoStatus->ulDataSize);
                ptCntx->ubRecvSize+=(UBYTE)ptSdoStatus->ulDataSize;

                if(ptSdoStatus->f.b.bLast)
                {
                    bufsize=ptCntx->ubRecvSize;
                    if(ptCntx->ptEntry->hpsComDBEntry->uwNElements>1)
                        retval=ComParDBEntryWrite(ptCntx->ptEntry->hpsComDBEntry, ptSdoStatus->ubSubIndex-1, ptCntx->u.uwCHBuf, &bufsize);
                    else
                        retval=ComParDBEntryWrite(ptCntx->ptEntry->hpsComDBEntry, 0, ptCntx->u.uwCHBuf, &bufsize);
    
                    if(retval!=COMMONPARAMDB_CH_OK)
                        return DS301_SDOABORT_GENERALPARAMINCOMPATIBILITY;
                }
            }
            else
            {
                memcpy(ptSdoStatus->pubDataBuffer, &((UBYTE *)ptCntx->u.uwCHBuf)[ptCntx->ubRecvSize], (UWORD)ptSdoStatus->ulDataSize);
                ptCntx->ubRecvSize+=(UBYTE)ptSdoStatus->ulDataSize;
            }
        else if(ptCntx->ptEntry->hpsComDBEntry->ubType==COMMONPARAMDB_TYPE_BINARY || ptCntx->ptEntry->hpsComDBEntry->ubType==COMMONPARAMDB_TYPE_STRING)
        {
            if(ptSdoStatus->f.b.bDownload)
                memcpy(&((UBYTE  *)ptCntx->ptEntry->hpsComDBEntry->hpvData)[ptCntx->u.ulGeneric], ptSdoStatus->pubDataBuffer, (UWORD)ptSdoStatus->ulDataSize);
            else
                memcpy(ptSdoStatus->pubDataBuffer, &((UBYTE  *)ptCntx->ptEntry->hpsComDBEntry->hpvData)[ptCntx->u.ulGeneric], (UWORD)ptSdoStatus->ulDataSize);
            ptCntx->u.ulGeneric+=(ULONG)ptSdoStatus->ulDataSize;
        }
        else
        {
            bufsize=(UWORD)ptSdoStatus->ulDataSize;
            if(ptSdoStatus->f.b.bDownload)
            {
                retval=(ptCntx->ptEntry->hpsComDBEntry->uf.fpfHook)(ptCntx->ptEntry->hpsComDBEntry, COMMONPARAMDB_CBFLAG_SEGMENT|COMMONPARAMDB_CBFLAG_WR, \
                            ptSdoStatus->ubSubIndex, ptSdoStatus->pubDataBuffer, &bufsize, &ptCntx->u.ulGeneric);

                if(ptSdoStatus->f.b.bLast && retval==COMMONPARAMDB_CH_OK)
                    retval=(ptCntx->ptEntry->hpsComDBEntry->uf.fpfHook)(ptCntx->ptEntry->hpsComDBEntry, COMMONPARAMDB_CBFLAG_END|COMMONPARAMDB_CBFLAG_WR, \
                                ptSdoStatus->ubSubIndex, NULL, NULL, &ptCntx->u.ulGeneric);
            }
            else
                retval=(ptCntx->ptEntry->hpsComDBEntry->uf.fpfHook)(ptCntx->ptEntry->hpsComDBEntry, COMMONPARAMDB_CBFLAG_SEGMENT|COMMONPARAMDB_CBFLAG_RD, \
                            ptSdoStatus->ubSubIndex, ptSdoStatus->pubDataBuffer, &bufsize, &ptCntx->u.ulGeneric);
        }
    }

    if(ptSdoStatus->f.b.bEnd)
        if((ptCntx->ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK) && ptSdoStatus->f.b.bUpload)
            retval=(ptCntx->ptEntry->hpsComDBEntry->uf.fpfHook)(ptCntx->ptEntry->hpsComDBEntry, COMMONPARAMDB_CBFLAG_END|COMMONPARAMDB_CBFLAG_RD, \
                        ptSdoStatus->ubSubIndex, NULL, NULL, &ptCntx->u.ulGeneric);
        
    if(ptSdoStatus->f.b.bAbort)
        if(ptCntx->ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK)
            if(ptSdoStatus->f.b.bDownload)
                retval=(ptCntx->ptEntry->hpsComDBEntry->uf.fpfHook)(ptCntx->ptEntry->hpsComDBEntry, COMMONPARAMDB_CBFLAG_ABORT|COMMONPARAMDB_CBFLAG_WR, \
                            ptSdoStatus->ubSubIndex, NULL, NULL, &ptCntx->u.ulGeneric);
            else
                retval=(ptCntx->ptEntry->hpsComDBEntry->uf.fpfHook)(ptCntx->ptEntry->hpsComDBEntry, COMMONPARAMDB_CBFLAG_ABORT|COMMONPARAMDB_CBFLAG_RD, \
                            ptSdoStatus->ubSubIndex, NULL, NULL, &ptCntx->u.ulGeneric);

translateaborts:
    switch(retval)
    {
        case COMMONPARAMDB_CH_OK:
            return DS301_SDOOK;
        case COMMONPARAMDB_CH_NO_READ_ACCESS:
            return DS301_SDOABORT_DENYREAD;
        case COMMONPARAMDB_CH_NO_WRITE_ACCESS:
            return DS301_SDOABORT_DENYWRITE;
        case COMMONPARAMDB_CH_INVALID_ELEMENT:
            return DS301_SDOABORT_INVALIDSUBINDEX;
        case COMMONPARAMDB_CH_LENGTHTOOLOW:
            return DS301_SDOABORT_PARAMLENGHTTOOLOW;
        case COMMONPARAMDB_CH_WRONGLENGTH:
            return DS301_SDOABORT_PARAMLENGHTUNMATCHED;
        case COMMONPARAMDB_CH_WRITE_DENIED:
            return DS301_SDOABORT_DATACANNOTSTOREDFORSTATE;
        case COMMONPARAMDB_CH_INVALID_DATA:
            return DS301_SDOABORT_DATACANNOTSTORED;
        case COMMONPARAMDB_CH_HARDWAREFAILURE:
            return DS301_SDOABORT_HARDWAREFAIL;
        case COMMONPARAMDB_CH_ELEMENT_NOTFOUND:
            return DS301_SDOABORT_OBJECTNOTFOUND;
        case COMMONPARAMDB_CH_NOTMAPPABLE:
            return DS301_SDOABORT_OBJECTNOTMAPPABLEINPDO;
        default:
            return DS301_SDOABORT_GENERALERROR;
    }
}

//***************************************************************************
// OD match

static BOOL objdictenummatch(const CANOPENCOMDB_ENTRY  * hpDbEntry,INFO_SELMASKS * pMask,UWORD * pSkip)
{
    BOOL objmatch=FALSE;

    *pSkip=0;

        // if array or record and hook then array/record is totally managed by hook
    if((hpDbEntry->uwFlags&(CANOPENCOMDB_F_INFO_RECORD|CANOPENCOMDB_F_INFO_ARRAY)) && \
        (hpDbEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK))
    {
        COMMONPARAMDB_ENTRY tInfo;
        UWORD wTot,wCt;

            // check ext masks before continue
        if((hpDbEntry->uwFlags&pMask->uwExtDBMask)==pMask->uwExtDBSel)
        {
                // first info is number of entries
            tInfo=*(hpDbEntry->hpsComDBEntry);
            (hpDbEntry->hpsComDBEntry->uf.fpfHook)(hpDbEntry->hpsComDBEntry, COMMONPARAMDB_CBFLAG_INFOREQ, 0, &tInfo, NULL, NULL);
            wTot=(UWORD)tInfo.swMoreType;
    
                // cycle trough entries
            for(wCt=1;wCt<=wTot;wCt++)
            {
                (hpDbEntry->hpsComDBEntry->uf.fpfHook)(hpDbEntry->hpsComDBEntry, COMMONPARAMDB_CBFLAG_INFOREQ, wCt, &tInfo, NULL, NULL);
    
                if((tInfo.ubFlags&pMask->uwDBMask)==pMask->uwDBSel)
                {
                    objmatch=TRUE;
                    break;
                }
            }
        }
    }

        // all other cases managed here
    else
        for(;;)
        {
                // if match both common and ext flags
            if((hpDbEntry->uwFlags&pMask->uwExtDBMask)==pMask->uwExtDBSel &&
                (hpDbEntry->hpsComDBEntry->ubFlags&pMask->uwDBMask)==pMask->uwDBSel)
                objmatch=TRUE;

                // check for more elements
            if(hpDbEntry[0].uwIndex==hpDbEntry[1].uwIndex)
            {
                (*pSkip)++;
                hpDbEntry++;
            }
            else
                break;
        }

        // return overall matching elements (match if at least one match)
        // and skip elements counter
    return objmatch;
}


//***************************************************************************
// Object Dictionary Enumeration (only on system db)

UWORD CanOpenComDBEnum(UWORD uwListType, UWORD uwDBSel)
{
    const CANOPENCOMDB_ENTRY  * hpDbEntry=hpsCanOpenParamTable;
    UWORD uwCnt,uwSkip;
    UWORD uwMatchCnt=0;

    for(uwCnt=0;uwCnt<uwCanOpenParamCount;uwCnt++,hpDbEntry++)
        if(hpDbEntry->uwFlags&uwDBSel)
        {
            if(objdictenummatch(hpDbEntry,&tOdInfoTr[uwListType],&uwSkip))
                uwMatchCnt++;
    
            if(uwSkip)
            {
                uwCnt+=uwSkip;
                hpDbEntry=&hpDbEntry[uwSkip];
            }
        }

    return uwMatchCnt;
}

//***************************************************************************
// Elements listing in a buffer

UWORD CanOpenComDBList(UWORD uwListType, UWORD uwDBSel, UWORD * pCntst, void * pData, UWORD wSize)
{
    const CANOPENCOMDB_ENTRY  * hpDbEntry;
    UWORD uwCnt;
    UWORD uwSkip;

		// if zero then init context
	if(wSize==0)
	{
		*pCntst=0;
		return 0;
	}

        // use locals for performance
    hpDbEntry=&hpsCanOpenParamTable[*pCntst];
    uwCnt=*pCntst;

    for(;uwCnt<uwCanOpenParamCount;uwCnt++,hpDbEntry++)
        if(hpDbEntry->uwFlags&uwDBSel)
        {
            if(objdictenummatch(hpDbEntry,&tOdInfoTr[uwListType],&uwSkip))
            {
                    // if match but no more space then exit
    			if(wSize<2)
                {
                    *pCntst=uwCnt;
    				return wSize;
                }
    
                    // add index number to the buffer
    			*(UWORD *)pData=hpDbEntry->uwIndex;
    			pData=(void *)&((UWORD *)pData)[1];
    			wSize-=2;
            }
    
            if(uwSkip)
            {
                uwCnt+=uwSkip;
                hpDbEntry=&hpDbEntry[uwSkip];
            }
        }

    *pCntst=uwCnt;
	return wSize;
}

//***************************************************************************
// Get detailed information about the object

UWORD CanOpenComDBGetInfo(UWORD uwIndex, UWORD uwSubIndex, UWORD uwDBSel, TSDOINFOOBJDESC * ptInfoObj, TSDOINFOENTRYDESC * ptInfoEntry)
{
	const CANOPENCOMDB_ENTRY  * ptEntry;
    COMMONPARAMDB_ENTRY tComEntry;
    BOOL zerosubidxsimul=FALSE;

	if(CanOpenComDBEntrySearch(uwIndex,uwSubIndex,uwDBSel,&ptEntry)!=DS301_SDOOK)
		return 0;
	
	tComEntry=*(ptEntry->hpsComDBEntry);

        // if array or record get number of entries
    if(ptEntry->uwFlags&(CANOPENCOMDB_F_INFO_RECORD|CANOPENCOMDB_F_INFO_ARRAY))
    {
            // if hook then array/record is totally managed by hook, so request to it
        if(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK)
            (ptEntry->hpsComDBEntry->uf.fpfHook)(ptEntry->hpsComDBEntry, COMMONPARAMDB_CBFLAG_INFOREQ, uwSubIndex, &tComEntry, NULL, NULL);

        else if(ptEntry->hpsComDBEntry->uwNElements>1)
        {
            tComEntry.swMoreType=(UWORD)ptEntry->hpsComDBEntry->uwNElements;

                // if element is common db array, then deny PDO mapping for subindex 0, force data type UNSIGNED8 and read-only
            if(uwSubIndex==0)
                zerosubidxsimul=TRUE;
        }
        else
        {
            UWORD uwDstSize=sizeof(tComEntry.swMoreType);
            if(ComParDBEntryRead(ptEntry->hpsComDBEntry, 0, &(tComEntry.swMoreType), &uwDstSize)!=COMMONPARAMDB_CH_OK)
                return 0;
        }
    }
    else
        tComEntry.swMoreType=0;

        // fill Info Entry
        // translate data type
    switch(tComEntry.ubType&COMMONPARAMDB_TYPE_TYPE_MASK)
    {
        case COMMONPARAMDB_TYPE_UBYTE:
            ptInfoEntry->DataType=DS301_ODC_UNSIGNED8;
            ptInfoEntry->BitLength=8;
            break;

        case COMMONPARAMDB_TYPE_SBYTE:
            ptInfoEntry->DataType=DS301_ODC_INTEGER8;
            ptInfoEntry->BitLength=8;
            break;

        case COMMONPARAMDB_TYPE_UWORD:
            ptInfoEntry->DataType=DS301_ODC_UNSIGNED16;
            ptInfoEntry->BitLength=16;
            break;

        case COMMONPARAMDB_TYPE_SWORD:
            ptInfoEntry->DataType=DS301_ODC_INTEGER16;
            ptInfoEntry->BitLength=16;
            break;

        case COMMONPARAMDB_TYPE_ULONG:
            ptInfoEntry->DataType=DS301_ODC_UNSIGNED32;
            ptInfoEntry->BitLength=32;
            break;

        case COMMONPARAMDB_TYPE_SLONG:
            ptInfoEntry->DataType=DS301_ODC_INTEGER32;
            ptInfoEntry->BitLength=32;
            break;

        case COMMONPARAMDB_TYPE_UQWRD:
            ptInfoEntry->DataType=DS301_ODC_UNSIGNED64;
            ptInfoEntry->BitLength=64;
            break;

        case COMMONPARAMDB_TYPE_SQWRD:
            ptInfoEntry->DataType=DS301_ODC_INTEGER64;
            ptInfoEntry->BitLength=64;
            break;

        case COMMONPARAMDB_TYPE_FLOAT:
            ptInfoEntry->DataType=DS301_ODC_REAL32;
            ptInfoEntry->BitLength=32;
            break;

        case COMMONPARAMDB_TYPE_BITW:
            ptInfoEntry->DataType=DS301_ODC_INTEGER8;
            ptInfoEntry->BitLength=8;
            break;

        case COMMONPARAMDB_TYPE_BITD:
            ptInfoEntry->DataType=DS301_ODC_INTEGER8;
            ptInfoEntry->BitLength=8;
            break;

        case COMMONPARAMDB_TYPE_STRING:
            ptInfoEntry->DataType=DS301_ODC_VISIBLE_STRING;
            ptInfoEntry->BitLength=0xffff;
            break;

        default:
            ptInfoEntry->DataType=DS301_ODC_DOMAIN;
            ptInfoEntry->BitLength=0xffff;
            break;
    }

        // access type
    ptInfoEntry->ObjAccess=0;

	if(tComEntry.ubFlags&COMMONPARAMDB_FLAG_RD)
	{
		if(ptEntry->uwFlags&CANOPENCOMDB_F_PDOMAPPABLE)
			ptInfoEntry->ObjAccess|=OBJACCESS_TXPDOMAPPING;
		
		ptInfoEntry->ObjAccess|=ACCESS_READ;
	}

	if(tComEntry.ubFlags&COMMONPARAMDB_FLAG_WR)
	{
		if(ptEntry->uwFlags&CANOPENCOMDB_F_PDOMAPPABLE)
			ptInfoEntry->ObjAccess|=OBJACCESS_RXPDOMAPPING;
		
		ptInfoEntry->ObjAccess|=ACCESS_WRITE_PREOP;

		if((ptEntry->uwFlags&CANOPENCOMDB_F_WRDENYWHENOPER)==0)
			ptInfoEntry->ObjAccess|=ACCESS_WRITE_SAFEOP|ACCESS_WRITE_OP;
	}

    if(zerosubidxsimul)
    {
        ptInfoEntry->DataType=DS301_ODC_UNSIGNED8;
        ptInfoEntry->BitLength=8;
		ptInfoEntry->ObjAccess&=~(OBJACCESS_TXPDOMAPPING|OBJACCESS_RXPDOMAPPING|ACCESS_WRITE);
		ptInfoEntry->ObjAccess|=ACCESS_READ;
    }

	if(ptEntry->uwFlags&CANOPENCOMDB_F_PARAM)
		ptInfoEntry->ObjAccess|=OBJACCESS_BACKUP|OBJACCESS_SETTINGS;

        // fill Info Obj Flags
    ptInfoObj->ObjFlags=tComEntry.swMoreType;
    switch(ptEntry->uwFlags&(CANOPENCOMDB_F_INFO_RECORD|CANOPENCOMDB_F_INFO_ARRAY))
    {
        case CANOPENCOMDB_F_INFO_ARRAY:
            ptInfoObj->DataType=DS301_ODC_MANUFACTURER;
            ptInfoObj->ObjFlags|=OBJCODE_ARR << OBJFLAGS_OBJCODESHIFT;
            break;

        case CANOPENCOMDB_F_INFO_RECORD:
            ptInfoObj->DataType=DS301_ODC_MANUFACTURER;            
            ptInfoObj->ObjFlags|=OBJCODE_REC << OBJFLAGS_OBJCODESHIFT;
            break;

        default:
            ptInfoObj->DataType=(UWORD)ptInfoEntry->DataType;            
            ptInfoObj->ObjFlags|=OBJCODE_VAR << OBJFLAGS_OBJCODESHIFT;
            break;
    }

	return 1;
}

//***************************************************************************
// Get textual description of the object

UWORD CanOpenComDBGetDescription(UWORD uwIndex, UWORD uwSubIndex, VISIBLE_STRING * pData, UWORD uwSize)
{
    const CANOPENCOMDB_INFODESCR  * pEntry=hpsCanOpenParamInfo;
	UWORD ct,sz;
	
	for(ct=0;ct<uwCanOpenParamInfoCount;ct++,pEntry++)
		if(pEntry->uwIndex==uwIndex && pEntry->uwSubIndex==uwSubIndex)
		{
            sz=strlen(pEntry->sDescr);
			sz=sz<uwSize?sz:uwSize;

            memcpy(pData, pEntry->sDescr, sz);

			return sz;
		}
	
	return 0;
}
