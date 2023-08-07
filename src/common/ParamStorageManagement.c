/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ParamStorageManagement.c                                   */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Parameter Storage Management                               */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "ParamStorageManagement.h"
#include "BlockStorage.h"
#include "common\ProgramFlashHandler.h"
#include "system\SysAppDataCodes.h"
#ifdef _AXX_SYSAPP
#include "system\SysAppInfo.h"
#include "system\SysAppGlobals.h"
#include "system\Os.h"
#else
#include "system\BootBlockInfo.h"
#include "system\BootBlockGlobals.h"
#endif
#include "common\CommonUtility.h"
#include "common\FlashManager.h"
#include <string.h>

//***************************************************************************
// Globals

#ifdef _AXX_SYSAPP
    // If valid then parameter load is possible
PARMGM_ENDSIGNATURE sParMgmParamSignature;

    // Used only from save and restore functions, as verify and load
    // are only used in the boot code (then without OS)
//OS_CREATEMUTEX(sParMgmMutex);
static OS_MUTEX sParMgmMutex;
#endif

//***************************************************************************
// Defines

#define WAIT_FLASHMGR_LOCK          1500            // msec

//***************************************************************************
// Data structure

typedef struct
{
    HPVOID hpvStart;
    ULONG ulSize;
} BLOCKDEFS;

//***************************************************************************
// Locals

#ifdef _AXX_SYSAPP
static const BLOCKDEFS sParamStorageBlockDefs[]=
{
    {(HPVOID)PARAMSTOR_BLK0_START, (ULONG)PARAMSTOR_BLK0_SIZE},
    {(HPVOID)PARAMSTOR_BLK1_START, (ULONG)PARAMSTOR_BLK1_SIZE},
};

#define PARAMSTORBLOCKCOUNT     (sizeof(sParamStorageBlockDefs)/sizeof(BLOCKDEFS))

static SWORD uwNextWritableParamBlock=-1;
#endif

//***************************************************************************
// Local prototypes;

static BOOL _addblock(SWORD swCode, HPVOID fpvSrc, UWORD uwSize, HPVOID * ppvDest, ULONG * pulDestSize, UWORD * puwCRC);
#ifdef _AXX_SYSAPP
static SWORD _param_save(void);
#endif

//***************************************************************************
// Add one block data with correlated header and update pointer to next
// location and remaining storage size; data are read from source in chunks
// of 16 bytes (atomically), then processed in local buffer, in order to
// avoid source data modification while processing it, resulting in
// presumed successfull data saved, but pratically with wrong crc

static BOOL _addblock(SWORD swCode, HPVOID hpvSrc, UWORD uwSize, HPVOID * ppvDest, ULONG * pulDestSize, UWORD * puwCRC)
{
    static UBYTE  ubLocBuffer[ATOMIC_CHUCK16_SIZE];
    static BLKSTOR_HEADER  sTail;
    UWORD uwChunk, uwLocSize;

        // check for no oversize
	assert(*pulDestSize > (ULONG)sizeof(sTail)+uwSize);

        // create tail info
    if(blkstor_tail_begin(&sTail, swCode)<0)
        return TRUE;

    uwLocSize=uwSize;
    while(uwLocSize>0)
    {
            // get chunk size
        uwChunk=uwLocSize>ATOMIC_CHUCK16_SIZE ? ATOMIC_CHUCK16_SIZE : uwLocSize;

            // get data
//        atomic_read_chunk16(ubLocBuffer, hpvSrc);
        memcpy(ubLocBuffer, hpvSrc, uwChunk);

            // write data
       if(FlashMgrWriteData(*ppvDest, ubLocBuffer, uwChunk) != FLASHMGR_R_OK)
           return TRUE;

            // add data to header info
        if(blkstor_tail_addata(&sTail, ubLocBuffer, uwChunk)<0)
            return TRUE;

            // update global CRC
        *puwCRC=crc16(*puwCRC, ubLocBuffer, uwChunk);

            // update pointers and left size
        *ppvDest=&((UBYTE  *)*ppvDest)[uwChunk];
        hpvSrc=&((UBYTE  *)hpvSrc)[uwChunk];
        uwLocSize-=uwChunk;
    }

        // finalize tail info
    if(blkstor_tail_end(&sTail)<0)
        return TRUE;

        // write complete header
   if(FlashMgrWriteData(*ppvDest, &sTail, sizeof(sTail)) != FLASHMGR_R_OK)
       return TRUE;

        // update dest address
    *ppvDest=&((UBYTE  *)*ppvDest)[sizeof(sTail)];

        // update global CRC
    *puwCRC=crc16(*puwCRC, (HPVOID)&sTail, sizeof(sTail));

        // update remaining size
    *pulDestSize-=(ULONG)sizeof(sTail)+uwSize;

	return FALSE;
}

#ifdef _AXX_SYSAPP
//***************************************************************************
// initialization
void parmgm_par_init(void)
{
    OS_CREATEMUTEX(sParMgmMutex);
}

//***************************************************************************
// load default values of parameters

void parmgm_par_default(void)
{
    const PARMGM_LIST * parptr;
    UWORD ct;

        // rewrite all param blocks to storage
	for(ct=0,parptr=psParMgmGlobalList; ct<uwParMgmGlobalListSize; ct++,parptr++)
	{
        if(!(parptr->uwFlags&PARMGM_F_NOSETDEFAULT))
        {
                // first erase memory block
            memset(parptr->hpvData, 0, parptr->uwDataSize);

                // if not NULL rewrite memory block with default, size of default could be
                // same or less
            if(parptr->hpvDefault)
                memcpy(parptr->hpvData, parptr->hpvDefault, parptr->uwDefaultDataSize);
        }
	}
}

//***************************************************************************
// find and verify the validity of a parameter block; it set also the
// right block for next save parameter

SWORD parmgm_par_verify(void)
{
	PARMGM_ENDSIGNATURE lendsign, selsign;
    HPVOID storageptr;
    HPVOID blkptr;
    ULONG storagesize;
    UWORD ct;
    SWORD retval,blkcode,selsgnblock;
    BOOL crcok;//,crcerrfound=FALSE;

#ifdef _APP_DEBUG
        // check size of all registered param structures,
        // size must be multiple of 2
	for(ct=0; ct<uwParMgmGlobalListSize; ct++)
        if(psParMgmGlobalList[ct].uwDataSize%4)
        {
#if (defined(_DEBUG_TRACES))
    	    xil_printf("ParamStorageManagement::parmgm_par_verify() : item %d : FATAL_ERROR_INVALID_PARAMSIZE\r\n", ct);
#endif
            //ShowFatalErrorAndHalt(FATAL_ERROR_INVALID_PARAMSIZE);
    	    ;
        }
#endif

        // invalidate signature block and next writable block
    memset(&sParMgmParamSignature,0,sizeof(sParMgmParamSignature));
    uwNextWritableParamBlock=0;

        // check if all blocks are erased
    for(ct=0; ct<PARAMSTORBLOCKCOUNT; ct++)
        if((retval=ProgramFlashErasedCheck(sParamStorageBlockDefs[ct].hpvStart, sParamStorageBlockDefs[ct].ulSize)) != 0)
            break;

        // if all erased, then assume no saved parameters
    if(retval==0)
        return PARMGM_B_ERASED;

        // zeroes local selected valid signature
    memset(&selsign, 0, sizeof(selsign));
    selsgnblock=0;

        // now scan each block to find and select the right param signature,
        // the one with higher timestamp (and of course crc valid)
    for(ct=0; ct<PARAMSTORBLOCKCOUNT; ct++)
    {
        storageptr=sParamStorageBlockDefs[ct].hpvStart;
        storagesize=sParamStorageBlockDefs[ct].ulSize;

            // seek in the block for signature
        while((blkcode=blkstor_enumvalid(&storageptr, &storagesize, &blkptr)) != BLKSTOR_ERR_NOTFOUND)
            if(blkcode==DATACODE_PARAM_ENDSIGNATURE)
            {
                memset(&lendsign,0,sizeof(lendsign));

                blkstor_getdata(blkptr,0,DATACODE_PARAM_ENDSIGNATURE,&lendsign,sizeof(lendsign));

                crcok=(crc16(0, lendsign.hpvBlockStart, lendsign.uwBlockSize)==lendsign.uwBlockCRC);

//                if(!crcok)
//                    crcerrfound=TRUE;

                    // if came from same application type and crc is ok
                if(lendsign.uwApplicatType==sSysAppInfo.uwApplicatType && crcok)
                        // and if it is the newer one (or actual one not initialized)
                    if(lendsign.ulTimeStamp > selsign.ulTimeStamp || selsign.ulTimeStamp==0l)
                    {
                        selsign=lendsign;
                        selsgnblock=ct;
                    }
            }
    }

        // valid and newer saved parameters found
    if(selsign.uwApplicatType==sSysAppInfo.uwApplicatType)
    {
            // write found signature in global parameter
        sParMgmParamSignature=selsign;

            // and select next block as parameter storage
        selsgnblock++;
        uwNextWritableParamBlock=(selsgnblock>=PARAMSTORBLOCKCOUNT?0:selsgnblock);

        return PARMGM_B_OK;
    }

        // if here, flash is not erased and nothing valid was found
    return PARMGM_B_ERROR;
}

//***************************************************************************
// Parameter load; assume global signature initialized by
// parmgm_par_verify function

SWORD parmgm_par_load(UBYTE ubOption)
{
    HPVOID storageptr=sParMgmParamSignature.hpvBlockStart;
    HPVOID pfound;
    ULONG storagesize=(ULONG)sParMgmParamSignature.uwBlockSize;
    SWORD blkcode,ct,retval,datasz;

        // check pointer/size
    assert(storageptr && storagesize);

    for(;;)
    {
            // analyze each valid block between boundaries
        blkcode=blkstor_enumvalid(&storageptr,&storagesize,&pfound);

            // not found, then end of scan
        if(blkcode==BLKSTOR_ERR_NOTFOUND)
            break;
        if(blkcode<0)
            return PARMGM_B_ERROR;

            // seek found block in the global structure
        for(ct=0; ct<uwParMgmGlobalListSize; ct++)
            if(psParMgmGlobalList[ct].uwDataCode==blkcode && !(psParMgmGlobalList[ct].uwFlags&PARMGM_F_STOREONLY) &&
                (ubOption!=PARMGM_LOAD_COREONLY || (psParMgmGlobalList[ct].uwFlags&PARMGM_F_CORE)))
            {
                if(!(psParMgmGlobalList[ct].uwFlags&PARMGM_F_CALLBACK))
                {
                        // retrieve data
                    retval=blkstor_getdata(pfound,0,blkcode,psParMgmGlobalList[ct].hpvData,(SWORD)psParMgmGlobalList[ct].uwDataSize);

                    if(retval<0)
                        return PARMGM_B_ERROR;
                }
                else
                {
                        // get data address for the callback
                    retval=blkstor_getaddr(pfound,0,blkcode,&pfound);

                    if(retval<0)
                        return PARMGM_B_ERROR;

                        // get data size, if zero take returned data size
                    datasz=psParMgmGlobalList[ct].uwDataSize;
                    if(datasz==0)
                        datasz=retval;

                        // check for length (if not zero)
                    if(datasz<retval)
                        return BLKSTOR_ERR_INVALIDSIZE;

                        // callback function
                    retval=((SWORD (*)(HPVOID,UWORD))(psParMgmGlobalList[ct].hpvData))(pfound,datasz);

                    if(retval<0)
                        return PARMGM_B_ERROR;
                }

                break;
            }
    }

    return PARMGM_B_OK;
}

//***************************************************************************
// Parameter save entry point: it save two time in order to have at least
// one valid block

SWORD parmgm_par_save(void)
{
    SWORD retval;

        // first time
    retval=_param_save();
    if(retval!=PARMGM_B_OK)
        return retval;

        // second time
    return _param_save();
}

//***************************************************************************
// Parameter save; assume global next block selection initialized by
// parmgm_par_verify function

static SWORD _param_save(void)
{
	static PARMGM_ENDSIGNATURE bsign;
    const PARMGM_LIST * parptr;
    HPVOID storageptr;
    ULONG storagesize;
	UWORD ct,globalcrc=0;
    SWORD retval = 1;

    assert(uwNextWritableParamBlock>=0 && uwNextWritableParamBlock<PARAMSTORBLOCKCOUNT);

//	system_wdt_clear();     //$TEST$
//	syslog_logtimer(FALSE); //$TEST$

        // if diagnostic mode detected prevent parameters saving in order
        // to preserve user application parameters
    if(uwSystemEnterDiagnostic==SYSTEM_ENTERDIAGNOSTIC_KEY)
        return PARMGM_B_OK;

        // get exclusive access to local module
    retval=Os_MutexWait(&sParMgmMutex, WAIT_FLASHMGR_LOCK);

    if(retval==OS_MUTEXWAIT_TIMEOUT)
        return PARMGM_B_LOCKFAILED;

    assert(retval==OS_MUTEXWAIT_SIGNALED);

        // now signal parameter flash writing in progress
    bSysStatParametersSaving=1;

        // select data block
    bsign.hpvBlockStart=storageptr=sParamStorageBlockDefs[uwNextWritableParamBlock].hpvStart;
    storagesize=sParamStorageBlockDefs[uwNextWritableParamBlock].ulSize;

        // erase all block
    if(ProgramFlashErase(storageptr, storagesize))
    {
        retval=PARMGM_B_ERROR;
        goto exit_on_error;
    }

            // lock flash manager
    if(FlashMgrBegin(WAIT_FLASHMGR_LOCK)==FLASHMGR_R_LOCKFAILED)
    {
        retval=PARMGM_B_LOCKFAILED;
        goto exit_on_error;
    }

        // add all param blocks to storage (but the ones with RESTOREONLY flag)
    for(ct=0,parptr=psParMgmGlobalList; ct<uwParMgmGlobalListSize; ct++,parptr++)
        if(!(parptr->uwFlags&PARMGM_F_RESTOREONLY))
        {
            if(parptr->uwFlags&PARMGM_F_PTRSOURCE)
            {
                    // retrieve data structure that contain pointer and size to data
                PARMGM_PTRSOURCE  * hpsPtr=(PARMGM_PTRSOURCE *)parptr->hpvData;

                    // if NULL ignore and does not save
                if(hpsPtr->hpvData!=NULL)
                    retval=_addblock(parptr->uwDataCode, hpsPtr->hpvData, hpsPtr->uwDataSize, &storageptr, &storagesize, &globalcrc);
                else
                    retval=FALSE;
            }
            else
                retval=_addblock(parptr->uwDataCode, parptr->hpvData, parptr->uwDataSize, &storageptr, &storagesize, &globalcrc);

            if(retval)
            {
                    // unlock flash manager and return
                FlashMgrEnd(NULL);
                retval=PARMGM_B_ERROR;
                goto exit_on_error;
            }
        }

        // fillup signature data structure
    bsign.uwApplicatType=sSysAppInfo.uwApplicatType;
    bsign.uwVersionMajor=sSysAppInfo.uwVersionMajor;
    bsign.uwVersionMinor=sSysAppInfo.uwVersionMinor;
    bsign.uwBlockSize=(UWORD)((ULONG)storageptr-(ULONG)bsign.hpvBlockStart);
    bsign.uwBlockCRC=globalcrc;
    bsign.ulTimeStamp=ulSysTimersTotalPowerOnTime;

        // write signature
    if(_addblock(DATACODE_PARAM_ENDSIGNATURE, &bsign, sizeof(bsign), &storageptr, &storagesize, &globalcrc))
    {
            // unlock flash manager and return
        FlashMgrEnd(NULL);
        retval=PARMGM_B_ERROR;
        goto exit_on_error;
    }

        // unlock flash manager
    FlashMgrEnd(NULL);

        // select next block as parameter storage
    uwNextWritableParamBlock=(uwNextWritableParamBlock+1>=PARAMSTORBLOCKCOUNT?0:uwNextWritableParamBlock+1);

    retval=PARMGM_B_OK;

exit_on_error:
        // reset parameter flash writing in progress
    bSysStatParametersSaving=0;

        // release local module
    Os_MutexSignal(&sParMgmMutex);

    return retval;
}

//***************************************************************************
// restore default parameters (erase all)

SWORD parmgm_par_restore(void)
{
    UWORD ct;
    SWORD retval;

         // get exclusive access to local module
    retval=Os_MutexWait(&sParMgmMutex, WAIT_FLASHMGR_LOCK);

    if(retval==OS_MUTEXWAIT_TIMEOUT)
        return PARMGM_B_LOCKFAILED;

    assert(retval==OS_MUTEXWAIT_SIGNALED);

        // erase all parameters blocks
    for(ct=0; ct<PARAMSTORBLOCKCOUNT; ct++)
       if(ProgramFlashErase(sParamStorageBlockDefs[ct].hpvStart, sParamStorageBlockDefs[ct].ulSize))
       {
           retval=PARMGM_B_ERROR;
           goto exit_on_error;
       }

        // invalidate signature block and next writable block
    memset(&sParMgmParamSignature,0,sizeof(sParMgmParamSignature));
    uwNextWritableParamBlock=0;

    retval=PARMGM_B_OK;

        // signal reset required
    atomic_long_set_bits( &ulSystemWarnings, SYSTEMWARNINGS_SAVEANDRESETREQ );

exit_on_error:
        // release local module
    Os_MutexSignal(&sParMgmMutex);

    return retval;
}

//***************************************************************************
// immediate restore of default parameters (only in ram, do not write flash)

SWORD parmgm_par_immrestore(void)
{
    SWORD retval;

         // get exclusive access to local module
    retval=Os_MutexWait(&sParMgmMutex, WAIT_FLASHMGR_LOCK);

    if(retval==OS_MUTEXWAIT_TIMEOUT)
        return PARMGM_B_LOCKFAILED;

    assert(retval==OS_MUTEXWAIT_SIGNALED);

        // rewrite all
    parmgm_par_default();

        // signal reset required
    atomic_long_set_bits( &ulSystemWarnings, SYSTEMWARNINGS_SAVEANDRESETREQ );

        // release local module
    Os_MutexSignal(&sParMgmMutex);

    return PARMGM_B_OK;
}

//***************************************************************************
// BootBlock Info Load

SWORD parmgm_bootblock_load(HEADER_FOOTER_INFO * hpHeader, HPULONG hpulSysID, HPULONG hpulFwID)
{
    HPVOID storageptr=(HPVOID)BOOTBLOCKPARAM_START;
    ULONG storagesize=(ULONG)BOOTBLOCKPARAM_SIZE;
    HPVOID pfound;
    SWORD blkcode,retval;

        // check pointer/size
    assert(storageptr && storagesize);

    for(;;)
    {
            // analyze each valid block between boundaries
        blkcode=blkstor_enumvalid(&storageptr,&storagesize,&pfound);

            // not found, then end of scan
        if(blkcode==BLKSTOR_ERR_NOTFOUND)
            break;
        else if(blkcode==DATACODE_PARAM_BOOTBLOCK_INFO)
        {
            retval=blkstor_getdata(pfound,0,blkcode,hpHeader,sizeof(HEADER_FOOTER_INFO));
            if(retval<0)
                return PARMGM_B_ERROR;
        }
        else if(blkcode==DATACODE_PARAM_BOOTBLOCK_SYSID)
        {
            retval=blkstor_getdata(pfound,0,blkcode,hpulSysID,4);
            if(retval<0)
                return PARMGM_B_ERROR;
        }
        else if(blkcode==DATACODE_PARAM_BOOTBLOCK_APPID)
        {
            retval=blkstor_getdata(pfound,0,blkcode,hpulFwID,4);
            if(retval<0)
                return PARMGM_B_ERROR;
        }
    }

    return PARMGM_B_OK;
}

#endif

#ifdef _AXX_BOOTBLOCK
//***************************************************************************
// BootBlock Info Save

SWORD parmgm_bootblock_save(void)
{
    static PARMGM_ENDSIGNATURE bsign;
    HPVOID storageptr;
    ULONG storagesize;
    UWORD globalcrc=0;
    SWORD retval = 1;

        // select data block
    bsign.hpvBlockStart=storageptr=(HPVOID)BOOTBLOCKPARAM_START;
    storagesize=(ULONG)BOOTBLOCKPARAM_SIZE;

        // erase all block
    if(ProgramFlashErase(storageptr, storagesize))
    {
        retval=PARMGM_B_ERROR;
        goto exit_on_error;
    }

        // add all bootblock information to storage
    retval=_addblock(DATACODE_PARAM_BOOTBLOCK_INFO, (HPVOID)&sBootBlockInfo, sizeof(sBootBlockInfo), &storageptr, &storagesize, &globalcrc);
    if(retval)
    {
        retval=PARMGM_B_ERROR;
        goto exit_on_error;
    }

    retval=_addblock(DATACODE_PARAM_BOOTBLOCK_SYSID, (HPVOID)chSystemId, sizeof(chSystemId), &storageptr, &storagesize, &globalcrc);
    if(retval)
    {
        retval=PARMGM_B_ERROR;
        goto exit_on_error;
    }

    retval=_addblock(DATACODE_PARAM_BOOTBLOCK_APPID, (HPVOID)chFwApplicationId, sizeof(chFwApplicationId), &storageptr, &storagesize, &globalcrc);
    if(retval)
    {
        retval=PARMGM_B_ERROR;
        goto exit_on_error;
    }

        // fillup signature data structure
    bsign.uwApplicatType=sBootBlockInfo.uwApplicatType;
    bsign.uwVersionMajor=sBootBlockInfo.uwVersionMajor;
    bsign.uwVersionMinor=sBootBlockInfo.uwVersionMinor;
    bsign.uwBlockSize=(UWORD)((ULONG)storageptr-(ULONG)bsign.hpvBlockStart);
    bsign.uwBlockCRC=globalcrc;
    bsign.ulTimeStamp=(ULONG)sBootBlockInfo.uwBuildNumber;

        // write signature
	if(_addblock(DATACODE_PARAM_ENDSIGNATURE, &bsign, sizeof(bsign), &storageptr, &storagesize, &globalcrc))
    {
        retval=PARMGM_B_ERROR;
        goto exit_on_error;
    }

    retval=PARMGM_B_OK;

exit_on_error:

    return retval;
}

//***************************************************************************
// BootBlock Info Verify

SWORD parmgm_bootblock_verify(void)
{
	PARMGM_ENDSIGNATURE lendsign, selsign;
    HPVOID storageptr;
    HPVOID blkptr;
    ULONG storagesize;
    SWORD retval,blkcode;
    BOOL crcok;
    HEADER_FOOTER_INFO sHeader;
    ULONG ulID;

    storageptr=(HPVOID)BOOTBLOCKPARAM_START;
    storagesize=(ULONG)BOOTBLOCKPARAM_SIZE;

        // check if all blocks are erased
    retval=ProgramFlashErasedCheck(storageptr, storagesize);

        // if all erased, then assume no saved parameters
    if(retval==0)
        return PARMGM_B_ERASED;

        // zeroes local selected valid signature
    memset(&selsign, 0, sizeof(selsign));

        // seek in the block for signature
    while((blkcode=blkstor_enumvalid(&storageptr, &storagesize, &blkptr)) != BLKSTOR_ERR_NOTFOUND)
        if(blkcode==DATACODE_PARAM_BOOTBLOCK_INFO)
        {
            memset(&sHeader,0,sizeof(sHeader));

            blkstor_getdata(blkptr,0,DATACODE_PARAM_BOOTBLOCK_INFO,&sHeader,sizeof(sHeader));

            if(memcmp(&sHeader, &sBootBlockInfo, sizeof(sHeader)))
                return PARMGM_B_ERROR;
        }
        else if(blkcode==DATACODE_PARAM_BOOTBLOCK_SYSID)
        {
            memset(&ulID,0,sizeof(ulID));

            blkstor_getdata(blkptr,0,DATACODE_PARAM_BOOTBLOCK_SYSID,&ulID,sizeof(ulID));

            if(memcmp(&ulID, chSystemId, sizeof(ulID)))
                return PARMGM_B_ERROR;
        }
        else if(blkcode==DATACODE_PARAM_BOOTBLOCK_APPID)
        {
            memset(&ulID,0,sizeof(ulID));

            blkstor_getdata(blkptr,0,DATACODE_PARAM_BOOTBLOCK_APPID,&ulID,sizeof(ulID));

            if(memcmp(&ulID, chFwApplicationId, sizeof(ulID)))
                return PARMGM_B_ERROR;
        }
        else if(blkcode==DATACODE_PARAM_ENDSIGNATURE)
        {
            memset(&lendsign,0,sizeof(lendsign));

            blkstor_getdata(blkptr,0,DATACODE_PARAM_ENDSIGNATURE,&lendsign,sizeof(lendsign));

            crcok=(crc16(0, lendsign.hpvBlockStart, lendsign.uwBlockSize)==lendsign.uwBlockCRC);

            if(crcok)
                selsign=lendsign;
        }

        // if here, flash is not erased and nothing valid was found
    if(selsign.uwApplicatType == sBootBlockInfo.uwApplicatType)
        return PARMGM_B_OK;
    else
        return PARMGM_B_ERROR;
}
#endif
