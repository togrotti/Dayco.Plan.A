/*****************************************************************************/
/* Project: Ax-Zynq Control Board                                            */
/*                                                                           */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved   */
/*                                                                           */
/* File        : FlashManager.c                                              */
/* Author      : Fabio Terrile                                               */
/*                                                                           */
/* Description : Flash storage manager and sequencer                         */
/*                                                                           */
/*****************************************************************************/

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

#include "FlashManager.h"
#include "CommonUtility.h"
#include "ProgramFlashHandler.h"
#include <string.h>
#include "assert.h"

#ifdef _AXX_SYSAPP
#include "system\Os.h"

//****************************************************************************
// Locals

// OS_CREATEMUTEX(FlashMgrGlobalLock);
static OS_MUTEX FlashMgrGlobalLock;

static UBYTE ubBuffer[PROGRAMFLASH_PAGE_SIZE];
static HPVOID hpvPageAddress;

//****************************************************************************
// Initialize
u32 FlashMgrInit(void)
{
    OS_CREATEMUTEX(FlashMgrGlobalLock);

    return Flash_Init();
}

//****************************************************************************
// lock manager

UWORD FlashMgrBegin(UWORD uwTimeOut)
{
    SWORD retval;

        // try locking module
    retval=Os_MutexWait(&FlashMgrGlobalLock, uwTimeOut);

    if(retval==OS_MUTEXWAIT_TIMEOUT)
        return FLASHMGR_R_LOCKFAILED;

    assert(retval==OS_MUTEXWAIT_SIGNALED);

        // reset page address
    hpvPageAddress=NULL;

    return FLASHMGR_R_OK;
}

//****************************************************************************
// Write Data

UWORD FlashMgrWriteData(HPVOID hpvDest, const HPVOID hpvSrc, ULONG ulSize)
{
    UWORD offset=(UWORD)(((ULONG)hpvDest)&(PROGRAMFLASH_PAGE_SIZE-1));
    SWORD retval;
    HPUBYTE hpubSrc=(HPUBYTE)hpvSrc;

    hpvDest=(HPVOID)(((ULONG)hpvDest)&~(PROGRAMFLASH_PAGE_SIZE-1));

        // loop until all data is written, each time write one page
    while(ulSize)
    {
            // if page change
        if(hpvDest!=hpvPageAddress)
        {
                // if not null then flush actual data page
            if(hpvPageAddress)
            {
                retval=ProgramFlashLoadWritePage((ULONG)hpvPageAddress, ubBuffer, PROGRAMFLASH_PAGE_SIZE);

                    // if error
                if(retval)
                {
                        // nulls actual page in order that releasing resource do not flush buffer
                    hpvPageAddress=NULL;
                    return FLASHMGR_R_FLASHERROR;
                }
            }

                // new page
            hpvPageAddress=hpvDest;

                // zeroes buffer
            memset(ubBuffer, 0xFF, PROGRAMFLASH_PAGE_SIZE);

                // and restart offset
            offset=0;
        }

            // copy data to buffer
        for(;offset<PROGRAMFLASH_PAGE_SIZE && ulSize; offset++, ulSize--)
            ubBuffer[offset]=*hpubSrc++;

            // apply offset to actual dest address
        hpvDest=(HPVOID)(&((HPUBYTE)hpvDest)[offset]);
    }
    return FLASHMGR_R_OK;
}

//****************************************************************************
// Flush buffer and unlock manager

UWORD FlashMgrEnd(HPVOID * ppNextValidLocation)
{
    SWORD retval=0;

        // if not null then flush actual data page
    if(hpvPageAddress)
        retval=ProgramFlashLoadWritePage((ULONG)hpvPageAddress, ubBuffer, PROGRAMFLASH_PAGE_SIZE);

        // set next valid location, if requested
    if(ppNextValidLocation)
    {
        if(hpvPageAddress)
            *ppNextValidLocation=(HPVOID)(&((HPUBYTE)hpvPageAddress)[PROGRAMFLASH_PAGE_SIZE]);
        else
            *ppNextValidLocation=NULL;
    }

    hpvPageAddress=NULL;

        // unlock manager
    Os_MutexSignal(&FlashMgrGlobalLock);

        // previous programming error
    if(retval)
        return FLASHMGR_R_FLASHERROR;

    return FLASHMGR_R_OK;
}
#else

//****************************************************************************
// Initialize
u32 FlashMgrInit(void)
{
    return Flash_Init();
}

//****************************************************************************
// Write Data

UWORD FlashMgrWriteData(HPVOID hpvDest, const HPVOID hpvSrc, ULONG ulSize)
{
    FlashWrite((u32)hpvDest, (u8 *)hpvSrc, (u32)ulSize);

    return FLASHMGR_R_OK;
}

#endif


