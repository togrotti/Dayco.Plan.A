/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ParametersCheck.c                                          */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Common parameters validity checker                         */
/*                                                                          */
/****************************************************************************/

// Compiler Option
#pragma GCC optimize (2)

#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "common\TaskScheduler.h"
#include "system\SystemStatus.h"
#include "system\SystemAlarms.h"
#include "common\ParametersCheck.h"

//***************************************************************************
// Defines

#define ERR_LIST_SIZE       16

//***************************************************************************
// Globals

UWORD               uwParChkParamCode=0;
ULONG               ulParChkLastUpdateTime;

//***************************************************************************
// Locals

static BOOL         bWarningActive=FALSE;
static UBYTE        ubCnt=0;
static UWORD        puwErrList[ERR_LIST_SIZE];
static ULONG        ulLastTimeStamp;

//***************************************************************************
// Local functions

static void slowtask(void);

//***************************************************************************
// Initialization entry point

BOOL ParChk_Init(void)
{
    UWORD uwCt;

    ulParChkLastUpdateTime=0l;

        // erase err list array
    for(uwCt=0;uwCt<ERR_LIST_SIZE;uwCt++)
        puwErrList[uwCt]=0;

        // add tasks
    return TaskSched_AddBackgroundTask(&slowtask);
}

//***************************************************************************
// Signal parameter value error

void ParChk_SignalValueError(UWORD uwParamCode)
{
    UWORD uwCt;

//    if(_testset_(bWarningActive))
    if(bWarningActive)
    {
        uwParChkParamCode=uwParamCode;
        atomic_long_set_bits( &ulSystemWarnings, SYSTEMWARNINGS_INVALIDPARAMETERS );
        if(bSysStatBooting)
            uwSystemBootErrorCode=SYSTEMBOOTERR_PARAMETERERROR;
    }

        // find and insert in the list
    for(;;)
    {
            // check if already in the list
        for(uwCt=0;uwCt<ERR_LIST_SIZE;uwCt++)
            if(puwErrList[uwCt]==uwParamCode)
                return;

            // find empty entry
        for(uwCt=0;uwCt<ERR_LIST_SIZE;uwCt++)
            if(puwErrList[uwCt]==0)
                break;

            // if no empty entry found then exit
        if(uwCt>=ERR_LIST_SIZE)
            return;

            // exclusive test and set
//        DISABLE_IRQ();
        if(puwErrList[uwCt]==0)
        {
            puwErrList[uwCt]=uwParamCode;
            uwParamCode=0;
        }
//        RESTORE_IRQ();

            // if written then exit, otherwise try again
        if(uwParamCode==0)
        {
            atomic_move(&ulParChkLastUpdateTime, &ulSysTimersTotalPowerOnTime, sizeof(ulSysTimersTotalPowerOnTime));
            break;
        }
    }
}

//***************************************************************************
// Reset parameter value error

void ParChk_ResetValueError(UWORD uwParamCode)
{
    UWORD uwCt;

        // check if in the list
    for(uwCt=0;uwCt<ERR_LIST_SIZE;uwCt++)
        if(puwErrList[uwCt]==uwParamCode)
        {
            puwErrList[uwCt]=0;
            atomic_move(&ulParChkLastUpdateTime, &ulSysTimersTotalPowerOnTime, sizeof(ulSysTimersTotalPowerOnTime));
            break;
        }
}

//***************************************************************************
// Get error list and return list size

UWORD ParChk_GetValueErrorList(HPUWORD hpuwList, UWORD uwBufSize)
{
    UWORD uwCt,uwSize;

    for(uwCt=uwSize=0;uwCt<ERR_LIST_SIZE&&uwBufSize>0;uwCt++)
        if(puwErrList[uwCt])
        {
            *hpuwList++=puwErrList[uwCt];
            uwBufSize--;
            uwSize++;
        }

    return uwSize;
}

//***************************************************************************
// slow task

static void slowtask(void)
{
    if(bWarningActive)
    {
        ubCnt=2;
        bWarningActive=FALSE;
    }

    else if(ubCnt)
        if(--ubCnt==0)
        {
//            DISABLE_IRQ();
            if(!bWarningActive)
            {
                ulSystemWarnings&=~SYSTEMWARNINGS_INVALIDPARAMETERS;
                uwParChkParamCode=0;
            }
//            RESTORE_IRQ();
        }
}        

