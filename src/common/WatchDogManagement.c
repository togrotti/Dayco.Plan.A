/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : WatchDogManagement.c                                       */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Infineon WDT module and FPGA WDT management                */
/*                                                                          */
/****************************************************************************/

#include "system\SysAppGlobals.h"
#include "common\TaskScheduler.h"
#include "fpga\FpgaHandler.h"
#include "WatchDogManagement.h"
#include "WatchDogManagementRT.h"
#include "system\SystemAlarms.h"
#include "system\SysLogManagement.h"

//***************************************************************************
// Defines

#define FPGA_WDT_SERVICE_PERIOD         100             // msec

//***************************************************************************
// Locals

static BOOL bWdtStarted=FALSE;
static BOOL bWdtSlowFailed=FALSE;

//***************************************************************************
// Local functions

static void slowtask(void);

//***************************************************************************
// Initialization entry point

BOOL WdtMgm_Init(UWORD uwOption)
{
    switch(uwOption)
    {
        case 0: // install first wdt into fast task
            return TaskSched_AddRTTask(&WdtMgmRTTask, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_LOCKEDBYPLCRELOAD), 0);
            
        case 1: // install second wdt into fast task
            return TaskSched_AddRTTask(&WdtMgmRTTask, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_LOCKEDBYPLCRELOAD), 0);
            
        default:
            return FALSE;
    }    
}

//***************************************************************************
// Startup entry point

BOOL WdtMgm_Start(void)
{
        // add tasks
    return TaskSched_AddBackgroundTask(&slowtask);
}

//***************************************************************************
// slow task

static void slowtask(void)
{
        // while booting do nothing
    if(bSysStatBooting)
        return;

        // first run, startup wdt, unlock fpga and start fast task
    if(!bWdtStarted)
    {
        FpgaSafeUnLock(FPGA_WDT_SERVICE_PERIOD);
        bWdtStarted=TRUE;
        bWdtFastEnabled=TRUE;
    }
    else if(!bWdtSlowFailed && !bWdtFastFailed)
    {
            // if CRC fail post alarm and disable fast task
        if(!FpgaCRCService())
        {
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_SOFTWARE_FAIL, SYSTEMALARMS_SUBCODE_SF_TM_FPGA_CRC_FAIL, TRUE);
            bWdtSlowFailed=TRUE;
            bWdtFastEnabled=FALSE;
        }
    }
}

