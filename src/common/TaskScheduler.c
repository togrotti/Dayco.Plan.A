/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : TaskScheduler.c                                            */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Scheduler for internal tasks (init, background and         */
/*               realtime)                                                  */
/*                                                                          */
/****************************************************************************/
#include "common\CommonDefines.h"
#include "system\SystemStatus.h"

#define DEFINE_EXTERNALS
#include "TaskScheduler.h"
#undef DEFINE_EXTERNALS

#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonUtility.h"
//#include "FatalErrorCodes.h"
#include "drive\HardwareConfig.h"

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


//***************************************************************************
// Local prototypes;

static void TaskListNulls(void);

//***************************************************************************
// Defines

#define RT_AVG_CALC_NSAMPLES            1024

//***************************************************************************
// Locals

    // Background task list, ends with NULL
static void (* ppfBackgroundList[TASKSCHEDULER_BACKGROUND_MAX_ENTRIES])(void);

static UWORD uwRTAvgCalcTimer;
static UWORD uwRTAvgCalcPrevTimer;

//***************************************************************************
// Globals
UWORD uwTaskSchedStatus;

volatile UWORD uwTaskSchedFreeTimer;
UWORD uwTaskSchedRTMaxTime;
volatile UWORD uwTaskSchedRTLocalMaxTime;
volatile UWORD uwTaskSchedRTLocalAvgTime;

// Realtime task list, ends when pfTask member is NULL
TASKSCHEDULER_RT_SCHEDULER_ENTRY  tTaskSchedRTList[TASKSCHEDULER_RT_MAX_ENTRIES];

volatile ULONG ulTaskSchedRTLocalTimeSum;

//***************************************************************************
// Nulls all RT and Background task lists

static void TaskListNulls(void)
{
    UWORD i;

    for(i=0;i<TASKSCHEDULER_RT_MAX_ENTRIES;i++)
        tTaskSchedRTList[i].pfTask=NULL;

    for(i=0;i<TASKSCHEDULER_BACKGROUND_MAX_ENTRIES;i++)
        ppfBackgroundList[i]=NULL;
}

//***************************************************************************
// Init

BOOL TaskSched_Init(void)
{
    TaskListNulls();

    uwTaskSchedStatus=0;
    uwTaskSchedRTMaxTime=0;
    uwTaskSchedRTLocalMaxTime=0;
    uwRTAvgCalcPrevTimer=uwTaskSchedFreeTimer=0;
    uwRTAvgCalcTimer=timer_settimeout(uwTaskSchedFreeTimer,RT_AVG_CALC_NSAMPLES);

#ifndef _APP_XC
    if(sGlbControlBoardParameters.sProductInfo.uwProductRev<200)
        bTaskSchedRstButtonCheck=FALSE;
    else
        bTaskSchedRstButtonCheck=TRUE;
#endif

    return TRUE;
}

#ifdef _DEBUG_TRACES
	extern TASKSCHEDULER_TASK_INIT psSysAppTaskCollection;
	extern TASKSCHEDULER_TASK_INIT psSysAppTaskAltCollection;
	extern TASKSCHEDULER_TASK_INIT psSysAppTaskAltCollectWPars;
	extern TASKSCHEDULER_TASK_INIT psSysAppTaskAltCollectFullCfg;
#endif

//***************************************************************************
// Begin the initialization of all selected task

UWORD TaskSched_InitializeAll(TASKSCHEDULER_TASK_INIT * sTaskInitList)
{
    UWORD uwCnt=1;
    UWORD uwFail=0;

        // prevent task switching during initialization
    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL);

#ifdef _DEBUG_TRACES
	xil_printf("SysApp::TaskSched_InitializeAll(0x%08lx) : ", sTaskInitList);
	if (sTaskInitList==&psSysAppTaskCollection)
		xil_printf("psSysAppTaskCollection\r\n", sTaskInitList);
	else if (sTaskInitList==&psSysAppTaskAltCollection)
		xil_printf("psSysAppTaskAltCollection\r\n", sTaskInitList);
	else if (sTaskInitList==&psSysAppTaskAltCollectWPars)
		xil_printf("psSysAppTaskAltCollectWPars\r\n", sTaskInitList);
	else if (sTaskInitList==&psSysAppTaskAltCollectFullCfg)
		xil_printf("psSysAppTaskAltCollectFullCfg\r\n", sTaskInitList);
	else
		xil_printf("UNKNOWN\r\n", sTaskInitList);
#endif

	//
    while(sTaskInitList->pfTaskInit)
    {
#ifdef _DEBUG_TRACES
    	xil_printf("  TaskInit : %2u --> %s(%d)", uwCnt-1, sTaskInitList->szTaskName, sTaskInitList->uwTaskParam);
#endif
            // sequence number used for diagnostic
#ifndef _HW_DC
        LEDS_OUTPUT(uwCnt);
#else
        LEDS_OUTPUT(uwCnt&1?LEDS_OUT_YEL:0);
#endif

            // calls task init function with specified parameter
        if(!(*sTaskInitList->pfTaskInit)(sTaskInitList->uwTaskParam))
        {
#ifdef _DEBUG_TRACES
        	xil_printf(" : FAILURE\r\n");
#endif
                // if fail take first failing to be signaled at end
            if(uwFail==0)
                uwFail=uwCnt;
        }
#ifdef _DEBUG_TRACES
        else
        	xil_printf(" : SUCCESS\r\n");
#endif

            // select next one
        sTaskInitList++;
        uwCnt++;
    }

        // reset sequence number
    LEDS_OUTPUT(0);

    Os_EndCriticalSection(OS_CRITSECT_GLOBAL);

    return uwFail;
}

//***************************************************************************
// Add realtime task to scheduler

BOOL TaskSched_AddRTTask(BOOL (* pfTask)(void), UBYTE ubFlags, SYSTEMSTATUS uStatusSelect, SYSTEMSTATUS uStatusMask, UWORD uwNSlots)
{
    return TaskSched_AddRTTaskEx((BOOL (*)(ULONG))pfTask, ubFlags, uStatusSelect, uStatusMask, uwNSlots, 0);
}

//***************************************************************************
// Add realtime task to scheduler, extended

BOOL TaskSched_AddRTTaskEx(BOOL (* pfTask)(ULONG), UBYTE ubFlags, SYSTEMSTATUS uStatusSelect, SYSTEMSTATUS uStatusMask, UWORD uwNSlots, ULONG uwArg)
{
    UWORD i;

    for(i=0;i<TASKSCHEDULER_RT_MAX_ENTRIES;i++)
            // first empty slot, use it
        if(tTaskSchedRTList[i].pfTask==NULL)
        {
//            BOOL bPrevIen=PSW_IEN;
            
                // disable irq as insertion is done with realtime task running
//            PSW_IEN=0;
            tTaskSchedRTList[i].ubFlags=ubFlags;
            tTaskSchedRTList[i].uStatusSelect=uStatusSelect;
            tTaskSchedRTList[i].uStatusMask=uStatusMask;
            tTaskSchedRTList[i].uwNSlots=uwNSlots;
            tTaskSchedRTList[i].uwSlotCnt=1;
            tTaskSchedRTList[i].pfTask=pfTask;
            tTaskSchedRTList[i].uwArg=uwArg;

                // restore previous irq enable status
//            PSW_IEN=bPrevIen;

            return TRUE;
        }

        // no empty slot found, fail
    return FALSE;
}
    
//***************************************************************************
// Add background task to scheduler

BOOL TaskSched_AddBackgroundTask(void (* pfTask)(void))
{
    UWORD i;

    for(i=0;i<TASKSCHEDULER_BACKGROUND_MAX_ENTRIES;i++)
            // first empty slot, use it
        if(ppfBackgroundList[i]==NULL)
        {
            ppfBackgroundList[i]=pfTask;

            return TRUE;
        }

        // no empty slot found, fail
    return FALSE;
}

//***************************************************************************
// Background task scheduler

void TaskSched_BackgroundScheduler(void)
{
    void (** ppfList)(void);
    ULONG ulSum;
    UWORD uwNSample;

    for(;;)
    {
    	ppfList = ppfBackgroundList;

		while(*ppfList)
			(*ppfList++)();

			// rt task timing measurements
		if(uwTaskSchedRTLocalMaxTime>uwTaskSchedRTMaxTime)
			uwTaskSchedRTMaxTime=uwTaskSchedRTLocalMaxTime;

		if(timer_istimedout(uwTaskSchedFreeTimer,uwRTAvgCalcTimer))
		{
	//        DISABLE_IRQ();
			ulSum=ulTaskSchedRTLocalTimeSum;
			uwNSample=uwTaskSchedFreeTimer;
			ulTaskSchedRTLocalTimeSum=0l;
	//        RESTORE_IRQ();

			uwTaskSchedRTLocalAvgTime=(UWORD)(ulSum/((UWORD)(uwNSample-uwRTAvgCalcPrevTimer)));
			uwRTAvgCalcPrevTimer=uwNSample;

			uwRTAvgCalcTimer=timer_settimeout(uwTaskSchedFreeTimer,RT_AVG_CALC_NSAMPLES);
		}

		vTaskDelay(0);
    }

}
