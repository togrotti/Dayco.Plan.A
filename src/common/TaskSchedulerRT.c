/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : TaskSchedulerRT.c                                          */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Scheduler for internal tasks (init, background and         */
/*               realtime)                                                  */
/*                                                                          */
/****************************************************************************/

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "system\SystemStatus.h"
#include "TaskScheduler.h"
#include "system\SysAppGlobals.h"
#include "system\SysAppConfig.h"
#include "plc\Plc.h"
#include "core\SystemReset.h"
#include "system\GlobalResetCodes.h"
#include "drive\AxM-E-Defines.h"
#include "system\SysLogManagement.h"
    
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

//***************************************************************************
// Locals

static UWORD uwTaskSchedLocRstButCnt=0;

//***************************************************************************
// Realtime task scheduler; if defined TASKSCHED_REALTIME_IV the scheduler
// will be invoked as interrupt

#ifdef TASKSCHED_REALTIME_IV
//#pragma savemac
void TaskSched_RTScheduler(void) interrupt TASKSCHED_REALTIME_IV
#else
void TaskSched_RTScheduler(void)
#endif
{
    static UWORD uwOverTimeSlotCnt=0;
    static BOOL bTempDisableOverTimeCheck=FALSE;
    TASKSCHEDULER_RT_SCHEDULER_ENTRY * psList=tTaskSchedRTList;
    UWORD uwProfiler;
    UWORD uwTaskTime;

        // restore default MAC settings as SAVEMAC does not
//    OS_SETDEFAULT_ALU();

    for(UWORD i=0;i<32;i++)
    {
    	sSystemStatus.b[i]  = (ulSystemStatus&(1<<i))==(1<<i); //sSystemStatus.l[i] = (ulSystemStatus&(1<<i))==(1<<i);
    	sSystemStatus2.b[i] = (ulSystemStatus2&(1<<i))==(1<<i);
    }

    uwProfiler=timer_profiler_start(uwSysTimers100ns);
    bTaskSchedRealTimeRunning=TRUE;
    uwTaskSchedFreeTimer++;
    bTaskSchedRTExecutingOddPhase=!bTaskSchedRTExecutingOddPhase;

    while(psList->pfTask)
    {
        uwTaskTime=timer_profiler_start(uwSysTimers100ns);
        if(psList->ubFlags==TASKSCHEDULER_FLAG_NONE)
        {
            if(psList->uStatusSelect == (psList->uStatusMask&ulSystemStatus))
                (*psList->pfTask)(psList->uwArg);
        }
        else
        {
            UBYTE ubFlags=psList->ubFlags;

            if(ubFlags&TASKSCHEDULER_FLAG_EXEEVERYNSLOTS)
            {
                if(--psList->uwSlotCnt)
                    continue;
                else
                    psList->uwSlotCnt=psList->uwNSlots;
            }

            if(psList->uStatusSelect == (psList->uStatusMask&ulSystemStatus))
            {
                if(((ubFlags&TASKSCHEDULER_FLAG_EXECUTEODD) && !bTaskSchedRTExecutingOddPhase) || \
                   ((ubFlags&TASKSCHEDULER_FLAG_EXECUTEEVEN) && bTaskSchedRTExecutingOddPhase))
                    continue;

                if(ubFlags&TASKSCHEDULER_FLAG_REITERATEIFREQ)
                {
                    do
                    {
                        if((ubFlags&TASKSCHEDULER_FLAG_OPTIONAL) && (timer_100nscorrect(timer_profiler_end(uwSysTimers100ns,uwProfiler))>=TASKSCHEDULER_RT_STD_EXECUTION_TIME))
                            continue;
                    }
                    while((*psList->pfTask)(psList->uwArg));
                }

                else
                    if(!(ubFlags&TASKSCHEDULER_FLAG_OPTIONAL) || (timer_100nscorrect(timer_profiler_end(uwSysTimers100ns,uwProfiler))<TASKSCHEDULER_RT_STD_EXECUTION_TIME))
                        (*psList->pfTask)(psList->uwArg);
            }
        }
        psList->uwExeTime = timer_100nscorrect(timer_profiler_end(uwSysTimers100ns,uwTaskTime));

        psList++;
    }

// SAIETTA don't have RESET button input, update on REV101
#ifndef _HW_AXS
        // reset button check and debounce
    if(bTaskSchedRstButtonCheck)
    {
        if(XE167_RESET_BUTTON)
        {
            if(++uwTaskSchedLocRstButCnt>=(UWORD)((ULONG)PF_RSTBUTPERIOD/(1000000/REALTIME_TASK_FREQ)))
#ifdef _INFINEON_
                SysRes_ExecuteReset(RESETCODE_POFSTOREDATA, ~RESETCODE_POFSTOREDATA);
#else
                SysLogMgm_PowerOnFail();
#endif
        }
        else
                uwTaskSchedLocRstButCnt=0;
    }
#endif

        // power fail check: copy from TrapHandler
// check SAIETTA ZQ_PFAIL before using POWER_FAIL_IN
#ifndef _HW_AXS
    if ( !POWER_FAIL_IN && ((ulSystemWarnings&SYSTEMWARNINGS_STARTWITHPOWERFAIL)==0) )
#ifdef _INFINEON_
        SysRes_ExecuteReset(RESETCODE_POFSTOREDATA, ~RESETCODE_POFSTOREDATA);
#else
        SysLogMgm_PowerOnFail();
#endif
#endif

        // profiling
    uwProfiler=timer_100nscorrect(timer_profiler_end(uwSysTimers100ns,uwProfiler));
    if(uwProfiler>uwTaskSchedRTLocalMaxTime)
        uwTaskSchedRTLocalMaxTime=uwProfiler;
    ulTaskSchedRTLocalTimeSum+=(ULONG)uwProfiler;

    bTaskSchedRealTimeRunning=FALSE;

        // recovery from plc overtime alarm
    if(bTempDisableOverTimeCheck)
    {
        bTempDisableOverTimeCheck=FALSE;
        return;
    }

        // overtime checking if internal status is fully active
    if((ulSystemStatus&(SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_RESETTING)|
        SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PROGRAMFLASHWRITING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_LOCKEDBYBOOTERROR)|
        SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_DISABLEOVERTIMECHECK)))==0ul)
    {
            // if great or eq than peak allowed time or new irq req for this task or overtime slow counter overflow
            // then immediate overtime is spawn
        if(uwProfiler>=TASKSCHEDULER_RT_PEAK_EXECUTION_TIME || TASKSCHED_REALTIME_IRFLAG || uwOverTimeSlotCnt>TASKSCHEDULER_RT_MAX_ALLOWED_SLOTS)
        {
                // if PLC enabled then call PLC handler
            if(bSysStatPlcRunning)
            {
                PlcRTOvertime();
                    // temporary disable check at next cycle
                bTempDisableOverTimeCheck=TRUE;
                    // and reset overtime slow counter overflow
                uwOverTimeSlotCnt=0;
            }
                // otherwise overtime is only due to system, then fatal
            else
            {
#if (defined(_DEBUG_TRACES))
    	      xil_printf("TaskSchedulerRT::TaskSched_RTScheduler() : FATAL_ERROR_RT_OVERTIME\r\n");
#endif
              //ShowFatalErrorAndHalt(FATAL_ERROR_RT_OVERTIME);
    	      ;
            }
        }
            // if more than max time then increment overtime slow counter
        else if(uwProfiler>TASKSCHEDULER_RT_MAX_EXECUTION_TIME)
            uwOverTimeSlotCnt++;
            // otherwise decrement
        else if(uwOverTimeSlotCnt>0)
            uwOverTimeSlotCnt--;
    }
}
