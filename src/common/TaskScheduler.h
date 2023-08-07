/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : TaskScheduler.h                                            */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Common definitions for internal tasks (init, background    */
/*               and realtime) scheduling                                   */
/*                                                                          */
/****************************************************************************/

#ifndef _TASKSCHEDULER_H
#define _TASKSCHEDULER_H

#include "DefineExternals.h"
#include "system\SystemStatus.h"
//***************************************************************************
// Tasks flags

    // always execute
#define TASKSCHEDULER_FLAG_NONE                 0X00

    // immediately reiterate the task if return TRUE
#define TASKSCHEDULER_FLAG_REITERATEIFREQ       0x01
    // optional if no time left in realtime slot
#define TASKSCHEDULER_FLAG_OPTIONAL             0X02
    // execute in the odd phase of realtime slot
#define TASKSCHEDULER_FLAG_EXECUTEODD           0X04
    // execute in the even phase of realtime slot
#define TASKSCHEDULER_FLAG_EXECUTEEVEN          0X08
    // execute every # realtime slot
#define TASKSCHEDULER_FLAG_EXEEVERYNSLOTS       0X10

//***************************************************************************
// Configuration

#define TASKSCHEDULER_RT_MAX_ENTRIES            32
#define TASKSCHEDULER_BACKGROUND_MAX_ENTRIES    64

#define TASKSCHEDULER_RT_STD_EXECUTION_TIME     1100        // * 100nsec
#define TASKSCHEDULER_RT_MAX_EXECUTION_TIME     1200        // * 100nsec
#define TASKSCHEDULER_RT_PEAK_EXECUTION_TIME    1240        // * 100nsec
#define TASKSCHEDULER_RT_MAX_ALLOWED_SLOTS      4

//***************************************************************************
// Structures

    // Initialization task list structure
typedef struct
{
    BOOL            (* pfTaskInit)( ULONG );
    ULONG           uwTaskParam;
#ifdef _DEBUG_TRACES
    const char * szTaskName;
#endif
} TASKSCHEDULER_TASK_INIT;

    // Realtime scheduler structure
typedef struct
{
    BOOL            (* pfTask)( ULONG );        // task to be executed
    UBYTE           ubFlags;                    // operative flags
    SYSTEMSTATUS    uStatusSelect;              // selection and mask to
    SYSTEMSTATUS    uStatusMask;                // check for execution
    UWORD           uwNSlots;                   // exe every # slots (realtime)
    UWORD           uwSlotCnt;                  // slot downcounter
    ULONG           uwArg;                      // task argument
    UWORD           uwExeTime;                  // task execute time
} TASKSCHEDULER_RT_SCHEDULER_ENTRY;

//***************************************************************************
// Globals

#ifdef _INFINEON_
EXTERN UWORD bdata uwTaskSchedStatus;

DEFINE_BIT(bTaskSchedRTExecutingOddPhase,               uwTaskSchedStatus, 0);
DEFINE_BIT(bTaskSchedRealTimeRunning,                   uwTaskSchedStatus, 1);
#ifndef _APP_XC
DEFINE_BIT(bTaskSchedRstButtonCheck,                    uwTaskSchedStatus, 2);
#else
#define bTaskSchedRstButtonCheck (TRUE)
#endif // _app_xc
#else
extern UWORD uwTaskSchedStatus;

#define bTaskSchedRTExecutingOddPhase               HBITW(uwTaskSchedStatus)->bit0
#define bTaskSchedRealTimeRunning                   HBITW(uwTaskSchedStatus)->bit1

#ifndef _APP_XC
#define bTaskSchedRstButtonCheck                    HBITW(uwTaskSchedStatus)->bit2
#else
#define bTaskSchedRstButtonCheck (TRUE)
#endif // _app_xc
#endif // _infineon_

extern volatile UWORD uwTaskSchedFreeTimer;
extern UWORD uwTaskSchedRTMaxTime;
extern volatile UWORD uwTaskSchedRTLocalMaxTime;
extern volatile UWORD uwTaskSchedRTLocalAvgTime;

    // Realtime task list, ends when pfTask member is NULL
extern TASKSCHEDULER_RT_SCHEDULER_ENTRY  tTaskSchedRTList[TASKSCHEDULER_RT_MAX_ENTRIES];

extern volatile ULONG ulTaskSchedRTLocalTimeSum;

//***************************************************************************
// Scheduler Initialization

// Init
BOOL TaskSched_Init(void);

// Begin the initialization of all selected task
UWORD TaskSched_InitializeAll(TASKSCHEDULER_TASK_INIT *);

// Add realtime task to scheduler
BOOL TaskSched_AddRTTask(BOOL (*)(void), UBYTE, SYSTEMSTATUS, SYSTEMSTATUS, UWORD);

// Add realtime task to scheduler, extended
BOOL TaskSched_AddRTTaskEx(BOOL (*)(ULONG), UBYTE, SYSTEMSTATUS, SYSTEMSTATUS, UWORD, ULONG);

// Add background task to scheduler
BOOL TaskSched_AddBackgroundTask(void (*)(void));

//***************************************************************************
// Scheduler RunTime

// Realtime task scheduler
void TaskSched_RTScheduler(void);

// Background task scheduler
void TaskSched_BackgroundScheduler(void);

#endif
