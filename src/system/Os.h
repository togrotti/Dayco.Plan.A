/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Os.h                                                       */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : RT operating system for AxX projects                       */
/*                                                                          */
/****************************************************************************/

#ifndef _OS_H
#define _OS_H

#include "common\CommonDefines.h"

#ifndef _RD
#include "system\SysAppConfig.h"
#else
#include "RemDispAppConfig.h"
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//***************************************************************************
// Data structures

typedef UWORD OS_TASKHANDLE;

#ifdef _INFINEON_
typedef struct
{
    UBYTE ubStatus;
} OS_MUTEX;
#else
typedef SemaphoreHandle_t OS_MUTEX;
#endif

typedef OS_MUTEX OS_SYNCPOINT;

typedef struct
{
    ULONG ulEntry;
} OS_QUEUEELEMENT;

#ifdef _INFINEON_
typedef struct
{
    UWORD uwIn,uwOut;
    UWORD uwSize;
    OS_QUEUEELEMENT volatile * pElements;
    OS_SYNCPOINT * psSyncPoint;
} OS_QUEUE;
#else
typedef QueueHandle_t OS_QUEUE;
#endif

//***************************************************************************
// Macros

#ifdef _INFINEON_
#define OS_CREATEMUTEX(mutexname)                   volatile OS_MUTEX mutexname={1}

#define OS_CREATESYNCPOINT(syncpointname)           OS_CREATEMUTEX(syncpointname)

#define OS_CREATEPUREQUEUE(queuename,queuesize)     volatile OS_QUEUEELEMENT _##queuename##_elements[queuesize]; \
                                                    volatile OS_QUEUE queuename={0,0,queuesize,_##queuename##_elements,NULL}

#define OS_CREATEWAITINGQUEUE(queuename,queuesize)  volatile OS_QUEUEELEMENT _##queuename##_elements[queuesize]; \
                                                    volatile OS_MUTEX _##queuename##_syncpoint={1}; \
                                                    volatile OS_QUEUE queuename={0,0,queuesize,_##queuename##_elements,&_##queuename##_syncpoint}

#define OS_SETDEFAULT_ALU()                         { MCW_MP=0; MCW_MS=0; }
#else
#define OS_CREATEMUTEX(mutexname)                             mutexname=xSemaphoreCreateMutex();
#define OS_CREATEPUREQUEUE(queuename,queuesize,itemsize)      queuename=xQueueCreate(queuesize,itemsize)
#endif

//***************************************************************************
// Queue status

#define OS_QUEUESTATUS_ERROR                    (-1)

#define OS_QUEUESTATUS_VALID                    1
#define OS_QUEUESTATUS_EMPTY                    2

//***************************************************************************
// Mutex wait exit status

#define OS_MUTEXWAIT_ERROR                      (-1)

#define OS_MUTEXWAIT_SIGNALED                   1
#define OS_MUTEXWAIT_WAITING                    2
#define OS_MUTEXWAIT_TIMEOUT                    3

//***************************************************************************
// Task status

#define OS_TASKSTATUS_ERROR                     (-1)

#define OS_TASKSTATUS_INVALID                   0
#define OS_TASKSTATUS_STOPPED                   1
#define OS_TASKSTATUS_WAITINGFORCREATE          2
#define OS_TASKSTATUS_RUNNING                   4
#define OS_TASKSTATUS_SLEEPING                  5

//***************************************************************************
// Critical sections type

    // prevent task switching and event notification
#define OS_CRITSECT_SWITCHANDEVENT              0

    // prevent only task switching (global kept for backwards compatibility)
#define OS_CRITSECT_TASKSWITCH                  1
#define OS_CRITSECT_GLOBAL                      (OS_CRITSECT_TASKSWITCH)

//***************************************************************************
// Globals

extern volatile UWORD uwOsFreeRunTimer1kHz;
extern volatile ULONG ulOsTimer1Hz;

#if (OS_MEASURESTACKSIZE)
    // Stack measurement status
extern UWORD uwOsSysStackFree;
extern UWORD uwOsUsrStackFree;

extern UWORD uwOsMinStackFreePerTask[OS_MAXTASKSALLOWED];
#endif

//***************************************************************************
// Global functions

// Os initialization, to be called before any other Os function
BOOL Os_Init(void);

// Sleep (msec)
void Os_Sleep(UWORD);

//***************************************************************************
// Tasks management

// Task creation and startup, return handle of created task or NULL if error
OS_TASKHANDLE Os_TaskCreate(void (*)(void));

// As above plus user stack size selection
OS_TASKHANDLE Os_TaskCreateEx(void (*pTaskPointer)(void), UWORD uwResUsrStack, const char * const pTaskName);

void vApplicationTickHook( void );

// Task resuming and suspending
//BOOL Os_TaskResume(OS_TASKHANDLE);
BOOL Os_TaskSuspend(OS_TASKHANDLE);

// Task status
//SWORD Os_TaskStatus(OS_TASKHANDLE);

// Event notification
BOOL Os_TaskEventNotify(OS_TASKHANDLE TaskHandle);
void Os_TaskEventNotifyWait(void);

//***************************************************************************
// Mutex management

// Wait for mutex signaling or timeout, if timeout is zero then wait forever
SWORD Os_MutexWait(OS_MUTEX *, UWORD);

// Signal mutex
BOOL Os_MutexSignal(OS_MUTEX *);

//// Get status of the mutex, return immediately
//SWORD Os_MutexQuery(OS_MUTEX *);
//
////***************************************************************************
//// Syncpoint management
//
//// Wait for syncpoint signaling or timeout, if timeout is zero then wait forever
//SWORD Os_SyncPointWait(OS_SYNCPOINT *, UWORD);
//
//// Signal syncpoint
//BOOL Os_SyncPointSignal(OS_SYNCPOINT *);
//
//***************************************************************************
// Queue management

// Post a message on the queue
BOOL Os_QueuePost(OS_QUEUE , HPULONG );

// Get a message from the queue, if queue empty wait timeout milliseconds
SWORD Os_QueueGet(OS_QUEUE , HPULONG , UWORD);

// Get status of the queue, return immediately
SWORD Os_QueueStatus(OS_QUEUE);

// Flush all messages on the queue
BOOL Os_QueueFlush(OS_QUEUE *);

//***************************************************************************
// Critical sections

// Begin code section
void Os_BeginCriticalSection(UWORD);

// End critical section
void Os_EndCriticalSection(UWORD);

// Check current runnning level
BOOL Os_IsInBackground(void);

#endif
