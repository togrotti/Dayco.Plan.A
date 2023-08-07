/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Os.c                                                       */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : RT operating system for AxX projects                       */
/*                                                                          */
/****************************************************************************/

// Compiler Option
#pragma GCC optimize (2)

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "Os.h"
#include "drive\AxM-E-Defines.h"
#include "system\SysAppGlobals.h"

//***************************************************************************
// Task structure definition

typedef struct
{
        // Status: status changing is only allowed when STOPPED, RUNNING
        // or SLEEPING; otherwise task switch handler is doing something,
        // then changing is not possible
    UBYTE ubStatus;

        // Entry point address
    void (* pEntryPoint)(void);

    TaskHandle_t xTaskHandler;
} TASK;

//***************************************************************************
// Globals

volatile UWORD uwOsFreeRunTimer1kHz;
volatile ULONG ulOsTimer1Hz;            // externally initialized as used
                                        // as life time clock

#if (OS_MEASURESTACKSIZE)
    // Stack measurement status
UWORD uwOsSysStackFree;
UWORD uwOsUsrStackFree;

UWORD uwOsMinStackFreePerTask[OS_MAXTASKSALLOWED];
#endif

//***************************************************************************
// Local defines

//***************************************************************************
// Locals

#define ISTASKHANDLERVALID(handle)          ((handle)>0 && (handle)<=OS_MAXTASKSALLOWED)

// Tasks definition array
static volatile TASK sTasks[OS_MAXTASKSALLOWED];

//***************************************************************************
// Os initialization, to be called before any other Os function

BOOL Os_Init(void)
{

    return TRUE;
}

//***************************************************************************
// Task creation and startup, return handle of created task or NULL if error

OS_TASKHANDLE Os_TaskCreate(void (*pTaskPointer)(void))
{
    return Os_TaskCreateEx(pTaskPointer,OS_DEFAULTUSRSTATICSTACK,NULL);
}

//***************************************************************************
// As above plus user stack size selection

OS_TASKHANDLE Os_TaskCreateEx(void (*pTaskPointer)(void), UWORD uwResUsrStack, const char * const pTaskName)
{
    UWORD uwHandle;

        // seek an invalid (empty) task slot
    for(uwHandle=0;uwHandle<OS_MAXTASKSALLOWED;uwHandle++)
        if(sTasks[uwHandle].ubStatus==OS_TASKSTATUS_INVALID)
            if(xTaskCreate((TaskFunction_t)pTaskPointer, pTaskName, uwResUsrStack, NULL, tskIDLE_PRIORITY, &sTasks[uwHandle].xTaskHandler))
            {
                    // setup parameters
                sTasks[uwHandle].pEntryPoint=pTaskPointer;

                    // and mark it for creation
                sTasks[uwHandle].ubStatus=OS_TASKSTATUS_RUNNING;

                    // return nonzero handle
                return (OS_TASKHANDLE)uwHandle;
            }


    return NULL;
}

void vApplicationTickHook( void )
{
    static int count=0;

    if(count == configTICK_RATE_HZ-1)
    {
        count = 0;
        ulOsTimer1Hz++;
    }
    else
        count++;
    
    uwOsFreeRunTimer1kHz++;
}

//***************************************************************************
// Task resume, just if marked stopped mark it as run

BOOL Os_TaskResume(OS_TASKHANDLE hTask)
{
    if(!ISTASKHANDLERVALID(hTask))
        return FALSE;

#ifdef _INFINEON_
    hTask--;

        // if stopped mark it as running
    AtomicTestAndSetTaskStatus(hTask, OS_TASKSTATUS_STOPPED, OS_TASKSTATUS_RUNNING);
#else
    vTaskResume(sTasks[hTask].xTaskHandler);
#endif
    return TRUE;
}

//***************************************************************************
// Task suspend, just if marked stopped or sleeping mark it as run

BOOL Os_TaskSuspend(OS_TASKHANDLE hTask)
{
    if(!ISTASKHANDLERVALID(hTask))
        return FALSE;

#ifdef _INFINEON_
    hTask--;
        // if running mark it as stopped
    AtomicTestAndSetTaskStatus(hTask, OS_TASKSTATUS_RUNNING, OS_TASKSTATUS_STOPPED);
        // or if sleeping
    AtomicTestAndSetTaskStatus(hTask, OS_TASKSTATUS_SLEEPING, OS_TASKSTATUS_STOPPED);
#else
    vTaskSuspend(sTasks[hTask].xTaskHandler);
#endif

   return TRUE;
}

////***************************************************************************
//// Task suspend, just if marked stopped or sleeping mark it as run
//
//SWORD Os_TaskStatus(OS_TASKHANDLE hTask)
//{
//    UBYTE ubStatus;
//
//    if(!ISTASKHANDLERVALID(hTask))
//        return OS_TASKSTATUS_ERROR;
//    hTask--;
//
//    ubStatus=sTasks[hTask].ubStatus;
//
//    if(ubStatus==OS_TASKSTATUS_FILLING)
//        return OS_TASKSTATUS_WAITINGFORCREATE;
//
//    else
//        return (SWORD)ubStatus;
//}
//
//***************************************************************************
// Task event notification, just push into the queue and force task switcher

BOOL Os_TaskEventNotify(OS_TASKHANDLE TaskHandle)
{
#ifdef _INFINEON_
   if(Os_QueuePost(&_Os_EventQueue, (ULONG)pEvent))
   {
    //    FORCETASKSWITCHHANDLER();

       return TRUE;
   }
   else
       return FALSE;
#else
   BaseType_t xHightPriority=pdTRUE;

   vTaskNotifyGiveFromISR( sTasks[(UWORD)TaskHandle].xTaskHandler, &xHightPriority );

   return TRUE;
#endif
}

//***************************************************************************
// Wait for task event notification

void Os_TaskEventNotifyWait(void)
{
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
}

//***************************************************************************
// Sleep current task for msec, if zero just yield to other task

void Os_Sleep(UWORD uwMilliSeconds)
{
#ifdef _INFINEON_
        // if called from an ISR then do nothing
    if(ISINTOISR())
        return;

    if(uwMilliSeconds>0)
    {
            // if time>0 then setup for task sleeping
        sTasks[uwRunningTask].uwDelayCounter=uwMilliSeconds;
        sTasks[uwRunningTask].psWaitingObject=NULL;

        AtomicTestAndSetTaskStatus(uwRunningTask, OS_TASKSTATUS_RUNNING, OS_TASKSTATUS_SLEEPING);

            // then trigger task switch handler
        FORCETASKSWITCHING();

            // if return and still in sleeping then wait until timer expires, as
            // OS has no other task to run (e.g. only one task or all other tasks sleeping)
        while(sTasks[uwRunningTask].ubStatus==OS_TASKSTATUS_SLEEPING);
    }

        // just trigger switch to next task
    else
        FORCETASKSWITCHING();
#else
    vTaskDelay(0);
#endif
}

//***************************************************************************
// Wait for mutex signaling or timeout, if timeout is zero then wait forever

SWORD Os_MutexWait(OS_MUTEX * psMutex, UWORD uwMilliSeconds)
{
#ifdef _INFINEON_
        // atomic test and set, if 1 (signal) set it to 0 (wait)
    DISABLE_IRQ();
    if(psMutex->ubStatus)
    {
        psMutex->ubStatus=0;
        RESTORE_IRQ();

        return OS_MUTEXWAIT_SIGNALED;
    }
    else
    {
        RESTORE_IRQ();

            // if OS is not running exit immediately to avoid system frozen forever
        if(ubGlobalStatus!=STAT_RUNNING)
            return OS_MUTEXWAIT_TIMEOUT;

            // if is called from an ISR it cannot wait here, otherwise if
            // the task that has to release resource has lower priority, the
            // system will freeze forever
        if(ISINTOISR())
            return OS_MUTEXWAIT_TIMEOUT;

            // if timeout specified
        if(uwMilliSeconds>0)
        {
                // set task for sleeping with conditionally wakeup from the mutex
            sTasks[uwRunningTask].uwDelayCounter=uwMilliSeconds;
            sTasks[uwRunningTask].psWaitingObject=psMutex;

            AtomicTestAndSetTaskStatus(uwRunningTask, OS_TASKSTATUS_RUNNING, OS_TASKSTATUS_SLEEPING);

                // then trigger task switch handler
            FORCETASKSWITCHING();

                // if return and still in sleeping then wait until timer expires, as
                // OS has no other task to run (e.g. only one task or all other tasks sleeping)
            while(sTasks[uwRunningTask].ubStatus==OS_TASKSTATUS_SLEEPING);

                // atomic test and set mutex
            DISABLE_IRQ();
            if(psMutex->ubStatus)
            {
                psMutex->ubStatus=0;
                RESTORE_IRQ();
                return OS_MUTEXWAIT_SIGNALED;
            }
            else
            {
                    // cannot set it to wait, timeout is elapsed then return
                RESTORE_IRQ();
                return OS_MUTEXWAIT_TIMEOUT;
            }
        }

            // if no timeout wait forever
        else
            for(;;)
            {
                    // trigger task switch handler
                FORCETASKSWITCHING();

                    // then atomic test and set
                DISABLE_IRQ();
                if(psMutex->ubStatus)
                {
                    psMutex->ubStatus=0;
                    RESTORE_IRQ();
                    return OS_MUTEXWAIT_SIGNALED;
                }
                RESTORE_IRQ();
            }
    }
#else
    TickType_t xTicksToWait = uwMilliSeconds==0 ? portMAX_DELAY : uwMilliSeconds;

    if(xSemaphoreTake(*psMutex, xTicksToWait))
        return OS_MUTEXWAIT_SIGNALED;
    else
        return OS_MUTEXWAIT_TIMEOUT;
#endif
}

//***************************************************************************
// Signal mutex

BOOL Os_MutexSignal(OS_MUTEX * psMutex)
{
#ifdef _INFINEON_
    UWORD uwHandle;
    volatile TASK * psTask;

        // release mutex by setting it to 1
    psMutex->ubStatus=1;

        // if OS is not running exit immediately as no other task to be awaken
        // are available
    if(ubGlobalStatus!=STAT_RUNNING)
        return TRUE;

        // check if there is waiting tasks for this mutex
    for(uwHandle=0,psTask=sTasks;uwHandle<OS_MAXTASKSALLOWED;uwHandle++,psTask++)
        if(psTask->ubStatus==OS_TASKSTATUS_SLEEPING && psTask->psWaitingObject==psMutex)
        {
                // if yes do an atomic (re)test and set on the object
            DISABLE_IRQ();

            if(psTask->ubStatus==OS_TASKSTATUS_SLEEPING && psTask->psWaitingObject==psMutex)
            {
                psTask->ubStatus=OS_TASKSTATUS_RUNNING;

                    // and force task switching, preventing at least same task to
                    // regain access to resource before the waiting one
                FORCETASKSWITCHING();
            }

            RESTORE_IRQ();

                // just resume the first one found, leave the others sleeping
            return TRUE;
        }
    
    return TRUE;
#else
    return (BOOL)xSemaphoreGive( *psMutex );
#endif
}

////***************************************************************************
//// Get status of the mutex, return immediately
//
//SWORD Os_MutexQuery(OS_MUTEX * psMutex)
//{
//    if(psMutex->ubStatus)
//        return OS_MUTEXWAIT_SIGNALED;
//    else
//        return OS_MUTEXWAIT_WAITING;
//}
//
////***************************************************************************
//// Wait for syncpoint signaling or timeout, if timeout is zero then wait forever
//
//SWORD Os_SyncPointWait(OS_SYNCPOINT * psSyncPoint, UWORD uwMilliSeconds)
//{
//        // just set mutex to wait state and call mutexwait
//    psSyncPoint->ubStatus=0;
//
//    return Os_MutexWait(psSyncPoint, uwMilliSeconds);
//}
//
////***************************************************************************
//// Signal syncpoint
//
//BOOL Os_SyncPointSignal(OS_SYNCPOINT * psSyncPoint)
//{
//        // just call to mutexsignal
//    return Os_MutexSignal(psSyncPoint);
//}
//
//***************************************************************************
// Post a message on the queue

BOOL Os_QueuePost(OS_QUEUE sQueue, HPULONG pulInMessage)
{
#ifdef _INFINEON_
    UWORD uwActInPos;
    UWORD uwNextPos;
    ULONG * pulEntry;

    for(;;)
    {
            // in order to stay less time as possibile in critical section,
            // it save actual In position, then calculate next In position
            // and entry pointer
        uwActInPos=psQueue->uwIn;
        uwNextPos=uwActInPos+1;
        if(uwNextPos>=psQueue->uwSize)
            uwNextPos=0;
        pulEntry=&psQueue->pElements[uwNextPos].ulEntry;

            // if next In position equal actual Out position, queue is full
        if(uwNextPos==psQueue->uwOut)
            return FALSE;

            // atomically check if actual In position is same as was at
            // begin of loop and if next In position still different than
            // actual Out position, if yes store entry and next In position
        DISABLE_IRQ();
        if(uwActInPos==psQueue->uwIn && uwNextPos!=psQueue->uwOut)
        {
            *pulEntry=ulInMessage;
            psQueue->uwIn=uwNextPos;
            RESTORE_IRQ();

                // signal new element (if there was another process waiting
                // and if is a waiting queue)
            if(psQueue->psSyncPoint)
                Os_SyncPointSignal(psQueue->psSyncPoint);
            return TRUE;
        }
            // otherwise retry to get an entry
        RESTORE_IRQ();
    }
#else
    return (BOOL)xQueueSendFromISR( sQueue, pulInMessage, 0);
#endif
}

//***************************************************************************
// Get a message from the queue, if queue empty wait timeout milliseconds

SWORD Os_QueueGet(OS_QUEUE sQueue, HPULONG pulOutMessage, UWORD uwMilliSeconds)
{
#ifdef _INFINEON_
    BOOL bFirstPass=TRUE;
    UWORD uwActOutPos;
    UWORD uwNextPos;
    ULONG * pulEntry;

    for(;;)
    {
            // in order to stay less time as possibile in critical section,
            // it save actual Out position, then calculate next Out position
            // and entry pointer
        uwActOutPos=psQueue->uwOut;
        uwNextPos=uwActOutPos+1;
        if(uwNextPos>=psQueue->uwSize)
            uwNextPos=0;
        pulEntry=&psQueue->pElements[uwNextPos].ulEntry;

            // if next Out position equal actual In position, queue is empty
        if(uwActOutPos==psQueue->uwIn)
        {
                // if it's not a waiting queue
            if(psQueue->psSyncPoint==NULL)
                return OS_QUEUESTATUS_EMPTY;

                // firstpass is to avoid that process goes to sleep, then a post
                // in the queue wakeup it, but, another waiting process get first
                // the new element, resulting in process that goes again sleeping
            if(bFirstPass)
            {
                if(Os_SyncPointWait(psQueue->psSyncPoint, uwMilliSeconds)!=OS_MUTEXWAIT_SIGNALED)
                    return OS_QUEUESTATUS_EMPTY;
                bFirstPass=FALSE;
            }
            else
                return OS_QUEUESTATUS_EMPTY;
        }

            // atomically check if actual Out position is same as was at
            // begin of loop and if actual Out position is still different than
            // actual In position, if yes get entry and store next Out position
        DISABLE_IRQ();
        if(uwActOutPos==psQueue->uwOut && uwActOutPos!=psQueue->uwIn)
        {
            *pulOutMessage=*pulEntry;
            psQueue->uwOut=uwNextPos;
            RESTORE_IRQ();
            return OS_QUEUESTATUS_VALID;
        }
            // otherwise retry to get an entry
        RESTORE_IRQ();
    }
    return (BOOL)xQueueReceive( sQueue, pulOutMessage, uwMilliSeconds==0 ? portMAX_DELAY : uwMilliSeconds);
#else
    return (BOOL)xQueueReceiveFromISR( sQueue, pulOutMessage, 0);
#endif
}

//***************************************************************************
// Get status of the queue, return immediately

SWORD Os_QueueStatus(OS_QUEUE Queue)
{
   if(uxQueueMessagesWaitingFromISR(Queue)==0)
       return OS_QUEUESTATUS_EMPTY;
   else
       return OS_QUEUESTATUS_VALID;
}

#ifdef _INFINEON_
//***************************************************************************
// Flush all messages on the queue

BOOL Os_QueueFlush(OS_QUEUE * psQueue)
{
    DISABLE_IRQ();
    psQueue->uwOut=psQueue->uwIn;
    RESTORE_IRQ();

    return TRUE;
}
#endif

//***************************************************************************
// Begin code section that will prevent task switching and event notification

void Os_BeginCriticalSection(UWORD uwType)
{
#ifdef _INFINEON_
    if(uwType==OS_CRITSECT_TASKSWITCH)
        __asm
        {
            atomic  #3
            mov     r2,uwTaskSwitchCritSectCounter
            add     r2,#1
            mov     uwTaskSwitchCritSectCounter,r2
        }
    else
        __asm
        {
            atomic  #3
            mov     r2,uwGlobalCritSectCounter
            add     r2,#1
            mov     uwGlobalCritSectCounter,r2
        }
#else
    vPortEnterCritical() ;
#endif
}

//***************************************************************************
// End critical section

void Os_EndCriticalSection(UWORD uwType)
{
#ifdef _INFINEON_
    if(uwType==OS_CRITSECT_TASKSWITCH)
        __asm
        {
            atomic  #4
            mov     r2,uwTaskSwitchCritSectCounter
            jmpr    cc_Z,nodec1
            sub     r2,#1
            mov     uwTaskSwitchCritSectCounter,r2
nodec1:
            nop
            nop
        }
    else
        __asm
        {
            atomic  #4
            mov     r2,uwGlobalCritSectCounter
            jmpr    cc_Z,nodec2
            sub     r2,#1
            mov     uwGlobalCritSectCounter,r2
nodec2:
            nop
            nop
        }
#else
    vPortExitCritical() ;
#endif
}

//***************************************************************************
// Check current runnning level

BOOL Os_IsInBackground(void)
{
#ifdef _INFINEON_
    return !ISINTOISR();
#else
    return TRUE; // temp
#endif
}

#ifdef _INFINEON_
//***************************************************************************
// Atomic test and set new task status, return TRUE if test was successful
// otherwise return FALSE and status is leaved as is

static BOOL AtomicTestAndSetTaskStatus(OS_TASKHANDLE hTask, UBYTE ubTest, UBYTE ubSet)
{
    UBYTE * pubStatus;
    SWORD usFilled;

        // prepare for asm atomic op
    pubStatus=&sTasks[hTask].ubStatus;
    usFilled=FALSE;

    __asm
    {
            // save r2 and r3
        push    r3
        push    r2

        mov     r3,ubTest
        movb    rl2,rl3

        mov     r3,ubSet
        movb    rh2,rl3

        mov     r3,pubStatus

            // atomically test and write
        atomic  #3
        cmpb    rl2,[r3]
        jmpr    cc_NZ,_skip
        movb    [r3],rh2
        mov     usFilled,#1     // TRUE

       _skip:
        nop
        nop

        pop     r2
        pop     r3
    }

    return usFilled;
}

//***************************************************************************
// Task suspend wrapper, the pointer is pushed into stack in order that
// this function is called whenever the task end (by return istruction or
// normal function termination)

static void TaskSuspendWrapper(void)
{
    for(;;)
    {
            // launch task
        (*(sTasks[uwRunningTask].pEntryPoint))();

            // if task terminate then it's suspended
        Os_TaskSuspend(uwRunningTask+1);
    }
}

//***************************************************************************
// Os clock tick interrupt handler

static void ClockTickHandler(void) interrupt GPT12E_T6IC_IV
{
    static UWORD uwTimerDivisor=0;
    UWORD uwCt;
    volatile TASK * psTask;

    for(uwCt=0,psTask=sTasks;uwCt<OS_MAXTASKSALLOWED;uwCt++,psTask++)
        if(psTask->ubStatus==OS_TASKSTATUS_SLEEPING)
            if(--psTask->uwDelayCounter==0)
                AtomicTestAndSetTaskStatus(uwCt, OS_TASKSTATUS_SLEEPING, OS_TASKSTATUS_RUNNING);

    if(uwLastSwitchCounter)
        uwLastSwitchCounter--;
    else if(bForceTaskSwitchHandler)
    {
        FORCETASKSWITCHHANDLER();
        bForceTaskSwitchHandler=FALSE;
    }

    if(uwTimerDivisor==0)
    {
        uwTimerDivisor=TIMERDIVISOR-1;
        ulOsTimer1Hz++;
    }
    else
        uwTimerDivisor--;

    uwOsFreeRunTimer1kHz++;
}

//***************************************************************************
// Task switcher interrupt handler wrapper

#pragma savesys
#pragma savemac
#pragma dynamicusrstk

static void TaskSwitcherHandlerWrapper(void) interrupt GPT12E_T5IC_IV
{
        // All locals are static in order to avoid the compiler to put
        // them in user and/or system stacks, simplifying stack switch
    static UWORD uwSysStackAbsAddress;
    static UWORD uwSysStackSegment;
    static UWORD uwCt;
    static UWORD uwNextTask;
    static volatile TASK * psRunTask;
    static volatile TASK * psNextTask;
    static UWORD uwSwapStack;
    static UWORD uwSysStackSize;
    static UWORD near * puwTopFrameStack;
    static UWORD near * puwBotFrameStack;
    static UWORD near * puwNewFrameStack;
    static UWORD uwDelta;
    static UWORD uwSize;
    static void (far * pTaskEntryPoint)(void);

        // The trick in order to force the compiler to save all register
        // in TaskSwitcherHandler is calling a function via
        // function pointer, then the compiler has to save all register as
        // it cannot know in advance what registers are used, as the called
        // function could virtually use any of the permitted registers;
        // this is done here for the event queue
    __asm
    {
            // Explicitly save those registers that are pushed on stack
            // only if and when used
        push    r13
        push    r14
        push    r15

            // Explicitly save DPPx registers and restore them from local copy
        push    DPP0
        push    DPP1
        push    DPP2
        push    DPP3

#if (OS_MEASURESTACKSIZE)
            // Check available sys stack
        mov     r3,#POF (uwOsSysStackBot)
        bclr    r3.15
        bclr    r3.14
        mov     DPP0,#PAG (uwOsSysStackBot)
        mov     r5,#TAGFORSTACKMEASUREMENT
        mov     r6,#-1

       _sysstackmeas:
        add     r6,#1
        cmp     r5,[r3+]
        jmpr    cc_Z,_sysstackmeas
#endif

        mov     r1,#POF (uwDefaultDPP)
        extp    #PAG (uwDefaultDPP),#04h
        mov     r2,[r1+]
        mov     r3,[r1+]
        mov     r4,[r1+]
        mov     r5,[r1+]

        mov     DPP0,r2
        mov     DPP1,r3
        mov     DPP2,r4
        mov     DPP3,r5

           // Get actual stacks addresses
        mov     r1,SP
        mov     uwSysStackAbsAddress,r1

#if (OS_MEASURESTACKSIZE)
           // Save in the right data page the measure of sys stack
        mov     uwSwapStack,r6
#endif
    }

#if (OS_MEASURESTACKSIZE)
        // If measure less than actual then update
    uwSwapStack<<=1;
    if(uwSwapStack<uwOsSysStackFree)
        uwOsSysStackFree=uwSwapStack;
#endif

    switch(ubGlobalStatus)
    {
            // Initializing section: fill the template registers array (see puwCleanRegsTemplate)
        case STAT_INITIALIZING:
                // Get sys stack segment
            __asm
            {
                mov     r1,SPSEG
                mov     uwSysStackSegment,r1
            }

                // correct start to point to first stacked value, as push first decrement SP then
                // write to [SP]
            uwCleanRegsStackStart-=0x02;
                // then remove the portion for the RETI istruction
            if(CPUCON1_SGTDIS())
                uwCleanRegsStackStart-=0x04;
            else
                uwCleanRegsStackStart-=0x06;

                // fill the array
            uwCleanRegsTemplateSize=0;
            for(uwCt=uwCleanRegsStackStart; uwCt>=uwSysStackAbsAddress; uwCt-=0x02)
                puwCleanRegsTemplate[uwCleanRegsTemplateSize++]=HVAR(UWORD, (ULONG)uwSysStackSegment<<16 | uwCt );

                // signal end of operation disabling all task switching handler tasks
            ubGlobalStatus=STAT_NOOPERATION;
            break;

            // Runtime section
        case STAT_RUNNING:
                // check for global critical section, if in it then exit immediately
            if(uwGlobalCritSectCounter>0)
            {
                bForceTaskSwitchHandler=TRUE;
                break;
            }

                // check for pending events, if any then dispatch immediately
            while(Os_QueueStatus(&_Os_EventQueue)==OS_QUEUESTATUS_VALID)
            {
                static ULONG ulData;
                static void (* pEvent)(void);

                    // Get event function pointer
                Os_QueueGet(&_Os_EventQueue, &ulData, 0);
                pEvent=(void (*)(void))ulData;

                    // dispatch event
                (*pEvent)();
            }

                // check for task switch critical section, if in it then exit immediately
            if(uwTaskSwitchCritSectCounter>0)
            {
                bForceTaskSwitchHandler=TRUE;
                break;
            }

                // if the msec counter from last task switch is not yet zero, do not make task switch
                // unless explicitly requested
            if(uwLastSwitchCounter && !bForceTaskSwitch)
                break;
            bForceTaskSwitch=FALSE;

                // begin looping in the task definition array searching next task that is
                // either running or waiting for creation
            uwNextTask=uwRunningTask;
            psRunTask=&sTasks[uwRunningTask];

                // calculate sys stack usage for current task
            uwSysStackSize=(uwSysStackTop-uwSysStackAbsAddress)>>1;

                // calculate top and bottom user stack of current allocated stack for the task
            puwTopFrameStack=psRunTask->puwStackFrameTop;
            puwBotFrameStack=(UWORD near *)((UWORD)puwTopFrameStack-(psRunTask->uwUserStackSize<<1));

#if (OS_MEASURESTACKSIZE)
            __asm
            {
                    // Check available usr stack
                mov     r3,puwBotFrameStack
                mov     r5,#TAGFORSTACKMEASUREMENT
                mov     r6,#-1

               _usrstackmeas:
                add     r6,#1
                cmp     r5,[r3+]
                jmpr    cc_Z,_usrstackmeas

                mov     uwSwapStack,r6
            }

                // if more than task usr stack size - task sys stack size then take
                // this subtraction as measurement
            if(uwSwapStack > psRunTask->uwUserStackSize-uwSysStackSize)
                uwSwapStack=psRunTask->uwUserStackSize-uwSysStackSize;

                // If measure less than actual then update
            uwSwapStack<<=1;
            if(uwSwapStack<uwOsUsrStackFree)
                uwOsUsrStackFree=uwSwapStack;
#endif

            for(;;)
            {
                    // terminate without doing anything if nothing but current task found
                uwNextTask=(uwNextTask==OS_MAXTASKSALLOWED-1?0:uwNextTask+1);
                if(uwNextTask==uwRunningTask)
                    break;
                psNextTask=&sTasks[uwNextTask];

                    // task found
                if(psNextTask->ubStatus==OS_TASKSTATUS_WAITINGFORCREATE || \
                   psNextTask->ubStatus==OS_TASKSTATUS_RUNNING)
                {
                        // save sys stack usage for future restoring
                    psRunTask->uwSystemStackSize=uwSysStackSize;

                            // calculate user stack usage for current task, that will also include sys stack usage
                        __asm { mov     uwSwapStack,r0 };
                    psRunTask->uwUserStackSize=(((UWORD)puwTopFrameStack-uwSwapStack)>>1)+uwSysStackSize;

                        // check shrinked user + sys stack size do not overcome reserved space
                    if(psRunTask->uwUserStackSize > psRunTask->uwResUsrStack)
                        ShowFatalErrorAndHalt(FATAL_ERROR_OS_STACKOVERFLOW);

#if (OS_MEASURESTACKSIZE)
                        // stack measurement
                    uwDelta=(psRunTask->uwResUsrStack-psRunTask->uwUserStackSize)<<1;
                    if(uwDelta<uwOsMinStackFreePerTask[uwRunningTask])
                        uwOsMinStackFreePerTask[uwRunningTask]=uwDelta;
#endif

                        // if next task index is more than running task index then stack between two
                        // must be moved up
                    if(uwNextTask>uwRunningTask)
                    {
                            // first flush sys stack onto user stack, as it will be not moved in this case
                        if(uwSysStackSize>0)
                            __asm
                            {
                                mov     r1,uwSysStackSize

                               _mvssu:
                                pop     r2
                                mov     [-r0],r2
                                sub     r1,#1
                                jmpr    cc_NZ,_mvssu
                            }

                            // add empty delta to usr stack to reach next top frame
                        uwDelta=psRunTask->uwResUsrStack-psRunTask->uwUserStackSize;
                        __asm
                            {
                            mov     r1,uwDelta
                            shl     r1,#1
                            sub     r0,r1
                            }

                            // move up stacks of all tasks between the two
                        for(uwCt=uwRunningTask+1;uwCt<uwNextTask;uwCt++)
                        {
                            uwDelta=sTasks[uwCt].uwUserStackSize;
                            puwBotFrameStack=sTasks[uwCt].puwStackFrameTop;
                            uwSize=sTasks[uwCt].uwResUsrStack;

                                // save new top frame stack for the task
                            __asm { mov     puwNewFrameStack,r0 };
                            sTasks[uwCt].puwStackFrameTop=puwNewFrameStack;

                                __asm
                                {
                                mov     r4,uwSize
                                mov     r2,puwBotFrameStack
                                mov     r1,uwDelta
                                jmpr    cc_Z,_skip

                                   _shrup:
                                    sub     r2,#2
                                    mov     r3,[r2]
                                    mov     [-r0],r3
                                sub     r4,#1
                                    sub     r1,#1
                                    jmpr    cc_NZ,_shrup

                               _skip:
                                shl     r4,#1
                                sub     r0,r4
                                }
                        }

                            // get actual top frame stack for later stack move
                        puwBotFrameStack=psNextTask->puwStackFrameTop;

                            // save new top frame stack for the next task
                        __asm { mov     puwNewFrameStack,r0 };
                        psNextTask->puwStackFrameTop=puwNewFrameStack;

                            // then move up user stack
                        uwDelta=psNextTask->uwUserStackSize;
                        if(uwDelta>0)
                        {
                                // if >0 restore user stack
                            uwDelta-=psNextTask->uwSystemStackSize;
                            if(uwDelta>0)
                                __asm
                                {
                                    mov     r2,puwBotFrameStack
                                    mov     r1,uwDelta

                                   _shrupt:
                                    sub     r2,#2
                                    mov     r3,[r2]
                                    mov     [-r0],r3
                                    sub     r4,#2
                                    sub     r1,#1
                                    jmpr    cc_NZ,_shrupt

                                    mov     puwBotFrameStack,r2
                                }

                                // if also system stack then restore directly in sys stack
                            uwDelta=psNextTask->uwSystemStackSize;
                            if(uwDelta>0)
                                __asm
                                {
                                    mov     r2,puwBotFrameStack
                                    mov     r1,uwDelta
                                    mov     r4,r1
                                    shl     r4,#1
                                    sub     r2,r4

                                   _shrups:
                                    mov     r3,[r2+]
                                    push    r3
                                    sub     r1,#1
                                    jmpr    cc_NZ,_shrups
                                }
                        }
                    }

                        // otherwise stack must be moved down
                    else
                    {
                            // if current task stack valid
                        if(psRunTask->uwUserStackSize>0)
                            {
                            uwDelta=psRunTask->uwUserStackSize-uwSysStackSize;
                            uwSize=psRunTask->uwResUsrStack;

                            __asm
                            {
                                mov     r2,puwBotFrameStack
                                mov     r4,uwSize
                                mov     r1,uwDelta
                                shl     r4,#1
                                add     r2,r4
                                mov     r4,r1
                                shl     r4,#1
                                sub     r2,r4

                                    // if run task sys stack size>0 then save it directly
                                    // at bottom frame
                                mov     r4,uwSysStackSize
                                jmpr    cc_Z,_shrdownb

                                    // pop from sys stack and put it bottom
                                mov     r6,r2
                               _shrdownss:
                                pop     r3
                                mov     [-r6],r3
                                sub     r4,#1
                                jmpr    cc_NZ,_shrdownss

                                    // if usr stack size>0
                               _shrdownb:
                                cmp     r1,#0
                                jmpr    cc_Z,_shrdownskip

                                    // pop from user stack and put it bottom
                               _shrdown:
                                mov     r3,[r0+]
                                mov     [r2],r3
                                add     r2,#2
                                sub     r1,#1
                                jmpr    cc_NZ,_shrdown

                               _shrdownskip:
                            }
                        }

                            // save new top frame stack for the task
                        psRunTask->puwStackFrameTop=(UWORD near *)((UWORD)puwBotFrameStack+(psRunTask->uwResUsrStack<<1));

                            // move down stacks of all tasks between the two, start top frame is
                            // the one of the previous task
                        puwNewFrameStack=psRunTask->puwStackFrameTop;
                        for(uwCt=uwRunningTask-1;uwCt>uwNextTask;uwCt--)
                        {
                            uwDelta=sTasks[uwCt].uwUserStackSize;
                            uwSize=sTasks[uwCt].uwResUsrStack;

                            __asm
                            {
                                mov     r4,uwSize
                                mov     r2,puwNewFrameStack
                                mov     r1,uwDelta

                                sub     r4,r1
                                shl     r4,#1
                                add     r0,r4
                                add     r2,r4

                                cmp     r1,#0
                                jmpr    cc_Z,_skipd

                               _shrdw:
                                mov     r3,[r0+]
                                mov     [r2],r3
                                add     r2,#2
                                sub     r1,#1
                                jmpr    cc_NZ,_shrdw

                               _skipd:
                                mov     puwNewFrameStack,r2
                            }

                                // save new top frame stack for the task
                            sTasks[uwCt].puwStackFrameTop=puwNewFrameStack;
                        }

                            // and if valid
                        if(psNextTask->uwUserStackSize>0)
                        {
                                // restore sys stack popping it from user stack
                                // and pushing it back on sys stack
                            uwSysStackSize=psNextTask->uwSystemStackSize;
                            uwDelta=psNextTask->uwUserStackSize;
                            uwSize=psNextTask->uwResUsrStack;

                            __asm
                            {
                                mov     r4,uwSize
                                sub     r4,uwDelta
                                shl     r4,#1
                                add     r0,r4
                                mov     r1,uwSysStackSize

                               _restoresysstack:
                                mov     r3,[r0+]
                                push    r3
                                sub     r1,#1
                                jmpr    cc_NZ,_restoresysstack
                            }
                        }
                            // new task, setup top frame stack
                        else
                            psNextTask->puwStackFrameTop=(UWORD near *)((UWORD)puwNewFrameStack+(uwRunningTaskUsrStackSize<<1));
                    }

                        // now stack swapping has done; then,
                        // if running task is pending for start
                    if(psNextTask->ubStatus==OS_TASKSTATUS_WAITINGFORCREATE)
                    {
                        if(CPUCON1_SGTDIS())
                            uwSwapStack=0;
                        else
                            uwSwapStack=1;

                            // push the return address for TaskSuspedWrapper that will be used
                            // by the core RETI opcode, that will be PSW+IP
                            // in case of non-segmented mode (like tiny memory model)
                            // or PSW+CSP+IP in case of segmented mode (like large memory model)
                            // this will be responsible to launch task
                        pTaskEntryPoint=&TaskSuspendWrapper;

                        __asm
                        {
                                // PSW
                            mov     r2,uwCleanRegPSW
                            push    r2

                            mov     r2,uwSwapStack
                            cmp     r2,#0
                            jmpr    cc_Z,_sgtdis2

                                // segment (if apply)
                            mov     r3,pTaskEntryPoint+02h
                            push    r3

                           _sgtdis2:
                                // return address
                            mov     r3,pTaskEntryPoint
                            push    r3

                                // fill-up with template registers
                            mov     r2,#puwCleanRegsTemplate
                            mov     r1,uwCleanRegsTemplateSize

                           _filltemplate:
                            mov     r3,[r2+]
                            push    r3
                            sub     r1,#1
                            jmpr    cc_NZ,_filltemplate

                                // get actual user stack to be written as task top user stack
                            mov     uwSwapStack,r0
                        }

                            // update overall stack size of running task
                        uwRunningTaskUsrStackSize-=psNextTask->uwResUsrStack;

#if (OS_MEASURESTACKSIZE)
                        uwOsMinStackFreePerTask[uwNextTask]=uwUsrStackSize<<1;
#endif
                            // and set as running
                        psNextTask->ubStatus=OS_TASKSTATUS_RUNNING;

                    }

                        // calc new user stack size
                    uwRunningTaskUsrStackSize+=psNextTask->uwResUsrStack-psRunTask->uwResUsrStack;

                        // and assign to current running task
                    psNextTask->uwUserStackSize=uwRunningTaskUsrStackSize;

#if (OS_MEASURESTACKSIZE)
                        // calculate bottom user stack of current allocated stack for the next running task
                    puwBotFrameStack=(UWORD near *)((UWORD)psNextTask->puwStackFrameTop-(psNextTask->uwUserStackSize<<1));

                    __asm
                    {
                            // Tag new user stack
                        mov     r3,puwBotFrameStack
                        mov     r4,r0
                        mov     r5,#TAGFORSTACKMEASUREMENT

                       _nusrstackzero:
                        mov     [-r4],r5
                        cmp     r3,r4
                        jmpr    cc_NZ,_nusrstackzero
                    }
#endif

                        // set current running task
                    uwRunningTask=uwNextTask;

                        // restart down counter
                    uwLastSwitchCounter=uwTimeSlice;

                    break;
                }
            }

            break;
    }

        // Restore R13-15 and DPPx registers
    __asm
    {
        pop     DPP3
        pop     DPP2
        pop     DPP1
        pop     DPP0

        pop     r15
        pop     r14
        pop     r13
    }
}

#pragma nosavemac
#endif
