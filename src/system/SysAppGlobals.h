/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppGlobals.h                                            */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Globals parameters/variables for system app                */
/*                                                                          */
/****************************************************************************/

#ifndef _SYSAPPGLOBALS_H
#define _SYSAPPGLOBALS_H

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "system\SysAppConfig.h"
#include "system\SystemStatus.h"
#include "system\SystemAlarms.h"
#include "system\Os.h"
#include "drive\AssemblyInfo.h"
#include "drive\HardwareParameters.h"
#include "drive\HardwareParamsCBrd.h"
#include "drive\HardwareParams2.h"
#include "drive\MotorParameters.h"

#include "core\Timer.h"
#include "core\Gpio.h"
#include "core\Flash.h"
#include "core\Interrupt.h"
#include "core\Adc.h"

#include "assert.h"

#include "FreeRTOS.h"
#include "task.h"

//****************************************************************************
// Data structures

typedef struct
{
    ULONG ulHardwareOpt;
    ULONG ulFPGAOpt;
} GLB_REQ_OPT;

//****************************************************************************
// Control board configuration and parameters

extern HWPRM_ALL_CBRD sGlbControlBoardParameters;

extern UWORD uwSystemMainClockFreq;       // [MHz]

#ifdef _APP_XC
extern UWORD  uwSystemTmr100Corr;          // fractional correction for
                                                // 100ns timer
extern UWORD  uwSystemSetTmr100;           // fractional correction for
                                                // correct setting of 100ns timer
extern UWORD  uwSystemFBusTSCorr;          // fractional correction for
                                                // fieldbus realtime timestamp
#endif

//****************************************************************************
// Power board configuration and parameters

extern UWORD  sGlbPowerBoardParamDataCode;
extern HWPRM_POWER_BOARD  sGlbPowerBoardParameters;

//****************************************************************************
// Assembly info

extern ASSEMBLY_INFO  sGlbAssemblyInfo;

//****************************************************************************
// IO expansion board configuration and parameters

extern HWPRM_IO_EXP_BOARD  sGlbIOExpBoardParameters;

//****************************************************************************
// Hardware options configuration requested/available

extern GLB_REQ_OPT  sGlbOptReq;

extern GLB_REQ_OPT  sGlbOptAvailable;

//****************************************************************************
// System Timers
#ifdef _INFINEON_
    // life timer [sec]
#define ulSysTimersTotalPowerOnTime     ulOsTimer1Hz

    // free running timers
#define uwSysTimers100ns                CC2_T8
#define uwSysTimers125us                uwTaskSchedFreeTimer
#define uwSysTimers1ms                  uwOsFreeRunTimer1kHz

#define ulSysAbsoluteTimer1s            ulOsTimer1Hz

    // timestamp timer for fieldbuses syncing
#define uwSysFieldbusTSTimer            CC2_T7
#define uwSysFieldbusRTTimer            uwSysTimers125us
EXTERN UWORD sdata uwSysFieldbusSampleTS;
EXTERN UWORD sdata uwSysFieldbusSampleRT;
#else
// life timer [sec]
#define ulSysTimersTotalPowerOnTime     ulOsTimer1Hz

// free running timers
#define uwSysTimers100ns                TTC_TMR
// #define uwSysTimers125us                uwXTTCTimersTask
#define uwSysTimers125us                uwTaskSchedFreeTimer
#define uwSysTimers1ms                  uwXTTCTimers1ms
//#define uwSysTimers1ms                  uwOsFreeRunTimer1kHz

#define ulSysAbsoluteTimer1s            ulOsTimer1Hz

// timestamp timer for fieldbuses syncing
#define uwSysFieldbusTSTimer            (UWORD)(0-XTtcPs_GetCounterValue(&xTimer0))
#define uwSysFieldbusRTTimer            uwSysTimers125us
extern UWORD  uwSysFieldbusSampleTS;
extern UWORD  uwSysFieldbusSampleRT;
#endif

//****************************************************************************
// Global Sync definitions

#define SYNCMGR_NOMINALVALUE_COMP       10000           // 12.5ns/tick @ 80MHz
#define SYNCMGR_NOMINALVALUE_PERF       15625           // 8ns/tick @ 125MHz
#define SYNCMGR_MAXSHIFT                0.025

#ifdef _INFINEON_
#define SYNCMGR_TIMER                   uwSysFieldbusTimestamp
#define SYNCMGR_FULLRELOAD              CC2_T7REL
#define SYNCMGR_HALFRELOAD              CC2_CC30
#else
#define SYNCMGR_NOMINALVALUE_TTC        (TTC_CLK_FREQ_KHZ/8)

#define SYNCMGR_TIMER                   uwSysFieldbusTimestamp
#define SYNCMGR_FULLRELOAD_SET(x)       Timer_SetIntervalTime((UWORD)(0-(x)))
#define SYNCMGR_FULLRELOAD_GET()        (UWORD)(0-Timer_GetIntervalTime())
#define SYNCMGR_HALFRELOAD_SET(x)       Timer_SetMatchTime((UWORD)(0-(x)))
#define SYNCMGR_HALFRELOAD_GET()        (UWORD)(0-Timer_GetMatchTime())
#endif

//****************************************************************************
// Task Scheduler configuration
#ifdef _INFINEON_
#define TASKSCHED_REALTIME_IV           0x1E            // CC2_CC30INT
#define TASKSCHED_REALTIME_IRFLAG       CC2_CC30IC_IR
#else
#define TASKSCHED_REALTIME_IRFLAG       0
#endif

//****************************************************************************
// Initialized Globals

extern const CHARS chSystemId[4];
extern const CHARS chFwApplicationId[4];

#define SYSTEM_DISABLEWDT_KEY           0xE3B6

extern UWORD uwSystemDisableWDT;

#define SYSTEM_ENTERDIAGNOSTIC_KEY      0xA15C

extern UWORD uwSystemEnterDiagnostic;

#ifdef _HW_DC
extern UBYTE ubIOboardType;
#endif

//****************************************************************************
// Global functions

// Initialization
BOOL Globals_Init(void);

//****************************************************************************
// Linker Memory Layout
#include "SysAppZYNQ.lin"

#endif
