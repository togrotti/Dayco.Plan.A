/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Timer.h                                                    */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : setup system tick time and interrupt	                    */
/*                                                                          */
/****************************************************************************/

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "xtime_l.h"
#include "xttcps.h"

//***************************************************************************
// Globals
static const ULONG kFrequency16KHz = 16000UL;
static const ULONG kFrequency8KHz = 8000UL;
static const ULONG kFrequency1KHz = 1000UL;
extern volatile UWORD uwXTTCTimers1ms;
extern volatile UWORD uwXTTCTimersTask;
extern XTtcPs xTimer0,xTimer1,xTimer2;
#if (XPAR_XTTCPS_NUM_INSTANCES>3)
extern XTtcPs xTimer3,xTimer4,xTimer5;
#endif

//***************************************************************************
// Defines

// GTC: Global Timer Counter
#define	COUNTS_PER_100NS   (COUNTS_PER_SECOND/10000000)
#define	COUNTS_PER_125US   (COUNTS_PER_SECOND/8000)
#define	COUNTS_PER_1MS     (COUNTS_PER_SECOND/1000)

#define GLOBAL_TMR          (*(HPULONG)(GLOBAL_TMR_BASEADDR)) // global timer tick = 1/133.33MHz*2 = 15ns

// TTC: TIMER0
#define TTC_CLK_FREQ_KHZ    (XPAR_PS7_TTC_0_TTC_CLK_FREQ_HZ/1000)

// TTC: Triple Timer Counter
#define TTC_TMR             XTtcPs_GetCounterValue(&xTimer2)   // ttc tick = 1/100MHz = 10ns

// TTC: Capture Compare Unit
#define TIMER_CCU0_ID       XPAR_XTTCPS_2_DEVICE_ID
#define TIMER_CCU0_INTR     XPAR_XTTCPS_2_INTR
#define TIMER_CCU0_PTR      (XTtcPs *)(&xTimer2)
#define TIMER_CCU0_COUNT    XTtcPs_GetCounterValue(&xTimer2)

#if (XPAR_XTTCPS_NUM_INSTANCES>3)
#define TIMER_CCU1_ID       XPAR_XTTCPS_3_DEVICE_ID
#define TIMER_CCU1_INTR     XPAR_XTTCPS_3_INTR
#define TIMER_CCU1_PTR      (XTtcPs *)(&xTimer3)
#define TIMER_CCU1_COUNT    XTtcPs_GetCounterValue(&xTimer3)

#define TIMER_CCU2_ID       XPAR_XTTCPS_4_DEVICE_ID
#define TIMER_CCU2_INTR     XPAR_XTTCPS_4_INTR
#define TIMER_CCU2_PTR      (XTtcPs *)(&xTimer4)
#define TIMER_CCU2_COUNT    XTtcPs_GetCounterValue(&xTimer4)

#define TIMER_CCU3_ID       XPAR_XTTCPS_5_DEVICE_ID
#define TIMER_CCU3_INTR     XPAR_XTTCPS_5_INTR
#define TIMER_CCU3_PTR      (XTtcPs *)(&xTimer5)
#define TIMER_CCU3_COUNT    XTtcPs_GetCounterValue(&xTimer5)
#endif

//***************************************************************************
// Globals

void Timer_Init( ULONG ulFreq, void (*callBack)() );
void Timer_SetRTask( void (*callback)() );
void Timer_CCSet(UWORD uwTimerID, UBYTE ubPriority, void *callback);
void Timer_CCStart(UWORD uwTimerID, UWORD uwMatchTime);
void Timer_CCStop(UWORD uwTimerID);
BOOL Timer_CCStatus(UWORD uwTimerID);
void Timer_SetMatchTime(UWORD uwMatchTime);
UWORD Timer_GetMatchTime(void);
void Timer_SetIntervalTime(UWORD Interval);
UWORD Timer_GetIntervalTime(void);
