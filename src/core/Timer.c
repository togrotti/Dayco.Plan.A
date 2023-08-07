/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Timer.c                                                    */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : setup system tick time and interrupt	                    */
/*                                                                          */
/****************************************************************************/

#include <stdint.h>

#include "xttcps.h"
#include "xscugic.h"
#include "Xil_assert.h"

#ifdef _AXX_SYSAPP
#include "system\SysAppGlobals.h"
#include "system\SysAppInterruptLevels.h"
#else
#include "system\BootBlockGlobals.h"
#include "system\BootBlockInterruptLevels.h"
#endif
#include <math.h>

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

//***************************************************************************
// Defines

typedef struct  {
	void (*callback)();
	ULONG ulFreq;	// interrupt frequency
	UBYTE ubDuty;	// output signal duty
} XTTCSettings;

typedef struct {
	BOOL  started;
	ULONG count;
	ULONG current;			// [ticks]
	ULONG interval; 		// [ticks]
	ULONG last_duration; 	// [ticks]
	ULONG max_duration; 	// [ticks]
} ISRSTATUS;

//***************************************************************************
// Locals

static XTTCSettings sXTTCSettings = { NULL, kFrequency8KHz, 50 };

static void xttc_0_handler( void *callback_ref );
static void xttc_1_handler( void *callback_ref );

static void xttc_setimer();
static void xttc_setintr();

static ISRSTATUS sISRStatus0,sISRStatus1;

static void update_interval(ISRSTATUS *pStatus);
static void update_duration(ISRSTATUS *pStatus);

//***************************************************************************
// Globals

XTtcPs xTimer0,xTimer1,xTimer2;
#if (XPAR_XTTCPS_NUM_INSTANCES>3)
XTtcPs xTimer3,xTimer4,xTimer5;
#endif

volatile UWORD uwXTTCTimers1ms;
volatile UWORD uwXTTCTimersTask;

//***************************************************************************
// Timer Initialization
void Timer_Init( ULONG ulFreq, void (*callback)() ) {
	uwXTTCTimers1ms = 0;
	uwXTTCTimersTask = 0;

	sXTTCSettings.ulFreq = ulFreq;
	sXTTCSettings.callback = callback;

	xttc_setimer();
	xttc_setintr();
	XTtcPs_Start( &xTimer0 );
	XTtcPs_Start( &xTimer1 );
	XTtcPs_Start( &xTimer2 );
#if (XPAR_XTTCPS_NUM_INSTANCES>3)
	XTtcPs_Start( &xTimer3 );
	XTtcPs_Start( &xTimer4 );
	XTtcPs_Start( &xTimer5 );
#endif
}

//***************************************************************************
// Set Real Time Task Handler
void Timer_SetRTask( void (*callback)() ) {
	sXTTCSettings.callback = callback;
}


//***************************************************************************
// Capture/Compare Unit Functions
void Timer_CCSet(UWORD uwTimerID, UBYTE ubPriority, void *callback)
{
	const UBYTE ucRisingEdge = 3;

	if(!xInterruptController.IsReady)
	{
		/* Initialize the interrupt controller driver so that it is ready to use. */
		XScuGic_Config* GicConfig = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);

		Xil_AssertVoid( XScuGic_CfgInitialize(&xInterruptController, GicConfig, GicConfig->CpuBaseAddress) == XST_SUCCESS );

		/* Perform a self-test to ensure that the hardware was built correctly */
		Xil_AssertVoid( XScuGic_SelfTest(&xInterruptController) == XST_SUCCESS );

		/*
		 * Connect the interrupt controller interrupt handler to the hardware
		 * interrupt handling logic in the ARM processor.
		 */
		Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler) XScuGic_InterruptHandler,
				NULL);

		/*
		 * Enable interrupts in the ARM
		 */
		Xil_ExceptionEnable();
	}

	switch(uwTimerID)
	{
		case TIMER_CCU0_ID:
			//****************************** Setup TTC 2 Interrupt ******************************//
			/* The priority must exceed configMAXAPICALLINTERRUPTPRIORITY. */
			XScuGic_SetPriorityTriggerType( &xInterruptController, TIMER_CCU0_INTR, ubPriority, ucRisingEdge );
			/* Connect to the interrupt controller. */
			Xil_AssertVoid( XScuGic_Connect( &xInterruptController, TIMER_CCU0_INTR, ( Xil_InterruptHandler )XTtcPs_InterruptHandler, ( void * ) TIMER_CCU0_PTR ) == XST_SUCCESS);
			/* Set TTC Status handler */
			XTtcPs_SetStatusHandler(TIMER_CCU0_PTR, TIMER_CCU0_PTR, (XTtcPs_StatusHandler)callback);
			/* Enable the interrupt in the GIC. */
			XScuGic_Enable( &xInterruptController, TIMER_CCU0_INTR );
			break;
#ifdef TIMER_CCU1_ID
		case TIMER_CCU1_ID:
			//****************************** Setup TTC 2 Interrupt ******************************//
			/* The priority must exceed configMAXAPICALLINTERRUPTPRIORITY. */
			XScuGic_SetPriorityTriggerType( &xInterruptController, TIMER_CCU1_INTR, ubPriority, ucRisingEdge );
			/* Connect to the interrupt controller. */
			Xil_AssertVoid( XScuGic_Connect( &xInterruptController, TIMER_CCU1_INTR, ( Xil_InterruptHandler )XTtcPs_InterruptHandler, ( void * ) TIMER_CCU1_PTR ) == XST_SUCCESS);
			/* Set TTC Status handler */
			XTtcPs_SetStatusHandler(TIMER_CCU1_PTR, TIMER_CCU1_PTR, (XTtcPs_StatusHandler)callback);
			/* Enable the interrupt in the GIC. */
			XScuGic_Enable( &xInterruptController, TIMER_CCU1_INTR );
			break;
#endif
#ifdef TIMER_CCU2_ID
		case TIMER_CCU2_ID:
			//****************************** Setup TTC 2 Interrupt ******************************//
			/* The priority must exceed configMAXAPICALLINTERRUPTPRIORITY. */
			XScuGic_SetPriorityTriggerType( &xInterruptController, TIMER_CCU2_INTR, ubPriority, ucRisingEdge );
			/* Connect to the interrupt controller. */
			Xil_AssertVoid( XScuGic_Connect( &xInterruptController, TIMER_CCU2_INTR, ( Xil_InterruptHandler )XTtcPs_InterruptHandler, ( void * ) TIMER_CCU2_PTR ) == XST_SUCCESS);
			/* Set TTC Status handler */
			XTtcPs_SetStatusHandler(TIMER_CCU2_PTR, TIMER_CCU2_PTR, (XTtcPs_StatusHandler)callback);
			/* Enable the interrupt in the GIC. */
			XScuGic_Enable( &xInterruptController, TIMER_CCU2_INTR );
			break;
#endif
#ifdef TIMER_CCU3_ID
		case TIMER_CCU3_ID:
			//****************************** Setup TTC 2 Interrupt ******************************//
			/* The priority must exceed configMAXAPICALLINTERRUPTPRIORITY. */
			XScuGic_SetPriorityTriggerType( &xInterruptController, TIMER_CCU3_INTR, ubPriority, ucRisingEdge );
			/* Connect to the interrupt controller. */
			Xil_AssertVoid( XScuGic_Connect( &xInterruptController, TIMER_CCU3_INTR, ( Xil_InterruptHandler )XTtcPs_InterruptHandler, ( void * ) TIMER_CCU3_PTR ) == XST_SUCCESS);
			/* Set TTC Status handler */
			XTtcPs_SetStatusHandler(TIMER_CCU3_PTR, TIMER_CCU3_PTR, (XTtcPs_StatusHandler)callback);
			/* Enable the interrupt in the GIC. */
			XScuGic_Enable( &xInterruptController, TIMER_CCU3_INTR );
			break;
#endif
	}
}

void Timer_CCStart(UWORD uwTimerID, UWORD uwMatchTime)
{
	switch(uwTimerID)
	{
		case TIMER_CCU0_ID:
			XTtcPs_SetMatchValue( TIMER_CCU0_PTR, 0, uwMatchTime);
			/* Enable the interrupts in the timer. */
			XTtcPs_EnableInterrupts( TIMER_CCU0_PTR, XTTCPS_IXR_MATCH_0_MASK );
			break;
#ifdef TIMER_CCU1_ID
		case TIMER_CCU1_ID:
			XTtcPs_SetMatchValue( TIMER_CCU1_PTR, 0, uwMatchTime);
			/* Enable the interrupts in the timer. */
			XTtcPs_EnableInterrupts( TIMER_CCU1_PTR, XTTCPS_IXR_MATCH_0_MASK );
			break;
#endif
#ifdef TIMER_CCU2_ID
		case TIMER_CCU2_ID:
			XTtcPs_SetMatchValue( TIMER_CCU2_PTR, 0, uwMatchTime);
			/* Enable the interrupts in the timer. */
			XTtcPs_EnableInterrupts( TIMER_CCU2_PTR, XTTCPS_IXR_MATCH_0_MASK );
			break;
#endif
#ifdef TIMER_CCU3_ID
		case TIMER_CCU3_ID:
			XTtcPs_SetMatchValue( TIMER_CCU3_PTR, 0, uwMatchTime);
			/* Enable the interrupts in the timer. */
			XTtcPs_EnableInterrupts( TIMER_CCU3_PTR, XTTCPS_IXR_MATCH_0_MASK );
			break;
#endif
	}
}

void Timer_CCStop(UWORD uwTimerID)
{
	switch(uwTimerID)
	{
		case TIMER_CCU0_ID:
			/* Disable the interrupts in the timer. */
			XTtcPs_DisableInterrupts( TIMER_CCU0_PTR, XTTCPS_IXR_MATCH_0_MASK);
			break;
#ifdef TIMER_CCU1_ID
		case TIMER_CCU1_ID:
			/* Disable the interrupts in the timer. */
			XTtcPs_DisableInterrupts( TIMER_CCU1_PTR, XTTCPS_IXR_MATCH_0_MASK);
			break;
#endif
#ifdef TIMER_CCU2_ID
		case TIMER_CCU2_ID:
			/* Disable the interrupts in the timer. */
			XTtcPs_DisableInterrupts( TIMER_CCU2_PTR, XTTCPS_IXR_MATCH_0_MASK);
			break;
#endif
#ifdef TIMER_CCU3_ID
		case TIMER_CCU3_ID:
			/* Disable the interrupts in the timer. */
			XTtcPs_DisableInterrupts( TIMER_CCU3_PTR, XTTCPS_IXR_MATCH_0_MASK);
			break;
#endif
	}
}

BOOL Timer_CCStatus(UWORD uwTimerID)
{
	UWORD uwIntMask;
	switch(uwTimerID)
	{
		case TIMER_CCU0_ID:
			uwIntMask = InstReadReg(TIMER_CCU0_PTR, XTTCPS_IER_OFFSET);
			break;
#ifdef TIMER_CCU1_ID
		case TIMER_CCU1_ID:
			uwIntMask = InstReadReg(TIMER_CCU1_PTR, XTTCPS_IER_OFFSET);
			break;
#endif
#ifdef TIMER_CCU2_ID
		case TIMER_CCU2_ID:
			uwIntMask = InstReadReg(TIMER_CCU2_PTR, XTTCPS_IER_OFFSET);
			break;
#endif
#ifdef TIMER_CCU3_ID
		case TIMER_CCU3_ID:
			uwIntMask = InstReadReg(TIMER_CCU3_PTR, XTTCPS_IER_OFFSET);
			break;
#endif
	}

	return (BOOL)((uwIntMask&XTTCPS_IXR_MATCH_0_MASK)==XTTCPS_IXR_MATCH_0_MASK);
}

//***************************************************************************
// Locals
static void xttc_setimer()
{
	XTtcPs * 		pTimer; 			  /* XTtcPs driver instance */
	XTtcPs_Config * pxTimerConfiguration; /* Look up the timer's configuration. */
	UWORD Interval;		/* Interval value. */
	UBYTE Prescaler;	/* Prescaler value. */
	UWORD Matchvalue;	/* Match value. */

	//*********************************** Setup TTC 0, system task timer ***********************************//
	pTimer =  &xTimer0;
	pxTimerConfiguration = XTtcPs_LookupConfig( XPAR_XTTCPS_0_DEVICE_ID );

	/* Initialise the device. */
	int xStatus = XTtcPs_CfgInitialize( pTimer, pxTimerConfiguration, pxTimerConfiguration->BaseAddress );
	if( xStatus != XST_SUCCESS )
	{
		/* Not sure how to do this before XTtcPs_CfgInitialize is called
				as pxTimerInstance is set within XTtcPs_CfgInitialize(). */
		XTtcPs_Stop( pTimer );
		xStatus = XTtcPs_CfgInitialize( pTimer, pxTimerConfiguration, pxTimerConfiguration->BaseAddress );
		Xil_AssertVoid( xStatus == XST_SUCCESS );
	}

	/* Set the options, Clock Source:
	 * 1. Internal Clock: PS&PL
	 * 2. External Clock: MIO
	 */
	XTtcPs_SetOptions( pTimer, XTTCPS_OPTION_INTERVAL_MODE |
							   XTTCPS_OPTION_DECREMENT     |
							   XTTCPS_OPTION_MATCH_MODE    |
							   XTTCPS_OPTION_WAVE_POLARITY );

	/* The timer frequency is preset in the pxTimerSettings structure.
			Derive the values for the other structure members. */
	/* CLKCOUNTER = 40MHz */
	XTtcPs_CalcIntervalFromFreq( pTimer, sXTTCSettings.ulFreq, &Interval, &Prescaler );
	Matchvalue = (Interval*sXTTCSettings.ubDuty)/100;

	/* Set the interval and prescale. */
	XTtcPs_SetInterval( pTimer, Interval );
	XTtcPs_SetPrescaler( pTimer, Prescaler );
	XTtcPs_SetMatchValue( pTimer, 0, Matchvalue);

	//*********************************** Setup TTC 1, base task timer ***********************************//
	pTimer = &xTimer1;
	pxTimerConfiguration = XTtcPs_LookupConfig( XPAR_XTTCPS_1_DEVICE_ID );

	/* Initialise the device. */
	xStatus = XTtcPs_CfgInitialize( pTimer, pxTimerConfiguration, pxTimerConfiguration->BaseAddress );
	if( xStatus != XST_SUCCESS )
	{
		/* Not sure how to do this before XTtcPs_CfgInitialize is called
				as pxTimerInstance is set within XTtcPs_CfgInitialize(). */
		XTtcPs_Stop( pTimer );
		xStatus = XTtcPs_CfgInitialize( pTimer, pxTimerConfiguration, pxTimerConfiguration->BaseAddress );
		Xil_AssertVoid( xStatus == XST_SUCCESS );
	}

	/* Set the options. */
	XTtcPs_SetOptions( pTimer, XTTCPS_OPTION_INTERVAL_MODE |
							   XTTCPS_OPTION_WAVE_DISABLE  );

	/* The timer frequency is preset in the pxTimerSettings structure.
			Derive the values for the other structure members. */
	XTtcPs_CalcIntervalFromFreq( pTimer, sXTTCSettings.ulFreq, &Interval, &Prescaler );

	/* Set the interval and prescale. */
	XTtcPs_SetInterval( pTimer, Interval );
	XTtcPs_SetPrescaler( pTimer, Prescaler );

	//****************************** Setup TTC 2, free running timer, tick 100ns ******************************//
	pTimer = &xTimer2;
	pxTimerConfiguration = XTtcPs_LookupConfig( XPAR_XTTCPS_2_DEVICE_ID );

	/* Initialise the device. */
	xStatus = XTtcPs_CfgInitialize( pTimer, pxTimerConfiguration, pxTimerConfiguration->BaseAddress );
	if( xStatus != XST_SUCCESS )
	{
		/* Not sure how to do this before XTtcPs_CfgInitialize is called
				as pxTimerInstance is set within XTtcPs_CfgInitialize(). */
		XTtcPs_Stop( pTimer );
		xStatus = XTtcPs_CfgInitialize( pTimer, pxTimerConfiguration, pxTimerConfiguration->BaseAddress );
		Xil_AssertVoid( xStatus == XST_SUCCESS );
	}

	/* CLKCOUNTER = 10MHz */
	Prescaler = log2(pTimer->Config.InputClockHz/10000000)-1;
	XTtcPs_SetPrescaler( pTimer, Prescaler );

	/* Set the options. */
	XTtcPs_SetOptions( pTimer, XTTCPS_OPTION_WAVE_DISABLE |
							   XTTCPS_OPTION_MATCH_MODE    |
							   XTTCPS_OPTION_EXTERNAL_CLK );

#if (XPAR_XTTCPS_NUM_INSTANCES>3)
	//****************************** Setup TTC 3, free running timer, tick 100ns ******************************//
	pTimer = &xTimer3;
	pxTimerConfiguration = XTtcPs_LookupConfig( XPAR_XTTCPS_3_DEVICE_ID );

	/* Initialise the device. */
	xStatus = XTtcPs_CfgInitialize( pTimer, pxTimerConfiguration, pxTimerConfiguration->BaseAddress );
	if( xStatus != XST_SUCCESS )
	{
		/* Not sure how to do this before XTtcPs_CfgInitialize is called
				as pxTimerInstance is set within XTtcPs_CfgInitialize(). */
		XTtcPs_Stop( pTimer );
		xStatus = XTtcPs_CfgInitialize( pTimer, pxTimerConfiguration, pxTimerConfiguration->BaseAddress );
		Xil_AssertVoid( xStatus == XST_SUCCESS );
	}

	/* CLKCOUNTER = 10MHz */
	Prescaler = log2(pTimer->Config.InputClockHz/10000000)-1;
	XTtcPs_SetPrescaler( pTimer, Prescaler );

	/* Set the options. */
	XTtcPs_SetOptions( pTimer, XTTCPS_OPTION_WAVE_DISABLE |
							   XTTCPS_OPTION_MATCH_MODE    |
							   XTTCPS_OPTION_EXTERNAL_CLK );

	//****************************** Setup TTC 4, free running timer, tick 100ns ******************************//
	pTimer = &xTimer4;
	pxTimerConfiguration = XTtcPs_LookupConfig( XPAR_XTTCPS_4_DEVICE_ID );

	/* Initialise the device. */
	xStatus = XTtcPs_CfgInitialize( pTimer, pxTimerConfiguration, pxTimerConfiguration->BaseAddress );
	if( xStatus != XST_SUCCESS )
	{
		/* Not sure how to do this before XTtcPs_CfgInitialize is called
				as pxTimerInstance is set within XTtcPs_CfgInitialize(). */
		XTtcPs_Stop( pTimer );
		xStatus = XTtcPs_CfgInitialize( pTimer, pxTimerConfiguration, pxTimerConfiguration->BaseAddress );
		Xil_AssertVoid( xStatus == XST_SUCCESS );
	}

	/* CLKCOUNTER = 10MHz */
	Prescaler = log2(pTimer->Config.InputClockHz/10000000)-1;
	XTtcPs_SetPrescaler( pTimer, Prescaler );

	/* Set the options. */
	XTtcPs_SetOptions( pTimer, XTTCPS_OPTION_WAVE_DISABLE |
							   XTTCPS_OPTION_MATCH_MODE    |
							   XTTCPS_OPTION_EXTERNAL_CLK );

	//****************************** Setup TTC 5, free running timer, tick 100ns ******************************//
	pTimer = &xTimer5;
	pxTimerConfiguration = XTtcPs_LookupConfig( XPAR_XTTCPS_5_DEVICE_ID );

	/* Initialise the device. */
	xStatus = XTtcPs_CfgInitialize( pTimer, pxTimerConfiguration, pxTimerConfiguration->BaseAddress );
	if( xStatus != XST_SUCCESS )
	{
		/* Not sure how to do this before XTtcPs_CfgInitialize is called
				as pxTimerInstance is set within XTtcPs_CfgInitialize(). */
		XTtcPs_Stop( pTimer );
		xStatus = XTtcPs_CfgInitialize( pTimer, pxTimerConfiguration, pxTimerConfiguration->BaseAddress );
		Xil_AssertVoid( xStatus == XST_SUCCESS );
	}

	/* CLKCOUNTER = 10MHz */
	Prescaler = log2(pTimer->Config.InputClockHz/10000000)-1;
	XTtcPs_SetPrescaler( pTimer, Prescaler );

	/* Set the options. */
	XTtcPs_SetOptions( pTimer, XTTCPS_OPTION_WAVE_DISABLE |
							   XTTCPS_OPTION_MATCH_MODE    |
							   XTTCPS_OPTION_EXTERNAL_CLK );
#endif
}

void Timer_SetMatchTime(UWORD uwMatchTime)
{
	XTtcPs_SetMatchValue( &xTimer0, 0, uwMatchTime);
}

UWORD Timer_GetMatchTime(void)
{
	return (UWORD)XTtcPs_GetMatchValue( &xTimer0, 0 );
}

void Timer_SetIntervalTime(UWORD Interval)
{
	/* Set the interval and prescale. */
	XTtcPs_SetInterval( &xTimer0, Interval );
}

UWORD Timer_GetIntervalTime(void)
{
	return (UWORD)XTtcPs_GetInterval( &xTimer0 );
}

static void xttc_setintr(void)
{
	const UBYTE ucRisingEdge = 3;

	if(!xInterruptController.IsReady)
	{
		/* Initialize the interrupt controller driver so that it is ready to use. */
		XScuGic_Config* GicConfig = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);

		Xil_AssertVoid( XScuGic_CfgInitialize(&xInterruptController, GicConfig, GicConfig->CpuBaseAddress) == XST_SUCCESS );

		/* Perform a self-test to ensure that the hardware was built correctly */
		Xil_AssertVoid( XScuGic_SelfTest(&xInterruptController) == XST_SUCCESS );

		/*
		 * Connect the interrupt controller interrupt handler to the hardware
		 * interrupt handling logic in the ARM processor.
		 */
		Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler) XScuGic_InterruptHandler,
				NULL);

		/*
		 * Enable interrupts in the ARM
		 */
		Xil_ExceptionEnable();
	}

	//****************************** Setup TTC 0 Interrupt ******************************//
	/* The priority must exceed configMAXAPICALLINTERRUPTPRIORITY. */
	XScuGic_SetPriorityTriggerType( &xInterruptController, XPAR_XTTCPS_0_INTR, SYSAPPIL_TASKSCHEDULER, ucRisingEdge );
	/* Connect to the interrupt controller. */
	Xil_AssertVoid( XScuGic_Connect( &xInterruptController, XPAR_XTTCPS_0_INTR, ( Xil_InterruptHandler )XTtcPs_InterruptHandler, ( void * ) &xTimer0 ) == XST_SUCCESS);
	/* Set TTC Status handler */
	XTtcPs_SetStatusHandler(&xTimer0, &xTimer0, (XTtcPs_StatusHandler)xttc_0_handler);
	/* Enable the interrupt in the GIC. */
	XScuGic_Enable( &xInterruptController, XPAR_XTTCPS_0_INTR );
	/* Enable the interrupts in the timer. */
	XTtcPs_EnableInterrupts( &xTimer0, XTTCPS_IXR_INTERVAL_MASK );

	sISRStatus0.started = true;

	//****************************** Setup TTC 1 Interrupt ******************************//
	/* The priority must exceed configMAXAPICALLINTERRUPTPRIORITY. */
	XScuGic_SetPriorityTriggerType( &xInterruptController, XPAR_XTTCPS_1_INTR, 0, ucRisingEdge );
	/* Connect to the interrupt controller. */
	Xil_AssertVoid( XScuGic_Connect( &xInterruptController, XPAR_XTTCPS_1_INTR, ( Xil_InterruptHandler )XTtcPs_InterruptHandler, ( void * ) &xTimer1 ) == XST_SUCCESS);
	/* Set TTC Status handler */
	XTtcPs_SetStatusHandler(&xTimer1, &xTimer1, (XTtcPs_StatusHandler)xttc_1_handler);
	/* Enable the interrupt in the GIC. */
	XScuGic_Enable( &xInterruptController, XPAR_XTTCPS_1_INTR );
	/* Enable the interrupts in the timer. */
	XTtcPs_EnableInterrupts( &xTimer1, XTTCPS_IXR_INTERVAL_MASK );

	sISRStatus1.started = true;
}

//***************************************************************************
// TTC 0 Interrupt handler
static void xttc_0_handler( void *callback_ref )
{
	update_interval(&sISRStatus0);

	if ( sXTTCSettings.callback != NULL )
	{
		sXTTCSettings.callback();
	}

	update_duration(&sISRStatus0);
}

//***************************************************************************
// TTC 1 Interrupt handler
static void xttc_1_handler( void *callback_ref )
{
	static UWORD count;

	update_interval(&sISRStatus1);

	uwXTTCTimersTask++;
	count++;
	if(count==sXTTCSettings.ulFreq/kFrequency1KHz)
	{
		count=0;
		uwXTTCTimers1ms++;
	}

	update_duration(&sISRStatus1);
}

static void update_interval(ISRSTATUS *pStatus) {
	ULONG previous = pStatus->current;

//	pStatus->current = (ULONG)GLOBAL_TMR;
	pStatus->current = (ULONG)TTC_TMR;

	if (pStatus->count++ > 1)
		pStatus->interval = (pStatus->current - previous);
}

static void update_duration(ISRSTATUS *pStatus) {
	if (pStatus->count > 1)
	{
//		pStatus->last_duration = (ULONG)((ULONG)GLOBAL_TMR - pStatus->current);
		pStatus->last_duration = (ULONG)((ULONG)TTC_TMR - pStatus->current);
		pStatus->max_duration = max(pStatus->last_duration, pStatus->max_duration);
	}
}

