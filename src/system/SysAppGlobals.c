/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppGlobals.c                                            */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Globals parameters/variables for system app                */
/*                                                                          */
/****************************************************************************/

#include "SysAppGlobals.h"

#include "SysAppIdentification.h"

#undef DEFINE_EXTERNALS

#include <string.h>

//****************************************************************************
// Configuration checking

#ifndef _APP_XC
#ifdef  _HW_DC
#error "Option _HW_DC is supported only with _APP_XC"
#endif
#endif

//****************************************************************************
// Initialized Globals

const CHARS chSystemId[4]=IDENT_SYSTEM_ID;
const CHARS chFwApplicationId[4]=IDENT_FWAPP_ID;

UWORD uwSystemDisableWDT=0;
UWORD uwSystemEnterDiagnostic=0;

volatile SYSTEMSTATUSFLAGS sSystemStatus;
volatile SYSTEMSTATUS      ulSystemStatus;

volatile SYSTEMSTATUSFLAGS sSystemStatus2;
volatile SYSTEMSTATUS      ulSystemStatus2;


UWORD uwSystemBootErrorCode;
ULONG ulSystemAlarms;
ULONG ulSystemAlarmSubCode;
ULONG ulSystemWarnings;

UWORD uwSystemMainClockFreq;       // [MHz]

UWORD uwSysFieldbusSampleTS;
UWORD uwSysFieldbusSampleRT;

//****************************************************************************
// Global initialization

BOOL Globals_Init(void)
{
    memset(&sSystemStatus, 0, sizeof(sSystemStatus));
    ulSystemStatus=0l;

    memset(&sSystemStatus2, 0, sizeof(sSystemStatus));
    ulSystemStatus2=0l;

    uwSystemBootErrorCode=0;

    ulSystemAlarms=0l;
    ulSystemAlarmSubCode=0l;

    ulSysTimersTotalPowerOnTime=0l;

    return TRUE;
}
