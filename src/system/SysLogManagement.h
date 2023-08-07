/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysLogManagement.h                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : System Alarms and Time Logging Management Facility         */
/*                                                                          */
/****************************************************************************/

#ifndef _SYSLOGMANAGEMENT_H
#define _SYSLOGMANAGEMENT_H

#include "common\CommonDefines.h"

//***************************************************************************
// Defines

#define SYSLOGMGM_DISALM_KEY        0xE9A4

//***************************************************************************
// Data structure

typedef struct
{
    ULONG ulAbsoluteTime;
    ULONG ulActiveAlarms;
    ULONG ulAlarm;
    ULONG ulAlarmSubCode;
} SYSLOGMGM_ALARMLOG;

typedef struct
{
    ULONG ulAbsoluteTime;
} SYSLOGMGM_CLOCKLOG;

typedef struct
{
	SWORD swCode;
    UWORD uwSize;
} SYSLOGMGM_DATATMPIDENT;

//***************************************************************************
// Global functions

BOOL SysLogMgm_Init(void);

void SysLogMgm_PowerOnFail(void);

void SysLogMgm_PostAlarm(ULONG ulAlarmMask, ULONG ulAlarmSubCode, SBYTE bStore);

void SysLogMgm_ClearAlarms(SBYTE bFlag);

void SysLogMgm_ClearAlarmMask(ULONG ulAlarmMask);

void SysLogMgm_SetDisableAlarmMask(ULONG ulAlarmMask, UWORD uwKey);

#endif
