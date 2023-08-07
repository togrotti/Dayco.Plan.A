/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysLogData.h                                               */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Data collection for syslog                                 */
/*                                                                          */
/****************************************************************************/

#ifndef _SYSLOGDATA_H
#define _SYSLOGDATA_H

#include "SysLogManagement.h"

//***************************************************************************
// Defines

#define SYSLOGDATA_ALARMS_MAX_ENTRIES               32

#define SYSLOGDATA_ENTRY_VOID                       0x80000000

//***************************************************************************
// Data structures

typedef struct
{
    UWORD uwOsSysStackFree;
    UWORD uwOsUsrStackFree;
    UWORD uwTaskSchedRTMaxTime;
} SYSLOGDATA_OS_STATISTICS;

typedef struct
{
    SLONG slIuFb;
    SLONG slIvFb;
    SLONG slIwFb;
    SLONG slIuAvg;
    SLONG slIvAvg;
    SLONG slIqFb;
    SLONG slIdFb;
    SWORD swTJunction;
    SWORD swTHeatsink;
    SLONG slSpeed;
    SWORD swDCBus;
    ULONG ulBrakeEnergy;
} SYSLOGDATA_ALARM_DATA;

typedef struct
{
    SYSLOGMGM_ALARMLOG sSys;
    SYSLOGDATA_ALARM_DATA sData;
} SYSLOGDATA_ALARM;

//***************************************************************************
// Old data structures

typedef struct
{
    SLONG slIuFb;
    SLONG slIvFb;
    SLONG slIuAvg;
    SLONG slIvAvg;
    SLONG slIqFb;
    SLONG slIdFb;
    SWORD swTJunction;
    SWORD swTHeatsink;
    SLONG slSpeed;
    SWORD swDCBus;
    SLONG slBrakingPower;
} SYSLOGDATA_ALARM_V1_DATA;

//***************************************************************************
// Globals

extern SYSLOGDATA_ALARM  psSysLogDataAlarmTable[SYSLOGDATA_ALARMS_MAX_ENTRIES];

extern HPVOID hpvSysLogDataAlarmPtr;
extern UWORD uwSysLogDataAlarmTblLength;
extern const UWORD uwSysLogDataAlarmRecordLength;

extern ULONG ulSysLogDataLastAlarmTime;

//***************************************************************************
// Globals functions

    // initializel
BOOL SysLogData_Init(void);

    // restore data collection that come with clock log
BOOL SysLogData_RestoreClockLogData(HPVOID hpvBuf, SLONG slSize);

    // decode data and insert into alarm history ram table
BOOL SysLogData_InsertIntoAlarmHistory(SYSLOGMGM_ALARMLOG  * psAlarmLog, HPVOID hpvBuf, SWORD swSize);

    // add system status data
SWORD SysLogData_PostAlarmData(HPUBYTE hpubBuf, SWORD swLeftSize, ULONG ulAlarmMask);

    // add system statistics data saved with global clock
SWORD SysLogData_PostClockData(HPUBYTE hpubBuf, SWORD swLeftSize);

    // get from alarm history the base alarm info
BOOL SysLogData_GetFromAlarmHistory(ULONG ulTime, SYSLOGMGM_ALARMLOG  * psLog);

#endif

