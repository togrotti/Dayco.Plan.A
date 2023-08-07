/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysLogData.c                                               */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Data collection for syslog                                 */
/*                                                                          */
/****************************************************************************/

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)


#include "common\CommonUtility.h"
#include "system\SysLogData.h"
#include "system\SystemStatus.h"
#include "common\BlockStorage.h"
#include "system\SysAppDataCodes.h"
#include "common\CommonDefines.h"
#include "Os.h"
#include "common\TaskScheduler.h"
#include "drive\MotorHandler.h"
#include "drive\ThermalModel.h"
#include "drive\EncoderManager.h"

//#include <xe167f.h>
#include <string.h>

//***************************************************************************
// Globals

SYSLOGDATA_ALARM  psSysLogDataAlarmTable[SYSLOGDATA_ALARMS_MAX_ENTRIES];

HPVOID hpvSysLogDataAlarmPtr;
UWORD uwSysLogDataAlarmTblLength;
const UWORD uwSysLogDataAlarmRecordLength=sizeof(SYSLOGDATA_ALARM);

ULONG ulSysLogDataLastAlarmTime;

//***************************************************************************
// Locals

//OS_CREATEMUTEX(tSysLogDataHistAccessMutex);
volatile OS_MUTEX tSysLogDataHistAccessMutex;

//***************************************************************************
// initialize

BOOL SysLogData_Init(void)
{
    uwSysLogDataAlarmTblLength=0;
    hpvSysLogDataAlarmPtr=psSysLogDataAlarmTable;
    ulSysLogDataLastAlarmTime=0l;

    OS_CREATEMUTEX(tSysLogDataHistAccessMutex);

    return TRUE;
}

//***************************************************************************
// restore data collection that come with clock log

BOOL SysLogData_RestoreClockLogData(HPVOID hpvBuf, SLONG slSize)
{
    SYSLOGDATA_OS_STATISTICS osstat;
    HPVOID blkptr;
    ULONG storagesize;
    SWORD blkcode;

        // then seek managed blocks
    storagesize=(ULONG)slSize;
	while((blkcode=blkstor_enumvalid(&hpvBuf, &storagesize, &blkptr)) != BLKSTOR_ERR_NOTFOUND)
        switch(blkcode)
        {
            case DATACODE_SYSLOG_OS_STATISTICS:
    		    blkstor_getdata(blkptr,0,DATACODE_SYSLOG_OS_STATISTICS,&osstat,sizeof(osstat));

#if (OS_MEASURESTACKSIZE)
                uwOsSysStackFree=osstat.uwOsSysStackFree;
                uwOsUsrStackFree=osstat.uwOsUsrStackFree;
#endif
                uwTaskSchedRTMaxTime=osstat.uwTaskSchedRTMaxTime;

                break;
        }

    return TRUE;
}

//***************************************************************************
// decode data and insert into alarm history ram table
// (both at reset or when alarm occur); newer are at top, time is checked
// when inserting to preserve right order

BOOL SysLogData_InsertIntoAlarmHistory(SYSLOGMGM_ALARMLOG  * psAlarmLog, HPVOID hpvBuf, SWORD swSize)
{
    SYSLOGDATA_ALARM  * psAlTb;
    SYSLOGDATA_ALARM_V1_DATA  * psAlV1Src;
    HPVOID blkptr;
    ULONG storagesize;
    SWORD blkcode;
    SWORD ct;

        // lock table
    assert(Os_MutexWait(&tSysLogDataHistAccessMutex,0));

        // insert new alarm preserving time order
    for(ct=SYSLOGDATA_ALARMS_MAX_ENTRIES-1;ct>0;ct--)
        if(psSysLogDataAlarmTable[ct-1].sSys.ulAbsoluteTime<=psAlarmLog->ulAbsoluteTime)
            psSysLogDataAlarmTable[ct]=psSysLogDataAlarmTable[ct-1];
        else
            break;
    psAlTb=&psSysLogDataAlarmTable[ct];

        // then update table length
    if(uwSysLogDataAlarmTblLength<SYSLOGDATA_ALARMS_MAX_ENTRIES)
        uwSysLogDataAlarmTblLength++;

        // zeroes the record
    memset(psAlTb,0,sizeof(*psAlTb));

        // store alarm definition
    psAlTb->sSys=*psAlarmLog;

        // then seek managed blocks
    storagesize=(ULONG)swSize;
	while((blkcode=blkstor_enumvalid(&hpvBuf, &storagesize, &blkptr)) != BLKSTOR_ERR_NOTFOUND)
        switch(blkcode)
        {
            case DATACODE_SYSLOG_ALARM_V1_DATA:
    		    blkstor_getaddr(blkptr,0,DATACODE_SYSLOG_ALARM_V1_DATA,&psAlV1Src);

                    // copy back in the new data structure and signal the variables that
                    // were stored with old data structure thus unavailable
                psAlTb->sData.slIuFb        =psAlV1Src->slIuFb;
                psAlTb->sData.slIvFb        =psAlV1Src->slIvFb;
                psAlTb->sData.slIwFb        =SYSLOGDATA_ENTRY_VOID;
                psAlTb->sData.slIuAvg       =psAlV1Src->slIuAvg;
                psAlTb->sData.slIvAvg       =psAlV1Src->slIvAvg;
                psAlTb->sData.slIqFb        =psAlV1Src->slIqFb;
                psAlTb->sData.slIdFb        =psAlV1Src->slIdFb;
                psAlTb->sData.swTJunction   =psAlV1Src->swTJunction;
                psAlTb->sData.swTHeatsink   =psAlV1Src->swTHeatsink;
                psAlTb->sData.slSpeed       =psAlV1Src->slSpeed;
                psAlTb->sData.swDCBus       =psAlV1Src->swDCBus;
                psAlTb->sData.ulBrakeEnergy =SYSLOGDATA_ENTRY_VOID;
                break;

            case DATACODE_SYSLOG_ALARM_DATA:
    		    blkstor_getdata(blkptr,0,DATACODE_SYSLOG_ALARM_DATA,&psAlTb->sData,sizeof(psAlTb->sData));
                break;
        }

        // set last alarm time
    atomic_write(&ulSysLogDataLastAlarmTime, &psAlTb->sSys.ulAbsoluteTime, sizeof(psAlTb->sSys.ulAbsoluteTime));

        // unlock table
    Os_MutexSignal(&tSysLogDataHistAccessMutex);

    return TRUE;
}

//***************************************************************************
// add system status data

SWORD SysLogData_PostAlarmData(HPUBYTE hpubBuf, SWORD swLeftSize, ULONG ulAlarmMask)
{
    SYSLOGMGM_DATATMPIDENT  * psIdent;
    SYSLOGDATA_ALARM_DATA  * psAlarmData;

        // allocate space for alarm data log
    psIdent=(SYSLOGMGM_DATATMPIDENT  *)hpubBuf;
    psAlarmData=(SYSLOGDATA_ALARM_DATA  *)&hpubBuf[sizeof(BLKSTOR_HEADER)];

        // then fill up data
    psIdent->swCode=DATACODE_SYSLOG_ALARM_DATA;
    psIdent->uwSize=sizeof(SYSLOGDATA_ALARM_DATA);

        // dummy
    psAlarmData->slIuFb=ulAlarmMask;

        // critical section for reading all alarm data
//    DISABLE_IRQ();

    psAlarmData->slIuFb=sMh_MotorDataOut.slIuFb;
    psAlarmData->slIvFb=sMh_MotorDataOut.slIvFb;
    psAlarmData->slIwFb=sMh_MotorDataOut.slIwFb;
    psAlarmData->slIuAvg=sTm_ThModOut.sOutValue.slIuPeakAvrg;
    psAlarmData->slIvAvg=sTm_ThModOut.sOutValue.slIvPeakAvrg;
    psAlarmData->slIqFb=sMh_MotorDataOut.slIqFb;
    psAlarmData->slIdFb=sMh_MotorDataOut.slIdFb;
    psAlarmData->swTJunction=sTm_ThModOut.sOutValue.swTjMax;
    psAlarmData->swTHeatsink=sTm_ThModOut.sOutValue.swTHeatSink;
    psAlarmData->slSpeed=sEm_Fbk2CLExt.slFilteredSpeed;
    psAlarmData->swDCBus=sMh_MotorDataOut.swDcBusValue;
    psAlarmData->ulBrakeEnergy=sTm_ThModOut.ulOutValueRBrakeEnergy;

//    RESTORE_IRQ();

    return swLeftSize-sizeof(BLKSTOR_HEADER)-sizeof(SYSLOGDATA_ALARM_DATA);
}

//***************************************************************************
// add system statistics data saved with global clock

SWORD SysLogData_PostClockData(HPUBYTE hpubBuf, SWORD swLeftSize)
{
    SYSLOGMGM_DATATMPIDENT  * psIdent;
    SYSLOGDATA_OS_STATISTICS  * psAlarmData;

        // allocate space for alarm data log
    psIdent=(SYSLOGMGM_DATATMPIDENT  *)hpubBuf;
    psAlarmData=(SYSLOGDATA_OS_STATISTICS  *)&hpubBuf[sizeof(BLKSTOR_HEADER)];

        // then fill up data
    psIdent->swCode=DATACODE_SYSLOG_OS_STATISTICS;
    psIdent->uwSize=sizeof(SYSLOGDATA_OS_STATISTICS);

        // collect statistics
#if (OS_MEASURESTACKSIZE)
    psAlarmData->uwOsSysStackFree=uwOsSysStackFree;
    psAlarmData->uwOsUsrStackFree=uwOsUsrStackFree;
#else
    psAlarmData->uwOsSysStackFree=0;
    psAlarmData->uwOsUsrStackFree=0;
#endif
    psAlarmData->uwTaskSchedRTMaxTime=uwTaskSchedRTMaxTime;

    return swLeftSize-sizeof(BLKSTOR_HEADER)-sizeof(SYSLOGDATA_OS_STATISTICS);
}

//***************************************************************************
// get from alarm history the base alarm info with post time immediately
// newer than specified, if nothing then return false

BOOL SysLogData_GetFromAlarmHistory(ULONG ulTime, SYSLOGMGM_ALARMLOG  * psLog)
{
    SWORD ct;
    BOOL retval=FALSE;

        // if table is empty then immediately exit
    if(uwSysLogDataAlarmTblLength==0)
        return FALSE;

        // lock table
    Os_MutexWait(&tSysLogDataHistAccessMutex,0);
    
    if(uwSysLogDataAlarmTblLength>0)
    {
            // start seeking from the oldest one
        for(ct=uwSysLogDataAlarmTblLength-1;ct>=0;ct--)
            if(psSysLogDataAlarmTable[ct].sSys.ulAbsoluteTime>=ulTime)
                break;

            // if found copy base alarm info
        if(ct>=0)
        {
            *psLog=psSysLogDataAlarmTable[ct].sSys;
            retval=TRUE;
        }
    }

        // unlock table
    Os_MutexSignal(&tSysLogDataHistAccessMutex);

    return retval;
}
