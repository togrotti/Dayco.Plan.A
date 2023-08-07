/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SyncManager.c                                              */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Main PWM and related linked RT re-synchronization manager  */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

#include <math.h>
#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonUtility.h"
#include "common\DspFunctions.h"
#include "common\TaskScheduler.h"
#include "SyncManager.h"
#include <stdlib.h>

//***************************************************************************
// Avoids warning C47: unreferenced parameter

//#pragma warning disable = 47

//****************************************************************************
// Macros

#define SYNCSTAT_REFRESHTIME        500        // msec

//****************************************************************************
// General global variables

SYNCMGR_PARAMS sSyncMgrParam;
SYNCMGR_OUT sSyncMgrDiagnosticOut;
#ifdef _APP_XC
UWORD uwSyncMgrAdjRTScaling;
#endif

//****************************************************************************
// Default parameters

const SYNCMGR_PARAMS  sSyncMgrDefParam=
{
    64000,
    110184,
    2000,
    24000,
    16,
    8192l,
};

//****************************************************************************
// Data structures

typedef struct
{
    SWORD swHiResTSScaling;
    SLONG slPrevFBusSP;
    SLONG slAvgFBusDelta;
    UWORD uwDiscard;
    UWORD uwCenteringValue;
    SLONG slPrecSyncSPDelta;
    UWORD uwMinNominal;
    UWORD uwMaxNominal;
    SLONG slMaxFBusDelta;
    SLONG slMinFBusDelta;
    UWORD uwSyncStatTimer;
    SBYTE bLocalValid;
} SYNCMGR_RUNTIME;

//****************************************************************************
// Locals

static SYNCMGR_RUNTIME sRunTime;

static BOOL newtsavailable(void);
static BOOL synchecking(void);
static void paramcheckandcalc(void);

//****************************************************************************
// Init entry point

BOOL SyncMgrInit(UWORD uwOption)
{
        // setup timer nominal value and scaling according to the configuration
#ifdef _APP_XC
    UWORD uwNominalValue;
#ifdef _INFINEON_
    if(uwSystemMainClockFreq==SYSTEMCLOCK_PERF)
    {
        uwNominalValue=SYNCMGR_NOMINALVALUE_PERF;
        uwSyncMgrAdjRTScaling = (UWORD)(16384l * SYNCMGR_NOMINALVALUE_PERF / SYNCMGR_NOMINALVALUE_COMP);
    }
    else
    {
        uwNominalValue=SYNCMGR_NOMINALVALUE_COMP;
        uwSyncMgrAdjRTScaling = 16384;
    }
#else
    uwNominalValue=SYNCMGR_NOMINALVALUE_TTC;
    uwSyncMgrAdjRTScaling = (UWORD)(16384l * SYNCMGR_NOMINALVALUE_TTC / SYNCMGR_NOMINALVALUE_COMP);
#endif // _infineon_
#else
#define uwNominalValue SYNCMGR_NOMINALVALUE_COMP
#endif
        // max +/-delta for correction around nominal value
        // (always setup for plc function)
    sRunTime.uwMinNominal=(SWORD)((FLOAT)uwNominalValue*(1.0-SYNCMGR_MAXSHIFT));
    sRunTime.uwMaxNominal=(SWORD)((FLOAT)uwNominalValue*(1.0+SYNCMGR_MAXSHIFT));

        // if system-wide not required then do nothing
    if(!bSysStatSyncMgrRequired)
        return TRUE;

        // hi res time scaling factor
    sRunTime.swHiResTSScaling=(SWORD)((FLOAT)8.0*32767.0/sRunTime.uwMaxNominal);

        // peak discard init counter
    sRunTime.uwDiscard=0;

        // setup sync stat timer
    sRunTime.uwSyncStatTimer=timer_settimeout(uwSysTimers1ms,SYNCSTAT_REFRESHTIME);

        // complete parameter check and calculation
    paramcheckandcalc();

        // add slow task for parameters management
    assert(TaskSched_AddBackgroundTask(&paramcheckandcalc));

        // add realtime task for new ts available
    assert(TaskSched_AddRTTask(&newtsavailable, TASKSCHEDULER_FLAG_NONE, \
            SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_FIELDBUS_TSRESYNC), \
            SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_FIELDBUS_TSRESYNC)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_RESETTING), 0));

        // add realtime task for sync checking (not run when previous is run)
    assert(TaskSched_AddRTTask(&synchecking, TASKSCHEDULER_FLAG_NONE, \
            0, \
            SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_FIELDBUS_TSRESYNC)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_RESETTING), 0));

    return TRUE;
}

//****************************************************************************
// Real time task called each time a new sync timestamp is available

static BOOL newtsavailable(void)
{
    UWORD uwActualReload;
//    UWORD uwfbusloc;
    SLONG slInstFBusDelta,slSamplePoint;

        // get actual timer reload value
    uwActualReload=SYNCMGR_FULLRELOAD_GET();

        // scale sample point to [0x0000:0xFFFF=0:125usec] and add absolute time
    slSamplePoint=(SLONG)(_sint32_mac_16_16(0l,(SWORD)(uwSysFieldbusSampleTS-uwActualReload),sRunTime.swHiResTSScaling)>>3);
    slSamplePoint+=(SLONG)((ULONG)uwSysFieldbusSampleRT<<16);

        // calc instantaneous sync time
    slInstFBusDelta=slSamplePoint-sRunTime.slPrevFBusSP;
    sRunTime.slPrevFBusSP=slSamplePoint;

        // record istantaneous sync period for diagnostic
    sSyncMgrDiagnosticOut.slSyncInstValue=slInstFBusDelta;

        // record min/max for statistics
    if(sRunTime.slMaxFBusDelta<slInstFBusDelta)
        sRunTime.slMaxFBusDelta=slInstFBusDelta;
    if(sRunTime.slMinFBusDelta>slInstFBusDelta)
        sRunTime.slMinFBusDelta=slInstFBusDelta;

        // check for peak discarding
    if(labs(slInstFBusDelta-sRunTime.slAvgFBusDelta)>=sSyncMgrParam.ulPeakThreshold)
    {
        if(sRunTime.uwDiscard>0)
        {
            sRunTime.uwDiscard--;

                // setup last average value
            slInstFBusDelta=sRunTime.slAvgFBusDelta;
                // and sample point is accumulated with same average value
            slSamplePoint=sSyncMgrDiagnosticOut.slLastSamplePoint+sRunTime.slAvgFBusDelta;
        }
    }
        // restore discard counter
    else if(sRunTime.uwDiscard<sSyncMgrParam.uwPeakNDiscard)
        sRunTime.uwDiscard++;
    else // check for sync validity
        sRunTime.bLocalValid=TRUE;

        // setup sample point (either real or accumulated)
    sSyncMgrDiagnosticOut.slLastSamplePoint=slSamplePoint;

        // filter incoming sync time
    sRunTime.slAvgFBusDelta=_sint32_scale_32(sRunTime.slAvgFBusDelta, (SLONG)sSyncMgrParam.uwTSFilter)+
                            _sint32_scale_32(slInstFBusDelta, (SLONG)(0x10000ul-(ULONG)sSyncMgrParam.uwTSFilter));

        // get sync sample point and offset with user parameter
    slSamplePoint=(SLONG)((SWORD)((sSyncMgrDiagnosticOut.slLastSamplePoint&0xffffl)-sRunTime.uwCenteringValue));

        // calculate out = Kp * in + Kd * (in - inprec) (scaling factors are Q15 instead of Q16)
    sSyncMgrDiagnosticOut.swCorrection=(SWORD)((_sint32_scale_32(slSamplePoint, (SLONG)sSyncMgrParam.swFiltKp)+
                                                _sint32_scale_32(slSamplePoint-sRunTime.slPrecSyncSPDelta, (SLONG)sSyncMgrParam.swFiltKd))<<1);
    sRunTime.slPrecSyncSPDelta=slSamplePoint;

        // check for corrections, if no then exit
    if(sSyncMgrDiagnosticOut.swCorrection==0)
        return TRUE;

        // otherwise apply correction, if in the valid range
    if(sSyncMgrDiagnosticOut.swCorrection<0l)
        if(uwActualReload<(UWORD)(0xffff-sRunTime.uwMinNominal))
            uwActualReload++;
        else
            return TRUE;
    else
        if(uwActualReload>(UWORD)(0xffff-sRunTime.uwMaxNominal))
            uwActualReload--;
        else
            return TRUE;

        // setup CC
    SYNCMGR_FULLRELOAD_SET(uwActualReload);
    SYNCMGR_HALFRELOAD_SET((SWORD)uwActualReload/2);

    return TRUE;
}

//****************************************************************************
// sync checking

BOOL synchecking(void)
{
        // if discard counter is zero then sync is not valid,
        // then immediately exit
    if(sRunTime.uwDiscard==0)
    {
        sRunTime.bLocalValid=FALSE;
        return TRUE;
    }

        // check that sync not arrived in last uwPeakNDiscard*slAvgFBusDelta
    if((SWORD)(sSyncMgrParam.uwPeakNDiscard*(UWORD)(sRunTime.slAvgFBusDelta>>16)+
               (UWORD)(sSyncMgrDiagnosticOut.slLastSamplePoint>>16)-uwSysFieldbusRTTimer)<0)
    {
        sRunTime.uwDiscard=0;
        sRunTime.bLocalValid=FALSE;
    }

    return TRUE;
}

//****************************************************************************
// Slow task for parameter checking and calculation

void paramcheckandcalc(void)
{
    static BOOL bLocRefresh=FALSE;
    FLOAT flcenterv;
    ULONG ulTmp;
    BOOL bLocalValid;

        // re-calculate centering value (if valid parameter)
    if(sSyncMgrParam.slReSyncDelta>=0l && sSyncMgrParam.slReSyncDelta<(SLONG)(1e9/REALTIME_TASK_FREQ))
    {
        flcenterv=(FLOAT)sSyncMgrParam.slReSyncDelta/1e9/(1.0/REALTIME_TASK_FREQ);
        sRunTime.uwCenteringValue=(UWORD)(flcenterv*65536.0)+32767;
    }

        // sync stat checking
    if(timer_istimedout(uwSysTimers1ms,sRunTime.uwSyncStatTimer))
    {
            // local copy
        bLocalValid=sRunTime.bLocalValid;

        if(bLocalValid)
            sRunTime.uwSyncStatTimer=timer_settimeout(uwSysTimers1ms,SYNCSTAT_REFRESHTIME);
        else
            sRunTime.uwSyncStatTimer=timer_settimeout(uwSysTimers1ms,1);

            // update only if valid, otherwise keep last
        if(bLocalValid && bLocRefresh)
        {
                // calculate conversion factor, based on the actual rt task period
            flcenterv=1.0/65536.0*1e9/REALTIME_TASK_FREQ*(UWORD)(0-SYNCMGR_FULLRELOAD_GET());
#ifdef _APP_XC
#ifdef _INFINEON_
            flcenterv/=uwSystemMainClockFreq==SYSTEMCLOCK_PERF?SYNCMGR_NOMINALVALUE_PERF:SYNCMGR_NOMINALVALUE_COMP;
#else
            flcenterv/=SYNCMGR_NOMINALVALUE_TTC;
#endif // _infineon_
#else
            flcenterv/=SYNCMGR_NOMINALVALUE_COMP;
#endif
                // get and convert max value
            atomic_read(&ulTmp, &sRunTime.slMaxFBusDelta, sizeof(ulTmp));
            ulTmp=(ULONG)(flcenterv*ulTmp);
            atomic_write(&sSyncMgrDiagnosticOut.ulSyncMax, &ulTmp, sizeof(ulTmp));

            atomic_read(&ulTmp, &sRunTime.slMinFBusDelta, sizeof(ulTmp));
            ulTmp=(ULONG)(flcenterv*ulTmp);
            atomic_write(&sSyncMgrDiagnosticOut.ulSyncMin, &ulTmp, sizeof(ulTmp));

            atomic_read(&ulTmp, &sRunTime.slAvgFBusDelta, sizeof(ulTmp));
            ulTmp=(ULONG)(flcenterv*ulTmp);
            atomic_write(&sSyncMgrDiagnosticOut.ulSyncTime, &ulTmp, sizeof(ulTmp));

                // reset min/max
            ulTmp=0l;
            atomic_write(&sRunTime.slMaxFBusDelta, &ulTmp, sizeof(ulTmp));
            ulTmp=SLONG_MAX_VALUE;
            atomic_write(&sRunTime.slMinFBusDelta, &ulTmp, sizeof(ulTmp));

            sSyncMgrDiagnosticOut.bValid=TRUE;
        }
        else
            sSyncMgrDiagnosticOut.bValid=bLocRefresh=FALSE;

            // check for first time
        if(bLocalValid && !bLocRefresh)
            bLocRefresh=TRUE;
    }
}

//****************************************************************************
// Setup RT period from plc application

BOOL SyncMgrAdjustRTPeriod(UWORD uwPeriod)
{
#ifdef _APP_XC
    uwPeriod=_sint16_mpy_16_q14(uwPeriod,uwSyncMgrAdjRTScaling);
#endif

    if(uwPeriod>=sRunTime.uwMinNominal && uwPeriod<=sRunTime.uwMaxNominal)
    {
        SYNCMGR_FULLRELOAD_SET(0-uwPeriod);
        SYNCMGR_HALFRELOAD_SET((SWORD)(0-uwPeriod)/2);

        return TRUE;
    }
    else
        return FALSE;
}
