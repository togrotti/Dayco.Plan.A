/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : CanOpenCommandMgr.c                                        */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : CanOpen specific command manager                           */
/*                                                                          */
/****************************************************************************/
#include "system\SysAppConfig.h"
#if CFG_CANDRV_CMDMGR
#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "CanOpenCommandMgr.h"
#include "common\ParamStorageManagement.h"
#include "common\TaskScheduler.h"
#include "system\Os.h"
#include "core\SystemReset.h"
#include "system\SystemStatus.h"
#include "system\SystemAlarms.h"
#include "system\SysLogManagement.h"
#include "MultiCANController.h"
#include "bus\canopen\CanOpenDs301.h"
#include "CanOpenDs301Externals.h"
#include "CanOpenComDB.h"
#include "CanOpenPdoSyncMgr.h"
#include "CanOpenCOECommon.h"
#include "drive\DriveTaskController.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "system\GlobalResetCodes.h"
#include "drive\HardwareUnitID.h"

//***************************************************************************
// Avoids warning C37: nonstandard extension - unnamed parameter

//#pragma warning disable = 37

//***************************************************************************
// Globals
#pragma GCC optimize (2)
    // Parameters
CANOPENCM_PARAM tCanOpenCMParams;
const CANOPENCM_PARAM tCanOpenCMDefParams=
{
	-1, 0l, {TRUE, TRUE, FALSE, FALSE},//-1, 0l, {1, 1},
};

    // external emcy handler
CANOPENCM_IN sCanOpenCMIn;

    // exported status
CANOPENCM_OUT sCanOpenCMOut;

    // alarm clearing from RT task
BOOL bCanOpenCMFaultClear=FALSE;

//***************************************************************************
// Defines

#define SLOWTASK_STACKSIZE                      (OS_DEFAULTUSRSTATICSTACK+0x00D0)

//***************************************************************************
// Locals

static BOOL bCanModuleDisabled=FALSE;
static ULONG ulAlarmSubCodesActive=0l;
static ULONG ulAlarmSubCodesToBeSignaled=0l;

static ULONG ulGetAlEntryActTime;
static ULONG ulGetAlEntryAlrmStat=0ul;

//***************************************************************************
// Local functions prototypes

static void dummyfunction(void) {}
static UWORD getsystemstatus(void);
static void nmtevent(UBYTE);
static BOOL getalarmentrywrapper(ULONG *, UWORD *, UBYTE *);
static void commfaultsignal(UWORD);
static BOOL lssexecstore(void);
static ULONG sdodispatchwrapper(DS301_SDOTRANSACTION * ptSdoStatus);

static void slowtaskmanager(void);

//***************************************************************************
// Callback definition for standard DS301 handler

const DS301_CALLBACKS tDs301CallBacks=
{
    &sdodispatchwrapper,
    &dummyfunction,
    &getsystemstatus,
    &nmtevent,
    &getalarmentrywrapper,
    &commfaultsignal,
    &lssexecstore,
    &CanOpenPSM_RT_CobSyncEvent,
    &CanOpenPSM_CreatePdoEvent,
    &CanOpenPSM_DestroyPdoEvent,
    &CanOpenPSM_SetDefaultCobID,
};

//****************************************************************************
// Init handler

BOOL CanOpenCM_Init(UWORD uwOption)
{
    UBYTE errorcode;

        // if disabled do nothing
    if(bCanModuleDisabled)
        return TRUE;

    switch(uwOption)
    {
        case CANOPENCM_INIT_CORE:
                // check cancontroller in the range of supported
            if(tCanOpenCMParams.swCanController<0 || tCanOpenCMParams.swCanController>=CANDRV_MAX_CANNODE)
            {
                bCanModuleDisabled=TRUE;
                return TRUE;
            }

                // initialize local
            atomic_read(&ulGetAlEntryActTime, &ulSysTimersTotalPowerOnTime, sizeof(ulSysTimersTotalPowerOnTime));
        
                // initialize DS301
            if(!ds301_init(tCanOpenCMParams.swCanController,&errorcode))
            {
                switch(errorcode)
                {
                    case DS301_INITERR_WRONGBAUDRATE:
                        ParChk_SignalValueError(PARCC_CANOPEN_WRONGBAUDRATE);
                        break;

                    case DS301_INITERR_WRONGNODEID:
                        ParChk_SignalValueError(PARCC_CANOPEN_WRONGNODEID);
                        break;
                }
                return FALSE;
            }
        
                // add slow task
            if(!Os_TaskCreateEx(&slowtaskmanager,SLOWTASK_STACKSIZE,"CAN_MGR"))
                return FALSE;

            break;

        case CANOPENCM_INIT_SYNCEVENT:
                // install realtime task for sync event
            if(!TaskSched_AddRTTask(&CanOpenPSM_RT_SyncEvent, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0))
                return FALSE;

            break;

        case CANOPENCM_INIT_SYNCPDOEVENT:
            break;

        default:
            assert(FALSE);
    }
        
        // initialize PDO mgr
    return CanOpenPSM_Init(uwOption, tCanOpenCMParams.flags.f.bReSyncEnable,
                           tCanOpenCMParams.flags.f.bSyncEvOnPeriodicCob, tCanOpenCMParams.flags.f.bDelayedInputs);
}

//***************************************************************************
// Hardware option callback

#ifdef _APP_XC
void CanOpenCM_GetHwOpt(UWORD can, ULONG * pulHwOpt, ULONG * pulFPGAOpt)
{
    if(tCanOpenCMParams.swCanController==0)
        *pulHwOpt  |=HWUNITID_CAN_PORT_0;

    if(tCanOpenCMParams.swCanController==1)
        *pulHwOpt  |=HWUNITID_CAN_PORT_1;

    *pulFPGAOpt |=0;
}
#endif

//***************************************************************************
// task for canopen protocol and command processing

static void slowtaskmanager(void)
{
    for(;;)
    {
        ULONG ulToBeSignaled;
        ULONG ulActive;

            // main loop handler
        ds301_loop();

            // pdo loop handler
        CanOpenPSM_Loop();

            // local fault check
        atomic_read(&ulToBeSignaled, &ulAlarmSubCodesToBeSignaled, sizeof(ULONG)); 
        atomic_read(&ulActive, &ulAlarmSubCodesActive, sizeof(ULONG)); 

            // select difference
        ulToBeSignaled = (ulToBeSignaled ^ ulActive) & ulToBeSignaled;

            // if new codes
        if(ulToBeSignaled)
        {
                // check for disable alarm mask
            if((ulToBeSignaled&tCanOpenCMParams.ulDisableCanAlarmMask)==0l)
            {
                (*sCanOpenCMIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL);
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_MN_CAN_FAIL, ulToBeSignaled, FALSE);
            }

            atomic_long_set_bits(&ulAlarmSubCodesActive, ulToBeSignaled);
        }

            // parameters check
        if(ds301_check_node_id(tDs301Param.bLssNodeId))
            ParChk_ResetValueError(PARCC_CANOPEN_WRONGNODEID);
        else
            ParChk_SignalValueError(PARCC_CANOPEN_WRONGNODEID);

        if(candrv_testspeed(tDs301Param.bLssTimingIndex))
            ParChk_ResetValueError(PARCC_CANOPEN_WRONGBAUDRATE);
        else
            ParChk_SignalValueError(PARCC_CANOPEN_WRONGBAUDRATE);

            // export NMT status
        sCanOpenCMOut.ubNMTStatus=ds301_get_nmt_state();

            // system alarm clearing request
        if(bCanOpenCMFaultClear)
        {
            ds301_faultclear();
            bCanOpenCMFaultClear=FALSE;
        }

            // yield control to OS
        Os_Sleep(0);
    }
}

//***************************************************************************
// Return system status for CanOpen handlers

static UWORD getsystemstatus(void)
{
    if(bSysStatBooting || bSysStatResetting)
        return DS301_NOTOPERATIVE;
    else
        return DS301_FULLYOPERATIVE;
}

//***************************************************************************
// NMT event

static void nmtevent(UBYTE ubEvent)
{
    if(!tCanOpenCMParams.flags.f.bDisableNmtResetNode && ubEvent==DS301_NMT_RESET_NODE)
    {
            // resetting flag and syslog flush flag
        bSysStatResetting=TRUE;
        bSysStatSysLogFlush=TRUE;

            // Wait system minimum delay
        timer_wait(uwOsFreeRunTimer1kHz, SW_RESET_DELAY);

            // wait until syslog is safely written
        while(bSysStatSysLogFlush);

            // then reset
#ifdef _APP_XC
        if(sCanOpenCOEReqReflash)
            SysRes_ExecuteReset(RESETCODE_ENTERBOOTANDUPGRADE, ~RESETCODE_ENTERBOOTANDUPGRADE);
        else
#endif
            SysRes_ExecuteReset(SYSRES_POWERON_VALUE_PAR0, SYSRES_POWERON_VALUE_PAR1);
    }
}

//***************************************************************************
// Return next alarm entry if any

static BOOL getalarmentrywrapper(ULONG * pulAlarm, UWORD * puwAlarmSubCode, UBYTE * pubAlarmClass)
{
    return CanOpenCOE_GetAlarmEntry(&ulGetAlEntryActTime, pulAlarm, puwAlarmSubCode, pubAlarmClass, &ulGetAlEntryAlrmStat);
}

//***************************************************************************
// Get faults from CAN/CANOpen and signal to core system

static void commfaultsignal(UWORD uwCode)
{
        // zero means comm alarm clears
    if(uwCode==0)
    {
        ULONG ulZero=0l;

        atomic_write(&ulAlarmSubCodesToBeSignaled, &ulZero, sizeof(ULONG));
        atomic_write(&ulAlarmSubCodesActive, &ulZero, sizeof(ULONG));
    }

        // if not yet active
    else if((UWORD)(ulAlarmSubCodesActive&uwCode)!=uwCode)
    {
            // check for disable alarm mask
        if((((ULONG)uwCode)&tCanOpenCMParams.ulDisableCanAlarmMask)==0l)
        {
            (*sCanOpenCMIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL);
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_MN_CAN_FAIL, (ULONG)uwCode, FALSE);
        }
        atomic_long_set_bits(&ulAlarmSubCodesActive, (ULONG)uwCode);
    }
}

//***************************************************************************
// Get faults from machine specific canopen implementation
// and signal to core system

void CanOpenCM_CanFaultSignal(ULONG ulCode)
{
    atomic_long_set_bits(&ulAlarmSubCodesToBeSignaled, ulCode);
}

//***************************************************************************
// Store LSS parameters, just exec a global parameters save

static BOOL lssexecstore(void)
{
    if(parmgm_par_save()!=PARMGM_B_OK)
    {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_HW_FLASH_FAIL, SYSTEMALARMS_SUBCODE_HF_FLASH_PARAMS, FALSE);
        return FALSE;
    }
    else
        return TRUE;
}

//***************************************************************************
// Wrapper for SDO transaction

static ULONG sdodispatchwrapper(DS301_SDOTRANSACTION * ptSdoStatus)
{
    return CanOpenComDBSdoDispatcher(ptSdoStatus, CANOPENCOMDB_F_DS301_VALID);
}
#endif
