/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2014, Phase Motion Control. All Rights Reserved.             */
/*                                                                          */
/* File        : HiperfaceEncoder.c                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Hiperface encoder manager                                  */
/*                                                                          */
/****************************************************************************/

#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonDefines.h"
#include "system\SysLogManagement.h"
#include "system\SystemStatus.h"
#include "system\Os.h"

#include "common\CommonEncoderManager.h"
#include "drive\HiperfaceEncoder.h"
#include "drive\HiperfaceHandlers.h"

#include "common\TaskScheduler.h"
#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"

#include "common\Int64Functions.h"
#include "fpga\FpgaHandler.h"

#ifdef _INFINEON_
//***************************************************************************
// Avoids warning C37: nonstandard extension - unnamed parameter

#pragma warning disable = 37
#else
/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)
#endif
//****************************************************************************
// General global variables

HIPFC_PARAMS sHf_Params;

//****************************************************************************
// Default parameters

#ifdef _INFINEON_
const HIPFC_PARAMS huge sHf_DefParams=
#else
const HIPFC_PARAMS      sHf_DefParams=
#endif
{
    0xFFFFFFFFul
};

#if CFG_ENC_HIP
//****************************************************************************
// Data structures

typedef struct {
    union {
        struct {
            UWORD bAlarm        : 1;
        } b ;
        UWORD w ;
    } sErrorSent ;

    union {
        struct {
            UWORD bInitDone     : 1;
            UWORD bValid        : 1;
            UWORD bInitReq      : 1;
            UWORD bFirstRun     : 1;
            UWORD bEnableFeed   : 1;
            UWORD bEnableSetPos : 1;
        } b ;
        UWORD w ;
    } sCmd ;

    UWORD uwMotPolarPairs;

    SQWRD sqAbsMechAbs;
    UWORD uwAbsElecAngle;

    ENCMGR_SPACEFEEDBACK * psFeedbackOut;

    SBYTE * psbEncMgrFault;

    ULONG ulMTurnStartPos;

    UBYTE pubSetPos[4];
  
} HIPFC_RUNTIME ;

//****************************************************************************
// Locals

#ifdef _INFINEON_
static BOOL bdata               bSlowTaskInstalled=FALSE;
static volatile HIPFC_RUNTIME sdata sRuntime;
static HIPFC_WORKS huge         sWorks;
#else
static BOOL                   bSlowTaskInstalled=FALSE;
static volatile HIPFC_RUNTIME sRuntime;
static HIPFC_WORKS            sWorks;
#endif

//****************************************************************************
// Local functions

static BOOL coreinit(BOOL);
static BOOL hipfchandlerinit(void);
static BOOL initruntime(void);
static BOOL hipfcinit(void);
static BOOL task8kHz(void);
static void slowtask(void);

//***************************************************************************
// Initialization entry point

BOOL Hf_Init(ENCMGR_SPACEFEEDBACK * psFeedback, SBYTE * psbEncMgrFault, ULONG ulRTDisMask)
{
        // if wrong parameters
    if(!Hf_ParametersCheck())
        return FALSE;

        // setup runtime
    sRuntime.psFeedbackOut=psFeedback;
    sRuntime.psbEncMgrFault=psbEncMgrFault;
    sRuntime.ulMTurnStartPos=sHf_Params.ulMTurnStartPos;
    sRuntime.uwMotPolarPairs=sGlbMotorParameters.uwPoleNumbers/2;

        // if not yet done init encoder
    if(!sRuntime.sCmd.b.bInitDone)
    {
            // core setup
        if(!coreinit(TRUE))
            return FALSE;
    
            // Initialize Encoder
        if(!hipfcinit())
        {
            coreinit(FALSE);
            return FALSE;
        }
    }
        // otherwise only init runtime
    else
    {
        if(!initruntime())
            return FALSE;
    }

        // if not yet, install slowtask for protocol management
    if(!bSlowTaskInstalled)
    {
        if(!TaskSched_AddBackgroundTask(&slowtask))
            return FALSE;

        bSlowTaskInstalled=TRUE;
    }

        // install rt task       
    if(!TaskSched_AddRTTaskEx((BOOL (*)(UWORD))&task8kHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|ulRTDisMask, 0, NULL))
        return FALSE;

        // module valid
    sRuntime.sCmd.b.bValid=TRUE;

    return TRUE;
}

//***************************************************************************
// core options

static BOOL coreinit(BOOL bEnable)
{
    FPGA_ENCODER_OPTIONS_HIPFC = bEnable;
    return TRUE;
}

//***************************************************************************
// init or re-init unit

static BOOL hipfchandlerinit(void)
{
    UWORD uwRetVal;
    
        // Initialize Hiperface encoder interface
    uwRetVal=HipfcInitialization(FPGA_HIPERFACE_BASEADDR, HIPFC_DEF_BAUDRATE, HIPFC_DEF_PARITY, &sWorks);
    if(uwRetVal)
    {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_HIPERFACE_FAIL, SYSTEMALARMS_SUBCODE_HIPFC_INIT | ((ULONG)uwRetVal<<16), FALSE);
        return FALSE ;
    }

        // set init done if full init was done
    sRuntime.sCmd.b.bInitDone=TRUE;

    return TRUE;
}

//***************************************************************************
// init runtime

static BOOL initruntime(void)
{
        // init
    INT64_ASSIGN(sRuntime.psFeedbackOut->sEncData.sqPostn,  0L, 0UL) ;
    INT64_ASSIGN(sRuntime.psFeedbackOut->sqMechAbsPosOffset,  0L, 0UL) ;
    INT64_ASSIGN(sRuntime.sqAbsMechAbs,  0L, 0UL) ;
    sRuntime.uwAbsElecAngle=0;
    sRuntime.sCmd.b.bFirstRun=TRUE;

        // Feedback
    sRuntime.psFeedbackOut->uwElecAngle = 0 ;    
    sRuntime.psFeedbackOut->swElecSpeed = 0 ;  
    sRuntime.psFeedbackOut->uwDeltaElecAngle = 0 ;    
    sRuntime.psFeedbackOut->sEncData.slSpeed = 0L ;
    sRuntime.psFeedbackOut->sEncData.slAccel = 0L ;
    sRuntime.psFeedbackOut->ubStatus = ENCMGR_NULL ;

    return TRUE;
}

//***************************************************************************
// init or re-init unit

static BOOL hipfcinit(void)
{
        // hiperface handler init
    if(!hipfchandlerinit())
    {
        sRuntime.psFeedbackOut->ubStatus = ENCMGR_FATAL_FAULT ; 
        return FALSE;
    }

    return initruntime();
}

//***************************************************************************
// Parameters check

BOOL Hf_ParametersCheck()
{
    return TRUE;
}

//***************************************************************************
// Electrical Angle Setup

void Hf_SetElecAngle(UWORD uwElecAngle)
{
    sRuntime.psFeedbackOut->uwDeltaElecAngle = uwElecAngle - sRuntime.uwAbsElecAngle - sGlbMotorParameters.uwPhaseOffset;
        // signal feed only if already initialized
    sRuntime.sCmd.b.bEnableFeed=!sRuntime.sCmd.b.bFirstRun;
}

//***************************************************************************
// fast task

static BOOL task8kHz(void)
{
    SQWRD sqLocPos;

    if(bSysStatAlarmsReset)
    {
            // if was faulty, then reset status
        if(sRuntime.sErrorSent.w || *sRuntime.psbEncMgrFault)
            sRuntime.sCmd.b.bInitReq = TRUE;
        
        sRuntime.sErrorSent.w = 0 ; /* clear errors */
    }
    
        // if init in progress and/or parametrization in progress, then do nothing and nullify data
    if(sRuntime.sCmd.b.bInitReq)
    {
        sRuntime.psFeedbackOut->ubStatus = ENCMGR_NULL;
        return TRUE;
    }

        // when signaled then calculate and send feedback
    if(sRuntime.sCmd.b.bEnableFeed)
    {
            // if first run then calculate absolute position offset
        if(sRuntime.sCmd.b.bFirstRun)
        {
            INT64_COPY(sqLocPos, sRuntime.sqAbsMechAbs);
    
                // check startup position for multiturns encoders
            if(sWorks.uwRevolutionNumber)
                    // if startup turns pos is above threshold, then extend sign
                if((ULONG)sqLocPos.hi >= sRuntime.ulMTurnStartPos)
                    sqLocPos.hi=~((ULONG)sWorks.uwRevolutionNumber-1)| \
                                 (sqLocPos.hi & ((ULONG)sWorks.uwRevolutionNumber-1));
    
                // setup absolute mech position            
            INT64_COPY(sRuntime.psFeedbackOut->sqMechAbsPosOffset, sqLocPos);
        }

            // signal as snapshot, encoder manager will reset all after consuming it
        sRuntime.psFeedbackOut->ubStatus = ENCMGR_ABSMECHTURN_VALID | ENCMGR_RELATIVE_VALID | ENCMGR_ELE_ANGLE_VALID | ENCMGR_SNAPSHOT_VALID;

            // reset flags
        sRuntime.sCmd.b.bFirstRun=FALSE;
        sRuntime.sCmd.b.bEnableFeed=FALSE;
    }

    return TRUE;
}

//***************************************************************************
// slow task

static void slowtask(void)
{
    UWORD uwRetVal;
    SQWRD sqLocPos;
    UBYTE pubPos[4];

        // if module not valid then exit
    if(!sRuntime.sCmd.b.bValid)
        return;

        // check for re-init request
    if(sRuntime.sCmd.b.bInitReq)
    {
        
        hipfcinit();
            // yield other tasks in order to let module get fresh data
        Os_Sleep(0);
        sRuntime.sCmd.b.bInitReq=FALSE;
    }

        // if already consumed then refresh position
    if(!sRuntime.sCmd.b.bEnableFeed && !(sRuntime.psFeedbackOut->ubStatus&ENCMGR_SNAPSHOT_VALID))
    {
            // get position
        uwRetVal=HipfcReadPosition(&sWorks, pubPos);
        if(uwRetVal)
        {
            if(!sRuntime.sErrorSent.b.bAlarm)
            {
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_HIPERFACE_FAIL, SYSTEMALARMS_SUBCODE_HIPFC_ALARM | (ULONG)uwRetVal<<16, FALSE);
    
                sRuntime.psFeedbackOut->ubStatus=ENCMGR_FATAL_FAULT;
                sRuntime.sErrorSent.b.bAlarm=TRUE;
            }
        }
            // got position without errors
        else
        {
                // convert to internal format
                // hiperface supported encoders format is 15LSB for mechanical angle,
                // if multiturn then 12bits more are used for multiturn
            INT64_ASSIGN(sqLocPos, 0, 0);
            INT64_MLONG(sqLocPos)=*((ULONG *)pubPos);
            _sint64_shl(&sqLocPos, sWorks.sbPosShift);

                // if first run set absolute positions
            if(sRuntime.sCmd.b.bFirstRun)
                INT64_COPY(sRuntime.sqAbsMechAbs,sqLocPos);
    
                // calculate relative position, only for monitoring purpose
            _sint64_sub(&sRuntime.psFeedbackOut->sEncData.sqPostn, &sqLocPos, &sRuntime.sqAbsMechAbs) ;
    
                // calculate electrical angle
            sRuntime.uwAbsElecAngle =(UWORD)((ULONG)HIWORD(sqLocPos.lo)*sRuntime.uwMotPolarPairs)+
                                        sRuntime.psFeedbackOut->uwDeltaElecAngle+sGlbMotorParameters.uwPhaseOffset;
            sRuntime.psFeedbackOut->uwElecAngle=sRuntime.uwAbsElecAngle;

                // Current Electrical Speed  = MechSpeed * MotorPolePairs */
            sRuntime.psFeedbackOut->swElecSpeed = (SWORD)_sint32_scale_32(sRuntime.psFeedbackOut->sEncData.slSpeed, sRuntime.uwMotPolarPairs) ;

                // and signal to fast task
            sRuntime.sCmd.b.bEnableFeed=TRUE;
        }
    }

        // process set position when requested
    if(sRuntime.sCmd.b.bEnableSetPos)
    {
        uwRetVal=HipfcSetPosition(&sWorks, sRuntime.pubSetPos);
        *((UWORD *)sRuntime.pubSetPos)=uwRetVal;
        sRuntime.sCmd.b.bEnableSetPos=FALSE;
    }
}

//***************************************************************************
// Set position into non-volatile encoder memory

#ifdef _INFINEON_
UWORD PlcHipfcSetPosition(UWORD uwKey, SQWRD huge * hpsqPos)
#else
UWORD PlcHipfcSetPosition(UWORD uwKey, SQWRD      * hpsqPos)
#endif
{
    SQWRD sqPos;

        // if module not valid then exit
    if(!sRuntime.sCmd.b.bValid)
        return HIPFC_NOTAVAILABLE;

        // check key
    if(uwKey!=HIPFC_PLC_SETPOS_KEY)
        return HIPFC_NOTAVAILABLE;

        // check right drive state
    if(!Os_IsInBackground())
        return HIPFC_NOTAVAILABLE;

        // if already running
    if(sRuntime.sCmd.b.bEnableSetPos)
        return HIPFC_NOTAVAILABLE;

        // convert position to hiperface format
    INT64_ASSIGN(sqPos, hpsqPos->hi, hpsqPos->lo);
    _sint64_shr(&sqPos, sWorks.sbPosShift);
    *((ULONG *)sRuntime.pubSetPos)=INT64_MLONG(sqPos);
    
        // signal write to encoder slow task
    sRuntime.sCmd.b.bEnableSetPos=TRUE;

        // wait for operation
    while(sRuntime.sCmd.b.bEnableSetPos);
        //$TEST$ os_sleep? wdt_clear?

        // return code is on the first two bytes
    return *((UWORD *)sRuntime.pubSetPos);
}
#endif
