/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : TAMAGAWAEncoder.c                                          */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : TAMAGAWA serial interface encoder                          */
/*                                                                          */
/****************************************************************************/

#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonDefines.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "system\SysLogManagement.h"
#include "system\SystemStatus.h"
#include "system\Os.h"

#include "common\CommonEncoderManager.h"
#include "drive\TAMAGAWAHandlers.h"
#include "drive\TAMAGAWAEncoder.h"

#include "common\TaskScheduler.h"
#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"

#include "common\Int64Functions.h"
#include "common\DspFunctions.h"
#include "fpga\FpgaHandler.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

//****************************************************************************
// General global variables

TMGW_PARAMS sTm_Params[TMGW_MAX_ENCODER];
TMGW_OUT sTm_DataOut[TMGW_MAX_ENCODER];

//****************************************************************************
// Default parameters

#ifdef _INFINEON_
const TMGW_PARAMS huge sTm_DefParams=
#else
const TMGW_PARAMS sTm_DefParams=
#endif
{
    {0,0}, 0xFFFFFFFFul
};

#if CFG_ENC_TMGW
//****************************************************************************
// Defines

//****************************************************************************
// Data structures

typedef struct {
    union {
        struct {
            UWORD bAlarm        : 1 ;
            UWORD bCrc          : 1 ;
            UWORD bOverTime     : 1 ;
            UWORD bWarn         : 1 ;
        } b ;
        UWORD w ;
    } sErrorSent ;

    union {
        struct {
            UWORD bInitDone     : 1;
            UWORD bValid        : 1;
            UWORD bInitReq      : 1;
            UWORD bPlcReq       : 1;
            UWORD bPlcReqStat   : 1;
            UWORD bDisAlarm     : 1;
        } b ;
        UWORD w ;
    } sCmd ;

    UWORD uwMotPolarPairs  ;

    SQWRD sqMechAbsPosition ;
    SQWRD sqMechAbsInitPos  ;
    UWORD uwAbsElecAngle;
    
    TMGW_OUT * psDataOut;
    ENCMGR_SPACEFEEDBACK * psFeedbackOut;

    UWORD uwChannelSel;

    SBYTE * psbEncMgrFault;

    TMGW_WORKS * psWorks;
    ULONG ulMTurnStartPos;
    ULONG ulRevMask;
    
    SQWRD sqAbsMechAbs;
      
} TMGW_RUNTIME ;

//****************************************************************************
// Locals

#ifdef _INFINEON_
static BOOL bdata bSlowTaskInstalled=FALSE;
static volatile TMGW_RUNTIME sdata sRuntime[TMGW_MAX_ENCODER];
#else
static BOOL bSlowTaskInstalled=FALSE;
static volatile TMGW_RUNTIME sRuntime[TMGW_MAX_ENCODER];
#endif

static TMGW_WORKS sWorks[TMGW_MAX_ENCODER];

static ULONG ulBaseAddress[TMGW_MAX_ENCODER]=
{
    FPGA_MTMGW_BASEADDR,
};

static ULONG ulAlarmSubCode[TMGW_MAX_ENCODER]=
{
    SYSTEMALARMS_SUBCODE_ENC_MAIN,
};

//****************************************************************************
// Local functions

static BOOL coreinit(UWORD,BOOL);
static BOOL TMGWhandlerinit(UWORD,BOOL);
static BOOL initruntime(UWORD);
static BOOL TMGWinit(UWORD,BOOL);
static void slowtask(void);
static BOOL task8kHz(TMGW_RUNTIME *);

//***************************************************************************
// Initialization entry point

BOOL Tm_Init(UWORD uwChannelSel, ENCMGR_SPACEFEEDBACK * psFeedback, SBYTE * psbEncMgrFault, ULONG ulRTDisMask)
{
        // if wrong parameters
    if(!Tm_ParametersCheck(uwChannelSel))
        return FALSE;

    sRuntime[uwChannelSel].psDataOut=&sTm_DataOut[uwChannelSel];
    sRuntime[uwChannelSel].psFeedbackOut=psFeedback;
    sRuntime[uwChannelSel].uwChannelSel=uwChannelSel;
    sRuntime[uwChannelSel].uwMotPolarPairs = sGlbMotorParameters.uwPoleNumbers / 2 ;
    sRuntime[uwChannelSel].psbEncMgrFault = psbEncMgrFault;
    sRuntime[uwChannelSel].psWorks = &sWorks[uwChannelSel];
    sRuntime[uwChannelSel].ulMTurnStartPos=sTm_Params[uwChannelSel].ulMTurnStartPos;
    sRuntime[uwChannelSel].psWorks->ubDisMTData=(UBYTE)sTm_Params[uwChannelSel].flags.b.bDisMTData;

       // if not yet done init encoder
    if(!sRuntime[uwChannelSel].sCmd.b.bInitDone)
    {
            // core setup
        if(!coreinit(uwChannelSel, TRUE))
            return FALSE;
    
            // Initialize TMGW Encoder
        if(!TMGWinit(uwChannelSel, TRUE))
        {
            coreinit(uwChannelSel, FALSE);
            return FALSE;
        }
    }
        // otherwise only init runtime
    else
        if(!initruntime(uwChannelSel))
            return FALSE;

         // if not yet, install slowtask for protocol management
    if(!bSlowTaskInstalled)
    {
        if(!TaskSched_AddBackgroundTask(&slowtask))
            return FALSE;

        bSlowTaskInstalled=TRUE;
    }

        // install rt task       
    if(!TaskSched_AddRTTaskEx((BOOL (*)(UWORD))&task8kHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|ulRTDisMask, 0, (UWORD)&sRuntime[uwChannelSel]))
        return FALSE;

        // module valid
    sRuntime[uwChannelSel].sCmd.b.bValid=TRUE;
    
    return TRUE;
}

//***************************************************************************
// Parameters check

BOOL Tm_ParametersCheck(UWORD uwChannelSel)
{
    assert(uwChannelSel<TMGW_MAX_ENCODER);
    return TRUE;
}

//***************************************************************************
// Electrical Angle Setup (only for main encoder)

void Tm_SetElecAngle(UWORD uwElecAngle)
{
    sRuntime[TMGW_SEL_MAIN].psFeedbackOut->uwDeltaElecAngle = uwElecAngle - sRuntime[TMGW_SEL_MAIN].uwAbsElecAngle - sGlbMotorParameters.uwPhaseOffset;
    sRuntime[TMGW_SEL_MAIN].psFeedbackOut->ubStatus        |= ENCMGR_ELE_ANGLE_VALID ;
}

//***************************************************************************
// core options

static BOOL coreinit(UWORD uwChannelSel,BOOL bEnable)
{
    switch(uwChannelSel)
    {
        case TMGW_SEL_MAIN:
            FPGA_ENCODER_OPTIONS_TMGW = bEnable;
            break;

        default:
            assert(FALSE);
    }
    return TRUE;
}

//***************************************************************************
// init or re-init unit

static BOOL TMGWhandlerinit(UWORD uwChannelSel, BOOL bFullInit)
{
    ULONG ulRetVal;
    
        // Initialize Hiperface encoder interface
    ulRetVal=TMGWInitialization(ulBaseAddress[uwChannelSel],&sWorks[uwChannelSel],bFullInit);
    if(ulRetVal)
    {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_TMGW_FAIL, SYSTEMALARMS_SUBCODE_TMGW_INIT|ulRetVal, FALSE);
        return FALSE ;
    }

        // set init done if full init was done
    sRuntime[uwChannelSel].sCmd.b.bInitDone=TRUE;

    return TRUE;
}

//***************************************************************************
// init runtime

static BOOL initruntime(UWORD uwChannelSel)
{
    UBYTE ubRevNumBits;
    
        // Setup diagnostics
    sTm_DataOut[uwChannelSel].ubBaudrate    = sWorks[uwChannelSel].ubBaudrate;
    sTm_DataOut[uwChannelSel].ubMultiTurn   = sWorks[uwChannelSel].ubMultiTurn;
    sTm_DataOut[uwChannelSel].ubResolution  = sWorks[uwChannelSel].ubResolution;

        // init everything from zero (the initial angle will be found during first servo)
    INT64_ASSIGN(sRuntime[uwChannelSel].psFeedbackOut->sEncData.sqPostn,  0L, 0UL) ;
    INT64_ASSIGN(sRuntime[uwChannelSel].psFeedbackOut->sqMechAbsPosOffset,  0L, 0UL) ;
    
    INT64_ASSIGN(sRuntime[uwChannelSel].sqMechAbsInitPos,  0L, 0UL) ;
    INT64_ASSIGN(sRuntime[uwChannelSel].sqMechAbsPosition, 0L, 0UL) ;
    
        // runtime
    ubRevNumBits = sWorks[uwChannelSel].ubMultiTurn;
    if(ubRevNumBits>0)
        sRuntime[uwChannelSel].ulRevMask = (1ul << ubRevNumBits) - 1;
    else
        sRuntime[uwChannelSel].ulRevMask = 0ul;
  
        // Feedback
    sRuntime[uwChannelSel].psFeedbackOut->uwElecAngle = 0 ;    
    sRuntime[uwChannelSel].psFeedbackOut->swElecSpeed = 0 ;  
    sRuntime[uwChannelSel].psFeedbackOut->uwDeltaElecAngle = 0 ;    
    sRuntime[uwChannelSel].psFeedbackOut->sEncData.slSpeed = 0L ;
    sRuntime[uwChannelSel].psFeedbackOut->sEncData.slAccel = 0L ;
    sRuntime[uwChannelSel].psFeedbackOut->ubStatus = ENCMGR_NULL ;

    return TRUE;
}

//***************************************************************************
// init or re-init unit

static BOOL TMGWinit(UWORD uwChannelSel,BOOL bFullInit)
{
        // hiperface handler init
    if(!TMGWhandlerinit(uwChannelSel,bFullInit))
    {
        sRuntime[uwChannelSel].psFeedbackOut->ubStatus = ENCMGR_FATAL_FAULT ; 
        return FALSE;
    }

    return initruntime(uwChannelSel);
}

//***************************************************************************
// fast task

static BOOL task8kHz(TMGW_RUNTIME * psRuntime)
{
    ULONG ulMechAngle=0l, ulMechAngle_1=0l ;
    SLONG slMechSpeed_1 ;
    ENCMGR_SPACEFEEDBACK * psFeedback;
    ULONG ulBaseAd;
    UWORD uwStat;
    UWORD uwEncError;
    
    psFeedback=psRuntime->psFeedbackOut;
    ulBaseAd=ulBaseAddress[psRuntime->uwChannelSel];
    
    if(bSysStatAlarmsReset)
    {
        // if was faulty, then reset status
        if(psRuntime->sErrorSent.w || *psRuntime->psbEncMgrFault)
        psRuntime->sCmd.b.bInitReq = TRUE;
        
        psRuntime->sErrorSent.w = 0 ; /* clear errors */
    }
    
    // if init in progress and/or parametrization in progress, then do nothing and nullify data
    if(psRuntime->sCmd.b.bInitReq || psRuntime->sCmd.b.bPlcReqStat)
    {
        psFeedback->ubStatus = ENCMGR_NULL;
        return TRUE;
    }
    
    // if faulty then exit
    if(psFeedback->ubStatus&ENCMGR_FATAL_FAULT)
        return TRUE;

    // get status, if busy skip to overtime alarm,
    // if not then readback data, then again status
    uwStat = FPGA_BASEOFF_16(ulBaseAd,FPGA_TMGW_STATUS);    
    if(!FPGA_TMGW_STAT_BUSY(uwStat)&FPGA_TMGW_STAT_EOF(uwStat))
    {
        if(!TMGWReadPosition(&sWorks[psRuntime->uwChannelSel],(HPSLONG)&ulMechAngle,(HPSLONG)&ulMechAngle_1,psFeedback->ubStatus == ENCMGR_NULL))
            return TRUE;
    }
    
    /* check for valid data transfer, enter here also if alarm is just triggered 
     to avoid triggering of other alarms */
    if(FPGA_TMGW_STAT_TIMEOUT(uwStat) || FPGA_TMGW_STAT_BUSY(uwStat) || psRuntime->sErrorSent.b.bOverTime)
    {
        if(!psRuntime->sErrorSent.b.bOverTime)
        {
          SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_TMGW_FAIL, ulAlarmSubCode[psRuntime->uwChannelSel] | SYSTEMALARMS_SUBCODE_TMGW_OVERTIME, bSysStatPowerEnabled) ;
            
          psFeedback->ubStatus  = ENCMGR_FATAL_FAULT ;
          psRuntime->sErrorSent.b.bOverTime = TRUE ;
        }
        return TRUE;
    }
        
    /* check for encoder error & communication alarm */
    TMGWReadError(&sWorks[psRuntime->uwChannelSel],&uwEncError);
    if((uwEncError&~TMGW_ENCERR_CO_MASK)!=0 && !psRuntime->sCmd.b.bDisAlarm)
    {
        if(psRuntime->psWorks->ubDisMTData) // if multi-turn is diabled, ignore Battery and Overspeed error
            uwEncError&=(~(TMGW_ENCERR_BA_MASK|TMGW_ENCERR_BE_MASK|TMGW_ENCERR_ME_MASK|TMGW_ENCERR_OS_MASK));
            
        if(!psRuntime->sErrorSent.b.bAlarm)
        {
            if((uwEncError&~(TMGW_ENCERR_BA_MASK|TMGW_ENCERR_CO_MASK))!=0)  // if not problem of battery, post alarm
            {
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_TMGW_FAIL, ((ULONG)uwEncError<<16) | ulAlarmSubCode[psRuntime->uwChannelSel] | SYSTEMALARMS_SUBCODE_TMGW_ALARM, bSysStatPowerEnabled) ;
                
                psFeedback->ubStatus = ENCMGR_FATAL_FAULT ;
                psRuntime->sErrorSent.b.bAlarm = TRUE ;
            return TRUE;
        }

        if(!psRuntime->sErrorSent.b.bWarn)    // if problem of battery, post warning
            if((uwEncError&TMGW_ENCERR_BA_MASK)==TMGW_ENCERR_BA_MASK)                
            {
                atomic_long_set_bits( &ulSystemWarnings, SYSTEMWARNINGS_ENCBATTERY_FAILURE );
                psRuntime->sErrorSent.b.bWarn = TRUE ;
            }
        }        
    }
    else
    {
        if(psRuntime->sErrorSent.b.bWarn) // if no problem of battery, clear warning
        {
            atomic_long_clear_bits( &ulSystemWarnings, SYSTEMWARNINGS_ENCBATTERY_FAILURE );
            psRuntime->sErrorSent.b.bWarn = FALSE ;
        }
    }
    
    /* if first run get abs pos */
    if(psFeedback->ubStatus == ENCMGR_NULL)
    {    
        psRuntime->sqMechAbsInitPos.lo  = ulMechAngle ;
        if(!psRuntime->psWorks->ubDisMTData)
        {
            psRuntime->sqMechAbsInitPos.hi  = ulMechAngle_1 ;
                
            // check startup position for multiturns encoders
            if(psRuntime->ulRevMask)
                    // if startup turns pos is above threshold, then extend sign
                if((ULONG)psRuntime->sqMechAbsInitPos.hi >= psRuntime->ulMTurnStartPos)
                    psRuntime->sqMechAbsInitPos.hi=~(psRuntime->ulRevMask)| \
                                                    (psRuntime->sqMechAbsInitPos.hi & psRuntime->ulRevMask);
        }
        else
            psRuntime->sqMechAbsInitPos.hi = 0;
            
        psRuntime->sqMechAbsPosition.lo = psRuntime->sqMechAbsInitPos.lo ;
        psRuntime->sqMechAbsPosition.hi = psRuntime->sqMechAbsInitPos.hi ;
        
        INT64_COPY(psFeedback->sqMechAbsPosOffset, psRuntime->sqMechAbsInitPos) ;
            
        psFeedback->ubStatus = ENCMGR_ABSMECHTURN_VALID | ENCMGR_ELE_ANGLE_VALID | ENCMGR_RELATIVE_VALID | ENCMGR_EFS_READY;
    }
  
    /* Previous absolute angle and mechanical speed */
    ulMechAngle_1 = psRuntime->sqMechAbsPosition.lo ;
    slMechSpeed_1 = psFeedback->sEncData.slSpeed ;
        
    /* ============================================ * 
    * =========== Module Output Values =========== * 
    * ============================================ */ 
    /* ---- Electrical Angle (MechAngle2Use = absolute angle from TMGW) ---- */
    psRuntime->uwAbsElecAngle = (UWORD)((ULONG)HIWORD(ulMechAngle) * psRuntime->uwMotPolarPairs); 
    psFeedback->uwElecAngle   = psRuntime->uwAbsElecAngle + psFeedback->uwDeltaElecAngle + sGlbMotorParameters.uwPhaseOffset ;  
    
    /* -- Current Abs Mechanical Position -- */
    if ( (ulMechAngle_1 & 0x80000000) &&  (ulMechAngle_1 & 0x40000000) &&
        !(ulMechAngle   & 0x80000000) && !(ulMechAngle   & 0x40000000)   ) 
    { /* Abs Mechanical Angle : Clockwise Overflow */
        psRuntime->sqMechAbsPosition.hi++ ; 
    } 
    else if (!(ulMechAngle_1 & 0x80000000) && !(ulMechAngle_1 & 0x40000000) &&
              (ulMechAngle   & 0x80000000) &&  (ulMechAngle   & 0x40000000)   ) 
    { /* Abs Mechanical Angle : Anticlockwise Overflow */
        psRuntime->sqMechAbsPosition.hi-- ; 
    } 
    psRuntime->sqMechAbsPosition.lo = ulMechAngle ;
    
    /* -- Current Relative Mechanical Position = AbsMechPosition(64) - AbsMechInitPosition(64) -- */
    _sint64_sub(&psFeedback->sEncData.sqPostn, &psRuntime->sqMechAbsPosition, &psRuntime->sqMechAbsInitPos) ;
    
    /* -- Current Mechanical Speed = Current MechAngle - Previous MechAngle (= delta angle) -- */
    psFeedback->sEncData.slSpeed = (SLONG)(ulMechAngle - ulMechAngle_1) ;
    
    /* -- Current Mechanical Acceleration = Current MechSpeed - Previous MechSpeed (= delta speed) -- */
    psFeedback->sEncData.slAccel = _sint32_asl12(psFeedback->sEncData.slSpeed - slMechSpeed_1) ;
    
    /* -- Current Electrical Speed  = MechSpeed * MotorPolePairs */
    psFeedback->swElecSpeed = (SWORD)_sint32_scale_32(psFeedback->sEncData.slSpeed, psRuntime->uwMotPolarPairs);
    
    return TRUE ;
}

//***************************************************************************
// slow task

static void slowtask(void)
{
    UWORD uwCt;

    for(uwCt=0;uwCt<TMGW_MAX_ENCODER;uwCt++)
    {
        if(sRuntime[uwCt].sCmd.b.bValid)
        {
                // if plc request
            if(sRuntime[uwCt].sCmd.b.bPlcReq != sRuntime[uwCt].sCmd.b.bPlcReqStat)
            {
                if(sRuntime[uwCt].sCmd.b.bPlcReqStat)
                {
                    TMGWEnterPositionMode(&sWorks[uwCt]);
                        // wait some rt cycles to allow communication restart and
                        // alarms clean-up before enabling rt error handling
                    timer_wait(uwSysTimers125us,4);
                    sRuntime[uwCt].sCmd.b.bPlcReqStat=FALSE;
                }
                else
                {
                    TMGWEnterCommandMode(&sWorks[uwCt]);
                    sRuntime[uwCt].sCmd.b.bPlcReqStat=TRUE;
                }
            }
                // check for re-init request
            if(sRuntime[uwCt].sCmd.b.bInitReq && !sRuntime[uwCt].sCmd.b.bPlcReqStat)
            {
                    // when get the re-init request, clear the alarm then re-init
                TMGWEnterCommandMode(&sWorks[uwCt]);
                TMGWHandlers_StatusFlagClear(&sWorks[uwCt]);
                TMGWinit(uwCt,FALSE);
                    // yield other tasks in order to let module get fresh data
                Os_Sleep(0);
                sRuntime[uwCt].sCmd.b.bInitReq=FALSE;
                sRuntime[uwCt].psFeedbackOut->ubStatus=ENCMGR_NULL;
            }

                // if change disable TMGW alarm
            if(sRuntime[uwCt].sCmd.b.bDisAlarm != sTm_Params[uwCt].flags.b.bDisAlarm)
                sRuntime[uwCt].sCmd.b.bDisAlarm=sTm_Params[uwCt].flags.b.bDisAlarm;
        }        
    }
}

//***************************************************************************
// PLC request for command mode or reset to position mode

ULONG PlcTMGWSetCommandMode(UWORD uwChannelSel, BOOL bSet)
{
        // check right drive state
    if(bSysStatBooting || bSysStatResetting || bSysStatPowerEnabled || !Os_IsInBackground())
        return TMGW_FUNCTION_ERROR_NOMODULE;

        // check if selected is valid
    if(uwChannelSel>=TMGW_MAX_ENCODER)
        return TMGW_FUNCTION_ERROR_NOMODULE;

    if(!sRuntime[uwChannelSel].sCmd.b.bValid)
        return TMGW_FUNCTION_ERROR_NOMODULE;

        // send request
    sRuntime[uwChannelSel].sCmd.b.bPlcReq=bSet;

        // wait until request ack
    while(sRuntime[uwChannelSel].sCmd.b.bPlcReqStat!=bSet)
        Os_Sleep(0);

    return 0l;
}

//***************************************************************************
// PLC reset active alarms 

ULONG PlcTMGWResetActiveAlarms(UWORD uwChannelSel)
{
        // check right drive state
    if(!Os_IsInBackground())
        return TMGW_FUNCTION_ERROR_NOMODULE;

        // check if selected is valid
    if(uwChannelSel>=TMGW_MAX_ENCODER)
        return TMGW_FUNCTION_ERROR_NOMODULE;

    if(!sRuntime[uwChannelSel].sCmd.b.bValid || !sRuntime[uwChannelSel].sCmd.b.bPlcReqStat)
        return TMGW_FUNCTION_ERROR_NOMODULE;

    if(!TMGWHandlers_StatusFlagClear(&sWorks[uwChannelSel]))
        return TMGW_FUNCTION_ERROR_TIMEOUT;
    else
    {
        sRuntime[uwChannelSel].psFeedbackOut->ubStatus=ENCMGR_NULL;
        return 0ul;
    }
}

//***************************************************************************
// PLC set zero position

ULONG PlcTMGWSetZeroPosition(UWORD uwChannelSel)
{
        // check right drive state
    if(!Os_IsInBackground())
        return TMGW_FUNCTION_ERROR_NOMODULE;

        // check if selected is valid
    if(uwChannelSel>=TMGW_MAX_ENCODER)
        return TMGW_FUNCTION_ERROR_NOMODULE;

    if(!sRuntime[uwChannelSel].sCmd.b.bValid || !sRuntime[uwChannelSel].sCmd.b.bPlcReqStat)
        return TMGW_FUNCTION_ERROR_NOMODULE;
    
        // Send one rotation data zero preset clear command
    if(!TMGWHandlers_ZeroPosSet(&sWorks[uwChannelSel]))
        return TMGW_FUNCTION_ERROR_TIMEOUT;
    else
        return 0ul;
}

//***************************************************************************
// PLC Multi-turn data clear

ULONG PlcTMGWMTurnClear(UWORD uwChannelSel)
{
        // check right drive state
    if(!Os_IsInBackground())
        return TMGW_FUNCTION_ERROR_NOMODULE;

        // check if selected is valid
    if(uwChannelSel>=TMGW_MAX_ENCODER)
        return TMGW_FUNCTION_ERROR_NOMODULE;

    if(!sRuntime[uwChannelSel].sCmd.b.bValid || !sRuntime[uwChannelSel].sCmd.b.bPlcReqStat)
        return TMGW_FUNCTION_ERROR_NOMODULE;
    
    if(!TMGWHandlers_EMClear(&sWorks[uwChannelSel]))
        return TMGW_FUNCTION_ERROR_TIMEOUT;
    else
        return 0ul;
}
#endif
