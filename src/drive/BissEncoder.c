/****************************************************************************/
/* Project: AxN Control Board                                               */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : BissEncoder.c                                              */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Biss Encoder management functions                          */
/*                                                                          */
/****************************************************************************/

#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonDefines.h"
#include "drive\MotorParameters.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "system\SysLogManagement.h"
#include "system\SystemStatus.h"
#include "system\Os.h"

#include "common\CommonEncoderManager.h"
#include "drive\BissEncoder.h"
#include "drive\BissHandlers.h"

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

BISS_PARAMS sBiss_Params[BISS_MAX_ENCODER];
BISS_OUT sBiss_DataOut[BISS_MAX_ENCODER];

//****************************************************************************
// Default parameters

#ifdef _INFINEON_
const BISS_PARAMS huge sBiss_DefParams =
#else
const BISS_PARAMS sBiss_DefParams =
#endif
{
    {0,0}, 0, 2000ul, 0xFFFFFFFFul
};

#if CFG_ENC_BISS
//****************************************************************************
// Defines

#define CRC_ERROR_LIMIT 0x2

//****************************************************************************
// Data structures

typedef struct {
    union {
        struct {
            UWORD bAlarm        : 1 ;
            UWORD bCrc          : 1 ;
            UWORD bOverTime     : 1 ;
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
    
    SWORD swCrcError ;
    BISS_OUT * psDataOut;
    ENCMGR_SPACEFEEDBACK * psFeedbackOut;

    UWORD uwChannelSel;
    ULONG ulClockFreq;

    SBYTE * psbEncMgrFault;

    BISS_WORKS * psWorks;
    ULONG ulMTurnStartPos;
    ULONG ulRevMask;
    
    UBYTE ubEncType;
  
} BISS_RUNTIME ;
  
//****************************************************************************
// Locals

#ifdef _INFINEON_
static BOOL                  bdata bSlowTaskInstalled=FALSE;
#else
static BOOL                        bSlowTaskInstalled=FALSE;
#endif

#ifdef _INFINEON_
static volatile BISS_RUNTIME sdata sRuntime[BISS_MAX_ENCODER];
#else
static volatile BISS_RUNTIME       sRuntime[BISS_MAX_ENCODER];
#endif
static BISS_WORKS                  sWorks[BISS_MAX_ENCODER];

static ULONG ulBaseAddress[BISS_MAX_ENCODER]=
{
    FPGA_MBISS_BASEADDR,
    FPGA_ABISS_BASEADDR,
};

static ULONG ulAlarmSubCode[BISS_MAX_ENCODER]=
{
    SYSTEMALARMS_SUBCODE_ENC_MAIN,
    SYSTEMALARMS_SUBCODE_ENC_AUX,
};

//****************************************************************************
// Local functions

static BOOL coreinit(UWORD,BOOL);
static BOOL bisshandlerinit(UBYTE, UWORD, ULONG, BOOL);
static BOOL initruntime(UWORD);
static BOOL bissinit(UWORD, BOOL);
static BOOL task8kHz(BISS_RUNTIME *);
static void slowtask(void);

//***************************************************************************
// Initialization entry point

BOOL Biss_Init(UWORD uwChannelSel, ENCMGR_SPACEFEEDBACK * psFeedback, SBYTE * psbEncMgrFault, ULONG ulRTDisMask)
{
        // if wrong parameters
    if(!Biss_ParametersCheck(uwChannelSel))
        return FALSE;

        // setup runtime
    sRuntime[uwChannelSel].psDataOut=&sBiss_DataOut[uwChannelSel];
    sRuntime[uwChannelSel].psFeedbackOut=psFeedback;
    sRuntime[uwChannelSel].uwChannelSel=uwChannelSel;
    sRuntime[uwChannelSel].ulClockFreq=sBiss_Params[uwChannelSel].ulClockFreq;
    sRuntime[uwChannelSel].uwMotPolarPairs = sGlbMotorParameters.uwPoleNumbers / 2 ;
    sRuntime[uwChannelSel].psbEncMgrFault = psbEncMgrFault;
    sRuntime[uwChannelSel].psWorks = &sWorks[uwChannelSel];
    sRuntime[uwChannelSel].ulMTurnStartPos=sBiss_Params[uwChannelSel].ulMTurnStartPos;
    sRuntime[uwChannelSel].ubEncType=sBiss_Params[uwChannelSel].ubType;

        // if not yet done init encoder
    if(!sRuntime[uwChannelSel].sCmd.b.bInitDone)
    {
            // core setup
        if(!coreinit(uwChannelSel, TRUE))
            return FALSE;
    
            // Initialize BISS Encoder
        if(!bissinit(uwChannelSel, TRUE))
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

BOOL Biss_ParametersCheck(UWORD uwChannelSel)
{
    assert(uwChannelSel<BISS_MAX_ENCODER);

    if(!BissCheckParameters((UBYTE)sBiss_Params[uwChannelSel].ubType,sBiss_Params[uwChannelSel].ulClockFreq))
    {
        ParChk_SignalValueError(uwChannelSel==BISS_SEL_MAIN?PARCC_BISS_MAINFREQ:PARCC_BISS_AUXFREQ);
        return FALSE;
    }
    else
    {
        ParChk_ResetValueError(uwChannelSel==BISS_SEL_MAIN?PARCC_BISS_MAINFREQ:PARCC_BISS_AUXFREQ);
        return TRUE;
    }
}

//***************************************************************************
// core options

static BOOL coreinit(UWORD uwChannelSel, BOOL bEnable)
{
    switch(uwChannelSel)
    {
        case BISS_SEL_MAIN:
            FPGA_ENCODER_OPTIONS_MBISS = bEnable;
            break;

        case BISS_SEL_AUX:
#ifndef _HW_DC
            FPGA_ENCODER_OPTIONS_AUXENA = FALSE;
            FPGA_ENCODER_OPTIONS_AUXENB = FALSE;
            
            FPGA_ENCODER_OPTIONS_ABISS = bEnable;
            
            FPGA_ENCODER_OPTIONS_AUXENA = bEnable;
            FPGA_ENCODER_OPTIONS_AUXENB = ~bEnable;
#else
            FPGA_ENCODER_OPTIONS_ABISS = bEnable;
#endif
            break;

        default:
            assert(FALSE);
    }

    return TRUE;
}

//***************************************************************************
// init or re-init unit

static BOOL bisshandlerinit(UBYTE ubType, UWORD uwChannelSel, ULONG ulClockFreq, BOOL bFullInit)
{
    UWORD uwRetVal;
    
        // Initialize BISS Encoder
    uwRetVal=BissInitializeEncoder(ubType, ulBaseAddress[uwChannelSel], ulClockFreq, &sWorks[uwChannelSel], bFullInit);
    if(uwRetVal)
    {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_ENCODERMANAGER_FAIL, ulAlarmSubCode[uwChannelSel] | SYSTEMALARMS_SUBCODE_BISS_INIT, FALSE);
        return FALSE ;
    }

        // set init done if full init was done
    sRuntime[uwChannelSel].sCmd.b.bInitDone|=bFullInit;

    return TRUE;
}

//***************************************************************************
// init runtime

static BOOL initruntime(UWORD uwChannelSel)
{
        // Setup diagnostics
    sBiss_DataOut[uwChannelSel].uwMaxFrequency    =sWorks[uwChannelSel].uwMaxFrequency;
    sBiss_DataOut[uwChannelSel].ubStepPerRevBits  =sWorks[uwChannelSel].ubStepPerRevBits;
    sBiss_DataOut[uwChannelSel].ubRevNumBits      =sWorks[uwChannelSel].ubRevNumBits;

        // init everything from zero (the initial angle will be found during first servo)
    INT64_ASSIGN(sRuntime[uwChannelSel].psFeedbackOut->sEncData.sqPostn,  0L, 0UL) ;
    INT64_ASSIGN(sRuntime[uwChannelSel].psFeedbackOut->sqMechAbsPosOffset,  0L, 0UL) ;
    
    INT64_ASSIGN(sRuntime[uwChannelSel].sqMechAbsInitPos,  0L, 0UL) ;
    INT64_ASSIGN(sRuntime[uwChannelSel].sqMechAbsPosition, 0L, 0UL) ;

        // runtime
    if(sWorks[uwChannelSel].ubRevNumBits>0)
        sRuntime[uwChannelSel].ulRevMask = (1ul << sWorks[uwChannelSel].ubRevNumBits) - 1;
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

static BOOL bissinit(UWORD uwChannelSel, BOOL bFullInit)
{
        // biss handler init
    if(!bisshandlerinit(sRuntime[uwChannelSel].ubEncType, uwChannelSel, sRuntime[uwChannelSel].ulClockFreq, bFullInit))
    {
        sRuntime[uwChannelSel].psFeedbackOut->ubStatus = ENCMGR_FATAL_FAULT ; 
        return FALSE;
    }

    return initruntime(uwChannelSel);
}

//***************************************************************************
// Electrical Angle Setup (only for main encoder)

void Biss_SetElecAngle(UWORD uwElecAngle)
{
    sRuntime[BISS_SEL_MAIN].psFeedbackOut->uwDeltaElecAngle = uwElecAngle - sRuntime[BISS_SEL_MAIN].uwAbsElecAngle - sGlbMotorParameters.uwPhaseOffset;
    sRuntime[BISS_SEL_MAIN].psFeedbackOut->ubStatus        |= ENCMGR_ELE_ANGLE_VALID ;
}

//***************************************************************************
// fast task

static BOOL task8kHz(BISS_RUNTIME * psRuntime)
{
  BOOL bInterpolate;
  ULONG ulMechAngle=0l, ulMechAngle_1=0l ;
  SLONG slMechSpeed_1 ;
  ENCMGR_SPACEFEEDBACK * psFeedback;
  UWORD uwStat;
  BISS_WORKS* psWorks;

  psFeedback=psRuntime->psFeedbackOut;
  psWorks=psRuntime->psWorks;
  
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
  BissReadStatus(psWorks, &uwStat);
  if(!FPGA_BISS_STAT_BUSY(uwStat))
  {
    BissReadPosition(psWorks, &ulMechAngle_1, &ulMechAngle);
    BissReadStatus(psWorks, &uwStat);
  }

  /* check for valid data transfer, enter here also if alarm is just triggered 
     to avoid triggering of other alarms */
  if(!FPGA_BISS_STAT_VALID(uwStat) || FPGA_BISS_STAT_OVERTIME(uwStat) || FPGA_BISS_STAT_BUSY(uwStat) || psRuntime->sErrorSent.b.bOverTime)
  {
    if(!psRuntime->sErrorSent.b.bOverTime)
    {
      SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_ENCODERMANAGER_FAIL, ulAlarmSubCode[psRuntime->uwChannelSel] | SYSTEMALARMS_SUBCODE_BISS_OVERTIME, bSysStatPowerEnabled) ;
        
      psFeedback->ubStatus  = ENCMGR_FATAL_FAULT ;             
      psRuntime->sErrorSent.b.bOverTime = TRUE ;
    }   
    return TRUE;
  }

  /* check for error */
  if(FPGA_BISS_STAT_ERROR(uwStat))
  {
    if(psRuntime->swCrcError>=CRC_ERROR_LIMIT)
    {
      if(!psRuntime->sErrorSent.b.bCrc)
      {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_ENCODERMANAGER_FAIL, ulAlarmSubCode[psRuntime->uwChannelSel] | SYSTEMALARMS_SUBCODE_BISS_CRCERROR, bSysStatPowerEnabled) ;

        psFeedback->ubStatus  = ENCMGR_FATAL_FAULT ;             
        psRuntime->sErrorSent.b.bCrc = TRUE ;
      }
      return TRUE;
    }
    psRuntime->swCrcError++;
    if(psRuntime->psDataOut->uwCRCErrorCounter<65535)
        psRuntime->psDataOut->uwCRCErrorCounter++;

    bInterpolate=TRUE;
  }
  else
  {
    if(psRuntime->swCrcError>0)
      psRuntime->swCrcError-- ;
    bInterpolate=FALSE;
  }

    /* check error */
  if(FPGA_BISS_STAT_ALARM(uwStat) && !psRuntime->sCmd.b.bDisAlarm)
  {
    if(!psRuntime->sErrorSent.b.bAlarm)
    {
      SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_ENCODERMANAGER_FAIL, ulAlarmSubCode[psRuntime->uwChannelSel] | SYSTEMALARMS_SUBCODE_BISS_ALARM, bSysStatPowerEnabled) ;
    
      psFeedback->ubStatus  = ENCMGR_FATAL_FAULT ;             
      psRuntime->sErrorSent.b.bAlarm = TRUE ;
    }
    return TRUE;
  }

  /* if first run get abs pos */
  if(psFeedback->ubStatus == ENCMGR_NULL)
  {
    // if interpolation requested then exit
    if(bInterpolate)
      return TRUE;
    
    psRuntime->sqMechAbsInitPos.lo  = ulMechAngle ;
    psRuntime->sqMechAbsInitPos.hi  = ulMechAngle_1 ;

    // check startup position for multiturns encoders
    if(psRuntime->ulRevMask)
            // if startup turns pos is above threshold, then extend sign
        if((ULONG)psRuntime->sqMechAbsInitPos.hi >= psRuntime->ulMTurnStartPos)
            psRuntime->sqMechAbsInitPos.hi=~(psRuntime->ulRevMask)| \
                                            (psRuntime->sqMechAbsInitPos.hi & psRuntime->ulRevMask);

    psRuntime->sqMechAbsPosition.lo = psRuntime->sqMechAbsInitPos.lo ;
    psRuntime->sqMechAbsPosition.hi = psRuntime->sqMechAbsInitPos.hi ;
    
    INT64_COPY(psFeedback->sqMechAbsPosOffset, psRuntime->sqMechAbsInitPos) ;
        
    psFeedback->ubStatus = ENCMGR_ABSMECHTURN_VALID | ENCMGR_ELE_ANGLE_VALID | ENCMGR_RELATIVE_VALID | ENCMGR_EFS_READY;
  }

  /* Previous absolute angle and mechanical speed */
  ulMechAngle_1 = psRuntime->sqMechAbsPosition.lo ;
  slMechSpeed_1 = psFeedback->sEncData.slSpeed ;

  /* if requested then interpolate, if not position readback is already read */
  if(bInterpolate)
    ulMechAngle = ulMechAngle_1 + slMechSpeed_1 ;

  /* ============================================ * 
   * =========== Module Output Values =========== * 
   * ============================================ */ 
  /* ---- Electrical Angle (MechAngle2Use = absolute angle from BISS) ---- */
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

    for(uwCt=0;uwCt<BISS_MAX_ENCODER;uwCt++)
        if(sRuntime[uwCt].sCmd.b.bValid)
        {
                // if plc request
            if(sRuntime[uwCt].sCmd.b.bPlcReq != sRuntime[uwCt].sCmd.b.bPlcReqStat)
            {
                if(sRuntime[uwCt].sCmd.b.bPlcReqStat)
                {
                    BissEnterPositionMode(&sWorks[uwCt]);
                        // wait some rt cycles to allow communication restart and
                        // alarms clean-up before enabling rt error handling
                    timer_wait(uwSysTimers125us,4);
                    sRuntime[uwCt].sCmd.b.bPlcReqStat=FALSE;
                }
                else
                {
                    sRuntime[uwCt].sCmd.b.bPlcReqStat=TRUE;
                    BissEnterCommandMode(&sWorks[uwCt]);
                }
            }
                // check for re-init request
            if(sRuntime[uwCt].sCmd.b.bInitReq && !sRuntime[uwCt].sCmd.b.bPlcReqStat)
            {
                
                bissinit(uwCt, FALSE);
                    // yield other tasks in order to let module get fresh data
                Os_Sleep(0);
                sRuntime[uwCt].sCmd.b.bInitReq=FALSE;
            }

                // if change disable BISS alarm
            if(sRuntime[uwCt].sCmd.b.bDisAlarm != sBiss_Params[uwCt].flags.b.bDisAlarm)
                sRuntime[uwCt].sCmd.b.bDisAlarm=sBiss_Params[uwCt].flags.b.bDisAlarm;
        }
}
#endif