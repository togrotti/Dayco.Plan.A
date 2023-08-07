/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : HallEncoder.c                                              */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "system\SysAppGlobals.h"
#include "system\SysLogManagement.h"
#include "system\SystemAlarms.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"

#include "common\CommonEncoderManager.h"
#include "drive\HallEncoder.h"

#include "common\TaskScheduler.h"

#include "common\Int64Functions.h"    /* needed for 64bit functions */
#include "common\DspFunctions.h"

#ifdef _INFINEON_
#include "AlteraFpgaHandler.h" /* needed to access FPGA registers */
#else
#include "fpga\FpgaHandler.h"
#endif // _infineon_

#include "fpga\AnProcessor.h"

/* ================================ #define ================================ */
/* Thresholds for Single Ended and Differential signals */
#define HALL_SINGLE_ENDED_THR_LEVEL    -8000
#define HALL_DIFFERENTIAL_THR_UPPER   +10922
#define HALL_DIFFERENTIAL_THR_LOWER   -10922
#define HALL_SIGNALS_HYSTERESIS         2048 /* Signal Hysteresis */

/* 30 degres */
#define HALL_SENSOR_ANGLE_OFFSET  0x15555555

/* ============================== structures =============================== */
typedef struct {
 /* Flags */
  union {
      struct {
        UWORD bFirstTime   : 1 ;
        UWORD b4WSelection : 1;
      } b ;
      UWORD w ;
  } flags ;

  union {
      struct {
          UWORD bInvalidState  : 1 ;
          UWORD bInvalidDelta  : 1 ; 
      } b ;
      UWORD w ;
  } sErrorSent ; 

  SQWRD sqMechAbsPosition ;
  SQWRD sqMechAbsInitPosition ;
    
  ULONG ulHallSensorAngles[6] ;  
  
  UWORD uwEncPolarPairs ;
  ULONG ulMechSteps ;
  ULONG ulMechAdder ;
    
  /* Motor Parameters */
  UWORD uwMotPolarPairs  ;

  ENCMGR_SPACEFEEDBACK * psFeedback;
  
  /* regression */
  SWORD swIndex_1 ;
  
  SBYTE * psbEncMgrFault;

} HALL_RUNTIME ;

/* =========================== global  variables =========================== */
HALL_PARAMS  sHl_HallParam ;
HALL_OUT     sHl_DataOut ;

// default parameters
#ifdef _INFINEON_
const HALL_PARAMS huge sHl_HallDefParam =
#else
const HALL_PARAMS sHl_HallDefParam =
#endif // _infineon_
{
    {0},
    0, // Encoder Poles Number
} ;

#if CFG_ENC_HALL
/* =========================== local  variables =========================== */
static HALL_RUNTIME sHallRun ;

/* =============================== functions =============================== */ 
static ULONG HallEncInit(ENCMGR_SPACEFEEDBACK *pHallOut, HALL_RUNTIME *pHallRun, HALL_PARAMS *pHallParam, SBYTE * psbEncMgrFault) ;
static BOOL HallEnc8KHz(HALL_RUNTIME *pHallRun) ;
static void initruntime(HALL_RUNTIME * pLocRT);

/* ######################################################################### */
BOOL Hl_Init(ENCMGR_SPACEFEEDBACK * psFeedback, SBYTE * psbEncMgrFault, ULONG ulRTDisMask)
{
  ULONG ulRetVal;
    
  ulRetVal = HallEncInit(psFeedback, &sHallRun, &sHl_HallParam, psbEncMgrFault);
  if ( ulRetVal )
  {
    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_HALLSENSORS_FAIL, SYSTEMALARMS_SUBCODE_ENC_MAIN | ulRetVal, FALSE);
    return FALSE ;
  }

  return TaskSched_AddRTTaskEx((BOOL (*)(UWORD))&HallEnc8KHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|ulRTDisMask, 0, (UWORD)&sHallRun) ; /* install 8KHz function */
}

/* ######################################################################### */
BOOL Hl_ParametersCheck(void)
{
    if(sHl_HallParam.uwEncPoleNumber%2)
    {
        ParChk_SignalValueError(PARCC_HALL_POLENUMBERS);
        return FALSE;
    }
    else
    {
        ParChk_ResetValueError(PARCC_HALL_POLENUMBERS);
        return TRUE;
    }
}

/* ######################################################################### */
void Hl_SetElecAngle(UWORD uwElecAngle)
{
    sHallRun.psFeedback->uwDeltaElecAngle=uwElecAngle-(UWORD)((ULONG)HIWORD(sHallRun.sqMechAbsPosition.lo) * sHallRun.uwMotPolarPairs)-sGlbMotorParameters.uwPhaseOffset;
    sHallRun.psFeedback->ubStatus |= ENCMGR_ELE_ANGLE_VALID ;
}

/* ######################################################################### */
static ULONG HallEncInit(ENCMGR_SPACEFEEDBACK *pHallOut, HALL_RUNTIME *pHallRun, HALL_PARAMS *pHallParam, SBYTE * psbEncMgrFault)
{
  ANPROC_CHAN sAInDef;
  SWORD swNIndex;

  pHallRun->sErrorSent.w = 0 ; /* clear errors */
  pHallRun->psbEncMgrFault = psbEncMgrFault;   
    
  pHallRun->psFeedback=pHallOut;

  if ( pHallParam->uwEncPoleNumber )
    pHallRun->uwEncPolarPairs = pHallParam->uwEncPoleNumber / 2 ;
  else
    pHallRun->uwEncPolarPairs = sGlbMotorParameters.uwPoleNumbers / 2;

  pHallRun->ulMechSteps  = ((ULONG)(0UL - pHallRun->uwEncPolarPairs) / pHallRun->uwEncPolarPairs) + 1 ;

  pHallRun->flags.b.b4WSelection=pHallParam->flags.b.b4WSelection;
    
  for (swNIndex = 0; swNIndex < 6; swNIndex++)
  {
    /* pHallRun->ulHallSensorAngles[swNIndex] = (ULONG)HALL_SENSOR_ANGLE_OFFSET + (ULONG)swNIndex * (pHallRun->ulMechSteps / 6) ; */
    pHallRun->ulHallSensorAngles[swNIndex] = (ULONG)swNIndex * (pHallRun->ulMechSteps / 6) ;
  }
    
  /* High Gain for HALL Encoders */
#ifdef _INFINEON_
#ifndef _APP_XC
  if(sGlbControlBoardParameters.sProductInfo.uwProductRev<200)
    FPGA_ENCODER_OPTIONS_GAIN = TRUE;
  else
  {
    P8_IOCR00 = 0x0080;
    P8_OUT_P0 = TRUE;
  }
#else
  P2_IOCR13 = 0x0080;
  P2_OUT_P13 = TRUE;
#endif
#endif // _infineon_
  /* Electrical Angle */
  pHallRun->uwMotPolarPairs  = sGlbMotorParameters.uwPoleNumbers / 2 ;

        // common data for analog input processing
  sAInDef.ubOpt=ANPROC_OF_AVG_1SHT|ANPROC_OF_BLK_NONE|ANPROC_OF_DST_SHORT;
  sAInDef.ubNumSample=8;
  sAInDef.sCal.uwOffst=FPGA_ADCDEF_GENERIC_OFF;
  sAInDef.sCal.swScale=FPGA_ADCDEF_GENERIC_MUL;
  sAInDef.uwDstIntImm=ANPROC_ADR_DISABLE;
  sAInDef.uwDstIntAvg=ANPROC_ADR_DISABLE;
  sAInDef.pvDstExtImm=NULL;
  sAInDef.flScale=1.0;

      // prepare sin unit measure conversion
  sAInDef.pvDstExtAvg=&sHl_DataOut.swSinChannel;
  AnProc_Set(FPGA_ADTAGS_ABSENC_SIN,&sAInDef);

      // prepare cos unit measure conversion
  sAInDef.pvDstExtAvg=&sHl_DataOut.swCosChannel;
  AnProc_Set(FPGA_ADTAGS_ABSENC_COS,&sAInDef);

  initruntime(pHallRun);

  return 0l ;
}

/* ######################################################################### */
static void initruntime(HALL_RUNTIME * pLocRT)
{
  pLocRT->ulMechAdder = 0UL ; 

  /* Absolute Position */
  INT64_COPY(pLocRT->sqMechAbsPosition,             sqZero64bit) ; 
  INT64_COPY(pLocRT->sqMechAbsInitPosition,         sqZero64bit) ; 
  INT64_COPY(pLocRT->psFeedback->sEncData.sqPostn,  sqZero64bit) ; 
  INT64_COPY(pLocRT->psFeedback->sqMechAbsPosOffset,sqZero64bit) ; 

  pLocRT->swIndex_1  = -1 ; 
  pLocRT->flags.b.bFirstTime = TRUE ;  
  
  /* Electrical Angle */
  pLocRT->psFeedback->uwElecAngle = 0 ;
  pLocRT->psFeedback->swElecSpeed = 0 ;
  pLocRT->psFeedback->uwDeltaElecAngle = 0;
    
  pLocRT->psFeedback->sEncData.slSpeed = 0L ;
  pLocRT->psFeedback->sEncData.slAccel = 0L ;

  pLocRT->psFeedback->ubStatus = ENCMGR_NULL ; 
}

/* ######################################################################### */
static BOOL HallEnc8KHz(HALL_RUNTIME *pHallRun)
{
  SWORD swIndex, swIndexDelta ;
  SWORD swHall12, swHall13 ;
  ULONG ulAbsMechAngle_1 ;
  SLONG slMechSpeed_1 = 0 ;
  SLONG slMechSpeed ;
  SLONG ulMechAngle_1 ;
  ENCMGR_SPACEFEEDBACK * pHallOut;
  
  pHallOut=pHallRun->psFeedback;

  if(bSysStatAlarmsReset)
  {
    // if was faulty, then reset status
    if(pHallRun->sErrorSent.w || *pHallRun->psbEncMgrFault)
      initruntime(pHallRun);

    pHallRun->sErrorSent.w = 0 ; /* clear errors */
  }

  swHall13 = sHl_DataOut.swSinChannel ; /* SIN channel (H3 or H1-H3) */
  swHall12 = sHl_DataOut.swCosChannel ; /* COS channel (H1-H2) */

  // if faulty then exit
  if(pHallRun->psFeedback->ubStatus&ENCMGR_FATAL_FAULT)
    return TRUE;
    
  if (!pHallRun->flags.b.b4WSelection) 
  {  /* Hall Sensors connected through 3 wires (COS = H1-H2, SIN = H3) */
    if (swHall13 >= (HALL_SINGLE_ENDED_THR_LEVEL + HALL_SIGNALS_HYSTERESIS)) 
    { /* Single Ended Levels ==> +1 */
      if (swHall12 >= (HALL_DIFFERENTIAL_THR_UPPER + HALL_SIGNALS_HYSTERESIS))
        swIndex = 0 ; /* Differential Levels ==> +1 */
      else if (swHall12 <= (HALL_DIFFERENTIAL_THR_LOWER - HALL_SIGNALS_HYSTERESIS))
        swIndex = 2 ; /* Differential Levels ==> -1 */  
      else if ((swHall12 < (HALL_DIFFERENTIAL_THR_UPPER - HALL_SIGNALS_HYSTERESIS)) && 
               (swHall12 > (HALL_DIFFERENTIAL_THR_LOWER + HALL_SIGNALS_HYSTERESIS))   )
        swIndex = 1 ; /* Differential Levels ==>  0 */ 
      else  
        swIndex = pHallRun->swIndex_1 ; /* Differential Levels ==> Hysteresis */
    } 
    else if (swHall13 <= (HALL_SINGLE_ENDED_THR_LEVEL - HALL_SIGNALS_HYSTERESIS)) 
    { /* Single Ended Levels ==> -1 */
      if (swHall12 >= (HALL_DIFFERENTIAL_THR_UPPER + HALL_SIGNALS_HYSTERESIS))
        swIndex = 5 ; /* Differential Levels ==> +1 */
      else if (swHall12 <= (HALL_DIFFERENTIAL_THR_LOWER - HALL_SIGNALS_HYSTERESIS))
        swIndex = 3 ; /* Differential Levels ==> -1 */ 
      else if ((swHall12 < (HALL_DIFFERENTIAL_THR_UPPER - HALL_SIGNALS_HYSTERESIS)) && 
               (swHall12 > (HALL_DIFFERENTIAL_THR_LOWER + HALL_SIGNALS_HYSTERESIS))   )
        swIndex = 4 ; /* Differential Levels ==>  0 */      
      else 
        swIndex = pHallRun->swIndex_1 ; /* Differential Levels ==> Hysteresis */  
    } 
    else 
      swIndex = pHallRun->swIndex_1 ; /* Single Ended Levels ==> Hysteresis */ 
  } 
  else
  { /* Hall Sensors connected through 4 wires (COS = H1-H2, SIN = H1-H3) */
    if (swHall13 >= (HALL_DIFFERENTIAL_THR_UPPER + HALL_SIGNALS_HYSTERESIS))
    { /* Differential Levels ==> +1 */
      if (swHall12 >= (HALL_DIFFERENTIAL_THR_UPPER + HALL_SIGNALS_HYSTERESIS))
        swIndex = 0 ;  /* Differential Levels ==> +1 */ 
      else if (swHall12 <= (HALL_DIFFERENTIAL_THR_LOWER - HALL_SIGNALS_HYSTERESIS))
        swIndex = -2 ; /* Differential Levels ==> -1 */
      else if ((swHall12 < (HALL_DIFFERENTIAL_THR_UPPER - HALL_SIGNALS_HYSTERESIS)) && 
               (swHall12 > (HALL_DIFFERENTIAL_THR_LOWER + HALL_SIGNALS_HYSTERESIS))   )
        swIndex = 1 ;  /* Differential Levels ==>  0 */
      else 
        swIndex = pHallRun->swIndex_1 ; /* Differential Levels ==> Hysteresis */  
    } 
    else if (swHall13 <= (HALL_DIFFERENTIAL_THR_LOWER - HALL_SIGNALS_HYSTERESIS))
    { /* Differential Levels ==> -1 */
      if (swHall12 >= (HALL_DIFFERENTIAL_THR_UPPER + HALL_SIGNALS_HYSTERESIS))
        swIndex = -2 ; /* Differential Levels ==> +1 */ 
      else if (swHall12 <= (HALL_DIFFERENTIAL_THR_LOWER - HALL_SIGNALS_HYSTERESIS))
        swIndex = 3 ;  /* Differential Levels ==> -1 */
      else if ((swHall12 < (HALL_DIFFERENTIAL_THR_UPPER - HALL_SIGNALS_HYSTERESIS)) && 
               (swHall12 > (HALL_DIFFERENTIAL_THR_LOWER + HALL_SIGNALS_HYSTERESIS))   )
        swIndex = 4 ;  /* Differential Levels ==>  0 */
      else 
        swIndex = pHallRun->swIndex_1 ; /* Differential Levels ==> Hysteresis */
    } 
    else if ((swHall13 < (HALL_DIFFERENTIAL_THR_UPPER - HALL_SIGNALS_HYSTERESIS)) && 
             (swHall13 > (HALL_DIFFERENTIAL_THR_LOWER + HALL_SIGNALS_HYSTERESIS))   )
    { /* Differential Levels ==> 0 */
      if (swHall12 >= (HALL_DIFFERENTIAL_THR_UPPER + HALL_SIGNALS_HYSTERESIS)) 
        swIndex = 5 ;  /* Differential Levels ==> +1 */
      else if (swHall12 <= (HALL_DIFFERENTIAL_THR_LOWER - HALL_SIGNALS_HYSTERESIS))
        swIndex = 2 ;  /* Differential Levels ==> -1 */     
      else if ((swHall12 < (HALL_DIFFERENTIAL_THR_UPPER - HALL_SIGNALS_HYSTERESIS)) && 
               (swHall12 > (HALL_DIFFERENTIAL_THR_LOWER + HALL_SIGNALS_HYSTERESIS))   )
        swIndex = -2 ; /* Differential Levels ==>  0 */ 
      else 
        swIndex = pHallRun->swIndex_1 ; /* Differential Levels ==> Hysteresis */
    } 
    else 
      swIndex = pHallRun->swIndex_1 ; /* Differential Levels ==> Hysteresis */
   
    if ((swIndex == -2) && !pHallRun->sErrorSent.b.bInvalidState)
    { /* For 4 Wire configuration, invalid HALL state */
      SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_HALLSENSORS_FAIL, SYSTEMALARMS_SUBCODE_ENC_MAIN | SYSTEMALARMS_SUBCODE_HL_INVALID_STATE, bSysStatPowerEnabled);
    
      pHallOut->ubStatus = ENCMGR_FATAL_FAULT ;
      pHallRun->sErrorSent.b.bInvalidState = TRUE ;
      return TRUE ;
    }
  }

  /* ======================================= *
   * ===== Wait first valid HALL state ===== *
   * ======================================= */
  if (pHallRun->flags.b.bFirstTime)
  {
    if (pHallRun->swIndex_1 == -1 && swIndex < 0)
      return TRUE ;
      
    pHallRun->flags.b.bFirstTime = FALSE ;
    pHallRun->swIndex_1 = swIndex ;
  }

  /* Index Delta */
  swIndexDelta = swIndex - pHallRun->swIndex_1 ;
    
  /* Save Index */
  pHallRun->swIndex_1 = swIndex ;
  
  if (swIndexDelta == -5) 
        pHallRun->ulMechAdder += pHallRun->ulMechSteps ; /* Electrical Angle : Clockwise Overflow */
  else if (swIndexDelta == +5) 
        pHallRun->ulMechAdder -= pHallRun->ulMechSteps ; /* Electrical Angle : Anticlockwise Overflow */
  else if (swIndexDelta < -1 || swIndexDelta > +1) 
  { /* Check if valid, must be -1, 0 or +1 */
        if(!pHallRun->sErrorSent.b.bInvalidDelta)
        {        
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_HALLSENSORS_FAIL, SYSTEMALARMS_SUBCODE_ENC_MAIN | SYSTEMALARMS_SUBCODE_HL_INVALID_DELTA, bSysStatPowerEnabled);        
            pHallOut->ubStatus = ENCMGR_FATAL_FAULT ;
            pHallRun->sErrorSent.b.bInvalidDelta = TRUE ;
        }
    return FALSE ;
  }

  ulAbsMechAngle_1 = pHallRun->sqMechAbsPosition.lo ; /* Previuos abs mechanical angle */

  /* Current mechanical angle */
  pHallRun->sqMechAbsPosition.lo = pHallRun->ulHallSensorAngles[swIndex] + pHallRun->ulMechAdder ;

  if ( (ulAbsMechAngle_1               & 0x80000000) &&  (ulAbsMechAngle_1               & 0x40000000) &&
      !(pHallRun->sqMechAbsPosition.lo & 0x80000000) && !(pHallRun->sqMechAbsPosition.lo & 0x40000000)   ) 
  { /* Mechanical Angle : Clockwise Overflow */
    pHallRun->sqMechAbsPosition.hi++ ;
  } 
  else if (!(ulAbsMechAngle_1               & 0x80000000) && !(ulAbsMechAngle_1               & 0x40000000) &&
            (pHallRun->sqMechAbsPosition.lo & 0x80000000) &&  (pHallRun->sqMechAbsPosition.lo & 0x40000000)   )
  { /* Mechanical Angle : Anticlockwise Overflow */
    pHallRun->sqMechAbsPosition.hi--;
  }

  ulMechAngle_1 = pHallOut->sEncData.sqPostn.lo ; /* Previous rel mechanical angle */ 
  slMechSpeed_1 = pHallOut->sEncData.slSpeed ;    /* Save previuos mechanical speed */

  /* Current Electrical Angle */
  pHallOut->uwElecAngle = (UWORD)((ULONG)HIWORD(pHallRun->sqMechAbsPosition.lo) * pHallRun->uwMotPolarPairs) + pHallOut->uwDeltaElecAngle + sGlbMotorParameters.uwPhaseOffset ;

  /* if first run and valid signal level then get abs pos */
  if(pHallOut->ubStatus == ENCMGR_NULL)
  {
    pHallRun->sqMechAbsInitPosition.lo = pHallRun->sqMechAbsPosition.lo;
    pHallRun->sqMechAbsInitPosition.hi = pHallRun->sqMechAbsPosition.hi;

    INT64_COPY(pHallOut->sqMechAbsPosOffset, pHallRun->sqMechAbsInitPosition) ;

    pHallOut->ubStatus = ENCMGR_ABSMECHTURN_VALID | ENCMGR_ELE_ANGLE_VALID | ENCMGR_RELATIVE_VALID | ENCMGR_EFS_READY;
  }    

  /* Current Relative Position */
  _sint64_sub(&pHallOut->sEncData.sqPostn, &pHallRun->sqMechAbsPosition, &pHallRun->sqMechAbsInitPosition) ;

  /* Current Mechanical Speed = MechAngleRel - MechAngleRel_1 (= delta rel position) */
  slMechSpeed = pHallOut->sEncData.sqPostn.lo - ulMechAngle_1 ; /* NOT filtered speed */

  /* filtering conditioned by initial sign in order to reduce rounding problem */
  if(pHallOut->sEncData.slSpeed>0)
    pHallOut->sEncData.slSpeed = (pHallOut->sEncData.slSpeed >> 8) * 255 + (slMechSpeed >> 8) ;
  else
    pHallOut->sEncData.slSpeed = (-pHallOut->sEncData.slSpeed >> 8) * -255 + (slMechSpeed >> 8) ;
    
  /* Current Mechanical Acceleration = CurrMechSpeed - PrevMechSpeed (= delta speed) */
  pHallOut->sEncData.slAccel = _sint32_asl12(pHallOut->sEncData.slSpeed - slMechSpeed_1) ;

  /* Current Electrical Speed  = MechSpeed * MotorPolePairs */
  pHallOut->swElecSpeed = (SWORD)_sint32_scale_32(pHallOut->sEncData.slSpeed, pHallRun->uwMotPolarPairs) ;
    
  return TRUE ;
}
#endif
