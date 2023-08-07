/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright $)AB) 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : SincosEncoder.c                                            */
/* Author      : Cristiano Tognetti                                         */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"

#include "common\CommonEncoderManager.h"
#include "SincosEncoder.h"

#include "common\TaskScheduler.h"
#include "system\SysLogManagement.h"
#include "fpga\AnProcessor.h"

#include "common\MathFunctions.h"
#include "common\Int64Functions.h"
#include "common\DspFunctions.h"
#include "fpga\FpgaHandler.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

/* ================================ #define ================================ */
#define SINCOSALARMCOUNTERMAX   5


/* ============================== structures =============================== */
typedef struct {
 /* Flags */
  union {
      struct {
        UWORD bReverseSignals        : 1 ; // 0 => Angle = Atan(Sin/Cos); 1 => Angle = 0xffff -  Atan(Sin/Cos)  (Resolver/SinCos)
        UWORD bSin2Cos2ErrorDisabled : 1 ; // 0 => sin^2 + cos^2 error check enabled; 1 => sin^2 + cos^2 error check disabled
        UWORD bInitialized           : 1 ; // setup TRUE when module is initialized
        UWORD bAnalogGain            : 1 ; // analog gain (lo/hi) selected by parameter
        UWORD bEnSinCosHw            : 1 ; // enable only hardware analog channels
      } b ;
      UWORD w ;
  } flags ;
    
  union {
    struct {
      UWORD bSincosLevel : 1 ;
    } b ;
    UWORD w ;
  } sErrorSent ;    
    
  SQWRD sqMechAbsPosition ;
  SQWRD sqMechAbsInitPosition ;
    
  UWORD uwEncPolarPairs ;
  ULONG ulMechSteps ;
  ULONG ulMechAdder ;

  ULONG ulSinCosAlarmThreshold ; // al quadrato..per risparmiare tempo ad 8KHz non faccio la radice quadrata
  SWORD swSinCosAlarmCounter ;
    
  UWORD uwEncAngle ;

  ENCMGR_SPACEFEEDBACK * psFeedback;
    
  /* Motor Parameters */
  UWORD uwMotPolarPairs  ;

  SBYTE * psbEncMgrFault;

} SINCOS_RUNTIME ;

/* =========================== global  variables =========================== */

SINCOS_PARAMS sSc_SinCosParam ;
SINCOS_OUT    sSc_DataOut ;

// default parameters
const SINCOS_PARAMS  sSc_SinCosDefParam =
{ 
    {0,1,0,0},
    2,    // Encoder Poles Number
    0,
    5000, // Resolver/Sincos Alarm
    2600, // gain = 2.6 BEFORE IS 1.0
    2600, // gain = 2.6 BEFORE IS 1.0
    0,
    0,
    337   // resolver freq offset 33.7usec
} ;

#if CFG_ENC_SINCOS
/* ============================ local  variables =========================== */

static SINCOS_RUNTIME sSinCosRun ;

#define MEN_ABS_ENC_GAIN_PIN  (MIO_PIN_BASE+17)

/* =============================== functions =============================== */ 
static void initruntime(SINCOS_RUNTIME * pLocRT);

static void Sc_SinCosEncoderBkGd(void) ;

static ULONG SinCosEncInit(ENCMGR_SPACEFEEDBACK *pSinCosOut, SINCOS_RUNTIME *pSinCosRun, SINCOS_PARAMS *pSinCosParam, SBYTE * psbEncMgrFault) ;
static BOOL SinCosEnc8KHz(SINCOS_RUNTIME *pSinCosRun) ;
static void SinCosEncBkGd(SINCOS_RUNTIME *pSinCosRun, SINCOS_PARAMS *pSinCosParam) ;
static void SinCosAnalogSet(SINCOS_RUNTIME *pSinCosRun, SINCOS_PARAMS *pSinCosParam) ;

/* ######################################################################### */
BOOL Sc_Init(ENCMGR_SPACEFEEDBACK * psFeedback, SBYTE * psbEncMgrFault, ULONG ulRTDisMask)
{
    ULONG ulRetVal ;
    
    ulRetVal = SinCosEncInit(psFeedback, &sSinCosRun, &sSc_SinCosParam, psbEncMgrFault) ;
    if ( ulRetVal )
    {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_SINCOS_FAIL, SYSTEMALARMS_SUBCODE_ENC_MAIN | ulRetVal, FALSE) ;
        return FALSE ;
    }

    if(!TaskSched_AddBackgroundTask(&Sc_SinCosEncoderBkGd))
        return FALSE ;

    if(psFeedback)
        return TaskSched_AddRTTaskEx((BOOL (*)(ULONG))&SinCosEnc8KHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|ulRTDisMask, 0, (ULONG)&sSinCosRun) ; /* install 8KHz function */
    else
        return TRUE;
}

/* ######################################################################### */
BOOL Sc_ParametersCheck(void)
{
    if(sSc_SinCosParam.uwEncPoleNumber==0 || sSc_SinCosParam.uwEncPoleNumber%2)
    {
        ParChk_SignalValueError(PARCC_ANALOG_POLENUMBERS);
        return FALSE;
    }
    else
    {
        ParChk_ResetValueError(PARCC_ANALOG_POLENUMBERS);
        return TRUE;
    }
}

/* ######################################################################### */
void Sc_SetElecAngle(UWORD uwElecAngle)
{
    sSinCosRun.psFeedback->uwDeltaElecAngle=uwElecAngle-(UWORD)((ULONG)HIWORD(sSinCosRun.sqMechAbsPosition.lo) * sSinCosRun.uwMotPolarPairs)-sGlbMotorParameters.uwPhaseOffset;
    sSinCosRun.psFeedback->ubStatus |= ENCMGR_ELE_ANGLE_VALID ;
}

/* ######################################################################### */
static ULONG SinCosEncInit(ENCMGR_SPACEFEEDBACK *pSinCosOut, SINCOS_RUNTIME *pSinCosRun, SINCOS_PARAMS *pSinCosParam, SBYTE * psbEncMgrFault)
{
  // if already initialized then error
  if(pSinCosRun->flags.b.bInitialized)
    return 1l;

  // enable and use sincos hardware ONLY
  if (pSinCosOut == NULL) 
    pSinCosRun->flags.b.bEnSinCosHw = TRUE ;
  else
    pSinCosRun->flags.b.bEnSinCosHw = FALSE ;
   
  pSinCosRun->sErrorSent.w = 0 ; /* clear errors */
  pSinCosRun->psbEncMgrFault = psbEncMgrFault;

  pSinCosRun->psFeedback=pSinCosOut;

  pSinCosRun->uwEncPolarPairs = pSinCosParam->uwEncPoleNumber / 2 ;
  pSinCosRun->ulMechSteps = ((ULONG)(0UL - pSinCosRun->uwEncPolarPairs) / pSinCosRun->uwEncPolarPairs) + 1 ; 

  pSinCosRun->flags.b.bReverseSignals           = pSinCosParam->flags.b.bReverseSignals ;
  pSinCosRun->flags.b.bSin2Cos2ErrorDisabled    = pSinCosParam->flags.b.bSin2Cos2ErrorDisabled ;
  pSinCosRun->flags.b.bAnalogGain               = pSinCosParam->flags.b.bGain2Use ;

  Gpio_SetMode(MEN_ABS_ENC_GAIN_PIN, GPIO_DIR_OUT);
  GPIO_OUT(MEN_ABS_ENC_GAIN_PIN, pSinCosParam->flags.b.bGain2Use); /* 0 = HIGH (SinCos); 1 = LOW (Resolver) */

  FPGA_ENCODER_OPTIONS_GAIN = pSinCosParam->flags.b.bGain2Use ; /* 0 = HIGH (SinCos); 1 = LOW (Resolver) */

  /* Electrical Angle */
  pSinCosRun->uwMotPolarPairs  = sGlbMotorParameters.uwPoleNumbers / 2 ;

  // setup runtime values
  SinCosEncBkGd(&sSinCosRun, &sSc_SinCosParam) ;

  // setup analog channels
  SinCosAnalogSet(&sSinCosRun, &sSc_SinCosParam) ;

  // initialize sin^2+cos^2 level counter
  pSinCosRun->swSinCosAlarmCounter = 0 ;

  pSinCosRun->flags.b.bInitialized = TRUE ;

  if(pSinCosOut)
    initruntime(pSinCosRun);

  return 0l ;
}

/* ######################################################################### */
static void initruntime(SINCOS_RUNTIME * pLocRT)
{
  pLocRT->ulMechAdder = 0UL ; 

  /* Absolute Position */
  INT64_COPY(pLocRT->sqMechAbsPosition,             sqZero64bit) ; 
  INT64_COPY(pLocRT->sqMechAbsInitPosition,         sqZero64bit) ; 
  INT64_COPY(pLocRT->psFeedback->sEncData.sqPostn,  sqZero64bit) ; 
  INT64_COPY(pLocRT->psFeedback->sqMechAbsPosOffset,sqZero64bit) ; 
  
  /* Electrical Angle */
  pLocRT->psFeedback->uwElecAngle = 0 ;
  pLocRT->psFeedback->swElecSpeed = 0 ;
  pLocRT->psFeedback->uwDeltaElecAngle = 0;
    
  pLocRT->psFeedback->sEncData.slSpeed = 0L ;
  pLocRT->psFeedback->sEncData.slAccel = 0L ;

  pLocRT->psFeedback->ubStatus = ENCMGR_NULL ; 
}

/* ######################################################################### */
static BOOL SinCosEnc8KHz(SINCOS_RUNTIME *pSinCosRun)
{
  UWORD uwEncAngle_1 ;
  ULONG ulMechAngleAbsPos_1 ;
  ULONG ulMechAngle_1, slMechSpeed_1 ;
  SLONG slMechSpeed ;
  BOOL  bLevelOk = FALSE;
  ENCMGR_SPACEFEEDBACK *pSinCosOut;

  pSinCosOut=pSinCosRun->psFeedback;

  if (bSysStatAlarmsReset)
  {
    // if was faulty, then reset status
    if(pSinCosRun->sErrorSent.w || *pSinCosRun->psbEncMgrFault)
      initruntime(pSinCosRun);

    pSinCosRun->sErrorSent.w = 0 ; /* clear errors */
    pSinCosRun->swSinCosAlarmCounter = 0 ; 
  }

  // if faulty then exit
  if(pSinCosOut->ubStatus&ENCMGR_FATAL_FAULT)
    return TRUE;

  sSc_DataOut.ulSinCosLevel = (ULONG)((SLONG)sSc_DataOut.swSinChannel * sSc_DataOut.swSinChannel + (SLONG)sSc_DataOut.swCosChannel * sSc_DataOut.swCosChannel) ;

  // controllo livelli Seno/Coseno 
  if ( sSc_DataOut.ulSinCosLevel > pSinCosRun->ulSinCosAlarmThreshold )
  {
    bLevelOk = TRUE;
    
    if(pSinCosRun->swSinCosAlarmCounter > 0)
      pSinCosRun->swSinCosAlarmCounter--;
  }
  else
  {
    pSinCosRun->swSinCosAlarmCounter++;

    if ((pSinCosRun->swSinCosAlarmCounter >= SINCOSALARMCOUNTERMAX) && (!pSinCosRun->sErrorSent.b.bSincosLevel && !pSinCosRun->flags.b.bSin2Cos2ErrorDisabled))
    {
      SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_SINCOS_FAIL, SYSTEMALARMS_SUBCODE_SC_ENCODER_LEVEL, bSysStatPowerEnabled) ;
      
      pSinCosOut->ubStatus = ENCMGR_FATAL_FAULT ;
      pSinCosRun->sErrorSent.b.bSincosLevel = TRUE ;   
          
      return TRUE ;
    }
  }

  /* ============================================ * 
   * ============== Encoder  Angle ============== * 
   * ============================================ */
  uwEncAngle_1 = pSinCosRun->uwEncAngle ; /* previous encoder angle */

  if(pSinCosRun->flags.b.bReverseSignals)
    pSinCosRun->uwEncAngle = 0xffff - ATan16(sSc_DataOut.swSinChannel, sSc_DataOut.swCosChannel) ;
  else 
    pSinCosRun->uwEncAngle = ATan16(sSc_DataOut.swSinChannel, sSc_DataOut.swCosChannel) ;

  if ( (            uwEncAngle_1 & 0x8000) &&  (            uwEncAngle_1 & 0x4000) &&
      !(pSinCosRun->uwEncAngle   & 0x8000) && !(pSinCosRun->uwEncAngle   & 0x4000)   ) 
  {  /* Clockwise Overflow */
    pSinCosRun->ulMechAdder += pSinCosRun->ulMechSteps ;
  } 
  else if (!(            uwEncAngle_1 & 0x8000) && !(            uwEncAngle_1 & 0x4000) &&
            (pSinCosRun->uwEncAngle   & 0x8000) &&  (pSinCosRun->uwEncAngle   & 0x4000)   ) 
  {  /* Anticlockwise Overflow */                              
    pSinCosRun->ulMechAdder -= pSinCosRun->ulMechSteps ;
  }


  /* ============================================ * 
   * ======= Absolute Mechanical Position ======= * 
   * ============================================ */
  ulMechAngleAbsPos_1 = pSinCosRun->sqMechAbsPosition.lo ; /* Previuos abs mechanical angle */
  pSinCosRun->sqMechAbsPosition.lo = ((ULONG)(pSinCosRun->uwEncAngle / pSinCosRun->uwEncPolarPairs) << 16) + pSinCosRun->ulMechAdder ;

  if ( (ulMechAngleAbsPos_1              & 0x80000000) &&  (ulMechAngleAbsPos_1              & 0x40000000) &&
      !(pSinCosRun->sqMechAbsPosition.lo & 0x80000000) && !(pSinCosRun->sqMechAbsPosition.lo & 0x40000000)   ) 
  { /* Mechanical Angle : Clockwise Overflow */
    pSinCosRun->sqMechAbsPosition.hi++ ; /* turns */
  } 
  else if (!(ulMechAngleAbsPos_1              & 0x80000000) && !(ulMechAngleAbsPos_1              & 0x40000000) &&
            (pSinCosRun->sqMechAbsPosition.lo & 0x80000000) &&  (pSinCosRun->sqMechAbsPosition.lo & 0x40000000)   ) 
  { /* Mechanical Angle : Anticlockwise Overflow */
    pSinCosRun->sqMechAbsPosition.hi-- ; /* turns */
  } 


  /* ============================================ * 
   * =========== Module Output Values =========== * 
   * ============================================ */  
  ulMechAngle_1 = pSinCosOut->sEncData.sqPostn.lo ; /* Previous mechanical angle */ 
  slMechSpeed_1 = pSinCosOut->sEncData.slSpeed ;       /* Previuos mechanical speed */  

  /* Current Electrical Angle */
  pSinCosOut->uwElecAngle = (UWORD)((ULONG)HIWORD(pSinCosRun->sqMechAbsPosition.lo) * pSinCosRun->uwMotPolarPairs) + pSinCosOut->uwDeltaElecAngle + sGlbMotorParameters.uwPhaseOffset ;

  /* if first run and valid signal level then get abs pos */
  if(pSinCosOut->ubStatus == ENCMGR_NULL && bLevelOk)
  {
    pSinCosRun->sqMechAbsInitPosition.lo = pSinCosRun->sqMechAbsPosition.lo;
    pSinCosRun->sqMechAbsInitPosition.hi = pSinCosRun->sqMechAbsPosition.hi;

    INT64_COPY(pSinCosOut->sqMechAbsPosOffset, pSinCosRun->sqMechAbsInitPosition) ;

    pSinCosOut->ubStatus = ENCMGR_ABSMECHTURN_VALID | ENCMGR_ELE_ANGLE_VALID | ENCMGR_RELATIVE_VALID | ENCMGR_EFS_READY;
  }    

  /* Current Relative Position */
  _sint64_sub(&pSinCosOut->sEncData.sqPostn, &pSinCosRun->sqMechAbsPosition, &pSinCosRun->sqMechAbsInitPosition) ;

  /* Current Mechanical Speed = MechAngleRel - MechAngleRel_1 (= delta rel position) */
  slMechSpeed = pSinCosOut->sEncData.sqPostn.lo - ulMechAngle_1 ; /* NOT filtered speed */

  /* filtering conditioned by initial sign in order to reduce rounding problem */
  if(pSinCosOut->sEncData.slSpeed>0)
    pSinCosOut->sEncData.slSpeed = (pSinCosOut->sEncData.slSpeed >> 4) * 15 + (slMechSpeed >> 4) ;
  else
    pSinCosOut->sEncData.slSpeed = (-pSinCosOut->sEncData.slSpeed >> 4) * -15 + (slMechSpeed >> 4) ;

  /* Current Mechanical Acceleration = CurrMechSpeed - PrevMechSpeed (= delta speed) */
  pSinCosOut->sEncData.slAccel = _sint32_asl12(pSinCosOut->sEncData.slSpeed - slMechSpeed_1) ;

  /* Current Electrical Speed  = MechSpeed * MotorPolePairs */
  pSinCosOut->swElecSpeed = (SWORD)_sint32_scale_32(pSinCosOut->sEncData.slSpeed, pSinCosRun->uwMotPolarPairs) ;

  return TRUE ;   
}

/* ######################################################################### */
static void Sc_SinCosEncoderBkGd(void)
{
  SinCosEncBkGd(&sSinCosRun, &sSc_SinCosParam) ;
}


/* ######################################################################### */
static void SinCosEncBkGd(SINCOS_RUNTIME *pSinCosRun, SINCOS_PARAMS *pSinCosParam)
{
    UWORD uwLocalSincosAlarmThreshold ;
    ULONG ulLocalSincosAlarmThreshold ;

    // tutto 'sto giro per mantenere le operazioni 'sincrone' (sono in background)
    // il controllo sulle soglie lo faccio sul quadrato e non sulla radice quadrata (per risparmiare tempo ad 8KHz)
    uwLocalSincosAlarmThreshold = pSinCosParam->uwSinCosAlarmThreshold ;
    ulLocalSincosAlarmThreshold = (ULONG)uwLocalSincosAlarmThreshold * uwLocalSincosAlarmThreshold ;
    atomic_write(&(pSinCosRun->ulSinCosAlarmThreshold), &ulLocalSincosAlarmThreshold, sizeof(ULONG)) ;

    // calc and setup resolver frequency offset
    // FPGA unit [25nsec], parameter unit is [1/10usec]
    // (limit is 120usec in order to allow frequency re-sync on the fieldbus)
    if(pSinCosParam->uwResFreqOffset<1200)
        FPGA_ENCODER_RESOLVERDELAY=pSinCosParam->uwResFreqOffset*4;
    else
        FPGA_ENCODER_RESOLVERDELAY=0;
}

/* ######################################################################### */
static void SinCosAnalogSet(SINCOS_RUNTIME *pSinCosRun, SINCOS_PARAMS *pSinCosParam)
{
    ANPROC_CHAN sAInDef;
    HWPRMS_AD_SETTINGS * psGainSin ;
    HWPRMS_AD_SETTINGS * psGainCos ;
    FLOAT flTmp ;
    SLONG slTmp ;
    SWORD swGainSin;
    SWORD swGainCos;
    SWORD swOffsetSin;
    SWORD swOffsetCos;

    // select gain set depending on hardware gain selection (0: HI, 1: LO)
#ifndef _HW_DC
    if(pSinCosRun->flags.b.bAnalogGain)
    {
        psGainSin=&sGlbControlBoardParameters.sStd.sAbsEncG1Sin;
        psGainCos=&sGlbControlBoardParameters.sStd.sAbsEncG1Cos;
    }
    else
    {
        psGainSin=&sGlbControlBoardParameters.sStd.sAbsEncG2Sin;
        psGainCos=&sGlbControlBoardParameters.sStd.sAbsEncG2Cos;
    }
#else
    if(pSinCosRun->flags.b.bAnalogGain)
    {
        psGainSin=&sGlbControlBoardParameters.sUniv.sAbsEncHISin;
        psGainCos=&sGlbControlBoardParameters.sUniv.sAbsEncHICos;
    }
    else
    {
        psGainSin=&sGlbControlBoardParameters.sUniv.sAbsEncLOSin;
        psGainCos=&sGlbControlBoardParameters.sUniv.sAbsEncLOCos;
    }
#endif

    // translate analog channels gains for internal usage (16bit, 8192 = 1.0)
    flTmp=(FLOAT)pSinCosParam->swGainSin*8192.0/1000.0;
    flTmp=flTmp*(FLOAT)psGainSin->swScale/8192.0;
    if(flTmp>32767.0)
        swGainSin=32767;
    else if(flTmp<-32767.0)
        swGainSin=-32767;
    else
        swGainSin=(SWORD)flTmp;

    flTmp=(FLOAT)pSinCosParam->swGainCos*8192.0/1000.0;
    flTmp=flTmp*(FLOAT)psGainCos->swScale/8192.0;
    if(flTmp>32767.0)
        swGainCos=32767;
    else if(flTmp<-32767.0)
        swGainCos=-32767;
    else
        swGainCos=(SWORD)flTmp;

    // translate analog channels offset for internal usage
    slTmp=(SLONG)pSinCosParam->swOffsetSin*32768l/10000l+(SLONG)((SWORD)(psGainSin->uwOffst-0x8000));
    if(slTmp>32767l)
        slTmp=32767l;
    else if(slTmp<-32767l)
        slTmp=-32767l;

    swOffsetSin=0x8000+(SWORD)slTmp;

    slTmp=(SLONG)pSinCosParam->swOffsetCos*32768l/10000l+(SLONG)((SWORD)(psGainCos->uwOffst-0x8000));
    if(slTmp>32767l)
        slTmp=32767l;
    else if(slTmp<-32767l)
        slTmp=-32767l;

    swOffsetCos=0x8000+(SWORD)slTmp;

        // common data for analog input processing
    if(pSinCosRun->flags.b.bEnSinCosHw)
    {   // when only hw channel are needed (plc), heavy filter
        sAInDef.ubOpt=ANPROC_OF_AVG_CONT|ANPROC_OF_BLK_NONE|ANPROC_OF_DST_SHORT;
        sAInDef.ubNumSample=125;
    }
    else
    {   // default
//        sAInDef.ubOpt=ANPROC_OF_AVG_1SHT|ANPROC_OF_BLK_NONE|ANPROC_OF_DST_SHORT;
//        sAInDef.ubNumSample=8;
    	sAInDef.ubOpt=ANPROC_OF_AVG_CONT|ANPROC_OF_BLK_NONE|ANPROC_OF_DST_SHORT;
    	sAInDef.ubNumSample=64;
    }
    sAInDef.uwDstIntImm=ANPROC_ADR_DISABLE;
    sAInDef.uwDstIntAvg=ANPROC_ADR_DISABLE;
    sAInDef.pvDstExtImm=NULL;
    sAInDef.flScale=1.0;

        // prepare sin unit measure conversion
    sAInDef.sCal.uwOffst=swOffsetSin;
    sAInDef.sCal.swScale=swGainSin;
    sAInDef.pvDstExtAvg=&sSc_DataOut.swSinChannel;
    AnProc_Set(FPGA_ADTAGS_ABSENC_SIN,&sAInDef);

        // prepare cos unit measure conversion
    sAInDef.sCal.uwOffst=swOffsetCos;
    sAInDef.sCal.swScale=swGainCos;
    sAInDef.pvDstExtAvg=&sSc_DataOut.swCosChannel;
    AnProc_Set(FPGA_ADTAGS_ABSENC_COS,&sAInDef);
}
#endif
