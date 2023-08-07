/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2010, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : IncrementalEncoder.c                                       */
/* Author      : Cristiano Tognetti                                         */
/*               Fabio Terrile                                              */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/
#include <math.h>
#include <stdlib.h> // to use labs()

#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "system\SysLogManagement.h"
#include "system\SystemAlarms.h"
#include "drive\AxM-E-Defines.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "drive\HardwareUnitID.h"
#include "system\SysAppFpgaOptCodes.h"

#include "common\CommonEncoderManager.h"
#include "drive\IncrementalEncoder.h"
#include "drive\AuxIncrementalCmd.h"

#include "common\TaskScheduler.h"
#include "fpga\AnProcessor.h"

#include "common\MathFunctions.h"
#include "common\Int64Functions.h"
#include "common\DspFunctions.h"
#ifdef _INFINEON_
#include "AlteraFpgaHandler.h" /* needed to access FPGA registers */
#include "dsp/FpgaIRegs.h"
#else
#include "fpga\FpgaHandler.h"
#include "fpga\FpgaIRegs.h"
#endif // _infineon_

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

/* ================================ #define ================================ */
#define ENABLE                      0x8000
#define POSITIVE                    0x0000
#define NEGATIVE                    0x4000

#define INDEX_RESET_STATE           0x80000000
#define INDEX_WDT_TRIP              16

#define RECOVERY_WAIT               10      // msec

#define DIGFILTER_FREQ_MIN          20      // kHz
#define DIGFILTER_FREQ_MAX          2500    // kHz
#define DIGFILTER_FREQ_DEFAULT      1000    // kHz

#define INTERPSWITCH_FREQ_MIN       1       // kHz
#define INTERPSWITCH_FREQ_MAX       1000    // kHz
#define INTERPSWITCH_FREQ_DEFAULT   100     // kHz

#define INTERPSWITCH_HYSTERESIS     50      // [1/1000]

#ifndef _HW_DC
#define GLB_CBRD                        sGlbControlBoardParameters.sStd
#else
#define GLB_CBRD                        sGlbControlBoardParameters.sUniv
#endif

#define FASTSIM_RATIO_MAX           12

/* ============================== structures =============================== */
typedef struct {
  UWORD uwNbit ;
  UWORD uwRightShift ;
  ULONG ulNormFactor ;
} INCREMENTAL_NORMPARS ;

typedef struct {
 /* Flags */
  union {
      struct {
        UWORD bSinCosInterpolation   : 1 ; // 0 => DO NOT USE SinCosInterpolation; 1 => USE SinCosInterpolation
        UWORD bIndexErrorDisabled    : 1 ; // 0 => index error check enabled; 1 => index error check disabled
        UWORD bSin2Cos2ErrorDisabled : 1 ; // 0 => sin^2 + cos^2 error check enabled; 1 => sin^2 + cos^2 error check disabled
        UWORD bHasIndex              : 1 ; // 1 => encoder has valid index track
        UWORD bAuxEnc                : 1 ; // 0 => used as MAIN encoder; 1 => used as Auxiliary Encoder
        UWORD bHasAnalog             : 1 ; // 1 if has analog sin/cos 
        UWORD bEnInterpolation       : 1 ; // 1 to enable digital or analog interpolation
        UWORD bDisIdxEAngle          : 1 ; // 1 => disable index track for electrical angle adjustment
        UWORD bDisIdxPosition        : 1 ; // 1 => disable index track for mechanical position adjustment
        UWORD bCaptureAllIndexes     : 1 ; // 1 => enable capturing of all indexes, unregarding index tolerance
        UWORD bSuspendProcessing     : 1 ; // 1 => suspend encoder processing
        UWORD bDisDigInterpolation   : 1 ; // 1 => disable digital periodimeter
        UWORD bEnIncrementalHw       : 1 ; // 1 => enable only hardware analog channels
      } b ;
      UWORD w ;
  } flags ;

  union {
      struct {
        UWORD bFpgaFault        : 1 ; 
        UWORD bCountFault       : 1 ;
        UWORD bSincosLevel      : 1 ;
        UWORD bSimMaxDiffExceed : 1 ;
      } b ;
      UWORD w ;
  } sErrorSent ;  

  ULONG ulTotEncPulse ;
  INCREMENTAL_NORMPARS np ;

  SBYTE bDirection_1 ;

  SBYTE bInitReq ;
  SBYTE bOnAnInterp ;
  SBYTE bEnAnInterp ;

  UBYTE ubFirstIndexRead ;
  UBYTE ubIndexWatchDog ;
  UWORD uwEnAnIntTmr ;
  ULONG ulCountIndex_1;
  ULONG ulFullIdxCapture;
  ULONG ulFullCntCapture;
  
  ULONG ulSinCosAlarmThreshold ;
  ULONG ulIndexTolerance ;

  ULONG ulInterpSpeedThresholdLo ;
  ULONG ulInterpSpeedThresholdHi ;

  /* Motor Parameters */
  UWORD uwMotPolarPairs  ;

  ENCMGR_SPACEFEEDBACK * psFeedBack;
  
  ULONG ulDeltaCounter;

  ULONG ulBaseAddress;

  INCREMENTAL_OUT * psDataOut;
  
  SBYTE * psbEncMgrFault;

} INCREMENTAL_RUNTIME ;

typedef struct {
  union {
    struct {
      UBYTE bFirstIndexCalculated : 1 ;
      UBYTE bEnFastAuxSim : 1 ;
      UBYTE bBlindAuxSim : 1 ;
    } b ;
    UBYTE w ;
  } flags ;
  union {
    struct {
      UBYTE bSimMaxDiffExceed : 1 ;
    } b ;
    UBYTE w ;
  } sErrorSent ;

  ULONG ulIdxTotPulseRpt ;
  ULONG ulIdxHalfPulseRpt ;
  SLONG slDeltaAngleFpgaCnts_1 ;
  SLONG slFpgaAuxOutIdxOffset ;
  ULONG slFpgaAuxOutIdx;

  INCREMENTAL_NORMPARS np ;

  ULONG ulTotEncPulse;
  UWORD uwDigFilterValue;
  ULONG ulDeltaCounter;

  ENCMGR_SPACEFEEDBACK *psInputEncoder ;

  UBYTE * pubStatus;
  
  UBYTE ubFastRatio;
} AUXSIM_RUN ;


/* =========================== global  variables =========================== */
INCREMENTAL_PARAMS sIc_IncEncParams[INCREMENTAL_MAX_ENCODER] ;
INCREMENTAL_OUT sIc_IncEncDataOut[INCREMENTAL_MAX_ENCODER] ;

AUXSIM_PARAMS sSe_AuxSimParam ;
AUXSIM_OUT sSe_AuxSimOut = { FALSE, AUXINCRCMD_INACTIVE };


// default
#ifdef _INFINEON_
const INCREMENTAL_PARAMS huge sIc_IncEncDefParams =
#else
const INCREMENTAL_PARAMS sIc_IncEncDefParams = 
#endif // _infineon_
{
    {0,0,0,0,0,0,0,0,0,0,0,0},  // Flags
    2,          // Poles Number
    512,        // Line Counts
    5000,       // Sincos Alarm 
    10,         // Index Tolerance
    DIGFILTER_FREQ_DEFAULT,
    1000,       // gain = 1.0
    1000,       // gain = 1.0
    0,
    0,
    INTERPSWITCH_FREQ_DEFAULT,
} ;
#ifdef _INFINEON_
const AUXSIM_PARAMS huge sSe_AuxSimDefParam =
#else
const AUXSIM_PARAMS sSe_AuxSimDefParam =
#endif // _infineon_
{
    512,        // index pulse repetition
    0,          // index offset
    2730,       // alarm threshold, 15deg
    512,        // line counts
    DIGFILTER_FREQ_DEFAULT,
    {0,0},
} ;
  
#if CFG_ENC_INCR
/* =========================== local  variables =========================== */
static INCREMENTAL_RUNTIME  sIncEncMainRun ; /* Incremental Encoder as Main Encoder */
static INCREMENTAL_RUNTIME  sIncEncAuxRun ;  /* Incremental Encoder as Auxiliary Encoder */

#ifdef _INFINEON_
static BOOL bdata sIncEncBgInstalled=FALSE;
static AUXSIM_RUN sdata sAuxSimRun ;
#else
static BOOL sIncEncBgInstalled=FALSE;
static AUXSIM_RUN sAuxSimRun ;
#endif // _infineon_

/* =============================== functions =============================== */ 
static ULONG IncEncInit(ENCMGR_SPACEFEEDBACK *pIncrOut, INCREMENTAL_RUNTIME *pIncrRun, INCREMENTAL_PARAMS *pIncrParam, BOOL bAuxEnc, ULONG ulBaseAddress, INCREMENTAL_OUT * psDOut, SBYTE * psbEncMgrFault) ;
static void analogsetup(INCREMENTAL_PARAMS *pIncrParam, INCREMENTAL_RUNTIME *pIncrRun);
static void initruntime(INCREMENTAL_RUNTIME * pLocRTime);
static void normcomp(ULONG ulPulses, INCREMENTAL_NORMPARS * pNP);
static BOOL IncEnc8KHzPlcOnly(INCREMENTAL_RUNTIME *pIncrRun);
static BOOL IncEnc8KHz(INCREMENTAL_RUNTIME *pIncrRun) ;
static void slowtask(void);
static void paramcalc(INCREMENTAL_RUNTIME *pIncrRun, INCREMENTAL_PARAMS *pIncrParam);
static void indexmonitor(INCREMENTAL_RUNTIME *pIncrRun);
static BOOL IncrementalAuxSimulation8KHz(void) ;

/* ######################################################################### * 
 * ############################  INIT  ENCODER  ############################ * 
 * ######################################################################### */
BOOL Ic_EncoderInit(UWORD uwChannelSel, ENCMGR_SPACEFEEDBACK * psFeedback, SBYTE * psbEncMgrFault)
{
  ULONG ulRetVal;
  BOOL (* rtf)(INCREMENTAL_RUNTIME *);

  switch(uwChannelSel)
  {
    case INCREMENTAL_SEL_MAIN:
      ulRetVal = IncEncInit(psFeedback, &sIncEncMainRun, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN], FALSE, FPGA_DIGENC_BASEADDR, &sIc_IncEncDataOut[INCREMENTAL_SEL_MAIN], psbEncMgrFault);
      if ( ulRetVal )
      {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL, SYSTEMALARMS_SUBCODE_ENC_MAIN | ulRetVal, FALSE);
        /* Inizializzazione non è andata bene: non installo la funzione 8KHz e segnalo l'errore */
        return FALSE ;
      }

      if(psFeedback)
        rtf = &IncEnc8KHz;
      else
        rtf = &IncEnc8KHzPlcOnly;

      if(!TaskSched_AddRTTaskEx((BOOL (*)(UWORD))rtf, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0, (UWORD)&sIncEncMainRun))
        return FALSE;

      break;

    case INCREMENTAL_SEL_AUX:
      ulRetVal = IncEncInit(psFeedback, &sIncEncAuxRun, &sIc_IncEncParams[INCREMENTAL_SEL_AUX], TRUE, FPGA_INCAUX_BASEADDR, &sIc_IncEncDataOut[INCREMENTAL_SEL_AUX], psbEncMgrFault);
      if ( ulRetVal )
      {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL, SYSTEMALARMS_SUBCODE_ENC_AUX | ulRetVal, FALSE);
        /* Inizializzazione non è andata bene: non installo la funzione 8KHz e segnalo l'errore */
        return FALSE ;
      }
    
      if(psFeedback)
        rtf = &IncEnc8KHz;
      else
        rtf = &IncEnc8KHzPlcOnly;

      if(!TaskSched_AddRTTaskEx((BOOL (*)(UWORD))rtf, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0, (UWORD)&sIncEncAuxRun))
        return FALSE;

      /* enable remdisp command switch */
      sSe_AuxSimOut.sbReqCmd=AUXINCRCMD_IDLE;

      break;

    default:
      assert(FALSE);
      return FALSE;
  }

    // install only one background for all
  if(!sIncEncBgInstalled)
  {
    if(!TaskSched_AddBackgroundTask(&slowtask))
        return FALSE ; 
    sIncEncBgInstalled=TRUE;
  }

  return TRUE;
}

/* ######################################################################### */
void Ic_GetHwOpt(UWORD uwChannelSel, ULONG * pulHwOpt, ULONG * pulFPGAOpt)
{
  switch(uwChannelSel)
  {
    case INCREMENTAL_SEL_MAIN:
      *pulHwOpt  |=HWUNITID_ENCM_INC_DIG;
      if(sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bSinCosInterpolation)
        *pulHwOpt  |=HWUNITID_ENCM_INC_AN;
      *pulFPGAOpt|=FPGA_HW_ENCMAINSINCOSINC;
      break;

    case INCREMENTAL_SEL_AUX:
      *pulHwOpt  |=HWUNITID_ENCA_INC_DIG;
      *pulFPGAOpt|=FPGA_HW_AUXIN;
      break;
  }
}

/* ######################################################################### */
static ULONG IncEncInit(ENCMGR_SPACEFEEDBACK *pIncrOut, INCREMENTAL_RUNTIME *pIncrRun, INCREMENTAL_PARAMS *pIncrParam, BOOL bAuxEnc, ULONG ulBaseAddress, INCREMENTAL_OUT * psDOut, SBYTE * psbEncMgrFault)
{
  ULONG ulLineCounts ;

  FPGA_INCENC_RESET(ulBaseAddress) = TRUE  ; /* reset encoder */

  pIncrRun->psbEncMgrFault = psbEncMgrFault;

  pIncrRun->flags.b.bAuxEnc = bAuxEnc ; /* memorize if Main or Auxiliary encoder */

  pIncrRun->psFeedBack=pIncrOut;
  pIncrRun->psDataOut=psDOut;
  pIncrRun->ulBaseAddress=ulBaseAddress;
  ulLineCounts = pIncrParam->ulLineCounts ;

  /* ===================================================================== */

  if(pIncrRun->flags.b.bAuxEnc)
  { // se ausiliario, no interpolazione sin-cos (ed ovviamente nessun errore)
    pIncrRun->flags.b.bSinCosInterpolation   = FALSE ;
    pIncrRun->flags.b.bSin2Cos2ErrorDisabled = TRUE ;
    pIncrRun->flags.b.bHasAnalog = FALSE;
  }
  else
  {
    if(pIncrParam->flags.b.bEnableStepDir || pIncrParam->flags.b.bEnableUpDown)
    { // se step/dir oppure up/down no interpolazione
      pIncrRun->flags.b.bSinCosInterpolation   = FALSE ;
      pIncrRun->flags.b.bSin2Cos2ErrorDisabled = TRUE ;
    }
    else
    {
      pIncrRun->flags.b.bSinCosInterpolation   = pIncrParam->flags.b.bSinCosInterpolation ;
      pIncrRun->flags.b.bSin2Cos2ErrorDisabled = pIncrParam->flags.b.bSin2Cos2ErrorDisabled ;
    }
    pIncrRun->flags.b.bHasAnalog = TRUE;

    // enable and use incremental hardware ONLY
    if (pIncrOut == NULL) 
        pIncrRun->flags.b.bEnIncrementalHw = TRUE ;
    else
        pIncrRun->flags.b.bEnIncrementalHw = FALSE ;

    analogsetup(pIncrParam, pIncrRun);
  }

  pIncrRun->flags.b.bDisDigInterpolation = pIncrParam->flags.b.bDisDigInterpolation;
  pIncrRun->flags.b.bEnInterpolation = pIncrRun->flags.b.bSinCosInterpolation || !pIncrRun->flags.b.bDisDigInterpolation;

  FPGA_INCENC_IGNORE_SINCOS(ulBaseAddress) = TRUE ;
  FPGA_INCENC_ENABLE_ADFLT(ulBaseAddress)  = !(pIncrRun->flags.b.bSin2Cos2ErrorDisabled) ;
  FPGA_INCENC_DIS_WDTFAULT(ulBaseAddress)  = pIncrParam->flags.b.bWdtFiltErrorDisabled;
   
  FPGA_INCENC_SWAPINPUTS(ulBaseAddress)    = pIncrParam->flags.b.bSwapTracks;
  FPGA_INCENC_EN_STEPDIR(ulBaseAddress)    = pIncrParam->flags.b.bEnableStepDir;
  FPGA_INCENC_EN_UPDOWN(ulBaseAddress)     = pIncrParam->flags.b.bEnableUpDown;
  FPGA_INCENC_DIS_IDXABSYNC(ulBaseAddress) = pIncrParam->flags.b.bDisIdxABSync ;
  FPGA_INCENC_DIS_IDXDYNFILTER(ulBaseAddress) = pIncrParam->flags.b.bDisIdxABSync ;

  /* ===================================================================== */
  /* setting up shift and normalization factor (used in Task8KHz) */
  if(!(pIncrParam->flags.b.bEnableStepDir || pIncrParam->flags.b.bEnableUpDown))
    pIncrRun->ulTotEncPulse = (ulLineCounts * 4) ;        /* Total Incremental Encoder Pulses = 4 x Encoder Pulse */
  else
    pIncrRun->ulTotEncPulse = ulLineCounts ;              /* same linecount for step/dir and up/down */

  normcomp(pIncrRun->ulTotEncPulse, &(pIncrRun->np));

  if(!pIncrParam->flags.b.bSinCosInterpolation) 
    pIncrRun->np.uwNbit+=2;                                  /* /4 the interpolation calculation */
  /* ===================================================================== */

  pIncrRun->flags.b.bHasIndex              = pIncrParam->flags.b.bHasIndex ;
  pIncrRun->flags.b.bIndexErrorDisabled    = pIncrParam->flags.b.bIndexErrorDisabled ;
  pIncrRun->flags.b.bDisIdxEAngle          = pIncrParam->flags.b.bDisIdxEAngle ;
  pIncrRun->flags.b.bDisIdxPosition        = pIncrParam->flags.b.bDisIdxPosition ;
  pIncrRun->flags.b.bCaptureAllIndexes     = pIncrParam->flags.b.bCaptureAllIndexes ;

  pIncrRun->ulIndexTolerance = (ULONG)pIncrParam->uwIndexTolerance * 4 ; // << 2 perche' ulTotEncPulse = ulLineCounts * 4
 
  /* Electrical Angle */
  pIncrRun->uwMotPolarPairs  = sGlbMotorParameters.uwPoleNumbers / 2 ;

  /* Prepare remaining runtime parameters */
  paramcalc(pIncrRun, pIncrParam);

  /* at first slowtask init encoder */
  pIncrRun->bInitReq = TRUE;
  
  return 0l ;
}

/* ######################################################################### */
static void analogsetup(INCREMENTAL_PARAMS *pIncrParam, INCREMENTAL_RUNTIME *pIncrRun)
{
    ANPROC_CHAN sAInDef;
    FLOAT flTmp ;
    SLONG slTmp ;
    SWORD swGainSin, swGainCos ;
    UWORD uwOfstSin, uwOfstCos ;

    if(pIncrParam->flags.b.bDisFactoryCalibr)
    {
      uwOfstSin = 0x8000;
      uwOfstCos = 0x8000;
      swGainSin = 0x2000;
      swGainCos = 0x2000;
    }
    else
    {
      uwOfstSin = GLB_CBRD.sIncEncSin.uwOffst;
      uwOfstCos = GLB_CBRD.sIncEncCos.uwOffst;
      swGainSin = GLB_CBRD.sIncEncSin.swScale;
      swGainCos = GLB_CBRD.sIncEncCos.swScale;
    }

      // translate analog channels gains for internal usage (16bit, 8192 = 1.0)
    flTmp=(FLOAT)pIncrParam->swGainSin*8192.0/1000.0;
    flTmp=flTmp*(FLOAT)swGainSin/8192.0;
    if(flTmp>32767.0)
        swGainSin=32767;
    else if(flTmp<-32767.0)
        swGainSin=-32767;
    else
        swGainSin=(SWORD)flTmp;

    flTmp=(FLOAT)pIncrParam->swGainCos*8192.0/1000.0;
    flTmp=flTmp*(FLOAT)swGainCos/8192.0;
    if(flTmp>32767.0)
        swGainCos=32767;
    else if(flTmp<-32767.0)
        swGainCos=-32767;
    else
        swGainCos=(SWORD)flTmp;

    // translate analog channels offset for internal usage
    slTmp=(SLONG)pIncrParam->swOffsetSin*32768l/10000l+(SLONG)((SWORD)(uwOfstSin-0x8000));
    if(slTmp>32767l)
        slTmp=32767l;
    else if(slTmp<-32767l)
        slTmp=-32767l;

    uwOfstSin=0x8000+(SWORD)slTmp;

    slTmp=(SLONG)pIncrParam->swOffsetCos*32768l/10000l+(SLONG)((SWORD)(uwOfstCos-0x8000));
    if(slTmp>32767l)
        slTmp=32767l;
    else if(slTmp<-32767l)
        slTmp=-32767l;

    uwOfstCos=0x8000+(SWORD)slTmp;

    if (pIncrRun->flags.b.bEnIncrementalHw)
    {     // enable only incremental hw analog channels (plc)
        sAInDef.ubOpt=ANPROC_OF_AVG_CONT|ANPROC_OF_BLK_NONE|ANPROC_OF_DST_SHORT;
        sAInDef.ubNumSample=125;
        sAInDef.uwDstIntImm=ANPROC_ADR_DISABLE;
        sAInDef.pvDstExtImm=NULL;
        sAInDef.pvDstExtAvg=NULL;
        sAInDef.flScale=1.0;

          // prepare sin unit measure conversion
        sAInDef.sCal.uwOffst=uwOfstSin;
        sAInDef.sCal.swScale=swGainSin;
        sAInDef.uwDstIntAvg=FPGAIR_MINC_SIN;
        AnProc_Set(FPGA_ADTAGS_INCENC_SIN,&sAInDef);
    
          // prepare cos unit measure conversion
        sAInDef.sCal.uwOffst=uwOfstCos;
        sAInDef.sCal.swScale=swGainCos;
        sAInDef.uwDstIntAvg=FPGAIR_MINC_COS;
        AnProc_Set(FPGA_ADTAGS_INCENC_COS,&sAInDef);
    }
    else
    {     // common data for analog input processing
        sAInDef.ubOpt=ANPROC_OF_AVG_NONE|ANPROC_OF_BLK_NONE;
        sAInDef.ubNumSample=0;
        sAInDef.uwDstIntAvg=ANPROC_ADR_DISABLE;
        sAInDef.pvDstExtImm=NULL;
        sAInDef.pvDstExtAvg=NULL;
        sAInDef.flScale=1.0;

          // prepare sin unit measure conversion
        sAInDef.sCal.uwOffst=uwOfstSin;
        sAInDef.sCal.swScale=swGainSin;
        sAInDef.uwDstIntImm=FPGAIR_MINC_SIN;
        AnProc_Set(FPGA_ADTAGS_INCENC_SIN,&sAInDef);
    
          // prepare cos unit measure conversion
        sAInDef.sCal.uwOffst=uwOfstCos;
        sAInDef.sCal.swScale=swGainCos;
        sAInDef.uwDstIntImm=FPGAIR_MINC_COS;
        AnProc_Set(FPGA_ADTAGS_INCENC_COS,&sAInDef);
    }
}

/* ######################################################################### */
static void initruntime(INCREMENTAL_RUNTIME * pLocRTime)
{
    // encoder reset
  FPGA_INCENC_RESET(pLocRTime->ulBaseAddress) = TRUE;
  FPGA_INCENC_RESET(pLocRTime->ulBaseAddress) = FALSE;
  FPGA_INCENC_IGNORE_SINCOS(pLocRTime->ulBaseAddress) = TRUE ;

  INT64_COPY(pLocRTime->psFeedBack->sEncData.sqPostn,    sqZero64bit) ; 
  INT64_COPY(pLocRTime->psFeedBack->sqMechAbsPosOffset,  sqZero64bit) ; 

  pLocRTime->psFeedBack->uwElecAngle = 0 ;
  pLocRTime->psFeedBack->swElecSpeed = 0 ;
  pLocRTime->psFeedBack->uwDeltaElecAngle = 0;      

  pLocRTime->psFeedBack->sEncData.slSpeed = 0L ; 
  pLocRTime->psFeedBack->sEncData.slAccel = 0L ;

  if(pLocRTime->flags.b.bHasIndex && !pLocRTime->flags.b.bDisIdxPosition)
    pLocRTime->psFeedBack->ubStatus = ENCMGR_ABSMECHTURN_WAITING;
  else
    pLocRTime->psFeedBack->ubStatus = ENCMGR_NULL;

  pLocRTime->ubFirstIndexRead = FALSE ;
  pLocRTime->bOnAnInterp      = FALSE;
  pLocRTime->bEnAnInterp      = FALSE;
  pLocRTime->ubIndexWatchDog  = 0;
  pLocRTime->ulCountIndex_1   = 0ul ;
  pLocRTime->ulFullIdxCapture = 0ul ;

  pLocRTime->psDataOut->ulIndexCapture = INDEX_RESET_STATE;
  pLocRTime->psDataOut->ubIndexCapCounter = 0;

  timer_wait(uwSysTimers1ms,RECOVERY_WAIT);

  pLocRTime->uwEnAnIntTmr=timer_settimeout(uwSysTimers1ms,RECOVERY_WAIT);
  pLocRTime->bInitReq=FALSE;
}

/* ######################################################################### */
static void normcomp(ULONG ulPulses, INCREMENTAL_NORMPARS * pNP)
{
  DOUBL dbNormFactor64;
  pNP->uwNbit       = _ilog2(ulPulses - 1) ; /* position of the highest bit set */
  pNP->uwRightShift = pNP->uwNbit - 1 ;      /* Left shift necessary: 32 - uwNbit; + 1 since NormFactor is multiplied for 2^31 */   
  // NormFactor64 = (2^31 * 2^Nbit) / (LineCounts * 4) = 2^(31 + Nbit) / (LineCounts * 4) 
  dbNormFactor64    = (2147483648.0 * ((DOUBL)(1UL << pNP->uwNbit)))/((DOUBL)ulPulses) ; 
  pNP->ulNormFactor = (ULONG)dbNormFactor64 ;
}

/* ######################################################################### */
BOOL Ic_EncoderParametersCheck(UWORD uwChannelSel)
{
    BOOL bValid=TRUE;

    if(sIc_IncEncParams[uwChannelSel].ulLineCounts==0)
    {
        ParChk_SignalValueError(uwChannelSel==INCREMENTAL_SEL_MAIN?PARCC_INCR_MAINLINENUMBERS:PARCC_INCR_AUXLINENUMBERS);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(uwChannelSel==INCREMENTAL_SEL_MAIN?PARCC_INCR_MAINLINENUMBERS:PARCC_INCR_AUXLINENUMBERS);

    if((ULONG)sIc_IncEncParams[uwChannelSel].uwIndexTolerance >= sIc_IncEncParams[uwChannelSel].ulLineCounts)
    {
        ParChk_SignalValueError(uwChannelSel==INCREMENTAL_SEL_MAIN?PARCC_INCR_MAININDEXTOLERANCE:PARCC_INCR_AUXINDEXTOLERANCE);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(uwChannelSel==INCREMENTAL_SEL_MAIN?PARCC_INCR_MAININDEXTOLERANCE:PARCC_INCR_AUXINDEXTOLERANCE);

    if(sIc_IncEncParams[uwChannelSel].uwMaxFrequency && (sIc_IncEncParams[uwChannelSel].uwMaxFrequency < DIGFILTER_FREQ_MIN || 
                                                         sIc_IncEncParams[uwChannelSel].uwMaxFrequency > DIGFILTER_FREQ_MAX))
    {
        ParChk_SignalValueError(uwChannelSel==INCREMENTAL_SEL_MAIN?PARCC_INCR_MAINMAXFREQUENCY:PARCC_INCR_AUXMAXFREQUENCY);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(uwChannelSel==INCREMENTAL_SEL_MAIN?PARCC_INCR_MAINMAXFREQUENCY:PARCC_INCR_AUXMAXFREQUENCY);

    if(sIc_IncEncParams[uwChannelSel].uwInterpSwitchFreq < INTERPSWITCH_FREQ_MIN || 
       sIc_IncEncParams[uwChannelSel].uwInterpSwitchFreq > INTERPSWITCH_FREQ_MAX)
    {
        ParChk_SignalValueError(uwChannelSel==INCREMENTAL_SEL_MAIN?PARCC_INCR_MAININTERPSWITCHFREQ:PARCC_INCR_AUXINTERPSWITCHFREQ);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(uwChannelSel==INCREMENTAL_SEL_MAIN?PARCC_INCR_MAININTERPSWITCHFREQ:PARCC_INCR_AUXINTERPSWITCHFREQ);

    return bValid;
}

/* ######################################################################### */
void Ic_EncoderSetElecAngle(UWORD uwElecAngle)
{
    sIncEncMainRun.psFeedBack->uwDeltaElecAngle=uwElecAngle-(UWORD)((ULONG)HIWORD(sIncEncMainRun.psFeedBack->sEncData.sqPostn.lo) * sIncEncMainRun.uwMotPolarPairs)-sGlbMotorParameters.uwPhaseOffset;
    sIncEncMainRun.psFeedBack->ubStatus |= ENCMGR_ELE_ANGLE_VALID ;
}

/* ######################################################################### */
void Ic_EncoderSetAbsPos(SQWRD sqAbsPos)
{
    if(!(sIncEncMainRun.psFeedBack->ubStatus & ENCMGR_ABSMECHTURN_VALID))
    {
        sIncEncMainRun.psFeedBack->sqMechAbsPosOffset.lo = sqAbsPos.lo;
        sIncEncMainRun.psFeedBack->sqMechAbsPosOffset.hi = sqAbsPos.hi;
        sIncEncMainRun.psFeedBack->ubStatus |= ENCMGR_ABSMECHTURN_VALID;
    }
}

/* ######################################################################### */
BOOL Ic_EncoderGetSnapshot(ULONG * pulSnapshot)
{
    ULONG ulSnapshot;
    UQWRD uqProduct64;

    // prepare with actual delta counter
#ifdef _INFINEON_
    _atomic_( 0 ); // defined in INTRINS.H C166
    ulSnapshot=sIncEncMainRun.ulDeltaCounter;
    _endatomic_();
#else
    ulSnapshot=sIncEncMainRun.ulDeltaCounter;
#endif

    // add last snapshot
    ulSnapshot+=FPGA_BASEOFF_32(sIncEncMainRun.ulBaseAddress, FPGA_INCENC_CNTSNAPSHOT32);

    // refine removing # of turns
    while((SLONG)ulSnapshot<0)
        ulSnapshot+=sIncEncMainRun.ulTotEncPulse;
    while((SLONG)ulSnapshot>=sIncEncMainRun.ulTotEncPulse)
        ulSnapshot-=sIncEncMainRun.ulTotEncPulse;

    // and normalize
    _uint64_mul_32_32(&uqProduct64, &ulSnapshot, &sIncEncMainRun.np.ulNormFactor);
    _uint64_shr(&uqProduct64, sIncEncMainRun.np.uwRightShift) ; 

    // return normalized position
    *pulSnapshot = INT64_LLONG(uqProduct64);
    return TRUE;
}

/* ######################################################################### */
static void slowtask(void)
{
    paramcalc(&sIncEncMainRun, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN]);
    paramcalc(&sIncEncAuxRun, &sIc_IncEncParams[INCREMENTAL_SEL_AUX]);

    if(sSe_AuxSimOut.sbReqCmd==AUXINCRCMD_MASKALARMS)
    {
      sIncEncAuxRun.flags.b.bSuspendProcessing=TRUE;
      sSe_AuxSimOut.sbReqCmd=AUXINCRCMD_IDLE;
    }
    if(sSe_AuxSimOut.sbReqCmd==AUXINCRCMD_REQINIT)
    {
      sIncEncAuxRun.bInitReq=TRUE;
      sIncEncAuxRun.flags.b.bSuspendProcessing=FALSE;
    }

    if(sIncEncMainRun.psFeedBack)
    {
      if(sIncEncMainRun.bInitReq)
        initruntime(&sIncEncMainRun);
      else
      {
        indexmonitor(&sIncEncMainRun);

        if(timer_istimedout(uwSysTimers1ms, sIncEncMainRun.uwEnAnIntTmr))
          sIncEncMainRun.bEnAnInterp = (SBYTE)sIncEncMainRun.flags.b.bSinCosInterpolation;
      }
    }

    if(sIncEncAuxRun.psFeedBack)
    {
      if(sIncEncAuxRun.bInitReq)
      {
        initruntime(&sIncEncAuxRun);
        if(sSe_AuxSimOut.sbReqCmd==AUXINCRCMD_REQINIT)
          sSe_AuxSimOut.sbReqCmd=AUXINCRCMD_IDLE;
      }
      else
        indexmonitor(&sIncEncAuxRun);
    }
}

/* ######################################################################### */
static void paramcalc(INCREMENTAL_RUNTIME *pIncrRun, INCREMENTAL_PARAMS *pIncrParam)
{
    UWORD uwLocalSincosAlarmThreshold ;
    ULONG ulLocalSincosAlarmThreshold ;
    UWORD uwFiltValue ;

    // tutto 'sto giro per mantenere le operazioni 'sincrone' (sono in background)
    // il controllo sulle soglie lo faccio sul quadrato e non sulla radice quadrata (per risparmiare tempo ad 8KHz)
    uwLocalSincosAlarmThreshold = pIncrParam->uwSinCosAlarmThreshold ;
    ulLocalSincosAlarmThreshold = (ULONG)uwLocalSincosAlarmThreshold * uwLocalSincosAlarmThreshold ;
    atomic_write(&(pIncrRun->ulSinCosAlarmThreshold), &ulLocalSincosAlarmThreshold, sizeof(ULONG)) ;

    // atomically get and handle max frequency
    uwFiltValue = pIncrParam->uwMaxFrequency;

    if((uwFiltValue>=DIGFILTER_FREQ_MIN) && (uwFiltValue<=DIGFILTER_FREQ_MAX) || (uwFiltValue==0))
    {
        if(uwFiltValue==0)
            uwFiltValue = DIGFILTER_FREQ_DEFAULT;

        uwFiltValue = 20000 / uwFiltValue;  // @ module freq 40Mhz
        FPGA_BASEOFF_16(pIncrRun->ulBaseAddress, FPGA_INCENC_DIG_FILTER) = uwFiltValue;
    }

    // atomically get and handle interpolation switch frequency
    uwFiltValue = pIncrParam->uwInterpSwitchFreq;
    if(uwFiltValue>=INTERPSWITCH_FREQ_MIN && uwFiltValue<=INTERPSWITCH_FREQ_MAX)
    {
        // recover line counts from ulTotEncPulse
        if(!(pIncrParam->flags.b.bEnableStepDir || pIncrParam->flags.b.bEnableUpDown))
            ulLocalSincosAlarmThreshold = 4 ;
        else
            ulLocalSincosAlarmThreshold = 1 ;

        // calculate d.u. speed from frequency
        ulLocalSincosAlarmThreshold = (ULONG)((FLOAT)uwFiltValue*1000.0/ \
                                      (pIncrRun->ulTotEncPulse/ulLocalSincosAlarmThreshold)* \
                                      (2.0*FLOAT_PI)*SPEED_RADS_K_CONVERSION*((1000.0+INTERPSWITCH_HYSTERESIS)/1000.0));
        atomic_write(&(pIncrRun->ulInterpSpeedThresholdHi), &ulLocalSincosAlarmThreshold, sizeof(ULONG)) ;

        ulLocalSincosAlarmThreshold = (ULONG)((FLOAT)ulLocalSincosAlarmThreshold/((1000.0+INTERPSWITCH_HYSTERESIS)/1000.0)*
                                      ((1000.0-INTERPSWITCH_HYSTERESIS)/1000.0));
        atomic_write(&(pIncrRun->ulInterpSpeedThresholdLo), &ulLocalSincosAlarmThreshold, sizeof(ULONG)) ;
}
}

/* ######################################################################### */
static void indexmonitor(INCREMENTAL_RUNTIME *pIncrRun)
{
    ULONG ulCountActual;
    ULONG ulFullIndex;

    // if has index and index error not disabled
    if(pIncrRun->flags.b.bHasIndex && !pIncrRun->flags.b.bIndexErrorDisabled)
    {
#ifdef _INFINEON_
        _atomic_(0);
        ulCountActual = pIncrRun->ulFullCntCapture;
        ulFullIndex = pIncrRun->ulFullIdxCapture;
        _endatomic_();
#else
        ulCountActual = pIncrRun->ulFullCntCapture;
        ulFullIndex = pIncrRun->ulFullIdxCapture;
#endif
        // if last captured index is far from actual position more than 16 turns
        if((labs(ulCountActual - ulFullIndex)/16 > pIncrRun->ulTotEncPulse ||
            // or number of false index more than threshold
                pIncrRun->ubIndexWatchDog >= INDEX_WDT_TRIP) &&
            // and not already sent alarm
            !pIncrRun->sErrorSent.b.bCountFault && (pIncrRun->psFeedBack->ubStatus&ENCMGR_FATAL_FAULT)==0)
        {
            if (pIncrRun->flags.b.bAuxEnc)
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL, SYSTEMALARMS_SUBCODE_ENC_AUX  | SYSTEMALARMS_SUBCODE_IN_INV_LINE_COUNT, bSysStatPowerEnabled) ;
            else
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL, SYSTEMALARMS_SUBCODE_ENC_MAIN | SYSTEMALARMS_SUBCODE_IN_INV_LINE_COUNT, bSysStatPowerEnabled) ;

#ifdef _INFINEON_
            _atomic_(0);          
            pIncrRun->sErrorSent.b.bCountFault = TRUE ;  
            _endatomic_();

            _atomic_(0);          
            pIncrRun->psFeedBack->ubStatus = ENCMGR_FATAL_FAULT ;
            _endatomic_();
#else
            pIncrRun->sErrorSent.b.bCountFault = TRUE ;
            pIncrRun->psFeedBack->ubStatus = ENCMGR_FATAL_FAULT ;
#endif // _infineon_
        }
    }
}

/* ######################################################################### */
static BOOL IncEnc8KHzPlcOnly(INCREMENTAL_RUNTIME *pIncrRun)
{
  INCREMENTAL_OUT *pDataOut=pIncrRun->psDataOut;
  ULONG ulBaseAddress=pIncrRun->ulBaseAddress;

  // always refresh analog data if apply
  if(pIncrRun->flags.b.bHasAnalog)
  {
    pDataOut->swSin=FPGA_BASEOFF_16(ulBaseAddress, FPGA_INCENC_SIN) ;
    pDataOut->swCos=FPGA_BASEOFF_16(ulBaseAddress, FPGA_INCENC_COS) ;
  }

  return TRUE;
}

/* ######################################################################### */
static BOOL IncEnc8KHz(INCREMENTAL_RUNTIME *pIncrRun)
{
  SWORD swSinData=0, swCosData=0 ;
  UWORD uwTmp, uwStatus ;
  ULONG ulATan16, ulSinCosInterpolation = 0UL, ulTmp ;
  ULONG ulCountActual, ulCountIndex  = 0UL;
  UQWRD uqProduct64, uqSinCos64, uqTmp ;
  ENCMGR_SPACEFEEDBACK *pIncrOut;
  INCREMENTAL_OUT *pDataOut;
  ULONG ulMPReg = 0UL ; 
  SLONG slMechSpeed_1 ;
  SLONG slAbsIndexDelta ;
  ULONG ulBaseAddress=pIncrRun->ulBaseAddress;
  BOOL  bFlag=FALSE;
  BOOL  bFirstIdxCap=FALSE;
  BOOL  bIdxValid=FALSE;

  pIncrOut=pIncrRun->psFeedBack;
  pDataOut=pIncrRun->psDataOut;

  // clear errors and reset status
  if(bSysStatAlarmsReset)
  {
    // if was faulty, then reset status
    if(pIncrRun->sErrorSent.w || *pIncrRun->psbEncMgrFault)
      pIncrRun->bInitReq=TRUE;

    pIncrRun->sErrorSent.w = 0 ; // clear errors
  }

  // if in init or suspended exit
  if(pIncrRun->bInitReq || pIncrRun->flags.b.bSuspendProcessing)
  {
    if(pIncrRun->flags.b.bHasIndex && !pIncrRun->flags.b.bDisIdxPosition)
      pIncrOut->ubStatus = ENCMGR_ABSMECHTURN_WAITING;
    else
      pIncrOut->ubStatus = ENCMGR_NULL;

    // initialize delta counter with lower part of encoder reading
    pIncrRun->ulDeltaCounter = (ULONG)(-(SLONG)(FPGA_BASEOFF_32(ulBaseAddress, FPGA_INCENC_COUNT32)&0x00000003ul));

    return TRUE;
  }

  // always refresh analog data if apply
  if(pIncrRun->flags.b.bHasAnalog)
  {
    pDataOut->swSin=swSinData=FPGA_BASEOFF_16(ulBaseAddress, FPGA_INCENC_SIN) ;
    pDataOut->swCos=swCosData=FPGA_BASEOFF_16(ulBaseAddress, FPGA_INCENC_COS) ;
  }

  // read actual status
  uwStatus = FPGA_BASEOFF_16(ulBaseAddress, FPGA_INCENC_STATUS );

  // read data from FPGA
  pIncrRun->ulFullCntCapture = ulCountActual = FPGA_BASEOFF_32(ulBaseAddress, FPGA_INCENC_COUNT32);

  // if index was captured
  if (FPGA_INCENC_S_IDX_EVCAP(uwStatus))
  {
    // capture index as is without delta for rollback as it will be used
    // in background for detecting missing index channel
    ulMPReg = FPGA_BASEOFF_32(ulBaseAddress, FPGA_INCENC_INDEX32);
    // allow index capture at least if outside index tolerance, to avoid false
    // index capture in case of index storming
    if(labs(ulMPReg - pIncrRun->ulFullIdxCapture) > pIncrRun->ulIndexTolerance)
    {
      ulCountIndex = ulMPReg + pIncrRun->ulDeltaCounter;
      bIdxValid = TRUE;
    }
  }

  // apply delta to actual captured value and
  ulCountActual += pIncrRun->ulDeltaCounter;

  // check counters rollover
  if(ulCountActual>=pIncrRun->ulTotEncPulse)
  {
    if((SLONG)ulCountActual<0)
        ulTmp=pIncrRun->ulTotEncPulse;
    else
        ulTmp=(ULONG)-(SLONG)pIncrRun->ulTotEncPulse;

    pIncrRun->ulDeltaCounter+=ulTmp;
    ulCountActual           +=ulTmp;
  }
  else
    ulTmp=0l;

    // if index was captured
  if (bIdxValid)
  {
      // apply delta if any
    ulCountIndex+=ulTmp;

    // check counter rollover
    if(ulCountIndex>=pIncrRun->ulTotEncPulse)
    {
      if((SLONG)ulCountIndex<0)
        ulCountIndex+=pIncrRun->ulTotEncPulse;
      else
        ulCountIndex+=(ULONG)-(SLONG)pIncrRun->ulTotEncPulse;
    }

    // if no tolerance check is requested, then capture all indexes
    if(pIncrRun->flags.b.bCaptureAllIndexes)
    {
      pDataOut->ulIndexCapture = ulCountIndex ;
      pDataOut->ubIndexCapCounter++;
      pIncrRun->ulFullIdxCapture = ulMPReg ;
    }
    else
    { // if tolerance check process it after first valid index capture
      if(pIncrRun->ubFirstIndexRead)
      {
        slAbsIndexDelta = labs((SLONG)ulCountIndex - (SLONG)pIncrRun->ulCountIndex_1) ;

          // if in tolerance capture index for monitoring
        if( slAbsIndexDelta < pIncrRun->ulIndexTolerance )
        {
          pIncrRun->ulFullIdxCapture = ulMPReg ;
          pIncrRun->ubIndexWatchDog = 0;
        }
        else
        { // if not in tolerance increment watchdog (will be checked in background)
          if( pIncrRun->ubIndexWatchDog<255 )
            pIncrRun->ubIndexWatchDog++;
        }
      }
      else
      {
        pIncrRun->ulCountIndex_1 = ulCountIndex ;
        pDataOut->ulIndexCapture = ulCountIndex ;
        pDataOut->ubIndexCapCounter++;
        pIncrRun->ulFullIdxCapture = ulMPReg ;
        pIncrRun->ubIndexWatchDog = 0;
      }
    }

    bFirstIdxCap = !pIncrRun->ubFirstIndexRead;
    pIncrRun->ubFirstIndexRead = TRUE;
  }

  // copy to data out
  pDataOut->ulCounter=ulCountActual;

  // if faulty then exit here
  if(pIncrOut->ubStatus&ENCMGR_FATAL_FAULT)
    return TRUE;

  // check for FPGA errors
  if (FPGA_INCENC_S_FAULT(uwStatus) && !pIncrRun->sErrorSent.b.bFpgaFault)
  { // Fpga informs that there's some kind of fault
    ULONG ulFltSubCode;

    // detect fault specific
    if(FPGA_INCENC_S_FLT_ANDIG(uwStatus))
      ulFltSubCode = SYSTEMALARMS_SUBCODE_IN_ANDIG_FAULT;
    else if(FPGA_INCENC_S_FLT_WDT(uwStatus))
      ulFltSubCode = SYSTEMALARMS_SUBCODE_IN_FILTER_WDT;
    else
      ulFltSubCode = SYSTEMALARMS_SUBCODE_IN_FPGA_FAULT;

    // add encoder reference
    if (pIncrRun->flags.b.bAuxEnc)
      ulFltSubCode |= SYSTEMALARMS_SUBCODE_ENC_AUX;
    else
      ulFltSubCode |= SYSTEMALARMS_SUBCODE_ENC_MAIN;

    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL, ulFltSubCode, bSysStatPowerEnabled) ;

    pIncrRun->sErrorSent.b.bFpgaFault = TRUE ;
    pIncrOut->ubStatus = ENCMGR_FATAL_FAULT ;  
    return TRUE;
  }


  // * ================================= *
  // * ========= Index Capture ========= *
  // * ================================= *
  if(pIncrRun->flags.b.bHasIndex && bIdxValid && bFirstIdxCap)
  {
    _uint64_mul_32_32( &uqProduct64, &ulCountIndex, &pIncrRun->np.ulNormFactor ) ;
    _uint64_shl(&uqProduct64, 32 - pIncrRun->np.uwRightShift) ;
  
    // first index position 
    uqProduct64.lo = INT64_HLONG(uqProduct64) ;
    uqProduct64.hi = 0UL ;

    // reset electrical angle if enabled
    if(!pIncrRun->flags.b.bDisIdxEAngle)
    {
      uwTmp = (UWORD)((ULONG)HIWORD(uqProduct64.lo) * pIncrRun->uwMotPolarPairs);
      pIncrOut->uwDeltaElecAngle = (UWORD)-(SWORD)uwTmp;
      pIncrOut->ubStatus |= ENCMGR_ELE_ANGLE_VALID ;
    }

    // adjust absolute pos offset if enabled
    if(!pIncrRun->flags.b.bDisIdxPosition)
    {
      _uint64_add(&uqTmp, &uqProduct64, (UQWRD *)&pIncrOut->sqMechAbsPosOffset);
      if((SLONG)uqTmp.lo<0)
        uqTmp.hi=-1l;
      else
        uqTmp.hi= 0l;
      
      INT64_COPY(uqProduct64, pIncrOut->sqMechAbsPosOffset);
      _uint64_sub((UQWRD *)&pIncrOut->sqMechAbsPosOffset, &uqProduct64, &uqTmp);

      pIncrOut->ubStatus |= ENCMGR_ABSMECHTURN_VALID ;
      pIncrOut->ubStatus &= ~ENCMGR_ABSMECHTURN_WAITING;
    }
  }

  // * ================================ *  
  // * ======== interpolazione ======== *
  // * ================================ *
  if (pIncrRun->flags.b.bEnInterpolation)
  {
    if(labs(pIncrOut->sEncData.slSpeed) > pIncrRun->ulInterpSpeedThresholdHi)
      pIncrRun->bOnAnInterp = FALSE;
    else if(labs(pIncrOut->sEncData.slSpeed) < pIncrRun->ulInterpSpeedThresholdLo)
      pIncrRun->bOnAnInterp = pIncrRun->bEnAnInterp;
    FPGA_INCENC_IGNORE_SINCOS(ulBaseAddress) = !pIncrRun->bOnAnInterp;

    if(pIncrRun->bOnAnInterp)
    {
      ulATan16 = 0x3fff & (ULONG)(ATan16(swCosData, swSinData)) ; 

      _uint64_mul_32_32(&uqSinCos64, &ulATan16, &pIncrRun->np.ulNormFactor) ;
    
      if (pIncrRun->np.uwNbit > 19)
      {
      uwTmp = pIncrRun->np.uwNbit - 19 ;
      _uint64_shr(&uqSinCos64, uwTmp) ;
      }
      else
      {
      uwTmp = 19 - pIncrRun->np.uwNbit ;
      _uint64_shl(&uqSinCos64, uwTmp) ;
      }
         
        // controllo livelli Seno/Coseno
      pDataOut->ulSinCosLevel = (ULONG)((SLONG)swSinData * swSinData + (SLONG)swCosData * swCosData) ;

      if (pDataOut->ulSinCosLevel > pIncrRun->ulSinCosAlarmThreshold)
      {
        ulSinCosInterpolation = INT64_HLONG(uqSinCos64) ;
        pIncrOut->ubStatus |= ENCMGR_RELATIVE_VALID | ENCMGR_EFS_READY ;
      }
      else if (!pIncrRun->flags.b.bSin2Cos2ErrorDisabled && !pIncrRun->sErrorSent.b.bSincosLevel)
      {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL, SYSTEMALARMS_SUBCODE_IN_ENCODER_LEVEL, bSysStatPowerEnabled) ;       
        pIncrRun->sErrorSent.b.bSincosLevel = TRUE ;       
        pIncrOut->ubStatus = ENCMGR_FATAL_FAULT ;      
        return TRUE ;
      }
    }
    else if(!pIncrRun->flags.b.bDisDigInterpolation)
    {
      uwTmp = FPGA_BASEOFF_16(ulBaseAddress, FPGA_INCENC_M_PERIOD);
      if (uwTmp>0u)
        ulATan16 = ( (ULONG)FPGA_BASEOFF_16(ulBaseAddress, FPGA_INCENC_M_FRACT) << 16 ) / uwTmp;
      else
        ulATan16 = 0l;

      _uint64_mul_32_32(&uqSinCos64, &ulATan16, &pIncrRun->np.ulNormFactor) ;
    
      if (pIncrRun->np.uwNbit > 17)
      {
        uwTmp = pIncrRun->np.uwNbit - 17 ;
        _uint64_shr(&uqSinCos64, uwTmp) ;        
      }
      else
      {
        uwTmp = 17 - pIncrRun->np.uwNbit ;
        _uint64_shl(&uqSinCos64, uwTmp) ;
      }
             
      ulSinCosInterpolation = INT64_HLONG(uqSinCos64) ;

      bFlag=FPGA_INCENC_S_M_DIRECTION(uwStatus);
      if (!bFlag)
        ulSinCosInterpolation=-(SLONG)ulSinCosInterpolation;

        // with digital interpolation when changing counting direction the speed is blanked
        // to avoid high spikes
      if ((bFlag && !pIncrRun->bDirection_1) || (!bFlag && pIncrRun->bDirection_1))
      {
        pIncrRun->bDirection_1 = bFlag;
        bFlag = TRUE;
      }
      else
        bFlag = FALSE;

      pIncrOut->ubStatus |= ENCMGR_RELATIVE_VALID | ENCMGR_EFS_READY ;
    }
  }
  else
    pIncrOut->ubStatus |= ENCMGR_RELATIVE_VALID | ENCMGR_EFS_READY ;

  /* Saving previous mechanical angle (for the speed) and speed (for the acceleration) */
  ulMPReg          = pIncrOut->sEncData.sqPostn.lo ;
  slMechSpeed_1    = pIncrOut->sEncData.slSpeed ;
  
  // * =============================== *
  // * == Present Relative Position == *
  // * =============================== *
  // Product64 = CountFPGA * NormFactor
  _uint64_mul_32_32(&uqProduct64, &ulCountActual, &pIncrRun->np.ulNormFactor) ; /* normalization */
  _uint64_shr(&uqProduct64, pIncrRun->np.uwRightShift) ; 
  pIncrOut->sEncData.sqPostn.lo = INT64_LLONG(uqProduct64) + ulSinCosInterpolation ;
   
  if ( (ulMPReg                       & 0x80000000) &&  (ulMPReg                       & 0x40000000) &&
      !(pIncrOut->sEncData.sqPostn.lo & 0x80000000) && !(pIncrOut->sEncData.sqPostn.lo & 0x40000000)   ) 
  { // Mechanical Angle : Clockwise Overflow
    pIncrOut->sEncData.sqPostn.hi++ ; 
  } 
  else if (!(ulMPReg                       & 0x80000000) && !(ulMPReg                       & 0x40000000) &&
            (pIncrOut->sEncData.sqPostn.lo & 0x80000000) &&  (pIncrOut->sEncData.sqPostn.lo & 0x40000000)   ) 
  { // Mechanical Angle : Anticlockwise Overflow
    pIncrOut->sEncData.sqPostn.hi-- ; 
  }    

  // Current Mechanical Speed = RelMechAngle - RelMechAngle_1 (= delta position)
  pIncrOut->sEncData.slSpeed = pIncrOut->sEncData.sqPostn.lo - ulMPReg ;
  if(bFlag)
      pIncrOut->sEncData.slSpeed /= 2;
    
  // Current Mechanical Acceleration = CurrMechSpeed - PrevMechSpeed (= delta speed)
  pIncrOut->sEncData.slAccel = _sint32_asl12(pIncrOut->sEncData.slSpeed - slMechSpeed_1) ;

  // Electrical Angle
  pIncrOut->uwElecAngle = (UWORD)((ULONG)HIWORD(pIncrOut->sEncData.sqPostn.lo) * pIncrRun->uwMotPolarPairs) + pIncrOut->uwDeltaElecAngle + sGlbMotorParameters.uwPhaseOffset ;   

  /*  Current Electrical Speed  = MechSpeed * MotorPolePairs */
  pIncrOut->swElecSpeed = (SWORD)_sint32_scale_32(pIncrOut->sEncData.slSpeed, pIncrRun->uwMotPolarPairs) ;

  return TRUE ;
}

/* ######################################################################### */
void Ic_SimulationGetHwOpt(ULONG * pulHwOpt, ULONG * pulFPGAOpt)
{
#ifndef _HW_DC
  *pulHwOpt  |=HWUNITID_ENCA_INC_SIM;
  *pulFPGAOpt|=FPGA_HW_AUXOUT;
#else
  *pulHwOpt  |=HWUNITID_ENCA_INC_BOUT;
  *pulFPGAOpt|=FPGA_HW_AUXENDOUT;
#endif
  if(sSe_AuxSimParam.flags.b.bEnFastAuxSim)
    *pulFPGAOpt|=FPGA_HW_FASTINCRSIM;
}

// ///////////////////////////////////////////////////////////////////////
BOOL Ic_SimulationInit(ENCMGR_SPACEFEEDBACK *psEnc2Sim, UBYTE * pubStatus)
{
    // prepare pulses count and normalization for the feedback
    sAuxSimRun.ulTotEncPulse = sSe_AuxSimParam.ulLineCounts * 4;
    normcomp(sAuxSimRun.ulTotEncPulse, &sAuxSimRun.np);

    // encoder to be simulated
    sAuxSimRun.psInputEncoder = psEnc2Sim ;

    // fault export
    sAuxSimRun.pubStatus=pubStatus;
    *sAuxSimRun.pubStatus = ENCMGR_NULL;

    // metto a posto l'FPGA
#ifndef _HW_DC
    if(sGlbControlBoardParameters.sProductInfo.uwProductRev < 600)
    {
        FPGA_ENCODER_OPTIONS_AUXENA = TRUE  ; // enable encoder simulation
        FPGA_ENCODER_OPTIONS_AUXENB = TRUE  ; // enable encoder simulation
    }
    FPGA_ENCODER_OPTIONS_AENDEN = FALSE ; // no auxiliary endat
#endif

    // signal remote display to change data direction
    sSe_AuxSimOut.sbOutEnabled  = TRUE ;

    // init structure data
    sAuxSimRun.flags.b.bFirstIndexCalculated = FALSE ;
#ifdef _INFINEON_
    FPGA_IOUTAUX_INDEX32 = sAuxSimRun.slFpgaAuxOutIdx = INDEX_RESET_STATE ; /* init the value in fpga */
#else
    sAuxSimRun.slFpgaAuxOutIdx = INDEX_RESET_STATE ; /* init the value in fpga */
    FPGA_IOUTAUX_POSID_LSW = LOWORD(sAuxSimRun.slFpgaAuxOutIdx);
    FPGA_IOUTAUX_POSID_MSW = HIWORD(sAuxSimRun.slFpgaAuxOutIdx);
#endif

    sAuxSimRun.ulIdxHalfPulseRpt = 2 * sSe_AuxSimParam.ulIndexPulseRpt ;
    sAuxSimRun.ulIdxTotPulseRpt  = 2 * sAuxSimRun.ulIdxHalfPulseRpt ; // total number of pulse to repeat index
    sAuxSimRun.slFpgaAuxOutIdxOffset = sSe_AuxSimParam.ulIndexOffset ;
    sAuxSimRun.ulDeltaCounter    = 0ul;
    
    // init fast simulation
    sAuxSimRun.flags.b.bEnFastAuxSim = sSe_AuxSimParam.flags.b.bEnFastAuxSim;
    if(sAuxSimRun.flags.b.bEnFastAuxSim)
    {
        FPGA_IOUTAUX_FASTSIM_EN=TRUE;
        sAuxSimRun.ubFastRatio=(UBYTE)(_ilog2(sIc_IncEncParams[INCREMENTAL_SEL_MAIN].ulLineCounts/sSe_AuxSimParam.ulLineCounts)-1);
    }
    
    sAuxSimRun.flags.b.bBlindAuxSim = sSe_AuxSimParam.flags.b.bBlindAuxSim ;

    // install only one background for all
    if(!sIncEncBgInstalled)
    {
        if(!TaskSched_AddBackgroundTask(&slowtask))
            return FALSE ; 
        sIncEncBgInstalled=TRUE;
    }

    return TaskSched_AddRTTask(&IncrementalAuxSimulation8KHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0) ; /* install 8KHz function */
}

// ///////////////////////////////////////////////////////////////////////
BOOL Ic_SimulationParametersCheck(SBYTE bIsMainRelIncr)
{
    BOOL bValid=TRUE;
    UBYTE ubRatio = 0xFF;

        // ulTotPulseRepetition deve essere inferiore a 0x7fffffff perche' uso la _uint64_mul_32_32 e non voglio che il bit31 sia interpretato come segno '-'
        // NON consento un indice di 0 impulsi
    if ((sSe_AuxSimParam.ulIndexPulseRpt == 0UL) || (sSe_AuxSimParam.ulIndexPulseRpt >= SLONG_MAX_VALUE/4))
    {
        ParChk_SignalValueError(PARCC_INCRSIM_IDXPULSEREPEAT);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_INCRSIM_IDXPULSEREPEAT);

    if((ULONG)sSe_AuxSimParam.ulIndexOffset >= sSe_AuxSimParam.ulIndexPulseRpt)
    {
        ParChk_SignalValueError(PARCC_INCRSIM_IDXOFFSET);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_INCRSIM_IDXOFFSET);

    if(sSe_AuxSimParam.ulLineCounts==0)
    {
        ParChk_SignalValueError(PARCC_INCRSIM_MAINLINENUMBERS);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_INCRSIM_MAINLINENUMBERS);

    if(sSe_AuxSimParam.uwMaxFrequency && (sSe_AuxSimParam.uwMaxFrequency < DIGFILTER_FREQ_MIN || 
                                          sSe_AuxSimParam.uwMaxFrequency > DIGFILTER_FREQ_MAX))
    {
        sAuxSimRun.uwDigFilterValue = DIGFILTER_FREQ_DEFAULT;
        ParChk_SignalValueError(PARCC_INCRSIM_MAINMAXFREQUENCY);
        bValid=FALSE;
    }
    else
    {
        sAuxSimRun.uwDigFilterValue = sSe_AuxSimParam.uwMaxFrequency;
        ParChk_ResetValueError(PARCC_INCRSIM_MAINMAXFREQUENCY);
    }

        // enable fast aux simulation when main incremental encoder is not selected
    if(sSe_AuxSimParam.flags.b.bEnFastAuxSim && !bIsMainRelIncr)
    {
        ParChk_SignalValueError(PARCC_INCRSIM_ENABLEFASTAUXSIM);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_INCRSIM_ENABLEFASTAUXSIM);
    
        // the line count ratio between main incremental encoder and fast aux simulation is not power of 2
    if(sSe_AuxSimParam.ulLineCounts > 0 && sIc_IncEncParams[INCREMENTAL_SEL_MAIN].ulLineCounts >= sSe_AuxSimParam.ulLineCounts)
    {
        ubRatio = (UBYTE)_ilog2(sIc_IncEncParams[INCREMENTAL_SEL_MAIN].ulLineCounts/sSe_AuxSimParam.ulLineCounts)-1;
        if((sSe_AuxSimParam.ulLineCounts << ubRatio) != sIc_IncEncParams[INCREMENTAL_SEL_MAIN].ulLineCounts)
            ubRatio = 0xFF;
    }
    if(sSe_AuxSimParam.flags.b.bEnFastAuxSim && ubRatio >= FASTSIM_RATIO_MAX)
    {
        ParChk_SignalValueError(PARCC_INCRSIM_RATIOFASTAUXSIM);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_INCRSIM_RATIOFASTAUXSIM);
        
    return bValid;
}

// ///////////////////////////////////////////////////////////////////////
static BOOL IncrementalAuxSimulation8KHz(void)
{
    UWORD uwPulse2Do = 0, uwSign2Use = 0 ;
    SLONG slAuxAngle ;
    SLONG slDeltaAngle, slDeltaAngleFpgaCnts ;
    ULONG ulTmp ;
    SQWRD sqProduct64 ;
    UQWRD uqProduct64 ;

    // clear errors and reset status
    if(bSysStatAlarmsReset)
    {
        *sAuxSimRun.pubStatus = ENCMGR_NULL;
        sAuxSimRun.sErrorSent.w = 0 ; // clear errors
    }
    
    // if input encoder still not valid
    if(!(sAuxSimRun.psInputEncoder->ubStatus & ENCMGR_RELATIVE_VALID))
      return TRUE;

    // if elec angle not valid do not emulate
    if(!(sAuxSimRun.psInputEncoder->ubStatus & ENCMGR_ELE_ANGLE_VALID) && sAuxSimRun.flags.b.bBlindAuxSim)
      return TRUE;

    // if faulty input encoder then stop updating
    if(sAuxSimRun.psInputEncoder->ubStatus & ENCMGR_FATAL_FAULT)
      return TRUE;

    // get actual reached position from the simulation
    slAuxAngle = (SLONG)FPGA_IOUTAUX_COUNT32;
    ulTmp = sAuxSimRun.ulDeltaCounter + (ULONG)slAuxAngle;

    // check counters rollover
    if(ulTmp>=sAuxSimRun.ulTotEncPulse)
    {
        if((SLONG)ulTmp<0)
        {
            slDeltaAngle=+(SLONG)sAuxSimRun.ulTotEncPulse;
            sSe_AuxSimOut.sqPostn.hi--;
        }
        else
        {
            slDeltaAngle=-(SLONG)sAuxSimRun.ulTotEncPulse;
            sSe_AuxSimOut.sqPostn.hi++;
        }
        
        sAuxSimRun.ulDeltaCounter+=slDeltaAngle;
        ulTmp                    +=slDeltaAngle;
    }

    // normalize feedback
    _uint64_mul_32_32(&uqProduct64, &ulTmp, &sAuxSimRun.np.ulNormFactor) ; /* normalization */
    _uint64_shr(&uqProduct64, sAuxSimRun.np.uwRightShift) ;
    sSe_AuxSimOut.sqPostn.lo = INT64_LLONG(uqProduct64) ;

    // Delta Angle = MainEncoderPosition - ActualSimEncoderPosition
    slDeltaAngle = (SLONG)(sAuxSimRun.psInputEncoder->sEncData.sqPostn.lo - sSe_AuxSimOut.sqPostn.lo) ;

    // check max diff, then alarm
    if(!sAuxSimRun.sErrorSent.b.bSimMaxDiffExceed && sSe_AuxSimParam.uwMaxAngleDiff && \
        abs((SWORD)(slDeltaAngle>>16))>sSe_AuxSimParam.uwMaxAngleDiff)
    {
      SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL, SYSTEMALARMS_SUBCODE_ENC_AUX | SYSTEMALARMS_SUBCODE_IN_SIM_MAXDIFFEXCEED, bSysStatPowerEnabled) ;
      sAuxSimRun.sErrorSent.b.bSimMaxDiffExceed = TRUE ; 
      *sAuxSimRun.pubStatus = ENCMGR_NONFATAL_FAULT ;
    }

    if(!sAuxSimRun.flags.b.bEnFastAuxSim)
    {

        // multiply by two in order to get approx of 0.5 steps, this avoid both output toggling between
        // two values and pair steps approx
        slDeltaAngle<<=1;
        _sint64_mul_32_32(&sqProduct64, &slDeltaAngle, (SLONG *)&(sAuxSimRun.ulTotEncPulse)) ;
        slDeltaAngleFpgaCnts = (SLONG)(INT64_HLONG(sqProduct64)) - sAuxSimRun.slDeltaAngleFpgaCnts_1 ; // main sensor angle in FPGA counts
       
        // ============= pulses generation =============
        if ((slDeltaAngleFpgaCnts > 1) || (slDeltaAngleFpgaCnts < -1))
        {   
            uwSign2Use = POSITIVE ; // sign to send to FPGA
            if (slDeltaAngleFpgaCnts < 0)
            {   // absolute value to fpga
                uwSign2Use = NEGATIVE ;
                slDeltaAngleFpgaCnts = -slDeltaAngleFpgaCnts ;  
            }
        
            if (slDeltaAngleFpgaCnts > sAuxSimRun.uwDigFilterValue)
                slDeltaAngleFpgaCnts = sAuxSimRun.uwDigFilterValue ;    // max freq selected
    
            uwPulse2Do = 20004 / (UWORD)slDeltaAngleFpgaCnts ;  
            FPGA_IOUTAUX_STEPS = (ENABLE | uwSign2Use | uwPulse2Do) ; // enable 
        }
        else
        {
            slDeltaAngleFpgaCnts = 0 ;
            FPGA_IOUTAUX_STEPS   = 0 ; // disabilito: no output
        }
    
        // salvo la regressione con il segno giusto
        if (uwSign2Use == POSITIVE)
            sAuxSimRun.slDeltaAngleFpgaCnts_1 =  slDeltaAngleFpgaCnts ;
        else
            sAuxSimRun.slDeltaAngleFpgaCnts_1 = -slDeltaAngleFpgaCnts ;  
    } 
    else // fast simulation
    {
        FPGA_IOUTAUX_STEPS = sAuxSimRun.ubFastRatio;
    }       

    // ============= index generation =============
    if ((sAuxSimRun.psInputEncoder->ubStatus & (ENCMGR_ABSMECHTURN_VALID|ENCMGR_ABSMECHTURN_WAITING)) == ENCMGR_ABSMECHTURN_VALID ) 
    {   // posizione assoluta dall'encoder principale e' valida
        if (sAuxSimRun.flags.b.bFirstIndexCalculated)
        {   // calcolo gli indici successivi
            slDeltaAngle = slAuxAngle - sAuxSimRun.slFpgaAuxOutIdx ;
            if(slDeltaAngle < -(SLONG)sAuxSimRun.ulIdxHalfPulseRpt)
                sAuxSimRun.slFpgaAuxOutIdx -= sAuxSimRun.ulIdxTotPulseRpt ;
            else if(slDeltaAngle > +(SLONG)sAuxSimRun.ulIdxHalfPulseRpt)
                sAuxSimRun.slFpgaAuxOutIdx += sAuxSimRun.ulIdxTotPulseRpt ;
        }
        else
        {  // calcolo il primo indice
           // 1stIndex = (MechAbsPosOffset * TotEncPulse) / 2^32
           ULONG ulDeltaAbsInit ;

           sAuxSimRun.flags.b.bFirstIndexCalculated = TRUE ; // solo una volta
           ulDeltaAbsInit = ULONG_MAX_VALUE - sAuxSimRun.psInputEncoder->sqMechAbsPosOffset.lo ; // 360° - MainEncAbsInitPos => 2^32 - MainEncAbsInitPos
           _uint64_mul_32_32(&uqProduct64, &ulDeltaAbsInit, &(sAuxSimRun.ulTotEncPulse)) ;
           sAuxSimRun.slFpgaAuxOutIdx = (SLONG)(INT64_HLONG(uqProduct64)) + sAuxSimRun.slFpgaAuxOutIdxOffset ;
        }
    }
    else
    {   // posizione assoluta dell'encoder principale NON e' valida
        // tengo il valore di indice 'resettato' finche' l'encoder principale non mi dice che la posizione assoluta e' valida
        sAuxSimRun.flags.b.bFirstIndexCalculated = FALSE ;
        sAuxSimRun.slFpgaAuxOutIdx = INDEX_RESET_STATE ;
    }

#ifdef _INFINEON_
    FPGA_IOUTAUX_INDEX32 = sAuxSimRun.slFpgaAuxOutIdx ; // write to FPGA
#else
    FPGA_IOUTAUX_POSID_LSW = LOWORD(sAuxSimRun.slFpgaAuxOutIdx);
    FPGA_IOUTAUX_POSID_MSW = HIWORD(sAuxSimRun.slFpgaAuxOutIdx);
#endif

    return TRUE ;
}
#endif
