/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : SpaceSpeedCntrLp.c                                         */
/* Author      : Cristiano Tognetti                                         */
/*                                                                          */
/* Description : SpaceSpeed Control Loop                                    */
/*                                                                          */
/****************************************************************************/

#include <string.h>
#include <math.h>

#include "common\CommonDefines.h"
#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"

#include "common\MathFunctions.h"
#include "common\Int64Functions.h"
#include "drive\SpaceSpeedCntrLp.h"

#include "common\TaskScheduler.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#if defined(_CRS_DBG)
#if (FALSE) //CRS_DBGDSK
#pragma GCC optimize (0) // crs_dbg
#else
#pragma GCC optimize (2)
#endif
#else
#pragma GCC optimize (2)
#endif

/* ================================ #define ================================ */
#define GAIN_POSITION_MAX_SHIFT_RIGHT  16
#define GAIN_ACCEL_MAX_SHIFT_LEFT      15

#define GAIN_GLOBAL_MAX_SHIFT_RIGHT    16
#define GAIN_GLOBAL_MAX_SHIFT_LEFT      8

#define NULL_GAIN                      -128
#define CONVFACT_ACCELGAIN             1.498027402198642e4
#define CONVFACT_SPEEDGAIN             7.678538655241527e3
#define CONVFACT_POSGAIN               0.959697234641399
#define CONVFACT_INTEGRALGAIN          8.182783952302470

// code optimization
#define CONVFACT_ACCELGAIN_INV         (1.0 / CONVFACT_ACCELGAIN)
#define CONVFACT_SPEEDGAIN_INV         (1.0 / CONVFACT_SPEEDGAIN)
#define CONVFACT_POSGAIN_INV           (1.0 / CONVFACT_POSGAIN)
#define CONVFACT_INTEGRALGAIN_INV      (1.0 / CONVFACT_INTEGRALGAIN)

#define FLOAT_2PI_IU                   (10000.0 * 2.0 * FLOAT_PI)

/* ============================== structures =============================== */
typedef struct {
  /* Flags */
  union {
      struct {
        UBYTE bInvElecAngleDetect : 1 ;
      } b ;
      UBYTE w ;
  } flags ;
  SBYTE bParUsr;
    
  UWORD uwCntrlWord ; /* control word */
    
  /* Parameters image to be used RunTime*/
  SS_RT_PARAMS sPars[2];
  SS_RT_PARAMS * psParUpd;
  SS_RT_PARAMS * psParStd;
  SS_RT_PARAMS sParUsr;

  /* integral current limits to be traslated */
  SLONG slILimMin;
  SLONG slILimMax;
    
  /* regressions */
  SQWRD sqIntegral_1 ; /* necessary to add here in order to reset the regression */
  SQWRD sqPosErr64 ; /* Position Error = PosRef - PosFbk */
  SWORD swGlobalGainShift_1 ;
} SS_RUNTIME ;

SS__INPUT sSS_CntrLoopIn  ;
SS_OUTPUT sSS_CntrLoopOut ;
SS_CMB_PARAMS sSS_CntrLoopParam ;
SS__WORKS sSS_CntrLoopWks ;

// -- input from plc
GLB_KINEMATIC_DATA  sSS_PlcPidRef ; 
MOTCTRL_STATUS      sSS_PlcMotCtrlStatus ;

// -- Runtime
static SS_RUNTIME sSpaceSpeedRun ;

// default parameters (referred to UL503.30.3)
#ifdef _INFINEON_
const SS_PARAMS huge sSS_CntrLoopDefParam = 
#else // _infineon_
const SS_PARAMS sSS_CntrLoopDefParam =
#endif //_infineon_
{
    {1},  // flags: bit 0 = 1 (use different Speed Kp)
    {0.0,6.512,0.02,0.02,0.0,0.0},
    {SLONG_MAX_VALUE, SLONG_MIN_VALUE}, // Limits
} ;

// shadow data used to detect changes by which recalc is triggered
#ifdef _INFINEON_
static SS_V15_PAR_GAINS huge sShadowIntegerGains ;
static SS_PAR_GAINS huge sShadowFloatGains ;
#else // _infineon_
static SS_V15_PAR_GAINS sShadowIntegerGains ;
static SS_PAR_GAINS sShadowFloatGains ;
#endif //_infineon_

/* =========================== global  variables =========================== */

/* =============================== functions =============================== */
static BOOL Ss_ControlLoop8KHz(void) ;
static void Ss_ControlLoopBkGd(void) ;
static BOOL copyandcheckparameters(void);

static UWORD CheckUWordParam(UWORD uwParamVal, UWORD uwLimit)  ;
static SWORD CheckSWordParam(SWORD swParamVal, SWORD swPositiveLimit, SWORD swNegativeLimit) ;

static SBYTE NBitsFloat(FLOAT flInValue) ;
static BOOL CtrLpGainsInternalUnits2EngineeringUnits(void) ;
static void AutoMaxPosError(void) ;

/* ######################################################################### */
static UWORD CheckUWordParam(UWORD uwParamVal, UWORD uwLimit) 
{
  if (uwParamVal > uwLimit)
    uwParamVal = uwLimit ;   
  
  return uwParamVal ;
}


/* ######################################################################### */
static SWORD CheckSWordParam(SWORD swParamVal, SWORD swPositiveLimit, SWORD swNegativeLimit) 
{
  if (swParamVal > swPositiveLimit)
    swParamVal = swPositiveLimit ; 
  else if (swParamVal < -swNegativeLimit)
    swParamVal = -swNegativeLimit ;  
  
  return swParamVal ;
}


/* ######################################################################### */
BOOL Ss_ControlLoopInit(void)
{
  sSpaceSpeedRun.bParUsr = FALSE;

  // initialize parameters pointer
  sSpaceSpeedRun.psParStd = &sSpaceSpeedRun.sPars[1];
  sSpaceSpeedRun.psParUpd = &sSpaceSpeedRun.sPars[0];

  // initialize local current limit copy
  sSpaceSpeedRun.slILimMax = sSS_CntrLoopIn.psIqLimit->slMax;
  sSpaceSpeedRun.slILimMin = sSS_CntrLoopIn.psIqLimit->slMin;

  // check parameters and force gains recalc
  sShadowFloatGains.flKi = sSS_CntrLoopParam.sPars.sGains.flKi+1.0;
  if(!copyandcheckparameters())
    return FALSE;

  /* ======================== Output Variables ======================== */
  sSS_CntrLoopOut.sIRef.slIqRef  = 0L ;
    
  /* ====================== reset the integral ====================== */  
  INT64_ASSIGN(sSpaceSpeedRun.sqIntegral_1, 0L, 0UL) ;

  // switch parameters to valid
  sSpaceSpeedRun.psParStd = &sSpaceSpeedRun.sPars[0];
  sSpaceSpeedRun.psParUpd = &sSpaceSpeedRun.sPars[1];
  sSpaceSpeedRun.swGlobalGainShift_1 = sSpaceSpeedRun.psParStd->swGlobalGainShift;

  /* ============== install background & 8KHz functions ============= */
  if(!TaskSched_AddBackgroundTask(&Ss_ControlLoopBkGd))
    return FALSE ;    

  /* install 8KHz function */
  if(!TaskSched_AddRTTask(&Ss_ControlLoop8KHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0))
    return FALSE ; 

  return TRUE ;
}

/* ######################################################################### */
static BOOL copyandcheckparameters(void)
{
  BOOL bValid;

  // detect and apply integer gains
  bValid = CtrLpGainsInternalUnits2EngineeringUnits();

  if(!PlcCtrLpGainsCompute(NULL, NULL))
  {
    ParChk_SignalValueError(PARCC_SS_OUTOFRANGEGAINS);
    bValid=FALSE;
  }
  else
    ParChk_ResetValueError(PARCC_SS_OUTOFRANGEGAINS);

  {
    SLONG slTmp;
    
    atomic_read(&slTmp, &sSS_CntrLoopParam.sPars.sUsrOutVal32.slMax, sizeof(sSS_CntrLoopParam.sPars.sUsrOutVal32.slMax));
    atomic_write(&sSS_CntrLoopWks.sUsrOutVal32.slMax, &slTmp, sizeof(sSS_CntrLoopWks.sUsrOutVal32.slMax));      
    atomic_read(&slTmp, &sSS_CntrLoopParam.sPars.sUsrOutVal32.slMin, sizeof(sSS_CntrLoopParam.sPars.sUsrOutVal32.slMax));
    atomic_write(&sSS_CntrLoopWks.sUsrOutVal32.slMin, &slTmp, sizeof(sSS_CntrLoopWks.sUsrOutVal32.slMax));
  }

  return bValid;
}

/* ######################################################################### */
static BOOL Ss_ControlLoop8KHz(void)
{
  SS_RT_PARAMS * psParRT;
  BOOL  bIntegralReset ;

  SWORD swMax, swMin, swGlbShift, swDifGlbShift ;
  SLONG slIntegralInValue32 ;
    
  SQWRD sqTmp64, sqMul64 ;
  SQWRD sqPosContribute, sqSpdContribute, sqAccContribute ;
  SQWRD sqRefProportional, sqFbkProportional, sqProportionalOutValue ;
  SQWRD sqIntegralInValue, sqIntegralOutValue ;
  SQWRD sqIqRef64 ;

  // data selection
  if(sSpaceSpeedRun.bParUsr)
    psParRT = &sSpaceSpeedRun.sParUsr;
  else
    psParRT = sSpaceSpeedRun.psParStd;

  // keep integral reset until full power is enabled 
  // arzigogolo perche' voglio poter usare il bit di potenza abilitata 
  // 0 = drive disabilitato oppure abilitato ed in controllo di coppia->reset integrale; 
  // 1 = drive abilitato e NON in controllo di coppia->integrale attivo (dipende da Ki)
  // Ki = 0 -> reset integrale
  if (sSS_CntrLoopIn.psMotCtrlStatus->b.bPID_IntegralEnable)
  {   // integrale dovrebbe essere attivo (sono abilitato e NON in controllo di coppia)
      if (psParRT->swKi == 0)
          bIntegralReset = TRUE ; // Ki = 0 -> devo resettare l'integrale
      else
          bIntegralReset = FALSE ;  
  }
  else
      bIntegralReset = TRUE ; // integrale deve essere resettato (NON sono abilitato o sono in controllo di coppia)


  /* ================================================ *
   * ======= Position proportional contribute ======= *
   * ================================================ */
  _sint64_sub(&sSpaceSpeedRun.sqPosErr64, &sSS_CntrLoopIn.psRef->sqPostn, &sSS_CntrLoopIn.psFbk->sEncData.sqPostn) ; /* Error(64) = ReferencePosition(64) - FeedbackPosition(64) */
  _sint64toSint48(&sSpaceSpeedRun.sqPosErr64) ;  /* limit the error to 48bit */
    
  if (psParRT->uwPosGainShift != 0) /* Kp Position gain right shift */
    _sint64_shr(&sSpaceSpeedRun.sqPosErr64, psParRT->uwPosGainShift) ; 

  _sint64_mul_48_16(&sqPosContribute, &sSpaceSpeedRun.sqPosErr64, &(psParRT->swPostnKp)) ;
  _sint64toSint48(&sqPosContribute) ;  /* limit the Position contribute to 48bit */


  /* ============================================= * 
   * ======= Speed proportional contribute ======= *
   * ============================================= */
  _sint64_mul_32_16(&sqRefProportional, &sSS_CntrLoopIn.psRef->slSpeed, &(psParRT->swSpdKpRef)) ; /* SpeedProportionalRef(64) = SpeedReference(32) * SpeedKpRef(16) */
  _sint64_mul_32_16(&sqFbkProportional, &sSS_CntrLoopIn.psFbk->sEncData.slSpeed, &(psParRT->swSpdKpFbk)) ; /* SpeedProportionalFbk(64) = SpeedFeedback(32)  * SpeedKpFbk(16) */
  _sint64_sub(&sqSpdContribute, &sqRefProportional, &sqFbkProportional) ; /* SpeedContribute(64) = SpeedProportionalRef(64) - SpeedProportionalFbk(64) */ 

  
  /* ==================================================== *
   * ======= Acceleration proportional contribute ======= *
   * ==================================================== */
  if (sSS_CntrLoopParam.sPars.flags.b.bFilterAccRef)
  { // filter acc ref
    sSS_CntrLoopOut.slAccRefFiltered32 = _sint32_scale_16(sSS_CntrLoopOut.slAccRefFiltered32, sSS_CntrLoopParam.sPars.swAccKFilter) + 
                                         _sint32_scale_16(sSS_CntrLoopIn.psRef->slAccel,     (32767-sSS_CntrLoopParam.sPars.swAccKFilter)) ;
  }
  else
  { // keep acc ref filter status updated
    sSS_CntrLoopOut.slAccRefFiltered32 = sSS_CntrLoopIn.psRef->slAccel ; 
  }

  _sint64_mul_32_16(&sqRefProportional, &sSS_CntrLoopOut.slAccRefFiltered32, &(psParRT->swAccKpRef)) ; /* AccelerationProportionalRef(64) = AccelerationReference(32) * AccelerationKpRef(16) */
  _sint64_mul_32_16(&sqFbkProportional, &sSS_CntrLoopIn.psFbk->sEncData.slAccel, &(psParRT->swAccKpFbk)) ; /* AccelerationProportionalFbk(64) = AccelerationFeedback(32)  * AccelerationKpFbk(16) */
  _sint64_sub(&sqAccContribute, &sqRefProportional, &sqFbkProportional) ;                 /* AccelerationContribute(64) = AccelerationProportionalRef(64) - AccelerationProportionalFbk(64) */   
  
  if (psParRT->uwAccGainShift != 0) /* acceleration left shift */
    _sint64_shl(&sqAccContribute, psParRT->uwAccGainShift) ;


  /* ================================= *
   * ========== 64 bit sums ========== *
   * ================================= */
  _sint64_add(&sqTmp64, &sqSpdContribute, &sqAccContribute) ;   /* sqSpeed + sqAcceleration */
  _sint64_add(&sqProportionalOutValue, &sqTmp64, &sqPosContribute) ; /* sqPosition + sqSpeed + sqAcceleration */  
    
    
  /* =================================== *
   * ======= Integral contribute ======= *
   * =================================== */
  /* reduce input value from 64bit to 32bit, removing the most and the least 16 bits */
  _sint64_atomic_copy(&sqIntegralInValue, &sqProportionalOutValue) ; /* necessary to copy in order not to corrupt the Proportional part */
  _sint64toSint48(&sqIntegralInValue) ;                              /* necessary to saturate the result */
  slIntegralInValue32 = (SLONG)(INT64_MLONG(sqIntegralInValue)) ;    /* get the 'middle' 32bit */ 
  INT64_ASSIGN(sqIntegralOutValue, 0L, 0UL) ; /* initialize the value to zero (otherwise it could be initialized with wrong values) */

  _sint64_mul_32_16(&sqMul64, &slIntegralInValue32, &(psParRT->swKi))  ; /* Integral(64) = InValue(32)  * Ki(16) */
  
  swDifGlbShift = sSpaceSpeedRun.swGlobalGainShift_1 - psParRT->swGlobalGainShift;
  sSpaceSpeedRun.swGlobalGainShift_1 = psParRT->swGlobalGainShift;
  if(swDifGlbShift>0)
    _sint64_shr(&sSpaceSpeedRun.sqIntegral_1, (UWORD)swDifGlbShift);
  else if(swDifGlbShift<0)
    _sint64_shl(&sSpaceSpeedRun.sqIntegral_1, (UWORD)(-swDifGlbShift));
  
  if(bIntegralReset)
  { 
    INT64_ASSIGN(sSpaceSpeedRun.sqIntegral_1, 0L, 0UL) ;
  }

  _sint64_add(&sqIntegralOutValue, &sqMul64, &sSpaceSpeedRun.sqIntegral_1) ; /* Integral(64) = Integral(64) + Integral_1(64) */
      
  /* limit integral contribute to avoid explosions: limit to TorqueMax64 */
  swMax = _sint64_comp(&sqIntegralOutValue, &psParRT->sqIntegralMaxLimit) ;
  swMin = _sint64_comp(&sqIntegralOutValue, &psParRT->sqIntegralMinLimit) ;
    
  if ((swMax == 1) && (swMin != -1))
    _sint64_atomic_copy(&sqIntegralOutValue, &psParRT->sqIntegralMaxLimit) ; /* Limit to IntegralMaxLimit */
  else if ((swMax != 1) && (swMin == -1))
    _sint64_atomic_copy(&sqIntegralOutValue, &psParRT->sqIntegralMinLimit) ; /* Limit to IntegralMinLimit */
  else if ((swMax == 1) && (swMin == -1))
  { /* sqOutValue contemporarly greater than IntegralMaxLimit and less than IntegralMinLimit  */ 
    INT64_ASSIGN(sqIntegralOutValue, 0L, 0UL) ;
  }
  /* regression */
  _sint64_atomic_copy(&sSpaceSpeedRun.sqIntegral_1, &sqIntegralOutValue) ; /* sqIntegral_1(64) = sqIntegral(64) */
    
    
  /* ================================ *
   * ======== 64bit OutValue ======== *
   * ================================ */ 
  _sint64_add(&sqIqRef64, &sqProportionalOutValue, &sqIntegralOutValue) ; /* sqPosition + sqSpeed + sqAcceleration + sqIntegral */
  
    
  /* ================================ *
   * ========= Global shift ========= *
   * ================================ */
    // aggiungo shift di 16 bit a sx in modo da avere gia' il risultato finale saturato nei 32 MSB
  swGlbShift = psParRT->swGlobalGainShift - 16;
  if (swGlbShift > 0)
    _sint64_shr(&sqIqRef64, (UWORD)swGlbShift) ;
  else if(swGlbShift < 0)
    _sint64_shl(&sqIqRef64, (UWORD)-swGlbShift) ;
    
    
  /* ================================ *
   * == reduce from 64bit to 32bit == *
   * ================================ */ 
   sSS_CntrLoopOut.sIRef.slIqRef = INT64_HLONG(sqIqRef64) ; /* reduce from 64bit to 32bit */
   sSS_CntrLoopOut.sIRef.slIdRef = sSS_CntrLoopIn.psCurrentRefs->slIdRef;

// ridondante: questo limite e' poi fatto dal MotorHandler
// ma se deve andare in PLC??
//    if (sSS_CntrLoopOut.sIRef.slIqRef > sSS_CntrLoopIn.psIqLimit->slMax)
//        sSS_CntrLoopOut.sIRef.slIqRef = sSS_CntrLoopIn.psIqLimit->slMax ;
//    else if (sSS_CntrLoopOut.sIRef.slIqRef < sSS_CntrLoopIn.psIqLimit->slMin)
//        sSS_CntrLoopOut.sIRef.slIqRef = sSS_CntrLoopIn.psIqLimit->slMin ;
    
  return TRUE ;
}


/* ######################################################################### */
static void Ss_ControlLoopBkGd(void)
{
  SQWRD sqTmp ;
  SLONG slMin, slMax ;
  FLOAT flPosErr, flPosErrMin, flPosErrMax, flPosErrRatio = 1.0 ;
  SWORD swGlobalShift ;
  SS_RT_PARAMS * psParRT;

  // check parameters
  copyandcheckparameters();

  /* ------------------------------------------------------------------------------------ *
  * update MaxIntegralValue and MinIntegralValue to limit integral contribute            *
  * ------------------------------------------------------------------------------------ */
  atomic_read(&slMax, &sSS_CntrLoopIn.psIqLimit->slMax, sizeof(slMax)); 
  atomic_read(&slMin, &sSS_CntrLoopIn.psIqLimit->slMin, sizeof(slMin));

  /* ------------------------------- INTEGRAL ANTIWINDUP ------------------------------- */
  if ((sSS_CntrLoopIn.psFbk->ubStatus & ENCMGR_ELE_ANGLE_VALID) != ENCMGR_ELE_ANGLE_VALID)
  {   /* angolo elettrico NON e' valido: spalanco i limiti all'integrale per
          non limitare la corrente durante la fasatura senza spostare il rotore */
      slMax = SLONG_MAX_VALUE ;
      slMin = SLONG_MIN_VALUE ;
      fast_atomic_set_flag(sSpaceSpeedRun.flags.b.bInvElecAngleDetect);
  }
  else
  {
    // if transition from invalid elec angle to valid then reset integral state
    if(sSpaceSpeedRun.flags.b.bInvElecAngleDetect)
    {
        _sint64_atomic_copy(&sSpaceSpeedRun.sqIntegral_1, &sqZero64bit) ;
        fast_atomic_clear_flag(sSpaceSpeedRun.flags.b.bInvElecAngleDetect);
    }

    /* il motore e' fasato: calcolo dinamicamente i limiti dell'integrale */
    /* converto PosErr attuale  da 64bit a floating point */
    atomic_read(&sqTmp, &sSpaceSpeedRun.sqPosErr64, sizeof(sqTmp));
    flPosErr = (FLOAT)sqTmp.hi * FLOAT_2POWER32 + (FLOAT)sqTmp.lo ; 

    if (sqTmp.hi < 0l)
    {   /* PosErr64 < 0 */
        if ((sSS_CntrLoopOut.sqFitPErrMin.hi != 0L) && (sSS_CntrLoopOut.sqFitPErrMin.lo != 0UL))
        {   /* PosErrMin e' diverso da zero, calcolo il limite di corrente negativo tenendo conto del PosErrActual e del PosErrMin */
            flPosErrMin = (FLOAT)sSS_CntrLoopOut.sqFitPErrMin.hi * FLOAT_2POWER32 + (FLOAT)sSS_CntrLoopOut.sqFitPErrMin.lo ;
            flPosErrRatio = (flPosErrMin - flPosErr) / flPosErrMin ;
            
            /* limito il rapporto tra [0; 1] */
            if (flPosErrRatio > 1.0)
                flPosErrRatio = 1.0 ;
            else if (flPosErrRatio <= 0.0)
                flPosErrRatio = 0.0 ;

            /* riduco solo il limite negativo, NON quello positivo */
            slMin = (SLONG)((FLOAT)slMin * flPosErrRatio) ;
        }
    }
    else
    {   /* PosErr64 >= 0 */
        if ((sSS_CntrLoopOut.sqFitPErrMax.hi != 0L) && (sSS_CntrLoopOut.sqFitPErrMax.lo != 0UL))
        {   /* PosErrMax e' diverso da zero, calcolo il limite di corrente positivo tenendo conto del PosErrActual e del PosErrMax */
            flPosErrMax = (FLOAT)sSS_CntrLoopOut.sqFitPErrMax.hi * FLOAT_2POWER32 + (FLOAT)sSS_CntrLoopOut.sqFitPErrMax.lo ;
            flPosErrRatio = (flPosErrMax - flPosErr) / flPosErrMax ; /* [0.0; 1.0] */

            /* limito il rapporto tra [0; 1] */           
            if (flPosErrRatio > 1.0)
                flPosErrRatio = 1.0 ;
            else if (flPosErrRatio <= 0.0)
                flPosErrRatio = 0.0 ;

            /* riduco solo il limite positivo, NON quello negativo */
            slMax = (SLONG)((FLOAT)slMax * flPosErrRatio) ;
        }
    }
  }
  /* ------------------------------------------------------------------------------------ */

  if(sSpaceSpeedRun.bParUsr)
      psParRT = &sSpaceSpeedRun.sParUsr;
  else
      psParRT = sSpaceSpeedRun.psParStd;

  /* copy back limits */
  atomic_write(&sSpaceSpeedRun.slILimMax, &slMax, sizeof(slMax));
  atomic_write(&sSpaceSpeedRun.slILimMin, &slMin, sizeof(slMin));

  /* update MaxIntegralValue to limit integral contribute */   
  swGlobalShift = psParRT->swGlobalGainShift;

  // adjust max integral gain
  sqTmp.hi =        slMax >> 16 ;
  sqTmp.lo = (ULONG)slMax << 16 ;
  if(swGlobalShift > 0)
    _sint64_shl(&sqTmp, (UWORD)swGlobalShift);
  else if(swGlobalShift < 0)
    _sint64_shr(&sqTmp, (UWORD)(-swGlobalShift));
  _sint64_atomic_copy(&(psParRT->sqIntegralMaxLimit), &sqTmp);

  // adjust min integral gain
  sqTmp.hi =        slMin >> 16 ;
  sqTmp.lo = (ULONG)slMin << 16 ;
  if(swGlobalShift > 0)
    _sint64_shl(&sqTmp, (UWORD)swGlobalShift);
  else if(swGlobalShift < 0)
    _sint64_shr(&sqTmp, (UWORD)(-swGlobalShift));
  _sint64_atomic_copy(&(psParRT->sqIntegralMinLimit), &sqTmp);

  /* ------------------------------------------------------------------------------------ */

  // swap RT param data area
  if(sSpaceSpeedRun.psParStd == &sSpaceSpeedRun.sPars[1])
  {
    sSpaceSpeedRun.psParStd = &sSpaceSpeedRun.sPars[0];
    sSpaceSpeedRun.psParUpd = &sSpaceSpeedRun.sPars[1];
  }
  else
  {
    sSpaceSpeedRun.psParStd = &sSpaceSpeedRun.sPars[1];
    sSpaceSpeedRun.psParUpd = &sSpaceSpeedRun.sPars[0];
  }

  AutoMaxPosError();
}

/* ######################################################################### */
static SBYTE NBitsFloat(FLOAT flInValue)
{
    if (flInValue != 0.0)
    	return (SBYTE)(log(flInValue) / log(2.0)) + 1;
    else
    	return NULL_GAIN ;
}

/* ######################################################################### */
/* conversione parametri anello di controllo da unita' ingegneristiche ad    *
 * unita' interne (con adattamento degli shift ECCETTO uwPosGainShift)       */
#ifdef _INFINEON_
BOOL PlcCtrLpGainsCompute(SS_PAR_GAINS huge * pfSrc, SS_RT_PARAMS huge * pvDst)
#else // _infineon_
BOOL PlcCtrLpGainsCompute(SS_PAR_GAINS * pfSrc, SS_RT_PARAMS * pvDst)
#endif // _infineon_
{
    // keep previous validity or not validity as parameters are only processed when changes
    static BOOL bRetVal=TRUE;
    // variables
    FLOAT flTmp, flAccKpRefNormalized, flAccKpFbkNormalized ;
    SBYTE sbNBitAccMax, sbNBitMax, sbNBitAbsMax, sbIndex, sbNBitPosKp ;
    SLONG slKi2Use;
    union
    {
        SBYTE sbNBit[7];
        SQWRD sqTmp;
    } u;

    // output variables
    SWORD swPosKp2Use, swSpdKpRef2Use, swSpdKpFbk2Use, swKi2Use, swGlobalGainShift2Use  ;
    SWORD swAccKpRef2Use, swAccKpFbk2Use, swAccShift2Use, swPosShift2Use ;

    // se struttura dati standard
    if(pfSrc == NULL)
    {
        // controllo se sono cambiati i valori
#ifdef _INFINEON_
    	if(hmemcmp(&sShadowFloatGains, &sSS_CntrLoopParam.sPars.sGains, sizeof(SS_PAR_GAINS)) == 0)
        {
            // prima di uscire ricopio in update i valori attuali
            hmemcpy(sSpaceSpeedRun.psParUpd, sSpaceSpeedRun.psParStd, sizeof(SS_RT_PARAMS));
            return bRetVal;
        }
#else // _infineon_
    	if(memcmp(&sShadowFloatGains, &sSS_CntrLoopParam.sPars.sGains, sizeof(SS_PAR_GAINS)) == 0)
        {
            // prima di uscire ricopio in update i valori attuali
            memcpy(sSpaceSpeedRun.psParUpd, sSpaceSpeedRun.psParStd, sizeof(SS_RT_PARAMS));
            return bRetVal;
        }
#endif // _infineon_
        // copio valori
        Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;
        atomic_read(&sShadowFloatGains.flKi,       &sSS_CntrLoopParam.sPars.sGains.flKi,       sizeof(FLOAT));
        atomic_read(&sShadowFloatGains.flPosKp,    &sSS_CntrLoopParam.sPars.sGains.flPosKp,    sizeof(FLOAT));
        atomic_read(&sShadowFloatGains.flSpdKpRef, &sSS_CntrLoopParam.sPars.sGains.flSpdKpRef, sizeof(FLOAT));
        atomic_read(&sShadowFloatGains.flSpdKpFbk, &sSS_CntrLoopParam.sPars.sGains.flSpdKpFbk, sizeof(FLOAT));
        atomic_read(&sShadowFloatGains.flAccKpRef, &sSS_CntrLoopParam.sPars.sGains.flAccKpRef, sizeof(FLOAT));
        atomic_read(&sShadowFloatGains.flAccKpFbk, &sSS_CntrLoopParam.sPars.sGains.flAccKpFbk, sizeof(FLOAT));
        Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;

        // e seleziono working copy
        pfSrc = &sShadowFloatGains;
        pvDst = sSpaceSpeedRun.psParUpd;
    }

    // default parametri validi
    bRetVal=TRUE;

    // converto valori utente (floating point, scala ingegneristica) in unita' interne (floating point)
    flAccKpRefNormalized = pfSrc->flAccKpRef * CONVFACT_ACCELGAIN ; /* A / (rad/s^2) */
    flAccKpFbkNormalized = pfSrc->flAccKpFbk * CONVFACT_ACCELGAIN ; /* A / (rad/s^2) */

    // Controllo sul Ki (inutile fare conversioni e scalature perche' e' un tempo)
	slKi2Use = (SLONG)(pfSrc->flKi * CONVFACT_INTEGRALGAIN) ; /* Hz = 1/s */                        
	if (slKi2Use > 32767l)
	{
		swKi2Use = 32767 ;
		bRetVal = FALSE;
	}
	else if (slKi2Use < 0l)
	{	
		swKi2Use = 0 ;
		bRetVal = FALSE;
	}
	else
		swKi2Use = (SWORD)slKi2Use ;

    // trovo il numero di bit dei valori floating point
    u.sbNBit[0] = NBitsFloat(flAccKpRefNormalized) ; 
    u.sbNBit[1] = NBitsFloat(flAccKpFbkNormalized) ;
    u.sbNBit[2] = NBitsFloat(pfSrc->flSpdKpRef * CONVFACT_SPEEDGAIN) ;
    u.sbNBit[3] = NBitsFloat(pfSrc->flSpdKpFbk * CONVFACT_SPEEDGAIN) ;
    u.sbNBit[4] = NBitsFloat(pfSrc->flPosKp * CONVFACT_POSGAIN) ;

    // cerco il massimo tra le accelerazioni
    sbNBitAccMax = u.sbNBit[0] ;
    if (u.sbNBit[1] > sbNBitAccMax)
        sbNBitAccMax = u.sbNBit[1] ;
                                       
    // cerco il massimo escludendo le accelerazioni
    sbNBitMax = u.sbNBit[2] ;
    if (u.sbNBit[3] > sbNBitMax)
        sbNBitMax = u.sbNBit[3] ;

    if (u.sbNBit[4] > sbNBitMax)
        sbNBitMax = u.sbNBit[4] ;


    // calcolo lo shift delle accelerazioni da usare (solo sinistro, [0; 15]) tenendo condo ANCHE degli altri guadagni
    if ((u.sbNBit[0] == NULL_GAIN) && (u.sbNBit[1] == NULL_GAIN))
    {   // i guadagni delle accelerazioni sono entrambi zero: tengo a zero lo shift delle accelerazioni
        swAccShift2Use = 0 ;
        flAccKpRefNormalized = 0.0 ;
        flAccKpFbkNormalized = 0.0 ;
    }
    else
    {   // considero ANCHE altri guadagni
        if (sbNBitMax == NULL_GAIN)
        {   // tutti gli altri guadagni sono zero: metto a posto SOLO lo shift dell'accelerazione
            swAccShift2Use = 0 ;
            flAccKpRefNormalized = flAccKpRefNormalized ;
            flAccKpFbkNormalized = flAccKpFbkNormalized ;
        }
        else
        {   // solo shift sinistro, [0; 15]; se > +15, allora clippo a +15	
            swAccShift2Use = sbNBitAccMax - sbNBitMax ;
            if (swAccShift2Use < 0) 
                swAccShift2Use = 0 ;
            else if (swAccShift2Use > 15)
                swAccShift2Use = 15 ;
        
            if (swAccShift2Use != 0)
            {
                flTmp = 1.0 / (FLOAT)((SLONG)1 << swAccShift2Use) ; // pow(2.0, swAccShift2Use); code optimization
                flAccKpRefNormalized = flAccKpRefNormalized * flTmp  ;
                flAccKpFbkNormalized = flAccKpFbkNormalized * flTmp ;
            }
            else
            {
                flAccKpRefNormalized = flAccKpRefNormalized ;
                flAccKpFbkNormalized = flAccKpFbkNormalized ;
            }	
        }
    }
    
    // ri-calcolo il numero di bit per le accelerazioni gia' scalate 
    u.sbNBit[5] = NBitsFloat(flAccKpRefNormalized) ;
    u.sbNBit[6] = NBitsFloat(flAccKpFbkNormalized) ;
    
    // cerco il massimo assoluto comprendendo anche le accelerazioni
    sbNBitAbsMax = u.sbNBit[2] ;
    for (sbIndex = 3; sbIndex < 7; sbIndex++)
    {
        if (u.sbNBit[sbIndex] > sbNBitAbsMax)
            sbNBitAbsMax = u.sbNBit[sbIndex] ;
    }

    // calcolo lo shift globale da usare [-8; 15] *)
    if (sbNBitAbsMax == NULL_GAIN)
        swGlobalGainShift2Use = 0 ;
    else
    {
        swGlobalGainShift2Use = 15 - sbNBitAbsMax ;
    	
        if (swGlobalGainShift2Use > 15)
        {
            swGlobalGainShift2Use = 15 ;
            bRetVal = FALSE;
        }		
        else if (swGlobalGainShift2Use < -8)
        {
            swGlobalGainShift2Use = -8 ;
            bRetVal = FALSE;
        }
    }
    
    if (swGlobalGainShift2Use > 0)
        flTmp = (FLOAT)((SLONG)1 << swGlobalGainShift2Use) ;
    else if (swGlobalGainShift2Use < 0)
        flTmp = 1.0 / (FLOAT)((SLONG)1 << (-swGlobalGainShift2Use)) ;	
    else
        flTmp = 1.0 ;        			

    swAccKpRef2Use = (SWORD)(flAccKpRefNormalized * flTmp) ;
    swAccKpFbk2Use = (SWORD)(flAccKpFbkNormalized * flTmp) ;
    swSpdKpRef2Use = (SWORD)(pfSrc->flSpdKpRef * CONVFACT_SPEEDGAIN * flTmp) ;
    swSpdKpFbk2Use = (SWORD)(pfSrc->flSpdKpFbk * CONVFACT_SPEEDGAIN * flTmp) ;
    flTmp          = pfSrc->flPosKp * CONVFACT_POSGAIN * flTmp ;

    // calcolo il numero di bit del guadagno di posizione solo se e' diverso da 0: 
    if (flTmp == 0.0)
        sbNBitPosKp = 0 ;
    else
        sbNBitPosKp = NBitsFloat(flTmp) ; 

    // se e' minore di 4 (ovvero PosKp (u.i.) < 16), allora lo shifto a destra fino ad averlo maggiore di 16 
    if (sbNBitPosKp < 4 && sbNBitPosKp != NULL_GAIN)
    {
        swPosShift2Use = 4 - sbNBitPosKp ;

        // consentito SOLO SHIFT DESTRO [0; 15]; 
        // controllo solo il limite superiore (swPosShift2Use > 15)
        // non controllo il limite inferiore (swPosShift2Use <= 0) perche' in questo 'if' non potra' mai succedere
        if (swPosShift2Use > 15)
        {
            swPosShift2Use = 15 ;		
            bRetVal = FALSE;
        }

        swPosKp2Use = (SWORD)(flTmp * ((SLONG)1 << swPosShift2Use)) ;
    }         
    else
    {
        swPosShift2Use = 0 ;
        swPosKp2Use    = (SWORD)flTmp ;
    }
    
    // copio in run
    pvDst->swPostnKp         = swPosKp2Use ;
    pvDst->swSpdKpRef        = swSpdKpRef2Use ; 
    if(sSS_CntrLoopParam.sPars.flags.b.bUseDifferentKp)
      pvDst->swSpdKpFbk      = swSpdKpFbk2Use ;
    else
      pvDst->swSpdKpFbk      = swSpdKpRef2Use ;
    pvDst->swAccKpRef        = swAccKpRef2Use ;
    pvDst->swAccKpFbk        = swAccKpFbk2Use ;
    pvDst->swKi              = swKi2Use ;
    pvDst->uwAccGainShift    = (UWORD)swAccShift2Use ;
    pvDst->uwPosGainShift    = (UWORD)swPosShift2Use ;
    pvDst->swGlobalGainShift = swGlobalGainShift2Use ;

    // adjust max integral gain
    atomic_read(&slKi2Use, &sSpaceSpeedRun.slILimMax, sizeof(slKi2Use));
    u.sqTmp.hi =        slKi2Use >> 16 ;
    u.sqTmp.lo = (ULONG)slKi2Use << 16 ;
    if(swGlobalGainShift2Use > 0)
      _sint64_shl(&u.sqTmp, (UWORD)swGlobalGainShift2Use);
    else if(swGlobalGainShift2Use < 0)
      _sint64_shr(&u.sqTmp, (UWORD)(-swGlobalGainShift2Use));
    INT64_COPY(pvDst->sqIntegralMaxLimit, u.sqTmp);

    // adjust min integral gain
    atomic_read(&slKi2Use, &sSpaceSpeedRun.slILimMin, sizeof(slKi2Use));
    u.sqTmp.hi =        slKi2Use >> 16 ;
    u.sqTmp.lo = (ULONG)slKi2Use << 16 ;
    if(swGlobalGainShift2Use > 0)
      _sint64_shl(&u.sqTmp, (UWORD)swGlobalGainShift2Use);
    else if(swGlobalGainShift2Use < 0)
      _sint64_shr(&u.sqTmp, (UWORD)(-swGlobalGainShift2Use));
    INT64_COPY(pvDst->sqIntegralMinLimit, u.sqTmp);

    return bRetVal;  
}

/* ######################################################################### */
/* conversione parametri anello di controllo da unita' interne ad unita'     *
 * unita' ingegneristiche (con calcolo degli shift                           */ 
static BOOL CtrLpGainsInternalUnits2EngineeringUnits(void)
{
    FLOAT flAccKpRef, flAccKpFbk, flSpdKpRef, flSpdKpFbk, flPosKp, flPosKpUsed, flKi, flGainShift, flAccShift ;
    UWORD uwTmp;
    SWORD swTmp;
    BOOL bValid=TRUE;

    // controllo se sono cambiati i valori espressi in intero, se no esco
#ifdef _INFINEON_
    if(!hmemcmp(&sShadowIntegerGains, &sSS_CntrLoopParam.sIntGains, sizeof(SS_V15_PAR_GAINS)))
        return bValid;
#else
    if(!memcmp(&sShadowIntegerGains, &sSS_CntrLoopParam.sIntGains, sizeof(SS_V15_PAR_GAINS)))
        return bValid;
#endif


    // mi copio i parametri interi su variabili d'appoggio, variabile per variabile
    // in modo tale da copiare le word in maniera atomica
    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;
    sShadowIntegerGains.swAccKpRef        = sSS_CntrLoopParam.sIntGains.swAccKpRef ;       
    sShadowIntegerGains.swAccKpFbk        = sSS_CntrLoopParam.sIntGains.swAccKpFbk ;       
    sShadowIntegerGains.swSpdKpRef        = sSS_CntrLoopParam.sIntGains.swSpdKpRef ;       
    sShadowIntegerGains.swSpdKpFbk        = sSS_CntrLoopParam.sIntGains.swSpdKpFbk ;       
    sShadowIntegerGains.swPostnKp         = sSS_CntrLoopParam.sIntGains.swPostnKp ;        
    sShadowIntegerGains.swKi              = sSS_CntrLoopParam.sIntGains.swKi ;             
    sShadowIntegerGains.uwAccGainShift    = sSS_CntrLoopParam.sIntGains.uwAccGainShift ;   
    sShadowIntegerGains.uwPosGainShift    = sSS_CntrLoopParam.sIntGains.uwPosGainShift ;   
    sShadowIntegerGains.swGlobalGainShift = sSS_CntrLoopParam.sIntGains.swGlobalGainShift ;
    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;

    // controllo validita'
    swTmp = CheckSWordParam(sShadowIntegerGains.swGlobalGainShift, GAIN_GLOBAL_MAX_SHIFT_RIGHT, GAIN_GLOBAL_MAX_SHIFT_LEFT) ; /* 60f9.9h */
    if(swTmp != sShadowIntegerGains.swGlobalGainShift)
    {
      sShadowIntegerGains.swGlobalGainShift = swTmp;
      ParChk_SignalValueError(PARCC_SS_INVALIDGLOBALSHIFT);
      bValid=FALSE;
    }
    else
      ParChk_ResetValueError(PARCC_SS_INVALIDGLOBALSHIFT);

    uwTmp = CheckUWordParam(sShadowIntegerGains.uwPosGainShift, GAIN_POSITION_MAX_SHIFT_RIGHT) ; /* 60f9.ch */
    if(uwTmp != sShadowIntegerGains.uwPosGainShift)
    {
      sShadowIntegerGains.uwPosGainShift = uwTmp;
      ParChk_SignalValueError(PARCC_SS_INVALIDPOSSHIFT);
      bValid=FALSE;
    }
    else
      ParChk_ResetValueError(PARCC_SS_INVALIDPOSSHIFT);

    uwTmp = CheckUWordParam(sShadowIntegerGains.uwAccGainShift, GAIN_ACCEL_MAX_SHIFT_LEFT)     ; /* 60f9.dh */
    if(uwTmp != sShadowIntegerGains.uwAccGainShift)
    {
      sShadowIntegerGains.uwAccGainShift = uwTmp;
      ParChk_SignalValueError(PARCC_SS_INVALIDACCSHIFT);
      bValid=FALSE;
    }
    else
      ParChk_ResetValueError(PARCC_SS_INVALIDACCSHIFT);

    // metto subito a posto il guadagno di posizione
    if (sShadowIntegerGains.uwPosGainShift != 0)
        flPosKpUsed = (FLOAT)sShadowIntegerGains.swPostnKp / (FLOAT)((SLONG)1 << sShadowIntegerGains.uwPosGainShift) ;
    else
        flPosKpUsed = (FLOAT)sShadowIntegerGains.swPostnKp ;

	// in base allo shift globale scalo i guadagni (floating point)
    if (sShadowIntegerGains.swGlobalGainShift > 0)
        flGainShift = 1.0 / (FLOAT)((SLONG)1 << sShadowIntegerGains.swGlobalGainShift) ;
    else if (sShadowIntegerGains.swGlobalGainShift < 0)
        flGainShift = (FLOAT)((SLONG)1 << -sShadowIntegerGains.swGlobalGainShift) ;   
    else
        flGainShift = 1.0 ;

    flAccKpRef = (FLOAT)sShadowIntegerGains.swAccKpRef * flGainShift ;
    flAccKpFbk = (FLOAT)sShadowIntegerGains.swAccKpFbk * flGainShift ;
    flSpdKpRef = (FLOAT)sShadowIntegerGains.swSpdKpRef * flGainShift ;
    flSpdKpFbk = (FLOAT)sShadowIntegerGains.swSpdKpFbk * flGainShift ;
    flPosKp    = flPosKpUsed * flGainShift ;

	// considero anche lo shift dell'accelerazione
    if (sShadowIntegerGains.uwAccGainShift != 0)
    {
        flAccShift = (FLOAT)((SLONG)1 << sShadowIntegerGains.uwAccGainShift) ;
        flAccKpRef = flAccKpRef * flAccShift ;
        flAccKpFbk = flAccKpFbk * flAccShift ;
    }
		
    // converto in valori utente (floating point, scala ingegneristica) da unita' interne (floating point)
    flAccKpRef = flAccKpRef * CONVFACT_ACCELGAIN_INV ; /* A / (rad/s^2) */
    flAccKpFbk = flAccKpFbk * CONVFACT_ACCELGAIN_INV ; /* A / (rad/s^2) */
    flSpdKpRef = flSpdKpRef * CONVFACT_SPEEDGAIN_INV ; /* A / (rad/s)   */
    flSpdKpFbk = flSpdKpFbk * CONVFACT_SPEEDGAIN_INV ; /* A / (rad/s)   */
    flPosKp    = flPosKp    * CONVFACT_POSGAIN_INV   ; /* A / rad       */
    flKi       = (FLOAT)sShadowIntegerGains.swKi * CONVFACT_INTEGRALGAIN_INV ; /* Hz = 1/s */  

    // ricopio in parametri float
    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;
    atomic_write(&sSS_CntrLoopParam.sPars.sGains.flAccKpRef , &flAccKpRef , sizeof(FLOAT));
    atomic_write(&sSS_CntrLoopParam.sPars.sGains.flAccKpFbk , &flAccKpFbk , sizeof(FLOAT));
    atomic_write(&sSS_CntrLoopParam.sPars.sGains.flSpdKpRef , &flSpdKpRef , sizeof(FLOAT));
    atomic_write(&sSS_CntrLoopParam.sPars.sGains.flSpdKpFbk , &flSpdKpFbk , sizeof(FLOAT));
    atomic_write(&sSS_CntrLoopParam.sPars.sGains.flPosKp    , &flPosKp    , sizeof(FLOAT));
    atomic_write(&sSS_CntrLoopParam.sPars.sGains.flKi       , &flKi       , sizeof(FLOAT));
    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;

    return bValid;
}

/* ######################################################################### */
/* Calcolo max pos error in funzione del guadagno di posizione e dei limiti di corrente   */
static void AutoMaxPosError(void)
{
    SLONG slMax, slMin ;  
    FLOAT flMaxPosErr, flMinPosErr, flPosKp, flPosKpInverse ;  
    SQWRD sqPstnErrMax, sqPstnErrMin, sqMinPosErr ;

    atomic_read(&flPosKp, &sSS_CntrLoopParam.sPars.sGains.flPosKp, sizeof(FLOAT)) ;
    flPosKpInverse = 1.0 / (flPosKp * FLOAT_2PI_IU) ; // ottimizzazione codice 

    if(flPosKp > 0.0)
    {
        if (sSS_CntrLoopParam.sPars.flags.b.bUseDifferentKp)
        {   // se uso guadagni Kp di velocita' separati, allora 'apro' al massimo l'errore di posizione
            slMax = 1073741823l ; // 2^30 - 1  = 10737.41823 A
            slMin = -slMax ;
        }
        else
        {
            atomic_read(&slMax, &sSS_CntrLoopIn.psIqLimit->slMax, sizeof(slMax)) ; 
            atomic_read(&slMin, &sSS_CntrLoopIn.psIqLimit->slMin, sizeof(slMin)) ;
        }

        // faccio i calcoli sdoppiati perche' i limiti di corrente possono essere asimmetrici
        flMaxPosErr = (FLOAT)( slMax) * flPosKpInverse ; // i.u. (rad/2PI)
        flMinPosErr = (FLOAT)(-slMin) * flPosKpInverse ; // cambio segno per lavorare con numeri positivi

        // MAX PosErr
        sqPstnErrMax.hi = (SLONG)( flMaxPosErr) ; // n. di giri (u.i.)
        sqPstnErrMax.lo = (ULONG)((flMaxPosErr - (FLOAT)sqPstnErrMax.hi) * 4294967296.0) ; // angolo (u.i.), 4294967296.0 = 2^32 = 360°
        
        // MIN PosErr (lavoro con numeri positivi)
        sqMinPosErr.hi = (SLONG)( flMinPosErr) ; // n. di giri (u.i.)
        sqMinPosErr.lo = (ULONG)((flMinPosErr - (FLOAT)sqMinPosErr.hi) * 4294967296.0) ; // angolo (u.i.), 4294967296.0 = 2^32 = 360°
        _sint64_sub(&sqPstnErrMin, &sqZero64bit, &sqMinPosErr) ; // ricambio segno
    }
    else
    {
        INT64_COPY(sqPstnErrMax, sqZero64bit) ;
        INT64_COPY(sqPstnErrMin, sqZero64bit) ;
    }

    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;
    _sint64_atomic_copy(&sSS_CntrLoopOut.sqFitPErrMax, &sqPstnErrMax) ;
    _sint64_atomic_copy(&sSS_CntrLoopOut.sqFitPErrMin, &sqPstnErrMin) ;
    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;
}

//***************************************************************************
// Convert old V14 data parameters if found while restoring data from NV mem

#ifdef _INFINEON_
SWORD Ss_GetV15Params(SS_V15_PARAMS huge * pvSrc, UWORD uwSize)
#else // _infineon_
SWORD Ss_GetV15Params(SS_V15_PARAMS * pvSrc, UWORD uwSize)
#endif // _infineon_

{
    if(sizeof(SS_V15_PARAMS)!=uwSize)
        return 0;

    sSS_CntrLoopParam.sPars.flags.b.bUseDifferentKp   = pvSrc->flags.b.bUseDifferentKp;
    sSS_CntrLoopParam.sIntGains                       = pvSrc->sGains;
    sSS_CntrLoopParam.sPars.sUsrOutVal32              = pvSrc->sUsrOutVal32;

    return 0;
}

//***************************************************************************
// Gain set selector
#ifdef _INFINEON_
BOOL PlcCtrLpGainsSelect(SS_RT_PARAMS huge * pvSrc)
#else // _infineon_
BOOL PlcCtrLpGainsSelect(SS_RT_PARAMS * pvSrc)
#endif // _infineon_
{
    // if NULL then switch back to standard mode
    if(pvSrc == NULL)
        sSpaceSpeedRun.bParUsr = FALSE;
    else
    // copy on user definition and activate
    {
#ifdef _INFINEON_
        hmemcpy(&sSpaceSpeedRun.sParUsr, pvSrc, sizeof(SS_RT_PARAMS));
#else // _infineon_
        memcpy(&sSpaceSpeedRun.sParUsr, pvSrc, sizeof(SS_RT_PARAMS));
#endif // _infineon_
        sSpaceSpeedRun.bParUsr = TRUE;
    }
    
    return TRUE;
}
