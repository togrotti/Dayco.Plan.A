/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : Positioner.c                                               */
/* Author      : Cristiano Tognetti                                         */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/
#include <stdlib.h>
#include <math.h>

#include "common\CommonDefines.h"
#include "system\SysAppGlobals.h"
#include "common\Int64Functions.h"

#include "drive\Positioner.h"
#include "common\DspFunctions.h"

#include "common\TaskScheduler.h"
#include "common\CommonEncoderManager.h"
#include "common\CommonUtility.h"

#include "system\SysLogManagement.h"

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
#define RAMP_POSITIVE      1
#define RAMP_NEGATIVE     -1
#define RAMP_NO            0
#define TWO_POWER_THIRTY   1073741824
#define FIVE               5
#define SEVENTEEN          17
#define SVA_COUNTER_LIMIT  3

/* ============================== structures =============================== */
typedef struct {
  /* Flags */
  SWORD swK1 ;
  SWORD swK2 ;
  SWORD sw1_K1 ;
  UWORD uwAbsK1 ;

  SQWRD sqFilterStatus64 ;
  SQWRD sqIntegralStatus64Spd ;

  SLONG slRGDeltaPos ;
} SRAMP ;

typedef struct {
  /* Flags */
  union {
      struct {
        UWORD bFlagSvaActive        :1;
        UWORD bPrevTgtPosReached    :1;
        UWORD bPostnerNegative      :1;
        UWORD bPosIntegrateNoAccel  :1;
        UWORD bEnableAntiWindupPosErr :1;
        UWORD bErrSentMotorBlocked  :1;
        UWORD bDisSpeedUpdate       :1;
      } b ;
      UWORD w ;
  } flags ;
    
  /* parameters imagine used at runtime */
  SQWRD sqPstnErrMax ;
  SQWRD sqPstnErrMin ; // = -sqPstnErrMax
    
  /* SVA */
  UWORD uwSvaCounter ;
    
  SLONG slProfileDecLimit ;
  SLONG slDec2Use ;
  SWORD swDemandAccelCoins ;  /* per gestire le accelerazioni mooolto lente */
    
  SQWRD sqDeltaPos ; /* = EndPosition - DemandPosition_1 */

  SLONG slTgtSpd2Use ;
  SQWRD sqTgtPos2Use ;

  SLONG slProfileAcc  ;
  SLONG slProfileDec  ;

  SWORD swMotBlockedTimer ;
  SWORD swMotBlockTout ;

  /* SRamp */
  SRAMP sSRamp ;
  SQWRD sqSRampPstnErrMax ;
  SQWRD sqSRampPstnErrMin ; // = -sqSRampPstnErrMax

  SQWRD sqLocPosErr ;

  GLB_KINEMATIC_DATA * psDemand ;
  SQWRD * psqPosErr ;

} POSITIONER_RUNTIME ;


PO_POSITIONER_IN     sPo_UsrPostnerIn ; // ------ input from usr (Default)
PO_POSITIONER_IN     sPo_PlcPostnerIn ; // ------ input from plc

// ------ positioner module
PO_POSITIONER_IN    *psPo_PostnerIn  ; /* reference */
PO_POSITIONER_HDL    sPo_Handlers    ;
GLB_KINEMATIC_DATA  *psPo_InFeedBack ; /* feedback  */
SS_OUTPUT           *psPo_InSSCtrlLoop ;
MH_CURLIMITFLAGS    *psPo_ILimitActive ;
ENCMGR_SPACEFB_EXT  *psPo_InFeedbackExt ; /* feedback filtered speed */
PO_POSITIONER_OUT    sPo_PostnerOut ;
PO_POSITIONER_PARAMS sPo_PostnerParam ;
PO_PLC_WORKS         sPo_PlcWorks = { 0 } ;

POSITIONER_RUNTIME   sPostnerRun ;

#define  TEMPFLAG(x)   ulSystemWarnings|=(1<<x)
// default
#ifdef _INFINEON_
const PO_POSITIONER_PARAMS huge sPo_PostnerDefParam =
#else
const PO_POSITIONER_PARAMS sPo_PostnerDefParam =
#endif
{
  0,       // End Speed (in position mode)
  17895700,// Profile Velocity: 2000 rpm
  1000000, // Profile Acceleration
  1000000, // Profile Deceleration
  10000000,// Decelerazione per il QuickStop
  {0,5},   // Max Position Error allowed: 5 mech turns
  4475,    // Threshold to consider speed = 0: 0.5 rpm
  100,     // Motor blocked timeout [msec]
  {0},     // SRamp disabled
  -16,     // SRamp: K1
  435,     // SRamp: K2
  8545,    // SRamp: SRampAbsSpeedThreshold 
  {0,5},   // SRamp Max Position Error allowed: 5 mech turns
} ;

/* =========================== global  variables =========================== */

/* =============================== functions =============================== */
static BOOL Po_ModeOfFunction8KHz(void) ;
static void Po_PositionerBkGd(void) ;
static BOOL Po_PositionerRst(void) ;

static void Positioner8KHz(void) ;
static void SpeedRamp8KHz(void) ;
static void PositionIntegrate8KHz(void) ;
static void Interpolation8KHz(void) ;
static void SpeedNoRamp8KHz(void) ;
static void CalculateAccDec(void) ;

static void SRamp8kHz(void) ;
static void SRampReset(void) ;
static void SRampBkGd(void) ;

/* ######################################################################### */
BOOL Po_PositionerInit(void)
{
  SQWRD sqDecelerationLimit ;
  SWORD swDecelerationMultiplier = FIVE ;
  SLONG slDecelerationLimit ;

  sPo_PostnerOut.flags.b.bSRampEnabled = sPo_PostnerParam.flags.b.bSRampEnable ;

  if(sPo_PostnerOut.flags.b.bSRampEnabled)
  {
    sPostnerRun.psDemand = &sPo_PostnerOut.sLocDemand;
    sPostnerRun.psqPosErr = &sPostnerRun.sqLocPosErr;
  }
  else
  {
    sPostnerRun.psDemand = &sPo_PostnerOut.sDemand;
    sPostnerRun.psqPosErr = &sPo_PostnerOut.sqPosErr;
  }

  sPostnerRun.slProfileAcc   = sPo_PostnerParam.slProfileAcc ;
  sPostnerRun.slProfileDec   = sPo_PostnerParam.slProfileDec ;  
    
  /* Position error limit: turns (max 2^31); degrees (360° = 2^32) */
  _sint64_atomic_copy(&sPostnerRun.sqPstnErrMax, &sPo_PostnerParam.sqPstnErrMax) ;
  _sint64_sub(&sPostnerRun.sqPstnErrMin, &sqZero64bit, &sPostnerRun.sqPstnErrMax) ;

  /* calcolo il limite sulla decelerazione */
  slDecelerationLimit = sPostnerRun.slProfileDec + 100 ;  
  _sint64_mul_32_16(&sqDecelerationLimit, &slDecelerationLimit, &swDecelerationMultiplier) ;
  _sint64_shr(&sqDecelerationLimit, 2) ;
  sPostnerRun.slProfileDecLimit = _sint64toSint32(&sqDecelerationLimit) ; /* slDecelerationLimit = ((slProfileDec+1) * 5)/4 */ 
    
  Po_PositionerRst() ; /* reset outvalues */
  psPo_PostnerIn->flags.b.bPostnerEnable = FALSE ; // keep reset positioner module    
  SRampReset() ;
    
  /* install background function */ 
  if(!TaskSched_AddBackgroundTask(&Po_PositionerBkGd))
    return FALSE ;  
  
  /* install 8KHz function */ 
  return TaskSched_AddRTTask(&Po_ModeOfFunction8KHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0) ;
}


/* ######################################################################### */
static BOOL Po_ModeOfFunction8KHz(void)
{
  sPostnerRun.flags.b.bPosIntegrateNoAccel = FALSE;

  if (bSysStatAlarmsReset)
    sPostnerRun.flags.b.bErrSentMotorBlocked = FALSE ; /* clear errors */

  if (psPo_PostnerIn->flags.b.bPostnerEnable)      //位置模式使能
  { /* executed only if enabled */
	  //TEMPFLAG(13);
    if (psPo_PostnerIn->flags.b.bQuickStop)      //快速停机指令
    {
      sPostnerRun.slTgtSpd2Use = 0L ;
      sPostnerRun.slDec2Use = psPo_PostnerIn->slQuickStopDec2Use ;

      sPostnerRun.flags.b.bFlagSvaActive = FALSE ;

      // se sono in quickstop e mi mettono in posizione, non voglio far subito 
      // partire il posizionatore: me lo devono imporre esplicitamente
      sPo_PostnerOut.flags.b.bTgtPosReached = TRUE ;

      SpeedRamp8KHz() ; 
    }
    else
    { 
      // allow acceleration/deceleration change in fast task
      if (sPo_PlcWorks.flags.b.bRampFastChange)         //修改加减速
        CalculateAccDec() ;        //计算新的加减速

      if (psPo_PostnerIn->flags.b.bVelocity)
      {
        sPostnerRun.slDec2Use    = sPostnerRun.slProfileDec ;
        if(psPo_PostnerIn->flags.b.bUseLocalAccSpd)
          sPostnerRun.slTgtSpd2Use = psPo_PostnerIn->slLocalSpeed ;
        else
          sPostnerRun.slTgtSpd2Use = psPo_PostnerIn->slTargetSpeed ;

        sPostnerRun.flags.b.bFlagSvaActive = FALSE ;

        // se sono in quickstop e mi mettono in posizione, non voglio far subito 
        // partire il posizionatore: me lo devono imporre esplicitamente
        sPo_PostnerOut.flags.b.bTgtPosReached = TRUE ;

        SpeedRamp8KHz() ; 
      }
      else if (psPo_PostnerIn->flags.b.bPosition)
      {
        sPostnerRun.slDec2Use = sPostnerRun.slProfileDec ;         //实际生效的减速度，设置
        _sint64_atomic_copy(&sPostnerRun.sqTgtPos2Use, &psPo_PostnerIn->sqTargetPostn) ;                 
      
        // comincio il posizionamento: la flag bNewTarget viene resettata dal chiamante!! 
        if (psPo_PostnerIn->flags.b.bNewTarget)                                  
          sPo_PostnerOut.flags.b.bTgtPosReached = FALSE ; //新的目标位置不是0，则说明指令依然在运行中
                                                                  
        Positioner8KHz() ;
      }
      else if (psPo_PostnerIn->flags.b.bInterpolate)
      {
        _sint64_atomic_copy(&sPostnerRun.sqTgtPos2Use, &psPo_PostnerIn->sqTargetPostn) ;                 
        
        sPostnerRun.flags.b.bFlagSvaActive = FALSE ;
        sPostnerRun.flags.b.bEnableAntiWindupPosErr = FALSE ;

        Interpolation8KHz() ;
      } 
      else if (psPo_PostnerIn->flags.b.bDirectSpeed)
      {
        if(psPo_PostnerIn->flags.b.bUseLocalAccSpd)
          sPostnerRun.slTgtSpd2Use = psPo_PostnerIn->slLocalSpeed ;
        else
          sPostnerRun.slTgtSpd2Use = psPo_PostnerIn->slTargetSpeed ;

        sPostnerRun.flags.b.bPosIntegrateNoAccel = TRUE;
        sPostnerRun.flags.b.bFlagSvaActive = FALSE ;
        sPostnerRun.flags.b.bEnableAntiWindupPosErr = TRUE ;

        SpeedNoRamp8KHz() ;
      }
      else
        Po_PositionerRst() ;
    }
  }       
  else
    Po_PositionerRst() ; /* keep everything stopped */

  sPostnerRun.flags.b.bPrevTgtPosReached=sPo_PostnerOut.flags.b.bTgtPosReached;
    
  return TRUE ;
}


/* ######################################################################### */
static void Positioner8KHz(void)
{
  SWORD swDeltaPosSign ;
  SQWRD sqDeltaPosCorrected ; /* = sqDeltaPos + DemandSpeed_1/2 */
  SLONG slAbsValDecSva, slHalfDemandSpeed ;
  SLONG slEndVel ;

  if(!sPo_PostnerOut.flags.b.bTgtPosReached)
  { /* determino quanto manca rispetto alla sqTargetPostn: DeltaPos = sqTargetPostn - sqDemandPostn_1 */
    _sint64_sub(&sPostnerRun.sqDeltaPos, &sPostnerRun.sqTgtPos2Use, &sPostnerRun.psDemand->sqPostn) ;
  
    /* guardo il segno del Delta di posizione:  */
    swDeltaPosSign = _sint64_comp(&sPostnerRun.sqDeltaPos, &sqZero64bit) ;
 
    /* se primo passaggio (inizio posizionamento) tengo conto della direzione per la end_velocity */
    if(sPostnerRun.flags.b.bPrevTgtPosReached)
        sPostnerRun.flags.b.bPostnerNegative=(swDeltaPosSign<0);
    slEndVel=sPostnerRun.flags.b.bPostnerNegative?-sPo_PostnerParam.slEndVelocity:sPo_PostnerParam.slEndVelocity;

    if (swDeltaPosSign == 1)
    { /* Delta > 0 */
      if(sPostnerRun.psDemand->slSpeed < slEndVel)
        sPostnerRun.slTgtSpd2Use = slEndVel ;
      else
        sPostnerRun.slTgtSpd2Use = sPo_PostnerParam.slProfileVel ;
    }
    else if (swDeltaPosSign == -1)
    { /* Delta < 0 */
      if(sPostnerRun.psDemand->slSpeed > slEndVel)
        sPostnerRun.slTgtSpd2Use = slEndVel ;
      else
        sPostnerRun.slTgtSpd2Use = -sPo_PostnerParam.slProfileVel ;
    }
    else
      sPostnerRun.slTgtSpd2Use = slEndVel ; /* Delta = 0 */

    /* ======================================= calcolo SVA ======================================= */  
    /* calcolo sPo_PostnerOut.slDemandSpeed/2 (mi serve per fare la '64+32') */
    slHalfDemandSpeed = sPostnerRun.psDemand->slSpeed >> 1 ; 

    _sint64_add_64_32(&sqDeltaPosCorrected, &sPostnerRun.sqDeltaPos, &slHalfDemandSpeed) ; /* sqDeltaPosCorrected = sqDeltaPos + DemandSpeed_1/2 */

    /* AbsValDec2UseCalculated = (DemandSpeed_1^2 - SpeedEnd^2)/(2 * (PostnEnd - sqDemandPosition_1 + DemandSpeed_1/2)) */
    slAbsValDecSva = _sva_evaluation(sPostnerRun.psDemand->slSpeed, slEndVel, &sqDeltaPosCorrected) ;          

    /* Clippo la decelerazione SVA */
    if (slAbsValDecSva > sPostnerRun.slProfileDecLimit)        //limit the deceleration
      slAbsValDecSva = sPostnerRun.slProfileDecLimit ; 

    /* verifico se devo usare la decelerazione SVA oppure la profileDec */
    if (slAbsValDecSva > sPostnerRun.slProfileDec)		//If the length is short than calculate, decelerate immediately
    {
      sPostnerRun.flags.b.bFlagSvaActive = TRUE ; /* devo usare SVA */
      sPostnerRun.uwSvaCounter = 0 ; /* resetto il contatore */
    }
    else
      sPostnerRun.uwSvaCounter++ ; /* incremento il contatore */

    if(sPostnerRun.uwSvaCounter >= SVA_COUNTER_LIMIT)
    { /* contatore ha raggiunto il limite: non uso la SVA (antiglitch) */ 
      sPostnerRun.flags.b.bFlagSvaActive = FALSE ;
      sPostnerRun.uwSvaCounter = 0 ;
    }
 
    if (sPostnerRun.flags.b.bFlagSvaActive)      //use the deceleration calculated not the parameter set by user
    { /* uso l'accelerazione Sva */
      sPostnerRun.slDec2Use = slAbsValDecSva ;    //
      sPostnerRun.slTgtSpd2Use = slEndVel ;
    }    
  }

  SpeedRamp8KHz() ; /* eseguo le rampe */
}


/* ######################################################################### */
static void SpeedRamp8KHz(void)
{
    ULONG ulAbsDeltaSpd32, ulAbsValDmndAcc ;
    ULONG ulAbsDeltaPos32, ulAbsValDmndSpd ;
    ULONG ulAbsDltSpdEndVel32;

    // //////////////////////////////////////////////// //
    // calcolo accelerazione da usare per fare le rampe //
    // //////////////////////////////////////////////// //
    if (sPostnerRun.slTgtSpd2Use >= 0)
    {   // TargetSpeed >= 0
        if(sPostnerRun.psDemand->slSpeed >= 0)
        {   // DemandSpeed_1 >= 0 
            if (sPostnerRun.psDemand->slSpeed > sPostnerRun.slTgtSpd2Use)
            {   // +DemandSpeed_1 > +TargetSpeed
                // devo decelerare: accel2use = -decel
                sPo_PostnerOut.flags.b.bInRamp = TRUE ;    // RAMP_NEGATIVE
                sPostnerRun.psDemand->slAccel = -sPostnerRun.slDec2Use ;
            }
            else if(sPostnerRun.psDemand->slSpeed < sPostnerRun.slTgtSpd2Use)
            {   // +DemandSpeed_1 < +TargetSpeed
                // devo accelerare: accel2use = +accel
                sPo_PostnerOut.flags.b.bInRamp = TRUE ;    // RAMP_POSITIVE
                sPostnerRun.psDemand->slAccel = +sPostnerRun.slProfileAcc ;
            }
            else
            {   // +DemandSpeed_1 = +TargetSpeed
                // sono alla velocita' richiesta: accel2use = 0
                sPo_PostnerOut.flags.b.bInRamp = FALSE ;   //RAMP_NO: Target Speed Reached
                sPostnerRun.psDemand->slAccel = 0 ;
            }
        }
        else
        {   // DemandSpeed_1 < 0  => -DemandSpeed_1 < +TargetSpeed 
            // devo decelerare: accel2use = +dec
            sPo_PostnerOut.flags.b.bInRamp = TRUE ;    // RAMP_POSITIVE
            sPostnerRun.psDemand->slAccel = +sPostnerRun.slDec2Use ;           
        }
    }
    else
    {   // TargetSpeed < 0
        if(sPostnerRun.psDemand->slSpeed < 0)
        {   // DemandSpeed_1 <0
            if (sPostnerRun.psDemand->slSpeed > sPostnerRun.slTgtSpd2Use)
            {   // -DemandSpeed_1 > -TargetSpeed
                // devo accelerare: accel2use = -accel
                sPo_PostnerOut.flags.b.bInRamp = TRUE ;    // RAMP_NEGATIVE
                sPostnerRun.psDemand->slAccel = -sPostnerRun.slProfileAcc ;
            }
            else if(sPostnerRun.psDemand->slSpeed < sPostnerRun.slTgtSpd2Use)
            {   // -DemandSpeed_1 < -TargetSpeed
                // devo decelerare: accel2use = +decel
                sPo_PostnerOut.flags.b.bInRamp = TRUE ;    // RAMP_POSITIVE
                sPostnerRun.psDemand->slAccel = +sPostnerRun.slDec2Use ;
            }
            else  
            {   // -DemandSpeed_1 = -TargetSpeed
                // sono alla velocita' richiesta: accel2use = 0
                sPo_PostnerOut.flags.b.bInRamp = FALSE ;   // RAMP_NO: Target Speed Reached
                sPostnerRun.psDemand->slAccel = 0 ;
            }
        }
        else
        {   // DemandSpeed_1 >= 0  => +DemandSpeed_1 > -TargetSpeed
            // devo decelerare: accel2use = -dec
            sPo_PostnerOut.flags.b.bInRamp = TRUE ;    // RAMP_NEGATIVE
            sPostnerRun.psDemand->slAccel = -sPostnerRun.slDec2Use ;            
        }
    }


    if (psPo_PostnerIn->flags.b.bPosition)
    {   // /////////////////////////////////// //
        // sono arrivato alla target position? //
        // il controllo deve essere fatto qui  //
        // perche' mi servono Demand.slAccel e //
        // Demand.slSpeed appena aggiornate    //
        // /////////////////////////////////// //
        if (sPo_PostnerOut.flags.b.bTgtPosReached)
        {  
            sPostnerRun.flags.b.bFlagSvaActive = FALSE ;  /* sono in posizione, resetto la flag SVA */
            sPostnerRun.flags.b.bEnableAntiWindupPosErr = FALSE ; /* disabilito l'antiwindup sul PosErrMax */
        }        
        else
        {   
            ulAbsValDmndAcc = labs(sPostnerRun.psDemand->slAccel) ;
            ulAbsValDmndSpd = labs(sPostnerRun.psDemand->slSpeed) ;
            ulAbsDltSpdEndVel32 = labs(sPostnerRun.psDemand->slSpeed - (sPostnerRun.flags.b.bPostnerNegative?-sPo_PostnerParam.slEndVelocity:sPo_PostnerParam.slEndVelocity));
            ulAbsDeltaSpd32 = labs(sPostnerRun.psDemand->slSpeed - sPostnerRun.slTgtSpd2Use) ;
            ulAbsDeltaPos32 = labs(_sint64toSint32(&sPostnerRun.sqDeltaPos)) ; /* riduco sqDeltaPos da 64bit a 32bit e ne faccio il valore assoluto */

            sPostnerRun.flags.b.bEnableAntiWindupPosErr = TRUE ; /* abilito l'antiwindup sul PosErrMax */
        
            if( ((ulAbsDeltaSpd32     < (ulAbsValDmndAcc >> 12)) && (ulAbsDeltaPos32 < ulAbsValDmndSpd)) ||
                ((ulAbsDltSpdEndVel32 < (ulAbsValDmndAcc >> 12)) && (ulAbsDeltaPos32 < ulAbsValDmndSpd)) || //se la tengo, con posizionamenti di 180° il motore va in fuga
                ((ulAbsDeltaSpd32 == 0) && (ulAbsDeltaPos32 == 0)) ) // il secondo controllo serve per posizionamenti sulla stessa posizione
            {   /* SE AbsDeltaSpeed < AbsValDmndAcc2Use/(2^12)         * 
                 * SE AbsDeltaPos < AbsDemandSpeed_1                   * 
                 * SONO ARRIVATO: impongo EndVelocity e TargetPosition */ 
                _sint64_atomic_copy(&sPostnerRun.psDemand->sqPostn, &sPostnerRun.sqTgtPos2Use) ;
                ulAbsValDmndSpd = sPostnerRun.psDemand->slSpeed;
                sPostnerRun.psDemand->slSpeed = sPostnerRun.flags.b.bPostnerNegative?-sPo_PostnerParam.slEndVelocity:sPo_PostnerParam.slEndVelocity ;
                sPostnerRun.psDemand->slAccel = _sint32_asl12(sPostnerRun.psDemand->slSpeed - (SLONG)ulAbsValDmndSpd) ;
                sPostnerRun.swDemandAccelCoins = 0 ; /* resetto gli spiccioli dell'accelerazione */
                sPostnerRun.slTgtSpd2Use = sPostnerRun.psDemand->slSpeed;
                sPo_PostnerOut.flags.b.bTgtPosReached = TRUE ; /* informo il mondo */
                sPostnerRun.flags.b.bEnableAntiWindupPosErr = FALSE ; /* disabilito l'antiwindup sul PosErrMax */
                sPostnerRun.flags.b.bDisSpeedUpdate = TRUE;
            }
        }   
    }
    else
    {   /* non sono in controllo di posizione:
           1) l'antiwindup sul PosErrMax e' sempre attivo;
           2) resetto la flag SVA  
        */
        sPostnerRun.flags.b.bEnableAntiWindupPosErr = TRUE ; 
        sPostnerRun.flags.b.bFlagSvaActive = FALSE ;
    }

    // //////////////////////////// //
    //  sono alla velocita' giusta? //
    // //////////////////////////// //
    // i valori assoluti vanno ricalcolati perche' se sono in modalita' posizionatore e sono arrivato in posizione, imposto accelerazione e velocita'
    ulAbsValDmndAcc = labs(sPostnerRun.psDemand->slAccel) ;
    ulAbsDeltaSpd32 = labs(sPostnerRun.psDemand->slSpeed - sPostnerRun.slTgtSpd2Use) ;

    if (ulAbsDeltaSpd32 < (ulAbsValDmndAcc >> 12))
    {   /* SE AbsDeltaSpeed < AbsValDmndAcc2Use/(2^12) * 
         * SONO ARRIVATO: impongo TargetSpeed          */
        ulAbsValDmndSpd = sPostnerRun.psDemand->slSpeed;
        sPostnerRun.psDemand->slSpeed = sPostnerRun.slTgtSpd2Use ;   
        sPostnerRun.psDemand->slAccel = _sint32_asl12(sPostnerRun.psDemand->slSpeed - (SLONG)ulAbsValDmndSpd) ;
        sPostnerRun.swDemandAccelCoins = 0 ; /* resetto gli spiccioli dell'accelerazione */
        sPo_PostnerOut.flags.b.bInRamp = FALSE ; /* forzo il valore..serve? */
        sPostnerRun.flags.b.bDisSpeedUpdate = TRUE;
    }

    PositionIntegrate8KHz();
}

/* ######################################################################### */
static void PositionIntegrate8KHz(void)
{
    SQWRD sqDemandPostn, sqPosErr_1, sqPosErrMax2Use, sqPosErrMin2Use ;
    SWORD swPosErrMax, swPosErrMin ;
    BOOL bBlocked = FALSE;
    BOOL bPosErrInLimitMax = FALSE ;
    BOOL bPosErrInLimitMin = FALSE ;

    if( !sPostnerRun.flags.b.bPosIntegrateNoAccel && !sPostnerRun.flags.b.bDisSpeedUpdate)
    {
        // //////////////// //
        // calcolo le rampe //
        // //////////////// //    
        /* Non voglio usare matematica 6 64bit! quindi clippo il riferimento a 2^30 (= 120000 rpm) per evitare overflow sommando l'accelerazione */
        if (sPostnerRun.psDemand->slSpeed > TWO_POWER_THIRTY)
            sPostnerRun.psDemand->slSpeed = TWO_POWER_THIRTY ;
        else if (sPostnerRun.psDemand->slSpeed < -TWO_POWER_THIRTY)
            sPostnerRun.psDemand->slSpeed = -TWO_POWER_THIRTY ;
        
        /* calcolo la nuova DemandSpeed 'rampando' */
        sPostnerRun.swDemandAccelCoins += (SWORD)(sPostnerRun.psDemand->slAccel & 0x00000fff) ; /* per gestire accelerazioni più lente */
        sPostnerRun.psDemand->slSpeed  = sPostnerRun.psDemand->slSpeed + (sPostnerRun.psDemand->slAccel >> 12) + (SLONG)(sPostnerRun.swDemandAccelCoins >> 12) ; /* DemandSpeed_1 + Acceleration + resto */
        sPostnerRun.swDemandAccelCoins &= 0x0fff ;
    }
    sPostnerRun.flags.b.bDisSpeedUpdate = FALSE;

    _sint64_add_64_32(&sqDemandPostn, &sPostnerRun.psDemand->sqPostn, &sPostnerRun.psDemand->slSpeed) ; /* DemandPostn = DemandPostn_1 + DemandSpeed */

    /* salvo l'errore di posizione al campione precedente */
    _sint64_atomic_copy(&sqPosErr_1, sPostnerRun.psqPosErr) ;

    /* calcolo l'errore di posizione aggiornato: Error(64) = DemandPosition(64) - FeedbackPosition(64) */
    _sint64_sub(sPostnerRun.psqPosErr, &sqDemandPostn, &psPo_InFeedBack->sqPostn) ;      
 
    if (sPostnerRun.flags.b.bEnableAntiWindupPosErr)
    {   /* limito l'errore solo quando non sono in controllo di posizione */   
         if ((psPo_ILimitActive->b.bIqMax || psPo_ILimitActive->b.bIqMin) && (!sPo_PlcWorks.flags.b.bRampInSaturation))
         {  /* sono in limite di corrente: decido quali limiti usare */
            if (sqPosErr_1.hi & 0x80000000)
            {   /* l'errore e' negativo: impongo che il limite negativo sia uguale al valore dell'errore al campione precedente */
                _sint64_atomic_copy(&sqPosErrMax2Use, &sPostnerRun.sqPstnErrMax) ;
                _sint64_atomic_copy(&sqPosErrMin2Use, &sqPosErr_1) ;
            }
            else
            {   /* l'errore e' positivo: impongo che il limite positivo sia uguale al valore dell'errore al campione precedente */
                _sint64_atomic_copy(&sqPosErrMax2Use, &sqPosErr_1) ;
                _sint64_atomic_copy(&sqPosErrMin2Use, &sPostnerRun.sqPstnErrMin) ;
            }
            bBlocked = TRUE;
         }
         else
         {  /* NON sono in limite di corrente: uso i valori utente */
            _sint64_atomic_copy(&sqPosErrMax2Use, &sPostnerRun.sqPstnErrMax) ;
            _sint64_atomic_copy(&sqPosErrMin2Use, &sPostnerRun.sqPstnErrMin) ;
         }

        swPosErrMax = _sint64_comp(sPostnerRun.psqPosErr, &sqPosErrMax2Use) ; 
        swPosErrMin = _sint64_comp(sPostnerRun.psqPosErr, &sqPosErrMin2Use) ; 
        
        // PosErr > PosErrMax => +1; PosErr = PosErrMax => 0; PosErr < PosErrErrMax => -1
        // PosErr > PosErrMin => +1; PosErr = PosErrMin => 0; PosErr < PosErrErrMin => -1   
        if ((swPosErrMax == 1) && (swPosErrMin != -1))
        {
           _sint64_atomic_copy(sPostnerRun.psqPosErr, &sqPosErrMax2Use) ; /* Limit to max position error */
            bPosErrInLimitMax = TRUE ;
        }
        else if ((swPosErrMax != 1) && (swPosErrMin == -1))
        {
           _sint64_atomic_copy(sPostnerRun.psqPosErr, &sqPosErrMin2Use) ; /* Limit to min position error */
            bPosErrInLimitMin = TRUE ;
        }
    }
    else
    {
        // se zero disabilito controllo
        if(_sint64_comp(&sqZero64bit, &sPo_PostnerParam.sqPstnErrMax))
        {
            _sint64_sub(&sqPosErrMin2Use, &sqZero64bit, &sPo_PostnerParam.sqPstnErrMax) ;
    
            // se sono in errore di inseguimento
            if(_sint64_comp(sPostnerRun.psqPosErr, &sqPosErrMin2Use) < 0 ||
               _sint64_comp(sPostnerRun.psqPosErr, &sPo_PostnerParam.sqPstnErrMax) > 0)
               bBlocked = TRUE;
        }
    }

    // motor blocked timeout management
    if(bBlocked)
    {
        if(sPo_PostnerParam.swMotBlockTout != sPostnerRun.swMotBlockTout)
        {   // update timer when user changes the timeout and the motor is already blocked, to avoid spurious fault
            sPostnerRun.swMotBlockedTimer = timer_settimeout(uwSysTimers1ms, sPo_PostnerParam.swMotBlockTout) ;
            sPostnerRun.swMotBlockTout = sPo_PostnerParam.swMotBlockTout ;
        }

        if(sPo_PostnerParam.swMotBlockTout && timer_istimedout(uwSysTimers1ms, sPostnerRun.swMotBlockedTimer) &&
           !sPostnerRun.flags.b.bErrSentMotorBlocked)
        {
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_MOTOR_FAIL, SYSTEMALARMS_SUBCODE_MT_MOTORBLOCKED, TRUE) ;
            (*sPo_Handlers.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
            sPostnerRun.flags.b.bErrSentMotorBlocked = TRUE ;
        }
    }
    else
    {
        sPostnerRun.swMotBlockedTimer = timer_settimeout(uwSysTimers1ms, sPo_PostnerParam.swMotBlockTout) ;
        sPostnerRun.swMotBlockTout = sPo_PostnerParam.swMotBlockTout ;
    }

    _sint64_add(&sPostnerRun.psDemand->sqPostn, sPostnerRun.psqPosErr, &psPo_InFeedBack->sqPostn) ; /* 'limited' DemandPostn */

    if (sPo_PlcWorks.flags.b.bRampInSaturation)
    {
        if (psPo_PostnerIn->flags.b.bVelocity)    
        {   /* calcolo la Demand Speed nel caso in cui voglio ripartire a rampa usando
               la velocita' encoder (SOLO QUANDO SONO IN CONTROLLO DI VELOCITA'):
               quando sono in limite di corrente: ricopio la velocita' attuale 
               dell'encoder sul riferimento di velocita' per eseguire la rampa */
            if (psPo_ILimitActive->b.bIqMax)
            {   
                if (sPostnerRun.psDemand->slSpeed > psPo_InFeedbackExt->slFilteredSpeed)
                    sPostnerRun.psDemand->slSpeed = psPo_InFeedbackExt->slFilteredSpeed ;
            }
            else if (psPo_ILimitActive->b.bIqMin)
            {
                if (sPostnerRun.psDemand->slSpeed < psPo_InFeedbackExt->slFilteredSpeed)
                    sPostnerRun.psDemand->slSpeed = psPo_InFeedbackExt->slFilteredSpeed ;
            }
        }
        else
        {   /* quando NON sono in controllo di velocita' e sono in limite di errore di posizione
               imposto che la velocita' di riferimento sia quella dell'encoder */
            if (bPosErrInLimitMax)
            {   
                if (sPostnerRun.psDemand->slSpeed > psPo_InFeedBack->slSpeed)
                    sPostnerRun.psDemand->slSpeed = psPo_InFeedBack->slSpeed ;
            }
            else if (bPosErrInLimitMin)
            {
                if (sPostnerRun.psDemand->slSpeed < psPo_InFeedBack->slSpeed)
                    sPostnerRun.psDemand->slSpeed = psPo_InFeedBack->slSpeed ;
            }
        }
    }
    //TEMPFLAG(13);
    // SRamp
    if(sPo_PostnerOut.flags.b.bSRampEnabled)
        SRamp8kHz() ;         //路径规划
}

/* ######################################################################### */
static void Interpolation8KHz(void)
{
    SQWRD sqDelta;
    SLONG slDelta,slNewAcc;

        // if new IP quota
    if(psPo_PostnerIn->flags.b.bNewTarget)
    {
            // delta between actual ip quota (demand pos) and next quota to be reached
        _sint64_sub(&sqDelta, &sPostnerRun.sqTgtPos2Use, &sPostnerRun.psDemand->sqPostn);
    
            // limit to 32bit
        slDelta=_sint64toSint32(&sqDelta);
    
            // apply multiplication timing factor
        _sint64_mul_32_32(&sqDelta, &slDelta, &psPo_PostnerIn->slIPScaling);

            // center part is new speed
        _sint64toSint48(&sqDelta);
        slDelta=INT64_MLONG(sqDelta);

            // calc acceleration
        slNewAcc=labs(slDelta-sPostnerRun.psDemand->slSpeed);

            // check for saturation scaling and apply acceleration
        if(slNewAcc&0xfff00000)
            slNewAcc=0x7fffffff;
        else
            slNewAcc=_sint32_asl12(slNewAcc);

            // recalc sign
        if(slDelta>=sPostnerRun.psDemand->slSpeed)
            sPostnerRun.psDemand->slAccel= slNewAcc;
        else
            sPostnerRun.psDemand->slAccel=-slNewAcc;
    }
    else
            // no new quota, constant speed, zero acceleration
        sPostnerRun.psDemand->slAccel=0l;
                 
    PositionIntegrate8KHz() ;
}

/* ######################################################################### */
static void SpeedNoRamp8KHz(void)
{
    sPostnerRun.psDemand->slSpeed = sPostnerRun.slTgtSpd2Use ;   

    if(psPo_PostnerIn->flags.b.bUseLocalAccSpd)
        sPostnerRun.psDemand->slAccel = psPo_PostnerIn->slLocalAcc ;
    else
        sPostnerRun.psDemand->slAccel = sPo_PostnerParam.slProfileAcc ;

    sPo_PostnerOut.flags.b.bInRamp = FALSE ;

    PositionIntegrate8KHz();
}

/* ######################################################################### */
static void Po_PositionerBkGd(void)
{
  SQWRD sqPstnErrMax, sqPstnErrMin ;

  // Position error limit
  _sint64_atomic_copy(&sqPstnErrMax, &sPo_PostnerParam.sqPstnErrMax) ;

  // If zero then take the automatic ones calculated in ss control loop
  if( _sint64_comp(&sqPstnErrMax, &sqZero64bit) == 0 )
  {
    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;
    _sint64_atomic_copy(&sPostnerRun.sqPstnErrMax, &psPo_InSSCtrlLoop->sqFitPErrMax) ;
    _sint64_atomic_copy(&sPostnerRun.sqPstnErrMin, &psPo_InSSCtrlLoop->sqFitPErrMin) ;
    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;
  }
  else
  {
    _sint64_sub(&sqPstnErrMin, &sqZero64bit, &sqPstnErrMax) ;
    
    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;
    _sint64_atomic_copy(&sPostnerRun.sqPstnErrMax, &sqPstnErrMax) ;
    _sint64_atomic_copy(&sPostnerRun.sqPstnErrMin, &sqPstnErrMin) ;
    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;
  }

  // Acceleration and deceleration 
  if (!sPo_PlcWorks.flags.b.bRampFastChange)
    CalculateAccDec() ;

  // SRamp
  if (sPo_PostnerOut.flags.b.bSRampEnabled)
    SRampBkGd() ;

}


/* ######################################################################### */    
static BOOL Po_PositionerRst(void)
{
    _sint64_atomic_copy(sPostnerRun.psqPosErr, &sqZero64bit) ;
    sPo_PostnerOut.flags.b.bTgtPosReached = TRUE ; // default: posizione gia' raggiunta
    sPo_PostnerOut.flags.b.bInRamp = FALSE ;

    /* ricopio i dati dal sensore di posizione (per tenere fermo l'anello di controllo) */
    _sint64_atomic_copy(&sPostnerRun.psDemand->sqPostn, &psPo_InFeedBack->sqPostn) ; 
    sPostnerRun.psDemand->slAccel = psPo_InFeedBack->slAccel ;

    /* se la velocità encoder e' sotto la soglia, la spiano a zero */
    if (labs(psPo_InFeedBack->slSpeed) > sPo_PostnerParam.slZeroSpeedThreshold)
        sPostnerRun.psDemand->slSpeed = psPo_InFeedBack->slSpeed ;        
    else
        sPostnerRun.psDemand->slSpeed = 0L ;

    /* aggiorno le variabili di runtime */
    sPostnerRun.flags.b.bFlagSvaActive = FALSE ;
    sPostnerRun.uwSvaCounter = 0 ;
    sPostnerRun.swDemandAccelCoins  = 0 ; /* resetto gli spiccioli dell'accelerazione */

    sPostnerRun.slTgtSpd2Use = sPostnerRun.psDemand->slSpeed ;
    _sint64_atomic_copy(&sPostnerRun.sqTgtPos2Use, &sPostnerRun.psDemand->sqPostn) ;   
    INT64_ASSIGN (sPostnerRun.sqDeltaPos, 0L, 0UL) ;    

    sPostnerRun.swMotBlockedTimer = timer_settimeout(uwSysTimers1ms, sPo_PostnerParam.swMotBlockTout) ;
    sPostnerRun.swMotBlockTout = sPo_PostnerParam.swMotBlockTout ;

    SRampReset() ;
    
  return TRUE ; 
}


/* ######################################################################### */
static void CalculateAccDec(void)
{
  SQWRD sqDecLimit ;
  SWORD swDecMultiplier = FIVE ; // needed by lx 32x16
  SLONG slDec2Use, slDecLimit ;

  if(psPo_PostnerIn->flags.b.bUseLocalAccSpd)
  {
    atomic_move(&sPostnerRun.slProfileAcc, &psPo_PostnerIn->slLocalAcc, sizeof(psPo_PostnerIn->slLocalAcc)) ;    
    atomic_read(&slDec2Use, &psPo_PostnerIn->slLocalDec, sizeof(psPo_PostnerIn->slLocalDec)) ; 
  }
  else
  {
    atomic_move(&sPostnerRun.slProfileAcc, &sPo_PostnerParam.slProfileAcc, sizeof(sPo_PostnerParam.slProfileAcc)) ;    
    atomic_read(&slDec2Use, &sPo_PostnerParam.slProfileDec, sizeof(sPo_PostnerParam.slProfileDec)) ; 
  }
    
  if (sPostnerRun.slProfileDec != slDec2Use)
  {
    /* calcolo il limite sulla decelerazione */
    slDecLimit = slDec2Use + 100 ; // somma necessaria per la mul_32_16
    _sint64_mul_32_16(&sqDecLimit, &slDecLimit, &swDecMultiplier) ;
    _sint64_shr(&sqDecLimit, 2) ;
    slDecLimit = _sint64toSint32(&sqDecLimit) ; /* slDecelerationLimit = ((slProfileDec+100) * 5)/4 */ 
    
    atomic_write(&sPostnerRun.slProfileDec, &slDec2Use, sizeof(slDec2Use)) ;        
    atomic_write(&sPostnerRun.slProfileDecLimit, &slDecLimit, sizeof(slDecLimit)) ;
  }
}


/* ######################################################################### */    
static void SRamp8kHz(void)
{
    SWORD swPosErrMax, swPosErrMin ;
    SQWRD sqTemp64, sqFilterIn, sqSpd48, sqPostnUnlimited64 ;
    SLONG slRGDeltaPos_1 ;
    BOOL bPosErrSRampInLimitMax = FALSE ;
    BOOL bPosErrSRampInLimitMin = FALSE ;
    SQWRD sqTemp64_1;

    // calcolo la differenza tra la traiettoria del generatore di rampe lineari e la traiettoria del generatore di rampe S
    _sint64_sub_nosaturation(&sqTemp64, &sPo_PostnerOut.sLocDemand.sqPostn, &sPo_PostnerOut.sDemand.sqPostn) ;

    _sint64_atomic_copy(&sqTemp64_1,&sqTemp64);
    _sint64_shr(&sqTemp64_1,8);

    slRGDeltaPos_1 = sPostnerRun.sSRamp.slRGDeltaPos ;
    sPostnerRun.sSRamp.slRGDeltaPos = (SLONG)sqTemp64_1.lo ;

    if (!psPo_PostnerIn->flags.b.bDirectSpeed && !sPo_PostnerOut.flags.b.bInRamp && 
        ((sPostnerRun.sSRamp.slRGDeltaPos > 0) != (slRGDeltaPos_1 > 0)))
    {   // non sono in rampa lineare ED il segno del delta di posizione tra rampa lineare e rampa S 
        // e' cambiato: solo una volta ricopio DemandPos su SRampPos e resetto i vari stati integrali
        INT64_ASSIGN(sqTemp64, 0L, 0UL) ;
        INT64_ASSIGN(sPostnerRun.sSRamp.sqFilterStatus64, 0L, 0UL) ;
        INT64_ASSIGN(sPostnerRun.sSRamp.sqIntegralStatus64Spd, 0L, 0UL) ;

        _sint64_atomic_copy(&sPo_PostnerOut.sDemand.sqPostn, &sPo_PostnerOut.sLocDemand.sqPostn) ; 
    }

    _sint64_sub_nosaturation(&sqFilterIn, &sqTemp64, &sPostnerRun.sSRamp.sqFilterStatus64) ;

    /* calcolo la 'S' demand acceleration  */ 
    _sint64_atomic_copy(&sqTemp64, &sqFilterIn) ; 
    if (sPostnerRun.sSRamp.sw1_K1 > 0)
    {   // shift positivo = shift left
        _sint64_shl(&sqTemp64, (UWORD)sPostnerRun.sSRamp.sw1_K1) ;
    }
    else if (sPostnerRun.sSRamp.sw1_K1 < 0)
    {   // shift negativo = shift right
        _sint64_shr(&sqTemp64, (UWORD)(-sPostnerRun.sSRamp.sw1_K1)) ;
    }
    sPo_PostnerOut.sDemand.slAccel = _sint64toSint32(&sqTemp64) ;

    /* calcolo la 'S' demand speed */ 
    _sint64_add(&sqTemp64, &sqFilterIn, &sPostnerRun.sSRamp.sqIntegralStatus64Spd) ;
    _sint64_atomic_copy(&sPostnerRun.sSRamp.sqIntegralStatus64Spd, &sqTemp64) ;
    _sint64_atomic_copy(&sqSpd48, &sqTemp64) ; /* mi serve per calcolare la regresessione dello stato del filtro */
    _sint64_shr(&sqTemp64, sPostnerRun.sSRamp.uwAbsK1) ;
    sPo_PostnerOut.sDemand.slSpeed = _sint64toSint32(&sqTemp64) ;

    /* calcolo la 'S' demand position: sqPostn64 = sqPostnOut64_1 + Speed32 */
    _sint64_add_64_32(&sqPostnUnlimited64, &sPo_PostnerOut.sDemand.sqPostn, &sPo_PostnerOut.sDemand.slSpeed) ;

    //TEMPFLAG(14);

    /* calcolo l'errore di posizione aggiornato: Error(64) = SRampUnlimitedDemandPosition(64) - FeedbackPosition(64) */
    _sint64_sub(&sPo_PostnerOut.sqPosErr, &sqPostnUnlimited64, &psPo_InFeedBack->sqPostn) ; 

    if (sPostnerRun.flags.b.bEnableAntiWindupPosErr)
    {   /* limito l'errore al valore utente */
        swPosErrMax = _sint64_comp(&sPo_PostnerOut.sqPosErr, &sPostnerRun.sqSRampPstnErrMax) ; 
        swPosErrMin = _sint64_comp(&sPo_PostnerOut.sqPosErr, &sPostnerRun.sqSRampPstnErrMin) ; 
        
        // PosErr > PosErrMax => +1; PosErr = PosErrMax => 0; PosErr < PosErrErrMax => -1
        // PosErr > PosErrMin => +1; PosErr = PosErrMin => 0; PosErr < PosErrErrMin => -1   
        if ((swPosErrMax == 1) && (swPosErrMin != -1))
        {  /* Limit to max position error */
           _sint64_atomic_copy(&sPo_PostnerOut.sqPosErr, &sPostnerRun.sqSRampPstnErrMax) ; 
           bPosErrSRampInLimitMax = TRUE ;
        }
        else if ((swPosErrMax != 1) && (swPosErrMin == -1))
        {  /* Limit to min position error */
           _sint64_atomic_copy(&sPo_PostnerOut.sqPosErr, &sPostnerRun.sqSRampPstnErrMin) ; 
           bPosErrSRampInLimitMin = TRUE ;
        }
    }

    /* 'limited' DemandPostn  = regressione dello stato dell'integrale */
    _sint64_add(&sPo_PostnerOut.sDemand.sqPostn, &sPo_PostnerOut.sqPosErr, &psPo_InFeedBack->sqPostn) ; 


    /* se e' attiva la flag di saturazione delle rampe e se sono in limite di PosErr (Max o Min),  
       ricopio la velocita' dell'encoder sul reference della rampa lineare */   
    if (sPo_PlcWorks.flags.b.bSRampInSaturation)
    {   
        if (bPosErrSRampInLimitMax)
        {   
            if (sPo_PostnerOut.sDemand.slSpeed > psPo_InFeedBack->slSpeed)
                sPo_PostnerOut.sDemand.slSpeed = psPo_InFeedBack->slSpeed ;
            if (sPo_PostnerOut.sLocDemand.slSpeed > psPo_InFeedBack->slSpeed)
                sPo_PostnerOut.sLocDemand.slSpeed = psPo_InFeedBack->slSpeed ;
        }
        else if (bPosErrSRampInLimitMin)
        {
            if (sPo_PostnerOut.sDemand.slSpeed < psPo_InFeedBack->slSpeed)
                sPo_PostnerOut.sDemand.slSpeed = psPo_InFeedBack->slSpeed ;
            if (sPo_PostnerOut.sLocDemand.slSpeed < psPo_InFeedBack->slSpeed)
                sPo_PostnerOut.sLocDemand.slSpeed = psPo_InFeedBack->slSpeed ;
        }
    }

    /* calcolo la regressione dello stato del filtro */
    _sint64toSint48(&sqSpd48) ;
    _sint64_mul_48_16(&sPostnerRun.sSRamp.sqFilterStatus64, &sqSpd48, &sPostnerRun.sSRamp.swK2) ;
    _sint64_shr(&sPostnerRun.sSRamp.sqFilterStatus64, sPostnerRun.sSRamp.uwAbsK1) ;
    if (sPo_PlcWorks.flags.b.bSRampInSaturation)
    {
        if ((psPo_ILimitActive->b.bIqMax || psPo_ILimitActive->b.bIqMin) && (sPo_PostnerOut.sqPosErr.hi & 0x80000000))
        {             
            _sint64_add(&sPostnerRun.psDemand->sqPostn, &sSS_CntrLoopOut.sqFitPErrMin, &psPo_InFeedBack->sqPostn) ;  /* Position error is set to AutoPosErr*/
            _sint64_add(&sPo_PostnerOut.sDemand.sqPostn, &sSS_CntrLoopOut.sqFitPErrMin, &psPo_InFeedBack->sqPostn) ;  /* Position error is set to AutoPosErr*/
        }
        else if (psPo_ILimitActive->b.bIqMax || psPo_ILimitActive->b.bIqMin)
        { 
            _sint64_add(&sPostnerRun.psDemand->sqPostn, &sSS_CntrLoopOut.sqFitPErrMax, &psPo_InFeedBack->sqPostn) ;  /* Position error is set to AutoPosErr*/
            _sint64_add(&sPo_PostnerOut.sDemand.sqPostn, &sSS_CntrLoopOut.sqFitPErrMax, &psPo_InFeedBack->sqPostn) ;  /* Position error is set to AutoPosErr*/
        }    
        else
        {
            _sint64_atomic_copy(&sPostnerRun.psDemand->sqPostn, &psPo_InFeedBack->sqPostn) ; 
            _sint64_atomic_copy(&sPo_PostnerOut.sDemand.sqPostn, &psPo_InFeedBack->sqPostn) ; 
  
        } 
               
        INT64_ASSIGN(sPostnerRun.sSRamp.sqFilterStatus64, 0L, 0UL) ;
    //    INT64_ASSIGN(sPostnerRun.sSRamp.sqIntegralStatus64Pos, 0L, 0UL) ;
        INT64_ASSIGN(sPostnerRun.sSRamp.sqIntegralStatus64Spd, 0L, 0UL) ;
        sPostnerRun.sSRamp.slRGDeltaPos = 0 ;
    }
}

/* ######################################################################### */    
static void SRampReset(void)
{
    /* ricopio i dati dal sensore di posizione (per tenere fermo l'anello di controllo) */
    _sint64_atomic_copy(&sPo_PostnerOut.sDemand.sqPostn, &psPo_InFeedBack->sqPostn) ; 
    sPo_PostnerOut.sDemand.slAccel = psPo_InFeedBack->slAccel ;

    /* se la velocità encoder e' sotto la soglia, la spiano a zero */
    if (labs(psPo_InFeedBack->slSpeed) > sPo_PostnerParam.slZeroSpeedThreshold)
        sPo_PostnerOut.sDemand.slSpeed = psPo_InFeedBack->slSpeed ;        
    else
        sPo_PostnerOut.sDemand.slSpeed = 0L ;

    INT64_ASSIGN(sPostnerRun.sSRamp.sqFilterStatus64, 0L, 0UL) ;
//    INT64_ASSIGN(sPostnerRun.sSRamp.sqIntegralStatus64Pos, 0L, 0UL) ;
    INT64_ASSIGN(sPostnerRun.sSRamp.sqIntegralStatus64Spd, 0L, 0UL) ;
    sPostnerRun.sSRamp.slRGDeltaPos = 0 ;

    if(sPo_PostnerOut.flags.b.bSRampEnabled)
    {   // otherwise Position error is not cleared when drive is disabled
        INT64_ASSIGN(sPo_PostnerOut.sqPosErr, 0L, 0UL) ;
    }
}


/* ######################################################################### */    
static void SRampBkGd(void)
{
  SQWRD  sqPstnErrMax, sqPstnErrMin ;

  sPostnerRun.sSRamp.swK1 = sPo_PostnerParam.swSRampK1 ;
  sPostnerRun.sSRamp.swK2 = sPo_PostnerParam.swSRampK2 ;
  sPostnerRun.sSRamp.sw1_K1 = 12 + sPo_PostnerParam.swSRampK1 ;
  sPostnerRun.sSRamp.uwAbsK1 = abs(sPostnerRun.sSRamp.swK1) ;

  // Position error limit
  _sint64_atomic_copy(&sqPstnErrMax, &sPo_PostnerParam.sqSRampPstnErrMax) ;

  // If zero then take the automatic ones calculated in ss control loop
  if( _sint64_comp(&sqPstnErrMax, &sqZero64bit) == 0 )
  {
    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;
    _sint64_atomic_copy(&sPostnerRun.sqSRampPstnErrMax, &psPo_InSSCtrlLoop->sqFitPErrMax) ;
    _sint64_atomic_copy(&sPostnerRun.sqSRampPstnErrMin, &psPo_InSSCtrlLoop->sqFitPErrMin) ;
    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;
  }
  else
  {
    _sint64_sub(&sqPstnErrMin, &sqZero64bit, &sqPstnErrMax) ;
    
    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;
    _sint64_atomic_copy(&sPostnerRun.sqSRampPstnErrMax, &sqPstnErrMax) ;
    _sint64_atomic_copy(&sPostnerRun.sqSRampPstnErrMin, &sqPstnErrMin) ;
    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;
  }
}
