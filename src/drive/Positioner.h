/* ************************************************************************ * 
 * Project: AxM-E Control Board                                             * 
 *                                                                          * 
 * Copyright © 2005,2008, Phase Motion Control. All Rights Reserved.        *
 *                                                                          * 
 * Version     : 1.0                                                        * 
 * File        : Positioner.h                                               * 
 * Author      : Cristiano Tognetti                                         * 
 *                                                                          * 
 * Description :                                                            * 
 *                                                                          * 
 * ************************************************************************ */
#ifndef _PO_H
 #define _PO_H

#include "common\GlobalStructures.h" /* to catch GLB_KINEMATIC_DATA typedef */
#include "drive\SpaceSpeedCntrLp.h"
#include "drive\EncoderManager.h"

/* ============================== structures =============================== */
typedef struct {
  /* Flags */
  union {
    struct {
        BOOL bQuickStop;
        BOOL bVelocity ;
        BOOL bPosition ;
        BOOL bNewTarget; // TRUE = nuovo target; FALSE = nessun nuovo target
        BOOL bPostnerEnable; // TRUE = abilita il modulo posizionatore
        BOOL bInterpolate  ; // enable interpolated mode (obsolete, keep'd for compatibility)
        BOOL bUseLocalAccSpd; // use local accel/decel/speed instead of parameters/works
        BOOL bDirectSpeed;    // ignore accel/decel when in speed mode, e.g. immediate apply of tgt speed
      } b ;
      UWORD w[4] ;
/*      struct {
        UWORD bQuickStop : 1 ;
        UWORD bVelocity  : 1 ;
        UWORD bPosition  : 1 ;
        UWORD bNewTarget : 1 ; //  TRUE = nuovo target; FALSE = nessun nuovo target
        UWORD bPostnerEnable: 1 ; // TRUE = abilita il modulo posizionatore
        UWORD bInterpolate:   1 ; // enable interpolated mode (obsolete, keep'd for compatibility)
        UWORD bUseLocalAccSpd: 1 ; // use local accel/decel/speed instead of parameters/works
        UWORD bDirectSpeed: 1 ;    // ignore accel/decel when in speed mode, e.g. immediate apply of tgt speed
      } b ;
      UWORD w ; */
  } flags ;
 
  SQWRD sqTargetPostn ; /* TargetPosition (in modalita' posizionatore) */
  SLONG slTargetSpeed ; /* TargetVelocity (in modalita velocita') */
  SLONG slQuickStopDec2Use ; /* value of quickstop deceleration to use */
  SLONG slIPScaling ;   /* interpolation scaling for speed calc (obsolete, keep'd for compatibility) */
                        /* 0x00010000 = ip time 125us, 0x00008000 = ip time 250us */
  SLONG slLocalAcc;
  SLONG slLocalDec;
  SLONG slLocalSpeed ;

} PO_POSITIONER_IN ;

typedef struct
{
    const COMCTRL_HANDLERS *psCtrlHandlers ;
} PO_POSITIONER_HDL ;

typedef struct {
  /* Flags */
  union {
      struct {
        BOOL bDummy0[3]    ;
        BOOL bTgtPosReached; /* 1 = target position raggiunta; 0 = target position NON raggiunta */
        BOOL bDummy1       ;
        BOOL bInRamp       ; /* 1 = in rampa; 0 = non in rampa*/
        BOOL bSRampEnabled ; 
        BOOL bDummy2       ;
      } b ;
      UWORD w[4];
  } flags ;     // Flags status modulo sw posizionatore

  GLB_KINEMATIC_DATA sDemand ; /* Demand Position, Demand Speed, Demand Acceleration */

  SQWRD  sqPosErr ; // position error measured (64bit)

  GLB_KINEMATIC_DATA sLocDemand ; /* demands before S-Ramp if enabled, nothing if disabled */
} PO_POSITIONER_OUT ;

typedef struct {
  SLONG slEndVelocity  ; /* End Speed (in modalita' posizionatore) */
  SLONG slProfileVel   ; /* Profile Velocity */
  SLONG slProfileAcc   ; /* Profile Acceleration */
  SLONG slProfileDec   ; /* Profile Deceleration */  
  SLONG slQuickStopDec ; /* Decelerazione per il QuickStop */ 
  SQWRD sqPstnErrMax   ; /* Max Position Error allowed */
  SLONG slZeroSpeedThreshold ; /* Threshold to consider speed = zero */
  SWORD swMotBlockTout ; /* Motor blocked timeout [msec] */
  union {
      struct {
        BOOL bSRampEnable;//UWORD bSRampEnable: 1; // enable SRamp fw module
      } b ;
      //UWORD w ;
#ifdef _INFINEON
  }                             flags ;
#else
  } __attribute__((aligned(4))) flags ;
#endif
  SWORD swSRampK1 ;
  SWORD swSRampK2 ;
  ULONG ulDummy ;
  SQWRD sqSRampPstnErrMax ;
} PO_POSITIONER_PARAMS ;

typedef struct {
  /* Flags */
  union {
      struct {
        BOOL bRampInSaturation ;
        BOOL bRampFastChange; // change accel/decel ramp in fast task
        BOOL bSRampInSaturation;
        BOOL bDummy;
      } b ;
      UWORD w[2] ;
  } flags ;
} PO_PLC_WORKS ;
  
/* =========================== global  variables =========================== */

extern PO_POSITIONER_IN     sPo_UsrPostnerIn ; // -- input from usr (default)
extern PO_POSITIONER_IN     sPo_PlcPostnerIn ; // -- input from plc

// ------ positioner module
extern PO_POSITIONER_IN    *psPo_PostnerIn  ;
extern PO_POSITIONER_HDL    sPo_Handlers    ;
extern GLB_KINEMATIC_DATA  *psPo_InFeedBack ;
extern SS_OUTPUT           *psPo_InSSCtrlLoop ;
extern MH_CURLIMITFLAGS    *psPo_ILimitActive ;
extern ENCMGR_SPACEFB_EXT  *psPo_InFeedbackExt ; 
extern PO_POSITIONER_OUT    sPo_PostnerOut ;
extern PO_POSITIONER_PARAMS sPo_PostnerParam ;
#ifdef _INFINEON_
extern const PO_POSITIONER_PARAMS huge sPo_PostnerDefParam ;
#else
extern const PO_POSITIONER_PARAMS sPo_PostnerDefParam ;
#endif // _infineon_
extern PO_PLC_WORKS         sPo_PlcWorks ;

/* =============================== functions =============================== */
BOOL Po_PositionerInit(void) ;

/* EOF */
#endif /* PO_H */

 
