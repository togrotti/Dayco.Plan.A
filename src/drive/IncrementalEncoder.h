/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2010, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : Incremental.h                                              */
/* Author      : Cristiano Tognetti                                         */
/*               Fabio Terrile                                              */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _IC_H
 #define _IC_H

//***************************************************************************
// Defines

#define INCREMENTAL_SEL_MAIN              0
#define INCREMENTAL_SEL_AUX               1

#define INCREMENTAL_MAX_ENCODER           2

//***************************************************************************
// Output data structures

typedef struct
{
    ULONG ulCounter;
    ULONG ulIndexCapture;
    SWORD swSin;
    SWORD swCos;
    ULONG ulSinCosLevel;
    UBYTE ubIndexCapCounter;
} INCREMENTAL_OUT;

typedef struct
{
    SBYTE sbOutEnabled;
    SBYTE sbReqCmd;
    SQWRD sqPostn;
} AUXSIM_OUT;

/* =============================== structure =============================== */  
typedef struct {
 /* Flags */
  union {
      struct {
        BOOL bSinCosInterpolation   ; // 0 => DO NOT USE SinCosInterpolation; 1 => USE SinCosInterpolation (Incremental)
        BOOL bIndexErrorDisabled    ; // 0 => index error check enabled; 1 => index error check disabled
        BOOL bSin2Cos2ErrorDisabled ; // 0 => sin^2 + cos^2 error check enabled; 1 => sin^2 + cos^2 error check disabled
        BOOL bHasIndex              ; // 1 => encoder has valid index track
        BOOL bSwapTracks            ; // 1 => swap input tracks A and B
        BOOL bEnableStepDir         ; // 1 => enable step pulse (track A) and direction (track B) mode
        BOOL bEnableUpDown          ; // 1 => enable up pulse (track A) and down pulse (track B) mode
        BOOL bDisFactoryCalibr      ; // 1 => take default calibration values rather than the stored ones
        BOOL bDisIdxEAngle          ; // 1 => disable index track for electrical angle adjustment
        BOOL bDisIdxPosition        ; // 1 => disable index track for mechanical position adjustment
        BOOL bDisIdxABSync          ; // 1 => un-sync index track from AB tracks
        BOOL bDisDigInterpolation   ; // 1 => disable digital interpolation with periodimeter
        BOOL bWdtFiltErrorDisabled  ; // 1 => disable digital filter watchdog fault
        BOOL bCaptureAllIndexes     ; // 1 => enable capturing of all indexes, unregarding index tolerance
      } b ;
    //   UWORD w[7] ;
#ifdef _INFINEON
  }                             flags ;
#else
  } __attribute__((aligned(4))) flags ;
#endif
  
      // Encoder Poles Number
  UWORD uwEncPoleNumber ;
      // Encoder Line Counts
  ULONG ulLineCounts ;
      // Resolver/Sincos Alarm
  UWORD uwSinCosAlarmThreshold ;
      // Index Tolerance
  UWORD uwIndexTolerance ;
      // Max input frequency allowed (referred to single line) [kHz]
  UWORD uwMaxFrequency ;
      // analog inputs level gain (1000 = gain 1.0, 0 = no adjust)
  SWORD swGainSin;
  SWORD swGainCos;
      // analog inputs level offset (10000 = pos full scale, 0 = no adjust, -10000 = neg full scale)
  SWORD swOffsetSin;
  SWORD swOffsetCos;
      // Frequency above which switch from analog to digital interpolation [kHz]
  UWORD uwInterpSwitchFreq ;
} INCREMENTAL_PARAMS ;


typedef struct {
    ULONG ulIndexPulseRpt ; // number of pulse to repeat index
    ULONG ulIndexOffset ;
    UWORD uwMaxAngleDiff ;  // max angle diff between main and simulated before alarm [0:65535=0:359deg]
    ULONG ulLineCounts ;    // simulation encoder line counts
    UWORD uwMaxFrequency ;  // max output frequency [kHz]
    union {
      struct {
        BOOL bEnFastAuxSim ;   // 1 to enable fast aux simulation output
        BOOL bBlindAuxSim  ;   // 1 to blind encoder simulation whene electrical angle not valid
      } b ;
    //   UWORD w ;
#ifdef _INFINEON
    }                             flags ;
#else
    } __attribute__((aligned(4))) flags ;
#endif
} AUXSIM_PARAMS ;


/* =========================== global  variables =========================== */
extern INCREMENTAL_PARAMS sIc_IncEncParams[INCREMENTAL_MAX_ENCODER] ;
#ifdef _INFINEON_
extern const INCREMENTAL_PARAMS huge sIc_IncEncDefParams ;
#else
extern const INCREMENTAL_PARAMS sIc_IncEncDefParams ;
#endif // _infineon_
extern INCREMENTAL_OUT sIc_IncEncDataOut[INCREMENTAL_MAX_ENCODER] ;

extern AUXSIM_PARAMS sSe_AuxSimParam ; /* Encoder Simulation */
#ifdef _INFINEON_
extern const AUXSIM_PARAMS huge sSe_AuxSimDefParam ;
#else
extern const AUXSIM_PARAMS  sSe_AuxSimDefParam ;
#endif // _infineon_
extern AUXSIM_OUT sSe_AuxSimOut ;
 
/* =============================== functions =============================== */
BOOL Ic_EncoderInit(UWORD, ENCMGR_SPACEFEEDBACK * psFeedback, SBYTE *) ;
void Ic_GetHwOpt(UWORD uwChannelSel, ULONG * pulHwOpt, ULONG * pulFPGAOpt) ;
BOOL Ic_EncoderParametersCheck(UWORD) ; 
void Ic_EncoderSetElecAngle(UWORD);
void Ic_EncoderSetAbsPos(SQWRD);
BOOL Ic_EncoderGetSnapshot(ULONG *);

BOOL Ic_SimulationInit(ENCMGR_SPACEFEEDBACK * psEnc2Sim, UBYTE * pubStatus) ;
void Ic_SimulationGetHwOpt(ULONG * pulHwOpt, ULONG * pulFPGAOpt) ;
BOOL Ic_SimulationParametersCheck(SBYTE bIsMainRelIncr) ; 

#endif // _IC_H
