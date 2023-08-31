/* ************************************************************************ * 
 * Project: AxM-E Control Board                                             * 
 *                                                                          * 
 * Copyright © 2005,2008, Phase Motion Control. All Rights Reserved.        * 
 *                                                                          * 
 * Version     : 1.0                                                        * 
 * File        : BackEmfEncoder.h                                           * 
 * Author      : Cristiano Tognetti                                         *  
 *                                                                          * 
 * Description :                                                            * 
 *                                                                          * 
 * ************************************************************************ */
#ifndef _BE_H
 #define _BE_H

#include "common\GlobalStructures.h"
#include "common\CommonEncoderManager.h"

/* ================================ #define ================================ */

/* ============================== structures =============================== */
#if (CFG_ENC_BEMF_DITEN_PARAM)
typedef struct {
	UWORD elThet;
	ULONG mechThet;
	SWORD elSpeed;
	SLONG mechSpeed;
	SLONG revolutionCounter;
} S_OBS_PLL_OUT;

typedef struct {
	ULONG pll_kp;
	ULONG pll_ki;
	ULONG pll_tfilter;
	ULONG obs_gamma1;
	ULONG obs_gamma2;
	ULONG obs_a;
	ULONG id_inj_maxValuePercent;
	ULONG id_inj_accRateInSec;
	ULONG id_inj_minSpeedWithoutInjection;
} OBS_GAINS_COCKPIT;
#endif // cfg_enc_bemf_diten

typedef struct {
    MH_POWERSTAGESTATUS *psPowerStageStatus;
    GLB_KINEMATIC_DATA *psRef ;      // to get speedref from SpeedLoop (needed for OpenLoop and SpeedEmf sign)
    MH_BACKEMF_DATA *psBackEmfData ; // data from MotorHandler (Atan e SquareRoot)
    const MH_HANDLERS *psMotorHandlers ;
    SWORD *psVdcBusFiltered;
} BE_EMFENC__IN ;


typedef struct {
    // flags
    union {
        struct {
            BOOL bDummy ;
        } b ;
//        UWORD w ;
#ifdef _INFINEON_
    } flags ;
#else
} __attribute__((aligned(4))) flags ;
#endif

    UWORD   uwSnsrlessState ;       // Sensorless StateMachine
    UWORD   uwAntiglitchCounter ;   // antiglitch algorithm counter
    ULONG   ulMagicNumber ;         // magic number (calculated from motor Kt: if zero => Alarm
    SLONG   slMechSpeedDeltaAngle ; // speed calculated as angles difference (DeltaAngle)
    FLOAT   flValuedKt ;            // evaluated Kt (ONLY in full sensorless)
    SLONG   slMechMaxSpeed ;        // evaluated max speed on the basis of DcBus and Kt (internal unit)

#if (CFG_ENC_BEMF_DITEN_PARAM)
    S_OBS_PLL_OUT diten_obs_pll_out;
#endif
} BE_EMFENC_DIAG_OUT ;

typedef struct {
    /* Flags */
    union {
        struct {
            BOOL bOpenLoopOnly ;       // flag: only Open Loop
            BOOL bDynamicIqLimit ;     // flag: use dynamic Iq limit
            BOOL bDynamicIdLimit ;     // flag: use dynamic Id limit
            BOOL bUseAntiglitch ;      // flag: use antiglitch filter
            BOOL bUseDeltaAngleSpeed ; // flag: full sensorless use delta angles speed NOT emf
            BOOL bStopMotorKtEval ;    // flag: do not evaluate motor Kt
        } b ;
#ifdef _INFINEON_
    } flags ;
#else
} __attribute__((aligned(4))) flags ;
#endif
      
    UWORD   uwPercentage_BackOnTheFlyUsrSpeed ; // back on the fly speed [0-9999] (potrebbe essere scelto da utente e diverso dal minimo consentito)
    UWORD   uwPercentage_SsnrlssSpeed ;         // full sensorless speed [0-9999]
    UWORD   uwPercentage_SsnrlssThreshold1 ;    // sensorless mix speed threshold1 [0-9999]
    UWORD   uwPercentage_SsnrlssThreshold0 ;    // sensorless mix speed threshold0 [0-9999]
    UWORD   uwEmfEncGlitchFilterPercentage ;    // antiglitch threshold
    ULONG   ulEmfEncGlitchFaultLimit ;          // antiglitch fault limit 
    UWORD   uwKtRefresh ;                       // refresh time of valued motor Kt
    ULONG   ulDisableBackEmfAlarmMask ;
    UWORD   uwIdOpenLoop;                       // [0:1000=0:MOTPRM_PARAMETERS.flCurrentPeak]

#if (CFG_ENC_BEMF_DITEN_PARAM)
    OBS_GAINS_COCKPIT diten_obs_gains;
#endif
}BE_EMFENC_PARAM ;

/* =========================== global  variables =========================== */
extern BE_EMFENC__IN       sBe_EmfEncIn ;
extern BE_EMFENC_DIAG_OUT  sBe_EmfEncDiagOut ;
extern BE_EMFENC_PARAM     sBe_EmfEncParam ;
#ifdef _INFINEON_
extern const BE_EMFENC_PARAM huge sBe_EmfEncDefParam ;
#else
extern const BE_EMFENC_PARAM sBe_EmfEncDefParam ;
#endif // _infineon_

/* =============================== functions =============================== */
BOOL Be_BackEmfHandlerEncInit(ENCMGR_SPACEFEEDBACK *, SBYTE *) ;
BOOL Be_ParametersCheck(void) ; 
void Be_SetElecAngle(UWORD);
void Be_SetAbsPos(SQWRD);

BOOL Be_Hook8KHz(GLB_IREF * psIRefIn, GLB_IREF * psIRefOut);

#endif /* _BE_H */



