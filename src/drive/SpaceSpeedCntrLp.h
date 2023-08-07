/* ************************************************************************ * 
 * Project: AxM-E Control Board                                             * 
 *                                                                          * 
 * Copyright � 2005,2008, Phase Motion Control. All Rights Reserved.        *
 *                                                                          * 
 * Version     : 1.0                                                        * 
 * File        : SpaceSpeed_CntrlLp.h                                       * 
 * Author      : Cristiano Tognetti                                         * 
 *                                                                          * 
 * Description :                                                            * 
 *                                                                          * 
 * ************************************************************************ */

#ifndef _SS_H
 #define _SS_H

#include "common\CommonMotorHandler.h"
#include "common\CommonController.h"
#include "common\CommonEncoderManager.h"
 
/* ================================ #define ================================ */

/* ========================== old structures =============================== */

typedef struct {
  SWORD swPostnKp ;  /* Position: proportional gain value */
  SWORD swSpdKpRef ; /* Speed: reference proportional gain value */
  SWORD swSpdKpFbk ; /* Speed: feedback  proportional gain value */
  SWORD swAccKpRef ; /* Acceleration: reference proportional gain value */
  SWORD swAccKpFbk ; /* Acceleration: feedback  proportional gain value */
  SWORD swKi ;       /* integral gain value */
    
  /* shifts */
  UWORD uwPosGainShift ;    /* Position: global gain shift: ONLY shift RIGHT, max +16 */
  UWORD uwAccGainShift ;    /* Acceleration: gains shift-> ONLY shift LEFT, max -15 */
  SWORD swGlobalGainShift ; /* gains global shift: max shift left = -8; max shift right = +16 */

} SS_V15_PAR_GAINS ;

typedef struct {
  /* Flags */
  union {
      struct {
        UWORD bUseDifferentKp  : 1 ;
      } b ;
      UWORD w ;
  } flags ;
  
  /* gains */
  SS_V15_PAR_GAINS sGains ;  
    
  /* Limits */
  GLB_TORQUE_LIMIT sUsrOutVal32 ;

} SS_V15_PARAMS ;

/* ============================== structures =============================== */

typedef struct {
  GLB_KINEMATIC_DATA *psRef ; /* reference */
  MOTCTRL_STATUS     *psMotCtrlStatus;

  ENCMGR_SPACEFEEDBACK *psFbk ; /* feedback  */
  GLB_TORQUE_LIMIT   *psIqLimit ; /* IqLimit to apply to the integral */

  GLB_IREF           *psCurrentRefs ; /* get the IdRef to be added */
  
  MH_CURLIMITFLAGS   *psILimitActive ;
} SS__INPUT ;


typedef struct {  
  GLB_IREF sIRef ;  
  SQWRD sqFitPErrMax ;
  SQWRD sqFitPErrMin ;
  SLONG slAccRefFiltered32 ;
} SS_OUTPUT ;

typedef struct {
  FLOAT flKi;         //
  FLOAT flPosKp;      // [Arms/rad]
  FLOAT flSpdKpRef;   // [Arms/(rad/s)]
  FLOAT flSpdKpFbk;   // [Arms/(rad/s)]
  FLOAT flAccKpRef;   // [Arms/(rad/s^2)]
  FLOAT flAccKpFbk;   // [Arms/(rad/s^2)]
} SS_PAR_GAINS;

typedef struct {
  SWORD swPostnKp ;  /* Position: proportional gain value */
  SWORD swSpdKpRef ; /* Speed: reference proportional gain value */
  SWORD swSpdKpFbk ; /* Speed: feedback  proportional gain value */
  SWORD swAccKpRef ; /* Acceleration: reference proportional gain value */
  SWORD swAccKpFbk ; /* Acceleration: feedback  proportional gain value */
  SWORD swKi ;       /* integral gain value */
    
  /* shifts */
  UWORD uwPosGainShift ;    /* Position: global gain shift: ONLY shift RIGHT, max +16 */
  UWORD uwAccGainShift ;    /* Acceleration: gains shift-> ONLY shift LEFT, max -15 */
  SWORD swGlobalGainShift ; /* gains global shift: max shift left = -8; max shift right = +16 */

  SQWRD sqIntegralMaxLimit ; /* 64bit: used in control loop to limit integral */
  SQWRD sqIntegralMinLimit ;
    
} SS_RT_PARAMS ;

typedef struct {
  /* Flags */
/*  union {
      struct {
        UWORD bUseDifferentKp  : 1 ;
        UWORD bFilterAccRef    : 1 ;
      } b ;
      UWORD w ;
  } flags ;
*/
  union {
	struct {
	        BOOL bUseDifferentKp;
	        BOOL bFilterAccRef ;
	  } b ;
  } flags ;
  /* gains */
  SS_PAR_GAINS sGains ;
    
  /* Limits */
  GLB_TORQUE_LIMIT sUsrOutVal32 ;

  /* accel filter weight */
  SWORD swAccKFilter ;

} SS_PARAMS ;

typedef struct {
  GLB_TORQUE_LIMIT sUsrOutVal32 ; /* image of the current limit user parameter */
} SS__WORKS ;

typedef struct {
  SS_V15_PAR_GAINS sIntGains ;
  SS_PARAMS sPars ;
} SS_CMB_PARAMS ;

/* =========================== global  variables =========================== */
extern SS__INPUT sSS_CntrLoopIn  ;
extern SS_OUTPUT sSS_CntrLoopOut ;
extern SS_CMB_PARAMS sSS_CntrLoopParam ;
#ifdef _INFINEON_
extern const SS_PARAMS huge sSS_CntrLoopDefParam ;
#else // _infineon_
extern const SS_PARAMS sSS_CntrLoopDefParam ;  
#endif // _infineon_
extern SS__WORKS sSS_CntrLoopWks ;

// -- input from plc
extern GLB_KINEMATIC_DATA  sSS_PlcPidRef ; 
extern MOTCTRL_STATUS      sSS_PlcMotCtrlStatus ;

/* =============================== functions =============================== */
BOOL Ss_ControlLoopInit(void) ;
#ifdef _INFINEON_
SWORD Ss_GetV15Params(SS_V15_PARAMS huge * pvSrc, UWORD uwSize);

/* =============================== plc functions =============================== */
BOOL PlcCtrLpGainsCompute(SS_PAR_GAINS huge * pfSrc, SS_RT_PARAMS huge * pvDst);
BOOL PlcCtrLpGainsSelect(SS_RT_PARAMS huge * pvSrc);
#else  // _infineon_
SWORD Ss_GetV15Params(SS_V15_PARAMS * pvSrc, UWORD uwSize);

/* =============================== plc functions =============================== */
BOOL PlcCtrLpGainsCompute(SS_PAR_GAINS * pfSrc, SS_RT_PARAMS * pvDst);
BOOL PlcCtrLpGainsSelect(SS_RT_PARAMS * pvSrc);   
#endif // _infineon_

#endif // _SS_H
/* EOF */
