/* ************************************************************************ * 
 * Project: AxM-E Control Board                                             * 
 *                                                                          * 
 * Copyright $)AB) 2005,2008, Phase Motion Control. All Rights Reserved.        *
 *                                                                          * 
 * Version     : 1.0                                                        * 
 * File        : ThermalModel.h                                             * 
 * Author      : Cristiano Tognetti                                         *  
 *                                                                          * 
 * Description :                                                            * 
 *                                                                          * 
 * ************************************************************************ */

#ifndef _THERMALMODEL_H
 #define _THERMALMODEL_H

#include "common\GlobalStructures.h"   /* to catch GLB_IREF typedef */
#include "common\CommonMotorHandler.h" /* to catch MH_MOTORDATA_OUT typedef */
#include "common\IOManager.h"          /* to catch IOMGR_ANALOGMEASUREMENTS typedef */
#include "common\CommonController.h"   /* to catch COMCTRL_HANDLERS typedef */

/* ================================ #define ================================ */

#define TM_PLCOPT_DISABLEBIQUAD             1
// do not use 2
#define TM_PLCOPT_ENABLEBIQUAD              3
#define TM_PLCOPT_DISABLE_PKPK              4
#define TM_PLCOPT_DISABLE_FW_MOTORTEMP      5

#define TM_PLCOPTKEY                        0x1ECD
//#define _CRS_DBG_A2DCOUNT					1

/* ============================== structures =============================== */
typedef struct /* outvalue: to user/fieldbus */
{
  SWORD  swDcBusAvrg   ; /* DcBus Average Value        (1e-1 V) */
  SLONG  slIuPeakAvrg  ; /* Iu Average Peak Value      (1e-4 A) */
  SLONG  slIvPeakAvrg  ; /* Iv Average Peak Value      (1e-4 A) */
  SLONG  slIwPeakAvrg  ; /* Iw Average Peak Value      (1e-4 A) */
  SWORD  swTNtc        ; /* Temperature from NTC       (1e-1 Celsius) */
  SWORD  swPjUHTot     ; /* Total Losses IGBT U-Hi     (W)      */
  SWORD  swPjVHTot     ; /* Total Losses IGBT V-Hi     (W)      */
  SWORD  swPjWHTot     ; /* Total Losses IGBT W-Hi     (W)      */
  SWORD  swPjULTot     ; /* Total Losses IGBT U-Lo     (W)      */
  SWORD  swPjVLTot     ; /* Total Losses IGBT V-Lo     (W)      */
  SWORD  swPjWLTot     ; /* Total Losses IGBT W-Lo     (W)      */
  SWORD  swTjUHi       ; /* TJunction IGBT U-Hi        (1e-1 Celsius) */
  SWORD  swTjVHi       ; /* TJunction IGBT V-Hi        (1e-1 Celsius) */
  SWORD  swTjWHi       ; /* TJunction IGBT W-Hi        (1e-1 Celsius) */
  SWORD  swTjULo       ; /* TJunction IGBT U-Lo        (1e-1 Celsius) */
  SWORD  swTjVLo       ; /* TJunction IGBT V-Lo        (1e-1 Celsius) */
  SWORD  swTjWLo       ; /* TJunction IGBT W-Lo        (1e-1 Celsius) */
  SLONG  slPjTot       ; /* Total Losses               (1e-1 W) */
  SWORD  swIgbtTjMax   ; /* Max TJunction for the IGBT (1e-1 Celsius) */
  UWORD  uwCommMargin  ; /* Actual modulation margin   (1e-1)   */
  UWORD  uwFree_1      ; /* free */
  UWORD  uwFree_2      ; /* free */
  SWORD  swTjMax       ; /* Max TJunction   (model)    (1e-1 Celsius) */
  SLONG  slRBrakePower ; /* Power Dissipated by RBrake (1e-1 W) */
  SWORD  swTHeatSink   ; /* T HeatSink (model)         (1e-1 Celsius) */
  UWORD  uwTaskLoad    ; /* Thermal model delta time   (1e-3 s) */
  FLOAT  flMotorTemp   ; /* Motor Temperature          (Celsius)      */
  SWORD  swMotorTemp   ; /* Motor Temperature          (1e-1 Celsius) */
#ifndef _INFINEON_
  FLOAT flSOnchipTemp  ; /* ZynQ on chip temperature   (Celsius) */
  FLOAT flSBoardTemp   ; /* controlboard onboard NTC   (Celsius) */
#endif
} TM_THERMAL_MODEL_DIAGNOSTIC_OUTPUT ;


typedef struct
{
    IOMGR_ANALOGMEASUREMENTS *psAdcIoData ;
    MH_MOTORDATA_OUT         *psMotorData ;
    const MH_HANDLERS        *psMotorHandlers ;
    const COMCTRL_HANDLERS   *psCtrlHandlers ;
} TM_THERMAL_MODEL_INPUT ;


typedef struct
{  
  GLB_TORQUE_LIMIT sIdLimit ; /* Id limit to use due to thermal model */
  GLB_TORQUE_LIMIT sIqLimit ; /* Iq limit to use due to thermal model */
  
  TM_THERMAL_MODEL_DIAGNOSTIC_OUTPUT sOutValue ;

  UWORD uwBrakeResistorValue; /* 1e-1 Ohm */ 
  UWORD uwBrakeResistorPower; /* 1e-1 W   */
  FLOAT flTNtc;               /* Celsius  */
  FLOAT flTjMax;              /* Celsius  */
  FLOAT flTHeatSink;          /* Celsius  */
  ULONG ulBrakeResistorEnergy;/* J        */
  ULONG ulOutValueRBrakeEnergy; // J
  SLONG slDynamicOverCurrent ;  // 1e-4Arms
  SWORD swI2T_MotorDeltaT ;

#ifdef _HW_DC
  FLOAT  flMotorKTY          ; /* Motor KTY Temperature (Celsius) */
  FLOAT  flMotorPTC          ; /* Motor PTC Temperature (Celsius) */
  FLOAT  flBr0TNtc;            /* Celsius  */
  FLOAT  flBr1TNtc;            /* Celsius  */
  FLOAT  flBr2TNtc;            /* Celsius  */
#endif

#ifdef _HW_CT
  FLOAT  flBrdTntc;
  SWORD  swBrdTemp;
#endif

#if (defined(_CRS_DBG) && defined(_CRS_DBG_A2DCOUNT))
  UWORD uwNtcCount_0 ;
  UWORD uwNtcCount_1 ;
  UWORD uwPtcCount ;
  UWORD uwKtyCount ;
#endif // _crs_dbg

} TM_THERMAL_MODEL_OUTPUT ;

typedef struct
{ 
  UWORD uwRBrakeValue ;    /* 1e-1 Ohm */ 
  UWORD uwRBrakeMaxPower ; /* 1e-1 W   */
  SWORD swCoolingTempOn  ; /* 1e-1 Celsius */
  SWORD swCoolingTempOff ; /* 1e-1 Celsius */
  SWORD swMotorOverTemp  ; /* 1e-1 Celsius */
  ULONG ulRBrakeMaxEnergy; /*      J   */
  SWORD swI2TMaxDelta    ; /* 1e-1 Celsius */
/*  union {
      struct {
        UWORD bDisFltReactOnLimit    : 1 ; // disable fault reaction when entering current derating for temperature excess
        UWORD bNoBrakeDisableOnFlt   : 1 ; // in case of brake overpower+10% brake drive is never disabled
        UWORD bNoFatalOnBrakeFlt     : 1 ; // in case of brake overpower+10% no fatal fault is sent, motor always under control
      } b ;
      UWORD w ;
  } flags ;
*/
  union {
        struct {
          BOOL bDisFltReactOnLimit; // disable fault reaction when entering current derating for temperature excess
          BOOL bNoBrakeDisableOnFlt; // in case of brake overpower+10% brake drive is never disabled
          BOOL bNoFatalOnBrakeFlt; // in case of brake overpower+10% no fatal fault is sent, motor always under control
        } b ;
    }__attribute__((aligned(4)))flags ;
#ifdef _HW_DC
  FLOAT sflPTCCoeff[4];    /* coefficients for 3rd order linearization of temperature from resistance measure */
#endif
  FLOAT sflKTYCoeff[4];

#ifndef _INFINEON_
  FLOAT flOnChipOverTemp ;
#endif
} TM_THERMAL_MODEL_PARAM ;


/* =========================== global  variables =========================== */
extern FLOAT flTm_PlcMotorTemp ;
extern TM_THERMAL_MODEL_INPUT  sTm_ThModIn  ;
extern TM_THERMAL_MODEL_OUTPUT sTm_ThModOut ;
extern TM_THERMAL_MODEL_PARAM  sTm_ThModParam ;
#ifdef _INFINEON_
extern const TM_THERMAL_MODEL_PARAM huge sTm_ThModDefParam ;
#else
extern const TM_THERMAL_MODEL_PARAM sTm_ThModDefParam ;

#endif // _infineon_
/* =============================== functions =============================== */
void Tm_GetHwOpt(UWORD, ULONG *, ULONG *);
BOOL Tm_ThermalModelInit(void) ;
BOOL Tm_SetPlcOption(UWORD uwOption, ULONG ulValue);

#endif
/* EOF */
