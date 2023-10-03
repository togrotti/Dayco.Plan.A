/* ************************************************************************ *
 * Project: Ax-Zynq Control Board                                           *
 *                                                                          *
 * Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. *
 *                                                                          *
 * Version     : 1.0                                                        *
 * File        : MotorHandler.h                                             *
 * Author      : Cristiano Tognetti                                         *
 *                                                                          *
 * Description :                                                            *
 *                                                                          *
 * ************************************************************************ */

#ifndef _MH_H
#define _MH_H

#include "common\CommonMotorHandler.h"

//****************************************************************************
// Defines

#define MH_CURCALIB_INPROGRESS      0
#define MH_CURCALIB_READY           1
#define MH_CURCALIB_ABORTED        -1
#define MH_CURCALIB_FORBIDDEN      -2

#define MH_USRBRIDGELAYOUT_UVW      0
#define MH_USRBRIDGELAYOUT_UWV      1
#define MH_USRBRIDGELAYOUT_VUW      2
#define MH_USRBRIDGELAYOUT_WUV      3
#define MH_USRBRIDGELAYOUT_VWU      4
#define MH_USRBRIDGELAYOUT_WVU      5

#define MH_PWMFREQ_0KHZ             0
#define MH_PWMFREQ_2KHZ             1
#define MH_PWMFREQ_4KHZ             2
#define MH_PWMFREQ_8KHZ             3
#define MH_PWMFREQ_16KHZ            4
#define MH_PWMFREQ_32KHZ            5
#define MH_PWMFREQ_64KHZ            6

#define MH_IQFILTERS_MAX            4

#define MH_FILTER_NONE              0
#define MH_FILTER_LOWPASS           1
#define MH_FILTER_NOTCH             2
#define MH_FILTER_BIQUAD            3

#define MH_FILTER_INTNORM           8192

#define MH_DSPDB_IEC_WS             433

#define USRVDC_SET_ADC             (+16383) // Vdc@1MHz (DSP ADC)
#define USRVDC_SET_FIX             (-16384) // Vdc@125uS

//****************************************************************************
// Input/Output data structures

typedef struct {
  MH_POWERSTAGECONTROL *psPStageCtrl;
  GLB_IREF  *psIRef2Use   ; /* IdRef and IqRef to use */
  UWORD     *puwElecAngle ; /* electrical angle to use */
  SLONG     *pslFilteredSpeed;
  const COMCTRL_HANDLERS *psCtrlHandlers ;
  UWORD     *puwBrakeRValue ;
  SWORD     *pswElecSpeed ;
} MH_MOTORDATA_IN ;

//****************************************************************************
// Parameters data structure

typedef struct {
  ULONG ubType;
  FLOAT flFreqMain;
  FLOAT flDampMain;
  FLOAT flFreqSec;
  FLOAT flDampSec;
} MH_FILTERPARS;

typedef struct {
  SWORD swVBrakeLow;
  SWORD swVBrakeHigh;
  SWORD swUnderVoltageThreshold;
  SWORD swOverVoltageThreshold;
  SLONG slOverCurrentThreshold;
    
  FLOAT flModKi ;
  FLOAT flModKp ;

  UWORD uwUserBridgeLayout;
  UWORD uwVacMinDistortion;     // [%]

  GLB_TORQUE_LIMIT sIqLimit ;
  GLB_TORQUE_LIMIT sIdLimit ;

  /* Flags */
  union {
    struct {
      BOOL bDummy           ;
      BOOL bEnableDeflux    ;
      BOOL bDummy1          ;
      BOOL bDummy2          ;
      BOOL bDisPhasesCheck  ;
      BOOL bDisablePOST     ;
      BOOL bForcePOST       ;
      BOOL bDisVacMgm       ;
#ifdef _HW_DC
      BOOL bEnParallelBr1   ;
      BOOL bEnParallelBr2   ;
#else
      BOOL bDummy4          ;
	  BOOL bDummy5          ;
#endif
      BOOL bDflxWithIqFltRef;
      BOOL bDummy3          ;
    } b ;
    UWORD w[6];
  } flags ;

  MH_FILTERPARS sIqFiltPars[MH_IQFILTERS_MAX];
} MH_MOTORDATA_PARAM ;

//****************************************************************************
// PLC-only input/output data structures

typedef struct {
  GLB_TORQUE_LIMIT sIqLimit;
  GLB_TORQUE_LIMIT sIdLimit;
} MH_PLC_WORKS ;

typedef struct {
  /* Flags */
  union {
      struct {
        BOOL bDisable2StepAuto     ; // word0.byte0
        BOOL bEnable2Steps         ; // word0.byte1
        BOOL bSelect2StepsLOnly    ; // word1.byte0
        BOOL bSelect2StepsFloat    ; // word1.byte1
        BOOL bDtCompCurrentSign    ; // word2.byte0
        BOOL bEnableDirectDrive    ; // word2.byte1
        BOOL bEnableTransfMod      ; // word3.byte0
        BOOL bDisableAutoFrequency ; // word3.byte1
        BOOL bUserVdcBusSet        ; // word4.byte0
        BOOL bSelEnOutEnable       ; // word4.byte1
        BOOL bSelectStraight       ; // word5.byte0
        BOOL bEnablePhaseShift     ; // word5.byte1
        BOOL bEnMultiBrgImmCurr    ; // word6.byte0
        BOOL bDisableOnChange      ; // word6.byte1
        BOOL bEnableCustomCurrScale; // word7.byte0
        BOOL bDisableOCPwmout      ; // word7.byte1
      } b ;
      UWORD w[8] ;
  } flags ;
  UWORD uwDtCompSet;
  UWORD uwDeadTime;
  UWORD uwHoldTime;
  UWORD uwBlankTime;
  UWORD uwTransfModValue;
  UWORD uwSelEnOutMask;
  UBYTE ubSetPwmFrequency;
  GLB_IREF sIOffset ;
  UWORD uwCurrPDTime;       // [nsec]
  UWORD uwVoltPDTime;       // [nsec]
  SWORD swVdcbusSet;
  UWORD uwPwmOffsetV;
  UWORD uwPwmOffsetW;
  UWORD uwCurrentSamples;
  UWORD uwOvCurrCntLimit;

  union {
      struct {
        BOOL bEnableChargerMode  ; // word0.byte0
        BOOL bDspIntegralSet ;     // word0.byte1
      } b ;
      UWORD w ;
  } flags2 ;

  UWORD uwTransfModValue_V;
  UWORD uwTransfModValue_W;

  SWORD swDspIntegralVal_D ;
  SWORD swDspIntegralVal_Q ;
  SWORD swDspOutLimit ;
  MH_DSPPARS sDspPar ; // fpga unit

} MH_PLCADVANCED_WORKS ;

typedef struct {
  FLOAT flRatioIpk;     // Iu, Iv, Iv 
  FLOAT flRatioIrms;    // Id, Iq
  FLOAT flRatioVDCpk;   // Vdc
  FLOAT flRatioVACpk;   // Vu, Vv, Vw
  FLOAT flRatioVACrms;  // Vd, Vq
  FLOAT flRatioVACLinepk; // Vline R, S, T
} MH_PLC_UMRATIOS;

//****************************************************************************
// Macros

#ifdef _INFINEON_
#define is_safetoff_active()    ((!P9_IN_P5 || !P9_IN_P7) && sMotorHandlerRun.flags.b.bSTOInstalled)
#define is_safetoff_active_hi() ((!P9_IN_P5)              && sMotorHandlerRun.flags.b.bSTOInstalled)
#define is_safetoff_active_lo() ((!P9_IN_P7)              && sMotorHandlerRun.flags.b.bSTOInstalled)
#else

#define PWB_RESET_FAULT_PIN     (MIO_PIN_BASE+21)

#ifndef _HW_DC
#define STO_HI_PIN              (MIO_PIN_BASE+37)
#define STO_LO_PIN              (MIO_PIN_BASE+38)
#else
#define STO_HI_PIN              (MIO_PIN_BASE+35)
#define STO_LO_PIN              (MIO_PIN_BASE+38)
#endif

#define is_safetoff_active()    ((!GPIO_IN(STO_HI_PIN) || !GPIO_IN(STO_LO_PIN)) && sMotorHandlerRun.flags.b.bSTOInstalled)
#define is_safetoff_active_hi() ((!GPIO_IN(STO_HI_PIN))                         && sMotorHandlerRun.flags.b.bSTOInstalled)
#define is_safetoff_active_lo() (                        (!GPIO_IN(STO_LO_PIN)) && sMotorHandlerRun.flags.b.bSTOInstalled)

#endif

//****************************************************************************
// General global variables

extern MH_MOTORDATA_IN          sMh_MotorDataIn ;
extern MH_MOTORDATA_OUT         sMh_MotorDataOut ;

extern MH_MOTORDATA_PARAM       sMh_MotorDataParam ;
extern const MH_MOTORDATA_PARAM sMh_MotorDataDefParam ;

extern GLB_IREF                 sMh_MotorDataUsrInIRef ;

extern GLB_TORQUE_LIMIT         sMh_MotorDataUsrSetIqLimit ;
extern GLB_TORQUE_LIMIT         sMh_MotorDataUsrSetIdLimit ;

extern const MH_HANDLERS        sMh_MotorHandlers;

//****************************************************************************
// PLC-only global variables

extern MH_PLC_WORKS             sMh_PlcWorks ;
extern MH_PLCADVANCED_WORKS     sMh_PlcAdvancedWorks ;
extern MH_PLC_UMRATIOS          sMh_PlcUMRatios ;
#ifndef _HW_DC
extern SWORD                    sMh_CurrScaleAdj[3];        // 1000 = 1.0
#endif

extern MH_POWERSTAGECONTROL     sMh_MotorDataPlcPStageCtrl ;
extern GLB_IREF                 sMh_MotorDataPlcInIRef ;
extern UWORD                    uwMh_MotoDataPlcInElecAngle ;
extern SWORD                    swMh_MotoDataPlcInElecSpeed ;
extern SLONG                    slMh_MotoDataPlcInFilteredSpeed ;

//****************************************************************************
// Global functions

BOOL Mh_MotorHandlerInit(UWORD uwTaskParam) ;
void Mh_GetHwOpt(UWORD, ULONG * pulHwOpt, ULONG * pulFPGAOpt);

SWORD Mh_ManageCurrentCalibration(UWORD uwReqSamples);
SWORD Mh_ManualCurrentOffset(SLONG slIuAdj, SLONG slIvAdj, SLONG slIwAdj);
void Mh_ForceCommand(UWORD uwCmd);
void Mh_ForceCommandEx(UWORD uwCmd, ULONG ulParam);
BOOL Mh_PlcCalcFilterConstants(MH_FILTERPARS  * hpsPars, HPSWORD hpswDstV) ;
BOOL Mh_SetIqFilter(UBYTE ubFiltNum, HPSWORD hpswSrcV);
BOOL Mh_PlcDSPLoad(HPUBYTE hpubBin, UWORD uwSize);
BOOL Mh_PlcDSPDBLoad(HPUBYTE hpubBin, UWORD uwSize);
void Mh_PlcDSPHalt(void);
void Mh_PlcDSPResume(void);
BOOL Mh_PlcDSPConfig(BOOL bAdvance);

SWORD Mh_POSTExecute(void);

#endif



