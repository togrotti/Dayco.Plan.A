/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CommonMotorHandler.h                                       */
/* Author      : Cristiano Tognetti, Fabio Terrile                          */
/*                                                                          */
/* Description : Common data for all kinds of encoders                      */
/*                                                                          */
/****************************************************************************/

#ifndef _COMMONMOTORHANDLER_H
#define _COMMONMOTORHANDLER_H

#include "system\SysAppConfig.h"
#include "GlobalStructures.h"
#include "CommonController.h"

//****************************************************************************
// Defines

#define MH_INIT_INPUT               0
#define MH_INIT_OUTPUT              1
#define MH_INIT_LIMITLIST           2
#define MH_INIT_POST                3

#define MH_LIMITLIST_IQ             1
#define MH_LIMITLIST_ID             2

#define MH_CMD_DISABLEBRAKEDRIVE    1
#define MH_CMD_BACKEMFDATAENABLE    2
#define MH_CMD_BACKEMFATANENABLE    3
#define MH_CMD_EN_ENERGYPROTECTION  4
#define MH_CMD_DIS_ENERGYPROTECTION 5
#define MH_CMD_SETOVERCURRENTLIMIT  6
#define MH_CMD_SETMODULATION        7
#define MH_CMD_FORCELOWFREQUENCY    8
#define MH_CMD_SETIQFILTER          9
#define MH_CMD_FORCEBRAKEON         10
#define MH_CMD_ENABLEBRAKEDRIVE     11
#define MH_CMD_EN_OVTOOUTSHORT      12
#ifdef _HW_CT
#define MH_CMD_SET_BRAKE_DUTY       13
#endif
#define MH_CMD_DISABLESTOALARM      14
#define MH_CMD_FORCE_ASC_CMD        15  // ASC crs
#ifdef _HW_DC
#define MH_CMD_PWM_SHIFT_SETVALUE   16  // multi bridge PWM shift set value: HiWord = Br2; LoWord = Br1
#endif
#define MH_CMD_SETMOD_ENABLE2STEPS  (1u<<FPGA_PWMS_B_2SWITCH)
#define MH_CMD_SETMOD_2STEPSFLOAT   (1u<<FPGA_PWMS_B_2SWITCH_HL)
#define MH_CMD_SETMOD_2STEPSLONLY   (1u<<FPGA_PWMS_B_2SWITCH_LOWONLY)
#define MH_CMD_SETMOD_STRAIGHT      (1u<<FPGA_PWMS_B_STRAIGHT)

#define MH_ACMON_NOERROR            0
#define MH_ACMON_FAIL_PHASE_R       1
#define MH_ACMON_FAIL_PHASE_S       2
#define MH_ACMON_FAIL_PHASE_T       3

//****************************************************************************
// Data types

typedef struct {
  SLONG slOverCurrent  ; /* 1e-4Arms */
  SLONG slCurrentLimit ; /* 1e-4Arms */
  SWORD swOverVoltage  ; /* 1e-1V */
} MH_DRIVE_LIMIT ;

typedef struct {
  struct {
    BOOL bPowerEnable      ;
    BOOL bReferenceEnable  ;
    BOOL bDisableBrakeDrive;
    BOOL bBackEMFDataEnable;
    BOOL bBackEMFATanEnable;
  } b ;
} MH_POWERSTAGECONTROL ;

typedef struct {
  struct {
    BOOL bVoltageEnabled  ; // bit from FPGA: PWM is switched on with dutycycle @50%
    BOOL bFullyActive     ; // bit from FPGA: PWM is fully enabled with dutycycle "working"
    BOOL bReferenceEnabled; // bit from fw: reference sent to FPGA are valid (current reference different from zero)
    BOOL bBackEMFDataValid;
    BOOL bACInputDataValid;
    BOOL bBridge1Valid    ;
    BOOL bBridge2Valid    ;
    BOOL bOutShortActive  ;
    BOOL bStoActiveHigh   ;
    BOOL bStoActiveLow    ;
    BOOL bPowerFault      ;
  } b ;
} MH_POWERSTAGESTATUS ;

typedef union {
    struct {
        UWORD bIdMin : 1 ;
        UWORD bIdMax : 1 ;
        UWORD bIqMin : 1 ;
        UWORD bIqMax : 1 ;
    } b ;
    UWORD w ;
} MH_CURLIMITFLAGS ;

typedef struct {
    SWORD swIU;  
    SWORD swIV;  
    SWORD swIW;  
} MH_BRIDGEOFFSETS;

typedef struct {
  SWORD swAlpha ; /* Alpha */
  SWORD swBeta  ; /* Beta  */
  UWORD uwVmotor ; /*  Vmotor = sqrt(x^2 + y^2) (filtered): valore istantaneo, NON RMS */ 
  UWORD uwAtanAngle ; /* Atan(x/y) */
} MH_BACKEMF_DATA ;

typedef struct {
  SWORD swKi;
  SWORD swKp;
  SWORD swIOutShift;
  UWORD uwModOutLimit;
} MH_DSPPARS;

typedef struct {
  SWORD swId;
  SWORD swIq;
  SWORD swESin;
  SWORD swECos;
} MH_DSPOUT;

//****************************************************************************
// Input/Output data structures

typedef struct {
  SLONG slIdFb ; /* FeedBack Id read by the FPGA (RMS)  */
  SLONG slIqFb ; /* FeedBack Iq read by the FPGA (RMS)  */
  SLONG slIuFb ; /* FeedBack Iu read by the FPGA (PEAK) */
  SLONG slIvFb ; /* FeedBack Iv read by the FPGA (PEAK) */
  
  MH_BACKEMF_DATA sBackEmfData ;
    
  SWORD swVuEstimated;
  SWORD swVvEstimated;
  SWORD swDcBusValue;
    
  MH_DRIVE_LIMIT sDriveLimit ;
  GLB_TORQUE_LIMIT sIqLimit ;
  GLB_TORQUE_LIMIT sIdLimit ;
  
  GLB_IREF sI2Fpga ; /* values written to the FPGA */

  MH_POWERSTAGESTATUS sPowerStageSts ;

  SWORD swVdOut ;
  SWORD swVqOut ;
  SWORD swVuOut ;
  SWORD swVvOut ;

  FLOAT flAutoModKi ;
  FLOAT flAutoModKp ;

  SLONG slIwFb ; /* FeedBack Iw read by the FPGA (PEAK) */

  SWORD swActiveVBrakeLow ;
  SWORD swActiveVBrakeHigh ;

  MH_CURLIMITFLAGS sCurrLimFlags ;

  MH_BRIDGEOFFSETS sB0Offsets;
  UWORD uwActualPwmSetup;

  UBYTE ubDSPLoad;
  UBYTE ubDSPIsCustom;
  UWORD uwDSPCompLevel;

  UBYTE ubActualPwmFrequency;
  UBYTE ubACStatus;

  SLONG slIqFltRef ;

  /* Noise compensation Ipkpk */
  MH_BRIDGEOFFSETS sB0IpkpkMaxOffsets ;
  MH_BRIDGEOFFSETS sB0IpkpkMinOffsets ;

  SWORD swACVrs ;
  SWORD swACVst ;
  SWORD swACVtr ;

  MH_DSPPARS sDSPPar ;

  MH_DSPOUT sDSPOut ;

#ifdef _HW_DC
  SLONG slBr1IuFb ;
  SLONG slBr1IvFb ;
  SLONG slBr1IwFb ;

  MH_BRIDGEOFFSETS sB1Offsets;

  SLONG slBr2IuFb ;
  SLONG slBr2IvFb ;
  SLONG slBr2IwFb ;

  MH_BRIDGEOFFSETS sB2Offsets;
#endif

  UWORD uwPostResultEx;

#if CFG_VMOTOR_READ
  SWORD swVuEffective;
  SWORD swVvEffective;
  SWORD swVwEffective;
#endif 

  SLONG slIuFb_Imm ;
  SLONG slIvFb_Imm ;
  SLONG slIwFb_Imm ;

#ifdef _HW_DC
  SLONG slBr1IuFb_Imm ;
  SLONG slBr1IvFb_Imm ;
  SLONG slBr1IwFb_Imm ;

  SLONG slBr2IuFb_Imm ;
  SLONG slBr2IvFb_Imm ;
  SLONG slBr2IwFb_Imm ;
#endif

#if defined (_HW_AXS_IML_PSU)
  SWORD swDcBusValueImm;
#endif // _hw_axs_iml_psu

#if defined(_CRS_DBG)
  SLONG slDspIntSts_D ; // Dbg
  SLONG slDspIntSts_Q ; // Dbg
  SWORD swVdcOut_ADC  ; // Dbg
#endif

  SWORD swVMotor;

} MH_MOTORDATA_OUT ;

//****************************************************************************
// Exported handlers

typedef struct {
  BOOL (* pfAdd2LimitList)(UWORD uwSel, GLB_TORQUE_LIMIT *sStruct2Add);
  BOOL (* pfDeleteLimitFromList)(UWORD uwSel, GLB_TORQUE_LIMIT *sStruct2Delete);
  void (* pfForceLimitListScan)(void);
  void (* pfForceCommand)(UWORD, ULONG);
} MH_HANDLERS;


#endif



