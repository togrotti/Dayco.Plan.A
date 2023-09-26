/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : MotorHandlerRT.h                                           */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Motorhandler time-optimized real time module               */
/*                                                                          */
/****************************************************************************/

#ifndef _MOTORHANDLERT_H
#define _MOTORHANDLERT_H

#include "drive\HardwareParameters.h"
#include "common\UnitMeasureConversion.h"

//***************************************************************************
// Defines

#define MHRT_CALSTEP_B0_AVG             1
#define MHRT_CALSTEP_B1_AVG             2
#define MHRT_CALSTEP_B2_AVG             3
#define MHRT_CALSTEP_B0_MIN             4
#define MHRT_CALSTEP_B0_MAX             5

//***************************************************************************
// Data structures

typedef union
{
    struct
    {
        UBYTE ubLayoutU:2;
        UBYTE ubLayoutV:2;
        UBYTE ubLayoutW:2;
    } d;
    UBYTE ubLayout;
} MHRT_CFGLAYOUT;

typedef struct {
    union {
        struct {
            UWORD bUnderVoltage         : 1 ;
            UWORD bCurrent              : 1 ;
            UWORD bOverVoltage          : 1 ;
            UWORD bPowerSection         : 1 ;
            UWORD bBrakeDesaturation    : 1 ;
            UWORD bSafeTorqueOff        : 1 ;
            UWORD bMissingMotorPhase    : 1 ;
            UWORD bACMonitorFail        : 1 ;
            UWORD bDSPOvertime          : 1 ;
            UWORD bAdcWdtExp            : 1 ;
        } b ;
        UWORD w ;
    } sErrorSent ;

  MH_POWERSTAGECONTROL sPStageStatusSet ;

  SWORD swStatorR ; 
  SWORD swStatorL ;
    
  SLONG slIuFb_1 ;
  SLONG slIvFb_1 ;
  
  ULONG ulEmfSQRootFiltered ;
  
  union {
    struct {
      UWORD bUpdateFpgaGains         : 1 ; // b.0
      UWORD bCurrentCalibrEnable     : 1 ; // b.1
      UWORD bCurrentCalibrAbort      : 1 ; // b.2
      UWORD bCurrentCalibrRTSum      : 1 ; // b.3
      UWORD bCurrentCalibrFirstRun   : 1 ; // b.4
      UWORD bCurrentCalibrUpdOnly    : 1 ; // b.5
      UWORD bCurrentCalibrResetTmr   : 1 ; // b.6
      UWORD bBrakeInstalled          : 1 ; // b.7
      UWORD bBackEMFDataEnable       : 1 ; // b.8
      UWORD bBackEMFATanEnable       : 1 ; // b.9
      UWORD bDIF                     : 1 ; // b.10
      UWORD bSTOInstalled            : 1 ; // b.11
      UWORD bResFltPolarity          : 1 ; // b.12
      UWORD bSTOWarnActivated        : 1 ; // b.13
      UWORD bEnergyProtEnabled       : 1 ; // b.14
      UWORD bUpdatePwmFrequency      : 1 ; // b.15
      UWORD bUpdateFilters           : 1 ; // b.16
      UWORD bDefluxEnabled           : 1 ; // b.17
      UWORD bDSPCustomCode           : 1 ; // b.18
      UWORD bAlmaPS                  : 1 ; // b.19
      UWORD bDflxWithIqFltRef        : 1 ; // b.20
      UWORD bDisableSTOAlarm         : 1 ; // b.21
      UWORD bSTOAlarmActivated       : 1 ; // b.22
      UWORD bDspIntegralSet          : 1 ; // b.23
      UWORD bDSPAdvance              : 1 ; // b.24
#ifdef _APP_XC
      UWORD bOvToShortEnabled        : 1 ; // b.25
#endif
#ifdef _HW_DC
      UWORD bParallelBr1Mode         : 1 ; // b.26
      UWORD bParallelBr2Mode         : 1 ; // b.27
#endif
    } b;
    ULONG l;
  } flags;

  FLOAT flModKp;
  FLOAT flModKi;

  GLB_TORQUE_LIMIT sLocalCurrentLimit;

  GLB_TORQUE_LIMIT sIqPartialLimit;
  GLB_TORQUE_LIMIT sIdPartialLimit;

  SLONG slCalibrIuSum;
  SLONG slCalibrIvSum;
  SLONG slCalibrIwSum;
  UWORD uwCalibrCounter;
  UWORD uwCalibrSamplesRequired;

  HWPRMS_AD_SETTINGS tCurCal[3];
#ifdef _HW_DC
  HWPRMS_AD_SETTINGS tCurBr1Cal[3];
  HWPRMS_AD_SETTINGS tCurBr2Cal[3];
#endif

  SLONG slMaxOverCurrent;
  SWORD swMaxOverVoltage;

  UBYTE ubSelFrequency;
  SBYTE bLoFreqRequested;

  UBYTE ubFltSelection;
  UBYTE ubFltNumber;
  SWORD swFltConst[5];

  UWORD uwFltCRC;
  SBYTE sbFltUpdCnt;
  UBYTE ubFltNumValid;
  SBYTE sbFltPlcNumValid;

  UBYTE ubCalibrStep;

  SLONG slAbsCurrLowerLimit ;
  SLONG slAbsSpeedLowerLimit ;

  SWORD swVBrakeLow ;
  SWORD swVBrakeHigh ;

  SWORD * pswACPU;
  SWORD * pswACPV;
  SWORD * pswACPUV;

  SWORD swACVrsMax;
  SWORD swACVtrMax;
  SWORD swACVstMax;
  SWORD swACVrsMin;
  SWORD swACVtrMin;
  SWORD swACVstMin;
  UWORD uwACMTimer;

  UWORD uwMMPCnt;

  UBYTE pubADTagsCurr[3];
#ifdef _HW_DC
  UBYTE pubADTagsBr1Curr[3];
  UBYTE pubADTagsBr2Curr[3];
#endif
  MHRT_CFGLAYOUT sCfgCurrents;

  SWORD swUsrVdcSet;
  SWORD swUsrVdcVal;

  UWORD uwCurrCalibrTmr;

  UWORD uwSTODeactTOut;

  SWORD swVdOut ;
  SWORD swVqOut ;
  FLOAT flRatioV_DC_PEAK ;

  SWORD swIdFb ;
  SWORD swIqFb ;
  FLOAT flRatioI_EQ_RMS ;

  SWORD swDspIntegralVal_D ;
  SWORD swDspIntegralVal_Q ;

#if defined(_CRS_DBG)
  SLONG slDspIntSts_D ; // Dbg
  SLONG slDspIntSts_Q ; // Dbg
#endif

} MHRT_RUNTIME ;

//***************************************************************************
// Globals

extern MHRT_RUNTIME        sMotorHandlerRun ;
extern UMCONV_CONV32TO16   sInternal2Fpga_I_RMS ;
extern UMCONV_CONV32TO16   sInternal2Fpga_I_PEAK ;
#ifndef _HW_DC
#define sInternal2Fpga_I_EQ_RMS sInternal2Fpga_I_RMS
#else
extern UMCONV_CONV32TO16   sInternal2Fpga_I_EQ_RMS ;
#endif



extern UMCONV_CONV32TO16   sInternal2Fpga_V_DC_PEAK ;

extern UMCONV_CONV32TO16   sInternal2Fpga_V_AC_PEAK ;
extern UMCONV_CONV32TO16   sInternal2Fpga_V_AC_RMS ;

extern UMCONV_CONV16TO32   sFpga2Internal_V_PSPL_PEAK ;
extern UMCONV_CONV32TO16   sInternal2Fpga_V_PSPL_PEAK ;

extern FLOAT               flGainsRatio ;

//***************************************************************************
// Prototypes

BOOL Mh_MotorDataFromFpga8KHz(void);
BOOL Mh_MotorDataToFpga8KHz(void);
void Mh_ResetPwrModFaults(void);
void Mh_MotorBackEMF(BOOL);
void Mh_ResetMotorBackEmf(void);
UBYTE Mh_GetActualPwmFrequency(void);

#endif
