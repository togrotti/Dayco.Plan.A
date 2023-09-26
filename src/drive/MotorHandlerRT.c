/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : MotorHandlerRT.c                                           */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Motorhandler time-optimized real time module               */
/*                                                                          */
/****************************************************************************/

#include <math.h>
#include <stdlib.h> // to use labs()

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "common\MathFunctions.h" // to use Sin16 and Cos16
#include "common\UnitMeasureConversion.h"

#include "drive\MotorHandler.h"
#include "drive\MotorHandlerRT.h"

#include "fpga\FpgaIRegs.h"
#include "fpga\FpgaHandler.h"

#include "drive\DriveTaskController.h"
#include "drive\Deflux.h"

#include "plc\Plc.h"
#include "drive\EncoderManager.h"
#include "drive\MotorParameters.h"

#include "fpga\AnProcessor.h"
#include "system\SysAppGlobals.h"
#include "system\SysLogManagement.h" // to use SysLogManagement

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#if defined(_CRS_DBG)
#if CRS_DBGDSK
#pragma GCC optimize (0) // crs_dbg
#else
#pragma GCC optimize (2)
#endif
#else
#pragma GCC optimize (2)
#endif

//***************************************************************************
// Defines

#define MMP_FILTER  16      // 16 RT cycles
#define SAFETORQUEOFF_REST_ALARM_TOUT 50 // ms

//***************************************************************************
// Globals

MHRT_RUNTIME        sMotorHandlerRun ;
UMCONV_CONV32TO16   sInternal2Fpga_I_RMS ;
UMCONV_CONV32TO16   sInternal2Fpga_I_PEAK ;
#ifdef _HW_DC
UMCONV_CONV32TO16   sInternal2Fpga_I_EQ_RMS ;
#endif

UMCONV_CONV32TO16   sInternal2Fpga_V_DC_PEAK ;

UMCONV_CONV32TO16   sInternal2Fpga_V_AC_PEAK ;
UMCONV_CONV32TO16   sInternal2Fpga_V_AC_RMS ;

UMCONV_CONV16TO32   sFpga2Internal_V_PSPL_PEAK ;
UMCONV_CONV32TO16   sInternal2Fpga_V_PSPL_PEAK ;

FLOAT               flGainsRatio ; // normalization factor for automatic gains

//***************************************************************************
//
BOOL Mh_MotorDataFromFpga8KHz(void)
{
  UWORD uwPwmStatus = FPGA_PWM_STATUS ;
#if CFG_ASC_DAYCO_MGMT
  UWORD uwAscStatus = FPGA_PWM_ASC_CTRL ;
#endif // cfg_asc_dayco_mgmt
  static UWORD uwPowerFaultCnt=0;
  BOOL bStoActiveHi,bStoActiveLo;

  if (bSysStatAlarmsReset)
  {
    Mh_ResetPwrModFaults();
    sMotorHandlerRun.sErrorSent.w = 0 ; /* clear errors */
    sMotorHandlerRun.uwMMPCnt = 0;
  }

  sMh_MotorDataOut.sPowerStageSts.b.bVoltageEnabled = FPGA_PWM_STATUS_ENABLED(uwPwmStatus) ;
  sMh_MotorDataOut.sPowerStageSts.b.bFullyActive = FPGA_PWM_STATUS_ENABLED(uwPwmStatus) & FPGA_PWM_STATUS_FULLY_ACTIVE(uwPwmStatus) ;
#ifdef _APP_XC
#if CFG_ASC_DAYCO_MGMT
  sMh_MotorDataOut.sPowerStageSts.b.bOutShortActive = FPGA_PWM_ASC_CTRL_STS_ACTIVE(uwAscStatus) ;
#else
  sMh_MotorDataOut.sPowerStageSts.b.bOutShortActive = FPGA_PWM_STATUS_OUTSHORT_ACTIVE(uwPwmStatus) ;
#endif // cfg_asc_dayco_mgmt
#else
  sMh_MotorDataOut.sPowerStageSts.b.bOutShortActive = FALSE ;
#endif  // _app_xc

  bSysStatLockedByAscActivated = sMh_MotorDataOut.sPowerStageSts.b.bOutShortActive ;

    // STO Active
  bStoActiveHi = sMh_MotorDataOut.sPowerStageSts.b.bStoActiveHigh = is_safetoff_active_hi();
  bStoActiveLo = sMh_MotorDataOut.sPowerStageSts.b.bStoActiveLow  = is_safetoff_active_lo();

  if ((bStoActiveHi || bStoActiveLo) && !sMotorHandlerRun.flags.b.bSTOWarnActivated)
  {
    sMotorHandlerRun.flags.b.bSTOAlarmActivated = TRUE;
    sMotorHandlerRun.uwSTODeactTOut = timer_settimeout (uwSysTimers1ms, SAFETORQUEOFF_REST_ALARM_TOUT);
  }
  else
  {
      // if not active but was activated then reset power faults as loss of
      // igbt driver supply cause power fault reading (after a timeout to let fully
      // power supply restore)
    if ( sMotorHandlerRun.flags.b.bSTOAlarmActivated && timer_istimedout(uwSysTimers1ms, sMotorHandlerRun.uwSTODeactTOut) )
      sMotorHandlerRun.flags.b.bSTOAlarmActivated = FALSE ;
  }

    // Powersection Fault
  sMh_MotorDataOut.sPowerStageSts.b.bPowerFault  = FPGA_PWM_STATUS_DESATURATION(uwPwmStatus);

  bSysStatPowerEnabled = sMh_MotorDataOut.sPowerStageSts.b.bVoltageEnabled ;
  bSysStatPowerReady   = sMh_MotorDataOut.sPowerStageSts.b.bFullyActive    ;

  if (sMotorHandlerRun.flags.b.bDSPAdvance)
  {  // Id and Iq filtered by DSP: conversion from FPGA unit to internal unit
    sMh_MotorDataOut.slIdFb = (SLONG)((FLOAT)sMotorHandlerRun.swIdFb * sMotorHandlerRun.flRatioI_EQ_RMS) ;
    sMh_MotorDataOut.slIqFb = (SLONG)((FLOAT)sMotorHandlerRun.swIqFb * sMotorHandlerRun.flRatioI_EQ_RMS) ;

#if defined(_CRS_DBG)
    sMh_MotorDataOut.slDspIntSts_D = sMotorHandlerRun.slDspIntSts_D ;
    sMh_MotorDataOut.slDspIntSts_Q = sMotorHandlerRun.slDspIntSts_Q ;
#endif
  }

    /* check fault from PWM */
  if (FPGA_PWM_STATUS_FAULT_ACTIVE(uwPwmStatus))
  {
      // the first time here issue a data refresh
    if(FPGA_REG16_GETBIT(FPGA_PWM_SET, FPGA_PWM_B_ENABLE_HOLD_SIGNALS))
      AnProc_ForceRefresh();

    if(FPGA_PWM_STATUS_OVER_CURRENT(uwPwmStatus) && !sMotorHandlerRun.sErrorSent.b.bCurrent)
    {
      sMotorHandlerRun.sErrorSent.b.bCurrent = TRUE ;
      SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_CURRENT_FAIL, 0l, TRUE) ;
      (*sMh_MotorDataIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
    }

    if(FPGA_PWM_STATUS_OVER_VOLTAGE(uwPwmStatus) && !sMotorHandlerRun.sErrorSent.b.bOverVoltage)
    {
      sMotorHandlerRun.sErrorSent.b.bOverVoltage = TRUE ;
      SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_VL_OVERVOLTAGE_FAIL, 0l, TRUE) ;
      (*sMh_MotorDataIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
    }

    if(!(bStoActiveHi || bStoActiveLo) && !sMotorHandlerRun.flags.b.bSTOWarnActivated && !sMotorHandlerRun.flags.b.bSTOAlarmActivated && FPGA_PWM_STATUS_DESATURATION(uwPwmStatus) && !sMotorHandlerRun.sErrorSent.b.bPowerSection)
    {
      uwPowerFaultCnt++;
      if(uwPowerFaultCnt>=16)
      {
        sMotorHandlerRun.sErrorSent.b.bPowerSection = TRUE ;
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_HW_POWERSECTION_FAIL, 0l, TRUE) ;
        (*sMh_MotorDataIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
      }
    }
    else if(uwPowerFaultCnt>0)
      uwPowerFaultCnt--;

    if((FPGA_PWM_STATUS_BRAKE_DESAT(uwPwmStatus)) && (!sMotorHandlerRun.sErrorSent.b.bBrakeDesaturation))
    {
      sMotorHandlerRun.sErrorSent.b.bBrakeDesaturation = TRUE ;
      SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_HW_BRAKE_FAIL, SYSTEMALARMS_SUBCODE_BF_DESATURATION, TRUE) ;
      (*sMh_MotorDataIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
    }

    if(FPGA_PWM_STATUS_EXT_FAULT(uwPwmStatus))
    {
      if((FPGA_DSPH_INSRAML_OVERTIME) && (!sMotorHandlerRun.sErrorSent.b.bDSPOvertime))
      {
        sMotorHandlerRun.sErrorSent.b.bDSPOvertime = TRUE ;
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_PLC_OVERTIME, SYSTEMALARMS_SUBCODE_PLC_DSP_OVERTIME, TRUE) ;
        (*sMh_MotorDataIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
      }
    }
      // disable signals holding, re-enable it at next startup
    FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_ENABLE_HOLD_SIGNALS, 0);

      // and return skipping remaining code for timing purpose
    return TRUE;
  }
#if CFG_VMOTOR_READ
  // update from voltage estimator [0.1V]
  /*
   * NOTE 20230704:
   * 	DC Bus voltage in FPGA is fixed to value 2048d. This means
   * 	that here we need to divide by 2000 (to normalize the estimation)
   * 	and to multiply for current DC Bus value (unit is 0.1V).
   */
  sMh_MotorDataOut.swVuEffective = (SWORD)((FLOAT)sMh_MotorDataOut.swDcBusValue * (((SWORD)FPGA_VEST_U) / 2000.0F));
  sMh_MotorDataOut.swVvEffective = (SWORD)((FLOAT)sMh_MotorDataOut.swDcBusValue * (((SWORD)FPGA_VEST_V) / 2000.0F));
  sMh_MotorDataOut.swVwEffective = (SWORD)((FLOAT)sMh_MotorDataOut.swDcBusValue * (((SWORD)FPGA_VEST_W) / 2000.0F));
#endif // cfg_vmotor_read

  if (sMotorHandlerRun.flags.b.bDSPAdvance)
  {  // Vd and Vq filtered by DSP: conversion from FPGA unit to internal unit
    sMh_MotorDataOut.swVdOut = (SWORD)((FLOAT)sMotorHandlerRun.swVdOut * sMotorHandlerRun.flRatioV_DC_PEAK) ;
    sMh_MotorDataOut.swVqOut = (SWORD)((FLOAT)sMotorHandlerRun.swVqOut * sMotorHandlerRun.flRatioV_DC_PEAK) ;
  }

  sMh_MotorDataOut.swVuEstimated = sMh_MotorDataOut.swVuOut;
  sMh_MotorDataOut.swVvEstimated = sMh_MotorDataOut.swVvOut;
  if (sMh_MotorDataOut.sPowerStageSts.b.bACInputDataValid)
  {
    *sMotorHandlerRun.pswACPUV = -*sMotorHandlerRun.pswACPU -*sMotorHandlerRun.pswACPV;

    if(sMotorHandlerRun.swACVrsMax<sMh_MotorDataOut.swACVrs)
      sMotorHandlerRun.swACVrsMax=sMh_MotorDataOut.swACVrs;
    if(sMotorHandlerRun.swACVtrMax<sMh_MotorDataOut.swACVtr)
      sMotorHandlerRun.swACVtrMax=sMh_MotorDataOut.swACVtr;
    if(sMotorHandlerRun.swACVstMax<sMh_MotorDataOut.swACVst)
      sMotorHandlerRun.swACVstMax=sMh_MotorDataOut.swACVst;
    if(sMotorHandlerRun.swACVrsMin>sMh_MotorDataOut.swACVrs)
      sMotorHandlerRun.swACVrsMin=sMh_MotorDataOut.swACVrs;
    if(sMotorHandlerRun.swACVtrMin>sMh_MotorDataOut.swACVtr)
      sMotorHandlerRun.swACVtrMin=sMh_MotorDataOut.swACVtr;
    if(sMotorHandlerRun.swACVstMin>sMh_MotorDataOut.swACVst)
      sMotorHandlerRun.swACVstMin=sMh_MotorDataOut.swACVst;
  }
    // calc BackEMF when voltage is enabled and if requested
  if (sMh_MotorDataOut.sPowerStageSts.b.bVoltageEnabled && sMotorHandlerRun.flags.b.bBackEMFDataEnable)
    Mh_MotorBackEMF(sMotorHandlerRun.flags.b.bBackEMFATanEnable) ; // calculate Motor BackEmf
  else
    Mh_ResetMotorBackEmf() ; // reset keep Motor BackEmf data

    // if power disabled and external request, make current averaging for offset compensation
  if(sMotorHandlerRun.flags.b.bCurrentCalibrRTSum)
  {
    switch(sMotorHandlerRun.ubCalibrStep)
    {
      case MHRT_CALSTEP_B0_AVG:
      case MHRT_CALSTEP_B1_AVG:
      case MHRT_CALSTEP_B2_AVG:
        if(sMh_MotorDataOut.sPowerStageSts.b.bVoltageEnabled)
          sMotorHandlerRun.flags.b.bCurrentCalibrAbort=TRUE;
        else
        {
          sMotorHandlerRun.slCalibrIuSum+=(SWORD)FPGA_CURCHMINMAX_IU_MAX+(SWORD)FPGA_CURCHMINMAX_IU_MIN;
          sMotorHandlerRun.slCalibrIvSum+=(SWORD)FPGA_CURCHMINMAX_IV_MAX+(SWORD)FPGA_CURCHMINMAX_IV_MIN;
          sMotorHandlerRun.slCalibrIwSum+=(SWORD)FPGA_CURCHMINMAX_IW_MAX+(SWORD)FPGA_CURCHMINMAX_IW_MIN;
        }
        break;

      case MHRT_CALSTEP_B0_MIN:
        if(sMh_MotorDataOut.sPowerStageSts.b.bVoltageEnabled)
          sMotorHandlerRun.flags.b.bCurrentCalibrAbort=TRUE;
        else
        {
          sMotorHandlerRun.slCalibrIuSum+=(SWORD)FPGA_CURCHMINMAX_IU_MIN;
          sMotorHandlerRun.slCalibrIvSum+=(SWORD)FPGA_CURCHMINMAX_IV_MIN;
          sMotorHandlerRun.slCalibrIwSum+=(SWORD)FPGA_CURCHMINMAX_IW_MIN;
        }
        break;

      case MHRT_CALSTEP_B0_MAX:
        if(sMh_MotorDataOut.sPowerStageSts.b.bVoltageEnabled)
          sMotorHandlerRun.flags.b.bCurrentCalibrAbort=TRUE;
        else
        {
          sMotorHandlerRun.slCalibrIuSum+=(SWORD)FPGA_CURCHMINMAX_IU_MAX;
          sMotorHandlerRun.slCalibrIvSum+=(SWORD)FPGA_CURCHMINMAX_IV_MAX;
          sMotorHandlerRun.slCalibrIwSum+=(SWORD)FPGA_CURCHMINMAX_IW_MAX;
        }
        break;

      default:
        assert(FALSE);
    }
    sMotorHandlerRun.uwCalibrCounter++;
  }

  return TRUE ;
}

//***************************************************************************
//
BOOL Mh_MotorDataToFpga8KHz(void)
{
  static BOOL  bDisModProcess=FALSE;
  // defined as static in order to force compiler to allocate in the bit RAM
  static BOOL  bGenFlag;
  static BOOL  bUpdILoop;
  static BOOL  bSkipUpdates;
  UWORD uwCt;
  SWORD swAngleIncConst;
  bGenFlag=FALSE;
  bUpdILoop=FALSE;
  bSkipUpdates=FALSE;
  static MH_PLCADVANCED_WORKS sLoc_Mh_PlcAdvancedWorks;

  /* if reset procedure started, disable power but not brake and exit */
  if(bSysStatResetting)
  {
    FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_SETUP_ENABLE, 0);
    return TRUE;
  }

  /* set-up power stage control (if changes) */
  if(memcmp(&sMotorHandlerRun.sPStageStatusSet.b, &sMh_MotorDataIn.psPStageCtrl->b, sizeof(MH_POWERSTAGECONTROL)))
  {
    if(!sMotorHandlerRun.flags.b.bCurrentCalibrFirstRun && !sMotorHandlerRun.flags.b.bCurrentCalibrEnable)
    {
      if(sMotorHandlerRun.sPStageStatusSet.b.bPowerEnable!=sMh_MotorDataIn.psPStageCtrl->b.bPowerEnable)
      {
              // Power Enable Commanded (only if not locked)
          if(sMh_MotorDataIn.psPStageCtrl->b.bPowerEnable && !bSysStatLockedByPlcReload)
          {
              sMh_MotorDataIn.psPStageCtrl->b.bDisableBrakeDrive=FALSE;

              if(!sMotorHandlerRun.flags.b.bDSPCustomCode)
              {  // Disable DSPH for integral status resetting and wait (not forever) ack
                 FPGA_DSPH_SET_HALT = TRUE;
                 for(uwCt=0;uwCt<100;uwCt++)
                   if(FPGA_DSPH_INSRAML_HALTACK)
                     break;
                 // reset integral status (48bit)
                 FPGA_CPUH_DRAM_WR_SW(FPGAIR_S_KI_D, 0);
                 FPGA_CPUH_DRAM_WR_SW(FPGAIR_S_KI_Q, 0);

                 // when DSPStandard the mode (swUsrVdcSet) and the value (swUsrVdcVal) are set in background and applied  only here (enable command rised edge): it is not possibile to change the value @runtime
                 // when DSPAdvanced the mode (swUsrVdcSet) is set at ONLY boot (Mh_MotorDataFromFpgaInit) and the value (swUsrVdcVal) can be changed @8kHz (and it is used FPGAIR_P_VDC_SETVAL instead of FPGAIR_P_VDC_VAL)
                 FPGA_CPUH_DRAM_WR_SW(FPGAIR_P_VDC_SEL, sMotorHandlerRun.swUsrVdcSet); // select if Vdc@1MHz (USRVDC_SET_ADC) or Vdc@8kHz (USRVDC_SET_FIX)
                 FPGA_CPUH_DRAM_WR_SW(FPGAIR_P_VDC_VAL, sMotorHandlerRun.swUsrVdcVal); // FPGAIR_P_VDC_VAL is used only when DSPStandard

                 // and reenable DSPH
                 FPGA_DSPH_SET_HALT = FALSE;
              }

                  // enable PWM and brake (if was disabled and/or if installed)
              FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_ENABLE_HOLD_SIGNALS, 1);
              FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_SETUP_BRAKE_ENABLE, sMotorHandlerRun.flags.b.bBrakeInstalled);

                  // check bridge modulation options
              if(sMh_PlcAdvancedWorks.flags2.b.bEnableChargerMode)
              {  /* set system in ChargerMode */
                  FPGA_REG16_SETBIT(FPGA_PWM_SET,   FPGA_PWM_B_SETUP_USE_MODULATOR, 1);
                  FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_TRANSFMOD, 0);
                  FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_EN_CHARGERMODE, 1);
                  bDisModProcess = TRUE;
              }
              else if(sMh_PlcAdvancedWorks.flags.b.bEnableDirectDrive)
              {
                  FPGA_REG16_SETBIT(FPGA_PWM_SET,   FPGA_PWM_B_SETUP_USE_MODULATOR, 0);
                  FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_TRANSFMOD, 0);
                  FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_EN_CHARGERMODE, 0);
                  bDisModProcess = TRUE;
              }
              else if(sMh_PlcAdvancedWorks.flags.b.bEnableTransfMod)
              {
                  FPGA_REG16_SETBIT(FPGA_PWM_SET,   FPGA_PWM_B_SETUP_USE_MODULATOR, 0);
                  FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_TRANSFMOD, 1);
                  FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_EN_CHARGERMODE, 0);
                  bDisModProcess = TRUE;
              }
              else
              {
                  FPGA_REG16_SETBIT(FPGA_PWM_SET,   FPGA_PWM_B_SETUP_USE_MODULATOR, 1);
                  FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_TRANSFMOD, 0);
                  FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_EN_CHARGERMODE, 0);
                  bDisModProcess = FALSE;
              }

                  // then enable power and lock bridge options
              FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_SETUP_ENABLE, 1);

                  // skip optional updates to avoid peak processing time
              bSkipUpdates = TRUE;
          }
          else
          {   // Disable Power Command
              FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_SETUP_ENABLE, 0);
              bDisModProcess = FALSE;
          }

          sMh_MotorDataOut.sPowerStageSts.b.bReferenceEnabled = FALSE ;
      }

      sMh_MotorDataOut.sPowerStageSts.b.bReferenceEnabled = sMh_MotorDataIn.psPStageCtrl->b.bReferenceEnable ;

          // disabilita gestione frenatura, la riabilitazione solo su bPowerEnabled
      if(sMotorHandlerRun.sPStageStatusSet.b.bDisableBrakeDrive!=sMh_MotorDataIn.psPStageCtrl->b.bDisableBrakeDrive)
          if(sMh_MotorDataIn.psPStageCtrl->b.bDisableBrakeDrive)
              FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_SETUP_BRAKE_ENABLE, 0);

          // check backemf calcs
      if(sMh_MotorDataIn.psPStageCtrl->b.bBackEMFDataEnable)
          sMotorHandlerRun.flags.b.bBackEMFDataEnable=TRUE;

      if(sMh_MotorDataIn.psPStageCtrl->b.bBackEMFATanEnable)
      {
          sMotorHandlerRun.flags.b.bBackEMFDataEnable=TRUE;
          sMotorHandlerRun.flags.b.bBackEMFATanEnable=TRUE;
      }

      memcpy(&sMotorHandlerRun.sPStageStatusSet.b, &sMh_MotorDataIn.psPStageCtrl->b, sizeof(MH_POWERSTAGECONTROL));
    }
        // if calibration in progress, abort it and wait before enabling power, as analog section
        // must be re-programmed
    else if(!sMotorHandlerRun.flags.b.bCurrentCalibrFirstRun)
      sMotorHandlerRun.flags.b.bCurrentCalibrAbort=TRUE;
  }

  /* setup energy protection status */
  FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_ENERGYPROT, sMotorHandlerRun.flags.b.bEnergyProtEnabled);

  /* set phase offsets if enabled */
  if(sMh_PlcAdvancedWorks.flags.b.bEnablePhaseShift)
  {
      FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_PHASESHIFT, 1);
      FPGA_PWM_PHV_OFFSET = sMh_PlcAdvancedWorks.uwPwmOffsetV;
      FPGA_PWM_PHW_OFFSET = sMh_PlcAdvancedWorks.uwPwmOffsetW;
  }
  else
      FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_PHASESHIFT, 0);

  /* setup output disable on change flag */
  FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_DISONCHANGE, sMh_PlcAdvancedWorks.flags.b.bDisableOnChange);

  /* check for update of PWM frequency, high priority on gains update */
  if(!bSkipUpdates)
  {
    if(sMotorHandlerRun.flags.b.bUpdatePwmFrequency)
    {
      UBYTE ubActFreq = Mh_GetActualPwmFrequency();

          // if same or going from high to lo frequency update gains
      if(ubActFreq>=sMotorHandlerRun.ubSelFrequency)
          bUpdILoop=TRUE;

          // if same then terminate
      if(ubActFreq==sMotorHandlerRun.ubSelFrequency)
      {
          sMotorHandlerRun.flags.b.bUpdatePwmFrequency=FALSE;
          sMotorHandlerRun.flags.b.bUpdateFpgaGains=FALSE;
      }
      else
      {      // otherwise setup new frequency
          assert(sMotorHandlerRun.ubSelFrequency<=FPGA_PWM_FREQ_64KHZ);

          FPGA_PWM_SETEX = (FPGA_PWM_SETEX & ~FPGA_PWM_B_SETEX_EN_FREQSEL_MASK) |
                           (sMotorHandlerRun.ubSelFrequency << FPGA_PWM_B_SETEX_EN_FREQSEL_BASE);
      }
    }
    /* check if gains has to be updated */
    else if (sMotorHandlerRun.flags.b.bUpdateFpgaGains )
    {
      bUpdILoop = TRUE;
      sMotorHandlerRun.flags.b.bUpdateFpgaGains = FALSE ;
    }
    /* if no gains and/or pwm freq updated then check for delayed filters update */
    else
      bGenFlag = sMotorHandlerRun.flags.b.bUpdateFilters;
  }

  /* Calculate limits merging all limits with user fast limits */
  if(sMh_MotorDataUsrSetIdLimit.slMax > sMotorHandlerRun.sIdPartialLimit.slMax)
    sMh_MotorDataOut.sIdLimit.slMax = sMotorHandlerRun.sIdPartialLimit.slMax;
  else if(sMh_MotorDataUsrSetIdLimit.slMax > 0l)
    sMh_MotorDataOut.sIdLimit.slMax = sMh_MotorDataUsrSetIdLimit.slMax;
  else
    sMh_MotorDataOut.sIdLimit.slMax = 0l;

  if(sMh_MotorDataUsrSetIdLimit.slMin < sMotorHandlerRun.sIdPartialLimit.slMin)
    sMh_MotorDataOut.sIdLimit.slMin = sMotorHandlerRun.sIdPartialLimit.slMin;
  else if(sMh_MotorDataUsrSetIdLimit.slMin < 0l)
    sMh_MotorDataOut.sIdLimit.slMin = sMh_MotorDataUsrSetIdLimit.slMin;
  else
    sMh_MotorDataOut.sIdLimit.slMin = 0l;

  if(sMh_MotorDataUsrSetIqLimit.slMax > sMotorHandlerRun.sIqPartialLimit.slMax)
    sMh_MotorDataOut.sIqLimit.slMax = sMotorHandlerRun.sIqPartialLimit.slMax;
  else if(sMh_MotorDataUsrSetIqLimit.slMax > 0l)
    sMh_MotorDataOut.sIqLimit.slMax = sMh_MotorDataUsrSetIqLimit.slMax;
  else
    sMh_MotorDataOut.sIqLimit.slMax = 0l;

  if(sMh_MotorDataUsrSetIqLimit.slMin < sMotorHandlerRun.sIqPartialLimit.slMin)
    sMh_MotorDataOut.sIqLimit.slMin = sMotorHandlerRun.sIqPartialLimit.slMin;
  else if(sMh_MotorDataUsrSetIqLimit.slMin < 0l)
    sMh_MotorDataOut.sIqLimit.slMin = sMh_MotorDataUsrSetIqLimit.slMin;
  else
    sMh_MotorDataOut.sIqLimit.slMin = 0l;

  if(sMh_MotorDataOut.sPowerStageSts.b.bReferenceEnabled && !bDisModProcess)
  { /* from SpaceSpeed control loop or Torque Reference */
	sMh_MotorDataOut.sI2Fpga.slIdRef = sMh_MotorDataIn.psIRef2Use->slIdRef ;
    sMh_MotorDataOut.sI2Fpga.slIqRef = sMh_MotorDataIn.psIRef2Use->slIqRef + sMh_PlcAdvancedWorks.sIOffset.slIqRef;

    /* Id Limit */
    if(sMh_MotorDataOut.sI2Fpga.slIdRef >= sMh_MotorDataOut.sIdLimit.slMax)
    {
      sMh_MotorDataOut.sI2Fpga.slIdRef = sMh_MotorDataOut.sIdLimit.slMax ;
      sMh_MotorDataOut.sCurrLimFlags.b.bIdMax = TRUE ;
    }
    else
      sMh_MotorDataOut.sCurrLimFlags.b.bIdMax = FALSE ;

    if(sMh_MotorDataOut.sI2Fpga.slIdRef <= sMh_MotorDataOut.sIdLimit.slMin)
    {
      sMh_MotorDataOut.sI2Fpga.slIdRef = sMh_MotorDataOut.sIdLimit.slMin ;
      sMh_MotorDataOut.sCurrLimFlags.b.bIdMin = TRUE ;
    }
    else
      sMh_MotorDataOut.sCurrLimFlags.b.bIdMin = FALSE ;

    /* Iq Limit */
    if(sMh_MotorDataOut.sI2Fpga.slIqRef >= sMh_MotorDataOut.sIqLimit.slMax)
    {
      sMh_MotorDataOut.sI2Fpga.slIqRef = sMh_MotorDataOut.sIqLimit.slMax ;
      sMh_MotorDataOut.sCurrLimFlags.b.bIqMax = TRUE ;
    }
    else
      sMh_MotorDataOut.sCurrLimFlags.b.bIqMax = FALSE ;

    if(sMh_MotorDataOut.sI2Fpga.slIqRef <= sMh_MotorDataOut.sIqLimit.slMin)
    {
      sMh_MotorDataOut.sI2Fpga.slIqRef = sMh_MotorDataOut.sIqLimit.slMin ;
      sMh_MotorDataOut.sCurrLimFlags.b.bIqMin = TRUE ;
    }
    else
      sMh_MotorDataOut.sCurrLimFlags.b.bIqMin = FALSE ;
  }
  else
  {
    sMh_MotorDataOut.sI2Fpga.slIdRef  = 0L ;
    sMh_MotorDataOut.sI2Fpga.slIqRef  = 0L ;

    sMh_MotorDataOut.sCurrLimFlags.b.bIdMin = FALSE ;
    sMh_MotorDataOut.sCurrLimFlags.b.bIdMax = FALSE ;
    sMh_MotorDataOut.sCurrLimFlags.b.bIqMin = FALSE ;
    sMh_MotorDataOut.sCurrLimFlags.b.bIqMax = FALSE ;
    if( sLoc_Mh_PlcAdvancedWorks.uwTransfModValue != sMh_PlcAdvancedWorks.uwTransfModValue )
      FPGA_PWM_MODVAL = sMh_PlcAdvancedWorks.uwTransfModValue;
    if( sLoc_Mh_PlcAdvancedWorks.uwTransfModValue_V != sMh_PlcAdvancedWorks.uwTransfModValue_V )
      FPGA_PWM_PHV_MODVAL = sMh_PlcAdvancedWorks.uwTransfModValue_V ;
    if( sLoc_Mh_PlcAdvancedWorks.uwTransfModValue_W != sMh_PlcAdvancedWorks.uwTransfModValue_W )
      FPGA_PWM_PHW_MODVAL = sMh_PlcAdvancedWorks.uwTransfModValue_W ;

    // AlmaPS: allow only IGBT direct control
    if (sMotorHandlerRun.flags.b.bAlmaPS)
        sMh_PlcAdvancedWorks.flags.b.bSelEnOutEnable = TRUE ;

    if( sLoc_Mh_PlcAdvancedWorks.flags.b.bSelEnOutEnable != sMh_PlcAdvancedWorks.flags.b.bSelEnOutEnable )
      FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_SELDISOUT, sMh_PlcAdvancedWorks.flags.b.bSelEnOutEnable );
    if( sLoc_Mh_PlcAdvancedWorks.uwSelEnOutMask != sMh_PlcAdvancedWorks.uwSelEnOutMask )
      FPGA_PWM_BRIDGE_LAYOUT = (FPGA_PWM_BRIDGE_LAYOUT&0x00FF) | ((sMh_PlcAdvancedWorks.uwSelEnOutMask^0x00ff)<<8);
  }

  /* deflux if enabled */
  if(sMotorHandlerRun.flags.b.bDefluxEnabled)
  {
    if(sMotorHandlerRun.flags.b.bDflxWithIqFltRef)
      Dflx8KHz(sMh_MotorDataOut.sPowerStageSts.b.bReferenceEnabled, &sMh_MotorDataOut.sI2Fpga.slIdRef, &sMh_MotorDataOut.slIqFltRef) ; // uso la IqRefFiltered (introduco un ciclo di ritardo)
    else
      Dflx8KHz(sMh_MotorDataOut.sPowerStageSts.b.bReferenceEnabled, &sMh_MotorDataOut.sI2Fpga.slIdRef, &sMh_MotorDataOut.sI2Fpga.slIqRef) ;
  }

  /* Conversion from Internal Unit to FPGA Unit and write to the FPGA */
  sMh_MotorDataOut.sDSPOut.swId = (SWORD)(UMCONV_CONVERT_32TO16(&sInternal2Fpga_I_EQ_RMS, sMh_MotorDataOut.sI2Fpga.slIdRef)) ; /* 1e-4A */
  sMh_MotorDataOut.sDSPOut.swIq   = (SWORD)(UMCONV_CONVERT_32TO16(&sInternal2Fpga_I_EQ_RMS, sMh_MotorDataOut.sI2Fpga.slIqRef)) ; /* 1e-4A */

  // keep for past compatibility
  sMh_MotorDataOut.sDSPOut.swESin = Sin16(0x4000 + *sMh_MotorDataIn.puwElecAngle) ;  // $TEST$ da capire la costante...
  sMh_MotorDataOut.sDSPOut.swECos = Cos16(0x4000 + *sMh_MotorDataIn.puwElecAngle) ;

  // sin cos FPGA generation
  FPGA_SCGEN_ANGLEBASE = *sMh_MotorDataIn.puwElecAngle;
  if(sEm_EncMngrParam.uwMainRelSel==ENCODER_TYPE_REL_BACK_EMF)
  {
    swAngleIncConst = (SWORD)_sint32_scale_16(4294967,sGlbMotorParameters.uwPoleNumbers);
    swAngleIncConst = (SWORD)_sint32_scale_16(_sint32_scale_16(*sMh_MotorDataIn.pslFilteredSpeed,1), swAngleIncConst);
  }
  else
  {
    swAngleIncConst = _sint16_mpy_16_16(*sMh_MotorDataIn.pswElecSpeed, 524);
  }
  if(swAngleIncConst!=0)
    FPGA_SCGEN_ANGLEINC = swAngleIncConst;

  /* Check for filters update */
  if(bGenFlag)
  {
    UWORD uwCt;

    /* enable config mode */
    FPGA_IQFLT_SET = FPGA_IQFLT_MASK_FLT_CFG;

    /* select filter to be modified */
    FPGA_IQFLT_SET = FPGA_IQFLT_MASK_FLT_CFG | (sMotorHandlerRun.ubFltSelection&FPGA_IQFLT_MASK_FLT_SEL);

    /* first is state reset */
    FPGA_IQFLT_IQREF = sMh_MotorDataOut.sDSPOut.swIq;

    /* then constants */
    for(uwCt=0;uwCt<5;uwCt++)
      FPGA_IQFLT_IQREF = sMotorHandlerRun.swFltConst[uwCt];

    /* select number of valid filters and back to run mode */
    FPGA_IQFLT_SET = sMotorHandlerRun.ubFltNumber&FPGA_IQFLT_MASK_FLT_SEL;

    sMotorHandlerRun.flags.b.bUpdateFilters = FALSE;
  }
  /* if here then do motor phases check, if not disabled and if power enabled */
  else if(!sMh_MotorDataParam.flags.b.bDisPhasesCheck && sMh_MotorDataOut.sPowerStageSts.b.bFullyActive &&
          !sMotorHandlerRun.sErrorSent.b.bMissingMotorPhase)
  {
    bGenFlag = FALSE;

    /* if below minimum speed */
    if(labs(*sMh_MotorDataIn.pslFilteredSpeed)<sMotorHandlerRun.slAbsSpeedLowerLimit)
    {
      /* check currents references saturation against near-to-zero feedbacks */
      if(sMh_MotorDataOut.sCurrLimFlags.b.bIdMin || sMh_MotorDataOut.sCurrLimFlags.b.bIdMax)
        if(labs(sMh_MotorDataOut.slIdFb)<sMotorHandlerRun.slAbsCurrLowerLimit && sMh_MotorDataOut.sI2Fpga.slIdRef!=0l)
          bGenFlag = TRUE;

      if(sMh_MotorDataOut.sCurrLimFlags.b.bIqMin || sMh_MotorDataOut.sCurrLimFlags.b.bIqMax)
        if(labs(sMh_MotorDataOut.slIqFb)<sMotorHandlerRun.slAbsCurrLowerLimit && sMh_MotorDataOut.sI2Fpga.slIqRef!=0l)
          bGenFlag = TRUE;
    }

    if(bGenFlag)
    {
      if(++sMotorHandlerRun.uwMMPCnt>=MMP_FILTER)
      {
        sMotorHandlerRun.sErrorSent.b.bMissingMotorPhase = TRUE ;
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_MOTOR_FAIL, SYSTEMALARMS_SUBCODE_MT_MISSINGMOTORPHASE, TRUE) ;
        (*sMh_MotorDataIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
      }
    }
    else
      if(sMotorHandlerRun.uwMMPCnt>0)
        sMotorHandlerRun.uwMMPCnt--;
  }

  if(!sMotorHandlerRun.flags.b.bDSPCustomCode)
  {
      // if hook defined let it modify input data
    if(sPlcDSPHookEntryPoint)
      (*sPlcDSPHookEntryPoint)();

      // set queue mode for update
    FPGA_CPUH_RGUPD_SET_QUEUE = TRUE;

    if(bUpdILoop)
    {
      FPGA_CPUH_URAM_WR_SW(FPGAIR_P_KI,       sMh_MotorDataOut.sDSPPar.swKi);
      FPGA_CPUH_URAM_WR_SW(FPGAIR_P_KP,       sMh_MotorDataOut.sDSPPar.swKp);
      FPGA_CPUH_URAM_WR_SW(FPGAIR_P_OUTSHIFT, sMh_MotorDataOut.sDSPPar.swIOutShift);
      FPGA_CPUH_URAM_WR_SW(FPGAIR_P_OUTLIM,   sMh_MotorDataOut.sDSPPar.uwModOutLimit);
    }

    FPGA_CPUH_URAM_WR_SW(FPGAIR_IREF_D, sMh_MotorDataOut.sDSPOut.swId);

    if (sMotorHandlerRun.flags.b.bDSPAdvance)
    {  // **************** VdcBus management for DSP modulator ****************
	  if(sMotorHandlerRun.swUsrVdcSet == USRVDC_SET_FIX) // it is set ONLY at boot (Mh_MotorDataFromFpgaInit)
      {  // VdcBus value from fw/plc @8kHz
        if (sMh_PlcAdvancedWorks.swVdcbusSet <= 0)
    	{  // Vdc < 0 not allowed
    	  sMotorHandlerRun.swUsrVdcVal = (SWORD)(UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK,(SLONG)31457280)) ; // set 48.0V (*65536) as default
    	}
    	else
    	{  // update value
    	  sMotorHandlerRun.swUsrVdcVal = (SWORD)(UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK,(SLONG)sMh_PlcAdvancedWorks.swVdcbusSet<<16)) ;
    	}
      }
      else
      {  // VdcBus value @1MHz from ADC: keep user value to zero since it is not used and can create error in DSP
         sMotorHandlerRun.swUsrVdcVal = 0 ;
      }
      // **************** VdcBus management for DSP modulator ****************

      // *************** Integral status management for DSP PI ***************
      if (sMotorHandlerRun.flags.b.bDspIntegralSet)
      {  // set a specific value
         sMotorHandlerRun.swDspIntegralVal_D = (SWORD)(UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK,(SLONG)sMh_PlcAdvancedWorks.swDspIntegralVal_D<<16)) ;
         sMotorHandlerRun.swDspIntegralVal_Q = (SWORD)(UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK,(SLONG)sMh_PlcAdvancedWorks.swDspIntegralVal_Q<<16)) ;
      }
      else
      {  // keep integral resetted
         sMotorHandlerRun.swDspIntegralVal_D = 0 ;
         sMotorHandlerRun.swDspIntegralVal_Q = 0 ;
      }
	  // *************** Integral status management for DSP PI ***************

	  FPGA_CPUH_URAM_WR_SW(FPGAIR_P_VDC_SETVAL, sMotorHandlerRun.swUsrVdcVal); // FPGAIR_P_VDC_SETVAL used only in DspAdvance

	  // DSP integral status user initialization
	  FPGA_CPUH_URAM_WR_SW(FPGAIR_P_INTSTS_D, sMotorHandlerRun.swDspIntegralVal_D); // FPGAIR_P_INTSTS_VAL_D used only in DspAdvance
	  FPGA_CPUH_URAM_WR_SW(FPGAIR_P_INTSTS_Q, sMotorHandlerRun.swDspIntegralVal_Q); // FPGAIR_P_INTSTS_VAL_Q used only in DspAdvance

      // FPGA filter for  Vd, Vq, IdFb, IqFb
      FPGA_CPUH_URAM_WR_SW(FPGAIR_P_CORR_VD, -sMotorHandlerRun.swVdOut);
      FPGA_CPUH_URAM_WR_SW(FPGAIR_P_CORR_VQ, -sMotorHandlerRun.swVqOut);
      FPGA_CPUH_URAM_WR_SW(FPGAIR_P_CORR_ID, -sMotorHandlerRun.swIdFb);
      FPGA_CPUH_URAM_WR_SW(FPGAIR_P_CORR_IQ, -sMotorHandlerRun.swIqFb);
    }

    // revert to data flush mode
    FPGA_CPUH_RGUPD_SET_QUEUE = FALSE;
  }
    // if hook defined with DSP custom code
  else if(sPlcDSPHookEntryPoint)
  {
      // set queue mode for update
    FPGA_CPUH_RGUPD_SET_QUEUE = TRUE;

      // setup input data
    (*sPlcDSPHookEntryPoint)();

      // revert to data flush mode
    FPGA_CPUH_RGUPD_SET_QUEUE = FALSE;
  }

  FPGA_IQFLT_IQREF = sMh_MotorDataOut.sDSPOut.swIq;

  if( sLoc_Mh_PlcAdvancedWorks.uwDtCompSet != sMh_PlcAdvancedWorks.uwDtCompSet )
    FPGA_PWM_DTCOMP_SET = sMh_PlcAdvancedWorks.uwDtCompSet;

  sLoc_Mh_PlcAdvancedWorks = sMh_PlcAdvancedWorks;

  return TRUE ;
}

//***************************************************************************
// Decode actual switching frequency

UBYTE Mh_GetActualPwmFrequency(void)
{
    sMh_MotorDataOut.ubActualPwmFrequency=(UBYTE)((FPGA_PWM_STATUS&FPGA_PWM_STATUS_B_FREQSEL_MASK)>>FPGA_PWM_STATUS_B_FREQSEL_BASE);
    return sMh_MotorDataOut.ubActualPwmFrequency;
}

//***************************************************************************
// DSP halt and resume for PLC handling

void Mh_PlcDSPHalt(void)
{
    UWORD uwCt;

    FPGA_DSPH_SET_HALT = TRUE;
    for(uwCt=0;uwCt<100;uwCt++)
        if(FPGA_DSPH_INSRAML_HALTACK)
            break;
}

void Mh_PlcDSPResume(void)
{
    FPGA_DSPH_SET_HALT = FALSE;
}
