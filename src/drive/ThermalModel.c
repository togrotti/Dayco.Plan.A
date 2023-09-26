/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2008, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : ThernalModel.c                                             */
/* Author      : Cristiano Tognetti                                         */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "drive\AxM-E-Defines.h"
#include "system\Os.h"
#include "system\SysAppGlobals.h"

#include "common\UnitMeasureConversion.h" /* needed to convert values to/from FPGA */
#include "common\TaskScheduler.h"

#include "drive\ThermalModel.h"
#include "common\MathFunctions.h"
#include "common\Int64Functions.h" /* to use 64bit operations (BiQuad) */

#ifdef _INFINEON_
#include "AlteraFpgaHandler.h" /* to manage the FAN */
#else
#include "fpga\FpgaHandler.h"
#endif // _infineon_

#include "system\SystemAlarms.h"  /* to manage Errors */
#include "system\SysLogManagement.h"

#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "system\SysAppDataCodes.h"
#include "common\ParamStorageManagement.h"
#include "drive\HardwareUnitID.h"

#include "drive\MotorHandler.h"

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

/* ================================ #define ================================ */
// Avoids warning C47: unreferenced parameter
//
//#pragma warning disable = 47

/* ================================ #define ================================ */
#define SLOWTASK_STACKSIZE          (OS_DEFAULTUSRSTATICSTACK+0x0090)

#define ZEROCELSIUSINKELVIN         273.15
#define TWENTYFIVECELSIUSINKELVIN   (ZEROCELSIUSINKELVIN + 25.0)

#ifdef _INFINEON_
	#define DACRESOLUTION           (1024.0)
	#define DACFULLSCALE            (DACRESOLUTION * 32.0) // x32 to gain resolution
#else
	#define DACRESOLUTION           (65536.0)
	#define DACFULLSCALE            (DACRESOLUTION) // value is already 64 time more resoluted than Infineon
#endif // _infineon_
// * ********************************************************************************
// * NOTE!!! Even if NTC is connected @3.3V while Vref Micro is @3.0V:
// * - if AxM, NTC is never below 100 ohm 
// * - if AxW there is a diode in series to the NTV, so the voltage always cannot be higher the 3.3V - 0.6V = 2.8V 
// * so the DAC scale is compensated and I assume that the maximum value is always 3.3V
// * ********************************************************************************
#ifdef _INFINEON_
#define CORRECTION_FACTOR_XE        (1.1 * 8192.0) /* since Vref Micro is 3.0V and the signal from the NTC is whatever volt (8192.0 = 3.3V) */
#define NTC_IGBT_FAULT_THRESHOLD    (20 * 32) /* A2D count: if I read a value below, I think that IGBT NTC is 'broken'; x32 to gain a little resolution */

#else	// _infineon_

#if ((defined _HW_AXS_SAIETTA) || (defined _HW_AXS_CABI35KW))
// hw CABI or SAIETTA
#define CORRECTION_FACTOR_XE        (1.0 * 8192.0) /* since Vref Micro is included in the adc mapping formula */
#define NTC_IGBT_FAULT_THRESHOLD    (20) /* A2D count: if I read a value below, I think that IGBT NTC is 'broken' */
#elif (defined _HW_AXS_DAYCO22KW)
// hw Dayco
#define NTC_HW_VCC                  (5.0) // V: hw schematics
#define VREF_ZYNQ					(1.0) // V
#define NTC_ADC2RESISTOR			(NTC_HW_VCC / (3.0 * (VREF_ZYNQ / DACRESOLUTION))) // x 3 : hw schematics
#define CORRECTION_FACTOR_XE        (VREF_ZYNQ * 8192.0) // 8192 = Vref ZynQ
#define NTC_IGBT_FAULT_THRESHOLD    (100) /* A2D count: if I read a value below, I think that IGBT NTC is 'broken' */
#else
// hw AxN/AxN-DC
#define CORRECTION_FACTOR_XE        (3.3 * 8192.0) /* since Vref Micro is 1.0V and the signal from the NTC is whatever volt (8192.0 = 1.8V) */
#define NTC_IGBT_FAULT_THRESHOLD    (20) /* A2D count: if I read a value below, I think that IGBT NTC is 'broken' */
#endif

#endif // _infineon_

#if !defined(_HW_AXS_DAYCO22KW)
#define DACFULLSCALE_CORRECTED      (DACFULLSCALE * CORRECTION_FACTOR_XE)
#endif

#define V_NORMALIZATION             600.0  /* V: Voltage where losses are calculated */
#define TWO_MILLISECONDS            (2 * 8) /* = 125us * 16 */

#define BRAKE_OVERDRIVE             1.10    /* 110% */
//#define BRAKE_TIMECONST             0.002 /* Time constant of the filter of the power wasted when brake is on: add to Id Chip? */

#define MIN_UPDATE_PERIOD           (16 * 8) // msec
#define MAX_UPDATE_PERIOD           (20 * 8) // msec

#ifdef _INFINEON_
#define FAN_DRIVE_STATUS            (getfanstatus())
#define FAN_DRIVE_SET(x)            {setfanstatus(x);}
#else
#define FAN_DRIVE_STATUS            0x0000
#define FAN_DRIVE_SET(x)            0x0000
#endif // _infineon_

#ifdef _INFINEON_
#ifndef _HW_DC
#define FAN_FAULT                   ((BOOL)sThModRun.flags.b.bFltPol!=(BOOL)(sThModRun.flags.b.bDIF?P9_IN_P0:FPGA_GENERIC_IO_FAN_FAULT))
#else  // _hw_dc
#define FAN_FAULT                   ((BOOL)sThModRun.flags.b.bFltPol!=(BOOL)P9_IN_P4)
#endif  // _hw_dc
#else // _infineon_
#define FAN_FAULT                   (FALSE)
#endif // _infineon_

#define PHASE_U 1
#define PHASE_V 2
#define PHASE_W 3

#define CONDUCTION_MARGIN       1.0

#define COMMUTATION_MARGIN      1.4
#define COMMUTATION_VDC_OFFSET  3000   // [1/10V]
#define COMMUTATION_VDC_RATIO   3.636E-5

#define CURRENT_LIMTONOM_RATIO  0.40   // ratio to estimate nominal current from limit current threshold
#define CURRENT_LIMTONOM_HYST   0.05   // hysteresis

// Thermal model runs @16ms
#define BIQUAD_COEFF_A0     4L
#define BIQUAD_COEFF_A1     8L
#define BIQUAD_COEFF_A2     4L
#define BIQUAD_COEFF_B0     15566 
#define BIQUAD_COEFF_B1    -31934 
#define BIQUAD_COEFF_B2     16384 

#define MOTOR_I2T_IU2ARMS_SQUARED 0.00000001  // (i.u.->Arms)^2 = (1.0 / 10000.0)^2

#define FANDRIVE_PERIOD     40000
#define FANDRIVE_MINSPEED      10       // %

#define ALARM_DELTAT            50      // 1e-1 Celsius

#define MOTOR_TEMP_MIN      (-50.0)
#define MOTOR_TEMP_MAX      (2000.0)    

//#define __MOTOR_I2T_IN_SLOWTASK__

/*
Mask used if direct igbt control enabled
sMh_PlcAdvancedWorks.uwSelEnOutMask
bit 0 U_Lo: 0x0001
bit 1 U_Hi: 0x0002
bit 2 V_Lo: 0x0004
bit 3 V_Hi: 0x0008
bit 4 W_Lo: 0x0010
bit 5 W_Hi: 0x0020
TRUE = IGBT enabled; FALSE = IGBT disabled
*/
#define IGBT_ENABLEMASK_ULO 0x0001
#define IGBT_ENABLEMASK_UHI 0x0002
#define IGBT_ENABLEMASK_VLO 0x0004
#define IGBT_ENABLEMASK_VHI 0x0008
#define IGBT_ENABLEMASK_WLO 0x0010
#define IGBT_ENABLEMASK_WHI 0x0020

// OnBoard NTC
#ifndef _INFINEON_
#ifndef _HW_AXS
// AxN-ZynQ
#ifndef _HW_DC
#define CTRLB_NTC_B 3630.0
#define CTRLB_NTC_DIVIDER_RESISTOR 10000.0
#define CTRLB_NTC_25DEG_RESISTOR   10000.0 // 10kOhm
#else
// to be checked
#define CTRLB_NTC_B 3630.0
#define CTRLB_NTC_DIVIDER_RESISTOR 10000.0
#define CTRLB_NTC_25DEG_RESISTOR   10000.0 // 10kOhm
#endif // !_hw_dc

#define CTRLB_NTC_FULLSCALE        (3*65536) // Vcc = 3.0V, Vxadc: 1.0V - 65536,
                                             // Vin = Vcc * Rdivider/(Rdivider + Rntc + R20k)
#else // !_hw_axs

#if defined(_HW_AXS_SAIETTA)

#elif defined(_HW_AXS_CABI35KW)
  #define CTRLB_NTC_FULLSCALE        (3*65536) // VNTC = 3.0V, VXADC = 1.0V
  #define CTRLB_NTC_BETA 3630.0
  #define CTRLB_NTC_DIVIDER_RESISTOR 1500.0
  #define CTRLB_NTC_SERIES_RESISTOR  3000.0
  #define CTRLB_NTC_25DEG_RESISTOR   10000.0
  #define CABI35KW_NTC_CURR_A		 0.000301

#elif defined(_HW_AXS_DAYCO22KW)
  #define CTRLB_NTC_FULLSCALE        (3*65536) // VNTC = 3.0V, VXADC = 1.0V
  #define CTRLB_NTC_BETA 3630.0
  #define CTRLB_NTC_DIVIDER_RESISTOR 10000.0
  #define CTRLB_NTC_SERIES_RESISTOR  20000.0
  #define CTRLB_NTC_25DEG_RESISTOR   10000.0
#endif
#endif // _hw_axs
#endif // !_infineon_

/* ============================== structures =============================== */
typedef struct
{
  FLOAT  flRBrakeValue ;
  SWORD  swRBrakeMaxPower ; /* 1e-1 W */
  FLOAT  flNtcVDividerResistor    ; /* Resistor of NTC Voltage Divider    (ohm) */
  FLOAT  flNtc25DegreeResistance  ; /* NTC Resistance value at 25 Celsius (ohm) */
  FLOAT  flNtcMaterialConstant    ; /* NTC Material Specific Constant       (K) */
  FLOAT  flIgbtThCapacitance      ; /* IGBT Thermal Capacitance           (J/K) */
  FLOAT  flHsnkThCapacitance      ; /* HeatSink Thermal Capacitance       (J/K) */
  UWORD  uwSwitchingFrequency     ; /* PWM switching frequency (x 1000Hz) */
  SWORD  swIgbtMaximumTemp        ; /* IGBT Max Temperature         (1e-1 Celsius) */
  SWORD  swHsnkMaximumTemp        ; /* Heatsink/NTC Max Temperature (1e-1 Celsius) */

  // code optimization
  UWORD  uwNtc2JunctionThResist ;
  UWORD  uwHsnk2NtcThResCoolingON ;
  UWORD  uwHsnk2NtcThResCoolingOFF ;
#ifdef _HW_CT
  FLOAT  flBrdNtcVDividerResistor  ; /* Resistor of OnBoard NTC Voltage Divider    (ohm) */
  FLOAT  flBrdNtc25DegreeResistance; /* OnBoard NTC Resistance value at 25 Celsius (ohm) */
  FLOAT  flBrdNtcMaterialConstant  ; /* OnBoard NTC Material Specific Constant       (K) */
#endif
} THERMAL_MODEL_HWPARAM ;

typedef struct {
  UWORD  uwCnts ;
  UWORD  uwRBrakeON ; /* Number of servo when VdcBus > VBrakeHigh */
  SLONG  slDcBus  ; /* DC Bus Average Value     (1e-1V) */
  SLONG  slIuPkHi ; /* PhaseU Current Max value (1e-4A) */
  SLONG  slIvPkHi ; /* PhaseV Current Max value (1e-4A) */
  SLONG  slIwPkHi ; /* PhaseW Current Max value (1e-4A) */
  SLONG  slIuPkLo ; /* PhaseU Current Min value (1e-4A) */
  SLONG  slIvPkLo ; /* PhaseV Current Min value (1e-4A) */
  SLONG  slIwPkLo ; /* PhaseW Current Min value (1e-4A) */
} THERMAL_MODEL_MEDIUM_TASK_VARIABLES ;

typedef struct {
  UWORD  uwCnts ;
  ULONG  ulNtc      ; /* Value read from Power Module NTC via Adc    */
#ifndef _HW_DC
  UWORD  uwMotorTemp ; /* Value read from motor KTY via Adc */
#else
  ULONG  ulBr1Ntc;
  ULONG  ulBr2Ntc;
  UWORD  uwMotorPTC;
  UWORD  uwMotorKTY;
#endif
#ifdef _HW_CT
  ULONG  ulBrdNtc; /* Value read from On board NTC via Adc    */
#endif

#ifndef _INFINEON_
  FLOAT flSOnchipTemp ;
  UWORD uwSBoardTemp ;
#endif // _infineon_
} THERMAL_MODEL_SLOW_TASK_VARIABLES ;


typedef struct {
    SWORD swX_1 ;
    SWORD swX_2 ;
    SLONG slY_1 ;
    SLONG slY_2 ;
} THERMAL_MODEL_BIQUAD ;

typedef struct {
    union {
        struct {
            UWORD bIgbtLow : 1 ;
            UWORD b2Switch : 1 ;
        } b ;
        UWORD w ;
    } flags ;  

    UWORD uwVdc ;     // in
    SWORD swTNtc ;    // in
    UWORD uwCnts ;    // in
    	
	// parameters (code optimized)
    UWORD uwIgbtTimeConstant ;
    UWORD uwNtc2JunctionThResist ;	
    UWORD uwBiQuadNtc2JunctionThResist ;
    SLONG slPjTot ;	// Conduction and commutation TOTAL power losses
    FLOAT flVceSat ;
    FLOAT flCommutationLosses ;
    FLOAT flPreCommutationLosses ;
    SLONG slTjMaxFactor ; //addictional igbt losses factor due to IGBT temperature
} THERMAL_MODEL_LOSSES_IGBT_COMMON ;

typedef struct {
    THERMAL_MODEL_BIQUAD sBiQuadFilterStatus ; // in-out
    SWORD swTjIgbt ;     // in-out	
    SWORD swPjIgbt ;     // out
    SLONG slTjIgbt ;     // status variable for the Tj Filter
} THERMAL_MODEL_LOSSES_IGBT_SINGLE ;

typedef struct {
    // output filtro
    FLOAT flDeltaTemp_Iu ; 
    FLOAT flDeltaTemp_Iv ;    
    FLOAT flDeltaTemp_Iw ;

    // param
    FLOAT flMotINom2 ; // (Motor Inominal)^2 (A^2)
    FLOAT flMotINom2Inv ; // (1/(Motor Inominal)^2) (1/A^2)

    FLOAT flKNum ;
    FLOAT flKDenInv ;

    // variabili d'appoggio per I2T motore eseguito in puro background
    UWORD uwTimer_1 ; 
    ULONG ulMotorThermalConstant ;
} MOTOR_I2T ;


typedef struct
{
  /* Flags */
  union {
      struct {
        UWORD bTaskRunning : 1 ;
        UWORD bDIF         : 1 ;
        UWORD bAMEnable    : 1 ;
        UWORD bFanVSpeed   : 1 ;
        UWORD bFanFaultChk : 1 ;
        UWORD bDisFltReactOnLimit  : 1 ;
        UWORD bNoBrakeDisableOnFlt : 1 ;
        UWORD bNoFatalOnBrakeFlt   : 1 ;
        UWORD bBrakeDisabled       : 1 ;
        UWORD bNoCooling   : 1 ;
        UWORD bFltPol      : 1 ;
        UWORD bAlmaPS      : 1 ;
      } b ;
      UWORD w ;
  } flags ;


    union {
        struct {
            UWORD bNtcIgbtFail      : 1 ; // b.0
            UWORD bJunctionOverTemp : 1 ; // b.1
            UWORD bFanLocked        : 1 ; // b.2
            UWORD bHeatSinkOverTemp : 1 ; // b.3
            UWORD bBrakeAlwaysOn    : 1 ; // b.4
            UWORD bBrakeOverPower   : 1 ; // b.5
            UWORD bSlowTaskRunning  : 1 ; // b.6
            UWORD bSyncPointTimeOut : 1 ; // b.7
            UWORD bSlowTaskOverTime : 1 ; // b.8
            UWORD bMotorOverTemp    : 1 ; // b.9
            UWORD bMotorI2T         : 1 ; // b.10
            UWORD bOnChipOverTemp   : 1 ; // b.11
        } b ;
        UWORD w ;
    } sErrorSent ;

  THERMAL_MODEL_MEDIUM_TASK_VARIABLES sImageMediumTask ;  /* images of servo values for the medium (8ms) slowtask  */
  THERMAL_MODEL_SLOW_TASK_VARIABLES sImageSlowTask ; /* images of servo values for the slow (32ms) slowtask  */
  THERMAL_MODEL_HWPARAM HwParam ;  /* image of the parameters saved in flash/IdChip */

  UWORD  uwHsnk2NtcThRes2Use ; /* HeatSink/NTC Thermal Resistance to use in the model (1e-2 K/W) */

  /* Drive Current Limit when drive is "cold" (=boot value) and modulation = 8kHz-3switch, */
  SLONG slIdChipCurrLimit ;
  SLONG slIdChipCompCurrNomHiTh ;
  SLONG slIdChipCompCurrNomLoTh ;
  SLONG slDynamicOverCurrentDelta ;

  GLB_TORQUE_LIMIT sIdLocLim ; // variabile d'appoggio
  GLB_TORQUE_LIMIT sIqLocLim ;

  SWORD swCoolingTempOn  ;
  SWORD swCoolingTempOff ;
  SWORD swCoolingVSpdRatio ;

  SWORD swFanLockedCounter ; // counter for fan fault
  UWORD uwFanMinSpeed ;

  FLOAT flRBrakeThConst ; // R Brake thermal time constant
  FLOAT flRBrakeEnergy ;  // R Brake energy

  SWORD swILimitMaxTemp ; // = IgbtMaximumTemp - 5 Celsius = (TjMax - 15) - 5 = TjMax - 20 Celsius
  SWORD swILimitMinTemp ; // = TjMax - 30 Celsius
  SWORD swILimitDelta ;   // = ILimitMaxTemp - ILimitMinTemp = 10 Celsius
  SWORD swHsnkDeratingMaximumTemp ; // = swHsnkMaximumTemp - 10 Celsius (DisablePkPk = TRUE);  swHsnkMaximumTemp - 5 (DisablePkPk = TRUE)
  SLONG slTHeatSink ; // status variables for THeasink filter

  UWORD uwBiQuadNtc2JunctionThResist ; // NEW ID-CHIP PARAMETER!!!!!

  THERMAL_MODEL_LOSSES_IGBT_COMMON sLossesIgbtCommon ;
  THERMAL_MODEL_LOSSES_IGBT_SINGLE sLossesIgbtUHi ;
  THERMAL_MODEL_LOSSES_IGBT_SINGLE sLossesIgbtVHi ;
  THERMAL_MODEL_LOSSES_IGBT_SINGLE sLossesIgbtWHi ;
  THERMAL_MODEL_LOSSES_IGBT_SINGLE sLossesIgbtULo ;
  THERMAL_MODEL_LOSSES_IGBT_SINGLE sLossesIgbtVLo ;
  THERMAL_MODEL_LOSSES_IGBT_SINGLE sLossesIgbtWLo ;

  SLONG slTmp ; // temporary variable used all over the sw module
  FLOAT flTmp ; // temporary variable used all over the sw module

  SLONG slIdLocLimMax_1 ; // Dynamic OverCurrent

  MOTOR_I2T sMotorI2T ;
  FLOAT flDacFullScaleCorrected	;

  SLONG slTjMaxFiltSts ;

} THERMAL_MODEL_RUN ;

static OS_TASKHANDLE sMediumTaskHandle;

FLOAT flTm_PlcMotorTemp ; // to get motor temp from plc

TM_THERMAL_MODEL_INPUT  sTm_ThModIn  ;
TM_THERMAL_MODEL_OUTPUT sTm_ThModOut ;
TM_THERMAL_MODEL_PARAM  sTm_ThModParam ;

#ifdef _INFINEON_
static BOOL bdata bDisableBiQuad = TRUE;
static BOOL bdata bDisablePkPk   = FALSE ;
#else
static BOOL bDisableBiQuad = TRUE ;
static BOOL bDisablePkPk   = FALSE ;
static BOOL bDisableFwMotorTemp = FALSE ;
#endif //_infineon_

static THERMAL_MODEL_MEDIUM_TASK_VARIABLES sThModRun_sServoMediumTask ; /* values calculated in servo for the medium (8ms) slowtask */
static THERMAL_MODEL_SLOW_TASK_VARIABLES   sThModRun_sServoSlowTask ;   /* values calculated in servo for the slow  (32ms) slowtask */
static UMCONV_CONV16TO32 sTm_Fpga2Internal_I_PEAK ;

#ifdef _INFINEON_
static THERMAL_MODEL_RUN huge sThModRun ;
const TM_THERMAL_MODEL_PARAM huge sTm_ThModDefParam =
#else
THERMAL_MODEL_RUN sThModRun ;
const TM_THERMAL_MODEL_PARAM sTm_ThModDefParam =
#endif //_infineon_
{ // default
  0,      // RBrakeValue
  0,      // RBrakeMaxPower
  500,    // CoolingTempOn
  450,    // CoolingTempOff
  1500,   // MotorOverTemp
  0,      // RBrakeMaxEnergy
  1000,   // I2T max delta (zero for disabling)
  {0,0,0},// flags

#ifdef _INFINEON_
// --- infineon
#ifndef _HW_DC
  {-249.2615,0.950023,-0.0011033,0.0000009184},         // default KTY/PTC
#else
  {0.0,0.0,0.0,0.0},                                    // disable PTC
  {-143.318,0.336,-0.0001095,0.00000001754},            // default KTY
#endif // !_hw_dc
// --- infineon
#else
// --- ZynQ
#ifndef _HW_DC
/* equation for AxN
kty84-130/ptc y = 0.0000000000020242 * x^3 -0.0000002266862779 * x^2 + 0.0145951132245646 * x -173.1500378279620000
pt1000        y = 0.0000000000006432 * x^3 +0.0000000186797935 * x^2 + 0.0080646221797776 * x -251.6685783491390000
*/
  {-251.6685783491390000,0.0080646221797776,0.0000000186797935,0.0000000000006432}, // default PT1000
#else
#if defined (_HW_AXS_DAYCO22KW)
  {-236.439050966643,0.0092665866795635,0.0000000485814645,-0.0000000000002985}, // default PT1000
  {-236.439050966643,0.0092665866795635,0.0000000485814645,-0.0000000000002985}, // default PT1000
#else
  {0.0,0.0,0.0,0.0}, // disable PTC
  {0.0,0.0,0.0,0.0}, // disable KTY
#endif // _hw_axs_dayco22kw
#endif // !_hw_dc

  100.0, // default from Xilinx
  // --- ZynQ
#endif
} ;

/* =========================== global  variables =========================== */

/* =============================== functions =============================== */
static BOOL Tm_ThermalModel8KHz(void) ;
void Tm_ThermalModelMediumTask(void) ; /* min: 8ms, max: 12ms */
static void coolingtempsetup(void) ;
static void Tm_ThermalModelBkGd(void) ;
static void MotorTemperature(void) ;

static FLOAT Poly3Linearization(UWORD uwSample, HWPRM2_ANALOG_RANGE * sRange, FLOAT flK[4]);

static BOOL liveparcheck(void);

#ifdef _INFINEON_
static SWORD BiQuadFilter48Bit(SWORD swInputVal, THERMAL_MODEL_BIQUAD huge * psFilterStatus) ;
static SWORD MotorI2T_Delta(SLONG *slMotorI, FLOAT huge *pDeltaT_Filtered) ;
static void  IgbtLosses(THERMAL_MODEL_LOSSES_IGBT_COMMON huge * psLossesIgbtCommon, THERMAL_MODEL_LOSSES_IGBT_SINGLE huge * psLossesIgbtSingle, SLONG slIpk, SLONG slIpkIGBTsHigh2Switch) ;
#else
static SWORD BiQuadFilter48Bit(SWORD swInputVal, THERMAL_MODEL_BIQUAD * psFilterStatus) ;
static SWORD MotorI2T_Delta(SLONG *slMotorI, FLOAT *pDeltaT_Filtered) ;
static void  IgbtLosses(THERMAL_MODEL_LOSSES_IGBT_COMMON * psLossesIgbtCommon, THERMAL_MODEL_LOSSES_IGBT_SINGLE * psLossesIgbtSingle, SLONG slIpk, SLONG slIpkIGBTsHigh2Switch) ;
#endif // _infineon_

static void AdjIgbtLosses(void) ;
static void IgbtTNtc(void) ;
static void IgbtModuleLosses(void) ;
static void IgbtCurrentLimit(void) ;
static void DriveHeatsink(void) ;
static void BrakeResistor(void) ;
static void DynamicOverCurrent(void) ;
static void ThermalModelCurrentLimit(void) ;
static void AdaptModulation(void);
static void MotorI2T_Init(void) ;

static void MotorI2T_Runtime(void) ;

#ifdef _INFINEON_
static void digiffansetup(UBYTE ubMinSpeed) ;
static BOOL getfanstatus(void) ;
static void setfanstatus(UWORD uwRatio) ;
#endif // _infineon_

#ifdef _HW_CT
static void BrdTNtc(void) ;
#endif

#ifndef _INFINEON_
static void DiagnosticTemperatures(void) ;
#endif
/* ========================================================================= */
// Hardware option callback

#ifdef _APP_XC
void Tm_GetHwOpt(UWORD uwOpt, ULONG * pulHwOpt, ULONG * pulFPGAOpt)
{
    UWORD i;

#ifdef _HW_DC
    if((sGlbOptAvailable.ulHardwareOpt&HWUNITID_BRI_FULL_0)==0)
        return;

    for(i=0;i<4;i++)
        if(sTm_ThModParam.sflPTCCoeff[i]!=0.0)
            *pulHwOpt  |=HWUNITID_MOT_RLIN_PTC;

    for(i=0;i<4;i++)
        if(sTm_ThModParam.sflKTYCoeff[i]!=0.0)
            *pulHwOpt  |=HWUNITID_MOT_RLIN_KTY;

#else
    *pulHwOpt  |=HWUNITID_MOT_RLIN_KTY;
#endif

    *pulFPGAOpt|=0;
}
#endif

/* ========================================================================= */

BOOL Tm_ThermalModelInit(void)
{  
  SLONG slIdChipOverCurrent ;
  SLONG slIdChipCurrentLimit ;
  UWORD uwCoolingType ;
  HWPRMS_BRAKE_CIRCUIT_V2 sBrakeCircuit ;

#ifdef _INFINEON_
  HWPRMS_BRIDGE_THERMAL_MODEL huge * psThermalParam=NULL;
  HWPRM2_BRAKE_RESISTOR huge * psBrkRes=NULL;
  HWPRM2_BRIDGE huge * psBridgeParam=NULL;
#else
  HWPRMS_BRIDGE_THERMAL_MODEL * psThermalParam=NULL;
  HWPRM2_BRAKE_RESISTOR * psBrkRes=NULL;
  HWPRM2_BRIDGE * psBridgeParam=NULL;
#endif // _infineon_

#ifdef _HW_CT
  HWPRMS_BOARD_NTC huge * psBrdThermalParam=&sGlbControlBoardParameters.sStd.sBoardNtc;
#endif

#ifdef _HW_DC
    if((sGlbOptAvailable.ulHardwareOpt&HWUNITID_BRI_FULL_0)==0)
        return TRUE;
#endif

    // energy patch in case of standard AxM with 10W and 70/42/30Ohm resistors,
    // only if parameters was saved with release 1.3 or older
  if((sParMgmParamSignature.uwVersionMajor<=1) && (sParMgmParamSignature.uwVersionMinor<=3))
  {
    if((sTm_ThModParam.uwRBrakeMaxPower==100) && ( (sTm_ThModParam.uwRBrakeValue==700) || (sTm_ThModParam.uwRBrakeValue==420) || (sTm_ThModParam.uwRBrakeValue==300) ))
      sTm_ThModParam.ulRBrakeMaxEnergy = 400;  // 400 J
    else
      sTm_ThModParam.ulRBrakeMaxEnergy = 0;
  }

  if(!liveparcheck())
  {
    TaskSched_AddBackgroundTask((void (*)(void))&liveparcheck);
    return FALSE;
  }

  sThModRun.flags.b.bAlmaPS = FALSE ;
  sThModRun.sErrorSent.w = 0 ; /* clear errors */

  /* setup power board specifics */
  switch(sGlbPowerBoardParameters.sProductInfo.uwProductCode)
  { /* parameters: get them from Id-chip */
    case HWPRM_PCODE_POWER_BOARD_OLD_AXW_DC: /* without Id-Chip */
    case HWPRM_PCODE_POWER_BOARD_OLD_AXW_AC: /* without Id-Chip */
    case HWPRM_PCODE_POWER_BOARD_OLD_AXM_AC: /* without Id-Chip */
    case HWPRM_PCODE_POWER_BOARD_AXP_AC:
    case HWPRM_PCODE_POWER_BOARD_AXP_DS_AC:
    case HWPRM_PCODE_POWER_BOARD_AXM_AC:
    case HWPRM_PCODE_POWER_BOARD_AXM_AC_ALMAPS:
    case HWPRM_PCODE_POWER_BOARD_AXM_DS_AC:
    case HWPRM_PCODE_POWER_BOARD_AXW_AC:
    case HWPRM_PCODE_POWER_BOARD_AXW_DC:
#ifdef _INFINEON_
        if(sGlbPowerBoardParamDataCode==DATACODE_HW_POWER_BOARD_AXM_AXP_AC)
        {
            slIdChipCurrentLimit = sGlbPowerBoardParameters.sAxmAxpAC.sBridgeParams.sOperativeLimits.slCurrentLimit ;
            slIdChipOverCurrent  = sGlbPowerBoardParameters.sAxmAxpAC.sBridgeParams.sOperativeLimits.slOverCurrent  ;
            uwCoolingType = sGlbPowerBoardParameters.sAxmAxpAC.uwCoolingType ;
            sBrakeCircuit.uwResistorValue = sGlbPowerBoardParameters.sAxmAxpAC.sBrakeCircuit.uwResistorValue ;
            sBrakeCircuit.uwResistorPower = sGlbPowerBoardParameters.sAxmAxpAC.sBrakeCircuit.uwResistorPower ;

                // energy patch in case of standard AxM with 10W and 70/42/30Ohm resistors
            if(sBrakeCircuit.uwResistorPower==100 && (sBrakeCircuit.uwResistorValue==700 || sBrakeCircuit.uwResistorValue==420 || sBrakeCircuit.uwResistorValue==300 ))
                sBrakeCircuit.ulRBrakeMaxEnergy = 400;  // 400 J
            else
                sBrakeCircuit.ulRBrakeMaxEnergy = 0;

            psThermalParam = &sGlbPowerBoardParameters.sAxmAxpAC.sBridgeParams.sThermalModel ;
        }
        else
#endif // _infineon_
        {
            slIdChipCurrentLimit = sGlbPowerBoardParameters.sAxmAxpACV2.sBridgeParams.sOperativeLimits.slCurrentLimit ;
            slIdChipOverCurrent  = sGlbPowerBoardParameters.sAxmAxpACV2.sBridgeParams.sOperativeLimits.slOverCurrent  ;
            uwCoolingType = sGlbPowerBoardParameters.sAxmAxpACV2.uwCoolingType ;
            sBrakeCircuit = sGlbPowerBoardParameters.sAxmAxpACV2.sBrakeCircuit ;
            psThermalParam = &sGlbPowerBoardParameters.sAxmAxpACV2.sBridgeParams.sThermalModel ;
        }

#ifdef _INFINEON_
        // not yet supported
        assert(uwCoolingType != HWPRM_POWER_BOARD_COOLING_FAN_3WIRE_VSPEED);
#endif // _infineon_
        if(uwCoolingType == HWPRM_POWER_BOARD_COOLING_FAN_3WIRE_HIGOOD || uwCoolingType == HWPRM_POWER_BOARD_COOLING_FAN_3WIRE_VSPEED)
            sThModRun.flags.b.bFanFaultChk = TRUE;

        if(uwCoolingType == HWPRM_POWER_BOARD_COOLING_WATER)
            sThModRun.flags.b.bNoCooling = TRUE;

        if (sGlbPowerBoardParameters.sProductInfo.uwProductCode == HWPRM_PCODE_POWER_BOARD_AXM_AC_ALMAPS)
            sThModRun.flags.b.bAlmaPS = TRUE ; // AxM2 used as ALMA PowerSupply

        break ;
    
    case HWPRM_PCODE_POWER_BOARD_DIF_CI:
    case HWPRM_PCODE_POWER_BOARD_DIF_PI:
    case HWPRM_PCODE_POWER_BOARD_DIF_BC:
    case HWPRM_PCODE_POWER_BOARD_DIF_BL:
        slIdChipCurrentLimit = sGlbPowerBoardParameters.sDif.sBridgeParams.sOperativeLimits.slCurrentLimit ;
        slIdChipOverCurrent  = sGlbPowerBoardParameters.sDif.sBridgeParams.sOperativeLimits.slOverCurrent  ;
        uwCoolingType = sGlbPowerBoardParameters.sDif.uwCoolingType ;
        sBrakeCircuit = sGlbPowerBoardParameters.sDif.sBrakeCircuit ;
        psThermalParam = &sGlbPowerBoardParameters.sDif.sBridgeParams.sThermalModel ;
        sThModRun.flags.b.bDIF = TRUE;

#ifdef _INFINEON_        
        if(uwCoolingType == HWPRM_POWER_BOARD_COOLING_FAN_3WIRE_VSPEED)
        {
          digiffansetup(FANDRIVE_MINSPEED);
          sThModRun.flags.b.bFanVSpeed = TRUE;
        }
        else
          P9_IOCR06 = 0x0080; // fan drive
#endif // _infineon_
 
        if(uwCoolingType == HWPRM_POWER_BOARD_COOLING_FAN_3WIRE_HIGOOD || uwCoolingType == HWPRM_POWER_BOARD_COOLING_FAN_3WIRE_VSPEED)
            sThModRun.flags.b.bFanFaultChk = TRUE;

        if(uwCoolingType == HWPRM_POWER_BOARD_COOLING_WATER)
            sThModRun.flags.b.bNoCooling = TRUE;

        break;

    case HWPRM_PCODE_POWER_BOARD_UNIVERSAL:
        slIdChipCurrentLimit = (SLONG)(sGlbPowerBoardParameters.sUniv.sBrdgFeat.fCurrentLimit * 10000.0) ;
        slIdChipOverCurrent  = (SLONG)(sGlbPowerBoardParameters.sUniv.sBrdgFeat.fOverCurrent  * 10000.0) ;
#ifdef _HW_DC
        if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge1Valid)
        {
            // if parallel bridge make all limits/calculations as aggregate,
            // effectively doubling or tripling all current limits
            if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge2Valid)
            {
                slIdChipCurrentLimit *= 3;
                slIdChipOverCurrent  *= 3;
            }
            else
            {
                slIdChipCurrentLimit *= 2;
                slIdChipOverCurrent  *= 2;
            }
        }
#endif
        sThModRun.flags.b.bDIF = TRUE;

        /* validate cooling options */
        if(sGlbPowerBoardParameters.sUniv.sCoolFeat.sOpt.ubMounted)
        {
            if(!sGlbPowerBoardParameters.sUniv.sHwFeat.sOpt.ubCoolDrive)
                return FALSE;

            if(sGlbPowerBoardParameters.sUniv.sCoolFeat.sOpt.ubDriveType!=HWPRM2_FANCOOL_DRV_ONOFF && !sGlbPowerBoardParameters.sUniv.sHwFeat.sOpt.ubCoolPWM)
                return FALSE;
            if(sGlbPowerBoardParameters.sUniv.sCoolFeat.sOpt.ubFeedback!=HWPRM2_FANCOOL_FB_NONE && 
                sGlbPowerBoardParameters.sUniv.sCoolFeat.sOpt.ubFeedback!=HWPRM2_FANCOOL_FB_FAULT)
                return FALSE;
            if(sGlbPowerBoardParameters.sUniv.sCoolFeat.sOpt.ubDriveType!=HWPRM2_FANCOOL_DRV_ONOFF && 
                sGlbPowerBoardParameters.sUniv.sCoolFeat.sOpt.ubDriveType!=HWPRM2_FANCOOL_DRV_PWM_2KHZ)
                return FALSE;

#ifdef _INFINEON_
            if(sGlbPowerBoardParameters.sUniv.sCoolFeat.sOpt.ubDriveType==HWPRM2_FANCOOL_DRV_PWM_2KHZ)
            {
                digiffansetup(sGlbPowerBoardParameters.sUniv.sCoolFeat.ubMinSpeed);
                sThModRun.flags.b.bFanVSpeed = TRUE;
            }
            else
                P9_IOCR06 = 0x0080; // fan drive
#endif // _infineon_

            if(sGlbPowerBoardParameters.sUniv.sCoolFeat.sOpt.ubFeedback==HWPRM2_FANCOOL_FB_FAULT)
            {
                sThModRun.flags.b.bFltPol=(BOOL)sGlbPowerBoardParameters.sUniv.sHwFeat.sOpt.ubCoolFltPol == (BOOL)sGlbPowerBoardParameters.sUniv.sCoolFeat.sOpt.ubFbPolarity;
                sThModRun.flags.b.bFanFaultChk = TRUE;
            }
        }
        else
            sThModRun.flags.b.bNoCooling = TRUE;

        psBrkRes=&sGlbPowerBoardParameters.sUniv.sBrkRes;
        psBridgeParam=&sGlbPowerBoardParameters.sUniv.sBrdgFeat;
        
        break;

    default:
        assert(FALSE);
        return FALSE;    // just avoid warning C184
  }     

  UmConv_Init_16to32(&sTm_Fpga2Internal_I_PEAK, sMh_PlcUMRatios.flRatioIpk) ;

  /* Current Limits: Max Current to use is the Drive slOverCurrent, NOT MaxCurrent */
  sThModRun.slIdChipCurrLimit = slIdChipCurrentLimit ; /* 1e-4A */
  sThModRun.slIdChipCompCurrNomHiTh = (SLONG)((FLOAT)slIdChipCurrentLimit*(CURRENT_LIMTONOM_RATIO+CURRENT_LIMTONOM_HYST));
  sThModRun.slIdChipCompCurrNomLoTh = (SLONG)((FLOAT)slIdChipCurrentLimit*(CURRENT_LIMTONOM_RATIO-CURRENT_LIMTONOM_HYST));
  sThModRun.slDynamicOverCurrentDelta = slIdChipOverCurrent - sThModRun.slIdChipCurrLimit ;

  /* Fan T_ON and T_OFF*/
  coolingtempsetup();

  /* fillup thermal model parameters */
  sThModRun.flDacFullScaleCorrected = 0.0 ;

  if(psThermalParam)
  {  
    sTm_ThModOut.sOutValue.swIgbtTjMax          = (SWORD)((FLOAT)psThermalParam->uwModuleMaxTJunction * 10.0) ; /* Junction Max Temperature, 1e-1 Celsius */
    sThModRun.HwParam.flNtcVDividerResistor     = (FLOAT)psThermalParam->uwNtcVDividerResistor   * 10.0    ; /* Resistor of NTC Voltage Divider    (ohm) */  
    sThModRun.HwParam.flNtc25DegreeResistance   = (FLOAT)psThermalParam->uwNtc25DegreeResistance * 10.0    ; /* NTC Resistance value at 25 Celsius (ohm) */
    sThModRun.HwParam.flNtcMaterialConstant     = (FLOAT)psThermalParam->uwNtcMaterialConstant   * 1.0     ; /* NTC Material Specific Constant       (K) */
    sThModRun.HwParam.swIgbtMaximumTemp         = (SWORD)psThermalParam->uwIgbtMaximumTemp                 ; /* IGBT Max Temperature           (1e-1 Celsius) */
    sThModRun.HwParam.swHsnkMaximumTemp         = (SWORD)psThermalParam->uwHsnkMaximumTemp                 ; /* Heatsink/NTC Max Temperature   (1e-1 Celsius) */
    
    sThModRun.HwParam.uwNtc2JunctionThResist    = psThermalParam->uwNtc2JunctionThResist ; 
    sThModRun.uwBiQuadNtc2JunctionThResist      = 40;
    sThModRun.HwParam.uwHsnk2NtcThResCoolingON  = psThermalParam->uwHsnk2NtcThResCoolingON ;
    sThModRun.HwParam.uwHsnk2NtcThResCoolingOFF = psThermalParam->uwHsnk2NtcThResCoolingOFF ;
    
    // *********************************************************************************
    // ** CODE OPTIMIZATION (to avoid to make  moltiplications during task execution) **
    // ** The optimizations are calculated with thermal model @8ms;                   **
    // ** If thermal model runs @16ms it is mandatory to modify the time constants    **
    // *********************************************************************************
    // 8000.0 => 1 second = 8000 tick
    // psThermalParam->uwIgbtThCapacitance / 100.0 = J/K
    // psThermalParam->uwNtc2JunctionThResist / 1000.0 = K/W
    // 32768 = Code Optimization for thermal model @16ms
    // 1.0 / (...) = Code Optimization to avoid divisions during task execution
    sThModRun.HwParam.flIgbtThCapacitance = 1.0 / ((8000.0 * ((FLOAT)psThermalParam->uwIgbtThCapacitance / 100.0) * ((FLOAT)psThermalParam->uwNtc2JunctionThResist / 1000.0)) / 32768.0) ;  // modello termico@16ms 
    
    // 8000.0 => 1 second = 8000 tick
    // psThermalParam->uwHsnkThCapacitance = J/K
    // 65536 = Code Optimization per modello termico @16ms
    sThModRun.HwParam.flHsnkThCapacitance = (8000.0 *(FLOAT)psThermalParam->uwHsnkThCapacitance) / (65536.0 * 100.0) ;
    
    // sMh_PlcUMRatios.flRatioIpk / 10000.0 = conversion factor Ifpga(i.u.) -> A
    sThModRun.sLossesIgbtCommon.flVceSat =(((FLOAT)psThermalParam->uwIgbtVceSat / 100.0) * CONDUCTION_MARGIN) * (sMh_PlcUMRatios.flRatioIpk / 10000.0) ;
    sThModRun.sLossesIgbtCommon.flPreCommutationLosses = ((1000.0 * ((FLOAT)psThermalParam->uwCommutationLosses / 10000.0)) / (10.0 * V_NORMALIZATION * ((FLOAT)psThermalParam->uwIValueLosses / 10.0))) * (sMh_PlcUMRatios.flRatioIpk / 10000.0) ;
  }
  else if(psBridgeParam)
  {
    sTm_ThModOut.sOutValue.swIgbtTjMax          = (SWORD)ceil(psBridgeParam->fModuleMaxTJunction    * 10.0)   ; /* Junction Max Temperature, 1e-1 Celsius */
    sThModRun.HwParam.flNtcVDividerResistor     = psBridgeParam->fNtcVDividerResistor                         ; /* Resistor of NTC Voltage Divider    (ohm) */  
    sThModRun.HwParam.flNtc25DegreeResistance   = psBridgeParam->fNtc25DegreeResistance                       ; /* NTC Resistance value at 25 Celsius (ohm) */
    sThModRun.HwParam.flNtcMaterialConstant     = psBridgeParam->fNtcMaterialConstant                         ; /* NTC Material Specific Constant       (K) */
    sThModRun.HwParam.swIgbtMaximumTemp         = (SWORD)ceil(psBridgeParam->fIgbtMaximumTemp       * 10.0)   ; /* IGBT Max Temperature           (1e-1 Celsius) */
    sThModRun.HwParam.swHsnkMaximumTemp         = (SWORD)ceil(psBridgeParam->fHsnkMaximumTemp       * 10.0)   ; /* Heatsink/NTC Max Temperature   (1e-1 Celsius) */
    
    sThModRun.HwParam.uwNtc2JunctionThResist    = (UWORD)ceil(psBridgeParam->fNtc2JunctionThResist    * 1000.0) ;
    sThModRun.uwBiQuadNtc2JunctionThResist      = (UWORD)ceil(psBridgeParam->fBiQuadNtc2JunctThRes    * 1000.0) ;
    sThModRun.HwParam.uwHsnk2NtcThResCoolingON  = (UWORD)ceil(psBridgeParam->fHsnk2NtcThResCoolingON  * 100.0)  ;
    sThModRun.HwParam.uwHsnk2NtcThResCoolingOFF = (UWORD)ceil(psBridgeParam->fHsnk2NtcThResCoolingOFF * 100.0)  ;
    
    // *********************************************************************************
    // ** CODE OPTIMIZATION (to avoid to make  moltiplications during task execution) **
    // ** The optimizations are calculated with thermal model @8ms;                   **
    // ** If thermal model runs @16ms it is mandatory to modify the time constants    **
    // *********************************************************************************
    // 8000.0 => 1 second = 8000 tick
    // 32768 = Code Optimization for thermal model @16ms
    // 1.0 / (...) = Code Optimization per evitare divisione a runtime
    sThModRun.HwParam.flIgbtThCapacitance = 1.0 / ((8000.0 * psBridgeParam->fIgbtThCapacitance * psBridgeParam->fNtc2JunctionThResist) / 32768.0) ;  // thermal model @16ms 
    
    // 8000.0 => 1 second = 8000 tick
    // 65536 = Code Optimization thermal model @16ms
    sThModRun.HwParam.flHsnkThCapacitance = (8000.0 * psBridgeParam->fHsnkThCapacitance) / (65536.0 * 100.0) ;
    
    // sMh_PlcUMRatios.flRatioIpk / 10000.0 = conversion factor Ifpga(i.u.) -> A
    sThModRun.sLossesIgbtCommon.flVceSat =(psBridgeParam->fIgbtVceSat * CONDUCTION_MARGIN) * (sMh_PlcUMRatios.flRatioIpk / 10000.0) ;
    sThModRun.sLossesIgbtCommon.flPreCommutationLosses = ((1000.0 * psBridgeParam->fCommutationLosses) / (10.0 * V_NORMALIZATION * psBridgeParam->fIValueLosses)) * (sMh_PlcUMRatios.flRatioIpk / 10000.0) ;
  }
  else
    assert(FALSE);

  sThModRun.sLossesIgbtCommon.flCommutationLosses = sThModRun.sLossesIgbtCommon.flPreCommutationLosses * COMMUTATION_MARGIN ;    // 10.0 = Vdc is in 1e-1V  ;
  sThModRun.sLossesIgbtCommon.uwNtc2JunctionThResist = sThModRun.HwParam.uwNtc2JunctionThResist ;	
  sThModRun.sLossesIgbtCommon.uwBiQuadNtc2JunctionThResist = sThModRun.uwBiQuadNtc2JunctionThResist ;	

#ifdef _HW_CT
  /* on board thermal model parameters */
  sThModRun.HwParam.flBrdNtcVDividerResistor     = (FLOAT)psBrdThermalParam->uwNtcVDividerResistor   * 10.0    ; /* Resistor of NTC Voltage Divider    (ohm) */  
  sThModRun.HwParam.flBrdNtc25DegreeResistance   = (FLOAT)psBrdThermalParam->uwNtc25DegreeResistance * 10.0    ; /* NTC Resistance value at 25 Celsius (ohm) */
  sThModRun.HwParam.flBrdNtcMaterialConstant     = (FLOAT)psBrdThermalParam->uwNtcMaterialConstant   * 1.0     ; /* NTC Material Specific Constant       (K) */
#endif

  // --------------------- operative flags ---------------------------------------
  sThModRun.flags.b.bDisFltReactOnLimit       = sTm_ThModParam.flags.b.bDisFltReactOnLimit ;
  sThModRun.flags.b.bNoBrakeDisableOnFlt      = sTm_ThModParam.flags.b.bNoBrakeDisableOnFlt ;
  sThModRun.flags.b.bNoFatalOnBrakeFlt        = sTm_ThModParam.flags.b.bNoFatalOnBrakeFlt ;

  // --------------------- modulation frequency and strategy ---------------------
  sThModRun.sLossesIgbtCommon.flags.b.b2Switch = (BOOL)(sTm_ThModIn.psMotorData->uwActualPwmSetup&MH_CMD_SETMOD_ENABLE2STEPS);

  // Dynamic Overcurrent = starts from IdChipOverCurrent
  sTm_ThModOut.slDynamicOverCurrent = slIdChipOverCurrent ;
  // -----------------------------------------------------------------------------

  // if user value = 0: set default values written into the Id-Chip and save it into flash
  if (sTm_ThModParam.uwRBrakeValue == 0 || sTm_ThModParam.uwRBrakeMaxPower == 0 || sTm_ThModParam.ulRBrakeMaxEnergy == 0l)
  {
    if (psBrkRes)
    {
      if(psBrkRes->fValue>3276.7)
        return FALSE;
      if(psBrkRes->fMaxPower>3276.7)
        return FALSE;
      if(psBrkRes->fMaxEnergy>(FLOAT)(SLONG_MAX_VALUE))
        return FALSE;

      sThModRun.HwParam.flRBrakeValue    = psBrkRes->fValue ;
      sThModRun.HwParam.swRBrakeMaxPower = (SWORD)(psBrkRes->fMaxPower*10.0) ;

      sTm_ThModOut.uwBrakeResistorValue  = (UWORD)(sThModRun.HwParam.flRBrakeValue*10.0) ;
      sTm_ThModOut.uwBrakeResistorPower  = sThModRun.HwParam.swRBrakeMaxPower;
      sTm_ThModOut.ulBrakeResistorEnergy = (ULONG)psBrkRes->fMaxEnergy;
    }
    else
    {
      sThModRun.HwParam.flRBrakeValue    = (FLOAT) sBrakeCircuit.uwResistorValue / 10.0 ; /* ohm */
      sThModRun.HwParam.swRBrakeMaxPower = (SWORD) sBrakeCircuit.uwResistorPower ;        /* (1e-1 W) */

      sTm_ThModOut.uwBrakeResistorValue  = sBrakeCircuit.uwResistorValue;
      sTm_ThModOut.uwBrakeResistorPower  = sBrakeCircuit.uwResistorPower;
      sTm_ThModOut.ulBrakeResistorEnergy = sBrakeCircuit.ulRBrakeMaxEnergy;
    }
  }
  else
  {
    sThModRun.HwParam.flRBrakeValue    = (FLOAT) sTm_ThModParam.uwRBrakeValue / 10.0 ; /* ohm */
    sThModRun.HwParam.swRBrakeMaxPower = (SWORD) sTm_ThModParam.uwRBrakeMaxPower     ; /* (1e-1 W) */

    sTm_ThModOut.uwBrakeResistorValue  = sTm_ThModParam.uwRBrakeValue;
    sTm_ThModOut.uwBrakeResistorPower  = sTm_ThModParam.uwRBrakeMaxPower;
    sTm_ThModOut.ulBrakeResistorEnergy = sTm_ThModParam.ulRBrakeMaxEnergy;
  }

  // R brake Thermal Time Constant: 1 / (EnergyMax / PowerMax) = PowerMax(1e-1W)/EnergyMax (J) = (PowerMax/EnergyMax) / 10
  if ((sTm_ThModOut.uwBrakeResistorPower != 0) && (sTm_ThModOut.ulBrakeResistorEnergy != 0l))
    sThModRun.flRBrakeThConst = ((FLOAT)sTm_ThModOut.uwBrakeResistorPower / (FLOAT)sTm_ThModOut.ulBrakeResistorEnergy) * 0.1 ;    

  sTm_ThModIn.psAdcIoData = &sIOMgrAnalogMeasurements ;

  // setting up temperature limit threshold
  sThModRun.swILimitMaxTemp = sThModRun.HwParam.swIgbtMaximumTemp - 50 ; // IgbtTMax - 5 Celsius
  sThModRun.swILimitMinTemp = sTm_ThModOut.sOutValue.swIgbtTjMax - 300 ; // IgbtTjMax - 30 Celsius
  sThModRun.swILimitDelta   = sThModRun.swILimitMaxTemp - sThModRun.swILimitMinTemp ; // = (IgbtTmax-5) - (IgbtTjMax-30) = (TjMax-15-5) - (IgbtTjMax-30) = 10 Celsius
  
  if (bDisablePkPk)
    sThModRun.swHsnkDeratingMaximumTemp = sThModRun.HwParam.swHsnkMaximumTemp - 50 ; // HsnkMaxT - 5 Celsius
  else
    sThModRun.swHsnkDeratingMaximumTemp = sThModRun.HwParam.swHsnkMaximumTemp - 100 ; // HsnkMaxT - 10 Celsius
                                                                                                             
  /* set output values */ 
  sTm_ThModOut.sIdLimit.slMax =  sThModRun.slIdChipCurrLimit ; 
  sTm_ThModOut.sIqLimit.slMax =  sTm_ThModOut.sIdLimit.slMax ;
  sTm_ThModOut.sIdLimit.slMin = -sTm_ThModOut.sIdLimit.slMax ;
  sTm_ThModOut.sIqLimit.slMin =  sTm_ThModOut.sIdLimit.slMin ;

  sThModRun.slIdLocLimMax_1 = sTm_ThModOut.sIdLimit.slMax ;

  /* reset runtime variables */
#ifdef _INFINEON_
  xmemset(&sThModRun_sServoMediumTask, 0, sizeof(THERMAL_MODEL_MEDIUM_TASK_VARIABLES)) ;
  xmemset(&sThModRun_sServoSlowTask, 0, sizeof(THERMAL_MODEL_SLOW_TASK_VARIABLES)) ;
#else
  memset(&sThModRun_sServoMediumTask, 0, sizeof(THERMAL_MODEL_MEDIUM_TASK_VARIABLES)) ;
  memset(&sThModRun_sServoSlowTask, 0, sizeof(THERMAL_MODEL_SLOW_TASK_VARIABLES)) ;
#endif // _infineon_

  sThModRun.uwHsnk2NtcThRes2Use = sThModRun.HwParam.uwHsnk2NtcThResCoolingOFF ;

  sTm_ThModOut.sOutValue.swTNtc = 0 ;
  sTm_ThModOut.sOutValue.swTjMax = 0 ;
  sTm_ThModOut.sOutValue.swTHeatSink = 0 ;
  sThModRun.slTHeatSink = 0 ;
  sThModRun.flags.b.bTaskRunning = FALSE ;

  sThModRun.sLossesIgbtCommon.slPjTot = 0 ;
#ifdef _INFINEON_
  xmemset(&sThModRun.sLossesIgbtUHi, 0, sizeof(THERMAL_MODEL_LOSSES_IGBT_SINGLE)) ;
  xmemset(&sThModRun.sLossesIgbtVHi, 0, sizeof(THERMAL_MODEL_LOSSES_IGBT_SINGLE)) ;
  xmemset(&sThModRun.sLossesIgbtWHi, 0, sizeof(THERMAL_MODEL_LOSSES_IGBT_SINGLE)) ;
  xmemset(&sThModRun.sLossesIgbtULo, 0, sizeof(THERMAL_MODEL_LOSSES_IGBT_SINGLE)) ;
  xmemset(&sThModRun.sLossesIgbtVLo, 0, sizeof(THERMAL_MODEL_LOSSES_IGBT_SINGLE)) ;
  xmemset(&sThModRun.sLossesIgbtWLo, 0, sizeof(THERMAL_MODEL_LOSSES_IGBT_SINGLE)) ;
#else
  memset(&sThModRun.sLossesIgbtUHi, 0, sizeof(THERMAL_MODEL_LOSSES_IGBT_SINGLE)) ;
  memset(&sThModRun.sLossesIgbtVHi, 0, sizeof(THERMAL_MODEL_LOSSES_IGBT_SINGLE)) ;
  memset(&sThModRun.sLossesIgbtWHi, 0, sizeof(THERMAL_MODEL_LOSSES_IGBT_SINGLE)) ;
  memset(&sThModRun.sLossesIgbtULo, 0, sizeof(THERMAL_MODEL_LOSSES_IGBT_SINGLE)) ;
  memset(&sThModRun.sLossesIgbtVLo, 0, sizeof(THERMAL_MODEL_LOSSES_IGBT_SINGLE)) ;
  memset(&sThModRun.sLossesIgbtWLo, 0, sizeof(THERMAL_MODEL_LOSSES_IGBT_SINGLE)) ;
#endif // _infineon_

  sThModRun.slTmp = 0 ; // variabile d'appoggio
  sThModRun.flTmp = 0.0 ; // variabile d'appoggio
  sThModRun.slTjMaxFiltSts = 0 ;

  MotorI2T_Init() ;

  /* install current limits */
  (*sTm_ThModIn.psMotorHandlers->pfAdd2LimitList)(MH_LIMITLIST_IQ, &sTm_ThModOut.sIqLimit);
  (*sTm_ThModIn.psMotorHandlers->pfAdd2LimitList)(MH_LIMITLIST_ID, &sTm_ThModOut.sIdLimit);

  /* install background function */ 
  if(!TaskSched_AddBackgroundTask(&Tm_ThermalModelBkGd))
    return FALSE ;  

  sMediumTaskHandle = Os_TaskCreateEx(&Tm_ThermalModelMediumTask,512,"TmMediumTask");

  /* install 8KHz function */
  return TaskSched_AddRTTask(&Tm_ThermalModel8KHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0) ; 
}


/* ========================================================================= */
static BOOL Tm_ThermalModel8KHz(void)
{
    SWORD   swLowerPhase = PHASE_U, swUpperPhase = PHASE_U, swVMin, swVMax ;
    SWORD   swTmp ;
    BOOL    b2Switch = (BOOL)(sTm_ThModIn.psMotorData->uwActualPwmSetup&MH_CMD_SETMOD_ENABLE2STEPS) && !(BOOL)(sTm_ThModIn.psMotorData->uwActualPwmSetup&MH_CMD_SETMOD_2STEPSFLOAT);
	BOOL 	bSelect2StepsLowOnly = (BOOL)(sTm_ThModIn.psMotorData->uwActualPwmSetup&MH_CMD_SETMOD_2STEPSLONLY);
    BOOL    b2SwitchLow = FALSE;
    BOOL    b2SwitchHigh = FALSE;

    if (!sTm_ThModIn.psAdcIoData->swAllValid)
        return TRUE ;   // data from ADC Manager not ready: exit

    if (bSysStatAlarmsReset)
        sThModRun.sErrorSent.w = 0 ; /* clear errors */

    /* updating Servo Value (to calculate average values in 8ms or 32ms task) */     
#ifndef _HW_DC
    sThModRun_sServoSlowTask.ulNtc  += (ULONG)(sTm_ThModIn.psAdcIoData->uwSBridgeTemp) ; /* NTC -> update rate: 125us */
#else
#ifdef _HW_AXS_SAIETTA
    sThModRun_sServoSlowTask.ulNtc += (ULONG)sTm_ThModIn.psAdcIoData->uwSBrgTemp[0];
    sThModRun_sServoSlowTask.ulBr1Ntc += (ULONG)sTm_ThModIn.psAdcIoData->uwSBrgTemp[1];
    sThModRun_sServoSlowTask.ulBr2Ntc += (ULONG)sTm_ThModIn.psAdcIoData->uwSBrgTemp[2];
#else	// _hw_axs_saietta
    sThModRun_sServoSlowTask.ulNtc += (ULONG)sTm_ThModIn.psAdcIoData->uwSBrgTemp[0];
    if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge1Valid)
    {
        sThModRun_sServoSlowTask.ulBr1Ntc += (ULONG)sTm_ThModIn.psAdcIoData->uwSBrgTemp[1];
        if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge2Valid)
            sThModRun_sServoSlowTask.ulBr2Ntc += (ULONG)sTm_ThModIn.psAdcIoData->uwSBrgTemp[2];
    }
#endif	// _hw_axs_saietta
#endif	// !_hw_dc

#ifdef _HW_CT
    sThModRun_sServoSlowTask.ulBrdNtc  += (ULONG)(sTm_ThModIn.psAdcIoData->uwSBoardTemp) ; /* Broad NTC -> update rate: 125us */
#endif

    sThModRun_sServoMediumTask.slDcBus += (SLONG)(sTm_ThModIn.psMotorData->swDcBusValue) ;  /* DC Bus  Voltage (1e-1V) */  
    
    if (b2Switch)
    {
        if (bSelect2StepsLowOnly)
        {   /* looking for the lowest phase */
            if (sTm_ThModIn.psMotorData->swVvOut > sTm_ThModIn.psMotorData->swVuOut)
            {
                 swLowerPhase = PHASE_U ; 
                 swVMin = sTm_ThModIn.psMotorData->swVuOut ; 
            }
            else
            {
                 swLowerPhase = PHASE_V ;
                 swVMin = sTm_ThModIn.psMotorData->swVvOut ; 
            }
        
            if (swVMin > (-(sTm_ThModIn.psMotorData->swVuOut + sTm_ThModIn.psMotorData->swVvOut)))
                 swLowerPhase = PHASE_W ;

            b2SwitchLow = TRUE;
        }
        else
        {   /* looking for the lowest phase */
            if (sTm_ThModIn.psMotorData->swVvOut < sTm_ThModIn.psMotorData->swVuOut)
            {
                 swUpperPhase = PHASE_U ; 
                 swVMax = sTm_ThModIn.psMotorData->swVuOut ; 
            }
            else
            {
                 swUpperPhase = PHASE_V ;
                 swVMax = sTm_ThModIn.psMotorData->swVvOut ; 
            }
        
            if (swVMax < (-(sTm_ThModIn.psMotorData->swVuOut + sTm_ThModIn.psMotorData->swVvOut)))
                 swUpperPhase = PHASE_W ;

            b2SwitchHigh = TRUE;
        }
    }

    if (bDisablePkPk)
    {
        sThModRun_sServoMediumTask.slIuPkHi = 0 ;
        sThModRun_sServoMediumTask.slIvPkHi = 0 ;
        sThModRun_sServoMediumTask.slIwPkHi = 0 ;
        sThModRun_sServoMediumTask.slIuPkLo = 0 ;
        sThModRun_sServoMediumTask.slIvPkLo = 0 ;
        sThModRun_sServoMediumTask.slIwPkLo = 0 ;
    }
    else
    {
        swTmp = (SWORD)FPGA_CURCHMINMAX_IU_MAX - sMh_MotorDataOut.sB0IpkpkMaxOffsets.swIU ;
        if (swTmp > 0)
        {
            if (b2SwitchHigh)
            {   /* 2 switch modulation: update filter ONLY if Phase U is not the lower */
                if (swUpperPhase != PHASE_U) 
                    sThModRun_sServoMediumTask.slIuPkHi += swTmp ; 
            }
            else
                sThModRun_sServoMediumTask.slIuPkHi += swTmp ;
        }
    
        swTmp = (SWORD)FPGA_CURCHMINMAX_IV_MAX - sMh_MotorDataOut.sB0IpkpkMaxOffsets.swIV ;
        if (swTmp > 0)
        {
            if (b2SwitchHigh)
            {   /* 2 switch modulation: update filter ONLY if Phase V is not the lower */
                if (swUpperPhase != PHASE_V) 
                    sThModRun_sServoMediumTask.slIvPkHi += swTmp ; 
            }
            else
                sThModRun_sServoMediumTask.slIvPkHi += swTmp ; 
        }
    
        swTmp = (SWORD)FPGA_CURCHMINMAX_IW_MAX - sMh_MotorDataOut.sB0IpkpkMaxOffsets.swIW ;
        if (swTmp > 0)
        {
            if (b2SwitchHigh)
            {   /* 2 switch modulation: update filter ONLY if Phase W is not the lower */
                if (swUpperPhase != PHASE_W) 
                    sThModRun_sServoMediumTask.slIwPkHi += swTmp ; 
            }
            else
                sThModRun_sServoMediumTask.slIwPkHi += swTmp ;
        }
    
        swTmp = (SWORD)FPGA_CURCHMINMAX_IU_MIN - sMh_MotorDataOut.sB0IpkpkMinOffsets.swIU ;
        if (swTmp < 0)
        {
            if (b2SwitchLow)
            {   /* 2 switch modulation: update filter ONLY if Phase U is not the lower */
                if (swLowerPhase != PHASE_U) 
                    sThModRun_sServoMediumTask.slIuPkLo += swTmp ; 
            }
            else
                sThModRun_sServoMediumTask.slIuPkLo += swTmp ;
        }
    
        swTmp = (SWORD)FPGA_CURCHMINMAX_IV_MIN - sMh_MotorDataOut.sB0IpkpkMinOffsets.swIV ;
        if (swTmp < 0)
        {
            if (b2SwitchLow)
            {   /* 2 switch modulation: update filter ONLY if Phase V is not the lower */
                if (swLowerPhase != PHASE_V) 
                    sThModRun_sServoMediumTask.slIvPkLo += swTmp ; 
            }
            else
                sThModRun_sServoMediumTask.slIvPkLo += swTmp ; 
        }
    
        swTmp = (SWORD)FPGA_CURCHMINMAX_IW_MIN - sMh_MotorDataOut.sB0IpkpkMinOffsets.swIW ;
        if (swTmp < 0)
        {
            if (b2SwitchLow)
            {   /* 2 switch modulation: update filter ONLY if Phase W is not the lower */
                if (swLowerPhase != PHASE_W) 
                    sThModRun_sServoMediumTask.slIwPkLo += swTmp ; 
            }
            else
                sThModRun_sServoMediumTask.slIwPkLo += swTmp ;
        }
    }              

    if (FPGA_PWM_STATUS_BRAKE_ACTIVE(FPGA_PWM_STATUS))
        sThModRun_sServoMediumTask.uwRBrakeON++ ; /* in this servo, RBrake is ON */ 

    if ( ++sThModRun_sServoMediumTask.uwCnts >= MIN_UPDATE_PERIOD)
    {
        if (!sThModRun.flags.b.bTaskRunning) 
        {   // thermal model task IS NOT running
            if (sThModRun_sServoMediumTask.uwCnts < MAX_UPDATE_PERIOD)
            {   // MAX update period NOT EXPIRED
                
                // update data to slow task
                sThModRun.sImageMediumTask.uwCnts     = sThModRun_sServoMediumTask.uwCnts ;
                sThModRun.sImageMediumTask.uwRBrakeON = sThModRun_sServoMediumTask.uwRBrakeON ;
                sThModRun.sImageMediumTask.slDcBus    = sThModRun_sServoMediumTask.slDcBus ;
                sThModRun.sImageMediumTask.slIuPkHi   = sThModRun_sServoMediumTask.slIuPkHi ; 
                sThModRun.sImageMediumTask.slIvPkHi   = sThModRun_sServoMediumTask.slIvPkHi ; 
                sThModRun.sImageMediumTask.slIwPkHi   = sThModRun_sServoMediumTask.slIwPkHi ; 
                sThModRun.sImageMediumTask.slIuPkLo   = sThModRun_sServoMediumTask.slIuPkLo ; 
                sThModRun.sImageMediumTask.slIvPkLo   = sThModRun_sServoMediumTask.slIvPkLo ; 
                sThModRun.sImageMediumTask.slIwPkLo   = sThModRun_sServoMediumTask.slIwPkLo ; 
#ifndef _INFINEON_
                sThModRun.sImageSlowTask.uwSBoardTemp  = sTm_ThModIn.psAdcIoData->uwSBoardTemp ;
                sThModRun.sImageSlowTask.flSOnchipTemp = sTm_ThModIn.psAdcIoData->fSOnchipTemp ;
#endif // _infineon_

                sThModRun.sImageSlowTask.ulNtc = sThModRun_sServoSlowTask.ulNtc ;
#ifndef _HW_DC
                sThModRun.sImageSlowTask.uwMotorTemp = sTm_ThModIn.psAdcIoData->uwSMotorTemp ; // no filtering (present value)
#else
                sThModRun.sImageSlowTask.ulBr1Ntc    = sThModRun_sServoSlowTask.ulBr1Ntc ;
                sThModRun.sImageSlowTask.ulBr2Ntc    = sThModRun_sServoSlowTask.ulBr2Ntc ;

                sThModRun.sImageSlowTask.uwMotorPTC  = sTm_ThModIn.psAdcIoData->uwSMotorPTCTemp ;
                sThModRun.sImageSlowTask.uwMotorKTY  = sTm_ThModIn.psAdcIoData->uwSMotorKTYTemp ;
#endif // _hw_dc
#ifdef _HW_CT
                sThModRun.sImageSlowTask.ulBrdNtc    = sThModRun_sServoSlowTask.ulBrdNtc ;
#endif // _hw_ct

                // execute slow task
#ifdef _INFINEON_
                Os_TaskEventNotify(&Tm_ThermalModelMediumTask) ;
#endif // _infineon_
                Os_TaskEventNotify(sMediumTaskHandle) ;
            }
            else
            {   // MAX update period EXPIRED: error
                if(!sThModRun.sErrorSent.b.bSlowTaskOverTime)
                {
                    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_SOFTWARE_FAIL, SYSTEMALARMS_SUBCODE_SF_TM_SLOWTASK_OVERTIME, TRUE) ;
                    (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
                    sThModRun.sErrorSent.b.bSlowTaskOverTime = TRUE ;
                }
            }
            
            // reset servo values
            sThModRun_sServoMediumTask.uwCnts   = 0 ;
            sThModRun_sServoMediumTask.uwRBrakeON = 0 ;
            sThModRun_sServoMediumTask.slDcBus  = 0 ;
            sThModRun_sServoMediumTask.slIuPkHi = 0 ;
            sThModRun_sServoMediumTask.slIvPkHi = 0 ;
            sThModRun_sServoMediumTask.slIwPkHi = 0 ;
            sThModRun_sServoMediumTask.slIuPkLo = 0 ;
            sThModRun_sServoMediumTask.slIvPkLo = 0 ;
            sThModRun_sServoMediumTask.slIwPkLo = 0 ;
#ifndef _INFINEON_
            sThModRun_sServoSlowTask.uwSBoardTemp  = 0 ;
            sThModRun_sServoSlowTask.flSOnchipTemp = 0 ;
#endif // _infineon_

            sThModRun_sServoSlowTask.ulNtc       = 0 ;
#ifndef _HW_DC
            sThModRun_sServoSlowTask.uwMotorTemp = 0 ;
#else
            sThModRun_sServoSlowTask.ulBr1Ntc    = 0 ;
            sThModRun_sServoSlowTask.ulBr2Ntc    = 0 ;
            sThModRun_sServoSlowTask.uwMotorPTC  = 0 ;
            sThModRun_sServoSlowTask.uwMotorKTY  = 0 ;
#endif // _hw_dc
#ifdef _HW_CT
            sThModRun_sServoSlowTask.ulBrdNtc    = 0 ;
#endif
        }
        else        
        {   // thermal model task IS running
            if (sThModRun_sServoMediumTask.uwCnts >= MAX_UPDATE_PERIOD)
            {   // MAX update period EXPIRED: error
                if(!sThModRun.sErrorSent.b.bSlowTaskRunning)
                {
                    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_SOFTWARE_FAIL, SYSTEMALARMS_SUBCODE_SF_TM_SLOWTASK_RUNNING, TRUE) ;
                    (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
                    sThModRun.sErrorSent.b.bSlowTaskRunning = TRUE ;
                }
    
                // reset servo values
                sThModRun_sServoMediumTask.uwCnts   = 0 ;
                sThModRun_sServoMediumTask.uwRBrakeON = 0 ;
                sThModRun_sServoMediumTask.slDcBus  = 0 ;
                sThModRun_sServoMediumTask.slIuPkHi = 0 ;
                sThModRun_sServoMediumTask.slIvPkHi = 0 ;
                sThModRun_sServoMediumTask.slIwPkHi = 0 ;
                sThModRun_sServoMediumTask.slIuPkLo = 0 ;
                sThModRun_sServoMediumTask.slIvPkLo = 0 ;
                sThModRun_sServoMediumTask.slIwPkLo = 0 ;
#ifndef _INFINEON_
                sThModRun_sServoSlowTask.uwSBoardTemp  = 0 ;
                sThModRun_sServoSlowTask.flSOnchipTemp = 0 ;
#endif // _infineon_

                sThModRun_sServoSlowTask.ulNtc       = 0 ;
#ifndef _HW_DC
                sThModRun_sServoSlowTask.uwMotorTemp = 0 ;
#else
                sThModRun_sServoSlowTask.ulBr1Ntc    = 0 ;
                sThModRun_sServoSlowTask.ulBr2Ntc    = 0 ;
                sThModRun_sServoSlowTask.uwMotorPTC  = 0 ;
                sThModRun_sServoSlowTask.uwMotorKTY  = 0 ;
#endif
#ifdef _HW_CT
                sThModRun_sServoSlowTask.ulBrdNtc    = 0 ;
#endif
            }
        }
    }


    return TRUE ;
}


/* ========================================================================= */
void Tm_ThermalModelMediumTask(void)
{    
    UWORD uwTimeCounts ;

    for(;;)
    {
    	Os_TaskEventNotifyWait();

		sThModRun.flags.b.bTaskRunning = TRUE ;

		uwTimeCounts = timer_profiler_start(uwSysTimers125us) ;

		IgbtTNtc() ;         // calculate TNtc igbt module
		IgbtModuleLosses() ; // calculate igbt module losses and evaluate Tjunctions
		IgbtCurrentLimit() ; // calculate current limit due to igbt module TjMax
		DriveHeatsink() ;    // calculate Theatsink and current limit due to Theatsink
#ifdef _HW_CT
		BrdTNtc() ;          // calculate OnBoard TNtc module
#endif

		if((UWORD)sThModRun.HwParam.flRBrakeValue != 0)
			BrakeResistor() ; // calculate brake resistor energy and power

		DynamicOverCurrent() ; // calculate dynamic overcurrent
		ThermalModelCurrentLimit() ; // apply Thermal Model current limit
		AdaptModulation();     // adapt modulation type
#ifdef __MOTOR_I2T_IN_SLOWTASK__
		MotorI2T_Runtime() ; // motor I2T
#endif
		// thermal model max time execution
		uwTimeCounts = timer_profiler_end(uwSysTimers125us, uwTimeCounts) ;
		if (uwTimeCounts > sTm_ThModOut.sOutValue.uwTaskLoad)
			sTm_ThModOut.sOutValue.uwTaskLoad = uwTimeCounts ;

		sThModRun.flags.b.bTaskRunning = FALSE ;
    }
}

/* ========================================================================= */
static void coolingtempsetup(void)
{
  sThModRun.swCoolingTempOn  = sTm_ThModParam.swCoolingTempOn  ;
  sThModRun.swCoolingTempOff = sTm_ThModParam.swCoolingTempOff ;
  sThModRun.swCoolingVSpdRatio = (SWORD)((SLONG)FANDRIVE_PERIOD/(sThModRun.swCoolingTempOn - sThModRun.swCoolingTempOff));
}

/* ========================================================================= */
static void Tm_ThermalModelBkGd(void)
{
    if(liveparcheck())
      coolingtempsetup();

    // motor temperature management
    MotorTemperature() ;

    // IGBT losses adjustment
    AdjIgbtLosses() ;

    // motor I2T
#ifndef __MOTOR_I2T_IN_SLOWTASK__
    MotorI2T_Runtime() ; 
#endif

#ifndef _INFINEON_
    DiagnosticTemperatures() ; // ZynQ and controlboard temperature
#endif
}

/* ========================================================================= */
static void MotorTemperature(void)
// IT CAN BE EXECUTED IN THE SLOW TASK (TO SAVE MICRO TIME)
{
    SWORD swMotOverTemp = sTm_ThModParam.swMotorOverTemp;
    FLOAT flMotTemp;

#ifndef _HW_DC
    if (bDisableFwMotorTemp)
        flMotTemp = flTm_PlcMotorTemp ; // data from plc
    else
        // polinomio (cubico, range ottimizzato [-40 Celsius;+200 Celsius]) per convertire da count dac a temperatura (Kty, PT1000 - vedi file excel)
        // KTY:    k[0] = -249.26152577319; k[1] = 0.95002264596960; k[2] = -0.0011032587471901; k[3] = 0.0000009184245785
        // PT1000: k[0] = -982.21052294105; k[1] = 4.33980380864001; k[2] = -0.0077264727361498; k[3] = 0.0000055359100517
        // TMotore = k[3] * count^3 + k[2] * count^2 + k[1]* count + k[0] 
        flMotTemp = Poly3Linearization(sThModRun.sImageSlowTask.uwMotorTemp, NULL, sTm_ThModParam.sflKTYCoeff);
#else
    if (bDisableFwMotorTemp)
        flMotTemp = flTm_PlcMotorTemp ; // data from plc
    else
    {
#if defined(_HW_AXS_DAYCO22KW)

#ifdef _CRS_DBG
    	sTm_ThModOut.uwPtcCount = sThModRun.sImageSlowTask.uwMotorKTY ;
    	sTm_ThModOut.uwKtyCount = sThModRun.sImageSlowTask.uwMotorPTC;
#endif
    	// for the moment IdChip range it is not used
    	sTm_ThModOut.flMotorKTY = Poly3Linearization(sThModRun.sImageSlowTask.uwMotorKTY, NULL, sTm_ThModParam.sflKTYCoeff);
		sTm_ThModOut.flMotorPTC = Poly3Linearization(sThModRun.sImageSlowTask.uwMotorPTC, NULL, sTm_ThModParam.sflPTCCoeff);
#else
    	sTm_ThModOut.flMotorKTY = Poly3Linearization(sThModRun.sImageSlowTask.uwMotorKTY, &sGlbControlBoardParameters.sUniv.sKTYRange, sTm_ThModParam.sflKTYCoeff);
		sTm_ThModOut.flMotorPTC = Poly3Linearization(sThModRun.sImageSlowTask.uwMotorPTC, &sGlbControlBoardParameters.sUniv.sPTCRange, sTm_ThModParam.sflPTCCoeff);
#endif // _hw_axs_dayco22kw

		/* select highest */
		if(sTm_ThModOut.flMotorKTY>sTm_ThModOut.flMotorPTC)
			flMotTemp = sTm_ThModOut.flMotorKTY;
		else
			flMotTemp = sTm_ThModOut.flMotorPTC;
    }
#endif // !_hw_dc

    // limit before conversion and test
    if(flMotTemp < MOTOR_TEMP_MIN)
        flMotTemp = MOTOR_TEMP_MIN;
    if(flMotTemp > MOTOR_TEMP_MAX)
        flMotTemp = MOTOR_TEMP_MAX;

    sTm_ThModOut.sOutValue.flMotorTemp = flMotTemp;
    sTm_ThModOut.sOutValue.swMotorTemp = (SWORD)(sTm_ThModOut.sOutValue.flMotorTemp * 10.0);

    if ((sTm_ThModOut.sOutValue.swMotorTemp >= swMotOverTemp) && (!sThModRun.sErrorSent.b.bMotorOverTemp))
    {
        sThModRun.sErrorSent.b.bMotorOverTemp = TRUE ;
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_TM_MOTOR_OVERTEMPERATURE, SYSTEMALARMS_SUBCODE_TM_MOTOR_OVERTEMP, bSysStatPowerEnabled) ;
        (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL) ;
    }

    if ((sTm_ThModOut.sOutValue.swMotorTemp >= swMotOverTemp+ALARM_DELTAT))
        (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
}

/* ========================================================================= */
static FLOAT Poly3Linearization(UWORD uwSample, HWPRM2_ANALOG_RANGE * sRange, FLOAT flK[4])
{
    FLOAT flRes,flLin,flPwr;
    SWORD i;

    flRes=(FLOAT)uwSample;

    if(sRange)
    {
        // ratio %
        flRes=flRes/DACRESOLUTION;
    
        // scaling
        flRes=flRes*(sRange->flHiSpan-sRange->flLoSpan)+sRange->flLoSpan;
    }

    // poly
    flLin=flK[0];
    flPwr=flRes;
    for(i=1;i<4;i++)
    {
        flLin+=flPwr*flK[i];
        flPwr*=flRes;
    }

    return flLin;
}

/* ========================================================================= */
static BOOL liveparcheck(void)
{
  BOOL bValid=TRUE;

  if(sTm_ThModParam.swCoolingTempOn < sTm_ThModParam.swCoolingTempOff)
  {
      ParChk_SignalValueError(PARCC_TM_INVALIDCOOLINGTHRESHOLDS);
      bValid=FALSE;
  }
  else
      ParChk_ResetValueError(PARCC_TM_INVALIDCOOLINGTHRESHOLDS);

  if(sTm_ThModParam.uwRBrakeValue == 0 && sTm_ThModParam.uwRBrakeMaxPower == 0 && sTm_ThModParam.ulRBrakeMaxEnergy == 0l)
  {
      ParChk_ResetValueError(PARCC_TM_INVALIDBRAKE_RESISTANCE);
      ParChk_ResetValueError(PARCC_TM_INVALIDBRAKE_POWER);
      ParChk_ResetValueError(PARCC_TM_INVALIDBRAKE_ENERGY);
  }
  else
  {
      if(sTm_ThModParam.uwRBrakeValue == 0)
      {
          ParChk_SignalValueError(PARCC_TM_INVALIDBRAKE_RESISTANCE);
          bValid=FALSE;
      }
      else
          ParChk_ResetValueError(PARCC_TM_INVALIDBRAKE_RESISTANCE);
        
      if(sTm_ThModParam.uwRBrakeMaxPower == 0)
      {
          ParChk_SignalValueError(PARCC_TM_INVALIDBRAKE_POWER);
          bValid=FALSE;
      }
      else
          ParChk_ResetValueError(PARCC_TM_INVALIDBRAKE_POWER);
        
      if(sTm_ThModParam.ulRBrakeMaxEnergy == 0l)
      {
          ParChk_SignalValueError(PARCC_TM_INVALIDBRAKE_ENERGY);
          bValid=FALSE;
      }
      else
          ParChk_ResetValueError(PARCC_TM_INVALIDBRAKE_ENERGY);
  }

#ifndef _INFINEON_
  if ((sTm_ThModParam.flOnChipOverTemp <= 0.0) || (sTm_ThModParam.flOnChipOverTemp > 150.0))
  {
      ParChk_SignalValueError(PARCC_TM_INVALIDONCHIPOVETERMPTHRESHOLDS);
      bValid=FALSE;
  }
  else
      ParChk_ResetValueError(PARCC_TM_INVALIDONCHIPOVETERMPTHRESHOLDS);
#endif
  
  return bValid;
}
/* ========================================================================= */
// Evaluation of the IGBT NTC temperature (solving equation)
// Rntc  = Rdivider * (1024/Va2dAvrg - 1)
// Tigbt = (NtcMatConst * 298.15) / (NtcMatConst + 298.15 * ln(Rntc/Rntc25?) - 273.15
// IT CAN BE EXECUTED IN THE SLOW TASK (TO SAVE MICRO TIME)
static void IgbtTNtc(void)
{    
    UWORD uwNtcAvrg ;
    FLOAT flTntc ;
#ifdef _HW_DC
    UWORD uwBr1NtcAvrg = NTC_IGBT_FAULT_THRESHOLD;
    UWORD uwBr2NtcAvrg = NTC_IGBT_FAULT_THRESHOLD;
    FLOAT flBr1Tntc, flBr2Tntc ;
	
#if (defined (_HW_AXS_SAIETTA) || defined (_HW_AXS_CABI35KW))
    FLOAT flVAdc;	// ADC VRead
    FLOAT flREqv;	// Equivalent resistance
    FLOAT flRNtc;	// NTC resistance
#else
    // Dayco || AxN-DC
	FLOAT flBr0_Rntc, flBr1_Rntc, flBr2_Rntc ;
#endif // (_hw_axs_saietta || _hw_axs_cabi35kw)
#endif // _hw_dc

#ifdef _INFINEON_
    uwNtcAvrg = (UWORD)(32 * sThModRun.sImageSlowTask.ulNtc / sThModRun.sImageMediumTask.uwCnts) ;
#ifdef _HW_DC
    if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge1Valid)
    {
        uwBr1NtcAvrg = (UWORD)(32 * sThModRun.sImageSlowTask.ulBr1Ntc / sThModRun.sImageMediumTask.uwCnts) ;
        if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge2Valid)
            uwBr2NtcAvrg = (UWORD)(32 * sThModRun.sImageSlowTask.ulBr2Ntc / sThModRun.sImageMediumTask.uwCnts) ;
    }
#endif // _hw_dc
#else
    uwNtcAvrg = (UWORD)(sThModRun.sImageSlowTask.ulNtc / sThModRun.sImageMediumTask.uwCnts) ;

#ifdef _HW_DC
#ifdef _HW_AXS_SAIETTA
    uwBr1NtcAvrg = (UWORD)(sThModRun.sImageSlowTask.ulBr1Ntc / sThModRun.sImageMediumTask.uwCnts) ;
	uwBr2NtcAvrg = (UWORD)(sThModRun.sImageSlowTask.ulBr2Ntc / sThModRun.sImageMediumTask.uwCnts) ;
#else
    if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge1Valid)
    {
        uwBr1NtcAvrg = (UWORD)(sThModRun.sImageSlowTask.ulBr1Ntc / sThModRun.sImageMediumTask.uwCnts) ;
        if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge2Valid)
            uwBr2NtcAvrg = (UWORD)(sThModRun.sImageSlowTask.ulBr2Ntc / sThModRun.sImageMediumTask.uwCnts) ;
    }
#endif // _hw_axs_SAIETTA

#ifdef _CRS_DBG
	sTm_ThModOut.uwNtcCount_0 = uwNtcAvrg ;
	sTm_ThModOut.uwNtcCount_1 = uwBr1NtcAvrg;
#endif // _crs_dbg

#endif // _hw_dc
#endif // _infineon_

    if(((uwNtcAvrg < NTC_IGBT_FAULT_THRESHOLD)
#ifdef _HW_DC
        || uwBr1NtcAvrg < NTC_IGBT_FAULT_THRESHOLD || uwBr2NtcAvrg < NTC_IGBT_FAULT_THRESHOLD
#endif
        ) && !sThModRun.sErrorSent.b.bNtcIgbtFail)
    {   // A2D reads less then 20 counts...
        sThModRun.sErrorSent.b.bNtcIgbtFail = TRUE ;
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_TM_THERMALMODEL_FAIL, SYSTEMALARMS_SUBCODE_NTC_IGBT_FAIL, TRUE) ;
        (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
    }

    /* manage Igbt Ntc Vref value */
    if ((sIOMgrAnalogMeasurements.swBridgeTempSensorCalibratonScale != 0) && (sThModRun.flDacFullScaleCorrected == 0.0))
    {
#if defined(_HW_AXS_DAYCO22KW)
    	// manage hw for DAC
    	sThModRun.flDacFullScaleCorrected = 8192.0 / (FLOAT)sIOMgrAnalogMeasurements.swBridgeTempSensorCalibratonScale ;
#else
    	sThModRun.flDacFullScaleCorrected = DACFULLSCALE_CORRECTED / (FLOAT)sIOMgrAnalogMeasurements.swBridgeTempSensorCalibratonScale ;
#endif
    }

#ifndef _HW_DC
    sThModRun.flTmp = sThModRun.HwParam.flNtcMaterialConstant + TWENTYFIVECELSIUSINKELVIN * log((sThModRun.HwParam.flNtcVDividerResistor * (sThModRun.flDacFullScaleCorrected - uwNtcAvrg) - (FLOAT)uwNtcAvrg*2200) / (sThModRun.HwParam.flNtc25DegreeResistance * uwNtcAvrg)) ;
    flTntc = ((sThModRun.HwParam.flNtcMaterialConstant * TWENTYFIVECELSIUSINKELVIN) / sThModRun.flTmp) - ZEROCELSIUSINKELVIN ; /* Celsius */
#else	// !_HW_DC

#if defined(_HW_AXS_SAIETTA)
    // Temp_IGBT_A
    flVAdc = (uwNtcAvrg * 1.0/*VRef*/) / sThModRun.flDacFullScaleCorrected;
    flREqv = sThModRun.HwParam.flNtcVDividerResistor * flVAdc / (3.0 - flVAdc);
    flRNtc = flREqv * 6200.0/*Rparallel*/ / (6200.0 - flREqv);

    sThModRun.flTmp = sThModRun.HwParam.flNtcMaterialConstant * TWENTYFIVECELSIUSINKELVIN / (sThModRun.HwParam.flNtcMaterialConstant + (TWENTYFIVECELSIUSINKELVIN * log(flRNtc / sThModRun.HwParam.flNtc25DegreeResistance)));
    flTntc = sThModRun.flTmp - ZEROCELSIUSINKELVIN; /* Celsius */

    atomic_write(&sTm_ThModOut.flBr0TNtc, &flTntc, sizeof(FLOAT)); /* Celsius */

    // Temp_IGBT_B
    flVAdc = (uwBr1NtcAvrg * 1.0/*VRef*/) / sThModRun.flDacFullScaleCorrected;
    flREqv = sThModRun.HwParam.flNtcVDividerResistor * flVAdc / (3.0 - flVAdc);
    flRNtc = flREqv * 6200.0/*Rparallel*/ / (6200.0 - flREqv);

    sThModRun.flTmp = sThModRun.HwParam.flNtcMaterialConstant * TWENTYFIVECELSIUSINKELVIN / (sThModRun.HwParam.flNtcMaterialConstant + (TWENTYFIVECELSIUSINKELVIN * log(flRNtc / sThModRun.HwParam.flNtc25DegreeResistance)));
    flBr1Tntc = sThModRun.flTmp - ZEROCELSIUSINKELVIN; /* �C */

	atomic_write(&sTm_ThModOut.flBr1TNtc, &flBr1Tntc, sizeof(FLOAT)); /* �C */

	if(flBr1Tntc>flTntc)
		flTntc=flBr1Tntc;

	// Temp_IGBT_C
    flVAdc = (uwBr2NtcAvrg * 1.0/*VRef*/) / sThModRun.flDacFullScaleCorrected;
    flREqv = sThModRun.HwParam.flNtcVDividerResistor * flVAdc / (3.0 - flVAdc);
    flRNtc = flREqv * 6200.0/*Rparallel*/ / (6200.0 - flREqv);

    sThModRun.flTmp = sThModRun.HwParam.flNtcMaterialConstant * TWENTYFIVECELSIUSINKELVIN / (sThModRun.HwParam.flNtcMaterialConstant + (TWENTYFIVECELSIUSINKELVIN * log(flRNtc / sThModRun.HwParam.flNtc25DegreeResistance)));
    flBr2Tntc = sThModRun.flTmp - ZEROCELSIUSINKELVIN; /* Celsius */

	atomic_write(&sTm_ThModOut.flBr2TNtc, &flBr2Tntc, sizeof(FLOAT)); /*Celsius */

	if(flBr2Tntc>flTntc)
		flTntc=flBr2Tntc;
		
#elif defined(_HW_AXS_CABI35KW)
	/*
	 * T = (B * T25) / (B + T25*log(Rntc/R25))
	 * 	Rntc = Vadc * 2.0 / 0.000301A
	 * 	Vadc = Vcnt * 1V/2^16
	 */
	flVAdc = uwNtcAvrg / sThModRun.flDacFullScaleCorrected;
    flRNtc = flVAdc * 2.0 / CABI35KW_NTC_CURR_A;

    sThModRun.flTmp = sThModRun.HwParam.flNtcMaterialConstant * TWENTYFIVECELSIUSINKELVIN / (sThModRun.HwParam.flNtcMaterialConstant + (TWENTYFIVECELSIUSINKELVIN * log(flRNtc / sThModRun.HwParam.flNtc25DegreeResistance)));
    flTntc = sThModRun.flTmp - ZEROCELSIUSINKELVIN; /* Celsius */

    atomic_write(&sTm_ThModOut.flBr0TNtc, &flTntc, sizeof(FLOAT)); /* Celsius */

#else

#ifdef _HW_AXS_DAYCO22KW
	// Ntc_Va2d = (1.0 / 65536.0) * NtcCount = DAC_count * [1.0V (Vref ZynQ) / 65536 (fondoscala ZynQ)]
	// Ntc_I    = 3.0 * Ntc_Va2d / NtcVDividerResistor ; // x 3: hw schematics
	// Rntc     = (Ntc_HwVcc - 3.0 * Ntc_Va2d) / Ntc_I = // Ntc_HwVcc = 5.0V and x 3: hw schematics
	//          = NtcVDividerResistor * (Ntc_HwVcc / (3.0 * Ntc_Va2d) - 1) =
	//          = NtcVDividerResistor * (Ntc_HwVcc / (3.0 * (1.0 / 65536) * NtcCount - 1) =
	//          = NtcVDividerResistor * (NTC_ADC2RESISTOR / NtcCount - 1)
	// NTC_ADC2RESISTOR = Ntc_HwVcc / (3.0 * (1.0 / 65536)) ;
	flBr0_Rntc = sThModRun.HwParam.flNtcVDividerResistor * (NTC_ADC2RESISTOR / (sThModRun.flDacFullScaleCorrected * (FLOAT)uwNtcAvrg) - 1.0) ;

	if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge1Valid)
	{
		flBr1_Rntc = sThModRun.HwParam.flNtcVDividerResistor * (NTC_ADC2RESISTOR / (sThModRun.flDacFullScaleCorrected * (FLOAT)uwBr1NtcAvrg) - 1.0) ;

		if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge2Valid)
			flBr2_Rntc = sThModRun.HwParam.flNtcVDividerResistor * (NTC_ADC2RESISTOR / (sThModRun.flDacFullScaleCorrected * (FLOAT)uwBr2NtcAvrg) - 1.0) ;
	}

#else
	// AxN-DC
	flBr0_Rntc = sThModRun.HwParam.flNtcVDividerResistor * (FLOAT)(sThModRun.flDacFullScaleCorrected - uwNtcAvrg) / (FLOAT)uwNtcAvrg ;
	flBr1_Rntc = sThModRun.HwParam.flNtcVDividerResistor * (FLOAT)(sThModRun.flDacFullScaleCorrected - uwBr1NtcAvrg) / (FLOAT)uwBr1NtcAvrg ;
	flBr2_Rntc = sThModRun.HwParam.flNtcVDividerResistor * (FLOAT)(sThModRun.flDacFullScaleCorrected - uwBr2NtcAvrg) / (FLOAT)uwBr2NtcAvrg ;
#endif // _hw_axs_dayco22kw

	sThModRun.flTmp = sThModRun.HwParam.flNtcMaterialConstant + TWENTYFIVECELSIUSINKELVIN * log(flBr0_Rntc / sThModRun.HwParam.flNtc25DegreeResistance) ;
	flTntc = ((sThModRun.HwParam.flNtcMaterialConstant * TWENTYFIVECELSIUSINKELVIN) / sThModRun.flTmp) - ZEROCELSIUSINKELVIN ; /* Celsius */

    atomic_write(&sTm_ThModOut.flBr0TNtc, &flTntc, sizeof(FLOAT)); /* Celsius */

    if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge1Valid)
    {
		sThModRun.flTmp = sThModRun.HwParam.flNtcMaterialConstant + TWENTYFIVECELSIUSINKELVIN * log(flBr1_Rntc / sThModRun.HwParam.flNtc25DegreeResistance) ;
        flBr1Tntc = ((sThModRun.HwParam.flNtcMaterialConstant * TWENTYFIVECELSIUSINKELVIN) / sThModRun.flTmp) - ZEROCELSIUSINKELVIN ; /* Celsius */

        atomic_write(&sTm_ThModOut.flBr1TNtc, &flBr1Tntc, sizeof(FLOAT)); /* Celsius */

        if(flBr1Tntc>flTntc)
            flTntc=flBr1Tntc;

        if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge2Valid)
        {
			sThModRun.flTmp = sThModRun.HwParam.flNtcMaterialConstant + TWENTYFIVECELSIUSINKELVIN * log(flBr2_Rntc / sThModRun.HwParam.flNtc25DegreeResistance) ;
            flBr2Tntc = ((sThModRun.HwParam.flNtcMaterialConstant * TWENTYFIVECELSIUSINKELVIN) / sThModRun.flTmp) - ZEROCELSIUSINKELVIN ; /* Celsius */
    
            atomic_write(&sTm_ThModOut.flBr2TNtc, &flBr2Tntc, sizeof(FLOAT)); /* Celsius */
    
            if(flBr2Tntc>flTntc)
                flTntc=flBr2Tntc;
        }
    }
#endif
#endif // !_hw_dc

    sThModRun.sLossesIgbtCommon.swTNtc = (SWORD)(flTntc * 10.0) ; /* 1e-1 Celsius */

    // output
    sTm_ThModOut.sOutValue.swTNtc = sThModRun.sLossesIgbtCommon.swTNtc ;
    atomic_write(&sTm_ThModOut.flTNtc, &flTntc, sizeof(FLOAT)); /* Celsius */
}

#ifdef _HW_CT
/* ========================================================================= */
// Evaluation of the On board NTC temperature (solving equation)
// Rntc  = Rdivider * (1024/Va2dAvrg - 1)
// Tigbt = (NtcMatConst * 298.15) / (NtcMatConst + 298.15 * ln(Rntc/Rntc25?) - 273.15
// IT CAN BE EXECUTED IN THE SLOW TASK (TO SAVE MICRO TIME)
static void BrdTNtc(void)
{    
    UWORD uwBrdNtcAvrg ;
    FLOAT flBrdTntc;
    
    if(sThModRun.HwParam.flBrdNtc25DegreeResistance==0 || sThModRun.HwParam.flBrdNtcVDividerResistor==0)
        return;
                
    uwBrdNtcAvrg = (UWORD)(32 * sThModRun.sImageSlowTask.ulBrdNtc / sThModRun.sImageMediumTask.uwCnts) ;

    sThModRun.flTmp = sThModRun.HwParam.flBrdNtcMaterialConstant + TWENTYFIVECELSIUSINKELVIN * log((sThModRun.HwParam.flBrdNtcVDividerResistor * (DACFULLSCALE - uwBrdNtcAvrg)) / (sThModRun.HwParam.flBrdNtc25DegreeResistance * uwBrdNtcAvrg)) ;
    flBrdTntc = ((sThModRun.HwParam.flBrdNtcMaterialConstant * TWENTYFIVECELSIUSINKELVIN) / sThModRun.flTmp) - ZEROCELSIUSINKELVIN ; /* Celsius */

    // output
    sTm_ThModOut.swBrdTemp = (SWORD)(flBrdTntc * 10.0) ; /* 1e-1 Celsius */
    atomic_write(&sTm_ThModOut.flBrdTntc, &flBrdTntc, sizeof(FLOAT)); /* Celsius */
}
#endif

/* ========================================================================= */
/* Output = (A0 * Input + A1 * Input_1 + A2 * Input_2 - B1 * Output_1 - B0 * Output_2 ) / B2 */
#ifdef _INFINEON_
static SWORD BiQuadFilter48Bit(SWORD swInputVal, THERMAL_MODEL_BIQUAD huge * psFilterStatus)
#else
static SWORD BiQuadFilter48Bit(SWORD swInputVal, THERMAL_MODEL_BIQUAD * psFilterStatus)
#endif // _infineon_
{
    SQWRD sqInput48, sqOutput48 ;
    SQWRD sqTmp48_0, sqTmp48_1, sqTmp48_2 ;
    SLONG slInput32 ;
    SWORD swTmp ;
    SLONG slTmp ;
    SLONG slOutputVal ;

    // input (sempre positivo perche' e' una potenza)
    slInput32 = BIQUAD_COEFF_A0 * (SLONG)swInputVal + BIQUAD_COEFF_A1 * (SLONG)psFilterStatus->swX_1 + BIQUAD_COEFF_A2 * (SLONG)psFilterStatus->swX_2 ;
    INT64_ASSIGN(sqInput48, slInput32>>16, (ULONG)slInput32<<16) ; // Input48 = Input32 * 65536    

    //outputs
    slTmp = psFilterStatus->slY_1;
    swTmp = BIQUAD_COEFF_B1 ;
    _sint64_mul_32_16(&sqTmp48_1, &slTmp, &swTmp) ; // Tmp48_1 = Output32_1 * Coeff16_B1
    slTmp = psFilterStatus->slY_2;
    swTmp = BIQUAD_COEFF_B0 ;
    _sint64_mul_32_16(&sqTmp48_0, &slTmp, &swTmp) ; // Tmp48_0 = Output32_2 * Coeff16_B0

    _sint64_add(&sqTmp48_2, &sqTmp48_1, &sqTmp48_0) ;  // sqTmp48_2  = sqTmp48_1 + sqTmp48_0
    _sint64_sub(&sqOutput48, &sqInput48, &sqTmp48_2) ; // sqOutput48 = sqInput48 - sqTmp48_2
    _sint64_shr(&sqOutput48, 14) ;                     // sqOutput48 / Coeff16_B2
    slOutputVal = (SLONG)INT64_LLONG(sqOutput48) ;     // stato del filtro: sono i 32 bit piu' bassi

    // regressioni
    psFilterStatus->swX_2 = psFilterStatus->swX_1 ; // sample i-2
    psFilterStatus->swX_1 = swInputVal ;            // sample i-1

    psFilterStatus->slY_2 = psFilterStatus->slY_1 ;
    psFilterStatus->slY_1 = slOutputVal ; 

    return (SWORD)(slOutputVal * 10 / 65536) ; // 1e-1 W
}


/* ========================================================================= */
// Evaluation of the IGBT conduction + commutation losses and junction temperature
// PjConduction = VceSat * IAvrg * ConductionMargin
// PjCommutation = (Vdc/V_NORMALIZATION) * (IAvrg/IValueLosses) * (CommutationLosses * SwitchingFrequency) * CommutationMargin
// Pj = PjConduction + PjCommutation = IAvrg * PjFactor
// Tntc = Tntc + Rj2cBiQuad * BiQuad(Pj)
// Tj (filtered) = Pj * Rj2c + Tntc
#ifdef _INFINEON_
static void IgbtLosses(THERMAL_MODEL_LOSSES_IGBT_COMMON huge * psLossesIgbtCommon, THERMAL_MODEL_LOSSES_IGBT_SINGLE huge * psLossesIgbtSingle, SLONG slIpk, SLONG slIpkIGBTsHigh2Switch)
#else
static void IgbtLosses(THERMAL_MODEL_LOSSES_IGBT_COMMON * psLossesIgbtCommon, THERMAL_MODEL_LOSSES_IGBT_SINGLE * psLossesIgbtSingle, SLONG slIpk, SLONG slIpkIGBTsHigh2Switch)
#endif // _infineon_
{
    UWORD uwSwitchingFrequency=0 ;
    SWORD swTNtcBiQuad ;
    FLOAT flPjFactor ;
    SLONG slAbsIpk ;

    slAbsIpk = labs(slIpk) ; // it is needed a positive current value

    if (psLossesIgbtCommon->flags.b.bIgbtLow && psLossesIgbtCommon->flags.b.b2Switch && (slAbsIpk == 0))
    {   // if 2 switch modulation, Low Igbt and Iigbt = 0, consider ONLY Conduction losses
        flPjFactor = psLossesIgbtCommon->flVceSat ;
        slAbsIpk = labs(slIpkIGBTsHigh2Switch) ; //current to use is the sum of the currents of the 2 high IGBTs
    }
    else
    {   // Switching Frequency (kHz)
        switch(sTm_ThModIn.psMotorData->ubActualPwmFrequency)
        {
            case MH_PWMFREQ_0KHZ:
                if (sThModRun.flags.b.bAlmaPS)
                {   // only conduction losses (no commutation losses)
                    uwSwitchingFrequency=0; 
                    break;
                }
            case MH_PWMFREQ_2KHZ:
                uwSwitchingFrequency=2;
                break;
            case MH_PWMFREQ_4KHZ:
                uwSwitchingFrequency=4;
                break;
            case MH_PWMFREQ_8KHZ:
                uwSwitchingFrequency=8;
                break;
            case MH_PWMFREQ_16KHZ:
                uwSwitchingFrequency=16;
                break;
            case MH_PWMFREQ_32KHZ:
                uwSwitchingFrequency=32;
                break;
            case MH_PWMFREQ_64KHZ:
                uwSwitchingFrequency=64;
                break;
        }
   
        // PjFactor =(VceSat * ConductionMargin) + (Vdc * SwitchingFrequency) * [(CommutationLosses * CommutationMargin) / (V_NORMALIZATION * IValueLosses)]
        flPjFactor = (psLossesIgbtCommon->flVceSat + (FLOAT)((SLONG)psLossesIgbtCommon->uwVdc * uwSwitchingFrequency) * psLossesIgbtCommon->flCommutationLosses) ; 
    }

    // PjIgbt = ((PjFactor * TjMaxFactor * AbsIpk) / Counts) / 65536 (W)
    psLossesIgbtSingle->swPjIgbt = (SWORD)((SLONG)(flPjFactor * (FLOAT)sThModRun.sLossesIgbtCommon.slTjMaxFactor * (FLOAT)slAbsIpk / (FLOAT)psLossesIgbtCommon->uwCnts + 0.5) / 65536) ; // W 
 
    /* Phase Junction Temperature (1e-1 Celsius) */
    if (bDisableBiQuad)
        swTNtcBiQuad = psLossesIgbtCommon->swTNtc ;
    else
        swTNtcBiQuad = (SWORD)((SLONG)psLossesIgbtCommon->swTNtc + ((SLONG)psLossesIgbtCommon->uwBiQuadNtc2JunctionThResist * BiQuadFilter48Bit(psLossesIgbtSingle->swPjIgbt, &(psLossesIgbtSingle->sBiQuadFilterStatus))) / 1000) ;  //BiQuad: Power 1e-1W

    psLossesIgbtSingle->slTjIgbt += ((SLONG)psLossesIgbtCommon->uwNtc2JunctionThResist * psLossesIgbtSingle->swPjIgbt / 100 + swTNtcBiQuad - psLossesIgbtSingle->slTjIgbt/65536) * psLossesIgbtCommon->uwIgbtTimeConstant ;
    psLossesIgbtSingle->swTjIgbt  = (SWORD)(psLossesIgbtSingle->slTjIgbt / 65536) ;
}

/* ========================================================================= */
// IGBT losses Vdc adjustment
static void AdjIgbtLosses(void)
{
    FLOAT flTmp;

    if(sThModRun.sLossesIgbtCommon.uwVdc>COMMUTATION_VDC_OFFSET)
        flTmp=COMMUTATION_MARGIN+(sThModRun.sLossesIgbtCommon.uwVdc-COMMUTATION_VDC_OFFSET)*COMMUTATION_VDC_RATIO;
    else
        flTmp=COMMUTATION_MARGIN;
    sTm_ThModOut.sOutValue.uwCommMargin=(UWORD)(flTmp*10.0);
    flTmp=sThModRun.sLossesIgbtCommon.flPreCommutationLosses*flTmp;
    atomic_write(&sThModRun.sLossesIgbtCommon.flCommutationLosses, &flTmp, sizeof(FLOAT)) ;
}

/* ========================================================================= */
// Evaluation of the IGBT losses and junction temperature
// IT MUST BE DONE AS FAST AS POSSIBLE
static void IgbtModuleLosses(void) 
{
    SWORD swTjMax ;

    sThModRun.sLossesIgbtCommon.uwCnts = sThModRun.sImageMediumTask.uwCnts ;
    sThModRun.sLossesIgbtCommon.uwVdc = (UWORD)(sThModRun.sImageMediumTask.slDcBus / sThModRun.sImageMediumTask.uwCnts) ; /* 1e-1V */
    sThModRun.sLossesIgbtCommon.uwIgbtTimeConstant = (UWORD)((FLOAT)sThModRun.sImageMediumTask.uwCnts * sThModRun.HwParam.flIgbtThCapacitance) ; // IgbtThCapacitance = (8000 * IgbtThCapacitance * Ntc2JunctionThResist) / 65536 (code optimization)

    // --------------------- modulation frequency and strategy ---------------------
    sThModRun.sLossesIgbtCommon.flags.b.b2Switch = (BOOL)(sTm_ThModIn.psMotorData->uwActualPwmSetup&MH_CMD_SETMOD_ENABLE2STEPS);
    // -----------------------------------------------------------------------------
    
    // HIGH igbts
    sThModRun.sLossesIgbtCommon.flags.b.bIgbtLow = (UWORD)FALSE ;
  
    IgbtLosses(&sThModRun.sLossesIgbtCommon, &sThModRun.sLossesIgbtUHi, sThModRun.sImageMediumTask.slIuPkHi, 0l) ;
    IgbtLosses(&sThModRun.sLossesIgbtCommon, &sThModRun.sLossesIgbtVHi, sThModRun.sImageMediumTask.slIvPkHi, 0l) ;
    IgbtLosses(&sThModRun.sLossesIgbtCommon, &sThModRun.sLossesIgbtWHi, sThModRun.sImageMediumTask.slIwPkHi, 0l) ;
    
    // LOW igbts
    sThModRun.sLossesIgbtCommon.flags.b.bIgbtLow = (UWORD)TRUE ;
    
    IgbtLosses(&sThModRun.sLossesIgbtCommon, &sThModRun.sLossesIgbtULo, sThModRun.sImageMediumTask.slIuPkLo, sThModRun.sImageMediumTask.slIvPkHi + sThModRun.sImageMediumTask.slIwPkHi) ;
    IgbtLosses(&sThModRun.sLossesIgbtCommon, &sThModRun.sLossesIgbtVLo, sThModRun.sImageMediumTask.slIvPkLo, sThModRun.sImageMediumTask.slIuPkHi + sThModRun.sImageMediumTask.slIwPkHi) ;
    IgbtLosses(&sThModRun.sLossesIgbtCommon, &sThModRun.sLossesIgbtWLo, sThModRun.sImageMediumTask.slIwPkLo, sThModRun.sImageMediumTask.slIuPkHi + sThModRun.sImageMediumTask.slIvPkHi) ;
    
    // workaround to manage ALMA Power Supply hardware
    if(sMh_PlcAdvancedWorks.flags.b.bSelEnOutEnable && sThModRun.flags.b.bAlmaPS)
    {
        if (!(BOOL)(sMh_PlcAdvancedWorks.uwSelEnOutMask & IGBT_ENABLEMASK_UHI) && !(BOOL)(sMh_PlcAdvancedWorks.uwSelEnOutMask & IGBT_ENABLEMASK_ULO))
        {
            sThModRun.sLossesIgbtUHi.swPjIgbt = sThModRun.sLossesIgbtULo.swPjIgbt = 0 ;
            sThModRun.sLossesIgbtUHi.swTjIgbt = sThModRun.sLossesIgbtULo.swTjIgbt = 0 ;
        }

        if (!(BOOL)(sMh_PlcAdvancedWorks.uwSelEnOutMask & IGBT_ENABLEMASK_VHI) && !(BOOL)(sMh_PlcAdvancedWorks.uwSelEnOutMask & IGBT_ENABLEMASK_VLO))
        {
            sThModRun.sLossesIgbtVHi.swPjIgbt = sThModRun.sLossesIgbtVLo.swPjIgbt = 0 ;
            sThModRun.sLossesIgbtVHi.swTjIgbt = sThModRun.sLossesIgbtVLo.swTjIgbt = 0 ;
        }

        if (!(BOOL)(sMh_PlcAdvancedWorks.uwSelEnOutMask & IGBT_ENABLEMASK_WHI) && !(BOOL)(sMh_PlcAdvancedWorks.uwSelEnOutMask & IGBT_ENABLEMASK_WLO))
        {
            sThModRun.sLossesIgbtWHi.swPjIgbt = sThModRun.sLossesIgbtWLo.swPjIgbt = 0 ;
            sThModRun.sLossesIgbtWHi.swTjIgbt = sThModRun.sLossesIgbtWLo.swTjIgbt = 0 ;
        }
    }

    /* Total power dissipated by the module (W) */
    sThModRun.sLossesIgbtCommon.slPjTot = (SLONG)sThModRun.sLossesIgbtUHi.swPjIgbt + sThModRun.sLossesIgbtVHi.swPjIgbt + sThModRun.sLossesIgbtWHi.swPjIgbt + sThModRun.sLossesIgbtULo.swPjIgbt + sThModRun.sLossesIgbtVLo.swPjIgbt + sThModRun.sLossesIgbtWLo.swPjIgbt ;
    
    /* looking for the higher junction temperature (1e-1 Celsius) */
    swTjMax = sThModRun.sLossesIgbtUHi.swTjIgbt ;
    
    if (swTjMax < sThModRun.sLossesIgbtVHi.swTjIgbt)
        swTjMax = sThModRun.sLossesIgbtVHi.swTjIgbt ;
    
    if (swTjMax < sThModRun.sLossesIgbtWHi.swTjIgbt)
        swTjMax = sThModRun.sLossesIgbtWHi.swTjIgbt ;
    
    if (swTjMax < sThModRun.sLossesIgbtULo.swTjIgbt)
        swTjMax = sThModRun.sLossesIgbtULo.swTjIgbt ;
    
    if (swTjMax < sThModRun.sLossesIgbtVLo.swTjIgbt)
        swTjMax = sThModRun.sLossesIgbtVLo.swTjIgbt ;
    
    if (swTjMax < sThModRun.sLossesIgbtWLo.swTjIgbt)
        swTjMax = sThModRun.sLossesIgbtWLo.swTjIgbt ;
    
    /* Tj can't be lower than Tntc */
    if(swTjMax < sThModRun.sLossesIgbtCommon.swTNtc)
        swTjMax = sThModRun.sLossesIgbtCommon.swTNtc ;

//    sTm_ThModOut.sOutValue.swTjMax = (7 * sTm_ThModOut.sOutValue.swTjMax + swTjMax) / 8 ; // 1e-1 Celsius, filter 7/8
    sThModRun.slTjMaxFiltSts = (7 * sThModRun.slTjMaxFiltSts) / 8 + swTjMax ;
    sTm_ThModOut.sOutValue.swTjMax = (SWORD)(sThModRun.slTjMaxFiltSts / 8) ; // 1e-1 Celsius, filter 7/8 without offset

    // TjFactor = 1 + 0.003 * (TjMax(Celsius) - 130(Celsius)) = 65536 + 3 * 65536 * (TjMax(1e-1 Celsius) - 1300(1e-1 Celsius)) / 10000
    if (sTm_ThModOut.sOutValue.swTjMax > 1300)
        sThModRun.sLossesIgbtCommon.slTjMaxFactor = 65536L + 196608L * (SLONG)(sTm_ThModOut.sOutValue.swTjMax - 1300) / 10000 ;
    else
        sThModRun.sLossesIgbtCommon.slTjMaxFactor = 65536L ;

    // output
    sTm_ThModOut.sOutValue.swTjUHi   = sThModRun.sLossesIgbtUHi.swTjIgbt ; /* 1e-1 Celsius */
    sTm_ThModOut.sOutValue.swTjVHi   = sThModRun.sLossesIgbtVHi.swTjIgbt ; /* 1e-1 Celsius */
    sTm_ThModOut.sOutValue.swTjWHi   = sThModRun.sLossesIgbtWHi.swTjIgbt ; /* 1e-1 Celsius */
    sTm_ThModOut.sOutValue.swTjULo   = sThModRun.sLossesIgbtULo.swTjIgbt ; /* 1e-1 Celsius */
    sTm_ThModOut.sOutValue.swTjVLo   = sThModRun.sLossesIgbtVLo.swTjIgbt ; /* 1e-1 Celsius */
    sTm_ThModOut.sOutValue.swTjWLo   = sThModRun.sLossesIgbtWLo.swTjIgbt ; /* 1e-1 Celsius */
    sTm_ThModOut.sOutValue.swPjUHTot = sThModRun.sLossesIgbtUHi.swPjIgbt ; /* W */
    sTm_ThModOut.sOutValue.swPjVHTot = sThModRun.sLossesIgbtVHi.swPjIgbt ; /* W */
    sTm_ThModOut.sOutValue.swPjWHTot = sThModRun.sLossesIgbtWHi.swPjIgbt ; /* W */
    sTm_ThModOut.sOutValue.swPjULTot = sThModRun.sLossesIgbtULo.swPjIgbt ; /* W */
    sTm_ThModOut.sOutValue.swPjVLTot = sThModRun.sLossesIgbtVLo.swPjIgbt ; /* W */
    sTm_ThModOut.sOutValue.swPjWLTot = sThModRun.sLossesIgbtWLo.swPjIgbt ; /* W */

    sThModRun.flTmp = (FLOAT)sTm_ThModOut.sOutValue.swTjMax / 10.0 ; /* Celsius */
    atomic_write(&sTm_ThModOut.flTjMax, &sThModRun.flTmp, sizeof(FLOAT)); /* W */
    atomic_write(&sTm_ThModOut.sOutValue.slPjTot, &sThModRun.sLossesIgbtCommon.slPjTot, sizeof(SLONG)); /* W */

    // output for the Alarm History (1e-4A) and for Motor I2T
    // NOTE: necessary to change sign to IPkLo since IPkHi always > 0 and IPkLo always < 0
    sThModRun.slTmp = UMCONV_CONVERT_16TO32(&sTm_Fpga2Internal_I_PEAK, (SWORD)((sThModRun.sImageMediumTask.slIuPkHi - sThModRun.sImageMediumTask.slIuPkLo) / sThModRun.sImageMediumTask.uwCnts)) ; 
    atomic_write(&sTm_ThModOut.sOutValue.slIuPeakAvrg, &sThModRun.slTmp, sizeof(SLONG));

    sThModRun.slTmp = UMCONV_CONVERT_16TO32(&sTm_Fpga2Internal_I_PEAK, (SWORD)((sThModRun.sImageMediumTask.slIvPkHi - sThModRun.sImageMediumTask.slIvPkLo) / sThModRun.sImageMediumTask.uwCnts)) ;
    atomic_write(&sTm_ThModOut.sOutValue.slIvPeakAvrg, &sThModRun.slTmp, sizeof(SLONG));

    sThModRun.slTmp = UMCONV_CONVERT_16TO32(&sTm_Fpga2Internal_I_PEAK, (SWORD)((sThModRun.sImageMediumTask.slIwPkHi - sThModRun.sImageMediumTask.slIwPkLo) / sThModRun.sImageMediumTask.uwCnts)) ;
    atomic_write(&sTm_ThModOut.sOutValue.slIwPeakAvrg, &sThModRun.slTmp, sizeof(SLONG));

    sTm_ThModOut.sOutValue.swDcBusAvrg = (SWORD)sThModRun.sLossesIgbtCommon.uwVdc ; /* 1e-1 V */

}

/* ========================================================================= */
// current limit based on IGBT max Tjunction
// IT MUST BE DONE AS FAST AS POSSIBLE
static void IgbtCurrentLimit(void)
{
    SWORD swRatio ; 

    if (sTm_ThModOut.sOutValue.swTjMax >= sThModRun.swILimitMinTemp) 
    {   // Reduce current starting from MaxTJunction - 30 (Celsius) (swILimitMinTemp)
        // I limit = 0 when LocTjMax = swIgbtMaximumTemp - 5 Celsius = (MaxTJunction - 15) - 5 = MaxTJunction - 20
        // Delta temperature = 10 (Celsius) = 100 (1e-1 Celsius)
        swRatio = (SWORD)((SLONG)(sThModRun.swILimitMaxTemp - sTm_ThModOut.sOutValue.swTjMax) * 256 / sThModRun.swILimitDelta) ;
        sThModRun.sIdLocLim.slMax = (sThModRun.slIdChipCurrLimit * swRatio) / 256 ; /* 1e-4A */

        // to avoid surprise since sTm_ThModOut.sOutValue.swTjMax is filtered
        if(sThModRun.sIdLocLim.slMax < 0)   
            sThModRun.sIdLocLim.slMax = 0 ; 

        if(sThModRun.sIdLocLim.slMax > sThModRun.slIdChipCurrLimit)
            sThModRun.sIdLocLim.slMax = sThModRun.slIdChipCurrLimit ;

        // if not disable trigger non fatal alarm
        if(!sThModRun.flags.b.bDisFltReactOnLimit && !sThModRun.sErrorSent.b.bJunctionOverTemp)
        {                          
            sThModRun.sErrorSent.b.bJunctionOverTemp = TRUE ; 
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_TM_THERMALMODEL_FAIL, SYSTEMALARMS_SUBCODE_JUNCTION_OVERTEMP, TRUE) ;
            (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL) ;
        }
    }
    else
        sThModRun.sIdLocLim.slMax = sThModRun.slIdChipCurrLimit ;

    sThModRun.sIqLocLim.slMax =  sThModRun.sIdLocLim.slMax ;
    sThModRun.sIdLocLim.slMin = -sThModRun.sIdLocLim.slMax ;
    sThModRun.sIqLocLim.slMin =  sThModRun.sIdLocLim.slMin ;

    
    /* Bridge Over Temperature */
    if (sTm_ThModOut.sOutValue.swTjMax >= sThModRun.HwParam.swIgbtMaximumTemp)
    {
        if (!sThModRun.sErrorSent.b.bJunctionOverTemp)
        {
            sThModRun.sErrorSent.b.bJunctionOverTemp = TRUE ; 
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_TM_THERMALMODEL_FAIL, SYSTEMALARMS_SUBCODE_JUNCTION_OVERTEMP, TRUE) ;
        }
        (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
    }
}

/* ========================================================================= */
// Evaluation of heatsink temperature and fan (if any)
// CAN IT BE DONE SLOWER??? (TO SAVE MICRO TIME)
static void DriveHeatsink(void) 
{  
    SWORD swRatio ; 
    UWORD uwHsnkTimeConstant ;
    static BOOL bFirstPowerON = FALSE;
 
    // ################ Fan Fault  (AxM) ################ 
    if (sThModRun.flags.b.bFanFaultChk)
    {   /* necessary to catch the information about the FAN */
        /* Fan locked: this check must be executed before the fan command, otherwise *
         * if I give the command to the fan, I'll read the fan status after 32ms and *
         * this mismatch could be considered an error of NOT working fan             */
        /* NECESSARY to CHECK the FAN TYPE ??!??? */
        if (!sThModRun.sErrorSent.b.bFanLocked)
        {   // FAN_DRIVE and FAN_FAULT must be equal, otherwise FAULT
            if ((FAN_DRIVE_STATUS == TRUE) && (FAN_FAULT == FALSE))
                sThModRun.swFanLockedCounter += 1 ;
            else
            {
                sThModRun.swFanLockedCounter -= 1 ;
    
                if (sThModRun.swFanLockedCounter < 0) 
                    sThModRun.swFanLockedCounter = 0 ;
            }
                    
            if (sThModRun.swFanLockedCounter > 100)
            {   // FAULT if fan is locked for more than 100 thermal model tick
                FAN_DRIVE_SET( FALSE ); // turn off the command to the fan 
                sThModRun.swFanLockedCounter = 0 ;  // ready to re-count if reset fault has been sent
                sThModRun.sErrorSent.b.bFanLocked = TRUE ; // set the fault                                           
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_TM_THERMALMODEL_FAIL, SYSTEMALARMS_SUBCODE_FAN_LOCKED, TRUE) ;
                (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL) ; 
            }
        }                              
    }
    
    // ###################  Heatsink  ###################
    /* heatsink temperature: if watercooled = NTC; if aircooled = calculated */
    if (sThModRun.flags.b.bNoCooling)
        sTm_ThModOut.sOutValue.swTHeatSink = sThModRun.sLossesIgbtCommon.swTNtc ; /* AxW: T_Heatsink = T_Ntc (1e-1 Celsius) */
    else
    {
        SWORD swTmp;

        // /* AxM/AxP: T_Heatsink = T_Ntc - (PjTot * R_Hsnk2Ntc) [without thermal capacitance] */
        if (!sTm_ThModIn.psMotorData->sPowerStageSts.b.bVoltageEnabled&&!bFirstPowerON)
        {
            sThModRun.slTHeatSink = (SLONG)sThModRun.sLossesIgbtCommon.swTNtc * 131072 ; // 1e-1 Celsius * 131072
            bFirstPowerON = TRUE;
        }
    
        // *******************************************************
        // ** AxM/AxP: using Thermal Resistance and Capacitance **
        // *******************************************************
        // 8000 = 1 second in fast task ticks
        // HsnkThCapacitance =  J/K
        // 1/100 = fattore di conversione da uwHsnk2NtcThRes2Use in u.i. a uwHsnk2NtcThRes2Use in K/W
        // 131072 = coefficiente per aumentare la risoluzione
        // Quindi al boot calcolo:
        // flHsnkThCapacitance = (8000 * HsnkThCapacitance) / (131072 * 100) (code optimization) 
        uwHsnkTimeConstant = (UWORD)((FLOAT)sThModRun.sImageMediumTask.uwCnts / (sThModRun.HwParam.flHsnkThCapacitance * (FLOAT)sThModRun.uwHsnk2NtcThRes2Use)) ;  
        
        // slPjTot * uwHsnk2NtcThRes2Use / 10 = 1e-1 Celsius
        // swTNtc = 1e-1 Celsius
        // slTHeatSink / 131072 = 1e-1 Celsius (since slTHeatsink is 1-1 Celsius * 131072)
        sThModRun.slTHeatSink += -((sThModRun.sLossesIgbtCommon.slPjTot * sThModRun.uwHsnk2NtcThRes2Use / 10 - sThModRun.sLossesIgbtCommon.swTNtc + sThModRun.slTHeatSink / 131072) * uwHsnkTimeConstant) ; // 1e-1 Celsius
        swTmp = (SWORD)(sThModRun.slTHeatSink / 131072) ;

        // THeatsink cannot be higher than TNtc
        if (swTmp > sThModRun.sLossesIgbtCommon.swTNtc)
        {
            sTm_ThModOut.sOutValue.swTHeatSink = sThModRun.sLossesIgbtCommon.swTNtc ;
            sThModRun.slTHeatSink = (SLONG)sThModRun.sLossesIgbtCommon.swTNtc * 131072 ; // 1e-1 Celsius * 131072
        }
        else
            sTm_ThModOut.sOutValue.swTHeatSink = swTmp;
    }
    
    /* reduce current when: 
       1) DisablePkPk = FALSE: HsnkMaximumTemp - 10 Celsius < T_HeatSink < HsnkMaximumTemp
       2) DisablePkPk = TRUE:  HsnkMaximumTemp -  5 Celsius < T_HeatSink < HsnkMaximumTemp
    */
    if(sTm_ThModOut.sOutValue.swTHeatSink >= sThModRun.swHsnkDeratingMaximumTemp)
    {   /* reduce current supplied */
        if (bDisablePkPk)
            swRatio = (SWORD)((SLONG)(sThModRun.HwParam.swHsnkMaximumTemp - sTm_ThModOut.sOutValue.swTHeatSink) * 256 / 50) ;
        else
            swRatio = (SWORD)((SLONG)(sThModRun.HwParam.swHsnkMaximumTemp - sTm_ThModOut.sOutValue.swTHeatSink) * 256 / 100) ;
        
        if(swRatio<0)
            swRatio=0;

        sThModRun.sIdLocLim.slMax = (sThModRun.sIdLocLim.slMax * swRatio) / 256 ; /* 1e-4A */ 
        sThModRun.sIqLocLim.slMax =  sThModRun.sIdLocLim.slMax ;
        sThModRun.sIdLocLim.slMin = -sThModRun.sIdLocLim.slMax ;
        sThModRun.sIqLocLim.slMin =  sThModRun.sIdLocLim.slMin ;

        // if not disable trigger non fatal alarm
        if(!sThModRun.flags.b.bDisFltReactOnLimit && !sThModRun.sErrorSent.b.bHeatSinkOverTemp)
        {
            sThModRun.sErrorSent.b.bHeatSinkOverTemp = TRUE ;
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_TM_THERMALMODEL_FAIL, SYSTEMALARMS_SUBCODE_HEATSINK_OVERTEMP, TRUE) ;
            (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL) ;
        }
    } 
    
    /* HeatSink Over Temperature */
    if (sTm_ThModOut.sOutValue.swTHeatSink >= sThModRun.HwParam.swHsnkMaximumTemp)
    {
        if (!sThModRun.sErrorSent.b.bHeatSinkOverTemp)
        {
            sThModRun.sErrorSent.b.bHeatSinkOverTemp = TRUE ;
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_TM_THERMALMODEL_FAIL, SYSTEMALARMS_SUBCODE_HEATSINK_OVERTEMP, TRUE) ;
        }
        (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
    }
    
    /* Bridge Cooling ON/OFF */
    if  (sThModRun.sErrorSent.b.bFanLocked)
    {   // fan locked error detected: unuseful to enable the fan..
        FAN_DRIVE_SET( FALSE ) ;
    }
    else if (sThModRun.flags.b.bFanVSpeed)
    {
        if (sTm_ThModOut.sOutValue.swTHeatSink >= sThModRun.swCoolingTempOff)
        {
            FAN_DRIVE_SET( sTm_ThModOut.sOutValue.swTHeatSink - sThModRun.swCoolingTempOff ) ;
            sThModRun.uwHsnk2NtcThRes2Use = sThModRun.HwParam.uwHsnk2NtcThResCoolingON ;
        }
        else
        {
            FAN_DRIVE_SET( 0 ) ; 
            sThModRun.uwHsnk2NtcThRes2Use = sThModRun.HwParam.uwHsnk2NtcThResCoolingOFF ;
        }
    }
    else
    { 
        if (sTm_ThModOut.sOutValue.swTHeatSink >= sThModRun.swCoolingTempOn)
        {
            FAN_DRIVE_SET( TRUE ) ;
            sThModRun.uwHsnk2NtcThRes2Use = sThModRun.HwParam.uwHsnk2NtcThResCoolingON ;
        }
        else if (sTm_ThModOut.sOutValue.swTHeatSink <= sThModRun.swCoolingTempOff)
        {
            FAN_DRIVE_SET( FALSE ) ; 
            sThModRun.uwHsnk2NtcThRes2Use = sThModRun.HwParam.uwHsnk2NtcThResCoolingOFF ;
        }
    }

    // output
    sTm_ThModOut.flTHeatSink  = (FLOAT)sTm_ThModOut.sOutValue.swTHeatSink * 0.1 ; // Celsius
}



/* ========================================================================= */
// Evaluation of the power dissipated by brake resistor (if any)
// CAN IT BE DONE SLOWER??? (TO SAVE MICRO TIME)
static void BrakeResistor(void)
{           
    FLOAT flRBrakePower ;
    
    /* Power (W) and Energy (J) dissipated in 8->32 ms */
    flRBrakePower = (FLOAT)((ULONG)sThModRun.sLossesIgbtCommon.uwVdc * sThModRun.sLossesIgbtCommon.uwVdc) * sThModRun.sImageMediumTask.uwRBrakeON / (100.0 * sThModRun.HwParam.flRBrakeValue * sThModRun.sImageMediumTask.uwCnts) ;
    sThModRun.flRBrakeEnergy += (flRBrakePower - sThModRun.flRBrakeEnergy * sThModRun.flRBrakeThConst) * (FLOAT)sThModRun.sImageMediumTask.uwCnts / 8000.0 ; // J

    // if not yet overpowered make standard checking
    if(!sThModRun.sErrorSent.b.bBrakeOverPower)
    {
        // if energy reached the limit
        if(sThModRun.flRBrakeEnergy >= (FLOAT)sTm_ThModOut.ulBrakeResistorEnergy)
        {
            sThModRun.sErrorSent.b.bBrakeOverPower = TRUE;

            // signal brake overpower alarm
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_HW_BRAKE_FAIL, SYSTEMALARMS_SUBCODE_BF_OVERPOWER, TRUE);

            // signal NON fatal fault
            (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL);
        }
        // if brake disabled but alarm state was reset then re-enable brake drive
        else if(sThModRun.flags.b.bBrakeDisabled)
        {
            (*sTm_ThModIn.psMotorHandlers->pfForceCommand)(MH_CMD_ENABLEBRAKEDRIVE, 0l);
            sThModRun.flags.b.bBrakeDisabled = FALSE;
        }
    }
    // here when in overpower state, NON fatal fault and alarm already signaled
    else
        // if power still rising (up to 10% more than threshold),
        if(sThModRun.flRBrakeEnergy >= (FLOAT)sTm_ThModOut.ulBrakeResistorEnergy * BRAKE_OVERDRIVE)
        {
            if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bVoltageEnabled)
            {
                // became fatal fault (if not disabled)
                if(!sThModRun.flags.b.bNoFatalOnBrakeFlt)
                    (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
            }
            // when power-off disable brake drive (if not disabled)
            else if(!sThModRun.flags.b.bNoBrakeDisableOnFlt && !sThModRun.flags.b.bBrakeDisabled)
            {
                (*sTm_ThModIn.psMotorHandlers->pfForceCommand)(MH_CMD_DISABLEBRAKEDRIVE, 0l);
                sThModRun.flags.b.bBrakeDisabled = TRUE;
            }

            // and signal brake overpower overdrive
            if(!sThModRun.sErrorSent.b.bBrakeAlwaysOn)
            {
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_HW_BRAKE_FAIL, SYSTEMALARMS_SUBCODE_BF_ALWAYS_ON, TRUE);
                sThModRun.sErrorSent.b.bBrakeAlwaysOn = TRUE;
            }
        }

    // output 
    sThModRun.slTmp = (SLONG)(flRBrakePower * 10.0) ; /* 1e-1 W */
    atomic_write(&sTm_ThModOut.sOutValue.slRBrakePower, &sThModRun.slTmp, sizeof(SLONG));
    sTm_ThModOut.ulOutValueRBrakeEnergy  = (ULONG)sThModRun.flRBrakeEnergy;/* J */
}


/* ========================================================================= */
// Dynamic OverCurrent (filtered)
//IT MUST BE DONE AS FAST AS POSSIBLE
static void DynamicOverCurrent(void)
{
    sThModRun.slTmp = sThModRun.slDynamicOverCurrentDelta ; 

    if (sThModRun.sIdLocLim.slMax < sThModRun.slIdLocLimMax_1)
        sTm_ThModOut.slDynamicOverCurrent = sThModRun.slIdLocLimMax_1 + sThModRun.slTmp ;
    else
        sTm_ThModOut.slDynamicOverCurrent = sThModRun.sIdLocLim.slMax + sThModRun.slTmp ;

    sThModRun.slIdLocLimMax_1 = sThModRun.sIdLocLim.slMax ;

    // scrivo immediatamente il limite di overcurrent da qui
#ifdef _HW_DC
    if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge1Valid)
    {
        if(sTm_ThModIn.psMotorData->sPowerStageSts.b.bBridge2Valid)
            (*sTm_ThModIn.psMotorHandlers->pfForceCommand)(MH_CMD_SETOVERCURRENTLIMIT, (ULONG)sTm_ThModOut.slDynamicOverCurrent / 3);
        else
            (*sTm_ThModIn.psMotorHandlers->pfForceCommand)(MH_CMD_SETOVERCURRENTLIMIT, (ULONG)sTm_ThModOut.slDynamicOverCurrent / 2);
    }
    else
#endif
        (*sTm_ThModIn.psMotorHandlers->pfForceCommand)(MH_CMD_SETOVERCURRENTLIMIT, (ULONG)sTm_ThModOut.slDynamicOverCurrent);
}


/* ========================================================================= */
// Thermal Model Current Limit application
//IT MUST BE DONE AS FAST AS POSSIBLE
static void ThermalModelCurrentLimit(void)
{
    /* ################################################## * 
     * #  write out (atomically) the values to be used  # *
     * ################################################## */
    atomic_write(&sTm_ThModOut.sIdLimit.slMax, &sThModRun.sIdLocLim.slMax, sizeof(sThModRun.sIdLocLim.slMax)) ;
    atomic_write(&sTm_ThModOut.sIdLimit.slMin, &sThModRun.sIdLocLim.slMin, sizeof(sThModRun.sIdLocLim.slMin)) ;
    atomic_write(&sTm_ThModOut.sIqLimit.slMax, &sThModRun.sIqLocLim.slMax, sizeof(sThModRun.sIqLocLim.slMax)) ;
    atomic_write(&sTm_ThModOut.sIqLimit.slMin, &sThModRun.sIqLocLim.slMin, sizeof(sThModRun.sIqLocLim.slMin)) ;

    /* Force limit rescan for immediate applying */
    (*sTm_ThModIn.psMotorHandlers->pfForceLimitListScan)();
}

/* ========================================================================= */
// Adapt modulation type to prevent unbalancing of IGBT temperatures

#define ADMOD_MAXIGBTDELTATEMP              300     // [1/10 Celsius]
#define ADMOD_MINIGBTDELTATEMP              100     // [1/10 Celsius]

#define ADMOD_TEMPTHRESHOLD_LOW             900     // [1/10 Celsius]
#define ADMOD_TEMPTHRESHOLD_HIGH            1000    // [1/10 Celsius]

static void AdaptModulation(void)
{
    SWORD swHiMax,swLoMax;
	SWORD swAllMax,swMaxDelta;
    BOOL bNextEnable=sThModRun.flags.b.bAMEnable;
    BOOL bBelowNomCurrent=FALSE;
    BOOL bAboveNomCurrent=FALSE;

        // get max temp of high IGBT side
    swHiMax=sTm_ThModOut.sOutValue.swTjUHi>sTm_ThModOut.sOutValue.swTjVHi?sTm_ThModOut.sOutValue.swTjUHi:sTm_ThModOut.sOutValue.swTjVHi;
    swHiMax=sTm_ThModOut.sOutValue.swTjWHi>swHiMax                       ?sTm_ThModOut.sOutValue.swTjWHi:swHiMax;

        // get max temp of low IGBT side
    swLoMax=sTm_ThModOut.sOutValue.swTjULo>sTm_ThModOut.sOutValue.swTjVLo?sTm_ThModOut.sOutValue.swTjULo:sTm_ThModOut.sOutValue.swTjVLo;
    swLoMax=sTm_ThModOut.sOutValue.swTjWLo>swLoMax                       ?sTm_ThModOut.sOutValue.swTjWLo:swLoMax;

	    // get absolute max between low and high IGBT
	swAllMax=swHiMax>swLoMax?swHiMax:swLoMax;

        // get max delta between all IGBTs
    swMaxDelta=swAllMax-sTm_ThModOut.sOutValue.swTjUHi;
    swMaxDelta=swAllMax-sTm_ThModOut.sOutValue.swTjVHi>swMaxDelta?swAllMax-sTm_ThModOut.sOutValue.swTjVHi:swMaxDelta;
    swMaxDelta=swAllMax-sTm_ThModOut.sOutValue.swTjWHi>swMaxDelta?swAllMax-sTm_ThModOut.sOutValue.swTjWHi:swMaxDelta;
    swMaxDelta=swAllMax-sTm_ThModOut.sOutValue.swTjULo>swMaxDelta?swAllMax-sTm_ThModOut.sOutValue.swTjULo:swMaxDelta;
    swMaxDelta=swAllMax-sTm_ThModOut.sOutValue.swTjVLo>swMaxDelta?swAllMax-sTm_ThModOut.sOutValue.swTjVLo:swMaxDelta;
    swMaxDelta=swAllMax-sTm_ThModOut.sOutValue.swTjWLo>swMaxDelta?swAllMax-sTm_ThModOut.sOutValue.swTjWLo:swMaxDelta;

        // check for Iq and Id if below nominal current, if so then restore back immediately
    if( (labs(sTm_ThModIn.psMotorData->slIdFb) < sThModRun.slIdChipCompCurrNomLoTh) &&
        (labs(sTm_ThModIn.psMotorData->slIqFb) < sThModRun.slIdChipCompCurrNomLoTh)   )
        bBelowNomCurrent=TRUE;

        // check for Iq and Id if above nominal current
    if( (labs(sTm_ThModIn.psMotorData->slIdFb) > sThModRun.slIdChipCompCurrNomHiTh) ||
        (labs(sTm_ThModIn.psMotorData->slIqFb) > sThModRun.slIdChipCompCurrNomHiTh)   )
        bAboveNomCurrent=TRUE;

        // if not already enabled
    if(!sThModRun.flags.b.bAMEnable)
    {       // check for max delta Tj and absolute Tj max and above nominal current
        if(swMaxDelta>ADMOD_MAXIGBTDELTATEMP && sTm_ThModOut.sOutValue.swTjMax>ADMOD_TEMPTHRESHOLD_HIGH && bAboveNomCurrent)
            bNextEnable=TRUE;
    }
    else
    {       // check for min delta Tj or absolute Tj max or below nominal current
        if(swMaxDelta<ADMOD_MINIGBTDELTATEMP || sTm_ThModOut.sOutValue.swTjMax<ADMOD_TEMPTHRESHOLD_LOW || bBelowNomCurrent)
            bNextEnable=FALSE;
    }

        // if changed then setup
    if(bNextEnable != sThModRun.flags.b.bAMEnable)
    {
        (*sTm_ThModIn.psMotorHandlers->pfForceCommand)(MH_CMD_FORCELOWFREQUENCY, (ULONG)bNextEnable);
        sThModRun.flags.b.bAMEnable=bNextEnable;
    }
}

/* ========================================================================= */
// Motor I2T protection - Init
static void MotorI2T_Init(void)
{
    FLOAT flMotorINominal2Use ; 
    sThModRun.sMotorI2T.uwTimer_1 = 0 ;

#ifdef __MOTOR_I2T_IN_SLOWTASK__   
    // filter executed in slowtask 
    // OutFilt = (KNum * OutFilt_1 + InFilt) / KDen = ((K-1) * OutFilt_1 + InFilt) / K
    // SampleT(sec) = (MAX_UPDATE_PERIOD / 8) / 1000
    // KNum = K - 1 = ThConstMotor(sec) / SampleT(sec)
    // KDen = K = 1 + ThConstMotor / SampleT
    sThModRun.sMotorI2T.flKNum = (FLOAT)((ULONG)sGlbMotorParameters.uwThermalConstant * 8000 / MAX_UPDATE_PERIOD) ;
    sThModRun.sMotorI2T.flKDenInv = 1.0 / (1.0 + sThModRun.sMotorI2T.flKNum) ; // Code Optimization
    sThModRun.sMotorI2T.ulMotorThermalConstant = 0L ;
#else
    // filter executed in background 
    sThModRun.sMotorI2T.flKNum = 0.0 ;
    sThModRun.sMotorI2T.flKDenInv = 1.0 ;
    sThModRun.sMotorI2T.ulMotorThermalConstant = (ULONG)sGlbMotorParameters.uwThermalConstant * 1000 ; // Code Optimization
#endif 
    // if user motor overtemperature threshold is higher than MotorMaxTemp (motor parameter), 
    // I2T protection is based on CurrentNominal AND NOT on CurrentNominal@ZeroSpeed
    if (sTm_ThModParam.swMotorOverTemp > (SWORD)(sGlbMotorParameters.flMaximumTemp * 10.0))
        flMotorINominal2Use = sGlbMotorParameters.flCurrentNominal ;
    else
        flMotorINominal2Use = sGlbMotorParameters.flCurrentNominalZeroSpeed ;

    sThModRun.sMotorI2T.flMotINom2 = flMotorINominal2Use * flMotorINominal2Use ; // A^2
    sThModRun.sMotorI2T.flMotINom2Inv = 100.0 / sThModRun.sMotorI2T.flMotINom2 ; // Code Optimization
    
    // filter status initialization
    sThModRun.sMotorI2T.flDeltaTemp_Iu = 0.0 ;
    sThModRun.sMotorI2T.flDeltaTemp_Iv = 0.0 ;    
    sThModRun.sMotorI2T.flDeltaTemp_Iw = 0.0 ;
}

/* ========================================================================= */
// Motor I2T protection - filter 
#ifdef _INFINEON_
static SWORD MotorI2T_Delta(SLONG *slMotorI, FLOAT huge *pDeltaT_Filtered)
#else
static SWORD MotorI2T_Delta(SLONG *slMotorI, FLOAT *pDeltaT_Filtered)
#endif // _infineon_
{
    FLOAT flDeltaT2Filter ;
    
    // DeltaT_In = 100.0 + ((I^2 - IMotNom^2) / IMotNom^2) * 100 (%)
    // DeltaT_Out = ((K-1)*DeltaT_Out_1 + DeltaT_In) / K (filter)
    // DeltaT_Out: 0.0%->100.0% (fault)
    flDeltaT2Filter = 100.0 + ((FLOAT)(*slMotorI) * (FLOAT)(*slMotorI) * MOTOR_I2T_IU2ARMS_SQUARED - sThModRun.sMotorI2T.flMotINom2) * sThModRun.sMotorI2T.flMotINom2Inv ;
    *pDeltaT_Filtered = (*pDeltaT_Filtered * sThModRun.sMotorI2T.flKNum + flDeltaT2Filter) * sThModRun.sMotorI2T.flKDenInv ;  
    
    return (SWORD)(*pDeltaT_Filtered * 10.0) ; // 1e-1 Celsius
}

/* ========================================================================= */
// Motor I2T protection - Runtime
static void MotorI2T_Runtime(void)
{
    SWORD swMotorDeltaT_Max, swMotorDeltaT_V, swMotorDeltaT_W ;
    SWORD swI2TMaxD = sTm_ThModParam.swI2TMaxDelta;
    UWORD uwTimer, uwDeltaTime ;

#ifndef __MOTOR_I2T_IN_SLOWTASK__
    // RUNTIME COMPUTATION OF FILTER GAINS (I'm in background)
    uwTimer = uwSysTimers1ms ; // freezing timer@1ms (I'm in background)
    uwDeltaTime = uwTimer - sThModRun.sMotorI2T.uwTimer_1 ; // ms 
    sThModRun.sMotorI2T.uwTimer_1 = uwTimer ; // regression     

    // OutFilt = (KNum * OutFilt_1 + InFilt) / KDen = ((K-1) * OutFilt_1 + InFilt) / K
    // SampleT(sec) = measured 
    // KNum = K - 1 = ThConstMotor(sec) / SampleT(sec)
    // KDen = K = 1 + ThConstMotor / SampleT    
    sThModRun.sMotorI2T.flKNum = (FLOAT)(sThModRun.sMotorI2T.ulMotorThermalConstant) / (FLOAT)uwDeltaTime ; //sec
    sThModRun.sMotorI2T.flKDenInv = 1.0 / (FLOAT)(1 + sThModRun.sMotorI2T.flKNum) ; // Code Optimization      
#endif
    
    // looking for MaxDeltaTemp of the 3 phases
#if CRS_DBGDSK
    swMotorDeltaT_Max = 0 ;
    swMotorDeltaT_V   = 0 ;
    swMotorDeltaT_W   = 0 ;
#else
    swMotorDeltaT_Max = MotorI2T_Delta(&(sTm_ThModIn.psMotorData->slIuFb), &sThModRun.sMotorI2T.flDeltaTemp_Iu) ;
    swMotorDeltaT_V   = MotorI2T_Delta(&(sTm_ThModIn.psMotorData->slIvFb), &sThModRun.sMotorI2T.flDeltaTemp_Iv) ;    
    swMotorDeltaT_W   = MotorI2T_Delta(&(sTm_ThModIn.psMotorData->slIwFb), &sThModRun.sMotorI2T.flDeltaTemp_Iw) ; 
#endif // _crs_dbg
    
    if (swMotorDeltaT_Max < swMotorDeltaT_V)
        swMotorDeltaT_Max = swMotorDeltaT_V ;
    
    if (swMotorDeltaT_Max < swMotorDeltaT_W)
        swMotorDeltaT_Max = swMotorDeltaT_W ;
    
    // if zero then I2T disabled    
    if ( swI2TMaxD )
    {
        if ((swMotorDeltaT_Max > swI2TMaxD) && !sThModRun.sErrorSent.b.bMotorI2T) 
        {   // DeltaTemp > 100.0% (= > 100.0 Celsius): fault
            sThModRun.sErrorSent.b.bMotorI2T = TRUE ;
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_TM_MOTOR_OVERTEMPERATURE, SYSTEMALARMS_SUBCODE_TM_MOTOR_I2T, bSysStatPowerEnabled) ;
            (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL) ;
        }

        if (swMotorDeltaT_Max > swI2TMaxD+ALARM_DELTAT)
            (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
    }

    sTm_ThModOut.swI2T_MotorDeltaT = swMotorDeltaT_Max ; // DiagnosticOutput
}

#ifdef _INFINEON_
/* ========================================================================= */
// Setup modulator for variable speed fan drive

static void digiffansetup(UBYTE ubMinSpeed)
{
        // enable module
    CCU63_KSCFG     = 0x0003;

        // wait startup
    _nop_();
    _nop_();
    _nop_();
    _nop_();

        // select compare mode for CC62
    CCU63_T12MSEL   = 0x0100;

        // period in order to have 2kHz
    CCU63_T12PR     = FANDRIVE_PERIOD;

        // T12 fclk
    CCU63_TCTR0     = 0x0000;

        // enable modulation output for COUT62
    CCU63_MODCTR    = 0x0020;

        // invert output state
    CCU63_PSLR      = 0x0020;

        // enable and set P9.6 for CCU63_COUT62
    P9_IOCR06       = 0x00A0;

        // zero out
    CCU63_CC62SR    = 0;

        // start timer and write shadow registers
    CCU63_TCTR4     = 0x0042;

        // calculate duty cycle equivalent to minimum speed
    sThModRun.uwFanMinSpeed=(UWORD)((ULONG)ubMinSpeed*FANDRIVE_PERIOD/100);
}

/* ========================================================================= */
// Get fan status

static BOOL getfanstatus(void)
{
    if(sThModRun.flags.b.bDIF)
        if(sThModRun.flags.b.bFanVSpeed)
            return CCU63_CC62SR>=sThModRun.uwFanMinSpeed;
        else
            return P9_OUT_P6;
    else
        return FPGA_GENERIC_IO_FAN_DRIVE;
}

/* ========================================================================= */
// Set fan status

static void setfanstatus(UWORD uwRatio)
{
    if(sThModRun.flags.b.bDIF)
        if(sThModRun.flags.b.bFanVSpeed)
        {
            SLONG slRatio=(SLONG)sThModRun.swCoolingVSpdRatio*uwRatio;

            if(slRatio>=FANDRIVE_PERIOD)
                CCU63_CC62SR = FANDRIVE_PERIOD;
            else if(slRatio<sThModRun.uwFanMinSpeed)
                CCU63_CC62SR = 0;
            else
                CCU63_CC62SR = (UWORD)slRatio;
                   
            CCU63_TCTR4  = 0x0042;
        }
        else
            P9_OUT_P6=(BOOL)uwRatio;
    else
        FPGA_GENERIC_IO_FAN_DRIVE=(BOOL)uwRatio;
}
#endif // infineon
/* ========================================================================= */
// Set PLC options  (valid only when system is booting)

BOOL Tm_SetPlcOption(UWORD uwOption, ULONG ulValue)
{
    if(ulValue!=TM_PLCOPTKEY || !bSysStatBooting)
        return TRUE;

    switch(uwOption)
    {
        case TM_PLCOPT_DISABLEBIQUAD:
            bDisableBiQuad=TRUE;
            return TRUE;

        case TM_PLCOPT_ENABLEBIQUAD:
            bDisableBiQuad=FALSE;
            return TRUE;

        case TM_PLCOPT_DISABLE_PKPK:
            bDisablePkPk = TRUE ;
            return TRUE ;

        case TM_PLCOPT_DISABLE_FW_MOTORTEMP:
            bDisableFwMotorTemp = TRUE ;
            return TRUE ;

        default:
            return FALSE;
    }
}

#ifndef _INFINEON_
static void DiagnosticTemperatures(void)
{
    FLOAT flCtrlBrd_RNtc, flTmpVal ;
    UWORD uwTmpVal ;

    sTm_ThModOut.sOutValue.flSOnchipTemp = sThModRun.sImageSlowTask.flSOnchipTemp ; // Celsius

    // ZynQ on chip overtemperature management
    if ((sTm_ThModOut.sOutValue.flSOnchipTemp >= sTm_ThModParam.flOnChipOverTemp) && (!sThModRun.sErrorSent.b.bOnChipOverTemp))
    {
        sThModRun.sErrorSent.b.bOnChipOverTemp = TRUE ;
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_TM_THERMALMODEL_FAIL, SYSTEMALARMS_SUBCODE_ONCHIP_OVERTEMP, bSysStatPowerEnabled) ;
        (*sTm_ThModIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
    }

#ifndef _HW_AXS

    // Controlboard onboard NTC management
    uwTmpVal = sThModRun.sImageSlowTask.uwSBoardTemp ; // A2D counts
    if (uwTmpVal > 0.0)
    {   // avoid division by zero
        // flCtrlBrd_RNtc = CTRLB_NTC_DIVIDER_RESISTOR * (FLOAT)(65536 - uwTmpVal) / (FLOAT)uwTmpVal ;
        flCtrlBrd_RNtc = CTRLB_NTC_DIVIDER_RESISTOR * (FLOAT)CTRLB_NTC_FULLSCALE/ (FLOAT)uwTmpVal - CTRLB_NTC_DIVIDER_RESISTOR - 20000;
        flTmpVal       = CTRLB_NTC_B + (log(flCtrlBrd_RNtc / CTRLB_NTC_25DEG_RESISTOR) * TWENTYFIVECELSIUSINKELVIN) ;
        sTm_ThModOut.sOutValue.flSBoardTemp = ((CTRLB_NTC_B * TWENTYFIVECELSIUSINKELVIN) / flTmpVal) - ZEROCELSIUSINKELVIN ;   // Celsius
    }
    else
        sTm_ThModOut.sOutValue.flSBoardTemp = -55.0 ; // Celsius

#else // !_hw_axs

#if defined(_HW_AXS_SAIETTA)

#elif defined(_HW_AXS_CABI35KW) || defined(_HW_AXS_DAYCO22KW)

    // Controlboard onboard NTC management
    UWORD uwSBoardTemp = sThModRun.sImageSlowTask.uwSBoardTemp; // A2D counts
    if (uwSBoardTemp > 0)
    {
    	FLOAT fRNtc = CTRLB_NTC_DIVIDER_RESISTOR * ((FLOAT)CTRLB_NTC_FULLSCALE / (FLOAT)uwSBoardTemp) - CTRLB_NTC_DIVIDER_RESISTOR - CTRLB_NTC_SERIES_RESISTOR;
        flTmpVal    = CTRLB_NTC_BETA + (log(fRNtc / CTRLB_NTC_25DEG_RESISTOR) * TWENTYFIVECELSIUSINKELVIN);
        sTm_ThModOut.sOutValue.flSBoardTemp = ((CTRLB_NTC_BETA * TWENTYFIVECELSIUSINKELVIN) / flTmpVal) - ZEROCELSIUSINKELVIN; // Celsius
    }
    else
        sTm_ThModOut.sOutValue.flSBoardTemp = -55.0; // Celsius

#endif

#endif // !_hw_axs

}
#endif // ! _infineon_
