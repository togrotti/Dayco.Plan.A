/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : HardwareParams2.h                                          */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Definition of hardware specific parameters, for            */
/*               all power boards combinations                              */
/*                                                                          */
/****************************************************************************/

#ifndef _HARDWAREPARAMS2_H
#define _HARDWAREPARAMS2_H

#include "common\CommonDefines.h"
#include "drive\HardwareParameters.h"

//***************************************************************************
// Output bridge layout

typedef struct
{
        // physical mapping of power output panel connectors (1=U, 2=V, 3=W)
        // e.g.: ubPhaseV=3 means power connector phase V is mapped on
        // fpga phase W
    UBYTE   ubPhaseU:2;      // default: U = 1
    UBYTE   ubPhaseV:2;      // default: V = 2
    UBYTE   ubPhaseW:2;      // default: W = 3
    UBYTE   ubReserved:2;
} HWPRM2_OUT_BRIDGE_LAYOUT;

//***************************************************************************
// Input AC layout

typedef struct
{
        // physical mapping of power output panel connectors (1=R, 2=S, 3=T)
        // e.g.: ubPhaseS=3 means power connector phase S is mapped on
        // fpga phase T
    UBYTE   ubPhaseR:2;      // default: R = 1
    UBYTE   ubPhaseS:2;      // default: S = 2
    UBYTE   ubPhaseT:2;      // default: T = 3
    UBYTE   ubReserved:2;
} HWPRM2_IN_AC_LAYOUT;

//***************************************************************************
// Assembly integrated brake resistor

typedef struct
{
        
    FLOAT   fValue;         // [Ohm] zero if not mounted
    FLOAT   fMaxPower;      // [W]
    FLOAT   fMaxEnergy;     // [J]
} HWPRM2_BRAKE_RESISTOR;

//***************************************************************************
// Assembly fan cooling equipment

#define HWPRM2_FANCOOL_DRV_ONOFF        0
#define HWPRM2_FANCOOL_DRV_PWM_2KHZ     1
#define HWPRM2_FANCOOL_DRV_PWM_4KHZ     2
#define HWPRM2_FANCOOL_DRV_PWM_8KHZ     3

#define HWPRM2_FANCOOL_FB_NONE          0
#define HWPRM2_FANCOOL_FB_FAULT         1
#define HWPRM2_FANCOOL_FB_TACHO         2

typedef struct
{
    struct
    {
        UBYTE   ubMounted:1;    // true if fan cooling is mounted
        UBYTE   ubDriveType:2;  // drive type (HWPRM2_FANCOOL_DRV_*)
        UBYTE   ubFeedback:2;   // feedback type (HWPRM2_FANCOOL_FB_*)
        UBYTE   ubFbPolarity:1; // feedback polarity: 1: fault high; 0: fault low
        UBYTE   ubReserved:2;
    } sOpt;
    
    UBYTE   ubMinSpeed;     // [%] when pwm drive, minimum speed allowed
} HWPRM2_COOLING_FEATURES;

//***************************************************************************
// Current sensing layout

typedef struct
{
        // physical mapping of current sensing  (1=U, 2=V, 3=W)
        // e.g.: ubCurrentX=2 means first current sensing (that normally is
        // Iu) lie on fpga phase V
    struct
    {
        UBYTE   ubCurrentU:2;   // default: U = 1
        UBYTE   ubCurrentV:2;   // default: V = 2
        UBYTE   ubCurrentW:2;   // default: W = 3
        UBYTE   ubCSFeedback:1; // CS feedback for propagation measurement
        UBYTE   ubReserved:1;
    } sOpt;
    UBYTE   ubReserved;

    UWORD   uwPDTime;       // [nsec] propagation delay
} HWPRM2_CURRENT_LAYOUT;

//***************************************************************************
// Hardware options supported features

typedef struct
{
    struct
    {
        UBYTE   ubSTO:1;        // Safety Torque Off supported
        UBYTE   ubCoolDrive:1;  // Cooling drive supported
        UBYTE   ubCoolPWM:1;    // PWM capability (0: only on/off supported; 1: pwm or on/off)
        UBYTE   ubCoolFltPol:1; // feedback polarity: 1: fault high (non-inverted); 0: fault low (inverted)
        UBYTE   ubCoolTacho:1;  // 1: Tacho or level feedback; 0: only level feedback
        UBYTE   ubReserved:3;
    } sOpt;
    UBYTE   ubReserved;
} HWPRM2_HW_FEATURES;

//***************************************************************************
// Currents range and calibration

typedef struct
{
    FLOAT               fRange;     // [Apk] full adc range = (-fRange;+fRange)
    HWPRMS_AD_SETTINGS  sCalPhaseU;
    HWPRMS_AD_SETTINGS  sCalPhaseV;
    HWPRMS_AD_SETTINGS  sCalPhaseW;
} HWPRM2_CURRENT_RANGE;

//***************************************************************************
// Voltage sensing layout

typedef struct
{
        // physical mapping of AC voltage sensing pairs to
        // ADC converters PSPL_U and PSPL_V
        // 1 = Vr-Vs, 2 = Vs-Vt, 3 = Vt-Vr
    struct
    {
        UBYTE   ubVac:1;        // Vac measurement supported
        UBYTE   ubVoltageU:2;   // default: RS = 1
        UBYTE   ubVoltageV:2;   // default: ST = 2
        UBYTE   ubCSFeedback:1; // CS feedback for propagation measurement
        UBYTE   ubReserved:2;
    } sOpt;
    UBYTE   ubReserved;

    UWORD   uwPDTime;       // [nsec] propagation delay
} HWPRM2_VOLTAGE_LAYOUT;

//***************************************************************************
// Voltage range and calibration

typedef struct
{
    FLOAT               fDCRange;   // [V] full adc range = [0;+fDCRange)
    FLOAT               fACRange;   // [V] full adc range = (-fACRange;+fACRange)
    HWPRMS_AD_SETTINGS  sCalVDC;
    HWPRMS_AD_SETTINGS  sCalVRS;
    HWPRMS_AD_SETTINGS  sCalVST;
} HWPRM2_VOLTAGE_RANGE;

//***************************************************************************
// Brake circuit features

typedef struct
{
    struct
    {
        UBYTE   ubBrakeDrive:1;     // brake support
        UBYTE   ubDrivePolarity:1;  // output polarity: 1: high activate brake; 0: low activate brake
        UBYTE   ubFaultPolarity:1;  // fault polarity: 1: fault active high; 0: fault active low
        UBYTE   ubReserved:5;
    } sOpt;
    UBYTE   ubReserved;
} HWPRM2_BRAKE_FEATURES;

//***************************************************************************
// DC charge circuit

typedef struct
{
    FLOAT   fCapacity;          // [uF] DC bus capacity
    FLOAT   fResistor;          // [Ohm] charging resistor (-1.0 if unsupported)
} HWPRM2_DC_CHARGE;

//***************************************************************************
// Bridge parameters

typedef struct
{
    struct
    {
        UBYTE   ubGateLoPolarity:1;     // 1: high turn on IGBT; 0: low turn on IGBT
        UBYTE   ubGateHiPolarity:1;     // 1: high turn on IGBT; 0: low turn on IGBT
        UBYTE   ubFaultPolarity:1;      // 1: fault high: 0: fault low
        UBYTE   ubResetFltPolarity:1;   // 1: fault reset low->high->low; 0: fault reset high->low->high
        UBYTE   ubReserved:4;
    } sOpt;
    UBYTE   ubReserved;

    UWORD   uwSwitchDeadTime;           // [1/100 usec] IGBT switch deadtime
    
    FLOAT   fModuleMaxTJunction;        // [Celsius] Junction Max Temperature       
    FLOAT   fNtc2JunctionThResist;      // [K/W] NTC/Junction Thermal Resistance    
    FLOAT   fBiQuadNtc2JunctThRes;      // [K/W] biquad filtering NTC/Junct ThRes
    FLOAT   fHsnk2NtcThResCoolingON;    // [K/W] Heatsink/NTC ThRes w/n cooling ON 
    FLOAT   fHsnk2NtcThResCoolingOFF;   // [K/W] Heatsink/NTC ThRes w/n cooling OFF
    
    FLOAT   fIgbtVceSat;                // [V] V across IGBT when switched on 
    
    FLOAT   fNtcVDividerResistor;       // [Ohm] Resistor of NTC Voltage Divider    
    FLOAT   fNtc25DegreeResistance;     // [Ohm] NTC Resistance value at 25 Celsius
    FLOAT   fNtcMaterialConstant;       // [K] NTC Material Specific Constant
    
    FLOAT   fIgbtThCapacitance;         // [J/K] IGBT Thermal Capacitance
    FLOAT   fHsnkThCapacitance;         // [J/K] Heatsink/NTC Thermal Capacitance
    
    FLOAT   fCommutationLosses;         // [J] Commutation losses (Eon+Eoff+Erec) 
    FLOAT   fIValueLosses;              // [Apk] I value when losses are computed

    FLOAT   fIgbtMaximumTemp;           // [Celsius] IGBT Max Temperature           
    FLOAT   fHsnkMaximumTemp;           // [Celsius] Heatsink/NTC Max Temperature

    FLOAT   fOverVoltage;               // [V] overvoltage fault limit
    FLOAT   fOverCurrent;               // [Arms] overcurrent fault limit
    FLOAT   fCurrentLimit;              // [Arms] current limit
} HWPRM2_BRIDGE;

//***************************************************************************
// Group data structures
//***************************************************************************

//***************************************************************************
// Assembly

typedef struct
{
    ASSEMBLY_INFO                   sInfo;
    HWPRM2_OUT_BRIDGE_LAYOUT        sOutBrdgCfg;
    HWPRM2_IN_AC_LAYOUT             sInBrdgCfg;
    HWPRM2_BRAKE_RESISTOR           sBrkRes;
    HWPRM2_COOLING_FEATURES         sCoolFeat;
    HWPRM2_DC_CHARGE                sDCChrg;
    HWPRM2_CURRENT_RANGE            sCurrRange;
}
#ifdef _INFINEON_
                        HWGRP_ASSEMBLY;
#else
__attribute__((packed)) HWGRP_ASSEMBLY;
#endif

//***************************************************************************
// Main power board for MPB systems

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_HW_FEATURES              sHwFeat;
    HWPRMS_AD_SETTINGS              sTempSens;
    HWPRM2_CURRENT_LAYOUT           sCurrCfg;
    HWPRM2_CURRENT_RANGE            sCurrRange;
    HWPRM2_VOLTAGE_LAYOUT           sVoltCfg;
    HWPRM2_VOLTAGE_RANGE            sVoltRange;
    HWPRM2_BRAKE_FEATURES           sBrkFeat;
    HWPRM2_DC_CHARGE                sDCChrg;
    HWPRM2_BRIDGE                   sBrdgFeat;
}
#ifdef _INFINEON_
                        HWGRP_MBP;
#else
__attribute__((packed)) HWGRP_MBP;
#endif

//***************************************************************************
// Main power board for MPB+PSI systems

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRMS_AD_SETTINGS              sTempSens;
    HWPRM2_CURRENT_LAYOUT           sCurrCfg;
    HWPRM2_CURRENT_RANGE            sCurrRange;
    HWPRM2_VOLTAGE_LAYOUT           sVoltCfg;
    HWPRM2_VOLTAGE_RANGE            sVoltRange;
    HWPRM2_BRAKE_FEATURES           sBrkFeat;
    HWPRM2_DC_CHARGE                sDCChrg;
    HWPRM2_BRIDGE                   sBrdgFeat;
}
#ifdef _INFINEON_
                        HWGRP_DS_MBP;
#else
__attribute__((packed)) HWGRP_DS_MBP;
#endif

//***************************************************************************
// Power supply board for MPB+PSI systems

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_HW_FEATURES              sHwFeat;
    HWPRMS_AD_SETTINGS              sTempSens;
    HWPRM2_CURRENT_LAYOUT           sCurrCfg;
    HWPRM2_VOLTAGE_LAYOUT           sVoltCfg;
}
#ifdef _INFINEON_
                        HWGRP_DS_PSI;
#else
__attribute__((packed)) HWGRP_DS_PSI;
#endif

//***************************************************************************
// Power supply board for PSI+FTB+GDB systems

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_HW_FEATURES              sHwFeat;
    HWPRMS_AD_SETTINGS              sTempSens;
    HWPRM2_CURRENT_LAYOUT           sCurrCfg;
    HWPRM2_CURRENT_RANGE            sCurrRange;
    HWPRM2_VOLTAGE_LAYOUT           sVoltCfg;
}
#ifdef _INFINEON_
                        HWGRP_TS_PSI;
#else
__attribute__((packed)) HWGRP_TS_PSI;
#endif

//***************************************************************************
// Filter board for PSI+FTB+GDB systems

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_VOLTAGE_LAYOUT           sVoltCfg;
    HWPRM2_VOLTAGE_RANGE            sVoltRange;
    HWPRM2_BRAKE_FEATURES           sBrkFeat;
    HWPRM2_DC_CHARGE                sDCChrg;
}
#ifdef _INFINEON_
                        HWGRP_TS_FTB;
#else
__attribute__((packed)) HWGRP_TS_FTB;
#endif

//***************************************************************************
// Gate driver board for PSI+FTB+GDB systems

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_BRIDGE                   sBrdgFeat;
}
#ifdef _INFINEON_
                        HWGRP_TS_GDB;
#else
__attribute__((packed)) HWGRP_TS_GDB;
#endif

//***************************************************************************
// Power supply board for multiple bridge systems

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_HW_FEATURES              sHwFeat;
    HWPRM2_CURRENT_LAYOUT           sCurrCfg;
    HWPRM2_VOLTAGE_LAYOUT           sVoltCfg;
    HWPRM2_VOLTAGE_RANGE            sVoltRange;
    HWPRM2_BRAKE_FEATURES           sBrkFeat;
    HWPRM2_DC_CHARGE                sDCChrg;
}
#ifdef _INFINEON_
                        HWGRP_XS_PSI;
#else
__attribute__((packed)) HWGRP_XS_PSI;
#endif

//***************************************************************************
// Bridge definition for multiple bridge systems

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_HW_UNITS                 sUnit;
    HWPRMS_AD_SETTINGS              sTempSens;
    HWPRM2_CURRENT_RANGE            sCurrRange;
}
#ifdef _INFINEON_
                        HWGRP_XS_BRI;
#else
__attribute__((packed)) HWGRP_XS_BRI;
#endif

//***************************************************************************
// PSU Power Supply Board

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_HW_FEATURES              sHwFeat;
}
#ifdef _INFINEON_
                        HWGRP_PSU_PSB;
#else
__attribute__((packed)) HWGRP_PSU_PSB;
#endif

//***************************************************************************
// PSU Brake Driver Board

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRMS_AD_SETTINGS              sTempSens;
    HWPRM2_BRAKE_FEATURES           sBrkFeat;
}
#ifdef _INFINEON_
                        HWGRP_PSU_BRK;
#else
__attribute__((packed)) HWGRP_PSU_BRK;
#endif

//***************************************************************************
// PSU SCR Driver Board

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRMS_AD_SETTINGS              sTempSens;
    HWPRM2_VOLTAGE_LAYOUT           sVoltCfg;
    HWPRM2_VOLTAGE_RANGE            sVoltRange;
}
#ifdef _INFINEON_
                        HWGRP_PSU_SCR;
#else
__attribute__((packed)) HWGRP_PSU_SCR;
#endif

//***************************************************************************
// Resulting power board parameters

typedef struct
{
    HWPRMS_PRODUCT_INFO             sProductInfo;
    HWPRM2_HW_FEATURES              sHwFeat;
    HWPRM2_COOLING_FEATURES         sCoolFeat;
#ifndef _HW_DC
    HWPRMS_AD_SETTINGS              sTempSens;
#else
    HWPRMS_AD_SETTINGS              sTempSens[3];
#endif
    HWPRM2_OUT_BRIDGE_LAYOUT        sOutBrdgCfg;
    HWPRM2_CURRENT_LAYOUT           sCurrCfg;
#ifndef _HW_DC
    HWPRM2_CURRENT_RANGE            sCurrRange;
#else
    FLOAT                           fRange;     // [Apk] full adc range = (-fRange;+fRange)
    HWPRM2_CURRENT_RANGE            sBrCurrRange[3];
#endif
    HWPRM2_IN_AC_LAYOUT             sInBrdgCfg;
    HWPRM2_VOLTAGE_LAYOUT           sVoltCfg;
    HWPRM2_VOLTAGE_RANGE            sVoltRange;
    HWPRM2_BRAKE_FEATURES           sBrkFeat;
    HWPRM2_DC_CHARGE                sDCChrg;
    HWPRM2_BRAKE_RESISTOR           sBrkRes;
    HWPRM2_BRIDGE                   sBrdgFeat;
#ifdef _HW_DC
    HWPRM2_HW_UNITS                 sUnit;
#endif
} HWGRP_POWER_BOARD_UNIVERSAL;

//***************************************************************************
// All power boards

typedef union
{
    // Always first entry in the structure
    HWPRMS_PRODUCT_INFO             sProductInfo;
    HWPRM_POWER_BOARD_AXM_AXP_AC    sAxmAxpAC;
    HWPRM_POWER_BOARD_AXM_AXP_AC_V2 sAxmAxpACV2;
    HWPRM_POWER_BOARD_DIF           sDif;
    HWGRP_POWER_BOARD_UNIVERSAL     sUniv;
} HWPRM_POWER_BOARD; 

#endif
