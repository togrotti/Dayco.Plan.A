/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : HardwareParameters.h                                       */
/* Author      : Fabio Terrile                                              */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description : Definition of hardware specific parameters, for drive and  */
/*               all power boards combinations                              */
/*                                                                          */
/****************************************************************************/

#ifndef _HARDWAREPARAMETERS_H
#define _HARDWAREPARAMETERS_H

#include "common\CommonDefines.h"

//***************************************************************************
// Product info, placed on top of all kind of structures

typedef struct
{
        // product code, zero is reserved for future expansions
    UWORD   uwProductCode;
        // product revision, zero is reserved for future expansions
    UWORD   uwProductRev;
} HWPRMS_PRODUCT_INFO;

//***************************************************************************
// Product codes

#define HWPRM_PCODE_BOARD_UNKNOWN                   0   // Unknown board

    // Control boards
#define HWPRM_PCODE_CONTROL_BOARD_AXM_II_XC164      1   // Revisions : 1.00, 1.01, 1.02, 1.03, 1.04
#define HWPRM_PCODE_CONTROL_BOARD_AXM_II_XE167      2   // Revisions : 1.00, 1.01, 1.02, 2.00, 3.xx, 4.xx
#define HWPRM_PCODE_CONTROL_BOARD_AXM_II_XC2788X    3   // Revisions : 5.xx
#define HWPRM_PCODE_CONTROL_BOARD_AXM_II_ZYNQ       4   // Zynq

    // Power boards
#define HWPRM_PCODE_POWER_BOARD_OLD_AXM_AC          1   // Standard AxM AC Power Board
#define HWPRM_PCODE_POWER_BOARD_OLD_AXW_AC          2   // Standard AxW AC Power Board
#define HWPRM_PCODE_POWER_BOARD_OLD_AXW_DC          3   // Standard AxW DC Power Board
#define HWPRM_PCODE_POWER_BOARD_AXP_AC              4   // AxP AC Power Board
#define HWPRM_PCODE_POWER_BOARD_AXM_AC              5   // AxM AC Power Board
#define HWPRM_PCODE_POWER_BOARD_AXW_AC              6   // AxW AC Power Board
#define HWPRM_PCODE_POWER_BOARD_AXW_DC              7   // AxW DC Power Board
// 8: do not use (old ZF_MARINE)
#define HWPRM_PCODE_POWER_BOARD_AXP_DS_AC           9   // AxP AC Power Board with IGBT desat input
#define HWPRM_PCODE_POWER_BOARD_AXM_DS_AC           10  // AxM AC Power Board with IGBT desat input
#define HWPRM_PCODE_POWER_BOARD_DIF_CI              16  // Digital interface - controlboard currents
#define HWPRM_PCODE_POWER_BOARD_DIF_PI              17  // Digital interface - powerboard currents
// 18: do not use (old DIF_DBLBRIDGE)
#define HWPRM_PCODE_POWER_BOARD_UNIVERSAL           19  // Universal Drive Power Board
#define HWPRM_PCODE_CONTROL_BOARD_AXM_II_UNIV       20  // Universal Drive Control Board
#define HWPRM_PCODE_POWER_BOARD_AXM_AC_ALMAPS       21  // AxM AC Power Board configured as Power Supply for ALMA
#define HWPRM_PCODE_CONTROL_BOARD_AXM_II_CTUNV      22  // Universal Drive Control Board with Cyclone 10
#define HWPRM_PCODE_POWER_BOARD_DIF_BC              23  // Digital interface - brake current
#define HWPRM_PCODE_POWER_BOARD_DIF_BL              24  // Digital interface for axbl

    // IO expansion boards
#define HWPRM_PCODE_IO_EXP_BOARD                    1   // Standard opto I/O exp board

//***************************************************************************
// Control board components mounting/version
//
// FPGA type (same as HDL):
// Bit 0-3	: Speed grade FPGA, 6/7/8
//     4-11	: FPGA size, 8/10/16/25
//     12-13: FPGA type: 0: Cyclone III, 1: Cyclone IV, 2,3: reserved

#define HWPRM_CTRLBRD_FPGA_UNKNOWN                  0

#define HWPRM_CTRLBRD_FPGA_EP3C10F256C8             0x00A8
#define HWPRM_CTRLBRD_FPGA_EP3C16F256C8             0x0108
#define HWPRM_CTRLBRD_FPGA_EP3C25F256C7             0x0197
#define HWPRM_CTRLBRD_FPGA_EP3C25F256C8             0x0198
#define HPPRM_CTRLBRD_FPGA_EP4CE6E22I7              0x1067
#define HPPRM_CTRLBRD_FPGA_EP4CE22F17C8             0x1168
#define HPPRM_CTRLBRD_FPGA_EP4CE10F17C8             0x10A8

#define HWPRM_CTRLBRD_HWCONFIG_UNKNOWN              0

#define HWPRM_CTRLBRD_HWCONFIG_FULL                 1
#define HWPRM_CTRLBRD_HWCONFIG_NOETHNOEXP           2
#define HWPRM_CTRLBRD_HWCONFIG_OPTION_ZF            3

//***************************************************************************
// ADC calibration parameters

typedef struct
{
    UWORD   uwOffst;  
    SWORD   swScale;  
} HWPRMS_AD_SETTINGS;

//***************************************************************************
// DAC calibration parameters

typedef struct
{
    UWORD   uwOffst;  
    SWORD   swScale;  
} HWPRMS_DA_SETTINGS;

//***************************************************************************
// Hardware unit codes, shared between controlboards and powerboards
// zero meant default, e.g. not subject to be optional

typedef struct
{
    ULONG   ulIDMask;
} HWPRM2_HW_UNITS;

#ifdef _HW_CT
//***************************************************************************
// On Board Thermal Model common parameters

typedef struct
{
    UWORD uwNtcVDividerResistor;      // Resistor of NTC Voltage Divider    (ohm) 
    UWORD uwNtc25DegreeResistance;    // NTC Resistance value at 25 Celsius (ohm) 
    UWORD uwNtcMaterialConstant;      // NTC Material Specific Constant       (K)   
} HWPRMS_BOARD_NTC;
#endif

//***************************************************************************
// Control board parameters

typedef struct
{
    HWPRMS_PRODUCT_INFO  sProductInfo;

    HWPRMS_AD_SETTINGS   sSupply24V;
    HWPRMS_AD_SETTINGS   sSupplyVenc;

    HWPRMS_AD_SETTINGS   sBoardTemp;
    HWPRMS_AD_SETTINGS   sBridgeTemp;
                      
    HWPRMS_AD_SETTINGS   sUserRefI0;
    HWPRMS_AD_SETTINGS   sUserRefI1;
                      
    HWPRMS_DA_SETTINGS   sUserAnaO0;
    HWPRMS_DA_SETTINGS   sUserAnaO1;
                      
    HWPRMS_AD_SETTINGS   sIncEncSin;
    HWPRMS_AD_SETTINGS   sIncEncCos;

        // LO gain is G1, HI gain is G2                      
    HWPRMS_AD_SETTINGS   sAbsEncG1Sin;
    HWPRMS_AD_SETTINGS   sAbsEncG1Cos;
    HWPRMS_AD_SETTINGS   sAbsEncG2Sin;
    HWPRMS_AD_SETTINGS   sAbsEncG2Cos;
                      
    HWPRMS_AD_SETTINGS   sCurrentPhaseU;
    HWPRMS_AD_SETTINGS   sCurrentPhaseV;
    HWPRMS_AD_SETTINGS   sVoltageDcLink;
    
    HWPRMS_AD_SETTINGS   sMotorTemp;

        // Resistor of Motor NTC Voltage Divider (ohm)
    UWORD                uwMotorNtcVDividerResistor;

    UWORD                uwHwConfig;        // HWPRM_CTRLBRD_HWCONFIG_*
    UWORD                uwFpgaType;        // HWPRM_CTRLBRD_FPGA_*

    HWPRMS_AD_SETTINGS   sUserRefI2;
    HWPRMS_AD_SETTINGS   sUserRefI3;
#ifdef _HW_CT    
    HWPRMS_BOARD_NTC     sBoardNtc;
#endif
} HWPRM_CNTRL_BOARD;

//***************************************************************************
// Bridge and Current Sensing Layout

typedef struct
{
        // physical mapping of power output panel connectors (1=U, 2=V, 3=W)
        // e.g.: ubPhaseV=3 means power connector phase V is mapped on
        // fpga phase W
    struct
    {
        UBYTE   ubPhaseU:2; /* default: U = 1 */
        UBYTE   ubPhaseV:2; /* default: V = 2 */
        UBYTE   ubPhaseW:2; /* default: W = 3 */
        UBYTE   ubResFltPolarity:1; /* default: 0 = fault reset when high */
        UBYTE   ubReserved:1;   // keep zero
    } sBridge;

        // physical mapping of current sensing  (1=U, 2=V, 3=W)
        // e.g.: ubCurrentX=2 means first current sensing (that normally is
        // Iu) lie on fpga phase V
    struct
    {
        UBYTE   ubCurrentU:2; /* default: U = 1 */
        UBYTE   ubCurrentV:2; /* default: V = 2 */
        UBYTE   ubCurrentW:2; /* default: W = 0 */
        UBYTE   ubReserved:2;   // keep zero
    } sCurrent;
} HWPRMS_BRIDGE_AND_CURRENT_LAYOUT;

//***************************************************************************
// Bridge Operative Limits

typedef struct
{
    SLONG   slOverCurrent;  /* 1e-4A rms */
    SLONG   slCurrentLimit; /* 1e-4A rms */
    SWORD   swOverVoltage;  /* 1e-1V istantaneo */
#ifdef _INFINEON_
} HWPRMS_BRIDGE_OPERATIVE_LIMITS;
#else
} __attribute__((packed)) HWPRMS_BRIDGE_OPERATIVE_LIMITS;
#endif

//***************************************************************************
// Bridge Thermal Model common parameters

typedef struct
{
    UWORD uwModuleMaxTJunction;       // Junction Max Temperature       (Celsius)
    UWORD uwNtc2JunctionThResist;     // NTC/Junction Thermal Resistance    (K/W)
    UWORD uwHsnk2NtcThResCoolingON;   // Heatsink/NTC Thermal Resistance when cooling is ON  (K/W) 
    UWORD uwHsnk2NtcThResCoolingOFF;  // Heatsink/NTC Thermal Resistance when cooling is OFF (K/W) 
    
    UWORD uwIgbtVceSat;               // V across IGBT when it is switched on (V) 
    
    UWORD uwNtcVDividerResistor;      // Resistor of NTC Voltage Divider    (ohm) 
    UWORD uwNtc25DegreeResistance;    // NTC Resistance value at 25 Celsius (ohm) 
    UWORD uwNtcMaterialConstant;      // NTC Material Specific Constant       (K)   
    
    UWORD uwIgbtThCapacitance;        // IGBT Thermal Capacitance           (J/K) 
    UWORD uwHsnkThCapacitance;        // Heatsink/NTC Thermal Capacitance   (J/K) 
    
    UWORD uwCommutationLosses;        // Commutation losses (Eon+Eoff+Erec)   (J) 
    UWORD uwIValueLosses;             // I value when losses are computed     (A) 

    UWORD uwIgbtMaximumTemp;          // IGBT Max Temperature           (Celsius) 
    UWORD uwHsnkMaximumTemp;          // Heatsink/NTC Max Temperature   (Celsius) 
} HWPRMS_BRIDGE_THERMAL_MODEL;

//***************************************************************************
// Power Board options

#define HWPRM_POWER_BOARD_TEMPSENSOR_NTC                1

#define HWPRM_POWER_BOARD_COOLING_WATER                 1
#define HWPRM_POWER_BOARD_COOLING_FAN_3WIRE_TACHO       3
#define HWPRM_POWER_BOARD_COOLING_FAN_2WIRE             4
#define HWPRM_POWER_BOARD_COOLING_FAN_3WIRE_HIGOOD      5
#define HWPRM_POWER_BOARD_COOLING_FAN_3WIRE_VSPEED      6

//***************************************************************************
// Bridge Parameters

typedef struct
{
    HWPRMS_BRIDGE_AND_CURRENT_LAYOUT   sBridgeCurrentLayout;

        // IGBT deadtime [1/100 usec]
    UWORD uwSwitchDeadTime;

    HWPRMS_BRIDGE_OPERATIVE_LIMITS     sOperativeLimits;
    HWPRMS_BRIDGE_THERMAL_MODEL        sThermalModel;
    
        // HWPRM_POWER_BOARD_TEMPSENSOR_*
    UWORD                              uwTempSensorType;
        // A/D calibration
    HWPRMS_AD_SETTINGS                 sTempSensorCalibr;
#ifdef _INFINEON_
} HWPRMS_BRIDGE_PARAMETERS;
#else
} __attribute__((packed)) HWPRMS_BRIDGE_PARAMETERS;
#endif

//***************************************************************************
// Brake Circuit

typedef struct
{
    struct
    {
            // 0 means brake active low, 1 brake active high
        UBYTE   ubDrivePolarity:1;
        UBYTE   ubReserved:7;          // keep zero
        UBYTE   ubDummy;
    } sOptions;

        // brake resistor, zero if not mounted [1e-1 Ohm]
    UWORD               uwResistorValue;
        // brake resistor power [1e-1 W]
    UWORD               uwResistorPower;
#ifdef _INFINEON_
} HWPRMS_BRAKE_CIRCUIT;
#else
} __attribute__((packed)) HWPRMS_BRAKE_CIRCUIT;
#endif

//***************************************************************************
// Brake Circuit (version 2)

typedef struct
{
    struct
    {
            // 0 means brake active low, 1 brake active high
        UBYTE   ubDrivePolarity:1;
        UBYTE   ubReserved:7;          // keep zero
        UBYTE   ubDummy;
    } sOptions;

        // brake resistor, zero if not mounted [1e-1 Ohm]
    UWORD               uwResistorValue;
        // brake resistor power [1e-1 W]
    UWORD               uwResistorPower;
        // brake resistor max energy [J]
    ULONG               ulRBrakeMaxEnergy;
#ifdef _INFINEON_
} HWPRMS_BRAKE_CIRCUIT_V2;
#else
} __attribute__((packed)) HWPRMS_BRAKE_CIRCUIT_V2;
#endif

//***************************************************************************
// Power Board Parameters : AxM / AxP 

typedef struct
{
        // Always first entry in the structure
    HWPRMS_PRODUCT_INFO          sProductInfo;

    HWPRMS_BRIDGE_PARAMETERS     sBridgeParams;

    HWPRMS_AD_SETTINGS           sVoltageDcLink;
    HWPRMS_AD_SETTINGS           sCurrentPhaseU;
    HWPRMS_AD_SETTINGS           sCurrentPhaseV;

    HWPRMS_BRAKE_CIRCUIT         sBrakeCircuit;

        // HWPRM_POWER_BOARD_COOLING_*
    UWORD                        uwCoolingType;

        // unit scaling from physical to eng
        // e.g.: [1/10V] = flDCVoltageScale * [AD sample]
    FLOAT                        flCurrentScale;
    FLOAT                        flDCVoltageScale;

#ifdef _INFINEON_
} HWPRM_POWER_BOARD_AXM_AXP_AC;
#else
} __attribute__((packed)) HWPRM_POWER_BOARD_AXM_AXP_AC;
#endif

//***************************************************************************
// Power Board Parameters : AxM / AxP (version 2)

typedef struct
{
        // Always first entry in the structure
    HWPRMS_PRODUCT_INFO          sProductInfo;

    HWPRMS_BRIDGE_PARAMETERS     sBridgeParams;

    HWPRMS_AD_SETTINGS           sVoltageDcLink;
    HWPRMS_AD_SETTINGS           sCurrentPhaseU;
    HWPRMS_AD_SETTINGS           sCurrentPhaseV;

    HWPRMS_BRAKE_CIRCUIT_V2      sBrakeCircuit;

        // HWPRM_POWER_BOARD_COOLING_*
    UWORD                        uwCoolingType;

        // unit scaling from physical to eng
        // e.g.: [1/10V] = flDCVoltageScale * [AD sample]
    FLOAT                        flCurrentScale;
    FLOAT                        flDCVoltageScale;

#ifdef _INFINEON_
} HWPRM_POWER_BOARD_AXM_AXP_AC_V2;
#else
} __attribute__((packed)) HWPRM_POWER_BOARD_AXM_AXP_AC_V2;
#endif

//***************************************************************************
// Power Board Parameters : DIF

typedef struct
{
        // Always first entry in the structure
    HWPRMS_PRODUCT_INFO                 sProductInfo;

    HWPRMS_BRIDGE_PARAMETERS            sBridgeParams;
    HWPRMS_BRIDGE_AND_CURRENT_LAYOUT    sBridge1CurrentLayout;

    HWPRMS_AD_SETTINGS           sVoltageDcLink;
    HWPRMS_AD_SETTINGS           sVoltageRS;
    HWPRMS_AD_SETTINGS           sVoltageST;
    HWPRMS_AD_SETTINGS           sCurrentPhaseU;
    HWPRMS_AD_SETTINGS           sCurrentPhaseV;
    HWPRMS_AD_SETTINGS           sCurrentPhaseW;
    HWPRMS_AD_SETTINGS           sCurrentB1PhaseU;
    HWPRMS_AD_SETTINGS           sCurrentB1PhaseV;
    HWPRMS_AD_SETTINGS           sCurrentB1PhaseW;

        // external current ADC propagation delay time [nsec]
    UWORD                        uwCurrentAdcPDTime;

    HWPRMS_BRAKE_CIRCUIT_V2      sBrakeCircuit;

        // HWPRM_POWER_BOARD_COOLING_*
    UWORD                        uwCoolingType;

        // unit scaling from physical to eng
        // e.g.: [1/10V] = flDCVoltageScale * [AD sample]
    FLOAT                        flCurrentScale;
    FLOAT                        flDCVoltageScale;
    FLOAT                        flACVoltageScale;
#ifdef _INFINEON_
} HWPRM_POWER_BOARD_DIF;
#else
} __attribute__((packed)) HWPRM_POWER_BOARD_DIF;
#endif

//***************************************************************************
// IO expansion board parameters

typedef struct
{
    HWPRMS_PRODUCT_INFO  sProductInfo;

    UWORD                uwHwConfig;        // for future expansion, keep zero
    UWORD                uwFpgaType;        // HWPRM_CTRLBRD_FPGA_*

    HWPRMS_AD_SETTINGS   sUserRefI0;
    HWPRMS_AD_SETTINGS   sUserRefI1;
    HWPRMS_AD_SETTINGS   sUserRefI2;
                      
    HWPRMS_DA_SETTINGS   sUserAnaO0;
    HWPRMS_DA_SETTINGS   sUserAnaO1;
} HWPRM_IO_EXP_BOARD;

#endif
