/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : MotorParameters.h                                          */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Definition of motor specific parameters, that could be     */
/*               written by the user and/or taken P'n'P from the motor      */
/*                                                                          */
/****************************************************************************/

#ifndef _MOTORPARAMETERS_H
#define _MOTORPARAMETERS_H                                                                     

#include "common\CommonDefines.h"

//***************************************************************************
// Motor parameters

typedef struct
{
        // motor s/n
    ULONG   ulSerialNumber;
        // production date (YYYYMMDD)
    ULONG   ulProductionDate;
        // motor model
    ULONG   ulModel;

        // [Ohm]
    FLOAT   flResistance;
        // [H]
    FLOAT   flInductance;
        // [Nm/Arms]
    FLOAT   flKT;

        // nominal at zero speed [Arms]
    FLOAT   flCurrentNominalZeroSpeed;
        // nominal at nominal speed [Arms]
    FLOAT   flCurrentNominal;
        // peak current [Arms]
    FLOAT   flCurrentPeak;
        // nominal speed [rad/sec]
    FLOAT   flSpeedNominal;
        // thermal constant [sec]
    UWORD   uwThermalConstant; UWORD uwDummy1;
        // Motor inertia [kg * m^2]
    FLOAT   flMotorInertia;

        // electrical phase offset [0:65535=0:359 electrical deg]
    UWORD   uwPhaseOffset; UWORD uwDummy2;

        // motor type
    ULONG   ulType;
        // poles number
    UWORD   uwPoleNumbers; UWORD uwDummy3;

        // start cooling at (if available) [C]
    FLOAT   flCoolingTempOn;
        // stop cooling at (if available) [C]
    FLOAT   flCoolingTempOff;
        // maximum coil temperature [C]
    FLOAT   flMaximumTemp;

        // synchronous or direct inductance [H]
    FLOAT   flDirectInductance;
} MOTPRM_PARAMETERS;

//***************************************************************************
// Old data structure

typedef struct
{
    ULONG   ulSerialNumber;
    ULONG   ulProductionDate;
    ULONG   ulModel;
    FLOAT   flResistance;
    FLOAT   flInductance;
    FLOAT   flKT;
    FLOAT   flCurrentNominalZeroSpeed;
    FLOAT   flCurrentNominal;
    FLOAT   flCurrentPeak;
    FLOAT   flSpeedNominal;
    UWORD   uwThermalConstant;
    FLOAT   flMotorInertia;
    UWORD   uwPhaseOffset;
    ULONG   ulType;
    UWORD   uwPoleNumbers;
    FLOAT   flCoolingTempOn;
    FLOAT   flCoolingTempOff;
    FLOAT   flMaximumTemp;
} MOTPRM_V14_PARAMETERS;

//***************************************************************************
// Globals

extern MOTPRM_PARAMETERS sGlbMotorParameters;
#ifdef _INFINEON_
extern const MOTPRM_PARAMETERS huge sGlbMotorDefParams;
#else
extern const MOTPRM_PARAMETERS sGlbMotorDefParams;
#endif

//***************************************************************************
// Entry points

BOOL MotPar_Init(void);
BOOL MotPar_Set(MOTPRM_PARAMETERS *);

#endif
