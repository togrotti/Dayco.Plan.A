/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : MotorParameters.c                                          */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Definition of motor specific parameters, that could be     */
/*               written by the user and/or taken P'n'P from the motor      */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "common\TaskScheduler.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "MotorParameters.h"

//***************************************************************************
// Defines

#define R_MAX_VALUE   32.766  /* Ohm */
#define R_MIN_VALUE   0.001   /* Ohm */

#define L_MAX_VALUE   0.32766 /* H */
#define L_MIN_VALUE   0.00001 /* H */

//***************************************************************************
// Globals (default referred to UL503.50.3)

MOTPRM_PARAMETERS sGlbMotorParameters;

const MOTPRM_PARAMETERS sGlbMotorDefParams=
{
    0ul,
    0ul,
    0ul,

    3.1,
    0.01433,
    1.01,

    3.82,
    2.2,
    7.0,
    523.0,
    2000, 0,
    0.0001,

    0, 0,

    503503,
    8, 0,
	100.0,
	90.0,
	140.0,
	0.0,
};

//***************************************************************************
// Local functions

static BOOL checkvalidity(void);

//***************************************************************************
// Initialization entry point

BOOL MotPar_Init(void)
{
        // add task
   if(!TaskSched_AddBackgroundTask((void (*)(void))&checkvalidity))
       return FALSE;

        // then force immediate parameters check
    return checkvalidity();
}

//***************************************************************************
// Setup new parameters

BOOL MotPar_Set(MOTPRM_PARAMETERS * psNewParams)
{
        // copy to global structure
    sGlbMotorParameters=*psNewParams;

        // and force immediate parameters check
    return checkvalidity();
}

//***************************************************************************
// slow task

static BOOL checkvalidity(void)
{
    BOOL bValid=TRUE;
    
    if(sGlbMotorParameters.flResistance < R_MIN_VALUE || sGlbMotorParameters.flResistance > R_MAX_VALUE)
    {
        ParChk_SignalValueError(PARCC_MOTOR_RESISTANCE);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_MOTOR_RESISTANCE);
    
    if(sGlbMotorParameters.flInductance < L_MIN_VALUE || sGlbMotorParameters.flInductance > L_MAX_VALUE)
    {
        ParChk_SignalValueError(PARCC_MOTOR_INDUCTANCE);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_MOTOR_INDUCTANCE);
    
    if(sGlbMotorParameters.flKT <= 0.0 )
    {
        ParChk_SignalValueError(PARCC_MOTOR_KT);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_MOTOR_KT);
    
    if(sGlbMotorParameters.uwPoleNumbers <= 0 || (sGlbMotorParameters.uwPoleNumbers%2))
    {
        ParChk_SignalValueError(PARCC_MOTOR_POLENUMBERS);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_MOTOR_POLENUMBERS);

    if(sGlbMotorParameters.flCurrentPeak < 0.0 || sGlbMotorParameters.flCurrentPeak >= 214748.3647)
    {
        ParChk_SignalValueError(PARCC_MOTOR_CURRENTPEAK);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_MOTOR_CURRENTPEAK);
    
    if(sGlbMotorParameters.flDirectInductance != 0.0 && (sGlbMotorParameters.flDirectInductance < L_MIN_VALUE || sGlbMotorParameters.flDirectInductance > L_MAX_VALUE))
    {
        ParChk_SignalValueError(PARCC_MOTOR_DIRECTINDUCTANCE);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_MOTOR_DIRECTINDUCTANCE);
    
    return bValid;
}
