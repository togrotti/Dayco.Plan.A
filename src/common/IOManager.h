/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : IOManager.h                                                */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/* Description : Generic analog and digital input/output manager            */
/*                                                                          */
/****************************************************************************/

#ifndef _IOMANAGER_H
#define _IOMANAGER_H

#include "common\DefineExternals.h"

//***************************************************************************
// Analog Measurements

typedef struct
{
#ifdef _INFINEON_
    UWORD uwSPowerBoardType;
    UWORD uwS15Vinp;
    UWORD uwS3V3out;
    UWORD uwS2V5out;
    UWORD uwS1V2out;
#else
    FLOAT fSVccPint;
    FLOAT fSVccPaux;
    FLOAT fSVccPdro;
    FLOAT fSOnchipTemp;
#endif // _infineon_
#ifndef _HW_DC
    UWORD uwSBoardTemp;
    UWORD uwSBridgeTemp;
    UWORD uwSMotorTemp;
    UWORD uwS24Vaux;
    UWORD uwSVENCout;
#else
    UWORD uwSBoardTemp;
    UWORD uwSBrgTemp[3];
    UWORD uwSMotorPTCTemp;
    UWORD uwSMotorKTYTemp;
    UWORD uwSVEMout;
    UWORD uwSVEAout;
#endif // _hw_dc
    SWORD swAllValid;
    SWORD swBridgeTempSensorCalibratonScale ;	// 8192 = 1.0 -> VrefIgbtNtc = 3.3V

} IOMGR_ANALOGMEASUREMENTS;

#ifdef _INFINEON_
EXTERN IOMGR_ANALOGMEASUREMENTS sdata sIOMgrAnalogMeasurements;
#else
EXTERN IOMGR_ANALOGMEASUREMENTS sIOMgrAnalogMeasurements;  
#endif // _infineon
//***************************************************************************
// Initialization entry point

BOOL IOMgr_Init(UWORD);

#endif
