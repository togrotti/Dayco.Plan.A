/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : DriveTaskController.h                                      */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Task configuration for drive controller                    */
/*                                                                          */
/****************************************************************************/

#ifndef _DRIVETASKCONTROLLER_H
#define _DRIVETASKCONTROLLER_H

#include "common\CommonDefines.h"

//***************************************************************************
// Defines

#define DRVTSKCTRL_MO_MOTORCONTROL          0x2D
//#define DRVTSKCTRL_MO_PARALLELBRIDGE        0x72  // do not use
//#define DRVTSKCTRL_MO_ACTIVEFRONTEND        0xA4  // do not use
#define DRVTSKCTRL_MO_PSU                   0x50 // power supply

#define DRVTSKCTRL_TO_AUTO                  0x00
#define DRVTSKCTRL_TO_OFF                   0x01
#define DRVTSKCTRL_TO_PLC                   0x02

#define DRVTSKCTRL_OP_TORQUE                0x01
#define DRVTSKCTRL_OP_POSITIONER            0x03
#define DRVTSKCTRL_OP_EFS_IDR_AND_PLOOP     0x05
#define DRVTSKCTRL_OP_IRTQ                  0x07
#define DRVTSKCTRL_OP_ACTIVEFRONTEND        0x10
#define DRVTSKCTRL_OP_PSU                   0x20
//***************************************************************************
// Data types

typedef struct
{
    UBYTE ubModActiveFrontEnd;        // AFE
    UBYTE ubModPiDcBusCntrlLp32bit;   // AFE
    UBYTE ubPositioner;               // MOT
    UBYTE ubSpaceSpeedCntrLp;         // MOT
    UBYTE ubThermalModel;             // AFE/MOT
    UBYTE ubMotorHandler;             // AFE/MOT
    UBYTE ubMotionController;         // MOT
    UBYTE ubActiveFrontEndController; // AFE
    UBYTE ubEncoderManager;
    UBYTE ubEncFeedBackSelection;
    UBYTE ubSpare[8*sizeof(UBYTE)];
} DRVTSKCTRL_TASKOPTION;

typedef struct
{
    UBYTE ubMainOperation;

    UBYTE ubDummy1;
    DRVTSKCTRL_TASKOPTION ubDummy2;
} DRVTSKCTRL_OLD_PARAM;

typedef struct
{
    UBYTE ubMainOperation;
    UBYTE ubDummy;
} DRVTSKCTRL_NEW_PARAM;

typedef union
{
    DRVTSKCTRL_NEW_PARAM sNew;
    DRVTSKCTRL_OLD_PARAM sOld;
} DRVTSKCTRL_PARAM;

typedef struct
{
    UBYTE ubRtHookPosition;

    DRVTSKCTRL_TASKOPTION sTo;
} DRVTSKCTRL_PLC_CONFIG;

//***************************************************************************
// Globals

extern DRVTSKCTRL_PARAM sDrvTskCtrlParams;
extern const DRVTSKCTRL_NEW_PARAM  sDrvTskCtrlDefParams;

extern DRVTSKCTRL_PLC_CONFIG sDrvTskCtrlPlcConfig;

extern UBYTE ubDrvTskCtrlMainOpSelected;

//***************************************************************************
// Globals

BOOL DrvTskCtrl_Init(void);

UBYTE DrvTskCtrl_Config(UBYTE ubOperation);

#endif
