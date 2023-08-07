/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2017, Phase Motion Control. All Rights Reserved.              */
/*                                                                          */
/* File        : PSUManager.c                                               */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Power Supply Unit SPI communication manager                */
/*                                                                          */
/****************************************************************************/

#ifndef _PSUMANAGER_H
#define _PSUMANAGER_H

#ifdef _HW_DC
#if CFG_PSU_MNGR
#include "common\CommonController.h"

//***************************************************************************
// Data structures

typedef struct
{
    const COMCTRL_HANDLERS  *psCtrlHandlers ;
} PSUBRD_IN;

typedef struct
{
/*    union
    {
        struct
        {
            UWORD bEnableModule:1;          // enable module
        } f;
        UWORD w;
    } flags;
*/
  BOOL bEnableModule;
} PSU_WORKS;

typedef struct
{
    ULONG               ulVersion;
    ULONG               ulModuleBuild;
} PSU_INFO;

typedef struct
{
    UWORD               uwAlarmCode;
    UWORD               uwWarnCode;
    SLONG               slVoltageDcLink;
    SLONG               slCurrentDcLink;
    FLOAT               flTempBridge;
    FLOAT               flTempBrake;
    SWORD               swVoltageDcLinkPeak;
    SWORD               swCurrentDcLinkPeak;
    SWORD               swVoltageRS;
    SWORD               swVoltageST; 
    SWORD               swVoltageTR;
    SWORD               swVoltagePhaseR;
    SWORD               swVoltagePhaseS;
    SWORD               swVoltagePhaseT;
	UWORD               uwSysState;
	UWORD               uwPowerOK;
	UWORD               uwInputEnable;
	UWORD               uwReady;
	ULONG               ulEnergy;             /* Added for getting brake risister energy */
} PSU_COMM_DATA;


typedef struct
{    
    union
    {
        struct
        {
          BOOL bPowerOn;  //UWORD bPowerOn:1;          // power on
          BOOL bResetPSUFault;  //UWORD bResetPSUFault:1;         // reset PSU alarm code
        } f;
//        UWORD w;
    } __attribute__((aligned(4))) flags;
} PSU_PARAM;

//***************************************************************************
// Globals

extern PSU_WORKS        sPSUWrk;
extern PSUBRD_IN        sPSUBrdIn;
extern PSU_INFO         sPSUInfo;
extern PSU_COMM_DATA    sPSUCommData;
extern PSU_PARAM        sPSUInParam;

//***************************************************************************
// Initialization entry point

BOOL PSUManager_Init(UWORD);
void PSUManager_GetHwOpt(UWORD, ULONG *, ULONG *);
#endif // cfg_psu_mngr

#endif // _hw_dc
#endif // _psumanager_h
