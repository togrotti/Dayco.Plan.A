/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ModBusCommandMgr.h                                         */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : ModBus/Cockpit specific command manager                    */
/*                                                                          */
/****************************************************************************/

#ifndef _MODBUSCOMMANDMGR_H
#define _MODBUSCOMMANDMGR_H

// #include "common\DefineExternals.h"
#include "common\CommonParamDB.h"

//***************************************************************************
// Global system status

#define MODBUSSM_BIT_DRIVEENABLE              0

#define MODBUSSM_BIT_SOFTWAREDRIVERESET       16
#define MODBUSSM_BIT_SAVESYSTPARAMETERS       17
#define MODBUSSM_BIT_CLEARSYSTEMERRORS        18
#define MODBUSSM_BIT_RESETANDENTERBOOTBLOCK   19

#define MODBUSSM_BIT_ENABLESIMULATEDINPUTS    20
#define MODBUSSM_BIT_DISABLEPHYSICALOUTPUTS   21

#define MODBUSSM_BIT_ENTERBOOTANDUPGRADE      22

#define MODBUSSM_BIT_SAVEHWASSEMBLYINFO       29
#define MODBUSSM_BIT_RESTOREDEFAULTPARAMS     30

//****************************************************************************
// Global system flags
typedef struct{
   BOOL b[31];
} MODBUSSMSYSTEMFLAGBITS;

extern MODBUSSMSYSTEMFLAGBITS            sModBusSMSystemFlagBits;

#define bModBusSMDriveEnable             sModBusSMSystemFlagBits.b[MODBUSSM_BIT_DRIVEENABLE]

#define bModBusSMSoftwareDriveReset      sModBusSMSystemFlagBits.b[MODBUSSM_BIT_SOFTWAREDRIVERESET]
#define bModBusSMSaveSystParameters      sModBusSMSystemFlagBits.b[MODBUSSM_BIT_SAVESYSTPARAMETERS]
#define bModBusSMClearSystemErrors       sModBusSMSystemFlagBits.b[MODBUSSM_BIT_CLEARSYSTEMERRORS]
#define bModBusSMResetAndEnterBootBlock  sModBusSMSystemFlagBits.b[MODBUSSM_BIT_RESETANDENTERBOOTBLOCK]

#define bModBusSMEnableSimulatedInputs   sModBusSMSystemFlagBits.b[MODBUSSM_BIT_ENABLESIMULATEDINPUTS]
#define bModBusSMDisablePhysicalOutputs  sModBusSMSystemFlagBits.b[MODBUSSM_BIT_DISABLEPHYSICALOUTPUTS]

#ifdef _APP_XC
#define bModBusSMEnterBootAndUpgrade     sModBusSMSystemFlagBits.b[MODBUSSM_BIT_ENTERBOOTANDUPGRADE]
#endif

#define bModBusSMSaveHwAssemblyInfo      sModBusSMSystemFlagBits.b[MODBUSSM_BIT_SAVEHWASSEMBLYINFO]
#define bModBusSMRestoreDefaultParams    sModBusSMSystemFlagBits.b[MODBUSSM_BIT_RESTOREDEFAULTPARAMS]

//***************************************************************************
// Defines

#define MODBUSCM_STATUSLIST_DEPTH             32
#define MODBUSCM_STATUSLIST_ALARM             0x1000
#define MODBUSCM_STATUSLIST_WARNING           0x2000
#define MODBUSCM_STATUSLIST_PARCHECK          0x4000
#define MODBUSCM_STATUSLIST_PARGRPCHECK       0x8000

#define MODBUSCM_IMMRESTORE_KEY               0xde1a
#define MODBUSCM_ENTERDIAGNOSTICAPP_KEY       0xa15c

//***************************************************************************
// Globals

extern UWORD  hpuwModBusCMStatList[MODBUSCM_STATUSLIST_DEPTH+3];
extern UWORD  uwModBusCMStatListCount;
extern UWORD  uwModBusCMImmRestore;
extern UWORD  uwModBusCMEnterDiagnosticApp;

extern TaskHandle_t ModbusCMTaskHandler;

//***************************************************************************
// Initialization entry point

BOOL ModBusCM_Init(UWORD uwParam);
UWORD ModBusCM_StatListRead(COMMONPARAMDB_ENTRY  *, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext);
void ModbusCM_GetHwOpt(UWORD, ULONG * pulHwOpt, ULONG * pulFPGAOpt);

#endif
