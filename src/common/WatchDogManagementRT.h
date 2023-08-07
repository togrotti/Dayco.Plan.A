/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : WatchDogManagementRT.h                                     */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Infineon WDT module and FPGA WDT management                */
/*                                                                          */
/****************************************************************************/

#ifndef _WATCHDOGMANAGEMENTRT_H
#define _WATCHDOGMANAGEMENTRT_H

//***************************************************************************
// Globals

extern BOOL bWdtFastFailed;
extern BOOL bWdtFastEnabled;

//***************************************************************************
// Prototypes

BOOL WdtMgmRTTask(void);

#endif
