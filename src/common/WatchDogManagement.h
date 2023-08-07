/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : WatchDogManagement.h                                       */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Infineon WDT module and FPGA WDT management                */
/*                                                                          */
/****************************************************************************/

#ifndef _WATCHDOGMANAGEMENT_H
#define _WATCHDOGMANAGEMENT_H

//***************************************************************************
// Initialization entry points

BOOL WdtMgm_Init(UWORD uwOption);
BOOL WdtMgm_Start(void);

#endif
