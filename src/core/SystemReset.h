/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SystemReset.h                                              */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Manage system reset and reset recovering; it use the       */
/*               two memory marker registers (SCU_MKMEM0/1) to pass         */
/*               parameters across reset                                    */
/*                                                                          */
/****************************************************************************/

#ifndef _SYSTEMRESET_H
#define _SYSTEMRESET_H

//****************************************************************************
// Defines

#define SYSRES_POWERON_VALUE_PAR0       0
#define SYSRES_POWERON_VALUE_PAR1       0

#define SYSRES_REBOOT_POR               0x40
#define SYSRES_REBOOT_SRSTB             0x20
#define SYSRES_REBOOT_DBGRST            0x10
#define SYSRES_REBOOT_SLCRST            0x08

#define SYSRES_STATE_ENTERBOOT          0x01

//****************************************************************************
// Globals variables

extern volatile UWORD uwSysResParam0;
extern volatile UWORD uwSysResParam1;

//****************************************************************************
// Globals functions

void SysRes_Recovery(void);
void SysRes_ExecuteReset(UWORD uwPar0, UWORD uwPar1);
void SysRes_ModifyMultiBoot(ULONG ulAddrOffset);
UBYTE SysRes_RebootReason(void);
void SysRes_RebootStateSet(UBYTE ubMark);
UBYTE SysRes_RebootStateGet(void);
void SysRes_RebootPorClr(void);

//****************************************************************************
// Globals to be created customized per application

void resetsystemrecovery(UWORD uwPar0, UWORD uwPar1);
void resetpoweronreset(void);

#endif
