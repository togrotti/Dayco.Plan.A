/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : GlobalResetCodes.h                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Global code passed through system reset for reset cause id */
/*                                                                          */
/****************************************************************************/

#ifndef _GLOBALRESETCODES_H
#define _GLOBALRESETCODES_H

//***************************************************************************
// Define

#define RESETCODE_BOOTBLOCKRESERVED             0x1000

//***************************************************************************
// Bootblock

#define RESETCODE_ENTERBOOTWMODBUSSER0          0x0001
#define RESETCODE_ENTERBOOT                     0x0002
#define RESETCODE_ENTERBOOTANDUPGRADE           0x0003

//***************************************************************************
// SysApp

#define RESETCODE_CLEANREBOOTREQUESTED          0xAA12
#define RESETCODE_POFSTOREDATA                  0x9A3B
#define RESETCODE_NOLOADPARAMSREQUESTED         0x39a1
#define RESETCODE_ENTERDIAGNOSTICAPPLICATION    0x7d03
#define RESETCODE_PLCREBOOTMINIMAL              0xC4A9
#define RESETCODE_CLEANREBOOT_COMP              0x496D
#define RESETCODE_CLEANREBOOT_PERF              0x3E56

//***************************************************************************
// RemDisp

#define RESETCODE_RD_SWITCH_TO_HMISYS           0x74F1
#define RESETCODE_RD_SWITCH_TO_HMIAPP           0xE356

#endif
