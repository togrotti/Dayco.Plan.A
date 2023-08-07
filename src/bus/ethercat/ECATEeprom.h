/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ECATEeprom.h                                               */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : ecat EEPROM management                                     */
/*                                                                          */
/****************************************************************************/

#ifndef _ECATEEPROM_H
#define _ECATEEPROM_H

//***************************************************************************
// Globals

extern BOOL bECATEESaveRequested;

//***************************************************************************
// Global functions

BOOL ECATEE_DataInit(void);
void ECATEE_Emulation(void);

#endif
