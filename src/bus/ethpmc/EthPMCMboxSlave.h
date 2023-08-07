/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCMboxSlave.h                                          */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Mailbox protocol handler, slave implementation             */
/*                                                                          */
/****************************************************************************/

#ifndef _ETHPMCMBOXSLAVE_H
#define _ETHPMCMBOXSLAVE_H

#include "EthPMCMbox.h"

//****************************************************************************
// Data structures

typedef struct
{
    BOOL (*pfHandler)(HPUWORD hpuDataIn,UWORD uwSizeIn,HPUWORD hpuDataOut,HPUWORD puwSizeOut);
    UBYTE ubProtocol;
} EPMCMBOXSL_PROTOCOLS;

//****************************************************************************
// Variables
extern BOOL  bEPMCMboxSlave_WDEnable;
extern BOOL  bEPMCMboxSlave_SyncEnable;
extern BOOL  bEPMCMboxSlave_ErrorCheckEnable;

//****************************************************************************
// Functions

// Mailbox Slave: Initial
void EPMCMboxSlave_Init(void);

// Slave state active
void EPMCMboxSlave_Active(void);

// Enable/disable slave protocol processing
void EPMCMboxSlave_SetProtocol(const EPMCMBOXSL_PROTOCOLS * psProt);

// Processing loop
void EPMCMboxSlave_Process(void);

// Check Node Number
BOOL EPMCMboxSlave_CheckNodeNum(UBYTE ubNodeNum);

// Set Slave Node Number
BOOL EPMCMboxSlave_SetNodeNum(UBYTE ubNodeNum);

// Get Slave Node Number
UBYTE EPMCMboxSlave_GetNodeNum(void);

// Set timeout for watchdog of incoming frame
BOOL EPMCMboxSlave_SetTimeout(UWORD uwTimeout);

// Get timeout for watchdog of incoming frame
UWORD EPMCMboxSlave_GetTimeout();

// Require late action to be performed after master has received the reply
void EPMCMboxSlave_RunLateAction(void (*pfAction)(void));

#endif
