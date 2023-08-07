/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCMboxMaster.h                                         */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Mailbox master handler                                     */
/*                                                                          */
/****************************************************************************/

#ifndef _ETHPMCMBOXMASTER_H
#define _ETHPMCMBOXMASTER_H

#include "EthPMCMbox.h"

//***************************************************************************
// Mailbox Master: Initial
void EPMCMboxMaster_Init(void);

//***************************************************************************
// Mailbox Master Transaction
BOOL EPMCMboxMaster_Transaction(UBYTE ubNodeNum, UBYTE ubProtocol, HPUWORD hpuDataIn, UWORD uwSizeIn, HPUWORD hpuDataOut, HPUWORD hpuSizeOut);

#endif
