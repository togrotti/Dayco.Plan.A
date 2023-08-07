/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCNetProtocolSlave.h                                   */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : EtherPMC Network Protocol Slave Handler                    */
/*                                                                          */
/****************************************************************************/

#ifndef _ETHPMCNETPROTOCOLSLAVE_H
#define _ETHPMCNETPROTOCOLSLAVE_H

#include "EthPMCNetProtocol.h"

//***************************************************************************
// Network Protocol Slave: Process 
// GetTransaction --> Command Decode&Execute --> SendTransaction
BOOL EPMCNetProtlSlave_Process(HPUWORD hpuDataIn,UWORD uwNetProtlSize,HPUWORD hpuDataOut,HPUWORD puwSizeOut);

// Get Network State
UBYTE EPMCNetProtlSlave_GetNetState(void);

#endif