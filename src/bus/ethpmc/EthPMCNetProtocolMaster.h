/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCNetProtocolMaster.h                                  */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : EtherPMC Network Protocol Master Handler                   */
/*                                                                          */
/****************************************************************************/

#ifndef _ETHPMCNETPROTOCOLMASTER_H
#define _ETHPMCNETPROTOCOLMASTER_H

#include "EthPMCNetProtocol.h"

// Node Ident
BOOL EPMCNetProtlMaster_NodeIdent(UBYTE ubNodeNum,HPULONG hpulUniqueID);

// Set Node Number
BOOL EPMCNetProtlMaster_SetNodeNum(UBYTE ubNodeNum, UBYTE ubNewNodeNum);

// Configure Realtime data Slot
BOOL EPMCNetProtlMaster_ConfigSlot(UBYTE ubNodeNum, UWORD uwSlotOffset, UWORD uwSlotSize);

// Enable/Disable Output Port
BOOL EPMCNetProtlMaster_EnOutPort(UBYTE ubNodeNum, BOOL bFlag);

// Get Output Port Link Status
BOOL EPMCNetProtlMaster_GetLinkStatus(UBYTE ubNodeNum, HPUWORD hpuLinkStatus);

// Network state request
BOOL EPMCNetProtlMaster_NetStateReq(UBYTE ubNodeNum,UBYTE ubNetState);

// Network frame timeout
BOOL EPMCNetProtlMaster_FrameTimeout(UBYTE ubNodeNum,UWORD uwTimeout);

#endif