/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : PanelManager.h                                             */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Panel leds and keyboard management                         */
/*                                                                          */
/****************************************************************************/

#ifndef _PANELMANAGER_H
#define _PANELMANAGER_H

//***************************************************************************
// Globals

extern ULONG ulPnlMgrRDLedStatus;

//***************************************************************************
// Global functions

BOOL PanelMgr_Init(void);

#endif
