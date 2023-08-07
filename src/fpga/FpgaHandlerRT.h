/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : FpgaHandlerRT.h                                      */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : FPGA real time WDT management                              */
/*                                                                          */
/****************************************************************************/

#ifndef _FPGAHANDLERRT_H
#define _FPGAHANDLERRT_H

//***************************************************************************
// Globals

extern UWORD uwFpgaWdtKey;

//***************************************************************************
// Prototypes

UWORD FpgaWdtService(void);

#endif
