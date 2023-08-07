/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Interrupt.h                                                */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Global Interrupt Handler                                   */
/*                                                                          */
/****************************************************************************/

#ifndef ___INTERRUPT_H___
#define ___INTERRUPT_H___

/***************************** Include Files *********************************/
#include "xil_io.h"
#include "xscugic.h"

/************************** Variable Definitions *****************************/
extern XScuGic xInterruptController;

/************************** Function Prototypes ******************************/
void  Intr_Init(void);

#endif /* ___INTERRUPT_H___ */
