/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Gpio.h                                                     */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Gpio manager                                               */
/*                                                                          */
/****************************************************************************/

//***************************************************************************
// Include
#ifndef ___GPIO_H___
#define ___GPIO_H___

/***************************** Include Files *********************************/
#include "xil_io.h"
#include "xgpiops.h"

/************************** Constant Definitions *****************************/
#define GPIO_DIR_OUT  1
#define GPIO_DIR_IN   0

#define MIO_PIN_BASE  0
#define EMIO_PIN_BASE 54

#define MIO_BANK0_BASE 0
#define MIO_BANK1_BASE 32
#define MIO_BANK2_BASE 54
#define MIO_BANK3_BASE 86

#define GPIO_OUT(p,v)         XGpioPs_WritePin(&Gpio,p,v)
#define GPIO_IN(p)            XGpioPs_ReadPin(&Gpio,p)
#define GPIO_BANK_OUT(b,m,v)  Gpio_Write(b,m,v)
#define GPIO_BANK_IN(b)       XGpioPs_Read(&Gpio,b)

/************************** Variable Definitions *****************************/
extern XGpioPs Gpio;

/************************** Function Prototypes ******************************/
u32  Gpio_Init(void);
void Gpio_SetMode(u32 Pin, u32 Direction);
void Gpio_Write(u8 Bank, u32 Mask, u32 Data);

#endif /* ___GPIO_H___ */
