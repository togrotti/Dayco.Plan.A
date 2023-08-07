/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Adc.h                                                      */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : XADC manager                                               */
/*                                                                          */
/****************************************************************************/

//***************************************************************************
// Include
#ifndef ___ADC_H___
#define ___ADC_H___

/***************************** Include Files *********************************/
#include "common\CommonTypedef.h"

/************************** Constant Definitions *****************************/

/************************** Variable Definitions *****************************/
typedef struct
{
   UWORD uwRawData;
   FLOAT fCurData;
} ADC_DATA;

/************************** Function Prototypes ******************************/
u32 Adc_Init(void);
UWORD Adc_GetVPVNData(void);
UWORD Adc_GetAuxData(UBYTE ubChnId);
FLOAT Adc_GetOnChipTemperature(void);
FLOAT Adc_GetVccPint(void);
FLOAT Adc_GetVccPaux(void);
FLOAT Adc_GetVccPdro(void);

#endif /* ___ADC_H___ */
