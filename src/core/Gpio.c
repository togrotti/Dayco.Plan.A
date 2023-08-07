/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Gpio.c                                                     */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : GPIO manager: MIO & EMIO                                   */
/*                                                                          */
/****************************************************************************/

/***************************** Include Files ********************************/

#include "xparameters.h"
#include "xgpiops.h"
#include "xstatus.h"

#include "Gpio.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

/************************** Constant Definitions ****************************/

#ifndef GPIO_DEVICE_ID
#define GPIO_DEVICE_ID		XPAR_XGPIOPS_0_DEVICE_ID
#endif

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions *******************/

/************************** Function Prototypes ****************************/

/************************** Variable Definitions **************************/

XGpioPs Gpio;	/* The driver instance for GPIO Device. */

/*****************************************************************************/
/**
*
* Gpio Initialization
*
* @param	None
*
* @return
*		- XST_SUCCESS if the example has completed successfully.
*		- XST_FAILURE if the example has failed.
*
* @note		None
*
******************************************************************************/
u32 Gpio_Init(void)
{
   int Status;
   XGpioPs_Config * pGpioCfg;

   pGpioCfg = XGpioPs_LookupConfig(GPIO_DEVICE_ID);

   Status = XGpioPs_CfgInitialize(&Gpio,pGpioCfg,pGpioCfg->BaseAddr);
   if (Status != XST_SUCCESS) {
      return XST_FAILURE;
   }

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* Gpio Set Input&Output Mode
*
* @param	Pin is the pin number to which the Data is to be written.
*		Valid values are 0-117 in Zynq and 0-173 in Zynq Ultrascale+ MP.
* @param	Direction is the direction to be set for the specified pin.
*		Valid values are 0 for Input Direction, 1 for Output Direction.
*
* @return
*		- XST_SUCCESS if the example has completed successfully.
*		- XST_FAILURE if the example has failed.
*
* @note		This function will not return if the test is running.
*
******************************************************************************/
void Gpio_SetMode(u32 Pin, u32 Direction)
{
   XGpioPs_SetDirectionPin(&Gpio,Pin,Direction);
   if(Direction == GPIO_DIR_OUT)
      XGpioPs_SetOutputEnablePin(&Gpio,Pin,1);
}

/****************************************************************************/
/**
*
* Write data to the specified bank with mask.
*
* @param	InstancePtr is a pointer to the XGpioPs instance.
* @param	Pin is the pin number to which the Data is to be written.
*		Valid values are 0-117 in Zynq and 0-173 in Zynq Ultrascale+ MP.
* @param	Data is the data to be written to the specified pin (0 or 1).
*
* @return	None.
*
* @note		This function does a masked write to the specified pin of
*		the specified GPIO bank. The previous state of other pins
*		is maintained.
*
*****************************************************************************/
void Gpio_Write(u8 Bank, u32 Mask, u32 Data)
{
   u32 RegOffset;
   u32 Value;
   u32 DataVar = Data;
   XGpioPs *InstancePtr = &Gpio;

   Xil_AssertVoid(InstancePtr != NULL);
   Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

   if (Mask & 0x0000FFFF)
   {
      RegOffset = XGPIOPS_DATA_LSW_OFFSET;

      /*
         * Get the 32 bit value to be written to the Mask/Data register where
         * the upper 16 bits is the mask and lower 16 bits is the data.
         */
      Value = ~((Mask & 0x0000FFFF)<<16) & ((DataVar & 0x0000FFFF) | 0xFFFF0000U);
      XGpioPs_WriteReg(InstancePtr->GpioConfig.BaseAddr,
               ((u32)(Bank) * XGPIOPS_DATA_MASK_OFFSET) +
               RegOffset, Value);
   }

   if (Mask & 0xFFFF0000)
   {
      RegOffset = XGPIOPS_DATA_MSW_OFFSET;

      /*
         * Get the 32 bit value to be written to the Mask/Data register where
         * the upper 16 bits is the mask and lower 16 bits is the data.
         */
      Value = ~(Mask & 0xFFFF0000) & (((DataVar & 0xFFFF0000)>>16) | 0xFFFF0000U);
      XGpioPs_WriteReg(InstancePtr->GpioConfig.BaseAddr,
               ((u32)(Bank) * XGPIOPS_DATA_MASK_OFFSET) +
               RegOffset, Value);
   }
}
