/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Interrupt.c                                                */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Global Interrupt Handler                                   */
/*                                                                          */
/****************************************************************************/

/***************************** Include Files *********************************/
#include "xil_io.h"
#include "xscugic.h"

#include "Interrupt.h"

/************************** Variable Definitions **************************/
#ifdef _AXX_BOOTBLOCK
//XScuGic xInterruptController;
#endif

/*****************************************************************************/
/**
*
* Interrupt Initialization
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
void Intr_Init(void)
{
	if(!xInterruptController.IsReady)
	{
		/* Initialize the interrupt controller driver so that it is ready to use. */
		XScuGic_Config* GicConfig = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);
		XScuGic_CfgInitialize(&xInterruptController, GicConfig, GicConfig->CpuBaseAddress);

		/* Perform a self-test to ensure that the hardware was built correctly */
		XScuGic_SelfTest(&xInterruptController);

		/* Connect the interrupt controller interrupt handler to the hardware
		  interrupt handling logic in the ARM processor. */
		Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			 (Xil_ExceptionHandler) XScuGic_InterruptHandler,
			 &xInterruptController);

		/* Enable interrupts in the ARM */
		Xil_ExceptionEnable();
	}
}
