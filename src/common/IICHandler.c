/*****************************************************************************/
/* Project: Ax-Zynq Control Board                                            */
/*                                                                           */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved   */
/*                                                                           */
/* File        : IICHander.c                                                 */
/* Author      : WF                                                          */
/*                                                                           */
/* Description : IIC hander                                                  */
/*                                                                           */
/*****************************************************************************/
//#include <intrins.h>
#include <stdio.h>

#ifdef _AXX_SYSAPP
#include "system\SysAppGlobals.h"
#endif

#if CFG_IIC
//#include "platform.h"
//#include "xil_printf.h"

#include "sleep.h"
#include "xiicps.h"

XIicPs IicInstance;     /* The instance of the IIC device. */

#include "drive\AxM-E-Defines.h"
#include "CommonDefines.h"
#include "IICHandler.h"


#define IIC_SCLK_RATE		400000

#define IIC_EXTERNAL_ID		XPAR_PS7_I2C_0_DEVICE_ID
#define IIC_IOEXP_ID		XPAR_PS7_I2C_1_DEVICE_ID

XIicPs_Config *ConfigPtrExter;   /* Pointer to configuration data */
XIicPs_Config *ConfigPtrIOexp;   /* Pointer to configuration data */
int IIC_HandlerInit( void )
{
    int Status;

    ConfigPtrExter = XIicPs_LookupConfig(IIC_EXTERNAL_ID);
    if (ConfigPtrExter == NULL) {
        return XST_FAILURE;
    }

    Status = XIicPs_CfgInitialize(&IicInstance, ConfigPtrExter,
    		ConfigPtrExter->BaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    XIicPs_SetSClk(&IicInstance, IIC_SCLK_RATE);

    ConfigPtrIOexp = XIicPs_LookupConfig(IIC_IOEXP_ID);
    if (ConfigPtrIOexp == NULL) {
        return XST_FAILURE;
    }

    Status = XIicPs_CfgInitialize(&IicInstance, ConfigPtrIOexp,
    		ConfigPtrIOexp->BaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    XIicPs_SetSClk(&IicInstance, IIC_SCLK_RATE);

    return XST_SUCCESS;
}



unsigned char IIC_devicesearch( unsigned char bus, IIC_SEARCH_STATE *state, unsigned char deviceaddr )
{
   unsigned char result = FALSE;
   int Status;
   u8 WriteBuffer[4]={0};
   if(bus == IIC_INTERNAL_BUS)
	   return result;
   if(IIC_EXTERNAL_BUS == bus)
   	{
   	    ConfigPtrExter = XIicPs_LookupConfig(IIC_EXTERNAL_ID);
   	    if (ConfigPtrExter == NULL) {
   	        return XST_FAILURE;
   	    }

   	    Status = XIicPs_CfgInitialize(&IicInstance, ConfigPtrExter,
   	    		ConfigPtrExter->BaseAddress);
   	    if (Status != XST_SUCCESS) {
   	        return XST_FAILURE;
   	    }

   	    XIicPs_SetSClk(&IicInstance, IIC_SCLK_RATE);
   	}
   	else if( IIC_IOEXP_BUS == bus )
   	{
   		 ConfigPtrIOexp = XIicPs_LookupConfig(IIC_IOEXP_ID);
   		    if (ConfigPtrIOexp == NULL) {
   		        return XST_FAILURE;
   		    }

   		    Status = XIicPs_CfgInitialize(&IicInstance, ConfigPtrIOexp,
   		    		ConfigPtrIOexp->BaseAddress);
   		    if (Status != XST_SUCCESS) {
   		        return XST_FAILURE;
   		    }

   		    XIicPs_SetSClk(&IicInstance, IIC_SCLK_RATE);
   	}

   Status = XIicPs_MasterSendPolled(&IicInstance, WriteBuffer, 0, deviceaddr>>1);

   if(Status == XST_SUCCESS)
   {
	   state->IICBusType = bus;
	   state->DeviceAddress = deviceaddr;
	   result = TRUE;
   }

   return result;

}


int IIC_ReadMemory( unsigned char bus, unsigned char *devaddr, unsigned char *outputbuffer, unsigned short baseaddr, unsigned short lenth )
{
	unsigned char device_addr = 0;      //EEPROM ADDRESS
	u8 WriteBuffer[4]={0};
	unsigned short memory_addr = baseaddr;     //interal address in EEPROM
    int Status;

    if(bus == IIC_INTERNAL_BUS)
 	   return FALSE;

	if(IIC_EXTERNAL_BUS == bus)
	{
	    ConfigPtrExter = XIicPs_LookupConfig(IIC_EXTERNAL_ID);
	    if (ConfigPtrExter == NULL) {
	        return FALSE;
	    }

	    Status = XIicPs_CfgInitialize(&IicInstance, ConfigPtrExter,
	    		ConfigPtrExter->BaseAddress);
	    if (Status != XST_SUCCESS) {
	        return FALSE;
	    }

	    XIicPs_SetSClk(&IicInstance, IIC_SCLK_RATE);
	}
	else if( IIC_IOEXP_BUS == bus )
	{
		 ConfigPtrIOexp = XIicPs_LookupConfig(IIC_IOEXP_ID);
		    if (ConfigPtrIOexp == NULL) {
		        return FALSE;
		    }

		    Status = XIicPs_CfgInitialize(&IicInstance, ConfigPtrIOexp,
		    		ConfigPtrIOexp->BaseAddress);
		    if (Status != XST_SUCCESS) {
		        return FALSE;
		    }

		    XIicPs_SetSClk(&IicInstance, IIC_SCLK_RATE);
	}

	device_addr = *devaddr;
	device_addr = device_addr>>1;

	if(memory_addr&0xFF00)
	{
		WriteBuffer[0] = memory_addr>>8;
		WriteBuffer[1] = memory_addr&0xFF;
		Status=XIicPs_MasterSendPolled(&IicInstance, WriteBuffer, 2, device_addr);
	}
	else
	{
		WriteBuffer[0] = memory_addr&0xFF;
		Status=XIicPs_MasterSendPolled(&IicInstance, WriteBuffer, 1, device_addr);
	}

	if(Status != XST_SUCCESS)
		return FALSE;

	while (XIicPs_BusIsBusy(&IicInstance));     //delay bus free

//	usleep(200000);

	Status=XIicPs_MasterRecvPolled(&IicInstance, outputbuffer, lenth, device_addr);

	if(Status != XST_SUCCESS)
		return FALSE;

	while (XIicPs_BusIsBusy(&IicInstance));

   return TRUE;
}

#endif // cfg_iic
