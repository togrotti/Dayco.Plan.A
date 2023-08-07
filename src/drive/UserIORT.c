/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2021 Ningbo Physis Technology Co.,Ltd. All Rights Reserved.  */
/*                                                                          */
/* File        : UserIORT.c                                                 */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/* Description : User analog/digital IO                                     */
/*                                                                          */
/****************************************************************************/

#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "common\TaskScheduler.h"
#include "fpga\FpgaHandler.h"
#include "common\UnitMeasureConversion.h"
#include "common\DspFunctions.h"
#include "UserIO.h"
#include "UserIORT.h"
#include "bus\modbus\ModBusCommandMgr.h"
#ifdef _INFINEON_
#include <xe167f.h>
#endif

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
//#pragma GCC optimize (0) // _crs_dbg
#pragma GCC optimize (2)

//***************************************************************************
// Globals

UWORD uwUsrMotorBrakeOut;

//***************************************************************************
// realtime task

BOOL UserIO_RT_task(void)
{
#ifndef _APP_LIMITED
    UWORD uwTmp;
    SWORD swTmp;
        // read physical digital inputs
#ifdef _INFINEON_
#ifndef _HW_DC
#ifndef _HW_CT
    if(bUsrIOboardge102)
        uwTmp = (~(((P5_IN >> 8) & 0x00F0) | (P15_IN & 0x000F))) & 0x00FF;
    else
        uwTmp = (~(P15_IN & 0x00FF)) & 0x00FF;
#else
    uwTmp = (~(P15_IN & 0x000F)) & 0x000F;
#endif // _hw_ct
#else
    uwTmp = (~(P5_IN >> 8)) & 0x00FF;
    // reverse bits in byte
    uwTmp = (((uwTmp & 0xaa) >> 1) | ((uwTmp & 0x55) << 1));
    uwTmp = (((uwTmp & 0xcc) >> 2) | ((uwTmp & 0x33) << 2));
    uwTmp = (((uwTmp & 0xf0) >> 4) | ((uwTmp & 0x0f) << 4));
#endif // _hw_dc
#else
    uwTmp = USRIO_DI_IN;
#ifdef _HW_DC
    if(ubIOboardType)     //IO Y just DI0 and DI1
        uwTmp &= 0x03;
#endif
#endif // _infineon_
#endif // _app_limited

        // filter out inputs selected for simulation
    if(!bModBusSMEnableSimulatedInputs)
    {
        sUsrIOCPBase.sDIO.uwInputs = sUsrIOCPBase.uwDgInForceOn
#ifndef _APP_LIMITED
                                 | (uwTmp & ~sUsrIOCPBase.uwDgInForceOff)
#endif
                                 ;
        for(UWORD i=0;i<USRIO_DIN_CHANNELS;i++)
            sPlcDIO.bInputs[i] = (sUsrIOCPBase.sDIO.uwInputs&(1<<i)) == (1<<i);
        
#ifndef _APP_LIMITED
            // analog inputs processing
        (*pfUsrIOAnInpProcessing)();
#endif
    }

        // if IO Exp board present and active
    if(pfUsrIOIOExpProcess)
        (*pfUsrIOIOExpProcess)();

#ifndef _APP_LIMITED
        // filter out forced outputs
    sUsrIOCPBase.sDIO.uwOutputs = 0;
    for(UWORD i=0;i<USRIO_DOUT_CHANNELS;i++)
    {
        if(sUsrIOCPBase.uwDgOutForceOn&(1u<<i))
            sPlcDIO.bOutputs[i] = 1;
        else if(sUsrIOCPBase.uwDgOutForceOff&(1u<<i))
            sPlcDIO.bOutputs[i] = 0;

        // $sm$ : verify why we have to add cast (UWORD)1u
        sUsrIOCPBase.sDIO.uwOutputs |= (sPlcDIO.bOutputs[i] ? ((UWORD)1u<<i) : 0);
    }
    uwTmp = sUsrIOCPBase.sDIO.uwOutputs;

        // write digital outputs
#ifdef _INFINEON_
#ifndef _HW_DC
    if(bUsrIOboardge200)
        P1_OUT_P6 = uwTmp & 0x0001;
    else
        P8_OUT_P0 = uwTmp & 0x0001;
    P8_OUT_P1 = uwTmp & 0x0002;
    P8_OUT_P2 = uwTmp & 0x0004;
    P8_OUT_P3 = uwTmp & 0x0008;
#else
    P9_OUT    = P9_OUT & ~0x000F | (uwTmp & 0x000F);
#endif // _hw_dc
#else
    USRIO_DO_OUT(uwTmp);
#endif // _infineon_

        // write analog outputs
    if(!(sUsrIOCPBase.uwAnOutSimulation&0x0001))
        swTmp = sUsrIOCPData.sAIO.swOutputs[0];
    else
        swTmp = sUsrIOCPData.swOutSimulation[0];
    FPGA_USRIO_ANA_OUT0 = UMCONV_CONVERT_32TO16(&sUsrIOAnalogOut0Conv, (SLONG)_sint16_addlim_16(swTmp, swUsrIOAnalogOut0Offset) << 16);

    if(!(sUsrIOCPBase.uwAnOutSimulation&0x0002))
        swTmp = sUsrIOCPData.sAIO.swOutputs[1];
    else
        swTmp = sUsrIOCPData.swOutSimulation[1];
    FPGA_USRIO_ANA_OUT1 = UMCONV_CONVERT_32TO16(&sUsrIOAnalogOut1Conv, (SLONG)_sint16_addlim_16(swTmp, swUsrIOAnalogOut1Offset) << 16);
#endif

    return TRUE;
}

//***************************************************************************
// analog input: 2 hardware channels processing

void UserIO_RT_aninput_2chanstd(void)
{
    if(!(sUsrIOCPBase.uwAnInSimulation&0x0001))
        sUsrIOCPData.sAIO.swInputs[0] = swUsrIOAnalogInValue[0];
    if(!(sUsrIOCPBase.uwAnInSimulation&0x0002))
        sUsrIOCPData.sAIO.swInputs[1] = swUsrIOAnalogInValue[1];
}

//***************************************************************************
// analog input: 2 software differential channels processing

void UserIO_RT_aninput_2chandiff(void)
{
    if(!(sUsrIOCPBase.uwAnInSimulation&0x0001))
        sUsrIOCPData.sAIO.swInputs[0] = _sint16_sublim_16(swUsrIOAnalogInValue[0], swUsrIOAnalogInValue[1]);
    if(!(sUsrIOCPBase.uwAnInSimulation&0x0002))
        sUsrIOCPData.sAIO.swInputs[1] = _sint16_sublim_16(swUsrIOAnalogInValue[2], swUsrIOAnalogInValue[3]);
}

//***************************************************************************
// analog input: 4 hardware single ended channels processing

void UserIO_RT_aninput_4chanse(void)
{
    if(!(sUsrIOCPBase.uwAnInSimulation&0x0001))
        sUsrIOCPData.sAIO.swInputs[0] = swUsrIOAnalogInValue[0];
//    if(!ubIOboardType)
    {
    	if(!(sUsrIOCPBase.uwAnInSimulation&0x0002))
    		sUsrIOCPData.sAIO.swInputs[1] = swUsrIOAnalogInValue[1];
    	if(!(sUsrIOCPBase.uwAnInSimulation&0x0004))
    		sUsrIOCPData.sAIO.swInputs[2] = swUsrIOAnalogInValue[2];
    }
    if(!(sUsrIOCPBase.uwAnInSimulation&0x0008))
        sUsrIOCPData.sAIO.swInputs[3] = swUsrIOAnalogInValue[3];
}
