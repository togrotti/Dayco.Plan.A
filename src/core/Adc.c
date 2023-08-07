/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Adc.c                                                      */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : XADC manager                                               */
/*                                                                          */
/****************************************************************************/

/***************************** Include Files ********************************/

#include "Adc.h"
#include "xparameters.h"
#include "xstatus.h"
#include "fpga\FpgaHandler.h"
#include "assert.h"
#include "xil_io.h"
#include "xadcps_hw.h"

#ifdef XPAR_SYSMON_0_DEVICE_ID
#include "xadcps.h"
#define ADC_DEVICE_ID         XPAR_SYSMON_0_DEVICE_ID
#endif

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (0)

/************************** Constant Definitions ****************************/
#ifndef _HW_DC
/* AD0: Board temp
 * AD1: Motor temp
 * AD2: Vencoder
 * AD8: Power Board temp (NTC igbt)
 * AD9: 24V aux
 * ZynQ Vcc Pint
 * ZynQ Vcc Paux
 * ZynQ Vcc Pdro
 * ZynQ OnChipTemp
 */
#define ADC_SEQ_CH_AUX_MASK    XADCPS_SEQ_CH_AUX00   | \
                               XADCPS_SEQ_CH_AUX01   | \
                               XADCPS_SEQ_CH_AUX02   | \
                               XADCPS_SEQ_CH_AUX08   | \
                               XADCPS_SEQ_CH_AUX09   | \
                               XADCPS_SEQ_CH_VCCPINT | \
                               XADCPS_SEQ_CH_VCCPAUX | \
                               XADCPS_SEQ_CH_VCCPDRO | \
                               XADCPS_SEQ_CH_TEMP
#else

#ifndef _HW_AXS

#define ADC_SEQ_CH_AUX_MASK    XADCPS_SEQ_CH_VPVN    | \
                               XADCPS_SEQ_CH_AUX02   | \
                               XADCPS_SEQ_CH_AUX03   | \
                               XADCPS_SEQ_CH_AUX08   | \
                               XADCPS_SEQ_CH_VCCPINT | \
                               XADCPS_SEQ_CH_VCCPAUX | \
                               XADCPS_SEQ_CH_VCCPDRO | \
                               XADCPS_SEQ_CH_TEMP

#else // _HW_AXS

#if defined(_HW_AXS_SAIETTA)
#define ADC_SEQ_CH_AUX_MASK    XADCPS_SEQ_CH_AUX00   | \
                               XADCPS_SEQ_CH_AUX08   | \
                               XADCPS_SEQ_CH_AUX01   | \
                               XADCPS_SEQ_CH_AUX09   | \
                               XADCPS_SEQ_CH_AUX02   | \
                               XADCPS_SEQ_CH_AUX10   | \
                               XADCPS_SEQ_CH_VCCPINT | \
                               XADCPS_SEQ_CH_VCCPAUX | \
                               XADCPS_SEQ_CH_VCCPDRO | \
                               XADCPS_SEQ_CH_TEMP
// Note : add unconnected PT100, PTC and Temp_PCB


#elif defined(_HW_AXS_CABI35KW)
#define ADC_SEQ_CH_AUX_MASK    XADCPS_SEQ_CH_AUX00   | \
                               XADCPS_SEQ_CH_AUX08   | \
                               XADCPS_SEQ_CH_AUX01   | \
                               XADCPS_SEQ_CH_AUX09   | \
                               XADCPS_SEQ_CH_VCCPINT | \
                               XADCPS_SEQ_CH_VCCPAUX | \
                               XADCPS_SEQ_CH_VCCPDRO | \
                               XADCPS_SEQ_CH_TEMP

#elif defined(_HW_AXS_DAYCO22KW)
#define ADC_SEQ_CH_AUX_MASK    XADCPS_SEQ_CH_AUX00   | \
                               XADCPS_SEQ_CH_AUX08   | \
                               XADCPS_SEQ_CH_AUX01   | \
                               XADCPS_SEQ_CH_AUX09   | \
                               XADCPS_SEQ_CH_AUX02   | \
                               XADCPS_SEQ_CH_AUX10   | \
                               XADCPS_SEQ_CH_AUX06   | \
                               XADCPS_SEQ_CH_VCCPINT | \
                               XADCPS_SEQ_CH_VCCPAUX | \
                               XADCPS_SEQ_CH_VCCPDRO | \
                               XADCPS_SEQ_CH_TEMP

#endif




#endif // _HW_AXS

#endif

#ifdef ADC_DEVICE_ID
/************************** Function Prototypes ****************************/
static void Adc_SetAuxMode(void);

/************************** Variable Definitions **************************/
static XAdcPs  xAdc;

#else

/************************** Function Prototypes ****************************/
static void ADC_SetSequencerMode(u8 SequencerMode);
static void ADC_SetAvg(u8 Average);
static u8 ADC_GetSequencerMode(void);
static int ADC_SetSeqInputMode(u32 InputModeChMask);
static int ADC_SetSeqAvgEnables(u32 AvgEnableChMask);
static int ADC_SetSeqChEnables(u32 ChEnableMask);

/***************** Macros (Inline Functions) Definitions *******************/

#define ADC_RawToVoltage(AdcData) 					\
	((((float)(AdcData))* (3.0f))/65536.0f)

#define ADC_RawToTemperature(AdcData)				\
	((((float)(AdcData)/65536.0f)/0.00198421639f ) - 273.15f)

#define SEQ_MODE_SAFE            0  /**< Default Safe Mode */
#define SEQ_MODE_ONEPASS         1  /**< Onepass through Sequencer */
#define SEQ_MODE_CONTINPASS      2  /**< Continuous Cycling Sequencer */
#define SEQ_MODE_SINGCHAN        3  /**< Single channel -No Sequencing */
#define SEQ_MODE_SIMUL_SAMPLING  4  /**< Simultaneous sampling */
#define SEQ_MODE_INDEPENDENT     8  /**< Independent mode */

#define AVG_0_SAMPLES            0  /**< No Averaging */
#define AVG_16_SAMPLES           1  /**< Average 16 samples */
#define AVG_64_SAMPLES           2  /**< Average 64 samples */
#define AVG_256_SAMPLES          3  /**< Average 256 samples */

#define ADC_READREG(a)           FPGA_REGISTER_16(FPGA_XADC_BASEADDR + ((a)<<1))
#define ADC_WRITEREG(a,d)        FPGA_REGISTER_16(FPGA_XADC_BASEADDR + ((a)<<1))=d

#endif

/*****************************************************************************/
/**
*
* Adc Initialization
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
u32 Adc_Init(void)
{
    int Status;

#ifdef ADC_DEVICE_ID
    XAdcPs_Config * pAdcCfg;

    /*
    * Initialize the XAdc driver.
    */
    pAdcCfg = XAdcPs_LookupConfig(ADC_DEVICE_ID);
    if (pAdcCfg == NULL) {
       return XST_FAILURE;
    }
    Status = XAdcPs_CfgInitialize(&xAdc, pAdcCfg,
             pAdcCfg->BaseAddress);
    if (Status != XST_SUCCESS) {
       return XST_FAILURE;
    }

    /*
    * Self Test the XADC/ADC device
    */
    Status = XAdcPs_SelfTest(&xAdc);
    if (Status != XST_SUCCESS) {
       return XST_FAILURE;
    }

    /*
    * Configuring the Sequence registers
    */

    // * Disable the Channel Sequencer before configuring the Sequence registers.
    XAdcPs_SetSequencerMode(&xAdc, XADCPS_SEQ_MODE_SAFE);

    /*
    * Sets the number of samples of averaging that is to be done for
    * all the channels in both the single channel mode and sequence mode of
    * operations.
    */
     XAdcPs_SetAvg(&xAdc, XADCPS_AVG_16_SAMPLES);

    /*
    * Set the ADCCLK frequency equal to 1/32 of System clock for the System
    * Monitor/ADC in the Configuration Register 2.
    */
    // XAdcPs_SetAdcClkDivisor(&xAdc, 4); // 200MHz/4=50MHz

    // * Set the Analog input mode: 1 - bipolar, 0 - unipolar
    Status = XAdcPs_SetSeqInputMode(&xAdc, 0);
    if (Status != XST_SUCCESS) {
       return XST_FAILURE;
    }

    // * Set Acquisition cycles: 1 - 10 ADCCLK, 0 - 4 ADCCLK
    // Status = XAdcPs_SetSeqAcqTime(&xAdc, 0);
    if (Status != XST_SUCCESS) {
       return XST_FAILURE;
    }

    // * Enables the averaging for the specified channels
     Status = XAdcPs_SetSeqAvgEnables(&xAdc, ADC_SEQ_CH_AUX_MASK);
     if (Status != XST_SUCCESS) {
        return XST_FAILURE;
     }

    // * Enable channels in the Sequencer registers
    Status = XAdcPs_SetSeqChEnables(&xAdc, ADC_SEQ_CH_AUX_MASK);
    if (Status != XST_SUCCESS) {
       return XST_FAILURE;
    }

    // * Enable external Mux and connect to Aux CH0.
//    XAdcPs_SetMuxMode(&xAdc, TRUE, 0x10);  /* 0b'10000 to CH[4:0] */
   
    // * Enable the Channel Sequencer in continuous sequencer cycling mode.
    XAdcPs_SetSequencerMode(&xAdc, XADCPS_SEQ_MODE_CONTINPASS);
#else
    /*
    * Configuring the Sequence registers
    */

    // * Disable the Channel Sequencer before configuring the Sequence registers.
    ADC_SetSequencerMode(SEQ_MODE_SAFE);

    /*
    * Sets the number of samples of averaging that is to be done for
    * all the channels in both the single channel mode and sequence mode of
    * operations.
    */
    ADC_SetAvg(AVG_16_SAMPLES);

    // * Set the Analog input mode: 1 - bipolar, 0 - unipolar
    Status = ADC_SetSeqInputMode(0);
    if (Status != XST_SUCCESS) {
       return XST_FAILURE;
    }

    // * Enables the averaging for the specified channels
    Status = ADC_SetSeqAvgEnables(ADC_SEQ_CH_AUX_MASK);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // * Enable channels in the Sequencer registers
    Status = ADC_SetSeqChEnables(ADC_SEQ_CH_AUX_MASK);
    if (Status != XST_SUCCESS) {
       return XST_FAILURE;
    }
   
    // * Enable the Channel Sequencer in continuous sequencer cycling mode.
    ADC_SetSequencerMode(SEQ_MODE_CONTINPASS);
#endif

    return XST_SUCCESS;
}

#ifdef _HW_DC
/****************************************************************************/
/*
* Read VPVN channel Data
* from the ADC data registers.
*/
UWORD Adc_GetVPVNData(void)
{
#ifdef ADC_DEVICE_ID
   return (UWORD)XAdcPs_GetAdcData(&xAdc, XADCPS_CH_VPVN+ubChnId);
#else
   return (UWORD)ADC_READREG(XADCPS_VPVN_OFFSET);
#endif
}
#endif

/****************************************************************************/
/*
* Read AUX channel Data
* from the ADC data registers.
*/
UWORD Adc_GetAuxData(UBYTE ubChnId)
{
#ifdef ADC_DEVICE_ID
   return (UWORD)XAdcPs_GetAdcData(&xAdc, XADCPS_CH_AUX_MIN+ubChnId);
#else
   return (UWORD)ADC_READREG(XADCPS_AUX00_OFFSET+ubChnId);
#endif
}

#ifdef ADC_DEVICE_ID
/****************************************************************************/
/*
* Set AUX channel Mode: external multiolexer mode 
* and sequencer simltaneous mode
*/
static void Adc_SetAuxMode(void)
{
   /*
      * Enable the following channels in the Sequencer registers:
      * 	for simultaneous sampling mode: Channels 0 and 8
      */
   XAdcPs_SetSeqChEnables(&xAdc, ADC_SEQ_CH_AUX_MASK);

   /*
      * Enable external Mux and connect to Aux CH0.
      * CH4 to CH0: 0b' 1 0 0 0 0
      */
   XAdcPs_SetMuxMode(&xAdc, TRUE, 0x10);  /* 0b'10000 to CH[4:0] */

   /*
      * Enable the Channel Sequencer in continuous sequencer cycling mode.
      */
   // XAdcPs_SetSequencerMode(&xAdc, XADCPS_SEQ_MODE_SIMUL_SAMPLING);
}
#endif

/****************************************************************************/
/*
* Read the on-chip Temperature Data (Current/Maximum/Minimum)
* from the ADC data registers.
*/
FLOAT Adc_GetOnChipTemperature(void)
{
   u16 TempRawData;
#ifdef ADC_DEVICE_ID
   TempRawData = XAdcPs_GetAdcData(&xAdc, XADCPS_CH_TEMP);

   return XAdcPs_RawToTemperature(TempRawData);
#else
   TempRawData = ADC_READREG(XADCPS_TEMP_OFFSET);

   return ADC_RawToTemperature(TempRawData);
#endif
}

/****************************************************************************/
/*
* Read the VccPint Votage Data (Current/Maximum/Minimum) from the
* ADC data registers.
*/
FLOAT Adc_GetVccPint(void)
{
   u16 TempRawData;
#ifdef ADC_DEVICE_ID
   TempRawData = XAdcPs_GetAdcData(&xAdc, XADCPS_CH_VCCPINT);

   return XAdcPs_RawToVoltage(TempRawData);
#else
   TempRawData = ADC_READREG(XADCPS_VCCPINT_OFFSET);

   return ADC_RawToVoltage(TempRawData);
#endif
}

/****************************************************************************/
/*
* Read the VccPaux Votage Data (Current/Maximum/Minimum) from the
* ADC data registers.
*/
FLOAT Adc_GetVccPaux(void)
{
   u16 TempRawData;
#ifdef ADC_DEVICE_ID
   TempRawData = XAdcPs_GetAdcData(&xAdc, XADCPS_CH_VCCPAUX);

   return XAdcPs_RawToVoltage(TempRawData);
#else
   TempRawData = ADC_READREG(XADCPS_VCCPAUX_OFFSET);

   return ADC_RawToVoltage(TempRawData);
#endif
}

/****************************************************************************/
/*
* Read the VccPdro Votage Data (Current/Maximum/Minimum) from the
* ADC data registers.
*/
FLOAT Adc_GetVccPdro(void)
{
   u16 TempRawData;
#ifdef ADC_DEVICE_ID
   TempRawData = XAdcPs_GetAdcData(&xAdc, XADCPS_CH_VCCPDRO);

   return XAdcPs_RawToVoltage(TempRawData);
#else
   TempRawData = ADC_READREG(XADCPS_VCCPDRO_OFFSET);

   return ADC_RawToVoltage(TempRawData);
#endif
}

#ifndef ADC_DEVICE_ID
static void ADC_SetSequencerMode(u8 SequencerMode)
{
   u32 RegValue;

   /*
      * Assert the arguments.
      */
//   assert((SequencerMode <= SEQ_MODE_SIMUL_SAMPLING) ||
//          (SequencerMode == SEQ_MODE_INDEPENDENT));

   /*
      * Set the specified sequencer mode in the Configuration Register 1.
      */
   RegValue = ADC_READREG(XADCPS_CFR1_OFFSET);
   RegValue &= (~ XADCPS_CFR1_SEQ_VALID_MASK);
   RegValue |= ((SequencerMode  << XADCPS_CFR1_SEQ_SHIFT) &
               XADCPS_CFR1_SEQ_VALID_MASK);
   ADC_WRITEREG(XADCPS_CFR1_OFFSET, RegValue);

}

static void ADC_SetAvg(u8 Average)
{
    u32 RegData;

    /*
        * Assert the arguments.
        */
//    assert(Average <= AVG_256_SAMPLES);

    /*
        * Write the averaging value into the Configuration Register 0.
        */
    RegData = ADC_READREG(XADCPS_CFR0_OFFSET) & (~XADCPS_CFR0_AVG_VALID_MASK);

    RegData |=  (((u32) Average << XADCPS_CFR0_AVG_SHIFT));
    ADC_WRITEREG(XADCPS_CFR0_OFFSET, RegData);

}

static u8 ADC_GetSequencerMode(void)
{
    /*
        * Read the channel sequencer mode from the Configuration Register 1.
        */
    return ((u8) ((ADC_READREG(XADCPS_CFR1_OFFSET) & XADCPS_CFR1_SEQ_VALID_MASK) >> XADCPS_CFR1_SEQ_SHIFT));

}

static int ADC_SetSeqInputMode(u32 InputModeChMask)
{
    /*
        * The sequencer must be disabled for writing any of these registers
        * Return XST_FAILURE if the channel sequencer is enabled.
        */
    if ((ADC_GetSequencerMode() != SEQ_MODE_SAFE)) {
        return XST_FAILURE;
    }

    /*
        * Set the input mode for the specified channels in the ADC Channel
        * Analog-Input Mode Sequencer Registers.
        */
    ADC_WRITEREG(XADCPS_SEQ04_OFFSET,
                (InputModeChMask & XADCPS_SEQ04_CH_VALID_MASK));

    ADC_WRITEREG(XADCPS_SEQ05_OFFSET,
                (InputModeChMask >> XADCPS_SEQ_CH_AUX_SHIFT) &
                XADCPS_SEQ05_CH_VALID_MASK);

    return XST_SUCCESS;
}

static int ADC_SetSeqAvgEnables(u32 AvgEnableChMask)
{
    /*
        * The sequencer must be disabled for writing any of these registers
        * Return XST_FAILURE if the channel sequencer is enabled.
        */
    if ((ADC_GetSequencerMode() != SEQ_MODE_SAFE)) {
        return XST_FAILURE;
    }

    /*
        * Enable/disable the averaging for the specified channels in the
        * ADC Channel Averaging Enables Sequencer Registers.
        */
    ADC_WRITEREG(XADCPS_SEQ02_OFFSET,
                (AvgEnableChMask & XADCPS_SEQ02_CH_VALID_MASK));

    ADC_WRITEREG(XADCPS_SEQ03_OFFSET,
                (AvgEnableChMask >> XADCPS_SEQ_CH_AUX_SHIFT) &
                XADCPS_SEQ03_CH_VALID_MASK);

    return XST_SUCCESS;
}

static int ADC_SetSeqChEnables(u32 ChEnableMask)
{
    /*
        * The sequencer must be disabled for writing any of these registers
        * Return XST_FAILURE if the channel sequencer is enabled.
        */
    if ((ADC_GetSequencerMode() != SEQ_MODE_SAFE)) {
        return XST_FAILURE;
    }

    /*
        * Enable the specified channels in the ADC Channel Selection Sequencer
        * Registers.
        */
    ADC_WRITEREG(XADCPS_SEQ00_OFFSET,
                (ChEnableMask & XADCPS_SEQ00_CH_VALID_MASK));

    ADC_WRITEREG(XADCPS_SEQ01_OFFSET,
                (ChEnableMask >> XADCPS_SEQ_CH_AUX_SHIFT) &
                XADCPS_SEQ01_CH_VALID_MASK);

    return XST_SUCCESS;
}
#endif
