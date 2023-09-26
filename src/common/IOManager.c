/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright $)AB) 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : IOManager.c                                                */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Generic analog and digital input/output manager            */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#ifdef _INFINEON_
#include <xe167f.h>
#include <intrins.h>
#else
#include "core\adc.h"
#endif // _infineon_

#include <string.h>
#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonUtility.h"
#include "common\TaskScheduler.h"
#include "system\SysAppDataCodes.h"

#define DEFINE_EXTERNALS
#include "IOManager.h"
#undef DEFINE_EXTERNALS


//***************************************************************************
#ifdef _INFINEON_
// Macro for ADC management (taken from DAVE)

#define ADC0_vSetLoadEvent()  (ADC0_CRMR1 |= 0x0200)
#define ADC0_uwBusy()  (ADC0_GLOBSTR & 0x0001)
#define ADC0_uwGetResultData(RegNum)  (ADC0_uwGetResultData_##RegNum)
#define ADC0_uwGetResultData_RESULT_REG_0 ((ADC0_RCR0 & 0x1000) ? ((ADC0_RESR0 & 0x0fff) >> 2) : 0)
#define ADC0_uwGetResultData_RESULT_REG_1 ((ADC0_RCR1 & 0x1000) ? ((ADC0_RESR1 & 0x0fff) >> 2) : 0)
#define ADC0_uwGetResultData_RESULT_REG_2 ((ADC0_RCR2 & 0x1000) ? ((ADC0_RESR2 & 0x0fff) >> 2) : 0)
#define ADC0_uwGetResultData_RESULT_REG_3 ((ADC0_RCR3 & 0x1000) ? ((ADC0_RESR3 & 0x0fff) >> 2) : 0)
#define ADC0_uwGetResultData_RESULT_REG_4 ((ADC0_RCR4 & 0x1000) ? ((ADC0_RESR4 & 0x0fff) >> 2) : 0)
#define ADC0_uwGetResultData_RESULT_REG_5 ((ADC0_RCR5 & 0x1000) ? ((ADC0_RESR5 & 0x0fff) >> 2) : 0)
#define ADC0_uwGetResultData_RESULT_REG_6 ((ADC0_RCR6 & 0x1000) ? ((ADC0_RESR6 & 0x0fff) >> 2) : 0)
#define ADC0_uwGetResultData_RESULT_REG_7 ((ADC0_RCR7 & 0x1000) ? ((ADC0_RESR7 & 0x0fff) >> 2) : 0)

#define ADC1_vSetLoadEvent()  (ADC1_CRMR1 |= 0x0200)
#define ADC1_uwBusy()  (ADC1_GLOBSTR & 0x0001)
#define ADC1_uwGetResultData(RegNum)  (ADC1_uwGetResultData_##RegNum)
#define ADC1_uwGetResultData_RESULT_REG_0 ((ADC1_RCR0 & 0x1000) ? ((ADC1_RESR0 & 0x0fff) >> 2) : 0)
#define ADC1_uwGetResultData_RESULT_REG_1 ((ADC1_RCR1 & 0x1000) ? ((ADC1_RESR1 & 0x0fff) >> 2) : 0)
#define ADC1_uwGetResultData_RESULT_REG_2 ((ADC1_RCR2 & 0x1000) ? ((ADC1_RESR2 & 0x0fff) >> 2) : 0)
#define ADC1_uwGetResultData_RESULT_REG_3 ((ADC1_RCR3 & 0x1000) ? ((ADC1_RESR3 & 0x0fff) >> 2) : 0)
#define ADC1_uwGetResultData_RESULT_REG_4 ((ADC1_RCR4 & 0x1000) ? ((ADC1_RESR4 & 0x0fff) >> 2) : 0)
#define ADC1_uwGetResultData_RESULT_REG_5 ((ADC1_RCR5 & 0x1000) ? ((ADC1_RESR5 & 0x0fff) >> 2) : 0)
#define ADC1_uwGetResultData_RESULT_REG_6 ((ADC1_RCR6 & 0x1000) ? ((ADC1_RESR6 & 0x0fff) >> 2) : 0)
#define ADC1_uwGetResultData_RESULT_REG_7 ((ADC1_RCR7 & 0x1000) ? ((ADC1_RESR7 & 0x0fff) >> 2) : 0)   

#define A2D_SCALE__ONE  8192
#define A2D_OFFST_ZERO  32768
#define A2D_RATIO       A2D_OFFST_ZERO / A2D_SCALE__ONE

#else

#ifndef _HW_AXS
#ifdef _HW_DC
#define ADC_MUX_EN_PIN  (MIO_PIN_BASE+49)
#define ADC_MUX_A_PIN   (MIO_PIN_BASE+50)
//#define ADC_MUX_B_PIN   (MIO_PIN_BASE+26)
#define ADC_MUX_B_PIN   (MIO_PIN_BASE+32)
#define ADC_MUX_C_PIN   (MIO_PIN_BASE+44)

#define CHANGE_CHANEL_DELAY   (10) //the delay time when change the ADC input chanel


#define ADC_MUX_INIT    { Gpio_SetMode(ADC_MUX_EN_PIN,GPIO_DIR_OUT); \
                          Gpio_SetMode(ADC_MUX_A_PIN,GPIO_DIR_OUT); \
                          Gpio_SetMode(ADC_MUX_B_PIN,GPIO_DIR_OUT); \
                          Gpio_SetMode(ADC_MUX_C_PIN,GPIO_DIR_OUT); }
#define ADC_MUX_EN      GPIO_OUT(ADC_MUX_EN_PIN,FALSE)
#define ADC_MUX_SET(x)  { GPIO_OUT(ADC_MUX_A_PIN,(((x)&1)==1)); GPIO_OUT(ADC_MUX_B_PIN,(((x)&2)==2)); GPIO_OUT(ADC_MUX_C_PIN,(((x)&4)==4)); }
#endif // _hw_dc
#endif

#endif // _infineon_

//***************************************************************************
// Structures 

#ifndef _HW_DC
typedef struct {
  HWPRMS_AD_SETTINGS  sBridgeTempSensorCalibration ;
} IOMGR_RUN ;

static IOMGR_RUN sIoMgrRun ;
#endif

//***************************************************************************
// Local functions prototypes

#ifndef _APP_XC
static void slowtaskmanagerrevLT102(void);
#endif
static void slowtaskmanagerrevGE102(void);

static void SetupCorrectionFactors(void) ;

#ifndef _HW_DC
static SWORD MeasureCorrection(UWORD uwA2DValue, HWPRMS_AD_SETTINGS sA2DCalibration) ;
#else
static SWORD MeasureCorrection(UWORD uwA2DValue, UWORD uwA2DOffset) ;
#endif
//***************************************************************************
// Init handler
// 1. initialize adc/P8 peripherals and get uwSPowerBoardType
// 2. initialize fpga adc/dac and create management tasks

BOOL IOMgr_Init(UWORD uwSequence)
{
	BOOL bInitOK = FALSE ;

    if(uwSequence==1)
    {
#ifdef _INFINEON_
        // USER CODE BEGIN (ADC0_Init,2)
        
        // USER CODE END
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of ADC0 kernel configuration register:
        ///  -----------------------------------------------------------------------
        ADC0_KSCFG     =  0x0003;      // load ADC0 kernel configuration register
        
        ///  - the ADC module clock is enabled
        ///  - the ADC module clock = 80.00 MHz
        ///   
        
        _nop_();  // one cycle delay 
        
        _nop_();  // one cycle delay 
        
        ///  -----------------------------------------------------------------------
        ///  Configure global control register:
        ///  -----------------------------------------------------------------------
        ///  --- Conversion Timing -----------------
        ///  - conversion time (CTC)    = 01.06 us
        
        ///  _Analog clock is 1/5th of module clock and digital clock is 1/1 times 
        ///  of module clock
        
        ///  - the permanent arbitration mode is selected
        ADC0_GLOBCTR   =  0x0004;      // load global control register
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of Arbitration Slot enable register and also the Source 
        ///  Priority register:
        ///  -----------------------------------------------------------------------
        ///  - Arbitration Slot 0 is disabled
        
        ///  - Arbitration Slot 1 is enabled
        
        ///  - Arbitration Slot 2 is disabled
        
        ///  - the priority of request source 0 is low
        ///  - the wait-for-start mode is selected for source 0
        ///  - the priority of request source 1 is low
        ///  - the wait-for-start mode is selected for source 1
        ///  - the priority of request source 2 is low
        ///  - the wait-for-start mode is selected for source 2
        ADC0_ASENR     =  0x0002;      // load Arbitration Slot enable register
        
        ADC0_RSPR0     =  0x0000;      // load Priority and Arbitration register
        
#ifdef _HW_DC
        ///  Configuration channels
        ADC0_CHCTR0    =  0x0004;
        ADC0_CHCTR1    =  0x1004;
        ADC0_CHCTR2    =  0x2004;
        ADC0_CHCTR3    =  0x3004;
        ADC0_CHCTR4    =  0x4004;
        ADC0_CHCTR5    =  0x5004;
        ADC0_CHCTR6    =  0x6004;
        ADC0_CHCTR7    =  0x7004;
#else // _hw_dc
        ///  -----------------------------------------------------------------------
        ///  Configuration of Channel Control Registers:
        ///  -----------------------------------------------------------------------
        ///  Configuration of Channel 0
        ///  - the result register0 is selected
        ///  - the limit check 0 is selected
        
        ///  - the reference voltage selected is Standard Voltage (Varef)
        
        ///  - the input class selected is Input Class 0
        
        ///  - LCBR0 is selected as upper boundary
        
        ///  - LCBR1 is selected as lower boundary
        
        ADC0_CHCTR0    =  0x0004;      // load channel control register
        
        ///  Configuration of Channel 1
        ///  - the result register1 is selected
        ///  - the limit check 0 is selected
        
        ///  - the reference voltage selected is Standard Voltage (Varef)
        
        ///  - the input class selected is Input Class 0
        
        ///  - LCBR0 is selected as upper boundary
        
        ///  - LCBR1 is selected as lower boundary
        
        ADC0_CHCTR1    =  0x1004;      // load channel control register
        
        ///  Configuration of Channel 3
        ///  - the result register2 is selected
        ///  - the limit check 0 is selected
        
        ///  - the reference voltage selected is Standard Voltage (Varef)
        
        ///  - the input class selected is Input Class 0
        
        ///  - LCBR0 is selected as upper boundary
        
        ///  - LCBR1 is selected as lower boundary
        
        ADC0_CHCTR3    =  0x2004;      // load channel control register
        
        ///  Configuration of Channel 5
        ///  - the result register3 is selected
        ///  - the limit check 0 is selected
        
        ///  - the reference voltage selected is Standard Voltage (Varef)
        
        ///  - the input class selected is Input Class 0
        
        ///  - LCBR0 is selected as upper boundary
        
        ///  - LCBR1 is selected as lower boundary
        
        ADC0_CHCTR5    =  0x3004;      // load channel control register
        
        ///  Configuration of Channel 6
        ///  - the result register4 is selected
        ///  - the limit check 0 is selected
        
        ///  - the reference voltage selected is Standard Voltage (Varef)
        
        ///  - the input class selected is Input Class 0
        
        ///  - LCBR0 is selected as upper boundary
        
        ///  - LCBR1 is selected as lower boundary
        
        ADC0_CHCTR6    =  0x4004;      // load channel control register
        
        ///  Configuration of Channel 7
        ///  - the result register5 is selected
        ///  - the limit check 0 is selected
        
        ///  - the reference voltage selected is Standard Voltage (Varef)
        
        ///  - the input class selected is Input Class 0
        
        ///  - LCBR0 is selected as upper boundary
        
        ///  - LCBR1 is selected as lower boundary
        
        ADC0_CHCTR7    =  0x5004;      // load channel control register

 #ifndef _APP_XC
           // if controlboard is 1.00 or 1.01
        if(sGlbControlBoardParameters.sProductInfo.uwProductRev<102)
        {
            ///  Configuration of Channel 12
            ///  - the result register0 is selected
            ///  - the limit check 0 is selected
            
            ///  - the reference voltage selected is Standard Voltage (Varef)
            
            ///  - the input class selected is Input Class 0
            
            ///  - LCBR0 is selected as upper boundary
            
            ///  - LCBR1 is selected as lower boundary
            
            ADC0_CHCTR12   =  0x0004;      // load channel control register
            
            ///  Configuration of Channel 13
            ///  - the result register1 is selected
            ///  - the limit check 0 is selected
            
            ///  - the reference voltage selected is Standard Voltage (Varef)
            
            ///  - the input class selected is Input Class 0
            
            ///  - LCBR0 is selected as upper boundary
            
            ///  - LCBR1 is selected as lower boundary
            
            ADC0_CHCTR13   =  0x1004;      // load channel control register
            
            ///  Configuration of Channel 14
            ///  - the result register2 is selected
            ///  - the limit check 0 is selected
            
            ///  - the reference voltage selected is Standard Voltage (Varef)
            
            ///  - the input class selected is Input Class 0
            
            ///  - LCBR0 is selected as upper boundary
            
            ///  - LCBR1 is selected as lower boundary
            
            ADC0_CHCTR14   =  0x2004;      // load channel control register
            
            ///  Configuration of Channel 15
            ///  - the result register3 is selected
            ///  - the limit check 0 is selected
            
            ///  - the reference voltage selected is Standard Voltage (Varef)
            
            ///  - the input class selected is Input Class 0
            
            ///  - LCBR0 is selected as upper boundary
            
            ///  - LCBR1 is selected as lower boundary
            
            ADC0_CHCTR15   =  0x3004;      // load channel control register
        }
#endif // _app_xc
#endif // _hw_dc
        ///  -----------------------------------------------------------------------
        ///  Configuration of Sample Time and Resolution:
        ///  -----------------------------------------------------------------------
        
        ///  10 bit resolution selected
        
        ADC0_INPCR0    =  0x0000;      // load input class0 register
        
        ///  10 bit resolution selected
        
        ADC0_INPCR1    =  0x0000;      // load input class1 register
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of Result Control Registers:
        ///  -----------------------------------------------------------------------
        ///  Configuration of Result Control Register 0
        ///  - the data reduction filter is disabled
        ///  - the event interrupt is disabled
        ///  - the wait-for-read mode is disabled
        
        ///  - the FIFO functionality is disabled
        
        ADC0_RCR0      =  0x0000;      // load result control register 0
        
        ///  Configuration of Result Control Register 1
        ///  - the data reduction filter is disabled
        ///  - the event interrupt is disabled
        ///  - the wait-for-read mode is disabled
        
        ///  - the FIFO functionality is disabled
        
        ADC0_RCR1      =  0x0000;      // load result control register 1
        
        ///  Configuration of Result Control Register 2
        ///  - the data reduction filter is disabled
        ///  - the event interrupt is disabled
        ///  - the wait-for-read mode is disabled
        
        ///  - the FIFO functionality is disabled
        
        ADC0_RCR2      =  0x0000;      // load result control register 2
        
        ///  Configuration of Result Control Register 3
        ///  - the data reduction filter is disabled
        ///  - the event interrupt is disabled
        ///  - the wait-for-read mode is disabled
        
        ///  - the FIFO functionality is disabled
        
        ADC0_RCR3      =  0x0000;      // load result control register 3
        
        ///  Configuration of Result Control Register 4
        ///  - the data reduction filter is disabled
        ///  - the event interrupt is disabled
        ///  - the wait-for-read mode is disabled
        
        ///  - the FIFO functionality is disabled
        
        ADC0_RCR4      =  0x0000;      // load result control register 4
        
        ///  Configuration of Result Control Register 5
        ///  - the data reduction filter is disabled
        ///  - the event interrupt is disabled
        ///  - the wait-for-read mode is disabled
        
        ///  - the FIFO functionality is disabled
        
        ADC0_RCR5      =  0x0000;      // load result control register 5
        
        ///  Configuration of Result Control Register 6
        ///  - the data reduction filter is disabled
        ///  - the event interrupt is disabled
        ///  - the wait-for-read mode is disabled
        
        ///  - the FIFO functionality is disabled
        
        ADC0_RCR6      =  0x0000;      // load result control register 6
        
        ///  Configuration of Result Control Register 7
        ///  - the data reduction filter is disabled
        ///  - the event interrupt is disabled
        ///  - the wait-for-read mode is disabled
        
        ///  - the FIFO functionality is disabled
        
        ADC0_RCR7      =  0x0000;      // load result control register 7
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of Channel Interrupt Node Pointer Register:
        ///  -----------------------------------------------------------------------
        ///  - the SR0 line become activated if channel 0 interrupt is generated
        
        ///  - the SR0 line become activated if channel 1 interrupt is generated
        
        ///  - the SR0 line become activated if channel 3 interrupt is generated
        
        ADC0_CHINPR0   =  0x0000;      // load channel interrupt node pointer 
                                     // register
        
        ///  - the SR0 line become activated if channel 5 interrupt is generated
        
        ///  - the SR0 line become activated if channel 6 interrupt is generated
        
        ///  - the SR0 line become activated if channel 7 interrupt is generated
        
        ADC0_CHINPR4   =  0x0000;      // load channel interrupt node pointer 
                                     // register
        
        ADC0_CHINPR8   =  0x0000;      // load channel interrupt node pointer 
                                     // register
        
        ///  - the SR0 line become activated if channel 12 interrupt is generated
        
        ///  - the SR0 line become activated if channel 13 interrupt is generated
        
        ///  - the SR0 line become activated if channel 14 interrupt is generated
        
        ///  - the SR0 line become activated if channel 15 interrupt is generated
        
        ADC0_CHINPR12  =  0x0000;      // load channel interrupt node pointer 
                                     // register
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of Event Interrupt Node Pointer Register for Source 
        ///  Interrupts:
        ///  -----------------------------------------------------------------------
        ///  - the SR 0 line become activated if the event 1 interrupt is generated
        
        ADC0_EVINPR0   =  0x0000;      // load event interrupt set flag register 
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of Event Interrupt Node Pointer Register for Result 
        ///  Interrupts:
        ///  -----------------------------------------------------------------------
        
        ADC0_EVINPR8   =  0x0000;      // load event interrupt set flag register 
        
        
        ADC0_EVINPR12  =  0x0000;      // load event interrupt set flag register 
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of Service Request Nodes 0 - 3 :
        ///  -----------------------------------------------------------------------
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of Limit Check Boundary:
        ///  -----------------------------------------------------------------------
        
        ADC0_LCBR0     =  0x0198;      // load limit check boundary register 0
        
        ADC0_LCBR1     =  0x0E64;      // load limit check boundary register 1
        
        ADC0_LCBR2     =  0x0554;      // load limit check boundary register 2
        
        ADC0_LCBR3     =  0x0AA8;      // load limit check boundary register 3
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of Gating source and External Trigger Control:
        ///  -----------------------------------------------------------------------
        ///  - No Gating source selected for Arbitration Source 0
        
        ///  - the trigger input ETR00 is selected for Source 0
        
        ///  - No Gating source selected for Arbitration Source 1
        
        ///  - the trigger input ETR00 is selected for Source 1
        
        ///  - No Gating source selected for Arbitration Source 2
        
        ///  - the trigger input ETR00 is selected for Source 1
        
#ifndef _APP_XC
        if( XFAMILY_GETID() == XFAMILYIDENT_XE167x )
            ADC0_PISEL =  0x0444;      // load external trigger control register
#endif // _app_xc
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of Conversion Queue Mode Register:Sequential Source 0
        ///  -----------------------------------------------------------------------
        ///  - the gating line is permanently Disabled
        ///  - the external trigger is disabled
        ///  - the trigger mode 0 is selected
        
        ADC0_QMR0      =  0x0000;      // load queue mode register
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of Conversion Queue Mode Register:Sequential Source 2
        ///  -----------------------------------------------------------------------
        ///  - the gating line is permanently Disabled
        ///  - the external trigger is disabled
        ///  - the trigger mode 0 is selected
        
        ADC0_QMR2      =  0x0000;      // load queue mode register
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of Conversion Request Mode Registers:Parallel Source 
        ///  -----------------------------------------------------------------------
        ///  - the gating line is permanently Enabled
        ///  - the external trigger is disabled
        ///  - the source interrupt is disabled
        ///  - the autoscan functionality is disabled
        
        ADC0_CRMR1     =  0x0001;      // load conversion request mode register 1
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of Synchronisation Registers:
        ///  -----------------------------------------------------------------------
        ///  - ADC0 is master 
        ADC0_SYNCTR   |=  0x0010;      // Synchronisation register
        
#ifndef _HW_DC
#ifndef _APP_XC
            // if controlboard is 1.00 or 1.01
        if(sGlbControlBoardParameters.sProductInfo.uwProductRev<102)
            P5_DIDIS       =  0xF0EB;      // Port 5 Digital input disable register
        else
#endif // _app_xc
#ifndef _HW_CT
            P5_DIDIS       =  0x00EB;      // Port 5 Digital input disable register
#else // _hw_ct
            P5_DIDIS       =  0xE0EB;      // Port 5 Digital input disable register
#endif // _hw_ct
#else //_hw_dc
        P5_DIDIS       =  0x00FF;      // Port 5 Digital input disable register
#endif // _hw_dc
        
        ADC0_GLOBCTR  |=  0x0300;      // turn on Analog part

#ifndef _APP_XC
        if(sGlbControlBoardParameters.sProductInfo.uwProductRev>=102)
#endif // _app_xc
        {
#ifndef _HW_DC
              // ADC0 : AN07, AN06, AN05, AN03, AN01, AN00
              ADC0_CRCR1 = 0x00EB;
#else // _hw_dc
              // ADC0 : AN07, AN06, AN05, AN04, AN03, AN02, AN01, AN00
              ADC0_CRCR1 = 0x00FF;
#endif // _hw_dc
            
              ///  -----------------------------------------------------------------------
              ///  Configure global control register:
              ///  -----------------------------------------------------------------------
              ///  --- Conversion Timing -----------------
              ///  - conversion time (CTC)    = 01.06 us
            
              ///  _Analog clock is 1/5th of module clock and digital clock is 1/1 times 
              ///  of module clock
            
              ///  - the permanent arbitration mode is selected
              ADC1_GLOBCTR   =  0x0004;      // load global control register
            
              ///  -----------------------------------------------------------------------
              ///  Configuration of Arbitration Slot enable register and also the Source 
              ///  Priority register:
              ///  -----------------------------------------------------------------------
              ///  - Arbitration Slot 0 is disabled
            
              ///  - Arbitration Slot 1 is enabled
            
              ///  - Arbitration Slot 2 is disabled
            
              ///  - the priority of request source 0 is low
              ///  - the wait-for-start mode is selected for source 0
              ///  - the priority of request source 1 is low
              ///  - the wait-for-start mode is selected for source 1
              ///  - the priority of request source 2 is low
              ///  - the wait-for-start mode is selected for source 2
              ADC1_ASENR     =  0x0002;      // load Arbitration Slot enable register
            
              ADC1_RSPR0     =  0x0000;      // load Priority and Arbitration register
            

#ifdef _HW_DC
              ///  Configuration channels
              ADC1_CHCTR0    =  0x0004;
              ADC1_CHCTR1    =  0x1004;
              ADC1_CHCTR2    =  0x2004;
              ADC1_CHCTR3    =  0x3004;
              ADC1_CHCTR4    =  0x4004;
              ADC1_CHCTR5    =  0x5004;
              ADC1_CHCTR6    =  0x6004;
              ADC1_CHCTR7    =  0x7004;
#else // _hw_dc
              ///  -----------------------------------------------------------------------
              ///  Configuration of Channel Control Registers:
              ///  -----------------------------------------------------------------------
              ///  Configuration of Channel 4
              ///  - the result register0 is selected
              ///  - the limit check 0 is selected
            
              ///  - the reference voltage selected is Standard Voltage (Varef)
            
              ///  - the input class selected is Input Class 0
            
              ///  - LCBR0 is selected as upper boundary
            
              ///  - LCBR1 is selected as lower boundary
            
              ADC1_CHCTR4    =  0x0004;      // load channel control register
            
              ///  Configuration of Channel 5
              ///  - the result register1 is selected
              ///  - the limit check 0 is selected
            
              ///  - the reference voltage selected is Standard Voltage (Varef)
            
              ///  - the input class selected is Input Class 0
            
              ///  - LCBR0 is selected as upper boundary
            
              ///  - LCBR1 is selected as lower boundary
            
              ADC1_CHCTR5    =  0x1004;      // load channel control register
            
              ///  Configuration of Channel 6
              ///  - the result register2 is selected
              ///  - the limit check 0 is selected
            
              ///  - the reference voltage selected is Standard Voltage (Varef)
            
              ///  - the input class selected is Input Class 0
            
              ///  - LCBR0 is selected as upper boundary
            
              ///  - LCBR1 is selected as lower boundary
            
              ADC1_CHCTR6    =  0x2004;      // load channel control register
            
              ///  Configuration of Channel 7
              ///  - the result register3 is selected
              ///  - the limit check 0 is selected
            
              ///  - the reference voltage selected is Standard Voltage (Varef)
            
              ///  - the input class selected is Input Class 0
            
              ///  - LCBR0 is selected as upper boundary
            
              ///  - LCBR1 is selected as lower boundary
            
              ADC1_CHCTR7    =  0x3004;      // load channel control register
#endif // _hw_dc
              ///  -----------------------------------------------------------------------
              ///  Configuration of Sample Time and Resolution:
              ///  -----------------------------------------------------------------------
            
              ///  10 bit resolution selected
            
              ADC1_INPCR0    =  0x0000;      // load input class0 register
            
              ///  10 bit resolution selected
            
              ADC1_INPCR1    =  0x0000;      // load input class1 register
            
              ///  -----------------------------------------------------------------------
              ///  Configuration of Result Control Registers:
              ///  -----------------------------------------------------------------------
              ///  Configuration of Result Control Register 0
              ///  - the data reduction filter is disabled
              ///  - the event interrupt is disabled
              ///  - the wait-for-read mode is disabled
            
              ///  - the FIFO functionality is disabled
            
              ADC1_RCR0      =  0x0000;      // load result control register 0
            
              ///  Configuration of Result Control Register 1
              ///  - the data reduction filter is disabled
              ///  - the event interrupt is disabled
              ///  - the wait-for-read mode is disabled
            
              ///  - the FIFO functionality is disabled
            
              ADC1_RCR1      =  0x0000;      // load result control register 1
            
              ///  Configuration of Result Control Register 2
              ///  - the data reduction filter is disabled
              ///  - the event interrupt is disabled
              ///  - the wait-for-read mode is disabled
            
              ///  - the FIFO functionality is disabled
            
              ADC1_RCR2      =  0x0000;      // load result control register 2
            
              ///  Configuration of Result Control Register 3
              ///  - the data reduction filter is disabled
              ///  - the event interrupt is disabled
              ///  - the wait-for-read mode is disabled
            
              ///  - the FIFO functionality is disabled
            
              ADC1_RCR3      =  0x0000;      // load result control register 3
            
              ///  Configuration of Result Control Register 4
              ///  - the data reduction filter is disabled
              ///  - the event interrupt is disabled
              ///  - the wait-for-read mode is disabled
            
              ///  - the FIFO functionality is disabled
            
              ADC1_RCR4      =  0x0000;      // load result control register 4
            
              ///  Configuration of Result Control Register 5
              ///  - the data reduction filter is disabled
              ///  - the event interrupt is disabled
              ///  - the wait-for-read mode is disabled
            
              ///  - the FIFO functionality is disabled
            
              ADC1_RCR5      =  0x0000;      // load result control register 5
            
              ///  Configuration of Result Control Register 6
              ///  - the data reduction filter is disabled
              ///  - the event interrupt is disabled
              ///  - the wait-for-read mode is disabled
            
              ///  - the FIFO functionality is disabled
            
              ADC1_RCR6      =  0x0000;      // load result control register 6
            
              ///  Configuration of Result Control Register 7
              ///  - the data reduction filter is disabled
              ///  - the event interrupt is disabled
              ///  - the wait-for-read mode is disabled
            
              ///  - the FIFO functionality is disabled
            
              ADC1_RCR7      =  0x0000;      // load result control register 7
            
              ///  -----------------------------------------------------------------------
              ///  Configuration of Channel Interrupt Node Pointer Register:
              ///  -----------------------------------------------------------------------
              ADC1_CHINPR0   =  0x0000;      // load channel interrupt node pointer 
                                             // register
            
              ///  - the SR0 line become activated if channel 4 interrupt is generated
            
              ///  - the SR0 line become activated if channel 5 interrupt is generated
            
              ///  - the SR0 line become activated if channel 6 interrupt is generated
            
              ///  - the SR0 line become activated if channel 7 interrupt is generated
            
              ADC1_CHINPR4   =  0x0000;      // load channel interrupt node pointer 
                                             // register
            
              ///  -----------------------------------------------------------------------
              ///  Configuration of Event Interrupt Node Pointer Register for Source 
              ///  Interrupts:
              ///  -----------------------------------------------------------------------
              ///  - the SR 0 line become activated if the event 1 interrupt is generated
            
              ADC1_EVINPR0   =  0x0000;      // load event interrupt set flag register 
            
              ///  -----------------------------------------------------------------------
              ///  Configuration of Event Interrupt Node Pointer Register for Result 
              ///  Interrupts:
              ///  -----------------------------------------------------------------------
            
              ADC1_EVINPR8   =  0x0000;      // load event interrupt set flag register 
            
            
              ADC1_EVINPR12  =  0x0000;      // load event interrupt set flag register 
            
              ///  -----------------------------------------------------------------------
              ///  Configuration of Service Request Nodes 0 - 3 :
              ///  -----------------------------------------------------------------------
            
              ///  -----------------------------------------------------------------------
              ///  Configuration of Limit Check Boundary:
              ///  -----------------------------------------------------------------------
            
              ADC1_LCBR0     =  0x0198;      // load limit check boundary register 0
            
              ADC1_LCBR1     =  0x0E64;      // load limit check boundary register 1
            
              ADC1_LCBR2     =  0x0554;      // load limit check boundary register 2
            
              ADC1_LCBR3     =  0x0AA8;      // load limit check boundary register 3
            
              ///  -----------------------------------------------------------------------
              ///  Configuration of Gating source and External Trigger Control:
              ///  -----------------------------------------------------------------------
              ///  - No Gating source selected for Arbitration Source 0
            
              ///  - the trigger input ETR00 is selected for Source 0
            
              ///  - No Gating source selected for Arbitration Source 1
            
              ///  - the trigger input ETR00 is selected for Source 1
            
              ///  - No Gating source selected for Arbitration Source 2
            
              ///  - the trigger input ETR00 is selected for Source 1
            
#ifndef _APP_XC
              if( XFAMILY_GETID() == XFAMILYIDENT_XE167x )
                  ADC1_PISEL =  0x0444;      // load external trigger control register
#endif // _app_xc
            
              ///  -----------------------------------------------------------------------
              ///  Configuration of Conversion Queue Mode Register:Sequential Source 0
              ///  -----------------------------------------------------------------------
              ///  - the gating line is permanently Disabled
              ///  - the external trigger is disabled
              ///  - the trigger mode 0 is selected
            
              ADC1_QMR0      =  0x0000;      // load queue mode register
            
              ///  -----------------------------------------------------------------------
              ///  Configuration of Conversion Queue Mode Register:Sequential Source 2
              ///  -----------------------------------------------------------------------
              ///  - the gating line is permanently Disabled
              ///  - the external trigger is disabled
              ///  - the trigger mode 0 is selected
            
              ADC1_QMR2      =  0x0000;      // load queue mode register
            
              ///  -----------------------------------------------------------------------
              ///  Configuration of Conversion Request Mode Registers:Parallel Source 
              ///  -----------------------------------------------------------------------
              ///  - the gating line is permanently Enabled
              ///  - the external trigger is disabled
              ///  - the source interrupt is disabled
              ///  - the autoscan functionality is disabled
            
              ADC1_CRMR1     =  0x0001;      // load conversion request mode register 1
            
              ///  -----------------------------------------------------------------------
              ///  Configuration of Synchronisation Registers:
              ///  -----------------------------------------------------------------------
              ///  - ADC1 is master 
              ADC1_SYNCTR   |=  0x0010;      // Synchronisation register

#ifndef _HW_DC            
              P15_DIDIS      =  0x00F0;      // Port 15 Digital input disable register
#else // _hw_dc
              P15_DIDIS      =  0x00FF;      // Port 15 Digital input disable register
#endif // _hw_dc
            
              ADC1_GLOBCTR  |=  0x0300;      // turn on Analog part
            
#ifndef _HW_DC            
              // ADC1 : AN07, AN06, AN05, AN04
              ADC1_CRCR1 = 0x00F0;
#else // _hw_dc
              // ADC1 : AN07, AN06, AN05, AN04, AN03, AN02, AN01, AN00
              ADC1_CRCR1 = 0x00FF;
#endif // _hw_dc
        }

        //**************************************************************************
    
            // clear data structures
        xmemset(&sIOMgrAnalogMeasurements, 0, sizeof(sIOMgrAnalogMeasurements));
    
#ifndef _APP_XC
            // Immediately get board type for subsequent initialization procedures
        if(sGlbControlBoardParameters.sProductInfo.uwProductRev<102)
            ADC0_CRCR1 = 0x0001;
#endif // _hw_dc
          
            // Set the Load Event bit
        ADC0_vSetLoadEvent();
        
            // wait 50us
        timer_wait(uwSysTimers100ns, 500);
        
        // Wait for ADC to complete the conversion
        while ( ADC0_uwBusy() || ADC0_CRPR1 ) ;
#ifndef _HW_DC
        sIOMgrAnalogMeasurements.uwSPowerBoardType = ADC0_uwGetResultData( RESULT_REG_0 );
#endif // hw_dc

#else // _infineon_

            // clear data structures
        memset(&sIOMgrAnalogMeasurements, 0, sizeof(sIOMgrAnalogMeasurements));
        
            // xadc initial
        if(!Adc_Init())
            bInitOK = TRUE ;

#ifndef _HW_AXS
#ifdef _HW_DC
        ADC_MUX_INIT;
        ADC_MUX_EN;
#endif // _hw_dc
#endif

#endif // _infineon_
        //**************************************************************************
    }
    else if(uwSequence==2)
    {
#ifndef _APP_XC
        if(sGlbControlBoardParameters.sProductInfo.uwProductRev<102)
            TaskSched_AddBackgroundTask(&slowtaskmanagerrevLT102);
        else
#endif // _app_xc
        {
#ifdef _INFINEON_
            ADC0_vSetLoadEvent();
            ADC1_vSetLoadEvent();
                // wait 50us to perform first complete conversion
            timer_wait(uwSysTimers100ns, 500);
#endif // _infineon_

            TaskSched_AddBackgroundTask(&slowtaskmanagerrevGE102);
            bInitOK = TRUE ;
        }

    }

    SetupCorrectionFactors() ;

    return bInitOK ;
}    


//***************************************************************************
// slow task for analog measurements sampling (board <1.02)
// $TEMP$ convertire qui i valori acquisiti in tensioni e temperature

// #ifndef _APP_XC
// static void slowtaskmanagerrevLT102(void)
// {
//     UWORD uwSBridgeTemp ;
//     static UBYTE convstat=0;

//         // if ADC busy, do nothing
//     if(ADC0_uwBusy() || ADC0_CRPR1)
//         return;

//     if(convstat==1)
//     {
//         sIOMgrAnalogMeasurements.uwS24Vaux  = ADC0_uwGetResultData( RESULT_REG_0 ); /* AN12 : XE167_AD_24VAUX */
//         sIOMgrAnalogMeasurements.uwS15Vinp  = ADC0_uwGetResultData( RESULT_REG_1 ); /* AN01 : XE167_AD_15V    */
//         sIOMgrAnalogMeasurements.uwSVENCout = ADC0_uwGetResultData( RESULT_REG_2 ); /* AN03 : XE167_AD_VENC   */
//         sIOMgrAnalogMeasurements.uwS3V3out  = ADC0_uwGetResultData( RESULT_REG_3 ); /* AN05 : XE167_AD_3.3V   */
//         sIOMgrAnalogMeasurements.uwS2V5out  = ADC0_uwGetResultData( RESULT_REG_4 ); /* AN06 : XE167_AD_2.5V   */
//         sIOMgrAnalogMeasurements.uwS1V2out  = ADC0_uwGetResultData( RESULT_REG_5 ); /* AN07 : XE167_AD_1.2V   */

//         /* AN15, AN14, AN13, AN00 */
//         ADC0_CRCR1 = 0xE001;
//         /* Set the Load Event bit */
//         ADC0_vSetLoadEvent();

//         convstat=2;
//     } 
//     else if(convstat==2)
//     { /* Read results */
//         sIOMgrAnalogMeasurements.uwSPowerBoardType  = ADC0_uwGetResultData( RESULT_REG_0 ); /* AN00 : XE167_AD_BOARD_TYPE  */
//         sIOMgrAnalogMeasurements.uwSBoardTemp       = ADC0_uwGetResultData( RESULT_REG_1 ); /* AN13 : XE167_AD_BOARD_TEMP  */
// //        sIOMgrAnalogMeasurements.uwSBridgeTemp      = ADC0_uwGetResultData( RESULT_REG_2 ); /* AN14 : XE167_AD_BRIDGE_TEMP */
//         uwSBridgeTemp                               = ADC0_uwGetResultData( RESULT_REG_2 ); /* AN14 : XE167_AD_BRIDGE_TEMP */
//         sIOMgrAnalogMeasurements.uwSMotorTemp       = ADC0_uwGetResultData( RESULT_REG_3 ); /* AN15 : XE167_AD_MOTOR_TEMP  */

//         /* apply correction scale and offset */
//         sIOMgrAnalogMeasurements.uwSBridgeTemp = (UWORD)MeasureCorrection(uwSBridgeTemp, sIoMgrRun.sBridgeTempSensorCalibration) ;
        
//         sIOMgrAnalogMeasurements.swAllValid         = TRUE;

//         convstat=1;
//     }

//         // if first time just start sampling
//     if(convstat==0 || convstat==1)
//     {
//         /* AN12, AN07, AN06, AN05, AN03, AN01 */
//         ADC0_CRCR1 = 0x10EA;
//         /* Set the Load Event bit */
//         ADC0_vSetLoadEvent();

//         convstat=1;
//     }
// }
// #endif // _app_xc

//***************************************************************************
// slow task for analog measurements sampling (board >=1.02)
// $TEMP$ convertire qui i valori acquisiti in tensioni e temperature

static void slowtaskmanagerrevGE102(void)
{
#ifdef _INFINEON_

        // if ADC units busy, do nothing
    if(ADC0_uwBusy() || ADC0_CRPR1 || ADC1_uwBusy() || ADC1_CRPR1)
        return;

#ifndef _HW_DC
    sIOMgrAnalogMeasurements.uwSPowerBoardType  = ADC0_uwGetResultData( RESULT_REG_0 ) ; // ADC0.AN00: Power Board Type
    sIOMgrAnalogMeasurements.uwS15Vinp          = ADC0_uwGetResultData( RESULT_REG_1 ) ; // ADC0.AN01: 15V
    sIOMgrAnalogMeasurements.uwSVENCout         = ADC0_uwGetResultData( RESULT_REG_2 ) ; // ADC0.AN03: V Encoder
    sIOMgrAnalogMeasurements.uwS3V3out          = ADC0_uwGetResultData( RESULT_REG_3 ) ; // ADC0.AN05: 3.3V
    sIOMgrAnalogMeasurements.uwS2V5out          = ADC0_uwGetResultData( RESULT_REG_4 ) ; // ADC0.AN06: 2.5V
    sIOMgrAnalogMeasurements.uwS1V2out          = ADC0_uwGetResultData( RESULT_REG_5 ) ; // ADC0.AN07: 1.2V
    sIOMgrAnalogMeasurements.uwS24Vaux          = ADC1_uwGetResultData( RESULT_REG_0 ) ; // ADC1.AN04: 24V Aux
    sIOMgrAnalogMeasurements.uwSBoardTemp       = ADC1_uwGetResultData( RESULT_REG_1 ) ; // ADC1.AN05: Control Board NTC
    sIOMgrAnalogMeasurements.uwSBridgeTemp      = MeasureCorrection(ADC1_uwGetResultData( RESULT_REG_2 ),
                                                                    sIoMgrRun.sBridgeTempSensorCalibration ) ; // ADC1.AN06: Power Board NTC
    sIOMgrAnalogMeasurements.uwSMotorTemp       = ADC1_uwGetResultData( RESULT_REG_3 ) ; // ADC1.AN07: Motor KTY
#else // _hw_dc
    sIOMgrAnalogMeasurements.uwS24Vaux          = ADC0_uwGetResultData( RESULT_REG_4 ) ; // ADC0.AN04: 24V Aux
    sIOMgrAnalogMeasurements.uwSBoardTemp       = ADC1_uwGetResultData( RESULT_REG_5 ) ; // ADC1.AN05: Control Board NTC
    sIOMgrAnalogMeasurements.uwSBrgTemp[0]      = MeasureCorrection(ADC1_uwGetResultData( RESULT_REG_0 ),
                                                                    sGlbPowerBoardParameters.sUniv.sTempSens[0].uwOffst ) ; // ADC1.AN00: Bridge #0 NTC																	
    sIOMgrAnalogMeasurements.uwSBrgTemp[1]      = MeasureCorrection(ADC1_uwGetResultData( RESULT_REG_1 ),
                                                                    sGlbPowerBoardParameters.sUniv.sTempSens[1].uwOffst ) ; // ADC1.AN01: Bridge #1 NTC
    sIOMgrAnalogMeasurements.uwSBrgTemp[2]      = MeasureCorrection(ADC1_uwGetResultData( RESULT_REG_2 ),
                                                                    sGlbPowerBoardParameters.sUniv.sTempSens[2].uwOffst ) ; // ADC1.AN02: Bridge #2 NTC
    sIOMgrAnalogMeasurements.uwSMotorPTCTemp    = ADC1_uwGetResultData( RESULT_REG_6 ) ; // ADC1.AN06: Motor PTC
    sIOMgrAnalogMeasurements.uwSMotorKTYTemp    = ADC1_uwGetResultData( RESULT_REG_7 ) ; // ADC1.AN07: Motor KTY
    sIOMgrAnalogMeasurements.uwSVEMout          = ADC0_uwGetResultData( RESULT_REG_5 ) ; // ADC0.AN05: V Encoder Main
    sIOMgrAnalogMeasurements.uwSVEAout          = ADC0_uwGetResultData( RESULT_REG_6 ) ; // ADC0.AN06: V Encoder Aux
#endif // _hw_dc
    
    sIOMgrAnalogMeasurements.swAllValid         = TRUE;
    
    ADC0_vSetLoadEvent() ; // Set ADC0 Load Event bit
    ADC1_vSetLoadEvent() ; // Set ADC1 Load Event bit
#else // _infineon_

    sIOMgrAnalogMeasurements.fSVccPint    = Adc_GetVccPint();
    sIOMgrAnalogMeasurements.fSVccPaux    = Adc_GetVccPaux();
    sIOMgrAnalogMeasurements.fSVccPdro    = Adc_GetVccPdro();
    sIOMgrAnalogMeasurements.fSOnchipTemp = Adc_GetOnChipTemperature();

#ifdef _HW_DC

#ifndef _HW_AXS

                                               ADC_MUX_SET(4); timer_wait(uwSysTimers125us,CHANGE_CHANEL_DELAY);
    sIOMgrAnalogMeasurements.uwSBoardTemp    = Adc_GetVPVNData(); // MUX4: Board temperature: Vin = Vbt * 10K/(10K+20K)

    sIOMgrAnalogMeasurements.uwSBrgTemp[0]   = Adc_GetAuxData(2);
    sIOMgrAnalogMeasurements.uwSBrgTemp[1]   = Adc_GetAuxData(3);
    sIOMgrAnalogMeasurements.uwSBrgTemp[2]   = Adc_GetAuxData(8);

                                               ADC_MUX_SET(5); timer_wait(uwSysTimers125us,CHANGE_CHANEL_DELAY);
    sIOMgrAnalogMeasurements.uwSMotorPTCTemp = (UWORD)(Adc_GetVPVNData()); // (UWORD)(Adc_GetVPVNData()*3/3) MUX5: Motor PTC: Vin = Vptc * 10K/(10K+20K)

                                               ADC_MUX_SET(6); timer_wait(uwSysTimers125us,CHANGE_CHANEL_DELAY);
    sIOMgrAnalogMeasurements.uwSMotorKTYTemp = (UWORD)(Adc_GetVPVNData()); // MUX6: Motor KTY: Vin = Vkty * 10K/(10K+20K)

                                               ADC_MUX_SET(1); timer_wait(uwSysTimers125us,CHANGE_CHANEL_DELAY);
    sIOMgrAnalogMeasurements.uwSVEMout       = (UWORD)(((ULONG)Adc_GetVPVNData() * 5*13*80)>>16); // MUX1: V Encoder Main: Vin = +VMENC * 8.2K/(33K+8.2K) * 1K/(1K + 12K)
                                               ADC_MUX_SET(2); timer_wait(uwSysTimers125us,CHANGE_CHANEL_DELAY);
    sIOMgrAnalogMeasurements.uwSVEAout       = (UWORD)(((ULONG)Adc_GetVPVNData() * 5*13*80)>>16); // MUX2: V Encoder Aux: Vin = +VAENC * 8.2K/(33K+8.2K) * 1K/(1K + 12K)


#else // _HW_AXS

#if defined(_HW_AXS_SAIETTA)
    sIOMgrAnalogMeasurements.uwSBoardTemp    = Adc_GetAuxData(2);	// Temp_1

    sIOMgrAnalogMeasurements.uwSBrgTemp[0]   = Adc_GetAuxData(0);	// Temp_IGBT_A
    sIOMgrAnalogMeasurements.uwSBrgTemp[1]   = Adc_GetAuxData(8);	// Temp_IGBT_B
    sIOMgrAnalogMeasurements.uwSBrgTemp[2]   = Adc_GetAuxData(1);	// Temp_IGBT_C

    sIOMgrAnalogMeasurements.uwSMotorKTYTemp = 0; // PT1000 : unconnected on PCB

    sIOMgrAnalogMeasurements.uwSMotorPTCTemp = 0; // PTC : unconnected on PCB

#elif defined(_HW_AXS_CABI35KW)
    sIOMgrAnalogMeasurements.uwSBoardTemp    = Adc_GetAuxData(0); // 65535 = Max Value = 1V

    sIOMgrAnalogMeasurements.uwSBrgTemp[0]   = Adc_GetAuxData(8);
    sIOMgrAnalogMeasurements.uwSBrgTemp[1]   = 0;
    sIOMgrAnalogMeasurements.uwSBrgTemp[2]   = 0;

    sIOMgrAnalogMeasurements.uwSMotorKTYTemp = (UWORD)(Adc_GetAuxData(1) * 4.921/3); // Motor KTY: Vin = Vkty * 5.1K/(5.1K+20K)

    sIOMgrAnalogMeasurements.uwSMotorPTCTemp = 0;

//    sIOMgrAnalogMeasurements.uwS24Vaux     = (UWORD)(((ULONG)Adc_GetAuxData(9) * 37*10)>>16); // Max Value: 65536 = 37V * 10 = 370 1e-1 V, Vin = 24V * 10K/(10K + 360K)

#elif defined(_HW_AXS_DAYCO22KW)
    sIOMgrAnalogMeasurements.uwSBoardTemp    = Adc_GetAuxData(0);

    sIOMgrAnalogMeasurements.uwSBrgTemp[0]   = Adc_GetAuxData(8);
    sIOMgrAnalogMeasurements.uwSBrgTemp[1]   = Adc_GetAuxData(9);
    sIOMgrAnalogMeasurements.uwSBrgTemp[2]   = 0;

    sIOMgrAnalogMeasurements.uwSMotorKTYTemp = (UWORD)(Adc_GetAuxData(10) * 1.0);
    sIOMgrAnalogMeasurements.uwSMotorPTCTemp = (UWORD)(Adc_GetAuxData(6)  * 1.0);

#endif

#endif // _HW_AXS


    sIOMgrAnalogMeasurements.swAllValid      = TRUE;

#else
    sIOMgrAnalogMeasurements.uwSBoardTemp  = Adc_GetAuxData(0); // 65536 = Max Value = 1V (AxN 1024 = MaxValue = 3V)
    sIOMgrAnalogMeasurements.uwSBridgeTemp = Adc_GetAuxData(8); // 65536 = Max Value = 1V (AxN 1024 = MaxValue = 3V)
//    sIOMgrAnalogMeasurements.uwSMotorTemp  = (UWORD)(((ULONG)Adc_GetAuxData(1) * 1289)>>16);  // CONVERT TO OLD SCALE
//    sIOMgrAnalogMeasurements.uwSMotorTemp  = (UWORD)(((ULONG)Adc_GetAuxData(1) * 3867)>>16);  // CONVERT TO OLD SCALE
    sIOMgrAnalogMeasurements.uwSMotorTemp  = Adc_GetAuxData(1);
    sIOMgrAnalogMeasurements.uwS24Vaux     = (UWORD)(((ULONG)Adc_GetAuxData(9) * 37*10)>>16); // Max Value: 65536 = 37V * 10 = 370 1e-1 V, Vin = 24V * 10K/(10K + 360K)
    sIOMgrAnalogMeasurements.uwSVENCout    = (UWORD)(((ULONG)Adc_GetAuxData(2) * 13*80)>>16); // Max Value: 65536 = 13V * 100 1e-2 V, Vin = +VENC * 1K/(1K + 12K)

    sIOMgrAnalogMeasurements.swAllValid = TRUE;
#endif // _hw_dc
#endif // _infineon_
}

//***************************************************************************
static void SetupCorrectionFactors(void)
{

#ifndef _HW_DC
#ifdef _INFINEON_
  HWPRMS_BRIDGE_PARAMETERS huge * psBridgeParams;
#else // infineon
  HWPRMS_BRIDGE_PARAMETERS * psBridgeParams;
#endif // _infineon_
#endif // _hw_dc

  switch(sGlbPowerBoardParameters.sProductInfo.uwProductCode)
  { /* parameters: get them from Id-chip */
#ifndef _HW_DC
    case HWPRM_PCODE_POWER_BOARD_OLD_AXW_DC: /* !!! */
    case HWPRM_PCODE_POWER_BOARD_OLD_AXW_AC: /* !!! */
    case HWPRM_PCODE_POWER_BOARD_OLD_AXM_AC:
    case HWPRM_PCODE_POWER_BOARD_AXP_AC:
    case HWPRM_PCODE_POWER_BOARD_AXP_DS_AC:
    case HWPRM_PCODE_POWER_BOARD_AXM_AC:
    case HWPRM_PCODE_POWER_BOARD_AXM_AC_ALMAPS:
    case HWPRM_PCODE_POWER_BOARD_AXM_DS_AC:
    case HWPRM_PCODE_POWER_BOARD_AXW_AC:
    case HWPRM_PCODE_POWER_BOARD_AXW_DC:
        if(sGlbPowerBoardParamDataCode==DATACODE_HW_POWER_BOARD_AXM_AXP_AC)
            psBridgeParams = &sGlbPowerBoardParameters.sAxmAxpAC.sBridgeParams;
        else
            psBridgeParams = &sGlbPowerBoardParameters.sAxmAxpACV2.sBridgeParams;
        sIoMgrRun.sBridgeTempSensorCalibration.swScale = psBridgeParams->sTempSensorCalibr.swScale ;
        sIoMgrRun.sBridgeTempSensorCalibration.uwOffst = (UWORD)((ULONG)psBridgeParams->sTempSensorCalibr.uwOffst) ;
        break ;

    case HWPRM_PCODE_POWER_BOARD_DIF_CI:
    case HWPRM_PCODE_POWER_BOARD_DIF_PI:
    case HWPRM_PCODE_POWER_BOARD_DIF_BC:
        psBridgeParams = &sGlbPowerBoardParameters.sDif.sBridgeParams;
        sIoMgrRun.sBridgeTempSensorCalibration.swScale = psBridgeParams->sTempSensorCalibr.swScale ;
        sIoMgrRun.sBridgeTempSensorCalibration.uwOffst = (UWORD)((ULONG)psBridgeParams->sTempSensorCalibr.uwOffst) ;
        break ;
#endif  // _hw_dc

    case HWPRM_PCODE_POWER_BOARD_UNIVERSAL:
#ifndef _HW_DC
        sIoMgrRun.sBridgeTempSensorCalibration.swScale = sGlbPowerBoardParameters.sUniv.sTempSens.swScale ;
        sIoMgrRun.sBridgeTempSensorCalibration.uwOffst = (UWORD)((ULONG)sGlbPowerBoardParameters.sUniv.sTempSens.uwOffst) ;
#endif // _hw_dc
        break ;

    default:
        assert(FALSE);
  }

  /* export to thermal model the IGBT NTC calibration scale */
#ifndef _HW_DC
  sIOMgrAnalogMeasurements.swBridgeTempSensorCalibratonScale = sIoMgrRun.sBridgeTempSensorCalibration.swScale ; 
#else
  sIOMgrAnalogMeasurements.swBridgeTempSensorCalibratonScale = sGlbPowerBoardParameters.sUniv.sTempSens->swScale ;
#endif // _hw_dc
}

#ifdef _INFINEON_
//***************************************************************************
#ifndef _HW_DC
// for the correction used scale and offset
// A2DCorretto = (A2DValue * Scale) / 8192 + (Offset - 32768) / 32768 = ((32768 / 8192) * (A2DValue * Scale) + (Offset - 32768))/32768
static SWORD MeasureCorrection(UWORD uwA2DValue, HWPRMS_AD_SETTINGS sA2DCalibration)
{
  return (SWORD)((A2D_RATIO * ((SLONG)sA2DCalibration.swScale * uwA2DValue) + (SLONG)sA2DCalibration.uwOffst - A2D_OFFST_ZERO) / A2D_OFFST_ZERO) ; 
}
#else // _hw_dc
// for the correction used only offset (function used only for the IGBT NTC)
// A2DCorretto = A2DValue + (Offset - 32768) / 32768 = (32768 * (A2DValue - 1) + Offset) / 32768
static SWORD MeasureCorrection(UWORD uwA2DValue, UWORD uwA2DOffset)
{
  return (SWORD)(((SLONG)(uwA2DValue - 1) * A2D_OFFST_ZERO + (SLONG)uwA2DOffset) / A2D_OFFST_ZERO) ; 
}
#endif // _hw_dc
#endif // _infineon
