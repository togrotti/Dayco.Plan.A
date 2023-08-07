/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2021 Ningbo Physis Technology Co.,Ltd. All Rights Reserved.  */
/*                                                                          */
/* File        : UserIO.c                                                   */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/* Description : User analog/digital IO                                     */
/*                                                                          */
/****************************************************************************/

// Compiler Option
#pragma GCC optimize (2)

#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "common\TaskScheduler.h"
#include "fpga\FpgaHandler.h"
#include "bus\modbus\ModBusCommandMgr.h"
#include "common\UnitMeasureConversion.h"
#include "fpga\AnProcessor.h"
#include "system\SystemAlarms.h"
#include "common\DspFunctions.h"
#ifdef _INFINEON_
#include "IOExpBrdManager.h"
#include <xe167f.h>
#endif
#include <string.h>

#include "system\SysAppFpgaOptCodes.h"
#include "drive\HardwareUnitID.h"

#define DEFINE_EXTERNALS
#include "UserIO.h"
#undef DEFINE_EXTERNALS

#include "UserIORT.h"

#ifdef _INFINEON_
//***************************************************************************
// Avoids warning C37: nonstandard extension - unnamed parameter
// Avoids warning C47: unreferenced parameter

#pragma warning disable = 37
#pragma warning disable = 47
#endif

//***************************************************************************
// Defines

#define EXTRLY_STARTUP_DELAY            100     // msec

#define USR_OUTPUT_MIN                  (-0x7FFF)
#define USR_OUTPUT_MAX                  ( 0x7FFF)

#ifndef _HW_DC
#define ANIN_RATIO                      (10000.0/32768.0)
#define ANOUT_RATIO                     (1.0/10000.0)

#ifdef _INFINEON_
#define RLY_DRIVE                       (FPGA_GENERIC_IO_RLY_DRIVE)
#else
#define RLY_DRIVE_PIN                   (MIO_PIN_BASE+7)
#endif // _infineon

#define GLB_CBRD                        sGlbControlBoardParameters.sStd

#else
#define ANIN_RATIO                      (1000.0/32768.0)
#define ANOUT_RATIO                     (1.0/1000.0)

#ifdef _INFINEON_
#define RLY_DRIVE                       (P1_OUT_P6)
#else

#ifndef _HW_AXS
#define RLY_DRIVE_PIN                   (MIO_PIN_BASE+7)
#endif

#endif // _infineon

#define GLB_CBRD                        sGlbControlBoardParameters.sUniv
#endif // _hw_dc

#ifndef _HW_AXS
#define USERREF0_MODE_PIN               (MIO_PIN_BASE+35)
#define USERREF1_MODE_PIN               (MIO_PIN_BASE+36)
#endif

//***************************************************************************
// Control panel definitions

USRIO_CPANEL_BASE sUsrIOCPBase;

#ifdef _INFINEON_
UWORD uwUsrIOHwDetect;
UWORD uwUsrIOFlags;
const USRIO_CPANEL_DEFS huge sUserIOCPDefs=
#else
USRIO_PLC_DIGITALIO sPlcDIO;
USRIOHWDETECT sUsrIOHwDetect;
USRIOFLAGS sUsrIOFlags;
const USRIO_CPANEL_DEFS sUserIOCPDefs=
#endif
{
    ((USRIO_AOUT_CHANNELS-1)<<12)|((USRIO_AIN_CHANNELS-1)<<8)|
    ((USRIO_DOUT_CHANNELS-1)<< 4)|((USRIO_DIN_CHANNELS-1)   ),

    &sUsrIOCPBase,
    &sUsrIOCPData
};

//***************************************************************************
// Globals for RT

#ifndef _APP_XC
bit bUsrIOboardge102;
bit bUsrIOboardge200;
bit bUsrIOboardge400;
#endif

#ifndef _HW_DC
BOOL bUsrIOaninput01lowg;
BOOL bUsrIOaninput23lowg;
BOOL bUsrIOaninputse;
#endif

SWORD swUsrIOAnalogInValue[4];
#ifdef _HW_DC
UBYTE ubIOboardType = 0;
#endif
UMCONV_CONV32TO16 sUsrIOAnalogOut0Conv;
SWORD swUsrIOAnalogOut0Offset;
UMCONV_CONV32TO16 sUsrIOAnalogOut1Conv;
SWORD swUsrIOAnalogOut1Offset;

void (* pfUsrIOAnInpProcessing)(void)=NULL;
void (* pfUsrIOIOExpProcess)(void)=NULL;

//***************************************************************************
// Local functions

static void slowtask(void);
static void setupchannelsgain(BOOL bForce);

//***************************************************************************
// Hardware option callback

#ifdef _APP_XC
void UserIO_GetHwOpt(UWORD w, ULONG * pulHwOpt, ULONG * pulFPGAOpt)
{
    // do not require mandatory IO board here, if not available
    // will be check later by preboot PLC
    *pulHwOpt  |=0;
    *pulFPGAOpt|=FPGA_HW_ANIOSTANDARD;
}
#endif

//***************************************************************************
// Initialization entry point

BOOL UserIO_Init(UWORD w)
{
    FLOAT flAddRatio;
    DOUBL dbTmp;
#ifndef _HW_DC
    UWORD uwAddOffset;
    UBYTE ubAddOptions;
#else
    #define uwAddOffset (0)
    #define ubAddOptions (0)
#endif
    ANPROC_CHAN sAInDef;

#ifdef _INFINEON_
#if defined(_HW_DC)
        // setup RLY port
    P1_IOCR06 = 0x0080;
#endif
#endif

#ifdef _HW_DC
        // check hardware availability, if not available
        // just install slowtask and exit
    bUsrIOHwAvailable=(BOOL)((sGlbControlBoardParameters.sUniv.sUnit.ulIDMask&HWUNITID_IO_STD)==HWUNITID_IO_STD) ? TRUE : FALSE;
    if(!bUsrIOHwAvailable)
        return TaskSched_AddBackgroundTask(&slowtask);
#endif

        // check controlboard version for different I/Os
#ifndef _APP_XC
    if(sGlbControlBoardParameters.sProductInfo.uwProductRev<102)
        bUsrIOboardge102=FALSE;
    else
        bUsrIOboardge102=TRUE;

    if(sGlbControlBoardParameters.sProductInfo.uwProductRev<200)
        bUsrIOboardge200=FALSE;
    else
        bUsrIOboardge200=TRUE;

    if(sGlbControlBoardParameters.sProductInfo.uwProductRev<400)
        bUsrIOboardge400=FALSE;
    else
        bUsrIOboardge400=TRUE;
#endif

        // Port Input/Output Control Registers
#ifdef _INFINEON_
#if !defined(_HW_DC)
    if(bUsrIOboardge200)
        P1_IOCR06 = 0x0080;
    else
        P8_IOCR00 = 0x0080;
    P8_IOCR01 = 0x0080;
    P8_IOCR02 = 0x0080;
    P8_IOCR03 = 0x0080;
#else
    P9_IOCR00 = 0x0080;
    P9_IOCR01 = 0x0080;
    P9_IOCR02 = 0x0080;
    P9_IOCR03 = 0x0080;
#endif

#else

#ifdef USRIO_DI0_PIN
    Gpio_SetMode(USRIO_DI0_PIN, GPIO_DIR_IN);
#endif
#ifdef USRIO_DI1_PIN
    Gpio_SetMode(USRIO_DI1_PIN, GPIO_DIR_IN);
#endif
#ifdef USRIO_DI2_PIN
    Gpio_SetMode(USRIO_DI2_PIN, GPIO_DIR_IN);
#endif
#ifdef USRIO_DI3_PIN
    Gpio_SetMode(USRIO_DI3_PIN, GPIO_DIR_IN);
#endif
#ifdef USRIO_DI4_PIN
    Gpio_SetMode(USRIO_DI4_PIN, GPIO_DIR_IN);
#endif
#ifdef USRIO_DI5_PIN
    Gpio_SetMode(USRIO_DI5_PIN, GPIO_DIR_IN);
#endif
#ifdef USRIO_DI6_PIN
    Gpio_SetMode(USRIO_DI6_PIN, GPIO_DIR_IN);
#endif
#ifdef USRIO_DI7_PIN
    Gpio_SetMode(USRIO_DI7_PIN, GPIO_DIR_IN);
#endif

#ifdef USRIO_DO0_PIN
    Gpio_SetMode(USRIO_DO0_PIN, GPIO_DIR_OUT);
#endif
#ifdef USRIO_DO1_PIN
    Gpio_SetMode(USRIO_DO1_PIN, GPIO_DIR_OUT);
#endif
#ifdef USRIO_DO2_PIN
    Gpio_SetMode(USRIO_DO2_PIN, GPIO_DIR_OUT);
#endif
#ifdef USRIO_DO3_PIN
    Gpio_SetMode(USRIO_DO3_PIN, GPIO_DIR_OUT);
#endif

#endif

        // clear data structures
    memset(&sUsrIOCPBase.sDIO,0,sizeof(sUsrIOCPBase.sDIO));
    memset(&sUsrIOCPData.sAIO,0,sizeof(sUsrIOCPData.sAIO));

        // check differential or SE selection; SE with old board is not supported
#if !defined(_HW_DC)
    if(bUsrIOFlgAnInputsSE && !bUsrIOboardge200)
        bUsrIOFlgAnInputsSE=FALSE;
    bUsrIOaninputse=bUsrIOFlgAnInputsSE;
#endif

        // additional analog setup
    setupchannelsgain(TRUE);

        // prepare analog in 0 and 1 ratio
#ifndef _HW_DC
    if(bUsrIOaninput01lowg)
    {
        flAddRatio=0.5;
        uwAddOffset=-32767;
        ubAddOptions=ANPROC_OF_UNSIGNED;
    }
    else
    {
        flAddRatio=1.0;
        uwAddOffset=0;
        ubAddOptions=0;
    }

#ifdef _INFINEON_
#ifndef _APP_DEBUG
    if(bUsrIOboardge400)
        P6_OUT_P0=bUsrIOaninput01lowg;
#endif // _app_debug
#else
    if(bUsrIOboardge400)
    {
        Gpio_SetMode(USERREF0_MODE_PIN, GPIO_DIR_OUT);
        GPIO_OUT(USERREF0_MODE_PIN,bUsrIOaninput01lowg);
    }
#endif // _infineon_
#endif // _hw_dc

        // common data for analog input processing
    sAInDef.ubNumSample=64;
    sAInDef.uwDstIntImm=ANPROC_ADR_DISABLE;
    sAInDef.uwDstIntAvg=ANPROC_ADR_DISABLE;
    sAInDef.pvDstExtImm=NULL;

        // common data for analog 0 and 1
    sAInDef.ubOpt=ANPROC_OF_AVG_1SHT|ANPROC_OF_BLK_NONE|ANPROC_OF_DST_SHORT|ubAddOptions;
#ifndef _HW_DC
    sAInDef.flScale=ANIN_RATIO*flAddRatio;
#else
    sAInDef.flScale=ANIN_RATIO*GLB_CBRD.sInRange.flHiSpan;
#endif


        // prepare analog in 0 unit measure conversion
    sAInDef.sCal.uwOffst=GLB_CBRD.sUserRefI0.uwOffst+uwAddOffset;
    sAInDef.sCal.swScale=GLB_CBRD.sUserRefI0.swScale;
    sAInDef.pvDstExtAvg=&swUsrIOAnalogInValue[0];
    AnProc_Set(FPGA_ADTAGS_USRIO_AIN0,&sAInDef);

        // prepare analog in 1 unit measure conversion
    sAInDef.sCal.uwOffst=GLB_CBRD.sUserRefI1.uwOffst+uwAddOffset;
    sAInDef.sCal.swScale=GLB_CBRD.sUserRefI1.swScale;
    sAInDef.pvDstExtAvg=&swUsrIOAnalogInValue[1];
    AnProc_Set(FPGA_ADTAGS_USRIO_AIN1,&sAInDef);

    if(bUsrIOboardge200)
    {
            // prepare analog in 2 and 3 ratio
#ifndef _HW_DC
        if(bUsrIOaninput23lowg)
        {
            flAddRatio=0.5;
            uwAddOffset=-32767;
            ubAddOptions=ANPROC_OF_UNSIGNED;
        }
        else
        {
            flAddRatio=1.0;
            uwAddOffset=0;
            ubAddOptions=0;
        }

#ifdef _INFINEON_
#ifndef _APP_DEBUG
        if(bUsrIOboardge400)
            P7_OUT_P0=bUsrIOaninput23lowg;
#endif // _app_debug
#else
        if(bUsrIOboardge400)
        {
            Gpio_SetMode(USERREF1_MODE_PIN, GPIO_DIR_OUT);
            GPIO_OUT(USERREF1_MODE_PIN,bUsrIOaninput23lowg);
        }
#endif // _infineon_
#endif // _hw_dc

            // common data for analog 2 and 3
        sAInDef.ubOpt=ANPROC_OF_AVG_1SHT|ANPROC_OF_BLK_NONE|ANPROC_OF_DST_SHORT|ubAddOptions;
#ifndef _HW_DC
        sAInDef.flScale=ANIN_RATIO*flAddRatio;
#else
        sAInDef.flScale=ANIN_RATIO*GLB_CBRD.sInRange.flHiSpan;
#endif

            // prepare analog in 2 unit measure conversion
        sAInDef.sCal.uwOffst=GLB_CBRD.sUserRefI2.uwOffst+uwAddOffset;
        sAInDef.sCal.swScale=GLB_CBRD.sUserRefI2.swScale;
        sAInDef.pvDstExtAvg=&swUsrIOAnalogInValue[2];
        AnProc_Set(FPGA_ADTAGS_USRIO_AIN2,&sAInDef);
    
            // prepare analog in 3 unit measure conversion
        sAInDef.sCal.uwOffst=GLB_CBRD.sUserRefI3.uwOffst+uwAddOffset;
        sAInDef.sCal.swScale=GLB_CBRD.sUserRefI3.swScale;
        sAInDef.pvDstExtAvg=&swUsrIOAnalogInValue[3];
        AnProc_Set(FPGA_ADTAGS_USRIO_AIN3,&sAInDef);
    }

        // select rt processing for analog inputs
#if !defined(_HW_DC)
    if(!bUsrIOboardge200)
        pfUsrIOAnInpProcessing=&UserIO_RT_aninput_2chanstd;
    else if(!bUsrIOaninputse)
        pfUsrIOAnInpProcessing=&UserIO_RT_aninput_2chandiff;
    else
#endif
        pfUsrIOAnInpProcessing=&UserIO_RT_aninput_4chanse;

        // prepare ratio for analog output
#ifndef _HW_DC
    flAddRatio=ANOUT_RATIO;
#else
    flAddRatio=ANOUT_RATIO/GLB_CBRD.sOutRange.flHiSpan;
#endif // _hw_dc

        // prepare analog out 0 unit measure conversion
    dbTmp = ((DOUBL)GLB_CBRD.sUserAnaO0.swScale/8192.0)*flAddRatio;
    swUsrIOAnalogOut0Offset=(SWORD)(((DOUBL)USR_OUTPUT_MIN+(SWORD)(GLB_CBRD.sUserAnaO0.uwOffst-32768))/(65536*dbTmp));
    UmConv_Init_32to16(&sUsrIOAnalogOut0Conv, dbTmp);

        // prepare analog out 1 unit measure conversion
    dbTmp = ((DOUBL)GLB_CBRD.sUserAnaO1.swScale/8192.0)*flAddRatio;
    swUsrIOAnalogOut1Offset=(SWORD)(((DOUBL)USR_OUTPUT_MIN+(SWORD)(GLB_CBRD.sUserAnaO1.uwOffst-32768))/(65536*dbTmp));
    UmConv_Init_32to16(&sUsrIOAnalogOut1Conv, dbTmp);

#ifdef _INFINEON_
        // initialize IO expansion board if present and not disabled
#if !defined(_HW_DC)
    if(!bUsrIOFlgDisableIOExp)
        if(!IOExpBrdManager_Init(&pfUsrIOIOExpProcess))
            return FALSE;
    bUsrIOHwDetIOExpAvailable=(pfUsrIOIOExpProcess!=NULL);
#endif // _hw_dc
#endif // _infineon_

#ifndef _HW_DC
        // setup user IO availability
    bUsrIOHwDetSEAvailable=bUsrIOboardge200;
    bUsrIOHwDetLowGAvailable=bUsrIOboardge400;

#endif // _hw_dc

        // add tasks
    if(!TaskSched_AddBackgroundTask(&slowtask))
        return FALSE;

    assert(pfUsrIOAnInpProcessing);

    if(!TaskSched_AddRTTask(&UserIO_RT_task, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0))
        return FALSE;

        // refresh values
    UserIO_RT_task();

    return TRUE;
}

//***************************************************************************
// slow task

static void slowtask(void)
{
    static BOOL bIsSetupTout=FALSE;
    static BOOL bDelayElapsed=FALSE;
    static BOOL bPrevEnableSimulatedInputs=FALSE;
    static BOOL bPrevDisablePhysicalOutputs=FALSE;
    static UWORD uwTimeOut;
    UWORD uwCnt;

        // while booting do nothing
    if(bSysStatBooting)
        return;

        // delayed management after startup and for each change
    if(bDelayElapsed)
    {
        BOOL bNewState;

            // external relay management for drive ready
        if(bSysStatBooting || bSysStatLockedByBootError || bSysStatLockedByPlcReload || bSysStatFault || bSysStatResetting)
            bNewState=FALSE;
        else
            bNewState=TRUE;

#ifndef _HW_AXS

#ifdef _INFINEON_
        if(bNewState!=RLY_DRIVE)
        {
            RLY_DRIVE=bNewState;
#else
        Gpio_SetMode(RLY_DRIVE_PIN, GPIO_DIR_IN);
        if(bNewState!=GPIO_IN(RLY_DRIVE_PIN))
        {
            Gpio_SetMode(RLY_DRIVE_PIN, GPIO_DIR_OUT);
            GPIO_OUT(RLY_DRIVE_PIN, bNewState);
#endif

#endif // _HW_AXS
            bIsSetupTout=FALSE;
            bDelayElapsed=FALSE;
#ifndef _HW_AXS
        }
#endif // _HW_AXS
    }
        // check for timer
    if(!bIsSetupTout)
    {
        uwTimeOut=timer_settimeout(uwSysTimers1ms,EXTRLY_STARTUP_DELAY);
        bIsSetupTout=TRUE;
    }
    else
        if(timer_istimedout(uwSysTimers1ms,uwTimeOut))
            bDelayElapsed=TRUE;

        // if IO not available then exit here
#ifdef _HW_DC
    if(!bUsrIOHwAvailable)
        return;
#endif

        // compatibility mode for old control panel - input section
    if(bPrevEnableSimulatedInputs!=bModBusSMEnableSimulatedInputs)
    {
        bPrevEnableSimulatedInputs=bModBusSMEnableSimulatedInputs;

            // same for both state, just to correctly reset all
            // flags in case disconnected from new control panel and
            // attached to old one
        sUsrIOCPBase.uwDgInForceOn=0x0000;
        sUsrIOCPBase.uwDgInForceOff=0x0000;
        sUsrIOCPBase.uwAnInSimulation=0x0000;
    }

        // compatibility mode for old control panel - output section
    if(bPrevDisablePhysicalOutputs!=bModBusSMDisablePhysicalOutputs)
    {
        bPrevDisablePhysicalOutputs=bModBusSMDisablePhysicalOutputs;

            // if set to disable physical output
        if(bModBusSMDisablePhysicalOutputs)
        {
            sUsrIOCPBase.uwDgOutForceOn=0x0000;
            sUsrIOCPBase.uwDgOutForceOff=0xFFFF;

            for(uwCnt=0;uwCnt<USRIO_AOUT_CHANNELS;uwCnt++)
                sUsrIOCPData.swOutSimulation[uwCnt]=0;
            sUsrIOCPBase.uwAnOutSimulation=0xFFFF;
        }
        else
        {
            sUsrIOCPBase.uwDgOutForceOn=0x0000;
            sUsrIOCPBase.uwDgOutForceOff=0x0000;
            sUsrIOCPBase.uwAnOutSimulation=0x0000;
        }
    }

        // setup request from plc for analog low or standard gain
    setupchannelsgain(FALSE);
}

//***************************************************************************
// setup low or standard gain for analog channels

static void setupchannelsgain(BOOL bForce)
{
#if !defined(_HW_DC)
    BOOL bFlag;

        // check for selection of lo gain for analog channels 0 and 1
    bFlag=bUsrIOFlgAnInput01LowG;
    if(bFlag && !bUsrIOboardge400)
        bFlag=FALSE;
    bUsrIOaninput01lowg=bUsrIOFlgAnInput01LowG=bFlag;

        // check for selection of lo gain for analog channels 2 and 3
    bFlag=bUsrIOFlgAnInput23LowG;
    if(bFlag && !bUsrIOboardge400)
        bFlag=FALSE;
    bUsrIOaninput23lowg=bUsrIOFlgAnInput23LowG=bFlag;

        // if different or forced then proceed with channels 0 and 1
    if(bUsrIOHwAnInput01LowGEn!=bUsrIOaninput01lowg || bForce)
    {
#ifdef _INFINEON_
#ifndef _APP_DEBUG
        if(bUsrIOboardge400)
            P6_OUT_P0=bUsrIOaninput01lowg;
#endif
#else
        if(bUsrIOboardge400)
        {
            Gpio_SetMode(USERREF0_MODE_PIN, GPIO_DIR_OUT);
            GPIO_OUT(USERREF0_MODE_PIN,bUsrIOaninput01lowg);
        }
#endif
            // signal new setup
        bUsrIOHwAnInput01LowGEn=bUsrIOaninput01lowg;
    }

        // if different or forced then proceed with channels 0 and 1
    if((bUsrIOHwAnInput23LowGEn!=bUsrIOaninput23lowg || bForce) && bUsrIOboardge200)
    {
#ifdef _INFINEON_
#ifndef _APP_DEBUG
        if(bUsrIOboardge400)
            P7_OUT_P0=bUsrIOaninput23lowg;
#endif
#else
        if(bUsrIOboardge400)
        {
            Gpio_SetMode(USERREF1_MODE_PIN, GPIO_DIR_OUT);
            GPIO_OUT(USERREF1_MODE_PIN,bUsrIOaninput23lowg);
        }
#endif
            // signal new setup
        bUsrIOHwAnInput23LowGEn=bUsrIOaninput23lowg;
    }
#endif
}
