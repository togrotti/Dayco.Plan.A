/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright ?2017, Phase Motion Control. All Rights Reserved.              */
/*                                                                          */
/* File        : PSUManager.c                                               */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Power Supply Unit SPI communication manager                */
/*                                                                          */
/****************************************************************************/

#include "system\SysAppGlobals.h"
#include "drive\HardwareConfig.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonUtility.h"
#include "common\AppIdentTypes.h"
#include "common\TaskScheduler.h"
#include "system\Os.h"
#include "system\SysLogManagement.h"
#include "system\SystemAlarms.h"
#include "drive\HardwareUnitID.h"
// #include <xe167f.h>
#include <string.h>
#include "common\UnitMeasureConversion.h"
#include "drive\MotorHandler.h"
#include "drive\MotorHandlerRT.h"
#include "drive\ThermalModel.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "PSUManager.h"

//***************************************************************************
// Avoids warning C37: nonstandard extension - unnamed parameter

// #pragma warning disable = 37

//***************************************************************************
// Defines

#ifdef _HW_DC
#if CFG_PSU_MNGR

#define PROT_SETUPMODE              0x4000
#define PROT_RUNMODE                0x0000
#define PROT_MODE_MSK               0x4000

#define PROT_SETUP_HEADER           0x4855

#define PROT_SETUP_READ             0x0000
#define PROT_SETUP_WRITE            0x0100

#define PROT_SETUP_ADDRESS(x)       ((x)&0x00FF)

#define ADR_IDCHIP                  0x00
#define ADR_VERSION                 0x02
#define ADR_BUILDNUMBER             0x04
#define ADR_PROTOCOL                0x06
#define ADR_QUICKSTOP               0x30
#define ADR_SYSRESET                0x32
#define ADR_BRAKETHD                0x34
#define ADR_POWERON                 0x36
#define ADR_COOLTEMP                0x38
#define ADR_SYSTATE                 0x40

#define ADR_VDCPEAK                 0x50
#define ADR_IDCPEAK                 0x52
#define ADR_VR                      0x54
#define ADR_VS                      0x56
#define ADR_VT                      0x58
#define ADR_VRS                     0x5A
#define ADR_VST                     0x5C
#define ADR_VTR                     0x5E
#define ADR_POWEROK                 0x60
#define ADR_INPUT_EN                0x62
#define ADR_READY                   0x64
#define ADR_OVERBVOLTHRESHOLD       0x66

#define PROT_RUN_SIZE               16
#define PROT_SETUP_SIZE             16

#define PROT_MAXWDTFAIL             4

#define TRANSACTION_TIMEOUT         1250

#define TRANSACTION_PERIOD_125US    0x0000
#define TRANSACTION_PERIOD_250US    0x0001
#define TRANSACTION_PERIOD_500US    0x0003

#define PSU_PROTOCOL_SUPPORTED      0x00010000

#define DEFAULT_PSUBRAKELOW         720 // [V]
#define DEFAULT_PSUBRAKEHIGH        750 // [V]

#define PSU_CHECK_CODE		        0x0132

//***************************************************************************
// Globals

PSU_WORKS       sPSUWrk;
PSUBRD_IN       sPSUBrdIn;
PSU_INFO        sPSUInfo;
PSU_COMM_DATA   sPSUCommData;
PSU_PARAM       sPSUInParam;

//***************************************************************************
// Structures

typedef struct
{
    UWORD uwHeader;
    UWORD uwCmd;
    SLONG slData;
    ULONG ulWDT;
    union
    {
        SLONG l[5];
        FLOAT f[5];
    }RGData;
}PSUCOMMFRAME;
union
{
    UWORD w[16];
    PSUCOMMFRAME fm;
}psucomm;

typedef struct
{
    ULONG ulIDChip;
    ULONG ulProtocol;
    ULONG ulWatchDogCnt;
    UWORD uwWDFailCnt;
    UWORD uwRTActCnt;
    union {
        struct {
            UWORD bAlarmSent : 1 ;
        } b ;
        UWORD w ;
    } sErrorSent ;

    UBYTE ubRTCmdWrite;
    UBYTE ubRTCmdAddr;
}PSU_RUNTIME;

//***************************************************************************
// Locals

static PSU_RUNTIME sRuntime;
static struct
{
    union
    {
        struct
        {
            UWORD bPowerOn:1;          // power on
        } f;
        UWORD w;
    } flags;

    union
    {
        struct
        {
            SWORD lo;
            SWORD hi;
        }w;
        ULONG l;
    } VBrake,CoolTemp, VolThreshold;
} sShadowPSUParam;

//***************************************************************************
// Local functions

static void cfg_spi_init(void);
static BOOL spitransaction(UBYTE ubAddress, BOOL bWrite, HPUWORD hpuwData);
static BOOL rttask(void);
static void liveParametersCheck(void);

BOOL PSUManager_Init(UWORD W)
{
    BOOL  bRetVal=TRUE;

        // if module disabled
    if(!sPSUWrk.bEnableModule)//if(!sPSUWrk.flags.f.bEnableModule)
        return TRUE;
        
        // init runtime
    sRuntime.uwWDFailCnt=0;
    sRuntime.uwRTActCnt=5;
    sRuntime.sErrorSent.b.bAlarmSent=FALSE;
    
        // init SPI HW
    cfg_spi_init();

        // get build number from the module and check validity
    spitransaction(ADR_IDCHIP, FALSE, (HPUWORD)&sRuntime.ulIDChip); // first dump
    
    bRetVal&=spitransaction(ADR_IDCHIP,      FALSE, (HPUWORD)&sRuntime.ulIDChip);
    bRetVal&=spitransaction(ADR_VERSION,     FALSE, (HPUWORD)&sPSUInfo.ulVersion);
    bRetVal&=spitransaction(ADR_BUILDNUMBER, FALSE, (HPUWORD)&sPSUInfo.ulModuleBuild);
    bRetVal&=spitransaction(ADR_PROTOCOL,    FALSE, (HPUWORD)&sRuntime.ulProtocol);
    
        // check protocol version 
    if(!bRetVal||(sRuntime.ulProtocol!=PSU_PROTOCOL_SUPPORTED))
    {
        uwSystemBootErrorCode=SYSTEMBOOTERR_PSU_INCOMPAT;
        memset(&sPSUInfo, 0, sizeof(sPSUInfo));
        return FALSE;
    }
        
        // get brake threshold
    bRetVal&=spitransaction(ADR_BRAKETHD,FALSE,(HPUWORD)&sShadowPSUParam.VBrake.l);
    bRetVal&=spitransaction(ADR_COOLTEMP,FALSE,(HPUWORD)&sShadowPSUParam.CoolTemp.l);
	bRetVal&=spitransaction(ADR_OVERBVOLTHRESHOLD,FALSE,(HPUWORD)&sShadowPSUParam.VolThreshold.l);
    if(!bRetVal)
        return FALSE;
    
        // create realtime task for communication
    if(!TaskSched_AddRTTask(&rttask, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0))
        return FALSE;

       /* install bg function */ 
    if(!TaskSched_AddBackgroundTask(&liveParametersCheck))
        return FALSE ;   

    return TRUE;
}

//***************************************************************************
// Hardware option callback

void PSUManager_GetHwOpt(UWORD W, ULONG * pulHwOpt, ULONG * P)
{
	if(sPSUWrk.bEnableModule)//if(sPSUWrk.flags.f.bEnableModule)
        *pulHwOpt |= HWUNITID_PSU_CB|HWUNITID_PSU_ROUTING;
}

//***************************************************************************
// SPI HW Init

static void cfg_spi_init(void)
{
#ifdef _INFINEON_
    ///  -----------------------------------------------------------------------
    ///  Configuration of the U0C1 Kernel State Configuration:
    ///  -----------------------------------------------------------------------
    U0C1_KSCFG     =  0x0003;      // load U0C1 kernel state configuration register
    _nop_();  // one cycle delay 
    _nop_();  // one cycle delay 
    
    ///  -----------------------------------------------------------------------
    ///  Configuration of the U0C1 Channel Control Register (Mode & Interrupts):
    ///  -----------------------------------------------------------------------
    /// - The USIC channel is disabled
    /// - The parity generation is disabled
    U0C1_CCR       =  0x0000;
    _nop_();  // one cycle delay 
    _nop_();  // one cycle delay 
    
#ifdef SYSPERF_SUPPORTED
    if(uwSystemMainClockFreq==SYSTEMCLOCK_PERF)
    {
        ///  -----------------------------------------------------------------------
        ///  Configuration of the U0C1 Fractional Divider:
        ///  -----------------------------------------------------------------------
        ///  - The normal divider is selected
        ///  - The step value STEP = 1022
        U0C1_FDRL      =  0x43FE;      // load U0C1 fractional divider register
    
        ///  -----------------------------------------------------------------------
        ///  Configuration of the U0C1 Baudrate Generator:
        ///  -----------------------------------------------------------------------
        ///  - The selected BaudRate is 5,000 Mbaud @ 125MHz
        ///  - The PreDivider for CTQ, PCTQ = 0
        ///  - The Denominator for CTQ, DCTQ = 0
        ///  - The Divider factor PDIV = 5
    
        U0C1_BRGL      =  0x0000;      // load U0C1 load baud rate generator register L
        U0C1_BRGH      =  0x0005;      // load U0C1 load baud rate generator register H (10 Mbaud)
    }
    else
#endif
    {
        ///  -----------------------------------------------------------------------
        ///  Configuration of the U0C1 Fractional Divider:
        ///  -----------------------------------------------------------------------
        ///  - The Normal divider is selected
        ///  - The step value STEP = 1022
        
        U0C1_FDRL      =  0x43FE;      // load U0C1 fractional divider register
        
        ///  -----------------------------------------------------------------------
        ///  Configuration of the U0C1 Baudrate Generator:
        ///  -----------------------------------------------------------------------
        ///  - The selected BaudRate is 5,000 Mbaud @ 80MHZ
        ///  - The PreDivider for CTQ, PCTQ = 0
        ///  - The Denominator for CTQ, DCTQ = 0
        ///  - The Divider factor PDIV = 3
        
        U0C1_BRGL      =  0x0000;      // load U0C1 load baud rate generator 
                                       // register L
        
        U0C1_BRGH      =  0x0003;      // load U0C1 load baud rate generator 
                                       // register H
    }
    
    ///  -----------------------------------------------------------------------
    ///  Configuration of the U0C1 Input Control Register:
    ///  -----------------------------------------------------------------------
    ///  - The data input DX0D is selected
    
    U0C1_DX0CR     =  0x0013;      // load U0C1 input control register(Data)
    
    ///  -----------------------------------------------------------------------
    ///  Configuration of the U0C1 Interrupt Node Pointer Register:
    ///  -----------------------------------------------------------------------
    
    U0C1_INPRL     =  0x0000;      // load U0C1 Interrupt Node Pointer register 
                                   // L
    U0C1_INPRH     =  0x0000;      // load U0C1 Interrupt Node Pointer register 
                                   // H
    
    ///  -----------------------------------------------------------------------
    ///  Configuration of the U0C1 Shift Control:
    ///  -----------------------------------------------------------------------
    ///  - Transmit/Receive MSB first is selected
    ///  - Frame length 64bit
    
    U0C1_SCTRL     =  0x0103;      // load U0C1 shift control register L
    U0C1_SCTRH     =  0x0F3F;      // load U0C1 shift control register H
    
    ///  -----------------------------------------------------------------------
    ///  Configuration of the U0C1 Transmit Control/Status Register:
    ///  -----------------------------------------------------------------------
    
    U0C1_TCSRL     =  0x0500;      // load U0C1 transmit control/status 
                                   // register L
    U0C1_TCSRH     =  0x0000;      // load U0C1 transmit control/status 
                                   // register H
    
    ///  -----------------------------------------------------------------------
    ///  Configuration of the U0C1 Protocol Control Register:
    ///  -----------------------------------------------------------------------
    
    U0C1_PCRL      =  0x0F47;      // load U0C1 protocol control register L
    U0C1_PCRH      =  0x0000;      // load U0C1 protocol control register H
    
    
    ///  -----------------------------------------------------------------------
    ///  Configuration of the used U0C1 Interrupts:
    ///  -----------------------------------------------------------------------
    
    
    
    ///  -----------------------------------------------------------------------
    ///  Configuration of the used U0C1 Port Pins:
    ///  -----------------------------------------------------------------------
    P2_IOCR03      =  0x0000;      // load port register for data input
    
    ///  - P2.4 is used for USIC0Channel1 Shift Data  output(DOUT)
    ///  - P2.8 is used for USIC0Channel1 Shift Clock Output(SCLKOUT)
    
    P2_IOCR04      =  0x0090;    //set direction register
    P2_IOCR08      =  0x0090;    //set direction register
    
    
    ///  -----------------------------------------------------------------------
    ///  Configuration of U0C1 FIFO:
    ///  -----------------------------------------------------------------------
    ///  -----------------------------------------------------------------------
    ///  Configuration of U0C1 Transmitter Buffer Conrol Register:
    ///  -----------------------------------------------------------------------
    ///  - Transmit FIFO buffer contains 16 entries
    ///  - Transmit FIFO buffer starts at Data Pointer 16
    ///  - Limit for transmit FIFO interrupt generation is 0
    ///  - Limit for transmit FIFO interrupt generation is 0
    ///  - Limit for transmit FIFO interrupt generation is 0
    
    U0C1_TBCTRL    =  0x0010;      // load U0C1 transmitter buffer control 
                                   // register L
    U0C1_TBCTRH    =  0x0400;      // load U0C1 transmitter buffer control 
                                   // register H
    
    ///  -----------------------------------------------------------------------
    ///  Configuration of U0C1 Receiver Buffer Conrol Register:
    ///  -----------------------------------------------------------------------
    ///  - Receive FIFO buffer contains 16 entries
    ///  - Receive FIFO buffer starts at Data Pointer 32
    ///  - Limit for receive FIFO interrupt generation is 0
    ///  - Filling level mode is selected
    ///  - Filling level mode is selected
    ///  - Filling level mode is selected
    ///  - Filling level mode is selected
    
    U0C1_RBCTRL    =  0x0020;      // load U0C1 receive buffer control register 
                                   // L
    U0C1_RBCTRH    =  0x0400;      // load U0C1 receive buffer control register 
                                   // H
    ///  -----------------------------------------------------------------------
    ///  Configuration of the U0C1 Channel Control Register (Mode & Interrupts):
    ///  -----------------------------------------------------------------------
    ///  - SSC (SPI) Protocol is selected 
    ///  - The parity generation is disabled

    U0C1_CCR       =  0x0001;
    _nop_();  // one cycle delay 
    _nop_();  // one cycle delay
#endif 
}

//***************************************************************************
// protocol setup mode - transaction

static BOOL spitransaction(UBYTE ubAddress, BOOL bWrite, HPUWORD hpuwData)
{
    UWORD uwCmd;
    BOOL  bRetVal=FALSE;
    UWORD uwTime;
    UWORD uwCt;
    
        // Send Request Command, 125us
    uwTime = timer_settimeout(uwSysTimers100ns,TRANSACTION_TIMEOUT);

    uwCmd=PROT_SETUPMODE|(bWrite?PROT_SETUP_WRITE:PROT_SETUP_READ)|PROT_SETUP_ADDRESS(ubAddress);
#ifdef _INFINEON_
        // write first word (header)
    while(U0C1_TRBSRL&0x1000);
    U0C1_IN00=PROT_SETUP_HEADER;
        // write second word (command)
    while(U0C1_TRBSRL&0x1000);
    U0C1_IN00=uwCmd;
        // write third word (data, if write transaction)
    while(U0C1_TRBSRL&0x1000);
    U0C1_IN00=(bWrite?hpuwData[0]:0x0000);
        // write forth word (data, if write transaction)
    while(U0C1_TRBSRL&0x1000);
    U0C1_IN00=(bWrite?hpuwData[1]:0x0000);  
        // dump 12 words
    for(uwCt=0;uwCt<PROT_SETUP_SIZE-4;uwCt++)
    {
        while(U0C1_TRBSRL&0x1000);
        U0C1_IN00=0x0000;
    }
    
    while(!timer_istimedout(uwSysTimers100ns,uwTime)); // wait until 125us
    U0C1_TRBSCR = 0x4000; // Flush RX FIFO
    
        // Receive Responce, 125us
    if(!bWrite)
    {
        uwTime = timer_settimeout(uwSysTimers100ns,TRANSACTION_TIMEOUT);
        
            // master sclkout
        for(uwCt=0;uwCt<PROT_SETUP_SIZE;uwCt++)
        {
            while(U0C1_TRBSRL&0x1000);
            U0C1_IN00=0x0000;
        }
        
            // read first word (header)
        while(U0C1_TRBSRL&0x0008);
        bRetVal=(PROT_SETUP_HEADER==U0C1_OUTRL);
                    
            // read second word (check received command)
        while(U0C1_TRBSRL&0x0008);
        bRetVal&=(uwCmd==U0C1_OUTRL);
            
        if(bRetVal)
        {
                // read third word (data, if read transaction)
            while(U0C1_TRBSRL&0x0008);
            hpuwData[0]=U0C1_OUTRL;
                // read forth word (data, if read transaction)
            while(U0C1_TRBSRL&0x0008);
            hpuwData[1]=U0C1_OUTRL;
        }
        else
        {
            while(U0C1_TRBSRL&0x0008);
            uwCmd=U0C1_OUTRL;
            while(U0C1_TRBSRL&0x0008);
            uwCmd=U0C1_OUTRL;
        }
                
        while(!timer_istimedout(uwSysTimers100ns,uwTime)); // wait until 125us
        U0C1_TRBSCR = 0x4000; // Flush RX FIFO
    }
    else
        bRetVal = TRUE; // if write command, return true
#endif
    return bRetVal;
}

//***************************************************************************
// realtime task

static BOOL rttask(void)
{
    BOOL  bWrite=FALSE;
    UBYTE ubAddress;
    UWORD uwCt;
    BOOL bReceiveCheck;
    static UWORD uwCommCnt=0;

    uwCommCnt++;
    
    if(sRuntime.uwRTActCnt==0 && !(uwCommCnt&TRANSACTION_PERIOD_125US))
    {
            // Receive RUN Frame
        if(!sRuntime.sErrorSent.b.bAlarmSent)
        {
                // get result triggered from last transaction
            for(uwCt=0;uwCt<PROT_RUN_SIZE;uwCt++)
#ifdef _INFINEON_
                psucomm.w[uwCt]=U0C1_OUTRL;
#else
            ;
#endif
            bReceiveCheck =(psucomm.fm.uwHeader==PROT_SETUP_HEADER);
            bReceiveCheck&=((psucomm.fm.uwCmd&PROT_MODE_MSK)==PROT_RUNMODE);
            bReceiveCheck&=(psucomm.fm.ulWDT==(sRuntime.ulWatchDogCnt-1));  // check watchdog
            
            if(!bReceiveCheck)
            {
                if(++sRuntime.uwWDFailCnt>PROT_MAXWDTFAIL)
                {
                        // watchdog detected, send alarm, emcy stop and permanently disable PSU board
                    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_SOFTWARE_FAIL, SYSTEMALARMS_SUBCODE_SF_TM_PSU_WDT, TRUE);
                    (*sPSUBrdIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL) ;
                    sRuntime.sErrorSent.b.bAlarmSent=TRUE;
                }
            }
            else 
            {
                if(sRuntime.uwWDFailCnt>0)
                    sRuntime.uwWDFailCnt--;
            
                // Receive one value @125us
                switch(PROT_SETUP_ADDRESS(psucomm.fm.uwCmd))
                {
                    case ADR_VDCPEAK: sPSUCommData.swVoltageDcLinkPeak= psucomm.w[2]; break;
                    case ADR_IDCPEAK: sPSUCommData.swCurrentDcLinkPeak= psucomm.w[2]; break;
                    case ADR_VR:      sPSUCommData.swVoltagePhaseR    = psucomm.w[2]; break;
                    case ADR_VS:      sPSUCommData.swVoltagePhaseS    = psucomm.w[2]; break;
                    case ADR_VT:      sPSUCommData.swVoltagePhaseT    = psucomm.w[2]; break;
                    case ADR_VRS:     sPSUCommData.swVoltageRS        = psucomm.w[2]; break;
                    case ADR_VST:     sPSUCommData.swVoltageST        = psucomm.w[2]; break;
                    case ADR_VTR:     sPSUCommData.swVoltageTR        = psucomm.w[2]; break;
		            case ADR_SYSTATE: sPSUCommData.uwSysState         = psucomm.w[2]; break;		 
                    case ADR_POWEROK: sPSUCommData.uwPowerOK          = psucomm.w[2]; break;
                    case ADR_INPUT_EN:sPSUCommData.uwInputEnable      = psucomm.w[2]; break;
                    case ADR_READY:   sPSUCommData.uwReady            = psucomm.w[2]; break;
                    default:;
                }
                
                // Receive all values @125us
                sPSUCommData.uwAlarmCode=psucomm.w[6];
                sPSUCommData.uwWarnCode =psucomm.w[7];
                sPSUCommData.slVoltageDcLink=psucomm.fm.RGData.l[1];
                sPSUCommData.slCurrentDcLink=psucomm.fm.RGData.l[2];
                sPSUCommData.flTempBridge=psucomm.fm.RGData.f[3];
                sPSUCommData.flTempBrake=psucomm.fm.RGData.f[4];
            }

#ifdef _INFINEON_
            U0C1_TRBSCR = 0x4000; // Flush RX FIFO
#endif
        }
        else
        {
#ifdef _INFINEON_
            U0C1_TRBSCR = 0x4000; // Flush RX FIFO
#endif
            sRuntime.uwWDFailCnt=0;
        }
    
            // Transmit RUN Frame
        psucomm.fm.uwHeader=PROT_SETUP_HEADER; // #0
        if(!sRuntime.sErrorSent.b.bAlarmSent)
        {
            if(sRuntime.ubRTCmdWrite)
            {
                psucomm.fm.uwCmd=PROT_RUNMODE|PROT_SETUP_WRITE|PROT_SETUP_ADDRESS(sRuntime.ubRTCmdAddr);
                switch(sRuntime.ubRTCmdAddr)
                {
                    case ADR_BRAKETHD: 
                        psucomm.w[2]=(UWORD)sShadowPSUParam.VBrake.w.hi;
                        psucomm.w[3]=(UWORD)sShadowPSUParam.VBrake.w.lo; 
                        break;
                    case ADR_POWERON:  
                        psucomm.w[2]=0x0000;
                        psucomm.w[3]=(UWORD)sShadowPSUParam.flags.f.bPowerOn; 
                        break;
                    case ADR_COOLTEMP:
                        psucomm.w[2]=(UWORD)sShadowPSUParam.CoolTemp.w.hi;
                        psucomm.w[3]=(UWORD)sShadowPSUParam.CoolTemp.w.lo;
                        break;
                    case ADR_OVERBVOLTHRESHOLD:
                        psucomm.w[2]=(UWORD)sShadowPSUParam.VolThreshold.w.hi;
                        psucomm.w[3]=(UWORD)sShadowPSUParam.VolThreshold.w.lo;    //checkcode
                        break;
                    default:;
                }
                sRuntime.ubRTCmdWrite = FALSE; // reset write flag
            }
            else
            {
                switch(uwCommCnt%12)
                {
                    case 0:  ubAddress=ADR_VDCPEAK; break;
                    case 1:  ubAddress=ADR_IDCPEAK; break;
                    case 2:  ubAddress=ADR_VR;      break;
                    case 3:  ubAddress=ADR_VS;      break;
                    case 4:  ubAddress=ADR_VT;      break;
                    case 5:  ubAddress=ADR_VRS;     break;
                    case 6:  ubAddress=ADR_VST;     break;
                    case 7:  ubAddress=ADR_VTR;     break;
                    case 8:  ubAddress=ADR_SYSTATE; break;
                    case 9:  ubAddress=ADR_POWEROK; break;
                    case 10: ubAddress=ADR_INPUT_EN;break;
                    case 11: ubAddress=ADR_READY;   break;
                    default: ubAddress=ADR_SYSTATE;
                }
                psucomm.fm.uwCmd=PROT_RUNMODE|PROT_SETUP_READ|PROT_SETUP_ADDRESS(ubAddress);
                psucomm.w[2]=0x0000;psucomm.w[3]=0x0000;
            }            
            psucomm.fm.ulWDT=(++sRuntime.ulWatchDogCnt);
        }
        else
        {
            psucomm.fm.uwCmd=PROT_RUNMODE;
            psucomm.w[2]=0x0000;psucomm.w[3]=0x0000;
            psucomm.fm.ulWDT=(++sRuntime.ulWatchDogCnt);
        }
                
        for(uwCt=0;uwCt<6;uwCt++)
#ifdef _INFINEON_
            U0C1_IN00=psucomm.w[uwCt];
#else
            ;
#endif       

        for(uwCt=0;uwCt<PROT_RUN_SIZE-6;uwCt++)
#ifdef _INFINEON_
            U0C1_IN00=0x0000;       // #6 ~ #15
#else
            ;
#endif
    }
    
        // update activation counter
    if(sRuntime.uwRTActCnt)
        sRuntime.uwRTActCnt--;
        
    return TRUE;
}

//***************************************************************************
// Live parameters check and activation

static void liveParametersCheck(void)
{
    PSU_PARAM sLocParam;
    SWORD swVBrakeLow,swVBrakeHigh,swCoolingTempOn,swCoolingTempOff;
	SWORD swOverVoltageThreshold;
    
    // copy parameters
    atomic_read(&sLocParam, &sPSUInParam, sizeof(PSU_PARAM));
    
    // _atomic_(0);
    swVBrakeLow=sMh_MotorDataParam.swVBrakeLow;
    swVBrakeHigh=sMh_MotorDataParam.swVBrakeHigh;
    // _endatomic_();
    
    // _atomic_(0);
    swCoolingTempOn=sTm_ThModParam.swCoolingTempOn;
    swCoolingTempOff=sTm_ThModParam.swCoolingTempOff;
    // _endatomic_();

	// _atomic_(0);
	swOverVoltageThreshold = sMh_MotorDataParam.swOverVoltageThreshold;
	// _endatomic_();

    // Check brake threshold
    if(swVBrakeLow < 0 || swVBrakeHigh < 0 || swVBrakeLow > swVBrakeHigh)
    {
        ParChk_SignalValueError(PARCC_MH_BRAKEVOLTAGERANGE);
        swVBrakeLow  = DEFAULT_PSUBRAKELOW*10;
        swVBrakeHigh = DEFAULT_PSUBRAKEHIGH*10;
    }
    else
    {
        ParChk_ResetValueError(PARCC_MH_BRAKEVOLTAGERANGE);
        if(swVBrakeLow==0 || swVBrakeHigh==0)
        {
                // setup default values
            swVBrakeLow  = DEFAULT_PSUBRAKELOW*10;
            swVBrakeHigh = DEFAULT_PSUBRAKEHIGH*10;
        }
        if(!sRuntime.ubRTCmdWrite && (swVBrakeLow!=sShadowPSUParam.VBrake.w.lo || swVBrakeHigh!=sShadowPSUParam.VBrake.w.hi))
        {
            sRuntime.ubRTCmdWrite=TRUE;
            sRuntime.ubRTCmdAddr=ADR_BRAKETHD;
            
            sShadowPSUParam.VBrake.w.lo = swVBrakeLow;
            sShadowPSUParam.VBrake.w.hi = swVBrakeHigh;
        }        
    }

    sMh_MotorDataOut.swActiveVBrakeLow  = swVBrakeLow;
    sMh_MotorDataOut.swActiveVBrakeHigh = swVBrakeHigh;
    
    
    // Check cooling temerature
    if(swCoolingTempOn < swCoolingTempOff)
    {
        ParChk_SignalValueError(PARCC_TM_INVALIDCOOLINGTHRESHOLDS);
    }
    else
    {
        ParChk_ResetValueError(PARCC_TM_INVALIDCOOLINGTHRESHOLDS);
        if(!sRuntime.ubRTCmdWrite && (swCoolingTempOn!=sShadowPSUParam.CoolTemp.w.hi || swCoolingTempOff!=sShadowPSUParam.CoolTemp.w.lo))
        {
            sRuntime.ubRTCmdWrite=TRUE;
            sRuntime.ubRTCmdAddr=ADR_COOLTEMP;
            
            sShadowPSUParam.CoolTemp.w.hi = swCoolingTempOn;
            sShadowPSUParam.CoolTemp.w.lo = swCoolingTempOff;
        }
    }

    // Power ON
    if(!sRuntime.ubRTCmdWrite && (sLocParam.flags.f.bPowerOn!=sShadowPSUParam.flags.f.bPowerOn))
    {
        sRuntime.ubRTCmdWrite=TRUE;
        sRuntime.ubRTCmdAddr=ADR_POWERON;
        
        sShadowPSUParam.flags.f.bPowerOn = sLocParam.flags.f.bPowerOn;
    }   

	    // voltage threshold
    if(swOverVoltageThreshold<0)
    {
        ParChk_SignalValueError(PARCC_MH_OVERVOLTAGE);
        sMh_MotorDataOut.sDriveLimit.swOverVoltage = sMotorHandlerRun.swMaxOverVoltage;
    }
    else
    {
        ParChk_ResetValueError(PARCC_MH_OVERVOLTAGE);
        if(swOverVoltageThreshold == 0)
        {
            // setup default values
            swOverVoltageThreshold = sMotorHandlerRun.swMaxOverVoltage;
        }
        
        if (swOverVoltageThreshold > sMotorHandlerRun.swMaxOverVoltage)
            sMh_MotorDataOut.sDriveLimit.swOverVoltage = sMotorHandlerRun.swMaxOverVoltage ;
        else
            sMh_MotorDataOut.sDriveLimit.swOverVoltage = swOverVoltageThreshold ;
        
        if(!sRuntime.ubRTCmdWrite && (swOverVoltageThreshold != sShadowPSUParam.VolThreshold.w.hi))
        {
            sRuntime.ubRTCmdWrite=TRUE;
            sRuntime.ubRTCmdAddr=ADR_OVERBVOLTHRESHOLD;
            
            sShadowPSUParam.VolThreshold.w.hi=swOverVoltageThreshold;
            sShadowPSUParam.VolThreshold.w.lo=PSU_CHECK_CODE;
        }
    }
}
#endif // cfg_psu_mngr
#endif // _hw_dc
