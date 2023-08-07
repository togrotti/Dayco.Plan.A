/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ModBusCommandMgr.c                                         */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : ModBus/Cockpit specific command manager                    */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

//#include <xe167f.h>

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "drive\AxM-E-Defines.h"
#include "system\SysAppGlobals.h"
#include "system\SysLogManagement.h"
#include "system\SysLogData.h"
#include "common\ParamStorageManagement.h"
#include "common\TaskScheduler.h"
#include "system\Os.h"
#include "ModbusAppProtocol.h"
#include "ModbusOverSerial.h"
#include "core\SerialPorts.h"
#include "drive\HardwareConfig.h"
#include "core\SystemReset.h"
#include "system\GlobalResetCodes.h"
#include "plc\Plc.h"
#include "plc\PlcRT.h"

//#define DEFINE_EXTERNALS
#include "ModBusCommandMgr.h"
//#undef DEFINE_EXTERNALS

#include "ModBusCommandMgrParams.h"

#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"

#ifdef _AXX_SYSAPP
#ifdef CFG_EN_MODBUSOVERCAN
#include "ModbusOverCAN.h"
#endif
//#include "BootBlockInfoCodes.h"
//#include "BootBlockExtension.h"
#include "system\HeaderFooterInfo.h"
#endif

#include <string.h>

#include "ModbusOverEth.h"

#include "system\SysAppFpgaOptCodes.h"
#include "drive\HardwareUnitID.h"

#include "FreeRTOS.h"
#include "task.h"

#ifdef CFG_EN_MODBUSOVERUSB
#include "ModbusOverUsb.h"
#endif

#include "system\SysAppSFlashPartZYNQ.h"

//***************************************************************************
// Avoids warning C37: nonstandard extension - unnamed parameter

//#pragma warning disable = 37

//***************************************************************************
// Global parameters

#ifndef _HW_DC
#define _LOC_DEFBAUDRATE        38400
#else
//#define _LOC_DEFBAUDRATE        460800
//#define _LOC_DEFBAUDRATE        115200
#define _LOC_DEFBAUDRATE        38400
#endif

#ifndef _HW_DC
MODBUS_PROTOCOL_OPTION sModBusCMParams;
const MODBUS_PROTOCOL_OPTION sModBusCMDefParams=
#else
const MODBUS_PROTOCOL_OPTION sModBusCMParams=
#endif
{
    1,0
};

#ifndef _HW_DC
MODBUS_OVERSER_DRIVER_SETUP sModBusCMParOverserial0;
const MODBUS_OVERSER_DRIVER_SETUP  sModBusCMDefParOverserial0=
#else
const MODBUS_OVERSER_DRIVER_SETUP sModBusCMParOverserial0=
#endif
{ 
    { { 1, 1 }, 1 },
    { _LOC_DEFBAUDRATE, 8, 0, 1, 0x0000, 0 },
    0,
};

#ifdef CFG_EN_MODBUSOVERCAN
static const MODBUSOVERCAN_PARAMS sModBusCMParOverCAN=
{
    1,                      // CAN node
    CANDRV_500KBPS,         // speed
    0x07f0,
    0x07f1
};    
#endif

MODBUS_OVERETH_DRIVER_SETUP sModBusCMParOverEth;
const MODBUS_OVERETH_DRIVER_SETUP  sModBusCMDefParOverEth=
{
   { 0, 1 }, // Drive Parameter: {flags, port selection}
   { ETHPROT_TCP, {192,168,0,1}, 502 }  // Protocol Setup: {protocol, ip address, port number}
};

#ifdef CFG_EN_MODBUSOVERUSB
static const MODBUS_OVERUSB_DRIVER_SETUP sDefaultCMParOverUsb=
{
    { { 1 } },
};
#endif

static const MODBUS_PROTOCOL_DRIVER sProtocolsSetup[]=
{ 
    { MODBUS_OVER_SERIAL_1, (HPVOID)&sModBusCMParOverserial0 },

#ifdef CFG_EN_MODBUSOVERETH
    { MODBUS_OVER_ETHERNET, (HPVOID)&sModBusCMParOverEth },
#endif

#ifdef CFG_EN_MODBUSOVERUSB
    { MODBUS_OVER_USB_PORT, (HPVOID)&sDefaultCMParOverUsb },
#endif

#ifdef CFG_EN_MODBUSOVERCAN
    { MODBUS_OVER_CAN, (HPVOID)&sModBusCMParOverCAN },
#else
    { 0x0000, NULL },
#endif
};

//***************************************************************************
// Default startup for safe start

static const MODBUS_PROTOCOL_OPTION sDefaultCMParams={1,0};
static const MODBUS_OVERSER_DRIVER_SETUP sDefaultCMParOverserial0=
{ 
    { { 1, 1 }, 1 },
    { _LOC_DEFBAUDRATE, 8, 0, 1, 0x0000, 0 },
    0,
};

static const MODBUS_PROTOCOL_DRIVER sDefaultProtocolsSetup[]=
{ 
    { MODBUS_OVER_SERIAL_1, (HPVOID)&sDefaultCMParOverserial0 },
    { 0x0000, NULL },
};

//***************************************************************************
// Globals

UWORD  hpuwModBusCMStatList[MODBUSCM_STATUSLIST_DEPTH+3];
UWORD  uwModBusCMStatListCount;
UWORD  uwModBusCMImmRestore=0;
UWORD  uwModBusCMEnterDiagnosticApp=0;
HEADER_FOOTER_INFO  sModBusCMBootBlockInfo;
ULONG  ulModBusCMBootBlockSysId;
ULONG  ulModBusCMBootBlockAppId;
MODBUSSMSYSTEMFLAGBITS sModBusSMSystemFlagBits={0};

//***************************************************************************
// Locals

static MODBUS_OVERSER_DRIVER_SETUP  * hpsUsedSerialParams;
static ULONG ulLastAlarmTStamp;
static ULONG ulLastAlarmTSReset;

#ifdef _AXX_SYSAPP
#ifdef CFG_EN_MODBUSOVERCAN
static UWORD uwRDDetectedTOut;
#endif
#endif

//***************************************************************************
// Local defines

#define MODBUS_WAIT_FOR_RESET           250     // msec
//#define SLOWTASK_STACKSIZE              (OS_DEFAULTUSRSTATICSTACK+0x0070)
#define SLOWTASK_STACKSIZE              512

#define REMDISP_DETECT_TIMEOUT          2000    // msec

//***************************************************************************
// Local functions prototypes

static BOOL parameterscheck(void);
static void statuslistsetup(void);
static void slowtaskmanager(void);

//****************************************************************************
// Init handler

BOOL ModBusCM_Init(UWORD uwParam)
{
        // read bootblock info, if supported
#ifdef _INFINEON_
    ULONG ulKeyAddress=(ULONG)&BootBlockExtendedTagId;
    
    if(*((HPULONG)ulKeyAddress) == BBEXT_KEY)
    {
        sModBusCMBootBlockInfo.uwApplicatType=(UWORD)BootBlockGetInfo(BBINFOCODE_APPTYPE);
        sModBusCMBootBlockInfo.uwVersionMajor=(UWORD)BootBlockGetInfo(BBINFOCODE_VERSIONMAJOR);
        sModBusCMBootBlockInfo.uwVersionMinor=(UWORD)BootBlockGetInfo(BBINFOCODE_VERSIONMINOR);
        sModBusCMBootBlockInfo.uwBuildNumber=(UWORD)BootBlockGetInfo(BBINFOCODE_BUILD);

        *(HPULONG)(&sModBusCMBootBlockInfo.chBlockDescrText[0])=BootBlockGetInfo(BBINFOCODE_DESCR_0);
        *(HPULONG)(&sModBusCMBootBlockInfo.chBlockDescrText[4])=BootBlockGetInfo(BBINFOCODE_DESCR_1);
        *(HPULONG)(&sModBusCMBootBlockInfo.chBlockDescrText[8])=BootBlockGetInfo(BBINFOCODE_DESCR_2);
        *(HPULONG)(&sModBusCMBootBlockInfo.chBlockDescrText[12])=BootBlockGetInfo(BBINFOCODE_DESCR_3);

        ulModBusCMBootBlockSysId=BootBlockGetInfo(BBINFOCODE_SYSID);
        ulModBusCMBootBlockAppId=BootBlockGetInfo(BBINFOCODE_APPID);
    }
#else
    parmgm_bootblock_load(&sModBusCMBootBlockInfo, &ulModBusCMBootBlockSysId, &ulModBusCMBootBlockAppId);
#endif

    memset(&sModBusSMSystemFlagBits, 0, sizeof(sModBusSMSystemFlagBits));

        // alarm timestamp initialization
    atomic_read(&ulLastAlarmTStamp, &ulSysTimersTotalPowerOnTime, sizeof(ulSysTimersTotalPowerOnTime));

        // force first scan by decrementing last valid timestamp
    ulLastAlarmTStamp--;
    ulLastAlarmTSReset=ulLastAlarmTStamp;

    SerialPortsInit();

    if(uwParam)
    {
            // safe startup
        ModbusAppProtInit((MODBUS_PROTOCOL_OPTION *)&sDefaultCMParams, (MODBUS_PROTOCOL_DRIVER *)sDefaultProtocolsSetup);
        hpsUsedSerialParams=(MODBUS_OVERSER_DRIVER_SETUP  *)sDefaultProtocolsSetup[0].hpvSetupPrms;
    }
    else
    {
            // normal startup
        if(!parameterscheck())
            return FALSE;

        ModbusAppProtInit((MODBUS_PROTOCOL_OPTION *)&sModBusCMParams, (MODBUS_PROTOCOL_DRIVER *)sProtocolsSetup);
        hpsUsedSerialParams=(MODBUS_OVERSER_DRIVER_SETUP  *)sProtocolsSetup[0].hpvSetupPrms;
    }

        // if diagnostic application is requested then signal to external
    if(uwSystemEnterDiagnostic==SYSTEM_ENTERDIAGNOSTIC_KEY)
        uwModBusCMEnterDiagnosticApp=MODBUSCM_ENTERDIAGNOSTICAPP_KEY;

    (void)Os_TaskCreateEx(&slowtaskmanager,SLOWTASK_STACKSIZE,"ModbusCM");

    return TRUE;
}

//***************************************************************************
// Hardware option callback

void ModbusCM_GetHwOpt(UWORD t, ULONG * pulHwOpt, ULONG * pulFPGAOpt)
{
    if(sModBusCMParOverEth.stDrvParams.flags.b.ubDriverEnable)
    {
        if(sModBusCMParOverEth.stDrvParams.ubEthPortSel==1 || sModBusCMParOverEth.stDrvParams.ubEthPortSel==2)
            *pulHwOpt  |=HWUNITID_ETH_DBLRMII_1;
        else if(sModBusCMParOverEth.stDrvParams.ubEthPortSel==3 || sModBusCMParOverEth.stDrvParams.ubEthPortSel==4)
            *pulHwOpt  |=HWUNITID_ETH_DBLRMII_0;
            
        *pulFPGAOpt|=FPGA_HW_ETHERNET;
    }
    
    *pulFPGAOpt|=0;
}

//***************************************************************************
// Parameters checking

static BOOL parameterscheck(void)
{
    BOOL bValid=TRUE;

    if(sModBusCMParOverserial0.stDrvParams.ubSlaveAddress==0 || sModBusCMParOverserial0.stDrvParams.ubSlaveAddress>127)
    {
        ParChk_SignalValueError(PARCC_MODBUS_INVSLAVEADDRESS);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_MODBUS_INVSLAVEADDRESS);

#ifdef _INFINEON_
    if(SerialPortGetTimingSetup(sModBusCMParOverserial0.stPortSetup.ulBaudRate)==NULL)
    {
        ParChk_SignalValueError(PARCC_MODBUS_INVBAUDRATE);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_MODBUS_INVBAUDRATE);
#endif

    if(sModBusCMParOverserial0.stPortSetup.uwDataBits!=SERIALPORTS_DATABITS_8)
    {
        ParChk_SignalValueError(PARCC_MODBUS_INVDATABITS);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_MODBUS_INVDATABITS);

    if( sModBusCMParOverserial0.stPortSetup.uwParityMode != SERIALPORTS_NOPARITY && \
        sModBusCMParOverserial0.stPortSetup.uwParityMode != SERIALPORTS_EVENPARITY && \
        sModBusCMParOverserial0.stPortSetup.uwParityMode != SERIALPORTS_ODDPARITY)
    {
        ParChk_SignalValueError(PARCC_MODBUS_INVPARITY);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_MODBUS_INVPARITY);

    if( sModBusCMParOverserial0.stPortSetup.uwStopBits != SERIALPORTS_ONESTOPBIT && \
        sModBusCMParOverserial0.stPortSetup.uwStopBits != SERIALPORTS_TWOSTOPBITS )
    {
        ParChk_SignalValueError(PARCC_MODBUS_INVSTOP);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_MODBUS_INVSTOP);

    return bValid;
}

//***************************************************************************
// Alarm, warning and parameters check list setup

static void statuslistsetup(void)
{
    static ULONG  ulLastAlarmMask=0ul;
    static ULONG  ulLastWarningMask=0ul;
    static ULONG  ulLastParChkTStamp=0ul;
    UWORD  * hpuwList;
    UWORD uwElMask=0;

        // check for any changes, if no changes then exit immediately
        // for alarms change is checked timestamp for alarm adding and
        // alarm mask for alarm resetting
    if(ulLastAlarmMask<=ulSystemAlarms && ulLastAlarmTStamp==ulSysLogDataLastAlarmTime && \
        ulLastWarningMask==ulSystemWarnings \
        && ulLastParChkTStamp==ulParChkLastUpdateTime
        )
        return;

        // reset actual list and recompile it from scratch
    uwModBusCMStatListCount=0;
    hpuwList=hpuwModBusCMStatList;

        // Alarms
    {
        SYSLOGMGM_ALARMLOG tLog;
        ULONG ulLocAlarms,ulTime;
        UWORD uwCt;
    
            // atomically get both actual alarm mask and last alarm time
        DISABLE_IRQ();
        ulLocAlarms=ulSystemAlarms;
        ulTime=ulSysLogDataLastAlarmTime;
        RESTORE_IRQ();

            // if previous are alarms and actual zero then alarm
            // reset has occurred
        if(ulLastAlarmMask!=0l && ulLocAlarms==0l)
        {
                // then set time at alarm reset
            ulLastAlarmTSReset=ulLastAlarmTStamp=ulTime;
    
                // and reset actual alarm mask
            ulLastAlarmMask=0l;
        }
            // otherwise new alarms are active, then read list
        else
        {
                // reset actual alarm mask
            ulLastAlarmMask=ulLocAlarms;

                // set time at last alarm
            ulLastAlarmTStamp=ulTime;

                // check last time reset, could not be future rather than ulTime
            if(ulLastAlarmTSReset>ulTime)
                ulLastAlarmTSReset=ulTime;
    
                // begin scanning from last alarm reset
            ulTime=ulLastAlarmTSReset+1;

                // iterate each alarm
            while(SysLogData_GetFromAlarmHistory(ulTime, &tLog))
            {
                    // set for next alarm
                ulTime=tLog.ulAbsoluteTime+1;

                    // get bit number
                uwCt=0;
                while(!(tLog.ulAlarm&0x00000001l))
                {
                    tLog.ulAlarm>>=1;
                    uwCt++;
                }

                    // fill data structure
                *hpuwList++=MODBUSCM_STATUSLIST_ALARM|0x0300|uwCt;
                *hpuwList++=(UWORD)(tLog.ulAlarmSubCode&0x0000ffff);
                *hpuwList++=(UWORD)(tLog.ulAlarmSubCode>>16);
                uwElMask|=MODBUSCM_STATUSLIST_ALARM;
                uwModBusCMStatListCount+=3;
            
                    // check list size
                if(uwModBusCMStatListCount>=MODBUSCM_STATUSLIST_DEPTH)
                {
                        // avoid scan of warnings and par check list, then exit
                    ulLastWarningMask=ulSystemWarnings;
                    ulLastParChkTStamp=ulParChkLastUpdateTime;

                    goto fine;
                }
            }
        }
    }

        // Warnings
    {
        ULONG ulLocWarnings;
        UWORD uwCt;

            // atomically get actual warning mask
        atomic_read(&ulLocWarnings, &ulSystemWarnings, sizeof(ulSystemWarnings));
        ulLastWarningMask=ulLocWarnings;

            // scan warnings
        uwCt=0;
        while(ulLocWarnings)
        {
            if(ulLocWarnings&0x00000001l)
            {
                    // fill data structure
                *hpuwList++=MODBUSCM_STATUSLIST_WARNING|0x0100|uwCt;
                uwElMask|=MODBUSCM_STATUSLIST_WARNING;
                uwModBusCMStatListCount+=1;
            
                    // check list size
                if(uwModBusCMStatListCount>=MODBUSCM_STATUSLIST_DEPTH)
                {
                        // avoid scan of par check list, then exit
                    ulLastParChkTStamp=ulParChkLastUpdateTime;

                    goto fine;
                }
            }
            
            ulLocWarnings>>=1;
            uwCt++;
        }
    }        

        // Parameter check list
    {
        UWORD uwList[MODBUSCM_STATUSLIST_DEPTH/2];
        UWORD uwCnt,uwCt;

            // atomically get actual timestamp of last modification
        atomic_read(&ulLastParChkTStamp, &ulParChkLastUpdateTime, sizeof(ulParChkLastUpdateTime));

            // get list
        uwCnt=ParChk_GetValueErrorList(uwList, MODBUSCM_STATUSLIST_DEPTH/2);

            // fill data structure
        for(uwCt=0;uwCt<uwCnt;uwCt++)
        {
            if(uwList[uwCt]<PARCC_GROUPTHRESHOLD)
            {
                *hpuwList++=MODBUSCM_STATUSLIST_PARGRPCHECK|0x0200;
                uwElMask|=MODBUSCM_STATUSLIST_PARGRPCHECK;
            }
            else
            {
                *hpuwList++=MODBUSCM_STATUSLIST_PARCHECK|0x0200;
                uwElMask|=MODBUSCM_STATUSLIST_PARCHECK;
            }
            *hpuwList++=uwList[uwCt];
            uwModBusCMStatListCount+=2;
        
                // check list size
            if(uwModBusCMStatListCount>=MODBUSCM_STATUSLIST_DEPTH)
                break;
        }
    }

fine:
    uwModBusCMStatListCount|=uwElMask;
}

//***************************************************************************
// task for modbus protocol and command processing

static void slowtaskmanager(void)
{
    for(;;)
    {
        if(bSysStatBooting)
        {
            Os_Sleep(0);
            continue;
        }

            // ModBus protocol handler
        (void)ModbusAppProtHandleRequests();

            // if new packet received from remote display then remote display is attached
            // such condition will force serial to 422/485 for proper 422/485 remote display attached master
#ifdef _AXX_SYSAPP
#ifdef CFG_EN_MODBUSOVERCAN
        if(bMOCRxValid)
        {
            bMOCRxValid=FALSE;

                // start timer which timeout means remote display detached
            uwRDDetectedTOut=timer_settimeout(uwOsFreeRunTimer1kHz, REMDISP_DETECT_TIMEOUT);

                // force serial driver to 422/485
            SerialPortForce422485(TRUE);
        }

            // if timeout then switch back serial driver to standard management
        if(timer_istimedout(uwOsFreeRunTimer1kHz, uwRDDetectedTOut))
            SerialPortForce422485(FALSE);
#endif
#endif

            // Software Drive Reset or enter diagnostic application (if not already in)
        if(bModBusSMSoftwareDriveReset
#if CFG_PLC_DIAGNOSTIC
            || uwModBusCMEnterDiagnosticApp==MODBUSCM_ENTERDIAGNOSTICAPP_KEY && uwSystemEnterDiagnostic!=SYSTEM_ENTERDIAGNOSTIC_KEY
#endif
#ifdef _AXX_SYSAPP
#ifdef _APP_XC
            || bModBusSMEnterBootAndUpgrade
#endif
#endif
            )
        {
                // resetting flag and syslog flush flag
            bSysStatResetting=TRUE;
            bSysStatSysLogFlush=TRUE;

                // Delay to avoid Modbus response dropped
            timer_wait(uwOsFreeRunTimer1kHz, (MODBUS_WAIT_FOR_RESET>SW_RESET_DELAY?MODBUS_WAIT_FOR_RESET:SW_RESET_DELAY));

                // wait until syslog is safely written
            while(bSysStatSysLogFlush);

                // then reset
            if(bModBusSMSoftwareDriveReset)
#ifdef _APP_DEBUG
                SysRes_ExecuteReset(RESETCODE_CLEANREBOOTREQUESTED, ~RESETCODE_CLEANREBOOTREQUESTED);
#else
                SysRes_ExecuteReset(SYSRES_POWERON_VALUE_PAR0, SYSRES_POWERON_VALUE_PAR1);
#endif // _app_debug
#ifdef _AXX_SYSAPP
#ifdef _APP_XC
            else if(bModBusSMEnterBootAndUpgrade)
                SysRes_ExecuteReset(RESETCODE_ENTERBOOTANDUPGRADE, ~RESETCODE_ENTERBOOTANDUPGRADE);
#endif // _app_xc
#endif // _app_sysapp
            else
                SysRes_ExecuteReset(RESETCODE_ENTERDIAGNOSTICAPPLICATION,~RESETCODE_ENTERDIAGNOSTICAPPLICATION);
        }

            // Software Drive Reset and Enter bootblock, only if power disabled
        if(bModBusSMResetAndEnterBootBlock)
        {
            if(!bSysStatPowerEnabled)
            {
                UWORD uwSerialParams;
    
                    // Encode actual port parameters
                if(ModbusOverSerMinimalParamsEnc(hpsUsedSerialParams, &uwSerialParams) == 0)
                {
                        // resetting flag and syslog flush flag
                    bSysStatResetting=TRUE;
                    bSysStatSysLogFlush=TRUE;
    
                        // Delay to avoid Modbus response dropped
                    timer_wait(uwOsFreeRunTimer1kHz, (MODBUS_WAIT_FOR_RESET>SW_RESET_DELAY?MODBUS_WAIT_FOR_RESET:SW_RESET_DELAY));
    
                        // wait until syslog is safely written
                    while(bSysStatSysLogFlush);

                        // switch to bootblock
                    SysRes_ModifyMultiBoot(SFPART_BOOTBLOCK_START);

                        // then reset
                    SysRes_ExecuteReset(RESETCODE_ENTERBOOTWMODBUSSER0, uwSerialParams);
                }
            }

                // if here reset flag
            bModBusSMResetAndEnterBootBlock=FALSE;
        }

            // Save System Parameters
        if(bModBusSMSaveSystParameters)
        {
                // execute save
            if(parmgm_par_save()!=PARMGM_B_OK)
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_HW_FLASH_FAIL, SYSTEMALARMS_SUBCODE_HF_FLASH_PARAMS, FALSE);
    
                // then reset flag
            bModBusSMSaveSystParameters=FALSE;
        }

            // Save Assembly Info only
        if(bModBusSMSaveHwAssemblyInfo)
        {
#ifndef _HW_DC
                // execute save
            if(!HwConfStore())
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_HW_FLASH_FAIL, SYSTEMALARMS_SUBCODE_HF_FLASH_HWCONFIG, FALSE);
#endif
    
                // then reset flag
            bModBusSMSaveHwAssemblyInfo=FALSE;
        }

            // Restore default parameters
        if(bModBusSMRestoreDefaultParams)
        {
                //execute restore
            if(parmgm_par_restore()!=PARMGM_B_OK)
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_HW_FLASH_FAIL, SYSTEMALARMS_SUBCODE_HF_FLASH_PARAMS, FALSE);
    
                // then reset flag
            bModBusSMRestoreDefaultParams=FALSE;
        }

            // Immediate restore default parameters
        if(uwModBusCMImmRestore==MODBUSCM_IMMRESTORE_KEY)
        {
                // check if possible to lock drive, then lock without unsetting common db app
            if(!PlcStandByRequired())
                if(PlcLockForReload(TRUE))
                        // drive disabled and locked, execute restore
                    if(parmgm_par_immrestore()==PARMGM_B_OK)
                        uwModBusCMImmRestore=0;

                // if not zero, then signal error (-1)
            if(uwModBusCMImmRestore)
                uwModBusCMImmRestore=-1;
        }

            // parameter checking
        parameterscheck();

            // setup alarms, warnings and parameters check list for cockpit, sinchronous with protocol handler
        statuslistsetup();

            // yield control to OS
//        Os_Sleep(0);
        vTaskDelay(0);
    }
}

//***************************************************************************
// Status list array reading

UWORD ModBusCM_StatListRead(COMMONPARAMDB_ENTRY  *t, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext)
{
        // if abort do nothing
    if(uwFlags&COMMONPARAMDB_CBFLAG_ABORT)
        return COMMONPARAMDB_CH_OK;

        // check element access
    if(uwElement!=0)
        return COMMONPARAMDB_CH_INVALID_ELEMENT;

        // if data read
    if(uwFlags&COMMONPARAMDB_CBFLAG_RD)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
                // left data size
            *hpulContext=uwModBusCMStatListCount*sizeof(UWORD);

                // data size if requested
            if(uwFlags&COMMONPARAMDB_CBFLAG_SIZEINQUIRY)
            {
                *((HPULONG)hpvBuffer)=*hpulContext;
                *puwBufSize=sizeof(ULONG);
            }
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            UWORD uwSize;
            HPUWORD hpuwDataStart;

                // data size left
            uwSize=(UWORD)*hpulContext;

                // data chunk start
            hpuwDataStart=&hpuwModBusCMStatList[uwModBusCMStatListCount*sizeof(UWORD)-uwSize];

                // check dest buffer
            if(uwSize>*puwBufSize)
                uwSize=*puwBufSize;

                // copy data chunk
            memcpy(hpvBuffer, hpuwDataStart, uwSize);

                // update data out buffer size
            *puwBufSize=uwSize;

                // update left size
            *hpulContext=*hpulContext-(ULONG)uwSize;
        }
    }

        // if data write
    if(uwFlags&COMMONPARAMDB_CBFLAG_WR)
        return COMMONPARAMDB_CH_NO_WRITE_ACCESS;

    return COMMONPARAMDB_CH_OK;
}

