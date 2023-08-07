/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright � 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppParamTable.c                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : System application parameter table definition              */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include <stddef.h>
#include "common\CommonParamDB.h"
#include "system\SystemStatus.h"
#include "system\SysAppDataCodes.h"
#include "system\SysAppInfo.h"
#include "system\SysAppGlobals.h"
#include "system\SystemAlarms.h"
#include "system\SysLogData.h"
#include "core\SystemReset.h"
#include "drive\HardwareConfig.h"
#include "drive\UserIO.h"
#include "common\ParametersCheck.h"
#include "common\ParamStorageManagement.h"
#include "drive\PanelManager.h"
#include "bus\modbus\ModBusCommandMgr.h"
#include "bus\modbus\ModBusCommandMgrParams.h"
#include "bus\canopen\CanOpenDs301.h"
#include "bus\canopen\CanOpenDs301Externals.h"
#if CFG_CANDRV_DS301
#include "bus\canopen\CanOpenPdoSyncMgr.h"
#endif
#if CFG_CANDRV_CMDMGR
#include "bus\canopen\CanOpenCommandMgr.h"
#endif
#include "bus\canopen\CanOpenCOECommon.h"
#include "bus\ethercat\ECATCommandMgr.h"
#ifdef _HW_DC
#include "bus\ethpmc\EthPMCCommandMgr.h"
#endif
#include "bus\SyncManager.h"
#include "system\Os.h"
#include "common\TaskScheduler.h"
#include "fpga\FpgaHandler.h"
#include "drive\MotorParameters.h"
#include "plc\Plc.h"
#include "plc\PlcRT.h"
#include "plc\SoftScope.h"
#ifdef _INFINEON_
#include "RDHmiServer.h"
#endif

#include "drive\EncoderManager.h"
#include "drive\EncoderEFSeek.h"
#include "drive\EnDatEncoder.h"
#include "drive\HallEncoder.h"
#include "drive\HiperfaceEncoder.h"
#include "drive\IncrementalEncoder.h"
#include "drive\SincosEncoder.h"
#include "drive\BackEmfEncoder.h"
#include "drive\NikonEncoder.h"
#include "drive\TAMAGAWAEncoder.h"
#include "drive\BissEncoder.h"

#include "drive\SpaceSpeedCntrLp.h"
#include "drive\Positioner.h"

#include "drive\MotionController.h"
#include "drive\ThermalModel.h"
#include "drive\PiDcBusCntrLp32bit.h"
#include "drive\MotorHandler.h"
#include "drive\Deflux.h"
#include "common\CommonController.h"
#include "drive\DriveTaskController.h"

#include "common\ProgramFlashHandler.h"

#ifdef _HW_CT
#include "Sensor.h"
#endif

#ifdef _HW_AXS_CABI35KW
#include "drive\stgap4s.h"
#endif



//****************************************************************************
// Write Deny common definitions

#define WRDENY_NONE                 (0)
#define WRDENY_DEFAULT              (SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_RESETTING))
#define WRDENY_PARAMSAVE            (WRDENY_DEFAULT|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PARAMETERSSAVING))
#define WRDENY_FLASHCOMMAND         (WRDENY_DEFAULT|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_RESETTING))
#define WRDENY_POWERENABLED         (WRDENY_DEFAULT|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_POWER_ZEROELFLDSEEK)| \
                                        SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_POWER_ENABLED)| \
                                        SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_FAULTREACTION))
#define WRDENY_DS301PDOPARAMS       (WRDENY_PARAMSAVE|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PDOMGR_LOCKCONFIG))
#define WRDENY_ECATPDOPARAMS        (WRDENY_PARAMSAVE|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_ECATPDOMGR_LOCKCONFIG))

//****************************************************************************
// Size of plc arrays

#define PLC_PARCOM_SIZE             (sizeof(tPlcParamCommon.uwCommon)/sizeof(UWORD)>128?128:sizeof(tPlcParamCommon.uwCommon)/sizeof(UWORD))
#define PLC_PARBIT_SIZE             (sizeof(tPlcParamBits.uwBools)>128?128:sizeof(tPlcParamBits.uwBools))
#define PLC_WRKCOM_SIZE             (sizeof(uwPlcWorksCommon)/sizeof(UWORD)>128?128:sizeof(uwPlcWorksCommon)/sizeof(UWORD))
#define PLC_WRKBIT_SIZE             (sizeof(uwPlcWorksBools)>128?128:sizeof(uwPlcWorksBools))

//****************************************************************************
// Table def
// Rules for common db table:
// - first compile common db source before any other file that reference common db
// - processor recognizes each line beginning with { <hex> and take hex as reference
// - place in common db table only common db definition array and nothing else
// - in the file that reference common db place first #%n to define common db array name
// - processor substitute each #%p <hex> token with pointer to common db array

const COMMONPARAMDB_ENTRY psCommonParamTable[]=
{
        // Sys App Info
    {0x0100, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &sSysAppInfo.uwApplicatType, NULL},
    {0x0101, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &sSysAppInfo.uwVersionMajor, NULL},
    {0x0102, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &sSysAppInfo.uwVersionMinor, NULL},
    {0x0103, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, sizeof(sSysAppInfo.chBlockDescrText)/sizeof(ULONG),
            WRDENY_DEFAULT, (HPVOID)sSysAppInfo.chBlockDescrText, NULL},
    {0x0104, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&chFwApplicationId, NULL},
    {0x0105, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &sSysAppInfo.uwBuildNumber, NULL},
    {0x0108, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &tHwFpgaConfig.uwType, NULL},
    {0x0109, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &tHwFpgaConfig.uwBuild, NULL},
 #ifndef _HW_DC
    {0x010A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &FPGA_BUILD_APP, NULL},
 #endif
    {0x010B, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&tHwFpgaConfig.chApp, NULL},
    {0x010C, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_STRING , 0, 1,
            WRDENY_DEFAULT, (HPVOID)sSysAppInfo.chBlockDescrText, NULL},
    {0x010D, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&chSystemId, NULL},
    {0x010E, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&sMh_MotorDataOut.ubDSPIsCustom, NULL},
    {0x010F, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&sMh_MotorDataOut.uwDSPCompLevel, NULL},

        // Bootblock Info
    {0x0110, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &sModBusCMBootBlockInfo.uwApplicatType, NULL},
    {0x0111, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &sModBusCMBootBlockInfo.uwVersionMajor, NULL},
    {0x0112, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &sModBusCMBootBlockInfo.uwVersionMinor, NULL},
    {0x0113, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&ulModBusCMBootBlockAppId, NULL},
    {0x0114, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &sModBusCMBootBlockInfo.uwBuildNumber, NULL},
    {0x0115, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&ulModBusCMBootBlockSysId, NULL},
    {0x0116, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, sizeof(sModBusCMBootBlockInfo.chBlockDescrText)/sizeof(ULONG),
            WRDENY_DEFAULT, (HPVOID)sModBusCMBootBlockInfo.chBlockDescrText, NULL},

        // IO exp board Info
    {0x0120, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&tHwIOExpBoardConfig.chApp, NULL},
    {0x0121, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &tHwIOExpBoardConfig.uwType, NULL},
    {0x0122, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &tHwIOExpBoardConfig.uwBuild, NULL},
    {0x0123, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &tHwIOExpBoardConfig.uwAppCode, NULL},
    {0x0124, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &sGlbIOExpBoardParameters.sProductInfo.uwProductRev, NULL},

        // Serial flash size
    {0x0150, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&QspiFlashSize, NULL},
        // System clock frequency
    {0x0151, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&uwSystemMainClockFreq, NULL},

    //{0x0152, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT, (HPVOID)&(sThModRun.sImageSlowTask.flSOnchipTemp), NULL},

        // Assembly Info
    {0x0170, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT,   &sGlbAssemblyInfo.ulSerialNumber, NULL},
    {0x0171, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, sizeof(sGlbAssemblyInfo.chDescrText)/sizeof(ULONG),
            WRDENY_DEFAULT,   (HPVOID)sGlbAssemblyInfo.chDescrText, NULL},
    {0x0172, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, sizeof(sGlbAssemblyInfo.chDateText)/sizeof(ULONG),
            WRDENY_DEFAULT,   (HPVOID)sGlbAssemblyInfo.chDateText, NULL},

        // System Flags
    {0x0190, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITD , 0, 1,
            WRDENY_DEFAULT, &sModBusSMSystemFlagBits.b[MODBUSSM_BIT_SOFTWAREDRIVERESET], NULL},
    {0x0191, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITD , 0, 1,
            WRDENY_FLASHCOMMAND, &sModBusSMSystemFlagBits.b[MODBUSSM_BIT_SAVESYSTPARAMETERS], NULL},
    {0x0192, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITD , 0, 1,
            WRDENY_DEFAULT, &sModBusSMSystemFlagBits.b[MODBUSSM_BIT_CLEARSYSTEMERRORS], NULL},
    {0x0193, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITD , 0, 1,
            WRDENY_DEFAULT, &sModBusSMSystemFlagBits.b[MODBUSSM_BIT_RESETANDENTERBOOTBLOCK], NULL},
    {0x0194, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITD , 0, 1,
            WRDENY_DEFAULT, &sModBusSMSystemFlagBits.b[MODBUSSM_BIT_ENABLESIMULATEDINPUTS], NULL},
    {0x0195, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITD , 0, 1,
            WRDENY_DEFAULT, &sModBusSMSystemFlagBits.b[MODBUSSM_BIT_DISABLEPHYSICALOUTPUTS], NULL},
#ifdef _APP_XC
    {0x0196, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITD , 0, 1,
            WRDENY_DEFAULT, &sModBusSMSystemFlagBits.b[MODBUSSM_BIT_ENTERBOOTANDUPGRADE], NULL},
#endif
    {0x019D, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITD , 0, 1,
            WRDENY_FLASHCOMMAND, &sModBusSMSystemFlagBits.b[MODBUSSM_BIT_SAVEHWASSEMBLYINFO], NULL},
    {0x019E, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITD , 0, 1,
            WRDENY_FLASHCOMMAND, &sModBusSMSystemFlagBits.b[MODBUSSM_BIT_RESTOREDEFAULTPARAMS], NULL},

        // Modbus parameters
#ifndef _HW_DC
    {0x01A0, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW , 0, 1,         // uwZeroBaseAddress
            WRDENY_PARAMSAVE, &sModBusCMParams.flags.w, NULL},
    {0x01A1, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW , 1, 1,         // uwDisableModbusOverCAN
            WRDENY_PARAMSAVE, &sModBusCMParams.flags.w, NULL},
    {0x01A8, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW , 0, 1,         // ubDriverEnable
            WRDENY_PARAMSAVE, &sModBusCMParOverserial0.stDrvParams.flags.w, NULL},
    {0x01A9, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW , 1, 1,         // ubBroadcAnswer
            WRDENY_PARAMSAVE, &sModBusCMParOverserial0.stDrvParams.flags.w, NULL},
    {0x01AA, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0, 1,
            WRDENY_PARAMSAVE, &sModBusCMParOverserial0.stDrvParams.ubSlaveAddress, NULL},
    {0x01AB, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0, 1,
            WRDENY_PARAMSAVE, &sModBusCMParOverserial0.stPortSetup.ulBaudRate, NULL},
    {0x01AC, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1,
            WRDENY_PARAMSAVE, &sModBusCMParOverserial0.stPortSetup.uwDataBits, NULL},
    {0x01AD, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1,
            WRDENY_PARAMSAVE, &sModBusCMParOverserial0.stPortSetup.uwParityMode, NULL},
    {0x01AE, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1,
            WRDENY_PARAMSAVE, &sModBusCMParOverserial0.stPortSetup.uwStopBits, NULL},
    {0x01AF, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW , 0, 1,         // Select Half Duplex
            WRDENY_PARAMSAVE, &sModBusCMParOverserial0.stPortSetup.uwPortMode, NULL},
    {0x01B0, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1,
            WRDENY_PARAMSAVE, &sModBusCMParOverserial0.stPortSetup.uwWriteDelay, NULL},
    {0x01B1, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1,
            WRDENY_PARAMSAVE, &sModBusCMParOverserial0.uwRxToTxDelay, NULL},
#endif

#ifdef CFG_EN_MODBUSOVERETH
    {0x01C0, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW , 0, 1,         // Modbus/TCP Flags: Enable
            WRDENY_PARAMSAVE, &sModBusCMParOverEth.stDrvParams.flags.w, NULL},
    {0x01C4, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0, 1,         // IP Address
            WRDENY_PARAMSAVE, &sModBusCMParOverEth.stProtSetup.ubIPAddr[0], NULL},
    {0x01C8, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1,         // Port number
            WRDENY_PARAMSAVE, &sModBusCMParOverEth.stProtSetup.uwPortNum, NULL},
    {0x01C9, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0, 1,         // Ethernet Port Select
            WRDENY_PARAMSAVE, &sModBusCMParOverEth.stDrvParams.ubEthPortSel, NULL},
    {0x01CA, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0, 1,         // Protocol
            WRDENY_PARAMSAVE, &sModBusCMParOverEth.stProtSetup.ubProt, NULL},
#endif

        // System Status
    {0x0200, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, &ulSystemStatus, NULL},
        // System Boot Error Code
    {0x0201, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &uwSystemBootErrorCode, NULL},
        // System Wrong Value Parameter Code
    {0x0202, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &uwParChkParamCode, NULL},
       // Power on time
    {0x0210, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, &ulSysTimersTotalPowerOnTime, NULL},
#if (OS_MEASURESTACKSIZE)
        // OS system stack min free
    {0x0218, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &uwOsSysStackFree, NULL},
        // OS user stack min free
    {0x0219, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &uwOsUsrStackFree, NULL},
#endif
        // Max RealTime task exec time
    {0x021A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &uwTaskSchedRTMaxTime, NULL},
        // Local Max RealTime task exec time
    {0x021B, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&uwTaskSchedRTLocalMaxTime, NULL},
#if (OS_MEASURESTACKSIZE)
        // OS user stack min free per task
    {0x021C, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, OS_MAXTASKSALLOWED,
            WRDENY_DEFAULT, &uwOsMinStackFreePerTask[0], NULL},
#endif
        // Local Avg RealTime task exec time
    {0x021D, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&uwTaskSchedRTLocalAvgTime, NULL},

        // System active alarms
    {0x0220, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, &ulSystemAlarms, NULL},
        // System alarms history table pointer
    {0x0221, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, &hpvSysLogDataAlarmPtr, NULL},
        // System alarms history table length
    {0x0222, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &uwSysLogDataAlarmTblLength, NULL},
        // System last activated alarm subcode
    {0x0223, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, &ulSystemAlarmSubCode, NULL},
        // System active warnings
    {0x0224, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1,
            WRDENY_DEFAULT, &ulSystemWarnings, NULL},
        // System alarms history table record length
    {0x0225, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, (HPVOID)&uwSysLogDataAlarmRecordLength, NULL},
        // System last key reset
    {0x0226, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1,
            WRDENY_DEFAULT, &uwSysResParam0, NULL},

    {0x0230, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_SBYTE , 0, 1, WRDENY_PARAMSAVE, &sHwConfParams.ubSysMode, NULL},

    {0x0240, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1, WRDENY_DEFAULT, &uwModBusCMStatListCount, NULL},
    {0x0241, COMMONPARAMDB_FLAG_RD|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   NULL, &ModBusCM_StatListRead},
    {0x0242, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD , 0, 1, WRDENY_DEFAULT, &uwModBusCMImmRestore, NULL},
    {0x0243, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1, WRDENY_DEFAULT, &ulPnlMgrRDLedStatus, NULL},
    {0x0244, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SBYTE , 0, 1, WRDENY_DEFAULT, &sSe_AuxSimOut.sbOutEnabled, NULL},
    {0x0245, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD , 0, 1, WRDENY_DEFAULT, &uwModBusCMEnterDiagnosticApp, NULL},
    {0x0246, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SBYTE , 0, 1, WRDENY_DEFAULT, &sSe_AuxSimOut.sbReqCmd, NULL},

        // Hw config blocks table pointer
    {0x0250, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1, WRDENY_DEFAULT, (HPVOID)&hpvHwCfgBlocksAddr, NULL},
        // Hw config blocks table number of elements
    {0x0251, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1, WRDENY_DEFAULT, &uwHwCfgBlocksSz, NULL},
        // Hw config blocks table record length
    {0x0252, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1, WRDENY_DEFAULT, (HPVOID)&uwHwCfgBlocksRecordLength, NULL},

////    {0x0230, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_P_SLONG , offsetof(GLB_KINEMATIC_DATA, slSpeed), 1,
////            WRDENY_PARAMSAVE, &pulGlbDebugParameters[0], NULL},                            //$TEST$ sample ptr

        //****************************************************************************
        // Hardware parameters
        //****************************************************************************
    {0x0400, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1, WRDENY_DEFAULT, &sGlbControlBoardParameters.sProductInfo.uwProductCode, NULL},
    {0x0401, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1, WRDENY_DEFAULT, &sGlbControlBoardParameters.sProductInfo.uwProductRev, NULL},

#ifndef _HW_DC
    {0x0427, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1, WRDENY_DEFAULT, &sGlbControlBoardParameters.sStd.uwHwConfig, NULL},
#else
    {0x0427, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1, WRDENY_DEFAULT, &sGlbControlBoardParameters.sProductInfo.uwProductCode, NULL},
    {0x0428, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1, WRDENY_DEFAULT, &sGlbOptReq.ulHardwareOpt, NULL},
    {0x0429, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1, WRDENY_DEFAULT, &sGlbOptReq.ulFPGAOpt, NULL},
    {0x042A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1, WRDENY_DEFAULT, &sGlbOptAvailable.ulHardwareOpt, NULL},
    {0x042B, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG , 0, 1, WRDENY_DEFAULT, &sGlbOptAvailable.ulFPGAOpt, NULL},
#endif
    {0x0430, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1, WRDENY_DEFAULT, &sGlbPowerBoardParameters.sProductInfo.uwProductCode, NULL},
    {0x0431, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD , 0, 1, WRDENY_DEFAULT, &sGlbPowerBoardParameters.sProductInfo.uwProductRev, NULL},

    {0x05F0, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG , 0, 1,                                                    WRDENY_PARAMSAVE,   &sGlbAssemblyInfo.ulSerialNumber, NULL},
    {0x05F1, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG , 0, sizeof(sGlbAssemblyInfo.chDescrText)/sizeof(ULONG),   WRDENY_PARAMSAVE,   (HPVOID)sGlbAssemblyInfo.chDescrText, NULL},
    {0x05F2, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG , 0, sizeof(sGlbAssemblyInfo.chDateText)/sizeof(ULONG),    WRDENY_PARAMSAVE,   (HPVOID)sGlbAssemblyInfo.chDateText, NULL},

        //****************************************************************************
        // Encoders
        //****************************************************************************
    {0x0800, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.uwVEncVoltage, NULL},             // Encoder Voltage to Use
    {0x0801, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.uwMainAbsSel, NULL},              // Main absolute sensor to use
    {0x0802, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.uwAuxSel, NULL},                  // Auxiliary sensor to use
    {0x0803, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0,/*0*/ 1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.flags.b.bPos2CntrLoop, NULL},     // flags to select which position-information will be used by the control loop (primary/auxiliary encoder)
    {0x0804, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0,/*1*/ 1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.flags.b.bSpd2CntrLoop, NULL},     // flags to select which speed-information will be used by the control loop (primary/auxiliary encoder)
    {0x0805, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0,/*2*/ 1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.flags.b.bAcc2CntrLoop, NULL},     // flags to select which acceleration-information will be used by the control loop (primary/auxiliary encoder)
    {0x0806, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0,/*3*/ 1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.flags.b.bElecAngle2Fpga, NULL},   // flags to select which electrical angle-information will be used by the control loop (primary/auxiliary encoder)
    {0x0807, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_UBYTE,  0,/*4*/ 1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.flags.b.bDisableIfRelFail, NULL}, // flags to disable fall-back on abs in case rel fail
    {0x0808, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.uwVEncDelay, NULL},               // Delay after applying encoder supply [msec]
    {0x0809, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.uwMainRelSel, NULL},              // Main relative sensor to use
    {0x080A, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.uwMaxAbsRelDiff, NULL},           // Max angle diff between Abs and Rel tracks for redundancy control [0:65535=0:359deg]
    {0x080B, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0,/*5*/ 1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.flags.b.bDisableEPlate, NULL},    // flags to disable electronic plate reading at startup
    {0x080C, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0,/*6*/ 1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.flags.b.bRestoreEPlate, NULL},    // flags to enable restore of electronic plate at startup
    {0x080D, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0,/*7*/ 1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.flags.b.bDisAbsAfterValid, NULL}, // flags to enable restore of electronic plate at startup

        /* ===================== Main and feedback status ===================== */
    {0x080E, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEm_MainEnc.ubStatus, NULL},
    {0x080F, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEm_Fbk2CntrLoop.ubStatus, NULL},

        /* ===================== Main Encoders ===================== */
    {0x0810, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainEnc.sEncData.sqPostn.hi, NULL},          // Main sensor mechanical turns number
    {0x0811, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainEnc.sEncData.sqPostn.lo, NULL},          // Main sensor mechanical angle
    {0x0812, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainEnc.sqMechAbsPosOffset.hi, NULL},        // Main sensor mechanical Abs position offset HI
    {0x0813, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainEnc.sqMechAbsPosOffset.lo, NULL},        // Main sensor mechanical Abs position offset LO
    {0x0814, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainEnc.sEncData.slSpeed, NULL},             // Main sensor mechanical speed
    {0x0815, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainEnc.sEncData.slAccel, NULL},             // Main sensor mechanical acceleration
    {0x0816, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEm_MainEnc.uwElecAngle, NULL},                  // Main sensor electrical angle
    {0x0817, COMMONPARAMDB_FLAG_RD|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sEm_MainEnc, &MotCtrl_AbsolutePositionScaling}, // Main sensor position 32bit

        /* ===================== Feedback to controlloop =========== */
    {0x0818, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_Fbk2CntrLoop.sEncData.sqPostn.hi, NULL},     // Mechanical turns number used by control loop
    {0x0819, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sEm_Fbk2CntrLoop.sEncData.sqPostn.lo, NULL},     // Mechanical angle to control loop
    {0x081A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_Fbk2CntrLoop.sqMechAbsPosOffset.hi, NULL},   // Mechanical Abs position offset HI used by control loop
    {0x081B, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sEm_Fbk2CntrLoop.sqMechAbsPosOffset.lo, NULL},   // Mechanical Abs position offset LO used by control loop
    {0x081C, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_Fbk2CntrLoop.sEncData.slSpeed, NULL},        // Mechanical speed used by control loop
    {0x081D, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_Fbk2CntrLoop.sEncData.slAccel, NULL},        // Mechanical acceleration used by control loop
    {0x081E, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEm_Fbk2CntrLoop.uwElecAngle, NULL},             // Main sensor electrical angle used by FPGA
    {0x081F, COMMONPARAMDB_FLAG_RD|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sEm_Fbk2CntrLoop, &MotCtrl_AbsolutePositionScaling}, // TargetPosition 32bit (in modalita' posizionatore)

        /* -------- Endat MAIN -------- */
    {0x0820, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sEn_Params[ENDAT_SEL_MAIN].uwClockFreq, NULL},           // Endat clock frequency
    {0x0821, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEn_DataOut[ENDAT_SEL_MAIN].uwCRCErrorCounter, NULL},    // Overall CRC error counter
    {0x0822, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEn_DataOut[ENDAT_SEL_MAIN].uwPropagationDelay, NULL},   // Propagation delay time [nsec]
    {0x0823, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE,   &sEn_Params[ENDAT_SEL_MAIN].ulMTurnStartPos, NULL},
    {0x0824, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEn_DataOut[ENDAT_SEL_MAIN].uwMaxFrequency, NULL},       // Max frequency supported
    {0x0825, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEn_DataOut[ENDAT_SEL_MAIN].ubProtocolVersion, NULL},    // Protocol version
    {0x0826, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEn_DataOut[ENDAT_SEL_MAIN].ubStepPerRevBits, NULL},
    {0x0827, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEn_DataOut[ENDAT_SEL_MAIN].ubRevNumBits, NULL},

        /* -------- Biss MAIN -------- */
    {0x0828, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sBiss_Params[BISS_SEL_MAIN].ulClockFreq, NULL},       // Biss clock frequency
    {0x0829, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sBiss_DataOut[BISS_SEL_MAIN].uwCRCErrorCounter, NULL},// Overall CRC error counter
    {0x082A, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &sBiss_Params[BISS_SEL_MAIN].ubType, NULL},            // Encoder Type
    {0x082B, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sBiss_Params[BISS_SEL_MAIN].ulMTurnStartPos, NULL},
    {0x082C, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sBiss_DataOut[BISS_SEL_MAIN].uwMaxFrequency, NULL},   // Max frequency supported
    {0x082E, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sBiss_DataOut[BISS_SEL_MAIN].ubStepPerRevBits, NULL},
    {0x082F, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sBiss_DataOut[BISS_SEL_MAIN].ubRevNumBits, NULL},

        /* -------- Incremental MAIN -------- */
    {0x0830, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0,      1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].ulLineCounts, NULL},                   // Incremental encoder line counts
    {0x0832, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].uwEncPoleNumber, NULL},                // Incremental encoder poles number
    {0x0833, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*0*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bSinCosInterpolation, NULL},   // flags: 0 => NO SinCosInterpolation; 1 => SinCosInterpolation
    {0x0834, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*1*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bIndexErrorDisabled, NULL},    // flags: 0 => Index Error Enabled; 1 => Index Error Disabled
    {0x0835, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*2*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bSin2Cos2ErrorDisabled, NULL}, // flags: 0 => sin^2 + cos^2 Error Enabled; 1 => sin^2 + cos^2 Error Disabled
    {0x0836, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].uwSinCosAlarmThreshold, NULL},         // sin^2 + cos^2 Error threshold
    {0x0837, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].uwIndexTolerance, NULL},               // Index Error Tolerance
    {0x0838, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*3*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bHasIndex, NULL},              // flags: 1 => encoder has valid index track
    {0x0839, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_SWORD, 0,      1, WRDENY_DEFAULT,   &sIc_IncEncDataOut[INCREMENTAL_SEL_MAIN].swSin, NULL},                         // SIN channel as read from ADC
    {0x083A, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_SWORD, 0,      1, WRDENY_DEFAULT,   &sIc_IncEncDataOut[INCREMENTAL_SEL_MAIN].swCos, NULL},                         // COS channel as read from ADC
    {0x083B, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_ULONG, 0,      1, WRDENY_DEFAULT,   &sIc_IncEncDataOut[INCREMENTAL_SEL_MAIN].ulSinCosLevel, NULL},                 // SIN^2+COS^2
    {0x083C, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*4*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bSwapTracks, NULL},            // flags: swap tracks
    {0x083D, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0,/*5*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bEnableStepDir, NULL},         // flags: enable step/dir
    {0x083E, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0,/*6*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bEnableUpDown, NULL},          // flags: enable up/down
    {0x083F, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0,/*7*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bDisFactoryCalibr, NULL},      // flags: disable internal calibration

		/* -------- Hall MAIN -------- */
    {0x0842, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE, &sHl_HallParam.uwEncPoleNumber, NULL},      // Hall encoder poles number
    {0x0843, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &sHl_HallParam.flags.b.b4WSelection, NULL}, // 0 => Hall 3W; 1 => Hall 4W
    {0x0844, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sHl_DataOut.swSinChannel, NULL},           // SIN channel as read from ADC
    {0x0845, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sHl_DataOut.swCosChannel, NULL},           // COS channel as read from ADC

        /* -------- SinCos MAIN -------- */
    {0x0852, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sSc_SinCosParam.uwEncPoleNumber, NULL},                // Incremental encoder poles number
    {0x0853, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0,/*0*/ 1, WRDENY_PARAMSAVE, &sSc_SinCosParam.flags.b.bReverseSignals, NULL},        // flag reverse signal: 0 => Angle = Atan(Sin/Cos); 1 => Angle = 0xffff - Atan(Sin/Cos)
    {0x0854, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0,/*1*/ 1, WRDENY_PARAMSAVE, &sSc_SinCosParam.flags.b.bGain2Use, NULL},              // flag gain to use   : 0 => HIGH (SinCos);         1 => LOW (Resolver)
    {0x0855, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_SWORD, 0,      1, WRDENY_DEFAULT,   &sSc_DataOut.swSinChannel, NULL},                       // SIN channel as read from ADC
    {0x0856, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_SWORD, 0,      1, WRDENY_DEFAULT,   &sSc_DataOut.swCosChannel, NULL},                       // COS channel as read from ADC
    {0x0857, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sSc_SinCosParam.uwSinCosAlarmThreshold, NULL},         // SinCos Level Alarm Threshold
    {0x0858, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_ULONG, 0,      1, WRDENY_DEFAULT,   &sSc_DataOut.ulSinCosLevel, NULL},                      // SIN^2+COS^2
    {0x0859, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SWORD, 0,      1, WRDENY_PARAMSAVE, &sSc_SinCosParam.swGainSin, NULL},                      // SinCos SIN gain adjust
    {0x085A, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SWORD, 0,      1, WRDENY_PARAMSAVE, &sSc_SinCosParam.swGainCos, NULL},                      // SinCos COS gain adjust
    {0x085B, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0,/*3*/ 1, WRDENY_PARAMSAVE, &sSc_SinCosParam.flags.b.bEnableAutoCalibration, NULL}, // flag enable runtime auto calibration
    {0x085C, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SWORD, 0,      1, WRDENY_PARAMSAVE, &sSc_SinCosParam.swOffsetSin, NULL},                    // SinCos SIN offset adjust
    {0x085D, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SWORD, 0,      1, WRDENY_PARAMSAVE, &sSc_SinCosParam.swOffsetCos, NULL},                    // SinCos COS offset adjust
    {0x085E, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sSc_SinCosParam.uwResFreqOffset, NULL},                // resolver frequency offset

        /* -------- Sensorless MAIN -------- */
    {0x0860, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.flags.b.bOpenLoopOnly, NULL},  // flag: Open Loop only
    {0x0861, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0 /*1*/, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.flags.b.bDynamicIqLimit, NULL},  // flag: use dynamic Iq limit
    {0x0862, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0 /*2*/, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.flags.b.bDynamicIdLimit, NULL},  // flag: use dynamic Id limit
    {0x0863, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.uwPercentage_BackOnTheFlyUsrSpeed, NULL}, // % per la velocita' di rientro al volo
    {0x0864, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.uwPercentage_SsnrlssSpeed, NULL},         // % per la velocita' di sensorless
    {0x0865, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.uwPercentage_SsnrlssThreshold1, NULL},    // % per la velocita' di soglia_1
    {0x0866, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.uwPercentage_SsnrlssThreshold0, NULL},    // % per la velocita' di soglia_0
    {0x0867, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.uwEmfEncGlitchFilterPercentage, NULL},    // % per l'intervento dell'antiglitch
    {0x0868, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.ulEmfEncGlitchFaultLimit, NULL}, // limite per fault antiglitch sempre attivo
    {0x0869, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0 /*3*/, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.flags.b.bUseAntiglitch, NULL},  // flag: use antiglitch filter
    {0x086A, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0 /*4*/, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.flags.b.bUseDeltaAngleSpeed, NULL},  // flag: emf speed in full sensorless
    {0x086B, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.ulDisableBackEmfAlarmMask, NULL}, // maschera per disabilitare i fault del sensorless
    {0x086C, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.uwKtRefresh, NULL},              // tempo di refresh per il Kt motore stimato
    {0x086D, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.uwIdOpenLoop, NULL}, // [0:1000=0:MOTPRM_PARAMETERS.flCurrentPeak]
    {0x086E, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0 /*5*/, 1, WRDENY_PARAMSAVE,   &sBe_EmfEncParam.flags.b.bStopMotorKtEval, NULL},  // flag: do not evaluate motor Kt


    {0x0871, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sBe_EmfEncDiagOut.uwSnsrlessState, NULL},        // Sensorless status
    {0x0872, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sBe_EmfEncDiagOut.uwAntiglitchCounter, NULL},    // antiglitch filter counter
    {0x0873, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sBe_EmfEncDiagOut.ulMagicNumber, NULL},          // magic number (evaulated from motor Kt): if zero => Alarm
    {0x0874, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sBe_EmfEncDiagOut.slMechSpeedDeltaAngle, NULL},  // speed calculated as delta angles
    {0x0875, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT,   &sBe_EmfEncDiagOut.flValuedKt, NULL},             // motor Kt evaluated


        /* ===================== Auxiliary ===================== */
    {0x0880, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_AuxEnc.sEncData.sqPostn.hi, NULL},           // Auxiliary sensor mechanical turns number
    {0x0881, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sEm_AuxEnc.sEncData.sqPostn.lo, NULL},           // Auxiliary sensor mechanical angle
    {0x0882, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_AuxEnc.sqMechAbsPosOffset.hi, NULL},         // Auxiliary sensor mechanical Abs position offset HI
    {0x0883, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sEm_AuxEnc.sqMechAbsPosOffset.lo, NULL},         // Auxiliary sensor mechanical Abs position offset LO
    {0x0884, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_AuxEnc.sEncData.slSpeed, NULL},              // Auxiliary sensor mechanical speed
    {0x0885, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_AuxEnc.sEncData.slAccel, NULL},              // Auxiliary sensor mechanical acceleration
    {0x0886, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEm_AuxEnc.uwElecAngle, NULL},                   // Auxiliary sensor electrical angle
    {0x0887, COMMONPARAMDB_FLAG_RD|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sEm_AuxEnc, &MotCtrl_AbsolutePositionScaling}, // Auxiliary sensor position 32bit
    {0x0888, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEm_AuxEnc.ubStatus, NULL},

        /* -------- Endat Auxiliary  -------- */
#ifndef _HW_DC
    {0x0890, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sEn_Params[ENDAT_SEL_AUX].uwClockFreq, NULL},           // Endat clock frequency
    {0x0891, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEn_DataOut[ENDAT_SEL_AUX].uwCRCErrorCounter, NULL},    // Overall CRC error counter
    {0x0892, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEn_DataOut[ENDAT_SEL_AUX].uwPropagationDelay, NULL},   // Propagation delay time [nsec]
    {0x0893, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE,   &sEn_Params[ENDAT_SEL_AUX].ulMTurnStartPos, NULL},
    {0x0894, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEn_DataOut[ENDAT_SEL_AUX].uwMaxFrequency, NULL},       // Max frequency supported
    {0x0895, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEn_DataOut[ENDAT_SEL_AUX].ubProtocolVersion, NULL},    // Protocol version
    {0x0896, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEn_DataOut[ENDAT_SEL_AUX].ubStepPerRevBits, NULL},
    {0x0897, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEn_DataOut[ENDAT_SEL_AUX].ubRevNumBits, NULL},
#endif

        /* -------- Biss Auxiliary  -------- */
    {0x0898, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sBiss_Params[BISS_SEL_AUX].ulClockFreq, NULL},       // Biss clock frequency
    {0x0899, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sBiss_DataOut[BISS_SEL_AUX].uwCRCErrorCounter, NULL},// Overall CRC error counter
    {0x089A, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &sBiss_Params[BISS_SEL_AUX].ubType, NULL},            // Encoder Type
    {0x089B, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sBiss_Params[BISS_SEL_AUX].ulMTurnStartPos, NULL},
    {0x089C, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sBiss_DataOut[BISS_SEL_AUX].uwMaxFrequency, NULL},   // Max frequency supported
    {0x089E, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sBiss_DataOut[BISS_SEL_AUX].ubStepPerRevBits, NULL},
    {0x089F, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sBiss_DataOut[BISS_SEL_AUX].ubRevNumBits, NULL},

        /* -------- Incremental Auxiliary -------- */
    {0x08A0, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0,      1, WRDENY_PARAMSAVE,   &sIc_IncEncParams[INCREMENTAL_SEL_AUX].ulLineCounts, NULL},         // Incremental encoder line counts
    {0x08A2, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE,   &sIc_IncEncParams[INCREMENTAL_SEL_AUX].uwEncPoleNumber, NULL},      // Incremental encoder poles number
    {0x08A4, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*1*/ 1, WRDENY_PARAMSAVE,   &sIc_IncEncParams[INCREMENTAL_SEL_AUX].flags.b.bIndexErrorDisabled, NULL},              // flags: 0 => Index Error Enabled; 1 => Index Error Disabled
    {0x08A5, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE,   &sIc_IncEncParams[INCREMENTAL_SEL_AUX].uwIndexTolerance, NULL},     // Index Error Tolerance
    {0x08A6, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*3*/ 1, WRDENY_PARAMSAVE,   &sIc_IncEncParams[INCREMENTAL_SEL_AUX].flags.b.bHasIndex, NULL},              // 1 => encoder has valid index track
    {0x08A7, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*4*/ 1, WRDENY_PARAMSAVE,   &sIc_IncEncParams[INCREMENTAL_SEL_AUX].flags.b.bSwapTracks, NULL},              // swap tracks
    {0x08A8, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*5*/ 1, WRDENY_PARAMSAVE,   &sIc_IncEncParams[INCREMENTAL_SEL_AUX].flags.b.bEnableStepDir, NULL},              // enable step/dir
    {0x08A9, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*6*/ 1, WRDENY_PARAMSAVE,   &sIc_IncEncParams[INCREMENTAL_SEL_AUX].flags.b.bEnableUpDown, NULL},              // enable up/down

        /* -------- Simulated Encoder -------- */
    {0x08B0, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0,      1, WRDENY_PARAMSAVE, &sSe_AuxSimParam.ulIndexPulseRpt, NULL},       // Simulated Encoder: Simulated Encoder Index Line Counts
    {0x08B1, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0,      1, WRDENY_PARAMSAVE, &sSe_AuxSimParam.ulIndexOffset, NULL},         // Simulated Encoder: index position offset
    {0x08B2, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sSe_AuxSimParam.uwMaxAngleDiff, NULL},        // Simulated Encoder: max angle diff before alarm
    {0x08B3, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0,      1, WRDENY_PARAMSAVE, &sSe_AuxSimParam.ulLineCounts, NULL},          // Simulated Encoder: line counts
    {0x08B4, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_UWORD, 0,      1, WRDENY_PARAMSAVE, &sSe_AuxSimParam.uwMaxFrequency, NULL},        // Simulated Encoder: max frequency [kHz]
    {0x08B5, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_SLONG, 0,      1, WRDENY_DEFAULT,   &sSe_AuxSimOut.sqPostn.hi, NULL},              // mechanical turns number
    {0x08B6, COMMONPARAMDB_FLAG_RD,                             COMMONPARAMDB_TYPE_ULONG, 0,      1, WRDENY_DEFAULT,   &sSe_AuxSimOut.sqPostn.lo, NULL},              // mechanical angle
    {0x08B7, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*0*/ 1, WRDENY_PARAMSAVE, &sSe_AuxSimParam.flags.b.bEnFastAuxSim, NULL},
    {0x08B8, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*1*/ 1, WRDENY_PARAMSAVE, &sSe_AuxSimParam.flags.b.bBlindAuxSim, NULL},

        /* -------- Hiperface Encoder -------- */
    {0x08C0, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE,   &sHf_Params.ulMTurnStartPos, NULL},

        /* -------- Nikon MAIN -------- */
    {0x08D0, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE,   &sNk_Params[NIKON_SEL_MAIN].ulMTurnStartPos, NULL},
    {0x08D1, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0 /*1*/, 1, WRDENY_PARAMSAVE,   &sNk_Params[NIKON_SEL_MAIN].flags.b.bDisMTData, NULL},
    {0x08D8, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sNk_DataOut[NIKON_SEL_MAIN].ubBaudrate, NULL},
    {0x08D9, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sNk_DataOut[NIKON_SEL_MAIN].ubBatteryLine, NULL},
    {0x08DA, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sNk_DataOut[NIKON_SEL_MAIN].ubStepPerRevBits, NULL},
    {0x08DB, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sNk_DataOut[NIKON_SEL_MAIN].ubRevNumBits, NULL},

        /* -------- Tamagawa MAIN -------- */
    {0x08E0, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE,   &sTm_Params[TMGW_SEL_MAIN].ulMTurnStartPos, NULL},
    {0x08E1, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0 /*1*/, 1, WRDENY_PARAMSAVE,   &sTm_Params[TMGW_SEL_MAIN].flags.b.bDisMTData, NULL},
    {0x08E8, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sTm_DataOut[TMGW_SEL_MAIN].ubMultiTurn, NULL},
    {0x08E9, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sTm_DataOut[TMGW_SEL_MAIN].ubResolution, NULL},
    {0x08EA, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sTm_DataOut[TMGW_SEL_MAIN].ubBaudrate, NULL},

        /* ==== Electrical Field Orientation Parameters ======== */
    {0x08F0, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sEfsParam.uwIdRampTime, NULL},         // time to direct the rotor [msec]
    {0x08F1, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sEfsParam.uwIdRampCurrent, NULL},      // [0:1000=0:MOTPRM_PARAMETERS.flCurrentPeak]
    {0x08F2, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE,   &sEfsParam.ubProcType, NULL},           // procedure type
	{0x08F3, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE,   &sEfsParam.bForce, NULL},//    {0x08F3, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITW,  0, 1, WRDENY_PARAMSAVE,   &sEfsParam.options.w, NULL},            // force procedure also if ENCMGR_ELE_ANGLE_VALID
    {0x08F4, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE,   &sEfsParam.slSpeedThreshold, NULL},     // wait for speed below this threshold
    {0x08F5, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,     &sEm_MainEnc.uwDeltaElecAngle, NULL},   // Main sensor elec angle offset calculation
    {0x08F6, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE,   &sEfsParam.slElecAngleFeed, NULL},      // ratio for feeding electrical angle from IqRef output
    {0x08F7, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sEfsParam.uwIdSteadyTime, NULL},       // time when Id is kept constant [msec]
    {0x08F8, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE,   &sEfsParam.sPI_Param.swKi, NULL},
    {0x08F9, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE,   &sEfsParam.sPI_Param.swKp, NULL},
    {0x08FA, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE,   &sEfsParam.sPI_Param.swErrMax, NULL},
    {0x08FB, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE,   &sEfsParam.sPI_Param.swGlobalShift, NULL},
    {0x08FC, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE,   &sEfsParam.sPI_Param.slOutValLimit, NULL},

        /* ===================== Main Abs Encoder ===================== */
    {0x0900, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainAbsEnc.sEncData.sqPostn.hi, NULL},          // mechanical turns number
    {0x0901, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainAbsEnc.sEncData.sqPostn.lo, NULL},          // mechanical angle
    {0x0902, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainAbsEnc.sqMechAbsPosOffset.hi, NULL},        // mechanical Abs position offset HI
    {0x0903, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainAbsEnc.sqMechAbsPosOffset.lo, NULL},        // mechanical Abs position offset LO
    {0x0904, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainAbsEnc.sEncData.slSpeed, NULL},             // mechanical speed
    {0x0905, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainAbsEnc.sEncData.slAccel, NULL},             // mechanical acceleration
    {0x0906, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEm_MainAbsEnc.uwElecAngle, NULL},                  // electrical angle
    {0x0907, COMMONPARAMDB_FLAG_RD|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sEm_MainAbsEnc, &MotCtrl_AbsolutePositionScaling}, // Main sensor position 32bit
    {0x0908, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEm_MainAbsEnc.ubStatus, NULL},

        /* ===================== Main Rel Encoder ===================== */
    {0x0910, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainRelEnc.sEncData.sqPostn.hi, NULL},          // mechanical turns number
    {0x0911, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainRelEnc.sEncData.sqPostn.lo, NULL},          // mechanical angle
    {0x0912, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainRelEnc.sqMechAbsPosOffset.hi, NULL},        // mechanical Abs position offset HI
    {0x0913, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainRelEnc.sqMechAbsPosOffset.lo, NULL},        // mechanical Abs position offset LO
    {0x0914, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainRelEnc.sEncData.slSpeed, NULL},             // mechanical speed
    {0x0915, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_MainRelEnc.sEncData.slAccel, NULL},             // mechanical acceleration
    {0x0916, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEm_MainRelEnc.uwElecAngle, NULL},                  // electrical angle
    {0x0917, COMMONPARAMDB_FLAG_RD|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sEm_MainRelEnc, &MotCtrl_AbsolutePositionScaling}, // Main sensor position 32bit
    {0x0918, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEm_MainRelEnc.ubStatus, NULL},

        /* ===================== 64bit direct access ===================== */
    {0x0920, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sEm_MainEnc.sEncData.sqPostn, NULL},
    {0x0921, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sEm_MainEnc.sqMechAbsPosOffset, NULL},
    {0x0922, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sEm_Fbk2CntrLoop.sEncData.sqPostn, NULL},
    {0x0923, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sEm_Fbk2CntrLoop.sqMechAbsPosOffset, NULL},
    {0x0924, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sEm_AuxEnc.sEncData.sqPostn, NULL},
    {0x0925, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sEm_AuxEnc.sqMechAbsPosOffset, NULL},
    {0x0926, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sEm_MainAbsEnc.sEncData.sqPostn, NULL},
    {0x0927, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sEm_MainAbsEnc.sqMechAbsPosOffset, NULL},
    {0x0928, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sEm_MainRelEnc.sEncData.sqPostn, NULL},
    {0x0929, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sEm_MainRelEnc.sqMechAbsPosOffset, NULL},
    {0x092A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sPo_PostnerOut.sDemand.sqPostn, NULL},

    {0x0930, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_C_UBYTE, 0, 1, WRDENY_DEFAULT, (HPVOID)2, NULL},

        /* ===================== Extras ===================== */
    {0x0940, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEm_Fbk2CLExt.slFilteredSpeed, NULL},        // Mechanical filtered speed
    {0x0941, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.slMaxSpeed, NULL},          // Maximum allowed speed [d.u.]
    {0x0942, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sEm_EncMngrParam.swElecAngleFFTime, NULL},   // Electrical Angle feed forward time [usec]
#ifdef _HW_DC
    {0x0943, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sEm_EncMngrParam.uwAuxVEncVoltage, NULL},
#endif
    {0x0944, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE,   &sEm_EncMngrParam.uwSimSel, NULL},

        /* -------- Incremental MAIN Extras -------- */
    {0x0980, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_UWORD, 0,       1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].uwMaxFrequency, NULL},                // Max frequency [kHz]
    {0x0981, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/* 8*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bDisIdxEAngle, NULL},         // disable index track for electrical angle adjustment
    {0x0982, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/* 9*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bDisIdxPosition, NULL},       // disable index track for mechanical position adjustment
    {0x0983, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*10*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bDisIdxABSync, NULL},         // un-sync index track from AB tracks
    {0x0984, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*11*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bDisDigInterpolation, NULL},  // disable digital interpolation with periodimeter
    {0x0985, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*12*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bWdtFiltErrorDisabled, NULL}, // disable filter watchdog error on digital tracks
    {0x0986, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*13*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].flags.b.bCaptureAllIndexes, NULL},    // enable capturing of all indexes, unregarding index tolerance
    {0x0987, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_UWORD, 0,       1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_MAIN].uwInterpSwitchFreq, NULL},            // Max frequency [kHz]

        /* -------- Incremental Aux Extras -------- */
    {0x0990, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_UWORD, 0,       1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_AUX].uwMaxFrequency, NULL},                // Max frequency [kHz]
    {0x0991, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/* 8*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_AUX].flags.b.bDisIdxEAngle, NULL},         // disable index track for electrical angle adjustment
    {0x0992, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/* 9*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_AUX].flags.b.bDisIdxPosition, NULL},       // disable index track for mechanical position adjustment
    {0x0993, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*10*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_AUX].flags.b.bDisIdxABSync, NULL},         // un-sync index track from AB tracks
    {0x0994, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*11*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_AUX].flags.b.bDisDigInterpolation, NULL},  // disable digital interpolation with periodimeter
    {0x0995, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*12*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_AUX].flags.b.bWdtFiltErrorDisabled, NULL}, // disable filter watchdog error on digital tracks
    {0x0996, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE , 0,/*13*/ 1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_AUX].flags.b.bCaptureAllIndexes, NULL},    // enable capturing of all indexes, unregarding index tolerance
    {0x0997, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_UWORD, 0,       1, WRDENY_PARAMSAVE, &sIc_IncEncParams[INCREMENTAL_SEL_AUX].uwInterpSwitchFreq, NULL},            // Max frequency [kHz]

        /* ==== Electrical Field Orientation Monitoring ======== */
    {0x09A0, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEfsOut.uwPhasingQuality, NULL},
    {0x09A1, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sEfsOut.sPI_Out.swErr, NULL},
    {0x09A2, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sEfsOut.sPI_Out.slPiOut, NULL},
    {0x09A3, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sEfsOut.sPI_Out.swCorrection, NULL},
    {0x09A4, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEfsOut.uwEfsStatus, NULL},
    {0x09A5, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEfsOut.uwEfsElecAngleCorrection, NULL},
    {0x09A6, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sEfsOut.swEfsPiAutoKi, NULL},
    {0x09A7, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sEfsOut.swEfsPiAutoKp, NULL},
    {0x09A8, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sEfsOut.swEfsPiAutoShift, NULL},

        //****************************************************************************
        // SpaceSpeed Control Loop
        //****************************************************************************
    {0x0A00, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE, &sSS_CntrLoopParam.sPars.flags.b.bUseDifferentKp, NULL},                 // SpaceSpeed use different SpeedKp
        /* ------------- gains ------------- */
    {0x0A01, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT  , &sSS_CntrLoopParam.sIntGains.swPostnKp, NULL},                // Position: proportional gain value
    {0x0A02, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT  , &sSS_CntrLoopParam.sIntGains.swSpdKpRef, NULL},               // Speed: reference proportional gain value
    {0x0A03, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT  , &sSS_CntrLoopParam.sIntGains.swSpdKpFbk, NULL},               // Speed: feedback  proportional gain value
    {0x0A04, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT  , &sSS_CntrLoopParam.sIntGains.swAccKpRef, NULL},               // Acceleration: reference proportional gain value
    {0x0A05, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT  , &sSS_CntrLoopParam.sIntGains.swAccKpFbk, NULL},               // Acceleration: feedback  proportional gain value
    {0x0A06, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT  , &sSS_CntrLoopParam.sIntGains.swKi, NULL},                     // integral gain value
        /* ------------- shifts ------------- */
    {0x0A07, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT  , &sSS_CntrLoopParam.sIntGains.uwPosGainShift, NULL},           // Position: global gain shift: ONLY shift RIGHT, max +16
    {0x0A08, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT  , &sSS_CntrLoopParam.sIntGains.uwAccGainShift, NULL},           // Acceleration: gains shift-> ONLY shift LEFT, max -15
    {0x0A09, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT  , &sSS_CntrLoopParam.sIntGains.swGlobalGainShift, NULL},        // gains global shift: max shift left = -8; max shift right = +16
        /* ------------- limits ------------- */
    {0x0A0C, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sSS_CntrLoopParam.sPars.sUsrOutVal32.slMax, NULL},      // User: Max Torque limit
    {0x0A0D, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sSS_CntrLoopParam.sPars.sUsrOutVal32.slMin, NULL},      // User: Min Torque limit
        /* ------------- gains ------------- */
    {0x0A0E, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sSS_CntrLoopParam.sPars.sGains.flKi, NULL},
    {0x0A0F, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sSS_CntrLoopParam.sPars.sGains.flPosKp, NULL},
    {0x0A10, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sSS_CntrLoopParam.sPars.sGains.flSpdKpRef, NULL},
    {0x0A11, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sSS_CntrLoopParam.sPars.sGains.flSpdKpFbk, NULL},
    {0x0A12, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sSS_CntrLoopParam.sPars.sGains.flAccKpRef, NULL},
    {0x0A13, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sSS_CntrLoopParam.sPars.sGains.flAccKpFbk, NULL},
    {0x0A14, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_C_UBYTE, 0, 1, WRDENY_DEFAULT, (HPVOID)6, NULL},

    {0x0A15, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE, &sSS_CntrLoopParam.sPars.flags.b.bFilterAccRef, NULL},                 // SpaceSpeed enable accel filter
    {0x0A16, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sSS_CntrLoopParam.sPars.swAccKFilter, NULL},            // SpaceSpeed accel filter constant

        /* ============== Monitor ============== */
        /* -------------- Output -------------- */
    {0x0A31, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sSS_CntrLoopOut.sIRef.slIqRef, NULL},             // Torque reference to FPGA (1e-4A)
    {0x0A32, COMMONPARAMDB_FLAG_RD|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sSS_CntrLoopOut.sqFitPErrMin, &MotCtrl_RelativePositionScaling},
    {0x0A33, COMMONPARAMDB_FLAG_RD|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sSS_CntrLoopOut.sqFitPErrMax, &MotCtrl_RelativePositionScaling},
    {0x0A34, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sSS_CntrLoopOut.slAccRefFiltered32, NULL},        // Filtered ref acc


        //****************************************************************************
        // PI DcBus Control Loop
        //****************************************************************************
        /* ----------- reference ----------- */
    {0x0B00, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &swDcBusRef_In_Usr, NULL},
        /* ------------- gains ------------- */
    {0x0B01, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Param.swKp, NULL},                     // proportional gain
    {0x0B02, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Param.swKi, NULL},                     // integral gain
        /* ------------- shifts ------------- */
    {0x0B03, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Param.swGlobalShift, NULL},            // gains global shift: max shift left = -8; max shift right = +16
       /* ------------- limits ------------- */
    {0x0B04, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Param.swErrMax, NULL},                 // Max Error allowed
    {0x0B05, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Param.swErrMin, NULL},                 // Min Error allowed
    {0x0B06, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Param.sI_DcBus.slMax, NULL},           // Limite sulla Max Corrente richiesta al DcBus
    {0x0B07, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Param.sI_DcBus.slMin, NULL},           // Limite sulla Min Corrente richiesta al DcBus
//    {0x0B08, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Param.sI2Fpga.slMax, NULL},            // Limite sulla Max Corrente richiesta all'FPGA
//    {0x0B09, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Param.sI2Fpga.slMin, NULL},            // Limite sulla Min Corrente richiesta all'FPGA
	{0x0B0A, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Param.bOnlyPI, NULL},//{0x0B0A, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITW, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Param.flags.b.bOnlyPI, NULL},                  // usa solo PI

        /* ============== Monitor ============== */
        /* -------------- Output --------------- */
    {0x0B20, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Out.swErr, NULL},                      // error measured
    {0x0B21, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Out.sIDcBus.slIqRef, NULL},            // DcBus Current (1e-4A)
    {0x0B22, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sDcBusCntrl_Out.sIRef.slIqRef, NULL},              // Current reference to FPGA (1e-4A)

        //****************************************************************************
        // Positioner
        //****************************************************************************
    {0x0C00, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.slProfileVel, NULL},              // Profile Acceleration
    {0x0C01, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.slProfileAcc, NULL},              // Profile Velocity
    {0x0C02, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.slProfileDec, NULL},              // Profile Deceleration
    {0x0C03, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.slQuickStopDec, NULL},            // QuickStop Deceleration
    {0x0C04, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.slEndVelocity, NULL},             // End Speed (in modalita' posizionatore)
    {0x0C05, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_UMCONV,   COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.sqPstnErrMax, &MotCtrl_RelativePositionScaling}, // Max Position Error allowed
    {0x0C06, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.slZeroSpeedThreshold, NULL},      // Threshold to consider speed = zero
    {0x0C07, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.swMotBlockTout, NULL},            // Motor blocked timeout
    {0x0C08, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_DEFAULT,   &sPo_PostnerParam.flags.b.bSRampEnable, NULL},
    {0x0C09, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.swSRampK1, NULL},
    {0x0C0A, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.swSRampK2, NULL},
    {0x0C0B, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.sqSRampPstnErrMax.hi, NULL},
    {0x0C0C, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.sqSRampPstnErrMax.lo, NULL},
    {0x0C0D, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_PARAMSAVE, &sPo_PostnerParam.sqSRampPstnErrMax, NULL},

        /* ============== Monitor ============== */
        /* --------------- Input --------------- */
    {0x0C17, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sPo_UsrPostnerIn.slTargetSpeed, NULL},             // TargetVelocity (in modalita velocita')

        /* --------------- Output --------------- */
    {0x0C30, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sPo_PostnerOut.flags.w, NULL},                     // Flags status modulo sw posizionatore
    {0x0C32, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sPo_PostnerOut.sDemand.sqPostn.hi, NULL},          // posizione di riferimento da mandare in anello: turns
    {0x0C33, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT, &sPo_PostnerOut.sDemand.sqPostn.lo, NULL},          // posizione di riferimento da mandare in anello: angle
    {0x0C34, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sPo_PostnerOut.sDemand.slSpeed, NULL},             // velocita' di riferimento da mandare in anello
    {0x0C35, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sPo_PostnerOut.sDemand.slAccel, NULL},             // accelerazione di riferimento da mandare in anello
//    {0x0C36, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sPo_PostnerOut.slPosErr, NULL},                    // Position error measured
    {0x0C36, COMMONPARAMDB_FLAG_RD|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sPo_PostnerOut.sqPosErr, &MotCtrl_RelativePositionScaling}, // Position error 32bit
    {0x0C37, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sPo_PostnerOut.sqPosErr.hi, NULL},                 // Position error 32bit: Turns
    {0x0C38, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT, &sPo_PostnerOut.sqPosErr.lo, NULL},                 // Position error 32bit: Angle
    {0x0C39, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT, &sPo_PostnerOut.sqPosErr, NULL},                    // Position error 64bit

        //****************************************************************************
        // Motor Handler
        //****************************************************************************
    {0x0E00, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.swVBrakeLow, NULL},             // Tensione di spegnimento resistenza frenatura
    {0x0E01, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.swVBrakeHigh, NULL},            // Tensione di accensione resistenza frenatura
    {0x0E02, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.swUnderVoltageThreshold, NULL}, // Soglia di undervoltage (fault se abilitato, warning se disabilitato)
    {0x0E03, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.swOverVoltageThreshold, NULL},  // Soglia di fault overvoltage (utente)
    {0x0E04, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.slOverCurrentThreshold, NULL},  // Soglia di fault overcurrent (utente)
    {0x0E07, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.flModKi, NULL},                 // Ki anello di corrente
    {0x0E08, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.flModKp, NULL},                 // Kp anello di corrente
    {0x0E09, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIdLimit.slMin, NULL},          // Limite Id Min
    {0x0E0A, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIdLimit.slMax, NULL},          // Limite Id Max
    {0x0E0B, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqLimit.slMin, NULL},          // Limite Iq Min
    {0x0E0C, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqLimit.slMax, NULL},          // Limite Iq Max
#ifdef _HW_DC
    {0x0E0F, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_DEFAULT,   &sMh_MotorDataParam.flags.b.bDflxWithIqFltRef, NULL}, // MotorHandler bDflxWithIqFltRef
    {0x0E10, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_DEFAULT,   &sMh_MotorDataParam.flags.b.bEnParallelBr2, NULL},    // MotorHandler bEnParallelBr2
    {0x0E11, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_DEFAULT,   &sMh_MotorDataParam.flags.b.bEnParallelBr1, NULL},    // MotorHandler bEnParallelBr1
#else
    {0x0E0F, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_DEFAULT,   &sMh_MotorDataParam.flags.b.bDflxWithIqFltRef, NULL}, // MotorHandler bDflxWithIqFltRef
#endif
    {0x0E12, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.uwUserBridgeLayout, NULL},     // user bridge layout
    {0x0E13, COMMONPARAMDB_FLAG_RW,                             COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_DEFAULT,   &sMh_MotorDataParam.flags.b.bDisPhasesCheck, NULL},// MotorHandler bDisPhasesCheck
    {0x0E14, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_DEFAULT,   &sMh_MotorDataParam.flags.b.bDisablePOST, NULL},   // MotorHandler bDisablePOST
    {0x0E15, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_DEFAULT,   &sMh_MotorDataParam.flags.b.bForcePOST, NULL},     // MotorHandler bForcePOST
    {0x0E16, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_DEFAULT,   &sMh_MotorDataParam.flags.b.bDisVacMgm, NULL},     // MotorHandler bDisVacMgm
    {0x0E17, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_DEFAULT,   &sMh_MotorDataParam.flags.b.bEnableDeflux, NULL},  // MotorHandler bEnableDeflux
    {0x0E18, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.uwVacMinDistortion, NULL},      // Soglia minima accettata distorsione Vac
    {0x0E20, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slIdFb, NULL},                    // Id feedback
    {0x0E21, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slIqFb, NULL},                    // Iq feedback
    {0x0E22, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slIuFb, NULL},                    // Iu feedback
    {0x0E23, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slIvFb, NULL},                    // Iv feedback
    {0x0E24, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.sBackEmfData.swAlpha, NULL},      // BackEmf Alpha
    {0x0E25, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.sBackEmfData.swBeta, NULL},       // BackEmf Beta
    {0x0E26, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swVuEstimated, NULL},             // Vu stimata
    {0x0E27, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swVvEstimated, NULL},             // Vv stimata
    {0x0E28, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swDcBusValue, NULL},              // valore Dc Bus
    {0x0E29, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.sBackEmfData.uwVmotor, NULL},     // tensione motore
    {0x0E2A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.sBackEmfData.uwAtanAngle, NULL},  // AtanAngle
    {0x0E2B, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.sI2Fpga.slIdRef, NULL},           // IdRef2Fpga
    {0x0E2C, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.sI2Fpga.slIqRef, NULL},           // IqRef2Fpga
    {0x0E2D, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slIwFb, NULL},                    // Iw feedback
    {0x0E2E, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slIqFltRef, NULL},                // IqFilteredRef2Fpga
    {0x0E2F, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swACVrs, NULL},                   // Vac RS
    {0x0E30, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swACVst, NULL},                   // Vac ST
    {0x0E31, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.ubACStatus, NULL},                // Vac input phases status
    {0x0E32, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swACVtr, NULL},                   // Vac TR
    {0x0E33, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swVdOut, NULL},                   // Vd Out
    {0x0E34, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swVqOut, NULL},                   // Vq Out
    {0x0E40, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataUsrInIRef.slIdRef, NULL},             // UsrIdRef (Torque mode)
    {0x0E41, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataUsrInIRef.slIqRef, NULL},             // UsrIqRef (Torque mode)
    {0x0E4F, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.uwDSPCompLevel, NULL},
    {0x0E50, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.flAutoModKi, NULL},               // AutoModKp
    {0x0E51, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.flAutoModKp, NULL},               // AutoModKi
    {0x0E52, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.sIdLimit.slMin, NULL},
    {0x0E53, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.sIdLimit.slMax, NULL},
    {0x0E54, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.sIqLimit.slMin, NULL},
    {0x0E55, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.sIqLimit.slMax, NULL},
    {0x0E56, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataUsrSetIdLimit.slMin, NULL},
    {0x0E57, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataUsrSetIdLimit.slMax, NULL},
    {0x0E58, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataUsrSetIqLimit.slMin, NULL},
    {0x0E59, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataUsrSetIqLimit.slMax, NULL},
    {0x0E5A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.sDriveLimit.slOverCurrent, NULL}, // actual overcurrent threshold
    {0x0E5B, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.sDriveLimit.swOverVoltage, NULL}, // actual overvoltage threshold
    {0x0E5C, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swActiveVBrakeLow, NULL},         // actual brake off threshold
    {0x0E5D, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swActiveVBrakeHigh, NULL},        // actual brake on threshold
    {0x0E5E, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.ubActualPwmFrequency, NULL},      // actual switching frequency
    {0x0E5F, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.ubDSPLoad, NULL},

    {0x0E60, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[0].ubType, NULL},
    {0x0E61, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[0].flFreqMain, NULL},
    {0x0E62, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[0].flDampMain, NULL},
    {0x0E63, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[0].flFreqSec, NULL},
    {0x0E64, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[0].flDampSec, NULL},
    {0x0E68, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[1].ubType, NULL},
    {0x0E69, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[1].flFreqMain, NULL},
    {0x0E6A, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[1].flDampMain, NULL},
    {0x0E6B, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[1].flFreqSec, NULL},
    {0x0E6C, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[1].flDampSec, NULL},
    {0x0E70, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[2].ubType, NULL},
    {0x0E71, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[2].flFreqMain, NULL},
    {0x0E72, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[2].flDampMain, NULL},
    {0x0E73, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[2].flFreqSec, NULL},
    {0x0E74, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[2].flDampSec, NULL},
    {0x0E78, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[3].ubType, NULL},
    {0x0E79, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[3].flFreqMain, NULL},
    {0x0E7A, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[3].flDampMain, NULL},
    {0x0E7B, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[3].flFreqSec, NULL},
    {0x0E7C, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataParam.sIqFiltPars[3].flDampSec, NULL},
#if CFG_VMOTOR_READ
	{0x0E7D, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swVuEffective, NULL},             // Vu effective
	{0x0E7E, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swVvEffective, NULL},             // Vv effective
	{0x0E7F, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.swVwEffective, NULL},             // Vw effective
#endif

#ifdef _HW_DC
    {0x0E88, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slBr1IuFb, NULL},
    {0x0E89, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slBr1IvFb, NULL},
    {0x0E8A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slBr1IwFb, NULL},
    {0x0E8B, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slBr2IuFb, NULL},
    {0x0E8C, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slBr2IvFb, NULL},
    {0x0E8D, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slBr2IwFb, NULL},
#endif

#ifdef _HW_CT
    {0x0E8E, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMh_MotorDataOut.slIBrake, NULL},
#endif
        //****************************************************************************
        // Field Weakening
        //****************************************************************************
    {0x0EA0, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sDflx_Param.slDflxSpdMax, NULL},
    {0x0EA1, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE, &sDflx_Param.uwKVOut, NULL},
    {0x0EA2, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sDflx_Param.swDflxVdc, NULL},

    {0x0EA8, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE, &sDflx_Out.flags.b.bDefluxActive, NULL},      // bDefluxActive
    {0x0EA9, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE, &sDflx_Out.flags.b.bMaxSpeedReachable, NULL}, // bMaxSpeedReachable
    {0x0EAA, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 1, 1, WRDENY_PARAMSAVE, &sDflx_Out.slDflxKneeSpd, NULL},
    {0x0EAB, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 1, 1, WRDENY_PARAMSAVE, &sDflx_Out.slMotorIcc, NULL},


        //****************************************************************************
        // Motion Controller
        //****************************************************************************
    {0x1000, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sMotCtrl_UsrControl.uwControlWord, NULL},
    {0x1001, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sMotCtrl_Out.uwStatusWord, NULL},
    {0x1008, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   (HPVOID)&ulMotCtrlSuppDriveModes},
    {0x1010, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.ubModeOfOperation, NULL},
    {0x1011, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sMotCtrl_Out.ubModeOfOperationDisplay, NULL},
    {0x1020, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMotCtrl_UsrControl.sqTargetPostn.hi, NULL},     // TargetPosition 32bit: Turns (in modalita' posizionatore)
    {0x1021, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sMotCtrl_UsrControl.sqTargetPostn.lo, NULL},     // TargetPosition 32bit: Angle (in modalita' posizionatore)
    {0x1022, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sMotCtrl_UsrControl.sqTargetPostn, NULL},        // TargetPosition 64bit (in modalita' posizionatore)
    {0x1023, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sMotCtrl_UsrControl.sqTargetPostn, &MotCtrl_RelativePositionScaling}, // TargetPosition 32bit (in modalita' posizionatore)
    {0x1024, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_DEFAULT,   &sMotCtrl_UsrControl.sqIPQuota, NULL},            // IP Quota 64bit (in modalita' interpolatore)
    {0x1025, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_C_UBYTE, 0, 1, WRDENY_DEFAULT, (HPVOID)1, NULL},
    {0x1026, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sMotCtrl_UsrControl.sqIPQuota, &MotCtrl_RelativePositionScaling}, // IP Quota 32bit (in modalita' interpolatore)
    {0x1027, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SBYTE, 0, 1, WRDENY_DEFAULT,   &sMotCtrl_UsrControl.sbHomingSoftPosSwitch, NULL},
    {0x1028, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SBYTE, 0, 1, WRDENY_DEFAULT,   &sMotCtrl_UsrControl.sbHomingSoftNegSwitch, NULL},
    {0x1029, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SBYTE, 0, 1, WRDENY_DEFAULT,   &sMotCtrl_UsrControl.sbHomingSoftHomeSwitch, NULL},
    {0x102A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sMotCtrl_Out.slIPQuotaMonitor, NULL},
    {0x102B, COMMONPARAMDB_FLAG_RD|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMh_MotorDataOut.slIqFb, &MotCtrl_RatedTorqueScaling},
    {0x102C, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT,   &sMotCtrl_UsrControl.swTargetTorque, NULL},
    {0x102D, COMMONPARAMDB_FLAG_RD|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sMotCtrl_Out.sqUserOffset, &MotCtrl_RelativePositionScaling},

    {0x1030, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.swOptQuickStop},               // quickstop option code
    {0x1031, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.swOptShutdown},                // shutdown option code
    {0x1032, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.swOptDisableOp},               // disable operation option code
    {0x1033, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.swOptHalt},                    // halt option code
    {0x1034, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.swOptFaultReaction},           // fault reaction option code
    {0x1035, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.sqFollowingErrWindow, &MotCtrl_RelativePositionScaling}, // following error window 64bit scalato a 32bit
    {0x1036, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.swFollowingErrTimeout},        // following error timeout
    {0x1037, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.sqPositionWindow, &MotCtrl_RelativePositionScaling}, // target position window 64bit scalato a 32bit
    {0x1038, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.swPositionWinTimeout},         // target position timeout
    {0x1039, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.slVelocityWindow},             // velocity window
    {0x103A, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.swVelocityWinTimeout},         // velocity timeout
    {0x103B, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.slVelocityThreshold},          // velocity threshold
    {0x103C, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.swVelocityThrTimeout},         // velocity threshold timeout
    {0x103D, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.sqFollowingErrWindow.hi, NULL},// following error window 32bit: Turns
    {0x103E, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.sqFollowingErrWindow.lo, NULL},// following error window 32bit: Angle
    {0x103F, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.sqFollowingErrWindow, NULL},   // following error window 64bit
    {0x1040, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.sqPositionWindow.hi, NULL},    // target position window 32bit: Turns
    {0x1041, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.sqPositionWindow.lo, NULL},    // target position window 32bit: Angle
    {0x1042, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.sqPositionWindow, NULL},       // target position window 64bit
    {0x1043, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_POWERENABLED|WRDENY_PARAMSAVE, &sMotCtrlParameters.ubIPTimeUnits, NULL},          // IP time period units
    {0x1044, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SBYTE, 0, 1, WRDENY_POWERENABLED|WRDENY_PARAMSAVE, &sMotCtrlParameters.sbIPTimeIndex, NULL},          // IP time period index
    {0x1045, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_C_UBYTE, 0, 1, WRDENY_DEFAULT, (HPVOID)2, NULL},
    {0x1046, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.ubHomingMethod, NULL},
    {0x1047, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_POWERENABLED|WRDENY_PARAMSAVE, &sMotCtrlParameters.ubHomingSSrcPosSwitch, NULL},
    {0x1048, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_POWERENABLED|WRDENY_PARAMSAVE, &sMotCtrlParameters.ubHomingSSrcNegSwitch, NULL},
    {0x1049, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_POWERENABLED|WRDENY_PARAMSAVE, &sMotCtrlParameters.ubHomingSSrcHomeSwitch, NULL},
    {0x104A, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.slHomingSpeedSwitch},
    {0x104B, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.slHomingSpeedZero},
    {0x104C, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.slHomingAcceleration},
    {0x104D, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SQWRD, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.sqHomingUserOffset, NULL},
    {0x104E, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_UMCONV, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.sqHomingUserOffset, &MotCtrl_RelativePositionScaling},
    {0x104F, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sMotCtrlParameters.ulMotorRatedTorque},
    {0x1050, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT,   &sMotCtrlParameters.sqHomingUserOffset.hi, NULL},  // Home offset
    {0x1051, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sMotCtrlParameters.sqHomingUserOffset.lo, NULL},


        //****************************************************************************
        // Motor Parameters
        //****************************************************************************
    {0x2000, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.ulSerialNumber, NULL},
    {0x2001, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.ulProductionDate, NULL},
    {0x2002, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.ulModel, NULL},
    {0x2003, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.flResistance, NULL},
    {0x2004, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.flInductance, NULL},
    {0x2005, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.flKT, NULL},
    {0x2006, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.flCurrentNominalZeroSpeed, NULL},
    {0x2007, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.flCurrentNominal, NULL},
    {0x2008, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.flCurrentPeak, NULL},
    {0x2009, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.flSpeedNominal, NULL},
    {0x200A, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.uwThermalConstant, NULL},
    {0x200B, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.flMotorInertia, NULL},
    {0x200C, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.uwPhaseOffset, NULL},
    {0x200D, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.ulType, NULL},
    {0x200E, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.uwPoleNumbers, NULL},
    {0x200F, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.flCoolingTempOn, NULL},
    {0x2010, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.flCoolingTempOff, NULL},
    {0x2011, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.flMaximumTemp, NULL},
    {0x2012, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sGlbMotorParameters.flDirectInductance, NULL},


        //****************************************************************************
        // Thermal Model
        //****************************************************************************
    {0x3000, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.uwRBrakeValue, NULL},
    {0x3001, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.uwRBrakeMaxPower, NULL},
    {0x3002, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.swCoolingTempOn, NULL},
    {0x3003, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.swCoolingTempOff, NULL},
    {0x3004, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.swMotorOverTemp, NULL},
    {0x3005, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.ulRBrakeMaxEnergy, NULL},
	{0x3006, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.flags.b.bDisFltReactOnLimit, NULL},//{0x3006, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW,  0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.flags.w, NULL},
	{0x3007, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.flags.b.bNoBrakeDisableOnFlt, NULL},//{0x3007, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW,  1, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.flags.w, NULL},
	{0x3008, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.flags.b.bNoFatalOnBrakeFlt, NULL},//{0x3008, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW,  2, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.flags.w, NULL},

#ifdef _HW_DC
    {0x3010, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.sflPTCCoeff[0], NULL},
    {0x3011, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.sflPTCCoeff[1], NULL},
    {0x3012, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.sflPTCCoeff[2], NULL},
    {0x3013, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.sflPTCCoeff[3], NULL},
#endif
    {0x3018, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.sflKTYCoeff[0], NULL},
    {0x3019, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.sflKTYCoeff[1], NULL},
    {0x301A, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.sflKTYCoeff[2], NULL},
    {0x301B, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_PARAMSAVE, &sTm_ThModParam.sflKTYCoeff[3], NULL},
    {0x301F, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_C_UBYTE, 0, 1, WRDENY_DEFAULT, (HPVOID)4, NULL},

        /* ============== Monitor ============== */
    {0x3100, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swDcBusAvrg, NULL},   /* DcBus Average Value        (1e-1 V) */
    {0x3101, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.slIuPeakAvrg, NULL},  /* Iu Peak Average Value      (1e-4 A) */
    {0x3102, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.slIvPeakAvrg, NULL},  /* Iv Peak Average Value      (1e-4 A) */
    {0x3103, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.slIwPeakAvrg, NULL},  /* Iw Peak Average Value      (1e-4 A) */
    {0x3104, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swTNtc, NULL},        /* Temperature from NTC       (1e-1 Celsius) */
    {0x3105, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swPjUHTot, NULL},     /* Losses IGBT U-H            (W) */
    {0x3106, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swPjVHTot, NULL},     /* Losses IGBT V-H            (W) */
    {0x3107, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swPjWHTot, NULL},     /* Losses IGBT W-H            (W) */
    {0x3108, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swPjULTot, NULL},     /* Losses IGBT U-L            (W) */
    {0x3109, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swPjVLTot, NULL},     /* Losses IGBT V-L            (W) */
    {0x3110, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swPjWLTot, NULL},     /* Losses IGBT W-L            (W) */
    {0x3111, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.slPjTot, NULL},       /* Total Losses               (W) */
    {0x3112, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swIgbtTjMax, NULL},   /* Max TJunction for the IGBT (1e-1 Celsius) */
    {0x3113, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swTjUHi, NULL},       /* TJunction U (model)        (1e-1 Celsius) */
    {0x3114, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swTjVHi, NULL},       /* TJunction V (model)        (1e-1 Celsius) */
    {0x3115, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swTjWHi, NULL},       /* TJunction W (model)        (1e-1 Celsius) */
    {0x3116, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swTjMax, NULL},       /* Max TJunction   (model)    (1e-1 Celsius) */
    {0x3117, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.slRBrakePower, NULL}, /* Power Dissipated by RBrake (1e-3 W) */
    {0x3118, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swTHeatSink, NULL},   /* T HeatSink (model)         (1e-1 Celsius) */
    {0x3119, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sIqLimit.slMax, NULL},
    {0x3120, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sIqLimit.slMin, NULL},
    {0x3121, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sIdLimit.slMax, NULL},
    {0x3122, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sIdLimit.slMin, NULL},
    {0x3123, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.uwTaskLoad, NULL},
    {0x3124, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.flMotorTemp, NULL},  /* T Motor (polynomial)        (Celsius)     */
    {0x3125, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.swMotorTemp, NULL},  /* T Motor (polynomial)        (1e-1 Celsius) */
    {0x3126, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.uwBrakeResistorValue, NULL},
    {0x3127, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.uwBrakeResistorPower, NULL},
    {0x3128, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.ulBrakeResistorEnergy, NULL},
    {0x3129, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.ulOutValueRBrakeEnergy, NULL},
#ifdef _HW_DC
    {0x312A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.flMotorPTC, NULL},
    {0x312B, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.flMotorKTY, NULL},
    {0x312C, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.flBr0TNtc, NULL},
    {0x312D, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.flBr1TNtc, NULL},
    {0x312E, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.flBr2TNtc, NULL},
#endif
#ifdef _HW_CT
    {0x312F, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.flBrdTntc, NULL}, /* T Control Board (Celsius)     */
    {0x3130, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.swBrdTemp, NULL}, /* T Control Board (1e-1 Celsius) */
#endif

#ifndef _INFINEON_
    {0x3131, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.flSOnchipTemp, NULL},  /* T ZynQ (Celsius) */
    {0x3132, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_FLOAT, 0, 1, WRDENY_DEFAULT, &sTm_ThModOut.sOutValue.flSBoardTemp, NULL},   /* T controlboard NTC (Celsius) */
#endif
        //****************************************************************************
        // Sensor
        //****************************************************************************

        /* ============== Digital Accelerometer ADXL345 ============== */
#ifdef _HW_CT
    {0x4000, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW,  0, 1, WRDENY_PARAMSAVE,   (HPVOID)&sSensor_Parameters.flags.w, NULL}, // Sensor parameters
    {0x4001, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW,  1, 1, WRDENY_PARAMSAVE,   (HPVOID)&sSensor_Parameters.flags.w, NULL}, // Sensor parameters
    {0x4002, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW,  2, 1, WRDENY_PARAMSAVE,   (HPVOID)&sSensor_Parameters.flags.w, NULL}, // Sensor parameters

        /* ============== Digital Accelerometer ADXL345 ============== */    
    {0x4010, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sSensor_DataOut.swDataX, NULL},
    {0x4011, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sSensor_DataOut.swDataY, NULL},
    {0x4012, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sSensor_DataOut.swDataZ, NULL},

        /* ============== Humodity and Temperature SHT3X ============== */    
    {0x4013, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sSensor_DataOut.uwTemperature, NULL},
    {0x4014, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sSensor_DataOut.uwHumidity, NULL},
#endif


#ifdef _HW_AXS_CABI35KW
	/* ============== STGAP4S System Values ============== */
    {0x4100, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sConfigDrivers, NULL},
    {0x4101, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sResetStatus, NULL},
    {0x4102, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSoftReset, NULL},
    {0x4120, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT, &stgap4sTotalBytesTransferred, NULL},
    {0x4121, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT, &stgap4sTotalWrongRxCRCerrors, NULL},
	/* ============== STGAP4S Configuration Register Params  ============== */
	{0x4130, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &stgap4sCONFIG_PARAMS.Reg.CFG1, NULL},
    {0x4131, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &stgap4sCONFIG_PARAMS.Reg.CFG2, NULL},
    {0x4132, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &stgap4sCONFIG_PARAMS.Reg.CFG3, NULL},
    {0x4133, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &stgap4sCONFIG_PARAMS.Reg.CFG4, NULL},
    {0x4134, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &stgap4sCONFIG_PARAMS.Reg.CFG5, NULL},
    {0x4135, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &stgap4sCONFIG_PARAMS.Reg.CFG6, NULL},
    {0x4136, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &stgap4sCONFIG_PARAMS.Reg.CFG7, NULL},
    {0x4137, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &stgap4sCONFIG_PARAMS.Reg.DIAG1CFGA, NULL},
    {0x4138, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &stgap4sCONFIG_PARAMS.Reg.DIAG1CFGB, NULL},
    {0x4139, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &stgap4sCONFIG_PARAMS.Reg.DIAG2CFGA, NULL},
    {0x4140, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &stgap4sCONFIG_PARAMS.Reg.DIAG2CFGB, NULL},
	/* ============== STGAP4S Phase U Status Registers ============== */
    {0x4150, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.U_H.Reg.STATUS1, NULL},
    {0x4151, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.U_H.Reg.STATUS2, NULL},
    {0x4152, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.U_H.Reg.STATUS3, NULL},
    {0x4153, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.U_H.Reg.STATUS4, NULL},
    {0x4154, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.U_H.Reg.STATUS5, NULL},
    {0x4155, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.U_L.Reg.STATUS1, NULL},
    {0x4156, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.U_L.Reg.STATUS2, NULL},
    {0x4157, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.U_L.Reg.STATUS3, NULL},
    {0x4158, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.U_L.Reg.STATUS4, NULL},
    {0x4159, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.U_L.Reg.STATUS5, NULL},
    /* ============== STGAP4S Phase U Status Registers ============== */
    {0x4160, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.V_H.Reg.STATUS1, NULL},
    {0x4161, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.V_H.Reg.STATUS2, NULL},
    {0x4162, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.V_H.Reg.STATUS3, NULL},
    {0x4163, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.V_H.Reg.STATUS4, NULL},
    {0x4164, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.V_H.Reg.STATUS5, NULL},
    {0x4165, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.V_L.Reg.STATUS1, NULL},
    {0x4166, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.V_L.Reg.STATUS2, NULL},
    {0x4167, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.V_L.Reg.STATUS3, NULL},
    {0x4168, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.V_L.Reg.STATUS4, NULL},
    {0x4169, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.V_L.Reg.STATUS5, NULL},
    /* ============== STGAP4S Phase W Status Registers ============== */
    {0x4170, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.W_H.Reg.STATUS1, NULL},
    {0x4171, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.W_H.Reg.STATUS2, NULL},
    {0x4172, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.W_H.Reg.STATUS3, NULL},
    {0x4173, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.W_H.Reg.STATUS4, NULL},
    {0x4174, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.W_H.Reg.STATUS5, NULL},
    {0x4175, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.W_L.Reg.STATUS1, NULL},
    {0x4176, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.W_L.Reg.STATUS2, NULL},
    {0x4177, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.W_L.Reg.STATUS3, NULL},
    {0x4178, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.W_L.Reg.STATUS4, NULL},
    {0x4179, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sSTATUS_VALUES.Phase.W_L.Reg.STATUS5, NULL},
	/* ============== STGAP4S System Values ============== */
	{0x4180, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sConfigValuesIndex, NULL},
	/* ============== STGAP4S Configuration Register Values ============== */
	{0x4190, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sCONFIG_VALUES.Reg.CFG1, NULL},
    {0x4191, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sCONFIG_VALUES.Reg.CFG2, NULL},
    {0x4192, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sCONFIG_VALUES.Reg.CFG3, NULL},
    {0x4193, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sCONFIG_VALUES.Reg.CFG4, NULL},
    {0x4194, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sCONFIG_VALUES.Reg.CFG5, NULL},
    {0x4195, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sCONFIG_VALUES.Reg.CFG6, NULL},
    {0x4196, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sCONFIG_VALUES.Reg.CFG7, NULL},
    {0x4197, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sCONFIG_VALUES.Reg.DIAG1CFGA, NULL},
    {0x4198, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sCONFIG_VALUES.Reg.DIAG1CFGB, NULL},
    {0x4199, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sCONFIG_VALUES.Reg.DIAG2CFGA, NULL},
    {0x4200, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &stgap4sCONFIG_VALUES.Reg.DIAG2CFGB, NULL},
#endif



        //****************************************************************************
        // Drive Task Configuration
        //****************************************************************************
    {0x5000, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &sDrvTskCtrlParams.sNew.ubMainOperation, NULL},                /* Tipo di drive */


        //****************************************************************************
        // PLC
        //****************************************************************************
    {0x7C00, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulPlcRegInfoArea, NULL},
    {0x7C01, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulPlcRegDownloadAddress, NULL},
    {0x7C02, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulPlcRegApplID, NULL},
    {0x7C03, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_NONE, &uwPlcRegCommand, NULL},
    {0x7C04, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_NONE, &uwPlcRegDownloadResponse, NULL},
    {0x7C05, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulPlcRegMemoID, NULL},
    {0x7C06, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulPlcRegFullMemoID, NULL},
    {0x7C07, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulPlcRegCommandResponse, NULL},

    {0x7C08, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, PLC_APP_NAME_SIZE/sizeof(ULONG), WRDENY_NONE, sPlcAppInfo.chAppName, NULL},
    {0x7C09, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_NONE, &sPlcAppInfo.uwMajor, NULL},
    {0x7C0A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_NONE, &sPlcAppInfo.uwMinor, NULL},

    {0x7C10, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, PLC_PARCOM_SIZE, WRDENY_PARAMSAVE, &tPlcParamCommon.uwCommon[0], NULL},
    {0x7C11, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITW, -1, PLC_PARBIT_SIZE, WRDENY_PARAMSAVE, &tPlcParamBits.uwBools[0], NULL},
    {0x7C12, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, PLC_WRKCOM_SIZE, WRDENY_DEFAULT,   &uwPlcWorksCommon[0], NULL},
    {0x7C13, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_BITW, -1, PLC_WRKBIT_SIZE, WRDENY_DEFAULT,   &uwPlcWorksBools[0], NULL},

    {0x7C40, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW,  0, 1, WRDENY_PARAMSAVE,   (HPVOID)&sPlcParams.flags.w, NULL}, // disable plc execution

    {0x7C50, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW,  0, 1, WRDENY_DEFAULT,   (HPVOID)&bPlcSysCommands, NULL}, // reboot minimal requested

        //****************************************************************************
        // SoftScope
        //****************************************************************************
    {0x7C80, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_NONE, &uwScoCommand, NULL},
    {0x7C81, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulScoData, NULL},
    {0x7C82, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_NONE, &uwScoTriggerRequiredMode, NULL},
    {0x7C83, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulScoAcquisitionId, NULL},
    {0x7C84, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_NONE, &uwScoTriggerStatus, NULL},
    {0x7C85, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulScoSampleCount, NULL},
    {0x7C86, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulScoTriggerPosition, NULL},
    {0x7C87, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulScoAcquiredDataId, NULL},
#if ALSSC_RUNTIME_ADV
    {0x7C88, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulScoAbsTimeRef, NULL},
    {0x7C89, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &ulScoRTBuffSwitch, &SoftScopeDataSwitch},
#endif

    {0x7C90, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UWORD, 0, sizeof(ulScoMemory)/sizeof(UWORD), WRDENY_NONE, ulScoMemory, &SoftScopeReqMemParHook},

        //****************************************************************************
        // Remote Display HMI Server
        //****************************************************************************
#ifdef CFG_EN_MODBUSOVERCAN
    {0x7CA0, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &sRDHmiSrvImgSys.ulBaseAddress, NULL},
    {0x7CA1, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &sRDHmiSrvImgSys.ulCodeID, NULL},
    {0x7CA2, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &sRDHmiSrvImgSys.ulFileSize, NULL},
    {0x7CA3, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &sRDHmiSrvImgApp.ulBaseAddress, NULL},
    {0x7CA4, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &sRDHmiSrvImgApp.ulCodeID, NULL},
    {0x7CA5, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_NONE, &sRDHmiSrvImgApp.ulFileSize, NULL},
    {0x7CA6, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_NONE, (HPVOID)&uwRDHmiSrvCompatLevel, NULL},
#endif

        //****************************************************************************
        // User I/O
        //****************************************************************************
    {0x7E00, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPBase.sDIO.uwInputs, NULL},
    {0x7E01, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPBase.sDIO.uwOutputs, NULL},
    {0x7E02, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swInputs[0], NULL},
    {0x7E03, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swInputs[1], NULL},
    {0x7E04, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swInputs[2], NULL},
    {0x7E05, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swOutputs[0], NULL},
    {0x7E06, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swOutputs[1], NULL},
    {0x7E07, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swOutputs[2], NULL},
    {0x7E08, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swOutputs[3], NULL},
    {0x7E09, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swInputs[3], NULL},

    {0x7E0A, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swInputs[4], NULL},
    {0x7E0B, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swInputs[5], NULL},
    {0x7E0C, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swInputs[6], NULL},
    {0x7E0D, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swOutputs[4], NULL},
    {0x7E0E, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.sAIO.swOutputs[5], NULL},

    {0x7E10, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_BITW, -1, 16,WRDENY_DEFAULT, &sUsrIOCPBase.sDIO.uwInputs, NULL},
    {0x7E11, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_BITW, -1, 10,WRDENY_DEFAULT, &sUsrIOCPBase.sDIO.uwOutputs, NULL},

    {0x7E21, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPBase.uwDgOutForceOn, NULL},
    {0x7E22, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPBase.uwDgOutForceOff, NULL},
    {0x7E23, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPBase.uwDgInForceOn, NULL},
    {0x7E24, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPBase.uwDgInForceOff, NULL},

    {0x7E30, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPBase.uwAnInSimulation, NULL},
    {0x7E31, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPBase.uwAnOutSimulation, NULL},
    {0x7E32, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.swOutSimulation[0], NULL},
    {0x7E33, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.swOutSimulation[1], NULL},
    {0x7E34, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.swOutSimulation[2], NULL},
    {0x7E35, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.swOutSimulation[3], NULL},
    {0x7E36, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.swOutSimulation[4], NULL},
    {0x7E37, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUsrIOCPData.swOutSimulation[5], NULL},

    {0x7E40, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sUserIOCPDefs.uwSizes, NULL},
    {0x7E41, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT, &sUserIOCPDefs.hpsAdrBase, NULL},
    {0x7E42, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT, &sUserIOCPDefs.hpsAdrData, NULL},

        //****************************************************************************
        // CanOpen
        //****************************************************************************
#if CFG_CANDRV_CMDMGR
    {0x8000, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_SWORD, 0, 1, WRDENY_PARAMSAVE, &tCanOpenCMParams.swCanController, NULL},
#endif

#if CFG_CANDRV_DS301
    {0x8001, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &tDs301Param.bLssNodeId, NULL},
    {0x8002, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &tDs301Param.bLssTimingIndex, NULL},
    {0x8003, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE, &tDs301Param.wEmcyInhibitTime, NULL},
    {0x8004, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &tDs301Param.tSyncCob, NULL},
    {0x8005, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &tDs301Param.tEmcyCob, NULL},
    {0x8006, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE, &tDs301Param.wGuardTime, NULL},
    {0x8007, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_PARAMSAVE, &tDs301Param.wHeartbeatTime, NULL},
    {0x8008, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_PARAMSAVE, &tDs301Param.bLifeTimeFactor, NULL},
    {0x8009, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &tDs301Param.dCommCyclePeriod, NULL},
#endif

#if CFG_CANDRV_CMDMGR
    {0x800B, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_PARAMSAVE, &tCanOpenCMParams.ulDisableCanAlarmMask, NULL},
	{0x800C, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE,   &tCanOpenCMParams.flags.f.bReSyncEnable, NULL},
	{0x800D, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE,   &tCanOpenCMParams.flags.f.bSyncEvOnPeriodicCob, NULL},
    {0x800E, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE,   &tCanOpenCMParams.flags.f.bDelayedInputs, NULL},
#endif

#if CFG_CANDRV_DS301
    {0x8020, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   (HPVOID)&bDs301ErrorRegister},
    {0x8021, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   (HPVOID)&dDs301ErrRegManufacturer},
    {0x8022, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   (HPVOID)&wDs301ErrCodeLast},

    {0x8040, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_C_UBYTE, 0, 1, WRDENY_DEFAULT, (HPVOID)4, NULL},

    {0x80C0, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, CANOPENPSM_NSUBINDX_RX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[0], &CanOpenPSM_PdoParam},
    {0x80C1, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, CANOPENPSM_NSUBINDX_RX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[1], &CanOpenPSM_PdoParam},
    {0x80C2, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, CANOPENPSM_NSUBINDX_RX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[2], &CanOpenPSM_PdoParam},
    {0x80C3, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, CANOPENPSM_NSUBINDX_RX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[3], &CanOpenPSM_PdoParam},
    {0x80C4, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, CANOPENPSM_NSUBINDX_RX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[4], &CanOpenPSM_PdoParam},
    {0x80C5, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, CANOPENPSM_NSUBINDX_RX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[5], &CanOpenPSM_PdoParam},
    {0x80C6, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, CANOPENPSM_NSUBINDX_RX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[6], &CanOpenPSM_PdoParam},
    {0x80C7, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, CANOPENPSM_NSUBINDX_RX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[7], &CanOpenPSM_PdoParam},
    {0x80D0, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[0], &CanOpenPSM_PdoMap},
    {0x80D1, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[1], &CanOpenPSM_PdoMap},
    {0x80D2, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[2], &CanOpenPSM_PdoMap},
    {0x80D3, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[3], &CanOpenPSM_PdoMap},
    {0x80D4, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[4], &CanOpenPSM_PdoMap},
    {0x80D5, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[5], &CanOpenPSM_PdoMap},
    {0x80D6, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[6], &CanOpenPSM_PdoMap},
    {0x80D7, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamRx[7], &CanOpenPSM_PdoMap},
    {0x80E0, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, CANOPENPSM_NSUBINDX_TX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[0], &CanOpenPSM_PdoParam},
    {0x80E1, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, CANOPENPSM_NSUBINDX_TX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[1], &CanOpenPSM_PdoParam},
    {0x80E2, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, CANOPENPSM_NSUBINDX_TX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[2], &CanOpenPSM_PdoParam},
    {0x80E3, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, CANOPENPSM_NSUBINDX_TX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[3], &CanOpenPSM_PdoParam},
    {0x80E4, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, CANOPENPSM_NSUBINDX_TX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[4], &CanOpenPSM_PdoParam},
    {0x80E5, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, CANOPENPSM_NSUBINDX_TX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[5], &CanOpenPSM_PdoParam},
    {0x80E6, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, CANOPENPSM_NSUBINDX_TX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[6], &CanOpenPSM_PdoParam},
    {0x80E7, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, CANOPENPSM_NSUBINDX_TX+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[7], &CanOpenPSM_PdoParam},
    {0x80F0, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[0], &CanOpenPSM_PdoMap},
    {0x80F1, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[1], &CanOpenPSM_PdoMap},
    {0x80F2, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[2], &CanOpenPSM_PdoMap},
    {0x80F3, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[3], &CanOpenPSM_PdoMap},
    {0x80F4, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[4], &CanOpenPSM_PdoMap},
    {0x80F5, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[5], &CanOpenPSM_PdoMap},
    {0x80F6, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[6], &CanOpenPSM_PdoMap},
    {0x80F7, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, CANOPENPSM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_DS301PDOPARAMS,
                (HPVOID)&tDs301PdoParamTx[7], &CanOpenPSM_PdoMap},
#endif
        //****************************************************************************
        // EtherCAT
        //****************************************************************************
#if CFG_ECAT
    {0x8100, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_C_UBYTE, 0, 1, WRDENY_DEFAULT, (HPVOID)1, NULL},
    {0x8101, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sParMgmParamSignature.uwBlockCRC, NULL},

    {0x8102, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, sizeof(tEcatCMSyncMgrCommType)/sizeof(UBYTE), WRDENY_DEFAULT,
                (HPVOID)&tEcatCMSyncMgrCommType[0], NULL},

    {0x8103, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_C_UBYTE, 0, 1, WRDENY_DEFAULT, (HPVOID)4, NULL},
    {0x8104, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &tEcatCMCycleDiag.smEventMissedCounter, NULL},
    {0x8105, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &tEcatCMCycleDiag.shiftTooShortCounter, NULL},
    {0x8106, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &tEcatCMCycleDiag.cycleExceededCounter, NULL},
    {0x8107, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &tEcatCMCycleDiag.syncFailedCounter, NULL},
	{0x8108, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE,   &tEcatCMParam.flags.f.bEnableModule, NULL},//{0x8108, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW,  0, 1, WRDENY_PARAMSAVE,   &tEcatCMParam.flags.w, NULL},
	{0x8109, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE,   &tEcatCMParam.flags.f.bReSyncEnable, NULL},//{0x8109, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW,  1, 1, WRDENY_PARAMSAVE,   &tEcatCMParam.flags.w, NULL},
	{0x810A, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE,   &tEcatCMParam.flags.f.bDelayedInputs, NULL},//{0x810A, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW,  2, 1, WRDENY_PARAMSAVE,   &tEcatCMParam.flags.w, NULL},

    {0x8110, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_C_UBYTE, 0, 1, WRDENY_DEFAULT, (HPVOID)32, NULL},
    {0x8111, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &tEcatCMSMOutParam.u16SyncType, NULL},
    {0x8112, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tEcatCMSMOutParam.u32CycleTime, NULL},
    {0x8113, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tEcatCMSyncManOutDiag.u32ShiftTime, NULL},
    {0x8114, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &tEcatCMSyncManOutDiag.u16SyncTypesSupported, NULL},
    {0x8115, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tEcatCMSyncManOutDiag.u32MinCycleTime, NULL},
    {0x8116, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tEcatCMSyncManOutDiag.u32CalcAndCopyTime, NULL},
    {0x8117, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tEcatCMSyncManOutDiag.u32DelayTime, NULL},
    {0x8118, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tEcatCMSyncManOutDiag.u32Sync0CycleTime, NULL},
    {0x8119, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tEcatCMSyncManOutDiag.u32CycleTimeTooSmall, NULL},
    {0x811A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tEcatCMSyncManOutDiag.u32SMEventMissed, NULL},
    {0x811B, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &tEcatCMSyncManOutDiag.bSyncError, NULL},
    {0x811C, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &tEcatCMSMInParam.u16SyncType, NULL},
    {0x811D, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tEcatCMSMInParam.u32CycleTime, NULL},
    {0x811E, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tEcatCMSyncManInDiag.u32ShiftTime, NULL},
    {0x811F, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tEcatCMSyncManInDiag.u32CalcAndCopyTime, NULL},
    {0x8120, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tEcatCMSyncManInDiag.u32DelayTime, NULL},

    {0x8130, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_RX, ECATCM_MAXNPDO+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMOutSyncPdoMap, &EcatCM_SyncMgrPdoMap},
    {0x8131, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_TX, ECATCM_MAXNPDO+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMInSyncPdoMap, &EcatCM_SyncMgrPdoMap},

    {0x8140, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMRxPdoMap[0], &EcatCM_PdoMap},
    {0x8141, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMRxPdoMap[1], &EcatCM_PdoMap},
    {0x8142, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMRxPdoMap[2], &EcatCM_PdoMap},
    {0x8143, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMRxPdoMap[3], &EcatCM_PdoMap},
    {0x8144, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMRxPdoMap[4], &EcatCM_PdoMap},
    {0x8145, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMRxPdoMap[5], &EcatCM_PdoMap},
    {0x8146, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMRxPdoMap[6], &EcatCM_PdoMap},
    {0x8147, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_RX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMRxPdoMap[7], &EcatCM_PdoMap},
    {0x8150, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMTxPdoMap[0], &EcatCM_PdoMap},
    {0x8151, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMTxPdoMap[1], &EcatCM_PdoMap},
    {0x8152, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMTxPdoMap[2], &EcatCM_PdoMap},
    {0x8153, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMTxPdoMap[3], &EcatCM_PdoMap},
    {0x8154, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMTxPdoMap[4], &EcatCM_PdoMap},
    {0x8155, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMTxPdoMap[5], &EcatCM_PdoMap},
    {0x8156, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMTxPdoMap[6], &EcatCM_PdoMap},
    {0x8157, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_UBYTE, ECATCM_PAR_TX, DS301_PDO_MAXDATAOBJECT+1, WRDENY_ECATPDOPARAMS,
                (HPVOID)&tEcatCMTxPdoMap[7], &EcatCM_PdoMap},

    {0x8180, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.sPort[0].ubInvFrameCnt, NULL},
    {0x8181, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.sPort[0].ubRxErrCnt, NULL},
    {0x8182, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.sPort[0].ubForwardedRxErrCnt, NULL},
    {0x8183, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.sPort[0].ubLostLinkCnt, NULL},
    {0x8184, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.sPort[1].ubInvFrameCnt, NULL},
    {0x8185, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.sPort[1].ubRxErrCnt, NULL},
    {0x8186, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.sPort[1].ubForwardedRxErrCnt, NULL},
    {0x8187, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.sPort[1].ubLostLinkCnt, NULL},
    {0x8188, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.ubProcUnitErrCnt, NULL},
    {0x8189, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.ubPDIErrCnt, NULL},
    {0x818A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.sFlags.ubFlags[0], NULL},
    {0x818B, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.sFlags.ubFlags[1], NULL},
    {0x818C, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.sFlags.ubFlags[2], NULL},
    {0x818D, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.sFlags.ubFlags[3], NULL},
    {0x818E, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &sEcatDiag.ubALStatus, NULL},
    {0x818F, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sEcatDiag.uwALStatusCode, NULL},
    {0x8190, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &sEcatDiag.uwSyncOutputMissingCnt, NULL},

    {0x81A0, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT, &ubEcatEmcyErrReg},
    {0x81A1, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT, &uwEcatEmcyErrCode},
#endif

        //****************************************************************************
        // Common CanOpen/COE
        //****************************************************************************
    {0x8200, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tDs301Identity.ulDeviceType},
    {0x8201, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tDs301Identity.ulVendorID},
    {0x8202, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tDs301Identity.ulProductCode},
    {0x8203, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &tDs301Identity.ulRevision},
    {0x8204, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_STRING,0, 1, WRDENY_DEFAULT,   (HPVOID)sCanOpenCOEManufacturerName},
    {0x8205, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_STRING,0, 1, WRDENY_DEFAULT,   (HPVOID)sCanOpenCOEDeviceNameBase},
    {0x8206, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_STRING,0, 1, WRDENY_DEFAULT,   (HPVOID)sCanOpenCOESWVersion},

    {0x8210, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   NULL, &CanOpenCOE_ParamSave},
    {0x8211, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   NULL, &CanOpenCOE_ParamRestore},

#ifdef _APP_XC
    // {0x8220, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_HOOK, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   NULL, &CanOpenCOE_FwDownload},
#endif

        //****************************************************************************
        // EtherPMC
        //****************************************************************************
#if CFG_ETHPMC
	{0x8280, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_BITW,  0, 1, WRDENY_PARAMSAVE,   &sEpmcCM_Params.flags.w, NULL},//{0x8280, COMMONPARAMDB_FLAG_RW|COMMONPARAMDB_FLAG_RESETREQ, COMMONPARAMDB_TYPE_UBYTE,  0, 1, WRDENY_PARAMSAVE,   &sEpmcCM_Params.flags.f.bEnableModule, NULL},//

    {0x8288, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEpmcCMInOut.uwControl, NULL},
    {0x8289, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEpmcCMInOut.uwStatus, NULL},
    {0x828A, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEpmcCM_Params.dev.uwSlaves, NULL},

    {0x8290, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEpmcCMDiag.uwMissingPacket, NULL},
    {0x8291, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UWORD, 0, 1, WRDENY_DEFAULT,   &sEpmcCMDiag.uwCRCErrorCounter, NULL},
    {0x8292, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG, 0, 1, WRDENY_DEFAULT,   &sEpmcCMDiag.ulLineDelay, NULL},
    {0x8293, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEpmcCMDiag.sFlags.ubFlags[0], NULL},
    {0x8294, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_UBYTE, 0, 1, WRDENY_DEFAULT,   &sEpmcCMDiag.sFlags.ubFlags[1], NULL},
#endif // cfg_ethpmc

        //****************************************************************************
        // Sync Manager
        //****************************************************************************
    {0x8300, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD,0, 1, WRDENY_PARAMSAVE,   &sSyncMgrParam.uwTSFilter},
    {0x8301, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SLONG,0, 1, WRDENY_PARAMSAVE,   &sSyncMgrParam.slReSyncDelta},
    {0x8302, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD,0, 1, WRDENY_PARAMSAVE,   &sSyncMgrParam.swFiltKp},
    {0x8303, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_SWORD,0, 1, WRDENY_PARAMSAVE,   &sSyncMgrParam.swFiltKd},
    {0x8304, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_UWORD,0, 1, WRDENY_PARAMSAVE,   &sSyncMgrParam.uwPeakNDiscard},
    {0x8305, COMMONPARAMDB_FLAG_RW, COMMONPARAMDB_TYPE_ULONG,0, 1, WRDENY_PARAMSAVE,   &sSyncMgrParam.ulPeakThreshold},

    {0x8310, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG,0, 1, WRDENY_DEFAULT,     &sSyncMgrDiagnosticOut.ulSyncTime},
    {0x8311, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG,0, 1, WRDENY_DEFAULT,     &sSyncMgrDiagnosticOut.ulSyncMax},
    {0x8312, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_ULONG,0, 1, WRDENY_DEFAULT,     &sSyncMgrDiagnosticOut.ulSyncMin},
    {0x8313, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SBYTE,0, 1, WRDENY_DEFAULT,     &sSyncMgrDiagnosticOut.bValid},
    {0x8315, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_SLONG,0, 1, WRDENY_DEFAULT,     &sSyncMgrDiagnosticOut.slSyncInstValue},

    {0x8320, COMMONPARAMDB_FLAG_RD, COMMONPARAMDB_TYPE_C_UBYTE, 0, 1, WRDENY_DEFAULT, (HPVOID)4, NULL},

};

const UWORD uwCommonParamCount=sizeof(psCommonParamTable)/sizeof(COMMONPARAMDB_ENTRY);
