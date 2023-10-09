/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright $)AB) 2005,2009, Phase Motion Control.                             */
/*           $)AB) 2021 Ningbo Physis Technology Co.,Ltd. All Rights Reserved.  */
/*                                                                          */
/* File        : AlPlcUserTabs.c                                            */
/* Author      : Fabio Terrile, Hu Xiaokai                                  */
/*               Axel                                                       */
/*                                                                          */
/* Description : User specific declarations or includes for:                */
/*                                                                          */
/*  - (*) firmware control variables (used in PlcIECUserConf.h)             */
/*  - (*) runtime callbacks (used in PlcIECUserConf.h)                      */
/*  - firmware embedded functions (linked in the function table below)      */
/*  - firmware data declarations (linked in the data block table below)     */
/*  - PLC execution functions (linked in the task table below)              */
/*                                                                          */
/*  (*) External declaration has to be put into PlcIECData.c                */
/*                                                                          */
/*  Put here user definitions and/or include statements...                  */
/*                                                                          */
/****************************************************************************/
#pragma GCC optimize (2)

#include <math.h>

#include "common\CommonDefines.h"
#include "common\MathFunctions.h"
#include "common\Int64Functions.h"
#include "common\DspFunctions.h"

#include "AlPlcRuntime2\AlPlcAreaDef.h"
#include "AlPlcRuntime2\AlPlcRuntimeCore.h"
#include "AlPlcRuntime2\AlPlcTarg.h"
#include "AlPlcRuntime2\AlPlcUserData.h"
#include "PlcComDBMgr.h"

#include "system\SysAppGlobals.h"
#include "system\SysLogManagement.h"
#include "system\SystemStatus.h"
#include "drive\HardwareConfig.h"
#include "common\ParametersCheck.h"
#include "common\TaskScheduler.h"
#include "drive\UserIO.h"
#include "drive\MotorHandler.h"
#include "drive\Deflux.h"
#include "drive\MotionController.h"
#include "drive\SpaceSpeedCntrLp.h"
#include "drive\Positioner.h"
#include "drive\PiDcBusCntrLp32bit.h"
#include "drive\ThermalModel.h"
#include "bus\modbus\ModBusCommandMgr.h"
#include "bus\modbus\ModBusCommandMgrParams.h"
#include "fpga\FpgaHandler.h"
#include "drive\DriveTaskController.h"
#include "system\SysAppInfo.h"
#include "common\IOManager.h"

#include "drive\EncoderManager.h"
#include "drive\EncoderEFSeek.h"
#include "drive\EnDatEncoder.h"
#include "drive\HallEncoder.h"
#include "drive\HiperfaceEncoder.h"
#include "drive\IncrementalEncoder.h"
#include "drive\SincosEncoder.h"
#include "drive\BackEmfEncoder.h"
#include "drive\NikonEncoder.h"
#include "drive\NikonHandlers.h"
#include "drive\TAMAGAWAEncoder.h"
#include "drive\TAMAGAWAHandlers.h"

#if CFG_CANDRV_CMDMGR
#include "bus\canopen\CanOpenCommandMgr.h"
#include "bus\canopen\MultiCANController.h"
#endif
#include "bus\canopen\CanOpenDs301.h"
#if CFG_CANDRV_DS301
#include "bus\canopen\CanOpenPdoSyncMgr.h"
#endif

#if CFG_ECAT
#include "bus\ethercat\ECATCommandMgr.h"
#endif // cfg_ecat

#include "bus\SyncManager.h"

#if CFG_ETHPMC
#include "bus\ethpmc\EthPMCCommandMgr.h"
#include "bus\ethpmc\EthPMCHw.h"
#endif // cfg_ethpmc

#if CFG_PSU_MNGR
#include "drive\PSUManager.h"
#endif // cfg_psu_mngr

#include "Plc.h"
#include "PlcRT.h"
#include "SoftScope.h"

#ifndef INFINEON
#include "AlPlcRuntime5\AlPlcBasicFun.h"
#include "AlplcRuntime2\AlPlcReal.h"
#include "AlplcRuntime2\AlPlcMath.h"
#endif

//****************************************************************************
// Runtime task definition tables

/*  Fields description
    ------------------

    taskId:     task ID for IEC compiler
    fnzExe:     address of PLC exec function
    fnzInit:    address of PLC init task variables function
    fnzInput:   address of input image function
    fnzOutput:  address of output image function
    rqInit      address of the flag for init request
    prgName     not used
    lineExe     not used
	exePeriod   not used
	exeTime     not used
	exeCount    not used
	exeStatus   not used
    taskValid   address of the flag for marking task validity

*/
PLC_ATTR_PLCIEC_TASKS plcTasksFunctions[] =
{
    {0, (void **)&sPlcTaskFast.pfRun,       (void **)&sPlcTaskFast.pfInit,         (void **)&sPlcTaskFast.pfInput,        (void **)&sPlcTaskFast.pfOutput,       &sPlcTaskFast.bReqInit,       0, 0, NULL, NULL, NULL, NULL, &sPlcTaskFast.bTaskValid},
    {1, (void **)&sPlcTaskSlow.pfRun,       (void **)&sPlcTaskSlow.pfInit,         (void **)&sPlcTaskSlow.pfInput,        (void **)&sPlcTaskSlow.pfOutput,       &sPlcTaskSlow.bReqInit,       0, 0, NULL, NULL, NULL, NULL, &sPlcTaskSlow.bTaskValid},
    {2, (void **)&sPlcTaskBackground.pfRun, (void **)&sPlcTaskBackground.pfInit,   (void **)&sPlcTaskBackground.pfInput,  (void **)&sPlcTaskBackground.pfOutput, &sPlcTaskBackground.bReqInit, 0, 0, NULL, NULL, NULL, NULL, &sPlcTaskBackground.bTaskValid},
    {3, (void **)&sPlcTaskRTHook.pfRun,     (void **)&sPlcTaskRTHook.pfInit,       (void **)&sPlcTaskRTHook.pfInput,      (void **)&sPlcTaskRTHook.pfOutput,     &sPlcTaskRTHook.bReqInit,     0, 0, NULL, NULL, NULL, NULL, &sPlcTaskRTHook.bTaskValid},
    {4, (void **)&sPlcTaskBootConfig.pfRun, (void **)&sPlcTaskBootConfig.pfInit,   (void **)&sPlcTaskBootConfig.pfInput,  (void **)&sPlcTaskBootConfig.pfOutput, &sPlcTaskBootConfig.bReqInit, 0, 0, NULL, NULL, NULL, NULL, &sPlcTaskBootConfig.bTaskValid},
    {5, (void **)&sPlcTaskBootInit.pfRun,   (void **)&sPlcTaskBootInit.pfInit,     (void **)&sPlcTaskBootInit.pfInput,    (void **)&sPlcTaskBootInit.pfOutput,   &sPlcTaskBootInit.bReqInit,   0, 0, NULL, NULL, NULL, NULL, &sPlcTaskBootInit.bTaskValid},
    {6, (void **)&sPlcTaskSystem.pfRun,     (void **)&sPlcTaskSystem.pfInit,       (void **)&sPlcTaskSystem.pfInput,      (void **)&sPlcTaskSystem.pfOutput,     &sPlcTaskSystem.bReqInit,     0, 0, NULL, NULL, NULL, NULL, &sPlcTaskSystem.bTaskValid},
    {7, (void **)&sPlcTaskDSPHook.pfRun,    (void **)&sPlcTaskDSPHook.pfInit,      (void **)&sPlcTaskDSPHook.pfInput,     (void **)&sPlcTaskDSPHook.pfOutput,    &sPlcTaskDSPHook.bReqInit,    0, 0, NULL, NULL, NULL, NULL, &sPlcTaskDSPHook.bTaskValid},
};

#ifdef _APP_DEBUG
const UWORD plcCntTasksFunctions=sizeof(plcTasksFunctions)/sizeof(PLCIEC_TASKS);
#endif

PLC_ATTR_PLCIEC_TASKDEF plcTasksDefs[] =
{
/*  Fields description
    ------------------

    taskId:     Task ID (the same used in LogicLab resources declaration
    name:       Task name
    flags:      Task characteristics (no image, normal image, multi-phase image)
    usExecTime: execution time in microseconds
    spare:      reserved for future uses

    taskId  name           flags               usExecTime  spare */
    {0,     "Fast",        TRGT_EMPTY,         125,        0},
    {1,     "Slow",        TRGT_HASIMAGE,        0,        0},
#ifdef INFINEON
    {2,     "Background",  TRGT_MULTIIMAGE,      0,        0},	// feature used only by Phase/Physis
#else
    {2,     "Background",  TRGT_HASIMAGE,        0,        0},	// only one image
#endif
	{3,     "OnDriveHook", TRGT_EMPTY,         125,        0},
    {4,     "PreBoot",     TRGT_EMPTY,           0,        0},
    {5,     "Boot",        TRGT_EMPTY,           0,        0},
    {6,     "System",      TRGT_EMPTY,           0,        0},
    {7,     "OnDSPHook",   TRGT_EMPTY,         125,        0},
};

#ifdef _APP_DEBUG
const UWORD plcCntTasksDefs=sizeof(plcTasksDefs)/sizeof(PLCIEC_TASKDEF);
#endif

//****************************************************************************
// Data blocks definitions for PLC IEC61131

PLC_ATTR_PLCIEC_DBREC plcDataBlocks[] = 
{
/*  Fields description
    ------------------

    img:        if >= 1 the compiler generates an image of the target variable and two (in/out) 
                procedures to manage the copy to/from the image variable and the target variable
                if > 1 then the procedures take a parameter that must be equals to this field 
                for copying the variable
    type:       is the IEC 1131 data block type Input/Output/Memory i.e. the I/Q/M
                of IEC language (ex.: the I of %IW10.4)
    dbId:       is the datablock index i.e. the first number of the IEC declaration
                (ex.: the 10 of %IW10.4)
    addr:       is the datablock base address
    nel:        is the number of elements of the datablock i.e. the last index + 1 of the datablock
    dataSize:   is the size of the element fo the datablock, the physical address is 
                phy = addr + dataSize * idx where idx is the second number of the IEC declaration
                (the 4 of %IW10.4)
    rw:         is the access allowed to the datablock elements (read only, write only or read/write)

    img      type      dbId  addr                                      nel                                          dataSize               rw */
        // Encoder Data
    {1,  DBTY_MEMO,    10,   (uint32_t)&sEm_Fbk2CntrLoop,              sizeof(sEm_Fbk2CntrLoop),                           1,              DBRW_R },
    {1,  DBTY_MEMO,    11,   (uint32_t)&sEm_MainEnc,                   sizeof(sEm_MainEnc),                                1,              DBRW_R },
    {1,  DBTY_MEMO,    12,   (uint32_t)&sEm_AuxEnc,                    sizeof(sEm_AuxEnc),                                 1,              DBRW_R },
    {1,  DBTY_MEMO,    13,   (uint32_t)&sEm_MainAbsEnc,                sizeof(sEm_MainAbsEnc),                             1,              DBRW_R },
    {1,  DBTY_MEMO,    14,   (uint32_t)&sEm_MainRelEnc,                sizeof(sEm_MainRelEnc),                             1,              DBRW_R },
    {1,  DBTY_MEMO,    15,   (uint32_t)&sEm_PlcSimulationEnc,          sizeof(sEm_PlcSimulationEnc),                       1,              DBRW_RW},
    {0,  DBTY_MEMO,    16,   (uint32_t)&sEm_EncMngrParam,              sizeof(sEm_EncMngrParam),                           1,              DBRW_RW},
    {1,  DBTY_MEMO,    17,   (uint32_t)&sEm_EncMngrOut.flags,          sizeof(sEm_EncMngrOut.flags),                       1,/*-1*/        DBRW_R },
    {0,  DBTY_MEMO,    18,   (uint32_t)&sEfsParam,                     sizeof(sEfsParam),                                  1,              DBRW_RW},
    {1,  DBTY_MEMO,    19,   (uint32_t)&sEm_PlcFbk2CntrLoop,           sizeof(sEm_PlcFbk2CntrLoop),                        1,              DBRW_RW},


        // Motor Handler
    {3,  DBTY_MEMO,    20,   (uint32_t)&sMh_MotorDataOut,              sizeof(sMh_MotorDataOut),                           1,              DBRW_R },
    {0,  DBTY_MEMO,    21,   (uint32_t)&sMh_MotorDataParam,            sizeof(sMh_MotorDataParam),                         1,              DBRW_RW},
    {2,  DBTY_MEMO,    22,   (uint32_t)&sMh_MotorDataUsrInIRef,        sizeof(sMh_MotorDataUsrInIRef),                     1,              DBRW_RW},
    {2,  DBTY_MEMO,    23,   (uint32_t)&sMh_MotorDataPlcInIRef,        sizeof(sMh_MotorDataPlcInIRef),                     1,              DBRW_RW},
    {2,  DBTY_MEMO,    24,   (uint32_t)&uwMh_MotoDataPlcInElecAngle,   sizeof(uwMh_MotoDataPlcInElecAngle),                1,              DBRW_RW},
    // 25: do not use
    {3,  DBTY_MEMO,    26,   (uint32_t)&sMh_MotorDataOut.sPowerStageSts,sizeof(sMh_MotorDataOut.sPowerStageSts),           1,/*-1*/        DBRW_R },
    // 27: do not use
    {3,  DBTY_MEMO,    28,   (uint32_t)&sMh_PlcWorks,                  sizeof(sMh_PlcWorks),                               1,              DBRW_RW},
    {3,  DBTY_MEMO,    29,   (uint32_t)&sMh_MotorDataPlcPStageCtrl,    sizeof(sMh_MotorDataPlcPStageCtrl),                 1,/*-1*/        DBRW_RW},


        // Space Speed Control Loop
    {4,  DBTY_MEMO,    30,   (uint32_t)&sSS_CntrLoopOut,               sizeof(sSS_CntrLoopOut),                            1,              DBRW_R },
    {2,  DBTY_MEMO,    32,   (uint32_t)&sSS_CntrLoopWks,               sizeof(sSS_CntrLoopWks),                            1,              DBRW_R },
    {2,  DBTY_MEMO,    33,   (uint32_t)&sPo_PostnerOut.sDemand,        sizeof(sPo_PostnerOut.sDemand),                     1,              DBRW_R },
    {2,  DBTY_MEMO,    34,   (uint32_t)&sSS_PlcPidRef,                 sizeof(sSS_PlcPidRef),                              1,              DBRW_RW},
    {2,  DBTY_MEMO,    35,   (uint32_t)&sSS_PlcMotCtrlStatus,          sizeof(sSS_PlcMotCtrlStatus),                       1,/*-1*/        DBRW_RW},
    {0,  DBTY_MEMO,    36,   (uint32_t)&sSS_CntrLoopParam,             sizeof(sSS_CntrLoopParam),                          1,              DBRW_RW},
    {2,  DBTY_MEMO,    37,   (uint32_t)&sPo_PostnerOut.sLocDemand,     sizeof(sPo_PostnerOut.sLocDemand),                  1,              DBRW_R },


        // Positioner
    {5,  DBTY_MEMO,    40,   (uint32_t)&sPo_PostnerOut,                sizeof(sPo_PostnerOut),                             1,              DBRW_R },
    {0,  DBTY_MEMO,    41,   (uint32_t)&sPo_PostnerParam,              sizeof(sPo_PostnerParam),                           1,              DBRW_RW},
    {2,  DBTY_MEMO,    42,   (uint32_t)&sPo_UsrPostnerIn,              sizeof(sPo_UsrPostnerIn),                           1,              DBRW_RW},
    {2,  DBTY_MEMO,    43,   (uint32_t)&sPo_PlcPostnerIn,              sizeof(sPo_PlcPostnerIn),                           1,              DBRW_RW},
    {5,  DBTY_MEMO,    44,   (uint32_t)&sPo_PostnerOut.flags,          sizeof(sPo_PostnerOut.flags),                       1,/*-1*/        DBRW_R },
    {2,  DBTY_MEMO,    45,   (uint32_t)&sPo_PlcPostnerIn.flags,        sizeof(sPo_PlcPostnerIn.flags),                     1,/*-1*/        DBRW_RW},
    {2,  DBTY_MEMO,    46,   (uint32_t)&sPo_PlcWorks.flags,            sizeof(sPo_PlcWorks.flags),                         1,/*-1*/        DBRW_RW},

        // Motion Controller
    {6,  DBTY_MEMO,    50,   (uint32_t)&sMotCtrl_Out,                  sizeof(sMotCtrl_Out),                               1,              DBRW_R },
    {0,  DBTY_MEMO,    51,   (uint32_t)&sMotCtrlParameters,            sizeof(sMotCtrlParameters),                         1,              DBRW_RW},
    {2,  DBTY_MEMO,    52,   (uint32_t)&sMotCtrl_PlcControl,           sizeof(sMotCtrl_PlcControl),                        1,              DBRW_RW},
    {2,  DBTY_MEMO,    53,   (uint32_t)&sMotCtrl_UsrControl,           sizeof(sMotCtrl_UsrControl),                        1,              DBRW_RW},
    {6,  DBTY_MEMO,    54,   (uint32_t)&sMotCtrl_OStatus,              sizeof(sMotCtrl_OStatus),                           1,/*-1*/        DBRW_R },
    {3,  DBTY_MEMO,    55,   (uint32_t)&sMotCtrl_Out.sPowerStageCtrl,  sizeof(sMotCtrl_Out.sPowerStageCtrl),               1,/*-1*/        DBRW_R },


        // PI DCbus Controller
    {8,  DBTY_MEMO,    60,   (uint32_t)&sDcBusCntrl_Out,               sizeof(sDcBusCntrl_Out),                            1,              DBRW_R },
    {2,  DBTY_MEMO,    61,   (uint32_t)&sDcBusCntrl_Plc,               sizeof(sDcBusCntrl_Plc),                            1,              DBRW_RW},
    {0,  DBTY_MEMO,    62,   (uint32_t)&sDcBusCntrl_Param,             sizeof(sDcBusCntrl_Param),                          1,              DBRW_RW},


        // Thermal Model
    {9,  DBTY_MEMO,    70,   (uint32_t)&sTm_ThModOut,                  sizeof(sTm_ThModOut),                               1,              DBRW_R },
    {2,  DBTY_MEMO,    71,   (uint32_t)&flTm_PlcMotorTemp,             sizeof(flTm_PlcMotorTemp),                          1,              DBRW_RW},
    {0,  DBTY_MEMO,    72,   (uint32_t)&sTm_ThModParam,                sizeof(sTm_ThModParam),                             1,              DBRW_RW},


    // 80 to 82 and 90 to 92: do not use

        // Digital I/O
    {7,  DBTY_INP,    130,   (uint32_t)&sPlcDIO.bInputs[0],            sizeof(sPlcDIO.bInputs),                            1,/*-1*/        DBRW_R },
    {7,  DBTY_OUT,    131,   (uint32_t)&sPlcDIO.bOutputs[0],           sizeof(sPlcDIO.bOutputs),                           1,/*-1*/        DBRW_RW},

        // Analog I/O
    {7,  DBTY_INP,    132,   (uint32_t)&sUsrIOCPData.sAIO.swInputs[0], ARRSIZE(sUsrIOCPData.sAIO.swInputs),                sizeof(sUsrIOCPData.sAIO.swInputs[0]),  DBRW_R },
    {7,  DBTY_OUT,    133,   (uint32_t)&sUsrIOCPData.sAIO.swOutputs[0],ARRSIZE(sUsrIOCPData.sAIO.swOutputs),               sizeof(sUsrIOCPData.sAIO.swOutputs[0]), DBRW_RW},
#ifndef _HW_DC
    {0,  DBTY_MEMO,   134,   (uint32_t)&sUsrIOFlags.b[0],              sizeof(sUsrIOFlags),                                1,/*-1*/       DBRW_RW},
    {0,  DBTY_MEMO,   135,   (uint32_t)&sUsrIOHwDetect.b[0],           sizeof(sUsrIOHwDetect),                             1,/*-1*/       DBRW_R },
#else
    {0,  DBTY_MEMO,   136,   (uint32_t)&sUsrIOFlags.b[0],              sizeof(sUsrIOFlags),                                1,/*-1*/       DBRW_RW},
    {0,  DBTY_MEMO,   137,   (uint32_t)&sUsrIOHwDetect.b[0],           sizeof(sUsrIOHwDetect),                             1,/*-1*/       DBRW_R },
#endif

#ifdef _HW_CT
    {7,  DBTY_INP,    138,   (uint32_t)&sUsrIOCPBase.sDIO.ubOutFeedbacks,1,                                                1,/*-1*/       DBRW_R},
#endif  

        // encoder specific data out
    {1,  DBTY_MEMO,   150,   (uint32_t)&sIc_IncEncDataOut[INCREMENTAL_SEL_MAIN], sizeof(INCREMENTAL_OUT),                  1,              DBRW_R },
    {1,  DBTY_MEMO,   151,   (uint32_t)&sIc_IncEncDataOut[INCREMENTAL_SEL_AUX], sizeof(INCREMENTAL_OUT),                   1,              DBRW_R },
    {1,  DBTY_MEMO,   152,   (uint32_t)&sSc_DataOut,                   sizeof(sSc_DataOut),                                1,              DBRW_R },
    {1,  DBTY_MEMO,   153,   (uint32_t)&sEfsSSInIdrAndPLoop,           sizeof(sEfsSSInIdrAndPLoop),                        1,              DBRW_R },
    {1,  DBTY_MEMO,   154,   (uint32_t)&sEn_DataOut[ENDAT_SEL_MAIN],   sizeof(ENDAT_OUT),                                  1,              DBRW_R },
#ifndef _HW_DC
    {1,  DBTY_MEMO,   155,   (uint32_t)&sEn_DataOut[ENDAT_SEL_AUX],    sizeof(ENDAT_OUT),                                  1,              DBRW_R },
#endif
    // 156: do not use
    {1,  DBTY_MEMO,   157,   (uint32_t)&sEm_Fbk2CLExt,                 sizeof(sEm_Fbk2CLExt),                              1,              DBRW_R },
    {1,  DBTY_MEMO,   158,   (uint32_t)&sSe_AuxSimOut,                 sizeof(sSe_AuxSimOut),                              1,              DBRW_R },
    {1,  DBTY_MEMO,   159,   (uint32_t)&sEfsOut,                       sizeof(sEfsOut),                                    1,              DBRW_R },

        // IO measurements
#ifndef _HW_DC
    {7,  DBTY_MEMO,   160,   (uint32_t)&sIOMgrAnalogMeasurements,      sizeof(sIOMgrAnalogMeasurements),                   1,              DBRW_R },
#else
    {7,  DBTY_MEMO,   161,   (uint32_t)&sIOMgrAnalogMeasurements,      sizeof(sIOMgrAnalogMeasurements),                   1,              DBRW_R },
#endif

        // Current Channels Min/Max Measurements
    {3,  DBTY_MEMO,   170,   FPGA_REGISTER_BASE_ADDRESS+FPGA_CURCHMINMAX_BASEADDR,   FPGA_CURCHMINMAX_SIZE,                sizeof(UBYTE),  DBRW_R },

        // System
    {0,  DBTY_MEMO,   200,   (uint32_t)&sModBusSMSystemFlagBits,       sizeof(sModBusSMSystemFlagBits),                    1,/*-1*/        DBRW_RW},
    {0,  DBTY_MEMO,   201,   FPGA_REGISTER_BASE_ADDRESS+FPGA_CUSTOMAPP_BASEADDR,     FPGA_CUSTOMAPP_SIZE,                  sizeof(UWORD),  DBRW_RW},
    {0,  DBTY_MEMO,   202,   (uint32_t)&ulSystemAlarms,                1,                                                  sizeof(ULONG),  DBRW_R },
    {0,  DBTY_MEMO,   203,   (uint32_t)&ulSystemAlarmSubCode,          1,                                                  sizeof(ULONG),  DBRW_R },
    {0,  DBTY_MEMO,   204,   (uint32_t)&sSysAppInfo,                   sizeof(sSysAppInfo),                                1,              DBRW_R },
    {0,  DBTY_MEMO,   205,   (uint32_t)&sGlbMotorParameters,           sizeof(sGlbMotorParameters),                        1,              DBRW_RW},
    {10, DBTY_MEMO,   206,   (uint32_t)&ulSysTimersTotalPowerOnTime,   1,                                                  sizeof(ulSysTimersTotalPowerOnTime),  DBRW_R },
    {0,  DBTY_MEMO,   207,   (uint32_t)&uwSysTimers1ms,                1,                                                  sizeof(uwSysTimers1ms),               DBRW_R },
    {0,  DBTY_MEMO,   208,   (uint32_t)&sSystemStatus,                 sizeof(sSystemStatus),                              1,/*-1*/        DBRW_R },
    {0,  DBTY_MEMO,   209,   FPGA_REGISTER_BASE_ADDRESS,               FPGA_STANDARD_SIZE,                                 sizeof(UWORD),  DBRW_R },
    {0,  DBTY_MEMO,   210,   (uint32_t)&ulSystemWarnings,              1,                                                  sizeof(ULONG),  DBRW_R },
    {0,  DBTY_MEMO,   211,   (uint32_t)&uwSystemBootErrorCode,         1,                                                  sizeof(UWORD),  DBRW_R },
    {0,  DBTY_MEMO,   212,   (uint32_t)&uwParChkParamCode,             1,                                                  sizeof(UWORD),  DBRW_R },
    {0,  DBTY_MEMO,   213,   (uint32_t)&uwTaskSchedRTLocalMaxTime,     1,                                                  sizeof(UWORD),  DBRW_R },
    {0,  DBTY_MEMO,   214,   (uint32_t)&uwTaskSchedRTLocalAvgTime,     1,                                                  sizeof(UWORD),  DBRW_R },
    {0,  DBTY_MEMO,   215,   (uint32_t)&uwSystemDisableWDT,            1,                                                  sizeof(UWORD),  DBRW_RW},
    {0,  DBTY_MEMO,   216,   (uint32_t)&sGlbControlBoardParameters,    sizeof(sGlbControlBoardParameters),                 1,              DBRW_R },
    // 217: do not use
    {0,  DBTY_MEMO,   218,   (uint32_t)&sSyncMgrParam,                 1,                                                  sizeof(sSyncMgrParam),  DBRW_RW},
    {0,  DBTY_MEMO,   219,   (uint32_t)&sSyncMgrDiagnosticOut,         sizeof(sSyncMgrDiagnosticOut),                      1,              DBRW_R },
    {0,  DBTY_MEMO,   220,   (uint32_t)&sMh_PlcUMRatios,               sizeof(sMh_PlcUMRatios),                            1,              DBRW_R },
    // 221: do not use
    {0,  DBTY_MEMO,   222,   (uint32_t)&sDrvTskCtrlParams.sNew,        sizeof(sDrvTskCtrlParams.sNew),                     1,              DBRW_RW},
    {0,  DBTY_MEMO,   224,   (uint32_t)&sGlbPowerBoardParameters,      sizeof(sGlbPowerBoardParameters),                   1,              DBRW_R },
    {0,  DBTY_MEMO,   225,   (uint32_t)&ubDrvTskCtrlMainOpSelected,    1,                                                  sizeof(UBYTE),  DBRW_R },
    // 226: do not use
    {0,  DBTY_MEMO,   227,   (uint32_t)&sEm_LastValidEPlate,           sizeof(sEm_LastValidEPlate),                        1,              DBRW_RW},
    {0,  DBTY_MEMO,   228,   (uint32_t)&sGlbAssemblyInfo.ulSerialNumber,sizeof(sGlbAssemblyInfo.ulSerialNumber),           1,              DBRW_R },
    {0,  DBTY_MEMO,   229,   (uint32_t)&uwSysTimers125us,              1,                                                  sizeof(uwSysTimers125us),             DBRW_R },
    // 230: do not use
    // 231: do not use
    {0,  DBTY_MEMO,   232,   (uint32_t)&sHwConfParams,                 1,                                                  sizeof(sHwConfParams),DBRW_RW},
    {0,  DBTY_MEMO,   233,   (uint32_t)&uwSystemMainClockFreq,         1,                                                  sizeof(uwSystemMainClockFreq),DBRW_R },
    {0,  DBTY_MEMO,   234,   (uint32_t)&sGlbAssemblyInfo,              sizeof(sGlbAssemblyInfo),                           1,              DBRW_R },
#ifdef CFG_DS301_IDENTITY
    {0,  DBTY_MEMO,   235,   (uint32_t)&tDs301Identity,                sizeof(tDs301Identity),                             1,              DBRW_RW},
#endif
    {0,  DBTY_MEMO,   236,   (uint32_t)&sSystemStatus2,                sizeof(sSystemStatus2),                             1,/*-1*/        DBRW_R },

#ifdef _APP_PLCDBGWORKS
    {0,  DBTY_MEMO,   250,   (uint32_t)&uwPlcDebugCommon,              sizeof(uwPlcDebugCommon),                           sizeof(UBYTE),  DBRW_RW},
#endif
#if (OS_MEASURESTACKSIZE)
    {0,  DBTY_MEMO,   260,   (uint32_t)&uwOsSysStackFree,              1,                                                  sizeof(UWORD),  DBRW_R },
    {0,  DBTY_MEMO,   261,   (uint32_t)&uwOsUsrStackFree,              1,                                                  sizeof(UWORD),  DBRW_R },
    {0,  DBTY_MEMO,   262,   (uint32_t)&uwOsMinStackFreePerTask,       sizeof(uwOsMinStackFreePerTask)/sizeof(UWORD),      sizeof(UWORD),  DBRW_R },
#endif
        // CanOpen
#if CFG_CANDRV_CMDMGR
    {0,  DBTY_MEMO,   300,   (uint32_t)&sCanOpenCMOut,                 sizeof(sCanOpenCMOut),                              1,              DBRW_RW},
    {0,  DBTY_MEMO,   301,   (uint32_t)&tCanOpenCMParams,              sizeof(tCanOpenCMParams),                           1,              DBRW_RW},
    {0,  DBTY_MEMO,   302,   (uint32_t)&uwCanDrvGlobalFlags[0],        sizeof(uwCanDrvGlobalFlags),                        1,              DBRW_R },
#endif
    {0,  DBTY_MEMO,   303,   (uint32_t)&tDs301Param,                   sizeof(tDs301Param),                                1,              DBRW_RW},
#if CFG_CANDRV_DS301
    {0,  DBTY_MEMO,   304,   (uint32_t)&uwCanOpenPSMRxPdoCnt,          sizeof(uwCanOpenPSMRxPdoCnt)/sizeof(UWORD),         sizeof(UWORD),  DBRW_R },
#endif
        // Modbus
#ifndef _HW_DC
    {0,  DBTY_MEMO,   350,   (uint32_t)&sModBusCMParams.flags,         sizeof(sModBusCMParams.flags),                      1,/*-1*/        DBRW_RW},
    {0,  DBTY_MEMO,   351,   (uint32_t)&sModBusCMParOverserial0,       sizeof(sModBusCMParOverserial0),                    1,              DBRW_RW},
#endif

        // Encoders Extra
    {0,  DBTY_MEMO,   400,   (uint32_t)&sBe_EmfEncDiagOut,             sizeof(sBe_EmfEncDiagOut),                          1,              DBRW_R },
    {0,  DBTY_MEMO,   401,   (uint32_t)&sEn_Params[ENDAT_SEL_MAIN],    sizeof(sEn_Params[ENDAT_SEL_MAIN]),                 1,              DBRW_RW},
#ifndef _HW_DC
    {0,  DBTY_MEMO,   402,   (uint32_t)&sEn_Params[ENDAT_SEL_AUX],     sizeof(sEn_Params[ENDAT_SEL_AUX]),                  1,              DBRW_RW},
#endif
    {0,  DBTY_MEMO,   403,   (uint32_t)&sSc_SinCosParam,               sizeof(sSc_SinCosParam),                            1,              DBRW_RW},
    {0,  DBTY_MEMO,   404,   (uint32_t)&sHl_HallParam,                 sizeof(sHl_HallParam),                              1,              DBRW_RW},
    {0,  DBTY_MEMO,   405,   (uint32_t)&sIc_IncEncParams[INCREMENTAL_SEL_MAIN], sizeof(sIc_IncEncParams[INCREMENTAL_SEL_MAIN]), 1,         DBRW_RW},
    {0,  DBTY_MEMO,   406,   (uint32_t)&sIc_IncEncParams[INCREMENTAL_SEL_AUX],  sizeof(sIc_IncEncParams[INCREMENTAL_SEL_AUX]),  1,         DBRW_RW},
    {0,  DBTY_MEMO,   407,   (uint32_t)&sBe_EmfEncParam,               sizeof(sBe_EmfEncParam),                            1,              DBRW_RW},
    {0,  DBTY_MEMO,   408,   (uint32_t)&sSe_AuxSimParam,               sizeof(sSe_AuxSimParam),                            1,              DBRW_RW},
    // 409: do not use
    {1,  DBTY_MEMO,   410,   (uint32_t)&sEm_PlcFbk2CLExt,              sizeof(sEm_PlcFbk2CLExt),                           1,              DBRW_RW},
    {0,  DBTY_MEMO,   411,   (uint32_t)&sHf_Params,                    sizeof(sHf_Params),                                 1,              DBRW_RW},
    {0,  DBTY_MEMO,   412,   (uint32_t)&sNk_Params[NIKON_SEL_MAIN],    sizeof(sNk_Params[NIKON_SEL_MAIN]),                 1,              DBRW_RW},
    {0,  DBTY_MEMO,   413,   (uint32_t)&sNk_DataOut[NIKON_SEL_MAIN],   sizeof(sNk_DataOut[NIKON_SEL_MAIN]),                1,              DBRW_R },
    {0,  DBTY_MEMO,   414,   (uint32_t)&sTm_Params[TMGW_SEL_MAIN],     sizeof(sTm_Params[TMGW_SEL_MAIN]),                  1,              DBRW_RW},
	{0,  DBTY_MEMO,   415,   (uint32_t)&sTm_DataOut[TMGW_SEL_MAIN],    sizeof(sTm_DataOut[TMGW_SEL_MAIN]),                 1,              DBRW_R },

        // Motorhandler Extra
    {2,  DBTY_MEMO,   420,   (uint32_t)&sMh_MotorDataUsrSetIdLimit,    sizeof(sMh_MotorDataUsrSetIdLimit),                 1,              DBRW_RW},
    {2,  DBTY_MEMO,   421,   (uint32_t)&sMh_MotorDataUsrSetIqLimit,    sizeof(sMh_MotorDataUsrSetIqLimit),                 1,              DBRW_RW},
    {0,  DBTY_MEMO,   422,   (uint32_t)&sDflx_Param,                   sizeof(sDflx_Param),                                1,              DBRW_RW},
    {2,  DBTY_MEMO,   423,   (uint32_t)&sDflx_Out,                     sizeof(sDflx_Out),                                  1,              DBRW_R },
    {0,  DBTY_MEMO,   424,   (uint32_t)&sDflx_Out.flags,               sizeof(sDflx_Out.flags),                            1,/*-1*/        DBRW_R },
    {0,  DBTY_MEMO,   425,   (uint32_t)&sMh_MotorDataParam.flags,      sizeof(sMh_MotorDataParam.flags),                   1,/*-1*/        DBRW_RW},
    {2,  DBTY_MEMO,   426,   (uint32_t)&slMh_MotoDataPlcInFilteredSpeed,sizeof(slMh_MotoDataPlcInFilteredSpeed),           1,              DBRW_RW},
    {3,  DBTY_MEMO,   427,   (uint32_t)&sMh_PlcAdvancedWorks,          sizeof(sMh_PlcAdvancedWorks),                       1,/*-1*/        DBRW_RW},
    {2,  DBTY_MEMO,   428,   (uint32_t)&swMh_MotoDataPlcInElecSpeed,   sizeof(swMh_MotoDataPlcInElecSpeed),                1,              DBRW_RW},
#ifndef _HW_DC
    {0,  DBTY_MEMO,   429,   (uint32_t)&sMh_CurrScaleAdj[0],           sizeof(sMh_CurrScaleAdj),                           1,              DBRW_RW},
#endif

    {0,  DBTY_MEMO,   430,   FPGA_CPUH_URAM_BASE,                         RGAD_CPUH_RGURAM_SIZE/2,                         1,              DBRW_RW},
    {0,  DBTY_MEMO,   431,   FPGA_CPUH_URAM_BASE+RGAD_CPUH_RGURAM_SIZE/2, RGAD_CPUH_RGURAM_SIZE/2,                         1,              DBRW_RW},
    {0,  DBTY_MEMO,   432,   FPGA_CPUH_DRAM_BASE,                         RGAD_CPUH_DATRAM_SIZE,                           1,              DBRW_RW},
    {0,  DBTY_MEMO,   MH_DSPDB_IEC_WS,(uint32_t)&ubPlcDSPDBWorks[0],      PLC_DSPDB_DIM_COM,                               1,              DBRW_R },

    // 500: do not use

        // imaged parameters
    {10, DBTY_MEMO,   616,   (uint32_t)&sEm_EncMngrParam,              sizeof(sEm_EncMngrParam),                           1,              DBRW_RW},
    {10, DBTY_MEMO,   618,   (uint32_t)&sEfsParam,                     sizeof(sEfsParam),                                  1,              DBRW_RW},
    {10, DBTY_MEMO,   621,   (uint32_t)&sMh_MotorDataParam,            sizeof(sMh_MotorDataParam),                         1,              DBRW_RW},
    {10, DBTY_MEMO,   636,   (uint32_t)&sSS_CntrLoopParam,             sizeof(sSS_CntrLoopParam),                          1,              DBRW_RW},
    {10, DBTY_MEMO,   641,   (uint32_t)&sPo_PostnerParam,              sizeof(sPo_PostnerParam),                           1,              DBRW_RW},
    {10, DBTY_MEMO,   651,   (uint32_t)&sMotCtrlParameters,            sizeof(sMotCtrlParameters),                         1,              DBRW_RW},
    {10, DBTY_MEMO,   662,   (uint32_t)&sDcBusCntrl_Param,             sizeof(sDcBusCntrl_Param),                          1,              DBRW_RW},
    {10, DBTY_MEMO,   672,   (uint32_t)&sTm_ThModParam,                sizeof(sTm_ThModParam),                             1,              DBRW_RW},
    // 682: do not use
    {10, DBTY_MEMO,   690,   (uint32_t)&sGlbMotorParameters,           sizeof(sGlbMotorParameters),                        1,              DBRW_RW},

        // EtherCAT                                                                                    
#if CFG_ECAT
    {0,  DBTY_MEMO,   800,   (uint32_t)&tEcatCMParam,                  sizeof(tEcatCMParam),                               1,              DBRW_RW},
    {0,  DBTY_MEMO,   801,   (uint32_t)&tEcatCMSyncManOutDiag,         sizeof(tEcatCMSyncManOutDiag),                      1,              DBRW_R },
    {0,  DBTY_MEMO,   802,   (uint32_t)&tEcatCMSyncManInDiag,          sizeof(tEcatCMSyncManInDiag),                       1,              DBRW_R },
    {0,  DBTY_MEMO,   803,   (uint32_t)&tEcatCMCycleDiag,              sizeof(tEcatCMCycleDiag),                           1,              DBRW_R },
    {0,  DBTY_MEMO,   804,   (uint32_t)&tEcatCMSMOutParam,             sizeof(tEcatCMSMOutParam),                          1,              DBRW_R },
    {0,  DBTY_MEMO,   805,   (uint32_t)&tEcatCMSMInParam,              sizeof(tEcatCMSMInParam),                           1,              DBRW_R },
    {0,  DBTY_MEMO,   806,   (uint32_t)&sEcatDiag,                     sizeof(sEcatDiag),                                  1,              DBRW_R },
    {0,  DBTY_MEMO,   807,   (uint32_t)&sEcatDiag.sFlags,              sizeof(sEcatDiag.sFlags),                           1,/*-1*/        DBRW_R },
    {0,  DBTY_MEMO,   808,   (uint32_t)&uwEcatEmcyErrCode,             sizeof(uwEcatEmcyErrCode),                          1,              DBRW_R },
    {0,  DBTY_MEMO,   810,   FPGA_ETHERNET_BASE_ADDRESS,               FPGA_ETHERNET_SIZE,                                 1,              DBRW_R },
#endif // cfg_ecat

        // EtherPMC
#if CFG_ETHPMC
    {0,  DBTY_MEMO,   850,   (uint32_t)&sEpmcCM_Params,                sizeof(sEpmcCM_Params),                             1,              DBRW_RW},
    {0,  DBTY_MEMO,   851,   (uint32_t)&sEpmcCMInOut,                  sizeof(sEpmcCMInOut),                               1,              DBRW_RW},
    {0,  DBTY_MEMO,   852,   FPGA_EPMCREGS_SLOT_BASE,                  (EPMCHw_SLOT_MAXSIZE<<1),                           1,              DBRW_RW},
    {0,  DBTY_MEMO,   853,   (uint32_t)&sEpmcCMDiag,                   sizeof(sEpmcCMDiag),                                1,              DBRW_R },
    {0,  DBTY_MEMO,   854,   (uint32_t)&sEpmcCMDiag.sFlags,            sizeof(sEpmcCMDiag.sFlags),                         1,/*-1*/        DBRW_R },
#endif // cfg_eth_pmc

#if CFG_PSU_MNGR
    {0,  DBTY_MEMO,   870,   (uint32_t)&sPSUWrk.bEnableModule,          sizeof(sPSUWrk.bEnableModule),                     1,/*-1,*/       DBRW_RW},
    {0,  DBTY_MEMO,   871,   (uint32_t)&sPSUInfo,                       sizeof(sPSUInfo),                                  1,              DBRW_R },
    {0,  DBTY_MEMO,   872,   (uint32_t)&sPSUCommData,                   sizeof(sPSUCommData),                              1,              DBRW_R },
	{0,  DBTY_MEMO,   874,   (uint32_t)&sPSUInParam.flags,              sizeof(sPSUInParam.flags),                         1,/*-1,*/       DBRW_RW},
#endif // cfg_psu_mngr

        // PLC Config
    {0,  DBTY_MEMO,  1000,   (uint32_t)&sPlcUserCfgFlags,              sizeof(sPlcUserCfgFlags),                           1,/*-1*/        DBRW_RW },
    {0,  DBTY_MEMO,  1001,   (uint32_t)&uwPlcSlowTaskPeriod,           1,                            sizeof(uwPlcSlowTaskPeriod),          DBRW_RW },
    {0,  DBTY_MEMO,  1002,   (uint32_t)&uwPlcSlowTaskTime,             1,                            sizeof(uwPlcSlowTaskTime),            DBRW_R  },
    {0,  DBTY_MEMO,  1003,   (uint32_t)&sDrvTskCtrlPlcConfig,          1,                            sizeof(sDrvTskCtrlPlcConfig),         DBRW_RW },
    // 1004: do not use
    // 1005: do not use

        // User Parameters/works
    {0,  DBTY_MEMO,  1010,   (uint32_t)&uwPlcWorksCommon[0],           sizeof(uwPlcWorksCommon)/sizeof(UWORD),             sizeof(UWORD),      DBRW_RW},
    {0,  DBTY_MEMO,  1011,   (uint32_t)&uwPlcWorksBools[0],            sizeof(uwPlcWorksBools),                            1,/*-1*/            DBRW_RW},
    {14, DBTY_MEMO,  1020,   (uint32_t)&uwPlcWorksCommon[0],           sizeof(uwPlcWorksCommon)/sizeof(UWORD),             sizeof(UWORD),      DBRW_RW},
    {14, DBTY_MEMO,  1021,   (uint32_t)&uwPlcWorksBools[0],            sizeof(uwPlcWorksBools),                            1,/*-1*/            DBRW_RW},
    {0,  DBTY_MEMO,  PLCCOMDBMGR_IEC_PAR_WS, (uint32_t)&tPlcParamCommon.uwCommon[0], sizeof(tPlcParamCommon.uwCommon)/sizeof(UWORD), sizeof(UWORD),        DBRW_RW},
    {0,  DBTY_MEMO,  PLCCOMDBMGR_IEC_PAR_BS, (uint32_t)&tPlcParamBits.uwBools[0],    sizeof(tPlcParamBits.uwBools),                  1,/*-1*/  DBRW_RW},
    {15, DBTY_MEMO,  PLCCOMDBMGR_IEC_PAR_WI, (uint32_t)&tPlcParamCommon.uwCommon[0], sizeof(tPlcParamCommon.uwCommon)/sizeof(UWORD), sizeof(UWORD),        DBRW_RW},
    {15, DBTY_MEMO,  PLCCOMDBMGR_IEC_PAR_BI, (uint32_t)&tPlcParamBits.uwBools[0],    sizeof(tPlcParamBits.uwBools),                  1,/*-1*/  DBRW_RW},

        // Memory block in softscope memory area
    {0,  DBTY_MEMO,  1100,   (uint32_t)&ulScoMemory[0],                sizeof(ulScoMemory)/sizeof(UBYTE),                  sizeof(UBYTE),  DBRW_RW},
};

#ifdef _APP_DEBUG
const UWORD plcCntDataBlocks=sizeof(plcDataBlocks)/sizeof(PLCIEC_DBREC);
#endif

//****************************************************************************
// PLC embedded functions

#ifdef INFINEON
void PLCSUBFC_CASTF(void);
void PLCSUBFC_FCAST(void);
void PLCSUBFC_DCASTF(void);
void PLCSUBFC_FCASTD(void);
void PLCSUBFC_FPDIV(void);
void PLCSUBFC_FPMUL(void);
void PLCSUBFC_FPADD(void);
void PLCSUBFC_FPSUB(void);
void PLCSUBFC_FPCMP(void);
void PLCSUBFC_ULDIV(void);
void PLCSUBFC_SLDIV(void);
#endif

PLC_ATTR_PLCIEC_FCNREC plcEmbeddedFunctions[] =
{
#ifdef INFINEON
    {"flt2int",                     (uint32_t)&PLCSUBFC_CASTF},
    {"int2flt",                     (uint32_t)&PLCSUBFC_FCAST},
    {"flt2dbl",                     (uint32_t)&PLCSUBFC_DCASTF},
    {"dbl2flt",                     (uint32_t)&PLCSUBFC_FCASTD},
    {"fltdiv",                      (uint32_t)&PLCSUBFC_FPDIV},
    {"fltmul",                      (uint32_t)&PLCSUBFC_FPMUL},
    {"fltadd",                      (uint32_t)&PLCSUBFC_FPADD},
    {"fltsub",                      (uint32_t)&PLCSUBFC_FPSUB},
    {"fltcmp",                      (uint32_t)&PLCSUBFC_FPCMP},
    {"uldiv",                       (uint32_t)&PLCSUBFC_ULDIV},
    {"sldiv",                       (uint32_t)&PLCSUBFC_SLDIV},
#else
//	Embedded functions required by ARM implementation (+18 func)
    ALPLC_BASICFUN_FCNREC
//	Other embedded function math and real required after porting to new LogicLab (+16 func)
    {"_ui_to_float",                (uint32_t)&AlPlcReal_UdintToReal},
    {"_i_to_float",                 (uint32_t)&AlPlcReal_DintToReal},
    {"_float_to_uint",              (uint32_t)&AlPlcReal_RealToUdint},
    {"_float_to_int",               (uint32_t)&AlPlcReal_RealToDint},
    {"_fadd",                       (uint32_t)&AlPlcReal_Add},
    {"_fsub",                       (uint32_t)&AlPlcReal_Sub},
    {"_fmul",                       (uint32_t)&AlPlcReal_Mul},
    {"_do_float_div",               (uint32_t)&AlPlcReal_Div},
    {"_do_float_gt",                (uint32_t)&AlPlcReal_Gt},
    {"_do_float_ge",                (uint32_t)&AlPlcReal_Ge},
    {"_do_float_eq",                (uint32_t)&AlPlcReal_Eq},
    {"_do_float_lt",                (uint32_t)&AlPlcReal_Lt},
    {"_do_float_le",                (uint32_t)&AlPlcReal_Le},
    {"_do_float_ne",                (uint32_t)&AlPlcReal_Ne},
    {"_do_pow",                     (uint32_t)&AlPlcMath_Pow},
    {"_do_atan2",                   (uint32_t)&AlPlcMath_ATan2},
#endif // ! _infineon

    {"Sin16",                       (uint32_t)&Sin16},
    {"Cos16",                       (uint32_t)&Cos16},
    {"ATan16",                      (uint32_t)&ATan16},
    {"_do_sin",                     (uint32_t)&AlPlcMath_Sin},
    {"_do_cos",                     (uint32_t)&AlPlcMath_Cos},
    {"_do_tan",                     (uint32_t)&AlPlcMath_Tan},
    {"_do_asin",                    (uint32_t)&AlPlcMath_ASin},
    {"_do_acos",                    (uint32_t)&AlPlcMath_ACos},
    {"_do_atan",                    (uint32_t)&AlPlcMath_ATan},
    {"_do_sinh",                    (uint32_t)&AlPlcMath_Sinh},
    {"_do_cosh",                    (uint32_t)&AlPlcMath_Cosh},
    {"_do_tanh",                    (uint32_t)&AlPlcMath_Tanh},
    {"_do_exp",                     (uint32_t)&AlPlcMath_Exp},
    {"_do_ln",                      (uint32_t)&AlPlcMath_Ln},
    {"_do_log",                     (uint32_t)&AlPlcMath_Log},
    {"_do_sqrt",                    (uint32_t)&AlPlcMath_Sqrt},
    {"_do_ceil",                    (uint32_t)&AlPlcMath_Ceil},
    {"_do_floor",                   (uint32_t)&AlPlcMath_Floor},
    {"_do_float_abs",               (uint32_t)&AlPlcMath_Fabs},
    {"sysPostAlarm",                (uint32_t)&SysLogMgm_PostAlarm},
    {"sysPostUserAlarm",            (uint32_t)&PlcPostUserAlarm},
    {"sysExecEmergencyStop",        (uint32_t)&PlcPostEmcyStop},
    {"sysClearAlarms",              (uint32_t)&SysLogMgm_ClearAlarms},
    {"sysPStageCurrentCalibration", (uint32_t)&Mh_ManageCurrentCalibration},
    {"sysPStageAdjustCurrOffset",   (uint32_t)&Mh_ManualCurrentOffset},

#if CFG_ENC_ENDAT
    {"sysEndatSetCommandMode",      (uint32_t)&PlcEndatSetCommandMode},
    {"sysEndatReadParameter",       (uint32_t)&PlcEndatReadParameter},
    {"sysEndatWriteParameter",      (uint32_t)&PlcEndatWriteParameter},
#endif // cfg_enc_endat

#if CFG_CANDRV_DS301
    {"sysCANOpenNMTSetState",       (uint32_t)&ds301_set_nmt_state},
#endif // cfg_candrv_ds301

    {"mDINTScale",                  (uint32_t)&_sint32_scale_32},
    {"mQINTScale",                  (uint32_t)&_sint64p_scale_32},
    {"mQINTAdd",                    (uint32_t)&_sint64p_add},
    {"mQINTSub",                    (uint32_t)&_sint64p_sub},
    {"mINTScaleOffset",             (uint32_t)&_sint16_offsetscale_16},
    {"sysPStageOptionInterface",    (uint32_t)&Mh_ForceCommand},
    {"sysPStageOptionInterfaceEx",  (uint32_t)&Mh_ForceCommandEx},
    {"sysILoopFilterConstsCalc",    (uint32_t)&Mh_PlcCalcFilterConstants},
    {"sysILoopFilterIqSet",         (uint32_t)&Mh_SetIqFilter},
    {"sysSetRTTaskPeriod",          (uint32_t)&SyncMgrAdjustRTPeriod},

#if CFG_CANDRV_CMDMGR
    {"sysCanSetSpeed",              (uint32_t)&candrv_plc_setspeed},
    {"sysCanRun",                   (uint32_t)&candrv_plc_run},
    {"sysCanStop",                  (uint32_t)&candrv_plc_stop},
    {"sysCanIsRunning",             (uint32_t)&candrv_isrunning},
#ifndef _CANDRV_EXTD
    {"sysCanAddRxCob",              (uint32_t)&candrv_plc_addrxcob},
    {"sysCanDeleteRxCob",           (uint32_t)&candrv_plc_deleterxcob},
    {"sysCanRequestRxCob",          (uint32_t)&candrv_plc_requestrxcob},
    {"sysCanSendCob",               (uint32_t)&candrv_plc_sendcob},
    {"sysCanSetPeriodicCob",        (uint32_t)&candrv_plc_setperiodiccob},
#else
    {"sysCanExtAddRxCob",           (uint32_t)&candrv_plc_addrxcob},
    {"sysCanExtDeleteRxCob",        (uint32_t)&candrv_plc_deleterxcob},
    {"sysCanExtRequestRxCob",       (uint32_t)&candrv_plc_requestrxcob},
    {"sysCanExtSendCob",            (uint32_t)&candrv_plc_sendcob},
    {"sysCanExtSetPeriodicCob",     (uint32_t)&candrv_plc_setperiodiccob},
#endif // _candrv_extd
#endif // cfg_candrv_cmdmgr

    {"sysPlcSetAppParDatabase",     (uint32_t)&PlcComDBAppSet},
    {"sysEncMgrOptionInterface",    (uint32_t)&Em_SetPlcOption},
    {"sysTimeProfilerBegin",        (uint32_t)&PlcProfilerBegin},
    {"sysTimeProfilerEnd",          (uint32_t)&PlcProfilerEnd},
    {"sysMemBlockReserve",          (uint32_t)&SoftScopeRequestMemory},
    {"sysMemBlockSave",             (uint32_t)&SoftScopeReqMemSave},
    {"sysMemBlockRestore",          (uint32_t)&SoftScopeReqMemLoad},
    {"sysAlarmsSetDisMask",         (uint32_t)&SysLogMgm_SetDisableAlarmMask},
    {"sysThModelOptionInterface",   (uint32_t)&Tm_SetPlcOption},
    {"sysDevCtrlOptionInterface",   (uint32_t)&MotCtrl_SetPlcOption},
#if CFG_ENC_HIP
    {"sysHipfcSetPosition",         (uint32_t)&PlcHipfcSetPosition},
#endif

#if CFG_ENC_ENDAT
    {"sysEndatResetEncoder",        (uint32_t)&PlcEndatResetEncoder},
    {"sysEndatResetActiveAlarms",   (uint32_t)&PlcEndatResetActiveAlarms},
#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
    {"sysEndatAddInfoSetup",        (uint32_t)&PlcEndatAddInfoSetup},
#endif // endat22 && endat22_addinfo
#endif

    {"sysILoopDSPLoad",             (uint32_t)&Mh_PlcDSPLoad},
    {"sysILoopDSPDBLoad",           (uint32_t)&Mh_PlcDSPDBLoad},
    {"sysILoopDSPHalt",             (uint32_t)&Mh_PlcDSPHalt},
    {"sysILoopDSPResume",           (uint32_t)&Mh_PlcDSPResume},
    {"RootOfSquareSum16",           (uint32_t)&RootOfSquareSum16},

#if CFG_ETHPMC
    {"sysEpmcMasterParams",         (uint32_t)&EpmcCM_MasterParams},
#endif // cfg_ethpmc

#ifdef _APP_XC
    {"mInterp1D",                   (uint32_t)&_interp1d},
    {"mInterp2D",                   (uint32_t)&_interp2d},
    {"mInterp3D",                   (uint32_t)&_interp3d},
#endif

#if CFG_ENC_NIKON
    {"sysNikonResetActiveAlarms",   (uint32_t)&PlcNikonResetActiveAlarms},
    {"sysNikonMTurnClear",          (uint32_t)&PlcNikonMTurnClear},
    {"sysNikonSetZeroPosition",     (uint32_t)&PlcNikonSetZeroPosition},
    {"sysNikonSetCommandMode",      (uint32_t)&PlcNikonSetCommandMode},
#endif

#if CFG_ENC_TMGW
    {"sysTamagawaResetActiveAlarms",(uint32_t)&PlcTMGWResetActiveAlarms},
    {"sysTamagawaMTurnClear",       (uint32_t)&PlcTMGWMTurnClear},
    {"sysTamagawaSetZeroPosition",  (uint32_t)&PlcTMGWSetZeroPosition},
    {"sysTamagawaSetCommandMode",   (uint32_t)&PlcTMGWSetCommandMode},
#endif

//    {"SetBreakpoint",   (uint32_t)&SetBreakpoint},
    {"sysSSComputeGains",           (uint32_t)&PlcCtrLpGainsCompute},
    {"sysSSSelectGains",            (uint32_t)&PlcCtrLpGainsSelect},
    {"sysHwProductRev",             (uint32_t)&PlcHwProductRev},
};

#ifdef _APP_DEBUG
const UWORD plcCntEmbeddedFunctions=sizeof(plcEmbeddedFunctions)/sizeof(PLCIEC_FCNREC);
#endif
