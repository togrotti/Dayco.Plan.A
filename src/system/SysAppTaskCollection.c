/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppTaskCollection.c                                     */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Collection of all applications that will be initialized    */
/*				 and run both in background and realtime					*/
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "common\CommonDefines.h"
#include "system\SysLogManagement.h"
#include "SystemStatus.h"
#include "SysAppTaskCollection.h"

#include "drive\HardwareConfig.h"
#include "fpga\AnProcessor.h"
#include "common\IOManager.h"
#include "drive\UserIO.h"
#ifdef _INFINEON_
#include "IOExpBrdManager.h"
#include "RDHmiServer.h"
#endif
#include "drive\PanelManager.h"
#include "bus\modbus\ModBusCommandMgr.h"
#include "bus\canopen\CanOpenCOECommon.h"
#if CFG_CANDRV_CMDMGR
#include "bus\canopen\CanOpenCommandMgr.h"
#include "bus\canopen\MultiCANController.h"
#endif
#include "bus\ethercat\ECATCommandMgr.h"
#ifdef _HW_DC
#include "bus\ethpmc\EthPMCCommandMgr.h"
#include "drive\PSUManager.h"
#endif
#include "bus\SyncManager.h"
#include "plc\SoftScope.h"
#include "drive\DriveTaskController.h"
#include "drive\MotorHandler.h"
#include "plc\Plc.h"
#include "common\ParametersCheck.h"
#include "common\WatchDogManagement.h"

#ifdef _APP_DEBUG
#include "common\CommonParamDB.h"
#include "bus\modbus\ModBusComDB.h"
#include "bus\canopen\CanOpenComDB.h"
#endif

#ifdef _HW_CT
#include "Sensor.h"
#endif


#ifdef _HW_AXS_CABI35KW
  #include "drive\stgap4s.h"
#endif



#ifndef _DEBUG_TRACES
  #define TASK_ENTRY(name, param) {&name, param}
#else
  #define TASK_ENTRY(name, param) {&name, param, #name}
#endif


//****************************************************************************
// Task list, valid hw startup

const TASKSCHEDULER_TASK_INIT psSysAppTaskCollection[]=
{
#ifdef _APP_DEBUG
	TASK_ENTRY(ComParDBCheckTable, 0),
	TASK_ENTRY(ModBusComDBCheckTable, 0),
	TASK_ENTRY(CanOpenComDBCheckTable, 0),
#endif // app_debug
	TASK_ENTRY(SysLogMgm_Init, 0),
	TASK_ENTRY(ParChk_Init, 0),

	TASK_ENTRY(HwConfInit, 0),

#ifndef _HW_AXS
#if CFG_ECAT
	TASK_ENTRY(EcatCM_Init, ECATCM_INIT_ALWAYS),
#endif
#ifdef _HW_DC
	TASK_ENTRY(EpmcCM_Init, EPMCCM_INIT_ALWAYS),
#endif // _hw_dc
#endif // _HW_AXS

#ifndef _APP_LIMITED
#if CFG_CANDRV_CMDMGR
	TASK_ENTRY(candrv_init, 0),
#endif
#endif // !_app_limited

	TASK_ENTRY(AnProc_Init, ANPROC_INIT_CORE),

	TASK_ENTRY(IOMgr_Init, 1),
	TASK_ENTRY(IOMgr_Init, 2),

#ifndef _APP_LIMITED
	TASK_ENTRY(CanOpenCOE_Init, 0),
#endif // !_app_limited

    TASK_ENTRY(PlcInit,PLC_INIT_CORE),
    TASK_ENTRY(PlcInit,PLC_INIT_USRPARINSTALL),
    TASK_ENTRY(PlcInit,PLC_INIT_BOOTCONFIG),

    TASK_ENTRY(PanelMgr_Init, 0),
    TASK_ENTRY(UserIO_Init, 0),

#ifdef _HW_AXS_CABI35KW
	TASK_ENTRY(STgap4s_Initialize, 0),
	TASK_ENTRY(STgap4s_ConfigDrivers, 0),
#endif // _hw_axs_cabi35kw


#ifdef CFG_EN_MODBUSOVERCAN
#ifdef _INFINEON_
	TASK_ENTRY(RDHmiSrv_Init, 0),
#endif
#endif

	TASK_ENTRY(ModBusCM_Init, 0),

#ifndef _APP_LIMITED
#if CFG_CANDRV_CMDMGR
	TASK_ENTRY(CanOpenCM_Init, CANOPENCM_INIT_CORE),
#endif // cfg_candrv_cmdmgr
	
#if CFG_ECAT
	TASK_ENTRY(EcatCM_Init, ECATCM_INIT_CORE),
#endif // cfg_ecat

#if CFG_ETHPMC
	TASK_ENTRY(EpmcCM_Init, EPMCCM_INIT_CORE),
#endif // !CFG_ETHPMC

#if CFG_CANDRV_CMDMGR
	TASK_ENTRY(CanOpenCM_Init, CANOPENCM_INIT_SYNCEVENT),
#endif  // cfg_candrv_cmdmgr

#if CFG_ECAT
	TASK_ENTRY(EcatCM_Init, ECATCM_INIT_SYNCEVENT),
#endif // cfg_ecat

#if CFG_ETHPMC
	TASK_ENTRY(EpmcCM_Init, EPMCCM_INIT_SYNCEVENT),
#endif // cfg_ethpmc

#if CFG_PSU_MNGR
	TASK_ENTRY(PSUManager_Init, 0),
#endif // cfg_psu_manager

#ifdef _HW_CT
	TASK_ENTRY(Sensor_Init, 0),
#endif // _hw_ct

        // SyncMgr init after all fieldbuses
        // that could need it
	TASK_ENTRY(SyncMgrInit, 0),
#endif // !_app_limited

	TASK_ENTRY(PlcInit, PLC_INIT_TASKS),
	TASK_ENTRY(WdtMgm_Init, 0),

	TASK_ENTRY(Mh_MotorHandlerInit, MH_INIT_LIMITLIST),

	TASK_ENTRY(DrvTskCtrl_Init, 0),

	TASK_ENTRY(AnProc_Init, ANPROC_INIT_RUN),

    TASK_ENTRY(WdtMgm_Init, 1),

	TASK_ENTRY(PlcInit, PLC_INIT_OTHERS),

#ifndef _APP_LIMITED
#if CFG_CANDRV_CMDMGR
	TASK_ENTRY(CanOpenCM_Init, CANOPENCM_INIT_SYNCPDOEVENT),
#endif // cfg_candrv_cmdmgr

#ifndef _HW_AXS
#if CFG_ECAT
	TASK_ENTRY(EcatCM_Init, ECATCM_INIT_SYNCPDOEVENT),
#endif // cfg_ecat
#endif // _hw_axs
#endif // ! _app_limited
    TASK_ENTRY(SoftScopeInit, 0),

	TASK_ENTRY(Mh_MotorHandlerInit, MH_INIT_POST),

        // Terminator
    {NULL, 0}
};

//****************************************************************************
// Task list, alternate startup just for configuration without modbus flash pars

const TASKSCHEDULER_TASK_INIT psSysAppTaskAltCollection[]=
{
	TASK_ENTRY(SysLogMgm_Init, 0),

	TASK_ENTRY(ParChk_Init, 0),

	TASK_ENTRY(PanelMgr_Init, 0),

	TASK_ENTRY(ModBusCM_Init, 1),

	TASK_ENTRY(PlcInit, PLC_INIT_CORE),

        // Terminator
    {NULL, 0}
};

//****************************************************************************
// Task list, alternate startup just for configuration with modbus flash pars

const TASKSCHEDULER_TASK_INIT psSysAppTaskAltCollectWPars[]=
{
	TASK_ENTRY(SysLogMgm_Init, 0),

	TASK_ENTRY(ParChk_Init, 0),

	TASK_ENTRY(PanelMgr_Init, 0),

	TASK_ENTRY(ModBusCM_Init, 0),

	TASK_ENTRY(PlcInit, PLC_INIT_CORE),

        // Terminator
    {NULL, 0}
};

//****************************************************************************
// Task list, alternate startup just for full parameters download

const TASKSCHEDULER_TASK_INIT psSysAppTaskAltCollectFullCfg[]=
{
	TASK_ENTRY(SysLogMgm_Init, 0),
	TASK_ENTRY(ParChk_Init, 0),

#if CFG_CANDRV_CMDMGR
	  TASK_ENTRY(candrv_init, 0),
#endif

	TASK_ENTRY(PanelMgr_Init, 0),
#ifdef CFG_EN_MODBUSOVERCAN
#ifdef _INFINEON_
	TASK_ENTRY(RDHmiSrv_Init, 0),
#endif
#endif

	TASK_ENTRY(ModBusCM_Init, 0),

	TASK_ENTRY(PlcInit, PLC_INIT_CORE),
	TASK_ENTRY(PlcInit, PLC_INIT_USRPARINSTALL),

        // Terminator
    {NULL, 0}
};
