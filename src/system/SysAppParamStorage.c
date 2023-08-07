/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppParamStorage.c                                       */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : System application parameter storage array                 */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "common\ParamStorageManagement.h"
#include "system\SysAppGlobals.h"
#include "system\SysAppDataCodes.h"
#include "plc\Plc.h"
#include "plc\PlcComDBMgr.h"

#include "drive\HardwareConfig.h"


#include "bus\modbus\ModBusCommandMgrParams.h"

#include "bus\canopen\CanOpenDs301.h"
#if CFG_CANDRV_CMDMGR
#include "bus\canopen\CanOpenCommandMgr.h"
#endif
#include "bus\canopen\CanOpenDs301Externals.h"
#include "bus\ethercat\ECATCommandMgr.h"
#ifdef _HW_DC
#include "bus\ethpmc\EthPMCCommandMgr.h"
#endif

#include "bus\SyncManager.h"

#include "drive\EncoderManager.h"
#include "drive\EncoderEFSeek.h"
#include "drive\EnDatEncoder.h"
#include "drive\HiperfaceEncoder.h"
#include "drive\HallEncoder.h"
#include "drive\IncrementalEncoder.h"
#include "drive\SincosEncoder.h"
#include "drive\BackEmfEncoder.h"
#include "drive\NikonEncoder.h"
#include "drive\TAMAGAWAEncoder.h"
#include "drive\BissEncoder.h"

#include "drive\SpaceSpeedCntrLp.h"
#include "drive\Positioner.h"
#include "drive\MotionController.h"
#include "drive\MotorHandler.h"
#include "drive\Deflux.h"
#include "drive\ThermalModel.h"
#include "drive\PiDcBusCntrLp32bit.h"
#include "drive\DriveTaskController.h"

#ifdef _HW_AXS_CABI35KW
#include "drive\stgap4s.h"
#endif

//****************************************************************************
// Parameter Management Definitions

const PARMGM_LIST psParMgmGlobalList[]=
{
    {DATACODE_PARAM_PLC,                &sPlcParams,                    sizeof(sPlcParams),                 PARMGM_F_DEFAULT,                       NULL,           0},
    {DATACODE_PARAM_MOTORPARAMETERS,    &sGlbMotorParameters,           sizeof(sGlbMotorParameters),        PARMGM_F_DEFAULT,                       &sGlbMotorDefParams, sizeof(sGlbMotorDefParams)},

    {DATACODE_PARAM_DTCTRL_OLD_PARAM,   &sDrvTskCtrlParams.sOld,        sizeof(sDrvTskCtrlParams.sOld),     PARMGM_F_RESTOREONLY|PARMGM_F_NOSETDEFAULT,                   NULL,           0},
    {DATACODE_PARAM_DTCTRL_NEW_PARAM,   &sDrvTskCtrlParams.sNew,        sizeof(sDrvTskCtrlParams.sNew),     PARMGM_F_DEFAULT,                       &sDrvTskCtrlDefParams, sizeof(sDrvTskCtrlDefParams)},

    {DATACODE_PARAM_ENCODER_MANAGER,    &sEm_EncMngrParam,              sizeof(sEm_EncMngrParam),           PARMGM_F_DEFAULT,                       &sEm_EncMngrDefParam, sizeof(sEm_EncMngrDefParam)},
    {DATACODE_PARAM_V14_ENCODER_MANAGER,(HPVOID)&Em_GetV14Params,       sizeof(ENCMGR_V14_PARAMS),          PARMGM_F_RESTOREONLY|PARMGM_F_CALLBACK|PARMGM_F_NOSETDEFAULT, NULL,           0},
    {DATACODE_PARAM_EM_LASTVALIDEPLATE, &sEm_LastValidEPlate,           sizeof(sEm_LastValidEPlate),        PARMGM_F_DEFAULT,                       NULL,           0},

    {DATACODE_PARAM_ENCMGR_EFA,         &sEfsParam,                     sizeof(sEfsParam),                  PARMGM_F_DEFAULT,                       &sEfsDefParam, sizeof(sEfsDefParam)},
    {DATACODE_PARAM_HALL_ENC,           &sHl_HallParam,                 sizeof(sHl_HallParam),              PARMGM_F_DEFAULT,                       &sHl_HallDefParam, sizeof(sHl_HallDefParam)},
    {DATACODE_PARAM_INCR_MAIN_ENC,      &sIc_IncEncParams[INCREMENTAL_SEL_MAIN],sizeof(sIc_IncEncParams[INCREMENTAL_SEL_MAIN]), PARMGM_F_DEFAULT,   &sIc_IncEncDefParams, sizeof(sIc_IncEncDefParams)},
    {DATACODE_PARAM_INCR_AUX__ENC,      &sIc_IncEncParams[INCREMENTAL_SEL_AUX],sizeof(sIc_IncEncParams[INCREMENTAL_SEL_AUX]),   PARMGM_F_DEFAULT,   &sIc_IncEncDefParams, sizeof(sIc_IncEncDefParams)},
    {DATACODE_PARAM_SINCOS_ENC,         &sSc_SinCosParam,               sizeof(sSc_SinCosParam),            PARMGM_F_DEFAULT,                       &sSc_SinCosDefParam, sizeof(sSc_SinCosDefParam)},
    {DATACODE_PARAM_ENDAT_MAIN_ENC,     &sEn_Params[ENDAT_SEL_MAIN],    sizeof(sEn_Params[ENDAT_SEL_MAIN]), PARMGM_F_DEFAULT,                       &sEn_DefParams, sizeof(sEn_DefParams)},
#ifndef _HW_DC
    {DATACODE_PARAM_ENDAT_AUX_ENC,      &sEn_Params[ENDAT_SEL_AUX],     sizeof(sEn_Params[ENDAT_SEL_AUX]),  PARMGM_F_DEFAULT,                       &sEn_DefParams, sizeof(sEn_DefParams)},
#endif
    {DATACODE_PARAM_BACKEMF_ENC,        &sBe_EmfEncParam,               sizeof(sBe_EmfEncParam),            PARMGM_F_DEFAULT,                       &sBe_EmfEncDefParam, sizeof(sBe_EmfEncDefParam)},
    {DATACODE_PARAM_SIMULATED_ENC,      &sSe_AuxSimParam,               sizeof(sSe_AuxSimParam),            PARMGM_F_DEFAULT,                       &sSe_AuxSimDefParam, sizeof(sSe_AuxSimDefParam)},
    {DATACODE_PARAM_HIPFC_ENC,          &sHf_Params,                    sizeof(sHf_Params),                 PARMGM_F_DEFAULT,                       &sHf_DefParams, sizeof(sHf_DefParams)},
    {DATACODE_PARAM_NIKON_MAIN_ENC,     &sNk_Params[NIKON_SEL_MAIN],    sizeof(sNk_Params[NIKON_SEL_MAIN]), PARMGM_F_DEFAULT,                       &sNk_DefParams, sizeof(sNk_DefParams)},
    {DATACODE_PARAM_TMGW_MAIN_ENC,      &sTm_Params[TMGW_SEL_MAIN],     sizeof(sTm_Params[TMGW_SEL_MAIN]),  PARMGM_F_DEFAULT,                       &sTm_DefParams, sizeof(sTm_DefParams)},
    {DATACODE_PARAM_BISS_MAIN_ENC,      &sBiss_Params[BISS_SEL_MAIN],   sizeof(sBiss_Params[BISS_SEL_MAIN]),PARMGM_F_DEFAULT,                       &sBiss_DefParams, sizeof(sBiss_DefParams)},
    {DATACODE_PARAM_BISS_AUX_ENC,       &sBiss_Params[BISS_SEL_AUX],    sizeof(sBiss_Params[BISS_SEL_AUX]), PARMGM_F_DEFAULT,                       &sBiss_DefParams, sizeof(sBiss_DefParams)},

    {DATACODE_PARAM_SPACESPEED_CNTRLOOP,&sSS_CntrLoopParam.sPars,       sizeof(sSS_CntrLoopParam.sPars),    PARMGM_F_DEFAULT,                       &sSS_CntrLoopDefParam, sizeof(sSS_CntrLoopDefParam)},
    {DATACODE_PARAM_V15_SS_CNTRLOOP,    (HPVOID)&Ss_GetV15Params,       sizeof(SS_V15_PARAMS),              PARMGM_F_RESTOREONLY|PARMGM_F_CALLBACK|PARMGM_F_NOSETDEFAULT, NULL,           0},
    {DATACODE_PARAM_POSITIONER,         &sPo_PostnerParam,              sizeof(sPo_PostnerParam),           PARMGM_F_DEFAULT,                       &sPo_PostnerDefParam, sizeof(sPo_PostnerDefParam)},
    {DATACODE_PARAM_MOTIONCONTROLLER,   &sMotCtrlParameters,            sizeof(sMotCtrlParameters),         PARMGM_F_DEFAULT,                       &sMotCtrlDefParameters, sizeof(sMotCtrlDefParameters)},
    {DATACODE_PARAM_MOTORHANDLER,       &sMh_MotorDataParam,            sizeof(sMh_MotorDataParam),         PARMGM_F_DEFAULT,                       &sMh_MotorDataDefParam, sizeof(sMh_MotorDataDefParam)},
    {DATACODE_PARAM_THERMAL_MODEL,      &sTm_ThModParam,                sizeof(sTm_ThModParam),             PARMGM_F_DEFAULT,                       &sTm_ThModDefParam, sizeof(sTm_ThModDefParam)},
    {DATACODE_PARAM_DEFLUX,             &sDflx_Param,                   sizeof(sDflx_Param),                PARMGM_F_DEFAULT,                       &sDflx_DefParam, sizeof(sDflx_DefParam)},

    {DATACODE_PARAM_PI_DCBUS,           &sDcBusCntrl_Param,             sizeof(sDcBusCntrl_Param),          PARMGM_F_DEFAULT,                       &sDcBusCntrl_DefParam, sizeof(sDcBusCntrl_DefParam)},

#ifndef _HW_DC
    {DATACODE_PARAM_MODBUS_BASE,        &sModBusCMParams,               sizeof(sModBusCMParams),            PARMGM_F_DEFAULT,                       &sModBusCMDefParams, sizeof(sModBusCMDefParams)},
    {DATACODE_PARAM_MODBUS_OVERSERIAL0, &sModBusCMParOverserial0,       sizeof(sModBusCMParOverserial0),    PARMGM_F_DEFAULT,                       &sModBusCMDefParOverserial0, sizeof(sModBusCMDefParOverserial0)},
#endif

#ifdef CFG_EN_MODBUSOVERETH
    {DATACODE_PARAM_MODBUS_OVERETHERNET,&sModBusCMParOverEth,           sizeof(sModBusCMParOverEth),        PARMGM_F_DEFAULT,                       &sModBusCMDefParOverEth, sizeof(sModBusCMDefParOverEth)},
#endif

    {DATACODE_PARAM_SYNCMANAGER,        &sSyncMgrParam,                 sizeof(sSyncMgrParam),              PARMGM_F_DEFAULT,                       &sSyncMgrDefParam, sizeof(sSyncMgrDefParam)},

#if CFG_CANDRV_DS301
    {DATACODE_PARAM_CANOPEN_BASE,       &tDs301Param,                   sizeof(tDs301Param),                PARMGM_F_DEFAULT,                       &tDs301DefParam, sizeof(tDs301DefParam)},
    {DATACODE_PARAM_CANOPEN_TXCOB,      &tDs301PdoParamTx,              sizeof(tDs301PdoParamTx),           PARMGM_F_DEFAULT,                       &tDs301PdoDefParamTx, sizeof(tDs301PdoDefParamTx)},
    {DATACODE_PARAM_CANOPEN_RXCOB,      &tDs301PdoParamRx,              sizeof(tDs301PdoParamRx),           PARMGM_F_DEFAULT,                       &tDs301PdoDefParamRx, sizeof(tDs301PdoDefParamRx)},
#endif

#if CFG_CANDRV_CMDMGR
    {DATACODE_PARAM_CANOPENCOMMANDMGR,  &tCanOpenCMParams,              sizeof(tCanOpenCMParams),           PARMGM_F_DEFAULT,                       &tCanOpenCMDefParams, sizeof(tCanOpenCMDefParams)},
#endif

#if CFG_ECAT
    {DATACODE_PARAM_ECAT_BASE,          &tEcatCMParam,                  sizeof(tEcatCMParam),               PARMGM_F_DEFAULT,                       &tEcatCMDefParam, sizeof(tEcatCMDefParam)},
    {DATACODE_PARAM_ECAT_TXPDOMAP,      &tEcatCMTxPdoMap,               sizeof(tEcatCMTxPdoMap),            PARMGM_F_DEFAULT,                       &tEcatCMDefTxPdoMap, sizeof(tEcatCMDefTxPdoMap)},
    {DATACODE_PARAM_ECAT_RXPDOMAP,      &tEcatCMRxPdoMap,               sizeof(tEcatCMRxPdoMap),            PARMGM_F_DEFAULT,                       &tEcatCMDefRxPdoMap, sizeof(tEcatCMDefRxPdoMap)},
    {DATACODE_PARAM_ECAT_SMOUTMAP,      &tEcatCMOutSyncPdoMap,          sizeof(tEcatCMOutSyncPdoMap),       PARMGM_F_DEFAULT,                       &tEcatCMDefOutSyncPdoMap, sizeof(tEcatCMDefOutSyncPdoMap)},
    {DATACODE_PARAM_ECAT_SMINMAP,       &tEcatCMInSyncPdoMap,           sizeof(tEcatCMInSyncPdoMap),        PARMGM_F_DEFAULT,                       &tEcatCMDefInSyncPdoMap, sizeof(tEcatCMDefInSyncPdoMap)},
    {DATACODE_PARAM_ECAT_SYNCMGR,       &tEcatCMSMOutParam,             sizeof(tEcatCMSMOutParam),          PARMGM_F_DEFAULT,                       NULL,           0},
#endif // CFG_ECAT

#if CFG_ETHPMC
    {DATACODE_PARAM_ETHERPMC,           &sEpmcCM_Params,                sizeof(sEpmcCM_Params),             PARMGM_F_DEFAULT,                       &sEpmcCM_DefParams, sizeof(sEpmcCM_DefParams)},
#endif // cfg_ethpmc

#ifdef _HW_AXS_CABI35KW
    {DATACODE_PARAM_STGAP4S_CONFIG,     &stgap4sCONFIG_PARAMS,          sizeof(stgap4sCONFIG_PARAMS),       PARMGM_F_DEFAULT,                       &stgap4sCONFIG_PARAMSdefault, sizeof(stgap4sCONFIG_PARAMSdefault)},
#endif

    {DATACODE_PARAM_PLC_USRPARAM_COMMON,&tPlcParamCommon,               sizeof(tPlcParamCommon),            PARMGM_F_DEFAULT,                       NULL,           0},
    {DATACODE_PARAM_PLC_USRPARAM_BITS,  &tPlcParamBits,                 sizeof(tPlcParamBits),              PARMGM_F_DEFAULT,                       NULL,           0},
    {DATACODE_PARAM_PLC_USRPARAM_ID,    &sPlcUsrParamsID,               sizeof(sPlcUsrParamsID),            PARMGM_F_NOSETDEFAULT,                  NULL,           0},
    {DATACODE_PARAM_PLC_USRPARAM_MAP,   &sPlcComParCurrCheck,           sizeof(sPlcComParCurrCheck),        PARMGM_F_STOREONLY|PARMGM_F_PTRSOURCE|PARMGM_F_NOSETDEFAULT, NULL, 0},
    {DATACODE_PARAM_PLC_USRPARAM_MAP,   (HPVOID)&PlcComDBAppRestParChk, 0,                                  PARMGM_F_RESTOREONLY|PARMGM_F_CALLBACK|PARMGM_F_NOSETDEFAULT, NULL, 0},

    {DATACODE_PARAM_HWCONFIG,           &sHwConfParams,                 sizeof(sHwConfParams),              PARMGM_F_CORE,                          &sHwConfDefParams, sizeof(sHwConfDefParams)},
};

const UWORD uwParMgmGlobalListSize=sizeof(psParMgmGlobalList)/sizeof(PARMGM_LIST);
