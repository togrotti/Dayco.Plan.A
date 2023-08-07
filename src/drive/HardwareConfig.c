/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : HardwareConfig.c                                           */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Configuration check of hardware related parameters and     */
/*               switches, related to configuration stored in bootblock     */
/*               area and on 1 wire serial IC                               */
/*                                                                          */
/****************************************************************************/

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "system\HeaderFooterInfo.h"
#include "common\AppIdentTypes.h"
#include "common\BlockStorage.h"
#include "common\FlashManager.h"
#include "common\ProgramFlashHandler.h"
#include "HardwareUnitID.h"
#include "HardwareConfig.h"
#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "system\SysAppDataCodes.h"
//#include "FatalErrorCodes.h"
#include "OneWireHandler.h"
#include "fpga\FpgaHandler.h"
#include "plc\Plc.h"
#include "system\GlobalResetCodes.h"
#include "core\SystemReset.h"
#include "common\TaskScheduler.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include <string.h>

#if CFG_IIC
#include "common\IICHandler.h"
#endif // cfg_iic
//***************************************************************************
// Avoids warning C47: unreferenced parameter

//#pragma warning disable = 47

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

//***************************************************************************
// Defines

#define WAIT_FLASHMGR_LOCK          1500            // msec
#define RESCAN_OW_TIMES             3

//***************************************************************************
// Data structure

typedef struct
{
    HPVOID hpvStart;
    ULONG ulSize;
} BLOCKDEFS;

typedef struct _hwdata_list
{
	UWORD   uwDataCode;
	HPVOID  hpvData;
	UWORD   uwDataSize;
    BOOL   (* pfCallBack)(const struct _hwdata_list * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
    struct
    {
        UBYTE   ubOWOnly:1;
        UBYTE   ub2ndValid:1;
        UBYTE   ubOWValidBus:5;
        UBYTE   ubUID:1;
    } src;
    SBYTE   sbCnt;
    UWORD * puwDestDataCode;
} HWDATA_LIST;

//***************************************************************************
// Globals
//SWORD tempbit = 0;
HWCONF_FPGACONFIG  tHwFpgaConfig;

HWCONF_FPGACONFIG  tHwIOExpBoardConfig;

HWCONF_PARAMS sHwConfParams;

//****************************************************************************
// Power board configuration and parameters
UWORD  		       sGlbPowerBoardParamDataCode;
HWPRM_POWER_BOARD  sGlbPowerBoardParameters;

//****************************************************************************
// Control board configuration and parameters
HWPRM_ALL_CBRD sGlbControlBoardParameters;

//****************************************************************************
// IO expansion board configuration and parameters
HWPRM_IO_EXP_BOARD  sGlbIOExpBoardParameters;

//****************************************************************************
// Assembly info
ASSEMBLY_INFO  sGlbAssemblyInfo;

//****************************************************************************
// Hardware options configuration requested/available
GLB_REQ_OPT  sGlbOptReq;
GLB_REQ_OPT  sGlbOptAvailable;

//****************************************************************************
// Default parameters

const HWCONF_PARAMS sHwConfDefParams =
{
#ifndef _HW_DC
    HWCONF_SYSMODE_COMP,
#else
    HWCONF_SYSMODE_PERF,
#endif
};

#ifdef _APP_XC
UWORD  uwSystemTmr100Corr;          // fractional correction for
                                                // 100ns timer
UWORD  uwSystemSetTmr100;           // fractional correction for
                                                // correct setting of 100ns timer
UWORD  uwSystemFBusTSCorr;          // fractional correction for
                                                // fieldbus realtime timestamp
#endif

//***************************************************************************
// Linker defined publics

//#ifndef _HW_DC
//extern UBYTE BOOTBLOCKPARAM_START;
//extern UBYTE BOOTBLOCKPARAM_SIZE;
//#endif

//***************************************************************************
// Locals

static HWCONF_CFGBLOCKS tHwCfgBlocks[HWCONF_MAXCFGBLOCKS];

    // internal flash bootblock data list
#ifndef _HW_DC
static const BLOCKDEFS sHwConfBlockDefs[]=
{
    {(HPVOID)BOOTBLOCKPARAM_START, (ULONG)BOOTBLOCKPARAM_SIZE},
};

#define HWCONFBLOCKCOUNT     (sizeof(sHwConfBlockDefs)/sizeof(BLOCKDEFS))
#endif

    // unique system ID
static UBYTE ubUniqueID[8]={0,0,0,0,0,0,0,0};

//***************************************************************************
// config blocks

const HPVOID hpvHwCfgBlocksAddr=&tHwCfgBlocks[0];
UWORD uwHwCfgBlocksSz=0;
const UWORD uwHwCfgBlocksRecordLength=sizeof(HWCONF_CFGBLOCKS);

//***************************************************************************
// Local prototypes

static void erasealldest(const HWDATA_LIST * pDatlst, UWORD uwDatLstSz);
static BOOL retrieveblock(const HWDATA_LIST * pDat, SWORD blkcode, HPVOID pfound, UWORD * blkrev);
static void owbusscan(SWORD swBus, void (* yield)(void), const HWDATA_LIST * pDatlst, UWORD uwDatLstSz);
#if CFG_IIC
static void iicbusscan( UBYTE ubBus, void (* yield)(void), const HWDATA_LIST * pDatlst, UWORD uwDatLstSz );
#endif
static BOOL configvalidater(void);
static BOOL retrieveconfiguration(void (* yield)(void));
static BOOL checkknownhardware(void);
#ifdef _HW_DC
static BOOL retrievepwbconfiguration(void);
#endif
#ifndef _HW_DC
static BOOL _addblock(SWORD swCode, HPVOID hpvSrc, UWORD uwSize, HPVOID * ppvDest, ULONG * pulDestSize);
static BOOL convertV19assembly(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
#endif
static BOOL parameterscheck(void);

#ifndef _HW_DC
static BOOL onGenericV19(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
static BOOL onGenericPwr(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
#endif

static BOOL onAssembly(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);

static BOOL onTsGdb(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);

#ifndef _HW_DC
static BOOL onMbp(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
#endif
static BOOL onDsMbp(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
static BOOL onDsPsi(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
#ifndef _HW_DC
static BOOL onTsPsi(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
static BOOL onTsFtb(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);

static BOOL onMergeCurrentRange(HWPRM2_CURRENT_RANGE * pMPB, HWPRM2_CURRENT_RANGE * pASM);
#else
static BOOL onXsPsi(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
static BOOL onXsBri(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);

static BOOL onPsuPsb(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
static BOOL onPsuBrk(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
static BOOL onPsuScr(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);

static BOOL cbconfigvalidater(void);
static BOOL onCtCb(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
static BOOL onCtEm(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
static BOOL onCtEa(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
static BOOL onCtIo(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
static BOOL onCtBp(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
static BOOL onCtPsu(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev);
#endif

static BOOL onMergeDCCharge(HWPRM2_DC_CHARGE * pMPB, HWPRM2_DC_CHARGE * pASM);
static BOOL onMergeCurrentLayout(HWPRM2_CURRENT_LAYOUT * pMPB, HWPRM2_CURRENT_LAYOUT * pPSI);
static BOOL onMergeVoltageLayout(HWPRM2_VOLTAGE_LAYOUT * pPSI, HWPRM2_VOLTAGE_LAYOUT * pFTB);
static BOOL onMergeTempSens(HWPRMS_AD_SETTINGS * pMPB, HWPRMS_AD_SETTINGS * pPSI);
static void mergeADsettings(HWPRMS_AD_SETTINGS * pExt, HWPRMS_AD_SETTINGS * pMid, HWPRMS_AD_SETTINGS * pDst);

//***************************************************************************
// Configuration structures handling

#define CNTP_DISABLED               -1

#define CNTP_ASSEMBLY_IDENT         0
#define CNTP_GENERIC_POWER_BRD      1
#define CNTP_PB_ASSEMBLY            2
#define CNTP_PB_MBP                 3
#define CNTP_PB_DS_MBP              4
#define CNTP_PB_DS_PSI              5
#define CNTP_PB_TS_PSI              6
#define CNTP_PB_TS_FTB              7
#define CNTP_PB_TS_GDB              8
#define CNTP_PB_XS_PSI              9
#define CNTP_PB_XS_BRI              10
#define CNTP_CB_ASSEMBLY            11
#define CNTP_CB_XS_PSI              12
#define CNTP_CB_XS_BRI              13
#define CNTP_CB_PSU_PSB             14
#define CNTP_CB_PSU_BRK             15
#define CNTP_CB_PSU_SCR             16

#define CNTP_PB_SIZE                (CNTP_CB_PSU_SCR+1)

#ifndef _HW_DC
static const HWDATA_LIST sHwIntConfDataDefs[]=
{
    {DATACODE_HW_CNTRL_BOARD,
        &sGlbControlBoardParameters.sStd, sizeof(sGlbControlBoardParameters.sStd), &onGenericV19,
        {FALSE,FALSE,OW_INTERNAL_BUS,TRUE}, CNTP_DISABLED, NULL},
};

#define HWINTDATALISTCOUNT  (sizeof(sHwIntConfDataDefs)/sizeof(HWDATA_LIST))
#endif

static const HWDATA_LIST sHwConfDataDefs[]=
{
#ifndef _HW_AXS

#ifndef _HW_DC
    {DATACODE_HW_POWER_BOARD_AXM_AXP_AC,
        &sGlbPowerBoardParameters.sAxmAxpAC, sizeof(sGlbPowerBoardParameters.sAxmAxpAC), &onGenericPwr,
        {FALSE,FALSE,OW_EXTERNAL_BUS}, CNTP_GENERIC_POWER_BRD, &sGlbPowerBoardParamDataCode},

    {DATACODE_HW_POWER_BOARD_AXM_AXP_AC_V2,
        &sGlbPowerBoardParameters.sAxmAxpACV2, sizeof(sGlbPowerBoardParameters.sAxmAxpACV2), &onGenericPwr,
        {FALSE,FALSE,OW_EXTERNAL_BUS}, CNTP_GENERIC_POWER_BRD, &sGlbPowerBoardParamDataCode},

    {DATACODE_HW_POWER_BOARD_DIF,
        &sGlbPowerBoardParameters.sDif, sizeof(sGlbPowerBoardParameters.sDif), &onGenericPwr,
        {FALSE,FALSE,OW_EXTERNAL_BUS}, CNTP_GENERIC_POWER_BRD, &sGlbPowerBoardParamDataCode},

    {DATACODE_HW_ASSEMBLY_V19_IDENT,
        &sGlbAssemblyInfo, sizeof(sGlbAssemblyInfo), &convertV19assembly,
        {FALSE,TRUE,OW_INTERNAL_BUS}, CNTP_ASSEMBLY_IDENT, NULL},

    {DATACODE_HW_ASSEMBLY_IDENT,
        &sGlbAssemblyInfo, sizeof(sGlbAssemblyInfo), NULL,
        {FALSE,TRUE,OW_INTERNAL_BUS}, CNTP_ASSEMBLY_IDENT, NULL},

    {DATACODE_HW_IO_EXP_BOARD,
        &sGlbIOExpBoardParameters, sizeof(sGlbIOExpBoardParameters), &onGenericV19,
        {TRUE,FALSE,OW_IOEXP_BUS}, CNTP_DISABLED, NULL},
#endif

    {DATACODE_SYSLOG_INVALID,   // only for global erase
    	&sGlbPowerBoardParameters.sUniv.sProductInfo.uwProductCode, sizeof(sGlbPowerBoardParameters.sUniv.sProductInfo.uwProductCode), NULL,/*&sGlbPowerBoardParameters.sUniv, sizeof(sGlbPowerBoardParameters.sUniv), NULL,*/
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_DISABLED, &sGlbPowerBoardParamDataCode},

#if CFG_IIC
    {DATACODE_SYSLOG_INVALID,   // only for global erase
	    &sGlbPowerBoardParameters.sUniv.sProductInfo.uwProductCode, sizeof(sGlbPowerBoardParameters.sUniv.sProductInfo.uwProductCode), NULL,
	    {TRUE,FALSE,IIC_EXTERNAL_BUS}, CNTP_DISABLED, &sGlbPowerBoardParamDataCode},      // IIC
#endif

    {DATACODE_HW_PB_ASSEMBLY,
        NULL, sizeof(HWGRP_ASSEMBLY), onAssembly,
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_PB_ASSEMBLY, NULL},

#if CFG_IIC
    {DATACODE_HW_PB_ASSEMBLY,
	    NULL, sizeof(HWGRP_ASSEMBLY), onAssembly,
	    {TRUE,FALSE,IIC_EXTERNAL_BUS}, CNTP_PB_ASSEMBLY, NULL},                           // IIC
#endif

#ifndef _HW_DC
    {DATACODE_HW_PB_MBP,
        NULL, sizeof(HWGRP_MBP), onMbp,
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_PB_MBP, NULL},
#endif

    {DATACODE_HW_PB_DS_MBP,
        NULL, sizeof(HWGRP_DS_MBP), onDsMbp,
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_PB_DS_MBP, NULL},

#if CFG_IIC
	{DATACODE_HW_PB_DS_MBP,
		NULL, sizeof(HWGRP_DS_MBP), onDsMbp,
		{TRUE,FALSE,IIC_EXTERNAL_BUS}, CNTP_PB_DS_MBP, NULL},    // IIC
#endif

    {DATACODE_HW_PB_DS_PSI,
        NULL, sizeof(HWGRP_DS_PSI), onDsPsi,
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_PB_DS_PSI, NULL},

#if CFG_IIC
	{DATACODE_HW_PB_DS_PSI,
		NULL, sizeof(HWGRP_DS_PSI), onDsPsi,
		{TRUE,FALSE,IIC_EXTERNAL_BUS}, CNTP_PB_DS_PSI, NULL},    // IIC
#endif

#ifndef _HW_DC
    {DATACODE_HW_PB_TS_PSI,
        NULL, sizeof(HWGRP_TS_PSI), onTsPsi,
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_PB_TS_PSI, NULL},

    {DATACODE_HW_PB_TS_FTB,
        NULL, sizeof(HWGRP_TS_FTB), onTsFtb,
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_PB_TS_FTB, NULL},
#endif

    {DATACODE_HW_PB_TS_GDB,
        NULL, sizeof(HWGRP_TS_GDB), onTsGdb,
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_PB_TS_GDB, NULL},

#if CFG_IIC
	{DATACODE_HW_PB_TS_GDB,
	    NULL, sizeof(HWGRP_TS_GDB), onTsGdb,
	    {TRUE,FALSE,IIC_EXTERNAL_BUS}, CNTP_PB_TS_GDB, NULL},     // IIC
#endif

#ifdef _HW_DC
    {DATACODE_HW_PB_XS_PSI,
        NULL, sizeof(HWGRP_XS_PSI), onXsPsi,
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_PB_XS_PSI, NULL},

#if CFG_IIC
	{DATACODE_HW_PB_XS_PSI,
		NULL, sizeof(HWGRP_XS_PSI), onXsPsi,
		{TRUE,FALSE,IIC_EXTERNAL_BUS}, CNTP_PB_XS_PSI, NULL},    // IIC
#endif

    {DATACODE_HW_PB_XS_BRI,
        NULL, sizeof(HWGRP_XS_BRI), onXsBri,
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_PB_XS_BRI, NULL},

#if CFG_IIC
	{DATACODE_HW_PB_XS_BRI,
		NULL, sizeof(HWGRP_XS_BRI), onXsBri,
		{TRUE,FALSE,IIC_EXTERNAL_BUS}, CNTP_PB_XS_BRI, NULL},    // IIC
#endif

    {DATACODE_HW_PB_ASSEMBLY,
        NULL, sizeof(HWGRP_ASSEMBLY), onAssembly,
        {TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_ASSEMBLY, NULL},  // EV

#if CFG_IIC
	{DATACODE_HW_PB_ASSEMBLY,
		NULL, sizeof(HWGRP_ASSEMBLY), onAssembly,
		{TRUE,FALSE,IIC_INTERNAL_BUS}, CNTP_CB_ASSEMBLY, NULL},  // IIC
#endif

    {DATACODE_HW_PB_XS_PSI,
        NULL, sizeof(HWGRP_XS_PSI), onXsPsi,
        {TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_XS_PSI, NULL},    // EV

#if CFG_IIC
	{DATACODE_HW_PB_XS_PSI,
		NULL, sizeof(HWGRP_XS_PSI), onXsPsi,
		{TRUE,FALSE,IIC_INTERNAL_BUS}, CNTP_CB_XS_PSI, NULL},    // IIC
#endif

    {DATACODE_HW_PB_XS_BRI,
        NULL, sizeof(HWGRP_XS_BRI), onXsBri,
        {TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_XS_BRI, NULL},    // EV

#if CFG_IIC
	{DATACODE_HW_PB_XS_BRI,
		NULL, sizeof(HWGRP_XS_BRI), onXsBri,
		{TRUE,FALSE,IIC_INTERNAL_BUS}, CNTP_CB_XS_BRI, NULL},    // IIC
#endif

    {DATACODE_HW_PB_PSU_PSB,
        NULL, sizeof(HWGRP_PSU_PSB), onPsuPsb,
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_CB_PSU_PSB, NULL},   // PSU

#if CFG_IIC
	{DATACODE_HW_PB_PSU_PSB,
		NULL, sizeof(HWGRP_PSU_PSB), onPsuPsb,
		{TRUE,FALSE,IIC_EXTERNAL_BUS}, CNTP_CB_PSU_PSB, NULL},    // IIC
#endif

    {DATACODE_HW_PB_PSU_BRK,
        NULL, sizeof(HWGRP_PSU_BRK), onPsuBrk,
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_CB_PSU_BRK, NULL},   // PSU

#if CFG_IIC
	{DATACODE_HW_PB_PSU_BRK,
		NULL, sizeof(HWGRP_PSU_BRK), onPsuBrk,
		{TRUE,FALSE,IIC_EXTERNAL_BUS}, CNTP_CB_PSU_BRK, NULL},    // IIC
#endif

    {DATACODE_HW_PB_PSU_SCR,
        NULL, sizeof(HWGRP_PSU_SCR), onPsuScr,
        {TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_CB_PSU_SCR, NULL},   // PSU

#if CFG_IIC
	{DATACODE_HW_PB_PSU_SCR,
		NULL, sizeof(HWGRP_PSU_SCR), onPsuScr,
		{TRUE,FALSE,IIC_EXTERNAL_BUS}, CNTP_CB_PSU_SCR, NULL},    // IIC
#endif

#endif // _HW_DC


#else // _HW_AXS

#if defined(_HW_AXS_SAIETTA)
	{DATACODE_HW_PB_ASSEMBLY,
		NULL, sizeof(HWGRP_ASSEMBLY), onAssembly,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_ASSEMBLY, NULL},

	{DATACODE_HW_PB_XS_PSI,
		NULL, sizeof(HWGRP_XS_PSI), onXsPsi,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_XS_PSI, NULL},

	{DATACODE_HW_PB_XS_BRI,
		NULL, sizeof(HWGRP_XS_BRI), onXsBri,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_XS_BRI, NULL},

	{DATACODE_HW_PB_TS_GDB,
		NULL, sizeof(HWGRP_TS_GDB), onTsGdb,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_PB_TS_GDB, NULL},

	{DATACODE_SYSLOG_INVALID,   // only for global erase
		&sGlbPowerBoardParameters.sUniv, sizeof(sGlbPowerBoardParameters.sUniv), NULL,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_DISABLED, &sGlbPowerBoardParamDataCode},

#elif defined(_HW_AXS_CABI35KW) // temp all INTERNAL
	{DATACODE_HW_PB_ASSEMBLY,
		NULL, sizeof(HWGRP_ASSEMBLY), onAssembly,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_ASSEMBLY, NULL},

	{DATACODE_HW_PB_XS_PSI,
		NULL, sizeof(HWGRP_XS_PSI), onXsPsi,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_XS_PSI, NULL},

	{DATACODE_HW_PB_XS_BRI,
		NULL, sizeof(HWGRP_XS_BRI), onXsBri,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_XS_BRI, NULL},

	{DATACODE_HW_PB_TS_GDB,
		NULL, sizeof(HWGRP_TS_GDB), onTsGdb,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_PB_TS_GDB, NULL},

	{DATACODE_SYSLOG_INVALID,   // only for global erase
		&sGlbPowerBoardParameters.sUniv, sizeof(sGlbPowerBoardParameters.sUniv), NULL,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_DISABLED, &sGlbPowerBoardParamDataCode},

#elif defined(_HW_AXS_DAYCO22KW)
		{DATACODE_HW_PB_ASSEMBLY,
			NULL, sizeof(HWGRP_ASSEMBLY), onAssembly,
			{TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_CB_ASSEMBLY, NULL},

		{DATACODE_HW_PB_XS_PSI,
			NULL, sizeof(HWGRP_XS_PSI), onXsPsi,
			{TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_CB_XS_PSI, NULL},

		{DATACODE_HW_PB_XS_BRI,
			NULL, sizeof(HWGRP_XS_BRI), onXsBri,
			{TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_CB_XS_BRI, NULL},

		{DATACODE_HW_PB_TS_GDB,
			NULL, sizeof(HWGRP_TS_GDB), onTsGdb,
			{TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_PB_TS_GDB, NULL},

		{DATACODE_SYSLOG_INVALID,   // only for global erase
			&sGlbPowerBoardParameters.sUniv, sizeof(sGlbPowerBoardParameters.sUniv), NULL,
			{TRUE,FALSE,OW_EXTERNAL_BUS}, CNTP_DISABLED, &sGlbPowerBoardParamDataCode},

#endif

#endif // _HW_AXS

};

#define HWDATALISTCOUNT     (sizeof(sHwConfDataDefs)/sizeof(HWDATA_LIST))

    // counters for data structures found
static UBYTE ubBlkFoundCnt[CNTP_PB_SIZE];

#ifdef _HW_DC
#if CFG_IIC
static const HWDATA_LIST sPbAsmConfDataDefs=
    {DATACODE_HW_PB_ASSEMBLY,
        NULL, sizeof(HWGRP_ASSEMBLY), onAssembly,
        {TRUE,FALSE,IIC_CTOPT_BUS}, CNTP_PB_ASSEMBLY, NULL};

static const HWDATA_LIST sPbXsPsiConfDataDefs=
    {DATACODE_HW_PB_XS_PSI,
        NULL, sizeof(HWGRP_XS_PSI), onXsPsi,
        {TRUE,FALSE,IIC_CTOPT_BUS}, CNTP_PB_XS_PSI, NULL};

static const HWDATA_LIST sPbXsBriConfDataDefs=
    {DATACODE_HW_PB_XS_BRI,
        NULL, sizeof(HWGRP_XS_BRI), onXsBri,
        {TRUE,FALSE,IIC_CTOPT_BUS}, CNTP_PB_XS_BRI, NULL};

static const HWDATA_LIST sPbTsGdbConfDataDefs=
    {DATACODE_HW_PB_TS_GDB,
        NULL, sizeof(HWGRP_TS_GDB), onTsGdb,
        {TRUE,FALSE,IIC_CTOPT_BUS}, CNTP_PB_TS_GDB, NULL};
#endif // cfg_iic
#endif // _hw_dc

// valid data structures allowed (same order as defined CNTP_*
static const UBYTE ubBlkAllowed[][CNTP_PB_SIZE]=
{
    {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},        // generic power board w/o assembly ident
    {1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},        // generic power board with assembly ident
    {0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0},        // MBP
    {0,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0},        // MPB+PSI
    {0,0,1,0,0,0,1,1,3,0,0,0,0,0,0,0,0},        // PSI+FTB+GDB
#ifdef _HW_DC
    {0,0,1,0,0,0,0,0,3,1,1,0,0,0,0,0,0},        // PSI+1x(BRI+3xGDB) AxN-DC400
    {0,0,1,0,0,0,0,0,6,1,2,0,0,0,0,0,0},        // PSI+2x(BRI+3xGDB) AxN-DC800
    {0,0,1,0,0,0,0,0,1,1,1,0,0,0,0,0,0},        // PSI+2x(BRI+3xGDB) AxN-DC800 (with single BRI and single GDB idchip)
    {0,0,0,0,0,0,0,0,1,0,0,1,1,1,0,0,0},        // PSI+1xBRI+1xGDB   EV-400
    {0,0,0,0,0,0,0,0,2,0,0,1,1,2,0,0,0},        // PSI+2xBRI+2xGDB   EV-800
    {0,0,0,0,0,0,0,0,3,0,0,1,1,3,0,0,0},        // PSI+3xBRI+3xGDB   EV-1200
    {0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1},        // PSB+BRK+SCR       PSU
#ifdef _APP_DEBUG
    {0,0,1,0,0,0,0,0,1,1,1,0,0,0,0,0,0},        // PSI+1xBRI, debug only, not valid for production
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},        // PSI+1xBRI, debug only, not valid for production
    {0,0,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0},        // PSI+1x(BRI+3xGDB)
#endif
#endif
};

    // Selector for _HW_DC support for DS_PSI and DS_MBP 
#ifndef _HW_DC
#define _DS_CURRRANGE           sGlbPowerBoardParameters.sUniv.sCurrRange
#define _DS_TEMPSENS            sGlbPowerBoardParameters.sUniv.sTempSens
#else
#define _DS_CURRRANGE           sGlbPowerBoardParameters.sUniv.sBrCurrRange[0]
#define _DS_TEMPSENS            sGlbPowerBoardParameters.sUniv.sTempSens[0]
#endif

//***************************************************************************
// Universal controlboard configuration structures handling

#ifdef _HW_DC

#define CNTP_CB_CT_CB               0   // CesTello+ControlBoard (internal bus)
#define CNTP_CB_CT_EM               1   // CesTello+EncoderMain (external bus)
#define CNTP_CB_CT_EA               2   // CesTello+EncoderAux (external bus)
#define CNTP_CB_CT_IO               3   // CesTello+IO (external bus)
#define CNTP_CB_CT_BP               4   // CesTello+BackPlane (external bus)
#define CNTP_CB_CB_EM               5   // ControlBoard+EncoderMain (EV) (internal bus)
#define CNTP_CB_CB_BP               6   // ControlBoard+BackPlane (EV) (internal bus)
#define CNTP_CB_CB_PSU              7   // ControlBoard+PSU (AxN-PSU) (external bus)
#define CNTP_CB_CB_IO               8   // ControlBoard+IO (internal bus)

#define CNTP_CB_SIZE                (CNTP_CB_CB_IO+1)//(CNTP_CB_CB_PSU+1)

static const HWDATA_LIST sHwUCBConfDataDefs[]=
{
#ifndef _HW_AXS

    {DATACODE_HW_CT_CB,
        NULL, sizeof(HWGRP_CT_CB), onCtCb,
        {TRUE,FALSE,OW_INTERNAL_BUS,TRUE}, CNTP_CB_CT_CB, NULL}, // CesTello+ControlBoard (internal bus)

#if CFG_IIC
	{DATACODE_HW_CT_CB,
		NULL, sizeof(HWGRP_CT_CB), onCtCb,
		{TRUE,FALSE,IIC_INTERNAL_BUS,TRUE}, CNTP_CB_CT_CB, NULL}, // IIC
#endif

    {DATACODE_HW_CT_EM,
        NULL, sizeof(HWGRP_CT_EM), onCtEm,
        {TRUE,FALSE,OW_CTOPT_BUS}, CNTP_CB_CT_EM, NULL},        // CesTello+EncoderMain (CesTello OPTional bus)

#if CFG_IIC
	{DATACODE_HW_CT_EM,
		NULL, sizeof(HWGRP_CT_EM), onCtEm,
		{TRUE,FALSE,IIC_CTOPT_BUS}, CNTP_CB_CT_EM, NULL},         // IIC
#endif

    {DATACODE_HW_CT_EM,
        NULL, sizeof(HWGRP_CT_EM), onCtEm,
        {TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_CB_EM, NULL},     // ControlBoard+EncoderMain (EV) (internal bus)

#if CFG_IIC
	{DATACODE_HW_CT_EM,
		NULL, sizeof(HWGRP_CT_EM), onCtEm,
		{TRUE,FALSE,IIC_INTERNAL_BUS}, CNTP_CB_CB_EM, NULL},      // IIC
#endif

    {DATACODE_HW_CT_EA,
        NULL, sizeof(HWGRP_CT_EA), onCtEa,
        {TRUE,FALSE,OW_CTOPT_BUS}, CNTP_CB_CT_EA, NULL},        // CesTello+EncoderAux (CesTello OPTional bus)

#if CFG_IIC
	{DATACODE_HW_CT_EA,
		NULL, sizeof(HWGRP_CT_EA), onCtEa,
		{TRUE,FALSE,IIC_CTOPT_BUS}, CNTP_CB_CT_EA, NULL},         // IIC
#endif

    {DATACODE_HW_CT_IO,
        NULL, sizeof(HWGRP_CT_IO), onCtIo,
        {TRUE,FALSE,OW_CTOPT_BUS}, CNTP_CB_CT_IO, NULL},        // CesTello+IO (CesTello OPTional bus)

#if CFG_IIC
	{DATACODE_HW_CT_IO,
		NULL, sizeof(HWGRP_CT_IO), onCtIo,
		{TRUE,FALSE,IIC_CTOPT_BUS}, CNTP_CB_CT_IO, NULL},         // IIC
#endif

    {DATACODE_HW_CT_BP,
        NULL, sizeof(HWGRP_CT_BP), onCtBp,
        {TRUE,FALSE,OW_CTOPT_BUS}, CNTP_CB_CT_BP, NULL},        // CesTello+BackPlane (CesTello OPTional bus)

#if CFG_IIC
	{DATACODE_HW_CT_BP,
		NULL, sizeof(HWGRP_CT_BP), onCtBp,
		{TRUE,FALSE,IIC_CTOPT_BUS}, CNTP_CB_CT_BP, NULL},         // IIC
#endif

    {DATACODE_HW_CT_BP,
        NULL, sizeof(HWGRP_CT_BP), onCtBp,
        {TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_CB_BP, NULL},     // ControlBoard+BackPlane (EV) (internal bus)

#if CFG_IIC
	{DATACODE_HW_CT_BP,
		NULL, sizeof(HWGRP_CT_BP), onCtBp,
		{TRUE,FALSE,IIC_INTERNAL_BUS}, CNTP_CB_CB_BP, NULL},      // IIC
#endif

    {DATACODE_HW_CT_PSU,
        NULL, sizeof(HWGRP_CT_PSU), onCtPsu,
        {TRUE,FALSE,OW_CTOPT_BUS}, CNTP_CB_CB_PSU, NULL},       // ControlBoard+PSU (AxN-PSU) (CesTello OPTional bus)

#if CFG_IIC
	{DATACODE_HW_CT_PSU,
		NULL, sizeof(HWGRP_CT_PSU), onCtPsu,
		{TRUE,FALSE,IIC_CTOPT_BUS}, CNTP_CB_CB_PSU, NULL},        // IIC
#endif

	{DATACODE_HW_CT_IO,
		NULL, sizeof(HWGRP_CT_IO), onCtIo,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_CB_IO, NULL},     // CesTello+IO (internal bus)
		
#else // _HW_AXS

    {DATACODE_HW_CT_CB,
        NULL, sizeof(HWGRP_CT_CB), onCtCb,
        {TRUE,FALSE,OW_INTERNAL_BUS,TRUE}, CNTP_CB_CT_CB, NULL},

	{DATACODE_HW_CT_BP,
		NULL, sizeof(HWGRP_CT_BP), onCtBp,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_CT_BP, NULL},

    {DATACODE_HW_CT_EM,
        NULL, sizeof(HWGRP_CT_EM), onCtEm,
        {TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_CT_EM, NULL},

	{DATACODE_HW_CT_IO,
		NULL, sizeof(HWGRP_CT_IO), onCtIo,
		{TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_CB_CT_IO, NULL},

#endif // _HW_AXS

    {DATACODE_SYSLOG_INVALID,   // only for global erase
        &sGlbControlBoardParameters.sUniv, sizeof(sGlbControlBoardParameters.sUniv), NULL,
        {TRUE,FALSE,OW_INTERNAL_BUS}, CNTP_DISABLED, NULL},

#ifndef _HW_AXS
#if CFG_IIC
	{DATACODE_SYSLOG_INVALID,   // only for global erase
		&sGlbControlBoardParameters.sUniv, sizeof(sGlbControlBoardParameters.sUniv), NULL,
		{TRUE,FALSE,IIC_INTERNAL_BUS}, CNTP_DISABLED, NULL},        // IIC
#endif
#endif // _HW_AXS
};

#define HWUCBDATALISTCOUNT  (sizeof(sHwUCBConfDataDefs)/sizeof(HWDATA_LIST))

/*
                          EV  EV
      CB  EM  EA  IO  BP  EM  BP  PSU 
    {  1,  0,  0,  0,  1,  0,  0,   0},   // CB(i) + BP(e)
    {  1,  1,  0,  0,  1,  0,  0,   0},   // CB(i) + BP(e) + EM(e)
    {  1,  1,  1,  0,  1,  0,  0,   0},   // CB(i) + BP(e) + EM(e) + EA(e)
    {  1,  1,  0,  1,  1,  0,  0,   0},   // CB(i) + BP(e) + EM(e) + IO(e)
    {  1,  1,  1,  1,  1,  0,  0,   0},   // CB(i) + BP(e) + EM(e) + EA(e) + IO(e)
    {  1,  0,  0,  0,  0,  1,  1,   0},   // CB(i) + BP(i) + EM(i) (EV)
    {  1,  0,  0,  0,  1,  0,  0,   1},   // CB(i) + BP(e) + PSU(e)
    {  1,  1,  0,  0,  1,  0,  0,   1},   // CB(i) + BP(e) + EM(e) + PSU(e)
    {  1,  1,  0,  1,  1,  0,  0,   1},   // CB(i) + BP(e) + EM(e) + IO(e) + PSU(e)

*/
    // valid data structures allowed (same order as defined CNTP_*)
static const UBYTE ubUCBBlkAllowed[][CNTP_CB_SIZE]=
{
#ifndef _HW_AXS
	    {1,0,0,0,1,0,0,0,0},                    // CB + BP
	    {1,1,0,0,1,0,0,0,0},                    // CB + BP + EM
	    {1,1,1,0,1,0,0,0,0},                    // CB + BP + EM + EA
	    {1,1,0,1,1,0,0,0,0},                    // CB + BP + EM + IO
	    {1,1,1,1,1,0,0,0,0},                    // CB + BP + EM + EA + IO
	    {1,0,0,0,0,1,1,0,0},                    // CB + BP + EM (EV)
	    {1,0,0,0,1,0,0,1,0},                    // CB + BP + PSU
	    {1,1,0,0,1,0,0,1,0},                    // CB + BP + EM + PSU
	    {1,1,0,1,1,0,0,1,0},                    // CB + BP + EM + IO + PSU
	    {1,0,0,0,0,1,1,0,1},                    // CB + BP + EM + IO (EV)
#else // _HW_AXS
		{1,1,0,1,1,0,0,0,0},                    // CB + BP + EM + IO
#endif

	#ifdef _APP_DEBUG
	    {1,0,0,0,0,0,0,0,0},                    // CB
	#endif
};
#endif

//***************************************************************************
// Erase all configuration data structures
//#define TEMP(x)    ulSystemWarnings = (1<<x)
static void erasealldest(const HWDATA_LIST * pDatlst, UWORD uwDatLstSz)
{
    SWORD ct;

    for(ct=0; ct<uwDatLstSz; ct++)
    {
        if(pDatlst[ct].hpvData)
            memset(pDatlst[ct].hpvData, 0, pDatlst[ct].uwDataSize);
        if(pDatlst[ct].puwDestDataCode)
            *pDatlst[ct].puwDestDataCode=DATACODE_SYSLOG_INVALID;
    }
}
            
//***************************************************************************
// Retrieve data

static BOOL retrieveblock(const HWDATA_LIST * pDat, SWORD blkcode, HPVOID pfound, UWORD * blkrev)
{
    HPVOID pdata;
    SWORD retval;

        // if callback just execute it
    if(pDat->pfCallBack)
    {
            // retrieve data pointer
        retval=blkstor_getaddr(pfound,0,blkcode,&pdata);

            // if error skip
        if(retval<0)
            return FALSE;

            // execute callback
        if(!(*pDat->pfCallBack)(pDat, pdata, retval, blkrev))
            return FALSE;
    }
    else
    {
            // retrieve data
        retval=blkstor_getdata(pfound,0,blkcode,pDat->hpvData,(SWORD)pDat->uwDataSize);

            // if error, re-erase area and skip
        if(retval<0)
        {
            memset(pDat->hpvData, 0, pDat->uwDataSize);
            return FALSE;
        }

            // not managed here
        if(blkrev)
            *blkrev=0;
    }

        // retrieved block code
    if(pDat->puwDestDataCode)
        *pDat->puwDestDataCode=blkcode;

        // and increment block counter
    if(pDat->sbCnt!=CNTP_DISABLED)
        ubBlkFoundCnt[pDat->sbCnt]++;

    return TRUE;
}

//***************************************************************************
// OW single bus scan

static void owbusscan(SWORD swBus, void (* yield)(void), const HWDATA_LIST * pDatlst, UWORD uwDatLstSz)
{
    HPVOID storageptr;
    HPVOID pfound;
    ULONG storagesize;
    SWORD blkcode,ct,ctbd;
    SWORD swFound = 0, swRslt, swI, swCnt, swLength;
    UWORD brev;
    OW_SEARCH_STATE owstate ;
    UBYTE ROM_NO[ 8 ][ OW_MAX_BUS_DEVICES ] ;

#ifdef _DEBUG_TRACES
	xil_printf("HardwareConfig::owbusscan(%d, 0x%08lx, %d)\r\n", swBus, pDatlst, uwDatLstSz);
#endif

        // find all devices
    swFound = 0;
    swRslt = OWSearchFirst( swBus, &owstate, yield ) ;
    while ( swRslt )
    {
        for ( swI = 0; swI < 8; swI++ )
            ROM_NO[ swI ][ swFound ] = owstate.ROM_NO[ swI ] ;

        swRslt = OWSearchNext( swBus, &owstate, yield ) ;
        swFound++ ;
    }

    for ( swCnt = 0; swCnt < swFound; swCnt++ )
    {
        UBYTE ubBuffer[ max( OW_DS2431_DATA_LENGTH, OW_DS2433_DATA_LENGTH ) ], ubRomno[ 8 ] ;

            // copy ROM code
        for ( swI = 0; swI < 8; swI++ )
            ubRomno[ swI ] = ROM_NO[ swI ][ swCnt ];

            // erase buffer for fast block search
        memset(ubBuffer, 0, sizeof(ubBuffer));

        if ( ROM_NO[ 0 ][ swCnt ] == OW_DS2431_FAMILY_CODE )
            swLength = OW_DS2431_DATA_LENGTH;
        else if ( ROM_NO[ 0 ][ swCnt ] == OW_DS2433_FAMILY_CODE )
            swLength = OW_DS2433_DATA_LENGTH;
        else
            swLength = 0;

        if ( swLength > 0 )
        {       // read memory in local buffer
            if ( OWReadMemory( swBus, ubRomno, ubBuffer, swLength, yield ) )
            {
                storageptr=ubBuffer;
                storagesize=sizeof(ubBuffer);

                for(;;)
                {   // analyze each valid block between boundaries
                    blkcode=blkstor_enumvalid(&storageptr,&storagesize,&pfound);

                        // if not found skip
                    if(blkcode==BLKSTOR_ERR_NOTFOUND)
                        break;

                        // if error skip too
                    if(blkcode<0)
                        break;

                                // seek found block in the global structure
                    for(ct=0; ct<uwDatLstSz; ct++)
                    {
                        if(pDatlst[ct].uwDataCode!=DATACODE_SYSLOG_INVALID && pDatlst[ct].uwDataCode==blkcode)
                        {
                                // found, valid only if bus match
                            if(pDatlst[ct].src.ubOWValidBus!=swBus)
                                continue;

                                // get block
                            if(!retrieveblock(&pDatlst[ct], blkcode, pfound, &brev))
                                brev=-1;

                                // if valid takes note of data structure for
                                // diagnostic purpose, if not already there
                            for(ctbd=0;ctbd<uwHwCfgBlocksSz;ctbd++)
                            {
                                for(swI=0;swI<8;swI++)
                                {
                                    if(tHwCfgBlocks[ctbd].ubSN[swI]!=ROM_NO[swI][swCnt])
                                        break;
                                }

                                if(swI<8)
                                    continue;

                                if(tHwCfgBlocks[ctbd].ubBus==(UBYTE)swBus && tHwCfgBlocks[ctbd].uwDataCode==blkcode)
                                    break;
                            }

                            if(ctbd>=uwHwCfgBlocksSz && uwHwCfgBlocksSz<HWCONF_MAXCFGBLOCKS)
                            {
                                for(swI=0;swI<8;swI++)
                                    tHwCfgBlocks[uwHwCfgBlocksSz].ubSN[swI]=ROM_NO[swI][swCnt];

                                tHwCfgBlocks[uwHwCfgBlocksSz].ubBus=(UBYTE)swBus;
                                tHwCfgBlocks[uwHwCfgBlocksSz].uwDataCode=blkcode;
                                tHwCfgBlocks[uwHwCfgBlocksSz].uwProductRev=brev;

                                if(pDatlst[ct].src.ubUID)
                                {
                                    for(swI=0;swI<8;swI++)
                                        ubUniqueID[swI]=ROM_NO[swI][swCnt];
                                }
                                uwHwCfgBlocksSz++;
                            }

                            break;
                        }
                    }
                }
            }
        }
    }
}

#if CFG_IIC
static void iicbusscan( UBYTE ubBus, void (* yield)(void), const HWDATA_LIST * pDatlst, UWORD uwDatLstSz )
{
   HPVOID  storageptr;
   HPVOID  pfound;
   ULONG   storagesize;
   SWORD   blkcode, lentgh, ct, ctbd;
   UWORD   brev;
   UBYTE   ubiicdevicecnt = 0, iicdeviceaddr = 0xA0, ubI, ubCnt;

   IIC_SEARCH_STATE iicstate, device[IIC_MAX_BUS_DEVICES];
   /* Get all the IIC device on the same bus */
   for( ubI = 0; ubI < IIC_MAX_BUS_DEVICES; ubI++ )
   {
      iicdeviceaddr = iicdeviceaddr | ((ubI << 2) & 0x0C);
      if( IIC_devicesearch(ubBus, &iicstate, iicdeviceaddr) )
      {
    	device[ubiicdevicecnt].DeviceAddress = iicstate.DeviceAddress;
    	device[ubiicdevicecnt].IICBusType = iicstate.IICBusType;
         ubiicdevicecnt++;
      }

      iicdeviceaddr = 0xA0;
   }

   for ( ubCnt = 0; ubCnt < ubiicdevicecnt; ubCnt++ )
   {
      unsigned char ucBuffer[ 512 ];
      lentgh = 512;

      memset(ucBuffer, 0, sizeof(ucBuffer));

      if ( IIC_ReadMemory( ubBus, &device[ubCnt].DeviceAddress, ucBuffer, 0, lentgh ) )
      {
         storageptr = ucBuffer;
         storagesize = sizeof(ucBuffer);

         for(;;)
         {
            // analyze each valid block between boundaries
            blkcode = blkstor_enumvalid(&storageptr, &storagesize, &pfound);

            // if not found skip
            if( BLKSTOR_ERR_NOTFOUND == blkcode )
               break;

            // if error skip too
            if( blkcode < 0 )
               break;

            // seek found block in the global structure
            for(ct=0; ct < uwDatLstSz; ct++)
            {
               if(pDatlst[ct].uwDataCode!=DATACODE_SYSLOG_INVALID && pDatlst[ct].uwDataCode==blkcode)
               {
                  // found, valid only if bus match
                  if(pDatlst[ct].src.ubOWValidBus != ubBus)
                     continue;

                  // get block
                  if(!retrieveblock(&pDatlst[ct], blkcode, pfound, &brev))
                     brev=-1;

                  // if valid takes note of data structure for
                  // diagnostic purpose, if not already there

                  for(ctbd = 0; ctbd < uwHwCfgBlocksSz; ctbd++)
                  {
                     if(tHwCfgBlocks[ctbd].ubSN[0] == device[ubCnt].DeviceAddress && tHwCfgBlocks[ctbd].ubBus == ubBus && tHwCfgBlocks[ctbd].uwDataCode==blkcode && tHwCfgBlocks[ctbd].uwProductRev == brev)
                        break;
                  }

                  if((ctbd >= uwHwCfgBlocksSz) && (uwHwCfgBlocksSz < HWCONF_MAXCFGBLOCKS))
                  {
                     tHwCfgBlocks[uwHwCfgBlocksSz].ubSN[0] = device[ubCnt].DeviceAddress;
                     tHwCfgBlocks[uwHwCfgBlocksSz].ubSN[1] = device[ubCnt].IICBusType;

                     tHwCfgBlocks[uwHwCfgBlocksSz].ubBus = ubBus;
                     tHwCfgBlocks[uwHwCfgBlocksSz].uwDataCode = blkcode;
                     tHwCfgBlocks[uwHwCfgBlocksSz].uwProductRev = brev;

                     if(pDatlst[ct].src.ubUID)
                        ubUniqueID[0] = device[ubCnt].DeviceAddress;

                     uwHwCfgBlocksSz++;
                  }

                  break;
               }
         	 }
           }
      }
   }
}
#endif // cfg_iic
//***************************************************************************
// Get product revision from IDChip
UWORD PlcHwProductRev(UWORD uwDataCode)
{
    UWORD blkn;

    for(blkn=0;blkn<uwHwCfgBlocksSz;blkn++)
    {
        if(tHwCfgBlocks[blkn].uwDataCode==uwDataCode)
            return tHwCfgBlocks[blkn].uwProductRev;
    }

    return 0;
}
    
//***************************************************************************
// Configuration validater

static BOOL configvalidater(void)
{
    SWORD i,j;

        // compare each allowed blocks combinations
    for(i=0;i<sizeof(ubBlkAllowed)/CNTP_PB_SIZE;i++)
    {
        for(j=0;j<CNTP_PB_SIZE;j++)
        {
            if(ubBlkAllowed[i][j]!=ubBlkFoundCnt[j])
                break;
        }

            // all counters match, valid configuration
        if(j>=CNTP_PB_SIZE)
            return TRUE;
    }

    return FALSE;
}

//***************************************************************************
// Controlboard configuration validater

#ifdef _HW_DC
static BOOL cbconfigvalidater(void)
{
    SWORD i,j;

        // check array size
    assert(CNTP_CB_SIZE<=CNTP_PB_SIZE);

        // compare each allowed blocks combinations
    for(i=0;i<sizeof(ubUCBBlkAllowed)/CNTP_CB_SIZE;i++)
    {
        for(j=0;j<CNTP_CB_SIZE;j++)
        {
            if(ubUCBBlkAllowed[i][j]!=ubBlkFoundCnt[j])
                break;
        }

            // all counters match, next step is verifying
            // unit ID support
        if(j>=CNTP_CB_SIZE)
        {
            if((sGlbControlBoardParameters.sUniv.sUnit.ulIDMask&HWUNITID_ALLSUPPORTED)==sGlbControlBoardParameters.sUniv.sUnit.ulIDMask)
                return TRUE;
            else
                return FALSE;
        }
    }

    return FALSE;
}
#endif

//***************************************************************************
// Retrieve configuration from available sources

static BOOL retrieveconfiguration(void (* yield)(void))
{
    HPVOID storageptr;
    HPVOID pfound;
    ULONG storagesize;
    SWORD blkcode,ct,ctblk;
    SWORD swScn;
    BOOL bValid;

#ifdef _DEBUG_TRACES
	xil_printf("HardwareConfig::retrieveconfiguration()\r\n");
#endif

        // scan up to X times the mandatory buses to reduce error probability
        // if valid configuration found then stop early

        // scan internal bus for mandatory controlboard configuration
    for(swScn=0, bValid=FALSE; swScn<RESCAN_OW_TIMES && !bValid; swScn++)
    {
#ifndef _HW_DC
            // erase all configuration data structures
        erasealldest(sHwConfDataDefs,HWDATALISTCOUNT);

            // scan internal bus only
        owbusscan(OW_INTERNAL_BUS, yield, sHwIntConfDataDefs, HWINTDATALISTCOUNT);

            // exit when found
        if(sGlbControlBoardParameters.sProductInfo.uwProductCode!=0)
        {
            bValid=TRUE;
            break;
        }
#else
            // erase all universal cb data structures
        erasealldest(sHwUCBConfDataDefs,HWUCBDATALISTCOUNT);

            // scan internal and accessory bus
        owbusscan(OW_INTERNAL_BUS, yield, sHwUCBConfDataDefs, HWUCBDATALISTCOUNT);
#ifndef _HW_AXS
        owbusscan(OW_CTOPT_BUS   , yield, sHwUCBConfDataDefs, HWUCBDATALISTCOUNT);
#if CFG_IIC
        iicbusscan(IIC_INTERNAL_BUS, yield, sHwUCBConfDataDefs, HWUCBDATALISTCOUNT);
        iicbusscan(IIC_CTOPT_BUS, yield, sHwUCBConfDataDefs, HWUCBDATALISTCOUNT);
#endif
#endif

        // now verify if valid configuration is found
        bValid=cbconfigvalidater();
#endif
    }

        // if no valid controlboard configuration then exit immediately
    if(!bValid)
        return FALSE;

    for(swScn=0, bValid=FALSE; swScn<RESCAN_OW_TIMES && !bValid; swScn++)
    {
            // erase all configuration data structures
        erasealldest(sHwConfDataDefs,HWDATALISTCOUNT);

            // erase block counters
        for(ct=0; ct<CNTP_PB_SIZE; ct++)
            ubBlkFoundCnt[ct]=0;

#ifndef _HW_AXS
            // scan internal bus then external one
        owbusscan(OW_INTERNAL_BUS, yield, sHwConfDataDefs, HWDATALISTCOUNT);
        owbusscan(OW_EXTERNAL_BUS, yield, sHwConfDataDefs, HWDATALISTCOUNT);
#ifdef _HW_DC
#if CFG_IIC
        // iicscan internal bus then external one for IIC
		iicbusscan(IIC_INTERNAL_BUS, yield, sHwConfDataDefs, HWDATALISTCOUNT);
		iicbusscan(IIC_EXTERNAL_BUS, yield, sHwConfDataDefs, HWDATALISTCOUNT);
#endif
#endif

#else // _HW_AXS

#if defined(_HW_AXS_SAIETTA)
        // scan internal bus
        owbusscan(OW_INTERNAL_BUS, yield, sHwConfDataDefs, HWDATALISTCOUNT);
#elif defined(_HW_AXS_CABI35KW)
        // scan external bus
        owbusscan(/*OW_EXTERNAL_BUS*/OW_INTERNAL_BUS, yield, sHwConfDataDefs, HWDATALISTCOUNT);
#elif defined(_HW_AXS_DAYCO22KW)
        // scan external bus
        owbusscan(OW_EXTERNAL_BUS, yield, sHwConfDataDefs, HWDATALISTCOUNT);
#endif

#endif // _HW_AXS
            // now verify if valid configuration is found
        bValid=configvalidater();
    }

        // if no valid configuration yet found re-erase
        // all configuration data structures
    if(!bValid)
        erasealldest(sHwConfDataDefs,HWDATALISTCOUNT);

#ifndef _HW_DC
        // if no valid configuration or no assembly ident found
    if(!bValid || ubBlkFoundCnt[CNTP_ASSEMBLY_IDENT]==0 && ubBlkFoundCnt[CNTP_PB_ASSEMBLY]==0)
    {
            // seek first in bootblock data list
        for(ctblk=0; ctblk<HWCONFBLOCKCOUNT; ctblk++)
        {
                // get block boundaries
            storageptr=sHwConfBlockDefs[ctblk].hpvStart;
            storagesize=sHwConfBlockDefs[ctblk].ulSize;

            for(;;)
            {
                    // analyze each valid block between boundaries
                blkcode=blkstor_enumvalid(&storageptr,&storagesize,&pfound);

                    // if error or not found skip entire block
                if(blkcode<0)
                    break;

                    // seek found block in the global structure, allowed in flash
                for(ct=0; ct<HWDATALISTCOUNT; ct++)
                    if(sHwConfDataDefs[ct].uwDataCode!=DATACODE_SYSLOG_INVALID && sHwConfDataDefs[ct].uwDataCode==blkcode &&
                        !sHwConfDataDefs[ct].src.ubOWOnly && (sHwConfDataDefs[ct].src.ub2ndValid || !bValid))
                    {
                            // get block
                        retrieveblock(&sHwConfDataDefs[ct], blkcode, pfound, NULL);
                        break;
                    }

                    // update power on joke
                if(yield)
                    (*yield)();
            }
        }
    }

        // scan X times the optional bus to reduce error probability
    for(swScn=0; swScn<RESCAN_OW_TIMES; swScn++)
        owbusscan(OW_IOEXP_BUS, yield, sHwConfDataDefs, HWDATALISTCOUNT);
#endif

    return TRUE;
}


#ifdef _HW_DC
//***************************************************************************
// Retrieve configuration for pwb
static BOOL retrievepwbconfiguration(void)
{
    SWORD swScn;
    BOOL bValid=FALSE;
//    tempbit |=1;
    for(swScn=0, bValid=FALSE; swScn<RESCAN_OW_TIMES && !bValid; swScn++)
    {
#if CFG_IIC
        if(ubBlkFoundCnt[CNTP_PB_ASSEMBLY]==0)
            iicbusscan(IIC_CTOPT_BUS, NULL, &sPbAsmConfDataDefs, 1);
        else
            sGlbPowerBoardParameters.sUniv.sProductInfo.uwProductCode=HWPRM_PCODE_POWER_BOARD_UNIVERSAL;

        if(ubBlkFoundCnt[CNTP_PB_XS_PSI]==0)
            iicbusscan(IIC_CTOPT_BUS, NULL, &sPbXsPsiConfDataDefs, 1);

        if(ubBlkFoundCnt[CNTP_PB_XS_BRI]==0)
            iicbusscan(IIC_CTOPT_BUS, NULL, &sPbXsBriConfDataDefs, 1);

        if(ubBlkFoundCnt[CNTP_PB_TS_GDB]==0)
            iicbusscan(IIC_CTOPT_BUS, NULL, &sPbTsGdbConfDataDefs, 1);
#else
        sGlbPowerBoardParameters.sUniv.sProductInfo.uwProductCode=HWPRM_PCODE_POWER_BOARD_UNIVERSAL;
#endif // cfg_iic
            // now verify if valid configuration is found
        bValid=configvalidater();
    }

    return bValid;
}
#endif

//***************************************************************************
// Check for known and managed hardware
static BOOL checkknownhardware(void)
{
//	tempbit |=(1<<2);
        // check control board and revisions
    switch(sGlbControlBoardParameters.sProductInfo.uwProductCode)
    {
#ifndef _APP_XC
        case HWPRM_PCODE_CONTROL_BOARD_AXM_II_XE167:
//            if(XFAMILY_GETID()!=XFAMILYIDENT_XE167x)
//                return FALSE;
//            if((sGlbControlBoardParameters.sProductInfo.uwProductRev<100 || sGlbControlBoardParameters.sProductInfo.uwProductRev>103) &&
//                (sGlbControlBoardParameters.sProductInfo.uwProductRev/100)!=3   &&
//                (sGlbControlBoardParameters.sProductInfo.uwProductRev/100)!=103 &&
//                (sGlbControlBoardParameters.sProductInfo.uwProductRev/100)!=4 )
//                return FALSE;
            break;
#endif

#ifndef _HW_DC
#ifndef _HW_CT
        case HWPRM_PCODE_CONTROL_BOARD_AXM_II_XC2788X:
#ifdef _INFINEON_
            if(XFAMILY_GETID()!=XFAMILYIDENT_XC2788X)
                return FALSE;

            if((sGlbControlBoardParameters.sProductInfo.uwProductRev/100)!=5 &&
               (sGlbControlBoardParameters.sProductInfo.uwProductRev/100)!=6)
                return FALSE;
#else
            if(XFAMILY_GETID()!=XFAMILYIDENT_ZYNQ)
                return FALSE;

            sGlbControlBoardParameters.sProductInfo.uwProductCode = HWPRM_PCODE_CONTROL_BOARD_AXM_II_ZYNQ;
#endif // _infineon_
            break;
#else
        case HWPRM_PCODE_CONTROL_BOARD_AXM_II_CTUNV:
            if(XFAMILY_GETID()!=XFAMILYIDENT_XC2788X)
                return FALSE;

            sGlbControlBoardParameters.sProductInfo.uwProductCode = HWPRM_PCODE_CONTROL_BOARD_AXM_II_ZYNQ;
            break;
#endif // _hw_ct
#else
        case HWPRM_PCODE_CONTROL_BOARD_AXM_II_UNIV:
            if(XFAMILY_GETID()!=XFAMILYIDENT_ZYNQ)
                return FALSE;

            sGlbControlBoardParameters.sProductInfo.uwProductCode = HWPRM_PCODE_CONTROL_BOARD_AXM_II_ZYNQ;
            break;
#endif // _hw_dc
        case HWPRM_PCODE_CONTROL_BOARD_AXM_II_ZYNQ:
            if(XFAMILY_GETID()!=XFAMILYIDENT_ZYNQ)
            	return FALSE;
            break;
        default:
            return FALSE;
    }

        // if control board is ok, then setup default FPGA app and type
#ifdef _INFINEON_
#ifndef _HW_DC
    tHwFpgaConfig.uwType=sGlbControlBoardParameters.sStd.uwFpgaType;

        // FPGA type
    if((tHwFpgaConfig.uwType>>12)==1)
    {
        if((tHwFpgaConfig.uwType&0xFF)==0xA7)
            strcpy(tHwFpgaConfig.chApp,"I");
        else
            strcpy(tHwFpgaConfig.chApp,"Q");
    }
        // patched rev 3.xx 10C8 controlboard clocking
    else if((sGlbControlBoardParameters.sProductInfo.uwProductRev/100)==103)
        strcpy(tHwFpgaConfig.chApp,"T");
    else
        strcpy(tHwFpgaConfig.chApp,"S");

#else
    tHwFpgaConfig.uwType=sGlbControlBoardParameters.sUniv.uwFpgaType;
    strcpy(tHwFpgaConfig.chApp,"V");
#endif // _hw_dc
#else
#ifndef _HW_DC
    tHwFpgaConfig.uwType=sGlbControlBoardParameters.sStd.uwFpgaType;
#else
    tHwFpgaConfig.uwType=sGlbControlBoardParameters.sUniv.uwFpgaType;
#endif // _hw_dc

        // FPGA type
    if((tHwFpgaConfig.uwType>>14)==0)
        strcpy(tHwFpgaConfig.chApp,"Z");
#endif // _infineon
    tHwFpgaConfig.uwAppCode=IDENT_FPGA_STANDARD;

        // check power board
    switch(sGlbPowerBoardParameters.sProductInfo.uwProductCode)
    {
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
        case HWPRM_PCODE_POWER_BOARD_DIF_CI:
        case HWPRM_PCODE_POWER_BOARD_DIF_PI:
        case HWPRM_PCODE_POWER_BOARD_DIF_BC:
#endif
        case HWPRM_PCODE_POWER_BOARD_UNIVERSAL:
            break;

        default:
#ifdef _APP_DEBUG
            sGlbPowerBoardParameters.sProductInfo.uwProductCode=HWPRM_PCODE_POWER_BOARD_UNIVERSAL;
            return TRUE;
#else
            return FALSE;
#endif
    }

#ifndef _HW_DC
        // check IO expansion board version, if installed
    if(sGlbIOExpBoardParameters.sProductInfo.uwProductCode!=HWPRM_PCODE_BOARD_UNKNOWN)
    {
            // unsupported on controlboard rev < 2.xx
        if(sGlbControlBoardParameters.sProductInfo.uwProductRev<200)
            return FALSE;

            // if found something unknown
        if(sGlbIOExpBoardParameters.sProductInfo.uwProductCode!=HWPRM_PCODE_IO_EXP_BOARD)
            return FALSE;

            // hw version check, supported only 1.xx
        if((sGlbIOExpBoardParameters.sProductInfo.uwProductRev/100)!=1)
            return FALSE;

            // validated, fill-up info data
        if((sGlbIOExpBoardParameters.uwFpgaType>>12)==1)
            strcpy(tHwIOExpBoardConfig.chApp,"Q");
        else
            strcpy(tHwIOExpBoardConfig.chApp,"S");
        tHwIOExpBoardConfig.uwType=sGlbIOExpBoardParameters.uwFpgaType;
    }
#endif

    return TRUE;
}

//***************************************************************************
// Init

BOOL HwConfInit(void)
{
        // add bg task for parameters checking
    if(!TaskSched_AddBackgroundTask((void (*)(void))&parameterscheck))
        return FALSE;

        // check parameters validity
    if(!parameterscheck())
        return FALSE;

        // valid
    return TRUE;
}

//***************************************************************************
// slow task

static BOOL parameterscheck(void)
{
    BOOL bValid;

        // check main operation mode
    switch(sHwConfParams.ubSysMode)
    {
        case HWCONF_SYSMODE_COMP:
#ifdef _APP_XC
        case HWCONF_SYSMODE_PERF:
#endif
            bValid=TRUE;
            break;

        default:
            bValid=FALSE;
            break;
    }

        // signal or reset error
    if(bValid)
        ParChk_ResetValueError(PARCC_HWCONF_CORESPEED);
    else
        ParChk_SignalValueError(PARCC_HWCONF_CORESPEED);

    return bValid;
}

//***************************************************************************
// Add one block data with correlated header and update pointer to next
// location and remaining storage size

#ifndef _HW_DC
static BOOL _addblock(SWORD swCode, HPVOID hpvSrc, UWORD uwSize, HPVOID * ppvDest, ULONG * pulDestSize)
{
    BLKSTOR_HEADER sHeader;

        // fillup header info
    if(blkstor_createheader(swCode, hpvSrc, uwSize, &sHeader)<0)
        return TRUE;

        // write header
    if(FlashMgrWriteData(*ppvDest, &sHeader, sizeof(sHeader)) != FLASHMGR_R_OK)
        return TRUE;

        // calculate data address
    *ppvDest=&((UBYTE *)*ppvDest)[sizeof(sHeader)];

        // write data
    if(FlashMgrWriteData(*ppvDest, hpvSrc, uwSize) != FLASHMGR_R_OK)
        return TRUE;

        // update pointer
    *ppvDest=&((UBYTE *)*ppvDest)[uwSize];

        // check for no oversize
	assert(*pulDestSize > (ULONG)sizeof(sHeader)+uwSize);

        // update remaining size
    *pulDestSize-=(ULONG)sizeof(sHeader)+uwSize;

	return FALSE;
}
#endif

//***************************************************************************
// Get configuration and check

BOOL HwConfGetAndCheck(void (* yield)(void))
{
        // reset FPGA derived config data
    memset(&tHwFpgaConfig,0,sizeof(tHwFpgaConfig));

        // retrieve configuration
    if(!retrieveconfiguration(yield))
    	return FALSE;

#ifdef _HW_DC
    if(!checkknownhardware())
    {
        if(!retrievepwbconfiguration())
            return FALSE;
    }
    else
        return TRUE;
#endif
        // check hardware
    return checkknownhardware();
}

//***************************************************************************
// Store hardware configuration in local flash

#ifndef _HW_DC
BOOL HwConfStore(void)
{
    HPVOID storageptr;
    ULONG storagesize;
	UWORD ct;
    BOOL retval;

//	system_wdt_clear();     //$TEST$
//	syslog_logtimer(FALSE); //$TEST$

        // lock flash manager
    if(FlashMgrBegin(WAIT_FLASHMGR_LOCK)==FLASHMGR_R_LOCKFAILED)
        return FALSE;

        // default return value
    retval=FALSE;

        // erase all flash blocks
    for(ct=0; ct<HWCONFBLOCKCOUNT; ct++)
        if(ProgramFlashErase(sHwConfBlockDefs[ct].hpvStart, sHwConfBlockDefs[ct].ulSize))
            goto exit_on_error;

        // block boundaries, take just the first
    storageptr=sHwConfBlockDefs[0].hpvStart;
    storagesize=sHwConfBlockDefs[0].ulSize;

        // add assembly informations
	if(_addblock(DATACODE_HW_ASSEMBLY_IDENT, &sGlbAssemblyInfo, sizeof(sGlbAssemblyInfo), &storageptr, &storagesize))
            goto exit_on_error;

        // no more data to be added
    retval=TRUE;

exit_on_error:
        // unlock flash manager
    FlashMgrEnd(NULL);

    return retval;
}
#endif

//***************************************************************************
// Enable/disable PSRAM write protection

//BOOL HwConfPSRAMWrProt(BOOL bEnable)
//{
//    if(bEnable)
//    {
//        IMB_IMBCTRH = (UWORD)(((ULONG)((HPVOID)&RAMMOD_FCODE_ENDBEFORE)-(ULONG)((HPVOID)&RAMMOD_FCODE_BEGIN))/0x1000l)<<8;
//        return TRUE;
//    }
//        // disable permitted only during bootup
//    else if(bSysStatBooting)
//    {
//        IMB_IMBCTRH = 0x0000;
//        return TRUE;
//    }
//    else
//        return FALSE;
//}

//***************************************************************************
// At startup request user-setup system mode

#ifdef _APP_XC
void HwConfBoot(BOOL bValid)
{
#ifdef _INFINEON_
        // if valid user flash parameters and requested performance mode
    if(bValid && sHwConfParams.ubSysMode==HWCONF_SYSMODE_PERF)
        SysRes_ExecuteReset(RESETCODE_CLEANREBOOT_PERF, ~RESETCODE_CLEANREBOOT_PERF);
    else
#endif
        SysRes_ExecuteReset(RESETCODE_CLEANREBOOT_COMP, ~RESETCODE_CLEANREBOOT_COMP);
}
#endif

//***************************************************************************
// Check global options requested by configuration

BOOL HwConfGlobalOptions(void)
{
#ifdef _HW_DC
    sGlbOptAvailable.ulHardwareOpt  = sGlbControlBoardParameters.sUniv.sUnit.ulIDMask;
    sGlbOptAvailable.ulHardwareOpt |= sGlbPowerBoardParameters.sUniv.sUnit.ulIDMask  ;

    if((sGlbOptAvailable.ulHardwareOpt & sGlbOptReq.ulHardwareOpt) != sGlbOptReq.ulHardwareOpt)
        return FALSE;

#endif
    return TRUE;
}

#define USE_ZYNQ_CHIPID

//***************************************************************************
// Get unique ID as 32bit

#ifdef _HW_DC
ULONG HwGetUID32(void)
{
    ULONG id;
#ifndef USE_ZYNQ_CHIPID
    UWORD c;

    for(c=0,id=0l;c<4;c++,id<<=8)
        id|=ubUniqueID[c]^ubUniqueID[4+c];
#else
    id = MAKELONG(FPGA_CHIPID_LL, FPGA_CHIPID_LH);
    id^= MAKELONG(FPGA_CHIPID_HL, FPGA_CHIPID_HH);
#endif

    return id;
}

//***************************************************************************
// Get unique ID as 24bit

void HwGetUID24(HPUBYTE hpubID)
{

#ifndef USE_ZYNQ_CHIPID
    UWORD c;

    for(c=0;c<3;c++)
        hpubID[c]=ubUniqueID[c]^ubUniqueID[3+c];
#else
    UWORD d;

    d = FPGA_CHIPID_LL;
    hpubID[0]=HIBYTE(d)^LOBYTE(d);
    d = FPGA_CHIPID_LH;
    hpubID[1]=HIBYTE(d)^LOBYTE(d);
    d = FPGA_CHIPID_HL;
    hpubID[2]=HIBYTE(d)^LOBYTE(d);
#endif
}
#endif

//***************************************************************************
// convert old assembly into new

#ifndef _HW_DC
static BOOL convertV19assembly(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
    ASSEMBLY_V19_INFO * hpSrc=(ASSEMBLY_V19_INFO *)hpvSrc;
    ASSEMBLY_INFO * hpDst=(ASSEMBLY_INFO *)psDat->hpvData;

    if(uwSrcSz>sizeof(ASSEMBLY_V19_INFO))
        return FALSE;

    memset(psDat->hpvData, 0, psDat->uwDataSize);

    hpDst->ulSerialNumber=hpSrc->ulSerialNumber;
    memcpy(hpDst->chDescrText, hpSrc->chDescrText, sizeof(hpSrc->chDescrText));
    memcpy(hpDst->chDateText,  hpSrc->chDateText,  sizeof(hpSrc->chDateText));

    if(puwRev)
        *puwRev=0;

    return TRUE;
}

//***************************************************************************
//

static BOOL onGenericV19(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    memcpy(psDat->hpvData, hpvSrc, uwSrcSz);

    if(puwRev)
        *puwRev=((HWPRMS_PRODUCT_INFO *)hpvSrc)->uwProductRev;

    return TRUE;
}

//***************************************************************************
//

static BOOL onGenericPwr(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
    // if already found adopt same strategy of previous system, take first valid and skip subsequent ones
    if(ubBlkFoundCnt[psDat->sbCnt]>0)
        return FALSE;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    memcpy(psDat->hpvData, hpvSrc, uwSrcSz);

    if(puwRev)
        *puwRev=((HWPRMS_PRODUCT_INFO *)hpvSrc)->uwProductRev;

    return TRUE;
}
#endif

//***************************************************************************
//

static BOOL onAssembly(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onAssembly(%d)\r\n", DATACODE_HW_PB_ASSEMBLY);
#endif

    HWGRP_ASSEMBLY * hpSrc=(HWGRP_ASSEMBLY *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sOutBrdgCfg.ubReserved || hpSrc->sInBrdgCfg.ubReserved || hpSrc->sCoolFeat.sOpt.ubReserved)
        return FALSE;

    memcpy(&sGlbAssemblyInfo,                          &hpSrc->sInfo,       sizeof(ASSEMBLY_INFO           ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sOutBrdgCfg,&hpSrc->sOutBrdgCfg, sizeof(HWPRM2_OUT_BRIDGE_LAYOUT));
    memcpy(&sGlbPowerBoardParameters.sUniv.sInBrdgCfg ,&hpSrc->sInBrdgCfg , sizeof(HWPRM2_IN_AC_LAYOUT     ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sBrkRes    ,&hpSrc->sBrkRes    , sizeof(HWPRM2_BRAKE_RESISTOR   ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sCoolFeat  ,&hpSrc->sCoolFeat  , sizeof(HWPRM2_COOLING_FEATURES ));

    if(ubBlkFoundCnt[CNTP_PB_MBP]>0 || ubBlkFoundCnt[CNTP_PB_DS_MBP]>0 || ubBlkFoundCnt[CNTP_PB_TS_FTB]>0
#ifdef _HW_DC
        || ubBlkFoundCnt[CNTP_PB_XS_PSI]>0
#endif
    )
    {
//        if(!onMergeDCCharge(&sGlbPowerBoardParameters.sUniv.sDCChrg, &hpSrc->sDCChrg))
//            return FALSE;
    	sGlbPowerBoardParameters.sUniv.sDCChrg.fCapacity = sGlbPowerBoardParameters.sUniv.sDCChrg.fCapacity + hpSrc->sDCChrg.fCapacity;
//    	sGlbPowerBoardParameters.sUniv.sDCChrg.fResistor =  sGlbPowerBoardParameters.sUniv.sDCChrg.fResistor;
    }
    else
        memcpy(&sGlbPowerBoardParameters.sUniv.sDCChrg    ,&hpSrc->sDCChrg    , sizeof(HWPRM2_DC_CHARGE        ));

#ifndef _HW_DC
    if(ubBlkFoundCnt[CNTP_PB_MBP]>0 || ubBlkFoundCnt[CNTP_PB_DS_MBP]>0 || ubBlkFoundCnt[CNTP_PB_TS_PSI]>0)
    {
        if(!onMergeCurrentRange(&sGlbPowerBoardParameters.sUniv.sCurrRange, &hpSrc->sCurrRange))
            return FALSE;
    }
    else
        memcpy(&sGlbPowerBoardParameters.sUniv.sCurrRange ,&hpSrc->sCurrRange , sizeof(HWPRM2_CURRENT_RANGE    ));
#else
    sGlbPowerBoardParameters.sUniv.fRange  = hpSrc->sCurrRange.fRange;
#endif

    // setup product code, revision is not used
    sGlbPowerBoardParamDataCode=DATACODE_SYSLOG_INVALID;
    sGlbPowerBoardParameters.sUniv.sProductInfo.uwProductCode=HWPRM_PCODE_POWER_BOARD_UNIVERSAL;

    if(puwRev)
        *puwRev=0;

    return TRUE;
}

//***************************************************************************
//

#ifndef _HW_DC
static BOOL onMbp(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
    HWGRP_MBP * hpSrc=(HWGRP_MBP *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sHwFeat.ubReserved || hpSrc->sCurrCfg.sOpt.ubReserved || hpSrc->sCurrCfg.ubReserved ||
        hpSrc->sVoltCfg.sOpt.ubReserved || hpSrc->sVoltCfg.ubReserved || hpSrc->sBrkFeat.ubReserved ||
        hpSrc->sBrdgFeat.sOpt.ubReserved || hpSrc->sBrdgFeat.ubReserved || hpSrc->sHwFeat.sOpt.ubReserved ||
        hpSrc->sBrkFeat.sOpt.ubReserved)
        return FALSE;

    memcpy(&sGlbPowerBoardParameters.sUniv.sHwFeat    ,&hpSrc->sHwFeat    , sizeof(HWPRM2_HW_FEATURES      ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sTempSens  ,&hpSrc->sTempSens  , sizeof(HWPRMS_AD_SETTINGS      ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sCurrCfg   ,&hpSrc->sCurrCfg   , sizeof(HWPRM2_CURRENT_LAYOUT   ));

    if(ubBlkFoundCnt[CNTP_PB_ASSEMBLY]>0)
    {
        if(!onMergeCurrentRange(&hpSrc->sCurrRange, &sGlbPowerBoardParameters.sUniv.sCurrRange))
            return FALSE;
    }
    else
        memcpy(&sGlbPowerBoardParameters.sUniv.sCurrRange ,&hpSrc->sCurrRange , sizeof(HWPRM2_CURRENT_RANGE    ));

    memcpy(&sGlbPowerBoardParameters.sUniv.sVoltCfg   ,&hpSrc->sVoltCfg   , sizeof(HWPRM2_VOLTAGE_LAYOUT   ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sVoltRange ,&hpSrc->sVoltRange , sizeof(HWPRM2_VOLTAGE_RANGE    ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sBrkFeat   ,&hpSrc->sBrkFeat   , sizeof(HWPRM2_BRAKE_FEATURES   ));

    if(ubBlkFoundCnt[CNTP_PB_ASSEMBLY]>0)
    {
        if(!onMergeDCCharge(&hpSrc->sDCChrg, &sGlbPowerBoardParameters.sUniv.sDCChrg))
            return FALSE;
    }
    else
        memcpy(&sGlbPowerBoardParameters.sUniv.sDCChrg    ,&hpSrc->sDCChrg    , sizeof(HWPRM2_DC_CHARGE        ));

    memcpy(&sGlbPowerBoardParameters.sUniv.sBrdgFeat  ,&hpSrc->sBrdgFeat  , sizeof(HWPRM2_BRIDGE           ));

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}
#endif

//***************************************************************************
//

static BOOL onDsMbp(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onDsMbp(%d)\r\n", DATACODE_HW_PB_DS_MBP);
#endif

    HWGRP_DS_MBP * hpSrc=(HWGRP_DS_MBP *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sCurrCfg.sOpt.ubReserved || hpSrc->sCurrCfg.ubReserved || hpSrc->sBrkFeat.sOpt.ubReserved ||
        hpSrc->sVoltCfg.sOpt.ubReserved || hpSrc->sVoltCfg.ubReserved ||
        hpSrc->sBrkFeat.ubReserved || hpSrc->sBrdgFeat.sOpt.ubReserved || hpSrc->sBrdgFeat.ubReserved)
        return FALSE;

    if(ubBlkFoundCnt[CNTP_PB_DS_PSI]>0)
    {
        if(!onMergeCurrentLayout(&hpSrc->sCurrCfg, &sGlbPowerBoardParameters.sUniv.sCurrCfg))
            return FALSE;
        if(!onMergeTempSens(&hpSrc->sTempSens, &_DS_TEMPSENS))
            return FALSE;
        if(!onMergeVoltageLayout(&sGlbPowerBoardParameters.sUniv.sVoltCfg, &hpSrc->sVoltCfg))
            return FALSE;
    }
    else
    {
        memcpy(&sGlbPowerBoardParameters.sUniv.sCurrCfg   ,&hpSrc->sCurrCfg   , sizeof(HWPRM2_CURRENT_LAYOUT   ));
        memcpy(&sGlbPowerBoardParameters.sUniv.sTempSens  ,&hpSrc->sTempSens  , sizeof(HWPRMS_AD_SETTINGS      ));
        memcpy(&sGlbPowerBoardParameters.sUniv.sVoltCfg   ,&hpSrc->sVoltCfg   , sizeof(HWPRM2_VOLTAGE_LAYOUT   ));
    }

#ifndef _HW_DC
    if(ubBlkFoundCnt[CNTP_PB_ASSEMBLY]>0)
    {
        if(!onMergeCurrentRange(&hpSrc->sCurrRange, &_DS_CURRRANGE))
            return FALSE;
    }
    else
#endif
        memcpy(&_DS_CURRRANGE ,&hpSrc->sCurrRange , sizeof(HWPRM2_CURRENT_RANGE    ));

    memcpy(&sGlbPowerBoardParameters.sUniv.sVoltRange ,&hpSrc->sVoltRange , sizeof(HWPRM2_VOLTAGE_RANGE    ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sBrkFeat   ,&hpSrc->sBrkFeat   , sizeof(HWPRM2_BRAKE_FEATURES   ));

    if(ubBlkFoundCnt[CNTP_PB_ASSEMBLY]>0)
    {
//        if(!onMergeDCCharge(&hpSrc->sDCChrg, &sGlbPowerBoardParameters.sUniv.sDCChrg))
//            return FALSE;
    	sGlbPowerBoardParameters.sUniv.sDCChrg.fCapacity = hpSrc->sDCChrg.fCapacity + sGlbPowerBoardParameters.sUniv.sDCChrg.fCapacity;
    	sGlbPowerBoardParameters.sUniv.sDCChrg.fResistor = hpSrc->sDCChrg.fResistor;
    }
    else
        memcpy(&sGlbPowerBoardParameters.sUniv.sDCChrg    ,&hpSrc->sDCChrg    , sizeof(HWPRM2_DC_CHARGE        ));

    memcpy(&sGlbPowerBoardParameters.sUniv.sBrdgFeat  ,&hpSrc->sBrdgFeat  , sizeof(HWPRM2_BRIDGE           ));

#ifdef _HW_DC
    sGlbPowerBoardParameters.sUniv.sUnit.ulIDMask |= HWUNITID_BRI_FULL_0;
#endif

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}

//***************************************************************************
//

static BOOL onDsPsi(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onDsPsi(%d)\r\n", DATACODE_HW_PB_DS_PSI);
#endif

    HWGRP_DS_PSI * hpSrc=(HWGRP_DS_PSI *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sHwFeat.ubReserved || hpSrc->sCurrCfg.sOpt.ubReserved || hpSrc->sCurrCfg.ubReserved ||
        hpSrc->sVoltCfg.sOpt.ubReserved || hpSrc->sVoltCfg.ubReserved || hpSrc->sHwFeat.sOpt.ubReserved)
        return FALSE;

    memcpy(&sGlbPowerBoardParameters.sUniv.sHwFeat    ,&hpSrc->sHwFeat    , sizeof(HWPRM2_HW_FEATURES      ));

    if(ubBlkFoundCnt[CNTP_PB_DS_MBP]>0)
    {
        if(!onMergeCurrentLayout(&sGlbPowerBoardParameters.sUniv.sCurrCfg, &hpSrc->sCurrCfg))
            return FALSE;
        if(!onMergeTempSens(&_DS_TEMPSENS, &hpSrc->sTempSens))
            return FALSE;
        if(!onMergeVoltageLayout(&hpSrc->sVoltCfg, &sGlbPowerBoardParameters.sUniv.sVoltCfg))
            return FALSE;
    }
    else
    {
        memcpy(&sGlbPowerBoardParameters.sUniv.sCurrCfg   ,&hpSrc->sCurrCfg   , sizeof(HWPRM2_CURRENT_LAYOUT   ));
        memcpy(&sGlbPowerBoardParameters.sUniv.sTempSens  ,&hpSrc->sTempSens  , sizeof(HWPRMS_AD_SETTINGS      ));
        memcpy(&sGlbPowerBoardParameters.sUniv.sVoltCfg   ,&hpSrc->sVoltCfg   , sizeof(HWPRM2_VOLTAGE_LAYOUT   ));
    }

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}

//***************************************************************************
//

#ifndef _HW_DC
static BOOL onTsPsi(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
    HWGRP_TS_PSI * hpSrc=(HWGRP_TS_PSI *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sHwFeat.ubReserved || hpSrc->sCurrCfg.sOpt.ubReserved || hpSrc->sCurrCfg.ubReserved ||
        hpSrc->sVoltCfg.sOpt.ubReserved || hpSrc->sVoltCfg.ubReserved || hpSrc->sHwFeat.sOpt.ubReserved)
        return FALSE;

    memcpy(&sGlbPowerBoardParameters.sUniv.sHwFeat    ,&hpSrc->sHwFeat    , sizeof(HWPRM2_HW_FEATURES      ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sTempSens  ,&hpSrc->sTempSens  , sizeof(HWPRMS_AD_SETTINGS      ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sCurrCfg   ,&hpSrc->sCurrCfg   , sizeof(HWPRM2_CURRENT_LAYOUT   ));

    if(ubBlkFoundCnt[CNTP_PB_ASSEMBLY]>0)
    {
        if(!onMergeCurrentRange(&hpSrc->sCurrRange, &sGlbPowerBoardParameters.sUniv.sCurrRange))
            return FALSE;
    }
    else
        memcpy(&sGlbPowerBoardParameters.sUniv.sCurrRange ,&hpSrc->sCurrRange , sizeof(HWPRM2_CURRENT_RANGE    ));

    if(ubBlkFoundCnt[CNTP_PB_TS_FTB]>0)
    {
        if(!onMergeVoltageLayout(&hpSrc->sVoltCfg, &sGlbPowerBoardParameters.sUniv.sVoltCfg))
            return FALSE;
    }
    else
        memcpy(&sGlbPowerBoardParameters.sUniv.sVoltCfg   ,&hpSrc->sVoltCfg   , sizeof(HWPRM2_VOLTAGE_LAYOUT   ));

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}

//***************************************************************************
//

static BOOL onTsFtb(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
    HWGRP_TS_FTB * hpSrc=(HWGRP_TS_FTB *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sVoltCfg.sOpt.ubReserved || hpSrc->sVoltCfg.ubReserved || hpSrc->sBrkFeat.ubReserved ||
        hpSrc->sBrkFeat.sOpt.ubReserved)
        return FALSE;

    if(ubBlkFoundCnt[CNTP_PB_TS_PSI]>0)
    {
        if(!onMergeVoltageLayout(&sGlbPowerBoardParameters.sUniv.sVoltCfg, &hpSrc->sVoltCfg))
            return FALSE;
    }
    else
        memcpy(&sGlbPowerBoardParameters.sUniv.sVoltCfg   ,&hpSrc->sVoltCfg   , sizeof(HWPRM2_VOLTAGE_LAYOUT   ));

    memcpy(&sGlbPowerBoardParameters.sUniv.sVoltRange ,&hpSrc->sVoltRange , sizeof(HWPRM2_VOLTAGE_RANGE    ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sBrkFeat   ,&hpSrc->sBrkFeat   , sizeof(HWPRM2_BRAKE_FEATURES   ));

    if(ubBlkFoundCnt[CNTP_PB_ASSEMBLY]>0)
    {
        if(!onMergeDCCharge(&hpSrc->sDCChrg, &sGlbPowerBoardParameters.sUniv.sDCChrg))
            return FALSE;
    }
    else
        memcpy(&sGlbPowerBoardParameters.sUniv.sDCChrg    ,&hpSrc->sDCChrg    , sizeof(HWPRM2_DC_CHARGE        ));

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}
#endif

//***************************************************************************
//

static BOOL onTsGdb(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onTsGdb(%d)\r\n", DATACODE_HW_PB_TS_GDB);
#endif

    HWGRP_TS_GDB * hpSrc=(HWGRP_TS_GDB *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sBrdgFeat.sOpt.ubReserved || hpSrc->sBrdgFeat.ubReserved)
        return FALSE;

    if(ubBlkFoundCnt[CNTP_PB_TS_GDB]>0)
    {
        if(memcmp(&sGlbPowerBoardParameters.sUniv.sBrdgFeat, &hpSrc->sBrdgFeat, sizeof(HWPRM2_BRIDGE))!=0)
            return FALSE;
    }
    else
        memcpy(&sGlbPowerBoardParameters.sUniv.sBrdgFeat  ,&hpSrc->sBrdgFeat  , sizeof(HWPRM2_BRIDGE           ));

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}

//***************************************************************************
//

#ifdef _HW_DC
static BOOL onXsPsi(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onXsPsi(%d)\r\n", DATACODE_HW_PB_XS_PSI);
#endif

    HWGRP_XS_PSI * hpSrc=(HWGRP_XS_PSI *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sHwFeat.ubReserved || hpSrc->sHwFeat.sOpt.ubReserved || hpSrc->sVoltCfg.sOpt.ubReserved ||
       hpSrc->sVoltCfg.ubReserved || hpSrc->sBrkFeat.ubReserved || hpSrc->sBrkFeat.sOpt.ubReserved ||
       hpSrc->sCurrCfg.sOpt.ubReserved || hpSrc->sCurrCfg.ubReserved )
        return FALSE;

    memcpy(&sGlbPowerBoardParameters.sUniv.sHwFeat    ,&hpSrc->sHwFeat    , sizeof(HWPRM2_HW_FEATURES      ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sCurrCfg   ,&hpSrc->sCurrCfg   , sizeof(HWPRM2_CURRENT_LAYOUT   ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sVoltCfg   ,&hpSrc->sVoltCfg   , sizeof(HWPRM2_VOLTAGE_LAYOUT   ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sVoltRange ,&hpSrc->sVoltRange , sizeof(HWPRM2_VOLTAGE_RANGE    ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sBrkFeat   ,&hpSrc->sBrkFeat   , sizeof(HWPRM2_BRAKE_FEATURES   ));

    if(ubBlkFoundCnt[CNTP_PB_ASSEMBLY]>0)
    {
//        if(!onMergeDCCharge(&hpSrc->sDCChrg, &sGlbPowerBoardParameters.sUniv.sDCChrg))
//            return FALSE;
    	sGlbPowerBoardParameters.sUniv.sDCChrg.fCapacity = hpSrc->sDCChrg.fCapacity + sGlbPowerBoardParameters.sUniv.sDCChrg.fCapacity;
    	sGlbPowerBoardParameters.sUniv.sDCChrg.fResistor = hpSrc->sDCChrg.fResistor;
    }
    else
        memcpy(&sGlbPowerBoardParameters.sUniv.sDCChrg    ,&hpSrc->sDCChrg    , sizeof(HWPRM2_DC_CHARGE        ));

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}

//***************************************************************************
//

static BOOL onXsBri(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onXsBri(%d)\r\n", DATACODE_HW_PB_XS_BRI);
#endif

    HWGRP_XS_BRI * hpSrc=(HWGRP_XS_BRI *)hpvSrc;
    SWORD u=-1;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sUnit.ulIDMask&HWUNITID_BRI_FULL_0)
        u=0;
    else if(hpSrc->sUnit.ulIDMask&HWUNITID_BRI_FULL_1)
        u=1;
    else if(hpSrc->sUnit.ulIDMask&HWUNITID_BRI_FULL_2)
        u=2;
    else
        return FALSE;

    sGlbPowerBoardParameters.sUniv.sUnit.ulIDMask |= hpSrc->sUnit.ulIDMask;

    memcpy(&sGlbPowerBoardParameters.sUniv.sTempSens[u]    ,&hpSrc->sTempSens  , sizeof(HWPRMS_AD_SETTINGS      ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sBrCurrRange[u] ,&hpSrc->sCurrRange , sizeof(HWPRM2_CURRENT_RANGE    ));

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}

//***************************************************************************
//

static BOOL onPsuPsb(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onPsuPsb(%d)\r\n", DATACODE_HW_PB_PSU_PSB);
#endif

    HWGRP_PSU_PSB * hpSrc=(HWGRP_PSU_PSB *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sHwFeat.ubReserved || hpSrc->sHwFeat.sOpt.ubReserved )
        return FALSE;

    memcpy(&sGlbPowerBoardParameters.sUniv.sHwFeat    ,&hpSrc->sHwFeat    , sizeof(HWPRM2_HW_FEATURES      ));

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}

//***************************************************************************
//

static BOOL onPsuBrk(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onPsuBrk(%d)\r\n", DATACODE_HW_PB_PSU_BRK);
#endif

    HWGRP_PSU_BRK * hpSrc=(HWGRP_PSU_BRK *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sBrkFeat.ubReserved || hpSrc->sBrkFeat.sOpt.ubReserved)
        return FALSE;

    memcpy(&sGlbPowerBoardParameters.sUniv.sTempSens[0],&hpSrc->sTempSens  , sizeof(HWPRMS_AD_SETTINGS      ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sBrkFeat    ,&hpSrc->sBrkFeat   , sizeof(HWPRM2_BRAKE_FEATURES   ));

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}

//***************************************************************************
//

static BOOL onPsuScr(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onPsuScr(%d)\r\n", DATACODE_HW_PB_PSU_SCR);
#endif

    HWGRP_PSU_SCR * hpSrc=(HWGRP_PSU_SCR *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sVoltCfg.sOpt.ubReserved || hpSrc->sVoltCfg.ubReserved  )
        return FALSE;

    memcpy(&sGlbPowerBoardParameters.sUniv.sTempSens[1],&hpSrc->sTempSens  , sizeof(HWPRMS_AD_SETTINGS      ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sVoltCfg    ,&hpSrc->sVoltCfg   , sizeof(HWPRM2_VOLTAGE_LAYOUT   ));
    memcpy(&sGlbPowerBoardParameters.sUniv.sVoltRange  ,&hpSrc->sVoltRange , sizeof(HWPRM2_VOLTAGE_RANGE    ));

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}

//***************************************************************************
//

static BOOL onCtCb(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onCtCb(%d)\r\n", DATACODE_HW_CT_CB);
#endif

    HWGRP_CT_CB * hpSrc=(HWGRP_CT_CB *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sBoardTemp.ubReserved || hpSrc->sBoardTemp.ubType!=HWPRM2_ANTYPE_STD_BOARDTEMP)
        return FALSE;

    sGlbControlBoardParameters.sUniv.sUnit.ulIDMask |= hpSrc->sUnit.ulIDMask;

    sGlbControlBoardParameters.sUniv.uwFpgaType = hpSrc->uwFpgaType;
    memcpy(&sGlbControlBoardParameters.sUniv.sBoardTemp  ,&hpSrc->sBoardTemp  , sizeof(HWPRM2_ANALOG_TYPE      ));

    // setup product code, revision is not used
    sGlbControlBoardParameters.sUniv.sProductInfo.uwProductCode=HWPRM_PCODE_CONTROL_BOARD_AXM_II_UNIV;

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;
    
    return TRUE; 
}

//***************************************************************************
//

static BOOL onCtEm(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onCtEm(%d)\r\n", DATACODE_HW_CT_EM);
#endif

    HWGRP_CT_EM * hpSrc=(HWGRP_CT_EM *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sEMVfb.ubReserved || hpSrc->sEMVfb.ubType!=HWPRM2_ANTYPE_EM_VPWR)
        return FALSE;

    sGlbControlBoardParameters.sUniv.sUnit.ulIDMask |= hpSrc->sUnit.ulIDMask;

    memcpy(&sGlbControlBoardParameters.sUniv.sEMVfb       ,&hpSrc->sEMVfb       , sizeof(HWPRM2_ANALOG_TYPE));

    memcpy(&sGlbControlBoardParameters.sUniv.sKTYRange    ,&hpSrc->sKTYRange    , sizeof(HWPRM2_ANALOG_RANGE));
    memcpy(&sGlbControlBoardParameters.sUniv.sKTYCalibr   ,&hpSrc->sKTYCalibr   , sizeof(HWPRMS_AD_SETTINGS));
    memcpy(&sGlbControlBoardParameters.sUniv.sPTCRange    ,&hpSrc->sPTCRange    , sizeof(HWPRM2_ANALOG_RANGE));
    memcpy(&sGlbControlBoardParameters.sUniv.sPTCCalibr   ,&hpSrc->sPTCCalibr   , sizeof(HWPRMS_AD_SETTINGS));

    memcpy(&sGlbControlBoardParameters.sUniv.sIncEncSin   ,&hpSrc->sIncEncSin   , sizeof(HWPRMS_AD_SETTINGS));
    memcpy(&sGlbControlBoardParameters.sUniv.sIncEncCos   ,&hpSrc->sIncEncCos   , sizeof(HWPRMS_AD_SETTINGS));

    memcpy(&sGlbControlBoardParameters.sUniv.sAbsEncHISin ,&hpSrc->sAbsEncHISin , sizeof(HWPRMS_AD_SETTINGS));
    memcpy(&sGlbControlBoardParameters.sUniv.sAbsEncHICos ,&hpSrc->sAbsEncHICos , sizeof(HWPRMS_AD_SETTINGS));
    memcpy(&sGlbControlBoardParameters.sUniv.sAbsEncLOSin ,&hpSrc->sAbsEncLOSin , sizeof(HWPRMS_AD_SETTINGS));
    memcpy(&sGlbControlBoardParameters.sUniv.sAbsEncLOCos ,&hpSrc->sAbsEncLOCos , sizeof(HWPRMS_AD_SETTINGS));

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}

//***************************************************************************
//

static BOOL onCtEa(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onCtEa(%d)\r\n", DATACODE_HW_CT_EA);
#endif

    HWGRP_CT_EA * hpSrc=(HWGRP_CT_EA *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sEAVfb.ubReserved || hpSrc->sEAVfb.ubType!=HWPRM2_ANTYPE_EA_VPWR)
        return FALSE;

    sGlbControlBoardParameters.sUniv.sUnit.ulIDMask |= hpSrc->sUnit.ulIDMask;

    memcpy(&sGlbControlBoardParameters.sUniv.sEAVfb       ,&hpSrc->sEAVfb       , sizeof(HWPRM2_ANALOG_TYPE));

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;

    return TRUE;
}

//***************************************************************************
//

static BOOL onCtIo(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onCtIo(%d)\r\n", DATACODE_HW_CT_IO);
#endif

    HWGRP_CT_IO * hpSrc=(HWGRP_CT_IO *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sInRange.flLoSpan != -hpSrc->sInRange.flHiSpan)
        return FALSE;

    if(hpSrc->sOutRange.flLoSpan != 0.0 || hpSrc->sOutRange.flHiSpan <= 0.0)
        return FALSE;

    sGlbControlBoardParameters.sUniv.sUnit.ulIDMask |= hpSrc->sUnit.ulIDMask;

    memcpy(&sGlbControlBoardParameters.sUniv.sInRange   ,&hpSrc->sInRange   , sizeof(HWPRM2_ANALOG_RANGE));
    memcpy(&sGlbControlBoardParameters.sUniv.sUserRefI0 ,&hpSrc->sUserRefI0 , sizeof(HWPRMS_AD_SETTINGS ));
    memcpy(&sGlbControlBoardParameters.sUniv.sUserRefI1 ,&hpSrc->sUserRefI1 , sizeof(HWPRMS_AD_SETTINGS ));
    memcpy(&sGlbControlBoardParameters.sUniv.sUserRefI2 ,&hpSrc->sUserRefI2 , sizeof(HWPRMS_AD_SETTINGS ));
    memcpy(&sGlbControlBoardParameters.sUniv.sUserRefI3 ,&hpSrc->sUserRefI3 , sizeof(HWPRMS_AD_SETTINGS ));

    memcpy(&sGlbControlBoardParameters.sUniv.sOutRange  ,&hpSrc->sOutRange  , sizeof(HWPRM2_ANALOG_RANGE));
    memcpy(&sGlbControlBoardParameters.sUniv.sUserAnaO0 ,&hpSrc->sUserAnaO0 , sizeof(HWPRMS_DA_SETTINGS ));
    memcpy(&sGlbControlBoardParameters.sUniv.sUserAnaO1 ,&hpSrc->sUserAnaO1 , sizeof(HWPRMS_DA_SETTINGS ));

    if(hpSrc->uwProductRev !=100)
    {
    	ubIOboardType = 1;
    }

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;
    
    return TRUE; 
}

//***************************************************************************
//

static BOOL onCtBp(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onCtBp(%d)\r\n", DATACODE_HW_CT_BP);
#endif

    HWGRP_CT_BP * hpSrc=(HWGRP_CT_BP *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    if(hpSrc->sVPwrSense.ubReserved || hpSrc->sVPwrSense.ubType!=HWPRM2_ANTYPE_STD_VAUXSENSE)
        return FALSE;

    sGlbControlBoardParameters.sUniv.sUnit.ulIDMask |= hpSrc->sUnit.ulIDMask;

    memcpy(&sGlbControlBoardParameters.sUniv.sVPwrSense   ,&hpSrc->sVPwrSense   , sizeof(HWPRM2_ANALOG_TYPE));

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;
    
    return TRUE; 
}

//***************************************************************************
//

static BOOL onCtPsu(const HWDATA_LIST * psDat, HPVOID hpvSrc, UWORD uwSrcSz, UWORD * puwRev)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onCtPsu(%d)\r\n", DATACODE_HW_CT_PSU);
#endif

    HWGRP_CT_PSU * hpSrc=(HWGRP_CT_PSU *)hpvSrc;

    if(uwSrcSz>psDat->uwDataSize)
        return FALSE;

    sGlbControlBoardParameters.sUniv.sUnit.ulIDMask |= hpSrc->sUnit.ulIDMask;

    if(puwRev)
        *puwRev=hpSrc->uwProductRev;
    
    return TRUE; 
}
#endif

//***************************************************************************
//

static BOOL onMergeDCCharge(HWPRM2_DC_CHARGE * pMPB, HWPRM2_DC_CHARGE * pASM)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onMergeDCCharge()\r\n");
#endif

    HWPRM2_DC_CHARGE sLoc;

    memset(&sLoc, 0, sizeof(HWPRM2_DC_CHARGE));

    sLoc.fCapacity=pMPB->fCapacity;
    sLoc.fCapacity+=pASM->fCapacity;
    sLoc.fResistor=pMPB->fResistor;

    memcpy(&sGlbPowerBoardParameters.sUniv.sDCChrg, &sLoc, sizeof(HWPRM2_DC_CHARGE));

    return TRUE;
}

//***************************************************************************
//

#ifndef _HW_DC
static BOOL onMergeCurrentRange(HWPRM2_CURRENT_RANGE * pMPB, HWPRM2_CURRENT_RANGE * pASM)
{
    HWPRM2_CURRENT_RANGE sLoc;

    memset(&sLoc, 0, sizeof(HWPRM2_CURRENT_RANGE));

    mergeADsettings(&pASM->sCalPhaseU, &pMPB->sCalPhaseU, &sLoc.sCalPhaseU);
    mergeADsettings(&pASM->sCalPhaseV, &pMPB->sCalPhaseV, &sLoc.sCalPhaseV);
    mergeADsettings(&pASM->sCalPhaseW, &pMPB->sCalPhaseW, &sLoc.sCalPhaseW);
    sLoc.fRange=pMPB->fRange*pASM->fRange;

    memcpy(&_DS_CURRRANGE, &sLoc, sizeof(HWPRM2_CURRENT_RANGE));

    return TRUE;
}
#endif

//***************************************************************************
//

static BOOL onMergeCurrentLayout(HWPRM2_CURRENT_LAYOUT * pMPB, HWPRM2_CURRENT_LAYOUT * pPSI)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onMergeCurrentLayout()\r\n");
#endif

    HWPRM2_CURRENT_LAYOUT sLoc;

    memset(&sLoc, 0, sizeof(HWPRM2_CURRENT_LAYOUT));

    if(pPSI->sOpt.ubCurrentU==1)
        sLoc.sOpt.ubCurrentU=pMPB->sOpt.ubCurrentU;
    else if(pPSI->sOpt.ubCurrentU==2)
        sLoc.sOpt.ubCurrentU=pMPB->sOpt.ubCurrentV;
    else
        sLoc.sOpt.ubCurrentU=pMPB->sOpt.ubCurrentW;

    if(pPSI->sOpt.ubCurrentV==1)
        sLoc.sOpt.ubCurrentV=pMPB->sOpt.ubCurrentU;
    else if(pPSI->sOpt.ubCurrentV==2)
        sLoc.sOpt.ubCurrentV=pMPB->sOpt.ubCurrentV;
    else
        sLoc.sOpt.ubCurrentV=pMPB->sOpt.ubCurrentW;

    if(pPSI->sOpt.ubCurrentW==1)
        sLoc.sOpt.ubCurrentW=pMPB->sOpt.ubCurrentU;
    else if(pPSI->sOpt.ubCurrentW==2)
        sLoc.sOpt.ubCurrentW=pMPB->sOpt.ubCurrentV;
    else
        sLoc.sOpt.ubCurrentW=pMPB->sOpt.ubCurrentW;

    sLoc.sOpt.ubCSFeedback=pMPB->sOpt.ubCSFeedback && pPSI->sOpt.ubCSFeedback;
    sLoc.uwPDTime=pMPB->uwPDTime+pPSI->uwPDTime;

    memcpy(&sGlbPowerBoardParameters.sUniv.sCurrCfg, &sLoc, sizeof(HWPRM2_CURRENT_LAYOUT));

    return TRUE;
}

//***************************************************************************
//

static BOOL onMergeVoltageLayout(HWPRM2_VOLTAGE_LAYOUT * pPSI, HWPRM2_VOLTAGE_LAYOUT * pFTB)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onMergeVoltageLayout()\r\n");
#endif

    HWPRM2_VOLTAGE_LAYOUT sLoc;

    memset(&sLoc, 0, sizeof(HWPRM2_VOLTAGE_LAYOUT));

    sLoc.sOpt.ubVac=pFTB->sOpt.ubVac && pPSI->sOpt.ubVac;

    sLoc.sOpt.ubVoltageU=pFTB->sOpt.ubVoltageU;
    sLoc.sOpt.ubVoltageV=pFTB->sOpt.ubVoltageV;

        // only support for pass-through without scrambling
    if(pPSI->sOpt.ubVoltageU!=1 || pPSI->sOpt.ubVoltageV!=2)
        return FALSE;

    sLoc.sOpt.ubCSFeedback=pFTB->sOpt.ubCSFeedback && pPSI->sOpt.ubCSFeedback;
    sLoc.uwPDTime=pFTB->uwPDTime+pPSI->uwPDTime;

    memcpy(&sGlbPowerBoardParameters.sUniv.sVoltCfg, &sLoc, sizeof(HWPRM2_VOLTAGE_LAYOUT));

    return TRUE;
}

//***************************************************************************
//

static BOOL onMergeTempSens(HWPRMS_AD_SETTINGS * pMPB, HWPRMS_AD_SETTINGS * pPSI)
{
#ifdef _DEBUG_TRACES
	xil_printf("  onMergeTempSens()\r\n");
#endif

    HWPRMS_AD_SETTINGS sLoc;

    memset(&sLoc, 0, sizeof(HWPRMS_AD_SETTINGS));

    mergeADsettings(pMPB, pPSI, &sLoc);

    memcpy(&_DS_TEMPSENS, &sLoc, sizeof(HWPRMS_AD_SETTINGS));

    return TRUE;
}

//***************************************************************************
// sensing -> EXT -> MID -> ADC

static void mergeADsettings(HWPRMS_AD_SETTINGS * pExt, HWPRMS_AD_SETTINGS * pMid, HWPRMS_AD_SETTINGS * pDst)
{
#ifdef _DEBUG_TRACES
	xil_printf("  mergeADsettings()\r\n");
#endif

    pDst->uwOffst=(UWORD)((FLOAT)pExt->uwOffst-32768.0+((FLOAT)pMid->uwOffst-32768.0)/((FLOAT)pExt->swScale/8192.0)+32768.0);
    pDst->swScale=(SWORD)((FLOAT)pExt->swScale*(FLOAT)pMid->swScale/8192.0);
}
