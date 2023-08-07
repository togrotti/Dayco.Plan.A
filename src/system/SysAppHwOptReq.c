/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppHwOptReq.c                                           */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Request internal modules for detecting required            */
/*               hardware configuration                                     */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "SysAppHwOptReq.h"
#include "SysAppGlobals.h"

#include "bus\ethercat\ECATCommandMgr.h"
#if CFG_CANDRV_CMDMGR
#include "bus\canopen\CanOpenCommandMgr.h"
#endif
#ifdef _HW_DC
#include "bus\ethpmc\EthPMCCommandMgr.h"
#include "drive\PSUManager.h"
#endif
#include "drive\EncoderManager.h"
#include "drive\MotorHandler.h"
#include "drive\ThermalModel.h"
#include "drive\UserIO.h"
#include "bus\modbus\ModBusCommandMgr.h"

//***************************************************************************
// Structures

    // entry points for configuration reading
typedef struct
{                       // param, hw opt, FPGA opt
    void            (* pfGetHwOpt)(UWORD, ULONG *, ULONG *);
    UWORD           uwParam;
} HWOPTREQ_LIST;

//****************************************************************************
// Task list to be queried for specific hardware configuration request
// based on user configuration

static const HWOPTREQ_LIST psEntryList[]=
{
#ifndef _APP_LIMITED
#if CFG_ECAT
   {&EcatCM_GetHwOpt, 0},
#endif // cfg_ecat

#ifdef _APP_XC
#if CFG_CANDRV_CMDMGR
   {&CanOpenCM_GetHwOpt, 0},
#endif // cfg_candrv_cmdmgr
#endif // _app_xc

#endif // ! _app_limited

#ifdef _HW_DC
#if CFG_ETHPMC
   {&EpmcCM_GetHwOpt, 0},
#endif

#if CFG_PSU_MNGR
   {&PSUManager_GetHwOpt, 0},
#endif // cfg_psu_mngr
#endif // _hw_dc

   {&Em_GetHwOpt, 0},

#ifdef _APP_XC
   {&Mh_GetHwOpt, 0},
   {&Tm_GetHwOpt, 0},
   {&UserIO_GetHwOpt, 0},
#endif
   {&ModbusCM_GetHwOpt,0},

        // Terminator
    {NULL, 0}
};

//***************************************************************************
// Configuration collector

void HwOptReqCollect(void)
{
    ULONG ulHwOpt,ulFPGAOpt;
    const HWOPTREQ_LIST  * psScan=psEntryList;

        // prepare destination data structure
    sGlbOptReq.ulHardwareOpt=sGlbOptReq.ulFPGAOpt=0ul;

        // scan all list
    while(psScan->pfGetHwOpt)
    {
        ulHwOpt=ulFPGAOpt=0ul;

            // callback for retrieving requested configuration
        (*psScan->pfGetHwOpt)(psScan->uwParam, &ulHwOpt, &ulFPGAOpt);

            // merge options
#ifdef _HW_DC
        sGlbOptReq.ulHardwareOpt|=ulHwOpt;
#endif
        sGlbOptReq.ulFPGAOpt    |=ulFPGAOpt;

            // select next one
        psScan++;
    }
}
