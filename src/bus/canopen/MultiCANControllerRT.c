/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : MultiCANControllerRT.c                                     */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Driver for Infineon MultiCAN Controller, targeted for      */
/*               CanOpen                                                    */
/*                                                                          */
/****************************************************************************/
/* SYSTEM CONFIG:                                                           */
/* - Module clock 40MHz                                                     */
/* - Two can controller supported, each with one low priority rx queue and  */
/*   one high priority (hi priority has higher ILVL but do not save MAC)    */
/* - tx queue                                                               */
/* - inhibit time manager with automatic tx for elapsed timer               */
/* - interrupt pointers allocation:                                         */
/*      0: node 0 RX hi priority                                            */
/*      1: node 0 RX lo priority                                            */
/*      2: node 0 TX                                                        */
/*      3: node 0 RX very hi priority                                       */
/*      4: node 1 RX hi priority                                            */
/*      5: node 1 RX lo priority                                            */
/*      6: node 1 TX                                                        */
/*      7: node 1 RX very hi priority                                       */
/*      15: exceptions                                                      */
/****************************************************************************/


#ifndef _RD
#include "system\SysAppConfig.h"
#else
#include "RemDispAppConfig.h"
#endif
#if CFG_CANDRV_CMDMGR
#include "MultiCANController.h"
#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "drive\AxM-E-Defines.h"
#include <string.h>
//#include <intrins.h>

#include "MultiCANControllerDefs.h"

#pragma GCC optimize (2)
//***************************************************************************
// check and send period COBs

#ifdef CFG_CANDRV_PERIODICCOB
BOOL candrv_RT_periodiccobprocessing(void)
{
    UWORD ct;
    SYNCCOBSTAT * pt;

    for(ct=0,pt=sCanDrvSyncCobStat;ct<CANDRV_MAX_CANNODE;ct++,pt++)
        if(pt->uwRestart>0)
            if(--(pt->uwCounter)==0)
            {
                    // if disabled then disable sync sending
                if((uwCanDrvGlobalFlags[ct]&CANDRV_GF_RUN)==0)
                    pt->uwRestart = 0;

                else
                {
                        // enable tx (TXREQ, TXEN0)
                    //CanMO[MO_BASE_SYNC+ct].MOCTRH = 0x0300;
                	candrv_periodsendID(ct);       //执行发送动作
                        // reset counter
                    pt->uwCounter = pt->uwRestart;

                        // if installed then execute callback
                    if(pfCanDrvSyncCobCallBack)
                        (*pfCanDrvSyncCobCallBack)(ct);
                }
            }

    return TRUE;
}
#endif
#endif
