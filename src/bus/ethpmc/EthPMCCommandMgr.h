/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCCommandMgr.h                                         */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : EtherPMC specific command manager                          */
/*                                                                          */
/****************************************************************************/

#ifndef _ETHPMCCOMMANDMGR_H
#define _ETHPMCCOMMANDMGR_H

#if CFG_ETHPMC
//***************************************************************************
// define
#define EPMCCM_INIT_ALWAYS          8
#define EPMCCM_INIT_CORE            0
#define EPMCCM_INIT_SYNCEVENT       1

//***************************************************************************
// Data Structure

typedef struct
{
    UWORD uwSlaves; // number of connected slaves
    ULONG ulID[64]; // ident of slaves, order by node number
                    // e.g. index 0 is node 1, index 1 is node 2
} EPMC_NETCONNDEVICES;

typedef struct
{
    union
    {
        struct
        {
            UBYTE bEnableModule:1;   // module enable flag
            UBYTE bReserved:7; // keep zero
        } f;
        UWORD w;
    } flags;
    EPMC_NETCONNDEVICES dev;
} EPMC_PARAMS;

typedef struct
{
    UWORD uwControl;
    UWORD uwStatus;
} EPMC_INOUT;

typedef struct
{
    UWORD uwMissingPacket;
    UWORD uwCRCErrorCounter;
    ULONG ulLineDelay;
    union
    {
        struct
        {
            unsigned char bLink0;
            unsigned char bLink1;
        } b;
        unsigned char ubFlags[2];
    } sFlags;
} EPMC_DIAG;

//***************************************************************************
// Globals

extern EPMC_PARAMS sEpmcCM_Params;
extern const EPMC_PARAMS sEpmcCM_DefParams;

extern EPMC_INOUT sEpmcCMInOut;
extern EPMC_DIAG sEpmcCMDiag;

//***************************************************************************
// Global functions

BOOL EpmcCM_Init(UWORD uwOption);
void EpmcCM_GetHwOpt(UWORD T, ULONG * pulHwOpt, ULONG * pulFPGAOpt);
BOOL EpmcCM_MasterParams(HPUBYTE hpubBin, UWORD uwSize);
BOOL EpmcCM_SyncEvent(void);
#endif // cfg_ethpmc
#endif
