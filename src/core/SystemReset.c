/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SystemReset.c                                              */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Manage system reset and reset recovering;                  */
/*                                                                          */
/****************************************************************************/

#ifdef _AXX_SYSAPP
#include "system\SysAppGlobals.h"
#else
#include "system\BootBlockGlobals.h"
#endif
#include "common\CommonUtility.h"
#include "common\BlockStorage.h"
#include "system\SysAppDataCodes.h"
#include "common\FlashManager.h"
#include "common\ProgramFlashHandler.h"
#include "system\GlobalResetCodes.h"

#include "SystemReset.h"

#ifdef _AXX_SYSAPP
#ifndef _RD
#include "fpga\FpgaHandler.h"
#endif
#endif

#include "xil_io.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

//****************************************************************************
// Defines

#define PSS_RST_CTRL_REG    (XPS_SYS_CTRL_BASEADDR + 0x200)
#define PSS_RST_MASK        0x01
#define SLCR_UNLOCK_ADDR    (XPS_SYS_CTRL_BASEADDR + 0x008)
#define UNLOCK_KEY          0xDF0D

#define REBOOT_STATUS_REG   (XPS_SYS_CTRL_BASEADDR + 0x258)
#define POR_MASK            0x00400000
#define STATE_MASK          0x0F000000

#define XDCFG_MULTIBOOT_REG (XPS_SYS_CTRL_BASEADDR + 0x702C) // MULTIBOOT_REGISTER
#define XDCFG_UNLOCK_REG    (XPS_SYS_CTRL_BASEADDR + 0x7034) // XDCFG_UNLOCK_REGISTER
#define UNLOCK_XDCFG_KEY    0x757BDF0D                // UNLOCK KEY

//****************************************************************************
// Globals

//#pragma NOINIT

volatile UWORD uwSysResParam0;
volatile UWORD uwSysResParam1;

//***************************************************************************
// Data structure

typedef struct
{
    HPVOID hpvStart;
    ULONG ulSize;
} BLOCKDEFS;

//***************************************************************************
// Storage Blocks definition

static const BLOCKDEFS sResParamBlockDefs[]=
{
    {(HPVOID)RESPARAM_BLK0_START, (ULONG)RESPARAM_BLK0_SIZE},
    {(HPVOID)RESPARAM_BLK1_START, (ULONG)RESPARAM_BLK1_SIZE},
};

#define RESPARAMBLOCKCOUNT     (sizeof(sResParamBlockDefs)/sizeof(BLOCKDEFS))

static UWORD resparam_nextblk;

//****************************************************************************
// Locals

static BOOL restore_resparam(void);
static BOOL save_resparam(void);

//****************************************************************************
// Check the last reset source and execute right task
// this is called before CINIT in order to keep RAM as is

void SysRes_Recovery(void)
{
        // save parameters
#ifdef _INFINEON_
    uwSysResParam0=SCU_MKMEM0;
    uwSysResParam1=SCU_MKMEM1;
#else
    FlashMgrInit();

    restore_resparam();
#endif

    if(uwSysResParam0!=SYSRES_POWERON_VALUE_PAR0 || uwSysResParam1!=SYSRES_POWERON_VALUE_PAR1)
        resetsystemrecovery(uwSysResParam0, uwSysResParam1);
    else
        resetpoweronreset();
}

//****************************************************************************
// Execute system reset, pass parameters across reset to SysRes_Recovery

void SysRes_ExecuteReset(UWORD uwPar0, UWORD uwPar1)
{
#ifdef _DEBUG_TRACES
    xil_printf("SystemReset::SysRes_ExecuteReset(0x%02X, 0x%02X)\r\n", uwPar0, uwPar1);
#endif


#ifdef _AXX_SYSAPP
#ifndef _RD
        // first reset and disable FPGA
    FpgaReset();
#endif
#endif

        // setup memory markers
#ifdef _INFINEON_
    SCU_MKMEM0=uwPar0;
    SCU_MKMEM1=uwPar1;
#else
        // save reset parameters
    uwSysResParam0=uwPar0;
    uwSysResParam1=uwPar1;
    (void)save_resparam();
#endif

    	// Select Application Reset
    Xil_Out32(SLCR_UNLOCK_ADDR, UNLOCK_KEY);

        // Then execure software reset
    Xil_Out32(PSS_RST_CTRL_REG, PSS_RST_MASK);
}

//****************************************************************************
// Modify MultiBoot Register

void SysRes_ModifyMultiBoot(ULONG ulAddrOffset)
{
    Xil_Out32(XDCFG_UNLOCK_REG , UNLOCK_XDCFG_KEY );
    Xil_Out32(XDCFG_MULTIBOOT_REG , (ulAddrOffset&0xFFFFFF)>>15); // MULTIBOOT ADDRESS, * 32KB
}

//***************************************************************************
// Return reboot reason status

UBYTE SysRes_RebootReason(void)
{
    UBYTE Regval;

    /* We are using REBOOT_STATUS_REG, we have to use bits 23:16 */
    /* for storing the RESET_REASON register value*/
    Regval = (UBYTE)((Xil_In32(REBOOT_STATUS_REG) >> 16) & 0xFF);

    return Regval;
}

void SysRes_RebootPorClr(void)
{
    Xil_Out32(REBOOT_STATUS_REG, Xil_In32(REBOOT_STATUS_REG)&~POR_MASK);
}

//***************************************************************************
// Set Reboot State

void SysRes_RebootStateSet(UBYTE ubMark)
{
    Xil_Out32(REBOOT_STATUS_REG, (Xil_In32(REBOOT_STATUS_REG)&~STATE_MASK)|((ULONG)(ubMark&0xF)<<24));
}

//***************************************************************************
// Set Reboot State

UBYTE SysRes_RebootStateGet(void)
{
    ULONG State;

    State = (Xil_In32(REBOOT_STATUS_REG)&STATE_MASK)>>24;

    return (UBYTE)State;
}

//***************************************************************************
// restore reset parameter from flash

static BOOL restore_resparam(void)
{
    HPVOID storageptr;
    ULONG  storagesize;
    HPVOID blkptr;
    SWORD  blkcode;
#ifdef _APP_DEBUG
    UWORD  uwResParam[2]={(UWORD)RESETCODE_CLEANREBOOTREQUESTED, (UWORD)~RESETCODE_CLEANREBOOTREQUESTED};
#else
    UWORD  uwResParam[2]={(UWORD)SYSRES_POWERON_VALUE_PAR0, (UWORD)SYSRES_POWERON_VALUE_PAR1};
#endif
    UWORD  ct;

    for(ct=0;ct<RESPARAMBLOCKCOUNT;ct++)
    {
        storageptr=(HPUBYTE)sResParamBlockDefs[ct].hpvStart;
        storagesize=sResParamBlockDefs[ct].ulSize;

        blkcode=blkstor_enumvalid(&storageptr, &storagesize, &blkptr);

            // then seek managed blocks
        while(blkcode != BLKSTOR_ERR_NOTFOUND)
            switch(blkcode)
            {
                case DATACODE_PARAM_SYSTEMRESET:
                    if(blkstor_getdata(blkptr,0,DATACODE_PARAM_SYSTEMRESET,uwResParam,sizeof(uwResParam))>0)
                    {
                        // read reset parameters
                        uwSysResParam0=uwResParam[0];
                        uwSysResParam1=uwResParam[1];

                        // erase current block
                        (void)ProgramFlashErase(storageptr, storagesize);

                        // next block
                        if(ct==RESPARAMBLOCKCOUNT-1)
                            resparam_nextblk = 0;
                        else
                            resparam_nextblk = ct+1;

                        return TRUE;
                    }

                    break;
            }
    }

    uwSysResParam0=uwResParam[0];
    uwSysResParam1=uwResParam[1];

    return FALSE;
}

//***************************************************************************
// save reset paraemter to flash

static BOOL save_resparam(void)
{
    
    HPVOID stor_addr=(HPUBYTE)sResParamBlockDefs[resparam_nextblk].hpvStart;
    ULONG  stor_size=sResParamBlockDefs[resparam_nextblk].ulSize;

    HPVOID * ppvDest = &stor_addr;
    UWORD uwResParam[2] = {uwSysResParam0, uwSysResParam1};
    static BLKSTOR_HEADER  sTail;
    UWORD uwLocSize;

        // create tail info
    if(blkstor_tail_begin(&sTail, DATACODE_PARAM_SYSTEMRESET)<0)
        return FALSE;

    uwLocSize=sizeof(uwResParam);

        // write data
    if(ProgramFlashLoadWritePage(*ppvDest, (HPUBYTE)uwResParam, uwLocSize) != FLASHMGR_R_OK)
        return FALSE;

        // update pointers and left size
    *ppvDest=&((UBYTE  *)*ppvDest)[uwLocSize];

        // add data to header info
    if(blkstor_tail_addata(&sTail, uwResParam, uwLocSize)<0)
        return FALSE;

        // finalize tail info
    if(blkstor_tail_end(&sTail)<0)
        return FALSE;

        // write complete header
    if(ProgramFlashLoadWritePage(*ppvDest, (HPUBYTE)&sTail, sizeof(sTail)) != FLASHMGR_R_OK)
        return FALSE;
    
    return TRUE;
}
