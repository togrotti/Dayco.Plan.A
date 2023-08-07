/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : HardwareConfig.h                                           */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Configuration of hardware related parameters and switched  */
/*               related to configuration stored in bootblock area and      */
/*               on externally serial EEPROM chips                          */
/*                                                                          */
/****************************************************************************/

#ifndef _HARDWARECONFIG_H
#define _HARDWARECONFIG_H

#include "system\SysAppGlobals.h"

//***************************************************************************
// Defines

#ifndef _HW_DC
#define HWCONF_MAXCFGBLOCKS         8
#else
#define HWCONF_MAXCFGBLOCKS         32
#endif

#define HWCONF_SYSMODE_COMP         0
#define HWCONF_SYSMODE_PERF         1

//***************************************************************************
// Parameters data structure

typedef struct {
#ifndef _INFINEON_
    UWORD ubSysMode;
    UWORD ubDummy;
#else
    UBYTE ubSysMode;
    UBYTE ubDummy;
#endif
} HWCONF_PARAMS;

//***************************************************************************
// Data structures

typedef struct
{
    CHARS    chApp[4];
    UWORD    uwAppCode;
    UWORD    uwType;
    UWORD    uwBuild;
} HWCONF_FPGACONFIG;

typedef struct
{
    UBYTE    ubSN[8];
    UBYTE    ubBus;
    UBYTE    ubDummy;
    UWORD    uwDataCode;
    UWORD    uwProductRev;
} HWCONF_CFGBLOCKS;

//***************************************************************************
// Globals

extern HWCONF_FPGACONFIG  tHwFpgaConfig;

extern HWCONF_FPGACONFIG  tHwIOExpBoardConfig;

// RAM FCODE limits defined by the linker
//extern UBYTE RAMMOD_FCODE_BEGIN;
//extern UBYTE RAMMOD_FCODE_STATIC_END;
//extern UBYTE RAMMOD_FCODE_ENDBEFORE;

extern const HPVOID hpvHwCfgBlocksAddr;
extern UWORD uwHwCfgBlocksSz;
extern const UWORD uwHwCfgBlocksRecordLength;

extern HWCONF_PARAMS sHwConfParams;
extern const HWCONF_PARAMS sHwConfDefParams;

//***************************************************************************
// Globals

    // Get configuration and check
BOOL HwConfGetAndCheck(void (* yield)(void));

#ifndef _HW_DC
   // Store hardware configuration in local flash
BOOL HwConfStore(void);
#endif

    // Enable/disable PSRAM write protection
BOOL HwConfPSRAMWrProt(BOOL bEnable);

    // Init handler for parameters checking
BOOL HwConfInit(void);

#ifdef _APP_XC
    // At startup request user-setup system mode
void HwConfBoot(BOOL bValid);
#endif

    // Check global options requested by configuration
BOOL HwConfGlobalOptions(void);

#ifdef _HW_DC
    // Get unique ID as 32bit
ULONG HwGetUID32(void);
#endif
    // Get unique ID as 24bit
void HwGetUID24(HPUBYTE hpubID);

UWORD PlcHwProductRev(UWORD uwDataCode);

#endif
