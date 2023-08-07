/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ParamStorageManagement.h                                   */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Parameter Storage Management                               */
/*                                                                          */
/****************************************************************************/

#ifndef _PARAMSTORAGEMANAGEMENT_H
#define _PARAMSTORAGEMANAGEMENT_H

#include "common\CommonDefines.h"
#include "system\HeaderFooterInfo.h"

//****************************************************************************
// Return values

#define	PARMGM_B_OK							0
#define	PARMGM_B_ERROR						1
#define	PARMGM_B_ERASED						2
#define	PARMGM_B_LOCKFAILED 				3

//****************************************************************************
// Parameters list flags

#define PARMGM_F_DEFAULT                    0x0000

#define PARMGM_F_RESTOREONLY                0x0001
#define PARMGM_F_CALLBACK                   0x0002
#define PARMGM_F_PTRSOURCE                  0x0004
#define PARMGM_F_NOSETDEFAULT               0x0008
#define PARMGM_F_STOREONLY                  0x0010
#define PARMGM_F_CORE                       0x0020

//****************************************************************************
// Load parameters flags

#define PARMGM_LOAD_COREONLY                1
#define PARMGM_LOAD_FULL                    2

//****************************************************************************
// Data structure

typedef struct
{
	UWORD   uwDataCode;
	HPVOID  hpvData;
	UWORD   uwDataSize;
    UWORD   uwFlags;
    const void  * hpvDefault;
    UWORD   uwDefaultDataSize;
} PARMGM_LIST;

typedef struct
{
    UWORD   uwApplicatType;
    UWORD   uwVersionMajor;
    UWORD   uwVersionMinor;
	UWORD   uwBlockCRC;
    ULONG   ulTimeStamp;
	HPVOID  hpvBlockStart;
	UWORD   uwBlockSize;
} PARMGM_ENDSIGNATURE;

typedef struct
{
	HPVOID  hpvData;
	UWORD   uwDataSize;
} PARMGM_PTRSOURCE;

//****************************************************************************
// Mapping between datacode and physical data

extern const PARMGM_LIST                psParMgmGlobalList[];
extern const UWORD                      uwParMgmGlobalListSize;

//***************************************************************************
// Globals

extern PARMGM_ENDSIGNATURE sParMgmParamSignature;

//***************************************************************************
// Parameter manager

#ifdef _AXX_SYSAPP
    // init
void parmgm_par_init(void);

    // load default values of parameters
void parmgm_par_default(void);

    // find and verify the validity of a parameter block
SWORD parmgm_par_verify(void);

    // load the verified parameter block
SWORD parmgm_par_load(UBYTE ubOption);

    // save parameter block
SWORD parmgm_par_save(void);

    // restore default parameters (erase all)
SWORD parmgm_par_restore(void);

    // immediate restore of default parameters (only in ram, do not write flash)
SWORD parmgm_par_immrestore(void);

    // bootblock info load
SWORD parmgm_bootblock_load(HEADER_FOOTER_INFO * hpHeader, HPULONG hpulSysID, HPULONG hpulFwID);
#endif

#ifdef _AXX_BOOTBLOCK
SWORD parmgm_bootblock_save(void);
SWORD parmgm_bootblock_verify(void);
#endif

#endif
