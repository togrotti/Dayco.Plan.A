/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : AnProcessor.h                                              */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Analog processor manager                                   */
/*                                                                          */
/****************************************************************************/

#ifndef _ANPROCESSOR_H
#define _ANPROCESSOR_H

#include "drive\HardwareParameters.h"

//***************************************************************************
// Defines

#define ANPROC_OF_AVG_1SHT              0x01
#define ANPROC_OF_AVG_CONT              0x02
#define ANPROC_OF_AVG_NONE              0x04
#define ANPROC_OF_BLK_NONE              0x00
#define ANPROC_OF_BLK_INST              0x10
#define ANPROC_OF_BLK_AVG               0x20
#define ANPROC_OF_DST_SHORT             0x00
#define ANPROC_OF_DST_LONG              0x40
#define ANPROC_OF_UNSIGNED              0x80
#define ANPROC_OF_NONE                  0x00

#define ANPROC_ADR_DISABLE              0xFFFF

#define ANPROC_INIT_CORE                1
#define ANPROC_INIT_RUN                 2

#define ANPROC_CHANNEL_SIZE             32
#define ANPROC_RGO_TAGOFFSET            (ANPROC_CHANNEL_SIZE*2)

//***************************************************************************
// Data structures

typedef struct
{
    UBYTE ubOpt;                // options mask
    UBYTE ubNumSample;          // number of samples for average
    HWPRMS_AD_SETTINGS sCal;    // channel calibration
    FLOAT flScale;              // channel scale (only for firmware destination)
    UWORD uwDstIntImm;          // destination for DSPH immediate value
    UWORD uwDstIntAvg;          // destination for DSPH average value
    void * pvDstExtImm;         // destination for firmware immediate value
    void * pvDstExtAvg;         // destination for firmware average value
} ANPROC_CHAN;

typedef struct
{
    UBYTE ubOpt;                // options mask
    FLOAT flScale;              // channel scale
    void * pvDst;               // destination
} ANPROC_RGOUT;

typedef struct
{
    UBYTE ubOpt;
    void * pvDst;
} ANPROC_TABLE;

//****************************************************************************
// Global functions

BOOL AnProc_Init(UWORD uwSequence);
BOOL AnProc_Set(UBYTE ubSrcTag, ANPROC_CHAN  * hpsChannel);
BOOL AnProc_Get(UBYTE ubSrcTag, ANPROC_CHAN  * hpsChannel);
BOOL AnProc_RGOutSet(UBYTE ubSrcTag, ANPROC_RGOUT  * hpsRgOut);
BOOL AnProc_UnSet(UBYTE ubSrcTag);
BOOL AnProc_AdjustOffset(UBYTE ubSrcTag, UWORD uwOffset);
BOOL AnProc_ReloadOffsets(void);
BOOL AnProc_ForceRefresh(void);

#endif

