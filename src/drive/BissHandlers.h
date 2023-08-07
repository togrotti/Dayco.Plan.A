/****************************************************************************/
/* Project: AxN Control Board                                               */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved.  */
/*                                                                          */
/* File        : BissHandlers.h                                             */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Low level Biss interface management functions              */
/*                                                                          */
/****************************************************************************/

#ifndef _BISSHANDLERS_H
#define _BISSHANDLERS_H

#include "common\CommonDefines.h"

//***************************************************************************
// Defines

#define BISS_FUNCTION_ERROR_TIMEOUT             0x80000000l
#define BISS_FUNCTION_ERROR_ERROR               0x40000000l
#define BISS_FUNCTION_ERROR_ACCESSDENIED        0x02000000l
#define BISS_FUNCTION_ERROR_NOMODULE            0x01000000l
#define BISS_FUNCTION_ERROR_MASK_ANY            0xff000000l

#define BISS_ENCTYPE_ROTARY                     0
#define BISS_ENCTYPE_LINEAR                     1
#define BISS_ENCTYPE_ROTARY_MT                  2
//***************************************************************************
// Data types

typedef struct
{
    UBYTE ubStepPerRevBits;
    UBYTE ubRevNumBits;
    ULONG dwBaseAddress;
    UWORD uwMaxFrequency;
    UBYTE ubEncType;
} BISS_WORKS;

//***************************************************************************
// Functions

#ifdef _INFINEON_
void  BissEnterCommandMode(BISS_WORKS huge * psWorks);
void  BissEnterPositionMode(BISS_WORKS huge * psWorks);
UWORD BissInitializeEncoder(UBYTE ubType, ULONG dwBaseAddress, ULONG ulClockFreq, BISS_WORKS huge * psWorks, BOOL bCalcStartDelay);
#else
void  BissEnterCommandMode(BISS_WORKS * psWorks);
void  BissEnterPositionMode(BISS_WORKS * psWorks);
UWORD BissInitializeEncoder(UBYTE ubType, ULONG dwBaseAddress, ULONG ulClockFreq, BISS_WORKS * psWorks, BOOL bCalcStartDelay);
#endif
BOOL BissCheckParameters(UBYTE ubEncType, ULONG ulClockFreq);
void  BissReadPosition(BISS_WORKS* psWorks, HPULONG hpulPosDataH, HPULONG hpulPosDataL);
void  BissReadStatus(BISS_WORKS* psWorks, HPUWORD hpuwStatus);

#endif
