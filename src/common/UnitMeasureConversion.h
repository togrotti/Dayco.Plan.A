/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : UnitMeasureConversion.h                                    */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Fast UM Conversion                                         */
/*                                                                          */
/****************************************************************************/

#ifndef _UNITMEASURECONVERSION_H
#define _UNITMEASURECONVERSION_H

#include <math.h>
#include "CommonDefines.h"

//***************************************************************************
// Conversion data structure

typedef struct _umconv_32to32
{
    SLONG slMult;
    SWORD swShift;
    UWORD uwSelection;
} UMCONV_CONV32TO32;

typedef struct _umconv_16to32
{
    SLONG slMult;
    SWORD swShift;
    UWORD uwSelection;
} UMCONV_CONV16TO32;

typedef struct _umconv_32to16
{
    SWORD swMult;
    SWORD swShift;
    UWORD uwSelection;
} UMCONV_CONV32TO16;

//***************************************************************************
// Macros for conversion

#define UMCONV_CONVERT_32TO32(ds,val)  (UmConv_Convert_32to32((ds)->slMult, (ds)->swShift, (val), (ds)->uwSelection))
#define UMCONV_CONVERT_16TO32(ds,val)  (UmConv_Convert_16to32((ds)->slMult, (ds)->swShift, (val), (ds)->uwSelection))
#define UMCONV_CONVERT_32TO16(ds,val)  (UmConv_Convert_32to16((ds)->swMult, (ds)->swShift, (val), (ds)->uwSelection))

//***************************************************************************
// Data structure initialization: fill-up the conversion data structure
// to be used at runtime
// Double precision value define ratio between input and output values:
// outval = inval * ratio

BOOL UmConv_Init_32to32(UMCONV_CONV32TO32 *, DOUBL);
BOOL UmConv_Init_16to32(UMCONV_CONV16TO32 *, DOUBL);
BOOL UmConv_Init_32to16(UMCONV_CONV32TO16 *, DOUBL);

//***************************************************************************
// Initialize both conversion data structure for the specified ratio and
// the conversion data structure to convert back to original value
// the first data structure let eval : outval = inval * ratio
// the second data structure let eval: outval = inval / ratio

BOOL UmConv_Init_32to32_Rev(UMCONV_CONV32TO32 *, UMCONV_CONV32TO32 *, DOUBL);
BOOL UmConv_Init_16to32_Rev(UMCONV_CONV16TO32 *, UMCONV_CONV32TO16 *, DOUBL);

//***************************************************************************
// Conversion modules, not to be called directly but via
// UMCONV_CONVERT_* macros

SLONG UmConv_Convert_32to32(SLONG mult, SWORD shift, SLONG value, UWORD sel);
SLONG UmConv_Convert_16to32(SLONG mult, SWORD shift, SWORD value, UWORD sel);
SWORD UmConv_Convert_32to16(SWORD mult, SWORD shift, SLONG value, UWORD sel);

//***************************************************************************
// Multiplier and scaler computing

BOOL UmConv_MultShiftCalc(DOUBL dbRatio, SLONG * pslMult, SWORD * pswShift, SWORD swNBitMult);

#endif
