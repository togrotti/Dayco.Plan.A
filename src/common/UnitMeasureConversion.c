/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : UnitMeasureConversion.c                                    */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Fast UM Conversion                                         */
/*                                                                          */
/****************************************************************************/

//#include <xe167f.h>

#include "UnitMeasureConversion.h"

//***************************************************************************
// Mult/shift calc

BOOL UmConv_MultShiftCalc(DOUBL dbRatio, SLONG * pslMult, SWORD * pswShift, SWORD swNBitMult)
{
    DOUBL le;
    SWORD og;
    SLONG mldst;

    le=fabs(dbRatio);
	le=log(le)/log(2.0);
	og=(SWORD)le+1;
    le=dbRatio*pow(2.0,swNBitMult-1-og);
    mldst=(SLONG)le;
    if(dbRatio>0.0 && dbRatio<1.0)
        mldst++;
    if(dbRatio>-1.0 && dbRatio<0.0)
        mldst--;
	
	if(dbRatio>0.0 && mldst<0l)
		return TRUE;

	if(dbRatio<0.0 && mldst>0l)
		return TRUE;

    *pslMult=mldst;
    *pswShift=og+1;

    return FALSE;
}

//***************************************************************************
// Select function and shift

static BOOL ShiftSelection(SWORD * pswShift, UWORD * puwSelection)
{
    if(*pswShift>-32 && *pswShift<=-16)
    {
        *puwSelection=1;
        *pswShift=-*pswShift-16;
    }
    else if(*pswShift>-16 && *pswShift<=0)
    {
        *puwSelection=2;
        *pswShift=-*pswShift;
    }
    else if(*pswShift>0 && *pswShift<8)
    {
        *puwSelection=3;
        *pswShift=*pswShift;
    }
    else if(*pswShift>=8 && *pswShift<16)
    {
        *puwSelection=4;
        *pswShift=*pswShift-8;
    }
    else if(*pswShift>=16 && *pswShift<24)
    {
        *puwSelection=5;
        *pswShift=*pswShift-16;
    }
    else if(*pswShift>=24 && *pswShift<32)
    {
        *puwSelection=6;
        *pswShift=*pswShift-24;
    }
    else
        return TRUE;

    return FALSE;
}

//***************************************************************************
// Data structure initialization

BOOL UmConv_Init_32to32(UMCONV_CONV32TO32 * sConvDS, DOUBL dbRatio)
{
    if(UmConv_MultShiftCalc(dbRatio, &sConvDS->slMult, &sConvDS->swShift, 32))
        return TRUE;

    if(ShiftSelection(&sConvDS->swShift, &sConvDS->uwSelection))
        return TRUE;

    return FALSE;
}

BOOL UmConv_Init_16to32(UMCONV_CONV16TO32 * sConvDS, DOUBL dbRatio)
{

    if(UmConv_MultShiftCalc(dbRatio, &sConvDS->slMult, &sConvDS->swShift, 32))
        return TRUE;

    sConvDS->swShift-=16;

    if(ShiftSelection(&sConvDS->swShift, &sConvDS->uwSelection))
        return TRUE;

    if(sConvDS->uwSelection>4)
        return TRUE;

    return FALSE;
}

BOOL UmConv_Init_32to16(UMCONV_CONV32TO16 * sConvDS, DOUBL dbRatio)
{
    SLONG slMult;

    if(UmConv_MultShiftCalc(dbRatio, &slMult, &sConvDS->swShift, 16))
        return TRUE;

    sConvDS->swMult=(SWORD)(slMult&0xffff);

    if(ShiftSelection(&sConvDS->swShift, &sConvDS->uwSelection))
        return TRUE;

    if(sConvDS->uwSelection>4)
        return TRUE;

    return FALSE;
}

BOOL UmConv_Init_32to32_Rev(UMCONV_CONV32TO32 * sConvDS, UMCONV_CONV32TO32 * sConvDSRev, DOUBL dbRatio)
{
    if(sConvDS)
        if(UmConv_Init_32to32(sConvDS, dbRatio))
            return TRUE;

    if(sConvDSRev)
        if(UmConv_Init_32to32(sConvDSRev, 1.0/dbRatio))
            return TRUE;

    return FALSE;
}

BOOL UmConv_Init_16to32_Rev(UMCONV_CONV16TO32 * sConvDS, UMCONV_CONV32TO16 * sConvDSRev, DOUBL dbRatio)
{
    if(sConvDS)
        if(UmConv_Init_16to32(sConvDS, dbRatio))
            return TRUE;

    if(sConvDSRev)
        if(UmConv_Init_32to16(sConvDSRev, 1.0/dbRatio))
            return TRUE;

    return FALSE;
}

