/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2013, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : PlcBuildInfo.c                                             */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : PLC Info area for build automation                         */
/*                                                                          */
/****************************************************************************/

#include "PlcRT.h"

//****************************************************************************
// Info structure

extern PLC_ATTR_CONST( TARGETADDRS_EX ) sPlcDiagInfo;

// #pragma NOINIT RENAMECLASS (HCONST=PLCDINFO)

const PLC_BUILDINFO sPlcDiagBuildInfo =
{   
    (ULONG)&sPlcDiagInfo,
    PLCD_RT_CODE_START,
    PLCD_RT_CODE_SIZE,
    0,                  // calculated and filled by post build
    0,                  // calculated and filled by post build
};
