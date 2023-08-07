/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppInfo.c                                               */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Application definition                                     */
/*                                                                          */
/****************************************************************************/

#include "HeaderFooterInfo.h"
#include "SysAppInfo.h"

#ifndef _RD
#include "SysAppIdentification.h"
#else
#include "RemDispAppIdentification.h"
#endif

//****************************************************************************
// Info structure

//#pragma NOINIT RENAMECLASS (HCONST=HDFTINFO)

const HEADER_FOOTER_INFO sSysAppInfo =
{   
    IDENT_APPLICATION_TYPE, 
    IDENT_VERSION_MAJOR, 
    IDENT_VERSION_MINOR,
    0x00000000,
    IDENT_VERSION_BUILD_NUMBER,
    0x0000,
    {0x00, 0x00},
    IDENT_APP_DESCRIPTION_SHORT
};
