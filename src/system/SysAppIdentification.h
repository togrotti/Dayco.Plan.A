/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppIdentification.h                                     */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Sys App Identification and versioning                      */
/*                                                                          */
/****************************************************************************/

#ifndef _SYSAPPIDENTIFICATION_H
#define _SYSAPPIDENTIFICATION_H

#include "common\CommonDefines.h"
#include "system\HeaderFooterInfo.h"
#include "common\AppIdentTypes.h"

//***************************************************************************
// Versioning, provided both as number and as string

#ifndef _HW_DC
#ifndef _HW_CT
#define IDENT_VERSION_MAJOR                 1   // #%m
#define IDENT_VERSION_MINOR                 16  // #%n

#define IDENT_SVERSION_MAJOR                "1"
#define IDENT_SVERSION_MINOR                "16"
#else
#define IDENT_VERSION_MAJOR                 3   // #%o
#define IDENT_VERSION_MINOR                 4   // #%p

#define IDENT_SVERSION_MAJOR                "3"
#define IDENT_SVERSION_MINOR                "4"
#endif

#else
#define IDENT_VERSION_MAJOR                 2   // #%r
#define IDENT_VERSION_MINOR                 5   // #%s

#define IDENT_SVERSION_MAJOR                "2"
#define IDENT_SVERSION_MINOR                "5"
#endif

//***************************************************************************
// Build number, provided both as number and as string

#define IDENT_VERSION_BUILD_NUMBER          15 // #%b
#define IDENT_SVERSION_BUILD_NUMBER         "15" // #%c

//***************************************************************************
// Identification

#define IDENT_APPLICATION_TYPE              (IDENT_FW_STANDARD)

#ifdef _APP_PLCDBGWORKS
#define IDENT_APP_PLCDBGWORKS               " P"
#else
#define IDENT_APP_PLCDBGWORKS               ""
#endif

#ifdef _APP_DEBUG

#ifndef _HW_AXS
#define IDENT_APP_DESCRIPTION_SHORT         "AxX SysApp D"IDENT_APP_PLCDBGWORKS
#define IDENT_APP_DESCRIPTION_LONG          "AxX System Application Dbg"IDENT_APP_PLCDBGWORKS" V"IDENT_SVERSION_MAJOR"-"IDENT_SVERSION_MINOR"-"IDENT_SVERSION_BUILD_NUMBER" "__DATE__" "__TIME__
#else // _HW_AXS
#if defined(_HW_AXS_CABI35KW)
#define IDENT_APP_DESCRIPTION_SHORT         "AxX Cabi D"IDENT_APP_PLCDBGWORKS
#define IDENT_APP_DESCRIPTION_LONG          "AxX Cabi Application Dbg"IDENT_APP_PLCDBGWORKS" V"IDENT_SVERSION_MAJOR"-"IDENT_SVERSION_MINOR"-"IDENT_SVERSION_BUILD_NUMBER" "__DATE__" "__TIME__
#elif defined(_HW_AXS_DAYCO22KW)
#define IDENT_APP_DESCRIPTION_SHORT         "AxX Dayco D"IDENT_APP_PLCDBGWORKS
#define IDENT_APP_DESCRIPTION_LONG          "AxX Dayco Application Dbg"IDENT_APP_PLCDBGWORKS" V"IDENT_SVERSION_MAJOR"-"IDENT_SVERSION_MINOR"-"IDENT_SVERSION_BUILD_NUMBER" "__DATE__" "__TIME__
#elif defined(_HW_AXS_SAIETTA)
#define IDENT_APP_DESCRIPTION_SHORT         "AxX S600 D"IDENT_APP_PLCDBGWORKS
#define IDENT_APP_DESCRIPTION_LONG          "AxX S600 Application Dbg"IDENT_APP_PLCDBGWORKS" V"IDENT_SVERSION_MAJOR"-"IDENT_SVERSION_MINOR"-"IDENT_SVERSION_BUILD_NUMBER" "__DATE__" "__TIME__
#endif
#endif // _HW_AXS

#else // !_app_debug

#ifdef _APP_LIMITED
#define IDENT_APP_DESCRIPTION_SHORT         "AxX SysApp L"IDENT_APP_PLCDBGWORKS
#define IDENT_APP_DESCRIPTION_LONG          "AxX System App Limited"IDENT_APP_PLCDBGWORKS" V"IDENT_SVERSION_MAJOR"-"IDENT_SVERSION_MINOR"-"IDENT_SVERSION_BUILD_NUMBER" "__DATE__" "__TIME__
#else
#define IDENT_APP_DESCRIPTION_SHORT         "AxX SysApp"IDENT_APP_PLCDBGWORKS
#define IDENT_APP_DESCRIPTION_LONG          "AxX System Application"IDENT_APP_PLCDBGWORKS" V"IDENT_SVERSION_MAJOR"-"IDENT_SVERSION_MINOR"-"IDENT_SVERSION_BUILD_NUMBER" "__DATE__" "__TIME__
#endif // _app_limited
#endif // _app_debug

#define IDENT_SYSTEM_ID                     "Ax-X"
#define IDENT_FWAPP_ID                      "SYAP"

#define IDENT_FACTORY_NAME                  "Ningbo Physis Technology"
#ifdef _HW_DC
#define IDENT_FAMILY_NAME                   "Ax Modular Series"
#else
#define IDENT_FAMILY_NAME                   "Ax Series"
#endif

#endif
