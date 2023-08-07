/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : HeaderFooterInfo.h                                         */
/* Author      : Stefano Martino                                            */
/*               Fabio Terrile                                              */
/*                                                                          */
/* Description : Information for Header/Footer                              */
/*                                                                          */
/****************************************************************************/

#ifndef _HEADER_FOOTER_INFO
 #define _HEADER_FOOTER_INFO

#include "common\CommonDefines.h"

/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
  UWORD   uwApplicatType;
  UWORD   uwVersionMajor;
  UWORD   uwVersionMinor;
  ULONG   ulNextBlockPtr;
  UWORD   uwBuildNumber;
  UWORD   uwOptionsMask;

  UBYTE   ubSpares[ 2 ];  

  CHARS   chBlockDescrText[ 16 ];

} HEADER_FOOTER_INFO;

#endif // _HEADER_FOOTER_INFO
