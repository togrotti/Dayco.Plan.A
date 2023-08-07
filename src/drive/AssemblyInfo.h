/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : AssemblyInfo.h                                             */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Information about complete drive assembly                  */
/*                                                                          */
/****************************************************************************/

#ifndef _ASSEMBLYINFO_H
#define _ASSEMBLYINFO_H

#include "common\CommonDefines.h"

//***************************************************************************
// Assembly Info

typedef struct
{
        // assembly s/n
    ULONG   ulSerialNumber;
        // assembly name/description
    CHARS   chDescrText[ 32 ];
        // assembly production date/lot
    CHARS   chDateText[ 16 ];
}
#ifdef _INFINEON_
                        ASSEMBLY_INFO;
#else
__attribute__((packed)) ASSEMBLY_INFO;
#endif

//***************************************************************************
// Assembly Info - previous generation

typedef struct
{
        // assembly s/n
    ULONG   ulSerialNumber;
        // assembly name/description
    CHARS   chDescrText[ 16 ];
        // assembly production date/lot
    CHARS   chDateText[ 16 ];
}
#ifdef _INFINEON_
                        ASSEMBLY_V19_INFO;
#else
__attribute__((packed)) ASSEMBLY_V19_INFO;
#endif

#endif
