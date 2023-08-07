/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : MemoryLayout.h                                             */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _MEMORY_LAYOUT_H
 #define _MEMORY_LAYOUT_H

#include "common\CommonDefines.h"

/////////////////////////////////////////////////////////////////////////////
//

#ifdef _AXX_BOOTBLOCK 
#define MEMORYLAYOUT_DATA
#define MEMORYLAYOUT_QUAL
#else
#define MEMORYLAYOUT_DATA       const
#define MEMORYLAYOUT_QUAL
#endif

/////////////////////////////////////////////////////////////////////////////
//

#define MEMORY_CONFIG_NORMAL    0x0001
#define MEMORY_CONFIG_PFLASH    0x0002
#define MEMORY_CONFIG_SFLASH    0x0004

#define MEMORY_CONFIG_READ      0x2000
#define MEMORY_CONFIG_WRITE     0x4000
#define MEMORY_CONFIG_ERASE     0x8000


/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
  ULONG   ulLowerAddress;
  ULONG   ulUpperAddress;
  UWORD   uwCryptoKeyNum;
  UWORD   uwMemoryConfig;
} MEMORY_SECTOR_INFO;


/////////////////////////////////////////////////////////////////////////////
//

MEMORYLAYOUT_DATA MEMORY_SECTOR_INFO MEMORYLAYOUT_QUAL * MemoryLayoutRetrieveSectorInfo( ULONG ulMemoryAddress );
MEMORYLAYOUT_DATA MEMORY_SECTOR_INFO MEMORYLAYOUT_QUAL * MemoryLayoutRetrieveListInfo( ULONG * pulSize );


#endif // _MEMORY_LAYOUT_H
