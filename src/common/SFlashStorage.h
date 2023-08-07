/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2016, Phase Motion Control. All Rights Reserved.             */
/*                                                                          */
/* File        : SFlashStorage.h                                            */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Serial Flash file storage manager                          */
/*                                                                          */
/****************************************************************************/

#ifndef _SFLASHSTORAGE_H
#define _SFLASHSTORAGE_H

#include "SecurityBlock.h"
#include "SerialFlashHandler.h"

//***************************************************************************
// Defines

#define SFSTOR_STR_OK                   0

#define SFSTOR_STR_DATAINVALID          -1
#define SFSTOR_STR_FLASHERROR           -2
#define SFSTOR_STR_NOMOREDATA           -3
#define SFSTOR_STR_OUTOFSPACE           -4

#define SFSTOR_DESTBUFSIZE              (SFLASH_SEQREAD_BLOCKSIZE)
#define SFSTOR_WRITEBUFSIZE             (SFLASH_PAGE_SIZE)

//***************************************************************************
// Data structures

typedef struct
{
    LOGICAL_BLOCK_HEADER sHeader;
#ifdef _INFINEON_
    MD5_CTX sHash;
#endif
    ULONG ulAddress;
    ULONG ulEndAddress;
    ULONG ulSizeLeft;
    ULONG ulStreamStart;
} SFSTOR_STATUS;

typedef struct
{
    ULONG ulAddress;
    ULONG ulEndAddress;
    UWORD uwBufSz;
    UBYTE ubBuf[SFSTOR_WRITEBUFSIZE];
} SFSTOR_WRSTATUS;

//****************************************************************************
// Global functions

#ifdef _INFINEON_
LOGICAL_BLOCK_HEADER * SFStor_SeekInit(SFSTOR_STATUS * psStatus, ULONG ulBaseAddress, ULONG ulSize);
LOGICAL_BLOCK_HEADER * SFStor_SeekNext(SFSTOR_STATUS * psStatus);

void SFStor_GetBoundaries(SFSTOR_STATUS * psStatus, ULONG * pulAddress, ULONG * pulSize);

SWORD SFStor_StreamBegin(SFSTOR_STATUS * psStatus, UBYTE * pubBuffer);
SWORD SFStor_StreamGetData(SFSTOR_STATUS * psStatus, UBYTE * pubBuffer);
SWORD SFStor_StreamEnd(SFSTOR_STATUS * psStatus, UBYTE * pubBuffer, BOOL bAbort);
#endif

SWORD SFStor_WriteBegin(SFSTOR_WRSTATUS * psWrStatus, ULONG ulBaseAddress, ULONG ulMaxSize);
SWORD SFStor_WriteData(SFSTOR_WRSTATUS * psWrStatus, HPUBYTE pubBuffer, UWORD uwSize);
SWORD SFStor_WriteEnd(SFSTOR_WRSTATUS * psWrStatus);

#endif

