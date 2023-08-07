/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : SerialFlashHandler.h                                       */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _SERIAL_FLASH_HANDLER_H
 #define _SERIAL_FLASH_HANDLER_H

#ifdef _INFINEON_
/////////////////////////////////////////////////////////////////////////////
//

SWORD  SerialFlashInit( void );

/////////////////////////////////////////////////////////////////////////////
//

#define SFLASH_ELECTR_SIGN_UNKNOWN    0x00

#define SFLASH_ELECTR_SIGN_M25P20     0x11
#define SFLASH_ELECTR_SIGN_M25P40     0x12
#define SFLASH_ELECTR_SIGN_M25P80     0x13
#define SFLASH_ELECTR_SIGN_M25P16     0x14
#define SFLASH_ELECTR_SIGN_GENERIC32  0x15
#define SFLASH_ELECTR_SIGN_GENERIC64  0x16
#define SFLASH_ELECTR_SIGN_GENERIC128 0x17
#define SFLASH_ELECTR_SIGN_N25Q032    0x16

/////////////////////////////////////////////////////////////////////////////
//

#define SFLASH_MEMORY_SIZE_M25P20    262144
#define SFLASH_MEMORY_SIZE_M25P40    524288
#define SFLASH_MEMORY_SIZE_M25P80   1048576
#define SFLASH_MEMORY_SIZE_M25P16   2097152
#define SFLASH_MEMORY_SIZE_N25Q032  4194304
#define SFLASH_MEMORY_SIZE_GENERIC  4194304

/////////////////////////////////////////////////////////////////////////////
//

#define SFLASH_SECTOR_SIZE            65536
#define SFLASH_PAGE_SIZE                256
#define SFLASH_ADDRESS_OFFSET    0x00800000

/////////////////////////////////////////////////////////////////////////////
//

#define SFLASH_SEQREAD_BLOCKSIZE         16

/////////////////////////////////////////////////////////////////////////////
//

extern ULONG sdata ulSerialFlashSize;

/////////////////////////////////////////////////////////////////////////////
//

UWORD SerialFlashReadSignature( void );

BOOL SerialFlashReadBytes(   ULONG ulAddress, UBYTE xhuge * xpubBuffer, UWORD uwLength );
BOOL SerialFlashReadWithCallBack( ULONG ulAddress, void ( * fCallBack)(UBYTE, void *), void * vContest, UWORD uwLength );
BOOL SerialFlashWriteBytesEx(  ULONG ulAddress, UBYTE xhuge * xpubBuffer, UWORD uwLength, BOOL bWait );
BOOL SerialFlashEraseSectorEx( ULONG ulAddress, BOOL bWait );
BOOL SerialFlashWaitForIdle( void );
BOOL SerialFlashEraseBulk( void );
BOOL SerialFlashReadSeqInit( ULONG ulAddress );
void SerialFlashReadSeqGetBlock( UBYTE * pubBuffer );
void SerialFlashReadSeqFinal( void );

#define SerialFlashWriteBytes(x,y,z)    SerialFlashWriteBytesEx(x,y,z,FALSE)
#define SerialFlashEraseSector(x)       SerialFlashEraseSectorEx(x,FALSE)

#else

/////////////////////////////////////////////////////////////////////////////
//

#define SFLASH_SECTOR_SIZE            65536
#define SFLASH_PAGE_SIZE                256
#define SFLASH_ADDRESS_OFFSET       (XPS_QSPI_LINEAR_BASEADDR+0x00800000)

/////////////////////////////////////////////////////////////////////////////
//

#define SFLASH_SEQREAD_BLOCKSIZE         16

/////////////////////////////////////////////////////////////////////////////
//

#define ulSerialFlashSize           4194304

/////////////////////////////////////////////////////////////////////////////
//

#include "common\ProgramFlashHandler.h"

#define SerialFlashReadBytes(x,y,z)      (!ProgramFlashRead(x,y,z))
#define SerialFlashWriteBytes(x,y,z)     (!ProgramFlashWrite(x,y,z))
#define SerialFlashEraseSector(x)        (!ProgramFlashErase(x,PROGRAMFLASH_SECTOR_SIZE))

#define SerialFlashWriteBytesEx(x,y,z,f) (!ProgramFlashWrite(x,y,z))
#define SerialFlashEraseSectorEx(x,y)    (!ProgramFlashErase(x,PROGRAMFLASH_SECTOR_SIZE))
#endif

#endif // _SERIAL_FLASH_HANDLER_H
