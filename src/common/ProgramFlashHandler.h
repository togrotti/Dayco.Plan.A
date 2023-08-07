/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ProgramFlashHandler.h                                      */
/* Author      : Stefano Martino                                            */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Programming Routines for Program Flash in XE16x device.    */
/*                                                                          */
/****************************************************************************/

#ifndef _PROGRAMFLASH_HANDLER_H
#define _PROGRAMFLASH_HANDLER_H

#include "core\Flash.h"

/////////////////////////////////////////////////////////////////////////////
//

#define PROGRAMFLASH_SECTOR_SIZE              SECTOR_SIZE
#define PROGRAMFLASH_PAGE_SIZE                PAGE_SIZE

/////////////////////////////////////////////////////////////////////////////
//

// #define PROGRAMFLASH_BASE_ADDRESS_MODULE_0    0xC00000
// #define PROGRAMFLASH_BASE_ADDRESS_MODULE_1    0xC40000
// #define PROGRAMFLASH_BASE_ADDRESS_MODULE_2    0xC80000

/////////////////////////////////////////////////////////////////////////////
//

#ifdef _AXX_BOOTBLOCK
#define PROGRAMFLASH_SEC_PAGE_WORD_SZ         (PROGRAMFLASH_PAGE_SIZE/2)
#define PROGRAMFLASH_SEC_BLOCK_WORD_SZ        8

// #define PROGRAMFLASH_BASE_ADDRESS_SEC_P0      0xC00000
// #define PROGRAMFLASH_BASE_ADDRESS_SEC_P1      0xC00080
#endif

/////////////////////////////////////////////////////////////////////////////
// Critical sections (if required)

#ifdef _AXX_SYSAPP
extern BOOL bProgramFlashCritSectBypass;
#endif

/////////////////////////////////////////////////////////////////////////////
//

// unsigned short ProgramFlashReadSignature( void );

// int ProgramFlashEraseSector( unsigned long adr );
// int ProgramFlashEraseWordLine( unsigned long adr );
int ProgramFlashLoadWritePage( unsigned long adr, unsigned char  * buf, unsigned int len );
// int ProgramFlashEraseAndWritePage( unsigned long adr, unsigned char  * buf, unsigned int len );

int ProgramFlashErase( void  *, unsigned long len );
int ProgramFlashWrite( void  *, void  * buf, unsigned long len );
int ProgramFlashErasedCheck( void  * adr, unsigned long len );
int ProgramFlashRead( void  *, void  * buf, unsigned long len );

#ifdef _AXX_BOOTBLOCK
int ProgramFlashEraseSecurityPage( unsigned long adr );
int ProgramFlashLoadSecurityPage( unsigned long adr, unsigned short  * buf );
int ProgramFlashDisableReadProtection( unsigned short pwd0, unsigned short pwd1, unsigned short pwd2, unsigned short pwd3 );
int ProgramFlashDisableWriteProtection( unsigned short pwd0, unsigned short pwd1, unsigned short pwd2, unsigned short pwd3 );
int ProgramFlashReEnableReadWriteProtection( void );
#endif //_AXX_BOOTBLOCK

#endif // _PROGRAMFLASH_HANDLER_H
