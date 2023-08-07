/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ProgramFlashHandler.c                                      */
/* Author      : Stefano Martino                                            */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Programming Routines for Program Flash in XE16x device.    */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "common\CommonTypedef.h"
#include "ProgramFlashHandler.h"

/////////////////////////////////////////////////////////////////////////////
// Critical sections (if required)

#ifdef _AXX_SYSAPP
// #include "Os.h"

BOOL bProgramFlashCritSectBypass=FALSE;

// OS_CREATEMUTEX(sFlashLock);

// #define _begin_critical()           {if(!bProgramFlashCritSectBypass) Os_BeginCriticalSection(OS_CRITSECT_GLOBAL);}
// #define _end_critical()             {if(!bProgramFlashCritSectBypass) Os_EndCriticalSection(OS_CRITSECT_GLOBAL);}

// #define _lock_module()              {if(!bProgramFlashCritSectBypass) Os_MutexWait(&sFlashLock, 0);}
// #define _unlock_module()            {if(!bProgramFlashCritSectBypass) Os_MutexSignal(&sFlashLock);}

// #else

// #define _begin_critical()
// #define _end_critical()

// #define _lock_module()
// #define _unlock_module()

#endif

// #define _LOCAL_ENABLEPROTMODULE     0
// #ifdef _AXX_BOOTBLOCK
// #undef  _LOCAL_ENABLEPROTMODULE
// #define _LOCAL_ENABLEPROTMODULE     1
// #endif
// #ifdef _AXX_BOOTUPDATER
// #undef  _LOCAL_ENABLEPROTMODULE
// #define _LOCAL_ENABLEPROTMODULE     1
// #endif

/////////////////////////////////////////////////////////////////////////////
//

// #include "ProgramFlashImpl.h"

/////////////////////////////////////////////////////////////////////////////
//

// unsigned short ProgramFlashReadSignature( void )
// {
//   return SCU_IDCHIP;
// }


/////////////////////////////////////////////////////////////////////////////
//
// Erase Sector in Flash Memory
//    Parameter:      adr:  Sector Address
//    Return Value:   0 - OK,  1 - Failed
//

// int ProgramFlashEraseSector( unsigned long adr )
// {
//   return LocalProgramFlashEraseSector( adr );
// }


/////////////////////////////////////////////////////////////////////////////
//
// Program Bytes in Flash Memory (one 128 bytes page)
//    Parameter:      adr:  Start Address
//                    buf:  Data Bytes
//                    len:  Length
//    Return Value:   0 - OK,  1 - Failed
//

int ProgramFlashLoadWritePage( unsigned long adr, unsigned char * buf, unsigned int len )
{
  unsigned long  pag;
  unsigned short dat;
  unsigned short blkstart;

  // Ensure that A0-A6 = 0 (128 byte page size)
  pag = adr & ~(PROGRAMFLASH_PAGE_SIZE-1);

  // Page Overlap not Allowed
  if ( adr + len > pag + PROGRAMFLASH_PAGE_SIZE )
    return 1;

  FlashWrite(adr, buf, len);

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
//
// Re-Write Bytes in Flash Memory (up to one 128 bytes page)
//    Parameter:      adr:  Start Address
//                    buf:  Data Bytes
//                    len:  Length
//    Return Value:   0 - OK,  1 - Failed
//

// int ProgramFlashEraseAndWritePage( unsigned long adr, unsigned char xhuge * buf, unsigned int len )
// {
//   unsigned long  base = adr & 0xFC0000; 
//   unsigned long  pag;
//   unsigned short idx;
//   unsigned short llen;
//   unsigned short dat;
//   unsigned char  lbuf[PROGRAMFLASH_PAGE_SIZE];
//   int            retval;

//   // Ensure that A0-A6 = 0 (128 byte page size)
//   pag = adr & ~(PROGRAMFLASH_PAGE_SIZE-1);

//   // Page Overlap not Allowed
//   if ( adr + len > pag + PROGRAMFLASH_PAGE_SIZE )
//     return 1;

//   // Read back actual page data
//   for ( idx = 0; idx < PROGRAMFLASH_PAGE_SIZE; idx++ )
//     lbuf[idx] = ((unsigned char xhuge *)pag)[idx];

//   // Prepare buffer with new data
//   for ( idx = (unsigned short)(adr-pag), llen=len; llen ; idx++, llen-- )
//     lbuf[idx] = *buf++;

//   _lock_module();
//   _begin_critical();

//   // Clear status flags
//   HVAR( unsigned short, base | 0xAA ) = 0xF5;
//   // Erase Page (1. Cycle)  
//   HVAR( unsigned short, base | 0xAA ) = 0x80;
//   // Erase Page (2. Cycle)
//   HVAR( unsigned short, base | 0x54 ) = 0xAA;
//   // Erase Page (3. Cycle)
//   HVAR( unsigned short, adr )         = 0x03;

//   _end_critical();

//   // Check Status until Device Ready  
//   retval = ProgramFlashCheckStatus( base );

//   if(retval)
//   {
//     _unlock_module();
//     return retval;
//   }

//   _begin_critical();

//   // Enter Page Mode (1. Cycle)
//   HVAR( unsigned short, base | 0xAA ) = 0x50;
//   // Enter Page Mode (2. Cycle)
//   HVAR( unsigned short, pag )         = 0xAA;

//   _end_critical();

//   WaitWhileBusy();

//   _begin_critical();

//   // Load Page Words
//   for ( idx = 0; idx < PROGRAMFLASH_PAGE_SIZE/2; idx++ ) {

//     ((unsigned char *)&dat)[ 0 ] = lbuf[idx*2];
//     ((unsigned char *)&dat)[ 1 ] = lbuf[idx*2+1];

//     // Store Page Word
//     HVAR( unsigned short, base | 0xF2 ) = dat;
//   }

//   // Write Page (1. Cycle)
//   HVAR( unsigned short, base | 0xAA ) = 0xA0;
//   // Write Page (2. Cycle)
//   HVAR( unsigned short, base | 0x5A ) = 0xAA;

//   _end_critical();

//   // Check Status until Device Ready
//   retval = ProgramFlashCheckStatus( base );

//   _unlock_module();

//   return retval;
// }

/////////////////////////////////////////////////////////////////////////////
//
// Erase Flash Memory from adr up to adr+len
//    Parameter:      addr:  Start Address
//                    len :  Length
//    Return Value:   0 - OK,  1 - Failed
//

int ProgramFlashErase( void * addr, unsigned long len )
{
    FlashErase(addr, len);
    return 0;
}

/////////////////////////////////////////////////////////////////////////////
//
// Program Flash Memory
//    Parameter:      addr:  Start Address
//                    buf :  Data Bytes
//                    len :  Length
//    Return Value:   0 - OK,  1 - Failed
//

int ProgramFlashWrite( void * addr, void * buf, unsigned long len )
{
    FlashWrite(addr, buf, len);
    return 0;
}

/////////////////////////////////////////////////////////////////////////////
//
// Read Flash Memory

int ProgramFlashRead( void * addr, void * buf, unsigned long len )
{
    FlashRead(addr, buf, len);
    return 0;
}

/////////////////////////////////////////////////////////////////////////////
//
// Check Flash Erased 
//    Parameter:      addr:  Start Address
//                    len :  Length
//    Return Value:   0 - OK,  1 - Failed
//

int ProgramFlashErasedCheck( void * addr, unsigned long len )
{
    return FlashErasedCheck(addr, len);
}

#if (_LOCAL_ENABLEPROTMODULE)

/////////////////////////////////////////////////////////////////////////////
//
// Erase Flash Security Page
//    Parameter:      adr:  0xC0'0000 for SecP0, 0xC0'0080 for SecP1
//    Return Value:   0 - OK,  1 - Failed
//

int ProgramFlashEraseSecurityPage( unsigned long adr )
{
  int retval;

  _lock_module();
  _begin_critical();

  // Erase Security Page (1. Cycle)  
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0xAA ) = 0x80;
  // Erase Security Page (2. Cycle)
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0x54 ) = 0xA5;
  // Erase Security Page (3. Cycle)
  HVAR( unsigned short, adr )         = 0x53;

  _end_critical();

  // Check Status until Device Ready  
  retval = ProgramFlashCheckStatus( PROGRAMFLASH_BASE_ADDRESS_MODULE_0 );

  _unlock_module();

  return retval;
}

/////////////////////////////////////////////////////////////////////////////
//
// Program Bytes in Security Page (one 128 bytes page)
//    Parameter:      adr:  0xC0'0000 for SecP0, 0xC0'0080 for SecP1
//                    buf :  Data Words, fixed page size 64 words (128 bytes)
//                    len :  Word length
//    Return Value:   0 - OK,  1 - Failed
//

int ProgramFlashLoadSecurityPage( unsigned long adr, unsigned short xhuge * buf )
{
  unsigned short idx;
  int retval; 

  _lock_module();
  _begin_critical();

  // Clear status flags
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0xAA ) = 0xF5;
  // Enter Page Mode (1. Cycle)
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0xAA ) = 0x55;
  // Enter Page Mode (2. Cycle)
  HVAR( unsigned short, adr )         = 0xAA;

  _end_critical();

  WaitWhileBusy();

  _begin_critical();

  // Load Page Words
  for ( idx = 0; idx < PROGRAMFLASH_PAGE_SIZE/2; idx++ )
    HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0xF2 ) = buf[idx];

  // Write Page (1. Cycle)
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0xAA ) = 0xA0;
  // Write Page (2. Cycle)
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0x5A ) = 0xAA;

  _end_critical();

  // Check Status until Device Ready
  retval = ProgramFlashCheckStatus( PROGRAMFLASH_BASE_ADDRESS_MODULE_0 );

  _unlock_module();

  return retval;
}

/////////////////////////////////////////////////////////////////////////////
//

int ProgramFlashDisableReadProtection( unsigned short pwd0, unsigned short pwd1, unsigned short pwd2, unsigned short pwd3 )
{
  _lock_module();
  _begin_critical();

  // Disable Read Protection (1. Cycle)  
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0x3C ) = 0x00;
  // Disable Read Protection (2. Cycle)
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0x54 ) = pwd0;
  // Disable Read Protection (3. Cycle)
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0xAA ) = pwd1;
  // Disable Read Protection (4. Cycle)
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0x54 ) = pwd2;
  // Disable Read Protection (5. Cycle)
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0xAA ) = pwd3;
  // Disable Read Protection (6. Cycle)  
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0x5A ) = 0x55;

  _end_critical();
  _unlock_module();

  return 0;
}


/////////////////////////////////////////////////////////////////////////////
//

int ProgramFlashDisableWriteProtection( unsigned short pwd0, unsigned short pwd1, unsigned short pwd2, unsigned short pwd3 )
{
  _lock_module();
  _begin_critical();

  // Disable Write Protection (1. Cycle)  
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0x3C ) = 0x00;
  // Disable Write Protection (2. Cycle)
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0x54 ) = pwd0;
  // Disable Write Protection (3. Cycle)
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0xAA ) = pwd1;
  // Disable Write Protection (4. Cycle)
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0x54 ) = pwd2;
  // Disable Write Protection (5. Cycle)
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0xAA ) = pwd3;
  // Disable Write Protection (6. Cycle)  
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0x5A ) = 0x05;

  _end_critical();
  _unlock_module();

  return 0;
}


/////////////////////////////////////////////////////////////////////////////
//

int ProgramFlashReEnableReadWriteProtection( void )
{
  // Re-Enable Read/Write Protection (1. Cycle)  
  HVAR( unsigned short, PROGRAMFLASH_BASE_ADDRESS_MODULE_0 | 0x5E ) = 0x00;

  return 0;
}

#endif

