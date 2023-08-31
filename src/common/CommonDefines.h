/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CommonDefines.h                                            */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/


/////////////////////////////////////////////////////////////////////////////
//

#ifndef _COMMON_DEFINES_H
 #define _COMMON_DEFINES_H

#include "CommonTypedef.h"
#include "xbasic_types.h"

/////////////////////////////////////////////////////////////////////////////
//

#ifndef FALSE
  #define FALSE     0
  #define TRUE      (!FALSE)
#endif

#ifndef NULL
  #define NULL 0
#endif

#ifndef FAIL
  #define FAIL    1  
  #define SUCCESS (!FALSE) 
#endif

/////////////////////////////////////////////////////////////////////////////
//

#define SWORD_MAX_VALUE         ( 65535)
#define SWORD_MIN_VALUE         (-65535)
#define SLONG_MAX_VALUE         ( 2147483647L)
#define SLONG_MIN_VALUE         (-2147483647L)
#define ULONG_MAX_VALUE         (4294967295UL)
#define FLOAT_SQRT_OF_TWO       1.4142135623730950488016887242097f
#define FLOAT_SQRT_OF_THREE     1.7320508075688772935274463415059f
#define FLOAT_PI                3.1415926535897932384626433832795f
#define FLOAT_2POWER32          4294967296.0f

#define INT64_MAX_VALUE			( 9.223372036854775807E+18LL)   //   2^63 - 1
#define INT64_MIN_VALUE			(-9.223372036854775807E+18LL)   // -(2^63 - 1)
#define UINT64_MAX_VALUE		( 1.8446744073709551615E+19ULL) //   2^64 - 1
/////////////////////////////////////////////////////////////////////////////
//

#define max(a,b)  (((a) > (b)) ? (a) : (b))
#define min(a,b)  (((a) < (b)) ? (a) : (b))


/////////////////////////////////////////////////////////////////////////////
//

#define HIBYTE(w)  ((UBYTE)(((UWORD)(w) >> 8) & 0xFF))
#define LOBYTE(w)  ((UBYTE)(w))
#define LOWORD(l)  ((UWORD)(ULONG)(l))
#define HIWORD(l)  ((UWORD)((((ULONG)(l)) >> 16) & 0xFFFF))

/////////////////////////////////////////////////////////////////////////////
//

#define MAKELONG(low, high) ((SLONG)(((UWORD)(low)) | (((ULONG)((UWORD)(high))) << 16)))

// int64 variables management
// build signed and unsigned INT64
#define MAKE_INT64(slValHi,  ulValLo)	((SLLNG)((( (SLLNG)slValHi) << 32) | ((ULLNG)ulValLo)))
#define MAKE_UINT64(ulValHi, ulValLo)	((ULLNG)((( (ULLNG)ulValHi) << 32) | ((ULLNG)ulValLo)))

// split signed and unsigned INT64
#define MAKE_SQWRDHI(s64Val) ((SLONG)((SLLNG)s64Val  >> 32))
#define MAKE_SQWRDLO(s64Val) ((ULONG)((SLLNG)s64Val))
#define MAKE_UQWRDHI(u64Val) ((ULONG)((ULLNG)u64Val >> 32))
#define MAKE_UQWRDLO(u64Val) ((ULONG)((ULLNG)u64Val))
/////////////////////////////////////////////////////////////////////////////
//

#define NBITW( var )  ((_bitw_ *)&var)
//#define FBITW( var )  ((_bitw_ far  *)&var)
#define HBITW( var )  ((_bitw_ *)&var)
#define HBITL( var )  ((_bitl_ *)&var)

//////////////////////////////////////////////////////////////////////////////
// Memory model dependant
    // TINY
#if (__MODEL__ == 0)
#define _ret    ret
#else
#define _ret    rets
#endif

//////////////////////////////////////////////////////////////////////////////
//

#define _seg_(adr) (UWORD)(((ULONG)(void *)adr) >> 16)

//////////////////////////////////////////////////////////////////////////////
//
#ifdef _INFINEON_
#define DISABLE_IRQ()                       { __asm { push PSW }; __asm { bclr PSW_IEN }; __asm { nop };  __asm { nop }; \
                                              __asm { nop };      __asm { nop };          __asm { nop };  __asm { nop }; }
#define RESTORE_IRQ()                       { __asm { pop PSW }; }
#else
#define DISABLE_IRQ()                       
#define RESTORE_IRQ()                       
#endif //_infineon_
#endif // _COMMON_DEFINES_H
