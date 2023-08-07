/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CommonTypedef.h                                            */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/


/////////////////////////////////////////////////////////////////////////////
//

#ifndef _COMMON_TYPEDEF_H
 #define _COMMON_TYPEDEF_H


#ifndef _INFINEON_
#include "xil_types.h"
#endif

/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
  uint32_t   lo;
  int32_t    hi;
} _int64_;

typedef struct
{
  uint32_t   lo;
  uint32_t   hi;
} _uint64_;

/////////////////////////////////////////////////////////////////////////////
//

typedef char       		CHARS;    // 1 byte unsigned;   prefix: ch
typedef uint8_t    		UBYTE;    // 1 byte unsigned;   prefix: ub
typedef int8_t     		SBYTE;    // 1 byte signed;     prefix: sb
typedef uint16_t  		UWORD;    // 2 byte unsigned;   prefix: uw
typedef int16_t    		SWORD;    // 2 byte signed;     prefix: sw
typedef uint32_t   		ULONG;    // 4 byte unsigned;   prefix: ul
typedef int32_t    		SLONG;    // 4 byte signed;     prefix: sl
typedef _uint64_        UQWRD;    // 8 byte unsigned;   prefix: uq
typedef _int64_         SQWRD;    // 8 byte signed;     prefix: sq
typedef float           FLOAT;    // 4 byte float;      prefix: fl
typedef double          DOUBL;    // 8 byte float;      prefix: db
typedef int64_t	        SLLNG;    // 8 byte signed;     prefix: sll
typedef uint64_t        ULLNG;    // 8 byte unsigned;   prefix: ull

//typedef UBYTE  far   *  FPUBYTE;  // 4 byte pointer     prefix: fpub
typedef UBYTE    	*  HPUBYTE;  // 4 byte pointer     prefix: hpub
typedef UBYTE    	*  XPUBYTE;  // 4 byte pointer     prefix: xpub

//typedef UWORD  far   *  FPUWORD;  // 4 byte pointer     prefix: fpuw
typedef UWORD    	*  HPUWORD;  // 4 byte pointer     prefix: hpuw
//typedef SWORD  far   *  FPSWORD;  // 4 byte pointer     prefix: fpsw
typedef SWORD    	*  HPSWORD;  // 4 byte pointer     prefix: hpsw
//typedef ULONG  far   *  FPULONG;  // 4 byte pointer     prefix: fpul
typedef ULONG    	*  HPULONG;  // 4 byte pointer     prefix: hpul
//typedef SLONG  far   *  FPSLONG;  // 4 byte pointer     prefix: fpsl
typedef SLONG    	*  HPSLONG;  // 4 byte pointer     prefix: hpsl

//typedef void far     *  FPVOID;   // 4 byte pointer     prefix: fpv 
typedef void     	*  HPVOID;   // 4 byte pointer     prefix: hpv
typedef void     	*  XPVOID;   // 4 byte pointer     prefix: xpv

typedef unsigned char  BOOL;


/////////////////////////////////////////////////////////////////////////////
//

typedef volatile struct
{
  uint16_t  bit0  : 1;
  uint16_t  bit1  : 1;
  uint16_t  bit2  : 1;
  uint16_t  bit3  : 1;
  uint16_t  bit4  : 1;
  uint16_t  bit5  : 1;
  uint16_t  bit6  : 1;
  uint16_t  bit7  : 1;
  uint16_t  bit8  : 1;
  uint16_t  bit9  : 1;
  uint16_t  bit10 : 1;
  uint16_t  bit11 : 1;
  uint16_t  bit12 : 1;
  uint16_t  bit13 : 1;
  uint16_t  bit14 : 1;
  uint16_t  bit15 : 1;

} _bitw_;

typedef volatile struct
{
  uint32_t  bit0  : 1;
  uint32_t  bit1  : 1;
  uint32_t  bit2  : 1;
  uint32_t  bit3  : 1;
  uint32_t  bit4  : 1;
  uint32_t  bit5  : 1;
  uint32_t  bit6  : 1;
  uint32_t  bit7  : 1;
  uint32_t  bit8  : 1;
  uint32_t  bit9  : 1;
  uint32_t  bit10 : 1;
  uint32_t  bit11 : 1;
  uint32_t  bit12 : 1;
  uint32_t  bit13 : 1;
  uint32_t  bit14 : 1;
  uint32_t  bit15 : 1;
  uint32_t  bit16 : 1;
  uint32_t  bit17 : 1;
  uint32_t  bit18 : 1;
  uint32_t  bit19 : 1;
  uint32_t  bit20 : 1;
  uint32_t  bit21 : 1;
  uint32_t  bit22 : 1;
  uint32_t  bit23 : 1;
  uint32_t  bit24 : 1;
  uint32_t  bit25 : 1;
  uint32_t  bit26 : 1;
  uint32_t  bit27 : 1;
  uint32_t  bit28 : 1;
  uint32_t  bit29 : 1;
  uint32_t  bit30 : 1;
  uint32_t  bit31 : 1;

} _bitl_;

#endif // _COMMON_TYPEDEF_H
