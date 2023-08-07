/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : Int64Functions.h                                           */
/* Author      : Stefano Martino                                            */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Math routines for 64bit integer operations; most of those  */
/*               are inline for efficiency reason                           */
/*                                                                          */
/****************************************************************************/

#ifndef _INT64FUNCTIONS_H
#define _INT64FUNCTIONS_H

#include "common\CommonDefines.h"

//***************************************************************************
// Basic macros

#define INT64_ASSIGN( dst, mso, lso ) { dst.hi = mso; dst.lo = lso; }
#define INT64_COPY( dst, src )        { dst.lo = src.lo; dst.hi = src.hi; }

#define INT64_WORD0( var ) (*(((UWORD *)&var)+0))
#define INT64_WORD1( var ) (*(((UWORD *)&var)+1))
#define INT64_WORD2( var ) (*(((UWORD *)&var)+2))
#define INT64_WORD3( var ) (*(((UWORD *)&var)+3))

#define INT64_LLONG( var ) (*(ULONG *)(((UWORD *)&var)+0))
#define INT64_MLONG( var ) (*(ULONG *)(((UWORD *)&var)+1))
#define INT64_HLONG( var ) (*(ULONG *)(((UWORD *)&var)+2))

//***************************************************************************
// Atomic copy
#warning "TODO: atomic & endatomic"
void _sint64_atomic_copy( SQWRD * dst, const SQWRD * src );
#define _uint64_atomic_copy( dst, src) _sint64_atomic_copy((SQWRD *)dst, (const SQWRD *)src)

//***************************************************************************
// Add

void _sint64_add( SQWRD * dst, const SQWRD * op1, const SQWRD * op2 ) ;
void _sint64_add_64_32( SQWRD * dst, const SQWRD * op1, const SLONG * op2 ) ;
void _sint64_add_32_32( SQWRD * dst, const SLONG * op1, const SLONG * op2 ) ;

void _uint64_add( UQWRD * dst, const UQWRD * op1, const UQWRD * op2 ) ;
void _uint64_add_64_32( UQWRD * dst, const UQWRD * op1, const ULONG * op2 ) ;
void _uint64_add_32_32( UQWRD * dst, const ULONG * op1, const ULONG * op2 ) ;

//***************************************************************************
// Sub

void _sint64_sub( SQWRD * dst, const SQWRD * op1, const SQWRD * op2 ) ;
void _sint64_sub_nosaturation( SQWRD * dst, const SQWRD * op1, const SQWRD * op2 ) ;
void _sint64_sub_64_32( SQWRD * dst, const SQWRD * op1, const SLONG * op2 ) ;
void _sint64_sub_32_32( SQWRD * dst, const SLONG * op1, const SLONG * op2 ) ;

void _uint64_sub( UQWRD * dst, const UQWRD * op1, const UQWRD * op2 ) ;
void _uint64_sub_64_32( UQWRD * dst, const UQWRD * op1, const ULONG * op2 ) ;
void _uint64_sub_32_32( UQWRD * dst, const ULONG * op1, const ULONG * op2 ) ;

//***************************************************************************
// Multiplication

void _sint64_mul_32_32( SQWRD * dst, const SLONG * op1, const SLONG * op2 ) ;
void _uint64_mul_32_32( UQWRD * dst, const ULONG * op1, const ULONG * op2 ) ;
void _sint64_mul_32_16( SQWRD * dst, const SLONG * op1, const SWORD * op2 ) ;
void _sint64_mul_48_16( SQWRD * dst, const SQWRD * op1, const SWORD * op2 ) ;

//***************************************************************************
// Compare and min/max selection

    // op1>op2 => +1, op1==op2 => 0, op1<op2 => -1
SBYTE _sint64_comp( SQWRD * op1, SQWRD * op2 ) ;

//void _sint64_min( SQWRD * dst, const SQWRD * op1) ;
//void _sint64_max( SQWRD * dst, const SQWRD * op1) ;

//***************************************************************************
// Conversion

SLONG _sint64toSint32(SQWRD * sqVar ) ;
//void _sint32toSint64(SQWRD * dst, SLONG * src ) ;
void _sint64toSint48(SQWRD * sqVar);

//***************************************************************************
// Shift

void _sint64_shl( SQWRD * var, UWORD cnt );
void _sint64_shr( SQWRD * var, UWORD cnt );
void _uint64_shl( UQWRD * var, UWORD cnt );
void _uint64_shr( UQWRD * var, UWORD cnt );

//***************************************************************************
// PLC compatible functions

void _sint64p_scale_32( SQWRD  * val, SLONG scale ) ;
void _sint64p_add( SQWRD  * val, ULONG lo, SLONG hi );
void _sint64p_sub( SQWRD  * val, ULONG lo, SLONG hi );

//***************************************************************************
// constants

extern const SQWRD sqZero64bit;

#endif
