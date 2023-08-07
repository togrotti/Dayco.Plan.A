/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : Int64Functions.c                                           */
/* Author      : Stefano Martino                                            */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Math routines for 64bit integer operations                 */
/*                                                                          */
/****************************************************************************/

//#include <XE167F.H>
//#include <intrins.h>

#include "Int64Functions.h"
#include <math.h>

//***************************************************************************
// Avoids warning C47: unreferenced parameter

//#pragma warning disable = 47

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (0)

//***************************************************************************
// constants

const SQWRD sqZero64bit = {0L, 0UL} ; // 'zero' a 64 bit

//#define _CRS_64FUNCFLOAT_
//***************************************************************************
// Atomic copy

void _sint64_atomic_copy( SQWRD * dst, const SQWRD * src )
{
	dst->lo = src->lo ;
	dst->hi = src->hi ;
}

//***************************************************************************
// s64bit = s64bit + s64bit

void _sint64_add( SQWRD * dst, const SQWRD * op1, const SQWRD * op2 )
{
#ifdef _CRS_64FUNCFLOAT_
	// needed to create single variables, otherwise compiler
	// create a mess with float cast to macro result
	SLLNG s64_op1 = MAKE_INT64(op1->hi, op1->lo) ;
	SLLNG s64_op2 = MAKE_INT64(op2->hi, op2->lo) ;
	FLOAT 	flDst = (FLOAT)s64_op1 + (FLOAT)s64_op2 ; // use double?

	// saturate to avoid overflow
	if (flDst >= INT64_MAX_VALUE)
		flDst = INT64_MAX_VALUE ;
	else if (flDst <= INT64_MIN_VALUE)
		flDst = INT64_MIN_VALUE ;

	dst->hi = MAKE_SQWRDHI((SLLNG)flDst) ;
	dst->lo = MAKE_SQWRDLO((SLLNG)flDst) ;
#else
	SLLNG s64Dst = (MAKE_INT64(op1->hi, op1->lo)) + (MAKE_INT64(op2->hi, op2->lo)) ;
	dst->hi = MAKE_SQWRDHI(s64Dst) ;
	dst->lo = MAKE_SQWRDLO(s64Dst) ;
#endif	// _CRS_64FUNCFLOAT_
}

//***************************************************************************
// u64bit = u64bit + u64bit

void _uint64_add( UQWRD * dst, const UQWRD * op1, const UQWRD * op2 )
{
	ULLNG u64Dst = (MAKE_UINT64(op1->hi, op1->lo)) + (MAKE_UINT64(op2->hi, op2->lo)) ;
	dst->hi = MAKE_UQWRDHI(u64Dst) ;
	dst->lo = MAKE_UQWRDLO(u64Dst) ;
}

//***************************************************************************
// s64bit = s64bit - s64bit

void _sint64_sub( SQWRD * dst, const SQWRD * op1, const SQWRD * op2 )
{
#ifdef _CRS_64FUNCFLOAT_
	// needed to create single variables, otherwise compiler
	// create a mess with float cast to macro result
	SLLNG s64_op1 = MAKE_INT64(op1->hi, op1->lo) ;
	SLLNG s64_op2 = MAKE_INT64(op2->hi, op2->lo) ;
	FLOAT flDst = (FLOAT)s64_op1 - (FLOAT)s64_op2 ; // use double?

	// saturate to avoid overflow
	if (flDst >= INT64_MAX_VALUE)
		flDst = INT64_MAX_VALUE ;
	else if (flDst <= INT64_MIN_VALUE)
		flDst = INT64_MIN_VALUE ;

	dst->hi = MAKE_SQWRDHI((SLLNG)flDst) ;
	dst->lo = MAKE_SQWRDLO((SLLNG)flDst) ;

#else
	SLLNG s64Dst = (MAKE_INT64(op1->hi, op1->lo)) - (MAKE_INT64(op2->hi, op2->lo)) ;
	dst->hi = MAKE_SQWRDHI(s64Dst) ;
	dst->lo = MAKE_SQWRDLO(s64Dst) ;
#endif // _CRS_64FUNCFLOAT_
}

//***************************************************************************
// u64bit = u64bit - u64bit

void _uint64_sub( UQWRD * dst, const UQWRD * op1, const UQWRD * op2 )
{
	ULLNG u64Dst = (MAKE_UINT64(op1->hi, op1->lo)) - (MAKE_UINT64(op2->hi, op2->lo)) ;
	dst->hi = MAKE_UQWRDHI(u64Dst) ;
	dst->lo = MAKE_UQWRDLO(u64Dst) ;
}

//***************************************************************************
// s64bit = s64bit + s32bit

void _sint64_add_64_32( SQWRD * dst, const SQWRD * op1, const SLONG * op2 )
{
	SLLNG s64Dst = (MAKE_INT64(op1->hi, op1->lo)) + (SLLNG)(*op2) ;
	dst->hi = MAKE_SQWRDHI(s64Dst) ;
	dst->lo = MAKE_SQWRDLO(s64Dst) ;
}

//***************************************************************************
// u64bit = u64bit + u32bit

void _uint64_add_64_32( UQWRD * dst, const UQWRD * op1, const ULONG * op2 )
{
	ULLNG u64Dst = (MAKE_UINT64(op1->hi, op1->lo)) + (ULLNG)(*op2) ;
	dst->hi = MAKE_UQWRDHI(u64Dst) ;
	dst->lo = MAKE_UQWRDLO(u64Dst) ;
}

//***************************************************************************
// s64bit = s32bit + s32bit

void _sint64_add_32_32( SQWRD * dst, const SLONG * op1, const SLONG * op2 )
{
	SLLNG s64Dst = (SLLNG)(*op1) + (SLLNG)(*op2) ;
	dst->hi = MAKE_SQWRDHI(s64Dst) ;
	dst->lo = MAKE_SQWRDLO(s64Dst) ;
}

//***************************************************************************
// u64bit = u32bit + u32bit

void _uint64_add_32_32( UQWRD * dst, const ULONG * op1, const ULONG * op2 )
{
	ULLNG u64Dst = (ULLNG)(*op1) + (ULLNG)(*op2) ;
	dst->hi = MAKE_UQWRDHI(u64Dst) ;
	dst->lo = MAKE_UQWRDLO(u64Dst) ;
}

//***************************************************************************
// s64bit = s64bit - s32bit

void _sint64_sub_64_32( SQWRD * dst, const SQWRD * op1, const SLONG * op2 )
{
	SLLNG s64Dst = (MAKE_INT64(op1->hi, op1->lo)) - (SLLNG)(*op2) ;
	dst->hi = MAKE_SQWRDHI(s64Dst) ;
	dst->lo = MAKE_SQWRDLO(s64Dst) ;
}

//***************************************************************************
// u64bit = u64bit - u32bit

void _uint64_sub_64_32( UQWRD * dst, const UQWRD * op1, const ULONG * op2 )
{
	ULLNG u64Dst = (MAKE_UINT64(op1->hi, op1->lo)) - (ULLNG)(*op2) ;
	dst->hi = MAKE_UQWRDHI(u64Dst) ;
	dst->lo = MAKE_UQWRDLO(u64Dst) ;
}

//***************************************************************************
// s64bit = s32bit - s32bit

void _sint64_sub_32_32( SQWRD * dst, const SLONG * op1, const SLONG * op2 )
{
	SLLNG s64Dst = (SLLNG)(*op1) - (SLLNG)(*op2) ;
	dst->hi = MAKE_SQWRDHI(s64Dst) ;
	dst->lo = MAKE_SQWRDLO(s64Dst) ;
}

//***************************************************************************
// u64bit = u32bit - u32bit

void _uint64_sub_32_32( UQWRD * dst, const ULONG * op1, const ULONG * op2 )
{
	ULLNG u64Dst = (ULLNG)(*op1) - (ULLNG)(*op2) ;
	dst->hi = MAKE_UQWRDHI(u64Dst) ;
	dst->lo = MAKE_UQWRDLO(u64Dst) ;
}

//***************************************************************************
// s64bit = s32bit * s32bit

void _sint64_mul_32_32( SQWRD * dst, const SLONG * op1 , const SLONG * op2 )
{
	SLLNG s64Dst = ((SLLNG)(*op1)) * ((SLLNG)(*op2)) ;
	dst->hi = MAKE_SQWRDHI(s64Dst) ;
	dst->lo = MAKE_SQWRDLO(s64Dst) ;
}

//***************************************************************************
// u64bit = u32bit * u32bit

void _uint64_mul_32_32( UQWRD * dst, const ULONG * op1, const ULONG * op2 )
{
	ULLNG u64Dst = ((ULLNG)(*op1)) * ((ULLNG)(*op2)) ;
	dst->hi = MAKE_UQWRDHI(u64Dst) ;
	dst->lo = MAKE_UQWRDLO(u64Dst) ;
}

//***************************************************************************
// s64bit = s32bit * s16bit

void _sint64_mul_32_16( SQWRD * dst, const SLONG * op1, const SWORD * op2 )
{
	SLLNG s64Dst = ((SLLNG)(*op1)) * ((SLLNG)(*op2)) ;
	dst->hi = MAKE_SQWRDHI(s64Dst) ;
	dst->lo = MAKE_SQWRDLO(s64Dst) ;
}

//***************************************************************************
// s64bit = s48bit * s16bit

void _sint64_mul_48_16( SQWRD * dst, const SQWRD * op1, const SWORD * op2 )
{
#ifdef _CRS_64FUNCFLOAT_
	// needed to create single variables, otherwise compiler
	// create a mess with float cast to macro result
	SLLNG   s64_op1 = MAKE_INT64(op1->hi, op1->lo) ;
	FLOAT   flDst = (FLOAT)s64_op1 * (FLOAT)(*op2) ; // use double?

	// saturate to avoid overflow
	if (flDst >= INT64_MAX_VALUE)
		flDst = INT64_MAX_VALUE ;
	else if (flDst <= INT64_MIN_VALUE)
		flDst = INT64_MIN_VALUE ;

	dst->hi = MAKE_SQWRDHI((SLLNG)flDst) ;
	dst->lo = MAKE_SQWRDLO((SLLNG)flDst) ;
#else
	SLLNG s64Dst = (MAKE_INT64(op1->hi, op1->lo)) * ((SLLNG)(*op2)) ;
	dst->hi = MAKE_SQWRDHI(s64Dst) ;
	dst->lo = MAKE_SQWRDLO(s64Dst) ;
#endif

}

//***************************************************************************
// op1>op2 => +1, op1==op2 => 0, op1<op2 => -1

SBYTE _sint64_comp( SQWRD * op1, SQWRD * op2 )
{
  if (op1->hi > op2->hi)
    return 1 ;                /* op1 > op2 */
  else if (op1->hi < op2->hi)
    return -1 ;               /* op1 < op2 */
  else if (op1->lo > op2->lo) /* op1.hi = op2.hi => check low part */
    return 1 ;                /* op1 > op2 */
  else if (op1->lo < op2->lo)
    return -1 ;               /* op1 < op2 */
  else
    return 0 ;                /* op1 = op2 */
}

//***************************************************************************
// Min selection

//void _sint64_min( SQWRD * dst, const SQWRD * op1)
//{
//    __asm
//    {
//        mov     r1,[dst+]
//        mov     r2,[dst+]
//        mov     r3,[dst+]
//        mov     r4,[dst+]
//
//        mov     r5,[op1+]
//        mov     r6,[op1+]
//        mov     r11,[op1+]
//        mov     r12,[op1+]
//
//        coload  r3,r4
//        cocmp   r11,r12
//
//        jb      MSW_MN, cmpend
//
//        jb      MSW_MZ, cmplo
//
//        mov     [-dst],r12
//        mov     [-dst],r11
//        mov     [-dst],r6
//        mov     [-dst],r5
//        _ret
//
//cmplo:
//        sub     dst, #0x08
//
//        coload  r1,r2
//        comin   r5,r6
//
//        costore [dst+],mal
//        costore [dst+],mah
//        coload  r3,r4
//        costore [dst+],mal
//        costore [dst+],mah
//
//cmpend:
//        _ret
//    }
//}

//***************************************************************************
// Max selection

//void _sint64_max( SQWRD * dst, const SQWRD * op1)
//{
//    __asm
//    {
//        mov     r1,[dst+]
//        mov     r2,[dst+]
//        mov     r3,[dst+]
//        mov     r4,[dst+]
//
//        mov     r5,[op1+]
//        mov     r6,[op1+]
//        mov     r11,[op1+]
//        mov     r12,[op1+]
//
//        coload  r3,r4
//        cocmp   r11,r12
//
//        jnb     MSW_MN, cmpend
//
//        jb      MSW_MZ, cmplo
//
//        mov     [-dst],r12
//        mov     [-dst],r11
//        mov     [-dst],r6
//        mov     [-dst],r5
//        _ret
//
//cmplo:
//        sub     dst, #0x08
//
//        coload  r1,r2
//        comax   r5,r6
//
//        costore [dst+],mal
//        costore [dst+],mah
//        coload  r3,r4
//        costore [dst+],mal
//        costore [dst+],mah
//
//cmpend:
//        _ret
//    }
//}

//***************************************************************************
// s64bit => s32bit clipping (+/-)(2^31) - 1

SLONG _sint64toSint32(SQWRD * sqVar)
{
	SLLNG s64Dst = MAKE_INT64(sqVar->hi, sqVar->lo) ;

	if (s64Dst >= (SLLNG)SLONG_MAX_VALUE)
		return SLONG_MAX_VALUE ;
	else if (s64Dst <= (SLLNG)SLONG_MIN_VALUE)
		return SLONG_MIN_VALUE ;
	else
		return (SLONG)s64Dst ;
}

//***************************************************************************
// s32bit => s64bit
//
//void _sint32toSint64(SQWRD * dst, SLONG * src )
//{
//    __asm
//    {
//        mov     r2,[src+]
//        coload  r2,[src+]
//
//        costore [dst+],mal
//        costore [dst+],mah
//
//        coashr  #16
//        costore [dst+],mah
//        costore [dst+],mah
//    }
//}

//***************************************************************************
// s64bit = s64bit << u16bit

void _sint64_shl( SQWRD * var, UWORD cnt )
{
	if (cnt != 0)
	{
#ifdef _CRS_64FUNCFLOAT_
		// needed to create single variables, otherwise compiler
		// create a mess with float cast to macro result
		SLLNG   s64_op1 = MAKE_INT64(var->hi, var->lo) ;
		FLOAT   flDst = ((FLOAT)s64_op1) * pow(2.0, cnt) ; // use double?

		// saturate to avoid overflow
		if (flDst >= INT64_MAX_VALUE)
			flDst = INT64_MAX_VALUE ;
		else if (flDst <= INT64_MIN_VALUE)
			flDst = INT64_MIN_VALUE ;

		var->hi = MAKE_SQWRDHI((SLLNG)flDst) ;
		var->lo = MAKE_SQWRDLO((SLLNG)flDst) ;
#else
		SLLNG s64Dst = (SLLNG)(MAKE_INT64(var->hi, var->lo)) << cnt ;
		var->hi = MAKE_SQWRDHI(s64Dst) ;
		var->lo = MAKE_SQWRDLO(s64Dst) ;
#endif
	}
	else
		return ;
}

//***************************************************************************
// s64bit = s64bit >> u16bit

void _sint64_shr( SQWRD * var, UWORD cnt )
{
	if (cnt != 0)
	{
		SLLNG s64Dst = (SLLNG)(MAKE_INT64(var->hi, var->lo)) >> cnt ;
	
		var->hi = MAKE_SQWRDHI(s64Dst) ;
		var->lo = MAKE_SQWRDLO(s64Dst) ;
	}
	else
		return ;
}

//***************************************************************************
// u64bit = u64bit >> u16bit

void _uint64_shr( UQWRD * var, UWORD cnt )
{
	if (cnt != 0)
	{
		ULLNG u64Dst = (ULLNG)(MAKE_UINT64(var->hi, var->lo)) >> cnt ;
	
		var->hi = MAKE_UQWRDHI(u64Dst) ;
		var->lo = MAKE_UQWRDLO(u64Dst) ;
	}
	else
		return ;
}

//***************************************************************************
// u64bit = u64bit << u16bit

void _uint64_shl( UQWRD * var, UWORD cnt )
{
	if (cnt != 0)
	{
		ULLNG u64Dst = (ULLNG)(MAKE_UINT64(var->hi, var->lo)) << cnt ;
	
		var->hi = MAKE_UQWRDHI(u64Dst) ;
		var->lo = MAKE_UQWRDLO(u64Dst) ;
	}
	else
		return ;
}

//***************************************************************************
//
// sqIn64  |--------| sqOut64
// ------->| 64->48 |--------->
//         |--------|
//

void _sint64toSint48(SQWRD *sqVar)
{
  if (sqVar->hi >= 32767l)
  { /* positive number: clamp to 0x00007fff_ffffffff */
    sqVar->hi = 32767l  ;
    sqVar->lo = (ULONG)0xffffffffUL ;
  }
  else if (sqVar->hi <= -32768l)
  { /* negative number: clamp to 0xffff8000_00000001 */
    sqVar->hi = -32768l  ;
    sqVar->lo = (ULONG)0x00000001UL ;
  }
}

//***************************************************************************
// s64bit = s64bit * s32bit/Q16

void _sint64p_scale_32( SQWRD  * val, SLONG scale )
{
#ifdef _CRS_64FUNCFLOAT_
	// needed to create single variables, otherwise compiler
	// create a mess with float cast to macro result
	SLLNG   s64_op1 = MAKE_INT64(val->hi, val->lo) ;
	FLOAT   flDst = (FLOAT)s64_op1 * (FLOAT)scale / 65536.0 ; // use double?

	// saturate to avoid overflow
	if (flDst >= INT64_MAX_VALUE)
		flDst = INT64_MAX_VALUE ;
	else if (flDst <= INT64_MIN_VALUE)
		flDst = INT64_MIN_VALUE ;

	val->hi = MAKE_SQWRDHI((SLLNG)flDst) ;
	val->lo = MAKE_SQWRDLO((SLLNG)flDst) ;
#else
	SLLNG s64Dst = (SLLNG)((MAKE_INT64(val->hi, val->lo)) * scale) >> 16 ;
	val->hi = MAKE_SQWRDHI(s64Dst) ;
	val->lo = MAKE_SQWRDLO(s64Dst) ;
#endif
}

//***************************************************************************
// s64bit = s64bit + s64bit WITHOUT SATURATION

void _sint64p_add( SQWRD  * val, ULONG lo, SLONG hi )
{
	SLLNG s64Dst = (MAKE_INT64(val->hi, val->lo)) + (MAKE_INT64(hi, lo));

	val->hi = MAKE_SQWRDHI(s64Dst) ;
	val->lo = MAKE_SQWRDLO(s64Dst) ;
}

//***************************************************************************
// s64bit = s64bit - s64bit WITHOUT SATURATION

void _sint64p_sub( SQWRD  * val, ULONG lo, SLONG hi )
{
	SLLNG s64Dst = (MAKE_INT64(val->hi, val->lo)) - (MAKE_INT64(hi, lo));

	val->hi = MAKE_SQWRDHI(s64Dst) ;
	val->lo = MAKE_SQWRDLO(s64Dst) ;
}

//***************************************************************************
// s64bit = s64bit - s64bit WITHOUT SATURATION

void _sint64_sub_nosaturation( SQWRD * dst, const SQWRD * op1, const SQWRD * op2 )
{
	SLLNG s64Dst = (MAKE_INT64(op1->hi, op1->lo)) - (MAKE_INT64(op2->hi, op2->lo));

	dst->hi = MAKE_SQWRDHI(s64Dst) ;
	dst->lo = MAKE_SQWRDLO(s64Dst) ;
}
