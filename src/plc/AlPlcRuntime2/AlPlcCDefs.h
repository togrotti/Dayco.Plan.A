/*------------------------------------------------------------------------------
**
**	Copyright:			AXEL s.r.l. 2010
**
**	AlPlcCDefs.h:		Specific architecture definitions
**
**-----------------------------------------------------------------------------
**
**	IMPORTANT:
**	THIS MODULE SHOULDN'T BE MODIFIED BY THE CUSTOMER
**
**------------------------------------------------------------------------------*/

#ifndef _PLCCCDEFS_H
#define _PLCCCDEFS_H

#ifdef __cplusplus
extern "C" {
#endif

#if ! defined( ALPLC_P_ARM9 ) && ! defined( ALPLC_P_ARM_V7_M ) && ! defined( ALPLC_P_COLDFIRE ) && ! defined( ALPLC_P_X86 ) && ! defined( ALPLC_P_16FX ) && ! defined( ALPLC_P_C166 )
#error "Define the processor type between one of the following: ALPLC_P_ARM9 (ARM9 compatible core), ALPLC_P_ARM_V7_M (ARM Architecture v7-M), ALPLC_P_COLDFIRE (Freescale ColdFire), ALPLC_P_X86 (X86 compatible core), ALPLC_P_16FX (Fujitsu 16FX mcu), ALPLC_P_C166 (Infineon C166 Family)"
#endif

#if defined( ALPLC_P_ARM9 ) || defined( ALPLC_P_ARM_V7_M )
#if ! defined( ALPLC_C_GCCARM9 ) && ! defined( ALPLC_C_MSDEVARM )
#error "Define the compiler between one of the following: ALPLC_C_GCCARM9 (GCC ARM C compiler), ALPLC_C_MSDEVARM (Microsoft ARM C compiler)"
#endif	/* ! defined( ALPLC_C_GCCARM9 ) && ! defined( ALPLC_C_MSDEVARM ) */
#endif	/* defined( ALPLC_P_ARM9 ) || defined( ALPLC_P_ARM_V7_M ) */

#if defined( ALPLC_P_COLDFIRE )
#if ! defined( ALPLC_C_CODEWARRIOR )
#error "Define the compiler between one of the following: ALPLC_C_CODEWARRIOR (Freescale CodeWarrior compiler for Freescale ColdFire microprocessors)"
#endif	/* ! defined( ALPLC_C_CODEWARRIOR ) */
#endif	/* defined( ALPLC_P_COLDFIRE ) */

#if defined( ALPLC_P_X86 )
#if ! defined( ALPLC_C_MSDEVX86 ) && ! defined( ALPLC_C_GCCX86 )
#error "Define the compiler between one of the following: ALPLC_C_MSDEVX86 (Microsoft X86 C compiler), ALPLC_C_GCCX86 (GCC X86 C compiler)"
#endif	/* ! defined( ALPLC_C_MSDEVX86 ) */
#endif	/* defined( ALPLC_P_X86 ) */

#if defined( ALPLC_P_16FX )
#if ! defined( ALPLC_C_SOFTUNE16FX )
#error "Define the compiler between one of the following: ALPLC_C_SOFTUNE16FX (Fujitsu Softune 16FX C compiler)"
#endif	/* ! defined( ALPLC_C_SOFTUNE16FX ) */
#endif	/* defined( ALPLC_P_16FX ) */

#if defined( ALPLC_P_C166 )
#if ! defined( ALPLC_C_C166K )
#error "Define the compiler between one of the following: ALPLC_C_C166K (Keil Infineon C166 Compile)"
#endif	/* ! defined( ALPLC_C_C166K ) */
#endif	/* defined( ALPLC_P_C166 ) */

#if ! defined( EXTERNAL_MISRA_C_TYPES )
#include "AlPlcMisraC.h"
#endif

/*	Structure bit field data type	*/
#if defined( ALPLC_P_ARM9 ) || defined( ALPLC_P_ARM_V7_M ) || defined( ALPLC_P_COLDFIRE ) || defined( ALPLC_P_X86 ) || defined( ALPLC_P_16FX )
#define BIT_FIELD		uint32_t	/* defined( ALPLC_P_ARM9 ) || defined( ALPLC_P_COLDFIRE ) || defined( ALPLC_P_X86 ) || defined( ALPLC_P_16FX ) */
#elif defined( ALPLC_P_C166 )	/* defined( ALPLC_P_ARM9 ) || defined( ALPLC_P_COLDFIRE ) || defined( ALPLC_P_X86 ) || defined( ALPLC_P_16FX ) */
#endif	/* defined( ALPLC_P_C166 ) */

/* Check architecture-specific data alignment requirements */
#if defined( ALPLC_P_ARM9 ) || defined( ALPLC_P_ARM_V7_M )
/*
**	Data block address must be aligned to a 4-byte boundary, so that the
**	compiler is able to check the alignment of mapped variables with respect
**	to the ARM Architecture's requirements. (Since version 2.17, LogicLab does
**	check the alignment of mapped variables by assuming the address of any data
**	block to be multiple of 4).
**	
**	Please note that the ARM Architecture requires 64-bit data type (e.g.,
**	the C programming language double data type or its IEC 61131-3 equivalent:
**	LREAL) to be aligned to a 8-byte boundary. Until the IEC 61131-3
**	implementation for the ARM Architecture provided by LogicLab is extended to
**	include support for 64-bit data types, data blocks won't be required to be
**	aligned to a 8-byte boundary.
*/
#define CHECK_DATA_ALIGNMENT( address, size )	( ( ( address ) % ( size ) ) == 0 )
#endif	/* defined( ALPLC_P_ARM9 ) */

/*--------------------------------------*/
/*										*/
/*	Utility macros and main constants	*/
/*										*/
/*--------------------------------------*/

/*  Array elements counting */
#ifndef ARRSIZE
#define ARRSIZE(a)  	(sizeof(a)/sizeof(a[0]))
#endif

/*  Single array element size   */
#ifndef ARRELSIZE
#define ARRELSIZE(a)	sizeof(a[0])
#endif

/*	Offset of structure element	*/
#define OFFSETOF(s,m)   	(uint32_t)&(((s*)0)->m)

/*  Word composition/decomposition utilities	*/
#ifndef MAKE_WORD
#define MAKE_WORD(lo,hi)	((uint16_t)(lo) + ((uint16_t)(hi)<<8))
#endif

#ifndef LO_BYTE
#define LO_BYTE(b)  		(int8_t)((b) & 0xFF )
#endif

#ifndef HI_BYTE
#define HI_BYTE(b)  		(int8_t)(((uint16_t)(b) >> 8 ) & 0xFF )
#endif

#ifndef LO_WORD
#define LO_WORD(w)  		(int16_t)((w) & 0xFFFF )
#endif

#ifndef HI_WORD
#define HI_WORD(w)  		(int16_t)(((uint32_t)(w) >> 16 ) & 0xFFFF )
#endif

/*	Boolean values	*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

/*	NULL pointer	*/
#ifndef NULL
#define NULL 0
#endif

/*------------------------------------------------------*/
/*														*/
/*	Optional C compiler directives for qualify memory	*/
/*	areas and alignments								*/
/*														*/
/*------------------------------------------------------*/


#if defined( ALPLC_C_GCCARM9 ) || defined( ALPLC_C_MSDEVX86 ) || defined( ALPLC_C_CODEWARRIOR ) || defined( ALPLC_C_GCCX86 ) || defined( ALPLC_C_MSDEVARM )

#define PLC_ATTR_CODE										/* Memory attribute for code area (ex. far, near etc.)	*/
#define PLC_ATTR_DATA										/* Memory attribute for data area (ex. far, near etc.)	*/
#define PLC_ATTR_BITDATA									/* Memory attribute for bita data area (ex. far, near etc.)	*/
#define PLC_ATTR_RETAINDATA									/* Memory attribute for retain data area (ex. far, near etc.)	*/
#define PLC_ATTR_SOURCE										/* Memory attribute for source code area (ex. far, near etc.)	*/
#define PLC_ATTR_CONST( type ) 			const type			/* Memory attribute for const data	*/
#define PLC_ATTR_CONST_PTR( type )		const type*			/* Memory attribute for pointer to const data	*/
#define PLC_ATTR_NON_CONST( type ) 		type				/* Memory attribute for non const data	*/
#define PLC_ATTR_NON_CONST_PTR( type )	type*				/* Memory attribute for pointer to non const data	*/
#define PLC_ATTR_GENERIC_PTR( type )	const type*			/* Memory attribute for pointer to data that can be either const or not const	*/

#elif defined( ALPLC_C_SOFTUNE16FX )

#define PLC_ATTR_CODE					__far				/* Memory attribute for code area (ex. far, near etc.)	*/
#define PLC_ATTR_DATA					__far				/* Memory attribute for data area (ex. far, near etc.)	*/
#define PLC_ATTR_BITDATA				__far				/* Memory attribute for bita data area (ex. far, near etc.)	*/
#define PLC_ATTR_RETAINDATA				__far				/* Memory attribute for retain data area (ex. far, near etc.)	*/
#define PLC_ATTR_SOURCE					__far				/* Memory attribute for source code area (ex. far, near etc.)	*/
#define PLC_ATTR_CONST( type ) 			const __far type	/* Memory attribute for const data	*/
#define PLC_ATTR_CONST_PTR( type )		const __far type *	/* Memory attribute for pointer to const data	*/
#define PLC_ATTR_NON_CONST( type ) 		__far type 	/*huge*//* Memory attribute for non const data	*/
#define PLC_ATTR_NON_CONST_PTR( type )	__far type *		/* Memory attribute for pointer to non const data	*/
#define PLC_ATTR_GENERIC_PTR( type )	const __far type *	/* Memory attribute for pointer to data that can be either const or not const	*/

#elif defined( ALPLC_C_C166K )

#define PLC_ATTR_CODE					far					/* Memory attribute for code area (ex. far, near etc.)	*/
#define PLC_ATTR_DATA					huge				/* Memory attribute for data area (ex. far, near etc.)	*/
#define PLC_ATTR_BITDATA				huge				/* Memory attribute for bita data area (ex. far, near etc.)	*/
#define PLC_ATTR_RETAINDATA				huge				/* Memory attribute for retain data area (ex. far, near etc.)	*/
#define PLC_ATTR_SOURCE					far					/* Memory attribute for source code area (ex. far, near etc.)	*/
#define PLC_ATTR_CONST( type ) 			const type huge		/* Memory attribute for const data	*/
#define PLC_ATTR_CONST_PTR( type )		const type huge *	/* Memory attribute for pointer to const data	*/
#define PLC_ATTR_NON_CONST( type ) 		type huge			/* Memory attribute for non const data	*/
#define PLC_ATTR_NON_CONST_PTR( type )	type huge *  		/* Memory attribute for pointer to non const data	*/
#define PLC_ATTR_GENERIC_PTR( type )	const type huge *	/* Memory attribute for pointer to data that can be either const or not const	*/

#define STATIC_VAR_ADDRESS				void PLC_ATTR_CODE *	/* Pointer to static variable address */

#endif	/* defined( ALPLC_C_MSDEVX86 ) || defined( ALPLC_C_CODEWARRIOR ) || defined( ALPLC_C_SOFTUNE16FX ) || defined( ALPLC_C_MSDEVARM ) */

#if defined( ALPLC_C_C166K )
#define PLC_TASK_PTR void PLC_ATTR_DATA **
#else
#define PLC_TASK_PTR void PLC_ATTR_CODE **
#endif

#if defined( ALPLC_P_ARM_V7_M )
#define PLC_CALCCODEADDR( x )			( void PLC_ATTR_CODE* )( (x) + 1 )
#else
#define PLC_CALCCODEADDR( x )			( void PLC_ATTR_CODE* )(x)
#endif	/* defined( ALPLC_P_ARM_V7_M ) */

/* Packing of C structures */
#if defined( ALPLC_C_MSDEVX86 ) || defined( ALPLC_C_MSDEVARM ) || defined( ALPLC_C_C166K )

#define _C_PACK_											/*  Compiler packing directive (near structure definition)  */
#define _C_START_PACK_PRAGMA	pack(1)						/*  Compiler packing pragma */
#define _C_END_PACK_PRAGMA		pack()						/*  Compiler packing pragma */

#elif defined( ALPLC_C_GCCARM9 ) || defined( ALPLC_C_GCCX86 )

#define _C_PACK_  				__attribute__((packed))		/*	Compiler packing directive (near structure definition)	*/
/*#define _C_START_PACK_PRAGMA							  	Not needed with GCC */
/*#define _C_END_PACK_PRAGMA								Not needed with GCC */

#elif defined( ALPLC_C_CODEWARRIOR )

#define _C_PACK_											/*  Compiler packing directive (near structure definition)  */
#define _C_START_PACK_PRAGMA	options align= packed		/*  Compiler packing pragma */
#define _C_END_PACK_PRAGMA		options align= reset		/*  Compiler packing pragma */

#elif defined( ALPLC_C_SOFTUNE16FX )

#define _C_PACK_											/*  Compiler packing directive (near structure definition)  */
#define _C_START_PACK_PRAGMA								/*  Compiler packing pragma */
#define _C_END_PACK_PRAGMA									/*  Compiler packing pragma */

#endif

#ifdef __cplusplus
}
#endif

#endif	/* _PLCCCDEFS_H */

