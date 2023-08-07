/*------------------------------------------------------------------------------
**
**	Copyright (C) 2009-2016 Axel S.r.l
**
**	Architecture-specific configuration
**
**------------------------------------------------------------------------------
**
**	IMPORTANT:
**	IN ORDER TO ALLOW SMOOTH UPGRADES TO BUG-FIXING RELEASES OF THIS PLC
**	RUN-TIME VERSION (AND REASONABLY MANAGEABLE UPGRADES TO FUTURE PLC RUN-TIME
**	VERSIONS) IT IS OF THE UTTERMOST IMPORTANCE NOT TO MODIFY THIS FILE IN ANY
**	OF ITS PART!
**
**----------------------------------------------------------------------------*/

#ifndef AL_PLC_C_DEFS_H_
#define AL_PLC_C_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif	/* __cplusplus */

// defined for compatibility with older releases of logic lab
#ifdef ALPLC_P_ARM9
#define ALPLC_P_ARM32
#endif
#ifdef ALPLC_P_ARM_V7_M
#define ALPLC_P_ARM_THUMB2
#endif

// defined for compatibility with obsolete definitions in older runtime versions
#if defined(ALPLC_C_GCCARM9) || defined(ALPLC_C_GCC_M4F)
#define ALPLC_C_GCCARM
#endif

#if ! defined( ALPLC_P_ARM32 ) && ! defined( ALPLC_P_ARM_THUMB2 ) && ! defined( ALPLC_P_ARM32VFP2 ) && ! defined( ALPLC_P_ARM_THUMB2VFP2 ) && !defined( ALPLC_P_ARM32VFP2_SP ) && !defined( ALPLC_P_ARM_THUMB2VFP2_SP ) &&! defined( ALPLC_P_COLDFIRE ) && ! defined( ALPLC_P_X86 ) && ! defined( ALPLC_P_X64 ) && ! defined( ALPLC_P_16FX ) && ! defined( ALPLC_P_C166 ) && ! defined(ALPLC_P_POWERPC) && !defined( ALPLC_P_POWERPCE200) && ! defined( ALPLC_P_TIDSP ) && ! defined( ALPLC_P_FSL_DSP56800E ) && ! defined( ALPLC_P_RENESASRX ) && !defined(ALPLC_P_TRICORE) && !defined(ALPLC_P_ARM64)
#error "Define the processor type between one of the following: ALPLC_P_ARM32 (ARM compatible core), ALPLC_P_ARM_THUMB2 (ARM Thumb2 Architecture), ALPLC_P_ARM32VFP2 (ARM compatible core with full VFP2), ALPLC_P_ARM_THUMB2VFP2 (ARM Thumb2 Architecture with full VFP2), ALPLC_P_ARM32VFP2_SP (ARM compatible core with single precision VFP2), ALPLC_P_ARM_THUMB2VFP2_SP (ARM Thumb2 Architecture with single precision VFP2), ALPLC_P_COLDFIRE (Freescale ColdFire), ALPLC_P_X86 (X86 compatible core), ALPLC_P_16FX (Fujitsu 16FX mcu), ALPLC_P_C166 (Infineon C166 Family), ALPLC_P_POWERPC (Power Pc Architecture), ALPLC_P_POWERPCE200 (Power Pc E200 Architecture), ALPLC_P_TIDSP (Texas Instr. TMS320xx), ALPLC_P_FSL_DSP56800E (Freescale DSP56800E core), ALPLC_P_RENESASRX (Renesas RX), ALPLC_P_TRICORE (Infineon TriCore), ALPLC_P_ARM64 (ARM 64bit)"
#endif

#if defined( ALPLC_P_ARM32 ) || defined( ALPLC_P_ARM_THUMB2 ) || defined( ALPLC_P_ARM32VFP2 ) || defined( ALPLC_P_ARM_THUMB2VFP2 ) || defined( ALPLC_P_ARM32VFP2_SP ) || defined( ALPLC_P_ARM_THUMB2VFP2_SP )
#if ! defined( ALPLC_C_GCCARM ) && ! defined( ALPLC_C_MSDEVARM ) && ! defined( ALPLC_C_MIKROC_M4F ) && ! defined( ALPLC_C_IAR_ARM )
#error "Define the compiler between one of the following: ALPLC_C_GCCARM (GCC ARM C compiler), ALPLC_C_MSDEVARM (Microsoft ARM C compiler), ALPLC_C_MIKROC_M4F (MikroC PRO ARM C compiler), ALPLC_C_IAR_ARM (IAR Compiler for ARM)"
#endif	/* ! defined( ALPLC_C_GCCARM ) && ! defined( ALPLC_C_MSDEVARM ) && ! defined( ALPLC_C_MIKROC_M4F ) && ! defined( ALPLC_C_IAR_ARM ) */
#endif	/* defined( ALPLC_P_ARM32 ) || defined( ALPLC_P_ARM_THUMB2 ) || defined( ALPLC_P_ARM32VFP2 ) || defined( ALPLC_P_ARM_THUMB2VFP2 ) || defined( ALPLC_P_ARM32VFP2_SP ) || defined( ALPLC_P_ARM_THUMB2VFP2_SP ) */

#if defined( ALPLC_P_COLDFIRE )
#if ! defined( ALPLC_C_CODEWARRIOR ) && ! defined( ALPLC_C_GCCCF )
#error "Define the compiler between one of the following: ALPLC_C_CODEWARRIOR (Freescale CodeWarrior compiler for Freescale ColdFire microprocessors), ALPLC_C_GCCCF (GCC for ColdFire)"
#endif	/* ! defined( ALPLC_C_CODEWARRIOR ) */
#endif	/* defined( ALPLC_P_COLDFIRE ) */

#if defined( ALPLC_P_X86 ) || defined( ALPLC_P_X64 )
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

#if defined( ALPLC_P_POWERPC ) || defined( ALPLC_P_POWERPCE200 )
#if ! defined( ALPLC_C_PXE ) && ! defined( ALPLC_C_GCCPPC ) && ! defined( ALPLC_C_DIABPPC )
#error "Define the compiler between one of the following: ALPLC_C_PXE (Power PC Compiler), ALPLC_C_GCCPPC (GCC for PowerPC), ALPLC_C_DIABPPC (WindRiver Diab compiler for PowerPC)"
#endif	/* ! defined( ALPLC_C_PXE ) */
#endif	/* defined( ALPLC_P_POWERPC ) || defined( ALPLC_P_POWERPCE200 ) */

#if defined( ALPLC_P_TIDSP )
#if ! defined( ALPLC_C_CODECOMP ) &&  ! defined( ALPLC_C_CODECOMP_BIT )
#error "Define the compiler between one of the following: ALPLC_C_CODECOMP (Code Composer compiler) ALPLC_C_CODECOMP (Code Composer compiler)"
#endif	/* ! defined( ALPLC_C_CODECOMP ) */
#endif	/* defined( ALPLC_P_TIDSP ) */

#if defined( ALPLC_P_FSL_DSP56800E )
#if ! defined( ALPLC_C_CW_DSP56800E  )
#error "Define the compiler between one of the following: ALPLC_C_CW_DSP56800E  (CodeWarrior for DSP56800E)"
#endif
#endif

#if defined( ALPLC_P_RENESASRX )
#if ! defined( ALPLC_C_CCRX  )
#error "Define the compiler between one of the following: ALPLC_C_CCRX  (IAR for Renesas RX)"
#endif
#endif

#if defined(ALPLC_P_TRICORE)
#if !defined(ALPLC_C_TASKING)
#error "Define the compiler between one of the following: ALPLC_C_TASKING (Tasking for TriCore)"
#endif
#endif

#if defined(ALPLC_P_ARM64)
#if !defined(ALPLC_C_GCCARM)
#error "Define the compiler between one of the following: ALPLC_C_GCCARM (GCC for ARM)"
#endif
#endif

#include "AlPlcMisraC.h"

/*	Memory address size: put here all the supported 64bits platforms	*/
#if defined(ALPLC_P_X64) || defined(ALPLC_P_ARM64)
	#define ALPLC_64BIT_ADDRESSES 1
	typedef uint64_t addr_t;
#else
	#define ALPLC_64BIT_ADDRESSES 0
	typedef uint32_t addr_t;
#endif

/*	Structure bit field data type	*/
#if defined( ALPLC_P_ARM32 ) || defined( ALPLC_P_ARM_THUMB2 ) || defined( ALPLC_P_ARM32VFP2 ) || defined( ALPLC_P_ARM_THUMB2VFP2 ) || defined( ALPLC_P_ARM32VFP2_SP ) || defined( ALPLC_P_ARM_THUMB2VFP2_SP ) || defined( ALPLC_P_COLDFIRE ) || defined( ALPLC_P_X86 ) || defined( ALPLC_P_X64 ) || defined( ALPLC_P_16FX ) || defined( ALPLC_P_POWERPC ) || defined( ALPLC_P_POWERPCE200 ) || defined( ALPLC_P_RENESASRX ) || defined( ALPLC_P_TRICORE ) || defined(ALPLC_P_ARM64)
#define BIT_FIELD		uint32_t	/* defined( ALPLC_P_ARM32 ) || defined( ALPLC_P_COLDFIRE ) || defined( ALPLC_P_X86 ) || defined( ALPLC_P_X64 ) || defined( ALPLC_P_16FX ) */
#elif defined( ALPLC_P_C166 )	/* defined( ALPLC_P_ARM32 ) || defined( ALPLC_P_COLDFIRE ) || defined( ALPLC_P_X86 ) || defined( ALPLC_P_16FX ) */
#endif	/* defined( ALPLC_P_C166 ) */

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
#define OFFSETOF(s,m)   	(addr_t)((uint8_t*) &(((s*)0)->m))

/*  Word composition/decomposition utilities	*/
#ifndef MAKE_WORD
#define MAKE_WORD(lo,hi)			((uint16_t)(lo) + ((uint16_t)(hi)<<8))
#endif

#ifndef MAKE_DWORD
#define MAKE_DWORD(wlo,whi)			((uint32_t)(wlo) + ((uint32_t)(whi)<<16))
#endif

#ifndef MAKE_DWORD_B
#define MAKE_DWORD_B(b0,b1,b2,b3)	((uint32_t)(b0) + ((uint32_t)(b1)<<8) + ((uint32_t)(b2)<<16) + ((uint32_t)(b3)<<24))
#endif

#ifndef LO_BYTE
#define LO_BYTE(b)  				(uint8_t)((b) & 0xFF )
#endif

#ifndef HI_BYTE
#define HI_BYTE(b)  				(uint8_t)(((uint16_t)(b) >> 8 ) & 0xFF )
#endif

#ifndef LO_WORD
#define LO_WORD(w)  				(uint16_t)((w) & 0xFFFF )
#endif

#ifndef HI_WORD
#define HI_WORD(w)  				(uint16_t)(((uint32_t)(w) >> 16 ) & 0xFFFF )
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

/*	Rende true se è presente la maschera di bit M nella word W */
#ifndef MASK
#define	MASK(w,m)			(( (w) & (m) ) == (m) )
#endif

/*------------------------------------------------------*/
/*														*/
/*	Optional C compiler directives for qualify memory	*/
/*	areas and alignments								*/
/*														*/
/*------------------------------------------------------*/


#if defined( ALPLC_C_MIKROC_M4F ) || defined( ALPLC_C_GCCARM ) || defined( ALPLC_C_MSDEVX86 ) || defined( ALPLC_C_CODEWARRIOR ) || defined( ALPLC_C_GCCX86 ) || defined( ALPLC_C_MSDEVARM ) || defined( ALPLC_C_PXE ) || defined( ALPLC_C_CODECOMP ) || defined( ALPLC_C_CODECOMP_BIT ) || defined( ALPLC_C_CW_DSP56800E ) || defined( ALPLC_C_IAR_ARM ) || defined(ALPLC_C_GCCPPC) || defined(ALPLC_C_GCCCF) || defined( ALPLC_C_CCRX ) || defined( ALPLC_C_DIABPPC ) || defined( ALPLC_C_TASKING )

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

/* Dynamic allocation, reallocation, and deallocation of PLC code areas */
#define PLC_M_ALLOC_CODE( dst, sz )			{ dst = malloc( sz ); }
#define PLC_M_REALLOC_CODE( ptr, sz )		{ ptr = realloc( ptr, sz ); }
#define PLC_M_FREE_CODE( ptr )				{ free( ptr ); }

/* Dynamic allocation, reallocation, and deallocation of PLC data area */
#define PLC_M_ALLOC_DATA( dst, sz )			{ dst = malloc( sz ); }
#define PLC_M_REALLOC_DATA( ptr, sz )		{ ptr = realloc( ptr, sz ); }
#define PLC_M_FREE_DATA( ptr )				{ free( ptr ); }
#define PLC_M_COPY_DATA( dst, src, sz )		{ memcpy( dst, src, sz ); }

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

/*
 *	IMPORTANT:
 *	DYNAMIC ALLOCATION, REALLOCATION, AND DEALLOCATION OF PLC AREAS IS NOT
 *	IMPLEMENTED YET!
 */

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

/*
 *	IMPORTANT:
 *	DYNAMIC ALLOCATION, REALLOCATION, AND DEALLOCATION OF PLC AREAS IS NOT
 *	IMPLEMENTED YET!
 */

#endif	/* #if defined( ALPLC_C_GCCARM ) || defined( ALPLC_C_MSDEVX86 ) || defined( ALPLC_C_CODEWARRIOR ) || defined( ALPLC_C_GCCX86 ) || defined( ALPLC_C_MSDEVARM ) || defined( ALPLC_C_CCRX ) #elif defined( ALPLC_C_SOFTUNE16FX ) #elif defined( ALPLC_C_C166K ) */

#if defined( ALPLC_C_C166K )
#define PLC_TASK_PTR void PLC_ATTR_DATA **
#elif defined( ALPLC_C_CW_DSP56800E )
// Pointers to code RAM must be WORD pointers
#define PLC_TASK_PTR uint16_t **
#else
#define PLC_TASK_PTR void PLC_ATTR_CODE * volatile *
#endif

#if defined( ALPLC_P_ARM_THUMB2 ) || defined( ALPLC_P_ARM_THUMB2VFP2 ) || defined( ALPLC_P_ARM_THUMB2VFP2_SP )
#define PLC_CALCCODEADDR( x )			( void PLC_ATTR_CODE* )( (x) + 1 )
#define PLC_CALCDATAADDR( x )			( void PLC_ATTR_DATA* )(x)
#else
#define PLC_CALCCODEADDR( x )			( void PLC_ATTR_CODE* )(x)
#define PLC_CALCDATAADDR( x )			( void PLC_ATTR_DATA* )(x)
#endif	/* defined( ALPLC_P_ARM_THUMB2 ) */

/* Packing of C structures */
#if defined( ALPLC_C_MSDEVX86 ) || defined( ALPLC_C_MSDEVARM ) || defined( ALPLC_C_C166K )

#define _C_PACK_											/*  Compiler packing directive (near structure definition)  */
#define _C_START_PACK_PRAGMA	pack(1)						/*  Compiler packing pragma */
#define _C_END_PACK_PRAGMA		pack()						/*  Compiler packing pragma */
#define _C_PACK_PREFIXED_		 							/* prefixed directive (not needed) */

#elif defined( ALPLC_C_MIKROC_M4F ) || defined( ALPLC_C_GCCARM ) || defined( ALPLC_C_GCCX86 ) || defined(ALPLC_C_GCCCF) || defined( ALPLC_C_DIABPPC )

#define _C_PACK_  				__attribute__((packed))		/*	Compiler packing directive (near structure definition)	*/
/*#define _C_START_PACK_PRAGMA							  	Not needed with GCC */
/*#define _C_END_PACK_PRAGMA								Not needed with GCC */
#define _C_PACK_PREFIXED_		 							/* prefixed directive (not needed) */

#elif defined( ALPLC_C_CODEWARRIOR ) || defined( ALPLC_C_PXE ) || defined( ALPLC_C_CW_DSP56800E ) || defined(ALPLC_C_GCCPPC)

#define _C_PACK_											/*  Compiler packing directive (near structure definition)  */
#define _C_START_PACK_PRAGMA	options align= packed		/*  Compiler packing pragma */
#define _C_END_PACK_PRAGMA		options align= reset		/*  Compiler packing pragma */
#define _C_PACK_PREFIXED_		 							/* prefixed directive (not needed) */

#elif defined( ALPLC_C_SOFTUNE16FX )

#define _C_PACK_											/*  Compiler packing directive (near structure definition)  */
#define _C_START_PACK_PRAGMA								/*  Compiler packing pragma */
#define _C_END_PACK_PRAGMA									/*  Compiler packing pragma */
#define _C_PACK_PREFIXED_		 							/* prefixed directive (not needed) */

#elif defined( ALPLC_C_CODECOMP ) || defined( ALPLC_C_CODECOMP_BIT )

#define _C_PACK_				 							/*  Compiler packing directive (near structure definition)  */
/*#define _C_START_PACK_PRAGMA	PACK(1)						Compiler packing pragma */
/*#define _C_END_PACK_PRAGMA		PACK()					Compiler packing pragma */
#define _C_PACK_PREFIXED_		 							/* prefixed directive (not needed) */

#elif defined( ALPLC_C_IAR_ARM ) || defined( ALPLC_C_CCRX )

#define _C_PACK_ 								/*  Compiler packing directive (near structure definition)  */
// #define _C_START_PACK_PRAGMA	                /* Compiler packing pragma */
// #define _C_END_PACK_PRAGMA                   /* Compiler packing pragma */
#define _C_PACK_PREFIXED_		__packed 		/* prefixed directive */

#elif defined(ALPLC_C_TASKING)

#define _C_PACK_ 								/* Compiler packing directive (near structure definition) */
//#define _C_START_PACK_PRAGMA	         		/* Compiler packing pragma */
//#define _C_END_PACK_PRAGMA      	            /* Compiler packing pragma */
#define _C_PACK_PREFIXED_		 	        	/* prefixed directive */
  
#endif


/* Definition of PLC_BYTE_ADDR, PLC_WORD_ADDR, MAKE_CODE_OFFS
   for that systems which has many different types of pointers,
   they depends on variable type */
#if defined( ALPLC_C_CW_DSP56800E )
// DSP56800E has 2 types of pointers: word an byte pointers
// when we cast a generic pointer to a byte or a word pointer, the compiler multiplies
// or divide it by 2, obtaining the corresponding byte or word address
#define PLC_BYTE_ADDR( addr ) ((uint8_t *) (addr)) // change address in byte address
#define PLC_WORD_ADDR( addr ) ((uint16_t *) (addr)) // change address in word address
#define MAKE_CODE_ADDR( addr ) PLC_WORD_ADDR( addr )
#define MAKE_CODE_OFFS( offs ) ((offs) >> 1) // code addresses are expressed in word pointers
#define MAKE_DATA_ADDR( addr ) (addr) // Not checked, do test
#error "TODO: MAKE_DATA_ADDR implementation must be tested! it is required if PLC_S_DYNAMIC_TASKS == 1"
#else // defined( ALPLC_C_CW_DSP56800E )
// Default implementation: doesn't change addresses and offset
#define PLC_BYTE_ADDR( addr ) (addr)
#define PLC_WORD_ADDR( addr ) (addr)
#define MAKE_CODE_OFFS( offs ) (offs)
#define MAKE_CODE_ADDR( addr ) (addr)
#define MAKE_DATA_ADDR( addr ) (addr)
#endif // else defined( ALPLC_C_CW_DSP56800E )


/* PLC variable types (the same values of PlcVar.h in LogicLab */
typedef enum _TYVAR
{
	tyvNull 		= 0,
	tyvBasic 		= 0,
	tyvBool 		= 1,
	tyvSInt 		= 2,
	tyvUSInt 		= 3,
	tyvByte 		= 4,
	tyvInt			= 5,
	tyvUInt			= 6,
	tyvWord			= 7,
	tyvDInt			= 9,
	tyvUDInt		= 10,
	tyvDWord		= 11,
	tyvLInt			= 12,
	tyvULInt		= 13,
	tyvLWord		= 14,
	tyvReal			= 15,
	tyvLReal		= 16,
	tyvString		= 17,
	tyvUserDef		= 18,
	tyvUndefined	= 19,
	tyvPointer		= 20,
	tyvDate			= 24,
	tyvLDate		= 25,
	tyvAny			= 100,
	
	// dummy value to force the allocation of this enum as 32bit on platforms that by default use 16bit enums
	_tyvDummy32bitVal = 0x7FFFFFFF,
}
TYVAR;

// ARRDIM replicato sia qui sia in OSTypes.h in quanto tale file non e' presente su Thumb2 e l'assenza della define
// comporta la comparsa di warning a tempo di compilazione (perche' warning e non errori? Mistero...)
#ifndef ARRDIM
#define ARRDIM(t)			(sizeof(t) / sizeof(t[0]))
#endif

/* ----------------------------- fixed width numerical limits (if not found in stdint.h and float.h) ------------------------------ */
#ifndef INT8_MIN
	#define INT8_MIN	(-128)
#endif
#ifndef INT8_MAX
	#define INT8_MAX	127
#endif
#ifndef INT16_MIN
	#define INT16_MIN	(-32768)
#endif
#ifndef INT16_MAX
	#define INT16_MAX	32767
#endif
#ifndef INT32_MIN
	#define INT32_MIN	(-2147483647L - 1)
#endif
#ifndef INT32_MAX
	#define INT32_MAX	2147483647L
#endif
#ifndef INT64_MIN
	#define INT64_MIN	(-9223372036854775807LL - 1)
#endif
#ifndef INT64_MAX
	#define INT64_MAX	9223372036854775807LL
#endif

#ifndef UINT8_MAX
	#define UINT8_MAX	0xff
#endif
#ifndef UINT16_MAX
	#define UINT16_MAX	0xffff
#endif
#ifndef UINT32_MAX
	#define UINT32_MAX	0xffffffffUL
#endif
#ifndef UINT64_MAX
	#define UINT64_MAX	0xffffffffffffffffULL
#endif

#ifndef FLT_MAX
	#define FLT_MAX		3.402823466e+38F
#endif
#define FLOAT32_MAX		FLT_MAX
#define FLOAT32_MIN		(-FLOAT32_MAX)

#ifndef DBL_MAX
	#define DBL_MAX		1.7976931348623158e+308
#endif
#define FLOAT64_MAX		DBL_MAX
#define FLOAT64_MIN		(-FLOAT64_MAX)




#ifdef __cplusplus
}
#endif	/* __cplusplus */

#endif	/* AL_PLC_C_DEFS_H_ */

