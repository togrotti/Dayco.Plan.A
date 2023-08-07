/*------------------------------------------------------------------------------
**
**	Copyright © 2009-2013 Axel S.r.l
**
**	Misra-C-compliant data types
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

#ifndef AL_PLC_MISRA_C_H_
#define AL_PLC_MISRA_C_H_

#ifdef __cplusplus
extern "C" {
#endif	/* __cplusplus */

#if !defined( EXTERNAL_MISRA_C_TYPES ) && !defined( USE_STDINT_FOR_MISRA_C )

/*  C data types MISRA C compliant */
#if defined( ALPLC_P_ARM32 ) || defined( ALPLC_P_ARM_THUMB2 ) || defined( ALPLC_P_ARM32VFP2 ) || defined( ALPLC_P_ARM_THUMB2VFP2 ) || defined( ALPLC_P_ARM32VFP2_SP ) || defined( ALPLC_P_ARM_THUMB2VFP2_SP ) || defined( ALPLC_P_X86 ) || defined( ALPLC_P_X64 ) || defined( ALPLC_P_COLDFIRE ) ||  defined( ALPLC_P_16FX ) || defined( ALPLC_P_C166 ) || defined( ALPLC_P_POWERPC ) || defined( ALPLC_P_POWERPCE200 ) || defined( ALPLC_P_TIDSP ) || defined( ALPLC_P_RENESASRX ) || defined( ALPLC_P_TRICORE ) || defined(ALPLC_P_ARM64)

	typedef char				char_t;
	typedef unsigned char   	uint8_t;
	typedef signed char 		int8_t;
#if defined(ALPLC_C_CODECOMP) || defined(ALPLC_C_CODECOMP_BIT)
	typedef unsigned int		uint16_t;
	typedef int   				int16_t;
#else
	typedef unsigned short  	uint16_t;
	typedef short   			int16_t;
#endif
#if defined(ALPLC_C_CCRX) || defined(ALPLC_C_MSDEVX86) || defined(ALPLC_C_GCCX86) || defined(ALPLC_C_GCCARM) || defined(ALPLC_C_TASKING)
	/* Better to define int32_t and uint32_t as in <stdint.h>. This may apply also to other compilers. */
	typedef unsigned int   		uint32_t;
	typedef int					int32_t;
#else
	typedef unsigned long   	uint32_t;
	typedef long				int32_t;
#endif
#if defined(ALPLC_C_MSDEVARM) && _MSC_VER < 1300
	/* MS embedded c++ 4.0 for WindowsCE5.0 does not have "long long" types */
	typedef unsigned __int64	uint64_t;
	typedef __int64				int64_t;
#elif defined(ALPLC_P_ARM64)
	typedef unsigned long		uint64_t;
	typedef long				int64_t;
#elif ! defined( ALPLC_P_C166 )
	typedef unsigned long long	uint64_t;
	typedef long long			int64_t;
#endif	/* ! defined( ALPLC_P_C166 ) */
	typedef float				float32_t;
	typedef double				float64_t;
	typedef uint8_t 			bool_t;

#endif /* defined( ALPLC_P_ARM32 ) || defined( ALPLC_P_ARM_THUMB2 ) || defined( ALPLC_P_ARM32VFP2 ) || defined( ALPLC_P_ARM_THUMB2VFP2 ) || defined( ALPLC_P_ARM32VFP2_SP ) || defined( ALPLC_P_ARM_THUMB2VFP2_SP ) || defined( ALPLC_P_X86 ) || defined( ALPLC_P_COLDFIRE ) ||  defined( ALPLC_P_16FX ) || defined( ALPLC_P_C166 ) || defined( ALPLC_P_POWERPC ) || defined( ALPLC_P_POWERPCE200 ) || defined( ALPLC_P_TIDSP ) || defined( ALPLC_P_RENESASRX ) */


#elif defined( USE_STDINT_FOR_MISRA_C )

/* if USE_STDINT_FOR_MISRA_C is defined, <stdint.h> is used to provide standard definitions for fixed types (C99) */
#include <stdint.h>

/* required types not defined in stdint.h! */
typedef char				char_t;
typedef float				float32_t;
typedef double				float64_t;
typedef unsigned char		bool_t;

#endif

#ifdef __cplusplus
}
#endif	/* __cplusplus */

#endif	/* AL_PLC_MISRA_C_H_ */

