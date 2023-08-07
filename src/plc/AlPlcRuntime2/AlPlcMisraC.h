/*------------------------------------------------------------------------------
**
**	Copyright:	AXEL s.r.l. 2010
**
**	AlPlcMisraC.h:	Misra-C data types definitions
**
**-----------------------------------------------------------------------------
**
**	IMPORTANT:
**	THIS MODULE SHOULDN'T BE MODIFIED BY THE CUSTOMER
**
**-----------------------------------------------------------------------------*/

#ifndef _ALPLCMISRAC_H_
#define _ALPLCMISRAC_H_

#ifdef __cplusplus
extern "C" {
#endif

/*  C data types MISRA C compliant */
#if defined( ALPLC_P_ARM9 ) || defined( ALPLC_P_ARM_V7_M ) || defined( ALPLC_P_X86 ) || defined( ALPLC_P_COLDFIRE ) ||  defined( ALPLC_P_16FX ) || defined( ALPLC_P_C166 )

	typedef char				char_t;
	typedef unsigned char   	uint8_t;
	typedef signed char 		int8_t;
	typedef unsigned short  	uint16_t;
	typedef short   			int16_t;
	typedef unsigned long   	uint32_t;
	typedef long				int32_t;
#if defined(ALPLC_C_MSDEVARM) && _MSC_VER < 1300
	/* MS embedded c++ 4.0 for WindowsCE5.0 does not have "long long" types */
	typedef unsigned __int64	uint64_t;
	typedef __int64				int64_t;
#elif ! defined( ALPLC_P_C166 )
	typedef unsigned long long	uint64_t;
	typedef long long			int64_t;
#endif	/* ! defined( ALPLC_P_C166 ) */
	typedef float				float32_t;
	typedef double				float64_t;
	typedef uint8_t 			bool_t;

#endif /* defined( ALPLC_P_ARM9 ) || defined( ALPLC_P_X86 ) || defined( ALPLC_P_COLDFIRE ) ||  defined( ALPLC_P_16FX ) || defined( ALPLC_P_C166 ) */

#ifdef __cplusplus
}
#endif

#endif	/*	_ALPLCMISRAC_H_	*/
