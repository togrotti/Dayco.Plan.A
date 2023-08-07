/*------------------------------------------------------------------------------
**
**	Copyright © 2009-2014 Axel S.r.l
**
**	[PLC run-time plug-in] Advanced mathematical function library (interface)
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

#ifndef AL_PLC_REAL_INTERFACE_H_
#define AL_PLC_REAL_INTERFACE_H_


#if !defined( ALPLCREAL_TYPE )
	#if defined( ALPLC_C_CODEWARRIOR )
		/* for these architectures/compilers the REAL type is implemented completely as float64_t (=double) ! */
		#define ALPLCREAL_TYPE float64_t
		#define ALPLCREAL_IS_FLOAT64
	#else
		#define ALPLCREAL_TYPE float32_t
	#endif
#endif

#if !defined( ALPLCLREAL_TYPE )
	#define ALPLCLREAL_TYPE float64_t
#endif

#if !defined( ALPLCREAL_TYPE_INTF )
	#if defined( ALPLC_C_MIKROC_M4F ) || defined( __ARM_PCS_VFP ) || defined( ALPLC_C_IAR_ARM )
		/* pass float values inside uint32_t variables, instead of VFP float registers,
		 * embedded functions will then convert back them to float values
		 * used by MIKROC and ARM platforms with hard-fp ABI (for example gnueabihf toolchain),
		 * until LogicLab will support hardfp calling convention natively!
		 * PS: __ARM_PCS_VFP is automatically defined on all GCC ARM compilers if hard FP is present. */
		#define ALPLCREAL_TYPE_INTF uint32_t
		#define ALPLCREAL_UINT32_INTF
	#else
		#define ALPLCREAL_TYPE_INTF ALPLCREAL_TYPE
	#endif
#endif

/* Passaggio di parametri float, per plugins */
#ifdef ALPLCREAL_UINT32_INTF /* in caso di hard float ABI */
	#define DECLARE_FLOAT_PARAM(param)		uint32_t __tmp_##param
	#define USE_FLOAT_PARAM(param)			float32_t param = *((float32_t *) &__tmp_##param);
	#define FLOAT_RET_TYPE					uint32_t
	#define RETURN_FLOAT(param)				{ float32_t __tmp_return = param; return *((uint32_t *) &__tmp_return); }

	#define DECLARE_DOUBLE_PARAM(param)		uint64_t __tmp_##param
	#define USE_DOUBLE_PARAM(param)			float64_t param = *((float64_t *) &__tmp_##param);
	#define DOUBLE_RET_TYPE					uint64_t
	#define RETURN_DOUBLE(param)			{ float64_t __tmp_return = param; return *((uint64_t *) &__tmp_return); }

#else
	#define DECLARE_FLOAT_PARAM(param)		ALPLCREAL_TYPE param
	#define USE_FLOAT_PARAM(param)
	#define FLOAT_RET_TYPE					ALPLCREAL_TYPE
	#define RETURN_FLOAT(param)				return param; 

	#define DECLARE_DOUBLE_PARAM(param)		ALPLCLREAL_TYPE param
	#define USE_DOUBLE_PARAM(param)
	#define DOUBLE_RET_TYPE					ALPLCLREAL_TYPE
	#define RETURN_DOUBLE(param)			return param; 
#endif


/* Check LREAL support (must be supported in corresponding code generator) */
#if defined(ALPLC_P_ARM32) || defined(ALPLC_P_ARM_THUMB2) || defined(ALPLC_P_ARM32VFP2) || defined(ALPLC_P_ARM_THUMB2VFP2) || defined( ALPLC_P_ARM32VFP2_SP ) || defined( ALPLC_P_ARM_THUMB2VFP2_SP ) || defined( ALPLC_P_ARM64 )
	#define ALPLC_LREAL_SUPPORT
#endif

#endif	/* AL_PLC_REAL_INTERFACE_H_ */
