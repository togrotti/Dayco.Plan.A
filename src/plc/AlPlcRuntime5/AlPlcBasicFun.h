/*------------------------------------------------------------------------------
**
**	Copyright � 2009-2020 Axel S.r.l
**
**	Specific compiler required exported embedded functions
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

#ifndef AL_PLC_BASIC_FUN_H_
#define AL_PLC_BASIC_FUN_H_

#include "AlPlcMisraC.h"
#include "AlPlcBasicFun_names.h"


/* gets a PLCIEC_FCNREC when PLC and C function names match */
#ifndef ALPLC_MAKEFCNREC
#define ALPLC_MAKEFCNREC(func)				{ #func, (ALPLC_FCNREC_TYPE)func },
#endif

/* gets a PLCIEC_FCNREC when PLC and C function names DO NOT match */
/*
#ifndef ALPLC_MAKEFCNREC2
#define ALPLC_MAKEFCNREC2(name, func)		{ name, (ALPLC_FCNREC_TYPE)func },
#endif
*/
#ifndef ALPLC_MAKEFCNREC2
#define ALPLC_MAKEFCNREC2(name, func)		{ name, (uint32_t)&func },
#endif

/* ------------------------------------- X86 ------------------------------------------- */
#if defined(ALPLC_P_X86)

#include <math.h>

//	wrapper required, function not a constant (some compilers, like VS, use inline version)
static float64_t _do_ceil(float64_t x)				{ return ceil(x); }
static float64_t _do_floor(float64_t x)				{ return floor(x); }

static uint64_t _div_ULINT(uint64_t a, uint64_t b)	{ return a / b; }
static int64_t _div_LINT(int64_t a, int64_t b)		{ return a / b; }
static uint64_t _mod_ULINT(uint64_t a, uint64_t b)	{ return a % b; }
static int64_t _mod_LINT(int64_t a, int64_t b)		{ return a % b; }

// these two functions are required because ASM x86 FPU instructions only allow conversion to/from signed 64bit integer
// another way could be to enable REAL plugin, that already provides these
static uint64_t  _LREAL_TO_ULINT(float64_t f)		{ return (uint64_t)f; }
static float64_t _ULINT_TO_LREAL(uint64_t u)		{ return (float64_t)u; }

#define ALPLC_BASICFUN_FCNREC \
	/* x86 still does not support MATH plugin, so export all required C functions here
    (maybe the problem is only for log->log10 and ln->log that have "non standard names")
    can directly use C library functions without wrapping, because x86 has relocable code */ \
	ALPLC_MAKEFCNREC(exp) \
	ALPLC_MAKEFCNREC(pow) \
	ALPLC_MAKEFCNREC(sqrt) \
	ALPLC_MAKEFCNREC(tan) \
	ALPLC_MAKEFCNREC(asin) \
	ALPLC_MAKEFCNREC(acos) \
	ALPLC_MAKEFCNREC(sinh) \
	ALPLC_MAKEFCNREC(cosh) \
	ALPLC_MAKEFCNREC(tanh) \
	ALPLC_MAKEFCNREC(atan) \
	ALPLC_MAKEFCNREC(atan2) \
	ALPLC_MAKEFCNREC(log10) \
	ALPLC_MAKEFCNREC(log) \
	ALPLC_MAKEFCNREC2("ceil", _do_ceil)  \
	ALPLC_MAKEFCNREC2("floor", _do_floor) \
	ALPLC_MAKEFCNREC2(PLCFN_X86_DIV_ULINT, _div_ULINT) \
	ALPLC_MAKEFCNREC2(PLCFN_X86_DIV_LINT,  _div_LINT) \
	ALPLC_MAKEFCNREC2(PLCFN_X86_MOD_ULINT, _mod_ULINT) \
	ALPLC_MAKEFCNREC2(PLCFN_X86_MOD_LINT,  _mod_LINT) \
	ALPLC_MAKEFCNREC2(PLCFN_X86_LREAL_TO_ULINT, _LREAL_TO_ULINT) \
	ALPLC_MAKEFCNREC2(PLCFN_X86_ULINT_TO_LREAL, _ULINT_TO_LREAL)



/* ------------------------------------- X64 ------------------------------------------- */
#elif defined(ALPLC_P_X64)

#include <math.h>

//	wrapper required, function not a constant (some compilers, like VS, use inline version)
static float64_t _do_ceil(float64_t x) 			{ return ceil(x); }
static float64_t _do_floor(float64_t x) 		{ return floor(x); }

// these two functions are required because ASM x86 FPU instructions only allow conversion to/from signed 64bit integer
// another way could be to enable REAL plugin, that already provides these
static uint64_t  _LREAL_TO_ULINT(float64_t f)	{ return (uint64_t)f; }
static float64_t _ULINT_TO_LREAL(uint64_t u)	{ return (float64_t)u; }

#define ALPLC_BASICFUN_FCNREC \
	/* x64 still does not support MATH plugin, so export all required C functions here
	(maybe the problem is only for log->log10 and ln->log that have "non standard names")
	can directly use C library functions without wrapping, because x64 has relocable code */ \
	ALPLC_MAKEFCNREC(exp) \
	ALPLC_MAKEFCNREC(pow) \
	ALPLC_MAKEFCNREC(sqrt) \
	ALPLC_MAKEFCNREC(tan) \
	ALPLC_MAKEFCNREC(asin) \
	ALPLC_MAKEFCNREC(acos) \
	ALPLC_MAKEFCNREC(sinh) \
	ALPLC_MAKEFCNREC(cosh) \
	ALPLC_MAKEFCNREC(tanh) \
	ALPLC_MAKEFCNREC(atan) \
	ALPLC_MAKEFCNREC(atan2) \
	ALPLC_MAKEFCNREC(log10) \
	ALPLC_MAKEFCNREC(log) \
	ALPLC_MAKEFCNREC2("ceil", _do_ceil)  \
	ALPLC_MAKEFCNREC2("floor", _do_floor) \
	ALPLC_MAKEFCNREC2(PLCFN_X86_LREAL_TO_ULINT, _LREAL_TO_ULINT) \
	ALPLC_MAKEFCNREC2(PLCFN_X86_ULINT_TO_LREAL, _ULINT_TO_LREAL)



/* ------------------------------------- ARM ------------------------------------------- */
#elif defined(ALPLC_P_ARM32) || defined(ALPLC_P_ARM32VFP2) || defined(ALPLC_P_ARM_THUMB2) || defined(ALPLC_P_ARM_THUMB2VFP2) || defined(ALPLC_P_ARM32VFP2_SP) || defined(ALPLC_P_ARM_THUMB2VFP2_SP)

#include "AlPlcRealInterface.h"

static int32_t 	_do_int_div(int32_t par1, int32_t par2)		{ return par1 / par2; }
static int32_t 	_do_mod(int32_t par1, int32_t par2)			{ return par1 % par2; }
static uint32_t _do_uint_div(uint32_t par1, uint32_t par2)	{ return par1 / par2; }
static uint32_t _do_uint_mod(uint32_t par1, uint32_t par2)	{ return par1 % par2; }
static int64_t	_do_lint_div(int64_t par1, int64_t par2)	{ return par1 / par2; }
static int64_t 	_do_lint_mod(int64_t par1, int64_t par2)	{ return par1 % par2; }
static uint64_t	_do_ulint_div(uint64_t par1, uint64_t par2)	{ return par1 / par2; }
static uint64_t	_do_ulint_mod(uint64_t par1, uint64_t par2)	{ return par1 % par2; }
static uint64_t	_do_ulint_mul(uint64_t par1, uint64_t par2)	{ return par1 * par2; }

static FLOAT_RET_TYPE _lint_to_float(int64_t val)
{
	RETURN_FLOAT(val)
}

static FLOAT_RET_TYPE _ret_max_val(DECLARE_FLOAT_PARAM(par1), DECLARE_FLOAT_PARAM(par2))
{
	USE_FLOAT_PARAM(par1)
	USE_FLOAT_PARAM(par2)

	if (par1 >= par2)
		RETURN_FLOAT(par1)
	else
		RETURN_FLOAT(par2)
}

static FLOAT_RET_TYPE _ret_min_val(DECLARE_FLOAT_PARAM(par1), DECLARE_FLOAT_PARAM(par2))
{
	USE_FLOAT_PARAM(par1)
	USE_FLOAT_PARAM(par2)

	if (par1 <= par2)
		RETURN_FLOAT(par1)
	else
		RETURN_FLOAT(par2)
}

static FLOAT_RET_TYPE _ret_max_limit(DECLARE_FLOAT_PARAM(par1), DECLARE_FLOAT_PARAM(par2))
{
	USE_FLOAT_PARAM(par1)
	USE_FLOAT_PARAM(par2)

	if (par1 >= par2)
		RETURN_FLOAT(par2)
	else
		RETURN_FLOAT(par1)
}

static FLOAT_RET_TYPE _ret_min_limit(DECLARE_FLOAT_PARAM(par1), DECLARE_FLOAT_PARAM(par2))
{
	USE_FLOAT_PARAM(par1)
	USE_FLOAT_PARAM(par2)

	if (par1 <= par2)
		RETURN_FLOAT(par2)
	else
		RETURN_FLOAT(par1)
}

static DOUBLE_RET_TYPE _ret_max_val_lreal(DECLARE_DOUBLE_PARAM(par1), DECLARE_DOUBLE_PARAM(par2))
{
	USE_DOUBLE_PARAM(par1)
	USE_DOUBLE_PARAM(par2)

	if (par1 >= par2)
		RETURN_DOUBLE(par1)
	else
		RETURN_DOUBLE(par2)
}

static DOUBLE_RET_TYPE _ret_min_val_lreal(DECLARE_DOUBLE_PARAM(par1), DECLARE_DOUBLE_PARAM(par2))
{
	USE_DOUBLE_PARAM(par1)
	USE_DOUBLE_PARAM(par2)

	if (par1 <= par2)
		RETURN_DOUBLE(par1)
	else
		RETURN_DOUBLE(par2)
}

static DOUBLE_RET_TYPE _ret_max_limit_lreal(DECLARE_DOUBLE_PARAM(par1), DECLARE_DOUBLE_PARAM(par2))
{
	USE_DOUBLE_PARAM(par1)
	USE_DOUBLE_PARAM(par2)

	if (par1 >= par2)
		RETURN_DOUBLE(par2)
	else
		RETURN_DOUBLE(par1)
}

static DOUBLE_RET_TYPE _ret_min_limit_lreal(DECLARE_DOUBLE_PARAM(par1), DECLARE_DOUBLE_PARAM(par2))
{
	USE_DOUBLE_PARAM(par1)
	USE_DOUBLE_PARAM(par2)

	if (par1 <= par2)
		RETURN_DOUBLE(par2)
	else
		RETURN_DOUBLE(par1)
}

/* common functions for both ARM32 and ARMThumb */
#define ALPLC_BASICFUN_ARM_COMMON_FCNREC \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_MAX_VAL,			_ret_max_val) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_MIN_VAL,			_ret_min_val) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_MAX_LIMIT,			_ret_max_limit) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_MIN_LIMIT,			_ret_min_limit) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_MAX_VAL_LREAL,		_ret_max_val_lreal) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_MIN_VAL_LREAL,		_ret_min_val_lreal) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_MAX_LIMIT_LREAL,	_ret_max_limit_lreal) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_MIN_LIMIT_LREAL,	_ret_min_limit_lreal) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_DIV_LINT,			_do_lint_div) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_MOD_LINT,			_do_lint_mod) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_DIV_ULINT,			_do_ulint_div) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_MOD_ULINT,			_do_ulint_mod) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_MUL_ULINT,			_do_ulint_mul) \
	ALPLC_MAKEFCNREC2(PLCFN_ARM_LINT_TO_FLOAT,		_lint_to_float) \

#if defined( ALPLC_P_ARM32 ) || defined(ALPLC_P_ARM32VFP2) || defined(ALPLC_P_ARM32VFP2_SP)
	/* ARM32 code generator also requires these mathematical functions */
	#define ALPLC_BASICFUN_FCNREC \
		ALPLC_BASICFUN_ARM_COMMON_FCNREC \
		ALPLC_MAKEFCNREC2(PLCFN_ARM_DIV_INT,		_do_int_div) \
		ALPLC_MAKEFCNREC2(PLCFN_ARM_MOD_INT,		_do_mod) \
		ALPLC_MAKEFCNREC2(PLCFN_ARM_DIV_UINT,		_do_uint_div) \
		ALPLC_MAKEFCNREC2(PLCFN_ARM_MOD_UINT,		_do_uint_mod)

#else /* defined( ALPLC_P_ARM_THUMB2 ) || defined( ALPLC_P_ARM_THUMB2VFP2 ) || defined( ALPLC_P_ARM_THUMB2VFP2_SP ) */
	/* ARMThumb code generator requires external memcpy */
	#define ALPLC_BASICFUN_FCNREC \
		ALPLC_BASICFUN_ARM_COMMON_FCNREC \
		ALPLC_MAKEFCNREC2(PLCFN_ARM_MEMCOPY,		memcpy)
#endif



/* ------------------------------------- TRICORE ------------------------------------------- */
#elif defined(ALPLC_P_TRICORE)

    #include <string.h>
    #include <math.h>


    static float32_t max( float32_t f1, float32_t f2)
    {
        return ( f1 > f2 ) ? f1 : f2;
    }

    static float32_t min( float32_t f1, float32_t f2 )
    {
        return ( f1 < f2 ) ? f1 : f2;
    }


    #define ALPLC_BASICFUN_FCNREC \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_MAXVAL,         max)        \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_MAXLIMIT,       max)        \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_MINVAL,         min)        \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_MINLIMIT,       min)        \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_STRNCPY,        strncpy) \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_FABS,           fabsf)      \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_SQRT,           sqrtf)      \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_SIN,            sinf)       \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_COS,            cosf)       \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_TAN,            tanf)       \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_ASIN,           asinf)      \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_ACOS,           acosf)      \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_ATAN,           atanf)      \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_SINH,           sinhf)      \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_COSH,           coshf)      \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_TANH,           tanhf)      \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_LOG,            log10f)     \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_LN,             logf)       \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_EXP,            expf)       \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_CEIL,           ceilf)      \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_FLOOR,          floorf)     \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_ATAN2,          atan2f)     \
        ALPLC_MAKEFCNREC2(PLCFN_TC1_POW,            powf)


/* ------------------------------------- Others... ------------------------------------------- */
#else
/* TODO add here all definitions for other architectures */
#define ALPLC_BASICFUN_FCNREC

#endif




#endif /* AL_PLC_BASIC_FUN_H_ */
