/*------------------------------------------------------------------------------
**
**	Copyright © 2009-2020 Axel S.r.l
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

#ifndef AL_PLC_BASIC_FUN_NAMES_H_
#define AL_PLC_BASIC_FUN_NAMES_H_


/* ------------------------------------- X86 ------------------------------------------- */
#define PLCFN_X86_DIV_ULINT			"_div_ULINT"
#define PLCFN_X86_DIV_LINT			"_div_LINT"
#define PLCFN_X86_MOD_ULINT			"_mod_ULINT"
#define PLCFN_X86_MOD_LINT			"_mod_LINT"
#define PLCFN_X86_LREAL_TO_ULINT	"_LREAL_TO_ULINT"
#define PLCFN_X86_ULINT_TO_LREAL	"_ULINT_TO_LREAL"



/* ------------------------------------- ARM ------------------------------------------- */
#define PLCFN_ARM_MAX_VAL			"_ret_max_val"
#define PLCFN_ARM_MIN_VAL			"_ret_min_val"
#define PLCFN_ARM_MAX_LIMIT			"_ret_max_limit"
#define PLCFN_ARM_MIN_LIMIT			"_ret_min_limit"
#define PLCFN_ARM_MAX_VAL_LREAL		"_ret_max_val_lreal"
#define PLCFN_ARM_MIN_VAL_LREAL		"_ret_min_val_lreal"
#define PLCFN_ARM_MAX_LIMIT_LREAL	"_ret_max_limit_lreal"
#define PLCFN_ARM_MIN_LIMIT_LREAL	"_ret_min_limit_lreal"
#define PLCFN_ARM_DIV_LINT			"_do_lint_div"
#define PLCFN_ARM_MOD_LINT			"_do_lint_mod"
#define PLCFN_ARM_DIV_ULINT			"_do_ulint_div"
#define PLCFN_ARM_MOD_ULINT			"_do_ulint_mod"
#define PLCFN_ARM_MUL_ULINT			"_do_ulint_mul"
#define PLCFN_ARM_LINT_TO_FLOAT		"_lint_to_float"
#define PLCFN_ARM_DIV_INT			"_do_int_div"		// ARM32 only
#define PLCFN_ARM_MOD_INT			"_do_mod"			// ARM32 only
#define PLCFN_ARM_DIV_UINT			"_do_uint_div"		// ARM32 only
#define PLCFN_ARM_MOD_UINT			"_do_uint_mod"		// ARM32 only
#define PLCFN_ARM_MEMCOPY			"_memcopy"			// ARMThumb only




/* ----------------------------------- TRICORE ---------------------------------------- */
#define PLCFN_TC1_STRNCPY			"_tri_strncpy"
#define PLCFN_TC1_MAXVAL			"_ret_max_val"
#define PLCFN_TC1_MINVAL			"_ret_min_val"
#define PLCFN_TC1_MAXLIMIT			"_ret_max_limit"
#define PLCFN_TC1_MINLIMIT			"_ret_min_limit"
#define PLCFN_TC1_FABS				"_do_float_abs"
#define PLCFN_TC1_SQRT				"_do_sqrt"
#define PLCFN_TC1_SIN				"_do_sin"
#define PLCFN_TC1_COS				"_do_cos"
#define PLCFN_TC1_TAN				"_do_tan"
#define PLCFN_TC1_ASIN				"_do_asin"
#define PLCFN_TC1_ACOS				"_do_acos"
#define PLCFN_TC1_ATAN				"_do_atan"
#define PLCFN_TC1_SINH				"_do_sinh"
#define PLCFN_TC1_COSH				"_do_cosh"
#define PLCFN_TC1_TANH				"_do_tanh"
#define PLCFN_TC1_LOG				"_do_log"
#define PLCFN_TC1_LN				"_do_ln"
#define PLCFN_TC1_EXP				"_do_exp"
#define PLCFN_TC1_CEIL				"_do_ceil"
#define PLCFN_TC1_FLOOR				"_do_floor"
#define PLCFN_TC1_ATAN2				"_do_atan2"
#define PLCFN_TC1_POW				"_do_pow"


#endif /* AL_PLC_BASIC_FUN_NAMES_H_ */
