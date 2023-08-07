
#ifndef _ALPLCMATH_H
#define _ALPLCMATH_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
*	include
******************************************************************************/

#include "AlPlcCDefs.h"
#include "AlPlcTarg.h"
#include "../AlPlcRuntime5/AlPlcRealInterface.h"

/******************************************************************************
*	public interface
******************************************************************************/

//	Calculate power
FLOAT_RET_TYPE AlPlcMath_Pow(DECLARE_FLOAT_PARAM(b), DECLARE_FLOAT_PARAM(e));

//	Calculate sqrt of the value
FLOAT_RET_TYPE AlPlcMath_Sqrt(DECLARE_FLOAT_PARAM(v));

//	Calculate arccos of the value
FLOAT_RET_TYPE AlPlcMath_ACos(DECLARE_FLOAT_PARAM(v));

//	Calculate arcsin of the value
FLOAT_RET_TYPE AlPlcMath_ASin(DECLARE_FLOAT_PARAM(v));

//	Calculate arctan of the value
FLOAT_RET_TYPE AlPlcMath_ATan(DECLARE_FLOAT_PARAM(v));

//	Calculate ATan2
FLOAT_RET_TYPE AlPlcMath_ATan2(DECLARE_FLOAT_PARAM(v1), DECLARE_FLOAT_PARAM(v2));

//	Calculate ceil of the value
FLOAT_RET_TYPE AlPlcMath_Ceil(DECLARE_FLOAT_PARAM(v));

//	Calculate cos of the value
FLOAT_RET_TYPE AlPlcMath_Cos(DECLARE_FLOAT_PARAM(v));

//	Calculate cosh of the value
FLOAT_RET_TYPE AlPlcMath_Cosh(DECLARE_FLOAT_PARAM(v));

//	Calculate exponential function of the value
FLOAT_RET_TYPE AlPlcMath_Exp(DECLARE_FLOAT_PARAM(v));

//	Calculate floor of the value
FLOAT_RET_TYPE AlPlcMath_Floor(DECLARE_FLOAT_PARAM(v));

//	Calculate natural logaritm of the value
FLOAT_RET_TYPE AlPlcMath_Ln(DECLARE_FLOAT_PARAM(v));

//	Calculate logaritm base 10 of the value
FLOAT_RET_TYPE AlPlcMath_Log(DECLARE_FLOAT_PARAM(v));

//	Calculate sin of the value
FLOAT_RET_TYPE AlPlcMath_Sin(DECLARE_FLOAT_PARAM(v));

//	Calculate sinh of the value
FLOAT_RET_TYPE AlPlcMath_Sinh(DECLARE_FLOAT_PARAM(v));

//	Calculate tan of the value
FLOAT_RET_TYPE AlPlcMath_Tan(DECLARE_FLOAT_PARAM(v));

//	Calculate tanh of the value
FLOAT_RET_TYPE AlPlcMath_Tanh(DECLARE_FLOAT_PARAM(v));

//	Calculate fabs of the value
FLOAT_RET_TYPE AlPlcMath_Fabs(DECLARE_FLOAT_PARAM(v));

#ifdef __cplusplus
}
#endif

#endif	//	_ALPLCMATH_H
