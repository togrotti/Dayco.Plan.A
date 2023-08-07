/******************************************************************************
	AlPlcMath.c
******************************************************************************/

/******************************************************************************
	include
******************************************************************************/
#pragma GCC optimize (2)
#include "AlPlcMath.h"
#include <math.h>

/******************************************************************************
	global function implementation
******************************************************************************/

#define ALPLCMATH_POW(r,b,e)	r = (ALPLCREAL_TYPE) pow( (float64_t) b, (float64_t) e )

//	Calculate power
FLOAT_RET_TYPE AlPlcMath_Pow(DECLARE_FLOAT_PARAM(b), DECLARE_FLOAT_PARAM(e))
{
	USE_FLOAT_PARAM(b)
	USE_FLOAT_PARAM(e)
	ALPLCREAL_TYPE r;

	ALPLCMATH_POW(r, b, e);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_SQRT(r,v)		r = (ALPLCREAL_TYPE) sqrt( (float64_t) v )

//	Calculate sqrt of the value
FLOAT_RET_TYPE AlPlcMath_Sqrt(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_SQRT(r, v);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_ACOS(r,v)		r = (ALPLCREAL_TYPE) acos( (float64_t) v )

//	Calculate arccos of the value
FLOAT_RET_TYPE AlPlcMath_ACos(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_ACOS(r, v);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_ASIN(r,v)		r = (ALPLCREAL_TYPE) asin( (float64_t) v )

//	Calculate arcsin of the value
FLOAT_RET_TYPE AlPlcMath_ASin(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_ASIN(r, v);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_ATAN(r,v)		r = (ALPLCREAL_TYPE) atan( (float64_t) v )

//	Calculate arctan of the value
FLOAT_RET_TYPE AlPlcMath_ATan(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_ATAN(r, v);
	RETURN_FLOAT(r);
}


#define ALPLCMATH_ATAN2(r,v1,v2)	r = (ALPLCREAL_TYPE) atan2( ( float64_t) v1, (float64_t) v2 )

//	Calculate ATan2
FLOAT_RET_TYPE AlPlcMath_ATan2(DECLARE_FLOAT_PARAM(v1), DECLARE_FLOAT_PARAM(v2))
{
	USE_FLOAT_PARAM(v1)
	USE_FLOAT_PARAM(v2)
	ALPLCREAL_TYPE r;

	ALPLCMATH_ATAN2(r, v1, v2);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_CEIL(r,v)		r = (ALPLCREAL_TYPE) ceil( (float64_t) v )

//	Calculate ceil of the value
FLOAT_RET_TYPE AlPlcMath_Ceil(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_CEIL(r, v);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_COS(r,v)		r = (ALPLCREAL_TYPE) cos( (float64_t) v )

//	Calculate cos of the value
FLOAT_RET_TYPE AlPlcMath_Cos(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_COS(r, v);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_COSH(r,v)		r = (ALPLCREAL_TYPE) cosh( (float64_t) v )

//	Calculate cosh of the value
FLOAT_RET_TYPE AlPlcMath_Cosh(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_COSH(r, v);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_EXP(r,v)		r = (ALPLCREAL_TYPE) exp( (float64_t) v )

//	Calculate exponential function of the value
FLOAT_RET_TYPE AlPlcMath_Exp(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_EXP(r, v);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_FLOOR(r,v)	r = (ALPLCREAL_TYPE) floor( (float64_t) v )

//	Calculate floor of the value
FLOAT_RET_TYPE AlPlcMath_Floor(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_FLOOR(r, v);
	RETURN_FLOAT(r);
}



#define ALPLCMATH_LN(r,v)		r = (ALPLCREAL_TYPE) log( (float64_t) v )

//	Calculate natural logaritm of the value
FLOAT_RET_TYPE AlPlcMath_Ln(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_LN(r, v);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_LOG(r,v)		r = (ALPLCREAL_TYPE) log10( (float64_t) v )

//	Calculate logaritm base 10 of the value
FLOAT_RET_TYPE AlPlcMath_Log(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_LOG(r, v);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_SIN(r,v)		r = (ALPLCREAL_TYPE) sin( (float64_t) v )

//	Calculate sin of the value
FLOAT_RET_TYPE AlPlcMath_Sin(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_SIN(r, v);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_SINH(r,v)		r = (ALPLCREAL_TYPE) sinh( (float64_t) v )

//	Calculate sinh of the value
FLOAT_RET_TYPE AlPlcMath_Sinh(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_SINH(r, v);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_TAN(r,v)			r = (ALPLCREAL_TYPE) tan( (float64_t) v )

//	Calculate tan of the value
FLOAT_RET_TYPE AlPlcMath_Tan(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_TAN(r, v);
	RETURN_FLOAT(r);
}

#define ALPLCMATH_TANH(r,v)		r = (ALPLCREAL_TYPE) tanh( (float64_t) v )

	//	Calculate tanh of the value
FLOAT_RET_TYPE AlPlcMath_Tanh(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_TANH(r, v);
	RETURN_FLOAT(r);
}


#define ALPLCMATH_FABS(r,v)		r = (ALPLCREAL_TYPE) fabs( (float64_t) v )

//	Calculate fabs of the value
FLOAT_RET_TYPE AlPlcMath_Fabs(DECLARE_FLOAT_PARAM(v))
{
	USE_FLOAT_PARAM(v)
	ALPLCREAL_TYPE r;

	ALPLCMATH_FABS(r, v);
	RETURN_FLOAT(r);
}
