/******************************************************************************
	AlPlcReal.c
******************************************************************************/

/******************************************************************************
	include
******************************************************************************/
#pragma GCC optimize (2)
#include "AlPlcReal.h"


/******************************************************************************
	global function implementation
******************************************************************************/

/*	U D I N T   T O   R E A L	*/

FLOAT_RET_TYPE AlPlcReal_UdintToReal( uint32_t i )
{
	RETURN_FLOAT((float32_t)i);
}

/*	D I N T   T O   R E A L	*/

FLOAT_RET_TYPE AlPlcReal_DintToReal( int32_t i )
{
	RETURN_FLOAT((float32_t)i);
}

/*	R E A L	  T O   U D I N T	*/

uint32_t AlPlcReal_RealToUdint( DECLARE_FLOAT_PARAM( f ) )
{
	USE_FLOAT_PARAM(f)
	return (uint32_t)f;
}

/*	R E A L	  T O   D I N T	*/

int32_t AlPlcReal_RealToDint( DECLARE_FLOAT_PARAM( f ) )
{
	USE_FLOAT_PARAM(f)
	return (int32_t)f;
}

/*	A D D	*/

FLOAT_RET_TYPE AlPlcReal_Add( DECLARE_FLOAT_PARAM( add1 ), DECLARE_FLOAT_PARAM( add2 ))
{
	USE_FLOAT_PARAM(add1);
	USE_FLOAT_PARAM(add2);
	RETURN_FLOAT((float32_t) ( add1 + add2 ));
}

/*	S U B	*/

FLOAT_RET_TYPE AlPlcReal_Sub( DECLARE_FLOAT_PARAM( subd ), DECLARE_FLOAT_PARAM( subr ))
{
	USE_FLOAT_PARAM(subd);
	USE_FLOAT_PARAM(subr);
	RETURN_FLOAT((float32_t) ( subd - subr ));
}

/*	M U L	*/

FLOAT_RET_TYPE AlPlcReal_Mul( DECLARE_FLOAT_PARAM(fac1), DECLARE_FLOAT_PARAM(fac2) )
{
	USE_FLOAT_PARAM(fac1);
	USE_FLOAT_PARAM(fac2);
	RETURN_FLOAT((float32_t) ( fac1 * fac2 ));
}

/*	D I V	*/

FLOAT_RET_TYPE AlPlcReal_Div( DECLARE_FLOAT_PARAM( divd ), DECLARE_FLOAT_PARAM( divr ) )
{
	USE_FLOAT_PARAM(divd);
	USE_FLOAT_PARAM(divr);
	RETURN_FLOAT((float32_t)divd / divr);
}

/*	G T	*/

uint8_t AlPlcReal_Gt( DECLARE_FLOAT_PARAM( cmp1 ), DECLARE_FLOAT_PARAM( cmp2 ) )
{
	USE_FLOAT_PARAM(cmp1);
	USE_FLOAT_PARAM(cmp2);
	if ( cmp1 > cmp2 )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*	G E	*/

uint8_t AlPlcReal_Ge( DECLARE_FLOAT_PARAM( cmp1 ), DECLARE_FLOAT_PARAM( cmp2 ) )
{
	USE_FLOAT_PARAM(cmp1);
	USE_FLOAT_PARAM(cmp2);
	if ( cmp1 >= cmp2 )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*	E Q	*/

uint8_t AlPlcReal_Eq( DECLARE_FLOAT_PARAM( cmp1 ), DECLARE_FLOAT_PARAM( cmp2 ) )
{
	USE_FLOAT_PARAM(cmp1);
	USE_FLOAT_PARAM(cmp2);
	if ( cmp1 == cmp2 )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


/*	L T	*/

uint8_t AlPlcReal_Lt( DECLARE_FLOAT_PARAM( cmp1 ), DECLARE_FLOAT_PARAM( cmp2 ) )
{
	USE_FLOAT_PARAM(cmp1);
	USE_FLOAT_PARAM(cmp2);
	if ( cmp1 < cmp2 )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*	L E	*/

uint8_t AlPlcReal_Le( DECLARE_FLOAT_PARAM( cmp1 ), DECLARE_FLOAT_PARAM( cmp2 ) )
{
	USE_FLOAT_PARAM(cmp1);
	USE_FLOAT_PARAM(cmp2);
	if ( cmp1 <= cmp2 )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*	N E	*/

uint8_t AlPlcReal_Ne( DECLARE_FLOAT_PARAM( cmp1 ), DECLARE_FLOAT_PARAM( cmp2 ) )
{
	USE_FLOAT_PARAM(cmp1);
	USE_FLOAT_PARAM(cmp2);
	if ( cmp1 != cmp2 )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

