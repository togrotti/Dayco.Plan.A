/*------------------------------------------------------------------------------
**
**	Copyright:	AXEL s.r.l. 2010
**
**	AlPlcReal.h:	Real management plugin
**
**-----------------------------------------------------------------------------
**
**	IMPORTANT:
**	THIS MODULE SHOULDN'T BE MODIFIED BY THE CUSTOMER
**
**-----------------------------------------------------------------------------*/

#ifndef _ALPLCREAL_H
#define _ALPLCREAL_H

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

/* Convert unsigned int 32 to real */
extern FLOAT_RET_TYPE AlPlcReal_UdintToReal( uint32_t i );

/* Convert int 32 to real */
extern FLOAT_RET_TYPE AlPlcReal_DintToReal( int32_t i );

/* Convert real to unsigned int 32 */
extern uint32_t AlPlcReal_RealToUdint( DECLARE_FLOAT_PARAM( f ) );

/* Convert real to signed int 32 */
extern int32_t AlPlcReal_RealToDint( DECLARE_FLOAT_PARAM( f ) );

/* Sum of two real numbers */
extern FLOAT_RET_TYPE AlPlcReal_Add( DECLARE_FLOAT_PARAM(add1), DECLARE_FLOAT_PARAM(add2) );

/* Subtracts two real numbers */
extern FLOAT_RET_TYPE AlPlcReal_Sub( DECLARE_FLOAT_PARAM(subd), DECLARE_FLOAT_PARAM(subr) );

/* Multiply two real numbers */
extern FLOAT_RET_TYPE AlPlcReal_Mul( DECLARE_FLOAT_PARAM(fac1), DECLARE_FLOAT_PARAM(fac2) );

/* Divides two real numbers */
extern FLOAT_RET_TYPE AlPlcReal_Div( DECLARE_FLOAT_PARAM( divd ), DECLARE_FLOAT_PARAM( divr ) );

/* cmp1 > cmp2 */
extern uint8_t AlPlcReal_Gt( DECLARE_FLOAT_PARAM( cmp1 ), DECLARE_FLOAT_PARAM( cmp2 ) );

/* cmp1 >= cmp2 */
extern uint8_t AlPlcReal_Ge( DECLARE_FLOAT_PARAM( cmp1 ), DECLARE_FLOAT_PARAM( cmp2 ) );

/* cmp1 == cmp2 */
extern uint8_t AlPlcReal_Eq( DECLARE_FLOAT_PARAM( cmp1 ), DECLARE_FLOAT_PARAM( cmp2 ) );

/* cmp1 < cmp2 */
extern uint8_t AlPlcReal_Lt( DECLARE_FLOAT_PARAM( cmp1 ), DECLARE_FLOAT_PARAM( cmp2 ) );

/* cmp1 <= cmp2 */
extern uint8_t AlPlcReal_Le( DECLARE_FLOAT_PARAM( cmp1 ), DECLARE_FLOAT_PARAM( cmp2 ) );

/* cmp1 != cmp2 */
extern uint8_t AlPlcReal_Ne( DECLARE_FLOAT_PARAM( cmp1 ), DECLARE_FLOAT_PARAM( cmp2 ) );

#ifdef __cplusplus
}
#endif

#endif	//	_ALPLCREAL_H
