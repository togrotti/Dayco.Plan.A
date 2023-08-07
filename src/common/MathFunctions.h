/////////////////////////////////////////////////////////////////////////////
//

#ifndef _MATH_FUNCTIONS_H
 #define _MATH_FUNCTIONS_H

/////////////////////////////////////////////////////////////////////////////
//

UWORD ATan16( SWORD swSIN, SWORD swCOS );
SWORD Sin16(  SWORD uwAngle );
SWORD Cos16(  SWORD uwAngle );


// UWORD  ComputeAngle( SWORD swSIN, SWORD swCOS, SWORD swMode );
// SWORD  ComputeSIN( UWORD uwAngle, SWORD swMode );
// SWORD  ComputeCOS( UWORD uwAngle, SWORD swMode );

SWORD _ilog2(ULONG ulArgument) ;  /* return the position of the highest bit set */
//UWORD _SquareRoot16_LookUpTable(UWORD uwValue) ;
UWORD _SquareRoot16_4096(ULONG ulValue) ;
UWORD CalculateSquareRoot(SWORD swSin, SWORD swCos) ;
SWORD RootOfSquareSum16(SWORD swX, SWORD swY) ; // SQRT(X^2+Y^2)
#endif // _MATH_FUNCTIONS_H
