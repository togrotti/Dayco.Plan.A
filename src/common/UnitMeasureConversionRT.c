/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : UnitMeasureConversionRT.c                                  */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Fast UM Conversion                                         */
/*                                                                          */
/****************************************************************************/

//#include <xe167f.h>
#include <math.h>
#include "UnitMeasureConversion.h"

//***************************************************************************
// Avoids warning C37: nonstandard extension - unnamed parameter

//#pragma warning disable = 37

//***************************************************************************
// Conversion module 32 to 32
// parameters: mult, nshift, value, function selector

SLONG UmConv_Convert_32to32(SLONG mult, SWORD shift, SLONG value, UWORD sel)
{
//    __asm
//    {
//        mov     r2,[r0]
//
//        cmp     r2, #1
//        jmpr    cc_Z, r16to31
//        cmp     r2, #2
//        jmpr    cc_Z, r0to15
//        cmp     r2, #3
//        jmpr    cc_Z, l0to7
//        jmp     csel
//
//r16to31:
//        comulu  r8,r11
//
//        coashr  #16
//        comacsu r12,r8
//        comacsu r9,r11
//
//        coashr  #16
//        comac   r9,r12
//        cornd
//
//        coashr  #16
//        coashr  r10
//
//        costore r4,mal
//        costore r5,mah
//        _ret
//
//r0to15:
//        comulu  r8,r11
//
//        coashr  #16
//        comacsu r12,r8
//        comacsu r9,r11
//        cornd
//
//        coashr  #16
//        comac   r9,r12
//
//        coashr  r10
//
//        costore r4,mal
//        costore r5,mah
//        _ret
//
//l0to7:
//        bclr    MSW_MSL
//
//        comulu  r8,r11
//
//        coashr  #16
//        comacsu r12,r8
//        comacsu r9,r11
//        costore r8,mal
//
//        bset    MCW_MS
//
//        coashr  #16
//        comac   r9,r12
//
//        coshl   r10
//        jb      MSW_MSL, l0to7sat
//
//        bclr    MCW_MS
//
//        costore r5,mah
//
//        coashr  r10
//        coshl   #16
//        mov     r9,#0
//        coadd   r8,r9
//        coshl   r10
//
//        costore r4,mah
//        _ret
//
//l0to7sat:
//        costore r4,mal
//        costore r5,mah
//
//        bclr    MCW_MS
//        _ret
//
//csel:
//        cmp     r2, #4
//        jmpr    cc_Z, l8to15
//        cmp     r2, #5
//        jmpr    cc_Z, l16to23
//        cmp     r2, #6
//        jmpr    cc_Z, l24to31
//        _ret
//
//l8to15:
//        bclr    MSW_MSL
//
//        comulu  r8,r11
//
//        coashr  #16
//        comacsu r12,r8
//        comacsu r9,r11
//        costore r8,mal
//
//        bset    MCW_MS
//
//        coashr  #16
//        comac   r9,r12
//
//        coshl   #8
//        coshl   r10
//        jb      MSW_MSL, l8to15sat
//
//        add     r10,#8
//
//        bclr    MCW_MS
//
//        costore r5,mah
//
//        coashr  r10
//        coshl   #16
//        mov     r9,#0
//        coadd   r8,r9
//        coshl   r10
//
//        costore r4,mah
//        _ret
//
//l8to15sat:
//        costore r4,mal
//        costore r5,mah
//
//        bclr    MCW_MS
//        _ret
//
//l16to23:
//        bclr    MSW_MSL
//
//        comulu  r8,r11
//        costore r6,mal
//
//        coashr  #16
//        comacsu r12,r8
//        comacsu r9,r11
//        costore r7,mal
//
//        bset    MCW_MS
//
//        coashr  #16
//        comac   r9,r12
//
//        mov     r9,#0
//
//        coshl   #8
//        coshl   #8
//        jb      MSW_MSL, l16to23sat
//
//        coadd   r7,r9
//        coshl   r10
//        jb      MSW_MSL, l16to23sat
//
//        bclr    MCW_MS
//
//        costore r5,mah
//
//        coashr  r10
//        coshl   #16
//        coadd   r6,r9
//        coshl   r10
//
//        costore r4,mah
//        _ret
//
//l16to23sat:
//        costore r4,mal
//        costore r5,mah
//
//        bclr    MCW_MS
//        _ret
//
//l24to31:
//        bclr    MSW_MSL
//
//        comulu  r8,r11
//        costore r6,mal
//
//        coashr  #16
//        comacsu r12,r8
//        comacsu r9,r11
//        costore r7,mal
//
//        bset    MCW_MS
//
//        coashr  #16
//        comac   r9,r12
//
//        mov     r9,#0
//
//        coshl   #8
//        coshl   #8
//        jb      MSW_MSL, l24to31sat
//
//        coadd   r7,r9
//        coshl   #8
//        coshl   r10
//        jb      MSW_MSL, l24to31sat
//
//        add     r10,#8
//
//        bclr    MCW_MS
//
//        costore r5,mah
//
//        coashr  r10
//        coshl   #16
//        coadd   r6,r9
//        coshl   r10
//
//        costore r4,mah
//        _ret
//
//l24to31sat:
//        costore r4,mal
//        costore r5,mah
//
//        bclr    MCW_MS
//        _ret
//    }
}

//***************************************************************************
// Conversion module 16 to 32
// parameters: mult, nshift, value, function selector
// mathematical: Value_32 = ((Value_16 * 2^shift) / mult) * 65536
// code optimiz: Value_32 = ((Value_16 * 2^shift) * mult) / 65536
SLONG UmConv_Convert_16to32(SLONG mult, SWORD shift, SWORD value, UWORD sel)
{
	long res;
	SWORD nshift;
//	FLOAT flRes ;

	if( sel == 1 )
		nshift = -shift+16;
	else if( sel == 2 )
		nshift = -shift;
	else if( sel == 3 )
		nshift = shift;
	else if( sel == 4 )
		nshift = shift-8;
	else if( sel == 5 )
		nshift = shift-16;
	else if( sel == 6 )
		nshift = shift-24;

	res = (SLONG)(((SLLNG)value * mult) >> (16 - nshift)) ; //16 to 32
	return res ;

//	flRes = (FLOAT)value * (FLOAT)mult * pow(2, (nshift - 16)) ;
//	return (SLONG)flRes0 ;
}

//***************************************************************************
// Conversion module 32 to 16
// parameters: mult, nshift, value, function selector
// Value_16 = ((Value_32 * mult) / (2^shift)) / 65536

SWORD UmConv_Convert_32to16(SWORD mult, SWORD shift, SLONG value, UWORD sel)
{
	long res;
	SWORD nshift;

	if( sel == 1 )
		nshift = shift+16;
	else if( sel == 2 )
		nshift = shift;
	else if( sel == 3 )
		nshift = -shift;
	else if( sel == 4 )
		nshift = -shift-8;
	else if( sel == 5 )
		nshift = -shift-16;
	else if( sel == 6 )
		nshift = -shift-24;

	res = ((SLLNG)(value>>nshift) * mult) >> 16; // 32 to 16

	return (SWORD)res;

//	   if(*pswShift>-32 && *pswShift<=-16)
//	    {
//	        *puwSelection=1;
//	        *pswShift=-*pswShift-16;
//	    }
//	    else if(*pswShift>-16 && *pswShift<=0)
//	    {
//	        *puwSelection=2;
//	        *pswShift=-*pswShift;
//	    }
//	    else if(*pswShift>0 && *pswShift<8)
//	    {
//	        *puwSelection=3;
//	        *pswShift=*pswShift;
//	    }
//	    else if(*pswShift>=8 && *pswShift<16)
//	    {
//	        *puwSelection=4;
//	        *pswShift=*pswShift-8;
//	    }
//	    else if(*pswShift>=16 && *pswShift<24)
//	    {
//	        *puwSelection=5;
//	        *pswShift=*pswShift-16;
//	    }
//	    else if(*pswShift>=24 && *pswShift<32)
//	    {
//	        *puwSelection=6;
//	        *pswShift=*pswShift-24;
//	    }



//    __asm
//    {
//        cmp     r12, #1
//        jmpr    cc_Z, r16to31
//        cmp     r12, #2
//        jmpr    cc_Z, r0to15
//        cmp     r12, #3
//        jmpr    cc_Z, l0to7
//        cmp     r12, #4
//        jmpr    cc_Z, l8to15
//        _ret
//
//r16to31:
//        comulsu r8,r10
//
//        coashr  #16
//        comac   r8,r11
//
//        bset    MCW_MS
//
//        coashr  #16
//        coashr  r9
//
//        coshl   #8
//        coshl   #8
//
//        costore r4,mah
//        bclr    MCW_MS
//        _ret
//
//r0to15:
//        comulsu r8,r10
//
//        coashr  #16
//        comac   r8,r11
//
//        bset    MCW_MS
//
//        coashr  r9
//
//        coshl   #8
//        coshl   #8
//
//        costore r4,mah
//        bclr    MCW_MS
//        _ret
//
//l0to7:
//        bclr    MSW_MSL
//        mov     r3,#0
//
//        comulsu r8,r10
//        costore r2,mal
//
//        coashr  #16
//        comac   r8,r11
//
//        bset    MCW_MS
//
//        coshl   #8
//        coshl   #8
//        jb      MSW_MSL, l0to7sat
//
//        coadd   r2,r3
//
//        coshl   r9
//
//        costore r4,mah
//        bclr    MCW_MS
//        _ret
//
//l0to7sat:
//        costore r4,mah
//
//        bclr    MCW_MS
//        _ret
//
//l8to15:
//        bclr    MSW_MSL
//        mov     r3,#0
//
//        comulsu r8,r10
//        costore r2,mal
//
//        coashr  #16
//        comac   r8,r11
//
//        bset    MCW_MS
//
//        coshl   #8
//        coshl   #8
//        jb      MSW_MSL, l8to15sat
//
//        coadd   r2,r3
//
//        coshl   #8
//        coshl   r9
//
//        costore r4,mah
//        bclr    MCW_MS
//        _ret
//
//l8to15sat:
//        costore r4,mah
//
//        bclr    MCW_MS
//        _ret
//
//    }
}
