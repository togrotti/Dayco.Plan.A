/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : DspFunctions.c                                             */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Optimized generic math routines for integer operations     */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

//#include <xe167f.h>
#include "DspFunctions.h"
#include <math.h>

//***************************************************************************
// Avoids warning C47: unreferenced parameter

//#pragma warning disable = 47

//***************************************************************************
// s32bit => s16bit

SWORD _sint32toSint16(SLONG slVar)
{
//    __asm
//    {
//        bset    MCW_MS
//
//        coload  r8,r9
//
//        coshl   #8
//        coshl   #8
//
//        costore r4,mah
//
//        bclr    MCW_MS
//    }
	return (SWORD)(slVar >> 16) ; // ?? understand correctly ??
}

//***************************************************************************
// s16bit => s32bit

SLONG _sint16toSint32(SWORD swVar)
{
//    __asm
//    {
//        coload  r9,r8
//
//        coashr  #16
//
//        costore r4,mal
//        costore r5,mah
//    }
	return ((SLONG)swVar << 16) ; // ?? understand correctly ??
}

//***************************************************************************
// s32bit = s32bit * s32bit/Q16
// NB: no saturation checking is performed here

SLONG _sint32_scale_32(SLONG slVar, SLONG slScale)
{
//    __asm
//    {
//        comulu  r8,r10
//
//        coashr  #16
//        comacsu r11,r8
//        comacsu r9,r10
//        costore r4,mal
//
//        coashr  #16
//        comac   r11,r9
//        costore r5,mal
//     }

	return (SLONG)(((SLLNG)slVar * slScale) >> 16) ;
}

//***************************************************************************
// s32bit = s32bit + s16bit * s16bit (Q15)
// NB: no saturation checking is performed here

SLONG _sint32_mac_16_16(SLONG slVar, SWORD swMul, SWORD swScale)
{
//    __asm
//    {
//        comul   r10,r11
//        coshl   #1
//        coadd   r8,r9
//        costore r4,mal
//        costore r5,mah
//     }

	return (slVar + (((SLONG)swMul * swScale) << 1) ) ;
}

//***************************************************************************
// s16bit = ( s16bit * s16bit (Q15) )>>16
// NB: no saturation checking is performed here

SWORD _sint16_mpy_16_16(SWORD swMul1, SWORD swMul2)
{
//    __asm
//    {
//        comul   r8,r9
//        coshl   #1
//        costore r4,mah
//     }
	return ( (SWORD)( ((SLONG)swMul1 * swMul2) >> 15 ) ) ;
}

//***************************************************************************
// s16bit = (s16bit * s16bit (Q14))>>16
// NB: no saturation checking is performed here

SWORD _sint16_mpy_16_q14(SWORD swMul1, SWORD swMul2)
{
//    __asm
//    {
//        comul   r8,r9
//        coshl   #2
//        costore r4,mah
//     }
	return ( (SWORD)( ((SLONG)swMul1 * swMul2) >> 14 ) ) ;
}

//***************************************************************************
// Evaluate a=|(v-ve)*(v+ve)/2s|<<8
// NB: no saturation checking is performed here
SLONG _sva_evaluation(SLONG slV, SLONG slVE, SQWRD * sqS)
{
	SLLNG  sllSpace = MAKE_INT64(sqS->hi, sqS->lo)*2 ;

	double  flAccSve = fabs((double)((SLLNG)(slV - slVE)) * (double)((SLLNG)slV + slVE) / (double)sllSpace) * 4096.0 ; // use double?

	return ((SLONG)flAccSve) ;

/*	  __asm
	    {
	        push    r13

	//-------------------------------------- |S| calc

	        mov     r1,[r12+#06]
	        jmpr    cc_N,sneg

	        mov     r1,[r12+]
	        mov     r2,[r12+]
	        mov     r3,[r12+]
	        mov     r4,[r12]
	        jmp     snorm

	sneg:
	        mov     r5,[r12+]
	        mov     r6,[r12+]
	        mov     r7,[r12+]
	        mov     r13,[r12]

	        mov     r1,#0
	        mov     r2,#0
	        mov     r3,#0
	        mov     r4,#0

	        sub     r1,r5
	        subc    r2,r6
	        subc    r3,r7
	        subc    r4,r13

	//-------------------------------------- |S| normalization

	snorm:
	        mov     r6,#48

	        coload  r3,r4
	        prior   r5,r4
	        jmpr    cc_NZ,snormf

	        sub     r6,#16
	        coload  r2,r3
	        prior   r5,r3
	        jmpr    cc_NZ,snormf

	        sub     r6,#16
	        coload  r1,r2
	        prior   r5,r2
	        jmpr    cc_NZ,snormf

	        mov     r13,#0
	        sub     r6,#16
	        coload  r13,r1
	        prior   r5,r1

	snormf:
	        coshl   r5
	        sub     r6,r5
	        neg     r6      // r6 <= -nshift
	        costore r13,mah // r13 <= S 16bit mantissa

	//-------------------------------------- (V-VE)*(V+VE)

	        coload  r8,r9
	        cosub   r10,r11
	        costore r8,mal
	        costore r9,mah  // r8|r9 <= V-VE

	        coadd2  r10,r11
	        costore r10,mal
	        costore r11,mah // r10|r11 <= V+VE

	        comulu  r8,r10
	        costore r1,mal

	        coashr  #16
	        comacsu r11,r8
	        comacsu r9,r10
	        costore r2,mal

	        coashr  #16
	        comac   r9,r11
	        costore r3,mal
	        costore r4,mah  // r1|r2|r3|r4 <= (V-VE)*(V+VE)

	//-------------------------------------- |(V-VE)*(V+VE)| calc

	        cmp     r4,#0
	        jmpr    cc_NN,vvenorm

	        push    r6      // save r6 in order to use it while
	                        // making modulo
	        mov     r5,r1
	        mov     r6,r2
	        mov     r7,r3
	        mov     r8,r4

	        mov     r1,#0
	        mov     r2,#0
	        mov     r3,#0
	        mov     r4,#0

	        sub     r1,r5
	        subc    r2,r6
	        subc    r3,r7
	        subc    r4,r8

	        pop     r6

	//-------------------------------------- |(V-VE)*(V+VE)| normalization

	vvenorm:
	        add     r6,#48

	        mov     r8,r2
	        mov     r9,r3
	        coload  r3,r4
	        prior   r5,r4
	        jmpr    cc_NZ,vvenormf

	        sub     r6,#16
	        mov     r8,r1
	        mov     r9,r2
	        coload  r2,r3
	        prior   r5,r3
	        jmpr    cc_NZ,vvenormf

	        sub     r6,#16
	        mov     r8,#0
	        mov     r9,r1
	        coload  r1,r2
	        prior   r5,r2
	        jmpr    cc_NZ,vvenormf

	        sub     r6,#16
	        mov     r8,#0
	        mov     r9,#0
	        coload  r8,r1
	        prior   r5,r1

	vvenormf:
	        coshl   r5
	        costore r2,mah
	        coload  r8,r9
	        coshl   r5
	        costore r1,mah  // r1|r2 <= |(V-VE)*(V+VE)| 32bit mantissa

	        coload  r1,r2
	        coshr   #1
	        costore r2,mah
	        and     r2,#0x7fff
	        costore r1,mal  // r1|r2 <= |(V-VE)*(V+VE)|/2 32bit mantissa

	        sub     r6,r5   // r6 <= final nshift

	        add     r6,#12   // enhance resolution by adding 12 LSB

	//-------------------------------------- division

	        sub     r6,#16

	        mov     MDH,r2
	        mov     MDL,r1
	        divlu   r13     // MDL <= quotient, read as later as possible
	                        // in order to get advantage from parallel
	                        // division hardware

	//-------------------------------------- sat/zero checking

	        cmp     r6,#-16
	        jmpr    cc_SLE,satzero
	        cmp     r6,#16
	        jmpr    cc_SLT,nosat

	        mov     r4,#0xffff
	        mov     r5,#0x7fff
	        jmp     fcend

	satzero:
	        mov     r4,#0
	        mov     r5,#0
	        jmp     fcend

	nosat:

	//-------------------------------------- final shifting

	        mov     r2,#0

	        cmp     r6,#0
	        jmpr    cc_N,fshneg

	        mov     r3,MDL
	        coload  r3,r2

	        coshl   r6

	        costore r4,mal
	        costore r5,mah
	        jmp     fcend

	fshneg:
	        neg     r6

	        mov     r3,MDL
	        coload  r3,r2

	        coashr  r6

	        costore r4,mal
	        costore r5,mah

	//-------------------------------------- end

	fcend:
	        pop     r13
	    }
*/
}

//***************************************************************************
// Interpolation: pwsDat[0]+(pswDat[1]-pwsDat[0])*swScale
// swScale: Q12

SWORD _interpolation(SWORD  * hpswDat, SWORD swScale)
{
//    __asm
//    {
//        exts    r9,#2
//        mov     r1,[r8+]
//        mov     r2,[r8+]
//        mov     r5,#0
//
//        coload  r5,r2
//        cosub   r5,r1
//
//        costore r5,mah
//
//        comul   r5,r10
//        coshl   #4
//
//        coadd   r5,r1
//
//        costore r4,mas
//    }

	SWORD swD0, swD1 ;

	swD0 = *hpswDat ;
	hpswDat++ ;
	swD1 = *hpswDat ;

	return (swD0 + (SWORD)(((SLONG)(swD1 - swD0) * swScale) >> 12)) ;
}

//***************************************************************************
// Interpolation: hpswDat[n]+(hpswDat[n+1]-hpswDat[n])*s
// n:     uwLinSize  MSB bits
// s: (16-uwLinSize) LSB bits

#ifdef _APP_XC
SWORD _interp1d(SWORD  * hpswDat, UWORD uwLinSize, SWORD swVal)
{
#ifdef _INFINEON_
   __asm
   {
       mov     r4,#16
       sub     r4,r10

       // compute array offset
       mov     r3,r11
       add     r3,#0x8000
       shr     r3,r4
       shl     r3,#1
       add     r8,r3

       // retrieve adjacent values
       exts    r9,#2
       mov     r1,[r8+]
       mov     r2,[r8+]

       // compute difference
       mov     r6,#0
       coload  r6,r2
       cosub   r6,r1
       costore r5,mas

       // extract multiplier
       mov     r3,#1
       shl     r3,r4
       sub     r3,#1
       and     r3,r11
       comul   r5,r3

       // fractional shifting
       coshl   r10

       // add difference and retrieve result
       coadd   r6,r1
       costore r4,mas
   }
#else
   return 0;
#endif
}

//***************************************************************************
// Values interpolation

SWORD _interpv(SWORD v0, SWORD v1, UWORD uwLinSize, SWORD swVal)
{
#ifdef _INFINEON_
   __asm
   {
       mov     r4,#16
       sub     r4,r10

       // compute difference
       mov     r6,#0
       coload  r6,r9
       cosub   r6,r8
       costore r5,mas

       // extract multiplier
       mov     r3,#1
       shl     r3,r4
       sub     r3,#1
       and     r3,r11
       comul   r5,r3

       // fractional shifting
       coshl   r10

       // add difference and retrieve result
       coadd   r6,r8
       costore r4,mas
   }
#else
   return 0;
#endif
}

//***************************************************************************
// Two-dimension interpolation

SWORD _interp2d(SWORD  * hpswDat, UWORD uwLinSize, SWORD swVal0, SWORD swVal1)
{
   UWORD o=((UWORD)swVal1+0x8000)>>(16-uwLinSize);
   UWORD s=(1<<uwLinSize)+1;
   SWORD v0,v1;

   hpswDat=&hpswDat[o*s];
   v0=_interp1d( hpswDat,   uwLinSize,swVal0);
   v1=_interp1d(&hpswDat[s],uwLinSize,swVal0);
   return _interpv(v0,v1,uwLinSize,swVal1);
}

//***************************************************************************
// Three-dimension interpolation

SWORD _interp3d(SWORD  * hpswDat, UWORD uwLinSize, SWORD swVal0, SWORD swVal1, SWORD swVal2)
{
   UWORD o=((UWORD)swVal2+0x8000)>>(16-uwLinSize);
   UWORD s=(1<<uwLinSize)+1;
   SWORD v0,v1;

   s=s*s;
   hpswDat=&hpswDat[o*s];
   v0=_interp2d( hpswDat,   uwLinSize,swVal0,swVal1);
   v1=_interp2d(&hpswDat[s],uwLinSize,swVal0,swVal1);
   return _interpv(v0,v1,uwLinSize,swVal2);
}
#endif // _APP_XC

//***************************************************************************
// s16bit = (s16bit - s16bit) * s16bit (Q13)

SWORD _sint16_offsetscale_16(SWORD swVar, SWORD swOffset, SWORD swScale)
{
//    __asm
//    {
//        mov     r5,#0
//
//        coload  r5,r8
//        cosub   r5,r9
//
//        costore r5,mah
//
//        comul   r5,r10
//        coshl   #3
//
//        costore r4,mas
//     }
	return ((SWORD)(((SLONG)(swVar - swOffset) * swScale) >> 13)) ;
}

//***************************************************************************
// s32bit << 12 => s32bit

SLONG _sint32_asl12(SLONG slVar)
{
//    __asm
//    {
//        coload  r8,r9
//
//        coshl   #8
//        coshl   #4
//
//        costore r4,mal
//        costore r5,mas
//    }

	SLLNG sllVar = (SLLNG)slVar << 12 ;

	if (sllVar > SLONG_MAX_VALUE)
		return SLONG_MAX_VALUE ;
	else if (sllVar < SLONG_MIN_VALUE)
		return SLONG_MIN_VALUE ;
	else
		return (SLONG)sllVar  ;
}

//***************************************************************************
// s32bit >> u16bit => s16bit

SWORD _sint32_asr_16(SLONG slVar, UWORD uwCnt)
{
//    __asm
//    {
//        coload  r8,r9
//
//        coashr  r10
//
//        costore r4,mal
//    }
	return ((SWORD)(slVar >> uwCnt)) ;
}

//***************************************************************************
// s16bit << u16bit => s32bit

SLONG _sint16_asl_32(SWORD swVar, UWORD uwCnt)
{
//    __asm
//    {
//        coload  r9,r8
//        coashr  #16
//
//        coshl   r9
//
//        costore r4,mal
//        costore r5,mas
//    }
	return ((SLONG)swVar << uwCnt) ;
}

//***************************************************************************
// s16bit = s16bit - s16bit

SWORD _sint16_sublim_16(SWORD swVar1, SWORD swVar2)
{
//    __asm
//    {
//        mov     r5,#0
//
//        coload  r5,r8
//        cosub   r5,r9
//
//        costore r4,mas
//     }
	SLONG slSum16Lim = (SLONG)swVar1 - swVar2 ;

	if (slSum16Lim > SWORD_MAX_VALUE)
		return SWORD_MAX_VALUE ;
	else if (slSum16Lim < SWORD_MIN_VALUE)
		return SWORD_MIN_VALUE ;
	else
		return (SWORD)slSum16Lim ;
}

//***************************************************************************
// s16bit = s16bit + s16bit

SWORD _sint16_addlim_16(SWORD swVar1, SWORD swVar2)
{
//    __asm
//    {
//        mov     r5,#0
//
//        coload  r5,r8
//        coadd   r5,r9
//
//        costore r4,mas
//     }
	SLONG slSum16Lim = (SLONG)swVar1 + swVar2 ;

	if (slSum16Lim > SWORD_MAX_VALUE)
		return SWORD_MAX_VALUE ;
	else if (slSum16Lim < SWORD_MIN_VALUE)
		return SWORD_MIN_VALUE ;
	else
		return (SWORD)slSum16Lim ;
}

//***************************************************************************
// s32bit = s32bit * s16bit/Q15
// NB: no saturation checking is performed here

SLONG _sint32_scale_16(SLONG slVar, SWORD swScale)
{
//    __asm
//    {
//        comulu  r8,r10
//        cornd
//
//        coashr  #16
//        comac   r9,r10
//
//        coshl  #1
//        costore r4,mal
//        costore r5,mah
//     }

	return ( (SLONG)( ((SLLNG)slVar * swScale) >> 15 ) ) ;
}

