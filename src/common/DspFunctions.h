/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : DspFunctions.h                                             */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Optimized generic math routines for integer operations     */
/*                                                                          */
/****************************************************************************/

#ifndef _DSPFUNCTIONS_H
#define _DSPFUNCTIONS_H

#include "CommonDefines.h"

//***************************************************************************
// Conversion

SWORD _sint32toSint16(SLONG slVar );
SLONG _sint16toSint32(SWORD swVar );

SLONG _sint32_scale_32(SLONG ulVar, SLONG ulScale);
SLONG _sint32_scale_16(SLONG slVar, SWORD swScale);
SLONG _sint32_mac_16_16(SLONG slVar, SWORD swMul, SWORD swScale);
SWORD _sint16_mpy_16_16(SWORD swMul1, SWORD swMul2);
SWORD _sint16_mpy_16_q14(SWORD swMul1, SWORD swMul2);

SWORD _sint16_offsetscale_16(SWORD swVar, SWORD swOffset, SWORD swScale);

SLONG _sint32_asl12(SLONG slVar);
SWORD _sint32_asr_16(SLONG slVar, UWORD uwCnt);
SLONG _sint16_asl_32(SWORD swVar, UWORD uwCnt);

//***************************************************************************
// Evaluate a=(v-ve)*(v+ve)/2s or s=(v-ve)*(v+ve)/2a

SLONG _sva_evaluation(SLONG slV, SLONG slVE, SQWRD * sqS);

//***************************************************************************
// Interpolation: pwsDat[0]+(pswDat[1]-pwsDat[0])*swScale
// swScale: Q12

SWORD _interpolation(SWORD  * hpswDat, SWORD swScale);

#ifdef _APP_XC
SWORD _interp1d(SWORD  * hpswDat, UWORD uwLinSize, SWORD swVal);
SWORD _interpv(SWORD v0, SWORD v1, UWORD uwLinSize, SWORD swVal);
SWORD _interp2d(SWORD  * hpswDat, UWORD uwLinSize, SWORD swVal0, SWORD swVal1);
SWORD _interp3d(SWORD  * hpswDat, UWORD uwLinSize, SWORD swVal0, SWORD swVal1, SWORD swVal2);
#endif

//***************************************************************************
// Limiting arithemtic operators

SWORD _sint16_sublim_16(SWORD swVar1, SWORD swVar2);
SWORD _sint16_addlim_16(SWORD swVar1, SWORD swVar2);

#endif
