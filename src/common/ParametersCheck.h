/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ParametersCheck.h                                          */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Common parameters validity checker                         */
/*                                                                          */
/****************************************************************************/

#ifndef _PARAMETERSCHECK_H
#define _PARAMETERSCHECK_H

//***************************************************************************
// Defines

#define PARCC_GROUPTHRESHOLD            5000

//***************************************************************************
// Globals

extern UWORD uwParChkParamCode;
extern ULONG ulParChkLastUpdateTime;

//***************************************************************************
// Entry points

BOOL ParChk_Init(void);
void ParChk_SignalValueError(UWORD);
void ParChk_ResetValueError(UWORD);
UWORD ParChk_GetValueErrorList(HPUWORD hpuwList, UWORD uwBufSize);

#endif
