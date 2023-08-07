/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2014, Phase Motion Control. All Rights Reserved.             */
/*                                                                          */
/* File        : HiperfaceEncoder.h                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Hiperface encoder manager                                  */
/*                                                                          */
/****************************************************************************/

#ifndef _HIPERFACEENCODER_H
#define _HIPERFACEENCODER_H

//***************************************************************************
// Defines

#define HIPFC_PLC_SETPOS_KEY                0x68F6

//***************************************************************************
// Parameters data structure

typedef struct {
    ULONG ulMTurnStartPos;
} HIPFC_PARAMS;

//****************************************************************************
// General global variables

extern HIPFC_PARAMS sHf_Params;
#ifdef _INFINEON_
extern const HIPFC_PARAMS huge sHf_DefParams;
#else
extern const HIPFC_PARAMS      sHf_DefParams;
#endif
//****************************************************************************
// Global functions

BOOL Hf_Init(ENCMGR_SPACEFEEDBACK * psFeedback, SBYTE * psbEncMgrFault, ULONG ulRTDisMask);
BOOL Hf_ParametersCheck(void); 
void Hf_SetElecAngle(UWORD uwElecAngle);

//***************************************************************************
// PLC management functions

#ifdef _INFINEON_
UWORD PlcHipfcSetPosition(UWORD uwKey, SQWRD huge * sqPos);
#else
UWORD PlcHipfcSetPosition(UWORD uwKey, SQWRD      * sqPos);
#endif

#endif
