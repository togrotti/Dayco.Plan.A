/****************************************************************************/
/* Project: AxN Control Board                                               */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved.  */
/*                                                                          */
/* File        : BissEncoder.h                                              */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Biss Encoder management functions                          */
/*                                                                          */
/****************************************************************************/

#ifndef _BISSENCODER_H
#define _BISSENCODER_H

//***************************************************************************
// Defines

#define BISS_SEL_MAIN              0
#define BISS_SEL_AUX               1

#define BISS_MAX_ENCODER           2

//***************************************************************************
// Output data structures

typedef struct {
    UWORD uwCRCErrorCounter;
    UWORD uwMaxFrequency;         // [kHz]
    UBYTE ubStepPerRevBits;
    UBYTE ubRevNumBits;
} BISS_OUT ;

//***************************************************************************
// Parameters data structure

typedef struct {
    union {
        struct {
            UWORD bDummy : 1 ;
            UWORD bDisAlarm : 1 ;
        } b ;
        UWORD w ;
    } flags ;
    
    UBYTE ubType;
    ULONG ulClockFreq;            // [kHz]
    ULONG ulMTurnStartPos;
} BISS_PARAMS ;

//****************************************************************************
// General global variables

extern BISS_PARAMS sBiss_Params[BISS_MAX_ENCODER];
#ifdef _INFINEON_
extern const BISS_PARAMS huge sBiss_DefParams ;
#else
extern const BISS_PARAMS      sBiss_DefParams ;
#endif
extern BISS_OUT sBiss_DataOut[BISS_MAX_ENCODER];

//****************************************************************************
// Global functions

BOOL Biss_Init(UWORD, ENCMGR_SPACEFEEDBACK *, SBYTE *, ULONG); 
BOOL Biss_ParametersCheck(UWORD) ; 
void Biss_SetElecAngle(UWORD);

//***************************************************************************
// PLC management functions

ULONG PlcBissSetCommandMode(UWORD uwChannelSel, BOOL bSet);
ULONG PlcBissResetEncoder(UWORD uwChannelSel) ;
ULONG PlcBissResetActiveAlarms(UWORD uwChannelSel);
#endif
