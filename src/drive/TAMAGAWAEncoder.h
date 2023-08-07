/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005-2018, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : TAMAGAWAEncoder.h                                          */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : TAMAGAWA serial interface encoder                          */
/*                                                                          */
/****************************************************************************/

#ifndef _TAMAGAWAENCODER_H
#define _TAMAGAWAENCODER_H

//***************************************************************************
// Defines

#define TMGW_SEL_MAIN              0
#define TMGW_MAX_ENCODER           1

#define TMGW_FUNCTION_ERROR_TIMEOUT             0x80000000l
#define TMGW_FUNCTION_ERROR_CRC_ERROR           0x40000000l
#define TMGW_FUNCTION_ERROR_ACCESSDENIED        0x02000000l
#define TMGW_FUNCTION_ERROR_NOMODULE            0x01000000l
#define TMGW_FUNCTION_ERROR_MASK_ANY            0xff000000l

//***************************************************************************
// Output data structures

typedef struct
{
    UBYTE ubMultiTurn;
    UBYTE ubResolution;
    UBYTE ubBaudrate;
} TMGW_OUT ;

//***************************************************************************
// Parameters data structure

typedef struct
{
    union
    {
        struct
        {
            BOOL bDisAlarm  ;
            BOOL bDisMTData ; // disable multi-turn data
        } b ;
//        UWORD w ;
#ifdef _INFINEON_
    } flags ;
#else
} __attribute__((aligned(4))) flags ;
#endif
    ULONG ulMTurnStartPos;
} TMGW_PARAMS;

//****************************************************************************
// General global variables

extern TMGW_PARAMS sTm_Params[TMGW_MAX_ENCODER];
#ifdef _INFINEON_
extern const TMGW_PARAMS huge sTm_DefParams ;
#else
extern const TMGW_PARAMS sTm_DefParams ;
#endif
extern TMGW_OUT sTm_DataOut[TMGW_MAX_ENCODER];

//****************************************************************************
// Global functions

BOOL Tm_Init(UWORD,ENCMGR_SPACEFEEDBACK *,SBYTE *,ULONG);
BOOL Tm_ParametersCheck(UWORD uwChannelSel) ; 
void Tm_SetElecAngle(UWORD);

//***************************************************************************
// PLC management functions

ULONG PlcTMGWResetActiveAlarms(UWORD uwChannelSel);
ULONG PlcTMGWMTurnClear(UWORD uwChannelSel);
ULONG PlcTMGWSetZeroPosition(UWORD uwChannelSel);
ULONG PlcTMGWSetCommandMode(UWORD uwChannelSel, BOOL bSet);

#endif
