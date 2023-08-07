/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright ?2005-2017, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : NikonEncoder.h                                             */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Nikon serial interface encoder                             */
/*                                                                          */
/****************************************************************************/

#ifndef _NIKONENCODER_H
#define _NIKONENCODER_H

//***************************************************************************
// Defines

#define NIKON_SEL_MAIN              0
#define NIKON_MAX_ENCODER           1

#define NIKON_FUNCTION_ERROR_TIMEOUT             0x80000000l
#define NIKON_FUNCTION_ERROR_CRC_ERROR           0x40000000l
#define NIKON_FUNCTION_ERROR_ACCESSDENIED        0x02000000l
#define NIKON_FUNCTION_ERROR_NOMODULE            0x01000000l
#define NIKON_FUNCTION_ERROR_MASK_ANY            0xff000000l

//***************************************************************************
// Output data structures
typedef struct {
    UBYTE ubBaudrate;
    UBYTE ubBatteryLine;
    UBYTE ubStepPerRevBits;
    UBYTE ubRevNumBits;
} NIKON_OUT ;

//***************************************************************************
// Parameters data structure
typedef struct {
    union {
        struct {
            BOOL bDisAlarm  ; // disable encoder alarm
            BOOL bDisMTData ; // disable multi-turn data
        } b ;
//          UWORD w ;
#ifdef _INFINEON_
    } flags ;
#else
} __attribute__((aligned(4))) flags ;
#endif
    ULONG ulMTurnStartPos;
} NIKON_PARAMS;

//****************************************************************************
// General global variables

extern NIKON_PARAMS sNk_Params[NIKON_MAX_ENCODER];
#ifdef _INFINEON_
extern const NIKON_PARAMS huge sNk_DefParams ;
#else
extern const NIKON_PARAMS sNk_DefParams ;
#endif
extern NIKON_OUT sNk_DataOut[NIKON_MAX_ENCODER];

//****************************************************************************
// Global functions

BOOL Nk_Init(UWORD,ENCMGR_SPACEFEEDBACK *,SBYTE *,ULONG);
BOOL Nk_ParametersCheck(UWORD uwChannelSel) ; 
void Nk_SetElecAngle(UWORD);

//***************************************************************************
// PLC management functions

ULONG PlcNikonResetActiveAlarms(UWORD uwChannelSel);
ULONG PlcNikonMTurnClear(UWORD uwChannelSel);
ULONG PlcNikonSetZeroPosition(UWORD uwChannelSel);
ULONG PlcNikonSetCommandMode(UWORD uwChannelSel, BOOL bSet);

#endif
