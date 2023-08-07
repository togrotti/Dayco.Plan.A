/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : EnDatEncoder.h                                             */
/* Author      : Fabio Terrile, Stefano Martino                             */
/*                                                                          */
/* Description : Endat encoder manager                                      */
/*               combinations												*/
/*                                                                          */
/****************************************************************************/

#ifndef _ED_H
 #define _ED_H

//***************************************************************************
// Defines

#define ENDAT_SEL_MAIN              0

#ifndef _HW_DC
#define ENDAT_SEL_AUX               1
#define ENDAT_MAX_ENCODER           2

#else
#define ENDAT_MAX_ENCODER           1
#endif

//***************************************************************************
// Output data structures

typedef struct {
    UWORD uwCRCErrorCounter;
    UWORD uwPropagationDelay;     // [nsec]
    UWORD uwMaxFrequency;         // [kHz]
    UBYTE ubProtocolVersion;      // xx.x
    UBYTE ubStepPerRevBits;
    UBYTE ubRevNumBits;
    UBYTE ubAddInfoVar; // crs-endat.22
    UWORD uwAddInfoVal; // crs-endat.22
    UWORD uwAddInfoCRCErrorCounter ; // crs-endat.22
} ENDAT_OUT ;

//***************************************************************************
// Parameters data structure

typedef struct {
    union {
        struct {
            BOOL bDummy ;
            BOOL bDisEndAlarm ;
        } b ;
        // UWORD w ;
#ifdef _INFINEON
    }                             flags ;
#else
    } __attribute__((aligned(4))) flags ;
#endif
    
    UWORD uwClockFreq;            // [kHz]
    ULONG ulMTurnStartPos;
} ENDAT_PARAMS ;

//****************************************************************************
// General global variables

extern ENDAT_PARAMS sEn_Params[ENDAT_MAX_ENCODER];
extern const ENDAT_PARAMS sEn_DefParams ;
extern ENDAT_OUT sEn_DataOut[ENDAT_MAX_ENCODER];

//****************************************************************************
// Global functions

BOOL En_Init(UWORD, ENCMGR_SPACEFEEDBACK *, SBYTE *, ULONG); 
BOOL En_ParametersCheck(UWORD) ; 
void En_SetElecAngle(UWORD);

BOOL En_GetEPlate(UWORD,MOTPRM_PARAMETERS *,SBYTE *); 

//***************************************************************************
// PLC management functions

ULONG PlcEndatSetCommandMode(UWORD uwChannelSel, BOOL bSet);
ULONG PlcEndatReadParameter(UWORD uwChannelSel, UBYTE ubMemArea, UBYTE ubAddress);
ULONG PlcEndatWriteParameter(UWORD uwChannelSel, UBYTE ubMemArea, UBYTE ubAddress, UWORD uwParam);
ULONG PlcEndatResetEncoder(UWORD uwChannelSel) ;
ULONG PlcEndatResetActiveAlarms(UWORD uwChannelSel);
ULONG PlcEndatAddInfoSetup(UWORD uwChannelSel, UWORD uwAddInfoCode); // crs-endat.22
#endif
