/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EndatHandlers.h                                            */
/* Author      : Stefano Martino                                            */
/*               Fabio Terrile                                              */
/*                                                                          */
/* Description : Low level ENDAT 2.1/2.2 management functions               */
/*               23/01/08: enhanced HW_ENDATEX handling                     */
/*                                                                          */
/****************************************************************************/

#ifndef _ENDATHANDLERS_H
#define _ENDATHANDLERS_H

#include "common\CommonDefines.h"

//***************************************************************************
// Defines

#define ENDAT_FUNCTION_ERROR_TIMEOUT             0x80000000l
#define ENDAT_FUNCTION_ERROR_CRC_ERROR           0x40000000l
#define ENDAT_FUNCTION_ERROR_ACCESSDENIED        0x02000000l
#define ENDAT_FUNCTION_ERROR_NOMODULE            0x01000000l
#define ENDAT_FUNCTION_ERROR_MASK_ANY            0xff000000l

// Endat additional info managed
// crs-endat.22
#define ENDAT_ADINFO1_NOP                               0x00
#define ENDAT_ADINFO1_DIAG                              0x01
#define ENDAT_ADINFO1_POS2_LW                           0x02
#define ENDAT_ADINFO1_POS2_MW                           0x03
#define ENDAT_ADINFO1_POS2_HW                           0x04
#define ENDAT_ADINFO1_MEM_LSB                           0x05
#define ENDAT_ADINFO1_MEM_MSB                           0x06
#define ENDAT_ADINFO1_MRS                               0x07
#define ENDAT_ADINFO1_TESTCMDACK                        0x08
#define ENDAT_ADINFO1_TESTV_LW                          0x09
#define ENDAT_ADINFO1_TESTV_MW                          0x0A
#define ENDAT_ADINFO1_TESTV_HW                          0x0B
#define ENDAT_ADINFO1_TEMPSENS1                         0x0C
#define ENDAT_ADINFO1_TEMPSENS2                         0x0D
#define ENDAT_ADINFO1_ADDSENS                           0x0E
#define ENDAT_ADINFO1_ERRHTYPEIII                       0x0F
// crs-endat.22
//***************************************************************************
// Data types

typedef struct
{
    UWORD uwClockDiv;
    UWORD uwPropComp;
    UWORD uwPosFormat;
    UWORD uwEarlyStop;
    UBYTE ubStepPerRevBits;
    UBYTE ubRevNumBits;
    UWORD uwPropagationTime;        // nsec
    ULONG dwBaseAddress;
    UWORD uwMaxFrequency;
    union
    {
        struct
        {
            UWORD bProt22:1;
            UWORD bAdInfoEnabled:1;
            UWORD bEndatEx:1; // crs-endat.22
        } b;
        UWORD w;
    } flags;
    UBYTE ubADInfo; // crs-endat.22
} ENDAT_WORKS;

//***************************************************************************
// Functions

BOOL  EndatEnterCommandMode(ENDAT_WORKS * psWorks);
BOOL  EndatEnterPositionMode(ENDAT_WORKS * psWorks);
ULONG EndatReadParameter(ENDAT_WORKS * psWorks, UBYTE ubMemArea, UBYTE ubAddress);
ULONG EndatWriteParameter(ENDAT_WORKS * psWorks, UBYTE ubMemArea, UBYTE ubAddress, UWORD uwParam);
BOOL  EndatSetRealtimeADInfo(ENDAT_WORKS * psWorks, UBYTE ubADInfo); // crs-endat.22
ULONG EndatResetActiveAlarms(ENDAT_WORKS * psWorks);
UWORD EndatInitializeEncoder(ULONG dwBaseAddress, UWORD uwClockFreq, ENDAT_WORKS * psWorks, BOOL bCalcStartDelay);
BOOL  EndatCheckParameters(UWORD uwClockFreq);
ULONG EndatResetEncoder(ENDAT_WORKS * psWorks);
#endif
