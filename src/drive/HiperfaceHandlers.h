/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2014, Phase Motion Control. All Rights Reserved.             */
/*                                                                          */
/* File        : HiperfaceHandlers.h                                        */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Low level HIPERFACE management functions                   */
/*                                                                          */
/****************************************************************************/

#ifndef _HIPERFACEHANDLERS_H
#define _HIPERFACEHANDLERS_H

#include "common\CommonDefines.h"

//***************************************************************************
// Defines

#define HIPFC_BAUDRATE_600                      600
#define HIPFC_BAUDRATE_1200                     1200
#define HIPFC_BAUDRATE_2400                     2400
#define HIPFC_BAUDRATE_4800                     4800
#define HIPFC_BAUDRATE_9600                     9600
#define HIPFC_BAUDRATE_19200                    19200
#define HIPFC_BAUDRATE_38400                    38400

#define HIPFC_PARITY_NONE                       0
#define HIPFC_PARITY_EVEN                       1
#define HIPFC_PARITY_ODD                        2

#define HIPFC_DEF_BAUDRATE                      (HIPFC_BAUDRATE_9600)
#define HIPFC_DEF_PARITY                        (HIPFC_PARITY_ODD)

#define HIPFC_SUCCESS                           0x0000

#define HIPFC_STAT_MASK                         0x0100

#define HIPFC_NOTAVAILABLE                      0x0201

#define HIPFC_INIT_UNSUPPORTED                  0x0401

#define HIPFC_CMD_RXUNEXPECTED                  0x0801
#define HIPFC_CMD_ERRORFLAG                     0x0802
#define HIPFC_CMD_RXBUFFEROVERFLOW              0x0803

#define HIPFC_TRAN_TIMEOUT                      0x1001
#define HIPFC_TRAN_PARITY_ERROR                 0x1002
#define HIPFC_TRAN_CHECKSUM_ERROR               0x1003

//***************************************************************************
// Data types

typedef struct
{
    ULONG ulBaseAddress;
    UWORD uwCharTime;                           // usec
    UWORD uwRevolutionNumber;
    UWORD uwRGSet;
    SBYTE sbPosShift;
} HIPFC_WORKS;

typedef struct
{
    UBYTE ubRS485Settings;
    UBYTE ubEncoderType;
    UBYTE ubEEPROMSize;
    UBYTE ubOptions;
} HIPFC_TYPELABEL;

typedef struct
{
    UBYTE ubType;
    ULONG ulResolution;
    ULONG ulRange;
} HIPFC_EXTTYPELABEL;

//***************************************************************************
// Functions

#ifdef _INFINEON_
UWORD HipfcInitialization(ULONG ulBaseAddress, UWORD uwBaudRate, UBYTE ubParity, HIPFC_WORKS huge * psWorks);
UWORD HipfcReadTypeLabel(HIPFC_WORKS huge * psWorks, HIPFC_TYPELABEL * psTypeLabel);
UWORD HipfcReadPosition(HIPFC_WORKS huge * psWorks, UBYTE * pubDest);
UWORD HipfcSetPosition(HIPFC_WORKS huge * psWorks, UBYTE * pubSrc);
UWORD HipfcReadExtTypeLabel(HIPFC_WORKS huge * psWorks, HIPFC_EXTTYPELABEL * psExtTypeLabel);
#else
UWORD HipfcInitialization(ULONG ulBaseAddress, UWORD uwBaudRate, UBYTE ubParity, HIPFC_WORKS * psWorks);
UWORD HipfcReadTypeLabel(HIPFC_WORKS * psWorks, HIPFC_TYPELABEL * psTypeLabel);
UWORD HipfcReadPosition(HIPFC_WORKS * psWorks, UBYTE * pubDest);
UWORD HipfcSetPosition(HIPFC_WORKS * psWorks, UBYTE * pubSrc);
UWORD HipfcReadExtTypeLabel(HIPFC_WORKS * psWorks, HIPFC_EXTTYPELABEL * psExtTypeLabel);
#endif

#endif
