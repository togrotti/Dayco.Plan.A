/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright ?2017, Phase Motion Control. All Rights Reserved.              */
/*                                                                          */
/* File        : NikonHandlers.h                                            */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Low level Nikon series encoder interface management funcs  */
/*                                                                          */
/****************************************************************************/

#ifndef _NIKONHANDLERS_H
#define _NIKONHANDLERS_H

//***************************************************************************
// Data types

typedef struct
{
    ULONG ulBaseAddress;
    UBYTE ubEncAddr;
    UBYTE ubCmdMode;
    UBYTE ubResolution;
    UBYTE ubMultiTurn;
    UBYTE ubBatteryLine;
    UBYTE ubBaudrate;
    UBYTE ubRevNumBits;
    UBYTE ubStepPerRevBits;
    UBYTE ubDisMTData;
} NIKON_WORKS;

//***************************************************************************
// Defines

#define NIKON_BLOCK_CMDCODE    0x08 // block selection: Command code/Encoder address
#define NIKON_BLOCK_ROMWDATA   0x09 // block selection: EEPROM write data
#define NIKON_BLOCK_ROMADDR    0x0A // block selection: EEPROM address
#define NIKON_BLOCK_TRIGSET    0x20 // block selection: trigger input setting register
#define NIKON_BLOCK_TRIG       0x21 // block selection: transmission trigger register
#define NIKON_BLOCK_BAUDRATE   0x28 // block selection: baudrate setting register 

#define NIKON_CMDCODE_MASK     0x00F8
#define NIKON_CMDCODE_CDF0     (0x0<<3)  // command code setting: CDF0  absolute full data request
#define NIKON_CMDCODE_CDF1     (0x1<<3)  // command code setting: CDF1  absolute lower 24-bit data request
#define NIKON_CMDCODE_CDF2     (0x2<<3)  // command code setting: CDF2  absolute upper 24-bit data request
#define NIKON_CMDCODE_CDF3     (0x3<<3)  // command code setting: CDF3  encoder status request
                                         // command code setting: CDF4 ~ CDF6 not supported
#define NIKON_CMDCODE_CDF8     (0x8<<3)  // command code setting: CDF8  status flag clear request
#define NIKON_CMDCODE_CDF9     (0x9<<3)  // command code setting: CDF9  multiple rotation data clear request
#define NIKON_CMDCODE_CDF10    (0xA<<3)  // command code setting: CDF10 status+multiple rotation data clear request
#define NIKON_CMDCODE_CDF11    (0xB<<3)  // command code setting: CDF11 encoder adderss change request
#define NIKON_CMDCODE_CDF12    (0xC<<3)  // command code setting: CDF12 one rotation data zero preset
#define NIKON_CMDCODE_CDF13    (0xD<<3)  // command code setting: CDF13 memory read request
#define NIKON_CMDCODE_CDF14    (0xE<<3)  // command code setting: CDF14 memory write request
#define NIKON_CMDCODE_CDF15    (0xF<<3)  // command code setting: CDF15 temperature data
#define NIKON_CMDCODE_CDF27    (0x1B<<3) // command code setting: CDF27 absolute lower 24-bit data + status request
#define NIKON_CMDCODE_CDF29    (0x1D<<3) // command code setting: CDF29 absolute lower 24-bit data + temperature request

#define NIKON_RDADDR_ABS_LL    0x0D // read address: Absolute data with alarm flag at the MSB of output register
#define NIKON_RDADDR_ABS_LH    0x0E 
#define NIKON_RDADDR_ABS_HL    0x0F
#define NIKON_RDADDR_STATUS    0x08 // read address: Status
#define NIKON_RDADDR_ALARM     0x09 // read address: Alarm
#define NIKON_RDADDR_MEMDATA   0x09 // read address: memory data

#define NIKON_ENCADDR_1        0x0
#define NIKON_ENCADDR_2        0x1
#define NIKON_ENCADDR_3        0x2
#define NIKON_ENCADDR_4        0x3
#define NIKON_ENCADDR_5        0x4
#define NIKON_ENCADDR_6        0x5
#define NIKON_ENCADDR_7        0x6
#define NIKON_ENCADDR_8        0x7

#define NIKON_TRIGSET_MICRO    0x0  // trigger signal is controlled by the micro
#define NIKON_TRIGSET_EXT      0x2  // trigger signal is controlled by trigger bit in SET register

#define NIKON_TRIG_EN          0x2

#define NIKON_BAUDRATE_25      0    // baudrate 2.5Mbps
#define NIKON_BAUDRATE_40      1    // baudrate 4.0Mbps
#define NIKON_DEF_BAUDRATE     0

#define NIKON_MEMADDR_TYPE     0x00F6
#define NIKON_ENCTYPE_RES_OFF  0
#define NIKON_ENCTYPE_MT_OFF   3
#define NIKON_ENCTYPE_BAT_OFF  5

#define NIKON_ENCTYPE_RES_MASK (0x0007<<NIKON_ENCTYPE_RES_OFF)
#define NIKON_ENCTYPE_RES_17   (0x0000<<NIKON_ENCTYPE_RES_OFF)
#define NIKON_ENCTYPE_RES_20   (0x0001<<NIKON_ENCTYPE_RES_OFF)
#define NIKON_ENCTYPE_RES_22   (0x0002<<NIKON_ENCTYPE_RES_OFF)
#define NIKON_ENCTYPE_RES_24   (0x0003<<NIKON_ENCTYPE_RES_OFF)

#define NIKON_ENCTYPE_MT_MASK  (0x0003<<NIKON_ENCTYPE_MT_OFF)
#define NIKON_ENCTYPE_MT_SG    (0x0000<<NIKON_ENCTYPE_MT_OFF)   // Single-turn ABS
#define NIKON_ENCTYPE_MT_MAG   (0x0001<<NIKON_ENCTYPE_MT_OFF)   // Multi-turn ABS(Magnetic type)
#define NIKON_ENCTYPE_MT_OPT   (0x0002<<NIKON_ENCTYPE_MT_OFF)   // Multi-turn ABS(Optical type)

#define NIKON_ENCTYPE_MT_NBITS  16

#define NIKON_ENCTYPE_BAT_MASK (0x0003<<NIKON_ENCTYPE_BAT_OFF)
#define NIKON_ENCTYPE_BAT_NONE (0x0000<<NIKON_ENCTYPE_BAT_OFF)  // None
#define NIKON_ENCTYPE_BAT_SEP  (0x0001<<NIKON_ENCTYPE_BAT_OFF)  // Separate type
#define NIKON_ENCTYPE_BAT_COM  (0x0002<<NIKON_ENCTYPE_BAT_OFF)  // Common type

#define NIKON_INIT_SUCCESS          0x0000
#define NIKON_INIT_TIMEOUT          0x0001
#define NIKON_INIT_ALARM            0x0002
#define NIKON_INIT_MEMRDFAIL        0x0003
#define NIKON_INIT_TYPEUNKNOW       0x0004
#define NIKON_INIT_TRGDLYFAIL       0x0005

#define NIKON_STATUS_ES0_MASK   0x0100  // Encoder Status: BUSY + MEMBUSY
#define NIKON_STATUS_ES1_MASK   0x0200  // Encoder Status: BATT
#define NIKON_STATUS_ES2_MASK   0x0400  // Encoder Status: OVSPD + MEMERR + OVTEMP
#define NIKON_STATUS_ES3_MASK   0x0800  // Encoder Status: STERR + PSERR + MTERR + INCERR

#define NIKON_ALARM_MTERR_MASK  0x0002
#define NIKON_ALARM_BATT_MASK   0x0001

//***************************************************************************
// Functions

#ifdef _INFINEON_
UWORD NikonInitialization(UBYTE ubEncAddr,ULONG ulBaseAddress,NIKON_WORKS huge * psWorks,BOOL bCalcStartDelay);
void  NikonEnterCommandMode(NIKON_WORKS huge * psWorks);
void  NikonEnterPositionMode(NIKON_WORKS huge * psWorks);
BOOL  NikonHandlers_StatusFlagClear(NIKON_WORKS huge * psWorks);
BOOL  NikonHandlers_SMClear(NIKON_WORKS huge * psWorks);
BOOL  NikonHandlers_MTClear(NIKON_WORKS huge * psWorks);
BOOL  NikonHandlers_ZeroPosSet(NIKON_WORKS * psWorks);
#else
UWORD NikonInitialization(UBYTE ubEncAddr,ULONG ulBaseAddress,NIKON_WORKS * psWorks,BOOL bCalcStartDelay);
void  NikonEnterCommandMode(NIKON_WORKS * psWorks);
void  NikonEnterPositionMode(NIKON_WORKS * psWorks);
BOOL  NikonHandlers_StatusFlagClear(NIKON_WORKS * psWorks);
BOOL  NikonHandlers_SMClear(NIKON_WORKS * psWorks);
BOOL  NikonHandlers_MTClear(NIKON_WORKS * psWorks);
BOOL  NikonHandlers_ZeroPosSet(NIKON_WORKS * psWorks);
#endif

#endif
