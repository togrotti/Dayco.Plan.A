/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright ?2018, Phase Motion Control. All Rights Reserved.              */
/*                                                                          */
/* File        : TAMAGAWAHandlers.h                                            */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Low level TMGW series encoder interface management funcs  */
/*                                                                          */
/****************************************************************************/

#ifndef _TMGWHANDLERS_H
#define _TMGWHANDLERS_H

//***************************************************************************
// Data types

typedef struct
{
    ULONG ulBaseAddress;
    UBYTE ubEncAddr;
    UBYTE ubCmdMode;
    UBYTE ubResolution;
    UBYTE ubMultiTurn;
    UBYTE ubBaudrate;
    UBYTE ubDisMTData;
} TMGW_WORKS;

//***************************************************************************
// Defines

#define TMGW_RDADDR_ABS_LL    0x00 // read address: Absolute data
#define TMGW_RDADDR_ABS_LH    0x01 
#define TMGW_RDADDR_ABS_HL    0x02
#define TMGW_RDADDR_ABS_HH    0x04
#define TMGW_RDADDR_ENID      0x01
#define TMGW_RDADDR_ENCERR    0x05 // read address: Encoder Error
#define TMGW_RDADDR_ALARM     0x06 // read address: Communication Alarm

#define TMGW_RDADDR_MASK      0x03

#define TMGW_ENCADDR_0        0
#define TMGW_ENCADDR_1        1

#define TMGW_BAUDRATE_25      0    // baudrate 2.5Mbps
#define TMGW_BAUDRATE_50      1    // baudrate 5.0Mbps
#define TMGW_DEF_BAUDRATE     0

#define TMGW_ENID_0B          0x0B
#define TMGW_ENID_11          0x11
#define TMGW_ENID_17          0x17

#define TMGW_ENID_NBITS       8

#define TMGW_ENCERR_BA_MASK   (1<<7)   // battery alarm
#define TMGW_ENCERR_BE_MASK   (1<<6)   // battery error
#define TMGW_ENCERR_ME_MASK   (1<<5)   // multi-turn error
#define TMGW_ENCERR_OH_MASK   (1<<4)   // over-heat
#define TMGW_ENCERR_CO_MASK   (1<<3)   // counter overflow
#define TMGW_ENCERR_CE_MASK   (1<<2)   // counting error
#define TMGW_ENCERR_FS_MASK   (1<<1)   // full absolute status
#define TMGW_ENCERR_OS_MASK   (1<<0)   // overspeed

#define TMGW_ENCERR_LATCH_MASK 0x7D   

#define TMGW_INIT_SUCCESS     0x00000000l
#define TMGW_INIT_TIMEOUT     0x00000100l
#define TMGW_INIT_ALARM       0x00000200l
#define TMGW_INIT_TYPEUNKNOW  0x00000400l
#define TMGW_INIT_TRGDLYFAIL  0x00000800l

//***************************************************************************
// Functions

#ifdef _INFINEON_
ULONG TMGWInitialization(ULONG ulBaseAddress,TMGW_WORKS huge * psWorks,BOOL bCalcStartDelay);
BOOL  TMGWReadPosition(TMGW_WORKS huge * psWorks, HPSLONG hpslPosLo, HPSLONG hpslPosHi, BOOL bRdHi);
void  TMGWEnterCommandMode(TMGW_WORKS huge * psWorks);
void  TMGWEnterPositionMode(TMGW_WORKS huge * psWorks);
BOOL  TMGWHandlers_StatusFlagClear(TMGW_WORKS huge * psWorks);
BOOL  TMGWHandlers_EMClear(TMGW_WORKS huge * psWorks);
BOOL  TMGWHandlers_ZeroPosSet(TMGW_WORKS * psWorks);
BOOL  TMGWReadError(TMGW_WORKS huge * psWorks, HPUWORD hpuwEncError);
#else
ULONG TMGWInitialization(ULONG ulBaseAddress,TMGW_WORKS * psWorks,BOOL bCalcStartDelay);
BOOL  TMGWReadPosition(TMGW_WORKS * psWorks, HPSLONG hpslPosLo, HPSLONG hpslPosHi, BOOL bRdHi);
void  TMGWEnterCommandMode(TMGW_WORKS * psWorks);
void  TMGWEnterPositionMode(TMGW_WORKS * psWorks);
BOOL  TMGWHandlers_StatusFlagClear(TMGW_WORKS * psWorks);
BOOL  TMGWHandlers_EMClear(TMGW_WORKS * psWorks);
BOOL  TMGWHandlers_ZeroPosSet(TMGW_WORKS * psWorks);
BOOL  TMGWReadError(TMGW_WORKS * psWorks, HPUWORD hpuwEncError);
#endif

#endif
