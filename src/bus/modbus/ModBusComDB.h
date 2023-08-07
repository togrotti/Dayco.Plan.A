/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ModBusComDB.h                                              */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : ModBus parameter DB interface for Common DB handling       */
/*                                                                          */
/****************************************************************************/

#ifndef _MODBUSCOMDB_H
#define _MODBUSCOMDB_H

#include "common\CommonParamDB.h"

//***************************************************************************
// Modbus Db structure

typedef struct
{
    UWORD                               uwIndex;            // parameter index
    const COMMONPARAMDB_ENTRY  *    hpsComDBEntry;      // reference to common DB
} MODBUSCOMDB_ENTRY;

//***************************************************************************
// Reference to physical parameter table

extern const MODBUSCOMDB_ENTRY  hpsModBusParamTable[];
extern const UWORD uwModBusParamCount;

extern const MODBUSCOMDB_ENTRY  hpsModBusCoilsTable[];
extern const UWORD uwModBusCoilsCount;

extern const MODBUSCOMDB_ENTRY  * hpsModBusDynParamTable;
extern UWORD uwModBusDynParamCount;

//***************************************************************************
// Global functions

    // Table checking, just for debug purpose
#ifdef _APP_DEBUG
BOOL ModBusComDBCheckTable(void);
#endif

    // Parameter Table search
const MODBUSCOMDB_ENTRY  * ModBusComDBEntrySearch(UWORD uwIndex);

    // Parameter Entry Read
UBYTE ModBusComDBEntryRead(UBYTE ubFCode, UWORD uwRegAddress, UWORD uwRegNumbers, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength);

    // Parameter Entry Write
UBYTE ModBusComDBEntryWrite(UBYTE ubFCode, UWORD uwRegAddress, UWORD uwRegNumbers, UWORD uwOrigRegAddress, HPUBYTE hpubInBuffer, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bRetRegValue);

    // Coil Entry Read
UBYTE ModBusComDBCoilRead(UBYTE ubFCode, UWORD uwRegAddress, UWORD uwRegNumbers, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength);

    // Coil Entry Write
UBYTE ModBusComDBCoilWrite(UBYTE ubFCode, UWORD uwRegAddress, UWORD uwRegNumbers, UWORD uwOrigRegAddress, HPUBYTE hpubInBuffer, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bRetRegValue);

#endif
