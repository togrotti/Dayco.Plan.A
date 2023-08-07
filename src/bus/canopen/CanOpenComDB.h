/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CanOpenComDB.h                                             */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : CanOpen parameter DB interface for Common DB handling      */
/*                                                                          */
/****************************************************************************/

#ifndef _CANOPENCOMDB_H
#define _CANOPENCOMDB_H

#include "common\CommonParamDB.h"
#include "CanOpenDs301Externals.h"

//***************************************************************************
// Object Dictionary Flags definitions

#define CANOPENCOMDB_F_DEFAULT          0x0000

#define CANOPENCOMDB_F_PDOMAPPABLE      0x0001      // PDO mappable object
#define CANOPENCOMDB_F_WRDENYWHENOPER   0x0002      // write denied when 
                                                    // NMT operational
#define CANOPENCOMDB_F_PARAM            0x0004      // parameter, stored in NV memory
#define CANOPENCOMDB_F_HIDDEN           0x0008      // hidden, not enumerable
#define CANOPENCOMDB_F_INFO_RECORD      0x0010      // return info as RECORD
#define CANOPENCOMDB_F_INFO_ARRAY       0x0020      // return info as ARRAY
#define CANOPENCOMDB_F_DS301_VALID      0x0040      // object valid for CANOpen DS301
#define CANOPENCOMDB_F_ECATCOE_VALID    0x0080      // object valid for EtherCAT COE

//***************************************************************************
// Object Dictionary Db structure

typedef struct
{
    UWORD                               uwIndex;            // parameter index
    UWORD                               uwSubIndex;         // and subindex
    UWORD                               uwFlags;            // specific OD flags
    const COMMONPARAMDB_ENTRY  *    hpsComDBEntry;      // reference to common DB
} CANOPENCOMDB_ENTRY;

//***************************************************************************
// Reference to physical parameter table

extern const CANOPENCOMDB_ENTRY  hpsCanOpenParamTable[];
extern const UWORD uwCanOpenParamCount;

extern const CANOPENCOMDB_ENTRY  * hpsCanOpenDynParamTable;
extern UWORD uwCanOpenDynParamCount;

//***************************************************************************
// Global functions

    // Table checking, just for debug purpose
#ifdef _APP_DEBUG
BOOL CanOpenComDBCheckTable(void);
#endif

    // Parameter Table search
ULONG CanOpenComDBEntrySearch(UWORD uwIndex, UWORD uwSubIndex, UWORD uwDBSel, const CANOPENCOMDB_ENTRY  * * ptEntry);

    // SDO Transaction callback
ULONG CanOpenComDBSdoDispatcher(DS301_SDOTRANSACTION * ptSdoStatus, UWORD uwDBSel);

#endif
