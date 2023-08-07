/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CanOpenComDBInfo.h                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : CanOpen parameter DB interface for Common DB handling      */
/*                                                                          */
/****************************************************************************/

#ifndef _CANOPENCOMDBINFO_H
#define _CANOPENCOMDBINFO_H

#include "bus\ethercat\sdoinfoserv.h"
#include "common\CommonDefines.h"
#include "CanOpenDs301Externals.h"

//***************************************************************************
// Object Dictionary Info structures

typedef struct
{
    UWORD                               uwIndex;            // parameter index
    UWORD                               uwSubIndex;         // and subindex
    const VISIBLE_STRING  *         sDescr;
} CANOPENCOMDB_INFODESCR; 

//***************************************************************************
// Reference to physical parameter table

extern const CANOPENCOMDB_INFODESCR  hpsCanOpenParamInfo[];
extern const UWORD uwCanOpenParamInfoCount;

//***************************************************************************
// Global functions

    // Enumeration count of db elements
UWORD CanOpenComDBEnum(UWORD uwListType, UWORD uwDBSel);

    // Elements listing in a buffer
UWORD CanOpenComDBList(UWORD uwListType, UWORD uwDBSel, UWORD * pCntst, void * pData, UWORD wSize);

    // Get detailed information about the object
UWORD CanOpenComDBGetInfo(UWORD uwIndex, UWORD uwSubIndex, UWORD uwDBSel, TSDOINFOOBJDESC * ptInfoObj, TSDOINFOENTRYDESC * ptInfoEntry);

    // Get textual description of the object
UWORD CanOpenComDBGetDescription(UWORD uwIndex, UWORD uwSubIndex, VISIBLE_STRING * pData, UWORD uwSize);

#endif
