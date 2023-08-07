/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2015, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : PlcRetainMgr.h                                             */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : PLC Retain data manager                                    */
/*                                                                          */
/****************************************************************************/

#ifndef _PLCRETAINMGR_H
#define _PLCRETAINMGR_H

//***************************************************************************
// Defines

#define PLCRETAINMGR_SIZE                           16

//***************************************************************************
// Data structures

typedef struct
{
    UBYTE bData[PLCRETAINMGR_SIZE];
} PLCRETAINMGR_DATA;

//***************************************************************************
// Globals

extern PLCRETAINMGR_DATA psPlcRetMgrData;

//***************************************************************************
// Globals functions

    // restore data collection that come with clock log
BOOL PlcRetMgr_RestoreRetainData(HPVOID hpvBuf, SWORD swSize);

    // add data saved with global clock
SWORD PlcRetMgr_PostClockData(HPUBYTE hpubBuf, SWORD swLeftSize);

#endif

