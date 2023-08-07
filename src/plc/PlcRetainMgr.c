/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2015, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : PlcRetainMgr.c                                             */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : PLC Retain data manager                                    */
/*                                                                          */
/****************************************************************************/
#pragma GCC optimize (2)

#include "common\CommonUtility.h"
#include "PlcRetainMgr.h"
#include "common\BlockStorage.h"
#include "system\SysLogManagement.h"
#include "system\SysAppDataCodes.h"
#include "assert.h"

//***************************************************************************
// Globals

PLCRETAINMGR_DATA psPlcRetMgrData;

//***************************************************************************
// restore data collection that come with clock log

BOOL PlcRetMgr_RestoreRetainData(HPVOID hpvBuf, SWORD swSize)
{
    HPVOID blkptr;
    ULONG storagesize;
    SWORD blkcode;

        // then seek managed blocks
    storagesize=(ULONG)swSize;
	while((blkcode=blkstor_enumvalid(&hpvBuf, &storagesize, &blkptr)) != BLKSTOR_ERR_NOTFOUND)
        switch(blkcode)
        {
            case DATACODE_SYSLOG_PLC_RETAIN:
    		    blkstor_getdata(blkptr,0,DATACODE_SYSLOG_PLC_RETAIN,&psPlcRetMgrData,sizeof(psPlcRetMgrData));
                break;
        }

    return TRUE;
}

//***************************************************************************
// add data saved with global clock

SWORD PlcRetMgr_PostClockData(HPUBYTE hpubBuf, SWORD swLeftSize)
{
    SYSLOGMGM_DATATMPIDENT * psIdent;
    PLCRETAINMGR_DATA * psRetainData;

        // allocate space for alarm data log
    psIdent=(SYSLOGMGM_DATATMPIDENT *)hpubBuf;
    psRetainData=(PLCRETAINMGR_DATA *)&hpubBuf[sizeof(BLKSTOR_HEADER)];

        // then fill up data
    psIdent->swCode=DATACODE_SYSLOG_PLC_RETAIN;
    psIdent->uwSize=sizeof(PLCRETAINMGR_DATA);

        // get data snapshot atomically
    assert(sizeof(PLCRETAINMGR_DATA)==ATOMIC_CHUCK16_SIZE);
    atomic_read_chunk16(psRetainData, &psPlcRetMgrData);

    return swLeftSize-sizeof(BLKSTOR_HEADER)-sizeof(PLCRETAINMGR_DATA);
}
