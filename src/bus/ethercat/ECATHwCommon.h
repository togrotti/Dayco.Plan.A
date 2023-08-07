/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ECATHwCommon.h                                             */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : ECAT specific command manager                              */
/*                                                                          */
/****************************************************************************/

#ifndef _ECATHWCOMMON_H
#define _ECATHWCOMMON_H

//***************************************************************************
// Data structures

typedef struct
{
    unsigned char   ubInvFrameCnt;
    unsigned char   ubRxErrCnt;
    unsigned char   ubForwardedRxErrCnt;
    unsigned char   ubLostLinkCnt;
} ECATCM_DG_PORT;

typedef struct
{
    ECATCM_DG_PORT  sPort[2];
    unsigned char   ubProcUnitErrCnt;
    unsigned char   ubPDIErrCnt;
    union
    {
        struct
        {
            unsigned char bLink0;
            unsigned char bLink1;
            unsigned char bCommunication0;
            unsigned char bCommunication1;
        } b;
        unsigned char ubFlags[4];
    } sFlags;
    unsigned char   ubALStatus;
    unsigned short  uwALStatusCode;
    unsigned short  uwSyncOutputMissingCnt;
} ECATCM_DIAG;

#endif