/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CommonEncoderManager.h                                     */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Common data for all kinds of encoders                      */
/*                                                                          */
/****************************************************************************/

#ifndef _COMMONENCODERMANAGER_H
#define _COMMONENCODERMANAGER_H

#include "common\GlobalStructures.h"

//***************************************************************************
// Encoder output status

#define ENCMGR_NULL                             0x00

#define ENCMGR_RELATIVE_VALID                   0x01
#define ENCMGR_ELE_ANGLE_VALID                  0x02
#define ENCMGR_FATAL_FAULT                      0x04
#define ENCMGR_NONFATAL_FAULT                   0x08
#define ENCMGR_ABSMECHTURN_VALID                0x10
#define ENCMGR_ABSMECHTURN_WAITING              0x20
#define ENCMGR_EFS_READY                        0x40
#define ENCMGR_SNAPSHOT_VALID                   0x80

#define ENCMGR_ELECANGLE_DEG2IU                 (FLOAT)(65536.0 / 360.0)
//****************************************************************************
// Input/Output data structures

typedef struct
{  
        // Position(64), Speed(32), Acceleration(32)
    GLB_KINEMATIC_DATA sEncData ;
        // electrical position
    UWORD  uwElecAngle;
        // offset for absolute position: abspos = relpos + offset
    SQWRD  sqMechAbsPosOffset;
        // output status
    UBYTE  ubStatus;
        // electrical angle correction
    UWORD  uwDeltaElecAngle;
	    // electrical speed
    SWORD  swElecSpeed;
} ENCMGR_SPACEFEEDBACK;

typedef struct
{  
        // Filtered Speed (32)
    SLONG  slFilteredSpeed;
} ENCMGR_SPACEFB_EXT;
#endif
