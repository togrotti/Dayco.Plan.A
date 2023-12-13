/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2010, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : DefluxRT.h                                                 */
/* Author      : Fabio Terrile; Cristiano Tognetti; Marco Calvini           */
/*                                                                          */
/* Description : Field weakening task time-optimized real time module       */
/*               (extension of MotorHandler)                                */
/*                                                                          */
/****************************************************************************/

#ifndef _DEFLUXRT_H
#define _DEFLUXRT_H

#include "common\CommonDefines.h"
#include "common\CommonMotorHandler.h"
#include "drive\Deflux.h"

//****************************************************************************
// Defines

#define MATRIX_DIMENSION_1  (MATRIX_DIMENSION - 1)
#define MATRIX_SHIFT        12
#define MATRIX_MASK         4096 // = 2^12
#define MATRIX_MASK_1       (MATRIX_MASK - 1)
#define DEFLUX_SQRT_24      4.8989794855663561963945681494118

//***************************************************************************
// Locals

typedef struct
{
    SWORD   swIdMatrix[MATRIX_DIMENSION][MATRIX_DIMENSION] ;
    SWORD   swIqRefLimMaxVector[MATRIX_DIMENSION] ;
} DFLX_MDATA ;

typedef struct
{ 
#ifdef _INFINEON_
    DFLX_MDATA huge * hpsMatrix ;
#else
    DFLX_MDATA * hpsMatrix ;
#endif // _infineon_    
    
    SWORD   swSpdMin ;
    SWORD   swSpdMax ;
    SWORD   swSpdDelta ;
    SWORD   swIqRefMax ;
    SWORD   swIqRefShift ;
} DFLX_DATA ;

#ifdef _INFINEON_
extern DFLX_MDATA huge sDlfxMData[2];
extern DFLX_DATA sdata sDlfxData[2];
#else
extern DFLX_MDATA sDlfxMData[2];
extern DFLX_DATA sDlfxData[2];
#endif // _infineon_

extern DFLX_DATA * psDflxData;

#if CFG_DFLX_VMOTOR
typedef struct
{
	// parameters needed by PI
	UWORD uwPiEnableSpeed ;
	FLOAT flMargin ;
	FLOAT flPiManualRefVal ;
	FLOAT flPiLimitMin ;
	FLOAT flPiLimitMax ;
	FLOAT flPiKp ;
	FLOAT flPiKi ;
} DFLX_VMOTOR ;

extern DFLX_VMOTOR  sDflxVMotor8kHz ;
extern DFLX_VMOTOR *psDflxVMotor ;
#endif // crs_dflx_vmotor
#endif