/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2010, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : Deflux.h                                                   */
/* Author      : Fabio Terrile; Cristiano Tognetti; Marco Calvini           */
/*                                                                          */
/* Description : Field weakening task                                       */
/*               (extension of MotorHandler)                                */
/*                                                                          */
/****************************************************************************/

#ifndef _DEFLUX_H
#define _DEFLUX_H

#include "common\CommonDefines.h"
#include "common\CommonMotorHandler.h"

#define MATRIX_DIMENSION      9  // Matrice 9x9, Vettore 9
#define ARMS2IU_FLOAT         10000.0
#define ARMS2IU_FLOAT_SQUARED (ARMS2IU_FLOAT * ARMS2IU_FLOAT)

//****************************************************************************
// Input/Output data structures

typedef struct {
    SWORD   *pswActualSpd ; // puntatore per prendere la velocita' (elettrica) su cui eseguire l'algoritmo del deflussaggio
    SWORD   *pswActualVdc ; // puntatore per prendere la tensione VdcBus
    MH_POWERSTAGESTATUS *psPowerStageStatus ;
#if CFG_DFLX_VMOTOR
    SWORD   *pswActualVMotor ;
#endif
} DFLX_IN ;

typedef struct {
    union {
        struct {
            BOOL bDefluxActive;
            BOOL bMaxSpeedReachable;
            BOOL bDefluxOnly_Matrix ;
            BOOL bDefluxActive_Matrix ;
#if CFG_DFLX_VMOTOR
            BOOL bDefluxOnly_VmotorPi ;
            BOOL bDefluxActive_VmotorPi ;
            BOOL bVmotorPiLimitActive ;
#endif
        } b ;
        UWORD w;
    } flags ;

    GLB_TORQUE_LIMIT sDefluxIqLimit ;
    SLONG slDflxKneeSpd ;   // velocita' minima a cui iniziare a deflussare (i.u.)
    SLONG slMotorIcc ;      // corrente di corto circuito [1/10mArms]
    SLONG slDflxSpdMax ;    // velocita' massima in uso (i.u.)
    FLOAT flDirectInductance ;
    FLOAT flIqLimit2Use[MATRIX_DIMENSION] ;  // vettore limiti di corrente 

    SLONG slDflxId ;
	SLONG slDflxId_Matrix ;
    GLB_TORQUE_LIMIT sMatrixIqLimit ;

#if CFG_DFLX_VMOTOR
    FLOAT flVdcBus ; // filtered
    FLOAT flVMotorMax ; // VdcBus/sqrt(2)
    FLOAT flPiRef ;
    FLOAT flPiFbk ; // Vmotor filtered
    FLOAT flPiErr ;
    FLOAT flPiIntegral ;
    FLOAT flPiProportional ;
    FLOAT flPiOut ; //
    GLB_TORQUE_LIMIT sPiIqLimit ;
	SLONG slDflxId_VmotorPi ;
#endif
} DFLX_OUT ;

typedef struct {
    SLONG   slDflxSpdMax ;  // velocita' massima (i.u.)
    UWORD   uwKVOut ;       // margine di sicurezza sulla tensione (0; 1000] = 0% -> 100%
    SWORD   swDflxVdc ;     // tensione Vdc (1e-1V) nel caso non si utilizzi quella letta dal drive
#if CFG_DFLX_VMOTOR
	FLOAT	flPiManualRefVal ; // V
	FLOAT	flMargin ;         // [0.0, 1.0]
	FLOAT	flPiKi ;
	FLOAT	flPiKp ;
	FLOAT	flPiLimitMax ;     // [0.0, +max]
	FLOAT	flPiLimitMin ;     // [-max, 0.0]
	FLOAT	flPiEnableSpeed ;  // rad/s
    UBYTE   bDefluxOnly_Matrix ;
	UBYTE   bDefluxOnly_VmotorPi ;
    UBYTE   bDummy_0 ; // arm 32bit alignment
    UBYTE   bDummy_1 ; // arm 32bit alignment
    UBYTE   bDummy_2 ; // arm 32bit alignment
    UBYTE   bDummy_3 ; // arm 32bit alignment
    UBYTE   bDummy_4 ; // arm 32bit alignment
    UBYTE   bDummy_5 ; // arm 32bit alignment
#endif
} DFLX_PARAM ;

extern DFLX_IN      sDflx_In ;
extern DFLX_OUT     sDflx_Out ;
extern DFLX_PARAM   sDflx_Param ;

#ifdef _INFINEON_
extern const DFLX_PARAM huge sDflx_DefParam ;
#else
extern const DFLX_PARAM sDflx_DefParam ;
#endif // _infineon_
//****************************************************************************
// Global functions

void DflxInit(SLONG slDrivePeakCurrent) ;
void Dflx8KHz(BOOL bReferenceEnabled, SLONG * pslIdRef, SLONG * pslIqRef) ;
void DlfxSlow(BOOL bDflxEnabled) ;

#endif