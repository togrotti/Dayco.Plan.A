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

#define MATRIX_DIMENSION    9  // Matrice 9x9, Vettore 9

//****************************************************************************
// Input/Output data structures

typedef struct {
    SWORD   *pswActualSpd ; // puntatore per prendere la velocita' (elettrica) su cui eseguire l'algoritmo del deflussaggio
    SWORD   *pswActualVdc ; // puntatore per prendere la tensione VdcBus
    MH_POWERSTAGESTATUS *psPowerStageStatus ;
} DFLX_IN ;

typedef struct {
    union {
        struct {
            BOOL bDefluxActive;
            BOOL bMaxSpeedReachable;
        } b ;
        UWORD w;
    } flags ;
    GLB_TORQUE_LIMIT sDefluxIqLimit ;
    SLONG slDflxKneeSpd ;   // velocita' minima a cui iniziare a deflussare (i.u.)
    SLONG slMotorIcc ;      // corrente di corto circuito [1/10mArms]
    SLONG slDflxSpdMax ;    // velocita' massima in uso (i.u.)
    FLOAT flDirectInductance ;
    FLOAT flIqLimit2Use[MATRIX_DIMENSION] ;  // vettore limiti di corrente 
} DFLX_OUT ;

typedef struct {
    SLONG   slDflxSpdMax ;  // velocita' massima (i.u.)
    UWORD   uwKVOut ;       // margine di sicurezza sulla tensione (0; 1000] = 0% -> 100%
    SWORD   swDflxVdc ;     // tensione Vdc (1e-1V) nel caso non si utilizzi quella letta dal drive
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