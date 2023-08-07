/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : SinCosEncoder.h                                            */
/* Author      : Cristiano Tognetti                                         */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _SC_H
 #define _SC_H

#include "common\CommonEncoderManager.h"

/* =============================== structure =============================== */  

typedef struct {
  SWORD swSinChannel  ; /* SIN channel as read from ADC */
  SWORD swCosChannel  ; /* COS channel as read from ADC */
  ULONG ulSinCosLevel ;  
} SINCOS_OUT ;

typedef struct
{ /* Flags */
    union {
        struct {
            BOOL bReverseSignals        ; // 0 => Angle = Atan(Sin/Cos); 1 => Angle = 0xffff -  Atan(Sin/Cos)  (Resolver/SinCos)
            BOOL bGain2Use              ; // 0 => HIGH (SinCos); 1 => LOW (Resolver/SinCos)
            BOOL bSin2Cos2ErrorDisabled ; // 0 => sin^2 + cos^2 error check enabled; 1 => sin^2 + cos^2 error check disabled
            BOOL bEnableAutoCalibration ; // enable auto calibration of sample point, offset and amplitude for resolver
        } b ;
        // UWORD w[2] ;
#ifdef _INFINEON
    }                             flags ;
#else
    } __attribute__((aligned(4))) flags ;
#endif

        // Encoder Poles Number
    UWORD uwEncPoleNumber ;
        // do not use
    ULONG ulDummy ;
        // Resolver/Sincos Alarm
    UWORD uwSinCosAlarmThreshold ;
        // analog inputs level gain (1000 = gain 1.0, 0 = no adjust)
    SWORD swGainSin;
    SWORD swGainCos;
        // analog inputs level offset (10000 = pos full scale, 0 = no adjust, -10000 = neg full scale)
    SWORD swOffsetSin;
    SWORD swOffsetCos;
        // offset for resolver frequency generator [usec*10]
    UWORD uwResFreqOffset;

} SINCOS_PARAMS ;

/* =========================== global  variables =========================== */
extern SINCOS_PARAMS sSc_SinCosParam ;
extern const SINCOS_PARAMS sSc_SinCosDefParam ;
extern SINCOS_OUT    sSc_DataOut ;

/* =============================== functions =============================== */
BOOL Sc_Init(ENCMGR_SPACEFEEDBACK * psFeedback, SBYTE *, ULONG) ;
BOOL Sc_ParametersCheck(void) ; 
void Sc_SetElecAngle(UWORD);
void Sc_Run(void);

#endif // _SC_H
