/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2008, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : HallEncoder.h                                              */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _HL_ENCODER_H
 #define _HL_ENCODER_H

/* =============================== structure =============================== */  
typedef struct {
  SWORD swSinChannel  ; /* SIN channel as read from ADC */
  SWORD swCosChannel  ; /* COS channel as read from ADC */
} HALL_OUT ;

typedef struct {
 /* Flags */
  union {
      struct {
        BOOL b4WSelection ;
      } b ;
      // UWORD w ;
#ifdef _INFINEON
  }                             flags ;
#else
  } __attribute__((aligned(4))) flags ;
#endif
  
      // Encoder Poles Number
  UWORD uwEncPoleNumber ;

} HALL_PARAMS ;


/* =========================== global  variables =========================== */
extern HALL_PARAMS  sHl_HallParam ;
#ifdef _INFINEON_
extern const HALL_PARAMS huge sHl_HallDefParam ;
#else
extern const HALL_PARAMS sHl_HallDefParam ;
#endif // _infineon_
extern HALL_OUT     sHl_DataOut ;
 
/* =============================== functions =============================== */
BOOL Hl_Init(ENCMGR_SPACEFEEDBACK * psFeedback, SBYTE *, ULONG) ;
BOOL Hl_ParametersCheck(void) ; 
void Hl_SetElecAngle(UWORD);

#endif // _HL_ENCODER_H
