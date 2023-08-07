/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CommonController.h                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Common controller handlers                                 */
/*                                                                          */
/****************************************************************************/

#ifndef _COMMONCONTROLLER_H
#define _COMMONCONTROLLER_H

//***************************************************************************
// Emergency stop options

#define COMCTRL_ES_FATAL                        1
#define COMCTRL_ES_NONFATAL                     2

//***************************************************************************
// Data types

typedef struct {
  void (* pfEmcyStop)(UWORD uwOption);
  void (* pfFaultClear)(void);
} COMCTRL_HANDLERS;

typedef struct
{
   struct
    {
        BOOL bQuickStop;
        BOOL bVelocity ;
        BOOL bPosition ;
        BOOL bNewTarget;     // TRUE = nuovo target; FALSE = nessun nuovo target
        BOOL bPostnerEnable; // TRUE = abilita il modulo posizionatore
        BOOL bApplicationEnabled;
        BOOL bPID_IntegralEnable;
        BOOL bInterpolate;
        BOOL bUseLocalAccSpd;
        BOOL bDirectSpeed;
    } b ;
/*    struct
    {
        UWORD bQuickStop : 1 ;
        UWORD bVelocity  : 1 ;
        UWORD bPosition  : 1 ;
        UWORD bNewTarget : 1 ;    // TRUE = nuovo target; FALSE = nessun nuovo target
        UWORD bPostnerEnable: 1 ; // TRUE = abilita il modulo posizionatore
        UWORD bApplicationEnabled : 1 ;
        UWORD bPID_IntegralEnable : 1 ;
        UWORD bInterpolate : 1 ;
        UWORD bUseLocalAccSpd : 1 ;
        UWORD bDirectSpeed : 1 ;
    } b ;
    UWORD w ;*/
} MOTCTRL_STATUS ;

#endif
