/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2010, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : EncoderEFSeek.h                                            */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Electrical Field Direction auto-seek                       */
/*               (extension of EncoderManager)                              */
/*                                                                          */
/****************************************************************************/

#ifndef _ENCODEREFSEEK_H
#define _ENCODEREFSEEK_H

#include "common\CommonDefines.h"
#include "common\CommonMotorHandler.h"
#include "common\CommonEncoderManager.h"

//***************************************************************************
// Electrical Field Auto find types

#define EFS_TYPE_DISABLED       0
#define EFS_TYPE_IDRAMP         1
#define EFS_TYPE_IDR_AND_PLOOP  2
#define EFS_TYPE_IDR_LOCAL_PID  3

//***************************************************************************
// Return values

#define EFS_RV_INPROGRESS       1
#define EFS_RV_TERMINATED       2
#define EFS_RV_FAULT            3
#define EFS_RV_ABORTED         -1

//****************************************************************************
// Values

#define EFS_VL_ELECANGLE_MIN    ((UWORD)((60.0)*182.0444))  // 60deg electrical
#define EFS_VL_IDRLPID_EA_MIN   ((UWORD)(( 80.0)*182.0444)) // 90 - 10 deg electrical
#define EFS_VL_IDRLPID_EA_MAX   ((UWORD)((100.0)*182.0444)) // 90 + 10 deg electrical
																			   
//****************************************************************************
// Data structures
/*
typedef union
{
    struct
    {
        UWORD bForce:1;       // force procedure also if ENCMGR_ELE_ANGLE_VALID
    } f;
    UWORD w;
} EFS_OPTIONS;
*/


typedef struct {
    SWORD swErr ;  
    SLONG slPiOut ;
    SWORD swCorrection ;
} EFS_PI_OUTPUT ;

typedef struct {
    SLONG slIntegral32 ; 
    SWORD swFreezedRef ;
} EFS_PI_RUNTIME ;

//****************************************************************************
// Input/Output data structures

typedef struct
{
    UBYTE ubProcType;
    BOOL bForce; //EFS_OPTIONS options;

    MH_MOTORDATA_OUT * psPSStatus;
    ENCMGR_SPACEFEEDBACK * psFeedback;
    GLB_IREF * psIRefIn;
    GLB_IREF * psIRefOut;
    GLB_IREF * psSpdLoopIRefIn;

    UWORD uwTickCnt;
    SLONG slSpeedThreshold;
    UWORD uwEAngleSnapshot;
    UWORD uwEAngleFinal;
    SLONG slEAngleSnapshotFilter;
    SLONG slEAngleFinalFilter;

    ULONG ulIdRampCurrentStep;
    UWORD uwIdRampQTicksCounter;
    UWORD uwIdRampTicksCounter;
    UWORD uwIdSteadyTicksCounter;
    UWORD uwEAngleSnapshotSteadyTicksCounter;
    UWORD uwEAngleFinalSteadyTicksCounter;

    SLONG slElecAngleMult;

#if CFG_ENCMGR_OPENLOOP
    ULONG ulIdRampCurrent ; // 1e-4A
#endif
    EFS_PI_RUNTIME sPI_Run ;
} EFS_RUNTIME;


typedef struct
{
    UWORD uwPhasingQuality; // 0

    EFS_PI_OUTPUT sPI_Out ; // 2-4-8
    UWORD uwEfsStatus; // to trigger 10
    UWORD uwEfsElecAngleCorrection; // to check correction 12

    SWORD swEfsPiAutoKi ; // 14
    SWORD swEfsPiAutoKp ; // 16
    SWORD swEfsPiAutoShift ; // 18
} EFS_OUT;
      
//****************************************************************************
// Parameters data structure
typedef struct {
    SWORD swKi ;
    SWORD swKp ;
    SWORD swErrMax ;
    SWORD swGlobalShift ;
    SLONG slOutValLimit ;
    SLONG slIntegralLimit ;

    union {
        struct {
            UWORD bNoLimits : 1 ;
        } b ;
        UWORD w ;
    } flags ;
} EFS_PI_PARAM ;

typedef struct
{
    UBYTE ubProcType;           // procedure type
    UBYTE ubDummy;

    UWORD uwIdRampCurrent;      // [0:1000=0:MOTPRM_PARAMETERS.flCurrentPeak]
    UWORD uwIdRampTime;         // [msec]

    BOOL bForce;//EFS_OPTIONS options;
    SLONG slSpeedThreshold;     // wait for speed below this threshold

    SLONG slElecAngleFeed;      // [0:1000=0:IqRef]
    UWORD uwIdSteadyTime;       // [msec]

    EFS_PI_PARAM sPI_Param;
} EFS_PARAMS;

//****************************************************************************
// Space Speed Control loop input data structure

typedef struct
{
    MOTCTRL_STATUS sFlags;
    GLB_KINEMATIC_DATA sDemand;
} EFS_SSIN_IDR_AND_PLOOP;

//****************************************************************************
// General global variables

extern EFS_PARAMS               sEfsParam;
#ifdef _INFINEON_
extern const EFS_PARAMS huge    sEfsDefParam;
extern EFS_PARAMS sdata         sEfsCheckedParam;
#else
extern const EFS_PARAMS         sEfsDefParam;
extern EFS_PARAMS               sEfsCheckedParam;
#endif 
extern EFS_SSIN_IDR_AND_PLOOP   sEfsSSInIdrAndPLoop;
extern EFS_OUT                  sEfsOut;

//****************************************************************************
// Global functions

void EfsInit(void);
SWORD EfsSetup(MH_MOTORDATA_OUT * psPSStatus,ENCMGR_SPACEFEEDBACK * psFeedback,GLB_IREF * psIRefIn,GLB_IREF * psIRefOut,GLB_IREF * psSpdLoopIRefIn,EFS_RUNTIME * psRuntime);
SWORD EfsProcess(EFS_RUNTIME * psRuntime, UWORD * puwElecAngleCorrection);

BOOL EfsParametersCheck(void);
void EfsSlowTask(void);

#endif
