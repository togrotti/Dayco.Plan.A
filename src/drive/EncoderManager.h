/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EncoderManager.h                                           */
/* Author      : Fabio Terrile, Cristiano Tognetti                          */
/*                                                                          */
/* Description : Defines common handling to differents encoder sources and  */
/*               combinations												*/
/*                                                                          */
/****************************************************************************/

#ifndef _ENCODERMANAGER_H
#define _ENCODERMANAGER_H

#include "common\CommonDefines.h"
#include "common\CommonMotorHandler.h"
#include "common\CommonController.h"
#include "common\CommonEncoderManager.h"
#include "drive\MotorParameters.h"

//***************************************************************************
// Defines

#define ENCMGR_INIT_EPLATE              0
#define ENCMGR_INIT_PLC                 1
#define ENCMGR_INIT_CORE                2
#define ENCMGR_INIT_HOOK                3
#define ENCMGR_INIT_PLCOPTIONS          4

#define ENCMGR_FILTEREDSPEED_NBIT       9
#define ENCMGR_FILTEREDSPEED_TIME       (UWORD)((1000000/REALTIME_TASK_FREQ)*(1l<<(ENCMGR_FILTEREDSPEED_NBIT))/1000)    // msec

#define ENCMGR_DIS_FAULTMGM_KEY         0x17A8

//***************************************************************************
// Encoder types

#define ENCODER_TYPE_NULL               0 // No sensor

#define ENCODER_TYPE_ABS_HALL           1 // Absolute Encoder: Hall
#define ENCODER_TYPE_ABS_ANALOG         3 // Absolute Encoder: sincos
#define ENCODER_TYPE_ABS_ENDAT          4 // Absolute Encoder: Endat

#define ENCODER_TYPE_REL_BACK_EMF       5 // Sensorless backemf
#define ENCODER_TYPE_REL_INCREMENTAL    6 // Digital Incremental Encoder
#define ENCODER_TYPE_REL_INCRSIMUL      8 // Digital Incremental Encoder Simulation

#define ENCODER_TYPE_ABS_B1_ENDAT       10 // Absolute Encoder: Endat from Bridge 1
#define ENCODER_TYPE_ABS_HIPERFACE      11 // Absolute Encoder: Hiperface
#define ENCODER_TYPE_ABS_NIKON          12 // Absolute Encoder: Nikon
#define ENCODER_TYPE_ABS_TMGW           13 // Absolute Encoder: TAMAGAWA
#define ENCODER_TYPE_ABS_BISS           14 // Absolute Encoder: BISS

//****************************************************************************
// Input/Output data structures

typedef struct
{
    const COMCTRL_HANDLERS  *psCtrlHandlers ;
    MH_MOTORDATA_OUT        *psMotorData;
    GLB_IREF                *sIRefIn;
    MH_POWERSTAGECONTROL    *sPowerStageCtrlIn ;
    GLB_IREF                *sSpdLoopIRefIn;
} ENCMGR_IN ;

typedef struct
{
    union {
        struct {
            BOOL bRequireIRefHook;
        } b ;
        UWORD w ;
    } flags ;
    GLB_IREF                sIRefOut;
    MH_POWERSTAGECONTROL    sPowerStageCtrlOut ;
} ENCMGR_OUT ;

//****************************************************************************
// Parameters data structure

typedef union {
    struct {
        BOOL bPos2CntrLoop     ; /* boolean to select which position-information will be used by the control loop (primary/auxiliary encoder)     */
        BOOL bSpd2CntrLoop     ; /* boolean to select which speed-information will be used by the control loop (primary/auxiliary encoder)        */
        BOOL bAcc2CntrLoop     ; /* boolean to select which acceleration-information will be used by the control loop (primary/auxiliary encoder) */
        BOOL bElecAngle2Fpga   ; /* boolean to select which electrical angle-information will be used by the FPGA (primary/auxiliary encoder)     */
        BOOL bDisableIfRelFail ; /* disable power if relative fail, otherwise (default) switch to absolute and emit NONFATAL fault                */
        BOOL bDisableEPlate    ; /* disable electronic plate reading at startup                                                                   */
        BOOL bRestoreEPlate    ; /* restore original electronic plate parameters at next reset                                                    */
        BOOL bDisAbsAfterValid ; /* disable absolute track processing after initial position validity                                             */
        BOOL bMainPosSel       ; /* abs/rel pos force abs */
        BOOL bMainSpdSel       ; /* abs/rel spd force abs */
        BOOL bMainAccSel       ; /* abs/rel acc force abs */
        BOOL bMainElecAngleSel ; /* abs/rel elec angle force abs */
        BOOL bSpeedFilter      ; /* enable speed filtering */
        // BOOL bDummy            ;
    } b ;
    // UWORD w[8] ;
#ifdef _INFINEON_
}                             ENCMGR_PARFLAGS ;
#else
} __attribute__((aligned(4))) ENCMGR_PARFLAGS;
#endif

typedef struct {
    ENCMGR_PARFLAGS flags ;     // flags
    
    UWORD uwVEncVoltage;        // Encoder Supply Voltage [12.5mV/step]

    UWORD uwMainAbsSel;         // parameter to select the main absolute encoder to use
    UWORD uwAuxSel;             // parameter to select the auxiliary encoder to use
    
    SQWRD sqDummy;
    
    UWORD uwVEncDelay;          // Delay after applying supply voltage [msec]

    UWORD uwMainRelSel;         // parameter to select the main relative encoder to use

    UWORD uwMaxAbsRelDiff;      // Max angle diff between Abs and Rel tracks for redundancy control [0:65535=0:359deg]

    UWORD uwAuxVEncVoltage;     // Aux Encoder Supply Voltage [12.5mV/step]

    SWORD swElecAngleFFTime;    // Electrical Angle feed forward time [usec]

    SLONG slMaxSpeed;           // Maximum allowed speed [d.u.]

    UWORD uwSimSel;             // parameter to select the simulation encoder to use
} ENCMGR_PARAMS ;

//****************************************************************************
// Old Parameters

typedef struct {
    union {
        struct {
            UWORD bPos2CntrLoop     :1 ; /* boolean to select which position-information will be used by the control loop (primary/auxiliary encoder)     */
            UWORD bSpd2CntrLoop     :1 ; /* boolean to select which speed-information will be used by the control loop (primary/auxiliary encoder)        */
            UWORD bAcc2CntrLoop     :1 ; /* boolean to select which acceleration-information will be used by the control loop (primary/auxiliary encoder) */
            UWORD bElecAngle2Fpga   :1 ; /* boolean to select which electrical angle-information will be used by the FPGA (primary/auxiliary encoder)     */
            UWORD bDisableIfRelFail :1 ; /* disable power if relative fail, otherwise (default) switch to absolute and emit NONFATAL fault                */
            UWORD bDisableEPlate    :1 ; /* disable electronic plate reading at startup                                                                   */
            UWORD bRestoreEPlate    :1 ; /* restore original electronic plate parameters at next reset                                                    */
        } b ;
        UWORD w ;
    } flags ;
    
    UWORD uwVEncVoltage;        // Encoder Supply Voltage [12.5mV/step]

    UWORD uwMainAbsSel;         // parameter to select the main absolute encoder to use
    UWORD uwAuxSel;             // parameter to select the auxiliary encoder to use
    
    SQWRD sqMechOffset;         // Encoder Mechanical Offset
    
    UWORD uwVEncDelay;          // Delay after applying supply voltage [msec]

    UWORD uwMainRelSel;         // parameter to select the main relative encoder to use

    UWORD uwMaxAbsRelDiff;      // Max angle diff between Abs and Rel tracks for redundancy control [0:65535=0:359deg]

    MOTPRM_V14_PARAMETERS sEPlate;  // Last valid eplate read, if zero filled then invalid

} ENCMGR_V14_PARAMS ;

//****************************************************************************
// General global variables

extern ENCMGR_IN                sEm_EncMngrIn ;
extern ENCMGR_OUT               sEm_EncMngrOut;

extern ENCMGR_PARAMS            sEm_EncMngrParam ;
extern const ENCMGR_PARAMS      sEm_EncMngrDefParam ;
extern MOTPRM_PARAMETERS        sEm_LastValidEPlate ;  // Last valid eplate read, if zero filled then invalid

extern ENCMGR_SPACEFEEDBACK     sEm_MainAbsEnc ;
extern ENCMGR_SPACEFEEDBACK     sEm_MainRelEnc ;
extern ENCMGR_SPACEFEEDBACK     sEm_MainEnc ;
extern ENCMGR_SPACEFEEDBACK     sEm_AuxEnc ;
extern ENCMGR_SPACEFEEDBACK     sEm_Fbk2CntrLoop ;
extern ENCMGR_SPACEFB_EXT       sEm_Fbk2CLExt ;

//****************************************************************************
// PLC-only global variables

extern ENCMGR_SPACEFEEDBACK     sEm_PlcSimulationEnc;
extern ENCMGR_SPACEFEEDBACK     sEm_PlcFbk2CntrLoop ;
extern ENCMGR_SPACEFB_EXT       sEm_PlcFbk2CLExt ;

//****************************************************************************
// PLC-only options

    // enable sincos hw analog channels conditioning and reading
#define ENCMGR_ENABLE_SINCOSHW      1
    // setup encoder voltage as parameter [mV]
#define ENCMGR_SETUP_VENCODER       2
    // disable fault management, for security valid only in preboot/boot
    // and with specific key access as parameter
#define ENCMGR_DIS_FAULTMGM         3
    // setup aux encoder voltage as parameter [mV]
#define ENCMGR_SETUP_AUXVENCODER    4
    // enable incremental hw analog channels conditioning and reading
#define ENCMGR_ENABLE_INCREMENTALHW 5

//****************************************************************************
// Global functions

BOOL Em_EncMngrInit(UWORD uwTaskParam) ;
SWORD Em_GetV14Params(ENCMGR_V14_PARAMS  * pvSrc, UWORD uwSize);
BOOL Em_SetPlcOption(UWORD uwOption, ULONG ulValue);
void Em_GetHwOpt(UWORD t, ULONG *, ULONG *);

#endif
