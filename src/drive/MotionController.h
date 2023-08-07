/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : MotionController.h                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Motion Controller Manager, command interface is compatible */
/*               with DSP402 V2.0; it is responsible to activate and        */
/*               deactivate needed modules and to create variables          */
/*               connections between them                                   */
/*                                                                          */
/****************************************************************************/

#ifndef _MOTIONCONTROLLER_H
#define _MOTIONCONTROLLER_H

#include "common\CommonController.h"
#include "common\CommonMotorHandler.h"
#include "common\CommonEncoderManager.h"
#include "drive\Positioner.h"

//***************************************************************************
// Defines

#define MOTCTRL_SWITCHSEL_OFF       255
#define MOTCTRL_SWITCHSEL_SOFT      254
#define MOTCTRL_SWITCHSEL_INVERT    128
#define MOTCTRL_SWITCHSEL_INMASK    127

#define MOTCTRL_IGNORE_ENCSTAT_KEY  0xF57F

//***************************************************************************
// Data types

typedef struct
{
    UWORD   uwControlWord ; /* controlword to use */
    SQWRD   sqTargetPostn ; /* Target Position to use */

    SQWRD   sqIPQuota ;     /* Quota for next interpolation */

    SBYTE   sbHomingSoftPosSwitch  ;
    SBYTE   sbHomingSoftNegSwitch  ;
    SBYTE   sbHomingSoftHomeSwitch ;

    SWORD   swTargetTorque; /* Target torque when in CST */
} MOTCTRL_CNTRL_IN ;


typedef struct
{
    MOTCTRL_CNTRL_IN        *psMotCtrl_In ;
    MH_POWERSTAGESTATUS     *psPowerStageStatus ;
    const MH_HANDLERS       *psMotorHandler ;
    ENCMGR_SPACEFEEDBACK    *psFbk2CntrLoop ;
    PO_POSITIONER_OUT       *psPoOut ;
} MOTCTRL_IN ;


typedef struct
{
    MOTCTRL_STATUS dummy ;    

    UWORD   uwStatusWord ;
    UBYTE   ubModeOfOperationDisplay ;

    MH_POWERSTAGECONTROL sPowerStageCtrl ;

    SLONG   slIPQuotaMonitor ;
    SQWRD   sqUserOffset ;

    SLONG   slCSCurrentToTorque ;
    SLONG   slCSTorqueToCurrent ;

    GLB_IREF sIRefs ;
} MOTCTRL_OUT ;


typedef struct
{
    UBYTE   ubModeOfOperation;
    UBYTE   ubDummy;

    SWORD   swOptQuickStop;         // quickstop option code
    SWORD   swOptShutdown;          // shutdown option code
    SWORD   swOptDisableOp ;        // disable operation option code
    SWORD   swOptHalt ;             // halt option code
    SWORD   swOptFaultReaction;     // fault reaction option code

    SQWRD   sqFollowingErrWindow;   // following error window
    SWORD   swFollowingErrTimeout ; // following error timeout

    SQWRD   sqPositionWindow ;      // target position window
    SWORD   swPositionWinTimeout ;  // target position timeout

    SLONG   slVelocityWindow ;      // velocity window
    SWORD   swVelocityWinTimeout ;  // velocity timeout

    SLONG   slVelocityThreshold ;   // velocity threshold
    SWORD   swVelocityThrTimeout ;  // velocity threshold timeout

    UBYTE   ubIPTimeUnits ;         // IP time period units
    SBYTE   sbIPTimeIndex ;         // IP time period index

    SLONG   slHomingSpeedSwitch ;   // homing speed while searching switch
    SLONG   slHomingSpeedZero ;     // homing speed while searching zero
    SLONG   slHomingAcceleration ;  // homing acceleration
    UBYTE   ubHomingMethod ;        // homing method
    UBYTE   ubHomingSSrcPosSwitch ; // homing signal source positive switch
    UBYTE   ubHomingSSrcNegSwitch ; // homing signal source negative switch
    UBYTE   ubHomingSSrcHomeSwitch ;// homing signal source home switch
    SQWRD   sqHomingUserOffset;     // homing user offset

    ULONG   ulMotorRatedTorque;     // motor rated torque [mNm], if zero take peak
                                    // torque calculated from motor parameters
    /* Flags */
    union {
        struct {
            UWORD bIgnoreHomingDoneBit : 1 ;  // flag: allow infinite homing sequence
        } b ;
        UWORD w ;
    } flags ;
} MOTCTRL_PARAM;


//***************************************************************************
// Globals

extern MOTCTRL_CNTRL_IN sMotCtrl_UsrControl ;

extern MOTCTRL_IN    sMotCtrl_In ; 
extern MOTCTRL_OUT   sMotCtrl_Out ;

#ifdef _INFINEON_
extern MOTCTRL_STATUS bdata sMotCtrl_OStatus ;
#else
extern MOTCTRL_STATUS sMotCtrl_OStatus ;
#endif // _infineon_

extern MOTCTRL_PARAM sMotCtrlParameters;

#ifdef _INFINEON_
extern const MOTCTRL_PARAM huge sMotCtrlDefParameters ;
#else
extern const MOTCTRL_PARAM sMotCtrlDefParameters ;
#endif // _infineon_

extern const COMCTRL_HANDLERS sMotCtrlHandlers;

extern MOTCTRL_CNTRL_IN sMotCtrl_PlcControl ;

extern const ULONG   ulMotCtrlSuppDriveModes;

//****************************************************************************
// PLC-only options

    // ignore encoder status assuming always valid
#define MOTCTRL_IGNORE_ENCSTAT      1

//***************************************************************************
// Motion Controller

BOOL MotCtrlInit(void);
UWORD MotCtrl_AbsolutePositionScaling(BOOL bDirection, void * pvDst, void * pvSrc);
UWORD MotCtrl_RelativePositionScaling(BOOL bDirection, void * pvDst, void * pvSrc);
UWORD MotCtrl_RatedTorqueScaling(BOOL bDirection, void * pvDst, void * pvSrc);
BOOL MotCtrl_SetPlcOption(UWORD uwOption, ULONG ulValue);

#endif
