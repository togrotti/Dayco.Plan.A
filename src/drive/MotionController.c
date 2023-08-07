/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright ?2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : MotionController.c                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Motion Controller Manager, command interface is compatible */
/*               with DSP402 V2.0; it is responsible to activate and        */
/*               deactivate needed modules and to create variables          */
/*               connections between them                                   */
/*                                                                          */
/****************************************************************************/
#include <math.h>
#include <stdlib.h>	// to use labs()

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "drive\AxM-E-Defines.h"
#include "system\SysAppGlobals.h"
#include "system\SystemAlarms.h"
#include "common\TaskScheduler.h"
#include "system\SysLogManagement.h"
#include "drive\MotionController.h"
#include "drive\SpaceSpeedCntrLp.h"
#include "drive\PiDcBusCntrLp32bit.h"
#include "common\CommonMotorHandler.h"
#include "drive\Positioner.h"
#include "drive\DriveTaskController.h"
#include "common\Int64Functions.h"
#include "common\DspFunctions.h"
#include "system\Os.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "drive\UserIO.h"
#ifdef _INFINEON_
#include "UserIO.h"
#endif // _infineon_
#include "common\CommonParamDB.h"

//****************************************************************************
#ifdef _INFINEON_
// Avoid warning C113: nonstandard extension

#pragma warning disable = 113
#else
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
//#pragma GCC optimize (0)
#pragma GCC optimize (2)

#endif // _infineon_

//***************************************************************************
// Interpolator recovery factor

#define IPMODE_RECOVERYFACTOR                   (3)

//***************************************************************************
// Flags controlword (dsp402)

#define MOTCTRL_402_CF_SWITCHON                 0x0001
#define MOTCTRL_402_CF_ENABLEVOLTAGE            0x0002
#define MOTCTRL_402_CF_QUICKSTOP                0x0004
#define MOTCTRL_402_CF_ENABLEOPERATION          0x0008
#define MOTCTRL_402_CF_FAULTRESET               0x0080
#define MOTCTRL_402_CF_HALT                     0x0100

#define MOTCTRL_402_CF_PP_NEWSETPOINT           0x0010
#define MOTCTRL_402_CF_PP_IMMCHANGE             0x0020
#define MOTCTRL_402_CF_PP_RELATIVE              0x0040

#define MOTCTRL_402_CF_IP_ENABLEIPMODE          0x0010

#define MOTCTRL_402_CF_HM_OP_START              0x0010

//#define MOTCTRL_402_CF_RT_NEWPOINT              0x0010
//#define MOTCTRL_402_CF_RT_ABSOLUTE              0x0020
//#define MOTCTRL_402_CF_RT_RELATIVE              0x0040

//***************************************************************************
// Flags statusword (dsp402)

#define MOTCTRL_402_SF_READYTOSWITCHON          0x0001
#define MOTCTRL_402_SF_SWITCHEDON               0x0002
#define MOTCTRL_402_SF_OPERATIONENABLED         0x0004        //状态切换到使能运行
#define MOTCTRL_402_SF_FAULT                    0x0008
#define MOTCTRL_402_SF_QUICKSTOP                0x0020
#define MOTCTRL_402_SF_SWITCHONDISABLED         0x0040
#define MOTCTRL_402_SMASK_STATEMACHINE          0x006f

#define MOTCTRL_402_SF_VOLTAGENABLED            0x0010
#define MOTCTRL_402_SF_WARNING                  0x0080
#define MOTCTRL_402_SF_REMOTE                   0x0200
#define MOTCTRL_402_SF_TARGETREACHED            0x0400    
#define MOTCTRL_402_SF_INTERNALIMITACTIVE       0x0800

#define MOTCTRL_402_SF_PP_SETPOINTACK           0x1000
#define MOTCTRL_402_SF_PP_FOLLOWINGERR          0x2000

#define MOTCTRL_402_SF_PV_SPEED                 0x1000
#define MOTCTRL_402_SF_PV_MAXSLIPPAGEERR        0x2000

#define MOTCTRL_402_SF_IP_IPMODEACTIVE          0x1000

#define MOTCTRL_402_SF_HM_ATTAINED              0x1000
#define MOTCTRL_402_SF_HM_ERROR                 0x2000

#define MOTCTRL_402_SF_RT_POINTACK              0x1000

#define MOTCTRL_402_SF_MODULO_ENABLED           0x4000
#define MOTCTRL_402_SF_HOMING_DONE              0x8000

#define MOTCTRL_402_SF_CYCLICTARGETUSED         0x1000

//***************************************************************************
// Mode of operation (dsp402)

#define MOTCTRL_402_OM_PROFILEPOSITION          1               // pp
#define MOTCTRL_402_OM_PROFILEVELOCITY          3               // pv
#define MOTCTRL_402_OM_HOMING                   6               // hm
#define MOTCTRL_402_OM_INTERPOLATED             7               // ip
#define MOTCTRL_402_OM_CYCLICSYNCPOSITION       8               // csp
#define MOTCTRL_402_OM_CYCLICSYNCVELOCITY       9               // csv
#define MOTCTRL_402_OM_CYCLICSYNCTORQUE         10              // cst
#define MOTCTRL_402_OM_PMC_TORQUE               128             // pmc-tq

#define MOTCTRL_SUPPDRIVEMODES                  0x00003E5ul     // (0b1111100101) cst,csv,csp,ip,hm,pp,pv
#define MOTCTRL_402_DEF_MODEOFOP                (MOTCTRL_402_OM_PROFILEVELOCITY)

//***************************************************************************
// Stop mode

#define MOTCTRL_402_OC_POWEROFF         0 
#define MOTCTRL_402_OC_SLOWDOWN         1 
#define MOTCTRL_402_OC_QUICKSTOP        2 
#define MOTCTRL_402_OC_SLOWDOWNANDSTAY  5
#define MOTCTRL_402_OC_QUICKSTOPANDSTAY 6

//***************************************************************************
// stato del device control utilizzato in slowtasks

#define LOCALSTATUS_SWITCHONDISABLED    2
#define LOCALSTATUS_READYTOSWITCHON     3
#define LOCALSTATUS_SWITCHEDON          4
#define LOCALSTATUS_OPERATIONENABLED    5
#define LOCALSTATUS_QUICKSTOPACTIVE     6
#define LOCALSTATUS_FAULTREACTIONACTIVE 7
#define LOCALSTATUS_FAULT               8
#define LOCALSTATUS_FAULTHIGH           9

#define GF_NOFAULT                      0
#define GF_NONFATALFAULT                1
#define GF_FATALFAULT                   2
#define GF_FATALFAULTRECOVERING         3

// local flag
#define LF_RESET                        0x0000                                          
#define LF_DISABLECONTROLWORDINPUT      0x0001
#define LF_FORCESTATUS                  0x0002
#define LF_DOPWROFFSTOP                 0x0004 // DOing PoWerOFF STOP
#define LF_DOHALT                       0x0008
#define LF_NEWSETPOINTSET               0x0010
#define LF_NEWTARGETSENT                0x0020  
#define LF_HM_OP_STARTED                0x0040

#define MOTCTRL_402_QUICKSTOP           10          // quickstop option mode
#define MOTCTRL_402_SHUTDOWN            11          // shutdown option mode
#define MOTCTRL_402_DISABLEOP           12          // disable operation option mode
#define MOTCTRL_402_HALT                13          // halt option mode
#define MOTCTRL_402_FAULTREACTION       14          // fault reaction option mode

#define MOTCTRL_CMD_ENABLE_POWER        1
#define MOTCTRL_CMD_ENABLE_REFERENCE    2
#define MOTCTRL_CMD_ENABLE_ZERO_SPEED   33
#define RESETMASK_HIGHSTATUSWORD                  (0x3400) // mask to clean bit only 10,12,13 of statusword - high part
#define RESETMASK_HIGHSTATUSWORD_IGNOREHOMINGDONE (0xB400) // mask to clean bit only 10,12,13,15 of statusword - high part

//***************************************************************************
// IP limits

#define IP_MIN_CYCLES                   2           // 250us
#define IP_MAX_CYCLES                   512         // 64ms

//***************************************************************************
// Homing

#define HM_ENDOFSEQUENCE                0x0000

#define HM_WAITHINEGLIMSWITCH           0x0001
#define HM_WAITLONEGLIMSWITCH           0x0002
#define HM_WAITHIPOSLIMSWITCH           0x0004
#define HM_WAITLOPOSLIMSWITCH           0x0008
#define HM_WAITRISEHOMESWITCH           0x0010
#define HM_WAITFALLHOMESWITCH           0x0020
#define HM_WAITHIHOMESWITCH             0x0040
#define HM_WAITLOHOMESWITCH             0x0080
#define HM_WAITINDEXPULSE               0x0100
#define HM_NOTRIGGERS                   0x0200
#define HM_SETNEGATIVESPEED             0x0400
#define HM_SETZEROSPEED                 0x0800
#define HM_CATCHPOSITION                0x1000

typedef struct
{
    UBYTE ubMethod;
    UWORD uwSeq[7];
} HM_SEQUENCER;

static const HM_SEQUENCER sHmSeq[]=
{
    {0,{
        HM_NOTRIGGERS|HM_SETZEROSPEED,
        HM_ENDOFSEQUENCE,
       }},
    {1,{
        HM_WAITHINEGLIMSWITCH|HM_SETNEGATIVESPEED,
        HM_WAITLONEGLIMSWITCH|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {2,{
        HM_WAITHIPOSLIMSWITCH,
        HM_WAITLOPOSLIMSWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {3,{
        HM_WAITHIHOMESWITCH,
        HM_WAITFALLHOMESWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {4,{
        HM_WAITLOHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITRISEHOMESWITCH|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {5,{
        HM_WAITHIHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITFALLHOMESWITCH|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {6,{
        HM_WAITLOHOMESWITCH,
        HM_WAITRISEHOMESWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {7,{
        HM_WAITHIHOMESWITCH|HM_WAITHIPOSLIMSWITCH,
        HM_WAITHIHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITFALLHOMESWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {8,{
        HM_WAITHIHOMESWITCH|HM_WAITHIPOSLIMSWITCH,
        HM_WAITFALLHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITRISEHOMESWITCH|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {9,{
        HM_WAITHIHOMESWITCH|HM_WAITHIPOSLIMSWITCH,
        HM_WAITHIHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITFALLHOMESWITCH,
        HM_WAITRISEHOMESWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {10,{
        HM_WAITHIHOMESWITCH|HM_WAITHIPOSLIMSWITCH,
        HM_WAITHIHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITFALLHOMESWITCH|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {11,{
        HM_WAITHIHOMESWITCH|HM_WAITHINEGLIMSWITCH|HM_SETNEGATIVESPEED,
        HM_WAITHIHOMESWITCH,
        HM_WAITFALLHOMESWITCH|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {12,{
        HM_WAITHIHOMESWITCH|HM_WAITHINEGLIMSWITCH|HM_SETNEGATIVESPEED,
        HM_WAITFALLHOMESWITCH,
        HM_WAITRISEHOMESWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {13,{
        HM_WAITHIHOMESWITCH|HM_WAITHINEGLIMSWITCH|HM_SETNEGATIVESPEED,
        HM_WAITHIHOMESWITCH,
        HM_WAITFALLHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITRISEHOMESWITCH|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {14,{
        HM_WAITHIHOMESWITCH|HM_WAITHINEGLIMSWITCH|HM_SETNEGATIVESPEED,
        HM_WAITHIHOMESWITCH,
        HM_WAITFALLHOMESWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED,
        HM_WAITINDEXPULSE|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {17,{
        HM_WAITHINEGLIMSWITCH|HM_SETNEGATIVESPEED,
        HM_WAITLONEGLIMSWITCH|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {18,{
        HM_WAITHIPOSLIMSWITCH,
        HM_WAITLOPOSLIMSWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {19,{
        HM_WAITHIHOMESWITCH,
        HM_WAITFALLHOMESWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {20,{
        HM_WAITLOHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITRISEHOMESWITCH|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {21,{
        HM_WAITHIHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITFALLHOMESWITCH|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {22,{
        HM_WAITLOHOMESWITCH,
        HM_WAITRISEHOMESWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {23,{
        HM_WAITHIHOMESWITCH|HM_WAITHIPOSLIMSWITCH,
        HM_WAITFALLHOMESWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {24,{
        HM_WAITHIHOMESWITCH|HM_WAITHIPOSLIMSWITCH,
        HM_WAITFALLHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITRISEHOMESWITCH|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {25,{
        HM_WAITHIHOMESWITCH|HM_WAITHIPOSLIMSWITCH,
        HM_WAITHIHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITFALLHOMESWITCH,
        HM_WAITRISEHOMESWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {26,{
        HM_WAITHIHOMESWITCH|HM_WAITHIPOSLIMSWITCH,
        HM_WAITHIHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITFALLHOMESWITCH|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {27,{
        HM_WAITHIHOMESWITCH|HM_WAITHINEGLIMSWITCH|HM_SETNEGATIVESPEED,
        HM_WAITFALLHOMESWITCH|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {28,{
        HM_WAITHIHOMESWITCH|HM_WAITHINEGLIMSWITCH|HM_SETNEGATIVESPEED,
        HM_WAITFALLHOMESWITCH,
        HM_WAITRISEHOMESWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {29,{
        HM_WAITHIHOMESWITCH|HM_WAITHINEGLIMSWITCH|HM_SETNEGATIVESPEED,
        HM_WAITHIHOMESWITCH,
        HM_WAITFALLHOMESWITCH|HM_SETNEGATIVESPEED,
        HM_WAITRISEHOMESWITCH|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {30,{
        HM_WAITHIHOMESWITCH|HM_WAITHINEGLIMSWITCH|HM_SETNEGATIVESPEED,
        HM_WAITHIHOMESWITCH,
        HM_WAITFALLHOMESWITCH|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {33,{
        HM_WAITINDEXPULSE|HM_SETNEGATIVESPEED|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {34,{
        HM_WAITINDEXPULSE|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
    {35,{
        HM_NOTRIGGERS|HM_SETZEROSPEED|HM_CATCHPOSITION,
        HM_ENDOFSEQUENCE,
       }},
};

//***************************************************************************
// Globals

MOTCTRL_CNTRL_IN sMotCtrl_UsrControl ;
MOTCTRL_CNTRL_IN sMotCtrl_PlcControl ;

MOTCTRL_IN    sMotCtrl_In ;
MOTCTRL_OUT   sMotCtrl_Out ;
#ifdef _INFINEON_
MOTCTRL_STATUS bdata sMotCtrl_OStatus ; 
#else
MOTCTRL_STATUS sMotCtrl_OStatus ; 
#endif // _infineon_
MOTCTRL_PARAM sMotCtrlParameters ;

// default
#ifdef _INFINEON_
const MOTCTRL_PARAM huge sMotCtrlDefParameters =
#else
const MOTCTRL_PARAM sMotCtrlDefParameters =
#endif // _infineon_
{
    MOTCTRL_402_DEF_MODEOFOP,
    0, // dummy

    MOTCTRL_402_OC_QUICKSTOP,       // quickstop option code
    MOTCTRL_402_OC_POWEROFF,        // shutdown option code: disabilitazione drive
    MOTCTRL_402_OC_SLOWDOWN,        // disable operation option code: fermata con rampa di decelerazione normale
    MOTCTRL_402_OC_SLOWDOWN,        // halt option code: fermata con rampa di decelerazione normale 
    MOTCTRL_402_OC_QUICKSTOP,       // fault reaction option code: fermata con rampa di decelerazione quickstop

    {0,1},                          // following error window: +/- 1 giro
    10,                             // following error timeout: 10ms 

    {0x00400000,0},                 // target position window: +/- 5deg
    20,                             // target position timeout: 20ms

    (ULONG)(SPEED_K_CONVERSION*100),// velocity window: 100rpm
    30,                             // velocity timeout: 30ms

    (ULONG)(SPEED_K_CONVERSION*20), // velocity threshold: 20rpm
    80,                             // velocity threshold timeout: 80ms

    1,                              // IP time period units: 1e-3s
    -3,                             // IP time period index: 1e-3s

    (ULONG)(SPEED_K_CONVERSION*100),// homing speed while searching switch: 100rpm
    (ULONG)(SPEED_K_CONVERSION*10), // homing speed while searching zero: 10rpm
    1000000,                        // homing acceleration
    1,                              // homing method
    3,                              // homing signal source positive switch
    4,                              // homing signal source negative switch
    7,                              // homing signal source home switch
    {0,0},                          // homing user offset

    0ul,                            // motor rated torque: 0, automatic calculation
    {0}                             // no infinite homing
} ;

const ULONG ulMotCtrlSuppDriveModes=MOTCTRL_SUPPDRIVEMODES;

//***************************************************************************
// Locals

// Vettore che serve per costruire la StatusWord
static const UWORD uwLocalStatusFlags[] =
{
    0,0,
    MOTCTRL_402_SF_SWITCHONDISABLED,
    MOTCTRL_402_SF_QUICKSTOP | MOTCTRL_402_SF_READYTOSWITCHON,
    MOTCTRL_402_SF_QUICKSTOP | MOTCTRL_402_SF_READYTOSWITCHON | MOTCTRL_402_SF_SWITCHEDON,
    MOTCTRL_402_SF_QUICKSTOP | MOTCTRL_402_SF_READYTOSWITCHON | MOTCTRL_402_SF_SWITCHEDON | MOTCTRL_402_SF_OPERATIONENABLED,
    MOTCTRL_402_SF_READYTOSWITCHON | MOTCTRL_402_SF_SWITCHEDON | MOTCTRL_402_SF_OPERATIONENABLED,
    MOTCTRL_402_SF_READYTOSWITCHON | MOTCTRL_402_SF_SWITCHEDON | MOTCTRL_402_SF_OPERATIONENABLED | MOTCTRL_402_SF_FAULT,
    MOTCTRL_402_SF_FAULT,
    MOTCTRL_402_SF_FAULT  
} ;

// Supported mode of op and entry point
typedef struct {
    UBYTE ubModeOfOp;
    UWORD (* pfSetupEntryPoint)(BOOL);
} MODEOFOPS;

typedef struct
{
    UWORD   uwImgCntrlWord ;
    UWORD   uwStsDevCntrlIdx ;        // indice del vettore che serve per costruire la StatusWord
    UWORD   uwForcedNextStsDevCntrl ; // variabile per forzare gli stati della macchina a stati
    UWORD   uwLocalFlags2Use ;        // flag 'locali'
    UWORD   uwStopMode ;

    UWORD   uwTgtReachedTimeOut ;
    UWORD   uwInSpeedTimeOut ;
    UWORD   uwFollowingErrorTimeOut ;

    SQWRD   sqOldTargetPostn ;

    SWORD   swOptQuickStop;         // quickstop option code
    SWORD   swOptShutdown;          // shutdown option code
    SWORD   swOptDisableOp ;        // disable operation option code
    SWORD   swOptHalt ;             // halt option code
    SWORD   swOptFaultReaction;     // fault reaction option code

    SQWRD   sqFollowingErrLimMax ;
    SQWRD   sqFollowingErrLimMin ;
    SWORD   swFollowingErrTimeout ; // following error timeout

    SQWRD   sqPosWindowLimitMax ;
    SQWRD   sqPosWindowLimitMin ;
    SWORD   swPositionWinTimeout ;  // target position timeout

    SLONG   slVelocityWindow ;      // velocity window
    SWORD   swVelocityWinTimeout ;  // velocity timeout

    SLONG   slVelocityThreshold ;   // velocity threshold
    SWORD   swVelocityThrTimeout ;  // velocity threshold timeout

    SWORD   swIPNumCycle ;          // translated ip period in RT time cycles
    SWORD   swIPRecoveryNumCycle ;  // recovery ip period in RT time cycles
    SWORD   swIPCycleCnt ;          // cycle counter for corrections
    UWORD   uwIPSpeedFract ;        // fractional part of speed applied every cycle
    SLONG   slIPScaling ;           // scaling factor
    SLONG   slIPSpeedSum ;          // integrator of fractional part
    SLONG   slIPTgtSpeed ;          // actual target speed

    const UWORD * puwHomingRun ;    // homing running sequence pointer
    const UWORD * puwHomingPCmd ;   // homing running actual cmd pointer
    SLONG   slHomingAcceleration ;  // homing acceleration
    SQWRD   sqHomingCalcOffset ;    // homing position calculated offset
    UBYTE   ubHomingSSrcPosSwitch ; // homing signal source positive switch
    UBYTE   ubHomingSSrcNegSwitch ; // homing signal source negative switch
    UBYTE   ubHomingSSrcHomeSwitch ;// homing signal source home switch
    UBYTE   ubHomingPrevHomeSwitch ;// homing previous home switch to detect edges
    SQWRD   sqHomingUserOffset8kHz ;// homing user offset used at 8kHz (avoid to write param sqHomingUserOffset when not in Homing mode)

} MOTCTRL_RUN ;

typedef union {
    struct {
        UWORD bEmcyDisabled             : 1 ; // 0
        UWORD bSysStatBooting_1         : 1 ; // 1
        UWORD bStopMotorSetupDone       : 1 ; // 2
        UWORD bTgtReachedTimerSet       : 1 ; // 3
        UWORD bInSpeedTimerSet          : 1 ; // 4
        UWORD bFollowingErrorTimerSet   : 1 ; // 5
        UWORD bMotorStopped             : 1 ; // 6
        UWORD bPID_IntegralEnable       : 1 ; // 7
        UWORD bAlarmClearRequest        : 1 ; // 8
        UWORD bFaultReaction            : 1 ; // 9
        UWORD bFaultReactActivated      : 1 ; // 10
        UWORD bPrevFieldbusSync         : 1 ; // 11
        UWORD bInterpolationActive      : 1 ; // 12
        UWORD bHomeAttained             : 1 ; // 13
        UWORD bHomeParamError           : 1 ; // 14
        UWORD bIsPositioning            : 1 ; // 15
        UWORD bIgnoreEncStatus          : 1 ; // 16
        UWORD bIgnoreHomingDoneBit      : 1 ; // 17
    } b ;
} MOTCTRL_RFLAGS ; 

static MOTCTRL_RUN          sMotCtrlRun ;

#ifdef _INFINEON_
static MOTCTRL_RFLAGS bdata sMotCtrlRFlags ;
#else
static MOTCTRL_RFLAGS sMotCtrlRFlags ;
#endif // _infineon_

//***************************************************************************
// Local prototypes

static void MotCtrlSlowTask(void) ;
static BOOL MotCtrl8KHz(void) ;
static void EmcyStop(UWORD uwOption) ;
static BOOL SetupModeOfOperation(UBYTE ubNewModeOfOperation, BOOL bActivate, BOOL bRstFromHalt) ;

static UBYTE motCtrl402OmProfilePositionSetup(BOOL bRstFromHalt) ;
static UBYTE motCtrl402OmProfileVelocitySetup(void) ;
static UBYTE motCtrl402OmProfileTorqueSetup(void) ;
static UBYTE motCtrl402OmProfileInterpolatedSetup(void) ;
static UBYTE motCtrl402OmProfileHomingSetup(void) ;
static UBYTE motCtrl402OmProfileCSPositionSetup(void) ;
static UBYTE motCtrl402OmProfileCSVelocitySetup(void) ;
static UBYTE motCtrl402OmProfileCSTorqueSetup(void) ;
static void  motCtrl402QuickStopSetup(UWORD uwStopModeCode, UWORD uwStopModeOption) ;

static void motCtrl402OmProfileVelocity8KHz(void) ;
static void motCtrl402OmProfilePosition8KHz(void) ;    
static void motCtrl402OmProfileTorque8KHz(void) ;
static void motCtrl402OmProfileInterpolated8KHz(void) ;
static void motCtrl402OmProfileHoming8KHz(void) ;
static void motCtrl402OmProfileCSPosition8KHz(void) ;
static void motCtrl402OmProfileCSVelocity8KHz(void) ;
static void motCtrl402OmProfileCSTorque8KHz(void) ;
static void ExecuteQuickstop8KHz(void) ;
static void ExecuteShutdown8KHz(void) ;
static void ExecuteDisableOperation8KHz(void) ;
static void ExecuteHalt8KHz(void) ;
static void ExecuteFaultReaction8KHz(void) ;

static BOOL PowerOffStop(UWORD uwPowerOffStopCommand) ;
static BOOL SlowDownStop(UWORD uwSlowDownStopCommand) ;
static void FaultsReset(void) ;
static void ExternalFaultReset(void) ;

static BOOL HmGetSwitchStatus(UBYTE ubSelector, UBYTE ubSoftSwitchStatus);

static BOOL LiveParametersCheck(void);

//***************************************************************************
// Supported mode of op and entry point

MODEOFOPS sModeOfOpList[]=
{
    {MOTCTRL_402_OM_PMC_TORQUE,         &motCtrl402OmProfileTorqueSetup},
    {MOTCTRL_402_OM_PROFILEVELOCITY,    &motCtrl402OmProfileVelocitySetup},
    {MOTCTRL_402_OM_PROFILEPOSITION,    &motCtrl402OmProfilePositionSetup},
    {MOTCTRL_402_OM_HOMING,             &motCtrl402OmProfileHomingSetup},
    {MOTCTRL_402_OM_INTERPOLATED,       &motCtrl402OmProfileInterpolatedSetup},
    {MOTCTRL_402_OM_CYCLICSYNCPOSITION, &motCtrl402OmProfileCSPositionSetup},
    {MOTCTRL_402_OM_CYCLICSYNCVELOCITY, &motCtrl402OmProfileCSVelocitySetup},
    {MOTCTRL_402_OM_CYCLICSYNCTORQUE,   &motCtrl402OmProfileCSTorqueSetup},
};

//***************************************************************************
// Global Handlers

const COMCTRL_HANDLERS sMotCtrlHandlers=
{
    &EmcyStop,
    &ExternalFaultReset,
};

// profilo di funzionamento da eseguire
void (* pfSelectedProfile2Use)(void) ;

//***************************************************************************
// Init

BOOL MotCtrlInit(void)
{
	BOOL bRetVal = FALSE ;

    // init runtime structure 
    sMotCtrlRFlags.b.bEmcyDisabled = FALSE ;
    sMotCtrlRFlags.b.bSysStatBooting_1 = bSysStatBooting ;
    sMotCtrlRFlags.b.bStopMotorSetupDone = FALSE ;
    sMotCtrlRFlags.b.bPID_IntegralEnable = FALSE ;
    sMotCtrlRun.uwStsDevCntrlIdx = 0 ;	
    sMotCtrlRun.uwLocalFlags2Use &= LF_RESET ; // reset flag 'locali'
    sMotCtrlRun.uwForcedNextStsDevCntrl = 0 ;  // reset status forzato

	sMotCtrl_Out.uwStatusWord = sMotCtrl_Out.uwStatusWord & (~MOTCTRL_402_SF_HOMING_DONE) ;

    // init output structure
    sMotCtrl_Out.uwStatusWord  =  0 ;       //状态字清零
    sMotCtrl_OStatus.b.bApplicationEnabled = FALSE ;
    sMotCtrl_OStatus.b.bPID_IntegralEnable = FALSE ;
    sMotCtrl_OStatus.b.bPostnerEnable      = FALSE ;
    sMotCtrl_Out.ubModeOfOperationDisplay    = MOTCTRL_402_DEF_MODEOFOP ;
    _uint64_atomic_copy(&sMotCtrlRun.sqHomingUserOffset8kHz, &sMotCtrlParameters.sqHomingUserOffset); // crs: do not allow to change sqHomingUserOffset at 8kHz when NOT in homing mode
    _uint64_atomic_copy(&sMotCtrl_Out.sqUserOffset, &sMotCtrlRun.sqHomingUserOffset8kHz);
    sMotCtrlRFlags.b.bIgnoreHomingDoneBit = sMotCtrlParameters.flags.b.bIgnoreHomingDoneBit ; // crs: allow infinite homing procedure

    /* install background function */
    if(!TaskSched_AddBackgroundTask(&MotCtrlSlowTask))
        return FALSE ;  

    // init parameters
    if(!LiveParametersCheck())
        return FALSE;

    // init mode of op
    if(!SetupModeOfOperation(sMotCtrlParameters.ubModeOfOperation, FALSE, FALSE))
        return FALSE;

    /* install 8KHz function */ 
    return TaskSched_AddRTTask(&MotCtrl8KHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_RESETTING), 0) ;
}

//***************************************************************************
// slow task
static void MotCtrlSlowTask(void)
{
    UWORD uwNextStsDevCntrlIdx ;
    UWORD uwLocControlWord;
    BOOL bSkipUpdate = FALSE;
    BOOL bForceUpdate = FALSE;
    SQWRD sqLocOffset;

    // read controlword to be used atomically in the slow task
    uwLocControlWord = sMotCtrl_In.psMotCtrl_In->uwControlWord;

    // update controlword ONLY if it is allowed  and ONLY if the system is fully active;
    // if MOTCTRL_402_CF_ENABLEVOLTAGE is removed, its priority is the highest and it prevails all other the variables
    if (!(sMotCtrlRun.uwLocalFlags2Use & LF_DISABLECONTROLWORDINPUT) && 
        !bSysStatLockedByBootError && !bSysStatLockedByPlcReload && !bSysStatResetting && !bSysStatLockedByAscActivated)
        sMotCtrlRun.uwImgCntrlWord = uwLocControlWord ;
    else if ((sMotCtrlRun.uwLocalFlags2Use & LF_DISABLECONTROLWORDINPUT) && !(uwLocControlWord & MOTCTRL_402_CF_ENABLEVOLTAGE))
    {
        sMotCtrlRun.uwImgCntrlWord = uwLocControlWord ;
        fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DISABLECONTROLWORDINPUT) ; // re-enable controlword
    }

    // #######################################################################

    // force the status to be used
    if(sMotCtrlRun.uwLocalFlags2Use & LF_FORCESTATUS)
    {
        uwNextStsDevCntrlIdx = sMotCtrlRun.uwForcedNextStsDevCntrl ;
        bForceUpdate = TRUE;

        // clear variabile e flag
        sMotCtrlRun.uwForcedNextStsDevCntrl = 0 ;
        fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_FORCESTATUS) ;

        // riabilito controlword in ingresso (se fosse stata disabilitata)
        fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DISABLECONTROLWORDINPUT) ;
    }
    else
        uwNextStsDevCntrlIdx = sMotCtrlRun.uwStsDevCntrlIdx ;  // prepare next DeviceStatusWord (if it's not necessary to change)   

    // #######################################################################
    // ## Cambio al volo del  Modo di funzionamento                         ##
    // #######################################################################
    if (sMotCtrl_Out.ubModeOfOperationDisplay != sMotCtrlParameters.ubModeOfOperation)
        SetupModeOfOperation(sMotCtrlParameters.ubModeOfOperation, sMotCtrl_OStatus.b.bApplicationEnabled, FALSE) ;


    // #######################################################################
    // ###########################  gestione HALT  ###########################
    // #######################################################################       
    if( (uwNextStsDevCntrlIdx == LOCALSTATUS_OPERATIONENABLED) && !(sMotCtrlRun.uwLocalFlags2Use & LF_DOPWROFFSTOP) )
    {   // sono in OperationEnabled e NON sto facendo un PowerOffStop          
        if(sMotCtrlRun.uwImgCntrlWord & MOTCTRL_402_CF_HALT)
        {   // ricevuto comando di Halt  

            if (!(sMotCtrlRun.uwLocalFlags2Use & LF_DOHALT))
            {   // imposto tutto il necessario per fare l'Halt
                fast_atomic_set_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DOHALT) ; // flag: sto facendo un Halt
                motCtrl402QuickStopSetup(MOTCTRL_402_HALT, sMotCtrlRun.swOptHalt) ;
            }                
        }        
        else
        {
            if (sMotCtrlRun.uwLocalFlags2Use & LF_DOHALT)
            {   // un attimo fa ero in halt...adesso non piu'. devo ripristinare tutto                
                SetupModeOfOperation(sMotCtrl_Out.ubModeOfOperationDisplay, TRUE, TRUE) ;

                sMotCtrl_OStatus.b.bQuickStop = FALSE ; // disattivo la flag di quickstop del modulo 'posizionatore'
                fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, MOTCTRL_402_SF_TARGETREACHED); // azzero il bit nella statusword

                sMotCtrlRFlags.b.bStopMotorSetupDone = FALSE ; // mi preparo per il prossimo stop
                fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DOHALT) ; // flag: finito halt
            }
        }
    } 


    // #######################################################################
    // ########################## Macchina a  stati ##########################
    // #######################################################################
    switch(uwNextStsDevCntrlIdx)
    {   // -------------------------------------------------------------------
        case LOCALSTATUS_SWITCHONDISABLED:
                // for local consistency always keep power disabled here
            sMotCtrl_Out.sPowerStageCtrl.b.bPowerEnable = FALSE;
            sMotCtrl_OStatus.b.bQuickStop = FALSE ;
            if(!((~sMotCtrlRun.uwImgCntrlWord) & (MOTCTRL_402_CF_QUICKSTOP | MOTCTRL_402_CF_ENABLEVOLTAGE)))
                uwNextStsDevCntrlIdx = LOCALSTATUS_READYTOSWITCHON ;    // transizione 2
            break ;

        // -------------------------------------------------------------------
        case LOCALSTATUS_READYTOSWITCHON:

            if((~sMotCtrlRun.uwImgCntrlWord) & (MOTCTRL_402_CF_QUICKSTOP | MOTCTRL_402_CF_ENABLEVOLTAGE))
                uwNextStsDevCntrlIdx = LOCALSTATUS_SWITCHONDISABLED ;   // transizione 7
            else if(sMotCtrlRun.uwImgCntrlWord & MOTCTRL_402_CF_SWITCHON)    
                uwNextStsDevCntrlIdx = LOCALSTATUS_SWITCHEDON ;         // transizione 3

            break ;

        // -------------------------------------------------------------------
        case LOCALSTATUS_SWITCHEDON:

            if((~sMotCtrlRun.uwImgCntrlWord) & (MOTCTRL_402_CF_QUICKSTOP | MOTCTRL_402_CF_ENABLEVOLTAGE))
                uwNextStsDevCntrlIdx = LOCALSTATUS_SWITCHONDISABLED ;   // transizione 10
            else if(!(sMotCtrlRun.uwImgCntrlWord & MOTCTRL_402_CF_SWITCHON))
                uwNextStsDevCntrlIdx = LOCALSTATUS_READYTOSWITCHON ;    // transizione 6
            else if(sMotCtrlRun.uwImgCntrlWord & MOTCTRL_402_CF_ENABLEOPERATION)  
                uwNextStsDevCntrlIdx = LOCALSTATUS_OPERATIONENABLED ;   // transizione 4
            break ;

        // -------------------------------------------------------------------
        case LOCALSTATUS_OPERATIONENABLED:            
            if(!(sMotCtrlRun.uwImgCntrlWord & MOTCTRL_402_CF_ENABLEVOLTAGE))
                uwNextStsDevCntrlIdx = LOCALSTATUS_SWITCHONDISABLED ;   // transizione 9
            else if(!(sMotCtrlRun.uwImgCntrlWord & MOTCTRL_402_CF_QUICKSTOP))
                uwNextStsDevCntrlIdx = LOCALSTATUS_QUICKSTOPACTIVE ;    // transizione 11           
            else if(!(sMotCtrlRun.uwImgCntrlWord & MOTCTRL_402_CF_SWITCHON))
            {   // transizione 8: stop mode = 605bh.0: sono ancora in Operation Enabled (quindi tutto e' attivo)

                if (!sMotCtrlRFlags.b.bStopMotorSetupDone)
                {   // imposto la funzione da usare per stoppare il motore
                    motCtrl402QuickStopSetup(MOTCTRL_402_SHUTDOWN, sMotCtrlRun.swOptShutdown) ;
                }

                if (sMotCtrlRFlags.b.bMotorStopped)
                {
                    sMotCtrl_OStatus.b.bQuickStop = FALSE ;          // disattivo la flag di quickstop del modulo 'posizionatore'
                    sMotCtrlRFlags.b.bStopMotorSetupDone = FALSE ;  // mi preparo per il prossimo stop

                    uwNextStsDevCntrlIdx = LOCALSTATUS_READYTOSWITCHON ; // Motore dichiarato fermo: passo al prossimo stato e
                    fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DISABLECONTROLWORDINPUT) ; // accetto nuove controlword
                }
            }
            else if(!(sMotCtrlRun.uwImgCntrlWord & MOTCTRL_402_CF_ENABLEOPERATION))
            {   // transizione 5: stop mode = 605ch.0: sono ancora in Operation Enabled (quindi tutto e' attivo)

                if (!sMotCtrlRFlags.b.bStopMotorSetupDone)
                {   // imposto la funzione da usare per stoppare il motore
                    motCtrl402QuickStopSetup(MOTCTRL_402_DISABLEOP, sMotCtrlRun.swOptDisableOp) ;
                }  
                                        
                if (sMotCtrlRFlags.b.bMotorStopped)
                {
                    sMotCtrl_OStatus.b.bQuickStop = FALSE ;         // disattivo la flag di quickstop del modulo 'posizionatore'
                    sMotCtrlRFlags.b.bStopMotorSetupDone = FALSE ; // mi preparo per il prossimo stop
                     
                    uwNextStsDevCntrlIdx = LOCALSTATUS_SWITCHEDON ; // Motore dichiarato fermo: passo al prossimo stato e
                    fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DISABLECONTROLWORDINPUT) ; // accetto nuove controlword
                }
            }

            break ;

        // -------------------------------------------------------------------
        case LOCALSTATUS_QUICKSTOPACTIVE:
            // motore ancora in QUICKSTOPACTIVE: sono in rampa e sto aspettando che si fermi
            if (sMotCtrlRFlags.b.bMotorStopped)
            {   // Motore fermato: vedo se devo passare al prossimo stato

                fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DISABLECONTROLWORDINPUT) ; // motore dichiarato fermo: accetto nuove controlword
                
                if( (sMotCtrlRun.swOptQuickStop == MOTCTRL_402_OC_SLOWDOWNANDSTAY ) || 
                    (sMotCtrlRun.swOptQuickStop == MOTCTRL_402_OC_QUICKSTOPANDSTAY)   )
                {
                    if(!(sMotCtrlRun.uwImgCntrlWord & MOTCTRL_402_CF_ENABLEVOLTAGE))
                    {   
                        uwNextStsDevCntrlIdx = LOCALSTATUS_SWITCHONDISABLED ; // transizione 12
                        sMotCtrlRFlags.b.bStopMotorSetupDone = FALSE ; // mi preparo per il prossimo stop
                
                        sMotCtrl_Out.sPowerStageCtrl.b.bPowerEnable = FALSE;
                        sMotCtrl_OStatus.b.bQuickStop = FALSE ; // disattivo la flag di quickstop del modulo 'posizionatore'
                    }
                    else if(!((~sMotCtrlRun.uwImgCntrlWord) & (MOTCTRL_402_CF_SWITCHON | MOTCTRL_402_CF_QUICKSTOP | MOTCTRL_402_CF_ENABLEOPERATION)))
                    {   // transizione 16                
                        
                        // reimposto il modo di funzionamento 
                        SetupModeOfOperation(sMotCtrl_Out.ubModeOfOperationDisplay, TRUE, FALSE) ;
                         
                        // passo ad Operation Enabled
                        sMotCtrl_OStatus.b.bApplicationEnabled = TRUE ; // posizionatore ON, cntrloop ON
                        sMotCtrl_OStatus.b.bQuickStop = FALSE ;         // disattivo la flag di quickstop del modulo 'posizionatore'
                        uwNextStsDevCntrlIdx = LOCALSTATUS_OPERATIONENABLED ;
                        sMotCtrlRFlags.b.bStopMotorSetupDone = FALSE ; // mi preparo per il prossimo stop
                    }
                    else
                    {   // per stare in quickstop com asse fermo a velocit?zero, devo tenere abilitati i riferimenti all'fpga
                        sMotCtrl_OStatus.b.bQuickStop = TRUE ; // tengo attivo la flag di quickstop del modulo 'posizionatore'
                    }
                }
                else
                {
                    uwNextStsDevCntrlIdx = LOCALSTATUS_SWITCHONDISABLED ; // forza transizione 12
                    sMotCtrl_OStatus.b.bQuickStop = FALSE ;             // disattivo la flag di quickstop
                    sMotCtrlRFlags.b.bStopMotorSetupDone = FALSE ;     // mi preparo per il prossimo stop
                }
            }
                // se nel frattempo ho ricevuto disable voltage allora forzo immediatamente la transizione 12
            else if (!(sMotCtrlRun.uwImgCntrlWord & MOTCTRL_402_CF_ENABLEVOLTAGE))
            {
                uwNextStsDevCntrlIdx = LOCALSTATUS_SWITCHONDISABLED ; // forza transizione 12
                sMotCtrl_OStatus.b.bQuickStop = FALSE ;             // disattivo la flag di quickstop
                sMotCtrlRFlags.b.bStopMotorSetupDone = FALSE ;     // mi preparo per il prossimo stop
            }
            break ;

        // -------------------------------------------------------------------
        case LOCALSTATUS_FAULTREACTIONACTIVE:
            uwNextStsDevCntrlIdx = LOCALSTATUS_FAULTREACTIONACTIVE;
            if (sMotCtrlRFlags.b.bEmcyDisabled)
            {
                    // transizione 14
                uwNextStsDevCntrlIdx = LOCALSTATUS_FAULTHIGH ;
                fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DISABLECONTROLWORDINPUT) ;
            }
            else if(!sMotCtrlRFlags.b.bFaultReactActivated)
            {
                    // activate fault reaction
                motCtrl402QuickStopSetup(MOTCTRL_402_FAULTREACTION, sMotCtrlRun.swOptFaultReaction) ;
                
                sMotCtrlRFlags.b.bFaultReactActivated = TRUE;
            }                             

                // if stopped
            if(sMotCtrlRFlags.b.bMotorStopped)  
            {
                sMotCtrl_Out.sPowerStageCtrl.b.bPowerEnable = FALSE;

                    // transizione 14
                uwNextStsDevCntrlIdx = LOCALSTATUS_FAULTHIGH ;
                fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DISABLECONTROLWORDINPUT) ;
            }

            break ;

        // -------------------------------------------------------------------
        case LOCALSTATUS_FAULTHIGH:    // transizione per triggerare FAULTRESET 0 -> 1
            if(sMotCtrlRFlags.b.bFaultReactActivated)
            {
                sMotCtrl_OStatus.b.bQuickStop = FALSE ;
                sMotCtrlRFlags.b.bStopMotorSetupDone = FALSE ;
                sMotCtrlRFlags.b.bFaultReactActivated = FALSE ;
            }
            if(!(sMotCtrlRun.uwImgCntrlWord & MOTCTRL_402_CF_FAULTRESET))
                uwNextStsDevCntrlIdx = LOCALSTATUS_FAULT ;
            else
                uwNextStsDevCntrlIdx = LOCALSTATUS_FAULTHIGH;
            break ;

        // -------------------------------------------------------------------
        case LOCALSTATUS_FAULT:
            if(sMotCtrlRun.uwImgCntrlWord & MOTCTRL_402_CF_FAULTRESET)
            {    // transizione 15
                FaultsReset() ;
                uwNextStsDevCntrlIdx = LOCALSTATUS_SWITCHONDISABLED ;
            }
            break ;
    }

    // #######################################################################
    // ########################  S T A T U S W O R D  ########################
    // ####################################################################### 
    if(uwNextStsDevCntrlIdx != sMotCtrlRun.uwStsDevCntrlIdx)
    {   // lo stato e' cambiato, prendiamo delle decisioni (anche affidandoci al caso...)   
 
        // ===================================================================
        // ============== attivazione/disattivazione quickstop ===============
        // ===================================================================        
        // blocco la control word in input non appena ricevo il comando di quickstop
        // ed imposto tutto cio' che mi serve
        if(uwNextStsDevCntrlIdx == LOCALSTATUS_QUICKSTOPACTIVE)
            motCtrl402QuickStopSetup(MOTCTRL_402_QUICKSTOP, sMotCtrlRun.swOptQuickStop) ;

        // ===================================================================
        // ============= attivazione/disattivazione applicazione =============
        // ===================================================================
        if((sMotCtrl_Out.uwStatusWord^uwLocalStatusFlags[uwNextStsDevCntrlIdx]) & MOTCTRL_402_SF_OPERATIONENABLED)
        {
            if(uwLocalStatusFlags[uwNextStsDevCntrlIdx] & MOTCTRL_402_SF_OPERATIONENABLED)
            {
                sMotCtrl_Out.sPowerStageCtrl.b.bPowerEnable = TRUE;    // switch ON, NO reference to FPGA  

                    // se power stage non pronto attendo, disabilitando nel frattempo l'aggiornamento della controlword
                if(sMotCtrl_In.psPowerStageStatus->b.bFullyActive && \
                    ((sMotCtrl_In.psFbk2CntrLoop->ubStatus&(ENCMGR_RELATIVE_VALID | ENCMGR_ELE_ANGLE_VALID))==(ENCMGR_RELATIVE_VALID | ENCMGR_ELE_ANGLE_VALID) || \
                     sMotCtrlRFlags.b.bIgnoreEncStatus) ) 
                {
                    fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DISABLECONTROLWORDINPUT) ;
                    SetupModeOfOperation(sMotCtrl_Out.ubModeOfOperationDisplay, TRUE, FALSE) ;
                    sMotCtrl_OStatus.b.bApplicationEnabled = TRUE ; // posizionatore ON, cntrloop ON
                }
                else
                {
                    fast_atomic_set_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DISABLECONTROLWORDINPUT) ;
                    bSkipUpdate = TRUE;
                }
            }
            else
            {  
                sMotCtrl_Out.sPowerStageCtrl.b.bPowerEnable = FALSE;   // switch OFF
                sMotCtrl_OStatus.b.bApplicationEnabled = FALSE ; // tengo disabilitata l'applicazione, NO posizionatore, NO cntrloop
            }
        }
        // ===================================================================
        // aggiorna status word, stato device control se non ho control word bloccata oppure se forzata
        if(!bSkipUpdate || bForceUpdate)
        {
#ifdef _INFINEON_
        	DISABLE_IRQ();
#endif
        	sMotCtrl_Out.uwStatusWord = sMotCtrl_Out.uwStatusWord & (~MOTCTRL_402_SMASK_STATEMACHINE) | uwLocalStatusFlags[uwNextStsDevCntrlIdx] ;
            sMotCtrlRun.uwStsDevCntrlIdx = uwNextStsDevCntrlIdx ;
#ifdef _INFINEON_
            RESTORE_IRQ();
#endif
        }
    }
    // #######################################################################      

    // Update Voltage Enable flag
    if(sMotCtrl_In.psPowerStageStatus->b.bVoltageEnabled)
    {
        fast_atomic_set_bits(sMotCtrl_Out.uwStatusWord, MOTCTRL_402_SF_VOLTAGENABLED);
    }
    else
    {
        fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, MOTCTRL_402_SF_VOLTAGENABLED);
    }

    // Update Warning flag
    if(ulSystemWarnings)       
    {
        fast_atomic_set_bits(sMotCtrl_Out.uwStatusWord, MOTCTRL_402_SF_WARNING);
    }
    else
    {
        fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, MOTCTRL_402_SF_WARNING);
    }

    // #######################################################################      
 
    // Update position overall offset

//    _uint64_sub((UQWRD *)&sqLocOffset, (UQWRD *)&sMotCtrlParameters.sqHomingUserOffset, (UQWRD *)&sMotCtrlRun.sqHomingCalcOffset);
    _uint64_sub((UQWRD *)&sqLocOffset, (UQWRD *)&sMotCtrlRun.sqHomingUserOffset8kHz, (UQWRD *)&sMotCtrlRun.sqHomingCalcOffset);
    _sint64_atomic_copy(&sMotCtrl_Out.sqUserOffset, &sqLocOffset); 

    // #######################################################################      

    // parameters checking
    LiveParametersCheck();
}

//***************************************************************************
// Fast task
static BOOL MotCtrl8KHz(void)
{
    // #######################################################################
    // ############################  System Boot  ############################
    // #######################################################################
    /* Start State machine after booting change status on fall edge */
    if ((sMotCtrlRFlags.b.bSysStatBooting_1 == TRUE) && (bSysStatBooting == FALSE))
    {   // default: switch on disabled
        sMotCtrlRun.uwStsDevCntrlIdx = LOCALSTATUS_SWITCHONDISABLED ;
        sMotCtrl_Out.uwStatusWord = MOTCTRL_402_SF_SWITCHONDISABLED ;
    }
    sMotCtrlRFlags.b.bSysStatBooting_1 = bSysStatBooting ; 

    // #######################################################################
    // ###  Alarm reset request                                            ###
    // #######################################################################   

    bSysStatAlarmsReset=sMotCtrlRFlags.b.bAlarmClearRequest;
    sMotCtrlRFlags.b.bAlarmClearRequest=FALSE;

    if(bSysStatAlarmsReset)
        sMotCtrlRFlags.b.bEmcyDisabled=FALSE;

    // #######################################################################
    // ###  Application enable  (forse la potenza ?pienamente abilitata)  ###
    // #######################################################################   
    if(sMotCtrl_OStatus.b.bApplicationEnabled && sMotCtrl_In.psPowerStageStatus->b.bFullyActive && \
        ((sMotCtrl_In.psFbk2CntrLoop->ubStatus&(ENCMGR_RELATIVE_VALID | ENCMGR_ELE_ANGLE_VALID))==(ENCMGR_RELATIVE_VALID | ENCMGR_ELE_ANGLE_VALID) || \
         sMotCtrlRFlags.b.bIgnoreEncStatus) )
    {   // PowerStage fully enabled = Application enabled = execute the Selected Profile
        sMotCtrl_Out.sPowerStageCtrl.b.bReferenceEnable = TRUE; // abilito i riferimenti
        sMotCtrl_OStatus.b.bPID_IntegralEnable = sMotCtrlRFlags.b.bPID_IntegralEnable ;         
        sPo_UsrPostnerIn.flags.b.bPostnerEnable   = sMotCtrl_OStatus.b.bPostnerEnable ; 
        (*pfSelectedProfile2Use)() ;
        sPo_UsrPostnerIn.flags.b.bNewTarget       = sMotCtrl_OStatus.b.bNewTarget ;
    }
    else
    {   // Application disabled (tengo tutto fermo)
        sMotCtrl_Out.sPowerStageCtrl.b.bReferenceEnable = FALSE; // disabilito i riferimenti
        sMotCtrl_OStatus.b.bPID_IntegralEnable = FALSE ; // tengo fermo l'integrale dell'anello di controllo
        sPo_UsrPostnerIn.flags.b.bNewTarget       = FALSE ;
        sPo_UsrPostnerIn.flags.b.bPostnerEnable   = FALSE ; // modulo posizionatore 'spento'
        sMotCtrlRFlags.b.bMotorStopped        = TRUE  ; // motore stoppato
        sMotCtrlRun.uwLocalFlags2Use         &= ~LF_DOPWROFFSTOP ;
    }

    // #################  imposto le flag del posizionatore  #################
    sPo_UsrPostnerIn.flags.b.bQuickStop = sMotCtrl_OStatus.b.bQuickStop ;
    sPo_UsrPostnerIn.flags.b.bVelocity  = sMotCtrl_OStatus.b.bVelocity  ;
    sPo_UsrPostnerIn.flags.b.bPosition  = sMotCtrl_OStatus.b.bPosition  ;
    sPo_UsrPostnerIn.flags.b.bInterpolate  = sMotCtrl_OStatus.b.bInterpolate  ;
    sPo_UsrPostnerIn.flags.b.bUseLocalAccSpd  = sMotCtrl_OStatus.b.bUseLocalAccSpd  ;
    sPo_UsrPostnerIn.flags.b.bDirectSpeed  = sMotCtrl_OStatus.b.bDirectSpeed  ;
    return TRUE ;
}


//***************************************************************************
// emergency stop
static void EmcyStop(UWORD uwOption)
{
        // if already fatal emcy, then exit
    if(sMotCtrlRFlags.b.bEmcyDisabled)
        return;

        // if fatal or power disabled
    if(uwOption == COMCTRL_ES_FATAL || !(sMotCtrl_Out.uwStatusWord&MOTCTRL_402_SF_OPERATIONENABLED))
    {
            // immediate power disable
        sMotCtrlRFlags.b.bEmcyDisabled = TRUE ;
        sMotCtrl_Out.sPowerStageCtrl.b.bPowerEnable = FALSE;

            // and forced transition 14
#ifdef _INFINEON_
        _atomic_( 0 ); // defined in INTRINS.H C166
        sMotCtrlRun.uwForcedNextStsDevCntrl = LOCALSTATUS_FAULTHIGH ;
        sMotCtrlRun.uwLocalFlags2Use|=LF_FORCESTATUS;
        _endatomic_();
#else
        sMotCtrlRun.uwForcedNextStsDevCntrl = LOCALSTATUS_FAULTHIGH ;
        sMotCtrlRun.uwLocalFlags2Use|=LF_FORCESTATUS;
#endif // _infineon_
        return;
    }

        // if non-fatal and not yet in fault reaction
    if(uwOption == COMCTRL_ES_NONFATAL && !sMotCtrlRFlags.b.bFaultReaction)
    {
        sMotCtrlRFlags.b.bFaultReaction = TRUE ;

            // force transition 13
#ifdef _INFINEON_
        _atomic_( 0 );
        sMotCtrlRun.uwForcedNextStsDevCntrl = LOCALSTATUS_FAULTREACTIONACTIVE ;
        sMotCtrlRun.uwLocalFlags2Use|=LF_FORCESTATUS;
        _endatomic_();
#else
        sMotCtrlRun.uwForcedNextStsDevCntrl = LOCALSTATUS_FAULTREACTIONACTIVE ;
        sMotCtrlRun.uwLocalFlags2Use|=LF_FORCESTATUS;
#endif
    }
}


//***************************************************************************
// Impostazione del modo di funzionamento
static BOOL SetupModeOfOperation(UBYTE ubNewModeOfOperation, BOOL bActivate, BOOL bRstFromHalt)
{
    UBYTE ubCnt;

    if(sMotCtrl_Out.ubModeOfOperationDisplay == ubNewModeOfOperation && !bActivate)
    {
        ParChk_ResetValueError(PARCC_MOTCTRL_MODEOFOPERATION);
        return TRUE;
    }

    for(ubCnt=0;ubCnt<sizeof(sModeOfOpList)/sizeof(MODEOFOPS);ubCnt++)
        if(sModeOfOpList[ubCnt].ubModeOfOp==ubNewModeOfOperation)
        {
            if(bActivate)
                (*sModeOfOpList[ubCnt].pfSetupEntryPoint)(bRstFromHalt);

            sMotCtrl_Out.ubModeOfOperationDisplay=ubNewModeOfOperation;

            ParChk_ResetValueError(PARCC_MOTCTRL_MODEOFOPERATION);
            return TRUE;
        }

    ParChk_SignalValueError(PARCC_MOTCTRL_MODEOFOPERATION);
    return FALSE;
}

//***************************************************************************
static UBYTE motCtrl402OmProfilePositionSetup(BOOL bRstFromHalt)    
{
    ULONG ulAddressFunction2Use ;

    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ; 
    
    DrvTskCtrl_Config(DRVTSKCTRL_OP_POSITIONER) ;
    sMotCtrl_OStatus.b.bPosition = TRUE  ; // informo il posizionatore che voglio attivare il posizionatore
    sMotCtrl_OStatus.b.bVelocity = !sMotCtrl_OStatus.b.bPosition ;//~sMotCtrl_OStatus.b.bPosition ;
    sMotCtrl_OStatus.b.bInterpolate = FALSE ;
    sMotCtrl_OStatus.b.bUseLocalAccSpd = FALSE ;
    sMotCtrl_OStatus.b.bDirectSpeed = FALSE;
    sMotCtrl_OStatus.b.bPostnerEnable = TRUE ; // modulo posizionatore 'acceso'
    sMotCtrl_OStatus.b.bNewTarget = FALSE ;
//    sMotCtrl_OStatus.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo        
    sMotCtrlRFlags.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo
   

    // pulisco le flag per la statusword: serve?
    sMotCtrlRFlags.b.bTgtReachedTimerSet = FALSE ;
    sMotCtrlRFlags.b.bFollowingErrorTimerSet = FALSE ; 
    
    if(!bRstFromHalt)
    {
        // imposto vecchia posizione con attuale
        _sint64_atomic_copy(&sMotCtrlRun.sqOldTargetPostn, &sMotCtrl_In.psFbk2CntrLoop->sEncData.sqPostn);
        sMotCtrlRFlags.b.bIsPositioning = FALSE ;
    }
    else
        sMotCtrl_OStatus.b.bNewTarget = sMotCtrlRFlags.b.bIsPositioning ;

	// avoid to immediate start position when entering in position mode
    if (sMotCtrl_In.psMotCtrl_In->uwControlWord & MOTCTRL_402_CF_PP_NEWSETPOINT)
        sMotCtrlRun.uwLocalFlags2Use |=  LF_NEWSETPOINTSET ; // bit NewSetPoint already set: avoid rise edge for immediate position start
	else
	    sMotCtrlRun.uwLocalFlags2Use &= ~LF_NEWSETPOINTSET ;

    // imposto il puntatore alla funzione da usare (lo faccio atomico per evitare casini)
    ulAddressFunction2Use = (ULONG)(&motCtrl402OmProfilePosition8KHz) ;
    atomic_move(&pfSelectedProfile2Use, &ulAddressFunction2Use, sizeof(ULONG)) ;

    // pulisco la parte addizionale della statusword (bit 15->8) e la target reached
    fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, RESETMASK_HIGHSTATUSWORD);

    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;

    return MOTCTRL_402_OM_PROFILEPOSITION ;
}

//***************************************************************************
static UBYTE motCtrl402OmProfileInterpolatedSetup(void)
{
    ULONG ulAddressFunction2Use ;

    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ; 
    
    DrvTskCtrl_Config(DRVTSKCTRL_OP_POSITIONER) ;

    sPo_UsrPostnerIn.slLocalAcc = 0;

    sMotCtrl_OStatus.b.bPosition = FALSE ;
    sMotCtrl_OStatus.b.bVelocity = FALSE ;
    sMotCtrl_OStatus.b.bInterpolate = FALSE ;
    sMotCtrl_OStatus.b.bUseLocalAccSpd = TRUE ;
    sMotCtrl_OStatus.b.bDirectSpeed = TRUE;
    sMotCtrl_OStatus.b.bPostnerEnable = TRUE ; // modulo posizionatore 'acceso'
//    sMotCtrl_OStatus.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo        
    sMotCtrlRFlags.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo
    sMotCtrlRFlags.b.bInterpolationActive = FALSE ;
   
    // pulisco la parte addizionale della statusword (bit 15->8) e la target reached
    fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, RESETMASK_HIGHSTATUSWORD);

    // pulisco le flag per la statusword: serve?
    sMotCtrlRFlags.b.bFollowingErrorTimerSet = FALSE ; 

    // tengo stoppata l'interpolazione
    sMotCtrlRun.swIPCycleCnt = -sMotCtrlRun.swIPNumCycle;
    
    // imposto il puntatore alla funzione da usare (lo faccio atomico per evitare casini)
    ulAddressFunction2Use = (ULONG)(&motCtrl402OmProfileInterpolated8KHz) ;
    atomic_move(&pfSelectedProfile2Use, &ulAddressFunction2Use, sizeof(ULONG)) ;

    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;

    return MOTCTRL_402_OM_INTERPOLATED ;
}

//***************************************************************************
static UBYTE motCtrl402OmProfileVelocitySetup(void)
{
    ULONG ulAddressFunction2Use ;

    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;
    
    DrvTskCtrl_Config(DRVTSKCTRL_OP_POSITIONER) ;
    sMotCtrl_OStatus.b.bVelocity = TRUE  ; // informo il posizionatore che voglio attivare solo il generatore di rampe
    sMotCtrl_OStatus.b.bPosition = !sMotCtrl_OStatus.b.bVelocity ;//~sMotCtrl_OStatus.b.bVelocity ;
    sMotCtrl_OStatus.b.bInterpolate = FALSE ;
    sMotCtrl_OStatus.b.bUseLocalAccSpd = FALSE ;
    sMotCtrl_OStatus.b.bDirectSpeed = FALSE;
    sMotCtrl_OStatus.b.bPostnerEnable = TRUE ; // modulo posizionatore 'acceso'
//    sMotCtrl_OStatus.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo    
    sMotCtrlRFlags.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo
   
    // pulisco la parte addizionale della statusword (bit 15->8) e la target reached
    fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, RESETMASK_HIGHSTATUSWORD);

    // resetto i timer
    sMotCtrlRFlags.b.bTgtReachedTimerSet = FALSE ;    
    sMotCtrlRFlags.b.bInSpeedTimerSet = FALSE ;  

    // imposto il puntatore alla funzione da usare (lo faccio atomico per evitare casini)
    ulAddressFunction2Use = (ULONG)(&motCtrl402OmProfileVelocity8KHz) ;
    atomic_move(&pfSelectedProfile2Use, &ulAddressFunction2Use, sizeof(ULONG)) ;

    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;

    return MOTCTRL_402_OM_PROFILEVELOCITY ;
}

//***************************************************************************
static UBYTE motCtrl402OmProfileHomingSetup(void)
{
    ULONG ulAddressFunction2Use ;

    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;
    
    DrvTskCtrl_Config(DRVTSKCTRL_OP_POSITIONER) ;

    sPo_UsrPostnerIn.slLocalAcc = sMotCtrlRun.slHomingAcceleration;
    sPo_UsrPostnerIn.slLocalDec = sMotCtrlRun.slHomingAcceleration;
    sPo_UsrPostnerIn.slLocalSpeed = 0l;

    sMotCtrl_OStatus.b.bVelocity = TRUE  ; // informo il posizionatore che voglio attivare solo il generatore di rampe
    sMotCtrl_OStatus.b.bPosition = !sMotCtrl_OStatus.b.bVelocity ;//~sMotCtrl_OStatus.b.bVelocity ;
    sMotCtrl_OStatus.b.bInterpolate = FALSE ;
    sMotCtrl_OStatus.b.bUseLocalAccSpd = TRUE ;
    sMotCtrl_OStatus.b.bDirectSpeed = FALSE;
    sMotCtrl_OStatus.b.bPostnerEnable = TRUE ; // modulo posizionatore 'acceso'
//    sMotCtrl_OStatus.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo    
    sMotCtrlRFlags.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo

    sMotCtrlRFlags.b.bHomeAttained = FALSE;
    sMotCtrlRun.puwHomingRun = NULL;  // later initialization
    sMotCtrlRun.puwHomingPCmd = NULL; // later initialization
    sMotCtrlRun.ubHomingPrevHomeSwitch = HmGetSwitchStatus(sMotCtrlRun.ubHomingSSrcHomeSwitch, sMotCtrl_In.psMotCtrl_In->sbHomingSoftHomeSwitch);
   
    sMotCtrlRun.uwLocalFlags2Use &= ~LF_HM_OP_STARTED ;

    if(sMotCtrlRFlags.b.bIgnoreHomingDoneBit)
    {   // reset also homing done flag
        fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, RESETMASK_HIGHSTATUSWORD_IGNOREHOMINGDONE);
    }
    else
    {   // pulisco la parte addizionale della statusword (bit 15->8) e la target reached
        fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, RESETMASK_HIGHSTATUSWORD);
    }

    // imposto il puntatore alla funzione da usare (lo faccio atomico per evitare casini)
    ulAddressFunction2Use = (ULONG)(&motCtrl402OmProfileHoming8KHz) ;
    atomic_move(&pfSelectedProfile2Use, &ulAddressFunction2Use, sizeof(ULONG)) ;

    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;

    return MOTCTRL_402_OM_HOMING ;
}

//***************************************************************************
static UBYTE motCtrl402OmProfileTorqueSetup(void)
{
    ULONG ulAddressFunction2Use ;

    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;
    
    DrvTskCtrl_Config(DRVTSKCTRL_OP_TORQUE) ;
    sMotCtrl_OStatus.b.bPostnerEnable = FALSE ; // modulo posizionatore 'spento'
//    sMotCtrl_OStatus.b.bPID_IntegralEnable = FALSE ; // tengo fermo l'integrale dell'anello di controllo
    sMotCtrlRFlags.b.bPID_IntegralEnable = FALSE ; // tengo fermo l'integrale dell'anello di controllo
    
    // pulisco la parte addizionale della statusword (bit 15->8) e la target reached
    fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, RESETMASK_HIGHSTATUSWORD);

    // resetto i timer? serve?
    sMotCtrlRFlags.b.bInSpeedTimerSet = FALSE ;  
    sMotCtrlRFlags.b.bTgtReachedTimerSet = FALSE ;
    sMotCtrlRFlags.b.bFollowingErrorTimerSet = FALSE ; 

    // imposto il puntatore alla funzione da usare (lo faccio atomico per evitare casini)
    ulAddressFunction2Use = (ULONG)(&motCtrl402OmProfileTorque8KHz) ;
    atomic_move(&pfSelectedProfile2Use, &ulAddressFunction2Use, sizeof(ULONG)) ;
    
    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;

    return MOTCTRL_402_OM_PMC_TORQUE ;
}

//***************************************************************************
static UBYTE motCtrl402OmProfileCSPositionSetup(void)
{
    ULONG ulAddressFunction2Use ;

    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ; 
    
    DrvTskCtrl_Config(DRVTSKCTRL_OP_POSITIONER) ;
    sMotCtrl_OStatus.b.bPosition = FALSE ;
    sMotCtrl_OStatus.b.bVelocity = FALSE ;
    sMotCtrl_OStatus.b.bInterpolate = FALSE ;
    sMotCtrl_OStatus.b.bUseLocalAccSpd = TRUE ;
    sMotCtrl_OStatus.b.bDirectSpeed = TRUE;
    sMotCtrl_OStatus.b.bPostnerEnable = TRUE ; // modulo posizionatore 'acceso'
//    sMotCtrl_OStatus.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo        
    sMotCtrlRFlags.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo
    sMotCtrlRFlags.b.bInterpolationActive = FALSE ;
   
    // pulisco la parte addizionale della statusword (bit 15->8) e la target reached
    fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, RESETMASK_HIGHSTATUSWORD);

    // pulisco le flag per la statusword: serve?
    sMotCtrlRFlags.b.bFollowingErrorTimerSet = FALSE ; 

    // tengo stoppata l'interpolazione
    sMotCtrlRun.swIPCycleCnt = -sMotCtrlRun.swIPNumCycle;
    sPo_UsrPostnerIn.slLocalAcc = 0l;
    
    // imposto il puntatore alla funzione da usare (lo faccio atomico per evitare casini)
    ulAddressFunction2Use = (ULONG)(&motCtrl402OmProfileCSPosition8KHz) ;
    atomic_move(&pfSelectedProfile2Use, &ulAddressFunction2Use, sizeof(ULONG)) ;

    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;

    return MOTCTRL_402_OM_CYCLICSYNCPOSITION ;
}

//***************************************************************************
static UBYTE motCtrl402OmProfileCSVelocitySetup(void)
{
    ULONG ulAddressFunction2Use ;

    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ; 
    
    DrvTskCtrl_Config(DRVTSKCTRL_OP_POSITIONER) ;
    sMotCtrl_OStatus.b.bPosition = FALSE ;
    sMotCtrl_OStatus.b.bVelocity = FALSE ;
    sMotCtrl_OStatus.b.bInterpolate = FALSE ;
    sMotCtrl_OStatus.b.bUseLocalAccSpd = TRUE ;
    sMotCtrl_OStatus.b.bDirectSpeed = TRUE;
    sMotCtrl_OStatus.b.bPostnerEnable = TRUE ; // modulo posizionatore 'acceso'
//    sMotCtrl_OStatus.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo        
    sMotCtrlRFlags.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo
    sMotCtrlRFlags.b.bInterpolationActive = FALSE ;
   
    // pulisco la parte addizionale della statusword (bit 15->8) e la target reached
    fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, RESETMASK_HIGHSTATUSWORD);

    // pulisco le flag per la statusword: serve?
    sMotCtrlRFlags.b.bFollowingErrorTimerSet = FALSE ; 

    // tengo stoppata l'interpolazione
    sMotCtrlRun.swIPCycleCnt = -sMotCtrlRun.swIPNumCycle;
    sPo_UsrPostnerIn.slLocalSpeed = 0l;
    
    // imposto il puntatore alla funzione da usare (lo faccio atomico per evitare casini)
    ulAddressFunction2Use = (ULONG)(&motCtrl402OmProfileCSVelocity8KHz) ;
    atomic_move(&pfSelectedProfile2Use, &ulAddressFunction2Use, sizeof(ULONG)) ;

    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;

    return MOTCTRL_402_OM_CYCLICSYNCVELOCITY ;
}

//***************************************************************************
static UBYTE motCtrl402OmProfileCSTorqueSetup(void)
{
    ULONG ulAddressFunction2Use ;

    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;
    
    DrvTskCtrl_Config(DRVTSKCTRL_OP_IRTQ) ;
    sMotCtrl_OStatus.b.bPostnerEnable = FALSE ; // modulo posizionatore 'spento'
//    sMotCtrl_OStatus.b.bPID_IntegralEnable = FALSE ; // tengo fermo l'integrale dell'anello di controllo
    sMotCtrlRFlags.b.bPID_IntegralEnable = FALSE ; // tengo fermo l'integrale dell'anello di controllo
    
    // pulisco la parte addizionale della statusword (bit 15->8) e la target reached
    fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, RESETMASK_HIGHSTATUSWORD);

    // resetto i timer? serve?
    sMotCtrlRFlags.b.bInSpeedTimerSet = FALSE ;  
    sMotCtrlRFlags.b.bTgtReachedTimerSet = FALSE ;
    sMotCtrlRFlags.b.bFollowingErrorTimerSet = FALSE ; 

    // imposto il puntatore alla funzione da usare (lo faccio atomico per evitare casini)
    ulAddressFunction2Use = (ULONG)(&motCtrl402OmProfileCSTorque8KHz) ;
    atomic_move(&pfSelectedProfile2Use, &ulAddressFunction2Use, sizeof(ULONG)) ;
    
    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;

    return MOTCTRL_402_OM_CYCLICSYNCTORQUE ;
}

//***************************************************************************
// Statusword bit in profile position 
static void motCtrl402OmProfilePosition8KHz(void)
{
    SWORD   swTgtPosWindowMax, swTgtPosWindowMin ;
    SWORD   swFollowingErrMax, swFollowingErrMin ;
    SQWRD   sqDeltaPos64 ;
    UWORD   uwLocCntrlWord = sMotCtrl_In.psMotCtrl_In->uwControlWord;

    // #######################################################################
    // ##################### check if 'new setpoint set' #####################
    // #######################################################################
    if ((uwLocCntrlWord & MOTCTRL_402_CF_PP_NEWSETPOINT) && !(sMotCtrlRun.uwLocalFlags2Use & LF_NEWSETPOINTSET))
    {   // bit NewSetPoint 0->1 : mi hanno detto di attivare il new set point
        sMotCtrlRun.uwLocalFlags2Use |=  LF_NEWSETPOINTSET ;

    //if (!(uwLocCntrlWord & MOTCTRL_402_CF_PP_NEWSETPOINT) && (sMotCtrlRun.uwLocalFlags2Use & LF_NEWSETPOINTSET))
    //        sMotCtrlRun.uwLocalFlags2Use &= ~LF_NEWSETPOINTSET ; // bit NewSetPoint 1->0

    //if (sMotCtrlRun.uwLocalFlags2Use & LF_NEWSETPOINTSET)
    //{   // bit NewSetPoint 0->1 : mi hanno detto di attivare il new set point
        if (!(sMotCtrl_Out.uwStatusWord & MOTCTRL_402_SF_PP_SETPOINTACK))          
        {   // bit SetPointAck=0: non mi sono ancora preso in carico la posizione
            if ((sMotCtrl_Out.uwStatusWord & MOTCTRL_402_SF_TARGETREACHED) || \
                (sMotCtrl_In.psPoOut->flags.b.bTgtPosReached) || \
                (uwLocCntrlWord & MOTCTRL_402_CF_PP_IMMCHANGE))
            {   // posizionamento precedente terminato: posso eseguirne un'altro
                // oppure posizionamento immediato: eseguo immediatamente
    
                // controllo se posizionamento e' relativo od assoluto
                if (uwLocCntrlWord & MOTCTRL_402_CF_PP_RELATIVE)
                    _uint64_add((UQWRD *)&sPo_UsrPostnerIn.sqTargetPostn, (UQWRD *)&sMotCtrlRun.sqOldTargetPostn, (UQWRD *)&sMotCtrl_In.psMotCtrl_In->sqTargetPostn) ; 
                else
                {
                    if(sMotCtrl_In.psFbk2CntrLoop->ubStatus & ENCMGR_ABSMECHTURN_VALID)
                        _uint64_sub((UQWRD *)&sPo_UsrPostnerIn.sqTargetPostn, (UQWRD *)&sMotCtrl_In.psMotCtrl_In->sqTargetPostn, (UQWRD *)&sMotCtrl_In.psFbk2CntrLoop->sqMechAbsPosOffset);
                    else
                        _sint64_atomic_copy(&sPo_UsrPostnerIn.sqTargetPostn, &sMotCtrl_In.psMotCtrl_In->sqTargetPostn) ;

                    _uint64_sub((UQWRD *)&sPo_UsrPostnerIn.sqTargetPostn, (UQWRD *)&sPo_UsrPostnerIn.sqTargetPostn, (UQWRD *)&sMotCtrl_Out.sqUserOffset);
                }
    
                // mi preparo per un eventuale successivo posizionamento relativo
                _sint64_atomic_copy(&sMotCtrlRun.sqOldTargetPostn, &sPo_UsrPostnerIn.sqTargetPostn) ;
    
                sMotCtrl_Out.uwStatusWord |= MOTCTRL_402_SF_PP_SETPOINTACK ; // setto il bit nella statusword
                sMotCtrl_OStatus.b.bNewTarget = TRUE ;  // faccio partire il posizionatore..
                sMotCtrlRFlags.b.bIsPositioning = TRUE ;
            }
            else
                sMotCtrl_OStatus.b.bNewTarget = FALSE ; // sto ancora posizionando: non disturbo il posizionatore ed aspetto
        }
        else
        {   // bit SetPointAck=1: mi sono preso in carico la posizione..presumo di star eseguendo un posizionamento
            if (!(sMotCtrl_Out.uwStatusWord & MOTCTRL_402_SF_TARGETREACHED))                
                sMotCtrl_OStatus.b.bNewTarget = FALSE ; // sto ancora posizionando: non disturbo il posizionatore ed aspetto
        }
    }
    else if (!(uwLocCntrlWord & MOTCTRL_402_CF_PP_NEWSETPOINT) && (sMotCtrlRun.uwLocalFlags2Use & LF_NEWSETPOINTSET))
    {   //bit NewSetPoint 1->0 :hanno disattivato il new set point
        sMotCtrlRun.uwLocalFlags2Use &= ~LF_NEWSETPOINTSET ;
        sMotCtrl_Out.uwStatusWord &= ~MOTCTRL_402_SF_PP_SETPOINTACK ; // butto a zero il bit di acknowledge
    }


    // #######################################################################
    // ############# check if target position is in 'the window' #############
    // #######################################################################
    // DeltaPos(64) = RefPos(64) - Fbk2CntrLoopPos(64)
    _sint64_sub(&sqDeltaPos64, &sMotCtrlRun.sqOldTargetPostn, &sMotCtrl_In.psFbk2CntrLoop->sEncData.sqPostn) ;

    swTgtPosWindowMax = _sint64_comp(&sqDeltaPos64, &sMotCtrlRun.sqPosWindowLimitMax) ; 
    swTgtPosWindowMin = _sint64_comp(&sqDeltaPos64, &sMotCtrlRun.sqPosWindowLimitMin) ; 

    if((swTgtPosWindowMax == 1) || (swTgtPosWindowMin == -1))
    {   // sono fuori dalla finestra
        sMotCtrl_Out.uwStatusWord &= ~MOTCTRL_402_SF_TARGETREACHED ; // azzero il bit nella statusword
        sMotCtrlRFlags.b.bTgtReachedTimerSet = FALSE ;            // ricarico il timeout 
    }
    else
    {   // sono entro la finestra: attendo il timeout
        if (!(sMotCtrl_Out.uwStatusWord & MOTCTRL_402_SF_TARGETREACHED))
        {   // Target position non ancora raggiunta                 
            if (!sMotCtrlRFlags.b.bTgtReachedTimerSet)
            {   // imposto il timeout
                sMotCtrlRun.uwTgtReachedTimeOut = timer_settimeout(uwSysTimers1ms, sMotCtrlRun.swPositionWinTimeout) ;
                sMotCtrlRFlags.b.bTgtReachedTimerSet = TRUE ;
            }

            if(timer_istimedout(uwSysTimers1ms, sMotCtrlRun.uwTgtReachedTimeOut))
                sMotCtrl_Out.uwStatusWord |= MOTCTRL_402_SF_TARGETREACHED ; // setto il bit nella statusword
        }
    }

    if (sMotCtrl_Out.uwStatusWord & MOTCTRL_402_SF_TARGETREACHED)
        sMotCtrlRFlags.b.bIsPositioning = FALSE ;


    // #######################################################################
    // ############# check if following error is in 'the window' #############
    // #######################################################################
    swFollowingErrMax = _sint64_comp(&sMotCtrl_In.psPoOut->sqPosErr, &sMotCtrlRun.sqFollowingErrLimMax) ; 
    swFollowingErrMin = _sint64_comp(&sMotCtrl_In.psPoOut->sqPosErr, &sMotCtrlRun.sqFollowingErrLimMin) ; 

    if((swFollowingErrMax == 1) || (swFollowingErrMin == -1))         
    {   // sono fuori dal limite
        if (!(sMotCtrl_Out.uwStatusWord & MOTCTRL_402_SF_PP_FOLLOWINGERR))
        {   // Following Error bit non ancora settato                
            if (!sMotCtrlRFlags.b.bFollowingErrorTimerSet)
            {   // imposto il timeout
                sMotCtrlRun.uwFollowingErrorTimeOut = timer_settimeout(uwSysTimers1ms, sMotCtrlRun.swFollowingErrTimeout) ;
                sMotCtrlRFlags.b.bFollowingErrorTimerSet = TRUE ;
            }

            if(timer_istimedout(uwSysTimers1ms, sMotCtrlRun.uwFollowingErrorTimeOut))
                sMotCtrl_Out.uwStatusWord |=  MOTCTRL_402_SF_PP_FOLLOWINGERR ; // setto il bit nella statusword
        }
    }
    else
    {   
        sMotCtrl_Out.uwStatusWord &= ~MOTCTRL_402_SF_PP_FOLLOWINGERR ; // azzero il bit nella statusword
        sMotCtrlRFlags.b.bFollowingErrorTimerSet = FALSE ;          // ricarico il timeout 
    }

}


//***************************************************************************
// Statusword bit in interpolated mode
static void motCtrl402OmProfileInterpolated8KHz(void)
{
    SWORD swFollowingErrMax, swFollowingErrMin;
    SQWRD sqDelta;
    SLONG slDelta;
    UWORD uwLocCntrlWord = sMotCtrl_In.psMotCtrl_In->uwControlWord;

    // #######################################################################
    // ##################### do interpolation ################################
    // #######################################################################

        // add fractional part
    slDelta=sMotCtrlRun.slIPSpeedSum+sMotCtrlRun.uwIPSpeedFract;

        // apply remainder to target speed
    sPo_UsrPostnerIn.slLocalSpeed=sMotCtrlRun.slIPTgtSpeed+(((ULONG)slDelta>>16)-((ULONG)sMotCtrlRun.slIPSpeedSum>>16));

        // save actual fractional part
    sMotCtrlRun.slIPSpeedSum=slDelta;

    // #######################################################################
    // ##################### check IP activation on sync #####################
    // #######################################################################

        // take sync when is deactivated
    if(!bSysStatFieldbusSyncing && sMotCtrlRFlags.b.bPrevFieldbusSync)
    {
            // ip mode 0 -> 1
        if((uwLocCntrlWord & MOTCTRL_402_CF_IP_ENABLEIPMODE) && !sMotCtrlRFlags.b.bInterpolationActive)
        {
            sMotCtrlRun.swIPCycleCnt=0;

            sMotCtrl_Out.uwStatusWord|=MOTCTRL_402_SF_IP_IPMODEACTIVE;
            sMotCtrlRFlags.b.bInterpolationActive=1;
        }

            // if enabled
        if(sMotCtrlRFlags.b.bInterpolationActive)
        {
                // reset cycle counter
            sMotCtrlRun.swIPCycleCnt=sMotCtrlRun.swIPNumCycle;

                // take new quota
            if(sMotCtrl_In.psFbk2CntrLoop->ubStatus & ENCMGR_ABSMECHTURN_VALID)
                _uint64_sub((UQWRD *)&sqDelta, (UQWRD *)&sMotCtrl_In.psMotCtrl_In->sqIPQuota, (UQWRD *)&sMotCtrl_In.psFbk2CntrLoop->sqMechAbsPosOffset);
            else
                _sint64_atomic_copy(&sqDelta, &sMotCtrl_In.psMotCtrl_In->sqIPQuota) ;
            _uint64_sub((UQWRD *)&sqDelta, (UQWRD *)&sqDelta, (UQWRD *)&sMotCtrl_Out.sqUserOffset);

                // delta between actual ip quota (demand pos) and next quota to be reached
				// if SRamp enabled, it mut be taken the linear ramp
			if(sPo_PostnerOut.flags.b.bSRampEnabled)								 
	        	_sint64_sub(&sqDelta, &sqDelta, &sPo_PostnerOut.sLocDemand.sqPostn);
			else
        		_sint64_sub(&sqDelta, &sqDelta, &sPo_PostnerOut.sDemand.sqPostn);

                // limit to 32bit
            slDelta=_sint64toSint32(&sqDelta);

                // apply multiplication timing factor and limit result to 48bit
            _sint64_mul_32_32(&sqDelta, &slDelta, &sMotCtrlRun.slIPScaling);
            _sint64toSint48(&sqDelta);

                // save previous speed
            slDelta=sPo_UsrPostnerIn.slLocalSpeed;

                // center part is new speed
            sPo_UsrPostnerIn.slLocalSpeed=sMotCtrlRun.slIPTgtSpeed=INT64_MLONG(sqDelta);

                // lo part is the fractional part (remainder)
            sMotCtrlRun.uwIPSpeedFract=INT64_WORD0(sqDelta);

                // calculate acceleration, spread on all cycles
            slDelta=sPo_UsrPostnerIn.slLocalSpeed-slDelta;
    
                // apply multiplication timing factor and limit result to 48bit
            _sint64_mul_32_32(&sqDelta, &slDelta, &sMotCtrlRun.slIPScaling);
            _sint64toSint48(&sqDelta);
    
                // calc next interpolation period acceleration
            _sint64_shr(&sqDelta,4);
            sPo_UsrPostnerIn.slLocalAcc=INT64_LLONG(sqDelta);
    
                // reset integral part
            sMotCtrlRun.slIPSpeedSum=0l;
        }
    }
        // IP activation/deactivation and following error management, executed only out of SYNC for rt task time saving
    else
    {
            // ip mode 1 -> 0
        if(!(uwLocCntrlWord & MOTCTRL_402_CF_IP_ENABLEIPMODE) && sMotCtrlRFlags.b.bInterpolationActive)
        {
            sPo_UsrPostnerIn.slLocalSpeed=sMotCtrlRun.slIPTgtSpeed=0l;
            sMotCtrlRun.uwIPSpeedFract=0;
    
            sMotCtrl_Out.uwStatusWord&=~MOTCTRL_402_SF_IP_IPMODEACTIVE;
            sMotCtrlRFlags.b.bInterpolationActive=0;
        }
    
            // following error check
        swFollowingErrMax = _sint64_comp(&sMotCtrl_In.psPoOut->sqPosErr, &sMotCtrlRun.sqFollowingErrLimMax) ; 
        swFollowingErrMin = _sint64_comp(&sMotCtrl_In.psPoOut->sqPosErr, &sMotCtrlRun.sqFollowingErrLimMin) ; 
    
        if((swFollowingErrMax == 1) || (swFollowingErrMin == -1))         
        {   // sono fuori dal limite
            if (!(sMotCtrl_Out.uwStatusWord & MOTCTRL_402_SF_PP_FOLLOWINGERR))
            {   // Following Error bit non ancora settato                
                if (!sMotCtrlRFlags.b.bFollowingErrorTimerSet)
                {   // imposto il timeout
                    sMotCtrlRun.uwFollowingErrorTimeOut = timer_settimeout(uwSysTimers1ms, sMotCtrlRun.swFollowingErrTimeout) ;
                    sMotCtrlRFlags.b.bFollowingErrorTimerSet = TRUE ;
                }
    
                if(timer_istimedout(uwSysTimers1ms, sMotCtrlRun.uwFollowingErrorTimeOut))
                    sMotCtrl_Out.uwStatusWord |=  MOTCTRL_402_SF_PP_FOLLOWINGERR ; // setto il bit nella statusword
            }
        }
        else
        {   
            sMotCtrl_Out.uwStatusWord &= ~MOTCTRL_402_SF_PP_FOLLOWINGERR ; // azzero il bit nella statusword
            sMotCtrlRFlags.b.bFollowingErrorTimerSet = FALSE ;          // ricarico il timeout 
        }
    }

        // update cycle left counter, tolerate one period for recovering
    if(sMotCtrlRun.swIPCycleCnt>sMotCtrlRun.swIPRecoveryNumCycle)
        sMotCtrlRun.swIPCycleCnt--;

        // after recovering period stop running
    else if(sMotCtrlRun.swIPCycleCnt==sMotCtrlRun.swIPRecoveryNumCycle)
    {
        sPo_UsrPostnerIn.slLocalSpeed=sMotCtrlRun.slIPTgtSpeed=0l;
        sMotCtrlRun.uwIPSpeedFract=0;
        sMotCtrlRun.swIPCycleCnt--;
    }

    sMotCtrlRFlags.b.bPrevFieldbusSync=bSysStatFieldbusSyncing;

        // IP quota monitor (32bit)
    sMotCtrl_Out.slIPQuotaMonitor = INT64_MLONG(sMotCtrl_In.psMotCtrl_In->sqIPQuota);
}


//***************************************************************************
// Statusword bit in profile velocity 
void motCtrl402OmProfileVelocity8KHz(void)
{
    // #######################################################################
    // ############## check if target speed is in 'the  window' ##############
    // #######################################################################
    if (labs(sPo_UsrPostnerIn.slTargetSpeed - sMotCtrl_In.psFbk2CntrLoop->sEncData.slSpeed) < sMotCtrlRun.slVelocityWindow)
    {
        if (!(sMotCtrl_Out.uwStatusWord & MOTCTRL_402_SF_TARGETREACHED))
        {   // Target speed non ancora raggiunta                 
            if (!sMotCtrlRFlags.b.bTgtReachedTimerSet)
            {   // imposto il timeout
                sMotCtrlRun.uwTgtReachedTimeOut = timer_settimeout(uwSysTimers1ms, sMotCtrlRun.swVelocityWinTimeout) ;
                sMotCtrlRFlags.b.bTgtReachedTimerSet = TRUE ;
            }

            if(timer_istimedout(uwSysTimers1ms, sMotCtrlRun.uwTgtReachedTimeOut))
                sMotCtrl_Out.uwStatusWord |=  MOTCTRL_402_SF_TARGETREACHED ; // setto il bit nella statusword
        }
    }
    else
    {   
        sMotCtrl_Out.uwStatusWord &= ~MOTCTRL_402_SF_TARGETREACHED ; // azzero il bit nella statusword
        sMotCtrlRFlags.b.bTgtReachedTimerSet = FALSE ;            // ricarico il timeout 
    }


    // #######################################################################
    // ############# check if shaft can be considered standstill #############
    // #######################################################################
    if (labs(sMotCtrl_In.psFbk2CntrLoop->sEncData.slSpeed) > sMotCtrlRun.slVelocityThreshold)
    {
        if (sMotCtrl_Out.uwStatusWord & MOTCTRL_402_SF_PV_SPEED)
        {
            if (!sMotCtrlRFlags.b.bInSpeedTimerSet)
            {   // imposto il timeout
                sMotCtrlRun.uwInSpeedTimeOut = timer_settimeout(uwSysTimers1ms, sMotCtrlRun.swVelocityThrTimeout) ;
                sMotCtrlRFlags.b.bInSpeedTimerSet = TRUE ;
            }

            if (timer_istimedout(uwSysTimers1ms, sMotCtrlRun.uwInSpeedTimeOut))
                sMotCtrl_Out.uwStatusWord &= ~MOTCTRL_402_SF_PV_SPEED ; // azzero il bit nella statusword
        }
    }
    else
    {
        sMotCtrl_Out.uwStatusWord |= MOTCTRL_402_SF_PV_SPEED ; // setto il bit nella statusword
        sMotCtrlRFlags.b.bInSpeedTimerSet = FALSE ;         // ricarico il timeout 
    }
}


//***************************************************************************
// Homing mode 8kHz
static void motCtrl402OmProfileHoming8KHz(void)
{
    UBYTE ubActHomeSwitch;
    UWORD uwSeqCmd;
    SLONG slSpeed;
    BOOL bStat;
    UWORD uwLocCntrlWord = sMotCtrl_In.psMotCtrl_In->uwControlWord;

    ubActHomeSwitch = HmGetSwitchStatus(sMotCtrlRun.ubHomingSSrcHomeSwitch, sMotCtrl_In.psMotCtrl_In->sbHomingSoftHomeSwitch);
    // #######################################################################
    // ##################### check home operation start ######################
    // #######################################################################

    if( uwLocCntrlWord & MOTCTRL_402_CF_HM_OP_START )
    {
            // if pointer is NULL then method is not valid or there's some
            // other parameters error or already error
        if(sMotCtrlRun.puwHomingPCmd==NULL || sMotCtrlRFlags.b.bHomeParamError || (sMotCtrl_Out.uwStatusWord&MOTCTRL_402_SF_HM_ERROR))
        {
            sMotCtrl_Out.uwStatusWord |= MOTCTRL_402_SF_HM_ERROR;
        }
        else
        {
			BOOL bHomingDone ;

			if ((sMotCtrl_Out.uwStatusWord & MOTCTRL_402_SF_HOMING_DONE) == MOTCTRL_402_SF_HOMING_DONE)
			{
				bHomingDone = TRUE ;
					// set EndOfSequence command
				uwSeqCmd = HM_ENDOFSEQUENCE ;
			}
			else
			{
				bHomingDone = FALSE ;
		        	// get actual command
            	uwSeqCmd=*sMotCtrlRun.puwHomingPCmd;	
		    }
    
                // check for termination
            if( uwSeqCmd==HM_ENDOFSEQUENCE )
            {
                    // check if already terminated
                if((!sMotCtrlRFlags.b.bHomeAttained) && (!bHomingDone))
                {
                        // signal home attained
                    sMotCtrlRFlags.b.bHomeAttained = TRUE;
                    sMotCtrl_Out.uwStatusWord |= MOTCTRL_402_SF_HM_ATTAINED;
                    sMotCtrl_Out.uwStatusWord |= MOTCTRL_402_SF_HOMING_DONE;
                        // and stop    
                    sPo_UsrPostnerIn.slLocalSpeed = 0l;
                }
            }
            else 
            {
                if (!(sMotCtrlRun.uwLocalFlags2Use & LF_HM_OP_STARTED))
                {    // bit Homing Start 1->0
                    sMotCtrlRun.uwLocalFlags2Use |= LF_HM_OP_STARTED ;
                }
                    // setup homing acceleration
                sPo_UsrPostnerIn.slLocalAcc = sMotCtrlRun.slHomingAcceleration;
                sPo_UsrPostnerIn.slLocalDec = sMotCtrlRun.slHomingAcceleration;
    
                    // select zero or switch speed
                if(uwSeqCmd&HM_SETZEROSPEED)
                    slSpeed = sMotCtrlParameters.slHomingSpeedZero;
                else
                    slSpeed = sMotCtrlParameters.slHomingSpeedSwitch;
    
                if(slSpeed<0l)
                    slSpeed=0l;
    
                    // select positive or negative speed
                if(uwSeqCmd&HM_SETNEGATIVESPEED)
                    sPo_UsrPostnerIn.slLocalSpeed = -slSpeed;
                else
                    sPo_UsrPostnerIn.slLocalSpeed = +slSpeed;

                    // check all enabled inputs, that will be or'ed
                bStat=(BOOL)(uwSeqCmd&HM_NOTRIGGERS);
    
                if(uwSeqCmd&HM_WAITHINEGLIMSWITCH)
                    bStat|=HmGetSwitchStatus(sMotCtrlRun.ubHomingSSrcNegSwitch, sMotCtrl_In.psMotCtrl_In->sbHomingSoftNegSwitch);

                if(uwSeqCmd&HM_WAITLONEGLIMSWITCH)
                    bStat|=!HmGetSwitchStatus(sMotCtrlRun.ubHomingSSrcNegSwitch, sMotCtrl_In.psMotCtrl_In->sbHomingSoftNegSwitch);

                if(uwSeqCmd&HM_WAITHIPOSLIMSWITCH)
                    bStat|= HmGetSwitchStatus(sMotCtrlRun.ubHomingSSrcPosSwitch, sMotCtrl_In.psMotCtrl_In->sbHomingSoftPosSwitch);
    
                if(uwSeqCmd&HM_WAITLOPOSLIMSWITCH)
                    bStat|=!HmGetSwitchStatus(sMotCtrlRun.ubHomingSSrcPosSwitch, sMotCtrl_In.psMotCtrl_In->sbHomingSoftPosSwitch);
    
                if(uwSeqCmd&HM_WAITRISEHOMESWITCH)
                    bStat|=!sMotCtrlRun.ubHomingPrevHomeSwitch &&  ubActHomeSwitch;
    
                if(uwSeqCmd&HM_WAITFALLHOMESWITCH)
                    bStat|= sMotCtrlRun.ubHomingPrevHomeSwitch && !ubActHomeSwitch;

                if(uwSeqCmd&HM_WAITHIHOMESWITCH)
                    bStat|= ubActHomeSwitch;
    
                if(uwSeqCmd&HM_WAITLOHOMESWITCH)
                    bStat|=!ubActHomeSwitch;
    
                    // if at least one switch is activated and requested then take current position snapshot
                if(bStat && (uwSeqCmd&HM_CATCHPOSITION))
                {   // if valid also add absolute position offset
                    if(sMotCtrl_In.psFbk2CntrLoop->ubStatus&ENCMGR_ABSMECHTURN_VALID)
                    {   // sqHomingCalcOffset = sqPostn + sqMechAbsPosOffset (abs pos)
                        _uint64_add((UQWRD *)&sMotCtrlRun.sqHomingCalcOffset, (UQWRD *)&sMotCtrl_In.psFbk2CntrLoop->sEncData.sqPostn, (UQWRD *)&sMotCtrl_In.psFbk2CntrLoop->sqMechAbsPosOffset);
                    }
                    else
                    {   // sqHomingCalcOffset = sqPostn (rel pos)
                        _sint64_atomic_copy(&sMotCtrlRun.sqHomingCalcOffset, &sMotCtrl_In.psFbk2CntrLoop->sEncData.sqPostn);
                    }
                }
    
                    // index pulse is valid when absolute position is signaled as valid
                if((uwSeqCmd&HM_WAITINDEXPULSE) && (sMotCtrl_In.psFbk2CntrLoop->ubStatus&ENCMGR_ABSMECHTURN_VALID))
                {
                    bStat=TRUE;

                        // if requested catch position
                    if(uwSeqCmd&HM_CATCHPOSITION)
                    {
                        _uint64_add((UQWRD *)&sMotCtrlRun.sqHomingCalcOffset, (UQWRD *)&sMotCtrl_In.psFbk2CntrLoop->sEncData.sqPostn, (UQWRD *)&sMotCtrl_In.psFbk2CntrLoop->sqMechAbsPosOffset);
        
                            // virtual index is at zero degrees
                        sMotCtrlRun.sqHomingCalcOffset.lo=0ul;
        
                            // if running positive index is at following turn, if negative is at previous
                            // then if positive increment number of turns, if negative do nothing
                        if(!(uwSeqCmd&HM_SETNEGATIVESPEED))
                            sMotCtrlRun.sqHomingCalcOffset.hi++;
                    }
                }
                
                    // if condition is triggered then select next step and calculate actual offset
                if(bStat)
                {
                        // if was requested then calculate overall offset = homing user offset - homing calc position
                        // in order to provide immediate feedback, such equation is evaluated also in background
                    if(uwSeqCmd&HM_CATCHPOSITION)
                    {
                        _uint64_atomic_copy(&sMotCtrlRun.sqHomingUserOffset8kHz, &sMotCtrlParameters.sqHomingUserOffset);
                        _uint64_sub((UQWRD *)&sMotCtrl_Out.sqUserOffset, (UQWRD *)&sMotCtrlRun.sqHomingUserOffset8kHz, (UQWRD *)&sMotCtrlRun.sqHomingCalcOffset);
                    }
                    sMotCtrlRun.puwHomingPCmd++;
                }
            }
        }
    }

    // #######################################################################
    // ##################### homing inactive, keep reset #####################
    // #######################################################################

    else
    {
        sPo_UsrPostnerIn.slLocalSpeed = 0l;
        sMotCtrlRFlags.b.bHomeAttained = FALSE;
        sMotCtrlRun.puwHomingPCmd = sMotCtrlRun.puwHomingRun;
        
        sMotCtrl_Out.uwStatusWord &= ~(MOTCTRL_402_SF_HM_ATTAINED|MOTCTRL_402_SF_HM_ERROR);

        if (sMotCtrlRun.uwLocalFlags2Use & LF_HM_OP_STARTED) // bit Homing Start 1->0 
            sMotCtrlRun.uwLocalFlags2Use &= ~LF_HM_OP_STARTED ; 
    }

    sMotCtrlRun.ubHomingPrevHomeSwitch = ubActHomeSwitch;
}


//***************************************************************************
static void motCtrl402OmProfileTorque8KHz(void)
{
    ;
}

//***************************************************************************
static void motCtrl402OmProfileCSPosition8KHz(void)
{
    SWORD swFollowingErrMax, swFollowingErrMin;
    SQWRD sqDelta;
    SLONG slDelta;

        // add fractional part
    slDelta=sMotCtrlRun.slIPSpeedSum+sMotCtrlRun.uwIPSpeedFract;

        // apply remainder to target speed
    sPo_UsrPostnerIn.slLocalSpeed=sMotCtrlRun.slIPTgtSpeed+(((ULONG)slDelta>>16)-((ULONG)sMotCtrlRun.slIPSpeedSum>>16));

        // save actual fractional part
    sMotCtrlRun.slIPSpeedSum=slDelta;

        // on sync
    if(!bSysStatFieldbusSyncing && sMotCtrlRFlags.b.bPrevFieldbusSync)
    {
            // reset cycle counter
        sMotCtrlRun.swIPCycleCnt=sMotCtrlRun.swIPNumCycle;

            // take new quota
        if(sMotCtrl_In.psFbk2CntrLoop->ubStatus & ENCMGR_ABSMECHTURN_VALID)
            _uint64_sub((UQWRD *)&sqDelta, (UQWRD *)&sMotCtrl_In.psMotCtrl_In->sqTargetPostn, (UQWRD *)&sMotCtrl_In.psFbk2CntrLoop->sqMechAbsPosOffset);
        else
            _sint64_atomic_copy(&sqDelta, &sMotCtrl_In.psMotCtrl_In->sqTargetPostn) ;
        _uint64_sub((UQWRD *)&sqDelta, (UQWRD *)&sqDelta, (UQWRD *)&sMotCtrl_Out.sqUserOffset);

            // delta between actual ip quota (demand pos) and next quota to be reached
			// if SRamp enabled, it mut be taken the linear ramp
		if(sPo_PostnerOut.flags.b.bSRampEnabled)								 
        	_sint64_sub(&sqDelta, &sqDelta, &sPo_PostnerOut.sLocDemand.sqPostn);
		else
        	_sint64_sub(&sqDelta, &sqDelta, &sPo_PostnerOut.sDemand.sqPostn);

            // limit to 32bit
        slDelta=_sint64toSint32(&sqDelta);

            // apply multiplication timing factor and limit result to 48bit
        _sint64_mul_32_32(&sqDelta, &slDelta, &sMotCtrlRun.slIPScaling);
        _sint64toSint48(&sqDelta);

            // save previous speed
        slDelta=sPo_UsrPostnerIn.slLocalSpeed;

            // center part is new speed
        sPo_UsrPostnerIn.slLocalSpeed=sMotCtrlRun.slIPTgtSpeed=INT64_MLONG(sqDelta);

            // lo part is the fractional part (remainder)
        sMotCtrlRun.uwIPSpeedFract=INT64_WORD0(sqDelta);

            // calculate acceleration, spread on all cycles
        slDelta=sPo_UsrPostnerIn.slLocalSpeed-slDelta;

            // apply multiplication timing factor and limit result to 48bit
        _sint64_mul_32_32(&sqDelta, &slDelta, &sMotCtrlRun.slIPScaling);
        _sint64toSint48(&sqDelta);

            // calc next interpolation period acceleration
        _sint64_shr(&sqDelta,4);
        sPo_UsrPostnerIn.slLocalAcc=INT64_LLONG(sqDelta);

            // reset integral part
        sMotCtrlRun.slIPSpeedSum=0l;
    }
    else
        // following error management, executed only out of SYNC for rt task time saving
    {
        swFollowingErrMax = _sint64_comp(&sMotCtrl_In.psPoOut->sqPosErr, &sMotCtrlRun.sqFollowingErrLimMax) ; 
        swFollowingErrMin = _sint64_comp(&sMotCtrl_In.psPoOut->sqPosErr, &sMotCtrlRun.sqFollowingErrLimMin) ; 
    
        if((swFollowingErrMax == 1) || (swFollowingErrMin == -1))         
        {
            if (!(sMotCtrl_Out.uwStatusWord & MOTCTRL_402_SF_PP_FOLLOWINGERR))
            {
                if (!sMotCtrlRFlags.b.bFollowingErrorTimerSet)
                {
                    sMotCtrlRun.uwFollowingErrorTimeOut = timer_settimeout(uwSysTimers1ms, sMotCtrlRun.swFollowingErrTimeout) ;
                    sMotCtrlRFlags.b.bFollowingErrorTimerSet = TRUE ;
                }
    
                if(timer_istimedout(uwSysTimers1ms, sMotCtrlRun.uwFollowingErrorTimeOut))
                    sMotCtrl_Out.uwStatusWord |=  MOTCTRL_402_SF_PP_FOLLOWINGERR ;
            }
        }
        else
        {   
            sMotCtrl_Out.uwStatusWord &= ~MOTCTRL_402_SF_PP_FOLLOWINGERR ;
            sMotCtrlRFlags.b.bFollowingErrorTimerSet = FALSE ;
        }
    }

        // update cycle left counter, tolerate one period for recovering
    if(sMotCtrlRun.swIPCycleCnt>sMotCtrlRun.swIPRecoveryNumCycle)
        sMotCtrlRun.swIPCycleCnt--;

        // after recovering period stop running
    else if(sMotCtrlRun.swIPCycleCnt==sMotCtrlRun.swIPRecoveryNumCycle)
    {
        sPo_UsrPostnerIn.slLocalSpeed=sMotCtrlRun.slIPTgtSpeed=0l;
        sMotCtrlRun.uwIPSpeedFract=0;
        sPo_UsrPostnerIn.slLocalAcc=0l;
        sMotCtrlRun.swIPCycleCnt--;
    }

    sMotCtrlRFlags.b.bPrevFieldbusSync=bSysStatFieldbusSyncing;

    sMotCtrl_Out.uwStatusWord |= MOTCTRL_402_SF_CYCLICTARGETUSED ;

        // IP quota monitor (32bit)
    sMotCtrl_Out.slIPQuotaMonitor = INT64_MLONG(sMotCtrl_In.psMotCtrl_In->sqTargetPostn);
}

//***************************************************************************
static void motCtrl402OmProfileCSVelocity8KHz(void)
{
    SQWRD sqDelta;
    SLONG slDelta;

        // add fractional part
    slDelta=sMotCtrlRun.slIPSpeedSum+sMotCtrlRun.uwIPSpeedFract;

        // apply remainder to target speed
    sPo_UsrPostnerIn.slLocalSpeed+=sMotCtrlRun.slIPTgtSpeed+(((ULONG)slDelta>>16)-((ULONG)sMotCtrlRun.slIPSpeedSum>>16));

        // save actual fractional part
    sMotCtrlRun.slIPSpeedSum=slDelta;

        // on sync
    if(!bSysStatFieldbusSyncing && sMotCtrlRFlags.b.bPrevFieldbusSync)
    {
            // reset cycle counter
        sMotCtrlRun.swIPCycleCnt=sMotCtrlRun.swIPNumCycle;

            // delta between new speed quota and actual one
        slDelta=sPo_UsrPostnerIn.slTargetSpeed-sPo_UsrPostnerIn.slLocalSpeed;

            // apply multiplication timing factor and limit result to 48bit
        _sint64_mul_32_32(&sqDelta, &slDelta, &sMotCtrlRun.slIPScaling);
        _sint64toSint48(&sqDelta);

            // center part is new speed step
        sMotCtrlRun.slIPTgtSpeed=INT64_MLONG(sqDelta);

            // lo part is the fractional part (remainder)
        sMotCtrlRun.uwIPSpeedFract=INT64_WORD0(sqDelta);

            // calc next interpolation period acceleration
        _sint64_shr(&sqDelta,4);
        sPo_UsrPostnerIn.slLocalAcc=INT64_LLONG(sqDelta);

            // reset integral part
        sMotCtrlRun.slIPSpeedSum=0l;
    }

        // update cycle left counter, tolerate one period for recovering
    if(sMotCtrlRun.swIPCycleCnt>sMotCtrlRun.swIPRecoveryNumCycle)
        sMotCtrlRun.swIPCycleCnt--;

        // after recovering period stop running
    else if(sMotCtrlRun.swIPCycleCnt==sMotCtrlRun.swIPRecoveryNumCycle)
    {
        sPo_UsrPostnerIn.slLocalSpeed=sMotCtrlRun.slIPTgtSpeed=0l;
        sMotCtrlRun.uwIPSpeedFract=0;
        sPo_UsrPostnerIn.slLocalAcc=0l;
        sMotCtrlRun.swIPCycleCnt--;
    }

    sMotCtrlRFlags.b.bPrevFieldbusSync=bSysStatFieldbusSyncing;

    sMotCtrl_Out.uwStatusWord |= MOTCTRL_402_SF_CYCLICTARGETUSED ;
}

//***************************************************************************
static void motCtrl402OmProfileCSTorque8KHz(void)
{
    SQWRD sqDelta;
    SLONG slDelta,slNewTgt;

        // add fractional part
    slDelta=sMotCtrlRun.slIPSpeedSum+sMotCtrlRun.uwIPSpeedFract;

        // apply remainder to target speed
    sMotCtrl_Out.sIRefs.slIqRef+=sMotCtrlRun.slIPTgtSpeed+(((ULONG)slDelta>>16)-((ULONG)sMotCtrlRun.slIPSpeedSum>>16));

        // save actual fractional part
    sMotCtrlRun.slIPSpeedSum=slDelta;

        // on sync
    if(!bSysStatFieldbusSyncing && sMotCtrlRFlags.b.bPrevFieldbusSync)
    {
            // reset cycle counter
        sMotCtrlRun.swIPCycleCnt=sMotCtrlRun.swIPNumCycle;

            // conversion from rated torque to current ref
        slNewTgt=_sint32_scale_32((SLONG)sMotCtrl_In.psMotCtrl_In->swTargetTorque, sMotCtrl_Out.slCSTorqueToCurrent);

            // delta between new torque quota and actual one
        slDelta=slNewTgt-sMotCtrl_Out.sIRefs.slIqRef;

            // apply multiplication timing factor and limit result to 48bit
        _sint64_mul_32_32(&sqDelta, &slDelta, &sMotCtrlRun.slIPScaling);
        _sint64toSint48(&sqDelta);

            // center part is new speed step
        sMotCtrlRun.slIPTgtSpeed=INT64_MLONG(sqDelta);

            // lo part is the fractional part (remainder)
        sMotCtrlRun.uwIPSpeedFract=INT64_WORD0(sqDelta);

            // reset integral part
        sMotCtrlRun.slIPSpeedSum=0l;
    }

        // update cycle left counter, tolerate one period for recovering
    if(sMotCtrlRun.swIPCycleCnt>sMotCtrlRun.swIPRecoveryNumCycle)
        sMotCtrlRun.swIPCycleCnt--;

        // after recovering period stop running
    else if(sMotCtrlRun.swIPCycleCnt==sMotCtrlRun.swIPRecoveryNumCycle)
    {
        sMotCtrl_Out.sIRefs.slIqRef=sMotCtrlRun.slIPTgtSpeed=0l;
        sMotCtrlRun.uwIPSpeedFract=0;
        sMotCtrlRun.swIPCycleCnt--;
    }

    sMotCtrlRFlags.b.bPrevFieldbusSync=bSysStatFieldbusSyncing;

    sMotCtrl_Out.uwStatusWord |= MOTCTRL_402_SF_CYCLICTARGETUSED ;
}

//***************************************************************************
// imposto il modo di velocita' (sia il modulo posizionatore che l'anello di spazio velocita' devono essere attivi)
// Inoltre, informo il modulo software posizionatore che voglio eseguire un quickstop, 
// ovvero una rampa di decelerazione con velocita' finale = zero
static void motCtrl402QuickStopSetup(UWORD uwStopModeCode, UWORD uwStopModeOption)
{
    ULONG ulAddressFunction2Use ;
    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL) ;

    fast_atomic_set_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DISABLECONTROLWORDINPUT) ; // NON accetto nuove controlword

    // pulisco la parte addizionale della statusword (bit 15->8) e la target reached
    fast_atomic_clear_bits(sMotCtrl_Out.uwStatusWord, RESETMASK_HIGHSTATUSWORD);
    
    DrvTskCtrl_Config(DRVTSKCTRL_OP_POSITIONER) ; // metto a posto i puntatori per mettermi in controllo di velocita'

    sMotCtrl_OStatus.b.bPostnerEnable = TRUE ; // modulo posizionatore 'acceso'
//    sMotCtrl_OStatus.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo
    sMotCtrlRFlags.b.bPID_IntegralEnable = TRUE ; // faccio andare l'integrale dell'anello di controllo
    sMotCtrl_OStatus.b.bQuickStop = TRUE ; // informo il posizionatore che voglio eseguire un quickstop,

    // imposto la decelerazione da usare
    switch(uwStopModeOption)
    {
        case MOTCTRL_402_OC_QUICKSTOP:
        case MOTCTRL_402_OC_QUICKSTOPANDSTAY:
            // disable with quickstop ramp
            atomic_move(&(sPo_UsrPostnerIn.slQuickStopDec2Use), &sPo_PostnerParam.slQuickStopDec, sizeof(SLONG)) ;            
            break ;

        case MOTCTRL_402_OC_SLOWDOWN:
        case MOTCTRL_402_OC_SLOWDOWNANDSTAY: 
        case MOTCTRL_402_OC_POWEROFF:           
            // disable with deceleration ramp                
            atomic_move(&(sPo_UsrPostnerIn.slQuickStopDec2Use), &sPo_PostnerParam.slProfileDec, sizeof(SLONG)) ; 
            break ;

        default:
            assert(FALSE) ;
    } 

    // imposto la funzione da eseguire ad 8KHz
    switch(uwStopModeCode)
    {
        case MOTCTRL_402_QUICKSTOP:    
            ulAddressFunction2Use = (ULONG)(&ExecuteQuickstop8KHz) ;
            break ;
            
        case MOTCTRL_402_SHUTDOWN:    
            ulAddressFunction2Use = (ULONG)(&ExecuteShutdown8KHz) ;
            break ;

        case MOTCTRL_402_DISABLEOP:    
            ulAddressFunction2Use = (ULONG)(&ExecuteDisableOperation8KHz) ;
            break ;
            
        case MOTCTRL_402_HALT:    
            ulAddressFunction2Use = (ULONG)(&ExecuteHalt8KHz) ;
            break ;

        case MOTCTRL_402_FAULTREACTION:    
            ulAddressFunction2Use = (ULONG)(&ExecuteFaultReaction8KHz) ;
            break ;
        
        default:
            assert(FALSE) ;
    }    

    sMotCtrlRun.uwStopMode = uwStopModeOption ; // imposto il modo di stop da usare ad 8KHz 
    
    // resetto i timer? serve?
    sMotCtrlRFlags.b.bInSpeedTimerSet = FALSE ;  
    sMotCtrlRFlags.b.bTgtReachedTimerSet = FALSE ;        
    sMotCtrlRFlags.b.bStopMotorSetupDone = TRUE ; // setup funzione per stop impostata

    // resetto flag motore stoppato
    sMotCtrlRFlags.b.bMotorStopped=FALSE;

    // imposto il puntatore alla funzione da usare (lo faccio atomico per evitare casini) 
    atomic_move(&pfSelectedProfile2Use, &ulAddressFunction2Use, sizeof(ULONG)) ;

    Os_EndCriticalSection(OS_CRITSECT_GLOBAL) ;
}


// ****************************************************
static void ExecuteQuickstop8KHz(void)
{
    if (!sMotCtrlRFlags.b.bMotorStopped)
    {   // motore non ancora dichiarato fermo 
        sMotCtrl_Out.uwStatusWord &= ~MOTCTRL_402_SF_TARGETREACHED ; // azzero il bit nella statusword

        switch(sMotCtrlRun.uwStopMode)
        {
            case MOTCTRL_402_OC_SLOWDOWN:  // fermata con rampa di decelerazione normale; disabilitazione azionamento
            case MOTCTRL_402_OC_QUICKSTOP: // fermata con rampa di decelerazione quickstop; disabilitazione azionamento
                sMotCtrlRFlags.b.bMotorStopped = SlowDownStop(MOTCTRL_CMD_ENABLE_POWER) ;
                break ;
    
            case MOTCTRL_402_OC_SLOWDOWNANDSTAY:  // fermata con rampa di decelerazione normale e rimane in quickstop
            case MOTCTRL_402_OC_QUICKSTOPANDSTAY: // fermata con rampa di decelerazione quickstop e rimane in quickstop
                sMotCtrlRFlags.b.bMotorStopped = SlowDownStop(MOTCTRL_CMD_ENABLE_ZERO_SPEED) ;
                break ;
            
            default:
            case MOTCTRL_402_OC_POWEROFF: 
                // disabilitazione diretta dell'azionamento
                sMotCtrlRFlags.b.bMotorStopped = PowerOffStop(MOTCTRL_CMD_ENABLE_POWER) ;                
                break ;
        }

        if (sMotCtrlRFlags.b.bMotorStopped)
            fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DISABLECONTROLWORDINPUT) ; // motore dichiarato fermo: accetto nuove controlword
    }
    else
        sMotCtrl_Out.uwStatusWord |=  MOTCTRL_402_SF_TARGETREACHED ; // setto il bit nella statusword
}


// ****************************************************
static void ExecuteShutdown8KHz(void)
{
    if (!sMotCtrlRFlags.b.bMotorStopped)
        // motore non ancora dichiarato fermo    
        switch(sMotCtrlRun.uwStopMode)
        {
            case MOTCTRL_402_OC_SLOWDOWN: 
                // fermata con rampa di decelerazione normale; disabilitazione azionamento
                sMotCtrlRFlags.b.bMotorStopped = SlowDownStop(MOTCTRL_CMD_ENABLE_POWER) ;               
                break ;
    
            default:
            case MOTCTRL_402_OC_POWEROFF: 
                // disabilitazione diretta dell'azionamento
                sMotCtrlRFlags.b.bMotorStopped = PowerOffStop(MOTCTRL_CMD_ENABLE_POWER) ;
                break ;
        }
}


// ****************************************************
static void ExecuteDisableOperation8KHz(void)
{
    if (!sMotCtrlRFlags.b.bMotorStopped)
        // motore non ancora dichiarato fermo    
        switch(sMotCtrlRun.uwStopMode)
        {
            case MOTCTRL_402_OC_SLOWDOWN: 
                // fermata con rampa di decelerazione normale, potenza abilitata, disabilito 'applicazione'
                sMotCtrlRFlags.b.bMotorStopped = SlowDownStop(MOTCTRL_CMD_ENABLE_REFERENCE) ;             
                break ;
    
            default:
            case MOTCTRL_402_OC_POWEROFF:
                // potenza abilitata, disabilito 'applicazione'
                sMotCtrlRFlags.b.bMotorStopped = PowerOffStop(MOTCTRL_CMD_ENABLE_REFERENCE) ;
                break ;
        }
}


// ****************************************************
static void ExecuteHalt8KHz(void)
{
    if (!sMotCtrlRFlags.b.bMotorStopped)
    {   // motore non ancora dichiarato fermo: sono in rampa e sto aspettando di fermarmi: azzero il bit nella statusword      
        sMotCtrl_Out.uwStatusWord &= ~MOTCTRL_402_SF_TARGETREACHED ; // azzero il bit nella statusword

        switch(sMotCtrlRun.uwStopMode)
        {
            default:
            case MOTCTRL_402_OC_SLOWDOWN:  // fermata con rampa di decelerazione normale
            case MOTCTRL_402_OC_QUICKSTOP: // fermata con rampa di decelerazione quickstop
                sMotCtrlRFlags.b.bMotorStopped = SlowDownStop(MOTCTRL_CMD_ENABLE_ZERO_SPEED) ;
                break ;
        }
        if (sMotCtrlRFlags.b.bMotorStopped) 
        { 
            fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DISABLECONTROLWORDINPUT) ; // motore dichiarato fermo: accetto nuove controlword
            sMotCtrl_Out.uwStatusWord |=  MOTCTRL_402_SF_TARGETREACHED ; // setto il bit nella statusword
        }
    }
    else
    {   // sono fermo..tengo fermo in posizione con il quickstop del posizionatore 
        sMotCtrl_OStatus.b.bQuickStop = TRUE ; // attivo la flag di quickstop del modulo 'posizionatore'
        sMotCtrl_Out.uwStatusWord |=  MOTCTRL_402_SF_TARGETREACHED ; // setto il bit nella statusword
    }
}


// ****************************************************
static void ExecuteFaultReaction8KHz(void)
{
    if (!sMotCtrlRFlags.b.bMotorStopped)
    {   // motore non ancora dichiarato fermo
        switch(sMotCtrlRun.uwStopMode)
        {   // option code
            case MOTCTRL_402_OC_SLOWDOWN:  // fermata con rampa di decelerazione normale
            case MOTCTRL_402_OC_QUICKSTOP: // fermata con rampa di decelerazione quickstop
                sMotCtrlRFlags.b.bMotorStopped = SlowDownStop(MOTCTRL_CMD_ENABLE_POWER) ;
                break ;
    
            default:
            case MOTCTRL_402_OC_POWEROFF:  // disabilitazione dell'azionamento, il motore gira libero
                sMotCtrlRFlags.b.bMotorStopped = PowerOffStop(MOTCTRL_CMD_ENABLE_POWER) ;
                break ;
        }
    }
}        


//***************************************************************************
// PowerOff Stop del motore
static BOOL PowerOffStop(UWORD uwPowerOffStopCommand /*, BOOL bPowerOffStopOption*/)
{
    // MOTCTRL_CMD_ENABLE_POWER    , FALSE : disabilitazione diretta della potenza
    // MOTCTRL_CMD_ENABLE_REFERENCE, FALSE : disabilitazione dell'applicazione (potenza abilitata)

    BOOL bRetVal ;

    if(uwPowerOffStopCommand == MOTCTRL_CMD_ENABLE_REFERENCE)
    {   // disabilito l'applicazione
        sMotCtrl_OStatus.b.bApplicationEnabled = FALSE ; // disabilito l'applicazione
        bRetVal = TRUE ;
    }
    else
    {   // disabilito la potenza
        fast_atomic_set_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DOPWROFFSTOP) ; // flag: sto facendo un poweroff 
        sMotCtrl_Out.sPowerStageCtrl.b.bPowerEnable = FALSE;
               
        if (sMotCtrl_In.psPowerStageStatus->b.bVoltageEnabled)
            bRetVal = FALSE ;        
        else
        {   // non c'e' piu' tensione
            fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DOPWROFFSTOP) ; // flag: finito il poweroff
            bRetVal = TRUE ;
        }
    }

    return bRetVal ;
} 


//***************************************************************************
// SlowDown Stop del motore
static BOOL SlowDownStop(UWORD uwSlowDownStopCommand)
{
    // MOTCTRL_CMD_ENABLE_POWER     : rampa e disabilitazione della potenza
    // MOTCTRL_CMD_ENABLE_REFERENCE : rampa e disabilitazione dell'applicazione (potenza abilitata, ma riferimenti a zero all'fpga)
    // MOTCTRL_CMD_ENABLE_ZERO_SPEED        : rampa e riferimento di velocit?a zero (halt e quickstop 'stay')

    // per verificare se sono in rampa, controllo la DemandSpeed. 
    // Ho impostato il modo di velocita' e target speed = 0, QUINDI quando la DemandSpeed e' = 0, vuol dire che sono fermo.

    BOOL bRetVal ;

    if(uwSlowDownStopCommand == MOTCTRL_CMD_ENABLE_REFERENCE)
    {   // disabilito l'applicazione
        if(sMotCtrl_In.psPoOut->sDemand.slSpeed == 0L)
        {   // rampa terminata                       
            sMotCtrl_OStatus.b.bApplicationEnabled = FALSE ; // disabilito l'applicazione 
            bRetVal = TRUE ;
        }   
        else
            bRetVal = FALSE ; // sono in rampa            
    }
    else if(uwSlowDownStopCommand == MOTCTRL_CMD_ENABLE_POWER)
    {   // disabilito la potenza   
        fast_atomic_set_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DOPWROFFSTOP) ; // flag: sto facendo un poweroff 
                            
        if(sMotCtrl_In.psPoOut->sDemand.slSpeed == 0L)
        {   /* rampa terminata: disabilito la potenza */
            sMotCtrl_Out.sPowerStageCtrl.b.bPowerEnable = FALSE;
        
            if (sMotCtrl_In.psPowerStageStatus->b.bVoltageEnabled)
                bRetVal = FALSE ;
            else
            {   // non c'e' piu' tensione
                fast_atomic_clear_bits(sMotCtrlRun.uwLocalFlags2Use, LF_DOPWROFFSTOP) ; // flag: finito il poweroff 
                bRetVal = TRUE ;
            }
        }
        else
            bRetVal = FALSE ; // sono in rampa
    }
    else
    {   // Halt e quickstop 'stay': asse abilitato, in velocita', con riferimento a zero
        if(sMotCtrl_In.psPoOut->sDemand.slSpeed == 0L)
        {   // rampa terminata: tengo attivo il quickstop del posizionatore                        
            sMotCtrl_OStatus.b.bQuickStop = TRUE ;
            bRetVal = TRUE ;
        }
        else
            bRetVal = FALSE ; // sono in rampa 
    }     
 
    return bRetVal ;    
}           


//***************************************************************************
// reset fault

static void FaultsReset(void)
{
    SysLogMgm_ClearAlarms(TRUE);
    sMotCtrlRFlags.b.bAlarmClearRequest=TRUE;
    sMotCtrlRFlags.b.bFaultReactActivated=FALSE;
    sMotCtrlRFlags.b.bFaultReaction=FALSE;
}

//***************************************************************************
// External fault reset and fault_active to switch_on_disabled transition

static void ExternalFaultReset(void)
{
    if(sMotCtrlRun.uwStsDevCntrlIdx==LOCALSTATUS_FAULT)
    {
        FaultsReset();
#ifdef _INFINEON_
        _atomic_( 0 );
        sMotCtrlRun.uwForcedNextStsDevCntrl = LOCALSTATUS_SWITCHONDISABLED ;
        sMotCtrlRun.uwLocalFlags2Use|=LF_FORCESTATUS;
        _endatomic_();
#else
        sMotCtrlRun.uwForcedNextStsDevCntrl = LOCALSTATUS_SWITCHONDISABLED ;
        sMotCtrlRun.uwLocalFlags2Use|=LF_FORCESTATUS;
#endif
    }
}

//***************************************************************************
// Get status switch selected from parameter

static BOOL HmGetSwitchStatus(UBYTE ubSelector, UBYTE ubSoftSwitchStatus)
{
    BOOL bStat = FALSE ;

    if(ubSelector==MOTCTRL_SWITCHSEL_SOFT)
        return (BOOL)ubSoftSwitchStatus;

    else if(ubSelector==MOTCTRL_SWITCHSEL_OFF)
        return FALSE;

    else
    {
//#ifdef _INFINEON_
        bStat=sUsrIOCPBase.sDIO.uwInputs&(1<<(ubSelector&MOTCTRL_SWITCHSEL_INMASK)); // need IO.h
//#endif
        if(ubSelector&MOTCTRL_SWITCHSEL_INVERT)
            return !bStat;
        else
            return  bStat;
    }
}

//***************************************************************************
// Live parameters check and activation

static BOOL LiveParametersCheck(void)
{
    SQWRD sqMax, sqMin ;
    SLONG slVel;
    SWORD swTmp;
    BOOL bValid=TRUE,bIPValid=TRUE,bHMValid=TRUE;
    UBYTE ubUnits;
    SBYTE sbIndex;
    UBYTE ubHmTmp;

    // Following Error Limit (64bit)
    atomic_read(&sqMax, &sMotCtrlParameters.sqFollowingErrWindow, sizeof(SQWRD)) ;
    if(_sint64_comp(&sqMax, &sqZero64bit)<0)
    {
        atomic_write(&sMotCtrlRun.sqFollowingErrLimMax, &sqZero64bit, sizeof(SQWRD)) ;
        atomic_write(&sMotCtrlRun.sqFollowingErrLimMin, &sqZero64bit, sizeof(SQWRD)) ;
    }
    else
    {  
        _sint64_sub(&sqMin, &sqZero64bit, &sqMax) ;
        atomic_write(&sMotCtrlRun.sqFollowingErrLimMax, &sqMax, sizeof(SQWRD)) ;
        atomic_write(&sMotCtrlRun.sqFollowingErrLimMin, &sqMin, sizeof(SQWRD)) ;  
    }

    // Following Error timeout
    swTmp=sMotCtrlParameters.swFollowingErrTimeout;
    if(swTmp<0)
        swTmp=0;
    sMotCtrlRun.swFollowingErrTimeout=swTmp;
      
    // Target Position Window Limit (64bit)
    atomic_read(&sqMax, &sMotCtrlParameters.sqPositionWindow, sizeof(SQWRD)) ;
    if(_sint64_comp(&sqMax, &sqZero64bit)<0)
    {
        atomic_write(&sMotCtrlRun.sqPosWindowLimitMax, &sqZero64bit, sizeof(SQWRD)) ;
        atomic_write(&sMotCtrlRun.sqPosWindowLimitMin, &sqZero64bit, sizeof(SQWRD)) ;
    }
    else
    {
        _sint64_sub(&sqMin, &sqZero64bit, &sqMax) ;
        atomic_write(&sMotCtrlRun.sqPosWindowLimitMax, &sqMax, sizeof(SQWRD)) ;
        atomic_write(&sMotCtrlRun.sqPosWindowLimitMin, &sqMin, sizeof(SQWRD)) ;
    }

    // Target Position timeout
    swTmp=sMotCtrlParameters.swPositionWinTimeout;
    if(swTmp<0)
        swTmp=0;
    sMotCtrlRun.swPositionWinTimeout=swTmp;
      
    // velocity window
    atomic_read(&slVel, &sMotCtrlParameters.slVelocityWindow, sizeof(SLONG)) ;
    if(slVel<0)
        slVel=0;
    atomic_write(&sMotCtrlRun.slVelocityWindow, &slVel, sizeof(SLONG)) ;
      
    // velocity window timeout
    swTmp=sMotCtrlParameters.swVelocityWinTimeout;
    if(swTmp<0)
        swTmp=0;
    sMotCtrlRun.swVelocityWinTimeout=swTmp;
      
    // velocity threshold
    atomic_read(&slVel, &sMotCtrlParameters.slVelocityThreshold, sizeof(SLONG)) ;
    if(slVel<0)
        slVel=0;
    atomic_write(&sMotCtrlRun.slVelocityThreshold, &slVel, sizeof(SLONG)) ;
      
    // velocity threshold timeout
    swTmp=sMotCtrlParameters.swVelocityThrTimeout;
    if(swTmp<0)
        swTmp=0;
    sMotCtrlRun.swVelocityThrTimeout=swTmp;
      
    // quickstop option code
    swTmp=sMotCtrlParameters.swOptQuickStop;
    switch(swTmp)
    {
        case MOTCTRL_402_OC_POWEROFF:
        case MOTCTRL_402_OC_SLOWDOWN:
        case MOTCTRL_402_OC_QUICKSTOP:
        case MOTCTRL_402_OC_SLOWDOWNANDSTAY:
        case MOTCTRL_402_OC_QUICKSTOPANDSTAY:
            sMotCtrlRun.swOptQuickStop=swTmp;
            ParChk_ResetValueError(PARCC_MOTCTRL_QUICKSTOP);
            break;

        default:
            ParChk_SignalValueError(PARCC_MOTCTRL_QUICKSTOP);
            bValid=FALSE;
            break;
    }

    // shutdown option code
    swTmp=sMotCtrlParameters.swOptShutdown;
    switch(swTmp)
    {
        case MOTCTRL_402_OC_POWEROFF:
        case MOTCTRL_402_OC_SLOWDOWN:
            sMotCtrlRun.swOptShutdown=swTmp;
            ParChk_ResetValueError(PARCC_MOTCTRL_SHUTDOWN);
            break;

        default:
            ParChk_SignalValueError(PARCC_MOTCTRL_SHUTDOWN);
            bValid=FALSE;
            break;
    }

    // disable operation option code
    swTmp=sMotCtrlParameters.swOptDisableOp;
    switch(swTmp)
    {
        case MOTCTRL_402_OC_POWEROFF:
        case MOTCTRL_402_OC_SLOWDOWN:
            sMotCtrlRun.swOptDisableOp=swTmp;
            ParChk_ResetValueError(PARCC_MOTCTRL_DISABLEOP);
            break;

        default:
            ParChk_SignalValueError(PARCC_MOTCTRL_DISABLEOP);
            bValid=FALSE;
            break;
    }

    // halt option code
    swTmp=sMotCtrlParameters.swOptHalt;
    switch(swTmp)
    {
        case MOTCTRL_402_OC_SLOWDOWN:
        case MOTCTRL_402_OC_QUICKSTOP:
            sMotCtrlRun.swOptHalt=swTmp;
            ParChk_ResetValueError(PARCC_MOTCTRL_HALT);
            break;

        default:
            ParChk_SignalValueError(PARCC_MOTCTRL_HALT);
            bValid=FALSE;
            break;
    }

    // fault reaction option code
    swTmp=sMotCtrlParameters.swOptFaultReaction;
    switch(swTmp)
    {
        case MOTCTRL_402_OC_POWEROFF:
        case MOTCTRL_402_OC_SLOWDOWN:
        case MOTCTRL_402_OC_QUICKSTOP:
            sMotCtrlRun.swOptFaultReaction=swTmp;
            ParChk_ResetValueError(PARCC_MOTCTRL_FAULTREACTION);
            break;

        default:
            ParChk_SignalValueError(PARCC_MOTCTRL_FAULTREACTION);
            bValid=FALSE;
            break;
    }

    // interpolation time period
    ubUnits=sMotCtrlParameters.ubIPTimeUnits;
    sbIndex=sMotCtrlParameters.sbIPTimeIndex;
    if(ubUnits<1)
    {
        ParChk_SignalValueError(PARCC_MOTCTRL_IPTIMINGUNITS);
        bValid=bIPValid=FALSE;
    }

    if(sbIndex<-6 || sbIndex>=0)
    {
        ParChk_SignalValueError(PARCC_MOTCTRL_IPTIMINGINDEX);
        bValid=bIPValid=FALSE;
    }

    if(bIPValid)
    {
        ULONG ulTime=(ULONG)ubUnits;
        ULONG ulCycles;

        for(sbIndex+=6;sbIndex>0;sbIndex--)
            ulTime*=10;

        ulCycles=ulTime/(1000000/REALTIME_TASK_FREQ);

        if(ulCycles*(1000000/REALTIME_TASK_FREQ) != ulTime || ulCycles<IP_MIN_CYCLES || ulCycles>IP_MAX_CYCLES)
        {
            ParChk_SignalValueError(PARCC_MOTCTRL_IPTIMINGUNITS);
            ParChk_SignalValueError(PARCC_MOTCTRL_IPTIMINGINDEX);
            bValid=FALSE;
        }
        else
        {
            ParChk_ResetValueError(PARCC_MOTCTRL_IPTIMINGUNITS);
            ParChk_ResetValueError(PARCC_MOTCTRL_IPTIMINGINDEX);

            sMotCtrlRun.swIPNumCycle=(SWORD)ulCycles;
            sMotCtrlRun.swIPRecoveryNumCycle=-(SWORD)(ulCycles*IPMODE_RECOVERYFACTOR);
            slVel=(SLONG)(0x4000/sMotCtrlRun.swIPNumCycle)<<2;
            atomic_write(&sMotCtrlRun.slIPScaling, &slVel, sizeof(slVel));
        }
    }

    // homing mode related parameters
    ubHmTmp=sMotCtrlParameters.ubHomingMethod;
    bIPValid=FALSE;
    for(swTmp=0;swTmp<sizeof(sHmSeq)/sizeof(HM_SEQUENCER);swTmp++)
        if(ubHmTmp==sHmSeq[swTmp].ubMethod)
        {
            bIPValid=TRUE;
            sMotCtrlRun.puwHomingRun=sHmSeq[swTmp].uwSeq;
            ParChk_ResetValueError(PARCC_MOTCTRL_HMMETHOD);
            break;
        }

    if(!bIPValid)
    {
        ParChk_SignalValueError(PARCC_MOTCTRL_HMMETHOD);
        sMotCtrlRun.puwHomingRun=NULL;
        bValid=FALSE;
        bHMValid=FALSE;
    }

    ubHmTmp=sMotCtrlParameters.ubHomingSSrcPosSwitch;
    if(ubHmTmp==MOTCTRL_SWITCHSEL_SOFT || ubHmTmp==MOTCTRL_SWITCHSEL_OFF || (ubHmTmp&MOTCTRL_SWITCHSEL_INMASK)<16)
    {
        sMotCtrlRun.ubHomingSSrcPosSwitch=ubHmTmp;
        ParChk_ResetValueError(PARCC_MOTCTRL_HMPOSSWITCH);
    }
    else
    {
        ParChk_SignalValueError(PARCC_MOTCTRL_HMPOSSWITCH);
        bValid=FALSE;
        bHMValid=FALSE;
    }
 
    ubHmTmp=sMotCtrlParameters.ubHomingSSrcNegSwitch;
    if(ubHmTmp==MOTCTRL_SWITCHSEL_SOFT || ubHmTmp==MOTCTRL_SWITCHSEL_OFF || (ubHmTmp&MOTCTRL_SWITCHSEL_INMASK)<16)
    {
        sMotCtrlRun.ubHomingSSrcNegSwitch=ubHmTmp;
        ParChk_ResetValueError(PARCC_MOTCTRL_HMNEGSWITCH);
    }
    else
    {
        ParChk_SignalValueError(PARCC_MOTCTRL_HMNEGSWITCH);
        bValid=FALSE;
        bHMValid=FALSE;
    }
 
    ubHmTmp=sMotCtrlParameters.ubHomingSSrcHomeSwitch;
    if(ubHmTmp==MOTCTRL_SWITCHSEL_SOFT || ubHmTmp==MOTCTRL_SWITCHSEL_OFF || (ubHmTmp&MOTCTRL_SWITCHSEL_INMASK)<16)
    {
        sMotCtrlRun.ubHomingSSrcHomeSwitch=ubHmTmp;
        ParChk_ResetValueError(PARCC_MOTCTRL_HMHOMESWITCH);
    }
    else
    {
        ParChk_SignalValueError(PARCC_MOTCTRL_HMHOMESWITCH);
        bValid=FALSE;
        bHMValid=FALSE;
    }
 
    atomic_read(&slVel, &sMotCtrlParameters.slHomingAcceleration, sizeof(SLONG)) ;
    if(slVel<0)
        slVel=0;
    atomic_write(&sMotCtrlRun.slHomingAcceleration, &slVel, sizeof(SLONG)) ;
    
    sMotCtrlRFlags.b.bHomeParamError=!bHMValid;

    // current to rated torque factors calculation
    {
        FLOAT flC2T;
        SLONG slC2T,slT2C;

            // if rated torque specified
        if(sMotCtrlParameters.ulMotorRatedTorque>0l)
        {
                // [1/10mArms] / 10000 * [Nm/Arms] / ([mN]/1000) * 1000;
            flC2T=1.0/10000.0*sGlbMotorParameters.flKT/((FLOAT)sMotCtrlParameters.ulMotorRatedTorque/1000.0)*1000.0;

            if(flC2T>32767.0 || flC2T<1/32767.0)
            {
                ParChk_SignalValueError(PARCC_MOTCTRL_CSMOTORRATEDTORQUE);
                bValid=FALSE;
            }
            else
                ParChk_ResetValueError(PARCC_MOTCTRL_CSMOTORRATEDTORQUE);
        }
            // otherwise take peak current as rated torque
        else
        {
                // current peak must be always not zero
            if(sGlbMotorParameters.flCurrentPeak==0.0)
                flC2T=0.0001;
            else
                flC2T=sGlbMotorParameters.flCurrentPeak;

                // [1/10mArms] / [Arms] * 1000;
            flC2T=1.0/10000.0/flC2T*1000.0;
        }

            // setup ratios
        assert(flC2T!=0.0);
        slC2T=(SLONG)(1.0*flC2T*65536.0+0.5);
        slT2C=(SLONG)(1.0/flC2T*65536.0+0.5);

        atomic_write(&sMotCtrl_Out.slCSCurrentToTorque, &slC2T, sizeof(SLONG)) ;
        atomic_write(&sMotCtrl_Out.slCSTorqueToCurrent, &slT2C, sizeof(SLONG)) ;
    }

    return bValid;
}

//***************************************************************************
// position scaling between 64 and 32 bit

UWORD MotCtrl_AbsolutePositionScaling(BOOL bDirection, void * pvDst, void * pvSrc)
{
    UQWRD loc;
    UWORD *uwpTemp ; //workaround to avoid "error: lvalue required as increment operand"

    if(bDirection==COMMONPARAMDB_UCFLAG_READ)
    {
        _uint64_atomic_copy(&loc, &(((ENCMGR_SPACEFEEDBACK *)pvSrc)->sEncData.sqPostn));

        if(((ENCMGR_SPACEFEEDBACK *)pvSrc)->ubStatus & ENCMGR_ABSMECHTURN_VALID)
            _uint64_add(&loc, &loc, (UQWRD *)&(((ENCMGR_SPACEFEEDBACK *)pvSrc)->sqMechAbsPosOffset));

        _uint64_add(&loc, &loc, (UQWRD *)&sMotCtrl_Out.sqUserOffset);

        *((ULONG *)pvDst)=INT64_MLONG((*(UBYTE *)&loc));
    }
    else
    {
        INT64_WORD0(loc)=0;
       uwpTemp = (UWORD *)pvSrc ;
       INT64_WORD1(loc)= *uwpTemp++ ;
       INT64_WORD2(loc)= *uwpTemp ;
//        INT64_WORD1(loc)=*((UWORD *)pvSrc)++;
//        INT64_WORD2(loc)=*((UWORD *)pvSrc)++;
            // sign extension
        if((SWORD)INT64_WORD2(loc)<0)
            INT64_WORD3(loc)=-1;
        else
            INT64_WORD3(loc)=0;

        if(((ENCMGR_SPACEFEEDBACK *)pvDst)->ubStatus & ENCMGR_ABSMECHTURN_VALID)
            _uint64_sub(&loc, &loc, (UQWRD *)&(((ENCMGR_SPACEFEEDBACK *)pvDst)->sqMechAbsPosOffset));

        _uint64_sub(&loc, &loc, (UQWRD *)&sMotCtrl_Out.sqUserOffset);

        _uint64_atomic_copy(&(((ENCMGR_SPACEFEEDBACK *)pvDst)->sEncData.sqPostn), &loc);
    }
    return 0;
}

//***************************************************************************
// position scaling between 64 and 32 bit

UWORD MotCtrl_RelativePositionScaling(BOOL bDirection, void * pvDst, void * pvSrc)
{
	UQWRD loc;
    UWORD *uwpTemp ; //workaround to avoid "error: lvalue required as increment operand"

    if(bDirection==COMMONPARAMDB_UCFLAG_READ)
    {
        _uint64_atomic_copy(&loc, pvSrc);

        *((ULONG *)pvDst)=INT64_MLONG((*(UBYTE *)&loc));
    }
    else
    {
        INT64_WORD0(loc)=0;
        uwpTemp = (UWORD *)pvSrc ;
        INT64_WORD1(loc)= *uwpTemp++ ;
        INT64_WORD2(loc)= *uwpTemp ;
//        INT64_WORD1(loc)=*((UWORD *)pvSrc)++;
//        INT64_WORD2(loc)=*((UWORD *)pvSrc)++;
            // sign extension
        if((SWORD)INT64_WORD2(loc)<0)
            INT64_WORD3(loc)=-1;
        else
            INT64_WORD3(loc)=0;

        _uint64_atomic_copy(pvDst, &loc);
    }
    return 0;
}

//***************************************************************************
// current scaling to 1/1000 rated torque

UWORD MotCtrl_RatedTorqueScaling(BOOL bDirection, void * pvDst, void * pvSrc)
{
    SLONG slVal;

    if(bDirection==COMMONPARAMDB_UCFLAG_READ)
        slVal=_sint32_scale_32(*(SLONG *)pvSrc,sMotCtrl_Out.slCSCurrentToTorque);
    else
        slVal=_sint32_scale_32(*(SLONG *)pvSrc,sMotCtrl_Out.slCSTorqueToCurrent);

   *(SLONG *)pvDst=slVal;

    return 0;
}

//***************************************************************************
// Setup specific PLC options

BOOL MotCtrl_SetPlcOption(UWORD uwOption, ULONG ulValue)
{
    switch(uwOption)
    {
        case MOTCTRL_IGNORE_ENCSTAT:
            if(ulValue==MOTCTRL_IGNORE_ENCSTAT_KEY && bSysStatBooting)
                sMotCtrlRFlags.b.bIgnoreEncStatus=TRUE;
            return TRUE;

        default:
            return FALSE;
    }
}

