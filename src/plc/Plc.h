/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Plc.h                                                      */
/* Author      : Fabio Terrile                                              */
/*               Axel                                                       */
/*                                                                          */
/* Description : Configuration of PLC and declaration of its resources      */
/*                                                                          */
/****************************************************************************/

#ifndef _PLC_H
#define _PLC_H

#include "common\CommonDefines.h"
#include "common\CommonController.h"
#include "AlPlcRuntime2\AlPlcCDefs.h"

//#include "Defineexternals.h"

//****************************************************************************
// User param/works sizing

#ifndef _APP_XC
    #define PLC_USRPARAM_DIM_COM        1280        // size in bytes
    #define PLC_USRPARAM_DIM_BIT        128         // size in bits
    
    #define PLC_USRWORKS_DIM_COM        1344        // size in bytes
    #define PLC_USRWORKS_DIM_BIT        128         // size in bits

    #define PLC_UDSEG

#else
    #define PLC_USRPARAM_DIM_COM        2304        // size in bytes
    #define PLC_USRPARAM_DIM_BIT        256         // size in bits
    
    #define PLC_USRWORKS_DIM_COM        2304        // size in bytes
    #define PLC_USRWORKS_DIM_BIT        256         // size in bits

    #define PLC_UDSEG
#endif                                

//****************************************************************************
// Defines

#define PLC_INIT_CORE                   0
#define PLC_INIT_USRPARINSTALL          1
#define PLC_INIT_BOOTCONFIG             2
#define PLC_INIT_TASKS                  3
#define PLC_INIT_RTHOOK                 4
#define PLC_INIT_FAST                   5
#define PLC_INIT_OTHERS                 6

#ifdef _APP_PLCDBGWORKS
#define PLC_DEBUG_DIM                   64          // size in bytes
#endif

#define PLC_DSPDB_DIM_COM               24          // size in bytes

#define PLC_APP_NAME_SIZE               (16)        // multiple of sizeof(ULONG)

#define PLC_CUSTOMCALIB_DIM             32

#define PLC_SLOWTASK_PERIOD_DEFAULT     8           // msec
#define PLC_SLOWTASK_PERIOD_MIN         1           // msec
#define PLC_SLOWTASK_PERIOD_MAX         64          // msec

#define PLC_PROFILER_TIMER100ns         1
#define PLC_PROFILER_TIMER125us         2
#define PLC_PROFILER_TIMER1ms           3

//****************************************************************************
// Data structures

typedef struct
{
    union
    {
        struct 
        {
            UWORD bDisablePlc               :1;
        } b ;
        UWORD w ;
    } flags ;

    UWORD uwDummy;
} PLC_PARAMS;

typedef struct
{
    UWORD uwCommon[PLC_USRPARAM_DIM_COM/sizeof(UWORD)];
} PLC_USRPARAM_COM;

typedef struct
{
    UWORD uwBools[PLC_USRPARAM_DIM_BIT/sizeof(UWORD)];
} PLC_USRPARAM_BITS;

typedef struct
{
        // CRC of parameters definition area, saved with parameters in order
        // to determine at bootup if area could be restored (same definition)
        // or has to be zeroed (changed definition)
    UWORD uwParDefCRC;
        // name of application that saved the parameters, if different application
        // then no chance, parameters will be zeroed
    CHARS   chAppName[PLC_APP_NAME_SIZE];
} PLC_USRPARAM_ID;

typedef struct
{
    CHARS   chAppName[PLC_APP_NAME_SIZE];
    UWORD   uwMajor;
    UWORD   uwMinor;
    ULONG   ulBuild;
} PLC_APP_INFO;

typedef struct
{
    const COMCTRL_HANDLERS *psCtrlHandlers ;
} PLC_IN ;

typedef struct {
    BOOL b[3];
} PLC_USERCFGFLAGS;

//****************************************************************************
// Globals

extern PLC_PARAMS                  		sPlcParams;
extern PLC_USRPARAM_ID             		sPlcUsrParamsID;

extern PLC_USERCFGFLAGS            		sPlcUserCfgFlags;
extern UBYTE                       		bPlcSysCommands;

extern PLC_USRPARAM_COM PLC_UDSEG  		tPlcParamCommon;
extern PLC_USRPARAM_BITS           		tPlcParamBits;

extern UWORD PLC_UDSEG             		uwPlcWorksCommon[PLC_USRWORKS_DIM_COM/sizeof(UWORD)];
extern UWORD                       		uwPlcWorksBools[PLC_USRWORKS_DIM_BIT/sizeof(UWORD)];

#ifdef _APP_PLCDBGWORKS
extern UWORD                       		uwPlcDebugCommon[PLC_DEBUG_DIM/sizeof(UWORD)];
#endif

extern UBYTE                       		ubPlcDSPDBWorks[PLC_DSPDB_DIM_COM];

extern UWORD                            uwPlcSlowTaskTime;  // [1/10msec]
extern UWORD                            uwPlcSlowTaskPeriod;
extern BOOL                             bPlcEnableSysHMIDevelopment;

extern PLC_APP_INFO                     sPlcAppInfo;

extern PLC_IN                           sPlcIn;

extern void (* sPlcDSPHookEntryPoint)(void);

//****************************************************************************
// Global functions

BOOL PlcInit(UWORD uwTaskParam);
void PlcRTOvertime(void);
BOOL PlcRequestStandby(void);
BOOL PlcLockForReload(BOOL bKeepComDBApp);
BOOL PlcIsInSlowTask(void);
void * PlcIECDBEntrySearch(UBYTE ubDbType, UWORD uwDbIdx, UWORD uwDbOffset);
void PlcPostUserAlarm(ULONG ulAlarmSubCode, SBYTE bStore);
void PlcPostEmcyStop(UWORD uwOption);
void PlcCheckAddressDwSourceCode(ULONG ulAddress, UWORD uwLength);
UWORD PlcProfilerBegin(const UBYTE ubTimerSel);
UWORD PlcProfilerEnd(const UBYTE ubTimerSel, UWORD uwStart);

#ifdef _APP_XC
SWORD PlcStreamCodeBegin(void);
SWORD PlcStreamCodeData(HPUBYTE pubBuffer, UWORD uwSize);
SWORD PlcStreamCodeEnd(void);
#endif

#endif
