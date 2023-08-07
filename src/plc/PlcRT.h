/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright $)AB) 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : PlcRT.h                                                    */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Configuration of PLC and declaration of its resources      */
/*                                                                          */
/****************************************************************************/

#ifndef _PLCRT_H
#define _PLCRT_H

#include "common\CommonDefines.h"
#include "AlPlcRuntime2\AlPlcCDefs.h"
#include "AlPlcRuntime2\AlPlcTArg.h"

//#include "Defineexternals.h"

//****************************************************************************
// Data structures

typedef struct
{
	bool_t (* pfRun)(void);
    void (* pfInit)(void);
    int16_t (* pfInput)(int16_t);
    int16_t (* pfOutput)(int16_t);
    bool_t bReqInit;
    bool_t bTaskValid;
} PLC_TASKDATA;

typedef struct
{
    ULONG ulInfoAreaAddress;
    ULONG ulDownloadAddress;
    ULONG ulCodeSize;
    UWORD uwMemoID;
    UWORD uwFullMemoID;
} PLC_BUILDINFO;

//****************************************************************************
// Sizing

#ifdef _INFINEON_
extern uint8_t PLC_LK_CODE_START;
extern uint8_t PLC_LK_CODE_SIZE;
extern uint8_t PLCD_LK_CODE_START;
extern uint8_t PLCD_LK_CODE_SIZE;

#define PLC_RT_CODE_START               ((uint32_t)(HPVOID)&PLC_LK_CODE_START)
#define PLC_RT_CODE_SIZE                ((uint32_t)(HPVOID)&PLC_LK_CODE_SIZE)
#define PLCD_RT_CODE_START              ((uint32_t)(HPVOID)&PLCD_LK_CODE_START)
#define PLCD_RT_CODE_SIZE               ((uint32_t)(HPVOID)&PLCD_LK_CODE_SIZE)
#else
#include "system\SysAppGlobals.h"

#define PLC_RT_CODE_START               ((uint32_t)(HPVOID)PLC_LK_CODE_START)
#define PLC_RT_CODE_SIZE                ((uint32_t)(HPVOID)PLC_LK_CODE_SIZE)
#define PLCD_RT_CODE_START              ((uint32_t)(HPVOID)PLCD_LK_CODE_START)
#define PLCD_RT_CODE_SIZE               ((uint32_t)(HPVOID)PLCD_LK_CODE_SIZE)
#endif

//****************************************************************************
// Auto vars sizing

#ifndef _APP_XC
    #define PLC_AUTO_VARS_SIZE          ( 0x600 )
    #define PLC_AUTO_BITS_SIZE          ( 0x200 )

    #define PLC_AVSEG                   near
#else
//    #define PLC_AUTO_VARS_SIZE          ( 0x1200 )
//    #define PLC_AUTO_BITS_SIZE          ( 0x300 )
	#define PLC_AUTO_VARS_SIZE          ( 0x1500 )
	#define PLC_AUTO_BITS_SIZE          ( 0x400 )

    #define PLC_AVSEG
#endif                                

//****************************************************************************
// Globals

extern PLC_TASKDATA                     sPlcTaskFast;
extern PLC_TASKDATA                     sPlcTaskSlow;
extern PLC_TASKDATA                     sPlcTaskBackground;
extern PLC_TASKDATA                     sPlcTaskBootConfig;
extern PLC_TASKDATA                     sPlcTaskBootInit;
extern PLC_TASKDATA                     sPlcTaskRTHook;
extern PLC_TASKDATA                     sPlcTaskSystem;
extern PLC_TASKDATA                     sPlcTaskDSPHook;

    // Auto vars data area
extern uint8_t PLC_AVSEG                sPlcAutoVars[ PLC_AUTO_VARS_SIZE ];

    // Auto bit vars data area
extern uint8_t                          sPlcAutoBits[ PLC_AUTO_BITS_SIZE / 8 ];

    // Control registers
extern uint32_t                         ulPlcRegInfoArea;
extern uint32_t                         ulPlcRegDownloadAddress;
extern uint32_t                         ulPlcRegApplID;
extern uint16_t                         uwPlcRegCommand;
extern uint16_t                         uwPlcRegDownloadResponse;
extern uint32_t                         ulPlcRegMemoID;
extern uint32_t                         ulPlcRegFullMemoID;
extern uint32_t                         ulPlcRegCommandResponse;

    // Control variables
extern bool_t                           bPlcProgramOk;
extern uint16_t                         uwPlcErrorCode;
extern uint32_t                         ulPlcFileSize;

    // Info structure
extern CIMG_HEAD_INFO                   sPlcInfoHeader;

    // Build Tools Info Area for Diagnostic PLC
extern const PLC_BUILDINFO              sPlcDiagBuildInfo;

    // Call-backs
bool_t PlcStandByRequired(void);
void PlcEndOfLoad(void);
void PlcInitDwSourceCode(void);
void * PlcGetSourceCodeAreaHeader(void);
uint32_t PlcChecksumSourceCode(void);
uint16_t PlcGetDiagMemoryID(void);
uint16_t PlcGetDiagFullMemoryID(void);

#endif
