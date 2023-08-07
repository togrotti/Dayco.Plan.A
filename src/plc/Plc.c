/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright $)AB) 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Plc.c                                                      */
/* Author      : Fabio Terrile,Hu Xiaokai                                   */
/*               Axel                                                       */
/*                                                                          */
/* Description : Configuration of PLC and declaration of its resources      */
/*                                                                          */
/****************************************************************************/
#pragma GCC optimize (2)

#include <string.h>

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "common\Int64Functions.h"
#include "drive\AxM-E-Defines.h"
#include "system\SysAppGlobals.h"

#include "common\TaskScheduler.h"
#include "system\SystemStatus.h"
#include "system\SysAppInterruptLevels.h"
#include "system\SystemAlarms.h"
#include "system\SysLogManagement.h"
#include "system\GlobalResetCodes.h"
#include "core\SystemReset.h"
#include "fpga\FpgaHandler.h"
#include "system\SysAppSFlashPartition.h"
#include "common\SerialFlashHandler.h"
#ifdef _APP_XC
#include "common\ProgramFlashHandler.h"
#include "common\SecurityCrypto.h"
#endif

#include "PlcComDBMgr.h"
//#define DEFINE_EXTERNALS
#include "Plc.h"
#include "PlcRT.h"
//#undef  DEFINE_EXTERNALS

#include "AlPlcRuntime2\AlPlcAreaDef.h"
#include "AlPlcRuntime2\AlPlcRuntimeCore.h"
#include "AlPlcRuntime2\AlPlcUserData.h"

//****************************************************************************
// Defines

#define DATABLOCK_IMAGE_MAX_INDEX       16

//****************************************************************************
// Slow task period selector

#define SLOWTASK_MAX_ALLOWED_RUNTIME    8           // msec

#define SLOWTASK_MAX_PEAK_SLOTS         4

#define SLOWTASK_INTERRUPT_VECTOR       0x19        // CC2_CC25INT

#define SLOWTASK_DIVIDER                8           // 1msec/125usec

#define PLCOK                                       // when the PLC do not work well please disable it

//****************************************************************************
// Globals

UWORD uwPlcSlowTaskPeriod=PLC_SLOWTASK_PERIOD_DEFAULT;
BOOL bPlcEnableSysHMIDevelopment=FALSE;
PLC_PARAMS     sPlcParams; // new added

// Plc.h
PLC_USRPARAM_ID                  sPlcUsrParamsID;

PLC_USRPARAM_COM PLC_UDSEG       tPlcParamCommon;
PLC_USRPARAM_BITS                tPlcParamBits;

UWORD PLC_UDSEG                  uwPlcWorksCommon[PLC_USRWORKS_DIM_COM/sizeof(UWORD)];
UWORD                            uwPlcWorksBools[PLC_USRWORKS_DIM_BIT/sizeof(UWORD)];

#ifdef _APP_PLCDBGWORKS
UWORD                            uwPlcDebugCommon[PLC_DEBUG_DIM/sizeof(UWORD)];
#endif

UBYTE                            ubPlcDSPDBWorks[PLC_DSPDB_DIM_COM];

UWORD                            uwPlcSlowTaskTime;  // [1/10msec]


PLC_APP_INFO                     sPlcAppInfo;

PLC_IN                           sPlcIn;

void (* sPlcDSPHookEntryPoint)(void);

// PlcRT.h
PLC_TASKDATA                     sPlcTaskFast;
PLC_TASKDATA                     sPlcTaskSlow;
PLC_TASKDATA                     sPlcTaskBackground;
PLC_TASKDATA                     sPlcTaskBootConfig;
PLC_TASKDATA                     sPlcTaskBootInit;
PLC_TASKDATA                     sPlcTaskRTHook;
PLC_TASKDATA                     sPlcTaskSystem;
PLC_TASKDATA                     sPlcTaskDSPHook;

    // Auto vars data area
uint8_t PLC_AVSEG                sPlcAutoVars[ PLC_AUTO_VARS_SIZE ];

    // Auto bit vars data area
uint8_t                          sPlcAutoBits[ PLC_AUTO_BITS_SIZE / 8 ];

    // Control registers
uint32_t                         ulPlcRegInfoArea;
uint32_t                         ulPlcRegDownloadAddress;
uint32_t                         ulPlcRegApplID;
uint16_t                         uwPlcRegCommand;
uint16_t                         uwPlcRegDownloadResponse;
uint32_t                         ulPlcRegMemoID;
uint32_t                         ulPlcRegFullMemoID;
uint32_t                         ulPlcRegCommandResponse;

    // Control variables
bool_t                           bPlcProgramOk;
uint16_t                         uwPlcErrorCode;
uint32_t                         ulPlcFileSize;

    // Info structure
CIMG_HEAD_INFO                   sPlcInfoHeader;

    // Build Tools Info Area for Diagnostic PLC
//$sm$ const PLC_BUILDINFO              sPlcDiagBuildInfo;

    // user options
PLC_USERCFGFLAGS                   sPlcUserCfgFlags;

#define bSlowTaskCopyOutImmediate  sPlcUserCfgFlags.b[0]
#define bDisablePwrOnRTOverTime    sPlcUserCfgFlags.b[1]
#define bEnableSysHMIDownload      sPlcUserCfgFlags.b[2]

    // system commands
UBYTE                            bPlcSysCommands;
#define bRebootMinimalRequested    HBITW(bPlcSysCommands)->bit0

//****************************************************************************
// Locals

    // Stati del PLC
static BOOL    bProgramValid=FALSE;
static BOOL    bSlowTerminated=FALSE;
static BOOL    bStandbyRequested=FALSE;
static BOOL    bUserParamValid=FALSE;

    // shadow user options, read on boot and kept unchanged
static BOOL    bShadowDisablePwrOnRTOverTime;

    // init multiimage copy for background task
static BOOL    bBackgroundInInit;
static BOOL    bBackgroundOutInit;
static UWORD   uwBackgroundInitCnt;

    // background task handle
static OS_TASKHANDLE hBackgroundHandle;

    // valid block list for background task image copy
static UWORD   uwBackgroundImageListIn[DATABLOCK_IMAGE_MAX_INDEX+1];
static UWORD   uwBackgroundImageListOut[DATABLOCK_IMAGE_MAX_INDEX+1];
static UWORD * puwBackgroundImageStatus;

    // slow task frequency selector
static UWORD   uwSlowTaskCntReload;
static UWORD   uwSlowTaskCounter;
static UWORD   uwSlowTaskMaxAllowed;
static UWORD   uwSlowTaskOvertimeSlotLeft;

    // reset management
static BOOL    bResetRequested=FALSE;
static UWORD   uwResetDelayTimer;

    // source code area download header
static CIMG_SOURCE_HEAD  sSrcCodeHeader;

    // source code serial flash erase sector map
static UQWRD  uqSrcCodeEraseMap={0ul,0ul};

    // status and buffer for binary stream
#ifdef _APP_XC
static ULONG  ulStrAddress;
static UBYTE  ubStrBuffer[PROGRAMFLASH_PAGE_SIZE];
#endif

//****************************************************************************
// Image copy wrappers

void PlcSlowTaskMgrIn(void);
void PlcSlowTaskMgrOut(void);
BOOL PlcImageBackgroundIn(void);
BOOL PlcImageBackgroundOut(void);

//****************************************************************************
// Local prototypes

static void backgroundloop(void);
static void resetrequestcheck(void);
static void slowtaskhandler(void);

//****************************************************************************
// Initialization

BOOL PlcInit(UWORD uwTaskParam)
{
#ifdef _APP_DEBUG
    UWORD cnt;
#endif

    switch(uwTaskParam)
    {
            // initialization phase, safe mode
        case PLC_INIT_CORE:
#ifdef _APP_DEBUG
                // check tables size
            assert(plcCntTasksFunctions==PLC_NUM_TASKS);
            assert(plcCntTasksDefs==PLC_NUM_TASKS);
            assert(plcCntDataBlocks==PLC_NUM_DB);
            assert(plcCntEmbeddedFunctions==PLC_NUM_FUNCTS);

                // table check
            for(cnt=0;cnt<PLC_NUM_DB;cnt++)
            {
                assert(plcDataBlocks[cnt].img>=0 && plcDataBlocks[cnt].img<DATABLOCK_IMAGE_MAX_INDEX);
                if(cnt>0)
                    assert(plcDataBlocks[cnt-1].dbId<plcDataBlocks[cnt].dbId);
            }
#endif

#ifdef _INFINEON_
                // configure CC25 for slow task, used only to trigger a task with right priority level
            CC2_M6      = (CC2_M6 & 0xff0f) | 0x00c0;
            CC2_CC25IC  = SYSAPPIL_PLC_SLOWTASK_CC25;
#else
            Timer_CCSet(TIMER_CCU2_ID, SYSAPPIL_PLC_SLOWTASK, slowtaskhandler);
#endif
                // init handle
            hBackgroundHandle=NULL;

                // init info structure
            strcpy(sPlcAppInfo.chAppName,"(none)");
            sPlcAppInfo.uwMajor=0;
            sPlcAppInfo.uwMinor=0;
            sPlcAppInfo.ulBuild=0l;

                // zeroes error register for com db
            ubPlcComDbErrCode=0;

                // init DSP Hook entry point
            sPlcDSPHookEntryPoint=NULL;

#if CFG_PLC_DIAGNOSTIC
                // check for hidden diagnostic application activation requested
            if(uwSystemEnterDiagnostic==SYSTEM_ENTERDIAGNOSTIC_KEY)
            {
                    // Setup code area definition for plc common DB - diagnostic
                ulPlcComParCodeStart=PLCD_RT_CODE_START;
                ulPlcComParCodeSize =PLCD_RT_CODE_SIZE;

                    // Init PLC Area
                InitPlcDiagRuntime();

                    // Load PLC Area
                LoadPlcDiag();

                    // No runtime manager for diagnostic application
            }
            else
#endif
            {
                    // Setup code area definition for plc common DB - user app
                ulPlcComParCodeStart=PLC_RT_CODE_START;
                ulPlcComParCodeSize =PLC_RT_CODE_SIZE;

                    // Init PLC Area
                InitPlcRuntime();

                    // Load PLC Area
                LoadPlc();

                    // Add runtime manager to background tasks
                assert(TaskSched_AddBackgroundTask(&ManagePLCState));
            }

                // Add reset check to background tasks
            assert(TaskSched_AddBackgroundTask(&resetrequestcheck));

#ifdef PLCOK
                // program valid if program checked, then if user enable or diagnostic app
             bProgramValid=bPlcProgramOk && (!sPlcParams.flags.b.bDisablePlc || uwSystemEnterDiagnostic==SYSTEM_ENTERDIAGNOSTIC_KEY);
#else
             bProgramValid = FALSE;
#endif
            break;

        case PLC_INIT_USRPARINSTALL:
                // if valid, enabled and task valid
            if(bProgramValid)
            {
                if(sPlcTaskSystem.bTaskValid)
                {
                        // then run the hidden system task that's used
                        // to setup user parameters for PLC
                    (*sPlcTaskSystem.pfInit)();
                    (*sPlcTaskSystem.pfRun)();
                }
                bUserParamValid=TRUE;

                    // set app info
                memcpy(sPlcAppInfo.chAppName, sPlcInfoHeader.strApp,
                        sizeof(sPlcInfoHeader.strApp)<sizeof(sPlcAppInfo.chAppName)?sizeof(sPlcInfoHeader.strApp):sizeof(sPlcAppInfo.chAppName));
                sPlcAppInfo.uwMajor=sPlcInfoHeader.appVer1;
                sPlcAppInfo.uwMinor=sPlcInfoHeader.appVer2;
                sPlcAppInfo.ulBuild=sPlcInfoHeader.buildTime;
            }

                // check return code from com db set handler
            if(ubPlcComDbErrCode>0)
                    // if error then disable execution, alarm will be sent at the end of boot
                bProgramValid=bUserParamValid=FALSE;

            else if(bUserParamValid)
            {
                PLC_USRPARAM_ID sLocUsrParID;

                    // erase local structure as allocated in stack
                memset(&sLocUsrParID, 0, sizeof(sLocUsrParID));

                    // create param ID identifier
                memcpy(sLocUsrParID.chAppName, sPlcInfoHeader.strApp,
                        sizeof(sPlcInfoHeader.strApp)<sizeof(sLocUsrParID.chAppName)?sizeof(sPlcInfoHeader.strApp):sizeof(sLocUsrParID.chAppName));
                sLocUsrParID.uwParDefCRC=uwPlcComParDefCRC;

                    // if different check for usr parameter appliance
                if(memcmp(&sLocUsrParID, &sPlcUsrParamsID, sizeof(PLC_USRPARAM_ID)))
                {
                        // if appname is different then usr parameters will voided
                    if(memcmp(sLocUsrParID.chAppName, sPlcUsrParamsID.chAppName, sizeof(sPlcUsrParamsID.chAppName)))
                    {
                        memset(&tPlcParamCommon,0,sizeof(tPlcParamCommon));
                        memset(&tPlcParamBits,0,sizeof(tPlcParamBits));
                    }
                        // if same name try to recover parameters
                    else
                    {
                        ubPlcComDbErrCode=PlcComDBAppRemapPar(tPlcParamCommon.uwCommon, sizeof(tPlcParamCommon.uwCommon)/sizeof(UWORD),
                                            tPlcParamBits.uwBools, sizeof(tPlcParamBits.uwBools)/sizeof(UWORD));

                            // if inconstencies while remapping disable execution
                        if(ubPlcComDbErrCode>0)
                            bProgramValid=FALSE;
                    }

                        // if no error then apply new usr params ID
                    if(ubPlcComDbErrCode==0)
                        memcpy(&sPlcUsrParamsID, &sLocUsrParID, sizeof(PLC_USRPARAM_ID));
                }
            }
            break;

            // Boot config initialization phase
        case PLC_INIT_BOOTCONFIG:
                // if valid PLC
            if(bProgramValid)
                    // if preboot task valid
                if(sPlcTaskBootConfig.bTaskValid)
                {
                        // initialize boot config task
                    (*sPlcTaskBootConfig.pfInit)();

                        // and run it
                    (*sPlcTaskBootConfig.pfRun)();
                }

            break;

            // 3nd initialization phase
        case PLC_INIT_TASKS:
                // if valid PLC
            if(bProgramValid)
            {
                    // initialize all PLC tasks
                if(sPlcTaskBootInit.bTaskValid)
                    (*sPlcTaskBootInit.pfInit)();
                if(sPlcTaskFast.bTaskValid)
                    (*sPlcTaskFast.pfInit)();
                if(sPlcTaskSlow.bTaskValid)
                    (*sPlcTaskSlow.pfInit)();
                if(sPlcTaskBackground.bTaskValid)
                    (*sPlcTaskBackground.pfInit)();
                if(sPlcTaskRTHook.bTaskValid)
                    (*sPlcTaskRTHook.pfInit)();
                if(sPlcTaskDSPHook.bTaskValid)
                {
                    (*sPlcTaskDSPHook.pfInit)();
                    sPlcDSPHookEntryPoint=(void (*)(void))sPlcTaskDSPHook.pfRun;
                }

                    // initialize background multiimage copy while
                    // using it to avoid writing random values
                bBackgroundInInit=FALSE;
                bBackgroundOutInit=FALSE;
                uwBackgroundInitCnt=1;
            }
            break;

            // realtime hook task initialization
        case PLC_INIT_RTHOOK:
            if(bProgramValid && sPlcTaskRTHook.bTaskValid)
                    // add realtime hook task to scheduler, active only if plc enabled
                assert(TaskSched_AddRTTask(sPlcTaskRTHook.pfRun, TASKSCHEDULER_FLAG_NONE,
                        SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_RUNNING),
                        SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_RUNNING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_OVERTIME_DETECTED),
                        0));
            break;

            // fast task initialization
        case PLC_INIT_FAST:
            if(bProgramValid && sPlcTaskFast.bTaskValid)
                    // add fast task to scheduler, active only if plc enabled
                assert(TaskSched_AddRTTask(sPlcTaskFast.pfRun, TASKSCHEDULER_FLAG_NONE,
                        SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_RUNNING),
                        SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_RUNNING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_OVERTIME_DETECTED),
                        0));
            break;

            // image copy and slow task initialization
        case PLC_INIT_OTHERS:
            if(bProgramValid)
            {
                    // if slow task is defined
                if(sPlcTaskSlow.bTaskValid)
                {
                        // slow task manager in
                    assert(TaskSched_AddRTTask((BOOL (*)(void))&PlcSlowTaskMgrIn, TASKSCHEDULER_FLAG_NONE,
                            SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_RUNNING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_IMG_SLOW_IN),
                            SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_RUNNING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_IMG_SLOW_IN)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_OVERTIME_DETECTED),
                            0));

                        // slow task manager out
                    assert(TaskSched_AddRTTask((BOOL (*)(void))&PlcSlowTaskMgrOut, TASKSCHEDULER_FLAG_NONE,
                            SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_RUNNING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_IMG_SLOW_OUT),
                            SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_RUNNING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_IMG_SLOW_OUT)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_OVERTIME_DETECTED),
                            0));
                }

                    // if background task is defined
                if(sPlcTaskBackground.bTaskValid)
                {
                        // background task copy in
                    assert(TaskSched_AddRTTask(&PlcImageBackgroundIn, TASKSCHEDULER_FLAG_REITERATEIFREQ|TASKSCHEDULER_FLAG_OPTIONAL,
                            SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_RUNNING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_IMG_BACKGROUND_IN),
                            SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_RUNNING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_IMG_BACKGROUND_IN),
                            0));

                        // background task copy out
                    assert(TaskSched_AddRTTask(&PlcImageBackgroundOut, TASKSCHEDULER_FLAG_REITERATEIFREQ|TASKSCHEDULER_FLAG_OPTIONAL,
                            SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_RUNNING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_IMG_BACKGROUND_OUT),
                            SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_RUNNING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_PLC_IMG_BACKGROUND_OUT),
                            0));
                }

                    // add task to perform runtime boot, slow task creation and background task loop
                hBackgroundHandle=Os_TaskCreate(&backgroundloop);
                assert(hBackgroundHandle);
            }

                // check return code from com db set handler
            if(ubPlcComDbErrCode>0)
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_PLC_OVERTIME, SYSTEMALARMS_SUBCODE_PLC_COMDBERRMASK|ubPlcComDbErrCode, FALSE);

            break;
    }

    return TRUE;
}

//****************************************************************************
// Realtime task overtime, disable PLC, power and post alarm

void PlcRTOvertime(void)
{
        // on user request put power in safe lock
    if(bShadowDisablePwrOnRTOverTime)
        FpgaSafeLock();

        // disable power
    (*sPlcIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;

        // disable all RT and slow tasks but leaves on background
    bSysStatPlcOvertimeDetected=TRUE;

        // post alarm
    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_PLC_OVERTIME, SYSTEMALARMS_SUBCODE_PLC_FASTOVERTIME, FALSE);
}

//****************************************************************************
// background task loop

static void backgroundloop(void)
{
    static UWORD uwPrevSlowTaskPeriod=0;

        // run boot task
    if(sPlcTaskBootInit.bTaskValid)
        (*sPlcTaskBootInit.pfRun)();

        // setup shadow copy user options
    bShadowDisablePwrOnRTOverTime=bDisablePwrOnRTOverTime;
    bPlcEnableSysHMIDevelopment  =bEnableSysHMIDownload;

        // then enable PLC
    bSysStatPlcRunning=TRUE;

        // background task loop
    for(;;)
    {
            // if enabled
        if(bSysStatPlcRunning)
        {
                // check if time is modified
            if(uwPrevSlowTaskPeriod!=uwPlcSlowTaskPeriod)
            {
                    // reconfigure slow task
                if(uwPlcSlowTaskPeriod>PLC_SLOWTASK_PERIOD_MAX)
                    uwPlcSlowTaskPeriod=PLC_SLOWTASK_PERIOD_MAX;

                if(uwPlcSlowTaskPeriod>=PLC_SLOWTASK_PERIOD_MIN)
                {
                        // calculate reload cnt
                    uwSlowTaskCntReload=uwPlcSlowTaskPeriod*SLOWTASK_DIVIDER;
                    uwSlowTaskCounter=0;

                        // calculate max execution time
                    uwSlowTaskMaxAllowed=uwSlowTaskCntReload/2;
                    uwSlowTaskMaxAllowed=uwSlowTaskMaxAllowed>SLOWTASK_MAX_ALLOWED_RUNTIME*SLOWTASK_DIVIDER?
                                            SLOWTASK_MAX_ALLOWED_RUNTIME*SLOWTASK_DIVIDER:
                                            uwSlowTaskMaxAllowed;

                        // overtime allowed slot counter
                    uwSlowTaskOvertimeSlotLeft=SLOWTASK_MAX_PEAK_SLOTS;

                        // then enable slow task
                    bSysStatPlcImgSlowOut=FALSE;
                    bSysStatPlcImgSlowIn=TRUE;
                }
                else
                {
                        // disable slow task
                    bSysStatPlcImgSlowOut=FALSE;
                    bSysStatPlcImgSlowIn=FALSE;
                    uwSlowTaskCntReload=0;
                }

                uwPrevSlowTaskPeriod=uwPlcSlowTaskPeriod;
            }

                // if background task is defined
            if(sPlcTaskBackground.bTaskValid)
            {
                    // enable image copy in and wait until finished
                puwBackgroundImageStatus=&uwBackgroundImageListIn[0];

				// #### crs: background image workaround: begin ####
				*puwBackgroundImageStatus = 1 ;
				// #### crs: background image workaround: end ####
                
                bSysStatPlcImgBackgroundIn=TRUE;
                while(bSysStatPlcImgBackgroundIn)
                    Os_Sleep(0);

                    // execute background task
                bSysStatPlcRunBackground=TRUE;
                (*sPlcTaskBackground.pfRun)();
                bSysStatPlcRunBackground=FALSE;

                    // enable image copy out and wait until finished
                puwBackgroundImageStatus=&uwBackgroundImageListOut[0];
                bSysStatPlcImgBackgroundOut=TRUE;
                while(bSysStatPlcImgBackgroundOut)
                    Os_Sleep(0);
            }
        }

            // yield for a while
        Os_Sleep(0);
    }
}

//****************************************************************************
// Background task image copy in wrapper

BOOL PlcImageBackgroundIn(void)
{
        // if list is initialized then use the optimized one
    if(bBackgroundInInit)
    {
        if(*puwBackgroundImageStatus) {
/******** PLC TEMPORARY FIX - SAVE & RESTORE R11 ***************/
            asm("sub sp, sp, #8");
            asm("str r11, [sp]");

            (*sPlcTaskBackground.pfInput)(*puwBackgroundImageStatus++);

            asm("ldr r11, [sp]");
            asm("add sp, sp, #8");
        }
        else
            bSysStatPlcImgBackgroundIn=FALSE;

            // return TRUE if more copy has to be done
        return bSysStatPlcImgBackgroundIn;
    }
        // otherwise initialize the list while using it
    else
    {
/******** PLC TEMPORARY FIX - SAVE & RESTORE R11 ***************/
        asm("sub sp, sp, #8");
        asm("str r11, [sp]");
        if((*sPlcTaskBackground.pfInput)(uwBackgroundInitCnt)>0)
        {
            *puwBackgroundImageStatus++=uwBackgroundInitCnt;
            *puwBackgroundImageStatus=0;
        }
        asm("ldr r11, [sp]");
        asm("add sp, sp, #8");

        uwBackgroundInitCnt++;

            // end of initialization
        if(uwBackgroundInitCnt>=DATABLOCK_IMAGE_MAX_INDEX)
        {
            uwBackgroundInitCnt=1;
            bBackgroundInInit=TRUE;
            bSysStatPlcImgBackgroundIn=FALSE;
        }

            // during initialization it make one image check per time
        return FALSE;
    }
}

//****************************************************************************
// Background task image copy out wrapper

BOOL PlcImageBackgroundOut(void)
{
        // if list is initialized then use the optimized one
    if(bBackgroundOutInit)
    {
        if(*puwBackgroundImageStatus) {		
/******** PLC TEMPORARY FIX - SAVE & RESTORE R11 ***************/
			asm("sub sp, sp, #8");
			asm("str r11, [sp]");
            (*sPlcTaskBackground.pfOutput)(*puwBackgroundImageStatus++);
            asm("ldr r11, [sp]");
            asm("add sp, sp, #8");
		}
        else
            bSysStatPlcImgBackgroundOut=FALSE;

            // return TRUE if more copy has to be done
        return bSysStatPlcImgBackgroundOut;
    }
        // otherwise initialize the list while using it
    else
    {
/******** PLC TEMPORARY FIX - SAVE & RESTORE R11 ***************/
		asm("sub sp, sp, #8");
		asm("str r11, [sp]");
        if((*sPlcTaskBackground.pfOutput)(uwBackgroundInitCnt)>0)
        {
            *puwBackgroundImageStatus++=uwBackgroundInitCnt;
            *puwBackgroundImageStatus=0;
        }
        asm("ldr r11, [sp]");
        asm("add sp, sp, #8");

        uwBackgroundInitCnt++;

            // end of initialization
        if(uwBackgroundInitCnt>=DATABLOCK_IMAGE_MAX_INDEX)
        {
            uwBackgroundInitCnt=1;
            bBackgroundOutInit=TRUE;
            bSysStatPlcImgBackgroundOut=FALSE;
        }

            // during initialization it make one image check per time
        return FALSE;
    }
}

//****************************************************************************
// Slow task manager in

void PlcSlowTaskMgrIn(void)
{
        // if time is elapsed
    if(++uwSlowTaskCounter>=uwSlowTaskCntReload)
    {
            // reset for execution time measurement
        uwSlowTaskCounter=0;

/******** PLC TEMPORARY FIX - SAVE & RESTORE R11 ***************/
        asm("sub sp, sp, #8");
        asm("str r11, [sp]");
            // image copy in
        (*sPlcTaskSlow.pfInput)(0);
        asm("ldr r11, [sp]");
        asm("add sp, sp, #8");

            // trigger the irq for the slow task
#ifdef _INFINEON_
        CC2_CC25IC_IE=1;
        _atomic_(0);
        CC2_CC25=CC2_T8+2;
        _endatomic_();
#else
        Timer_CCStart(TIMER_CCU2_ID,TIMER_CCU2_COUNT+60);
#endif
            // enable mgr out
        bSlowTerminated=FALSE;
        bSysStatPlcImgSlowOut=TRUE;
    }
}

//****************************************************************************
// slow task handler

#ifdef _INFINEON_
#pragma savemac
static void slowtaskhandler(void) interrupt SLOWTASK_INTERRUPT_VECTOR
{
        // restore default MAC settings as SAVEMAC does not
    OS_SETDEFAULT_ALU();

        // disable itself, will be re-enabled by the slow task manager
    CC2_CC25IC_IE=0;
#else
static void slowtaskhandler(void)
{
        // disable itself, will be re-enabled by the slow task manager
    Timer_CCStop(TIMER_CCU2_ID);
#endif

        // execute slow task
    bSysStatPlcRunSlow=TRUE;
    (*sPlcTaskSlow.pfRun)();
    bSysStatPlcRunSlow=FALSE;

        // calculate execution time
    uwPlcSlowTaskTime=(uwSlowTaskCounter+1)*10/SLOWTASK_DIVIDER;

        // manage allowed peak execution counter
    if(uwSlowTaskCounter>=uwSlowTaskMaxAllowed)
    {
            // if overtime then remove one allowed slot
        if(uwSlowTaskOvertimeSlotLeft)
            uwSlowTaskOvertimeSlotLeft--;
    }
        // if in time add allowed slot
    else if(uwSlowTaskOvertimeSlotLeft<SLOWTASK_MAX_PEAK_SLOTS)
        uwSlowTaskOvertimeSlotLeft++;

        // notify end of execution
    bSlowTerminated=TRUE;
}
#ifdef _INFINEON_
#pragma nosavemac
#endif

//****************************************************************************
// Slow task manager in out

void PlcSlowTaskMgrOut(void)
{
        // if enabled, disable here and exit, to avoid running
        // both In and Out in same RT cycle
    if(bSysStatPlcImgSlowIn)
    {
        bSysStatPlcImgSlowIn=FALSE;
        return;
    }

        // always increment for next execution slot
    uwSlowTaskCounter++;

    if(!bSlowTerminated)
    {
            // if absolute max is elapsed or allowed left slot is zero then signal immediate fault
        if(uwSlowTaskCounter>uwSlowTaskCntReload || uwSlowTaskOvertimeSlotLeft==0)
        {
                // disable power
            (*sPlcIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;

                // disable all RT and slow tasks but leaves on background
            bSysStatPlcOvertimeDetected=TRUE;

                // post alarm
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_PLC_OVERTIME, SYSTEMALARMS_SUBCODE_PLC_SLOWOVERTIME, FALSE);
        }
    }

        // task terminated
    else
    {
           // image copy out immediately or at last cycle
        if(bSlowTaskCopyOutImmediate || uwSlowTaskCounter>=(uwSlowTaskCntReload-1))
        {
/******** PLC TEMPORARY FIX - SAVE & RESTORE R11 ***************/
			asm("sub sp, sp, #8");
			asm("str r11, [sp]");
        	(*sPlcTaskSlow.pfOutput)(0);
            asm("ldr r11, [sp]");
            asm("add sp, sp, #8");

                // disable mgr out and enable mgr in
            bSysStatPlcImgSlowOut=FALSE;
            bSysStatPlcImgSlowIn=TRUE;
        }
    }
}

//****************************************************************************
// Standby request callback from runtime

bool_t PlcStandByRequired(void)
{
        // if power disabled
    if(!bSysStatPowerEnabled)
    {
        bStandbyRequested=TRUE;
            // ack standby request
        return FALSE;
    }
    else
            // drive enabled, cannot download code
        return TRUE;
}

//****************************************************************************
// External standby request

BOOL PlcRequestStandby(void)
{
    return !PlcStandByRequired();
}

//****************************************************************************
// Standby request callback from runtime

BOOL PlcLockForReload(BOOL bKeepComDBApp)
{
        // if was not requested, then deny
    if(!bStandbyRequested)
        return FALSE;

    DISABLE_IRQ();
        // if power disabled
    if(!bSysStatPowerEnabled)
    {
            // lock drive until reset
        bSysStatLockedByPlcReload=TRUE;
        bSysStatPlcRunning=FALSE;
        RESTORE_IRQ();

        if(hBackgroundHandle)
        {
            Os_TaskSuspend(hBackgroundHandle);
            hBackgroundHandle=NULL;
        }

//        _atomic_(0);
        sPlcDSPHookEntryPoint=NULL;
//        _endatomic_();

            // unset parameters
        if(!bKeepComDBApp)
            PlcComDBAppUnset();

            // ack
        return TRUE;
    }
    else
    {
        RESTORE_IRQ();
            // drive enabled, cannot download code
        return FALSE;
    }
}

//****************************************************************************
// End of code download event

void PlcEndOfLoad(void)
{
        // resetting flag and syslog flush flag
    bSysStatResetting=TRUE;
    bSysStatSysLogFlush=TRUE;

        // then mark for resetting
    bResetRequested=TRUE;
    uwResetDelayTimer=timer_settimeout(uwOsFreeRunTimer1kHz, SW_RESET_DELAY);
}

//****************************************************************************
// Init download source code

void PlcInitDwSourceCode(void)
{
        // 64 is the number of bit available in the flash erase map variable
    assert(ulSerialFlashSize/SFLASH_SECTOR_SIZE <= 64);
    INT64_COPY(uqSrcCodeEraseMap,sqZero64bit);
}

//****************************************************************************
// Check download source code address if it's to be erased

void PlcCheckAddressDwSourceCode(ULONG ulAddress, UWORD uwLength)
{
    ULONG ulTmp;
    UQWRD uqMask;

        // if address range is source code area, system HMI (if enabled) or app HMI
    if((ulAddress>=SFPART_PLC_PRJ_START    && ulAddress<SFPART_PLC_PRJ_START+SFPART_PLC_PRJ_SIZE) ||
       (ulAddress>=SFPART_RD_HMI_APP_START && ulAddress<SFPART_RD_HMI_APP_START+SFPART_RD_HMI_APP_SIZE))
    {
        ulTmp=ulAddress-SFLASH_ADDRESS_OFFSET+uwLength-1;
        INT64_ASSIGN(uqMask,0ul,1ul);

            // calculate involved sector
        while(ulTmp>=SFLASH_SECTOR_SIZE)
        {
            ulTmp-=SFLASH_SECTOR_SIZE;
            _uint64_shl(&uqMask,1);
        }

            // check if it's to be erased
        if((uqSrcCodeEraseMap.hi&uqMask.hi)==0ul && (uqSrcCodeEraseMap.lo&uqMask.lo)==0ul)
        {
            SerialFlashEraseSector(ulAddress+uwLength-1);
            uqSrcCodeEraseMap.hi|=uqMask.hi;
            uqSrcCodeEraseMap.lo|=uqMask.lo;
        }
    }
}

//****************************************************************************
// Get header of source code area

void  * PlcGetSourceCodeAreaHeader(void)
{
        // check flash area size
    if(SFPART_PLC_PRJ_START+SFPART_PLC_PRJ_SIZE>SFLASH_ADDRESS_OFFSET+ulSerialFlashSize)
            // if out of serial flash size, void area
        memset(&sSrcCodeHeader, 0, sizeof(sSrcCodeHeader));

    else
        SerialFlashReadBytes(SFPART_PLC_PRJ_START, (UBYTE *)&sSrcCodeHeader, sizeof(sSrcCodeHeader));

    return &sSrcCodeHeader;
}

//****************************************************************************
// Calculate source code area checksum

static void _checksumcback(UBYTE ubVal, void * vContest)
{
    *(uint32_t *)vContest+=ubVal;
}

uint32_t PlcChecksumSourceCode(void)
{
	uint32_t calc_chk=0;
    uint32_t size_left=sSrcCodeHeader.size>32767l?32767:sSrcCodeHeader.size;
    uint32_t address=SFPART_PLC_PRJ_START;
    uint16_t size;
    uint8_t  * pSource;
    uint8_t  data[SFLASH_PAGE_SIZE];
    uint16_t count;

        // calculate overall area checksum (max chunks of 32kb)
    while(size_left>0)
    {
#ifdef _INFINEON_
        size=size_left>32767l?32767:(uint16_t)size_left;
        SerialFlashReadWithCallBack(address, &_checksumcback, &calc_chk, size);
#else
        size=size_left>SFLASH_PAGE_SIZE?SFLASH_PAGE_SIZE:size_left;
        SerialFlashReadBytes(address,data,SFLASH_PAGE_SIZE);
        for(count=0;count<size;count++)
            _checksumcback(data[count],&calc_chk);
#endif
        address+=size;
        size_left-=size;
    }

	/*	Subtract checksum value	*/
	pSource = (uint8_t  *)&sSrcCodeHeader.chksum;
	calc_chk -= pSource[0];
	calc_chk -= pSource[1];
	calc_chk -= pSource[2];
	calc_chk -= pSource[3];

	return calc_chk;
}

//****************************************************************************
// Check if reset is requested

static void resetrequestcheck(void)
{
        // if reset requested
    if(bResetRequested)
        // wait until syslog is safely written and timer elapsed
        if(!bSysStatSysLogFlush && timer_istimedout(uwOsFreeRunTimer1kHz, uwResetDelayTimer))
        {
                // then reset
            if(bRebootMinimalRequested)
                SysRes_ExecuteReset(RESETCODE_PLCREBOOTMINIMAL, ~RESETCODE_PLCREBOOTMINIMAL);
            else
                SysRes_ExecuteReset(SYSRES_POWERON_VALUE_PAR0, SYSRES_POWERON_VALUE_PAR1);
        }
}

//***************************************************************************
// Check if in slow task

BOOL PlcIsInSlowTask(void)
{
#ifdef _INFINEON_
    return (PSW>>12)==((SYSAPPIL_PLC_SLOWTASK_CC25>>2)&0x000f);
#else
    return Timer_CCStatus(TIMER_CCU2_ID);
#endif
}

//***************************************************************************
// IEC DB table search, return address added of the right offset

void  * PlcIECDBEntrySearch(UBYTE B, UWORD uwDbIdx, UWORD uwDbOffset)
{
    UWORD cnt;
    ULONG address;
    const PLCIEC_DBREC * psDb=plcDataBlocks;

        // table search
    for(cnt=0;cnt<PLC_NUM_DB;cnt++,psDb++)
    {
            // if same IEC address
        if(psDb->dbId==uwDbIdx)
        {
                // get base address
            address=psDb->addr;

                // add offset for specific data type
            if((SBYTE)psDb->dataSize==-1)
                address+=(uwDbOffset>>4)*sizeof(UWORD);

            else
                address+=psDb->dataSize*uwDbOffset;

                // and return
            return (void  *)address;
        }
    }

        // no match, return NULL
    return NULL;
}

//***************************************************************************
// Post user alarm from PLC application

void PlcPostUserAlarm(ULONG ulAlarmSubCode, SBYTE bStore)
{
    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_USERDEFINED, ulAlarmSubCode, bStore);
}

//***************************************************************************
// Trigger emergency stop from PLC application

void PlcPostEmcyStop(UWORD uwOption)
{
    (*sPlcIn.psCtrlHandlers->pfEmcyStop)(uwOption) ;
}

//***************************************************************************
// Time profiler functions for PLC application

UWORD PlcProfilerBegin(const UBYTE ubTimerSel)
{
    switch(ubTimerSel)
    {
        case PLC_PROFILER_TIMER100ns:
            return timer_profiler_start(uwSysTimers100ns);

        case PLC_PROFILER_TIMER125us:
            return timer_profiler_start(uwSysTimers125us);

        case PLC_PROFILER_TIMER1ms:
            return timer_profiler_start(uwSysTimers1ms);
    }

    return 0;
}

UWORD PlcProfilerEnd(const UBYTE ubTimerSel, UWORD uwStart)
{
    switch(ubTimerSel)
    {
        case PLC_PROFILER_TIMER100ns:
            return timer_100nscorrect(timer_profiler_end(uwSysTimers100ns,uwStart));

        case PLC_PROFILER_TIMER125us:
            return timer_profiler_end(uwSysTimers125us,uwStart);

        case PLC_PROFILER_TIMER1ms:
            return timer_profiler_end(uwSysTimers1ms,uwStart);
    }

    return 0;
}

//***************************************************************************
// Get memory ID calculated offline for diagnostic PLC checking

uint16_t PlcGetDiagMemoryID(void)
{
    return sPlcDiagBuildInfo.uwMemoID;
}

uint16_t PlcGetDiagFullMemoryID(void)
{
    return sPlcDiagBuildInfo.uwFullMemoID;
}

//***************************************************************************
// PLC main binaries streaming - begin

#ifdef _APP_XC
SWORD PlcStreamCodeBegin(void)
{
    SWORD swRetVal=0;

        // ask for stand-by
    if(!PlcRequestStandby())
        return -1;
    if(!PlcLockForReload(FALSE))
        return -1;

        // setup status
    ulStrAddress=PLC_RT_CODE_START;
    assert((ulStrAddress&(PROGRAMFLASH_PAGE_SIZE-1))==0);
    assert((ulStrAddress&(PROGRAMFLASH_SECTOR_SIZE-1))==0);

        // and erase first sector
    bSysStatProgramFlashWriting=TRUE;
    DISABLE_IRQ();
    SecurityFlashDisableWriteProtection(0x55AA, 0xAA55);

    if(ProgramFlashErase((void *)ulStrAddress, PROGRAMFLASH_SECTOR_SIZE)!=0)
        swRetVal=-1;

    SecurityFlashReEnableRdWrProtection();
    RESTORE_IRQ();
    bSysStatProgramFlashWriting=FALSE;

    return swRetVal;
}

//***************************************************************************
// PLC main binaries streaming - data

SWORD PlcStreamCodeData(HPUBYTE pubBuffer, UWORD uwSize)
{
    UBYTE  * pubPos;
    SWORD swRetVal=0;

        // queue data in local buffer
    pubPos=&(ubStrBuffer[ulStrAddress&(PROGRAMFLASH_PAGE_SIZE-1)]);
    while(uwSize>0 && swRetVal==0)
    {
            // if out of space
        if(ulStrAddress>=PLC_RT_CODE_START+PLC_RT_CODE_SIZE)
            return -1;

            // add byte
        *pubPos++=*pubBuffer++;
        ulStrAddress++;
        uwSize--;

            // when local buffer is full
        if((ulStrAddress&(PROGRAMFLASH_PAGE_SIZE-1))==0)
        {
            bSysStatProgramFlashWriting=TRUE;
            DISABLE_IRQ();
            SecurityFlashDisableWriteProtection(0x55AA, 0xAA55);

                // write data page
            if(ProgramFlashWrite((void *)(ulStrAddress-PROGRAMFLASH_PAGE_SIZE), ubStrBuffer, PROGRAMFLASH_PAGE_SIZE)!=0)
                swRetVal=-1;

                // check for sector change, in case prepare next sector
            if((ulStrAddress&(PROGRAMFLASH_SECTOR_SIZE-1))==0)
                if(ProgramFlashErase((void *)ulStrAddress, PROGRAMFLASH_SECTOR_SIZE)!=0)
                    swRetVal=-1;

            SecurityFlashReEnableRdWrProtection();
            RESTORE_IRQ();
            bSysStatProgramFlashWriting=FALSE;

            pubPos=ubStrBuffer;
        }
    }

    return swRetVal;
}

//***************************************************************************
// PLC main binaries streaming - end

SWORD PlcStreamCodeEnd(void)
{
    SWORD swRetVal=0;

        // flush buffer (if necessary)
    if(ulStrAddress&(PROGRAMFLASH_PAGE_SIZE-1))
    {
            // write data page
        bSysStatProgramFlashWriting=TRUE;
        DISABLE_IRQ();
        SecurityFlashDisableWriteProtection(0x55AA, 0xAA55);

        if(ProgramFlashWrite((void *)(ulStrAddress&~(PROGRAMFLASH_PAGE_SIZE-1)), ubStrBuffer, ulStrAddress&(PROGRAMFLASH_PAGE_SIZE-1))!=0)
            swRetVal=-1;

        SecurityFlashReEnableRdWrProtection();
        RESTORE_IRQ();
        bSysStatProgramFlashWriting=FALSE;
    }

    return swRetVal;
}

#endif
