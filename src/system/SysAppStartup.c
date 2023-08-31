/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppStartup.c                                            */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Main entry point of System Application                     */
/*                                                                          */
/****************************************************************************/
#include "system\SysAppGlobals.h"
#include "system\SysAppTaskCollection.h"
#include "system\SysAppHwOptReq.h"
#include "system\GlobalResetCodes.h"
#include "system\SysLogManagement.h"
#include "system\SysAppConfig.h"

#include "core\SystemReset.h"

#include "drive\HardwareConfig.h"
#include "drive\AxM-E-Defines.h"
#include "drive\OneWireHandler.h"
#include "drive\MotorHandler.h"

#include "fpga\FpgaHandler.h"

#include "common\ParamStorageManagement.h"
#include "common\ParametersCheck.h"
#include "common\FlashManager.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#if defined(_CRS_DBG)
#if CRS_DBGDSK
#pragma GCC optimize (0) // crs_dbg
#else
#pragma GCC optimize (2)
#endif
#else
#pragma GCC optimize (2)
#endif
//****************************************************************************
// Defines

#ifndef _HW_DC
#define SKIPBYUSERBUTTONS()     (XE167_DIAG_BUTN1 && XE167_DIAG_BUTN2)
#else
#define SKIPBYUSERBUTTONS()     (FALSE)
#endif

//****************************************************************************
// Base hardware configuration
//****************************************************************************
static void hardware_config(void)
{

    ///  -----------------------------------------------------------------------
    ///  Reset config
    ///  -----------------------------------------------------------------------

    ///  -----------------------------------------------------------------------
    ///  Setup Running System Frequency
    ///  -----------------------------------------------------------------------
    uwSystemMainClockFreq=SYSTEMCLOCK_COMP;

    ///  -----------------------------------------------------------------------
    ///  Gpio config
    ///  -----------------------------------------------------------------------
    Gpio_Init();

#ifdef _HW_DC
    ///  -----------------------------------------------------------------------
    ///  AxN-DC Configuration
    ///  -----------------------------------------------------------------------

    //  -----------------------------------------------------------------------
    //  EMIO GPIO[2]    : LED output green
    //  EMIO GPIO[3]    : LED output yellow
    //  EMIO GPIO[4]    : LED output red
    //  MIO[7]  	    : ETH PMC LED A
    //  MIO[8]	        : ETH PMC LED B
    //  -----------------------------------------------------------------------

    Gpio_SetMode(LEDS_GRN_PIN,GPIO_DIR_OUT);
    Gpio_SetMode(LEDS_YEL_PIN,GPIO_DIR_OUT);
    Gpio_SetMode(LEDS_RED_PIN,GPIO_DIR_OUT);

#ifndef _HW_AXS
    Gpio_SetMode(7,GPIO_DIR_OUT);
    Gpio_SetMode(8,GPIO_DIR_OUT);
#endif

#ifdef _HW_AXS
#ifdef CAN0_EN_PIN
    Gpio_SetMode(CAN0_EN_PIN, GPIO_DIR_OUT); // set direction register
	GPIO_OUT(CAN0_EN_PIN, TRUE);             // and set high to enable transceiver
#endif
#endif




#else
    // RESET BUTTON
    Gpio_SetMode(RESET_BUTTON_PIN,GPIO_DIR_IN);

    // LEDs
    Gpio_SetMode(LEDS_0_PIN,GPIO_DIR_OUT);
    Gpio_SetMode(LEDS_1_PIN,GPIO_DIR_OUT);
    Gpio_SetMode(LEDS_2_PIN,GPIO_DIR_OUT);
    Gpio_SetMode(LEDS_3_PIN,GPIO_DIR_OUT);
    Gpio_SetMode(LEDS_4_PIN,GPIO_DIR_OUT);
    Gpio_SetMode(LEDS_5_PIN,GPIO_DIR_OUT);
    Gpio_SetMode(LEDS_6_PIN,GPIO_DIR_OUT);
    Gpio_SetMode(LEDS_7_PIN,GPIO_DIR_OUT);

	Gpio_SetMode(CAN0_EN_PIN, GPIO_DIR_OUT); // set direction register
	GPIO_OUT(CAN0_EN_PIN, TRUE);             // and set high to enable transceiver

	Gpio_SetMode(CAN1_EN_PIN, GPIO_DIR_OUT); // set direction register
	GPIO_OUT(CAN1_EN_PIN, TRUE);             // and set high to enable transceiver
#endif

    ///  -----------------------------------------------------------------------
    ///  Timer config : RealTime Tasks Scheduler
    ///  -----------------------------------------------------------------------
    Timer_Init(REALTIME_TASK_FREQ, TaskSched_RTScheduler);

    ///  -----------------------------------------------------------------------
    ///  Flash Manager Init
    ///  -----------------------------------------------------------------------
    FlashMgrInit();
}

//****************************************************************************
// Power On Joke

#define PWRONJOKE_TIMERSTEP()   (COUNTS_PER_1MS*40) // 40ms

typedef struct
{
    SWORD swRun;
    SWORD swSel;
    UWORD uwIdx;
    UWORD uwIdx1;
    UWORD uwTmrStat;
#ifdef _HW_DC
    SBYTE bUserButton;
#endif
} PWRONJOKE_STATUS;

static PWRONJOKE_STATUS * sPwrOnJokeStat;

static void PowerOnJokeInit(PWRONJOKE_STATUS * sStat, BOOL safemode)
{
        // only pointer is maintained as global, in order to save
        // global heap after bootup
    sPwrOnJokeStat=sStat;

        // init status data structure
    sPwrOnJokeStat->uwIdx=0;
    sPwrOnJokeStat->uwIdx1=0;
    sPwrOnJokeStat->swSel=safemode?-1:0;
    sPwrOnJokeStat->swRun=+1;
#ifdef _HW_DC
    sPwrOnJokeStat->bUserButton=0;
#endif
}

static BOOL PowerOnJokeProc(void)
{
        // first time initialize timer and continue
    if(sPwrOnJokeStat->swRun>0)
    {
        sPwrOnJokeStat->swRun=-1;
        sPwrOnJokeStat->uwTmrStat=timer_settimeout((UWORD)(GLOBAL_TMR>>16), 0);
    }
        // if finished sequence exit immediately
    else if(sPwrOnJokeStat->swRun==0)
        return FALSE;
        // if timer not yet expired then exit
    else if(!timer_istimedout((UWORD)(GLOBAL_TMR>>16), sPwrOnJokeStat->uwTmrStat))
        return TRUE;

        // process safe mode if requested
    if(sPwrOnJokeStat->swSel)
    {
        if(sPwrOnJokeStat->uwIdx<8)
        {
            sPwrOnJokeStat->uwIdx++;
#ifndef _HW_DC
            LEDS_OUTPUT( ~ (0xFF >> sPwrOnJokeStat->uwIdx) );
#else
            LEDS_OUTPUT( (sPwrOnJokeStat->uwIdx&0x01?LEDS_OUT_YEL:0x00) | LEDS_OUT_GRN );
#endif
        }
        else if(sPwrOnJokeStat->uwIdx++>=11)
        {
            sPwrOnJokeStat->swRun=0;
            return FALSE;
        }

        sPwrOnJokeStat->uwTmrStat=timer_settimeout(sPwrOnJokeStat->uwTmrStat, (UWORD)(PWRONJOKE_TIMERSTEP()>>16));
        return TRUE;
    }

    {
#ifndef _HW_DC
        static const UBYTE ubSeq[]={0x81,0x42,0x24,0x18,0x24,0x42,0x81,0x81,0x41,0x41,0x21,0x21,0x11,0x11,0x09,0x09,0x05,0x05,0x03,0x03,0x01,0x01,0x00,0x00};

        if(sPwrOnJokeStat->uwIdx<sizeof(ubSeq)/sizeof(UBYTE))
        {
            LEDS_OUTPUT( ubSeq[sPwrOnJokeStat->uwIdx] );
            sPwrOnJokeStat->uwIdx++;
            sPwrOnJokeStat->uwTmrStat=timer_settimeout(sPwrOnJokeStat->uwTmrStat, (UWORD)(PWRONJOKE_TIMERSTEP()>>16));
            return TRUE;
        }

#else
        static const UBYTE ubSeq[]=
                                        {LEDS_OUT_YEL,0,LEDS_OUT_YEL,0,LEDS_OUT_YEL,0,0,0,0,
                                         LEDS_OUT_YEL,0,LEDS_OUT_YEL,0,LEDS_OUT_YEL,0,0,0,0,
                                         LEDS_OUT_YEL,0,LEDS_OUT_YEL,0,LEDS_OUT_YEL,0,0,0,0,
                                         LEDS_OUT_YEL,0,LEDS_OUT_YEL,0,LEDS_OUT_YEL,0};
        static const UBYTE ubSUsr[sizeof(ubSeq)]=
                                        {LEDS_OUT_YEL|LEDS_OUT_RED,LEDS_OUT_RED,LEDS_OUT_YEL|LEDS_OUT_RED,LEDS_OUT_RED,LEDS_OUT_YEL|LEDS_OUT_RED,LEDS_OUT_RED,0,0,0,
                                         LEDS_OUT_YEL|LEDS_OUT_RED,LEDS_OUT_RED,LEDS_OUT_YEL|LEDS_OUT_RED,LEDS_OUT_RED,LEDS_OUT_YEL|LEDS_OUT_RED,LEDS_OUT_RED,0,0,0,
                                         LEDS_OUT_YEL|LEDS_OUT_RED,LEDS_OUT_RED,LEDS_OUT_YEL|LEDS_OUT_RED,LEDS_OUT_RED,LEDS_OUT_YEL|LEDS_OUT_RED,LEDS_OUT_RED,0,0,0,
                                         LEDS_OUT_YEL|LEDS_OUT_RED,LEDS_OUT_RED,LEDS_OUT_YEL|LEDS_OUT_RED,LEDS_OUT_RED,LEDS_OUT_YEL|LEDS_OUT_RED,LEDS_OUT_RED};

#ifndef _HW_AXS
            // check user button during startup
        if(XE167_RESET_BUTTON)
           sPwrOnJokeStat->bUserButton=TRUE;
#endif

        if(sPwrOnJokeStat->uwIdx<sizeof(ubSeq)/sizeof(UBYTE))
        {
            if(sPwrOnJokeStat->bUserButton)
            {
                LEDS_OUTPUT(ubSUsr[sPwrOnJokeStat->uwIdx] );
            }
            else
            {
                LEDS_OUTPUT( ubSeq[sPwrOnJokeStat->uwIdx] );
            }
            sPwrOnJokeStat->uwIdx++;
            sPwrOnJokeStat->uwTmrStat=timer_settimeout(sPwrOnJokeStat->uwTmrStat, (UWORD)(PWRONJOKE_TIMERSTEP()>>16));
            return TRUE;
        }
#endif
    }

    LEDS_OUTPUT( 0x00 );
    sPwrOnJokeStat->swRun=0;
    return FALSE;
}

//****************************************************************************
// Boot up the system: all code placed here in order to free all locals
// after the bootup from the user stack (to maximize it)

static void SystemBoot(void)
{
    UWORD uwInitAllExitCode;
    BOOL bSkipByUserButtons=FALSE;
    BOOL bStartWithPowerFail=FALSE;
    BOOL bParamValid=FALSE;
    BOOL bUnLockPowerSection=FALSE;
    UWORD uwTimeout;

    {
        PWRONJOKE_STATUS sPwrOnJoke;
        BOOL (* pfPwrOnJoke)(void)=NULL;
        ULONG ulAlteraLoadedBytes;
        SWORD swResult;

#ifdef _DEBUG_TRACES
    	xil_printf("SysAppStartup::SystemBoot()\r\n");
#endif

            // basic initialization
        Globals_Init();
    
            // system flags/status
        bSysStatBooting=1;
    
            // basic system hw configuration
        hardware_config();
#ifdef _INFINEON_
        assert(HwConfPSRAMWrProt(TRUE));
#endif

            // wait if active otherwise powerfail will be not triggered
        uwTimeout=PF_TIMEOUTONSTARTUP;
        POWER_FAIL_SETMODE;
// check SAIETTA ZQ_PFAIL before using POWER_FAIL_IN
#ifndef _HW_AXS
        while(!POWER_FAIL_IN)
        {
                // 1 msec
            timer_wait(uwSysTimers100ns, 10000);

            if(--uwTimeout==0)
            {
                bStartWithPowerFail=TRUE;
                break;
            }
        }
#endif

            // get button status for configuration-only startup
#ifndef _HW_AXS
        bSkipByUserButtons = SKIPBYUSERBUTTONS();
#endif

            // signal startup via leds if normal reset requested
        if((uwSysResParam0==~uwSysResParam1) && (uwSysResParam0==RESETCODE_CLEANREBOOTREQUESTED
#ifdef _APP_XC
           || uwSysResParam0==RESETCODE_CLEANREBOOT_COMP
#ifdef _INFINEON_
           || uwSysResParam0==RESETCODE_CLEANREBOOT_PERF
#endif // _infineon_
#endif // _app_xc
        ))
        {
            PowerOnJokeInit(&sPwrOnJoke, bSkipByUserButtons);
            pfPwrOnJoke=&PowerOnJokeProc;
        }

            // disable write protection
        SecurityFlashDisableWriteProtection( 0x55AA, 0xAA55 );

            // 1wire initialization
        OWHandlerInit();
        timer_wait(uwSysTimers100ns, 1000);

            // hardware config retrieve and check
        if(!HwConfGetAndCheck((void (*)(void))pfPwrOnJoke))
        {
            uwSystemBootErrorCode=SYSTEMBOOTERR_INVALIDHWCONFIG;
            bSysStatLockedByBootError=1;
        }

#ifndef _HW_AXS
#ifdef _HW_DC
            // get user button here
        bSkipByUserButtons=sPwrOnJoke.bUserButton;
#endif
#endif

            // if diagnostic application is requested
        if(uwSysResParam0==~uwSysResParam1 && uwSysResParam0==RESETCODE_ENTERDIAGNOSTICAPPLICATION)
            uwSystemEnterDiagnostic=SYSTEM_ENTERDIAGNOSTIC_KEY;

            // task scheduler initialization
        TaskSched_Init();

            // parameter area default values load
        parmgm_par_init();
        parmgm_par_default();

            // parameter area check and load
        swResult=parmgm_par_verify();
#if (defined(_DEBUG_TRACES))
        xil_printf("SysAppStartup::SystemBoot() : parmgm_par_verify() = %d\r\n", swResult);
#endif

            // if invalid parameters
        if(swResult==PARMGM_B_ERROR)
            if(!bSysStatLockedByBootError)
            {
                uwSystemBootErrorCode=SYSTEMBOOTERR_INVALIDFLASHPARAMETERS;
                bSysStatLockedByBootError=1;
            }

            // if valid and not safe start try to load parameters
            // if error reset in order to restore partially restored parameters
            // with default values, if reset code is this then do not load
            // and signal boot error
        if(uwSysResParam0==~uwSysResParam1 && uwSysResParam0==RESETCODE_NOLOADPARAMSREQUESTED)
        {
            if(!bSysStatLockedByBootError)
            {
                uwSystemBootErrorCode=SYSTEMBOOTERR_INVALIDFLASHPARAMETERS;
                bSysStatLockedByBootError=1;
            }
        }
        else if(swResult==PARMGM_B_OK)
        {
                // set param valid, but only if user has not requested skip by buttons
            bParamValid=!bSkipByUserButtons;

            if(parmgm_par_load(PARMGM_LOAD_FULL)!=PARMGM_B_OK)
              SysRes_ExecuteReset((UWORD)RESETCODE_NOLOADPARAMSREQUESTED,(UWORD)~RESETCODE_NOLOADPARAMSREQUESTED);
        }
        else if(swResult==PARMGM_B_ERASED)
            parmgm_par_restore();

            //setup hardware option request
        HwOptReqCollect();

            // check global options
        if(!HwConfGlobalOptions())
            if(!bSysStatLockedByBootError)
            {
                uwSystemBootErrorCode=SYSTEMBOOTERR_UNSUPPORTEDGLBOPT;      // unsupported hardware
                bSysStatLockedByBootError=1;
            }

            // fpga initialization and check
        if(!bSkipByUserButtons)
        {
            FpgaInit();
            if((swResult = FpgaLoad(&ulAlteraLoadedBytes, (void (*)(void))pfPwrOnJoke)) != FPGA_LOAD_SUCCESSFULLY)
                if(!bSysStatLockedByBootError)
                {
                    uwSystemBootErrorCode=SYSTEMBOOTERR_FPGAPROGFAIL-swResult;
                    bSysStatLockedByBootError=1;
                }

                // fpga identification verify
            if(!bSysStatLockedByBootError)
                if(!FpgaIdentVerify())
                {
                    uwSystemBootErrorCode=SYSTEMBOOTERR_FPGAINCOMPAT;
                    bSysStatLockedByBootError=1;
                }

                // test registers
            if(!bSysStatLockedByBootError)
                if((swResult = FpgaTest()) != 0 )
                {
                    uwSystemBootErrorCode=SYSTEMBOOTERR_FPGATESTFAIL-swResult;
                    bSysStatLockedByBootError=1;
                }
        }

            // terminate power on joke
        if(pfPwrOnJoke)
            while((*pfPwrOnJoke)());
    }

#if CFG_FASTTASKOVERTIMEDISABLE
    ulSystemStatus |= 0x00000800 ; // bit 11

#if (defined(_DEBUG_TRACES))
    xil_printf("SysAppStartup::SystemBoot() :FastTaskOverTimeDisable\r\n");
#endif

#endif


        // enable interrupt
    Intr_Init();

        // initialize Os
    Os_Init();

        // then enter critical section until the end of initialization
    Os_BeginCriticalSection(OS_CRITSECT_SWITCHANDEVENT);

        // if error on startup or configuration-only requested
    if(bSysStatLockedByBootError || bSkipByUserButtons)
    {
            // minimal task startup with or w/out flash params
        if(bParamValid)
            TaskSched_InitializeAll((TASKSCHEDULER_TASK_INIT *)psSysAppTaskAltCollectWPars);
        else
            TaskSched_InitializeAll((TASKSCHEDULER_TASK_INIT *)psSysAppTaskAltCollection);

            // and lock system
        FpgaSafeLock();

        bSysStatLockedByBootError=1;
    }
        // after PLC reload if requested for parametrization then execute minimal boot
    else if(uwSysResParam0==~uwSysResParam1 && uwSysResParam0==RESETCODE_PLCREBOOTMINIMAL)
    {
            // minimal task startup with flash params and plc params created
        TaskSched_InitializeAll(psSysAppTaskAltCollectFullCfg);

            // and lock system
        FpgaSafeLock();

        bSysStatLockedByBootError=1;
    }
    else
    {
            // end of core initialization, begin tasks and fieldbus common initialization
        uwInitAllExitCode=TaskSched_InitializeAll((TASKSCHEDULER_TASK_INIT *)psSysAppTaskCollection);

            // if fail, then inhibit the system but not FPGA as could be still used
            // by special applications
        if(uwInitAllExitCode)
        {
                // if not yet setup with specific code
            if(uwSystemBootErrorCode==0)
                uwSystemBootErrorCode=SYSTEMBOOTERR_INITTASKFAILBASE+uwInitAllExitCode;

            bSysStatLockedByBootError=1;
        }

            // unlock power section (FPGA)
        bUnLockPowerSection=TRUE;
    }

        // bootlock if start with power fail
    if(bStartWithPowerFail)
        bSysStatLockedByBootError=1;

        // if requested unlock power sections or startup wdt mgm
    if(bUnLockPowerSection)
    {
        SWORD swResult;

            // unlock power section for POST execution
        FpgaSafeUnLock(0);

            // execute POST
        swResult=Mh_POSTExecute();
        if(swResult)
        {
            uwSystemBootErrorCode=SYSTEMBOOTERR_POSTFAILURE+swResult;
            bSysStatLockedByBootError=1;
            bUnLockPowerSection=FALSE;
        }

#ifndef _APP_DEBUG
            // if not requested disable wdt, enable it
        if(uwSystemDisableWDT!=SYSTEM_DISABLEWDT_KEY)
            if(!WdtMgm_Start())
            {
                uwSystemBootErrorCode=SYSTEMBOOTERR_WDTINSTALLFAIL;
                bSysStatLockedByBootError=1;
                bUnLockPowerSection=FALSE;
            }
#endif
    }

        // if bootlocked and/or safe mode requested and/or start with powerfail
        // then setup proper warning and alarm
    if(bSysStatLockedByBootError)
    {
        if(uwSystemBootErrorCode!=0)
        {
            ulSystemWarnings|=SYSTEMWARNINGS_LOCKEDBYBOOTERROR;

            if(ulSystemAlarms==0l)
            {
                if(uwSystemBootErrorCode==SYSTEMBOOTERR_PARAMETERERROR)
                    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_BOOT_FAIL_FOR_INVALID_PAR,uwParChkParamCode,FALSE);
                else
                    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_BOOT_FAIL,uwSystemBootErrorCode,FALSE);
            }
        }
        else if(bStartWithPowerFail)
        {
            ulSystemWarnings|=SYSTEMWARNINGS_STARTWITHPOWERFAIL;
            uwSystemBootErrorCode=SYSTEMBOOTERR_STARTWITHPOWERFAIL;
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_BOOT_FAIL,uwSystemBootErrorCode,FALSE);
        }
        else
            ulSystemWarnings|=SYSTEMWARNINGS_SAFEMODESTARTUPREQUESTED;
    }

    // Timer_SetRTask( TaskSched_RTScheduler );

        // end of boot
    Os_EndCriticalSection(OS_CRITSECT_SWITCHANDEVENT);
    bSysStatBooting=0;
}

//****************************************************************************
// reset system recovery, called before CINIT to check if some special task
// has to be executed

void resetsystemrecovery(UWORD uwPar0, UWORD uwPar1)
{
        // if not clean reset requested then proceed to process the
        // reset cause
    if(uwPar0==~uwPar1 && uwPar0==RESETCODE_POFSTOREDATA)
    {
        hardware_config();

        SysLogMgm_PowerOnFail();
    }
#ifdef _APP_XC
        // if clean reset requested temporarily verify and restore parameters
        // in order to decode which system mode was requested, to re-issue
        // the proper reset
    else if(uwPar0==~uwPar1 && uwPar0==RESETCODE_CLEANREBOOTREQUESTED)
    {
        SWORD swResult;

        hardware_config();

            // get early user buttons, if requested void parameters
        if(SKIPBYUSERBUTTONS())
            swResult=PARMGM_B_ERROR;
        else
        {
            parmgm_par_default();
            swResult=parmgm_par_verify();
#if (defined(_DEBUG_TRACES))
            xil_printf("SysAppStartup::resetsystemrecovery() : parmgm_par_verify() = %d\r\n", swResult);
#endif
            if(swResult==PARMGM_B_OK)
                swResult=parmgm_par_load(PARMGM_LOAD_COREONLY);

            if(swResult==PARMGM_B_ERASED)
                swResult=PARMGM_B_OK;
        }

        HwConfBoot(swResult==PARMGM_B_OK);
    }
#endif
}

//****************************************************************************
// reset system recovery from power on reset

void resetpoweronreset(void)
{
        // execute a soft reset in order to have all peripherals reset to
        // default values as bootblock make some setup
    SysRes_ExecuteReset((UWORD)RESETCODE_CLEANREBOOTREQUESTED, (UWORD)~RESETCODE_CLEANREBOOTREQUESTED);
}

static void install_freertos( void )
{
    /* Ensure no interrupts execute while the scheduler is in an inconsistent
    state.  Interrupts are automatically enabled when the scheduler is started. */
    portDISABLE_INTERRUPTS();

    // Switch to use the FreeRTOS vector table defined in port_asm_vectors.S in the RTOS library.
    vPortInstallFreeRTOSVectorTable();
}

int main()
{
#ifdef _DEBUG_TRACES
   	xil_printf("SysAppStartup::main()\r\n");
#endif

    /* Reset Recovery */
    SysRes_Recovery();

    /* Install RTOS */
    install_freertos();

    /* Start System Boot */
    SystemBoot();

    /* create main background task */
    (void)Os_TaskCreateEx(TaskSched_BackgroundScheduler,512,"BackGround Task");

    /* Start the RTOS scheduler. */
    vTaskStartScheduler();

    return 0;
}
