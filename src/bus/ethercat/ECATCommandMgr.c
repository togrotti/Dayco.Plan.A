/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright $)AB) 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ECATCommandMgr.c                                           */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : ECAT specific command manager                              */
/*                                                                          */
/****************************************************************************/

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "bus\ethercat\ECATCommandMgr.h"
#include "bus\canopen\CanOpenDs301.h"
#include "bus\canopen\CanOpenComDB.h"
#include "bus\canopen\CanOpenCOECommon.h"
#include "ECATEeprom_def.h"
#include "ECATEeprom.h"
#include "fpga\FpgaHandler.h"
#include "drive\HardwareUnitID.h"
#include "system\SysAppFpgaOptCodes.h"
#include "system\Os.h"
#include "common\TaskScheduler.h"
#include "system\SystemAlarms.h"
#include "ECATCommandMgrRT.h"
#include "bus\SyncManager.h"
#include "system\SysLogManagement.h"
#include "common\ParamStorageManagement.h"

#ifdef _APP_XC
#include "core\SystemReset.h"
//#include "ECATFileManager.h"
#include "system\GlobalResetCodes.h"
#include "system\SysAppGlobals.h"
#endif

#include "ecatslv.h"
#include "mailbox.h"
#include "ecatappl.h"
#include "emcy.h"
#include "ECATHw.h"

#include <string.h>

#ifndef _INFINEON_
/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)
#endif

//***************************************************************************
// Local defines

#define SLOWTASK_STACKSIZE              (OS_DEFAULTUSRSTATICSTACK+0x00C0)

#define PDO_WRKELSIZE                   ((CFG_DS301_PDO_RX_TOT+CFG_DS301_PDO_TX_TOT)*CFG_DS301_PDO_AVG_ELEM_PER_PDO)

#define EEPROMEMUL_SAVETIMEOUT          500         // msec

#define ECAT_A_LED1_PIN                 (EMIO_PIN_BASE+0)
#define ECAT_B_LED1_PIN                 (EMIO_PIN_BASE+1)

//***************************************************************************
// Globals

ECATCM_PARAM tEcatCMParam;
const ECATCM_PARAM  tEcatCMDefParam={{FALSE,TRUE,FALSE,FALSE}};//{{0,1,0,0}};

const UBYTE tEcatCMSyncMgrCommType[ECATCM_NO_OF_SYNCMANAGER]=
{
	SM_TYPE_MBOXOUT,
	SM_TYPE_MBOXIN,
	SM_TYPE_PDOUT,
	SM_TYPE_PDIN
};

ECATCM_PDO_MAPPING  tEcatCMTxPdoMap[CFG_DS301_PDO_TX_TOT];
const ECATCM_PDO_MAPPING  tEcatCMDefTxPdoMap[CFG_DS301_PDO_TX_TOT]=
{
	{1,{0x60410010}},
	{2,{0x60410010,0x60610008}},
	{2,{0x60410010,0x60640020}},
	{2,{0x60410010,0x606c0020}},
};

ECATCM_PDO_MAPPING  tEcatCMRxPdoMap[CFG_DS301_PDO_RX_TOT];
const ECATCM_PDO_MAPPING  tEcatCMDefRxPdoMap[CFG_DS301_PDO_RX_TOT]=
{
	{1,{0x60400010}},
	{2,{0x60400010,0x60600008}},
	{2,{0x60400010,0x607a0020}},
	{2,{0x60400010,0x60ff0020}},
};

ECATCM_SYNCMGR_PDOMAPPING  tEcatCMOutSyncPdoMap;
const ECATCM_SYNCMGR_PDOMAPPING  tEcatCMDefOutSyncPdoMap=
{
	1,{DS301_PDO_RXMAP_BASEIDX},
};
ECATCM_SYNCMGR_PDOMAPPING  tEcatCMInSyncPdoMap;
const ECATCM_SYNCMGR_PDOMAPPING  tEcatCMDefInSyncPdoMap=
{
	1,{DS301_PDO_TXMAP_BASEIDX},
};

ECATCM_TSYNCMANDIAG tEcatCMSyncManOutDiag;
ECATCM_TSYNCMANDIAG tEcatCMSyncManInDiag;
ECATCM_TSYNCMANPAR tEcatCMSMOutParam;
ECATCM_TSYNCMANPAR tEcatCMSMInParam;
ECATCM_TCYCLEDIAG tEcatCMCycleDiag;
ECATCM_IN sEcatCMIn;
ECATCM_DIAG sEcatDiag;

UWORD uwEcatEmcyErrCode=0;
UBYTE ubEcatEmcyErrReg=0;
ULONG ulEcatEmcyErrMask=0ul;

//***************************************************************************
// Locals

static ECATMGR_RT_PDOFASTENTRY_ELEMENT  sPdoWrkEl[PDO_WRKELSIZE+2];
static BOOL  bECATModuleDisabled=FALSE;
static BOOL  bReSyncEnabled=FALSE;
static BOOL  bPdoValid=FALSE;
static BOOL  bAlShutdown=FALSE;
static BOOL  bEEmulSaveReq=FALSE;
static UWORD uwEEmulTimer;

static ULONG ulGetAlEntryActTime;
static ULONG ulGetAlEntryAlrmStat=0ul;

//***************************************************************************
// Local prototypes

static void slowtaskmanager(void);
static void emcy_handler(void);

static UWORD createpdoevent(void);
static UWORD destroypdoevent(void);
static UWORD checkandcompilepdo(UWORD, BOOL, ECATMGR_RT_PDOFASTENTRY_ELEMENT *, UWORD *, UWORD *);

//***************************************************************************
// Init handler

BOOL EcatCM_Init(UWORD uwOption)
{
    switch(uwOption)
    {
        case ECATCM_INIT_ALWAYS:
#ifndef _HW_VERSION_C
#ifdef _HW_DC
                // if EtherCAT hardware is available
            if(sGlbOptAvailable.ulHardwareOpt&HWUNITID_ETH_DBLRMII_1)
#endif
            {
                    // set ports as output and set leds off
                    // even if module is disabled
                Gpio_SetMode(ECAT_A_LED1_PIN, GPIO_DIR_OUT);
                Gpio_SetMode(ECAT_B_LED1_PIN, GPIO_DIR_OUT);
                GPIO_OUT(ECAT_A_LED1_PIN, TRUE);
                GPIO_OUT(ECAT_B_LED1_PIN, TRUE);
            }
#endif
        // only for som board
#ifdef _HW_SOM
            timer_wait(uwSysTimers1ms,10);
            UINT16 u16PhyData;
            HW_MIIMRead(0, 0x1F, &u16PhyData);
            HW_MIIMWrite(0, 0x1F, u16PhyData|0x90);	// Reference clock select
            HW_MIIMRead(1, 0x1F, &u16PhyData);
            HW_MIIMWrite(1, 0x1F, u16PhyData|0x90);	// Reference clock select
#endif
            break;
        
        case ECATCM_INIT_CORE:
                // if disabled do nothing
            if(!tEcatCMParam.flags.f.bEnableModule || bECATModuleDisabled)
            {
                bECATModuleDisabled=TRUE;
                
#ifndef _HW_VERSION_C
                Gpio_SetMode(ECAT_A_LED1_PIN, GPIO_DIR_OUT);
                Gpio_SetMode(ECAT_B_LED1_PIN, GPIO_DIR_OUT);
                GPIO_OUT(ECAT_A_LED1_PIN, FALSE);
                GPIO_OUT(ECAT_B_LED1_PIN, FALSE);
#endif

                return TRUE;
            }

            FPGA_ECATREGS_SET_ENMAC = TRUE;

                // init EEPROM emulation
            if(!ECATEE_DataInit())
            {
                bECATModuleDisabled=TRUE;
                return FALSE;
            }

            // only for hardware version C board,use the phy chip YT8512
#ifdef _HW_VERSION_C 
			// enable phy
			timer_wait(uwSysTimers1ms,200);
			FPGA_ECATREGS_SET_ENPHY=TRUE;
			timer_wait(uwSysTimers1ms,200);
			UINT16 u16PhyData =0;
			//select the 50MHZ 
			HW_MIIMWrite(1, 0x1E, 0x0050); // Read ext reg 0x50 select the 50MHZ clock
			HW_MIIMRead(1, 0x1F, &u16PhyData); // Read ext_reg 0x50 value to u16phyData
			HW_MIIMWrite(0, 0x1E, 0x0050); // Write ext reg 0x50 bit6 to 1
			HW_MIIMWrite(0, 0x1F, u16PhyData|0x0040); // Write ext reg 0x50 bit6 to 1
			HW_MIIMWrite(1, 0x1E, 0x0050); // Write ext reg 0x50 bit6 to 1
			HW_MIIMWrite(1, 0x1F, u16PhyData|0x0040); // Write ext reg 0x50 bit6 to 1

			//HW_MIIMWrite(0, 0x1E, 0x4000); // Read ext reg 0x4000 set to RX_DV
			//HW_MIIMRead(0, 0x1F, &u16PhyData); // Read ext_reg 0x4000 value to u16phyData
			//HW_MIIMWrite(0, 0x1E, 0x4000); // Write ext reg 0x4000 bit4 to 1
			//HW_MIIMWrite(0, 0x1F, u16PhyData|0x0010); // Write ext reg 0x4000 bit4 to 1

			HW_MIIMRead(1, 0x00, &u16PhyData); // Read reg 0x0000 set to soft reset
			HW_MIIMWrite(0, 0x00, u16PhyData|0x8000); // Write reg 0x0000 bit15 to 1
			HW_MIIMWrite(1, 0x00, u16PhyData|0x8000); // Write reg 0x0000 bit15 to 1
			//disable the phy address 0 broadcast
			timer_wait(uwSysTimers1ms,50);
			HW_MIIMWrite(1, 0x1E, 0x0000); // Read ext reg 0
			HW_MIIMRead(1, 0x1F, &u16PhyData); // Read ext_reg 0 value to u16phyData
			HW_MIIMWrite(0, 0x1E, 0x0000); // Write ext reg 0 bit6 to 0
			HW_MIIMWrite(0, 0x1F, u16PhyData&0xFFBF); // Write ext reg 0 bit6 to 0
			
#endif

                // internal flags
            bPdoValid=bEcatCMRTPdoError=FALSE;
            bReSyncEnabled=tEcatCMParam.flags.f.bReSyncEnable;
            bSysStatSyncMgrRequired|=bReSyncEnabled;
            bEcatCMRTDelayTxPdo=tEcatCMParam.flags.f.bDelayedInputs;

                // initialize local
            atomic_read(&ulGetAlEntryActTime, &ulSysTimersTotalPowerOnTime, sizeof(ulSysTimersTotalPowerOnTime));

                // add slow task
            if(!Os_TaskCreateEx(&slowtaskmanager,SLOWTASK_STACKSIZE,"ECAT_MGR"))
                return FALSE;

            break;

        case ECATCM_INIT_SYNCEVENT:

                // if disabled do nothing
            if(bECATModuleDisabled)
                return TRUE;
        
                // install realtime task for sync event
            if(!TaskSched_AddRTTask(&EcatCM_RT_SyncEvent, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0))
                return FALSE;

            break;

        case ECATCM_INIT_SYNCPDOEVENT:
                // if disabled do nothing
            if(bECATModuleDisabled)
                return TRUE;
        
                // install realtime task for pdo handling
            if(!TaskSched_AddRTTask(&EcatCM_RT_SyncPdoProcessing, TASKSCHEDULER_FLAG_NONE, \
                    SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_FIELDBUS_SYNCING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_ETHERCAT_SYNC), \
                    SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_FIELDBUS_SYNCING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_ETHERCAT_SYNC)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_RESETTING), 0))
                return FALSE;
            break;

        default:
            assert(FALSE);
    }

    return TRUE;
}

//***************************************************************************
// Hardware option callback

void EcatCM_GetHwOpt(UWORD w, ULONG * pulHwOpt, ULONG * pulFPGAOpt)
{
    if(tEcatCMParam.flags.f.bEnableModule && !bECATModuleDisabled)
    {
        *pulHwOpt  |=HWUNITID_ETH_DBLRMII_1;
        *pulFPGAOpt|=FPGA_HW_ETHERCAT;
    }
}

//***************************************************************************
// Return if state machine is SAFEOP or OP

BOOL EcatCM_IsALStateMachineOP(void)
{
    return (nAlStatus&STATE_MASK)==STATE_OP || (nAlStatus&STATE_MASK)==STATE_SAFEOP;
}

//***************************************************************************
// loop for ethercat control and management

static void slowtaskmanager(void)
{
    UWORD uwSetLedStatus=32;

        // wait until booting
    while(bSysStatBooting)
        Os_Sleep(0);

        // enable phy
    FPGA_ECATREGS_SET_ENPHY=TRUE;
        // 1ms to let phy autoconfig
    timer_wait(uwSysTimers100ns, 10000);
        // enable mac
    FPGA_ECATREGS_SET_ENMAC=TRUE;

        // ESC init
    HW_Init();
    ECAT_Init();

    for(;;)
    {
        ECAT_Main();
        if((nAlStatus&STATE_MASK)==STATE_PREOP || (nAlStatus&STATE_MASK)==STATE_SAFEOP || (nAlStatus&STATE_MASK)==STATE_OP)
            emcy_handler();

            // try to setup LED on PHY, in case of failure
            // retry N times, if success stop trying
        if(uwSetLedStatus)
        {
            if(HW_SetPHYLeds())
                uwSetLedStatus=0;
            else
            {
                uwSetLedStatus--;
                // 1ms to let phy autoconfig
                timer_wait(uwSysTimers100ns, 10000);
            }
        }

            // if fieldbus resync enabled, sync not valid and DC or SM sync enabled
        if(bEcatCMRTReSyncEnable && !sSyncMgrDiagnosticOut.bValid )
                // if state OP and power enabled then trigger back to PREOP
            if((nAlStatus&STATE_MASK)==STATE_OP && bSysStatPowerEnabled)
                ECAT_StateChange(STATE_PREOP,ALSTATUSCODE_SYNCERROR);
        tEcatCMSyncManOutDiag.bSyncError=!sSyncMgrDiagnosticOut.bValid;

            // if state change from OP to others and power enabled then trigger power section emergency
            // (if not disabled by parameter)
        if(bAlShutdown)
        {
            if(bSysStatPowerEnabled && !tEcatCMParam.flags.f.bDisableAlarms)
            {
                if(sEcatCMIn.psCtrlHandlers->pfEmcyStop!=NULL)
                    (*sEcatCMIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL);

                    // force diagnostic refresh to get AL status code
                HW_DiagRefresh(&sEcatDiag);

                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_ETHERCAT_FAIL, (ULONG)sEcatDiag.uwALStatusCode, FALSE);
            }
            bAlShutdown=FALSE;
        }

            // if save requested from EEPROM emulation module
        if(bECATEESaveRequested && !bEEmulSaveReq)
        {
                // then start timer before saving, timer is restarted
                // everytime save request is set
            bEEmulSaveReq=TRUE;
            bECATEESaveRequested=FALSE;

            uwEEmulTimer=timer_settimeout(uwSysTimers1ms, EEPROMEMUL_SAVETIMEOUT);
        }

            // if save pending
        if(bEEmulSaveReq && timer_istimedout(uwSysTimers1ms, uwEEmulTimer))
        {
            bEEmulSaveReq=FALSE;

            if(parmgm_par_save()!=PARMGM_B_OK)
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_HW_FLASH_FAIL, SYSTEMALARMS_SUBCODE_HF_FLASH_PARAMS, FALSE);
        }

#ifdef _APP_XC
            // reset request from file manager after firmware download
//        if(bECATFMReqReset)
//        {
//                // resetting flag and syslog flush flag
//            bSysStatResetting=TRUE;
//            bSysStatSysLogFlush=TRUE;
//
//                // Wait system minimum delay
//            timer_wait(uwOsFreeRunTimer1kHz, SW_RESET_DELAY);
//
//                // wait until syslog is safely written
//            while(bSysStatSysLogFlush);
//
//                // then reset requesting upgrade if required
//            if(bECATFMReqReflash)
//                SysRes_ExecuteReset(RESETCODE_ENTERBOOTANDUPGRADE, ~RESETCODE_ENTERBOOTANDUPGRADE);
//            else
//                SysRes_ExecuteReset(SYSRES_POWERON_VALUE_PAR0, SYSRES_POWERON_VALUE_PAR1);
//        }
#endif

            // refresh diagnostics
        HW_DiagRefresh(&sEcatDiag);

//        Os_Sleep(0);
        vTaskDelay(0);
    }
}

//***************************************************************************
// Emergency handler

static void emcy_handler(void)
{
	TEMCYMESSAGE * pEmcy;
//    ULONG ulBackupTime;
    ULONG ulAlarm;
    UWORD uwAlarmSubCode;
    UBYTE ubAlarmClass;

        // backup local time
//    ulBackupTime=ulGetAlEntryActTime;

        // if new alarm
    if(CanOpenCOE_GetAlarmEntry(&ulGetAlEntryActTime, &ulAlarm, &uwAlarmSubCode, &ubAlarmClass, &ulGetAlEntryAlrmStat))
    {        
/*
    	if ((pEmcy=EMCY_GetEmcyBuffer())==NULL)
        {   // if no more space in queue then restore backup time
            ulGetAlEntryActTime=ulBackupTime;
        }     

        else
*/
        {   // otherwise process alarm, one alarm per cycle
                // check if reset alarms
            if(uwAlarmSubCode==0)
            {
                uwEcatEmcyErrCode=0;
                ubEcatEmcyErrReg=0;
                ulEcatEmcyErrMask=0ul;
            }
                // otherwise setup and send EMCY
            else
            {
                    // record actual alarm
                uwEcatEmcyErrCode=uwAlarmSubCode;
                ubEcatEmcyErrReg|=ubAlarmClass|DS301_REGERR_GENERIC;
                ulEcatEmcyErrMask|=ulAlarm;

                    // setup EMCY message
            	pEmcy=EMCY_GetEmcyBuffer() ;
                pEmcy->Emcy.Word[EMCY_OFFS_ERRORCODE]=uwAlarmSubCode;
                pEmcy->Emcy.Byte[EMCY_OFFS_ERRORREGISTER]=ubEcatEmcyErrReg;
                pEmcy->Emcy.Byte[EMCY_OFFS_DIAGCODE  ]=(UBYTE)((ulEcatEmcyErrMask&0x000000ffl)    );
                pEmcy->Emcy.Byte[EMCY_OFFS_DIAGCODE+1]=(UBYTE)((ulEcatEmcyErrMask&0x0000ff00l)>>8 );
                pEmcy->Emcy.Byte[EMCY_OFFS_DIAGCODE+2]=(UBYTE)((ulEcatEmcyErrMask&0x00ff0000l)>>16);
                pEmcy->Emcy.Byte[EMCY_OFFS_DIAGCODE+3]=(UBYTE)((ulEcatEmcyErrMask&0xff000000l)>>24);

                EMCY_SendEmergency(pEmcy);
            }
        }
    }
}

//***************************************************************************
// Callback per transizione INIT->PREOP

UINT16 APPL_StartMailboxHandler(void)
{
    return 0;
}

//***************************************************************************
// Callback per transizione PREOP->INIT

UINT16 APPL_StopMailboxHandler(void)
{
    bAlShutdown=TRUE;

    return 0;
}

//***************************************************************************
// Callback per generazione mapping input/output (PREOP->SAFEOP)

UINT16 APPL_GenerateMapping(void)
{
    UWORD uwRetVal=createpdoevent();

    if(uwRetVal!=ALSTATUSCODE_NOERROR)
    {
        return uwRetVal;
    }
    tEcatCMCycleDiag.smEventMissedCounter=0l;

    return 0;
}

//***************************************************************************
// Callback per aggiornamento input durante transizione PREOP->SAFEOP

UINT16 APPL_StartInputHandler(UINT16 * w)
{
    if(!bPdoValid)
        return ALSTATUSCODE_INCONSISTENTSETTINGS;

        // enable resync
    if( bReSyncEnabled && (bDcSyncActive || bEscIntEnabled) )
        bEcatCMRTReSyncEnable=TRUE;
    else
        bEcatCMRTReSyncEnable=FALSE;
    FPGA_ECATREGS_SET_LATCHSYN0=bEcatCMRTReSyncEnable && bDcSyncActive;

    bEcatCMRTPdoError|=!EcatCM_RT_TxEncode(ptEcatCMRTPdoTxElemList,(HPVOID)(FPGA_ETHERNET_BASE_ADDRESS+nEscAddrInputData),uwEcatCMRTPdoTxWordBufSize);

    return 0;
}

//***************************************************************************
// Callback per transizione SAFEOP->PREOP

UINT16 APPL_StopInputHandler(void)
{
    bAlShutdown=TRUE;
    nPdOutputSize=nPdInputSize=0;
    bEcatCMRTReSyncEnable=FALSE;

    return destroypdoevent();
}

//***************************************************************************
// Callback per transizione SAFEOP->OP

UINT16 APPL_StartOutputHandler(void)
{
    if(!bPdoValid)
    {
        return ALSTATUSCODE_INCONSISTENTSETTINGS;
    }

    return 0;
}

//***************************************************************************
// Callback per transizione OP->SAFEOP

UINT16 APPL_StopOutputHandler(void)
{
    bAlShutdown=TRUE;

    return 0;
}

//***************************************************************************
// Inizializza timer in free run mode

void APPL_StartFreeRunTimer(void)
{
}

//***************************************************************************
// Hook for object mapping management

UWORD EcatCM_PdoMap(COMMONPARAMDB_ENTRY  * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG pl)
{
    ECATCM_PDO_MAPPING  * sPar;
    BOOL bIsRxPdo;
    UWORD uwSize;

    if(uwFlags&COMMONPARAMDB_CBFLAG_INFOREQ)
    {
        COMMONPARAMDB_ENTRY  * hpsInfo=hpvBuffer;

        *hpsInfo=*hpsEntry;

        if(uwElement)
        {
            hpsInfo->ubFlags=COMMONPARAMDB_FLAG_RW;
            hpsInfo->ubType=COMMONPARAMDB_TYPE_ULONG;
            hpsInfo->uwNElements=0;
        }
        else
        {
            hpsInfo->ubFlags=COMMONPARAMDB_FLAG_RW;
            hpsInfo->ubType=COMMONPARAMDB_TYPE_UBYTE;
            hpsInfo->swMoreType=CFG_DS301_MAXDATAOBJECT;
        }

        return COMMONPARAMDB_CH_OK;
    }

        // get pointer to PDO param structure
    sPar=(ECATCM_PDO_MAPPING  *)hpsEntry->hpvData;

        // if abort do nothing
    if(uwFlags&COMMONPARAMDB_CBFLAG_ABORT)
        return COMMONPARAMDB_CH_OK;

        // get PDO type
    bIsRxPdo=(hpsEntry->swMoreType==ECATCM_PAR_RX);

        // check element access
    if(uwElement>CFG_DS301_MAXDATAOBJECT)
        return COMMONPARAMDB_CH_INVALID_ELEMENT;

        // calculate element size
    uwSize=(uwElement?sizeof(ECATCM_PDO_MAP_ENTRY):sizeof(UBYTE));

        // if data read
    if(uwFlags&COMMONPARAMDB_CBFLAG_RD)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
                // data size if requested
            if(uwFlags&COMMONPARAMDB_CBFLAG_SIZEINQUIRY)
            {
                *((HPULONG)hpvBuffer)=uwSize;
                *puwBufSize=sizeof(ULONG);
            }
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            UWORD uwCt;

            if(uwFlags&COMMONPARAMDB_CBFLAG_NODSCHECK)
            {
                if(*puwBufSize<uwSize)
                    return COMMONPARAMDB_CH_WRONGLENGTH;
            }
            else
                if(*puwBufSize!=uwSize)
                    return COMMONPARAMDB_CH_WRONGLENGTH;

                // element
            if(uwElement)
                memcpy(hpvBuffer, &(sPar->Map[uwElement-1]), sizeof(ECATCM_PDO_MAP_ENTRY));

                // no. of elements
            else
                *((HPUBYTE)hpvBuffer)=sPar->NoMapped;

            for(uwCt=uwSize;uwCt<*puwBufSize;uwCt++)
                ((HPUBYTE)hpvBuffer)[uwCt]=0;
            *puwBufSize=uwSize;

        }
    }

        // if data write
    if(uwFlags&COMMONPARAMDB_CBFLAG_WR)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
                // accept zero length for clients that
                // do not send data length
            if(*((HPULONG)hpvBuffer)!=0l)
            {
                if(uwFlags&COMMONPARAMDB_CBFLAG_NODSCHECK)
                {
                    if(*((HPULONG)hpvBuffer)<uwSize)
                        return COMMONPARAMDB_CH_WRONGLENGTH;
                }
                else
                    if(*((HPULONG)hpvBuffer)!=uwSize)
                        return COMMONPARAMDB_CH_WRONGLENGTH;
            }
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            if(*puwBufSize<uwSize)
                return COMMONPARAMDB_CH_WRONGLENGTH;

                // map write
            if(uwElement)
            {
                ECATCM_PDO_MAP_ENTRY sMapElement;
                const CANOPENCOMDB_ENTRY  * ptEntry;
                UBYTE ubObjLen=0;

                    // get map element
                memcpy(&sMapElement, hpvBuffer, sizeof(ECATCM_PDO_MAP_ENTRY));

                    // accept zero as null object
                if(sMapElement.dMap!=0l)
                {
                    ULONG ulRetVal;

                        // find element
                    ulRetVal=CanOpenComDBEntrySearch(sMapElement.f.wIndex,sMapElement.f.bSubIndex,CANOPENCOMDB_F_ECATCOE_VALID,&ptEntry);
                    if(ulRetVal!=DS301_SDOOK)
                    {
                        if(ulRetVal==DS301_SDOABORT_INVALIDSUBINDEX)
                            return COMMONPARAMDB_CH_INVALID_ELEMENT;
                        else
                            return COMMONPARAMDB_CH_ELEMENT_NOTFOUND;
                    }

                        // check if mappable in PDO
                    if(!(ptEntry->uwFlags&CANOPENCOMDB_F_PDOMAPPABLE))
                        return COMMONPARAMDB_CH_NOTMAPPABLE;

                        // if array and the selected element is zero then error
                    if(ptEntry->hpsComDBEntry->uwNElements>1 && sMapElement.f.bSubIndex==0)
                        return COMMONPARAMDB_CH_NOTMAPPABLE;

                        // if RX check for object writable
                    if(bIsRxPdo)
                    {
                        if(ptEntry->uwFlags&CANOPENCOMDB_F_WRDENYWHENOPER)
                            return COMMONPARAMDB_CH_NO_WRITE_ACCESS;

                        if(!(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_WR))
                            return COMMONPARAMDB_CH_NO_WRITE_ACCESS;
                    }
                        // otherwise if TX check for object readable
                    else
                        if(!(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_RD))
                            return COMMONPARAMDB_CH_NO_READ_ACCESS;

                    ubObjLen=(UBYTE)ComParDBEntryGetEntrySize(ptEntry->hpsComDBEntry);

                        // check for length
                    if(ubObjLen==0 || (ubObjLen*8)!=sMapElement.f.bLength)
                        return COMMONPARAMDB_CH_WRONGLENGTH;
                }

                atomic_write(&sPar->Map[uwElement-1], &sMapElement, sizeof(sMapElement));
            }
                // write no. of elements
            else
            {
                if(*((HPUBYTE)hpvBuffer)>CFG_DS301_MAXDATAOBJECT)
                    return COMMONPARAMDB_CH_WRONGLENGTH;

                sPar->NoMapped=*((HPUBYTE)hpvBuffer);
            }
        }
    }

    return COMMONPARAMDB_CH_OK;
}

//***************************************************************************
// Hook for sync manager pdo mapping

UWORD EcatCM_SyncMgrPdoMap(COMMONPARAMDB_ENTRY  * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG pl)
{
    ECATCM_SYNCMGR_PDOMAPPING  * sPar;
    BOOL bIsRxPdo;
    UWORD uwSize;

    if(uwFlags&COMMONPARAMDB_CBFLAG_INFOREQ)
    {
        COMMONPARAMDB_ENTRY  * hpsInfo=hpvBuffer;

        *hpsInfo=*hpsEntry;

        if(uwElement)
        {
            hpsInfo->ubFlags=COMMONPARAMDB_FLAG_RW;
            hpsInfo->ubType=COMMONPARAMDB_TYPE_UWORD;
            hpsInfo->uwNElements=0;
        }
        else
        {
            hpsInfo->ubFlags=COMMONPARAMDB_FLAG_RW;
            hpsInfo->ubType=COMMONPARAMDB_TYPE_UBYTE;
            hpsInfo->swMoreType=ECATCM_MAXNPDO;
        }

        return COMMONPARAMDB_CH_OK;
    }

        // get pointer to sync mgr PDO map param structure
    sPar=(ECATCM_SYNCMGR_PDOMAPPING  *)hpsEntry->hpvData;

        // if abort do nothing
    if(uwFlags&COMMONPARAMDB_CBFLAG_ABORT)
        return COMMONPARAMDB_CH_OK;

        // get PDO type
    bIsRxPdo=(hpsEntry->swMoreType==ECATCM_PAR_RX);

        // check element access
    if(uwElement>ECATCM_MAXNPDO)
        return COMMONPARAMDB_CH_INVALID_ELEMENT;

        // calculate element size
    uwSize=(uwElement?sizeof(UWORD):sizeof(UBYTE));

        // if data read
    if(uwFlags&COMMONPARAMDB_CBFLAG_RD)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
                // data size if requested
            if(uwFlags&COMMONPARAMDB_CBFLAG_SIZEINQUIRY)
            {
                *((HPULONG)hpvBuffer)=uwSize;
                *puwBufSize=sizeof(ULONG);
            }
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            UWORD uwCt;

            if(uwFlags&COMMONPARAMDB_CBFLAG_NODSCHECK)
            {
                if(*puwBufSize<uwSize)
                    return COMMONPARAMDB_CH_WRONGLENGTH;
            }
            else
                if(*puwBufSize!=uwSize)
                    return COMMONPARAMDB_CH_WRONGLENGTH;

                // element
            if(uwElement)
                memcpy(hpvBuffer, &(sPar->Map[uwElement-1]), sizeof(UWORD));

                // no. of elements
            else
                *((HPUBYTE)hpvBuffer)=sPar->NoMapped;

            for(uwCt=uwSize;uwCt<*puwBufSize;uwCt++)
                ((HPUBYTE)hpvBuffer)[uwCt]=0;
            *puwBufSize=uwSize;

        }
    }

        // if data write
    if(uwFlags&COMMONPARAMDB_CBFLAG_WR)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
                // accept zero length for clients that
                // do not send data length
            if(*((HPULONG)hpvBuffer)!=0l)
            {
                if(uwFlags&COMMONPARAMDB_CBFLAG_NODSCHECK)
                {
                    if(*((HPULONG)hpvBuffer)<uwSize)
                        return COMMONPARAMDB_CH_WRONGLENGTH;
                }
                else
                    if(*((HPULONG)hpvBuffer)!=uwSize)
                        return COMMONPARAMDB_CH_WRONGLENGTH;
            }
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            if(*puwBufSize<uwSize)
                return COMMONPARAMDB_CH_WRONGLENGTH;

                // map write
            if(uwElement)
            {
                UWORD uwMapElement;

                    // get map element
                memcpy(&uwMapElement, hpvBuffer, sizeof(UWORD));

                    // accept zero as null object
                if(uwMapElement!=0l)
                {
                    if(bIsRxPdo)
                        if(uwMapElement<DS301_PDO_RXMAP_BASEIDX || uwMapElement>=DS301_PDO_RXMAP_BASEIDX+CFG_DS301_PDO_RX_TOT)
                            return COMMONPARAMDB_CH_INVALID_DATA;

                    if(!bIsRxPdo)
                        if(uwMapElement<DS301_PDO_TXMAP_BASEIDX || uwMapElement>=DS301_PDO_TXMAP_BASEIDX+CFG_DS301_PDO_TX_TOT)
                            return COMMONPARAMDB_CH_INVALID_DATA;
                }

                atomic_write(&sPar->Map[uwElement-1], &uwMapElement, sizeof(uwMapElement));
            }
                // write no. of elements
            else
            {
                if(*((HPUBYTE)hpvBuffer)>ECATCM_MAXNPDO)
                    return COMMONPARAMDB_CH_WRONGLENGTH;

                sPar->NoMapped=*((HPUBYTE)hpvBuffer);
            }
        }
    }

    return COMMONPARAMDB_CH_OK;
}

//***************************************************************************
// Create PDO event

static UWORD createpdoevent(void)
{
    UWORD uwPdoNum;
    UWORD uwLeftSize=PDO_WRKELSIZE;
    UWORD uwProcDataSize;
    UWORD uwRetVal=ALSTATUSCODE_NOERROR;

        // lock PDO configuration change
    bSysStatEcatPdoMgrLockConfig=TRUE;

        // reset PDO valid
    bPdoValid=FALSE;

        // create RX list for output mapping
    uwProcDataSize=MAX_PD_OUTPUT_SIZE;
    for(uwPdoNum=0;uwPdoNum<tEcatCMOutSyncPdoMap.NoMapped;uwPdoNum++)
        if(tEcatCMOutSyncPdoMap.Map[uwPdoNum]!=0)
        {
            UWORD uwPdoIdx=tEcatCMOutSyncPdoMap.Map[uwPdoNum]-DS301_PDO_RXMAP_BASEIDX;

            if(uwPdoIdx>=CFG_DS301_PDO_RX_TOT)
            {
                uwRetVal=ALSTATUSCODE_INCONSISTENTSETTINGS;
                goto processingerror;
            }

            uwRetVal=checkandcompilepdo(uwPdoIdx, TRUE, &sPdoWrkEl[PDO_WRKELSIZE-uwLeftSize], &uwLeftSize, &uwProcDataSize);

            if(uwRetVal!=ALSTATUSCODE_NOERROR)
            {
                if(uwRetVal==ALSTATUSCODE_NOVALIDINPUTSANDOUTPUTS)
                    uwRetVal=ALSTATUSCODE_INVALIDOUTPUTMAPPING;

                goto processingerror;
            }
        }

        // terminator
    sPdoWrkEl[PDO_WRKELSIZE-uwLeftSize].pvDataAddress=NULL;

        // setup for RT processing
    nPdOutputSize=MAX_PD_OUTPUT_SIZE-uwProcDataSize;
    uwEcatCMRTPdoRxWordBufSize=(nPdOutputSize+1)/2;

        // setup addresses
    ptEcatCMRTPdoRxElemList=&sPdoWrkEl[0];
    ptEcatCMRTPdoTxElemList=&sPdoWrkEl[PDO_WRKELSIZE+1-uwLeftSize];

        // create TX list for input mapping
    uwProcDataSize=MAX_PD_INPUT_SIZE;
    for(uwPdoNum=0;uwPdoNum<tEcatCMInSyncPdoMap.NoMapped;uwPdoNum++)
        if(tEcatCMInSyncPdoMap.Map[uwPdoNum]!=0)
        {
            UWORD uwPdoIdx=tEcatCMInSyncPdoMap.Map[uwPdoNum]-DS301_PDO_TXMAP_BASEIDX;

            if(uwPdoIdx>=CFG_DS301_PDO_TX_TOT)
            {
                uwRetVal=ALSTATUSCODE_INCONSISTENTSETTINGS;
                goto processingerror;
            }

            uwRetVal=checkandcompilepdo(uwPdoIdx, FALSE, &sPdoWrkEl[PDO_WRKELSIZE+1-uwLeftSize], &uwLeftSize, &uwProcDataSize);

            if(uwRetVal!=ALSTATUSCODE_NOERROR)
            {
                if(uwRetVal==ALSTATUSCODE_NOVALIDINPUTSANDOUTPUTS)
                    uwRetVal=ALSTATUSCODE_INVALIDINPUTMAPPING;

                goto processingerror;
            }
        }

        // terminator
    sPdoWrkEl[PDO_WRKELSIZE+1-uwLeftSize].pvDataAddress=NULL;

        // setup for RT processing
    nPdInputSize=MAX_PD_INPUT_SIZE-uwProcDataSize;
    uwEcatCMRTPdoTxWordBufSize=(nPdInputSize+1)/2;

        // PDO valid
    bPdoValid=TRUE;

processingerror:
        // unlock PDO configuration change
    bSysStatEcatPdoMgrLockConfig=FALSE;

    return uwRetVal;
}

//***************************************************************************
// Destroy PDO event

static UWORD destroypdoevent(void)
{
        // reset PDO valid
    bPdoValid=FALSE;

    return ALSTATUSCODE_NOERROR;
}

//***************************************************************************
// check configuration and compile PDO for fast processing

static UWORD checkandcompilepdo(UWORD uwPdoNum, BOOL bIsRxPdo, ECATMGR_RT_PDOFASTENTRY_ELEMENT * psMapEl, UWORD * puwElArraySize, UWORD * puwProcDataSize)
{
    ECATCM_PDO_MAPPING * psParam;
    UWORD uwCt;

        // select parameters
    if(bIsRxPdo)
        psParam=&tEcatCMRxPdoMap[uwPdoNum];
    else
        psParam=&tEcatCMTxPdoMap[uwPdoNum];

        // check for valid mapping
    if(psParam->NoMapped>DS301_PDO_MAXDATAOBJECT)
        return ALSTATUSCODE_INCONSISTENTSETTINGS;

        // check for element map availability
    if(psParam->NoMapped>*puwElArraySize)
        return ALSTATUSCODE_NOVALIDINPUTSANDOUTPUTS;

        // begin mapping elements
    for(uwCt=0;uwCt<psParam->NoMapped;uwCt++)
    {
        ECATCM_PDO_MAP_ENTRY sMapElement=psParam->Map[uwCt];
        const CANOPENCOMDB_ENTRY  * ptEntry;
        UWORD uwSize;
        UWORD uwDummy;

            // check for valid object (must be not zero)
        if(sMapElement.dMap==0l)
            return ALSTATUSCODE_NOVALIDINPUTSANDOUTPUTS;

            // find element
        if(CanOpenComDBEntrySearch(sMapElement.f.wIndex,sMapElement.f.bSubIndex,CANOPENCOMDB_F_ECATCOE_VALID,&ptEntry)!=DS301_SDOOK)
            return ALSTATUSCODE_INCONSISTENTSETTINGS;

            // check if mappable in PDO
        if(!(ptEntry->uwFlags&CANOPENCOMDB_F_PDOMAPPABLE))
            return ALSTATUSCODE_INCONSISTENTSETTINGS;

            // if array and the selected element is zero then error
        if(ptEntry->hpsComDBEntry->uwNElements>1 && sMapElement.f.bSubIndex==0)
            return ALSTATUSCODE_INCONSISTENTSETTINGS;

            // if RX check for object writable
        if(bIsRxPdo)
        {
            if(ptEntry->uwFlags&CANOPENCOMDB_F_WRDENYWHENOPER)
                return ALSTATUSCODE_INCONSISTENTSETTINGS;

            if(!(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_WR))
                return ALSTATUSCODE_INCONSISTENTSETTINGS;
        }
            // otherwise if TX check for object readable
        else
            if(!(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_RD))
                return ALSTATUSCODE_INCONSISTENTSETTINGS;

            // map element, if subindex not zero map array element, but if found element is not
            // an array then all subindex are mapped with multiple entry, then address of base
            // element should be choose
        if(sMapElement.f.bSubIndex>0 && ptEntry->hpsComDBEntry->uwNElements>1)
            psMapEl[uwCt].pvDataAddress=(void *)ComParDBEntryGetPointer(ptEntry->hpsComDBEntry, sMapElement.f.bSubIndex-1, &uwDummy);
        else
            psMapEl[uwCt].pvDataAddress=(void *)ComParDBEntryGetPointer(ptEntry->hpsComDBEntry, 0, &uwDummy);

        switch(ptEntry->hpsComDBEntry->ubType)
        {
            case COMMONPARAMDB_TYPE_UBYTE:
            case COMMONPARAMDB_TYPE_SBYTE:
                if(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_UMCONV)
                {
                    psMapEl[uwCt].uwSelector=ECATCM_RT_SELECTOR_8BIT_UM;
                    psMapEl[uwCt].fpfUmConv=ptEntry->hpsComDBEntry->uf.fpfUmConv;
                }
                else
                {
                    psMapEl[uwCt].uwSelector=ECATCM_RT_SELECTOR_8BIT;
                    psMapEl[uwCt].fpfUmConv=NULL;
                }
                uwSize=sizeof(UBYTE);
                break;

            case COMMONPARAMDB_TYPE_UWORD:
            case COMMONPARAMDB_TYPE_SWORD:
                if(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_UMCONV)
                {
                    if(*puwProcDataSize&1)
                        psMapEl[uwCt].uwSelector=ECATCM_RT_SELECTOR_ODD16_UM;
                    else
                        psMapEl[uwCt].uwSelector=ECATCM_RT_SELECTOR_EVEN16_UM;
                    psMapEl[uwCt].fpfUmConv=ptEntry->hpsComDBEntry->uf.fpfUmConv;
                }
                else
                {
                    if(*puwProcDataSize&1)
                        psMapEl[uwCt].uwSelector=ECATCM_RT_SELECTOR_ODD16;
                    else
                        psMapEl[uwCt].uwSelector=ECATCM_RT_SELECTOR_EVEN16;
                    psMapEl[uwCt].fpfUmConv=NULL;
                }
                uwSize=sizeof(UWORD);
                break;

            case COMMONPARAMDB_TYPE_ULONG:
            case COMMONPARAMDB_TYPE_SLONG:
            case COMMONPARAMDB_TYPE_FLOAT:
                if(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_UMCONV)
                {
                    if(*puwProcDataSize&1)
                        psMapEl[uwCt].uwSelector=ECATCM_RT_SELECTOR_ODD32_UM;
                    else
                        psMapEl[uwCt].uwSelector=ECATCM_RT_SELECTOR_EVEN32_UM;
                    psMapEl[uwCt].fpfUmConv=ptEntry->hpsComDBEntry->uf.fpfUmConv;
                }
                else
                {
                    if(*puwProcDataSize&1)
                        psMapEl[uwCt].uwSelector=ECATCM_RT_SELECTOR_ODD32;
                    else
                        psMapEl[uwCt].uwSelector=ECATCM_RT_SELECTOR_EVEN32;
                    psMapEl[uwCt].fpfUmConv=NULL;
                }
                uwSize=sizeof(ULONG);
                break;

            case COMMONPARAMDB_TYPE_UQWRD:
            case COMMONPARAMDB_TYPE_SQWRD:
            case COMMONPARAMDB_TYPE_DOUBL:
                if(ptEntry->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_UMCONV)
                {
                    psMapEl[uwCt].uwSelector=ECATCM_RT_SELECTOR_64BIT_UM;
                    psMapEl[uwCt].fpfUmConv=ptEntry->hpsComDBEntry->uf.fpfUmConv;
                }
                else
                {
                    psMapEl[uwCt].uwSelector=ECATCM_RT_SELECTOR_64BIT;
                    psMapEl[uwCt].fpfUmConv=NULL;
                }
                uwSize=sizeof(UQWRD);
                break;

            default:
                return ALSTATUSCODE_INCONSISTENTSETTINGS;
        }

            // check process data size left
        if(uwSize>*puwProcDataSize)
            return ALSTATUSCODE_NOVALIDINPUTSANDOUTPUTS;

            // update size left
        *puwProcDataSize-=uwSize;
    }

        // update no. of elements left
    *puwElArraySize-=psParam->NoMapped;

    return ALSTATUSCODE_NOERROR;
}

