/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CanOpenDs301.c                                             */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : CiA CANOpen DS301, DS302 slave only                        */
/*                                                                          */
/****************************************************************************/

#include "common\CommonDefines.h"
#include "system\SysAppGlobals.h"
#include "system\SysAppConfig.h"
#include "common\CommonUtility.h"
#include "system\Os.h"
#include "CanOpenDs301Externals.h"
#include "CanOpenDs301.h"
#include <string.h>
#include <stdio.h>
#if CFG_CANDRV_DS301
#include "MultiCANController.h"
#endif
#pragma GCC optimize (2)
//***************************************************************************
// SDO mapping and commands

#define SDO_INITDOWNLOAD_REQ        1
#define SDO_INITDOWNLOAD_RESP       3
#define SDO_SEGDOWNLOAD_REQ         0
#define SDO_SEGDOWNLOAD_RESP        1

#define SDO_INITUPLOAD_REQ          2
#define SDO_INITUPLOAD_RESP         2
#define SDO_SEGUPLOAD_REQ           3
#define SDO_SEGUPLOAD_RESP          0

#define SDO_ABORT                   4

#define SDO_TIMEOUT                 8000        // * 100usec

//***************************************************************************
// Definizione bit nello status register

#define NMT_PREOPERATIONAL          0x0001
#define NMT_OPERATIONAL             0x0002
#define NMT_STOPPED                 0X0080

//***************************************************************************
// Definizione stato x error protocol

#define NMT_ERRPROT_STOP            0
#define NMT_ERRPROT_BOOTUP          1
#define NMT_ERRPROT_START           3
#define NMT_ERRPROT_RUN             2

//***************************************************************************
// Definizione bit nel command register

#define CMD_DOCANRESET              0x0001
#define CMD_DOCANRESFORPDO          0x0002
#define CMD_LSSSWITCHEDON           0x0004
#define CMD_NEW_NMTCOB              0x0008
#define CMD_COMMFAULT_RESET         0x0010
#define CMD_EMCY_FORCE              0x0020
#define CMD_ENTER_FULLYOPERATIVE    0x0040
#define CMD_FULLYOPERATIVE          0x0080

//***************************************************************************
// LSS commands

#define LSS_SWITCHGLOBAL            04
#define LSS_SWITCHSELECT_VENDOR     64
#define LSS_SWITCHSELECT_PRODUCT    65
#define LSS_SWITCHSELECT_REVISION   66
#define LSS_SWITCHSELECT_SERIAL     67
#define LSS_SWITCHEDSELECTIVE       68
#define LSS_CONFIG_NODEID           17
#define LSS_CONFIG_BITIMING         19
#define LSS_ACTIVATE_BITIMING       21
#define LSS_CONFIG_STORE            23
#define LSS_INQUIRE_VENDOR          90
#define LSS_INQUIRE_PRODUCT         91
#define LSS_INQUIRE_REVISION        92
#define LSS_INQUIRE_SERIAL          93
#define LSS_INQUIRE_NODEID          94
#define LSS_IDENT_VENDOR            70
#define LSS_IDENT_PRODUCT           71
#define LSS_IDENT_REVISION_LOW      72
#define LSS_IDENT_REVISION_HIGH     73
#define LSS_IDENT_SERIAL_LOW        74
#define LSS_IDENT_SERIAL_HIGH       75
#define LSS_IDENT_SLAVE             79
#define LSS_IDENT_NOCONFIGURED      76
#define LSS_IDENT_NOCONFIG_SLAVE    80
#define LSS_SWITCHGLOBAL_SERIAL     128

//***************************************************************************
// Globals

DS301_PARAM DS301_MEMQ_PARAM tDs301Param;

const DS301_PARAM DS301_MEMQ_PARAM tDs301DefParam=
{
    0,

    DS301_DEFNODEID,
    DS301_DEFTIMING,

    DS301_CREATECOBIDENTRY(0,DS301_COBID_SYNC),
    DS301_CREATECOBIDENTRY(0,DS301_COBID_EMERGENCY+DS301_DEFNODEID),
    0,

    0,
    0,
    0,

    0l
};

//***************************************************************************
// Globals for EMCY management

#ifdef CFG_DS301_EMCY
volatile UNSIGNED8   bDs301ErrorRegister;       // 1001h Error register
volatile UNSIGNED32  dDs301ErrRegManufacturer;  // 1002h Manufacturer error register
volatile UNSIGNED16  wDs301ErrCodeLast;         // 603fh Error code
#endif

//***************************************************************************
// Local data structure

typedef struct
{
    UNSIGNED8                       bCanController;
    UNSIGNED8                       bLssNodeId;
    UNSIGNED8                       bLssTimingIndex;

#ifdef CFG_DS301_ERRPROT
    UNSIGNED16                      wGuardTime;         // msec
    UNSIGNED16                      wHeartbeatTime;     // msec
    UNSIGNED8                       bLifeTimeFactor;
#endif

#ifdef CFG_DS301_EMCY
    DS301_COBIDENTRY                tEmcyCob;
#endif

#ifdef CFG_DS301_SYNC
    DS301_COBIDENTRY                tSyncCob;
#endif
} WRK_PARAM;

#if CFG_CANDRV_DS301
//***************************************************************************
// Locals

    // works parameters
static WRK_PARAM            tWrkParam;

    // general purpose tx cob
static volatile CANDRV_COB  tGPTxCob;

    // local generated errors
static volatile UNSIGNED16  wLocalErrors=0;

    // general commands across tasks
static volatile UNSIGNED16  wLocalCommands=0;

//***************************************************************************
// Locals for Error Control Protocol management

#ifdef CFG_DS301_ERRPROT
static volatile CANDRV_COB  tErrCtrlTxCob;
static volatile CANDRV_COB  tErrCtrlRxCob;
static UNSIGNED8            bErrCtrlRun;
#endif

//***************************************************************************
// Locals for NMT management

#ifdef CFG_DS301_NMT
static volatile CANDRV_COB  tNmtCob;
static volatile UNSIGNED8   bNmtRxCmd;
static UNSIGNED16           wNmtStatusReg;
#endif

//***************************************************************************
// Locals for SDO management

static volatile CANDRV_COB  tSdoRxCob;

//***************************************************************************
// Locals for EMCY management

#ifdef CFG_DS301_EMCY
static volatile CANDRV_COB  tEmcyObjCob;
#endif

//***************************************************************************
// Locals for LSS management

#ifdef CFG_DS301_LSS
static volatile CANDRV_COB tLssRxCob;
#endif

//***************************************************************************
// Locals for SYNC management

#ifdef CFG_DS301_SYNC
static volatile CANDRV_COB tSyncCob;
static BOOL bSyncCobInstalled=FALSE;
#endif

//***************************************************************************
// Local functions

static BOOL setup_cob_ids(void);
static void set_default_cobid(UNSIGNED8);
static void reset_comm(void);
static BOOL validate_generic_cobid(DS301_COBIDENTRY * cobid);

#ifdef CFG_DS301_ERRPROT
static void loop_errorprotocol(void);
#endif

#ifdef CFG_DS301_NMT
static void nmt_cob_event(CANDRV_COB *);
static void nmt_mgr(void);
#endif

#ifdef CFG_DS301_EMCY
static void emcy_mgr(void);
#endif

#ifdef CFG_DS301_LSS
static void lss_mgr(CANDRV_PCOB);
#endif

#ifdef CFG_DS301_SYNC
static void sync_mgr(void);
#endif

static void candrvstatus_mgr(void);
static void commfault_mgr(void);
static void sdo_mgr(CANDRV_PCOB sdotx);

//***************************************************************************
// Init

BOOL ds301_init(UNSIGNED8 selcanctrl, PUNSIGNED8 errorcode)
{
    *errorcode=DS301_INITERR_NONE;

        // get working parameters
    tWrkParam.bCanController    =selcanctrl;
    tWrkParam.bLssNodeId        =tDs301Param.bLssNodeId;
    tWrkParam.bLssTimingIndex   =tDs301Param.bLssTimingIndex;

#ifdef CFG_DS301_ERRPROT
    tWrkParam.wGuardTime        =tDs301Param.wGuardTime;
    tWrkParam.wHeartbeatTime    =tDs301Param.wHeartbeatTime;
    tWrkParam.bLifeTimeFactor   =tDs301Param.bLifeTimeFactor;
#endif

#ifdef CFG_DS301_EMCY
    tWrkParam.tEmcyCob          =tDs301Param.tEmcyCob;
#endif

#ifdef CFG_DS301_LSS
    tWrkParam.tSyncCob          =tDs301Param.tSyncCob;
#endif

        // check for valid node id
    if(!(tWrkParam.bLssNodeId>=DS301_LSS_NODEID_BEGIN && tWrkParam.bLssNodeId<=DS301_LSS_NODEID_END
#ifdef CFG_DS301_LSS
        || tWrkParam.bLssNodeId==DS301_LSS_UNCONFIGUREDNODEID
#endif
        ))
    {
        *errorcode=DS301_INITERR_WRONGNODEID;
        return FALSE;
    }

#ifdef CFG_DS301_LSS
    if(tWrkParam.bLssNodeId==DS301_LSS_UNCONFIGUREDNODEID)
        wLocalCommands|=CMD_LSSSWITCHEDON;
#endif

        // set bit timing
    if(!candrv_setspeed(tWrkParam.bCanController, tWrkParam.bLssTimingIndex))
    {
        *errorcode=DS301_INITERR_WRONGBAUDRATE;
        return FALSE;
    }

#ifdef CFG_DS301_NMT
    wNmtStatusReg=NMT_PREOPERATIONAL;

    tNmtCob.id=DS301_COBID_NMT;
    tNmtCob.len=1;
    memset(&tNmtCob.flags,0,sizeof(tNmtCob.flags));//tNmtCob.flags=CANDRV_F_DEFAULT;

    assert(candrv_addrxcob(tWrkParam.bCanController, &tNmtCob, nmt_cob_event));
#endif

#ifdef CFG_DS301_NMT
    tLssRxCob.id=DS301_COBID_LSSRX;
    tLssRxCob.len=8;
    memset(&(tLssRxCob.flags),0,sizeof(tLssRxCob.flags));//tLssRxCob.flags=CANDRV_F_DEFAULT;

    assert(candrv_addrxcob(tWrkParam.bCanController, &tLssRxCob, NULL));
#endif

#ifdef CFG_DS301_ERRPROT
    bErrCtrlRun=NMT_ERRPROT_BOOTUP;

    if(!Os_TaskCreate(&loop_errorprotocol))
    {
        *errorcode=DS301_INITERR_OTHERS;
        return FALSE;
    }
#endif

        // add cobs
    assert(setup_cob_ids());

        // enable can
    candrv_run(tWrkParam.bCanController);

    return TRUE;
}

//***************************************************************************
// Setup node-id dependant and/or configurable COB-IDs

static BOOL setup_cob_ids(void)
{
#ifdef CFG_DS301_ERRPROT
    tErrCtrlTxCob.id=(UNSIGNED16)(DS301_COBID_NMT_ERRCTRL+tWrkParam.bLssNodeId);
    tErrCtrlTxCob.len=1;
    tErrCtrlTxCob.inhibit=0;
    memset(&tErrCtrlTxCob.flags,0,sizeof(tErrCtrlTxCob.flags));//tErrCtrlTxCob.flags=CANDRV_F_DEFAULT;

    tErrCtrlRxCob.id=(UNSIGNED16)(DS301_COBID_NMT_ERRCTRL+tWrkParam.bLssNodeId);
    memset(&tErrCtrlTxCob.flags,0,sizeof(tErrCtrlTxCob.flags));
    tErrCtrlTxCob.flags.bRTR=TRUE;   //tErrCtrlRxCob.flags=CANDRV_F_RTR;
    if(!candrv_addrxcob(tWrkParam.bCanController, &tErrCtrlRxCob, NULL))
        return FALSE;
#endif

#ifdef CFG_DS301_DEFNODEID
    tSdoRxCob.id=DS301_GETCANIDS(DS301_CREATECOBIDENTRY(0,DS301_COBID_SDO_RX+tWrkParam.bLssNodeId));
    tSdoRxCob.len=8;
    memset(&(tSdoRxCob.flags),0,sizeof(tSdoRxCob.flags));//tSdoRxCob.flags=CANDRV_F_DEFAULT;

    if(!candrv_addrxcob(tWrkParam.bCanController, &tSdoRxCob, NULL))
        return FALSE;
#endif

#ifdef CFG_DS301_EMCY
    tEmcyObjCob.len=8;
    memset(&(tEmcyObjCob.flags),0,sizeof(tEmcyObjCob.flags));
    tEmcyObjCob.flags.bTxOK = TRUE;//tEmcyObjCob.flags=CANDRV_F_DEFAULT|CANDRV_F_TXOK;
    tEmcyObjCob.inhibit=tDs301Param.wEmcyInhibitTime;
#endif

    return TRUE;
}

//***************************************************************************
// Setup default COB-ID

static void set_default_cobid(UNSIGNED8 prNodeId)
{
    DS301_COBIDENTRY cobid;

#ifdef CFG_DS301_DEFNODEID
    if(tWrkParam.bLssNodeId==DS301_LSS_UNCONFIGUREDNODEID)
        return;
#endif

#ifdef CFG_DS301_EMCY
    if((prNodeId==DS301_LSS_UNCONFIGUREDNODEID) || (DS301_GETCANID(tWrkParam.tEmcyCob)==DS301_COBID_EMERGENCY+prNodeId))
    {
        cobid=DS301_CREATECOBIDENTRY(0,DS301_COBID_EMERGENCY+tWrkParam.bLssNodeId);
        DS301_MON_MOVE_32(tDs301Param.tEmcyCob, cobid);
    }
#endif

#ifdef CFG_DS301_PDO
    (*tDs301CallBacks.pfSetDefaultPdoCobID)(prNodeId, tWrkParam.bLssNodeId);
#endif
}

//***************************************************************************
// Reset communication states

static void reset_comm(void)
{
    Os_BeginCriticalSection(OS_CRITSECT_TASKSWITCH);

    candrv_stop(tWrkParam.bCanController);

#ifdef CFG_DS301_NMT
    wNmtStatusReg=NMT_PREOPERATIONAL;
#endif

#ifdef CFG_DS301_ERRPROT
    bErrCtrlRun=NMT_ERRPROT_BOOTUP;
#endif

    DS301_MON_SETBIT_16(wLocalCommands,CMD_COMMFAULT_RESET|CMD_EMCY_FORCE);

    candrv_run(tWrkParam.bCanController);

    Os_EndCriticalSection(OS_CRITSECT_TASKSWITCH);
}

//***************************************************************************
// Check that selected cobid is not in the range of SDOs or NMT

BOOL ds301_check_cob_id(DS301_COBIDENTRY cobid)
{
    if(DS301_GETCANID(cobid)>=DS301_COBID_SDO_TX&&DS301_GETCANID(cobid)<=(DS301_COBID_SDO_TX+DS301_LSS_NODEID_END))
        return FALSE;

    if(DS301_GETCANID(cobid)>=DS301_COBID_SDO_RX&&DS301_GETCANID(cobid)<=(DS301_COBID_SDO_RX+DS301_LSS_NODEID_END))
        return FALSE;

    if(DS301_GETCANID(cobid)==DS301_COBID_NMT)
        return FALSE;

    return TRUE;
}

//***************************************************************************
// Check for valid node id

BOOL ds301_check_node_id(UNSIGNED8 nodeid)
{
    if(!(nodeid>=DS301_LSS_NODEID_BEGIN && nodeid<=DS301_LSS_NODEID_END
#ifdef CFG_DS301_LSS
        || nodeid==DS301_LSS_UNCONFIGUREDNODEID
#endif
        ))
        return FALSE;

    return TRUE;
}

//***************************************************************************
// Validate generic COB-ID

static BOOL validate_generic_cobid(DS301_COBIDENTRY * cobid)
{
    if((*cobid&(3l<<29))||*cobid==0l)
        return FALSE;

    if(!ds301_check_cob_id(*cobid))
        return FALSE;

    *cobid&=~DS301_COBIDRESET29BIT;

    return TRUE;
}

//***************************************************************************
// Can driver status check

static void candrvstatus_mgr(void)
{
    UNSIGNED16 wStatus;

    wStatus=uwCanDrvGlobalFlags[tWrkParam.bCanController];

    if(wStatus&CANDRV_GF_ERRPASV)
        DS301_MON_SETBIT_16(wLocalErrors,DS301_COMMERR_CAN_PASSIVE);

    if(wStatus&CANDRV_GF_HWOVERRUN)
        DS301_MON_SETBIT_16(wLocalErrors,DS301_COMMERR_CAN_HWOVERRUN);

    if(wStatus&CANDRV_GF_BUSOFF)
        DS301_MON_SETBIT_16(wLocalErrors,DS301_COMMERR_CAN_BUSOFF);

    if((wStatus&CANDRV_GF_BUSOFF)==0 && (wLocalErrors&DS301_COMMERR_CAN_BUSOFF))
        DS301_MON_SETBIT_16(wLocalErrors,DS301_COMMERR_CAN_EXITFROMBUSOFF);
}

//***************************************************************************
// Communication faults manager

static void commfault_mgr(void)
{
    static UNSIGNED16 wPrevCommErrReg;
    UNSIGNED16 cnt,actreg,mask;

        // if local comm fault reset requested
    if(wLocalCommands&CMD_COMMFAULT_RESET)
    {
            // clear all comm errors
        wPrevCommErrReg=0;
        DS301_MON_MOVE_16(wLocalErrors,wPrevCommErrReg);

            // signal that comm errors are cleared
        (*tDs301CallBacks.pfCommFault)(0);
        DS301_MON_CLEARBIT_16(wLocalCommands,CMD_COMMFAULT_RESET);

        return;
    }

        // get actual comm error reg
    DS301_MON_MOVE_16(actreg, wLocalErrors);

        // if changed
    if(actreg!=wPrevCommErrReg)
    {
            // find differences and signal each one
        for(cnt=0,mask=1;cnt<16;cnt++,mask<<=1)
            if((wPrevCommErrReg&mask)==0 && (actreg&mask)!=0)
                (*tDs301CallBacks.pfCommFault)(mask);

        wPrevCommErrReg=actreg;
    }
}

//***************************************************************************
// error protocol manager

#ifdef CFG_DS301_ERRPROT
static void loop_errorprotocol(void)
{
    UNSIGNED16 wLocTout=0;
    UNSIGNED8 bFactor=0;
    UNSIGNED8 nmtstatval;
    UNSIGNED8 toggle=0;
    BOOLEAN guardtimefirst;

    for(;;)
    {
        if(!(wLocalCommands&CMD_FULLYOPERATIVE))
        {
                // do nothing until system is not fully operative
            while(!(wLocalCommands&CMD_FULLYOPERATIVE))
            {
                (*tDs301CallBacks.pfWdtClear)();
                Os_Sleep(0);
            }

                // ignore cobs received when waiting
            //DS301_MON_CLEARBIT_16(tErrCtrlRxCob.flags, CANDRV_F_NEWCOB|CANDRV_F_OVERRUN);
            tErrCtrlRxCob.flags.bNewRxCob = FALSE;
            tErrCtrlRxCob.flags.bOverRun = FALSE;
                // and restart from bootup
            bErrCtrlRun=NMT_ERRPROT_BOOTUP;
        }

#ifdef CFG_DS301_LSS
            // if LSS active do nothing
        if(wLocalCommands&CMD_LSSSWITCHEDON)
        {
            //DS301_MON_CLEARBIT_16(tErrCtrlRxCob.flags, CANDRV_F_NEWCOB|CANDRV_F_OVERRUN);
            tErrCtrlRxCob.flags.bNewRxCob = FALSE;
            tErrCtrlRxCob.flags.bOverRun = FALSE;
            continue;
        }
#endif
            // if detect parameter change or if stopped always and is not booting
        if(bErrCtrlRun!=NMT_ERRPROT_BOOTUP && \
            (tWrkParam.wGuardTime!=tDs301Param.wGuardTime || \
            tWrkParam.wHeartbeatTime!=tDs301Param.wHeartbeatTime || \
            tWrkParam.bLifeTimeFactor!=tDs301Param.bLifeTimeFactor || \
            bErrCtrlRun==NMT_ERRPROT_STOP))
        {
            Os_BeginCriticalSection(OS_CRITSECT_GLOBAL);

                // check valid values
            if(tDs301Param.wHeartbeatTime>0 && tDs301Param.wHeartbeatTime<DS301_ERRPROT_MINTIME || \
                tDs301Param.wHeartbeatTime>DS301_ERRPROT_MAXTIME || \
                tDs301Param.wGuardTime>0 && tDs301Param.wGuardTime<DS301_ERRPROT_MINTIME || \
                tDs301Param.wGuardTime>DS301_ERRPROT_MAXTIME)
            {
                    // if not valid stop protocol and signal EMCY
                bErrCtrlRun=NMT_ERRPROT_STOP;
                DS301_MON_SETBIT_16(wLocalErrors,DS301_COMMERR_GUARDHEARTBEAT_WRONGPARAMS);
            }
                // if valid
            else
            {
                UNSIGNED8 bPrevSetup, bNewSetup;

                    // get actual setup
                if(tWrkParam.wHeartbeatTime>0)
                    bPrevSetup=1;
                else if(tWrkParam.wGuardTime)
                    bPrevSetup=2;
                else
                    bPrevSetup=0;

                    // get new setup
                if(tDs301Param.wHeartbeatTime>0)
                    bNewSetup=1;
                else if(tDs301Param.wGuardTime)
                    bNewSetup=2;
                else
                    bNewSetup=0;

                    // copy new parameters
                tWrkParam.wGuardTime        =tDs301Param.wGuardTime;
                tWrkParam.wHeartbeatTime    =tDs301Param.wHeartbeatTime;
                tWrkParam.bLifeTimeFactor   =tDs301Param.bLifeTimeFactor;

                    // and trigger restart only if setup is different
                    // or if it was stopped
                if(bPrevSetup!=bNewSetup || bErrCtrlRun==NMT_ERRPROT_STOP)
                    bErrCtrlRun=NMT_ERRPROT_START;
            }

            Os_EndCriticalSection(OS_CRITSECT_GLOBAL);
        }

            // protocol processing
        nmtstatval=0;
        switch(bErrCtrlRun)
        {
            case NMT_ERRPROT_BOOTUP:
            	memset(&tErrCtrlTxCob.flags,0,sizeof(tErrCtrlTxCob.flags));//tErrCtrlTxCob.flags=CANDRV_F_DEFAULT;
                candrv_put_unsigned8(&tErrCtrlTxCob,0,DS301_NMT_STATUS_BOOTUP);

                candrv_sendcob(tWrkParam.bCanController, (CANDRV_PCOB)&tErrCtrlTxCob);

                while(!tErrCtrlTxCob.flags.bTxOK)//while(!(tErrCtrlTxCob.flags&CANDRV_F_TXOK))
                {
                    (*tDs301CallBacks.pfWdtClear)();
                    Os_Sleep(0);
                }

                bErrCtrlRun=NMT_ERRPROT_START;
                break;

            case NMT_ERRPROT_START:
                if(tWrkParam.wHeartbeatTime>0)
                {
                    wLocTout=timer_settimeout(uwSysTimers1ms,tWrkParam.wHeartbeatTime);
                    toggle=0x80;

                    bErrCtrlRun=NMT_ERRPROT_RUN;
                }
                else if(tWrkParam.wGuardTime>0)
                {
                    wLocTout=timer_settimeout(uwSysTimers1ms,tWrkParam.wGuardTime);
                    bFactor=(UNSIGNED8)(tWrkParam.bLifeTimeFactor?tWrkParam.bLifeTimeFactor:1);
                    toggle=0x80;
                    guardtimefirst=TRUE;

                    bErrCtrlRun=NMT_ERRPROT_RUN;
                }
                else
                    bErrCtrlRun=NMT_ERRPROT_STOP;

                toggle^=0x80;

                break;

            case NMT_ERRPROT_RUN:
                if(timer_istimedout(uwSysTimers1ms,wLocTout))
                {
                    if(tWrkParam.wHeartbeatTime>0)
                    {
                        nmtstatval=1;
                        wLocTout=timer_settimeout(uwSysTimers1ms,tWrkParam.wHeartbeatTime);
                    }
                    else if(--bFactor==0)
                    {
                        DS301_MON_SETBIT_16(wLocalErrors,DS301_COMMERR_GUARDHEARTBEAT);
                        wLocTout=timer_settimeout(uwSysTimers1ms,tWrkParam.wGuardTime);
                        bFactor++;
                    }
                    else
                        wLocTout=timer_settimeout(uwSysTimers1ms,tWrkParam.wGuardTime);
                }

                    // for guardtime do nothing until first reset COB is received by the master
                    // requested from many customers with slow startup canopen master
                if(tWrkParam.wHeartbeatTime==0 && tWrkParam.wGuardTime>0 && (tErrCtrlRxCob.flags.bNewRxCob || guardtimefirst))//if(tWrkParam.wHeartbeatTime==0 && tWrkParam.wGuardTime>0 && ((tErrCtrlRxCob.flags&CANDRV_F_NEWCOB) || guardtimefirst))
                {
                    if(!guardtimefirst)
                        toggle^=0x80;

                    if(tErrCtrlRxCob.flags.bNewRxCob)//if(tErrCtrlRxCob.flags&CANDRV_F_NEWCOB)
                        guardtimefirst=FALSE;

                    wLocTout=timer_settimeout(uwSysTimers1ms,tWrkParam.wGuardTime);
                    bFactor=(UNSIGNED8)(tWrkParam.bLifeTimeFactor?tWrkParam.bLifeTimeFactor:1);
                }

                break;

            case NMT_ERRPROT_STOP:
                if(tErrCtrlRxCob.flags.bNewRxCob)//if(tErrCtrlRxCob.flags&CANDRV_F_NEWCOB)
                    toggle^=0x80;
                break;
        }

        if((nmtstatval || tErrCtrlRxCob.flags.bNewRxCob) && (tErrCtrlTxCob.flags.bTxOK))//if((nmtstatval || tErrCtrlRxCob.flags&CANDRV_F_NEWCOB) && (tErrCtrlTxCob.flags&CANDRV_F_TXOK))
        {
            nmtstatval=ds301_get_nmt_state();

            memset(&(tErrCtrlTxCob.flags),0,sizeof(tErrCtrlTxCob.flags));//tErrCtrlTxCob.flags=CANDRV_F_DEFAULT;
            tErrCtrlTxCob.len=1;
            candrv_put_unsigned8(&tErrCtrlTxCob,0,nmtstatval|toggle);
            candrv_sendcob(tWrkParam.bCanController, (CANDRV_PCOB)&tErrCtrlTxCob);

            while(!(tErrCtrlTxCob.flags.bTxOK))//while(!(tErrCtrlTxCob.flags&CANDRV_F_TXOK))
            {
                (*tDs301CallBacks.pfWdtClear)();
                Os_Sleep(0);
            }

            tErrCtrlRxCob.flags.bNewRxCob =FALSE;//DS301_MON_CLEARBIT_16(tErrCtrlRxCob.flags, CANDRV_F_NEWCOB);
        }

        (*tDs301CallBacks.pfWdtClear)();
        Os_Sleep(0);
    }
}
#endif

//***************************************************************************
// Get NMT FSM state

#ifdef CFG_DS301_NMT
UNSIGNED8 ds301_get_nmt_state(void)
{
    UNSIGNED16 nmtstatval;

    if(bErrCtrlRun==NMT_ERRPROT_BOOTUP)
        return DS301_NMT_STATUS_BOOTUP;

    DS301_MON_MOVE_16(nmtstatval, wNmtStatusReg);

    if(nmtstatval&NMT_PREOPERATIONAL)
        return DS301_NMT_STATUS_PREOPERATIONAL;
    else if(nmtstatval&NMT_OPERATIONAL)
        return DS301_NMT_STATUS_OPERATIONAL;
    else
        return DS301_NMT_STATUS_STOPPED;
}

//***************************************************************************
// Set NMT FSM state

void ds301_set_nmt_state(UNSIGNED8 bNewStat)
{
    switch(bNewStat)
    {
        case DS301_NMT_ENTER_PREOPERATIONAL:
        case DS301_NMT_START:
        case DS301_NMT_STOP:
        case DS301_NMT_RESET_NODE:
        case DS301_NMT_RESET_COMMUNICATION:
            break;

        default:
            return;
    }

        // set new command
    bNmtRxCmd=bNewStat;

        // and signal to slow task
    DS301_MON_SETBIT_16(wLocalCommands,CMD_NEW_NMTCOB);
}

//***************************************************************************
// NMT RX COB event

static void nmt_cob_event(CANDRV_COB * pCob)
{
    UNSIGNED8 nodo;

        // destination node
    nodo=candrv_get_unsigned8(pCob,8);

        // if not broadcast neither node or by status then do nothing
    if(nodo!=0 && nodo!=tWrkParam.bLssNodeId
#ifdef CFG_DS301_LSS
        || (wLocalCommands&CMD_LSSSWITCHEDON)
#endif
#ifdef CFG_DS301_ERRPROT
        || (bErrCtrlRun==NMT_ERRPROT_BOOTUP)
#endif
        )
    {
        //DS301_MON_CLEARBIT_16(pCob->flags,(CANDRV_F_NEWCOB|CANDRV_F_OVERRUN));
    	pCob->flags.bNewRxCob = FALSE;
    	pCob->flags.bOverRun = FALSE;
        return;
    }

        // check for overrun
    if((pCob->flags.bOverRun) || (wLocalCommands&CMD_NEW_NMTCOB))//if((pCob->flags&CANDRV_F_OVERRUN) || (wLocalCommands&CMD_NEW_NMTCOB))
        DS301_MON_SETBIT_16(wLocalErrors,DS301_COMMERR_CANOVERRUN);

    pCob->flags.bNewRxCob= FALSE;
    pCob->flags.bOverRun = FALSE;//DS301_MON_CLEARBIT_16(pCob->flags,CANDRV_F_NEWCOB|CANDRV_F_OVERRUN);

        // get command
    bNmtRxCmd=candrv_get_unsigned8(pCob,0); // comando

        // and signal to slow task
    DS301_MON_SETBIT_16(wLocalCommands,CMD_NEW_NMTCOB);
}

//***************************************************************************
// NMT manager

static void nmt_mgr(void)
{
    UNSIGNED8 curstate=0;
    UNSIGNED8 cmd;

        // check for waiting commands
    if(!(wLocalCommands&CMD_NEW_NMTCOB))
        return;

        // get command and clear flag
    cmd=bNmtRxCmd;
    DS301_MON_CLEARBIT_16(wLocalCommands,CMD_NEW_NMTCOB);

        // if LSS active do nothing
    if(wLocalCommands&CMD_LSSSWITCHEDON)
        return;

        // check current state
    if(wNmtStatusReg&NMT_PREOPERATIONAL)
        curstate=DS301_NMT_ENTER_PREOPERATIONAL;
    else if(wNmtStatusReg&NMT_OPERATIONAL)
        curstate=DS301_NMT_START;
    else if(wNmtStatusReg&NMT_STOPPED)
        curstate=DS301_NMT_STOP;

        // if same do nothing
    if(curstate==cmd)
        return;

    switch(cmd)
    {
        case DS301_NMT_START:
            wNmtStatusReg&=~(NMT_PREOPERATIONAL|NMT_STOPPED);
            wNmtStatusReg|=NMT_OPERATIONAL;
            (*tDs301CallBacks.pfNmtEvent)(DS301_NMT_START);
#ifdef CFG_DS301_PDO
            (*tDs301CallBacks.pfPdoCreate)(tWrkParam.bCanController);
#endif
            break;

        case DS301_NMT_STOP:
#ifdef CFG_DS301_PDO
            (*tDs301CallBacks.pfPdoDestroy)(tWrkParam.bCanController);
#endif
            wNmtStatusReg&=~(NMT_PREOPERATIONAL|NMT_OPERATIONAL);
            wNmtStatusReg|=NMT_STOPPED;
            (*tDs301CallBacks.pfNmtEvent)(DS301_NMT_STOP);
            break;

        case DS301_NMT_ENTER_PREOPERATIONAL:
#ifdef CFG_DS301_PDO
            (*tDs301CallBacks.pfPdoDestroy)(tWrkParam.bCanController);
#endif
            wNmtStatusReg&=~(NMT_OPERATIONAL|NMT_STOPPED);
            wNmtStatusReg|=NMT_PREOPERATIONAL;
            (*tDs301CallBacks.pfNmtEvent)(DS301_NMT_ENTER_PREOPERATIONAL);
            break;

        case DS301_NMT_RESET_NODE:
        case DS301_NMT_RESET_COMMUNICATION:
#ifdef CFG_DS301_PDO
            (*tDs301CallBacks.pfPdoDestroy)(tWrkParam.bCanController);
#endif
            wNmtStatusReg=0;
            (*tDs301CallBacks.pfNmtEvent)(cmd);
            reset_comm();
            break;
    }
}
#endif

//***************************************************************************
// SDO manager
// it has responsibility just for wrapping and unwrapping data from
// packets, data processing is done externally as dependent from object
// dictionary specific implementation

static void sdo_mgr(CANDRV_PCOB sdotx)
{
    DS301_SDOTRANSACTION transtate;
    OCTET_STRING locbuf[7];
    UNSIGNED8 cmd;
    UNSIGNED8 cursize=0,curstart=0;
    UNSIGNED16 ct;
    UNSIGNED32 abcode,size;
    struct __stato
    {
        unsigned toggle:1;
        unsigned segment:1;
        unsigned first:1;
        unsigned last:1;
        unsigned dispatchinit:1;
        unsigned expedited:1;
        unsigned zerosize:1;
    } stato;

        // look at various state in order to take decision to process or not
    if(FALSE
#ifdef CFG_DS301_LSS
        || (wLocalCommands&CMD_LSSSWITCHEDON)
#endif
#ifdef CFG_DS301_NMT
        || (wNmtStatusReg&NMT_STOPPED)
#endif
#ifdef CFG_DS301_ERRPROT
        || (bErrCtrlRun==NMT_ERRPROT_BOOTUP)
#endif
        )
    {
        //DS301_MON_CLEARBIT_16(tSdoRxCob.flags, CANDRV_F_NEWCOB|CANDRV_F_OVERRUN);
    	tSdoRxCob.flags.bNewRxCob = FALSE;
    	tSdoRxCob.flags.bOverRun = FALSE;
        return;
    }

        // if nothing received then return
    if((tSdoRxCob.flags.bNewRxCob)==0)//if((tSdoRxCob.flags&CANDRV_F_NEWCOB)==0)
        return;

        // initialize TX COB
    sdotx->id=DS301_GETCANIDS(DS301_CREATECOBIDENTRY(0,DS301_COBID_SDO_TX+tWrkParam.bLssNodeId));
    sdotx->len=8;
    sdotx->inhibit=0;
    memset(&(sdotx->flags),0,sizeof(sdotx->flags));
    sdotx->flags.bTxBlock =TRUE;//sdotx->flags=CANDRV_F_DEFAULT|CANDRV_F_TXBLOCK;

        // reset trasaction state data
    memset(&transtate, 0, sizeof(transtate));
    transtate.pubDataBuffer=locbuf;

        // decode SDO transaction type, index and subindex
    cmd=(UNSIGNED8)candrv_get_bitfield((CANDRV_PCOB)&tSdoRxCob,5,3);
    transtate.uwIndex=candrv_get_unsigned16((CANDRV_PCOB)&tSdoRxCob,8);
    transtate.ubSubIndex=candrv_get_unsigned8((CANDRV_PCOB)&tSdoRxCob,24);

    stato.dispatchinit=0;

        // strictly check for RX COB length
    if(tSdoRxCob.len!=8)
    {
        abcode=DS301_SDOABORT_GENERALDEVINCOMPATIBILITY;
        goto abort;
    }

    switch(cmd)
    {
        case SDO_INITDOWNLOAD_REQ:
            curstart=cursize=0;

                // decode download type command and parameters
            if(candrv_get_bitfield((CANDRV_PCOB)&tSdoRxCob,1,1))
            {       // expedited
                stato.expedited=1;
                curstart=32;

                if(candrv_get_bitfield((CANDRV_PCOB)&tSdoRxCob,0,1))
                {
                    cursize=(UNSIGNED8)(4-candrv_get_bitfield((CANDRV_PCOB)&tSdoRxCob,2,2));
                    size=(UNSIGNED32)cursize;
                }
                else
                {
                    cursize=4;
                    size=0l;
                }
            }
            else
            {       // normal
                stato.expedited=0;

                if(candrv_get_bitfield((CANDRV_PCOB)&tSdoRxCob,0,1))
                    size=candrv_get_unsigned32((CANDRV_PCOB)&tSdoRxCob,32);
                else
                    size=0l;
            }

            transtate.f.b.bDownload=1;
            transtate.f.b.bInit=1;
            transtate.ulDataSize=size;

                // initialize transaction
            abcode=(*tDs301CallBacks.pfSdoTransactions)(&transtate);
            if(abcode!=DS301_SDOOK)
                goto abort;

            stato.toggle=stato.segment=stato.last=0;
            stato.dispatchinit=1;
            transtate.f.b.bInit=0;
            for(;;)
            {
                if(cursize)
                {
                    if(size>0l)
                    {
                        if(cursize>size)
                        {
                            abcode=DS301_SDOABORT_PARAMLENGHTTOOHIGH;
                            goto abort;
                        }

                        size-=cursize;
                    }

                    for(ct=0;ct<cursize;ct++,curstart+=8)
                        locbuf[ct]=candrv_get_unsigned8((CANDRV_PCOB)&tSdoRxCob,curstart);

                    if(stato.last||stato.expedited)
                        transtate.f.b.bLast=1;
                    else
                        transtate.f.b.bLast=0;

                        // send data chunk
                    transtate.f.b.bData=1;
                    transtate.ulDataSize=cursize;
                    abcode=(*tDs301CallBacks.pfSdoTransactions)(&transtate);
                    if(abcode!=DS301_SDOOK)
                    {
                        stato.dispatchinit=0;
                        goto abort;
                    }
                }

                candrv_put_integer32(sdotx,0,0l);
                candrv_put_integer32(sdotx,32,0l);

                if(stato.segment)
                {
                    candrv_put_bitfield(sdotx,5,3,SDO_SEGDOWNLOAD_RESP);
                    candrv_put_bitfield(sdotx,4,1,stato.toggle);
                    stato.toggle=!stato.toggle;
                }
                else
                {
                    candrv_put_bitfield(sdotx,5,3,SDO_INITDOWNLOAD_RESP);
                    candrv_put_unsigned16(sdotx,8,transtate.uwIndex);
                    candrv_put_unsigned8(sdotx,24,transtate.ubSubIndex);
                    stato.segment=1;
                }

                if(tSdoRxCob.flags.bOverRun)//if(tSdoRxCob.flags&CANDRV_F_OVERRUN)
                    DS301_MON_SETBIT_16(wLocalErrors,DS301_COMMERR_CANOVERRUN);

                tSdoRxCob.flags.bNewRxCob = FALSE;
                tSdoRxCob.flags.bOverRun = FALSE;//DS301_MON_CLEARBIT_16(tSdoRxCob.flags,CANDRV_F_NEWCOB|CANDRV_F_OVERRUN);

                candrv_sendcob(tWrkParam.bCanController, sdotx);

                if(stato.last||stato.expedited)
                    break;

                ct=timer_settimeout(uwCanDrvTimer100us,SDO_TIMEOUT);
                while((tSdoRxCob.flags.bNewRxCob)==0&&!timer_istimedout(uwCanDrvTimer100us,ct))//while((tSdoRxCob.flags&CANDRV_F_NEWCOB)==0&&!timer_istimedout(uwCanDrvTimer100us,ct))
                    (*tDs301CallBacks.pfWdtClear)();

                if(tSdoRxCob.flags.bNewRxCob)//if(tSdoRxCob.flags&CANDRV_F_NEWCOB)
                {
                    if(tSdoRxCob.len!=8)
                    {
                        abcode=DS301_SDOABORT_GENERALDEVINCOMPATIBILITY;
                        goto abort;
                    }

                    ct=(UNSIGNED8)candrv_get_bitfield((CANDRV_PCOB)&tSdoRxCob,5,3);
                    if(ct==SDO_ABORT)
                    {
                        abcode=0l;
                        goto abort;
                    }
                    if(ct!=SDO_SEGDOWNLOAD_REQ)
                    {
                        abcode=DS301_SDOABORT_INVALIDCOMMAND;
                        goto abort;
                    }
                    if(candrv_get_bitfield((CANDRV_PCOB)&tSdoRxCob,4,1)!=stato.toggle)
                    {
                        abcode=DS301_SDOABORT_TOGGLEBIT;
                        goto abort;
                    }

                    curstart=8;
                    cursize=(UNSIGNED8)(7-candrv_get_bitfield((CANDRV_PCOB)&tSdoRxCob,1,3));

                    if(candrv_get_bitfield((CANDRV_PCOB)&tSdoRxCob,0,1))
                        stato.last=1;
                }
                else
                {
                    abcode=DS301_SDOABORT_TIMEOUT;
                    goto abort;
                }
            }

                // end transaction
            transtate.f.b.bData=0;
            transtate.f.b.bLast=0;
            transtate.f.b.bEnd=1;
            transtate.ulDataSize=0;
            (*tDs301CallBacks.pfSdoTransactions)(&transtate);
            break;

        case SDO_INITUPLOAD_REQ:
            transtate.f.b.bUpload=1;
            transtate.f.b.bInit=1;

                // initialize transaction
            abcode=(*tDs301CallBacks.pfSdoTransactions)(&transtate);
            if(abcode!=DS301_SDOOK)
                goto abort;

            size=transtate.ulDataSize;

            stato.toggle=stato.segment=stato.last=0;
            stato.dispatchinit=1;
            transtate.f.b.bInit=0;

            if(size)
                stato.zerosize=0;
            else
            {
                stato.zerosize=1;
                size=1l;
            }

            stato.toggle=stato.segment=stato.expedited=stato.last=0;

            for(;;)
            {
                if(!stato.segment)
                {
                    candrv_put_integer8(sdotx,0,0);

                    candrv_put_bitfield(sdotx,5,3,SDO_INITUPLOAD_RESP);
                    candrv_put_unsigned16(sdotx,8,transtate.uwIndex);
                    candrv_put_unsigned8(sdotx,24,transtate.ubSubIndex);

                    if(size<=4l)
                    {
                        candrv_put_bitfield(sdotx,0,2,0x03);
                        candrv_put_bitfield(sdotx,2,2,(UNSIGNED16)(4-size));

                        curstart=32;
                        cursize=(UNSIGNED8)size;

                        stato.expedited=1;
                    }
                    else
                    {
                        candrv_put_bitfield(sdotx,0,2,0x01);
                        candrv_put_unsigned32(sdotx,32,size);

                        stato.segment=1;
                    }

                    stato.first=1;
                }
                else
                {
                    candrv_put_integer8(sdotx,0,0);

                    candrv_put_bitfield(sdotx,5,3,SDO_SEGUPLOAD_RESP);
                    candrv_put_bitfield(sdotx,4,1,stato.toggle);
                    stato.toggle=!stato.toggle;

                    curstart=8;
                    if(size>7)
                    {
                        candrv_put_bitfield(sdotx,0,1,0);
                        size-=7;
                        cursize=7;
                    }
                    else
                    {
                        candrv_put_bitfield(sdotx,0,1,1);
                        cursize=(UNSIGNED8)size;
                        stato.last=1;
                    }
                    candrv_put_bitfield(sdotx,1,3,(UNSIGNED16)(7-cursize));

                    stato.first=0;
                }

                if(stato.expedited||!stato.first)
                {
                    for(ct=curstart;ct<64;ct+=8)
                        candrv_put_integer8(sdotx,ct,0);

                    if(!stato.zerosize)
                    {
                            // get data chunk
                        transtate.f.b.bData=1;
                        transtate.ulDataSize=cursize;
                        abcode=(*tDs301CallBacks.pfSdoTransactions)(&transtate);
                        if(abcode!=DS301_SDOOK)
                        {
                            stato.dispatchinit=0;
                            goto abort;
                        }

                        for(ct=0;ct<cursize;ct++,curstart+=8)
                            candrv_put_unsigned8(sdotx,curstart,locbuf[ct]);
                    }
                }

                if(tSdoRxCob.flags.bOverRun)//if(tSdoRxCob.flags&CANDRV_F_OVERRUN)
                    DS301_MON_SETBIT_16(wLocalErrors,DS301_COMMERR_CANOVERRUN);

                tSdoRxCob.flags.bNewRxCob = FALSE;
                tSdoRxCob.flags.bOverRun = FALSE;//DS301_MON_CLEARBIT_16(tSdoRxCob.flags,CANDRV_F_NEWCOB|CANDRV_F_OVERRUN);

                candrv_sendcob(tWrkParam.bCanController, sdotx);

                if(stato.last||stato.expedited)
                    break;

                ct=timer_settimeout(uwCanDrvTimer100us,SDO_TIMEOUT);
                while((tSdoRxCob.flags.bNewRxCob)==0&&!timer_istimedout(uwCanDrvTimer100us,ct))//while((tSdoRxCob.flags&CANDRV_F_NEWCOB)==0&&!timer_istimedout(uwCanDrvTimer100us,ct))
                    (*tDs301CallBacks.pfWdtClear)();

                if(tSdoRxCob.flags.bNewRxCob)//if(tSdoRxCob.flags&CANDRV_F_NEWCOB)
                {
                    if(tSdoRxCob.len!=8)
                    {
                        abcode=DS301_SDOABORT_GENERALDEVINCOMPATIBILITY;
                        goto abort;
                    }

                    ct=(UNSIGNED8)candrv_get_bitfield((CANDRV_PCOB)&tSdoRxCob,5,3);
                    if(ct==SDO_ABORT)
                    {
                        abcode=0l;
                        goto abort;
                    }
                    if(ct!=SDO_SEGUPLOAD_REQ)
                    {
                        abcode=DS301_SDOABORT_INVALIDCOMMAND;
                        goto abort;
                    }
                    if(candrv_get_bitfield((CANDRV_PCOB)&tSdoRxCob,4,1)!=stato.toggle)
                    {
                        abcode=DS301_SDOABORT_TOGGLEBIT;
                        goto abort;
                    }
                }
                else
                {
                    abcode=DS301_SDOABORT_TIMEOUT;
                    goto abort;
                }
            }

                // end transaction
            transtate.f.b.bData=0;
            transtate.f.b.bLast=0;
            transtate.f.b.bEnd=1;
            transtate.ulDataSize=0;
            (*tDs301CallBacks.pfSdoTransactions)(&transtate);
            break;

        default:
            abcode=DS301_SDOABORT_INVALIDCOMMAND;
            goto abort;
    }

    return;

abort:
    if(stato.dispatchinit)
    {
        transtate.f.b.bData=0;
        transtate.f.b.bLast=0;
        transtate.f.b.bAbort=1;
        transtate.ulDataSize=0;
        (*tDs301CallBacks.pfSdoTransactions)(&transtate);
    }

    if(tSdoRxCob.flags.bOverRun)//if(tSdoRxCob.flags&CANDRV_F_OVERRUN)
        DS301_MON_SETBIT_16(wLocalErrors,DS301_COMMERR_CANOVERRUN);

    tSdoRxCob.flags.bNewRxCob = FALSE;
    tSdoRxCob.flags.bOverRun = FALSE;//DS301_MON_CLEARBIT_16(tSdoRxCob.flags,CANDRV_F_NEWCOB|CANDRV_F_OVERRUN);

    if(abcode)
    {
        candrv_put_bitfield(sdotx,5,3,SDO_ABORT);
        candrv_put_bitfield(sdotx,0,5,0);
        candrv_put_unsigned16(sdotx,8,transtate.uwIndex);
        candrv_put_unsigned8(sdotx,24,transtate.ubSubIndex);
        candrv_put_unsigned32(sdotx,32,abcode);

        candrv_sendcob(tWrkParam.bCanController, sdotx);
    }
}

//***************************************************************************
// EMCY manager

#ifdef CFG_DS301_EMCY
static void emcy_mgr(void)
{
    UNSIGNED32 newalman;
    UNSIGNED16 newalcode;
    BOOL bSendCob=FALSE;
    UNSIGNED8 bClassMask;

        // look at various state in order to take decision to process or not
    if(FALSE
#ifdef CFG_DS301_LSS
        || (wLocalCommands&CMD_LSSSWITCHEDON)
#endif
#ifdef CFG_DS301_NMT
        || (wNmtStatusReg&NMT_STOPPED)
#endif
#ifdef CFG_DS301_ERRPROT
        || (bErrCtrlRun==NMT_ERRPROT_BOOTUP)
#endif
        )
        return;

        // if it's still to be transmitted or inhibited do nothing
    if((!tEmcyObjCob.flags.bTxOK)||(tEmcyObjCob.flags.bInhibited))//if(!(tEmcyObjCob.flags&CANDRV_F_TXOK) || (tEmcyObjCob.flags&CANDRV_F_INHIBITED))
        return;

        // check and get new alarm entry
    if((*tDs301CallBacks.pfGetAlarmEntry)(&newalman, &newalcode, &bClassMask))
    {
            // if both zero then alarm clear was done
        if(newalman==0l && newalcode==0)
        {
            Os_BeginCriticalSection(OS_CRITSECT_GLOBAL);
            DS301_MON_MOVE_32(dDs301ErrRegManufacturer, newalman);
            DS301_MON_MOVE_16(wDs301ErrCodeLast, newalcode);
            bDs301ErrorRegister=0;
            Os_EndCriticalSection(OS_CRITSECT_GLOBAL);

            bSendCob=TRUE;
        }
            // if different than then last ones
        else if((dDs301ErrRegManufacturer&newalman)!=newalman || wDs301ErrCodeLast!=newalcode)
        {
                // add active alarms
            newalman|=dDs301ErrRegManufacturer;

                // update all error registers
            Os_BeginCriticalSection(OS_CRITSECT_GLOBAL);
            DS301_MON_MOVE_32(dDs301ErrRegManufacturer, newalman);
            DS301_MON_MOVE_16(wDs301ErrCodeLast, newalcode);
            bDs301ErrorRegister|=bClassMask|DS301_REGERR_GENERIC;
            Os_EndCriticalSection(OS_CRITSECT_GLOBAL);

            bSendCob=TRUE;
        }
    }

        // check if cobid parameters is changed
    if(tDs301Param.tEmcyCob!=tWrkParam.tEmcyCob)
    {
        DS301_MON_MOVE_32(tWrkParam.tEmcyCob, tDs301Param.tEmcyCob);

            // check for validity of the new one, only if enabled
        if(DS301_COBIDVALID(tWrkParam.tEmcyCob))
            if(!validate_generic_cobid(&tWrkParam.tEmcyCob))
            {
                    // signal error
                DS301_MON_SETBIT_16(wLocalErrors, DS301_COMMERR_EMCY_WRONGCOBID);

                    // and invalidate EMCY
                tWrkParam.tEmcyCob=DS301_CREATEPDOCOBIDENTRY(1,0,0);
            }
    }

        // check for forced EMCY tx, only if some errors
    if((wLocalCommands&CMD_EMCY_FORCE) && bDs301ErrorRegister!=0)
        bSendCob=TRUE;

    DS301_MON_CLEARBIT_16(wLocalCommands, CMD_EMCY_FORCE);

        // if send cob requested
    if(bSendCob)
    {
        candrv_put_unsigned16((CANDRV_PCOB)&tEmcyObjCob,0,wDs301ErrCodeLast);
        candrv_put_unsigned8((CANDRV_PCOB)&tEmcyObjCob,16,bDs301ErrorRegister);
        candrv_put_unsigned32((CANDRV_PCOB)&tEmcyObjCob,24,dDs301ErrRegManufacturer);
        candrv_put_unsigned8((CANDRV_PCOB)&tEmcyObjCob,56,0);

        if(DS301_COBIDVALID(tWrkParam.tEmcyCob))
        {
                // setup here inhibit time as any value is valid
            tEmcyObjCob.inhibit=tDs301Param.wEmcyInhibitTime;
            tEmcyObjCob.id=DS301_GETCANIDS(tWrkParam.tEmcyCob);

            candrv_sendcob(tWrkParam.bCanController, &tEmcyObjCob);
        }
    }
}
#endif

//***************************************************************************
// LSS manager

#ifdef CFG_DS301_LSS
static void lss_mgr(CANDRV_PCOB txcob)
{
    static BOOLEAN bLssActive=FALSE;
    static BOOLEAN bNodeIdModified=FALSE;
    static UNSIGNED16 wlssstatus=0;
    static UNSIGNED8 prNodeId;
    UNSIGNED8 bloc;
    UNSIGNED16 wloc;
    UNSIGNED32 dloc;

    if(!(wLocalCommands&CMD_LSSSWITCHEDON) && (FALSE
#ifdef CFG_DS301_NMT
        || (wNmtStatusReg&NMT_OPERATIONAL)
#endif
#ifdef CFG_DS301_ERRPROT
        || (bErrCtrlRun==NMT_ERRPROT_BOOTUP)
#endif
        ))
    {
        //DS301_MON_CLEARBIT_16(tLssRxCob.flags, CANDRV_F_NEWCOB|CANDRV_F_OVERRUN);
    	tLssRxCob.flags.bNewRxCob = FALSE;
    	tLssRxCob.flags.bOverRun = FALSE;
        return;
    }

        // if nothing received then return
    if((tLssRxCob.flags.bNewRxCob)==FALSE)//if((tLssRxCob.flags&CANDRV_F_NEWCOB)==0)
        return;

        // check for overrun
    if(tLssRxCob.flags.bOverRun)//if(tLssRxCob.flags&CANDRV_F_OVERRUN)
        DS301_MON_SETBIT_16(wLocalErrors,DS301_COMMERR_CANOVERRUN);

    tLssRxCob.flags.bNewRxCob=FALSE;
    tLssRxCob.flags.bOverRun =FALSE;//DS301_MON_CLEARBIT_16(tLssRxCob.flags,CANDRV_F_NEWCOB|CANDRV_F_OVERRUN);

    txcob->id=DS301_COBID_LSSTX;
    txcob->len=8;
    txcob->inhibit=0;
    memset(&(txcob->flags),0,sizeof(txcob->flags));
    txcob->flags.bTxBlock = TRUE;//txcob->flags=CANDRV_F_DEFAULT|CANDRV_F_TXBLOCK;
    candrv_put_unsigned32(txcob,0,0l);
    candrv_put_unsigned32(txcob,32,0l);

    if(tWrkParam.bLssNodeId==DS301_LSS_UNCONFIGUREDNODEID)
        bLssActive=TRUE;

    if(tLssRxCob.len==8)
    {
        switch(candrv_get_unsigned8((CANDRV_PCOB)&tLssRxCob,0))
        {
            case LSS_SWITCHGLOBAL:
                bloc=candrv_get_unsigned8((CANDRV_PCOB)&tLssRxCob,8);
                if(bloc==0)
                {
                    if(bLssActive && bNodeIdModified)
                    {
                        setup_cob_ids();
                        reset_comm();
                    }

                    bLssActive=FALSE;
                    bNodeIdModified=FALSE;
                }
                if(bloc==1&&!bLssActive)
                {
                    bLssActive=TRUE;
                    prNodeId=tWrkParam.bLssNodeId;
                }
                break;

#ifdef CFG_DS301_IDENTITY
            case LSS_SWITCHGLOBAL_SERIAL:
                if(!bLssActive)
                {
                    dloc=candrv_get_unsigned32((CANDRV_PCOB)&tLssRxCob,8);
                    if(dloc==tDs301Identity.ulSerialNumber)
                    {
                        bLssActive=TRUE;
                        prNodeId=tWrkParam.bLssNodeId;
                        candrv_put_unsigned8(txcob,0,LSS_SWITCHEDSELECTIVE);
                        candrv_sendcob(tWrkParam.bCanController,txcob);
                    }
                }
                break;

            case LSS_SWITCHSELECT_VENDOR:
                if(!bLssActive)
                {
                    dloc=candrv_get_unsigned32((CANDRV_PCOB)&tLssRxCob,8);
                    if(dloc==tDs301Identity.ulVendorID)
                        wlssstatus=1;
                    else
                        wlssstatus=0;
                }
                break;

            case LSS_SWITCHSELECT_PRODUCT:
                if(!bLssActive)
                {
                    dloc=candrv_get_unsigned32((CANDRV_PCOB)&tLssRxCob,8);
                    if(dloc==tDs301Identity.ulProductCode&&wlssstatus==1)
                        wlssstatus=2;
                    else
                        wlssstatus=0;
                }
                break;

            case LSS_SWITCHSELECT_REVISION:
                if(!bLssActive)
                {
                    dloc=candrv_get_unsigned32((CANDRV_PCOB)&tLssRxCob,8);
                    if(dloc==tDs301Identity.ulRevision&&wlssstatus==2)
                        wlssstatus=3;
                    else
                        wlssstatus=0;
                }
                break;

            case LSS_SWITCHSELECT_SERIAL:
                if(!bLssActive)
                {
                    dloc=candrv_get_unsigned32((CANDRV_PCOB)&tLssRxCob,8);
                    if(dloc==tDs301Identity.ulSerialNumber&&wlssstatus==3)
                    {
                        bLssActive=TRUE;
                        prNodeId=tWrkParam.bLssNodeId;
                        candrv_put_unsigned8(txcob,0,LSS_SWITCHEDSELECTIVE);
                        candrv_sendcob(tWrkParam.bCanController,txcob);
                    }
                    else
                        wlssstatus=0;
                }
                break;
#endif

            case LSS_CONFIG_NODEID:
                if(bLssActive)
                {
                    candrv_put_unsigned8(txcob,0,LSS_CONFIG_NODEID);
                    bloc=candrv_get_unsigned8((CANDRV_PCOB)&tLssRxCob,8);
                    if(bloc>=DS301_LSS_NODEID_BEGIN&&bloc<=DS301_LSS_NODEID_END||bloc==DS301_LSS_UNCONFIGUREDNODEID)
                    {
                        if(tWrkParam.bLssNodeId!=bloc)
                            bNodeIdModified=TRUE;
                        tWrkParam.bLssNodeId=bloc;
                        tDs301Param.bLssNodeId=bloc;
                        candrv_put_unsigned8(txcob,8,0);
                    }
                    else
                        candrv_put_unsigned8(txcob,8,1);

                    candrv_sendcob(tWrkParam.bCanController,txcob);
                }
                break;

            case LSS_CONFIG_BITIMING:
                if(bLssActive)
                {
                    candrv_put_unsigned8(txcob,0,LSS_CONFIG_BITIMING);
                    candrv_put_unsigned8(txcob,8,1);

                    if(candrv_get_unsigned8((CANDRV_PCOB)&tLssRxCob,8)==0)
                    {
                        bloc=candrv_get_unsigned8((CANDRV_PCOB)&tLssRxCob,16);
                        if(candrv_testspeed(bloc))
                        {
                            tWrkParam.bLssTimingIndex=bloc;
                            tDs301Param.bLssTimingIndex=bloc;
                            candrv_put_unsigned8(txcob,8,0);
                        }
                    }

                    candrv_sendcob(tWrkParam.bCanController,txcob);
                }
                break;

            case LSS_ACTIVATE_BITIMING:
                if(bLssActive)
                {
                    candrv_stop(tWrkParam.bCanController);

                    wloc=timer_settimeout(uwSysTimers1ms,candrv_get_unsigned16((CANDRV_PCOB)&tLssRxCob,8));
                    while(!timer_istimedout(uwSysTimers1ms,wloc));

                    candrv_setspeed(tWrkParam.bCanController, tWrkParam.bLssTimingIndex);
                    candrv_run(tWrkParam.bCanController);

                    wloc=timer_settimeout(uwSysTimers1ms,candrv_get_unsigned16((CANDRV_PCOB)&tLssRxCob,8));
                    while(!timer_istimedout(uwSysTimers1ms,wloc));
                }
                break;

            case LSS_CONFIG_STORE:
                if(bLssActive)
                {
                    if(bNodeIdModified)
                        set_default_cobid(prNodeId);

                    candrv_put_unsigned8(txcob,0,LSS_CONFIG_STORE);
                    if((*tDs301CallBacks.pfExecLssStore)())
                    {
                        candrv_put_unsigned8(txcob,8,0);
                    }
                    else
                    {
                        candrv_put_unsigned8(txcob,8,2);    // error
                    }
                    candrv_sendcob(tWrkParam.bCanController,txcob);
                }
                break;

#ifdef CFG_DS301_IDENTITY
            case LSS_INQUIRE_VENDOR:
                if(bLssActive)
                {
                    candrv_put_unsigned8(txcob,0,LSS_INQUIRE_VENDOR);
                    candrv_put_unsigned32(txcob,8,tDs301Identity.ulVendorID);
                    candrv_sendcob(tWrkParam.bCanController,txcob);
                }
                break;

            case LSS_INQUIRE_PRODUCT:
                if(bLssActive)
                {
                    candrv_put_unsigned8(txcob,0,LSS_INQUIRE_PRODUCT);
                    candrv_put_unsigned32(txcob,8,tDs301Identity.ulProductCode);
                    candrv_sendcob(tWrkParam.bCanController,txcob);
                }
                break;

            case LSS_INQUIRE_REVISION:
                if(bLssActive)
                {
                    candrv_put_unsigned8(txcob,0,LSS_INQUIRE_REVISION);
                    candrv_put_unsigned32(txcob,8,tDs301Identity.ulRevision);
                    candrv_sendcob(tWrkParam.bCanController,txcob);
                }
                break;
#endif

            case LSS_INQUIRE_SERIAL:
                if(bLssActive)
                {
                    candrv_put_unsigned8(txcob,0,LSS_INQUIRE_SERIAL);
                    candrv_put_unsigned32(txcob,8,tDs301Identity.ulSerialNumber);
                    candrv_sendcob(tWrkParam.bCanController,txcob);
                }
                break;

            case LSS_INQUIRE_NODEID:
                if(bLssActive)
                {
                    candrv_put_unsigned8(txcob,0,LSS_INQUIRE_NODEID);
                    candrv_put_unsigned8(txcob,8,tWrkParam.bLssNodeId);
                    candrv_sendcob(tWrkParam.bCanController,txcob);
                }
                break;

#ifdef CFG_DS301_IDENTITY
            case LSS_IDENT_VENDOR:
                dloc=candrv_get_unsigned32((CANDRV_PCOB)&tLssRxCob,8);
                if(dloc==tDs301Identity.ulVendorID)
                    wlssstatus=1;
                else
                    wlssstatus=0;
                break;

            case LSS_IDENT_PRODUCT:
                dloc=candrv_get_unsigned32((CANDRV_PCOB)&tLssRxCob,8);
                if(dloc==tDs301Identity.ulProductCode&&wlssstatus==1)
                    wlssstatus=2;
                else
                    wlssstatus=0;
                break;

            case LSS_IDENT_REVISION_LOW:
                dloc=candrv_get_unsigned32((CANDRV_PCOB)&tLssRxCob,8);
                if(dloc<=tDs301Identity.ulRevision&&wlssstatus==2)
                    wlssstatus=3;
                else
                    wlssstatus=0;
                break;

            case LSS_IDENT_REVISION_HIGH:
                dloc=candrv_get_unsigned32((CANDRV_PCOB)&tLssRxCob,8);
                if(dloc>=tDs301Identity.ulRevision&&wlssstatus==3)
                    wlssstatus=4;
                else
                    wlssstatus=0;
                break;
#endif

            case LSS_IDENT_SERIAL_LOW:
                dloc=candrv_get_unsigned32((CANDRV_PCOB)&tLssRxCob,8);
                if(dloc<=tDs301Identity.ulSerialNumber&&wlssstatus==4)
                    wlssstatus=5;
                else
                    wlssstatus=0;
                break;

            case LSS_IDENT_SERIAL_HIGH:
                dloc=candrv_get_unsigned32((CANDRV_PCOB)&tLssRxCob,8);
                if(dloc>=tDs301Identity.ulSerialNumber&&wlssstatus==5)
                {
                    candrv_put_unsigned8(txcob,0,LSS_IDENT_SLAVE);
                    candrv_sendcob(tWrkParam.bCanController,txcob);
                }
                wlssstatus=0;
                break;

            case LSS_IDENT_NOCONFIGURED:
                if(tWrkParam.bLssNodeId==DS301_LSS_UNCONFIGUREDNODEID)
                {
                    candrv_put_unsigned8(txcob,0,LSS_IDENT_NOCONFIG_SLAVE);
                    candrv_sendcob(tWrkParam.bCanController,txcob);
                }
                break;
        }
    }

    if(bLssActive || tWrkParam.bLssNodeId==DS301_LSS_UNCONFIGUREDNODEID)
    {
        DS301_MON_SETBIT_16(wLocalCommands, CMD_LSSSWITCHEDON);
    }
    else
    {
        DS301_MON_CLEARBIT_16(wLocalCommands, CMD_LSSSWITCHEDON);
    }
}
#endif

//***************************************************************************
// SYNC manager

#ifdef CFG_DS301_SYNC
static void sync_mgr(void)
{
    BOOL bIsToProcess;

        // check if cobid parameters is changed
    if(tDs301Param.tSyncCob!=tWrkParam.tSyncCob)
    {
        DS301_COBIDENTRY cobid;

        DS301_MON_MOVE_32(cobid, tDs301Param.tSyncCob);

        if((cobid&DS301_COBIDMASK)==cobid && validate_generic_cobid(&cobid))
            tWrkParam.tSyncCob=cobid;
        else
        {
                // signal error
            DS301_MON_SETBIT_16(wLocalErrors, DS301_COMMERR_SYNC_WRONGCOBID);
        }
    }

        // check validity of SYNC object
    if(FALSE
#ifdef CFG_DS301_LSS
        || (wLocalCommands&CMD_LSSSWITCHEDON)
#endif
#ifdef CFG_DS301_NMT
        || (wNmtStatusReg&NMT_STOPPED)
#endif
#ifdef CFG_DS301_ERRPROT
        || (bErrCtrlRun==NMT_ERRPROT_BOOTUP)
#endif
        )
        bIsToProcess=FALSE;
    else
        bIsToProcess=TRUE;

        // if validity has changed
    if(bIsToProcess!=bSyncCobInstalled)
        if(bIsToProcess)
        {
            tSyncCob.id=DS301_GETCANIDS(DS301_CREATECOBIDENTRY(0,tWrkParam.tSyncCob));
            tSyncCob.len=0;
            memset(&(tSyncCob.flags),0,sizeof(tSyncCob.flags));
            tSyncCob.flags.bRxVHPR = TRUE;//tSyncCob.flags=CANDRV_F_DEFAULT|CANDRV_F_RX_VHPR;

            candrv_addrxcob(tWrkParam.bCanController, &tSyncCob, tDs301CallBacks.pfSyncEvent);
        }
        else
            candrv_deleterxcob(tWrkParam.bCanController, &tSyncCob);

    bSyncCobInstalled=bIsToProcess;

}
#endif

//***************************************************************************
// main loop for service

void ds301_loop(void)
{
        // if system is not fully operative do nothing
    if((*tDs301CallBacks.pfSysStatus)()!=DS301_FULLYOPERATIVE)
    {
        DS301_MON_CLEARBIT_16(wLocalCommands,CMD_FULLYOPERATIVE|CMD_ENTER_FULLYOPERATIVE);
        return;
    }
    else if(!(wLocalCommands&CMD_FULLYOPERATIVE))
    {
        DS301_MON_SETBIT_16(wLocalCommands,CMD_FULLYOPERATIVE|CMD_ENTER_FULLYOPERATIVE);
    }
    else
    {
        DS301_MON_CLEARBIT_16(wLocalCommands,CMD_ENTER_FULLYOPERATIVE);
    }

    candrvstatus_mgr();
    commfault_mgr();
    sdo_mgr(&tGPTxCob);

#ifdef CFG_DS301_EMCY
    emcy_mgr();
#endif

#ifdef CFG_DS301_LSS
    lss_mgr(&tGPTxCob);
#endif

#ifdef CFG_DS301_SYNC
    sync_mgr();
#endif

#ifdef CFG_DS301_NMT
        // keep last as it could trigger a reset and then
        // re-enter in the bootup state
    nmt_mgr();
#endif
}

//***************************************************************************
// Fault clear entry point

void ds301_faultclear(void)
{
        // signal fault clearing
    DS301_MON_SETBIT_16(wLocalCommands,CMD_COMMFAULT_RESET|CMD_EMCY_FORCE);
}
#endif
