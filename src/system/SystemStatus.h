/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SystemStatus.h                                             */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Global internal status of the drive, in order to filter    */
/*               which tasks are to be executed and the write denial for    */
/*               common db entries                                          */
/*                                                                          */
/****************************************************************************/

#ifndef _SYSTEMSTATUS_H
#define _SYSTEMSTATUS_H

#include "system\SysAppGlobals.h"

//***************************************************************************
// Global system status

#define SYSTEMSTATUS_BIT_BOOTING                0
#define SYSTEMSTATUS_BIT_RESETTING              1
#define SYSTEMSTATUS_BIT_POWER_READY            2
#define SYSTEMSTATUS_BIT_POWER_ZEROELFLDSEEK    3
#define SYSTEMSTATUS_BIT_POWER_ENABLED          4
#define SYSTEMSTATUS_BIT_FAULT                  5
#define SYSTEMSTATUS_BIT_FAULTREACTION          6
#define SYSTEMSTATUS_BIT_PROGRAMFLASHWRITING    7
#define SYSTEMSTATUS_BIT_PARAMETERSSAVING       8
#define SYSTEMSTATUS_BIT_SYNCMGR_REQUIRED       9
#define SYSTEMSTATUS_BIT_LOCKEDBYBOOTERROR      10
#define SYSTEMSTATUS_BIT_DISABLEOVERTIMECHECK   11
#define SYSTEMSTATUS_BIT_SYSLOGFLUSH            12
#define SYSTEMSTATUS_BIT_FIELDBUS_SYNCING       13
#define SYSTEMSTATUS_BIT_LOCKEDBYPLCRELOAD      14
#define SYSTEMSTATUS_BIT_FIELDBUS_TSRESYNC      15
#define SYSTEMSTATUS_BIT_PLC_RUNNING            16
#define SYSTEMSTATUS_BIT_PLC_IMG_SLOW_IN        17
#define SYSTEMSTATUS_BIT_PLC_IMG_SLOW_OUT       18
#define SYSTEMSTATUS_BIT_PLC_IMG_BACKGROUND_IN  19
#define SYSTEMSTATUS_BIT_PLC_IMG_BACKGROUND_OUT 20
#define SYSTEMSTATUS_BIT_ALARMS_RESET           21
#define SYSTEMSTATUS_BIT_PDOMGR_LOCKCONFIG      22
#define SYSTEMSTATUS_BIT_PNLMGR_PACKETRECEIVED  23
#define SYSTEMSTATUS_BIT_PLC_RUN_SLOW           24
#define SYSTEMSTATUS_BIT_PLC_RUN_BACKGROUND     25
#define SYSTEMSTATUS_BIT_CANOPEN_SYNC           26
#define SYSTEMSTATUS_BIT_ETHERCAT_SYNC          27
#define SYSTEMSTATUS_BIT_ECATPDOMGR_LOCKCONFIG  28
#define SYSTEMSTATUS_BIT_PLC_OVERTIME_DETECTED  29
#define SYSTEMSTATUS_BIT_EM_DISABLEABSPROCESS   30
#define SYSTEMSTATUS_BIT_ANALOG_REFRESHING      31

#define SYSTEMSTATUS_MASK(x)                    (1ul<<(x))

#define SYSTEMSTATUS2_BIT_ASC_ACTIVATED          1

//***************************************************************************
// Data types

typedef ULONG       SYSTEMSTATUS;

typedef struct {
   //ULONG l[32];
	BOOL b[32];
} SYSTEMSTATUSFLAGS;

//****************************************************************************
// Global system status
extern volatile SYSTEMSTATUSFLAGS           sSystemStatus;
extern volatile SYSTEMSTATUS                ulSystemStatus;

#define bSysStatBooting                     HBITL(ulSystemStatus)->bit0
#define bSysStatResetting                   HBITL(ulSystemStatus)->bit1
#define bSysStatPowerReady                  HBITL(ulSystemStatus)->bit2
#define bSysStatPowerZeroElFldSeek          HBITL(ulSystemStatus)->bit3
#define bSysStatPowerEnabled                HBITL(ulSystemStatus)->bit4
#define bSysStatFault                       HBITL(ulSystemStatus)->bit5
#define bSysStatFaultReaction               HBITL(ulSystemStatus)->bit6
#define bSysStatProgramFlashWriting         HBITL(ulSystemStatus)->bit7
#define bSysStatParametersSaving            HBITL(ulSystemStatus)->bit8
#define bSysStatSyncMgrRequired             HBITL(ulSystemStatus)->bit9
#define bSysStatLockedByBootError           HBITL(ulSystemStatus)->bit10
#define bSysStatDisableOverTimeCheck        HBITL(ulSystemStatus)->bit11
#define bSysStatSysLogFlush                 HBITL(ulSystemStatus)->bit12
#define bSysStatFieldbusSyncing             HBITL(ulSystemStatus)->bit13
#define bSysStatLockedByPlcReload           HBITL(ulSystemStatus)->bit14
#define bSysStatFieldbusTSReSync            HBITL(ulSystemStatus)->bit15
#define bSysStatPlcRunning                  HBITL(ulSystemStatus)->bit16
#define bSysStatPlcImgSlowIn                HBITL(ulSystemStatus)->bit17
#define bSysStatPlcImgSlowOut               HBITL(ulSystemStatus)->bit18
#define bSysStatPlcImgBackgroundIn          HBITL(ulSystemStatus)->bit19
#define bSysStatPlcImgBackgroundOut         HBITL(ulSystemStatus)->bit20
#define bSysStatAlarmsReset                 HBITL(ulSystemStatus)->bit21
#define bSysStatPdoMgrLockConfig            HBITL(ulSystemStatus)->bit22
#define bSysStatPnlMgrPacketReceived        HBITL(ulSystemStatus)->bit23
#define bSysStatPlcRunSlow                  HBITL(ulSystemStatus)->bit24
#define bSysStatPlcRunBackground            HBITL(ulSystemStatus)->bit25
#define bSysStatCANOpenSync                 HBITL(ulSystemStatus)->bit26
#define bSysStatEtherCATSync                HBITL(ulSystemStatus)->bit27
#define bSysStatEcatPdoMgrLockConfig        HBITL(ulSystemStatus)->bit28
#define bSysStatPlcOvertimeDetected         HBITL(ulSystemStatus)->bit29
#define bSysStatEmDisableAbsProcess         HBITL(ulSystemStatus)->bit30
#define bSysStatAnalogRefreshing            HBITL(ulSystemStatus)->bit31

extern volatile SYSTEMSTATUSFLAGS           sSystemStatus2;
extern volatile SYSTEMSTATUS                ulSystemStatus2;

#define bSysStatLockedByAscActivated        HBITL(ulSystemStatus2)->bit0


//****************************************************************************
// Boot Error codes

extern UWORD uwSystemBootErrorCode;

#define SYSTEMBOOTERR_INVALIDHWCONFIG           0x0001
#define SYSTEMBOOTERR_INVALIDFLASHPARAMETERS    0x0002
#define SYSTEMBOOTERR_STARTWITHPOWERFAIL        0x0003
#define SYSTEMBOOTERR_WDTINSTALLFAIL            0x0004
#define SYSTEMBOOTERR_PARAMETERERROR            0x0005
#define SYSTEMBOOTERR_UNSUPPORTEDGLBOPT         0x0006
#define SYSTEMBOOTERR_FPGAPROGFAIL              0x0010
#define SYSTEMBOOTERR_FPGAINCOMPAT              0x0018
#define SYSTEMBOOTERR_FPGATESTFAIL              0x0019
#define SYSTEMBOOTERR_IOEXP_PROGFAIL            0x0020
#define SYSTEMBOOTERR_IOEXP_INCOMPAT            0x0028
#define SYSTEMBOOTERR_PSU_INCOMPAT              0x0029
#define SYSTEMBOOTERR_POSTFAILURE               0x0040

#define SYSTEMBOOTERR_INITTASKFAILBASE          0x0100

#endif
