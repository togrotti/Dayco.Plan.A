/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : PanelManager.c                                             */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Panel leds and keyboard management                         */
/*                                                                          */
/****************************************************************************/

#include "common\CommonDefines.h"
#include "system\SystemStatus.h"
#include "system\SystemAlarms.h"
#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonUtility.h"
//#include "FatalErrorCodes.h"
#include "drive\PanelManager.h"
#include "common\TaskScheduler.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

//***************************************************************************
// Defines

#define IDLE_LED_TIMEOUT        100         // msec
#define NOIDLE_LED_TIMEOUT      200         // msec
#define POWER_LED_TIMEOUT       100         // msec
#define LED_CYCLE_STATUS        10
#define CYCLESUBCODE_TIMEOUT    1000        // msec

//***************************************************************************
// Data structure

typedef union
{
    struct
    {
        UWORD bS0:2;    // bit1: red led; bit0: green led
        UWORD bS1:2;
        UWORD bS2:2;
        UWORD bS3:2;
        UWORD bS4:2;
        UWORD bS5:2;
        UWORD bS6:2;
        UWORD bS7:2;
        UWORD bS8:2;
        UWORD bS9:2;
        UWORD bNStat:4;
        UWORD bCycle:4; // bCycle * 50 msec
    } sLStat;
    ULONG ulLStat;
} PNLMGR_RDLEDSTATUS;

//***************************************************************************
// Globals

ULONG ulPnlMgrRDLedStatus=0ul;

//***************************************************************************
// Remote Display Led pattern definitions

static const PNLMGR_RDLEDSTATUS  sRemDispStat[]=
{
    {0x1,0x3,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,  2, 2},    // RDLED_LOCKEDBYPLCRELOAD
    {0x2,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,  2, 4},    // RDLED_ALARM
    {0x2,0x1,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,  2, 4},    // RDLED_LOCKEDBYBOOTERROR
    {0x1,0x0,0x1,0x0,0x0,0x0,0x0,0x0,0x0,0x0,  8, 2},    // RDLED_POWERENABLED
    {0x3,0x0,0x1,0x0,0x0,0x0,0x0,0x0,0x0,0x0,  8, 2},    // RDLED_ENABLEDANDWARNING
    {0x1,0x1,0x1,0x1,0x1,0x0,0x0,0x0,0x0,0x0, 10, 2},    // RDLED_IDLE
    {0x3,0x1,0x1,0x1,0x1,0x0,0x0,0x0,0x0,0x0, 10, 2},    // RDLED_WARNING
};

#define RDLED_LOCKEDBYPLCRELOAD         0
#define RDLED_ALARM                     1
#define RDLED_LOCKEDBYBOOTERROR         2
#define RDLED_POWERENABLED              3
#define RDLED_ENABLEDANDWARNING         4
#define RDLED_IDLE                      5
#define RDLED_WARNING                   6

//***************************************************************************
// Local prototypes;

static void setupled(UWORD);
static void slowtask(void);

//***************************************************************************
// Init

BOOL PanelMgr_Init(void)
{
    assert(TaskSched_AddBackgroundTask(&slowtask));

    return TRUE;
}

//***************************************************************************
// Setup right led pattern

static void setupled(UWORD uwPattern)
{
#ifndef _HW_DC
    UWORD uwTmp, uwCode;

    for(uwTmp=uwCode=0; uwTmp<8; uwTmp++)
    {
        uwCode=(uwCode<<1)|(uwPattern&0x01);
        uwPattern>>=1;
    }

    LEDS_OUTPUT(uwCode);
#else
    LEDS_OUTPUT(uwPattern);
#endif
}

//***************************************************************************
// Slow task

static void slowtask(void)
{
    static BOOL bInit=TRUE,bPrevBtn=FALSE;
    static UBYTE ubSelCnt=0;
    static UBYTE ubSelNum=0;
    static UWORD uwLedTimer;
    static UBYTE pubLedStatus[LED_CYCLE_STATUS];
    BOOL bActBtn=FALSE;
    ULONG ulLocSystemAlarms;
    ULONG ulTmp;
    UWORD ulSelTimeOut=IDLE_LED_TIMEOUT;
    UWORD uwRDLedSelector;

    atomic_read(&ulLocSystemAlarms,&ulSystemAlarms,sizeof(ulSystemAlarms));
    atomic_read(&ulTmp,&ulSystemAlarmSubCode,sizeof(ulSystemAlarmSubCode));

#ifndef _HW_DC
    if(XE167_DIAG_BUTN1 && ulTmp)
    {
        pubLedStatus[0]=(UBYTE)( ulTmp    &0xff);
        pubLedStatus[1]=(UBYTE)( ulTmp    &0xff);
        pubLedStatus[2]=(UBYTE)((ulTmp>>8)&0xff);
        pubLedStatus[3]=(UBYTE)((ulTmp>>8)&0xff);
        pubLedStatus[4]=0x00;
        ubSelNum=5;
        ulSelTimeOut=CYCLESUBCODE_TIMEOUT;
        bActBtn=TRUE;
    }
    else if(XE167_DIAG_BUTN1 && ulSystemWarnings)
    {
        UWORD uwCt;

        atomic_read(&ulTmp,&ulSystemWarnings,sizeof(ulSystemWarnings)); 

        for(uwCt=0; uwCt<32 && (ulTmp&1)==0; uwCt++,ulTmp>>=1);
        uwCt<<=3;

        pubLedStatus[0]=(UBYTE)uwCt;
        pubLedStatus[1]=(UBYTE)uwCt;
        ubSelNum=2;
        ulSelTimeOut=NOIDLE_LED_TIMEOUT;
        bActBtn=TRUE;
    }
#endif
    
    if(bSysStatLockedByPlcReload)
    {
        if(!bActBtn)
        {
#ifndef _HW_DC
            pubLedStatus[0]=0x01;
            pubLedStatus[1]=0x05;
#else
            pubLedStatus[0]=LEDS_OUT_GRN;
            pubLedStatus[1]=LEDS_OUT_RED|LEDS_OUT_GRN;
#endif
            ubSelNum=2;
            ulSelTimeOut=IDLE_LED_TIMEOUT;
        }
        uwRDLedSelector=RDLED_LOCKEDBYPLCRELOAD;
    }
    else if(ulLocSystemAlarms)
    {
        if(!bActBtn)
        {
#ifndef _HW_DC
            UWORD uwCt;
    
            for(uwCt=0; uwCt<32 && (ulLocSystemAlarms&1)==0; uwCt++,ulLocSystemAlarms>>=1);
    
            uwCt<<=3;
    
            pubLedStatus[0]=(UBYTE)uwCt|0x04;
            pubLedStatus[1]=(UBYTE)uwCt;
#else
            pubLedStatus[0]=0;
            pubLedStatus[1]=LEDS_OUT_RED;
#endif
            ubSelNum=2;
            ulSelTimeOut=NOIDLE_LED_TIMEOUT;
        }
        uwRDLedSelector=RDLED_ALARM;
    }
    else if(bSysStatLockedByBootError)
    {
        if(!bActBtn)
        {
#ifndef _HW_DC
            pubLedStatus[0]=0x04;
            pubLedStatus[1]=0x02;
#else
            pubLedStatus[0]=LEDS_OUT_RED;
            pubLedStatus[1]=LEDS_OUT_YEL;
#endif
            ubSelNum=2;
            ulSelTimeOut=NOIDLE_LED_TIMEOUT;
        }
        uwRDLedSelector=RDLED_LOCKEDBYBOOTERROR;
    }
    else if(bSysStatPowerEnabled)
    {
        if(!bActBtn)
        {
            if(ulSystemWarnings)
            {
#ifndef _HW_DC
                pubLedStatus[0]=0x05;
                pubLedStatus[1]=0x00;
                pubLedStatus[2]=0x01;
                pubLedStatus[3]=0x00;
                pubLedStatus[4]=0x00;
                pubLedStatus[5]=0x00;
                pubLedStatus[6]=0x00;
                pubLedStatus[7]=0x00;
#else
                pubLedStatus[0]=LEDS_OUT_YEL|LEDS_OUT_GRN;
                pubLedStatus[1]=0x00;
                pubLedStatus[2]=LEDS_OUT_GRN;
                pubLedStatus[3]=0x00;
                pubLedStatus[4]=0x00;
                pubLedStatus[5]=0x00;
                pubLedStatus[6]=0x00;
                pubLedStatus[7]=0x00;
#endif
            }
            else
            {
#ifndef _HW_DC
                pubLedStatus[0]=0x01;
                pubLedStatus[1]=0x00;
                pubLedStatus[2]=0x01;
                pubLedStatus[3]=0x00;
                pubLedStatus[4]=0x00;
                pubLedStatus[5]=0x00;
                pubLedStatus[6]=0x00;
                pubLedStatus[7]=0x00;
#else
                pubLedStatus[0]=LEDS_OUT_GRN;
                pubLedStatus[1]=0x00;
                pubLedStatus[2]=LEDS_OUT_GRN;
                pubLedStatus[3]=0x00;
                pubLedStatus[4]=0x00;
                pubLedStatus[5]=0x00;
                pubLedStatus[6]=0x00;
                pubLedStatus[7]=0x00;
#endif
            }
            ubSelNum=8;
            ulSelTimeOut=POWER_LED_TIMEOUT;
        }
        if(ulSystemWarnings)
            uwRDLedSelector=RDLED_ENABLEDANDWARNING;
        else
            uwRDLedSelector=RDLED_POWERENABLED;
    }
    else
    {
        if(!bActBtn)
        {
            if(ulSystemWarnings)
            {
#ifndef _HW_DC
                pubLedStatus[0]=0x05;
                pubLedStatus[1]=0x01;
                pubLedStatus[2]=0x01;
                pubLedStatus[3]=0x01;
                pubLedStatus[4]=0x01;
                pubLedStatus[5]=0x00;
                pubLedStatus[6]=0x00;
                pubLedStatus[7]=0x00;
                pubLedStatus[8]=0x00;
                pubLedStatus[9]=0x00;
#else
                pubLedStatus[0]=LEDS_OUT_YEL|LEDS_OUT_GRN;
                pubLedStatus[1]=LEDS_OUT_GRN;
                pubLedStatus[2]=LEDS_OUT_GRN;
                pubLedStatus[3]=LEDS_OUT_GRN;
                pubLedStatus[4]=0x00;
                pubLedStatus[5]=0x00;
                pubLedStatus[6]=0x00;
                pubLedStatus[7]=0x00;
                pubLedStatus[8]=0x00;
                pubLedStatus[9]=0x00;
#endif
            }
            else
            {
#ifndef _HW_DC
                pubLedStatus[0]=0x01;
                pubLedStatus[1]=0x01;
                pubLedStatus[2]=0x01;
                pubLedStatus[3]=0x01;
                pubLedStatus[4]=0x01;
                pubLedStatus[5]=0x00;
                pubLedStatus[6]=0x00;
                pubLedStatus[7]=0x00;
                pubLedStatus[8]=0x00;
                pubLedStatus[9]=0x00;
#else
                pubLedStatus[0]=LEDS_OUT_GRN;
                pubLedStatus[1]=LEDS_OUT_GRN;
                pubLedStatus[2]=LEDS_OUT_GRN;
                pubLedStatus[3]=LEDS_OUT_GRN;
                pubLedStatus[4]=LEDS_OUT_GRN;
                pubLedStatus[5]=0x00;
                pubLedStatus[6]=0x00;
                pubLedStatus[7]=0x00;
                pubLedStatus[8]=0x00;
                pubLedStatus[9]=0x00;
#endif
            }
            ubSelNum=10;
            ulSelTimeOut=IDLE_LED_TIMEOUT;
        }
        if(ulSystemWarnings)
            uwRDLedSelector=RDLED_WARNING;
        else
            uwRDLedSelector=RDLED_IDLE;
    }

        // if button then restart counter
    if(bPrevBtn!=bActBtn)
        ubSelCnt=ubSelNum;

        // if init or buttons, then immediate retrigger
    if(bInit || bPrevBtn!=bActBtn)
    {
            // cause immediate timeout triggering
        uwLedTimer=timer_settimeout(uwSysTimers1ms, 0);
        bInit=FALSE;
        bPrevBtn=bActBtn;
    }

    if(timer_istimedout(uwSysTimers1ms, uwLedTimer))
    {
        uwLedTimer=timer_settimeout(uwLedTimer, ulSelTimeOut);

        if(++ubSelCnt>=ubSelNum)
            ubSelCnt=0;
    }

        // if no button then update communication led, that in case of diagnostic
        // application active it follow power status led
    if(!bActBtn)
#ifndef _HW_DC
        if(uwSystemEnterDiagnostic==SYSTEM_ENTERDIAGNOSTIC_KEY)
            pubLedStatus[ubSelCnt]=(UBYTE)((pubLedStatus[ubSelCnt]&~0x02)|((pubLedStatus[ubSelCnt]&0x01)<<1));

        else if(bSysStatPnlMgrPacketReceived)
        {
            pubLedStatus[ubSelCnt]|=0x02;
            bSysStatPnlMgrPacketReceived=FALSE;
        }
        else
            pubLedStatus[ubSelCnt]&=~0x02;
#else
        if(uwSystemEnterDiagnostic==SYSTEM_ENTERDIAGNOSTIC_KEY)
            pubLedStatus[ubSelCnt]=(UBYTE)((pubLedStatus[ubSelCnt]&~LEDS_OUT_RED)|
                                   (pubLedStatus[ubSelCnt]&LEDS_OUT_GRN?LEDS_OUT_RED:0));
#endif

    setupled(pubLedStatus[ubSelCnt]);

        // update remote display led pattern
    atomic_write(&ulPnlMgrRDLedStatus, &sRemDispStat[uwRDLedSelector].ulLStat, sizeof(ULONG));
}
        

    
