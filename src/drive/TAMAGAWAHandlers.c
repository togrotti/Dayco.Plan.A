/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : TAMAGAWAHandlers.c                                         */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Low level TAMAGAWA series encoder interface                */
/*               management funcs                                           */
/*                                                                          */
/****************************************************************************/

#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "common\MathFunctions.h"
#include "drive\TAMAGAWAHandlers.h"
#include "fpga\FpgaHandler.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

#if CFG_ENC_TMGW
//***************************************************************************
// Defines

#define TMGWHANDLER_REQ_TMOUT              1250
#define TMGWHANDLER_REQ_INT                10

#define TMGW_STARTDELAY_FULLCYCLETIME      5000/2    // 125usec @50ns
#define TMGW_STARTDELAY_GUARDTIME          200/2     // 5usec @50ns
#define TMGW_STARTDELAY_STEP               40/2      // 1usec @50ns
#define TMGW_STARTDELAY_TIMEOUT            750/2     // usec @50ns

#define TMGW_TIMEWAIT_CMMMAND              40      // more than 40usec
#define TMGW_TIMEWAIT_CLEAR                18      // more than 18msec

#define TMGW_INC_CCW                       1

#define TMGW_RQC_POSDATA                   0
#define TMGW_RQC_ERRESET                   1
#define TMGW_RQC_STRESET                   2
#define TMGW_RQC_MTRESET                   3

#define TMGW_POWERUP_TIMEOUT               1000    // msec

//***************************************************************************
// Data Typs

typedef struct{
    UBYTE c0:1;
    UBYTE c1:1;
    UBYTE reserved:6;
} RQC;

//***************************************************************************
// Commands

//***************************************************************************
// Locals

static RQC sRqc[2][4]={{{0,0},{0,1},{1,1},{1,0}},
                       {{0,0},{1,0},{0,1},{1,1}}};

#ifdef _INFINEON_
static BOOL TMGWHandlers_Read(TMGW_WORKS huge * psWorks,UWORD uwRdAddr,HPUWORD hpuwOutData);
static BOOL TMGWHandlers_EncIDRead(TMGW_WORKS huge * psWorks,HPUWORD hpuwEncID);
static BOOL TMGWHandlers_Request(TMGW_WORKS huge * psWorks);
static BOOL TMGWHandlers_StatusRequest(TMGW_WORKS huge * psWorks,HPUWORD hpuwAlarm);
#else
static BOOL TMGWHandlers_Read(TMGW_WORKS * psWorks,UWORD uwRdAddr,HPUWORD hpuwOutData);
static BOOL TMGWHandlers_EncIDRead(TMGW_WORKS * psWorks,HPUWORD hpuwEncID);
static BOOL TMGWHandlers_Request(TMGW_WORKS * psWorks);
static BOOL TMGWHandlers_StatusRequest(TMGW_WORKS * psWorks,HPUWORD hpuwAlarm);
#endif

//***************************************************************************
// Interface initialization
#ifdef _INFINEON_
ULONG TMGWInitialization(ULONG ulBaseAddress,TMGW_WORKS huge * psWorks,BOOL bCalcStartDelay)
#else
ULONG TMGWInitialization(ULONG ulBaseAddress,TMGW_WORKS * psWorks,BOOL bCalcStartDelay)
#endif
{
    UWORD uwEncError;
    UWORD uwStartDelay;
    UWORD uwTimeOut;
    UWORD uwEncType;
        
    // set base address
    psWorks->ulBaseAddress = ulBaseAddress;

    // enter command mode
    TMGWEnterCommandMode(psWorks);
    
    // set encoder selector
    FPGA_TMGW_SET_ESEL(ulBaseAddress) = TMGW_ENCADDR_1;
    psWorks->ubEncAddr = TMGW_ENCADDR_1;
    
    // check encoder powerup
    uwTimeOut=timer_settimeout(uwSysTimers1ms, TMGW_POWERUP_TIMEOUT);
    FPGA_TMGW_SET_SELRT(ulBaseAddress) = TMGW_DEF_BAUDRATE;
    for(;;)
    {
        if(TMGWHandlers_StatusRequest(psWorks,&uwEncError))
            break;
            
        if(timer_istimedout(uwSysTimers1ms, uwTimeOut)) // wait for starting until timeout
            return TMGW_INIT_TIMEOUT;
    }
         
    // command: try to set maximum baudrate
    FPGA_TMGW_SET_SELRT(ulBaseAddress) = TMGW_BAUDRATE_50;
    if(TMGWHandlers_StatusRequest(psWorks,&uwEncError))
    {
        psWorks->ubBaudrate = TMGW_BAUDRATE_50;
        if(psWorks->ubDisMTData) // if multi-turn is disabled, ignore battery and overspeed error
            uwEncError&=(~(TMGW_ENCERR_BA_MASK|TMGW_ENCERR_BE_MASK|TMGW_ENCERR_OS_MASK));
            
        // detect encoder error, except battery error
        if((uwEncError&~TMGW_ENCERR_LATCH_MASK)!=0x0000) // skip the alarm: battery is removed when power-off
            return TMGW_INIT_ALARM|((ULONG)uwEncError<<16);
    }
    else
    {
        // if no reply with the maximum baudrate, try default 
        FPGA_TMGW_SET_SELRT(ulBaseAddress) = TMGW_DEF_BAUDRATE;
        if(TMGWHandlers_StatusRequest(psWorks,&uwEncError))
        {
            psWorks->ubBaudrate = TMGW_DEF_BAUDRATE;
            if(psWorks->ubDisMTData) // if multi-turn is disabled, ignore battery and overspeed error
                uwEncError&=(~(TMGW_ENCERR_BA_MASK|TMGW_ENCERR_BE_MASK|TMGW_ENCERR_OS_MASK));

            // detect encoder error, except battery error
            if((uwEncError&~TMGW_ENCERR_LATCH_MASK)!=0x0000) // skip the alarm: battery is removed when power-off
                return TMGW_INIT_ALARM|((ULONG)uwEncError<<16);
        }
        else
            return TMGW_INIT_TIMEOUT;
    }
    
    // read type of encoder
    if(!TMGWHandlers_EncIDRead(psWorks,&uwEncType))
        return TMGW_INIT_TIMEOUT;
    
    // check encoder ID
    switch(uwEncType)
    {
        case TMGW_ENID_17:
            psWorks->ubResolution = 23;
            psWorks->ubMultiTurn  = 16;
            break;
        case TMGW_ENID_11:
            psWorks->ubResolution = 17;
            psWorks->ubMultiTurn  = 16;
            break;
        case TMGW_ENID_0B:
            psWorks->ubResolution = 11;
            psWorks->ubMultiTurn  = 13;
            break;
        default:
            return TMGW_INIT_TYPEUNKNOW;
    }
    
    // if start delay is not requested
    if(!bCalcStartDelay)
    {
        TMGWEnterPositionMode(psWorks);
        return TMGW_INIT_SUCCESS;
    }
        
    // begin calibration of start delay
    FPGA_BASEOFF_16(ulBaseAddress, FPGA_TMGW_PRETRG)=uwStartDelay=TMGW_STARTDELAY_FULLCYCLETIME-TMGW_STARTDELAY_GUARDTIME;
    
    // enter position mode
    TMGWEnterPositionMode(psWorks);

    for(;;)
    {
            // cycle transaction
        uwTimeOut=timer_settimeout(uwSysTimers100ns, TMGW_STARTDELAY_TIMEOUT*10);
        while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) && !FPGA_TMGW_STAT_BUSY(FPGA_BASEOFF_16(ulBaseAddress,FPGA_TMGW_STATUS)));
        if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
            return TMGW_INIT_TRGDLYFAIL;

        uwTimeOut=timer_settimeout(uwSysTimers100ns, TMGW_STARTDELAY_TIMEOUT*10);
        while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) &&  FPGA_TMGW_STAT_BUSY(FPGA_BASEOFF_16(ulBaseAddress,FPGA_TMGW_STATUS)));
        if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
            return TMGW_INIT_TRGDLYFAIL;

            // check pre-trigger overtime fault
        if(!FPGA_TMGW_STAT_TRGFAULT(FPGA_BASEOFF_16(ulBaseAddress,FPGA_TMGW_STATUS)))
            break;

            // delay must be modified, check if possible, otherwise is not possible to
            // have position each cycle due to lower clock and higher transfer bit number
        if(uwStartDelay<=TMGW_STARTDELAY_GUARDTIME+TMGW_STARTDELAY_STEP)
            return TMGW_INIT_TRGDLYFAIL;

        uwStartDelay-=TMGW_STARTDELAY_STEP;
        FPGA_BASEOFF_16(ulBaseAddress, FPGA_TMGW_PRETRG)=uwStartDelay;
    }

    // apply guard time
    uwStartDelay-=TMGW_STARTDELAY_GUARDTIME;
    FPGA_BASEOFF_16(ulBaseAddress, FPGA_TMGW_PRETRG)=uwStartDelay;

    // wait for clean transaction
    uwTimeOut=timer_settimeout(uwSysTimers100ns, TMGW_STARTDELAY_TIMEOUT*10);
    while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) && !FPGA_TMGW_STAT_BUSY(FPGA_BASEOFF_16(ulBaseAddress,FPGA_TMGW_STATUS)));
    if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
        return TMGW_INIT_TIMEOUT;

    uwTimeOut=timer_settimeout(uwSysTimers100ns, TMGW_STARTDELAY_TIMEOUT*10);
    while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) &&  FPGA_TMGW_STAT_BUSY(FPGA_BASEOFF_16(ulBaseAddress,FPGA_TMGW_STATUS)));
    if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
        return TMGW_INIT_TIMEOUT;
        
    return TMGW_INIT_SUCCESS;
}

//***************************************************************************
// Enter Command Mode
#ifdef _INFINEON_
void TMGWEnterCommandMode(TMGW_WORKS huge * psWorks)
#else
void TMGWEnterCommandMode(TMGW_WORKS * psWorks)
#endif
{    
    // set request code
    FPGA_TMGW_SET_RQC0(psWorks->ulBaseAddress) = sRqc[psWorks->ubEncAddr][TMGW_RQC_POSDATA].c0;
    FPGA_TMGW_SET_RQC1(psWorks->ulBaseAddress) = sRqc[psWorks->ubEncAddr][TMGW_RQC_POSDATA].c1;

    FPGA_TMGW_SET_CMD(psWorks->ulBaseAddress) = FALSE;
    psWorks->ubCmdMode = TRUE;
}

//***************************************************************************
// Enter Position Mode, automatically trigger by external signal
#ifdef _INFINEON_
void TMGWEnterPositionMode(TMGW_WORKS huge * psWorks)
#else
void TMGWEnterPositionMode(TMGW_WORKS * psWorks)
#endif
{    
    // set request code
    FPGA_TMGW_SET_RQC0(psWorks->ulBaseAddress) = sRqc[psWorks->ubEncAddr][TMGW_RQC_POSDATA].c0;
    FPGA_TMGW_SET_RQC1(psWorks->ulBaseAddress) = sRqc[psWorks->ubEncAddr][TMGW_RQC_POSDATA].c1;
    
    FPGA_TMGW_SET_CMD(psWorks->ulBaseAddress) = TRUE;
    psWorks->ubCmdMode = FALSE;
}

//***************************************************************************
// Read Position, Realtime
#ifdef _INFINEON_
BOOL TMGWReadPosition(TMGW_WORKS huge * psWorks, HPSLONG hpslPosLo, HPSLONG hpslPosHi, BOOL bRdHi)
#else
BOOL TMGWReadPosition(TMGW_WORKS * psWorks, HPSLONG hpslPosLo, HPSLONG hpslPosHi, BOOL bRdHi)
#endif
{
    union{
        struct{
            UWORD lo;
            UWORD hi;
        }w;
        SLONG l;
    }slRData;
                
    // read absolute data in one resolution
    if(!TMGWHandlers_Read(psWorks,TMGW_RDADDR_ABS_LL,&slRData.w.lo))
        return FALSE;
    
    if(!TMGWHandlers_Read(psWorks,TMGW_RDADDR_ABS_LH,&slRData.w.hi))
        return FALSE;
        
    slRData.l = slRData.l<<(32-psWorks->ubResolution);
    *hpslPosLo = TMGW_INC_CCW ? -slRData.l : slRData.l;

    // read multi-turn data
    if(bRdHi&&(psWorks->ubMultiTurn!=0))
    {
        if(!TMGWHandlers_Read(psWorks,TMGW_RDADDR_ABS_HL,&slRData.w.lo))
            return FALSE;
        slRData.w.lo<<=8;
        if(!TMGWHandlers_Read(psWorks,TMGW_RDADDR_ABS_HH,&slRData.w.hi))
            return FALSE;
                    
        slRData.l = TMGW_INC_CCW ? (-(slRData.l<<(24-psWorks->ubMultiTurn)))>>(32-psWorks->ubMultiTurn) : slRData.l>>8;
        *hpslPosHi = slRData.l;
    }
    else
        *hpslPosHi = 0;
    
    return TRUE;
}

//***************************************************************************
// Encoder Status Flag Clear
#ifdef _INFINEON_
BOOL TMGWHandlers_StatusFlagClear(TMGW_WORKS huge * psWorks)
#else
BOOL TMGWHandlers_StatusFlagClear(TMGW_WORKS * psWorks)
#endif
{
    UBYTE ubCnt;
    
    // set request code
    FPGA_TMGW_SET_RQC0(psWorks->ulBaseAddress) = sRqc[psWorks->ubEncAddr][TMGW_RQC_ERRESET].c0;
    FPGA_TMGW_SET_RQC1(psWorks->ulBaseAddress) = sRqc[psWorks->ubEncAddr][TMGW_RQC_ERRESET].c1;
    
    for(ubCnt=0;ubCnt<10;ubCnt++)
    {
        if(!TMGWHandlers_Request(psWorks))
            return FALSE;            
        
        timer_wait(uwSysTimers100ns,TMGW_TIMEWAIT_CMMMAND); // wait 40us between two commands
    }
    
    return TRUE;
}

//***************************************************************************
// Encoder Error and Multiple Rotation Data Clear
#ifdef _INFINEON_
BOOL TMGWHandlers_EMClear(TMGW_WORKS huge * psWorks)
#else
BOOL TMGWHandlers_EMClear(TMGW_WORKS * psWorks)
#endif
{
    UBYTE ubCnt;
    
    // set request code
    FPGA_TMGW_SET_RQC0(psWorks->ulBaseAddress) = sRqc[psWorks->ubEncAddr][TMGW_RQC_MTRESET].c0;
    FPGA_TMGW_SET_RQC1(psWorks->ulBaseAddress) = sRqc[psWorks->ubEncAddr][TMGW_RQC_MTRESET].c1;
    
    for(ubCnt=0;ubCnt<10;ubCnt++)
    {
        if(!TMGWHandlers_Request(psWorks))
            return FALSE;
            
        timer_wait(uwSysTimers100ns,TMGW_TIMEWAIT_CMMMAND); // wait 40us between two commands
    }
    
    return TRUE;
}

//***************************************************************************
// Encoder Single Turn Zero Position Set
BOOL TMGWHandlers_ZeroPosSet(TMGW_WORKS * psWorks)
{
    UBYTE ubCnt;
    
    // set request code
    FPGA_TMGW_SET_RQC0(psWorks->ulBaseAddress) = sRqc[psWorks->ubEncAddr][TMGW_RQC_STRESET].c0;
    FPGA_TMGW_SET_RQC1(psWorks->ulBaseAddress) = sRqc[psWorks->ubEncAddr][TMGW_RQC_STRESET].c1;
    
    for(ubCnt=0;ubCnt<10;ubCnt++)
    {
        if(!TMGWHandlers_Request(psWorks))
            return FALSE;
            
        timer_wait(uwSysTimers100ns,TMGW_TIMEWAIT_CMMMAND); // wait 40us between two commands
    }
    
    timer_wait(uwSysTimers1ms,TMGW_TIMEWAIT_CLEAR); // wait 18ms after reseting one resolution data
    
    return TRUE;
}


//***************************************************************************
// Encoder Status Request
#ifdef _INFINEON_
static BOOL TMGWHandlers_StatusRequest(TMGW_WORKS huge * psWorks,HPUWORD hpuwAlarm)
#else
static BOOL TMGWHandlers_StatusRequest(TMGW_WORKS * psWorks,HPUWORD hpuwAlarm)
#endif
{
    BOOL  bRetVal=TRUE;
        
    // send request in command mode
    if(psWorks->ubCmdMode)
        bRetVal = TMGWHandlers_Request(psWorks);
        
    // read output register
    if(bRetVal&!TMGWHandlers_Read(psWorks,TMGW_RDADDR_ENCERR,hpuwAlarm))
        bRetVal=FALSE;
    
    return bRetVal;
}

//***************************************************************************
// Encoder ID
#ifdef _INFINEON_
static BOOL TMGWHandlers_EncIDRead(TMGW_WORKS huge * psWorks,HPUWORD hpuwEncID)
#else
static BOOL TMGWHandlers_EncIDRead(TMGW_WORKS * psWorks,HPUWORD hpuwEncID)
#endif
{
    BOOL  bRetVal=TRUE;
    UWORD uwEncID;
        
    // send request commnd
    if(psWorks->ubCmdMode)
        bRetVal = TMGWHandlers_Request(psWorks);
        
    // read output register
    if(bRetVal)
    {
        if(TMGWHandlers_Read(psWorks,TMGW_RDADDR_ENID,&uwEncID))
            *hpuwEncID=uwEncID>>(16-TMGW_ENID_NBITS);
        else
            bRetVal&=FALSE;
    }
    
    return bRetVal;
}

//***************************************************************************
// Read Output Register
#ifdef _INFINEON_
static BOOL TMGWHandlers_Read(TMGW_WORKS huge * psWorks,UWORD uwRdAddr,HPUWORD hpuwOutData)
#else
static BOOL TMGWHandlers_Read(TMGW_WORKS * psWorks,UWORD uwRdAddr,HPUWORD hpuwOutData)
#endif
{
    ULONG ulBaseAddress = psWorks->ulBaseAddress;
    
    FPGA_TMGW_SET_DS40(ulBaseAddress) = (BOOL)(uwRdAddr&~TMGW_RDADDR_MASK) ? TRUE : FALSE;
    
    *hpuwOutData = FPGA_BASEOFF_16(ulBaseAddress,FPGA_TMGW_DATA_0+((uwRdAddr&TMGW_RDADDR_MASK)<<1));
    return TRUE;
}

//***************************************************************************
// Request Handler
#ifdef _INFINEON_
static BOOL TMGWHandlers_Request(TMGW_WORKS huge * psWorks)
#else
static BOOL TMGWHandlers_Request(TMGW_WORKS * psWorks)
#endif
{
    UWORD uwTOut;
    ULONG ulBaseAddress;
    UWORD uwStatus;
    
    ulBaseAddress = psWorks->ulBaseAddress;

    uwTOut=timer_settimeout(uwSysTimers100ns, TMGWHANDLER_REQ_TMOUT); // set timeout parameter
    for(;;)
    {
        uwStatus=FPGA_BASEOFF_16(ulBaseAddress,FPGA_TMGW_STATUS);
        if(!FPGA_TMGW_STAT_BUSY(uwStatus)|FPGA_TMGW_STAT_TIMEOUT(uwStatus))
            break;
        if(timer_istimedout(uwSysTimers100ns, uwTOut))
            return FALSE;
    }

    FPGA_TMGW_SET_REQ(ulBaseAddress) = TRUE;
    timer_wait(uwSysTimers100ns,TMGWHANDLER_REQ_INT); // manual mode strobe signal time 1us
    FPGA_TMGW_SET_REQ(ulBaseAddress) = FALSE;

    for(;;)
    {
        uwStatus=FPGA_BASEOFF_16(ulBaseAddress,FPGA_TMGW_STATUS);
        if((!FPGA_TMGW_STAT_BUSY(uwStatus))&&(!FPGA_TMGW_STAT_TIMEOUT(uwStatus))&&FPGA_TMGW_STAT_EOF(uwStatus))
            break;
        if(timer_istimedout(uwSysTimers100ns, uwTOut))
            return FALSE;
    }
    
    return TRUE;
}

//***************************************************************************
// Read encoder error, Realtime
#ifdef _INFINEON_
BOOL TMGWReadError(TMGW_WORKS huge * psWorks, HPUWORD hpuwEncError)
#else
BOOL TMGWReadError(TMGW_WORKS * psWorks, HPUWORD hpuwEncError)
#endif
{
    BOOL  bRetVal=TRUE;
                
    // read output register
    if(!TMGWHandlers_Read(psWorks,TMGW_RDADDR_ENCERR,hpuwEncError))
        bRetVal=FALSE;
    
    return bRetVal;
}
#endif
