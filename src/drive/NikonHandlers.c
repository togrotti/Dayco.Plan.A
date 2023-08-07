/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : NikonHandlers.c                                            */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Low level Nikon series encoder interface management funcs  */
/*                                                                          */
/****************************************************************************/

#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "common\MathFunctions.h"
#include "drive\NikonHandlers.h"
#include "fpga\FpgaHandler.h"
#include "common\TaskScheduler.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

#if CFG_ENC_NIKON
//***************************************************************************
// Defines
#define NIKONHANDLER_SET_A_OFF              0
#define NIKONHANDLER_SET_M_OFF              8
#define NIKONHANDLER_SET_RES_OFF            9
#define NIKONHANDLER_SETRG(r,m,a)           (((UWORD)r<<NIKONHANDLER_SET_RES_OFF)+\
                                             ((UWORD)m<<NIKONHANDLER_SET_M_OFF)+\
                                             ((UWORD)a<<NIKONHANDLER_SET_A_OFF))

#define NIKONHANDLER_REQ_TMOUT              3*8 // 3ms @125us

#define NIKON_STARTDELAY_FULLCYCLETIME      5000/2    // 125usec @50ns
#define NIKON_STARTDELAY_GUARDTIME          200/2     // 5usec @50ns
#define NIKON_STARTDELAY_STEP               40/2      // 1usec @50ns
#define NIKON_STARTDELAY_TIMEOUT            750/2     // usec @50ns

#define NIKON_TIMEWAIT_CMMMAND              100     // 10usec
#define NIKON_TIMEWAIT_CLEAR                5000    // more than 400usec

//***************************************************************************
// Commands

//***************************************************************************
// Locals

#ifdef _INFINEON_
static void NikonHandlers_Write(NIKON_WORKS huge * psWorks,UWORD uwBlock,UWORD uwData);
static BOOL NikonHandlers_Trigger(NIKON_WORKS huge * psWorks);
static BOOL NikonHandlers_StatusRequest(NIKON_WORKS huge * psWorks,HPUWORD hpuwStatus,HPUWORD hpuwAlarm);
static BOOL NikonHandlers_MemRead(NIKON_WORKS huge * psWorks,UWORD uwMemAddr,HPUWORD hpuwMemData);
static BOOL NikonHandlers_Read(NIKON_WORKS huge * psWorks,UWORD uwRdAddr,HPUWORD hpuwOutData);
#else
static void NikonHandlers_Write(NIKON_WORKS * psWorks,UWORD uwBlock,UWORD uwData);
static BOOL NikonHandlers_Trigger(NIKON_WORKS * psWorks);
static BOOL NikonHandlers_StatusRequest(NIKON_WORKS * psWorks,HPUWORD hpuwStatus,HPUWORD hpuwAlarm);
static BOOL NikonHandlers_MemRead(NIKON_WORKS * psWorks,UWORD uwMemAddr,HPUWORD hpuwMemData);
static BOOL NikonHandlers_Read(NIKON_WORKS * psWorks,UWORD uwRdAddr,HPUWORD hpuwOutData);
#endif

//***************************************************************************
// Interface initialization
#ifdef _INFINEON_
UWORD NikonInitialization(UBYTE ubEncAddr,ULONG ulBaseAddress,NIKON_WORKS huge * psWorks,BOOL bCalcStartDelay)
#else
UWORD NikonInitialization(UBYTE ubEncAddr,ULONG ulBaseAddress,NIKON_WORKS * psWorks,BOOL bCalcStartDelay)
#endif
{
    UWORD uwStatus;
    UWORD uwAlarm;
    UWORD uwStartDelay;
    UWORD uwTimeOut;
    UWORD uwEncType;
    
    // set base address
    psWorks->ulBaseAddress = ulBaseAddress;
	psWorks->ubEncAddr     = ubEncAddr;

    // enter command mode
    NikonEnterCommandMode(psWorks);

    // command: try to set maximum baudrate
    NikonHandlers_Write(psWorks,NIKON_BLOCK_BAUDRATE,NIKON_BAUDRATE_40);
    timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CMMMAND); // wait 10us between two commands
 
    // request: get status of encoder
    if(NikonHandlers_StatusRequest(psWorks,&uwStatus,&uwAlarm))
    {
        // if disable Multi-turn data, ignore battery check
        if(psWorks->ubDisMTData) 
            uwAlarm &= (~NIKON_ALARM_BATT_MASK);
        psWorks->ubBaudrate = NIKON_BAUDRATE_40;
        // detect communication error or encoder alarm, except the MTERR caused by battery removed
        if(uwStatus!=0x0000 || (uwAlarm&~NIKON_ALARM_MTERR_MASK)!=0x0000)
            return NIKON_INIT_ALARM|uwStatus|((uwAlarm&0xFF)<<4); // {CA[3:0],ALM[7:0],SUBCODE_ALARM}
    }
    else
    {
        // if no reply with the maximum baudrate, try default 
        NikonHandlers_Write(psWorks,NIKON_BLOCK_BAUDRATE,NIKON_DEF_BAUDRATE);
        timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CMMMAND); // wait 10us between two commands
       
        // request: get status of encoder
        if(NikonHandlers_StatusRequest(psWorks,&uwStatus,&uwAlarm))
        {
            // if disable Multi-turn data, ignore battery check
            if(psWorks->ubDisMTData) 
                uwAlarm &= (~NIKON_ALARM_BATT_MASK);
            psWorks->ubBaudrate = NIKON_DEF_BAUDRATE;
            // detect communication error or encoder alarm, except the MTERR caused by battery removed
            if(uwStatus!=0x0000 || (uwAlarm&~NIKON_ALARM_MTERR_MASK)!=0x0000)
                return NIKON_INIT_ALARM|uwStatus|((uwAlarm&0xFF)<<4); // {CA[3:0],ALM[7:0],SUBCODE_ALARM}
        }
        else
            return NIKON_INIT_TIMEOUT;
    }
    
    // read type of encoder from EEPROM
    if(!NikonHandlers_MemRead(psWorks,NIKON_MEMADDR_TYPE,&uwEncType))
        return NIKON_INIT_MEMRDFAIL;
        
    // check resolution type
    switch(uwEncType&NIKON_ENCTYPE_RES_MASK)
    {
        case NIKON_ENCTYPE_RES_17:
            psWorks->ubStepPerRevBits = 17;
            break;
        case NIKON_ENCTYPE_RES_20:
            psWorks->ubStepPerRevBits = 20;
            break;
        case NIKON_ENCTYPE_RES_22:
            psWorks->ubStepPerRevBits = 22;
            break;
        case NIKON_ENCTYPE_RES_24:
            psWorks->ubStepPerRevBits = 24;
            break;
        default:
            return NIKON_INIT_TYPEUNKNOW;
    }
    psWorks->ubResolution = (UBYTE)(uwEncType&NIKON_ENCTYPE_RES_MASK)>>NIKON_ENCTYPE_RES_OFF;
    
    // check if encoder is multiturn or single turn
    switch(uwEncType&NIKON_ENCTYPE_MT_MASK)
    {
        case NIKON_ENCTYPE_MT_SG:
            psWorks->ubRevNumBits = 0;
            break;
        case NIKON_ENCTYPE_MT_MAG:
        case NIKON_ENCTYPE_MT_OPT:
            psWorks->ubRevNumBits = NIKON_ENCTYPE_MT_NBITS;
            break;
        default:
            return NIKON_INIT_TYPEUNKNOW;
    }    
    psWorks->ubMultiTurn = (UBYTE)(uwEncType&NIKON_ENCTYPE_MT_MASK)>>NIKON_ENCTYPE_MT_OFF;

    // check battery line
    switch(uwEncType&NIKON_ENCTYPE_BAT_MASK)
    {
        case NIKON_ENCTYPE_BAT_NONE:
        case NIKON_ENCTYPE_BAT_SEP:
        case NIKON_ENCTYPE_BAT_COM: break;
        default: return NIKON_INIT_TYPEUNKNOW;
    }    
    psWorks->ubBatteryLine = (UBYTE)(uwEncType&NIKON_ENCTYPE_BAT_MASK)>>NIKON_ENCTYPE_BAT_OFF;
    
    // if start delay is not requested
    if(!bCalcStartDelay)
    {
        NikonEnterPositionMode(psWorks);
        return NIKON_INIT_SUCCESS;
    }
        
    // begin calibration of start delay
    FPGA_BASEOFF_16(ulBaseAddress, FPGA_NIKON_PRETRG)=uwStartDelay=NIKON_STARTDELAY_FULLCYCLETIME-NIKON_STARTDELAY_GUARDTIME;
    
    // enter position mode
    NikonEnterPositionMode(psWorks);

    for(;;)
    {
            // cycle transaction
        uwTimeOut=timer_settimeout(uwSysTimers100ns, NIKON_STARTDELAY_TIMEOUT*10);
        while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) && !FPGA_NIKON_STAT_BUSY(FPGA_BASEOFF_16(ulBaseAddress,FPGA_NIKON_STATUS)));
        if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
            return NIKON_INIT_TRGDLYFAIL;

        uwTimeOut=timer_settimeout(uwSysTimers100ns, NIKON_STARTDELAY_TIMEOUT*10);
        while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) &&  FPGA_NIKON_STAT_BUSY(FPGA_BASEOFF_16(ulBaseAddress,FPGA_NIKON_STATUS)));
        if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
            return NIKON_INIT_TRGDLYFAIL;

            // check pre-trigger overtime fault
        if(!FPGA_NIKON_STAT_TRGFAULT(FPGA_BASEOFF_16(ulBaseAddress,FPGA_NIKON_STATUS)))
            break;

            // delay must be modified, check if possible, otherwise is not possible to
            // have position each cycle due to lower clock and higher transfer bit number
        if(uwStartDelay<=NIKON_STARTDELAY_GUARDTIME+NIKON_STARTDELAY_STEP)
            return NIKON_INIT_TRGDLYFAIL;

        uwStartDelay-=NIKON_STARTDELAY_STEP;
        FPGA_BASEOFF_16(ulBaseAddress, FPGA_NIKON_PRETRG)=uwStartDelay;
    }

    // apply guard time
    uwStartDelay-=NIKON_STARTDELAY_GUARDTIME;
    FPGA_BASEOFF_16(ulBaseAddress, FPGA_NIKON_PRETRG)=uwStartDelay;

    // wait for clean transaction
    uwTimeOut=timer_settimeout(uwSysTimers100ns, NIKON_STARTDELAY_TIMEOUT*10);
    while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) && !FPGA_NIKON_STAT_BUSY(FPGA_BASEOFF_16(ulBaseAddress,FPGA_NIKON_STATUS)));
    if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
        return NIKON_INIT_TIMEOUT;

    uwTimeOut=timer_settimeout(uwSysTimers100ns, NIKON_STARTDELAY_TIMEOUT*10);
    while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) &&  FPGA_NIKON_STAT_BUSY(FPGA_BASEOFF_16(ulBaseAddress,FPGA_NIKON_STATUS)));
    if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
        return NIKON_INIT_TIMEOUT;
        
    return NIKON_INIT_SUCCESS;
}

//***************************************************************************
// Enter Command Mode
#ifdef _INFINEON_
void NikonEnterCommandMode(NIKON_WORKS huge * psWorks)
#else
void NikonEnterCommandMode(NIKON_WORKS * psWorks)
#endif
{
    // set cmdmode bit to 0
    psWorks->ubCmdMode = 0;

    // command: set trigger source to micro
    NikonHandlers_Write(psWorks,NIKON_BLOCK_TRIGSET,NIKON_TRIGSET_MICRO); 
    
    timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CMMMAND); // wait 10us between two commands
}

//***************************************************************************
// Enter Position Mode, automatically trigger by external signal
#ifdef _INFINEON_
void NikonEnterPositionMode(NIKON_WORKS huge * psWorks)
#else
void NikonEnterPositionMode(NIKON_WORKS * psWorks)
#endif
{    
    // set cmdmode bit to 1
    psWorks->ubCmdMode = 1;

    // set command to full absolute data request 
    NikonHandlers_Write(psWorks,NIKON_BLOCK_CMDCODE,NIKON_CMDCODE_CDF0+(psWorks->ubEncAddr));
    
    timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CMMMAND); // wait 10us between two commands
    
    // set trigger source to micro
    NikonHandlers_Write(psWorks,NIKON_BLOCK_TRIGSET,NIKON_TRIGSET_EXT);
}

//***************************************************************************
// Encoder Status Flag Clear
#ifdef _INFINEON_
BOOL NikonHandlers_StatusFlagClear(NIKON_WORKS huge * psWorks)
#else
BOOL NikonHandlers_StatusFlagClear(NIKON_WORKS * psWorks)
#endif
{
    UBYTE ubClearCnt;
    // send command 8 times to clear the status flag
    for(ubClearCnt=0;ubClearCnt<8;ubClearCnt++)
    {
        // send data request command
        NikonHandlers_Write(psWorks,NIKON_BLOCK_CMDCODE,NIKON_CMDCODE_CDF8+(psWorks->ubEncAddr));
        if(!NikonHandlers_Trigger(psWorks))
            return FALSE;
        
        timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CMMMAND); // wait 10us between two commands
    }
    timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CLEAR); // wait 400us after clear
    
    return TRUE;
}

//***************************************************************************
// Encoder Status and Multiple Rotation Data Clear
#ifdef _INFINEON_
BOOL NikonHandlers_SMClear(NIKON_WORKS huge * psWorks)
#else
BOOL NikonHandlers_SMClear(NIKON_WORKS * psWorks)
#endif
{
    UBYTE ubClearCnt;
    
    // send command 8 times to clear the status flag and multi-turn data
    for(ubClearCnt=0;ubClearCnt<8;ubClearCnt++)
    {
        // send data request command
        NikonHandlers_Write(psWorks,NIKON_BLOCK_CMDCODE,NIKON_CMDCODE_CDF10+(psWorks->ubEncAddr));
        if(!NikonHandlers_Trigger(psWorks))
            return FALSE;
        
        timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CMMMAND); // wait 10us between two commands
    }
    timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CLEAR); // wait 400us after clear
    
    return TRUE;
}

//***************************************************************************
// Encoder Multiple Rotation Data Clear
#ifdef _INFINEON_
BOOL NikonHandlers_MTClear(NIKON_WORKS huge * psWorks)
#else
BOOL NikonHandlers_MTClear(NIKON_WORKS * psWorks)
#endif
{
    UBYTE ubClearCnt;
    
    // send command 8 times to clear multi-turn data
    for(ubClearCnt=0;ubClearCnt<8;ubClearCnt++)
    {
        // send data request command
        NikonHandlers_Write(psWorks,NIKON_BLOCK_CMDCODE,NIKON_CMDCODE_CDF9+(psWorks->ubEncAddr));
        if(!NikonHandlers_Trigger(psWorks))
            return FALSE;
        
        timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CMMMAND); // wait 10us between two commands
    }
    timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CLEAR); // wait 400us after clear
    
    return TRUE;
}

//***************************************************************************
// Encoder Single Turn Zero Position Set

BOOL NikonHandlers_ZeroPosSet(NIKON_WORKS * psWorks)
{
    UBYTE ubClearCnt;
    
    // send command 8 times to clear the status flag
    for(ubClearCnt=0;ubClearCnt<8;ubClearCnt++)
    {
        // send data request command
        NikonHandlers_Write(psWorks,NIKON_BLOCK_CMDCODE,NIKON_CMDCODE_CDF12+(psWorks->ubEncAddr));
        if(!NikonHandlers_Trigger(psWorks))
            return FALSE;
        
        timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CMMMAND); // wait 10us between two commands
    }
    timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CLEAR); // wait 400us after clear
    return TRUE;
}


//***************************************************************************
// Encoder Status Request
#ifdef _INFINEON_
static BOOL NikonHandlers_StatusRequest(NIKON_WORKS huge * psWorks,HPUWORD hpuwStatus,HPUWORD hpuwAlarm)
#else
static BOOL NikonHandlers_StatusRequest(NIKON_WORKS * psWorks,HPUWORD hpuwStatus,HPUWORD hpuwAlarm)
#endif
{
    BOOL  bRetVal;
    UWORD uwStatus=0x0000;
    UWORD uwAlarm=0x0000;
            
    // send data request command
    NikonHandlers_Write(psWorks,NIKON_BLOCK_CMDCODE,NIKON_CMDCODE_CDF3+(psWorks->ubEncAddr));
    
    // trigger command
    bRetVal = NikonHandlers_Trigger(psWorks);
        
    // read output register
    if(bRetVal)
    {
        if(NikonHandlers_Read(psWorks,NIKON_RDADDR_STATUS,&uwStatus)) //{CA[3:0],ES[3:0],CMDCODE[4:0],ECADDR[2:0]}
        {
            if((uwStatus&0x00FF)==NIKON_CMDCODE_CDF3+(psWorks->ubEncAddr))
            {
                *hpuwAlarm  = uwAlarm;
                if((uwStatus&0x0F00)!=0x0000) // when encoder alarm ES[3:0], read alarm register
                    if(NikonHandlers_Read(psWorks,NIKON_RDADDR_ALARM,&uwAlarm)) // ALM[15:0]
                        *hpuwAlarm  = uwAlarm; // return alarm:  {6'b000000,OVTEMP,MEMBUSY,BUSY,PSERR,STERR,MEMERR,OVSPD,1'b0,MTERR,BATT}
                *hpuwStatus = uwStatus&0xF000; // return status: {CA[3:0],12'h000}
                return TRUE;
            }
        }
    }
    
    return FALSE;
}

//***************************************************************************
// Read Memory Data
#ifdef _INFINEON_
static BOOL NikonHandlers_MemRead(NIKON_WORKS huge * psWorks,UWORD uwMemAddr,HPUWORD hpuwMemData)
#else
static BOOL NikonHandlers_MemRead(NIKON_WORKS * psWorks,UWORD uwMemAddr,HPUWORD hpuwMemData)
#endif
{
    BOOL bRetVal;
        
    NikonHandlers_Write(psWorks,NIKON_BLOCK_ROMADDR,uwMemAddr); // set memory address
    
    timer_wait(uwSysTimers100ns,NIKON_TIMEWAIT_CMMMAND); // wait 10us between two commands
    
    NikonHandlers_Write(psWorks,NIKON_BLOCK_CMDCODE,NIKON_CMDCODE_CDF13+(psWorks->ubEncAddr)); // send data request command
    
    bRetVal = NikonHandlers_Trigger(psWorks); // trigger reading
    if(!bRetVal)
        return FALSE;
    
    bRetVal = NikonHandlers_Read(psWorks,NIKON_RDADDR_MEMDATA,hpuwMemData);
    if(!bRetVal)
        return FALSE;
    
    return TRUE;
}

//***************************************************************************
// Write data to the specific block selection
#ifdef _INFINEON_
static void NikonHandlers_Write(NIKON_WORKS huge * psWorks,UWORD uwBlock,UWORD uwData)
#else
static void NikonHandlers_Write(NIKON_WORKS * psWorks,UWORD uwBlock,UWORD uwData)
#endif
{
    // Write Block Selection & Data    
    FPGA_BASEOFF_16(psWorks->ulBaseAddress,FPGA_NIKON_DATA) = uwData;
    FPGA_BASEOFF_16(psWorks->ulBaseAddress,FPGA_NIKON_SET)  = NIKONHANDLER_SETRG(psWorks->ubResolution,psWorks->ubCmdMode,uwBlock);
    FPGA_NIKON_SET_WR(psWorks->ulBaseAddress) = TRUE;
}

//***************************************************************************
// Micro Trigger Input
#ifdef _INFINEON_
static BOOL NikonHandlers_Trigger(NIKON_WORKS huge * psWorks)
#else
static BOOL NikonHandlers_Trigger(NIKON_WORKS * psWorks)
#endif
{
    BOOL  bRetVal;
    UWORD uwTOut;
    ULONG ulBaseAddress;
    UWORD uwStatus;
    
    ulBaseAddress = psWorks->ulBaseAddress;
    
    // set trigger input setting register
    NikonHandlers_Write(psWorks,NIKON_BLOCK_TRIG,NIKON_TRIG_EN);

    // wait for transmission start
    uwTOut = timer_settimeout(uwSysTimers125us, NIKONHANDLER_REQ_TMOUT);
    do{uwStatus=FPGA_BASEOFF_16(ulBaseAddress,FPGA_NIKON_STATUS);}
    while( !timer_istimedout(uwSysTimers125us, uwTOut) && !(bRetVal=(!FPGA_NIKON_STAT_TMOUT(uwStatus))&&FPGA_NIKON_STAT_BUSY(uwStatus)&&FPGA_NIKON_STAT_EOF(uwStatus)));
    if(!bRetVal)
        return FALSE; // when timeout, return FALSE
        
    // transmission done
    uwTOut = timer_settimeout(uwSysTimers125us, NIKONHANDLER_REQ_TMOUT);
    do{uwStatus=FPGA_BASEOFF_16(ulBaseAddress,FPGA_NIKON_STATUS);}
    while(!timer_istimedout(uwSysTimers125us, uwTOut) && !(bRetVal=(!FPGA_NIKON_STAT_TMOUT(uwStatus))&&(!FPGA_NIKON_STAT_BUSY(uwStatus))&&(!FPGA_NIKON_STAT_EOF(uwStatus))));
    if(!bRetVal)
        return FALSE; // when timeout, return FALSE

    return TRUE; 
}

//***************************************************************************
// Read Output Register
#ifdef _INFINEON_
static BOOL NikonHandlers_Read(NIKON_WORKS huge * psWorks,UWORD uwRdAddr,HPUWORD hpuwOutData)
#else
static BOOL NikonHandlers_Read(NIKON_WORKS * psWorks,UWORD uwRdAddr,HPUWORD hpuwOutData)
#endif
{
    UWORD uwTOut;
    BOOL  bRetVal;
    ULONG ulBaseAddress;
    UWORD uwStatus;
    
    ulBaseAddress = psWorks->ulBaseAddress;

    FPGA_BASEOFF_16(ulBaseAddress,FPGA_NIKON_SET) = NIKONHANDLER_SETRG(psWorks->ubResolution,psWorks->ubCmdMode,uwRdAddr);
    FPGA_NIKON_SET_RD(ulBaseAddress) = TRUE;
    uwTOut=timer_settimeout(uwSysTimers125us, NIKONHANDLER_REQ_TMOUT); // set timeout parameter
    do{uwStatus=FPGA_BASEOFF_16(ulBaseAddress,FPGA_NIKON_STATUS);}
    while(!timer_istimedout(uwSysTimers125us, uwTOut) && !(bRetVal=(!FPGA_NIKON_STAT_BUSY(uwStatus))&&(!FPGA_NIKON_STAT_TMOUT(uwStatus))&&FPGA_NIKON_STAT_EOF(uwStatus)));
    if(bRetVal)
    {
        *hpuwOutData = FPGA_BASEOFF_16(ulBaseAddress,FPGA_NIKON_DATA);        
        return TRUE;
    }
    
    return FALSE;
}
#endif
