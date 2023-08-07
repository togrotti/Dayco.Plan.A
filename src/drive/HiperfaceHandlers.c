/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2014, Phase Motion Control. All Rights Reserved.             */
/*                                                                          */
/* File        : HiperfaceHandlers.c                                        */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Low level HIPERFACE management functions                   */
/*                                                                          */
/****************************************************************************/

#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "common\MathFunctions.h"
#include "drive\HiperfaceHandlers.h"
#include "fpga\FpgaHandler.h"
#include <stdlib.h>

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

#if CFG_ENC_HIP
//***************************************************************************
// Defines

#define HIPFC_CLOCK_BASE_FREQUENCY                      156250  // Hz

#define HIPFC_POWERUP_TIMEOUT                           2000    // msec
#define HIPFC_POWERUP_TEST                              250     // msec

#define HIPFC_CMDTIMEOUT_QUERY                          25      // msec
#define HIPFC_CMDTIMEOUT_SET                            90      // msec
#define HIPFC_CMDTIMEOUT_STORE                          250     // msec

#define HIPFC_COMMAND_GAP                               6       // msec

#define HIPFC_GETRXCNT(x)                               ((UBYTE)(FPGA_BASEOFF_16((x)->ulBaseAddress, FPGA_HIPFC_STATUS)&0x00FF))

#define HIPFC_DEF_ADDRESS                               0xFF
#define HIPFC_DEF_CODE0                                 0x55

#define HIPFC_SUPPENC_SRS                               0x22    // single turn SRS series
#define HIPFC_SUPPENC_SRM                               0x27    // multi turn SRM series
#define HIPFC_SUPPENC_SEK                               0x42    // single turn SEK series

#define HIPFC_SRM_MULTITURN_REVS                        4096
#define HIPFC_SRx_BITS_REV                              15
#define HIPFC_SEK_BITS_REV                              9

#define HIPFC_EXTYPE_ABSRES                             32
#define HIPFC_EXTYPE_MAXPERIODS                         (UWORD)(65536/HIPFC_EXTYPE_ABSRES)
#define HIPFC_EXTYPE_MAXMULTITURN                       32767

//***************************************************************************
// Commands

#define HIPFC_COMMAND_MASK                              0x7F

#define HIPFC_COMMAND_READPOSITION                      0x42
#define HIPFC_COMMAND_SETPOSITION                       0x43
#define HIPFC_COMMAND_READANALOGVALUE                   0x44
#define HIPFC_COMMAND_READDATA                          0x4A
#define HIPFC_COMMAND_STOREDATA                         0x4B
#define HIPFC_COMMAND_READSTATUS                        0x50
#define HIPFC_COMMAND_READTYPELABEL                     0x52
#define HIPFC_COMMAND_RESET                             0x53

//***************************************************************************
// Locals

#ifdef _INFINEON_
static UWORD _Transaction(HIPFC_WORKS huge * psWorks, UBYTE * pubCmd, UBYTE ubCmdSz, UBYTE * pubRet, UBYTE * pubRetSz, UWORD uwTimeout);
static UWORD _CmdReset(HIPFC_WORKS huge * psWorks);
#else
static UWORD _Transaction(HIPFC_WORKS * psWorks, UBYTE * pubCmd, UBYTE ubCmdSz, UBYTE * pubRet, UBYTE * pubRetSz, UWORD uwTimeout);
static UWORD _CmdReset(HIPFC_WORKS * psWorks);
#endif

//***************************************************************************
// Interface initialization

#ifdef _INFINEON_
UWORD HipfcInitialization(ULONG ulBaseAddress, UWORD uwBaudRate, UBYTE ubParity, HIPFC_WORKS huge * psWorks)
#else
UWORD HipfcInitialization(ULONG ulBaseAddress, UWORD uwBaudRate, UBYTE ubParity, HIPFC_WORKS * psWorks)
#endif
{
    union {
        HIPFC_TYPELABEL sTypeLabel;
        HIPFC_EXTTYPELABEL sExtTypeLabel;
    } tl;
    UWORD uwTmp,uwPTest;

        // calculate baudrate
    uwTmp=(UWORD)(HIPFC_CLOCK_BASE_FREQUENCY/uwBaudRate);
    if(uwTmp<4||uwTmp>256)
        return 1;

        // set baudrate and reset parity setting
    psWorks->uwRGSet=uwTmp-1;

        // setup parity
    switch(ubParity)
    {
        case HIPFC_PARITY_NONE:
            uwTmp=10;
            break;

        case HIPFC_PARITY_EVEN:
            FPGA_HIPFC_SET_PARITY_EN(psWorks->uwRGSet)=TRUE;
            uwTmp=11;
            break;

        case HIPFC_PARITY_ODD:
            FPGA_HIPFC_SET_PARITY_EN(psWorks->uwRGSet)=TRUE;
            FPGA_HIPFC_SET_PARITY_ODD(psWorks->uwRGSet)=TRUE;
            uwTmp=11;
            break;

        default:
            return 2;
    }

        // write config register
    FPGA_BASEOFF_16(ulBaseAddress, FPGA_HIPFC_SET)=psWorks->uwRGSet;

        // setup works data structure
    psWorks->ulBaseAddress=ulBaseAddress;
    psWorks->uwCharTime=(UWORD)((ULONG)uwTmp*1000000/uwBaudRate);

        // send reset
    uwTmp=_CmdReset(psWorks);
    if(uwTmp!=HIPFC_SUCCESS)
        return uwTmp;

        // wait for reset, try to interrogate
        // encoder until successful
    uwPTest=HIPFC_POWERUP_TIMEOUT/HIPFC_POWERUP_TEST;
    while(uwPTest>0)
    {
            // wait test time
        timer_wait(uwSysTimers1ms, HIPFC_POWERUP_TEST);
    
            // get type label
        uwTmp=HipfcReadTypeLabel(psWorks, &tl.sTypeLabel);
        if(uwTmp==HIPFC_SUCCESS)
            break;

            // if error but timeout exit
        if(uwTmp!=HIPFC_TRAN_TIMEOUT)
            return uwTmp;

        uwPTest--;
    }

        // if error then exit
    if(uwTmp!=HIPFC_SUCCESS)
        return uwTmp;

        // verify if supported
    switch(tl.sTypeLabel.ubEncoderType)
    {
        case HIPFC_SUPPENC_SRS:
            psWorks->uwRevolutionNumber=0;
            psWorks->sbPosShift=16-HIPFC_SRx_BITS_REV;
            break;

        case HIPFC_SUPPENC_SRM:
            psWorks->uwRevolutionNumber=HIPFC_SRM_MULTITURN_REVS;
            psWorks->sbPosShift=16-HIPFC_SRx_BITS_REV;
            break;

        case HIPFC_SUPPENC_SEK:
            psWorks->uwRevolutionNumber=0;
            psWorks->sbPosShift=16-HIPFC_SEK_BITS_REV;
            break;

        default:
            uwTmp=HipfcReadExtTypeLabel(psWorks, &tl.sExtTypeLabel);
            if(uwTmp!=HIPFC_SUCCESS)
                return uwTmp;

                // extended type support only for single-turn absolute
            if(tl.sExtTypeLabel.ulRange>HIPFC_EXTYPE_MAXMULTITURN)
                return HIPFC_INIT_UNSUPPORTED;

                // max sin/cos period per revolution
            if(tl.sExtTypeLabel.ulResolution>HIPFC_EXTYPE_MAXPERIODS)
                return HIPFC_INIT_UNSUPPORTED;

                // calculate parameters
            psWorks->sbPosShift=(SBYTE)(16-(_ilog2(tl.sExtTypeLabel.ulResolution*HIPFC_EXTYPE_ABSRES)-1));
            if(tl.sExtTypeLabel.ulRange==1)
                psWorks->uwRevolutionNumber=0;
            else
                psWorks->uwRevolutionNumber=(UWORD)tl.sExtTypeLabel.ulRange;
            break;
    }

    return HIPFC_SUCCESS;
}

//***************************************************************************
// Transaction processing

#ifdef _INFINEON_
static UWORD _Transaction(HIPFC_WORKS huge * psWorks, UBYTE * pubCmd, UBYTE ubCmdSz, UBYTE * pubRet, UBYTE * pubRetSz, UWORD uwTimeout)
#else
static UWORD _Transaction(HIPFC_WORKS * psWorks, UBYTE * pubCmd, UBYTE ubCmdSz, UBYTE * pubRet, UBYTE * pubRetSz, UWORD uwTimeout)
#endif
{
    UWORD uwTOut,uwSet=psWorks->uwRGSet;
    UBYTE ubCnt,ubRxTot,ubChk,ubOutBufSz=*pubRetSz,ubVal;

        // wait for transaction gap
    uwTOut=timer_settimeout(uwSysTimers1ms, HIPFC_COMMAND_GAP);
    while(!timer_istimedout(uwSysTimers1ms, uwTOut));
        //$TEST$ os_sleep? wdt_clear?

        // default output is zero bytes
    *pubRetSz=0;

        // transfer command to output buffer while calculating checksum
    for(ubCnt=0,ubChk=0x00;ubCnt<ubCmdSz;ubCnt++)
    {
        FPGA_BASEOFF_16(psWorks->ulBaseAddress, FPGA_HIPFC_DATA)=*pubCmd;
        ubChk^=*pubCmd++;
    }
    FPGA_BASEOFF_16(psWorks->ulBaseAddress, FPGA_HIPFC_DATA)=ubChk;

        // recalc timeout adding output bytes timing
    uwTimeout+=(UWORD)((ULONG)psWorks->uwCharTime*(ubCmdSz+1)/1000)+1;

        // enable transaction
    FPGA_HIPFC_SET_EXECUTE(uwSet)=TRUE;
    FPGA_BASEOFF_16(psWorks->ulBaseAddress, FPGA_HIPFC_SET)=uwSet;

        // wait first byte in
    uwTOut=timer_settimeout(uwSysTimers1ms, uwTimeout);
    while(!timer_istimedout(uwSysTimers1ms, uwTOut) && HIPFC_GETRXCNT(psWorks)==0);
        //$TEST$ os_sleep? wdt_clear?

        // if still zero
    if(HIPFC_GETRXCNT(psWorks)==0)
    {
        FPGA_BASEOFF_16(psWorks->ulBaseAddress, FPGA_HIPFC_SET)=0;
        return HIPFC_TRAN_TIMEOUT;
    }
          
        // recalc timeout for single byte * 2
    uwTimeout=(UWORD)((ULONG)psWorks->uwCharTime*2/1000)+1;

        // wait until no more bytes in
    ubRxTot=0;
    while(HIPFC_GETRXCNT(psWorks)!=ubRxTot)
    {
        ubRxTot=HIPFC_GETRXCNT(psWorks);

        uwTOut=timer_settimeout(uwSysTimers1ms, uwTimeout);
        while(!timer_istimedout(uwSysTimers1ms, uwTOut) && HIPFC_GETRXCNT(psWorks)==ubRxTot);
            //$TEST$ os_sleep? wdt_clear?
    }

        // returned length without checksum byte
    ubRxTot--;

        // if parity error detected
    if(FPGA_HIPFC_STAT_PARITYERROR(psWorks->ulBaseAddress))
    {
        FPGA_BASEOFF_16(psWorks->ulBaseAddress, FPGA_HIPFC_SET)=0;
        return HIPFC_TRAN_PARITY_ERROR;
    }

        // retrieve bytes in while calculating checksum
    for(ubCnt=0,ubChk=0x00;ubCnt<ubRxTot;ubCnt++)
    {
        ubVal=(UBYTE)FPGA_BASEOFF_16(psWorks->ulBaseAddress, FPGA_HIPFC_DATA);
        ubChk^=ubVal;
        if(ubCnt<ubOutBufSz)
            pubRet[ubCnt]=ubVal;
    }
        // last is checksum
    ubChk^=(UBYTE)FPGA_BASEOFF_16(psWorks->ulBaseAddress, FPGA_HIPFC_DATA);

        // disable transaction
    FPGA_BASEOFF_16(psWorks->ulBaseAddress, FPGA_HIPFC_SET)=0;

        // if not zero then checksum error
    if(ubChk)
        return HIPFC_TRAN_CHECKSUM_ERROR;

        // if return is encoder status
    if((pubRet[1]&HIPFC_COMMAND_MASK)==HIPFC_COMMAND_READSTATUS)
    {
            // data size checking
        if(ubRxTot!=3)
            return HIPFC_CMD_RXUNEXPECTED;

            // return status message
        return HIPFC_STAT_MASK|pubRet[2];
    }

        // check for error flag
    if(pubRet[1]&0x80)
        return HIPFC_CMD_ERRORFLAG;

        // set output bytes and return
    if(ubOutBufSz>ubRxTot)
        *pubRetSz=ubRxTot;
    else
        *pubRetSz=ubOutBufSz;
    return HIPFC_SUCCESS;
}

//***************************************************************************
// 42h: Read position

#ifdef _INFINEON_
UWORD HipfcReadPosition(HIPFC_WORKS huge * psWorks, UBYTE * pubDest)
#else
UWORD HipfcReadPosition(HIPFC_WORKS * psWorks, UBYTE * pubDest)
#endif
{
    UBYTE ubCmd[]={HIPFC_DEF_ADDRESS,HIPFC_COMMAND_READPOSITION};
    UBYTE ubRet[7];
    UBYTE ubRetSz=sizeof(ubRet);
    UWORD uwRetVal;

    uwRetVal=_Transaction(psWorks, ubCmd, sizeof(ubCmd), ubRet, &ubRetSz, HIPFC_CMDTIMEOUT_QUERY);
    if(uwRetVal!=HIPFC_SUCCESS)
        return uwRetVal;

        // validate command
    if(ubRet[1]!=HIPFC_COMMAND_READPOSITION || ubRetSz!=6)
        return HIPFC_CMD_RXUNEXPECTED;

        // read out and realignment of HH,HL,LH,LL
    pubDest[3]=ubRet[2];
    pubDest[2]=ubRet[3];
    pubDest[1]=ubRet[4];
    pubDest[0]=ubRet[5];

    return HIPFC_SUCCESS;
}

//***************************************************************************
// 43h: Set position

#ifdef _INFINEON_
UWORD HipfcSetPosition(HIPFC_WORKS huge * psWorks, UBYTE * pubSrc)
#else
UWORD HipfcSetPosition(HIPFC_WORKS * psWorks, UBYTE * pubSrc)
#endif
{
    UBYTE ubCmd[]={HIPFC_DEF_ADDRESS,HIPFC_COMMAND_SETPOSITION,0,0,0,0,HIPFC_DEF_CODE0};
    UBYTE ubRet[3];
    UBYTE ubRetSz=sizeof(ubRet);
    UWORD uwRetVal;

        // align HH,HL,LH,LL to command
    ubCmd[2]=pubSrc[3];
    ubCmd[3]=pubSrc[2];
    ubCmd[4]=pubSrc[1];
    ubCmd[5]=pubSrc[0];

    uwRetVal=_Transaction(psWorks, ubCmd, sizeof(ubCmd), ubRet, &ubRetSz, HIPFC_CMDTIMEOUT_SET);
    if(uwRetVal!=HIPFC_SUCCESS)
        return uwRetVal;

        // validate command
    if(ubRet[1]!=HIPFC_COMMAND_SETPOSITION || ubRetSz!=2)
        return HIPFC_CMD_RXUNEXPECTED;

    return HIPFC_SUCCESS;
}

//***************************************************************************
// 52h: Read out type label

#ifdef _INFINEON_
UWORD HipfcReadTypeLabel(HIPFC_WORKS huge * psWorks, HIPFC_TYPELABEL * psTypeLabel)
#else
UWORD HipfcReadTypeLabel(HIPFC_WORKS * psWorks, HIPFC_TYPELABEL * psTypeLabel)
#endif
{
    UBYTE ubCmd[]={HIPFC_DEF_ADDRESS,HIPFC_COMMAND_READTYPELABEL};
    UBYTE ubRet[6];
    UBYTE ubRetSz=sizeof(ubRet);
    UWORD uwRetVal;

    uwRetVal=_Transaction(psWorks, ubCmd, sizeof(ubCmd), ubRet, &ubRetSz, HIPFC_CMDTIMEOUT_QUERY);
    if(uwRetVal!=HIPFC_SUCCESS)
        return uwRetVal;

        // validate command
    if(ubRet[1]!=HIPFC_COMMAND_READTYPELABEL || ubRetSz!=6)
        return HIPFC_CMD_RXUNEXPECTED;

        // fill-up output data
    psTypeLabel->ubRS485Settings=ubRet[2];
    psTypeLabel->ubEncoderType=ubRet[3];
    psTypeLabel->ubEEPROMSize=ubRet[4];
    psTypeLabel->ubOptions=ubRet[5];

    return HIPFC_SUCCESS;
}

//***************************************************************************
// 53h: Encoder reset

#ifdef _INFINEON_
static UWORD _CmdReset(HIPFC_WORKS huge * psWorks)
#else
static UWORD _CmdReset(HIPFC_WORKS * psWorks)
#endif
{
    UBYTE ubCmd[]={HIPFC_DEF_ADDRESS,HIPFC_COMMAND_RESET};
    UBYTE ubRet[3];
    UBYTE ubRetSz=sizeof(ubRet);
    UWORD uwRetVal;

    uwRetVal=_Transaction(psWorks, ubCmd, sizeof(ubCmd), ubRet, &ubRetSz, HIPFC_CMDTIMEOUT_QUERY);

        // expected return is timeout
    if(uwRetVal==HIPFC_TRAN_TIMEOUT)
        return HIPFC_SUCCESS;

    return uwRetVal;
}

//***************************************************************************
// 4Ah/FFh: Read extended type label specification

#ifdef _INFINEON_
UWORD HipfcReadExtTypeLabel(HIPFC_WORKS huge * psWorks, HIPFC_EXTTYPELABEL * psExtTypeLabel)
#else
UWORD HipfcReadExtTypeLabel(HIPFC_WORKS * psWorks, HIPFC_EXTTYPELABEL * psExtTypeLabel)
#endif
{
    UBYTE ubRet[10+5];
    UBYTE ubCmd[]={HIPFC_DEF_ADDRESS,HIPFC_COMMAND_READDATA,0xFF,0,10,0x55};
    UBYTE ubRetSz=sizeof(ubRet);
    UWORD uwRetVal;

    uwRetVal=_Transaction(psWorks, ubCmd, sizeof(ubCmd), ubRet, &ubRetSz, HIPFC_CMDTIMEOUT_QUERY);
    if(uwRetVal!=HIPFC_SUCCESS)
        return uwRetVal;

        // validate command
    if(ubRet[1]!=HIPFC_COMMAND_READDATA || ubRet[2]!=ubCmd[2] || ubRet[3]!=ubCmd[3] ||
        ubRet[4]!=ubCmd[4] || ubRetSz!=sizeof(ubRet))
        return HIPFC_CMD_RXUNEXPECTED;

        // fill-up output data
    psExtTypeLabel->ubType=ubRet[6];
    psExtTypeLabel->ulResolution=((ULONG)ubRet[7]<<24)|((ULONG)ubRet[8]<<16)|((ULONG)ubRet[9]<<8)|(ULONG)ubRet[10];
    psExtTypeLabel->ulRange=((ULONG)ubRet[11]<<24)|((ULONG)ubRet[12]<<16)|((ULONG)ubRet[13]<<8)|(ULONG)ubRet[14];

    return HIPFC_SUCCESS;
}
#endif
