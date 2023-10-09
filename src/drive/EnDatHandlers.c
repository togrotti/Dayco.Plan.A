/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EndatHandlers.c                                            */
/* Author      : Stefano Martino                                            */
/*               Fabio Terrile                                              */
/*                                                                          */
/* Description : Low level ENDAT 2.1/2.2 management functions               */
/*                                                                          */
/****************************************************************************/

#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "drive\EndatHandlers.h"
#include "fpga\FpgaHandler.h"
#include <stdlib.h>

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
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

#if CFG_ENC_ENDAT
//***************************************************************************
// Defines

#define ENDAT_CLOCK_BASE_FREQUENCY                      160000  // khz

#define ENDAT_CLOCK_MIN_FREQUENCY                       100     // khz
#define ENDAT_CLOCK_MAX_21_FREQUENCY                    2000    // khz
#define ENDAT_CLOCK_MAX_22_FREQUENCY                    8000    // khz

#define ENDAT_CLOCK_CMD_MODE_FREQUENCY                  500     // khz

#define ENDAT_POWERUP_TIMEOUT                           1000    // msec
#define ENDAT_POWERUP_GAP                               50      // msec

#define ENDAT_COMMANDMODE_SET2BUSYGAP                   5       // usec
#define ENDAT_COMMANDMODE_TIMEOUT                       100     // msec
#define ENDAT_COMMANDMODE_RECOVERYTIME                  30      // usec

#define ENDAT_NBIT_MCT                                  6
#define ENDAT_NBIT_TPAR                                 (16+8)
#define ENDAT_NBIT_ADINFO                               (8+8+8+5+1)

#define ENDAT_PROPCOMP_CALC_NUM_SAMPLES                 8
#define ENDAT_PROPCOMP_CALC_TOLERANCE                   10      // %

#define ENDAT_STARTDELAY_FULLCYCLETIME                  5000    // 125usec
#define ENDAT_STARTDELAY_GUARDTIME                      200     // 5usec
#define ENDAT_STARTDELAY_STEP                           40      // 1usec
#define ENDAT_STARTDELAY_TIMEOUT                        750     // usec

//***************************************************************************
// Command and register access

#define ENDAT_CMDM_MODE                                 0x40
#define ENDAT_CMDM_EXT                                  0x80

#define ENDAT_COMMAND_ENCODER_TO_SEND_POSITION_VALUES   0x07
#define ENDAT_COMMAND_SELECTION_OF_MEMORY_AREA          0x0e
#define ENDAT_COMMAND_ENCODER_TO_RECEIVE_PARAMETERS     0x1c
#define ENDAT_COMMAND_ENCODER_TO_SEND_PARAMETERS        0x23
#define ENDAT_COMMAND_ENCODER_TO_RECEIVE_RESET          0x2a   

#define ENDAT_CMD22_ENCODER_TO_SEND_POSITION_VALUES     (ENDAT_CMDM_EXT|ENDAT_CMDM_MODE|0x38)
#define ENDAT_CMD22_SELECTION_OF_MEMORY_AREA            (ENDAT_CMDM_EXT|0x09)
#define ENDAT_CMD22_ENCODER_TO_SEND_PARAMETERS          (ENDAT_CMDM_EXT|0x24)
#define ENDAT_CMD22_ENCODER_TO_RECEIVE_RESET            (ENDAT_CMDM_EXT|0x2d)

#define ENDAT_MEMAREA_OPERATING_CONDITION               0xb9
#define ENDAT_MEMAREA_MANUFACTURER_PARAM_A1             0xa1
#define ENDAT_MEMAREA_MANUFACTURER_PARAM_A3             0xa3
#define ENDAT_MEMAREA_MANUFACTURER_PARAM_A5             0xa5
#define ENDAT_MEMAREA_OPERATING_PARAMETERS              0xa7
#define ENDAT_MEMAREA_MANUFACTURER_PARAM22              0xbd

#define ADINFO_SEL(x)                                   ((x)|0x40)
#define ADINFO_ACK_WRN(x)                               ((x)&0x80)
#define ADINFO_ACK_BUSY(x)                              ((x)&0x20)
#define ADINFO_ACK_SEL(x)                               ((x)&0x1F)

#define ENDAT_MEMAREA_OEM_PARAM_AREA_1                  0xa9
#define ENDAT_MEMAREA_OEM_PARAM_AREA_2                  0xab
#define ENDAT_MEMAREA_OEM_PARAM_AREA_3                  0xad

#define ENDAT_ADDRESS_OPERATING_STATUS_ERRORS           0x00
#define ENDAT_ADDRESS_OPERATING_STATUS_WARNINGS         0x01

#define ENDAT_ADDRESS_TRANSFER_FORMAT_FOR_POSITION      0x0d
#define ENDAT_ADDRESS_DISTINGUISHABLE_REVOLUTIONS       0x01
#define ENDAT_ADDRESS_DISTINGUISHABLE_REV_W_SCALING     0x22

#define ENDAT_ADDRESS_MAX_FREQUENCY                     0x19

#define ENDAT_ADDRESS_ENDAT_COMMAND_SET                 0x05

typedef union
{
    ULONG ulData;
    UBYTE ubData[4];
} ADINFO;

//***************************************************************************
// Locals

static ULONG EndatExecuteTransaction(ENDAT_WORKS * psWorks, UBYTE ubCommand, UBYTE ubAddress, UWORD uwParam);
static ULONG EndatEnableADInfo(ENDAT_WORKS * psWorks);

//***************************************************************************
// Power of two checking

static BOOL CheckIfNumberIsPowerOfTwo( ULONG ulValue )
{
    return ( ulValue != 0 ) && ( ( ulValue & ( ulValue - 1 ) ) == 0 );
}

//***************************************************************************
// Enter command mode

BOOL EndatEnterCommandMode(ENDAT_WORKS * psWorks)
{
    ULONG dwBaseAddress=psWorks->dwBaseAddress;

        // aborting processing transaction and position mode
    FPGA_ENDAT_SET_START(dwBaseAddress) = FALSE;
    FPGA_ENDAT_SET_MODE(dwBaseAddress) = FALSE;
    FPGA_ENDAT_SET_EXT(dwBaseAddress) = FALSE;
#ifdef ENDAT22
    FPGA_ENDAT_SET_SYNCTRAN(dwBaseAddress) = FALSE; // crs-endat.22
#endif // endat22

        // disable write protection
    FPGA_ENDAT_SET_WRPROT(dwBaseAddress) = FALSE;

        // adapt to module clock
    timer_wait(uwSysTimers100ns,ENDAT_COMMANDMODE_SET2BUSYGAP*10);

        // wait module ready
    while(FPGA_ENDAT_STAT_BUSY(FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STATUS)));

        // setup for slow transfer command mode
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_CMDADR)    = 0;
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_CLOCKDIV)  = (UWORD)(ENDAT_CLOCK_BASE_FREQUENCY/2/ENDAT_CLOCK_CMD_MODE_FREQUENCY-1);
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_PROPCOMP)  = 0;

        // enable module
    FPGA_ENDAT_SET_ENABLE(dwBaseAddress) = TRUE;

        // recovery time before command mode transaction
    timer_wait(uwSysTimers100ns,ENDAT_COMMANDMODE_RECOVERYTIME*10);

    return TRUE;
}

//***************************************************************************
// Enter position mode (self triggered)

BOOL EndatEnterPositionMode(ENDAT_WORKS * psWorks)
{
    ULONG dwBaseAddress=psWorks->dwBaseAddress;

        // aborting processing transaction and position mode
    FPGA_ENDAT_SET_START(dwBaseAddress) = FALSE;
    FPGA_ENDAT_SET_MODE(dwBaseAddress) = FALSE;
    FPGA_ENDAT_SET_EXT(dwBaseAddress) = FALSE;

        // adapt to module clock
    timer_wait(uwSysTimers100ns,ENDAT_COMMANDMODE_SET2BUSYGAP*10);

        // wait module ready
    while(FPGA_ENDAT_STAT_BUSY(FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STATUS)));

        // setup for fast position transfer, according to calibration parameters
#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
    if(psWorks->ubADInfo == ENDAT_ADINFO1_NOP || !psWorks->flags.b.bEndatEx || !psWorks->flags.b.bProt22) // crs-endat.22
        FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_CMDADR) = ENDAT_COMMAND_ENCODER_TO_SEND_POSITION_VALUES << 8;
    else
    {	// crs-endat.22
        FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_CMDADR) = (ENDAT_CMD22_ENCODER_TO_SEND_POSITION_VALUES & 0x3F) << 8;
        FPGA_ENDAT_SET_EXT(dwBaseAddress) = TRUE;
    }	// crs-endat.22
#else
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_CMDADR) = ENDAT_COMMAND_ENCODER_TO_SEND_POSITION_VALUES << 8;
#endif // endat22 && endat22_addinfo

    FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_CLOCKDIV)  = psWorks->uwClockDiv;
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_PROPCOMP)  = psWorks->uwPropComp;
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_POSFORMAT) = psWorks->uwPosFormat;
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_EARLYSTOP) = psWorks->uwEarlyStop;

        // enable position mode
    FPGA_ENDAT_SET_MODE(dwBaseAddress) = TRUE;
    FPGA_ENDAT_SET_START(dwBaseAddress) = TRUE;
#ifdef ENDAT22	
    FPGA_ENDAT_SET_SYNCTRAN(dwBaseAddress) = TRUE; // crs-endat.22
#endif // endat22
        // and write protection
    FPGA_ENDAT_SET_WRPROT(dwBaseAddress) = TRUE;

        // adapt to module clock
    timer_wait(uwSysTimers100ns,ENDAT_COMMANDMODE_SET2BUSYGAP*10);

    return TRUE;
}

//***************************************************************************
// Execute transaction

static ULONG EndatExecuteTransaction(ENDAT_WORKS * psWorks, UBYTE ubCommand, UBYTE ubAddress, UWORD uwParam)
{
    ULONG dwBaseAddress=psWorks->dwBaseAddress;
    UWORD uwTmp; 
    ULONG ulResult;

        // FPGA_XENDAT_CMDADR : COMMAND[13:8] and ADDRESS[7:0]
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_CMDADR) = ( (ubCommand&0x3F) << 8 ) | ubAddress;
    
        // FPGA_XENDAT_PARAM
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_PARAM) = uwParam;
    
        // set mode and ext flags from 2 MSB bits of ubCommand
    FPGA_ENDAT_SET_MODE(dwBaseAddress) = (BOOL)(ubCommand&ENDAT_CMDM_MODE) ? TRUE : FALSE;
    FPGA_ENDAT_SET_EXT(dwBaseAddress)  = (BOOL)(ubCommand&ENDAT_CMDM_EXT) ? TRUE : FALSE;

        // set position format and early stop according to protocol selection
    if(ubCommand&ENDAT_CMDM_EXT)
    {
            // on endat 2.2 position format is just for calculating frame length,
            // but position return value will be overwritten by additional information
            // early stop is used here to include additional information frame length
        FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_POSFORMAT) = psWorks->uwPosFormat;
        uwTmp = 1+(psWorks->uwPosFormat>>8)+ENDAT_NBIT_MCT;
        if(psWorks->flags.b.bAdInfoEnabled)
            uwTmp += ENDAT_NBIT_ADINFO;
        FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_EARLYSTOP) = uwTmp;
    }
    else
    {
            // standard protocol command mode
        FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_POSFORMAT) = ENDAT_NBIT_TPAR << 8;
        FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_EARLYSTOP) = ENDAT_NBIT_TPAR+ENDAT_NBIT_MCT;
    }

        // Start Transaction
    FPGA_ENDAT_SET_START(dwBaseAddress) = TRUE; 

        // adapt to module clock
    timer_wait(uwSysTimers100ns,ENDAT_COMMANDMODE_SET2BUSYGAP*10);

        // wait end of transaction
    uwTmp=timer_settimeout(uwSysTimers1ms, ENDAT_COMMANDMODE_TIMEOUT);
    while(!timer_istimedout(uwSysTimers1ms, uwTmp) && FPGA_ENDAT_STAT_BUSY(FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STATUS)));
        //$TEST$ os_sleep? wdt_clear?

        // End Transaction
    FPGA_ENDAT_SET_START(dwBaseAddress) = FALSE; 

        // recovery time before next command mode transaction
    timer_wait(uwSysTimers100ns,ENDAT_COMMANDMODE_RECOVERYTIME*10);

        // if cmd ext without adinfo then ignore any error, return zero
        // as command is just write w/o feedback
    if((ubCommand&ENDAT_CMDM_EXT) && !psWorks->flags.b.bAdInfoEnabled)
        return 0ul;

        // if valid transaction
    if(FPGA_ENDAT_STAT_VALID(FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STATUS)))
    {
#ifdef ENDAT22
        if(psWorks->flags.b.bEndatEx) // crs-endat.22
        {
            ulResult = FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_PARAM) | ((ULONG)FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_CMDADR)<<16);
            if(FPGA_ENDAT_STAT_ADICRCERR(FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STATUS)))
                ulResult|=ENDAT_FUNCTION_ERROR_CRC_ERROR;
        }
        else // crs-endat.22
#endif // endat22
        {   // get result
            ulResult=FPGA_BASEOFF_32(dwBaseAddress, FPGA_ENDAT_DATAOUT_LLSW);
 
                // and check CRC
            if(FPGA_ENDAT_STAT_CRCERR(FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STATUS)))
                ulResult|=ENDAT_FUNCTION_ERROR_CRC_ERROR;
        }
    }
    else
        ulResult=ENDAT_FUNCTION_ERROR_TIMEOUT;

    return ulResult;
}

//***************************************************************************
// Verify and enable protocol 2.2 additional information

static ULONG EndatEnableADInfo(ENDAT_WORKS * psWorks)
{
    ULONG ulResult;

        // if protocol 2.2 then select additional information to be always sent
    if(psWorks->flags.b.bProt22)
    {
            // enable additional information with NOP
        ulResult=EndatExecuteTransaction(psWorks, ENDAT_CMD22_SELECTION_OF_MEMORY_AREA, ADINFO_SEL(ENDAT_ADINFO1_NOP), 0x0000);
        if( ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return 16 | (( ulResult >> 16 ) & 0xFF00);

            // subsequent parameter access will be made with protocol 2.2
        psWorks->flags.b.bAdInfoEnabled = TRUE;
    }

    return 0ul;
}    

//***************************************************************************
#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
// set additional info, if requested
// crs-endat.22
static ULONG EndatSetADInfo(ENDAT_WORKS * psWorks)
{
    ULONG ulResult;

    if(psWorks->ubADInfo != ENDAT_ADINFO1_NOP)
    {
        BOOL prevadinfoen = psWorks->flags.b.bAdInfoEnabled;
        psWorks->flags.b.bAdInfoEnabled = TRUE;

        ulResult=EndatExecuteTransaction(psWorks, ENDAT_CMD22_SELECTION_OF_MEMORY_AREA, ADINFO_SEL(psWorks->ubADInfo), 0x0000);

        psWorks->flags.b.bAdInfoEnabled = prevadinfoen;

        if( ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return 17 | (( ulResult >> 16 ) & 0xFF00);
    }

    return 0ul;
}
#endif // endat22 && endat22_addinfo

//***************************************************************************
// Read parameter

ULONG EndatReadParameter(ENDAT_WORKS * psWorks, UBYTE ubMemArea, UBYTE ubAddress)
{
    ADINFO sRes;
    UWORD uwTimeout,uwReadVal;

        // standard protocol
    if(!psWorks->flags.b.bProt22)
    {
            // selection of memory area
        sRes.ulData=EndatExecuteTransaction(psWorks, ENDAT_COMMAND_SELECTION_OF_MEMORY_AREA, ubMemArea, 0x0000);
        if( sRes.ulData & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return sRes.ulData;
    
            // Read Parameter
        return EndatExecuteTransaction(psWorks, ENDAT_COMMAND_ENCODER_TO_SEND_PARAMETERS, ubAddress, 0x0000 );
    }
        // protocol 2.2
    else
    {
            // enable 2.2 protocol if necessary
        sRes.ulData=EndatEnableADInfo(psWorks);
        if( sRes.ulData & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return sRes.ulData;

            // selection of memory area
        sRes.ulData=EndatExecuteTransaction(psWorks, ENDAT_CMD22_SELECTION_OF_MEMORY_AREA, ubMemArea, 0x0000);
        if( sRes.ulData & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return sRes.ulData;

            // select MRS as adinfo
        sRes.ulData=EndatExecuteTransaction(psWorks, ENDAT_CMD22_SELECTION_OF_MEMORY_AREA, ADINFO_SEL(ENDAT_ADINFO1_MRS), 0x0000);
        if( sRes.ulData & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return sRes.ulData;

            // ack MRS
        sRes.ulData=EndatExecuteTransaction(psWorks, ENDAT_CMD22_ENCODER_TO_SEND_POSITION_VALUES, 0x00, 0x0000);
        if( sRes.ulData & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return sRes.ulData;

        if(ADINFO_ACK_SEL(sRes.ubData[2])!=ENDAT_ADINFO1_MRS || sRes.ubData[1]!=ubMemArea)
            return ENDAT_FUNCTION_ERROR_ACCESSDENIED;

            // request memory area
        sRes.ulData=EndatExecuteTransaction(psWorks, ENDAT_CMD22_ENCODER_TO_SEND_PARAMETERS, ubAddress, 0x0000);
        if( sRes.ulData & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return sRes.ulData;

            // wait until busy bit
        uwTimeout=timer_settimeout(uwSysTimers1ms, ENDAT_COMMANDMODE_TIMEOUT);
        for(;;)
        {
                // readback info for busy checking
            sRes.ulData=EndatExecuteTransaction(psWorks, ENDAT_CMD22_ENCODER_TO_SEND_POSITION_VALUES, 0x00, 0x0000);
            if(sRes.ulData & ENDAT_FUNCTION_ERROR_MASK_ANY)
                return sRes.ulData;

            if(!ADINFO_ACK_BUSY(sRes.ubData[2]))
                break;

            if(timer_istimedout(uwSysTimers1ms, uwTimeout))
                return ENDAT_FUNCTION_ERROR_TIMEOUT;

            //$TEST$ os_sleep? wdt_clear?
        }

            // select LSB readback as adinfo
        sRes.ulData=EndatExecuteTransaction(psWorks, ENDAT_CMD22_SELECTION_OF_MEMORY_AREA, ADINFO_SEL(ENDAT_ADINFO1_MEM_LSB), 0x0000);
        if( sRes.ulData & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return sRes.ulData;

            // read LSB
        sRes.ulData=EndatExecuteTransaction(psWorks, ENDAT_CMD22_ENCODER_TO_SEND_POSITION_VALUES, 0x00, 0x0000);
        if( sRes.ulData & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return sRes.ulData;

        if(ADINFO_ACK_SEL(sRes.ubData[2])!=ENDAT_ADINFO1_MEM_LSB || sRes.ubData[1]!=ubAddress)
            return ENDAT_FUNCTION_ERROR_ACCESSDENIED;

            // LSB part
        uwReadVal=(UWORD)sRes.ubData[0];

            // select MSB readback as adinfo
        sRes.ulData=EndatExecuteTransaction(psWorks, ENDAT_CMD22_SELECTION_OF_MEMORY_AREA, ADINFO_SEL(ENDAT_ADINFO1_MEM_MSB), 0x0000);
        if( sRes.ulData & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return sRes.ulData;

            // read MSB
        sRes.ulData=EndatExecuteTransaction(psWorks, ENDAT_CMD22_ENCODER_TO_SEND_POSITION_VALUES, 0x00, 0x0000);
        if( sRes.ulData & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return sRes.ulData;

        if(ADINFO_ACK_SEL(sRes.ubData[2])!=ENDAT_ADINFO1_MEM_MSB || sRes.ubData[1]!=ubAddress)
            return ENDAT_FUNCTION_ERROR_ACCESSDENIED;

            // MSB part
        uwReadVal|=(UWORD)sRes.ubData[0]<<8;

#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
        	// restore realtime additional info if requested
       sRes.ulData=EndatSetADInfo(psWorks);
       if( sRes.ulData & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return sRes.ulData;
#endif // endat22 && endat22_addinfo

        return (ULONG)uwReadVal;
    }
}

//***************************************************************************
// Write parameter

ULONG EndatWriteParameter(ENDAT_WORKS * psWorks, UBYTE ubMemArea, UBYTE ubAddress, UWORD uwParam)
{
    ULONG ulResult;

        // selection of memory area
    ulResult=EndatExecuteTransaction(psWorks, ENDAT_COMMAND_SELECTION_OF_MEMORY_AREA, ubMemArea, 0x0000);
    if( ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY )
        return ulResult;

        // Write Parameter
    ulResult=EndatExecuteTransaction(psWorks, ENDAT_COMMAND_ENCODER_TO_RECEIVE_PARAMETERS, ubAddress, uwParam );
    
    return ulResult;
}

#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
//***************************************************************************
// Enable/disable additional info on realtime position reading
// crs-endat.22
BOOL EndatSetRealtimeADInfo(ENDAT_WORKS * psWorks, UBYTE ubADInfo)
{
    if(ubADInfo & ~0x0F)
        return FALSE;
    psWorks->ubADInfo = ubADInfo;
    return TRUE;
}
#endif // endat22 && endat22_addinfo

//***************************************************************************
// Reset Active Alarms

ULONG EndatResetActiveAlarms(ENDAT_WORKS * psWorks)
{
    ULONG ulResult;

        // standard protocol
    if(!psWorks->flags.b.bProt22)
    {
            // Clear Warnings
        ulResult=EndatWriteParameter(psWorks, ENDAT_MEMAREA_OPERATING_CONDITION, ENDAT_ADDRESS_OPERATING_STATUS_WARNINGS, 0x0000);
        if( ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return ulResult;
    
            // Clear Errors
        ulResult=EndatWriteParameter(psWorks, ENDAT_MEMAREA_OPERATING_CONDITION, ENDAT_ADDRESS_OPERATING_STATUS_ERRORS, 0x0000);
        if( ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return ulResult;
        
            // Execute Reset
        ulResult=EndatExecuteTransaction(psWorks, ENDAT_COMMAND_ENCODER_TO_RECEIVE_RESET, 0x00, 0x0000 );
    }
        // protocol 2.2
    else
    {
            // enable 2.2 protocol if necessary
        ulResult=EndatEnableADInfo(psWorks);
        if( ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return ulResult;

            // Execute Reset
        ulResult=EndatExecuteTransaction(psWorks, ENDAT_CMD22_ENCODER_TO_RECEIVE_RESET, 0x00, 0x0000 );

            // Reset will also cause no anymore additional info
        psWorks->flags.b.bAdInfoEnabled = FALSE;

#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
            // restore realtime additional info if requested
        ulResult=EndatSetADInfo(psWorks);
        if( ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY )
            return ulResult;
#endif // endat22 && endat22_addinfo
    }

    return ulResult;
}

//***************************************************************************
// Reset Encoder

ULONG EndatResetEncoder(ENDAT_WORKS * psWorks)
{
    ULONG ulResult;

        // Execute Reset
    ulResult=EndatExecuteTransaction(psWorks, ENDAT_COMMAND_ENCODER_TO_RECEIVE_RESET, 0x00, 0x0000 );

        // Reset will also cause no anymore additional info
    psWorks->flags.b.bAdInfoEnabled = FALSE;

    return ulResult;
}


//***************************************************************************
// Initialize encoder, check interface version, adjust propagation time,
// get position format

UWORD EndatInitializeEncoder(ULONG dwBaseAddress, UWORD uwClockFreq, ENDAT_WORKS * psWorks, BOOL bCalcStartDelay)
{
    ULONG ulResult,ulPropCompSample;
    UWORD uwTransferFormat,uwDistingRevolut,uwStepPerRevolutionBits,uwRevolutionNumbersBits;
    UWORD uwCnt,uwPrevPropComp,uwTimeOut,uwTolerance,uwStartDelay;
    BOOL bProt22;

        // pre-initialize works data structure as old endat 2.1 protocol
    psWorks->dwBaseAddress=dwBaseAddress;
    psWorks->flags.b.bProt22=bProt22=FALSE;
    psWorks->flags.b.bAdInfoEnabled=FALSE;

#ifdef ENDAT22
        // reset all SYNC tran flag
    FPGA_ENDAT_SET_SYNCTRAN(dwBaseAddress) = FALSE;

        // get module implementation type
    psWorks->flags.b.bEndatEx = FPGA_ENDAT_STAT_ENDATEX(FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STATUS)) != 0;
#if defined(ENDAT22_ADDINFO)
        // test if realtime ADInfo is supported by the module
    if(psWorks->ubADInfo != ENDAT_ADINFO1_NOP && !psWorks->flags.b.bEndatEx)
        return 22;
#endif  // endat22_addinfo
#endif  // endat22

        // set command mode
    if(!EndatEnterCommandMode(psWorks))
        return 1;

        // check frequency range
    if(!EndatCheckParameters(uwClockFreq))
        return 2;

        // get command set to check max frequency allowed, first transaction will be repeated
        // up to timeout in order to give time for powerup
    uwTimeOut=timer_settimeout(uwSysTimers1ms, ENDAT_POWERUP_TIMEOUT);
    for(;;)
    {
        ulResult=EndatReadParameter(psWorks, ENDAT_MEMAREA_MANUFACTURER_PARAM_A5, ENDAT_ADDRESS_ENDAT_COMMAND_SET);

            // if no error then transaction ok, then proceed further
        if((ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY) == 0l)
            break;

            // if timeout then error
        if(timer_istimedout(uwSysTimers1ms, uwTimeOut))
            return 3 | (( ulResult >> 16 ) & 0xFF00);

            // gap period before trying again
        timer_wait(uwSysTimers1ms, ENDAT_POWERUP_GAP);
    }
    
        // if Endat 2.1 then max freq is lower
    if((ulResult&0x0003)!=0x0001)
    {
            // max frequency managed by 2.1
        psWorks->uwMaxFrequency=ENDAT_CLOCK_MAX_21_FREQUENCY;

        if(uwClockFreq>ENDAT_CLOCK_MAX_21_FREQUENCY)
            return 2;
    }
        // takes note of protocol 2.2, for later setup in the works area when position
        // frame size is known
    else
        bProt22=TRUE;

#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
        // if not protocol 2.2, ADInfo is not supported
    if(psWorks->ubADInfo != ENDAT_ADINFO1_NOP && !bProt22)
        return 23;
#endif // endat22 && endat22_addinfo

        // Read Transfer Format for Position Values
    ulResult=EndatReadParameter(psWorks, ENDAT_MEMAREA_MANUFACTURER_PARAM_A1, ENDAT_ADDRESS_TRANSFER_FORMAT_FOR_POSITION);
    if(ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY)
        return 4 | (( ulResult >> 16 ) & 0xFF00);

        // Bit 15 Always set to logical High for EEPROM checking
    if(!( ulResult & 0x00008000 ))
        return 5;

        // Number of clock pulses (Transfer Format)
    uwTransferFormat = (UWORD)( ulResult & 0x00007fff );
    
        // FPGA Shift Register 64 bits limt
    if(uwTransferFormat > 64)
        return 6;

        // pre-fillup works area to enable system to protocol 2.2, if needed
    psWorks->uwPosFormat = uwTransferFormat<<8;
    psWorks->flags.b.bProt22 = bProt22;

        // and if enabled test protocol 2.2 by dummy reading before proceeding further
    if(bProt22)
    {
        ulResult=EndatReadParameter(psWorks, ENDAT_MEMAREA_MANUFACTURER_PARAM_A5, ENDAT_ADDRESS_ENDAT_COMMAND_SET);

            // if error revert back to standard protocol
        if(ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY)
            psWorks->flags.b.bProt22=bProt22=FALSE;
    }

        // Read Number of Distinguishable Revolutions
    ulResult=EndatReadParameter(psWorks, ENDAT_MEMAREA_MANUFACTURER_PARAM_A3, ENDAT_ADDRESS_DISTINGUISHABLE_REVOLUTIONS);
    if(ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY)
        return 7 | (( ulResult >> 16 ) & 0xFF00);
    
        // Distinguishable revolutions
    uwDistingRevolut = (UWORD)( ulResult & 0x0000ffff );
    
        // Assuming Single Turn
    uwStepPerRevolutionBits = uwTransferFormat; uwRevolutionNumbersBits = 0;
    
        // EnDat 2.2 encoder with more than 65535 revolutions
    if(uwDistingRevolut == 65535 && bProt22)
    {
            // Read Number of Distinguishable Revolutions with scaling factor
        ulResult=EndatReadParameter(psWorks, ENDAT_MEMAREA_MANUFACTURER_PARAM22, ENDAT_ADDRESS_DISTINGUISHABLE_REV_W_SCALING);
        if(ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY)
            return 8 | (( ulResult >> 16 ) & 0xFF00);

            // only supported 2^x scaling type
        if((ulResult&0x0000E000)!=0x00002000)
            return 19;

            // 2^number of rev
        uwRevolutionNumbersBits=(UWORD)(ulResult&0x00001FFF);

            // more than 32bit as number of revolution are not supported
        if(uwRevolutionNumbersBits>32)
            return 20;

            // Adjust Step Per Revolution Bits
        uwStepPerRevolutionBits -= uwRevolutionNumbersBits;
    }
        // If standard Multi Turn Compute Number of Bits per Revolution
    else if(uwDistingRevolut != 0)
    {
        UWORD uwTemp = uwDistingRevolut;
    
            // Safety Check for actual implementation
        if(!CheckIfNumberIsPowerOfTwo( uwDistingRevolut ))
            return 9;
    
            // Compute Number of Bits per Revolution
        for(uwRevolutionNumbersBits = 0; uwTemp > 0; uwRevolutionNumbersBits++)
            uwTemp = uwTemp >> 1;
        uwRevolutionNumbersBits--;
        
            // Adjust Step Per Revolution Bits
        uwStepPerRevolutionBits -= uwRevolutionNumbersBits;
    }

        // more than 32bit per turn not supported
    if(uwStepPerRevolutionBits>32)
        return 10;

        // if protocol 2.2 read max frequency supported
    if(bProt22)
    {
            // Read Number of Distinguishable Revolutions with scaling factor
        ulResult=EndatReadParameter(psWorks, ENDAT_MEMAREA_MANUFACTURER_PARAM22, ENDAT_ADDRESS_MAX_FREQUENCY);
        if(ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY)
            return 21 | (( ulResult >> 16 ) & 0xFF00);

            // max frequency, lower of encoder and drive ones
        psWorks->uwMaxFrequency=(UWORD)(ulResult&0x0000FFFF);

        if(psWorks->uwMaxFrequency>ENDAT_CLOCK_MAX_22_FREQUENCY)
            psWorks->uwMaxFrequency=ENDAT_CLOCK_MAX_22_FREQUENCY;

            // if selected more than supported then error
        if(uwClockFreq>psWorks->uwMaxFrequency)
            return 2;
    }

        // start filling parameters
    psWorks->uwClockDiv  = (UWORD)(ENDAT_CLOCK_BASE_FREQUENCY/2/uwClockFreq)-1;
    psWorks->uwPosFormat = (uwTransferFormat<<8)|uwRevolutionNumbersBits;

        // tolerance for propagation compensation must be less than 10% of
        // clock period time
    uwTolerance=(UWORD)(2l*psWorks->uwClockDiv*ENDAT_PROPCOMP_CALC_TOLERANCE/100l);

        // start propagation time measurement, done with lower clock
    for(uwCnt=0,ulPropCompSample=0l,uwPrevPropComp=0;uwCnt<ENDAT_PROPCOMP_CALC_NUM_SAMPLES;uwCnt++)
    {
            // execute dummy read
        ulResult=EndatReadParameter(psWorks, ENDAT_MEMAREA_OPERATING_CONDITION, ENDAT_ADDRESS_OPERATING_STATUS_ERRORS);
        if(ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY)
            return 11 | (( ulResult >> 16 ) & 0xFF00);

            // each sample must remain inside max tolerance admitted
        uwTimeOut=FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_CLOCKDIV)-FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_PROPCOMPMEAS);
        if(uwPrevPropComp>0)
            if(labs((SLONG)uwPrevPropComp-(SLONG)uwTimeOut) > (SLONG)uwTolerance)
                return 12;

            // get measurement and accumulate
        uwPrevPropComp=uwTimeOut;
        ulPropCompSample+=(ULONG)uwPrevPropComp;
    }

        // calculate propagation time
    psWorks->uwPropComp        = (UWORD)(ulPropCompSample/ENDAT_PROPCOMP_CALC_NUM_SAMPLES);
    psWorks->uwPropagationTime = (UWORD)(1000000*psWorks->uwPropComp/ENDAT_CLOCK_BASE_FREQUENCY);

        // calculate early stop clock cycle in order to avoid more clock than necessary
    uwCnt=1+uwTransferFormat+ENDAT_NBIT_MCT-psWorks->uwPropComp/(2*psWorks->uwClockDiv);
    psWorks->uwEarlyStop = uwCnt;

#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
    if(psWorks->ubADInfo != ENDAT_ADINFO1_NOP)
        psWorks->uwEarlyStop += ENDAT_NBIT_ADINFO + 1;
#endif // endat22 && endat22_addinfo

        // fillup position format
    psWorks->ubStepPerRevBits = (UBYTE)uwStepPerRevolutionBits;
    if(uwStepPerRevolutionBits>0)
        psWorks->ubRevNumBits  = (UBYTE)uwRevolutionNumbersBits;
    else
        psWorks->ubRevNumBits  = 0;
    
        // reset active alarms
    ulResult=EndatResetActiveAlarms(psWorks);
    if(ulResult & ENDAT_FUNCTION_ERROR_MASK_ANY)
        return 13 | (( ulResult >> 16 ) & 0xFF00);

        // if start delay calculation not requested
    if(!bCalcStartDelay)
    {
            // enter position mode
        EndatEnterPositionMode(psWorks);
    
            // enable write protection
        FPGA_ENDAT_SET_WRPROT(dwBaseAddress) = TRUE;

            // and exit
        return 0;
    }

        // begin calibration of start delay
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STARTDELAY)=uwStartDelay=ENDAT_STARTDELAY_FULLCYCLETIME-ENDAT_STARTDELAY_GUARDTIME;

        // enter position mode
    EndatEnterPositionMode(psWorks);

        // but disable write protection
    FPGA_ENDAT_SET_WRPROT(dwBaseAddress) = FALSE;

    for(;;)
    {
            // cycle transaction
        uwTimeOut=timer_settimeout(uwSysTimers100ns, ENDAT_STARTDELAY_TIMEOUT*10);
        while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) && !FPGA_ENDAT_STAT_BUSY(FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STATUS)));
        if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
            return 14;

        uwTimeOut=timer_settimeout(uwSysTimers100ns, ENDAT_STARTDELAY_TIMEOUT*10);
        while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) &&  FPGA_ENDAT_STAT_BUSY(FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STATUS)));
        if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
            return 14;

            // check overtime fault
        if(!FPGA_ENDAT_STAT_OVERTIME(FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STATUS)))
            break;

            // delay must be modified, check if possible, otherwise is not possible to
            // have position each cycle due to lower clock and higher transfer bit number
        if(uwStartDelay<=ENDAT_STARTDELAY_GUARDTIME+ENDAT_STARTDELAY_STEP)
            return 14;

        uwStartDelay-=ENDAT_STARTDELAY_STEP;
        FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STARTDELAY)=uwStartDelay;
    }

        // apply guard time
    uwStartDelay-=ENDAT_STARTDELAY_GUARDTIME;
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STARTDELAY)=uwStartDelay;

        // wait for clean transaction
    uwTimeOut=timer_settimeout(uwSysTimers100ns, ENDAT_STARTDELAY_TIMEOUT*10);
    while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) && !FPGA_ENDAT_STAT_BUSY(FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STATUS)));
    if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
        return 15;

    uwTimeOut=timer_settimeout(uwSysTimers100ns, ENDAT_STARTDELAY_TIMEOUT*10);
    while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) &&  FPGA_ENDAT_STAT_BUSY(FPGA_BASEOFF_16(dwBaseAddress, FPGA_ENDAT_STATUS)));
    if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
        return 15;

        // and enable write protection
    FPGA_ENDAT_SET_WRPROT(dwBaseAddress) = TRUE;

    return 0;
}

//***************************************************************************
// Check Endat parameters (right now just frequency)

BOOL EndatCheckParameters(UWORD uwClockFreq)
{
        // check frequency range
    if(uwClockFreq<ENDAT_CLOCK_MIN_FREQUENCY || uwClockFreq>ENDAT_CLOCK_MAX_22_FREQUENCY)
        return FALSE;
    else 
        return TRUE;
}
#endif
