/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : BissHandlers.c                                             */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Low level Biss interface management functions              */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "drive\BissHandlers.h"
#include "fpga\FpgaHandler.h"
#include "common\Int64Functions.h"
#include "common\TaskScheduler.h"
#include <stdlib.h>

#if CFG_ENC_BISS
//***************************************************************************
// Defines

#define BISS_POWERUP_TIMEOUT                       1000    // msec
#define BISS_COMMANDMODE_TIMEOUT                   100     // msec

#define BISS_CLOCK_BASE_FREQUENCY                 (160000ul)// khz

#define BISS_CLOCK_MIN_FREQUENCY                   1000    // khz
#define BISS_CLOCK_MAX_FREQUENCY                   10000   // khz
#define BISS_CLOCK_STEP_FREQUENCY                  500     // khz

#define BISS_STARTDELAY_FULLCYCLETIME              5000*4    // 125usec @160MHz
#define BISS_STARTDELAY_GUARDTIME                  200*4     // 5usec @160MHz
#define BISS_STARTDELAY_STEP                       40*4      // 1usec @160MHz
#define BISS_STARTDELAY_TIMEOUT                    750       // usec

#define BISS_CRC_POLY                              0x43
#define BISS_CRC_LEN                               6       // CRC[6:1]
#define BISS_FLAG_LEN                              2       // WARN & ERR
#define BISS_MIN_DATALEN                           12
#define BISS_MAX_DATALEN                           55

typedef struct
{
  UBYTE mtlen;
  UBYTE stlen;
}BISS_DATA_LEN;

#ifdef BISS_BIDIRECTIONAL
typedef struct
{
  UWORD scdlen  : 6;
  UWORD enscd   : 1;
  UWORD grays   : 1;
  UWORD scrcpoly: 7;
  UWORD invcrcs : 1;
}BISS_SLAVECFG;

typedef struct
{
  struct
  {
    UWORD regadr : 7;
    UWORD wnr    : 1;
    UWORD regnum : 8;
  }w0;
  struct
  {
    UWORD chsel  : 8;
    UWORD holdcdm: 1;
    UWORD enmo   : 1;
    UWORD resverd: 1;
    UWORD slaveid: 3;
    UWORD regvers: 1;
    UWORD cts    : 1;
  }w1;
}BISS_REGCFG;

typedef struct
{
  UBYTE freq : 5;
  UBYTE freqr: 3;
  UBYTE reserved;
  UBYTE freqags;
  UBYTE mo_busy;
//  UBYTE revision;
//  UBYTE version;
}BISS_MASTERCFG;

typedef struct
{
    UBYTE slaveloc;
    UBYTE setbiss0;
    UBYTE setbiss1;
    UBYTE actnsens;
}BISS_CHCFG;

typedef struct
{
  UWORD eof    : 1;
  UWORD h1     : 1;
  UWORD regend : 1;
  UWORD nregerr: 1;
  UWORD nscderr: 1;
  UWORD h2     : 1;
  UWORD nwderr : 1;
  UWORD nerr   : 1;
  UWORD l1     : 1;
  UWORD svalid1: 1;
  UWORD l2     : 1;
  UWORD svalid2: 1;
  UWORD l3     : 1;
  UWORD svalid3: 1;
  UWORD l4     : 1;
  UWORD svalid4: 1;
}BISS_STATUSINFO;

typedef struct
{
  UWORD ags: 1;
  UWORD instr: 3;
  UWORD init: 1;
  UWORD swbank: 1;
  UWORD holdbank: 1;
  UWORD bread: 1;
  UWORD reserved: 4;
  UWORD mafs: 1;
  UWORD mavs: 1;
  UWORD mafo: 1;
  UWORD mavo: 1;
}BISS_INSTR;

//***************************************************************************
// Command and register access

#define BISS_COMMAND_WRITE                         0x0
#define BISS_COMMAND_READ                          0x1   

// Register mapping of MB101x
#define BISS_ADDRESS_SCDATA                        0x00 // Sensor and Actuator Data: SCDATA[7:0]

#define BISS_ADDRESS_SLAVECFG                      0xC0 // Slave Configuration - 0xC0: GRAYS1(7) ENSCD1(6) SCDLENA[5:0]
                                                        //                     - 0xC1: INVCRCS1(7) SCRCPOLY1[7:1]

#define BISS_ADDRESS_REGDATA                       0x80 // Register Data: RDATA[7:0]
#define BISS_ADDRESS_REGCFG                        0xE2 // Register Communication Config - 0xE2: WNR(7) REGADR[6:0]
                                                        //                               - 0xE3: REGNUM[5:0]
                                                        //                               - 0xE4: CHSEL[7:0]
                                                        //                               - 0xE5: CTS(7) REGVERS(6) SLAVEID[2:0] (2) EnMO(1) HOLDCDM(0)
#define BISS_REGCFG_WNR_WR                         1
#define BISS_REGCFG_WNR_RD                         0
#define BISS_REGCFG_CTS_INS                        0
#define BISS_REGCFG_CTS_REG                        1
#define BISS_REGCFG_HOLDCDM_HIGH                   0
#define BISS_REGCFG_HOLDCDM_CONST                  1
#define BISS_REGCFG_REGVERS_AB                     0
#define BISS_REGCFG_REGVERS_C                      1

#define BISS_ADDRESS_MASTERCFG                     0xE6
#define BISS_ADDRESS_MASTERCFG_FREQR               0xE6 // Master Configuration: FREQR[2:0] FREQ[4:0]
#define BISS_ADDRESS_MASTERCFG_FREQAGS             0xE8 // Master Configuration: FREQAGS[7:0]
#define BISS_ADDRESS_MASTERCFG_MOBUSY              0xE9 // Master Configuration: MO_BUSY[7:0]
#define BISS_ADDRESS_MASTERCFG_REVISION            0xEA // Master Configuration: REVISION[7:0]
#define BISS_ADDRESS_MASTERCFG_VERSION             0xEB // Master Configuration: VERSION[7:0]

#define BISS_ADDRESS_CHCFG                         0xEC // Channel COnfiguration: SLAVELOC[8:1]
                                                        //                        SELSSI4(7) BISSMOD4(6) SELSSI3(5) BISSMOD3(4) SELSSI2(3) BISSMOD2(2) SELSSI1(1) BISSMOD1(0)
                                                        //                        SELSSI8(7) BISSMOD8(6) SELSSI7(5) BISSMOD7(4) SELSSI6(3) BISSMOD6(2) SELSSI5(1) BISSMOD5(0)
                                                        //                        ACTnSENS[8:1]
#define BISS_CHCFG_BISSMOD_AB                      0
#define BISS_CHCFG_BISSMOD_C                       1
#define BISS_CHCFG_SELSSI_BISS                     0
#define BISS_CHCFG_SELSSI_SSI                      1
#define BISS_CHCFG_ACTNSENS_SENSOR                 0
#define BISS_CHCFG_ACTNSENS_ACTUATOR               1

#define BISS_ADDRESS_STATUSINFO                    0xF0 // Status Information: nERR(7) nWDERR(6) '1' nSCDERR(4) nREGERR(3) REGEND(2) '1' EOT(0)

#define BISS_ADDRESS_INSTR                         0xF4 // Instruction Register: BREAK(7) HOLDBANK(6) SWBANK(5) INIT(4) INSTR[2:0] AGS(0)
                                                        // Instruction Register: MAVO(7) MAFO(6) MAVS(5) MAFS(4)
#endif

#define BISS_SUPPORT_NUM                           16

#define BISS_TRANS_OK                              0
#define BISS_TRANS_TIMEOUT                         1
#define BISS_TRANS_CRCERROR                        2

//***************************************************************************
// Locals
#ifdef BISS_BIDIRECTIONAL
static void BissInitRegister(BISS_WORKS huge * psWorks);
static void BissReadParameter(ULONG dwBaseAddress, UBYTE ubAddress, HPUWORD hpuwData, UBYTE ubNum);
static void BissWriteParameter(ULONG dwBaseAddress, UBYTE ubAddress, HPUWORD hpuwData, UBYTE ubNum);
static UWORD BissReadRegister(BISS_WORKS huge * psWorks, UBYTE ubAddress);
#endif
static UBYTE BissStartTranscation(ULONG dwBaseAddress);
static void BissSetClockFreq(ULONG dwBaseAddress, ULONG ulClockFreq);
static void BissSetDataLen(ULONG dwBaseAddress, UBYTE ubDataLen);

static const BISS_DATA_LEN sDataLenTotal[BISS_SUPPORT_NUM+1] ={
                                                   {12,12}, //    24-bit
                                                   {13,12},
                                                   {16,12},
                                                   
                                                   {12,13}, //    25-bit
                                                   {16,13}, //    29-bit
                                                   
                                                   {12,14}, //    26-bit

                                                   {12,16}, //    28-bit
                                                   
                                                   {12,17}, //    29-bit
                                                   {13,17}, //    30-bit
                                                   {16,17}, //    33-bit
                                                   {24,17}, //    41-bit
                                                   
                                                   {13,18}, //    31-bit
                                                   
                                                   {12,19}, //    31-bit
                                                   
                                                   {12,22}, //    34-bit

                                                   {16,23}, //    39-bit
                                                   
                                                   {12,24}, //    36-bit
                                                   
                                                   {0 ,0 }};

//***************************************************************************
// Biss command mode

#ifdef _INFINEON_
void BissEnterCommandMode(BISS_WORKS huge * psWorks)
#else
void BissEnterCommandMode(BISS_WORKS      * psWorks)
#endif
{
    ULONG dwBaseAddress=psWorks->dwBaseAddress;

    FPGA_BISS_SET_MODE(dwBaseAddress) = TRUE;
}

//***************************************************************************
// Biss position mode (self triggered)

#ifdef _INFINEON_
void BissEnterPositionMode(BISS_WORKS huge * psWorks)
#else
void BissEnterPositionMode(BISS_WORKS      * psWorks)
#endif
{
    ULONG dwBaseAddress=psWorks->dwBaseAddress;

    FPGA_BISS_SET_MODE(dwBaseAddress) = FALSE;
}

//***************************************************************************
// Initialize encoder, check interface version, adjust propagation time,
// get position format

#ifdef _INFINEON_
UWORD BissInitializeEncoder(UBYTE ubType, ULONG dwBaseAddress, ULONG ulClockFreq, BISS_WORKS huge * psWorks, BOOL bCalcStartDelay)
#else
UWORD BissInitializeEncoder(UBYTE ubType, ULONG dwBaseAddress, ULONG ulClockFreq, BISS_WORKS      * psWorks, BOOL bCalcStartDelay)
#endif
{
    UWORD uwStepPerRevolutionBits=0,uwRevolutionNumbersBits=0;
    UWORD uwTimeOut,uwStartDelay;
    UBYTE ubChkCnt=0;
    UBYTE ubValidCnt=0;
    UWORD uwStatus;
    UBYTE ubChkStatus;
    UWORD uwChkClockFreq;
    BOOL  bClockFreqChked=TRUE;
    BOOL  bEncTypeSupported=FALSE;
    
        // pre-initialize works data structure
    psWorks->dwBaseAddress=dwBaseAddress;
    psWorks->ubEncType=ubType;
    
        // set command mode
    BissEnterCommandMode(psWorks);

#ifdef BISS_BIDIRECTIONAL
    BissInitRegister(psWorks);
#endif

        // Encoder Recovery from powerup
    BissSetDataLen(dwBaseAddress,BISS_MIN_DATALEN);
    BissSetClockFreq(dwBaseAddress, BISS_CLOCK_MIN_FREQUENCY);
    uwTimeOut=timer_settimeout(uwSysTimers1ms, BISS_POWERUP_TIMEOUT);
    FPGA_BISS_SET_START(dwBaseAddress)=TRUE;
    
    do{
        uwStatus = FPGA_BASEOFF_16(dwBaseAddress,FPGA_BISS_STATUS);
        if(timer_istimedout(uwSysTimers1ms, uwTimeOut)) // if SL is not reponsed, init failure
        {
            FPGA_BISS_SET_STOP(dwBaseAddress)=TRUE;
            return 2;
        }
    }while(!FPGA_BISS_STAT_BUSY(uwStatus)); // wait for starting
     
        // Check Maximum Frequency
    if(!BissCheckParameters(ubType,ulClockFreq))
        return 1;
        
    uwChkClockFreq = BISS_CLOCK_MAX_FREQUENCY;        
    for(;;)
    {
            // Set check frequency
        BissSetClockFreq(dwBaseAddress, uwChkClockFreq);

            // Start check
        if(BissStartTranscation(dwBaseAddress)==BISS_TRANS_TIMEOUT)
            bClockFreqChked = FALSE;
        
        timer_wait(uwSysTimers100ns,200); // Time Gap between transimission
        
        if(bClockFreqChked) // if clock frequency check ok, then check crc
        {
            for(ubChkCnt=BISS_MIN_DATALEN;ubChkCnt<=BISS_MAX_DATALEN;ubChkCnt++)
            {
                BissSetDataLen(dwBaseAddress, ubChkCnt);
                for(ubValidCnt=0;ubValidCnt<32;ubValidCnt++)
                {
                    ubChkStatus = BissStartTranscation(dwBaseAddress);
                    
                    timer_wait(uwSysTimers100ns,200); // Time Gap between transimission
                    
                    // iterating crc checking
                    if(ubChkStatus != BISS_TRANS_OK)
                        break;    // crc error
                    else
                        continue; // crc ok
                }
                
                // Supported one found, skip the loop checking
                if(ubChkStatus == BISS_TRANS_OK)
                {
                    uwStepPerRevolutionBits = ubChkCnt; 
                    uwRevolutionNumbersBits = 0;
                    bEncTypeSupported = TRUE;
                    break; 
                }
            }
        }    
        
        if(!bEncTypeSupported)
        {
            bClockFreqChked = TRUE;
            uwChkClockFreq -= BISS_CLOCK_STEP_FREQUENCY;
            if(uwChkClockFreq < BISS_CLOCK_MIN_FREQUENCY) // if all frequency are checked, init failure
                return 2;
        }
        else
            break;
    }

        // max frequency
    psWorks->uwMaxFrequency=uwChkClockFreq;
        
        // Set Actual Clock Frequency
    if(ulClockFreq > uwChkClockFreq)
        return 2;
    else
        BissSetClockFreq(dwBaseAddress, ulClockFreq);
    
        // Check support Encoder Type: linear or rotary singleturn
    for(ubChkCnt=BISS_MIN_DATALEN;ubChkCnt<=32;ubChkCnt++)
    {
        BissSetDataLen(dwBaseAddress, ubChkCnt);
        for(ubValidCnt=0;ubValidCnt<3;ubValidCnt++)
        {
            ubChkStatus = BissStartTranscation(dwBaseAddress);
            
            timer_wait(uwSysTimers100ns,200); // Time Gap between transimission
            
            // iterating crc checking
            if(ubChkStatus == BISS_TRANS_TIMEOUT)
                return 2;
            else if(ubChkStatus == BISS_TRANS_CRCERROR)
                break;    // crc error
            else
                continue; // crc ok
        }
        
        // Supported one found, skip the loop checking
        if(ubChkStatus == BISS_TRANS_OK)
        {
            uwStepPerRevolutionBits = ubChkCnt; 
            uwRevolutionNumbersBits = 0;
            bEncTypeSupported = TRUE;
            break; 
        }
    }
    
        // Check support Encoder Type: rotary multiturn
    if(!bEncTypeSupported || ubType==BISS_ENCTYPE_ROTARY_MT)
    {
        for(ubChkCnt=0;ubChkCnt<BISS_SUPPORT_NUM;ubChkCnt++)
        {
            BissSetDataLen(dwBaseAddress, sDataLenTotal[ubChkCnt].mtlen+sDataLenTotal[ubChkCnt].stlen);
            for(ubValidCnt=0;ubValidCnt<3;ubValidCnt++)
            {
                ubChkStatus = BissStartTranscation(dwBaseAddress);
                
                timer_wait(uwSysTimers100ns,200); // Time Gap between transimission
                
                // iterating crc checking
                if(ubChkStatus == BISS_TRANS_TIMEOUT)
                    return 2;
                else if(ubChkStatus == BISS_TRANS_CRCERROR)
                    break;    // crc error
                else
                    continue; // crc ok
            }
            
            // Supported one found, skip the loop checking
            if(ubChkStatus == BISS_TRANS_OK)
                break; 
        }

        if(ubChkCnt < BISS_SUPPORT_NUM)
        {
            bEncTypeSupported = TRUE;
            uwStepPerRevolutionBits = sDataLenTotal[ubChkCnt].stlen; 
            uwRevolutionNumbersBits = sDataLenTotal[ubChkCnt].mtlen;
        }
        else if(bEncTypeSupported) // if failed, restore parameters
            BissSetDataLen(dwBaseAddress, uwStepPerRevolutionBits);
    }    
    
    if(!bEncTypeSupported)
        return 2;
        
        // fillup position format
    psWorks->ubStepPerRevBits = (UBYTE)uwStepPerRevolutionBits;
    if(uwStepPerRevolutionBits>0)
        psWorks->ubRevNumBits  = (UBYTE)uwRevolutionNumbersBits;
    else
        psWorks->ubRevNumBits  = 0;
    
        // if start delay calculation not requested
    if(!bCalcStartDelay)
    {
            // enter position mode
        BissEnterPositionMode(psWorks);
    
            // and exit
        return 0;
    }

        // begin calibration of start delay
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_PRETRG)=uwStartDelay=BISS_STARTDELAY_FULLCYCLETIME-BISS_STARTDELAY_GUARDTIME;

        // enter position mode
    BissEnterPositionMode(psWorks);

    for(;;)
    {
            // cycle transaction
        uwTimeOut=timer_settimeout(uwSysTimers100ns, BISS_STARTDELAY_TIMEOUT*10);
        while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) && !FPGA_BISS_STAT_BUSY(FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_STATUS)));
        if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
            return 14;

        uwTimeOut=timer_settimeout(uwSysTimers100ns, BISS_STARTDELAY_TIMEOUT*10);
        while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) &&  FPGA_BISS_STAT_BUSY(FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_STATUS)));
        if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
            return 14;

            // check overtime fault
        if(!FPGA_BISS_STAT_OVERTIME(FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_STATUS)))
            break;

            // delay must be modified, check if possible, otherwise is not possible to
            // have position each cycle due to lower clock and higher transfer bit number
        if(uwStartDelay<=BISS_STARTDELAY_GUARDTIME+BISS_STARTDELAY_STEP)
            return 14;

        uwStartDelay-=BISS_STARTDELAY_STEP;
        FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_PRETRG)=uwStartDelay;
    }

        // apply guard time
    uwStartDelay-=BISS_STARTDELAY_GUARDTIME;
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_PRETRG)=uwStartDelay;

        // wait for clean transaction
    uwTimeOut=timer_settimeout(uwSysTimers100ns, BISS_STARTDELAY_TIMEOUT*10);
    while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) && !FPGA_BISS_STAT_BUSY(FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_STATUS)));
    if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
        return 15;

    uwTimeOut=timer_settimeout(uwSysTimers100ns, BISS_STARTDELAY_TIMEOUT*10);
    while(!timer_istimedout(uwSysTimers100ns, uwTimeOut) &&  FPGA_BISS_STAT_BUSY(FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_STATUS)));
    if(timer_istimedout(uwSysTimers100ns, uwTimeOut))
        return 15;

    return 0;
}

//***************************************************************************
// Check Biss parameters (right now just frequency)

BOOL BissCheckParameters(UBYTE ubEncType, ULONG ulClockFreq)
{
        // check frequency range
    if(ulClockFreq<BISS_CLOCK_MIN_FREQUENCY || ulClockFreq>BISS_CLOCK_MAX_FREQUENCY)
        return FALSE;
    
        // check encoder type
    if(!(ubEncType==BISS_ENCTYPE_LINEAR || ubEncType==BISS_ENCTYPE_ROTARY || ubEncType==BISS_ENCTYPE_ROTARY_MT))
        return FALSE;
    
    return TRUE;
}

//***************************************************************************
// Read Position
void BissReadPosition(BISS_WORKS* psWorks, HPULONG hpulPosDataH, HPULONG hpulPosDataL)
{
    SQWRD sqPosData;
    ULONG dwBaseAddress;
    
    dwBaseAddress = psWorks->dwBaseAddress;
    
#ifdef BISS_BIDIRECTIONAL
    BissReadParameter(dwBaseAddress, BISS_ADDRESS_SCDATA, (HPUWORD)&sqPosData, sizeof(SQWRD)/2);
#else
    sqPosData.lo = FPGA_BASEOFF_32(dwBaseAddress, FPGA_BISS_DATA_LL);
    sqPosData.hi = FPGA_BASEOFF_32(dwBaseAddress, FPGA_BISS_DATA_HL);
#endif    
    
    if(psWorks->ubEncType!=BISS_ENCTYPE_LINEAR)
        _sint64_shl(&sqPosData, 32-psWorks->ubStepPerRevBits);
    
    *hpulPosDataH = sqPosData.hi;
    *hpulPosDataL = sqPosData.lo;
}

//***************************************************************************
// Read Encoder Status
void BissReadStatus(BISS_WORKS* psWorks, HPUWORD hpuwStatus)
{
    ULONG dwBaseAddress;
    dwBaseAddress = psWorks->dwBaseAddress;
#ifdef BISS_BIDIRECTIONAL
    BISS_STATUSINFO sStatusInfo;
    
    BissReadParameter(dwBaseAddress, BISS_ADDRESS_STATUSINFO, (HPUWORD)&sStatusInfo, sizeof(BISS_STATUSINFO)/2);
    
    FPGA_BISS_STAT_BUSY(*hpuwStatus)     = ~sStatusInfo.eof;     // data transmission completed
    FPGA_BISS_STAT_VALID(*hpuwStatus)    = sStatusInfo.svalid1;  // single cycle data valid
    FPGA_BISS_STAT_ERROR(*hpuwStatus)    = ~sStatusInfo.nscderr; // error in single cycle data transmission
    FPGA_BISS_STAT_OVERTIME(*hpuwStatus) = ~sStatusInfo.nwderr;  // watchdog error
    FPGA_BISS_STAT_ALARM(*hpuwStatus)    = ~sStatusInfo.nerr;    // error
#else
    *hpuwStatus = FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_STATUS);
#endif
}

//***************************************************************************
// Start Transcation
static UBYTE BissStartTranscation(ULONG dwBaseAddress)
{
    UWORD uwStatus;
    UWORD uwTimeOut;
    
    FPGA_BISS_SET_START(dwBaseAddress)=TRUE;
    
    uwTimeOut=timer_settimeout(uwSysTimers1ms, BISS_COMMANDMODE_TIMEOUT);
    
    // wait for starting
    do{
        uwStatus = FPGA_BASEOFF_16(dwBaseAddress,FPGA_BISS_STATUS);
        if(timer_istimedout(uwSysTimers1ms, uwTimeOut))
        {
            FPGA_BISS_SET_STOP(dwBaseAddress)=TRUE;
            return BISS_TRANS_TIMEOUT;
        }
    }while(!FPGA_BISS_STAT_BUSY(uwStatus));
    
    // wait for finishing
    do{
        uwStatus = FPGA_BASEOFF_16(dwBaseAddress,FPGA_BISS_STATUS);
        if(timer_istimedout(uwSysTimers1ms, uwTimeOut))
        {
            FPGA_BISS_SET_STOP(dwBaseAddress)=TRUE;
            return BISS_TRANS_TIMEOUT;
        }
    }while(FPGA_BISS_STAT_BUSY(uwStatus));
        
    // iterating crc checking
    if(!(FPGA_BISS_STAT_VALID(uwStatus)&&(!FPGA_BISS_STAT_ERROR(uwStatus))))
        return BISS_TRANS_CRCERROR;    // crc error
    
    return BISS_TRANS_OK;
}

//***************************************************************************
// Set Clock Frequency
static void BissSetClockFreq(ULONG dwBaseAddress, ULONG ulClockFreq)
{
#ifdef BISS_BIDIRECTIONAL
    BISS_MASTERCFG sMasterCfg;

    BissReadParameter(dwBaseAddress, BISS_ADDRESS_MASTERCFG, (HPUWORD)&sMasterCfg, sizeof(BISS_MASTERCFG)/2);  
    if(ulClockFreq >= 5000)
        sMasterCfg.freq = (UBYTE)(BISS_CLOCK_BASE_FREQUENCY/2/ulClockFreq-1) & 0x1F;
    else
        sMasterCfg.freq = (UBYTE)(BISS_CLOCK_BASE_FREQUENCY/20/ulClockFreq-1)| 0x10;
        // MB101X Master Configuration: frequency
    BissWriteParameter(dwBaseAddress, BISS_ADDRESS_MASTERCFG, (HPUWORD)&sMasterCfg, sizeof(BISS_MASTERCFG)/2);  
#else
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_CLKDIV) = (UBYTE)(BISS_CLOCK_BASE_FREQUENCY/ulClockFreq);
#endif
}

//***************************************************************************
// Set Data Length
static void BissSetDataLen(ULONG dwBaseAddress, UBYTE ubDataLen)
{
#ifdef BISS_BIDIRECTIONAL
    BISS_SLAVECFG  sSlaveCfg;

    BissReadParameter(dwBaseAddress, BISS_ADDRESS_SLAVECFG, (HPUWORD)&sSlaveCfg, sizeof(BISS_SLAVECFG)/2);
    sSlaveCfg.scdlen = ubDataLen+BISS_FLAG_LEN+BISS_CRC_LEN-1;
    BissWriteParameter(dwBaseAddress, BISS_ADDRESS_SLAVECFG, (HPUWORD)&sSlaveCfg, sizeof(BISS_SLAVECFG)/2);
#else
    FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_DATALEN) = ubDataLen;
#endif
}

#ifdef BISS_BIDIRECTIONAL
//***************************************************************************
// Init Register
static void BissInitRegister(BISS_WORKS huge * psWorks)
{
    ULONG dwBaseAddress=psWorks->dwBaseAddress;
    BISS_MASTERCFG sMasterCfg;
    BISS_CHCFG     sChannelCfg;
    BISS_SLAVECFG  sSlaveCfg;
    
    // Master Configuration
    sMasterCfg.freq = 0;
    sMasterCfg.freqr = 0;
    sMasterCfg.freqags = 0;
    sMasterCfg.mo_busy = 0;
    BissWriteParameter(dwBaseAddress, BISS_ADDRESS_MASTERCFG, (HPUWORD)&sMasterCfg, sizeof(BISS_MASTERCFG)/2);  
      
    // Slave Configuration
    sSlaveCfg.scdlen   = 10+BISS_FLAG_LEN+BISS_CRC_LEN-1
    sSlaveCfg.enscd    = 1;
    sSlaveCfg.scrcpoly = BISS_CRC_POLY>>1;
    sSlaveCfg.invcrcs  = 0;
    BissWriteParameter(dwBaseAddress, BISS_ADDRESS_SLAVECFG, (HPUWORD)&sSlaveCfg, sizeof(BISS_SLAVECFG)/2);
    
    // Channel Configuration
    sChannelCfg.slaveloc = 0xFF;
    sChannelCfg.setbiss0 = 0x55;
    sChannelCfg.setbiss1 = 0x55;
    sChannelCfg.actnsens = BISS_CHCFG_ACTNSENS_SENSOR;
    BissWriteParameter(dwBaseAddress, BISS_ADDRESS_CHCFG, (HPUWORD)&sChannelCfg, sizeof(BISS_CHCFG)/2);
}

//***************************************************************************
// Read Register
static UWORD BissReadRegister(BISS_WORKS huge * psWorks, UBYTE ubAddress)
{
    ULONG dwBaseAddress=psWorks->dwBaseAddress;
    UWORD uwData;
    UBYTE ubCycle;
    BISS_REGCFG sRegCfg;
    BISS_INSTR sInstr;
    
    // Instruction
    sInstr.init = 0;
    sInstr.ags = 1;
    sInstr.instr = 2;
    BissWriteParameter(dwBaseAddress, BISS_ADDRESS_INSTR, (HPUWORD)&sInstr, sizeof(BISS_INSTR)/2);
    
    // Register Configuration
    sRegCfg.w0.regadr = ubAddress&0x7F;
    sRegCfg.w0.wnr    = BISS_REGCFG_WNR_RD;
    sRegCfg.w0.regnum = 1;
    sRegCfg.w1.chsel  = 1;
    sRegCfg.w1.holdcdm= BISS_REGCFG_HOLDCDM_HIGH;
    sRegCfg.w1.enmo   = 0;
    sRegCfg.w1.slaveid= 0;
    sRegCfg.w1.regvers= BISS_REGCFG_REGVERS_C;
    sRegCfg.w1.cts    = BISS_REGCFG_CTS_REG;
    BissWriteParameter(dwBaseAddress, BISS_ADDRESS_REGCFG, (HPUWORD)&sRegCfg, sizeof(BISS_REGCFG)/2);
    
    // Start Register Communication: at least 14 SCD cycles with CDM=0
    for(ubCycle=0;ubCycle<30;ubCycle++)
        FPGA_BISS_SET_START(dwBaseAddress) = TRUE;
    
    // Get Register Data
    BissReadParameter(dwBaseAddress, BISS_ADDRESS_REGDATA, &uwData, sizeof(UWORD));    
    
    return uwData;
}

//***************************************************************************
// Read MB101x Parameter

static void BissReadParameter(ULONG dwBaseAddress, UBYTE ubAddress, HPUWORD hpuwData, UBYTE ubNum)
{
    UBYTE nWord;
    for(nWord=0;nWord<ubNum;nWord++)
    {    
            // COMMAND ADDRESS[7:0]
        FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_ADDR) = ubAddress+(nWord<<1);
                
            // Start Reading
        FPGA_BISS_SET_READ(dwBaseAddress) = TRUE; 
        
            // Get Data
        hpuwData[nWord] = FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_DATA);
    }
}

//***************************************************************************
// Write MB101x parameter

static void BissWriteParameter(ULONG dwBaseAddress, UBYTE ubAddress, HPUWORD hpuwData, UBYTE ubNum)
{
    UBYTE nWord;
    
    for(nWord=0;nWord<ubNum;nWord++)
    {
            // COMMAND ADDRESS[7:0]
        FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_ADDR) = ubAddress+(nWord<<1);
    
            // Write Data
        FPGA_BASEOFF_16(dwBaseAddress, FPGA_BISS_DATA) = hpuwData[nWord]; 
                
            // Start Writing
        FPGA_BISS_SET_WRITE(dwBaseAddress) = TRUE; 
    }
}
#endif
#endif
