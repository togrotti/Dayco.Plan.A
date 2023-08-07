/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : FpgaHandler.c                                              */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "common\CommonDefines.h"
#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "drive\AxM-E-Defines.h"
#include "system\SysAppSFlashPartition.h"
#include "drive\HardwareConfig.h"
//#include "SysAppFpgaList.h"
//#include "SFlashStorage.h"
//#include "AppIdentTypes.h"

#include "fpga\FpgaHandler.h"
#include "fpga\FpgaHandlerRT.h"
//#include "MemoryTest.h"

#include "common\SecurityBlock.h"

/////////////////////////////////////////////////////////////////////////////
//

#define FPGA_MOREDATATOCOMPLETE     2000

/////////////////////////////////////////////////////////////////////////////
//

#define FPGA_CFGONSTART_CK_LOW      1
#define FPGA_CFGONSTART_CK_HIGH     2
#define FPGA_CFGONSTART_CK_REF      3
#define FPGA_CFGONSTART_CK_SWITCH   4

#define RGOF(reg,offset)            (*((unsigned int volatile  *)((ULONG)&(reg)+(ULONG)(offset))))

/////////////////////////////////////////////////////////////////////////////
//

#ifndef _HW_DC
#define _FPGA_TYPE                  (sGlbControlBoardParameters.sStd.uwFpgaType)
#else
#define _FPGA_TYPE                  (sGlbControlBoardParameters.sUniv.uwFpgaType)
#endif


/////////////////////////////////////////////////////////////////////////////
//

#ifndef _HW_DC
static BOOL HwConfigMatch(LOGICAL_BLOCK_HEADER * psBlockHeader, SWORD * pswOption);
static void HwPreBoot(SWORD swOption);
static void HwBoot(SWORD swOption);
#endif

/////////////////////////////////////////////////////////////////////////////
//

SWORD FpgaInit( void )
{
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
//

#ifndef _APP_XC
static void SerialTXU0C1( UWORD uwData )
{
  // if tx buffer full then wait
  while(U0C1_TRBSRL&0x1000);
  U0C1_IN00=uwData;
}
#endif

/////////////////////////////////////////////////////////////////////////////
//

static void SerialTXU2C1( UWORD uwData )
{
  // if tx buffer full then wait
//  while(U2C1_TRBSRL&0x1000);
//  U2C1_IN00=uwData;
}

/////////////////////////////////////////////////////////////////////////////
//

SWORD FpgaLoad( ULONG  * ulLoadedBytes, void (* yield)(void) )
{
#ifdef _INFINEON_
  UBYTE ubDataBuffer[ SFSTOR_DESTBUFSIZE ];
  SFSTOR_STATUS sStorStatus;
  LOGICAL_BLOCK_HEADER * psHeader;
  const HWCONF_FPGAALLOWED  * pFpgaAllow;
  ULONG ulSentMoreData=FPGA_MOREDATATOCOMPLETE;
  UWORD uwCount, uwDelay;
#ifndef _HW_DC
  SWORD swBootOpt;
#endif
  SWORD swRetVal;
#ifndef _APP_XC
  BOOL bRevLT4xx=((sGlbControlBoardParameters.sProductInfo.uwProductRev/100)<4);
#endif
  BOOL bAllDataSent,bInvHwOptReq=FALSE;

    // Seek init
#ifdef _APP_XC
  psHeader=SFStor_SeekInit(&sStorStatus, SFPART_SYSAPP_START, SFPART_SYSAPP_SIZE);
#else
  psHeader=SFStor_SeekInit(&sStorStatus, SFPART_FPGA_CFG_START, SFPART_FPGA_CFG_SIZE);
#endif
  if(psHeader==NULL)
    return FPGA_LOAD_INVALIDDATABLOCK;

  for(;;)
  {
      // we're looking for FPGA standard
    if(psHeader->sParameters.uwApplicatType==IDENT_FPGA_STANDARD)
    {
        // check allowance of block found
      pFpgaAllow=hpsHwFpgaAllowed;
      while(pFpgaAllow->chType)
      {
          // ref. definition of HWPRM_CTRLBRD_FPGA_*
        UWORD uwType = ((UWORD)pFpgaAllow->ubFpgaType<<12)+((UWORD)pFpgaAllow->ubSize<<4)+pFpgaAllow->ubSpeed;

          // if all features match then the block found is an FPGA compatible with firmware
        if( pFpgaAllow->chType   == psHeader->sParameters.uwVersionMinor/256 &&
            pFpgaAllow->ubTgtRev == psHeader->sParameters.uwVersionMinor%256 &&
            uwType               == psHeader->sParameters.uwVersionMajor &&
            pFpgaAllow->uwPCode  == sGlbControlBoardParameters.sProductInfo.uwProductCode
#ifndef _APP_DEBUG
            && pFpgaAllow->uwBuild  == psHeader->sParameters.uwBuildNumber
#endif
            )
          break;

          // yield external concurrent processing for status update
        if(yield)
          (*yield)();

        pFpgaAllow++;
      }

        // if block found is allowed then check if matching with hardware configuration
      if( pFpgaAllow->chType )
#ifndef _HW_DC
        if( HwConfigMatch(psHeader, &swBootOpt) )
#endif
        {
#ifdef _HW_DC
            // track valid FPGA option mask
          sGlbOptAvailable.ulFPGAOpt = psHeader->sParameters.ulOptionMask;
#endif
            // here check user option requested, in order to send specific
            // boot code in case of mismatching
          if((psHeader->sParameters.ulOptionMask & sGlbOptReq.ulFPGAOpt) == sGlbOptReq.ulFPGAOpt)
          {
            bInvHwOptReq=FALSE;
            break;
          }
          else
            bInvHwOptReq=TRUE;
        }
    }

      // seek next
    psHeader=SFStor_SeekNext(&sStorStatus);
    if(psHeader==NULL)
#ifdef _HW_DC
      if(bInvHwOptReq)
        return FPGA_LOAD_NOHWOPTREQMATCH;
      else
#endif
        return FPGA_LOAD_NOHWMATCHINGFOUND;
  }

  // track down build number for later checking
  tHwFpgaConfig.uwBuild = psHeader->sParameters.uwBuildNumber;

  // Clear Loaded Bytes
  *ulLoadedBytes = 0;

  // Assert nCONFIG
  XE167_FPGA_CONFIG = 0;

  // tCFG min 500nS
  timer_wait(uwSysTimers100ns, 10);

  // Release nCONFIG
  XE167_FPGA_CONFIG = 1;

  // prepare fast sequential read from serial flash
  if( SFStor_StreamBegin(&sStorStatus, ubDataBuffer) != SFSTOR_STR_OK )
    return FPGA_LOAD_INVALIDDATABLOCK;

  // Wait nStatus : tCF2ST1 max 230uS
  for ( uwDelay = 0; uwDelay < 250; uwDelay++ ) {
    if ( XE167_FPGA_STATUS )
      break;
    timer_wait(uwSysTimers100ns, 10);
  }

  // Error if nStatus low after tCF2ST1
  if ( !XE167_FPGA_STATUS )
  {
    SFStor_StreamEnd(&sStorStatus, ubDataBuffer, TRUE);
    return FPGA_LOAD_ERROR_STATUS_A;
  }

  // Send configuration until CONF_DONE and all data verified
  bAllDataSent=FALSE;
  while ( !(XE167_FPGA_CONFDONE && bAllDataSent) ) {

    // if more data to read
    if ( !bAllDataSent )
    {
      // get data from flash storage
      swRetVal = SFStor_StreamGetData(&sStorStatus, ubDataBuffer);

      // check return code
      if(swRetVal==SFSTOR_STR_DATAINVALID)
      {
        SFStor_StreamEnd(&sStorStatus, ubDataBuffer, TRUE);
        return FPGA_LOAD_INVALIDDATABLOCK;
      }
      else if(swRetVal==SFSTOR_STR_NOMOREDATA)
      {
        swRetVal=SFStor_StreamEnd(&sStorStatus, ubDataBuffer, FALSE);
#ifndef _APP_DEBUG
        if(swRetVal!=SFSTOR_STR_OK)
          return FPGA_LOAD_INVALIDDATABLOCK;
#endif

        bAllDataSent=TRUE;
      }
    }
    // if no more data send dummy data required from FPGA to
    // complete init
    else
      if( ulSentMoreData >= sizeof( ubDataBuffer ) )
        ulSentMoreData -= sizeof( ubDataBuffer );
      else
        return FPGA_LOAD_ERROR_CONFDONE;

    // send to FPGA if not already configured and if in the right range
    if(!XE167_FPGA_CONFDONE)
      for( uwCount = 0; uwCount < sizeof(ubDataBuffer); uwCount+=sizeof(UWORD) )
      {
#ifndef _APP_XC
        if(bRevLT4xx)
          SerialTXU0C1( *((UWORD *)&ubDataBuffer[uwCount]) );
        else
#endif
          SerialTXU2C1( *((UWORD *)&ubDataBuffer[uwCount]) );

        (*ulLoadedBytes)+=sizeof(UWORD);
      }

    // yield external concurrent processing for status update
    if(yield)
      (*yield)();

    // Error during configuration
    if ( !XE167_FPGA_STATUS )
    {
      if(!bAllDataSent)
        SFStor_StreamEnd(&sStorStatus, ubDataBuffer, TRUE);
      return FPGA_LOAD_ERROR_STATUS_B;
    }
  }

  // release serial flash sequential read
  if(!bAllDataSent)
  {
#ifndef _APP_DEBUG
    swRetVal=SFStor_StreamEnd(&sStorStatus, ubDataBuffer, FALSE);
    if(swRetVal!=SFSTOR_STR_OK)
      return FPGA_LOAD_INVALIDDATABLOCK;
#else
    SFStor_StreamEnd(&sStorStatus, ubDataBuffer, FALSE);
#endif
  }

  // Wait INIT_DONE : tCD2UM max 650uS
  for ( uwDelay = 0; uwDelay < 700; uwDelay++ ) {
    if ( XE167_FPGA_INITDONE )
      break;
    timer_wait(uwSysTimers100ns, 10);
  }
  // Error if INIT_DONE low after tCD2UM
  if ( !XE167_FPGA_INITDONE )
    return FPGA_LOAD_ERROR_INITDONE;

  // Boot based on hw options
#ifndef _HW_DC
  HwBoot(swBootOpt);
#endif

#else
  // track down build number for later checking
  tHwFpgaConfig.uwBuild = FPGA_BUILD_NUMBER;
#endif

  // preset registers
  FPGA_RESET = 1;
  timer_wait(uwSysTimers100ns, 5);
  FPGA_RESET = 0;

  return FPGA_LOAD_SUCCESSFULLY;
}


/////////////////////////////////////////////////////////////////////////////
//

SWORD FpgaTest( void )
{
  volatile UWORD  * hpuwTestAddress = (volatile UWORD  *)FPGA_MEMORYTEST_BASE;

  // Check Build Type & Build Number
  if ( FPGA_BUILD_TYPE   == 0x0000 || FPGA_BUILD_TYPE   == 0xFFFF ||
       FPGA_BUILD_NUMBER == 0x0000 || FPGA_BUILD_NUMBER == 0xFFFF )
    return FPGA_TEST_ERROR_BUILD_TYPE;

  // Perform Memory Test using three FPGA consecutive registers.

  // Test the data bus
//  if ( MemoryTestDataBus( &hpuwTestAddress[0] ) != 0 ||
//       MemoryTestDataBus( &hpuwTestAddress[2] ) != 0 ||
//       MemoryTestDataBus( &hpuwTestAddress[4] ) != 0 )
//    return FPGA_TEST_ERROR_DATA_BUS;
//
//  // Test the address bus
//  if ( MemoryTestAddressBus( hpuwTestAddress, 6 ) != NULL )
//    return FPGA_TEST_ERROR_ADRS_BUS;
//
//  // Test the integrity of the device
//  if ( MemoryTestDevice( hpuwTestAddress, 6 ) != NULL )
//    return FPGA_TEST_ERROR_DEVICE;

  // Clear FPGA Registers used for test
  hpuwTestAddress[0] = hpuwTestAddress[2] = hpuwTestAddress[4] = 0x0000;

  return FPGA_TEST_SUCCESSFULLY;
}

/////////////////////////////////////////////////////////////////////////////
//

void FpgaReset( void )
{
  // Assert nCONFIG
#ifdef _INFINEON_
  XE167_FPGA_CONFIG = 0;
#else
  // Xil_Out32(FPGA_RST_CTRL_REG,FPGA_RST_MASK);
#endif
}

/////////////////////////////////////////////////////////////////////////////
//

void FpgaSafeUnLock( UWORD uwServicePeriod )
{
  // reset safelock section
  FPGA_SAFE_LOCK=FPGA_SAFE_LOCK_TOKEN_RESET;

  // write WDT service period (if zero then disabled)
  FPGA_WDT_PERIOD=uwServicePeriod;

  // reset key
  uwFpgaWdtKey=FPGA_WDT_KEY=0;

  // then write the sequence to enable power sections (and start WDT)
  FPGA_SAFE_LOCK=FPGA_SAFE_LOCK_TOKEN_0;
  FPGA_SAFE_LOCK=FPGA_SAFE_LOCK_TOKEN_1;
}

/////////////////////////////////////////////////////////////////////////////
//

void FpgaSafeLock( void )
{
  // just write 0 and power sections will be locked
  FPGA_SAFE_LOCK=0;
}

/////////////////////////////////////////////////////////////////////////////
//

BOOL FpgaCRCService( void )
{
#ifdef _INFINEON_
    BOOL bCrcError=TRUE;
        // check three times to be noise free
    bCrcError&=XE167_FPGA_CRCERROR;
    bCrcError&=XE167_FPGA_CRCERROR;
    bCrcError&=XE167_FPGA_CRCERROR;

        // if on, FPGA fail (and force FPGA reset)
    if(bCrcError)
    {
        FpgaReset();
        return FALSE;
    }
#endif
    return TRUE;
}

/////////////////////////////////////////////////////////////////////////////
//

BOOL FpgaIdentVerify( void )
{
    // check for type
  if((tHwFpgaConfig.uwType&0x0FFF)!=(FPGA_BUILD_TYPE&0x0FFF))
    return FALSE;

    // check for application
#ifndef _HW_DC
  if(tHwFpgaConfig.uwAppCode!=FPGA_BUILD_APP)
    return FALSE;
#endif

    // check build
#ifndef _APP_DEBUG
  if(tHwFpgaConfig.uwBuild!=FPGA_BUILD_NUMBER)
    return FALSE;
#endif

    // setup for right controlboard hw revision
#ifndef _HW_DC
#ifndef _APP_XC
  if(sGlbControlBoardParameters.sProductInfo.uwProductRev >= 200)
    FPGA_CONTROLBOARD_REV2=TRUE;
  else
    FPGA_CONTROLBOARD_REV2=FALSE;
#else
  if(sGlbControlBoardParameters.sProductInfo.uwProductRev < 600)
    FPGA_CONTROLBOARD_LT_REV6=TRUE;
  else
    FPGA_CONTROLBOARD_LT_REV6=FALSE;
#endif
#endif

  return TRUE;
}

/////////////////////////////////////////////////////////////////////////////
//

#ifndef _HW_DC
static BOOL HwConfigMatch(LOGICAL_BLOCK_HEADER * psBlockHeader, SWORD * pswOption)
{
    UWORD uwTgtRev=psBlockHeader->sParameters.uwVersionMinor%256;

        // check matching FPGA code
    if(psBlockHeader->sParameters.uwVersionMajor!=_FPGA_TYPE)
        return FALSE;

        // check for 10C8
    if(_FPGA_TYPE==HWPRM_CTRLBRD_FPGA_EP3C10F256C8)
    {
            // if 10C8 and rev 3.xx then use asynchronous bus
        if((sGlbControlBoardParameters.sProductInfo.uwProductRev/100)==3)
        {
            if(uwTgtRev==1)
            {
                *pswOption=FPGA_CFGONSTART_CK_REF;
                HwPreBoot(*pswOption);
                return TRUE;
            }
        }
            // if 10C8 on rev 3.xx hw patched then use synchronous bus
            // with clocking from M1
        else if((sGlbControlBoardParameters.sProductInfo.uwProductRev/100)==103)
        {
            if(uwTgtRev==3)
            {
                *pswOption=FPGA_CFGONSTART_CK_LOW;
                HwPreBoot(*pswOption);
                return TRUE;
            }
        }
            // if 10C8 and rev 1.xx or 4.xx then use synchronous bus
        else
        {
            if(uwTgtRev==2)
            {
                *pswOption=FPGA_CFGONSTART_CK_LOW;
                HwPreBoot(*pswOption);
                return TRUE;
            }
        }
    }
        // other types must have no special type
    else
        if(uwTgtRev==0)
        {
                // autoselect after reset
            *pswOption=FPGA_CFGONSTART_CK_SWITCH;
            HwPreBoot(*pswOption);
            return TRUE;
        }

    return FALSE;
}

/////////////////////////////////////////////////////////////////////////////
//

static void HwPreBoot(SWORD swOption)
{
    switch(swOption)
    {
            // keep LOW for FPGA clock from E2 (rev 1.xx and 4.xx)
        case FPGA_CFGONSTART_CK_LOW:
#ifdef _INFINEON_
            if((sGlbControlBoardParameters.sProductInfo.uwProductRev/100)>=4)
            {
                P2_OUT_P8 = FALSE;
                P2_IOCR08 = 0x0080;
            }
            else
            {
                P7_OUT_P1 = FALSE;
                P7_IOCR01 = 0x0080;
            }
#endif
            break;

            // keep HIGH for FPGA clock from B9 (rev 3.xx)
        case FPGA_CFGONSTART_CK_HIGH:
#ifdef _INFINEON_
            if((sGlbControlBoardParameters.sProductInfo.uwProductRev/100)>=4)
            {
                P2_OUT_P8 = TRUE;
                P2_IOCR08 = 0x0080;
            }
            else
            {
                P7_OUT_P1 = TRUE;
                P7_IOCR01 = 0x0080;
            }
#endif
            break;

            // set internal clock as external reference clock or
        case FPGA_CFGONSTART_CK_REF:
            // set internal clock as clock switch helper
        case FPGA_CFGONSTART_CK_SWITCH:
#ifdef _INFINEON_
            SCU_EXTCON= 0x0001;
            if((sGlbControlBoardParameters.sProductInfo.uwProductRev/100)>=4)
                P2_IOCR08 = 0x00A0;
            else
                P7_IOCR01 = 0x0090;
#endif
            break;

        default:
            assert(FALSE);
    }
}

/////////////////////////////////////////////////////////////////////////////
//

static void HwBoot(SWORD swOption)
{
    UWORD uwDelay;

    switch(swOption)
    {
        case FPGA_CFGONSTART_CK_LOW:
        case FPGA_CFGONSTART_CK_HIGH:
        case FPGA_CFGONSTART_CK_REF:
                // Delay 10mS
            for ( uwDelay = 0; uwDelay < 10; uwDelay++ )
                timer_wait(uwSysTimers100ns, 10000);
            break;

        case FPGA_CFGONSTART_CK_SWITCH:
                // Delay 10mS
            for ( uwDelay = 0; uwDelay < 10; uwDelay++ )
                timer_wait(uwSysTimers100ns, 10000);

#ifdef _INFINEON_
                // remove helper clocking
            SCU_EXTCON= 0x0000;
            if((sGlbControlBoardParameters.sProductInfo.uwProductRev/100)>=4)
                P2_IOCR08 = 0x0080;
            else
                P7_IOCR01 = 0x0080;
            break;
#endif

        default:
            assert(FALSE);
    }
}
#endif
