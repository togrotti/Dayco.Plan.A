/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : PacketCommand.c                                            */
/* Author      : Stefano Martino                                            */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

#ifdef _INFINEON_
#include <XE167F.H>
#include <intrins.h>
#endif
#include <stdlib.h>
#include <string.h>

#include "common\CommonDefines.h"

#ifndef _RD
#include "drive\AxM-E-Defines.h"
#else
#include "RemDisp-Defines.h"
#endif

#include "PacketCommand.h"

#ifndef _RD
#include "common\ProgramFlashHandler.h"
#ifdef _APP_XC
#include "common\SFlashStorage.h"
#include "common\SerialFlashHandler.h"
#endif
#endif

#include "system\MemoryLayout.h"
#include "common\SecurityCrypto.h"

#ifdef _AXX_SYSAPP
#include "system\SystemStatus.h"
#include "plc\AlPlcRuntimeConfUser.h"
#include "plc\Plc.h"
#else
#ifdef _APP_XC
#include "system\BootBlockStartup.h"
#endif
#endif

/////////////////////////////////////////////////////////////////////////////
//

#ifdef _APP_XC
#ifdef _AXX_SYSAPP
#ifdef _INFINEON_
#include "system\SysAppSFlashPartXC.h"
#else
#include "system\SysAppSFlashPartZYNQ.h"
#endif // _infineon_
#define _SYSAPPSTOR         (SFPART_SYSAPP_START)
#define _SYSAPPSTSZ         (SFPART_SYSAPP_SIZE)
#else
//extern UBYTE LB_SYSAPPSTOR_POINTER;
//extern UBYTE LB_SYSAPPSTOR_SIZE;
#define _SYSAPPSTOR         ((ULONG)LB_SYSAPPSTOR_POINTER)
#define _SYSAPPSTSZ         ((ULONG)LB_SYSAPPSTOR_SIZE)
#endif // _axx_sysapp
#endif // _app_xc

/////////////////////////////////////////////////////////////////////////////
//

#define PACKET_COMMAND_CMD_SYNCHRONIZE     0x00
#define PACKET_COMMAND_CMD_WRITE_MEMORY    0x01
#define PACKET_COMMAND_CMD_READ_MEMORY     0x02
#define PACKET_COMMAND_CMD_WRITE_FLASH     0x03
#define PACKET_COMMAND_CMD_ERASE_FLASH     0x04
#define PACKET_COMMAND_CMD_START_STREAM    0x06
#define PACKET_COMMAND_CMD_WRITE_STREAM    0x07
#define PACKET_COMMAND_CMD_END_STREAM      0x08


/////////////////////////////////////////////////////////////////////////////
//

#define MOVB( d, s )  { ((HPUBYTE)(d))[0] = ((HPUBYTE)(s))[0];  }

#define MOVW( d, s )  { ((HPUBYTE)(d))[0] = ((HPUBYTE)(s))[0];  \
                        ((HPUBYTE)(d))[1] = ((HPUBYTE)(s))[1];  }

#define MOVD( d, s )  { ((HPUBYTE)(d))[0] = ((HPUBYTE)(s))[0];  \
                        ((HPUBYTE)(d))[1] = ((HPUBYTE)(s))[1];  \
                        ((HPUBYTE)(d))[2] = ((HPUBYTE)(s))[2];  \
                        ((HPUBYTE)(d))[3] = ((HPUBYTE)(s))[3];  }




/////////////////////////////////////////////////////////////////////////////
//

SWORD ExecutePacketCommand( HPUBYTE hpubInpBuffer, HPUBYTE hpubOutBuffer )
{
  UBYTE ubCommand;
  ULONG ulAddrs;
  UWORD uwLength, uwCount;
  const MEMORY_SECTOR_INFO MEMORYLAYOUT_QUAL * psMemorySectorInfo;

  UBYTE  ubResult      = PACKET_COMMAND_ANS_ERROR;
  SWORD  swReplyLength = 0;

#ifdef _INFINEON_
  static UBYTE ubSecurityIVecs[ SECURITY_CRYPTO_MAX_KEYS ][ AES_BLOCK_SIZE ];
#endif
#ifdef _APP_XC
  UBYTE ubLength;
  static SFSTOR_WRSTATUS  sStrStatus={0ul};
#endif

  // Get Command
  ubCommand = hpubInpBuffer[ 0 ];
  hpubInpBuffer++;

  switch ( ubCommand )
  {
    // PACKET_COMMAND_CMD_SYNCHRONIZE
    case PACKET_COMMAND_CMD_SYNCHRONIZE :
      // Set Result
      ubResult = PACKET_COMMAND_ANS_READY;
      break;

    // PACKET_COMMAND_CMD_READ_MEMORY
    case PACKET_COMMAND_CMD_READ_MEMORY :
      // Get Address
      MOVD( &ulAddrs, hpubInpBuffer );
      hpubInpBuffer += sizeof( ULONG );
      // Get Length
      MOVW( &uwLength, hpubInpBuffer );
      hpubInpBuffer += sizeof( UWORD );

      // Retrieve Memory Sector Info
      psMemorySectorInfo = MemoryLayoutRetrieveSectorInfo( ulAddrs );
      // Requested Address not found or Reading not allowed
      if ( psMemorySectorInfo == NULL || !( psMemorySectorInfo->uwMemoryConfig & MEMORY_CONFIG_READ ) )
        break;

      // check max read size
      if ( uwLength > PACKET_COMMAND_MAX_READ_LENGTH )
        break;

#ifndef _RD
      if ( psMemorySectorInfo->uwMemoryConfig & MEMORY_CONFIG_SFLASH )
      {
          // Read Bytes from SFLASH and update out buffer address
          if( !SerialFlashReadBytes( ulAddrs, hpubOutBuffer, uwLength ))
            break;
          hpubOutBuffer=&hpubOutBuffer[uwLength];
      }
      else
#endif
        // Read Bytes from micro address space
        for ( uwCount = 0; uwCount < uwLength; uwCount++ )
        {
          MOVB( hpubOutBuffer, ulAddrs );
          ulAddrs++;
          hpubOutBuffer++;
        }

      // Set Reply Length
      swReplyLength = uwLength;
      // Set Result
      ubResult = PACKET_COMMAND_ANS_READY;
      break;

    // PACKET_COMMAND_CMD_WRITE_MEMORY
    case PACKET_COMMAND_CMD_WRITE_MEMORY :
      // Get Address
      MOVD( &ulAddrs,hpubInpBuffer );
      hpubInpBuffer += sizeof( ULONG );
      // Get Length
      MOVW( &uwLength, hpubInpBuffer );
      hpubInpBuffer += sizeof( UWORD );

      // Retrieve Memory Sector Info
      psMemorySectorInfo = MemoryLayoutRetrieveSectorInfo( ulAddrs );
      // Requested Address not found or Writing not allowed
      if ( psMemorySectorInfo == NULL || !( psMemorySectorInfo->uwMemoryConfig & MEMORY_CONFIG_WRITE ) )
        break;

      // Write Bytes
      for ( uwCount = 0; uwCount < uwLength; uwCount++ )
      {
        MOVB( ulAddrs, hpubInpBuffer );
        ulAddrs++; hpubInpBuffer++;
      }
      ubResult = PACKET_COMMAND_ANS_READY;
      break;

    // PACKET_COMMAND_CMD_ERASE_FLASH
    case PACKET_COMMAND_CMD_ERASE_FLASH :
#ifdef _AXX_SYSAPP
      if( !PlcLockForReload(FALSE) )
        break;
#endif
      // Get Address
      MOVD( &ulAddrs, hpubInpBuffer );
      if ((void*)ulAddrs == m_PlcCodeArea1)
      {
    	  memset((void*)ulAddrs, 0, ALPLCAREA_CODE_SIZE);
          ubResult = PACKET_COMMAND_ANS_READY;
      }
      else
      {
    	  // Retrieve Memory Sector Info
    	  psMemorySectorInfo = MemoryLayoutRetrieveSectorInfo( ulAddrs );
    	  // Requested Address not found or Erasing not allowed
    	  if ( psMemorySectorInfo == NULL || !( psMemorySectorInfo->uwMemoryConfig & MEMORY_CONFIG_ERASE ) )
    		  break;

#ifdef _INFINEON_
    	  // If Encrypted set IV
    	  if ( psMemorySectorInfo->uwCryptoKeyNum != SECURITY_CRYPTO_KEY_NONE && psMemorySectorInfo->uwCryptoKeyNum < SECURITY_CRYPTO_MAX_KEYS )
    		  SecurityCryptoAESInitialValue( psMemorySectorInfo->uwCryptoKeyNum, ubSecurityIVecs[ psMemorySectorInfo->uwCryptoKeyNum ] );
#endif

#ifdef _AXX_SYSAPP
    	  bSysStatProgramFlashWriting=TRUE;
#endif
    	  // Disable Interrupts
    	  DISABLE_IRQ();
    	  // Erase Sector : Internal Program FLASH
    	  if ( psMemorySectorInfo->uwMemoryConfig & MEMORY_CONFIG_PFLASH ) {
#ifdef _AXX_BOOTBLOCK
    		  SecurityFlashDisableWriteProtection( 0x55AA, 0xAA55 );
#endif
    		  if ( ProgramFlashErase( (void *)psMemorySectorInfo->ulLowerAddress, psMemorySectorInfo->ulUpperAddress - psMemorySectorInfo->ulLowerAddress ) == 0 )
    			  ubResult = PACKET_COMMAND_ANS_READY;
#ifdef _AXX_BOOTBLOCK
    		  SecurityFlashReEnableRdWrProtection();
#endif
#ifndef _RD
    		  // Erase Sector : External Serial FLASH
    	  } else if ( psMemorySectorInfo->uwMemoryConfig & MEMORY_CONFIG_SFLASH ) {
    		  if( SerialFlashEraseSector( psMemorySectorInfo->ulLowerAddress ) )
    			  ubResult = PACKET_COMMAND_ANS_READY;
#ifdef _AXX_BOOTBLOCK
    		  // if bootblock then always return OK, to comply with older controlboard
    		  // small sflash chip
    		  ubResult = PACKET_COMMAND_ANS_READY;
#endif
#endif
    	  }
    	  // Enable Interrupts
    	  RESTORE_IRQ();

    	  // Set Result
    	  ubResult = PACKET_COMMAND_ANS_READY;

#ifdef _AXX_SYSAPP
    	  bSysStatProgramFlashWriting=FALSE;
#endif
      }

      break;

    // PACKET_COMMAND_CMD_WRITE_FLASH
    case PACKET_COMMAND_CMD_WRITE_FLASH :
#ifdef _AXX_SYSAPP
      if( !PlcLockForReload(FALSE) )
        break;
#endif
      // Get Address
      MOVD( &ulAddrs, hpubInpBuffer );
      hpubInpBuffer += sizeof( ULONG );
      // Get Length
      MOVW( &uwLength, hpubInpBuffer );
      hpubInpBuffer += sizeof( UWORD );

      if ((uint8_t*)ulAddrs >= m_PlcCodeArea1 &&
    		  (uint8_t*)ulAddrs < (m_PlcCodeArea1 + ALPLCAREA_CODE_SIZE))
      {
    	  // Handle copy of PCL code to OCM
    	  memcpy((void*)ulAddrs, (const void*)hpubInpBuffer, uwLength);
    	  ubResult = PACKET_COMMAND_ANS_READY;
      }
      else
      {
#ifdef _AXX_SYSAPP
    	  PlcCheckAddressDwSourceCode(ulAddrs, uwLength);
#endif

    	  // Retrieve Memory Sector Info
    	  psMemorySectorInfo = MemoryLayoutRetrieveSectorInfo( ulAddrs );
    	  // Requested Address not found, Sector Overlapping or Writing not allowed
    	  if ( psMemorySectorInfo == NULL || ( ( ulAddrs + ( uwLength - 1 ) ) > psMemorySectorInfo->ulUpperAddress ) ||
    			  !( psMemorySectorInfo->uwMemoryConfig & MEMORY_CONFIG_WRITE ) )
    		  break;

#ifdef _INFINEON_
    	  // If Encrypted decrypt hpubInpBuffer::uwLength
    	  if ( psMemorySectorInfo->uwCryptoKeyNum != SECURITY_CRYPTO_KEY_NONE && psMemorySectorInfo->uwCryptoKeyNum < SECURITY_CRYPTO_MAX_KEYS ) {
    		  // Must be AES_BLOCK_SIZE multiple
    		  if ( ( uwLength % AES_BLOCK_SIZE != 0 ) )
    			  break;

    		  // decrypt all blocks
    		  for( uwCount = 0; uwCount < (uwLength / AES_BLOCK_SIZE); uwCount ++ )
    			  SecurityCryptoAESDecryptBlock( &hpubInpBuffer[ uwCount * AES_BLOCK_SIZE ], &hpubInpBuffer[ uwCount * AES_BLOCK_SIZE ], AES_BLOCK_SIZE,
    					  psMemorySectorInfo->uwCryptoKeyNum, ubSecurityIVecs[ psMemorySectorInfo->uwCryptoKeyNum ] );
    	  }
#endif

#ifdef _AXX_SYSAPP
    	  bSysStatProgramFlashWriting=TRUE;
#endif
    	  // Disable Interrupts
    	  DISABLE_IRQ();
    	  // Write Bytes : Internal Program FLASH
    	  if ( psMemorySectorInfo->uwMemoryConfig & MEMORY_CONFIG_PFLASH ) {
#ifdef _AXX_BOOTBLOCK
    		  SecurityFlashDisableWriteProtection( 0x55AA, 0xAA55 );
#endif
    		  if ( ProgramFlashWrite( (void *) ulAddrs, hpubInpBuffer, uwLength ) == 0 )
    			  ubResult = PACKET_COMMAND_ANS_READY;
#ifdef _AXX_BOOTBLOCK
    		  SecurityFlashReEnableRdWrProtection();
#endif
#ifndef _RD
    		  // Write Bytes : External Serial FLASH
    	  } else if ( psMemorySectorInfo->uwMemoryConfig & MEMORY_CONFIG_SFLASH ) {
    		  if( SerialFlashWriteBytes( ulAddrs, hpubInpBuffer, uwLength ) )
    			  ubResult = PACKET_COMMAND_ANS_READY;
#ifdef _AXX_BOOTBLOCK
    		  // if bootblock then always return OK, to comply with older controlboard
    		  // small sflash chip
    		  ubResult = PACKET_COMMAND_ANS_READY;
#endif
#endif
    	  }
    	  // Enable Interrupts
    	  RESTORE_IRQ();

#ifdef _AXX_SYSAPP
    	  bSysStatProgramFlashWriting=FALSE;
#endif
      }
      break;

#ifdef _APP_XC
    // PACKET_COMMAND_CMD_START_STREAM
    case PACKET_COMMAND_CMD_START_STREAM :
#ifdef _AXX_SYSAPP
      if( !PlcRequestStandby() )
        break;
      if( !PlcLockForReload(FALSE) )
        break;
#endif
      // Get Length
      MOVB( &ubLength, hpubInpBuffer );
      hpubInpBuffer += sizeof( UBYTE );

      if(SFStor_WriteBegin(&sStrStatus, _SYSAPPSTOR, _SYSAPPSTSZ)==SFSTOR_STR_OK)
        ubResult = PACKET_COMMAND_ANS_READY;
      break;

    // PACKET_COMMAND_CMD_WRITE_STREAM
    case PACKET_COMMAND_CMD_WRITE_STREAM :
      // Get Length
      MOVB( &ubLength, hpubInpBuffer );
      hpubInpBuffer += sizeof( UBYTE );

      if(SFStor_WriteData(&sStrStatus, hpubInpBuffer, ubLength)==SFSTOR_STR_OK)
        ubResult = PACKET_COMMAND_ANS_READY;
      break;

    // PACKET_COMMAND_CMD_END_STREAM
    case PACKET_COMMAND_CMD_END_STREAM :
      // Get Length
      MOVB( &ubLength, hpubInpBuffer );
      hpubInpBuffer += sizeof( UBYTE );

      if(SFStor_WriteEnd(&sStrStatus)==SFSTOR_STR_OK)
      {
        ubResult = PACKET_COMMAND_ANS_READY;
#ifdef _AXX_BOOTBLOCK
#ifdef _APP_XC
        bBootBlockDataStreamValid = TRUE;
#endif
#endif
      }
      break;
#endif

    // UNKNOWN COMMAND
    default :
      ubResult = PACKET_COMMAND_ANS_ERROR;
      break;

  }

  // Copy Result to output buffer
  *hpubOutBuffer = ubResult;

  // Return Length
  return swReplyLength + 1;
}

