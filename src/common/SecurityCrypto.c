/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright 漏 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SecurityCrypto.c                                           */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

/////////////////////////////////////////////////////////////////////////////
//
#include <string.h>

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "SecurityCrypto.h"
#include "common\ProgramFlashHandler.h"

#ifdef _INFINEON_
#ifndef _RD
#ifndef _APP_XC
#include "SecurityKeysAxXXE.h"
#else
#include "Keys\SecurityKeysAxXXC.h"
#endif
#else
#include "SecurityKeysRD.h"
#endif

/////////////////////////////////////////////////////////////////////////////
//

void SecurityCryptoAESInitialValue( UWORD uwKeyNum, UBYTE  * pubIVec )
{
  if ( uwKeyNum < SECURITY_CRYPTO_MAX_KEYS )
    memcpy( pubIVec, &ubSecurityIVs[ uwKeyNum ], AES_BLOCK_SIZE );
}

/////////////////////////////////////////////////////////////////////////////
//

void SecurityCryptoAESDecryptBlock( UBYTE  * pubInp, UBYTE  * pubOut, ULONG ulLength, UWORD uwKeyNum, UBYTE  * pubIVec )
{
  if ( uwKeyNum < SECURITY_CRYPTO_MAX_KEYS )
    AES_cbc_decrypt( pubInp, pubOut, ulLength, &sSecurityKeys[ uwKeyNum ], pubIVec );
}



/////////////////////////////////////////////////////////////////////////////
//

void SecurityComputeMD5HashInit( MD5_CTX  * c )
{
  MD5_Init( c );
}

/////////////////////////////////////////////////////////////////////////////
//

void SecurityComputeMD5HashUpdate( MD5_CTX  * c, const void  * data, size_t len )
{
  MD5_Update( c, data, len );
}

/////////////////////////////////////////////////////////////////////////////
//

void SecurityComputeMD5HashFinal( unsigned char  * md, MD5_CTX  * c )
{
  MD5_Final( md, c );
}

#endif

/////////////////////////////////////////////////////////////////////////////
//

void SecurityFlashDisableWriteProtection( unsigned short chka, unsigned short chkb )
{
#ifdef _INFINEON_
  if ( ( chka ^ chkb ) == 0xFFFF )
    ProgramFlashDisableWriteProtection( uwFlashSecurityKey[ 0 ], uwFlashSecurityKey[ 1 ], uwFlashSecurityKey[ 2 ], uwFlashSecurityKey[ 3 ] );
#endif
}

void SecurityFlashDisableReadProtection( unsigned short chka, unsigned short chkb )
{
#ifdef _INFINEON_
  if ( ( chka ^ chkb ) == 0xFFFF )
    ProgramFlashDisableReadProtection( uwFlashSecurityKey[ 0 ], uwFlashSecurityKey[ 1 ], uwFlashSecurityKey[ 2 ], uwFlashSecurityKey[ 3 ] );
#endif
}

/////////////////////////////////////////////////////////////////////////////
//

void SecurityFlashReEnableRdWrProtection( void )
{
#ifdef _INFINEON_
  ProgramFlashReEnableReadWriteProtection();
#endif
}

/////////////////////////////////////////////////////////////////////////////
//

#ifdef _INFINEON_
const UWORD  * SecurityFlashGetSecurityKeys( void )
{
    return uwFlashSecurityKey;
}
#endif
