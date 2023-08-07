/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SecurityCrypto.h                                           */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _SECURITY_CRYPTO_H
 #define _SECURITY_CRYPTO_H

#include <stddef.h>

#include "common\CommonDefines.h"

#ifdef _INFINEON_
#include "common\aes.h"
#include "common\md5.h"
#endif

/////////////////////////////////////////////////////////////////////////////
//

#define SECURITY_CRYPTO_MAX_KEYS    4

/////////////////////////////////////////////////////////////////////////////
//

#define SECURITY_CRYPTO_KEY_NONE    (UWORD)-1
#define SECURITY_CRYPTO_KEY_USR0    0
#define SECURITY_CRYPTO_KEY_USR1    1
#define SECURITY_CRYPTO_KEY_USR2    2
#define SECURITY_CRYPTO_KEY_USR3    3


/////////////////////////////////////////////////////////////////////////////
//

#ifdef _INFINEON_
// AES Decryption
void SecurityCryptoAESInitialValue( UWORD uwKeyNum, UBYTE  * pubIVec );
void SecurityCryptoAESDecryptBlock( UBYTE  * pubInp, UBYTE  * pubOut, ULONG ulLength, UWORD uwKeyNum, UBYTE  * pubIVec );

// MD5 Hash
void SecurityComputeMD5HashInit( MD5_CTX  * c );
void SecurityComputeMD5HashUpdate( MD5_CTX  * c, const void  * data, size_t len );
void SecurityComputeMD5HashFinal( unsigned char  * md, MD5_CTX  * c );

const UWORD  * SecurityFlashGetSecurityKeys( void );
#endif

// Flash Security
void SecurityFlashDisableWriteProtection( unsigned short chka, unsigned short chkb );
void SecurityFlashDisableReadProtection( unsigned short chka, unsigned short chkb );
void SecurityFlashReEnableRdWrProtection( void );

#endif // _SECURITY_CRYPTO_H
