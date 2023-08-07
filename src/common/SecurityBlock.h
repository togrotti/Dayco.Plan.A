/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : SecurityBlock.h                                            */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _SECURITY_BLOCK_H
 #define _SECURITY_BLOCK_H

#include <stddef.h>

#include "common\CommonDefines.h"

#ifdef _INFINEON_
#include "md5.h"
#endif

/////////////////////////////////////////////////////////////////////////////
//

#define LOGICAL_BLOCK_PARAMS_SIZE    32


/////////////////////////////////////////////////////////////////////////////
//

#ifdef _INFINEON_
typedef void (far * LOGICAL_BLOCK_CODE_ENTRY_POINT)( void );
#else
typedef void (* LOGICAL_BLOCK_CODE_ENTRY_POINT)( void );
#endif

/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
  struct
  {
    UBYTE   ubHeadFootType : 4; // Logical Block Header/Footer Type
    UBYTE   ubSignatureKey : 2; // nn = Signature Encryption Key Number
    UBYTE   ubSignatureEnc : 1; //  0 = Signature not Encrypted, 1 = Signature Encrypted
    UBYTE   ubHeaderFooter : 1; //  0 = Logical Block Header, 1 = Logical Block Footer
  } ubBlockType;

  UBYTE   ubBlockVers;

  UWORD   uwApplicatType;
  UWORD   uwVersionMajor;
  UWORD   uwVersionMinor;
  ULONG   ulStartAddress;
  ULONG   ulTotalByteLen;
  ULONG   ulEntryAddress;
  ULONG   ulNextBlockPtr;
  UWORD   uwBuildNumber;
  ULONG   ulOptionMask;

  UWORD   uwBlockCheck;

} LOGICAL_BLOCK_PARAMS;


/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
#ifdef _INFINEON_
  UBYTE                 ubSignature[ MD5_DIGEST_LENGTH ];
#endif
  CHARS                 chFreeDescr[ 16 ];
  LOGICAL_BLOCK_PARAMS  sParameters;

} LOGICAL_BLOCK_HEADER;


/////////////////////////////////////////////////////////////////////////////
//

typedef struct
{
  LOGICAL_BLOCK_PARAMS  sParameters;
  CHARS                 chFreeDescr[ 16 ];
#ifdef _INFINEON_
  UBYTE                 ubSignature[ MD5_DIGEST_LENGTH ];
#endif
} LOGICAL_BLOCK_FOOTER;



/////////////////////////////////////////////////////////////////////////////
//

#define CHECK_LOGICAL_BLOCK_HEADER              1
#define CHECK_LOGICAL_BLOCK_FOOTER              2


/////////////////////////////////////////////////////////////////////////////
//

#define CHECK_LOGICAL_BLOCK_SUCCESSFULLY        0
#define CHECK_LOGICAL_BLOCK_ERROR_CHECKCODE    -1
#define CHECK_LOGICAL_BLOCK_ERROR_SIGNATURE    -2


/////////////////////////////////////////////////////////////////////////////
//

// Logical Block
SWORD SecurityCheckLogicalBlock( HPVOID hpvBlockAddress );

// Logical Block Chunked
#ifdef _INFINEON_
SWORD SecurityCheckLogicalBlockInit( HPVOID hpvBlockAddress, UBYTE ubBlockType, ULONG * hpulStartAddress, ULONG * hpulTotalByteLen, MD5_CTX * hpCTX );
SWORD SecurityCheckLogicalBlockUpdate( HPVOID hpvStartAddress, ULONG ulTotalByteLen, MD5_CTX * hpCTX );
SWORD SecurityCheckLogicalBlockFinal( HPVOID hpvBlockAddress, UBYTE ubBlockType, MD5_CTX * hpCTX );
#endif

#endif // _SECURITY_BLOCK_H
