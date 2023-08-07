/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CommonParamDB.c                                            */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Definition of common parameter database, in order to have  */
/*               a common interface between all fieldbus and internal       */
/*               parameters                                                 */
/*                                                                          */
/****************************************************************************/

//#include <intrins.h>
#include <stdlib.h>
#include <string.h>
#include "common\CommonParamDB.h"
#include "common\CommonUtility.h"
#include "system\SystemAlarms.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

//***************************************************************************
// Locals

    // look at COMMONPARAMDB_TYPE_* definitions
static const UWORD puwDataSizes[]=
{
    sizeof(UBYTE),sizeof(SBYTE),
    sizeof(UWORD),sizeof(SWORD),
    sizeof(ULONG),sizeof(SLONG),
    sizeof(UQWRD),sizeof(SQWRD),
    sizeof(FLOAT),sizeof(DOUBL),
    0,0,
    0,0,
};

//***************************************************************************
// Table checking, just for debug purpose

#ifdef _APP_DEBUG
BOOL ComParDBCheckTable(void)
{
    UWORD uwIndxCount;
    void  * pvGeneric;
    
        // Check Index Contiguity and Duplicates for fast search
    for ( uwIndxCount = 0; uwIndxCount < uwCommonParamCount-1; uwIndxCount++ ) 
        assert ( psCommonParamTable[ uwIndxCount ].uwIndex < psCommonParamTable[ uwIndxCount + 1 ].uwIndex );
    
        // Check Bit Number Range (0-15 if BITWORD, 0-31 if BITDWORD)
    for ( uwIndxCount = 0; uwIndxCount < uwCommonParamCount; uwIndxCount++ ) 
    {
        if ( (psCommonParamTable[ uwIndxCount ].ubType&COMMONPARAMDB_TYPE_TYPE_MASK) == COMMONPARAMDB_TYPE_BITW )
            assert ( ! (psCommonParamTable[ uwIndxCount ].uwNElements == 1 && psCommonParamTable[ uwIndxCount ].swMoreType > 15) );

        if ( (psCommonParamTable[ uwIndxCount ].ubType&COMMONPARAMDB_TYPE_TYPE_MASK) == COMMONPARAMDB_TYPE_BITD )
            assert ( ! (psCommonParamTable[ uwIndxCount ].uwNElements == 1 && psCommonParamTable[ uwIndxCount ].swMoreType > 31) );
    }
    
        // Check Bit Number for Bits Array (Must be -1)
    for ( uwIndxCount = 0; uwIndxCount < uwCommonParamCount; uwIndxCount++ ) 
        if ( psCommonParamTable[ uwIndxCount ].ubType == COMMONPARAMDB_TYPE_BITW || psCommonParamTable[ uwIndxCount ].ubType == COMMONPARAMDB_TYPE_BITD )
        {
            if ( psCommonParamTable[ uwIndxCount ].uwNElements != 1 )
            {
                assert( psCommonParamTable[ uwIndxCount ].swMoreType == -1 );
            }
            else
            {
                assert( ! (psCommonParamTable[ uwIndxCount ].ubType == COMMONPARAMDB_TYPE_BITW && psCommonParamTable[ uwIndxCount ].swMoreType >=16 ));
                assert( ! (psCommonParamTable[ uwIndxCount ].ubType == COMMONPARAMDB_TYPE_BITD && psCommonParamTable[ uwIndxCount ].swMoreType >=32 ));
                assert( ! (psCommonParamTable[ uwIndxCount ].swMoreType <0 ));
            }
        }

    for ( uwIndxCount = 0; uwIndxCount < uwCommonParamCount; uwIndxCount++ )
    {
            // Check for not simultaneously hook type spec flags
        if(psCommonParamTable[ uwIndxCount ].ubFlags & COMMONPARAMDB_FLAG_HOOK)
            assert ((psCommonParamTable[ uwIndxCount ].ubFlags & (COMMONPARAMDB_FLAG_UMCONV|COMMONPARAMDB_FLAG_VALIDATE)) == 0 );

            // if not hook then NULL data pointer is denied
        assert ( (psCommonParamTable[ uwIndxCount ].ubFlags & COMMONPARAMDB_FLAG_HOOK) || psCommonParamTable[ uwIndxCount ].hpvData!=NULL );

        // Check data types in the range
        assert ( ! (psCommonParamTable[ uwIndxCount ].ubType == 0 || (psCommonParamTable[ uwIndxCount ].ubType&COMMONPARAMDB_TYPE_TYPE_MASK) > COMMONPARAMDB_BASETYPE_MAXTYPE ));

        // Check if UMCONV then datasize must be fixed
        assert ( ! ((psCommonParamTable[ uwIndxCount ].ubFlags&COMMONPARAMDB_FLAG_UMCONV) && puwDataSizes[(psCommonParamTable[ uwIndxCount ].ubType&COMMONPARAMDB_TYPE_TYPE_MASK)-1] == 0 ));

            // Check if CONST then array is not possible
        assert ( ! ((psCommonParamTable[ uwIndxCount ].ubType&COMMONPARAMDB_TYPE_CONST_MASK) && psCommonParamTable[ uwIndxCount ].uwNElements!=1) );

            // if UMCONV pointer must be  (for fast processing)
        if ( psCommonParamTable[ uwIndxCount ].ubFlags&COMMONPARAMDB_FLAG_UMCONV ) 
        {
                // cast to
            pvGeneric=(void  *)psCommonParamTable[ uwIndxCount ].hpvData;
    
                // then back to , must remain same
            assert( psCommonParamTable[ uwIndxCount ].hpvData == (HPVOID)pvGeneric );
        }
    }
    
    return TRUE;
}
#endif

//***************************************************************************
// Fast search compare

static SWORD ParameterElementsIndxCompare( const UWORD  * puwIndex, const COMMONPARAMDB_ENTRY  * psEntry )
{
  if ( *puwIndex > psEntry->uwIndex )
    return +1;
  else if ( *puwIndex < psEntry->uwIndex )
    return -1;                    
  else
    return 0;
}

//***************************************************************************
// Fast table search

static void  * hbsearch( const void  * key, const void  * base, size_t num, size_t width, SWORD (*compare)(const void  *, const void  *) )
{
  const void  * current;
  size_t lower = 0;
  size_t upper = num;
  size_t index;
  SWORD  swResult;

  if ( num == 0 || width == 0 )
    return NULL;

  while ( lower < upper )
  {
    index = ( lower + upper ) / 2;
    current = (const void  *)( ( (const SBYTE  *)base ) + ( index * width ) );
  
    swResult = compare( key, current );
  
    if ( swResult < 0 )
      upper = index;
    else if ( swResult > 0 )
      lower = index + 1;
    else
      return (void  *)current;
 
  }
  return NULL;
}

//***************************************************************************
// Parameter Table search

COMMONPARAMDB_ENTRY  * ComParDBEntrySearch(UWORD uwIndex)
{
    return (COMMONPARAMDB_ENTRY  *)hbsearch( &uwIndex, psCommonParamTable, uwCommonParamCount, sizeof( COMMONPARAMDB_ENTRY ),
         (SWORD (*)(const void  *, const void  *))&ParameterElementsIndxCompare );
}

//***************************************************************************
// Parameter Get Entry Size: return entry (or single element in case of array)
// size in bytes for basic types, return 0 for BIT and STRING

UWORD ComParDBEntryGetEntrySize(COMMONPARAMDB_ENTRY  * psEntry)
{
    return puwDataSizes[(psEntry->ubType&COMMONPARAMDB_TYPE_TYPE_MASK)-1];
}

//***************************************************************************
// Parameter Get Entry Signed/Unsigned (only for integer types)

UWORD ComParDBEntryGetEntrySign(COMMONPARAMDB_ENTRY  * psEntry)
{
    UBYTE ubType=psEntry->ubType&COMMONPARAMDB_TYPE_TYPE_MASK;

    if(ubType==COMMONPARAMDB_TYPE_SBYTE || ubType==COMMONPARAMDB_TYPE_SWORD || \
        ubType==COMMONPARAMDB_TYPE_SLONG || ubType==COMMONPARAMDB_TYPE_SQWRD)
        return COMMONPARAMDB_CH_SIGN_SIGNED;

    else if(ubType==COMMONPARAMDB_TYPE_UBYTE || ubType==COMMONPARAMDB_TYPE_UWORD || \
        ubType==COMMONPARAMDB_TYPE_ULONG || ubType==COMMONPARAMDB_TYPE_UQWRD)
        return COMMONPARAMDB_CH_SIGN_UNSIGNED;

    else
        return COMMONPARAMDB_CH_SIGN_NA;
}

//***************************************************************************
// Parameter Get pointer for data

HPVOID ComParDBEntryGetPointer(COMMONPARAMDB_ENTRY  * psEntry, UWORD uwElement, HPUWORD hpuwOffset)
{
    UWORD uwSize;

    *hpuwOffset=0;

        // get element size
    uwSize=puwDataSizes[(psEntry->ubType&COMMONPARAMDB_TYPE_TYPE_MASK)-1];

        // if >0 then is common handled 
    if(uwSize)
    {
            // is const to local data
        if(psEntry->ubType&COMMONPARAMDB_TYPE_CONST_MASK)
            return &(psEntry->hpvData);

            // is pointer to data
        if(psEntry->ubType&COMMONPARAMDB_TYPE_POINTER_MASK)
        {
            *hpuwOffset=psEntry->swMoreType+uwSize*uwElement;
            return psEntry->hpvData;
        }

            return &((HPUBYTE)psEntry->hpvData)[uwSize*uwElement];
    }

        // otherwise the other types that is common handled are bit fields
    else if(psEntry->ubType==COMMONPARAMDB_TYPE_BITW || psEntry->ubType==COMMONPARAMDB_TYPE_BITD)
        if(psEntry->swMoreType>=0)
            return &((HPUWORD)(psEntry->hpvData))[psEntry->swMoreType/16];
        else
            return &((HPUWORD)(psEntry->hpvData))[uwElement/16];

    else
        return NULL;
}

//***************************************************************************
// Parameter Entry Check for common handling
// NB: if the entry is BITW or BITD, the exchanged data is one byte (like BOOL)

UWORD ComParDBEntryCheckForCommonHandling(COMMONPARAMDB_ENTRY  * psEntry)
{
        // if hook, parameter must be externally processed via hook function
    if(psEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK)
        return COMMONPARAMDB_CH_HOOK_REQUIRED;

        // if >0 then is common handled 
    if(puwDataSizes[(psEntry->ubType&COMMONPARAMDB_TYPE_TYPE_MASK)-1]>0)
        return COMMONPARAMDB_CH_OK;

        // otherwise the other types that is common handled are bit fields
    if(psEntry->ubType==COMMONPARAMDB_TYPE_BITW || psEntry->ubType==COMMONPARAMDB_TYPE_BITD)
        return COMMONPARAMDB_CH_OK;

        // if here this COMMONPARAMDB_TYPE data is not handled here 
    return COMMONPARAMDB_CH_EXTERN_HANDLER_REQ;
}

//***************************************************************************
// Parameter Entry Common Read
// NB: if the entry is BITW or BITD, the exchanged data is one byte (like BOOL)

UWORD ComParDBEntryRead(COMMONPARAMDB_ENTRY  * psEntry, UWORD uwElement, void * pvBufferDataOut, UWORD * puwBufferSize)
{
    UBYTE ubLocBuf[COMMONPARAMDB_BASETYPE_MAXSIZE];
    UWORD uwSize;
    UWORD uwRetVal;
    HPUBYTE hpubData;
    UWORD uwOffset;

        // if hook, parameter must be externally processed via hook function
    if(psEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK)
        return COMMONPARAMDB_CH_HOOK_REQUIRED;

        // check for read permission
    if((psEntry->ubFlags&COMMONPARAMDB_FLAG_RD)==0)
        return COMMONPARAMDB_CH_NO_READ_ACCESS;

        // get element size
    uwSize=puwDataSizes[(psEntry->ubType&COMMONPARAMDB_TYPE_TYPE_MASK)-1];

        // if >0 then is common handled 
    if(uwSize)
    {
            // check for array size compatibility
        if(uwElement>=psEntry->uwNElements)
            return COMMONPARAMDB_CH_INVALID_ELEMENT;

            // check dest buffer size
        if(uwSize>*puwBufferSize)
            return COMMONPARAMDB_CH_LENGTHTOOLOW;

            // get data pointer
        hpubData=ComParDBEntryGetPointer(psEntry, uwElement, &uwOffset);

            // proceed to um conversion if required
        if(psEntry->ubFlags&COMMONPARAMDB_FLAG_UMCONV)
        {
            uwRetVal=(*psEntry->uf.fpfUmConv)(COMMONPARAMDB_UCFLAG_READ, pvBufferDataOut, (void  *)hpubData);
            if(uwRetVal!=COMMONPARAMDB_CH_OK)
                return uwRetVal;
        }
        else
        {
                // get data atomically
            if(psEntry->ubType&COMMONPARAMDB_TYPE_POINTER_MASK)
                atomic_read(ubLocBuf, &(*(UBYTE  *  *)hpubData)[uwOffset], uwSize);
            else
                atomic_read(ubLocBuf, hpubData, uwSize);

            memcpy(pvBufferDataOut, ubLocBuf, uwSize);
        }

            // fix data out size
        *puwBufferSize=uwSize;
    }

        // otherwise the other types that is common handled are bit fields
    else if(psEntry->ubType==COMMONPARAMDB_TYPE_BITW || psEntry->ubType==COMMONPARAMDB_TYPE_BITD)
    {
            // check dest buffer size
        if(*puwBufferSize<1)
            return COMMONPARAMDB_CH_LENGTHTOOLOW;

            // get data pointer
        hpubData=ComParDBEntryGetPointer(psEntry, uwElement, &uwOffset);

        if(psEntry->swMoreType>=0)
        {
                // if single bit spec then selected element must be zero
            if(uwElement>=1)
                return COMMONPARAMDB_CH_INVALID_ELEMENT;

                // calc bit position
            uwElement=psEntry->swMoreType%16;
        }
        else
        {
                // otherwise overall array size is no. of bit multiplied by no. of elements
            if(psEntry->ubType==COMMONPARAMDB_TYPE_BITW)
                uwSize=16*psEntry->uwNElements;
            else
                uwSize=32*psEntry->uwNElements;

            if(uwElement>=uwSize)
                return COMMONPARAMDB_CH_INVALID_ELEMENT;

                // calc bit position
            uwElement%=16;
        }

            // select bit
        if((*((HPUWORD)hpubData))&(1u<<uwElement))
            *(UBYTE *)pvBufferDataOut=1;
        else
            *(UBYTE *)pvBufferDataOut=0;

            // data out size is one byte
        *puwBufferSize=1;
    }
    else
            // if here this COMMONPARAMDB_TYPE data is not handled here 
        return COMMONPARAMDB_CH_EXTERN_HANDLER_REQ;

    return COMMONPARAMDB_CH_OK;
}

//***************************************************************************
// Parameter Entry Common Write
// NB: if the entry is BITW or BITD, the exchanged data is one byte (like BOOL)

UWORD ComParDBEntryWrite(COMMONPARAMDB_ENTRY  * psEntry, UWORD uwElement, void * pvBufferDataIn, UWORD * puwBufferSize)
{
    UBYTE ubLocBuf[COMMONPARAMDB_BASETYPE_MAXSIZE];
    UWORD uwSize;
    UWORD uwRetVal;
    UWORD uwBitValue;
    HPUBYTE hpubData;
    UWORD uwOffset;

        // if hook, parameter must be externally processed via hook function
    if(psEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK)
        return COMMONPARAMDB_CH_HOOK_REQUIRED;

        // check for write permission
    if((psEntry->ubFlags&COMMONPARAMDB_FLAG_WR)==0)
        return COMMONPARAMDB_CH_NO_WRITE_ACCESS;

        // check for write deny by drive status
    if(psEntry->uWrDeny&ulSystemStatus)
        return COMMONPARAMDB_CH_WRITE_DENIED;

        // get element size
    uwSize=puwDataSizes[(psEntry->ubType&COMMONPARAMDB_TYPE_TYPE_MASK)-1];

        // if >0 then is common handled 
    if(uwSize)
    {
            // check for array size compatibility
        if(uwElement>=psEntry->uwNElements)
            return COMMONPARAMDB_CH_INVALID_ELEMENT;

            // check buffer size
        if(uwSize!=*puwBufferSize)
            return COMMONPARAMDB_CH_WRONGLENGTH;

            // get data pointer
        hpubData=ComParDBEntryGetPointer(psEntry, uwElement, &uwOffset);

            // proceed to um conversion if required
        if(psEntry->ubFlags&COMMONPARAMDB_FLAG_UMCONV)
        {
            uwRetVal=(*psEntry->uf.fpfUmConv)(COMMONPARAMDB_UCFLAG_WRITE, (void  *)hpubData, pvBufferDataIn);
            if(uwRetVal!=COMMONPARAMDB_CH_OK)
                return uwRetVal;
        }
        else
        {
                // proceed to data validation if required
            if(psEntry->ubFlags&COMMONPARAMDB_FLAG_VALIDATE)
            {
                uwRetVal=(*psEntry->uf.fpfValidate)(pvBufferDataIn);
                if(uwRetVal!=COMMONPARAMDB_CH_OK)
                    return uwRetVal;
    
                memcpy(ubLocBuf, pvBufferDataIn, uwSize);
            }
            else
                memcpy(ubLocBuf, pvBufferDataIn, uwSize);
    
                // put data atomically
            if(psEntry->ubType&COMMONPARAMDB_TYPE_POINTER_MASK)
                atomic_write(&(*(UBYTE  *  *)hpubData)[uwOffset], ubLocBuf, uwSize);
            else
                atomic_write(hpubData, ubLocBuf, uwSize);
        }
    }

        // otherwise the other types that is common handled are bit fields
    else if(psEntry->ubType==COMMONPARAMDB_TYPE_BITW || psEntry->ubType==COMMONPARAMDB_TYPE_BITD)
    {
            // check dest buffer size
        if(*puwBufferSize!=1)
            return COMMONPARAMDB_CH_WRONGLENGTH;

            // get data pointer
        hpubData=ComParDBEntryGetPointer(psEntry, uwElement, &uwOffset);

        if(psEntry->swMoreType>=0)
        {
                // if single bit spec then selected element must be zero
            if(uwElement>=1)
                return COMMONPARAMDB_CH_INVALID_ELEMENT;

                // calc bit position
            uwElement=psEntry->swMoreType%16;
        }
        else
        {
                // otherwise overall array size is no. of bit multiplied by no. of elements
            if(psEntry->ubType==COMMONPARAMDB_TYPE_BITW)
                uwSize=16*psEntry->uwNElements;
            else
                uwSize=32*psEntry->uwNElements;

            if(uwElement>=uwSize)
                return COMMONPARAMDB_CH_INVALID_ELEMENT;

                // calc bit position
            uwElement%=16;
        }

            // atomically write bit
        uwBitValue=*(UBYTE *)pvBufferDataIn?1:0;
        atomic_write_bits((HPUWORD)hpubData, uwBitValue<<uwElement, 1<<uwElement);
    }
    else
            // if here this COMMONPARAMDB_TYPE data is not handled here 
        return COMMONPARAMDB_CH_EXTERN_HANDLER_REQ;

        // if here write was ok, then check for reset required
    if(psEntry->ubFlags&COMMONPARAMDB_FLAG_RESETREQ)
        atomic_long_set_bits( &ulSystemWarnings, SYSTEMWARNINGS_SAVEANDRESETREQ );

    return COMMONPARAMDB_CH_OK;
}
