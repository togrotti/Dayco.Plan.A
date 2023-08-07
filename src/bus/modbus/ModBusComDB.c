/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ModBusComDB.c                                              */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : ModBus parameter DB interface for Common DB handling       */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

//#include <intrins.h>
#include <stdlib.h>
#include <string.h>

#include "ModBusComDB.h"
#include "common\CommonUtility.h"
#include "ModbusAppProtocol.h"

//***************************************************************************
// Globals

const MODBUSCOMDB_ENTRY  * hpsModBusDynParamTable=NULL;
UWORD uwModBusDynParamCount=0;

//***************************************************************************
// Locals

static ULONG getCoilsSize(const MODBUSCOMDB_ENTRY  * psEntry);
static SWORD CoilsElementsIndxCompare( const UWORD  * puwIndex, const MODBUSCOMDB_ENTRY  * psEntry );
static const MODBUSCOMDB_ENTRY  * coilsentrysearch(UWORD uwIndex);

//***************************************************************************
// Table checking, just for debug purpose

#ifdef _APP_DEBUG
BOOL ModBusComDBCheckTable(void)
{
    UWORD uwIndxCount;
    ULONG ulSize;
    
        // Check Index Contiguity and Duplicates for fast search
    for ( uwIndxCount = 0; uwIndxCount < uwModBusParamCount-1; uwIndxCount++ ) 
        assert ( hpsModBusParamTable[ uwIndxCount ].uwIndex < hpsModBusParamTable[ uwIndxCount + 1 ].uwIndex );
    
        // Check Index Contiguity and Duplicates for fast search,
        // Check overlapping array sizes
        // Check well-know sizes (=0 not allowed)
        // BITW and BITD are not allowed right now
    for ( uwIndxCount = 0; uwIndxCount < uwModBusCoilsCount-1; uwIndxCount++ ) 
    {
        assert ( hpsModBusCoilsTable[ uwIndxCount ].uwIndex < hpsModBusCoilsTable[ uwIndxCount + 1 ].uwIndex );
        assert ( ulSize = getCoilsSize( &hpsModBusCoilsTable[ uwIndxCount ] ) );
        assert ( hpsModBusCoilsTable[ uwIndxCount ].uwIndex+ulSize <= hpsModBusCoilsTable[ uwIndxCount + 1 ].uwIndex );
        assert ( hpsModBusCoilsTable[ uwIndxCount ].hpsComDBEntry->ubType != COMMONPARAMDB_TYPE_BITW );
        assert ( hpsModBusCoilsTable[ uwIndxCount ].hpsComDBEntry->ubType != COMMONPARAMDB_TYPE_BITD );
        assert ( (hpsModBusCoilsTable[ uwIndxCount ].hpsComDBEntry->ubFlags & (COMMONPARAMDB_FLAG_UMCONV|COMMONPARAMDB_FLAG_VALIDATE)) == 0 );
    }
    
    return TRUE;
}
#endif

//***************************************************************************
// Fast search compare

static SWORD ParameterElementsIndxCompare( const UWORD  * puwIndex, const MODBUSCOMDB_ENTRY  * psEntry )
{
    if ( *puwIndex > psEntry->uwIndex + (psEntry->hpsComDBEntry->uwNElements - 1))
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

const MODBUSCOMDB_ENTRY  * ModBusComDBEntrySearch(UWORD uwIndex)
{
  if(hpsModBusDynParamTable)
  {
    const MODBUSCOMDB_ENTRY  * hpsLDb=hpsModBusDynParamTable;
    UWORD uwCt;

    for(uwCt=0;uwCt<uwModBusDynParamCount;uwCt++,hpsLDb++)
      if(hpsLDb->uwIndex==uwIndex)
        return hpsLDb;
  }

  return (const MODBUSCOMDB_ENTRY  *)hbsearch( &uwIndex, hpsModBusParamTable, uwModBusParamCount, sizeof( MODBUSCOMDB_ENTRY ),
         (SWORD (*)(const void  *, const void  *))&ParameterElementsIndxCompare );
}

//***************************************************************************
// Parameter Entry Read

UBYTE ModBusComDBEntryRead(UBYTE ubFCode, UWORD uwRegAddress, UWORD uwRegNumbers, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength)
{
    const MODBUSCOMDB_ENTRY  * pElem;
    UBYTE ubOutBuffer[4];
    UWORD uwReplyLength = 0;
    UWORD uwBufSize, uwCt;
    UBYTE ubSwap;
    
        // check reading range
    assert(uwRegNumbers>=1 && uwRegNumbers<=125);

        // Search in the parameters database
    if( ( pElem = ModBusComDBEntrySearch( uwRegAddress ) ) == NULL )
        return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

        // First byte contains the function code.
    hpubReplyBuffer[ uwReplyLength++ ] = ubFCode;
    
        // Second byte contains the number of bytes in the response 
    hpubReplyBuffer[ uwReplyLength++ ] = (UBYTE)( uwRegNumbers * 2 );

        // if hook
    if(pElem->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK)
    {
        UWORD uwRetVal;
        ULONG ulContext=0l;

            // set bufsize for hook, that is unrelated to the entry data type and
            // is used only to signal data buffer space
        uwBufSize=uwRegNumbers*2;

            // init hook transaction
        uwRetVal=(pElem->hpsComDBEntry->uf.fpfHook)(pElem->hpsComDBEntry, COMMONPARAMDB_CBFLAG_INIT|COMMONPARAMDB_CBFLAG_NODSCHECK|COMMONPARAMDB_CBFLAG_RD, \
                uwRegAddress - pElem->uwIndex, NULL, NULL, &ulContext);
        if(uwRetVal!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // read data
        uwRetVal=(pElem->hpsComDBEntry->uf.fpfHook)(pElem->hpsComDBEntry, COMMONPARAMDB_CBFLAG_SEGMENT|COMMONPARAMDB_CBFLAG_NODSCHECK|COMMONPARAMDB_CBFLAG_RD, \
                uwRegAddress - pElem->uwIndex, &hpubReplyBuffer[uwReplyLength], &uwBufSize, &ulContext);
        if(uwRetVal!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // end hook transaction
        uwRetVal=(pElem->hpsComDBEntry->uf.fpfHook)(pElem->hpsComDBEntry, COMMONPARAMDB_CBFLAG_END|COMMONPARAMDB_CBFLAG_NODSCHECK|COMMONPARAMDB_CBFLAG_RD, \
                uwRegAddress - pElem->uwIndex, NULL, NULL, &ulContext);
        if(uwRetVal!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // Reorder bytes for output (data extension, if needed, is done inside hook)
        for(uwCt=0;uwCt<uwRegNumbers*2;uwCt+=2)
        {
            ubSwap = hpubReplyBuffer[ uwReplyLength+uwCt ] ;
            hpubReplyBuffer[ uwReplyLength+uwCt ] = hpubReplyBuffer[ uwReplyLength+uwCt+1 ];
            hpubReplyBuffer[ uwReplyLength+uwCt+1 ] = ubSwap;
        }

            // adjust output size
        uwReplyLength += uwRegNumbers*2;
    }
    else
    {
            // for backward compatibility no more than 32bit are allowed to trasfer, e.g. one register per time
        if(uwRegNumbers>2)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // calculate entry size requested
        if(pElem->hpsComDBEntry->ubType==COMMONPARAMDB_TYPE_BITW || pElem->hpsComDBEntry->ubType==COMMONPARAMDB_TYPE_BITD)
            uwBufSize=uwRegNumbers;
        else
            uwBufSize=uwRegNumbers*ComParDBEntryGetEntrySize(pElem->hpsComDBEntry);

            // Get Entry
        if(ComParDBEntryRead(pElem->hpsComDBEntry, uwRegAddress - pElem->uwIndex, ubOutBuffer, &uwBufSize)!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // if read one byte, return word anyway (sign extended if necessary)
        if(uwBufSize==1)
        {
            if(ComParDBEntryGetEntrySign(pElem->hpsComDBEntry)==COMMONPARAMDB_CH_SIGN_SIGNED && (SBYTE)ubOutBuffer[0]<0)
                ubOutBuffer[1]=0xff;
            else
                ubOutBuffer[1]=0;
        }
        
            // Reorder and write bytes to output
        hpubReplyBuffer[ uwReplyLength++ ] = ubOutBuffer[1];
        hpubReplyBuffer[ uwReplyLength++ ] = ubOutBuffer[0];
        
        if(uwRegNumbers == 2)
        {
            if(uwBufSize == 4)
            {
                hpubReplyBuffer[ uwReplyLength++ ] = ubOutBuffer[3];
                hpubReplyBuffer[ uwReplyLength++ ] = ubOutBuffer[2];
            }
            else
            {
                if((SBYTE)(ubOutBuffer[1])<0 && ComParDBEntryGetEntrySign(pElem->hpsComDBEntry)==COMMONPARAMDB_CH_SIGN_SIGNED)
                {
                    hpubReplyBuffer[ uwReplyLength++ ] = 0xff;
                    hpubReplyBuffer[ uwReplyLength++ ] = 0xff;
                }
                else
                {
                    hpubReplyBuffer[ uwReplyLength++ ] = 0;
                    hpubReplyBuffer[ uwReplyLength++ ] = 0;
                }
            }
        }
    }

        // write out length
    *hpuwReplyLength=uwReplyLength;

    return MODBUS_EXCEPTION_SUCCESSFULLY;
}

//***************************************************************************
// Parameter Entry Write

UBYTE ModBusComDBEntryWrite(UBYTE ubFCode, UWORD uwRegAddress, UWORD uwRegNumbers, UWORD uwOrigRegAddress, HPUBYTE hpubInBuffer, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bRetRegValue)
{
    const MODBUSCOMDB_ENTRY  * pElem;
    UBYTE ubInBuffer[4],ubRetRegValue[2];
    UWORD uwReplyLength = 0;
    UWORD uwBufSize,uwCt;
    UBYTE ubSwap;
    
        // check writing range
    assert(uwRegNumbers>=1 && uwRegNumbers<=123);

        // Search in the parameters database
    if( ( pElem = ModBusComDBEntrySearch( uwRegAddress ) ) == NULL )
        return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

        // record 'as is' actual passed value to be written
    ubRetRegValue[0]=hpubInBuffer[0];
    ubRetRegValue[1]=hpubInBuffer[1];

        // if hook
    if(pElem->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK)
    {
        UWORD uwRetVal;
        ULONG ulContext=0l;
        ULONG ulDataSize;

            // check for write deny by drive status
        if(pElem->hpsComDBEntry->uWrDeny&ulSystemStatus)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // Reorder input bytes
        for(uwCt=0;uwCt<uwRegNumbers*2;uwCt+=2)
        {
            ubSwap = hpubInBuffer[ uwCt ] ;
            hpubInBuffer[ uwCt ] = hpubInBuffer[ uwCt+1 ];
            hpubInBuffer[ uwCt+1 ] = ubSwap;
        }

            // prepare data size
        ulDataSize=uwRegNumbers*2;
        uwBufSize=sizeof(ulDataSize);

            // init hook transaction
        uwRetVal=(pElem->hpsComDBEntry->uf.fpfHook)(pElem->hpsComDBEntry, COMMONPARAMDB_CBFLAG_INIT|COMMONPARAMDB_CBFLAG_NODSCHECK|COMMONPARAMDB_CBFLAG_WR, \
                uwRegAddress - pElem->uwIndex, &ulDataSize, &uwBufSize, &ulContext);
        if(uwRetVal!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // calculate entry size received
        uwBufSize=uwRegNumbers*2;

            // write data
        uwRetVal=(pElem->hpsComDBEntry->uf.fpfHook)(pElem->hpsComDBEntry, COMMONPARAMDB_CBFLAG_SEGMENT|COMMONPARAMDB_CBFLAG_NODSCHECK|COMMONPARAMDB_CBFLAG_WR, \
                uwRegAddress - pElem->uwIndex, hpubInBuffer, &uwBufSize, &ulContext);
        if(uwRetVal!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // end hook transaction
        uwRetVal=(pElem->hpsComDBEntry->uf.fpfHook)(pElem->hpsComDBEntry, COMMONPARAMDB_CBFLAG_END|COMMONPARAMDB_CBFLAG_NODSCHECK|COMMONPARAMDB_CBFLAG_WR, \
                uwRegAddress - pElem->uwIndex, NULL, NULL, &ulContext);
        if(uwRetVal!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
    else
    {
            // for backward compatibility no more than 32bit are allowed to trasfer, e.g. one register per time
        if(uwRegNumbers>2)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // calculate entry size requested
        uwBufSize=uwRegNumbers*2;

            // Reorder bytes for writing
        ubInBuffer[1]=*hpubInBuffer++;
        ubInBuffer[0]=*hpubInBuffer++;
        
        if(uwRegNumbers == 2)
        {
            ubInBuffer[3]=*hpubInBuffer++;
            ubInBuffer[2]=*hpubInBuffer++;
        }
        else if((SBYTE)(ubInBuffer[1])<0 && ComParDBEntryGetEntrySign(pElem->hpsComDBEntry)==COMMONPARAMDB_CH_SIGN_SIGNED)
        {
            ubInBuffer[3]=0xff;
            ubInBuffer[2]=0xff;
        }
        else
        {
            ubInBuffer[3]=0;
            ubInBuffer[2]=0;
        }
        
        if(pElem->hpsComDBEntry->ubType==COMMONPARAMDB_TYPE_BITW || pElem->hpsComDBEntry->ubType==COMMONPARAMDB_TYPE_BITD)
            uwBufSize=1;
        else
            uwBufSize=ComParDBEntryGetEntrySize(pElem->hpsComDBEntry);
        
            // Write Entry
        if(ComParDBEntryWrite(pElem->hpsComDBEntry, uwRegAddress - pElem->uwIndex, ubInBuffer, &uwBufSize)!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }

    // First byte contains the function code.
    hpubReplyBuffer[ uwReplyLength++ ] = ubFCode;
    
    // Second word contains the register address
    hpubReplyBuffer[ uwReplyLength++ ] = HIBYTE( uwOrigRegAddress );
    hpubReplyBuffer[ uwReplyLength++ ] = LOBYTE( uwOrigRegAddress );

    if(bRetRegValue)
    {
        // Third word contains the incoming register value
        hpubReplyBuffer[ uwReplyLength++ ] = ubRetRegValue[0];
        hpubReplyBuffer[ uwReplyLength++ ] = ubRetRegValue[1];
    }
    else
    {    
        // Third word contains the register numbers 
        hpubReplyBuffer[ uwReplyLength++ ] = HIBYTE( uwRegNumbers );
        hpubReplyBuffer[ uwReplyLength++ ] = LOBYTE( uwRegNumbers );
    }

        // write out length
    *hpuwReplyLength=uwReplyLength;

    return MODBUS_EXCEPTION_SUCCESSFULLY;
}

//***************************************************************************
// Get size as coil size

static ULONG getCoilsSize(const MODBUSCOMDB_ENTRY  * psEntry)
{
    ULONG ulSize=(ULONG)ComParDBEntryGetEntrySize(psEntry->hpsComDBEntry);

    if(ulSize)
        ulSize*=(ULONG)(sizeof(UBYTE)*8)*psEntry->hpsComDBEntry->uwNElements;
    else
        ulSize=0;

    return ulSize;
}

//***************************************************************************
// Fast coils search compare

static SWORD CoilsElementsIndxCompare( const UWORD  * puwIndex, const MODBUSCOMDB_ENTRY  * psEntry )
{
    ULONG ulSize=getCoilsSize(psEntry);

        // if zero then elements cannot be used as coil
    if(ulSize==0l)
        if(*puwIndex >= psEntry->uwIndex)
            return +1;
        else
            return -1;

    if ( *puwIndex > psEntry->uwIndex + (ulSize - 1))
        return +1;
    else if ( *puwIndex < psEntry->uwIndex )
        return -1;                    
    else
        return 0;
}

//***************************************************************************
// Coils Table search

static const MODBUSCOMDB_ENTRY  * coilsentrysearch(UWORD uwIndex)
{
  if(hpsModBusDynParamTable)
  {
    const MODBUSCOMDB_ENTRY  * hpsLDb=hpsModBusDynParamTable;
    UWORD uwCt;
    ULONG ulSize;

    for(uwCt=0;uwCt<uwModBusDynParamCount;uwCt++,hpsLDb++)
    {
      ulSize=getCoilsSize(hpsLDb);
      if(uwIndex>=hpsLDb->uwIndex && uwIndex<(hpsLDb->uwIndex+ulSize))
        return hpsLDb;
    }
  }

  return (const MODBUSCOMDB_ENTRY  *)hbsearch( &uwIndex, hpsModBusCoilsTable, uwModBusCoilsCount, sizeof( MODBUSCOMDB_ENTRY ),
         (SWORD (*)(const void  *, const void  *))&CoilsElementsIndxCompare );
}

//***************************************************************************
// Coil Entry Read

UBYTE ModBusComDBCoilRead(UBYTE ubFCode, UWORD uwRegAddress, UWORD uwRegNumbers, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength)
{
    const MODBUSCOMDB_ENTRY  * pElem;
    UWORD uwReplyLength = 0;
    UWORD uwBufSize, uwCt;
    UBYTE ubSwap;
    
        // check reading range
    assert(uwRegNumbers>=1 && uwRegNumbers<=2000);

        // Search in the parameters database
    if( ( pElem = coilsentrysearch( uwRegAddress ) ) == NULL )
        return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

        // First byte contains the function code.
    hpubReplyBuffer[ uwReplyLength++ ] = ubFCode;

        // Second byte contains the number of bytes in the response,
        // skip it now, it will be filled at the end
    uwReplyLength++;
    
        // if hook
    if(pElem->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK)
    {
        UWORD uwRetVal;
        ULONG ulContext=0l;

            // init hook transaction
        uwRetVal=(pElem->hpsComDBEntry->uf.fpfHook)(pElem->hpsComDBEntry, COMMONPARAMDB_CBFLAG_INIT|COMMONPARAMDB_CBFLAG_NODSCHECK|COMMONPARAMDB_CBFLAG_BIT|COMMONPARAMDB_CBFLAG_RD, \
                uwRegAddress-pElem->uwIndex, NULL, NULL, &ulContext);
        if(uwRetVal!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // read data
        uwRetVal=(pElem->hpsComDBEntry->uf.fpfHook)(pElem->hpsComDBEntry, COMMONPARAMDB_CBFLAG_SEGMENT|COMMONPARAMDB_CBFLAG_NODSCHECK|COMMONPARAMDB_CBFLAG_BIT|COMMONPARAMDB_CBFLAG_RD, \
                uwRegAddress-pElem->uwIndex, &hpubReplyBuffer[uwReplyLength], &uwRegNumbers, &ulContext);
        if(uwRetVal!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // end hook transaction
        uwRetVal=(pElem->hpsComDBEntry->uf.fpfHook)(pElem->hpsComDBEntry, COMMONPARAMDB_CBFLAG_END|COMMONPARAMDB_CBFLAG_NODSCHECK|COMMONPARAMDB_CBFLAG_BIT|COMMONPARAMDB_CBFLAG_RD, \
                uwRegAddress-pElem->uwIndex, NULL, NULL, &ulContext);
        if(uwRetVal!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
    else
    {
        UBYTE ubOutBuffer[sizeof(ULONG)];
        UWORD uwXBufSize;
        UWORD uwRegOffset=(uwRegAddress-pElem->uwIndex)/(sizeof(UBYTE)*8);

            // for backward compatibility no more than 32bit are allowed to trasfer, e.g. one register per time
        if(uwRegAddress-pElem->uwIndex+uwRegNumbers>(sizeof(ubOutBuffer)*8))
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // entry size always 32bit
        uwXBufSize=sizeof(ubOutBuffer);

            // Get Entry
        if(ComParDBEntryRead(pElem->hpsComDBEntry, uwRegOffset, ubOutBuffer, &uwXBufSize)!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // check against returned data length
        if(uwRegAddress-pElem->uwIndex+uwRegNumbers>(uwXBufSize*8))
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // move to upper layer buffer
        memcpy(&hpubReplyBuffer[uwReplyLength], ubOutBuffer, sizeof(ubOutBuffer));

            // calculate number of shift to be applied
        uwRegOffset=(uwRegAddress-pElem->uwIndex)%(sizeof(UBYTE)*8);
    
            // align output bits
        for(uwCt=0;uwCt<uwXBufSize;uwCt++)
        {
            ubSwap =hpubReplyBuffer[uwReplyLength+uwCt]   >> uwRegOffset;
            ubSwap|=hpubReplyBuffer[uwReplyLength+uwCt+1] << (8-uwRegOffset);
            hpubReplyBuffer[uwReplyLength+uwCt]=ubSwap;
        }

            // zeroes extra bits if necessary
        uwXBufSize=uwRegNumbers%8;
        if(uwXBufSize)
            hpubReplyBuffer[uwReplyLength+((uwRegNumbers-1)/8)]&=(UBYTE)(((UWORD)0x01<<uwXBufSize)-1);
    }

        // calculate # of byte to contain the requested # of coils
    uwBufSize=uwRegNumbers/(sizeof(UBYTE)*8);
    if(uwRegNumbers%(sizeof(UBYTE)*8))
        uwBufSize++;

        // Second byte contains the number of bytes in the response 
    hpubReplyBuffer[ uwReplyLength-1 ] = (UBYTE)uwBufSize;

        // write out length
    *hpuwReplyLength=uwReplyLength+uwBufSize;

    return MODBUS_EXCEPTION_SUCCESSFULLY;
}

//***************************************************************************
// Coil Entry Write, directly processed

UBYTE ModBusComDBCoilWrite(UBYTE ubFCode, UWORD uwRegAddress, UWORD uwRegNumbers, UWORD uwOrigRegAddress, HPUBYTE hpubInBuffer, HPUBYTE hpubReplyBuffer, HPUWORD hpuwReplyLength, BOOL bRetRegValue)
{
    const MODBUSCOMDB_ENTRY  * pElem;
    UBYTE ubRetRegValue[2];
    UWORD uwReplyLength = 0;
    UWORD uwBufSize,uwCt;
    UWORD uwRegOffset;

        // check writing range
    assert(uwRegNumbers>=1 && uwRegNumbers<=1968);

        // Search in the parameters database
    if( ( pElem = coilsentrysearch( uwRegAddress ) ) == NULL )
        return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

        // record 'as is' actual passed value to be written
    ubRetRegValue[0]=hpubInBuffer[0];
    ubRetRegValue[1]=hpubInBuffer[1];

        // if single transfer then incoming value must be checked against
        // only two allowed values: 0x0000 and 0xFF00
    if(bRetRegValue)
    {
        if(ubRetRegValue[1])
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

        if(ubRetRegValue[0]!=0x00 && ubRetRegValue[0]!=0xFF)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // transform it as single bit coil transfer
        if(ubRetRegValue[0])
            hpubInBuffer[0]=0x01;
        else
            hpubInBuffer[0]=0x00;
    }

        // calc offset
    uwRegOffset=uwRegAddress-pElem->uwIndex;

        // if hook
    if(pElem->hpsComDBEntry->ubFlags&COMMONPARAMDB_FLAG_HOOK)
    {
        UWORD uwRetVal;
        ULONG ulContext=0l;
        ULONG ulDataSize;

            // check for write deny by drive status
        if(pElem->hpsComDBEntry->uWrDeny&ulSystemStatus)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // prepare data
        ulDataSize=uwRegNumbers;
        uwBufSize=sizeof(ulDataSize);

            // init hook transaction
        uwRetVal=(pElem->hpsComDBEntry->uf.fpfHook)(pElem->hpsComDBEntry, COMMONPARAMDB_CBFLAG_INIT|COMMONPARAMDB_CBFLAG_NODSCHECK|COMMONPARAMDB_CBFLAG_BIT|COMMONPARAMDB_CBFLAG_WR, \
                uwRegOffset, &ulDataSize, &uwBufSize, &ulContext);
        if(uwRetVal!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // copy entry size received
        uwBufSize=uwRegNumbers;

            // write data
        uwRetVal=(pElem->hpsComDBEntry->uf.fpfHook)(pElem->hpsComDBEntry, COMMONPARAMDB_CBFLAG_SEGMENT|COMMONPARAMDB_CBFLAG_NODSCHECK|COMMONPARAMDB_CBFLAG_BIT|COMMONPARAMDB_CBFLAG_WR, \
                uwRegOffset, hpubInBuffer, &uwBufSize, &ulContext);
        if(uwRetVal!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // end hook transaction
        uwRetVal=(pElem->hpsComDBEntry->uf.fpfHook)(pElem->hpsComDBEntry, COMMONPARAMDB_CBFLAG_END|COMMONPARAMDB_CBFLAG_NODSCHECK|COMMONPARAMDB_CBFLAG_BIT|COMMONPARAMDB_CBFLAG_WR, \
                uwRegOffset, NULL, NULL, &ulContext);
        if(uwRetVal!=COMMONPARAMDB_CH_OK)
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
    else
    {
        const COMMONPARAMDB_ENTRY  * psEntry=pElem->hpsComDBEntry;
        UWORD uwXBufSize,uwOffset;
        UWORD uwInMask,uwOutMask;
        HPUBYTE hpubData;

            // for backward compatibility no more than 32bit are allowed to trasfer, e.g. one register per time
        if(uwRegOffset+uwRegNumbers>(sizeof(ULONG)*8))
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // get entry size
        uwXBufSize=ComParDBEntryGetEntrySize(pElem->hpsComDBEntry);

            // check against object size
        if(uwRegOffset+uwRegNumbers>(uwXBufSize*8))
            return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

            // check for write permission
        if((psEntry->ubFlags&COMMONPARAMDB_FLAG_WR)==0)
            return COMMONPARAMDB_CH_NO_WRITE_ACCESS;
    
            // check for write deny by drive status
        if(psEntry->uWrDeny&ulSystemStatus)
            return COMMONPARAMDB_CH_WRITE_DENIED;

            // calc offset as per element size
        uwOffset=uwRegOffset/(uwXBufSize*8);

            // get data pointer
        hpubData=ComParDBEntryGetPointer(psEntry, uwOffset, &uwOffset);

            // resolve pointer to pointer
        if(psEntry->ubType&COMMONPARAMDB_TYPE_POINTER_MASK)
            hpubData=&(*(UBYTE  **)hpubData)[uwOffset];

            // adjust data pointer and offset to byte boundary
        hpubData=&hpubData[uwRegOffset/8];
        uwRegOffset%=8;

            // prepare masks
        uwInMask=0x0001;
        uwOutMask=0x0001<<uwRegOffset;

            // write each single bit
        for(uwCt=0;uwCt<uwRegNumbers;uwCt++,uwInMask<<=1,uwOutMask<<=1)
        {
                // check overflow of input and output buffers
            if(uwInMask==0x0100)
            {
                uwInMask=0x01;
                hpubInBuffer++;
            }
            if(uwOutMask==0x0100)
            {
                uwOutMask=0x01;
                hpubData++;
            }
            
            atomic_byte_write_bits(hpubData, (UBYTE)((*hpubInBuffer&uwInMask)?uwOutMask:0), (UBYTE)uwOutMask);
        }
    }

    // First byte contains the function code.
    hpubReplyBuffer[ uwReplyLength++ ] = ubFCode;
    
    // Second word contains the register address
    hpubReplyBuffer[ uwReplyLength++ ] = HIBYTE( uwOrigRegAddress );
    hpubReplyBuffer[ uwReplyLength++ ] = LOBYTE( uwOrigRegAddress );
    
    if(bRetRegValue)
    {
        // Third word contains the incoming register value
        hpubReplyBuffer[ uwReplyLength++ ] = ubRetRegValue[0];
        hpubReplyBuffer[ uwReplyLength++ ] = ubRetRegValue[1];
    }
    else
    {    
        // Third word contains the register numbers 
        hpubReplyBuffer[ uwReplyLength++ ] = HIBYTE( uwRegNumbers );
        hpubReplyBuffer[ uwReplyLength++ ] = LOBYTE( uwRegNumbers );
    }

        // write out length
    *hpuwReplyLength=uwReplyLength;

    return MODBUS_EXCEPTION_SUCCESSFULLY;
}
