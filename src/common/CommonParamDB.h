/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CommonParamDB.h                                            */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Definition of common parameter database, in order to have  */
/*               a common interface between all fieldbus and internal       */
/*               parameters                                                 */
/*                                                                          */
/****************************************************************************/

#ifndef _COMMONPARAMDB_H
#define _COMMONPARAMDB_H

#include "common\CommonDefines.h"
#include "system\SystemStatus.h"

//***************************************************************************
// Parameter Types

#define COMMONPARAMDB_TYPE_POINTER_MASK     0x20
#define COMMONPARAMDB_TYPE_CONST_MASK       0x40
#define COMMONPARAMDB_TYPE_TYPE_MASK        (~(COMMONPARAMDB_TYPE_POINTER_MASK|COMMONPARAMDB_TYPE_CONST_MASK))

#define COMMONPARAMDB_TYPE_UBYTE            1
#define COMMONPARAMDB_TYPE_SBYTE            2
#define COMMONPARAMDB_TYPE_UWORD            3
#define COMMONPARAMDB_TYPE_SWORD            4
#define COMMONPARAMDB_TYPE_ULONG            5
#define COMMONPARAMDB_TYPE_SLONG            6
#define COMMONPARAMDB_TYPE_UQWRD            7
#define COMMONPARAMDB_TYPE_SQWRD            8
#define COMMONPARAMDB_TYPE_FLOAT            9
#define COMMONPARAMDB_TYPE_DOUBL            10
#define COMMONPARAMDB_TYPE_BITW             11
#define COMMONPARAMDB_TYPE_BITD             12
#define COMMONPARAMDB_TYPE_STRING           13
#define COMMONPARAMDB_TYPE_BINARY           14

#define COMMONPARAMDB_TYPE_P_UBYTE          (COMMONPARAMDB_TYPE_POINTER_MASK|COMMONPARAMDB_TYPE_UBYTE)
#define COMMONPARAMDB_TYPE_P_SBYTE          (COMMONPARAMDB_TYPE_POINTER_MASK|COMMONPARAMDB_TYPE_SBYTE)
#define COMMONPARAMDB_TYPE_P_UWORD          (COMMONPARAMDB_TYPE_POINTER_MASK|COMMONPARAMDB_TYPE_UWORD)
#define COMMONPARAMDB_TYPE_P_SWORD          (COMMONPARAMDB_TYPE_POINTER_MASK|COMMONPARAMDB_TYPE_SWORD)
#define COMMONPARAMDB_TYPE_P_ULONG          (COMMONPARAMDB_TYPE_POINTER_MASK|COMMONPARAMDB_TYPE_ULONG)
#define COMMONPARAMDB_TYPE_P_SLONG          (COMMONPARAMDB_TYPE_POINTER_MASK|COMMONPARAMDB_TYPE_SLONG)
#define COMMONPARAMDB_TYPE_P_UQWRD          (COMMONPARAMDB_TYPE_POINTER_MASK|COMMONPARAMDB_TYPE_UQWRD)
#define COMMONPARAMDB_TYPE_P_SQWRD          (COMMONPARAMDB_TYPE_POINTER_MASK|COMMONPARAMDB_TYPE_SQWRD)
#define COMMONPARAMDB_TYPE_P_FLOAT          (COMMONPARAMDB_TYPE_POINTER_MASK|COMMONPARAMDB_TYPE_FLOAT)
#define COMMONPARAMDB_TYPE_P_DOUBL          (COMMONPARAMDB_TYPE_POINTER_MASK|COMMONPARAMDB_TYPE_DOUBL)

#define COMMONPARAMDB_TYPE_C_UBYTE          (COMMONPARAMDB_TYPE_CONST_MASK|COMMONPARAMDB_TYPE_UBYTE)
#define COMMONPARAMDB_TYPE_C_SBYTE          (COMMONPARAMDB_TYPE_CONST_MASK|COMMONPARAMDB_TYPE_SBYTE)
#define COMMONPARAMDB_TYPE_C_UWORD          (COMMONPARAMDB_TYPE_CONST_MASK|COMMONPARAMDB_TYPE_UWORD)
#define COMMONPARAMDB_TYPE_C_SWORD          (COMMONPARAMDB_TYPE_CONST_MASK|COMMONPARAMDB_TYPE_SWORD)
#define COMMONPARAMDB_TYPE_C_ULONG          (COMMONPARAMDB_TYPE_CONST_MASK|COMMONPARAMDB_TYPE_ULONG)
#define COMMONPARAMDB_TYPE_C_SLONG          (COMMONPARAMDB_TYPE_CONST_MASK|COMMONPARAMDB_TYPE_SLONG)
#define COMMONPARAMDB_TYPE_C_FLOAT          (COMMONPARAMDB_TYPE_CONST_MASK|COMMONPARAMDB_TYPE_FLOAT)

#define COMMONPARAMDB_BASETYPE_MAXSIZE      8
#define COMMONPARAMDB_BASETYPE_MAXTYPE      (COMMONPARAMDB_TYPE_BINARY)

//***************************************************************************
// Parameter Flags

#define COMMONPARAMDB_FLAG_RD               0x01
#define COMMONPARAMDB_FLAG_WR               0x02
#define COMMONPARAMDB_FLAG_RW               ( COMMONPARAMDB_FLAG_RD | COMMONPARAMDB_FLAG_WR )
#define COMMONPARAMDB_FLAG_UMCONV           0x04
#define COMMONPARAMDB_FLAG_HOOK             0x08
#define COMMONPARAMDB_FLAG_VALIDATE         0x10
#define COMMONPARAMDB_FLAG_RESETREQ         0x20
#define COMMONPARAMDB_FLAG_WRLOCKREQ        0x40

//***************************************************************************
// Hook callback operative flags

    // initialize transaction, in this case no data is
    // transferred, in case of write data pointer to data buffer
    // point to an ULONG that specify data trasfer size,
    // zero means unknown data size
#define COMMONPARAMDB_CBFLAG_INIT           0x0001
    // read/write chunk of data
#define COMMONPARAMDB_CBFLAG_SEGMENT        0x0002
    // end of transaction
#define COMMONPARAMDB_CBFLAG_END            0x0004
    // read transaction (must be set for all transaction states)
#define COMMONPARAMDB_CBFLAG_RD             0x0008
    // write transaction (must be set for all transaction states)
#define COMMONPARAMDB_CBFLAG_WR             0x0010
    // data size inquiry (when read), return ULONG value in the data
    // buffer transfer location and return sizeof(ULONG)
#define COMMONPARAMDB_CBFLAG_SIZEINQUIRY    0x0020
    // abort transaction on fieldbus event
#define COMMONPARAMDB_CBFLAG_ABORT          0x0040
    // no strict data size check/data conversion may be req'ed
#define COMMONPARAMDB_CBFLAG_NODSCHECK      0x0080
    // info parameter required
#define COMMONPARAMDB_CBFLAG_INFOREQ        0x0100
    // bit access, e.g. element and sizes are expressed
    // as no. of bits
#define COMMONPARAMDB_CBFLAG_BIT            0x0200

//***************************************************************************
// UM Conv operative flags

#define COMMONPARAMDB_UCFLAG_READ           (FALSE)
#define COMMONPARAMDB_UCFLAG_WRITE          (TRUE)

//***************************************************************************
// Db structure

typedef struct _commonparamdb_entry
{
    UWORD           uwIndex;            // internal index
    UBYTE           ubFlags;            // operative flags
    UBYTE           ubType;             // type
    SWORD           swMoreType;         // additional type def, depend from ubType spec:
                                        // if TYPE_BITW/BITD this specify bit number, if
                                        // -1 then BITW/BITD became array of bit
                                        // if COMMONPARAMDB_TYPE_P_* this is
                                        //      the displacement in byte
                                        // if BINARY this could be byte size of data (if >0)
                                        // if hook this could be hook additional parameter
    UWORD           uwNElements;        // No. of elements
    SYSTEMSTATUS    uWrDeny;            // write denial mask selection
    HPVOID          hpvData;            // data pointer to parameter

        // hook callback pointer, parameters are (order of appearance):
        // - pointer to COMMONPARAMDBENTRY that define the parameter
        // - flags and state of the connection
        // - requested element
        // - pointer to buffer for data transfer, always allocated by the caller
        // - pointer to buffer length, the caller send incoming buffer size,
        //       the callback return data size written in the buffer in case of read transaction
        // - pointer to general ULONG value to be used as callback internal status, when required,
        //       in order to have callback fully reentrant
        // in case of umconv local data read/write is performed by the callback
        // return 0 in case of successfull transaction, otherwise error code
        // umconv callback pointer, parameters are (order of appearance):
        // - conversion direction according to COMMONPARAMDB_UCFLAG_*
        // - near destination data pointer (if writing to internal write must be atomically)
        // - near source data pointer (if reading from internal read must be atomically)
        // return FALSE in case of successfull transaction
        // validate callback pointer, parameters are (order of appearance):
        // - source data pointer
        // return FALSE in case of successfull transaction
    union
    {
        UWORD   (  * fpfHook )( struct _commonparamdb_entry  *, UWORD, UWORD, HPVOID, UWORD *, HPULONG );
        UWORD   (  * fpfUmConv )( BOOL, void *, void * );
        UWORD   (  * fpfValidate )( void * );
    } uf;
} COMMONPARAMDB_ENTRY;

//***************************************************************************
// Common handling return values

#define COMMONPARAMDB_CH_OK                 0

#define COMMONPARAMDB_CH_HOOK_REQUIRED      1
#define COMMONPARAMDB_CH_EXTERN_HANDLER_REQ 2

#define COMMONPARAMDB_CH_NO_READ_ACCESS     10
#define COMMONPARAMDB_CH_NO_WRITE_ACCESS    11
#define COMMONPARAMDB_CH_INVALID_ELEMENT    12
#define COMMONPARAMDB_CH_LENGTHTOOLOW       13
#define COMMONPARAMDB_CH_WRONGLENGTH        14
#define COMMONPARAMDB_CH_WRITE_DENIED       15
#define COMMONPARAMDB_CH_INVALID_DATA       16
#define COMMONPARAMDB_CH_HARDWAREFAILURE    17
#define COMMONPARAMDB_CH_ELEMENT_NOTFOUND   18
#define COMMONPARAMDB_CH_NOTMAPPABLE        19
#define COMMONPARAMDB_CH_INVALID_ACCESS     20

#define COMMONPARAMDB_CH_SIGN_SIGNED        32
#define COMMONPARAMDB_CH_SIGN_UNSIGNED      33
#define COMMONPARAMDB_CH_SIGN_NA            34

//***************************************************************************
// Reference to physical parameter table

extern const COMMONPARAMDB_ENTRY  psCommonParamTable[];
extern const UWORD uwCommonParamCount;

//***************************************************************************
// Global functions

    // Table checking, just for debug purpose
#ifdef _APP_DEBUG
BOOL ComParDBCheckTable(void);
#endif

    // Parameter Table search
COMMONPARAMDB_ENTRY  * ComParDBEntrySearch(UWORD uwIndex);

//***************************************************************************
// Common access (read and write) for standard data (all above but
// COMMONPARAMDB_TYPE_STRING)
// Parameters are (order of appearance):
// - pointer to COMMONPARAMDBENTRY that define the parameter
// - selected element (0 to n-1)
// - pointer to caller allocated output/input buffer
// - pointer to buffer size, rewritten by the functions
// NB: if the entry is BITW or BITD, the exchanged data is one byte (like BOOL)

    // Check for common handling
UWORD ComParDBEntryCheckForCommonHandling(COMMONPARAMDB_ENTRY  * psEntry);

    // Common Read
UWORD ComParDBEntryRead(COMMONPARAMDB_ENTRY  * psEntry, UWORD uwElement, void * pvBufferDataOut, UWORD * puwBufferSize);

    // Common Write
UWORD ComParDBEntryWrite(COMMONPARAMDB_ENTRY  * psEntry, UWORD uwElement, void * pvBufferDataIn, UWORD * puwBufferSize);

    // Parameter Get Entry Size
UWORD ComParDBEntryGetEntrySize(COMMONPARAMDB_ENTRY  * psEntry);

    // Parameter Get Entry Signed/Unsigned
UWORD ComParDBEntryGetEntrySign(COMMONPARAMDB_ENTRY  * psEntry);

    // Parameter Get pointer for data
HPVOID ComParDBEntryGetPointer(COMMONPARAMDB_ENTRY  * psEntry, UWORD uwElement, HPUWORD hpuwOffset);

#endif
