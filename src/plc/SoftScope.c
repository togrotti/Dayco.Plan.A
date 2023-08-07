/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SoftScope.c                                                */
/* Author      : Fabio Terrile,Hu Xiaokai                                   */
/*               Axel                                                       */
/*                                                                          */
/* Description : Configuration and integration of SoftScope runtime         */
/*                                                                          */
/****************************************************************************/

#include <string.h>

#include "drive\AxM-E-Defines.h"
#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"
#include "SoftScope.h"
#include "common\TaskScheduler.h"
#include "Plc.h"
#include "bus\modbus\ModBusComDB.h"
#include "common\CommonParamDB.h"
#include "common\SerialFlashHandler.h"
#include "system\SysAppSFlashPartition.h"

//***************************************************************************
// Avoids warning C37: nonstandard extension - unnamed parameter

//#pragma warning disable = 37

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

//****************************************************************************
// Globals

ULONG           ulScoMemory[ SOFTSCOPE_SAMPLES_NUMBER ] __attribute__((section(".data_buff_section")));

UWORD           uwScoCommand;               // SoftScope command register
ULONG           ulScoData;                  // SoftScope data register

UWORD           uwScoTriggerRequiredMode;   // Required acquisition status
ULONG           ulScoAcquisitionId;         // Acquisition ID set by SoftScope
UWORD           uwScoTriggerStatus;         // Current acquisition status
ULONG           ulScoSampleCount;           // Counter of the samples acquired since the start of acquisition
ULONG           ulScoTriggerPosition;       // Sample position at the trigger event
ULONG           ulScoAcquiredDataId;        // Acquisition ID of the current sample buffer
#if ALSSC_RUNTIME_ADV
ULONG           ulScoAbsTimeRef;            // Absolute time reference
ULONG           ulScoRTBuffSwitch;
#endif

//****************************************************************************
// Local variables
static ULONG  * pulSampleBuffer=ulScoMemory;
static ULONG    ulSampleSize=SOFTSCOPE_SAMPLES_NUMBER;
static BOOL     bInitialized=FALSE;

//****************************************************************************
// Definition for runtime

#define DESCRIPTION_OF_SSC_RUNTIME_CONFIGURATION    (0)

/*  Standard definitions    */
#if ALSSC_RUNTIME_ADV
#include "AlPlcRuntime5/AlPlcCDefs.h"

/*	New Features	*/
#define SSC_FEATURES            SSC_FT_BUFFEREDMODE // | SSC_FT_REALTIMEMODE | SSC_FT_REALTIMEPUSHMODE

/*	Session storage supported	*/
//#define SSC_SAVEACQ_SUPPORTED	TRUE

/*	Session restart acquisition supported	*/
//#define SSC_RESTARTACQ_SUPPORTED	TRUE

/*	Real-time mode acquisition supported	*/
//#define SSC_REALTIME_ACQUIRE	TRUE

/*  Number of samples in the buffer */
#define SSC_NUM_SAMPLES         SOFTSCOPE_SAMPLES_NUMBER
#else
#include "AlPlcRuntime2/AlPlcCDefs.h"

/*  Pointer to sample buffer (if not defined then runtime define it) */
#define SSC_SAMPLES_BUFFER      pulSampleBuffer

/*  Sample buffer pointer data type */
#define SSC_SAMPLES_MQUALIFIER  uint32_t  *
#endif

#include "AISSRunTime/AlSSCTarg.h"

/*  Number of SoftScope tracks  */
#define SSC_NUMTRACKS           8           /*  It shold be 1, 2, 4, 6 or 8*/

/*  Acquisition time base in ns */
#define SSC_TIME_BASE           125000

/*  SoftScope command registers */
#define SSC_R_COMMAND           uwScoCommand        /*  SoftScope command register  */
#define SSC_R_DATA              ulScoData           /*  SoftScope data register     */

/*  Control and status registers    */
#define SSC_R_W_TRIGGERREQUIREDMODE     uwScoTriggerRequiredMode    /*  W - Required acquisition status */
#define SSC_R_DW_ACQUISITIONID          ulScoAcquisitionId          /*  W - Acquisition ID set by SoftScope */
#define SSC_R_W_TRIGGERSTATUS           uwScoTriggerStatus          /*  R - Current acquisition status */
#define SSC_R_DW_SAMPLECOUNT            ulScoSampleCount            /*  R - Counter of the samples acquired since the start of acquisition */
#define SSC_R_DW_TRIGGERPOSITION        ulScoTriggerPosition        /*  R - Sample position at the trigger event */
#define SSC_R_DW_ACQUIREDDATAID         ulScoAcquiredDataId         /*  R - Acquisition ID of the current sample buffer */
#if ALSSC_RUNTIME_ADV
#define SSC_R_DW_ABSTIMEREF             ulScoAbsTimeRef             /*  RW - Absolute time reference */
#if defined( SSC_LOGGING_SUPPORTED )
#define	SSC_R_DW_SAVEDLOGID				m_dwSavedLogId				/*	RW - Save log identifier */
#endif	//	SSC_LOGGING_SUPPORTED
#endif

#define SSC_GET_PHYSADDR(adr)           get_physaddr(adr)           /*  Function to resolve physical addresses  */
#define SSC_GET_DBADDR(dbt,dbIdx,off)   get_dbaddr(dbt,dbIdx,off)   /*  Function to resolve data blocks addresses   */
#define SSC_GET_OBJADDR(idx,sub)        get_objaddr(idx,sub)        /*  Function to resolve object dictionary addresses */

#if ALSSC_RUNTIME_ADV
#define SSC_GET_OBJTYPES(idx,sub,tPar,tVar) get_objtypes(idx,sub,tPar,tVar) /*  Function to resolve object dictionary types */
#define SSC_GET_OBJSCALE(idx,sub,scale)     get_objscale(idx,sub,scale)     /*  Function to resolve object dictionary scale */
#define SSC_GET_OBJOFFSET(idx,sub,offset)   get_objoffset(idx,sub,offset)   /*  Function to resolve object dictionary offset    */
#else
#define SSC_GET_OBJTYPES(idx,sub,tPar,tVar) (0)                     /*  Function to resolve object dictionary types */
#define SSC_GET_OBJSCALE(idx,sub,scale)     (0)                     /*  Function to resolve object dictionary scale */
#define SSC_GET_OBJOFFSET(idx,sub,offset)   (0)                     /*  Function to resolve object dictionary offset    */
#endif

/*  Entry points */
#define ALSSCINIT                       _SSCInit
#define ALSSCMANAGE                     _SSCManage
#define ALSSCACQUIRE                    _SSCAcquire

#if ALSSC_RUNTIME_ADV
#if defined( SSC_SAVEACQ_SUPPORTED )
#define ALSSCLOAD                       _SSCLoad
#if defined( SSC_RESTARTACQ_SUPPORTED )
#define ALSSCRESTART                    _SSCRestart
#endif	//	SSC_RESTARTACQ_SUPPORTED
#endif	//	SSC_SAVEACQ_SUPPORTED
#if defined( SSC_REALTIME_ACQUIRE )
#define ALSSCRTDATASWITCH               _SSCDataSwitch
#endif	//	SSC_REALTIME_ACQUIRE
#endif

//****************************************************************************
// Locals

static void ALSSCINIT(void);
static void ALSSCACQUIRE(void);
static void ALSSCMANAGE(void);

#if ALSSC_RUNTIME_ADV
#if defined( SSC_SAVEACQ_SUPPORTED )
static void ALSSCLOAD(void);
#if defined( SSC_RESTARTACQ_SUPPORTED )
static void ALSSCRESTART(void);
#endif	//	SSC_RESTARTACQ_SUPPORTED
#endif	//	SSC_SAVEACQ_SUPPORTED
#if defined( SSC_REALTIME_ACQUIRE )
static uint32_t ALSSCRTDATASWITCH(void);
#endif	//	SSC_REALTIME_ACQUIRE
#endif

//****************************************************************************
// Init entry point

BOOL SoftScopeInit(UWORD t)
{
        // mark as initialized to disable plc memory request
    bInitialized=TRUE;

        // check available # of samples for the buffer, if below
        // 2 * SSC_NUMTRACKS skip module activation
    if(ulSampleSize<SSC_NUMTRACKS*2)
        return TRUE;

        // Init runtime
    ALSSCINIT();

        // add slow task for management
    assert(TaskSched_AddBackgroundTask(&ALSSCMANAGE));

        // add realtime task for acquire process
    assert(TaskSched_AddRTTask((BOOL (*)(void))&ALSSCACQUIRE, TASKSCHEDULER_FLAG_NONE, \
            0, \
            SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_RESETTING), 0));

    return TRUE;
}

#if SOFTSCOPE_REQ_MEMORY
//****************************************************************************
// Request fixed memory area from sample memory area for PLC

BOOL SoftScopeRequestMemory(ULONG ulByteSize)
{
        // after softscope initialization access is denied
    if(bInitialized)
        return FALSE;

        // if exceed then deny
    if(ulByteSize>(ULONG)SOFTSCOPE_SAMPLES_NUMBER*sizeof(ULONG))
        return FALSE;

        // set 32bit alignment for start buffer adjustment
    if(ulByteSize%sizeof(ULONG))
        ulByteSize+=sizeof(ULONG)-(ulByteSize%sizeof(ULONG));

        // size as 32bit elements
    ulByteSize/=sizeof(ULONG);
    pulSampleBuffer=&ulScoMemory[ulByteSize];

        // adjust number of samples available, that
        // must be multiple of SSC_NUMTRACKS
    ulSampleSize-=ulByteSize;
    if(ulSampleSize%SSC_NUMTRACKS)
        ulSampleSize-=ulSampleSize%SSC_NUMTRACKS;

        // clear reserved memory
    memset(ulScoMemory, 0, ulByteSize*sizeof(ULONG));
 
    return TRUE;
}

//****************************************************************************
// Store plc requested memory to flash

BOOL SoftScopeReqMemSave(void)
{
   ULONG ulChk;

       // check right drive state
   if(!Os_IsInBackground())
       return FALSE;

       // sector erase
   if(!SerialFlashEraseSector(SFPART_PLCREQMEM_START))
       return FALSE;

       // write data
   for(ulChk=0ul; ulChk<(ULONG)pulSampleBuffer-(ULONG)ulScoMemory; ulChk+=SFLASH_PAGE_SIZE)
       if(!SerialFlashWriteBytes(SFPART_PLCREQMEM_START+ulChk, &(((UBYTE *)ulScoMemory)[ulChk]), SFLASH_PAGE_SIZE))
           return FALSE;

   return TRUE;
}

//****************************************************************************
// Restore plc requested memory to flash, consistency is plc
// application dependant

BOOL SoftScopeReqMemLoad(void)
{
       // check right drive state
   if(!Os_IsInBackground())
       return FALSE;

       // read data
   return SerialFlashReadBytes(SFPART_PLCREQMEM_START, (UBYTE *)ulScoMemory, (UWORD)((ULONG)pulSampleBuffer-(ULONG)ulScoMemory));
}

//****************************************************************************
// Hook for modbus management of plc requested memory

UWORD SoftScopeReqMemParHook(COMMONPARAMDB_ENTRY  *p, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext)
{
    UWORD uwNElem=(UWORD)((ULONG)pulSampleBuffer-(ULONG)ulScoMemory)/sizeof(UWORD);
    UWORD uwOffset;

        // if abort do nothing
    if(uwFlags&COMMONPARAMDB_CBFLAG_ABORT)
        return COMMONPARAMDB_CH_OK;

        // check element access and calculate initial offset (term of words)
    if(uwFlags&COMMONPARAMDB_CBFLAG_BIT)
        uwOffset=uwElement/(sizeof(UWORD)*8);
    else
        uwOffset=uwElement;

        // check size
    if(uwOffset>=uwNElem)
        return COMMONPARAMDB_CH_INVALID_ELEMENT;

        // if data read
    if(uwFlags&COMMONPARAMDB_CBFLAG_RD)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
                // left data size
            *hpulContext=(uwNElem-uwOffset)*sizeof(UWORD);

                // data size if requested
            if(uwFlags&COMMONPARAMDB_CBFLAG_SIZEINQUIRY)
            {
                *((HPULONG)hpvBuffer)=*hpulContext;
                *puwBufSize=sizeof(ULONG);
            }
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            if(uwFlags&COMMONPARAMDB_CBFLAG_BIT)
            {
                UWORD uwSize,uwRegOffset,uwCt;
                UBYTE ubSwap;
                HPUBYTE hpubDataStart;
    
                    // here supported one segment per transaction
                uwSize=(UWORD)*hpulContext;
                if(*hpulContext==0ul)
                    return COMMONPARAMDB_CH_INVALID_ACCESS;

                    // zeroes context for next segment failure detection
                *hpulContext=0ul;

                    // check requested amount of data is allowed
                if(*puwBufSize>(ULONG)uwSize*sizeof(UWORD)*8)
                    return COMMONPARAMDB_CH_WRONGLENGTH;

                    // data chunk start
                hpubDataStart=&(((UBYTE  *)ulScoMemory)[uwElement/(sizeof(UBYTE)*8)]);

                    // calculate number of shift to be applied
                uwRegOffset=uwElement%(sizeof(UBYTE)*8);

                    // calculate number of bytes involved
                uwSize=(uwRegOffset+*puwBufSize+7)/8;
            
                    // align output bits
                for(uwCt=0;uwCt<uwSize;uwCt++)
                {
                    ubSwap =hpubDataStart[uwCt]   >> uwRegOffset;
                    ubSwap|=hpubDataStart[uwCt+1] << (8-uwRegOffset);
                    ((HPUBYTE)hpvBuffer)[uwCt] = ubSwap;
                }
        
                    // zeroes extra bits if necessary
                uwSize=*puwBufSize%8;
                if(uwSize)
                    ((HPUBYTE)hpvBuffer)[*puwBufSize/8]&=(UBYTE)(((UWORD)0x01<<uwSize)-1);
            }
            else
            {
                UWORD uwSize;
                HPUBYTE hpuwDataStart;
    
                    // data size left
                uwSize=(UWORD)*hpulContext;
    
                    // data chunk start
                hpuwDataStart=&(((UBYTE  *)ulScoMemory)[uwNElem*sizeof(UWORD)-*hpulContext]);
    
                    // check dest buffer
                if(uwSize>*puwBufSize)
                    uwSize=*puwBufSize;
    
                    // copy data chunk
                memcpy(hpvBuffer, hpuwDataStart, uwSize);
    
                    // update data out buffer size
                *puwBufSize=uwSize;
    
                    // update left size
                *hpulContext=*hpulContext-(ULONG)uwSize;
            }
        }
    }

        // if data write
    if(uwFlags&COMMONPARAMDB_CBFLAG_WR)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
                // left data size
            *hpulContext=(uwNElem-uwElement)*sizeof(UWORD);

                // if write request is more than available
            if(uwFlags&COMMONPARAMDB_CBFLAG_BIT)
            {
                if(*((HPULONG)hpvBuffer) > (*hpulContext*sizeof(UWORD)*8))
                    return COMMONPARAMDB_CH_WRONGLENGTH;
            }
            else
            {
                if(*((HPULONG)hpvBuffer) > *hpulContext)
                    return COMMONPARAMDB_CH_WRONGLENGTH;
            }
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            if(uwFlags&COMMONPARAMDB_CBFLAG_BIT)
            {
                UWORD uwSize,uwCt;
                UWORD uwInMask,uwOutMask;
                HPUBYTE hpubDataStart;
                HPUBYTE hpubInBuffer=(HPUBYTE)hpvBuffer;
    
                    // here supported one segment per transaction
                uwSize=(UWORD)*hpulContext;
                if(*hpulContext==0ul)
                    return COMMONPARAMDB_CH_INVALID_ACCESS;

                    // zeroes context for next segment failure detection
                *hpulContext=0ul;

                    // check requested amount of data is allowed
                if(*puwBufSize>(ULONG)uwSize*sizeof(UWORD)*8)
                    return COMMONPARAMDB_CH_WRONGLENGTH;

                    // data chunk start
                hpubDataStart=&(((UBYTE  *)ulScoMemory)[uwElement/(sizeof(UBYTE)*8)]);
                uwElement%=sizeof(UBYTE)*8;

                    // prepare masks
                uwInMask=0x0001;
                uwOutMask=0x0001<<uwElement;
        
                    // write each single bit
                for(uwCt=0;uwCt<*puwBufSize;uwCt++,uwInMask<<=1,uwOutMask<<=1)
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
                        hpubDataStart++;
                    }
                    
                    atomic_byte_write_bits(hpubDataStart, (UBYTE)((*hpubInBuffer&uwInMask)?uwOutMask:0), (UBYTE)uwOutMask);
                }
            }
            else
            {
                UWORD uwSize;
                HPUBYTE hpuwDataStart;
    
                    // data size left
                uwSize=(UWORD)*hpulContext;
    
                    // data chunk start
                hpuwDataStart=&(((UBYTE  *)ulScoMemory)[uwNElem*sizeof(UWORD)-*hpulContext]);
    
                    // check dest buffer
                if(uwSize>*puwBufSize)
                    uwSize=*puwBufSize;
    
                    // copy data chunk
                memcpy(hpuwDataStart, hpvBuffer, uwSize);
    
                    // update data out buffer size
                *puwBufSize=uwSize;
    
                    // update left size
                *hpulContext=*hpulContext-(ULONG)uwSize;
            }
        }
    }

    return COMMONPARAMDB_CH_OK;
}
#else

//****************************************************************************
// Request fixed memory area from sample memory area for PLC

BOOL SoftScopeRequestMemory(ULONG ulByteSize)
{
    return TRUE;
}

//****************************************************************************
// Store plc requested memory to flash

BOOL SoftScopeReqMemSave(void)
{
    return TRUE;
}

//****************************************************************************
// Restore plc requested memory to flash, consistency is plc
// application dependant

BOOL SoftScopeReqMemLoad(void)
{
    return TRUE;
}

//****************************************************************************
// Hook for modbus management of plc requested memory

UWORD SoftScopeReqMemParHook(COMMONPARAMDB_ENTRY  *p, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext)
{
    return COMMONPARAMDB_CH_OK;
}

#endif

//****************************************************************************
// check if 'near' physical address

static ULONG * get_physaddr(ULONG ulAddress)
{
    ULONG * pulLocal;

        // make double conversion,  -> near and back
    pulLocal=(ULONG *)(ULONG  *)ulAddress;

        // if back conversion same as originating one then address is ok
    if((ULONG)pulLocal==ulAddress)
        return pulLocal;
    else
        return NULL;
}

//****************************************************************************
// translate data block addressing 

static ULONG * get_dbaddr(UBYTE ubDbType, UWORD uwDbIdx, UWORD uwDbOffset)
{
    ULONG address;

        // get address
    address=(ULONG)PlcIECDBEntrySearch(ubDbType, uwDbIdx, uwDbOffset);

        // if not NULL traslate to near
    if(address)
        return get_physaddr(address);
    else
        return NULL;
}

//****************************************************************************
// translate fieldbus addressing (modbus)

static ULONG * get_objaddr(UWORD uwIndex, UWORD uwSubIndex)
{
    const MODBUSCOMDB_ENTRY  * pElem;
    ULONG address;
    UWORD uwDummy;
    
        // subindex not supported
    if(uwSubIndex!=0)
        return NULL;
    
        // db search, if not found
    if((pElem=ModBusComDBEntrySearch(uwIndex))==NULL)
        return NULL;

        // traslate address
    address=(ULONG)ComParDBEntryGetPointer(pElem->hpsComDBEntry, uwIndex - pElem->uwIndex, &uwDummy);

        // if not NULL traslate to near
    if(address)
        return get_physaddr(address);
    else
        return NULL;
}

#if ALSSC_RUNTIME_ADV
static uint8_t ConvertDBTypeToSSCType(uint8_t dbType)
{
    switch (dbType)
    {
    case COMMONPARAMDB_TYPE_UBYTE:  // 1
		return tyvByte;

    case COMMONPARAMDB_TYPE_SBYTE:  // 2
		return tyvByte;

    case COMMONPARAMDB_TYPE_UWORD:  // 3
		return tyvUInt;

    case COMMONPARAMDB_TYPE_SWORD:  // 4
		return tyvInt;

    case COMMONPARAMDB_TYPE_ULONG:  // 5
		return tyvUDInt;

    case COMMONPARAMDB_TYPE_SLONG:  // 6
		return tyvDInt;

    case COMMONPARAMDB_TYPE_UQWRD:  // 7
		return tyvULInt;

    case COMMONPARAMDB_TYPE_SQWRD:  // 8
		return tyvLInt;

    case COMMONPARAMDB_TYPE_FLOAT:  // 9
		return tyvReal;

    case COMMONPARAMDB_TYPE_DOUBL:  // 10
		return tyvLReal;

    case COMMONPARAMDB_TYPE_BITW:   // 11
		return tyvBool;

    case COMMONPARAMDB_TYPE_BITD:   // 12
		return tyvBool;

    case COMMONPARAMDB_TYPE_STRING: // 13
		return tyvString;

    case COMMONPARAMDB_TYPE_BINARY: // 14
		return tyvBool;
	}

	return tyvNull;	//	not managed
}

//****************************************************************************
// translate fieldbus addressing types
static bool_t get_objtypes( uint16_t idx, uint16_t sub, uint8_t * typePar, uint8_t * typeVar )
{
    const MODBUSCOMDB_ENTRY  * pElem;

        // subindex not supported
    if(sub!=0)
        return NULL;
    
        // db search, if not found
    if((pElem=ModBusComDBEntrySearch(idx))==NULL)
        return NULL;

    *typePar = ConvertDBTypeToSSCType( pElem->hpsComDBEntry->ubType );
    *typeVar = ConvertDBTypeToSSCType( pElem->hpsComDBEntry->ubType );
    return TRUE;
}

//****************************************************************************
// translate fieldbus addressing scale
static bool_t get_objscale( uint16_t idx, uint16_t sub, float32_t * scale )
{
    *scale = 1;
    return TRUE;
}

//****************************************************************************
// translate fieldbus addressing offset
static bool_t get_objoffset( uint16_t idx, uint16_t sub, float32_t * offset )
{
    *offset = 0;
    return TRUE;
}

//****************************************************************************
//
UWORD SoftScopeDataSwitch(COMMONPARAMDB_ENTRY  *pElem, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext)
{
#if defined( SSC_REALTIME_ACQUIRE )
    ulScoRTBuffSwitch = _SSCDataSwitch();
#endif
    return COMMONPARAMDB_CH_OK;
}

#endif

//****************************************************************************
// Runtime generation

// Avoids warning C183: dead assignment eliminated

//#pragma warning disable = 183

#include "AISSRunTime/AlSSCRuntimeCore.lib"
