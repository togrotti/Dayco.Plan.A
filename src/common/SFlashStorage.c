/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2016, Phase Motion Control. All Rights Reserved.             */
/*                                                                          */
/* File        : SFlashStorage.c                                            */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Serial Flash file storage manager                          */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "SFlashStorage.h"

#ifdef _INFINEON_
//****************************************************************************
// Seek Init

LOGICAL_BLOCK_HEADER * SFStor_SeekInit(SFSTOR_STATUS * psStatus, ULONG ulBaseAddress, ULONG ulSize)
{
    // initialize data structure
    psStatus->ulAddress = ulBaseAddress;
    psStatus->ulEndAddress = ulBaseAddress + ulSize;
    psStatus->sHeader.sParameters.ulTotalByteLen = 0ul;

    return SFStor_SeekNext(psStatus);
}

//****************************************************************************
// Seek Next

LOGICAL_BLOCK_HEADER * SFStor_SeekNext(SFSTOR_STATUS * psStatus)
{
    // compute next valid address from previous valid data block
    psStatus->ulAddress += psStatus->sHeader.sParameters.ulTotalByteLen;

    // check boundaries
    if( psStatus->ulAddress + sizeof( psStatus->sHeader ) >= psStatus->ulEndAddress )
        return NULL;

    // get header
    if( !SerialFlashReadBytes( psStatus->ulAddress, (UBYTE *)&(psStatus->sHeader), sizeof( psStatus->sHeader ) ) )
        return NULL;
 
    // initialize and check header
    if( SecurityCheckLogicalBlockInit( &(psStatus->sHeader), CHECK_LOGICAL_BLOCK_HEADER, &(psStatus->ulStreamStart),
                                       &(psStatus->ulSizeLeft), &(psStatus->sHash)) != CHECK_LOGICAL_BLOCK_SUCCESSFULLY )
        return NULL;

    return &(psStatus->sHeader);
}

//****************************************************************************
// Get data block boundaries

void SFStor_GetBoundaries(SFSTOR_STATUS * psStatus, ULONG * pulAddress, ULONG * pulSize)
{
    // compute start address and size
    *pulAddress = psStatus->ulAddress + psStatus->sHeader.sParameters.ulEntryAddress - psStatus->sHeader.sParameters.ulStartAddress;
    *pulSize = psStatus->ulSizeLeft;
}

//****************************************************************************
// Data streaming begin

SWORD SFStor_StreamBegin(SFSTOR_STATUS * psStatus, UBYTE * pubBuffer)
{
    SLONG slSkipSz;
    SWORD swRetVal;

    // compute skip data for stream
    slSkipSz = psStatus->ulStreamStart - psStatus->sHeader.sParameters.ulStartAddress;

    // compute start address from flash
    psStatus->ulAddress += slSkipSz;

    // calculate skip data size (if needed) before streaming
    slSkipSz = psStatus->sHeader.sParameters.ulEntryAddress - psStatus->sHeader.sParameters.ulStartAddress - slSkipSz;

    // then prepare fast sequential read from serial flash
    if( !SerialFlashReadSeqInit(psStatus->ulAddress) )
        return SFSTOR_STR_FLASHERROR;

    // read and update data checking until stream begin
    while( slSkipSz > 0 )
    {
        swRetVal=SFStor_StreamGetData(psStatus, pubBuffer);
        if(swRetVal<0)
            return swRetVal;
        
        slSkipSz-=swRetVal;
        
        // must be multiple of SFSTOR_DESTBUFSIZE
        assert(slSkipSz>=0);
    } 

    return SFSTOR_STR_OK;
}

//****************************************************************************
// Data streaming

SWORD SFStor_StreamGetData(SFSTOR_STATUS * psStatus, UBYTE * pubBuffer)
{
    ULONG ulSize;

    // check if data stream not finished
    if(psStatus->ulSizeLeft == 0)
        return SFSTOR_STR_NOMOREDATA;

    // read from serial flash
    SerialFlashReadSeqGetBlock( pubBuffer );

    // calculate data size according to initial size and data left
    ulSize = psStatus->ulSizeLeft>SFSTOR_DESTBUFSIZE?SFSTOR_DESTBUFSIZE:psStatus->ulSizeLeft;
    psStatus->ulSizeLeft -= ulSize;

    // add data to check
    if( SecurityCheckLogicalBlockUpdate( pubBuffer, ulSize, &(psStatus->sHash)) != CHECK_LOGICAL_BLOCK_SUCCESSFULLY )
        return SFSTOR_STR_DATAINVALID;

    return (SWORD)ulSize;
}

//****************************************************************************
// Data streaming end and check

SWORD SFStor_StreamEnd(SFSTOR_STATUS * psStatus, UBYTE * pubBuffer, BOOL bAbort)
{
    SWORD sRetVal;

    // if abort then exit without further checking, closing serial flash
    if(bAbort)
    {
        SerialFlashReadSeqFinal();
        return SFSTOR_STR_OK;
    }

    // if data left then readback remaining to validate
    while(psStatus->ulSizeLeft)
        if((sRetVal=SFStor_StreamGetData(psStatus, pubBuffer))<=0)
        {
            SerialFlashReadSeqFinal();
            return sRetVal;
        }

    // close serial flash data stream
    SerialFlashReadSeqFinal();

    // then make final check to the data block
    if( SecurityCheckLogicalBlockFinal( &(psStatus->sHeader), CHECK_LOGICAL_BLOCK_HEADER, &(psStatus->sHash)) != CHECK_LOGICAL_BLOCK_SUCCESSFULLY )
        return SFSTOR_STR_DATAINVALID;

    return SFSTOR_STR_OK;
}
#endif

//****************************************************************************
// Data streaming write begin

SWORD SFStor_WriteBegin(SFSTOR_WRSTATUS * psWrStatus, ULONG ulBaseAddress, ULONG ulMaxSize)
{
    // init status
    psWrStatus->uwBufSz=0;
    psWrStatus->ulAddress=ulBaseAddress;
    psWrStatus->ulEndAddress=ulBaseAddress+ulMaxSize-1;

    // prepare serial flash erasing first sector
    if(!SerialFlashEraseSectorEx(ulBaseAddress, TRUE))
        return SFSTOR_STR_FLASHERROR;

    return SFSTOR_STR_OK;
}

//****************************************************************************
// Data streaming write data

SWORD SFStor_WriteData(SFSTOR_WRSTATUS * psWrStatus, HPUBYTE pubBuffer, UWORD uwSize)
{
    UBYTE * pubPos;
    ULONG ulPrevAddr;

    // if not init
    if(psWrStatus->ulAddress==0ul)
        return SFSTOR_STR_NOMOREDATA;

    // if out of space
    if(psWrStatus->ulAddress>psWrStatus->ulEndAddress)
        return SFSTOR_STR_OUTOFSPACE; 

    // queue data in local buffer
    pubPos=&(psWrStatus->ubBuf[psWrStatus->uwBufSz]);
    while(uwSize>0)
    {
        // add byte
        *pubPos++=*pubBuffer++;
        psWrStatus->uwBufSz++;
        uwSize--;

        // when local buffer is full
        if(psWrStatus->uwBufSz>=SFSTOR_WRITEBUFSIZE)
        {
            // write data
            if(!SerialFlashWriteBytesEx(psWrStatus->ulAddress,psWrStatus->ubBuf,SFSTOR_WRITEBUFSIZE,TRUE))
                return SFSTOR_STR_FLASHERROR;

            // adjust pointers/size
            ulPrevAddr=psWrStatus->ulAddress;
            psWrStatus->ulAddress+=SFSTOR_WRITEBUFSIZE;
            psWrStatus->uwBufSz=0;
            pubPos=psWrStatus->ubBuf;

            // if next valid page exceed allowed space then exit here
            if(psWrStatus->ulAddress>psWrStatus->ulEndAddress)
                return SFSTOR_STR_OK; 

            // check for sector change, in case prepare next sector
            if((ulPrevAddr&~(SFLASH_SECTOR_SIZE-1))!=(psWrStatus->ulAddress&~(SFLASH_SECTOR_SIZE-1)))
                if(!SerialFlashEraseSectorEx(psWrStatus->ulAddress, TRUE))
                    return SFSTOR_STR_FLASHERROR;
        }
    }

    return SFSTOR_STR_OK;
}

//****************************************************************************
// Data streaming write end (flush)

SWORD SFStor_WriteEnd(SFSTOR_WRSTATUS * psWrStatus)
{
    // if not init
    if(psWrStatus->ulAddress==0ul)
        return SFSTOR_STR_NOMOREDATA;

    // flush buffer, if not zero
    if(psWrStatus->uwBufSz>0)
        if(!SerialFlashWriteBytesEx(psWrStatus->ulAddress,psWrStatus->ubBuf,psWrStatus->uwBufSz,TRUE))
            return SFSTOR_STR_FLASHERROR;

    // disable streaming
    psWrStatus->ulAddress=0ul;

    // and wait for serial flash become idle
//    SerialFlashWaitForIdle();

    return SFSTOR_STR_OK;
}
