/*****************************************************************************
  Progetto:	Motore Integrato/Matrix
******************************************************************************
  Autore:	Terrile Fabio
  Descr:	CoE (CANopen over EtherCAT) - SDO Server
  			conversione unita' di misura
******************************************************************************
\defgroup sdoserv sdoserv.c: CoE (CANopen over EtherCAT) - SDO Server
\brief The SDO server handles all sdo and sdo information services\n 
\brief Changes to version V3.20:\n
\brief Changes to version V3.20:\n
\brief V4.00 SDO 1: The size of the written data in case of a SDO Download will be
\brief              in the function OBJ_Write to be more flexible
\brief V4.00 SDO 2: The object lists will be generated in the functions OBJ_GetNoOfObjects
\brief              and OBJ_GetObjectList in objdef.c to decouple the SDO services from
\brief              the implementation of the object dictionary
\brief V4.00 SDO 3: The name of an object or entry description will only be transmitted
\brief              if it fits in the mailbox because the fragmentation is not supported in the sample code,\n
\brief V4.00 SDO 4: SDOs with size greater than 65535 were not handled correctly, that is fixed now

\author  Holger Buettner
\version 4.00
\date    23.03.2007
*****************************************************************************/

/*---------------------------------------------------------------------------------------
------	
------	Includes
------ 
---------------------------------------------------------------------------------------*/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

#include <stdlib.h>
#include <string.h>

#include "common\CommonDefines.h"
#include "ecat_def.h"

#if COE_SUPPORTED

#define _SDOSERV_ 1
#include "sdoserv.h"
#undef  _SDOSERV_
#define _SDOSERV_ 0

#include "bus\canopen\CanOpenDs301.h"
#include "bus\canopen\CanOpenDs301Externals.h"
#include "bus\canopen\CanOpenComDB.h"
#include "bus\canopen\CanOpenComDBInfo.h"

#if COMPLETE_ACCESS_SUPPORTED
#if !SEGMENTED_SDO_SUPPORTED
	!!! ERROR, if switch COMPLETE_ACCESS_SUPPORTED is set, the switch SEGMENTED_SDO_SUPPORTED has to be set too !!!
#endif
#endif

/*---------------------------------------------------------------------------------------
------	
------	Modulintern variable definitions
------	
---------------------------------------------------------------------------------------*/

const UINT32 MBXMEM cAbortCode[] =
{
	ABORT_NOERROR,
	ABORT_TOGGLE_BIT_NOT_CHANGED,
	ABORT_SDO_PROTOCOL_TIMEOUT,
	ABORT_COMMAND_SPECIFIER_UNKNOWN,
	ABORT_OUT_OF_MEMORY,
	ABORT_UNSUPPORTED_ACCESS,
	ABORT_WRITE_ONLY_ENTRY,
	ABORT_READ_ONLY_ENTRY,
	ABORT_OBJECT_NOT_EXISTING,
	ABORT_OBJECT_CANT_BE_PDOMAPPED,
	ABORT_MAPPED_OBJECTS_EXCEED_PDO,
	ABORT_PARAM_IS_INCOMPATIBLE,
	ABORT_INTERNAL_DEVICE_INCOMPATIBILITY,
	ABORT_HARDWARE_ERROR,
	ABORT_PARAM_LENGTH_ERROR,
	ABORT_PARAM_LENGTH_TOO_LONG,
	ABORT_PARAM_LENGTH_TOO_SHORT,
	ABORT_SUBINDEX_NOT_EXISTING,
	ABORT_VALUE_EXCEEDED,
	ABORT_VALUE_TOO_GREAT,
	ABORT_VALUE_TOO_SMALL,
	ABORT_MAX_VALUE_IS_LESS_THAN_MIN_VALUE,
	ABORT_GENERAL_ERROR,
	ABORT_DATA_CANNOT_BE_READ_OR_STORED,
	ABORT_DATA_CANNOT_BE_READ_OR_STORED_BECAUSE_OF_LOCAL_CONTROL,
	ABORT_DATA_CANNOT_BE_READ_OR_STORED_IN_THIS_STATE,
	ABORT_NO_OBJECT_DICTIONARY_IS_PRESENT								
};

static UINT8							nSdoInfoFragmentsLeft;
static UWORD				            tSdoInfoContext;

#if SEGMENTED_SDO_SUPPORTED
static UINT8							nSdoSegService;
static INT8								bSdoSegFollows;
static UINT8							bSdoSegAccess;
/* ECATCHANGE_START(V4.00) SDO 4 */
static UINT32 							nSdoSegBytesToHandle;
/* ECATCHANGE_END(V4.00) SDO 4 */
static UINT8 							bSdoSegLastToggle;
/* ECATCHANGE_START(V4.00) SDO 4 */
static UINT32 							nSdoSegCompleteSize;
/* ECATCHANGE_END(V4.00) SDO 4 */
#endif // #if SEGMENTED_SDO_SUPPORTED

struct __stato
{
	unsigned toggle:1;
	unsigned segment:1;
	unsigned first:1;
	unsigned last:1;
	unsigned dispatchinit:1;
	unsigned expedited:1;
	unsigned download:1;
	unsigned upload:1;
	unsigned zerosize:1;
	unsigned segdispatchpending:1;
} tSdoSegState={0,0,0,0,0,0,0,0,0,0};

static DS301_SDOTRANSACTION tSdoTranstate;

#if SEGMENTED_SDO_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pSdoInd	Pointer to the received mailbox data from the master. 

 \return	Indicates if an error occured while the operation ( good = 0 ).

 \brief	This function is called when a Download SDO Segment request 
			service is received from the master. If its the last segment
			the data will be written to the object dictionary. The 
			function sends a response by itsself.
*////////////////////////////////////////////////////////////////////////////////////////

static UINT32 SdoDownloadSegmentInd( TDOWNLOADSDOSEGREQMBX MBXMEM * pSdoInd )
{
	UINT8 abort = 0; 
	UINT32 abortcode = ABORT_NOERROR;
/* ECATCHANGE_START(V4.00) SDO 4 */
	UINT32 bytesToSave = 0;
/* ECATCHANGE_END(V4.00) SDO 4 */

	if ( (pSdoInd->SdoHeader.SegHeader & SEGHEADER_TOGGLE) == bSdoSegLastToggle )
	{
		/* toggle bit has not toggled... */
		abortcode = ABORT_TOGGLE_BIT_NOT_CHANGED;
	}
	else
	{
		/* maxData contains the maximum data to be received with a SDO-DownloadSegment */
		UINT16 maxData =	u16ReceiveMbxSize - SIZEOF( UMBXHEADER ) - SEGMENT_NORM_HEADER_SIZE;
		/* the new toggle bit is stored in bSdoSegLastToggle */
		bSdoSegLastToggle = (UINT8)(pSdoInd->SdoHeader.SegHeader & SEGHEADER_TOGGLE);

		/* bytesToSave contains the remaining data with this and maybe the following 
		   SDO-Download Segment services */
		bytesToSave = nSdoSegCompleteSize - nSdoSegBytesToHandle;

		if ( pSdoInd->SdoHeader.SegHeader & SEGHEADER_NOMOREFOLLOWS )
		{
			/* the last segment is received, check if the length of the remaining data is the 
			   same as the length of the received data */
			if ( bytesToSave <= maxData ) 
			{
				/* for the check it is distinguished if the remaing bytes are less than 8 (in that
				   case 7 data bytes were sent and the SDO-Download Segement header contains the information
					how much bytes are valid (CAN-compatibilty)), otherwise the length has to match exactly
					and the SDO-Download Segment-Headerbyte is ignored */
				if ( ( ( bytesToSave <= pSdoInd->MbxHeader.Word[MBX_OFFS_LENGTH] - SEGMENT_NORM_HEADER_SIZE )
				     &&( bytesToSave == ((UINT16) (MIN_SEGMENTED_DATA - ((pSdoInd->SdoHeader.SegHeader & SEGHEADER_SEGDATASIZE) >> SEGHEADERSHIFT_SEGDATASIZE))) )
					  ) 
					||( ( bytesToSave > MIN_SEGMENTED_DATA )
					  &&( bytesToSave == (pSdoInd->MbxHeader.Word[MBX_OFFS_LENGTH] - SEGMENT_NORM_HEADER_SIZE) ) 
					) )  
				{
					/* length is correct */
					bSdoSegFollows = FALSE;
				}
				else
					abort = ABORTIDX_PARAM_LENGTH_ERROR;			
			}	
			else
				abort = ABORTIDX_PARAM_LENGTH_ERROR;	
		}
		else
		{
			/* its not the last segment */
			bSdoSegFollows = TRUE;
			/* we have to check if we expect less bytes than the maximum size which can be send with a single 
			   SDO Download Segment */
			if ( bytesToSave <= maxData )
				abort = ABORTIDX_PARAM_LENGTH_ERROR;
			else
				/* length is okay, bytesToSave contains the data size to be copied */
				bytesToSave = maxData; 
		}

		if ( abort == 0 )
		{
			/* the received data is copied in the buffer */
            tSdoTranstate.f.b.bData=1;
            tSdoTranstate.f.b.bLast=(bSdoSegFollows == FALSE);
            tSdoTranstate.ulDataSize=bytesToSave;
            tSdoTranstate.pubDataBuffer=pSdoInd->SdoHeader.Data;

            abortcode=CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);

            if(bSdoSegFollows == FALSE && abortcode == ABORT_NOERROR)
            {
                tSdoTranstate.f.b.bData=0;
                tSdoTranstate.f.b.bLast=0;
                tSdoTranstate.f.b.bEnd=1;
                tSdoTranstate.ulDataSize=0;
                CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);

    			tSdoSegState.segdispatchpending=tSdoSegState.dispatchinit=0;
            }
		}
	}

	if ( abort == 0 && abortcode==ABORT_NOERROR )
	{
		/* send the SDO Download Segment response */
		pSdoInd->MbxHeader.Word[MBX_OFFS_LENGTH] = SEGMENT_NORM_RES_SIZE;
		pSdoInd->CoeHeader.b[COEHEADER_COESERVICEOFFSET] = COESERVICE_SDORESPONSE << COEHEADER_COESERVICESHIFT;
		/* the SDO Download Segment header depends if it was the last segment or not */
		if ( bSdoSegLastToggle )
		{
   			pSdoInd->SdoHeader.SegHeader		= SDOSERVICE_DOWNLOADSEGMENTRES|SEGHEADER_TOGGLE;
		}
		else
   			pSdoInd->SdoHeader.SegHeader		= SDOSERVICE_DOWNLOADSEGMENTRES;

		if ( bSdoSegFollows == TRUE ) 
		{
			/* segments are still expected, nSdoSegBytesToHandle contains the number of received data bytes */
			nSdoSegBytesToHandle += bytesToSave;
		}
		else
		{
			/* the last segment was received, the variables are reset */
			nSdoSegBytesToHandle = 0;	
			nSdoSegService	= 0;
		}
	}
	else
	{	
/* ECATCHANGE_START(V4.00) */
		/* the Abort-Req will be sent in SDOS_SdoInd */
		bSdoSegFollows = FALSE;
		nSdoSegService	= 0;
/* ECATCHANGE_END(V4.00) */

		nSdoSegBytesToHandle = 0;
		
		if(abortcode==ABORT_NOERROR)
			abortcode=cAbortCode[abort];
	}

	return abortcode;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pSdoInd			Pointer to the received mailbox data from the master.

 \return	Indicates if an error occured while the operation ( good = 0 ).

 \brief	This function is called when a Upload SDO Segment request service
			is received from the master. It prepares and operates the 
			response and sends it by itself.
*////////////////////////////////////////////////////////////////////////////////////////

static UINT32 SdoUploadSegmentInd( TUPLOADSDOSEGREQMBX MBXMEM * pSdoInd )
{
	UINT32 abortcode = ABORT_NOERROR;

	if ( (pSdoInd->SegHeader & SEGHEADER_TOGGLE) == bSdoSegLastToggle )
	{
		/* toggle bit has not toggled... */
		abortcode = ABORT_TOGGLE_BIT_NOT_CHANGED;
	}
	else
	{
		/* maxData contains the maximum data to be sent with a SDO-Upload Segment response */
		UINT16 maxData =	u16SendMbxSize - SIZEOF(UMBXHEADER) - SEGMENT_NORM_HEADER_SIZE;
/* ECATCHANGE_START(V4.00) SDO 4 */
		UINT32 size = 0;
/* ECATCHANGE_END(V4.00) SDO 4 */

		/* the new toggle bit is stored in bSdoSegLastToggle */
		bSdoSegLastToggle = (UINT8)(pSdoInd->SegHeader & SEGHEADER_TOGGLE);

		if ( nSdoSegCompleteSize < (nSdoSegBytesToHandle + maxData) )
		{
			/* the remaing data can be send with one SDO Upload Segment response,
			   size contains the data to be copied */
			size = nSdoSegCompleteSize - nSdoSegBytesToHandle;
			bSdoSegFollows = FALSE;
		}
		else
		{ 
			/* more data will follow, size contains the data to be copied */
			size = maxData;
			bSdoSegFollows = TRUE;
		}

		/* copy the object data in the SDO Upload segment response */
        tSdoTranstate.f.b.bData=1;
        tSdoTranstate.ulDataSize=size;
        tSdoTranstate.pubDataBuffer=((TUPLOADSDOSEGRESMBX MBXMEM *) pSdoInd)->SdoHeader.Data;

        abortcode=CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);
		if(abortcode!=ABORT_NOERROR)
			return abortcode;

		/* the SDO Upload Segment header depends if there is still data to be sent */
		((TUPLOADSDOSEGRESMBX MBXMEM *) pSdoInd)->CoeHeader.b[COEHEADER_COESERVICEOFFSET] = COESERVICE_SDORESPONSE << COEHEADER_COESERVICESHIFT;
		if (bSdoSegFollows)
			((TUPLOADSDOSEGRESMBX MBXMEM *) pSdoInd)->SdoHeader.SegHeader		= (UINT8)(SDOSERVICE_UPLOADSEGMENTRES | bSdoSegLastToggle);
		else
			((TUPLOADSDOSEGRESMBX MBXMEM *) pSdoInd)->SdoHeader.SegHeader		= (UINT8)(SDOSERVICE_UPLOADSEGMENTRES | bSdoSegLastToggle | SEGHEADER_NOMOREFOLLOWS);

		// operate CAN specific flag segDataSize:
		/* HBu 06.02.06: the sizes were wrong */
		if ( size < MIN_SEGMENTED_DATA )
		{
			// at least	MIN_SEGMENTED_DATA bytes have to be send:
			((TUPLOADSDOSEGRESMBX MBXMEM *) pSdoInd)->MbxHeader.Word[MBX_OFFS_LENGTH] = SEGMENT_NORM_RES_SIZE;
			((TUPLOADSDOSEGRESMBX MBXMEM *) pSdoInd)->SdoHeader.SegHeader	|= (UINT8)((MIN_SEGMENTED_DATA - size) << SEGHEADERSHIFT_SEGDATASIZE);
		}
		else
		{
			((TUPLOADSDOSEGRESMBX MBXMEM *) pSdoInd)->MbxHeader.Word[MBX_OFFS_LENGTH] 		= ((UINT16) size) + SEGMENT_NORM_HEADER_SIZE;
		}						

		if ( bSdoSegFollows == TRUE )
			// updating the val of sended bytes:
			nSdoSegBytesToHandle += size;
		else
		{
            tSdoTranstate.f.b.bData=0;
            tSdoTranstate.f.b.bLast=0;
            tSdoTranstate.f.b.bEnd=1;
            tSdoTranstate.ulDataSize=0;
            CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);

			tSdoSegState.segdispatchpending=tSdoSegState.dispatchinit=0;

			nSdoSegBytesToHandle = 0;
			nSdoSegService	= 0;
		}
	}
  
	return abortcode;
}

#endif // #if SEGMENTED_SDO_SUPPORTED
#if ECAT_SDO_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pCoeMbx	Pointer to the received mailbox data from the master.

 \return	Indicates if an error occured while the operation ( good = 0 ).

 \brief	This function is called when a SDO request service
			is received from the master and calls depending from
			the command the concerning function.
*////////////////////////////////////////////////////////////////////////////////////////

UINT8 SDOS_SdoInd(TINITSDOMBX MBXMEM *pSdoInd)
{
	UINT8 abort = 0; 
	UINT32 abortcode = ABORT_NOERROR;
	UINT8 sdoHeader = (UINT8)(pSdoInd->SdoHeader.Sdo[SDOHEADER_COMMANDOFFSET] & SDOHEADER_COMMANDMASK);
	/* the SDO-command is in bit 5-7 of the first SDO-Byte */
	UINT8 command = (UINT8)(sdoHeader & SDOHEADER_COMMAND);
	/* mbxSize contains the size of the mailbox (CoE-Header (2 Bytes) + SDO-Header (8 Bytes) + SDO-Data (if the data length is greater than 4)) */
	UINT16 mbxSize = pSdoInd->MbxHeader.Word[MBX_OFFS_LENGTH];
	/* this variable contains the information, if all entries of an object will be read (bCompleteAccess > 0) or a single entry */
	UINT32 objLength = 0;
	UINT32 dataSize = 0;
	/* transferType contains the information if the SDO Download Request or the SDO Upload Response 
	   can be an expedited service (SDO data length <= 4, that means the data is stored in the
		SDO-Header completely */
	UINT8 transferType = 0;
	/* pData is the pointer to the received (SDO-Download) or sent (SDO-Upload) SDO data in the mailbox */
	UINT16 MBXMEM * pData = NULL;
#if SEGMENTED_SDO_SUPPORTED
	UINT8 segTransfer = 0;
#endif
	
	// if previous segmented transaction is resetted by the master, then inform dispatch, if any
	if(tSdoSegState.segdispatchpending)
    {
        tSdoTranstate.f.b.bData=0;
        tSdoTranstate.f.b.bLast=0;
        tSdoTranstate.f.b.bAbort=1;
        tSdoTranstate.ulDataSize=0;
        CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);
    }

        // reset trasaction state data
    memset(&tSdoTranstate, 0, sizeof(tSdoTranstate));
	
	tSdoSegState.dispatchinit=tSdoSegState.download=tSdoSegState.upload=tSdoSegState.segdispatchpending=0;

	switch (command)
	{
	case SDOSERVICE_INITIATEDOWNLOADREQ:		
	case SDOSERVICE_INITIATEUPLOADREQ:		
		/* the variable index contains the requested index of the SDO service */
		tSdoTranstate.uwIndex       = (UINT16)(pSdoInd->SdoHeader.Sdo[SDOHEADER_INDEXHIOFFSET] & SDOHEADER_INDEXHIMASK);
		tSdoTranstate.uwIndex       <<= 8;
		tSdoTranstate.uwIndex       += pSdoInd->SdoHeader.Sdo[SDOHEADER_INDEXLOOFFSET] >> SDOHEADER_INDEXLOSHIFT;
		/* the variable subindex contains the requested subindex of the SDO service */
		tSdoTranstate.ubSubIndex    = (UINT8)(pSdoInd->SdoHeader.Sdo[SDOHEADER_SUBINDEXOFFSET] >> SDOHEADER_SUBINDEXSHIFT);

		if ( command == SDOSERVICE_INITIATEUPLOADREQ )
		{
			/* SDO Upload */
			if ( mbxSize != EXPEDITED_FRAME_SIZE )
				/* a SDO Upload request has always a fixed size (2 Byte CoE-Header plus 8 Byte SDO-Header) */
				return MBXERR_INVALIDSIZE;

            tSdoTranstate.f.b.bUpload=1;
            tSdoTranstate.f.b.bInit=1;

                // initialize transaction
            abortcode=CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);
            if(abortcode!=ABORT_NOERROR)
                goto aborttran;

            objLength=dataSize=tSdoTranstate.ulDataSize;

            tSdoTranstate.f.b.bInit=0;

			tSdoSegState.dispatchinit=1;
			tSdoSegState.upload=1;

			/* distinguish between expedited and normal upload response within the length of the response data */
			if ( objLength <= MAX_EXPEDITED_DATA )
			{
				/* Expedited Upload */
				transferType = 1;
				/* pData is the pointer where the object data has to be copied for the response */
				pData = ((TINITSDOUPLOADEXPRESMBX MBXMEM *) pSdoInd)->Data;
				/* initialize the 4 data bytes of the SDO upload response because the requested object data
				   could be less than 4 */
				pData[0] = 0;
				pData[1] = 0;
			}
			else
			{
#if SEGMENTED_SDO_SUPPORTED
				/* HBu 06.02.06: the variable dataSize has to be set to the available size in one mailbox */
				dataSize = u16SendMbxSize - SIZEOF(UMBXHEADER) - UPLOAD_NORM_RES_SIZE;
				if ( dataSize < objLength )
					/* Segmented Upload */
					segTransfer = 1;
				else
					/* Normal Upload */
					pData = ((TINITSDOUPLOADNORMRESMBX MBXMEM *) pSdoInd)->Data;
#else
				/* Normal Upload */
				/* pData is the pointer where the object data has to be copied for the response */
				pData = ((TINITSDOUPLOADNORMRESMBX MBXMEM *) pSdoInd)->Data;
#endif
			}
		}

		if ( command == SDOSERVICE_INITIATEDOWNLOADREQ )
		{
			/* SDO-Download: store if the request is a expedited or normal request  */
			transferType = (UINT8)(sdoHeader & SDOHEADER_TRANSFERTYPE);

			/* SDO Download */
			if ( transferType )
			{
				/* Expedited Download */
				if ( mbxSize != EXPEDITED_FRAME_SIZE )
					/* an Expedited SDO Download request has always a fixed size (2 Byte CoE-Header plus 8 Byte SDO-Header) */
					return MBXERR_INVALIDSIZE;
				/* dataSize gets the real size of the downloaded object data (1,2,3 or 4) */
				dataSize = MAX_EXPEDITED_DATA - ((sdoHeader & SDOHEADER_DATASETSIZE) >> SDOHEADERSHIFT_DATASETSIZE);
				/* pData is the pointer to the downloaded object data */
				pData = (UINT16 MBXMEM *) &pSdoInd[1];
			}
			else
			{
				/* Normal Download */
				/* downloadSize gets the real size of the downloaded data */
/* ECATCHANGE_START(V4.00) SDO 4 */
				UINT32 downloadSize = SWAPDWORD(((UINT32 MBXMEM *)&((TINITSDODOWNLOADNORMREQMBX MBXMEM *) pSdoInd)->CompleteSize)[0]);
/* ECATCHANGE_END(V4.00) SDO 4 */
				/* HBu 29.03.06: if it is a segmented download the mbxSize has to be the complete mailbox size */
				if ( (SIZEOF(UMBXHEADER)+EXPEDITED_FRAME_SIZE+downloadSize) > u16ReceiveMbxSize )
				{
					if ( mbxSize != (u16ReceiveMbxSize-SIZEOF(UMBXHEADER)) )
						return MBXERR_INVALIDSIZE;
				}
				else
				{
					if ( mbxSize != (EXPEDITED_FRAME_SIZE+downloadSize) )
						/* the mbxSize and the downloadSize are not consistent (mbxSize = downloadSize + 2 byte CoE-Header + 8 byte SDO Header */
						return MBXERR_INVALIDSIZE;
				}

				/* pData is the pointer to the downloaded object data */
				pData = (UINT16 MBXMEM *) ((TINITSDODOWNLOADNORMREQMBX MBXMEM *) pSdoInd)->Data;
/* ECATCHANGE_START(V4.00) SDO 1 */
				/* the received dataSize will be checked in the object specific functions called from 
				   OBJ_Write (in objdef.c) */
				dataSize = downloadSize;
				if ( dataSize > (mbxSize-DOWNLOAD_NORM_REQ_SIZE) )
#if SEGMENTED_SDO_SUPPORTED
					/* Segmented Download */
					segTransfer = 1;
#else /* SEGMENTED_SDO_SUPPORTED */
					abort = ABORTIDX_PARAM_LENGTH_ERROR;
#endif /* SEGMENTED_SDO_SUPPORTED */
/* ECATCHANGE_END(V4.00) SDO 1 */
			}

            tSdoTranstate.f.b.bDownload=1;
            tSdoTranstate.f.b.bInit=1;
            tSdoTranstate.ulDataSize=dataSize;

                // initialize transaction
            abortcode=CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);
            if(abortcode!=ABORT_NOERROR)
                goto aborttran;

            tSdoTranstate.f.b.bInit=0;

			tSdoSegState.dispatchinit=1;
			tSdoSegState.download=1;

			objLength=dataSize;
		}

		if ( sdoHeader & SDOHEADER_COMPLETEACCESS )
#if COMPLETE_ACCESS_SUPPORTED
		{
			bCompleteAccess = 1;
			// HBu 02.05.06: Complete Access is only supported with subindex 0 and 1
			if (subindex > 1)
				abort = ABORTIDX_UNSUPPORTED_ACCESS;
		}
#else
			abort = ABORTIDX_UNSUPPORTED_ACCESS;
#endif
		
		if ( abort == 0 )
		{
#if SEGMENTED_SDO_SUPPORTED
			if ( segTransfer )
			{
				bSdoSegFollows 		= TRUE;
				bSdoSegLastToggle 	= 1;
#if COMPLETE_ACCESS_SUPPORTED
				bSdoSegAccess 		= bCompleteAccess;
#endif
				if ( command == SDOSERVICE_INITIATEUPLOADREQ )
				{
					nSdoSegCompleteSize	= objLength;

                        // get data chunk
                    tSdoTranstate.f.b.bData=1;
                    tSdoTranstate.ulDataSize=dataSize;
                    tSdoTranstate.pubDataBuffer=(UINT8 *)((TINITSDOUPLOADNORMRESMBX MBXMEM *) pSdoInd)->Data;

                    abortcode=CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);
                    if(abortcode!=ABORT_NOERROR)
                        goto aborttran;

					nSdoSegService	= SDOSERVICE_UPLOADSEGMENTREQ;
				}
				else
				{
					nSdoSegCompleteSize	= dataSize;

                        // send data chunk
                    tSdoTranstate.f.b.bData=1;
                    tSdoTranstate.f.b.bLast=0;
                    tSdoTranstate.ulDataSize=mbxSize-DOWNLOAD_NORM_REQ_SIZE;
                    tSdoTranstate.pubDataBuffer=(UINT8 *)((TINITSDOUPLOADNORMRESMBX MBXMEM *) pSdoInd)->Data;

                    abortcode=CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);
                    if(abortcode!=ABORT_NOERROR)
                        goto aborttran;

					nSdoSegService	= SDOSERVICE_DOWNLOADSEGMENTREQ;
					dataSize = (mbxSize-DOWNLOAD_NORM_REQ_SIZE);
				}

				nSdoSegBytesToHandle = dataSize;
			}
			else
#endif // SEGMENTED_SDO_SUPPORTED
			{ 
				if ( command == SDOSERVICE_INITIATEUPLOADREQ )
				{
                        // get data chunk
                    tSdoTranstate.f.b.bData=1;
                    tSdoTranstate.ulDataSize=objLength;
                    tSdoTranstate.pubDataBuffer=(UINT8 *)pData;

                    abortcode=CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);
                    if(abortcode!=ABORT_NOERROR)
                        goto aborttran;

					tSdoSegState.segdispatchpending=tSdoSegState.dispatchinit=0;
				}
				else
				{
                        // send data chunk
                    tSdoTranstate.f.b.bData=1;
                    tSdoTranstate.f.b.bLast=1;
                    tSdoTranstate.ulDataSize=dataSize;
                    tSdoTranstate.pubDataBuffer=(UINT8 *)pData;

                    abortcode=CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);
                    if(abortcode!=ABORT_NOERROR)
                        goto aborttran;

					tSdoSegState.segdispatchpending=tSdoSegState.dispatchinit=0;
				}

                tSdoTranstate.f.b.bData=0;
                tSdoTranstate.f.b.bLast=0;
                tSdoTranstate.f.b.bEnd=1;
                tSdoTranstate.ulDataSize=0;
                CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);
			}	/* if ( segTransfer ) is closed */
		
			if ( abort == 0 && abortcode == 0l )
			{
				/* SDO-Download or SDO-Upload was successful, generate the SDO- and CoE-Header */
				pSdoInd->CoeHeader.b[COEHEADER_COESERVICEOFFSET] = COESERVICE_SDORESPONSE << COEHEADER_COESERVICESHIFT;
				if ( command == SDOSERVICE_INITIATEUPLOADREQ )
				{
					// HBu 06.02.06: Complete Access Bit in the SDO-Upload-Resonse too */
					sdoHeader &= SDOHEADER_COMPLETEACCESS;
					if ( transferType )
					{
						/* Expedited Upload Response */
						pSdoInd->MbxHeader.Word[MBX_OFFS_LENGTH] 		= 	EXPEDITED_FRAME_SIZE;
						pSdoInd->SdoHeader.Sdo[SDOHEADER_COMMANDOFFSET]	= 	(UINT8)(SDOHEADER_SIZEINDICATOR 	|
																							SDOHEADER_TRANSFERTYPE		|
																							sdoHeader |
																							((MAX_EXPEDITED_DATA - objLength) << SDOHEADERSHIFT_DATASETSIZE) |
																							SDOSERVICE_INITIATEUPLOADRES);
					}
					else
					{
						/* Normal or Segmented Upload Response */
#if SEGMENTED_SDO_SUPPORTED
						if ( segTransfer )
							pSdoInd->MbxHeader.Word[MBX_OFFS_LENGTH] 		= 	(UINT16)(UPLOAD_NORM_RES_SIZE+dataSize);
						else
#endif
							pSdoInd->MbxHeader.Word[MBX_OFFS_LENGTH] 		= 	(UINT16)(SWAPWORD(UPLOAD_NORM_RES_SIZE+objLength));
						((TINITSDOUPLOADNORMRESMBX MBXMEM *) pSdoInd)->CompleteSize[0]	= (UINT16)objLength;
						((TINITSDOUPLOADNORMRESMBX MBXMEM *) pSdoInd)->CompleteSize[1]	= 0;
						pSdoInd->SdoHeader.Sdo[SDOHEADER_COMMANDOFFSET]	= 	(UINT8)(SDOHEADER_SIZEINDICATOR 	|
																							sdoHeader |
																							SDOSERVICE_INITIATEUPLOADRES);
					}
				}
				else
				{
					/* Download response */
					pSdoInd->MbxHeader.Word[MBX_OFFS_LENGTH] 		= DOWNLOAD_NORM_RES_SIZE;
					pSdoInd->SdoHeader.Sdo[SDOHEADER_COMMANDOFFSET]	= SDOSERVICE_INITIATEDOWNLOADRES;
				}
#if BYTE_NOT_SUPPORTED
				pSdoInd->SdoHeader.Sdo[SDOHEADER_INDEXLOOFFSET]	|= (index & 0xFF) << SDOHEADER_INDEXLOSHIFT;
#endif
			}
		}
		break;

#if SEGMENTED_SDO_SUPPORTED
	case SDOSERVICE_DOWNLOADSEGMENTREQ:		
	case SDOSERVICE_UPLOADSEGMENTREQ:		
		if ( command == nSdoSegService )
		{
			if ( command == SDOSERVICE_DOWNLOADSEGMENTREQ )
				abortcode = SdoDownloadSegmentInd( (TDOWNLOADSDOSEGREQMBX MBXMEM *) pSdoInd );
			else
				abortcode = SdoUploadSegmentInd( (TUPLOADSDOSEGREQMBX MBXMEM *) pSdoInd );
		}
		else
			abort = ABORTIDX_COMMAND_SPECIFIER_UNKNOWN;
		break;

#endif
	default:
		abort = ABORTIDX_COMMAND_SPECIFIER_UNKNOWN;
		break;
	}

aborttran:
	if ( abort || abortcode )
	{
		// call dispatch if init'ed
		if(tSdoSegState.dispatchinit)
        {
            tSdoTranstate.f.b.bData=0;
            tSdoTranstate.f.b.bLast=0;
            tSdoTranstate.f.b.bAbort=1;
            tSdoTranstate.ulDataSize=0;
            CanOpenComDBSdoDispatcher(&tSdoTranstate,CANOPENCOMDB_F_ECATCOE_VALID);
        }

		// if abort idx is submitted
		if ( abort && abortcode==0l )
			abortcode=cAbortCode[abort];

		/* generate a SDO-Abort-Request */
		pSdoInd->MbxHeader.Word[MBX_OFFS_LENGTH] 		= ABORT_NORM_RES_SIZE;
		pSdoInd->CoeHeader.b[COEHEADER_COESERVICEOFFSET] = COESERVICE_SDOREQUEST << COEHEADER_COESERVICESHIFT;
#if BYTE_NOT_SUPPORTED
		pSdoInd->SdoHeader.Sdo[SDOHEADER_COMMANDOFFSET] &= ~SDOHEADER_COMMANDMASK;
		pSdoInd->SdoHeader.Sdo[SDOHEADER_COMMANDOFFSET]	|= SDOSERVICE_ABORTTRANSFER;
#else
		pSdoInd->SdoHeader.Sdo[SDOHEADER_COMMANDOFFSET] = SDOSERVICE_ABORTTRANSFER;	
#endif
#if MOTOROLA_16BIT
		((UINT16 MBXMEM *) ((TABORTSDOTRANSFERREQMBX MBXMEM *) pSdoInd)->AbortCode)[0] = ((const UINT16 *) &abortcode)[1];
		((UINT16 MBXMEM *) ((TABORTSDOTRANSFERREQMBX MBXMEM *) pSdoInd)->AbortCode)[1] = ((const UINT16 *) &abortcode)[0];
#else
		((TABORTSDOTRANSFERREQMBX MBXMEM *) pSdoInd)->AbortCode = SWAPDWORD(abortcode);
#endif
	}

	// HBu 02.05.06: if the CoE-response could not be sent because the 
	//               send mailbox is full it should be stored
	if (MBX_MailboxSendReq((TMBX MBXMEM *) pSdoInd, COE_SERVICE) != 0)
	{
		/* we store the CoE mailbox service to send it later (in COE_ContinueInd) when the mailbox is read */
		pCoeSendStored = (TMBX MBXMEM *) pSdoInd;
	}
	
	return 0;
}

#endif // ECAT_SDO_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pCoeMbx			Pointer to the received mailbox data from the master.
 \param	bNextFragment	Is TRUE if a fragment of this service was send before.

 \return	Indicates if an error occured while the operation ( good = 0 ).

 \brief	This function is called when a SDO-Info request service
			is received from the master and calls depending from
			the opcode the concerning function.
*////////////////////////////////////////////////////////////////////////////////////////

UINT8 SDOS_SdoInfoInd( TSDOINFORMATION MBXMEM *pSdoInfoInd )
{																				 
	UINT8 abort = 0;
	/* the variable opCode contains the requested SDO Information type */
	UINT8 opCode = (UINT8)(pSdoInfoInd->SdoHeader.InfoHead.b[INFOHEAD_OFFS_OPCODE] & INFOHEADER_OPCODEMASK);
	UINT16 index;
	UINT8 flags = COE_SERVICE;
    TSDOINFOOBJDESC tInfoObj;
    TSDOINFOENTRYDESC tInfoEntry;

	/* it has to be checked if the mailbox protocol is correct, the sent mailbox data length has to 
	   great enough for the service haeder of the requested SDO Information type */
	if ( opCode == SDOINFOSERVICE_ENTRYDESCRIPTION_Q )
	{
		if ( pSdoInfoInd->MbxHeader.Word[MBX_OFFS_LENGTH] < SIZEOF_SDOINFOENTRYSTRUCT )
			return MBXERR_SIZETOOSHORT;
	}
	else
	{
		if ( pSdoInfoInd->MbxHeader.Word[MBX_OFFS_LENGTH] < SIZEOF_SDOINFOLISTSTRUCT )
			return MBXERR_SIZETOOSHORT;
	}

	switch ( opCode )
	{
	case SDOINFOSERVICE_OBJDICTIONARYLIST_Q:
		/* an object list is requested, check if the list type is supported */
		if ( SWAPWORD(pSdoInfoInd->SdoHeader.Data.List.ListType) <= INFO_LIST_TYPE_MAX )
		{
			UINT16 size = 0;
			/* the variable listType contains the requested listType */
			UINT8 listType = (UINT8) SWAPWORD(pSdoInfoInd->SdoHeader.Data.List.ListType);

			/* the SDO Information Header has to be stored because this function will be
			   called again if the response could not be sent with one mailbox service, the
			   variable nSdoInfoFragmentsLeft is 0 zero for the first call and unequal 0
			   for the following calls */
			MBXMEMCPY(aSdoInfoHeader, pSdoInfoInd, SIZEOF(aSdoInfoHeader));
			if ( listType-- == 0 )
			{
				/* List-Type 0: length of the lists */
				UINT8 i;

				/* the needed mailbox size for List-Type 0 response is just 24 bytes, the mailbox has always
				   to be at least 24 bytes to support the SDO Information service */
				nSdoInfoFragmentsLeft = 0;
				for (i = 0; i < INFO_LIST_TYPE_MAX; i++)
				{
					UINT16 n = CanOpenComDBEnum( i,CANOPENCOMDB_F_ECATCOE_VALID );

					/* copy the number of objects of the lis type in the SDO Information response */
					((UINT16 MBXMEM *) &pSdoInfoInd->CoeHeader)[(SIZEOF_SDOINFOLISTSTRUCT>>1)+i] = SWAPWORD(n);
				}
				
				/* size of the mailbox service response */
				size = (INFO_LIST_TYPE_MAX << 1) + SIZEOF_SDOINFOLISTSTRUCT;
			}
			else
			{
				/* object list with indexes is requested */
				UINT16 MBXMEM * pData;
				UINT16 n = 0;

				if ( nSdoInfoFragmentsLeft )
				{
					/* the next fragment of the SDO Information response shall be sent */
					/* initialize size with the maximum size to be sendable with one mailbox service */
					size = u16SendMbxSize - SIZEOF_SDOINFO - sizeof(UMBXHEADER);
					/* initialize pData with the pointer where the fragment has to be copied */
					pData = &((UINT16 MBXMEM *) &pSdoInfoInd->CoeHeader)[SIZEOF_SDOINFO>>1];
					/* decrement the number of fragments to be sent */
					nSdoInfoFragmentsLeft--;
				}
				else
				{
					/* the first fragment of the SDO Information response has to be sent */
					/* get the number of objects of the requested object list */
					n = (UINT16)CanOpenComDBEnum( listType,CANOPENCOMDB_F_ECATCOE_VALID );
					/* initialize objdictlist context */
					CanOpenComDBList( listType, CANOPENCOMDB_F_ECATCOE_VALID, &tSdoInfoContext, NULL, 0);
					/* initialize size with the maximum size to be sendable with one mailbox service */
					size = u16SendMbxSize - SIZEOF_SDOINFOLISTSTRUCT - sizeof(UMBXHEADER);
					/* initialize pData with the pointer where the fragment has to be copied */
					pData = &((UINT16 MBXMEM *) &pSdoInfoInd->CoeHeader)[SIZEOF_SDOINFOLISTSTRUCT>>1];
					/* check how many fragments has to be sent */
					if (n)
						nSdoInfoFragmentsLeft = (UINT8)(((n<<1)-1)/size);
					else
						nSdoInfoFragmentsLeft = 0;
				}

				/* get the next part of the requested object list */
				size = CanOpenComDBList( listType, CANOPENCOMDB_F_ECATCOE_VALID, &tSdoInfoContext, pData, size);
				/* size contains before the instruction the size still available in the mailbox buffer 
					and shall contain the size of the mailbox response data after the next instruction */
				size = u16SendMbxSize - size - sizeof(UMBXHEADER);
			}

			/* size of the mailbox response data */
			pSdoInfoInd->MbxHeader.Word[MBX_OFFS_LENGTH]	= size;
			/* number of fragments still has to be sent */
			pSdoInfoInd->SdoHeader.FragmentsLeft 			= SWAPWORD(nSdoInfoFragmentsLeft);
			if (nSdoInfoFragmentsLeft)
			{
				/* there still are fragments to be sent, 
				   the InComplete flag in the SDO Information response has to be sent */
				((UINT8 MBXMEM *) &pSdoInfoInd->SdoHeader)[INFOHEAD_OFFS_OPCODE] = SDOINFOSERVICE_OBJDICTIONARYLIST_S | SDOINFOSERVICE_INCOMPLETE;
				/* the FRAGMENTS_FOLLOW flag has to be set for the function MBX_MailboxSendReq to 
				   indicate the mailbox handler that still fragments has to be sent so that this
					function shall be called again from COE_ContinueInd when the actual mailbox buffer
					was sent */
				flags = FRAGMENTS_FOLLOW | COE_SERVICE;
			}
			else
				((UINT8 MBXMEM *) &pSdoInfoInd->SdoHeader)[INFOHEAD_OFFS_OPCODE] = SDOINFOSERVICE_OBJDICTIONARYLIST_S;
		}
		break;

	case SDOINFOSERVICE_OBJDESCRIPTION_Q:
	case SDOINFOSERVICE_ENTRYDESCRIPTION_Q:
		/* get the requested index */
		index = SWAPWORD(pSdoInfoInd->SdoHeader.Data.Obj.Index);
		/* get the object handle of the requested index */
		if ( CanOpenComDBGetInfo(index, 0, CANOPENCOMDB_F_ECATCOE_VALID, &tInfoObj, &tInfoEntry) )
		{
			/* object exists */
			UINT16 size = 0;
			if ( opCode == SDOINFOSERVICE_OBJDESCRIPTION_Q )
			{
				/* object description is requested */
                pSdoInfoInd->SdoHeader.Data.Obj.Res=tInfoObj;

				/* the mailbox should be big enough that the maximum object description
				   fits in the mailbox (the fragmentation is not done in the sample code),
					so it will be checked only if the object description fits */
				size = CanOpenComDBGetDescription(index, 0xFF, (PVISIBLE_STRING)((UINT16 MBXMEM *) &(&pSdoInfoInd->SdoHeader.Data.Obj.Res)[1]), \
						u16SendMbxSize - SIZEOF_SDOINFOOBJSTRUCT - sizeof(UMBXHEADER));
				size += SIZEOF_SDOINFOOBJSTRUCT;
			}
			else
			{
				/* entry description is requested,
				   get the requested subindex */
#if BYTE_NOT_SUPPORTED
				UINT8 subindex = pSdoInfoInd->SdoHeader.Data.Entry.Info.b[ENTRY_OFFS_SUBINDEX] & ENTRY_MASK_SUBINDEX;
#else
				UINT8 subindex = pSdoInfoInd->SdoHeader.Data.Entry.Info.b[ENTRY_OFFS_SUBINDEX];
#endif
				if ( CanOpenComDBGetInfo(index, subindex, CANOPENCOMDB_F_ECATCOE_VALID, &tInfoObj, &tInfoEntry) )
				{
					/* requested subindex is not too great */
					/* get the entry description of the requested entry */
                    pSdoInfoInd->SdoHeader.Data.Entry.Res=tInfoEntry;

					/* the transmission of the value info is not supported yet of the sample code */
#if BYTE_NOT_SUPPORTED
					pSdoInfoInd->SdoHeader.Data.Entry.Info.b[ENTRY_OFFS_VALUEINFO] &= ENTRY_MASK_VALUEINFO;	
#else
					pSdoInfoInd->SdoHeader.Data.Entry.Info.b[ENTRY_OFFS_VALUEINFO] = 0;	
#endif
					/* the mailbox should be big enough that the maximum entry description
					   fits in the mailbox (the fragmentation is not done in the sample code),
						so it will be checked only if the entry description fits */
					size = CanOpenComDBGetDescription(index, subindex, (PVISIBLE_STRING)((UINT16 MBXMEM *) &(&pSdoInfoInd->SdoHeader.Data.Entry.Res)[1]), \
							u16SendMbxSize - SIZEOF_SDOINFO - SIZEOF(TSDOINFOENTRY) - sizeof(UMBXHEADER));
					size += SIZEOF_SDOINFO + SIZEOF(TSDOINFOENTRY);
				}
				else
					abort = ABORTIDX_SUBINDEX_NOT_EXISTING;
			}

			if ( abort == 0 )
			{
				/* for the object and entry description the sample code does not support the fragmentation, 
				   the mailbox has to be big enough */
				pSdoInfoInd->SdoHeader.FragmentsLeft = 0;
				/* store the size of the mailbox data in the mailbox buffer */
				pSdoInfoInd->MbxHeader.Word[MBX_OFFS_LENGTH] = size;
				/* set the opCode of the SDO Information response */
				((UINT8 MBXMEM *) &pSdoInfoInd->SdoHeader)[INFOHEAD_OFFS_OPCODE] = (UINT8)(opCode+1);
			}
		}
		else
			abort = ABORTIDX_OBJECT_NOT_EXISTING;
		break;
	default:
		abort = ABORTIDX_COMMAND_SPECIFIER_UNKNOWN;
	}

	if ( abort )
	{
		/* send a SDO Information Error response */
		pSdoInfoInd->MbxHeader.Word[MBX_OFFS_LENGTH] = SIZEOF_SDOINFOERRORSTRUCT;
		((UINT8 MBXMEM *) &pSdoInfoInd->SdoHeader)[INFOHEAD_OFFS_OPCODE] = SDOINFOSERVICE_ERROR_Q;
		pSdoInfoInd->SdoHeader.FragmentsLeft = 0;
#if MOTOROLA_16BIT
		((UINT16 MBXMEM *) &pSdoInfoInd->SdoHeader.Data.Error.ErrorCode)[0] = ((const UINT16 *) &cAbortCode[abort])[1];
		((UINT16 MBXMEM *) &pSdoInfoInd->SdoHeader.Data.Error.ErrorCode)[1] = ((const UINT16 *) &cAbortCode[abort])[0];
#else
		pSdoInfoInd->SdoHeader.Data.Error.ErrorCode = SWAPDWORD(cAbortCode[abort]);
#endif
		nSdoInfoFragmentsLeft = 0;
	}

	if (MBX_MailboxSendReq((TMBX MBXMEM *) pSdoInfoInd, flags) != 0)
	{
		/* if the mailbox response could not be sent (or stored), the response will be
		   stored in the variable pCoeSendStored and will be sent automatically
			from the mailbox handler (COE_ContinueInd) when the send mailbox will be read
			the next time from the master */
		pCoeSendStored = (TMBX MBXMEM *) pSdoInfoInd;
	}

	return 0;
}

/** @} */
#endif /* COE_SUPPORTED */
