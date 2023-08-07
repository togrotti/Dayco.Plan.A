/** 
\defgroup ecatfoe ecatfoe.c: FoE (File Access over EtherCAT) functions
\brief This file contains the FoE mailbox interface\n
\brief Changes to version V3.20:\n
\brief V4.00 FOE 1 - if a FoE Error service is recieved it should not be acknowledged\n 
\brief V4.00 FOE 2 - if a FoE service response could not be sent because the mailbox is full
\brief               it has to be re-transmit otherwise the response get lost

\author  Holger Buettner
\version 4.00
\date    23.03.2007
*/

//---------------------------------------------------------------------------------------
/**
\ingroup ecatfoe	
\file ecatfoe.c
\brief Implementation.
*/
//---------------------------------------------------------------------------------------

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/

#include "common\CommonDefines.h"

#include "ecat_def.h"

#if FOE_SUPPORTED

#define	_ECATFOE_ 1
#include "ecatfoe.h"
#undef	_ECATFOE_
#define	_ECATFOE_ 0

#if BOOTSTRAPMODE_SUPPORTED
#include "bootmode.h"
#endif
#include "ecatslv.h"
#include "mailbox.h"

/*-----------------------------------------------------------------------------------------
------  
------	AxX specific
------	
-----------------------------------------------------------------------------------------*/

// Avoids warning C37: nonstandard extension - unnamed parameter

//#pragma warning disable = 37

#include "ECATFileManager.h"

#define FOE_Read            ECATFM_Read
#define FOE_Write           ECATFM_Write
#define FOE_Data            ECATFM_Data
#define FOE_Ack             ECATFM_Ack
#define FOE_Busy            ECATFM_Busy
#define FOE_Error           ECATFM_Error

/*-----------------------------------------------------------------------------------------
------  
------	functions
------	
-----------------------------------------------------------------------------------------*/

/**
\addtogroup ecatfoe
@{
*/

#endif
#if FOE_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief	This function intializes the FoE Interface.
*////////////////////////////////////////////////////////////////////////////////////////

void FOE_Init(void)
{
	/* no file transmission sequence is running */
	u16FileAccessState	= FOE_READY;
	/* initialize the expected packet number */
	u16PacketNo = 0;
/* ECATCHANGE_START(V4.00) FOE 2 */
	pFoeSendStored = NULL;
/* ECATCHANGE_END(V4.00) FOE 2 */
}


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pFoeInd	  Pointer to the received mailbox data from the master.

 \return	result of the operation (0 (success) or mailbox error code (MBXERR_.... defined in
			mailbox.h))

 \brief	This function is called when a FoE (File Access over EtherCAT) service is received from 
 			the master.
*////////////////////////////////////////////////////////////////////////////////////////

UINT8 FOE_ServiceInd(TFOEMBX MBXMEM * pFoeInd)
{
	/* initialize the result of the service checking */
	UINT16 nextState = ECAT_FOE_ERRCODE_ILLEAGAL;
	/* dataSize contains the size of the file data */
	UINT16 dataSize = pFoeInd->MbxHeader.Word[MBX_OFFS_LENGTH] - sizeof(TFOEHEADER);

	/* it has to be checked if the mailbox protocol is correct, the sent mailbox data length has to 
	   great enough for the service haeder of the FoE service */
	if ( pFoeInd->MbxHeader.Word[MBX_OFFS_LENGTH] < SIZEOF(TFOEHEADER) )
		return MBXERR_SIZETOOSHORT;

	switch ( SWAPWORD(pFoeInd->FoeHeader.OpMode) )
	{
	case ECAT_FOE_OPMODE_RRQ:
		/* file read is requested */
		if ( u16FileAccessState == FOE_READY )
		{
			/* last FoE sequence was finished, call application function */
			nextState = FOE_Read(pFoeInd->Data, dataSize, pFoeInd->Data, SWAPDWORD(pFoeInd->FoeHeader.Cmd.Password));
			/* the first data packet shall be sent */
			u16PacketNo	= 1;
			/* u32LastFileOffset contains the offset of the file which is sent now */
			u32LastFileOffset = 0;
			/* u32FileOffset contains the offset of the file which shall be sent when the next FoE ACK is received */
			u32FileOffset = nextState;
		}			
		break;

	case ECAT_FOE_OPMODE_WRQ:
		/* file write is requested */
		if ( u16FileAccessState == FOE_READY )
		{
			/* last FoE sequence was finished, call application function */
			nextState = FOE_Write(pFoeInd->Data, dataSize, SWAPDWORD(pFoeInd->FoeHeader.Cmd.Password));
			if ( nextState == 0 )
			{
				/* checking was successful, sent a FoE Ack service */
				nextState = FOE_ACK;
			}
			/* initialize the packet number */
			u16PacketNo	= 0;
		}
		break;

	case ECAT_FOE_OPMODE_DATA:
		/* file data is recievd */
		if ( u16FileAccessState == FOE_WAIT_FOR_DATA 
		  || u16FileAccessState == FOE_WAIT_FOR_LAST_DATA )
		{
			/* we are waiting for file data, service is correct */
			if ( SWAPDWORD(pFoeInd->FoeHeader.Cmd.PacketNo) == u16PacketNo )
			{
				/* the packet number is correct, call application function to store the file data */
				nextState = FOE_Data(pFoeInd->Data, dataSize);
				if ( nextState == 0 )
				{
					/* checking was successful, sent a FoE Ack service */
					nextState = FOE_ACK;
				}
			}
			else
				nextState = ECAT_FOE_ERRCODE_PACKENO;
		}
		break;

	case ECAT_FOE_OPMODE_ACK:
		/* acknowledge is recievd, next file part can be sent */
		if ( u16FileAccessState == FOE_WAIT_FOR_ACK )
		{
			/* we are waiting for an acknowledge, service is correct, call the application function
			   to get the next part of the file */
			nextState = FOE_Ack( u32FileOffset, pFoeInd->Data );
			/* u32LastFileOffset contains the offset of the file which is sent now */
			u32LastFileOffset = u32FileOffset;
			/* u32FileOffset contains the offset of the file which shall be sent when the next FoE ACK is received */
			u32FileOffset += nextState;
			/* increment the packet number */
			u16PacketNo++;
		}
		else if ( u16FileAccessState == FOE_WAIT_FOR_LAST_ACK )
		{
			/* we were waiting for the last acknowledge, now the sequence is finished */
			nextState = FOE_FINISHED_NOACK;
		}
		break;

	case ECAT_FOE_OPMODE_ERR:
		/* a FoE Error service is recievd */
		if ( u16FileAccessState != FOE_READY )
		{
			/* a file transmission sequence is active, inform the application, that this sequence
			   was stopped */
			FOE_Error( SWAPDWORD(pFoeInd->FoeHeader.Cmd.ErrorCode) );		
			nextState = FOE_FINISHED;
		}
		break;

	case ECAT_FOE_OPMODE_BUSY:
		/* a FoE Busy service is recievd */
		if ( u16FileAccessState == FOE_WAIT_FOR_ACK 
		  || u16FileAccessState == FOE_WAIT_FOR_LAST_ACK )
		{
			/* we are waiting for an acknowledge, service is correct, call the application function
			   to resend the last part of the file */
			nextState = FOE_Busy( SWAPWORD(pFoeInd->FoeHeader.Cmd.Busy.Done), u32LastFileOffset, pFoeInd->Data );
		}
		break;

	}

	if ( nextState <= FOE_MAXDATA )
	{
		/* we send DATA and wait for ACK */
		UINT32 d = u16PacketNo;

		/* store the OpMode in the mailbox buffer */
		pFoeInd->FoeHeader.OpMode						= SWAPWORD(ECAT_FOE_OPMODE_DATA);
		/* store the packet number in the mailbox buffer */
		pFoeInd->FoeHeader.Cmd.PacketNo				= SWAPDWORD(d);
		/* store the size of the mailbox data in the mailbox buffer */
		pFoeInd->MbxHeader.Word[MBX_OFFS_LENGTH]	= sizeof(TFOEHEADER) + nextState;

		if ( nextState == u16SendMbxSize-sizeof(TFOEHEADER)-sizeof(UMBXHEADER) )
		{
			/* packets still following, we wait for an ACK */
			u16FileAccessState = FOE_WAIT_FOR_ACK;
		}
		else
		{
			/* it was the last Packet, we wait for the last ACK */
			u16FileAccessState = FOE_WAIT_FOR_LAST_ACK;
		}
	}
	else if ( nextState < FOE_MAXBUSY )
	{
		/* we are still storing the recieved file data (in flash for example) and 
		   send BUSY and wait for the DATA to be sent again */
		/* store the OpMode in the mailbox buffer */
		pFoeInd->FoeHeader.OpMode						= SWAPWORD(ECAT_FOE_OPMODE_BUSY);
		/* store the information how much progress we made until we can receive file data again */
		pFoeInd->FoeHeader.Cmd.Busy.Done				= SWAPWORD(nextState-FOE_MAXDATA);
		pFoeInd->FoeHeader.Cmd.Busy.Entire			= 0;
		/* store the size of the mailbox data in the mailbox buffer */
		pFoeInd->MbxHeader.Word[MBX_OFFS_LENGTH]	= sizeof(TFOEHEADER);
	}
	else if ( nextState == FOE_ACK || nextState == FOE_ACK_LAST )
	{
		/* we send ACK and wait for DATA 
			the next file data is expectzed with an incremented packet number, but
			we have to acknowledge the old packet first */
		UINT32 d = u16PacketNo++;
		/* store the OpMode in the mailbox buffer */
		pFoeInd->FoeHeader.OpMode						= SWAPWORD(ECAT_FOE_OPMODE_ACK);
		/* store the packet number in the mailbox buffer */
		pFoeInd->FoeHeader.Cmd.PacketNo				= SWAPDWORD(d);
		/* store the size of the mailbox data in the mailbox buffer */
		pFoeInd->MbxHeader.Word[MBX_OFFS_LENGTH]	= sizeof(TFOEHEADER);

		/* we wait for the next data part */
        if ( nextState == FOE_ACK )
		    u16FileAccessState = FOE_WAIT_FOR_DATA;
		/* last packet */
        else
		    u16FileAccessState = FOE_READY;
	}
	else if ( nextState < FOE_ERROR )
	{
/* ECATCHANGE_START(V4.00) FOE 1 */
		/* the file transmission sequence is finished, we have to send nothing */
		u16FileAccessState = FOE_READY;

		return 0;
/* ECATCHANGE_END(V4.00) FOE 1 */
	}
	else
	{
		/* we send an ERROR service */
		UINT32 d = nextState & 0x7FFF;
		/* store the OpMode in the mailbox buffer */
		pFoeInd->FoeHeader.OpMode					= SWAPWORD(ECAT_FOE_OPMODE_ERR);
		/* store the ErrorCode in the mailbox buffer */
		pFoeInd->FoeHeader.Cmd.ErrorCode			= SWAPDWORD(d);
		/* store the size of the mailbox data in the mailbox buffer */
		pFoeInd->MbxHeader.Word[MBX_OFFS_LENGTH]	= sizeof(TFOEHEADER);

		/* the file transmission sequence is finished */
		u16FileAccessState = FOE_READY;
	}

#if BOOTSTRAPMODE_SUPPORTED
	if ( bBootMode )
	{
		/* in BOOT mode the mailbox buffer is not sent via the mailbox functions
		   because only FoE is allowed in BOOT mode, so we have to include the
			mailbox data link layer counter */
		pFoeInd->MbxHeader.Byte[MBX_OFFS_COUNTER] &= ~MBX_MASK_COUNTER;
		if ( (u8MbxReadCounter & 0x07) == 0 )
			/* counter 0 is not allowed if mailbox data link layer is supported */
			u8MbxReadCounter = 1;
		/* store the counter in the mailbox header */
		pFoeInd->MbxHeader.Byte[MBX_OFFS_COUNTER] |= u8MbxReadCounter << MBX_SHIFT_COUNTER;
		/* increment the counter for the next service */
  		u8MbxReadCounter++;
		/* call the fucntion to send the mailbox service directly,
		   in BOOT mode we can be sure that the send mailbox is empty
		   because no parallel services are allowed */
		HW_CopyToSendMailbox((TMBX MBXMEM *) pFoeInd);
	}
	else
#endif
	{
/* ECATCHANGE_START(V4.00) FOE 2 */
		if ( MBX_MailboxSendReq((TMBX MBXMEM *) pFoeInd, FOE_SERVICE) != 0 )
		{
			/* if the mailbox service could not be sent (or stored), the response will be
			   stored in the variable pFoeSendStored and will be sent automatically
				from the mailbox handler (FOE_ContinueInd) when the send mailbox will be read
				the next time from the master */
			pFoeSendStored = (TMBX MBXMEM *) pFoeInd;
		}
/* ECATCHANGE_END(V4.00) FOE 2 */
	}

	return 0;
}


/* ECATCHANGE_START(V4.00) FOE 2 */
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pMbx	  Pointer to the free mailbox buffer

 \brief	This function is called when the next mailbox fragment can be sent.
*////////////////////////////////////////////////////////////////////////////////////////

void FOE_ContinueInd(TMBX MBXMEM * T)
{
	if ( pFoeSendStored )
	{
		/* send the stored FoE service which could not be sent before */
		MBX_MailboxSendReq(pFoeSendStored, 0);
		pFoeSendStored = 0;
	}
}
/* ECATCHANGE_END(V4.00) FOE 2 */

#endif // FOE_SUPPORTED
/** @} */
