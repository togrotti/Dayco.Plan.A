/** 
\defgroup mailbox mailbox.c: Mailbox functions
\brief This file conatins the EtherCAT mailbox handler.
\brief Changes to version V3.20:\n

\author  Holger Buettner
\version 4.00
\date    23.03.2007
*/

//---------------------------------------------------------------------------------------
/**
\ingroup mailbox	
\file	mailbox.c
\brief Implementation.
*/
//---------------------------------------------------------------------------------------

/*
\brief Description of the mailbox buffer handling (MAILBOX_QUEUE = 0):\n
\brief There are two mailbox buffer for sending and receiving mailbox services.\n
\brief Normal operation:\n
\brief When starting the mailbox handler psWriteMbx contains mailbox buffer 1,\n
\brief psReadMbx, psRepeatMbx and psStoreMbx are 0.\n
\brief In this state a repeat request would be ignored because there was no service sent yet.\n
\brief When the first mailbox service is sent (in HW_CopyToSendMbx), psWriteMbx gets mailbox buffer 2\n
\brief and psReadMbx gets the sent mailbox buffer 1, psRepeatMbx and psStoreMbx are still 0.\n
\brief When the first mailbox service was read from the master, the sent mailbox buffer 1 is stored\n
\brief in psRepeatMbx (in MBX_MailboxReadInd).\n
\brief After that psReadMbx gets always the actual sent mailbox buffer, psWriteMbx is set to 0 (another\n
\brief received mailbox service from the master will not be handled until the sent mailbox service was read\n
\brief and MBX_MailboxReadInd was called).\n
\brief When the mailbox service is read, psWriteMbx gets the Buffer of psRepeatMbx and psRepeatMbx gets the \n
\brief buffer of psReadMbx.\n
\brief Repeat Request from the master:\n
\brief When a Repeat from the master is requested (MBX_MailboxRepeatReq), there are three different possibilities:\n
\brief 1. no mailbox service was sent since the mailbox handler was started (psRepeatMbx = 0): nothing to do\n
\brief 2. the acknowledge of the last sent mailbox service was received (in MBX_MailboxReadInd) (bSendMbxIsFull = 0):\n
\brief    the last sent mailbox service (psRepeatMbx) will be sent again (in HW_CopyToSendMbx) and stored in psReadMbx,\n
\brief    psRepeatMbx will be set to 0\n
\brief 3. the acknowledge of the last sent mailbox service was not received (psReadMbx and psRepeatMbx contain different buffers,\n
\brief    psReadMbx is still in the mailbox (because MBX_MailboxReadInd is not called yet, bSendMbxIsFull = 1): \n
\brief    psReadMbx will be deleted in the mailbox (call of HW_DisableSyncManChannel and HW_EnableSyncManChannel) and\n
\brief    stored in psStoreMbx, psRepeatMbx will be sent again (in HW_CopyToSendMbx) and stored in psReadMbx,\n
\brief    psRepeatMbx will be set to 0.\n
\brief    When the repeated mailbox service was sent (call of MBX_MailboxReadInd), psReadMbx will be stored in psRepeatMbx\n
\brief    and psStoreMbx will be sent (in HW_CopyToSendMbx) and stored in psReadMbx, psStoreMbx will be set to 0.\n

\brief Description of the mailbox buffer handling (MAILBOX_QUEUE = 1):\n
\brief There are two mailbox buffer for sending and receiving mailbox services.\n
\brief Normal operation (psWriteMbx is only used for local storage):\n
\brief When starting the mailbox handler psReadMbx, psRepeatMbx and psStoreMbx are 0.\n
\brief In this state a repeat request would be ignored because there was no service sent yet.\n
\brief When a mailbox service is received from the master (in HW_CheckAndCopyMailbox) a mailbox buffer\n
\brief will be get with APPL_AllocMailboxBuffer and the corresponding protocol service function will\n
\brief be called (in MBX_WriteMailboxInd). This buffer shall be used for the protocol service response.\n
\brief When the first mailbox service is sent (in HW_CopyToSendMbx), psReadMbx gets the sent mailbox buffer,\n
\brief psRepeatMbx and psStoreMbx are still 0.\n
\brief When the first mailbox service was read from the master, the sent mailbox buffer (psReadMbx) is stored\n
\brief in psRepeatMbx (in MBX_MailboxReadInd).\n
\brief After that psReadMbx gets always the actual sent mailbox buffer (in HW_CopyToSendMbx)\n
\brief When the mailbox service is read, psRepeatMbx is returned (with APPL_FreeMailboxBuffer) and psRepeatMbx gets the \n
\brief buffer of psReadMbx.\n
\brief Repeat Request from the master:\n
\brief When a Repeat from the master is requested (MBX_MailboxRepeatReq), there are three different possibilities:\n
\brief 1. no mailbox service was sent since the mailbox handler was started (psRepeatMbx = 0): nothing to do\n
\brief 2. the acknowledge of the last sent mailbox service was received (in MBX_MailboxReadInd) (bSendMbxIsFull = 0):\n
\brief    the last sent mailbox service (psRepeatMbx) will be sent again (in HW_CopyToSendMbx) and stored in psReadMbx,\n
\brief    psRepeatMbx will be set to 0\n
\brief 3. the acknowledge of the last sent mailbox service was not received (psReadMbx and psRepeatMbx contain different buffers,\n
\brief    psReadMbx is still in the mailbox (because MBX_MailboxReadInd is not called yet, bSendMbxIsFull = 1): \n
\brief    psReadMbx will be deleted in the mailbox (call of HW_DisableSyncManChannel and HW_EnableSyncManChannel) and\n
\brief    stored in psStoreMbx, psRepeatMbx will be sent again (in HW_CopyToSendMbx) and stored in psReadMbx,\n
\brief    psRepeatMbx will be set to 0.\n
\brief    When the repeated mailbox service was sent (call of MBX_MailboxReadInd), psReadMbx will be stored in psRepeatMbx\n
\brief    and psStoreMbx will be sent (in HW_CopyToSendMbx) and stored in psReadMbx, psStoreMbx will be set to 0.\n
*/

/*---------------------------------------------------------------------------------------
------	
------	Includes
------ 
---------------------------------------------------------------------------------------*/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

#include "common\CommonDefines.h"
#include <stdlib.h>

#include "ecat_def.h"


#define	_MAILBOX_	1
#include "mailbox.h"
#undef _MAILBOX_
#define	_MAILBOX_	0

#if COE_SUPPORTED
#include "ecatcoe.h"
#endif
#if EOE_SUPPORTED
#include "ecateoe.h"
#endif
#if FOE_SUPPORTED
#include "ecatfoe.h"
#endif
#include "emcy.h"
#include "coeappl.h"

/*--------------------------------------------------------------------------------------
------	
------	internal Types and Defines
------	
--------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------
------	
------	internal Variables
------	
--------------------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------------------
------  
------	internal functions
------	
--------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
------  
------	functions
------	
---------------------------------------------------------------------------------------*/

/**
\addtogroup mailbox
@{
*/


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief	This function intializes the Mailbox Interface.
*////////////////////////////////////////////////////////////////////////////////////////

void MBX_Init(void)
{
	u16ReceiveMbxSize = MIN_MBX_SIZE;
	u16SendMbxSize = MAX_MBX_SIZE;
	u16EscAddrReceiveMbx = MIN_MBX_WRITE_ADDRESS;
	u16EscAddrSendMbx = MIN_MBX_READ_ADDRESS;


	psWriteMbx = &asMbx[0];
	psRepeatMbx = NULL;
	psReadMbx = NULL;
	psStoreMbx = NULL;

	bMbxRepeatToggle	= FALSE;
	bMbxRunning = FALSE;
	bSendMbxIsFull = FALSE;
	bReceiveMbxIsLocked = FALSE;
	u8MailboxSendReqStored	= 0;
	u8MbxWriteCounter = 0;
	u8MbxReadCounter	= 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief	This function includes the state transtion from INIT to
 \brief	PRE-OPERATIONAL in the EtherCAT Slave corresponding to 
 \brief  local management service Start Mailbox Handler
 \brief  it is checked if the mailbox areas overlaps each other
 \brief  and the Sync Manager channels 0 and 1 are enabled
*////////////////////////////////////////////////////////////////////////////////////////

UINT8 MBX_StartMailboxHandler(void)
{
	/* get address of the receive mailbox sync manager */
	TSYNCMAN ESCMEM * pSyncMan = HW_GetSyncMan(MAILBOX_WRITE);
	/* store size of the receive mailbox */
	u16ReceiveMbxSize 	= pSyncMan->Length;
	/* store the address of the receive mailbox */
	u16EscAddrReceiveMbx = pSyncMan->PhysicalStartAddress;

	/* get address of the send mailbox sync manager */
	pSyncMan = HW_GetSyncMan(MAILBOX_READ);
	/* store the size of the send mailbox */
	u16SendMbxSize = pSyncMan->Length;
	/* store the address of the send mailbox */
	u16EscAddrSendMbx = pSyncMan->PhysicalStartAddress;

	// HBu 02.05.06: it should be checked if there are overlaps in the sync manager areas
	if ((u16EscAddrReceiveMbx+u16ReceiveMbxSize) > u16EscAddrSendMbx && (u16EscAddrReceiveMbx < (u16EscAddrSendMbx+u16SendMbxSize)))
	{
		return ALSTATUSCODE_INVALIDMBXCFGINPREOP;
	}
	/* enable the receive mailbox sync manager channel */
	HW_EnableSyncManChannel(MAILBOX_WRITE);
	/* enable the send mailbox sync manager channel */
	HW_EnableSyncManChannel(MAILBOX_READ);
	/* mailbox handler is running */
	bMbxRunning = TRUE;

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief	This function includes the state transtion from 
 \brief	PRE-OPERATIONAL to INIT in the EtherCAT Slave corresponding to 
 \brief  local management service Stop Mailbox Handler
 \brief  the Sync Manager channels 0 and 1 are disabled
*////////////////////////////////////////////////////////////////////////////////////////

void MBX_StopMailboxHandler(void)
{
	/* mailbox handler is stopped */
	bMbxRunning = FALSE;
	/* disable the receive mailbox sync manager channel */
	HW_DisableSyncManChannel(MAILBOX_WRITE);
	/* disable the send mailbox sync manager channel */
	HW_DisableSyncManChannel(MAILBOX_READ);
	/* initialize variables again */
	psWriteMbx = &asMbx[0];
	psRepeatMbx = NULL;
	psReadMbx = NULL;
	psStoreMbx = NULL;
	bMbxRepeatToggle	= FALSE;
	bSendMbxIsFull = FALSE;
	bReceiveMbxIsLocked = FALSE;
	u8MailboxSendReqStored	= 0;
	u8MbxWriteCounter = 0;
	u8MbxReadCounter	= 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pMbx		Pointer to the received Mailbox command from Master.
  
 \brief	This function is called when the Master has written the Receive-Mailbox. 
 \brief	It will only be called if the send mailbox is empty, that a response for the
 \brief	mailbox service could be stored.
 \brief	The function checks the mailbox header for the requested service and calls the 
 \brief	corresponding XXXX_ServiceInd-function
*////////////////////////////////////////////////////////////////////////////////////////

void MBX_MailboxWriteInd(TMBX MBXMEM *pMbx)
{
	UINT8 result = 0;
	UINT8 mbxCounter = (UINT8)(pMbx->MbxHeader.Byte[MBX_OFFS_COUNTER] >> MBX_SHIFT_COUNTER);

	/* if the mailbox datagram counter (Bit 4-6 of Byte 5 of the mailbox header) is unequal zero, 
	   the master supports the mailbox data link layer,
		in that case a repeated mailbox write request will be detected, if the counter is unequal zero
		and unchanged */
	if ( mbxCounter == 0 || mbxCounter != u8MbxWriteCounter )
	{
		/* new mailbox service received */
		/* mbxCounter = 0: old EtherCAT master */
		/* new MBX service received, store the new mailbox counter */
		u8MbxWriteCounter = mbxCounter;
		/* check the protocol type and call the XXXX_ServiceInd-function */	
		switch ( (pMbx->MbxHeader.Byte[MBX_OFFS_TYPE] & MBX_MASK_TYPE) >> MBX_SHIFT_TYPE )
		{
#if COE_SUPPORTED
		case MBX_TYPE_COE:
			/* CoE datagram received */
			result = COE_ServiceInd((TCOEMBX MBXMEM *) pMbx);
			break;

#endif /* COE_SUPPORTED */
#if SOE_SUPPORTED
		case MBX_TYPE_SOE:
			/* SoE datagram received */
			result = SOE_ServiceInd(pMbx);
			break;

#endif /* SOE_SUPPORTED */
#if EOE_SUPPORTED
		case MBX_TYPE_EOE:
			/* EoE datagram received */
			result = EOE_ServiceInd(pMbx);
			break;

#endif /* EOE_SUPPORTED */
#if FOE_SUPPORTED
		case MBX_TYPE_FOE:
			/* FoE datagram received */
			result = FOE_ServiceInd((TFOEMBX MBXMEM *) pMbx);
			break;

#endif /* FOE_SUPPORTED */
#if VOE_SUPPORTED
		case MBX_TYPE_VOE:
			/* VoE datagram received */
			result = VOE_ServiceInd((TVOEMBX MBXMEM *) pMbx);
			break;

#endif /* VOE_SUPPORTED */
		default:
			
			result = MBXERR_UNSUPPORTEDPROTOCOL;
			break;
		}

		if ( result != 0 )
		{
			/* Mailbox error response: type 0 (mailbox service protocol) */
			pMbx->MbxHeader.Word[MBX_OFFS_LENGTH] 	= 4;
			pMbx->MbxHeader.Byte[MBX_OFFS_TYPE]		&= ~((UINT8) MBX_MASK_TYPE);
			pMbx->Data[0]						= SWAPWORD(MBXSERVICE_MBXERRORCMD);
			pMbx->Data[1]						= SWAPWORD(result);
			MBX_MailboxSendReq(pMbx, 0);
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief This function is called when the Master has read the Send-Mailbox.
*////////////////////////////////////////////////////////////////////////////////////////

void MBX_MailboxReadInd(void)
{
	bSendMbxIsFull = FALSE;

	// HBu 02.05.06: the pointer psRepeatMbx is only free if there is no stored
	//               mailbox service from the last repeat 
	if ( psRepeatMbx && psStoreMbx == NULL )
	{
		/* the last sent service is not stored for repeat any longer */
		psWriteMbx = psRepeatMbx;
	}
	
	/* the actual sent service has to be stored for repeat */	
	psRepeatMbx = psReadMbx;
	
  	if ( psStoreMbx )
  	{
		/* there was a buffer stored */
		HW_CopyToSendMailbox(psStoreMbx);
		/* no more buffer to be stored any more */
		psStoreMbx = NULL;
  	}
  	else
  	if ( u8MailboxSendReqStored ) 
	{
		/* there are mailbox services stored to be sent */
#if COE_SUPPORTED || SOE_SUPPORTED
		if ( u8MailboxSendReqStored & EMCY_SERVICE )
		{
			/* call EMCY function that will send the stored Emergency service */
			EMCY_ContinueInd(psWriteMbx);
			if (EMCY_IsQueueEmpty())
			{
				u8MailboxSendReqStored &= ~EMCY_SERVICE;			
			}
		}
		else
#endif
#if COE_SUPPORTED
		if ( u8MailboxSendReqStored & COE_SERVICE )
		{
			/* reset the flag indicating that CoE service to be sent was stored */
			u8MailboxSendReqStored &= ~COE_SERVICE;
			/* call CoE function that will send the stored CoE service */
			COE_ContinueInd(psWriteMbx);
		}
		else
#endif
#if SOE_SUPPORTED
		if ( u8MailboxSendReqStored & SOE_SERVICE )
		{
			/* reset the flag indicating that SoE service to be sent was stored */
			u8MailboxSendReqStored &= ~SOE_SERVICE;
			/* call CoE function that will send the stored SoE service */
			SOE_ContinueInd(psWriteMbx);
		}
		else
#endif
#if EOE_SUPPORTED
		if ( u8MailboxSendReqStored & EOE_SERVICE )
		{
			/* reset the flag indicating that EoE service to be sent was stored */
			u8MailboxSendReqStored &= ~EOE_SERVICE;
			/* call EoE function that will send the stored EoE service */
			EOE_ContinueInd(psWriteMbx);
		}
		else
#endif
#if FOE_SUPPORTED
/* ECATCHANGE_START(V4.00) FOE 2 */
		if ( u8MailboxSendReqStored & FOE_SERVICE )
		{
			/* reset the flag indicating that FoE service to be sent was stored */
			u8MailboxSendReqStored &= ~FOE_SERVICE;
			/* call FoE function that will send the stored FoE service */
			FOE_ContinueInd(psWriteMbx);
		}
		else
/* ECATCHANGE_END(V4.00) FOE 2 */
#endif
#if VOE_SUPPORTED
		if ( u8MailboxSendReqStored & VOE_SERVICE )
		{
			/* reset the flag indicating that VoE service to be sent was stored */
			u8MailboxSendReqStored &= ~VOE_SERVICE;
			/* call CoE function that will send the stored VoE service */
			VOE_ContinueInd(psWriteMbx);
		}
		else
#endif
		{
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief This function is called if the Master has requested a resending of the last
 \brief sent mailbox
*////////////////////////////////////////////////////////////////////////////////////////

void MBX_MailboxRepeatReq(void)
{
	if (psRepeatMbx)
	{
		TMBX MBXMEM *pMbx = psRepeatMbx;
		/* send mailbox service stored for repeat */
		/* HBu 13.10.06: if a repeat request is received (again) before the previuosly repeated mailbox telegram
		   was read from the master (psStoreMbx != NULL) the next mailbox telegram to be sent is still in the
			read mailbox so it has not to updated exchanged */

		if (bSendMbxIsFull && psStoreMbx == NULL)
		{
			UINT8 smStatus = SM_BUFFERWRITTEN;
			/* mailbox is full, take the buffer off */
			HW_DisableSyncManChannel(MAILBOX_READ);
			/* HBu 13.10.06: if the ESC is receiving a telegram it waits with disabling the sync manager 
			                 until the telegram is completely received, so we have to wait here until
								  the buffer is empty */
			while (smStatus & SM_BUFFERWRITTEN)
			{
				HW_EscReadAccess( &smStatus, ESC_ADDR_SM_MBXREAD+ESC_OFFS_SMSETTINGS+ESC_OFFS_SMSTATUS, 1 );
			}
			/* store the buffer to be sent next */
			psStoreMbx = psReadMbx;
			/* enable the mailbox again */
			HW_EnableSyncManChannel(MAILBOX_READ);
			/* HBu 15.02.06: flag has to be reset otherwise the mailbox service 
			                 will not be copied by HW_CopyToSendMailbox */
			bSendMbxIsFull = FALSE;
		}

		HW_CopyToSendMailbox(pMbx);
	}

	// Repeat was finished, toggle the acknowledge bit
	bMbxRepeatToggle = !bMbxRepeatToggle;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pMbx			Pointer to a Mailbox command to be sent (read by the Master)
 \param flags			Bit 0-6:	mailbox protocol type:
										0x01 - emergency service
										0x02 - CoE service
										0x04 - SoE service
										0x80 - EoE service
										0x10 - AoE service
										0x20 - VoE service
 							Bit 7:   0 - no more fragments to be sent for the reauested mailbox service
									   1 - additional fragments to be sent for the mailbox service, the
										    correspondig XXXX_ContinueInd-function will be called to get
										    the next fragment

 \return	0: Success - mailbox command could be stored in the send mailbox
			1: Failed - mailbox command could not be stored in the send mailbox, the 
							XXXX_ContinueInd service will be called when the mailbox was
							read from the master to 

 \brief		This function puts a new Mailbox service in the Send Mailbox
*////////////////////////////////////////////////////////////////////////////////////////

UINT8 MBX_MailboxSendReq( TMBX MBXMEM * pMbx, UINT8 flags )
{
	UINT8 result = 0;

	/* HBu 06.02.06: in INIT-state a mailbox send request shall be refused */
	if ( (nAlStatus & STATE_MASK) == STATE_INIT )
		return ERROR_INVALIDSTATE;


	DISABLE_MBX_INT;
	/* the counter in the mailbox header has to be incremented with every new mailbox service to be sent 
	   if the mailbox data link layer is supported (software switch MAILBOX_REPEAT_SUPPORTED set)*/
	pMbx->MbxHeader.Byte[MBX_OFFS_COUNTER] &= ~MBX_MASK_COUNTER;
	/* HBu 13.02.06: Repeat-Counter was incremented too much if the mailbox service could not be sent */
	/* u8MbxCounter holds the actual counter for the mailbox header, only the values
	   1-7 are allowed if the mailbox data link layer is supported  */
	if ( (u8MbxReadCounter & 0x07) == 0 )
		u8MbxReadCounter = 1;
	pMbx->MbxHeader.Byte[MBX_OFFS_COUNTER] |= u8MbxReadCounter << MBX_SHIFT_COUNTER;

	/* try to copy the mailbox comamnd in the ESC */
	if ( HW_CopyToSendMailbox(pMbx) != 0 )	  
	{
		/* no success, send mailbox was full, set flag  */
		flags |= FRAGMENTS_FOLLOW;
		result = 1;
	}
	/* HBu 13.02.06: Repeat-Counter was incremented too much if the mailbox service could not be sent */
	else
	{
		u8MbxReadCounter++;
	}

	if ( flags & FRAGMENTS_FOLLOW )
	{
		/* store the mailbox service that the corresponding XXX_ContinueInd function will
		   be called when the send mailbox will have been read by the master because there
		   are mailbox commands to be sent for this service */
		u8MailboxSendReqStored |= (flags & ((UINT8) ~FRAGMENTS_FOLLOW));
	}

	ENABLE_MBX_INT;


	return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief	This function is called cyclically to check if a received Mailbox service was 
 			stored.
*////////////////////////////////////////////////////////////////////////////////////////

void MBX_Main(void)
{
  	if ( bReceiveMbxIsLocked )
  	{
  		/* the work on the receive mailbox is locked, check if it can be unlocked (if all
  		   mailbox commands has been sent */
  		HW_CheckAndCopyMailbox();
  	}
}


/** @} */
