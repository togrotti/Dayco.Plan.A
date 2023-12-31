/** 
\defgroup emcy emcy.c: Emergency functions
\brief This file contains the Emergency interface\n
\brief Changes to version V3.20:\n

\author  Holger Buettner
\version 4.00
\date    23.03.2007
*/

//---------------------------------------------------------------------------------------
/**
\ingroup emcy
\file emcy.c
\brief Implementation.
*/
//---------------------------------------------------------------------------------------

// Avoids warning C37: nonstandard extension - unnamed parameter

//#pragma warning disable = 37

/*---------------------------------------------------------------------------------------
------	
------	Includes
------ 
---------------------------------------------------------------------------------------*/

#include <string.h>
#include "system\os.h"

#include "ecat_def.h"

#if COE_SUPPORTED || SOE_SUPPORTED

#define	_EMCY_	1
#include "emcy.h"
#undef  _EMCY_
#define	_EMCY_	0

#if COE_SUPPORTED
#include "ecatcoe.h"
#endif
#if SOE_SUPPORTED
#include "ecatsoe.h"
#endif

/*---------------------------------------------------------------------------------------
------  
------	constants
------	
---------------------------------------------------------------------------------------*/

#if COE_SUPPORTED
const UINT16 MBXMEM acEmcyCoeHeader[4] = {SIZEOF(TCOEHEADER) + SIZEOF(UEMCY), 0, SWAPWORD(MBX_TYPE_COE<<8), SWAPWORD(COESERVICE_EMERGENCY<<12)};
#define	MAX_EMCY_SIZE	(sizeof(acEmcyCoeHeader)+sizeof(TEMCYMESSAGE))

#endif
#if SOE_SUPPORTED 
const UINT16 MBXMEM acEmcySoeHeader[3] = {SIZEOF(TSOEHEADER) + SIZEOF(UEMCY), 0, SWAPWORD(MBX_TYPE_SOE<<8)};
const UINT16 MBXMEM acNotiSoeHeader[3] = {SIZEOF(TSOEHEADER) + SIZEOF(UINT16), 0, SWAPWORD(MBX_TYPE_SOE<<8)};
#define	MAX_EMCY_SIZE	(sizeof(acEmcySoeHeader)+sizeof(TEMCYMESSAGE))

#endif
/*---------------------------------------------------------------------------------------
------  
------	Functions
------	
---------------------------------------------------------------------------------------*/

/**
\addtogroup emcy
@{
*/
#endif


#if COE_SUPPORTED || SOE_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pEmcy 			empty emergency buffer

 \brief	This function puts an emergency buffer back to the empty queue
*////////////////////////////////////////////////////////////////////////////////////////

void PutInEmptyEmcyQueue(TEMCYMESSAGE EMCYMEM * pEmcy)
{
	UINT16 lastInQueue = sEmptyEmcyQueue.LastInQueue+1;

	if (lastInQueue == sEmptyEmcyQueue.MaxQueueSize)
	{
		lastInQueue = 0;
	}


	sEmptyEmcyQueue.pQueue[sEmptyEmcyQueue.LastInQueue] = pEmcy;
	sEmptyEmcyQueue.LastInQueue = lastInQueue;
}


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return		empty emergency buffer

 \brief	This function gets an emergency buffer from the empty queue
*////////////////////////////////////////////////////////////////////////////////////////

TEMCYMESSAGE EMCYMEM * GetOutOfEmptyEmcyQueue(void)
{
	TEMCYMESSAGE EMCYMEM * pEmcy;

//	DISABLE_EMCY_INT;
	if (sEmptyEmcyQueue.FirstInQueue != sEmptyEmcyQueue.LastInQueue)
	{
		UINT16 firstInQueue = sEmptyEmcyQueue.FirstInQueue;

		pEmcy = sEmptyEmcyQueue.pQueue[firstInQueue++];
		sEmptyEmcyQueue.FirstInQueue = firstInQueue;
		if (sEmptyEmcyQueue.FirstInQueue == sEmptyEmcyQueue.MaxQueueSize)
		{
			sEmptyEmcyQueue.FirstInQueue = 0;
		}
	}
	else
		pEmcy = 0;
//	ENABLE_EMCY_INT;

	return pEmcy;
}


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pEmcy 		full emergency buffer

 \return	0: Success.

 \brief	This function puts an emergency buffer in the send queue
*////////////////////////////////////////////////////////////////////////////////////////

void PutInSendEmcyQueue(TEMCYMESSAGE EMCYMEM * pEmcy)
{
	UINT16 lastInQueue;

//	DISABLE_EMCY_INT;
	lastInQueue = sSendEmcyQueue.LastInQueue+1;

	if (lastInQueue == sSendEmcyQueue.MaxQueueSize)
	{
		lastInQueue = 0;
	}

	sSendEmcyQueue.pQueue[sSendEmcyQueue.LastInQueue] = pEmcy;
	sSendEmcyQueue.LastInQueue = lastInQueue;
//	ENABLE_EMCY_INT;
}


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return		full emergency buffer

 \brief	This function gets an emergency buffer to be sent
*////////////////////////////////////////////////////////////////////////////////////////

TEMCYMESSAGE EMCYMEM * GetOutOfSendEmcyQueue(void)
{
	TEMCYMESSAGE EMCYMEM * pEmcy;

//	DISABLE_EMCY_INT;
	if (sSendEmcyQueue.FirstInQueue != sSendEmcyQueue.LastInQueue)
	{
		UINT16 firstInQueue = sSendEmcyQueue.FirstInQueue;

		pEmcy = sSendEmcyQueue.pQueue[firstInQueue++];
		sSendEmcyQueue.FirstInQueue = firstInQueue;
		if (sSendEmcyQueue.FirstInQueue == sSendEmcyQueue.MaxQueueSize)
		{
			sSendEmcyQueue.FirstInQueue = 0;
		}
	}
	else
		pEmcy = 0;
//	ENABLE_EMCY_INT;

	return pEmcy;
}


/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief	This function intializes the Emergency Interface.
*////////////////////////////////////////////////////////////////////////////////////////

void EMCY_Init(void)
{
	UINT8 i;

	/* initialize send queue */
	sSendEmcyQueue.FirstInQueue					= 0;
	sSendEmcyQueue.LastInQueue					= 0;
	sSendEmcyQueue.PutInQueueFailedCounter	= 0;
	/* Bug 27.07: MaxQueueSize has to be set to the number of elements */
	sSendEmcyQueue.MaxQueueSize					= MAX_EMERGENCIES+1;
	for (i = 0; i < MAX_EMERGENCIES; i++)
	{
		sSendEmcyQueue.pQueue[i] = 0;
	}

	/* initialize empty queue */
	sEmptyEmcyQueue.FirstInQueue					= 0;
	sEmptyEmcyQueue.LastInQueue					= 0;
	sEmptyEmcyQueue.PutInQueueFailedCounter	= 0;
	/* Bug 27.07: MaxQueueSize has to be set to the number of elements */
	sEmptyEmcyQueue.MaxQueueSize					= MAX_EMERGENCIES+1;
	for (i = 0; i < MAX_EMERGENCIES; i++)
	{
		sEmptyEmcyQueue.pQueue[i] = 0;
	}

	/* put all buffers in empty queue */
	for (i = 0; i < MAX_EMERGENCIES; i++)
		PutInEmptyEmcyQueue(&asEmcy[i]);
}


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return		1 is the emergency queue is empty

 \brief	This function checks if the emergency queue is empty
*////////////////////////////////////////////////////////////////////////////////////////

UINT8 EMCY_IsQueueEmpty(void)
{
	return (sSendEmcyQueue.FirstInQueue == sSendEmcyQueue.LastInQueue);
}


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pMbx	  empty mailbox buffer

 \brief	This function is called when the next mailbox command can be stored the ESC.
*////////////////////////////////////////////////////////////////////////////////////////

void EMCY_ContinueInd(TMBX MBXMEM * T)
{
	TEMCYMESSAGE EMCYMEM *pEmcy = GetOutOfSendEmcyQueue();

	{
		/* send next emergency message */
		EMCY_SendEmergency( pEmcy );
	}
}


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return					empty emergency buffer 

 \brief	This function gets an empty emergency buffer 
*////////////////////////////////////////////////////////////////////////////////////////

TEMCYMESSAGE EMCYMEM * EMCY_GetEmcyBuffer(void)
{
#if SOE_SUPPORTED
	TEMCYMESSAGE EMCYMEM * pEmcy = GetOutOfEmptyEmcyQueue();
	pEmcy->SoeHeader[0] = SWAPWORD(ECAT_SOE_OPCODE_EMCY);
	pEmcy->SoeHeader[1] = 0;

	return pEmcy;
#else
	TEMCYMESSAGE EMCYMEM * pEmcy;

	// HBu 02.05.06: when using the mailbox event in an ISR it should be disabled here
	DISABLE_MBX_INT;
	pEmcy = GetOutOfEmptyEmcyQueue();
	// HBu 02.05.06: when using the mailbox event in an ISR it should be enabled here
	ENABLE_MBX_INT;
	return pEmcy;
#endif
}	
	

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pEmcy				emergency message to be sent

 \brief	This function sends an emergency message
*////////////////////////////////////////////////////////////////////////////////////////

UINT8 EMCY_SendEmergency( TEMCYMESSAGE EMCYMEM *pEmcy )
{
	// HBu 02.05.06: when using the mailbox event in an ISR it should be disabled here
	DISABLE_MBX_INT;
	if ( bSendMbxIsFull || nAlStatus == STATE_INIT )
	{
		/* send mailbox is full store the emergency message */
		PutInSendEmcyQueue( pEmcy );
	
		u8MailboxSendReqStored |= EMCY_SERVICE;
	// HBu 02.05.06: when using the mailbox event in an ISR it should be enabled here
	ENABLE_MBX_INT;
		return 0;
	}

	/* buffer psWriteMbx is free if bSendMbxIsFull is not set */
#if COE_SUPPORTED
	OBJTOMBXMEMCPY(psWriteMbx, acEmcyCoeHeader, sizeof(acEmcyCoeHeader));
	MBXMEMCPY(&psWriteMbx->Data[1], pEmcy, sizeof(TEMCYMESSAGE));
#endif
#if SOE_SUPPORTED
	OBJTOMBXMEMCPY(psWriteMbx, acEmcySoeHeader, sizeof(acEmcySoeHeader));
	MBXMEMCPY(&psWriteMbx->Data[1], pEmcy, sizeof(TEMCYMESSAGE));
	if	( (SWAPWORD(((TSOEHEADER EMCYMEM *) pEmcy->SoeHeader)->Flags.Word) & SOEFLAGS_OPCODE) == ECAT_SOE_OPCODE_NFC )
		OBJTOMBXMEMCPY(psWriteMbx, acNotiSoeHeader, sizeof(acNotiSoeHeader));
#if COE_SUPPORTED
	else
		OBJTOMBXMEMCPY(psWriteMbx, acEmcyCoeHeader, sizeof(acEmcySoeHeader));
#endif
#endif

	// HBu 02.05.06: emergency buffer has to be put in the empty queue only
	//               if the sending was successful
	if (MBX_MailboxSendReq(psWriteMbx, EMCY_SERVICE) == 0)
		/* put emergency buffer back in the empty queue */
		PutInEmptyEmcyQueue( pEmcy );
	// HBu 02.05.06: when using the mailbox event in an ISR it should be enabled here
	ENABLE_MBX_INT;

	return 0;
}


#endif // COE_SUPPORTED || SOE_SUPPORTED

/** @} */
