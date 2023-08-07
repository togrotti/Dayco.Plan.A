/*-----------------------------------------------------------------------------------------
------	
------	emcy.h
------			 																																					------
-----------------------------------------------------------------------------------------*/

#ifndef _EMCY_H_
#define _EMCY_H_

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/

#include "mailbox.h"
#ifndef EMCY_PROTOCOL
	#define	EMCY_PROTOCOL	MBX_TYPE_COE
#endif
#if EMCY_PROTOCOL == MBX_TYPE_COE && COE_SUPPORTED
#include "ecatcoe.h"
#endif
#if EMCY_PROTOCOL == MBX_TYPE_SOE && SOE_SUPPORTED
#include "ecatsoe.h"
#endif

/*-----------------------------------------------------------------------------------------
------	
------	Defines and Types
------	
-----------------------------------------------------------------------------------------*/

///////////////////////////////////////////////////////////////
//
//	General
//

#ifndef	DISABLE_EMCY_INT
	#define	DISABLE_EMCY_INT		Os_BeginCriticalSection(OS_CRITSECT_GLOBAL)
#endif

#ifndef	ENABLE_EMCY_INT
	#define	ENABLE_EMCY_INT			Os_EndCriticalSection(OS_CRITSECT_GLOBAL)
#endif

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// Emergency error codes
*/

#define	ERROR_PREOP_2_SAFEOP			0xA000
#define	ERROR_SAFEOP_2_OP				0xA001

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// Data Types
*/

typedef struct STRUCT_PACKED
{
	UINT16						FirstInQueue;					
	UINT16						LastInQueue;					
	UINT16						PutInQueueFailedCounter;	
	UINT16						MaxQueueSize;					
	TEMCYMESSAGE EMCYMEM *	pQueue[MAX_EMERGENCIES+1];
} TEMCYQUEUE;

#if !COE_SUPPORTED
	#define	COESERVICE_EMERGENCY		0x01
#endif

#endif //_EMCY_H_

/*-----------------------------------------------------------------------------------------
------	
------	global variables
------	
-----------------------------------------------------------------------------------------*/
#ifndef _EMCY_
#define _EMCY_  0
#endif

#if _EMCY_
	#define PROTO
#else 
	#define PROTO extern
#endif
PROTO	TEMCYQUEUE				sSendEmcyQueue;
PROTO	TEMCYQUEUE				sEmptyEmcyQueue;
PROTO	TEMCYMESSAGE EMCYMEM	asEmcy[MAX_EMERGENCIES];

/*-----------------------------------------------------------------------------------------
------  
------	global functions
------	
-----------------------------------------------------------------------------------------*/

PROTO	void PutInEmptyEmcyQueue(TEMCYMESSAGE EMCYMEM * pEmcy);
PROTO	TEMCYMESSAGE EMCYMEM * GetOutOfEmptyEmcyQueue(void);
PROTO	void PutInSendEmcyQueue(TEMCYMESSAGE EMCYMEM * pEmcy);
PROTO	TEMCYMESSAGE EMCYMEM * GetOutOfSendEmcyQueue(void);
PROTO	void EMCY_Init(void);
PROTO	UINT8 EMCY_IsQueueEmpty(void);
PROTO	void EMCY_ContinueInd(TMBX MBXMEM * pMbx);
PROTO	TEMCYMESSAGE EMCYMEM * EMCY_GetEmcyBuffer(void);
PROTO	UINT8 EMCY_SendEmergency(TEMCYMESSAGE EMCYMEM *pEmcy);

#undef PROTO

