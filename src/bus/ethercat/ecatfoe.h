/*-----------------------------------------------------------------------------------------
------	
------	ecatfoe.h
------
-----------------------------------------------------------------------------------------*/

#ifndef _ECATFOE_H_
#define _ECATFOE_H_

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/

#include "mailbox.h"

/*-----------------------------------------------------------------------------------------
------	
------	Defines and Types
------	
-----------------------------------------------------------------------------------------*/

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// Error Codes
*/

#define	ECAT_FOE_ERRCODE_NOTDEFINED		0x8000
#define	ECAT_FOE_ERRCODE_NOTFOUND			0x8001
#define	ECAT_FOE_ERRCODE_ACCESS				0x8002
#define	ECAT_FOE_ERRCODE_DISKFULL			0x8003
#define	ECAT_FOE_ERRCODE_ILLEAGAL			0x8004
#define	ECAT_FOE_ERRCODE_PACKENO			0x8005
#define	ECAT_FOE_ERRCODE_EXISTS				0x8006
#define	ECAT_FOE_ERRCODE_NOUSER				0x8007
#define	ECAT_FOE_ERRCODE_BOOTSTRAPONLY	0x8008
#define	ECAT_FOE_ERRCODE_NOTINBOOTSTRAP	0x8009
#define	ECAT_FOE_ERRCODE_NORIGHTS			0x800A
#define	ECAT_FOE_ERRCODE_PROGERROR			0x800B

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// Services
*/

#define	ECAT_FOE_OPMODE_RRQ				1
#define	ECAT_FOE_OPMODE_WRQ				2
#define	ECAT_FOE_OPMODE_DATA				3
#define	ECAT_FOE_OPMODE_ACK				4
#define	ECAT_FOE_OPMODE_ERR				5
#define	ECAT_FOE_OPMODE_BUSY				6

/*///////////////////////////////////////////////////////////////////////////////////////
//
// States
*/

#define	FOE_READY							0
#define	FOE_WAIT_FOR_ACK					1
#define	FOE_WAIT_FOR_DATA					2
#define	FOE_WAIT_FOR_RESET				3
#define	FOE_WAIT_FOR_LAST_ACK			4
#define	FOE_PROGRAMMING					5
#define	FOE_WAIT_FOR_LAST_DATA			6

/*///////////////////////////////////////////////////////////////////////////////////////
//
// Results
*/

#define	FOE_ERROR							0x8000
#define	FOE_FINISHED						(FOE_ERROR-1)
#define	FOE_FINISHED_NOACK			    	(FOE_FINISHED-1)
#define	FOE_ACK								(FOE_FINISHED_NOACK-1)
#define	FOE_ACK_LAST						(FOE_ACK-1)
#define	FOE_MAXBUSY							(FOE_ACK_LAST-1)

#if BOOTSTRAPMODE_SUPPORTED
#define	FOE_MAXDATA							(sizeof(TEFWUPDATE))
#else
#define	FOE_MAXDATA							MAX_FILE_SIZE
#endif

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// Structures
*/

typedef struct STRUCT_PACKED
{
	UINT16 (*pFoeRead)(UINT16 MBXMEM * pName, UINT16 nameSize, UINT16 MBXMEM * pData, UINT32 password);
	UINT16 (*pFoeWrite)(UINT16 MBXMEM * pName, UINT16 nameSize, UINT32 password);
	UINT16 (*pFoeData)(UINT16 MBXMEM * pData, UINT16 Size);
	UINT16 (*pFoeAck)(UINT32 fileOffset, UINT16 MBXMEM * pData);
	UINT16 (*pFoeBusy)(UINT16 done, UINT32 fileOffset, UINT16 MBXMEM * pData);
	void (*pFoeError)(UINT32 errorCode);
} TFoeFuncPtr;

#endif //_ECATFOE_H_

/*-----------------------------------------------------------------------------------------
------	
------	global variables
------	
-----------------------------------------------------------------------------------------*/

#ifdef _ECATFOE_
	#define PROTO 
#else 
	#define PROTO extern
#endif

PROTO	UINT16								u16PacketNo;				/* stores the actual packet number to be expected */
PROTO	UINT32								u32FileOffset;			/* stores the next file offset to be sent */
PROTO	UINT32								u32LastFileOffset;		/* stores the actual sent file offset */
PROTO	UINT16								u16FileAccessState;		/* stores the actual state of the file transmission sequence */
PROTO	TMBX MBXMEM *						pFoeSendStored;			/* if the mailbox service could not be sent (or stored), 
																					the FoE service will be stored in this variable 
																					and will be sent automatically from the mailbox handler 
																					(FOE_ContinueInd) when the send mailbox will be read
																					the next time from the master */
#if BOOTSTRAPMODE_SUPPORTED
PROTO	TFoeFuncPtr							sFoeFuncPtr;
#endif

/*-----------------------------------------------------------------------------------------
------  
------	global functions
------				 
-----------------------------------------------------------------------------------------*/

PROTO	void 	FOE_Init(void);
PROTO	UINT8 FOE_ServiceInd(TFOEMBX MBXMEM * pFoeInd);
/* ECATCHANGE_START(V4.00) FOE 2 */
PROTO	void 	FOE_ContinueInd(TMBX MBXMEM * pMbx);
/* ECATCHANGE_END(V4.00) FOE 2 */

#undef PROTO

