/*-----------------------------------------------------------------------------------------
------
------	Description
------	
------	mailbox.h
------
------	EtherCAT Mailbox
------			 																																					------
-----------------------------------------------------------------------------------------*/

#ifndef _MAILBOX_H_
#define _MAILBOX_H_

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/

#include "esc.h"
#include "ecatslv.h"

/*-----------------------------------------------------------------------------------------
------	
------	Defines and Types
------	
-----------------------------------------------------------------------------------------*/

///////////////////////////////////////////////////////////////
//
//	General
//

#define	SYNC_CHANNEL_MBX_WRITE			0
#define	SYNC_CHANNEL_MBX_READ			1

#define 	MBX_TYPE_AOE						1
#define 	MBX_TYPE_EOE						2
#define 	MBX_TYPE_COE						3
#define 	MBX_TYPE_FOE						4
#define 	MBX_TYPE_SOE						5
#define 	MBX_TYPE_VOE						15

#define	EMCY_SERVICE						((UINT8) 0x0001)
#define	COE_SERVICE							((UINT8) 0x0002)
#define	SOE_SERVICE							((UINT8) 0x0004)
#define	EOE_SERVICE							((UINT8) 0x0008)
#define	AOE_SERVICE							((UINT8) 0x0010)
#define	VOE_SERVICE							((UINT8) 0x0020)
/* ECATCHANGE_START(V4.00) FOE 2 */
#define	FOE_SERVICE							((UINT8) 0x0040)
/* ECATCHANGE_END(V4.00) FOE 2 */
#define	FRAGMENTS_FOLLOW					((UINT8) 0x0080)

	#ifndef DISABLE_MBX_INT
		#define	DISABLE_MBX_INT
	#endif
	#ifndef ENABLE_MBX_INT
		#define	ENABLE_MBX_INT	
	#endif

///////////////////////////////////////////////////////////////
//
//	Command Codes for the mailbox type 0
//

#define	MBXSERVICE_MBXERRORCMD			0x01

///////////////////////////////////////////////////////////////
//
//	Error Codes for a mailbox error response
//

#define	MBXERR_SYNTAX						0x01
#define	MBXERR_UNSUPPORTEDPROTOCOL		0x02
#define	MBXERR_INVALIDCHANNEL			0x03
#define	MBXERR_SERVICENOTSUPPORTED		0x04
#define	MBXERR_INVALIDHEADER				0x05
#define	MBXERR_SIZETOOSHORT				0x06
#define	MBXERR_NOMOREMEMORY				0x07
#define	MBXERR_INVALIDSIZE				0x08

/*---------------------------------------------------------------------------------
------				 
------	Data Types
------				 
---------------------------------------------------------------------------------*/

#endif //_MAILBOX_H_

/*-----------------------------------------------------------------------------------------
------	
------	global variables
------	
-----------------------------------------------------------------------------------------*/

#ifndef _MAILBOX_
#define _MAILBOX_   0
#endif

#if _MAILBOX_
	#define PROTO 
#else 
	#define PROTO extern
#endif

PROTO	BOOL						bReceiveMbxIsLocked;
PROTO	BOOL						bSendMbxIsFull;
PROTO	BOOL						bMbxRunning;
PROTO	BOOL						bMbxRepeatToggle;
PROTO	UINT16 					u16SendMbxSize;
PROTO	UINT16 					u16ReceiveMbxSize;
PROTO	UINT16 					u16EscAddrReceiveMbx;
PROTO	UINT16 					u16EscAddrSendMbx;
PROTO	UINT8						u8MbxWriteCounter;
PROTO	UINT8						u8MbxReadCounter;
PROTO	TMBX MBXMEM				asMbx[2];
PROTO	UINT8						u8MailboxSendReqStored;
PROTO	TMBX MBXMEM *			psWriteMbx;
PROTO	TMBX MBXMEM *			psReadMbx;
PROTO	TMBX MBXMEM *			psRepeatMbx;
PROTO	TMBX MBXMEM *			psStoreMbx;

/*-----------------------------------------------------------------------------------------
------  
------	global functions
------	
-----------------------------------------------------------------------------------------*/

PROTO	void 	MBX_Init(void);
PROTO	UINT8	MBX_StartMailboxHandler(void);
PROTO	void 	MBX_StopMailboxHandler(void);
PROTO	void 	MBX_MailboxWriteInd(TMBX MBXMEM *pMbx);
PROTO	void 	MBX_MailboxReadInd(void);
PROTO	void 	MBX_MailboxRepeatReq(void);
PROTO	UINT8	MBX_MailboxSendReq(TMBX MBXMEM * pMbx, UINT8 flags);
PROTO	void	MBX_Main(void);

#undef PROTO

