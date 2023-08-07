/*-----------------------------------------------------------------------------------------
------
------	ecatcoe.h
------			 																																					------
-----------------------------------------------------------------------------------------*/

#ifndef _ECATCOE_H_

#define _ECATCOE_H_

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

#define	ERROR_COEINVALIDSERVICE		0x01
#define	ERROR_COENOTSUPPORTED		0x02

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// COE services
*/

#define	COESERVICE_EMERGENCY			0x01
#define	COESERVICE_SDOREQUEST		0x02
#define	COESERVICE_SDORESPONSE		0x03
#define	COESERVICE_TXPDO				0x04
#define	COESERVICE_RXPDO				0x05
#define	COESERVICE_TXPDOREMREQ		0x06
#define	COESERVICE_RXPDOREMREQ		0x07
#define	COESERVICE_SDOINFO			0x08

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// COE Structures
*/

typedef struct STRUCT_PACKED
{
  UMBXHEADER        MbxHeader;
  TCOEHEADER        CoeHeader;
  UINT16            Data[(MAX_MBX_DATA_SIZE-SIZEOF(TCOEHEADER)) >> 1];
} TCOEMBX;

/*-----------------------------------------------------------------------------------------
------	
------	global variables
------	
-----------------------------------------------------------------------------------------*/

#ifndef _ECATCOE_
#define _ECATCOE_   0
#endif

#if _ECATCOE_
	#define PROTO
#else 
	#define PROTO extern
#endif

PROTO	TMBX MBXMEM *pCoeSendStored;				/* if the mailbox service could not be sent (or stored), 
																the CoE service will be stored in this variable 
																and will be sent automatically from the mailbox handler 
																(COE_ContinueInd) when the send mailbox will be read
																the next time from the master */

/*-----------------------------------------------------------------------------------------
------  
------	global functions
------	
-----------------------------------------------------------------------------------------*/

PROTO	void 	COE_Init(void);
PROTO	UINT8 COE_ServiceInd(TCOEMBX MBXMEM *pCoeMbx);
PROTO	void 	COE_ContinueInd(TMBX MBXMEM * pMbx);

#undef PROTO

#endif //_ECATCOE_H_
