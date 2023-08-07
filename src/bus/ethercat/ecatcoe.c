/** 
\defgroup ecatcoe ecatcoe.c: CoE (CANopen over EtherCAT) functions
\brief This file contains the CoE mailbox interface\n
\brief Changes to version V3.20:\n

\author  Holger Buettner
\version 4.00
\date    23.03.2007
*/

//---------------------------------------------------------------------------------------
/**
\ingroup ecatcoe	
\file ecatcoe.c
\brief Implementation.
*/
//---------------------------------------------------------------------------------------

/*---------------------------------------------------------------------------------------
------	
------	Includes
------ 
---------------------------------------------------------------------------------------*/

#include <string.h>

#include "ecat_def.h"

#if COE_SUPPORTED

#define	_ECATCOE_	1
#include "ecatcoe.h"
#undef  	_ECATCOE_
#define	_ECATCOE_	0

#include "sdoserv.h"

/*---------------------------------------------------------------------------------------
------	
------	internal Types and Defines
------	
---------------------------------------------------------------------------------------*/

#define	ECATCOE		0x4300
#define	ECATCOEMAX	0x02

/*---------------------------------------------------------------------------------------
------	
------	static variables 
------	
---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
------  
------	static functions 
------	
---------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------
------  
------	functions
------	
---------------------------------------------------------------------------------------*/

/**
\addtogroup ecatcoe
@{
*/

/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief	This function intializes the CoE Interface.
*////////////////////////////////////////////////////////////////////////////////////////

void COE_Init(void)
{
	pCoeSendStored = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pReceiveMbx	  Pointer to the received mailbox data from the master.

 \return	result of the operation (0 (success) or mailbox error code (MBXERR_.... defined in
			mailbox.h))

 \brief	This function is called when a CoE (CANopen over EtherCAT) service is received from 
 			the master.
*////////////////////////////////////////////////////////////////////////////////////////

UINT8 COE_ServiceInd(TCOEMBX MBXMEM *pCoeMbx)
{
	UINT8 result = 0;
	
	switch ((pCoeMbx->CoeHeader.b[COEHEADER_COESERVICEOFFSET] & COEHEADER_COESERVICEMASK) >> COEHEADER_COESERVICESHIFT)
	{
	case COESERVICE_SDOREQUEST:
		/* SDO-Request received, call SDOS_SdoInd to process the SDO-Request
		   if an existing SDO-Stack shall be used, the corresponding fucntion
			should be called */
		result = SDOS_SdoInd( (TINITSDOMBX MBXMEM *) pCoeMbx );
		break;

	case COESERVICE_SDOINFO:
		/* SDO-Information Request received, call SDOS_SdoInfoInd to process the SDO-Request */
		result = SDOS_SdoInfoInd( (TSDOINFORMATION MBXMEM *) pCoeMbx );
		break;

	case COESERVICE_EMERGENCY:		
	case COESERVICE_SDORESPONSE:
	case COESERVICE_TXPDO:			
	case COESERVICE_RXPDO:			
	case COESERVICE_TXPDOREMREQ:
	case COESERVICE_RXPDOREMREQ:
		/* these CoE services are not supported yet */
		result = MBXERR_SERVICENOTSUPPORTED;
		break;

	default:
		result = MBXERR_INVALIDHEADER;
		break;
	}		

	return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param 	pMbx	  Pointer to the free mailbox to sent.

 \brief	This function is called when a CoE service to be sent is stored and can
 \brief  be put in the send mailbox.
*////////////////////////////////////////////////////////////////////////////////////////

void COE_ContinueInd(TMBX MBXMEM * pMbx)
{
	if (pCoeSendStored)
	{
		/* send the stored CoE service which could not be sent before */
		MBX_MailboxSendReq(pCoeSendStored, 0);
		pCoeSendStored = 0;
	}
	else
	{
		/* send the next fragment of the last CoE service (only for SDO-Information possible) */
		/* copy the stored SDO-Info-Header in the request */
		MBXMEMCPY(pMbx, aSdoInfoHeader, SIZEOF(aSdoInfoHeader));
		/* call SDOS_SdoInfoInd to generate and send the next fragment */
		SDOS_SdoInfoInd( (TSDOINFORMATION MBXMEM *) pMbx );
	}
}

/** @} */

#endif /* COE_SUPPORTED */

