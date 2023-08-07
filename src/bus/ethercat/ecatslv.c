/** 
\defgroup ecatslv ecatslv.c: EtherCAT Slave State Machine
\brief This file contains the EtherCAT State Machine\n
\brief Changes to version V4.00:\n
\brief V4.01 ECAT 1: The Output sync Manager was not disabled when the state OP was left
\brief               by a local request (watchdog or io error)\n
\brief V4.01 ECAT 2: APPL_StopOutputHandler returns an UINT16\n
\brief V4.01 ECAT 3: TwinCAT compatibility mode: The state transition to OP is allowed when the
\brief 					WD-Trigger-Bit of the SM2-Control-Byte (0x814.6) is FALSE, in that case the
\brief 					watchdog will not be started before the outputs were received the first time\n
\brief V4.01 ECAT 4: "else" was too much\n
\brief Changes to version V3.20:\n
\brief V4.00 ECAT 1: The handling of the Sync Manager Parameter was included according to
\brief               the EtherCAT Guidelines and Protocol Enhancements Specification\n
\brief V4.00 ECAT 2: The output sync manager is initialized during the state transition
\brief               from PREOP to SAFEOP that the master can check if the slave could update
\brief               inputs and outputs before switching the slave to OP\n
\brief               behaviour according to the EtherCAT Guidelines and Protocol Enhancements Specification\n
\brief V4.00 ECAT 3: The watchdog will be enabled in SAFE-OP that it can be checked if the last SM event
\brief               was received during the watchdog time before switching to OP\n
\brief V4.00 ECAT 4: The function CheckSmChannelParameters is included in the function 
\brief               CheckSmSettings to get a better overview\n
\brief V4.00 ECAT 5: In synchronous mode the slave should support 1- and 3-buffer mode, 3-buffer mode
\brief               should be the standard setting, because the controlling if the process data was updated
\brief               should be done with the TxPDO Toggle, but the 1-buffer mode should be settable too,
\brief               that the master could easily check if all slaves are synchronous by checking the 
\brief               the working counter (if the outputs were not read or the inputs were not written
\brief               the ESC of the slave would not increment the working counter with expected value
\brief               if the 1-buffer mode is running)\n
\brief V4.00 ECAT 6: The function ECAT_StateChange was added, which the application should call if a local error
\brief					is detected (with the parameters alStatus = STATE_SAFEOP, alStatusCode = error code (> 0x1000))
\brief					or gone (with the parameters alStatus = STATE_OP, alStatusCode = 0)
\brief					or if one of the functions APPL_StartMailboxHandler, APPL_StopMailboxHandler, APPL_StartInputHandler,
\brief					APPL_StopInputHandler, APPL_StartOutputHandler, APPL_StopOutputHandler has returned NOERROR_INWORK
\brief					to acknowledge the last state transition (with the parameters alStatus = new AL-Status, alStatusCode =
\brief					new AL-Status-Code)\n
\brief V4.00 ECAT 7: The return values for the AL-StatusCode were changed to UINT16\n

\author  Holger Buettner
\version 4.01
\date    26.04.2007
*/

//---------------------------------------------------------------------------------------
/**
\ingroup ecatslv	
\file ecatslv.c
\brief Implementation.
*/
//---------------------------------------------------------------------------------------

// Avoids warning C183: dead assignment eliminated

//#pragma warning disable = 183

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

#include "common\CommonDefines.h"

#define	_ECATSLV_	1
#include "ecatslv.h"
#undef	_ECATSLV_
#define	_ECATSLV_	0

#include "mailbox.h"
#if COE_SUPPORTED
#include "ecatcoe.h"
#endif
#if BOOTSTRAPMODE_SUPPORTED
#include	"ecbtl18f.h"
#endif // BOOTSTRAPMODE_SUPPORTED
#include "emcy.h"
#include "ecatappl.h"
#include "coeappl.h"
#include "objdef.h"


/*--------------------------------------------------------------------------------------
------	
------	local Types and Defines
------	
--------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------
------  
------	local variables and constants
------	
-----------------------------------------------------------------------------------------*/

static BOOL bWdtExpiredNotify=FALSE;

/*-----------------------------------------------------------------------------------------
------  
------	local functions
------	
-----------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------
------  
------	functions
------	
-----------------------------------------------------------------------------------------*/

/**
\addtogroup ecatslv
@{
*/


#if COE_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param  channel		SM channel
 \param  code			faulty SM channel word
 \param	size			size of the SM channel

 \brief	This function sends an emregency when SM channel check failed

*////////////////////////////////////////////////////////////////////////////////////////

static void SendSmFailedEmergency(UINT8 channel, UINT8 code)
{
	TEMCYMESSAGE EMCYMEM *	pEmcy	= EMCY_GetEmcyBuffer();

	if ( pEmcy )
	{
		/* Emergency buffer is available, the code gives the information about the error reason
		   and has to be decremented to match the following definitions:
			 8: Sync Manager 2 does not support an odd address
			 9: Address of Sync Manager 2 is not supported
			10: Size of Sync Manager 2 is not supported
			11: Settings of Sync Manager 2 are not supported
			12: Sync Manager 3 does not support an odd address
			13: Address of Sync Manager 3 is not supported
			14: Size of Sync Manager 3 is not supported
			15: Settings of Sync Manager 3 are not supported
			16: Sync Manager 4 does not support an odd address
			17: Address of Sync Manager 4 is not supported
			18: Size of Sync Manager 4 is not supported
			19: Settings of Sync Manager 4 are not supported
			20: Sync Manager 5 does not support an odd address
			21: Address of Sync Manager 5 is not supported
			22: Size of Sync Manager 5 is not supported
			23: Settings of Sync Manager 5 are not supported
			24: Sync Manager 6 does not support an odd address
			25: Address of Sync Manager 6 is not supported
			26: Size of Sync Manager 6 is not supported
			27: Settings of Sync Manager 6 are not supported
			28: Sync Manager 7 does not support an odd address
			29: Address of Sync Manager 7 is not supported
			30: Size of Sync Manager 7 is not supported
			31: Settings of Sync Manager 7 are not supported
		*/
		code--;				 
		/* the code is stored in byte 3 of the Emergency */
#if BYTE_NOT_SUPPORTED
		pEmcy->Emcy.Byte[EMCY_OFFS_DIAGCODE] &= ~EMCY_MASK_DIAGCODE;
		pEmcy->Emcy.Byte[EMCY_OFFS_DIAGCODE] |= ERROR_SYNCMANCH(code, channel) << EMCY_SHIFT_DIAGCODE;
#else
		pEmcy->Emcy.Byte[EMCY_OFFS_DIAGCODE] = (UINT8)ERROR_SYNCMANCH(code, channel);
#endif
		/* the correct settings are stored in byte 4-7 of the Emergency */
		switch (channel)
		{
		case PROCESS_DATA_OUT:
			switch (code)
			{
			case SYNCMANCHODDADDRESS:
			case SYNCMANCHADDRESS:
				/* store the minimum output address in byte 4,5 */
				pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA] = SWAPWORD(MIN_PD_WRITE_ADDRESS);
				/* store the maximum output address in byte 6,7 */
				pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA+1] = SWAPWORD(MAX_PD_WRITE_ADDRESS);
				break;
			case SYNCMANCHSIZE:		
				/* store the minimum output size in byte 4,5 */
				pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA] = SWAPWORD(nPdOutputSize);
				/* store the maximum output size in byte 6,7 */
				pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA+1] = SWAPWORD(nPdOutputSize);
				break;
			case SYNCMANCHSETTINGS:
				/* store the correct settings for the output sync manager in byte 4-7 */
				if (nPdOutputSize)
				{
					pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA] = SWAPWORD(PD_BUFFER_TYPE | SM_WRITESETTINGS);
					pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA+1] = SWAPWORD(SM_ECATENABLE);
				}
				else
				{
					pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA] = 0;
					pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA+1] = 0;
				}
				break;
			}
			break;
		case PROCESS_DATA_IN:
			switch (code)
			{
			case SYNCMANCHODDADDRESS:
			case SYNCMANCHADDRESS:
				/* store the minimum input address in byte 4,5 */
				pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA]   = SWAPWORD(MIN_PD_READ_ADDRESS);
				/* store the maximum input address in byte 6,7 */
				pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA+1] = SWAPWORD(MAX_PD_READ_ADDRESS);
				break;
			case SYNCMANCHSIZE:		
				/* store the minimum input size in byte 4,5 */
				pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA]   = SWAPWORD(nPdInputSize);
				/* store the maximum input size in byte 6,7 */
				pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA+1] = SWAPWORD(nPdInputSize);
				break;
			case SYNCMANCHSETTINGS:
				/* store the correct settings for the input sync manager in byte 4-7 */
				if (nPdInputSize)
				{
					pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA]   = SWAPWORD(PD_BUFFER_TYPE | SM_READSETTINGS);
					pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA+1] = SWAPWORD(SM_ECATENABLE);
				}
				else
				{
					pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA]   = 0;
					pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA+1] = 0;
				}
				break;
			}
			break;
		default:
			pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA]   = 0;
			pEmcy->Emcy.Word[EMCY_OFFS_DIAGDATA+1] = 0;
		}

		/* set the byte 0,1 of the Emergency to the state transition fault */
		pEmcy->Emcy.Word[EMCY_OFFS_ERRORCODE] = SWAPWORD(EMCY_SM_ERRORCODE);
		/* set the byte 2 of the Emergency to the actual state */
#if BYTE_NOT_SUPPORTED
		pEmcy->Emcy.Byte[EMCY_OFFS_ERRORREGISTER] &= ~EMCY_MASK_ERRORREGISTER;
		pEmcy->Emcy.Byte[EMCY_OFFS_ERRORREGISTER] |= nAlStatus << EMCY_SHIFT_DIAGCODE;
#else
		pEmcy->Emcy.Byte[EMCY_OFFS_ERRORREGISTER] = nAlStatus;
#endif
		EMCY_SendEmergency(pEmcy);
	}
}

#endif // COE_SUPPORTED

/* ECATCHANGE_START(V4.00) ECAT 4 */
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param  maxChannel	last SM channel which should be checked

 \return 				0: okay else AL Status Code

 \brief	This function checks all SM channels

*////////////////////////////////////////////////////////////////////////////////////////

static UINT8 CheckSmSettings(UINT8 maxChannel)
{
	UINT8 i;
	UINT8 result = 0;
	UINT8 smFailed = 0;
	TSYNCMAN ESCMEM *pSyncMan;

	/* check the Sync Manager Parameter for the Receive Mailbox (Sync Manager Channel 0) */
	pSyncMan = HW_GetSyncMan(MAILBOX_WRITE);
	if ( (pSyncMan->Settings.Byte[ESC_OFFS_ECATENABLE] & SM_ECATENABLE) != SM_ECATENABLE )	
		/* receive mailbox is not enabled */
		result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
	else if ( (pSyncMan->Settings.Byte[ESC_OFFS_SMCTRL] & SM_INITMASK) != (ONE_BUFFER | SM_WRITESETTINGS) )
		/* receive mailbox is not writable by the master or not in one buffer mode */
		result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
	else if ( pSyncMan->Length < MIN_MBX_SIZE )
		/* receive mailbox size is too small */
		result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
	else if ( pSyncMan->Length > MAX_MBX_SIZE )
		/* receive mailbox size is too great */
		result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
 	else if ( pSyncMan->PhysicalStartAddress < MIN_MBX_WRITE_ADDRESS )
		/* receive mailbox address is too small */
		result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
	else if ( pSyncMan->PhysicalStartAddress > MAX_MBX_WRITE_ADDRESS )
		/* receive mailbox address is too great */
		result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
	else if ( (pSyncMan->PhysicalStartAddress & 0x0001) != 0 )	
		/* receive mailbox address is not even */
		result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;

	if ( result == 0 )
	{
		/* check the Sync Manager Parameter for the Send Mailbox (Sync Manager Channel 1) */
		pSyncMan = HW_GetSyncMan(MAILBOX_READ);
		if ( (pSyncMan->Settings.Byte[ESC_OFFS_ECATENABLE] & SM_ECATENABLE) != SM_ECATENABLE )	
			/* send mailbox is not enabled */
			result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
		else if ( (pSyncMan->Settings.Byte[ESC_OFFS_SMCTRL] & SM_INITMASK) != (ONE_BUFFER | SM_READSETTINGS) )
			/* send mailbox is not writable by the master or not in one buffer mode */
			result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
		else if ( pSyncMan->Length < MIN_MBX_SIZE )
			/* send mailbox size is too small */
			result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
		else if ( pSyncMan->Length > MAX_MBX_SIZE )
			/* send mailbox size is too great */
			result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
 		else if ( pSyncMan->PhysicalStartAddress < MIN_MBX_READ_ADDRESS )
			/* send mailbox address is too small */
			result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
		else if ( pSyncMan->PhysicalStartAddress > MAX_MBX_READ_ADDRESS )
			/* send mailbox address is too great */
			result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
		else if ( (pSyncMan->PhysicalStartAddress & 0x0001) != 0 )	
			/* send mailbox address is not even	*/
			result = ALSTATUSCODE_INVALIDMBXCFGINPREOP;

		if (result != 0)
			smFailed = MAILBOX_READ;
	}
	else
		smFailed = MAILBOX_WRITE;

	if ( result == 0 && maxChannel > PROCESS_DATA_IN )
	{
/* ECATCHANGE_START(V4.00) ECAT 5 */
		/* b3BufferMode is only set, if inputs and outputs are running in 3-Buffer-Mode when leaving this function */
		b3BufferMode = TRUE;
/* ECATCHANGE_END(V4.00) ECAT 5 */
		/* check the Sync Manager Parameter for the Inputs (Sync Manager Channel 3) */
		pSyncMan = HW_GetSyncMan(PROCESS_DATA_IN);

		if ( (pSyncMan->Settings.Byte[ESC_OFFS_ECATENABLE] & SM_ECATENABLE) != 0 && pSyncMan->Length == 0 )
			/* the SM3 size is 0 and the SM3 is not active */
			result = SYNCMANCHSETTINGS+1;
		else if ( (pSyncMan->Settings.Byte[ESC_OFFS_ECATENABLE] & SM_ECATENABLE) == SM_ECATENABLE )
		{
			/* Sync Manager Channel 3 is active, input size has to greater 0 */
			if ( pSyncMan->Length != nPdInputSize || nPdInputSize == 0 )
				/* sizes don't match */
				result = SYNCMANCHSIZE+1;
			else
				/* sizes matches */
			if ( (pSyncMan->Settings.Byte[ESC_OFFS_SMCTRL] & SM_PDINITMASK) == SM_READSETTINGS )
			{
				/* settings match */
				if ( ( ( nAlStatus == STATE_PREOP )&&( pSyncMan->PhysicalStartAddress >= MIN_PD_READ_ADDRESS )&&( pSyncMan->PhysicalStartAddress <= MAX_PD_READ_ADDRESS ) )
				   ||( ( nAlStatus != STATE_PREOP )&&( pSyncMan->PhysicalStartAddress == nEscAddrInputData ) )
					) 
				{
					/* addresses match */
/* ECATCHANGE_START(V4.00) ECAT 5 */
					if ( (pSyncMan->Settings.Byte[ESC_OFFS_SMCTRL] & SM_INITMASK) == (ONE_BUFFER | SM_READSETTINGS) )
						/* inputs are running in 1-Buffer-Mode, reset flag b3BufferMode */
						b3BufferMode = FALSE;
/* ECATCHANGE_END(V4.00) ECAT 5 */
				}
				else
					/* input address is out of the allowed area or has changed in SAFEOP or OP */
					result = SYNCMANCHADDRESS+1;
			}
			else
				/* input settings do not match */
				result = SYNCMANCHSETTINGS+1;
		}
		else if ( pSyncMan->Length != 0 || nPdInputSize != 0 )
			/* input size is not zero although the SM3 channel is not enabled */
			result = SYNCMANCHSIZE+1;

		if ( result != 0 )
		{
#if COE_SUPPORTED || SOE_SUPPORTED
			/* state transition refused, send an Emergency with the error and the correct settings */
			SendSmFailedEmergency(PROCESS_DATA_IN, result);
#endif
			result = ALSTATUSCODE_INVALIDSMINCFG;
			smFailed = PROCESS_DATA_IN;
		}
	}
/* ECATCHANGE_START(V4.01) ECAT 3 */
//	else 
/* ECATCHANGE_END(V4.01) ECAT 4 */
		
	if ( result == 0 && maxChannel > PROCESS_DATA_OUT )
	{
		/* check the Sync Manager Parameter for the Outputs (Sync Manager Channel 2) */
		pSyncMan = HW_GetSyncMan(PROCESS_DATA_OUT);

		if ( (pSyncMan->Settings.Byte[ESC_OFFS_ECATENABLE] & SM_ECATENABLE) != 0 && pSyncMan->Length == 0 )
			/* the SM2 size is 0 and the SM2 is not active */
			result = SYNCMANCHSETTINGS+1;
		else if ( (pSyncMan->Settings.Byte[ESC_OFFS_ECATENABLE] & SM_ECATENABLE) == SM_ECATENABLE )
		{
			/* Sync Manager Channel 2 is active, output size has to greater 0 */
			if ( pSyncMan->Length == nPdOutputSize && nPdOutputSize != 0 )
			{
				/* sizes match */
				if ( (pSyncMan->Settings.Byte[ESC_OFFS_SMCTRL] & SM_PDINITMASK) == SM_WRITESETTINGS )
				{
					/* settings match */
					if ( ( ( nAlStatus == STATE_PREOP )&&( pSyncMan->PhysicalStartAddress >= MIN_PD_WRITE_ADDRESS )&&( pSyncMan->PhysicalStartAddress <= MAX_PD_WRITE_ADDRESS ) )
					   ||( ( nAlStatus != STATE_PREOP )&&( pSyncMan->PhysicalStartAddress == nEscAddrOutputData ) )
						) 
					{
						/* addresses match */
						{
							/* check, if watchdog trigger is enabled */
							if (pSyncMan->Settings.Byte[ESC_OFFS_SMCTRL] & WATCHDOG_TRIGGER)
								bWdTrigger = TRUE;
							else
								bWdTrigger = FALSE;
/* ECATCHANGE_START(V4.00) ECAT 5 */
							if ( (pSyncMan->Settings.Byte[ESC_OFFS_SMCTRL] & SM_INITMASK) == (ONE_BUFFER | SM_WRITESETTINGS) )
								/* outputs are running in 1-Buffer-Mode, reset flag b3BufferMode */
								b3BufferMode = FALSE;
/* ECATCHANGE_END(V4.00) ECAT 5 */
						}
					}
					else
						/* output address is out of the allowed area or has changed in SAFEOP or OP */
						result = SYNCMANCHADDRESS+1;
				}
				else
					/* output settings do not match */
					result = SYNCMANCHSETTINGS+1;
			}
			else
				/* output sizes don't match */
				result = SYNCMANCHSIZE+1;
		}
		else if ( pSyncMan->Length != 0 || nPdOutputSize != 0 )
			/* output size is not zero although the SM2 channel is not enabled */
			result = SYNCMANCHSIZE+1;

		if ( result != 0 )
		{
#if COE_SUPPORTED || SOE_SUPPORTED
			/* state transition refused, send an Emergency with the error and the correct settings */
			SendSmFailedEmergency(PROCESS_DATA_OUT, result);
#endif
			result = ALSTATUSCODE_INVALIDSMOUTCFG;
			smFailed = PROCESS_DATA_OUT;
		}
	}

	if ( result == 0 )
	{
		/* the Enable-Byte of the rest of the SM channels has to be read to acknowledge the SM-Change-Interrupt */
		for (i = maxChannel; i < nMaxSyncMan; i++)
		{
			pSyncMan = HW_GetSyncMan(i);
		}
	}

	return result;
}
/* ECATCHANGE_END(V4.00) ECAT 4 */

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief	This function is called in case of the state transition from PREOP to SAFEOP.
 |brief  the areas of the Sync Managers will be checked for overlapping,
 \brief  the synchronization mode (Free Run, Synchron, Distributed Clocks) is selected,
 \brief  the requested cycle time will be checked, the watchdog is started 
 \brief  and the AL Event Mask register will be set

*////////////////////////////////////////////////////////////////////////////////////////

/* ECATCHANGE_START(V4.00) ECAT 7 */
static UINT16 StartInputHandler(void)
/* ECATCHANGE_END(V4.00) ECAT 7 */
{
/* ECATCHANGE_START(V4.00) ECAT 7 */
	UINT16 result;
/* ECATCHANGE_END(V4.00) ECAT 7 */
	TSYNCMAN ESCMEM * pSyncMan;
#if AL_EVENT_ENABLED
#if COE_SUPPORTED
	UINT32 	cycleTime;
#endif // COE_SUPPORTED
#if DC_SUPPORTED
	UINT8		dcControl;
#endif // DC_SUPPORTED
#endif // AL_EVENT_ENABLED
/* ECATCHANGE_START(V4.00) ECAT 3 */
	UINT16 	wdiv;
	UINT16 	wd;
/* ECATCHANGE_END(V4.00) ECAT 3 */
/* ECATCHANGE_START(V4.00) ECAT 5 */
	UINT16	nPdInputBuffer = 3;
	UINT16	nPdOutputBuffer = 3;
/* ECATCHANGE_END(V4.00) ECAT 5 */
	UINT16 	intMask = 0;

	/* get a pointer to the Sync Manager Channel 2 (Outputs) */
	pSyncMan = HW_GetSyncMan(PROCESS_DATA_OUT);
	/* store the address of the Sync Manager Channel 2 (Outputs) */
	nEscAddrOutputData = pSyncMan->PhysicalStartAddress;
	/* get a pointer to the Sync Manager Channel 3 (Inputs) */
	pSyncMan = HW_GetSyncMan(PROCESS_DATA_IN);
	/* store the address of the Sync Manager Channel 3 (Inputs) */
	nEscAddrInputData = pSyncMan->PhysicalStartAddress;
/* ECATCHANGE_START(V4.00) ECAT 5 */
	/* get the number of input buffer used for calculating the address areas */
	if ( pSyncMan->Settings.Byte[ESC_OFFS_SMCTRL] & ONE_BUFFER )
		nPdInputBuffer = 1;
/* ECATCHANGE_END(V4.00) ECAT 5 */

	/* it has be checked if the Sync Manager memory areas for Inputs and Outputs will not overlap 
	   the Sync Manager memory areas for the Mailbox */
/* ECATCHANGE_START(V4.00) ECAT 5 */
	if ( ((nEscAddrInputData+nPdInputSize*nPdInputBuffer) > u16EscAddrSendMbx && (nEscAddrInputData < (u16EscAddrSendMbx+u16SendMbxSize)))
		||((nEscAddrInputData+nPdInputSize*nPdInputBuffer) > u16EscAddrReceiveMbx && (nEscAddrInputData < (u16EscAddrReceiveMbx+u16ReceiveMbxSize)))
		)
/* ECATCHANGE_END(V4.00) ECAT 5 */
	{
		/* Sync Manager Channel 3 memory area (Inputs) overlaps the Sync Manager memory areas for the Mailbox */
		SendSmFailedEmergency(PROCESS_DATA_IN, SYNCMANCHADDRESS+1);
		return ALSTATUSCODE_INVALIDSMINCFG;
	}

/* ECATCHANGE_START(V4.00) ECAT 5 */
	if ( ((nEscAddrOutputData+nPdOutputSize*nPdOutputBuffer) > u16EscAddrSendMbx && (nEscAddrOutputData < (u16EscAddrSendMbx+u16SendMbxSize)))
		||((nEscAddrOutputData+nPdOutputSize*nPdOutputBuffer) > u16EscAddrReceiveMbx && (nEscAddrOutputData < (u16EscAddrReceiveMbx+u16ReceiveMbxSize)))
		||((nEscAddrOutputData+nPdOutputSize*nPdOutputBuffer) > nEscAddrInputData && (nEscAddrOutputData < (nEscAddrInputData+nPdInputSize)))
		)
/* ECATCHANGE_END(V4.00) ECAT 5 */
	{
		/* Sync Manager Channel 2 memory area (Outputs) overlaps the Sync Manager memory areas for the Mailbox 
		   or the Sync Manager Channel 3 memory area (Inputs) */
		SendSmFailedEmergency(PROCESS_DATA_OUT, SYNCMANCHADDRESS+1);
		return ALSTATUSCODE_INVALIDSMOUTCFG;
	}

#if AL_EVENT_ENABLED
#if DC_SUPPORTED
	/* check the DC-Registers */
/* ECATCHANGE_START(V4.00) ECAT 1 */
	HW_EscReadAccess(&dcControl, ESC_ADDR_SYNCCONTROL+1, 1);
	
	/* check if Distruibuted Clocks are enabled */
	if ( dcControl & (DC_SYNC0_ACTIVE | DC_SYNC1_ACTIVE) )
	{
		/* Distributed Clocks are enabled, check the correct SYNC0/SYNC1-setting,
		   DC_SYNC_ACTIVE is set per default to SYNC0 (in ecatslv.h)
		   but this can be overwritten in ecat_def.h */
		if ( dcControl != (DC_CYCLIC_ACTIVE | DC_SYNC_ACTIVE) )
			return ALSTATUSCODE_DCINVALIDSYNCCFG;

		/* DC mask for the AL-Event-Mask-Register (0x204)
		   DC_EVENT_MASK is set by default to PROCESS_OUTPUT_EVENT or SYNC0_EVENT
		   (optimized DC Cycle in the Synchronization document)
		   but this can be overwritten in ecat_def.h */
		intMask = DC_EVENT_MASK;

		/* slave is running in DC-mode */
		bDcSyncActive = TRUE;
		HW_EscReadAccess((UINT8 *) &cycleTime, ESC_ADDR_SYNC_CYCLETIME, 4);
	}
	else 
#endif	/* DC_SUPPORTED */
	{
#if COE_SUPPORTED
		/* check entry 0x1C32:01, if free run or synchron synchronization mode is requested */
		if ( sSyncManOutPar.u16SyncType != SYNCTYPE_FREERUN )
		{
			/* ECAT Synchron Mode, the ESC interrupt is enabled */
			bEscIntEnabled = TRUE;
		}
		else
		{
			/* ECAT FreeRun Mode, sync manager has to run in 3-Buffer Mode */
			if ( !b3BufferMode )
				/* 1-Buffer-Mode, refuse the state transition */
				return ALSTATUSCODE_FREERUNNEEDS3BUFFERMODE;
		}
		/* get the requested cycle time from entry 0x1C32:02 */
		cycleTime = sSyncManOutPar.u32CycleTime;
#endif // COE_SUPPORTED
	}

	/* if ECAT Synchron Mode or FREERUN is active and the output size is zero, 
	   the ESC interrupt will be generated by the PROCESS_INPUT_EVENT */
	if ( !bDcSyncActive && nPdOutputSize == 0 )
	{
		/* Sync Manager Channel 3 event has to activated in the AL-Event mask register */
		intMask |= PROCESS_INPUT_EVENT; 
	}

#if COE_SUPPORTED
	/* check if the requested cycle time exceeds the minimum and maximum values of the cycle time 
	   (MIN_PDCYCLE_TIME and MAX_PD_CYCLE_TIME are defined in ecat_def.h) and not multiple of */
	if ( cycleTime != 0 && (cycleTime < MIN_PD_CYCLE_TIME || cycleTime > MAX_PD_CYCLE_TIME) && \
		cycleTime != (UINT32)(cycleTime/GRANULARITY_PD_CYCLE_TIME)*GRANULARITY_PD_CYCLE_TIME )
		return ALSTATUSCODE_DCINVALIDSYNCCYCLETIME;						

	/* reset the error counter indicating synchronization problems */
	sCycleDiag.smEventMissedCounter = 0;
	sCycleDiag.shiftTooShortCounter = 0;
	sCycleDiag.cycleExceededCounter = 0;
	sCycleDiag.syncFailedCounter = 0;

#endif // COE_SUPPORTED
/* ECATCHANGE_END(V4.00) ECAT 1 */
#endif // AL_EVENT_ENABLED
/* ECATCHANGE_START(V4.00) ECAT 3 */
	/* initialize the watchdog, get watchdog divider (register 0x400) first */
	HW_EscReadAccess((UINT8 *) &wdiv, ESC_WATCHDOG_DIVIDER, 2);
	if ( wdiv != 0 )
	{
		/* watchdog is activated, get watchdog value */
		HW_EscReadAccess((UINT8 *) &wd, ESC_SM_WATCHDOG, 2);
		if ( wd != 0 )
		{
			/* store watchdog in ms in variable u16WdValue */
			UINT32 d = wd;
			d *= wdiv;
			d /= 25000;
			u16WdValue = (UINT16) d;
		}
		/* reset watchdog counter */
		u16WdCounter = 0;
	}
	else
	{
		/* the watchdog is deactivated */
		wd = 0;
		u16WdValue = 0;
		if ( bWdTrigger )
		{
			/* if the WD-Trigger in the Sync Manager Channel 2 Control-Byte is set (Bit 6 of Register 0x814)
			   an error has to be returned */
			return ALSTATUSCODE_INVALIDWDCFG;
		}
	}
/* ECATCHANGE_END(V4.00) ECAT 3 */

#if AL_EVENT_ENABLED
/* ECATCHANGE_START(V4.00) ECAT 2 */
	if ( !bDcSyncActive && nPdOutputSize != 0 )
	{
		/* ECAT synchron Mode is active, the Sync Manager Channel 2 event 
		   has to activated in the AL-Event mask register */
		intMask |= PROCESS_OUTPUT_EVENT; 
	}
/* ECATCHANGE_END(V4.00) ECAT 2 */
#endif // AL_EVENT_ENABLED

	/* update the input data once before switching to SAFEOP */
	if ( nPdInputSize > 0 )
	{
		/* the SM3-channel has to be enabled (was disabled in StopInputHandler) */
		HW_EnableSyncManChannel(PROCESS_DATA_IN);
		/* first input update has to be done before confirming SAFE-OP */
/* ECATCHANGE_START(V4.00) */
		result = APPL_StartInputHandler(&intMask);
/* ECATCHANGE_END(V4.00) */
		if (result)
			return result;

		bEcatInputUpdateRunning = TRUE;
	}
/* ECATCHANGE_START(V4.00) ECAT 2 */
	if ( nPdOutputSize > 0 )
	{
/* ECATCHANGE_START(V4.00) ECAT 6 */
		if ( !bEcatLocalError )
/* ECATCHANGE_END(V4.00) ECAT 6 */
		/* if we don't have a local error,
			the SM2-channel has to be enabled (was disabled in StopInputHandler), because
		   in the state SAFE-OP and OP the slave shall respond to the reading and writing
		   of process data that the master don't get a working counter mismatch */
		HW_EnableSyncManChannel(PROCESS_DATA_OUT);
	}
/* ECATCHANGE_END(V4.00) ECAT 2 */
#if AL_EVENT_ENABLED

	/* initialize the AL Event Mask register (0x204) */
	HW_SetIntMask( intMask );
#if COE_SUPPORTED

/* ECATCHANGE_START(V4.00) ECAT 1 */
	/* update the entries 0x1C32:01 and 0x1C33:01 */
#if DC_SUPPORTED
	if ( bDcSyncActive )
	{
		/* Distributed Clocks synchronization mode */
		sSyncManOutPar.u16SyncType = SYNCTYPE_DCSYNC0;
		sSyncManInPar.u16SyncType = SYNCTYPE_DCSYNC0;
	}
	else
#endif // DC_SUPPORTED
	if ( bEscIntEnabled )
	{
		/* ECAT Synchron Mode */
		sSyncManOutPar.u16SyncType = SYNCTYPE_SYNCHRON;
		sSyncManInPar.u16SyncType = SYNCTYPE_SM2INT;
	}
	else
	{
		/* ECAT FreeRun Mode */
		sSyncManOutPar.u16SyncType = SYNCTYPE_FREERUN;
		sSyncManInPar.u16SyncType = SYNCTYPE_FREERUN;
		/* enable interrupt from the timer */
		APPL_StartFreeRunTimer();
	}
	sSyncManOutPar.u32CycleTime = cycleTime;
	sSyncManInPar.u32CycleTime = cycleTime;
/* ECATCHANGE_END(V4.00) ECAT 1 */
#endif // COE_SUPPORTED
#endif // AL_EVENT_ENABLED

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief	This function is called in case of the state transition from SAFEOP to OP.
 \brief  It will be checked if outputs had to be received before switching to OP
 \brief  and the state transition would be refused if outputs are missing

*////////////////////////////////////////////////////////////////////////////////////////

/* ECATCHANGE_START(V4.00) ECAT 7 */
static UINT16 StartOutputHandler(void)
/* ECATCHANGE_END(V4.00) ECAT 7 */
{
/* ECATCHANGE_START(V4.00) ECAT 7 */
	UINT16 result = 0;
/* ECATCHANGE_END(V4.00) ECAT 7 */

/* ECATCHANGE_START(V4.00) ECAT 3 */
#if LEGACY_MODE
/* ECATCHANGE_START(V4.01) ECAT 3 */
	if (bWdTrigger)
	/* for the legacy to the older TwinCAT versions, the state transition to OP is allowed
	   when the WD-Trigger-Bit of the SM2-Control-Byte (0x814.6) is FALSE, in that case the watchdog 
	   will not be started before the outputs were received the first time */
/* ECATCHANGE_END(V4.01) ECAT 3 */
#endif // LEGACY_MODE
	{
		/* The outputs had to be received during the watchdog time before the state transition 
		   (the flag bEcatFirstOutputsReceived should be set, if outputs are received (can be checked 
		   with the Sync-Manager Channel 2 event)).
		   If no outputs are transmitted (variable nPdOutputSize is 0), the reading of the inputs
		   will be checked (the flag bEcatFirstOutputsReceived should be set, if the inputs were read
		   (can be checked with Sync Manager Channel 3 event)) */
		if (!bEcatFirstOutputsReceived)
		{
			/* outputs were not received or inputs were not read, refuse the state transition */
			return ALSTATUSCODE_SMWATCHDOG;
		}
	}
/* ECATCHANGE_END(V4.00) ECAT 3 */

	if (nPdOutputSize > 0)
	{
		/* all standard checkings were done, an application specific reaction to the state transition
		   from SAFEOP to OP can be done in APPL_StartOutputHandler */
		result = APPL_StartOutputHandler();
/* ECATCHANGE_START(V4.00) ECAT 6 */
		if ( bEcatLocalError && (result == 0 || NOERROR_INWORK) )
		{
			/* we enable the SM2, because the local error is gone */
			HW_EnableSyncManChannel(PROCESS_DATA_OUT);
			bEcatLocalError = FALSE;
		}
		if (result != 0)
		{
			if ( result != NOERROR_INWORK )
				bEcatLocalError = TRUE;
			return result;
		}
/* ECATCHANGE_END(V4.00) ECAT 6 */
	}

	/* Slave is OPERATIONAL */
	bEcatOutputUpdateRunning = TRUE;
	
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	 0, NOERROR_INWORK

 \brief	This function is called in case of the state transition from OP to SAFEOP
 \brief  the outputs can be set to an application specific safe state,
 \brief  the state transition can be delayed by returning NOERROR_INWORK

*////////////////////////////////////////////////////////////////////////////////////////

/* ECATCHANGE_START(V4.00) ECAT 7 */
static UINT16 StopOutputHandler(void)
/* ECATCHANGE_END(V4.00) ECAT 7 */
{
/* ECATCHANGE_START(V4.01) ECAT 2 */
	UINT16 result = 0;
/* ECATCHANGE_END(V4.01) ECAT 2 */

/* ECATCHANGE_START(V4.00) ECAT 2 */
	/* reset the flags that outputs were received and that the slave is in OP */
	bEcatFirstOutputsReceived = FALSE;
 	bEcatOutputUpdateRunning = FALSE;
/* ECATCHANGE_END(V4.00) ECAT 2 */

	/* the outputs can be set to an application specific safe state at this place */
	if ( nPdOutputSize > 0 )
		result = APPL_StopOutputHandler();

	return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	 0, NOERROR_INWORK

 \brief	This function is called in case of the state transition from SAFEOP to PREOP

*////////////////////////////////////////////////////////////////////////////////////////

/* ECATCHANGE_START(V4.00) ECAT 7 */
static UINT16 StopInputHandler(void)
/* ECATCHANGE_END(V4.00) ECAT 7 */
{
	UINT8 result = 0;

	/* disable the Sync Manager Channel 2 (outputs) */
	HW_DisableSyncManChannel(PROCESS_DATA_OUT);
	/* reset the events in the AL Event mask register (0x204) */
#if AL_EVENT_ENABLED
	HW_ResetIntMask( ~(UINT16)(SYNC0_EVENT | SYNC1_EVENT | PROCESS_INPUT_EVENT | PROCESS_OUTPUT_EVENT) );
#endif // AL_EVENT_ENABLED
	/* reset the flags */
/* ECATCHANGE_START(V4.00) ECAT 1 */
	bEscIntEnabled = FALSE;
/* ECATCHANGE_END(V4.00) ECAT 1 */
#if DC_SUPPORTED
	bDcSyncActive = FALSE;
#endif
/* ECATCHANGE_START(V4.00) ECAT 3 */
	bWdTrigger = FALSE;
/* ECATCHANGE_END(V4.00) ECAT 3 */
 	bEcatInputUpdateRunning = FALSE;
	/* disable the Sync Manager Channel 3 (inputs) */
	HW_DisableSyncManChannel(PROCESS_DATA_IN);
	/* the application can react to the state transition in the function APPL_StopInputHandler */
	result = (UINT8)APPL_StopInputHandler();

	return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param	alControl		requested new state

 \brief	This function handles the EtherCAT State Machine. It is called 
 			- in case of an AL Control event (Bit 0 of AL-Event (Reg 0x220), 
 			  when the Master has written the AL Control Register (from HW_Main),
			  alControl contains the content of the AL Control (Reg 0x120)
			- in case of a SM-Change event (Bit 4 of AL-Event (Reg 0x220)), 
			  when an Activate SYNCM y register is written by the master (from HW_Main),
			  alControl contains the actual state (Bit 0-3 of AL Status (Reg 0x130))
			- in case of a locally expired watchdog (from HW_Main),
			  alControl contains the requested new state (SAFE_OP)
			- in case of an application specific event to change the EtherCAT state (from application),
			  alControl contains the requested new state (INIT, PRE_OP or SAFE_OP)

*////////////////////////////////////////////////////////////////////////////////////////

void AL_ControlInd(UINT8 alControl)
{
/* ECATCHANGE_START(V4.00) ECAT 7 */
	UINT16		result = 0;
/* ECATCHANGE_END(V4.00) ECAT 7 */
	UINT8 		stateTrans;

	/* reset the Error Flag in case of acknowledge by the Master */
	if ( alControl & STATE_CHANGE )
	{
		nAlStatus &= ~STATE_CHANGE;

/* ECATCHANGE_START(V4.00) ECAT 6 */
		if ( alControl == (STATE_CHANGE | STATE_SAFEOP) ) 
		{
			if ( !bEcatLocalError )
			/* we have to enable the SM2 again, because we have no local error */
			HW_EnableSyncManChannel(PROCESS_DATA_OUT);
		}
/* ECATCHANGE_END(V4.00) ECAT 6 */
	}
	else if ( (nAlStatus & STATE_CHANGE)
	        &&( (alControl & STATE_MASK) > (nAlStatus & STATE_MASK) ) 
	        )
		/* the error flag (Bit 4) is set in the AL-Status and the ErrAck bit (Bit 4)
		   is not set in the AL-Control, so the state cannot be set to a higher state
		   and the new state request will be ignored */
		return;

	/* generate a variable for the state transition 
	  (Bit 0-3: new state (AL Control), Bit 4-7: old state (AL Status) */
	alControl &= STATE_MASK;
	stateTrans = nAlStatus;
	stateTrans <<= 4;
	stateTrans += alControl;

	/* check the SYNCM settings depending on the state transition */
	switch ( stateTrans )
	{
	case INIT_2_PREOP:
	case OP_2_PREOP:
	case SAFEOP_2_PREOP:
	case PREOP_2_PREOP:
		/* in PREOP only the SYNCM settings for SYNCM0 and SYNCM1 (mailbox) 
		   are checked, if result is unequal 0, the slave will stay in or
		   switch to INIT and set the ErrorInd Bit (bit 4) of the AL-Status */
		result = CheckSmSettings(MAILBOX_READ+1);
		break;
	case PREOP_2_SAFEOP:
		/* before checking the SYNCM settings for SYNCM2 and SYNCM3 (process data)
		   the expected length of input data (nPdInputSize) and output data (nPdOutputSize)
			could be adapted (changed by PDO-Assign and/or PDO-Mapping) 
			if result is unequal 0, the slave will stay in PREOP and set
			the ErrorInd Bit (bit 4) of the AL-Status */
		result = APPL_GenerateMapping();
		if (result != 0)
			break;
	case SAFEOP_2_OP:
	case OP_2_SAFEOP:
	case SAFEOP_2_SAFEOP:
	case OP_2_OP:
		/* in SAFEOP or OP the SYNCM settings for all SYNCM are checked 
		   if result is unequal 0, the slave will stay in or
		   switch to PREOP and set the ErrorInd Bit (bit 4) of the AL-Status */
		result = CheckSmSettings(nMaxSyncMan);
		break;
	}
	
	if ( result == 0 )
	{	
		/* execute the corresponding local management service(s) depending on the state transition */
		switch ( stateTrans )
		{
		case INIT_2_BOOT	:
#if BOOTSTRAPMODE_SUPPORTED
			/* if the application has to execute code when going to BOOT this shall be done at this place */
			bBootMode = TRUE;
			/* disable all evnts in BOOT state */
			HW_ResetIntMask(0);
			/* enable the receive mailbox sync manager channel */
			HW_EnableSyncManChannel(MAILBOX_WRITE);
			/* enable the send mailbox sync manager channel */
			HW_EnableSyncManChannel(MAILBOX_READ);
			BL_Start( STATE_BOOT );
#else
			result = ALSTATUSCODE_BOOTNOTSUPP;
#endif
			break;

		case INIT_2_PREOP :
			/* MBX_StartMailboxHandler (in mailbox.c) checks if the areas of the mailbox
			   sync managers SYNCM0 and SYNCM1 overlap each other
		      if result is unequal 0, the slave will stay in INIT 
		      and sets the ErrorInd Bit (bit 4) of the AL-Status */
			result = MBX_StartMailboxHandler(); 
			if (result == 0)
			{
				/* additionally there could be an application specific check (in ecatappl.c)
				   if the state transition from INIT to PREOP should be done 
		         if result is unequal 0, the slave will stay in INIT 
		         and sets the ErrorInd Bit (bit 4) of the AL-Status */
				result = APPL_StartMailboxHandler();
			}
			break;

		case PREOP_2_SAFEOP:
			/* start the input handler (function is defined above) */
			result = StartInputHandler();
			break;	
	
		case SAFEOP_2_OP:
			/* start the output handler (function is defined above) */
			result = StartOutputHandler();
			break;

		case OP_2_SAFEOP:
			/* stop the output handler (function is defined above) */
			result = StopOutputHandler();
			break;

		case OP_2_PREOP:
			/* stop the output handler (function is defined above) */
			result = StopOutputHandler();
			if (result != 0)
				break;

		case SAFEOP_2_PREOP:
			/* stop the input handler (function is defined above) */
			result = StopInputHandler();
			break;

		case OP_2_INIT:
			/* stop the output handler (function is defined above) */
			result = StopOutputHandler();
			if (result != 0)
				break;

		case SAFEOP_2_INIT:
			/* stop the input handler (function is defined above) */
			result = StopInputHandler();
			if (result != 0)
				break;

		case PREOP_2_INIT:
			MBX_StopMailboxHandler();
			result = APPL_StopMailboxHandler();
			break;

		case INIT_2_INIT:
		case PREOP_2_PREOP:
		case SAFEOP_2_SAFEOP:
		case OP_2_OP:
			result = NOERROR_NOSTATECHANGE;
			break;

		case INIT_2_SAFEOP:
		case INIT_2_OP:
		case PREOP_2_OP:
		case PREOP_2_BOOT:
		case SAFEOP_2_BOOT:
		case OP_2_BOOT:
			result = ALSTATUSCODE_INVALIDALCONTROL;
			break;

		default:
			result = ALSTATUSCODE_UNKNOWNALCONTROL; 
			break;	
		}
	}
	else
	{
		/* the checking of the sync manager settings was not successful
		   we switch back the state to PREOP or INIT */
		switch (nAlStatus)
		{
		case STATE_OP:
			/* stop the output handler (function is defined above) */
			StopOutputHandler();
		case STATE_SAFEOP:
			/* stop the input handler (function is defined above) */
/* ECATCHANGE_START(V4.00) ECAT 2 */
			StopInputHandler();
/* ECATCHANGE_END(V4.00) ECAT 2 */

		case STATE_PREOP:
   		if ( result == ALSTATUSCODE_INVALIDMBXCFGINPREOP )
			{
				/* the mailbox sync manager settings were wrong, switch back to INIT */
				MBX_StopMailboxHandler();
				APPL_StopMailboxHandler();
				nAlStatus = STATE_INIT;
			}
			else		
				nAlStatus = STATE_PREOP;
		}
	}

	if ( result == NOERROR_INWORK )
	{
/* ECATCHANGE_START(V4.00) ECAT 6 */
		/* state transition is still in work 
			ECAT_StateChange must be called from the application */
		bEcatWaitForAlControlRes = TRUE;
/* ECATCHANGE_END(V4.00) ECAT 6 */
	}
	else
	/* The AL Status Code register shall not be modified if the function is called
	   in case of SM change event or an AL-Control event with the same state */
	if ( alControl != (nAlStatus & STATE_MASK) )
	{
/* ECATCHANGE_START(V4.01) ECAT 1 */
		if ( (result != 0 || nAlStatusCode != 0) && ((alControl | nAlStatus) & STATE_OP) ) 
		{
			/* we have to disable the SM2 if we leave the state OP or if we refuse the state transition to OP */
			HW_DisableSyncManChannel(PROCESS_DATA_OUT);
		}
/* ECATCHANGE_END(V4.01) ECAT 1 */
		if ( result != 0 )
		{
			/* save the failed status to be able to decide, if the AL Status Code shall be
			   reset in case of a coming successful state transition */
			nAlStatusFailed = nAlStatus;
			nAlStatus |= STATE_CHANGE;
		}
		else
		{
			/* state transition was succesful */
			if ( nAlStatusCode != 0 )
			{
				/* state change request from the user */
				result = nAlStatusCode;
				nAlStatusFailed = alControl;
				alControl |= STATE_CHANGE;
			}
			else if ( alControl <= nAlStatusFailed ) 
			{
				/* the old AL Status Code register shall not be overwritten */
				result = 0xFF;
			}
			/* reset failed status, when state transition successful */
			else
				nAlStatusFailed = 0;
			/* acknowledge the new state */
			nAlStatus = alControl;
		}
		
		/* write the AL Status register */
		HW_SetAlStatus(nAlStatus, result);
		nAlStatusCode = 0;
	}
	else
	{
		/* AL-Status has to be updated and AL-Status-Code has to be reset 
		   if the the error bit was acknowledged */
		HW_SetAlStatus(nAlStatus, 0);
	}
}

/* ECATCHANGE_START(V4.00) ECAT 3 */
/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief	This function checks the watchdog and shall be called every ms
 \brief  The checking is changed from the checking of register 0x440 to a timer controlled 
 \brief  checking because the ESC will reset the Watchdog even if only parts of the
 \brief  Sync Manager area are written.
 \brief  If the watchdog expires in SAFEOP only the flag indicating will be reset.
 \brief  If the watchdog expires in OP the reaction depends on the watchdog mode:
 \brief	if the WD-Trigger of the SM2/3 is 1 (bit 6 of Control-Byte), the state will 
 \brief  always be changed to SAFEOP
 \brief  if the WD-Trigger of the SM2/3 is 0 (bit 6 of Control-Byte), the state will
 \brief  only be changed if new outputs were received (SM2-event)	or inputs
 \brief  were read (SM3-event, if the output size is 0) once after switching to OP
 
*////////////////////////////////////////////////////////////////////////////////////////

void ECAT_CheckWatchdog(void)
{
	if ( u16WdValue != 0 )
	{
		// Watchdog is active
		u16WdCounter++;
		if ( u16WdCounter >= u16WdValue )
		{
			// Watchdog is expired
			u16WdCounter = 0;
			if ( bEcatOutputUpdateRunning )
			{
				/* watchdog expired in OP */
				/* notify mainloop task */
				bWdtExpiredNotify=TRUE;
			}
			else
			{
				/* watchdog expired in SAFE-OP */
				bEcatFirstOutputsReceived = FALSE;
			}
		}
	}
}
/* ECATCHANGE_END(V4.00) ECAT 3 */

/* ECATCHANGE_START(V4.00) ECAT 6 */
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param	alStatus: requested state
 \param	alStatusCode: value for the AL-Status register

 \brief	This function changes the state of the EtherCAT slave if the requested state
 			is lower than the actual state, otherwise the error condition will be reset.
*////////////////////////////////////////////////////////////////////////////////////////

void ECAT_StateChange(UINT8 alStatus, UINT16 alStatusCode)
{
	if ( bEcatWaitForAlControlRes )
	{
		/* we are waiting for AL Control Response because one of the functions
		   APPL_StartMailboxHandler, APPL_StopMailboxHandler, 
			APPL_StartInputHandler, APPL_StopInputHandler,
			APPL_StartOutputHandler, APPL_StopOutputHandler
			returned NOERROR_INWORK */
		alStatus &= STATE_MASK;
		if ( alStatus != STATE_OP && alStatusCode != 0 )
		{
			/* the state change was refused */
			alStatus |= STATE_CHANGE;
			bEcatLocalError = TRUE;
			if ( (nAlStatus & STATE_MASK) == STATE_SAFEOP && !bEcatLocalError )
			{
				/* we have to disable the SM2, if we are in SAFE-OP */
				HW_DisableSyncManChannel(PROCESS_DATA_OUT);
			}
		}
		/* reset the local flag */
		bEcatWaitForAlControlRes = FALSE;
		/* Update the AL-Status and AL-StatusCode register */
		HW_SetAlStatus(alStatus, alStatusCode);
	}
	else if ( alStatusCode != 0 )
	{
		/* Local error has happened, we change the state if necessary */
		bEcatLocalError = TRUE;
   		if ( (alStatus & STATE_MASK) < (nAlStatus & STATE_MASK) ) 
		{
			nAlStatusCode = alStatusCode;
			AL_ControlInd(alStatus);
		}
	}
	else if ( bEcatLocalError )
	{
		/* a local error is gone */
		if ( (nAlStatus & STATE_MASK) == STATE_SAFEOP )
		{
			/* we have to enable the SM2, because it was disabled when switching back to SAFE-OP */
			HW_EnableSyncManChannel(PROCESS_DATA_OUT);
		}
		bEcatLocalError = FALSE;
	}
}
/* ECATCHANGE_END(V4.00) ECAT 6 */

/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief	This function intializes the EtherCAT Slave Interface.
*////////////////////////////////////////////////////////////////////////////////////////

void ECAT_Init(void)
{
	/* initialize the mailbox handler */
	MBX_Init();	

	/* initialize variables */
	nAlStatusFailed = 0;
	nAlStatusCode = 0;
	bEcatFirstOutputsReceived = FALSE;
 	bEcatOutputUpdateRunning = FALSE;
 	bEcatInputUpdateRunning = FALSE;
	bWdTrigger = FALSE;
#if DC_SUPPORTED
	bDcSyncActive = FALSE;
#endif
	/* initialize the AL Status register */
	nAlStatus	= STATE_INIT;
	HW_SetAlStatus(nAlStatus, 0);

/* ECATCHANGE_START(V4.00) ECAT 1 */
	bEscIntEnabled = FALSE;
/* ECATCHANGE_END(V4.00) ECAT 1 */

	/* initialize sync managers parameters */
	tEcatCMSMInParam.u16SyncType=SYNCTYPE_FREERUN;
#if DC_SUPPORTED
	tEcatCMSyncManOutDiag.u16SyncTypesSupported=SYNCTYPE_FREERUNSUPP|SYNCTYPE_SYNCHRONSUPP|SYNCTYPE_DCSYNC0SUPP|SYNCTYPE_SUBAPPLSYNC0;
#else
	tEcatCMSyncManOutDiag.u16SyncTypesSupported=SYNCTYPE_FREERUNSUPP|SYNCTYPE_SYNCHRONSUPP;
#endif

	tEcatCMSyncManOutDiag.u32MinCycleTime=MIN_PD_CYCLE_TIME;
	tEcatCMSyncManOutDiag.u32Sync0CycleTime=MIN_PD_CYCLE_TIME;

#if COE_SUPPORTED || SOE_SUPPORTED
	/* initialize the emergency handler */
	EMCY_Init();
#endif
#if COE_SUPPORTED
	/* initialize the COE part */
	COE_Init();
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief		This function has to be called cyclically.
*////////////////////////////////////////////////////////////////////////////////////////

void ECAT_Main(void)
{
	/* check if services are stored in the mailbox */
	MBX_Main();

	/* check if masked interrupts were received */
	HW_Main();
	
	/* check for wdt expiration */
	if(bWdtExpiredNotify)
	{
		nAlStatusCode = ALSTATUSCODE_SMWATCHDOG;
		AL_ControlInd(STATE_SAFEOP);
		bWdtExpiredNotify=FALSE;
	}
}


/** @} */
