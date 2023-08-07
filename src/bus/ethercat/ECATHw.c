/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright $)AB) 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ECATHw.c                                                   */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : ECAT hardware access                                       */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "fpga\FpgaHandler.h"
#include "ECATHw.h"
#include "ECATEeprom.h"
#include "mailbox.h"
#include <string.h>

//***************************************************************************
// Globals

volatile BOOL bAlEventEnabled;
volatile UALEVENT EscAlEvent;
volatile UALCONTROL EscAlControl;

//***************************************************************************
// Locals

static UWORD uwLastAlStatusCode=ALSTATUSCODE_NOERROR;

//***************************************************************************
// ESC Locals

static TSYNCMAN TmpSyncMan;

//***************************************************************************
// ESC Defines

#ifndef	DISABLE_AL_EVENT_INT
	#define	DISABLE_AL_EVENT_INT
#endif
#ifndef	ENABLE_AL_EVENT_INT
	#define	ENABLE_AL_EVENT_INT
#endif
#ifndef START_TIMER
	#define	START_TIMER
#endif
#ifndef STOP_TIMER
	#define	STOP_TIMER
#endif
#ifndef ACK_TIMER_INT
	#define	ACK_TIMER_INT
#endif

//***************************************************************************
// ESC Read, must be word-wise as hw interface is, in order to avoid
// mailbox unlocking while reading last bytes

void HW_EscReadAccess( UINT8  *pData, UINT16 Address, UINT16 Len )
{
    UWORD  * puwSrc=(UWORD  *)(FPGA_ETHERNET_BASE_ADDRESS+(Address&0xFFFE));
    UWORD  * puwDst;
    UWORD uwDat;

    // if both source and destination pointers are word-aligned
    // then use an optimized read
    if((Address&1)==0 && ((ULONG)pData&1)==0)
    {
        uwDat=Len>>1;
        puwDst=(UWORD  *)pData;
        while(uwDat-->0)
            *puwDst++=*puwSrc++;

        // if odd then read last byte
        if(Len&1)
            *((UINT8  *)puwDst)=(UBYTE)(*puwSrc&0x00FF);
    }
    else
    {
        // if source address is odd then read first byte
        // separately
        if(Address&1)
        {
            *pData++=*(UINT8  *)(FPGA_ETHERNET_BASE_ADDRESS+Address);
            puwSrc++;
            Len--;
        }

        while(Len>0)
        {
            uwDat=*puwSrc++;
            *pData++=(UBYTE)(uwDat&0x00FF);
            Len--;
            if(Len>0)
            {
                *pData++=(UBYTE)(uwDat>>8);
                Len--;
            }
        }
    }
}

//***************************************************************************
// ESC Write

void HW_EscWriteAccess( UINT8  *pData, UINT16 Address, UINT16 Len )
{
    memcpy((HPVOID)(FPGA_ETHERNET_BASE_ADDRESS+Address),pData,Len);
}

//***************************************************************************
// This function makes an logical and with the AL Event Mask register

void HW_ResetIntMask(UINT16 intMask)
{
	UINT16 mask;

    mask=ESC_UINT16(ESC_ADDR_ALEVENTMASK);
#if MOTOROLA_FORMAT
	mask = SWAPWORD(mask);
#endif
	mask &= intMask;
#if AL_EVENT_ENABLED
	DISABLE_AL_EVENT_INT;
	if ( mask == 0 )
	{
		bAlEventEnabled = FALSE;
		START_TIMER;
	}
#endif
#if MOTOROLA_FORMAT
	mask = SWAPWORD(mask);
#endif
    ESC_UINT16(ESC_ADDR_ALEVENTMASK)=mask;
#if AL_EVENT_ENABLED
	ENABLE_AL_EVENT_INT;
#endif
}

#if AL_EVENT_ENABLED
//***************************************************************************
// This function makes an logical or with the AL Event Mask register

void HW_SetIntMask(UINT16 intMask)
{
	UINT16 mask;

    mask=ESC_UINT16(ESC_ADDR_ALEVENTMASK);
#if MOTOROLA_FORMAT
	mask = SWAPWORD(mask);
#endif
	mask |= intMask;
#if AL_EVENT_ENABLED
	DISABLE_AL_EVENT_INT;
	STOP_TIMER;
	ACK_TIMER_INT;
	bAlEventEnabled = TRUE;
#endif
#if MOTOROLA_FORMAT
	mask = SWAPWORD(mask);
#endif
    ESC_UINT16(ESC_ADDR_ALEVENTMASK)=mask;
#if AL_EVENT_ENABLED
	ENABLE_AL_EVENT_INT;
#endif
}

#endif

//***************************************************************************
// The function changes the state of the EtherCAT ASIC to the requested.

void HW_SetAlStatus(UINT8 alStatus, UINT16 alStatusCode)
{
	UINT16 state = alStatus;

    ESC_UINT16(ESC_ADDR_ALSTATUS)=state;
	if (alStatusCode != 0xFF)
	{
		uwLastAlStatusCode = state = alStatusCode;
        ESC_UINT16(ESC_ADDR_ALSTATUSCODE)=state;
	}
}

//***************************************************************************
// This function copies data to the Send Mailbox.

UINT8 HW_CopyToSendMailbox( TMBX MBXMEM *pMbx )
{
	if ( bSendMbxIsFull )
	{
		/* mailbox service cannot be sent because the send mailbox is still full */
		return ERROR_NOMEMORY;
	}
	else
	{
		UINT16 mbxSize = pMbx->MbxHeader.Word[MBX_OFFS_LENGTH];
#if MOTOROLA_FORMAT
		/* the size has to be swapped here, all other bytes of the mailbox service were swapped before */
		pMbx->MbxHeader.Word[MBX_OFFS_LENGTH] = SWAPWORD(mbxSize);
#endif
		HW_EscWriteAccess( (UINT8 *) pMbx, u16EscAddrSendMbx, mbxSize + sizeof(UMBXHEADER) );

		if ( (mbxSize + sizeof(UMBXHEADER)) < u16SendMbxSize )
			// last byte has to be written to unlock the Mailbox:
			ESC_UINT8( u16EscAddrSendMbx + u16SendMbxSize - 1 )=0;

		/* store last send mailbox service for a possible repeat
			one buffer includes the last send service (psRepeatMbx),
			the other one the actual service to be sent (psReadMbx),
			there is no buffer available for a mailbox receive service
			until the last sent buffer was read from thze master
			the exception is after the INIT2PREOP transition, in that
			case there is no last sent service (psReadMbx = 0) */
		if ( psReadMbx )
			psWriteMbx = NULL;
		else
			/* only the first time after the INIT2PREOP-transition */
			psWriteMbx = &asMbx[1];
		// HBu 17.06.06: psRepeatMbx was already updated in MBX_MailboxReadInd
		// psRepeatMbx = psReadMbx;
		psReadMbx = pMbx;

		/* set flag that send mailbox is full now */
		bSendMbxIsFull = TRUE;
		return 0;
	}
}

//***************************************************************************
// This function is used to check if the received mailbox command can be processed.
// Also the contents of the Receive Mailbox will be copied in the variable sMbx.

void HW_CheckAndCopyMailbox( void )
{
	UINT16 mbxLen;

	/* get the size of the received mailbox command and acknowledge the event (by reading first byte) */
	mbxLen=ESC_UINT16( u16EscAddrReceiveMbx );
#if MOTOROLA_FORMAT
	/* the size has to be swapped here, all other bytes of the mailbox service will be swapped later */
	mbxLen = SWAPWORD(mbxLen);
#endif
	/* the length of the mailbox data is in the first two bytes of the mailbox,
	   so the length of the mailbox header has to be added */
	mbxLen += SIZEOF(UMBXHEADER);

	/* in this example there are only two mailbox buffers available in the firmware (one for processing and
	   one to stored the last sent response for a possible repeat request), so a
	   received mailbox service can only be processed if a free buffer is available */
	if ( ( bSendMbxIsFull )				/* a received mailbox service will not be processed
													as long as the send mailbox is still full
													(waits to be read from the master) */
		||( u8MailboxSendReqStored )	/* a mailbox service to be sent is still stored
													so the received mailbox service will not be processed
													until all stored mailbox services are sent */
		)
	{
		/* set flag that the processing of the mailbox service will be checked in the
			function MBX_Main (called from ECAT_Main) */
		bReceiveMbxIsLocked = TRUE;
	}
	else
	{
		/* received mailbox command can be processed, reset flag */
		bReceiveMbxIsLocked = FALSE;
		/* if the read mailbox size is too big for the buffer, set the copy size to the maximum buffer size, otherwise
		   memory could be overwritten,
		   the evaluation of the mailbox size will be done in the mailbox protocols called from MBX_WriteMailboxInd */
		if (mbxLen > sizeof(TMBX))
			mbxLen = sizeof(TMBX);

		/* copy the mailbox header and data except the first two bytes */
		HW_EscReadAccess( (UINT8 MBXMEM *) &psWriteMbx->MbxHeader.Word[MBX_OFFS_ADDRESS], u16EscAddrReceiveMbx+2, mbxLen-2 );

		if ( mbxLen < u16ReceiveMbxSize )
		{
			UINT8 dummy;

			/* the last byte of the receive mailbox has to be read to unlock the mailbox  */
			dummy=ESC_UINT8( u16EscAddrReceiveMbx + u16ReceiveMbxSize - 1 );
		}

		/* store the mailbox data length in the buffer */
		psWriteMbx->MbxHeader.Word[MBX_OFFS_LENGTH] = mbxLen - SIZEOF(UMBXHEADER);
		/* in MBX_MailboxWriteInd the mailbox protocol will be processed */
		MBX_MailboxWriteInd( psWriteMbx );
	}
}

//***************************************************************************
// This function disables a Sync Manager channel

void HW_DisableSyncManChannel(UINT8 channel)
{
	/* HBu 13.02.06: the offset of the Sync manager settings had to be added */
	ESC_UINT8(ESC_ADDR_SYNCMAN+sizeof(TSYNCMAN)*channel+ESC_OFFS_SMSETTINGS+ESC_OFFS_PDIDISABLE)=1;
}

//***************************************************************************
// This function enables a Sync Manager channel

void HW_EnableSyncManChannel(UINT8 channel)
{
	/* HBu 13.02.06: the offset of the Sync manager settings had to be added */
	ESC_UINT8(ESC_ADDR_SYNCMAN+sizeof(TSYNCMAN)*channel+ESC_OFFS_SMSETTINGS+ESC_OFFS_PDIDISABLE)=0;
}

//***************************************************************************
// This function is called to read the SYNC Manager channel descriptions of the
// process data SYNC Managers.

TSYNCMAN * HW_GetSyncMan(UINT8 channel)
{
	// get a temporary structure of the Sync Manager:
	HW_EscReadAccess( (UINT8 *) &TmpSyncMan, ESC_ADDR_SYNCMAN + (channel * sizeof(TSYNCMAN)), sizeof(TSYNCMAN) );
#if MOTOROLA_FORMAT
	TmpSyncMan.PhysicalStartAddress = SWAPWORD(TmpSyncMan.PhysicalStartAddress);
	TmpSyncMan.Length = SWAPWORD(TmpSyncMan.Length);
#endif

	return &TmpSyncMan;
}

//***************************************************************************
// This function is called to read the AL Event register

UINT16 HW_GetAlEvent(void)
{
	EscAlEvent.Word[0]=ESC_UINT16(ESC_ADDR_ALEVENT);

	return EscAlEvent.Word[0];
}

//***************************************************************************
// Read from PHY via MDIO

BOOL HW_MIIMRead(UINT8 ubPhyAddr, UINT8 ubRegAddr, UINT16 * hpuRegRdata)
{
    BOOL bValid=FALSE;
    UMIIADDRESSES adrs;

    // check if possible takeover control from ECAT
    if(ESC_UINT16(ESC_MII_ACCESS)&ESC_MIIAC_ECATEXCLUSIVE)
        return FALSE;

    // request access
    ESC_UINT16(ESC_MII_ACCESS)|=ESC_MIIAC_PDIACCESS;

    // wait until not busy
    while(ESC_UINT16(ESC_MII_CTRLSTAT)&ESC_MIICS_BUSY);

    // setup transfer
    adrs.Byte[0]=ubPhyAddr;
    adrs.Byte[1]=ubRegAddr;
    ESC_UINT16(ESC_MII_ADDRESSES)=adrs.Word[0];

    // execute
    ESC_UINT16(ESC_MII_CTRLSTAT)|=ESC_MIICS_CMD_READ;

    // and wait
    while(ESC_UINT16(ESC_MII_CTRLSTAT)&ESC_MIICS_BUSY);

    // if no errors
    if((ESC_UINT16(ESC_MII_CTRLSTAT)&(ESC_MIICS_READ_ERROR|ESC_MIICS_CMD_ERROR))==0)
    {
        // read data
        *hpuRegRdata=ESC_UINT16(ESC_MII_DATA);
        bValid=TRUE;
    }

    // return access to ECAT
    ESC_UINT16(ESC_MII_ACCESS)&=~ESC_MIIAC_PDIACCESS;

    return bValid;
}

//***************************************************************************
// Write to PHY via MDIO

BOOL HW_MIIMWrite(UINT8 ubPhyAddr, UINT8 ubRegAddr, UINT16 uwRegData)
{
    BOOL bValid=FALSE;
    UMIIADDRESSES adrs;

    // check if possible takeover control from ECAT
    if(ESC_UINT16(ESC_MII_ACCESS)&ESC_MIIAC_ECATEXCLUSIVE)
        return FALSE;

    // request access
    ESC_UINT16(ESC_MII_ACCESS)|=ESC_MIIAC_PDIACCESS;

    // wait until not busy
    while(ESC_UINT16(ESC_MII_CTRLSTAT)&ESC_MIICS_BUSY);

    // setup transfer
    adrs.Byte[0]=ubPhyAddr;
    adrs.Byte[1]=ubRegAddr;
    ESC_UINT16(ESC_MII_ADDRESSES)=adrs.Word[0];
    ESC_UINT16(ESC_MII_DATA)=uwRegData;

    // execute
    ESC_UINT16(ESC_MII_CTRLSTAT)|=ESC_MIICS_CMD_WRITE;

    // and wait
    while(ESC_UINT16(ESC_MII_CTRLSTAT)&ESC_MIICS_BUSY);

    // if no errors
    if((ESC_UINT16(ESC_MII_CTRLSTAT)&(ESC_MIICS_READ_ERROR|ESC_MIICS_CMD_ERROR))==0)
        bValid=TRUE;

    // return access to ECAT
    ESC_UINT16(ESC_MII_ACCESS)&=~ESC_MIIAC_PDIACCESS;

    return bValid;
}

//***************************************************************************
// ESC Retrieve Diagnostics

void HW_DiagRefresh(ECATCM_DIAG * psDiag)
{
    UWORD uwCt,uwTmp;

        // report diagnostic from ESC
    for(uwCt=0;uwCt<2;uwCt++)
    {
        uwTmp=ESC_UINT16(ESC_ADDR_ERRORS+sizeof(UWORD)*uwCt);
        psDiag->sPort[uwCt].ubInvFrameCnt=(UBYTE)(uwTmp&0x00ff);
        psDiag->sPort[uwCt].ubRxErrCnt=(UBYTE)(uwTmp>>8);

        psDiag->sPort[uwCt].ubForwardedRxErrCnt=ESC_UINT8(ESC_ADDR_ERRORS+0x0008+uwCt);

        psDiag->sPort[uwCt].ubLostLinkCnt=ESC_UINT8(ESC_ADDR_ERRORS+0x0010+uwCt);
    }

    psDiag->ubProcUnitErrCnt=ESC_UINT8(ESC_ADDR_ERRORS+0x000C);
    psDiag->ubPDIErrCnt=ESC_UINT8(ESC_ADDR_ERRORS+0x000D);

    uwTmp=ESC_UINT16(ESC_ADDR_DLLSTATUS);

#ifdef _INFINEON_
    psDiag->sFlags.b.bLink0=(bit)(uwTmp&0x0010);
    psDiag->sFlags.b.bLink1=(bit)(uwTmp&0x0020);
    psDiag->sFlags.b.bCommunication0=(bit)(uwTmp&0x0200);
    psDiag->sFlags.b.bCommunication1=(bit)(uwTmp&0x0800);
#else
    psDiag->sFlags.b.bLink0=((uwTmp&0x0010)==0x0010);
    psDiag->sFlags.b.bLink1=((uwTmp&0x0020)==0x0020);
    psDiag->sFlags.b.bCommunication0=((uwTmp&0x0200)==0x0200);
    psDiag->sFlags.b.bCommunication1=((uwTmp&0x0800)==0x0800);
#endif

    psDiag->ubALStatus=(unsigned char)ESC_UINT16(ESC_ADDR_ALSTATUS)&0x000F;
    psDiag->uwALStatusCode=uwLastAlStatusCode;
}

//***************************************************************************
// set PHYs led control to link-only

BOOL HW_SetPHYLeds()
{
    UWORD data,rdata,adr;

    for(adr=0x00;adr<=0x01;adr++)
    {
        // PHY Identifier 2 [9:4]: model number
        HW_MIIMRead(adr, 0x03, &data);
        if((data&0x3F0)==0x170) // ksz8061
        {
            // register = 0x1F
            HW_MIIMRead(adr, 0x1F, &data);
            data=(data&~0x0030)|0x0010;
            HW_MIIMWrite(adr, 0x1F, data);
        
            // readback data for checking
            HW_MIIMRead(adr, 0x1F, &rdata);
            if(data!=rdata)
                return FALSE;
        }
        else if((data&0x3F0)==0x110) // ksz8041
        {
            // register = 0x1E, [15:14]: LED Mode
            //                      00 = LED1 : Speed, LED0 : Link/Activity
            //                      01 = LED1 : Activity, LED0 : Link
            HW_MIIMRead(adr, 0x1E, &data);
            data=(data&~0xC000)|0x4000;
            HW_MIIMWrite(adr, 0x1E, data);

            // readback data for checking
            HW_MIIMRead(adr, 0x1E, &rdata);
            if(data!=rdata)
                return FALSE;

            // register 14h [7:6]: Preamble Restore
            //                   1 = Restore received preamble to
            //                       MII output (random latency)
            //                   0 = Consume 1-byte preamble
            //                       before sending frame to MII
            //                       output for fixed latency
            HW_MIIMRead(adr,0x14,&data);
            data=(data&~(0x0003<<6))|(0x0003<<6);
            HW_MIIMWrite(adr,0x14,data);
        
            // readback data for checking
            HW_MIIMRead(adr, 0x14, &rdata);
            if(data!=rdata)
                return FALSE;
        }
        else if((data&0x3F0)==0x120)//YT8512
        {
			//set led0 turn on when link led1 blink when active 
			HW_MIIMWrite(adr, 0x1E, 0x40C0); // write ext reg 0x40C0 to set the led0 work model
			HW_MIIMWrite(adr, 0x1F, 0x0030); //  Set led0 turn on when linked or active
			HW_MIIMWrite(adr, 0x1E, 0x40C3); // Write ext reg 0x40C2 to set the led1 work model
			HW_MIIMWrite(adr, 0x1F, 0X1300); // Write ext reg 0x50 bit6 to 1

            // readback data for checking
            HW_MIIMWrite(adr, 0x1E, 0x40C0); // Read ext reg 0x40C0 
			HW_MIIMRead(adr, 0x1F, &rdata); // 
            if(rdata!=0x0030)
            {
                return FALSE;
            }

            HW_MIIMWrite(adr, 0x1E, 0x40C3); // Read ext reg 0x40C0 
			HW_MIIMRead(adr, 0x1F, &rdata); // 
            if(rdata!=0x1300)
            {
                return FALSE;
            }
        }
        else
            return FALSE;
    }

    return TRUE;
}

//***************************************************************************
// ESC init

void HW_Init(void)
{
	UINT8 u8PDICtrl = 0;

	do
	{
		/* we have to retry because the ESC might not booted completely */
		u8PDICtrl=ESC_UINT8( ESC_ADDR_PDICTRL );
		/* MichaelSt 02.06.06: if PDI Type is SPI, DPRAM size could be changed in future (ASIC) */
#ifdef _INFINEON_
	} while (u8PDICtrl != 0x08); // 16 Bit asynchronous Microcontroller interface
#else
	} while (u8PDICtrl != 0x80); // On-chip bus: AXI
#endif
	nMaxSyncMan=ESC_UINT8( ESC_ADDR_SYNCMANCHANNELS );

	HW_ResetIntMask(0);
}

//***************************************************************************
// Main Loop

void HW_Main(void)
{
	/* Read AL Event-Register from ESC */
	HW_GetAlEvent();

	if (EscAlEvent.Byte[0] & AL_CONTROL_EVENT)
	{
		/* AL Control event is set, get the AL Control register sent by the Master to acknowledge the event
		  (that the corresponding bit in the AL Event register will be reset) */
		EscAlControl.Byte[0]=ESC_UINT8( ESC_ADDR_ALCONTROL );
		/* reset AL Control event and the SM Change event (because the Sync Manager settings will be checked
		   in AL_ControlInd, too) in the global variable EscAlEvent */
		EscAlEvent.Byte[0] &= ~(((UINT8) AL_CONTROL_EVENT) | ((UINT8) SM_CHANGE_EVENT));
		AL_ControlInd(EscAlControl.Byte[0]); /* in AL_ControlInd the state transition will be checked and done */
	}

	if ( EscAlEvent.Byte[0] & SM_CHANGE_EVENT )
	{
		/* the SM Change event is set (Bit 4 of Register 0x220), when the Byte 6 (Enable, Lo-Byte of Register 0x806, 0x80E, 0x816,...)
		   of a Sync Manager channel was written */
		/* HBu 12.12.06: the SM Change event in the global variable EscAlEvent has to be reset before calling AL_ControlInd
		   because EscAlEvent will be updated in the ESC interrupt routine (which could be called in this example
		   depending on the synchronization mode when the SYNC0/SYNC1-, SM2- or SM3-event is set) that it could happen
		   that the SM Change event will be set again while the last one is still processed in AL_ControlInd */
		EscAlEvent.Byte[0] &= ~((UINT8) SM_CHANGE_EVENT);
		/* AL_ControlInd is called with the actual state, so that the correct SM settings will be checked */
		AL_ControlInd((UINT8)(nAlStatus & STATE_MASK));
	}

	// HBu 06.02.06: the order of the checking was changed (the three events
	//               has to be read consistent from the ESC):
	//					  1. check the toggle bits for repeat
	//					  2. check if the send mailbox was read by the master
	//					  3. check if the receive mailbox was written by the master
	if ( bMbxRunning )
	{
		/* Slave is at least in PREOP, Mailbox is running */
		UINT8 sm1Activate;

		/* get the Activate-Byte of SM 1 (Register 0x80E) to check if a mailbox repeat request was received */
		sm1Activate=ESC_UINT8( ESC_ADDR_SM_MBXREAD+ESC_OFFS_SMSETTINGS+ESC_OFFS_ECATENABLE );
		/* bMbxRepeatToggle holds the last state of the Repeat Bit (Bit 1) */
		if ( ( (sm1Activate & SM_TOGGLEMASTER) && !bMbxRepeatToggle )
			||( !(sm1Activate & SM_TOGGLEMASTER) && bMbxRepeatToggle )
			)
		{
			/* Repeat Bit (Bit 1) has toggled, there is a repeat request, in MBX_MailboxRepeatReq the correct
			   response will put in the send mailbox again */
			MBX_MailboxRepeatReq();
			/* acknowledge the repeat request after the send mailbox was updated by writing the Repeat Bit
			   in the Repeat Ack Bit (Bit 1) of the PDI Ctrl-Byte of SM 1 (Register 0x80F) */
			sm1Activate &= 0x02;
			ESC_UINT8( ESC_ADDR_SM_MBXREAD+ESC_OFFS_SMSETTINGS+ESC_OFFS_PDIDISABLE )=sm1Activate;
		}

		if ( (EscAlEvent.Byte[1] & (MAILBOX_READ_EVENT>>8)) )
		{
			/* SM 1 (Mailbox Read) event is set, when the mailbox was read from the master,
			   to acknowledge the event the first byte of the mailbox has to be written,
			   by writing the first byte the mailbox is locked, too */
			ESC_UINT8( u16EscAddrSendMbx )=0;
			/* the Mailbox Read event in the global variable EscAlEvent shall be reset before calling
			   MBX_MailboxReadInd, where a new mailbox datagram (if available) could be stored in the send mailbox */
			EscAlEvent.Byte[1] &= ~((UINT8) (MAILBOX_READ_EVENT>>8));
			MBX_MailboxReadInd();
		}

		if ( (EscAlEvent.Byte[1] & (MAILBOX_WRITE_EVENT>>8)) )
		{
			/* SM 0 (Mailbox Write) event is set, when the mailbox was written from the master,
			   to acknowledge the event the first byte of the mailbox has to be read,
			   which will be done in HW_CheckAndCopyMailbox */
			/* the Mailbox Write event in the global variable EscAlEvent shall be reset before calling
			   HW_CheckAndCopyMailbox, where the received mailbox datagram will be processed */
			EscAlEvent.Byte[1] &= ~((UINT8) (MAILBOX_WRITE_EVENT>>8));
			HW_CheckAndCopyMailbox();
		}
	}

	if ( EscAlEvent.Byte[0] & EEPROM_COMMAND_EVENT )
	{
        // EEPROM Emulation
		EscAlEvent.Byte[0] &= ~((UINT8) EEPROM_COMMAND_EVENT);
        ECATEE_Emulation();
    }
}

