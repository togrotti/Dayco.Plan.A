/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : MultiCANController.c                                       */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Driver for Xillinx MultiCAN Controller, targeted for      */
/*               CanOpen                                                    */
/*                                                                          */
/****************************************************************************/
/* SYSTEM CONFIG:                                                           */
/* - Module clock 40MHz                                                     */
/****************************************************************************/

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "xparameters.h"
#include "xscutimer.h"
#include "xscugic.h"
#ifndef _RD
#include "system\SysAppConfig.h"
#include "drive\AxM-E-Defines.h"
#include "drive\HardwareUnitID.h"
#else
#include "RemDispAppGlobals.h"
#include "RemDisp-Defines.h"
#endif

#include "system\SysAppInterruptLevels.h"
#include "MultiCANController.h"
#include "common\CommonUtility.h"
//#include "FatalErrorCodes.h"
//#include "InterruptLevels.h"
#include "system\Os.h"
#include "system\SysAppGlobals.h"
#ifdef CFG_CANDRV_PERIODICCOB
#include "common\TaskScheduler.h"
#endif

#ifdef CFG_CANDRV_PLCWRAPPER
#include "plc\Plc.h"
#endif

#ifdef CFG_CANDRV_STATUSSIGNAL
#include "system\SystemStatus.h"
#endif

#if CFG_CANDRV_CMDMGR
#include <string.h>
//#include <intrins.h>

#include "xcanps.h"
#include "xparameters.h"
#include "xstatus.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "MultiCANControllerDefs.h"
#include "MultiCANController.h"
//***************************************************************************
// Globals
#define	CANDRV_10KBPS		8
#define	CANDRV_20KBPS		7
#define	CANDRV_50KBPS		6
#define	CANDRV_100KBPS		5
#define	CANDRV_125KBPS		4
#define	CANDRV_250KBPS		3
#define	CANDRV_500KBPS		2
#define	CANDRV_800KBPS		1
#define	CANDRV_1000KBPS		0

#define CAN_DEVICE_ID_0                XPAR_XCANPS_0_DEVICE_ID
#define CAN_INT_ID_0                   XPS_CAN0_INT_ID
#ifndef _HW_AXS
#define CAN_DEVICE_ID_1                XPAR_XCANPS_1_DEVICE_ID
#define CAN_INT_ID_1                   XPS_CAN1_INT_ID
#endif
#define MAX_FIFO  64

#pragma GCC optimize (2)
/************************** Constant Definitions ****************************/


/*
 * The Baud Rate Prescaler Register (BRPR) and Bit Timing Register (BTR)
 * are setup such that CAN baud rate equals 40Kbps, assuming that the
 * the CAN clock is 24MHz. The user needs to modify these values based on
 * the desired baud rate and the CAN clock frequency. For more information
 * see the CAN 2.0A, CAN 2.0B, ISO 11898-1 specifications.
 */

typedef struct{
   UWORD sjw  : 2;
   UWORD tseg1: 4;
   UWORD tseg2: 2;
   UWORD pre  : 8;
} CANDRV_TIMING;

static CANDRV_TIMING sCandrvTiming[]=
{
   {1, 5 , 2, 5  },  // 1 Mbps
   {1, 7 , 2, 5  },  // 800 kbps
   {1, 13, 2, 5  },  // 500 kbps
   {1, 13, 2, 10 },  // 250 kbps
   {1, 13, 2, 20 }, // 125 kbps
   {1, 13, 2, 25 },  // 100 kbps
   {1, 13, 2, 50 },  // 50 kbps
   {1, 7 , 2, 200},  // 20 kbps
//   {1, 7 , 2, 400}   // 10 kbps
};

static XCanPs Can[CANDRV_MAX_CANNODE];

#ifndef _HW_AXS
static u16 CanDevID[CANDRV_MAX_CANNODE]=
{
   CAN_DEVICE_ID_0, CAN_DEVICE_ID_1
};

static u16 CanIntrID[CANDRV_MAX_CANNODE]=
{
   CAN_INT_ID_0, CAN_INT_ID_1
};
#else
static u16 CanDevID[CANDRV_MAX_CANNODE]=
{
   CAN_DEVICE_ID_0
};

static u16 CanIntrID[CANDRV_MAX_CANNODE]=
{
   CAN_INT_ID_0
};
#endif


volatile UWORD uwCanDrvGlobalFlags[CANDRV_MAX_CANNODE];
volatile UWORD uwCanDrvTimer100us;
volatile BOOL bJumpQueueFlag[CANDRV_MAX_CANNODE];

RXMBOXPARAMS sCanDrvRxList[CANDRV_MAX_CANNODE][MAX_RXLIST_COBS];

    // registered tx cob for inhibit time management
static TXMBOXPARAMS sTxAwaitingCobs[CANDRV_MAX_CANNODE][MAX_TXLIST_COBS];

    // local flags
static volatile UWORD uwCanFlags[CANDRV_MAX_CANNODE];

    // status for sync cob sending management
SYNCCOBSTAT sCanDrvSyncCobStat[CANDRV_MAX_CANNODE];

    // callback for sync cob sending management
void (* pfCanDrvSyncCobCallBack)(UWORD);

    // TX callback vector
static void (* pfTxCobCallBackVect[MAX_TXCALLBACKS])(CANDRV_COB *);

static void RecvHandler0(void *CallBackRef);
static void SendHandler0(void *CallBackRef);

static void RecvHandler1(void *CallBackRef);
static void SendHandler1(void *CallBackRef);

/*****************************************************************************/
/**
*
* This function sets up the interrupt system so interrupts can occur for the
* CAN. This function is application-specific since the actual system may or
* may not have an interrupt controller. The CAN could be directly connected
* to a processor without an interrupt controller. The user should modify this
* function to fit the application.
*
* @param	IntcInstancePtr is a pointer to the instance of the ScuGic.
* @param	CanInstancePtr contains a pointer to the instance of the CAN
*		which is going to be connected to the interrupt
*		controller.
* @param	CanIntrId is the interrupt Id and is typically
*		XPAR_<CANPS_instance>_INTR value from xparameters.h.
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None.
*
****************************************************************************/
static int SetupInterruptSystem(XScuGic *IntcInstancePtr, XCanPs *CanInstancePtr, u16 CanIntrId)
{
   int Status;

//   if(!IntcInstancePtr->IsReady)
   {
      XScuGic_Config *IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
      if (NULL == IntcConfig) {
         return XST_FAILURE;
      }

      Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
               IntcConfig->CpuBaseAddress);
      if (Status != XST_SUCCESS) {
         return XST_FAILURE;
      }

      /*
      * Connect the interrupt controller interrupt handler to the
      * hardware interrupt handling logic in the processor.
      */
      Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler) XScuGic_InterruptHandler, IntcInstancePtr);

      /* Enable interrupts */
//      Xil_ExceptionEnable();
   }

   /* The priority setting */
//   XScuGic_SetPriorityTriggerType(IntcInstancePtr, CanIntrId, 8<<0, 1);

   /*
   * Connect the device driver handler that will be called when an
   * interrupt for the device occurs, the handler defined above performs
   * the specific interrupt processing for the device.
   */
   Status = XScuGic_Connect(IntcInstancePtr, CanIntrId, (Xil_InterruptHandler)XCanPs_IntrHandler, (void *)CanInstancePtr);
   if (Status != XST_SUCCESS) {
      return Status;
   }

   /*
   * Enable the interrupt for the CAN device.
   */
   XScuGic_Enable(IntcInstancePtr, CanIntrId);
   /* Enable interrupts */
   Xil_ExceptionEnable();

   return XST_SUCCESS;
}



/*****************************************************************************/
/**
*
* CAN Initialization
*
* @param	None
*
* @return
*		- XST_SUCCESS if the example has completed successfully.
*		- XST_FAILURE if the example has failed.
*
* @note		None
*
******************************************************************************/
u32 Can_Init(u16 CanID)
{
   int Status;
   XCanPs *CanInstPtr = &Can[CanID];
   XCanPs_Config *ConfigPtr;

   ConfigPtr = XCanPs_LookupConfig(CanDevID[CanID]);
	if (ConfigPtr == NULL)
		return FALSE;

   Status = XCanPs_CfgInitialize(CanInstPtr,ConfigPtr,ConfigPtr->BaseAddr);
	if (Status != XST_SUCCESS)
		return FALSE;

   Status = XCanPs_SelfTest(CanInstPtr);
   if (Status != XST_SUCCESS)
      return FALSE;

   XCanPs_EnterMode(CanInstPtr, XCANPS_MODE_CONFIG);
   while(XCanPs_GetMode(CanInstPtr) != XCANPS_MODE_CONFIG);


   XCanPs_SetBaudRatePrescaler(CanInstPtr, sCandrvTiming[CANDRV_125KBPS].pre-1);
   XCanPs_SetBitTiming(CanInstPtr, sCandrvTiming[CANDRV_125KBPS].sjw,
                                   sCandrvTiming[CANDRV_125KBPS].tseg2-1,
                                   sCandrvTiming[CANDRV_125KBPS].tseg1-1);


   if(NODE0==CanID)
   {
	   XCanPs_SetHandler(CanInstPtr, XCANPS_HANDLER_SEND, (void *)SendHandler0, (void *)CanInstPtr);
	   XCanPs_SetHandler(CanInstPtr, XCANPS_HANDLER_RECV, (void *)RecvHandler0, (void *)CanInstPtr);
   }
   else
   {
	    XCanPs_SetHandler(CanInstPtr, XCANPS_HANDLER_SEND, (void *)SendHandler1, (void *)CanInstPtr);
	    XCanPs_SetHandler(CanInstPtr, XCANPS_HANDLER_RECV, (void *)RecvHandler1, (void *)CanInstPtr);
   }

    Status =  SetupInterruptSystem(&xInterruptController, CanInstPtr, CanIntrID[CanID]);
    if (Status != XST_SUCCESS)
       return FALSE;
    XCanPs_IntrEnable(CanInstPtr,XCANPS_IXR_RXOK_MASK|XCANPS_IXR_TXOK_MASK|XCANPS_IXR_RXNEMP_MASK);   //

#if CAN_DEBUG
    XCanPs_EnterMode(CanInstPtr, XCANPS_MODE_NORMAL);
    while(XCanPs_GetMode(CanInstPtr) != XCANPS_MODE_NORMAL);
#ifdef SENDTEST
    while(!sendinterrupt)
    SendFrame(CanInstPtr);    //
#endif
#endif

   return TRUE;
}


// Node enable
BOOL candrv_run(UWORD cannode)
{
    volatile UWORD * uwPtr;

    assert(cannode<CANDRV_MAX_CANNODE);

    if((uwCanFlags[cannode]&CAN_PORTAVAILABLE)==0)
        return FALSE;

#ifdef CFG_DS301_INHIBITIME
	{
		short ct;
		for(ct=0;ct<INHIBIT_LIST_SZ;ct++)		// azzera la lista di controllo dell'inhibit time
			pwInhibitCnt[ct]=0;
	}
#endif

        // get pointer for atomic operations
    uwPtr=&uwCanDrvGlobalFlags[cannode];

        // reset global status words
	fast_atomic_clear_bits(*uwPtr,CANDRV_GF_ERRPASV|CANDRV_GF_BUSOFF|CANDRV_GF_HWOVERRUN);

        // then enable node
    exitsoftres(cannode);     // */

	fast_atomic_set_bits(*uwPtr,CANDRV_GF_RUN);
    uwCanFlags[cannode]=CAN_PORTAVAILABLE|CAN_INHIBITENABLED;

	return TRUE;
}

//***************************************************************************
// Node disable
BOOL candrv_stop(UWORD cannode)
{
    volatile UWORD * uwPtr;
    UWORD i;

    assert(cannode<CANDRV_MAX_CANNODE);

    if((uwCanFlags[cannode]&CAN_PORTAVAILABLE)==0)
        return FALSE;

/*	if(is_softrescan(cannode))
		return FALSE;
//*/
        // get pointer for atomic operations
    uwPtr=&uwCanFlags[cannode];

        // begin shutdown sequence
    fast_atomic_set_bits(*uwPtr, CAN_SHUTTINGDOWN);
    fast_atomic_clear_bits(*uwPtr, CAN_INHIBITENABLED);

    do
    {
            // wait all tx pendings
        for(i=0;i<MAX_TXLIST_COBS;i++)
            if(sTxAwaitingCobs[cannode][i].psTxCob)
                break;

            // or in case of bus errors exit immediately
        if(uwCanDrvGlobalFlags[cannode]&(CANDRV_GF_ERRPASV|CANDRV_GF_BUSOFF))
            break;

    } while(i<MAX_TXLIST_COBS);

        // disable can node
	entersoftres(cannode);

	uwCanDrvGlobalFlags[cannode]=0;
    uwCanFlags[cannode]=CAN_PORTAVAILABLE;

	return TRUE;
}

//***************************************************************************
// Can status
BOOL candrv_isrunning(UWORD cannode)
{
    assert(cannode<CANDRV_MAX_CANNODE);

    if((uwCanFlags[cannode]&CAN_PORTAVAILABLE)==0)
        return FALSE;

	return !(is_softrescan(cannode) || (uwCanFlags[cannode]&CAN_SHUTTINGDOWN));
}


//***************************************************************************
// Setup baudrate
BOOL candrv_setspeed(UWORD cannode, SWORD speed)
{
    assert(cannode<CANDRV_MAX_CANNODE);      //

    if((uwCanFlags[cannode]&CAN_PORTAVAILABLE)==0)     //
        return FALSE;

	if(!candrv_testspeed(speed))      //
		return FALSE;

	if(!is_softrescan(cannode))
		return FALSE;

//	 XCanPs *CanInstPtr = &Can[cannode];
//	 XCanPs_Config *ConfigPtr;

//	 ConfigPtr = XCanPs_LookupConfig(CanDevID[cannode]);

//	 if (XCanPs_CfgInitialize(CanInstPtr,ConfigPtr,ConfigPtr->BaseAddr) != XST_SUCCESS)
//	      return FALSE;

	 XCanPs_EnterMode(&Can[cannode], XCANPS_MODE_CONFIG);
	 while(XCanPs_GetMode(&Can[cannode]) != XCANPS_MODE_CONFIG);

	 XCanPs_SetBaudRatePrescaler(&Can[cannode], sCandrvTiming[speed].pre-1);
	 XCanPs_SetBitTiming(&Can[cannode], sCandrvTiming[speed].sjw,sCandrvTiming[speed].tseg2-1,sCandrvTiming[speed].tseg1-1);
//*/
	return TRUE;
}


u32 TxFrame[4];            //
u32 RxFrame[4];            //
UBYTE SendIndex[CANDRV_MAX_CANNODE][MAX_FIFO];
UBYTE ucWaitTickLable, ucSendTickLable;
//***************************************************************************
// send cob and/or update data info if inhibited

BOOL candrv_sendcob(UWORD cannode, volatile CANDRV_COB * psTxCob)
{

    TXMBOXPARAMS * psloctxwait;
    UBYTE i;

    assert(cannode<CANDRV_MAX_CANNODE);

    if(XCanPs_IsTxFifoFull(&Can[cannode]))
 	   return FALSE;

    if((uwCanFlags[cannode]&CAN_PORTAVAILABLE)==0)
        return FALSE;

        // if disabled or going to shutdown then exit
    if((uwCanFlags[cannode]&CAN_SHUTTINGDOWN) || (uwCanDrvGlobalFlags[cannode]&CANDRV_GF_RUN)==0)
        return FALSE;

        // optimize for time
    psloctxwait=&sTxAwaitingCobs[cannode][0];

        // if inhibited then seek it in the list
    if(psTxCob->flags.bInhibited)//if(psTxCob->flags&CANDRV_F_INHIBITED)       //
    {
    	for(;;)
        {
                // first seek it to see if it's already on the list
            for(i=0;i<MAX_TXLIST_COBS;i++)            //
                if(psloctxwait[i].psTxCob==psTxCob)
                    break;

                // if not go to next step
            if(i>=MAX_TXLIST_COBS)
                break;

                // if found and still there,
            DISABLE_IRQ();
            if(psloctxwait[i].psTxCob==psTxCob)
            {
                    // if it's just being writing to queue or it's already updating then exit with error
            	if(psTxCob->flags.bTxQueuing || psTxCob->flags.bTxUpdating)//if(psTxCob->flags&(CANDRV_F_TXQUEUING|CANDRV_F_TXUPDATING))       //
                {
                    RESTORE_IRQ();
                    return FALSE;
                }
                    // prevent queuing of the cob during data update        //
            	psTxCob->flags.bTxUpdating = TRUE;//fast_atomic_set_bits(psTxCob->flags, CANDRV_F_TXUPDATING);

                    // then exit loop as found valid
                RESTORE_IRQ();
                break;
            }
            RESTORE_IRQ();
        }
    }
    else
        i=MAX_TXLIST_COBS;

        // if not found or not inhibited then seek and lock an empty location
    if(i>=MAX_TXLIST_COBS)    //
    {
            // preset queuing prevention
    	psTxCob->flags.bTxUpdating = TRUE;//fast_atomic_set_bits(psTxCob->flags, CANDRV_F_TXUPDATING);

        for(;;)
        {
                // first seek an empty location
            for(i=0;i<MAX_TXLIST_COBS;i++)
                if(psloctxwait[i].psTxCob==NULL)
                    break;

                // list full
            if(i>=MAX_TXLIST_COBS)
                return FALSE;

                // if found try to lock it
            DISABLE_IRQ();
            if(psloctxwait[i].psTxCob==NULL)
                psloctxwait[i].psTxCob=psTxCob;
            RESTORE_IRQ();

                // if locked successful
            if(psloctxwait[i].psTxCob==psTxCob)
                break;
        }
    }
    //
    SendIndex[cannode][ucWaitTickLable] = i;
    ucWaitTickLable++;
    if(ucWaitTickLable>=MAX_FIFO) ucWaitTickLable=0;
    // now, if inhibited, just let timer elapse for msg start
    DISABLE_IRQ();
    if(psTxCob->flags.bInhibited)//if(psTxCob->flags&CANDRV_F_INHIBITED)
    {
    	psTxCob->flags.bTxReq = TRUE;//psTxCob->flags|=CANDRV_F_TXREQ;
    	psTxCob->flags.bTxOK = FALSE;
    	psTxCob->flags.bTxUpdating =FALSE;//psTxCob->flags&=~(CANDRV_F_TXOK|CANDRV_F_TXUPDATING);
        RESTORE_IRQ();

        return TRUE;
    }
    RESTORE_IRQ();

        // otherwise send immediately
    psloctxwait[i].uwInhibitTimer = psTxCob->inhibit;

        // set tx queuing
    psTxCob->flags.bTxReq = TRUE;
    psTxCob->flags.bTxUpdating = TRUE;//fast_atomic_set_bits(psTxCob->flags, CANDRV_F_TXREQ|CANDRV_F_TXQUEUING);

        // clear TXOK flag and callback
    psTxCob->flags.bTxOK = FALSE;
    psTxCob->flags.bTxWCallback = FALSE;//fast_atomic_clear_bits(psTxCob->flags, CANDRV_F_TXOK|CANDRV_F_TXWCB);

#ifdef _CANDRV_EXTD
	 if(psTxCob->id&CANDRV_ID_EXTD)
		 TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)((psTxCob->id>>18)&0x7FF), 1, 1, (u32)(psTxCob->id)&0xFFFF, 0);
	 else
		 TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)(psTxCob->id&0x7FF), 0, 0, 0, 0);
#else
	 TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)(psTxCob->id&0x7FF), 0, 0, 0, 0);
#endif
	 TxFrame[1] = (u32)XCanPs_CreateDlcValue((u32)(psTxCob->len));

	 TxFrame[2] =  (((u32)psTxCob->d[1])<<16)|(psTxCob->d[0]);
	 TxFrame[3] =  (((u32)psTxCob->d[3])<<16)|(psTxCob->d[2]);
	 XCanPs_Send(&Can[cannode], TxFrame);        //

        // release msg
	 psTxCob->flags.bTxUpdating = FALSE;//fast_atomic_clear_bits(psTxCob->flags, CANDRV_F_TXUPDATING);

        // if blocking
//    if(psTxCob->flags&(CANDRV_F_TXBLOCK|CANDRV_F_TXBLOCKERR))
//    while(!(psTxCob->flags&CANDRV_F_TXOK))
//   {
//    	if((uwCanDrvGlobalFlags[cannode]&(CANDRV_GF_ERRPASV|CANDRV_GF_BUSOFF)) && !(psTxCob->flags&CANDRV_F_TXBLOCKERR))
//			return FALSE;
//   }

    return TRUE;
}


//***************************************************************************
// send cob with callback, inhibit time not managed
BOOL candrv_sendcobwcb(UWORD cannode, volatile CANDRV_COB * psTxCob, void (* pfCallBack)(CANDRV_COB *))
{

    TXMBOXPARAMS * psloctxwait;
    UBYTE i,j;

    assert(cannode<CANDRV_MAX_CANNODE);

    if(XCanPs_IsTxFifoFull(&Can[cannode]))
    	return FALSE;

    if((uwCanFlags[cannode]&CAN_PORTAVAILABLE)==0)
        return FALSE;

        // if disabled or going to shutdown then exit
    if((uwCanFlags[cannode]&CAN_SHUTTINGDOWN) || (uwCanDrvGlobalFlags[cannode]&CANDRV_GF_RUN)==0)
        return FALSE;

        // optimize for time
    psloctxwait=&sTxAwaitingCobs[cannode][0];

        // seek and lock an empty location
        // preset queuing prevention
    psTxCob->flags.bTxUpdating = TRUE;//fast_atomic_set_bits(psTxCob->flags, CANDRV_F_TXUPDATING);

    for(;;)
    {
            // first seek an empty location
        for(i=0;i<MAX_TXLIST_COBS;i++)
            if(psloctxwait[i].psTxCob==NULL)
                break;

            // list full
        if(i>=MAX_TXLIST_COBS)
            return FALSE;

            // if found try to lock it
        DISABLE_IRQ();
        if(psloctxwait[i].psTxCob==NULL)
            psloctxwait[i].psTxCob=psTxCob;
        RESTORE_IRQ();

            // if locked successful
        if(psloctxwait[i].psTxCob==psTxCob)
            break;
    }

        // seek and lock an empty location
        // for tx callback registering
    for(;;)
    {
            // first seek an empty location
        for(j=0;j<MAX_TXCALLBACKS;j++)
            if(pfTxCobCallBackVect[j]==NULL)
                break;

            // list full
        if(j>=MAX_TXCALLBACKS)
        {
                // release locked COB and exit
            psloctxwait[i].psTxCob=NULL;
            return FALSE;
        }

            // if found try to lock it
        DISABLE_IRQ();
        if(pfTxCobCallBackVect[j]==NULL)
            pfTxCobCallBackVect[j]=pfCallBack;
        RESTORE_IRQ();

            // if locked successful
        if(pfTxCobCallBackVect[j]==pfCallBack)
            break;
    }

    SendIndex[cannode][ucWaitTickLable]=i;
    ucWaitTickLable++;
    if(ucWaitTickLable>=MAX_FIFO) ucWaitTickLable = 0;

        // set callback position
    psloctxwait[i].uwInhibitTimer = j;     //

        // set tx queuing and registered callback
    psTxCob->flags.bTxReq = TRUE;
    psTxCob->flags.bTxQueuing = TRUE;
    psTxCob->flags.bTxWCallback = TRUE;//fast_atomic_set_bits(psTxCob->flags, CANDRV_F_TXREQ|CANDRV_F_TXQUEUING|CANDRV_F_TXWCB);

        // clear TXOK flag and inhibit
    psTxCob->flags.bTxOK = FALSE;
    psTxCob->flags.bInhibited = FALSE;//fast_atomic_clear_bits(psTxCob->flags, CANDRV_F_TXOK|CANDRV_F_INHIBITED);


#ifdef _CANDRV_EXTD
	 if(psTxCob->id&CANDRV_ID_EXTD)
		 TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)((psTxCob->id>>18)&0x7FF), 1, 1, (u32)(psTxCob->id)&0xFFFF, 0);
	 else
		 TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)(psTxCob->id&0x7FF), 0, 0, 0, 0);
#else
		 TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)(psTxCob->id&0x7FF), 0, 0, 0, 0);
#endif

	 TxFrame[1] = (u32)XCanPs_CreateDlcValue((u32)(psTxCob->len));

	 TxFrame[2] = (((u32)psTxCob->d[1])<<16)|(psTxCob->d[0]);
	 TxFrame[3] = (((u32)psTxCob->d[3])<<16)|(psTxCob->d[2]);


//	 while (XCanPs_IsTxFifoFull(&Can[cannode]) == TRUE);

	 XCanPs_Send(&Can[cannode], TxFrame);

        // release msg
	 psTxCob->flags.bTxUpdating=FALSE;//fast_atomic_clear_bits(psTxCob->flags, CANDRV_F_TXUPDATING);

        // if blocking
//    if(psTxCob->flags&(CANDRV_F_TXBLOCK|CANDRV_F_TXBLOCKERR))
//    while(!(psTxCob->flags&CANDRV_F_TXOK))
//    {
//		if((uwCanDrvGlobalFlags[cannode]&(CANDRV_GF_ERRPASV|CANDRV_GF_BUSOFF)) && !(psTxCob->flags&CANDRV_F_TXBLOCKERR))
//			return FALSE;
//    }

    return TRUE;
}

//***************************************************************************
// set periodic (automatic) and highest priority sending of COB (period = [usec])
u32 ulPeriodSendBuffer[4];
//
#ifdef CFG_CANDRV_PERIODICCOB
BOOL candrv_setperiodiccob(UWORD cannode, CANDRV_ID id, ULONG period)
{
     assert(cannode<CANDRV_MAX_CANNODE);

    if((uwCanFlags[cannode]&CAN_PORTAVAILABLE)==0)
        return FALSE;

        // if period is zero then disable sending
    if(period==0)
    {
        sCanDrvSyncCobStat[cannode].uwRestart=0;
        return TRUE;
    }

#ifdef _CANDRV_EXTD
	 if(id&CANDRV_ID_EXTD)
		 ulPeriodSendBuffer[0] = (u32)XCanPs_CreateIdValue((u32)((id>>18)&0x7FF), 1, 1, (u32)(id)&0xFFFF, 0);
	 else
		 ulPeriodSendBuffer[0] = (u32)XCanPs_CreateIdValue((u32)(id&0x7FF), 0, 0, 0, 0);
#else
		 ulPeriodSendBuffer[0] = (u32)XCanPs_CreateIdValue((u32)(id&0x7FF), 0, 0, 0, 0);
#endif

	 ulPeriodSendBuffer[1] = (u32)XCanPs_CreateDlcValue(0);

	 ulPeriodSendBuffer[2]=0;
	 ulPeriodSendBuffer[3]=0;

    // set counter to 1, then next RT calling will send first COB
    sCanDrvSyncCobStat[cannode].uwCounter=1;

    // calculate number of realtime task ticks
    sCanDrvSyncCobStat[cannode].uwRestart=(UWORD)(period/(1000000/REALTIME_TASK_FREQ));

    return TRUE;
}

BOOL candrv_periodsendID(UWORD cannode)
{
   if(XCanPs_IsTxFifoFull(&Can[cannode]))
	   return FALSE;

   SendIndex[cannode][ucWaitTickLable] = MAX_FIFO;
   ucWaitTickLable++;
   if(ucWaitTickLable>=MAX_FIFO) ucWaitTickLable = 0;
	XCanPs_Send(&Can[cannode], ulPeriodSendBuffer);   //

}


#if !CAN_DEBUG
// Node 0 TX
static void SendHandler0(void *CallBackRef)
{

	UWORD moidx;
    volatile CANDRV_COB * pcob;

    {
    	moidx = SendIndex[NODE0][ucSendTickLable];
    	ucSendTickLable++;
    	if(ucSendTickLable>=MAX_FIFO) ucSendTickLable=0;

    	if(moidx==MAX_FIFO) return;
            // retrieve usr cob destination
        pcob=sTxAwaitingCobs[NODE0][moidx].psTxCob;

            // reset TX req and set TXOK
        pcob->flags.bTxOK = TRUE;   //fast_atomic_set_bits(pcob->flags, CANDRV_F_TXOK);

            // clear im flags
        pcob->flags.bTxReq = FALSE;
        pcob->flags.bTxQueuing = FALSE;//fast_atomic_clear_bits(pcob->flags, CANDRV_F_TXREQ|CANDRV_F_TXQUEUING);

            // if callback
        if(pcob->flags.bTxWCallback)//if(pcob->flags & CANDRV_F_TXWCB)
        {
                // spawn callback
            (pfTxCobCallBackVect[sTxAwaitingCobs[NODE0][moidx].uwInhibitTimer])(pcob);

                // and free locations
            pfTxCobCallBackVect[sTxAwaitingCobs[NODE0][moidx].uwInhibitTimer]=NULL;
            sTxAwaitingCobs[NODE0][moidx].psTxCob = 0;
            sTxAwaitingCobs[NODE0][moidx].psTxCob = NULL;
        }
        else if(pcob->inhibit > 0 && (uwCanFlags[NODE0]&CAN_INHIBITENABLED))
        {
                // if needed inhibit time then set flag and leave timer irq
                // make further processing
        	pcob->flags.bInhibited = TRUE;//fast_atomic_set_bits(pcob->flags, CANDRV_F_INHIBITED);
        }
        else
                // otherwise free location
            sTxAwaitingCobs[NODE0][moidx].psTxCob = NULL;

    }

}

//***************************************************************************
// Node 1 TX

static void SendHandler1(void *CallBackRef)
{
	UWORD moidx;
    volatile CANDRV_COB * pcob;

    {
    	moidx = SendIndex[NODE1][ucSendTickLable];
    	ucSendTickLable++;
    	if(ucSendTickLable>=MAX_FIFO) ucSendTickLable=0;

    	if(moidx==MAX_FIFO) return;
            // retrieve usr cob destination
        pcob=sTxAwaitingCobs[NODE1][moidx].psTxCob;

            // reset TX req and set TXOK
        pcob->flags.bTxOK = TRUE;   //fast_atomic_set_bits(pcob->flags, CANDRV_F_TXOK);

            // clear im flags
        pcob->flags.bTxReq = FALSE;
        pcob->flags.bTxQueuing = FALSE;//fast_atomic_clear_bits(pcob->flags, CANDRV_F_TXREQ|CANDRV_F_TXQUEUING);

            // if callback
        if(pcob->flags.bTxWCallback)//if(pcob->flags & CANDRV_F_TXWCB)
        {
                // spawn callback
            (pfTxCobCallBackVect[sTxAwaitingCobs[NODE1][moidx].uwInhibitTimer])(pcob);

                // and free locations
            pfTxCobCallBackVect[sTxAwaitingCobs[NODE1][moidx].uwInhibitTimer]=NULL;
            sTxAwaitingCobs[NODE1][moidx].psTxCob = 0;
            sTxAwaitingCobs[NODE1][moidx].psTxCob = NULL;
        }
        else if(pcob->inhibit > 0 && (uwCanFlags[NODE1]&CAN_INHIBITENABLED))
        {
                // if needed inhibit time then set flag and leave timer irq
                // make further processing
        	pcob->flags.bInhibited = TRUE;//fast_atomic_set_bits(pcob->flags, CANDRV_F_INHIBITED);
        }
        else
                // otherwise free location
            sTxAwaitingCobs[NODE1][moidx].psTxCob = NULL;

    }

}


static void RecvHandler0(void *CallBackRef)
{

    UWORD moidx;
    ULONG cobid;
    BOOL rtrflag;

    volatile CANDRV_COB * pcob;
    CANDRV_ID revID ;

    XCanPs_Recv(&Can[NODE0], RxFrame);
#ifdef _CANDRV_EXTD
    if((RxFrame[0]&(1<<19))&&(RxFrame[0]&(1<<20)))      //
    {
    	cobid = ((ULONG)(RxFrame[0]&0xFFE00000000)>>3)|((ULONG)(RxFrame[0]&0x7FFFE)>>1);
    	revID = cobid|(0x60000000);
    	rtrflag = RxFrame[0]&0x0001;      //
    }
    else
#endif
    {
    	cobid = (RxFrame[0])>>21;
    	revID = cobid;
    	rtrflag = (RxFrame[0]>>20)&0x0001;
    }

    if(rtrflag)
    {
    	for(moidx=0; moidx < MAX_RXLIST_COBS; moidx++)
    	{
    		if((sCanDrvRxList[NODE0][moidx].psRxCob->flags.bRTR)&&(sCanDrvRxList[NODE0][moidx].psRxCob->id == revID))//if((sCanDrvRxList[NODE0][moidx].psRxCob->flags&CANDRV_F_RTR)&&(sCanDrvRxList[NODE0][moidx].psRxCob->id == revID))
    		break;
    	}
    }
    else
    {
    	for(moidx=0; moidx < MAX_RXLIST_COBS; moidx++)
    	{
    		if(sCanDrvRxList[NODE0][moidx].psRxCob->id == revID)
    		break;
    	}
    }
    if(moidx>=MAX_RXLIST_COBS)          //
    	return;

    if(!(uwCanFlags[NODE0]&CAN_SHUTTINGDOWN))
    {
    	pcob=sCanDrvRxList[NODE0][moidx].psRxCob;
		pcob->len=(RxFrame[1]>>28)&0x000f;

		pcob->d[0]= RxFrame[2]&0xFFFF;
		pcob->d[1]= RxFrame[2]>>16;
		pcob->d[2]= RxFrame[3]&0xFFFF;
		pcob->d[3]= RxFrame[3]>>16;

		pcob->inhibit = RxFrame[1]&0x00FF;      //

       // check for overrun, then set newcob flag
	   if(pcob->flags.bNewRxCob)//if(pcob->flags&CANDRV_F_NEWCOB)
		   pcob->flags.bOverRun = TRUE;//pcob->flags|=CANDRV_F_OVERRUN;

	   pcob->flags.bNewRxCob = TRUE;//pcob->flags|=CANDRV_F_NEWCOB;
		// process callback, if any
        if(sCanDrvRxList[NODE0][moidx].pfCallBack)
			(*(sCanDrvRxList[NODE0][moidx].pfCallBack))(pcob);

		// Notify successfully packet processed
#ifdef CFG_CANDRV_STATUSSIGNAL
		bSysStatPnlMgrPacketReceived=TRUE;
#endif
    }

}


static void RecvHandler1(void *CallBackRef)
{
    UWORD moidx;
    ULONG cobid;
    BOOL rtrflag;

    volatile CANDRV_COB * pcob;
    CANDRV_ID revID ;

    XCanPs_Recv(&Can[NODE1], RxFrame);
#ifdef _CANDRV_EXTD
    if((RxFrame[0]&(1<<19))&&(RxFrame[0]&(1<<20)))      //
    {
    	cobid = ((ULONG)(RxFrame[0]&0xFFE00000000)>>3)|((ULONG)(RxFrame[0]&0x7FFFE)>>1);
    	revID = cobid|(0x60000000);
    	rtrflag = RxFrame[0]&0x0001;      //
    }
    else
#endif
    {
    	cobid = (RxFrame[0])>>21;
    	revID = cobid;
    	rtrflag = (RxFrame[0]>>20)&0x0001;
    }

    if(rtrflag)
    {
    	for(moidx=0; moidx < MAX_RXLIST_COBS; moidx++)
    	{
    		if((sCanDrvRxList[NODE1][moidx].psRxCob->flags.bRTR)&&(sCanDrvRxList[NODE1][moidx].psRxCob->id == revID))
    		break;
    	}
    }
    else
    {
    	for(moidx=0; moidx < MAX_RXLIST_COBS; moidx++)
    	{
    		if(sCanDrvRxList[NODE1][moidx].psRxCob->id == revID)
    		break;
    	}
    }
    if(moidx>=MAX_RXLIST_COBS)          //
    	return;

    if(!(uwCanFlags[NODE1]&CAN_SHUTTINGDOWN))
    {
    	pcob=sCanDrvRxList[NODE1][moidx].psRxCob;
		pcob->len=(RxFrame[1]>>28)&0x000f;

		pcob->d[0]= RxFrame[2]&0xFFFF;
		pcob->d[1]= RxFrame[2]>>16;
		pcob->d[2]= RxFrame[3]&0xFFFF;
		pcob->d[3]= RxFrame[3]>>16;

		pcob->inhibit = RxFrame[1]&0x00FF;      //

       // check for overrun, then set newcob flag
	   if(pcob->flags.bNewRxCob)//if(pcob->flags&CANDRV_F_NEWCOB)
		   pcob->flags.bOverRun = TRUE;//pcob->flags|=CANDRV_F_OVERRUN;

	   pcob->flags.bNewRxCob = TRUE;//pcob->flags|=CANDRV_F_NEWCOB;
		// process callback, if any
        if(sCanDrvRxList[NODE1][moidx].pfCallBack)
			(*(sCanDrvRxList[NODE1][moidx].pfCallBack))(pcob);

		// Notify successfully packet processed
#ifdef CFG_CANDRV_STATUSSIGNAL
		bSysStatPnlMgrPacketReceived=TRUE;
#endif
    }
}

#endif


static void inhibit_time_manager(void)
{
	UWORD i=0;
    UWORD * uwPtr;
    TXMBOXPARAMS * ptxp;
    BOOL tempbool;
        // update free running timer for timing check of background tasks
    uwCanDrvTimer100us++;

    UWORD uwMatchValue = TIMER_CCU3_COUNT + 1000;
    // update itself for next irq, not from T8 as this irq could be triggered not accurately due to low ILVL
    Timer_CCStart(TIMER_CCU3_ID,uwMatchValue);

        // if node 0 active process data
    if(uwCanFlags[NODE0]&CAN_INHIBITENABLED)
            // foreach valid element
        for(i=0,ptxp=sTxAwaitingCobs[NODE0];i<MAX_TXLIST_COBS;i++,ptxp++)
            if(ptxp->psTxCob)
                    // if inhibited and not being updated and not callback
            	if(ptxp->uwInhibitTimer>0 && (ptxp->psTxCob->flags.bTxUpdating||ptxp->psTxCob->flags.bTxWCallback)==0)//if(ptxp->uwInhibitTimer>0 && (ptxp->psTxCob->flags&(CANDRV_F_TXUPDATING|CANDRV_F_TXWCB))==0)
                    if(--ptxp->uwInhibitTimer==0)
                    {
                    	tempbool = ptxp->psTxCob->flags.bInhibited;
                    	memset(&(ptxp->psTxCob->flags),0,sizeof(ptxp->psTxCob->flags));
                    	ptxp->psTxCob->flags.bInhibited = tempbool;//ptxp->psTxCob->flags &= ~CANDRV_F_INHIBITED;
                            // if TX REQ was pending
                    	if(ptxp->psTxCob->flags.bTxReq)//if(ptxp->psTxCob->flags&CANDRV_F_TXREQ)
                        {
                                // send
                    		ptxp->psTxCob->flags.bTxQueuing;//ptxp->psTxCob->flags |= CANDRV_F_TXQUEUING;

						#ifdef _CANDRV_EXTD
                    		if(ptxp->psTxCob->id&CANDRV_ID_EXTD)
								 TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)((ptxp->psTxCob->id>>18)&0x7FF), 1, 1, (u32)(ptxp->psTxCob->id)&0xFFFF, 0);
							 else
								 TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)(ptxp->psTxCob->id&0x7FF), 0, 0, 0, 0);
						#else
								 TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)(ptxp->psTxCob->id&0x7FF), 0, 0, 0, 0);
						#endif
							 TxFrame[1] = (u32)XCanPs_CreateDlcValue((u32)(ptxp->psTxCob->len));

							 TxFrame[2]=(((u32)ptxp->psTxCob->d[0])<<16)|(ptxp->psTxCob->d[1]);
							 TxFrame[3]=(((u32)ptxp->psTxCob->d[2])<<16)|(ptxp->psTxCob->d[3]);

							 while (XCanPs_IsTxFifoFull(&Can[NODE0]) == TRUE);

							 XCanPs_Send(&Can[NODE0], TxFrame);

                                // and set again inhibit timer
                            ptxp->uwInhibitTimer = ptxp->psTxCob->inhibit;
                        }
                            // otherwise remove MO from the list
                        else
                            ptxp->psTxCob=NULL;

                    }

        // if node 1 active process data
    if(uwCanFlags[NODE1]&CAN_INHIBITENABLED)
            // foreach valid element
        for(i=0,ptxp=sTxAwaitingCobs[NODE1];i<MAX_TXLIST_COBS;i++,ptxp++)
            if(ptxp->psTxCob)
                    // if inhibited and not being updated and not callback
            	if(ptxp->uwInhibitTimer>0 && (ptxp->psTxCob->flags.bTxUpdating||ptxp->psTxCob->flags.bTxWCallback)==0)//if(ptxp->uwInhibitTimer>0 && (ptxp->psTxCob->flags&(CANDRV_F_TXUPDATING|CANDRV_F_TXWCB))==0)
            	     if(--ptxp->uwInhibitTimer==0)
            	     {
            	          tempbool = ptxp->psTxCob->flags.bInhibited;
            	          memset(&(ptxp->psTxCob->flags),0,sizeof(ptxp->psTxCob->flags));
            	          ptxp->psTxCob->flags.bInhibited = tempbool;//ptxp->psTxCob->flags &= ~CANDRV_F_INHIBITED;
            	                            // if TX REQ was pending
            	          if(ptxp->psTxCob->flags.bTxReq)//if(ptxp->psTxCob->flags&CANDRV_F_TXREQ)
            	          {
            	                                // send
            	               ptxp->psTxCob->flags.bTxQueuing;//ptxp->psTxCob->flags |= CANDRV_F_TXQUEUING;

            				#ifdef _CANDRV_EXTD
            					if(ptxp->psTxCob->id&CANDRV_ID_EXTD)
            							TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)((ptxp->psTxCob->id>>18)&0x7FF), 1, 1, (u32)(ptxp->psTxCob->id)&0xFFFF, 0);
            					else
            							TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)(ptxp->psTxCob->id&0x7FF), 0, 0, 0, 0);
            				#else
            							TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)(ptxp->psTxCob->id&0x7FF), 0, 0, 0, 0);
            				#endif
            							TxFrame[1] = (u32)XCanPs_CreateDlcValue((u32)(ptxp->psTxCob->len));

            							TxFrame[2]=(((u32)ptxp->psTxCob->d[0])<<16)|(ptxp->psTxCob->d[1]);
            							TxFrame[3]=(((u32)ptxp->psTxCob->d[2])<<16)|(ptxp->psTxCob->d[3]);

            							while (XCanPs_IsTxFifoFull(&Can[NODE1]) == TRUE);

            							XCanPs_Send(&Can[NODE1], TxFrame);

            	                                // and set again inhibit timer
            	                        ptxp->uwInhibitTimer = ptxp->psTxCob->inhibit;
            	            }
            	                            // otherwise remove MO from the list
            	            else
            	               ptxp->psTxCob=NULL;

                    }

    for(i=0;i<CANDRV_MAX_CANNODE;i++)
        if(uwCanDrvGlobalFlags[i]&CANDRV_GF_RUN)
        {
                // get pointer for atomic operations
            uwPtr=&uwCanDrvGlobalFlags[i];

                // check for error warning
            if(XCanPs_GetStatus(&Can[i])&XCANPS_SR_ERRWRN_MASK)      //
            {
                fast_atomic_set_bits(*uwPtr,CANDRV_GF_ERRPASV);
            }
            else
            {
                fast_atomic_clear_bits(*uwPtr,CANDRV_GF_ERRPASV);
            }

                // if bus off
            if(XCanPs_GetStatus(&Can[i])&XCANPS_SR_ESTAT_MASK == 0x100)      //
            {
                fast_atomic_set_bits(*uwPtr,CANDRV_GF_BUSOFF);

                XCanPs_Reset(&Can[i]);
                XCanPs_EnterMode(&Can[i], XCANPS_MODE_CONFIG);
                while(XCanPs_GetMode(&Can[i]) != XCANPS_MODE_CONFIG);
            }
            else
            {
                if(*uwPtr&CANDRV_GF_BUSOFF)
                    if(XCanPs_GetStatus(&Can[i])&XCANPS_SR_NORMAL_MASK)       //
                    {
                        fast_atomic_clear_bits(*uwPtr,CANDRV_GF_BUSOFF);
                    }
            }
            // clear all errors
        }
}

//***************************************************************************
// CAN Initialization

BOOL candrv_init(void)
{
    UWORD ct=0,ct1;

    for(ct=0;ct<CANDRV_MAX_CANNODE;ct++)

#ifndef _HW_AXS

#ifndef _HW_DC
        uwCanFlags[ct]=CAN_PORTAVAILABLE;
#else
        uwCanFlags[ct]=0;

    if((sGlbOptAvailable.ulHardwareOpt&(HWUNITID_CAN_PORT_0|HWUNITID_CAN_PORT_1)) == 0)
        return TRUE;

    if(sGlbOptAvailable.ulHardwareOpt&HWUNITID_CAN_PORT_0)
        uwCanFlags[NODE0]|=CAN_PORTAVAILABLE;

    if(sGlbOptAvailable.ulHardwareOpt&HWUNITID_CAN_PORT_1)
        uwCanFlags[NODE1]|=CAN_PORTAVAILABLE;
#endif

#else // _HW_AXS
    uwCanFlags[ct]=CAN_PORTAVAILABLE;
#endif

     // setup all nodes
    for(ct=0;ct<CANDRV_MAX_CANNODE;ct++)
        if(uwCanFlags[ct]&CAN_PORTAVAILABLE)
    {
            // global flags and phy error
    	uwCanDrvGlobalFlags[ct]=CANDRV_GF_DEFAULT;        //

            // disable all rx lists
        for(ct1=0;ct1<MAX_RXLIST_COBS;ct1++)
        {
            sCanDrvRxList[ct][ct1].psRxCob=NULL;      //
        }

        // disable all rx lists and allocate tx MO
        for(ct1=0;ct1<MAX_TXLIST_COBS;ct1++)
        {
        	 sTxAwaitingCobs[ct][ct1].psTxCob=NULL;            //
        }
            // cob disabled
        sCanDrvSyncCobStat[ct].uwRestart = 0;
    }

        // erase tx callbacks array
    for(ct=0;ct<MAX_TXCALLBACKS;ct++)
        pfTxCobCallBackVect[ct]=NULL;		//

    if(uwCanFlags[NODE0]&CAN_PORTAVAILABLE)     //
    	Can_Init(NODE0);

    if(uwCanFlags[NODE1]&CAN_PORTAVAILABLE)
    	Can_Init(NODE1);

	Timer_CCSet(TIMER_CCU3_ID, SYSAPPIL_MULTICANCTRL, inhibit_time_manager);       //
    UWORD uwMatchValue = TIMER_CCU3_COUNT + 1000;           //
    Timer_CCStart(TIMER_CCU3_ID,uwMatchValue);              //

        // disable callback for sync sending
    pfCanDrvSyncCobCallBack = NULL;

        // install realtime task for periodic COBS
#ifdef CFG_CANDRV_PERIODICCOB
    if(!TaskSched_AddRTTask(&candrv_RT_periodiccobprocessing, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0))
        return FALSE;
#endif

	return TRUE;
}

//***************************************************************************
// Test selected baudrate to check if it's supported here

BOOL candrv_testspeed(SWORD speed)
{
	if(speed<0||speed>=(sizeof(sCandrvTiming)/sizeof(CANDRV_TIMING)))
		return FALSE;
	
	return TRUE;
}



//***************************************************************************
// Add RX cob for rx event support, very hi priority are always kept
// in first array element

#ifndef _CANDRV_EXTD
BOOL candrv_addrxcob(UWORD cannode, CANDRV_COB * psRxCob, void (* pfCallBack)(CANDRV_COB *))
#else
BOOL candrv_addmrxcob(UWORD cannode, CANDRV_COB * psRxCob, ULONG mask, void (* pfCallBack)(CANDRV_COB *))
#endif
{
    UWORD i;
    BOOL bool0,bool2,bool3;
    assert(cannode<CANDRV_MAX_CANNODE);

    if((uwCanFlags[cannode]&CAN_PORTAVAILABLE)==0)
        return FALSE;

    for(i=0;i<MAX_RXLIST_COBS;i++)
         if(sCanDrvRxList[cannode][i].psRxCob==psRxCob)         //
             break;

		// if not seek and lock an empty location
	if(i>=MAX_RXLIST_COBS)      //
	{
		Os_BeginCriticalSection(OS_CRITSECT_GLOBAL);

		if(psRxCob->flags.bRxVHPR) //if(psRxCob->flags&CANDRV_F_RX_VHPR)          //
		{
				// already setup a very hi priority
			if(sCanDrvRxList[cannode][0].psRxCob)      //
			{
				Os_EndCriticalSection(OS_CRITSECT_GLOBAL);
				return FALSE;
			}
			i=0;
		}
		else     //
		{
			for(i=1;i<MAX_RXLIST_COBS;i++)           //
				if(sCanDrvRxList[cannode][i].psRxCob==NULL)
					break;

				// rx list full
			if(i>=MAX_RXLIST_COBS)	//
			{
				Os_EndCriticalSection(OS_CRITSECT_GLOBAL);
				return FALSE;
			}
		}

			// fillup data (and then lock entry list)
		sCanDrvRxList[cannode][i].psRxCob=psRxCob;      //

		Os_EndCriticalSection(OS_CRITSECT_GLOBAL);

//		reconfig=FALSE;     //
	}
//	else
//		reconfig=TRUE;     //

		// set callback if any
	sCanDrvRxList[cannode][i].pfCallBack=pfCallBack;

//	XCanPs_IntrEnable(&Can[i],XCANPS_IXR_RXOK_MASK);       //

	bool0 = psRxCob->flags.bRTR;
	bool2 = psRxCob->flags.bRxHiPriority;
	bool3 = psRxCob->flags.bRxVHPR;
	memset(&(psRxCob->flags),0,sizeof(psRxCob->flags));
	psRxCob->flags.bRTR = bool0;
	psRxCob->flags.bRxHiPriority = bool2;
	psRxCob->flags.bRxVHPR = bool3;//psRxCob->flags=CANDRV_F_DEFAULT | (psRxCob->flags&(CANDRV_F_RTR|CANDRV_F_RXHIPRIOR|CANDRV_F_RX_VHPR));


    return TRUE;
}

//***************************************************************************
// Delete RX cob for rx event support
BOOL candrv_deleterxcob(UWORD cannode, CANDRV_COB * psRxCob)
{
    UBYTE i;

    assert(cannode<CANDRV_MAX_CANNODE);

    if((uwCanFlags[cannode]&CAN_PORTAVAILABLE)==0)
        return FALSE;

    for(i=0;i<MAX_RXLIST_COBS;i++)
        if(sCanDrvRxList[cannode][i].psRxCob==psRxCob)
            break;

        // not found in the rx list
    if(i>=MAX_RXLIST_COBS)
        return FALSE;

        // free entry list
    sCanDrvRxList[cannode][i].psRxCob=NULL;
    sCanDrvRxList[cannode][i].pfCallBack=NULL;

    return TRUE;
}

//***************************************************************************
// Request RTR from the actual RX object

BOOL candrv_requestrxcob(UWORD cannode, CANDRV_COB * psRxCob)
{
    UWORD i;

    assert(cannode<CANDRV_MAX_CANNODE);

    if((uwCanFlags[cannode]&CAN_PORTAVAILABLE)==0)
        return FALSE;

    for(i=0;i<MAX_RXLIST_COBS;i++)
        if(sCanDrvRxList[cannode][i].psRxCob==psRxCob)
            break;

        // not found in the rx list
    if(i>=MAX_RXLIST_COBS)
        return FALSE;

    return TRUE;
}

//***************************************************************************






//***************************************************************************
// install callback for periodic sending of COB

BOOL candrv_instcallbackperiodiccob(void (* pfCallBack)(UWORD))
{
//    _atomic_(0);
    pfCanDrvSyncCobCallBack = pfCallBack;
//    _endatomic_();

    return TRUE;
}
#endif

//***************************************************************************
// Data conversion

ULONG candrv_get_bitfield(CANDRV_PCOB pc,UWORD st,UWORD sz)
{
	UWORD ns,ct,nt,*pd;
	volatile UWORD *pld;
    ULONG dest=0l;
	
	pd=(UWORD *)&dest;
	pld=pc->d;
	ns=st&15;
	for(ct=(st>>4),nt=st;nt<=(st+sz);ct++,nt+=16)
        if(ns>0)
		*pd++=(pld[ct]>>ns)|(pld[ct+1]<<(16-ns));
        else
		    *pd++=pld[ct];

	*--pd&=(1<<(sz&15))-1;

    return dest;
}

void candrv_put_bitfield(CANDRV_PCOB pc,UWORD st,UWORD sz,ULONG src)
{
	UWORD ns,ct,nt,*pd,tmp;
	volatile UWORD *pld;
	
	pd=(UWORD *)&src;
	pld=pc->d;
	ns=st&15;
	ct=st>>4;
	nt=(st+sz-1)>>4;
	
	if(nt>ct)
	{
		tmp=pld[ct]&((1<<ns)-1)|(pd[0]<<ns);
		for(;ct<nt;ct++,pd++)
		{
			pld[ct]=tmp;
            if(ns>0)
			tmp=(pd[0]>>(16-ns))|(pd[1]<<ns);
            else
    			tmp=pd[1];
		}
		nt=(UWORD)(1<<((st+sz)&15))-1;
		pld[ct]=tmp&nt|pld[ct]&(0xffff^nt);
	}
	else
	{
		nt=(0xffff>>(16-sz))<<st;
		pld[ct]=pld[ct]&(nt^0xffff)|(pd[0]<<st)&nt;
	}
}

//***************************************************************************
// PLC wrappers

#ifdef CFG_CANDRV_PLCWRAPPER
    // Setup baudrate
BOOL candrv_plc_setspeed(UWORD cannode, SWORD speed)
{
        // check right drive state
    if(!Os_IsInBackground() && !PlcIsInSlowTask())
        return FALSE;

    return candrv_setspeed(cannode, speed);
}

    // Add RX cob for rx event support
#ifndef _CANDRV_EXTD
BOOL candrv_plc_addrxcob(UWORD cannode, CANDRV_COB * psRxCob)
#else
BOOL candrv_plc_addrxcob(UWORD cannode, CANDRV_COB * psRxCob, ULONG mask)
#endif
{
    CANDRV_COB * psNearRxCob=(CANDRV_COB *)psRxCob;

        // check right drive state
    if(!Os_IsInBackground() && !PlcIsInSlowTask())
        return FALSE;

        // check near pointer
    if((HPVOID)psNearRxCob != psRxCob)
        return FALSE;

#ifndef _CANDRV_EXTD
    return candrv_addrxcob(cannode, psNearRxCob, NULL);
#else
    return candrv_addmrxcob(cannode, psNearRxCob, mask, NULL);
#endif
}

    // Delete RX cob for rx event support
BOOL candrv_plc_deleterxcob(UWORD cannode, CANDRV_COB * psRxCob)
{
    CANDRV_COB * psNearRxCob=(CANDRV_COB *)psRxCob;

        // check right drive state
    if(!Os_IsInBackground() && !PlcIsInSlowTask())
        return FALSE;

        // check near pointer
    if((HPVOID)psNearRxCob != psRxCob)
        return FALSE;

    return candrv_deleterxcob(cannode, psNearRxCob);
}

    // Request RTR from the actual RX object
BOOL candrv_plc_requestrxcob(UWORD cannode, CANDRV_COB * psRxCob)
{
    CANDRV_COB * psNearRxCob=(CANDRV_COB *)psRxCob;

        // check right drive state
    if(!Os_IsInBackground() && !PlcIsInSlowTask())
        return FALSE;

        // check near pointer
    if((HPVOID)psNearRxCob != psRxCob)
        return FALSE;

    return candrv_requestrxcob(cannode, psNearRxCob);
}

    // Node enable
BOOL candrv_plc_run(UWORD cannode)
{
        // check right drive state
    if(!Os_IsInBackground() && !PlcIsInSlowTask())
        return FALSE;

    return candrv_run(cannode);
}

    // Node disable
BOOL candrv_plc_stop(UWORD cannode)
{
        // check right drive state
    if(!Os_IsInBackground() && !PlcIsInSlowTask())
        return FALSE;

    return candrv_stop(cannode);
}

    // send cob and/or update data info if inhibited
BOOL candrv_plc_sendcob(UWORD cannode, CANDRV_COB * psTxCob)
{
    CANDRV_COB * psNearTxCob=(CANDRV_COB *)psTxCob;

        // check right drive state
    if(!Os_IsInBackground() && !PlcIsInSlowTask())
        return FALSE;

        // check near pointer
    if((HPVOID)psNearTxCob != psTxCob)
        return FALSE;

    return candrv_sendcob(cannode, (volatile CANDRV_COB *)psNearTxCob);
}

    // set periodic (automatic) and highest priority sending of COB (period = [usec])
BOOL candrv_plc_setperiodiccob(UWORD cannode, CANDRV_ID id, ULONG period)
{
        // check right drive state
    if(!Os_IsInBackground() && !PlcIsInSlowTask())
        return FALSE;

    return candrv_setperiodiccob(cannode, id, period);
}
#endif
#endif
