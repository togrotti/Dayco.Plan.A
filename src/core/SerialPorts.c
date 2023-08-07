/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SerialPorts.c                                              */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

/***************************** Include Files *******************************/

#include "xparameters.h"
#include "xplatform_info.h"
#include "xil_exception.h"
#include "xil_printf.h"

#ifdef _INFINEON_
#include <XE167F.H>
#include <intrins.h>
#endif

#include "xscugic.h"

#if XPAR_XUARTPS_NUM_INSTANCES
#include "xuartps.h"
#endif

#include <string.h>

#include "common\CommonDefines.h"

#ifndef _RD
#include "drive\AxM-E-Defines.h"
#ifdef _AXX_SYSAPP
#include "system\SysAppGlobals.h"
#include "system\SysAppInterruptLevels.h"
#else
#include "system\BootBlockGlobals.h"
#include "system\BootBlockInterruptLevels.h"
#endif
#define TMR_100NS           uwSysTimers100ns
#else
#include "RemDisp-Defines.h"
#define TMR_100NS           CC2_T8
#endif

#include "common\CommonUtility.h"

#include "SerialPorts.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

/////////////////////////////////////////////////////////////////////////////
//

//#define SERIALPORTS_DEBUG_TIMINGS

#ifdef SERIALPORTS_DEBUG_TIMINGS
#warning "SERIALPORTS_DEBUG_TIMINGS Active"
#endif

/////////////////////////////////////////////////////////////////////////////
//

#define SERIALPORTS_PEC_TRANSFERT_MAXIMUM_LENGTH    254

/////////////////////////////////////////////////////////////////////////////
// Serial Port U0C0: Support RS232, RS422, RS485

#ifdef _INFINEON_
#define XE167_COMM_TXD    P7_OUT_P3   // --> Tx Data Line
#define XE167_COMM_RXD    P7_IN_P4    // <-- Rx Data Line
#define XE167_COMM_RTS    P8_OUT_P4   // --> RTS Control Line
#define XE167_COMM_CTS    P8_IN_P5    // <-- CTS Control Line
#define XE167_COMM_TYP    P8_IN_P6    // <-- Port Type : 0 = RS232, 1 = RS422/RS485
#else
#ifndef _HW_DC
#define COMM_RTS_PIN      (MIO_PIN_BASE+15)   // --> RTS Control Line
#define COMM_CTS_PIN      (MIO_PIN_BASE+10)   // <-- CTS Control Line
#define COMM_TYP_PIN      (MIO_PIN_BASE+0 )   // <-- Port Type : 0 = RS232, 1 = RS422/RS485
#endif // _hw_dc
#endif // _infineon_

/////////////////////////////////////////////////////////////////////////////
//
#define SER_PORT_0_RX_BUFFER_SIZE   256
#define SER_PORT_0_TX_BUFFER_SIZE   256

/************************** Variable Definitions ***************************/
#ifdef XPAR_PS7_UART_0_DEVICE_ID
//#if XPAR_PS7_UART_0_DEVICE_ID >= SERIALPORTS_CN_LOWERPORT && XPAR_PS7_UART_0_DEVICE_ID <= SERIALPORTS_CN_UPPERPORT
#define SERIAL_PORT_0              0
#define SERIAL_PORT_0_DEVICE_ID    XPAR_PS7_UART_0_DEVICE_ID
#define SERIAL_PORT_0_INTR         XPS_UART0_INT_ID
//#endif
#endif

#ifdef XPAR_PS7_UART_1_DEVICE_ID
//#if XPAR_PS7_UART_1_DEVICE_ID >= SERIALPORTS_CN_LOWERPORT && XPAR_PS7_UART_1_DEVICE_ID <= SERIALPORTS_CN_UPPERPORT
#define SERIAL_PORT_1              1
#define SERIAL_PORT_1_DEVICE_ID    XPAR_PS7_UART_1_DEVICE_ID
#define SERIAL_PORT_1_INTR         XPS_UART1_INT_ID
//#endif
#endif

/////////////////////////////////////////////////////////////////////////////
//

// Serial Port 0#
#ifdef SERIAL_PORT_0
static XUartPs  Uart0;
static XUartPs *UartInstPtr0;

static UBYTE  ubPort0RxBuffer[ SER_PORT_0_RX_BUFFER_SIZE ];
static UWORD  uwPort0RxBufferWrIdx;
static UWORD  uwPort0RxBufferRdIdx;
static UBYTE  ubPort0TxBuffer[ SER_PORT_0_TX_BUFFER_SIZE ];
static BOOL   bPort0TxPecTransfert;
static UWORD  uwPort0TxBufferWrIdx;

static UWORD  uwPort0TimerEndTxCount;
//static UWORD  uwPort0TimerEndTxTicks;
static UWORD  uwPort0TimerEndTxTimer;

static BOOL   bPort0HalfDuplex;
static BOOL   bPort0HideRx;
static BOOL   bPort0Force422485;

static SERIAL_PORT_CALLBACK  pfnPort0RxCallback;
static SERIAL_PORT_CALLBACK  pfnPort0TxCallback;
#endif

// Serial Port 1#
#ifdef SERIAL_PORT_1
static XUartPs  Uart1;
static XUartPs *UartInstPtr1;

static UBYTE  ubPort1RxBuffer[ SER_PORT_0_RX_BUFFER_SIZE ];
static UWORD  uwPort1RxBufferWrIdx;
static UWORD  uwPort1RxBufferRdIdx;
static UBYTE  ubPort1TxBuffer[ SER_PORT_0_TX_BUFFER_SIZE ];
static BOOL   bPort1TxPecTransfert;
static UWORD  uwPort1TxBufferWrIdx;

static UWORD  uwPort1TimerEndTxCount;
//static UWORD  uwPort1TimerEndTxTicks;
static UWORD  uwPort1TimerEndTxTimer;

static BOOL   bPort1HalfDuplex;
static BOOL   bPort1HideRx;
static BOOL   bPort1Force422485;

static SERIAL_PORT_CALLBACK pfnPort1RxCallback;
static SERIAL_PORT_CALLBACK pfnPort1TxCallback;
#endif

/************************** Function Prototypes *****************************/

static int SetupInterruptSystem(XScuGic *IntcInstancePtr, XUartPs *UartInstancePtr, u16 UartIntrId);
static void Handler(void *CallBackRef, u32 Event, unsigned int EventData);

/////////////////////////////////////////////////////////////////////////////
//

SWORD SerialPortsInit( void )
{
  int Status;

#ifndef _INFINEON_
#ifndef _HW_DC
   Gpio_SetMode(COMM_RTS_PIN, GPIO_DIR_OUT);
   Gpio_SetMode(COMM_CTS_PIN, GPIO_DIR_IN);
   Gpio_SetMode(COMM_TYP_PIN, GPIO_DIR_IN);
#endif
#endif

  // Serial Port 0: Init
#ifdef SERIAL_PORT_0
   UartInstPtr0 = &Uart0;

   /*
      * Initialize the UART driver so that it's ready to use
      * Look up the configuration in the config table, then initialize it.
      */
   XUartPs_Config *UartConfig0 = XUartPs_LookupConfig(SERIAL_PORT_0_DEVICE_ID);
   if (NULL == UartConfig0) {
      return XST_FAILURE;
   }

   Status = XUartPs_CfgInitialize(UartInstPtr0, UartConfig0, UartConfig0->BaseAddress);
   if (Status != XST_SUCCESS) {
      return XST_FAILURE;
   }

   /* Check hardware build */
   Status = XUartPs_SelfTest(UartInstPtr0);
   if (Status != XST_SUCCESS) {
      return XST_FAILURE;
   }

   /*
      * Connect the UART to the interrupt subsystem such that interrupts
      * can occur. This function is application specific.
      */
   Status = SetupInterruptSystem(&xInterruptController, UartInstPtr0, SERIAL_PORT_0_INTR);
   if (Status != XST_SUCCESS) {
      return XST_FAILURE;
   }
#endif

  // Serial Port 1: Init
#ifdef SERIAL_PORT_1
   UartInstPtr1 = &Uart1;

   /*
      * Initialize the UART driver so that it's ready to use
      * Look up the configuration in the config table, then initialize it.
      */
   XUartPs_Config *UartConfig1 = XUartPs_LookupConfig(SERIAL_PORT_1_DEVICE_ID);
   if (NULL == UartConfig1) {
      return XST_FAILURE;
   }

   Status = XUartPs_CfgInitialize(UartInstPtr1, UartConfig1, UartConfig1->BaseAddress);
   if (Status != XST_SUCCESS) {
      return XST_FAILURE;
   }

   /* Check hardware build */
   Status = XUartPs_SelfTest(UartInstPtr1);
   if (Status != XST_SUCCESS) {
      return XST_FAILURE;
   }

   /*
      * Connect the UART to the interrupt subsystem such that interrupts
      * can occur. This function is application specific.
      */
   Status = SetupInterruptSystem(&xInterruptController, UartInstPtr1, SERIAL_PORT_1_INTR);
   if (Status != XST_SUCCESS) {
      return XST_FAILURE;
   }
#endif

   return 0;
}

/////////////////////////////////////////////////////////////////////////////
//

SWORD SerialPortOpen( UWORD uwPortNumber, ULONG ulBaudRate, UWORD uwDataBits, UWORD uwParityMode, UWORD uwStopBits, UWORD uwPortMode, UWORD uwWriteDelay, SERIAL_PORT_CALLBACK pfnRxCallback, SERIAL_PORT_CALLBACK pfnTxCallback )
{
   int Status;
   XUartPsFormat Format;

   // Check Port Number range
   if ( uwPortNumber < SERIALPORTS_CN_LOWERPORT || uwPortNumber > SERIALPORTS_CN_UPPERPORT )
      return SERIALPORTS_IE_BADID;

   // Check Port Already Open
#ifdef SERIAL_PORT_0
   if ( ( uwPortNumber == 0 &&  UartInstPtr0 == NULL  ) )
      return SERIALPORTS_IE_OPEN;
#endif
#ifdef SERIAL_PORT_1
   if ( ( uwPortNumber == 1 &&  UartInstPtr1 == NULL  ) )
      return SERIALPORTS_IE_OPEN;
#endif

   // Check Data Bits
   if ( uwDataBits != SERIALPORTS_DATABITS_6 &&
         uwDataBits != SERIALPORTS_DATABITS_7 && uwDataBits != SERIALPORTS_DATABITS_8 )
      return SERIALPORTS_IE_DATABITS;

   // Check Parity Mode
   if ( uwParityMode != SERIALPORTS_NOPARITY && uwParityMode != SERIALPORTS_EVENPARITY &&
         uwParityMode != SERIALPORTS_ODDPARITY )
      return SERIALPORTS_IE_PARITYMODE;

   // Check Stop Bits
   if ( uwStopBits != SERIALPORTS_ONESTOPBIT && uwStopBits != SERIALPORTS_TWOSTOPBITS )
      return SERIALPORTS_IE_STOPBITS;

   // Set Mode Register: Baudrate, Databits, Parity, Stopbit
   Format.BaudRate = ulBaudRate;
   Format.DataBits = uwDataBits==SERIALPORTS_DATABITS_6 ? XUARTPS_FORMAT_6_BITS :
                    (uwDataBits==SERIALPORTS_DATABITS_7 ? XUARTPS_FORMAT_7_BITS : XUARTPS_FORMAT_8_BITS);
   Format.Parity   = uwParityMode==SERIALPORTS_NOPARITY ? XUARTPS_FORMAT_NO_PARITY :
                    (uwParityMode==SERIALPORTS_ODDPARITY ? XUARTPS_FORMAT_ODD_PARITY : XUARTPS_FORMAT_EVEN_PARITY);
   Format.StopBits = uwStopBits==SERIALPORTS_ONESTOPBIT ? XUARTPS_FORMAT_1_STOP_BIT : XUARTPS_FORMAT_2_STOP_BIT;

#ifdef SERIAL_PORT_0
   if( uwPortNumber == 0 )
   {
      Status = XUartPs_SetDataFormat(UartInstPtr0, &Format);
      if (Status != XST_SUCCESS)
         return SERIALPORTS_IE_OPEN;

      // Check Baud Rate
      Status = XUartPs_SetBaudRate(UartInstPtr0, ulBaudRate);
      if (Status != XST_SUCCESS)
         return SERIALPORTS_IE_BAUDRATE;

      // Init Rx Buffer Wr/Rd pointers
      uwPort0RxBufferWrIdx = uwPort0RxBufferRdIdx = 0;

      // Setup Rx/Tx Callbacks
      pfnPort0RxCallback = pfnRxCallback;
      pfnPort0TxCallback = pfnTxCallback;

      // Setup Tx Flag
      bPort0TxPecTransfert = FALSE;

      // Init Tx Buffer Wr pointer
      uwPort0TxBufferWrIdx = 0;

      // Duplex selection
      bPort0HalfDuplex = uwPortMode & SERIALPORTS_PM_HALFDUPLEX;

      // Used to hide RX in half duplex mode
      bPort0HideRx = FALSE;

      // Set uart operation mode
      XUartPs_SetOperMode(UartInstPtr0, XUARTPS_OPER_MODE_NORMAL);

      // Set uart auto flow control
      XUartPs_SetOptions(UartInstPtr0, XUARTPS_OPTION_SET_FCM);

      // Set Receive FIFO trigger level
      XUartPs_SetFifoThreshold(UartInstPtr0, 1);

      // Set receive timeout: disable timeout when port open
      XUartPs_SetRecvTimeout(UartInstPtr0, 0);

      // Start receiving data
      uwPort0RxBufferWrIdx = XUartPs_Recv(UartInstPtr0, ubPort0RxBuffer, 1);
   }
#endif

#ifdef SERIAL_PORT_1
   if( uwPortNumber == 1 )
   {
      Status = XUartPs_SetDataFormat(UartInstPtr1, &Format);
      if (Status != XST_SUCCESS)
         return SERIALPORTS_IE_OPEN;

      // Check Baud Rate
      Status = XUartPs_SetBaudRate(UartInstPtr1, ulBaudRate);
      if (Status != XST_SUCCESS)
         return SERIALPORTS_IE_BAUDRATE;

      // Init Rx Buffer Wr/Rd pointers
      uwPort1RxBufferWrIdx = uwPort1RxBufferRdIdx = 0;

      // Setup Rx/Tx Callbacks
      pfnPort1RxCallback = pfnRxCallback;
      pfnPort1TxCallback = pfnTxCallback;

      // Setup Tx Flag
      bPort1TxPecTransfert = FALSE;

      // Init Tx Buffer Wr pointer
      uwPort1TxBufferWrIdx = 0;

      // Duplex selection
      bPort1HalfDuplex = uwPortMode & SERIALPORTS_PM_HALFDUPLEX;

      // Used to hide RX in half duplex mode
      bPort1HideRx = FALSE;

      // Set uart operation mode
      XUartPs_SetOperMode(UartInstPtr1, XUARTPS_OPER_MODE_NORMAL);

      // Set uart auto flow control
      XUartPs_SetOptions(UartInstPtr1, XUARTPS_OPTION_SET_FCM);

      // Set Receive FIFO trigger level
      XUartPs_SetFifoThreshold(UartInstPtr1, 1);

      // Set receive timeout: disable timeout when port open
      XUartPs_SetRecvTimeout(UartInstPtr1, 0);

      // Start receiving data
      uwPort1RxBufferWrIdx = XUartPs_Recv(UartInstPtr1, ubPort1RxBuffer, 1);
   }
#endif

   return 0;
}



/////////////////////////////////////////////////////////////////////////////
//

void SerialPortReceiverOn( UWORD uwPortNumber )
{
   // Avoids warning C47
   uwPortNumber = uwPortNumber;
}

/////////////////////////////////////////////////////////////////////////////
//

void SerialPortReceiverOff( UWORD uwPortNumber )
{
   // Avoids warning C47
   uwPortNumber = uwPortNumber;
}



/////////////////////////////////////////////////////////////////////////////
//

SWORD SerialPortClose( UWORD uwPortNumber )
{
   // Check Port Number range
   if ( uwPortNumber < SERIALPORTS_CN_LOWERPORT || uwPortNumber > SERIALPORTS_CN_UPPERPORT )
      return SERIALPORTS_IE_BADID;

   // Check Port Not Open
#ifdef SERIAL_PORT_0
   if ( ( uwPortNumber == 0 &&  UartInstPtr0 == NULL  ) )
      return SERIALPORTS_IE_NOPEN;
#endif
#ifdef SERIAL_PORT_1
   if ( ( uwPortNumber == 1 &&  UartInstPtr1 == NULL  ) )
      return SERIALPORTS_IE_NOPEN;
#endif

   return 0;
}

/////////////////////////////////////////////////////////////////////////////
//

void SerialPortForce422485( BOOL bForce )
{
#ifdef SERIAL_PORT_0
   bPort0Force422485 = bForce;
#endif
#ifdef SERIAL_PORT_1
   bPort1Force422485 = bForce;
#endif
}

/////////////////////////////////////////////////////////////////////////////
//

SWORD SerialPortRxFlush( UWORD uwPortNumber )
{
   // Check Port Number range
   if ( uwPortNumber < SERIALPORTS_CN_LOWERPORT || uwPortNumber > SERIALPORTS_CN_UPPERPORT )
      return SERIALPORTS_IE_BADID;

   // Check Port Not Open
#ifdef SERIAL_PORT_0
   if ( ( uwPortNumber == 0 &&  UartInstPtr0 == NULL  ) )
      return SERIALPORTS_IE_NOPEN;
#endif
#ifdef SERIAL_PORT_1
   if ( ( uwPortNumber == 1 &&  UartInstPtr1 == NULL  ) )
      return SERIALPORTS_IE_NOPEN;
#endif

  // Init Rx Buffer Wr/Rd pointers
#ifdef SERIAL_PORT_0
   if(uwPortNumber == 0)
   {
//  _atomic_( 0 );
      uwPort0RxBufferRdIdx = uwPort0RxBufferWrIdx;
//  _endatomic_();
   }
#endif
#ifdef SERIAL_PORT_1
   if(uwPortNumber == 1)
   {
//  _atomic_( 0 );
      uwPort1RxBufferRdIdx = uwPort1RxBufferWrIdx;
//  _endatomic_();
   }
#endif

   return 0;
}


/////////////////////////////////////////////////////////////////////////////
//

SWORD SerialPortRxStatus( UWORD uwPortNumber )
{
   SWORD swAvailableBytes=0;

   // Check Port Number range
   if ( uwPortNumber < SERIALPORTS_CN_LOWERPORT || uwPortNumber > SERIALPORTS_CN_UPPERPORT )
      return SERIALPORTS_IE_BADID;

   // Check Port Not Open
#ifdef SERIAL_PORT_0
   if ( ( uwPortNumber == 0 &&  UartInstPtr0 == NULL  ) )
      return SERIALPORTS_IE_NOPEN;
#endif
#ifdef SERIAL_PORT_1
   if ( ( uwPortNumber == 1 &&  UartInstPtr1 == NULL  ) )
      return SERIALPORTS_IE_NOPEN;
#endif

  // Return available bytes
#ifdef SERIAL_PORT_0
   if( uwPortNumber == 0 )
      swAvailableBytes = uwPort0RxBufferWrIdx - uwPort0RxBufferRdIdx;
#endif
#ifdef SERIAL_PORT_1
   if( uwPortNumber == 1 )
      swAvailableBytes = uwPort1RxBufferWrIdx - uwPort1RxBufferRdIdx;
#endif
    
   if ( swAvailableBytes < 0 )
   {
#ifdef SERIAL_PORT_0
      if( uwPortNumber == 0 )
         swAvailableBytes += sizeof( ubPort0RxBuffer );
#endif
#ifdef SERIAL_PORT_1
      if( uwPortNumber == 1 )
         swAvailableBytes += sizeof( ubPort1RxBuffer );
#endif
  }
  
  return swAvailableBytes;
}


/////////////////////////////////////////////////////////////////////////////
//

SWORD SerialPortRead( UWORD uwPortNumber, HPUBYTE hpubBuffer, UWORD uwLength )
{
   UWORD uwByteCount;
   
   // Check Port Number range
   if ( uwPortNumber < SERIALPORTS_CN_LOWERPORT || uwPortNumber > SERIALPORTS_CN_UPPERPORT )
      return SERIALPORTS_IE_BADID;

   // Check Port Not Open
#ifdef SERIAL_PORT_0
   if ( ( uwPortNumber == 0 &&  UartInstPtr0 == NULL  ) )
      return SERIALPORTS_IE_NOPEN;
#endif
#ifdef SERIAL_PORT_1
   if ( ( uwPortNumber == 1 &&  UartInstPtr1 == NULL  ) )
      return SERIALPORTS_IE_NOPEN;
#endif

   // 
   if ( SerialPortRxStatus( uwPortNumber ) < uwLength )
      return 0;

#ifdef SERIAL_PORT_0
   if( uwPortNumber == 0 )
   {
      //
      for ( uwByteCount = 0; uwByteCount < uwLength; uwByteCount++ ) {
         // Copy Data
         *hpubBuffer++ = ubPort0RxBuffer[ uwPort0RxBufferRdIdx++ ];
   
         // Adjust Rd index
         if ( uwPort0RxBufferRdIdx >= sizeof( ubPort0RxBuffer ) )
            uwPort0RxBufferRdIdx = 0;
      }
   }
#endif
#ifdef SERIAL_PORT_1
   if( uwPortNumber == 1 )
   {
      //
      for ( uwByteCount = 0; uwByteCount < uwLength; uwByteCount++ ) {
         // Copy Data
         *hpubBuffer++ = ubPort1RxBuffer[ uwPort1RxBufferRdIdx++ ];
   
         // Adjust Rd index
         if ( uwPort1RxBufferRdIdx >= sizeof( ubPort1RxBuffer ) )
            uwPort1RxBufferRdIdx = 0;
      }
   }
#endif

   return uwLength;
}


/////////////////////////////////////////////////////////////////////////////
//

SWORD SerialPortWrite( UWORD uwPortNumber, HPUBYTE hpubBuffer, UWORD uwLength, UWORD uwBuffering )
{
   // Check Port Number range
   if ( uwPortNumber < SERIALPORTS_CN_LOWERPORT || uwPortNumber > SERIALPORTS_CN_UPPERPORT )
      return SERIALPORTS_IE_BADID;

   // Check Port Not Open
#ifdef SERIAL_PORT_0
   if ( ( uwPortNumber == 0 &&  UartInstPtr0 == NULL  ) )
      return SERIALPORTS_IE_NOPEN;
#endif
#ifdef SERIAL_PORT_1
   if ( ( uwPortNumber == 1 &&  UartInstPtr1 == NULL  ) )
      return SERIALPORTS_IE_NOPEN;
#endif

   // Check Length
   if ( uwLength > SERIALPORTS_PEC_TRANSFERT_MAXIMUM_LENGTH + 1 )
      return SERIALPORTS_IE_BADLENGTH;

   // Check Buffer

   // Wait until PEC active
#ifdef SERIAL_PORT_0
   if(uwPortNumber == 0)
   {
      while ( bPort0TxPecTransfert );

      // select buffering type
      switch(uwBuffering)
      {
         case SERIALPORTS_BF_SENDONLY:
         // Copy to local buffer
         memcpy( ubPort0TxBuffer, hpubBuffer, uwLength );
         break;

         case SERIALPORTS_BF_QUEUEONLY:
         // Add to local buffer
         memcpy( &ubPort0TxBuffer[uwPort0TxBufferWrIdx], hpubBuffer, uwLength );
         uwPort0TxBufferWrIdx += uwLength;
         return uwPort0TxBufferWrIdx;

         case SERIALPORTS_BF_QUEUEANDSEND:
         // Add to local buffer
         memcpy( &ubPort0TxBuffer[uwPort0TxBufferWrIdx], hpubBuffer, uwLength );
         uwLength += uwPort0TxBufferWrIdx;
         break;

         default:
         return SERIALPORTS_IE_UNKNOWNBUFTYPE;
      }

      //
      bPort0TxPecTransfert = TRUE;
      uwPort0TimerEndTxTimer = uwPort0TimerEndTxCount;

      // Activate TX
   //  XE167_COMM_RTS = TRUE;
   //  XUartPs_SetOptions(UartInstPtr0, XUARTPS_OPTION_ASSERT_RTS);

      // If RS422/485 detected (or forced) and half duplex then activate RX hiding
   //  bPort0HideRx = (XE167_COMM_TYP || bPort0Force422485) && bPort0HalfDuplex;

      /*
      * Send the buffer using the UART and ignore the number of bytes sent
      * as the return value since we are using it in interrupt mode.
      */
      XUartPs_Send(UartInstPtr0, ubPort0TxBuffer, uwLength);
   }
#endif
#ifdef SERIAL_PORT_1
   if(uwPortNumber == 1)
   {
      while ( bPort1TxPecTransfert );

      // select buffering type
      switch(uwBuffering)
      {
         case SERIALPORTS_BF_SENDONLY:
         // Copy to local buffer
         memcpy( ubPort1TxBuffer, hpubBuffer, uwLength );
         break;

         case SERIALPORTS_BF_QUEUEONLY:
         // Add to local buffer
         memcpy( &ubPort1TxBuffer[uwPort1TxBufferWrIdx], hpubBuffer, uwLength );
         uwPort1TxBufferWrIdx += uwLength;
         return uwPort1TxBufferWrIdx;

         case SERIALPORTS_BF_QUEUEANDSEND:
         // Add to local buffer
         memcpy( &ubPort1TxBuffer[uwPort1TxBufferWrIdx], hpubBuffer, uwLength );
         uwLength += uwPort1TxBufferWrIdx;
         break;

         default:
         return SERIALPORTS_IE_UNKNOWNBUFTYPE;
      }

      //
      bPort1TxPecTransfert = TRUE;
      uwPort1TimerEndTxTimer = uwPort1TimerEndTxCount;

#ifndef _HW_DC
      // Activate TX
#ifdef _INFINEON_
      XE167_COMM_RTS = TRUE;
#else
      GPIO_OUT(COMM_RTS_PIN,TRUE);
#endif

      // If RS422/485 detected (or forced) and half duplex then activate RX hiding
#ifdef _INFINEON_
      bPort1HideRx = (XE167_COMM_TYP || bPort1Force422485) && bPort1HalfDuplex;
#else
      bPort1HideRx = (GPIO_IN(COMM_TYP_PIN) || bPort1Force422485) && bPort1HalfDuplex;
#endif
#endif // _hw_dc

      /*
      * Send the buffer using the UART and ignore the number of bytes sent
      * as the return value since we are using it in interrupt mode.
      */
      XUartPs_Send(UartInstPtr1, ubPort1TxBuffer, uwLength);
   }
#endif

   return uwLength;
}

/**************************************************************************/
/**
*
* This function is the handler which performs processing to handle data events
* from the device.  It is called from an interrupt context. so the amount of
* processing should be minimal.
*
* This handler provides an example of how to handle data for the device and
* is application specific.
*
* @param	CallBackRef contains a callback reference from the driver,
*		in this case it is the instance pointer for the XUartPs driver.
* @param	Event contains the specific kind of event that has occurred.
* @param	EventData contains the number of bytes sent or received for sent
*		and receive events.
*
* @return	None.
*
* @note		None.
*
***************************************************************************/
static void Handler(void *CallBackRef, u32 Event, unsigned int EventData)
{
   /* All of the data has been sent */
   if (Event == XUARTPS_EVENT_SENT_DATA) {
#ifdef SERIAL_PORT_0
      if(CallBackRef == UartInstPtr0)
      {
         // re-enable RX
         bPort0HideRx = FALSE;

         // Call Tx callback
         if ( pfnPort0TxCallback != NULL )
            pfnPort0TxCallback();

         // Signal End of TX
         bPort0TxPecTransfert = FALSE;

         // and reset Tx Buffer Wr pointer
         uwPort0TxBufferWrIdx = 0;
      }
#endif
#ifdef SERIAL_PORT_1
#ifndef _HW_DC
    // If RS422/485 detected (or forced) and half duplex then disable TX driver
#ifdef _INFINEON_
      if((XE167_COMM_TYP || bPort1Force422485) && bPort1HalfDuplex)
         XE167_COMM_RTS = FALSE;
#else
      if((GPIO_IN(COMM_TYP_PIN) || bPort1Force422485) && bPort1HalfDuplex)
         GPIO_OUT(COMM_RTS_PIN,FALSE);
#endif // _infineon
#endif // _hw_dc

      if(CallBackRef == UartInstPtr1)
      {
         // re-enable RX
         bPort1HideRx = FALSE;

         // Call Tx callback
         if ( pfnPort1TxCallback != NULL )
            pfnPort1TxCallback();

         // Signal End of TX
         bPort1TxPecTransfert = FALSE;

         // and reset Tx Buffer Wr pointer
         uwPort1TxBufferWrIdx = 0;
      }
#endif
   }

   /* All of the data has been received */
   if (Event == XUARTPS_EVENT_RECV_DATA) {
#ifdef SERIAL_PORT_0
      if(CallBackRef == UartInstPtr0)
      {
         do
         {
            // if rx hiding selected, then just flush buffer
            if(!bPort0HideRx)
            {
               // Increase Wr index
               uwPort0RxBufferWrIdx++;

               // Adjust Wr index
               if ( uwPort0RxBufferWrIdx >= sizeof( ubPort0RxBuffer ) )
                  uwPort0RxBufferWrIdx = 0;

               // Call Rx callback
               if ( pfnPort0RxCallback != NULL )
                  pfnPort0RxCallback();
            }
         }
         while(XUartPs_Recv(UartInstPtr0, &ubPort0RxBuffer[uwPort0RxBufferWrIdx], 1));
      }
#endif
#ifdef SERIAL_PORT_1
      if(CallBackRef == UartInstPtr1)
      {
         do
         {
            // if rx hiding selected, then just flush buffer
            if(!bPort1HideRx)
            {
               // Increase Wr index
               uwPort1RxBufferWrIdx++;

               // Adjust Wr index
               if ( uwPort1RxBufferWrIdx >= sizeof( ubPort1RxBuffer ) )
                  uwPort1RxBufferWrIdx = 0;

               // Call Rx callback
               if ( pfnPort1RxCallback != NULL )
                  pfnPort1RxCallback();
            }
         }
         while(XUartPs_Recv(UartInstPtr1, &ubPort1RxBuffer[uwPort1RxBufferWrIdx], 1));
      }
#endif
   }

   // Flush RX when timeout
   if (Event == XUARTPS_EVENT_RECV_TOUT ||
         Event == XUARTPS_EVENT_RECV_ERROR ||
         Event == XUARTPS_EVENT_PARE_FRAME_BRKE ||
         Event == XUARTPS_EVENT_RECV_ORERR
      )
   {}
}


/*****************************************************************************/
/**
*
* This function sets up the interrupt system so interrupts can occur for the
* Uart. This function is application-specific. The user should modify this
* function to fit the application.
*
* @param	IntcInstancePtr is a pointer to the instance of the INTC.
* @param	UartInstancePtr contains a pointer to the instance of the UART
*		driver which is going to be connected to the interrupt
*		controller.
* @param	UartIntrId is the interrupt Id and is typically
*		XPAR_<UARTPS_instance>_INTR value from xparameters.h.
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None.
*
****************************************************************************/
static int SetupInterruptSystem(XScuGic *IntcInstancePtr, XUartPs *UartInstancePtr, u16 UartIntrId)
{
   int Status;

   if(!IntcInstancePtr->IsReady)
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
      Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
            (Xil_ExceptionHandler) XScuGic_InterruptHandler,
            IntcInstancePtr);

      /* Enable interrupts */
      Xil_ExceptionEnable();
   }

   /* The priority setting */
   XScuGic_SetPriorityTriggerType(IntcInstancePtr, UartIntrId, SYSAPPIL_SERIALPORTS, 3);

   /*
   * Connect a device driver handler that will be called when an
   * interrupt for the device occurs, the device driver handler
   * performs the specific interrupt processing for the device
   */
   Status = XScuGic_Connect(IntcInstancePtr, UartIntrId,
            (Xil_ExceptionHandler) XUartPs_InterruptHandler,
            (void *) UartInstancePtr);
   if (Status != XST_SUCCESS) {
      return XST_FAILURE;
   }

   /*
   * Setup the handlers for the UART that will be called from the
   * interrupt context when data has been sent and received, specify
   * a pointer to the UART driver instance as the callback reference
   * so the handlers are able to access the instance data
   */
   XUartPs_SetHandler(UartInstancePtr, (XUartPs_Handler)Handler, UartInstancePtr);

   /* Enable the interrupt for the device */
   XScuGic_Enable(IntcInstancePtr, UartIntrId);

   /*
   * Enable the interrupt of the UART so interrupts will occur, setup
   * a local loopback so data that is sent will be received.
   */
   int IntrMask =
      // XUARTPS_IXR_RXFULL  | /**< RX FIFO full interrupt. */
      // XUARTPS_IXR_RXEMPTY | /**< RX FIFO empty interrupt. */
      XUARTPS_IXR_RXOVR   | /**< RX FIFO trigger interrupt. */
      // XUARTPS_IXR_TOVR    | /**< Tx FIFO Overflow interrupt */
      // XUARTPS_IXR_TNFUL   | /**< Tx FIFO Nearly Full interrupt */
      // XUARTPS_IXR_TTRIG   | /**< Tx Trig interrupt */
      // XUARTPS_IXR_TXFULL  | /**< TX FIFO full interrupt. */
      XUARTPS_IXR_TXEMPTY | /**< TX FIFO empty interrupt. */
      // XUARTPS_IXR_DMS     | /**< Modem status change interrupt */
      XUARTPS_IXR_PARITY  | /**< Parity error interrupt */
      XUARTPS_IXR_FRAMING | /**< Framing error interrupt */
      XUARTPS_IXR_OVER    | /**< Overrun error interrupt */
      XUARTPS_IXR_TOUT      /**< Timeout error interrupt */
   ;

   XUartPs_SetInterruptMask(UartInstancePtr, IntrMask);

   return XST_SUCCESS;
}
