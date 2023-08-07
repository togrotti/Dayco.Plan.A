/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SerialPorts.h                                              */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _SERIALPORTS_H
#define _SERIALPORTS_H

/////////////////////////////////////////////////////////////////////////////
//

#define SERIALPORTS_DATABITS_5          5
#define SERIALPORTS_DATABITS_6          6
#define SERIALPORTS_DATABITS_7          7
#define SERIALPORTS_DATABITS_8          8

#define SERIALPORTS_NOPARITY            0
#define SERIALPORTS_ODDPARITY           1
#define SERIALPORTS_EVENPARITY          2

#define SERIALPORTS_ONESTOPBIT          1
#define SERIALPORTS_TWOSTOPBITS         2

//
// Baud rates at which the communication device can operates
//
#define SERIALPORTS_CBR_1200          1200
#define SERIALPORTS_CBR_2400          2400
#define SERIALPORTS_CBR_4800          4800
#define SERIALPORTS_CBR_9600          9600
#define SERIALPORTS_CBR_14400        14400
#define SERIALPORTS_CBR_19200        19200
#define SERIALPORTS_CBR_38400        38400
#define SERIALPORTS_CBR_56000        56000
#define SERIALPORTS_CBR_57600        57600
#define SERIALPORTS_CBR_115200      115200
#define SERIALPORTS_CBR_128000      128000
#define SERIALPORTS_CBR_230400      230400
#define SERIALPORTS_CBR_256000      256000
#define SERIALPORTS_CBR_460800      460800
#define SERIALPORTS_CBR_512000      512000


//
// Port mode
//
#define SERIALPORTS_PM_HALFDUPLEX   0x0001

// Error Flags
//
#define SERIALPORTS_CE_RXOVER           0x0001  // Receive Queue overflow
#define SERIALPORTS_CE_OVERRUN          0x0002  // Receive Overrun Error
#define SERIALPORTS_CE_RXPARITY         0x0004  // Receive Parity Error
#define SERIALPORTS_CE_FRAME            0x0008  // Receive Framing error
#define SERIALPORTS_CE_BREAK            0x0010  // Break Detected
#define SERIALPORTS_CE_TXFULL           0x0100  // TX Queue is full
#define SERIALPORTS_CE_MODE             0x8000  // Requested mode unsupported

#define SERIALPORTS_IE_BADID            (-1)    // Invalid or unsupported id
#define SERIALPORTS_IE_OPEN             (-2)    // Device Already Open
#define SERIALPORTS_IE_NOPEN            (-3)    // Device Not Open
#define SERIALPORTS_IE_DATABITS         (-4)    // Illegal Byte Size
#define SERIALPORTS_IE_BAUDRATE         (-5)    // Unsupported BaudRate
#define SERIALPORTS_IE_PARITYMODE       (-6)    // Illegal Parity Mode
#define SERIALPORTS_IE_STOPBITS         (-6)    // Illegal Stop Bits
#define SERIALPORTS_IE_BADLENGTH        (-7)    // Bad Length
#define SERIALPORTS_IE_UNKNOWNBUFTYPE   (-8)    // Unknown buffering type

// Buffering flags for SerialPortWrite
//
#define SERIALPORTS_BF_SENDONLY         (1)
#define SERIALPORTS_BF_QUEUEONLY        (2)
#define SERIALPORTS_BF_QUEUEANDSEND     (3)


/////////////////////////////////////////////////////////////////////////////
//

#define SERIALPORTS_CN_LOWERPORT        1
#define SERIALPORTS_CN_UPPERPORT        1


/////////////////////////////////////////////////////////////////////////////
//

typedef void (*SERIAL_PORT_CALLBACK)( void );

/////////////////////////////////////////////////////////////////////////////
//

SWORD SerialPortsInit( void );

SWORD SerialPortOpen( UWORD uwPortNumber, ULONG ulBaudRate, UWORD uwDataBits, UWORD uwParityMode, UWORD uwStopBits, UWORD uwPortMode, UWORD uwWriteDelay, SERIAL_PORT_CALLBACK pfnRxCallback, SERIAL_PORT_CALLBACK pfnTxCallback );

void SerialPortReceiverOn( UWORD uwPortNumber );
void SerialPortReceiverOff( UWORD uwPortNumber );

SWORD SerialPortRxStatus( UWORD uwPortNumber );
SWORD SerialPortRxFlush( UWORD uwPortNumber );

SWORD SerialPortRead( UWORD uwPortNumber, HPUBYTE hpubBuffer, UWORD uwLength );
SWORD SerialPortWrite( UWORD uwPortNumber, HPUBYTE hpubuffer, UWORD uwLength, UWORD uwBuffering );

SWORD SerialPortClose( UWORD uwPortNumber );

void SerialPortForce422485( BOOL bForce );

#ifdef _INFINEON_
void * SerialPortGetTimingSetup( ULONG ulBaudRate );
#endif

#endif
