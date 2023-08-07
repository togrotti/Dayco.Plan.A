/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : OneWireHandler.h                                           */
/* Author      : Stefano Martino                                            */
/*               Fabio Terrile                                              */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _ONE_WIRE_HANDLER_H
 #define _ONE_WIRE_HANDLER_H



/////////////////////////////////////////////////////////////////////////////
//

#ifndef FALSE
  #define FALSE     0
  #define TRUE      (!FALSE)
#endif


/////////////////////////////////////////////////////////////////////////////
//

#define OW_DS2431_FAMILY_CODE    0x2D
#define OW_DS2431_DATA_LENGTH     128

#define OW_DS2433_FAMILY_CODE    0x23
#define OW_DS2433_DATA_LENGTH     512

#define OW_MAX_BUS_SEGMENTS       3
#define OW_MAX_BUS_DEVICES        8

/////////////////////////////////////////////////////////////////////////////
//

#define OW_INTERNAL_BUS     0
#define OW_EXTERNAL_BUS     1
#define OW_IOEXP_BUS        2
#define OW_CTOPT_BUS        (OW_IOEXP_BUS)

/////////////////////////////////////////////////////////////////////////////
//

int OWHandlerInit( void );

/////////////////////////////////////////////////////////////////////////////
// 1-Wire Basic Functions

int OWReset( int bus );
void OWWriteBit( int bus, int val );
int OWReadBit( int bus );

/////////////////////////////////////////////////////////////////////////////
// 1-Wire Derived Functions

void OWWriteByte( int bus, int data );
int OWReadByte( int bus );
int OWTouchByte( int bus, int data );
void OWBlock( int bus, unsigned char * data, int length, void (* yield)(void) );


/////////////////////////////////////////////////////////////////////////////
// 1-Wire Search Algorithm Functions

typedef struct
{
  unsigned char ROM_NO[ 8 ];
  int           LastDiscrepancy;
  int           LastFamilyDiscrepancy;
  int           LastDeviceFlag;
  unsigned char crc8;

} OW_SEARCH_STATE;

/////////////////////////////////////////////////////////////////////////////
//

int OWSearchFirst( int bus, OW_SEARCH_STATE  * state, void (* yield)(void) );
int OWSearchNext( int bus, OW_SEARCH_STATE  * state, void (* yield)(void) );


/////////////////////////////////////////////////////////////////////////////
// 1-Wire Specific Device Functions

int OWReadMemory( int bus, unsigned char * romno, unsigned char * buffer, int length, void (* yield)(void) );



#endif // _ONE_WIRE_HANDLER_H
