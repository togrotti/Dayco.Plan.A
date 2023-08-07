/*****************************************************************************/
/* Project: Ax-Zynq Control Board                                            */
/*                                                                           */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved   */
/*                                                                           */
/* File        : IICHander.h                                                 */
/* Author      : WF                                                          */
/*                                                                           */
/* Description : IIC hander heard file                                       */
/*                                                                           */
/*****************************************************************************/
#ifndef _IIC_HANDLER_H
 #define _IIC_HANDLER_H

/////////////////////////////////////////////////////////////////////////////
//

#ifndef FALSE
  #define FALSE     0
  #define TRUE      (!FALSE)
#endif

/////////////////////////////////////////////////////////////////////////////
//

#define IIC_INTERNAL_BUS      3
#define IIC_EXTERNAL_BUS      4
#define IIC_IOEXP_BUS         5
#define IIC_CTOPT_BUS         (IIC_IOEXP_BUS)

/////////////////////////////////////////////////////////////////////////////
//

#define IIC_MAX_BUS_DEVICES   4


/////////////////////////////////////////////////////////////////////////////
//  New type
/////////////////////////////////////////////////////////////////////////////
// IIC search Algorithm Functions
typedef struct
{
   unsigned char   DeviceAddress;
   unsigned char   IICBusType;
   unsigned char   crc8;
} IIC_SEARCH_STATE;

/////////////////////////////////////////////////////////////////////////////
// Globle variables


/////////////////////////////////////////////////////////////////////////////
//

int IIC_HandlerInit( void );
/////////////////////////////////////////////////////////////////////////////
// IIC Basic Functions



/////////////////////////////////////////////////////////////////////////////
// IIC Derived Functions

unsigned char IIC_devicesearch( unsigned char bus, IIC_SEARCH_STATE * state, unsigned char deviceaddr );
int IIC_SelfReadMemory(  unsigned char bus, unsigned short lenth );
int IIC_SelfWriteSingleByte(  unsigned char bus );

/////////////////////////////////////////////////////////////////////////////
// IIC Search Algorithm Functions


/////////////////////////////////////////////////////////////////////////////
//




/////////////////////////////////////////////////////////////////////////////
// IIC Specific Device Functions
int IIC_ReadMemory( unsigned char bus, unsigned char *devaddr, unsigned char *outputbuffer, unsigned short baseaddr, unsigned short lenth );
int IIC_WriteMemory( unsigned char bus, unsigned char *devaddr, unsigned char *inputbuffer, unsigned short baseaddr, unsigned short lenth );

#endif // _IIC_HANDLER_H
