/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EnetTcpIp.h                                                */
/* Author      : Hu Xiaokai                                                 */
/*               Zhang Ganggang                                             */
/* Description : Ethernet Tcp/IP Handler                                    */
/*                                                                          */
/****************************************************************************/

#ifndef _ENETPROT_H
#define _ENETPROT_H

//***************************************************************************
// Definition 
#define UWTW(uwData) 	(((UWORD)uwData)<<8|((UWORD)uwData)>>8)
#define ULTW(ulData) 	(((ULONG)ulData&0xff000000)>>24|\
						(( ULONG)ulData&0x00ff0000)>>8 |\
						(( ULONG)ulData&0x0000ff00)<<8 |\
						(( ULONG)ulData&0x000000ff)<<24)

/////////////////////////////////////////////////////////////////////////////
//

#define ETHPROT_TCP     0x6
#define ETHPROT_UDP     0x11

typedef struct
{
  UBYTE ubProt; // TCP: 6, UDP: 17
  UBYTE ubIPAddr[4];
  UWORD uwPortNum;
  
} ENET_PROT_SETUP;

//***************************************************************************
// Global functions
BOOL EnetProtocol_Init(ENET_PROT_SETUP  *hpProtSetup);
BOOL EnetProtocol_Recv(void);
void EnetProtocol_Read(HPUBYTE hpubFrameBuf, UWORD uwOffset, UWORD uwSize);
void EnetProtocol_Send(HPUBYTE hpubFrameBuf, UWORD uwSize);

#endif
