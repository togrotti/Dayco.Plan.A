/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : PacketCommand.h                                            */
/* Author      : Stefano Martino                                            */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/


/////////////////////////////////////////////////////////////////////////////
//

#define PACKET_COMMAND_ANS_READY           0xAA
#define PACKET_COMMAND_ANS_ERROR           0xFF

#define PACKET_COMMAND_MAX_READ_LENGTH     248

/////////////////////////////////////////////////////////////////////////////
//

SWORD ExecutePacketCommand( HPUBYTE hpubInpBuffer, HPUBYTE hpubOutBuffer );


