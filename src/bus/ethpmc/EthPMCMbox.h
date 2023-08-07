/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCMbox.h                                               */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Mailbox common handler                                     */
/*                                                                          */
/****************************************************************************/

#ifndef _ETHPMCMBOX_H
#define _ETHPMCMBOX_H

// Mailbox CommControl Word
#define EPMCMBOX_COMMCTRL_COMM         0    // "Command" bit offset in CommControl Word
#define EPMCMBOX_COMMCTRL_NODE         2    // "Node Number" bit offset in CommControl Word
#define EPMCMBOX_COMMCTRL_DSIZE        8    // "Data Size" bit offset in CommControl Word
#define EPMCMBOX_COMMCTRL_PROTL        12   // "Protocol" bit offset in CommControl Word

#define EPMCMBOX_COMMCTRL_COMM_MASK    ((1<<EPMCMBOX_COMMCTRL_NODE)-1)
#define EPMCMBOX_COMMCTRL_NODE_MASK    (((1<<EPMCMBOX_COMMCTRL_DSIZE)-1)-((1<<EPMCMBOX_COMMCTRL_NODE)-1))
#define EPMCMBOX_COMMCTRL_DSIZE_MASK   (((1<<EPMCMBOX_COMMCTRL_PROTL)-1)-((1<<EPMCMBOX_COMMCTRL_DSIZE)-1))
#define EPMCMBOX_COMMCTRL_PROTL_MASK   (0xFFFF-((1<<EPMCMBOX_COMMCTRL_PROTL)-1))

// Node
#define EPMCMBOX_COMMCTRL_NODE_NULL    0x00 // Node Number: 0x00, reserved
#define EPMCMBOX_COMMCTRL_NODE_BST     0x3F // Node Number: 0x3F, default un-configured state/broadcast
// Command
#define EPMCMBOX_COMMCTRL_CMD_IDLE     0    // Command: IDLE
#define EPMCMBOX_COMMCTRL_CMD_REQ      1    // Command: Request
#define EPMCMBOX_COMMCTRL_CMD_RPLY     2    // Command: Reply
// Protocol
#define EPMCMBOX_COMMCTRL_PROTL_EPMC   0    // Protocol: EtherPMC network protocol
#define EPMCMBOX_COMMCTRL_PROTL_MODBUS 1    // Protocol: Modbus


#define EPMCMBOXL_MAX_PAYLOADSIZE      14   // bytes
  
// Utilities
#define EPMCMBOX_SET(p,n,s,c)          (((UWORD)(p)<<EPMCMBOX_COMMCTRL_PROTL) | \
                                        ((UWORD)(s)<<EPMCMBOX_COMMCTRL_DSIZE) | \
                                        ((UWORD)(n)<<EPMCMBOX_COMMCTRL_NODE)  | \
                                        ((UWORD)(c)<<EPMCMBOX_COMMCTRL_COMM))

#define EPMCMBOX_GET_PROTL(x)          (((x)&EPMCMBOX_COMMCTRL_PROTL_MASK)>>EPMCMBOX_COMMCTRL_PROTL)
#define EPMCMBOX_GET_DSIZE(x)          (((x)&EPMCMBOX_COMMCTRL_DSIZE_MASK)>>EPMCMBOX_COMMCTRL_DSIZE)
#define EPMCMBOX_GET_NODE(x)           (((x)&EPMCMBOX_COMMCTRL_NODE_MASK)>>EPMCMBOX_COMMCTRL_NODE)
#define EPMCMBOX_GET_COMM(x)           (((x)&EPMCMBOX_COMMCTRL_COMM_MASK)>>EPMCMBOX_COMMCTRL_COMM)

#endif