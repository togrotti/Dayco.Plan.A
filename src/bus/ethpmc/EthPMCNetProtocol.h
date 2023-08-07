/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCNetProtocol.h                                        */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : EtherPMC Network Protocol Common Handler                   */
/*                                                                          */
/****************************************************************************/

#ifndef _ETHPMCNETPROTOCOL_H
#define _ETHPMCNETPROTOCOL_H

  // Data Size
#define EPMCNETPROTL_MASTER_NODEIDENT_DATASIZE    2 // byte unit
#define EPMCNETPROTL_MASTER_SETNODENUM_DATASIZE   4 // byte unit
#define EPMCNETPROTL_MASTER_CONFIGSLOT_DATASIZE   6 // byte unit
#define EPMCNETPROTL_MASTER_ENOUTPORT_DATASIZE    4 // byte unit
#define EPMCNETPROTL_MASTER_GETLINKST_DATASIZE    2 // byte unit
#define EPMCNETPROTL_MASTER_NETSTATEREQ_DATASIZE  4 // byte unit
#define EPMCNETPROTL_MASTER_FRAMETIMEOUT_DATASIZE 4 // byte unit
 
#define EPMCNETPROTL_SLAVE_NODEIDENT_DATASIZE     6 // byte unit
#define EPMCNETPROTL_SLAVE_SETNODENUM_DATASIZE    8 // byte unit
#define EPMCNETPROTL_SLAVE_CONFIGSLOT_DATASIZE    6 // byte unit
#define EPMCNETPROTL_SLAVE_ENOUTPORT_DATASIZE     4 // byte unit
#define EPMCNETPROTL_SLAVE_GETLINKST_DATASIZE     4 // byte unit
#define EPMCNETPROTL_SLAVE_NETSTATEREQ_DATASIZE   4 // byte unit
#define EPMCNETPROTL_SLAVE_FRAMETIMEOUT_DATASIZE  4 // byte unit

// Command
#define EPMCNETPROTL_COMMAND_NODEIDENT            1 // EtherPMC Protocol Command: Node Ident
#define EPMCNETPROTL_COMMAND_SETNODENUM           2 // EtherPMC Protocol Command: Set Node Number
#define EPMCNETPROTL_COMMAND_CONFIGSLOT           3 // EtherPMC Protocol Command: Configure real time data slot
#define EPMCNETPROTL_COMMAND_ENOUTPORT            4 // EtherPMC Protocol Command: Enable/Disable output port
#define EPMCNETPROTL_COMMAND_GETLINKST            5 // EtherPMC Protocol Command: Output port link status
#define EPMCNETPROTL_COMMAND_NETSTATEREQ          6 // EtherPMC Protocol Command: Network state request
#define EPMCNETPROTL_COMMAND_FRAMETIMEOUT         7 // EtherPMC Protocol Command: Network frame timeout

// Link Status
#define EPMCNETPROTL_LINKSTATUS_LINK		      0
#define EPMCNETPROTL_LINKSTATUS_SPEED		      1
#define EPMCNETPROTL_LINKSTATUS_DUPLEX		      2
#define EPMCNETPROTL_LINKSTATUS_MDI               3

// Network state
#define EPMCNETPROTL_STATE_INIT                   0
#define EPMCNETPROTL_STATE_SLAVE                  1
#define EPMCNETPROTL_STATE_SCAN                   2
#define EPMCNETPROTL_STATE_MASTER                 3

#endif
