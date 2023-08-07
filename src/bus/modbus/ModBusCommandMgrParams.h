/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ModBusCommandMgrParams.h                                   */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : ModBus/Cockpit specific command manager                    */
/*                                                                          */
/****************************************************************************/

#ifndef _MODBUSCOMMANDMGRPARAMS_H
#define _MODBUSCOMMANDMGRPARAMS_H

#include "ModbusAppProtocol.h"
#include "system\HeaderFooterInfo.h"

//***************************************************************************
// Globals

#ifndef _HW_DC
extern MODBUS_PROTOCOL_OPTION sModBusCMParams;
extern const MODBUS_PROTOCOL_OPTION  sModBusCMDefParams;
extern MODBUS_OVERSER_DRIVER_SETUP sModBusCMParOverserial0;
extern const MODBUS_OVERSER_DRIVER_SETUP  sModBusCMDefParOverserial0;

#else
extern const MODBUS_PROTOCOL_OPTION sModBusCMParams;
extern const MODBUS_OVERSER_DRIVER_SETUP sModBusCMParOverserial0;
#endif

extern MODBUS_OVERETH_DRIVER_SETUP sModBusCMParOverEth;
extern const MODBUS_OVERETH_DRIVER_SETUP  sModBusCMDefParOverEth;

extern HEADER_FOOTER_INFO  sModBusCMBootBlockInfo;
extern ULONG  ulModBusCMBootBlockSysId;
extern ULONG  ulModBusCMBootBlockAppId;

#endif
