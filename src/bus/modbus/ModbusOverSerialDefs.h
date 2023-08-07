/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ModbusOverSerialDefs.h                                     */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _MODBUSOVERSERIALDEFS_H
#define _MODBUSOVERSERIALDEFS_H

//***************************************************************************

#define MODBUS_SER_PDU_SIZE_MIN     4       // Minimum size of a Modbus RTU frame.
#define MODBUS_SER_PDU_SIZE_MAX     256     // Maximum size of a Modbus RTU frame.
#define MODBUS_SER_PDU_SIZE_CRC     2       // Size of CRC field in PDU.
#define MODBUS_SER_PDU_OFFS_ADR     0       // Offset of slave address in Ser-PDU.
#define MODBUS_SER_PDU_OFFS_PDU     1       // Offset of Modbus-PDU in Ser-PDU.

//***************************************************************************

#define MODBUS_OVERSER_ADDRESS_BROADCAST      0   // Modbus broadcast address.
#define MODBUS_OVERSER_ADDRESS_MIN_VALUE      1   // Smallest possible slave address.
#define MODBUS_OVERSER_ADDRESS_MAX_VALUE    247   // Biggest possible slave address.

//***************************************************************************

#define MODBUSOVERSER_CRC_STARTVALUE        (0xFFFF)

#endif
