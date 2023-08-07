/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : HardwareUnitID.h                                           */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Definition of hardware unit ID mask, valid in every        */
/*               hardware definition section; not to be confused with       */
/*               FPGA_HWOPT_* options, that describe implemented features   */
/*               in the FPGA                                                */
/*                                                                          */
/****************************************************************************/

#ifndef _HARDWAREUNITID_H
#define _HARDWAREUNITID_H

#include "common\CommonDefines.h"

//***************************************************************************
// Definitions

#define HWUNITID_DEFAULT            (0ul)

#define HWUNITID_BRI_FULL_0         (0x000000001ul) // #1 full power bridge
#define HWUNITID_BRI_FULL_1         (0x000000002ul) // #2 full power bridge
#define HWUNITID_BRI_FULL_2         (0x000000004ul) // #3 full power bridge
#define HWUNITID_ETH_DBLRMII_0      (0x000000008ul) // main double RMII eth ports
#define HWUNITID_ETH_DBLRMII_1      (0x000000010ul) // second double RMII eth ports
#define HWUNITID_MOT_RLIN_KTY       (0x000000020ul) // motor KTY linear resistance sensor
#define HWUNITID_MOT_RLIN_PTC       (0x000000040ul) // motor PTC linear resistance sensor
#define HWUNITID_STO_ROUTING        (0x000000080ul) // safety torque off routing backplane
#define HWUNITID_ENCM_VPWR          (0x000000100ul) // main enc - variable power supply
#define HWUNITID_ENCM_ABS           (0x000000200ul) // main enc - analog tracks + double gain + excitation out
#define HWUNITID_ENCM_INC_DIG       (0x000000400ul) // main enc - secondary bidirectional digital pairs + index
#define HWUNITID_ENCM_INC_AN        (0x000000800ul) // main enc - secondary analog tracks
#define HWUNITID_ENCA_VPWR          (0x000001000ul) // aux enc - variable power supply
#define HWUNITID_ENCA_INC_DIG       (0x000002000ul) // aux enc - input only digital pairs
#define HWUNITID_ENCA_INC_SIM       (0x000004000ul) // aux enc - output only digital pairs
#define HWUNITID_IO_STD             (0x000008000ul) // IO standard board (8DI, 4DO, 4AI, 2AO)
#define HWUNITID_STO                (0x000010000ul) // safety torque off main
#define HWUNITID_CAN_PORT_0         (0x000020000ul) // main CAN bus port
#define HWUNITID_CAN_PORT_1         (0x000040000ul) // second CAN bus port
#define HWUNITID_PSU_ROUTING        (0x000080000ul) // power supply unit routing backplane
#define HWUNITID_PSU_CB             (0x000100000ul) // power supply unit controlboard
#define HWUNITID_ENCA_INC_BOUT      (0x000200000ul) // aux enc - two output only plus one bi-dir digital pairs

#ifndef _HW_AXS
  #define HWUNITID_ALLSUPPORTED     (0x0003BFFFBul) // all supported
#else
  #define HWUNITID_ALLSUPPORTED     (0x0003FFFFBul) // all supported
#endif


#endif
