/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppSFlashPartZYNQ.h                                     */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Partitioning of serial flash, ZYNQ version                 */
/*                                                                          */
/****************************************************************************/

#ifndef _SYSAPPSFLASHPARTZYNQ_H
#define _SYSAPPSFLASHPARTZYNQ_H

#include "SysAppZYNQ.lin"

//***************************************************************************
// Application storage (8192kB): FSBL.ELF, APP.BIT, APP.ELF

#define SFPART_SYSAPP_START             SYSAPP_CODE_START
#define SFPART_SYSAPP_SIZE              SYSAPP_CODE_SIZE 

//***************************************************************************
// Bootblock(Golden Image) storage (8192kB): FSBL.ELF, APP.BIT, APP.ELF

#define SFPART_BOOTBLOCK_START          BOOTBLOCK_CODE_START
#define SFPART_BOOTBLOCK_SIZE           BOOTBLOCK_CODE_SIZE 

//***************************************************************************
// PLC source code storage (320kB)

#define SFPART_PLC_PRJ_START            PLC_PRJ_CODE_START
#define SFPART_PLC_PRJ_SIZE             PLC_PRJ_CODE_SIZE 

//***************************************************************************
// RD Application HMI (320kB)

#define SFPART_RD_HMI_APP_START         RD_HMI_APP_START
#define SFPART_RD_HMI_APP_SIZE          RD_HMI_APP_SIZE 

//***************************************************************************
// PLC requested memory area storage (64kB)

#define SFPART_PLCREQMEM_START          DIGITALSCOPE_LOG_START
#define SFPART_PLCREQMEM_SIZE           DIGITALSCOPE_LOG_SIZE 

#endif
