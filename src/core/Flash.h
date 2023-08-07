/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Flash.h                                                    */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Qspi Flash controller                                      */
/*                                                                          */
/****************************************************************************/

//***************************************************************************
// Include
#ifndef ___QSPI_H___
#define ___QSPI_H___

/***************************** Include Files *********************************/
#include "xparameters.h"
#include "xil_io.h"
#include "xqspips_hw.h"

/************************** Constant Definitions *****************************/

/*
 * Identification of Flash
 * Micron:
 * Byte 0 is Manufacturer ID;
 * Byte 1 is first byte of Device ID - 0xBB or 0xBA
 * Byte 2 is second byte of Device ID describes flash size:
 * 128Mbit : 0x18; 256Mbit : 0x19; 512Mbit : 0x20
 * Spansion:
 * Byte 0 is Manufacturer ID;
 * Byte 1 is Device ID - Memory Interface type - 0x20 or 0x02
 * Byte 2 is second byte of Device ID describes flash size:
 * 128Mbit : 0x18; 256Mbit : 0x19; 512Mbit : 0x20
 */

#define MICRON_ID                      0x20
#define SPANSION_ID                    0x01
#define WINBOND_ID                     0xEF
#define MACRONIX_ID                    0xC2
#define ISSI_ID                        0x9D

#define FLASH_SIZE_ID_8M               0x14
#define FLASH_SIZE_ID_16M              0x15
#define FLASH_SIZE_ID_32M              0x16
#define FLASH_SIZE_ID_64M              0x17
#define FLASH_SIZE_ID_128M             0x18
#define FLASH_SIZE_ID_256M             0x19
#define FLASH_SIZE_ID_512M             0x20
#define FLASH_SIZE_ID_1G               0x21
/* Macronix size constants are different for 512M and 1G */
#define MACRONIX_FLASH_SIZE_ID_512M    0x1A
#define MACRONIX_FLASH_SIZE_ID_1G      0x1B
#define MACRONIX_FLASH_1_8_V_MX66_ID_512   (0x3A)
/*
 * Size in bytes
 */
#define FLASH_SIZE_8M                  0x0100000
#define FLASH_SIZE_16M                 0x0200000
#define FLASH_SIZE_32M                 0x0400000
#define FLASH_SIZE_64M                 0x0800000
#define FLASH_SIZE_128M                0x1000000
#define FLASH_SIZE_256M                0x2000000
#define FLASH_SIZE_512M                0x4000000
#define FLASH_SIZE_1G                  0x8000000

/*
 * The following constants specify the page size, sector size, and number of
 * pages and sectors for the FLASH.  The page size specifies a max number of
 * bytes that can be written to the FLASH with a single transfer.
 */
#define SECTOR_SIZE                    0x10000  // 64KB
#define SUBSECTOR_SIZE                 0x01000  // 4KB
#define PAGE_SIZE                      0x00100  // 256B

#define NUM_SECTORS                    0x00100  // 256
#define NUM_PAGES                      0x10000

/************************** Function Prototypes ******************************/
u32  Flash_Init(void);
u32  FlashReadID(void);
void FlashErase(u32 Address, u32 ByteCount);
void FlashRead(u32 Address, u8 * hpdata, u32 ByteCount);
u32  FlashLinearRead(u32 Address, u8 * hpdata, u32 ByteCount);
void FlashWrite(u32 Address, u8 * hpdata, u32 ByteCount);
u32  FlashErasedCheck( void * addr, u32 len );

/************************** Variable Definitions *****************************/
extern u32 QspiFlashSize;
extern u32 QspiFlashMake;

#endif /* ___QSPI_H___ */
