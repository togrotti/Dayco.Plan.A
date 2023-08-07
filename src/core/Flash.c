/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : Flash.c                                                    */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Qspi Flash controller                                      */
/*                                                                          */
/****************************************************************************/

/***************************** Include Files *********************************/
#include "Flash.h"
#include "common/CommonDefines.h"
#include "xparameters.h"

#ifdef XPAR_PS7_QSPI_LINEAR_0_S_AXI_BASEADDR
#include "xqspips_hw.h"
#include "xqspips.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

/************************** Constant Definitions *****************************/

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define QSPI_DEVICE_ID		XPAR_XQSPIPS_0_DEVICE_ID

/*
 * The following constants define the commands which may be sent to the FLASH
 * device.
 */
#define WRITE_STATUS_CMD        0x01
#define WRITE_CMD               0x02
#define READ_CMD                0x03
#define WRITE_DISABLE_CMD       0x04
#define WRITE_ENABLE_CMD        0x06
#define READ_STATUS_CMD         0x05
#define FAST_READ_CMD           0x0B
#define DUAL_READ_CMD           0x3B
#define QUAD_READ_CMD           0x6B

#define BULK_ERASE_CMD          0xC7 // ENTIRE FLASH ERASE
#define	SEC_ERASE_CMD           0xD8 // 64KB ERASE
#define SUBSEC_ERASE_CMD        0x20 // 4KB  ERASE

#define READ_ID_CMD             0x9F

#define BANK_REG_RD             0x16
#define BANK_REG_WR             0x17

/* Bank register is called Extended Address Reg in Micron */
#define EXTADD_REG_RD			0xC8
#define EXTADD_REG_WR			0xC5

#define COMMAND_OFFSET			0 /* FLASH instruction */
#define ADDRESS_1_OFFSET		1 /* MSB byte of address to read or write */
#define ADDRESS_2_OFFSET		2 /* Middle byte of address to read or write */
#define ADDRESS_3_OFFSET		3 /* LSB byte of address to read or write */
#define DATA_OFFSET				4 /* Start of Data for Read/Write */
#define DUMMY_OFFSET			4 /* Dummy byte offset for fast, dual and quad
                                   reads */
#define DUMMY_SIZE				1 /* Number of dummy bytes for fast, dual and
                                   quad reads */
#define RD_ID_SIZE				4 /* Read ID command + 3 bytes ID response */
#define BANK_SEL_SIZE			2 /* BRWR or EARWR command + 1 byte bank value */
#define WRITE_ENABLE_CMD_SIZE	1 /* WE command */
#define BULK_ERASE_SIZE			1 /* Bulk Erase command size */
#define SEC_ERASE_SIZE			4 /* Sector Erase command + Sector address */

/*
 * The following constants specify the extra bytes which are sent to the
 * FLASH on the QSPI interface, that are not data, but control information
 * which includes the command and address
 */
#define OVERHEAD_SIZE			4

/*
 * The following constants specify the max amount of data and the size of the
 * the buffer required to hold the data and overhead to transfer the data to
 * and from the FLASH.
 */
#define DATA_SIZE				1024//4096

#define FLASH_MODE_LINEAR 		0
#define FLASH_MODE_IO     		1

#define LQSPI_CR_FAST_READ		0x0000000B
#define LQSPI_CR_FAST_DUAL_READ	0x0000003B
#define LQSPI_CR_FAST_QUAD_READ	0x0000006B /* Fast Quad Read output */
#define LQSPI_CR_1_DUMMY_BYTE	0x00000100 /* 1 Dummy Byte between */


/*
 *
 */
#define XQSPIPS_FLASH_OPSIZE_WREN		1	// WREN command

/*
 * The following constants define the commands specific to ISSI devices
 */
#define ISSI_FLASH_OPCODE_RDSR		XQSPIPS_FLASH_OPCODE_RDSR1
#define ISSI_FLASH_OPCODE_WRSR		XQSPIPS_FLASH_OPCODE_WRSR

#define ISSI_STATUS_REGISTER_WIP	(1 << 0)	// Write In Progress
#define ISSI_STATUS_REGISTER_WEL	(1 << 1)	// Write Enable Latch
#define ISSI_STATUS_REGISTER_QE		(1 << 6)	// Quad Enable bit

/*
 * The following constants define the commands specific to SPANSION devices
 */
#define SPANSION_FLASH_OPCODE_RDSR1		XQSPIPS_FLASH_OPCODE_RDSR1
#define SPANSION_FLASH_OPCODE_RDCR		XQSPIPS_FLASH_OPCODE_RDSR2
#define SPANSION_FLASH_OPCODE_WRR		XQSPIPS_FLASH_OPCODE_WRSR

#define SPANSION_CONFIGURATION_REGISTER_1_QUAD	(1 << 1)	// Quad I/O operation




/************************** Function Prototypes ******************************/
static void FlashQuadEnable(XQspiPs *QspiPtr);
static void FlashLinearMode(void);
static void FlashCmdMode(void);
static u32 SendBankSelect(u8 BankSel);

/************************** Variable Definitions *****************************/

static XQspiPs QspiInstance;
static XQspiPs *QspiInstancePtr;
static u32 FlashReadBaseAddress = 0;
static u8  FlashMode=FLASH_MODE_LINEAR;

u32 QspiFlashSize;
u32 QspiFlashMake;

/*
 * The following variables are used to read and write to the eeprom and they
 * are global to avoid having large buffers on the stack
 */
static u8 ReadBuffer[DATA_SIZE + DATA_OFFSET + DUMMY_SIZE];
static u8 WriteBuffer[PAGE_SIZE + DATA_OFFSET];

/******************************************************************************/
/**
*
* This function initializes the controller for the QSPI interface.
*
* @param	None
*
* @return	None
*
* @note		None
*
****************************************************************************/
u32 Flash_Init(void)
{
    XQspiPs_Config *QspiConfig;
    int Status;

    QspiInstancePtr = &QspiInstance;

    /*
     * Set up the base address for access
     */
    FlashReadBaseAddress = XPS_QSPI_LINEAR_BASEADDR;

    /*
     * Initialize the QSPI driver so that it's ready to use
     */
    QspiConfig = XQspiPs_LookupConfig(QSPI_DEVICE_ID);
    if (NULL == QspiConfig) {
        return XST_FAILURE;
    }

    Status = XQspiPs_CfgInitialize(QspiInstancePtr, QspiConfig,
                    QspiConfig->BaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /* Perform a self-test to check hardware build*/
    Status = XQspiPs_SelfTest(QspiInstancePtr);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /*
     * Set the prescaler for QSPI clock
     */
    XQspiPs_SetClkPrescaler(QspiInstancePtr, XQSPIPS_CLK_PRESCALE_8);

    /*
     * Set Manual Start and Manual Chip select options and drive the
     * HOLD_B high.
     */
    XQspiPs_SetOptions(QspiInstancePtr, XQSPIPS_FORCE_SSELECT_OPTION |
                                        XQSPIPS_MANUAL_START_OPTION  |
                                        XQSPIPS_HOLD_B_DRIVE_OPTION  );

    /*
     * Assert the FLASH chip select.
     */
    XQspiPs_SetSlaveSelect(QspiInstancePtr);

    /*
     * Read Flash ID and extract Manufacture and Size information
     */
    Status = FlashReadID();
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    FlashQuadEnable(QspiInstancePtr);

    /*
     * Select low 128Mb when flash density over 128Mb
     */
    if(QspiFlashSize > FLASH_SIZE_128M)
    {
        Status = SendBankSelect(0);
        if (Status != XST_SUCCESS) {
            return XST_FAILURE;
        }
    }

    /*
     * Single flash IO read
     */
    XQspiPs_SetLqspiConfigReg(QspiInstancePtr, LQSPI_CR_1_DUMMY_BYTE |
                                                       LQSPI_CR_FAST_QUAD_READ);

    XQspiPs_SetOptions(QspiInstancePtr, XQSPIPS_LQSPI_MODE_OPTION |
                                           XQSPIPS_HOLD_B_DRIVE_OPTION);

    /*
     * Enable Linear Mode
     */
    XQspiPs_Enable(QspiInstancePtr);

    FlashMode = FLASH_MODE_LINEAR;

    return XST_SUCCESS;
}

/******************************************************************************
*
* This function reads serial FLASH ID connected to the SPI interface.
* It then deduces the make and size of the flash and obtains the
* connection mode to point to corresponding parameters in the flash
* configuration table. The flash driver will function based on this and
* it presently supports Micron and Spansion - 128, 256 and 512Mbit and
* Winbond 128Mbit
*
* @param	none
*
* @return	XST_SUCCESS if read id, otherwise XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
u32 FlashReadID(void)
{
    u32 Status;

    if(QspiInstancePtr == NULL)
        QspiInstancePtr = &QspiInstance;

    /*
     * Read ID in Auto mode.
     */
    WriteBuffer[COMMAND_OFFSET]   = READ_ID_CMD;
    WriteBuffer[ADDRESS_1_OFFSET] = 0x00;		/* 3 dummy bytes */
    WriteBuffer[ADDRESS_2_OFFSET] = 0x00;
    WriteBuffer[ADDRESS_3_OFFSET] = 0x00;

    Status = XQspiPs_PolledTransfer(QspiInstancePtr, WriteBuffer, ReadBuffer,
                RD_ID_SIZE);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /*
     * Deduce flash make
     */
    if (ReadBuffer[1] == MICRON_ID) {
        QspiFlashMake = MICRON_ID;
    } else if(ReadBuffer[1] == SPANSION_ID) {
        QspiFlashMake = SPANSION_ID;
    } else if(ReadBuffer[1] == WINBOND_ID) {
        QspiFlashMake = WINBOND_ID;
    } else if(ReadBuffer[1] == MACRONIX_ID) {
        QspiFlashMake = MACRONIX_ID;
    } else if(ReadBuffer[1] == ISSI_ID) {
        QspiFlashMake = ISSI_ID;
    }

    /*
     * Deduce flash Size
     */
    if (ReadBuffer[3] == FLASH_SIZE_ID_8M) {
        QspiFlashSize = FLASH_SIZE_8M;
    } else if (ReadBuffer[3] == FLASH_SIZE_ID_16M) {
        QspiFlashSize = FLASH_SIZE_16M;
    } else if (ReadBuffer[3] == FLASH_SIZE_ID_32M) {
        QspiFlashSize = FLASH_SIZE_32M;
    } else if (ReadBuffer[3] == FLASH_SIZE_ID_64M) {
        QspiFlashSize = FLASH_SIZE_64M;
    } else if (ReadBuffer[3] == FLASH_SIZE_ID_128M) {
        QspiFlashSize = FLASH_SIZE_128M;
    } else if (ReadBuffer[3] == FLASH_SIZE_ID_256M) {
        QspiFlashSize = FLASH_SIZE_256M;
    } else if ((ReadBuffer[3] == FLASH_SIZE_ID_512M)
            || (ReadBuffer[3] == MACRONIX_FLASH_1_8_V_MX66_ID_512)
            || (ReadBuffer[3] == MACRONIX_FLASH_SIZE_ID_512M)) {
        QspiFlashSize = FLASH_SIZE_512M;
    } else if ((ReadBuffer[3] == FLASH_SIZE_ID_1G)
            || (ReadBuffer[3] == MACRONIX_FLASH_SIZE_ID_1G)) {
        QspiFlashSize = FLASH_SIZE_1G;
    }

    return XST_SUCCESS;
}


/******************************************************************************
*
* This function reads from the  serial FLASH connected to the
* QSPI interface.
*
* @param	Address contains the address to read data from in the FLASH.
* @param	ByteCount contains the number of bytes to read.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void FlashRead(u32 Address, u8 * hpdata, u32 ByteCount)
{
    if(QspiInstancePtr == NULL)
        QspiInstancePtr = &QspiInstance;

    FlashCmdMode();

    while (ByteCount) {
    	// do not read more data than the buffer can contain
    	u32 ReadLen = min(DATA_SIZE, ByteCount);

        /*
         * Setup the write command with the specified address and data for the
         * FLASH
         */
        WriteBuffer[COMMAND_OFFSET]   = QUAD_READ_CMD;
        WriteBuffer[ADDRESS_1_OFFSET] = (u8)((Address & 0xFF0000) >> 16);
        WriteBuffer[ADDRESS_2_OFFSET] = (u8)((Address & 0xFF00) >> 8);
        WriteBuffer[ADDRESS_3_OFFSET] = (u8)(Address & 0xFF);

        /*
         * Send the read command to the FLASH to read the specified number
         * of bytes from the FLASH, send the read command and address and
         * receive the specified number of bytes of data in the data buffer
         */
        XQspiPs_PolledTransfer(QspiInstancePtr, WriteBuffer, ReadBuffer,
        		ReadLen + DUMMY_SIZE + OVERHEAD_SIZE);

        memcpy(hpdata, &ReadBuffer[OVERHEAD_SIZE+DUMMY_SIZE], ReadLen);

        hpdata += ReadLen;
        Address += ReadLen;
        ByteCount -= ReadLen;
    }

    FlashLinearMode();
}

/******************************************************************************
*
* This function reads from the  serial FLASH connected to the
* QSPI interface in Linear mode.
*
* @param	Address contains the address to read data from in the FLASH.
* @param	ByteCount contains the number of bytes to read.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
u32 FlashLinearRead(u32 Address, u8 * hpdata, u32 ByteCount)
{
    u32 Status;

    if(QspiInstancePtr == NULL)
        QspiInstancePtr = &QspiInstance;

    Status = XQspiPs_LqspiRead(QspiInstancePtr, hpdata, Address&0xFFFFFF, ByteCount);

    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function writes to the  serial FLASH connected to the QSPI interface.
* All the data put into the buffer must be in the same page of the device with
* page boundaries being on 256 byte boundaries.
*
* @param	Address contains the address to write data to in the FLASH.
* @param	ByteCount contains the number of bytes to write.
* @param	Command is the command used to write data to the flash. QSPI
*		device supports only Page Program command to write data to the
*		flash.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void FlashWrite(u32 Address, u8 *hpdata, u32 ByteCount)
{
    u8 WriteEnableCmd = { WRITE_ENABLE_CMD };
    u8 ReadStatusCmd[] = { READ_STATUS_CMD, 0 };  /* must send 2 bytes */
    u8 FlashStatus[2];
    u32 BaseAddress,StartAddress,NextAddress;
    u32 LeftSize,WriteSize;

    BaseAddress=StartAddress=Address;
    LeftSize=ByteCount;

    if(QspiInstancePtr == NULL)
        QspiInstancePtr = &QspiInstance;

    FlashCmdMode();

    while(LeftSize>0)
    {
        // Calculate start address and write size
        NextAddress = (StartAddress/PAGE_SIZE + 1)*PAGE_SIZE;
        WriteSize = LeftSize;
        if(StartAddress + WriteSize > NextAddress) // if write beyond the end of the current page
            WriteSize = NextAddress - StartAddress;

        /*
         * Setup the write command with the specified address and data for the
         * FLASH
         */
        WriteBuffer[COMMAND_OFFSET]   = WRITE_CMD;
        WriteBuffer[ADDRESS_1_OFFSET] = (u8)((StartAddress & 0xFF0000) >> 16);
        WriteBuffer[ADDRESS_2_OFFSET] = (u8)((StartAddress & 0xFF00) >> 8);
        WriteBuffer[ADDRESS_3_OFFSET] = (u8)(StartAddress & 0xFF);

        // Copy write data
        memcpy(&WriteBuffer[OVERHEAD_SIZE],&hpdata[StartAddress-BaseAddress],WriteSize);

        /*
         * Send the write enable command to the FLASH so that it can be
         * written to, this needs to be sent as a separate transfer before
         * the write
         */
        XQspiPs_PolledTransfer(QspiInstancePtr, &WriteEnableCmd, NULL,
                    sizeof(WriteEnableCmd));

        /*
         * Send the write command, address, and data to the FLASH to be
         * written, no receive buffer is specified since there is nothing to
         * receive
         */
        XQspiPs_PolledTransfer(QspiInstancePtr, WriteBuffer, NULL, WriteSize + OVERHEAD_SIZE);

        StartAddress += WriteSize;
        LeftSize -= WriteSize;

        /*
         * Wait for the write command to the FLASH to be completed, it takes
         * some time for the data to be written
         */
        while (1) {
            /*
             * Poll the status register of the FLASH to determine when it
             * completes, by sending a read status command and receiving the
             * status byte
             */
            XQspiPs_PolledTransfer(QspiInstancePtr, ReadStatusCmd, FlashStatus,
                        sizeof(ReadStatusCmd));

            /*
             * If the status indicates the write is done, then stop waiting,
             * if a value of 0xFF in the status byte is read from the
             * device and this loop never exits, the device slave select is
             * possibly incorrect such that the device status is not being
             * read
             */
            FlashStatus[1] |= FlashStatus[0];
            if ((FlashStatus[1] & 0x01) == 0) {
                break;
            }
        }
    }

    FlashLinearMode();
}

#pragma GCC push_options
#pragma GCC optimize (0)
/*****************************************************************************/
/**
*
* This function erases the sectors in the  serial FLASH connected to the
* QSPI interface.
*
* @param	Address contains the address of the first sector which needs to
*		be erased.
* @param	ByteCount contains the total size to be erased.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void FlashErase(u32 Address, u32 ByteCount)
{
    u8 WriteEnableCmd = { WRITE_ENABLE_CMD };
    u8 ReadStatusCmd[] = { READ_STATUS_CMD, 0 };  /* must send 2 bytes */
    u8 FlashStatus[2];
    int Sector;

    if(QspiInstancePtr == NULL)
        QspiInstancePtr = &QspiInstance;

    FlashCmdMode();

    /*
     * If erase size is same as the total size of the flash, use bulk erase
     * command
     */
    if (ByteCount == (NUM_SECTORS * SECTOR_SIZE)) {
        /*
         * Send the write enable command to the FLASH so that it can be
         * written to, this needs to be sent as a separate transfer
         * before the erase
         */
        XQspiPs_PolledTransfer(QspiInstancePtr, &WriteEnableCmd, NULL,
                  sizeof(WriteEnableCmd));

        /* Setup the bulk erase command*/
        WriteBuffer[COMMAND_OFFSET]   = BULK_ERASE_CMD;

        /*
         * Send the bulk erase command; no receive buffer is specified
         * since there is nothing to receive
         */
        XQspiPs_PolledTransfer(QspiInstancePtr, WriteBuffer, NULL,
                    BULK_ERASE_SIZE);

        /* Wait for the erase command to the FLASH to be completed*/
        while (1) {
            /*
             * Poll the status register of the device to determine
             * when it completes, by sending a read status command
             * and receiving the status byte
             */
            XQspiPs_PolledTransfer(QspiInstancePtr, ReadStatusCmd,
                        FlashStatus,
                        sizeof(ReadStatusCmd));

            /*
             * If the status indicates the write is done, then stop
             * waiting; if a value of 0xFF in the status byte is
             * read from the device and this loop never exits, the
             * device slave select is possibly incorrect such that
             * the device status is not being read
             */
            FlashStatus[1] |= FlashStatus[0];
            if ((FlashStatus[1] & 0x01) == 0) {
                break;
            }
        }

        FlashLinearMode();
        return;
    }

    /*
     * If the erase size is less than the total size of the flash, use
     * sector erase command
     */
    for (Sector = 0; Sector < (((ByteCount-1) / SECTOR_SIZE) + 1); Sector++) {
        /*
         * Send the write enable command to the SEEPOM so that it can be
         * written to, this needs to be sent as a separate transfer
         * before the write
         */
        XQspiPs_PolledTransfer(QspiInstancePtr, &WriteEnableCmd, NULL,
                  sizeof(WriteEnableCmd));

        /*
         * Setup the write command with the specified address and data
         * for the FLASH
         */
        WriteBuffer[COMMAND_OFFSET]   = ByteCount>SUBSECTOR_SIZE ? SEC_ERASE_CMD : SUBSEC_ERASE_CMD;

        WriteBuffer[ADDRESS_1_OFFSET] = (u8)(Address >> 16);
        WriteBuffer[ADDRESS_2_OFFSET] = (u8)(Address >> 8);
        WriteBuffer[ADDRESS_3_OFFSET] = (u8)(Address & 0xFF);

        /*
         * Send the sector erase command and address; no receive buffer
         * is specified since there is nothing to receive
         */
        XQspiPs_PolledTransfer(QspiInstancePtr, WriteBuffer, NULL,
                    SEC_ERASE_SIZE);

        /*
         * Wait for the sector erse command to the
         * FLASH to be completed
         */
        while (1) {
            /*
             * Poll the status register of the device to determine
             * when it completes, by sending a read status command
             * and receiving the status byte
             */
            XQspiPs_PolledTransfer(QspiInstancePtr, ReadStatusCmd,
                        FlashStatus,
                        sizeof(ReadStatusCmd));

            /*
             * If the status indicates the write is done, then stop
             * waiting, if a value of 0xFF in the status byte is
             * read from the device and this loop never exits, the
             * device slave select is possibly incorrect such that
             * the device status is not being read
             */
            FlashStatus[1] |= FlashStatus[0];
            if ((FlashStatus[1] & 0x01) == 0) {
                break;
            }
        }

        Address += SECTOR_SIZE;
    }

    FlashLinearMode();
}
#pragma GCC pop_options

/*****************************************************************************/
/**
//
// Check Flash Erased
//    Parameter:      addr:  Start Address
//                    len :  Length
//    Return Value:   0 - OK,  1 - Failed
//
*
******************************************************************************/
u32 FlashErasedCheck( void * addr, u32 len )
{
    u32 adr=(u32)addr;
    u8 * ptr;

    ptr=(u8 *)adr;

    while(len--)
        if((*ptr++) != 0xFF)
            return 1;

    return 0;
}




/******************************************************************************
*
* This functions sets QE bit on ISSI Status Register
*
* @param	None
*
* @return	XST_SUCCESS
*
* @note		None.
*
******************************************************************************/

u32 ISSIsetQuadEnableBit(XQspiPs *QspiPtr)
{
	u32 Status;

	/*
	 * Read Status Register
	 */
    u8 ReadStatusOpCode[] = {ISSI_FLASH_OPCODE_RDSR, 0};
    u8 ReadStatusResult[2];
	Status = XQspiPs_PolledTransfer(QspiPtr, ReadStatusOpCode, ReadStatusResult, sizeof(ReadStatusOpCode));
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

	u8 StatusRegister = ReadStatusResult[1];

	/*
	 * ISSI quad output mode already set
     */
	if (StatusRegister & ISSI_STATUS_REGISTER_QE)
		return XST_SUCCESS;

	/*
	 * Write Enable
     */
	u8 WriteEnableOpCode[] = {XQSPIPS_FLASH_OPCODE_WREN};
	Status = XQspiPs_PolledTransfer(QspiPtr, WriteEnableOpCode, NULL, sizeof(WriteEnableOpCode));
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

	/*
	 * Write Status Register
	 */
    u8 WriteStatusRegisterOpCode[] = {XQSPIPS_FLASH_OPCODE_WRSR, StatusRegister | ISSI_STATUS_REGISTER_WEL | ISSI_STATUS_REGISTER_QE};
	Status = XQspiPs_PolledTransfer(QspiPtr, WriteStatusRegisterOpCode, NULL, sizeof(WriteStatusRegisterOpCode));
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

    /*
     * Poll the status register of the FLASH to determine when Quad Mode is enabled and the device is ready
     */
    while (1) {

    	Status = XQspiPs_PolledTransfer(QspiPtr, ReadStatusOpCode, ReadStatusResult, sizeof(ReadStatusOpCode));
    	if (Status != XST_SUCCESS)
    		return XST_FAILURE;

    	/*
         * If QE bit is set & WIP bit is reset, then Quad is Enabled and device is ready.
         */
    	if ((ReadStatusResult[1] & ISSI_STATUS_REGISTER_QE) && !(ReadStatusResult[1] & ISSI_STATUS_REGISTER_WIP))
            break;
    }



	return XST_SUCCESS;
}

/******************************************************************************
*
* This functions sets QUAD bit on SPANSION Configuration Register 1
*
* @param	None
*
* @return	XST_SUCCESS
*
* @note		None.
*
******************************************************************************/

u32 SPANSIONsetQuadEnableBit(XQspiPs *QspiPtr)
{
	u32 Status;

	/*
	 * Read Status Register 1
	 */
    u8 ReadStatusOpCode[] = {SPANSION_FLASH_OPCODE_RDSR1, 0};
    u8 ReadStatusResult[2];
	Status = XQspiPs_PolledTransfer(QspiPtr, ReadStatusOpCode, ReadStatusResult, sizeof(ReadStatusOpCode));
	if (Status != XST_SUCCESS)
		return XST_FAILURE;
	u8 StatusRegister1 = ReadStatusResult[1];

	/*
	 * Read Configuration Register 1
	 */
    u8 ReadConfigOpCode[] = {SPANSION_FLASH_OPCODE_RDCR, 0};
    u8 ReadConfigResult[2];
	Status = XQspiPs_PolledTransfer(QspiInstancePtr, ReadConfigOpCode, ReadConfigResult, sizeof(ReadConfigOpCode));
	if (Status != XST_SUCCESS)
		return XST_FAILURE;
	u8 ConfigurationRegister1 = ReadConfigResult[1];

	/*
	 * SPANSION quad output mode already set
     */
	if (ConfigurationRegister1 & SPANSION_CONFIGURATION_REGISTER_1_QUAD)
		return XST_SUCCESS;

	/*
	 * Write Enable
     */
	u8 WriteEnableOpCode[] = {XQSPIPS_FLASH_OPCODE_WREN};
	Status = XQspiPs_PolledTransfer(QspiPtr, WriteEnableOpCode, NULL, sizeof(WriteEnableOpCode));
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

	/*
	 * Write Status Register 1 and Configuration Register 1
	 */
    u8 WriteStatusAndConfigOpCode[] = {SPANSION_FLASH_OPCODE_WRR, StatusRegister1, ConfigurationRegister1 | SPANSION_CONFIGURATION_REGISTER_1_QUAD};
	Status = XQspiPs_PolledTransfer(QspiInstancePtr, WriteStatusAndConfigOpCode, NULL, sizeof(WriteStatusAndConfigOpCode));
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

    /*
     * Poll the status register of the FLASH to determine when Quad Mode is enabled and the device is ready
     */
//$SM TODO


	return XST_SUCCESS;
}

/**
 *
 * This function enables quad mode in the serial flash connected to the
 * SPI interface.
 *
 * @param	QspiPtr is a pointer to the QSPI driver component to use.
 *
 * @return	None.
 *
 * @note		None.
 *
 ******************************************************************************/
static void FlashQuadEnable(XQspiPs *QspiPtr)
{
	/*
	 * ISSI
     */
	if (QspiFlashMake == ISSI_ID)
    	ISSIsetQuadEnableBit(QspiPtr);

	/*
	 * SPANSION
     */
    else if (QspiFlashMake == SPANSION_ID)
    	SPANSIONsetQuadEnableBit(QspiPtr);

	/*
	 * MICRON_ID, WINBOND_ID, MACRONIX_ID
     */
    else {

	    u8 WriteEnableCmd = {WRITE_ENABLE_CMD};
	    u8 ReadStatusCmd[] = {READ_STATUS_CMD, 0};
	    u8 QuadEnableCmd[] = {WRITE_STATUS_CMD, 0};
	    u8 FlashStatus[2];
	
	
	    XQspiPs_PolledTransfer(QspiPtr, ReadStatusCmd,
	                FlashStatus,
	                sizeof(ReadStatusCmd));
	
	    QuadEnableCmd[1] = FlashStatus[1] | 1 << 6;
	
	    XQspiPs_PolledTransfer(QspiPtr, &WriteEnableCmd, NULL,
	              sizeof(WriteEnableCmd));
	
	    XQspiPs_PolledTransfer(QspiPtr, QuadEnableCmd, NULL,
	                sizeof(QuadEnableCmd));
	
	    while (1) {
	        /*
	         * Poll the status register of the FLASH to determine when
	         * Quad Mode is enabled and the device is ready, by sending
	         * a read status command and receiving the status byte
	         */
	        XQspiPs_PolledTransfer(QspiPtr, ReadStatusCmd, FlashStatus,
	                sizeof(ReadStatusCmd));
	        /*
	         * If 6th bit is set & 0th bit is reset, then Quad is Enabled
	         * and device is ready.
	         */
	        if ((FlashStatus[0] == 0x40) && (FlashStatus[1] == 0x40)) {
	            break;
	        }
		}
    }
}

/******************************************************************************
*
* This functions selects the current bank
*
* @param	BankSel is the bank to be selected in the flash device(s).
*
* @return	XST_SUCCESS if bank selected
*			XST_FAILURE if selection failed
* @note		None.
*
******************************************************************************/
static u32 SendBankSelect(u8 BankSel)
{
    u32 Status;

    /*
     * bank select commands for Micron and Spansion are different
     * Macronix bank select is same as Micron
     */
    if (QspiFlashMake == MICRON_ID || QspiFlashMake == MACRONIX_ID)	{
        /*
         * For micron command WREN should be sent first
         * except for some specific feature set
         */
        WriteBuffer[COMMAND_OFFSET] = WRITE_ENABLE_CMD;
        Status = XQspiPs_PolledTransfer(QspiInstancePtr, WriteBuffer, NULL,
                WRITE_ENABLE_CMD_SIZE);
        if (Status != XST_SUCCESS) {
            return XST_FAILURE;
        }

        /*
         * Send the Extended address register write command
         * written, no receive buffer required
         */
        WriteBuffer[COMMAND_OFFSET]   = EXTADD_REG_WR;
        WriteBuffer[ADDRESS_1_OFFSET] = BankSel;
        Status = XQspiPs_PolledTransfer(QspiInstancePtr, WriteBuffer, NULL,
                BANK_SEL_SIZE);
        if (Status != XST_SUCCESS) {
            return XST_FAILURE;
        }
    }

    if (QspiFlashMake == SPANSION_ID) {
        WriteBuffer[COMMAND_OFFSET]   = BANK_REG_WR;
        WriteBuffer[ADDRESS_1_OFFSET] = BankSel;

        /*
         * Send the Extended address register write command
         * written, no receive buffer required
         */
        Status = XQspiPs_PolledTransfer(QspiInstancePtr, WriteBuffer, NULL,
                BANK_SEL_SIZE);
        if (Status != XST_SUCCESS) {
            return XST_FAILURE;
        }
    }

    /*
     * For testing - Read bank to verify
     */
    if (QspiFlashMake == SPANSION_ID) {
        WriteBuffer[COMMAND_OFFSET]   = BANK_REG_RD;
        WriteBuffer[ADDRESS_1_OFFSET] = 0x00;

        /*
         * Send the Extended address register write command
         * written, no receive buffer required
         */
        Status = XQspiPs_PolledTransfer(QspiInstancePtr, WriteBuffer, ReadBuffer,
                BANK_SEL_SIZE);
        if (Status != XST_SUCCESS) {
            return XST_FAILURE;
        }
    }

    if (QspiFlashMake == MICRON_ID || QspiFlashMake == MACRONIX_ID) {
        WriteBuffer[COMMAND_OFFSET]   = EXTADD_REG_RD;
        WriteBuffer[ADDRESS_1_OFFSET] = 0x00;

        /*
         * Send the Extended address register write command
         * written, no receive buffer required
         */
        Status = XQspiPs_PolledTransfer(QspiInstancePtr, WriteBuffer, ReadBuffer,
                BANK_SEL_SIZE);
        if (Status != XST_SUCCESS) {
            return XST_FAILURE;
        }
    }

    if (ReadBuffer[1] != BankSel) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/******************************************************************************/
/**
*
* This function start the QSPI interface as linear mode.
*
* @param	None
*
* @return	None
*
* @note		None
*
****************************************************************************/
static void FlashLinearMode(void)
{
    if(QspiInstancePtr == NULL)
        QspiInstancePtr = &QspiInstance;

    if(FlashMode != FLASH_MODE_LINEAR)
    {
        FlashMode = FLASH_MODE_LINEAR;

        /*
         * Enable linear mode
         */
        XQspiPs_SetOptions(QspiInstancePtr, XQSPIPS_LQSPI_MODE_OPTION |
                                            XQSPIPS_HOLD_B_DRIVE_OPTION);

        /*
         * Enable the controller
         */
        XQspiPs_Enable(QspiInstancePtr);
    }
}

/******************************************************************************/
/**
*
* This function start the QSPI interface as IO mode.
*
* @param	None
*
* @return	None
*
* @note		None
*
****************************************************************************/
static void FlashCmdMode(void)
{
    if(QspiInstancePtr == NULL)
        QspiInstancePtr = &QspiInstance;

    if(FlashMode != FLASH_MODE_IO)
    {
        FlashMode = FLASH_MODE_IO;

        /*
         * Set Manual Start and Manual Chip select options and drive the
         * HOLD_B high.
         */
        XQspiPs_SetOptions(QspiInstancePtr, XQSPIPS_FORCE_SSELECT_OPTION |
    //										XQSPIPS_MANUAL_START_OPTION  |
                                            XQSPIPS_HOLD_B_DRIVE_OPTION  );
    }
}

#endif
