/****************************************************************************/
/* Project: Zynq development                                         		*/
/*                                                                          */
/* Copyright © 2016-2022, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : stgap4s.h                                       	        */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description : ST STGAP4S functions	                    				*/
/*                                                                          */
/****************************************************************************/

#ifdef _HW_AXS_CABI35KW

#include "xparameters.h"
#include "xspips.h"
#include "sleep.h"
#include "xgpiops.h"

#include "system\SysAppGlobals.h"

#include "core\Gpio.h"

#include "stgap4s.h"




#pragma GCC optimize (0)


//#define STGAP4S_TEST_DAISY_CHAIN_REVERSED
//#define STGAP4S_TEST_DAISY_CHAIN_SHUFFLED

#ifdef _DEBUG_TRACES
//#define STGAP4S_TRACE_READ_CONFIG_REGISTERS
#endif


#define STGAP4S_READ_BACK_CONFIG_REGISTERS




/*
 *  STGAP4S SPI commands.
 */
#define STGAP4S_COMMAND_START_CONFIG	0b00101010		// Enter CFG mode (SD low only)
#define STGAP4S_COMMAND_STOP_CONFIG		0b00111010		// Leave CFG mode (SD low only)
#define STGAP4S_COMMAND_NO_OPERATION	0b00000000		// No operation
#define STGAP4S_COMMAND_WRITE_REGISTER	0b10000000		// Write register (SD low only)
#define STGAP4S_COMMAND_READ_REGISTER	0b10100000		// Read register
#define STGAP4S_COMMAND_RESET_STATUS	0b11010000		// Reset all the status registers (SD low only)
#define STGAP4S_COMMAND_SOFT_RESET		0b11101010		// Soft reset (CFG mode only)


/*
 *  STGAP4S register addresses.
 */
#define STGAP4S_REGADRS_CFG1			0x00		// Local
#define STGAP4S_REGADRS_CFG2			0x11		// Remote
#define STGAP4S_REGADRS_CFG3			0x12		// Remote
#define STGAP4S_REGADRS_CFG4			0x13		// Remote
#define STGAP4S_REGADRS_CFG5			0x14		// Remote
#define STGAP4S_REGADRS_CFG6			0x01		// Local
#define STGAP4S_REGADRS_CFG7			0x15		// Remote
#define STGAP4S_REGADRS_STATUS1			0x1b		// Remote
#define STGAP4S_REGADRS_STATUS2			0x1c		// Remote
#define STGAP4S_REGADRS_STATUS3			0x0d		// Local
#define STGAP4S_REGADRS_STATUS4			0x0e		// Local
#define STGAP4S_REGADRS_STATUS5			0x1d		// Local
#define STGAP4S_REGADRS_TEST1			0x1f		// Remote
#define STGAP4S_REGADRS_DIAG1CFGA		0x17		// Remote
#define STGAP4S_REGADRS_DIAG1CFGB		0x18		// Local
#define STGAP4S_REGADRS_DIAG2CFGA		0x19		// Remote
#define STGAP4S_REGADRS_DIAG2CFGB		0x1a		// Local

/*
 *  STGAP4S register masks.
 */
#define STGAP4S_REGMASK_CFG1			0xff		//
#define STGAP4S_REGMASK_CFG2			0xff		//
#define STGAP4S_REGMASK_CFG3			0xff		//
#define STGAP4S_REGMASK_CFG4			0xff		//
#define STGAP4S_REGMASK_CFG5			0x0f		//
#define STGAP4S_REGMASK_CFG6			0xe0		//
#define STGAP4S_REGMASK_CFG7			0xff		//
#define STGAP4S_REGMASK_STATUS1			0xff		//
#define STGAP4S_REGMASK_STATUS2			0xdf		//
#define STGAP4S_REGMASK_STATUS3			0xdb		//
#define STGAP4S_REGMASK_STATUS4			0x7f		//
#define STGAP4S_REGMASK_STATUS5			0xff		//
#define STGAP4S_REGMASK_TEST1			0xff		//
#define STGAP4S_REGMASK_DIAG1CFGA		0xff		//
#define STGAP4S_REGMASK_DIAG1CFGB		0xff		//
#define STGAP4S_REGMASK_DIAG2CFGA		0xff		//
#define STGAP4S_REGMASK_DIAG2CFGB		0xff		//


/*
 *  STGAP4S requested configuration (Alvise email 2022-08-25)
 */
#define STGAP4S_REGCONF_CFG1			0b10000000		// UVLO3V3IN_EN + not SD_FLAG
#define STGAP4S_REGCONF_CFG2			0b01101100		// SAFE_OFF + DESAT_EN + DESATcur + DESATth
#define STGAP4S_REGCONF_CFG3			0b11100001		// OVLOVHth + VHONth
//$ap$ #define STGAP4S_REGCONF_CFG3			0b01101101		// OVLOVHth + VHONth
#define STGAP4S_REGCONF_CFG4			0b00000000		//
//#define STGAP4S_REGCONF_CFG5			0b00001111		// 2LTO_SOFTtime
#define STGAP4S_REGCONF_CFG5			0b00000101		// 2LTO_SOFTtime 3us
//#define STGAP4S_REGCONF_CFG6			0b00100000		// CRC_SPI
#define STGAP4S_REGCONF_CFG6			0b11100000		// CRC_SPI 600Kz
#define STGAP4S_REGCONF_CFG7			0b10101010		// ADC_HIGHth + ADC_LOWth + ADC_EN
#define STGAP4S_REGCONF_DIAG1CFGA		0b11111111		// 
#define STGAP4S_REGCONF_DIAG1CFGB		0b11000011		//
#define STGAP4S_REGCONF_DIAG2CFGA		0b00000000		//
#define STGAP4S_REGCONF_DIAG2CFGB		0b00000000		//


/*
 * The following constants map to the XPAR parameters created in the xparameters.h file. They are defined here such
 * that a user can easily change all the needed parameters in one place.
 */
#define SPI_DEVICE_ID		XPAR_PS7_SPI_0_DEVICE_ID


//
#define STGAP4S_COMMAND_MAXIMUM_LENGTH		4
#define STGAP4S_DAISY_CHAIN_DEVICES			2
#define STGAP4S_READ_REGISTER_BYTES			2
#define STGAP4S_DAISY_CHAIN_CHANNELS		3


// CS deselect time
#define STGAP4S_DELAY_tdesCS_LRD			1		// Local Register Read
#define STGAP4S_DELAY_tdesCS_RRD			30		// RemoteRegister Read
#define STGAP4S_DELAY_tdesCS_STC			1		// Start Configuration
#define STGAP4S_DELAY_tdesCS_SPC			8		// Stop Configuration
#define STGAP4S_DELAY_tdesCS_RST			8		// Reset Status / Soft Reset
#define STGAP4S_DELAY_tdesCS_LWR			1		// Local Register Write
#define STGAP4S_DELAY_tdesCS_RWR			12		// Remote Register Write
#define STGAP4S_DELAY_tdesCS_NOP			1		// NOP Command
//
#define STGAP4S_DELAY_tSDLCSL				1		// SD falling to CS falling
#define STGAP4S_DELAY_tCSHSDH				1		// CS rising to SD rising
//
#define STGAP4S_DELAY_tSDLreset		  	  100		// SD low reset time
#define STGAP4S_DELAY_tSDHmin			  100		// SD high minimum time



//
#define STGAP4S_SLOW_TASK_REFRESH_TIME		1


/*
 *  This is the size of the buffer to be transmitted/received
 */
#define STGAP4S_BUFFER_SIZE		(STGAP4S_COMMAND_MAXIMUM_LENGTH * STGAP4S_DAISY_CHAIN_DEVICES)

/*
 * The instance to support the device drivers.
 */
static XSpiPs  xSpiPs = { .IsReady = FALSE };


/*
 * The following variables are used to read and write to the SPI device.
 */
static u8 writeBuffer[STGAP4S_BUFFER_SIZE];
static u8 readBuffer[STGAP4S_BUFFER_SIZE];

// STGAP4S SD trought MIO
#define STGAP4S_SD_MIO_PORT			52
#define STGAP4S_SD_SET_LOW(...) 	GPIO_OUT(STGAP4S_SD_MIO_PORT, 0);
#define STGAP4S_SD_SET_HIGH(...) 	GPIO_OUT(STGAP4S_SD_MIO_PORT, 1);

// Add background task to scheduler
extern BOOL TaskSched_AddBackgroundTask(void (*)(void));
//
static void STgap4s_SlowTask(void);


/// <summary>
///
///</summary>
typedef struct
{
    u8	regAdrs;
    u8	regMask;
    u8	regRdly;
    u8	regWdly;

} STGAP4S_REGISTER_INFO;


/// <summary>
///
///</summary>
typedef struct
{
	union {
        struct {
        	STGAP4S_REGISTER_INFO	CFG1;
        	STGAP4S_REGISTER_INFO	CFG2;
        	STGAP4S_REGISTER_INFO	CFG3;
        	STGAP4S_REGISTER_INFO	CFG4;
        	STGAP4S_REGISTER_INFO	CFG5;
        	STGAP4S_REGISTER_INFO	CFG6;
        	STGAP4S_REGISTER_INFO	CFG7;

        	STGAP4S_REGISTER_INFO	DIAG1CFGA;
        	STGAP4S_REGISTER_INFO	DIAG1CFGB;
        	STGAP4S_REGISTER_INFO	DIAG2CFGA;
        	STGAP4S_REGISTER_INFO	DIAG2CFGB;
        } Reg;
        STGAP4S_REGISTER_INFO	Regs[11];
    };
} STGAP4S_CONFIG_REGISTER_INFOS;

/// <summary>
///
///</summary>
typedef struct
{
	union {
        struct {
        	STGAP4S_REGISTER_INFO	STATUS1;
        	STGAP4S_REGISTER_INFO	STATUS2;
        	STGAP4S_REGISTER_INFO	STATUS3;
        	STGAP4S_REGISTER_INFO	STATUS4;
        	STGAP4S_REGISTER_INFO	STATUS5;

        } Reg;
        STGAP4S_REGISTER_INFO	Regs[5];
    };
} STGAP4S_STATUS_REGISTER_INFOS;


/// <summary>
///
///</summary>
const STGAP4S_CONFIG_REGISTER_INFOS stgap4sCONFIG_INFOS =
{
	.Reg.CFG1      = {STGAP4S_REGADRS_CFG1, STGAP4S_REGMASK_CFG1, STGAP4S_DELAY_tdesCS_LRD, STGAP4S_DELAY_tdesCS_LWR},
	.Reg.CFG2      = {STGAP4S_REGADRS_CFG2, STGAP4S_REGMASK_CFG2, STGAP4S_DELAY_tdesCS_RRD, STGAP4S_DELAY_tdesCS_RWR},
	.Reg.CFG3      = {STGAP4S_REGADRS_CFG3, STGAP4S_REGMASK_CFG3, STGAP4S_DELAY_tdesCS_RRD, STGAP4S_DELAY_tdesCS_RWR},
	.Reg.CFG4      = {STGAP4S_REGADRS_CFG4, STGAP4S_REGMASK_CFG4, STGAP4S_DELAY_tdesCS_RRD, STGAP4S_DELAY_tdesCS_RWR},
	.Reg.CFG5      = {STGAP4S_REGADRS_CFG5, STGAP4S_REGMASK_CFG5, STGAP4S_DELAY_tdesCS_RRD, STGAP4S_DELAY_tdesCS_RWR},
	.Reg.CFG6      = {STGAP4S_REGADRS_CFG6, STGAP4S_REGMASK_CFG6, STGAP4S_DELAY_tdesCS_LRD, STGAP4S_DELAY_tdesCS_LWR},
	.Reg.CFG7      = {STGAP4S_REGADRS_CFG7, STGAP4S_REGMASK_CFG7, STGAP4S_DELAY_tdesCS_RRD, STGAP4S_DELAY_tdesCS_RWR},

	.Reg.DIAG1CFGA = {STGAP4S_REGADRS_DIAG1CFGA, STGAP4S_REGMASK_DIAG1CFGA, STGAP4S_DELAY_tdesCS_RRD, STGAP4S_DELAY_tdesCS_RWR},
	.Reg.DIAG1CFGB = {STGAP4S_REGADRS_DIAG1CFGB, STGAP4S_REGMASK_DIAG1CFGB, STGAP4S_DELAY_tdesCS_LRD, STGAP4S_DELAY_tdesCS_LWR},
	.Reg.DIAG2CFGA = {STGAP4S_REGADRS_DIAG2CFGA, STGAP4S_REGMASK_DIAG2CFGA, STGAP4S_DELAY_tdesCS_RRD, STGAP4S_DELAY_tdesCS_RWR},
	.Reg.DIAG2CFGB = {STGAP4S_REGADRS_DIAG2CFGB, STGAP4S_REGMASK_DIAG2CFGB, STGAP4S_DELAY_tdesCS_LRD, STGAP4S_DELAY_tdesCS_LWR}
};

/// <summary>
///
///</summary>
const STGAP4S_CONFIG_REGISTER_PARAMS stgap4sCONFIG_PARAMSdefault =
{
	.Reg.CFG1      = STGAP4S_REGCONF_CFG1,
	.Reg.CFG2      = STGAP4S_REGCONF_CFG2,
	.Reg.CFG3      = STGAP4S_REGCONF_CFG3,
	.Reg.CFG4      = STGAP4S_REGCONF_CFG4,
	.Reg.CFG5      = STGAP4S_REGCONF_CFG5,
	.Reg.CFG6      = STGAP4S_REGCONF_CFG6,
	.Reg.CFG7      = STGAP4S_REGCONF_CFG7,

	.Reg.DIAG1CFGA = STGAP4S_REGCONF_DIAG1CFGA,
	.Reg.DIAG1CFGB = STGAP4S_REGCONF_DIAG1CFGB,
	.Reg.DIAG2CFGA = STGAP4S_REGCONF_DIAG2CFGA,
	.Reg.DIAG2CFGB = STGAP4S_REGCONF_DIAG2CFGB
};

/// <summary>
///
///</summary>
STGAP4S_CONFIG_REGISTER_PARAMS stgap4sCONFIG_PARAMS;


/// <summary>
///
///</summary>
const STGAP4S_STATUS_REGISTER_INFOS stgap4sSTATUS_INFOS =
{
	.Reg.STATUS1 = {STGAP4S_REGADRS_STATUS1, STGAP4S_REGMASK_STATUS1, STGAP4S_DELAY_tdesCS_RRD, STGAP4S_DELAY_tdesCS_RWR},
	.Reg.STATUS2 = {STGAP4S_REGADRS_STATUS2, STGAP4S_REGMASK_STATUS2, STGAP4S_DELAY_tdesCS_RRD, STGAP4S_DELAY_tdesCS_RWR},
	.Reg.STATUS3 = {STGAP4S_REGADRS_STATUS3, STGAP4S_REGMASK_STATUS3, STGAP4S_DELAY_tdesCS_LRD, STGAP4S_DELAY_tdesCS_LWR},
	.Reg.STATUS4 = {STGAP4S_REGADRS_STATUS4, STGAP4S_REGMASK_STATUS4, STGAP4S_DELAY_tdesCS_LRD, STGAP4S_DELAY_tdesCS_LWR},
	.Reg.STATUS5 = {STGAP4S_REGADRS_STATUS5, STGAP4S_REGMASK_STATUS5, STGAP4S_DELAY_tdesCS_LRD, STGAP4S_DELAY_tdesCS_LWR}
};


/// <summary>
///
///</summary>
static BOOL stgap4sInitialized = FALSE;
static BOOL stgap4sRunningConf = FALSE;


/// <summary>
/// Field bus public variables
///</summary>
u8 stgap4sSoftReset = FALSE;
u8 stgap4sResetStatus = FALSE;
u8 stgap4sConfigDrivers = FALSE;
u32 stgap4sTotalBytesTransferred = 0;
u32 stgap4sTotalWrongRxCRCerrors = 0;

u8 stgap4sConfigValuesIndex = 0;

STGAP4S_SYSTEM_STATUS_REGISTER_VALUES stgap4sSTATUS_VALUES;
STGAP4S_CONFIG_REGISTER_PARAMS stgap4sCONFIG_VALUES;



/// <summary>
/// CRC-8/ROHC : x^8 + x^2 + x^1 + 1
///</summary>
//
#define CRC8_POLYNOMIAL 	0x07
#define CRC8_INTIALSEED 	0xff
//
static u8 __crc8(u8 crc, u8 dat)
{
	crc = crc ^ dat;

	for (u8 i=0; i<8; i++)
	{
		if (crc & 0x80)
			crc = (crc << 1) ^ CRC8_POLYNOMIAL;
		else
			crc = (crc << 1);
	}
	return crc;
}



/// <summary>
///
///</summary>
static int SendDaisyChainCommand(u8 command)
{
	//
	u8 commCRC = __crc8(CRC8_INTIALSEED, command);

	// Prepare data buffer.
	for (int device = 0; device < STGAP4S_DAISY_CHAIN_DEVICES * 2; device += 2) {
		writeBuffer[device + 0] = command;
		writeBuffer[device + 1] = ~commCRC;
	}

	// Transmit the data.
	XSpiPs_PolledTransfer(&xSpiPs, writeBuffer, readBuffer, STGAP4S_DAISY_CHAIN_DEVICES * 2);

	// Update statistics
	stgap4sTotalBytesTransferred += 2 * STGAP4S_DAISY_CHAIN_DEVICES;

	//
	return XST_SUCCESS;
}


/// <summary>
///
///</summary>
static int WriteDaisyChainRegisters(u8 regadrs, u8 regdata)
{
	//
	u8 commCRC = __crc8(CRC8_INTIALSEED, STGAP4S_COMMAND_WRITE_REGISTER | regadrs);

	// Prepare data buffer for write register command
	for (int device = 0; device < STGAP4S_DAISY_CHAIN_DEVICES * 2; device += 2) {
		writeBuffer[device + 0] = STGAP4S_COMMAND_WRITE_REGISTER | regadrs;
		writeBuffer[device + 1] = ~commCRC;
	}

	// Transmit the data.
	XSpiPs_PolledTransfer(&xSpiPs, writeBuffer, readBuffer, STGAP4S_DAISY_CHAIN_DEVICES * 2);



	//
	u8 dataCRC = __crc8(commCRC, regdata);

	// Prepare data buffer for register value
	for (int device = 0; device < STGAP4S_DAISY_CHAIN_DEVICES * 2; device += 2) {
		writeBuffer[device + 0] = regdata;
		writeBuffer[device + 1] = ~dataCRC;
	}

	// Transmit the data.
	XSpiPs_PolledTransfer(&xSpiPs, writeBuffer, readBuffer, STGAP4S_DAISY_CHAIN_DEVICES * 2);

	// Update statistics
	stgap4sTotalBytesTransferred += 4 * STGAP4S_DAISY_CHAIN_DEVICES;

	//
	return XST_SUCCESS;
}


/// <summary>
///
///</summary>
static int ReadDaisyChainRegisters(u8 regadrs, u8 regdelay, u8 regdatas[])
{
	// Send Read Register command
	SendDaisyChainCommand(STGAP4S_COMMAND_READ_REGISTER | regadrs);

	// Wait result
	usleep(regdelay);

	// Send No Operation command to read results
	SendDaisyChainCommand(STGAP4S_COMMAND_NO_OPERATION);
	usleep(STGAP4S_DELAY_tdesCS_NOP);

	//
	BOOL bWrongCRC = FALSE;

	// Copy results taking into account that daisy chain revert byte order 
	for (int device = 0; device < STGAP4S_DAISY_CHAIN_DEVICES; device++)
	{
		u8 data = readBuffer[(device * STGAP4S_READ_REGISTER_BYTES) + 0];
		u8 crc8 = readBuffer[(device * STGAP4S_READ_REGISTER_BYTES) + 1];
		if (crc8 == __crc8(CRC8_INTIALSEED, data))
		{
		 	regdatas[STGAP4S_DAISY_CHAIN_DEVICES - device - 1] = data;
		}
		else
		{
			bWrongCRC = TRUE;
			stgap4sTotalWrongRxCRCerrors++;
		 	regdatas[STGAP4S_DAISY_CHAIN_DEVICES - device - 1] = 0x00;
	    }
	}

	//
	if (bWrongCRC)
		return XST_RECV_ERROR;
	//
	return XST_SUCCESS;
}

/// <summary>
///
///</summary>
int STgap4s_HardwareReset(void)
{
	STGAP4S_SD_SET_LOW();
	usleep(STGAP4S_DELAY_tSDLreset);
	STGAP4S_SD_SET_HIGH();
	usleep(STGAP4S_DELAY_tSDHmin);

	//
	return XST_SUCCESS;
}


/// <summary>
///
///</summary>
BOOL STgap4s_Initialize(void)
{
	//
	uint8_t clk_pha = 1;
	uint8_t clk_pol = 0;
	uint8_t spi_csd = 0;

	// Clear Status register values to known state
	for (u8 slaveSel = 0; slaveSel < STGAP4S_DAISY_CHAIN_CHANNELS; slaveSel++)
	{
		//
		for (u8 statusReg = 0; statusReg < sizeof(stgap4sSTATUS_INFOS.Regs) / sizeof(STGAP4S_REGISTER_INFO); statusReg++)
		{
			stgap4sSTATUS_VALUES.Phases[(slaveSel * STGAP4S_DAISY_CHAIN_DEVICES) + 0].Regs[statusReg] = 0x77;
			stgap4sSTATUS_VALUES.Phases[(slaveSel * STGAP4S_DAISY_CHAIN_DEVICES) + 1].Regs[statusReg] = 0x88;
		}
	}
	// Clear Config register values to known state
	for (u8 cfgReg = 0; cfgReg < sizeof(stgap4sCONFIG_INFOS.Regs) / sizeof(STGAP4S_REGISTER_INFO); cfgReg++)
		stgap4sCONFIG_VALUES.Regs[cfgReg] = 0x99;


	// STGAP4S SD trought MIO
    Gpio_SetMode(STGAP4S_SD_MIO_PORT, GPIO_DIR_OUT);
	STGAP4S_SD_SET_HIGH();
	usleep(STGAP4S_DELAY_tSDHmin);

    int status;

	// Pointer to Configuration data
	XSpiPs_Config * configPtr;

	// Initialize the SPI driver so that it is  ready to use.
	configPtr = XSpiPs_LookupConfig(SPI_DEVICE_ID);
	if (configPtr == NULL) {
		return XST_DEVICE_NOT_FOUND;
	}
	status = XSpiPs_CfgInitialize(&xSpiPs, configPtr, configPtr->BaseAddress);
	if (status != XST_SUCCESS)
		return FALSE;

	// Sets the clock prescaler
	status = XSpiPs_SetClkPrescaler(&xSpiPs, XSPIPS_CLK_PRESCALE_64);
	if (status != XST_SUCCESS)
		return FALSE;

	// Set SPI delays : DelayAfter = 200nS between bytes, DelayInit = 400ns between CS and first CK
	u8 DelayNss = 0, DelayBtwn = 0, DelayAfter = 49, DelayInit = 79;
	status = XSpiPs_SetDelays(&xSpiPs, DelayNss, DelayBtwn, DelayAfter, DelayInit);
	if (status != XST_SUCCESS)
		return FALSE;

	// The SPI device is a master
	status = XSpiPs_SetOptions(&xSpiPs, XSPIPS_MASTER_OPTION | (clk_pol ? XSPIPS_CLK_ACTIVE_LOW_OPTION : 0) |
			                                                   (clk_pha ? XSPIPS_CLK_PHASE_1_OPTION : 0) |
			                                                   (spi_csd ? XSPIPS_DECODE_SSELECT_OPTION : 0) |
															   XSPIPS_MANUAL_START_OPTION);
	if (status != XST_SUCCESS)
		return FALSE;

	// in case of power on reset
	STgap4s_HardwareReset();

	//
    stgap4sInitialized = TRUE;

	// in case of button or software reset (Cockpit, PLC, etc...)
	STgap4s_SoftReset();

	// Install background function
    if (!TaskSched_AddBackgroundTask(&STgap4s_SlowTask))
        return FALSE ;

    //
	return TRUE;
}


/// <summary>
///
///</summary>
BOOL STgap4s_ConfigDrivers(void)
{
	//
	if (!stgap4sInitialized)
		return FALSE;

	//
	stgap4sRunningConf = TRUE;

	//
	for (u8 slaveSel = 0; slaveSel < STGAP4S_DAISY_CHAIN_CHANNELS; slaveSel++)
	{
		//
#if !defined(STGAP4S_TEST_DAISY_CHAIN_REVERSED) && !defined(STGAP4S_TEST_DAISY_CHAIN_SHUFFLED)
		XSpiPs_SetSlaveSelect(&xSpiPs, slaveSel);
#else
#if defined(STGAP4S_TEST_DAISY_CHAIN_REVERSED)
		XSpiPs_SetSlaveSelect(&xSpiPs, STGAP4S_DAISY_CHAIN_CHANNELS - slaveSel - 1);
#elif defined(STGAP4S_TEST_DAISY_CHAIN_SHUFFLED)
		XSpiPs_SetSlaveSelect(&xSpiPs, slaveSel == 0 ? 0 : slaveSel == 1 ? 2 : 1);
#endif
#endif

		// Set SD Low
		STGAP4S_SD_SET_LOW();
		usleep(STGAP4S_DELAY_tSDLCSL);

		// Reset Status
		SendDaisyChainCommand(STGAP4S_COMMAND_RESET_STATUS);
		usleep(STGAP4S_DELAY_tdesCS_RST);

		// Start Config
		SendDaisyChainCommand(STGAP4S_COMMAND_START_CONFIG);
		usleep(STGAP4S_DELAY_tdesCS_STC);

		// Write configuration registers
		for (u8 confReg = 0; confReg < sizeof(stgap4sCONFIG_INFOS.Regs) / sizeof(STGAP4S_REGISTER_INFO); confReg++)
		{
			WriteDaisyChainRegisters(stgap4sCONFIG_INFOS.Regs[confReg].regAdrs, stgap4sCONFIG_PARAMS.Regs[confReg]);
			usleep(stgap4sCONFIG_INFOS.Regs[confReg].regWdly);
		}

		// Stop Config
		SendDaisyChainCommand(STGAP4S_COMMAND_STOP_CONFIG);
		usleep(STGAP4S_DELAY_tdesCS_SPC);

		// Read back and check CFG

		// Set SD High
		STGAP4S_SD_SET_HIGH();
		usleep(STGAP4S_DELAY_tSDHmin);
	}

    //
	stgap4sRunningConf = FALSE;

	//
	return TRUE;
}


/// <summary>
///
///</summary>
BOOL STgap4s_ResetStatus(void)
{
	//
	if (!stgap4sInitialized)
		return FALSE;

	//
	for (u8 slaveSel = 0; slaveSel < STGAP4S_DAISY_CHAIN_CHANNELS; slaveSel++)
	{
		//
		XSpiPs_SetSlaveSelect(&xSpiPs, slaveSel);

		// Set SD Low
		STGAP4S_SD_SET_LOW();
		usleep(STGAP4S_DELAY_tSDLCSL);

		// Reset Status
		SendDaisyChainCommand(STGAP4S_COMMAND_RESET_STATUS);
		usleep(STGAP4S_DELAY_tdesCS_RST);

		// Set SD High
		STGAP4S_SD_SET_HIGH();
		usleep(STGAP4S_DELAY_tSDHmin);

	}

	//
	return TRUE;
}

/// <summary>
///
///</summary>
BOOL STgap4s_SoftReset(void)
{
	//
	if (!stgap4sInitialized)
		return FALSE;

	//
	for (u8 slaveSel = 0; slaveSel < STGAP4S_DAISY_CHAIN_CHANNELS; slaveSel++)
	{
		//
		XSpiPs_SetSlaveSelect(&xSpiPs, slaveSel);

		// Set SD Low
		STGAP4S_SD_SET_LOW();
		usleep(STGAP4S_DELAY_tSDLCSL);

		// Start Config
		SendDaisyChainCommand(STGAP4S_COMMAND_START_CONFIG);
		usleep(STGAP4S_DELAY_tdesCS_STC);

		// Soft Reset
		SendDaisyChainCommand(STGAP4S_COMMAND_SOFT_RESET);
		usleep(STGAP4S_DELAY_tdesCS_RST);

		// Stop Config
		SendDaisyChainCommand(STGAP4S_COMMAND_STOP_CONFIG);
		usleep(STGAP4S_DELAY_tdesCS_SPC);

		// Set SD High
		STGAP4S_SD_SET_HIGH();
		usleep(STGAP4S_DELAY_tSDHmin);

	}

	//
	return TRUE;
}








/// <summary>
///
///</summary>
BOOL STgap4s_ReadStatusRegisters(void)
{
	//
	if (!stgap4sInitialized)
		return FALSE;

	//
	for (u8 slaveSel = 0; slaveSel < STGAP4S_DAISY_CHAIN_CHANNELS; slaveSel++)
	{
		//
		XSpiPs_SetSlaveSelect(&xSpiPs, slaveSel);

		// Read status registers
		for (u8 statusReg = 0; statusReg < sizeof(stgap4sSTATUS_INFOS.Regs) / sizeof(STGAP4S_REGISTER_INFO); statusReg++)
		{
			u8 regDatas[STGAP4S_DAISY_CHAIN_DEVICES];
			ReadDaisyChainRegisters(stgap4sSTATUS_INFOS.Regs[statusReg].regAdrs, stgap4sSTATUS_INFOS.Regs[statusReg].regRdly, regDatas);

			stgap4sSTATUS_VALUES.Phases[(slaveSel * STGAP4S_DAISY_CHAIN_DEVICES) + 0].Regs[statusReg] = regDatas[0] & stgap4sSTATUS_INFOS.Regs[statusReg].regMask;
			stgap4sSTATUS_VALUES.Phases[(slaveSel * STGAP4S_DAISY_CHAIN_DEVICES) + 1].Regs[statusReg] = regDatas[1] & stgap4sSTATUS_INFOS.Regs[statusReg].regMask;
		}
	}

	//
	return TRUE;
}


/// <summary>
///
///</summary>
BOOL STgap4s_ReadConfigRegisters(u8 gdidx)
{
	//
	if (!stgap4sInitialized)
		return FALSE;

	//
	if (gdidx >= 3 * STGAP4S_DAISY_CHAIN_DEVICES)
	{
		// Clear CFG register values
		for (u8 cfgReg = 0; cfgReg < sizeof(stgap4sCONFIG_INFOS.Regs) / sizeof(STGAP4S_REGISTER_INFO); cfgReg++)
			stgap4sCONFIG_VALUES.Regs[cfgReg] = 0xff;
		return FALSE;
	}

	//
	XSpiPs_SetSlaveSelect(&xSpiPs, gdidx / 2);

#ifdef STGAP4S_TRACE_READ_CONFIG_REGISTERS
	xil_printf("stgap4s::STgap4s_ReadConfigRegisters(%u)\r\n", gdidx);
#endif


	// Read CFG registers
	for (u8 cfgReg = 0; cfgReg < sizeof(stgap4sCONFIG_INFOS.Regs) / sizeof(STGAP4S_REGISTER_INFO); cfgReg++)
	{
		u8 regDatas[STGAP4S_DAISY_CHAIN_DEVICES];
		int status = ReadDaisyChainRegisters(stgap4sCONFIG_INFOS.Regs[cfgReg].regAdrs, stgap4sCONFIG_INFOS.Regs[cfgReg].regRdly, regDatas);

		if ((gdidx & 0x01) == 0)
			stgap4sCONFIG_VALUES.Regs[cfgReg] = regDatas[0] & stgap4sCONFIG_INFOS.Regs[cfgReg].regMask;
		else
			stgap4sCONFIG_VALUES.Regs[cfgReg] = regDatas[1] & stgap4sCONFIG_INFOS.Regs[cfgReg].regMask;

#ifdef STGAP4S_TRACE_READ_CONFIG_REGISTERS
		xil_printf("  %02u --> %02X, %02X : %d\r\n", cfgReg, regDatas[0], regDatas[1], status);
#endif

	}

	//
	return TRUE;
}


/// <summary>
///
///</summary>
static void STgap4s_SlowTask(void)
{
	static ULONG ulNextRefreshTime = 0;

	//
	if (!stgap4sInitialized || stgap4sRunningConf)
		return;

	//
	if (ulSysTimersTotalPowerOnTime >= ulNextRefreshTime)
	{
		//
		ulNextRefreshTime = ulSysTimersTotalPowerOnTime + STGAP4S_SLOW_TASK_REFRESH_TIME;

		//
		if (stgap4sResetStatus)
		{
			STgap4s_ResetStatus();
			stgap4sResetStatus = FALSE;
			return;
		}

		//
		if (stgap4sConfigDrivers)
		{
			STgap4s_ConfigDrivers();
			stgap4sConfigDrivers = FALSE;
			return;
		}

		//
		if (stgap4sSoftReset)
		{
			STgap4s_SoftReset();
			stgap4sSoftReset = FALSE;
			return;
		}

		//
		STgap4s_ReadStatusRegisters();

#ifdef STGAP4S_READ_BACK_CONFIG_REGISTERS
		//
		STgap4s_ReadConfigRegisters(stgap4sConfigValuesIndex);
#endif

	}
}

#endif // _HW_AXS_CABI35KW
