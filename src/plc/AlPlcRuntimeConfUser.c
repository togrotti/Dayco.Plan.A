#pragma GCC optimize (2)

#include "plc/AlPlcRuntimeConfUser.h"
#include "core/Flash.h"
#include "xil_cache.h"
#include "xil_cache_l.h"

uint8_t m_PlcCodeArea1[ALPLCAREA_CODE_SIZE] __attribute__((section(".plc_code_section")));

#define ALPLCAREA_CODE_FLASH_ADDRESS		PLCD_LK_CODE_START
#define ALPLCAREA_CODE_FLASH_SIZE			PLCD_LK_CODE_SIZE

// DDRless system, invalidate L1 cache only since L2 cache is used for program execution
void InvalidateCache() {
	Xil_L1DCacheFlush();
	Xil_ICacheInvalidateRange((INTPTR)m_PlcCodeArea1, ALPLCAREA_CODE_SIZE);
}

/* -------------------------------------------------------------------------------------------- */
/* -------------------------------------- STORAGE --------------------------------------------- */
/* -------------------------------------------------------------------------------------------- */

/* Function used to save PLC code into permanent memory */

//  flash must be written using 128 blocks
#define FLASH_BLOCK_WRITE 128
uint8_t m_WrPlcBuffer[ FLASH_BLOCK_WRITE ];

bool_t WrPlc(void* data, uint32_t off, uint32_t len)
{
	uint32_t i;
	uint32_t pDest;

	if ( off != 0 )
		return FALSE;

	if ( len > ALPLCAREA_CODE_SIZE )
		return FALSE;

	for ( i = 0, pDest = ALPLCAREA_CODE_FLASH_ADDRESS; i < len; i += FLASH_BLOCK_WRITE, pDest += FLASH_BLOCK_WRITE )
	{
		uint32_t pData = (uint32_t)data + i;

		if ( ( len - i ) >= FLASH_BLOCK_WRITE )
			memcpy( (void *)m_WrPlcBuffer, (void *)pData, FLASH_BLOCK_WRITE );
		else
			memcpy( (void *)m_WrPlcBuffer, (void *)pData, ( len - i ) );

		FlashWrite( pDest, m_WrPlcBuffer, FLASH_BLOCK_WRITE );
	}

	return TRUE;
}

/* Function used to load PLC code from permanent memory */
bool_t RdPlc(void* data, uint32_t off, uint32_t len)
{
	if (len > PLCD_LK_CODE_SIZE) return FALSE;

	uint32_t address = ALPLCAREA_CODE_FLASH_ADDRESS + off;
	FlashRead(address, (uint8_t*)data, len);

	return TRUE;
}

/* Function used to initialize permanent memory read */
bool_t RdPlcInit(void)
{
	return TRUE;
}
/* Function used to terminate permanent memory read */
void RdPlcEnd(void)
{
	// This is typically called after PLC code has been copied from flash to RAM
	InvalidateCache();
}
/* Function used to initialize permanent memory writing */
bool_t WrPlcInit(void)
{
	FlashErase( ALPLCAREA_CODE_FLASH_ADDRESS, ALPLCAREA_CODE_FLASH_SIZE );
	return TRUE;
}
/* Function used to terminate permanent memory writing */
void WrPlcEnd(void)
{
	return;
}
