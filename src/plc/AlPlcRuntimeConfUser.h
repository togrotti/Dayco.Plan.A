#ifndef ALPLCRUNTIMECONFUSER_H_
#define ALPLCRUNTIMECONFUSER_H_

#include "PlcRT.h"

#define ALPLCAREA_CODE_SIZE            	PLCD_RT_CODE_SIZE		/*	Size of code area(s)  */
extern uint8_t m_PlcCodeArea1[];

bool_t WrPlc(void* data, uint32_t off, uint32_t len);
bool_t RdPlc(void* data, uint32_t off, uint32_t len);
bool_t RdPlcInit(void);
void RdPlcEnd(void);
bool_t WrPlcInit(void);
void WrPlcEnd(void);

void InvalidateCache();

#endif	//	ALPLCRUNTIMECONFUSER_H_
