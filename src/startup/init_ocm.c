/*
 * init_ocm.cpp
 *
 *  Created on: 3 Dec 2020
 *      Author: MatteoDemartini
 */

#include <string.h>

extern char _dataLMA, __data_start, __data_end;
extern char __bss_start, __bss_end;

static void copy(char *src, char *dstStart, char *dstEnd) {
	/* ROM has data at end of text; copy it. */
	while (dstStart < dstEnd)
	{

		*dstStart++ = *src++;
		asm volatile ("dsb sy");

	}
}

void init_ocm(void) {
	// copy the data section from L2CACHE(load region) to OCM(Execution region) memory region.
	copy(&_dataLMA,&__data_start,&__data_end);

	// Init bss to 0
	memset(&__bss_start, 0, (&__bss_end - &__bss_start));
}

