/*-----------------------------------------------------------------------------------------
------	
------	ecat_def.h
------
-----------------------------------------------------------------------------------------*/

#ifndef _ECATDEF_H_
#define _ECATDEF_H_

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/

#include "drive\AxM-E-Defines.h"

/*-----------------------------------------------------------------------------------------
------	
------	Defines and Types
------	
-----------------------------------------------------------------------------------------*/

/*/////////////////////////////////////////////////////////////
//
//	Compiler defines
//
// The compiler defines are used to adapt the used microcontroller
// to the slave sample code. 
*/

/* BOOL should be adapted to the boolean type of the microcontroller */
//#define	BOOL								unsigned char
/* UINT8 should be adapted to the unsigned8 type of the microcontroller */
#define	UINT8								unsigned char
/* UINT16 should be adapted to the unsigned16 type of the microcontroller */
#define	UINT16								unsigned short int
/* UINT32 should be adapted to the unsigned32 type of the microcontroller */
#define	UINT32								unsigned int
/* USHORT should be adapted to the unsigned16 type of the microcontroller */
#define	USHORT								unsigned short
/* INT8 should be adapted to the integer8 type of the microcontroller */
#define	INT8								char
/* INT16 should be adapted to the integer16 type of the microcontroller */
#define	INT16								int
/* INT32 should be adapted to the integer32 type of the microcontroller */
#define	INT32								long
/* CHAR should be adapted to the character type of the microcontroller */
#define	CHAR								char
/* UCHAR should be adapted to the unsigned character type of the microcontroller */
#define	UCHAR								unsigned char
/* FAR should be adapted to the far type of the microcontroller, if the microcontroller
   does not support a far type, FAR shall be defined to nothing */
#define	FAR								    huge
/* HUGE should be adapted to the huge type of the microcontroller, if the microcontroller
   does not support a huge type, HUGE shall be defined to nothing */
#define	HUGE								huge
/* HMEMSET should be defined to the memset function for huge memory, if the microcontroller
   does not support a huge type, HMEMSET shall be defined to a "normal" memset function */
#define	HMEMSET								memset
/* HMEMCPY should be defined to the memcpy function for huge memory, if the microcontroller
   does not support a huge type, HMEMCPY shall be defined to a "normal" memcpy function */
#define	HMEMCPY								memcpy
/* HMEMCMP should be defined to the memcmp function for huge memory, if the microcontroller
   does not support a huge type, HMEMCMP shall be defined to a "normal" memcmp function */
#define	HMEMCMP								memcmp
/* ESCMEM should be defined to select the memory type of the ESC memory (f.e. near, far or huge), if the microcontroller
   does not support different memory types, ESCMEM shall be defined to nothing */
#define	ESCMEM							
/* ESCMEMCPY should be defined to the memcpy function for ESCMEM memory, if the microcontroller
   does not support different memory types, ESCMEMCPY shall be defined to a "normal" memcpy function */
#define	ESCMEMCPY							memcpy
/* ESCMEMSET should be defined to the memset function for ESCMEM memory, if the microcontroller
   does not support different memory types, ESCMEMSET shall be defined to a "normal" memset function */
#define	ESCMEMSET							memset
/* ESCMBXMEMCPY should be defined to the memcpy function for copying ESCMEM memory to or from MBXMEM memory, if the microcontroller
   does not support different memory types, ESCMBXMEMCPY shall be defined to a "normal" memcpy function */
#define	ESCMBXMEMCPY						memcpy
/* MBXMEM should be defined to select the memory type of the memory used for mailbox communication (f.e. near, far or huge), 
   if the microcontroller does not support different memory types, MBXMEM shall be defined to nothing */
#define	MBXMEM							    
/* MBXMEMCPY should be defined to the memcpy function for MBXMEM memory, if the microcontroller
   does not support different memory types, MBXMEMCPY shall be defined to a "normal" memcpy function */
#define	MBXMEMCPY							memcpy
/* MBXMEMCMP should be defined to the memcmp function for MBXMEM memory, if the microcontroller
   does not support different memory types, MBXMEMCMP shall be defined to a "normal" memcmp function */
#define	MBXMEMCMP							memcmp
/* MBXMEMSET should be defined to the memcpy function for MBXMEM memory, if the microcontroller
   does not support different memory types, MBXMEMSET shall be defined to a "normal" memset function */
#define	MBXMEMSET							memset
/* MBXSTRLEN should be defined to the strlen function for MBXMEM memory, if the microcontroller
   does not support different memory types, MBXSTRLEN shall be defined to a "normal" strlen function */
#define	MBXSTRLEN							strlen
/* MBXSTRCPY should be defined to the strcpy function for MBXMEM memory, if the microcontroller
   does not support different memory types, MBXSTRCPY shall be defined to a "normal" strcpy function */
#define	MBXSTRCPY							memcpy
/* OBJCONST should be used to define the object dictionary in ROM (f.e. define OBJCONST const) or
   in RAM (f.e. define OBJCONST) */
#define	OBJCONST							const
/* OBJCONST should be used to define the constant variables in ROM (f.e. define VARCONST const) or
   in RAM (f.e. define VARCONST) */
#define	VARCONST							
/* OBJMEM should be defined to select the memory type of the memory used for the object dictionary (f.e. near, far or huge), 
   if the microcontroller does not support different memory types, OBJMEM shall be defined to nothing */
#define	OBJMEM							
/* OBJTOMBXMEMCPY should be defined to the memcpy function for copying OBJMEM memory to MBXMEM memory, if the microcontroller
   does not support different memory types, OBJTOMBXMEMCPY shall be defined to a "normal" memcpy function */
#define	OBJTOMBXMEMCPY						memcpy
/* OBJTOMBXSTRCPY should be defined to the strcpy function for copying OBJMEM memory to MBXMEM memory, if the microcontroller
   does not support different memory types, OBJTOMBXSTRCPY shall be defined to a "normal" memcpy function */
#define	OBJTOMBXSTRCPY						memcpy
/* OBJMEMCPY should be defined to the memcpy function for OBJMEM memory, if the microcontroller
   does not support different memory types, OBJMEMCPY shall be defined to a "normal" memcpy function */
#define	OBJMEMCPY							memcpy
/* OBJSTRLEN should be defined to the strlen function for OBJMEM memory, if the microcontroller
   does not support different memory types, OBJSTRLEN shall be defined to a "normal" strlen function */
#define	OBJSTRLEN							strlen
/* OBJSTRCPY should be defined to the strcpy function for OBJMEM memory, if the microcontroller
   does not support different memory types, OBJSTRCPY shall be defined to a "normal" strcpy function */
#define	OBJSTRCPY							memcpy
/* MAKE_HUGE_PTR should be defined to the initialize a pointer variable with an absolute address */
#define	MAKE_HUGE_PTR					
/* MAKE_PTR_TO_ESC should be defined to the initialize the pointer to the ESC */
#define	MAKE_PTR_TO_ESC				
/* EMCYMEMCPY should be defined to the memcpy function for EMCYMEM memory, if the microcontroller
   does not support different memory types, EMCYMEMCPY shall be defined to a "normal" memcpy function */
#define	EMCYMEMCPY							memcpy
/* EMCYMEMSET should be defined to the memset function for EMCYMEM memory, if the microcontroller
   does not support different memory types, EMCYMEMSET shall be defined to a "normal" memcset function */
#define	EMCYMEMSET							memset
/* EMCYMEM should be defined to select the memory type of the memory used for the emergencies (f.e. near, far or huge), 
   if the microcontroller does not support different memory types, EMCYMEM shall be defined to nothing */
#define	EMCYMEM							
/* ALLOCMEM should be defined to the alloc function to get dynamic memory */
#undef	ALLOCMEM							
/* FREEMEM should be defined to the free function to put back dynamic memory */
#undef	FREEMEM								
/* VARMEM should be defined to select the memory type of the memory used for dynamic memory (f.e. near, far or huge), 
   if the microcontroller does not support different memory types, VARMEM shall be defined to nothing */
#define	VARMEM							
/* APPL_AllocMailboxBuffer should be defined to a function to get a buffer for a mailbox service,
   this is only used if the switch MAILBOX_QUEUE is set */
#define	APPL_AllocMailboxBuffer		
/* APPL_FreeMailboxBuffer should be defined to a function to put back a buffer for a mailbox service,
   this is only used if the switch MAILBOX_QUEUE is set */
#define	APPL_FreeMailboxBuffer		
/* STRUCT_PACKED is defined after the typedef struct construct to pack the structures if necessary */
#define	STRUCT_PACKED

/*/////////////////////////////////////////////////////////////
//
//	Application specific defines
//
// The application specific defines are used to adapt the EtherCAT
// Slave Stack to the application requirements
*/

/* MIN_PD_WRITE_ADDRESS: minimum address for the process output data (Sync Manager 2)
   inside the application memory of the EtherCAT Slave Controller which could be set by the master */
#define	MIN_PD_WRITE_ADDRESS			0x1000
/* MAX_PD_WRITE_ADDRESS: maximum address for the process output data (Sync Manager 2)
   inside the application memory of the EtherCAT Slave Controller which could be set by the master */
#define	MAX_PD_WRITE_ADDRESS			0x2000
/* MIN_PD_READ_ADDRESS: minimum address for the process input data (Sync Manager 3)
   inside the application memory of the EtherCAT Slave Controller which could be set by the master */
#define	MIN_PD_READ_ADDRESS				0x1000
/* MAX_PD_READ_ADDRESS: maximum address for the process input data (Sync Manager 3)
   inside the application memory of the EtherCAT Slave Controller	which could be set by the master */
#define	MAX_PD_READ_ADDRESS				0x2000
/* MIN_MBX_SIZE: minimum mailbox size (Sync Manager 0 and 1) which could be set by the master */
#define	MIN_MBX_SIZE					0x0080
/* MAX_MBX_SIZE: maximum mailbox size (Sync Manager 0 and 1) which could be set by the master */
#define	MAX_MBX_SIZE					0x0080
/* MIN_MBX_WRITE_ADDRESS: minimum address for the write (receive) mailbox (Sync Manager 0) */
#define	MIN_MBX_WRITE_ADDRESS			0x1000
/* MAX_MBX_WRITE_ADDRESS: maximum address for the write (receive) mailbox (Sync Manager 0) */
#define	MAX_MBX_WRITE_ADDRESS			0x2000
/* MIN_MBX_READ_ADDRESS: minimum address for the read (send) mailbox (Sync Manager 1) */
#define	MIN_MBX_READ_ADDRESS			0x1000
/* MAX_MBX_READ_ADDRESS: maximum address for the read (send) mailbox (Sync Manager 1) */
#define	MAX_MBX_READ_ADDRESS			0x2000
/* DEF_MBX_WRITE_ADDRESS: default address for the write (receive) mailbox (Sync Manager 0) */
#define	DEF_MBX_WRITE_ADDRESS			0x1300
/* DEF_MBX_READ_ADDRESS: default address for the read (send) mailbox (Sync Manager 1) */
#define	DEF_MBX_READ_ADDRESS			DEF_MBX_WRITE_ADDRESS+MAX_MBX_SIZE
/* MAX_EMERGENCIES: number of emergencies supported in parallel */
#define	MAX_EMERGENCIES					0x0005
/* MIN_PD_CYCLE_TIME: minimum cycle time in ns the slave is supporting 
   (entry 0x1C32:05 or entry 0x1C33:05) (250 μs) */
#define	MIN_PD_CYCLE_TIME				(1000000000/REALTIME_TASK_FREQ*2)
/* MAX_PD_CYCLE_TIME: maximum cycle time in ns the slave is supporting (64 ms) */
#define	MAX_PD_CYCLE_TIME				64000000
/* Granularity of cycle time (125 us) */
#define GRANULARITY_PD_CYCLE_TIME       (1000000000/REALTIME_TASK_FREQ)
/* TIMER_TICKS_MS: one milli second in number of timer ticks */
#define	TIMER_TICKS_MS					0x1770
/* MAX_PD_INPUT_SIZE: maximum size of the process input data (Sync Manager 3) for cyclic exchange */
#define	MAX_PD_INPUT_SIZE				0x0040
/* MAX_PD_OUTPUT_SIZE: maximum size of the process output data (Sync Manager 2) for cyclic exchange */
#define	MAX_PD_OUTPUT_SIZE				0x0040
/* MAX_FILE_SIZE: maximum file size */
#define	MAX_FILE_SIZE					0x0100

/*/////////////////////////////////////////////////////////////
//
//	Software Switches to reduce code size and to adapt the code
*/

/* AL_EVENT_ENABLED: if an interrupt routine shall be called when one of the Events in the AL Event Register (0x220) changes, 
   this switch has to be defined to 1 (synchronous modes are supported), 
   if the AL Event register shall only be polled, this switch has to be defined to 0 (only free run mode is supported)
	DC_SUPPORTED: if distributed clocks should be supported by the slave, this switch shall be set, if this switch is set,
	AL_EVENT_ENABLED shall be set too
	COE_SUPPORTED: if the CoE services are supported, this switch shall be set
	COMPLETE_ACCESS_SUPPORTED: if the complete SDO access (accessing all entries of an object with one SDO service, this
	switch shall be set, COE_SUPPORTED shall be set too
	SEGMENTED_SDO_SUPPORTED: if the segmented SDO services should be supported, this switch shall be set, COE_SUPPORTED shall be set too 
	BACKUP_PARAMETER_SUPPORTED: with this switch set, the functions in the application example to load and
	store backup parameter will be compiled, COE_SUPPORTED shall be set too
	EOE_SUPPORTED: if the EoE services should be supported, this switch shall be set 
	FOE_SUPPORTED: if the FoE services should be supported, this switch shall be set 
	VOE_SUPPORTED: if the VoE services should be supported, this switch shall be set, only the calling of the 
	VoE functions in mailbox.c is added, the VoE service functions have to be added, the example code cannot be linked
	correctly because these functions are missing 
	SOE_SUPPORTED: if the SoE services should be supported, this switch shall be set, only the calling of the 
	SoE functions in mailbox.c is added, the VoE service functions have to be added, the example code cannot be linked
	correctly because these functions are missing 
	BOOTSTRAPMODE_SUPPORTED: if the firmware update over FoE services should be supported, this switch shall be set,
	if this switch is set, FOE_SUPPORTED shall be set too 
	ECAT_TIMER_INT: with this switch set, the watchdog time for the EtherCAT watchdog will be checked in a timer interrupt routine
	MAILBOX_QUEUE: with this switch set, the mailbox services with be stored in a queue, with this switch reset only one mailbox
	service can be processed in parallel
	BYTE_NOT_SUPPORTED: if the microcontroller cannot make 8 bit-accesses to external memory, this switch shall be set
	MOTOROLA_16BIT: if the microcontroller always make 16 bit-accesses to external memory, operates in motorola format
	and the switching of the high and low byte is done in hardware, this switch shall be set, if this switch is set,
	BYTE_NOT_SUPPORTED shall be set and MOTOROLA_FORMAT shall be reset 
	MOTOROLA_FORMAT: if the microcontroller works with motorola format, this switch shall be set, in that case all WORD-
	and DWORD-accesses will make a BYTE- or WORD-swapping, the makros SWAPWORD and SWAPDWORD in ecatslv.h might be adapted, 
	if this switch is set MOTOROLA_16BIT shall be reset
   */
#define BOOTSTRAPMODE_SUPPORTED 0
#define SOE_SUPPORTED 0
#define EOE_SUPPORTED 0
#ifndef _APP_XC
    #define FOE_SUPPORTED 0
#else
	#warning "temp disable FOE"
    #define FOE_SUPPORTED 0
#endif
#define COE_SUPPORTED 1
#define COMPLETE_ACCESS_SUPPORTED 0
#define SEGMENTED_SDO_SUPPORTED 1
#define BYTE_NOT_SUPPORTED 0
#define DC_SUPPORTED 1
#define _PIC18 0
#define VOE_SUPPORTED 0
#define AL_EVENT_ENABLED 1
#define MOTOROLA_16BIT 0
#define TEST_ON_EVA_BOARD 1
#define ECAT_TIMER_INT 0
#define MOTOROLA_FORMAT 0
#define BACKUP_PARAMETER_SUPPORTED 1
#define ECAT_SDO_SUPPORTED 1
#define NIOS_CPU 0
#define LEGACY_MODE 1

#endif // _ECATDEF_H_
