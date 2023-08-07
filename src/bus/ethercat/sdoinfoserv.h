/*-----------------------------------------------------------------------------------------
------
------	Description
------	
------	sdoinfoserv.h
------
------	SDO-Server definitions
------			 																																					------
-----------------------------------------------------------------------------------------*/

#ifndef _SDOINFOSERV_H_
#define _SDOINFOSERV_H_

#include "ecat_def.h"

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// SDO Information service structures:
*/

// SDO Information / Object Dictionary Lists:
typedef struct STRUCT_PACKED
{
	UINT16				ListType;
		#define	INFO_LIST_TYPE_LENGTH	0
		#define	INFO_LIST_TYPE_ALL		1
		#define	INFO_LIST_TYPE_RXPDO		2
		#define	INFO_LIST_TYPE_TXPDO		3
		#define	INFO_LIST_TYPE_BACKUP	4
		#define	INFO_LIST_TYPE_SET		5
		#define	INFO_LIST_TYPE_MAX		5
} TSDOINFOLIST;


typedef struct STRUCT_PACKED
{
		UINT16			DataType;			// refer to data type index
		UINT16			ObjFlags;
#if MOTOROLA_FORMAT
							#define	OBJFLAGS_MAXSUBINDEXMASK	0xFF00			
							#define	OBJFLAGS_MAXSUBINDEXSHIFT	8			
							#define	OBJFLAGS_OBJCODEMASK			0x000F			
							#define	OBJFLAGS_OBJCODESHIFT		0			
#else
							#define	OBJFLAGS_MAXSUBINDEXMASK	0x00FF			
							#define	OBJFLAGS_MAXSUBINDEXSHIFT	0			
							#define	OBJFLAGS_OBJCODEMASK			0x0F00			
							#define	OBJFLAGS_OBJCODESHIFT		8			
#endif
								#define	OBJCODE_VAR					0x07
								#define	OBJCODE_ARR					0x08
								#define	OBJCODE_REC					0x09
//		char				Name[];				// rest of mailbox data
} TSDOINFOOBJDESC;

typedef struct TETHERCAT_SDO_INFO_OBJ STRUCT_PACKED
{
	UINT16					Index;
	TSDOINFOOBJDESC	Res;
} TSDOINFOOBJ, *PTSDOINFOOBJ;

// SDO Information / Entry Description:
typedef struct STRUCT_PACKED
{
	UINT16				DataType;				// refer to data type index
	UINT16				BitLength;
	UINT16				ObjAccess;				// Bit 0: Read Access in Pre-Op
										// Bit 1: Read Access in Safe-Op
										// Bit 2: Read Access in Op
										// Bit 3: Write Access in Pre-Op
										// Bit 4: Write Access in Safe-Op
										// Bit 5: Write Access in Op
										// Bit 6: mapbar in RxPDO
										// Bit 7: mapbar in TxPDO
										// Bit 8: entry will be included in backup
										// Bit 9: entry will be included in settings
			#define	ACCESS_READWRITE			0x003F
			#define	ACCESS_READ					0x0007
			#define	ACCESS_READ_PREOP			0x0001
			#define	ACCESS_READ_SAFEOP		0x0002
			#define	ACCESS_READ_OP				0x0004
			#define	ACCESS_WRITE				0x0038
			#define	ACCESS_WRITE_PREOP		0x0008
			#define	ACCESS_WRITE_SAFEOP		0x0010
			#define	ACCESS_WRITE_OP			0x0020
			#define	OBJACCESS_NOPDOMAPPING	0x0000
			#define	OBJACCESS_RXPDOMAPPING	0x0040
			#define	OBJACCESS_TXPDOMAPPING	0x0080
			#define	OBJACCESS_BACKUP			0x0100
			#define	OBJACCESS_SETTINGS		0x0200
//		UINT16			UnitType;				// optional if bit3 of valueInfo
//		UINT8				DefaultValue[];		// optional if bit4 of valueInfo
//		UINT8				MinValue[];				// optional if bit5 of valueInfo
//		UINT8				MaxValue[];				// optional if bit6 of valueInfo
//		char				Desc[];					// rest of mailbox data
} TSDOINFOENTRYDESC;

// ObjCategory values:
#define	OBJ_OPTIONAL				0x00
#define	OBJ_MANDATORY  			0x01

#define SIZEOF_MAXVALUEINFO		26			// 2 Bytes UnitType + ( 3 * DEFTYPE_REAL64 ) 

typedef struct STRUCT_PACKED
{
	UINT16				Index;
	union
	{
#if BYTE_NOT_SUPPORTED
		UINT8				b[1];
							#define	ENTRY_OFFS_SUBINDEX		0
							#define	ENTRY_OFFS_VALUEINFO		0
							#define	ENTRY_MASK_SUBINDEX		0x00FF
							#define	ENTRY_MASK_VALUEINFO		0xFF00
#else
		UINT8				b[2];
#if MOTOROLA_16BIT
							#define	ENTRY_OFFS_SUBINDEX		1
							#define	ENTRY_OFFS_VALUEINFO		0
#else
							#define	ENTRY_OFFS_SUBINDEX		0
							#define	ENTRY_OFFS_VALUEINFO		1
#endif
#endif
		UINT16			w;
	} Info;
	TSDOINFOENTRYDESC	Res;
} TSDOINFOENTRY;

#endif
