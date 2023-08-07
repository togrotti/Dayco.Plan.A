/*-----------------------------------------------------------------------------------------
------
------	Description
------	
------	sdoserv.h
------
------	SDO-Server definitions
------			 																																					------
-----------------------------------------------------------------------------------------*/

#ifndef _SDOSERV_H_
#define _SDOSERV_H_

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/

#include "ecatcoe.h"

/*-----------------------------------------------------------------------------------------
------	
------	Defines and Types
------	
-----------------------------------------------------------------------------------------*/

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// Error Codes
*/

#define	ERROR_SDOINVALIDCOMMAND		0x1101
#define	ERROR_SDOINVALIDHEADER		0x1102
#define	ERROR_SDONOTSUPPORTED		0x1103

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// SDO services
*/

#define	SDOHEADER_SIZEINDICATOR					((UINT8) 0x01)
#define	SDOHEADER_TRANSFERTYPE					((UINT8) 0x02)
#define	SDOHEADER_DATASETSIZE					((UINT8) 0x0C)
#define	SDOHEADER_COMPLETEACCESS				((UINT8) 0x10)
#define	SDOHEADER_COMMAND							((UINT8) 0xE0)
#define	SDOHEADERSHIFT_SIZEINDICATOR			((UINT8) 0)
#define	SDOHEADERSHIFT_TRANSFERTYPE			((UINT8) 1)
#define	SDOHEADERSHIFT_DATASETSIZE				((UINT8) 2)
#define	SDOHEADERSHIFT_INDEXACCESS				((UINT8) 4)
#define	SDOHEADERSHIFT_COMMAND					((UINT8) 5)
#define	SDOSERVICE_SIZEINDICATOR				((UINT8) 0x01)
#define	SDOSERVICE_TRANSFERTYPE					((UINT8) 0x02)
#define	SDOSERVICE_DATASETSIZESHIFT			((UINT8) 2)
#define	SDOSERVICE_COMMANDSHIFT					((UINT8) 5)
#define	SDOSERVICE_INITIATEDOWNLOADREQ		((UINT8) (0x01 << SDOHEADERSHIFT_COMMAND))
#define	SDOSERVICE_INITIATEDOWNLOADRES		((UINT8) (0x03 << SDOHEADERSHIFT_COMMAND))
#define	SDOSERVICE_DOWNLOADSEGMENTREQ			((UINT8) (0x00 << SDOHEADERSHIFT_COMMAND))
#define	SDOSERVICE_DOWNLOADSEGMENTRES			((UINT8) (0x01 << SDOHEADERSHIFT_COMMAND))
#define	SDOSERVICE_INITIATEUPLOADREQ			((UINT8) (0x02 << SDOHEADERSHIFT_COMMAND))
#define	SDOSERVICE_INITIATEUPLOADRES			((UINT8) (0x02 << SDOHEADERSHIFT_COMMAND))
#define	SDOSERVICE_UPLOADSEGMENTREQ			((UINT8) (0x03 << SDOHEADERSHIFT_COMMAND))
#define	SDOSERVICE_UPLOADSEGMENTRES			((UINT8) (0x00 << SDOHEADERSHIFT_COMMAND))
#define	SDOSERVICE_ABORTTRANSFER				((UINT8) (((UINT8) 0x04) << SDOHEADERSHIFT_COMMAND))

///////////////////////////////////////////////////////////////////////////////////////////
/*/////////////////////////////////////////////////////////////////////////////////////////
//
// SDO structures
*/

///////////////////////////////////////////////////////////////////////////////////////////
// BASIC structures:

// initial service header:
typedef struct STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8 Sdo[2];
			#define			SDOHEADER_COMMANDOFFSET				0
			#define			SDOHEADER_INDEXLOOFFSET				0
			#define			SDOHEADER_INDEXHIOFFSET				1
			#define			SDOHEADER_SUBINDEXOFFSET			1
			#define			SDOHEADER_COMMANDMASK				0xFF
			#define			SDOHEADER_INDEXLOSHIFT				8
			#define			SDOHEADER_INDEXHIMASK				0xFF
			#define			SDOHEADER_SUBINDEXSHIFT				8
#else
	UINT8 Sdo[4];
#if MOTOROLA_16BIT
			#define			SDOHEADER_COMMANDOFFSET				1
			#define			SDOHEADER_INDEXLOOFFSET				0
			#define			SDOHEADER_INDEXHIOFFSET				3
			#define			SDOHEADER_SUBINDEXOFFSET			2
#else
			#define			SDOHEADER_COMMANDOFFSET				0
			#define			SDOHEADER_INDEXLOOFFSET				1
			#define			SDOHEADER_INDEXHIOFFSET				2
			#define			SDOHEADER_SUBINDEXOFFSET			3
#endif
			#define			SDOHEADER_COMMANDMASK				0xFF
			#define			SDOHEADER_INDEXLOSHIFT				0
			#define			SDOHEADER_INDEXHIMASK				0xFF
			#define			SDOHEADER_SUBINDEXSHIFT				0
#endif
} TINITSDOHEADER;

// complete initial service header: 
typedef struct STRUCT_PACKED
{
  UMBXHEADER        	MbxHeader;
  TCOEHEADER        	CoeHeader;
  TINITSDOHEADER    	SdoHeader;
} TINITSDOMBX;

// defines:
#define MAX_EXPEDITED_DATA			4
#define MIN_SEGMENTED_DATA			((UINT16) 7)
#define EXPEDITED_FRAME_SIZE		( SIZEOF( TCOEHEADER ) + SIZEOF( TINITSDOHEADER ) + MAX_EXPEDITED_DATA )
#define DOWNLOAD_NORM_REQ_SIZE	( SIZEOF( TCOEHEADER ) + SIZEOF( TINITSDOHEADER ) + 4	)
/* HBu 06.02.06: names of defines changed */
/* HBu 21.03.06: the SDO_Download-Response has to have always 8 bytes */
#define DOWNLOAD_NORM_RES_SIZE	( SIZEOF( TCOEHEADER ) + SIZEOF( TINITSDOHEADER ) + 4 )
#define UPLOAD_NORM_RES_SIZE		( SIZEOF( TCOEHEADER ) + SIZEOF( TINITSDOHEADER ) + 4 )
#define SEGMENT_NORM_HEADER_SIZE	( SIZEOF( TCOEHEADER ) + 1 )
#define SEGMENT_NORM_RES_SIZE		( SEGMENT_NORM_HEADER_SIZE + MIN_SEGMENTED_DATA )
#define SEGMENT_MBX_SIZE			( SIZEOF( UMBXHEADER ) + SEGMENT_NORM_HEADER_SIZE )
 
// segmented service header with data:
typedef struct STRUCT_PACKED
{
	UINT8		SegHeader;
				#define	SEGHEADER_NOMOREFOLLOWS			((UINT8) 0x01)
				#define	SEGHEADER_SEGDATASIZE			((UINT8) 0x0E)
				#define	SEGHEADER_TOGGLE					((UINT8) 0x10)
				#define	SEGHEADER_COMMAND					((UINT8) 0xE0)
				#define	SEGHEADERSHIFT_SEGDATASIZE		((UINT8) 1)
				#define	SEGHEADERSHIFT_TOGGLE			((UINT8) 4)
				#define	SEGHEADERSHIFT_COMMAND			((UINT8) 5)
  	UINT8    Data[MAX_MBX_DATA_SIZE-SEGMENT_NORM_HEADER_SIZE];
} TSDOSEGHEADERDATA;

///////////////////////////////////////////////////////////////////////////////////////////
// DOWNLOAD service structures:

// expedited download request:
typedef struct STRUCT_PACKED
{
  UMBXHEADER			MbxHeader;
  TCOEHEADER        	CoeHeader;
  TINITSDOHEADER		SdoHeader;
  UINT16					Data[MAX_EXPEDITED_DATA >> 1];
} TINITSDODOWNLOADEXPREQMBX;

// normal download resquest: 
typedef struct STRUCT_PACKED
{
  UMBXHEADER        	MbxHeader;
  TCOEHEADER        	CoeHeader;
  TINITSDOHEADER    	SdoHeader;
  UINT16            	CompleteSize[2];
  UINT16            	Data[(MAX_MBX_DATA_SIZE-DOWNLOAD_NORM_REQ_SIZE) >> 1];
} TINITSDODOWNLOADNORMREQMBX;

// expedited and normal download response:
typedef struct STRUCT_PACKED
{
  UMBXHEADER        	MbxHeader;
  TCOEHEADER        	CoeHeader;
  TINITSDOHEADER    	SdoHeader;
} TINITSDODOWNLOADRESMBX;

// segmented download request:
typedef struct STRUCT_PACKED
{
  UMBXHEADER        	MbxHeader;
  TCOEHEADER        	CoeHeader;
  TSDOSEGHEADERDATA 	SdoHeader;	// data is included in header !
} TDOWNLOADSDOSEGREQMBX;

// segmented download response:
typedef struct STRUCT_PACKED
{
  UMBXHEADER        	MbxHeader;
  TCOEHEADER        	CoeHeader;
  UINT8			     	SegHeader;
} TDOWNLOADSDOSEGRESMBX;


///////////////////////////////////////////////////////////////////////////////////////////
// UPLOAD service structures:

// expedited and normal upload request:
typedef struct STRUCT_PACKED
{
  UMBXHEADER        	MbxHeader;
  TCOEHEADER        	CoeHeader;
  TINITSDOHEADER    	SdoHeader;
} TINITSDOUPLOADREQMBX;

// expedited upload response:
typedef struct STRUCT_PACKED
{
  UMBXHEADER        	MbxHeader;	// 6 bytes
  TCOEHEADER        	CoeHeader;	// 2 bytes
  TINITSDOHEADER    	SdoHeader;	// 4 bytes
  UINT16            	Data[MAX_EXPEDITED_DATA >> 1];
} TINITSDOUPLOADEXPRESMBX;
	
// normal upload response:
typedef struct STRUCT_PACKED
{
  UMBXHEADER        	MbxHeader;
  TCOEHEADER        	CoeHeader;
  TINITSDOHEADER    	SdoHeader;
  UINT16            	CompleteSize[2];
  UINT16            	Data[(MAX_MBX_DATA_SIZE-UPLOAD_NORM_RES_SIZE) >> 1];
} TINITSDOUPLOADNORMRESMBX;

// segmented upload request:
typedef struct STRUCT_PACKED
{
  UMBXHEADER        	MbxHeader;
  TCOEHEADER        	CoeHeader;
  UINT8			     	SegHeader;
} TUPLOADSDOSEGREQMBX;

// segmented upload response:
typedef struct STRUCT_PACKED
{
  UMBXHEADER        	MbxHeader;
  TCOEHEADER        	CoeHeader;
  TSDOSEGHEADERDATA 	SdoHeader;	// data is included in header ! 
} TUPLOADSDOSEGRESMBX;

///////////////////////////////////////////////////////////////////////////////////////////
// ABORT service structure and defines:

// abort request:
typedef struct STRUCT_PACKED
{
  UMBXHEADER        	MbxHeader;
  TCOEHEADER        	CoeHeader;
  TINITSDOHEADER    	SdoHeader;
  UINT32             AbortCode;
} TABORTSDOTRANSFERREQMBX;

#define ABORT_NORM_RES_SIZE	( SIZEOF( TCOEHEADER ) + SIZEOF( TINITSDOHEADER ) + 4 )

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// Abort codes
*/

#define ABORTIDX_TOGGLE_BIT_NOT_CHANGED												0x01
#define ABORTIDX_SDO_PROTOCOL_TIMEOUT													0x02
#define ABORTIDX_COMMAND_SPECIFIER_UNKNOWN											0x03
#define ABORTIDX_OUT_OF_MEMORY															0x04
#define ABORTIDX_UNSUPPORTED_ACCESS														0x05
#define ABORTIDX_WRITE_ONLY_ENTRY														0x06
#define ABORTIDX_READ_ONLY_ENTRY															0x07
#define ABORTIDX_OBJECT_NOT_EXISTING													0x08
#define ABORTIDX_OBJECT_CANT_BE_PDOMAPPED 											0x09
#define ABORTIDX_MAPPED_OBJECTS_EXCEED_PDO											0x0A
#define ABORTIDX_PARAM_IS_INCOMPATIBLE													0x0B
#define ABORTIDX_INTERNAL_DEVICE_INCOMPATIBILITY									0x0C
#define ABORTIDX_HARDWARE_ERROR															0x0D
#define ABORTIDX_PARAM_LENGTH_ERROR														0x0E
#define ABORTIDX_PARAM_LENGTH_TOO_LONG													0x0F
#define ABORTIDX_PARAM_LENGTH_TOO_SHORT												0x10
#define ABORTIDX_SUBINDEX_NOT_EXISTING													0x11
#define ABORTIDX_VALUE_EXCEEDED															0x12
#define ABORTIDX_VALUE_TOO_GREAT															0x13
#define ABORTIDX_VALUE_TOO_SMALL															0x14
#define ABORTIDX_MAX_VALUE_IS_LESS_THAN_MIN_VALUE									0x15
#define ABORTIDX_GENERAL_ERROR															0x16
#define ABORTIDX_DATA_CANNOT_BE_READ_OR_STORED										0x17
/* ECATCHANGE_START(V4.00) */
#define ABORTIDX_DATA_CANNOT_BE_ACCESSED_BECAUSE_OF_LOCAL_CONTROL				0x18
#define ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED					0x19
/* ECATCHANGE_END(V4.00) */
#define ABORTIDX_NO_OBJECT_DICTIONARY_IS_PRESENT									0x1A

#define ABORT_NOERROR																	0x00000000
#define ABORT_TOGGLE_BIT_NOT_CHANGED												0x05030000
#define ABORT_SDO_PROTOCOL_TIMEOUT													0x05040000
#define ABORT_COMMAND_SPECIFIER_UNKNOWN											0x05040001
#define ABORT_OUT_OF_MEMORY															0x05040005
#define ABORT_UNSUPPORTED_ACCESS														0x06010000
#define ABORT_WRITE_ONLY_ENTRY														0x06010001
#define ABORT_READ_ONLY_ENTRY															0x06010002
#define ABORT_OBJECT_NOT_EXISTING													0x06020000
#define ABORT_OBJECT_CANT_BE_PDOMAPPED 											0x06040041
#define ABORT_MAPPED_OBJECTS_EXCEED_PDO											0x06040042
#define ABORT_PARAM_IS_INCOMPATIBLE													0x06040043
#define ABORT_INTERNAL_DEVICE_INCOMPATIBILITY									0x06040047
#define ABORT_HARDWARE_ERROR															0x06060000
#define ABORT_PARAM_LENGTH_ERROR														0x06070010
#define ABORT_PARAM_LENGTH_TOO_LONG													0x06070012
#define ABORT_PARAM_LENGTH_TOO_SHORT												0x06070013
#define ABORT_SUBINDEX_NOT_EXISTING													0x06090011
#define ABORT_VALUE_EXCEEDED															0x06090030
#define ABORT_VALUE_TOO_GREAT															0x06090031
#define ABORT_VALUE_TOO_SMALL															0x06090032
#define ABORT_MAX_VALUE_IS_LESS_THAN_MIN_VALUE									0x06090036
#define ABORT_GENERAL_ERROR															0x08000000
#define ABORT_DATA_CANNOT_BE_READ_OR_STORED										0x08000020
#define ABORT_DATA_CANNOT_BE_READ_OR_STORED_BECAUSE_OF_LOCAL_CONTROL		0x08000021
#define ABORT_DATA_CANNOT_BE_READ_OR_STORED_IN_THIS_STATE					0x08000022
#define ABORT_NO_OBJECT_DICTIONARY_IS_PRESENT									0x08000023

/*/////////////////////////////////////////////////////////////////////////////////////////
//
// SDO Information services
*/

#include "sdoinfoserv.h"

#define	SDOINFOSERVICE_OBJDICTIONARYLIST_Q	0x01
#define	SDOINFOSERVICE_OBJDICTIONARYLIST_S	0x02
#define	SDOINFOSERVICE_OBJDESCRIPTION_Q	 	0x03
#define	SDOINFOSERVICE_OBJDESCRIPTION_S		0x04 
#define	SDOINFOSERVICE_ENTRYDESCRIPTION_Q 	0x05
#define	SDOINFOSERVICE_ENTRYDESCRIPTION_S 	0x06
#define	SDOINFOSERVICE_ERROR_Q				 	0x07
#define	SDOINFOSERVICE_INCOMPLETE 				0x80

// SDO Information / Error:
typedef struct STRUCT_PACKED
{
	UINT32				ErrorCode;
//	char					ErrorText[];
} TSDOINFOERROR;


// SDO Information / Header
typedef struct STRUCT_PACKED
{
	union
	{
#if BYTE_NOT_SUPPORTED
		UINT8		b[1];
					#define	INFOHEAD_OFFS_OPCODE					0
#else
		UINT8		b[2];
#if MOTOROLA_16BIT
					#define	INFOHEAD_OFFS_OPCODE					1
#else
					#define	INFOHEAD_OFFS_OPCODE					0
#endif
#endif
					#define	INFOHEADER_OPCODEMASK				0x7F		
					#define	INFOHEADER_INCOMPLETEMASK			0x80
		UINT16	w;
	} InfoHead;
	UINT16				FragmentsLeft;

	union
	{
		TSDOINFOLIST	List;
		TSDOINFOOBJ		Obj;
		TSDOINFOENTRY	Entry;
		TSDOINFOERROR	Error;
		UINT16 			Data[1];
	} Data;
} TSDOINFOHEADER;


// SDO Information / complete structure									 
typedef struct STRUCT_PACKED
{
  UMBXHEADER        MbxHeader;
  TCOEHEADER        CoeHeader;
  TSDOINFOHEADER    SdoHeader;
} TSDOINFORMATION;							
			 
#define SIZEOF_SDOINFOHEAD				4
#define SIZEOF_SDOINFO					( SIZEOF( TCOEHEADER ) + SIZEOF_SDOINFOHEAD )
#define SIZEOF_SDOINFOSTRUCT			( SIZEOF( UMBXHEADER ) + SIZEOF( TCOEHEADER ) + SIZEOF_SDOINFOHEAD )
#define SIZEOF_SDOINFOLISTHEAD		2
#define SIZEOF_SDOINFOLISTSTRUCT		( SIZEOF( TCOEHEADER ) + SIZEOF_SDOINFOHEAD + SIZEOF_SDOINFOLISTHEAD )
#define SIZEOF_SDOINFOOBJSTRUCT		( SIZEOF( TCOEHEADER ) + SIZEOF_SDOINFOHEAD + SIZEOF( TSDOINFOOBJ ) )		
#define SIZEOF_SDOINFOENTRYHEAD		4
#define SIZEOF_SDOINFOENTRYSTRUCT	( SIZEOF( TCOEHEADER ) + SIZEOF_SDOINFOHEAD + SIZEOF_SDOINFOENTRYHEAD )
#define SIZEOF_SDOINFOERRORSTRUCT	( SIZEOF( TCOEHEADER ) + SIZEOF_SDOINFOHEAD + SIZEOF( TSDOINFOERROR ) )

/*-----------------------------------------------------------------------------------------
------	
------	global variables
------	
-----------------------------------------------------------------------------------------*/

#ifndef _SDOSERV_
#define _SDOSERV_   0
#endif

#if _SDOSERV_
	#define PROTO
#else 
	#define PROTO extern
#endif

PROTO UINT8	aSdoInfoHeader[SIZEOF_SDOINFOSTRUCT+SIZEOF_SDOINFOLISTHEAD];

/*-----------------------------------------------------------------------------------------
------  
------	global functions
------	
-----------------------------------------------------------------------------------------*/

PROTO	UINT8 SDOS_SdoInfoInd(TSDOINFORMATION MBXMEM *pSdoInfoInd);
PROTO	UINT8 SDOS_SdoInd(TINITSDOMBX MBXMEM *pSdoMbx);

PROTO	void 	OBJ_CopyNumberToString(UCHAR MBXMEM *pStr, UINT8 Number);

#undef PROTO

#endif //_SDOSRV_H_
