/*------------------------------------------------------------------------------
**
**	Copyright:	AXEL s.r.l. 2011
**
**	PLCIECTARG.H:	Interface between SoftScope and its runtime 
**
**-----------------------------------------------------------------------------
**
**	IMPORTANT:
**	THIS MODULE SHOULDN'T BE MODIFIED BY THE CUSTOMER EXCEPT FOR
**	PACKING PRAGMAS REQUIRED FOR CORRECTLY INTERFACE WITH LOGICLAB
**
**-----------------------------------------------------------------------------*/

#ifndef _SSCTARG_H
#define _SSCTARG_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _C_START_PACK_PRAGMA
#pragma _C_START_PACK_PRAGMA
#endif

/*	SoftScope features	*/
#define SSC_FT_BUFFEREDMODE			0x00000001
#define SSC_FT_REALTIMEMODE			0x00000002
#define SSC_FT_REALTIMEPUSHMODE		0x00000004
#define SSC_FT_DBMEMORYACCESS		0x00000008


/*	SoftScope commands IDs	*/
#define SSC_IDLE_COMMAND								0

#define SSC_SET_TRACK0_DATATYPE							1
#define SSC_SET_TRACK1_DATATYPE							2
#define SSC_SET_TRACK2_DATATYPE							3
#define SSC_SET_TRACK3_DATATYPE							4
#define SSC_SET_TRACK4_DATATYPE							5
#define SSC_SET_TRACK5_DATATYPE							6
#define SSC_SET_TRACK6_DATATYPE							7
#define SSC_SET_TRACK7_DATATYPE							8

#define SSC_SET_TRACK0_ADDR								11
#define SSC_SET_TRACK1_ADDR								12
#define SSC_SET_TRACK2_ADDR								13
#define SSC_SET_TRACK3_ADDR								14
#define SSC_SET_TRACK4_ADDR								15
#define SSC_SET_TRACK5_ADDR								16
#define SSC_SET_TRACK6_ADDR								17
#define SSC_SET_TRACK7_ADDR								18

#define SSC_SET_TRIGGER_THR_HI							19
#define SSC_SET_TRIG_DATATYPE							20
#define SSC_SET_TRIG_ADDR								21
#define SSC_SET_TIME_PRESCALER							22
#define SSC_SET_SAMPLE_REQUIRED							23
#define SSC_SET_TRIGGER_EDGE							24
#define SSC_SET_TRIGGER_THR								25
#define SSC_SET_TRIGGER_PRE								26
#define SSC_GET_SAMPLEBUFFSIZE							27
#define SSC_GET_NUMTRACKS								28
#define SSC_GET_SAMPLE_TIMEBASE							29
#define SSC_GET_SAMPLE_BUFFBASE							30

#define SSC_GET_OBJECT0_SCALE							31
#define SSC_GET_OBJECT1_SCALE							32
#define SSC_GET_OBJECT2_SCALE							33
#define SSC_GET_OBJECT3_SCALE							34
#define SSC_GET_OBJECT4_SCALE							35
#define SSC_GET_OBJECT5_SCALE							36
#define SSC_GET_OBJECT6_SCALE							37
#define SSC_GET_OBJECT7_SCALE							38

#define SSC_GET_OBJECT0_OFFSET							39
#define SSC_GET_OBJECT1_OFFSET							40
#define SSC_GET_OBJECT2_OFFSET							41
#define SSC_GET_OBJECT3_OFFSET							42
#define SSC_GET_OBJECT4_OFFSET							43
#define SSC_GET_OBJECT5_OFFSET							44
#define SSC_GET_OBJECT6_OFFSET							45
#define SSC_GET_OBJECT7_OFFSET							46

#define SSC_GET_OBJECT0_DATATYPE						47
#define SSC_GET_OBJECT1_DATATYPE						48
#define SSC_GET_OBJECT2_DATATYPE						49
#define SSC_GET_OBJECT3_DATATYPE						50
#define SSC_GET_OBJECT4_DATATYPE						51
#define SSC_GET_OBJECT5_DATATYPE						52
#define SSC_GET_OBJECT6_DATATYPE						53
#define SSC_GET_OBJECT7_DATATYPE						54
#define SSC_SET_TRIGGER_CMPTYPE							55

#define SSC_FORCE_TRIGGER_CONDITION 					56

#define SSC_SET_TRACK8_DATATYPE							57
#define SSC_SET_TRACK9_DATATYPE							58
#define SSC_SET_TRACK10_DATATYPE						59
#define SSC_SET_TRACK11_DATATYPE						60
#define SSC_SET_TRACK12_DATATYPE						61
#define SSC_SET_TRACK13_DATATYPE						62
#define SSC_SET_TRACK14_DATATYPE						63
#define SSC_SET_TRACK15_DATATYPE						64
#define SSC_SET_TRACK16_DATATYPE						65
#define SSC_SET_TRACK17_DATATYPE						66
#define SSC_SET_TRACK18_DATATYPE						67
#define SSC_SET_TRACK19_DATATYPE						68

#define SSC_SET_TRACK8_ADDR								69
#define SSC_SET_TRACK9_ADDR								70
#define SSC_SET_TRACK10_ADDR							71
#define SSC_SET_TRACK11_ADDR							72
#define SSC_SET_TRACK12_ADDR							73
#define SSC_SET_TRACK13_ADDR							74
#define SSC_SET_TRACK14_ADDR							75
#define SSC_SET_TRACK15_ADDR							76
#define SSC_SET_TRACK16_ADDR							77
#define SSC_SET_TRACK17_ADDR							78
#define SSC_SET_TRACK18_ADDR							79
#define SSC_SET_TRACK19_ADDR							80

#define SSC_GET_OBJECT8_SCALE							81
#define SSC_GET_OBJECT9_SCALE							82
#define SSC_GET_OBJECT10_SCALE							83
#define SSC_GET_OBJECT11_SCALE							84
#define SSC_GET_OBJECT12_SCALE							85
#define SSC_GET_OBJECT13_SCALE							86
#define SSC_GET_OBJECT14_SCALE							87
#define SSC_GET_OBJECT15_SCALE							88
#define SSC_GET_OBJECT16_SCALE							89
#define SSC_GET_OBJECT17_SCALE							90
#define SSC_GET_OBJECT18_SCALE							91
#define SSC_GET_OBJECT19_SCALE							92

#define SSC_GET_OBJECT8_OFFSET							93
#define SSC_GET_OBJECT9_OFFSET							94
#define SSC_GET_OBJECT10_OFFSET							95
#define SSC_GET_OBJECT11_OFFSET							96
#define SSC_GET_OBJECT12_OFFSET							97
#define SSC_GET_OBJECT13_OFFSET							98
#define SSC_GET_OBJECT14_OFFSET							99
#define SSC_GET_OBJECT15_OFFSET							100
#define SSC_GET_OBJECT16_OFFSET							101
#define SSC_GET_OBJECT17_OFFSET							102
#define SSC_GET_OBJECT18_OFFSET							103
#define SSC_GET_OBJECT19_OFFSET							104

#define SSC_GET_OBJECT8_DATATYPE						105
#define SSC_GET_OBJECT9_DATATYPE						106
#define SSC_GET_OBJECT10_DATATYPE						107
#define SSC_GET_OBJECT11_DATATYPE						108
#define SSC_GET_OBJECT12_DATATYPE						109
#define SSC_GET_OBJECT13_DATATYPE						110
#define SSC_GET_OBJECT14_DATATYPE						111
#define SSC_GET_OBJECT15_DATATYPE						112
#define SSC_GET_OBJECT16_DATATYPE						113
#define SSC_GET_OBJECT17_DATATYPE						114
#define SSC_GET_OBJECT18_DATATYPE						115
#define SSC_GET_OBJECT19_DATATYPE						116

#define SSC_RESET_ALL_TRACKS							117

#define SSC_SET_ACQUISITION_MODE						118
#define SSC_GET_RTSAMPLE_BUFF0							119
#define SSC_GET_RTSAMPLE_BUFF1							120
#define SSC_GET_SYN_SAMPLE_COUNTER						121
/*	Log general commands	*/
#define SSC_SET_LOG_MODE_ENABLED						122	//	Enable/disable data logging mode
#define SSC_GET_LOG_MODE_ENABLED						123	//	Get data logging mode enabled/disabled
/*	Log save commands	*/
#define SSC_NOTIFY_XML_CONFIG_DATA_BUFFER_REQUIRED_SIZE	124	//	Notify to firmware the required data size to store xml data
#define SSC_GET_XML_CONFIG_DATA_BUFFER_ADDRESS			125	//	Get buffer address in which acquisition data xml will be set, 0 if not available or not big enough
#define SSC_NOTIFY_XML_CONFIG_DATA_DOWNLOAD_COMPLETED	126	//	Notify to firmware that acquisition config data download is completed
#define SSC_LOAD_ACQ_REQUEST							127	//	Load acquisition request
#define SSC_GET_XML_CONFIG_DATA_BUFFER_SIZE				128	//	Restore session (session to load must be specified in target dependent way)
/*	Auto restart */
#define SSC_PING										129	//	Ping to check if pc is connected
#define SSC_SET_AUTORESTART_MODE						130	//	Use to put avoid restart
#define SSC_GET_AUTORESTART_MODE						131	//	Use to put avoid restart
/*	Monitor */
#define SSC_MONITOR_ADDR								132	//	Monitor window set/get value. address info
#define SSC_MONITOR_DATATYPE							133	//	Monitor window set/get value. datatype info
#define SSC_MONITOR_SET_VALUE							134	//	Monitor window set value. Request to write (must be called after setting 132 and 133 cmds)
#define SSC_MONITOR_GET_VALUE							135 //	Monitor window get value. Request to read (must be called after setting 132 and 133 cmds)
/*	Features	*/
#define SSC_GET_FEATURES								136
#define SSC_MONITOR_GET_VALUE_HI						137 //	Monitor window get value (hi dword)
#define SSC_MONITOR_SET_VALUE_HI						138 //	Monitor window set value (hi dword9

/*	Command responses	*/
#define	SSC_ACK_CODE		0xAA
#define	SSC_NACK_CODE		0xAB

/*	Acquisition index and counter */
typedef struct _SSC_BUFFER_CTRL
{
	uint32_t	dwSamplesCounter;		/*	Number of samples collected in this buffer (with overflows)	*/
	uint32_t	dwSampleIndex;			/*	Index of the first sample in the buffer (0=no overflows, >0 overflowed buffer)	*/
}
SSC_BUFFER_CTRL;

/*	Type of address of signals	*/
typedef enum
{
	tyaPhysical		= 1,
	tyaObjDict		= 2,
	tyaDataBlockInp	= 3,
	tyaDataBlockOut	= 4,
	tyaDataBlockMem	= 5,
	tyaRelativeAddr	= 6,
}
TYADDR;

/*	Required trigger status (trigger command)	*/
typedef enum
{
	trgCmdIdle		= 1,
	trgCmdAuto		= 2,
	trgCmdNormal	= 3,
	trgCmdStop		= 4,
}
TRIGGERCOMMAND;

/*	Status of the trigger	*/
typedef enum
{
	trgIdle				= 1,
	trgPreTriggering	= 2,
	trgTriggered		= 3,
	trgStopped			= 4,
}
TRIGGERSTAT;

/*	Edge detection for the trigger	*/
typedef enum
{
	trgEdgeFalling		= 0,
	trgEdgeRising		= 1,
}
TRIGGERSLOPE;

/*	Type of the trigger comparison operation	*/
typedef enum
{
	trgCmpNumeric		= 0,
	trgCmpLogicalAND	= 1,
}
TRIGGERCMPTYPE;

/*	Operation mode	*/
typedef enum
{
	acqStandard			= 0,
	acqRealTime			= 1,
	acqRealTimePush		= 2,
}
ACQ_OPERATION_MODE;

/*	Type of relative address base (NB "code" has no sense here) */
typedef enum
{
	relData = 0,
	relRetainData = 1,
	relBitData = 2,
	relAuxData = 3,
}
RELATIVE_ADDR_BASE;

/*	Composition of signal characteristics	*/
#if defined( ALPLC_P_TIDSP )
typedef struct
{
	uint8_t		bDataType : 8;		/*	Data type according to TYVAR enumeration	*/
	uint8_t		bAddrType : 8;		/*	Address type according to TYADDR enumeration	*/
	uint16_t	wDataElement;		/*	Data element index for arrays	*/
}
SIGN_TYPES;
#else
typedef struct
{
	uint8_t		bDataType;		/*	Data type according to TYVAR enumeration	*/
	uint8_t		bAddrType;		/*	Address type according to TYADDR enumeration	*/
	uint16_t	wDataElement;	/*	Data element index for arrays	*/
}
SIGN_TYPES;
#endif

typedef union
{
	SIGN_TYPES	s;
	uint32_t	dw;
}
SIGN_DWTYPES;

/*	Data block address composition (main part) */
typedef struct
{
	uint16_t	wDataBlock;		/*	Data block index	*/
	uint16_t	wIndex;			/*	Element inside the data block	*/
}
SIGN_ADR_DB;

/*	Object dictionary object address composition */
typedef struct
{
	uint16_t	wObjectIndex;			/*	Main index	*/
	uint16_t	wObjectSubIndex;		/*	Sub index	*/
}
SIGN_ADR_DICOBJ;

/*	Relative address composition */
typedef struct
{
	uint32_t	bBase    : 4,			/*	Base area type	*/
				dwOffset : 28;			/*	Offset in bytes from base area	*/
}
SIGN_ADR_RELATIVEADDR;

/*	Type of data address */
typedef union
{
	addr_t					adrPhysAddress;      /*	Physical address			*/
	SIGN_ADR_DB				dbAddress;			/*	Data block address (db.idx)	*/
	SIGN_ADR_DICOBJ			dictObjectAddress;	/*	Object dictionary address	*/
	SIGN_ADR_RELATIVEADDR	relativeAddress;	/*	Relative address */
}
SIGN_ADDRESS;

/*  Type of data blocks */
#define DBTY_MEMO   0
#define DBTY_INP    1
#define DBTY_OUT    2

#ifdef _C_END_PACK_PRAGMA
#pragma _C_END_PACK_PRAGMA
#endif

#ifdef __cplusplus
}
#endif

#endif  /* _SSCTARG_H */

