/*-----------------------------------------------------------------------------------------
------
------	Description
------	
------	esc.h
------
------	EtherCAT Slave Controller
------			 																																					------
-----------------------------------------------------------------------------------------*/

#ifndef _ESC_H_

#define _ESC_H_

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/

#include "ecat_def.h"

/*-----------------------------------------------------------------------------------------
------	
------	Defines and Types
------	
-----------------------------------------------------------------------------------------*/

///////////////////////////////////////////////////////////////
//
//	Standard Data Types
//

#ifndef	UBYTEWORD
	typedef union STRUCT_PACKED
	{
		UINT8		b[2];
		UINT16	w;
	} UBYTEWORD;
#endif

#ifndef	UBYTEDWORD
	typedef union STRUCT_PACKED
	{
		UINT8		b[4];
		UINT16	w[2];
		UINT32	d;
	} UBYTEDWORD;
#endif

#if BYTE_NOT_SUPPORTED
	#define	SIZEOF(a)			(sizeof(a)<<1)
#else
	#define	SIZEOF(a)			sizeof(a)
#endif

#ifndef	BL_PAGE_SIZE
	#define	BL_PAGE_SIZE		512
#endif

/*---------------------------------------------------------------------------------
------																								------
------	Typdefinitionen																		------
------																								------
---------------------------------------------------------------------------------*/

/****************************************************************
**
** DLL-Information
*/

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[5];
									#define	ESC_ADDR_SYNCMANCHANNELS		2
									#define	ESC_SHIFT_SYNCMANCHANNELS		8
#else
	UINT8			 				Byte[10];
									#define	ESC_ADDR_SYNCMANCHANNELS		5
									#define	ESC_SHIFT_SYNCMANCHANNELS		0
#endif
	UINT16		 				Word[5];
									#define	ESC_ADDR_DPRAMSIZE				6
} UDLLINFORMATION;

/****************************************************************
**
** DLL-Control
*/

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
#else
	UINT8			 				Byte[2];
#endif
	UINT16		 				Word[1];
} UDLLCONTROL;

/****************************************************************
**
** DLL-Status
*/

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
#else
	UINT8			 				Byte[2];
#endif
	UINT16		 				Word[1];
} UDLLSTATUS;

/****************************************************************
**
** AL-Control
*/

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
									#define	ESC_OFFS_ALCONTROL		0
#else
	UINT8			 				Byte[2];
#if MOTOROLA_16BIT
									#define	ESC_OFFS_ALCONTROL		1
#else
									#define	ESC_OFFS_ALCONTROL		0
#endif
#endif
	UINT16		 				Word[1];
} UALCONTROL;
	
/****************************************************************
**
** AL-Status
*/

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
#else
	UINT8			 				Byte[2];
#endif
	UINT16 		 				Word[1];
} UALSTATUS;
	
/****************************************************************
**
** PDI Control
*/

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
#else
	UINT8			 				Byte[2];
#endif
	UINT16 		 				Word[1];
} UPDICONTROL;
	
/****************************************************************
**
** PDI-Configuration
*/

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
#else
	UINT8			 				Byte[2];
#endif
	UINT16 		 				Word[1];
} UPDICONFIGURATIONMCI16;
	
/****************************************************************
**
** AL-Event
*/

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[2];
#else
	UINT8			 				Byte[4];
#endif
	UINT16 		 				Word[2];
} UALEVENT;
	
/****************************************************************
**
** CRC-Fault-Counter
*/

typedef struct STRUCT_PACKED
{
  UINT16        				ChannelACrcFaultCounter;
  UINT16        				ChannelBCrcFaultCounter;
} TCHANNELCRCFAULTCOUNTER;

/****************************************************************
**
** Serial E²PROM access
*/

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[1];
									
#else
	UINT8			 				Byte[2];
#endif
	UINT16 		 				Word[1];
/* ECATCHANGE_START(V4.00) APPL 4 */
									#define	ESCEE_WRITEACCESS					0x0001
									#define	ESCEE_READBYTES					0x0040
									#define	ESCEE_ADDRESSSIZE					0x0080
									#define	ESCEE_READ							0x0100
									#define	ESCEE_WRITE							0x0200
									#define	ESCEE_RELOAD						0x0400
									#define	ESCEE_ERROR_ACKMISSING			0x2000
									#define	ESCEE_ERROR							0x7800
									#define	ESCEE_BUSY							0x8000
/* ECATCHANGE_END(V4.00) APPL 4 */
} USERIALEEPROMCONTROL;
	
typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8					 		Byte[1];	
#else
	UINT8					 		Byte[2];	
#endif
	UINT16						Word[1];
/* ECATCHANGE_START(V4.00) APPL 4 */
									#define ESCEE_ASSIGNTOPDI		0x0001
									#define ESCEE_LOCKEDBYPDI		0x0100
/* ECATCHANGE_END(V4.00) APPL 4 */
} USERIALEEPROMCONFIG;

/****************************************************************
**
** MII Management Interface
*/

typedef union STRUCT_PACKED
{
	UINT8			 				Byte[2];
	UINT16 		 				    Word[1];
} UMIIADDRESSES;

/****************************************************************
**
** FMMU
*/

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8			 				Byte[8];
#else
	UINT8			 				Byte[16];
#endif
	UINT16 		 				Word[8];
} UFMMU;
	
/****************************************************************
**
** Sync Manager
*/

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
  	UINT8         				Byte[2];
									#define	ESC_OFFS_SMCTRL			0
										#define	SM_PDINITMASK				0x0D
										#define	SM_INITMASK					0x0F
										#define	SM_WRITESETTINGS			0x04				
										#define	SM_READSETTINGS			0x00				
										#define	SM_PDIEVENT					0x20				
										#define	THREE_BUFFER				0x00
										#define	ONE_BUFFER					0x02
										#define	SYNCMAN_READ				0x00
										#define	SYNCMAN_WRITE				0x01
										#define	WATCHDOG_TRIGGER			0x40
/* ECATCHANGE_START(V4.00) */
									#define	ESC_OFFS_SMSTATUS			0
										#define	SM_BUFFERWRITTEN			0x0800
									#define	ESC_OFFS_ACTIVATE			1
										#define	SM_ECATENABLE				0x01
										#define	SM_ECATREPEAT				0x02
									#define	ESC_OFFS_PDICTRL			1
										#define	SM_PDIDISABLE				0x0100
										#define	SM_PDIREPEATACK			0x0200
/* ECATCHANGE_END(V4.00) */
#else
  	UINT8         				Byte[4];
#if MOTOROLA_16BIT
									#define	ESC_OFFS_SMCTRL			1
#else
									#define	ESC_OFFS_SMCTRL			0
#endif
										#define	SM_PDINITMASK				0x0D
										#define	SM_INITMASK					0x0F
										#define	SM_WRITESETTINGS			0x04				
										#define	SM_READSETTINGS			0x00				
										#define	SM_PDIEVENT					0x20				
										#define	THREE_BUFFER				0x00
										#define	ONE_BUFFER					0x02
										#define	SYNCMAN_READ				0x00
										#define	SYNCMAN_WRITE				0x01
										#define	WATCHDOG_TRIGGER			0x40
#if MOTOROLA_16BIT
									#define	ESC_OFFS_SMSTATUS			0
#else
									#define	ESC_OFFS_SMSTATUS			1
#endif
										#define	SM_WRITEEVENT				0x01
										#define	SM_READEVENT				0x02
										#define	SM_BUFFERWRITTEN			0x08
#if MOTOROLA_16BIT
									#define	ESC_OFFS_ECATENABLE		3
#else
									#define	ESC_OFFS_ECATENABLE		2
#endif
										#define	SM_ECATENABLE				0x01
										#define	SM_TOGGLEMASTER			0x02
#if MOTOROLA_16BIT
									#define	ESC_OFFS_PDIDISABLE		2
#else
									#define	ESC_OFFS_PDIDISABLE		3
#endif
										#define	SM_PDIDISABLE				0x01
										#define	SM_TOGGLESLAVE				0x02
#endif
  	UINT16        				Word[2];
} USMSETTINGS;  
  	
typedef struct STRUCT_PACKED
{
  UINT16        				PhysicalStartAddress;					
  UINT16        				Length;	
#if BYTE_NOT_SUPPORTED
									#define	ESC_OFFS_SMSETTINGS		2
#else
									#define	ESC_OFFS_SMSETTINGS		4
#endif
  USMSETTINGS   				Settings;
} TSYNCMAN;

typedef struct STRUCT_PACKED
{
  UINT32        				ReceiveTimeChannelA;				// 0x0900
  UINT32        				ReceiveTimeChannelB;				// 0x0904
  UINT16        				Reserved1[4];						
  UINT32        				SystemTime[2];						// 0x0910
  UINT16        				Reserved2[4];						
  UINT32        				SystemTimeOffset[2];				// 0x0920
  UINT32        				DelayTime;							// 0x0928
  UINT16        				Reserved3[2];						
} TDCTRANSMISSION;

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
  	UINT8         				Byte[1];
									#define	ESC_OFFS_DC_CONTROL	  		0x00
										#define	DC_CYCLIC_ACTIVE				0x0100
										#define	DC_SYNC0_ACTIVE				0x0200
										#define	DC_SYNC1_ACTIVE				0x0400
#else
  	UINT8         				Byte[2];
#if MOTOROLA_16BIT
									#define	ESC_OFFS_DC_CONTROL	  		0x00
#else
									#define	ESC_OFFS_DC_CONTROL	  		0x01
#endif
										#define	DC_CYCLIC_ACTIVE				0x01
										#define	DC_SYNC0_ACTIVE				0x02
										#define	DC_SYNC1_ACTIVE				0x04
#endif
  	UINT16        				Word[1];
} UDCSYNCCONTROL;  
  	
typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
  	UINT8             		Byte[1];
#else
  	UINT8             		Byte[2];
#endif
  	UINT16             		Word[1];
} UDCSYNCSTATUS;  
  	
typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
  	UINT8             		Byte[1];
#else
  	UINT8             		Byte[2];
#endif
  	UINT16             		Word[1];
} UDCLATCHCONTROL;  
  	
typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
  	UINT8             		Byte[1];
#else
  	UINT8             		Byte[2];
#endif
  	UINT16             		Word[1];
} UDCLATCHSTATUS;  
  	
typedef struct STRUCT_PACKED
{
	UINT32						PosEdgeTime_L;
	UINT32						PosEdgeTime_H;	
	UINT32						NegEdgeTime_L;
	UINT32						NegEdgeTime_H;	
} TLATCHTIMES32;

typedef struct STRUCT_PACKED
{
	UINT16						PosEdgeTime_LL;
	UINT16						PosEdgeTime_LH;
	UINT16						PosEdgeTime_HL;
	UINT16						PosEdgeTime_HH;	
	UINT16						NegEdgeTime_LL;
	UINT16						NegEdgeTime_LH;
	UINT16						NegEdgeTime_HL;
	UINT16						NegEdgeTime_HH;	
} TLATCHTIMES16;

typedef union STRUCT_PACKED
{
	TLATCHTIMES32				Dword;
	TLATCHTIMES16				Word;
} ULATCHTIMES;

typedef struct STRUCT_PACKED
{
  UDCSYNCCONTROL   			SyncControl;           			// 0x0980
#define	ESC_ADDR_SYNCCONTROL								0x0980
  UINT16             		ImpulseLength;						// 0x0982
  UINT16             		Reserved4[5];						
#define	ESC_ADDR_SYNC_STATUS								0x098E
  UDCSYNCSTATUS    			SyncStatus;           			// 0x098e
  UINT32            			SyncStartTime[2][2];				// 0x0990
  UINT32            			SyncCycleTime[2];					// 0x09a0
#define	ESC_ADDR_SYNC_CYCLETIME							0x09A0
  UDCLATCHCONTROL  			LatchControl;           		// 0x09a8
  UINT16             		Reserved9[2];						
  UDCLATCHSTATUS    			LatchStatus;           			// 0x09ae
  ULATCHTIMES					LatchTimes[2];						// 0x09b0
} TDCINTERRUPT;

typedef struct STRUCT_PACKED
{																			// offset...
	UDLLINFORMATION			DllInformation;					// 0x0000
	
	UINT16						cRes00[0x0003];					// 0x000A
	
	UINT16          			FixedStationAddress;				// 0x0010
	UINT16						cRes01[0x0077];					// 0x0012

	UDLLCONTROL					DllControl;							// 0x0100
	UINT16						cRes02[0x0007];					// 0x0102

	UDLLSTATUS					DllStatus;							// 0x0110
#define		ESC_ADDR_DLLSTATUS							0x0110
	UINT16						cRes03[0x0007];					// 0x0112

	UALCONTROL					AlControl;							// 0x0120
#define		ESC_ADDR_ALCONTROL							0x0120
	UINT16						cRes04[0x0007];					// 0x0122	

	UALSTATUS 					AlStatus;							// 0x0130
#define		ESC_ADDR_ALSTATUS								0x0130
	UINT16						cRes05[0x0001];					// 0x0132
#define		ESC_ADDR_ALSTATUSCODE						0x0134
	UINT16						AlStatusCode;						// 0x0134	
	UINT16						cRes05a[0x0005];					// 0x0136	

#define		ESC_ADDR_PDICTRL								0x0140
	UPDICONTROL					PdiControl;							// 0x0140
	UINT16						cRes06[0x0007];					// 0x0142	

	UPDICONFIGURATIONMCI16	PdiConfigurationMci16;			// 0x0150
	UINT16						cRes07[0x0059];					// 0x0152	

	UALEVENT 					AlEventMask; 						// 0x0204
#define		ESC_ADDR_ALEVENTMASK							0x0204
	UINT16						cRes07a[0x000C];					// 0x0208	

	UALEVENT 					AlEvent;								// 0x0220
#define		ESC_ADDR_ALEVENT								0x0220
	UINT16						cRes08[0x006E];					// 0x0224

#define     ESC_ADDR_ERRORS                                 0x0300
	TCHANNELCRCFAULTCOUNTER	CrcFaultCounter;					// 0x0300
	UINT16						cRes09[0x007E];					// 0x0304

	UINT16          			WatchdogDivider;					// 0x0400
#define		ESC_WATCHDOG_DIVIDER							0x400
	UINT16						cRes10[0x0007];					// 0x0402

	UINT16          			PdiWatchdog;						// 0x0410
	UINT16						cRes11[0x0007];					// 0x0412

	UINT16          			SyncManChannelWatchdog[16];	// 0x0420
#define		ESC_SM_WATCHDOG								0x420
	UINT16          			SyncManWatchdogStatus;			// 0x0440
	UINT16						cRes12[0x005F];					// 0x0442

/* ECATCHANGE_START(V4.00) APPL 4 */
	USERIALEEPROMCONFIG		SerialEepromConfig;				// 0x0500
#define		ESC_EEPROM_CONFIG								0x500
	USERIALEEPROMCONTROL		SerialEepromControl;				// 0x0502
#define		ESC_EEPROM_CONTROL							0x502
  	UINT16          			SerialEepromAddress[2];			// 0x0504
#define		ESC_EEPROM_ADDRESS							0x504
  	UINT16          			SerialEepromData[4];				// 0x0508
#define		ESC_EEPROM_DATA								0x508
    UINT16                      MIICtrlStat;                        // 0x0510
#define     ESC_MII_CTRLSTAT                            0x510
            #define ESC_MIICS_WRITEEN                           0x0001
            #define ESC_MIICS_PDICONTROL                        0x0002
            #define ESC_MIICS_MILINKDETACTIVE                   0x0004
            #define ESC_MIICS_PHYADDROFFSET                     0x00F8
            #define ESC_MIICS_CMD_IDLE                          0x0000
            #define ESC_MIICS_CMD_READ                          0x0100
            #define ESC_MIICS_CMD_WRITE                         0x0200
            #define ESC_MIICS_READ_ERROR                        0x2000
            #define ESC_MIICS_CMD_ERROR                         0x4000
            #define ESC_MIICS_BUSY                              0x8000
    UMIIADDRESSES               MIIAddresses;                       // 0x0512
#define     ESC_MII_ADDRESSES                           0x512
    UINT16                      MIIData;                            // 0x0514
#define     ESC_MII_DATA                                0x514
    UINT16                      MIIAccess;                          // 0x0516
#define     ESC_MII_ACCESS                              0x516
            #define ESC_MIIAC_ECATEXCLUSIVE                     0x0001
            #define ESC_MIIAC_PDIACCESS                         0x0100
            #define ESC_MIIAC_FORCEPDISTATE                     0x0200
	UINT16						cRes15[0x0070];					// 0x0518
/* ECATCHANGE_START(V4.00) APPL 4 */

	UFMMU	 						Fmmu[16];							// 0x0600
	UINT16						cRes16[0x0080];					// 0x0700

	#define	MAX_NO_OF_SYNC_MAN	16
	TSYNCMAN						SyncMan[MAX_NO_OF_SYNC_MAN]; 	// 0x0800
#define		ESC_ADDR_SYNCMAN								0x0800
#define		ESC_ADDR_SM_MBXWRITE						0x0800
#define		ESC_ADDR_SM_MBXREAD						0x0808
#define		ESC_ADDR_SM_MBXREAD_ECATCTRL			0x080E
#define		ESC_ADDR_SM_MBXREAD_PDICTRL			0x080F
	UINT16						cRes17[0x0040];					// 0x0880

	TDCTRANSMISSION			DcTransmission;					// 0x0900
	UINT16						cRes18[0x0028];					// 0x0930

	TDCINTERRUPT				DcInterrupt;						// 0x0980
	UINT16						cRes19[0x0318];					// 0x09D0
} TESCREGS;

typedef struct STRUCT_PACKED
{
	TESCREGS						Regs;
#define		ESC_ADDR_MEMORY								0x1000
	UINT16						Memory[0x0800];
} TESC;
	
typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8 			   		Byte[3];
									#define	MBX_OFFS_TYPE		2
									#define	MBX_OFFS_COUNTER	2
									#define	MBX_MASK_TYPE		0x0F00
									#define	MBX_MASK_COUNTER	0xF000
									#define	MBX_SHIFT_TYPE		8
									#define	MBX_SHIFT_COUNTER	12
#else
	UINT8 			   		Byte[6];
#if MOTOROLA_16BIT
									#define	MBX_OFFS_TYPE		4
									#define	MBX_OFFS_COUNTER	4
#else
									#define	MBX_OFFS_TYPE		5
									#define	MBX_OFFS_COUNTER	5
#endif
									#define	MBX_MASK_TYPE		0x0F
									#define	MBX_MASK_COUNTER	0xF0
									#define	MBX_SHIFT_TYPE		0
									#define	MBX_SHIFT_COUNTER	4
#endif
	UINT16 			   		Word[3];
									#define	MBX_OFFS_LENGTH	0
									#define	MBX_OFFS_ADDRESS	1
									#define	MBX_OFFS_FLAGS		2
} UMBXHEADER;

#define	MAX_MBX_DATA_SIZE	(MAX_MBX_SIZE-SIZEOF(UMBXHEADER))

typedef struct STRUCT_PACKED
{
  UMBXHEADER      			MbxHeader;
  UINT16            			Data[MAX_MBX_DATA_SIZE >> 1];
} TMBX;

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8		b[1];
				#define	COEHEADER_COESERVICEOFFSET				0
				#define	COEHEADER_COESERVICEMASK				0xF000		
				#define	COEHEADER_COESERVICESHIFT				12
#else
	UINT8		b[2];
#if MOTOROLA_16BIT
				#define	COEHEADER_COESERVICEOFFSET				0
#else
				#define	COEHEADER_COESERVICEOFFSET				1
#endif
				#define	COEHEADER_COESERVICEMASK				0xF0		
				#define	COEHEADER_COESERVICESHIFT				4
#endif
	UINT16 	w;
} TCOEHEADER;

typedef struct STRUCT_PACKED
{
	union
	{
		UINT16 Word;
			#define	SOEFLAGS_OPCODE		0x0007	// 0 = unused, 1 = readReq, 2 = readRes, 3 = writeReq, 4 = writeRes
																// 5 = notification (command changed notification)
			#define	SOEFLAGS_INCOMPLETE	0x0008	// more follows
			#define	SOEFLAGS_ERROR			0x0010	// an error word follows
			#define	SOEFLAGS_DRIVENO		0x00E0	// drive number

			#define	SOEFLAGS_DATASTATE	0x0100	// follows or requested
			#define	SOEFLAGS_NAME			0x0200	// follows or requested
			#define	SOEFLAGS_ATTRIBUTE	0x0400	// follows or requested
			#define	SOEFLAGS_UNIT			0x0800	// follows or requested
			#define	SOEFLAGS_MIN			0x1000	// follows or requested
			#define	SOEFLAGS_MAX			0x2000	// follows or requested
			#define	SOEFLAGS_VALUE			0x4000	// follows or requested
			#define	SOEFLAGS_DEFAULT		0x8000	// 
	} Flags;

	UINT16		IDN_Frag;			// if (InComplete==0) SERCOS IDN else FragmentsLeft
//	union
//	{
//		UINT8		Data[]			// rest of mailbox data		if (Error==0)
//		UINT16		ErrorCode		//									if (Error==1)
//	};
} TSOEHEADER;

//////////////////////////////////////////////////////////////////////

typedef union STRUCT_PACKED
{
#if BYTE_NOT_SUPPORTED
	UINT8 			   		Byte[4];
									#define	EMCY_OFFS_ERRORREGISTER		1
									#define	EMCY_MASK_ERRORREGISTER		0x00FF
									#define	EMCY_SHIFT_ERRORREGISTER	0
									#define	EMCY_OFFS_DIAGCODE			1
									#define	EMCY_MASK_DIAGCODE			0xFF00
									#define	EMCY_SHIFT_DIAGCODE			8
#else
	UINT8 			   		Byte[8];
#if MOTOROLA_16BIT
									#define	EMCY_OFFS_ERRORREGISTER		3
									#define	EMCY_OFFS_DIAGCODE			2
#else
									#define	EMCY_OFFS_ERRORREGISTER		2
									#define	EMCY_OFFS_DIAGCODE			3
#endif
#endif
	UINT16 			   		Word[4];
									#define	EMCY_OFFS_ERRORCODE			0
									#define	EMCY_OFFS_DIAGDATA			2
} UEMCY;

typedef struct STRUCT_PACKED
{
#if SOE_SUPPORTED
	UINT16						SoeHeader[2];
#endif
	UEMCY							Emcy;
} TEMCYMESSAGE;

/* ECATCHANGE_START(V4.00) */
typedef struct  STRUCT_PACKED
{
	UINT16		OpMode;			// = 1 (RRQ), = 2 (WRQ), = 3 (DATA), = 4 (ACK), = 5 (ERR), = 6 (BUSY)
	union
	{
		UINT32		Password;		// (RRQ, WRQ)       = 0 if unknown
		UINT32		PacketNo;		// (DATA, ACK)
		UINT32		ErrorCode;		// (ERR)
		struct
		{
			UINT16	Done;			// (BUSY)
			UINT16	Entire;		// (BUSY)
		} Busy;
	} Cmd;
//	union
//	{
//		CHAR		Name[]			// (RRQ, WRQ)	rest of mailbox data
//		UINT8		Data[]			// (DATA)		rest of mailbox data (if OpMode = 3)
//		CHAR		ErrorText[]		// (ERR)			rest of mailbox data
//	};
} TFOEHEADER;

typedef struct STRUCT_PACKED
{
	UINT16	Cmd;
			#define	EFW_CMD_IGNORE				0
			#define	EFW_CMD_MEMORY_TRANSFER	1
			#define	EFW_CMD_WRCODE				2
			#define	EFW_CMD_CHK_DEVICEID		3
			#define	EFW_CMD_CHKSUM				4
			#define	EFW_CMD_WRCODECHKSUM		5
			#define	EFW_CMD_SET_DEVID			6
			#define	EFW_CMD_CHKSUMCHKSUM		6
			#define	EFW_CMD_BOOTCHKSUM		7
	UINT16	Size;
	UINT32	Address;
#if _PIC18
	UINT8		Data[BL_PAGE_SIZE];
#else
	UINT16	Data[BL_PAGE_SIZE>>1];
#endif
} TEFWUPDATE;

typedef struct STRUCT_PACKED
{
  	UMBXHEADER        MbxHeader;
  	TFOEHEADER        FoeHeader;
	UINT16            Data[(MAX_MBX_SIZE-SIZEOF(TFOEHEADER)) >> 1];
} TFOEMBX;

/* ECATCHANGE_END(V4.00) */

#endif //_ESC_H_

