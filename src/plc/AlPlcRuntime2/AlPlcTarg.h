/*------------------------------------------------------------------------------
**
**	Copyright:	AXEL s.r.l. 2009
**
**	PLCIECTARG.H:	Interface between LogicLab and Runtime 
**
**-----------------------------------------------------------------------------
**
**	IMPORTANT:
**	THIS MODULE SHOULDN'T BE MODIFIED BY THE CUSTOMER EXCEPT FOR
**	PACKING PRAGMAS REQUIRED FOR CORRECTLY INTERFACE WITH LOGICLAB
**
**-----------------------------------------------------------------------------*/

#ifndef _PLCIECTARG_H
#define _PLCIECTARG_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _C_START_PACK_PRAGMA
#pragma _C_START_PACK_PRAGMA
#endif

/*	Download request responses */
#define SWP_OK				0   /* All OK */
#define SWP_STANBY			1   /* Request Stand-By */
#define SWP_ALARM			2   /* Request ALARM after download PLC */
#define SWP_IMGERROR		3   /* Invalid comunications */
#define SWP_FILEERROR		4
#define SWP_LOCK			5
#define SWP_COMMERROR		6
#define SWP_INIT			254	/*	PLC init status */
#define SWP_UNDEF			255	/*	Undefined response */

/*	PLC load error codes */
#define PLCERR_OK           				0
#define PLCERR_INVMOD       				1
#define PLCERR_NTASK        				2
#define PLCERR_TARGID       				3
#define PLCERR_MEMID        				4
#define PLCERR_PRECRUN      				5
#define PLCERR_TASKID       				6
#define PLCERR_TASKNUM      				7
#define PLCERR_DBPATCH_FAILED				8
#define PLCERR_FNPATCH_FAILED				9
#define PLCERR_PLCCHKSUM    				10
#define PLCERR_INVVERINFO					11
#define PLCERR_EMBEDDEDFNRELOC_FAILED		12
#define PLCERR_EMBEDDEDDBRELOC_FAILED		13
#define PLCERR_INVALID_TASK_CONFIGURATION	14
#define PLCERR_WRONG_DATA_ALIGNMENT			15

/*	PLC user code start number */
#define PLCERR_START_USER_ERROR				60000			

/*	Max task managed in compile */
#define MAX_TASK_PLC		( 32 * 6 )

	/*----------------------------------------------
    **
    **  Typedef tasks
    **
    **---------------------------------------------*/
#define TRGT_EMPTY				0x00000000
#define TRGT_HASIMAGE			0x00000001	
#define TRGT_MULTIIMAGE			0x00000002
#define TRGT_SET_EXEC_PERIOD	0x00000004
#define TRGT_GET_EXEC_PERIOD	0x00000008
#define TRGT_GET_EXEC_TIME		0x00000010
#define TRGT_GET_EXEC_COUNT		0x00000020
#define TRGT_GET_EXEC_STATUS	0x00000040

	/*	Size task name */
#define DIM_TNAME			0x1E

typedef struct _PLCIEC_TASKDEF
{
	uint16_t	taskId;				/*	Task ID (the same of resources) */
	char_t		name[ DIM_TNAME	];	/*	Task name */
	uint32_t	flags;				/*	Task flags (TRGT_...) */
	uint32_t	usExecTime;			/*	Execution time in microseconds */
	uint32_t	dwSpare;
} _C_PACK_
PLCIEC_TASKDEF;

	/*----------------------------------------------
    **
    **  Typedef Function table
    **
    **--------------------------------------------*/

    /*  Size function name */
#define DIM_FNAME   0x20

typedef struct _PLCIEC_FCNREC
{
    char_t    name[ DIM_FNAME ];  /*  Function Name */
    uint32_t  addr;               /*  Address of function */
}_C_PACK_
PLCIEC_FCNREC;


    /*----------------------------------------------
    **
    **  Typdefs datablock table
    **
    **---------------------------------------------*/

    /*  Type of db */
#define DBTY_MEMO   0
#define DBTY_INP    1
#define DBTY_OUT    2
#define	DBTY_HND	100

    /*  Type of access variable db */
#define DBRW_R      0x01
#define DBRW_W      0x02
#define DBRW_RW     ( DBRW_W | DBRW_R )

typedef struct _PLCIEC_DBREC
{
    uint8_t		img;        /*  If TRUE var has image */
    uint8_t    	type;       /*  Type of DATA BLOCK (DBTYPE) */
    uint16_t  	dbId;       /*  Index of DATA BLOCK */
    uint32_t  	addr;       /*  Base address of DATA BLOCK */
    uint32_t  	nel;        /*  Number of DATA BLOCK */
    uint8_t    	dataSize;   /*  Size data for index mul (-1 = bit) */
    uint8_t    	rw;         /*  Read/Write attribute */
} _C_PACK_
PLCIEC_DBREC;

        
	/*	Definitions of target features */
#define TRG_EXTPLC				0x00000001	/*	header _TARGETEXT */
#define TRG_FLOATSUPPORT		0x00000002	/*	Floating point support on device */
#define TRG_INTERNALDB			0x00000004	/*	Internal parameters Database on device */
#define TRG_DOWNLREQ			0x00000008	/*	Need request for download */
#define TRG_HOTSWAP				0x00000010	/*	Load PLC without stop the tasks */
#define TRG_NODOWNLAREA			0x00000020	/*	Device without dedicated download area */
#define TRG_DEBUGAREA			0x00000040	/*	Device with dedicated debug area */
#define TRG_RELOCATABLE 		0x00000080	/*	..... */	
#define TRG_CHECKSUM			0x00000100	/*	Code checksum */
#define TRG_DOUBLEAREA			0x00000200	/*	Double area SWAP and NON SWAP */
#define TRG_FIXEDTASKS			0x00000400	/*	Tasks fixed and always exist */
#define TRG_INFOHEADER			0x00000800	/*	Info about PLC application */
#define TRG_INITHANDLES			0x00001000	/*	Init handles of associate functions */
#define TRG_CHECKARRAYS			0x00002000	/*	Function of runtime check array */
#define TRG_CHECKSTACK			0x00004000	/*	Function of runtime check stack */
#define TRG_CHECKSTACKCALL		0x00008000	/*	Function of runtime check stack inside functions */
#define TRG_DISABLEDIRECTACC	0x00010000	
#define TRG_TRACEAREA			0x00020000	/*	Device has trace area  */
#define	TRG_SYNCPATCH			0x00040000	/*	Sync patch function ( Synchro Trigger ) */
#define	TRG_AUXDATAAREA			0x00080000	/*	Aux data area */
#define	TRG_TRACECALLS			0x00100000	/*	Trace of FB call and FUNCTION call */
#define	TRG_CHECKPOINTERS		0x00200000	/*	Function of runtime check pointers */
#define	TRG_BREAKPTS			0x00400000	/*	Device has breakpoints */
#define	TRG_FORCEIO				0x00800000	/*	Function of forcing I/O */
#define	TRG_TIME_PLUGIN			0x01000000	/*	Device has internal system time (sysTimer) */
#define	TRG_REAL_PLUGIN			0x02000000	/*	Device manages operations with real using optional module */
#define	TRG_STRING_PLUGIN		0x04000000	/*	Device manages strings using optional module */
#define	TRG_MATH_PLUGIN			0x08000000	/*	Device manages advanced math operations using optional module */
#define	TRG_EXTFEATURES			0x80000000	/*	Extended features */
	/*	Definitions of extended target features */
#define TRG_DYN_LINK_SUPPORT	0x00000001	/*	Device has dynamic links */
#define	TRG_DBACC_PATCH			0x00000002	/*	Runtime patch for DATA BLOCK access */
#define	TRG_DBACC_INDIR			0x00000004	/*	Use DATA BLOCKS with indirect address */
#define	TRG_FNACC_PATCH			0x00000008	/*	Runtime patch for EMBEDDED FUNCTIONS */
#define	TRG_FNACC_INDIR			0x00000010	/*	Use EMBEDDED FUNCTIONS with indirect address */
#define	TRG_DATA_RELAT			0x00000020	/*	Relocable Data area */
#define	TRG_CODE_RELAT			0x00000040	/*	Relocable Code area  */
#define	TRG_RTV2				0x00000080	/*	Release 2 of runtime (register for memoId, commands handshake, RQ dwnl with registers...) */
#define TRG_TASK_SETTINGS		0x00000100	/*	Dynamic task settings support */
#define TRG_RUNTIME_CTRL		0x00000200	/*	PLC run-time execution control from LogicLab */
#define TRG_SOURCE_CODE			0x00000400	/*	Target supports he storage of the source code  */
#define TRG_EMBED_SOURCE_CODE	0x00000800	/*	Target supports he storage of the source code embedded into binary code */
#define TRG_RUNTIME_STATUS		0x00001000	/* PLC run-time status available to LogicLab */

    /*----------------------------------------------
    **
    **  Definitions of PLC
    **
    **--------------------------------------------*/

    /*  Record */
#define CIMG_IDMOD      0x5750
#define CIMG_IDMOD_EX   0x5751
#define CIMG_IDTASK     0x544B
#define CIMG_IDINFO		0x5745
#define CIMG_IDDB       0x554C
#define CIMG_PLCCHECK	0x03317827
#define CIMG_IDREALL	0x524C
#define CIMG_IDSRC		0x5352

    /*  Header */
typedef struct _CIMG_HEAD
{
    uint16_t  Id;             /*  Record 0x5750 */
    uint32_t  targId;         /*  Target version */
    uint32_t  memoId;         /*  ID memory configuration */
    uint32_t  modId;          /*  PLC code identification */
    uint32_t  fileSize;       /*  File size */
    uint32_t  codeSize;       /*  Code Size */
    uint32_t  codeOffs;       /*  Offset from start code */
    uint32_t  initCodeOffs;   /*  Offset from init start code */
    uint16_t  numTask;        /*  Number of tasks */
} _C_PACK_
CIMG_HEAD;


	/*	Extended header */
typedef struct _CIMG_HEAD_EX
{
	uint16_t	Id;				/*	Record 0x5751 */
	uint16_t	numTask;		/*	Number of tasks */
	uint32_t	targId;			/*	Target version */
	uint32_t	memoId;			/*	ID memory configuration */
	uint32_t	modId;			/*	PLC code identification */
	uint32_t	fileSize;		/*	File size */
	uint32_t	codeSize;		/*	Code size */
	uint32_t	codeOffs;		/*	Offset of the code from the start of the image */
	uint32_t	initCodeOffs;	/*	Offset of the init code from the start of the image */
	uint32_t	chksum;			/*	Checksum of PLC code */
	uint32_t	execAddr;		/*	Index of used swap area */
	uint32_t	reallocOffs;	/*	Offset of realloc table (dynamic link) */
	uint32_t	sourceOffs;		/*	Offset of the source code from the start of the image */
	uint32_t	spare[ 4 ];		/*	Spare data to use in the future */
}_C_PACK_
CIMG_HEAD_EX;
    
    /*  Task code definition */
typedef struct _CIMG_TASK
{
    uint16_t  Id;             /*  Record 0x544B */
    uint16_t  taskId;         /*  Task id */
    uint32_t  offTask;        /*  Offset of task code */
    uint32_t  offInp;         /*  Offset of input function */
    uint32_t  offOut;         /*  Offset of output function */
}_C_PACK_
CIMG_TASK;

    /*  Extended task code definition */
typedef struct _CIMG_TASK_EX
{
    uint16_t  Id;             /*  Record 0x544B */
    uint16_t  taskId;         /*  Identificatore task */
    uint32_t  offTask;        /*  Offset of task code */
    uint32_t  offInp;         /*  Offset of input function */
    uint32_t  offOut;         /*  Offset of output function */
    uint32_t  offInit;        /*  Offset of init function */
    uint32_t  adrLine;	      /*  Address of var for line number */
    uint32_t  adrProgName;    /*  Address of string which is the program name */
    uint32_t  exePeriod;      /*  Periodic task execution period */
}_C_PACK_
CIMG_TASK_EX;

	/*	Application versioning	*/
#define DIM_APP_NAME	10
typedef struct _CIMG_HEAD_INFO
{
	uint16_t	Id;						/*	Record 0x5745	*/
	char_t		strApp[ DIM_APP_NAME ];	/*	Application name	*/
	uint16_t	appVer1;				/*	Major version number	*/
	uint16_t	appVer2;				/*	Minor version number	*/
	uint32_t	buildTime;				/*	Build time	*/
	uint32_t	spare32[ 3 ];
}_C_PACK_
CIMG_HEAD_INFO;


    /*----------------------------------------------
    **
    **  Control bits for realloc memory used 
	**  by tasks
    **
    **--------------------------------------------*/

typedef struct _TASKCTRLBITS
{
	uint32_t	modId;											/*	Code ID for calculate realloc		*/
	uint32_t	taskBits[ MAX_TASK_PLC / sizeof( uint32_t ) ];	/*	Bit for request realloc  			*/
	uint32_t	spare0;											/*	Unused (ex code ID for no-swap area)*/
} _C_PACK_
TASKCTRLBITS;
    

    /*----------------------------------------------
    **
    **  Addresses and sizes of target
    **
    **---------------------------------------------*/

typedef struct _TARGETADDRS
{
    uint32_t  targId;             /*  Target Version */
    uint32_t  memoId;             /*  ID memory configuration */
    uint32_t  addrCode;           /*  Code address */
    uint32_t  addrAutoVar;        /*  Address of auto vars */
    uint32_t  addrBitVar;         /*  Address of bit auto vars */
    uint32_t  addrRitVar;         /*  Address of retentive auto vars */
    uint32_t  addrDataBlockTab;   /*  Address of DATA BLOCKS table */
    uint32_t  addrFunctionTab;    /*  Address of EMBEDDED FUNCTIONS table */
    uint32_t  numDataBlockRec;    /*  Number of DATA BLOCK records */
    uint32_t  numFunctionRec;     /*  Number of EMBEDDED FUNCTIONS records */
    uint32_t  dimCode;            /*  Max size of code */
    uint32_t  dimData;            /*  Max size of auto vars */
    uint32_t  dimDataBit;         /*  Max size of auto vars */
    uint32_t  dimRitData;         /*  Max size of retentive auto vars */
    uint32_t  dimDebug;           /*  Max size of debug area */
} _C_PACK_
TARGETADDRS;

    /*----------------------------------------------
    **
    **  Extended addresses and sizes of target
    **
    **---------------------------------------------*/
typedef struct _TARGETEXT
{
	uint32_t		addrCode1;			/*	Address of Code area 1 (swap PLC) */
	uint32_t		addrCode2;			/*	Address of Code area 2 (swap PLC) */
	uint32_t		addrTaskCtrlBits;	/*	Address of task control bits area  */
	uint32_t		addrDebug;			/*	Address of debug area */
	uint32_t		addrTrace;			/*	Address of trace area */
	uint32_t		dimTrace;			/*	Max size of trace area */
	uint32_t		spare0[ 2 ];		/*	Ex no-swap code management (obsolete) */
	uint32_t		stackLimit;			/*	Limits for stack allocation */
	uint32_t		addrDataAux;		/*	Address of aux data area */
	uint32_t		dimDataAux;			/*	Size of aux data area */
	uint32_t		addrSysTimer;		/*	Address of system timer */
	uint32_t		extFeatures;		/*	Extended features */
	uint32_t		addrTaskTable;		/*	Task table address */
	uint32_t		numTaskRec;			/*	Number of task table recs */
	uint32_t		spare1[ 1 ];		/*	Disponibili per future implementazioni */
}_C_PACK_
TARGETEXT;
    
    
    /*----------------------------------------------
    **
    **  Extended addresses and sizes of target
    **
    **---------------------------------------------*/

typedef struct _TARGETADDRS_EX
{
    TARGETADDRS t;      /*  Base info */
    TARGETEXT   e;      /*  Extended info */
} _C_PACK_
TARGETADDRS_EX;

	/*----------------------------------------------------------------
	**
	**		Dynamic link structures
	**
	** --------------------------------------------------------------*/

	/*	Header of realloc table */
typedef struct _CIMG_REALLOC_TAB
{
	uint16_t	Id;				/*	Record 0x524C */
	uint16_t	tabType;		/*	Type of realloc table */
	uint32_t	nRecCheckDb;	/*	Number or record for check DATA BLOCKS */
	uint32_t	nRecDb;			/*	Number of record for realloc DATA BLOCKS */
	uint32_t	nRecFunct;		/*	Number of record for realloc EMBEDDED FUNCTIONS */
	uint32_t	offsCheckDb;	/*	Offset of check DB table */
	uint32_t	offsTabDb;		/*	Offset of realloc DB table */
	uint32_t	offsTabFunct;	/*	Offset of EMBEDDED FUNCTIONS table */
}_C_PACK_
CIMG_REALLOC_TAB;

typedef struct _LINK_DBCHECK
{
	bool_t		img;		/*	TRUE if request image */
	uint8_t		type;		/*	Type of DATA BLOCK (DBTYPE) */
	uint16_t	dbId;		/*	Index of DATA BLOCK */
	uint32_t	nel;		/*	Element number of DATA BLOCK */
	uint32_t	dataSize;	/*	Element size of DATA BLOCK */
}_C_PACK_
LINK_DBCHECK;

#if defined (BIT_FIELD)
typedef struct _LINK_OFFS
{
	BIT_FIELD	codeOffs  	: 24,
				instrType 	: 7,
				extRec		: 1;
}_C_PACK_
LINK_OFFS;
#endif

typedef struct _REC_LINK_DB
{
	uint16_t		dbIdx;
	uint32_t		offset;
	uint16_t		type;
	uint16_t		numLink;
}_C_PACK_
REC_LINK_DB;

typedef struct _REC_LINK_FUNC
{
	char_t		name[ DIM_FNAME	];
	uint16_t	numLink;
}_C_PACK_
REC_LINK_FUNC;

typedef struct _REC_LINK_DB_EX
{
   uint16_t    rec_db_ptr_addr;
   REC_LINK_DB rec_db;
}_C_PACK_
REC_LINK_DB_EX;

typedef struct _REC_LINK_FUNC_EX
{
   REC_LINK_FUNC    rec_fn;
   uint16_t         rec_fn_ptr_addr;
}_C_PACK_
REC_LINK_FUNC_EX; 

typedef struct _DYNAMIC_ADDRESSES
{
	uint32_t codeArea;						/* Start address of the active code area */
	uint32_t dataArea;						/* Start address of the data area */
	uint32_t bitDataArea;					/* Start address of the bit data area */
	uint32_t retainDataArea;				/* Start address of the retain data area */
	uint32_t numberOfDataBlocks;			/* Number of records in the pointer table of the data blocks */
	uint32_t dataBlockPointerTable;			/* Address of the pointer table of the data blocks */
	uint32_t numberOfEmbeddedBlocks;		/* Number of records in the pointer table of the embedded blocks */
	uint32_t embeddedBlockPointerTable;		/* Address of the pointer table of the embedded blocks */
} _C_PACK_
DYNAMIC_ADDRESSES;

	/*----------------------------------------------------------------
	**
	**		Source code structures
	**
	** --------------------------------------------------------------*/

typedef struct _CIMG_SOURCE_HEAD
{
	uint16_t	Id;			/*	Keyword 0x5352 (CIMG_IDSRC)	*/
	uint16_t	spare0;		
    uint32_t	sourceId;	/*	Unique ID of source file	*/
	uint32_t	size;		/*	Source code size			*/
	uint32_t	chksum;		/*	Source code checksum		*/
} _C_PACK_
CIMG_SOURCE_HEAD;

typedef struct _SYNC_TRIGGER
{
	uint32_t	adrJmpPatch;	/* Patch address */
	uint32_t	lenByte;		/* Patch size */
	uint8_t		code[ 24 ];		/* Patch code */
} _C_PACK_
SYNC_TRIGGER;

#ifdef _C_END_PACK_PRAGMA
#pragma _C_END_PACK_PRAGMA
#endif

#ifdef __cplusplus
}
#endif

#endif  /* _PLCIECTARG_H */


