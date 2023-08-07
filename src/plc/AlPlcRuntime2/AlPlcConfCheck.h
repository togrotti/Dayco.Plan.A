/*------------------------------------------------------------------------------
**
**	Copyright:			AXEL s.r.l. 2010
**
**	AlPlcConfCheck.h:	Configuration check for LogicLab PLC runtime
**
**-----------------------------------------------------------------------------
**
**	IMPORTANT:
**	THIS MODULE SHOULDN'T BE MODIFIED BY THE CUSTOMER
**
**------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------*/
/*	
**	This directive is used only to delimit the documentation of
**	the runtime configuration file.
**	The use of preprocessor directive allows to copy and paste directly the
**	source code into an implementation file
*/
#ifdef DESCRIPTION_OF_RUNTIME_CONFIGURATION

	/*	
		The runtime configuration is tipically defined in a C source file that includes
		the module "AlPlcRuntimeCore.c" at the end of itself.
		Several runtimes can be instantiated by creating different copies
		of the configuration file and assigning different names to the three
		entry points described below (ALPLCINIT, ALPLCLOAD, ALPLCMANAGE)

		The runtime configuration can be freely divided in several C and H
		module depending on the system firmware structure needings.
		The key point is that AlPlcRuntimeCore.c must "know" (or "see")
		all the definitions described below

		In the first lines of that source file are normally placed all
		necessary include files with external data/function declarations 
		used in the runtime configuration.
	*/

	/*
	** If EXTERNAL_MISRA_C_TYPES is defined AlPlCMisraC.h definitions have no effects.
	** Uncomment this definition only if your firmware has already a file that implements Misra-C definitions.
	*/
	/*#define EXTERNAL_MISRA_C_TYPES	*/
	#if defined( EXTERNAL_MISRA_C_TYPES )
	#include "your_misra_c_file.h"
	#endif

	/*
	** The following are the target feature supported. All switches should be defined.
	** Put 0 or 1 to enable or disable the feature
	*/
	#define PLC_S_FLOATSUPPORT			0/1				/*	Floating point support on device */
	#define PLC_S_HOTSWAP				0/1				/*	Load PLC without stop the tasks */
	#define PLC_S_NODOWNLAREA			0/1				/*	Device without separate download area */
	#define PLC_S_DEBUGAREA				0/1				/*	Device with dedicated debug area */
	#define PLC_S_CHECKSUM				0/1				/*	Code checksum support */
	#define PLC_S_FIXEDTASKS			0/1				/*	Tasks fixed and always exist */
	#define PLC_S_INFOHEADER			0/1				/*	Info about PLC application */
	#define PLC_S_INITHANDLES			0/1				/*	Init handles of associate functions */
	#define PLC_S_CHECKARRAYS			0/1				/*	Function of runtime check array */
	#define PLC_S_CHECKSTACK			0/1				/*	Function of runtime check stack */
	#define PLC_S_CHECKSTACKCALL		0/1				/*	Function of runtime check stack inside functions */
	#define PLC_S_TRACEAREA				0/1				/*	Device has trace area */
	#define	PLC_S_SYNCPATCH				0/1				/*	Sync patch function ( Synchro Trigger ) */
	#define	PLC_S_AUXDATAAREA			0/1				/*	Auxiliary data area */
	#define	PLC_S_TRACECALLS			0/1				/*	Trace of FB call and FUNCTION call */
	#define	PLC_S_CHECKPOINTERS			0/1				/*	Function for runtime pointers check */
	#define	PLC_S_BREAKPTS				0/1				/*	When PLC_S_BREAKPTS is switched on, LogicLab enables breakpoints, when the developer switch to debug mode. */
	#define	PLC_S_FORCEIO				0/1				/*	Function of forcing I/O */
	#define	PLC_S_TIME_PLUGIN			0/1				/*	Device has internal system time (sysTimer) */
	#define	PLC_S_REAL_PLUGIN			0/1				/*	Device supports real operations using AlPlcReal plugin */
	#define	PLC_S_STRING_PLUGIN			0/1				/*	Device supports string operations using AlPlcString plugin */
	#define	PLC_S_MATH_PLUGIN			0/1				/*	Device supports math operations using AlPlcMath plugin */
	#define PLC_S_DYN_LINK_SUPPORT		0/1				/*	Device has dynamic links */
	#define	PLC_S_DBACC_PATCH			0/1				/*	Runtime patch for DATA BLOCK access */
	#define	PLC_S_DBACC_INDIR			0/1				/*	Use DATA BLOCKS with indirect address */
	#define	PLC_S_FNACC_PATCH			0/1				/*	Runtime patch for EMBEDDED FUNCTIONS */
	#define	PLC_S_FNACC_INDIR			0/1				/*	Use EMBEDDED FUNCTIONS with indirect address */
	#define	PLC_S_DATA_RELAT			0/1				/*	Relocable Data area */
	#define	PLC_S_CODE_RELAT			0/1				/*	Relocable Code area */
	#define PLC_S_TASK_SETTINGS			0/1				/*	Dynamic task settings support */
	#define PLC_S_RUNTIME_CTRL			0/1				/*	PLC run-time execution control from LogicLab */
	#define PLC_S_SOURCE_CODE			0/1				/*	Target supports the storage of the source code embedded in the binary code */
	#define PLC_S_SOURCE_CODE_AREA		0/1				/*	Target supports the storage of the source code in a dedicated area */
	#define PLC_S_RUNTIME_STATUS		0/1				/* (optional) PLC run-time status available to LogicLab */

	/*	Standard definitions	*/
	#include "AlPlcRuntimeCore.h"
	#include "AlPlcCDefs.h"
	#include "AlPlcTarg.h"

	/*	Defines target info	*/
	#define ALPLC_TARGET_ID		"target_id"
	#define ALPLC_TARGET_COMM	"target_comm"

	/*
	** Type of configuration:
	** 	- Static (const/ROM)
	** 	- Dynamic (RAM, runtime built)
	** Only one of these two switches should be defined:
	*/
	#define PLC_CONST_CONF
	#define PLC_RUNTIME_CONF

	/*
	** The following switch enables the saving of the PLC code file and PLC source file
	*/
	#define PLC_CODE_FILE_SAVE_CONF
	#define PLC_SOURCE_FILE_SAVE_CONF

	/*
	** Enable synchronous swap feature.
	**
	** When PLC_CONF_SYNC_SWAP is defined, the PLC run-time expects the hosting
	** environment to call the PLC run-time entry point ALPLCSWAP, which is
	** responsible to swap the PLC application. The hosting environment is
	** notified after a successful download through the callback
	** ALPLC_F_WAIT_SWAP.
    */
	#define PLC_CONF_SYNC_SWAP

	/*
	**	The three following definitions assign the name to
	**	the public functions of the runtime.
	**	The names on the right are the entry points of
	**	the runtime for the target firmware
	*/
	#define ALPLCINIT		AlPlcInit		/*	Init oft the runtime, prototype: void ALPLCINIT(void)	*/
	#define ALPLCLOAD		AlPlcLoad		/*	First time load code into exec memory, prototype: bool_t ALPLCLOAD(void)	*/
	#define ALPLCMANAGE		AlPlcManage		/*	Manage of runtime state, prototype: void ALPLCMANAGE(void)	*/

	/*
	** When PLC_CONF_SYNC_SWAP is defined, the PLC run-time provides a fourth
	** entry point, which the hosting environment is responsible to call at the
	** right time (according to its own requirements), to actually swap the PLC
	** application.
    */
	#define ALPLCSWAP		AlPlcSwap		/* Swap the PLC application */

	/*
	** The following are the target elements definitions (tasks, datablocks, functions)
	*/
	#define PLC_TASK_TAB				m_rtPlcEnvTabTasks		/*	Pointer to task table (type PLCIEC_TASKS *) */
	#define PLC_TASK_DEFS				m_rtPlcEnvTabTaskDefs	/*	Pointer to task table (type PLCIEC_TASKDEF *) */
	#define PLC_DB_TAB					m_rtPlcEnvTabDBlocks	/*	Pointer to datablocks table (type PLCIEC_DBREC *) */
	#define PLC_FUNCT_TAB				m_rtPlcEnvTabFunctions	/*	Pointer to functions table (type PLCIEC_FCNREC *) */

	/*
	** The following are the target areas for storing executable code, data, retain data and bit addressable data
	** The code and data areas definitions are mandatory.
	** Retain and bit-addressable areas are optional and should be defined only if target supports them
	*/
	#define PLC_CODE_AREA1				m_rtPlcCode1Addr		/*	Required - Address of main or first hot swap code area */
	#define PLC_CODE_AREA2				m_rtPlcCode2Addr		/*	Address of second hot swap code area */
	#define PLC_DATA_AREA				m_rtPlcDataAddr			/*	Required - Address of PLC data area */
	#define PLC_BITDATA_AREA			m_rtPlcDataBitAddr		/*	Address of bit data area (if supported by processor) */
	#define PLC_RETDATA_AREA			m_rtPlcDataRetAddr		/*	Address of PLC retain data area */

	#define PLC_CODE_SIZE				m_rtCodeSize			/*	Required - Size of code area(s) */
	#define PLC_DATA_SIZE				m_rtDataSize			/*	Required - Size of data area */
	#define PLC_BITDATA_SIZE			m_rtDataBitSize 		/*	Size of bit data area */
	#define PLC_RETDATA_SIZE			m_rtDataRetSize			/*	Size of PLC retain data area */

	/*
	** The following are the definions of debug and trace areas. These areas are optional
	** and should be defined only if target supports these features (PLC_S_DEBUGAREA and/or PLC_S_TRACEAREA)
	*/
	#define PLC_DEBUGDATA_AREA			m_rtPlcDebugAddr		/*	Address of debug data area */
	#define PLC_TRACEDATA_AREA			m_rtPlcTraceAddr		/*	Address of trace area */
	#define PLC_DEBUGDATA_SIZE			m_rtDebugSize 			/*	Size of debug data area */
	#define PLC_TRACEDATA_SIZE			m_rtTraceSize 			/*	Size of trace area */

	/*
	** More specific features
	*/
	#define PLC_STACK_LIMIT				0						/*	Limits for stack allocation */
	#define PLC_AUXDATA_AREA			NULL					/*	(optional) Address of auxiliary data area */
	#define PLC_AUXDATA_SIZE			0						/*	(optional) Size of auxiliary data area */
	#define PLC_SYSTIMER_ADDR			NULL					/*	(optional) Address of system timer */

	/*
	** Runtime data exchange with LogicLab by means of
	** comunication objecs (as Modbus register, CANopen objects etc.)
	*/
	#define PLC_R_COMMAND				m_wRegCommand			/*	LogicLab request for target reset/reload of PLC */
	#define PLC_R_DOWNLADDR				m_dwRegDownlAddr		/*	Address for the next download */
	#define PLC_R_INFOAREA				m_dwRegInfo				/*	Address of runtime info area */
	#define PLC_R_DOWNL_RESP			m_wDownlResp			/*	Target response for a LogicLab request for download acknowledge */
	#define PLC_R_APPL_ID				m_wRegApplID			/*	Application ID for the currently running PLC */
	#define PLC_R_MEMO_ID				m_dwExeMemoID			/*	Firmware memory ID */
	#define PLC_R_FULLMEMO_ID			m_dwFullMemoID			/*	Firmware memory ID with db and function check */
	#define PLC_R_COMMAND_RESPONSE		m_dwCommandResponse		/*	Response to LogicLab requests */

	/*
	**	PLC control variables shared with firmware to allow
	**	the firmware itself to control and manage PLC
	**	execution states
	*/
	#define PLC_V_STATUS_OK				sch_plcOk				/*	Required - bool_t - valid PLC */
	#define PLC_V_ERROR_CODE			sch_plcErrCode			/*	Required - uint16_t - load PLC error code */
	#define PLC_V_STATUS_ALARM						  			/*	Optional - bool_t - alarm status */
	#define PLC_V_RUNTIME_RUNNING_STATE							/*	Optional - bool_t - host system defines an independent run-time running state (with respect to PLC application status) */
    #define PLC_V_FILESIZE              ulPlcFileSize           /*  uint32_t - plc file size */

	/*
	**	PLC application version info
	**	Here ar mapped target variables that hold application version info
	*/
	#if ( PLC_S_INFOHEADER == 1 )
	#define PLC_V_APPVER_NAME	m_strAppName		/*	Required if PLC_S_INFOHEADER is on - char_t* - Application name */
	#define PLC_V_APPVER_MAJOR	m_wVerMajor			/*	Required if PLC_S_INFOHEADER is on - uint16_t - Major version number */
	#define PLC_V_APPVER_MINOR	m_wVerMinor 		/*	Required if PLC_S_INFOHEADER is on - uint16_t - Minor version number */
	#define PLC_V_APPVER_BUILD	m_buildTime			/*	Optional - uint32_t - Build time (in seconds since 00:00:00 UTC on 1 January 1970) */
	#endif
	
	/* Breakpoint management */

	/*
	**	PLC_V_HIT_BREAKPOINT_MASK stores the state (hit or not) of all the
	**	breakpoints set by the developer.
	**	PLC_V_HIT_BREAKPOINT_MASK has to be defined as an l-value of type
	**	uint32_t (32-bit unsigned integer). Typically, it is defined as a C/C++
	**	variable of type uint32_t.
	**	When PLC_S_BREAKPTS is switched on, PLC_V_HIT_BREAKPOINT_MASK has to
	**	be defined.
    */
	#define PLC_V_HIT_BREAKPOINT_MASK		activeBreakpointMask

	/*
	**	PLC_V_RUN_BREAKPOINT_MASK stores the command issued by the
	**	development environment to run a breakpoint in hit state.
	**	PLC_V_RUN_BREAKPOINT_MASK has to be defined as an l-value of type
	**	uint32_t (32-bit unsigned integer). Typically, it is defined as a C/C++
	**	variable of type uint32_t.
	**	When PLC_S_BREAKPTS is switched on, PLC_V_RUN_BREAKPOINT_MASK has to
	**	be defined.
    */
	#define PLC_V_RUN_BREAKPOINT_MASK		clearedBreakpointMask
	
	/*
	**	PLC_F_GET_PLC_BREAKPOINT_ADDRESS evaluates to the address of the C/C++
	**	function which implements breakpoints and which is called by the PLC
	**	application when a breakpoint set by the developer is hit.
	**
	**	Function prototype is the following:
	**		bool_t PlcBreakPoint( uint16_t bpno, uint32_t bpaddr );
	**	
	**	When PLC_S_BREAKPTS is switched on, PLC_F_GET_PLC_BREAKPOINT_ADDRESS has
	**	to be defined.
    */
	#define PLC_F_GET_PLC_BREAKPOINT_ADDRESS	PlcBreakPoint

	/*
	**	IEC runtime callbacks used to manage target
	**	specific functions (disable interrupts, stop PLC execution, memory copy etc.)
	**	(leave commented if not used/required)
	*/

	/*
	**	Memory reset functions:
	*/
	#define PLC_F_ZERODATA_AREA			void zeromem(uint8_t * addr, uint32_t mem)	/*	Function call for reset data area: void funct( uint8_t address, uint32_t size_in_bytes ) */
	#define PLC_F_ZERODATABIT_AREA		void zeromem(uint8_t * addr, uint32_t mem)	/*	Function call for reset data bit area: void funct( uint8_t address, uint32_t size_in_bytes ) */
	#define PLC_F_ZERODATARET_AREA		void zeromem(uint8_t * addr, uint32_t mem)	/*	Function call for reset data retain area: void funct( uint8_t address, uint32_t size_in_bytes ) */
	#define PLC_F_ZEROAUXDATA_AREA		void zeromem(uint8_t * addr, uint32_t mem)	/*	Function call for reset data retain area: void funct( uint8_t address, uint32_t size_in_bytes ) */

	/*
	**	Runtime notifications callbacks
	*/
	#define PLC_F_INIT_DOWNLOAD			void initdownload()					/*	Function call preparing the target for a new download */
	#define PLC_F_STOP_TASKS			void LLStopSystem()					/*	Function call for stop PLC execution */
	#define PLC_F_START_TASKS			void LLStartSystem()				/*	Function call for starting PLC execution */
	#define PLC_F_SYSLOCK_REQ(t)		bool_t startplctasks(uint16_t nt)	/*	Reqest for system lock required if task T data is to reset */
	#define PLC_F_SWAPCHECK(t,modified) uint16_t SwapCheck(uint16_t nTask,bool_t modified)
	#define PLC_F_INTERRUPTDISABLE		void disableinterrupt(void)			/*	Disable interrupt function */
	#define PLC_F_INTERRUPTENABLE		void enableinterrupt(void)			/*	Enable interrupt function */
	#define PLC_F_END_LOAD(s)			void PlcEndLoad(bool_t status)		/*	End PLC load notification */

	/*
	** 	Function used to change the task period according to the application settings
	*/
	#define PLC_F_TASK_CONFIGURATION_INIT			bool_t InitTaskConfiguration()
	#define PLC_F_UPDATE_TASK_PERIOD(task,period)	bool_t UpdateTaskPeriod( uint16_t task, uint32_t period )
	#define PLC_F_TASK_CONFIGURATION_END			bool_t EndTaskConfiguration()

	/*
	** 	Call-backs invoked before and after the execution of a PLC run-time start/stop command
	*/
	#define PLC_F_START_RUNTIME_INIT	bool_t InitStartPlcRuntime()	/* Invoked before the execution of a PLC run-time start command */
	#define PLC_F_START_RUNTIME_END		bool_t EndStartPlcRuntime()		/* Invoked after the execution of a PLC run-time start command */
	#define PLC_F_STOP_RUNTIME_INIT		bool_t InitStopPlcRuntime()		/* Invoked before the execution of a PLC run-time stop command */
	#define PLC_F_STOP_RUNTIME_END		bool_t EndStopPlcRuntime()		/* Invoked after the execution of a PLC run-time stop command */

	/*
	** 	Read the host system-defined independent run-time running state
	**	(with respect to PLC application status) 
	*/
	#define PLC_F_UPDATE_RUNTIME_RUNNING_STATE	bool_t UpdateRuntimeRunningState()

	/*
	** Callbacks for executable code saving/reading to/from permanent memory
	** Mandatory if PLC_CODE_FILE_SAVE_CONF is active
	*/
	#define PLC_F_WRITE(dat,off,len)	bool_t FWrPlc(void* dat,uint32_t off,uint32_t len)	/* Function used to save PLC code into permanent memory */
	#define PLC_F_READ(dat,off,len)		bool_t FRdPlc(void* dat,uint32_t off,uint32_t len)	/* Function used to load PLC code from permanent memory */
	#define PLC_F_READ_INIT				bool_t RdPlcInit()									/* Function used to initialize permanent memory read */
	#define PLC_F_READ_END				void RdPlcEnd()										/* Function used to terminate permanent memory read */
	#define PLC_F_WRITE_INIT			bool_t WrPlcInit()									/* Function used to initialize permanent memory writing */
	#define PLC_F_WRITE_END				void WrPlcEnd()										/* Function used to terminate permanent memory writing */

	/*
	** Callbacks to calculate Exe Memo ID or Full Memo ID with an user-defined algorithm.
	** User functions should be used only in special cases and the implementor has to assure that the computed values satisfy the same requirements the default implementations do.
	** A simpler implementation would just assure that the computed values always change if the firmware changes: this implementation is somewhat more limitating, with special respect to successful PLC load upon power-on.
	** See default CalculateMemoryID() and CalculateFullMemoryID() functions in AlPlcRuntimeCore.c
	*/
	#define PLC_F_CALCULATE_EXEMEMOID	uint16_t UserCalculateMemoryID()					/* Function to calculate Exe Memo ID with a user specified algorithm */
	#define PLC_F_CALCULATE_FULLMEMOID	uint16_t UserCalculateFullMemoryID()				/* Function to calculate Full Memo ID with a user specified algorithm */ 

	/*
	** Callbacks for source code saving/reading to/from permanent memory
	** Mandatory if PLC_S_SOURCE_CODE_AREA is active
	*/
	#define PLC_F_WRITESRC(dat,off,len)	bool_t FWrSrc(void* dat,uint32_t off,uint32_t len)	/* Function used to save source code into permanent memory */
	#define PLC_F_READSRC(dat,off,len)	bool_t FRdSrc(void* dat,uint32_t off,uint32_t len)	/* Function used to load source code from permanent memory */
	#define PLC_F_READRSC_INIT			bool_t RdSrcInit()									/* Function used to initialize permanent memory read */
	#define PLC_F_READSRC_END			void RdSrcEnd()										/* Function used to terminate permanent memory read */
	#define PLC_F_WRITESRC_INIT			bool_t WrSrcInit()									/* Function used to initialize permanent memory writing */
	#define PLC_F_WRITESRC_END			void WrSrcEnd()										/* Function used to terminate permanent memory writing */
	/*
	** Optional if PLC_S_SOURCE_CODE_AREA is active
	*/
	#define PLC_F_GET_SOURCE_HEADER				GetSourceAreaHeader()		/* Function to get header informations CIMG_SOURCE_HEAD of the source area when it is not directly accessible */
	#define PLC_F_CALCULATE_SOURCE_CHECKSUM		CalculateSourceChecksum()	/* Calculate checksum function */

	/*
	** The following are the target areas for storing source code
	** Mandatory if PLC_S_SOURCE_CODE and PLC_S_SOURCE_CODE_AREA are active
	*/
	#define PLC_SOURCE_AREA							m_sourceCodeArea			/* Source code dedicated area address */
	#define PLC_SOURCE_AREA_SIZE					sizeof(m_sourceCodeArea)	/* Source code area size */
	#define PLC_F_NOTIFY_SOURCE_CODE_SIZE( size )	void NotifySourceAreaSize( uint32_t downloadSize )
	#define PLC_F_INIT_DOWNL_SOURCE					void InitDwSource()			/* Callback for init of source download notification */

	/*
	**	Callback which is responsible to patch the PLC
	**	application with the debug code provided by
	**	LogicLab through the PLC run-time.
	*/
	#define PLC_F_APPLY_PATCH( address, length, code )	bool_t ApplyPatch( void* address, uint32_t length, void* code )

	/*
	** When PLC_CONF_SYNC_SWAP is defined, the hosting environment, which is
	** responsible to call ALPLCSWAP to actually swap the PLC application, is
	** notified after a successful download through the callback
	** PLC_F_WAIT_SWAP.
	**
	** This callback has to synchronize with the process/thread that calls
	** ALPLCSWAP, waiting for it to accomplish that task and then returning
	** control to the PLC run-time.
    */
	#define PLC_F_WAIT_SWAP				void WaitSwap( void )

	/*
	**	Local versions of C runtime standard functions (if required)
	*/
	#define PLC_F_STRLEN				strlen
	#define PLC_F_STRCMP				strcmp
	#define PLC_F_TOUPPER				toupper

	/*
	**
	**	The following part of the configuration is relative to the tables
	**	of tasks, data blocks and functions.
	**	In these tables are listed all elements shared with PLC applications
	**	These tables can be defined statically or  created dynamically
	**	(use PLC_CONST_CONF or PLC_RUNTIME_CONF configuration switch)
	**
	*/

	/*
	**	The following mandatory definitions are
	**	relative to the number of elements of the 
	**	tables of tasks, DBs and functions
	**	In a static runtime they tipically are constants
	**	in a dynamic runtime they tipically are variables
	*/
	#define PLC_NUM_TASKS		m_rtNumTasks		/*	Number of PLC tasks	*/
	#define PLC_NUM_DB			m_rtNumDataBlocks	/*	Number of data blocks	*/
	#define PLC_NUM_FUNCTS		m_rtNumFunctions	/*	Number of embedded functions	*/

	/*
	**	The folliwing mandatory definitions are the
	**	qualifiers used to declare the target 
	**	elements tables and used by runtime core
	**	to access the tables themselves (const/non const)
	*/
	#define	PLC_ATTR_PLCIEC_TASKS_PTR		PLC_ATTR_CONST_PTR( PLCIEC_TASKS )	/* Definition of task shared tables */
	#define	PLC_ATTR_PLCIEC_DBREC			PLC_ATTR_CONST( PLCIEC_DBREC ) 		/* Definition of datablock shared tables */
	#define	PLC_ATTR_PLCIEC_DBREC_PTR		PLC_ATTR_CONST_PTR( PLCIEC_DBREC ) 	/* Definition of datablock shared tables */
	#define	PLC_ATTR_PLCIEC_FCNREC			PLC_ATTR_CONST( PLCIEC_FCNREC )		/* Definition of functions shared tables */
	#define	PLC_ATTR_PLCIEC_FCNREC_PTR		PLC_ATTR_CONST_PTR( PLCIEC_FCNREC )	/* Definition of functions shared tables */
	#define	PLC_ATTR_PLCIEC_TASKDEF			PLC_ATTR_CONST( PLCIEC_TASKDEF )	/* Definition of task definitions shared tables */

	/*
	**	Table of task definition used by runtime core
	**	List here the PLC task list with relative firmware entry points
	*/

	PLC_ATTR_CONST( PLCIEC_TASKS ) plcTasksFunctions[ PLC_NUM_TASKS ] =
	{
	/*  Fields description
		------------------
		taskId:		task ID for IEC compiler
		fnzExe:		address of PLC exec function
		fnzInit:	address of PLC init task variables function
		fnzInput:	address of input image function
		fnzOutput:	address of output image function
		rqInit		address of the flag for init request
		prgName		not used
		lineExe		not used

			taskId	fnzExe				fnzInit					 	fnzInput					fnzOutput					rqinit			prgName		lineExe		*/
		{ 	0, 		(void **)&plc_bck, 	(void **)&plc_bck_init, 	(void **)&plc_bck_input, 	(void **)&plc_bck_output, 	&rq_bck_init, 	0, 			0 },
	};


	/*
	**	Table of task definition used by LogicLab compiler
	**	List here the PLC task list with relative charateristics
	*/

	PLC_ATTR_CONST( PLCIEC_TASKDEF ) plcTasksDefs[ PLC_NUM_TASKS ] =
	{
	/*  Fields description
		------------------
		taskId:		Task ID (the same used in LogicLab resources declaration
		name:		Task name
		flags:		Task characteristics (no image, normal image, multi-phase image)
		usExecTime:	execution time in microseconds
		spare:		reserved for future uses

			taskId	name		flags				usExecTime	spare */
		{	0,      "Bckg",     TRGT_HASIMAGE,      1000,       0 },  
	};


	/*
	**	Data blocks table
	**	Here are listed all firmware variables shared with PLC application
	*/
	PLC_ATTR_CONST( PLCIEC_DBREC ) plcDataBlocks[ PLC_NUM_DB ] = 
	{
	/*  Fields description
		------------------
		img: 		if >= 1 the compiler generates an image of the target variable and two (in/out) 
					procedures to manage the copy to/from the image variable and the target variable
 					if > 1 then the procedures take a parameter that must be equals to this field 
 					for copying the variable
		type:		is the IEC 1131 data block type Input/Output/Memory i.e. the I/Q/M
					of IEC language (ex.: the I of %IW10.4)
		dbId:		is the datablock index i.e. the first number of the IEC declaration
					(ex.: the 10 of %IW10.4)
		addr:		is the datablock base address
		nel:		is the number of elements of the datablock i.e. the last index + 1 of the datablock
		dataSize:	is the size of the element fo the datablock, the physical address is 
					phy = addr + dataSize * idx where idx is the second number of the IEC declaration
					(the 4 of %IW10.4)
		rw:			is the access allowed to the datablock elements (read only, write only or read/write)

	img     type		dbId	addr    				nel			dataSize      			rw */
	{ 1,	DBTY_INP,	2,		(uint32_t)inputData,	10,			sizeof(inputData[0]),	DBRW_R 	},			
	{ 1,	DBTY_INP,	2,		(uint32_t)outputData,	10,			sizeof(outputData[0]),	DBRW_W 	},			
	{ 0,    DBTY_MEMO,	4,		(uint32_t)sharedVars,	100,		sizeof(sharedVars[0]),	DBRW_RW	},
	};


	/*
	**	C-exported functions table
	**	All these funactions are available to PLC programmer in the
	**	form of IEC FUNCTION or FUNCTION_BLOCK
	*/
	PLC_ATTR_CONST( PLCIEC_FCNREC ) plcEmbeddedFunctions[ PLC_NUM_FUNCTS ] = 
	{
	/*  Fields description
		------------------
		fname:		name of the function, the same used in the LogicLab environment
		faddress:	address of the function

		fname			faddress	*/
	{	"strncpy",		(uint32_t)strncpy,		},
	{	"pid",			(uint32_t)pid_funct,	},
	{	"strncat",		(uint32_t)strncat,		},
	};

	/*
	**	Plugin definitions
	**	Complete definitions for desired plugins
	*/

	/*
	**	AlPlcReal plugin - System plugin for real management
	*/

	#if ( PLC_S_REAL_PLUGIN == 1 )

	/***** AlPlcReal plugin configuration *****/
	/*	It is here provided the implementation for a system running only one runtime. */
	/*	AlPlcReal.c must be excluded from build */

	/***** Required definitions *****/

	/***** Optional definitions *****/

	#endif

	/*
	**	AlPlcString plugin - System plugin for string management
	*/	

	#if ( PLC_S_STRING_PLUGIN == 1 )

	/***** AlPlcString plugin configuration *****/
	/*	It is here provided the implementation for a system running only one runtime. */
	/*	AlPlcString.c must be excluded from build */

	/***** Required definitions *****/
	/*	PLC_S_STRING_PLUGIN requires ALPLCSTRGETBUFFER(b,s) definition */
	/*	ex: #define ALPLCSTRGETBUFFER(b,s) { b = (uint32_t) &buffer[0]; s = (uint32_t) sizeof(buffer); } */
	/*	Please define ALPLCSTRGETBUFFER(b,s) here */
	/*
	#define ALPLCSTRGETBUFFER(b,s)
	*/

	/***** Optional definitions *****/
	/*	If this compiler environment is supported by AlPlcString plugin following definitions are already provided by the plugin itself. */
	/*	If this compiler environment is not supported by AlPlcString plugin following definitions are implemented with standard c calls to <string.h> library */
	/*	Even if this compiler environment is supported you can redefine here these definitions using your specific library */
	/*
	#include <string.h>
	#define ALPLCSTRRESET(s,n)			memset( (void *) s, 0, n )
	#define ALPLCSTRCOPY(d,s,n)			strncpy( (char_t *) d, (const char_t *) s, n )
	#define ALPLCSTRCMP(r,a,b)			r = strcmp( (const char_t *) a, (const char_t *) b )
	#define ALPLCSTRLEN(n,s)			n = strlen( (const char_t *) s )
	#define ALPLCSTRCONCAT(d,c,n)		d = (uint32_t) ( strncat( (char_t *) d, (const char_t *) c, n ) )
	#define ALPLCSTRGETCHARAT(c,b,i)	c = (char_t) *( ( (char_t *) b ) + i )
	#define ALPLCSTRSETCHARAT(c,b,i)	*( ( (char_t *) b ) + i ) = c
	*/

	#endif

	/*
	**	AlPlcMath plugin - System plugin for math management
	*/

	#if ( PLC_S_MATH_PLUGIN == 1 )

	/***** AlPlcMath plugin configuration *****/
	/*	It is here provided the implementation for a system running only one runtime. */
	/*	AlPlcMath.c must be excluded from build */

	/***** Required definitions *****/

	/***** Optional definitions *****/
	/*	If this compiler environment is supported by AlPlcMath plugin following definitions are already provided by the plugin itself. */
	/*	If this compiler environment is not supported by AlPlcMath plugin following definitions are implemented with standard c calls to <math.h> library */
	/*	Even if this compiler environment is supported you can redefine here these definitions using your specific library */
	/*
	#include <math.h>
	#define ALPLCMATH_FABS(r,v)			r = (float32_t) ( fabs( (float64_t) v ) )
	#define ALPLCMATH_SQRT(r,v)			r = (float32_t) ( sqrt( (float64_t) v ) )
	#define ALPLCMATH_LN(r,v)			r = (float32_t) ( log( (float64_t) v ) )
	#define ALPLCMATH_LOG(r,v)			r = (float32_t) ( log10( (float64_t) v ) )
	#define ALPLCMATH_EXP(r,v)			r = (float32_t) ( exp( (float64_t) v ) )
	#define ALPLCMATH_POW(r,b,e)		r = (float32_t) ( pow( (float64_t) b, (float64_t) e ) )
	#define ALPLCMATH_SIN(r,v)			r = (float32_t) ( sin( (float64_t) v ) )
	#define ALPLCMATH_COS(r,v)			r = (float32_t) ( cos( (float64_t) v ) )
	#define ALPLCMATH_TAN(r,v)			r = (float32_t) ( tan( (float64_t) v ) )
	#define ALPLCMATH_ASIN(r,v)			r = (float32_t) ( asin( (float64_t) v ) )
	#define ALPLCMATH_ACOS(r,v)			r = (float32_t) ( acos( (float64_t) v ) )
	#define ALPLCMATH_ATAN(r,v)			r = (float32_t) ( atan( (float64_t) v ) )
	#define ALPLCMATH_ATAN2(r,v1,v2)	r = (float32_t) ( atan2( (float64_t) v1 ), (float64_t) v2 ) )
	#define ALPLCMATH_SINH(r,v)			r = (float32_t) ( sinh( (float64_t) v ) )
	#define ALPLCMATH_COSH(r,v)			r = (float32_t) ( cosh( (float64_t) v ) )
	#define ALPLCMATH_TANH(r,v)			r = (float32_t) ( tanh( (float64_t) v ) )
	#define ALPLCMATH_CEIL(r,v)			r = (float32_t) ( ceil( (float64_t) v ) )
	#define ALPLCMATH_FLOOR(r,v)		r = (float32_t) ( floor( (float64_t) v ) )
	*/

	#endif

	/*
	**	Other plugins should be here described
	*/

	/*
	**	The runtime core has to be included at the end
	**	of the runtime configuration file
	*/
	#include "AlPlcRuntimeCore.c"

	/*
	**	Include plugins code
	*/

	#if ( PLC_S_REAL_PLUGIN == 1 )
	#include "AlPlcRuntime/AlPlcReal.c"
	#endif
	#if ( PLC_S_STRING_PLUGIN == 1 )
	#include "AlPlcRuntime/AlPlcString.c"
	#endif
	#if ( PLC_S_MATH_PLUGIN == 1 )
	#include "AlPlcRuntime/AlPlcMath.c"
	#endif

#endif	//	 DESCRIPTION_OF_RUNTIME_CONFIGURATION

/*----------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------*/

/*											*/
/*	Check for proper entry point definition	*/
/*											*/

#if ! defined( ALPLCINIT )
#error "Missing runtime entry point definition of ALPLCINIT"
#endif
#if ! defined( ALPLCLOAD )
#error "Missing runtime entry point definition of ALPLCLOAD"
#endif
#if ! defined( ALPLCMANAGE )
#error "Missing runtime entry point definition of ALPLCMANAGE"
#endif
#if defined( PLC_CONF_SYNC_SWAP ) && ! defined( ALPLCSWAP )
#error "Since PLC_CONF_SYNC_SWAP feature is enabled, ALPLCSWAP entry point must be defined"
#endif

/*
**	Check for tables sizes definition
*/

#if ! defined( PLC_NUM_TASKS )
#error "Missing table size definition PLC_NUM_TASKS"
#endif
#if ! defined( PLC_NUM_DB )
#error "Missing table size definition PLC_NUM_DB"
#endif
#if ! defined( PLC_NUM_FUNCTS )
#error "Missing table size definition PLC_NUM_FUNCTS"
#endif

/*
**	Check for table qualifiers
*/
#if ! defined( PLC_ATTR_PLCIEC_TASKS_PTR )
#error "Missing table qualifier definition PLC_ATTR_PLCIEC_TASKS_PTR"
#endif
#if ! defined( PLC_ATTR_PLCIEC_DBREC_PTR )
#error "Missing table qualifier definition PLC_ATTR_PLCIEC_DBREC_PTR"
#endif
#if ! defined( PLC_ATTR_PLCIEC_FCNREC_PTR )
#error "Missing table qualifier definition PLC_ATTR_PLCIEC_FCNREC_PTR"
#endif

/*														*/
/*	Check for proper configuration switches definition	*/
/*														*/
#if ! defined(PLC_S_FLOATSUPPORT)
#define PLC_S_FLOATSUPPORT		0
#endif
#if	! defined(PLC_S_HOTSWAP)
#define PLC_S_HOTSWAP			0
#endif
#if	! defined(PLC_S_NODOWNLAREA)
#define PLC_S_NODOWNLAREA		0
#endif
#if	! defined(PLC_S_DEBUGAREA)
#define PLC_S_DEBUGAREA			0
#endif
#if	! defined(PLC_S_CHECKSUM)
#define PLC_S_CHECKSUM			0
#endif
#if	! defined(PLC_S_FIXEDTASKS)
#define PLC_S_FIXEDTASKS		0
#endif
#if	! defined(PLC_S_INFOHEADER)
#define PLC_S_INFOHEADER		0
#endif
#if	! defined(PLC_S_INITHANDLES)
#define PLC_S_INITHANDLES		0
#endif
#if	! defined(PLC_S_CHECKARRAYS)
#define PLC_S_CHECKARRAYS		0
#endif
#if	! defined(PLC_S_CHECKSTACK)
#define PLC_S_CHECKSTACK		0
#endif
#if	! defined(PLC_S_CHECKSTACKCALL)
#define PLC_S_CHECKSTACKCALL		0
#endif
#if	! defined(PLC_S_TRACEAREA)
#define PLC_S_TRACEAREA			0
#endif
#if	! defined(PLC_S_SYNCPATCH)
#define PLC_S_SYNCPATCH			0
#endif
#if	! defined(PLC_S_AUXDATAAREA)
#define PLC_S_AUXDATAAREA		0
#endif
#if	! defined(PLC_S_TRACECALLS)
#define PLC_S_TRACECALLS		0
#endif
#if	! defined(PLC_S_CHECKPOINTERS)
#define PLC_S_CHECKPOINTERS		0
#endif
#if	! defined(PLC_S_BREAKPTS)
#define PLC_S_BREAKPTS			0
#endif
#if	! defined(PLC_S_FORCEIO)
#define PLC_S_FORCEIO			0
#endif
#if	! defined(PLC_S_TIME_PLUGIN)
#define PLC_S_TIME_PLUGIN		0
#endif
#if	! defined(PLC_S_REAL_PLUGIN)
#define PLC_S_REAL_PLUGIN		0
#endif
#if	! defined(PLC_S_STRING_PLUGIN)
#define PLC_S_STRING_PLUGIN		0
#endif
#if	! defined(PLC_S_MATH_PLUGIN)
#define PLC_S_MATH_PLUGIN		0
#endif
#if	! defined(PLC_S_DYN_LINK_SUPPORT)
#define PLC_S_DYN_LINK_SUPPORT		0
#endif
#if	! defined(PLC_S_DBACC_PATCH)
#define PLC_S_DBACC_PATCH		0
#endif
#if	! defined(PLC_S_DBACC_INDIR)
#define PLC_S_DBACC_INDIR		0
#endif
#if	! defined(PLC_S_FNACC_PATCH)
#define PLC_S_FNACC_PATCH		0
#endif
#if	! defined(PLC_S_FNACC_INDIR)
#define PLC_S_FNACC_INDIR		0
#endif
#if	! defined(PLC_S_DATA_RELAT)
#define PLC_S_DATA_RELAT		0
#endif
#if	! defined(PLC_S_CODE_RELAT)
#define PLC_S_CODE_RELAT		0
#endif
#if	! defined(PLC_S_TASK_SETTINGS)
#define PLC_S_TASK_SETTINGS		0
#endif
#if	! defined(PLC_S_RUNTIME_CTRL)
#define PLC_S_RUNTIME_CTRL		0
#endif
#if	! defined(PLC_S_SOURCE_CODE)
#define PLC_S_SOURCE_CODE		0
#endif
#if	! defined(PLC_S_SOURCE_CODE_AREA)
#define PLC_S_SOURCE_CODE_AREA		0
#endif
#if ! defined(PLCIEC_DBREC_CONST)
#define PLCIEC_DBREC_CONST		0
#endif
#if ! defined(PLCIEC_FCNREC_CONST)
#define PLCIEC_FCNREC_CONST		0
#endif

/*								*/
/*	Runtime configuration check	*/
/*								*/
#if defined( PLC_CONST_CONF ) && defined( PLC_RUNTIME_CONF )
#error "Incorrect dynamic/constant runtime configuration: only one between PLC_CONST_CONF and PLC_RUNTIME_CONF should be used"
#endif

#if ! defined( PLC_CONST_CONF ) && ! defined( PLC_RUNTIME_CONF )
#error "Missing dynamic/constant runtime configuration: select between one of PLC_CONST_CONF and PLC_RUNTIME_CONF"
#endif

/*									*/
/*	Target features switches check	*/
/*									*/
#if ( PLC_S_SOURCE_CODE == 1 ) && ( PLC_S_SOURCE_CODE_AREA == 1 )
#error "Incorrect source code management switches: only one between PLC_S_SOURCE_CODE and PLC_S_SOURCE_CODE_AREA switches should be active"
#endif

/*											*/
/*	Callbacks and target definitions check	*/
/*											*/

#if ( PLC_CONF_ALCAP_V1 == 1 ) && ! defined( PLC_R_COMMAND_RESPONSE )
#error "Current configuration requires the definition of PLC_R_COMMAND_RESPONSE"
#endif

#if ( PLC_S_INFOHEADER == 1 ) && ! ( defined( PLC_V_APPVER_NAME ) && defined( PLC_V_APPVER_MAJOR ) && defined( PLC_V_APPVER_MINOR ))
#error "PLC_S_INFOHEADER is switched on, thus the following elements must be defined: PLC_V_APPVER_NAME, PLC_V_APPVER_MAJOR, and PLC_V_APPVER_MINOR"
#endif

#if ( PLC_CONF_RELOCATABLE == 1 ) && ! defined( ALPLCDYNADDRS )
#error "At least one of the following configuration switches is on: PLC_S_CODE_RELAT, PLC_S_DATA_RELAT, PLC_S_DBACC_INDIR, PLC_S_FNACC_INDIR thus ALPLCDYNADDRS must be defined"
#endif

#if ( PLC_S_DBACC_INDIR == 1 ) && ( PLCIEC_DBREC_CONST == 1 )
#error "Incompatible definition of PLC_S_DBACC_INDIR and PLCIEC_DBREC_CONST: PLC_S_DBACC_INDIR is switched on so PLCIEC_DBREC_CONST must be 0"
#endif

#if ( PLC_S_FNACC_INDIR == 1 ) && ( PLCIEC_FCNREC_CONST == 1 )
#error "Incompatible definition of PLC_S_FNACC_INDIR and PLCIEC_DBREC_CONSTPLC_S_FNACC_INDIR is switched on so PLCIEC_FCNREC_CONST must be 0"
#endif

#if ( PLC_S_SOURCE_CODE_AREA == 0 ) && defined( PLC_SOURCE_FILE_SAVE_CONF )
#error "PLC_SOURCE_FILE_SAVE_CONF requires PLC_S_SOURCE_CODE_AREA feature"
#endif

#if ( PLC_S_STRING_PLUGIN == 1 ) && ! defined( ALPLCSTRGETBUFFER )
#error "PLC_S_STRING_PLUGIN requires ALPLCSTRGETBUFFER(b,s) definition ( ex: #define ALPLCSTRGETBUFFER(b,s) { b = (uint32_t) &buffer[0]; s = (uint32_t) sizeof(buffer); } )"
#endif

#if defined(PLC_CODE_FILE_SAVE_CONF) && ! (defined(PLC_F_WRITE) && defined(PLC_F_READ) && defined(PLC_F_READ_INIT) && defined(PLC_F_READ_END) && defined(PLC_F_WRITE_INIT) && defined(PLC_F_WRITE_END))
#error "Incomplete callbacks definitions for PLC_CODE_FILE_SAVE_CONF feature. All the following callbacks are to be defined: PLC_F_WRITE, PLC_F_READ, PLC_F_READ_INIT, PLC_F_READ_END, PLC_F_WRITE_INIT, PLC_F_WRITE_END"
#endif

#if ! defined( PLC_DEBUGDATA_SIZE )
#define PLC_DEBUGDATA_SIZE	0
#endif	/* ! defined( PLC_DEBUGDATA_SIZE ) */

#if PLC_S_DEBUGAREA == 1
#if ! defined( PLC_DEBUGDATA_AREA )
#error "PLC_S_DEBUGAREA is switched on, but no debug area has been provided: PLC_DEBUGDATA_AREA has to be defined"
#endif	/* ! defined( PLC_DEBUGDATA_AREA ) */
#if ! ( PLC_DEBUGDATA_SIZE > 0 )
#error "PLC_S_DEBUGAREA is switched on, but the size of the debug area is invalid: please, set PLC_DEBUGDATA_SIZE to a positive value"
#endif	/* ! ( PLC_DEBUGDATA_SIZE > 0 ) */
#endif	/* PLC_S_DEBUGAREA == 1 */

#if ! defined( PLC_TRACEDATA_SIZE )
#define PLC_TRACEDATA_SIZE	0
#endif	/* ! defined( PLC_TRACEDATA_SIZE ) */

#if PLC_S_TRACEAREA == 1
#if ! defined( PLC_TRACEDATA_AREA )
#error "PLC_S_TRACEAREA is switched on, but no trace area has been provided: PLC_TRACEDATA_AREA has to be defined"
#endif	/* ! defined( PLC_TRACEDATA_AREA ) */
#if ! ( PLC_TRACEDATA_SIZE > 0 )
#error "PLC_S_TRACEAREA is switched on, but the size of the trace area is invalid: please, set PLC_TRACEDATA_SIZE to a positive value"
#endif	/* ! ( PLC_TRACEDATA_SIZE > 0 ) */
#endif	/* PLC_S_TRACEAREA == 1 */

#if PLC_S_SYNCPATCH == 1
#if PLC_S_DEBUGAREA == 0
#error "PLC_S_SYNCPATCH is switched on: thus, PLC_S_DEBUGAREA has to be switched on, too"
#endif	/* PLC_S_DEBUGAREA == 0 */
#if ! defined( PLC_F_APPLY_PATCH )
#error "PLC_S_SYNCPATCH is switched on: PLC_F_APPLY_PATCH has to be defined"
#endif	/* ! defined( PLC_F_APPLY_PATCH )*/
#endif	/* PLC_S_SYNCPATCH == 1 */

#if ( PLC_S_BREAKPTS == 1 )
#if ! ( defined( PLC_F_GET_PLC_BREAKPOINT_ADDRESS ) && defined( PLC_V_HIT_BREAKPOINT_MASK ) && defined( PLC_V_RUN_BREAKPOINT_MASK ) )
#error "PLC_S_BREAKPTS is switched on, thus the following elements must be defined: PLC_F_GET_PLC_BREAKPOINT_ADDRESS, PLC_V_HIT_BREAKPOINT_MASK, and PLC_V_RUN_BREAKPOINT_MASK"
#endif
#endif	/* ( PLC_S_BREAKPTS == 1 ) */

#if ( PLC_S_TASK_SETTINGS == 1 ) && ! defined( PLC_F_UPDATE_TASK_PERIOD )
#error "PLC_F_UPDATE_TASK_PERIOD must be defined if PLC_S_TASK_SETTINGS switch is on"
#endif

#if defined( PLC_CONF_SYNC_SWAP ) && ! defined( PLC_F_WAIT_SWAP )
#error "Since PLC_CONF_SYNC_SWAP feature is enabled, PLC_F_WAIT_SWAP callback must be defined"
#endif

