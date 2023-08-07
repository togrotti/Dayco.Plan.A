/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2012, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : AlPlcAreaDef.c                                             */
/* Author      : Fabio Terrile                                              */
/*               Axel                                                       */
/*                                                                          */
/* Description : PLC configuration and instancing                           */
/*                                                                          */
/****************************************************************************/

/*
** The following are the target feature supported. All switches should be defined.
** Put 0 or 1 to enable or disable the feature
*/
#define PLC_S_FLOATSUPPORT			1				/*	Floating point support on device */
#define PLC_S_HOTSWAP				0				/*	Load PLC without stop the tasks */
#define PLC_S_NODOWNLAREA			1				/*	Device without separate download area */
#define PLC_S_DEBUGAREA				0				/*	Device with dedicated debug area */
#define PLC_S_CHECKSUM				1				/*	Code checksum support */
#define PLC_S_FIXEDTASKS			0				/*	Tasks fixed and always exist */
#define PLC_S_INFOHEADER			1				/*	Info about PLC application */
#define PLC_S_INITHANDLES			0				/*	Init handles of associate functions */
#define PLC_S_CHECKARRAYS			0				/*	Function of runtime check array */
#define PLC_S_CHECKSTACK			0				/*	Function of runtime check stack */
#define PLC_S_CHECKSTACKCALL		0				/*	Function of runtime check stack inside functions */
#define PLC_S_TRACEAREA				0				/*	Device has trace area */
#define	PLC_S_SYNCPATCH				0				/*	Sync patch function ( Synchro Trigger ) */
#define	PLC_S_AUXDATAAREA			0				/*	Auxiliary data area */
#define	PLC_S_TRACECALLS			0				/*	Trace of FB call and FUNCTION call */
#define	PLC_S_CHECKPOINTERS			0				/*	Function for runtime pointers check */
#define	PLC_S_BREAKPTS				0				/*	Device supports breakpoints */
#define	PLC_S_FORCEIO				0				/*	Function of forcing I/O */
#define	PLC_S_TIME_PLUGIN			0				/*	Device has internal system time (sysTimer) */
#define	PLC_S_REAL_PLUGIN			0				/*	Device supports real operations using AlPlcReal plugin */
#define	PLC_S_STRING_PLUGIN			0				/*	Device supports string operations using AlPlcString plugin */
#define	PLC_S_MATH_PLUGIN			0				/*	Device supports math operations using AlPlcMath plugin */
#define PLC_S_DYN_LINK_SUPPORT		0				/*	Device has dynamic links */
#define	PLC_S_DBACC_PATCH			0				/*	Runtime patch for DATA BLOCK access */
#define	PLC_S_DBACC_INDIR			0				/*	Use DATA BLOCKS with indirect address */
#define	PLC_S_FNACC_PATCH			0				/*	Runtime patch for EMBEDDED FUNCTIONS */
#define	PLC_S_FNACC_INDIR			0				/*	Use EMBEDDED FUNCTIONS with indirect address */
#define	PLC_S_DATA_RELAT			0				/*	Relocable Data area */
#define	PLC_S_CODE_RELAT			0				/*	Relocable Code area */
#define PLC_S_TASK_SETTINGS			0				/*	Dynamic task settings support */
#define PLC_S_RUNTIME_CTRL			0				/*	PLC run-time execution control from LogicLab */
#define PLC_S_SOURCE_CODE			0				/*	Target supports the storage of the source code embedded in the binary code */
#define PLC_S_SOURCE_CODE_AREA		0				/*	Target supports the storage of the source code in a dedicated area */

/*	Standard definitions	*/
#include <string.h>
#include "AlPlcCDefs.h"
#include "AlPlcTarg.h"
#include "AlPlcRuntimeCore.h"
#include "common\CommonTypedef.h"
#include "core\SystemReset.h"

/*	User files */
#include "plc\PlcRT.h"
#include "AlPlcAreaDef.h"
#include "AlPlcUserData.h"
#include "system\SysAppSFlashPartition.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

/*  PCT prefix definition */
//#define ALPLC_TARGET_ID		"AXMK_1p0"

/*  TGT resource definition */
#define ALPLC_TARGET_COMM	"AxXRT3"

/*
** Type of configuration:
** 	- Static (const/ROM)
** 	- Dynamic (RAM, runtime built)
** Only one of these two switches should be defined:
*/
#define PLC_CONST_CONF
//#define PLC_RUNTIME_CONF

/*
** Runtime core callbacks
*/
#define ALPLCINIT                   InitPlcDiagRuntime      /*  Init oft the runtime, prototype: bool_t ALPLCINIT(void)   */
#define ALPLCLOAD                   LoadPlcDiag             /*  Load code into exec memory, prototype: bool_t ALPLCLOAD(void)   */
#define ALPLCMANAGE                 ManagePLCDiagState      /*  Manage of runtime state, prototype: void ALPLCMANAGE(void)  */

/*
** The following switch enables the saving of the PLC code file and PLC source file
*/
//#define PLC_CODE_FILE_SAVE_CONF
//#define PLC_SOURCE_FILE_SAVE_CONF

/*
** The following are the target elements definitions (tasks, datablocks, functions)
*/
#define PLC_TASK_TAB				plcTasksFunctions		/*	Pointer to task table (type PLCIEC_TASKS *) */
#define PLC_TASK_DEFS				plcTasksDefs			/*	Pointer to task table (type PLCIEC_TASKDEF *) */
#define PLC_DB_TAB					plcDataBlocks			/*	Pointer to datablocks table (type PLCIEC_DBREC *) */
#define PLC_FUNCT_TAB				plcEmbeddedFunctions	/*	Pointer to functions table (type PLCIEC_FCNREC *) */

/*
** The following are the target areas for storing executable code, data, retain data and bit addressable data
** The code and data areas definitions are mandatory.
** Retain and bit-addressable areas are optional and should be defined only if target supports them
*/

#define PLC_CODE_AREA1              PLCD_RT_CODE_START      /*  Required - Address of main or first hot swap code area */
//#define PLC_CODE_AREA2            0                       /*  Address of second hot swap code area*/
#define PLC_DATA_AREA               sPlcAutoVars            /*  Required - Address of PLC data area */
#define PLC_BITDATA_AREA            sPlcAutoBits            /*  Address of bit data area (if supported by processor) */
//#define PLC_RETDATA_AREA          0                       /*  Address of PLC retain data area */

#define PLC_CODE_SIZE               PLCD_RT_CODE_SIZE       /*  Required - Size of code area(s) */
#define PLC_DATA_SIZE               PLC_AUTO_VARS_SIZE      /*  Required - Size of data area */
//#define PLC_RETDATA_SIZE          0                       /*  Size of PLC retain data area */

#define PLC_BITDATA_UNITS			( PLC_AUTO_BITS_SIZE / 16 ) /* number of bits */
#define PLC_BITDATA_SIZE		 	( PLC_AUTO_BITS_SIZE / 8  )	/*	Size of bit data area */

/*
** The following are the definions of debug and trace areas. These areas are optional
** and should be defined only if target supports these features (PLC_S_DEBUGAREA and/or PLC_S_TRACEAREA)
*/
//#define PLC_DEBUGDATA_AREA			m_rtPlcDebugAddr		/*	Address of debug data area */
//#define PLC_TRACEDATA_AREA			m_rtPlcTraceAddr		/*	Address of trace area */
//#define PLC_DEBUGDATA_SIZE			m_rtDebugSize 			/*	Size of debug data area */
//#define PLC_TRACEDATA_SIZE			m_rtTraceSize 			/*	Size of trace area */

/*
** More specific features
*/
#define PLC_STACK_LIMIT				0											/*	Limits for stack allocation */
//#define PLC_AUXDATA_AREA			NULL				                        /*	(optional) Address of auxiliary data area */
//#define PLC_AUXDATA_SIZE			0l		                                    /*	(optional) Size of auxiliary data area */
//#define PLC_SYSTIMER_ADDR			NULL										/*	(optional) Address of system timer */

/*
** Runtime data exchange with LogicLab by means of
** comunication objecs (as Modbus register, CANopen objects etc.)
*/
static uint16_t uwDummy = 0;
static uint32_t ulDummy = 0;
/*	in questa configurazione di runtime non vengono pubblicati registri per la comunicazione con LL */
/*	questi registri vengono definiti e associati a variabili dummy inizializzate a 0 */
#define PLC_R_COMMAND				uwDummy		/*	LogicLab request for target reset/reload of PLC */
#define PLC_R_DOWNLADDR				ulDummy		/*	Address for the next download */
#define PLC_R_INFOAREA				ulDummy		/*	Address of runtime info area */
#define PLC_R_DOWNL_RESP			uwDummy 	/*	Target response for a LogicLab request for download acknowledge */
#define PLC_R_APPL_ID				ulDummy		/*	Application ID for the currently running PLC */
#define PLC_R_MEMO_ID				ulDummy		/*	Firmware memory ID */
#define PLC_R_FULLMEMO_ID			ulDummy		/*	Firmware memory ID with db and function check */
#define PLC_R_COMMAND_RESPONSE		ulDummy		/*	Response to LogicLab requests */

/*
**	PLC control variables shared with firmware to allow
**	the firmware itself to control and manage PLC
**	execution states
*/
#define PLC_V_STATUS_OK				bPlcProgramOk			/*	Required - bool_t - valid PLC */
#define PLC_V_ERROR_CODE			uwPlcErrorCode  		/*	Required - uint16_t - load PLC error code */
//#define PLC_V_STATUS_ALARM						  			/*	Optional - bool_t - alarm status */
//#define PLC_V_RUNTIME_RUNNING_STATE							/*	Optional - bool_t - host system defines an independent run-time running state (with respect to PLC application status) */
#define PLC_V_FILESIZE              ulPlcFileSize           /*  uint32_t - plc file size */
#define PLC_V_MPLCINFO              sPlcDiagInfo            /*  TARGETADDRS_EX - plc info area public defined */

/*
**	PLC application version info
**	Here ar mapped target variables that hold application version info
*/
#define PLC_V_APPVER_NAME			sPlcInfoHeader.strApp	/*	Required if PLC_S_INFOHEADER is on - char_t* - Application name */
#define PLC_V_APPVER_MAJOR		    sPlcInfoHeader.appVer1	/*	Required if PLC_S_INFOHEADER is on - uint16_t - Major version number */
#define PLC_V_APPVER_MINOR	 	    sPlcInfoHeader.appVer2	/*	Required if PLC_S_INFOHEADER is on - uint16_t - Minor version number */
#define PLC_V_APPVER_BUILD		    sPlcInfoHeader.buildTime /*	Optional - uint32_t - Build time (in seconds since 00:00:00 UTC on 1 January 1970) */

/*
**	IEC runtime callbacks used to manage target
**	specific functions (disable interrupts, stop PLC execution, memory copy etc.)
**	(leave commented if not used/required)
*/

#define PLC_F_ZERODATA_AREA				memset(PLC_DATA_AREA, 0, PLC_DATA_SIZE)	/*	Function call for reset data area: void funct( uint8_t address, uint32_t size_in_bytes ) */
//#define PLC_F_ZERODATABIT_AREA		void zeromem(uint8_t * addr, uint32_t mem)	/*	Function call for reset data bit area: void funct( uint8_t address, uint32_t size_in_bytes ) */
//#define PLC_F_ZERODATARET_AREA		void zeromem(uint8_t * addr, uint32_t mem)	/*	Function call for reset data retain area: void funct( uint8_t address, uint32_t size_in_bytes ) */
//#define PLC_F_ZEROAUXDATA_AREA			memset(PLC_AUXDATA_AREA, 0, PLC_AUXDATA_SIZE)	/*	Function call for reset data area: void funct( uint8_t address, uint32_t size_in_bytes ) */

/*
**	Runtime notifications callbacks
*/
//#define PLC_F_INIT_DOWNLOAD		StopHmiTasks()						/*	Function call preparing the target for a new download */
//#define PLC_F_STOP_TASKS			StopHmiTasks()						/*	Function call for stop PLC execution */
//#define PLC_F_START_TASKS			StartHmiTasks()						/*	Function call for starting PLC execution */
//#define PLC_F_SYSLOCK_REQ(t)		PlcStandByRequired()	            /*	Reqest for system lock required if task T data is to reset */
//#define PLC_F_INTERRUPTDISABLE	void disableinterrupt(void)			/*	Disable interrupt function */
//#define PLC_F_INTERRUPTENABLE		void enableinterrupt(void)			/*	Enable interrupt function */
//#define PLC_F_END_LOAD(s)			PlcEndOfLoad()		                /*	End PLC load notification */
//#define PLC_F_INIT_LOAD				RestartSystem()

/*
** 	Function used to change the task period according to the application settings
*/
//#define PLC_F_TASK_CONFIGURATION_INIT			bool_t InitTaskConfiguration()
//#define PLC_F_UPDATE_TASK_PERIOD(task,period)	bool_t UpdateTaskPeriod( uint16_t task, uint32_t period )
//#define PLC_F_TASK_CONFIGURATION_END			bool_t EndTaskConfiguration()

/*
** 	Call-backs invoked before and after the execution of a PLC run-time start/stop command
*/
//#define PLC_F_START_RUNTIME_INIT	bool_t InitStartPlcRuntime()	/* Invoked before the execution of a PLC run-time start command */
//#define PLC_F_START_RUNTIME_END		bool_t EndStartPlcRuntime()		/* Invoked after the execution of a PLC run-time start command */
//#define PLC_F_STOP_RUNTIME_INIT		bool_t InitStopPlcRuntime()		/* Invoked before the execution of a PLC run-time stop command */
//#define PLC_F_STOP_RUNTIME_END		bool_t EndStopPlcRuntime()		/* Invoked after the execution of a PLC run-time stop command */

/*
** 	Read the host system-defined independent run-time running state
**	(with respect to PLC application status) 
*/
//#define PLC_F_UPDATE_RUNTIME_RUNNING_STATE	bool_t UpdateRuntimeRunningState()

/*
** Callbacks for executable code saving/reading to/from permanent memory
** Mandatory if PLC_CODE_FILE_SAVE_CONF is active
*/
//#define PLC_F_WRITE(dat,off,len)		HmiFlashWrite( (dat), (off), (len))	/* Function used to save PLC code into permanent memory */
//#define PLC_F_READ(dat,off,len)		HmiFlashRead( (dat), (off), (len))	/* Function used to load PLC code from permanent memory */
//#define PLC_F_READ_INIT				HmiFlashReadInit()					/* Function used to initialize permanent memory read */
//#define PLC_F_READ_END				HmiFlashReadEnd()					/* Function used to terminate permanent memory read */
//#define PLC_F_WRITE_INIT				HmiFlashWriteInit()					/* Function used to initialize permanent memory writing */
//#define PLC_F_WRITE_END				HmiFlashWriteEnd()					/* Function used to terminate permanent memory writing */

/*
** Callbacks to calculate Exe Memo ID or Full Memo ID with an user-defined algorithm.
** User functions should be used only in special cases and the implementor has to assure that the computed values satisfy the same requirements the default implementations do.
** A simpler implementation would just assure that the computed values always change if the firmware changes: this implementation is somewhat more limitating, with special respect to successful PLC load upon power-on.
** See default CalculateMemoryID() and CalculateFullMemoryID() functions in AlPlcRuntimeCore.c
*/
#define PLC_F_CALCULATE_EXEMEMOID	PlcGetDiagMemoryID()					/* Function to calculate Exe Memo ID with a user specified algorithm */
#define PLC_F_CALCULATE_FULLMEMOID	PlcGetDiagFullMemoryID()				/* Function to calculate Full Memo ID with a user specified algorithm */ 

/*
** Callbacks for source code saving/reading to/from permanent memory
** Mandatory if PLC_S_SOURCE_CODE_AREA is active
*/
//#define PLC_F_WRITESRC(dat,off,len)	bool_t FWrSrc(void* dat,uint32_t off,uint32_t len)	/* Function used to save source code into permanent memory */
//#define PLC_F_READSRC(dat,off,len)	bool_t FRdSrc(void* dat,uint32_t off,uint32_t len)	/* Function used to load source code from permanent memory */
//#define PLC_F_READRSC_INIT			bool_t RdSrcInit()									/* Function used to initialize permanent memory read */
//#define PLC_F_READSRC_END				void RdSrcEnd()										/* Function used to terminate permanent memory read */
//#define PLC_F_WRITESRC_INIT			bool_t WrSrcInit()									/* Function used to initialize permanent memory writing */
//#define PLC_F_WRITESRC_END			void WrSrcEnd()										/* Function used to terminate permanent memory writing */

/*
** Optional if PLC_S_SOURCE_CODE_AREA is active
*/
//#define PLC_F_GET_SOURCE_HEADER				    PlcGetSourceCodeAreaHeader() /* Function to get header informations CIMG_SOURCE_HEAD of the source area when it is not directly accessible */
//#define PLC_F_CALCULATE_SOURCE_CHECKSUM		    PlcChecksumSourceCode()     /* Calculate checksum function */

/*
** The following are the target areas for storing source code
** Mandatory if PLC_S_SOURCE_CODE and PLC_S_SOURCE_CODE_AREA are active
*/
//#define PLC_SOURCE_AREA						SFPART_PLC_PRJ_START			/* Source code dedicated area address */
//#define PLC_SOURCE_AREA_SIZE					SFPART_PLC_PRJ_SIZE	            /* Source code area size */
//#define PLC_F_NOTIFY_SOURCE_CODE_SIZE( size )	void NotifySourceAreaSize( uint32_t downloadSize )
//#define PLC_F_INIT_DOWNL_SOURCE				PlcInitDwSourceCode()		    /* Callback for init of source download notification */

/*
**	Callback which is responsible to patch the PLC
**	application with the debug code provided by
**	LogicLab through the PLC run-time.
*/
//#define PLC_F_APPLY_PATCH( address, length, code )	bool_t ApplyPatch( void* address, uint32_t length, void* code )

/*
**	Local versions of C runtime standard functions (if required)
*/
#define PLC_F_STRLEN				strlen
#define PLC_F_STRCMP				strcmp
#define PLC_F_TOUPPER				toupper

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

// Avoids warning C183: dead assignment eliminated
// Avoids warning C174: unreferenced 'static' function
#pragma warning disable = 183
#pragma warning disable = 174

#include "AlPlcRuntimeCore.lib"

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
