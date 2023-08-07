/*-----------------------------------------------------------------------------------------
------
------	Description
------	
------	coeappl.h
------
------	CANopen over EtherCAT application example
------	
------
-----------------------------------------------------------------------------------------*/

#ifndef _COEAPPL_H_
#define _COEAPPL_H_

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/

#include	"sdoserv.h"

/*-----------------------------------------------------------------------------------------
------	
------	Defines and Types
------	
-----------------------------------------------------------------------------------------*/

#ifndef _COEAPPL_
#define _COEAPPL_   0
#endif

#if _COEAPPL_
	#define PROTO
#else 
	#define PROTO extern
#endif

/*-----------------------------------------------------------------------------------------
------	
------	Global variables
------	
-----------------------------------------------------------------------------------------*/
 
/*-----------------------------------------------------------------------------------------
------  
------	Global functions
------	
-----------------------------------------------------------------------------------------*/

PROTO	void COE_ResetOutputs(void);
PROTO void COE_ReadInputs(void);
PROTO	void COE_ObjInit(void);
PROTO	void COE_GenerateMapping(void);
PROTO	void COE_OutputMapping(UINT16 *pOutputs);
PROTO void COE_InputMapping(UINT16 *pInputs);
PROTO	void COE_Application(void);
/* ECATCHANGE_START(V4.00) */
PROTO	void COE_Main(void);
/* ECATCHANGE_END(V4.00) */

#undef PROTO

#endif //_COEAPPL_H_

