/*------------------------------------------------------------------------------
**
**	Copyright:	AXEL s.r.l. 2009
**
**	PLCIECRUNTIME.H:	Runtime driver header for LogicLab
**
**-----------------------------------------------------------------------------
**
**	IMPORTANT:
**	THIS MODULE SHOULDN'T BE MODIFIED BY THE CUSTOMER
**
**-----------------------------------------------------------------------------*/


#ifndef _PLCIECRUN_H
#define _PLCIECRUN_H

#ifdef __cplusplus
extern "C" {
#endif

/*	C compiler definitions	*/
#include "AlPlcCDefs.h"
	
/*-----------------------------------------------------------------------------
**	Runtime structures
**----------------------------------------------------------------------------*/

/*	Struct of IEC tasks table */
typedef struct _PLCIEC_TASKS
{
	uint32_t				taskId;		/*	Task ID for IEC compiler */
	PLC_TASK_PTR				fnzExe;		/*  Address of exec function */
	PLC_TASK_PTR				fnzInit;	/*  Address of init var function */
	PLC_TASK_PTR				fnzInput;	/*  Address of init var function */
	PLC_TASK_PTR				fnzOutput;	/*  Address of init var function */
	bool_t *				rqInit;		/*  Flag of request Init function */
	char_t **				prgName;	/*  Program name */
	uint32_t **				lineExe;	/*  Address of execution line number */
	uint32_t * 				exePeriod;
	uint32_t * 				exeTime;
	uint32_t * 				exeCount;
	bool_t * 				exeStatus;
    bool_t *                taskValid;  /*  task exist */
}
PLCIEC_TASKS;

#ifdef __cplusplus
}
#endif

#endif  /* _PLCIECRUN_H */

