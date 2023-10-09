#ifndef ALPLCAREADEF_H_
#define ALPLCAREADEF_H_

#include "AlPlcCDefs.h"
#include "AlPlcTarg.h"
#include "system\SysAppConfig.h"

/* Relocation of HCONST class in order to enable area reading via packetcommand */

//#pragma renameclass(HCONST=PLCCNST)

/*  Name of the public functions of the runtime core                */
bool_t InitPlcRuntime(void);                /*  Init oft the runtime, prototype: bool_t ALPLCINIT(void)   */
bool_t LoadPlc(void);                       /*  Load code into exec memory, prototype: bool_t ALPLCLOAD(void)   */
void ManagePLCState(void);                  /*  Manage of runtime state, prototype: void ALPLCMANAGE(void)  */

/*  Name of the public functions of the diagnostic runtime core     */
bool_t InitPlcDiagRuntime(void);            /*  Init oft the runtime, prototype: bool_t ALPLCINIT(void)   */
bool_t LoadPlcDiag(void);                   /*  Load code into exec memory, prototype: bool_t ALPLCLOAD(void)   */
void ManagePLCDiagState(void);              /*  Manage of runtime state, prototype: void ALPLCMANAGE(void)  */

#define  PLC_NUM_TASKS     8                /*  Number of PLC tasks */

// ###########################################################
// ################### NUMBER OF FUNCTIONS ###################
// ###########################################################
// #define _PLC_NUM_FUNCTS   87                /*  Number of embedded functions    */

#if CFG_ENC_HIP
#define _PLC_NUM_ENCHIP_FUNCTS  1	// crs-ok
#else
#define _PLC_NUM_ENCHIP_FUNCTS  0
#endif

#if CFG_ENC_ENDAT
#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
#define _PLC_NUM_ENCEND_FUNCTS  6  // crs-ok
#else
#define _PLC_NUM_ENCEND_FUNCTS  5  // crs-ok
#endif // endat22 && endat22_addinfo
#else
#define _PLC_NUM_ENCEND_FUNCTS  0
#endif // cfg_enc_endat

#if CFG_ENC_NIKON
#define _PLC_NUM_ENCNIK_FUNCTS  4 // crs-ok
#else
#define _PLC_NUM_ENCNIK_FUNCTS  0
#endif
#if CFG_ENC_TMGW
#define _PLC_NUM_ENCTMG_FUNCTS  4 // crs-ok
#else
#define _PLC_NUM_ENCTMG_FUNCTS  0
#endif

#define _PLC_NUM_ENC_FUNCTS    (_PLC_NUM_ENCHIP_FUNCTS + _PLC_NUM_ENCEND_FUNCTS + _PLC_NUM_ENCNIK_FUNCTS + _PLC_NUM_ENCTMG_FUNCTS)

#ifdef INFINEON_
#define _PLC_NUM_FUNCTS        (65 + _PLC_NUM_ENC_FUNCTS) /*  Number of embedded functions */
#else
#define _PLC_NUM_FUNCTS        (87 + _PLC_NUM_ENC_FUNCTS) /*  Number of embedded functions */ // crs-ok
#endif

#ifndef _HW_DC
#define _PLC_NUM_FN1    ((_PLC_NUM_FUNCTS) + 0)
#else
#if CFG_ETHPMC
#define _PLC_NUM_FN1    ((_PLC_NUM_FUNCTS) + 1) // crs-ok
#else
#define _PLC_NUM_FN1    ((_PLC_NUM_FUNCTS) + 0)
#endif // cfg_ethpmc
#endif // ! _hw_dc

#if  CFG_CANDRV_CMDMGR
#define _PLC_NUM_FN2    ((_PLC_NUM_FN1) + 9) // crs-ok
#else
#define _PLC_NUM_FN2    ((_PLC_NUM_FN1) + 0)
#endif

#if  CFG_CANDRV_DS301
#define _PLC_NUM_FN3    ((_PLC_NUM_FN2) + 1) // crs-ok
#else
#define _PLC_NUM_FN3    (_PLC_NUM_FN2)
#endif

#ifndef _APP_XC
#define PLC_NUM_FUNCTS  (_PLC_NUM_FN3)
#else
#define PLC_NUM_FUNCTS  ((_PLC_NUM_FN3) + 3) // crs-ok
#endif
// ###########################################################
// ################### NUMBER OF FUNCTIONS ###################
// ###########################################################

// ############################################################
// ################## NUMBER OF DATA  BLOCKS ##################
// ############################################################
//#define _PLC_NUM_DB     (150) /*  Number of data blocks  */ // crs-ok
#define _PLC_NUM_DB     (140) /*  Number of data blocks  */ // crs-ok (due to cfg_ecat)

#ifdef _APP_PLCDBGWORKS
#define _PLC_NUM_DB1    ((_PLC_NUM_DB) + 1) // crs-ok
#else
#define _PLC_NUM_DB1    (_PLC_NUM_DB)
#endif // _app_plcdbgworks

#ifndef _HW_DC
#define _PLC_NUM_DB2    ((_PLC_NUM_DB1) + 5) // EndatAux: +2; Modbus: +2; CurrScaleAdj: +1  // crs-ok
#else
//#define _PLC_NUM_DB2    ((_PLC_NUM_DB1) + 9) // EthPmc: +5; PSU: +4 // crs-ok
#if CFG_ETHPMC
#define _PLC_NUM_DB2A   ((_PLC_NUM_DB1) + 5) // EthPmc: +5  // crs-ok
#else
#define _PLC_NUM_DB2A   (_PLC_NUM_DB)
#endif // cfg_ethpmc

#if CFG_PSU_MNGR
#define _PLC_NUM_DB2    ((_PLC_NUM_DB2A) + 4) // PSU: +4  // crs-ok
#else
#define _PLC_NUM_DB2    (_PLC_NUM_DB2A)
#endif // cfg_psu_mngr
#endif // ! _hw_dc

#ifndef CFG_DS301_IDENTITY
#define _PLC_NUM_DB3    (_PLC_NUM_DB2)
#else
#define _PLC_NUM_DB3    ((_PLC_NUM_DB2) + 1) // crs-ok
#endif // !cfg_ds301_identity

#ifndef _HW_CT
#define _PLC_NUM_DB4    (_PLC_NUM_DB3)
#else
#define _PLC_NUM_DB4    ((_PLC_NUM_DB3) + 1) // crs-ok
#endif // !_hw_ct

#if CFG_CANDRV_DS301
#define _PLC_NUM_DB5    ((_PLC_NUM_DB4) + 1) // crs-ok
#else
#define _PLC_NUM_DB5    (_PLC_NUM_DB4)
#endif // cfg_candrv_ds301

#if CFG_CANDRV_CMDMGR
#define _PLC_NUM_DB6    ((_PLC_NUM_DB5) + 3) // crs-ok
#else
#define _PLC_NUM_DB6    (_PLC_NUM_DB5)
#endif // cfg_candrv_cmdmgr

#if (OS_MEASURESTACKSIZE)
#define _PLC_NUM_DB7      ((_PLC_NUM_DB6) + 3)  // crs-ok
#else
#define _PLC_NUM_DB7      (_PLC_NUM_DB6)
#endif // os_measurestacksize

#if CFG_ECAT
#define PLC_NUM_DB      ((_PLC_NUM_DB7) + 10) // crs-ok
#else
#define PLC_NUM_DB       (_PLC_NUM_DB7)
#endif // cfg_ecat



// ############################################################
// ################## NUMBER OF DATA  BLOCKS ##################
// ############################################################

/*
**	The folliwing mandatory definitions are the
**	qualifiers used to declare the target 
**	elements tables and used by runtime core
**	to access the tables themselves (const/non const)
*/
#define	PLC_ATTR_PLCIEC_TASKS 		    PLC_ATTR_CONST( PLCIEC_TASKS ) 	/* definitions of task shared tables */
#define	PLC_ATTR_PLCIEC_TASKS_PTR		PLC_ATTR_CONST_PTR( PLCIEC_TASKS )	/* Definition of task shared tables */
#define	PLC_ATTR_PLCIEC_DBREC			PLC_ATTR_CONST( PLCIEC_DBREC ) 		/* Definition of datablock shared tables */
#define	PLC_ATTR_PLCIEC_DBREC_PTR		PLC_ATTR_CONST_PTR( PLCIEC_DBREC ) 	/* Definition of datablock shared tables */
#define	PLC_ATTR_PLCIEC_FCNREC			PLC_ATTR_CONST( PLCIEC_FCNREC )		/* Definition of functions shared tables */
#define	PLC_ATTR_PLCIEC_FCNREC_PTR		PLC_ATTR_CONST_PTR( PLCIEC_FCNREC )	/* Definition of functions shared tables */
#define	PLC_ATTR_PLCIEC_TASKDEF			PLC_ATTR_CONST( PLCIEC_TASKDEF )	/* Definition of task definitions shared tables */


#endif  /*  ALPLCAREADEF_H_ */
