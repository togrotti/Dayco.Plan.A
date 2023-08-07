/* ************************************************************************ * 
 * Project: AxM-E Control Board                                             * 
 *                                                                          * 
 * Copyright � 2005,2008, Phase Motion Control. All Rights Reserved.        *
 *                                                                          * 
 * Version     : 1.0                                                        * 
 * File        : PiDcBusCntrLp32bit.h                                       * 
 * Author      : Cristiano Tognetti                                         *  
 *                                                                          * 
 * Description :                                                            * 
 *                                                                          * 
 * ************************************************************************ */

#ifndef _PI_H
 #define _PI_H

#include "common\GlobalStructures.h" /* to catch GLB_IREF typedef */


/* ================================ #define ================================ */

/* ============================== structures =============================== */
typedef struct {
    SWORD *pswRef ; /* DcBus Reference */
    SWORD *pswFbk ; /* DcBus Feedback  */
    SWORD *pswVoltage ; /* Motor Voltage - VLine */
    UBYTE *pubVoltageValid ; /* Motor Voltage - Vline valid */
} DCBUSCNTRL__INPUT ;


typedef struct {
    /* Output Variables */
    SWORD    swErr    ;  /* Error measured  */
    GLB_IREF sIDcBus  ;  /* DcBus Current   */
    GLB_IREF sIRef    ;  /* Current to FPGA */
    SLONG    slVRatio ;  /* (*swFbk) / (*swVoltage) */
} DCBUSCNTRL_OUTPUT ;


typedef struct {
    /* Flags */
/*    union {
        struct {
            UWORD bOnlyPI : 1 ;
        } b;
        UWORD w;
    } flags;
*/

	BOOL bOnlyPI;
    /* Parameters */
    SWORD swGlobalShift ; /* gains global shift: max shift left = -8; max shift right = +16 */  
    SWORD swKp ; /* proportional gain */
    SWORD swKi ; /* integral gain */
    
    /* Limits */
    SWORD swErrMax ; /* Max Error allowed */
    SWORD swErrMin ; /* Min Error allowed */
    
    /* Current DcBus Limit */
    GLB_TORQUE_LIMIT sI_DcBus ; /* Limite sulla corrente richiesta al DcBus */
    GLB_TORQUE_LIMIT sDummy  ;
} DCBUSCNTRL_PARAMS ; /* PI Control Loop */


typedef struct {
    SWORD swDcBusRef     ;
    SWORD swDcBusFbk     ;
    SWORD swVoltage      ;
    UBYTE ubVoltageValid ;
} DCBUSCNTRL_IN_PLC ;

/* =========================== global  variables =========================== */

extern DCBUSCNTRL__INPUT sDcBusCntrl_In ;
extern DCBUSCNTRL_OUTPUT sDcBusCntrl_Out ;
extern DCBUSCNTRL_PARAMS sDcBusCntrl_Param ;
#ifdef _INFINEON_
extern const DCBUSCNTRL_PARAMS huge sDcBusCntrl_DefParam ;
#else 
extern const DCBUSCNTRL_PARAMS sDcBusCntrl_DefParam ;
#endif // _infineon_

extern DCBUSCNTRL_IN_PLC sDcBusCntrl_Plc ;
extern SWORD swDcBusRef_In_Usr ;

/* =============================== functions =============================== */
BOOL  DcBusCntrl_ControlLoopInit(void) ;

#endif // _PI_H
/* EOF */

