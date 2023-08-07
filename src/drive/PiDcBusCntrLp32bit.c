/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2008, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : PiDcBusCntrLp32bit.c                                       */
/* Author      : Cristiano Tognetti                                         */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/
#include <math.h>

#include "system\SysAppGlobals.h"
#include "common\CommonDefines.h"

#include "drive\PiDcBusCntrLp32bit.h"
#include "common\TaskScheduler.h"

#include "common\Int64Functions.h"
#include "drive\MotorHandler.h"     /* to catch DcBusValue read and Motor Voltage */
#include "drive\MotionController.h" /* to catch the mode of operation */
#include "system\SysLogManagement.h" /* to be able to send errors */

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

/* ================================ #define ================================ */
#define GAIN_GLOBAL_MAX_SHIFT_RIGHT    16
#define GAIN_GLOBAL_MAX_SHIFT_LEFT     8

/* ============================== structures =============================== */
typedef struct { 
    /* Flags */
    union {
        struct {
            UWORD bUpdateOutValueLimit : 1 ;
            UWORD bUpdateIntegralLimit : 1 ;
            UWORD bOnlyPI              : 1 ;
        } b;
        UWORD w;
    } flags;
    
    /* parameters image */
    SWORD swErrMax ;
    SWORD swErrMin ; 
    SWORD swKp ;
    SWORD swKi ;
    SWORD swGlobalShift ;
    GLB_TORQUE_LIMIT sI_DcBus ;
    
    /* runtime */
    SLONG slIntegral32 ; 
    GLB_TORQUE_LIMIT slIqOutVal ;
    
    SLONG slIntegralMaxLimit ;
    SLONG slIntegralMinLimit ;

} DCBUSCNTRL_RUNTIME ;


DCBUSCNTRL__INPUT sDcBusCntrl_In ;
DCBUSCNTRL_OUTPUT sDcBusCntrl_Out ;
DCBUSCNTRL_PARAMS sDcBusCntrl_Param ;

DCBUSCNTRL_IN_PLC sDcBusCntrl_Plc ;
SWORD swDcBusRef_In_Usr ;

#ifdef _INFINEON_
static DCBUSCNTRL_RUNTIME sdata sDcBusCntrl_Run ;

// default parameters
const DCBUSCNTRL_PARAMS huge sDcBusCntrl_DefParam =
#else // _infineon_
static DCBUSCNTRL_RUNTIME sDcBusCntrl_Run ;

// default parameters
const DCBUSCNTRL_PARAMS sDcBusCntrl_DefParam =

#endif // _infineon_
{
    {1},   // Flags: bit0 = 0 (PI only)
    0,     // Gains global shift
    0,     // Kp
    0,     // Ki
    10,    // Max Error allowed
    -10,   // Min Error allowed
    {0,0}, // PI Limits
    {0,0}, // Dummy
} ;

/* =========================== global  variables =========================== */

/* =============================== functions =============================== */
static SWORD CheckSWordParam(SWORD swParamVal, SWORD swPositiveLimit, SWORD swNegativeLimit) ;
static BOOL  DcBusCntrl_ControlLoop8KHz(void) ;
static BOOL  DcBusCntrl_UpdateValue8KHz(void) ;
static void  DcBusCntrl_ControlLoopBkGd(void) ;
static void  DcBusCntrl_ControlLoopRst(void) ; /* resetta solo l'integrale e le variabili d'uscita */

/* ######################################################################### */
static SWORD CheckSWordParam(SWORD swParamVal, SWORD swPositiveLimit, SWORD swNegativeLimit) 
{
    if (swParamVal > swPositiveLimit)
        swParamVal = swPositiveLimit ;
    else if (swParamVal < -swNegativeLimit)
        swParamVal = -swNegativeLimit ;
    
    return swParamVal ;
}


/* ######################################################################### */
BOOL DcBusCntrl_ControlLoopInit(void)
{
    /* ========= pointer initialization (DEFAULT configuration) ========= */
//    sDcBusCntrl_In.pswRef          = &swDcBusRef_In_Usr ;
//    sDcBusCntrl_In.pswFbk          = &sMh_MotorDataOut.swDcBusValue ;
//    sDcBusCntrl_In.pswVoltage      = &sActFrEmfEncOut.swBackEmfVline ;     
//    sDcBusCntrl_In.pubVoltageValid = &sActFrEmfEncOut.ubEmfEncValid  ;
    sDcBusCntrl_In.pswRef          = &sDcBusCntrl_Plc.swDcBusRef     ;
    sDcBusCntrl_In.pswFbk          = &sDcBusCntrl_Plc.swDcBusFbk     ;
    sDcBusCntrl_In.pswVoltage      = &sDcBusCntrl_Plc.swVoltage      ;     
    sDcBusCntrl_In.pubVoltageValid = &sDcBusCntrl_Plc.ubVoltageValid ; 

    /* select if it is used as SIMPLE PI control loop */
    sDcBusCntrl_Run.flags.b.bOnlyPI = sDcBusCntrl_Param.bOnlyPI ;//sDcBusCntrl_Param.flags.b.bOnlyPI ;
    sDcBusCntrl_Run.swKp = sDcBusCntrl_Param.swKp ;
    sDcBusCntrl_Run.swKi = sDcBusCntrl_Param.swKi ;
    
    /* control loop limit 32bit */   
    sDcBusCntrl_Run.sI_DcBus.slMax     = sDcBusCntrl_Param.sI_DcBus.slMax ;
    sDcBusCntrl_Run.sI_DcBus.slMin     = sDcBusCntrl_Param.sI_DcBus.slMin ;  
    sDcBusCntrl_Run.slIqOutVal.slMax   = sDcBusCntrl_Run.sI_DcBus.slMax ;
    sDcBusCntrl_Run.slIqOutVal.slMin   = sDcBusCntrl_Run.sI_DcBus.slMin ; 
    sDcBusCntrl_Run.slIntegralMaxLimit = sDcBusCntrl_Run.sI_DcBus.slMax ;
    sDcBusCntrl_Run.slIntegralMinLimit = sDcBusCntrl_Run.sI_DcBus.slMin ;
    
    
    /* APPLYING global shift NOTE THAT shiftLeft and shiftLeft are OPPOSITE to GlobalShift *
    * if globalshift > 0 (it means right), slIntegralLimit MUST be shifted left!          *
    * if globalshift < 0 (it means left) , slIntegralLimit MUST be shifted right!         */  
    sDcBusCntrl_Run.swGlobalShift = CheckSWordParam(sDcBusCntrl_Param.swGlobalShift, GAIN_GLOBAL_MAX_SHIFT_RIGHT, GAIN_GLOBAL_MAX_SHIFT_LEFT) ; /* 60fa.9h */
    
    if (sDcBusCntrl_Run.swGlobalShift > 0)
    {
        sDcBusCntrl_Run.slIntegralMaxLimit <<= (UWORD)sDcBusCntrl_Run.swGlobalShift ;
        sDcBusCntrl_Run.slIntegralMinLimit <<= (UWORD)sDcBusCntrl_Run.swGlobalShift ;   
    }
    else if(sDcBusCntrl_Run.swGlobalShift < 0)
    {
        sDcBusCntrl_Run.slIntegralMaxLimit >>= (UWORD)(-sDcBusCntrl_Run.swGlobalShift) ;
        sDcBusCntrl_Run.slIntegralMinLimit >>= (UWORD)(-sDcBusCntrl_Run.swGlobalShift) ; 
    }   
    
    /* check error limit */
    if (sDcBusCntrl_Param.swErrMax < 0)
        sDcBusCntrl_Param.swErrMax = -sDcBusCntrl_Param.swErrMax ; 
    
    sDcBusCntrl_Run.swErrMax =  sDcBusCntrl_Param.swErrMax ;
    sDcBusCntrl_Run.swErrMin = -sDcBusCntrl_Param.swErrMax ;
    
    /* Runtime variables */ 
    sDcBusCntrl_Run.flags.b.bUpdateOutValueLimit = FALSE ;
    sDcBusCntrl_Run.flags.b.bUpdateIntegralLimit = FALSE ;
    
    DcBusCntrl_ControlLoopRst() ;  /* reset OutValues */
    
    if(!TaskSched_AddBackgroundTask(&DcBusCntrl_ControlLoopBkGd))
        return FALSE ; 
    
    /* install 8KHz function */
    if(!TaskSched_AddRTTask(&DcBusCntrl_ControlLoop8KHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0))
        return FALSE ; 
    
    return TRUE ;    
}

/* ######################################################################### */
static BOOL DcBusCntrl_ControlLoop8KHz(void)
{
    SWORD swError ;
    SLONG slProportional32, slIqRef2Fpga ; 
    SQWRD sqIqRef2Fpga ;
    
    DcBusCntrl_UpdateValue8KHz() ; /* update values from background (if necessary) */
    
    /* keep integral reset until full power */
    if (!*sDcBusCntrl_In.pubVoltageValid)
    {   /* reset and DO NOT execute the control loop */
        DcBusCntrl_ControlLoopRst() ;  
        return TRUE ; 
    }  

    /* == error limit == */
    swError = (SWORD)((SLONG)*sDcBusCntrl_In.pswRef - *sDcBusCntrl_In.pswFbk) ; /* Error(2^15+s) = Reference(2^15+s) - Feedback(2^15+s) */
    
    if (swError > sDcBusCntrl_Run.swErrMax) 
        swError = sDcBusCntrl_Run.swErrMax ;
    else if(swError < sDcBusCntrl_Run.swErrMin)
        swError = sDcBusCntrl_Run.swErrMin ;
    
    
    /* == Proportional contribute == */
    slProportional32 = (SLONG)swError * sDcBusCntrl_Run.swKp ; /* slProportional32(2^31+s) = Error(2^15+s) * Kp(2^15+s) */
    
    
    /* == Integral contribute  (and limit it to void 'explosions') == */
    sDcBusCntrl_Run.slIntegral32 += ((SLONG)swError * sDcBusCntrl_Run.swKi) ; /* slIntegral32(2^31+s) = Error(2^15+s) * Ki(2^15+s) + slIntegral32_1(2^31+s) */
    
    if (sDcBusCntrl_Run.slIntegral32 > sDcBusCntrl_Run.slIntegralMaxLimit)  
        sDcBusCntrl_Run.slIntegral32 = sDcBusCntrl_Run.slIntegralMaxLimit ;
    else if (sDcBusCntrl_Run.slIntegral32 < sDcBusCntrl_Run.slIntegralMinLimit)
        sDcBusCntrl_Run.slIntegral32 = sDcBusCntrl_Run.slIntegralMinLimit ;
    
    
    /* == Proportional + Integral == */
    sDcBusCntrl_Out.sIDcBus.slIqRef = slProportional32 + sDcBusCntrl_Run.slIntegral32 ;  
    
    
    /* == Global  shift == */
    if (sDcBusCntrl_Run.swGlobalShift > 0)  
        sDcBusCntrl_Out.sIDcBus.slIqRef  >>=   sDcBusCntrl_Run.swGlobalShift  ; /* global shift right */
    else if(sDcBusCntrl_Run.swGlobalShift < 0)
        sDcBusCntrl_Out.sIDcBus.slIqRef  <<= (-sDcBusCntrl_Run.swGlobalShift) ; /* global shift left */
    
    
    /* == limit to slIqOutVal.Max or slIqOutVal.Min == */
    if (sDcBusCntrl_Out.sIDcBus.slIqRef > sDcBusCntrl_Run.slIqOutVal.slMax)   
        sDcBusCntrl_Out.sIDcBus.slIqRef = sDcBusCntrl_Run.slIqOutVal.slMax ;
    else if (sDcBusCntrl_Out.sIDcBus.slIqRef < sDcBusCntrl_Run.slIqOutVal.slMin)
        sDcBusCntrl_Out.sIDcBus.slIqRef  = sDcBusCntrl_Run.slIqOutVal.slMin ; 
    
    
    // ================================================================================================================= //
    /* send the values out */
    sDcBusCntrl_Out.swErr = swError ; 
    
    if(sDcBusCntrl_Run.flags.b.bOnlyPI)
    {
        sDcBusCntrl_Out.sIRef.slIqRef = sDcBusCntrl_Out.sIDcBus.slIqRef ;
        sDcBusCntrl_Out.slVRatio = 0 ;
    }
    else
    {   /* if Motor,   Voltage = Motor BackEmfVoltage (sign change with the direction (clockwise = positive; anticlockwise = negative)) ;
         * if AC Line, Voltage = VLine Voltage (sign DOESN'T change: it is found during backonthefly phase and the sign depends on how the phase are connected) 
         */
        if (*sDcBusCntrl_In.pubVoltageValid)
            sDcBusCntrl_Out.slVRatio = ((SLONG)(*sDcBusCntrl_In.pswFbk) << 8) / *sDcBusCntrl_In.pswVoltage ;  /* VRatio = Vdc(always positive) / Voltage(signed) (Motor or Line)*/
        else
        {
            sDcBusCntrl_Out.slVRatio = 0 ;
            sDcBusCntrl_Run.slIntegral32 = 0 ; /* keep integral resetted */
        }
        _sint64_mul_32_32(&sqIqRef2Fpga, &sDcBusCntrl_Out.sIDcBus.slIqRef, &sDcBusCntrl_Out.slVRatio) ;
        _sint64_shl(&sqIqRef2Fpga, 8) ; /* slVRatio is multiplied for 256 */
        slIqRef2Fpga = (SLONG)(INT64_MLONG(sqIqRef2Fpga)) ;
    
        sDcBusCntrl_Out.sIRef.slIqRef = -slIqRef2Fpga ; /* Invert current sign. Controlling the DC-Bus: *
                                                         * - when voltage rise, current negative        *
                                                         * - when voltage down, current positive        */
    }
    // ================================================================================================================= //
    
    return TRUE ;
}

/* ######################################################################### */
static void DcBusCntrl_ControlLoopBkGd(void)
{  
	if (sDcBusCntrl_Run.flags.b.bOnlyPI != sDcBusCntrl_Param.bOnlyPI)  //if (sDcBusCntrl_Run.flags.b.bOnlyPI != sDcBusCntrl_Param.flags.b.bOnlyPI)
        sDcBusCntrl_Run.flags.b.bOnlyPI = sDcBusCntrl_Param.bOnlyPI ;  //sDcBusCntrl_Param.flags.b.bOnlyPI ;

    if (sDcBusCntrl_Run.swKp != sDcBusCntrl_Param.swKp)
        sDcBusCntrl_Run.swKp = sDcBusCntrl_Param.swKp ;
    
    if (sDcBusCntrl_Run.swKi != sDcBusCntrl_Param.swKi)
        sDcBusCntrl_Run.swKi = sDcBusCntrl_Param.swKi ;     
    
    
    /* check error limit */
    if (sDcBusCntrl_Param.swErrMax < 0)
        sDcBusCntrl_Param.swErrMax = -sDcBusCntrl_Param.swErrMax ; 
    
    if (sDcBusCntrl_Run.swErrMax != sDcBusCntrl_Param.swErrMax) 
    {  
        sDcBusCntrl_Run.swErrMax =  sDcBusCntrl_Param.swErrMax ;
        sDcBusCntrl_Run.swErrMin = -sDcBusCntrl_Param.swErrMax ;  
    }
    
    
    /* ------------------------------------------------------- * 
    * catching the Imax and Imin requested to the DcBus ONLY  *
    * IF it has changed from the ones I'm using               *
    * ------------------------------------------------------- */ 
    if ((sDcBusCntrl_Run.sI_DcBus.slMax != sDcBusCntrl_Param.sI_DcBus.slMax) ||
        (sDcBusCntrl_Run.sI_DcBus.slMin != sDcBusCntrl_Param.sI_DcBus.slMin)   )
    {   /* checking positive limit */
        if (sDcBusCntrl_Param.sI_DcBus.slMax < 0) 
            sDcBusCntrl_Run.sI_DcBus.slMax = 0L ; 
        else
            sDcBusCntrl_Run.sI_DcBus.slMax = sDcBusCntrl_Param.sI_DcBus.slMax ;
    
        /* checking negative limit */
        if (sDcBusCntrl_Param.sI_DcBus.slMin > 0) 
            sDcBusCntrl_Run.sI_DcBus.slMin = 0L ;
        else
            sDcBusCntrl_Run.sI_DcBus.slMin = sDcBusCntrl_Param.sI_DcBus.slMin ;
         
        sDcBusCntrl_Param.sI_DcBus = sDcBusCntrl_Run.sI_DcBus ; /* readback */
    
        /* values will be transferred by Pi_DcBusUpdateValue8KHz to also update the integral limit */
        sDcBusCntrl_Run.flags.b.bUpdateOutValueLimit = TRUE ;
    }
    
    
    /* ---------------------------------------------------------------------- *
     * catching gains global shift ONLY IF it has changed from previous value *
     * - positive value means shift right                                     *
     * - negative value means shift left                                      *
     * ---------------------------------------------------------------------- */
    if (sDcBusCntrl_Run.swGlobalShift != sDcBusCntrl_Param.swGlobalShift)
    {   /* checking limit */
        sDcBusCntrl_Run.swGlobalShift = CheckSWordParam(sDcBusCntrl_Param.swGlobalShift, GAIN_GLOBAL_MAX_SHIFT_RIGHT, GAIN_GLOBAL_MAX_SHIFT_LEFT) ;  
        sDcBusCntrl_Param.swGlobalShift = sDcBusCntrl_Run.swGlobalShift ; /* readback */    
        sDcBusCntrl_Run.flags.b.bUpdateIntegralLimit = TRUE ; /* allow Fast Task to update the integral limit */
    }
}

/* ######################################################################### */
/* ************************************************************ *
 * 1) FastTask is faster than and can interrupt BackGround Task *
 * 2) If I have to update ONLY OutValue limit:                  *
 *    2.1) first update limit and integral limit                *
 *    2.2) then shift if necessary (set the flag)               *
 * 3) If I have to update ONLY global shift:                    *
 *    3.1) shift the integral limit value I already have        *
 * ************************************************************ */
static BOOL DcBusCntrl_UpdateValue8KHz(void)
{
    if (sDcBusCntrl_Run.flags.b.bUpdateOutValueLimit)
    {   /* copy OutValue limit to control loop */
        sDcBusCntrl_Run.slIqOutVal.slMax = sDcBusCntrl_Run.sI_DcBus.slMax ;
        sDcBusCntrl_Run.slIqOutVal.slMin = sDcBusCntrl_Run.sI_DcBus.slMin ;
    
        sDcBusCntrl_Run.flags.b.bUpdateOutValueLimit = FALSE ; /* reset the flag */
        sDcBusCntrl_Run.flags.b.bUpdateIntegralLimit = TRUE  ; /* set the flag to update integral limit */      
    }
    
    if (sDcBusCntrl_Run.flags.b.bUpdateIntegralLimit)
    {   /* update TorqueLimit: NOTE THAT _shl and _shr are OPPOSITE to GlobalGainShift  *
         * if globalshift > 0 (it means right), MaxIntegralValue MUST be shifted left!  *
         * if globalshift < 0 (it means left) , MaxIntegralValue MUST be shifted right! */
        sDcBusCntrl_Run.slIntegralMaxLimit = sDcBusCntrl_Run.slIqOutVal.slMax ;
        sDcBusCntrl_Run.slIntegralMinLimit = sDcBusCntrl_Run.slIqOutVal.slMin ;  
    
        if (sDcBusCntrl_Run.swGlobalShift > 0)
        {
            sDcBusCntrl_Run.slIntegralMaxLimit <<= sDcBusCntrl_Run.swGlobalShift ;
            sDcBusCntrl_Run.slIntegralMinLimit <<= sDcBusCntrl_Run.swGlobalShift ;
        }
        else if(sDcBusCntrl_Run.swGlobalShift < 0)
        {
            sDcBusCntrl_Run.slIntegralMaxLimit >>= (-sDcBusCntrl_Run.swGlobalShift) ;
          sDcBusCntrl_Run.slIntegralMinLimit >>= (-sDcBusCntrl_Run.swGlobalShift) ;
        } 
    
        sDcBusCntrl_Run.flags.b.bUpdateIntegralLimit = FALSE ; /* reset the flag */
    }  
    
    return TRUE ;  
}

/* ######################################################################### */
static void DcBusCntrl_ControlLoopRst(void)
{
    sDcBusCntrl_Run.slIntegral32 = 0L ; 
    
    sDcBusCntrl_Out.swErr = 0 ;  
    sDcBusCntrl_Out.sIDcBus.slIdRef = 0L ;
    sDcBusCntrl_Out.sIDcBus.slIqRef = 0L ;
    sDcBusCntrl_Out.sIRef.slIdRef   = 0L ;
    sDcBusCntrl_Out.sIRef.slIqRef   = 0L ;
}
