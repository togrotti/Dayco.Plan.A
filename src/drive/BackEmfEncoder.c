/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : BackEmfEncoder.c                                           */
/* Author      : Cristiano Tognetti                                         */
/*                                                                          */
/* Description : BackEmf Encoder                                            */
/*                                                                          */
/****************************************************************************/
#include <stdlib.h> // to use labs()
#include <math.h>

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "system\SysLogManagement.h"
#include "system\SystemAlarms.h"
#include "common\TaskScheduler.h"
#include "common\MathFunctions.h"
#include "common\Int64Functions.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "common\DspFunctions.h"

#include "common\CommonMotorHandler.h"
#include "common\CommonEncoderManager.h"
#include "drive\BackEmfEncoder.h"

#if CFG_ENC_BEMF_DITEN
#include "drive\MotorHandler.h"     // to catch sMh_MotorDataOut structure: slIuFb, slIvFb, slIwFb, swVuOut, swVvOut;
                                    // swVuEffective, swVvEffective, swVwEffective when available
									// pay attention that it works only with in 1 bridge drive!
#include "drive\EncoderEFSeek.h"    // to catch sEfsParam.ubProcType
#endif // cfg_enc_bemf_iten

#include "drive\Deflux.h" // per prendere la velocita' di ginocchio

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#ifdef _CRS_DBG
#if (FALSE) // CRS_DBGDSK
	#pragma GCC optimize (0)
#else
	#pragma GCC optimize (2)
#endif // crs_dbgdsk
#else
	#pragma GCC optimize (2)
#endif // _crs_dbg

// ================================ #define ================================ 
#define SNSRLESS_DISABLED       0
#define SNSRLESS_THRESHOLD_0    1
#define SNSRLESS_THRESHOLD_1    2 
#define SNSRLESS_MIXING         3
#define SNSRLESS_FULL           4
#define SNSRLESS_MIXING_OL      5
#define SNSRLESS_FULL_OL        6

#define EMFENC_OK         0x00
#define EMFENC_RESET      0x55
#define EMFENC_ZEROOUT    0xAA
#define EMFENC_FAULT      0x5A

// define per le velocita'
#define EMFSPEED_MEASURE_TIMEOUT  24  // = 24 ticks = 3ms * 8ticks
#define HUNDREDPERCENT_INT        10000
#define HUNDREDPERCENT_FLOAT      10000.0

#define BACKONTHEFLYDELTASPEED    10        // 10%

//#define IQSHIFT_THRESHOLD_0   3 // IqRef / 8
//#define IQSHIFT_THRESHOLD_1   2 // IqRef / 4
//#define IQSHIFT_MIXING        1 // IqRef / 2
//#define IQSHIFT_FULL          0 // IqRef    

// define per il numero magico / valutazione Kt
#define ONE_SECOND  1000
#define FLOAT_2PI                       (2.0 * FLOAT_PI)
#define FLOAT_1SECOND_IN_TICKS           8000.0
#define FLOAT_RADSEC2INTERNALUNIT       (FLOAT_2POWER32 / (FLOAT_1SECOND_IN_TICKS * FLOAT_2PI))  // 2^32 / (8000 * 2 * PI)
#define FLOAT_KT_CONVFACTOR             (FLOAT_SQRT_OF_THREE * FLOAT_RADSEC2INTERNALUNIT) / (10.0 * FLOAT_SQRT_OF_TWO)

#define FLOAT_KT_PERCENTAGE_MIN          0.75
#define FLOAT_KT_PERCENTAGE_MAX          1.25

#define SLOWTASK_FLAG_FAULT_KT           0x0008
#define SLOWTASK_FLAG_FAULT_MAGICNUMBER  0x0010

// adattamento soglie in base a tensione dcbus e Kt motore                
// MaxMechSpeed = ((DcBus(iu)/10)(V) * sqrt(3)) / (1.03 * sqrt(2) * Kt) = DcBus(iu)/Kt * (sqrt(3)/(10 * 1.03 * sqrt(2)) = DcBus(iu)/Kt * 0.11891
// 1.03 = 3% di margine
#define MAXMECHSPDCONVFACTOR_RADSEC     (FLOAT_SQRT_OF_THREE / (10.0 * 1.03 * FLOAT_SQRT_OF_TWO))   // rad/sec
#define MAXMECHSPDCONVFACTOR_IU         (MAXMECHSPDCONVFACTOR_RADSEC * FLOAT_RADSEC2INTERNALUNIT)   // internal unit (increment x tick)

#define CURRENT_RAMP_TIME                (5 * 8)    // 5ms

#if CFG_ENC_BEMF_DITEN_PARAM
#define DITEN_OBS_SPEED_GAIN			85469.23
#define DITEN_OBS_POSITION_GAIN			683565275.57
#define DITEN_OBS_EL_POSITION_GAIN		10430.37

#define DITEN_OBS_PLL_KP_GAIN			0.01
#define DITEN_OBS_PLL_KI_GAIN			0.01
#define DITEN_OBS_PLL_TF_GAIN			0.0001
#define DITEN_OBS_A_GAIN				0.01
#define DITEN_OBS_G1_GAIN				0.00001
#define DITEN_OBS_G2_GAIN				0.00001
#define DITEN_OBS_INJ_MAXVALUE_GAIN		0.01
#define DITEN_OBS_INJ_ACCRATE_GAIN		0.01
#define DITEN_OBS_MIN_SPEED_GAIN		0.01

#define DITEN_270DEG_IN_RADIANT        (FLOAT)(270.0 * FLOAT_PI / 180.0)
#define DITEN_OBS_SERVOTIME            (FLOAT)(1.0 / FLOAT_1SECOND_IN_TICKS)
#define C_2D3                          (FLOAT)(2.0 / 3.0)
#define C_1DSQRT3                      (FLOAT)(1.0 / FLOAT_SQRT_OF_THREE)
#endif // cfg_enc_bemf_diten

// ============================== structures =============================== 

typedef struct {
    // Output
    SLONG   slPostnOPL ;
    SLONG   slSpeedOPL ;
    
    // regressioni
    SLONG   slPostnOPL_1 ; // regressione posizione openloop
} BKEMF_OPENLOOP ;


typedef struct {
    // Output
    SLONG   slPostnEmf ; // posizione elettrica emf 
    SLONG   slSpeedEmf ; // velocita' meccanica emf 
    SLONG   slAccelEmf ; // = 0 !! 
        
    // runtime variables
    // Flags
    union {
        struct {
            UWORD bAngleH_1 : 1 ; // regressione sul quadrante dove stava angolo emf al servo i-1
            UWORD bAngleL_1 : 1 ; // regressione sul quadrante dove stava angolo emf al servo i-1
        } b ;
        UWORD w ;
    } flags ;     
    
    UWORD   uwEmfAtanAngle ; // serve in struttura per il filtro antiglitch
    
    SLONG   slEmfPos ;     // posizione elettrica emf 
    SLONG   slPrecAngleO ; // filtro angolo emf   

    ULONG   ulAngles_1[16] ; // recupero ritardo 
    UWORD   uwIndex ;

    SWORD   swTurnsEmf ; // numero di giri elettrici per sincronizzare OL ed EMF 
    ULONG   ulAngleEmf ; // angolo elettrico Emf da cui derivo il numero di giri Emf
    
    SLONG   slEmfAbsSpdFiltered ; // filtro velocita' emf
    
} BKEMF_EMF ;


typedef struct {
    UWORD uwOneSecBkgdTimer ;
    UWORD uwOneSecBkgdCntr ;
    ULONG ulKtCounter ;
    FLOAT flSQRootSum ;
    FLOAT flMechSpeedDeltaAngleSum ;
    FLOAT flKtMin ;
    FLOAT flKtMax ;
    FLOAT flIncrementoKt ;
} KT_EVALUATION ;


typedef struct {
    FLOAT   flSsnrlssSpeed ;       
    FLOAT   flSsnrlssThreshold1 ;
    FLOAT   flSsnrlssThreshold0 ; 
    FLOAT   flEmfEncAGlitchFilter ; 
    FLOAT   flBackOnTheFlyUsrSpeed ;
    FLOAT   flBackOnTheFlyMinSpeed ;
} BACKEMF_PERCENTAGE ;


typedef struct {
    BKEMF_OPENLOOP  sOpl ; // dati OpenLoop
    BKEMF_EMF       sEmf ; // dati Emf

    // Flags
    union {
        struct {
            UWORD bUseOPLTurns : 1 ;      // Manager: flag per imporre n. giri elettrici OpenLoop = n. giri Emf
            UWORD bEmfSpeedMeasured : 1 ; // Manager: flag per il rientro al volo
            UWORD bKtOneSecondTimerSet : 1 ; // Timer per la valutazione del Kt
            UWORD bDummy: 2 ;
            UWORD bOpenLoopOnly : 1 ; // flag to work in openloop only
            UWORD bRotorZeroFound : 1;
            UWORD bHookInTransition : 1;
            UWORD bNegativeSign : 1 ;
        } b ;
        UWORD w ;
    } flags ; 

    union {
        struct {
            UWORD bAntiGlitch : 1 ;
            UWORD bSyncLost : 1 ;
            UWORD bInvalidMagicNumber : 1 ;
            UWORD bKtOutOfRange : 1 ;
        } b ;
        UWORD w ;
    } sErrorSent ;

    ENCMGR_SPACEFEEDBACK *psBackEmfOut ;

    // parametri motore    
    UWORD   uwPolePairs ;
    SLONG   slMotorCurrentPeak ;
    ULONG   ulMechAdder ; // controllare!!
    ULONG   ulMechSteps ; // controllare!!  
    
    ULONG   ulElecEmfEncAngle ; // per calcolare overflow di giro elettrico e numero di giri swElecEmfEncTurns
    SWORD   swElecEmfEncTurns ; // per mantenere OpenLoop ed Emf sincronizzati e fare insieme l'overflow di giro

    SLONG   slAbsMechSpeed ; // valore assoluto velocità meccanica del sensore (i-1) per le soglie, per i pesi ed il filtro antiglitch
    UBYTE   ubEmfEncStatus ; // Variabile per decidere cosa fare con i dati in uscita dal BackEmf
//    UBYTE   ubIqShift2Use ;  // riduzione Iq durante fase mix

    SLONG   slBackOnTheFlyMinSpeed ; // minimo valore di rientro al volo (derivato dai parametri motore)
    SLONG   slBackOnTheFlyUsrSpeed ; // valore di rientro al volo utente
    SLONG   slEmfEncGlitchFilterThreshold ; // soglia di intervento antiglitch

    // Manager - mix   
    SWORD   swWeightOPL ; // e' segnata per la moltiplicazione 32x16, in realta' e' un numero [0; 0x7fff]
    SWORD   swWeightEmf ; // e' segnata per la moltiplicazione 32x16, in realta' e' un numero [0; 0x7fff]  

    SLONG   slSpeedSensorless ; // velocita' di pieno sensorless (% velocita' nominale motore)
    SLONG   slSpeedThreshold1 ; // Soglia 2: 12.5->25% slSpeedSensorless 
    SLONG   slSpeedThreshold0 ; // Soglia 1:  0->12.5% slSpeedSensorless
    SWORD   swSpeedSlope ;      // pendenza retta di mix: dipende da slSpeedSensorless e slSpeedThreshold1

    // Manager - rientro al volo
    UWORD   uwEmfSpeedTimer ; // timer per misurare il periodo in cui eseguire il controllo per il rientro al volo

    KT_EVALUATION sKtEval ;
    
    ULONG   ulAlarm2Send ;

    BACKEMF_PERCENTAGE  sPercentage ;

    // generatore rampe corrente nel passaggio da EFS a BEMF
    SLONG   slHookMaxStep;

    SBYTE * psbEncMgrFault;

    UWORD   uwSyncLostHysteresis ; // isteresi sul fault di perdita di sincronismo (serve per il rientro al volo)

} EMFENC_RUNTIME ;

#if CFG_ENC_BEMF_DITEN
typedef struct {
	float gamma1;
	float gamma2;
	float a;
	float tfilter;
} S_OBSERVER_PARAMETER;

typedef struct {
	float flux[4];
	float deflux[4];
	float q1[2];
	float q2[2];
	float q12q[2];
	float yq;
	float omegaq1;
	float omegaq2;

	float yq_i;
	float omegaq1_i;
	float omegaq2_i;

	float x1;
	float x2;
	float thet;
	float t_idr;
} S_OBSERVER_VARIABLE;

typedef struct {
	float elAngleDelayed;
	int revolutionCounter;
	int mAngleAdder;
} S_OBSERVER_MECH_OUT;

typedef struct {
	S_OBSERVER_PARAMETER parameter;
	S_OBSERVER_VARIABLE	 variable;
	S_OBSERVER_MECH_OUT MechOut;
} S_OBSERVER;

typedef struct {
	float kp;
	float ki;
	float tfilter;
} S_PLL_PARAMETER;

typedef struct {
	float cos_teta;
	float sin_teta;
	float cos_tetaest;
	float sin_tetaest;
	float epsilon;
	float tetaest;
	float integral;
	float omegaest;
	float omegafiltr[2];
	int wrap;
} S_PLL_VARIABLE;

typedef struct {
	S_PLL_PARAMETER parameter;
	S_PLL_VARIABLE  variable;
} S_PLL;

typedef struct {
	float maxValuePercentOfRated;
	float accRateInSecToRated;
	float minSpeedWithoutInjection;
} OBS_ID_INJECT_PARAMS;

typedef struct {
	OBS_ID_INJECT_PARAMS params;
	float current_value;
	float target_value;
} OBS_ID_INJECT;

typedef struct {
	OBS_ID_INJECT	id_inject;
	S_OBSERVER		diten_observer;
	S_PLL			diten_pll;
	S_OBS_PLL_OUT 	diten_obs_pll_out;
	SWORD 			swTestCounter ;
	FLOAT 			flHookTimer ;
} DITEN_RUNTIME ;
#endif // cfg_enc_bemf_diten

// =========================== global  variables ===========================
BE_EMFENC__IN       sBe_EmfEncIn ;
BE_EMFENC_DIAG_OUT  sBe_EmfEncDiagOut ;
BE_EMFENC_PARAM     sBe_EmfEncParam ;

// default parameters
#ifdef _INFINEON_
const BE_EMFENC_PARAM huge sBe_EmfEncDefParam =
#else
const BE_EMFENC_PARAM sBe_EmfEncDefParam =
#endif // _infineon
{
    {0,0,0,1,0},    // antiglitch enabled, DeltaAngleSpeed
    9000, // BackOnTheFly: 90.00% speed max
    2000, // sensorless:   20.00% speed max
    500,  // threshold_1:  5.00% speed max
    250,  // threshold_0:  2.50% speed max 
    250,  // antiglitch threshold:  2.50% speed max 
    0,    // antiglitch fault disabled
    15,   // valued Kt refresh
    0l,

#if (CFG_ENC_BEMF_DITEN_PARAM)
    300,  // 30% of MOTPRM_PARAMETERS.flCurrentPeak

	// diten
    80000,   // kp/DITEN_OBS_PLL_KP_GAIN
    1000000, // ki/DITEN_OBS_PLL_KI_GAIN
    4,       // tf/DITEN_OBS_PLL_TFILTER_GAIN
    500000,  // g1/DITEN_OBS_G1_GAIN
    50000,   // g2/DITEN_OBS_G2_GAIN
    10000,   // alpha/DITEN_OBS_A_GAIN
	1000,    // id_inj_maxValuePercent/DITEN_OBS_INJ_MAXVALUE_GAIN
	100,     //id_inj_accRateInSec/DITEN_OBS_INJ_ACCRATE_GAIN
	1600     //id_inj_minSpeedWithoutInjection/ DITEN_OBS_MIN_SPEED_GAIN
#else
    300   // 30% of MOTPRM_PARAMETERS.flCurrentPeak
#endif // cfg_enc_bemf_diten

};

#if CFG_ENC_BEMF
// =========================== local  variables ===========================
static EMFENC_RUNTIME   sEmfEncRun ;

#if CFG_ENC_BEMF_DITEN
DITEN_RUNTIME sDitenRun ;
#endif // cfg_enc_bemf_diten

// =============================== functions ===============================
static ULONG BackEmfEncInit(ENCMGR_SPACEFEEDBACK *psEncOut, SBYTE * psbEncMgrFault) ;
static ULONG calcbasemagicnumber(FLOAT flKT);

#if CFG_ENC_BEMF_DITEN
static BOOL Diten_Init(S_OBSERVER *observer, S_PLL *pll, MOTPRM_PARAMETERS *parameters, MH_MOTORDATA_OUT *currVolts);
static BOOL Diten_Task8KHz(S_OBSERVER *observer, S_PLL *pll, MOTPRM_PARAMETERS *parameters, MH_MOTORDATA_OUT *currVolts);
static void Diten_Observer_Run(void);
static void Diten_8kHz(void);
#endif // cfg_enc_bemf_diten

static BOOL BackEmfHandler8KHz(void) ;
static void BackEmfHandlerBkGd(void) ;
static void BackEmfEncOpl8KHz(void) ;                        
static void BackEmfEncEmf8KHz(void) ;                                        
static void BackEmfMngrMix8KHz(void) ;
static void BackEmfEncMechOut8KHz(void) ;
static void BackEmfMngrBackOnTheFly8KHz(void) ;
static void BackEmfMngrFault8KHz(void) ;
static void BackEmfEncOut8KHz(UWORD uwStatus) ;
static void BackEmfEncResetOut(void) ;
static void BackEmfEncZeroOut(void) ;
static void CalculateThreshold(void);

static UWORD AtanAntiglitchFilter8KHz(UWORD uwValue, UWORD uwValue_1, SLONG slEncMechSpd, SLONG slEncAbsMechSpd, SLONG slEncPostnOL) ;
static UWORD FindWeightUnsigned(SLONG slSpeed, SWORD swSlope) ;
static void MotorKtEvaluation(void) ;

// #########################################################################
BOOL Be_BackEmfHandlerEncInit(ENCMGR_SPACEFEEDBACK * psFeedback, SBYTE * psbEncMgrFault)
{
    ULONG ulRetVal ;
    
    ulRetVal = BackEmfEncInit(psFeedback, psbEncMgrFault) ;
    if (ulRetVal)
    {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_BE_BACKEMF_FAIL, SYSTEMALARMS_SUBCODE_ENC_MAIN | ulRetVal, FALSE) ;
        /* Inizializzazione non è andata bene: non installo la funzione 8KHz e segnalo l'errore */
        return FALSE ;
    }
    else
    {    
        if(!TaskSched_AddBackgroundTask(&BackEmfHandlerBkGd)) // install background function
            return FALSE ; 
        else  
            return TaskSched_AddRTTask(&BackEmfHandler8KHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0) ; // install 8KHz function
    }
}

// #########################################################################
static ULONG BackEmfEncInit(ENCMGR_SPACEFEEDBACK *psEncOut, SBYTE * psbEncMgrFault)
{
    UWORD uwIdRamp;

    // ====== Input pointers
    sEmfEncRun.psBackEmfOut = psEncOut ;
    sEmfEncRun.psbEncMgrFault = psbEncMgrFault;


    // ====== Motor parameters
    if(sBe_EmfEncParam.uwIdOpenLoop>1000)
        uwIdRamp=1000;
    else
        uwIdRamp=sBe_EmfEncParam.uwIdOpenLoop;
    sEmfEncRun.uwPolePairs = sGlbMotorParameters.uwPoleNumbers / 2 ;
    sEmfEncRun.slMotorCurrentPeak=(SLONG)(sGlbMotorParameters.flCurrentPeak * 10000.0 * (FLOAT)uwIdRamp / 1000.0);
    sEmfEncRun.ulMechSteps = ((ULONG)(0UL - sEmfEncRun.uwPolePairs) / sEmfEncRun.uwPolePairs) + 1 ;
    sEmfEncRun.ulMechAdder = 0UL ; 


    // ====== Encoder Output
    INT64_ASSIGN(sEmfEncRun.psBackEmfOut->sEncData.sqPostn, 0L, 0UL) ;
    sEmfEncRun.psBackEmfOut->sEncData.slSpeed = 0L ;
    sEmfEncRun.psBackEmfOut->sEncData.slAccel = 0L ;
    sEmfEncRun.psBackEmfOut->uwElecAngle = 0 ;
    sEmfEncRun.psBackEmfOut->swElecSpeed = 0 ;

    // ====== Manager 
    sEmfEncRun.flags.b.bUseOPLTurns = TRUE ;         
    sEmfEncRun.swWeightEmf = 0x0000 ;
    sEmfEncRun.swWeightOPL = 0x7fff ;
    
    sEmfEncRun.flags.b.bOpenLoopOnly = sBe_EmfEncParam.flags.b.bOpenLoopOnly ; 

    sEmfEncRun.slHookMaxStep=sEmfEncRun.slMotorCurrentPeak/CURRENT_RAMP_TIME;
    if(sEmfEncRun.slHookMaxStep==0l)
        sEmfEncRun.slHookMaxStep=1;

    // runtime
    sEmfEncRun.ulAlarm2Send = 0 ;
    sEmfEncRun.sErrorSent.w = 0 ;
    sEmfEncRun.ubEmfEncStatus = EMFENC_RESET ;
//    sEmfEncRun.ubIqShift2Use = IQSHIFT_FULL ;
    sEmfEncRun.flags.b.bEmfSpeedMeasured = FALSE ;
    sEmfEncRun.uwEmfSpeedTimer = EMFSPEED_MEASURE_TIMEOUT ;

    // ---------
    // Kt Motore
    // ---------
    // imposto i limiti validi del Kt (per la stima a runtime)
    sEmfEncRun.sKtEval.flKtMin = sGlbMotorParameters.flKT * FLOAT_KT_PERCENTAGE_MIN ;
    sEmfEncRun.sKtEval.flKtMax = sGlbMotorParameters.flKT * FLOAT_KT_PERCENTAGE_MAX ;
    sBe_EmfEncDiagOut.ulMagicNumber = calcbasemagicnumber(sGlbMotorParameters.flKT);

    // ---------------------------------------------------------------------------
    // calcolo le percentuali (in float)
    sEmfEncRun.sPercentage.flSsnrlssSpeed         = (FLOAT)sBe_EmfEncParam.uwPercentage_SsnrlssSpeed         / HUNDREDPERCENT_FLOAT ;      
    sEmfEncRun.sPercentage.flSsnrlssThreshold1    = (FLOAT)sBe_EmfEncParam.uwPercentage_SsnrlssThreshold1    / HUNDREDPERCENT_FLOAT ;    
    sEmfEncRun.sPercentage.flSsnrlssThreshold0    = (FLOAT)sBe_EmfEncParam.uwPercentage_SsnrlssThreshold0    / HUNDREDPERCENT_FLOAT ;    
    sEmfEncRun.sPercentage.flEmfEncAGlitchFilter  = (FLOAT)sBe_EmfEncParam.uwEmfEncGlitchFilterPercentage    / HUNDREDPERCENT_FLOAT ;  
    sEmfEncRun.sPercentage.flBackOnTheFlyUsrSpeed = (FLOAT)sBe_EmfEncParam.uwPercentage_BackOnTheFlyUsrSpeed / HUNDREDPERCENT_FLOAT ;  
    sEmfEncRun.sPercentage.flBackOnTheFlyMinSpeed = (sEmfEncRun.sPercentage.flSsnrlssSpeed + sEmfEncRun.sPercentage.flSsnrlssSpeed * BACKONTHEFLYDELTASPEED / 100.0) ; 

    // ------------
    // stima del Kt
    // ------------
    sEmfEncRun.flags.b.bKtOneSecondTimerSet = FALSE ;
    sEmfEncRun.sKtEval.ulKtCounter = 0 ;    
    sEmfEncRun.sKtEval.uwOneSecBkgdCntr = 0 ;
    sEmfEncRun.sKtEval.flSQRootSum = 0.0 ;
    sEmfEncRun.sKtEval.flMechSpeedDeltaAngleSum = 0.0 ;
    sEmfEncRun.sKtEval.flIncrementoKt = 0.0 ;

        
    // ====== Diagnostic Output
    sBe_EmfEncDiagOut.uwSnsrlessState = SNSRLESS_DISABLED ;
    sBe_EmfEncDiagOut.slMechSpeedDeltaAngle = 0L ;
    sBe_EmfEncDiagOut.uwAntiglitchCounter = 0 ;
    sBe_EmfEncDiagOut.flValuedKt = sGlbMotorParameters.flKT ; // parto con il valore 'utente'

    CalculateThreshold();

    BackEmfEncOut8KHz(sEmfEncRun.ubEmfEncStatus) ; // reset the structures
    sEmfEncRun.flags.b.bNegativeSign = FALSE ;
    INT64_ASSIGN(sEmfEncRun.psBackEmfOut->sEncData.sqPostn, 0L, 0UL) ;  // reset position only at boot
    sEmfEncRun.uwSyncLostHysteresis = 0 ;

#if CFG_ENC_BEMF_DITEN
	sDitenRun.swTestCounter = 0 ;
	sDitenRun.flHookTimer   = 0.0 ;
#endif

    return FALSE ;
}

// #########################################################################
BOOL Be_ParametersCheck(void)
{
    BOOL bValid=TRUE;
    ULONG ulMagicNumber;
    
    if(sBe_EmfEncParam.uwPercentage_BackOnTheFlyUsrSpeed > HUNDREDPERCENT_INT)
    {
        ParChk_SignalValueError(PARCC_BEMF_BACKONTHEFLYUSRSPEED);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_BEMF_BACKONTHEFLYUSRSPEED);
    
    if(sBe_EmfEncParam.uwPercentage_SsnrlssSpeed == 0 || sBe_EmfEncParam.uwPercentage_SsnrlssSpeed > HUNDREDPERCENT_INT)
    {
        ParChk_SignalValueError(PARCC_BEMF_SENSORLESSSPEED);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_BEMF_SENSORLESSSPEED);
    
    if(sBe_EmfEncParam.uwPercentage_SsnrlssThreshold1 == 0 || sBe_EmfEncParam.uwPercentage_SsnrlssThreshold1 >= sBe_EmfEncParam.uwPercentage_SsnrlssSpeed)
    {
        ParChk_SignalValueError(PARCC_BEMF_SPEEDTHRESHOLD1);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_BEMF_SPEEDTHRESHOLD1);
    
    if(sBe_EmfEncParam.uwPercentage_SsnrlssThreshold0 == 0 || sBe_EmfEncParam.uwPercentage_SsnrlssThreshold0 >= sBe_EmfEncParam.uwPercentage_SsnrlssThreshold1)
    {
        ParChk_SignalValueError(PARCC_BEMF_SPEEDTHRESHOLD0);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_BEMF_SPEEDTHRESHOLD0);
    
    if(sBe_EmfEncParam.uwEmfEncGlitchFilterPercentage == 0 || sBe_EmfEncParam.uwEmfEncGlitchFilterPercentage >= sBe_EmfEncParam.uwPercentage_SsnrlssThreshold1)
    {
        ParChk_SignalValueError(PARCC_BEMF_GLITCHFILTER);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_BEMF_GLITCHFILTER);

    if((sBe_EmfEncParam.uwPercentage_BackOnTheFlyUsrSpeed != 0) && (sBe_EmfEncParam.uwPercentage_BackOnTheFlyUsrSpeed < sBe_EmfEncParam.uwPercentage_SsnrlssSpeed+BACKONTHEFLYDELTASPEED))
    {
        ParChk_SignalValueError(PARCC_BEMF_INVALIDBACKONTHEFLY);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_BEMF_INVALIDBACKONTHEFLY);

    ulMagicNumber=calcbasemagicnumber(sGlbMotorParameters.flKT);
    if(ulMagicNumber == 0 || ulMagicNumber > 0x10000)
    {   // fault: il numero magico deve essere SEMPRE compreso tra (zero ; 65535]
        // range radice quadrata (15bit): [0; 0x7fff]
        // range speed (31bit + segno):   [0; 0x7fffffff]
        // speed = numero magico * radice quadrata => numero magico = speed / radice quadrata => range numero magico: (0; 65535]
        ParChk_SignalValueError(PARCC_BEMF_INVMAGICNUMBER);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_BEMF_INVMAGICNUMBER);

    return bValid;
}

// #########################################################################
void Be_SetElecAngle(UWORD uwElecAngle)
{
        // $TEST$ per ora valido in quanto l'unica procedura EFS ritorna
        // sempre l'angolo pari a zero; prevedere procedure che a termine
        // EFS ritornano angolo qualsiasi
    sEmfEncRun.psBackEmfOut->uwDeltaElecAngle=uwElecAngle;
    sEmfEncRun.psBackEmfOut->ubStatus |= ENCMGR_ELE_ANGLE_VALID | ENCMGR_RELATIVE_VALID ;
}

// #########################################################################
void Be_SetAbsPos(SQWRD sqAbsPos)
{
    sEmfEncRun.psBackEmfOut->sqMechAbsPosOffset.lo = sqAbsPos.lo;
    sEmfEncRun.psBackEmfOut->sqMechAbsPosOffset.hi = sqAbsPos.hi;
    sEmfEncRun.psBackEmfOut->ubStatus |= ENCMGR_ABSMECHTURN_VALID ;
}

// #########################################################################
static ULONG calcbasemagicnumber(FLOAT flKT)
{
    // -------------
    // numero magico
    // -------------
    // Numero Magico = (1/Ke) * [(2^32 /8000) * (1/2PI)] * [(1/10) * (1/sqrt(2))]
    // Ke = Kt/sqrt(3)
    // 1 giro(incrementi)/secondo = 2^32 / 8000
    // 1/2PI = per passare da giri (incrementi)/ secondo a rad/secondo
    // 1/10 = perche' Vmotor e' in unita' interne (1e-1V)
    // 1/sqrt(2) = perche' la tensione motore in uscita dalla radice quadrate è il valore efficace, mentre a me serve l'RMS
    // sBe_EmfEncDiagOut.ulMagicNumber = (ULONG)(((SQRTOF3_FLOAT / sGlbMotorParameters.flKT) * RADSEC2INTERNALUNIT_FLOAT) / (10.0 * SQRTOF2_FLOAT)) ; 
    return (ULONG)(FLOAT_KT_CONVFACTOR / flKT) ; 
}

// #########################################################################
static BOOL BackEmfHandler8KHz(void)
{
#if CFG_ENC_BEMF_DITEN
    if(sBe_EmfEncIn.psPowerStageStatus->b.bVoltageEnabled)
    {   // PWM switched on, let's start the show..
    	Diten_8kHz() ;
        BackEmfMngrBackOnTheFly8KHz() ; // back on the fly/rotor direction
        BackEmfEncMechOut8KHz() ;
        BackEmfEncOut8KHz(sEmfEncRun.ubEmfEncStatus) ; // check what to do with the output
    }
    else
    {   //  keep Diten "resetted"

        sEmfEncRun.flags.b.bRotorZeroFound = FALSE ; //  rotor has not been yet directed
	    BackEmfEncOut8KHz(EMFENC_RESET) ; // reset
    }
#else
    if(sBe_EmfEncIn.psPowerStageStatus->b.bVoltageEnabled)
    {   // PWM switched on, let's start the show..
        // segno preso dal riferimento di velocita' solo quando il rotore e' dichiarato orientato
		// attenzione che non lo resetto perche' così con il rientro al volo dovrei rientrare già con il segno giusto
        if (sEmfEncRun.flags.b.bRotorZeroFound)
            sEmfEncRun.flags.b.bNegativeSign = (BOOL)(sBe_EmfEncIn.psRef->slSpeed & 0x80000000) ? TRUE : FALSE; 
        
        // absolute value of the mechanical speed (servo-1 value). 
        // Used for:
        // 1) antiglitch threshold ; 
        // 2) calculate weights ; 
        // 3) mix threshold .           
        sEmfEncRun.slAbsMechSpeed = labs(sEmfEncRun.psBackEmfOut->sEncData.slSpeed) ;  // sample i-1   

        BackEmfEncOpl8KHz() ;     // Open Loop contribute
        BackEmfEncEmf8KHz() ;     // Emf contribute     
        BackEmfMngrMix8KHz() ;    // find OpenLoop and Emf mix data 
        BackEmfMngrBackOnTheFly8KHz() ; // back on the fly/rotor direction       
        BackEmfEncMechOut8KHz() ; // calculate mechanical output
        BackEmfMngrFault8KHz() ;  // fault manager 
        BackEmfEncOut8KHz(sEmfEncRun.ubEmfEncStatus) ; // check what to do with the output
    }
    else
    {   // set the variables to allow back on the fly
        sEmfEncRun.flags.b.bRotorZeroFound = FALSE ; //  rotor has not been yet directed 
        sEmfEncRun.flags.b.bEmfSpeedMeasured = FALSE ;

        sEmfEncRun.uwSyncLostHysteresis = 0 ;
        sEmfEncRun.uwEmfSpeedTimer = EMFSPEED_MEASURE_TIMEOUT ; // set the timeout to measure speed
     
        BackEmfEncOut8KHz(EMFENC_RESET) ; // reset
    }
#endif // cfg_enc_bemf_diten
    return TRUE ;
}


// #########################################################################
static void BackEmfHandlerBkGd(void)
{
    (*sBe_EmfEncIn.psMotorHandlers->pfForceCommand)(MH_CMD_BACKEMFATANENABLE, 0l);

    // stima del Kt: stoppata o su parametro o quando deflusso
    if (!sBe_EmfEncParam.flags.b.bStopMotorKtEval)
        if (!sDflx_Out.flags.b.bDefluxActive)
            MotorKtEvaluation() ;

    // adattamento soglie in base a tensione dcbus e Kt motore 
    CalculateThreshold() ;
}


// #########################################################################
// =========================================================== 
// ======================== OPEN-LOOP ======================== 
// =========================================================== 
// == calculating OpenLoop acceleration, speed and position == 
// ===========================================================
//  Variabili Input che vengono dall'esterno (aggiornate al servo i):
//      sEmfEncIn->pslSpeedRef = riferimento di velocità (meccanica) dell'anello di controllo
//  
//  Variabili Input che vengono dal servo i-1:
//      sEmfEncRun.sOpl.slPostnOPL_1    = regressione della Posizione (elettrica)OpenLoop: ATTENZIONE!! è la PositionOut non la PositionOPL!!
//      sEmfEncRun.swElecEmfEncTurns = numero di giri elettrici necessari per ricostruire la posizione elettrica OpenLoop.
//      
//  Variabili Output: 
//      sEmfEncRun.sOpl.slSpeedOPL = velocita' (meccanica) OpenLoop 
//      sEmfEncRun.sOpl.slPostnOPL = posizione (elettrica) OpenLoop 
//      
//  Parametri:
//      sEmfEncParam.uwMotorPolePairs = coppie polari motore  
static void BackEmfEncOpl8KHz(void)
{   // speedOPL = SpeedRef (open loop uses reference speed as open loop speed)
    sEmfEncRun.sOpl.slSpeedOPL = sBe_EmfEncIn.psRef->slSpeed ; 
    
    // positionOPL = electrical position
    sEmfEncRun.sOpl.slPostnOPL = sEmfEncRun.sOpl.slPostnOPL_1 + ((sEmfEncRun.sOpl.slSpeedOPL * sEmfEncRun.uwPolePairs + 0x80) >> 8) + ((SLONG)sEmfEncRun.swElecEmfEncTurns << 24) ; 
}


// #########################################################################
// ===========================================================
// =========================== EMF ===========================
// ===========================================================
// ====== calculating emf angle, speed and acceleration ======
// ===========================================================
//  Variabili che vengono dall'esterno (aggiornate al servo i):
//      sMotor.swBackEmfAtan, sMotor.swBackEmfSqrt
//      sEmfEncRun.sOpl.slSpeedOPL = stSpeed.slRef = mi serve per mettere il segno alla velocita' emf
//  
//  Valori che vengono dal servo i-1:
//      slEmfAbsSpdFiltered = serve per il filtro
//      sEmfEncOut.slMechSpeed = velocita' meccanica in uscita dal modulo backemf e che entra nell'anello di controllo.
//   
//  Variabili di scambio Manager<->Encoder:     
//      sEmfEncRun.sEmf.slAbsMechSpeed = valore assoluto di sEmfEncOut.slMechSpeed (aggiornata al servo i-1)
//                                        E' usata per attivare il filtro antiglitch, per attivare le soglie della macchina a
//                                        stati sensorless (OL; MIX; EMF), per calcolare i pesi         
//      sEmfEncRun.sEmf.slSpeedEmf = velocità  (meccanica) EMF
//      sEmfEncRun.sEmf.slPostnEmf = posizione (elettrica) EMF
//      
//      
//  Parametri:
//      sEmfEncParam.uwMagicNumber = Kt del motore (aggiornato lentamente in background)
//      sEmfEncParam.uwMotorPolePairs = coppie polari motore  
static void BackEmfEncEmf8KHz(void)
{   // Emf Electrical Angle
    BOOL    bCurrAngleL = FALSE, bCurrAngleH = FALSE ;
    BOOL    bAngleCmpHL = FALSE, bAngleCmpLH = FALSE ;
    UWORD   uwNewAtanAngle ;
    SLONG   slAngleCarry = 0 ; // Overflow di giro elettrico emf
    SLONG   slAngleFeedF ;    
    ULONG   ulAngleEmf_1, ulAngleNoFFW ;
 
    // ------------------------------------------------------------------------ 
    // ------------------------------ Atan Angle ------------------------------ 
    // ------------------------------------------------------------------------  
    uwNewAtanAngle = sBe_EmfEncIn.psBackEmfData->uwAtanAngle ; 

      if (sEmfEncRun.flags.b.bNegativeSign)
        uwNewAtanAngle += 0x8000 ; // SpeedRef < 0

    // Atan antiglitch filter
    if (sBe_EmfEncParam.flags.b.bUseAntiglitch)
        sEmfEncRun.sEmf.uwEmfAtanAngle = AtanAntiglitchFilter8KHz(uwNewAtanAngle, sEmfEncRun.sEmf.uwEmfAtanAngle, sEmfEncRun.psBackEmfOut->sEncData.slSpeed, sEmfEncRun.slAbsMechSpeed, sEmfEncRun.sOpl.slPostnOPL) ; 
    else
        sEmfEncRun.sEmf.uwEmfAtanAngle = uwNewAtanAngle ;

    // ------------------------------------------------------------------------
    // ---------------------- EMF Position  (electrical) ----------------------
    // ------------------------------------------------------------------------
    sEmfEncRun.sEmf.slEmfPos = (SLONG)((ULONG)(sEmfEncRun.sEmf.slEmfPos & 0xffff0000) + (ULONG)sEmfEncRun.sEmf.uwEmfAtanAngle) ; // turns number + new EmfAngle (filtered)

    if (sEmfEncRun.sEmf.uwEmfAtanAngle < 0x4000)      // 1st quadrant
        bCurrAngleL = TRUE ;
    else if (sEmfEncRun.sEmf.uwEmfAtanAngle > 0xc000) // 4th quadrant
        bCurrAngleH = TRUE ;    

    bAngleCmpHL = sEmfEncRun.sEmf.flags.b.bAngleH_1 && bCurrAngleL ; // turn overflow clockwise    
    bAngleCmpLH = sEmfEncRun.sEmf.flags.b.bAngleL_1 && bCurrAngleH ; // turn overflow anticlockwise
    sEmfEncRun.sEmf.flags.b.bAngleL_1 = bCurrAngleL ; // saving for next servo
    sEmfEncRun.sEmf.flags.b.bAngleH_1 = bCurrAngleH ; // saving for next servo

    if (bAngleCmpHL)
    {   // clockwise
        sEmfEncRun.sEmf.slEmfPos += 0x10000 ; // add 1 turn

        if (sEmfEncRun.sEmf.slEmfPos & 0xff800000) 
        { // 128 turns reached
            sEmfEncRun.sEmf.slEmfPos &= 0x0000ffff ;
            slAngleCarry = 0x08000000 ; // 0x08000000 = 0x00800000 * 0x10
        }
    } 
    
    if (bAngleCmpLH)
    {   // anticlockwise
        sEmfEncRun.sEmf.slEmfPos -= 0x10000 ; // subtract 1 turn

        if (sEmfEncRun.sEmf.slEmfPos < 0)
        {
            sEmfEncRun.sEmf.slEmfPos += 0x00800000 ;
            slAngleCarry = 0xf8000000 ; // 0xf8000000 = 0xff800000 * 0x10
        }                
    } 

    // filtering Emf Position
    sEmfEncRun.sEmf.slPrecAngleO = ((sEmfEncRun.sEmf.slPrecAngleO - slAngleCarry) * 15)/16 + sEmfEncRun.sEmf.slEmfPos ;
    ulAngleNoFFW = (ULONG)sEmfEncRun.sEmf.slPrecAngleO ;

    // Compute feed forward
    slAngleFeedF = ((SLONG)(32 * ulAngleNoFFW) - (SLONG)(32 * sEmfEncRun.sEmf.ulAngles_1[sEmfEncRun.sEmf.uwIndex]))/32 ;
    sEmfEncRun.sEmf.ulAngles_1[sEmfEncRun.sEmf.uwIndex] = ulAngleNoFFW ;
    sEmfEncRun.sEmf.uwIndex = ++sEmfEncRun.sEmf.uwIndex & 15 ;
    
    ulAngleEmf_1 = sEmfEncRun.sEmf.ulAngleEmf ;
    sEmfEncRun.sEmf.ulAngleEmf = (ulAngleNoFFW << 12) + ((ULONG)slAngleFeedF << 12) ;     
        
    if ( (                ulAngleEmf_1 & 0x80000000) &&  (              ulAngleEmf_1 & 0x40000000) &&
          !(sEmfEncRun.sEmf.ulAngleEmf & 0x80000000) && !(sEmfEncRun.sEmf.ulAngleEmf & 0x40000000)   ) 
    {    
        sEmfEncRun.sEmf.swTurnsEmf++ ; /* Clockwise Overflow */
    } 
    else if (!(              ulAngleEmf_1 & 0x80000000) && !(              ulAngleEmf_1 & 0x40000000) &&
              (sEmfEncRun.sEmf.ulAngleEmf & 0x80000000) &&  (sEmfEncRun.sEmf.ulAngleEmf & 0x40000000)   ) 
    {            
        sEmfEncRun.sEmf.swTurnsEmf-- ; /* Anticlockwise Overflow */
    }

    sEmfEncRun.sEmf.swTurnsEmf += sEmfEncRun.swElecEmfEncTurns ;
    sEmfEncRun.sEmf.slPostnEmf  = ((SLONG)sEmfEncRun.sEmf.swTurnsEmf << 24) + (SLONG)(sEmfEncRun.sEmf.ulAngleEmf >> 8) ;
    // ------------------------------------------------------------------------


    // ------------------------------------------------------------------------
    // ------------------------ EMF Speed (mechanical) ------------------------
    // ------------------------------------------------------------------------
    //$TEST$ "crs: se Kt < 0.16, allora overflow (=> Magic Number > 65535). Devo arrivare a Kt = 0.01!!"
    sEmfEncRun.sEmf.slEmfAbsSpdFiltered = (SLONG)(sBe_EmfEncDiagOut.ulMagicNumber * sBe_EmfEncIn.psBackEmfData->uwVmotor) ; // mechanical speed in unified unit = filtered Square Root * MagicNumber
//    _sint64_mul_32_16(&sqEmfAbsSpeed, &(sBe_EmfEncDiagOut.ulMagicNumber), &(sBe_EmfEncIn.psBackEmfData->uwSQRooT)) ; // mechanical speed in unified unit = filtered Square Root * MagicNumber
//    sEmfEncRun.sEmf.slEmfAbsSpdFiltered = INT64_MLONG(sqEmfAbsSpeed) ;  

    if (sEmfEncRun.flags.b.bNegativeSign)
        sEmfEncRun.sEmf.slSpeedEmf = -sEmfEncRun.sEmf.slEmfAbsSpdFiltered ; // SpeedRef < 0 
    else                                                 
        sEmfEncRun.sEmf.slSpeedEmf =  sEmfEncRun.sEmf.slEmfAbsSpdFiltered ; // SpeedRef > 0   
    // ------------------------------------------------------------------------ 


    // ------------------------------------------------------------------------
    // --------------------------- EMF acceleration ---------------------------
    // ------------------------------------------------------------------------
    sEmfEncRun.sEmf.slAccelEmf = 0L ; /* for the moment zero */     
    // ------------------------------------------------------------------------
} 


// ######################################################################### 
// ========================================================================= 
// ========= mixing openloop and emf position, speed, acceleration ========= 
// ========================================================================= 
static void BackEmfMngrMix8KHz(void)
{
    if (sEmfEncRun.slAbsMechSpeed < sEmfEncRun.slSpeedThreshold0)
    {   // at very low speed do not use weights, use open-loop 
        // TurnsEmf    = TurnsOpl (keeping OL Position and Emf 
        //                         Position sinchronized)      
        // PositionMix = Position     OpenLoop (electrical)    
        // SpeedMix    = Speed        OpenLoop (mechanical)    
        // AccelMix    = Acceleration OpenLoop (=0!!)          
        // IdRef       = 100% if Id                            
        // IqLimit     = Iq/16                              

        sEmfEncRun.flags.b.bUseOPLTurns = TRUE ;         
        sEmfEncRun.swWeightEmf = 0x0000 ;
        sEmfEncRun.swWeightOPL = 0x7fff ;         
//        sEmfEncRun.ubIqShift2Use = IQSHIFT_THRESHOLD_0 ;
        sBe_EmfEncDiagOut.uwSnsrlessState = SNSRLESS_THRESHOLD_0 ;

    }
    else if ((sEmfEncRun.slAbsMechSpeed >= sEmfEncRun.slSpeedThreshold0) &&
             (sEmfEncRun.slAbsMechSpeed <  sEmfEncRun.slSpeedThreshold1)   )
    {   // at very low speed do not use weights, use open-loop 
        // Emf Position data and OpenLoop Position data should 
        // be sinchronized...                                  
        // PositionOut = Position     OpenLoop (electrical)    
        // SpeedOut    = Speed        OpenLoop (mechanical)    
        // AccelOut    = Acceleration OpenLoop (=0!!)          
        // IdRef       = 100% if Id                            
        // IqLimit     = Iq/8                               
         
        sEmfEncRun.flags.b.bUseOPLTurns = FALSE ;
        sEmfEncRun.swWeightEmf = 0x0000 ;
        sEmfEncRun.swWeightOPL = 0x7fff ; 
//        sEmfEncRun.ubIqShift2Use = IQSHIFT_THRESHOLD_1 ;
        sBe_EmfEncDiagOut.uwSnsrlessState = SNSRLESS_THRESHOLD_1 ;        
    }                         
    else if ((sEmfEncRun.slAbsMechSpeed >= sEmfEncRun.slSpeedThreshold1) &&
             (sEmfEncRun.slAbsMechSpeed <  sEmfEncRun.slSpeedSensorless)   )
    {   // slSpeedThreshold1 <= slAbsMechSpeed < slMechSpeedSensorless        
        // mixing OpenLoop and Emf data using weights
        // PositionOut = Kemf * Position_Emf + Kol * Position_OL (electrical)    
        // SpeedOut    = Kemf * Speed_Emf    + Kol * Speed_OL    (mechanical)    
        // AccelOut    = =0
        // IdRef       = Kol * Id                         
        // IqLimit     = Iq/2                                        
        sEmfEncRun.flags.b.bUseOPLTurns = FALSE ;
        // la pendenza della retta(swSpeedSlope) deve essere SEMPRE positiva!!!"
        sEmfEncRun.swWeightEmf = FindWeightUnsigned(sEmfEncRun.slAbsMechSpeed, sEmfEncRun.swSpeedSlope) ; // find the weights: always [0; 2^15-1]
        sEmfEncRun.swWeightOPL = 0x7fff - sEmfEncRun.swWeightEmf ;    
//        sEmfEncRun.ubIqShift2Use = IQSHIFT_MIXING ;
        sBe_EmfEncDiagOut.uwSnsrlessState = SNSRLESS_MIXING ; 
        
        if (sEmfEncRun.flags.b.bOpenLoopOnly)
        {   // only OpenLoop
            sEmfEncRun.swWeightEmf = 0x0000 ;
            sEmfEncRun.swWeightOPL = 0x7fff ;
            sBe_EmfEncDiagOut.uwSnsrlessState = SNSRLESS_MIXING_OL ;
        }
    }
    else 
    {   // slAbsMechSpeed >= slMechSpeedSensorless        
        // at sensorless speed do not use weights, use Emf
        // PositionOut = Position     Emf (electrical)    
        // SpeedOut    = Speed        Emf (mechanical)    
        // AccelOut    = Acceleration Emf (=0!!)          
        // IdRef       = 0% if Id                         
        // IqLimit     = Iq                            
    
        sEmfEncRun.flags.b.bUseOPLTurns = FALSE ;
        sEmfEncRun.swWeightEmf = 0x7fff ;
        sEmfEncRun.swWeightOPL = 0x0000 ;
//        sEmfEncRun.ubIqShift2Use = IQSHIFT_FULL ;
        sBe_EmfEncDiagOut.uwSnsrlessState = SNSRLESS_FULL ;
        
        if (sEmfEncRun.flags.b.bOpenLoopOnly) 
        {   // only OpenLoop
            sEmfEncRun.swWeightEmf = 0x0000 ;
            sEmfEncRun.swWeightOPL = 0x7fff ;
            sBe_EmfEncDiagOut.uwSnsrlessState = SNSRLESS_FULL_OL ; 
        } 
    }
}


// ######################################################################### 
// ========================================================================= 
// =========================== mechanical output =========================== 
// ========================================================================= 
//  Variabili di input (aggiornate al servo i):
//      slPostnOPL = Posizione     Elettrica in uscita dal modulo OpenLoop
//      slSpeedOPL = Velocità      Elettrica in uscita dal modulo OpenLoop
//      slAccelOPL = Accelerazione Elettrica in uscita dal modulo OpenLoop
//      slPostnEmf = Posizione     Elettrica in uscita dal modulo Emf
//      slSpeedEmf = Velocità      Elettrica in uscita dal modulo Emf
//      slAccelEmf = Accelerazione Elettrica in uscita dal modulo Emf
//
//      uwWeightOPL, uwWeightEmf = Pesi dal BackEmf Manager
//      bUseOPLTurns = flag (dal BackEmf Manager) per imporre sincronizzazione OpenLoop ed Emf
//
//  Variabili necessarie per le regressioni:
//      ulElecAngle = serve per calcolare swElecEmfEncTurns e per aggiornare ulMechAdder
//      
//  Variabili di Output: 
//      swTurnsOut            = numero di giri elettrici per sincronizzare posizione elettrica EMF ed OL
//      sqMechAbsPosition.hi  = numero di giri meccanici
//      sqMechAbsPosition.lo  = angolo meccanico 
//      slMechSpeed           = velocita' meccanica (ottenuta dalla radice quadrata)
//      slMechSpeedDeltaAngle = velocita' meccanica (ottenuta dal delta degli angoli)
//      slMechAccel           = accelerazione meccanica (sempre zero!)
//      uwElecAngle           = angolo elettrico da mandare all'FPGA
//
//  Parametri necessari:
//      uwMotorPolePairs = coppie polari motore
//      ulMechSteps      = step necessari per fare un overflow di giro
//      ulMechAdder      = overflow di giro
//      ulMechOffset     = offset 
void BackEmfEncMechOut8KHz(void)
{
#if CFG_ENC_BEMF_DITEN
	ULONG ulMechEmfEncAngle_1 = sEmfEncRun.psBackEmfOut->sEncData.sqPostn.lo ;

	// all necessary information from observer is set as a BackEmf feedback out.
	sEmfEncRun.psBackEmfOut->sEncData.sqPostn.hi = sBe_EmfEncDiagOut.diten_obs_pll_out.revolutionCounter ;
	sEmfEncRun.psBackEmfOut->sEncData.sqPostn.lo = sBe_EmfEncDiagOut.diten_obs_pll_out.mechThet;
	sEmfEncRun.psBackEmfOut->sEncData.slSpeed    = sBe_EmfEncDiagOut.diten_obs_pll_out.mechSpeed;
	sEmfEncRun.psBackEmfOut->uwElecAngle         = sBe_EmfEncDiagOut.diten_obs_pll_out.elThet;
	sEmfEncRun.psBackEmfOut->swElecSpeed         = sBe_EmfEncDiagOut.diten_obs_pll_out.elSpeed;
	sEmfEncRun.psBackEmfOut->sEncData.slAccel    = 0; // for the moment no acceleration

    // SpeedEmf from DeltaAngle
    sBe_EmfEncDiagOut.slMechSpeedDeltaAngle = (SLONG)(sEmfEncRun.psBackEmfOut->sEncData.sqPostn.lo - ulMechEmfEncAngle_1) ;
#else
	ULONG ulMechEmfEncAngle ;
    ULONG ulElecEmfEncAngle_1, ulMechEmfEncAngle_1 ;

    SQWRD sqPostnOPL, sqPostnEmf, sqPostnMix ;
    SQWRD sqSpeedOPL, sqSpeedEmf, sqSpeedMix ;
    SLONG slPostnMix, slSpeedMix ;

    // -------------------------------------------------------------------------  
    // --------- mixing openloop and emf position, speed, acceleration ---------  
    // -------------------------------------------------------------------------       
    if (sEmfEncRun.flags.b.bUseOPLTurns) // force Emf electrical turns to be equal to OpenLoop turns (to sinchronize Emf and OL)
        sEmfEncRun.sEmf.swTurnsEmf = (SWORD)(sEmfEncRun.sOpl.slPostnOPL >> 24) ;    

    if (sEmfEncRun.swWeightOPL == 0x7fff)
    {
        slPostnMix = sEmfEncRun.sOpl.slPostnOPL ;
        slSpeedMix = sEmfEncRun.sOpl.slSpeedOPL ;
    }
    else if (sEmfEncRun.swWeightEmf == 0x7fff)
    {
        slPostnMix = sEmfEncRun.sEmf.slPostnEmf ;
        slSpeedMix = sEmfEncRun.sEmf.slSpeedEmf ;
    }
    else
    {   // slPostnMix = weighted_PositionEmf + weighted_PositionOL
        _sint64_mul_32_16(&sqPostnOPL, &sEmfEncRun.sOpl.slPostnOPL, &sEmfEncRun.swWeightOPL) ;
        _sint64_mul_32_16(&sqPostnEmf, &sEmfEncRun.sEmf.slPostnEmf, &sEmfEncRun.swWeightEmf) ;    
        _sint64_add(&sqPostnMix, &sqPostnOPL, &sqPostnEmf) ;
        slPostnMix = (SLONG)(INT64_MLONG(sqPostnMix)) << 1 ;
    
        // SpeedOut = weighted_SpeedEmf + weightedSpeedOL
        _sint64_mul_32_16(&sqSpeedOPL, &sEmfEncRun.sOpl.slSpeedOPL, &sEmfEncRun.swWeightOPL) ;
        _sint64_mul_32_16(&sqSpeedEmf, &sEmfEncRun.sEmf.slSpeedEmf, &sEmfEncRun.swWeightEmf) ;
        _sint64_add(&sqSpeedMix, &sqSpeedOPL, &sqSpeedEmf) ;
        slSpeedMix = (SLONG)(INT64_MLONG(sqSpeedMix)) << 1 ;
    }

    // regressions for OpenLoop 
    sEmfEncRun.sOpl.slPostnOPL_1 = slPostnMix ; // OKKIO!!! NON e' slPostnOPL!!!!!
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    // ---------- turns number  to correct EMF and OL turns overflow ----------
    // ------------------------------------------------------------------------
    ulElecEmfEncAngle_1 = sEmfEncRun.ulElecEmfEncAngle ; 
    sEmfEncRun.ulElecEmfEncAngle = (ULONG)slPostnMix << 8 ;

    if ( (         ulElecEmfEncAngle_1 & 0x80000000) &&  (         ulElecEmfEncAngle_1 & 0x40000000) &&
        !(sEmfEncRun.ulElecEmfEncAngle & 0x80000000) && !(sEmfEncRun.ulElecEmfEncAngle & 0x40000000)   ) 
    { // Clockwise Overflow
        sEmfEncRun.swElecEmfEncTurns = -1 ;                    
        sEmfEncRun.ulMechAdder += sEmfEncRun.ulMechSteps ; // update adder
    } 
    else if (!(         ulElecEmfEncAngle_1 & 0x80000000) && !(         ulElecEmfEncAngle_1 & 0x40000000) &&
              (sEmfEncRun.ulElecEmfEncAngle & 0x80000000) &&  (sEmfEncRun.ulElecEmfEncAngle & 0x40000000)   ) 
    { // Anticlockwise Overflow 
        sEmfEncRun.swElecEmfEncTurns = +1 ;                                
        sEmfEncRun.ulMechAdder -= sEmfEncRun.ulMechSteps ; // update adder */
    }
    else
        sEmfEncRun.swElecEmfEncTurns = 0 ;
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    // ---------- Mechanical Outputs (Position, Speed, Acceleration) ----------
    // ------------------------------------------------------------------------
    // saving previous mechanical data necessary to calculate turns number, speed and acceleration
    ulMechEmfEncAngle_1 = sEmfEncRun.psBackEmfOut->sEncData.sqPostn.lo ;    

    // calculating mechanical angle
    ulMechEmfEncAngle = sEmfEncRun.ulElecEmfEncAngle/sEmfEncRun.uwPolePairs + sEmfEncRun.ulMechAdder ;

    // calculating mechanical turns number
    if ( (ulMechEmfEncAngle_1 & 0x80000000) &&  (ulMechEmfEncAngle_1 & 0x40000000) &&
        !(ulMechEmfEncAngle   & 0x80000000) && !(ulMechEmfEncAngle   & 0x40000000)   ) 
    {    
        sEmfEncRun.psBackEmfOut->sEncData.sqPostn.hi++ ; // Clockwise Overflow
    } 
    else if (!(ulMechEmfEncAngle_1 & 0x80000000) && !(ulMechEmfEncAngle_1 & 0x40000000) &&
              (ulMechEmfEncAngle   & 0x80000000) &&  (ulMechEmfEncAngle   & 0x40000000)   ) 
    {    
        sEmfEncRun.psBackEmfOut->sEncData.sqPostn.hi-- ; // Anticlockwise Overflow
    }

    // SpeedEmf from DeltaAngle: it is zero ONLY when 1) backemf is disabled; 2) the rotor is directing itself
    sBe_EmfEncDiagOut.slMechSpeedDeltaAngle = (SLONG)(ulMechEmfEncAngle - ulMechEmfEncAngle_1) ; 

    sEmfEncRun.psBackEmfOut->sEncData.sqPostn.lo = ulMechEmfEncAngle ;
    sEmfEncRun.psBackEmfOut->sEncData.slSpeed = slSpeedMix ;

    if((sBe_EmfEncDiagOut.uwSnsrlessState == SNSRLESS_FULL) && (sBe_EmfEncParam.flags.b.bUseDeltaAngleSpeed))
        sEmfEncRun.psBackEmfOut->sEncData.slSpeed = sBe_EmfEncDiagOut.slMechSpeedDeltaAngle ;

    sEmfEncRun.psBackEmfOut->sEncData.slAccel = 0l ;
    sEmfEncRun.psBackEmfOut->uwElecAngle = HIWORD(sEmfEncRun.ulElecEmfEncAngle) + sEmfEncRun.psBackEmfOut->uwDeltaElecAngle ;

    /* -- Current Electrical Speed  = MechSpeed * MotorPolePairs -- */
    sEmfEncRun.psBackEmfOut->swElecSpeed = (SWORD)_sint32_scale_32(sEmfEncRun.psBackEmfOut->sEncData.slSpeed, sEmfEncRun.uwPolePairs) ;
#endif
}


// ######################################################################### 
// ========================================================================= 
// ================= Is it possible to reenter on the fly? ================= 
// ========================================================================= 
static void BackEmfMngrBackOnTheFly8KHz(void)
{ 
#if (!CFG_ENC_BEMF_DITEN)
	// trying to load all the filters while the drive is enabling
    // Softstart NOT ended or Application NOT enabled: exit and keep output value to zero   
    if (!(sBe_EmfEncIn.psPowerStageStatus->b.bFullyActive && sBe_EmfEncIn.psPowerStageStatus->b.bReferenceEnabled))
    {
        sEmfEncRun.flags.b.bEmfSpeedMeasured = FALSE ;
        sEmfEncRun.ubEmfEncStatus = EMFENC_ZEROOUT ; // keep sensor output informations to zero  
        sEmfEncRun.uwEmfSpeedTimer = EMFSPEED_MEASURE_TIMEOUT ; // set the timeout to measure speed
        return ;
    }

    if (!sEmfEncRun.flags.b.bEmfSpeedMeasured)
    {
        if ( --sEmfEncRun.uwEmfSpeedTimer == 0 )
        {   // time is up! it's necessary to take the decision:
            // to direct or not to not direct the rotor?  
            sEmfEncRun.flags.b.bEmfSpeedMeasured = TRUE ;           // no more speed measurement will be done
            sEmfEncRun.uwEmfSpeedTimer = EMFSPEED_MEASURE_TIMEOUT ; // reload timer

//            if (sEmfEncRun.slAbsMechSpeed > sEmfEncRun.slBackOnTheFlyUsrSpeed)
            if ((sEmfEncRun.sEmf.slEmfAbsSpdFiltered > sEmfEncRun.slBackOnTheFlyUsrSpeed) && (!sEmfEncRun.flags.b.bOpenLoopOnly) && (sEmfEncRun.slBackOnTheFlyUsrSpeed != 0))
            {   // speed from SQRT > BackOnTheFlySpeedThreshold: it is possible to re-enable while the motor is turning
                // only if NOT in openloop and BackOnTheFlyUserSpeed <> 0 (0 = back on the fyl disabled)
                sEmfEncRun.ubEmfEncStatus = EMFENC_OK ;     // allow sensor output informations
                sEmfEncRun.flags.b.bRotorZeroFound = TRUE ; // inform the world outside that it's not necessary to direct rotor: EMF speed is high enough
                sEmfEncRun.swWeightOPL = 0x0000 ;
                sEmfEncRun.swWeightEmf = 0x7fff ;           // use directly EMF data 
            }
            else
            {   // necessary to direct rotor
                sEmfEncRun.ubEmfEncStatus = EMFENC_ZEROOUT ; // keep sensor output informations to zero
                sEmfEncRun.flags.b.bRotorZeroFound = FALSE ; // inform the world outside that it's the rotor has not been yet directed
            }
            return ; // mi serve un tick di ritardo per l'encoder manager ed il generatore di rampe
        }
        else
        {    // wait to measure mechanical speed and load all the emf/OL filters 
            sEmfEncRun.flags.b.bEmfSpeedMeasured = FALSE ;  // still waiting to evaluate the speed
            sEmfEncRun.ubEmfEncStatus = EMFENC_ZEROOUT ;    // keep sensor output informations to zero
            return ;
        }    
    }
#endif // !cfg_enc_bemf_diten
    if(!sEmfEncRun.flags.b.bRotorZeroFound) // this flag could be set TRUE only or by BackOnTheFly or DirectSensorless
    { 
        if(sEmfEncRun.psBackEmfOut->ubStatus & ENCMGR_ELE_ANGLE_VALID)
        {
            sEmfEncRun.ubEmfEncStatus = EMFENC_ZEROOUT ;       // start from ZERO
            sEmfEncRun.flags.b.bRotorZeroFound = TRUE ; // declare rotor directed
        }
        else
        {
            sEmfEncRun.ubEmfEncStatus = EMFENC_RESET ; // reset all the variables (filters and output)
            sEmfEncRun.flags.b.bRotorZeroFound = FALSE ; // inform the world outside that it's the rotor has not been yet directed
        }

        return ;
    }    
}

// #########################################################################
// =========================================================================
// =========================== Manage the errors ===========================
// =========================================================================
static void BackEmfMngrFault8KHz(void)
{
    if (bSysStatAlarmsReset)
    {   // clear errors
        if(sEmfEncRun.sErrorSent.w || *sEmfEncRun.psbEncMgrFault)
            sEmfEncRun.sErrorSent.w = 0 ;
        sEmfEncRun.ulAlarm2Send = 0 ;
    }
    // ---------------------------------------------------------------------------------------------------
    // check if the synchronization has been lost ONLY when softstart has ended AND references are enabled
    // ---------------------------------------------------------------------------------------------------
    // slSpeedRef = Control Loop Reference Speed                         
    // slSpeedEmf = SpeedEmf obtained by square root (with SpeedRef sign)
    // Control Enabled only in sensorless mix and full
    if((sBe_EmfEncIn.psPowerStageStatus->b.bFullyActive && sBe_EmfEncIn.psPowerStageStatus->b.bReferenceEnabled) && sEmfEncRun.flags.b.bRotorZeroFound &&
       (labs(sBe_EmfEncIn.psRef->slSpeed - sEmfEncRun.sEmf.slSpeedEmf) > (sEmfEncRun.slSpeedSensorless + (labs(sBe_EmfEncIn.psRef->slSpeed) >> 2))) &&
       (sEmfEncRun.swWeightEmf != 0x0000) && (!sEmfEncRun.flags.b.bOpenLoopOnly) && 
       (!sEmfEncRun.sErrorSent.b.bSyncLost))
    {   
        if (sEmfEncRun.uwSyncLostHysteresis == 1)
        {   // SyncLost
            sEmfEncRun.sErrorSent.b.bSyncLost = TRUE ;       
            sEmfEncRun.ubEmfEncStatus = EMFENC_FAULT ;
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_BE_BACKEMF_FAIL, SYSTEMALARMS_SUBCODE_BACKEMF_SYNCLOST, sBe_EmfEncIn.psPowerStageStatus->b.bFullyActive) ; 
        }
        else
            sEmfEncRun.uwSyncLostHysteresis++ ;          
    }
    else
    {   // alarms hysteresis
        if ((BOOL)sEmfEncRun.uwSyncLostHysteresis)
            sEmfEncRun.uwSyncLostHysteresis-- ;
    }

    // ----------------------------------------
    // check if the antiglitch is always active
    // ----------------------------------------
    if ((sBe_EmfEncIn.psPowerStageStatus->b.bFullyActive && sBe_EmfEncIn.psPowerStageStatus->b.bReferenceEnabled) &&
        (sBe_EmfEncDiagOut.uwAntiglitchCounter > sBe_EmfEncParam.ulEmfEncGlitchFaultLimit) && 
        (sBe_EmfEncParam.ulEmfEncGlitchFaultLimit != 0) && 
        (!sEmfEncRun.sErrorSent.b.bAntiGlitch))
    {   // antiglitch sempre attivo..    
        sEmfEncRun.sErrorSent.b.bAntiGlitch = TRUE ;
        sEmfEncRun.ubEmfEncStatus = EMFENC_FAULT ;
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_BE_BACKEMF_FAIL, SYSTEMALARMS_SUBCODE_BACKEMF_AGLITCH, sBe_EmfEncIn.psPowerStageStatus->b.bFullyActive) ;
    }
}


// #########################################################################
// =========================================================================
// ========================== BackEmf Encoder Out ==========================
// =========================================================================
static void BackEmfEncOut8KHz(UWORD uwStatus)
{
    switch(uwStatus)
    {
        case EMFENC_RESET :   
            BackEmfEncResetOut() ;
            if(sBe_EmfEncIn.psPowerStageStatus->b.bVoltageEnabled)
              sEmfEncRun.psBackEmfOut->ubStatus = ENCMGR_EFS_READY; // allow EFS run
            else
              sEmfEncRun.psBackEmfOut->ubStatus = ENCMGR_NULL ;
            break ;
    
        case EMFENC_ZEROOUT :
            BackEmfEncZeroOut() ;
//            sEmfEncRun.psBackEmfOut->ubStatus = ENCMGR_NULL ;

            if(sEmfEncRun.flags.b.bRotorZeroFound)
                sEmfEncRun.ubEmfEncStatus = EMFENC_OK ; // rotor has been declare directed, change emf internal status
            break ;

        case EMFENC_FAULT :
            BackEmfEncResetOut() ;
//            BackEmfEncZeroOut() ;         
            sEmfEncRun.psBackEmfOut->ubStatus = ENCMGR_FATAL_FAULT ;
            break ;
        
        case EMFENC_OK :
            sEmfEncRun.psBackEmfOut->ubStatus = (ENCMGR_RELATIVE_VALID | ENCMGR_EFS_READY | ENCMGR_ELE_ANGLE_VALID) ;
            break ;

        default:
            assert(FALSE) ;
    }
}


// #########################################################################
static void BackEmfEncResetOut(void)
{
    // reset all the variables
    UWORD uwI ;
    
    // Emf: angolo Emf
    sEmfEncRun.sEmf.uwEmfAtanAngle = 0 ;
    sEmfEncRun.sEmf.slEmfPos = 0 ;
    sEmfEncRun.sEmf.flags.b.bAngleL_1 = 0 ;
    sEmfEncRun.sEmf.flags.b.bAngleH_1 = 0 ;
    sEmfEncRun.sEmf.slPrecAngleO = 0 ;
    
    for (uwI = 0; uwI < 16; uwI++)
        sEmfEncRun.sEmf.ulAngles_1[uwI] = 0 ;
        
    sEmfEncRun.sEmf.uwIndex = 0 ;
                
    // Emf: Output Emf
    sEmfEncRun.sEmf.ulAngleEmf = 0 ;
    sEmfEncRun.sEmf.swTurnsEmf = 0 ;
    sEmfEncRun.sEmf.slEmfAbsSpdFiltered = 0 ;

    // aggiunto questi reset per evitare salti ad una successiva riabilitazione
    sEmfEncRun.sEmf.slPostnEmf = 0 ;
    sEmfEncRun.sEmf.slSpeedEmf = 0 ;
    sEmfEncRun.sEmf.slAccelEmf = 0 ;
    sEmfEncRun.sOpl.slPostnOPL = 0 ;
    sEmfEncRun.sOpl.slSpeedOPL = 0 ;

    sBe_EmfEncDiagOut.uwSnsrlessState = SNSRLESS_DISABLED ;
//    sEmfEncRun.ubIqShift2Use = IQSHIFT_FULL ;
    BackEmfEncZeroOut() ;
}


// #########################################################################
static void BackEmfEncZeroOut(void)
{
    // OpenLoop: Output OL
    sEmfEncRun.sOpl.slPostnOPL_1 = 0L + (((ULONG)sEmfEncRun.psBackEmfOut->uwDeltaElecAngle) << 8) ;
    sEmfEncRun.ulElecEmfEncAngle = 0UL ;
    sEmfEncRun.swElecEmfEncTurns = 0 ;
    sEmfEncRun.ulMechAdder = 0UL ;

    sBe_EmfEncDiagOut.slMechSpeedDeltaAngle = 0L ; 
    sBe_EmfEncDiagOut.uwAntiglitchCounter = 0 ;

    // Encoder Out: ZERO (except position data that it is resetted only at boot)
//    INT64_ASSIGN(sEmfEncRun.psBackEmfOut->sEncData.sqPostn, 0L, 0UL) ;
    sEmfEncRun.psBackEmfOut->sEncData.slSpeed = 0L ;
    sEmfEncRun.psBackEmfOut->sEncData.slAccel = 0L ;
    sEmfEncRun.psBackEmfOut->uwElecAngle = 0 ;
    sEmfEncRun.psBackEmfOut->swElecSpeed = 0 ;

#if CFG_ENC_BEMF_DITEN
	Diten_Init(&sDitenRun.diten_observer, &sDitenRun.diten_pll, &sGlbMotorParameters, &sMh_MotorDataOut) ;
#endif
}


// #########################################################################
// =========================================================================
// ============================= BackEmf  Hook =============================
// =========================================================================
BOOL Be_Hook8KHz(GLB_IREF * psIRefIn, GLB_IREF * psIRefOut)
{    
    SQWRD sqCur2Use, sqHookId2Use, sqUserId2Use ;
    GLB_IREF sIRefs, * psROut=NULL;

#if CFG_ENC_BEMF_DITEN
    SLONG slHookId2Use ;

    // if psIRefIn = NULL it means that EncMngr keep in reset (efs is in progress and/or elec angle is not valid)
    // psIRefOut is the output current reference to use
    if(psIRefIn==NULL)
    {   // efs is in progress and/or elec angle is not valid: exit
        sEmfEncRun.flags.b.bHookInTransition = TRUE ;
        return TRUE;
    }

    slHookId2Use = (SLONG)(sDitenRun.id_inject.current_value * 10000) ;
    psIRefOut->slIdRef = psIRefIn->slIdRef + slHookId2Use ;

    if (sEmfEncRun.flags.b.bHookInTransition)
    {   // efs just finished: for one tick keep IqRef = 0
    	psROut->slIqRef = 0 ;
        sEmfEncRun.flags.b.bHookInTransition = FALSE;
    }
    else
    {   // efs is not in progress and elec angle is valid
        psIRefOut->slIqRef = psIRefIn->slIqRef ;
    }

#else
    // se NULL allora e' reset da enc mgr, out contiene riferimenti attuali
    if(psIRefIn==NULL)
    {
        sEmfEncRun.flags.b.bHookInTransition=TRUE;
        return TRUE;
    }

    // crea uscita locale
    if(sEmfEncRun.flags.b.bHookInTransition)
    {
        psROut=psIRefOut;
        psIRefOut=&sIRefs;
    }


    // Output delle correnti: Id           
    _sint64_mul_32_16(&sqHookId2Use, &sEmfEncRun.slMotorCurrentPeak,  &sEmfEncRun.swWeightOPL) ; 
   
    if (sBe_EmfEncParam.flags.b.bDynamicIdLimit)
    {   
        _sint64_mul_32_16(&sqUserId2Use, &(psIRefIn->slIdRef), &sEmfEncRun.swWeightEmf) ;
        _sint64_add(&sqCur2Use, &sqHookId2Use, &sqUserId2Use) ;
        psIRefOut->slIdRef = (SLONG)(INT64_MLONG(sqCur2Use)) << 1 ; 
    }
    else   
        psIRefOut->slIdRef = ((SLONG)(INT64_MLONG(sqHookId2Use)) << 1) + psIRefIn->slIdRef ; 

    // Output delle correnti: Iq
    // Nota: non correggo il limite di corrente dell'anello di controllo spazio' velocita' perche' il sensorless e' usato 
    //       'principalmente' per controllare in velocita', quindi l'integrale globale finale NON DEVE essere usato (non serve)
    if (sBe_EmfEncParam.flags.b.bDynamicIqLimit)
    {
//        if(psIRefIn->slIqRef >= sMh_MotorDataOut.sIqLimit.slMax)
//            psIRefOut->slIqRef = sMh_MotorDataOut.sIqLimit.slMax >> sEmfEncRun.ubIqShift2Use ;
//        else if(psIRefIn->slIqRef <= sMh_MotorDataOut.sIqLimit.slMin)
//            psIRefOut->slIqRef = sMh_MotorDataOut.sIqLimit.slMin >> sEmfEncRun.ubIqShift2Use ;
//        else           
//            psIRefOut->slIqRef = psIRefIn->slIqRef >> sEmfEncRun.ubIqShift2Use ;

        _sint64_mul_32_16(&sqCur2Use, &psIRefIn->slIqRef, &sEmfEncRun.swWeightEmf) ; 
        psIRefOut->slIqRef = (SLONG)(INT64_MLONG(sqCur2Use)) << 1 ; 
    }
    else
    {
        if (sEmfEncRun.swWeightOPL == 0x7fff)
            psIRefOut->slIqRef = 0 ; // tengo forzatamente a zero la Iq (tanto sono in open loop)
        else
            psIRefOut->slIqRef = psIRefIn->slIqRef ; /* always 100% */
    }

    // esegue rampe
    if(sEmfEncRun.flags.b.bHookInTransition)
    {
        BOOL bEndRamp=TRUE;

        if(psROut->slIdRef+sEmfEncRun.slHookMaxStep < psIRefOut->slIdRef)
        {
            psROut->slIdRef+=sEmfEncRun.slHookMaxStep;
            bEndRamp=FALSE;
        }
        else if(psROut->slIdRef-sEmfEncRun.slHookMaxStep > psIRefOut->slIdRef)
        {
            psROut->slIdRef-=sEmfEncRun.slHookMaxStep;
            bEndRamp=FALSE;
        }
        else
            psROut->slIdRef=psIRefOut->slIdRef;

        if(psROut->slIqRef+sEmfEncRun.slHookMaxStep < psIRefOut->slIqRef)
        {
            psROut->slIqRef+=sEmfEncRun.slHookMaxStep;
            bEndRamp=FALSE;
        }
        else if(psROut->slIqRef-sEmfEncRun.slHookMaxStep > psIRefOut->slIqRef)
        {
            psROut->slIqRef-=sEmfEncRun.slHookMaxStep;
            bEndRamp=FALSE;
        }
        else
            psROut->slIqRef=psIRefOut->slIqRef;

        if(bEndRamp)
            sEmfEncRun.flags.b.bHookInTransition=FALSE;
    }
#endif
    return TRUE ;
}


// #########################################################################
// =========================================================================
// ========================== Anti  Glitch Filter ==========================
// =========================================================================
//    Variabili aggiornate al servo i:
//      uwValue      = uwEmfAtanAngle
//      slEncPostnOL = usata come angolo di uscita di atan quando il filtro antiglitch non è attivo
//
//    Variabili aggiornate al servo i-1:
//      uwValue_1          = uwEmfAtanAngle_1
//      slEncOutMechSpd    = velocità meccanica in uscita dal modulo backemf e che entra nell'anello di controllo.
//      slEncOutAbsMechSpd = valore assoluto di slEncOutMechSpd (calcolato 'fuori' perche' serve anche altrove)
//
//    Variabili che escono:
//      uwEmfAtanAngle (eventualmente) antiglitchato
//
//    Parametri:
//      sEmfEncParam.slEmfEncGlitchFilterThreshold = soglia per far attivare il filtro antiglitch
//      sEmfEncParam.uwMotorPolePairs = coppie polari motore
static UWORD AtanAntiglitchFilter8KHz(UWORD uwValue, UWORD uwValue_1, SLONG slEncMechSpd, SLONG slEncAbsMechSpd, SLONG slEncPostnOL)
{
    SWORD swSpdInst, swSpdAvg, swDelta, swSpdLimL, swSpdLimH ;
    UWORD uwRetVal ;

    // no antiglitch filter if emf is very low
    // se slAbsMechSpeed e' maggiore della soglia antiglitch, allora regressione atan
    // è fatta sul valore dell'arcotangente, altrimenti è fatta sull'angolo openloop
    if (slEncAbsMechSpd > sEmfEncRun.slEmfEncGlitchFilterThreshold)
    {
        swSpdInst = (SWORD)uwValue - (SWORD)uwValue_1 ;
        swSpdAvg  = (SWORD)((slEncMechSpd * sEmfEncRun.uwPolePairs) >> 16) ;
        swDelta   = swSpdInst - swSpdAvg ;

        if (abs(swDelta) >= (abs(swSpdAvg)/2))
        {   // antiglitch deve correggere

            sBe_EmfEncDiagOut.uwAntiglitchCounter++ ;

            swSpdLimL = swSpdAvg / 2 ;
            swSpdLimH = swSpdAvg + swSpdLimL ;

            if(((swDelta < 0) && (swSpdAvg > 0)) || ((swDelta > 0) && (swSpdAvg < 0)))
                uwRetVal = (UWORD)((SWORD)uwValue_1 + swSpdLimL) ;
            else
                uwRetVal = (UWORD)((SWORD)uwValue_1 + swSpdLimH) ;
        }
        else
        {  // antiglitch non deve correggere: decremento il contatore (in maniera meno rapida se sono in pieno sensorless)
           if (slEncAbsMechSpd > sEmfEncRun.slSpeedSensorless)
           {    // controllo piu' stretto
                if(sBe_EmfEncDiagOut.uwAntiglitchCounter >= 4)
                    sBe_EmfEncDiagOut.uwAntiglitchCounter -= 4 ;
                else
                    sBe_EmfEncDiagOut.uwAntiglitchCounter = 0 ;
           }
           else
           {    // controllo piu' blando
                if(sBe_EmfEncDiagOut.uwAntiglitchCounter >= 256)
                    sBe_EmfEncDiagOut.uwAntiglitchCounter -= 256 ;
                else
                    sBe_EmfEncDiagOut.uwAntiglitchCounter = 0 ;
           }

            uwRetVal = uwValue ;
        }
    }
    else
        uwRetVal = (UWORD)(slEncPostnOL >> 8) ; // NOTE: uwEmfAtanAngle = openloop angle

    return uwRetVal ;
}


// #########################################################################
// =========================================================================
// ============================= Find  Weights =============================
// =========================================================================
static UWORD FindWeightUnsigned(SLONG slSpeed, SWORD swSlope)
{   // f(w) = k * (w - w0)
    SQWRD sqWeight ;
    SWORD swWeightW1, swWeightW2 ;
    SLONG slSpeedTmp ;

    slSpeedTmp = slSpeed - sEmfEncRun.slSpeedThreshold1 ;
    if (slSpeedTmp < 0)
        return 0 ;

    _sint64_mul_32_16(&sqWeight, &slSpeedTmp, &swSlope) ;
    swWeightW2 =INT64_WORD2(sqWeight) ;
    swWeightW1 =INT64_WORD1(sqWeight) ; // >> 16

    // saturate result if word2 != 0 and word1 < 0
    if(swWeightW2 || (swWeightW1 < 0))
        swWeightW1 = 0x7fff ;

    return (UWORD)swWeightW1 ;
}


// #########################################################################
// =========================================================================
// ============================= Kt Evaluation =============================
// =========================================================================
static void MotorKtEvaluation(void)
{
    // vedi il plc!!

    SLONG slMechSpeedDeltaAngle ;
    ULONG ulMagicNumber ;
    FLOAT flValuedKt ;

    if((sBe_EmfEncDiagOut.uwSnsrlessState == SNSRLESS_FULL) || (sEmfEncRun.flags.b.bOpenLoopOnly && (sBe_EmfEncDiagOut.uwSnsrlessState > SNSRLESS_THRESHOLD_1)))
    {
        if (!sEmfEncRun.flags.b.bKtOneSecondTimerSet)
        {   // imposto il timeout per il contatore ad 1 secondo
            sEmfEncRun.sKtEval.uwOneSecBkgdTimer = timer_settimeout(uwSysTimers1ms, ONE_SECOND) ;
            sEmfEncRun.flags.b.bKtOneSecondTimerSet = TRUE ;
        }

        if(timer_istimedout(uwSysTimers1ms, sEmfEncRun.sKtEval.uwOneSecBkgdTimer))
        {   // e' passato un secondo
            sEmfEncRun.sKtEval.uwOneSecBkgdCntr++ ;
            sEmfEncRun.flags.b.bKtOneSecondTimerSet = FALSE ; // ricarico il timer

            // aggiorno dolcemente il Kt
            if (sEmfEncRun.sKtEval.uwOneSecBkgdCntr < sBe_EmfEncParam.uwKtRefresh)
            {
                sBe_EmfEncDiagOut.flValuedKt += sEmfEncRun.sKtEval.flIncrementoKt ;
                ulMagicNumber = (ULONG)(FLOAT_KT_CONVFACTOR / sBe_EmfEncDiagOut.flValuedKt) ;
                atomic_write(&sBe_EmfEncDiagOut.ulMagicNumber, &ulMagicNumber, sizeof(ULONG)) ;
            }
        }

        if (sEmfEncRun.sKtEval.uwOneSecBkgdCntr == sBe_EmfEncParam.uwKtRefresh)
        {   // e' il momento di stimare il Kt..
            // Kt = (MAGICNUMBER_CONVFACTOR * uwSQRooT) / slMechSpeedDeltaAngle
            flValuedKt = (FLOAT_KT_CONVFACTOR * sEmfEncRun.sKtEval.flSQRootSum / (FLOAT)sEmfEncRun.sKtEval.ulKtCounter) / (sEmfEncRun.sKtEval.flMechSpeedDeltaAngleSum / (FLOAT)sEmfEncRun.sKtEval.ulKtCounter) ;

            // controllo se il Kt e' nel range valido
            if ((flValuedKt < sEmfEncRun.sKtEval.flKtMin) || (flValuedKt > sEmfEncRun.sKtEval.flKtMax))
            {   // mi sto smagnetizzando: fault (se non openloop)
                sBe_EmfEncDiagOut.flValuedKt = flValuedKt ;
                sEmfEncRun.sKtEval.flIncrementoKt = 0.0 ;

                sEmfEncRun.ulAlarm2Send |= SYSTEMALARMS_SUBCODE_BACKEMF_KT_OUTOFRANGE ;

                if ((sEmfEncRun.flags.b.bOpenLoopOnly) || (SYSTEMALARMS_SUBCODE_BACKEMF_KT_OUTOFRANGE & sBe_EmfEncParam.ulDisableBackEmfAlarmMask))
                {   // sono in anello aperto: warning
                    atomic_long_set_bits( &ulSystemWarnings, SYSTEMWARNINGS_KT_OUTOFRANGE );
                }
                else
                {   // sono in sensorless: fault
                    if(!sEmfEncRun.sErrorSent.b.bKtOutOfRange)
                    {
                        sEmfEncRun.ubEmfEncStatus = EMFENC_FAULT ;
                        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_BE_BACKEMF_FAIL, SYSTEMALARMS_SUBCODE_BACKEMF_KT_OUTOFRANGE, TRUE) ;
                        sEmfEncRun.sErrorSent.b.bKtOutOfRange = TRUE ;
                    }
                }
            }
            else
            {
                    // resetto il warning
                atomic_long_clear_bits( &ulSystemWarnings, SYSTEMWARNINGS_KT_OUTOFRANGE );

                // faccio i dovuti controlli sul Kt appena stimato prima di iniziare ad aggiornare il numero magico
                ulMagicNumber = (ULONG)(FLOAT_KT_CONVFACTOR / flValuedKt) ;

                if((ulMagicNumber == 0) || (ulMagicNumber >= 0x10000))
                {   // fault (se non openloop): il numero magico deve essere SEMPRE compreso tra (zero ; 65535]
                    sBe_EmfEncDiagOut.flValuedKt = flValuedKt ;
                    sEmfEncRun.sKtEval.flIncrementoKt = 0.0 ;

                    if (sEmfEncRun.flags.b.bOpenLoopOnly)
                    {   // sono in anello aperto: warning
                        atomic_long_set_bits( &ulSystemWarnings, SYSTEMWARNINGS_INV_MAGICNUMBER );
                    }
                    else
                    {   // sono in sensorless: fault
                        if(!sEmfEncRun.sErrorSent.b.bInvalidMagicNumber)
                        {
                            sEmfEncRun.ubEmfEncStatus = EMFENC_FAULT ;
                            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_BE_BACKEMF_FAIL, SYSTEMALARMS_SUBCODE_BACKEMF_INV_MAGICNUMBER, TRUE) ;
                            sEmfEncRun.sErrorSent.b.bInvalidMagicNumber = TRUE ;
                        }
                    }
                }
                else
                {
                    atomic_long_clear_bits( &ulSystemWarnings, SYSTEMWARNINGS_INV_MAGICNUMBER );

                    if (sEmfEncRun.flags.b.bOpenLoopOnly)
                    {   // se anello aperto, nessun 'dolce' aggiornamento
                        sBe_EmfEncDiagOut.flValuedKt = flValuedKt ;
                        sEmfEncRun.sKtEval.flIncrementoKt = 0.0 ;
                    }
                    else
                    {  // tutto ok, mi preparo per aggiornare dolcemente (ogni secondo) il Kt del motore (calcolo l'incremento)
                       sEmfEncRun.sKtEval.flIncrementoKt = (flValuedKt - sBe_EmfEncDiagOut.flValuedKt) / ((FLOAT)sBe_EmfEncParam.uwKtRefresh) ;
                    }
                }
            }

            // ..e resettare le variabili
            sEmfEncRun.sKtEval.ulKtCounter = 0 ;
            sEmfEncRun.sKtEval.uwOneSecBkgdCntr = 0 ;
            sEmfEncRun.sKtEval.flSQRootSum = 0.0 ;
            sEmfEncRun.sKtEval.flMechSpeedDeltaAngleSum = 0.0 ;
        }
        else
        {
            atomic_read(&slMechSpeedDeltaAngle, &sBe_EmfEncDiagOut.slMechSpeedDeltaAngle, sizeof(SLONG)) ;
            sEmfEncRun.sKtEval.flMechSpeedDeltaAngleSum += (FLOAT)(labs(slMechSpeedDeltaAngle)) ;
            sEmfEncRun.sKtEval.flSQRootSum += (FLOAT)(sBe_EmfEncIn.psBackEmfData->uwVmotor) ;
            sEmfEncRun.sKtEval.ulKtCounter++ ;
        }
    }
    else
    {   // tengo tutto fermo
        sEmfEncRun.flags.b.bKtOneSecondTimerSet = FALSE ;
        sEmfEncRun.sKtEval.ulKtCounter = 0 ;
        sEmfEncRun.sKtEval.uwOneSecBkgdCntr = 0 ;
        sEmfEncRun.sKtEval.flSQRootSum  = 0.0 ;
        sEmfEncRun.sKtEval.flMechSpeedDeltaAngleSum = 0.0 ;
        sEmfEncRun.sKtEval.flIncrementoKt = 0.0 ;
    }
}

// #########################################################################
// =========================================================================
// =========== Variabili runtime derivate dalla velocita' motore ===========
// =========================================================================
static void CalculateThreshold(void)
{
    SLONG slTmp;

//    // Velocita' di ginocchio: MechMaxSpeed = ((DcBus(iu)/10)(V) * sqrt(3)) / (1.03 * sqrt(2) * Kt) = DcBus(iu)/Kt * (sqrt(3)/(10 * 1.03 * sqrt(2)) = DcBus(iu)/Kt * 0.11891
//    // 1.03 = 3% di margine; DcBus = DcBus filtrato @8ms dal modello termico
//    slTmp = (SLONG)(MAXMECHSPDCONVFACTOR_IU * (FLOAT) * sBe_EmfEncIn.psVdcBusFiltered / sBe_EmfEncDiagOut.flValuedKt) ; // internal unit
//    atomic_write(&sBe_EmfEncDiagOut.slMechMaxSpeed, &slTmp, sizeof(SLONG));

    // uso la velocita' di ginocchio calcolato dall'algoritmo del deflussaggio
    atomic_write(&sBe_EmfEncDiagOut.slMechMaxSpeed, &sDflx_Out.slDflxKneeSpd, sizeof(SLONG));

    // soglie per il mix
    slTmp = (SLONG)((FLOAT)sBe_EmfEncDiagOut.slMechMaxSpeed * sEmfEncRun.sPercentage.flSsnrlssSpeed) ;
    atomic_write(&sEmfEncRun.slSpeedSensorless, &slTmp, sizeof(SLONG));

    slTmp = (SLONG)((FLOAT)sBe_EmfEncDiagOut.slMechMaxSpeed * sEmfEncRun.sPercentage.flSsnrlssThreshold1) ;
    atomic_write(&sEmfEncRun.slSpeedThreshold1, &slTmp, sizeof(SLONG));

    slTmp = (SLONG)((FLOAT)sBe_EmfEncDiagOut.slMechMaxSpeed * sEmfEncRun.sPercentage.flSsnrlssThreshold0) ;
    atomic_write(&sEmfEncRun.slSpeedThreshold0, &slTmp, sizeof(SLONG));

    // soglia per il filtro antiglitch
    slTmp = (SLONG)((FLOAT)sBe_EmfEncDiagOut.slMechMaxSpeed * sEmfEncRun.sPercentage.flEmfEncAGlitchFilter) ;
    atomic_write(&sEmfEncRun.slEmfEncGlitchFilterThreshold, &slTmp, sizeof(SLONG));

    // velocita' di rientro al volo
    // nota: per il momento minima velocita' di rientro al volo = sensorless speed + 10%
    slTmp = (SLONG)((FLOAT)sBe_EmfEncDiagOut.slMechMaxSpeed * sEmfEncRun.sPercentage.flBackOnTheFlyMinSpeed) ;
    atomic_write(&sEmfEncRun.slBackOnTheFlyMinSpeed, &slTmp, sizeof(SLONG));

    slTmp = (SLONG)((FLOAT)sBe_EmfEncDiagOut.slMechMaxSpeed * sEmfEncRun.sPercentage.flBackOnTheFlyUsrSpeed) ;
    atomic_write(&sEmfEncRun.slBackOnTheFlyUsrSpeed, &slTmp, sizeof(SLONG));

    // pendenza della retta del 'mix' [A = (slSpeedThreshold1, 0); B = (slSpeedSensorless, 1)]
    sEmfEncRun.swSpeedSlope = (SWORD)((FLOAT)SLONG_MAX_VALUE / ((FLOAT)sEmfEncRun.slSpeedSensorless - (FLOAT)sEmfEncRun.slSpeedThreshold1)) ;
}


// #########################################################################
// =========================================================================
// =============================== BEMF DITEN===============================
// =========================================================================
#if CFG_ENC_BEMF_DITEN
BOOL Diten_Init(S_OBSERVER *observer, S_PLL *pll, MOTPRM_PARAMETERS *parameters, MH_MOTORDATA_OUT *currVolts)
{
	UWORD uwPolePairs = parameters->uwPoleNumbers / 2 ;

	// observer init parameters
	observer->parameter.gamma1 = (FLOAT)(sBe_EmfEncDefParam.diten_obs_gains.obs_gamma1 * DITEN_OBS_G1_GAIN); //guadagno osservatore. settare indicativamente a (0.033/phi)
	observer->parameter.gamma2 = (FLOAT)(sBe_EmfEncDefParam.diten_obs_gains.obs_gamma2 * DITEN_OBS_G2_GAIN); //guadagno osservatore. settare indicativamente a (0.033/phi)
	observer->parameter.a      = (FLOAT)(sBe_EmfEncDefParam.diten_obs_gains.obs_a      * DITEN_OBS_A_GAIN) ; //parametro alfa del filtro [alfa*s/(s+alfa)]

	// observer init variables
	observer->variable.flux[0]   = parameters->flKT * 0.5773 / (FLOAT)uwPolePairs ;//-parameters->flInductance * flIa; //e1
	observer->variable.flux[1]   = 0.0 ; //-parameters->flInductance * flIb; //e2
	observer->variable.flux[2]   = 0.0 ; //q1+L*i1
	observer->variable.flux[3]   = 0.0 ; //q2+L*i2
	observer->variable.deflux[0] = 0.0 ; //de1
	observer->variable.deflux[1] = 0.0 ; //de2
	observer->variable.deflux[2] = 0.0 ; //dq1
	observer->variable.deflux[3] = 0.0 ; //dq2
	observer->variable.q1[0]     = 0.0 ; //q1 current step
	observer->variable.q1[1]     = 0.0 ; //q1 previous step
	observer->variable.q2[0]     = 0.0 ; //q2 current step
	observer->variable.q2[1]     = 0.0 ; //q2 previous step
	observer->variable.q12q[0]   = 0.0 ; //q12q current step
	observer->variable.q12q[1]   = 0.0 ; //q12q previous step
	observer->variable.yq        = 0.0 ; //uscita filtro yq
	observer->variable.omegaq1   = 0.0 ; //uscita filtro omegaq1
	observer->variable.omegaq2   = 0.0 ; //uscita filtro omegaq2
	observer->variable.x1        = 0.0 ; //x1
	observer->variable.x2        = 0.0 ; //x2
	observer->variable.thet      = DITEN_270DEG_IN_RADIANT ;
	observer->variable.t_idr     = 0.0 ;

	observer->variable.omegaq1_i = 0.0 ;
	observer->variable.omegaq2_i = 0.0 ;
	observer->variable.yq_i      = 0.0 ;

	observer->MechOut.elAngleDelayed    = DITEN_270DEG_IN_RADIANT ;
	observer->MechOut.revolutionCounter = 0;
	observer->MechOut.mAngleAdder       = 0;

	//PLL parameters
	/*
	pll->parameter.kp = 800;
	pll->parameter.ki = 10000;
	pll->parameter.tfilter = 0.0125;
	*/
	pll->parameter.kp      = (FLOAT)(DITEN_OBS_PLL_KP_GAIN * sBe_EmfEncDefParam.diten_obs_gains.pll_kp);
	pll->parameter.ki      = (FLOAT)(DITEN_OBS_PLL_KI_GAIN * sBe_EmfEncDefParam.diten_obs_gains.pll_ki);
	pll->parameter.tfilter = (FLOAT)(DITEN_OBS_PLL_TF_GAIN * sBe_EmfEncDefParam.diten_obs_gains.pll_tfilter);

	//PLL variables
	pll->variable.cos_teta      = 0.0 ;
	pll->variable.sin_teta      = 0.0 ;
	pll->variable.cos_tetaest   = 0.0 ;
	pll->variable.sin_tetaest   = 0.0 ;
	pll->variable.epsilon       = 0.0 ;
	pll->variable.tetaest       = 0.0 ;
	pll->variable.integral      = 0.0 ;
	pll->variable.omegaest      = 0.0 ;
	pll->variable.omegafiltr[0] = 0.0 ;
	pll->variable.omegafiltr[1] = 0.0 ;
	pll->variable.wrap          = 0.0 ;

	sBe_EmfEncDiagOut.diten_obs_pll_out.elSpeed   = 0 ;
	sBe_EmfEncDiagOut.diten_obs_pll_out.mechSpeed = 0 ;
	sBe_EmfEncDiagOut.diten_obs_pll_out.elThet    = (UWORD)(observer->variable.thet * DITEN_OBS_EL_POSITION_GAIN);
	sBe_EmfEncDiagOut.diten_obs_pll_out.mechThet  = (ULONG)((observer->MechOut.mAngleAdder * (FLOAT_2PI / (FLOAT)uwPolePairs) + observer->variable.thet / (FLOAT)uwPolePairs) * DITEN_OBS_POSITION_GAIN);
	sBe_EmfEncDiagOut.diten_obs_pll_out.revolutionCounter = 0 ;

	sDitenRun.id_inject.params.minSpeedWithoutInjection = (FLOAT)(DITEN_OBS_MIN_SPEED_GAIN    * sBe_EmfEncDefParam.diten_obs_gains.id_inj_minSpeedWithoutInjection);
	sDitenRun.id_inject.params.accRateInSecToRated      = (FLOAT)(DITEN_OBS_INJ_ACCRATE_GAIN  * sBe_EmfEncDefParam.diten_obs_gains.id_inj_accRateInSec);
	sDitenRun.id_inject.params.maxValuePercentOfRated   = (FLOAT)(DITEN_OBS_INJ_MAXVALUE_GAIN * sBe_EmfEncDefParam.diten_obs_gains.id_inj_maxValuePercent / 100.0);

//	sDitenRun.id_inject.current_value = sDitenRun.id_inject.params.maxValuePercentOfRated * parameters->flCurrentNominal;
	sDitenRun.id_inject.current_value = 0.0 ;
	sDitenRun.id_inject.target_value  = sDitenRun.id_inject.params.maxValuePercentOfRated * parameters->flCurrentNominal ;

	sDitenRun.swTestCounter = 0 ;
	sDitenRun.flHookTimer = 0.0 ; // is used of injection of constant id current. The current is inject the first second after start. so it is important to set it to zero if sMotCtrl_UsrControl.uwControlWord = 0

	return TRUE ;
}

/*
 * The function contains observer equations for position estimation + PLL for speed estimation
 */
BOOL Diten_Task8KHz(S_OBSERVER *observer, S_PLL *pll, MOTPRM_PARAMETERS * parameters, MH_MOTORDATA_OUT * currVolts)
{
	FLOAT flVu, flVv, flVw, flV3Harm ;
	FLOAT flIu, flIv, flIw ;

	FLOAT flVa, flVb ;
	FLOAT flIa, flIb ;

	UWORD uwPolePairs = parameters->uwPoleNumbers / 2 ;

	FLOAT flPsim = parameters->flKT * C_1DSQRT3 / (FLOAT)uwPolePairs;
	FLOAT flTsample = DITEN_OBS_SERVOTIME ;

	flIu = (float)currVolts->slIuFb * 0.0001;
	flIv = (float)currVolts->slIvFb * 0.0001;
	flIw = (float)currVolts->slIwFb * 0.0001;

#if CFG_VMOTOR_READ
	flVu = (FLOAT)currVolts->swVuEffective * 0.1 ;
	flVv = (FLOAT)currVolts->swVvEffective * 0.1 ;
	flVw = (FLOAT)currVolts->swVwEffective * 0.1 ;
	flV3Harm = (flVu + flVv + flVw) / 3.0 ;

	// 3a harm compensation
	flVu = flVu - flV3Harm ;
	flVv = flVv - flV3Harm ;
	flVw = flVw - flV3Harm ;
#else
	flVu = (float)currVolts->swVuEstimated * 0.1;
	flVv = (float)currVolts->swVvEstimated * 0.1;
	flVw = - flVu - flVv;
#endif

	// Alfa = (2/3) * (U - V/2 - W/2) = 0.666 * (U - V/2 - W/2)
	// Beta = (V - W) / SQRT(3) 0.666 * (0.866 * V - 0.866 * W)
	// 0.666 = 2/3 = C_2D3
	// 0.866 = sqrt(3.0) / 2 = C_SQRT3D2
	// 0.666 * 0.866 = C_1DSQRT(3)
	flIa = C_2D3     * (flIu - 0.5 * flIv - 0.5 * flIw) ;
	flIb = C_1DSQRT3 * (flIv - flIw) ;

	flVa = C_2D3     * (flVu - 0.5 * flVv - 0.5 * flVw) ;
	flVb = C_1DSQRT3 * (flVv - flVw) ;

	observer->parameter.tfilter = 1 - observer->parameter.a * flTsample;  	//parametro denominatore del filtro precendente in seguito a discretizzazione

	observer->variable.q12q[0] =  observer->variable.q1[0]   * observer->variable.q1[0] +observer->variable.q2[0]   *observer->variable.q2[0];
	observer->variable.yq      = -observer->variable.q12q[0] * observer->parameter.a    +observer->variable.q12q[1] *observer->parameter.a + observer->parameter.tfilter * observer->variable.yq;
	observer->variable.q12q[1] =  observer->variable.q12q[0] ;

	//filtro omegaq1
	observer->variable.omegaq1 = 2 * observer->variable.q1[0] * observer->parameter.a - 2 * observer->variable.q1[1] * observer->parameter.a + observer->parameter.tfilter * observer->variable.omegaq1;
	observer->variable.q1[1]   = observer->variable.q1[0] ;

	//filtro omegaq2
	observer->variable.omegaq2 = 2 * observer->variable.q2[0] * observer->parameter.a - 2 * observer->variable.q2[1] * observer->parameter.a + observer->parameter.tfilter * observer->variable.omegaq2;
	observer->variable.q2[1]   = observer->variable.q2[0] ;

	//e1 ed e2 derivate (de1, de2)//
	observer->variable.deflux[0] = (observer->parameter.gamma2 * observer->variable.omegaq1 * (observer->variable.yq - observer->variable.omegaq1 * observer->variable.flux[0] - observer->variable.omegaq2 * observer->variable.flux[1]));
	observer->variable.deflux[1] = (observer->parameter.gamma2 * observer->variable.omegaq2 * (observer->variable.yq - observer->variable.omegaq1 * observer->variable.flux[0] - observer->variable.omegaq2 * observer->variable.flux[1]));

	//q1 e q2 derivate  (dq1, dq2)//
	observer->variable.deflux[2] = (flVa - parameters->flResistance * flIa + observer->parameter.gamma1 * observer->variable.flux[0] * (observer->variable.flux[0] * observer->variable.flux[0] + observer->variable.flux[1]*observer->variable.flux[1] - flPsim * flPsim));
	observer->variable.deflux[3] = (flVb - parameters->flResistance * flIb + observer->parameter.gamma1 * observer->variable.flux[1] * (observer->variable.flux[0] * observer->variable.flux[0] + observer->variable.flux[1]*observer->variable.flux[1] - flPsim * flPsim));

	//integrale
	observer->variable.flux[0] = observer->variable.flux[0] + observer->variable.deflux[0] * flTsample;
	observer->variable.flux[1] = observer->variable.flux[1] + observer->variable.deflux[1] * flTsample;
	observer->variable.flux[2] = observer->variable.flux[2] + observer->variable.deflux[2] * flTsample;
	observer->variable.flux[3] = observer->variable.flux[3] + observer->variable.deflux[3] * flTsample;

	//aggiunta termine -L*i a q1 e q2
	observer->variable.q1[0] = observer->variable.flux[2] - parameters->flInductance * flIa;
	observer->variable.q2[0] = observer->variable.flux[3] - parameters->flInductance * flIb;

	//uscite
	observer->variable.x1 = observer->variable.flux[0] + observer->variable.q1[0];
	observer->variable.x2 = observer->variable.flux[1] + observer->variable.q2[0];

	observer->variable.thet = atan2(observer->variable.x2, observer->variable.x1);

	//<<<<<<<<<<<<<<PLL>>>>>>>>>>>>>>>>>>>>//
	pll->variable.cos_teta    = cos(observer->variable.thet);
	pll->variable.sin_teta    = sin(observer->variable.thet);
	pll->variable.cos_tetaest = cos(pll->variable.tetaest);
	pll->variable.sin_tetaest = sin(pll->variable.tetaest);

	pll->variable.epsilon  = pll->variable.sin_teta * pll->variable.cos_tetaest - pll->variable.cos_teta * pll->variable.sin_tetaest;
	pll->variable.integral = pll->variable.integral + pll->parameter.ki * pll->variable.epsilon * 0.5 * flTsample;
	pll->variable.omegaest = pll->variable.integral + pll->parameter.kp * pll->variable.epsilon;
	pll->variable.tetaest  = pll->variable.tetaest  + pll->variable.omegaest * flTsample;

	//filtro omega in uscita
	pll->variable.omegafiltr[0] = pll->variable.omegafiltr[1] * (1 - flTsample / pll->parameter.tfilter) + pll->variable.omegaest * flTsample / pll->parameter.tfilter;
	pll->variable.omegafiltr[1] = pll->variable.omegafiltr[0];

	/*
	 *  It looks like 0 angle of dq reference frame corresponds to -q axis.
	 *  Observer estimates angle considering that 0 angle in dq reference  frame is aligned with d axis
	 *  So it is required to add 270 degrees to estimated position value after calculation
	 * */
	observer->variable.thet += DITEN_270DEG_IN_RADIANT ;

	/*
	 *  Output range of atan2 is [-pi, pi]. FPGA and control require the angle to be of unsigned type (ULONG or UWORD).
	 *  So in the next section observer->variable.thet is scaled to be from [0 2*pi].
	 * */
	if (observer->variable.thet > FLOAT_2PI)
	{
		observer->variable.thet -= FLOAT_2PI;
	}

	if (observer->variable.thet < 0.0)
	{
		observer->variable.thet += FLOAT_2PI;
	}

	/*
	 *  The section presents recalculation from electrical angle to mechanical angle.
	 *  Recalculation is based on number of pole pairs of the motor.
	 *  It is necessary to detect direction of rotation. Depending on direction of rotation and once angle
	 *  crosses 2*pi value or 0 value, mAngleAdder is increased or decreased by 1. if value of mAngleAdder exceeds
	 *  range [0, motor_pole_pairs - 1], revolution counter value is changed.
	 *  mechanical angle is composed as mechThet = mAngleAdder * 2 * pi / motor_pole_pairs + elThet / motor_pole_pairs.
	 * */
	if (abs(observer->variable.thet-observer->MechOut.elAngleDelayed)>5*FLOAT_PI/6)
	{
		if (observer->variable.thet < observer->MechOut.elAngleDelayed)
		{
			observer->MechOut.mAngleAdder += 1;
		}

		if (observer->variable.thet > observer->MechOut.elAngleDelayed)
		{
			observer->MechOut.mAngleAdder -= 1;
		}

		if (observer->MechOut.mAngleAdder == (SWORD)uwPolePairs)
		{
			observer->MechOut.mAngleAdder = 0;
			observer->MechOut.revolutionCounter += 1;
		}

		if (observer->MechOut.mAngleAdder == -1)
		{
			observer->MechOut.mAngleAdder = (SWORD)uwPolePairs - 1;
			observer->MechOut.revolutionCounter -= 1;
		}
	}

	observer->MechOut.elAngleDelayed = observer->variable.thet;

	/*
	 * Structure diten_obs_pll_out stored scaled output values of the observer based on the requirements of control and fpga.
	 */
	sBe_EmfEncDiagOut.diten_obs_pll_out.mechSpeed = (SLONG)(pll->variable.omegafiltr[0] * DITEN_OBS_SPEED_GAIN /(FLOAT)uwPolePairs);
	sBe_EmfEncDiagOut.diten_obs_pll_out.elSpeed   = (SWORD)_sint32_scale_32(sBe_EmfEncDiagOut.diten_obs_pll_out.mechSpeed, uwPolePairs);
	sBe_EmfEncDiagOut.diten_obs_pll_out.elThet    = (UWORD)(observer->variable.thet * DITEN_OBS_EL_POSITION_GAIN);
	sBe_EmfEncDiagOut.diten_obs_pll_out.mechThet  = (ULONG)((observer->MechOut.mAngleAdder * (FLOAT_2PI/(FLOAT)uwPolePairs) + observer->variable.thet/(FLOAT)uwPolePairs) * DITEN_OBS_POSITION_GAIN);
	sBe_EmfEncDiagOut.diten_obs_pll_out.revolutionCounter = (SLONG)observer->MechOut.revolutionCounter;

	return TRUE ;
}

void Diten_Observer_Run(void)
{
	FLOAT flId2InjectStep ;

	Diten_Task8KHz(&sDitenRun.diten_observer, &sDitenRun.diten_pll, &sGlbMotorParameters, &sMh_MotorDataOut); // observer calculation

	/*
	 * The constant current is injected in the first second after start and if estimated speed if below a certain threshold.
	 * If the speed goes higher than a threshold, id current is injected for one more second.
	 * Speed threshold value, id constant value and id acceleration rate can be changed via cockpit.
	 * At the moment speed threshold is 16 rad/s (5% of rated speed), acceleration rate 1s to a rated value, id value 10% of rated current.
	 */
	if ((sDitenRun.flHookTimer < 1.0) || (fabs((float)sBe_EmfEncDiagOut.diten_obs_pll_out.mechSpeed / DITEN_OBS_SPEED_GAIN) < sDitenRun.id_inject.params.minSpeedWithoutInjection)) // check if id current injection is required
	{
		if ((fabs((float)sBe_EmfEncDiagOut.diten_obs_pll_out.mechSpeed / DITEN_OBS_SPEED_GAIN) < sDitenRun.id_inject.params.minSpeedWithoutInjection)) // if injection is required and threshold is higher than estimated speed value the timer value has to be set to zero
		{
			sDitenRun.flHookTimer = 0.0 ; // set of the timer to zero (to hold id current for 1s after) if the speed is lower than a threshold.
		}
		sDitenRun.id_inject.target_value = sDitenRun.id_inject.params.maxValuePercentOfRated * sGlbMotorParameters.flCurrentNominal; //target value of id current
	}
	else
		sDitenRun.id_inject.target_value = 0;

	// if statements below serve to set id reference, since id reference can be changed only with limited acceleration
	flId2InjectStep = sGlbMotorParameters.flCurrentNominal / sDitenRun.id_inject.params.accRateInSecToRated * DITEN_OBS_SERVOTIME ;

	if (fabs(sDitenRun.id_inject.current_value - sDitenRun.id_inject.target_value) > flId2InjectStep)
	{
		if (sDitenRun.id_inject.current_value > sDitenRun.id_inject.target_value)
			sDitenRun.id_inject.current_value -= flId2InjectStep ;
		else if (sDitenRun.id_inject.current_value < sDitenRun.id_inject.target_value)
			sDitenRun.id_inject.current_value += flId2InjectStep ;
	}

	sDitenRun.flHookTimer += DITEN_OBS_SERVOTIME ;
}

void Diten_8kHz(void)
{
	if (sEfsParam.ubProcType) //check if field orientation procedure is on.
		//The problem was that if it is on, observer started to estimate right after the procedure starts, but not after it is finished.
		//So I have to consider two options:
		// - if field orientaion is off - start estimation when sMotCtrl_UsrControl.uwControlWord is not 0
		// - if field orientaion is on  - start estimation when the procedure is finished.
	{
		sDitenRun.swTestCounter++ ;

		if ((sBe_EmfEncIn.psPowerStageStatus->b.bFullyActive) && sEmfEncRun.flags.b.bRotorZeroFound)
		{
			Diten_Observer_Run(); // the function that contain the observer and manages id current injection
			sBe_EmfEncDiagOut.uwSnsrlessState = SNSRLESS_FULL ;
		}
		else
		{
			Diten_Init(&sDitenRun.diten_observer, &sDitenRun.diten_pll, &sGlbMotorParameters, &sMh_MotorDataOut) ;
			sBe_EmfEncDiagOut.uwSnsrlessState = SNSRLESS_DISABLED ;
		}
	}
	else //field orientation procedure is off: set manually the encoder status
	{
		sDitenRun.swTestCounter = 0 ;
		sEmfEncRun.flags.b.bRotorZeroFound = TRUE ;

		if(!sBe_EmfEncIn.psPowerStageStatus->b.bVoltageEnabled)
		{	// PWM disabled: keep observer resetted (Diten_Init)
			Diten_Init(&sDitenRun.diten_observer, &sDitenRun.diten_pll, &sGlbMotorParameters, &sMh_MotorDataOut) ;
			sEmfEncRun.ubEmfEncStatus = EMFENC_ZEROOUT ;
			sBe_EmfEncDiagOut.uwSnsrlessState = SNSRLESS_DISABLED ;
		}
		else
		{	// PWM enabled: run observer
			Diten_Observer_Run();
			sEmfEncRun.ubEmfEncStatus = EMFENC_OK ;
			sBe_EmfEncDiagOut.uwSnsrlessState = SNSRLESS_FULL ;
		}
	}
}
#endif // cfg_enc_bemf_diten
#endif // cfg_enc_bemf
