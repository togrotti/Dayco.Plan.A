/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2010, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : Deflux.c                                                   */
/* Author      : Fabio Terrile; Cristiano Tognetti; Marco Calvini           */
/*                                                                          */
/* Description : Field weakening task                                       */
/*               (extension of MotorHandler)                                */
/*                                                                          */
/****************************************************************************/
#include <math.h>
#include <stdlib.h>
#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonUtility.h"
#include "common\MathFunctions.h"
#include "common\DspFunctions.h"
#include "drive\Deflux.h"
#include "drive\DefluxRT.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#if defined(_CRS_DBG)
#if (FALSE) /* CRS_DBGDSK */
#pragma GCC optimize (0) // crs_dbg
#else
#pragma GCC optimize (2)
#endif
#else
#pragma GCC optimize (2)
#endif
//***************************************************************************
// Global
DFLX_IN     sDflx_In ;
DFLX_OUT    sDflx_Out ;
DFLX_PARAM  sDflx_Param ;

// default parameters 
#ifdef _INFINEON_
const DFLX_PARAM huge sDflx_DefParam = 
#else
const DFLX_PARAM sDflx_DefParam = 
#endif // _infineon_
{
  0ul,    // MaxSpeed
  950,    // fattore di sicurezza
  0,      // fix Vdcbus, se zero usa lettura VdcBus
#if CFG_DFLX_VMOTOR
  0.0, 	  // flPiManualRefVal
  1.0,    // flMargin
  0.0,    // flPiKi
  0.0,    // flPiKp
  0.0,    // flPiLimitMax
  0.0,    // flPiLimitMin
  314.0,  // flPiEnableSpeed
  1,      // bDefluxOnly_Matrix
  0,      // bDefluxOnly_VmotorPi
  0,      // bDummy_0
  0,      // bDummy_1
  0,      // bDummy_2
  0,      // bDummy_3
  0,      // bDummy_4
  0       // bDummy_5
#endif
} ;

//***************************************************************************
// Locals
typedef struct
{
    UBYTE   ubDataSel ;
    FLOAT   flDrivePeakCurrent ;
    FLOAT   fl4Kt ;
    FLOAT   fl3LmotPn ;

    // DrivePeakCurrent e' il valore piu' basso tra Picco Azionamento e Picco Motore (init)
    SWORD   swDrivePeakCurrentShift ;
    SWORD   swDrivePeakCurrentMax ; 
    UBYTE   ubOnlyMatrix ;
    UBYTE   ubOnlyVMotor ;

#if CFG_DFLX_VMOTOR
    FLOAT   flPiEnableSpeed ;
#endif
} DFLX_RUNTIME ;
#ifdef _INFINEON_
static DFLX_RUNTIME huge sDflxRun ;
#else
static DFLX_RUNTIME sDflxRun ;
#endif // _infineon_

//***************************************************************************
// Local prototypes
static BOOL LiveParametersCheck(void);

//***************************************************************************
//
void DflxInit(SLONG slDrivePeakCurrent)
{
    FLOAT flDrivePeakCurrent ;
    SLONG slPeakCurrent ;

    // DrivePeakCurrent is the lower between drive and motor peak current
    flDrivePeakCurrent = (FLOAT)slDrivePeakCurrent / ARMS2IU_FLOAT ; // Arms

    if (flDrivePeakCurrent > sGlbMotorParameters.flCurrentPeak)
        sDflxRun.flDrivePeakCurrent = sGlbMotorParameters.flCurrentPeak ;
    else
        sDflxRun.flDrivePeakCurrent = flDrivePeakCurrent ;

    // code optimization
    slPeakCurrent = (SLONG)(sDflxRun.flDrivePeakCurrent * ARMS2IU_FLOAT) ; // i.u.
    sDflxRun.swDrivePeakCurrentShift = _ilog2((ULONG)slPeakCurrent / 32768) ;
    sDflxRun.swDrivePeakCurrentMax   = (SWORD)(slPeakCurrent >> sDflxRun.swDrivePeakCurrentShift) ;

    sDflx_Out.flags.b.bDefluxActive = FALSE ;
    sDflx_Out.flags.b.bMaxSpeedReachable = FALSE ;
	
    sDflx_Out.flags.b.bDefluxActive_Matrix = TRUE ;

    LiveParametersCheck();

    sDflx_Out.slDflxId = 0 ;
	sDflx_Out.slDflxId_Matrix = 0 ;

#if CFG_DFLX_VMOTOR
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // for the moment both deflux methods together are not allowed
    if ((BOOL)sDflx_Param.bDefluxOnly_Matrix)
    {
    	sDflx_Param.bDefluxOnly_VmotorPi = FALSE ;
    }
    else if ((BOOL)sDflx_Param.bDefluxOnly_VmotorPi)
    {
    	sDflx_Param.bDefluxOnly_Matrix = FALSE ;
    }
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    sDflx_Out.flags.b.bDefluxOnly_Matrix   = (BOOL)sDflx_Param.bDefluxOnly_Matrix ;
    sDflx_Out.flags.b.bDefluxOnly_VmotorPi = (BOOL)sDflx_Param.bDefluxOnly_VmotorPi ;

    sDflx_Out.flags.b.bDefluxActive_VmotorPi = FALSE ;
    sDflx_Out.flags.b.bVmotorPiLimitActive = FALSE ;

    sDflx_Out.flVdcBus = 0.0 ;
    sDflx_Out.flVMotorMax = 0.0 ;
    sDflx_Out.flPiRef = 0.0 ;
    sDflx_Out.flPiErr = 0.0 ;
    sDflx_Out.flPiIntegral = 0.0 ;
    sDflx_Out.flPiProportional = 0.0 ;
    sDflx_Out.flPiOut = 0.0 ;
    sDflx_Out.sPiIqLimit.slMax =  slPeakCurrent ; // SLONG_MAX_VALUE ;
    sDflx_Out.sPiIqLimit.slMin = -sDflx_Out.sPiIqLimit.slMax ;
	sDflx_Out.slDflxId_VmotorPi = 0 ;
#else
    sDflx_Out.flags.b.bDefluxOnly_Matrix = TRUE ;
#endif
}

//***************************************************************************
//
static BOOL LiveParametersCheck(void)
{
    // ----------------------------------------------------------------
	//runtime change of direct inductance and speedmax

    if (sGlbMotorParameters.flDirectInductance <= 0.0) 
        sDflx_Out.flDirectInductance = sGlbMotorParameters.flInductance ;
    else
        sDflx_Out.flDirectInductance = sGlbMotorParameters.flDirectInductance ;

    if (sDflx_Param.slDflxSpdMax <= 0l)
        sDflx_Out.slDflxSpdMax = (SLONG)(sGlbMotorParameters.flSpeedNominal * SPEED_RADS_K_CONVERSION) ;
    else
        sDflx_Out.slDflxSpdMax = sDflx_Param.slDflxSpdMax;

#if CFG_DFLX_VMOTOR
    // ***********
	if ((sDflx_Param.flMargin <= 0.0) || (sDflx_Param.flMargin > 1.0))
		sDflx_Param.flMargin = 1.0 ;

	if (sDflxVMotor8kHz.flMargin != sDflx_Param.flMargin)
		sDflxVMotor8kHz.flMargin  = sDflx_Param.flMargin ;

    // ***********
	if (sDflx_Param.flPiManualRefVal < 0.0)
		sDflx_Param.flPiManualRefVal  = sDflx_Out.flVdcBus * sDflxVMotor8kHz.flMargin / FLOAT_SQRT_OF_TWO ; ;

	if (sDflxVMotor8kHz.flPiManualRefVal != sDflx_Param.flPiManualRefVal)
		sDflxVMotor8kHz.flPiManualRefVal  = sDflx_Param.flPiManualRefVal ;

    // ***********
	if (sDflx_Param.flPiKi < 0.0)
		sDflx_Param.flPiKi = 0.0 ;

	if (sDflxVMotor8kHz.flPiKi != sDflx_Param.flPiKi)
		sDflxVMotor8kHz.flPiKi  = sDflx_Param.flPiKi ;

    // ***********
	if (sDflx_Param.flPiKp < 0.0)
		sDflx_Param.flPiKp = 0.0 ;

	if (sDflxVMotor8kHz.flPiKp != sDflx_Param.flPiKp)
		sDflxVMotor8kHz.flPiKp  = sDflx_Param.flPiKp ;

    // ***********
	if (sDflx_Param.flPiLimitMax < 0.0)
		sDflx_Param.flPiLimitMax = 0.0 ;

	if (sDflxVMotor8kHz.flPiLimitMax != sDflx_Param.flPiLimitMax)
		sDflxVMotor8kHz.flPiLimitMax  = sDflx_Param.flPiLimitMax ;

    // ***********
	if (sDflx_Param.flPiLimitMin > 0.0)
		sDflx_Param.flPiLimitMin = 0.0 ;

	if (sDflxVMotor8kHz.flPiLimitMin != sDflx_Param.flPiLimitMin)
		sDflxVMotor8kHz.flPiLimitMin  = sDflx_Param.flPiLimitMin ;

    // ***********
	if (sDflx_Param.flPiEnableSpeed <= 0.0)
		sDflx_Param.flPiEnableSpeed = 0.2 * sGlbMotorParameters.flSpeedNominal ; // 20% of nominal speed

	if (sDflxRun.flPiEnableSpeed != sDflx_Param.flPiEnableSpeed)
	{
		sDflxRun.flPiEnableSpeed = sDflx_Param.flPiEnableSpeed ;
		sDflxVMotor8kHz.uwPiEnableSpeed = (UWORD)((sDflxRun.flPiEnableSpeed * SPEED_RADS_K_CONVERSION * (FLOAT)sGlbMotorParameters.uwPoleNumbers) / 131072.0) ; // elec speed
	}

    // ***********
#endif

    return TRUE;
}

//***************************************************************************
//
void DlfxSlow(BOOL bDflxEnabled)
{
    static BOOL bFirstRun = TRUE ;
    BOOL    bSpeedReachable = TRUE ;
    FLOAT   flVdc, flIdMax ;
    FLOAT   flDflxSpdDelta, flDflxKneeSpd ;
    FLOAT   fl24Vdc_2, fl4Kt_2, fl3LmotPn_2, fl3ImaxLmotPn_2, flMotorIcc ;
    FLOAT   flSqrt ;
    UWORD   uwRow, uwCol ;
    DFLX_DATA *psDflxData2Set ;

    LiveParametersCheck();

    // se diverso da zero allora uso valore fisso per il Vdcbus
    if(sDflx_Param.swDflxVdc > 0)
        flVdc = (FLOAT)sDflx_Param.swDflxVdc / 10.0 ;
    else
#if (CFG_DFLX_VMOTOR)
    	flVdc = sDflx_Out.flVdcBus ;
#else
        flVdc = (FLOAT)*sDflx_In.pswActualVdc / 10.0 ;
#endif
    // optimizations
    flVdc     = flVdc * (FLOAT)sDflx_Param.uwKVOut / 1000.0 ; // V
    fl24Vdc_2 = 24.0 * flVdc * flVdc ;    // = 24 * V^2
    sDflxRun.fl4Kt     = 4.0 * sGlbMotorParameters.flKT ; // = 4 * Kt
    fl4Kt_2   = sDflxRun.fl4Kt * sDflxRun.fl4Kt ; // = 16 * Kt^2
    sDflxRun.fl3LmotPn = 3.0 * sDflx_Out.flDirectInductance * (FLOAT)sGlbMotorParameters.uwPoleNumbers ; // 3 * Lmot * Pn
    fl3LmotPn_2 = sDflxRun.fl3LmotPn * sDflxRun.fl3LmotPn ; // 9 * Lmot^2 * Pn^2

    // ----------------------------------------------------------------
    // calcolo la IdMax da usare: e' la piu' bassa tra corrente di 
    // corto circuito del motore e DrivePeakCurrent (valore piu' basso tra picco azionamento e picco motore (init))
    flMotorIcc = sDflxRun.fl4Kt / sDflxRun.fl3LmotPn ; // Arms; corrente di corto circuito del motore
    sDflx_Out.slMotorIcc = (SLONG)(flMotorIcc * ARMS2IU_FLOAT) ; // 1e-4Arms

    if (flMotorIcc > sDflxRun.flDrivePeakCurrent)
        flIdMax = sDflxRun.flDrivePeakCurrent ;
    else
        flIdMax = flMotorIcc ;

    // IMax e' SEMPRE la corrente DrivePeakCurrent (valore piu' basso tra picco azionamento e picco motore (init))
    fl3ImaxLmotPn_2 = fl3LmotPn_2 * sDflxRun.flDrivePeakCurrent * sDflxRun.flDrivePeakCurrent ;  // 9 * Lmot^2 * Pn^2 * IMax^2 (ottimizzazione)                         
    // ----------------------------------------------------------------  
    // calcolo la velocita' minima a cui iniziare a deflussare
    flDflxKneeSpd  = DEFLUX_SQRT_24 * flVdc /sqrt(fl4Kt_2 + fl3ImaxLmotPn_2) ;   // rad/s
    flDflxSpdDelta = (FLOAT)sDflx_Out.slDflxSpdMax / SPEED_RADS_K_CONVERSION - flDflxKneeSpd ; // rad/s
    sDflx_Out.slDflxKneeSpd = (SLONG)(flDflxKneeSpd * SPEED_RADS_K_CONVERSION) ; // i.u.

    // ----------------------------------------------------------------
    if (bDflxEnabled && sDflx_Out.flags.b.bDefluxOnly_Matrix)
    { // deflussaggio abilitato: riempo alternativamente le matrici per il fast task 
        psDflxData2Set = &sDlfxData[sDflxRun.ubDataSel];

        // riempo la struttura dati da far usare al fast task
        psDflxData2Set->swSpdMin     = (SWORD)((sDflx_Out.slDflxKneeSpd * sGlbMotorParameters.uwPoleNumbers) / 131072) ;  // i.u. elec speed (131072 = 65536 * 2 (2=divisore per avere coppie polari))
        psDflxData2Set->swSpdMax     = (SWORD)((sDflx_Out.slDflxSpdMax  * sGlbMotorParameters.uwPoleNumbers) / 131072) ;  // i.u. elec speed
        psDflxData2Set->swSpdDelta   = (SWORD)(psDflxData2Set->swSpdMax - psDflxData2Set->swSpdMin) ; // i.u. elec speed
        psDflxData2Set->swIqRefShift = sDflxRun.swDrivePeakCurrentShift ;
        psDflxData2Set->swIqRefMax   = sDflxRun.swDrivePeakCurrentMax ;
           
        // calcolo la matrice ed il vettore
        for (uwRow = 0; uwRow < MATRIX_DIMENSION; uwRow++)
        {
            FLOAT   flOmega, flOmega2 ;

            // calcolo il vettore di IqLimite
            flOmega = (flDflxSpdDelta * uwRow) / MATRIX_DIMENSION_1 + flDflxKneeSpd ; // rad/s
            flOmega2 = flOmega  * flOmega ;

            // evito divisioni per zero 
            if (flOmega == 0.0)  
                sDflx_Out.flIqLimit2Use[uwRow] = sDflxRun.flDrivePeakCurrent ;
            else
            {   // calcolo la IqLimite sulla base di IdLimite calcolato su DrivePeakCurrent (= valore piu' basso tra Ipicco drive ed Ipicco motore (init))      
                FLOAT flIdLim ;

                flIdLim = 0.5 * (fl24Vdc_2 - (fl4Kt_2 + fl3ImaxLmotPn_2) * flOmega2) / (sDflxRun.fl4Kt * sDflxRun.fl3LmotPn * flOmega2) ; // A

                // IdLim sempre [-flIdMax; 0]
                if (flIdLim > 0.0)
                    flIdLim = 0.0 ;
                else if (flIdLim < -flIdMax)
                    flIdLim = -flIdMax ;

                flSqrt = fl24Vdc_2 - flOmega2 * (fl4Kt_2 + flIdLim * (2.0 * sDflxRun.fl4Kt * sDflxRun.fl3LmotPn + fl3LmotPn_2 * flIdLim)) ;
                
                // evito radici negative
                if (flSqrt > 0.0)
                {   // il vettore floating point mi serve per calcolare la matrice dopo
                    sDflx_Out.flIqLimit2Use[uwRow] = sqrt(flSqrt) / (sDflxRun.fl3LmotPn * flOmega) ; // A
                    
                    if (sDflx_Out.flIqLimit2Use[uwRow] > sDflxRun.flDrivePeakCurrent)
                        sDflx_Out.flIqLimit2Use[uwRow] = sDflxRun.flDrivePeakCurrent ;
                }
                else
                {   // radice negativa, limite di corrente = 0, velocita' massima non raggiungibile
                    sDflx_Out.flIqLimit2Use[uwRow] = 0.0 ;
                    bSpeedReachable = FALSE ;
                }
            }

            // converto a 16bit per ottimizzare i tempi in fast task
            psDflxData2Set->hpsMatrix->swIqRefLimMaxVector[uwRow] = (SWORD)((SLONG)(ARMS2IU_FLOAT * sDflx_Out.flIqLimit2Use[uwRow]) >> psDflxData2Set->swIqRefShift) ; // i.u. shiftate su Imax
           
            // calcolo la matrice di Id da fornire
            for (uwCol = 0; uwCol < MATRIX_DIMENSION; uwCol++)
            {
                SWORD swElem ;

                // evito divisioni per zero
                if (flOmega == 0.0)
                    swElem = 0 ;
                else
                {
                    flSqrt = (sDflx_Out.flIqLimit2Use[uwRow] * uwCol) / MATRIX_DIMENSION_1 ; // ottimizzazione    
                    flSqrt = fl24Vdc_2 - fl3LmotPn_2 * flSqrt * flSqrt * flOmega2 ;

                    // verifico cosa fare sulla base del radicando
                    if (flSqrt >= 0.0)
                    {   // radice strettamente positiva: esiste una soluzione
                        swElem = (SWORD)((SLONG)(ARMS2IU_FLOAT * (sqrt(flSqrt) / (sDflxRun.fl3LmotPn * flOmega) - flMotorIcc)) >> psDflxData2Set->swIqRefShift) ;
                        
                        // voglio solo Id negativa
                        if (swElem > 0)
                            swElem = 0 ;
                    }
                    else  
                         swElem = -(SWORD)((SLONG)(flMotorIcc * ARMS2IU_FLOAT)  >> psDflxData2Set->swIqRefShift) ;  // radice <= 0, non esiste soluzione: clippo al valore di corrente di corto circuito del motore
                    
                    // clippo al valore (negativo) di DrivePeakCurrent = valore piu' basso tra corrente di picco del drive e corrente di picco del motore (init)
                    if (swElem < -sDflxRun.swDrivePeakCurrentMax)
                        swElem = -sDflxRun.swDrivePeakCurrentMax ;
                }
                psDflxData2Set->hpsMatrix->swIdMatrix[uwRow][uwCol] = swElem;
            }
        }

        sDflx_Out.flags.b.bMaxSpeedReachable = bSpeedReachable ;

        // se velocita' richiesta e' raggiungibile swappo struttura, altrimenti mantengo la precedente
        // per evitare di avere limiti di Iq pari a zero e quindi perdere la controllabilita' dell'asse
        if(bFirstRun || bSpeedReachable)
        {
            psDflxData = psDflxData2Set;
            sDflxRun.ubDataSel ^= 1 ;
            bFirstRun = FALSE ;
        }

        // se non raggiungibile e potenza abilitata segnalo warning
        if((!bSpeedReachable) && sDflx_In.psPowerStageStatus->b.bVoltageEnabled)
            atomic_long_set_bits( &ulSystemWarnings, SYSTEMWARNINGS_DEFLUX_SPEEDNOTREACHABLE );
        else
            atomic_long_clear_bits( &ulSystemWarnings, SYSTEMWARNINGS_DEFLUX_SPEEDNOTREACHABLE );
    }
    else
        atomic_long_clear_bits( &ulSystemWarnings, SYSTEMWARNINGS_DEFLUX_SPEEDNOTREACHABLE );


#if CFG_DFLX_VMOTOR
    if (bDflxEnabled && sDflx_Out.flags.b.bDefluxOnly_VmotorPi)
    {
		flSqrt = sDflxRun.flDrivePeakCurrent * sDflxRun.flDrivePeakCurrent - (FLOAT)sDflx_Out.slDflxId * (FLOAT)sDflx_Out.slDflxId / ARMS2IU_FLOAT_SQUARED ;
		if (flSqrt < 0.0)
			sDflx_Out.sPiIqLimit.slMax = 0 ;
		else
			sDflx_Out.sPiIqLimit.slMax = (SLONG)(ARMS2IU_FLOAT * sqrt(flSqrt)) ;
    }
    else
    {
    	sDflx_Out.sPiIqLimit.slMax = SLONG_MAX_VALUE ;
    }
	sDflx_Out.sPiIqLimit.slMin = -sDflx_Out.sPiIqLimit.slMax ;
#endif

}
