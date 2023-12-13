/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2010, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : DefluxRT.c                                                 */
/* Author      : Fabio Terrile; Cristiano Tognetti; Marco Calvini           */
/*                                                                          */
/* Description : Field weakening task time-optimized real time module       */
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
// Locals
#ifdef _INFINEON_
DFLX_MDATA huge sDlfxMData[2];
DFLX_DATA sdata sDlfxData[2]=
#else
DFLX_MDATA sDlfxMData[2];
DFLX_DATA sDlfxData[2]=
#endif // _infineon_
{
    {&sDlfxMData[0]},
    {&sDlfxMData[1]},
};
DFLX_DATA   * psDflxData=sDlfxData;

#if CFG_DFLX_VMOTOR
DFLX_VMOTOR  sDflxVMotor8kHz ;
DFLX_VMOTOR *psDflxVMotor = &sDflxVMotor8kHz;
#endif

//***************************************************************************
//
void Dflx8KHz(BOOL bReferenceEnabled, SLONG * pslIdRef, SLONG * pslIqRef)
{
    SWORD   swAbsActualSpd, swActualSpdDelta, swFractionSpeed ;
    SWORD   swActualAbsIqRef, swFractionIqRef, swIqRefLim, swMatrixInterpolation[3] ;
    UWORD   uwValue2Use, uwIdxSpd, uwIdxIq, uwAbsActualSpd ;
#ifdef _INFINEON_
    SWORD huge * hpsMatrix ;
#else
    SWORD * hpsMatrix ;
#endif

    // absolute value of electrical speed (calculated somewhere else)
    uwAbsActualSpd = abs(*sDflx_In.pswActualSpd) ; // elec speed

    // ----------------------------------------------------------------------------
    // for deflux-matrix clip the speed (electrical) to the max speed set by user
    swAbsActualSpd = (SWORD)uwAbsActualSpd ;
    if(swAbsActualSpd > psDflxData->swSpdMax)
        swAbsActualSpd = psDflxData->swSpdMax ; 

    swActualSpdDelta = swAbsActualSpd - psDflxData->swSpdMin ; // delta (elec) speed

    // ---------------------------------------------------------------------------- 
    if ((swActualSpdDelta < 0) || (!bReferenceEnabled)
#if CFG_DFLX_VMOTOR
        || sDflx_Out.flags.b.bDefluxOnly_VmotorPi
#endif
	    )
    {   // sono fuori dalla zona di deflussaggio o sono disabilitato
    	sDflx_Out.slDflxId_Matrix = 0 ;
        sDflx_Out.sMatrixIqLimit.slMax =  SLONG_MAX_VALUE ;
        sDflx_Out.sMatrixIqLimit.slMin = -sDflx_Out.sMatrixIqLimit.slMax ;
        sDflx_Out.flags.b.bDefluxActive_Matrix = FALSE ;
    }
    else
    {   // sto deflussando, trovo gli indici della matrice e faccio le interpolazioni
        sDflx_Out.flags.b.bDefluxActive_Matrix = TRUE ;

        // ---------------------------------------------------------------------------- 
        // riduzione da 32bit a 16bit delle variabili a 32bit (solo la corrente)   
        // corrente diversa da zero SOLO SE segno corrente e velocita' concordi
        if ((BOOL)(*pslIqRef < 0) != (BOOL)(*sDflx_In.pswActualSpd < 0)) 
            swActualAbsIqRef = 0 ;
        else
            swActualAbsIqRef = abs(_sint32_asr_16(*pslIqRef, psDflxData->swIqRefShift)) ; 
        // ---------------------------------------------------------------------------- 

        // ---------------------------------------------------------------------------- 
        if (swActualSpdDelta >= psDflxData->swSpdDelta)
            uwValue2Use = 32767 ; // 100% = 32767 
        else
            uwValue2Use = (UWORD)((SLONG)swActualSpdDelta * 32767 / psDflxData->swSpdDelta) ;

        // righe della matrice da usare 
        uwIdxSpd = uwValue2Use >> MATRIX_SHIFT ; 
        swFractionSpeed = (SWORD)(uwValue2Use & MATRIX_MASK_1) ;
        
        // mi calcolo subito la MaxIqRef da usare: interpolo sul vettore IqRefMax             
        swIqRefLim = _interpolation(&psDflxData->hpsMatrix->swIqRefLimMaxVector[uwIdxSpd], swFractionSpeed) ;     
        // ---------------------------------------------------------------------------- 

        // ---------------------------------------------------------------------------- 
        // clippo la IqRef ad IqRefLim (appena calcolato) e quindi interpolo sulla IqRefLim non sulla IqRefMax
        if (swActualAbsIqRef >= swIqRefLim)
            uwValue2Use = 32767 ; // 100% = 32767 
        else
            uwValue2Use = (UWORD)((SLONG)swActualAbsIqRef * 32767 / swIqRefLim) ;

        // colonne della matrice da usare 
        uwIdxIq = uwValue2Use >> MATRIX_SHIFT ;
        swFractionIqRef = (SWORD)(uwValue2Use & MATRIX_MASK_1) ;

        // 1a interpolazione sulla corrente (sulla riga SpdIdx[0])
         hpsMatrix = &psDflxData->hpsMatrix->swIdMatrix[uwIdxSpd][uwIdxIq];
         swMatrixInterpolation[0] = _interpolation(hpsMatrix, swFractionIqRef) ;

        // 2a interpolazione sulla corrente (sulla riga SpdIdx[1])
#ifdef _INFINEON_
         hpsMatrix = (HPSWORD)&(((SWORD xhuge *)hpsMatrix)[MATRIX_DIMENSION]);
#else
         hpsMatrix = (HPSWORD)&(((SWORD *)hpsMatrix)[MATRIX_DIMENSION]);
#endif
         swMatrixInterpolation[1] = _interpolation(hpsMatrix, swFractionIqRef) ;

        // 3a interpolazione: sulla velocita' 
         swMatrixInterpolation[2] = _interpolation(swMatrixInterpolation, swFractionSpeed) ;
                
        // riconverto la corrente in unita' interne; SOLO valori negativi  
        if (swMatrixInterpolation[2] >= 0)
        	sDflx_Out.slDflxId_Matrix = 0 ;
        else
        	sDflx_Out.slDflxId_Matrix = _sint16_asl_32(swMatrixInterpolation[2], psDflxData->swIqRefShift) ; // 1e-4A

        sDflx_Out.sMatrixIqLimit.slMax = _sint16_asl_32(swIqRefLim, psDflxData->swIqRefShift) ; // 1e-4A
        sDflx_Out.sMatrixIqLimit.slMin = -sDflx_Out.sMatrixIqLimit.slMax ;
        // ---------------------------------------------------------------------------- 
    }

// ###################################################################################################
// ###################################################################################################
#if CFG_DFLX_VMOTOR
    // Filter VdcBus @2ms; 0.00625 = 0.0625 / 10.0
    sDflx_Out.flVdcBus = 0.9375 * sDflx_Out.flVdcBus + 0.00625 * (FLOAT)(*sDflx_In.pswActualVdc) ; // V

    // Filter VdcMotor @1ms; 0.0125 = 0.125 / 10.0
    sDflx_Out.flPiFbk = 0.875 * sDflx_Out.flPiFbk + 0.0125 * (FLOAT)(*sDflx_In.pswActualVMotor) ; // V

    /* VmotorMax = (VdcBus / SQRT(2)) * Margin */
    sDflx_Out.flVMotorMax = sDflx_Out.flVdcBus * psDflxVMotor->flMargin / FLOAT_SQRT_OF_TWO ;

    if (bReferenceEnabled && (uwAbsActualSpd > psDflxVMotor->uwPiEnableSpeed) && sDflx_Out.flags.b.bDefluxOnly_VmotorPi)
    {
    	sDflx_Out.flags.b.bDefluxActive_VmotorPi = TRUE  ;

    	if (psDflxVMotor->flPiManualRefVal != 0.0)
    		sDflx_Out.flPiRef = psDflxVMotor->flPiManualRefVal ;
    	else
    		sDflx_Out.flPiRef = sDflx_Out.flVMotorMax ;
    }
    else
    {
    	sDflx_Out.flags.b.bDefluxActive_VmotorPi = FALSE ;
    	sDflx_Out.flPiRef = sDflx_Out.flPiFbk ;
    	sDflx_Out.flPiIntegral = 0.0 ;
    }

    sDflx_Out.flPiErr = sDflx_Out.flPiRef - sDflx_Out.flPiFbk ;
    sDflx_Out.flPiProportional = sDflx_Out.flPiErr * psDflxVMotor->flPiKp ;

    if ((!sDflx_Out.flags.b.bVmotorPiLimitActive) ||
        (sDflx_Out.flags.b.bVmotorPiLimitActive && (sDflx_Out.flPiOut <= psDflxVMotor->flPiLimitMin) && (sDflx_Out.flPiErr > 0.0)))
    {	/* integral part calculated when:
        1) output is not clipped to the max/min value
        2) output is clipped to the negative min value, but error is positive (integral part can be only discharged) */
    	if (psDflxVMotor->flPiKi == 0.0)
    		sDflx_Out.flPiIntegral = 0.0 ;
    	else
        {
    		sDflx_Out.flPiIntegral = sDflx_Out.flPiIntegral + sDflx_Out.flPiErr * psDflxVMotor->flPiKi ;

            if (sDflx_Out.flPiIntegral > psDflxVMotor->flPiLimitMax)
            	sDflx_Out.flPiIntegral = psDflxVMotor->flPiLimitMax ;
            else if (sDflx_Out.flPiIntegral < psDflxVMotor->flPiLimitMin)
            	sDflx_Out.flPiIntegral = psDflxVMotor->flPiLimitMin ;
        }
    }

    sDflx_Out.flPiOut = sDflx_Out.flPiProportional + sDflx_Out.flPiIntegral ;

     /* stop integral if drive enabled and limit reached */
    if (sDflx_Out.flPiOut > psDflxVMotor->flPiLimitMax)
    {
    	sDflx_Out.flPiOut = psDflxVMotor->flPiLimitMax ;
    	sDflx_Out.flags.b.bVmotorPiLimitActive = bReferenceEnabled ;
    }
    else if (sDflx_Out.flPiOut < psDflxVMotor->flPiLimitMin)
    {
    	sDflx_Out.flPiOut = psDflxVMotor->flPiLimitMin ;
    	sDflx_Out.flags.b.bVmotorPiLimitActive = bReferenceEnabled ;
    }
    else
    	sDflx_Out.flags.b.bVmotorPiLimitActive = FALSE ;

    if (!sDflx_Out.flags.b.bDefluxActive_VmotorPi)
    	sDflx_Out.flPiOut = 0.0 ;

    sDflx_Out.slDflxId_VmotorPi = (SLONG)(sDflx_Out.flPiOut * ARMS2IU_FLOAT) ;
#endif
// ###################################################################################################
// ###################################################################################################

#if CFG_DFLX_VMOTOR
    sDflx_Out.flags.b.bDefluxActive = sDflx_Out.flags.b.bDefluxActive_Matrix || sDflx_Out.flags.b.bDefluxActive_VmotorPi ;

    // use the lower Iqlimit
    if (sDflx_Out.sMatrixIqLimit.slMax > sDflx_Out.sPiIqLimit.slMax)
    	sDflx_Out.sDefluxIqLimit.slMax = sDflx_Out.sPiIqLimit.slMax ;
    else
    	sDflx_Out.sDefluxIqLimit.slMax = sDflx_Out.sMatrixIqLimit.slMax ;

    if (sDflx_Out.sMatrixIqLimit.slMin < sDflx_Out.sPiIqLimit.slMin)
    	sDflx_Out.sDefluxIqLimit.slMin = sDflx_Out.sPiIqLimit.slMin ;
    else
    	sDflx_Out.sDefluxIqLimit.slMin = sDflx_Out.sDefluxIqLimit.slMin ;

    sDflx_Out.slDflxId = sDflx_Out.slDflxId_Matrix + sDflx_Out.slDflxId_VmotorPi ;

    if(sDflx_Out.slDflxId > 0)
    	sDflx_Out.slDflxId = 0 ; // only negative current allowed

#else
    sDflx_Out.flags.b.bDefluxActive = sDflx_Out.flags.b.bDefluxActive_Matrix ;
    sDflx_Out.sDefluxIqLimit.slMax = sDflx_Out.sMatrixIqLimit.slMax ;
    sDflx_Out.sDefluxIqLimit.slMin = sDflx_Out.sMatrixIqLimit.slMin ;

    sDflx_Out.slDflxId = sDflx_Out.slDflxId_Matrix ;
#endif

    // quick Iq limiting
    if(*pslIqRef > sDflx_Out.sDefluxIqLimit.slMax)
        *pslIqRef =  sDflx_Out.sDefluxIqLimit.slMax;
    else if(*pslIqRef < sDflx_Out.sDefluxIqLimit.slMin)
        *pslIqRef = sDflx_Out.sDefluxIqLimit.slMin;

    *pslIdRef += sDflx_Out.slDflxId ;
}
