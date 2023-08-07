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
// Compiler Option
#pragma GCC optimize (2)

#include <math.h>
#include <stdlib.h>
#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonUtility.h"
#include "common\MathFunctions.h"
#include "common\DspFunctions.h"
#include "drive\DefluxRT.h"

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
DFLX_DATA * psDflxData=sDlfxData;

//***************************************************************************
//
void Dflx8KHz(BOOL bReferenceEnabled, SLONG * pslIdRef, SLONG * pslIqRef)
{
    SWORD   swAbsActualSpd, swActualSpdDelta, swFractionSpeed ;
    SWORD   swActualAbsIqRef, swFractionIqRef, swIqRefLim, swMatrixInterpolation[3] ;
    UWORD   uwValue2Use, uwIdxSpd, uwIdxIq ;
    SLONG   slDflxId ;
#ifdef _INFINEON_
    SWORD huge * hpsMatrix ;
#else
    SWORD * hpsMatrix ;
#endif

    // faccio il valore assoluto della velocita' elettrica (calcolata da qualche altra parte)
    swAbsActualSpd = abs(*sDflx_In.pswActualSpd) ; // elec speed  
    
    // per il deflussaggio clippo alla massima velocita' (elettrica)
    if(swAbsActualSpd > psDflxData->swSpdMax)
        swAbsActualSpd = psDflxData->swSpdMax ; 

    swActualSpdDelta = swAbsActualSpd - psDflxData->swSpdMin ; // delta (elec) speed

    // ---------------------------------------------------------------------------- 
    if ((swActualSpdDelta < 0) ||!bReferenceEnabled)
    {   // sono fuori dalla zona di deflussaggio o sono disabilitato
        slDflxId = 0 ;     
        sDflx_Out.sDefluxIqLimit.slMax = SLONG_MAX_VALUE ;
        sDflx_Out.sDefluxIqLimit.slMin = -sDflx_Out.sDefluxIqLimit.slMax ;
        sDflx_Out.flags.b.bDefluxActive = FALSE ;
    }
    else
    {   // sto deflussando, trovo gli indici della matrice e faccio le interpolazioni
        sDflx_Out.flags.b.bDefluxActive = TRUE ;

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
            slDflxId = 0 ;
        else
            slDflxId = _sint16_asl_32(swMatrixInterpolation[2], psDflxData->swIqRefShift) ; // 1e-4A 

        sDflx_Out.sDefluxIqLimit.slMax = _sint16_asl_32(swIqRefLim, psDflxData->swIqRefShift) ; // 1e-4A 
        sDflx_Out.sDefluxIqLimit.slMin = -sDflx_Out.sDefluxIqLimit.slMax ;
        // ---------------------------------------------------------------------------- 
    }

    // quick Iq limiting
    if(*pslIqRef > sDflx_Out.sDefluxIqLimit.slMax)
        *pslIqRef =  sDflx_Out.sDefluxIqLimit.slMax;
    else if(*pslIqRef < sDflx_Out.sDefluxIqLimit.slMin)
        *pslIqRef = sDflx_Out.sDefluxIqLimit.slMin;        

    *pslIdRef += slDflxId ;
}
