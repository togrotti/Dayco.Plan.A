/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright $)AB) 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EncoderManager.c                                           */
/* Author      : Fabio Terrile, Cristiano Tognetti                          */
/*                                                                          */
/* Description : Defines common handling to differents encoder sources and  */
/*               combinations												*/
/*                                                                          */
/****************************************************************************/
#include <stdlib.h>	// to use labs()

#include "common\CommonUtility.h"
#include "drive\AxM-E-Defines.h"
#include "system\SysAppGlobals.h"
#include "system\SystemAlarms.h"
#include "system\SysLogManagement.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "drive\HardwareUnitID.h"
#include "system\SysAppFpgaOptCodes.h"
#include "EncoderManager.h"

#include "drive\EncoderEFSeek.h"
#include "drive\EnDatEncoder.h"
#include "drive\HallEncoder.h"
#include "drive\IncrementalEncoder.h"
#include "drive\SincosEncoder.h"
#include "drive\BackEmfEncoder.h"
#include "drive\HiperfaceEncoder.h"
#include "drive\NikonEncoder.h"
#include "drive\TAMAGAWAEncoder.h"
#include "drive\BissEncoder.h"

#include "common\TaskScheduler.h"
#include "common\Int64Functions.h"
#include "common\DspFunctions.h"
#include "fpga\FpgaHandler.h"

#include "drive\DriveTaskController.h"
#include "drive\MotionController.h"
#include "common\IOManager.h"

//#include <intrins.h>
#include <string.h>
#include <math.h>

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#if defined(_CRS_DBG)
#if (FALSE) //CRS_DBGDSK
#pragma GCC optimize (0) // crs_dbg
#else
#pragma GCC optimize (2)
#endif
#else
#pragma GCC optimize (2)
#endif

//****************************************************************************
// Defines

#define FILTSPD_CNTOVR   (1<<(ENCMGR_FILTEREDSPEED_NBIT))

#define filterspeed_calc() \
        { \
            if(++sEncMgrRun.uwFiltSpdCounter>=FILTSPD_CNTOVR) \
            { \
                if(sEncMgrRun.uwFiltSpdCounter==FILTSPD_CNTOVR) \
                    _uint64_sub((UQWRD *)&sEncMgrRun.sqFiltSpdPrevPosition, (UQWRD *)&sEm_Fbk2CntrLoop.sEncData.sqPostn, (UQWRD *)&sEncMgrRun.sqFiltSpdPrevPosition); \
                if(!bSysStatFieldbusSyncing) \
                { \
                    sEncMgrRun.uwFiltSpdCounter=0; \
                    _sint64_shr(&sEncMgrRun.sqFiltSpdPrevPosition, ENCMGR_FILTEREDSPEED_NBIT); \
                    sEm_Fbk2CLExt.slFilteredSpeed = sEncMgrRun.sqFiltSpdPrevPosition.lo; \
                    sEncMgrRun.sqFiltSpdPrevPosition.lo = sEm_Fbk2CntrLoop.sEncData.sqPostn.lo; \
                    sEncMgrRun.sqFiltSpdPrevPosition.hi = sEm_Fbk2CntrLoop.sEncData.sqPostn.hi; \
                    if(labs(sEm_Fbk2CLExt.slFilteredSpeed)>sEncMgrRun.slOverSpeed && !sEncMgrRun.sErrorSent.b.bOverSpeedFault && bSysStatPowerEnabled) \
                    { \
                        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_MOTOR_FAIL, SYSTEMALARMS_SUBCODE_MT_OVERSPEED, TRUE); \
                        (*sEm_EncMngrIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL); \
                        sEncMgrRun.sErrorSent.b.bOverSpeedFault=TRUE; \
                    } \
                } \
            } \
        }

// electrical angle feedforward calculated every 125us
// experimental feed forward periods:
// Endat    = (21000/16384) * 125us = 160.22us 
// Resolver = (17500/16384) * 125us = 133.51us
// Sincos   = (13500/16384) * 125us = 103.00us

#define elecangleff_calc() \
        { \
           sEncMgrRun.swElecAngleFF = _sint16_offsetscale_16(*sEncMgrRun.pswElecSpeed2Use, 0, sEncMgrRun.swElecAngleFFScaling); \
        }

#define maxabsreldiff_verify(abs,rel) \
        { \
            UWORD uwDiff; \
            uwDiff=(UWORD)((abs-rel)>>16); \
            if((SWORD)uwDiff<0) \
                uwDiff=(UWORD)-(SWORD)uwDiff; \
            if(uwDiff>sEm_EncMngrParam.uwMaxAbsRelDiff) \
            { \
                SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_ENCODERMANAGER_FAIL, SYSTEMALARMS_SUBCODE_ENCMGR_ABSRELDIFFFAIL, bSysStatPowerEnabled) ; \
                sEncMgrRun.sErrorSent.b.bMaxDiffFault=TRUE; \
                sEm_MainEnc.ubStatus=ENCMGR_FATAL_FAULT; \
            } \
        }

#define OVERSPEED_RATIO     1.20        // 120% of nominal speed

//****************************************************************************
// General global variables

ENCMGR_IN                sEm_EncMngrIn ;
ENCMGR_OUT               sEm_EncMngrOut;

ENCMGR_PARAMS            sEm_EncMngrParam ;
MOTPRM_PARAMETERS        sEm_LastValidEPlate ;

ENCMGR_SPACEFEEDBACK     sEm_MainAbsEnc ;
ENCMGR_SPACEFEEDBACK     sEm_MainRelEnc ;
ENCMGR_SPACEFEEDBACK     sEm_MainEnc ;
ENCMGR_SPACEFEEDBACK     sEm_AuxEnc ;
ENCMGR_SPACEFEEDBACK     sEm_Fbk2CntrLoop ;
ENCMGR_SPACEFB_EXT       sEm_Fbk2CLExt ;

//****************************************************************************
// PLC-only global variables

ENCMGR_SPACEFEEDBACK     sEm_PlcSimulationEnc;
ENCMGR_SPACEFEEDBACK     sEm_PlcFbk2CntrLoop ;
ENCMGR_SPACEFB_EXT       sEm_PlcFbk2CLExt ;

//****************************************************************************
// Default parameters

const ENCMGR_PARAMS  sEm_EncMngrDefParam=
{
    {1,1,1,1,1,0,0,0,0,0,0,0
#if (CFG_ENCMGR_OPENLOOP)
    ,0
#endif
    },// position, speed, acceleration, el ang from main enc, dis if rel fail
#ifndef _HW_CT

#ifdef _HW_AXS_DAYCO22KW
    0,
    ENCODER_TYPE_ABS_ANALOG,
#else
    415,                    // Encoder Supply Voltage = 5.2V
    ENCODER_TYPE_ABS_ENDAT, // main abs encoder to use
#endif // _hw_axs_dayco22kw
#else
    0,
    ENCODER_TYPE_ABS_ANALOG,
#endif
    ENCODER_TYPE_NULL,      // auxiliary encoder to use
    {0,0},                  // Encoder Mechanical Offset
    500,                    // Venc Delay
    ENCODER_TYPE_NULL,      // main rel encoder to use 
    1820,                   // max angle diff, default 10deg
    ENCODER_TYPE_NULL,      // simulation encoder selection
};

//****************************************************************************
// Data structures
#if (CFG_ENCMGR_OPENLOOP)
typedef struct{
	union {
		struct {
			UWORD bHookInTransition : 1 ; // b.0
		} b ;
		UWORD w ;
	} flags ;

	ENCMGR_SPACEFEEDBACK sEncoder ;
	SLONG slPostnOPL ;
	SLONG slSpeedOPL ;
	SWORD swElecTurns ;
	ULONG ulMechAdder ;
	ULONG ulElecAngle ;
	ULONG ulMechSteps ;
	UWORD uwPolePairs ;
} OPEN_LOOP ;
#endif

typedef struct {
    union {
        struct {
            UWORD bEncFatalFault    : 1 ; // b.0
            UWORD bEncNonFatalFault : 1 ; // b.1
            UWORD bMaxDiffFault     : 1 ; // b.2
            UWORD bEfsFault         : 1 ; // b.3
            UWORD bOverSpeedFault   : 1 ; // b.4
        } b ;
        UWORD w ;
    } sErrorSent ;
    union {
        struct {
            UWORD bPlcInput         : 1 ; // b.0
            UWORD bMainCombine      : 1 ; // b.1
            UWORD bCritFbFromAux    : 1 ; // b.2
            UWORD bPrevPSState      : 1 ; // b.3
            UWORD bEFSInProgress    : 1 ; // b.4
            UWORD bBEMFHook         : 1 ; // b.5
            UWORD bAbsFbFromAux     : 1 ; // b.6
            UWORD bEnSinCosHw       : 1 ; // b.7
            UWORD bDisFaultMgm      : 1 ; // b.8
            UWORD bFreeBit09        : 1 ; // b.9
            UWORD bExecAbsRelDiff   : 1 ; // b.10
            UWORD bDisAbsAfterValid : 1 ; // b.11
            UWORD bMainEncPresent   : 1 ; // b.12
            UWORD bSpeedFilter      : 1 ; // b.13
            UWORD bEnIncrementalHw  : 1 ; // b.14
            UWORD bFreeBit15        : 1 ; // b.15
        } b ;
        UWORD w ;
    } sFlags ;

    SQWRD *psqPosition2Use ;
    SQWRD *psqMechAbsOffset2Use ;
    UBYTE *pubStatus2Use ;

    SLONG *pslSpeed2Use ;
    SLONG *pslAccel2Use ;
    UWORD *puwElecAngle2Use ;

    void (* pfMainAbsEACorrect)(UWORD);
    void (* pfMainRelEACorrect)(UWORD);
    void (* pfMainRelAPCorrect)(SQWRD);
    BOOL (* pfMainRelGetSnapshot)(ULONG *);

    EFS_RUNTIME sEfsRun;

    SBYTE sbExportFault;
    UBYTE ubSimStatus;

    SQWRD sqFiltSpdPrevPosition ;
    UWORD uwFiltSpdCounter ;

    SWORD swElecAngleFF ;
    SWORD swElecAngleFFScaling ;

    SLONG slOverSpeed ;

    UWORD uwMotorPoleNumbers ;

    SWORD *pswElecSpeed2Use ;

    UWORD * puwMainElecAngle ;
    UWORD * puwMainDeltaElecAngle ;
    SWORD * pswMainElecSpeed ;
    SQWRD * psqMainPos ;
    SLONG * pslMainSpd ;
    SLONG * pslMainAcc ;

    ENCMGR_PARFLAGS prevflags;

    UWORD uwVEncVoltage;

#if (CFG_ENCMGR_OPENLOOP)
    OPEN_LOOP sOpenLoop ;
#endif
} ENCMGR_RUNTIME ;

//****************************************************************************
// Locals

static ENCMGR_RUNTIME  sEncMgrRun ;

//****************************************************************************
// Local functions

static BOOL eplate(void);
static BOOL initcore(void);
static BOOL inithook(void);
static void rtflagsapply();
static BOOL initplc(void);
static BOOL initplcoptions(void);

static void setvencoder(void);
static void adjustvencoder(void);
static void turnoffvencoder(void);
#ifdef _HW_DC
static void setauxvencoder(void);
static void turnoffauxvencoder(void);
#endif

static BOOL task8kHzminimal(void) ;
static BOOL task8kHzfull(void) ;
static BOOL hook8kHz(void) ;
static BOOL noencoderinit(ENCMGR_SPACEFEEDBACK *sNoEncoder) ;
static void slowtask(void);
static BOOL paramcheck(void);
static BOOL elecanglefftime(void);
static BOOL absreldiffcheck(void);

#if CFG_ENCMGR_OPENLOOP
static BOOL OpenLoopEncInit(void) ;
static BOOL OpenLoopEnc8KHz(GLB_IREF * psIRefIn, GLB_IREF * psIRefOut) ;
static void OpenLoopEncResetOut(void) ;
static void OpenLoopEncZeroOut(void) ;
static void OpenLoopEncData(void) ;
#endif

//***************************************************************************
// Initialization entry point

BOOL Em_EncMngrInit(UWORD uwTaskParam)
{
    switch(uwTaskParam)
    {
        case ENCMGR_INIT_EPLATE:
            return eplate();

        case ENCMGR_INIT_PLCOPTIONS:
            return initplcoptions();

        case ENCMGR_INIT_PLC:
            return initplc();

        case ENCMGR_INIT_CORE:
            return initcore();

        case ENCMGR_INIT_HOOK:
            return inithook();

        default:
            assert(FALSE);
    }

    return FALSE;
}

//***************************************************************************
// Setup Encoder voltage

static void setvencoder(void)
{
    UWORD uwVoltage = sEm_EncMngrParam.uwVEncVoltage;

    sEncMgrRun.uwVEncVoltage = uwVoltage ;

#ifdef _INFINEON_
        // if controlboard >= 6.00 or Ax DC
#ifndef _HW_DC
    if(sGlbControlBoardParameters.sProductInfo.uwProductRev/100>=6)
#endif // _hw_dc
#endif // _infineon_
        uwVoltage = (UWORD)((FLOAT)uwVoltage/1.836);

    FPGA_ENCODER_OPTIONS_SUPPLY = TRUE ;
    FPGA_VENC_SETUP = uwVoltage ;
    timer_wait(uwSysTimers1ms, sEm_EncMngrParam.uwVEncDelay);      
}

#ifndef _HW_DC
//***************************************************************************
// Adjust Encoder voltage with feedback VENC
#define VENC_MAXSHIFT 10

static void adjustvencoder(void)
{
    UWORD uwVoltage;
    BOOL  bUpdate=FALSE;

    if(sEncMgrRun.uwVEncVoltage > 0)
    {
        uwVoltage = sEncMgrRun.uwVEncVoltage;

        if(sIOMgrAnalogMeasurements.uwSVENCout > sEm_EncMngrParam.uwVEncVoltage + VENC_MAXSHIFT)
        {
            uwVoltage--;
            bUpdate = TRUE;
        }
        else if(sIOMgrAnalogMeasurements.uwSVENCout < sEm_EncMngrParam.uwVEncVoltage - VENC_MAXSHIFT)
        {
            uwVoltage++;
            bUpdate = TRUE;
        }
    }

    if(bUpdate)
    {
        sEncMgrRun.uwVEncVoltage = uwVoltage;

        uwVoltage = (UWORD)((FLOAT)uwVoltage/1.836);

        FPGA_VENC_SETUP = uwVoltage ;
    }
}
#endif

//***************************************************************************
// Turn-off Encoder voltage

static void turnoffvencoder(void)
{
    sEncMgrRun.uwVEncVoltage = 0 ;
    FPGA_ENCODER_OPTIONS_SUPPLY = FALSE ;
    FPGA_VENC_SETUP = 0 ;
}

//***************************************************************************
// Setup Aux Encoder voltage

#ifdef _HW_DC
static void setauxvencoder(void)
{
    FPGA_AUXENC_OPTIONS_SUPPLY = TRUE ;
    FPGA_VAUX_SETUP = (UWORD)((FLOAT)sEm_EncMngrParam.uwAuxVEncVoltage/1.836) ;
    timer_wait(uwSysTimers1ms, sEm_EncMngrParam.uwVEncDelay);      
}

//***************************************************************************
// Turn-off Aux Encoder voltage

static void turnoffauxvencoder(void)
{
    FPGA_AUXENC_OPTIONS_SUPPLY = FALSE ;
    FPGA_VAUX_SETUP = 0 ;
}
#endif

//***************************************************************************
// Electronic plate read info

BOOL eplate(void)
{
    MOTPRM_PARAMETERS sMotPar;
    BOOL bRetVal = TRUE ;
    SBYTE bDataValid=FALSE;

#if (FALSE)// (CRS_DBGDSK)
    return TRUE;
#else
        // if not requested exit
    if(sEm_EncMngrParam.flags.b.bDisableEPlate)
        return TRUE;

        // zeroes return data structure
    memset(&sMotPar, 0, sizeof(sMotPar));

        // check base parameters validity
    if(!paramcheck())
        return FALSE;

        // only if main absolute encoder is Endat
#if CFG_ENC_ENDAT
    if(sEm_EncMngrParam.uwMainAbsSel==ENCODER_TYPE_ABS_ENDAT)
    {
        if(!En_ParametersCheck(ENDAT_SEL_MAIN))
            bRetVal=FALSE;
        else
        {
            setvencoder();
            bRetVal=En_GetEPlate(ENDAT_SEL_MAIN,&sMotPar,&bDataValid);
        }
    }
#endif // cfg_enc_endat

        // if not generic error and valid data reading
    if(bRetVal && bDataValid)
    {
            // if different than last one or restore requested
        if(memcmp(&sMotPar, &sEm_LastValidEPlate, sizeof(sMotPar)) || sEm_EncMngrParam.flags.b.bRestoreEPlate)
        {
                // copy for difference checking
            sEm_LastValidEPlate=sMotPar;

                // and send to global motor parameters
            bRetVal=MotPar_Set(&sMotPar);
        }

        sEm_EncMngrParam.flags.b.bRestoreEPlate=FALSE;
    }

    return bRetVal;
#endif // crs_dbgdsk
}

//***************************************************************************
// Plc encoder initializations

BOOL initplc(void)
{
    sEncMgrRun.sFlags.b.bPlcInput=TRUE;
    return TRUE;
}

//***************************************************************************
// Plc specific options initializations

BOOL initplcoptions(void)
{
        // if requested initialize sincos hardware (PLC option)
#if CFG_ENC_SINCOS
    if(sEncMgrRun.sFlags.b.bEnSinCosHw)
    {
        if(!Sc_ParametersCheck())
            return FALSE;

        if(!Sc_Init(NULL, NULL, 0ul))
            return FALSE;
    }
#endif

        // if requested initialize incremental hardware (PLC option)
#if CFG_ENC_INCR
    if(sEncMgrRun.sFlags.b.bEnIncrementalHw)
    {
        if(!Ic_EncoderParametersCheck(INCREMENTAL_SEL_MAIN))
            return FALSE;

        if(!Ic_EncoderInit(INCREMENTAL_SEL_MAIN, NULL, NULL))
            return FALSE;
    }
#endif

    return TRUE;
}

//***************************************************************************
// Core encoder initializations

BOOL initcore(void)
{
    BOOL bRetVal = FALSE ;
    BOOL bMainAbsValid;
    BOOL bMainRelValid;
    BOOL bAuxNull;
    BOOL bSmartSelection=FALSE;
    ULONG ulAbsDisMask=0ul;
    ENCMGR_SPACEFEEDBACK * psAbsSel;
    ENCMGR_SPACEFEEDBACK * psRelSel;
    ENCMGR_SPACEFEEDBACK * psSimSrcSel;

        // prepare runtime data structure
    sEncMgrRun.sErrorSent.w = 0 ;
    sEncMgrRun.sbExportFault = FALSE;
    sEncMgrRun.uwMotorPoleNumbers = sGlbMotorParameters.uwPoleNumbers ;

        // add bg task mainly for parameters checking
    if(!TaskSched_AddBackgroundTask(&slowtask))
        return FALSE;

        // check parameters validity
    if( (!paramcheck())
#if CFG_ENC_EFS
     || (!EfsParametersCheck())
#endif
     || (!elecanglefftime()) )
        return FALSE;

#if CFG_ENC_EFS
        // init EFS
    EfsInit();
#endif

        // if EFS procedure selected then require hook
    sEm_EncMngrOut.flags.b.bRequireIRefHook=(sEfsParam.ubProcType!=EFS_TYPE_DISABLED);

#if CFG_ENCMGR_OPENLOOP
    OpenLoopEncInit() ;
#endif

        // check for aux encoder
    if(sEm_EncMngrParam.flags.b.bPos2CntrLoop && sEm_EncMngrParam.flags.b.bSpd2CntrLoop && \
        sEm_EncMngrParam.flags.b.bAcc2CntrLoop && sEm_EncMngrParam.flags.b.bElecAngle2Fpga && \
        sEm_EncMngrParam.uwAuxSel==ENCODER_TYPE_NULL)
        bAuxNull=TRUE;
    else
        bAuxNull=FALSE;

        // check for main encoder abs+rel or single
    switch(sEm_EncMngrParam.uwMainAbsSel)
    {
        case ENCODER_TYPE_ABS_ENDAT:
        case ENCODER_TYPE_ABS_ANALOG:
        case ENCODER_TYPE_ABS_HALL:
        case ENCODER_TYPE_ABS_HIPERFACE:
        case ENCODER_TYPE_ABS_NIKON:
        case ENCODER_TYPE_ABS_TMGW:
        case ENCODER_TYPE_ABS_BISS:
            bMainAbsValid=TRUE;
            break;

        default:
            bMainAbsValid=FALSE;
            break;
    }

    switch(sEm_EncMngrParam.uwMainRelSel)
    {
        case ENCODER_TYPE_REL_BACK_EMF:
        case ENCODER_TYPE_REL_INCREMENTAL:
            bMainRelValid=TRUE;
            break;

        default:
            bMainRelValid=FALSE;
            break;
    }

        // then extract pointers for dest fb
    if(bMainAbsValid && (!bMainRelValid) && (!sEm_EncMngrOut.flags.b.bRequireIRefHook) && bAuxNull)
    {
        psAbsSel=&sEm_MainEnc;
        psRelSel=NULL;
        psSimSrcSel=&sEm_MainEnc;
        bSmartSelection=TRUE;
    }
    else if(bMainAbsValid && bMainRelValid)
    {
        psAbsSel=&sEm_MainAbsEnc;
        psRelSel=&sEm_MainRelEnc;
        psSimSrcSel=&sEm_MainRelEnc;
        if(sEm_EncMngrParam.flags.b.bDisAbsAfterValid)
        {
            ulAbsDisMask=SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_EM_DISABLEABSPROCESS);
            sEncMgrRun.sFlags.b.bDisAbsAfterValid=TRUE;
        }
    }
    else if(bMainAbsValid)
    {
        psAbsSel=&sEm_MainEnc;
        psRelSel=NULL;
        psSimSrcSel=&sEm_MainEnc;
    }
    else if(bMainRelValid)
    {
        psAbsSel=NULL;
        psRelSel=&sEm_MainEnc;
        psSimSrcSel=&sEm_MainEnc;
    }
    else
    {
        psAbsSel=NULL;
        psRelSel=NULL;
        psSimSrcSel=NULL;
    }

    bMainAbsValid=bMainRelValid=FALSE;

        // main absolute encoder selection    
    switch(sEm_EncMngrParam.uwMainAbsSel)
    {
        case ENCODER_TYPE_NULL:
            bRetVal=noencoderinit(&sEm_MainAbsEnc);
            sEncMgrRun.pfMainAbsEACorrect=NULL;
            break;

        case ENCODER_TYPE_ABS_ENDAT:
#if CFG_ENC_ENDAT
            if(En_ParametersCheck(ENDAT_SEL_MAIN))
            {
                setvencoder();
                assert(psAbsSel);
                bRetVal=En_Init(ENDAT_SEL_MAIN, psAbsSel, &sEncMgrRun.sbExportFault, ulAbsDisMask);
                sEncMgrRun.pfMainAbsEACorrect=&En_SetElecAngle;
                bMainAbsValid=TRUE;
            }
            else
                bRetVal=FALSE;
#endif
            break;

        case ENCODER_TYPE_ABS_ANALOG:
#if CFG_ENC_SINCOS
            if(!sEncMgrRun.sFlags.b.bEnSinCosHw)
            {
                if(Sc_ParametersCheck())
                {
                    setvencoder();
                    assert(psAbsSel);
                    bRetVal=Sc_Init(psAbsSel, &sEncMgrRun.sbExportFault, ulAbsDisMask);
                    sEncMgrRun.pfMainAbsEACorrect=&Sc_SetElecAngle;
                    bMainAbsValid=TRUE;
                }
                else
                    bRetVal=FALSE;
            }
#endif
            break;
            
        case ENCODER_TYPE_ABS_HALL:
#if CFG_ENC_HALL
            if(!sEncMgrRun.sFlags.b.bEnSinCosHw)
            {
                if(Hl_ParametersCheck())
                {
                    setvencoder();
                    assert(psAbsSel);
                    bRetVal=Hl_Init(psAbsSel, &sEncMgrRun.sbExportFault, ulAbsDisMask);
                    sEncMgrRun.pfMainAbsEACorrect=&Hl_SetElecAngle;
                    bMainAbsValid=TRUE;
                }
                else
                    bRetVal=FALSE;
            }
#endif
            break;

        case ENCODER_TYPE_ABS_HIPERFACE:
#if CFG_ENC_HIP
            if(Hf_ParametersCheck())
            {
                setvencoder();
                assert(psAbsSel);
                bRetVal=Hf_Init(psAbsSel, &sEncMgrRun.sbExportFault, ulAbsDisMask);
                sEncMgrRun.pfMainAbsEACorrect=&Hf_SetElecAngle;
                bMainAbsValid=TRUE;
            }
            else
                bRetVal=FALSE;
#endif
            break;

        case ENCODER_TYPE_ABS_NIKON:
#if CFG_ENC_NIKON
            if(Nk_ParametersCheck(NIKON_SEL_MAIN))
            {
                setvencoder();
                assert(psAbsSel);
                bRetVal=Nk_Init(NIKON_SEL_MAIN,psAbsSel, &sEncMgrRun.sbExportFault, ulAbsDisMask);
                sEncMgrRun.pfMainAbsEACorrect=&Nk_SetElecAngle;
                bMainAbsValid=TRUE;
            }
            else
                bRetVal=FALSE;
#endif
            break;

        case ENCODER_TYPE_ABS_TMGW:
#if CFG_ENC_TMGW
            if(Tm_ParametersCheck(TMGW_SEL_MAIN))
            {
                setvencoder();
                assert(psAbsSel);
                bRetVal=Tm_Init(TMGW_SEL_MAIN,psAbsSel, &sEncMgrRun.sbExportFault, ulAbsDisMask);
                sEncMgrRun.pfMainAbsEACorrect=&Tm_SetElecAngle;
                bMainAbsValid=TRUE;
            }
            else
                bRetVal=FALSE;
#endif
            break;

        case ENCODER_TYPE_ABS_BISS:
#if CFG_ENC_BISS
            if(Biss_ParametersCheck(BISS_SEL_MAIN))
            {
                setvencoder();
                assert(psAbsSel);
                bRetVal=Biss_Init(BISS_SEL_MAIN, psAbsSel, &sEncMgrRun.sbExportFault, ulAbsDisMask);
                sEncMgrRun.pfMainAbsEACorrect=&Biss_SetElecAngle;
                bMainAbsValid=TRUE;
            }
            else
                bRetVal=FALSE;
#endif
            break;

        default:
            bRetVal=FALSE;
    }

        // if error
    if(!bRetVal)
    {
        sEncMgrRun.pfMainAbsEACorrect=NULL;
            // if fault management not disabled
        if(!sEncMgrRun.sFlags.b.bDisFaultMgm)
        {
                // shutdown power and return
            turnoffvencoder();
            return FALSE;
        }
        else
            psAbsSel->ubStatus=ENCMGR_FATAL_FAULT;
    }
    
        // main relative encoder selection    
    switch(sEm_EncMngrParam.uwMainRelSel)
    {
        case ENCODER_TYPE_NULL:
            bRetVal=noencoderinit(&sEm_MainRelEnc);
            sEncMgrRun.pfMainRelEACorrect=NULL;
            sEncMgrRun.pfMainRelAPCorrect=NULL;
            sEncMgrRun.pfMainRelGetSnapshot=NULL;
            break;

        case ENCODER_TYPE_REL_BACK_EMF:
#if CFG_ENC_BEMF
            if(Be_ParametersCheck())
            {
                setvencoder();	// allow encoder voltage
                assert(psRelSel);
                bRetVal=Be_BackEmfHandlerEncInit(psRelSel, &sEncMgrRun.sbExportFault);
                sEncMgrRun.pfMainRelEACorrect=&Be_SetElecAngle;
                sEncMgrRun.pfMainRelAPCorrect=&Be_SetAbsPos;
                sEncMgrRun.pfMainRelGetSnapshot=NULL;
                sEncMgrRun.sFlags.b.bBEMFHook=TRUE;
                sEm_EncMngrOut.flags.b.bRequireIRefHook=TRUE;
                bMainRelValid=TRUE;
            }
            else
                bRetVal=FALSE;
#endif
            break;
            
        case ENCODER_TYPE_REL_INCREMENTAL:
#if CFG_ENC_INCR
            if(!sEncMgrRun.sFlags.b.bEnIncrementalHw)
            {
                if(Ic_EncoderParametersCheck(INCREMENTAL_SEL_MAIN))
                {
                    setvencoder();
                    assert(psRelSel);
                    bRetVal=Ic_EncoderInit(INCREMENTAL_SEL_MAIN, psRelSel, &sEncMgrRun.sbExportFault);
                    sEncMgrRun.pfMainRelEACorrect=&Ic_EncoderSetElecAngle;
                    sEncMgrRun.pfMainRelAPCorrect=&Ic_EncoderSetAbsPos;
                    sEncMgrRun.pfMainRelGetSnapshot=&Ic_EncoderGetSnapshot;
                    bMainRelValid=TRUE;
                }
                else
                    bRetVal=FALSE;
            }
#endif
            break;
            
        default:
            bRetVal=FALSE;
    }

        // if error
    if(!bRetVal)
    {
        sEncMgrRun.pfMainAbsEACorrect=NULL;
        sEncMgrRun.pfMainRelEACorrect=NULL;
        sEncMgrRun.pfMainRelAPCorrect=NULL;
        sEncMgrRun.pfMainRelGetSnapshot=NULL;
            // if fault management not disabled
        if(!sEncMgrRun.sFlags.b.bDisFaultMgm)
        {
                // shutdown power and return
            turnoffvencoder();
            return FALSE;
        }
        else
            psRelSel->ubStatus=ENCMGR_FATAL_FAULT;
    }

        // if both abs and rel selected then combine feedbacks
    sEncMgrRun.sFlags.b.bMainCombine = bMainRelValid && bMainAbsValid;

        // auxiliary encoder selection
    switch(sEm_EncMngrParam.uwAuxSel)
    {
        case ENCODER_TYPE_NULL:
            bRetVal=noencoderinit(&sEm_AuxEnc);
            break;

        case ENCODER_TYPE_REL_INCREMENTAL:
#if CFG_ENC_INCR
            if(Ic_EncoderParametersCheck(INCREMENTAL_SEL_AUX))
            {
#ifndef _HW_DC
                setvencoder();
#else
                setauxvencoder();
#endif
                bRetVal=Ic_EncoderInit(INCREMENTAL_SEL_AUX, &sEm_AuxEnc, &sEncMgrRun.sbExportFault);
            }
            else
                bRetVal=FALSE;
#endif
            break;

        case ENCODER_TYPE_ABS_BISS:
#if CFG_ENC_BISS
            if(Biss_ParametersCheck(BISS_SEL_AUX))
            {
#ifndef _HW_DC
                setvencoder();
#else
                setauxvencoder();
#endif
                bRetVal=Biss_Init(BISS_SEL_AUX, &sEm_AuxEnc, &sEncMgrRun.sbExportFault, ulAbsDisMask);
            }
            else
                bRetVal=FALSE;
#endif
            break;
            
        case ENCODER_TYPE_ABS_ENDAT:
#if CFG_ENC_ENDAT
#ifndef _HW_DC
            if(En_ParametersCheck(ENDAT_SEL_AUX))
            {
#ifndef _HW_DC
                setvencoder();
#else
                setauxvencoder();
#endif // !_hw_dc
                bRetVal=En_Init(ENDAT_SEL_AUX, &sEm_AuxEnc, &sEncMgrRun.sbExportFault, ulAbsDisMask);
            }
            else
                bRetVal=FALSE;
#endif // !_hw_dc
#endif // cfg_enc_endat
            break;
        case ENCODER_TYPE_REL_INCRSIMUL:
        case ENCODER_TYPE_ABS_B1_ENDAT:
            bRetVal=FALSE;
            break;
            
        default:
            bRetVal=FALSE;
    }

        // auxiliary encoder selection
    if(bRetVal)
        switch(sEm_EncMngrParam.uwSimSel)
        {
            case ENCODER_TYPE_NULL:
                bRetVal=TRUE;
                break;
    
            case ENCODER_TYPE_REL_INCRSIMUL:
#if CFG_ENC_INCR
#ifndef _HW_DC
                    // for controlboards up to 5.00 aux in and sim out together
                    // cannot be enabled
                if(sGlbControlBoardParameters.sProductInfo.uwProductRev < 600)
                    if(sEm_EncMngrParam.uwAuxSel==ENCODER_TYPE_REL_INCREMENTAL)
                    {
                        bRetVal=FALSE;
                        break;
                    }
#endif
                if(Ic_SimulationParametersCheck(sEm_EncMngrParam.uwMainRelSel==ENCODER_TYPE_REL_INCREMENTAL))
                {
#ifndef _HW_DC
                    setvencoder();
#else
                    setauxvencoder();
#endif
//                    if(sEncMgrRun.sFlags.b.bPlcInput)
//                        bRetVal=Ic_SimulationInit(&sEm_PlcSimulationEnc, &sEncMgrRun.ubSimStatus);
//                    else
                    {
                        if(psSimSrcSel)
                            bRetVal=Ic_SimulationInit(psSimSrcSel, &sEncMgrRun.ubSimStatus);
                        else
                            bRetVal=FALSE;
                    }
                }
                else
                    bRetVal=FALSE;
#endif
                break;

            default:
                bRetVal=FALSE;
        }

        // if error
    if(!bRetVal)
    {
        sEncMgrRun.pfMainAbsEACorrect=NULL;
        sEncMgrRun.pfMainRelEACorrect=NULL;
        sEncMgrRun.pfMainRelAPCorrect=NULL;
        sEncMgrRun.pfMainRelGetSnapshot=NULL;
            // if fault management not disabled
        if(!sEncMgrRun.sFlags.b.bDisFaultMgm)
        {
                // shutdown power and return
#ifndef _HW_DC
            turnoffvencoder();
#else
            turnoffauxvencoder();
#endif
            return FALSE;
        }
        else
            sEm_AuxEnc.ubStatus=ENCMGR_FATAL_FAULT;
    }

#if CFG_ENCMGR_OPENLOOP
    OpenLoopEncResetOut() ;

    if (sEm_EncMngrOut.flags.b.bOpenLoop)
    {	// get data from open loop "encoder"
		sEncMgrRun.psqPosition2Use         = &sEncMgrRun.sOpenLoop.sEncoder.sEncData.sqPostn ;
		sEncMgrRun.pubStatus2Use           = &sEncMgrRun.sOpenLoop.sEncoder.ubStatus ;
		sEncMgrRun.psqMechAbsOffset2Use    = &sEncMgrRun.sOpenLoop.sEncoder.sqMechAbsPosOffset ;
		sEncMgrRun.pslSpeed2Use            = &sEncMgrRun.sOpenLoop.sEncoder.sEncData.slSpeed ;
		sEncMgrRun.pslAccel2Use            = &sEncMgrRun.sOpenLoop.sEncoder.sEncData.slAccel ;
		sEncMgrRun.puwElecAngle2Use        = &sEncMgrRun.sOpenLoop.sEncoder.uwElecAngle ;
		sEncMgrRun.pswElecSpeed2Use        = &sEncMgrRun.sOpenLoop.sEncoder.swElecSpeed ;
		sEncMgrRun.sFlags.b.bAbsFbFromAux  = FALSE ;
		sEncMgrRun.sFlags.b.bCritFbFromAux = FALSE ;
    }
    else
#endif // cfg_encmgr_openloop
    {		// select which position-information will be used by the control loop
		if (sEm_EncMngrParam.flags.b.bPos2CntrLoop)
		{
			sEncMgrRun.psqPosition2Use      = &sEm_MainEnc.sEncData.sqPostn ;
			sEncMgrRun.pubStatus2Use        = &sEm_MainEnc.ubStatus ;
			sEncMgrRun.psqMechAbsOffset2Use = &sEm_MainEnc.sqMechAbsPosOffset ;
		}
		else
		{
			sEncMgrRun.psqPosition2Use      = &sEm_AuxEnc.sEncData.sqPostn ;
			sEncMgrRun.pubStatus2Use        = &sEm_AuxEnc.ubStatus ;
			sEncMgrRun.psqMechAbsOffset2Use = &sEm_AuxEnc.sqMechAbsPosOffset ;
			sEncMgrRun.sFlags.b.bAbsFbFromAux = TRUE;
		}

			// select which speed-information will be used by the control loop
		if (sEm_EncMngrParam.flags.b.bSpd2CntrLoop)
			sEncMgrRun.pslSpeed2Use = &sEm_MainEnc.sEncData.slSpeed ;
		else
			sEncMgrRun.pslSpeed2Use = &sEm_AuxEnc.sEncData.slSpeed ;

			// select which acceleration-information will be used by the control loop
		if(sEm_EncMngrParam.flags.b.bAcc2CntrLoop)
			sEncMgrRun.pslAccel2Use = &sEm_MainEnc.sEncData.slAccel ;
		else
			sEncMgrRun.pslAccel2Use = &sEm_AuxEnc.sEncData.slAccel ;

			// select which electrical angle-information will be used by the FPGA
		if(sEm_EncMngrParam.flags.b.bElecAngle2Fpga)
		{
			sEncMgrRun.puwElecAngle2Use = &sEm_MainEnc.uwElecAngle ;
			sEncMgrRun.pswElecSpeed2Use = &sEm_MainEnc.swElecSpeed ;
		}
		else
		{
			sEncMgrRun.puwElecAngle2Use = &sEm_AuxEnc.uwElecAngle ;
			sEncMgrRun.pswElecSpeed2Use = &sEm_AuxEnc.swElecSpeed ;
			sEncMgrRun.sFlags.b.bCritFbFromAux = TRUE;
		}
    }
    
        // runtime flags init
    rtflagsapply();
    sEncMgrRun.prevflags = sEm_EncMngrParam.flags;

        // main encoder physically present: allow EFS procedure execution (if necessary)
    if (bMainAbsValid || bMainRelValid)
        sEncMgrRun.sFlags.b.bMainEncPresent = TRUE ;
    else
        sEncMgrRun.sFlags.b.bMainEncPresent = FALSE ; 

        // add rt task
    if(bSmartSelection)
        return TaskSched_AddRTTask(&task8kHzminimal, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0) ;
    else
        return TaskSched_AddRTTask(&task8kHzfull, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0) ;
}

//***************************************************************************
// runtime flags apply

static void rtflagsapply()
{
#if CFG_ENCMGR_OPENLOOP
    if (sEm_EncMngrOut.flags.b.bOpenLoop)
    {
    	sEncMgrRun.sFlags.b.bSpeedFilter = FALSE ;
    	sEncMgrRun.psqMainPos            = &sEncMgrRun.sOpenLoop.sEncoder.sEncData.sqPostn ;
    	sEncMgrRun.pslMainSpd            = &sEncMgrRun.sOpenLoop.sEncoder.sEncData.slSpeed ;
    	sEncMgrRun.pslMainAcc            = &sEncMgrRun.sOpenLoop.sEncoder.sEncData.slAccel ;
		sEncMgrRun.puwMainElecAngle      = &sEncMgrRun.sOpenLoop.sEncoder.uwElecAngle ;
		sEncMgrRun.puwMainDeltaElecAngle = &sEncMgrRun.sOpenLoop.sEncoder.uwDeltaElecAngle ;
		sEncMgrRun.pswMainElecSpeed      = &sEncMgrRun.sOpenLoop.sEncoder.swElecSpeed ;
    }
    else
#endif
    {	// speed filtering selection
		sEncMgrRun.sFlags.b.bSpeedFilter = sEm_EncMngrParam.flags.b.bSpeedFilter ;

			// abs/rel selection
		if(sEm_EncMngrParam.flags.b.bMainPosSel)
			sEncMgrRun.psqMainPos = &sEm_MainAbsEnc.sEncData.sqPostn;
		else
			sEncMgrRun.psqMainPos = &sEm_MainRelEnc.sEncData.sqPostn;

		if(sEm_EncMngrParam.flags.b.bMainSpdSel)
			sEncMgrRun.pslMainSpd = &sEm_MainAbsEnc.sEncData.slSpeed;
		else
			sEncMgrRun.pslMainSpd = &sEm_MainRelEnc.sEncData.slSpeed;

		if(sEm_EncMngrParam.flags.b.bMainAccSel)
			sEncMgrRun.pslMainAcc = &sEm_MainAbsEnc.sEncData.slAccel;
		else
			sEncMgrRun.pslMainAcc = &sEm_MainRelEnc.sEncData.slAccel;

		if(sEm_EncMngrParam.flags.b.bMainElecAngleSel)
		{
			sEncMgrRun.puwMainElecAngle     = &sEm_MainAbsEnc.uwElecAngle;
			sEncMgrRun.puwMainDeltaElecAngle= &sEm_MainAbsEnc.uwDeltaElecAngle;
			sEncMgrRun.pswMainElecSpeed     = &sEm_MainAbsEnc.swElecSpeed;
		}
		else
		{
			sEncMgrRun.puwMainElecAngle     = &sEm_MainRelEnc.uwElecAngle;
			sEncMgrRun.puwMainDeltaElecAngle= &sEm_MainRelEnc.uwDeltaElecAngle;
			sEncMgrRun.pswMainElecSpeed     = &sEm_MainRelEnc.swElecSpeed;
		}
    }
}

//***************************************************************************
// Hardware option callback

void Em_GetHwOpt(UWORD t, ULONG * pulHwOpt, ULONG * pulFPGAOpt)
{
    switch(sEm_EncMngrParam.uwMainAbsSel)
    {
        case ENCODER_TYPE_ABS_ENDAT:
            *pulHwOpt  |=HWUNITID_ENCM_INC_DIG;
            *pulFPGAOpt|=FPGA_HW_ENCMAINENDAT;
            break;

        case ENCODER_TYPE_ABS_ANALOG:
            *pulHwOpt  |=HWUNITID_ENCM_ABS;
            *pulFPGAOpt|=FPGA_HW_ENCMAINSINCOSABS;
            break;

        case ENCODER_TYPE_ABS_HALL:
            *pulHwOpt  |=HWUNITID_ENCM_ABS;
            *pulFPGAOpt|=FPGA_HW_ENCMAINSINCOSABS;
            break;

        case ENCODER_TYPE_ABS_HIPERFACE:
            *pulHwOpt  |=HWUNITID_ENCM_INC_DIG;
            *pulFPGAOpt|=FPGA_HW_ENCHIPERFACE;
            break;

        case ENCODER_TYPE_ABS_NIKON:
            *pulHwOpt  |=HWUNITID_ENCM_INC_DIG;
            *pulFPGAOpt|=FPGA_HW_ENCNIKON;
            break;
            
        case ENCODER_TYPE_ABS_TMGW:
            *pulHwOpt  |=HWUNITID_ENCM_INC_DIG;
            *pulFPGAOpt|=FPGA_HW_ENCTMGW;
            break;
            
        case ENCODER_TYPE_ABS_BISS:
            *pulHwOpt  |=HWUNITID_ENCM_INC_DIG;
            *pulFPGAOpt|=FPGA_HW_ENCMAINBISS;
            break;
    }

    switch(sEm_EncMngrParam.uwMainRelSel)
    {
        case ENCODER_TYPE_REL_INCREMENTAL:
#if CFG_ENC_INCR
            Ic_GetHwOpt(INCREMENTAL_SEL_MAIN, pulHwOpt, pulFPGAOpt);
#endif
            break;
    }

    switch(sEm_EncMngrParam.uwAuxSel)
    {
        case ENCODER_TYPE_REL_INCREMENTAL:
#if CFG_ENC_INCR
            Ic_GetHwOpt(INCREMENTAL_SEL_AUX, pulHwOpt, pulFPGAOpt);
#endif
            break;
        case ENCODER_TYPE_ABS_ENDAT:
#if CFG_ENC_ENDAT
            *pulHwOpt  |=HWUNITID_ENCA_INC_BOUT;
            *pulFPGAOpt|=FPGA_HW_ENCAUXENDAT;
#endif
            break;
        case ENCODER_TYPE_ABS_BISS:
#if CFG_ENC_BISS
            *pulHwOpt  |=HWUNITID_ENCA_INC_BOUT;
            *pulFPGAOpt|=FPGA_HW_ENCAUXBISS;
#endif
            break;
    }

    switch(sEm_EncMngrParam.uwSimSel)
    {
        case ENCODER_TYPE_REL_INCRSIMUL:
#if CFG_ENC_INCR
            Ic_SimulationGetHwOpt(pulHwOpt, pulFPGAOpt);
#endif
            break;
    }

    if(sEm_EncMngrParam.uwVEncVoltage>0)
    {
        *pulHwOpt  |=HWUNITID_ENCM_VPWR;
        *pulFPGAOpt|=FPGA_HW_VMAINSPL;
    }

#ifdef _HW_DC
    if(sEm_EncMngrParam.uwAuxVEncVoltage>0)
    {
        *pulHwOpt  |=HWUNITID_ENCA_VPWR;
        *pulFPGAOpt|=FPGA_HW_VAUXSPL;
    }
#endif
}

//***************************************************************************
// Currents hook encoder initializations

BOOL inithook(void)
{
        // check parameters validity
    if( (!paramcheck()) 
#if CFG_ENC_EFS
     || (!EfsParametersCheck()) 
#endif
     )
        return FALSE;

        // if requested/required add rt task
    if(sEm_EncMngrOut.flags.b.bRequireIRefHook)
        return TaskSched_AddRTTask(&hook8kHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0) ;
    else
        return TRUE;
}

//***************************************************************************
// minimal fast task

static BOOL task8kHzminimal(void)
{
        // reset errors if requested
    if(bSysStatAlarmsReset)
    {
        sEncMgrRun.sErrorSent.w = 0 ;
        sEncMgrRun.sbExportFault = FALSE;
    }

        // feedforward calculation
    elecangleff_calc();

        // if snapshot then reset all flags, position/electrical 
        // angle not valid for control loops
    if(sEm_MainAbsEnc.ubStatus&ENCMGR_SNAPSHOT_VALID)
        sEm_MainAbsEnc.ubStatus&=(ENCMGR_FATAL_FAULT|ENCMGR_NONFATAL_FAULT|ENCMGR_SNAPSHOT_VALID);

        // copy back on fb encoder
    sEm_Fbk2CntrLoop.sEncData.sqPostn.lo=sEm_MainEnc.sEncData.sqPostn.lo;
    sEm_Fbk2CntrLoop.sEncData.sqPostn.hi=sEm_MainEnc.sEncData.sqPostn.hi;
    sEm_Fbk2CntrLoop.sEncData.slSpeed=sEm_MainEnc.sEncData.slSpeed;
    sEm_Fbk2CntrLoop.sEncData.slAccel=sEm_MainEnc.sEncData.slAccel;
    sEm_Fbk2CntrLoop.uwElecAngle=sEm_MainEnc.uwElecAngle+sEncMgrRun.swElecAngleFF;
    sEm_Fbk2CntrLoop.swElecSpeed=sEm_MainEnc.swElecSpeed;
    sEm_Fbk2CntrLoop.sqMechAbsPosOffset.lo=sEm_MainEnc.sqMechAbsPosOffset.lo;
    sEm_Fbk2CntrLoop.sqMechAbsPosOffset.hi=sEm_MainEnc.sqMechAbsPosOffset.hi;
    sEm_Fbk2CntrLoop.ubStatus=sEm_MainEnc.ubStatus;

        // check for fatal and non-fatal faults and take corrective actions
    if(!sEncMgrRun.sErrorSent.b.bEncFatalFault && (sEm_Fbk2CntrLoop.ubStatus & (ENCMGR_FATAL_FAULT|ENCMGR_NONFATAL_FAULT)))
    {
        if(!sEncMgrRun.sFlags.b.bDisFaultMgm)
            (*sEm_EncMngrIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
			
        sEncMgrRun.sErrorSent.b.bEncFatalFault = TRUE ;
        sEncMgrRun.sbExportFault = TRUE;
    }

        // filtered speed calculation
    filterspeed_calc();

    return TRUE ; 
}

//***************************************************************************
// full fast task

static BOOL task8kHzfull(void)
{
    UBYTE ubStatus;

        // reset errors if requested
    if(bSysStatAlarmsReset)
    {
        sEncMgrRun.sErrorSent.w = 0 ;
        sEncMgrRun.sbExportFault = FALSE;
    }

        // if changed then resetup runtime flags
    if(memcmp(&sEncMgrRun.prevflags, &sEm_EncMngrParam.flags, sizeof(ENCMGR_PARFLAGS)))
    {
        rtflagsapply();
        memcpy(&sEncMgrRun.prevflags, &sEm_EncMngrParam.flags, sizeof(ENCMGR_PARFLAGS));
    }

        // combine main encoder feedbacks if selected both abs and rel
    if(sEncMgrRun.sFlags.b.bMainCombine)
    {
            // if valid, take values from relative encoder
        if(sEm_MainRelEnc.ubStatus & ENCMGR_ELE_ANGLE_VALID)
        {
            sEm_MainEnc.uwElecAngle      = *sEncMgrRun.puwMainElecAngle;
            sEm_MainEnc.uwDeltaElecAngle = *sEncMgrRun.puwMainDeltaElecAngle;
            sEm_MainEnc.swElecSpeed      = *sEncMgrRun.pswMainElecSpeed;
        }
        else
        {
            sEm_MainEnc.uwElecAngle=sEm_MainAbsEnc.uwElecAngle;
            sEm_MainEnc.uwDeltaElecAngle=sEm_MainAbsEnc.uwDeltaElecAngle;
            sEm_MainEnc.swElecSpeed=sEm_MainAbsEnc.swElecSpeed;
                // if abs ele angle valid and rel valid, setup electrical angle info to rel
            if((sEm_MainAbsEnc.ubStatus & ENCMGR_ELE_ANGLE_VALID) && \
                (sEm_MainRelEnc.ubStatus & ENCMGR_RELATIVE_VALID) && \
                 sEncMgrRun.pfMainRelEACorrect)
                (*sEncMgrRun.pfMainRelEACorrect)(sEm_MainAbsEnc.uwElecAngle);
        }

        if(sEm_MainRelEnc.ubStatus & ENCMGR_RELATIVE_VALID)
        {
            sEm_MainEnc.sEncData.sqPostn=*sEncMgrRun.psqMainPos;
            sEm_MainEnc.sEncData.slSpeed=*sEncMgrRun.pslMainSpd;
            sEm_MainEnc.sEncData.slAccel=*sEncMgrRun.pslMainAcc;
        }
        else
            sEm_MainEnc.sEncData=sEm_MainAbsEnc.sEncData;

        if(sEm_MainRelEnc.ubStatus & ENCMGR_ABSMECHTURN_VALID)
            sEm_MainEnc.sqMechAbsPosOffset=sEm_MainRelEnc.sqMechAbsPosOffset;
        else
        {
            sEm_MainEnc.sqMechAbsPosOffset=sEm_MainAbsEnc.sqMechAbsPosOffset;

                // if abs pos offset valid and rel valid, setup abs pos offset info to rel
            if((sEm_MainAbsEnc.ubStatus & ENCMGR_ABSMECHTURN_VALID) && \
                (sEm_MainRelEnc.ubStatus & ENCMGR_RELATIVE_VALID) && \
                 sEncMgrRun.pfMainRelAPCorrect)
                (*sEncMgrRun.pfMainRelAPCorrect)(sEm_MainAbsEnc.sqMechAbsPosOffset);
        }

            // combine status from both encoders, if main has snapshot flag then data
            // is valid just for diagnostic/init purpose, but not for control loops
        ubStatus=sEm_MainRelEnc.ubStatus;
        if(sEm_MainAbsEnc.ubStatus&ENCMGR_SNAPSHOT_VALID)
            ubStatus|=sEm_MainAbsEnc.ubStatus&(ENCMGR_FATAL_FAULT|ENCMGR_NONFATAL_FAULT);
        else
            ubStatus|=sEm_MainAbsEnc.ubStatus;

            // if one is faulty but ENCMGR_ELE_ANGLE_VALID and ENCMGR_RELATIVE_VALID
            // then result in non-fatal fault as there's at least one valid feedback
        if(ubStatus&ENCMGR_FATAL_FAULT)
        {
            if((ubStatus&(ENCMGR_ELE_ANGLE_VALID|ENCMGR_RELATIVE_VALID))==(ENCMGR_ELE_ANGLE_VALID|ENCMGR_RELATIVE_VALID))
            {
                ubStatus&=~ENCMGR_FATAL_FAULT;
                ubStatus|=ENCMGR_NONFATAL_FAULT;
            }
        }
        else
                // if relative is valid for elec, rel and abs then disable (if apply)
                // the absolute track processing
            if((sEm_MainRelEnc.ubStatus & (ENCMGR_ELE_ANGLE_VALID|ENCMGR_RELATIVE_VALID|ENCMGR_ABSMECHTURN_VALID)) ==
                                          (ENCMGR_ELE_ANGLE_VALID|ENCMGR_RELATIVE_VALID|ENCMGR_ABSMECHTURN_VALID))
                bSysStatEmDisableAbsProcess=sEncMgrRun.sFlags.b.bDisAbsAfterValid;

            // resulting encoder status
        sEm_MainEnc.ubStatus=ubStatus;

            // monitor difference between abs and rel (if abs is enabled)
        if((sEm_MainAbsEnc.ubStatus & ENCMGR_RELATIVE_VALID) && (sEm_MainRelEnc.ubStatus & ENCMGR_RELATIVE_VALID) && \
            sEm_EncMngrParam.uwMaxAbsRelDiff && !sEncMgrRun.sErrorSent.b.bMaxDiffFault && !bSysStatEmDisableAbsProcess)
        {
            if(sEm_MainAbsEnc.ubStatus&ENCMGR_SNAPSHOT_VALID)
            {
                    // here if main has snapshot, it is already used, then reset all flags but fault status
                    // and snapshot flag itself, for which if background checking has not already terminated
                    // keep it enabled to prevent abs encoder to prepare a new snapshot
                sEm_MainAbsEnc.ubStatus&=(ENCMGR_FATAL_FAULT|ENCMGR_NONFATAL_FAULT|ENCMGR_SNAPSHOT_VALID);
                sEncMgrRun.sFlags.b.bExecAbsRelDiff=TRUE;
            }
            else
            {
                maxabsreldiff_verify(sEm_MainAbsEnc.sEncData.sqPostn.lo, sEm_MainRelEnc.sEncData.sqPostn.lo);
            }
        }

            // if faulty, force invalid and fault encoder
        if(sEncMgrRun.sErrorSent.b.bMaxDiffFault)
            sEm_MainEnc.ubStatus=ENCMGR_FATAL_FAULT;

            // when background snapshot is consumed then reset flag in order to trigger a new snapshot
        if((sEm_MainAbsEnc.ubStatus&ENCMGR_SNAPSHOT_VALID) && !sEncMgrRun.sFlags.b.bExecAbsRelDiff)
            sEm_MainAbsEnc.ubStatus&=~ENCMGR_SNAPSHOT_VALID;
    }

        // if faulty EFS procedure, force invalid and fault encoder
    if(sEncMgrRun.sErrorSent.b.bEfsFault)
        sEm_MainEnc.ubStatus=ENCMGR_FATAL_FAULT;

        // check faults, rel and elec angle between main and aux
    if(sEncMgrRun.sFlags.b.bCritFbFromAux)
        ubStatus = sEm_AuxEnc.ubStatus  | (sEm_MainEnc.ubStatus & (ENCMGR_FATAL_FAULT|ENCMGR_NONFATAL_FAULT));
    else
        ubStatus = sEm_MainEnc.ubStatus | (sEm_AuxEnc.ubStatus  & (ENCMGR_FATAL_FAULT|ENCMGR_NONFATAL_FAULT));

        // add sim encoder faults
    ubStatus|=sEncMgrRun.ubSimStatus&(ENCMGR_FATAL_FAULT|ENCMGR_NONFATAL_FAULT);

        // check abs between main and aux
    if(sEncMgrRun.sFlags.b.bAbsFbFromAux)
        ubStatus = (ubStatus & ~ENCMGR_ABSMECHTURN_VALID) | (sEm_AuxEnc.ubStatus  & ENCMGR_ABSMECHTURN_VALID);
    else
        ubStatus = (ubStatus & ~ENCMGR_ABSMECHTURN_VALID) | (sEm_MainEnc.ubStatus & ENCMGR_ABSMECHTURN_VALID);

        // if one is faulty but ENCMGR_ELE_ANGLE_VALID and ENCMGR_RELATIVE_VALID
        // then result in non-fatal fault as motor could be kept controlled
    if(ubStatus&ENCMGR_FATAL_FAULT)
        if((ubStatus&(ENCMGR_ELE_ANGLE_VALID|ENCMGR_RELATIVE_VALID))==(ENCMGR_ELE_ANGLE_VALID|ENCMGR_RELATIVE_VALID))
        {
            ubStatus&=~ENCMGR_FATAL_FAULT;
            ubStatus|=ENCMGR_NONFATAL_FAULT;
        }

        // data to control loop
    sEm_Fbk2CntrLoop.sEncData.sqPostn   = *sEncMgrRun.psqPosition2Use ;
    sEm_Fbk2CntrLoop.sEncData.slAccel   = *sEncMgrRun.pslAccel2Use ;
    sEm_Fbk2CntrLoop.sqMechAbsPosOffset = *sEncMgrRun.psqMechAbsOffset2Use ;

        // if requested then filter speed
    if(sEncMgrRun.sFlags.b.bSpeedFilter)
    {
        if(*sEncMgrRun.pslSpeed2Use > 0)
            sEm_Fbk2CntrLoop.sEncData.slSpeed = ( sEm_Fbk2CntrLoop.sEncData.slSpeed >> 4) *  15 + (*sEncMgrRun.pslSpeed2Use >> 4) ;
        else
            sEm_Fbk2CntrLoop.sEncData.slSpeed = (-sEm_Fbk2CntrLoop.sEncData.slSpeed >> 4) * -15 + (*sEncMgrRun.pslSpeed2Use >> 4) ;
    }
    else
        sEm_Fbk2CntrLoop.sEncData.slSpeed   = *sEncMgrRun.pslSpeed2Use ;

        // if EFS in progress keep ELE_ANGLE_VALID disabled, otherwise setup electric angle
    if(sEncMgrRun.sFlags.b.bEFSInProgress)
    {
        ubStatus&=~ENCMGR_ELE_ANGLE_VALID;
        sEm_Fbk2CntrLoop.swElecSpeed = 0 ;
    }
    else
    {
#if CFG_ENCMGR_OPENLOOP
        sEm_Fbk2CntrLoop.uwElecAngle = *sEncMgrRun.puwElecAngle2Use ;
        sEm_Fbk2CntrLoop.swElecSpeed = *sEncMgrRun.pswElecSpeed2Use ;
#else
        elecangleff_calc();
        sEm_Fbk2CntrLoop.uwElecAngle=*sEncMgrRun.puwElecAngle2Use+sEncMgrRun.swElecAngleFF;
        sEm_Fbk2CntrLoop.swElecSpeed=*sEncMgrRun.pswElecSpeed2Use;
#endif
    }

        // setup resulting encoder status
    sEm_Fbk2CntrLoop.ubStatus = ubStatus;
    
        // check for fatal and non-fatal faults and take corrective actions
    if( (!sEncMgrRun.sErrorSent.b.bEncFatalFault) && \
        ((sEm_Fbk2CntrLoop.ubStatus & ENCMGR_FATAL_FAULT) || \
         (sEm_Fbk2CntrLoop.ubStatus & ENCMGR_NONFATAL_FAULT) && sEm_EncMngrParam.flags.b.bDisableIfRelFail) )
    {
        if(!sEncMgrRun.sFlags.b.bDisFaultMgm)
            (*sEm_EncMngrIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;

        sEncMgrRun.sErrorSent.b.bEncFatalFault = TRUE ;
        sEncMgrRun.sbExportFault = TRUE;
        bSysStatEmDisableAbsProcess = FALSE;
    }
    
    if((sEm_Fbk2CntrLoop.ubStatus & ENCMGR_NONFATAL_FAULT) && (!sEncMgrRun.sErrorSent.b.bEncNonFatalFault) && (!sEncMgrRun.sErrorSent.b.bEncFatalFault))
    {
        if(!sEncMgrRun.sFlags.b.bDisFaultMgm)
            (*sEm_EncMngrIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL) ;

        sEncMgrRun.sErrorSent.b.bEncNonFatalFault = TRUE ;
        sEncMgrRun.sbExportFault = TRUE;
    }

        // filtered speed calculation
    filterspeed_calc();

    return TRUE ; 
}

//***************************************************************************
// current hook fast task

static BOOL hook8kHz(void)
{
    UWORD uwElecAngle;
    UWORD uwRetVal;

#if CFG_ENC_EFS
        // if power switched on and EFS selected with a real-physical main encoder (ignore if only aux encoder exists)
    if( sEm_EncMngrIn.psMotorData->sPowerStageSts.b.bFullyActive && (!sEncMgrRun.sFlags.b.bPrevPSState) &&
       ((sEfsCheckedParam.ubProcType!=EFS_TYPE_DISABLED) && sEncMgrRun.sFlags.b.bMainEncPresent) )
    {
            // if fieldbus syncing then exit immediately and skip setup that could take long time
        if(bSysStatFieldbusSyncing)
            return TRUE;

            // run EFS setup to check if EFS is needed
#if CFG_ENCMGR_OPENLOOP
        {
			ENCMGR_SPACEFEEDBACK *psEnc2Use ;

			if(sEm_EncMngrOut.flags.b.bOpenLoop)
				psEnc2Use = &sEncMgrRun.sOpenLoop.sEncoder ;
			else
				psEnc2Use = &sEm_MainEnc ;

		    if(EfsSetup(sEm_EncMngrIn.psMotorData, psEnc2Use, sEm_EncMngrIn.sIRefIn, &sEm_EncMngrOut.sIRefOut, sEm_EncMngrIn.sSpdLoopIRefIn, &sEncMgrRun.sEfsRun)==EFS_RV_INPROGRESS)
		    	sEncMgrRun.sFlags.b.bEFSInProgress=TRUE;
        }
#else
	    if(EfsSetup(sEm_EncMngrIn.psMotorData, &sEm_MainEnc, sEm_EncMngrIn.sIRefIn, &sEm_EncMngrOut.sIRefOut, sEm_EncMngrIn.sSpdLoopIRefIn, &sEncMgrRun.sEfsRun)==EFS_RV_INPROGRESS)
	    	sEncMgrRun.sFlags.b.bEFSInProgress=TRUE;
#endif
    }
#endif

    sEncMgrRun.sFlags.b.bPrevPSState=sEm_EncMngrIn.psMotorData->sPowerStageSts.b.bFullyActive;

        // if EFS active
    if(sEncMgrRun.sFlags.b.bEFSInProgress)
    {
            // copy PSTAGE control to out and activate current references
        sEm_EncMngrOut.sPowerStageCtrlOut=*sEm_EncMngrIn.sPowerStageCtrlIn;
        sEm_EncMngrOut.sPowerStageCtrlOut.b.bReferenceEnable=TRUE;

#if CFG_ENC_EFS
            // execute EFS processing; if in progress output setup feedback electrical angle
        uwElecAngle=sEm_Fbk2CntrLoop.uwElecAngle;
        uwRetVal=EfsProcess(&sEncMgrRun.sEfsRun, &uwElecAngle);

        if(uwRetVal==EFS_RV_INPROGRESS)
        { // until EFS is in progress, exit here
            sEm_Fbk2CntrLoop.uwElecAngle=uwElecAngle;
            return TRUE;  // EFS status (from EfsProcess) is in progress, EXIT HERE
        }

            // if terminated and valid send corrections
        if(uwRetVal==EFS_RV_TERMINATED)
        {
            if(sEncMgrRun.pfMainAbsEACorrect)
                (*sEncMgrRun.pfMainAbsEACorrect)(uwElecAngle);

            if(sEncMgrRun.pfMainRelEACorrect)
                (*sEncMgrRun.pfMainRelEACorrect)(uwElecAngle);
        }

        sEncMgrRun.sFlags.b.bEFSInProgress=FALSE;
#endif

            // Set main fb elec angle to right value and enable ELE_ANGLE_VALID
        sEm_Fbk2CntrLoop.uwElecAngle = *sEncMgrRun.puwElecAngle2Use ;
        sEm_Fbk2CntrLoop.ubStatus   |= ENCMGR_ELE_ANGLE_VALID;

#if CFG_ENCMGR_OPENLOOP
        // Pay attention that OpenLoop must override BEMF
		if(sEm_EncMngrOut.flags.b.bOpenLoop)
		{
#if CFG_ENC_BEMF
	        if(sEncMgrRun.sFlags.b.bBEMFHook)
	            Be_Hook8KHz(NULL, &sEm_EncMngrOut.sIRefOut);
#endif // cfg_enc_bemf
	        OpenLoopEnc8KHz(NULL, &sEm_EncMngrOut.sIRefOut);
		}
		else
		{
#if CFG_ENC_BEMF
	        if(sEncMgrRun.sFlags.b.bBEMFHook)
	            Be_Hook8KHz(NULL, &sEm_EncMngrOut.sIRefOut);
#endif // cfg_enc_bemf
		}
#else // cfg_encmgr_openloop
#if CFG_ENC_BEMF
            // if BEMF hook active then notify current transition
        	// set NULL pointer: in this way backemf knows EFS has ended
        if(sEncMgrRun.sFlags.b.bBEMFHook)
            Be_Hook8KHz(NULL, &sEm_EncMngrOut.sIRefOut);
#endif // cfg_enc_bemf
#endif // cfg_encmgr_openloop

#if CFG_ENC_EFS
            // signal fault
        if(uwRetVal==EFS_RV_FAULT && !sEncMgrRun.sErrorSent.b.bEfsFault)
        {
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_ENCODERMANAGER_FAIL, SYSTEMALARMS_SUBCODE_ENCMGR_EFSFAIL, bSysStatPowerEnabled) ;
            sEncMgrRun.sErrorSent.b.bEfsFault=TRUE;
        }
#endif // cfg_enc_efs
        return TRUE; // when EFS is in progress, hook8kHz EXIT HERE
    }

    	// ****** EFS procedure is no more in progress ******
        // copy in PSTAGE control to out, as BEMF do not use it
    sEm_EncMngrOut.sPowerStageCtrlOut=*sEm_EncMngrIn.sPowerStageCtrlIn;

#if CFG_ENCMGR_OPENLOOP
    // Pay attention that OpenLoop must override BEMF
	if(sEm_EncMngrOut.flags.b.bOpenLoop)
	{	// open loop: hook8kHz EXIT HERE
#if CFG_ENC_BEMF
		if(sEncMgrRun.sFlags.b.bBEMFHook)
		{	// keep bemf stopped (for bemf EFS never ends)
			Be_Hook8KHz(NULL, &sEm_EncMngrOut.sIRefOut);
		}
#endif  // cfg_enc_bemf

		sEm_EncMngrOut.sPowerStageCtrlOut.b.bReferenceEnable = sEm_EncMngrIn.psMotorData->sPowerStageSts.b.bFullyActive;

		if(sEm_EncMngrIn.sPowerStageCtrlIn->b.bReferenceEnable)
			return OpenLoopEnc8KHz(sEm_EncMngrIn.sIRefIn, &sEm_EncMngrOut.sIRefOut);
		else
		{	// when no power, keep OpenLoop encoder in reset (needed NULL pointer)
			return OpenLoopEnc8KHz(NULL, &sEm_EncMngrOut.sIRefOut);
		}
	}
	else
	{
#if CFG_ENC_BEMF
			// if BEMF hook active
		if(sEncMgrRun.sFlags.b.bBEMFHook)
		{	// BEMF: hook8kHz EXIT HERE
				// here reference must be always enabled to keep steady,
				// if incoming reference enable is off then override
				// with zero current references
			sEm_EncMngrOut.sPowerStageCtrlOut.b.bReferenceEnable = sEm_EncMngrIn.psMotorData->sPowerStageSts.b.bFullyActive;

			if(sEm_EncMngrIn.sPowerStageCtrlIn->b.bReferenceEnable)
				return Be_Hook8KHz(sEm_EncMngrIn.sIRefIn, &sEm_EncMngrOut.sIRefOut);
			else
			{
				GLB_IREF sLRefNull = {0l,0l};
				return Be_Hook8KHz(&sLRefNull, &sEm_EncMngrOut.sIRefOut);
			}
		}
#endif // cfg_enc_bemf
	}
#else // cfg_encmgr_openloop
#if CFG_ENC_BEMF
        // if BEMF hook active
    if(sEncMgrRun.sFlags.b.bBEMFHook)
    {	// BEMF: hook8kHz EXIT HERE
            // here reference must be always enabled to keep steady,
            // if incoming reference enable is off then override
            // with zero current references
        sEm_EncMngrOut.sPowerStageCtrlOut.b.bReferenceEnable = sEm_EncMngrIn.psMotorData->sPowerStageSts.b.bFullyActive;

        if(sEm_EncMngrIn.sPowerStageCtrlIn->b.bReferenceEnable)
            return Be_Hook8KHz(sEm_EncMngrIn.sIRefIn, &sEm_EncMngrOut.sIRefOut);
        else
        {
            GLB_IREF sLRefNull = {0l,0l};
            return Be_Hook8KHz(&sLRefNull, &sEm_EncMngrOut.sIRefOut);
        }
    }
#endif // cfg_enc_bemf
#endif // cfg_encmgr_openloop
        // otherwise copy in to out (hook8kHz EXIT HERE)
    sEm_EncMngrOut.sIRefOut=*sEm_EncMngrIn.sIRefIn;

    return TRUE ; 
}

//***************************************************************************
// slow task

static void slowtask(void)
{
    paramcheck();
    elecanglefftime();
#if CFG_ENC_EFS
    absreldiffcheck();
    EfsParametersCheck();
#endif
#if CFG_ENC_ENDAT
    En_ParametersCheck(ENDAT_SEL_MAIN);
#ifndef _HW_DC
    En_ParametersCheck(ENDAT_SEL_AUX);
#endif
#endif
#if CFG_ENC_SINCOS
    Sc_ParametersCheck();
#endif
#if CFG_ENC_HALL
    Hl_ParametersCheck();
#endif
#if CFG_ENC_HIP
    Hf_ParametersCheck();
#endif
#if CFG_ENC_INCR
    Ic_EncoderParametersCheck(INCREMENTAL_SEL_MAIN);
    Ic_EncoderParametersCheck(INCREMENTAL_SEL_AUX);
    Ic_SimulationParametersCheck(sEm_EncMngrParam.uwMainRelSel==ENCODER_TYPE_REL_INCREMENTAL);
#endif
#if CFG_ENC_BEMF
    Be_ParametersCheck();
#endif

#if CFG_ENC_EFS
    EfsSlowTask();
#endif
#if CFG_ENC_NIKON
    Nk_ParametersCheck(NIKON_SEL_MAIN);
#endif
#if CFG_ENC_TMGW
    Tm_ParametersCheck(TMGW_SEL_MAIN);
#endif
#ifndef _HW_DC
    adjustvencoder();
#endif
}

//***************************************************************************
// parameter validity check

static BOOL paramcheck(void)
{
    BOOL bValid=TRUE;
    SLONG slSpeed;

    if(sEm_EncMngrParam.uwVEncVoltage>=1024)
    {
        ParChk_SignalValueError(PARCC_ENCMGR_ENCODERVOLTAGE);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_ENCMGR_ENCODERVOLTAGE);

#ifdef _HW_DC
    if(sEm_EncMngrParam.uwAuxVEncVoltage>=1024)
    {
        ParChk_SignalValueError(PARCC_ENCMGR_AUXENCODERVOLTAGE);
        bValid=FALSE;
    }
else
        ParChk_ResetValueError(PARCC_ENCMGR_AUXENCODERVOLTAGE);
#endif

    if(sEm_EncMngrParam.uwVEncDelay>=10000)
    {
        ParChk_SignalValueError(PARCC_ENCMGR_VOLTAGEDELAY);
        bValid=FALSE;
    }
    else
        ParChk_ResetValueError(PARCC_ENCMGR_VOLTAGEDELAY);

    switch(sEm_EncMngrParam.uwMainAbsSel)
    {
        case ENCODER_TYPE_NULL:
        case ENCODER_TYPE_ABS_ENDAT:
        case ENCODER_TYPE_ABS_ANALOG:
        case ENCODER_TYPE_ABS_HALL:
        case ENCODER_TYPE_ABS_HIPERFACE:
        case ENCODER_TYPE_ABS_NIKON:
        case ENCODER_TYPE_ABS_TMGW:
        case ENCODER_TYPE_ABS_BISS:
            ParChk_ResetValueError(PARCC_ENCMGR_MAINABSSEL);
            break;

        default:
            ParChk_SignalValueError(PARCC_ENCMGR_MAINABSSEL);
            bValid=FALSE;
            break;
    }
        
    switch(sEm_EncMngrParam.uwMainRelSel)
    {
        case ENCODER_TYPE_NULL:
        case ENCODER_TYPE_REL_BACK_EMF:
        case ENCODER_TYPE_REL_INCREMENTAL:
            ParChk_ResetValueError(PARCC_ENCMGR_MAINRELSEL);
            break;

        default:
            ParChk_SignalValueError(PARCC_ENCMGR_MAINRELSEL);
            bValid=FALSE;
            break;
    }

    switch(sEm_EncMngrParam.uwAuxSel)
    {
        case ENCODER_TYPE_NULL:
        case ENCODER_TYPE_REL_INCREMENTAL:
        case ENCODER_TYPE_ABS_ENDAT:
        case ENCODER_TYPE_ABS_BISS:
            ParChk_ResetValueError(PARCC_ENCMGR_AUXSEL);
            break;

        default:
            ParChk_SignalValueError(PARCC_ENCMGR_AUXSEL);
            bValid=FALSE;
            break;
    }

    switch(sEm_EncMngrParam.uwSimSel)
    {
        case ENCODER_TYPE_NULL:
        case ENCODER_TYPE_REL_INCRSIMUL:
            ParChk_ResetValueError(PARCC_ENCMGR_SIMSEL);
            break;

        default:
            ParChk_SignalValueError(PARCC_ENCMGR_SIMSEL);
            bValid=FALSE;
            break;
    }

#ifndef _HW_DC
        // for controlboards up to 5.00 aux in and sim out together
        // cannot be enabled
    if(sGlbControlBoardParameters.sProductInfo.uwProductRev < 600)
        if(sEm_EncMngrParam.uwAuxSel==ENCODER_TYPE_REL_INCREMENTAL &&
           sEm_EncMngrParam.uwSimSel==ENCODER_TYPE_REL_INCRSIMUL)
        {
            ParChk_SignalValueError(PARCC_ENCMGR_AUXSIMMISMATCH);
            bValid=FALSE;
        }
        else
            ParChk_ResetValueError(PARCC_ENCMGR_AUXSIMMISMATCH);
#endif

    atomic_read(&slSpeed, &sEm_EncMngrParam.slMaxSpeed, sizeof(sEm_EncMngrParam.slMaxSpeed));
    if(slSpeed<=0l)
    {
        FLOAT flSpeed;

        atomic_read(&flSpeed, &sGlbMotorParameters.flSpeedNominal, sizeof(FLOAT));
        flSpeed*=SPEED_K_CONVERSION*60.0/(2.0*FLOAT_PI);
        slSpeed=(SLONG)(flSpeed*OVERSPEED_RATIO);
    }
    atomic_write(&sEncMgrRun.slOverSpeed, &slSpeed, sizeof(slSpeed));

    return bValid;
}

//***************************************************************************
// no encoder init, give back an always valid structure

static BOOL noencoderinit(ENCMGR_SPACEFEEDBACK *sNoEncoder)
{
    INT64_ASSIGN(sNoEncoder->sEncData.sqPostn, 0L, 0UL) ;
    INT64_ASSIGN(sNoEncoder->sqMechAbsPosOffset, 0L, 0UL) ;  
    sNoEncoder->uwElecAngle = 0 ;
    sNoEncoder->uwDeltaElecAngle = 0 ;
    sNoEncoder->sEncData.slSpeed = 0L ;
    sNoEncoder->sEncData.slAccel = 0L ;
    sNoEncoder->ubStatus = ENCMGR_RELATIVE_VALID | ENCMGR_ELE_ANGLE_VALID | ENCMGR_ABSMECHTURN_VALID ;

    return TRUE ; 
}

//***************************************************************************
// Convert old V14 data parameters if found while restoring data from NV mem

SWORD Em_GetV14Params(ENCMGR_V14_PARAMS  * pvSrc, UWORD uwSize)
{
    if(sizeof(ENCMGR_V14_PARAMS)!=uwSize)
        return 0;

    sEm_EncMngrParam.flags.b.bPos2CntrLoop     = pvSrc->flags.b.bPos2CntrLoop;
    sEm_EncMngrParam.flags.b.bSpd2CntrLoop     = pvSrc->flags.b.bSpd2CntrLoop;
    sEm_EncMngrParam.flags.b.bAcc2CntrLoop     = pvSrc->flags.b.bAcc2CntrLoop;
    sEm_EncMngrParam.flags.b.bElecAngle2Fpga   = pvSrc->flags.b.bElecAngle2Fpga;
    sEm_EncMngrParam.flags.b.bDisableIfRelFail = pvSrc->flags.b.bDisableIfRelFail;
    sEm_EncMngrParam.flags.b.bDisableEPlate    = pvSrc->flags.b.bDisableEPlate;
    sEm_EncMngrParam.flags.b.bRestoreEPlate    = pvSrc->flags.b.bRestoreEPlate;
    sEm_EncMngrParam.uwVEncVoltage             = pvSrc->uwVEncVoltage;
    sEm_EncMngrParam.uwMainAbsSel              = pvSrc->uwMainAbsSel;
    sEm_EncMngrParam.uwAuxSel                  = pvSrc->uwAuxSel;
    sEm_EncMngrParam.uwVEncDelay               = pvSrc->uwVEncDelay;
    sEm_EncMngrParam.uwMainRelSel              = pvSrc->uwMainRelSel;
    sEm_EncMngrParam.uwMaxAbsRelDiff           = pvSrc->uwMaxAbsRelDiff;

    sEm_LastValidEPlate.ulSerialNumber         = pvSrc->sEPlate.ulSerialNumber;
    sEm_LastValidEPlate.ulProductionDate       = pvSrc->sEPlate.ulProductionDate;
    sEm_LastValidEPlate.ulModel                = pvSrc->sEPlate.ulModel;
    sEm_LastValidEPlate.flResistance           = pvSrc->sEPlate.flResistance;
    sEm_LastValidEPlate.flInductance           = pvSrc->sEPlate.flInductance;
    sEm_LastValidEPlate.flKT                   = pvSrc->sEPlate.flKT;
    sEm_LastValidEPlate.flCurrentNominalZeroSpeed = pvSrc->sEPlate.flCurrentNominalZeroSpeed;
    sEm_LastValidEPlate.flCurrentNominal       = pvSrc->sEPlate.flCurrentNominal;
    sEm_LastValidEPlate.flCurrentPeak          = pvSrc->sEPlate.flCurrentPeak;
    sEm_LastValidEPlate.flSpeedNominal         = pvSrc->sEPlate.flSpeedNominal;
    sEm_LastValidEPlate.uwThermalConstant      = pvSrc->sEPlate.uwThermalConstant;
    sEm_LastValidEPlate.flMotorInertia         = pvSrc->sEPlate.flMotorInertia;
    sEm_LastValidEPlate.uwPhaseOffset          = pvSrc->sEPlate.uwPhaseOffset;
    sEm_LastValidEPlate.ulType                 = pvSrc->sEPlate.ulType;
    sEm_LastValidEPlate.uwPoleNumbers          = pvSrc->sEPlate.uwPoleNumbers;
    sEm_LastValidEPlate.flCoolingTempOn        = pvSrc->sEPlate.flCoolingTempOn;
    sEm_LastValidEPlate.flCoolingTempOff       = pvSrc->sEPlate.flCoolingTempOff;
    sEm_LastValidEPlate.flMaximumTemp          = pvSrc->sEPlate.flMaximumTemp;

    return 0;
}

//***************************************************************************
// Setup specific PLC options

BOOL Em_SetPlcOption(UWORD uwOption, ULONG ulValue)
{
    switch(uwOption)
    {
        case ENCMGR_ENABLE_SINCOSHW:
            sEncMgrRun.sFlags.b.bEnSinCosHw=TRUE;
            return TRUE;

        case ENCMGR_SETUP_VENCODER:
            if(ulValue)
            {
                sEm_EncMngrParam.uwVEncVoltage=(UWORD)((FLOAT)ulValue*0.0799);
                setvencoder();
            }
            else
                turnoffvencoder();
            return TRUE;

        case ENCMGR_DIS_FAULTMGM:
            if(ulValue==ENCMGR_DIS_FAULTMGM_KEY && bSysStatBooting)
                sEncMgrRun.sFlags.b.bDisFaultMgm=TRUE;
            return TRUE;

#ifdef _HW_DC
        case ENCMGR_SETUP_AUXVENCODER:
            if(ulValue)
            {
                sEm_EncMngrParam.uwAuxVEncVoltage=(UWORD)((FLOAT)ulValue*0.0799);
                setauxvencoder();
            }
            else
                turnoffauxvencoder();
            return TRUE;
#endif

        case ENCMGR_ENABLE_INCREMENTALHW:
            sEncMgrRun.sFlags.b.bEnIncrementalHw=TRUE;
            return TRUE;

        default:
            return FALSE;
    }
}

//***************************************************************************
// Setup ElecAngle Feed Forward Time

static BOOL elecanglefftime(void)
{
    SLONG slFFCalc; 

    // calculate scaling factor (Q13) for electrical angle feed forward calculated every 125us
    slFFCalc = (SLONG)(8192.0 * (FLOAT)REALTIME_TASK_FREQ * ((FLOAT)sEm_EncMngrParam.swElecAngleFFTime/1000000.0)) ;

    // boundary check (+/-500us)
    if(slFFCalc<-32767l || slFFCalc>32767l)
        return FALSE;

    // if EFS is in progress, NO Elec Angle Feed Forward
    if(sEncMgrRun.sFlags.b.bEFSInProgress)
        sEncMgrRun.swElecAngleFFScaling = 0 ;
    else
        sEncMgrRun.swElecAngleFFScaling = (SWORD)slFFCalc;

    return TRUE;
}

//***************************************************************************
// Calculate max abs/rel diff for abs encoder that cannot feedback
// for control loops

static BOOL absreldiffcheck(void)
{
    ULONG ulPosRel;

    // no new snapshot
    if(!sEncMgrRun.sFlags.b.bExecAbsRelDiff)
        return TRUE;

    // if relative has snapshot capability
    if(sEncMgrRun.pfMainRelGetSnapshot)
        // if snapshot available
        if((*sEncMgrRun.pfMainRelGetSnapshot)(&ulPosRel))
        {
            maxabsreldiff_verify(sEm_MainAbsEnc.sEncData.sqPostn.lo,ulPosRel);
        }

    // signal terminated and exit
    sEncMgrRun.sFlags.b.bExecAbsRelDiff=FALSE;
    return TRUE;
}

//***************************************************************************
#if (CFG_ENCMGR_OPENLOOP)
static BOOL OpenLoopEncInit(void)
{
    if(sEm_EncMngrOut.flags.b.bRequireIRefHook)
    	sEm_EncMngrOut.flags.b.bOpenLoop = sEm_EncMngrParam.flags.b.bOpenLoop ;
    else
    	sEm_EncMngrOut.flags.b.bOpenLoop = FALSE ;

	sEncMgrRun.sOpenLoop.uwPolePairs = sEncMgrRun.uwMotorPoleNumbers / 2 ;
	sEncMgrRun.sOpenLoop.ulMechSteps = ((ULONG)(0UL - sEncMgrRun.sOpenLoop.uwPolePairs) / sEncMgrRun.sOpenLoop.uwPolePairs) + 1 ;
	sEncMgrRun.sOpenLoop.ulMechAdder = 0UL ;

	OpenLoopEncResetOut() ;

    return TRUE ;
}

// manage open loop
static BOOL OpenLoopEnc8KHz(GLB_IREF *psIRefIn, GLB_IREF *psIRefOut)
{
    // if NULL then I don't have to do anything, out = present reference
    if(psIRefIn == NULL)
    {
    	sEncMgrRun.sOpenLoop.flags.b.bHookInTransition = TRUE ;
    	OpenLoopEncResetOut() ;
        return TRUE ;
    }

    if(sEncMgrRun.sOpenLoop.flags.b.bHookInTransition)
    {	// declare open loop encoder ready
    	sEncMgrRun.sOpenLoop.sEncoder.ubStatus = (ENCMGR_RELATIVE_VALID | ENCMGR_EFS_READY | ENCMGR_ELE_ANGLE_VALID) ;
    	sEncMgrRun.sOpenLoop.flags.b.bHookInTransition = FALSE ;
    }

    // Current output
    psIRefOut->slIdRef = psIRefIn->slIdRef + sEncMgrRun.sEfsRun.ulIdRampCurrent ; // keep EFS Id value
    psIRefOut->slIqRef = 0 ; // since I'm in open loop, keep IqRef = 0

    OpenLoopEncData() ;

    return TRUE ;
}

static void OpenLoopEncResetOut(void)
{
	sEncMgrRun.sOpenLoop.slSpeedOPL  = 0L ;
	sEncMgrRun.sOpenLoop.slPostnOPL  = 0L ;
	sEncMgrRun.sOpenLoop.ulMechAdder = 0UL ;
	sEncMgrRun.sOpenLoop.swElecTurns = 0 ;
	sEncMgrRun.sOpenLoop.ulElecAngle = 0UL ;

	OpenLoopEncZeroOut() ;
}

static void OpenLoopEncZeroOut(void)
{
	INT64_ASSIGN(sEncMgrRun.sOpenLoop.sEncoder.sEncData.sqPostn, 0L, 0UL) ;
	INT64_ASSIGN(sEncMgrRun.sOpenLoop.sEncoder.sqMechAbsPosOffset, 0L, 0UL) ;
	sEncMgrRun.sOpenLoop.sEncoder.sEncData.slSpeed = 0 ;
	sEncMgrRun.sOpenLoop.sEncoder.sEncData.slAccel = 0 ;
	sEncMgrRun.sOpenLoop.sEncoder.uwElecAngle      = 0 ;
	sEncMgrRun.sOpenLoop.sEncoder.swElecSpeed      = 0 ;
	sEncMgrRun.sOpenLoop.sEncoder.ubStatus         = ENCMGR_EFS_READY ;
}

static void OpenLoopEncData(void)
{
    ULONG ulElecAngle_1, ulMechEncAngle_1, ulMechEncAngle ;
    SLONG slPostnOPL_1 ;

    // speedOPL = SpeedRef (open loop uses reference speed as open loop speed)
    sEncMgrRun.sOpenLoop.slSpeedOPL = sEm_EncMngrIn.psRef->slSpeed ;

    // positionOPL = electrical position
    slPostnOPL_1 = sEncMgrRun.sOpenLoop.slPostnOPL ;
    sEncMgrRun.sOpenLoop.slPostnOPL = slPostnOPL_1 + ((sEncMgrRun.sOpenLoop.slSpeedOPL * sEncMgrRun.sOpenLoop.uwPolePairs + 0x80) >> 8) + ((SLONG)sEncMgrRun.sOpenLoop.swElecTurns << 24) ;

    // ------------------------------------------------------------------------
    // -------------------- turns number OL  overflow mgmt --------------------
    // ------------------------------------------------------------------------
    ulElecAngle_1 = sEncMgrRun.sOpenLoop.ulElecAngle ;
    sEncMgrRun.sOpenLoop.ulElecAngle = (ULONG)sEncMgrRun.sOpenLoop.slPostnOPL << 8 ;

    if ( (                     ulElecAngle_1 & 0x80000000) &&  (                    ulElecAngle_1 & 0x40000000) &&
        !(sEncMgrRun.sOpenLoop.ulElecAngle   & 0x80000000) && !(sEncMgrRun.sOpenLoop.ulElecAngle  & 0x40000000)   )
    { // Clockwise Overflow
        sEncMgrRun.sOpenLoop.swElecTurns  = -1 ;
        sEncMgrRun.sOpenLoop.ulMechAdder += sEncMgrRun.sOpenLoop.ulMechSteps ; // update adder
    }
    else if (!(                     ulElecAngle_1 & 0x80000000) && !(                     ulElecAngle_1 & 0x40000000) &&
              (sEncMgrRun.sOpenLoop.ulElecAngle   & 0x80000000) &&  (sEncMgrRun.sOpenLoop.ulElecAngle   & 0x40000000)   )
    { // Anticlockwise Overflow
        sEncMgrRun.sOpenLoop.swElecTurns  = +1 ;
        sEncMgrRun.sOpenLoop.ulMechAdder -= sEncMgrRun.sOpenLoop.ulMechSteps ; // update adder */
    }
    else
        sEncMgrRun.sOpenLoop.swElecTurns = 0 ;
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    // ---------- Mechanical Outputs (Position, Speed, Acceleration) ----------
    // ------------------------------------------------------------------------
    // saving previous mechanical data necessary to calculate turns number, speed and acceleration
    ulMechEncAngle_1 = sEncMgrRun.sOpenLoop.sEncoder.sEncData.sqPostn.lo ;

    // calculating mechanical angle
    ulMechEncAngle = sEncMgrRun.sOpenLoop.ulElecAngle / sEncMgrRun.sOpenLoop.uwPolePairs + sEncMgrRun.sOpenLoop.ulMechAdder ;

    // calculating mechanical turns number
    if ( (ulMechEncAngle_1 & 0x80000000) &&  (ulMechEncAngle_1 & 0x40000000) &&
        !(ulMechEncAngle   & 0x80000000) && !(ulMechEncAngle   & 0x40000000)   )
    {
        sEncMgrRun.sOpenLoop.sEncoder.sEncData.sqPostn.hi++ ; // Clockwise Overflow
    }
    else if (!(ulMechEncAngle_1 & 0x80000000) && !(ulMechEncAngle_1 & 0x40000000) &&
              (ulMechEncAngle   & 0x80000000) &&  (ulMechEncAngle   & 0x40000000)   )
    {
        sEncMgrRun.sOpenLoop.sEncoder.sEncData.sqPostn.hi-- ; // Anticlockwise Overflow
    }

    sEncMgrRun.sOpenLoop.sEncoder.sEncData.sqPostn.lo = ulMechEncAngle ;
    sEncMgrRun.sOpenLoop.sEncoder.sEncData.slSpeed    = sEncMgrRun.sOpenLoop.slSpeedOPL ;
    sEncMgrRun.sOpenLoop.sEncoder.sEncData.slAccel    = 0l ;
    sEncMgrRun.sOpenLoop.sEncoder.uwElecAngle         = HIWORD(sEncMgrRun.sOpenLoop.ulElecAngle) ;

    // Actual electrical speed = MechSpeed * MotorPolePairs
    sEncMgrRun.sOpenLoop.sEncoder.swElecSpeed = (SWORD)_sint32_scale_32(sEncMgrRun.sOpenLoop.sEncoder.sEncData.slSpeed, sEncMgrRun.sOpenLoop.uwPolePairs) ;
    // ------------------------------------------------------------------------
}
#endif
