/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright � 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EncoderEFSeek.c                                            */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Electrical Field Direction auto-seek                       */
/*               (extension of EncoderManager)                              */
/*                                                                          */
/****************************************************************************/
#include <stdlib.h>	//to use labs()
#include <math.h>
#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonUtility.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "drive\EncoderEFSeek.h"
#include "drive\DriveTaskController.h"
#include "common\Int64Functions.h"
#include "common\DspFunctions.h"

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

#define ELECANGLESTEADYTIMEOUT              128     // ms

#define PI_GAIN_GLOBAL_MAX_SHIFT_RIGHT      16
#define PI_GAIN_GLOBAL_MAX_SHIFT_LEFT       8

#define PI_GAIN_COEFFREF   1129931.034 // = (1.28 * (8 * 8) * (8 * 0.5)) / 0.00029
#define PI_GAIN_KPREF      3.0         // = 1536 / 2^9
#define PI_GAIN_KIREF      0.015625    // = 8 / 2^9
#define PI_GAIN_LOG10OF2   (log10(2.0))
#define PI_GAIN_SHIFTCOEFF (32768.0 / PI_GAIN_KPREF)

//****************************************************************************
// General global variables

EFS_PARAMS sEfsParam;
EFS_OUT    sEfsOut;

    // double copy in order to let fast setup in realtime
    // task without validity check
#ifdef _INFINEON_
EFS_PARAMS sdata sEfsCheckedParam;
#else
EFS_PARAMS sEfsCheckedParam;
#endif // _infineon_

    // used for space speed control loop feeding during IdRamp and
    // position control loop procedure
EFS_SSIN_IDR_AND_PLOOP sEfsSSInIdrAndPLoop;

//****************************************************************************
// Default parameters
#ifdef _INFINEON_
const EFS_PARAMS huge sEfsDefParam=
#else
const EFS_PARAMS sEfsDefParam=
#endif // _infineon_
{
    EFS_TYPE_DISABLED,
    0,

    300,                                // 10%
    500,                                // 500 msec

    {0},
    (ULONG)(SPEED_K_CONVERSION*50),     // 50 rpm

    1000,                               // 100%
    0,                                  // 0 msec
    
    {                                   // U3.503.30.3.Sincos
        162,                            // Ki
        31103,                          // Kp
        4550,                           // Err Max: 15 elec deg
        13,                             // Global shift
        65535,                          // Integral limit: 359.9 elec deg
    },
};

#if CFG_ENC_EFS
//****************************************************************************
// Locals

static SLONG slMotorCurrentPeak;

#ifdef _INFINEON_
static BOOL bdata bAbortInProgress=FALSE;
static BOOL bdata bTerminationInProgress=FALSE;
static BOOL bdata bReqTaskSetup=FALSE;
static BOOL bdata bAckTaskSetup=FALSE;
static BOOL bdata bReqRstSetup=FALSE;
static BOOL bdata bAckRstSetup=FALSE;
static BOOL bdata bLocPIRunning=FALSE;
#else // _infineon_
static BOOL bAbortInProgress=FALSE;
static BOOL bTerminationInProgress=FALSE;
static BOOL bReqTaskSetup=FALSE;
static BOOL bAckTaskSetup=FALSE;
static BOOL bReqRstSetup=FALSE;
static BOOL bAckRstSetup=FALSE;
static BOOL bLocPIRunning=FALSE;
#endif // _infineon_

static FLOAT flEfsPiMotorGainBoot;


static BOOL EfsPi_8KHz(SWORD swFbk, EFS_PI_RUNTIME *psEfsPI8kHz_Run, EFS_PI_PARAM *psEfsPI_CheckedParam, EFS_PI_OUTPUT *psEfsPI8kHz_Out) ;
static BOOL EfsPi_Param(EFS_PI_PARAM *psEfsPI_CheckedParam, EFS_PI_PARAM *psEfsPI_Param) ;
static void EfsPi_Reset(EFS_PI_RUNTIME *psEfsPI_Run, EFS_PI_OUTPUT *psEfsPI_Out) ;

//****************************************************************************
// Init entry point

void EfsInit(void)
{
        // shadow copy of EFS parameters
    sEfsCheckedParam=sEfsParam;

        // motor current peak [Arms * 10]
    slMotorCurrentPeak=(SLONG)(sGlbMotorParameters.flCurrentPeak * 10.0);
    
        // EFS PI Local: motor parameter coeff = (Kt * Poles^2 * CurrentPeak/1000.0) / J
    flEfsPiMotorGainBoot = (sGlbMotorParameters.flKT * ((FLOAT)sGlbMotorParameters.uwPoleNumbers * sGlbMotorParameters.uwPoleNumbers) * (sGlbMotorParameters.flCurrentPeak / 1000.0)) / sGlbMotorParameters.flMotorInertia ;

}

//****************************************************************************
// Setup data process

SWORD EfsSetup(MH_MOTORDATA_OUT * psPSStatus,ENCMGR_SPACEFEEDBACK * psFeedback,GLB_IREF * psIRefIn,GLB_IREF * psIRefOut,GLB_IREF * psSpdLoopIRefIn,EFS_RUNTIME * psRuntime)
{
        // record all input/output data structures
    psRuntime->psPSStatus       =psPSStatus;
    psRuntime->psFeedback       =psFeedback;
    psRuntime->psIRefIn         =psIRefIn;
    psRuntime->psIRefOut        =psIRefOut;
    psRuntime->psSpdLoopIRefIn  =psSpdLoopIRefIn;

        // setup procedure
    psRuntime->ubProcType=sEfsCheckedParam.ubProcType;
    psRuntime->bForce =sEfsCheckedParam.bForce;//psRuntime->options=sEfsCheckedParam.options;

    sEfsOut.uwEfsStatus=0;

    switch(psRuntime->ubProcType)
    {
            // disabled, do nothing
        case EFS_TYPE_DISABLED:
            return EFS_RV_INPROGRESS;

            // Id Ramp with or w/o position control loop
        case EFS_TYPE_IDR_AND_PLOOP:
            bReqTaskSetup=bAckTaskSetup=bReqRstSetup=bAckRstSetup=bAbortInProgress=bTerminationInProgress=FALSE;
            psRuntime->slElecAngleMult=(SLONG)sEfsCheckedParam.slElecAngleFeed<<16;
            psRuntime->slElecAngleMult=sEfsCheckedParam.slElecAngleFeed * (sGlbMotorParameters.uwPoleNumbers/2) / slMotorCurrentPeak;

        case EFS_TYPE_IDRAMP:
        case EFS_TYPE_IDR_LOCAL_PID:
                // if not forced and ENCMGR_ELE_ANGLE_VALID is valid
                // then immediately exit
        	if(!psRuntime->bForce && (psRuntime->psFeedback->ubStatus&ENCMGR_ELE_ANGLE_VALID))//if(!psRuntime->options.f.bForce && (psRuntime->psFeedback->ubStatus&ENCMGR_ELE_ANGLE_VALID))
                return EFS_RV_ABORTED;

                // setup number of ticks for the ramps and for the steady state
            psRuntime->uwIdRampQTicksCounter=sEfsCheckedParam.uwIdRampTime;
            psRuntime->uwIdRampTicksCounter=sEfsCheckedParam.uwIdRampTime;
            psRuntime->uwIdSteadyTicksCounter=sEfsCheckedParam.uwIdSteadyTime;

                // then get current step
            psRuntime->ulIdRampCurrentStep=slMotorCurrentPeak * sEfsCheckedParam.uwIdRampCurrent / psRuntime->uwIdRampTicksCounter;

            if(psRuntime->ulIdRampCurrentStep==0)
                psRuntime->ulIdRampCurrentStep=1;

            psRuntime->uwTickCnt=0;
            psRuntime->slSpeedThreshold=sEfsCheckedParam.slSpeedThreshold;

            psRuntime->psIRefOut->slIdRef=0;
            psRuntime->psIRefOut->slIqRef=0;

                // load the timeout to filter the angle
            psRuntime->uwEAngleSnapshotSteadyTicksCounter=ELECANGLESTEADYTIMEOUT; 
            psRuntime->uwEAngleFinalSteadyTicksCounter=ELECANGLESTEADYTIMEOUT;

                // clearing filter status
            psRuntime->slEAngleSnapshotFilter=0;
            psRuntime->slEAngleFinalFilter=0;  

                // reset local PI
            EfsPi_Reset(&psRuntime->sPI_Run, &sEfsOut.sPI_Out);

            sEfsOut.uwPhasingQuality=0;                  

            return EFS_RV_INPROGRESS;

        default:
            assert(FALSE);
            return EFS_RV_ABORTED;
    }
}

//****************************************************************************
// Process aligment procedure, to be called every RT cycle

SWORD EfsProcess(EFS_RUNTIME * psRuntime, UWORD * puwElecAngleCorrection)
{
    UWORD uwElecAngle;

    switch(psRuntime->ubProcType)
    {
            // disabled, do nothing
        case EFS_TYPE_DISABLED:
            *puwElecAngleCorrection=sEfsOut.uwEfsElecAngleCorrection=sEfsOut.uwEfsStatus=0;
            return EFS_RV_TERMINATED;

            // Id Ramp with or w/o position control loop
        case EFS_TYPE_IDR_AND_PLOOP:
            sEfsOut.uwEfsElecAngleCorrection=sEfsOut.uwEfsStatus=0;
            if(bAbortInProgress)
            {
                psRuntime->psIRefOut->slIdRef=0;
                psRuntime->psIRefOut->slIqRef=0;
                if(!bAckRstSetup)
                    return EFS_RV_INPROGRESS;
                else
                {
                    bReqTaskSetup=bAckTaskSetup=bReqRstSetup=bAckRstSetup=bAbortInProgress=bTerminationInProgress=FALSE;
                    return EFS_RV_ABORTED;
                }
            }

            if(bTerminationInProgress)
            {
                if(!bAckRstSetup)
                    return EFS_RV_INPROGRESS;
                else
                {
                    bReqTaskSetup=bAckTaskSetup=bReqRstSetup=bAckRstSetup=bAbortInProgress=bTerminationInProgress=FALSE;
                    return EFS_RV_TERMINATED;
                }
            }
                    
        case EFS_TYPE_IDRAMP:
                // setup zero by default
            *puwElecAngleCorrection=sEfsOut.uwEfsElecAngleCorrection=sEfsOut.uwEfsStatus=0;

                // if not forced, then if at any time ENCMGR_ELE_ANGLE_VALID became valid
                // then exit from procedure
            if(!psRuntime->bForce && (psRuntime->psFeedback->ubStatus&ENCMGR_ELE_ANGLE_VALID))//if(!psRuntime->options.f.bForce && (psRuntime->psFeedback->ubStatus&ENCMGR_ELE_ANGLE_VALID))
            {
            	if(psRuntime->ubProcType==EFS_TYPE_IDR_AND_PLOOP)
                {
                    bReqRstSetup=bAbortInProgress=TRUE;
                    return EFS_RV_INPROGRESS;
                }
                else
                    return EFS_RV_ABORTED;
            }

                // if power disabled then abort
            if(!psRuntime->psPSStatus->sPowerStageSts.b.bFullyActive)
            {
            	if(psRuntime->ubProcType==EFS_TYPE_IDR_AND_PLOOP)
                {
                    bReqRstSetup=bAbortInProgress=TRUE;
                    return EFS_RV_INPROGRESS;
                }
                else
                    return EFS_RV_ABORTED;
            }

                // wait until encoder is ready for EFS
            if(!(psRuntime->psFeedback->ubStatus&ENCMGR_EFS_READY))
            {
                psRuntime->psIRefOut->slIdRef=0;
                psRuntime->psIRefOut->slIqRef=0;
                return EFS_RV_INPROGRESS;
            }

                // if zero, procedure has yet to be started, then wait for speed
            if( (psRuntime->psIRefOut->slIdRef==0) && (psRuntime->psIRefOut->slIqRef==0) && (labs(psRuntime->psFeedback->sEncData.slSpeed) > psRuntime->slSpeedThreshold) )
                return EFS_RV_INPROGRESS;

                // if with position control loop
            if(psRuntime->ubProcType==EFS_TYPE_IDR_AND_PLOOP)
            {
                    // if not still enabled then enable
                if(!bAckTaskSetup)
                {
                        // request task setup
                    bReqTaskSetup=TRUE;
        
                        // and setup reference, same as feedback position
                        // in order to try keep steady the rotor
                    INT64_COPY(sEfsSSInIdrAndPLoop.sDemand.sqPostn, psRuntime->psFeedback->sEncData.sqPostn);
                    sEfsSSInIdrAndPLoop.sDemand.slSpeed=0l;
                    sEfsSSInIdrAndPLoop.sDemand.slAccel=0l;
        
                        // disable (and then reset) integral gain, will be enabled later
                    sEfsSSInIdrAndPLoop.sFlags.b.bPID_IntegralEnable=FALSE;

                        // then wait
                    return EFS_RV_INPROGRESS;
                }
                else
                        // if ack'ed then enable integral gain and begin procedure
                    sEfsSSInIdrAndPLoop.sFlags.b.bPID_IntegralEnable=TRUE;

                    // take IqRef out from position control loop and use it to feed
                    // electrical angle, actual angle for procedure checking is taken from correction
                *puwElecAngleCorrection=uwElecAngle=(UWORD)(_sint32_scale_32(psRuntime->psSpdLoopIRefIn->slIqRef,psRuntime->slElecAngleMult)&0x0000FFFFul);
            }
            else
                    // actual angle for procedure checking is taken from feedback
                uwElecAngle=psRuntime->psFeedback->uwElecAngle;

                // update rt tick
            if(psRuntime->uwTickCnt)
            {
                psRuntime->uwTickCnt--;
                return EFS_RV_INPROGRESS;
            }
            psRuntime->uwTickCnt=REALTIME_TASK_FREQ/1000-1; // reload TickCnt

            	// 1st RAMP: Iq from 0->IqRamp2reach, Id = 0
                // update first ramp tick counter and quadrature current
            if(psRuntime->uwIdRampQTicksCounter)
            {
                psRuntime->psIRefOut->slIqRef+=psRuntime->ulIdRampCurrentStep;
                psRuntime->uwIdRampQTicksCounter--;
                psRuntime->uwEAngleSnapshot=uwElecAngle;
                return EFS_RV_INPROGRESS;
            }

            	// 2nd RAMP: Iq from IqRamp2reach to 0, Id from 0 to IdRamp2reach
                // update second ramp tick counter and both quadrature and direct current
            if(psRuntime->uwIdRampTicksCounter)
            {
                if(psRuntime->psIRefOut->slIqRef>psRuntime->ulIdRampCurrentStep)
                    psRuntime->psIRefOut->slIqRef-=psRuntime->ulIdRampCurrentStep;
                else
                    psRuntime->psIRefOut->slIqRef=0l;
                psRuntime->psIRefOut->slIdRef+=psRuntime->ulIdRampCurrentStep;
                psRuntime->uwIdRampTicksCounter--;
                return EFS_RV_INPROGRESS;
            }

            	// steady state: keep Id current constant for uwIdSteadyTicksCounter time
                // update steady tick counter
            if(psRuntime->uwIdSteadyTicksCounter)
            {
                psRuntime->uwIdSteadyTicksCounter--;
                return EFS_RV_INPROGRESS;
            }

            	// calculate the offset and check if the rotor has moved
                // at the end of procedure check delta electrical angle,
                // if under threshold then procedure fails (only if relative valid)
            sEfsOut.uwPhasingQuality=abs(uwElecAngle-psRuntime->uwEAngleSnapshot);
            if((psRuntime->psFeedback->ubStatus&ENCMGR_RELATIVE_VALID) && sEfsOut.uwPhasingQuality<EFS_VL_ELECANGLE_MIN)
                return EFS_RV_FAULT;

                // procedure terminated
            *puwElecAngleCorrection=*puwElecAngleCorrection; // set correction in encoder manager

                // if with position control loop
            if(psRuntime->ubProcType==EFS_TYPE_IDR_AND_PLOOP)
            {
                    // setup reference same as feedback position
                    // and clear integral enable flag, in order to keep
                    // motor steady until application is restored
                INT64_COPY(sEfsSSInIdrAndPLoop.sDemand.sqPostn, psRuntime->psFeedback->sEncData.sqPostn);
                sEfsSSInIdrAndPLoop.sDemand.slSpeed=0l;
                sEfsSSInIdrAndPLoop.sDemand.slAccel=0l;

                sEfsSSInIdrAndPLoop.sFlags.b.bPID_IntegralEnable=FALSE;

                bReqRstSetup=bTerminationInProgress=TRUE;

                return EFS_RV_INPROGRESS;
            }

            return EFS_RV_TERMINATED;

            // Id Ramp with local PID
        case EFS_TYPE_IDR_LOCAL_PID:
                // setup zero by default
            *puwElecAngleCorrection=sEfsOut.uwEfsElecAngleCorrection=0;

                // if not forced, then if at any time ENCMGR_ELE_ANGLE_VALID became valid
                // then exit from procedure
            if(!psRuntime->bForce && (psRuntime->psFeedback->ubStatus&ENCMGR_ELE_ANGLE_VALID))//if(!psRuntime->options.f.bForce && (psRuntime->psFeedback->ubStatus&ENCMGR_ELE_ANGLE_VALID))
            {
                sEfsOut.uwEfsStatus=0;
                return EFS_RV_ABORTED;
            }
                // if power disabled then abort
            if(!psRuntime->psPSStatus->sPowerStageSts.b.bFullyActive)
            {
                sEfsOut.uwEfsStatus=0;
                return EFS_RV_ABORTED;
            }

                // wait until encoder is ready for EFS
            if(!(psRuntime->psFeedback->ubStatus&ENCMGR_EFS_READY))
            {
                psRuntime->psIRefOut->slIdRef=0;
                psRuntime->psIRefOut->slIqRef=0;
                bLocPIRunning=FALSE;

                EfsPi_Reset(&psRuntime->sPI_Run, &sEfsOut.sPI_Out) ; // integral and output values reset
                sEfsOut.uwEfsStatus=0;
                return EFS_RV_INPROGRESS;
            }

                // if not yet started
            if(!bLocPIRunning)
            {
               sEfsOut.uwEfsStatus=0;
                    // wait for speed
                if(labs(psRuntime->psFeedback->sEncData.slSpeed) > psRuntime->slSpeedThreshold)
                    return EFS_RV_INPROGRESS;

                    // ready to start
                psRuntime->sPI_Run.swFreezedRef = (SWORD)psRuntime->psFeedback->uwElecAngle ;        
                bLocPIRunning=TRUE;
            }
                // execute PI
            EfsPi_8KHz((SWORD)psRuntime->psFeedback->uwElecAngle, &psRuntime->sPI_Run, &sEfsCheckedParam.sPI_Param, &sEfsOut.sPI_Out) ;
            *puwElecAngleCorrection=sEfsOut.uwEfsElecAngleCorrection=(UWORD)(sEfsOut.sPI_Out.slPiOut&0x0000FFFFul); 

                // update rt tick
            if(psRuntime->uwTickCnt)
            {
                psRuntime->uwTickCnt--;
                return EFS_RV_INPROGRESS;
            }
            psRuntime->uwTickCnt=REALTIME_TASK_FREQ/1000-1;

                // update first ramp tick counter and quadrature current
            if(psRuntime->uwIdRampQTicksCounter)
            {
                psRuntime->psIRefOut->slIdRef=0;
                psRuntime->psIRefOut->slIqRef+=psRuntime->ulIdRampCurrentStep;
                psRuntime->uwIdRampQTicksCounter--;
                
                psRuntime->slEAngleSnapshotFilter=sEfsOut.sPI_Out.slPiOut;
                psRuntime->uwEAngleSnapshot=(UWORD)((ULONG)psRuntime->slEAngleSnapshotFilter&0x0000FFFFul);

                sEfsOut.uwEfsStatus=1;
                return EFS_RV_INPROGRESS;
            }

                // wait to filter the snapshot of the electrical angle
            if(psRuntime->uwEAngleSnapshotSteadyTicksCounter)
            {
                psRuntime->uwEAngleSnapshotSteadyTicksCounter--;
                psRuntime->slEAngleSnapshotFilter+=sEfsOut.sPI_Out.slPiOut;
                psRuntime->uwEAngleSnapshot=(UWORD)((ULONG)(psRuntime->slEAngleSnapshotFilter/ELECANGLESTEADYTIMEOUT)&0x0000FFFFul);

                sEfsOut.uwEfsStatus=2;
                return EFS_RV_INPROGRESS;
            }

                // update second ramp tick counter and both quadrature and direct current
            if(psRuntime->uwIdRampTicksCounter)
            {
                psRuntime->psIRefOut->slIqRef=0l;
                psRuntime->psIRefOut->slIdRef+=psRuntime->ulIdRampCurrentStep;
                psRuntime->uwIdRampTicksCounter--;
                
                psRuntime->slEAngleFinalFilter=sEfsOut.sPI_Out.slPiOut;
                psRuntime->uwEAngleFinal=(UWORD)((ULONG)psRuntime->slEAngleFinalFilter&0x0000FFFFul);
                                              
                sEfsOut.uwEfsStatus=3;
                return EFS_RV_INPROGRESS;
            }

                // update steady tick counter
            if(psRuntime->uwIdSteadyTicksCounter)
            {
                psRuntime->uwIdSteadyTicksCounter--;
                psRuntime->slEAngleFinalFilter=sEfsOut.sPI_Out.slPiOut;
                psRuntime->uwEAngleFinal=(UWORD)((ULONG)psRuntime->slEAngleFinalFilter&0x0000FFFFul);

                sEfsOut.uwEfsStatus=4;
                return EFS_RV_INPROGRESS;
            }

                // wait to filter the final electrical angle (done ONLY if ubProcType==EFS_TYPE_IDR_AND_PLOOP)
            if(psRuntime->uwEAngleFinalSteadyTicksCounter)
            {
                psRuntime->uwEAngleFinalSteadyTicksCounter--;
                psRuntime->slEAngleFinalFilter+=sEfsOut.sPI_Out.slPiOut;
                psRuntime->uwEAngleFinal=(UWORD)((ULONG)(psRuntime->slEAngleFinalFilter/ELECANGLESTEADYTIMEOUT)&0x0000FFFFul);

                sEfsOut.uwEfsStatus=5;
                return EFS_RV_INPROGRESS;
            }
                  
                // at the end of procedure check delta electrical angle,
                // if under threshold then procedure fails (only if relative valid)
            sEfsOut.uwPhasingQuality=abs(psRuntime->uwEAngleFinal-psRuntime->uwEAngleSnapshot);

            if(psRuntime->psFeedback->ubStatus&ENCMGR_RELATIVE_VALID)
                if(sEfsOut.uwPhasingQuality<EFS_VL_IDRLPID_EA_MIN || sEfsOut.uwPhasingQuality>EFS_VL_IDRLPID_EA_MAX)
                {
                    EfsPi_Reset(&psRuntime->sPI_Run, &sEfsOut.sPI_Out) ; // integral and output values reset
                    sEfsOut.uwEfsStatus=0;
                    bLocPIRunning=FALSE;                 
                    return EFS_RV_FAULT;
                }

                // procedure terminated
            *puwElecAngleCorrection=sEfsOut.uwEfsElecAngleCorrection=psRuntime->uwEAngleFinal ; // use the filtered value
            EfsPi_Reset(&psRuntime->sPI_Run, &sEfsOut.sPI_Out) ; // integral and output values reset
            sEfsOut.uwEfsStatus=0;
            bLocPIRunning=FALSE;

            return EFS_RV_TERMINATED;

        default:
            assert(FALSE);
            return EFS_RV_ABORTED;
    }
}

//****************************************************************************
// Parameters check

BOOL EfsParametersCheck(void)
{
    BOOL bValid=TRUE;
    SLONG slSpeed;
    UWORD uwIdRamp;

    switch(sEfsParam.ubProcType)
    {
        case EFS_TYPE_DISABLED:
        case EFS_TYPE_IDRAMP:
        case EFS_TYPE_IDR_AND_PLOOP:
        case EFS_TYPE_IDR_LOCAL_PID:
            ParChk_ResetValueError(PARCC_EFS_PROCTYPE);
            break;

        default:
            ParChk_SignalValueError(PARCC_EFS_PROCTYPE);
            bValid=FALSE;
            break;
    }

    uwIdRamp=sEfsParam.uwIdRampCurrent;
    if(uwIdRamp>=1000)
        uwIdRamp=1000;
    sEfsCheckedParam.uwIdRampCurrent=uwIdRamp;

    atomic_read(&slSpeed, &sEfsParam.slSpeedThreshold, sizeof(sEfsParam.slSpeedThreshold));
    if(slSpeed<=0)
    {
        ParChk_SignalValueError(PARCC_EFS_SPEEDTHRESHOLD);
        bValid=FALSE;
    }
    else
    {
        atomic_write(&sEfsCheckedParam.slSpeedThreshold, &slSpeed, sizeof(slSpeed));
        ParChk_ResetValueError(PARCC_EFS_SPEEDTHRESHOLD);
    }

    sEfsCheckedParam.uwIdRampTime=sEfsParam.uwIdRampTime;
    sEfsCheckedParam.bForce =sEfsParam.bForce; //sEfsCheckedParam.options=sEfsParam.options;
    atomic_move(&sEfsCheckedParam.slElecAngleFeed, &sEfsParam.slElecAngleFeed, sizeof(sEfsParam.slElecAngleFeed));
    sEfsCheckedParam.uwIdSteadyTime=sEfsParam.uwIdSteadyTime;

    bValid&=EfsPi_Param(&sEfsCheckedParam.sPI_Param, &sEfsParam.sPI_Param);

    return bValid;
}

//****************************************************************************
// Slow task

void EfsSlowTask(void)
{
    static UBYTE ubActualDTCConfig;

        // when requested from realtime task setup position control loop
    if(bReqTaskSetup && !bAckTaskSetup)
    {
        ubActualDTCConfig=DrvTskCtrl_Config(DRVTSKCTRL_OP_EFS_IDR_AND_PLOOP);
        bAckTaskSetup=TRUE;
    }

        // when requested from realtime task setup previous config
    if(bReqRstSetup && !bAckRstSetup)
    {
        DrvTskCtrl_Config(ubActualDTCConfig);
        bAckRstSetup=TRUE;
    }
}

//****************************************************************************
// EFS PI - 8kHz Function

static BOOL EfsPi_8KHz(SWORD swFbk, EFS_PI_RUNTIME *psEfsPI8kHz_Run, EFS_PI_PARAM *psEfsPI_CheckedParam, EFS_PI_OUTPUT *psEfsPI8kHz_Out)
{
    psEfsPI8kHz_Out->swErr = (psEfsPI8kHz_Run->swFreezedRef + psEfsPI8kHz_Out->swCorrection) - swFbk ; // Error(2^15+s) = Reference(2^15+s) - Feedback(2^15+s)

    if ((psEfsPI8kHz_Out->swErr - psEfsPI_CheckedParam->swErrMax) > 0)
        psEfsPI8kHz_Out->swCorrection -= (psEfsPI8kHz_Out->swErr - psEfsPI_CheckedParam->swErrMax) ; 
    else if ((psEfsPI8kHz_Out->swErr + psEfsPI_CheckedParam->swErrMax) < 0)
        psEfsPI8kHz_Out->swCorrection -= (psEfsPI8kHz_Out->swErr + psEfsPI_CheckedParam->swErrMax) ; 
   
    // == Integral contribute  (and limit it to void 'explosions') == 
    psEfsPI8kHz_Run->slIntegral32 += ((SLONG)psEfsPI8kHz_Out->swErr * psEfsPI_CheckedParam->swKi) ;
    
    if (psEfsPI8kHz_Run->slIntegral32 > psEfsPI_CheckedParam->slIntegralLimit) 
        psEfsPI8kHz_Run->slIntegral32 = psEfsPI_CheckedParam->slIntegralLimit ;
    else if (psEfsPI8kHz_Run->slIntegral32 < -psEfsPI_CheckedParam->slIntegralLimit)
        psEfsPI8kHz_Run->slIntegral32 = -psEfsPI_CheckedParam->slIntegralLimit ;

    // == Proportional (= (2^31+s) = Error(2^15+s) * Kp(2^15+s)) + Integral (= (2^31+s) = Error(2^15+s) * Ki(2^15+s) + slIntegral32_1(2^31+s)) == 
    psEfsPI8kHz_Out->slPiOut = psEfsPI8kHz_Run->slIntegral32 + (SLONG)psEfsPI8kHz_Out->swErr * psEfsPI_CheckedParam->swKp ;  
   
    // == Global  shift ==
    if (psEfsPI_CheckedParam->swGlobalShift > 0) 
        psEfsPI8kHz_Out->slPiOut  >>= psEfsPI_CheckedParam->swGlobalShift  ; // global shift right
    else if(psEfsPI_CheckedParam->swGlobalShift < 0)
        psEfsPI8kHz_Out->slPiOut  <<= (-psEfsPI_CheckedParam->swGlobalShift) ; // global shift left

    // == limit to sOutVal.Max or sOutVal.Min ==
    if (psEfsPI8kHz_Out->slPiOut > psEfsPI_CheckedParam->slOutValLimit)
        psEfsPI8kHz_Out->slPiOut = psEfsPI_CheckedParam->slOutValLimit ;
    else if (psEfsPI8kHz_Out->slPiOut < -psEfsPI_CheckedParam->slOutValLimit)
        psEfsPI8kHz_Out->slPiOut = -psEfsPI_CheckedParam->slOutValLimit ;
    
    return TRUE ;
}


//****************************************************************************
// EFS PI - Param check and update

static BOOL EfsPi_Param(EFS_PI_PARAM *psEfsPI_CheckedParam, EFS_PI_PARAM *psEfsPI_Param)
{
    SLONG slTmp = 0;
    SWORD swTmp = 0;

    // ********************************************************
    // EFS PI Auto Gains: find auto gains for Local PI Efs
    FLOAT flTmp, flEfsPiMotorGainRatio;

    flEfsPiMotorGainRatio = PI_GAIN_COEFFREF / (flEfsPiMotorGainBoot * sEfsCheckedParam.uwIdRampCurrent) ;
  
    // shift is always calculated on the base ok the Kp (it always has the higher value)
    swTmp = (SWORD)(log10(PI_GAIN_SHIFTCOEFF / flEfsPiMotorGainRatio) / PI_GAIN_LOG10OF2) ; 

    // clip to the maximum value
    if(swTmp > PI_GAIN_GLOBAL_MAX_SHIFT_RIGHT)
        swTmp = PI_GAIN_GLOBAL_MAX_SHIFT_RIGHT ;
    else if(swTmp < -PI_GAIN_GLOBAL_MAX_SHIFT_LEFT)
        swTmp = -PI_GAIN_GLOBAL_MAX_SHIFT_LEFT ;

    if (swTmp > 0)
        flTmp = flEfsPiMotorGainRatio * (FLOAT)(1L <<  swTmp) ;
    else if (swTmp < 0)
        flTmp = flEfsPiMotorGainRatio / (FLOAT)(1L << -swTmp) ;
    else
        flTmp = flEfsPiMotorGainRatio ;

    sEfsOut.swEfsPiAutoShift = swTmp ;
    sEfsOut.swEfsPiAutoKi = (SWORD)(PI_GAIN_KIREF * flTmp + 0.5) ;
    sEfsOut.swEfsPiAutoKp = (SWORD)(PI_GAIN_KPREF * flTmp + 0.5) ;

    // ********************************************************


    swTmp = psEfsPI_Param->swErrMax;
    if(swTmp < 0)
        swTmp = 0;
    psEfsPI_CheckedParam->swErrMax = swTmp;

    if (psEfsPI_Param->flags.b.bNoLimits)
        slTmp = 2147483647L ;
    else
    {   // use User Value
        atomic_read(&slTmp, &psEfsPI_Param->slOutValLimit, sizeof(slTmp));
        if(slTmp < 0l)
            slTmp = 0l;
    }
    atomic_write(&psEfsPI_CheckedParam->slOutValLimit, &slTmp, sizeof(slTmp));

    if ((psEfsPI_Param->swKp == 0) && (psEfsPI_Param->swKi == 0) && (psEfsPI_Param->swGlobalShift == 0))
    {   // use gains automatically calculated
        psEfsPI_CheckedParam->swKi = sEfsOut.swEfsPiAutoKi ;
        psEfsPI_CheckedParam->swKp = sEfsOut.swEfsPiAutoKp ;
        psEfsPI_CheckedParam->swGlobalShift = sEfsOut.swEfsPiAutoShift ;
    }
    else
    {   // use User gains
        psEfsPI_CheckedParam->swKp = psEfsPI_Param->swKp;
        psEfsPI_CheckedParam->swKi = psEfsPI_Param->swKi;
    
        swTmp = psEfsPI_Param->swGlobalShift;
        if(swTmp > PI_GAIN_GLOBAL_MAX_SHIFT_RIGHT)
            swTmp = PI_GAIN_GLOBAL_MAX_SHIFT_RIGHT ;
        else if(swTmp < -PI_GAIN_GLOBAL_MAX_SHIFT_LEFT)
            swTmp = -PI_GAIN_GLOBAL_MAX_SHIFT_LEFT ;
    
        psEfsPI_CheckedParam->swGlobalShift=swTmp;
    }
 
    if (psEfsPI_Param->flags.b.bNoLimits)
        slTmp = 2147483647L ;
    else
    {   // Note: slTmp = psEfsPI_CheckedParam->slOutValLimit 
        if(psEfsPI_CheckedParam->swGlobalShift > 0)
            slTmp <<=  psEfsPI_CheckedParam->swGlobalShift ;
        else if(psEfsPI_CheckedParam->swGlobalShift < 0)
            slTmp >>= -psEfsPI_CheckedParam->swGlobalShift ;
    }
    atomic_write(&psEfsPI_CheckedParam->slIntegralLimit, &slTmp, sizeof(slTmp));

    return TRUE;
}

//****************************************************************************
// EFS PI - Reset

static void EfsPi_Reset(EFS_PI_RUNTIME *psEfsPI_Run, EFS_PI_OUTPUT *psEfsPI_Out)
{
    psEfsPI_Run->slIntegral32 = 0L ; 
    psEfsPI_Out->swCorrection = 0 ;
    psEfsPI_Out->swErr = 0 ;  
    psEfsPI_Out->slPiOut = 0L ;
}
#endif
