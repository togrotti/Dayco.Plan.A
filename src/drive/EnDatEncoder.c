/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright $)AB) 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : EnDatEncoder.c                                             */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonDefines.h"
#include "drive\MotorParameters.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "system\SysLogManagement.h"
#include "system\SystemStatus.h"
#include "system\Os.h"

#include "common\CommonEncoderManager.h"
#include "drive\EnDatEncoder.h"
#include "drive\EnDatHandlers.h"

#include "common\TaskScheduler.h"
#include "system\SysAppGlobals.h"
#include "common\CommonUtility.h"

#include "common\Int64Functions.h"
//#include "common\DspFunctions.h" //crs: already included in CommonUtility.h
#include "fpga\FpgaHandler.h"

#define ENDAT_WAITE_TIMEOUT                           1000    // msec


/////////////////////////////////////////////////////////////////////////////
// Compiler Option
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
// General global variables

ENDAT_PARAMS sEn_Params[ENDAT_MAX_ENCODER];
ENDAT_OUT sEn_DataOut[ENDAT_MAX_ENCODER];

//****************************************************************************
// Default parameters

const ENDAT_PARAMS sEn_DefParams =
{
    {0,0}, 2000, 0xFFFFFFFFul
};

#if CFG_ENC_ENDAT
//****************************************************************************
// Defines

#define CRC_ERROR_LIMIT     0x02
#define CRC_ERROR_LIMIT_ADI 0x80 // crs-endat.22

#define ENDAT_OLD_EP_MEMAREA        0xAB
#define ENDAT_OLD_EP_MEMADDR        0x00

//****************************************************************************
// Data structures

typedef struct {
    union {
        struct {
            UWORD bAlarm        : 1 ;
            UWORD bCrc          : 1 ;
            UWORD bOverTime     : 1 ;
#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
            UWORD bCrcAdi       : 1 ; // crs-endat.22
#endif // endat22 && endat22_addinfo
        } b ;
        UWORD w ;
    } sErrorSent ;

    union {
        struct {
            UWORD bInitDone     : 1; // b.0
            UWORD bValid        : 1; // b.1
            UWORD bInitReq      : 1; // b.2
            UWORD bPlcReq       : 1; // b.3
            UWORD bPlcReqStat   : 1; // b.4
            UWORD bDisEndAlarm  : 1; // b.5
#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
            UWORD bAddInfoEnable: 1; // b.6 // crs-endat.22
#endif // endat22 && endat22_addinfo
        } b ;
        UWORD w ;
    } sCmd ;

    UWORD uwMotPolarPairs  ;

    SQWRD sqMechAbsPosition ;
    SQWRD sqMechAbsInitPos  ;
    UWORD uwAbsElecAngle;
    
    SWORD swCrcError ;
    ENDAT_OUT * psDataOut;
    ENCMGR_SPACEFEEDBACK * psFeedbackOut;

    UWORD uwChannelSel;
    UWORD uwClockFreq;

    SBYTE * psbEncMgrFault;

    ENDAT_WORKS * psWorks;
    ULONG ulMTurnStartPos;
    ULONG ulRevMask;

#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
    UWORD uwAddInfo2Use ; // crs-endat.22
    SWORD swCrcErrorAdi ; // crs-endat.22
#endif // endat22 && endat22_addinfo
} ENDAT_RUNTIME ;
  
typedef struct
{
    ULONG   ulSerialNumber;         //
    ULONG   ulProductionDate;       //
    ULONG   ulMotorModel;           //
    ULONG   ulMotorType;            //
    UWORD   uwInductance;           // mH * 10exp-1
    UWORD   uwResistance;           // Ohm * 10exp-1
    UWORD   uwTorqueConstant;       // Nm/Arms * 10exp-2
    UWORD   uwRotorLockedArms;      // Arms * 10exp-2
    UWORD   uwNominalSpeedArms;     // Arms * 10exp-2
    UWORD   uwCurrentPeakArms;      // Arms * 10exp-2
    UWORD   uwThermalConstant;      // Seconds
    UWORD   uwMomentOfInertia;      // Kg*m2 * 10exp-5
    UWORD   uwEncoderPhase;         // Electrical Degrees [0-359deg]
    UWORD   uwPoleNumbers;          //
    ULONG   ulNominalSpeed;         // cntVi / (250us) * 2exp16 <=> 1rpm = 17895du

} ENDAT_OLD_ELECTRONIC_PLATE;

//****************************************************************************
// Locals

static BOOL                     bSlowTaskInstalled=FALSE;

static volatile ENDAT_RUNTIME   sRuntime[ENDAT_MAX_ENCODER];
static ENDAT_WORKS              sWorks[ENDAT_MAX_ENCODER];

static ULONG ulBaseAddress[ENDAT_MAX_ENCODER]=
{
    FPGA_MENDAT_BASEADDR,
#ifndef _HW_DC
    FPGA_AENDAT_BASEADDR,
#endif
};

static ULONG ulAlarmSubCode[ENDAT_MAX_ENCODER]=
{
    SYSTEMALARMS_SUBCODE_ENC_MAIN,
#ifndef _HW_DC
    SYSTEMALARMS_SUBCODE_ENC_AUX,
#endif
};

//****************************************************************************
// Local functions

static BOOL coreinit(UWORD,BOOL);
static BOOL endathandlerinit(UWORD, UWORD, BOOL);
static BOOL initruntime(UWORD);
static BOOL endatinit(UWORD, BOOL);
static BOOL readelectronicplate(UWORD,MOTPRM_PARAMETERS *,SBYTE *);
static BOOL task8kHz(ENDAT_RUNTIME *);
static void slowtask(void);

//***************************************************************************
// Initialization entry point

BOOL En_Init(UWORD uwChannelSel, ENCMGR_SPACEFEEDBACK * psFeedback, SBYTE * psbEncMgrFault, ULONG ulRTDisMask)
{
        // if wrong parameters
    if(!En_ParametersCheck(uwChannelSel))
        return FALSE;

        // setup runtime
    sRuntime[uwChannelSel].psDataOut=&sEn_DataOut[uwChannelSel];
    sRuntime[uwChannelSel].psFeedbackOut=psFeedback;
    sRuntime[uwChannelSel].uwChannelSel=uwChannelSel;
    sRuntime[uwChannelSel].uwClockFreq=sEn_Params[uwChannelSel].uwClockFreq;
    sRuntime[uwChannelSel].uwMotPolarPairs = sGlbMotorParameters.uwPoleNumbers / 2 ;
    sRuntime[uwChannelSel].psbEncMgrFault = psbEncMgrFault;
    sRuntime[uwChannelSel].psWorks = &sWorks[uwChannelSel];
    sRuntime[uwChannelSel].ulMTurnStartPos=sEn_Params[uwChannelSel].ulMTurnStartPos;

        // if not yet done init encoder
    if(!sRuntime[uwChannelSel].sCmd.b.bInitDone)
    {
            // core setup
        if(!coreinit(uwChannelSel, TRUE))
            return FALSE;
    
            // Initialize EnDat Encoder
        if(!endatinit(uwChannelSel, TRUE))
        {
            coreinit(uwChannelSel, FALSE);
            return FALSE;
        }
    }
        // otherwise only init runtime
    else
        if(!initruntime(uwChannelSel))
            return FALSE;

        // if not yet, install slowtask for protocol management
    if(!bSlowTaskInstalled)
    {
        if(!TaskSched_AddBackgroundTask(&slowtask))
            return FALSE;

        bSlowTaskInstalled=TRUE;
    }

        // install rt task       
    if(!TaskSched_AddRTTaskEx((BOOL (*)(UWORD))&task8kHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|ulRTDisMask, 0, (UWORD)&sRuntime[uwChannelSel]))
        return FALSE;

        // module valid
    sRuntime[uwChannelSel].sCmd.b.bValid=TRUE;

    return TRUE;
}

//***************************************************************************
// Read electronic plate

BOOL En_GetEPlate(UWORD uwChannelSel, MOTPRM_PARAMETERS * psMotPar, SBYTE * bDataValid)
{
        // if wrong parameters
    if(!En_ParametersCheck(uwChannelSel))
        return FALSE;

        // if not yet done init encoder
    if(!sRuntime[uwChannelSel].sCmd.b.bInitDone)
    {    
            // core setup
        if(!coreinit(uwChannelSel, TRUE))
            return FALSE;
    
            // full handler init
        if(!endathandlerinit(uwChannelSel, sEn_Params[uwChannelSel].uwClockFreq, TRUE))
        {
            coreinit(uwChannelSel, FALSE);
            return FALSE;
        }
    }

        // get and decode electronic plate
    return readelectronicplate(uwChannelSel, psMotPar, bDataValid);
}

//***************************************************************************
// core options

static BOOL coreinit(UWORD uwChannelSel, BOOL bEnable)
{
    switch(uwChannelSel)
    {
        case ENDAT_SEL_MAIN:
            FPGA_ENCODER_OPTIONS_MENDEN = bEnable;
            break;

#ifndef _HW_DC
        case ENDAT_SEL_AUX:
            FPGA_ENCODER_OPTIONS_AUXENA = FALSE;
            FPGA_ENCODER_OPTIONS_AUXENB = FALSE;
            
            FPGA_ENCODER_OPTIONS_AENDEN = bEnable;
            
            FPGA_ENCODER_OPTIONS_AUXENA = bEnable;
            FPGA_ENCODER_OPTIONS_AUXENB = bEnable;
            break;
#endif // !_hw_dc

        default:
            assert(FALSE);
    }

    return TRUE;
}

//***************************************************************************
// init or re-init unit

static BOOL endathandlerinit(UWORD uwChannelSel, UWORD uwClockFreq, BOOL bFullInit)
{
    UWORD uwRetVal,uwTimeOut;
	
#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
        // setup endat additional info: mandatory to make it here
    if(sRuntime[uwChannelSel].sCmd.b.bAddInfoEnable)
    {
        uwRetVal = EndatSetRealtimeADInfo(&sWorks[uwChannelSel], (UBYTE)sRuntime[uwChannelSel].uwAddInfo2Use) ;
        if(!(BOOL)uwRetVal)
        {
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_ENDAT_FAIL, ulAlarmSubCode[uwChannelSel] | SYSTEMALARMS_SUBCODE_ENDAT_INIT_ADDINFO, FALSE);
            return FALSE ;      
        }
    }
#endif // endat22 && endat22_addinfo
        // Initialize EnDat Encoder
    uwTimeOut=timer_settimeout(uwSysTimers1ms, ENDAT_WAITE_TIMEOUT);
    for(;;)
    {
        uwRetVal=EndatInitializeEncoder(ulBaseAddress[uwChannelSel], uwClockFreq, &sWorks[uwChannelSel], bFullInit);
        if(timer_istimedout(uwSysTimers1ms, uwTimeOut)||uwRetVal==0)
        {
            break;
        }
    }
    if(uwRetVal)
    {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_ENDAT_FAIL, ulAlarmSubCode[uwChannelSel] | SYSTEMALARMS_SUBCODE_ENDAT_INIT | ((ULONG)uwRetVal<<16), FALSE);
        //SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_ENDAT_FAIL, 0, FALSE);
        return FALSE ;
    }

        // set init done if full init was done
    sRuntime[uwChannelSel].sCmd.b.bInitDone|=bFullInit;

    return TRUE;
}

//***************************************************************************
// init runtime

static BOOL initruntime(UWORD uwChannelSel)
{
        // Setup diagnostics
    sEn_DataOut[uwChannelSel].uwPropagationDelay=sWorks[uwChannelSel].uwPropagationTime;
    sEn_DataOut[uwChannelSel].uwMaxFrequency    =sWorks[uwChannelSel].uwMaxFrequency;
    sEn_DataOut[uwChannelSel].ubStepPerRevBits  =sWorks[uwChannelSel].ubStepPerRevBits;
    sEn_DataOut[uwChannelSel].ubRevNumBits      =sWorks[uwChannelSel].ubRevNumBits;

    if(sWorks[uwChannelSel].flags.b.bProt22)
        sEn_DataOut[uwChannelSel].ubProtocolVersion=22;
    else
        sEn_DataOut[uwChannelSel].ubProtocolVersion=21;

        // init everything from zero (the initial angle will be found during first servo)
    INT64_ASSIGN(sRuntime[uwChannelSel].psFeedbackOut->sEncData.sqPostn,  0L, 0UL) ;
    INT64_ASSIGN(sRuntime[uwChannelSel].psFeedbackOut->sqMechAbsPosOffset,  0L, 0UL) ;
    
    INT64_ASSIGN(sRuntime[uwChannelSel].sqMechAbsInitPos,  0L, 0UL) ;
    INT64_ASSIGN(sRuntime[uwChannelSel].sqMechAbsPosition, 0L, 0UL) ;

        // runtime
    if(sWorks[uwChannelSel].ubRevNumBits>0)
        sRuntime[uwChannelSel].ulRevMask = (1ul << sWorks[uwChannelSel].ubRevNumBits) - 1;
    else
        sRuntime[uwChannelSel].ulRevMask = 0ul;
  
        // Feedback
    sRuntime[uwChannelSel].psFeedbackOut->uwElecAngle = 0 ;    
    sRuntime[uwChannelSel].psFeedbackOut->swElecSpeed = 0 ;  
    sRuntime[uwChannelSel].psFeedbackOut->uwDeltaElecAngle = 0 ;    
    sRuntime[uwChannelSel].psFeedbackOut->sEncData.slSpeed = 0L ;
    sRuntime[uwChannelSel].psFeedbackOut->sEncData.slAccel = 0L ;
    sRuntime[uwChannelSel].psFeedbackOut->ubStatus = ENCMGR_NULL ;

#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
    // additional info mgmt
    if (!sRuntime[uwChannelSel].sCmd.b.bAddInfoEnable)
        sRuntime[uwChannelSel].uwAddInfo2Use = 0x0000 ;

    sRuntime[uwChannelSel].swCrcErrorAdi = 0;
    sEn_DataOut[uwChannelSel].ubAddInfoVar =(UBYTE)sRuntime[uwChannelSel].uwAddInfo2Use;
    sEn_DataOut[uwChannelSel].uwAddInfoVal = 0;
    sEn_DataOut[uwChannelSel].uwAddInfoCRCErrorCounter = 0;
    // crs-endat.22
#endif // endat22 && endat22_addinfo
    return TRUE;
}

//***************************************************************************
// init or re-init unit

static BOOL endatinit(UWORD uwChannelSel, BOOL bFullInit)
{
        // endat handler init
    if(!endathandlerinit(uwChannelSel, sRuntime[uwChannelSel].uwClockFreq, bFullInit))
    {
        sRuntime[uwChannelSel].psFeedbackOut->ubStatus = ENCMGR_FATAL_FAULT ; 
        return FALSE;
    }

    return initruntime(uwChannelSel);
}

//***************************************************************************
// read electronic plate

static BOOL readelectronicplate(UWORD uwChannelSel, MOTPRM_PARAMETERS * psMotPar, SBYTE * bDataValid)
{
    ENDAT_OLD_ELECTRONIC_PLATE sOldPlate;
    UBYTE i;
    ULONG ulRetval;
    UWORD * puwPtr=(UWORD *)&sOldPlate;

        // by default data invalid
    *bDataValid=FALSE;

        // enter command mode
    EndatEnterCommandMode(&sWorks[uwChannelSel]);

        // read data structure
    for(i=0;i<sizeof(ENDAT_OLD_ELECTRONIC_PLATE)/sizeof(UWORD);i++)
    {
        ulRetval=EndatReadParameter(&sWorks[uwChannelSel], ENDAT_OLD_EP_MEMAREA, ENDAT_OLD_EP_MEMADDR+i);

            // if error
        if(ulRetval&ENDAT_FUNCTION_ERROR_MASK_ANY)
            break;

        *puwPtr++=(UWORD)(ulRetval&0xffff);
    }

        // back to position mode
    EndatEnterPositionMode(&sWorks[uwChannelSel]);

        // if error
    if(ulRetval&ENDAT_FUNCTION_ERROR_MASK_ANY)
        return TRUE;

        // decode values
    psMotPar->ulSerialNumber=sOldPlate.ulSerialNumber;
    psMotPar->ulProductionDate=sOldPlate.ulProductionDate;
    psMotPar->ulModel=sOldPlate.ulMotorModel;
    psMotPar->flResistance=(FLOAT)sOldPlate.uwResistance/10.0;
    psMotPar->flInductance=(FLOAT)sOldPlate.uwInductance/10000.0;
    psMotPar->flKT=(FLOAT)sOldPlate.uwTorqueConstant/100.0;
    psMotPar->flCurrentNominalZeroSpeed=(FLOAT)sOldPlate.uwRotorLockedArms/100.0;
    psMotPar->flCurrentNominal=(FLOAT)sOldPlate.uwNominalSpeedArms/100.0;
    psMotPar->flCurrentPeak=(FLOAT)sOldPlate.uwCurrentPeakArms/100.0;
    psMotPar->flSpeedNominal=(FLOAT)sOldPlate.ulNominalSpeed/17895/60.0*2.0*FLOAT_PI;
    psMotPar->uwThermalConstant=sOldPlate.uwThermalConstant;
    psMotPar->flMotorInertia=(FLOAT)sOldPlate.uwMomentOfInertia/100000.0;
    psMotPar->uwPhaseOffset=(UWORD)((FLOAT)sOldPlate.uwEncoderPhase/360.0*65535.0);
    psMotPar->ulType=sOldPlate.ulMotorType;
    psMotPar->uwPoleNumbers=sOldPlate.uwPoleNumbers;
    psMotPar->flDirectInductance=0.0;

        // if s/n not zero, then data is valid
    if(psMotPar->ulSerialNumber)
        *bDataValid=TRUE;

    return TRUE;
}      

//***************************************************************************
// Parameters check

BOOL En_ParametersCheck(UWORD uwChannelSel)
{
    assert(uwChannelSel<ENDAT_MAX_ENCODER);

    if(!EndatCheckParameters(sEn_Params[uwChannelSel].uwClockFreq))
    {
        ParChk_SignalValueError(uwChannelSel==ENDAT_SEL_MAIN?PARCC_ENDAT_MAINFREQ:PARCC_ENDAT_AUXFREQ);
        return FALSE;
    }
    else
    {
        ParChk_ResetValueError(uwChannelSel==ENDAT_SEL_MAIN?PARCC_ENDAT_MAINFREQ:PARCC_ENDAT_AUXFREQ);
        return TRUE;
    }
}

//***************************************************************************
// Electrical Angle Setup (only for main encoder)

void En_SetElecAngle(UWORD uwElecAngle)
{
    sRuntime[ENDAT_SEL_MAIN].psFeedbackOut->uwDeltaElecAngle = uwElecAngle - sRuntime[ENDAT_SEL_MAIN].uwAbsElecAngle - sGlbMotorParameters.uwPhaseOffset;
    sRuntime[ENDAT_SEL_MAIN].psFeedbackOut->ubStatus        |= ENCMGR_ELE_ANGLE_VALID ;
}

//***************************************************************************
// fast task

static BOOL task8kHz(ENDAT_RUNTIME * psRuntime)
{
  BOOL bInterpolate;
  ULONG ulMechAngle=0l, ulMechAngle_1=0l ;
  SLONG slMechSpeed_1 ;
  ENCMGR_SPACEFEEDBACK * psFeedback;
  ULONG ulBaseAd;
  UWORD uwStat;

  psFeedback=psRuntime->psFeedbackOut;
  ulBaseAd=ulBaseAddress[psRuntime->uwChannelSel];

  if(bSysStatAlarmsReset)
  {
        // if was faulty, then reset status
      if(psRuntime->sErrorSent.w || *psRuntime->psbEncMgrFault)
        psRuntime->sCmd.b.bInitReq = TRUE;

      psRuntime->sErrorSent.w = 0 ; /* clear errors */
  }

  // if init in progress and/or parametrization in progress, then do nothing and nullify data
  if(psRuntime->sCmd.b.bInitReq || psRuntime->sCmd.b.bPlcReqStat)
  {
    psFeedback->ubStatus = ENCMGR_NULL;
    return TRUE;
  }

  // if faulty then exit
  if(psFeedback->ubStatus&ENCMGR_FATAL_FAULT)
    return TRUE;

  // get status, if busy skip to overtime alarm,
  // if not then readback data, then again status
  uwStat = FPGA_BASEOFF_16(ulBaseAd, FPGA_ENDAT_STATUS);
  if(!FPGA_ENDAT_STAT_BUSY(uwStat))
  {
    ulMechAngle   = FPGA_BASEOFF_32(ulBaseAd, FPGA_ENDAT_DATAOUT_LLSW) ;
    ulMechAngle_1 = FPGA_BASEOFF_32(ulBaseAd, FPGA_ENDAT_DATAOUT_HLSW) ;
    uwStat = FPGA_BASEOFF_16(ulBaseAd, FPGA_ENDAT_STATUS);
  }

  /* check for valid data transfer, enter here also if alarm is just triggered 
     to avoid triggering of other alarms */
  if(!FPGA_ENDAT_STAT_VALID(uwStat) || FPGA_ENDAT_STAT_OVERTIME(uwStat) || FPGA_ENDAT_STAT_BUSY(uwStat) || psRuntime->sErrorSent.b.bOverTime)
  {
    if(!psRuntime->sErrorSent.b.bOverTime)
    {
      SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_ENDAT_FAIL, ulAlarmSubCode[psRuntime->uwChannelSel] | SYSTEMALARMS_SUBCODE_ENDAT_OVERTIME, bSysStatPowerEnabled) ;
        
      psFeedback->ubStatus  = ENCMGR_FATAL_FAULT ;             
      psRuntime->sErrorSent.b.bOverTime = TRUE ;
    }   
    return TRUE;
  }

  /* check for CRC error */
  if(FPGA_ENDAT_STAT_CRCERR(uwStat))
  {
    if(psRuntime->swCrcError>=CRC_ERROR_LIMIT)
    {
      if(!psRuntime->sErrorSent.b.bCrc)
      {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_ENDAT_FAIL, ulAlarmSubCode[psRuntime->uwChannelSel] | SYSTEMALARMS_SUBCODE_ENDAT_CRC, bSysStatPowerEnabled) ;

        psFeedback->ubStatus  = ENCMGR_FATAL_FAULT ;             
        psRuntime->sErrorSent.b.bCrc = TRUE ;
      }
      return TRUE;
    }
    psRuntime->swCrcError++;

    if(psRuntime->psDataOut->uwCRCErrorCounter<65535)
        psRuntime->psDataOut->uwCRCErrorCounter++;

    bInterpolate=TRUE;
  }
  else
  {
    if(psRuntime->swCrcError>0)
        psRuntime->swCrcError-- ;
    bInterpolate=FALSE;

    /* if CRC is ok then check for alarm bit */
    if(FPGA_ENDAT_STAT_ALARM(uwStat) && !psRuntime->sCmd.b.bDisEndAlarm)
    {
      if(!psRuntime->sErrorSent.b.bAlarm)
      {
        SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_ENDAT_FAIL, ulAlarmSubCode[psRuntime->uwChannelSel] | SYSTEMALARMS_SUBCODE_ENDAT_ALARM, bSysStatPowerEnabled) ;
    
        psFeedback->ubStatus  = ENCMGR_FATAL_FAULT ;             
        psRuntime->sErrorSent.b.bAlarm = TRUE ;
      }
      return TRUE;
    }
   }

#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
  /* ============================================ *
   * ==== Additional Info runtime management ==== *
   * ============================================ */
  if(psRuntime->sCmd.b.bAddInfoEnable)
  { // endat additional info mgmt

    // always get the Additional Info variable read
    psRuntime->psDataOut->ubAddInfoVar = (UBYTE)FPGA_BASEOFF_16(ulBaseAd, FPGA_ENDAT_CMDADR) ;
    
    if(FPGA_ENDAT_STAT_ADICRCERR(uwStat))
    {
        if(psRuntime->swCrcErrorAdi >= CRC_ERROR_LIMIT_ADI)
        {
          if(!psRuntime->sErrorSent.b.bCrcAdi)
          {
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_ENDAT_FAIL, ulAlarmSubCode[psRuntime->uwChannelSel] | SYSTEMALARMS_SUBCODE_ENDAT_CRC_ADDINFO, bSysStatPowerEnabled) ;
    
            psFeedback->ubStatus  = ENCMGR_FATAL_FAULT ;             
            psRuntime->sErrorSent.b.bCrcAdi = TRUE ;
          }
          return TRUE;
        }
        psRuntime->swCrcErrorAdi++;
    
        if(psRuntime->psDataOut->uwAddInfoCRCErrorCounter < 65535)
            psRuntime->psDataOut->uwAddInfoCRCErrorCounter++;
    }
    else
    {   // get the additional info data 
        if(psRuntime->swCrcErrorAdi>0)
            psRuntime->swCrcErrorAdi-- ; 
    
        psRuntime->psDataOut->uwAddInfoVal = (UWORD)FPGA_BASEOFF_16(ulBaseAd, FPGA_ENDAT_PARAM) ;   
    }
  }
  else
  {
    psRuntime->psDataOut->ubAddInfoVar = 0 ;
    psRuntime->psDataOut->uwAddInfoVal = 0 ;
  }
#endif // endat22 && endat22_addinfo

  /* ============================================ *
   * ==== Additional Info runtime management ==== *
   * ============================================ */

  /* if first run get abs pos */
  if(psFeedback->ubStatus == ENCMGR_NULL)
  {
    // if interpolation requested then exit
    if(bInterpolate)
      return TRUE;
    
    psRuntime->sqMechAbsInitPos.lo  = ulMechAngle ;
    psRuntime->sqMechAbsInitPos.hi  = ulMechAngle_1 ;

    // check startup position for multiturns encoders
    if(psRuntime->ulRevMask)
            // if startup turns pos is above threshold, then extend sign
        if((ULONG)psRuntime->sqMechAbsInitPos.hi >= psRuntime->ulMTurnStartPos)
            psRuntime->sqMechAbsInitPos.hi=~(psRuntime->ulRevMask)| \
                                            (psRuntime->sqMechAbsInitPos.hi & psRuntime->ulRevMask);

    psRuntime->sqMechAbsPosition.lo = psRuntime->sqMechAbsInitPos.lo ;
    psRuntime->sqMechAbsPosition.hi = psRuntime->sqMechAbsInitPos.hi ;
    
    INT64_COPY(psFeedback->sqMechAbsPosOffset, psRuntime->sqMechAbsInitPos) ;
        
    psFeedback->ubStatus = ENCMGR_ABSMECHTURN_VALID | ENCMGR_ELE_ANGLE_VALID | ENCMGR_RELATIVE_VALID | ENCMGR_EFS_READY;
  }

  /* Previous absolute angle and mechanical speed */
  ulMechAngle_1 = psRuntime->sqMechAbsPosition.lo ;
  slMechSpeed_1 = psFeedback->sEncData.slSpeed ;

  /* if requested then interpolate, if not position readback is already read */
  if(bInterpolate)
    ulMechAngle = ulMechAngle_1 + slMechSpeed_1 ;

  /* ============================================ * 
   * =========== Module Output Values =========== * 
   * ============================================ */ 
  /* ---- Electrical Angle (MechAngle2Use = absolute angle from EnDat) ---- */
  psRuntime->uwAbsElecAngle = (UWORD)((ULONG)HIWORD(ulMechAngle) * psRuntime->uwMotPolarPairs); 
  psFeedback->uwElecAngle   = psRuntime->uwAbsElecAngle + psFeedback->uwDeltaElecAngle + sGlbMotorParameters.uwPhaseOffset ;  

  /* -- Current Abs Mechanical Position -- */
  if ( (ulMechAngle_1 & 0x80000000) &&  (ulMechAngle_1 & 0x40000000) &&
      !(ulMechAngle   & 0x80000000) && !(ulMechAngle   & 0x40000000)   ) 
  { /* Abs Mechanical Angle : Clockwise Overflow */
   psRuntime->sqMechAbsPosition.hi++ ; 
  } 
  else if (!(ulMechAngle_1 & 0x80000000) && !(ulMechAngle_1 & 0x40000000) &&
            (ulMechAngle   & 0x80000000) &&  (ulMechAngle   & 0x40000000)   ) 
  { /* Abs Mechanical Angle : Anticlockwise Overflow */
    psRuntime->sqMechAbsPosition.hi-- ; 
  } 
  psRuntime->sqMechAbsPosition.lo = ulMechAngle ;

  /* -- Current Relative Mechanical Position = AbsMechPosition(64) - AbsMechInitPosition(64) -- */
  _sint64_sub(&psFeedback->sEncData.sqPostn, &psRuntime->sqMechAbsPosition, &psRuntime->sqMechAbsInitPos) ;

  /* -- Current Mechanical Speed = Current MechAngle - Previous MechAngle (= delta angle) -- */
  psFeedback->sEncData.slSpeed = (SLONG)(ulMechAngle - ulMechAngle_1) ;
  
  /* -- Current Mechanical Acceleration = Current MechSpeed - Previous MechSpeed (= delta speed) -- */
  psFeedback->sEncData.slAccel = _sint32_asl12(psFeedback->sEncData.slSpeed - slMechSpeed_1) ;

  /* -- Current Electrical Speed  = MechSpeed * MotorPolePairs */
  psFeedback->swElecSpeed = (SWORD)_sint32_scale_32(psFeedback->sEncData.slSpeed, psRuntime->uwMotPolarPairs);

  return TRUE ;
}

//***************************************************************************
// slow task

static void slowtask(void)
{
    UWORD uwCt;

    for(uwCt=0;uwCt<ENDAT_MAX_ENCODER;uwCt++)
        if(sRuntime[uwCt].sCmd.b.bValid)
        {
                // if plc request
            if(sRuntime[uwCt].sCmd.b.bPlcReq != sRuntime[uwCt].sCmd.b.bPlcReqStat)
            {
                if(sRuntime[uwCt].sCmd.b.bPlcReqStat)
                {
                    EndatEnterPositionMode(&sWorks[uwCt]);
                        // wait some rt cycles to allow communication restart and
                        // alarms clean-up before enabling rt error handling
                    timer_wait(uwSysTimers125us,4);
                    sRuntime[uwCt].sCmd.b.bPlcReqStat=FALSE;
                }
                else
                {
                    sRuntime[uwCt].sCmd.b.bPlcReqStat=TRUE;
                    EndatEnterCommandMode(&sWorks[uwCt]);
                }
            }
                // check for re-init request
            if(sRuntime[uwCt].sCmd.b.bInitReq && !sRuntime[uwCt].sCmd.b.bPlcReqStat)
            {
                
                endatinit(uwCt, FALSE);
                    // yield other tasks in order to let module get fresh data
                // Os_Sleep(0);
                sRuntime[uwCt].sCmd.b.bInitReq=FALSE;
            }

                // if change disable endat alarm
            if(sRuntime[uwCt].sCmd.b.bDisEndAlarm != sEn_Params[uwCt].flags.b.bDisEndAlarm)
                sRuntime[uwCt].sCmd.b.bDisEndAlarm=sEn_Params[uwCt].flags.b.bDisEndAlarm;
        }
}

//***************************************************************************
// PLC request for command mode or reset to position mode

ULONG PlcEndatSetCommandMode(UWORD uwChannelSel, BOOL bSet)
{
        // check right drive state
    if(bSysStatBooting || bSysStatResetting || bSysStatPowerEnabled
//    		|| !Os_IsInBackground()
			)
        return ENDAT_FUNCTION_ERROR_NOMODULE;

        // check if selected is valid
    if(uwChannelSel>=ENDAT_MAX_ENCODER)
        return ENDAT_FUNCTION_ERROR_NOMODULE;

    if(!sRuntime[uwChannelSel].sCmd.b.bValid)
        return ENDAT_FUNCTION_ERROR_NOMODULE;

        // send request
    sRuntime[uwChannelSel].sCmd.b.bPlcReq=bSet;

        // wait until request ack
    while(sRuntime[uwChannelSel].sCmd.b.bPlcReqStat!=bSet)
        // Os_Sleep(0);
        ;

    return 0l;
}

//***************************************************************************
// PLC read parameter

ULONG PlcEndatReadParameter(UWORD uwChannelSel, UBYTE ubMemArea, UBYTE ubAddress)
{
        // check right drive state
    // if(!Os_IsInBackground())
    //     return ENDAT_FUNCTION_ERROR_NOMODULE;

        // check if selected is valid
    if(uwChannelSel>=ENDAT_MAX_ENCODER)
        return ENDAT_FUNCTION_ERROR_NOMODULE;

    if(!sRuntime[uwChannelSel].sCmd.b.bValid || !sRuntime[uwChannelSel].sCmd.b.bPlcReqStat)
        return ENDAT_FUNCTION_ERROR_NOMODULE;

    return EndatReadParameter(&sWorks[uwChannelSel], ubMemArea, ubAddress);
}
    
//***************************************************************************
// PLC write parameter

ULONG PlcEndatWriteParameter(UWORD uwChannelSel, UBYTE ubMemArea, UBYTE ubAddress, UWORD uwParam)
{
        // check right drive state
    // if(!Os_IsInBackground())
    //     return ENDAT_FUNCTION_ERROR_NOMODULE;

        // check if selected is valid
    if(uwChannelSel>=ENDAT_MAX_ENCODER)
        return ENDAT_FUNCTION_ERROR_NOMODULE;

    if(!sRuntime[uwChannelSel].sCmd.b.bValid || !sRuntime[uwChannelSel].sCmd.b.bPlcReqStat)
        return ENDAT_FUNCTION_ERROR_NOMODULE;

    return EndatWriteParameter(&sWorks[uwChannelSel], ubMemArea, ubAddress, uwParam);
}

//***************************************************************************
// PLC reset encoder

ULONG PlcEndatResetEncoder(UWORD uwChannelSel)
{
        // check right drive state
    // if(!Os_IsInBackground())
    //     return ENDAT_FUNCTION_ERROR_NOMODULE;

        // check if selected is valid
    if(uwChannelSel>=ENDAT_MAX_ENCODER)
        return ENDAT_FUNCTION_ERROR_NOMODULE;

    if(!sRuntime[uwChannelSel].sCmd.b.bValid || !sRuntime[uwChannelSel].sCmd.b.bPlcReqStat)
        return ENDAT_FUNCTION_ERROR_NOMODULE;

    return EndatResetEncoder(&sWorks[uwChannelSel]);
}

//***************************************************************************
// PLC reset active alarms 

ULONG PlcEndatResetActiveAlarms(UWORD uwChannelSel)
{
        // check right drive state
    // if(!Os_IsInBackground())
    //     return ENDAT_FUNCTION_ERROR_NOMODULE;

        // check if selected is valid
    if(uwChannelSel>=ENDAT_MAX_ENCODER)
        return ENDAT_FUNCTION_ERROR_NOMODULE;

    if(!sRuntime[uwChannelSel].sCmd.b.bValid || !sRuntime[uwChannelSel].sCmd.b.bPlcReqStat)
        return ENDAT_FUNCTION_ERROR_NOMODULE;

    return EndatResetActiveAlarms(&sWorks[uwChannelSel]);
}

//***************************************************************************
#if (defined(ENDAT22) && defined(ENDAT22_ADDINFO))
ULONG PlcEndatAddInfoSetup(UWORD uwChannelSel, UWORD uwAddInfoCode)
{
    if (uwChannelSel == ENDAT_SEL_MAIN)
    {   // only main endat 
        if(uwAddInfoCode > 0x0F)
        {   // additional info not supported
            sRuntime[uwChannelSel].sCmd.b.bAddInfoEnable = FALSE ;
            sRuntime[uwChannelSel].uwAddInfo2Use = 0x00 ;
            return ENDAT_FUNCTION_ERROR_NOMODULE ; 
        }
        else
        {   
            sRuntime[uwChannelSel].sCmd.b.bAddInfoEnable = TRUE ;
            sRuntime[uwChannelSel].uwAddInfo2Use = uwAddInfoCode ; 
            return 0 ;
        }
    }
    else
    {
        sRuntime[uwChannelSel].sCmd.b.bAddInfoEnable = FALSE ;
        sRuntime[uwChannelSel].uwAddInfo2Use = 0x00 ;
        return ENDAT_FUNCTION_ERROR_NOMODULE ; 
    }
}
#endif // endat22 && endat22_addinfo
#endif // cfg_enc_endat
