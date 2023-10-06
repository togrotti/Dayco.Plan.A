/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : MotorHandler.c                                             */
/* Author      : Cristiano Tognetti                                         */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
#include <stdlib.h> // to use labs()
#include <math.h>

#include "system\SysAppGlobals.h"
#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "common\UnitMeasureConversion.h"
#include "drive\AxM-E-Defines.h"
#include "drive\HardwareUnitID.h"
#include "system\SysAppFpgaOptCodes.h"

#include "drive\HardwareParameters.h"
#include "drive\MotorHandler.h"
#include "drive\MotorHandlerRT.h"
#include "common\TaskScheduler.h"
#include "system\SysLogManagement.h"
#include "common\MathFunctions.h"
#include "common\Int64Functions.h"
#include "fpga\FpgaHandler.h"
#include "fpga\FpgaIRegs.h"
#include "fpga\AnProcessor.h"

#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"
#include "system\SysAppDataCodes.h"

#include "drive\DriveTaskController.h"
#include "drive\Deflux.h"

#include "plc\Plc.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#if defined(_CRS_DBG)
#if CRS_DBGDSK
#pragma GCC optimize (0) // crs_dbg
#else
#pragma GCC optimize (2)
#endif
#else
#pragma GCC optimize (2)
#endif
//***************************************************************************
// Avoids warning C47: unreferenced parameter

//#pragma warning disable = 47


/* ================================ #define ================================ */
/* 32767 : 1 = x : 0.866     *
 * x = 32767 * 0.866 = 28377 */
/* Nota: RADICE_TRE_MEZZI in realta' e' RADICE_TRE_MEZZI/2 */
const SWORD RADICE_TRE_MEZZI  = 28377 ;
const SWORD RADICE_TRE_QUARTI = 28377 ;

#define LIST_SIZE  8

#define R_CONV_FACTOR 1000.0   /* range: 0.001   -> 32.767  Ohm; resolution: 1  mOhm */
#define L_CONV_FACTOR 100000.0 /* range: 0.00001 -> 0.32767 H  ; resolution: 10 uH   */

// conversion factor to automatically calculate FPGA Ki and Kp (from L motor) @ 16kHz switching frequency
#define FPGA_KI_CONV_FACTOR     (2.85714285714286 * 1000.0) // x 1000 perche' lavoro in H
#define FPGA_KP_CONV_FACTOR     (2857.14285714286 * 1000.0)
#define FPGA_GAINS_VOLTAGE_NORMALIZATION (0.38) // normalized on this conversion ratio

#define CURRENT_CALIBR_NSAMPLES 512 // RT cycle
#define CURRENT_CALIBR_1ST_DEL  100 // [msec]

#define DEFAULT_BRAKELOW        720 // [V]
#define DEFAULT_BRAKEHIGH       750 // [V]

#ifdef _INFINEON_
#define SAFETORQUEOFF_IN_L      P9_IN_P5
#define SAFETORQUEOFF_IN_H      P9_IN_P7
#else
#define SAFETORQUEOFF_IN_L      GPIO_IN(STO_LO_PIN)
#define SAFETORQUEOFF_IN_H      GPIO_IN(STO_HI_PIN)
#endif
#define SAFETORQUEOFF_REST_TOUT 500 // [msec]

#define CRC_SEED                0xdead

#define RT_TASK_PERIOD          (1.0/(FLOAT)REALTIME_TASK_FREQ)
#define FILTER_QTLIMIT          32

#define PHASEMONITOR_MINRATIO   0.01    // 1%
#define PHASEMONITOR_MINVDCBUS  20      // [V]

#define ACLINEMONITOR_INTERVAL  100     // [msec]

#define PWM_BRIDGELAYOUT(s)     ((UWORD)((s).d.ubLayoutU)|(UWORD)((s).d.ubLayoutV<<2)|(UWORD)((s).d.ubLayoutW<<4))

#define OFFSET_DELTA(d,m)       (UWORD)((SWORD)((FLOAT)(d)*FPGA_ADCDEF_PWI_II_MUL/(FLOAT)(m)))
#define CURR_SCALING(n,a)       ((SWORD)((SLONG)(n)*(SLONG)(a)/1000l))

#define SETMINMAX_RUNMODE       (TRUE)
#define SETMINMAX_CALIBRATION   (FALSE)

//***************************************************************************

#define PDTIMEBASE              6.25    // [nsec]
#define PDMAXTICKS              127
#define PDTIMEOFFSET_CURR       50      // [nsec]
#define PDTIMEOFFSET_VOLT       31.25   // [nsec]

#define PDDEFAULT_CURR          0       // [nsec]
#define PDDEFAULT_VOLT          0       // [nsec]
#define PDDEF_PWB_VOLT          69      // [nsec]

//***************************************************************************

#define POST_TIMEOUT_DISCHARGE      40     // rttask ticks (5 ms)
#define POST_TIMEOUT_DEF_CHARGE     2400   // rttask ticks (300 ms)
#define POST_TIMEOUT_BRAKE_DISCHRG  160    // rttask ticks (20 ms)

#define POST_OVERVOLTAGE            200    // [1/10 V]
#define POST_IGBTSHORTVOLTAGE       80     // [1/10 V]
#define POST_BRAKEONVOLTAGE         80     // [1/10 V]
#define POST_UNDERVOLTAGE           50     // [1/10 V]
#define POST_DCBUSSHORTED           10     // [1/10 V]

#define POST_FAIL_NONE              0
#define POST_FAIL_LOIGBTBROKEN      1
#define POST_FAIL_HIIGBTBROKEN      2
#define POST_FAIL_PHASEDISCONNECT   3
#define POST_FAIL_BRAKEUNIT         4
#define POST_FAIL_BRIDGEDESAT       5
#define POST_FAIL_BRAKEDESAT        6
#define POST_FAIL_OVERCURRENT       7
#define POST_FAIL_GENFAULT          8
#define POST_FAIL_DCBUSSHORTCIRCUIT 9
#define POST_FAIL_LOWVDC            10
#define POST_FAIL_BROKENIGBT        11
#define POST_FAIL_MISSINGSTO        12
#define POST_FAIL_ABORTED           16


/*  Tabella di verita' IGBT
    bit 0   Uhi->Vlo
    bit 1   Uhi->Wlo
    bit 2   Vhi->Wlo
    bit 3   Vhi->Ulo
    bit 4   Whi->Ulo
    bit 5   Whi->Vlo
    - maschera per tutti gli igbt = 0x003F
    - maschera per Uhi = 0x0003
    - maschera per Vhi = 0x00C0
    - maschera per Whi = 0x0030
    - maschera per Ulo = 0x0018
    - maschera per Vlo = 0x0021
    - maschera per Wlo = 0x0006
*/   
#define POST_ALLIGBT_STS    0x003F
#define POST_ALLIGBT_OK     POST_ALLIGBT_STS
#define POST_ALLIGBT_NOT_OK 0x0000
#define POST_IGBT_NOT_OK    POST_ALLIGBT_NOT_OK
#define POST_UHI_STS        0x0003
#define POST_VHI_STS        0x00C0
#define POST_WHI_STS        0x0030
#define POST_ULO_STS        0x0018
#define POST_VLO_STS        0x0021
#define POST_WLO_STS        0x0006   


typedef struct {
    union {
        struct {
            UWORD bExecute     : 1 ;
            UWORD bUhi2VLoOK   : 1 ;
            UWORD bUhi2WLoOK   : 1 ;
            UWORD bVhi2WLoOK   : 1 ;
            UWORD bVhi2ULoOK   : 1 ;
            UWORD bWhi2ULoOK   : 1 ;
            UWORD bWhi2VLoOK   : 1 ;
        } b ;
        UWORD w ;
    } sFlags ;

    UWORD uwTimer ;
    UWORD uwWDTTimer ;
    UWORD uwStatus ;
    UWORD uwPrevStatus ;
    UWORD uwFailedTest ;
    UWORD uwTimeoutCharge ;
    UWORD uwIGBTFailed ;
} POST_RUNTIME ;

//***************************************************************************
// Global

MH_MOTORDATA_IN         sMh_MotorDataIn ;
MH_MOTORDATA_OUT        sMh_MotorDataOut ;
MH_MOTORDATA_PARAM      sMh_MotorDataParam ;
MH_PLC_WORKS            sMh_PlcWorks={{SLONG_MAX_VALUE,SLONG_MIN_VALUE},{SLONG_MAX_VALUE,SLONG_MIN_VALUE}};
MH_PLCADVANCED_WORKS    sMh_PlcAdvancedWorks ;
MH_PLC_UMRATIOS         sMh_PlcUMRatios ;
#ifndef _HW_DC
SWORD                   sMh_CurrScaleAdj[3]={1000,1000,1000};
#endif

// ----- Input from User (torque mode only)
GLB_IREF                sMh_MotorDataUsrInIRef ;

// ----- Input from User (all mode)
GLB_TORQUE_LIMIT        sMh_MotorDataUsrSetIqLimit={SLONG_MAX_VALUE,SLONG_MIN_VALUE};
GLB_TORQUE_LIMIT        sMh_MotorDataUsrSetIdLimit={SLONG_MAX_VALUE,SLONG_MIN_VALUE};

// ----- Input from Plc
MH_POWERSTAGECONTROL    sMh_MotorDataPlcPStageCtrl ;
GLB_IREF                sMh_MotorDataPlcInIRef ;
UWORD                   uwMh_MotoDataPlcInElecAngle ;
SWORD                   swMh_MotoDataPlcInElecSpeed ;
SLONG                   slMh_MotoDataPlcInFilteredSpeed ;

// ------ Limit ------
GLB_TORQUE_LIMIT  *puwIqLimitList[LIST_SIZE] ;
GLB_TORQUE_LIMIT  *puwIdLimitList[LIST_SIZE] ;

//***************************************************************************

#define LOCDSPDBSCALE_CUSTOM    0
#define LOCDSPDBSCALE_ARMS      1
#define LOCDSPDBSCALE_APK       2
#define LOCDSPDBSCALE_VPK       3

typedef struct
{
    UBYTE ubSize;
    UBYTE ubTypeScale;
    FLOAT fScale;
    UBYTE ubIntIECDest;
    UBYTE ubSrcTag;
} LOCDSPPARDB;

//***************************************************************************
// default parameters

const MH_MOTORDATA_PARAM  sMh_MotorDataDefParam =
{
  0,    // VBrakeLow  = 0V
  0,    // VBrakeHigh = 0V
  0,    // UnderVoltageThreshold = 0V
  0,    // User OverVoltageThreshold = 0V
  0,    // User OverCurrentThreshold = 0A   
  0,    // ModKi
  0,    // ModKp
  MH_USRBRIDGELAYOUT_UVW, // Bridge layout UVW
  70,   // Vac max distortion allowed
  {SLONG_MAX_VALUE,SLONG_MIN_VALUE},// IqLimit
  {SLONG_MAX_VALUE,SLONG_MIN_VALUE},// IdLimit
  {0,0,0,0,0,0},
  {
    {MH_FILTER_NONE,0.0,0.0,0.0,0.0},
    {MH_FILTER_NONE,0.0,0.0,0.0,0.0},
    {MH_FILTER_NONE,0.0,0.0,0.0,0.0},
    {MH_FILTER_NONE,0.0,0.0,0.0,0.0},
  },
} ;
 
//***************************************************************************
// Externals

extern const unsigned char  ILoopStandard[];
extern const unsigned long  ILoopStandard_length;
extern const unsigned char  ILoopAdvanced[];
extern const unsigned long  ILoopAdvanced_length;

#ifdef _HW_DC
extern const unsigned char  ILoopParBr1[];
extern const unsigned long  ILoopParBr1_length;
extern const unsigned char  ILoopParBr1Adv[];
extern const unsigned long  ILoopParBr1Adv_length;
extern const unsigned char  ILoopParBr2[];
extern const unsigned long  ILoopParBr2_length;

#endif // _hw_dc

//***************************************************************************
// Locals

static BOOL  bBrakeForcedOn=FALSE;
static BOOL  bPOSTInstalled=FALSE;
static volatile POST_RUNTIME * psPOSTRun=NULL;

static UBYTE  *  hpubDSPCustomAddr=NULL;
static UWORD  uwDSPCustomSize=0;

static LOCDSPPARDB *  hpubDSPCustomDBAddr=NULL;
static UWORD  uwDSPCustomDBLength=0;

static BOOL bDSPAdvance;

//***************************************************************************
// AC in layout

static const HWPRM2_VOLTAGE_LAYOUT  sDefVoltCfg={{1,1,2,0},0,0};
static const HWPRM2_IN_AC_LAYOUT  sDefInBrdgCfg={1,2,3};

#define ACIN_DEST(x)    ((x)==1?(&sMh_MotorDataOut.swACVrs):((x)==2?(&sMh_MotorDataOut.swACVst):(&sMh_MotorDataOut.swACVtr)))

//***************************************************************************
// Local prototypes

static void currentbridgelayout(HWPRMS_BRIDGE_AND_CURRENT_LAYOUT  *, UWORD, MHRT_CFGLAYOUT *, MHRT_CFGLAYOUT *);
static BOOL voltagelayout(HWPRM2_VOLTAGE_LAYOUT  *, HWPRM2_IN_AC_LAYOUT  *, UWORD *, SWORD *);

static BOOL Mh_MotorDataFromFpgaInit(void);
static BOOL Mh_MotorDataToFpgaInit(void);

static void MotorDataBkGd(void) ;
static void DefluxWrapper(void) ;
static void NormalizeTorqueLoopGains(FLOAT flUserFpgaKp, FLOAT flUserFpgaKi, MH_DSPPARS * psDest) ;
static void ComputeISenseOffsets(void);
static void SetMinMaxChannels(BOOL bRunMode, UWORD uwChannel);
static void SetPDDelays(SWORD swPDCurr, SWORD swPDVolt);
static BOOL ResetOnlyParametersCheck(void);
static BOOL LiveParametersCheck(void);
static BOOL SetupILoopAutoGains(BOOL bFirstRun, UBYTE ubFrequencyRequest);
static void CurrentSetOffsets(SLONG slIuAdj, SLONG slIvAdj, SLONG slIwAdj, HWPRMS_AD_SETTINGS * sCal, UBYTE * sTags, MH_BRIDGEOFFSETS * sOutDiag);
static BOOL CalcFilterConstants(UBYTE ubType, FLOAT flFreqMain, FLOAT flDampMain, FLOAT flFreqSec, FLOAT flDampSec, HPSWORD hpswDstV);
static void manageACline(void);

static BOOL InitLimitList(void);
static void ScanAllLimitList(void);
static void FindLimitList(GLB_TORQUE_LIMIT ** plist, GLB_TORQUE_LIMIT * dataout);
static BOOL Add2LimitList(UWORD uwSel, GLB_TORQUE_LIMIT *sStruct2Add);
static BOOL DeleteLimitFromList(UWORD uwSel, GLB_TORQUE_LIMIT *sStruct2Delete);

static BOOL Mh_POSTInit(void);
static BOOL Mh_POST8KHz(void);
static void SelfTestPwmBridgeLayout(BOOL bULo, BOOL bUHi, BOOL bVLo, BOOL bVHi, BOOL bWLo, BOOL bWHi);

static BOOL DSPHLoad(HPUBYTE hpubBin, UWORD uwSize, UWORD uwStartTag);

//***************************************************************************
// Global handlers

const MH_HANDLERS sMh_MotorHandlers=
{
  &Add2LimitList,
  &DeleteLimitFromList,
  &ScanAllLimitList,
  &Mh_ForceCommandEx,
};

//***************************************************************************
// Module Initialization

BOOL Mh_MotorHandlerInit(UWORD uwTaskParam)
{
#ifdef _HW_DC
    if((sGlbOptAvailable.ulHardwareOpt&HWUNITID_BRI_FULL_0)==0)
        return TRUE;
#endif

    switch(uwTaskParam)
    {
        case MH_INIT_INPUT:
            return Mh_MotorDataFromFpgaInit();

        case MH_INIT_OUTPUT:
            return Mh_MotorDataToFpgaInit();

        case MH_INIT_LIMITLIST:
            return InitLimitList();

        case MH_INIT_POST:
            return Mh_POSTInit();

        default:
            assert(FALSE);
    }

    return FALSE;
}

//***************************************************************************
// Hardware option callback

#ifdef _APP_XC
void Mh_GetHwOpt(UWORD uwOpt, ULONG * pulHwOpt, ULONG * pulFPGAOpt)
{
#ifdef _HW_DC
    if((sGlbOptAvailable.ulHardwareOpt&HWUNITID_BRI_FULL_0)==0)
        return;
#endif

    *pulHwOpt  |=HWUNITID_BRI_FULL_0;
    *pulFPGAOpt|=FPGA_HW_CURCHANMINMAX;

#ifdef _HW_DC
    if(sMh_MotorDataParam.flags.b.bEnParallelBr1)
    {
        *pulHwOpt  |=HWUNITID_BRI_FULL_1;
        *pulFPGAOpt|=FPGA_HW_BRIDGE1;
    }
    if(sMh_MotorDataParam.flags.b.bEnParallelBr2)
    {
        *pulHwOpt  |=HWUNITID_BRI_FULL_2;
        *pulFPGAOpt|=FPGA_HW_BRIDGE2;
    }
#endif
    if(sGlbPowerBoardParameters.sUniv.sHwFeat.sOpt.ubSTO)
        *pulHwOpt  |=HWUNITID_STO_ROUTING|HWUNITID_STO;

    if(sMh_PlcAdvancedWorks.flags.b.bSelectStraight || sMh_PlcAdvancedWorks.flags.b.bEnablePhaseShift ||
       sMh_PlcAdvancedWorks.uwOvCurrCntLimit!=0 || sMh_PlcAdvancedWorks.flags.b.bDisableOCPwmout)
        *pulFPGAOpt|=FPGA_HW_PWMEX;
}
#endif

//***************************************************************************
// Calculate AC voltage layout

static BOOL voltagelayout(HWPRM2_VOLTAGE_LAYOUT  * psVoltCfg, HWPRM2_IN_AC_LAYOUT  * psInBrdgCfg, UWORD * puwSeq, SWORD * pswSign)
{
    UBYTE ubVRS,ubVST,ubVTR;

    // scramble pairs connection to PSPL_U and PSPL_V in order to have
    // in ubV?? the source of the nominal pair: 1=PU, 2=PV, 3=-PU-PV
    switch( (psVoltCfg->sOpt.ubVoltageU<<4) | (psVoltCfg->sOpt.ubVoltageV<<0) )
    {
        case 0x12:  // RS ST
            ubVRS=1; ubVST=2; ubVTR=3;
            break;

        case 0x13:  // RS TR
            ubVRS=1; ubVST=3; ubVTR=2;
            break;

        case 0x21:  // ST RS
            ubVRS=2; ubVST=1; ubVTR=3;
            break;

        case 0x23:  // ST TR
            ubVRS=2; ubVST=3; ubVTR=1;
            break;

        case 0x31:  // TR RS
            ubVRS=3; ubVST=1; ubVTR=2;
            break;

        case 0x32:  // TR ST
            ubVRS=3; ubVST=2; ubVTR=1;
            break;

        default:
            return FALSE;
    }

    // calculate external connections naming to internal RST
    switch( (psInBrdgCfg->ubPhaseR<<8) | (psInBrdgCfg->ubPhaseS<<4) | (psInBrdgCfg->ubPhaseT<<0) )
    {
        case 0x123: // R S T => +RS +ST
            puwSeq[0]=ubVRS; puwSeq[1]=ubVST; puwSeq[2]=ubVTR; pswSign[0]=+1; pswSign[1]=+1;
            break;

        case 0x321: // T S R => -ST -RS
            puwSeq[0]=ubVST; puwSeq[1]=ubVRS; puwSeq[2]=ubVTR; pswSign[0]=-1; pswSign[1]=-1;
            break;

        case 0x132: // R T S => -TR -ST
            puwSeq[0]=ubVTR; puwSeq[1]=ubVST; puwSeq[2]=ubVRS; pswSign[0]=-1; pswSign[1]=-1;
            break;

        case 0x312: // T R S => +TR +RS
            puwSeq[0]=ubVTR; puwSeq[1]=ubVRS; puwSeq[2]=ubVST; pswSign[0]=+1; pswSign[1]=+1;
            break;

        case 0x213: // S R T => -RS -TR
            puwSeq[0]=ubVRS; puwSeq[1]=ubVTR; puwSeq[2]=ubVST; pswSign[0]=-1; pswSign[1]=-1;
            break;

        case 0x231: // S T R => +ST +TR
            puwSeq[0]=ubVST; puwSeq[1]=ubVTR; puwSeq[2]=ubVRS; pswSign[0]=+1; pswSign[1]=+1;
            break;

        default:
            return FALSE;
    }

    return TRUE;
}

//***************************************************************************
// Calculate current and bridge layout from bridge and user params

static void currentbridgelayout(HWPRMS_BRIDGE_AND_CURRENT_LAYOUT  * psBrgLayout, UWORD uwUsrBrgLayout, MHRT_CFGLAYOUT * psDstCurLayout, MHRT_CFGLAYOUT * psDstPwmLayout)
{
  HWPRMS_BRIDGE_AND_CURRENT_LAYOUT sBridgeLayout;

    // if zero, find right value (old configuration)
  if(psBrgLayout->sCurrent.ubCurrentW==0)
  {
    if( ((psBrgLayout->sCurrent.ubCurrentU==1) && (psBrgLayout->sCurrent.ubCurrentV==2)) ||
        ((psBrgLayout->sCurrent.ubCurrentU==2) && (psBrgLayout->sCurrent.ubCurrentV==1)) )
      psBrgLayout->sCurrent.ubCurrentW=3;

    if( ((psBrgLayout->sCurrent.ubCurrentU==1) && (psBrgLayout->sCurrent.ubCurrentV==3)) ||
        ((psBrgLayout->sCurrent.ubCurrentU==3) && (psBrgLayout->sCurrent.ubCurrentV==1)) )
      psBrgLayout->sCurrent.ubCurrentW=2;

    if( ((psBrgLayout->sCurrent.ubCurrentU==2) && (psBrgLayout->sCurrent.ubCurrentV==3)) ||
        ((psBrgLayout->sCurrent.ubCurrentU==3) && (psBrgLayout->sCurrent.ubCurrentV==2)) )
      psBrgLayout->sCurrent.ubCurrentW=1;
  }

    // local work copy
  sBridgeLayout=*psBrgLayout;

    // assign user layout
  switch(uwUsrBrgLayout)
  {
    case MH_USRBRIDGELAYOUT_UVW:
        break;

    case MH_USRBRIDGELAYOUT_UWV:
        sBridgeLayout.sCurrent.ubCurrentV   = psBrgLayout->sCurrent.ubCurrentW;
        sBridgeLayout.sCurrent.ubCurrentW   = psBrgLayout->sCurrent.ubCurrentV;
        sBridgeLayout.sBridge.ubPhaseV      = psBrgLayout->sBridge.ubPhaseW;
        sBridgeLayout.sBridge.ubPhaseW      = psBrgLayout->sBridge.ubPhaseV;
        break;

    case MH_USRBRIDGELAYOUT_VUW:
        sBridgeLayout.sCurrent.ubCurrentU   = psBrgLayout->sCurrent.ubCurrentV;
        sBridgeLayout.sCurrent.ubCurrentV   = psBrgLayout->sCurrent.ubCurrentU;
        sBridgeLayout.sBridge.ubPhaseU      = psBrgLayout->sBridge.ubPhaseV;
        sBridgeLayout.sBridge.ubPhaseV      = psBrgLayout->sBridge.ubPhaseU;
        break;

    case MH_USRBRIDGELAYOUT_WUV:
        sBridgeLayout.sCurrent.ubCurrentU   = psBrgLayout->sCurrent.ubCurrentV;
        sBridgeLayout.sCurrent.ubCurrentV   = psBrgLayout->sCurrent.ubCurrentW;
        sBridgeLayout.sCurrent.ubCurrentW   = psBrgLayout->sCurrent.ubCurrentU;
        sBridgeLayout.sBridge.ubPhaseU      = psBrgLayout->sBridge.ubPhaseV;
        sBridgeLayout.sBridge.ubPhaseV      = psBrgLayout->sBridge.ubPhaseW;
        sBridgeLayout.sBridge.ubPhaseW      = psBrgLayout->sBridge.ubPhaseU;
        break;

    case MH_USRBRIDGELAYOUT_VWU:
        sBridgeLayout.sCurrent.ubCurrentU   = psBrgLayout->sCurrent.ubCurrentW;
        sBridgeLayout.sCurrent.ubCurrentV   = psBrgLayout->sCurrent.ubCurrentU;
        sBridgeLayout.sCurrent.ubCurrentW   = psBrgLayout->sCurrent.ubCurrentV;
        sBridgeLayout.sBridge.ubPhaseU      = psBrgLayout->sBridge.ubPhaseW;
        sBridgeLayout.sBridge.ubPhaseV      = psBrgLayout->sBridge.ubPhaseU;
        sBridgeLayout.sBridge.ubPhaseW      = psBrgLayout->sBridge.ubPhaseV;
        break;

    case MH_USRBRIDGELAYOUT_WVU:
        sBridgeLayout.sCurrent.ubCurrentU   = psBrgLayout->sCurrent.ubCurrentW;
        sBridgeLayout.sCurrent.ubCurrentV   = psBrgLayout->sCurrent.ubCurrentV;
        sBridgeLayout.sCurrent.ubCurrentW   = psBrgLayout->sCurrent.ubCurrentU;
        sBridgeLayout.sBridge.ubPhaseU      = psBrgLayout->sBridge.ubPhaseW;
        sBridgeLayout.sBridge.ubPhaseV      = psBrgLayout->sBridge.ubPhaseV;
        sBridgeLayout.sBridge.ubPhaseW      = psBrgLayout->sBridge.ubPhaseU;
        break;

    default:
        assert(FALSE);
  }

  psDstCurLayout->d.ubLayoutU = sBridgeLayout.sCurrent.ubCurrentU;
  psDstCurLayout->d.ubLayoutV = sBridgeLayout.sCurrent.ubCurrentV;
  psDstCurLayout->d.ubLayoutW = sBridgeLayout.sCurrent.ubCurrentW;

  psDstPwmLayout->d.ubLayoutU = sBridgeLayout.sBridge.ubPhaseU;
  psDstPwmLayout->d.ubLayoutV = sBridgeLayout.sBridge.ubPhaseV;
  psDstPwmLayout->d.ubLayoutW = sBridgeLayout.sBridge.ubPhaseW;
}

//***************************************************************************
//    

static BOOL Mh_MotorDataFromFpgaInit(void)
{
#ifndef _HW_DC
  BOOL bExternalDcBus ;
#endif
  HWPRMS_BRIDGE_AND_CURRENT_LAYOUT sBrgLayout ;
  MHRT_CFGLAYOUT sCfgBridge;
  ANPROC_CHAN sAInDef;
  ANPROC_RGOUT sRGODef;
  FLOAT flRatioV_DC_PEAK, flRatioI_PEAK, flRatioI_RMS, flRatioI_EQ_RMS ; 
  FLOAT flRatioV_AC_PEAK, flRatioV_AC_RMS, flRatioV_PSPL_PEAK=1.0 ;
#ifndef _HW_DC
  FLOAT flIuScale, flIvScale, flIwScale ;
#endif
  UWORD uwPwmDeadTime, uwCt;
  UWORD uwACLayout[3];
  SWORD swACPSPLSign[2];
  UWORD uwSwitchDeadTime;
  SWORD swLocOverVoltage=0, swDCBusScale=0, swPSPlUScale=0, swPSPlVScale=0;
  UWORD uwDCBusOffst=FPGA_ADCDEF_DCBUS_OFF,uwPSPlUOffst=FPGA_ADCDEF_PSPL_OFF,uwPSPlVOffst=FPGA_ADCDEF_PSPL_OFF;
  BOOL bBrakeDrivePresent = FALSE;
  BOOL bBrakeDrivePolarity;
  BOOL bInvertIWSign = FALSE;
  BOOL bACLineValid = FALSE;
  // used to pre-setup additional adc mapping to custom DSP, as DSP
  // custom code verification is done later
  BOOL bVerifyDSPCustom = (hpubDSPCustomAddr!=NULL && uwDSPCustomSize>0);

#ifdef _HW_DC
  BOOL bCustomCurrentScale = FALSE ;
#endif
#ifdef _HW_CT
  BOOL bIBrakeValid = FALSE;
#endif

  sMotorHandlerRun.sErrorSent.w = 0 ; /* clear errors */
  memset(&sMotorHandlerRun.sPStageStatusSet.b, 0, sizeof(MH_POWERSTAGECONTROL)) ; /* clear status setup */
  sMotorHandlerRun.flags.l = 0l;

  sMotorHandlerRun.flags.b.bDSPAdvance = bDSPAdvance ; // workaround to use bDSPAdvance in RT

  sMh_MotorDataOut.sIqLimit.slMin=SLONG_MIN_VALUE;
  sMh_MotorDataOut.sIqLimit.slMax=SLONG_MAX_VALUE;
  sMh_MotorDataOut.sIdLimit=sMh_MotorDataOut.sIqLimit;

  // setup default current adc tags
#ifndef _HW_DC
  sMotorHandlerRun.pubADTagsCurr[0]=FPGA_ADTAGS_PWI_ISENSEU;
  sMotorHandlerRun.pubADTagsCurr[1]=FPGA_ADTAGS_PWI_ISENSEV;
  sMotorHandlerRun.pubADTagsCurr[2]=FPGA_ADTAGS_PWI_DCBUS;
#else
  sMotorHandlerRun.pubADTagsCurr[0]=FPGA_ADTAGS_PWI_BR0_ISENSEU;
  sMotorHandlerRun.pubADTagsCurr[1]=FPGA_ADTAGS_PWI_BR0_ISENSEV;
  sMotorHandlerRun.pubADTagsCurr[2]=FPGA_ADTAGS_PWI_BR0_ISENSEW;
  sMotorHandlerRun.pubADTagsBr1Curr[0]=FPGA_ADTAGS_PWI_BR1_ISENSEU;
  sMotorHandlerRun.pubADTagsBr1Curr[1]=FPGA_ADTAGS_PWI_BR1_ISENSEV;
  sMotorHandlerRun.pubADTagsBr1Curr[2]=FPGA_ADTAGS_PWI_BR1_ISENSEW;
  sMotorHandlerRun.pubADTagsBr2Curr[0]=FPGA_ADTAGS_PWI_BR2_ISENSEU;
  sMotorHandlerRun.pubADTagsBr2Curr[1]=FPGA_ADTAGS_PWI_BR2_ISENSEV;
  sMotorHandlerRun.pubADTagsBr2Curr[2]=FPGA_ADTAGS_PWI_BR2_ISENSEW;
#endif

  // setup default current offsets and multiplier
  sMotorHandlerRun.tCurCal[0].uwOffst = FPGA_ADCDEF_PWI_II_OFF;
  sMotorHandlerRun.tCurCal[1].uwOffst = FPGA_ADCDEF_PWI_II_OFF;
  sMotorHandlerRun.tCurCal[2].uwOffst = FPGA_ADCDEF_PWI_II_OFF;
  sMotorHandlerRun.tCurCal[0].swScale = FPGA_ADCDEF_PWI_II_MUL;
  sMotorHandlerRun.tCurCal[1].swScale = FPGA_ADCDEF_PWI_II_MUL;
  sMotorHandlerRun.tCurCal[2].swScale = FPGA_ADCDEF_PWI_II_MUL;
#ifdef _HW_DC
  sMotorHandlerRun.tCurBr1Cal[0].uwOffst = FPGA_ADCDEF_PWI_II_OFF;
  sMotorHandlerRun.tCurBr1Cal[1].uwOffst = FPGA_ADCDEF_PWI_II_OFF;
  sMotorHandlerRun.tCurBr1Cal[2].uwOffst = FPGA_ADCDEF_PWI_II_OFF;
  sMotorHandlerRun.tCurBr1Cal[0].swScale = FPGA_ADCDEF_PWI_II_MUL;
  sMotorHandlerRun.tCurBr1Cal[1].swScale = FPGA_ADCDEF_PWI_II_MUL;
  sMotorHandlerRun.tCurBr1Cal[2].swScale = FPGA_ADCDEF_PWI_II_MUL;
  sMotorHandlerRun.tCurBr2Cal[0].uwOffst = FPGA_ADCDEF_PWI_II_OFF;
  sMotorHandlerRun.tCurBr2Cal[1].uwOffst = FPGA_ADCDEF_PWI_II_OFF;
  sMotorHandlerRun.tCurBr2Cal[2].uwOffst = FPGA_ADCDEF_PWI_II_OFF;
  sMotorHandlerRun.tCurBr2Cal[0].swScale = FPGA_ADCDEF_PWI_II_MUL;
  sMotorHandlerRun.tCurBr2Cal[1].swScale = FPGA_ADCDEF_PWI_II_MUL;
  sMotorHandlerRun.tCurBr2Cal[2].swScale = FPGA_ADCDEF_PWI_II_MUL;
#endif

  // setup default delay for PWI and PWB
  sMh_PlcAdvancedWorks.uwCurrPDTime = PDDEFAULT_CURR;
  sMh_PlcAdvancedWorks.uwVoltPDTime = PDDEFAULT_VOLT;

  // current scale calibration (Iw in place of Vdc when apply)
#ifndef _HW_DC
  flIuScale = (FLOAT)sGlbControlBoardParameters.sStd.sCurrentPhaseU.swScale/8192.0;
  flIvScale = (FLOAT)sGlbControlBoardParameters.sStd.sCurrentPhaseV.swScale/8192.0;
  flIwScale = (FLOAT)sGlbControlBoardParameters.sStd.sVoltageDcLink.swScale/8192.0;
#endif

  sMotorHandlerRun.flags.b.bAlmaPS = FALSE ;    
    
  // default AC in layout
  assert(voltagelayout(&sDefVoltCfg, &sDefInBrdgCfg, uwACLayout, swACPSPLSign));

    // setup power board specifics
  switch(sGlbPowerBoardParameters.sProductInfo.uwProductCode)
  {
#ifndef _HW_DC
    case HWPRM_PCODE_POWER_BOARD_OLD_AXW_DC: /* without Id-Chip */
    case HWPRM_PCODE_POWER_BOARD_OLD_AXW_AC: /* without Id-Chip */
    case HWPRM_PCODE_POWER_BOARD_OLD_AXM_AC: /* without Id-Chip */
    case HWPRM_PCODE_POWER_BOARD_AXP_AC:     /* with    Id-Chip */
    case HWPRM_PCODE_POWER_BOARD_AXP_DS_AC:
    case HWPRM_PCODE_POWER_BOARD_AXM_AC:     /* with    Id-Chip */
    case HWPRM_PCODE_POWER_BOARD_AXM_AC_ALMAPS:
    case HWPRM_PCODE_POWER_BOARD_AXM_DS_AC:
    case HWPRM_PCODE_POWER_BOARD_AXW_AC:
    case HWPRM_PCODE_POWER_BOARD_AXW_DC:
        FPGA_OPTIONS_PWB_ANALOG_BRD=TRUE;

        if(sGlbPowerBoardParamDataCode==DATACODE_HW_POWER_BOARD_AXM_AXP_AC)
        {
            sBrgLayout                = sGlbPowerBoardParameters.sAxmAxpAC.sBridgeParams.sBridgeCurrentLayout ;
            uwSwitchDeadTime          = sGlbPowerBoardParameters.sAxmAxpAC.sBridgeParams.uwSwitchDeadTime;
            sMotorHandlerRun.slMaxOverCurrent = sGlbPowerBoardParameters.sAxmAxpAC.sBridgeParams.sOperativeLimits.slOverCurrent;
            sMh_MotorDataOut.sDriveLimit.slCurrentLimit = sGlbPowerBoardParameters.sAxmAxpAC.sBridgeParams.sOperativeLimits.slCurrentLimit ;
            swLocOverVoltage          = sGlbPowerBoardParameters.sAxmAxpAC.sBridgeParams.sOperativeLimits.swOverVoltage ;

                // scale factors
            flRatioI_PEAK             = sGlbPowerBoardParameters.sAxmAxpAC.flCurrentScale;
            flRatioV_DC_PEAK          = sGlbPowerBoardParameters.sAxmAxpAC.flDCVoltageScale;

                // voltage scale calibration
            swDCBusScale = sGlbPowerBoardParameters.sAxmAxpAC.sVoltageDcLink.swScale ;
    
                // current scale calibration correction
            flIuScale *= (FLOAT)sGlbPowerBoardParameters.sAxmAxpAC.sCurrentPhaseU.swScale/8192.0;
            flIvScale *= (FLOAT)sGlbPowerBoardParameters.sAxmAxpAC.sCurrentPhaseV.swScale/8192.0;

            bBrakeDrivePresent  = TRUE;
            bBrakeDrivePolarity = sGlbPowerBoardParameters.sAxmAxpAC.sBrakeCircuit.sOptions.ubDrivePolarity;
            sMotorHandlerRun.flags.b.bResFltPolarity=sGlbPowerBoardParameters.sAxmAxpAC.sBridgeParams.sBridgeCurrentLayout.sBridge.ubResFltPolarity;
        }
        else
        {
            sBrgLayout                = sGlbPowerBoardParameters.sAxmAxpACV2.sBridgeParams.sBridgeCurrentLayout ;
            uwSwitchDeadTime          = sGlbPowerBoardParameters.sAxmAxpACV2.sBridgeParams.uwSwitchDeadTime;
            sMotorHandlerRun.slMaxOverCurrent = sGlbPowerBoardParameters.sAxmAxpACV2.sBridgeParams.sOperativeLimits.slOverCurrent;
            sMh_MotorDataOut.sDriveLimit.slCurrentLimit = sGlbPowerBoardParameters.sAxmAxpACV2.sBridgeParams.sOperativeLimits.slCurrentLimit ;
            swLocOverVoltage          = sGlbPowerBoardParameters.sAxmAxpACV2.sBridgeParams.sOperativeLimits.swOverVoltage ;

                // scale factors
            flRatioI_PEAK             = sGlbPowerBoardParameters.sAxmAxpACV2.flCurrentScale;
            flRatioV_DC_PEAK          = sGlbPowerBoardParameters.sAxmAxpACV2.flDCVoltageScale;

                // voltage scale calibration
            swDCBusScale = sGlbPowerBoardParameters.sAxmAxpACV2.sVoltageDcLink.swScale ;
    
                // current scale calibration correction
            flIuScale *= (FLOAT)sGlbPowerBoardParameters.sAxmAxpACV2.sCurrentPhaseU.swScale/8192.0;
            flIvScale *= (FLOAT)sGlbPowerBoardParameters.sAxmAxpACV2.sCurrentPhaseV.swScale/8192.0;

            bBrakeDrivePresent  = TRUE;
            bBrakeDrivePolarity = sGlbPowerBoardParameters.sAxmAxpACV2.sBrakeCircuit.sOptions.ubDrivePolarity;
            sMotorHandlerRun.flags.b.bResFltPolarity=sGlbPowerBoardParameters.sAxmAxpACV2.sBridgeParams.sBridgeCurrentLayout.sBridge.ubResFltPolarity;
        }

        if ((sGlbPowerBoardParameters.sProductInfo.uwProductCode == HWPRM_PCODE_POWER_BOARD_OLD_AXW_DC) ||
            (sGlbPowerBoardParameters.sProductInfo.uwProductCode == HWPRM_PCODE_POWER_BOARD_OLD_AXW_AC) || 
            (sGlbPowerBoardParameters.sProductInfo.uwProductCode == HWPRM_PCODE_POWER_BOARD_AXW_AC)     ||
            (sGlbPowerBoardParameters.sProductInfo.uwProductCode == HWPRM_PCODE_POWER_BOARD_AXW_DC)     ) 
        {
          bExternalDcBus = TRUE ; /* DcBus External A2D */
          flRatioV_PSPL_PEAK = flRatioV_DC_PEAK / 2.0;
          swDCBusScale = (SWORD)((SLONG)swDCBusScale*8192l/6553l);   // factor correction for past-compatibility
          FPGA_OPTIONS_PWB_SEL_TRUE_IW = TRUE;
          bInvertIWSign = TRUE;
        }
        else
          bExternalDcBus = FALSE ; /* DcBus Internal A2D */

        if ((sGlbPowerBoardParameters.sProductInfo.uwProductCode == HWPRM_PCODE_POWER_BOARD_OLD_AXW_AC) || 
            (sGlbPowerBoardParameters.sProductInfo.uwProductCode == HWPRM_PCODE_POWER_BOARD_AXW_AC)     ) 
          bACLineValid = TRUE;
        else
          bACLineValid = FALSE;

            // for AxM (old and new) and AxP there's no igbt desat line
        if ((sGlbPowerBoardParameters.sProductInfo.uwProductCode == HWPRM_PCODE_POWER_BOARD_OLD_AXM_AC) ||
            (sGlbPowerBoardParameters.sProductInfo.uwProductCode == HWPRM_PCODE_POWER_BOARD_AXP_AC) || 
            (sGlbPowerBoardParameters.sProductInfo.uwProductCode == HWPRM_PCODE_POWER_BOARD_AXM_AC) ||
            (sGlbPowerBoardParameters.sProductInfo.uwProductCode == HWPRM_PCODE_POWER_BOARD_AXM_AC_ALMAPS))
            FPGA_ENABLE_DESATFROMCURR = TRUE;
        else
            FPGA_ENABLE_DESATFROMCURR = FALSE;

        if (sGlbPowerBoardParameters.sProductInfo.uwProductCode == HWPRM_PCODE_POWER_BOARD_AXM_AC_ALMAPS)
            sMotorHandlerRun.flags.b.bAlmaPS = TRUE ;

        sMotorHandlerRun.flags.b.bSTOInstalled = FALSE;

            // patch overvoltage threshold
        swLocOverVoltage = (swLocOverVoltage>9000) ? 9000 : swLocOverVoltage;

        break;
        
    case HWPRM_PCODE_POWER_BOARD_DIF_CI:
    case HWPRM_PCODE_POWER_BOARD_DIF_PI:
    case HWPRM_PCODE_POWER_BOARD_DIF_BC:
    case HWPRM_PCODE_POWER_BOARD_DIF_BL:
        FPGA_OPTIONS_PWB_DIGITAL_BRD=TRUE;
        
        if((sGlbPowerBoardParameters.sProductInfo.uwProductCode!=HWPRM_PCODE_POWER_BOARD_DIF_BC) && 
           (sGlbPowerBoardParameters.sProductInfo.uwProductCode!=HWPRM_PCODE_POWER_BOARD_DIF_BL))
        {
            FPGA_OPTIONS_PWB_SEL_TRUE_IW=TRUE;
            sMotorHandlerRun.flags.b.bSTOInstalled = TRUE;
            bBrakeDrivePresent  = TRUE;
        }
        else
        {
            FPGA_OPTIONS_PWB_SEL_TRUE_IW=FALSE;
            sMotorHandlerRun.flags.b.bSTOInstalled = FALSE;
            bBrakeDrivePresent  = FALSE;
        }        

        if(sGlbPowerBoardParameters.sProductInfo.uwProductCode!=HWPRM_PCODE_POWER_BOARD_DIF_CI)
        {
            FPGA_OPTIONS_PWB_EXT_IADC=TRUE;
            flIuScale=flIvScale=flIwScale=1.0;
            sMh_PlcAdvancedWorks.uwCurrPDTime = sGlbPowerBoardParameters.sDif.uwCurrentAdcPDTime - (UWORD)PDTIMEOFFSET_CURR;
            sMh_PlcAdvancedWorks.uwVoltPDTime = PDDEF_PWB_VOLT;
        }

        sBrgLayout                = sGlbPowerBoardParameters.sDif.sBridgeParams.sBridgeCurrentLayout ;
        uwSwitchDeadTime          = sGlbPowerBoardParameters.sDif.sBridgeParams.uwSwitchDeadTime;
        sMotorHandlerRun.slMaxOverCurrent = sGlbPowerBoardParameters.sDif.sBridgeParams.sOperativeLimits.slOverCurrent;
        sMh_MotorDataOut.sDriveLimit.slCurrentLimit = sGlbPowerBoardParameters.sDif.sBridgeParams.sOperativeLimits.slCurrentLimit ;

            // scale factors
        flRatioI_PEAK             = sGlbPowerBoardParameters.sDif.flCurrentScale;
        flRatioV_DC_PEAK          = sGlbPowerBoardParameters.sDif.flDCVoltageScale;
        flRatioV_PSPL_PEAK        = flRatioV_DC_PEAK / 2.0;

            // ratio must be further divided by 2 due to bug on old FPGA PWB handling
        flRatioV_DC_PEAK          /= 2.0;
        flRatioV_PSPL_PEAK        /= 2.0;

            // voltage scale calibration fixed as in the previous fw was never used
        swDCBusScale = FPGA_ADCDEF_PWI_II_MUL ;

            // current scale calibration correction
        flIuScale   *= (FLOAT)sGlbPowerBoardParameters.sDif.sCurrentPhaseU.swScale/8192.0;
        flIvScale   *= (FLOAT)sGlbPowerBoardParameters.sDif.sCurrentPhaseV.swScale/8192.0;
        flIwScale   *= (FLOAT)sGlbPowerBoardParameters.sDif.sCurrentPhaseW.swScale/8192.0;

        swPSPlUScale = sGlbPowerBoardParameters.sDif.sVoltageRS.swScale*swACPSPLSign[0];
        swPSPlVScale = sGlbPowerBoardParameters.sDif.sVoltageST.swScale*swACPSPLSign[1];

        bBrakeDrivePolarity = sGlbPowerBoardParameters.sDif.sBrakeCircuit.sOptions.ubDrivePolarity;
        sMotorHandlerRun.flags.b.bResFltPolarity=sGlbPowerBoardParameters.sDif.sBridgeParams.sBridgeCurrentLayout.sBridge.ubResFltPolarity;

        bExternalDcBus = TRUE ; /* DcBus External A2D */
#ifdef _HW_CT
        if(sGlbPowerBoardParameters.sProductInfo.uwProductCode==HWPRM_PCODE_POWER_BOARD_DIF_BC ||
           sGlbPowerBoardParameters.sProductInfo.uwProductCode==HWPRM_PCODE_POWER_BOARD_DIF_BL)
            bIBrakeValid = TRUE; /* Brake current A2D */
        else
#endif
            bACLineValid = TRUE ;

        sMotorHandlerRun.flags.b.bDIF = TRUE;
#ifdef _INFINEON_
        P9_IOCR01 = 0x0080; // reset fault port

#else
        Gpio_SetMode(PWB_RESET_FAULT_PIN, GPIO_DIR_OUT);

        if(sMotorHandlerRun.flags.b.bSTOInstalled)
        {
          Gpio_SetMode(STO_HI_PIN, GPIO_DIR_IN);
          Gpio_SetMode(STO_LO_PIN, GPIO_DIR_IN);
        }
#endif
            // get and patch overvoltage threshold
        swLocOverVoltage = sGlbPowerBoardParameters.sDif.sBridgeParams.sOperativeLimits.swOverVoltage;
        swLocOverVoltage = (swLocOverVoltage>9000) ? 9000 : swLocOverVoltage;

        break;
#endif

    case HWPRM_PCODE_POWER_BOARD_UNIVERSAL:
#ifndef _HW_DC
            // only U and V sensors configuration not supported
        if(sGlbPowerBoardParameters.sUniv.sCurrRange.sCalPhaseW.swScale==0)
            return FALSE;
#endif

            // only standard high<->IGBT ON and low fault supported here (both power bridge and brake)
        if(!sGlbPowerBoardParameters.sUniv.sBrdgFeat.sOpt.ubGateLoPolarity ||
           !sGlbPowerBoardParameters.sUniv.sBrdgFeat.sOpt.ubGateHiPolarity ||
           sGlbPowerBoardParameters.sUniv.sBrdgFeat.sOpt.ubFaultPolarity ||
           sGlbPowerBoardParameters.sUniv.sBrkFeat.sOpt.ubFaultPolarity)
            return FALSE;

           //  no CS feedback supported
        if(sGlbPowerBoardParameters.sUniv.sVoltCfg.sOpt.ubCSFeedback ||
            sGlbPowerBoardParameters.sUniv.sCurrCfg.sOpt.ubCSFeedback)
            return FALSE;

#ifndef _HW_DC
        FPGA_OPTIONS_PWB_DIGITAL_BRD=TRUE;
        FPGA_OPTIONS_PWB_SEL_TRUE_IW=TRUE;
        FPGA_OPTIONS_PWB_EXT_IADC=TRUE;
        flIuScale=flIvScale=flIwScale=1.0;
#endif

        sMh_PlcAdvancedWorks.uwCurrPDTime = sGlbPowerBoardParameters.sUniv.sCurrCfg.uwPDTime;
        sMh_PlcAdvancedWorks.uwVoltPDTime = sGlbPowerBoardParameters.sUniv.sVoltCfg.uwPDTime;

        sBrgLayout.sBridge.ubPhaseU    = sGlbPowerBoardParameters.sUniv.sOutBrdgCfg.ubPhaseU;
        sBrgLayout.sBridge.ubPhaseV    = sGlbPowerBoardParameters.sUniv.sOutBrdgCfg.ubPhaseV;
        sBrgLayout.sBridge.ubPhaseW    = sGlbPowerBoardParameters.sUniv.sOutBrdgCfg.ubPhaseW;
        sBrgLayout.sCurrent.ubCurrentU = sGlbPowerBoardParameters.sUniv.sCurrCfg.sOpt.ubCurrentU;
        sBrgLayout.sCurrent.ubCurrentV = sGlbPowerBoardParameters.sUniv.sCurrCfg.sOpt.ubCurrentV;
        sBrgLayout.sCurrent.ubCurrentW = sGlbPowerBoardParameters.sUniv.sCurrCfg.sOpt.ubCurrentW;

        uwSwitchDeadTime               = sGlbPowerBoardParameters.sUniv.sBrdgFeat.uwSwitchDeadTime;
        sMotorHandlerRun.slMaxOverCurrent = (SLONG)(sGlbPowerBoardParameters.sUniv.sBrdgFeat.fOverCurrent*10000.0);
        sMh_MotorDataOut.sDriveLimit.slCurrentLimit = (SLONG)(sGlbPowerBoardParameters.sUniv.sBrdgFeat.fCurrentLimit*10000.0);
        swLocOverVoltage               = (SWORD)(sGlbPowerBoardParameters.sUniv.sBrdgFeat.fOverVoltage*10.0);

            // scale factors, referred to full adc range and calculated to convert into internal integer units
#ifndef _HW_DC
        flRatioI_PEAK             = sGlbPowerBoardParameters.sUniv.sCurrRange.fRange*10000.0/32768.0;
#else
        flRatioI_PEAK             = sGlbPowerBoardParameters.sUniv.fRange*10000.0/32768.0;
#endif
        flRatioV_DC_PEAK          = sGlbPowerBoardParameters.sUniv.sVoltRange.fDCRange*10.0/32768.0;
        flRatioV_PSPL_PEAK        = sGlbPowerBoardParameters.sUniv.sVoltRange.fACRange*10.0/32768.0;

            // current scale calibration correction
#ifndef _HW_DC
        flIuScale   *= (FLOAT)sGlbPowerBoardParameters.sUniv.sCurrRange.sCalPhaseU.swScale/8192.0;
        flIvScale   *= (FLOAT)sGlbPowerBoardParameters.sUniv.sCurrRange.sCalPhaseV.swScale/8192.0;
        flIwScale   *= (FLOAT)sGlbPowerBoardParameters.sUniv.sCurrRange.sCalPhaseW.swScale/8192.0;
#else
            // currents calibrations
        sMotorHandlerRun.tCurCal[0]    = sGlbPowerBoardParameters.sUniv.sBrCurrRange[0].sCalPhaseU;
        sMotorHandlerRun.tCurCal[1]    = sGlbPowerBoardParameters.sUniv.sBrCurrRange[0].sCalPhaseV;
        sMotorHandlerRun.tCurCal[2]    = sGlbPowerBoardParameters.sUniv.sBrCurrRange[0].sCalPhaseW;
        sMotorHandlerRun.tCurBr1Cal[0] = sGlbPowerBoardParameters.sUniv.sBrCurrRange[1].sCalPhaseU;
        sMotorHandlerRun.tCurBr1Cal[1] = sGlbPowerBoardParameters.sUniv.sBrCurrRange[1].sCalPhaseV;
        sMotorHandlerRun.tCurBr1Cal[2] = sGlbPowerBoardParameters.sUniv.sBrCurrRange[1].sCalPhaseW;
        sMotorHandlerRun.tCurBr2Cal[0] = sGlbPowerBoardParameters.sUniv.sBrCurrRange[2].sCalPhaseU;
        sMotorHandlerRun.tCurBr2Cal[1] = sGlbPowerBoardParameters.sUniv.sBrCurrRange[2].sCalPhaseV;
        sMotorHandlerRun.tCurBr2Cal[2] = sGlbPowerBoardParameters.sUniv.sBrCurrRange[2].sCalPhaseW;
#endif

        sMotorHandlerRun.flags.b.bResFltPolarity=!sGlbPowerBoardParameters.sUniv.sBrdgFeat.sOpt.ubResetFltPolarity;
        bBrakeDrivePresent  = sGlbPowerBoardParameters.sUniv.sBrkFeat.sOpt.ubBrakeDrive;
        bBrakeDrivePolarity = sGlbPowerBoardParameters.sUniv.sBrkFeat.sOpt.ubDrivePolarity;

        	// voltage scale calibration
        swDCBusScale = sGlbPowerBoardParameters.sUniv.sVoltRange.sCalVDC.swScale ;
        swPSPlUScale = sGlbPowerBoardParameters.sUniv.sVoltRange.sCalVRS.swScale*swACPSPLSign[0];
        swPSPlVScale = sGlbPowerBoardParameters.sUniv.sVoltRange.sCalVST.swScale*swACPSPLSign[1];

        uwDCBusOffst += sGlbPowerBoardParameters.sUniv.sVoltRange.sCalVDC.uwOffst-0x8000;
        uwPSPlUOffst += sGlbPowerBoardParameters.sUniv.sVoltRange.sCalVRS.uwOffst-0x8000;
        uwPSPlVOffst += sGlbPowerBoardParameters.sUniv.sVoltRange.sCalVST.uwOffst-0x8000;

        bACLineValid = sGlbPowerBoardParameters.sUniv.sVoltCfg.sOpt.ubVac ;

            // AC input layout only if valid
        if(bACLineValid)
          if(!voltagelayout(&sGlbPowerBoardParameters.sUniv.sVoltCfg, &sGlbPowerBoardParameters.sUniv.sInBrdgCfg, uwACLayout, swACPSPLSign))
            return FALSE;

        sMotorHandlerRun.flags.b.bSTOInstalled = sGlbPowerBoardParameters.sUniv.sHwFeat.sOpt.ubSTO;
        sMotorHandlerRun.flags.b.bDIF = TRUE;

#ifndef _HW_DC
        bExternalDcBus = TRUE ; // DcBus External A2D
#ifdef _INFINEON_
        P9_IOCR01 = 0x0080;     // reset fault port
#else
        Gpio_SetMode(PWB_RESET_FAULT_PIN, GPIO_DIR_OUT);
#endif // _infineon_
#else
#ifdef _INFINEON_
        P1_IOCR07 = 0x0080;
#else
        Gpio_SetMode(PWB_RESET_FAULT_PIN, GPIO_DIR_OUT);
#endif // _infineon_
        
        sMotorHandlerRun.flags.b.bParallelBr1Mode=sMh_MotorDataParam.flags.b.bEnParallelBr1;
        sMotorHandlerRun.flags.b.bParallelBr2Mode=sMh_MotorDataParam.flags.b.bEnParallelBr2;
#endif // _hw_dc
        break;

    default:
        assert(FALSE);
        return FALSE;    // just avoid warning C184
  }

  sMotorHandlerRun.flRatioV_DC_PEAK = flRatioV_DC_PEAK ; // neeeded by Vd and Vq filter and Vmotor

    // check parameters validity  
  if(!ResetOnlyParametersCheck())
    return FALSE;

  // assign input AC to the appropriate variables
  sMotorHandlerRun.pswACPU  = ACIN_DEST(uwACLayout[0]);
  sMotorHandlerRun.pswACPV  = ACIN_DEST(uwACLayout[1]);
  sMotorHandlerRun.pswACPUV = ACIN_DEST(uwACLayout[2]);

  /* hardware pwm and currents layout configuration */
  currentbridgelayout(&sBrgLayout, sMh_MotorDataParam.uwUserBridgeLayout,
                        &sMotorHandlerRun.sCfgCurrents, &sCfgBridge);

  FPGA_PWM_BRIDGE_LAYOUT = PWM_BRIDGELAYOUT(sCfgBridge);

  // if deadtime is not zero then setup
  if(uwSwitchDeadTime)
  {
    FLOAT flDT;

    flDT = (FLOAT)uwSwitchDeadTime / 100.0 / 0.025 ; // [25nsec] [40MHz]
      // max 2^9
    if(flDT>511.0)
        flDT = 511.0;

    uwPwmDeadTime = (UWORD)flDT;
    FPGA_PWM_DEADTIME = uwPwmDeadTime;
  }
  else
    uwPwmDeadTime = FPGA_PWM_DEADTIME;

  /* I and V ratios setup */
  flRatioI_RMS = flRatioI_PEAK / FLOAT_SQRT_OF_TWO;
  flRatioV_AC_RMS  = flRatioV_DC_PEAK / FLOAT_SQRT_OF_THREE ; // = Vdc / sqrt(3) (necessario per Vd, Vq (?))
  flRatioV_AC_PEAK = flRatioV_DC_PEAK * (FLOAT_SQRT_OF_TWO / 3.0) ;   // = Vdc * (sqrt(2) / (sqrt(3) * sqrt(3))) (necessario per Vu, Vv)

#ifdef _HW_DC
  // if Custom DSP, check if current scale is custom
  if (bVerifyDSPCustom)
  {
    if(sMh_PlcAdvancedWorks.flags.b.bEnableCustomCurrScale)
        bCustomCurrentScale = TRUE ;
    else
        bCustomCurrentScale = FALSE ;
  }
  else
    bCustomCurrentScale = FALSE ;

  // if parallel bridge selected and standard DSP, Irms is computed by sum of two
  // currents phase divided by 2 to keep same resolution
  if(!bCustomCurrentScale && sMotorHandlerRun.flags.b.bParallelBr1Mode)
  {
    // as well as for three parallel bridge
    if(sMotorHandlerRun.flags.b.bParallelBr2Mode)
    {
      flRatioI_EQ_RMS = flRatioI_RMS * 3.0;
      sMh_MotorDataOut.sDriveLimit.slCurrentLimit *= 3.0;
    }
    else
    {
      flRatioI_EQ_RMS = flRatioI_RMS * 2.0;
      sMh_MotorDataOut.sDriveLimit.slCurrentLimit *= 2.0;
    }
  }
  else
    flRatioI_EQ_RMS = flRatioI_RMS;

  UmConv_Init_16to32_Rev(NULL, &sInternal2Fpga_I_EQ_RMS,   flRatioI_EQ_RMS) ;
#else
  flRatioI_EQ_RMS = flRatioI_RMS;
#endif
    
  UmConv_Init_16to32_Rev(NULL, &sInternal2Fpga_I_PEAK,     flRatioI_PEAK) ;
  UmConv_Init_16to32_Rev(NULL, &sInternal2Fpga_I_RMS,      flRatioI_RMS) ;
  UmConv_Init_16to32_Rev(NULL, &sInternal2Fpga_V_DC_PEAK,  flRatioV_DC_PEAK * 65536.0) ;
  UmConv_Init_16to32_Rev(NULL, &sInternal2Fpga_V_AC_PEAK,  flRatioV_AC_PEAK * 65536.0) ;
  UmConv_Init_16to32_Rev(NULL, &sInternal2Fpga_V_AC_RMS,   flRatioV_AC_RMS  * 65536.0) ;
#ifndef _HW_DC
  if(bExternalDcBus)
#endif
    UmConv_Init_16to32_Rev(&sFpga2Internal_V_PSPL_PEAK, &sInternal2Fpga_V_PSPL_PEAK, flRatioV_PSPL_PEAK * 65536.0) ;

  // ratio calc for automatic gains
  flGainsRatio = flRatioI_EQ_RMS * FPGA_GAINS_VOLTAGE_NORMALIZATION / flRatioV_DC_PEAK ;
    
  // record ratios for plc usage
  sMh_PlcUMRatios.flRatioIpk=flRatioI_PEAK;
  sMh_PlcUMRatios.flRatioIrms=flRatioI_RMS;
  sMh_PlcUMRatios.flRatioVDCpk=flRatioV_DC_PEAK;
  sMh_PlcUMRatios.flRatioVACpk=flRatioV_AC_PEAK;
  sMh_PlcUMRatios.flRatioVACrms=flRatioV_AC_RMS;
  sMh_PlcUMRatios.flRatioVACLinepk=flRatioV_PSPL_PEAK;

#ifndef _HW_DC
  FPGA_OPTIONS_POWER_BOARD_TYPE = bExternalDcBus ;
#endif

  SetPDDelays(sMh_PlcAdvancedWorks.uwCurrPDTime, sMh_PlcAdvancedWorks.uwVoltPDTime);
 
  // current scale correction with default value and application
#ifndef _HW_DC
  if(flIuScale==0.0 && flIvScale==0.0)
    flIuScale  =flIvScale  =flIwScale  =1.0;

  if(bInvertIWSign)
    flIwScale=-flIwScale;

  sMotorHandlerRun.tCurCal[0].swScale = (SWORD)(flIuScale*FPGA_ADCDEF_PWI_II_MUL);
  sMotorHandlerRun.tCurCal[1].swScale = (SWORD)(flIvScale*FPGA_ADCDEF_PWI_II_MUL);
  if(!FPGA_OPTIONS_PWB_SEL_TRUE_IW)
  {
    // virtual Iw is made in the FPGA as +Iu+Iv, relying on scaling unit to change the sign
    sMotorHandlerRun.tCurCal[2].swScale = -(SWORD)(flIuScale*flIvScale*FPGA_ADCDEF_PWI_II_MUL);
    sMotorHandlerRun.pubADTagsCurr[2]=FPGA_ADTAGS_VIRTUAL_IW;
  }
  else
    sMotorHandlerRun.tCurCal[2].swScale = (SWORD)(flIwScale*FPGA_ADCDEF_PWI_II_MUL);

  // final current scaling user adjustment
  for(uwCt=0;uwCt<3;uwCt++)
    sMotorHandlerRun.tCurCal[uwCt].swScale=CURR_SCALING(sMotorHandlerRun.tCurCal[uwCt].swScale,sMh_CurrScaleAdj[uwCt]);
#endif

  // setup Vac reading
  sMh_MotorDataOut.sPowerStageSts.b.bACInputDataValid = bACLineValid && !sMh_MotorDataParam.flags.b.bDisVacMgm;

  // setup bridge 1/2 handling
#ifdef _HW_DC
  sMh_MotorDataOut.sPowerStageSts.b.bBridge1Valid = sMotorHandlerRun.flags.b.bParallelBr1Mode;
  if(sMotorHandlerRun.flags.b.bParallelBr1Mode)
    sMh_MotorDataOut.sPowerStageSts.b.bBridge2Valid = sMotorHandlerRun.flags.b.bParallelBr2Mode;
#endif

  // setup analog input channels
  
  // one shot average of # samples
  if(sMh_PlcAdvancedWorks.uwCurrentSamples>=4 && sMh_PlcAdvancedWorks.uwCurrentSamples<=125)
  {
      sAInDef.ubOpt=ANPROC_OF_AVG_1SHT|ANPROC_OF_BLK_NONE|ANPROC_OF_DST_LONG;
      sAInDef.ubNumSample=(UBYTE)sMh_PlcAdvancedWorks.uwCurrentSamples;
  }
  // standard setting (continuos average of 125 samples)
  else
  {
      sAInDef.ubOpt=ANPROC_OF_AVG_CONT|ANPROC_OF_BLK_INST|ANPROC_OF_DST_LONG;
      sAInDef.ubNumSample=125;
  }

  // phase currents
  sAInDef.pvDstExtImm=NULL;
  sAInDef.flScale=flRatioI_PEAK;

  // virtual U current
  sAInDef.sCal.uwOffst=sMotorHandlerRun.tCurCal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutU-1].uwOffst;
  sAInDef.sCal.swScale=sMotorHandlerRun.tCurCal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutU-1].swScale;
  sAInDef.uwDstIntImm=FPGAIR_IFB_IU;
  sAInDef.uwDstIntAvg=FPGAIR_IFB_AU;
  sAInDef.pvDstExtImm=&sMh_MotorDataOut.slIuFb_Imm;
  sAInDef.pvDstExtAvg=&sMh_MotorDataOut.slIuFb;
  AnProc_Set(sMotorHandlerRun.pubADTagsCurr[sMotorHandlerRun.sCfgCurrents.d.ubLayoutU-1],&sAInDef);

  // virtual V current
  sAInDef.sCal.uwOffst=sMotorHandlerRun.tCurCal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutV-1].uwOffst;
  sAInDef.sCal.swScale=sMotorHandlerRun.tCurCal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutV-1].swScale;
  sAInDef.uwDstIntImm=FPGAIR_IFB_IV;
  sAInDef.uwDstIntAvg=FPGAIR_IFB_AV;
  sAInDef.pvDstExtImm=&sMh_MotorDataOut.slIvFb_Imm;
  sAInDef.pvDstExtAvg=&sMh_MotorDataOut.slIvFb;
  AnProc_Set(sMotorHandlerRun.pubADTagsCurr[sMotorHandlerRun.sCfgCurrents.d.ubLayoutV-1],&sAInDef);

  // virtual W current
  sAInDef.sCal.uwOffst=sMotorHandlerRun.tCurCal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutW-1].uwOffst;
  sAInDef.sCal.swScale=sMotorHandlerRun.tCurCal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutW-1].swScale;
  sAInDef.uwDstIntImm=FPGAIR_IFB_IW;
  sAInDef.uwDstIntAvg=FPGAIR_IFB_AW;
  sAInDef.pvDstExtImm=&sMh_MotorDataOut.slIwFb_Imm;
  sAInDef.pvDstExtAvg=&sMh_MotorDataOut.slIwFb;
  AnProc_Set(sMotorHandlerRun.pubADTagsCurr[sMotorHandlerRun.sCfgCurrents.d.ubLayoutW-1],&sAInDef);

#ifdef _HW_DC
  // if parallel bridge selected
  if(sMotorHandlerRun.flags.b.bParallelBr1Mode)
  {
	if (sMotorHandlerRun.flags.b.bDSPAdvance)
      sMh_PlcAdvancedWorks.flags.b.bEnMultiBrgImmCurr = TRUE ;

    // bridge 1 phase currents (same as above but just average DSP destination)
    sAInDef.uwDstIntImm=ANPROC_ADR_DISABLE;
    sAInDef.uwDstIntAvg=ANPROC_ADR_DISABLE;
    sAInDef.pvDstExtImm=NULL;
    sAInDef.pvDstExtAvg=NULL;
    
    // virtual bridge 1 U current
    sAInDef.sCal.uwOffst=sMotorHandlerRun.tCurBr1Cal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutU-1].uwOffst;
    sAInDef.sCal.swScale=sMotorHandlerRun.tCurBr1Cal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutU-1].swScale;
    if(sMh_PlcAdvancedWorks.flags.b.bEnMultiBrgImmCurr)
    {
        sAInDef.uwDstIntImm=FPGAIR_IFB_BR1_IU;
        sAInDef.pvDstExtImm=&sMh_MotorDataOut.slBr1IuFb_Imm;
    }
    sAInDef.uwDstIntAvg=FPGAIR_IFB_BR1_AU;
    sAInDef.pvDstExtAvg=&sMh_MotorDataOut.slBr1IuFb;
    AnProc_Set(sMotorHandlerRun.pubADTagsBr1Curr[sMotorHandlerRun.sCfgCurrents.d.ubLayoutU-1],&sAInDef);

    // virtual bridge 1 V current
    sAInDef.sCal.uwOffst=sMotorHandlerRun.tCurBr1Cal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutV-1].uwOffst;
    sAInDef.sCal.swScale=sMotorHandlerRun.tCurBr1Cal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutV-1].swScale;
    if(sMh_PlcAdvancedWorks.flags.b.bEnMultiBrgImmCurr)
    {
        sAInDef.uwDstIntImm=FPGAIR_IFB_BR1_IV;
        sAInDef.pvDstExtImm=&sMh_MotorDataOut.slBr1IvFb_Imm;
    }
    sAInDef.uwDstIntAvg=FPGAIR_IFB_BR1_AV;
    sAInDef.pvDstExtAvg=&sMh_MotorDataOut.slBr1IvFb;
    AnProc_Set(sMotorHandlerRun.pubADTagsBr1Curr[sMotorHandlerRun.sCfgCurrents.d.ubLayoutV-1],&sAInDef);

    // virtual bridge 1 W current
    sAInDef.sCal.uwOffst=sMotorHandlerRun.tCurBr1Cal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutW-1].uwOffst;
    sAInDef.sCal.swScale=sMotorHandlerRun.tCurBr1Cal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutW-1].swScale;
    if(sMh_PlcAdvancedWorks.flags.b.bEnMultiBrgImmCurr)
    {
        sAInDef.uwDstIntImm=FPGAIR_IFB_BR1_IW;
        sAInDef.pvDstExtImm=&sMh_MotorDataOut.slBr1IwFb_Imm;
    }
    sAInDef.uwDstIntAvg=FPGAIR_IFB_BR1_AW;
    sAInDef.pvDstExtAvg=&sMh_MotorDataOut.slBr1IwFb;
    AnProc_Set(sMotorHandlerRun.pubADTagsBr1Curr[sMotorHandlerRun.sCfgCurrents.d.ubLayoutW-1],&sAInDef);

    // if parallel bridge 2 selected
    if(sMotorHandlerRun.flags.b.bParallelBr2Mode)
    {
      // virtual bridge 2 U current
      sAInDef.sCal.uwOffst=sMotorHandlerRun.tCurBr2Cal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutU-1].uwOffst;
      sAInDef.sCal.swScale=sMotorHandlerRun.tCurBr2Cal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutU-1].swScale;
      if(sMh_PlcAdvancedWorks.flags.b.bEnMultiBrgImmCurr)
      {
        sAInDef.uwDstIntImm=FPGAIR_IFB_BR2_IU;
        sAInDef.pvDstExtImm=&sMh_MotorDataOut.slBr2IuFb_Imm;
      }
      sAInDef.uwDstIntAvg=FPGAIR_IFB_BR2_AU;
      sAInDef.pvDstExtAvg=&sMh_MotorDataOut.slBr2IuFb;
      AnProc_Set(sMotorHandlerRun.pubADTagsBr2Curr[sMotorHandlerRun.sCfgCurrents.d.ubLayoutU-1],&sAInDef);

      // virtual bridge 2 V current
      sAInDef.sCal.uwOffst=sMotorHandlerRun.tCurBr2Cal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutV-1].uwOffst;
      sAInDef.sCal.swScale=sMotorHandlerRun.tCurBr2Cal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutV-1].swScale;
      if(sMh_PlcAdvancedWorks.flags.b.bEnMultiBrgImmCurr)
      {
         sAInDef.uwDstIntImm=FPGAIR_IFB_BR2_IV;
         sAInDef.pvDstExtImm=&sMh_MotorDataOut.slBr2IvFb_Imm;
      }
      sAInDef.uwDstIntAvg=FPGAIR_IFB_BR2_AV;
      sAInDef.pvDstExtAvg=&sMh_MotorDataOut.slBr2IvFb;
      AnProc_Set(sMotorHandlerRun.pubADTagsBr2Curr[sMotorHandlerRun.sCfgCurrents.d.ubLayoutV-1],&sAInDef);

      // virtual bridge 2 W current
      sAInDef.sCal.uwOffst=sMotorHandlerRun.tCurBr2Cal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutW-1].uwOffst;
      sAInDef.sCal.swScale=sMotorHandlerRun.tCurBr2Cal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutW-1].swScale;
      if(sMh_PlcAdvancedWorks.flags.b.bEnMultiBrgImmCurr)
      {
        sAInDef.uwDstIntImm=FPGAIR_IFB_BR2_IW;
        sAInDef.pvDstExtImm=&sMh_MotorDataOut.slBr2IwFb_Imm;
      }
      sAInDef.uwDstIntAvg=FPGAIR_IFB_BR2_AW;
      sAInDef.pvDstExtAvg=&sMh_MotorDataOut.slBr2IwFb;
      AnProc_Set(sMotorHandlerRun.pubADTagsBr2Curr[sMotorHandlerRun.sCfgCurrents.d.ubLayoutW-1],&sAInDef);
    }
  }
#endif

  // input voltages

  // Vdc
  sAInDef.flScale=flRatioV_DC_PEAK;
  sAInDef.sCal.uwOffst=uwDCBusOffst;
  sAInDef.sCal.swScale=swDCBusScale/2;  // old ADC has a preshift just for the DCBUS module

  if(bVerifyDSPCustom)
    sAInDef.uwDstIntImm=FPGAIR_VDC_I;
  else
    sAInDef.uwDstIntImm=ANPROC_ADR_DISABLE;

  sAInDef.uwDstIntAvg=FPGAIR_VDC_A;
  sAInDef.pvDstExtImm=NULL;
  sAInDef.pvDstExtAvg=&sMh_MotorDataOut.swDcBusValue;

#ifndef _HW_DC
  if(!bExternalDcBus)
  {
    sAInDef.ubOpt=ANPROC_OF_AVG_CONT|ANPROC_OF_BLK_INST|ANPROC_OF_DST_SHORT|ANPROC_OF_UNSIGNED;
    sAInDef.ubNumSample=125;
    AnProc_Set(FPGA_ADTAGS_PWI_DCBUS,&sAInDef);
  }
  else
#endif
  {
    sAInDef.ubOpt=ANPROC_OF_AVG_CONT|ANPROC_OF_BLK_INST|ANPROC_OF_DST_SHORT|ANPROC_OF_UNSIGNED;
    sAInDef.ubNumSample=40;
    AnProc_Set(FPGA_ADTAGS_PWB_DCBUS,&sAInDef);
  }

  // Vac
#ifndef _HW_DC
  if(sMh_MotorDataOut.sPowerStageSts.b.bACInputDataValid)
  {
    sAInDef.ubOpt=ANPROC_OF_AVG_CONT|ANPROC_OF_BLK_INST|ANPROC_OF_DST_SHORT;
    sAInDef.ubNumSample=40;
    sAInDef.flScale=flRatioV_PSPL_PEAK;
    sAInDef.uwDstIntImm=ANPROC_ADR_DISABLE;
    sAInDef.uwDstIntAvg=ANPROC_ADR_DISABLE;
    sAInDef.pvDstExtImm=NULL;

    sAInDef.sCal.uwOffst=uwPSPlUOffst;
    sAInDef.sCal.swScale=swPSPlUScale;
    sAInDef.pvDstExtAvg=sMotorHandlerRun.pswACPU;
    if(bVerifyDSPCustom)
    {
      sAInDef.uwDstIntImm=FPGAIR_VAC_IRS;
      sAInDef.uwDstIntAvg=FPGAIR_VAC_ARS;
    }
    AnProc_Set(FPGA_ADTAGS_PWB_PSPLU,&sAInDef);
    
    sAInDef.sCal.uwOffst=uwPSPlVOffst;
    sAInDef.sCal.swScale=swPSPlVScale;
    sAInDef.pvDstExtAvg=sMotorHandlerRun.pswACPV;
    if(bVerifyDSPCustom)
    {
      sAInDef.uwDstIntImm=FPGAIR_VAC_IST;
      sAInDef.uwDstIntAvg=FPGAIR_VAC_AST;
    }
    AnProc_Set(FPGA_ADTAGS_PWB_PSPLV,&sAInDef);
  }
#endif

#ifdef _HW_CT
  // brake current
  if(bIBrakeValid)
  {
    sAInDef.ubOpt=ANPROC_OF_AVG_CONT|ANPROC_OF_BLK_INST|ANPROC_OF_DST_LONG;
    sAInDef.ubNumSample=125;
    sAInDef.flScale=sGlbPowerBoardParameters.sDif.flACVoltageScale;
    sAInDef.uwDstIntImm=ANPROC_ADR_DISABLE;
    sAInDef.uwDstIntAvg=ANPROC_ADR_DISABLE;
    
    sAInDef.sCal.uwOffst=sGlbPowerBoardParameters.sDif.sVoltageRS.uwOffst;
    sAInDef.sCal.swScale=sGlbPowerBoardParameters.sDif.sVoltageRS.swScale;
    sAInDef.pvDstExtImm=NULL;
    sAInDef.pvDstExtAvg=&sMh_MotorDataOut.slIBrake;
    AnProc_Set(FPGA_ADTAGS_PWB_PSPLU,&sAInDef);
  }
#endif

  // filtered Iq internal routing

  sAInDef.ubOpt=ANPROC_OF_AVG_NONE|ANPROC_OF_BLK_NONE|ANPROC_OF_DST_LONG;
  sAInDef.ubNumSample=1;
  sAInDef.flScale=flRatioI_EQ_RMS;
  sAInDef.sCal.uwOffst=FPGA_ADCDEF_GENERIC_OFF;
  sAInDef.sCal.swScale=FPGA_ADCDEF_GENERIC_MUL;
  sAInDef.uwDstIntImm=FPGAIR_IREF_FQ;
  sAInDef.uwDstIntAvg=ANPROC_ADR_DISABLE;
  sAInDef.pvDstExtImm=&sMh_MotorDataOut.slIqFltRef;
  sAInDef.pvDstExtAvg=NULL;
  AnProc_Set(FPGA_ADTAGS_VIRTUAL_IQR_FLT,&sAInDef);

  // LUT Sin & Cos
  
  sAInDef.ubOpt=ANPROC_OF_AVG_NONE|ANPROC_OF_BLK_NONE|ANPROC_OF_DST_LONG;
  sAInDef.ubNumSample=1;
  sAInDef.sCal.uwOffst=FPGA_ADCDEF_GENERIC_OFF;
  sAInDef.sCal.swScale=FPGA_ADCDEF_GENERIC_MUL;
  sAInDef.uwDstIntAvg=ANPROC_ADR_DISABLE;
  sAInDef.pvDstExtImm=NULL;
  sAInDef.pvDstExtAvg=NULL;

  sAInDef.uwDstIntImm=FPGAIR_K_SIN;
  AnProc_Set(FPGA_ADTAGS_LUT_SIN,&sAInDef);
  sAInDef.uwDstIntImm=FPGAIR_K_COS;
  AnProc_Set(FPGA_ADTAGS_LUT_COS,&sAInDef);

  // modulator output values

  sAInDef.ubNumSample=1;
  sAInDef.pvDstExtAvg=NULL;
  sAInDef.sCal.uwOffst=FPGA_ADCDEF_GENERIC_OFF;
  sAInDef.sCal.swScale=FPGA_ADCDEF_GENERIC_MUL;
  sAInDef.uwDstIntImm=ANPROC_ADR_DISABLE;
  sAInDef.uwDstIntAvg=ANPROC_ADR_DISABLE;

  // feedback Id and Iq
  if (sMotorHandlerRun.flags.b.bDSPAdvance)
  {
    sRGODef.ubOpt=ANPROC_OF_DST_SHORT;
    sMotorHandlerRun.flRatioI_EQ_RMS = flRatioI_EQ_RMS ;
    sRGODef.flScale=1.0;

    sRGODef.pvDst=&sMotorHandlerRun.swIdFb;
    AnProc_RGOutSet(FPGAIR_RGO_IFB_AD,&sRGODef);

    sRGODef.pvDst=&sMotorHandlerRun.swIqFb;
    AnProc_RGOutSet(FPGAIR_RGO_IFB_AQ,&sRGODef);
  }
  else
  {
    sRGODef.ubOpt=ANPROC_OF_DST_LONG;
    sRGODef.flScale=flRatioI_EQ_RMS;

    sRGODef.pvDst=&sMh_MotorDataOut.slIdFb;
    AnProc_RGOutSet(FPGAIR_RGO_IFB_AD,&sRGODef);

    sRGODef.pvDst=&sMh_MotorDataOut.slIqFb;
    AnProc_RGOutSet(FPGAIR_RGO_IFB_AQ,&sRGODef);
  }

  // output Vd and Vq
  sRGODef.ubOpt=ANPROC_OF_DST_SHORT;

  if (sMotorHandlerRun.flags.b.bDSPAdvance)
  {
    sRGODef.flScale=1.0;

    sRGODef.pvDst=&sMotorHandlerRun.swVdOut;
    AnProc_RGOutSet(FPGAIR_RGO_VOUT_D,&sRGODef);

    sRGODef.pvDst=&sMotorHandlerRun.swVqOut;
    AnProc_RGOutSet(FPGAIR_RGO_VOUT_Q,&sRGODef);
  }
  else
  {
    sRGODef.flScale=flRatioV_DC_PEAK;

    sRGODef.pvDst=&sMh_MotorDataOut.swVdOut;
    AnProc_RGOutSet(FPGAIR_RGO_VOUT_D,&sRGODef);

    sRGODef.pvDst=&sMh_MotorDataOut.swVqOut;
    AnProc_RGOutSet(FPGAIR_RGO_VOUT_Q,&sRGODef);
  }

  // output Vu and Vv
  sRGODef.flScale=-flRatioV_AC_PEAK;

  sRGODef.pvDst=&sMh_MotorDataOut.swVuOut;
  AnProc_RGOutSet(FPGAIR_RGO_VOUT_U,&sRGODef);

  sRGODef.pvDst=&sMh_MotorDataOut.swVvOut;
  AnProc_RGOutSet(FPGAIR_RGO_VOUT_V,&sRGODef);

#if defined(_CRS_DBG)
  if (sMotorHandlerRun.flags.b.bDSPAdvance)
  { // Dbg
    sRGODef.ubOpt=ANPROC_OF_DST_LONG;
    sRGODef.flScale=1.0;

    sRGODef.pvDst=&sMotorHandlerRun.slDspIntSts_D;
    AnProc_RGOutSet(FPGAIR_RGO_INTSTS_D,&sRGODef);

    sRGODef.pvDst=&sMotorHandlerRun.slDspIntSts_Q;
    AnProc_RGOutSet(FPGAIR_RGO_INTSTS_Q,&sRGODef);

    // VdcBus A2D value (max = 16384, not 32768)
    sRGODef.ubOpt=ANPROC_OF_DST_SHORT;
    sRGODef.flScale=1.0;
    sRGODef.pvDst=&sMh_MotorDataOut.swVdcOut_ADC;
    AnProc_RGOutSet(FPGAIR_RGO_VDC_OUTADC,&sRGODef);
  }
#endif

  // setup standard address for current channels min/max
  SetMinMaxChannels(SETMINMAX_RUNMODE, 0);

  // enable calibration at end of boot
  sMotorHandlerRun.flags.b.bCurrentCalibrFirstRun=TRUE;
  sMotorHandlerRun.flags.b.bCurrentCalibrResetTmr=TRUE;

    // limits: imposto i limiti e le soglie di fault overcurrent ed overvoltage                     
  sMh_MotorDataOut.sDriveLimit.slOverCurrent = sMotorHandlerRun.slMaxOverCurrent;
  FPGA_PWM_FLT_CURRLIMIT = UMCONV_CONVERT_32TO16(&sInternal2Fpga_I_RMS, sMh_MotorDataOut.sDriveLimit.slOverCurrent) ;

  sMh_MotorDataOut.sDriveLimit.swOverVoltage = sMotorHandlerRun.swMaxOverVoltage = swLocOverVoltage;
  FPGA_PWM_FLT_DCBUSLIMIT = UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK, (SLONG)sMh_MotorDataOut.sDriveLimit.swOverVoltage<<16);

  // Setup drive current limit (minimum between drive and motor)
  if((SLONG)(sGlbMotorParameters.flCurrentPeak * 10000.0) < sMh_MotorDataOut.sDriveLimit.slCurrentLimit)
    sMh_MotorDataOut.sDriveLimit.slCurrentLimit = (SLONG)(sGlbMotorParameters.flCurrentPeak * 10000.0) ;

  sMotorHandlerRun.sLocalCurrentLimit.slMax =  sMh_MotorDataOut.sDriveLimit.slCurrentLimit ;
  sMotorHandlerRun.sLocalCurrentLimit.slMin = -sMotorHandlerRun.sLocalCurrentLimit.slMax ;
  Add2LimitList(MH_LIMITLIST_IQ, &sMotorHandlerRun.sLocalCurrentLimit) ;
  Add2LimitList(MH_LIMITLIST_ID, &sMotorHandlerRun.sLocalCurrentLimit) ;
    
  // Setup user current limit (attivi in Torque Mode, disattivi se PID attivo)
  Add2LimitList(MH_LIMITLIST_IQ, &sMh_MotorDataParam.sIqLimit); // IqLimit in Torque mode
  Add2LimitList(MH_LIMITLIST_ID, &sMh_MotorDataParam.sIdLimit); // IdLimit in Torque mode
  
  // Setup plc current limit (attivi sempre)
  Add2LimitList(MH_LIMITLIST_IQ, &sMh_PlcWorks.sIqLimit);
  Add2LimitList(MH_LIMITLIST_ID, &sMh_PlcWorks.sIdLimit);
    
  // R & L setup  
  sMotorHandlerRun.swStatorR = (SWORD)(sGlbMotorParameters.flResistance * R_CONV_FACTOR) ;
  sMotorHandlerRun.swStatorL = (SWORD)(sGlbMotorParameters.flInductance * L_CONV_FACTOR) ;
    
  SetupILoopAutoGains(TRUE, Mh_GetActualPwmFrequency());
  sMotorHandlerRun.flags.b.bUpdateFpgaGains=TRUE;

  // DSPH setup
  // get compatibility level
  sMh_MotorDataOut.uwDSPCompLevel=FPGA_CPUH_INSRAM_CMPLEV;

  if(bVerifyDSPCustom)
  { // ***************** PLC custom DSP *****************
    BOOL bFail=FALSE;

    // load custom code
    if(DSPHLoad(hpubDSPCustomAddr, uwDSPCustomSize, FPGAIR_IFB_AW))
    {
      // and prepare outputs
      if(hpubDSPCustomDBAddr!=NULL)
      {
        void  * hpvPtr;
        void  * npvPtr;

        for(uwCt=0;uwCt<uwDSPCustomDBLength;uwCt++)
        {
          if(hpubDSPCustomDBAddr[uwCt].ubSize==2)
            sRGODef.ubOpt=ANPROC_OF_DST_SHORT;
          else if(hpubDSPCustomDBAddr[uwCt].ubSize==4)
            sRGODef.ubOpt=ANPROC_OF_DST_LONG;
          else
          {
            bFail=TRUE;
            break;
          }

          switch(hpubDSPCustomDBAddr[uwCt].ubTypeScale)
          {
            case LOCDSPDBSCALE_CUSTOM:
              sRGODef.flScale=hpubDSPCustomDBAddr[uwCt].fScale;
              break;

            case LOCDSPDBSCALE_ARMS:
              sRGODef.flScale=flRatioI_RMS;
              break;

            case LOCDSPDBSCALE_APK:
              sRGODef.flScale=flRatioI_PEAK;
              break;

            case LOCDSPDBSCALE_VPK:
              sRGODef.flScale=flRatioV_DC_PEAK;
              break;

            default:
              sRGODef.flScale=1.0;
          }

          // translate into pointer
          hpvPtr=PlcIECDBEntrySearch(0,MH_DSPDB_IEC_WS,hpubDSPCustomDBAddr[uwCt].ubIntIECDest);

          // traslate to near, if equal (and not NULL) then it's near thus valid
          npvPtr=(void *)hpvPtr;
          if((ULONG)(void  *)npvPtr == (ULONG)hpvPtr && hpvPtr)
          {
            sRGODef.pvDst=npvPtr;
            AnProc_RGOutSet(hpubDSPCustomDBAddr[uwCt].ubSrcTag+ANPROC_RGO_TAGOFFSET,&sRGODef);
          }
          else
          {
            bFail=TRUE;
            break;
          }
        }
      }
    }
    else
      bFail=TRUE;

    // custom DSP failure, signal and exit
    if(bFail)
    {
      SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_PLC_OVERTIME, SYSTEMALARMS_SUBCODE_PLC_INVALID_DSP, FALSE) ;
      return FALSE;
    }

    sMotorHandlerRun.flags.b.bDSPCustomCode=TRUE;
    sMh_MotorDataOut.ubDSPIsCustom = 1;
    // ***************** PLC custom DSP *****************
  }
  else
  {	// **************** FW Standard  DSP ****************
#ifdef _HW_DC
    if(sMotorHandlerRun.flags.b.bParallelBr1Mode)
    {
      if(sMotorHandlerRun.flags.b.bParallelBr2Mode)
      { // ********** FW Standard DSP  (3 bridges) **********
        sMh_MotorDataOut.ubDSPIsCustom=0;
        assert(DSPHLoad(ILoopParBr2, (UWORD)ILoopParBr2_length, FPGAIR_IFB_AW));
      }
      else
      { // *************** FW DSP (2 bridges) ***************
        if(sMotorHandlerRun.flags.b.bDSPAdvance)
        {  // ****************** Advanced DSP ******************
           sMh_MotorDataOut.ubDSPIsCustom=2;
           assert(DSPHLoad(ILoopParBr1Adv, (UWORD)ILoopParBr1Adv_length, FPGAIR_IFB_AW));
        }
        else
        { // ****************** Standard DSP ******************
          sMh_MotorDataOut.ubDSPIsCustom=0;
          assert(DSPHLoad(ILoopParBr1, (UWORD)ILoopParBr1_length, FPGAIR_IFB_AW));
        }
      }
    }
    else
#endif // _hwdc
    { // *********** FW Standard DSP (1 bridge) ***********
      if(sMotorHandlerRun.flags.b.bDSPAdvance)
      { // ****************** Advanced DSP ******************
    	sMh_MotorDataOut.ubDSPIsCustom=2;
    	assert(DSPHLoad(ILoopAdvanced, (UWORD)ILoopAdvanced_length, FPGAIR_IFB_AW));
      }
      else
      {  // ****************** Standard DSP ******************
         sMh_MotorDataOut.ubDSPIsCustom=0;
    	 assert(DSPHLoad(ILoopStandard, (UWORD)ILoopStandard_length, FPGAIR_IFB_AW));
      }
    }
  }

  {
    HPUWORD hpuwDat=(HPUWORD)RGAD_CPUH_DATRAM_BASEADDR;
    UWORD uwCt;

    for(uwCt=0;uwCt<100;uwCt++)
      if(FPGA_DSPH_INSRAML_HALTACK)
        break;

    // reset all register RAM area
    for(uwCt=0; uwCt<RGAD_CPUH_DATRAM_SIZE/sizeof(UWORD); uwCt+=sizeof(UWORD))
        *hpuwDat++=0x0000;
  }

  if(!sMotorHandlerRun.flags.b.bDSPCustomCode)
  { // setup constants
    FPGA_CPUH_DRAM_WR_SW(FPGAIR_C_ZERO,   0x0000); // = zero
    FPGA_CPUH_DRAM_WR_SW(FPGAIR_C_1DSQT3, 0x49E7); // = 18919 = 32768 * 1/sqrt(3)
    FPGA_CPUH_DRAM_WR_SW(FPGAIR_C_SQT3D2, 0x6EDA); // = 28378 = 32768 * sqrt(3)/2
    FPGA_CPUH_DRAM_WR_SW(FPGAIR_C_1D2,    0x4000); // = 16384 = 32768 / 2
    FPGA_CPUH_DRAM_WR_SW(FPGAIR_C_1D3,    0x2AAB); // = 10923 = 32768 / 3
    FPGA_CPUH_DRAM_WR_SW(FPGAIR_C_1D4,    0x2000); // = 8192  = 32768 / 4
    FPGA_CPUH_DRAM_WR_SW(FPGAIR_C_1D8,    0x1000); // = 4096  = 32768 / 8
    FPGA_CPUH_DRAM_WR_SW(FPGAIR_C_7D8,    0x7000); // = 28672 = 32768 * 7/8
    FPGA_CPUH_DRAM_WR_SW(FPGAIR_C_1D125,  0x0106); // = 262   = 32768 / 125

    // set user Vdc setup feature (only at boot) and initialize user set Vdcbus
    if (sMotorHandlerRun.flags.b.bDSPAdvance)
    {
      if(sMh_PlcAdvancedWorks.flags.b.bUserVdcBusSet)
        sMotorHandlerRun.swUsrVdcSet = USRVDC_SET_FIX;
      else
        sMotorHandlerRun.swUsrVdcSet = USRVDC_SET_ADC;

      if (sMh_PlcAdvancedWorks.swVdcbusSet <= 0)
      {
#if defined(_HW_AXS_DAYCO22KW)
	    sMh_PlcAdvancedWorks.swVdcbusSet = 480 ; // zero or negative value not allowed: set default at 48.0V
#else
	    sMh_PlcAdvancedWorks.swVdcbusSet = 6000 ; // zero or negative value not allowed: set default at 600.0V
#endif
      }

      sMotorHandlerRun.swUsrVdcVal = (SWORD)(UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK,(SLONG)sMh_PlcAdvancedWorks.swVdcbusSet<<16)) ;

      // set user DSP integral status initialization value feature (only at boot)
      sMotorHandlerRun.flags.b.bDspIntegralSet = sMh_PlcAdvancedWorks.flags2.b.bDspIntegralSet ;
      sMotorHandlerRun.swDspIntegralVal_D = 0 ;
      sMotorHandlerRun.swDspIntegralVal_Q = 0 ;
    }
    else
    {
      sMotorHandlerRun.swUsrVdcSet = USRVDC_SET_ADC;
      sMotorHandlerRun.swUsrVdcVal = 0;
    }
  }

  // re-enable DSPH
  FPGA_DSPH_SET_HALT = FALSE;

  sMotorHandlerRun.sbFltUpdCnt=-1;
  sMotorHandlerRun.uwFltCRC=0;
  sMotorHandlerRun.sbFltPlcNumValid=-1;

  LiveParametersCheck();

#ifdef _HW_DC
  // if parallel bridge selected
  if(sMotorHandlerRun.flags.b.bParallelBr1Mode)
  {
    FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_BRIDGE1_OVERCURR, 1);
    FPGA_BRIDGE1_ENABLE = TRUE;
    if(sMotorHandlerRun.flags.b.bParallelBr2Mode)
    {
      FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_BRIDGE2_OVERCURR, 1);
      FPGA_BRIDGE2_ENABLE = TRUE;
    }
  }
#endif

  // enable power outputs
  FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_ENABLE_OUTPADS, 1);
    
  /* Brake parameters (if used) */
  if(bBrakeDrivePresent)
  {
    FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_BRAKE_ACTIVE_HIGH,  bBrakeDrivePolarity);
    FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_SETUP_BRAKE_ENABLE, 1);  /* enable the brake*/
    sMotorHandlerRun.flags.b.bBrakeInstalled = TRUE;
  }
    
  // enable log signals
  FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_ENABLE_HOLD_SIGNALS, 1);

    // reset faults at startup
  Mh_ResetPwrModFaults();

  sMh_MotorDataOut.slIdFb = 0L ; /* RMS */
  sMh_MotorDataOut.slIqFb = 0L ; /* RMS */
  sMh_MotorDataOut.slIuFb = 0L ; /* PEAK */
  sMh_MotorDataOut.slIvFb = 0L ; /* PEAK */

  sMh_MotorDataOut.sPowerStageSts.b.bReferenceEnabled = FALSE ;

  sMh_PlcAdvancedWorks.flags.b.bDtCompCurrentSign  = FPGA_REG16_GETBIT(FPGA_PWM_SET, FPGA_PWM_B_DTCOMP_CUR_SIGN);
  sMh_PlcAdvancedWorks.flags.b.bDtCompCurrentSign  = FPGA_REG16_GETBIT(FPGA_PWM_SET, FPGA_PWM_B_DTCOMP_CUR_SIGN);
  sMh_PlcAdvancedWorks.uwDtCompSet                 = FPGA_PWM_DTCOMP_SET;
  sMh_PlcAdvancedWorks.uwDeadTime                  = FPGA_PWM_DEADTIME;
  sMh_PlcAdvancedWorks.uwHoldTime                  = FPGA_PWM_HOLDTIME;
  sMh_PlcAdvancedWorks.uwBlankTime                 = FPGA_PWM_BLANKTIME;
  sMh_PlcAdvancedWorks.ubSetPwmFrequency           = Mh_GetActualPwmFrequency();

  // Deflux
  if(sMh_MotorDataParam.flags.b.bEnableDeflux)
  {
    DflxInit(sMh_MotorDataOut.sDriveLimit.slCurrentLimit) ;
    Add2LimitList(MH_LIMITLIST_IQ, &sDflx_Out.sDefluxIqLimit);
    sMotorHandlerRun.flags.b.bDefluxEnabled=TRUE;
    sMotorHandlerRun.flags.b.bDflxWithIqFltRef = sMh_MotorDataParam.flags.b.bDflxWithIqFltRef ;
  }

  return TaskSched_AddRTTask(&Mh_MotorDataFromFpga8KHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0) ; /* install 8KHz function */
}


//***************************************************************************
//
static BOOL Mh_MotorDataToFpgaInit(void)
{
  sMotorHandlerRun.slIuFb_1  = 0L ;
  sMotorHandlerRun.slIvFb_1  = 0L ;

  /* install bg function */
  if(!TaskSched_AddBackgroundTask(&MotorDataBkGd))
   return FALSE ;


  /* install bg deflux */
  if(!TaskSched_AddBackgroundTask(&DefluxWrapper))
    return FALSE ;

  if(bSysStatLockedByBootError)
    return TRUE;

  return TaskSched_AddRTTask(&Mh_MotorDataToFpga8KHz, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0) ; /* install 8KHz function */
}

//***************************************************************************
// Reset-only parameters check

static BOOL ResetOnlyParametersCheck(void)
{
  BOOL bValid=TRUE;

  switch(sMh_MotorDataParam.uwUserBridgeLayout)
  {
    case MH_USRBRIDGELAYOUT_UVW:
    case MH_USRBRIDGELAYOUT_UWV:
    case MH_USRBRIDGELAYOUT_VUW:
    case MH_USRBRIDGELAYOUT_VWU:
    case MH_USRBRIDGELAYOUT_WUV:
    case MH_USRBRIDGELAYOUT_WVU:
      ParChk_ResetValueError(PARCC_MH_USERBRIDGELAYOUT);
      break;

    default:
      ParChk_SignalValueError(PARCC_MH_USERBRIDGELAYOUT);
      bValid=FALSE;
  }

  return bValid;
}

//***************************************************************************
// Live parameters check and activation

static BOOL LiveParametersCheck(void)
{
    BOOL bValid=TRUE;
    SWORD swVBrakeLow,swVBrakeHigh;
    SLONG slOverCurrent;

    // brake threshold
//    _atomic_(0);
    swVBrakeLow=sMh_MotorDataParam.swVBrakeLow;
    swVBrakeHigh=sMh_MotorDataParam.swVBrakeHigh;
//    _endatomic_();

    if(swVBrakeLow < 0 || swVBrakeHigh < 0 || swVBrakeLow > swVBrakeHigh)
    {
        ParChk_SignalValueError(PARCC_MH_BRAKEVOLTAGERANGE);
        bValid=FALSE;
        swVBrakeLow  = DEFAULT_BRAKELOW*10;
        swVBrakeHigh = DEFAULT_BRAKEHIGH*10;
    }
    else
    {
        ParChk_ResetValueError(PARCC_MH_BRAKEVOLTAGERANGE);
        if(swVBrakeLow==0 || swVBrakeHigh==0)
        {
                // setup default values
            swVBrakeLow  = DEFAULT_BRAKELOW*10;
            swVBrakeHigh = DEFAULT_BRAKEHIGH*10;
        }
    }

    sMh_MotorDataOut.swActiveVBrakeLow  = swVBrakeLow;
    sMh_MotorDataOut.swActiveVBrakeHigh = swVBrakeHigh;

    swVBrakeLow  = (SWORD)UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK, (SLONG)swVBrakeLow<<16);
    swVBrakeHigh = (SWORD)UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK, (SLONG)swVBrakeHigh<<16);

    if(swVBrakeLow>sMotorHandlerRun.swVBrakeLow)
    {
//        _atomic_(0);
        if(!bBrakeForcedOn)
        {
            FPGA_PWM_BRAKE_HIGH=swVBrakeHigh;
            FPGA_PWM_BRAKE_LOW=swVBrakeLow;
        }
//        _endatomic_();
    }
    else
    {
//        _atomic_(0);
        if(!bBrakeForcedOn)
        {
            FPGA_PWM_BRAKE_LOW=swVBrakeLow;
            FPGA_PWM_BRAKE_HIGH=swVBrakeHigh;
        }
//        _endatomic_();
    }

    sMotorHandlerRun.swVBrakeLow  = swVBrakeLow;
    sMotorHandlerRun.swVBrakeHigh = swVBrakeHigh;

    // current threshold (cyclically applied by thermal model)
//    _atomic_(0);
    slOverCurrent=sMh_MotorDataParam.slOverCurrentThreshold;
//    _endatomic_();
    if(slOverCurrent<0)
    {
        ParChk_SignalValueError(PARCC_MH_OVERCURRENT);
        bValid=FALSE;
        sMh_MotorDataOut.sDriveLimit.slOverCurrent = sMotorHandlerRun.slMaxOverCurrent ;
    }
    else
    {
        if(slOverCurrent == 0)
                // setup default values
            slOverCurrent = sMotorHandlerRun.slMaxOverCurrent;

            // take lower
        if(slOverCurrent > sMotorHandlerRun.slMaxOverCurrent)
            sMh_MotorDataOut.sDriveLimit.slOverCurrent = sMotorHandlerRun.slMaxOverCurrent ;
        else
            sMh_MotorDataOut.sDriveLimit.slOverCurrent = slOverCurrent ;

        ParChk_ResetValueError(PARCC_MH_OVERCURRENT);
    }

    // voltage threshold
    swVBrakeLow=sMh_MotorDataParam.swOverVoltageThreshold;
    if(swVBrakeLow<0)
    {
        ParChk_SignalValueError(PARCC_MH_OVERVOLTAGE);
        bValid=FALSE;
        sMh_MotorDataOut.sDriveLimit.swOverVoltage = sMotorHandlerRun.swMaxOverVoltage ;
    }
    else
    {
        if(swVBrakeLow == 0)
                // setup default values
            swVBrakeLow = sMotorHandlerRun.swMaxOverVoltage;

            // take lower
        if (swVBrakeLow > sMotorHandlerRun.swMaxOverVoltage)
            sMh_MotorDataOut.sDriveLimit.swOverVoltage = sMotorHandlerRun.swMaxOverVoltage ;
        else
            sMh_MotorDataOut.sDriveLimit.swOverVoltage = swVBrakeLow ;

        ParChk_ResetValueError(PARCC_MH_OVERVOLTAGE);
    }

    FPGA_PWM_FLT_DCBUSLIMIT = UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK, (SLONG)sMh_MotorDataOut.sDriveLimit.swOverVoltage<<16);

    // parallel bridge 2 must be enabled when requested parallel bridge 3
#ifdef _HW_DC
    if(sMh_MotorDataParam.flags.b.bEnParallelBr2 && !sMh_MotorDataParam.flags.b.bEnParallelBr1)
        ParChk_SignalValueError(PARCC_MH_BRIDGEPARMISMATCH);
    else
        ParChk_ResetValueError(PARCC_MH_BRIDGEPARMISMATCH);
#endif

    // get actual DSP work load
    sMh_MotorDataOut.ubDSPLoad=(UBYTE)((FPGA_CPUH_INSRAM_LEN&FPGA_DSPH_INSRAML_TIMEMASK)*100/FPGA_CPUH_DSPMAXCYLE);

    return bValid;
}

//***************************************************************************
// Setup automatic current loop gains

static BOOL SetupILoopAutoGains(BOOL bFirstRun, UBYTE ubFrequencyRequest)
{
  FLOAT flModKi,flModKp,flDiv=0.0;
  MH_DSPPARS sLocDSPPars;

    // calculate automatic gains based on motor data and switching frequency
  switch(ubFrequencyRequest)
  {
    case MH_PWMFREQ_0KHZ: // only conduction losses (no commutation losses)
    case MH_PWMFREQ_2KHZ:
        flDiv=8.0;
        break;

    case MH_PWMFREQ_4KHZ:
        flDiv=4.0;
        break;

    case MH_PWMFREQ_8KHZ:
        flDiv=2.0;
        break;

    case MH_PWMFREQ_16KHZ:
        flDiv=1.0;
        break;

    case MH_PWMFREQ_32KHZ:
        flDiv=0.5;
        break;

    case MH_PWMFREQ_64KHZ:
        flDiv=0.25;
        break;

    default:
        assert(FALSE);
  }

  flModKi = FPGA_KI_CONV_FACTOR / flDiv * (FLOAT)sMotorHandlerRun.swStatorL / L_CONV_FACTOR ;
  flModKp = FPGA_KP_CONV_FACTOR / flDiv * (FLOAT)sMotorHandlerRun.swStatorL / L_CONV_FACTOR ;

  atomic_write(&sMh_MotorDataOut.flAutoModKi, &flModKi, sizeof(FLOAT));
  atomic_write(&sMh_MotorDataOut.flAutoModKp, &flModKp, sizeof(FLOAT));

    // if changes then read back them atomically, avoid many irq disable and enable
  if(flModKi!=sMh_MotorDataParam.flModKi || flModKp!=sMh_MotorDataParam.flModKp)
  {
    DISABLE_IRQ();
    flModKi=sMh_MotorDataParam.flModKi;
    flModKp=sMh_MotorDataParam.flModKp;
    RESTORE_IRQ();
  }

    // if both zero then user select auto gains
  if ((flModKi == 0.0) && (flModKp == 0.0))
  {
    flModKi = sMh_MotorDataOut.flAutoModKi ;
    flModKp = sMh_MotorDataOut.flAutoModKp ;
  }


  if ((sMh_PlcAdvancedWorks.sDspPar.swKi != 0) && (sMh_PlcAdvancedWorks.sDspPar.swKp != 0))
  { // user dsp gain value override (in fpga scale)
	sLocDSPPars.swKi = sMh_PlcAdvancedWorks.sDspPar.swKi ;
	sLocDSPPars.swKp = sMh_PlcAdvancedWorks.sDspPar.swKp ;
	sLocDSPPars.swIOutShift = sMh_PlcAdvancedWorks.sDspPar.swIOutShift ;
	sLocDSPPars.uwModOutLimit = sMh_PlcAdvancedWorks.sDspPar.uwModOutLimit ;

	atomic_write(&sMh_MotorDataOut.sDSPPar, &sLocDSPPars, sizeof(MH_DSPPARS));

	// set run vars to zero in order to force the parameters update when override is disabled
	sMotorHandlerRun.flModKi = 0.0 ;
	sMotorHandlerRun.flModKp = 0.0 ;

	return TRUE ;
  }
  else
  { // compute output limiter from overvoltage threshold
	if ((BOOL)sMh_PlcAdvancedWorks.swDspOutLimit)
	{ // user value override: uwDspOutLimit is in uC iu (1e-1V)
	  sLocDSPPars.uwModOutLimit = UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK, (SLONG)sMh_PlcAdvancedWorks.swDspOutLimit <<16) ;
	  // since uwModOutLimit can be only [0; 32767], so the value is clipped to the max value allowed
	  if (sLocDSPPars.uwModOutLimit > 32767)
		sLocDSPPars.uwModOutLimit = 32767 ;
	}
	else
	  sLocDSPPars.uwModOutLimit = UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK, (SLONG)sMh_MotorDataOut.sDriveLimit.swOverVoltage<<16) ;

	  // if changes from actual set or first run
	  if((sMotorHandlerRun.flModKi != flModKi) || (sMotorHandlerRun.flModKp != flModKp) ||
		 (sMh_MotorDataOut.sDSPPar.uwModOutLimit != sLocDSPPars.uwModOutLimit) || bFirstRun)
	  {
		sMotorHandlerRun.flModKi = flModKi ;
		sMotorHandlerRun.flModKp = flModKp ;

		NormalizeTorqueLoopGains(flModKp, flModKi, &sLocDSPPars) ;

		atomic_write(&sMh_MotorDataOut.sDSPPar, &sLocDSPPars, sizeof(MH_DSPPARS));

		return TRUE;
	  }
	  else
		return FALSE;
  }
}

//***************************************************************************
// Manual current offset adjustement

SWORD Mh_ManualCurrentOffset(SLONG slIuAdj, SLONG slIvAdj, SLONG slIwAdj)
{
       // check right drive state
  if(!Os_IsInBackground() && !PlcIsInSlowTask())
      return MH_CURCALIB_FORBIDDEN;

#ifndef _HW_DC
    // setup calibration
  Os_BeginCriticalSection(OS_CRITSECT_GLOBAL);

  if(sMotorHandlerRun.flags.b.bCurrentCalibrEnable)
  {
    Os_EndCriticalSection(OS_CRITSECT_GLOBAL);
    return MH_CURCALIB_INPROGRESS;
  }

  sMotorHandlerRun.flags.b.bCurrentCalibrEnable=TRUE;
  sMotorHandlerRun.flags.b.bCurrentCalibrUpdOnly=TRUE;
  sMotorHandlerRun.flags.b.bCurrentCalibrAbort=FALSE;

  slIuAdj=UMCONV_CONVERT_32TO16(&sInternal2Fpga_I_PEAK, slIuAdj);
  slIvAdj=UMCONV_CONVERT_32TO16(&sInternal2Fpga_I_PEAK, slIvAdj);
  slIwAdj=UMCONV_CONVERT_32TO16(&sInternal2Fpga_I_PEAK, slIwAdj);

  CurrentSetOffsets(slIuAdj, slIvAdj, slIwAdj, sMotorHandlerRun.tCurCal, sMotorHandlerRun.pubADTagsCurr, &sMh_MotorDataOut.sB0Offsets);

  sMotorHandlerRun.flags.b.bCurrentCalibrUpdOnly=FALSE;
  sMotorHandlerRun.flags.b.bCurrentCalibrEnable=FALSE;

  Os_EndCriticalSection(OS_CRITSECT_GLOBAL);

  if(sMotorHandlerRun.flags.b.bCurrentCalibrAbort)
    return MH_CURCALIB_ABORTED;
  else
    return MH_CURCALIB_READY;

#else
   return MH_CURCALIB_FORBIDDEN;
#endif
}

//***************************************************************************
// Add adjustment to actual offset

static void CurrentSetOffsets(SLONG slIuAdj, SLONG slIvAdj, SLONG slIwAdj, HWPRMS_AD_SETTINGS * sCal, UBYTE * sTags, MH_BRIDGEOFFSETS * sOutDiag)
{
    // calculate offset for all currents
  sCal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutU-1].uwOffst+=OFFSET_DELTA(slIuAdj, \
                                                    sCal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutU-1].swScale); 

  sCal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutV-1].uwOffst+=OFFSET_DELTA(slIvAdj, \
                                                    sCal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutV-1].swScale); 

  sCal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutW-1].uwOffst+=OFFSET_DELTA(slIwAdj, \
                                                    sCal[sMotorHandlerRun.sCfgCurrents.d.ubLayoutW-1].swScale); 

      // adjust offset of the true currents reading
  AnProc_AdjustOffset(sTags[0], sCal[0].uwOffst);
  AnProc_AdjustOffset(sTags[1], sCal[1].uwOffst);
  AnProc_AdjustOffset(sTags[2], sCal[2].uwOffst);

      // request analog config reload, prevent task switching just for performance reason
  Os_BeginCriticalSection(OS_CRITSECT_GLOBAL);
  AnProc_ReloadOffsets();
  Os_EndCriticalSection(OS_CRITSECT_GLOBAL);

      // setup output diagnostics value (deviation from theoretical offset)
  if(sOutDiag)
  {    
    sOutDiag->swIU=(SWORD)(((FLOAT)sCal[0].uwOffst-(FLOAT)FPGA_ADCDEF_PWI_II_OFF)/32768.0*1000.0);
    sOutDiag->swIV=(SWORD)(((FLOAT)sCal[1].uwOffst-(FLOAT)FPGA_ADCDEF_PWI_II_OFF)/32768.0*1000.0);
    sOutDiag->swIW=(SWORD)(((FLOAT)sCal[2].uwOffst-(FLOAT)FPGA_ADCDEF_PWI_II_OFF)/32768.0*1000.0);
  }
}

//***************************************************************************
// Execute current sampling and re-calc offsets

static void ComputeISenseOffsets(void)
{
    if(sMotorHandlerRun.flags.b.bCurrentCalibrEnable && !sMotorHandlerRun.flags.b.bCurrentCalibrUpdOnly)
    {
        if(sMotorHandlerRun.flags.b.bCurrentCalibrAbort)
        {
            SetMinMaxChannels(SETMINMAX_RUNMODE, 0);
            sMotorHandlerRun.flags.b.bCurrentCalibrRTSum=FALSE;
            sMotorHandlerRun.flags.b.bCurrentCalibrEnable=FALSE;
            return;
        }
        
        if(sMotorHandlerRun.uwCalibrCounter<sMotorHandlerRun.uwCalibrSamplesRequired)
            return;

            // sum executed, stop rt task
        sMotorHandlerRun.flags.b.bCurrentCalibrRTSum=FALSE;

            // check for data validity
        if(sMotorHandlerRun.flags.b.bCurrentCalibrAbort)
        {
            SetMinMaxChannels(SETMINMAX_RUNMODE, 0);
            sMotorHandlerRun.flags.b.bCurrentCalibrEnable=FALSE;
            return;
        }

            // calc step
        switch(sMotorHandlerRun.ubCalibrStep)
        {
            case MHRT_CALSTEP_B0_AVG:
                CurrentSetOffsets((SLONG)((FLOAT)sMotorHandlerRun.slCalibrIuSum/(FLOAT)sMotorHandlerRun.uwCalibrCounter/2.0), \
                                  (SLONG)((FLOAT)sMotorHandlerRun.slCalibrIvSum/(FLOAT)sMotorHandlerRun.uwCalibrCounter/2.0), \
                                  (SLONG)((FLOAT)sMotorHandlerRun.slCalibrIwSum/(FLOAT)sMotorHandlerRun.uwCalibrCounter/2.0), \
                                  sMotorHandlerRun.tCurCal, sMotorHandlerRun.pubADTagsCurr, &sMh_MotorDataOut.sB0Offsets);
                break;

#ifdef _HW_DC
            case MHRT_CALSTEP_B1_AVG:
                CurrentSetOffsets((SLONG)((FLOAT)sMotorHandlerRun.slCalibrIuSum/(FLOAT)sMotorHandlerRun.uwCalibrCounter/2.0), \
                                  (SLONG)((FLOAT)sMotorHandlerRun.slCalibrIvSum/(FLOAT)sMotorHandlerRun.uwCalibrCounter/2.0), \
                                  (SLONG)((FLOAT)sMotorHandlerRun.slCalibrIwSum/(FLOAT)sMotorHandlerRun.uwCalibrCounter/2.0), \
                                  sMotorHandlerRun.tCurBr1Cal, sMotorHandlerRun.pubADTagsBr1Curr, &sMh_MotorDataOut.sB1Offsets);
                break;

            case MHRT_CALSTEP_B2_AVG:
                CurrentSetOffsets((SLONG)((FLOAT)sMotorHandlerRun.slCalibrIuSum/(FLOAT)sMotorHandlerRun.uwCalibrCounter/2.0), \
                                  (SLONG)((FLOAT)sMotorHandlerRun.slCalibrIvSum/(FLOAT)sMotorHandlerRun.uwCalibrCounter/2.0), \
                                  (SLONG)((FLOAT)sMotorHandlerRun.slCalibrIwSum/(FLOAT)sMotorHandlerRun.uwCalibrCounter/2.0), \
                                  sMotorHandlerRun.tCurBr2Cal, sMotorHandlerRun.pubADTagsBr2Curr, &sMh_MotorDataOut.sB2Offsets);
                break;
#endif

            case MHRT_CALSTEP_B0_MIN:
                sMh_MotorDataOut.sB0IpkpkMinOffsets.swIU = (SWORD)((FLOAT)sMotorHandlerRun.slCalibrIuSum / sMotorHandlerRun.uwCalibrCounter) ;
                sMh_MotorDataOut.sB0IpkpkMinOffsets.swIV = (SWORD)((FLOAT)sMotorHandlerRun.slCalibrIvSum / sMotorHandlerRun.uwCalibrCounter) ;
                sMh_MotorDataOut.sB0IpkpkMinOffsets.swIW = (SWORD)((FLOAT)sMotorHandlerRun.slCalibrIwSum / sMotorHandlerRun.uwCalibrCounter) ;
                break;

            case MHRT_CALSTEP_B0_MAX:
                sMh_MotorDataOut.sB0IpkpkMaxOffsets.swIU = (SWORD)((FLOAT)sMotorHandlerRun.slCalibrIuSum / sMotorHandlerRun.uwCalibrCounter) ;
                sMh_MotorDataOut.sB0IpkpkMaxOffsets.swIV = (SWORD)((FLOAT)sMotorHandlerRun.slCalibrIvSum / sMotorHandlerRun.uwCalibrCounter) ;
                sMh_MotorDataOut.sB0IpkpkMaxOffsets.swIW = (SWORD)((FLOAT)sMotorHandlerRun.slCalibrIwSum / sMotorHandlerRun.uwCalibrCounter) ;
                break;

            default:
                assert(FALSE);
        }

            // reset status var for next step
        sMotorHandlerRun.slCalibrIuSum=0l;
        sMotorHandlerRun.slCalibrIvSum=0l;
        sMotorHandlerRun.slCalibrIwSum=0l;
        sMotorHandlerRun.uwCalibrCounter=0;

        switch(sMotorHandlerRun.ubCalibrStep)
        {
            case MHRT_CALSTEP_B0_AVG:
#ifdef _HW_DC
                if(sMotorHandlerRun.flags.b.bParallelBr1Mode)
                {
                    sMotorHandlerRun.ubCalibrStep=MHRT_CALSTEP_B1_AVG;
                    SetMinMaxChannels(SETMINMAX_CALIBRATION, 1);
                }
                else
#endif
                    sMotorHandlerRun.ubCalibrStep=MHRT_CALSTEP_B0_MIN;
                sMotorHandlerRun.flags.b.bCurrentCalibrRTSum=TRUE;
                break;

#ifdef _HW_DC
            case MHRT_CALSTEP_B1_AVG:
                if(sMotorHandlerRun.flags.b.bParallelBr2Mode)
                {
                    sMotorHandlerRun.ubCalibrStep=MHRT_CALSTEP_B2_AVG;
                    SetMinMaxChannels(SETMINMAX_CALIBRATION, 2);
                }
                else
                {
                    SetMinMaxChannels(SETMINMAX_CALIBRATION, 0);
                    sMotorHandlerRun.ubCalibrStep=MHRT_CALSTEP_B0_MIN;
                }
                sMotorHandlerRun.flags.b.bCurrentCalibrRTSum=TRUE;
                break;

            case MHRT_CALSTEP_B2_AVG:
                SetMinMaxChannels(SETMINMAX_CALIBRATION, 0);
                sMotorHandlerRun.ubCalibrStep=MHRT_CALSTEP_B0_MIN;
                sMotorHandlerRun.flags.b.bCurrentCalibrRTSum=TRUE;
                break;
#endif

            case MHRT_CALSTEP_B0_MIN:
                sMotorHandlerRun.ubCalibrStep=MHRT_CALSTEP_B0_MAX;
                sMotorHandlerRun.flags.b.bCurrentCalibrRTSum=TRUE;
                break;

            case MHRT_CALSTEP_B0_MAX:
                SetMinMaxChannels(SETMINMAX_RUNMODE, 0);
                sMotorHandlerRun.flags.b.bCurrentCalibrFirstRun=FALSE;
                sMotorHandlerRun.flags.b.bCurrentCalibrEnable=FALSE;
                break;

            default:
                assert(FALSE);
        }
    }    
}

//***************************************************************************
// set min/max channels module for run mode or calibration mode

static void SetMinMaxChannels(BOOL bRunMode, UWORD uwChannel)
{
    if(bRunMode)
    {
        FPGA_CURCHMINMAX_ADDR_IUIV = ((FPGAIR_IFB_IU)<<8)|(FPGAIR_IFB_IV);
        FPGA_CURCHMINMAX_ADDR_IW   = FPGAIR_IFB_IW;
    }
    else
#ifdef _HW_DC
        if(uwChannel==2)
        {
            FPGA_CURCHMINMAX_ADDR_IUIV = ((FPGAIR_IFB_BR2_AU)<<8)|(FPGAIR_IFB_BR2_AV);
            FPGA_CURCHMINMAX_ADDR_IW   = FPGAIR_IFB_BR2_AW;
        }
        else if(uwChannel==1)
        {
            FPGA_CURCHMINMAX_ADDR_IUIV = ((FPGAIR_IFB_BR1_AU)<<8)|(FPGAIR_IFB_BR1_AV);
            FPGA_CURCHMINMAX_ADDR_IW   = FPGAIR_IFB_BR1_AW;
        }
        else
#endif
        {
            FPGA_CURCHMINMAX_ADDR_IUIV = ((FPGAIR_IFB_AU)<<8)|(FPGAIR_IFB_AV);
            FPGA_CURCHMINMAX_ADDR_IW   = FPGAIR_IFB_AW;
        }
#ifndef _HW_DC
    assert(uwChannel<1);
#else
    assert(uwChannel<3);
#endif
}

//***************************************************************************
// Setup PWI and PWB delays

static void SetPDDelays(SWORD swPDCurr, SWORD swPDVolt)
{
    UWORD ticks;

    ticks=(UWORD)(((FLOAT)PDTIMEOFFSET_CURR+(FLOAT)swPDCurr)/PDTIMEBASE);
    if(ticks>PDMAXTICKS)
        ticks=PDMAXTICKS;
    FPGA_ADC_PWI_DEL=ticks;
                            
    ticks=(UWORD)(((FLOAT)PDTIMEOFFSET_VOLT+(FLOAT)swPDVolt)/PDTIMEBASE);
    if(ticks>PDMAXTICKS)
        ticks=PDMAXTICKS;
    FPGA_ADC_PWB_DEL=ticks;
}

//***************************************************************************
// Manage current calibration

SWORD Mh_ManageCurrentCalibration(UWORD uwReqSamples)
{
    if(sMotorHandlerRun.flags.b.bCurrentCalibrEnable)
        return MH_CURCALIB_INPROGRESS;

        // status required
    if(uwReqSamples==0)
    {
        if(sMotorHandlerRun.flags.b.bCurrentCalibrAbort)
            return MH_CURCALIB_ABORTED;
        else
            return MH_CURCALIB_READY;
    }
        // setup calibration
    Os_BeginCriticalSection(OS_CRITSECT_GLOBAL);

    sMotorHandlerRun.flags.b.bCurrentCalibrEnable=TRUE;
    sMotorHandlerRun.flags.b.bCurrentCalibrAbort=FALSE;

    sMotorHandlerRun.slCalibrIuSum=0l;
    sMotorHandlerRun.slCalibrIvSum=0l;
    sMotorHandlerRun.slCalibrIwSum=0l;
    sMotorHandlerRun.uwCalibrCounter=0;
    sMotorHandlerRun.uwCalibrSamplesRequired=uwReqSamples;

    SetMinMaxChannels(SETMINMAX_CALIBRATION, 0);

    sMotorHandlerRun.ubCalibrStep=MHRT_CALSTEP_B0_AVG;
    sMotorHandlerRun.flags.b.bCurrentCalibrRTSum=TRUE;

    Os_EndCriticalSection(OS_CRITSECT_GLOBAL);

    return MH_CURCALIB_INPROGRESS;
}

//***************************************************************************
//

static void manageACline(void)
{
    SWORD swVprs, swVpst, swVptr;
    SWORD swVMax, swVMin, swDist;

    // if disabled or hardware not available
    if(!sMh_MotorDataOut.sPowerStageSts.b.bACInputDataValid)
        return;

    // evaluated periodically
    if(timer_istimedout(uwSysTimers1ms, sMotorHandlerRun.uwACMTimer))
    {
        sMotorHandlerRun.uwACMTimer = timer_settimeout(uwSysTimers1ms, ACLINEMONITOR_INTERVAL);

        // get max delta
        swVprs = sMotorHandlerRun.swACVrsMax - sMotorHandlerRun.swACVrsMin;
        swVpst = sMotorHandlerRun.swACVstMax - sMotorHandlerRun.swACVstMin;
        swVptr = sMotorHandlerRun.swACVtrMax - sMotorHandlerRun.swACVtrMin;

        // get min/max between RS, ST and TR
        swVMax = swVprs;
        if(swVMax < swVpst) swVMax = swVpst;
        if(swVMax < swVptr) swVMax = swVptr;
        swVMin = swVprs;
        if(swVMin > swVpst) swVMin = swVpst;
        if(swVMin > swVptr) swVMin = swVptr;

        // evaluate distortion
        swDist = abs((SWORD)((SLONG)swVMin*100/swVMax));

        // if undervoltage enabled, not undervoltage fault and below distortion then fail
        if(sMh_MotorDataParam.swUnderVoltageThreshold>0 &&
           swDist<sMh_MotorDataParam.uwVacMinDistortion &&
           swVMax>sMh_MotorDataParam.swUnderVoltageThreshold)
        {
            // diagnostic output
            if(swVMax == swVprs)
                sMh_MotorDataOut.ubACStatus = MH_ACMON_FAIL_PHASE_T;
            else if(swVMax == swVptr)
                sMh_MotorDataOut.ubACStatus = MH_ACMON_FAIL_PHASE_S;
            else
                sMh_MotorDataOut.ubACStatus = MH_ACMON_FAIL_PHASE_R;

            // if enabled it's an alarm, if disabled just warning
            if(sMh_MotorDataOut.sPowerStageSts.b.bFullyActive)
            {
                if(!sMotorHandlerRun.sErrorSent.b.bACMonitorFail)
                {
                    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_VL_UNDERVOLTAGE_FAIL, SYSTEMALARMS_SUBCODE_VLU_ACLINEFAILURE, FALSE) ;
                    (*sMh_MotorDataIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL) ;
                    sMotorHandlerRun.sErrorSent.b.bACMonitorFail = TRUE ;
                }
            }
            else
                atomic_long_set_bits( &ulSystemWarnings, SYSTEMWARNINGS_ACLINEFAILURE );
        }
        // otherwise reset warning (if was set)
        else
        {
            sMh_MotorDataOut.ubACStatus = MH_ACMON_NOERROR;
            atomic_long_clear_bits( &ulSystemWarnings, SYSTEMWARNINGS_ACLINEFAILURE );
        }

        // reset status variables
//        _atomic_(0);
        sMotorHandlerRun.swACVrsMax = sMotorHandlerRun.swACVrsMin = 0;
//        _endatomic_();
//        _atomic_(0);
        sMotorHandlerRun.swACVtrMax = sMotorHandlerRun.swACVtrMin = 0;
//        _endatomic_();
//        _atomic_(0);
        sMotorHandlerRun.swACVstMax = sMotorHandlerRun.swACVstMin = 0;
//        _endatomic_();
    }
}

//***************************************************************************
//
static void MotorDataBkGd(void)
{
  SWORD swUnderVoltage;
  UBYTE ubSelFreq;
  SLONG slAbsCurrLowerLimit,slTmp;
  FLOAT flTmp;
  BOOL bRetVal;

    // parameter validity check
  ResetOnlyParametersCheck();
  LiveParametersCheck();

    // calculate lower abs current limit for motor phases monitoring
  atomic_read(&slAbsCurrLowerLimit, &sMh_MotorDataOut.sIdLimit.slMin, sizeof(slAbsCurrLowerLimit));
  slAbsCurrLowerLimit=labs(slAbsCurrLowerLimit);

  atomic_read(&slTmp, &sMh_MotorDataOut.sIdLimit.slMax, sizeof(slTmp));
  slTmp=labs(slTmp);
  if(slTmp<slAbsCurrLowerLimit)
    slAbsCurrLowerLimit=slTmp;

  atomic_read(&slTmp, &sMh_MotorDataOut.sIqLimit.slMin, sizeof(slTmp));
  slTmp=labs(slTmp);
  if(slTmp<slAbsCurrLowerLimit)
    slAbsCurrLowerLimit=slTmp;

  atomic_read(&slTmp, &sMh_MotorDataOut.sIqLimit.slMax, sizeof(slTmp));
  slTmp=labs(slTmp);
  if(slTmp<slAbsCurrLowerLimit)
    slAbsCurrLowerLimit=slTmp;

  slAbsCurrLowerLimit=(SLONG)((FLOAT)slAbsCurrLowerLimit*PHASEMONITOR_MINRATIO);

  atomic_write(&sMotorHandlerRun.slAbsCurrLowerLimit, &slAbsCurrLowerLimit, sizeof(slAbsCurrLowerLimit));

    // calculate approx lower abs speed for motor phases monitoring
  if(sMh_MotorDataOut.swDcBusValue > PHASEMONITOR_MINVDCBUS*10)
  {
    flTmp  = (FLOAT)sMh_MotorDataOut.swDcBusValue/10.0/FLOAT_SQRT_OF_TWO;
    flTmp *= 1.0/(sGlbMotorParameters.flKT/FLOAT_SQRT_OF_THREE);
    flTmp *= 60.0/(2*FLOAT_PI) * SPEED_K_CONVERSION;
    slTmp = (SLONG)(flTmp*PHASEMONITOR_MINRATIO);
  }
  else
    slTmp = 0l;
      
  atomic_write(&sMotorHandlerRun.slAbsSpeedLowerLimit, &slTmp, sizeof(slTmp));

    // check for current calibration at startup, after a delay to get cleanest data
  if(!bSysStatBooting)
  {
    if(sMotorHandlerRun.flags.b.bCurrentCalibrResetTmr)
    {
      sMotorHandlerRun.uwCurrCalibrTmr=timer_settimeout(uwSysTimers1ms,CURRENT_CALIBR_1ST_DEL);
      sMotorHandlerRun.flags.b.bCurrentCalibrResetTmr=FALSE;
    }

    if(sMotorHandlerRun.flags.b.bCurrentCalibrFirstRun && !sMotorHandlerRun.flags.b.bCurrentCalibrEnable)
      if(timer_istimedout(uwSysTimers1ms,sMotorHandlerRun.uwCurrCalibrTmr))
        Mh_ManageCurrentCalibration(CURRENT_CALIBR_NSAMPLES);
  }

    // current offset calibration loop
  ComputeISenseOffsets();

    // if locked by boot error, just here for parameters checking and current calibration (avoid mis-reading
    // of wrong current by user)
  if(bSysStatLockedByBootError)
    return;

    // if no RT operation in progress
  if(!sMotorHandlerRun.flags.b.bUpdateFpgaGains && !sMotorHandlerRun.flags.b.bUpdatePwmFrequency)
  {
      // select switching frequency
#ifndef _HW_AXS_CABI35KW
    ubSelFreq=sMh_PlcAdvancedWorks.ubSetPwmFrequency;
#else
    ubSelFreq=FPGA_PWM_FREQ_16KHZ;
#endif

      // check system request for low frequency
    if(!sMh_PlcAdvancedWorks.flags.b.bDisableAutoFrequency && sMotorHandlerRun.bLoFreqRequested)
      ubSelFreq=MH_PWMFREQ_2KHZ;

      // recalc gains if needed
    bRetVal=SetupILoopAutoGains(FALSE, ubSelFreq);

    if(ubSelFreq!=Mh_GetActualPwmFrequency())
    {
      sMotorHandlerRun.ubSelFrequency=ubSelFreq;
      sMotorHandlerRun.flags.b.bUpdatePwmFrequency=TRUE;
    }
    else
      sMotorHandlerRun.flags.b.bUpdateFpgaGains=bRetVal;
  }

    // if no filter updating in progress
  if(!sMotorHandlerRun.flags.b.bUpdateFilters)
  {
      // if no plc handling request
    if(sMotorHandlerRun.sbFltPlcNumValid<0)
    {
        // if not updating
      if(sMotorHandlerRun.sbFltUpdCnt<0)
      {
          // CRC is used to detect parameters modification in place of
          // a shadow copy in order to save memory
        UWORD uwNewCRC=CRC_SEED;

          // CRC of all filters parameters
        uwNewCRC=crc16(uwNewCRC, (const UBYTE  *)sMh_MotorDataParam.sIqFiltPars, sizeof(sMh_MotorDataParam.sIqFiltPars));

          // if different trigger updating of all filters
        if(sMotorHandlerRun.uwFltCRC!=uwNewCRC)
        {
            // record actual CRC, must remain same troughout all updating procedure
          sMotorHandlerRun.uwFltCRC=uwNewCRC;

            // set first filter to be analyzed and reset valid filters counter
          sMotorHandlerRun.sbFltUpdCnt=0;
          sMotorHandlerRun.ubFltNumValid=0;
        }
      }

        // if already updating
      if(sMotorHandlerRun.sbFltUpdCnt>=0)
      {
        UWORD uwAftCRC=CRC_SEED;
        BOOL bValid;

          // calc constants of specific filter, if valid
        bValid=CalcFilterConstants(sMh_MotorDataParam.sIqFiltPars[sMotorHandlerRun.sbFltUpdCnt].ubType,
                                   sMh_MotorDataParam.sIqFiltPars[sMotorHandlerRun.sbFltUpdCnt].flFreqMain,
                                   sMh_MotorDataParam.sIqFiltPars[sMotorHandlerRun.sbFltUpdCnt].flDampMain,
                                   sMh_MotorDataParam.sIqFiltPars[sMotorHandlerRun.sbFltUpdCnt].flFreqSec,
                                   sMh_MotorDataParam.sIqFiltPars[sMotorHandlerRun.sbFltUpdCnt].flDampSec,
                                   sMotorHandlerRun.swFltConst);

          // recheck to avoid setting up messy filter
        uwAftCRC=crc16(uwAftCRC, (const UBYTE  *)sMh_MotorDataParam.sIqFiltPars, sizeof(sMh_MotorDataParam.sIqFiltPars));

          // if different then restart
        if(sMotorHandlerRun.uwFltCRC!=uwAftCRC)
        {
            // record actual CRC, must remain same troughout all updating procedure
          sMotorHandlerRun.uwFltCRC=uwAftCRC;

            // set first filter to be analyzed and reset valid filters counter
          sMotorHandlerRun.sbFltUpdCnt=0;
          sMotorHandlerRun.ubFltNumValid=0;
        }
            // check if filter is not valid
        else if(!bValid)
        {
            // stop filters handling
          sMotorHandlerRun.sbFltUpdCnt=-1;

            // setup void filter
          CalcFilterConstants(MH_FILTER_NONE, 0.0, 0.0, 0.0, 0.0, sMotorHandlerRun.swFltConst);

            // reset all filters
          sMotorHandlerRun.ubFltSelection=0;
          sMotorHandlerRun.ubFltNumber=0;

            // enable updating
          sMotorHandlerRun.flags.b.bUpdateFilters=TRUE;

            // issue system warning
          atomic_long_set_bits( &ulSystemWarnings, SYSTEMWARNINGS_INV_IQFILTERSETUP );
        }
        else
        {
            // do update if valid
          if(sMh_MotorDataParam.sIqFiltPars[sMotorHandlerRun.sbFltUpdCnt].ubType!=MH_FILTER_NONE)
          {
              // set filter to be updated to the next valid
            sMotorHandlerRun.ubFltSelection=sMotorHandlerRun.ubFltNumValid;
            sMotorHandlerRun.ubFltNumber=sMotorHandlerRun.ubFltNumValid;

              // enable updating
            sMotorHandlerRun.flags.b.bUpdateFilters=TRUE;

              // valid filters
            sMotorHandlerRun.ubFltNumValid++;
          }

            // next filter
          if(++sMotorHandlerRun.sbFltUpdCnt>=MH_IQFILTERS_MAX)
          {
              // stop filters handling
            sMotorHandlerRun.sbFltUpdCnt=-1;

              // if no filters are enabled then system require one void filter
            if(sMotorHandlerRun.ubFltNumValid==0)
            {
                // setup void filter
              CalcFilterConstants(MH_FILTER_NONE, 0.0, 0.0, 0.0, 0.0, sMotorHandlerRun.swFltConst);

                // reset all filters
              sMotorHandlerRun.ubFltSelection=0;
              sMotorHandlerRun.ubFltNumber=0;

                // enable updating
              sMotorHandlerRun.flags.b.bUpdateFilters=TRUE;
            }

              // reset system warning
            atomic_long_clear_bits( &ulSystemWarnings, SYSTEMWARNINGS_INV_IQFILTERSETUP );
          }
        }
      }
    }
      // if plc handling requested then just process constants setup
    else
    {
        // reset system filter handling as when returning to system filtering
        // it trigger a filter refresh
      sMotorHandlerRun.sbFltUpdCnt=-1;
      sMotorHandlerRun.uwFltCRC=0;
    }
  }

    // rescan limit list table
  ScanAllLimitList();

  /* == check underVoltage: se sono abilitato e' un fault, altrimenti e' un warning (autoresettante) == */
  swUnderVoltage=sMh_MotorDataParam.swUnderVoltageThreshold;
  if (sMh_MotorDataOut.swDcBusValue < swUnderVoltage && swUnderVoltage > 0)
  { // undervoltage: controllo se e' un fault od un warning
    if (sMh_MotorDataOut.sPowerStageSts.b.bFullyActive)
    {   // sono abilitato: fault
        if (!sMotorHandlerRun.sErrorSent.b.bUnderVoltage)
        {   // mando solo una volta
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_VL_UNDERVOLTAGE_FAIL, 0l, FALSE) ;
            (*sMh_MotorDataIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_NONFATAL) ;
            sMotorHandlerRun.sErrorSent.b.bUnderVoltage = TRUE ;
        }
    }
    else
        atomic_long_set_bits( &ulSystemWarnings, SYSTEMWARNINGS_UNDERVOLTAGE );
  }
  else
    atomic_long_clear_bits( &ulSystemWarnings, SYSTEMWARNINGS_UNDERVOLTAGE );

  if(swUnderVoltage < 0)
    ParChk_SignalValueError(PARCC_MH_UNDERVOLTAGE);
  else
    ParChk_ResetValueError(PARCC_MH_UNDERVOLTAGE);

  /* AC line monitoring */
  manageACline();

  /* check safetorqueoff: se sono abilitato e' un fault, altrimenti e' un warning (autoresettante) == */
 if (is_safetoff_active())
 {
   if (sMh_MotorDataOut.sPowerStageSts.b.bFullyActive&!sMotorHandlerRun.flags.b.bDisableSTOAlarm)
   {
       if (!sMotorHandlerRun.sErrorSent.b.bSafeTorqueOff)
       {
           SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_HW_SAFETORQUEOFF, 0l, FALSE) ;
           (*sMh_MotorDataIn.psCtrlHandlers->pfEmcyStop)(COMCTRL_ES_FATAL) ;
           sMotorHandlerRun.sErrorSent.b.bSafeTorqueOff = TRUE ;
       }
   }
   else
   {
       sMotorHandlerRun.flags.b.bSTOWarnActivated = TRUE;
       atomic_long_set_bits( &ulSystemWarnings, SYSTEMWARNINGS_SAFETORQUEOFFACTIVE );
       sMotorHandlerRun.uwSTODeactTOut = timer_settimeout (uwSysTimers1ms, SAFETORQUEOFF_REST_TOUT);
   }
 }
 else
 {
     // if not active but was activated then reset power faults as loss of
     // igbt driver supply cause power fault reading (after a timeout to let fully
     // power supply restore)
   if ( sMotorHandlerRun.flags.b.bSTOWarnActivated )
   {
     if ( timer_istimedout(uwSysTimers1ms, sMotorHandlerRun.uwSTODeactTOut) )
     {
       Mh_ResetPwrModFaults();
       atomic_long_clear_bits( &ulSystemWarnings, SYSTEMWARNINGS_SAFETORQUEOFFACTIVE );
       sMotorHandlerRun.flags.b.bSTOWarnActivated = FALSE ;
     }
   }
 }

  // se vengono cambiati da plc i parametri avanzati, li applica
  FPGA_REG16_ATSETBIT(FPGA_PWM_SET,   FPGA_PWM_B_DTCOMP_CUR_SIGN, sMh_PlcAdvancedWorks.flags.b.bDtCompCurrentSign);

  FPGA_PWM_DEADTIME  = sMh_PlcAdvancedWorks.uwDeadTime;
  FPGA_PWM_HOLDTIME  = sMh_PlcAdvancedWorks.uwHoldTime;
  FPGA_PWM_BLANKTIME = sMh_PlcAdvancedWorks.uwBlankTime;

  /* setup overcurrent counter limit and flag of PWM output */
  if(sMh_PlcAdvancedWorks.uwOvCurrCntLimit!=0)
     FPGA_PWM_OC_CNTLIMIT = sMh_PlcAdvancedWorks.uwOvCurrCntLimit-1;
  FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_EN_DISOCPWM, sMh_PlcAdvancedWorks.flags.b.bDisableOCPwmout);

  // if automatic two steps selection is disabled, then it is allowed to be manually set by the plc
  if(sMh_PlcAdvancedWorks.flags.b.bDisable2StepAuto)
  {
    UWORD uwPwmSet=0x0000;

    if(sMh_PlcAdvancedWorks.flags.b.bEnable2Steps)      uwPwmSet |= (1u<<FPGA_PWMS_B_2SWITCH);
    if(sMh_PlcAdvancedWorks.flags.b.bSelect2StepsFloat) uwPwmSet |= (1u<<FPGA_PWMS_B_2SWITCH_HL);
    if(sMh_PlcAdvancedWorks.flags.b.bSelect2StepsLOnly) uwPwmSet |= (1u<<FPGA_PWMS_B_2SWITCH_LOWONLY);
    if(sMh_PlcAdvancedWorks.flags.b.bSelectStraight)    uwPwmSet |= (1u<<FPGA_PWMS_B_STRAIGHT);

//    _atomic_(0);
    FPGA_PWMS_SET=sMh_MotorDataOut.uwActualPwmSetup=uwPwmSet;
//    _endatomic_();
  }

  // analog channels check watchdog used as PWM module protection
  if(FPGA_PWM_STATUS_ADC_WDT_EXP(FPGA_PWM_STATUS) && !sMotorHandlerRun.sErrorSent.b.bAdcWdtExp)
  {
    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_SOFTWARE_FAIL, SYSTEMALARMS_SUBCODE_SF_TM_ADC_WDT_EXPIRED, TRUE);
    sMotorHandlerRun.sErrorSent.b.bAdcWdtExp = TRUE;
  }

  // refresh propagation delays
  SetPDDelays(sMh_PlcAdvancedWorks.uwCurrPDTime, sMh_PlcAdvancedWorks.uwVoltPDTime);

  if (!sMotorHandlerRun.flags.b.bDSPAdvance)
  { // refresh user Vdc setup
    if(sMh_PlcAdvancedWorks.flags.b.bUserVdcBusSet)
    {
      SWORD swCv=(SWORD)(UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK,(SLONG)sMh_PlcAdvancedWorks.swVdcbusSet<<16)) ;

      sMotorHandlerRun.swUsrVdcSet = USRVDC_SET_FIX;
      sMotorHandlerRun.swUsrVdcVal = swCv;
    }
    else
    {
      sMotorHandlerRun.swUsrVdcSet = USRVDC_SET_ADC;
      sMotorHandlerRun.swUsrVdcVal = 0;
    }
  }
}

//***************************************************************************
//

static void DefluxWrapper(void)
{
  // in order to save stack it is executed from a wrapper
  DlfxSlow(sMotorHandlerRun.flags.b.bDefluxEnabled) ;
}

//***************************************************************************
//

void Mh_ForceCommand(UWORD uwCmd)
{
   Mh_ForceCommandEx(uwCmd, 0l);
}

void Mh_ForceCommandEx(UWORD uwCmd, ULONG ulParam)
{
   switch(uwCmd)
   {
       case MH_CMD_DISABLEBRAKEDRIVE:
           FPGA_REG16_ATSETBIT(FPGA_PWM_SET, FPGA_PWM_B_SETUP_BRAKE_ENABLE, 0);
           break;

       case MH_CMD_BACKEMFDATAENABLE:
//           _atomic_(0);
           sMotorHandlerRun.flags.b.bBackEMFDataEnable=TRUE;
//           _endatomic_();
           break;

       case MH_CMD_BACKEMFATANENABLE:
//           _atomic_(0);
           sMotorHandlerRun.flags.b.bBackEMFDataEnable=TRUE;
           sMotorHandlerRun.flags.b.bBackEMFATanEnable=TRUE;
//           _endatomic_();
           break;

       case MH_CMD_EN_ENERGYPROTECTION:
//           _atomic_(0);
           sMotorHandlerRun.flags.b.bEnergyProtEnabled=TRUE;
//           _endatomic_();
           break;

       case MH_CMD_DIS_ENERGYPROTECTION:
//           _atomic_(0);
           sMotorHandlerRun.flags.b.bEnergyProtEnabled=FALSE;
//           _endatomic_();
           break;

       case MH_CMD_SETOVERCURRENTLIMIT:
           {
               SLONG slTmp;

               atomic_read(&slTmp, &sMh_MotorDataOut.sDriveLimit.slOverCurrent, sizeof(SLONG));

               if ((SLONG)ulParam > slTmp)
                   FPGA_PWM_FLT_CURRLIMIT = UMCONV_CONVERT_32TO16(&sInternal2Fpga_I_RMS, slTmp) ;
               else
                   FPGA_PWM_FLT_CURRLIMIT = UMCONV_CONVERT_32TO16(&sInternal2Fpga_I_RMS, (SLONG)ulParam) ;
           }
           break;

       case MH_CMD_SETMODULATION:
           if(!sMh_PlcAdvancedWorks.flags.b.bDisable2StepAuto)
           {
               ulParam&=MH_CMD_SETMOD_ENABLE2STEPS|MH_CMD_SETMOD_2STEPSFLOAT|MH_CMD_SETMOD_2STEPSLONLY|MH_CMD_SETMOD_STRAIGHT;
//               _atomic_(0);
               FPGA_PWMS_SET=sMh_MotorDataOut.uwActualPwmSetup=(UWORD)ulParam;
//               _endatomic_();
           }
           break;

       case MH_CMD_FORCELOWFREQUENCY:
           if(!sMh_PlcAdvancedWorks.flags.b.bDisableAutoFrequency)
               sMotorHandlerRun.bLoFreqRequested=(SBYTE)ulParam;
           break;

       case MH_CMD_SETIQFILTER:
           sMotorHandlerRun.sbFltPlcNumValid=(SBYTE)ulParam;
           break;

       case MH_CMD_FORCEBRAKEON:
           if(ulParam)
           {
               SWORD swVZero=0;

//               _atomic_(0);
               bBrakeForcedOn=TRUE;
               FPGA_PWM_BRAKE_HIGH=swVZero;
               FPGA_PWM_BRAKE_LOW=swVZero;
//               _endatomic_();
           }
           else
           {
               SWORD swVBrakeHigh=sMotorHandlerRun.swVBrakeHigh;
               SWORD swVBrakeLow =sMotorHandlerRun.swVBrakeLow;

//               _atomic_(0);
               bBrakeForcedOn=FALSE;
               FPGA_PWM_BRAKE_HIGH=swVBrakeHigh;
               FPGA_PWM_BRAKE_LOW=swVBrakeLow;
//               _endatomic_();
           }

           break;

       case MH_CMD_ENABLEBRAKEDRIVE:
           FPGA_REG16_ATSETBIT(FPGA_PWM_SET, FPGA_PWM_B_SETUP_BRAKE_ENABLE, 1);
           break;

#ifdef _APP_XC
        case MH_CMD_EN_OVTOOUTSHORT:
#if CFG_ASC_DAYCO_MGMT
            if(bSysStatBooting)
            {
                sMotorHandlerRun.flags.b.bOvToShortEnabled = TRUE ;

                FPGA_REG16_ATSETBIT(FPGA_PWM_SET, FPGA_PWM_B_OV_TO_OUTSHORT, 0) ;  // remove Xiaokai ASC mgmt 

                FPGA_REG16_ATSETBIT(FPGA_PWM_ASC_CTRL, FPGA_PWM_ASC_CTRL_ENABLE, 1) ; // enable Antonio ASC mgmt
                FPGA_REG16_ATSETBIT(FPGA_PWM_ASC_CTRL, FPGA_PWM_ASC_CTRL_USE_HIGH, 0) ;  // use low Igbts
                FPGA_REG16_ATSETBIT(FPGA_PWM_ASC_CTRL, FPGA_PWM_ASC_CTRL_RELEASE_ASC, 0) ; // clear release cmd 

#if defined(_DEBUG_TRACES) && defined(_CRS_DBG)
#if _CRS_DBGDSK
                xil_printf("MotorHandler.c::Mh_ForceCommandEx(MH_CMD_EN_OVTOOUTSHORT): OvToShortEnabled [ %d ]\r\n", (UWORD)sMotorHandlerRun.flags.b.bOvToShortEnabled);
#endif
#endif
            }
            break;

        case MH_CMD_FORCE_ASC_CMD:
            /* 0->1: disable ASC */
            FPGA_REG16_SETBIT(FPGA_PWM_ASC_CTRL, FPGA_PWM_ASC_CTRL_RELEASE_ASC, (BOOL)ulParam) ;
#if defined(_DEBUG_TRACES) && defined(_CRS_DBG)
#if _CRS_DBGDSK
            xil_printf("MotorHandler.c::Mh_ForceCommandEx(MH_CMD_EN_OVTOOUTSHORT): ReleaseAscCmd [ %d ]\r\n", (BOOL)ulParam);
#endif
#endif
            break ; 
#else
            if(bSysStatBooting)
            {
                sMotorHandlerRun.flags.b.bOvToShortEnabled = TRUE;
                FPGA_REG16_ATSETBIT(FPGA_PWM_SET, FPGA_PWM_B_OV_TO_OUTSHORT, 1);
                FPGA_REG16_ATSETBIT(FPGA_PWM_SET, FPGA_PWM_B_EN_ACTIVESHORTCIRCUIT, 0); // ASC crs: clear usr cmd
            }

            // duty cycle expressed as 1/1000
            RGAD_PWM_OUTSHORT_VAL = (UWORD)(ulParam*FPGA_PWM_FREQMODULO/1000);
            break;

        case MH_CMD_FORCE_ASC_CMD:
            /* TRUE: enable ASC, FALSE: disable ASC */
            FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_EN_ACTIVESHORTCIRCUIT, (BOOL)ulParam) ;    
            break ;           
#endif // cfg_asc_dayco_mgmt
#endif // _app_xc
#ifdef _HW_CT
       case MH_CMD_SET_BRAKE_DUTY:
           // duty cycle expressed as 1/1000
           FPGA_PWM_BRK_COUNT = (UWORD)(ulParam*FPGA_PWM_FREQMODULO/1000);
           break;
#endif // _hw_ct
       case MH_CMD_DISABLESTOALARM:
           if(ulParam)
               sMotorHandlerRun.flags.b.bDisableSTOAlarm = TRUE;
           else
               sMotorHandlerRun.flags.b.bDisableSTOAlarm = FALSE;
           break;
#if CFG_PWM_SHIFT
        case MH_CMD_PWM_SHIFT_SETVALUE:
            // ulParam_LO = PWM shift for bridge 1; ulParam_HI = PWM shift for bridge 2
            UWORD uwPwmShift2Use_1, uwPwmShift2Use_2 ;
          
            if (sMotorHandlerRun.flags.b.bParallelBr1Mode)
            {
                uwPwmShift2Use_1 = (UWORD)ulParam ; 
                uwPwmShift2Use_2 = 0x0000 ;

                if (sMotorHandlerRun.flags.b.bParallelBr2Mode)
                    uwPwmShift2Use_2 = (UWORD)(ulParam >> 16) ;  
            }
            else
            {
                uwPwmShift2Use_1 = 0x0000 ; 
                uwPwmShift2Use_2 = 0x0000 ;
            } 

            // apply the shift to the FPGA 
            FPGA_PWM_SHIFT_1_DELAY = uwPwmShift2Use_1 ;
            FPGA_PWM_SHIFT_2_DELAY = uwPwmShift2Use_2 ;

            break;    
#endif // cfg_pwm_shift
    }
}

//***************************************************************************
// NOTE: Iu, Iv, Vu, Vv are ALL PEAK values, NOT RMS !!!!!!!!!!!
// Calculating EMF values: Emf = V - [R * I + L * (dI/dt)]      
// Alpha = (sqrt(3) / 2) * EmfU                                 
// Beta  = (0.5 * EmfU) + EmfV                                  
void Mh_MotorBackEMF(BOOL bATanEnabled)
{
  SLONG slRxIu, slRxIv ;
  SLONG sldIudT, sldIvdT, slLxdIudT, slLxdIvdT ;
  SQWRD sqRxIu, sqRxIv, sqLxdIudT, sqLxdIvdT ;
  SWORD swRxIu, swRxIv, swLxdIudT, swLxdIvdT ;
  SWORD swEmfU, swEmfV ;

  UWORD uwEmfSQRoot ;

  // ===================================
  // ============== R * I ==============
  // ===================================
  // V (1e-1V) = R (1e-3 Ohm) * I (1e-4A) = R * I * 1e-6
  _sint64_mul_32_16(&sqRxIu, &sMh_MotorDataOut.slIuFb, &sMotorHandlerRun.swStatorR) ;
  _sint64_mul_32_16(&sqRxIv, &sMh_MotorDataOut.slIvFb, &sMotorHandlerRun.swStatorR) ;
  slRxIu = (SLONG)(INT64_MLONG(sqRxIu)) >> 4 ; // 1e-6 * 2^(16+4) (1e-1V)
  slRxIv = (SLONG)(INT64_MLONG(sqRxIv)) >> 4 ;

  // porto tutto in 1e-V
  if (slRxIu > 31200L) // = 32767 / 1.05 (1.05 perche' divido per 2^16 con MLONG)
    swRxIu = 32767L ;
  else if(slRxIu <-31200L)
    swRxIu = -32767L ;
  else
    swRxIu = (SWORD)((slRxIu * 17180) >> 14) ;

  if (slRxIv > 31200L) // = 32767 / 1.05
    swRxIv = 32767 ;
  else if(slRxIv <-31200L)
    swRxIv = -32767 ;
  else
    swRxIv = (SWORD)((slRxIv * 17180) >> 14) ;

  // ===================================
  // =========== L * (dI/dt) ===========
  // ===================================
  // (Vui * 1e-1) = [0.5 *(Lui * 1e-5)] * (dIui * 1e-4) / (tick / 8000) => Vui = Lui * dI/tick * 4e-5
  // 4e-5 * 2^16 = 2.62 (2^16 perche' INT64_MLONG)
  sldIudT = sMh_MotorDataOut.slIuFb - sMotorHandlerRun.slIuFb_1 ;
  sldIvdT = sMh_MotorDataOut.slIvFb - sMotorHandlerRun.slIvFb_1 ;
  _sint64_mul_32_16(&sqLxdIudT, &sldIudT, &sMotorHandlerRun.swStatorL) ;
  _sint64_mul_32_16(&sqLxdIvdT, &sldIvdT, &sMotorHandlerRun.swStatorL) ;
  slLxdIudT = (SLONG)(INT64_MLONG(sqLxdIudT)) ; // INT64_MLONG(sqLxdIudT) * 2.62 (1e-1V)
  slLxdIvdT = (SLONG)(INT64_MLONG(sqLxdIvdT)) ; // INT64_MLONG(sqLxdIvdT) * 2.62 (1e-1V)

  // porto tutto in 1e-V
  if (slLxdIudT > 12500L) // 2.62/2^16 = 671/2^24
    swLxdIudT = 32767 ;
  else if(slLxdIudT <-12500L)
    swLxdIudT = -32767 ;
  else
    swLxdIudT = (SWORD)((slLxdIudT * 671) >> 8) ;

  if (slLxdIvdT > 12500L)
    swLxdIvdT = 32767 ;
  else if(slLxdIvdT <-12500L)
    swLxdIvdT = -32767 ;
  else
    swLxdIvdT = (SWORD)((slLxdIvdT * 671) >> 8) ;

  // ===================================
  // =============== Emf ===============
  // ===================================
  /* Vemf_Peak(1e-1V) = VEstimated_Peak(1e-1V) - [ R(1e-3Ohm) * I_Peak(1e-4A) + L(1e-5H) * dI_Peak/dt(1e-4A) ] */
  swEmfU = sMh_MotorDataOut.swVuOut - swRxIu - swLxdIudT ;
  swEmfV = sMh_MotorDataOut.swVvOut - swRxIv - swLxdIvdT ;

  /* regressione */
  sMotorHandlerRun.slIuFb_1 = sMh_MotorDataOut.slIuFb ;
  sMotorHandlerRun.slIvFb_1 = sMh_MotorDataOut.slIvFb ;

  sMh_MotorDataOut.sBackEmfData.swAlpha = (SWORD)(((SLONG)swEmfU * RADICE_TRE_QUARTI) >> 16) ; /* Back EMF Alpha */
  sMh_MotorDataOut.sBackEmfData.swBeta  = (swEmfU / 4) + (swEmfV / 2) ;                        /* Back EMF Beta  */

  if(bATanEnabled)
    sMh_MotorDataOut.sBackEmfData.uwAtanAngle = 0x8000 + ATan16(sMh_MotorDataOut.sBackEmfData.swBeta, sMh_MotorDataOut.sBackEmfData.swAlpha) ;

  uwEmfSQRoot = CalculateSquareRoot(sMh_MotorDataOut.sBackEmfData.swBeta, sMh_MotorDataOut.sBackEmfData.swAlpha) ;
  sMotorHandlerRun.ulEmfSQRootFiltered = (7 * sMotorHandlerRun.ulEmfSQRootFiltered + 1024 * (ULONG)uwEmfSQRoot) / 8 ; // uwVmotor e' il valore di picco concatenato (filtro)
  sMh_MotorDataOut.sBackEmfData.uwVmotor = (UWORD)(sMotorHandlerRun.ulEmfSQRootFiltered / 512) ; /* 512 = 256 * 2 -> since the result from SquareRoot has to be divided by 2 (originale) */
  sMh_MotorDataOut.sPowerStageSts.b.bBackEMFDataValid = TRUE;
}

//***************************************************************************
//
void Mh_ResetMotorBackEmf(void)
{
  sMotorHandlerRun.ulEmfSQRootFiltered  = 0 ;
  sMh_MotorDataOut.sBackEmfData.swAlpha = 0 ;
  sMh_MotorDataOut.sBackEmfData.swBeta  = 0 ;
  sMh_MotorDataOut.sBackEmfData.uwAtanAngle = 0 ;
  sMh_MotorDataOut.sBackEmfData.uwVmotor = 0 ;
  sMh_MotorDataOut.sPowerStageSts.b.bBackEMFDataValid = FALSE;
}

//***************************************************************************
//
static void NormalizeTorqueLoopGains(FLOAT flUserFpgaKp, FLOAT flUserFpgaKi, MH_DSPPARS * psDest)
{
  SWORD swKpShift, swKiShift, swShift2do ;

  /* conversione in scala FPGA */
  flUserFpgaKp = flUserFpgaKp * flGainsRatio ;
  flUserFpgaKi = flUserFpgaKi * flGainsRatio ;

  /* calcolo lo shift */
  swKpShift = (SWORD)(log10(flUserFpgaKp) / log10(2.0)) ;
  swKiShift = (SWORD)(log10(flUserFpgaKi) / log10(2.0)) ;

  /* prendo il pi?alto dei due shift */
  if(swKpShift > swKiShift)
    swShift2do = swKpShift ;
  else
    swShift2do = swKiShift ;

  psDest->swIOutShift = swShift2do - 14 ;

  /* shift overflow check */
  if(psDest->swIOutShift > 15)
    psDest->swIOutShift = 15 ;
  else if (psDest->swIOutShift < -15)
    psDest->swIOutShift = -15 ;

  /* normalizzazione */
  flUserFpgaKp = flUserFpgaKp / pow(2, psDest->swIOutShift) ;
  flUserFpgaKi = flUserFpgaKi / pow(2, psDest->swIOutShift) ;

  /* overfow checks */
  if (flUserFpgaKp > 32767.0)
    psDest->swKp = 32767 ;
  else if (flUserFpgaKp < -32768.0)
    psDest->swKp = -32768 ;
  else
    psDest->swKp = (SWORD)flUserFpgaKp ;

  if (flUserFpgaKi > 32767.0)
    psDest->swKi = 32767 ;
  else if (flUserFpgaKi < -32768.0)
    psDest->swKi = -32768 ;
  else
    psDest->swKi = (SWORD)flUserFpgaKi ;
}

//***************************************************************************
//
BOOL Mh_SetIqFilter(UBYTE ubFiltNum, HPSWORD hpswSrcV)
{	// plc function to manage IqFilter
   SBYTE ubNumValid=sMotorHandlerRun.sbFltPlcNumValid;

       // check right task
   if(!Os_IsInBackground() && !PlcIsInSlowTask())
       return FALSE;

       // if not plc handling enabled
   if(ubNumValid<0)
       return FALSE;

       // if still updating some filters
   if(sMotorHandlerRun.flags.b.bUpdateFilters)
       return FALSE;

       // if wrong filter selection
   if(ubFiltNum>=MH_IQFILTERS_MAX)
       return FALSE;

       // if zero selected then no filters are requested, then setup void
   if(ubNumValid==0)
   {
           // setup void filter
       CalcFilterConstants(MH_FILTER_NONE, 0.0, 0.0, 0.0, 0.0, sMotorHandlerRun.swFltConst);

           // reset all filters
       sMotorHandlerRun.ubFltSelection=0;
       sMotorHandlerRun.ubFltNumber=0;

           // enable updating
       sMotorHandlerRun.flags.b.bUpdateFilters=TRUE;
   }
       // setup new filter
   else
   {
       UWORD uwCt;

           // constants copy
       for(uwCt=0;uwCt<5;uwCt++)
           sMotorHandlerRun.swFltConst[uwCt]=*hpswSrcV++;

           // set filter selection and number
       sMotorHandlerRun.ubFltSelection=ubFiltNum;
       sMotorHandlerRun.ubFltNumber=(ubNumValid<=MH_IQFILTERS_MAX?ubNumValid-1:MH_IQFILTERS_MAX-1);

           // enable updating
       sMotorHandlerRun.flags.b.bUpdateFilters=TRUE;
   }

   return TRUE;
}

//***************************************************************************
//

static BOOL CalcFilterConstants(UBYTE ubType, FLOAT flFreqMain, FLOAT flDampMain, FLOAT flFreqSec, FLOAT flDampSec, HPSWORD hpswDstV)
{
    FLOAT flDv,flPw;
    SWORD swSum;
    UWORD uwCt;

    switch(ubType)
    {
        case MH_FILTER_NONE:
            hpswDstV[0]=MH_FILTER_INTNORM;
            hpswDstV[1]=0;
            hpswDstV[2]=0;
            hpswDstV[3]=0;
            hpswDstV[4]=0;
                // skip constants refining and checking
            return TRUE;

        case MH_FILTER_NOTCH:
            flPw=pow(RT_TASK_PERIOD*flFreqMain,2.0);
    		flDv=4+2*flFreqMain*flDampMain*RT_TASK_PERIOD+flPw;
            flDv=(FLOAT)MH_FILTER_INTNORM/flDv;

    		hpswDstV[0]=(SWORD)(0.5+flDv*(4+flPw));
    		hpswDstV[1]=(SWORD)(0.5+flDv*(2*flPw-8));
    		hpswDstV[2]=(SWORD)(0.5+flDv*(4+flPw));
    		hpswDstV[3]=(SWORD)(0.5+flDv*-(2*flPw-8));
    		hpswDstV[4]=(SWORD)(0.5+flDv*-(4-2*RT_TASK_PERIOD*flFreqMain*flDampMain+flPw));
            break;

        case MH_FILTER_BIQUAD:
            flPw=pow(RT_TASK_PERIOD*flFreqMain,2.0);
    		flDv=4*flFreqMain*flFreqMain+4*flDampSec*flFreqMain*flFreqMain*flFreqSec*RT_TASK_PERIOD+pow(RT_TASK_PERIOD*flFreqMain*flFreqSec,2.0);
            flDv=(FLOAT)MH_FILTER_INTNORM/flDv;

    		hpswDstV[0]=(SWORD)(0.5+flDv*flFreqSec*flFreqSec*(4+4*flDampMain*flFreqMain*RT_TASK_PERIOD+flPw));
    		hpswDstV[1]=(SWORD)(0.5+flDv*flFreqSec*flFreqSec*(2*flPw-8));
    		hpswDstV[2]=(SWORD)(0.5+flDv*flFreqSec*flFreqSec*(4-4*flDampMain*flFreqMain*RT_TASK_PERIOD+flPw));
    		hpswDstV[3]=(SWORD)(0.5+flDv*-(flFreqMain*flFreqMain*(2*pow(RT_TASK_PERIOD*flFreqSec,2.0)-8)));
    		hpswDstV[4]=(SWORD)(0.5+flDv*-(4*flFreqMain*flFreqMain-4*flDampSec*flFreqMain*flFreqMain*flFreqSec*RT_TASK_PERIOD+pow(RT_TASK_PERIOD*flFreqMain*flFreqSec,2.0)));
            break;

        case MH_FILTER_LOWPASS:
            flPw=pow(RT_TASK_PERIOD*flFreqMain,2.0);
    		flDv=4+4*flFreqMain*flDampMain*RT_TASK_PERIOD+flPw;
            flDv=(FLOAT)MH_FILTER_INTNORM/flDv;

    		hpswDstV[0]=(SWORD)(0.5+flDv*flPw);
    		hpswDstV[1]=(SWORD)(0.5+flDv*2*flPw);
    		hpswDstV[2]=(SWORD)(0.5+flDv*flPw);
    		hpswDstV[3]=(SWORD)(0.5+flDv*-(2*flPw-8));
    		hpswDstV[4]=(SWORD)(0.5+flDv*-(4-4*flFreqMain*flDampMain*RT_TASK_PERIOD+flPw));
            break;

        default:
            return FALSE;

	}

        // adjust values across approx in order to keep unity gain
	for(uwCt=0,swSum=0;uwCt<5;uwCt++)
        swSum+=hpswDstV[uwCt];

	hpswDstV[1]+=MH_FILTER_INTNORM-swSum;

        // if quantization is below minimum limit then filter is not valid
	for(uwCt=0;uwCt<5;uwCt++)
        if(abs(hpswDstV[uwCt])<FILTER_QTLIMIT)
            return FALSE;

    return TRUE;
}

//***************************************************************************
//
BOOL Mh_PlcCalcFilterConstants(MH_FILTERPARS  * hpsPars, HPSWORD hpswDstV)
{
   return CalcFilterConstants(hpsPars->ubType,
                              hpsPars->flFreqMain,
                              hpsPars->flDampMain,
                              hpsPars->flFreqSec,
                              hpsPars->flDampSec,
                              hpswDstV);
}

//***************************************************************************
//
static BOOL InitLimitList(void)
{
  SWORD swElement ;

  for(swElement = 0; swElement<LIST_SIZE; swElement++)
  {
    puwIqLimitList[swElement] = NULL ;
    puwIdLimitList[swElement] = NULL ;
  }

  return TRUE;
}


//***************************************************************************
//
static BOOL Add2LimitList(UWORD uwSel, GLB_TORQUE_LIMIT *sStruct2Add)
{
  GLB_TORQUE_LIMIT ** plist;
  SWORD swElement ;

  switch(uwSel)
  {
    case MH_LIMITLIST_IQ:
      plist=puwIqLimitList;
      break;

    case MH_LIMITLIST_ID:
      plist=puwIdLimitList;
      break;

    default:
      return TRUE;
  }

  for (swElement = 0; swElement < LIST_SIZE; swElement++)
  {
    if (plist[swElement] == NULL)
    {
      plist[swElement] = sStruct2Add ;
      return FALSE ;
    }
  }
  return TRUE ; /* Errore!! list full!! */
}


//***************************************************************************
//
static BOOL DeleteLimitFromList(UWORD uwSel, GLB_TORQUE_LIMIT *sStruct2Delete)
{
  GLB_TORQUE_LIMIT ** plist;
  SWORD swElement ;

  switch(uwSel)
  {
    case MH_LIMITLIST_IQ:
      plist=puwIqLimitList;
      break;

    case MH_LIMITLIST_ID:
      plist=puwIdLimitList;
      break;

    default:
      return TRUE;
  }

  for (swElement = 0; swElement < LIST_SIZE; swElement++)
  {
    if (plist[swElement] == sStruct2Delete)
    {
      plist[swElement] = NULL ;
      return FALSE ;
    }
  }
  return TRUE ; /* Errore!! not found!! */
}

//***************************************************************************
//
static void FindLimitList(GLB_TORQUE_LIMIT ** plist, GLB_TORQUE_LIMIT * dataout)
{
  SWORD swElement ;
  SLONG slMax2Use, slMin2Use ;

  slMax2Use = SLONG_MAX_VALUE ;
  slMin2Use = SLONG_MIN_VALUE ;

  /* find Maximum and Minimum of the list */
  for (swElement = 0; swElement < LIST_SIZE; swElement++)
  {
    if (plist[swElement] != NULL)
    {
      if (plist[swElement]->slMax < slMax2Use)
      {
        if (plist[swElement]->slMax > 0)
          slMax2Use = plist[swElement]->slMax ;
        else
          slMax2Use = 0 ;
      }

      if (plist[swElement]->slMin > slMin2Use)
      {
        if (plist[swElement]->slMin < 0)
          slMin2Use = plist[swElement]->slMin ;
        else
          slMin2Use = 0 ;
      }
    }
  }
  /* write out the results */
  dataout->slMax = slMax2Use ;
  dataout->slMin = slMin2Use ;
}

//***************************************************************************
//
static void ScanAllLimitList(void)
{
  GLB_TORQUE_LIMIT sLocILimit;

  assert(sizeof(GLB_TORQUE_LIMIT)<=ATOMIC_RW_MAX_SIZE);

  sLocILimit=sMotorHandlerRun.sIqPartialLimit;
  FindLimitList(puwIqLimitList, &sLocILimit);
  atomic_write(&sMotorHandlerRun.sIqPartialLimit, &sLocILimit, sizeof(sLocILimit));
    
  sLocILimit=sMotorHandlerRun.sIdPartialLimit;
  FindLimitList(puwIdLimitList, &sLocILimit);
  atomic_write(&sMotorHandlerRun.sIdPartialLimit, &sLocILimit, sizeof(sLocILimit));
}

//***************************************************************************
//

void Mh_ResetPwrModFaults(void)
{
    // reset faults cycling L -> H -> L
    FPGA_REG16_ATSETBIT(FPGA_PWM_SET, FPGA_PWM_B_SETUP_RESET_FAULT, !sMotorHandlerRun.flags.b.bResFltPolarity);
#ifdef _INFINEON_
#ifndef _HW_DC
    if(sMotorHandlerRun.flags.b.bDIF)
        P9_OUT_P1 = !sMotorHandlerRun.flags.b.bResFltPolarity;
#else
    P1_OUT_P7 = !sMotorHandlerRun.flags.b.bResFltPolarity;
#endif
#else
    GPIO_OUT(PWB_RESET_FAULT_PIN,!sMotorHandlerRun.flags.b.bResFltPolarity);
#endif

    // wait at least 1us
    timer_wait(uwSysTimers100ns,10);

    // deassert reset
    FPGA_REG16_ATSETBIT(FPGA_PWM_SET, FPGA_PWM_B_SETUP_RESET_FAULT, sMotorHandlerRun.flags.b.bResFltPolarity);
#ifdef _INFINEON_
#ifndef _HW_DC
    if(sMotorHandlerRun.flags.b.bDIF)
        P9_OUT_P1 = sMotorHandlerRun.flags.b.bResFltPolarity;
#else
    P1_OUT_P7 = sMotorHandlerRun.flags.b.bResFltPolarity;
#endif
#else
    GPIO_OUT(PWB_RESET_FAULT_PIN,sMotorHandlerRun.flags.b.bResFltPolarity);
#endif
}

//***************************************************************************
//

static BOOL Mh_POSTInit(void)
{
  if(bSysStatLockedByBootError)
    return TRUE;

  // allowed only on specific hardware
  switch(sGlbPowerBoardParameters.sProductInfo.uwProductCode)
  {
    case HWPRM_PCODE_POWER_BOARD_OLD_AXW_AC:
    case HWPRM_PCODE_POWER_BOARD_OLD_AXW_DC:
    case HWPRM_PCODE_POWER_BOARD_AXW_AC:
    case HWPRM_PCODE_POWER_BOARD_AXW_DC:
    case HWPRM_PCODE_POWER_BOARD_DIF_CI:
    case HWPRM_PCODE_POWER_BOARD_DIF_PI:
    case HWPRM_PCODE_POWER_BOARD_DIF_BC:
        break;

    case HWPRM_PCODE_POWER_BOARD_UNIVERSAL:
        // not supported if -1.0
        if(sGlbPowerBoardParameters.sUniv.sDCChrg.fResistor<0.0)
            return TRUE;
        break;

    default:
        return TRUE;
  }

  // if disabled then exit immediately
  if(sMh_MotorDataParam.flags.b.bDisablePOST)
    return TRUE;

  bPOSTInstalled=TaskSched_AddRTTask(&Mh_POST8KHz, TASKSCHEDULER_FLAG_NONE, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0) ;

  return bPOSTInstalled;
}

//***************************************************************************
//

SWORD Mh_POSTExecute(void)
{
  POST_RUNTIME sPOSTRun;

  // if POST was not installed then exit immediately
  if(!bPOSTInstalled)
    return POST_FAIL_NONE;

  // setup data structure in local stack to save heap space,
  // as after POST this became unused
  psPOSTRun=&sPOSTRun;

  // disable output short if selected as intefere with procedure
#ifdef _APP_XC
  FPGA_REG16_ATSETBIT(FPGA_PWM_SET, FPGA_PWM_B_OV_TO_OUTSHORT, 0);
#if CFG_ASC_DAYCO_MGMT
    FPGA_REG16_ATSETBIT(FPGA_PWM_ASC_CTRL, FPGA_PWM_ASC_CTRL_ENABLE, 0) ; // enable Dayco ASC mgmt
#endif // cfg_asc_dayco_mgmt
#endif // _app_xc

  // setup overvoltage for fast disable in case of power connection
  FPGA_PWM_FLT_DCBUSLIMIT = UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK, (SLONG)POST_OVERVOLTAGE<<16);

  // calculate charge time
  if(sGlbPowerBoardParameters.sProductInfo.uwProductCode==HWPRM_PCODE_POWER_BOARD_UNIVERSAL)
  {
    FLOAT ftout;
    // 5*RC, expressed as 125usec ticks
    ftout = sGlbPowerBoardParameters.sUniv.sDCChrg.fCapacity*1e-6*sGlbPowerBoardParameters.sUniv.sDCChrg.fResistor*
                5*REALTIME_TASK_FREQ;
    if(ftout>32767.0)
        ftout=32767.0;
    psPOSTRun->uwTimeoutCharge = (UWORD)ftout;
  }
  else
    psPOSTRun->uwTimeoutCharge = POST_TIMEOUT_DEF_CHARGE;

  // setup runtime status
  psPOSTRun->uwStatus = 0;
  psPOSTRun->uwPrevStatus = 0;
  psPOSTRun->uwFailedTest = POST_FAIL_NONE;
  psPOSTRun->uwIGBTFailed = 0;
  psPOSTRun->uwWDTTimer = psPOSTRun->uwTimeoutCharge;

  // start rt task processing
  psPOSTRun->sFlags.b.bExecute = TRUE;

  // wait rt task completion
  while(psPOSTRun->sFlags.b.bExecute);

  // disable POST rt
  psPOSTRun=NULL;

  // if aborted and not forced, then return no failure
  if(sPOSTRun.uwFailedTest==POST_FAIL_ABORTED && !sMh_MotorDataParam.flags.b.bForcePOST)
    sPOSTRun.uwFailedTest = POST_FAIL_NONE;

  // restore system
  FPGA_REG16_SETBIT(FPGA_PWM_SET, FPGA_PWM_B_SETUP_ENABLE, 0);
  FPGA_PWM_FLT_DCBUSLIMIT = UMCONV_CONVERT_32TO16(&sInternal2Fpga_V_DC_PEAK, (SLONG)sMh_MotorDataOut.sDriveLimit.swOverVoltage<<16);
#ifdef _APP_XC

#if CFG_ASC_DAYCO_MGMT
  FPGA_REG16_ATSETBIT(FPGA_PWM_ASC_CTRL, FPGA_PWM_ASC_CTRL_ENABLE, sMotorHandlerRun.flags.b.bOvToShortEnabled) ; // enable Dayco ASC mgmt
#else
  FPGA_REG16_ATSETBIT(FPGA_PWM_SET, FPGA_PWM_B_OV_TO_OUTSHORT, sMotorHandlerRun.flags.b.bOvToShortEnabled);

#if defined(_DEBUG_TRACES) && defined(_CRS_DBG)
  xil_printf("MotorHandler.c::Mh_POSTExecute(): OvToShortEnabled [ %d ]\r\n", (UWORD)sMotorHandlerRun.flags.b.bOvToShortEnabled);
#endif

#endif // cfg_asc_dayco_mgmt
#endif // _app_ax

  // setup extended POST return value
  sMh_MotorDataOut.uwPostResultEx = psPOSTRun->uwFailedTest | (psPOSTRun->uwIGBTFailed<<8);

  return sPOSTRun.uwFailedTest;
}

//***************************************************************************
// Maschera di accensione degli IGBT 

static void SelfTestPwmBridgeLayout(BOOL bULo, BOOL bUHi, BOOL bVLo, BOOL bVHi, BOOL bWLo, BOOL bWHi)
{
   UWORD uwLayout = ((UWORD)bULo        + (UWORD)bUHi * 0x02 +
                     (UWORD)bVLo * 0x04 + (UWORD)bVHi * 0x08 +
                     (UWORD)bWLo * 0x10 + (UWORD)bWHi * 0x20  ) ;

   FPGA_REG16_SETBIT(FPGA_PWM_SET,   FPGA_PWM_B_SETUP_USE_MODULATOR, 0);
   FPGA_REG16_SETBIT(FPGA_PWM_SETEX, FPGA_PWM_B_SETEX_TRANSFMOD, 0);
   FPGA_PWM_BRIDGE_LAYOUT = (FPGA_PWM_BRIDGE_LAYOUT&0x00FF) | ((uwLayout^0x00ff)<<8);
   FPGA_REG16_SETBIT(FPGA_PWM_SET,   FPGA_PWM_B_SETUP_ENABLE, 1);
}

//***************************************************************************
// Routine di autotest del drive:
// verifico i 3 igbt alti, i 3 igbt bassi, gli igbt incrociati, la resistenza di frenatura, le tensioni della controlboard

static BOOL Mh_POST8KHz(void)
{
   UWORD uwPwmStatus = FPGA_PWM_STATUS ;
   UWORD uwBrokenIgbt = 0, uwIgbtStatus = 0 ;

   // non richiesta esecuzione
   if(psPOSTRun==NULL)
       return TRUE;

   // esecuzione terminata o non ancora partita
   if(!psPOSTRun->sFlags.b.bExecute)
       return TRUE;

   // controllo fault
   if(FPGA_PWM_STATUS_FAULT_ACTIVE(uwPwmStatus))
   {
       psPOSTRun->uwFailedTest = POST_FAIL_ABORTED;

       if(FPGA_PWM_STATUS_OVER_CURRENT(uwPwmStatus))
           psPOSTRun->uwFailedTest = POST_FAIL_OVERCURRENT;

       if(FPGA_PWM_STATUS_OVER_VOLTAGE(uwPwmStatus))
           psPOSTRun->uwFailedTest = POST_FAIL_ABORTED;

       if(FPGA_PWM_STATUS_DESATURATION(uwPwmStatus))
           psPOSTRun->uwFailedTest = POST_FAIL_BRIDGEDESAT;

       if(FPGA_PWM_STATUS_BRAKE_DESAT(uwPwmStatus))
           psPOSTRun->uwFailedTest = POST_FAIL_BRAKEDESAT;

       psPOSTRun->sFlags.b.bExecute = FALSE;
       return TRUE;
   }

   // state processing
   switch (psPOSTRun->uwStatus)
   {
       case 0: /* tutti gli IGBT spenti */
           SelfTestPwmBridgeLayout(FALSE, FALSE, FALSE, FALSE, FALSE, FALSE) ;

           if (sMh_MotorDataOut.swDcBusValue < POST_DCBUSSHORTED)
           {   // Vdc e' gia' zero: vuol dire che c'e' un corto circuito sul DcBus -> test finito
               psPOSTRun->uwFailedTest = POST_FAIL_DCBUSSHORTCIRCUIT ;
               psPOSTRun->uwStatus = 666 ;
           }
           else if (sMh_MotorDataOut.swDcBusValue < POST_UNDERVOLTAGE)
           {   // Vdc troppo bassa: aspetto 200ms e se Vdc non sale entro il timeout
               // allora non eseguo il test, altrimenti avrei risultati fuorvianti
               if (psPOSTRun->uwTimer >= psPOSTRun->uwTimeoutCharge)
               {
                   psPOSTRun->uwFailedTest = POST_FAIL_LOWVDC ;
                   psPOSTRun->uwStatus = 666 ;
               }
               else
                   psPOSTRun->uwTimer += 1 ;
           }
           else
           {   // le Vdc e' a posto: comincio il test solo se STO NON e' attivo
               if (is_safetoff_active())
               {   // STO e' attivo (mi manca un ingresso od entrambi gli ingressi STO)
                   psPOSTRun->uwFailedTest = POST_FAIL_MISSINGSTO;
                   psPOSTRun->uwStatus = 666 ;
               }
               else
               {   // comincio l'autotest
                   psPOSTRun->uwTimer = 0 ;
                   psPOSTRun->uwStatus = 10 ;
               }
           }
           break ;

       /* ============================================== */
       /* ================= IGBT  ALTI ================= */
       /* ============================================== */
       case 10: /* Accendo i 3 IGBT alti */
           SelfTestPwmBridgeLayout(FALSE, TRUE, FALSE, TRUE, FALSE, TRUE) ; // accendo PWM
           psPOSTRun->uwFailedTest = POST_FAIL_NONE ;
           psPOSTRun->uwStatus = 15 ;
           psPOSTRun->uwTimer = 0 ;
           break ;

       case 15: /* 1) PWM e' ACCESO
                 * 2) il test dura POST_TIMEOUT_DISCHARGE (= 5ms)
                 * 3) Per tutta la durate del test controllo cosa fa la Vdc (con il pwm acceso):
                 * 3.1) se la Vdc scende sotto POST_IGBTSHORTVOLTAGE vuol dire che e' rotto un IGBT di quelli BASSI (disabilito in fault)
                 * 3.2) se la Vdc NON scende sotto POST_IGBTSHORTVOLTAGE vuol dire che e' tutto a posto e posso passare al test successivo */
           if (psPOSTRun->uwTimer >= POST_TIMEOUT_DISCHARGE)
           {
               psPOSTRun->uwTimer = 0 ;
               psPOSTRun->uwStatus = 20 ;
           }
           else
           {
               if (sMh_MotorDataOut.swDcBusValue < POST_IGBTSHORTVOLTAGE)
               {
                   psPOSTRun->uwFailedTest = POST_FAIL_LOIGBTBROKEN ;
                   psPOSTRun->uwStatus = 666 ;
               }
               psPOSTRun->uwTimer = psPOSTRun->uwTimer + 1 ;
           }
           break ;

       case 20: /* aspetto un 1 tick di fast */
           psPOSTRun->uwStatus = 25 ;
           break ;
       /* ============================================== */

       case 25: /* spengo gli IGBT alti */
           SelfTestPwmBridgeLayout(FALSE, FALSE, FALSE, FALSE, FALSE, FALSE) ;
           psPOSTRun->uwStatus = 30 ;
           break ;

       /* ============================================== */
       /* ================= IGBT BASSI ================= */
       /* ============================================== */
       case 30: /* Accendo i 3 IGBT bassi */
           SelfTestPwmBridgeLayout(TRUE, FALSE, TRUE, FALSE, TRUE, FALSE) ;
           psPOSTRun->uwFailedTest = POST_FAIL_NONE ;
           psPOSTRun->uwStatus = 35 ;
           break ;

       case 35: /* 1) PWM e' ACCESO
                 * 2) il test dura POST_TIMEOUT_DISCHARGE (= 5ms)
                 * 3) Per tutta la durate del test controllo cosa fa la Vdc (con il pwm acceso):
                 * 3.1) se la Vdc scende sotto POST_IGBTSHORTVOLTAGE vuol dire che e' rotto un IGBT di quelli ALTI (disabilito in fault)
                 * 3.2) se la Vdc NON scende sotto POST_IGBTSHORTVOLTAGE vuol dire che e' tutto a posto e posso passare al test successivo */
           if (psPOSTRun->uwTimer >= POST_TIMEOUT_DISCHARGE)
           {
                   psPOSTRun->uwTimer = 0 ;
                   psPOSTRun->uwStatus = 40 ;
           }
           else
           {
               if (sMh_MotorDataOut.swDcBusValue < POST_IGBTSHORTVOLTAGE)
               {
                   psPOSTRun->uwFailedTest = POST_FAIL_HIIGBTBROKEN ;
                   psPOSTRun->uwStatus = 666 ;
               }
               psPOSTRun->uwTimer = psPOSTRun->uwTimer + 1 ;
           }
           break ;

       case 40: /* aspetto un 1 tick */
           psPOSTRun->uwStatus = 45 ;
           break ;
       /* ============================================== */

       case 45: /* spengo gli IGBT bassi */
           SelfTestPwmBridgeLayout(FALSE, FALSE, FALSE, FALSE, FALSE, FALSE) ;
           psPOSTRun->uwStatus = 50 ;
           break ;

       /* ============================================== */
       /* ============== IGBT  INCROCIATI ============== */
       /* ============================================== */
       case 50: /* U_HI -> V_LO */
           if (sMh_MotorDataOut.swDcBusValue > POST_IGBTSHORTVOLTAGE)
           {
               SelfTestPwmBridgeLayout(FALSE, TRUE, TRUE, FALSE, FALSE, FALSE) ;
               psPOSTRun->uwStatus = 52 ;
           }
           break ;

       case 52: /* aspetto TIMOUT (ms) dopo che l'FPGA mi ha detto di essersi *
                 * abilitata per dare il tempo alla Vdc di scendere */
           if (psPOSTRun->uwTimer >= POST_TIMEOUT_DISCHARGE)
           {
               psPOSTRun->uwTimer  = 0 ;
               psPOSTRun->uwStatus = 54 ;
           }
           else
               psPOSTRun->uwTimer = psPOSTRun->uwTimer + 1 ;
           break ;

       case 54: /* controllo cosa fa la Vdc (con il pwm acceso):
                 * 1) se la  Vdc NON e' scesa vuol dire che o un IGBT e' rotto o c'e' una fase scollegata
                 * 2) disabilito */
           if (sMh_MotorDataOut.swDcBusValue < POST_IGBTSHORTVOLTAGE)
               psPOSTRun->sFlags.b.bUhi2VLoOK = TRUE ;
           else
               psPOSTRun->sFlags.b.bUhi2VLoOK = FALSE ;

           psPOSTRun->uwStatus = 56 ;
           break ;

       case 56:
           SelfTestPwmBridgeLayout(FALSE, FALSE, FALSE, FALSE, FALSE, FALSE) ;
           psPOSTRun->uwStatus = 60 ; /* aspetto un 1 tick */
           break ;

       /* -------------------------------------------------------- */
       case 60: /* U_HI -> W_LO */
           if (sMh_MotorDataOut.swDcBusValue > POST_IGBTSHORTVOLTAGE)
           {
               SelfTestPwmBridgeLayout(FALSE, TRUE, FALSE, FALSE, TRUE, FALSE) ;
               psPOSTRun->uwStatus = 62 ;
           }
           break ;

       case 62: /* aspetto TIMOUT (ms) dopo che l'FPGA mi ha detto di essersi *
                 * abilitata per dare il tempo alla Vdc di scendere */
           if (psPOSTRun->uwTimer >= POST_TIMEOUT_DISCHARGE)
           {
               psPOSTRun->uwTimer = 0 ;
               psPOSTRun->uwStatus = 64 ;
           }
           else
               psPOSTRun->uwTimer = psPOSTRun->uwTimer + 1 ;
           break ;

       case 64: /* controllo cosa fa la Vdc (con il pwm acceso):
                 * 1) se la  Vdc NON e' scesa vuol dire che o un IGBT e' rotto o c'e' una fase scollegata
                 * 2) disabilito
                 */
           if (sMh_MotorDataOut.swDcBusValue < POST_IGBTSHORTVOLTAGE)
               psPOSTRun->sFlags.b.bUhi2WLoOK = TRUE ;
           else
               psPOSTRun->sFlags.b.bUhi2WLoOK = FALSE ;

           psPOSTRun->uwStatus = 66 ;
           break ;

       case 66:
           SelfTestPwmBridgeLayout(FALSE, FALSE, FALSE, FALSE, FALSE, FALSE) ;
           psPOSTRun->uwStatus = 70 ; /* aspetto un 1 tick */
           break ;

       /* -------------------------------------------------------- */
       case 70: /* V_HI -> W_LO */
           if (sMh_MotorDataOut.swDcBusValue > POST_IGBTSHORTVOLTAGE)
           {
               SelfTestPwmBridgeLayout(FALSE, FALSE, FALSE, TRUE, TRUE, FALSE) ;
               psPOSTRun->uwStatus = 72 ;
           }
           break ;

       case 72: /* aspetto TIMOUT (ms) dopo che l'FPGA mi ha detto di essersi *
                 * abilitata per dare il tempo alla Vdc di scendere   */
           if (psPOSTRun->uwTimer >= POST_TIMEOUT_DISCHARGE)
           {
               psPOSTRun->uwTimer = 0 ;
               psPOSTRun->uwStatus = 74 ;
           }
           else
               psPOSTRun->uwTimer = psPOSTRun->uwTimer + 1 ;
           break ;

       case 74: /* controllo cosa fa la Vdc (con il pwm acceso):
                 * 1) se la  Vdc NON e' scesa vuol dire che o un IGBT e' rotto o c'e' una fase scollegata
                 * 2) disabilito
                 */
           if (sMh_MotorDataOut.swDcBusValue < POST_IGBTSHORTVOLTAGE)
               psPOSTRun->sFlags.b.bVhi2WLoOK = TRUE ;
           else
               psPOSTRun->sFlags.b.bVhi2WLoOK = FALSE ;

           psPOSTRun->uwStatus = 76 ;
           break ;

       case 76:
           SelfTestPwmBridgeLayout(FALSE, FALSE, FALSE, FALSE, FALSE, FALSE) ;
           psPOSTRun->uwStatus = 80 ; /* aspetto un 1 tick */
           break ;

       /* -------------------------------------------------------- */
       case 80: /* V_HI -> U_LO */
           if (sMh_MotorDataOut.swDcBusValue > POST_IGBTSHORTVOLTAGE)
           {
               SelfTestPwmBridgeLayout(TRUE, FALSE, FALSE, TRUE, FALSE, FALSE) ;
               psPOSTRun->uwStatus = 82 ;
           }
           break ;

       case 82: /* aspetto TIMOUT (ms) dopo che l'FPGA mi ha detto di essersi *
                 * abilitata per dare il tempo alla Vdc di scendere   */
           if (psPOSTRun->uwTimer >= POST_TIMEOUT_DISCHARGE)
           {
               psPOSTRun->uwTimer = 0 ;
               psPOSTRun->uwStatus = 84 ;
           }
           else
               psPOSTRun->uwTimer = psPOSTRun->uwTimer + 1 ;
           break ;

       case 84: /* controllo cosa fa la Vdc (con il pwm acceso):
                 * 1) se la  Vdc NON e' scesa vuol dire che o un IGBT e' rotto o c'e' una fase scollegata
                 * 2) disabilito
                 */
           if (sMh_MotorDataOut.swDcBusValue < POST_IGBTSHORTVOLTAGE)
               psPOSTRun->sFlags.b.bVhi2ULoOK = TRUE ;
           else
               psPOSTRun->sFlags.b.bVhi2ULoOK = FALSE ;

           psPOSTRun->uwStatus = 86 ;
           break ;

       case 86:
           SelfTestPwmBridgeLayout(FALSE, FALSE, FALSE, FALSE, FALSE, FALSE) ;
           psPOSTRun->uwStatus = 90 ; /* aspetto un 1 tick */
           break ;

       /* -------------------------------------------------------- */
       case 90: /* W_HI -> U_LO */
           if (sMh_MotorDataOut.swDcBusValue > POST_IGBTSHORTVOLTAGE)
           {
               SelfTestPwmBridgeLayout(TRUE, FALSE, FALSE, FALSE, FALSE, TRUE) ;
               psPOSTRun->uwStatus = 92 ;
           }
           break ;

       case 92: /* aspetto TIMOUT (ms) dopo che l'FPGA mi ha detto di essersi *
                 * abilitata per dare il tempo alla Vdc di scendere */
           if (psPOSTRun->uwTimer >= POST_TIMEOUT_DISCHARGE)
           {
               psPOSTRun->uwTimer = 0 ;
               psPOSTRun->uwStatus = 94 ;
           }
           else
               psPOSTRun->uwTimer = psPOSTRun->uwTimer + 1 ;
           break ;

       case 94: /* controllo cosa fa la Vdc (con il pwm acceso):
                 * 1) se la  Vdc NON e' scesa vuol dire che o un IGBT e' rotto o c'e' una fase scollegata
                 * 2) disabilito */
           if (sMh_MotorDataOut.swDcBusValue < POST_IGBTSHORTVOLTAGE)
               psPOSTRun->sFlags.b.bWhi2ULoOK = TRUE ;
           else
               psPOSTRun->sFlags.b.bWhi2ULoOK = FALSE ;

           psPOSTRun->uwStatus = 96 ;
           break ;

       case 96:
           SelfTestPwmBridgeLayout(FALSE, FALSE, FALSE, FALSE, FALSE, FALSE) ;
           psPOSTRun->uwStatus = 100 ; /* aspetto un 1 tick */
           break ;

       /* -------------------------------------------------------- */
       case 100: /* W_HI -> V_LO */
           if (sMh_MotorDataOut.swDcBusValue > POST_IGBTSHORTVOLTAGE)
           {
               SelfTestPwmBridgeLayout(FALSE, FALSE, TRUE, FALSE, FALSE, TRUE) ;
               psPOSTRun->uwStatus = 102 ;
           }
           break ;

       case 102: /* aspetto TIMOUT (ms) dopo che l'FPGA mi ha detto di essersi *
                  * abilitata per dare il tempo alla Vdc di scendere */
           if (psPOSTRun->uwTimer >= POST_TIMEOUT_DISCHARGE)
           {
               psPOSTRun->uwTimer = 0 ;
               psPOSTRun->uwStatus = 104 ;
           }
           else
               psPOSTRun->uwTimer = psPOSTRun->uwTimer + 1 ;
           break ;

       case 104: /* controllo cosa fa la Vdc (con il pwm acceso):
                  * 1) se la  Vdc NON e' scesa vuol dire che o un IGBT e' rotto o c'e' una fase scollegata
                  * 2) disabilito */
           if (sMh_MotorDataOut.swDcBusValue < POST_IGBTSHORTVOLTAGE)
               psPOSTRun->sFlags.b.bWhi2VLoOK = TRUE ;
           else
               psPOSTRun->sFlags.b.bWhi2VLoOK = FALSE ;

           psPOSTRun->uwStatus = 106 ;
           break ;

       case 106:
           SelfTestPwmBridgeLayout(FALSE, FALSE, FALSE, FALSE, FALSE, FALSE) ;
           psPOSTRun->uwStatus = 110 ; /* aspetto un 1 tick */
           break ;

       /* -------------------------------------------------------- */
       case 110: /* verifico se e' tutto a posto: tutto OK se TUTTI gli IGBT si accendono o NON si accendono (motore scollegato) */
           uwIgbtStatus = psPOSTRun->sFlags.w >> 1 ; // elimino il bit che mi dice che il Post e' in esecuzione

           if ((uwIgbtStatus & POST_ALLIGBT_STS) == POST_ALLIGBT_OK)
           {
               psPOSTRun->uwStatus = 150 ; // tutti e 6 gli IGBT si sono accesi: OK, vado avanti
           }
           else if ((uwIgbtStatus & POST_ALLIGBT_STS) == POST_ALLIGBT_NOT_OK)
           {   // tutti e 6 gli IGBT non si sono accesi: fasi motore molto probabilmente sconnesse
               psPOSTRun->uwFailedTest = POST_FAIL_PHASEDISCONNECT ;
               psPOSTRun->uwStatus = 666 ;
           }
           else
           {   // c'e' almeno un igbt che non si e' acceso: cerco di capire quale..
               if ((uwIgbtStatus & POST_UHI_STS) == POST_IGBT_NOT_OK)
                   uwBrokenIgbt = uwBrokenIgbt + (1 << 0) ; // Uhi non si e' acceso

               if ((uwIgbtStatus & POST_VHI_STS) == POST_IGBT_NOT_OK)
                   uwBrokenIgbt = uwBrokenIgbt + (1 << 1) ; // Vhi non si e' acceso

               if ((uwIgbtStatus & POST_WHI_STS) == POST_IGBT_NOT_OK)
                   uwBrokenIgbt = uwBrokenIgbt + (1 << 2) ; // Whi non si e' acceso

               if ((uwIgbtStatus & POST_ULO_STS) == POST_IGBT_NOT_OK)
                   uwBrokenIgbt = uwBrokenIgbt + (1 << 3) ; // Ulo non si e' acceso

               if ((uwIgbtStatus & POST_VLO_STS) == POST_IGBT_NOT_OK)
                   uwBrokenIgbt = uwBrokenIgbt + (1 << 4) ; // Vlo non si e' acceso

               if ((uwIgbtStatus & POST_WLO_STS) == POST_IGBT_NOT_OK)
                   uwBrokenIgbt = uwBrokenIgbt + (1 << 5) ; // Wlo non si e' acceso

               psPOSTRun->uwFailedTest = POST_FAIL_BROKENIGBT;
               psPOSTRun->uwIGBTFailed = uwBrokenIgbt;
               psPOSTRun->uwStatus = 666 ;
           }
           break ;

       /* ============================================== */
       /* ========== RESISTENZA  DI FRENATURA ========== */
       /* ============================================== */
       case 150:
           /* aspetto che la Vdc sia stabile */
           if (sMh_MotorDataOut.swDcBusValue > POST_IGBTSHORTVOLTAGE)
               psPOSTRun->uwStatus = 155 ;
           break ;

       case 155: /* Accendo le resistenza di frenatura (se c'e') */
           if (*sMh_MotorDataIn.puwBrakeRValue && sMotorHandlerRun.flags.b.bBrakeInstalled)
           {
               /* forzo accensione */
#ifdef _HW_CT
               Mh_ForceCommandEx(MH_CMD_SET_BRAKE_DUTY, 1000);
#endif
               Mh_ForceCommandEx(MH_CMD_FORCEBRAKEON, TRUE);
               psPOSTRun->uwStatus = 160 ;
           }
           else
               psPOSTRun->uwStatus = 165 ;
           break ;

       case 160:
           if (sMh_MotorDataOut.swDcBusValue < POST_BRAKEONVOLTAGE)
               psPOSTRun->uwStatus = 165 ;
           else
           {   /* aspetto che la Vdc scenda */
               if (psPOSTRun->uwTimer >= POST_TIMEOUT_BRAKE_DISCHRG)
               {   /* e' passato il timeout e la Vdc NON e' scesa: la resistenza non frena */
                   psPOSTRun->uwTimer = 0 ;
                   psPOSTRun->uwFailedTest = POST_FAIL_BRAKEUNIT ;
                   /* spengo frenatura */
#ifdef _HW_CT
                   Mh_ForceCommandEx(MH_CMD_SET_BRAKE_DUTY, 0);
#endif
                   Mh_ForceCommandEx(MH_CMD_FORCEBRAKEON, FALSE);
                   psPOSTRun->uwStatus = 666 ;
               }
               else
                   psPOSTRun->uwTimer = psPOSTRun->uwTimer + 1 ;
           }
           break ;

       case 165: /* spengo frenatura */
#ifdef _HW_CT
           Mh_ForceCommandEx(MH_CMD_SET_BRAKE_DUTY, 0);
#endif
           Mh_ForceCommandEx(MH_CMD_FORCEBRAKEON, FALSE);
           psPOSTRun->uwStatus  = 999 ;
           break ;
       /* ============================================== */

       /* ============================================== */
       case 666: /* TEST ENDED: NOT OK */
           psPOSTRun->sFlags.b.bExecute = FALSE ;
           break ;

       case 999: /* TEST ENDED: OK */
           psPOSTRun->sFlags.b.bExecute = FALSE ;
           break ;
       /* ============================================== */
   }

   /* watchdog per FSM */
   if(psPOSTRun->uwStatus != psPOSTRun->uwPrevStatus)
   {
       psPOSTRun->uwPrevStatus = psPOSTRun->uwStatus;
       psPOSTRun->uwWDTTimer = psPOSTRun->uwTimeoutCharge;
   }
   else if(psPOSTRun->uwWDTTimer>0)
       psPOSTRun->uwWDTTimer--;
   else
   {
       psPOSTRun->uwFailedTest = POST_FAIL_GENFAULT ;
       psPOSTRun->sFlags.b.bExecute = FALSE ;
   }

   return TRUE ;
}

//***************************************************************************
// Load binary code in the DSP

static BOOL DSPHLoad(HPUBYTE hpubBin, UWORD uwSize, UWORD uwStartTag)
{
    UWORD uwData;

    assert((uwSize%2)==0);

    // first word contains compatibility level,
    // matching is mandatory
    uwData =(UWORD)(*hpubBin++);
    uwData|=((UWORD)(*hpubBin++))<<8;
    uwSize-=2;

    if(uwData!=FPGA_CPUH_INSRAM_CMPLEV)
        return FALSE;

    FPGA_DSPH_SET_HALT = TRUE;

    while(uwSize)
    {
        uwData =(UWORD)(*hpubBin++);
        uwData|=((UWORD)(*hpubBin++))<<8;
        FPGA_CPUH_INSRAM_CODE=uwData;
        uwSize-=2;
    }

    FPGA_CPUH_INSRAM_STROBE=uwStartTag;

    // code loaded successfully
    return TRUE;
}

//***************************************************************************
// Setup for post-loading custom DSP code

BOOL Mh_PlcDSPLoad(HPUBYTE hpubBin, UWORD uwSize)
{
       // must be executed only at preboot
   if(!bSysStatBooting)
       return FALSE;

       // set address and size for post-loading
   hpubDSPCustomAddr=hpubBin;
   uwDSPCustomSize  =uwSize;

   if(hpubBin==NULL)
       bDSPAdvance = (BOOL)uwSize;

   return TRUE;
}

//***************************************************************************
// Setup for post-loading custom DSP output parameters

BOOL Mh_PlcDSPDBLoad(HPUBYTE hpubBin, UWORD uwSize)
{
       // must be executed only at preboot
   if(!bSysStatBooting)
       return FALSE;

       // set address and length for post-loading
   hpubDSPCustomDBAddr=(LOCDSPPARDB *)hpubBin;
   uwDSPCustomDBLength=uwSize/sizeof(LOCDSPPARDB);

   return TRUE;
}

//***************************************************************************
// Setup DSP using in firmware: 0 - Standard, 1 - Advance

BOOL Mh_PlcDSPConfig(BOOL bAdvance)
{
        // must be executed only at preboot
    if(!bSysStatBooting)
        return FALSE;

    bDSPAdvance = bAdvance;

    return TRUE;
}
