/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : DriveTaskController.c                                      */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Task configuration for drive controller                    */
/*                                                                          */
/****************************************************************************/
#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "drive\DriveTaskController.h"
#include "drive\MotorParameters.h"

#include "drive\Positioner.h"
#include "drive\SpaceSpeedCntrLp.h"

#include "drive\MotorHandler.h"
#include "drive\Deflux.h"
#include "drive\ThermalModel.h"
#include "drive\PiDcBusCntrLp32bit.h"

#include "drive\MotionController.h"

#include "plc\Plc.h"
#include "common\CommonController.h"
#include "drive\EncoderManager.h"
#include "drive\EncoderEFSeek.h"
#include "drive\BackEmfEncoder.h"
#if CFG_CANDRV_CMDMGR
#include "bus\canopen\CanOpenCommandMgr.h"
#endif
#include "bus\ethercat\ECATCommandMgr.h"
#ifdef _INFINEON_
#include "IOExpBrdManager.h"
#endif
#ifdef _HW_DC
#include "drive\PSUManager.h"
#endif
#include "common\TaskScheduler.h"
#include "common\ParametersCheck.h"
#include "common\ParametersCheckCodes.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
/*
#if defined(_CRS_DBG)
#if CRS_DBGDSK
#pragma GCC optimize (0) // crs_dbg
#else
#pragma GCC optimize (2)
#endif
#else
#pragma GCC optimize (2)
#endif
*/
#pragma GCC optimize (2)
//***************************************************************************
// Globals

DRVTSKCTRL_PARAM sDrvTskCtrlParams;
const DRVTSKCTRL_NEW_PARAM  sDrvTskCtrlDefParams={DRVTSKCTRL_MO_MOTORCONTROL};

DRVTSKCTRL_PLC_CONFIG sDrvTskCtrlPlcConfig=
{
    0x00,

    {DRVTSKCTRL_TO_AUTO, DRVTSKCTRL_TO_AUTO, DRVTSKCTRL_TO_AUTO, DRVTSKCTRL_TO_AUTO,
     DRVTSKCTRL_TO_AUTO, DRVTSKCTRL_TO_AUTO, DRVTSKCTRL_TO_AUTO, DRVTSKCTRL_TO_AUTO,
     DRVTSKCTRL_TO_AUTO, DRVTSKCTRL_TO_AUTO, DRVTSKCTRL_TO_AUTO, DRVTSKCTRL_TO_AUTO,
     DRVTSKCTRL_TO_AUTO, DRVTSKCTRL_TO_AUTO, DRVTSKCTRL_TO_AUTO, DRVTSKCTRL_TO_AUTO},
};

UBYTE ubDrvTskCtrlMainOpSelected;

//***************************************************************************
// Locals

static DRVTSKCTRL_PLC_CONFIG sLocParams;
static UBYTE ubMainOperationMask;
static UBYTE ubActualConfig;
static ENCMGR_SPACEFEEDBACK * psFbEncoder;
static ENCMGR_SPACEFB_EXT * psFbEncExt;

//***************************************************************************
// Drive task list

#define MO_MOTORCTRL                0x01
#define MO_AFE                      0x02
#define MO_PSU                      0x04

#define MO_NONE                     0x00
#define MO_ALL                      (MO_MOTORCTRL|MO_AFE|MO_PSU)

typedef struct
{
    UBYTE           ubSeqNumber;
    UBYTE           ubMainOperation;
    UBYTE *         pubTaskOption;
    BOOL            (* pfTaskInit)( UWORD );
    UWORD           uwTaskParam;
} DRVTASKINIT;

const DRVTASKINIT tDrvTaskInitList[]=
{   
   {  50, MO_MOTORCTRL|MO_AFE,  NULL,                                       &Em_EncMngrInit, ENCMGR_INIT_PLCOPTIONS},
   {  60, MO_MOTORCTRL       ,  &sLocParams.sTo.ubEncoderManager,           &Em_EncMngrInit, ENCMGR_INIT_EPLATE},
   {  70, MO_MOTORCTRL       ,  NULL,                                       &MotPar_Init, 0},
   {  80, MO_NONE            ,  &sLocParams.sTo.ubEncoderManager,           &Em_EncMngrInit, ENCMGR_INIT_PLC},
   {  90, MO_MOTORCTRL|MO_AFE,  &sLocParams.sTo.ubMotorHandler,             &Mh_MotorHandlerInit, MH_INIT_INPUT},
   { 100, MO_MOTORCTRL       ,  &sLocParams.sTo.ubEncoderManager,           &Em_EncMngrInit, ENCMGR_INIT_CORE},
//    { 110,              // do not use
   { 120, MO_MOTORCTRL|MO_AFE,  &sLocParams.sTo.ubThermalModel,             &Tm_ThermalModelInit, 0},
   { 130, MO_MOTORCTRL       ,  &sLocParams.sTo.ubPositioner,               &Po_PositionerInit, 0},
   { 140, MO_MOTORCTRL       ,  &sLocParams.sTo.ubSpaceSpeedCntrLp,         &Ss_ControlLoopInit, 0},
   { 150, MO_AFE			,  	&sLocParams.sTo.ubModPiDcBusCntrlLp32bit,   &DcBusCntrl_ControlLoopInit, 0},
   { 160, MO_MOTORCTRL       ,  &sLocParams.sTo.ubEncoderManager,           &Em_EncMngrInit, ENCMGR_INIT_HOOK},
   { 170, MO_MOTORCTRL|MO_AFE,  &sLocParams.sTo.ubMotorHandler,             &Mh_MotorHandlerInit, MH_INIT_OUTPUT},
   { 180, MO_MOTORCTRL       ,  &sLocParams.sTo.ubMotionController,         &MotCtrlInit, 0},
//    { 190,              // do not use
   { 200, MO_MOTORCTRL|MO_AFE,  NULL,                                       &PlcInit, PLC_INIT_FAST},
};

//***************************************************************************
// Local prototypes

static void setdefaults(void);
static BOOL parameterscheck(void);

//***************************************************************************
// Init

BOOL DrvTskCtrl_Init(void)
{
    const DRVTASKINIT * ptLst;
    UWORD uwCnt;
    BOOL bInstall;
    BOOL bFailFlag=FALSE;

        // local copy of task configuration
    sLocParams=sDrvTskCtrlPlcConfig;

#if (defined(_DEBUG_TRACES) && defined (_CRS_DBG))
#if _CRS_DBGDSK
        xil_printf("\r\n");
#endif
#endif

        // add bg task for parameters checking
    if(!TaskSched_AddBackgroundTask((void (*)(void))&parameterscheck))
    {
#if (defined(_DEBUG_TRACES) && defined (_CRS_DBG))
#if _CRS_DBGDSK
        xil_printf("DriveTaskController::DrvTskCtrl_Init::[0] \r\n");
#endif
#endif
        return FALSE;
    }
        // check parameters validity
    if(!parameterscheck())
    {
#if (defined(_DEBUG_TRACES) && defined (_CRS_DBG))
#if _CRS_DBGDSK
        xil_printf("DriveTaskController::DrvTskCtrl_Init::[1] \r\n");
#endif
#endif
        return FALSE;
    }
        // get main operation mode
    switch(sDrvTskCtrlParams.sNew.ubMainOperation)
    {
        case DRVTSKCTRL_MO_MOTORCONTROL:
            ubMainOperationMask=MO_MOTORCTRL;

            // ========= Emergency  function =========
            sEm_EncMngrIn.psCtrlHandlers   = &sMotCtrlHandlers ;
            sTm_ThModIn.psCtrlHandlers     = &sMotCtrlHandlers ;
            sMh_MotorDataIn.psCtrlHandlers = &sMotCtrlHandlers ;
            sPlcIn.psCtrlHandlers          = &sMotCtrlHandlers ;
#if CFG_CANDRV_CMDMGR
            sCanOpenCMIn.psCtrlHandlers    = &sMotCtrlHandlers ;
#endif

#if CFG_ECAT
            sEcatCMIn.psCtrlHandlers       = &sMotCtrlHandlers ;
#endif // cfg_ecat

#ifdef _INFINEON_
#if !defined(_HW_DC) && !defined(_HW_CT)
            sIOExpBrdIn.psCtrlHandlers     = &sMotCtrlHandlers ;
#endif
#endif

#if CFG_PSU_MNGR
            sPSUBrdIn.psCtrlHandlers       = &sMotCtrlHandlers ;
#endif // cfg_psu_mngr

            sPo_Handlers.psCtrlHandlers    = &sMotCtrlHandlers ;
            break;

        case DRVTSKCTRL_MO_PSU:
            ubMainOperationMask=MO_PSU;

            sPlcIn.psCtrlHandlers          = &sMotCtrlHandlers ;
            //sCanOpenCMIn.psCtrlHandlers    = &sMotCtrlHandlers ;
#if CFG_ECAT
            sEcatCMIn.psCtrlHandlers       = &sMotCtrlHandlers ;
#endif // cfg_ecat

#if CFG_PSU_MNGR
            sPSUBrdIn.psCtrlHandlers       = &sMotCtrlHandlers ;
#endif // cfg_psu_mngr

            break;
        default:
            assert(FALSE);
    }

        // set mode of operation for system management
    ubDrvTskCtrlMainOpSelected=sDrvTskCtrlParams.sNew.ubMainOperation;

        // set feedback encoder
    if(sDrvTskCtrlPlcConfig.sTo.ubEncFeedBackSelection==DRVTSKCTRL_TO_PLC)
    {
        psFbEncoder=&sEm_PlcFbk2CntrLoop;
        psFbEncExt=&sEm_PlcFbk2CLExt;
    }
    else
    {
        psFbEncoder=&sEm_Fbk2CntrLoop;
        psFbEncExt=&sEm_Fbk2CLExt;
    }

        // setup default inputs
    setdefaults();

        // begin task configuration
    for(uwCnt=0,ptLst=tDrvTaskInitList;uwCnt<sizeof(tDrvTaskInitList)/sizeof(DRVTASKINIT);uwCnt++,ptLst++)
    {
            // if selected position for real time hook task then add before the specific task
        if(ptLst->ubSeqNumber==sLocParams.ubRtHookPosition)
            bFailFlag |= !PlcInit(PLC_INIT_RTHOOK);

            // check main operation mode mask
        bInstall=(ptLst->ubMainOperation & ubMainOperationMask);

            // then check for overrides
        if(ptLst->pubTaskOption!=NULL)
        {       // force off
            if(*ptLst->pubTaskOption==DRVTSKCTRL_TO_OFF)
                bInstall=FALSE;
                // force user inputs
            else if(*ptLst->pubTaskOption==DRVTSKCTRL_TO_PLC)
                bInstall=TRUE;
                // check the last valid option
            else if(*ptLst->pubTaskOption!=DRVTSKCTRL_TO_AUTO)
            {
#if (defined(_DEBUG_TRACES) && defined (_CRS_DBG))
#if _CRS_DBGDSK
                xil_printf("DriveTaskController::DrvTskCtrl_Init::[2] \r\n");
#endif
#endif
                return FALSE;
            }
        }

#if (defined(_DEBUG_TRACES) && defined (_CRS_DBG))
#if _CRS_DBGDSK
        xil_printf("DriveTaskController::DrvTskCtrl_Init::\tCnt[ %d ]\tSeqNumber [ %d ]\tInstall [ %d ]\t", uwCnt, (UWORD)ptLst->ubSeqNumber, (UWORD)bInstall);
#endif
#endif

        if(bInstall)
        {
                // calls task init function with specified parameter
            bFailFlag |= !(*ptLst->pfTaskInit)(ptLst->uwTaskParam);
        	bInstall = FALSE ; // paranoic clear
        }

#if (defined(_DEBUG_TRACES) && defined (_CRS_DBG))
#if _CRS_DBGDSK
        xil_printf("FailFlag [ %d ]\r\n", (UWORD)bFailFlag);
#endif
#endif
    }

        // default to torque mode, this ensure that all connections
        // are setup after all tasks are configured, that is when
        // all hooks and plc options are well known
    if(sDrvTskCtrlParams.sNew.ubMainOperation==DRVTSKCTRL_MO_MOTORCONTROL)
        DrvTskCtrl_Config(DRVTSKCTRL_OP_TORQUE);
    else
        DrvTskCtrl_Config(DRVTSKCTRL_OP_PSU);

    return !bFailFlag;
}

//***************************************************************************
// Setup default data input for all tasks

static void setdefaults(void)
{
       // Encoder manager
   sEm_EncMngrIn.sIRefIn               = &sEm_EncMngrOut.sIRefOut ;
   sEm_EncMngrIn.sPowerStageCtrlIn     = &sEm_EncMngrOut.sPowerStageCtrlOut ;
   sEm_EncMngrIn.psMotorData           = &sMh_MotorDataOut ;
   sEm_EncMngrIn.sSpdLoopIRefIn        = &sSS_CntrLoopOut.sIRef ;

        // BackEmf Encoder
    sBe_EmfEncIn.psRef                  = &sPo_PostnerOut.sDemand ;
    sBe_EmfEncIn.psBackEmfData          = &sMh_MotorDataOut.sBackEmfData ;
    sBe_EmfEncIn.psPowerStageStatus     = &sMh_MotorDataOut.sPowerStageSts ;
    sBe_EmfEncIn.psMotorHandlers        = &sMh_MotorHandlers ;
    sBe_EmfEncIn.psVdcBusFiltered       = &sTm_ThModOut.sOutValue.swDcBusAvrg;

        // Motor Handler
    sMh_MotorDataIn.psPStageCtrl        = &sMotCtrl_Out.sPowerStageCtrl ;
    sMh_MotorDataIn.psIRef2Use          = &sSS_CntrLoopOut.sIRef ;
    sMh_MotorDataIn.puwElecAngle        = &psFbEncoder->uwElecAngle ;
    sMh_MotorDataIn.pslFilteredSpeed    = &psFbEncoder->sEncData.slSpeed ;
    sMh_MotorDataIn.puwBrakeRValue      = &sTm_ThModOut.uwBrakeResistorValue ;
    sMh_MotorDataIn.pswElecSpeed        = &psFbEncoder->swElecSpeed ;

        // Deflux
    sDflx_In.pswActualVdc               = &sTm_ThModOut.sOutValue.swDcBusAvrg ;
    sDflx_In.pswActualSpd               = &psFbEncoder->swElecSpeed ;
    sDflx_In.psPowerStageStatus         = &sMh_MotorDataOut.sPowerStageSts ;

        // Thermal model
    sTm_ThModIn.psMotorData             = &sMh_MotorDataOut ;
    sTm_ThModIn.psMotorHandlers         = &sMh_MotorHandlers ;

        // Motion controller
    sMotCtrl_In.psMotorHandler          = &sMh_MotorHandlers ;
    sMotCtrl_In.psPowerStageStatus      = &sMh_MotorDataOut.sPowerStageSts ;
    sMotCtrl_In.psMotCtrl_In            = &sMotCtrl_UsrControl ;
    sMotCtrl_In.psFbk2CntrLoop          = psFbEncoder ;
    sMotCtrl_In.psPoOut                 = &sPo_PostnerOut;

        // Space Speed control loop
    sSS_CntrLoopIn.psRef                = &sPo_PostnerOut.sDemand ;
    sSS_CntrLoopIn.psFbk                = psFbEncoder ;
    sSS_CntrLoopIn.psIqLimit            = &sMh_MotorDataOut.sIqLimit ;
    sSS_CntrLoopIn.psILimitActive       = &sMh_MotorDataOut.sCurrLimFlags ;
    sSS_CntrLoopIn.psMotCtrlStatus      = &sMotCtrl_OStatus ;
    sSS_CntrLoopIn.psCurrentRefs        = &sMh_MotorDataUsrInIRef ;

        // Positioner
    psPo_InFeedBack                     = &psFbEncoder->sEncData ;
    psPo_InSSCtrlLoop                   = &sSS_CntrLoopOut;
    psPo_PostnerIn                      = &sPo_UsrPostnerIn ;
    psPo_ILimitActive                   = &sMh_MotorDataOut.sCurrLimFlags ;
    psPo_InFeedbackExt                  = &sEm_Fbk2CLExt ;

}

//***************************************************************************
// Setup proper operation of various task and return the previous setup one

UBYTE DrvTskCtrl_Config(UBYTE ubOperation)
{
    UBYTE ubPreviousCfg=ubActualConfig;

    ubActualConfig=ubOperation;

    switch(ubOperation)
    {
        case DRVTSKCTRL_OP_TORQUE:
            // ###########################################################
            // ######################  T O R Q U E  ######################
            // ###########################################################
            assert(ubMainOperationMask & MO_MOTORCTRL);

            /* =============== Motion Controller ================ */
            if(sLocParams.sTo.ubMotionController == DRVTSKCTRL_TO_AUTO)
                sMotCtrl_In.psMotCtrl_In = &sMotCtrl_UsrControl ; // DRVTSKCTRL_TO_AUTO: firmware control word
            else
                sMotCtrl_In.psMotCtrl_In = &sMotCtrl_PlcControl ; // DRVTSKCTRL_TO_PLC: plc control word

            /* =================== Positioner =================== */
            if(sLocParams.sTo.ubPositioner == DRVTSKCTRL_TO_AUTO)
                psPo_PostnerIn = &sPo_UsrPostnerIn ; // DRVTSKCTRL_TO_AUTO: positioner input from fieldbus/user
            else
                psPo_PostnerIn = &sPo_PlcPostnerIn ; // DRVTSKCTRL_TO_PLC: positioner input from plc


            /* ================== Control Loop ================== */
            if(sLocParams.sTo.ubSpaceSpeedCntrLp == DRVTSKCTRL_TO_AUTO)
            {
                sSS_CntrLoopIn.psRef = &sPo_PostnerOut.sDemand ; // DRVTSKCTRL_TO_AUTO: control loop input from positioner_out
                sSS_CntrLoopIn.psMotCtrlStatus = &sMotCtrl_OStatus ;
                sSS_CntrLoopIn.psCurrentRefs = &sMh_MotorDataUsrInIRef ;
            }
            else
            {
                sSS_CntrLoopIn.psRef = &sSS_PlcPidRef ; // DRVTSKCTRL_TO_PLC: control loop input from plc
                sSS_CntrLoopIn.psMotCtrlStatus = &sSS_PlcMotCtrlStatus ;
                sSS_CntrLoopIn.psCurrentRefs = &sMh_MotorDataPlcInIRef ;
            }


            /* ================= Motor Handler ================== */
            if(sLocParams.sTo.ubMotorHandler == DRVTSKCTRL_TO_AUTO)
            {   // DRVTSKCTRL_TO_AUTO: motor handler input from fieldbus/user
                sMh_MotorDataIn.puwElecAngle = &psFbEncoder->uwElecAngle ;
                sMh_MotorDataIn.pswElecSpeed = &psFbEncoder->swElecSpeed ;
                sMh_MotorDataIn.pslFilteredSpeed = &psFbEncoder->sEncData.slSpeed ;
                sDflx_In.pswActualSpd = &psFbEncoder->swElecSpeed ;

                if (sEm_EncMngrOut.flags.b.bRequireIRefHook)
                {
                    sEm_EncMngrIn.sIRefIn = &sMh_MotorDataUsrInIRef ;
                    sMh_MotorDataIn.psIRef2Use = &sEm_EncMngrOut.sIRefOut;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sMotCtrl_Out.sPowerStageCtrl ;
                    sMh_MotorDataIn.psPStageCtrl = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                }
                else
                {
                    sEm_EncMngrIn.sIRefIn = &sEm_EncMngrOut.sIRefOut ;
                    sMh_MotorDataIn.psIRef2Use = &sMh_MotorDataUsrInIRef ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                    sMh_MotorDataIn.psPStageCtrl = &sMotCtrl_Out.sPowerStageCtrl ;
                }
            }
            else
            {   // DRVTSKCTRL_TO_PLC: motor handler input from plc
                sMh_MotorDataIn.puwElecAngle = &uwMh_MotoDataPlcInElecAngle ;
                sMh_MotorDataIn.pswElecSpeed = &swMh_MotoDataPlcInElecSpeed ;
                sMh_MotorDataIn.pslFilteredSpeed = &slMh_MotoDataPlcInFilteredSpeed ;
                sDflx_In.pswActualSpd = &swMh_MotoDataPlcInElecSpeed ;

                if (sEm_EncMngrOut.flags.b.bRequireIRefHook)
                {
                    sEm_EncMngrIn.sIRefIn = &sMh_MotorDataPlcInIRef ;
                    sMh_MotorDataIn.psIRef2Use = &sEm_EncMngrOut.sIRefOut ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sMh_MotorDataPlcPStageCtrl ;
                    sMh_MotorDataIn.psPStageCtrl = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                }
                else
                {
                    sEm_EncMngrIn.sIRefIn = &sEm_EncMngrOut.sIRefOut ;
                    sMh_MotorDataIn.psIRef2Use = &sMh_MotorDataPlcInIRef ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                    sMh_MotorDataIn.psPStageCtrl = &sMh_MotorDataPlcPStageCtrl ;
                }
            }


            /* ================ PI16bit Handler ================= */
            if(sLocParams.sTo.ubModPiDcBusCntrlLp32bit == DRVTSKCTRL_TO_PLC)
            {   // consento di usare il PI 16bit con il plc (ZF)
                // DRVTSKCTRL_TO_PLC: PI input from plc
                sDcBusCntrl_In.pswRef          = &sDcBusCntrl_Plc.swDcBusRef ;
                sDcBusCntrl_In.pswFbk          = &sDcBusCntrl_Plc.swDcBusFbk  ;
                sDcBusCntrl_In.pswVoltage      = &sDcBusCntrl_Plc.swVoltage ;
                sDcBusCntrl_In.pubVoltageValid = &sDcBusCntrl_Plc.ubVoltageValid  ;
            }

            /* ================ BackEMF ========================= */
            sBe_EmfEncIn.psRef = sSS_CntrLoopIn.psRef;

            // ========= Add and delete limit to the list =======
            (*sMh_MotorHandlers.pfDeleteLimitFromList)(MH_LIMITLIST_IQ, &sSS_CntrLoopWks.sUsrOutVal32); /* delete space speed current limits (do not install) */
            (*sMh_MotorHandlers.pfDeleteLimitFromList)(MH_LIMITLIST_ID, &sMh_MotorDataParam.sIdLimit);  /* delete Torque ID current limits (avoid to install it two times) */
            (*sMh_MotorHandlers.pfDeleteLimitFromList)(MH_LIMITLIST_IQ, &sMh_MotorDataParam.sIqLimit);  /* delete Torque IQ current limits (avoid to install it two times) */
            (*sMh_MotorHandlers.pfAdd2LimitList)(MH_LIMITLIST_ID, &sMh_MotorDataParam.sIdLimit);        /* install Torque ID current limits */
            (*sMh_MotorHandlers.pfAdd2LimitList)(MH_LIMITLIST_IQ, &sMh_MotorDataParam.sIqLimit);        /* install Torque IQ current limits */
            break;


        case DRVTSKCTRL_OP_POSITIONER:
            // ###########################################################
            // ##################  P O S I T I O N E R  ##################
            // ###########################################################
            assert(ubMainOperationMask & MO_MOTORCTRL);

            /* =============== Motion Controller ================ */
            if(sLocParams.sTo.ubMotionController == DRVTSKCTRL_TO_AUTO)
                sMotCtrl_In.psMotCtrl_In = &sMotCtrl_UsrControl ; // DRVTSKCTRL_TO_AUTO: firmware control word
            else
                sMotCtrl_In.psMotCtrl_In = &sMotCtrl_PlcControl ; // DRVTSKCTRL_TO_PLC: plc control word


            /* =================== Positioner =================== */
            if(sLocParams.sTo.ubPositioner == DRVTSKCTRL_TO_AUTO)
                psPo_PostnerIn = &sPo_UsrPostnerIn ; // DRVTSKCTRL_TO_AUTO: positioner input from fieldbus/user
            else
                psPo_PostnerIn = &sPo_PlcPostnerIn ; // DRVTSKCTRL_TO_PLC: positioner input from plc


            /* ================== Control Loop ================== */
            if(sLocParams.sTo.ubSpaceSpeedCntrLp == DRVTSKCTRL_TO_AUTO)
            {
                sSS_CntrLoopIn.psRef = &sPo_PostnerOut.sDemand ; // DRVTSKCTRL_TO_AUTO: control loop input from positioner_out
                sSS_CntrLoopIn.psMotCtrlStatus = &sMotCtrl_OStatus ;
                sSS_CntrLoopIn.psCurrentRefs = &sMh_MotorDataUsrInIRef ;
            }
            else
            {
                sSS_CntrLoopIn.psRef = &sSS_PlcPidRef ; // DRVTSKCTRL_TO_PLC: control loop input from plc
                sSS_CntrLoopIn.psMotCtrlStatus = &sSS_PlcMotCtrlStatus ;
                sSS_CntrLoopIn.psCurrentRefs = &sMh_MotorDataPlcInIRef ;
            }


            /* ================= Motor Handler ================== */
            if(sLocParams.sTo.ubMotorHandler == DRVTSKCTRL_TO_AUTO)
            {   // DRVTSKCTRL_TO_AUTO: motor handler input from pid_out
                sMh_MotorDataIn.puwElecAngle = &psFbEncoder->uwElecAngle ;
                sMh_MotorDataIn.pswElecSpeed = &psFbEncoder->swElecSpeed ;
                sMh_MotorDataIn.pslFilteredSpeed = &psFbEncoder->sEncData.slSpeed ;
                sDflx_In.pswActualSpd = &psFbEncoder->swElecSpeed ;

                if (sEm_EncMngrOut.flags.b.bRequireIRefHook)
                {
                    sEm_EncMngrIn.sIRefIn = &sSS_CntrLoopOut.sIRef ;
                    sMh_MotorDataIn.psIRef2Use = &sEm_EncMngrOut.sIRefOut ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sMotCtrl_Out.sPowerStageCtrl ;
                    sMh_MotorDataIn.psPStageCtrl = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                }
                else
                {
                    sEm_EncMngrIn.sIRefIn = &sEm_EncMngrOut.sIRefOut ;
                    sMh_MotorDataIn.psIRef2Use = &sSS_CntrLoopOut.sIRef ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                    sMh_MotorDataIn.psPStageCtrl = &sMotCtrl_Out.sPowerStageCtrl ;
                }
            }
            else
            {   // DRVTSKCTRL_TO_PLC: motor handler input from plc
                sMh_MotorDataIn.puwElecAngle = &uwMh_MotoDataPlcInElecAngle ;
                sMh_MotorDataIn.pswElecSpeed = &swMh_MotoDataPlcInElecSpeed ;
                sMh_MotorDataIn.pslFilteredSpeed = &slMh_MotoDataPlcInFilteredSpeed ;
                sDflx_In.pswActualSpd = &swMh_MotoDataPlcInElecSpeed ;

                if (sEm_EncMngrOut.flags.b.bRequireIRefHook)
                {
                    sEm_EncMngrIn.sIRefIn = &sMh_MotorDataPlcInIRef ;
                    sMh_MotorDataIn.psIRef2Use = &sEm_EncMngrOut.sIRefOut ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sMh_MotorDataPlcPStageCtrl ;
                    sMh_MotorDataIn.psPStageCtrl = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                }
                else
                {
                    sEm_EncMngrIn.sIRefIn = &sEm_EncMngrOut.sIRefOut ;
                    sMh_MotorDataIn.psIRef2Use = &sMh_MotorDataPlcInIRef ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                    sMh_MotorDataIn.psPStageCtrl = &sMh_MotorDataPlcPStageCtrl ;
                }
            }


            /* ========== PI-16bit Handler (PLC only) =========== */
            if(sLocParams.sTo.ubModPiDcBusCntrlLp32bit == DRVTSKCTRL_TO_PLC)
            {   // consento di usare il PI 16bit con il plc (ZF)
                // DRVTSKCTRL_TO_PLC: PI input from plc
                sDcBusCntrl_In.pswRef          = &sDcBusCntrl_Plc.swDcBusRef ;
                sDcBusCntrl_In.pswFbk          = &sDcBusCntrl_Plc.swDcBusFbk  ;
                sDcBusCntrl_In.pswVoltage      = &sDcBusCntrl_Plc.swVoltage ;
                sDcBusCntrl_In.pubVoltageValid = &sDcBusCntrl_Plc.ubVoltageValid  ;
            }


            /* ================ BackEMF ========================= */
            sBe_EmfEncIn.psRef = sSS_CntrLoopIn.psRef;

            // ========= Add and delete limit to the list =======
            (*sMh_MotorHandlers.pfDeleteLimitFromList)(MH_LIMITLIST_ID, &sMh_MotorDataParam.sIdLimit);  /* delete Torque ID current limits */
            (*sMh_MotorHandlers.pfDeleteLimitFromList)(MH_LIMITLIST_IQ, &sMh_MotorDataParam.sIqLimit);  /* delete Torque IQ current limits */
            (*sMh_MotorHandlers.pfDeleteLimitFromList)(MH_LIMITLIST_IQ, &sSS_CntrLoopWks.sUsrOutVal32); /* delete space speed current limits (avoid to install it two times) */
            (*sMh_MotorHandlers.pfAdd2LimitList)(MH_LIMITLIST_IQ, &sSS_CntrLoopWks.sUsrOutVal32);       /* install space speed current limits */
            break;


        case DRVTSKCTRL_OP_EFS_IDR_AND_PLOOP:
            // ###########################################################
            // ######  EFS with Id ramp and position control loop  #######
            // ###########################################################
            assert(ubMainOperationMask & MO_MOTORCTRL);

            /* =============== Motion Controller ================ */
            if(sLocParams.sTo.ubMotionController == DRVTSKCTRL_TO_AUTO)
                sMotCtrl_In.psMotCtrl_In = &sMotCtrl_UsrControl ; // DRVTSKCTRL_TO_AUTO: firmware control word
            else
                sMotCtrl_In.psMotCtrl_In = &sMotCtrl_PlcControl ; // DRVTSKCTRL_TO_PLC: plc control word


            /* =================== Positioner =================== */
            if(sLocParams.sTo.ubPositioner == DRVTSKCTRL_TO_AUTO)
                psPo_PostnerIn = &sPo_UsrPostnerIn ; // DRVTSKCTRL_TO_AUTO: positioner input from fieldbus/user
            else
                psPo_PostnerIn = &sPo_PlcPostnerIn ; // DRVTSKCTRL_TO_PLC: positioner input from plc


            /* ================== Control Loop ================== */
            if(sLocParams.sTo.ubSpaceSpeedCntrLp == DRVTSKCTRL_TO_AUTO)
            {
                sSS_CntrLoopIn.psRef = &sEfsSSInIdrAndPLoop.sDemand ;
                sSS_CntrLoopIn.psMotCtrlStatus = &sEfsSSInIdrAndPLoop.sFlags ;
                sSS_CntrLoopIn.psCurrentRefs = &sMh_MotorDataUsrInIRef ;
            }
            else
            {
                sSS_CntrLoopIn.psRef = &sSS_PlcPidRef ; // DRVTSKCTRL_TO_PLC: control loop input from plc
                sSS_CntrLoopIn.psMotCtrlStatus = &sSS_PlcMotCtrlStatus ;
                sSS_CntrLoopIn.psCurrentRefs = &sMh_MotorDataPlcInIRef ;
            }


            /* ================= Motor Handler ================== */
            if(sLocParams.sTo.ubMotorHandler == DRVTSKCTRL_TO_AUTO)
            {   // DRVTSKCTRL_TO_AUTO: motor handler input from pid_out
                sMh_MotorDataIn.puwElecAngle = &psFbEncoder->uwElecAngle ;
                sMh_MotorDataIn.pswElecSpeed = &psFbEncoder->swElecSpeed ;
                sMh_MotorDataIn.pslFilteredSpeed = &psFbEncoder->sEncData.slSpeed ;
                sDflx_In.pswActualSpd = &psFbEncoder->swElecSpeed ;

                if (sEm_EncMngrOut.flags.b.bRequireIRefHook)
                {
                    sEm_EncMngrIn.sIRefIn = &sSS_CntrLoopOut.sIRef ;
                    sMh_MotorDataIn.psIRef2Use = &sEm_EncMngrOut.sIRefOut ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sMotCtrl_Out.sPowerStageCtrl ;
                    sMh_MotorDataIn.psPStageCtrl = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                }
                else
                {
                    sEm_EncMngrIn.sIRefIn = &sEm_EncMngrOut.sIRefOut ;
                    sMh_MotorDataIn.psIRef2Use = &sSS_CntrLoopOut.sIRef ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                    sMh_MotorDataIn.psPStageCtrl = &sMotCtrl_Out.sPowerStageCtrl ;
                }
            }
            else
            {   // DRVTSKCTRL_TO_PLC: motor handler input from plc
                sMh_MotorDataIn.puwElecAngle = &uwMh_MotoDataPlcInElecAngle ;
                sMh_MotorDataIn.pswElecSpeed = &swMh_MotoDataPlcInElecSpeed ;
                sMh_MotorDataIn.pslFilteredSpeed = &slMh_MotoDataPlcInFilteredSpeed ;
                sDflx_In.pswActualSpd = &swMh_MotoDataPlcInElecSpeed ;

                if (sEm_EncMngrOut.flags.b.bRequireIRefHook)
                {
                    sEm_EncMngrIn.sIRefIn = &sMh_MotorDataPlcInIRef ;
                    sMh_MotorDataIn.psIRef2Use = &sEm_EncMngrOut.sIRefOut ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sMh_MotorDataPlcPStageCtrl ;
                    sMh_MotorDataIn.psPStageCtrl = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                }
                else
                {
                    sEm_EncMngrIn.sIRefIn = &sEm_EncMngrOut.sIRefOut ;
                    sMh_MotorDataIn.psIRef2Use = &sMh_MotorDataPlcInIRef ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                    sMh_MotorDataIn.psPStageCtrl = &sMh_MotorDataPlcPStageCtrl ;
                }
            }


            /* ========== PI-16bit Handler (PLC only) =========== */
            if(sLocParams.sTo.ubModPiDcBusCntrlLp32bit == DRVTSKCTRL_TO_PLC)
            {   // consento di usare il PI 16bit con il plc (ZF)
                // DRVTSKCTRL_TO_PLC: PI input from plc
                sDcBusCntrl_In.pswRef          = &sDcBusCntrl_Plc.swDcBusRef ;
                sDcBusCntrl_In.pswFbk          = &sDcBusCntrl_Plc.swDcBusFbk  ;
                sDcBusCntrl_In.pswVoltage      = &sDcBusCntrl_Plc.swVoltage ;
                sDcBusCntrl_In.pubVoltageValid = &sDcBusCntrl_Plc.ubVoltageValid  ;
            }


            /* ================ BackEMF ========================= */
            sBe_EmfEncIn.psRef = sSS_CntrLoopIn.psRef;

            // ========= Add and delete limit to the list =======
            (*sMh_MotorHandlers.pfDeleteLimitFromList)(MH_LIMITLIST_ID, &sMh_MotorDataParam.sIdLimit);  /* delete Torque ID current limits */
            (*sMh_MotorHandlers.pfDeleteLimitFromList)(MH_LIMITLIST_IQ, &sMh_MotorDataParam.sIqLimit);  /* delete Torque IQ current limits */
            (*sMh_MotorHandlers.pfDeleteLimitFromList)(MH_LIMITLIST_IQ, &sSS_CntrLoopWks.sUsrOutVal32); /* delete space speed current limits (avoid to install it two times) */
            (*sMh_MotorHandlers.pfAdd2LimitList)(MH_LIMITLIST_IQ, &sSS_CntrLoopWks.sUsrOutVal32);       /* install space speed current limits */
            break;


        case DRVTSKCTRL_OP_IRTQ:
            // ###########################################################
            // #### Internal Generated References Torque #################
            // ###########################################################
            assert(ubMainOperationMask & MO_MOTORCTRL);

            /* =============== Motion Controller ================ */
            if(sLocParams.sTo.ubMotionController == DRVTSKCTRL_TO_AUTO)
                sMotCtrl_In.psMotCtrl_In = &sMotCtrl_UsrControl ; // DRVTSKCTRL_TO_AUTO: firmware control word
            else
                sMotCtrl_In.psMotCtrl_In = &sMotCtrl_PlcControl ; // DRVTSKCTRL_TO_PLC: plc control word

            /* =================== Positioner =================== */
            if(sLocParams.sTo.ubPositioner == DRVTSKCTRL_TO_AUTO)
                psPo_PostnerIn = &sPo_UsrPostnerIn ; // DRVTSKCTRL_TO_AUTO: positioner input from fieldbus/user
            else
                psPo_PostnerIn = &sPo_PlcPostnerIn ; // DRVTSKCTRL_TO_PLC: positioner input from plc

            /* ================== Control Loop ================== */
            if(sLocParams.sTo.ubSpaceSpeedCntrLp == DRVTSKCTRL_TO_AUTO)
            {
                sSS_CntrLoopIn.psRef = &sPo_PostnerOut.sDemand ; // DRVTSKCTRL_TO_AUTO: control loop input from positioner_out
                sSS_CntrLoopIn.psMotCtrlStatus = &sMotCtrl_OStatus ;
                sSS_CntrLoopIn.psCurrentRefs = &sMh_MotorDataUsrInIRef ;
            }
            else
            {
                sSS_CntrLoopIn.psRef = &sSS_PlcPidRef ; // DRVTSKCTRL_TO_PLC: control loop input from plc
                sSS_CntrLoopIn.psMotCtrlStatus = &sSS_PlcMotCtrlStatus ;
                sSS_CntrLoopIn.psCurrentRefs = &sMh_MotorDataPlcInIRef ;
            }

            /* ================= Motor Handler ================== */
            if(sLocParams.sTo.ubMotorHandler == DRVTSKCTRL_TO_AUTO)
            {   // DRVTSKCTRL_TO_AUTO: motor handler input from fieldbus/user
                sMh_MotorDataIn.puwElecAngle = &psFbEncoder->uwElecAngle ;
                sMh_MotorDataIn.pswElecSpeed = &psFbEncoder->swElecSpeed ;
                sMh_MotorDataIn.pslFilteredSpeed = &psFbEncoder->sEncData.slSpeed ;
                sDflx_In.pswActualSpd = &psFbEncoder->swElecSpeed ;

                if (sEm_EncMngrOut.flags.b.bRequireIRefHook)
                {
                    sEm_EncMngrIn.sIRefIn = &sMotCtrl_Out.sIRefs ;
                    sMh_MotorDataIn.psIRef2Use = &sEm_EncMngrOut.sIRefOut;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sMotCtrl_Out.sPowerStageCtrl ;
                    sMh_MotorDataIn.psPStageCtrl = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                }
                else
                {
                    sEm_EncMngrIn.sIRefIn = &sEm_EncMngrOut.sIRefOut ;
                    sMh_MotorDataIn.psIRef2Use = &sMotCtrl_Out.sIRefs ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                    sMh_MotorDataIn.psPStageCtrl = &sMotCtrl_Out.sPowerStageCtrl ;
                }
            }
            else
            {   // DRVTSKCTRL_TO_PLC: motor handler input from plc
                sMh_MotorDataIn.puwElecAngle = &uwMh_MotoDataPlcInElecAngle ;
                sMh_MotorDataIn.pswElecSpeed = &swMh_MotoDataPlcInElecSpeed ;
                sMh_MotorDataIn.pslFilteredSpeed = &slMh_MotoDataPlcInFilteredSpeed ;
                sDflx_In.pswActualSpd = &swMh_MotoDataPlcInElecSpeed ;

                if (sEm_EncMngrOut.flags.b.bRequireIRefHook)
                {
                    sEm_EncMngrIn.sIRefIn = &sMh_MotorDataPlcInIRef ;
                    sMh_MotorDataIn.psIRef2Use = &sEm_EncMngrOut.sIRefOut ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sMh_MotorDataPlcPStageCtrl ;
                    sMh_MotorDataIn.psPStageCtrl = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                }
                else
                {
                    sEm_EncMngrIn.sIRefIn = &sEm_EncMngrOut.sIRefOut ;
                    sMh_MotorDataIn.psIRef2Use = &sMh_MotorDataPlcInIRef ;
                    sEm_EncMngrIn.sPowerStageCtrlIn = &sEm_EncMngrOut.sPowerStageCtrlOut ;
                    sMh_MotorDataIn.psPStageCtrl = &sMh_MotorDataPlcPStageCtrl ;
                }
            }


            /* ================ PI16bit Handler ================= */
            if(sLocParams.sTo.ubModPiDcBusCntrlLp32bit == DRVTSKCTRL_TO_PLC)
            {   // consento di usare il PI 16bit con il plc (ZF)
                // DRVTSKCTRL_TO_PLC: PI input from plc
                sDcBusCntrl_In.pswRef          = &sDcBusCntrl_Plc.swDcBusRef ;
                sDcBusCntrl_In.pswFbk          = &sDcBusCntrl_Plc.swDcBusFbk  ;
                sDcBusCntrl_In.pswVoltage      = &sDcBusCntrl_Plc.swVoltage ;
                sDcBusCntrl_In.pubVoltageValid = &sDcBusCntrl_Plc.ubVoltageValid  ;
            }

            /* ================ BackEMF ========================= */
            sBe_EmfEncIn.psRef = sSS_CntrLoopIn.psRef;

            // ========= Add and delete limit to the list =======
            (*sMh_MotorHandlers.pfDeleteLimitFromList)(MH_LIMITLIST_IQ, &sSS_CntrLoopWks.sUsrOutVal32); /* delete space speed current limits (do not install) */
            (*sMh_MotorHandlers.pfDeleteLimitFromList)(MH_LIMITLIST_ID, &sMh_MotorDataParam.sIdLimit);  /* delete Torque ID current limits (avoid to install it two times) */
            (*sMh_MotorHandlers.pfDeleteLimitFromList)(MH_LIMITLIST_IQ, &sMh_MotorDataParam.sIqLimit);  /* delete Torque IQ current limits (avoid to install it two times) */
            (*sMh_MotorHandlers.pfAdd2LimitList)(MH_LIMITLIST_ID, &sMh_MotorDataParam.sIdLimit);        /* install Torque ID current limits */
            (*sMh_MotorHandlers.pfAdd2LimitList)(MH_LIMITLIST_IQ, &sMh_MotorDataParam.sIqLimit);        /* install Torque IQ current limits */
            break;

        case DRVTSKCTRL_OP_PSU:
            assert(ubMainOperationMask & MO_PSU);

            break;
        default:
            assert(0);
    }

    return ubPreviousCfg;
}

//***************************************************************************
// slow task

static BOOL parameterscheck(void)
{
    BOOL bValid;

        // check main operation mode
    switch(sDrvTskCtrlParams.sNew.ubMainOperation)
    {
        case DRVTSKCTRL_MO_MOTORCONTROL:
        case DRVTSKCTRL_MO_PSU:
            bValid=TRUE;
            break;

        default:
            bValid=FALSE;
            break;
    }

        // signal or reset error
    if(bValid)
        ParChk_ResetValueError(PARCC_DRVTSKCTRL_MAINOPERATION);
    else
        ParChk_SignalValueError(PARCC_DRVTSKCTRL_MAINOPERATION);
    
    return bValid;
}
