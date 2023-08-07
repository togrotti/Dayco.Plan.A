/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CanOpenCOECommon.c                                         */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : CanOpen/COE Common                                         */
/*                                                                          */
/****************************************************************************/

#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\ParamStorageManagement.h"
#include "system\SysLogManagement.h"
#include "system\SysLogData.h"
#include "CanOpenDs301.h"
#include "CanOpenComDB.h"
#include "system\SysAppIdentification.h"
#include "CanOpenCOECommon.h"
#if CFG_CANDRV_DS301
#include "CanOpenPdoSyncMgr.h"
#endif
#include <string.h>

#ifdef _APP_XC
#include "plc\Plc.h"
#include "common\SFlashStorage.h"
#ifdef _INFINEON_
#include "system\SysAppSFlashPartXC.h"
#else
#include "system\SysAppSFlashPartZYNQ.h"
#endif // _infineon_
#endif

#ifdef _INFINEON_
//***************************************************************************
// Avoids warning C37: nonstandard extension - unnamed parameter

 #pragma warning disable = 37
#else
/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)
#endif

//***************************************************************************
// Data structures

typedef struct
{
    ULONG   ulAlarm;
    ULONG   ulAlarmSubCode;
    UWORD   uwErrCode;
} CANOPENCOE_ERRCODETRANSLATION;

//***************************************************************************
// Globals

    // Identification
const CHARS sCanOpenCOEManufacturerName[]=   IDENT_FACTORY_NAME;
const CHARS sCanOpenCOEDeviceNameBase[]=     IDENT_FAMILY_NAME;
const CHARS sCanOpenCOESWVersion[]=          IDENT_APP_DESCRIPTION_LONG;

DS301_IDENTITY  tDs301Identity=
{
    0x00020192,                 // Servo drive / DS402
	0x00504D43,                 // CiA assigned Vendor ID
								// 0x00504D43 = Ningbo Physis Technology Co.,Ltd.
								// 0x000000D9 = Phase Motion Control SpA
    0,                          // productcode (assigned at startup)
    0,                          // revision (assigned at startup)
    0,                          // serial number (assigned at startup)
    (PVISIBLE_STRING)sCanOpenCOEManufacturerName,
    (PVISIBLE_STRING)sCanOpenCOEDeviceNameBase,
    (PVISIBLE_STRING)sCanOpenCOESWVersion,
};

#ifdef _APP_XC
BOOL sCanOpenCOEReqReflash=FALSE;
#endif

//****************************************************************************
// Alarm translation table

static const CANOPENCOE_ERRCODETRANSLATION  tCanOpenCOEErrCodeTranslation[]=
{
    {SYSTEMALARMS_BIT_SOFTWARE_FAIL,            SYSTEMALARMS_SUBCODE_SF_TM_SYNCPOINT_TIMEOUT,                             0x6011},
    {SYSTEMALARMS_BIT_SOFTWARE_FAIL,            SYSTEMALARMS_SUBCODE_SF_TM_SLOWTASK_RUNNING,                              0x6012},
    {SYSTEMALARMS_BIT_SOFTWARE_FAIL,            SYSTEMALARMS_SUBCODE_SF_TM_SLOWTASK_OVERTIME,                             0x6013},
    {SYSTEMALARMS_BIT_SOFTWARE_FAIL,            SYSTEMALARMS_SUBCODE_SF_TM_FPGA_WDT_FAIL,                                 0x6014},
    {SYSTEMALARMS_BIT_SOFTWARE_FAIL,            SYSTEMALARMS_SUBCODE_SF_TM_FPGA_CRC_FAIL,                                 0x6015},
    {SYSTEMALARMS_BIT_SOFTWARE_FAIL,            SYSTEMALARMS_SUBCODE_SF_TM_FPGA_LOCKOUT,                                  0x6016},
    {SYSTEMALARMS_BIT_SOFTWARE_FAIL,            SYSTEMALARMS_SUBCODE_SF_TM_FPGA_PLLLOSSOFLOCK,                            0x6017},
    {SYSTEMALARMS_BIT_SOFTWARE_FAIL,            SYSTEMALARMS_SUBCODE_SF_TM_IOEXP_WDT,                                     0x6018},
    {SYSTEMALARMS_BIT_SOFTWARE_FAIL,            SYSTEMALARMS_SUBCODE_SF_TM_PSU_WDT,                                       0x6019},
    {SYSTEMALARMS_BIT_SOFTWARE_FAIL,            0l,                                                                       0x6010},
    {SYSTEMALARMS_BIT_PLC_OVERTIME,             SYSTEMALARMS_SUBCODE_PLC_SLOWOVERTIME,                                    0x6201},
    {SYSTEMALARMS_BIT_PLC_OVERTIME,             SYSTEMALARMS_SUBCODE_PLC_FASTOVERTIME,                                    0x6202},
    {SYSTEMALARMS_BIT_BOOT_FAIL,                SYSTEMBOOTERR_STARTWITHPOWERFAIL,                                         0x6101},
    {SYSTEMALARMS_BIT_BOOT_FAIL,                SYSTEMBOOTERR_INVALIDFLASHPARAMETERS,                                     0x5532},
    {SYSTEMALARMS_BIT_BOOT_FAIL,                0l,                                                                       0x6100},
    {SYSTEMALARMS_BIT_BOOT_FAIL_FOR_INVALID_PAR,0l,                                                                       0x6320},
    {SYSTEMALARMS_BIT_HW_POWERSECTION_FAIL,     0l,                                                                       0x2110},
    {SYSTEMALARMS_BIT_HW_POWERSECTION_FAIL,     SYSTEMALARMS_SUBCODE_PS_BRIDGE1,                                          0x2120},
    {SYSTEMALARMS_BIT_HW_FLASH_FAIL,            SYSTEMALARMS_SUBCODE_HF_FLASH_SYSLOG,                                     0x5531},
    {SYSTEMALARMS_BIT_HW_FLASH_FAIL,            SYSTEMALARMS_SUBCODE_HF_FLASH_PARAMS,                                     0x5532},
    {SYSTEMALARMS_BIT_HW_FLASH_FAIL,            SYSTEMALARMS_SUBCODE_HF_FLASH_HWCONFIG,                                   0x5533},
    {SYSTEMALARMS_BIT_HW_SAFETORQUEOFF,         0l,                                                                       0x9002},
    {SYSTEMALARMS_BIT_CURRENT_FAIL,             0l,                                                                       0x2310},

    {SYSTEMALARMS_BIT_MOTOR_FAIL,               SYSTEMALARMS_SUBCODE_MT_MISSINGMOTORPHASE,                                0x3331},
    {SYSTEMALARMS_BIT_MOTOR_FAIL,               SYSTEMALARMS_SUBCODE_MT_MOTORBLOCKED,                                     0x7121},
    {SYSTEMALARMS_BIT_MOTOR_FAIL,               SYSTEMALARMS_SUBCODE_MT_OVERSPEED,                                        0x8401},
    {SYSTEMALARMS_BIT_MOTOR_FAIL,               0l,                                                                       0x7120},

    {SYSTEMALARMS_BIT_CURRENT_FAIL,             SYSTEMALARMS_SUBCODE_CR_BRIDGE1,                                          0x2320},
    {SYSTEMALARMS_BIT_VL_OVERVOLTAGE_FAIL,      0l,                                                                       0x3210},

    {SYSTEMALARMS_BIT_VL_UNDERVOLTAGE_FAIL,     SYSTEMALARMS_SUBCODE_VLU_ACLINEFAILURE,                                   0x3120},
    {SYSTEMALARMS_BIT_VL_UNDERVOLTAGE_FAIL,     0l,                                                                       0x3220},

    {SYSTEMALARMS_BIT_ENCODERMANAGER_FAIL,      SYSTEMALARMS_SUBCODE_ENCMGR_ABSRELDIFFFAIL,                               0x7341},
    {SYSTEMALARMS_BIT_ENCODERMANAGER_FAIL,      SYSTEMALARMS_SUBCODE_ENCMGR_VENCFAIL,                                     0x7342},
    {SYSTEMALARMS_BIT_ENCODERMANAGER_FAIL,      SYSTEMALARMS_SUBCODE_ENCMGR_EFSFAIL,                                      0x7343},
    {SYSTEMALARMS_BIT_ENCODERMANAGER_FAIL,      SYSTEMALARMS_SUBCODE_ENCMGR_AUXENDATNOTPRESENT,                           0x7344},
    {SYSTEMALARMS_BIT_ENCODERMANAGER_FAIL,      0l,                                                                       0x7340},

    {SYSTEMALARMS_BIT_EN_ENDAT_FAIL,            SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_ENDAT_ALARM,           0x7381},
    {SYSTEMALARMS_BIT_EN_ENDAT_FAIL,            SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_ENDAT_CRC,             0x7382},
    {SYSTEMALARMS_BIT_EN_ENDAT_FAIL,            SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_ENDAT_OVERTIME,        0x7383},
    {SYSTEMALARMS_BIT_EN_SINCOS_FAIL,           SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_SC_INV_LINE_COUNT,     0x7390},
    {SYSTEMALARMS_BIT_EN_SINCOS_FAIL,           SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_SC_ENCODER_LEVEL,      0x7391},
    {SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL,      SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_IN_INV_LINE_COUNT,     0x73A0},
    {SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL,      SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_IN_ENCODER_LEVEL,      0x73A1},
    {SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL,      SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_IN_FPGA_FAULT,         0x73A2},
    {SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL,      SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_IN_ANDIG_FAULT,        0x73A3},
    {SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL,      SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_IN_FILTER_WDT,         0x73A4},
    {SYSTEMALARMS_BIT_EN_HALLSENSORS_FAIL,      SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_HL_INV_LINE_COUNT,     0x73B0},
    {SYSTEMALARMS_BIT_EN_HALLSENSORS_FAIL,      SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_HL_INVALID_STATE,      0x73B1},
    {SYSTEMALARMS_BIT_EN_HALLSENSORS_FAIL,      SYSTEMALARMS_SUBCODE_ENC_MAIN|SYSTEMALARMS_SUBCODE_HL_INVALID_DELTA,      0x73B3},

    {SYSTEMALARMS_BIT_EN_ENDAT_FAIL,            SYSTEMALARMS_SUBCODE_ENC_AUX |SYSTEMALARMS_SUBCODE_ENDAT_ALARM,           0x7389},
    {SYSTEMALARMS_BIT_EN_ENDAT_FAIL,            SYSTEMALARMS_SUBCODE_ENC_AUX |SYSTEMALARMS_SUBCODE_ENDAT_CRC,             0x738A},
    {SYSTEMALARMS_BIT_EN_ENDAT_FAIL,            SYSTEMALARMS_SUBCODE_ENC_AUX |SYSTEMALARMS_SUBCODE_ENDAT_OVERTIME,        0x738B},
    {SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL,      SYSTEMALARMS_SUBCODE_ENC_AUX |SYSTEMALARMS_SUBCODE_IN_INV_LINE_COUNT,     0x73A8},
    {SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL,      SYSTEMALARMS_SUBCODE_ENC_AUX |SYSTEMALARMS_SUBCODE_IN_ENCODER_LEVEL,      0x73A9},
    {SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL,      SYSTEMALARMS_SUBCODE_ENC_AUX |SYSTEMALARMS_SUBCODE_IN_FPGA_FAULT,         0x73AA},
    {SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL,      SYSTEMALARMS_SUBCODE_ENC_AUX |SYSTEMALARMS_SUBCODE_IN_SIM_MAXDIFFEXCEED,  0x73AB},
    {SYSTEMALARMS_BIT_EN_INCREMENTAL_FAIL,      SYSTEMALARMS_SUBCODE_ENC_AUX |SYSTEMALARMS_SUBCODE_IN_FILTER_WDT,         0x73AC},

    {SYSTEMALARMS_BIT_EN_ENDAT_FAIL,            0l,                                                                       0x7380},

    {SYSTEMALARMS_BIT_TM_THERMALMODEL_FAIL,     SYSTEMALARMS_SUBCODE_NTC_IGBT_FAIL,                                       0x4001},
    {SYSTEMALARMS_BIT_TM_THERMALMODEL_FAIL,     SYSTEMALARMS_SUBCODE_JUNCTION_OVERTEMP,                                   0x4310},
    {SYSTEMALARMS_BIT_TM_THERMALMODEL_FAIL,     SYSTEMALARMS_SUBCODE_FAN_LOCKED,                                          0x4002},
    {SYSTEMALARMS_BIT_TM_THERMALMODEL_FAIL,     SYSTEMALARMS_SUBCODE_HEATSINK_OVERTEMP,                                   0x4311},

    {SYSTEMALARMS_BIT_EN_ACTIVEFRONTEND_FAIL,   SYSTEMALARMS_SUBCODE_ACTFRNTEND_AGLITCH,                                  0x31F0},
    {SYSTEMALARMS_BIT_EN_ACTIVEFRONTEND_FAIL,   SYSTEMALARMS_SUBCODE_ACTFRNTEND_SYNC_LOST,                                0x31F1},

    {SYSTEMALARMS_BIT_BE_BACKEMF_FAIL,          SYSTEMALARMS_SUBCODE_BACKEMF_KT_OUTOFRANGE,                               0x73C0},
    {SYSTEMALARMS_BIT_BE_BACKEMF_FAIL,          SYSTEMALARMS_SUBCODE_BACKEMF_INV_MAGICNUMBER,                             0x73C1},
    {SYSTEMALARMS_BIT_BE_BACKEMF_FAIL,          SYSTEMALARMS_SUBCODE_BACKEMF_SYNCLOST,                                    0x73C3},
    {SYSTEMALARMS_BIT_BE_BACKEMF_FAIL,          SYSTEMALARMS_SUBCODE_BACKEMF_AGLITCH,                                     0x73C4},

    {SYSTEMALARMS_BIT_HW_BRAKE_FAIL,            SYSTEMALARMS_SUBCODE_BF_ALWAYS_ON,                                        0x7111},
    {SYSTEMALARMS_BIT_HW_BRAKE_FAIL,            SYSTEMALARMS_SUBCODE_BF_OVERPOWER,                                        0x7112},
    {SYSTEMALARMS_BIT_HW_BRAKE_FAIL,            SYSTEMALARMS_SUBCODE_BF_DESATURATION,                                     0x7113},

    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              DS301_COMMERR_CAN_HWOVERRUN,                                              DS301_EMCYCODE_MON_CANHWOVERRUN},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              DS301_COMMERR_CAN_PASSIVE,                                                DS301_EMCYCODE_MON_CANERRPASV},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              DS301_COMMERR_GUARDHEARTBEAT_WRONGPARAMS,                                 DS301_EMCYCODE_GUARDHEARTBEAT_WRONGPARAMS},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              DS301_COMMERR_GUARDHEARTBEAT,                                             DS301_EMCYCODE_MON_LIFEHEARTBEAT},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              DS301_COMMERR_PDOLENGHTERROR,                                             DS301_EMCYCODE_PROT_PDOLENGTH},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              DS301_COMMERR_CANOVERRUN,                                                 DS301_EMCYCODE_MON_CANSWOVERRUN},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              DS301_COMMERR_EMCY_WRONGCOBID,                                            DS301_EMCYCODE_EMCY_WRONGCOBID},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              DS301_COMMERR_CAN_EXITFROMBUSOFF,                                         DS301_EMCYCODE_MON_CANBUSOFFRECOVER},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              DS301_COMMERR_SYNC_WRONGCOBID,                                            DS301_EMCYCODE_SYNC_WRONGCOBID},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              DS301_COMMERR_CAN_TXOVERRUN,                                              DS301_EMCYCODE_MON_CANTXOVERRUN},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              DS301_COMMERR_SYNCERROR,                                                  DS301_EMCYCODE_COMMUNICATIONERROR},
#if CFG_CANDRV_DS301
	{SYSTEMALARMS_BIT_MN_CAN_FAIL,              CANOPENPSM_PE_MASK(CANOPENPSM_PDOERR_INVALIDCOBID),                       (DS301_EMCYCODE_DEVICESPECIFIC-(CANOPENPSM_PDOERR_INVALIDCOBID))},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              CANOPENPSM_PE_MASK(CANOPENPSM_PDOERR_RTRNOTVALID),                        (DS301_EMCYCODE_DEVICESPECIFIC-(CANOPENPSM_PDOERR_RTRNOTVALID))},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              CANOPENPSM_PE_MASK(CANOPENPSM_PDOERR_INVALIDTXTYPE),                      (DS301_EMCYCODE_DEVICESPECIFIC-(CANOPENPSM_PDOERR_INVALIDTXTYPE))},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              CANOPENPSM_PE_MASK(CANOPENPSM_PDOERR_INVALIDMAPCOUNT),                    (DS301_EMCYCODE_DEVICESPECIFIC-(CANOPENPSM_PDOERR_INVALIDMAPCOUNT))},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              CANOPENPSM_PE_MASK(CANOPENPSM_PDOERR_OUTOFMEMORY),                        (DS301_EMCYCODE_DEVICESPECIFIC-(CANOPENPSM_PDOERR_OUTOFMEMORY))},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              CANOPENPSM_PE_MASK(CANOPENPSM_PDOERR_INVALIDSYNCSTARTVALUE),              (DS301_EMCYCODE_DEVICESPECIFIC-(CANOPENPSM_PDOERR_INVALIDSYNCSTARTVALUE))},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              CANOPENPSM_PE_MASK(CANOPENPSM_PDOERR_INTERNALERROR),                      (DS301_EMCYCODE_DEVICESPECIFIC-(CANOPENPSM_PDOERR_INTERNALERROR))},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              CANOPENPSM_PE_MASK(CANOPENPSM_PDOERR_PDOLENGTHEXCEED),                    (DS301_EMCYCODE_DEVICESPECIFIC-(CANOPENPSM_PDOERR_PDOLENGTHEXCEED))},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              CANOPENPSM_PE_MASK(CANOPENPSM_PDOERR_HOOKPROCESSINGFAIL),                 (DS301_EMCYCODE_DEVICESPECIFIC-(CANOPENPSM_PDOERR_HOOKPROCESSINGFAIL))},
    {SYSTEMALARMS_BIT_MN_CAN_FAIL,              CANOPENPSM_PE_MASK(CANOPENPSM_PDOERR_DUPLICATEDCOBID),                    (DS301_EMCYCODE_DEVICESPECIFIC-(CANOPENPSM_PDOERR_DUPLICATEDCOBID))},
#endif
    {SYSTEMALARMS_BIT_TM_MOTOR_OVERTEMPERATURE, 0l,                                                                       0x4211},

    {SYSTEMALARMS_BIT_USERDEFINED,              0l,                                                                       0xFFF0},

    {0l,0l,0}       // terminator
};

//****************************************************************************
// Init handler

BOOL CanOpenCOE_Init(UWORD t)
{
        // initialize identity
    tDs301Identity.ulProductCode=(ULONG)IDENT_APPLICATION_TYPE;
    tDs301Identity.ulRevision=(((ULONG)(IDENT_VERSION_MAJOR)&0xff)<<24)|(((ULONG)(IDENT_VERSION_MINOR)&0xff)<<16)|((ULONG)(IDENT_VERSION_BUILD_NUMBER)&0xffff);
    tDs301Identity.ulSerialNumber=sGlbAssemblyInfo.ulSerialNumber;

    return TRUE;
}

//***************************************************************************
// Table checking, just for debug purpose

#ifdef _APP_DEBUG
BOOL CanOpenCOE_CheckDBTable(void)
{
    UWORD uwIndxCount;
    void * pvGeneric;
    UBYTE ubType;

        // check data pointer for PDO mappable object, that must be near-castable
        // for time-optimized PDO management
    for(uwIndxCount = 0; uwIndxCount < uwCanOpenParamCount; uwIndxCount++)
        if(hpsCanOpenParamTable[uwIndxCount].uwFlags & CANOPENCOMDB_F_PDOMAPPABLE)
        {
                // cast to near
            pvGeneric=(void *)hpsCanOpenParamTable[uwIndxCount].hpsComDBEntry->hpvData;

                // then back to , must remain same
            assert( hpsCanOpenParamTable[uwIndxCount].hpsComDBEntry->hpvData == (HPVOID)pvGeneric );

                // check for supported data types
            ubType=hpsCanOpenParamTable[uwIndxCount].hpsComDBEntry->ubType;
            assert( ubType==COMMONPARAMDB_TYPE_UBYTE || ubType==COMMONPARAMDB_TYPE_SBYTE || \
                    ubType==COMMONPARAMDB_TYPE_UWORD || ubType==COMMONPARAMDB_TYPE_SWORD || \
                    ubType==COMMONPARAMDB_TYPE_ULONG || ubType==COMMONPARAMDB_TYPE_SLONG || \
                    ubType==COMMONPARAMDB_TYPE_UQWRD || ubType==COMMONPARAMDB_TYPE_SQWRD || \
                    ubType==COMMONPARAMDB_TYPE_FLOAT || ubType==COMMONPARAMDB_TYPE_DOUBL );
        }

    return TRUE;
}
#endif

//***************************************************************************
// Return next alarm entry if any

BOOL CanOpenCOE_GetAlarmEntry(ULONG * pulRefTime, ULONG * pulAlarm, UWORD * puwAlarmSubCode, UBYTE * pubAlarmClass, ULONG * pulAlarmStat)
{
    SYSLOGMGM_ALARMLOG tLog;
    const CANOPENCOE_ERRCODETRANSLATION  * pErrCodes;
    ULONG ulLocAlarms,ulTime;

        // atomically get both actual alarm mask and last alarm time
//    DISABLE_IRQ();
    ulLocAlarms=ulSystemAlarms;
    ulTime=ulSysLogDataLastAlarmTime;
//    RESTORE_IRQ();

        // if previous are alarms and actual zero then alarm
        // reset has occurred
    if(*pulAlarmStat!=0l && ulLocAlarms==0l)
    {
            // then set time at last alarm
        *pulRefTime=ulTime+1;

            // and return all zero signaling alarm reset
        *pulAlarm=0l;
        *puwAlarmSubCode=0;
        *pubAlarmClass=0;
        *pulAlarmStat=0;

        return TRUE;
    }

        // update local status
    *pulAlarmStat=ulLocAlarms;

        // if found
    if(SysLogData_GetFromAlarmHistory(*pulRefTime, &tLog))
    {
        *pulRefTime=tLog.ulAbsoluteTime+1;

            // translate and return code
        pErrCodes=tCanOpenCOEErrCodeTranslation;
        while(pErrCodes->ulAlarm)
        {
            if(pErrCodes->ulAlarm==tLog.ulAlarm && (pErrCodes->ulAlarmSubCode==0l || pErrCodes->ulAlarmSubCode==tLog.ulAlarmSubCode))
                break;
            pErrCodes++;
        }
        
            // if found then is also managed for DS301
        if(pErrCodes->ulAlarm)
        {
            *pulAlarm=tLog.ulAlarm;
            *puwAlarmSubCode=pErrCodes->uwErrCode;

                // select alarm code class
            switch((pErrCodes->uwErrCode>>12)&0xf)
            {
                case DS301_EMCYCODE_CURRENT>>12:
                    *pubAlarmClass=DS301_REGERR_CURRENT;
                    break;
                    
                case DS301_EMCYCODE_VOLTAGE>>12:
                    *pubAlarmClass=DS301_REGERR_VOLTAGE;
                    break;
                    
                case DS301_EMCYCODE_TEMPERATURE>>12:
                    *pubAlarmClass=DS301_REGERR_TEMPERATURE;
                    break;
                    
                case DS301_EMCYCODE_ADDITIONALMODULES>>12:
                    *pubAlarmClass=DS301_REGERR_MANUFACTURERSPECIFIC;
                    break;
                    
                case DS301_EMCYCODE_MONITORING>>12:
                    *pubAlarmClass=DS301_REGERR_COMMUNICATION;
                    break;
                    
                case DS301_EMCYCODE_EXTERNAL>>12:
                    *pubAlarmClass=DS301_REGERR_MANUFACTURERSPECIFIC;
                    break;
                    
                case DS301_EMCYCODE_ADDITIONALFUNCTION>>12:
                    *pubAlarmClass=DS301_REGERR_DEVICESPECIFIC;
                    break;
                    
                default:
                    *pubAlarmClass=0;
            }
    
            return TRUE;
        }
    }

    return FALSE;
}

//***************************************************************************
// hook for parameter saving

UWORD CanOpenCOE_ParamSave(COMMONPARAMDB_ENTRY  * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext)
{
    static const CHARS sig[]="save";
    UWORD ct;

        // dummy to avoid compiler warning
    ct=hpsEntry->uwIndex;

    if(uwFlags&COMMONPARAMDB_CBFLAG_ABORT)
        return COMMONPARAMDB_CH_OK;

    if(uwElement>1)
        return COMMONPARAMDB_CH_INVALID_ELEMENT;

	if(uwFlags&COMMONPARAMDB_CBFLAG_INFOREQ)
	{
		COMMONPARAMDB_ENTRY  * hpsInfo=hpvBuffer;
		
        *hpsInfo=*hpsEntry;

		if(uwElement)
		{
            hpsInfo->ubFlags=COMMONPARAMDB_FLAG_RW;
            hpsInfo->ubType=COMMONPARAMDB_TYPE_ULONG;
            hpsInfo->uwNElements=0;
        }
        else
		{
            hpsInfo->ubFlags=COMMONPARAMDB_FLAG_RD;
            hpsInfo->ubType=COMMONPARAMDB_TYPE_UBYTE;
            hpsInfo->swMoreType=1;
        }
	
		return COMMONPARAMDB_CH_OK;
	}

    if(uwFlags&COMMONPARAMDB_CBFLAG_RD)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
            if(uwFlags&COMMONPARAMDB_CBFLAG_SIZEINQUIRY)
            {
                *((HPULONG)hpvBuffer)=uwElement?sizeof(ULONG):sizeof(UBYTE);
                *puwBufSize=sizeof(ULONG);
            }
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            if(uwElement)
            {
                    // store capability
                ULONG ulCap=1l;
                memcpy(hpvBuffer, &ulCap, sizeof(ULONG));
                *puwBufSize=sizeof(ULONG);
            }
            else
            {
                *((HPUBYTE)hpvBuffer)=1;
                *puwBufSize=sizeof(UBYTE);
            }
        }
    }

    if(uwFlags&COMMONPARAMDB_CBFLAG_WR)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
            if(uwElement==0)
                return COMMONPARAMDB_CH_NO_WRITE_ACCESS;
    
                // also accept zero length for clients that
                // do not send data length
            if(*((HPULONG)hpvBuffer)!=0l && *((HPULONG)hpvBuffer)!=4l)
                return COMMONPARAMDB_CH_WRONGLENGTH;
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            for(ct=0;ct<4;ct++)
                if(((HPUBYTE)hpvBuffer)[ct]!=sig[ct])
                    return COMMONPARAMDB_CH_INVALID_DATA;

            *hpulContext=TRUE;
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_END)
            if(*hpulContext)
                if(parmgm_par_save()!=PARMGM_B_OK)
                {
                    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_HW_FLASH_FAIL, SYSTEMALARMS_SUBCODE_HF_FLASH_PARAMS, FALSE);
                    return COMMONPARAMDB_CH_HARDWAREFAILURE;
                }
    }

	return COMMONPARAMDB_CH_OK;
}

//***************************************************************************
// hook for parameter restore default

UWORD CanOpenCOE_ParamRestore(COMMONPARAMDB_ENTRY  * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext)
{
    static const CHARS sig[]="load";
    UWORD ct;

        // dummy to avoid compiler warning
    ct=hpsEntry->uwIndex;

    if(uwFlags&COMMONPARAMDB_CBFLAG_ABORT)
        return COMMONPARAMDB_CH_OK;

    if(uwElement>1)
        return COMMONPARAMDB_CH_INVALID_ELEMENT;

	if(uwFlags&COMMONPARAMDB_CBFLAG_INFOREQ)
	{
		COMMONPARAMDB_ENTRY  * hpsInfo=hpvBuffer;
		
        *hpsInfo=*hpsEntry;

		if(uwElement)
		{
            hpsInfo->ubFlags=COMMONPARAMDB_FLAG_RW;
            hpsInfo->ubType=COMMONPARAMDB_TYPE_ULONG;
            hpsInfo->uwNElements=0;
        }
        else
		{
            hpsInfo->ubFlags=COMMONPARAMDB_FLAG_RD;
            hpsInfo->ubType=COMMONPARAMDB_TYPE_UBYTE;
            hpsInfo->swMoreType=1;
        }
	
		return COMMONPARAMDB_CH_OK;
	}

    if(uwFlags&COMMONPARAMDB_CBFLAG_RD)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
            if(uwFlags&COMMONPARAMDB_CBFLAG_SIZEINQUIRY)
            {
                *((HPULONG)hpvBuffer)=uwElement?sizeof(ULONG):sizeof(UBYTE);
                *puwBufSize=sizeof(ULONG);
            }
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            if(uwElement)
            {
                    // store capability
                ULONG ulCap=1l;
                memcpy(hpvBuffer, &ulCap, sizeof(ULONG));
                *puwBufSize=sizeof(ULONG);
            }
            else
            {
                *((HPUBYTE)hpvBuffer)=1;
                *puwBufSize=sizeof(UBYTE);
            }
        }
    }

    if(uwFlags&COMMONPARAMDB_CBFLAG_WR)
    {
        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
        {
            if(uwElement==0)
                return COMMONPARAMDB_CH_NO_WRITE_ACCESS;
    
                // also accept zero length for clients that
                // do not send data length
            if(*((HPULONG)hpvBuffer)!=0l && *((HPULONG)hpvBuffer)!=4l)
                return COMMONPARAMDB_CH_WRONGLENGTH;
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
        {
            for(ct=0;ct<4;ct++)
                if(((HPUBYTE)hpvBuffer)[ct]!=sig[ct])
                    return COMMONPARAMDB_CH_INVALID_DATA;

            *hpulContext=TRUE;
        }
        else if(uwFlags&COMMONPARAMDB_CBFLAG_END)
            if(*hpulContext)
                if(parmgm_par_restore()!=PARMGM_B_OK)
                {
                    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_HW_FLASH_FAIL, SYSTEMALARMS_SUBCODE_HF_FLASH_PARAMS, FALSE);
                    return COMMONPARAMDB_CH_HARDWAREFAILURE;
                }
    }

	return COMMONPARAMDB_CH_OK;
}

//***************************************************************************
// hook for firmware/binaries download

#ifdef _APP_XC

#define SUBIDX_FIRMWARE         1
#define SUBIDX_PLCBINARIES      2
#define SUBIDX_PLCSOURCES       3
#define SUBIDX_HMI              4

#define SUBIDX_MAXINDEX         (SUBIDX_HMI)

//UWORD CanOpenCOE_FwDownload(COMMONPARAMDB_ENTRY  * hpsEntry, UWORD uwFlags, UWORD uwElement, HPVOID hpvBuffer, UWORD * puwBufSize, HPULONG hpulContext)
//{
//    static SFSTOR_WRSTATUS  sStrStatus={0ul};
//
//    if(uwFlags&COMMONPARAMDB_CBFLAG_ABORT)
//        return COMMONPARAMDB_CH_OK;
//
//    if(uwElement>SUBIDX_MAXINDEX)
//        return COMMONPARAMDB_CH_INVALID_ELEMENT;
//
//	if(uwFlags&COMMONPARAMDB_CBFLAG_INFOREQ)
//	{
//		COMMONPARAMDB_ENTRY  * hpsInfo=hpvBuffer;
//
//        *hpsInfo=*hpsEntry;
//
//		if(uwElement)
//		{
//            hpsInfo->ubFlags=COMMONPARAMDB_FLAG_WR;
//            hpsInfo->ubType=COMMONPARAMDB_TYPE_BINARY;
//            hpsInfo->uwNElements=0;
//        }
//        else
//		{
//            hpsInfo->ubFlags=COMMONPARAMDB_FLAG_RD;
//            hpsInfo->ubType=COMMONPARAMDB_TYPE_UBYTE;
//            hpsInfo->swMoreType=1;
//        }
//
//		return COMMONPARAMDB_CH_OK;
//	}
//
//    if(uwFlags&COMMONPARAMDB_CBFLAG_RD)
//    {
//        if(uwElement>0)
//            return COMMONPARAMDB_CH_NO_READ_ACCESS;
//
//        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
//        {
//            if(uwFlags&COMMONPARAMDB_CBFLAG_SIZEINQUIRY)
//            {
//                *((HPULONG)hpvBuffer)=sizeof(UBYTE);
//                *puwBufSize=sizeof(ULONG);
//            }
//        }
//        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
//        {
//            *((HPUBYTE)hpvBuffer)=SUBIDX_MAXINDEX;
//            *puwBufSize=sizeof(UBYTE);
//        }
//    }
//
//    if(uwFlags&COMMONPARAMDB_CBFLAG_WR)
//    {
//        if(uwFlags&COMMONPARAMDB_CBFLAG_INIT)
//        {
//            if(uwElement==0)
//                return COMMONPARAMDB_CH_NO_WRITE_ACCESS;
//
//            if( !PlcRequestStandby() )
//                return COMMONPARAMDB_CH_WRITE_DENIED;
//            if( !PlcLockForReload(FALSE) )
//                return COMMONPARAMDB_CH_WRITE_DENIED;
//
//            switch(uwElement)
//            {
//                case SUBIDX_FIRMWARE:
//                    if(SFStor_WriteBegin(&sStrStatus, SFPART_SYSAPP_START, SFPART_SYSAPP_SIZE)!=SFSTOR_STR_OK)
//                        return COMMONPARAMDB_CH_HARDWAREFAILURE;
//                    *hpulContext=0;
//                    break;
//
//                case SUBIDX_PLCBINARIES:
//                    if(PlcStreamCodeBegin())
//                        return COMMONPARAMDB_CH_HARDWAREFAILURE;
//                    *hpulContext=1;
//                    break;
//
//                case SUBIDX_PLCSOURCES:
//                    if(SFStor_WriteBegin(&sStrStatus, SFPART_PLC_PRJ_START, SFPART_PLC_PRJ_SIZE)!=SFSTOR_STR_OK)
//                        return COMMONPARAMDB_CH_HARDWAREFAILURE;
//                    *hpulContext=0;
//                    break;
//
//                case SUBIDX_HMI:
//                    if(SFStor_WriteBegin(&sStrStatus, SFPART_RD_HMI_APP_START, SFPART_RD_HMI_APP_SIZE)!=SFSTOR_STR_OK)
//                        return COMMONPARAMDB_CH_HARDWAREFAILURE;
//                    *hpulContext=0;
//                    break;
//
//                default:
//                    return COMMONPARAMDB_CH_NO_WRITE_ACCESS;
//            }
//        }
//        else if(uwFlags&COMMONPARAMDB_CBFLAG_SEGMENT)
//        {
//            if(*hpulContext)
//            {
//                if(PlcStreamCodeData(hpvBuffer, (UBYTE)*puwBufSize))
//                    return COMMONPARAMDB_CH_HARDWAREFAILURE;
//            }
//            else
//            {
//                if(SFStor_WriteData(&sStrStatus, hpvBuffer, (UBYTE)*puwBufSize)!=SFSTOR_STR_OK)
//                    return COMMONPARAMDB_CH_HARDWAREFAILURE;
//            }
//        }
//        else if(uwFlags&COMMONPARAMDB_CBFLAG_END)
//        {
//            if(*hpulContext)
//            {
//                if(PlcStreamCodeEnd())
//                    return COMMONPARAMDB_CH_HARDWAREFAILURE;
//            }
//            else
//            {
//                if(SFStor_WriteEnd(&sStrStatus)!=SFSTOR_STR_OK)
//                    return COMMONPARAMDB_CH_HARDWAREFAILURE;
//            }
//
//            if(uwElement==SUBIDX_FIRMWARE)
//                sCanOpenCOEReqReflash=TRUE;
//        }
//    }
//
//	return COMMONPARAMDB_CH_OK;
//}
#endif
