/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : WatchDogManagementRT.c                                     */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Infineon WDT module and FPGA WDT management                */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "system\SysAppGlobals.h"
#include "fpga\FpgaHandler.h"
#include "fpga\FpgaHandlerRT.h"
#include "WatchDogManagement.h"
#include "WatchDogManagementRT.h"
#include "system\SystemAlarms.h"
#include "system\SysLogManagement.h"

//***************************************************************************
// Globals

BOOL bWdtFastFailed=FALSE;
BOOL bWdtFastEnabled=FALSE;

//***************************************************************************
// fast task

BOOL WdtMgmRTTask(void)
{
        // run only if enabled and not PLC locked
    if(bWdtFastEnabled && !bSysStatLockedByPlcReload)
    {
        UWORD uwFailCode;

            // if wdt fail post alarm
        uwFailCode=FpgaWdtService();
        if(uwFailCode!=FPGA_WDTCHECK_NOERROR)
        {
            switch(uwFailCode)
            {
                case FPGA_WDTCHECK_WDT_EXPIRED:
                    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_SOFTWARE_FAIL, SYSTEMALARMS_SUBCODE_SF_TM_FPGA_WDT_FAIL, TRUE);
                    break;
            
                case FPGA_WDTCHECK_PLL_LOSSOFLOCK:
                    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_SOFTWARE_FAIL, SYSTEMALARMS_SUBCODE_SF_TM_FPGA_PLLLOSSOFLOCK, TRUE);
                    break;
            
                default:
                    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_SOFTWARE_FAIL, SYSTEMALARMS_SUBCODE_SF_TM_FPGA_LOCKOUT, TRUE);
                    break;
            }

                // signal fast task alarm
            bWdtFastFailed=TRUE;

                // and prevent it running
            bWdtFastEnabled=FALSE;
        }
    }

    return TRUE;
}

