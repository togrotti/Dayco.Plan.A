/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : FpgaHandlerRT.c                                            */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : FPGA real time WDT management                              */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "system\SysAppGlobals.h"
#include "fpga\FpgaHandler.h"
#include "fpga\FpgaHandlerRT.h"
#include "system\SystemAlarms.h"
#include "system\SysLogManagement.h"

//***************************************************************************
// Globals

UWORD uwFpgaWdtKey;

//***************************************************************************
// WDT servicing

UWORD FpgaWdtService(void)
{
        // expected key
    uwFpgaWdtKey++;
#ifdef _INFINEON_
        // if different or setup by FPGA, wdt fail (and force FPGA reset)
    if(FPGA_WDT_KEY!=uwFpgaWdtKey || FPGA_GENERIC_FAULTS_WDT_EXPIRED)
    {
        FpgaReset();
        return FPGA_WDTCHECK_WDT_EXPIRED;
    }

        // if in safe lock state then fail
    if(FPGA_SAFE_LOCK==0)
    {
        if(FPGA_GENERIC_FAULTS_WDT_EXPIRED)
            return FPGA_WDTCHECK_WDT_EXPIRED;

        if(FPGA_GENERIC_FAULTS_PLL_LOSSOFLOCK)
            return FPGA_WDTCHECK_PLL_LOSSOFLOCK;

        return FPGA_WDTCHECK_SAFE_LOCKED;
    }

        // calc new key and service wdt
    uwFpgaWdtKey++;
    FPGA_WDT_KEY=uwFpgaWdtKey;
#endif
    return FPGA_WDTCHECK_NOERROR;
}

