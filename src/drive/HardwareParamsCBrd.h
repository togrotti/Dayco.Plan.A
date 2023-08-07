/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : HardwareParamsCBrd.h                                       */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Definition of hardware specific parameters, for            */
/*               the new controlboards                                      */
/*                                                                          */
/****************************************************************************/

#ifndef _HARDWAREPARAMSCBRD_H
#define _HARDWAREPARAMSCBRD_H

#include "common\CommonDefines.h"
#include "HardwareParameters.h"

//***************************************************************************
// Analog channel span

typedef struct
{
    FLOAT                           flLoSpan;
    FLOAT                           flHiSpan;
} HWPRM2_ANALOG_RANGE;

//***************************************************************************
// Analog calibration with formula/linearization table selection
// and/or unit-dependant options

typedef struct
{
    UBYTE                           ubType;     // 0: reserved
    UBYTE                           ubReserved;
    HWPRMS_AD_SETTINGS              sCalib;
} HWPRM2_ANALOG_TYPE;

//***************************************************************************
// Analog types

#define HWPRM2_ANTYPE_STD_BOARDTEMP         1
#define HWPRM2_ANTYPE_EM_VPWR               2
#define HWPRM2_ANTYPE_EA_VPWR               3
#define HWPRM2_ANTYPE_STD_VAUXSENSE         4

//***************************************************************************
// Group data structures
//***************************************************************************

#ifdef _HW_DC

//***************************************************************************
// Controlboard - CPU

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_HW_UNITS                 sUnit;
    UWORD                           uwFpgaType;     // HWPRM_CTRLBRD_FPGA_*
    HWPRM2_ANALOG_TYPE              sBoardTemp;
}
#ifdef _INFINEON_
                        HWGRP_CT_CB;
#else
__attribute__((packed)) HWGRP_CT_CB;
#endif

//***************************************************************************
// Controlboard - Encoder main

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_HW_UNITS                 sUnit;

    HWPRM2_ANALOG_TYPE              sEMVfb;
    HWPRM2_ANALOG_RANGE             sKTYRange;      // Ohm
    HWPRMS_AD_SETTINGS              sKTYCalibr;
    HWPRM2_ANALOG_RANGE             sPTCRange;      // Ohm
    HWPRMS_AD_SETTINGS              sPTCCalibr;

    HWPRMS_AD_SETTINGS              sIncEncSin;
    HWPRMS_AD_SETTINGS              sIncEncCos;

    HWPRMS_AD_SETTINGS              sAbsEncHISin;
    HWPRMS_AD_SETTINGS              sAbsEncHICos;
    HWPRMS_AD_SETTINGS              sAbsEncLOSin;
    HWPRMS_AD_SETTINGS              sAbsEncLOCos;
}
#ifdef _INFINEON_
                        HWGRP_CT_EM;
#else
__attribute__((packed)) HWGRP_CT_EM;
#endif

//***************************************************************************
// Controlboard - Encoder aux

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_HW_UNITS                 sUnit;
    HWPRM2_ANALOG_TYPE              sEAVfb;
}
#ifdef _INFINEON_
                        HWGRP_CT_EA;
#else
__attribute__((packed)) HWGRP_CT_EA;
#endif

//***************************************************************************
// Controlboard - Standard I/O

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_HW_UNITS                 sUnit;
    HWPRM2_ANALOG_RANGE             sInRange;       // V
    HWPRMS_AD_SETTINGS              sUserRefI0;
    HWPRMS_AD_SETTINGS              sUserRefI1;
    HWPRMS_AD_SETTINGS              sUserRefI2;
    HWPRMS_AD_SETTINGS              sUserRefI3;

    HWPRM2_ANALOG_RANGE             sOutRange;      // V
    HWPRMS_DA_SETTINGS              sUserAnaO0;
    HWPRMS_DA_SETTINGS              sUserAnaO1;
}
#ifdef _INFINEON_
                        HWGRP_CT_IO;
#else
__attribute__((packed)) HWGRP_CT_IO;
#endif

//***************************************************************************
// Controlboard - Backplane

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_HW_UNITS                 sUnit;
    HWPRM2_ANALOG_TYPE              sVPwrSense;
}
#ifdef _INFINEON_
                        HWGRP_CT_BP;
#else
__attribute__((packed)) HWGRP_CT_BP;
#endif

//***************************************************************************
// Controlboard - PSU

typedef struct
{
    UWORD                           uwProductRev;   // always first entry
    HWPRM2_HW_UNITS                 sUnit;
}
#ifdef _INFINEON_
                        HWGRP_CT_PSU;
#else
__attribute__((packed)) HWGRP_CT_PSU;
#endif

//***************************************************************************
// Resulting controlboard parameters

typedef struct
{
    HWPRMS_PRODUCT_INFO             sProductInfo;
    HWPRM2_HW_UNITS                 sUnit;
    UWORD                           uwFpgaType;     // HWPRM_CTRLBRD_FPGA_*
    HWPRM2_ANALOG_TYPE              sBoardTemp;
    HWPRM2_ANALOG_TYPE              sVPwrSense;
    HWPRM2_ANALOG_TYPE              sEMVfb;
    HWPRM2_ANALOG_TYPE              sEAVfb;
    HWPRM2_ANALOG_RANGE             sKTYRange;      // Ohm
    HWPRMS_AD_SETTINGS              sKTYCalibr;
    HWPRM2_ANALOG_RANGE             sPTCRange;      // Ohm
    HWPRMS_AD_SETTINGS              sPTCCalibr;
    HWPRMS_AD_SETTINGS              sIncEncSin;
    HWPRMS_AD_SETTINGS              sIncEncCos;
    HWPRMS_AD_SETTINGS              sAbsEncHISin;
    HWPRMS_AD_SETTINGS              sAbsEncHICos;
    HWPRMS_AD_SETTINGS              sAbsEncLOSin;
    HWPRMS_AD_SETTINGS              sAbsEncLOCos;
    HWPRM2_ANALOG_RANGE             sInRange;
    HWPRMS_AD_SETTINGS              sUserRefI0;
    HWPRMS_AD_SETTINGS              sUserRefI1;
    HWPRMS_AD_SETTINGS              sUserRefI2;
    HWPRMS_AD_SETTINGS              sUserRefI3;
    HWPRM2_ANALOG_RANGE             sOutRange;
    HWPRMS_DA_SETTINGS              sUserAnaO0;
    HWPRMS_DA_SETTINGS              sUserAnaO1;
} HWGRP_CBRD_UNIVERSAL;

#endif

//***************************************************************************
// All controlboards

typedef union
{
    // Always first entry in the structure
    HWPRMS_PRODUCT_INFO             sProductInfo;
#ifndef _HW_DC
    HWPRM_CNTRL_BOARD               sStd;
#else
    HWGRP_CBRD_UNIVERSAL            sUniv;
#endif
} HWPRM_ALL_CBRD; 

#endif
