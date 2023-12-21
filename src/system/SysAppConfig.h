/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppConfig.h                                             */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Globals configuration macros for all the modules           */
/*                                                                          */
/****************************************************************************/

#ifndef _SYSAPPCONFIG_H
#define _SYSAPPCONFIG_H

//***************************************************************************
// Os Configuration

// Slice time (usec), possible values are limited to:
// 3277, 6554, 13107 and 26214 @ 80MHZ (2097, 4195, 8388  and 16777 @ 125MHZ)
#define OS_SLICETIME_COMP           13107
#define OS_SLICETIME_PERF           26214

// Maximum concurrent tasks allowed
#define OS_MAXTASKSALLOWED          7

// Event queue depth
#define OS_EVENTQUEUEDEPTH          4

// Enable measurement of unused stacks size
#define OS_MEASURESTACKSIZE         1

// Default reserved static user stack per task
#define OS_DEFAULTUSRSTATICSTACK    configMINIMAL_STACK_SIZE

// Reserved static user stack for main task
#define OS_MAINUSRSTATICSTACK      (configMINIMAL_STACK_SIZE*2)

//***************************************************************************
// Power Fail

#define PF_TIMEOUTONSTARTUP         750     // msec

//***************************************************************************
// Reset button debounce filter

#define PF_RSTBUTPERIOD             1000    // usec

//***************************************************************************
// MultiCANController

#define CFG_CANDRV_PERIODICCOB      1
#define CFG_CANDRV_PLCWRAPPER       1
#define CFG_CANDRV_STATUSSIGNAL     1

#ifndef _HW_AXS

#ifndef _HW_DC
#define CFG_IIC						0
#define CFG_CANDRV_DS301            1
#define CFG_CANDRV_CMDMGR           1
#else
#define CFG_IIC						1
#define CFG_CANDRV_DS301            0
#define CFG_CANDRV_CMDMGR           0
#endif

#else // _HW_AXS

#define CFG_IIC						0
#define CFG_CANDRV_DS301            1
#define CFG_CANDRV_CMDMGR           1

#endif

#ifdef _APP_XC
#define _CANDRV_EXTD
#endif
//***************************************************************************
// CanOpen DS301

#define CFG_DS301_DEFTIMING                 4
#define CFG_DS301_DEFNODEID                 1

#define CFG_DS301_PDO
#define CFG_DS301_PDO_TX_TOT                8
#define CFG_DS301_PDO_RX_TOT                8
#define CFG_DS301_PDO_AVG_ELEM_PER_PDO      4
#define CFG_DS301_MAXDATAOBJECT             8

#define CFG_DS301_IDENTITY
#define CFG_DS301_ERRPROT
#define CFG_DS301_NMT
#define CFG_DS301_EMCY
#define CFG_DS301_LSS
#define CFG_DS301_SYNC

//***************************************************************************
// Encoder Manager

#if !defined(_HW_AXS) // AxN & AxD
#define CFG_ENC_SINCOS            1
#define CFG_ENC_INCR              1
#define CFG_ENC_HALL              1
#define CFG_ENC_BEMF              1
#define CFG_ENC_EFS               1
#define CFG_ENC_ENDAT             1
#define CFG_ENC_HIP               1
#define CFG_ENC_NIKON             0
#define CFG_ENC_TMGW              0
#define CFG_ENC_BISS              1
#else // _hw_axs
#define CFG_ENC_SINCOS            1
#define CFG_ENC_INCR              1
#define CFG_ENC_HALL              0
#define CFG_ENC_BEMF              1
#define CFG_ENC_EFS               1
#define CFG_ENC_ENDAT             1
#define CFG_ENC_HIP               0
#define CFG_ENC_NIKON             0
#define CFG_ENC_TMGW              0
#define CFG_ENC_BISS              0
#endif

#define ENDAT22                   1
//#define ENDAT22_ADDINFO           1
#define CFG_ENC_BEMF_DITEN        1
#define CFG_ENC_BEMF_DITEN_PARAM  1
#define CFG_ENCMGR_OPENLOOP       1
#define CFG_DFLX_VMOTOR	          1
#define CFG_IPM_PHASEOFFSET       1
//***************************************************************************
// Define Modbus

#ifdef _AXX_SYSAPP
#if !defined(_HW_DC) && !defined(_HW_CT)
#ifndef _APP_DEBUG          // only in release, as in debug mode CAN1 pins overlap
                            // with JTAG pins
// #define CFG_EN_MODBUSOVERCAN
#endif
#endif
#endif

// #define CFG_EN_MODBUSOVERUSB
// #define CFG_EN_MODBUSOVERETH

//***************************************************************************
// Define EtherCAT
#if !defined(_HW_AXS) // AxN & AxN-DC
#define CFG_ECAT    1
#else // _hw_axs
#define CFG_ECAT	0
#endif

// Define EthPMC (available only in AxN-DC)
#if !defined(_HW_AXS) && defined(_HW_DC)
#define CFG_ETHPMC	1 // AxN-DC
#else // _hw_axs
#define CFG_ETHPMC	0 // AxN, AxS
#endif
// Define PSU Manager (available only in AxN-DC)
#if !defined(_HW_AXS) && defined(_HW_DC)
#define CFG_PSU_MNGR	1 // AxN-DC
#else // _hw_axs
#define CFG_PSU_MNGR	0 // AxN, AxS
#endif // _hw_axs


//***************************************************************************
// Manage fast task overtime
#if defined(_CRS_DBG)

#define CRS_DBGDSK 0

#if CRS_DBGDSK
#define	CFG_FASTTASKOVERTIMEDISABLE 1
#else
#define	CFG_FASTTASKOVERTIMEDISABLE 0
#endif

#else
#define CRS_DBGDSK 0
#define	CFG_FASTTASKOVERTIMEDISABLE 0
#endif

#if defined(_HW_AXS) &&  defined(_HW_AXS_DAYCO22KW)
#define CFG_ASC_DAYCO_MGMT  1
#define CFG_VMOTOR_READ     1
#else
#define CFG_ASC_DAYCO_MGMT  0
#define CFG_VMOTOR_READ     0
#endif

#if !defined(_HW_AXS) && defined(_HW_DC)
#define CFG_PWM_SHIFT       1 // available only for multibridge drive (= AxN-DC)
#else
#define CFG_PWM_SHIFT       0
#endif

//***************************************************************************
// PLC
#define CFG_PLC_DIAGNOSTIC 0	// load the plc application to autosetup the system

#endif
