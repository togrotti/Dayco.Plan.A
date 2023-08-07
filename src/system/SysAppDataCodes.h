/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppDataCodes.h                                          */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : System application data codes definition for block storage */
/*                                                                          */
/****************************************************************************/

#ifndef _SYSAPPDATACODES_H
#define _SYSAPPDATACODES_H

#define DATACODE_SYSLOG_INVALID                     0

//****************************************************************************
// SYSLOG data

#define DATACODE_SYSLOG_CLOCK                       4
#define DATACODE_SYSLOG_ALARMS                      5

#define DATACODE_SYSLOG_OS_STATISTICS               6
#define DATACODE_SYSLOG_ALARM_V1_DATA               7
#define DATACODE_SYSLOG_ALARM_DATA                  8
#define DATACODE_SYSLOG_PLC_RETAIN                  9

//****************************************************************************
// PARAM mgm

#define DATACODE_PARAM_ENDSIGNATURE                 16
#define DATACODE_PARAM_SYSTEMRESET                  17
#define DATACODE_PARAM_BOOTBLOCK_INFO               18
#define DATACODE_PARAM_BOOTBLOCK_SYSID              19
#define DATACODE_PARAM_BOOTBLOCK_APPID              20

//****************************************************************************
// PARAM data

#define DATACODE_PARAM_MOTORPARAMETERS              33
#define DATACODE_PARAM_V14_ENCODER_MANAGER          34
#define DATACODE_PARAM_INCR_MAIN_ENC                36
#define DATACODE_PARAM_INCR_AUX__ENC                37
#define DATACODE_PARAM_SINCOS_ENC                   38
#define DATACODE_PARAM_ENDAT_MAIN_ENC               39
#define DATACODE_PARAM_ENDAT_AUX_ENC                40
#define DATACODE_PARAM_BACKEMF_ENC                  41
#define DATACODE_PARAM_ENCMGR_EFA                   42
#define DATACODE_PARAM_SIMULATED_ENC                43
#define DATACODE_PARAM_ENCODER_MANAGER              44
#define DATACODE_PARAM_EM_LASTVALIDEPLATE           45
#define DATACODE_PARAM_HALL_ENC                     46
#define DATACODE_PARAM_HIPFC_ENC                    47
#define DATACODE_PARAM_NIKON_MAIN_ENC               48
#define DATACODE_PARAM_TMGW_MAIN_ENC                49

#define DATACODE_PARAM_V15_SS_CNTRLOOP              50
#define DATACODE_PARAM_POSITIONER                   51
#define DATACODE_PARAM_MOTIONCONTROLLER             52
#define DATACODE_PARAM_MOTORHANDLER                 53
#define DATACODE_PARAM_MOTORHANDLER_EXT             54
#define DATACODE_PARAM_THERMAL_MODEL                55
#define DATACODE_PARAM_PI_DCBUS                     56
#define DATACODE_PARAM_AFECONTROLLER                58
#define DATACODE_PARAM_SPACESPEED_CNTRLOOP          59
#define DATACODE_PARAM_DEFLUX                       60

#define DATACODE_PARAM_BISS_MAIN_ENC                61
#define DATACODE_PARAM_BISS_AUX_ENC                 62

#define DATACODE_PARAM_CANOPEN_BASE                 80
#define DATACODE_PARAM_CANOPENCOMMANDMGR            81
#define DATACODE_PARAM_CANOPEN_TXCOB                82
#define DATACODE_PARAM_CANOPEN_RXCOB                83

#define DATACODE_PARAM_MODBUS_BASE                  85
#define DATACODE_PARAM_MODBUS_OVERSERIAL0           86
#define DATACODE_PARAM_MODBUS_OVERETHERNET          87

#define DATACODE_PARAM_SYNCMANAGER                  90

#define DATACODE_PARAM_PLC                          104
#define DATACODE_PARAM_PLC_USRPARAM_COMMON          105
#define DATACODE_PARAM_PLC_USRPARAM_BITS            106
#define DATACODE_PARAM_PLC_USRPARAM_ID              107
#define DATACODE_PARAM_PLC_USRPARAM_MAP             108

#define DATACODE_PARAM_ECAT_BASE                    110
#define DATACODE_PARAM_ECAT_TXPDOMAP                111
#define DATACODE_PARAM_ECAT_RXPDOMAP                112
#define DATACODE_PARAM_ECAT_SMOUTMAP                113
#define DATACODE_PARAM_ECAT_SMINMAP                 114
#define DATACODE_PARAM_ECAT_SYNCMGR                 115

#define DATACODE_PARAM_ETHERPMC                     117

#ifdef _HW_CT
#define DATACODE_PARAM_SENSOR                       118
#endif

#define DATACODE_PARAM_DTCTRL_OLD_PARAM             120
#define DATACODE_PARAM_DTCTRL_NEW_PARAM             121

#define DATACODE_PARAM_HWCONFIG                     130

#ifdef _HW_AXS_CABI35KW
#define DATACODE_PARAM_STGAP4S_CONFIG               131
#endif


//****************************************************************************
// DO NOT USE data codes

#define DATACODE_PARAM_DNU_1                        192
#define DATACODE_PARAM_DNU_2                        35
#define DATACODE_PARAM_DNU_3                        100
#define DATACODE_PARAM_DNU_4                        101
#define DATACODE_PARAM_DNU_5                        102
#define DATACODE_PARAM_DNU_6                        103
#define DATACODE_PARAM_DNU_7                        32
#define DATACODE_PARAM_DNU_8                        57

#define DATACODE_PARAM_DNU_9                        226
#define DATACODE_PARAM_DNU_A                        227
#define DATACODE_PARAM_DNU_B                        228
#define DATACODE_PARAM_DNU_C                        232
#define DATACODE_PARAM_DNU_D                        242

//****************************************************************************
// HARDWARE data

#define DATACODE_HW_CNTRL_BOARD                     224
#define DATACODE_HW_POWER_BOARD_AXM_AXP_AC          225
#define DATACODE_HW_ASSEMBLY_V19_IDENT              229
#define DATACODE_HW_POWER_BOARD_AXM_AXP_AC_V2       230
#define DATACODE_HW_POWER_BOARD_DIF                 231
#define DATACODE_HW_IO_EXP_BOARD                    233

#define DATACODE_HW_ASSEMBLY_IDENT                  234
#define DATACODE_HW_PB_ASSEMBLY                     235
#define DATACODE_HW_PB_MBP                          236
#define DATACODE_HW_PB_DS_MBP                       237
#define DATACODE_HW_PB_DS_PSI                       238
#define DATACODE_HW_PB_TS_PSI                       239
#define DATACODE_HW_PB_TS_FTB                       240
#define DATACODE_HW_PB_TS_GDB                       241

#ifdef _APP_DEBUG
#define DATACODE_HW_PB_DBGMBP                       243
#endif

#define DATACODE_HW_PB_XS_PSI                       244
#define DATACODE_HW_PB_XS_BRI                       245

#define DATACODE_HW_CT_CB                           246
#define DATACODE_HW_CT_EM                           247
#define DATACODE_HW_CT_EA                           248
#define DATACODE_HW_CT_IO                           249
#define DATACODE_HW_CT_BP                           250
#define DATACODE_HW_CT_PSU                          251

#define DATACODE_HW_PB_PSU_PSB                      252
#define DATACODE_HW_PB_PSU_BRK                      253
#define DATACODE_HW_PB_PSU_SCR                      254

#endif
