/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ParametersCheckCodes.h                                     */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Codes for validity checker                                 */
/*                                                                          */
/****************************************************************************/

#ifndef _PARAMETERSCHECKCODES_H
#define _PARAMETERSCHECKCODES_H

//***************************************************************************
// Global Motor Parameters

#define PARCC_MOTOR_RESISTANCE                  27803
#define PARCC_MOTOR_INDUCTANCE                  27804
#define PARCC_MOTOR_KT                          27805
#define PARCC_MOTOR_POLENUMBERS                 27814
#define PARCC_MOTOR_CURRENTPEAK                 27808
#define PARCC_MOTOR_DIRECTINDUCTANCE            27818

//***************************************************************************
// Hardware Config

#define PARCC_HWCONF_CORESPEED                  19100

//***************************************************************************
// Drive Task Controller

#define PARCC_DRVTSKCTRL_MAINOPERATION          22000

//***************************************************************************
// Motor Handler

#define PARCC_MH_BRAKEVOLTAGERANGE              27400
#define PARCC_MH_UNDERVOLTAGE                   27402
#define PARCC_MH_OVERVOLTAGE                    27403
#define PARCC_MH_OVERCURRENT                    27404
#define PARCC_MH_USERBRIDGELAYOUT               27490
#define PARCC_MH_BRIDGEPARMISMATCH              1400

//***************************************************************************
// Encoder Manager

#define PARCC_ENCMGR_ENCODERVOLTAGE             26000
#define PARCC_ENCMGR_VOLTAGEDELAY               26008
#define PARCC_ENCMGR_MAINABSSEL                 26001
#define PARCC_ENCMGR_MAINRELSEL                 26009
#define PARCC_ENCMGR_AUXSEL                     26002
#define PARCC_ENCMGR_AUXENCODERVOLTAGE          26106
#define PARCC_ENCMGR_SIMSEL                     26107
#define PARCC_ENCMGR_AUXSIMMISMATCH             1450

//***************************************************************************
// Encoder EF Seek

#define PARCC_EFS_PROCTYPE                      26302
#define PARCC_EFS_SPEEDTHRESHOLD                26304

//***************************************************************************
// Encoders - Endat

#define PARCC_ENDAT_MAINFREQ                    26020
#define PARCC_ENDAT_AUXFREQ                     26120

//***************************************************************************
// Encoders - Biss

#define PARCC_BISS_MAINFREQ                     26088
#define PARCC_BISS_AUXFREQ                      26260

//***************************************************************************
// Encoders - Analog (SinCos)

#define PARCC_ANALOG_POLENUMBERS                26052

//***************************************************************************
// Encoders - Hall

#define PARCC_HALL_POLENUMBERS                  26042

//***************************************************************************
// Encoders - Incremental

#define PARCC_INCR_MAINPOLENUMBERS              26032
#define PARCC_INCR_MAINLINENUMBERS              26030
#define PARCC_INCR_MAININDEXTOLERANCE           26037
#define PARCC_INCR_MAINMAXFREQUENCY             27300
#define PARCC_INCR_MAININTERPSWITCHFREQ         27307

#define PARCC_INCR_AUXPOLENUMBERS               26132
#define PARCC_INCR_AUXLINENUMBERS               26130
#define PARCC_INCR_AUXINDEXTOLERANCE            26135
#define PARCC_INCR_AUXMAXFREQUENCY              27320
#define PARCC_INCR_AUXINTERPSWITCHFREQ          27327

//***************************************************************************
// Encoders - Incremental simulation

#define PARCC_INCRSIM_IDXPULSEREPEAT            26146
#define PARCC_INCRSIM_IDXOFFSET                 26147
#define PARCC_INCRSIM_ENABLESTEPDIR             26138
#define PARCC_INCRSIM_ENABLEUPDOWN              26139
#define PARCC_INCRSIM_MAINLINENUMBERS           26149
#define PARCC_INCRSIM_MAINMAXFREQUENCY          26150
#define PARCC_INCRSIM_ENABLEFASTAUXSIM          1470
#define PARCC_INCRSIM_RATIOFASTAUXSIM           1471

//***************************************************************************
// Encoders - BackEMF

#define PARCC_BEMF_BACKONTHEFLYUSRSPEED         26069
#define PARCC_BEMF_SENSORLESSSPEED              26070
#define PARCC_BEMF_SPEEDTHRESHOLD1              26071
#define PARCC_BEMF_SPEEDTHRESHOLD0              26072
#define PARCC_BEMF_GLITCHFILTER                 26073
#define PARCC_BEMF_INVALIDBACKONTHEFLY          1545
#define PARCC_BEMF_INVMAGICNUMBER               1546

//***************************************************************************
// Space Speed control loop

#define PARCC_SS_INVALIDGLOBALSHIFT             27009
#define PARCC_SS_INVALIDPOSSHIFT                27007
#define PARCC_SS_INVALIDACCSHIFT                27008
#define PARCC_SS_OUTOFRANGEGAINS                1600

//***************************************************************************
// Thermal model

#define PARCC_TM_INVALIDCOOLINGTHRESHOLDS       1700
#define PARCC_TM_INVALIDBRAKE_RESISTANCE        28100
#define PARCC_TM_INVALIDBRAKE_POWER             28101
#define PARCC_TM_INVALIDBRAKE_ENERGY            28107

#warning "TBD onchip invalid overtemp parameter"
#ifndef _INFINEON_
#define PARCC_TM_INVALIDONCHIPOVETERMPTHRESHOLDS 1700
#endif

//***************************************************************************
// Modbus

#define PARCC_MODBUS_INVSLAVEADDRESS            18070
#define PARCC_MODBUS_INVBAUDRATE                18031
#define PARCC_MODBUS_INVDATABITS                18032
#define PARCC_MODBUS_INVPARITY                  18033
#define PARCC_MODBUS_INVSTOP                    18034

//***************************************************************************
// CANOpen

#define PARCC_CANOPEN_WRONGNODEID               30001
#define PARCC_CANOPEN_WRONGBAUDRATE             30002

//***************************************************************************
// Motion Controller

#define PARCC_MOTCTRL_MODEOFOPERATION           29016
#define PARCC_MOTCTRL_QUICKSTOP                 29020
#define PARCC_MOTCTRL_SHUTDOWN                  29021
#define PARCC_MOTCTRL_DISABLEOP                 29022
#define PARCC_MOTCTRL_HALT                      29023
#define PARCC_MOTCTRL_FAULTREACTION             29024
#define PARCC_MOTCTRL_IPTIMINGUNITS             29033
#define PARCC_MOTCTRL_IPTIMINGINDEX             29034
#define PARCC_MOTCTRL_HMMETHOD                  29035
#define PARCC_MOTCTRL_HMPOSSWITCH               29036
#define PARCC_MOTCTRL_HMNEGSWITCH               29037
#define PARCC_MOTCTRL_HMHOMESWITCH              29038
#define PARCC_MOTCTRL_CSMOTORRATEDTORQUE        29043

#endif
