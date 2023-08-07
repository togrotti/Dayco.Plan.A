/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppCanOpenParamInfo.c                                   */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : System application parameter info table                    */
/*                                                                          */
/****************************************************************************/
// Compiler Option
#pragma GCC optimize (2)

#include "bus\canopen\CanOpenComDBInfo.h"

//***************************************************************************
// String table

static const VISIBLE_STRING  sOdDs_MapObj1[]="1. mapped object";
static const VISIBLE_STRING  sOdDs_MapObj2[]="2. mapped object";
static const VISIBLE_STRING  sOdDs_MapObj3[]="3. mapped object";
static const VISIBLE_STRING  sOdDs_MapObj4[]="4. mapped object";
static const VISIBLE_STRING  sOdDs_MapObj5[]="5. mapped object";
static const VISIBLE_STRING  sOdDs_MapObj6[]="6. mapped object";
static const VISIBLE_STRING  sOdDs_MapObj7[]="7. mapped object";
static const VISIBLE_STRING  sOdDs_MapObj8[]="8. mapped object";
static const VISIBLE_STRING  sOdDs_Angle[]="Angle";
static const VISIBLE_STRING  sOdDs_CobIDPdo[]="COB-ID used by PDO";
static const VISIBLE_STRING  sOdDs_AbsPosOffset[]="Encoder Abs Track: Absolute position offset";
static const VISIBLE_STRING  sOdDs_AbsFb[]="Encoder Abs Track: Feedback";
static const VISIBLE_STRING  sOdDs_AuxPosOffset[]="Encoder Aux: Absolute position offset";
static const VISIBLE_STRING  sOdDs_AuxFb[]="Encoder Aux: Feedback";
static const VISIBLE_STRING  sOdDs_FbPosOffset[]="Encoder Feedback: Absolute position offset";
static const VISIBLE_STRING  sOdDs_FbFb[]="Encoder Feedback: Feedback";
static const VISIBLE_STRING  sOdDs_RelPosOffset[]="Encoder Rel Track: Absolute position offset";
static const VISIBLE_STRING  sOdDs_RelFb[]="Encoder Rel Track: Feedback";
static const VISIBLE_STRING  sOdDs_InhibitTime[]="Inhibit Time";
static const VISIBLE_STRING  sOdDs_NoOfEntries[]="Number of entries";
static const VISIBLE_STRING  sOdDs_NoOfMappedObj[]="Number of mapped objects";
static const VISIBLE_STRING  sOdDs_DemandPos[]="Positioner: Demand position";
static const VISIBLE_STRING  sOdDs_TxType[]="Transmission Type";
static const VISIBLE_STRING  sOdDs_Turns[]="Turns";
static const VISIBLE_STRING  sOdDs_1000_ff[]="Device Type";
static const VISIBLE_STRING  sOdDs_1001_ff[]="Error Register";
static const VISIBLE_STRING  sOdDs_1018_ff[]="Identity Record";
static const VISIBLE_STRING  sOdDs_1018_01[]="Vendor ID";
static const VISIBLE_STRING  sOdDs_1018_02[]="Product Code";
static const VISIBLE_STRING  sOdDs_1018_03[]="Revision";
static const VISIBLE_STRING  sOdDs_1018_04[]="Serial";
static const VISIBLE_STRING  sOdDs_1002_ff[]="Manufacturer Status Register";
static const VISIBLE_STRING  sOdDs_1005_ff[]="COB-ID SYNC message";
static const VISIBLE_STRING  sOdDs_1008_ff[]="Manufacturer Device Name";
static const VISIBLE_STRING  sOdDs_100A_ff[]="Manufacturer Software Version";
static const VISIBLE_STRING  sOdDs_100C_ff[]="Guard Time";
static const VISIBLE_STRING  sOdDs_100D_ff[]="Life Time Factor";
static const VISIBLE_STRING  sOdDs_1010_ff[]="Store Parameters";
static const VISIBLE_STRING  sOdDs_1010_01[]="Save all parameters";
static const VISIBLE_STRING  sOdDs_1011_ff[]="Restore default Parameters";
static const VISIBLE_STRING  sOdDs_1011_01[]="Restore all default parameters";
static const VISIBLE_STRING  sOdDs_1014_ff[]="COB-ID Emergency Message";
static const VISIBLE_STRING  sOdDs_1015_ff[]="Inhibit Time Emergency Message";
static const VISIBLE_STRING  sOdDs_1017_ff[]="Producer Heartbeat Time";
static const VISIBLE_STRING  sOdDs_1800_ff[]="TPDO 1 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1801_ff[]="TPDO 2 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1802_ff[]="TPDO 3 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1803_ff[]="TPDO 4 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1804_ff[]="TPDO 5 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1805_ff[]="TPDO 6 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1806_ff[]="TPDO 7 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1807_ff[]="TPDO 8 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1A00_ff[]="TPDO 1 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1A01_ff[]="TPDO 2 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1A02_ff[]="TPDO 3 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1A03_ff[]="TPDO 4 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1A04_ff[]="TPDO 5 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1A05_ff[]="TPDO 6 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1A06_ff[]="TPDO 7 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1A07_ff[]="TPDO 8 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1400_ff[]="RPDO 1 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1401_ff[]="RPDO 2 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1402_ff[]="RPDO 3 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1403_ff[]="RPDO 4 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1404_ff[]="RPDO 5 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1405_ff[]="RPDO 6 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1406_ff[]="RPDO 7 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1407_ff[]="RPDO 8 Communication Parameter";
static const VISIBLE_STRING  sOdDs_1600_ff[]="RPDO 1 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1601_ff[]="RPDO 2 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1602_ff[]="RPDO 3 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1603_ff[]="RPDO 4 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1604_ff[]="RPDO 5 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1605_ff[]="RPDO 6 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1606_ff[]="RPDO 7 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_1607_ff[]="RPDO 8 Mapping Parameter";
static const VISIBLE_STRING  sOdDs_10F0_01[]="Backup Parameter Checksum";
static const VISIBLE_STRING  sOdDs_10F0_ff[]="Backup Parameter";
static const VISIBLE_STRING  sOdDs_1C00_01[]="Communication Type SM 0";
static const VISIBLE_STRING  sOdDs_1C00_02[]="Communication Type SM 1";
static const VISIBLE_STRING  sOdDs_1C00_03[]="Communication Type SM 2";
static const VISIBLE_STRING  sOdDs_1C00_04[]="Communication Type SM 3";
static const VISIBLE_STRING  sOdDs_1C00_ff[]="Sync Manager Communication Type";
static const VISIBLE_STRING  sOdDs_1C02_01[]="SM-Event Missed Counter";
static const VISIBLE_STRING  sOdDs_1C02_02[]="Shift Time Too Short Counter";
static const VISIBLE_STRING  sOdDs_1C02_03[]="Cycle Time Exceeded Counter";
static const VISIBLE_STRING  sOdDs_1C02_04[]="SYNC0 Missed Counter";
static const VISIBLE_STRING  sOdDs_1C02_ff[]="Cycle diagnosis";
static const VISIBLE_STRING  sOdDs_1C32_01[]="Synchronization Type";
static const VISIBLE_STRING  sOdDs_1C32_02[]="Cycle Time";
static const VISIBLE_STRING  sOdDs_1C32_03[]="Shift Time";
static const VISIBLE_STRING  sOdDs_1C32_04[]="Synchronization Types supported";
static const VISIBLE_STRING  sOdDs_1C32_05[]="Minimum Cycle Time";
static const VISIBLE_STRING  sOdDs_1C32_06[]="Calc and Copy Time";
static const VISIBLE_STRING  sOdDs_1C32_09[]="Delay Time";
static const VISIBLE_STRING  sOdDs_1C32_0A[]="Sync0 Cycle Time";
static const VISIBLE_STRING  sOdDs_1C32_0B[]="Cycle Time Too Small";
static const VISIBLE_STRING  sOdDs_1C32_0C[]="SM-Event Missed";
static const VISIBLE_STRING  sOdDs_1C32_20[]="Sync Error ";
static const VISIBLE_STRING  sOdDs_1C32_ff[]="Output Sync Manager Parameter";
static const VISIBLE_STRING  sOdDs_1C33_ff[]="Input Sync Manager Parameter";
static const VISIBLE_STRING  sOdDs_1C12_00[]="Number of assigned RXPDO";
static const VISIBLE_STRING  sOdDs_1C12_01[]="1. PDO mapped";
static const VISIBLE_STRING  sOdDs_1C12_02[]="2. PDO mapped";
static const VISIBLE_STRING  sOdDs_1C12_03[]="3. PDO mapped";
static const VISIBLE_STRING  sOdDs_1C12_04[]="4. PDO mapped";
static const VISIBLE_STRING  sOdDs_1C12_05[]="5. PDO mapped";
static const VISIBLE_STRING  sOdDs_1C12_06[]="6. PDO mapped";
static const VISIBLE_STRING  sOdDs_1C12_07[]="7. PDO mapped";
static const VISIBLE_STRING  sOdDs_1C12_08[]="8. PDO mapped";
static const VISIBLE_STRING  sOdDs_1C12_ff[]="Sync Manager RXPDO Assignment";
static const VISIBLE_STRING  sOdDs_1C13_00[]="Number of assigned TXPDO";
static const VISIBLE_STRING  sOdDs_1C13_ff[]="Sync Manager TXPDO Assignment";
static const VISIBLE_STRING  sOdDs_603f_ff[]="Error code";
static const VISIBLE_STRING  sOdDs_6040_ff[]="Control word";
static const VISIBLE_STRING  sOdDs_6041_ff[]="Status word";
static const VISIBLE_STRING  sOdDs_605a_ff[]="Quick stop option code";
static const VISIBLE_STRING  sOdDs_605b_ff[]="Shutdown option code";
static const VISIBLE_STRING  sOdDs_605c_ff[]="Disable operation option code";
static const VISIBLE_STRING  sOdDs_605d_ff[]="Halt option code";
static const VISIBLE_STRING  sOdDs_605e_ff[]="Fault reaction option code";
static const VISIBLE_STRING  sOdDs_6060_ff[]="Mode of operation";
static const VISIBLE_STRING  sOdDs_6061_ff[]="Mode of operation display";
static const VISIBLE_STRING  sOdDs_6064_ff[]="Position actual value";
static const VISIBLE_STRING  sOdDs_6065_ff[]="Following error window";
static const VISIBLE_STRING  sOdDs_6066_ff[]="Following error time-out";
static const VISIBLE_STRING  sOdDs_6067_ff[]="Position window";
static const VISIBLE_STRING  sOdDs_6068_ff[]="Position time-out";
static const VISIBLE_STRING  sOdDs_6069_ff[]="Velocity sensor actual value";
static const VISIBLE_STRING  sOdDs_606b_ff[]="Velocity demand value";
static const VISIBLE_STRING  sOdDs_606c_ff[]="Velocity actual value";
static const VISIBLE_STRING  sOdDs_606d_ff[]="Velocity window";
static const VISIBLE_STRING  sOdDs_606e_ff[]="Velocity window time-out";
static const VISIBLE_STRING  sOdDs_606f_ff[]="Velocity threshold";
static const VISIBLE_STRING  sOdDs_6070_ff[]="Velocity threshold time-out";
static const VISIBLE_STRING  sOdDs_6071_ff[]="Target torque";
static const VISIBLE_STRING  sOdDs_6076_ff[]="Motor rated torque";
static const VISIBLE_STRING  sOdDs_6077_ff[]="Torque actual value";
static const VISIBLE_STRING  sOdDs_607a_ff[]="Target position";
static const VISIBLE_STRING  sOdDs_607c_ff[]="Home Offset";
static const VISIBLE_STRING  sOdDs_6081_ff[]="Profile velocity";
static const VISIBLE_STRING  sOdDs_6082_ff[]="End velocity";
static const VISIBLE_STRING  sOdDs_6083_ff[]="Profile acceleration";
static const VISIBLE_STRING  sOdDs_6084_ff[]="Profile deceleration";
static const VISIBLE_STRING  sOdDs_6085_ff[]="Quickstop deceleration";
static const VISIBLE_STRING  sOdDs_6098_ff[]="Homing Method";
static const VISIBLE_STRING  sOdDs_6099_01[]="Speed during search for switch";
static const VISIBLE_STRING  sOdDs_6099_02[]="Speed during search for zero";
static const VISIBLE_STRING  sOdDs_6099_ff[]="Homing Speeds";
static const VISIBLE_STRING  sOdDs_609A_ff[]="Homing Acceleration";
static const VISIBLE_STRING  sOdDs_60c1_01[]="X1";
static const VISIBLE_STRING  sOdDs_60c1_ff[]="Interpolation data record";
static const VISIBLE_STRING  sOdDs_60c2_01[]="Time units";
static const VISIBLE_STRING  sOdDs_60c2_02[]="Time index";
static const VISIBLE_STRING  sOdDs_60c2_ff[]="Interpolation time period";
static const VISIBLE_STRING  sOdDs_60f4_ff[]="Following error actual value";
static const VISIBLE_STRING  sOdDs_60ff_ff[]="Target velocity";
static const VISIBLE_STRING  sOdDs_6502_ff[]="Supported drive modes";
static const VISIBLE_STRING  sOdDs_3000_ff[]="Positioner: Target Position";
static const VISIBLE_STRING  sOdDs_3001_ff[]="Homing: Positive switch source (parDevCtrl.HMPositiveSwitchSrc)";
static const VISIBLE_STRING  sOdDs_3002_ff[]="Homing: Negative switch source (parDevCtrl.HMNegativeSwitchSrc)";
static const VISIBLE_STRING  sOdDs_3003_ff[]="Homing: Home switch source (parDevCtrl.HMHomeSwitchSrc)";
static const VISIBLE_STRING  sOdDs_3008_ff[]="Homing: Positive switch fieldbus input (wksHMSoftPositiveSwitch)";
static const VISIBLE_STRING  sOdDs_3009_ff[]="Homing: Negative switch fieldbus input (wksHMSoftNegativeSwitch)";
static const VISIBLE_STRING  sOdDs_300A_ff[]="Homing: Home switch fieldbus input (wksHMSoftHomeSwitch)";
static const VISIBLE_STRING  sOdDs_300B_ff[]="Homing: Actual home offset";
static const VISIBLE_STRING  sOdDs_3100_ff[]="Torque Loop: Current Loop Id feedback [1/10 mArms] (varILoopIdFb)";
static const VISIBLE_STRING  sOdDs_3101_ff[]="Torque Loop: Current Loop Iq feedback [1/10 mArms] (varILoopIqFb)";
static const VISIBLE_STRING  sOdDs_3102_ff[]="Torque Loop: Bridge 1 Iu feedback [1/10 mApk] (varILoopIuFb)";
static const VISIBLE_STRING  sOdDs_3103_ff[]="Torque Loop: Bridge 1 Iv feedback [1/10 mApk] (varILoopIvFb)";
static const VISIBLE_STRING  sOdDs_3104_ff[]="Brake Unit: Power Stage DC-bus [1/10 V] (varPStageVdcBus)";
static const VISIBLE_STRING  sOdDs_3105_ff[]="Torque Loop: Current Loop Actual Id reference [1/10 mArms] (varILoopIdRef)";
static const VISIBLE_STRING  sOdDs_3106_ff[]="Torque Loop: Current Loop Actual Iq reference [1/10 mArms] (varILoopIqRef)";
static const VISIBLE_STRING  sOdDs_3107_ff[]="Torque Loop: Output Vu [1/10 V] (varILoopVuOut)";
static const VISIBLE_STRING  sOdDs_3108_ff[]="Torque Loop: Output Vv [1/10 V] (varILoopVvOut)";
static const VISIBLE_STRING  sOdDs_3109_ff[]="Torque Loop: Modulator Ki (auto) (varILoopAutoILoopKi)";
static const VISIBLE_STRING  sOdDs_310A_ff[]="Torque Loop: Modulator Kp (auto) (varILoopAutoILoopKp)";
static const VISIBLE_STRING  sOdDs_310B_ff[]="Torque Loop: Actual Direct Current Limit Min [1/10 mArms] (varILoopIdLimMin)";
static const VISIBLE_STRING  sOdDs_310C_ff[]="Torque Loop: Actual Direct Current Limit Max [1/10 mArms] (varILoopIdLimMax)";
static const VISIBLE_STRING  sOdDs_310D_ff[]="Torque Loop: Actual Quadrature Current Limit Min [1/10 mArms] (varILoopIqLimMin)";
static const VISIBLE_STRING  sOdDs_310E_ff[]="Torque Loop: Actual Quadrature Current Limit Max [1/10 mArms] (varILoopIqLimMax)";
static const VISIBLE_STRING  sOdDs_310F_ff[]="Torque Loop: Bridge 1 Iw feedback [1/10 mApk] (varILoopIwFb)";
static const VISIBLE_STRING  sOdDs_3110_ff[]="Torque Loop: Dynamic Set Direct Current Limit Min [1/10 mArms] (wksDynamicIdLimMin)";
static const VISIBLE_STRING  sOdDs_3111_ff[]="Torque Loop: Dynamic Set Direct Current Limit Max [1/10 mArms] (wksDynamicIdLimMax)";
static const VISIBLE_STRING  sOdDs_3112_ff[]="Torque Loop: Dynamic Set Quadrature Current Limit Min [1/10 mArms] (wksDynamicIqLimMin)";
static const VISIBLE_STRING  sOdDs_3113_ff[]="Torque Loop: Dynamic Set Quadrature Current Limit Max [1/10 mArms] (wksDynamicIqLimMax)";
static const VISIBLE_STRING  sOdDs_3114_ff[]="Torque Loop: Actual Drive Over Current Threshold Detection [1/10 mApk] (varPStageOverCurrentThreshold)";
static const VISIBLE_STRING  sOdDs_3115_ff[]="Brake Unit: Actual Drive Over Voltage Threshold Detection [1/10 V] (varPStageOverVoltageThreshold)";
static const VISIBLE_STRING  sOdDs_3116_ff[]="Brake Unit: Actual Brake Low Voltage Threshold [1/10 V] (varPStageVBrakeLow)";
static const VISIBLE_STRING  sOdDs_3117_ff[]="Brake Unit: Actual Brake High Voltage Threshold [1/10 V] (varPStageVBrakeHigh)";
static const VISIBLE_STRING  sOdDs_3118_ff[]="Torque Loop: Actual PWM Switching Frequency (varPStageSwitchingFrequency)";
static const VISIBLE_STRING  sOdDs_3119_ff[]="Torque Loop: Disable runtime motor phases connection check (parPStage_DisableMotorPhasesCheck)";
static const VISIBLE_STRING  sOdDs_311a_ff[]="Torque Loop: Disable power stage self-test at startup (parPStage_DisablePOST)";
static const VISIBLE_STRING  sOdDs_311b_ff[]="Torque Loop: Force successfully completion of power stage self-test at startup (parPStage_ForcePOST)";
static const VISIBLE_STRING  sOdDs_311c_ff[]="Brake Unit: Disable power stage AC monitoring (parPStage_DisableACMgm)";
static const VISIBLE_STRING  sOdDs_311d_ff[]="Field Weakening: Enable field weakening (parPStage_EnableFieldWeakening)";
static const VISIBLE_STRING  sOdDs_311e_ff[]="Brake Unit: Minimum AC phases distortion allowed [%] (parPStage.VacMinDistortionAllowed)";
static const VISIBLE_STRING  sOdDs_311f_ff[]="Torque Loop: Actual Current Loop DSP workload [%] (varPStageDSPLoad)";
static const VISIBLE_STRING  sOdDs_3120_ff[]="Torque Loop: Modulator Ki (parILoop.ILoopKi)";
static const VISIBLE_STRING  sOdDs_3121_ff[]="Torque Loop: Modulator Kp (parILoop.ILoopKp)";
static const VISIBLE_STRING  sOdDs_3122_ff[]="Torque Loop: Direct Current Limit Min [1/10 mArms] (parILoop.IdLimit.Min)";
static const VISIBLE_STRING  sOdDs_3123_ff[]="Torque Loop: Direct Current Limit Max [1/10 mArms] (parILoop.IdLimit.Max)";
static const VISIBLE_STRING  sOdDs_3124_ff[]="Torque Loop: Quadrature Current Limit Min [1/10 mArms] (parILoop.IqLimit.Min)";
static const VISIBLE_STRING  sOdDs_3125_ff[]="Torque Loop: Quadrature Current Limit Max [1/10 mArms] (parILoop.IqLimit.Max)";
static const VISIBLE_STRING  sOdDs_3126_ff[]="Torque Loop: User Id reference [1/10 mArms] (wksIdRef)";
static const VISIBLE_STRING  sOdDs_3127_ff[]="Torque Loop: User Iq reference [1/10 mArms] (wksIqRef)";
static const VISIBLE_STRING  sOdDs_3128_ff[]="Torque Loop: Output Vd [1/10 V] (varILoopVdOut)";
static const VISIBLE_STRING  sOdDs_3129_ff[]="Torque Loop: Output Vq [1/10 V] (varILoopVqOut)";
#ifdef _HW_DC
static const VISIBLE_STRING  sOdDs_312a_ff[]="Torque Loop: Enable 2nd parallel bridge (parPStage_EnableParallelBridge2)";
static const VISIBLE_STRING  sOdDs_312b_ff[]="Torque Loop: Enable 3nd parallel bridge (parPStage_EnableParallelBridge3)";
#endif
static const VISIBLE_STRING  sOdDs_312c_ff[]="Field Weakening: Use filtered Iq reference (parPStage_EnableFldWeakFiltIQ)";
static const VISIBLE_STRING  sOdDs_3130_ff[]="Torque Loop: Current Loop Actual Filtered Iq reference [1/10 mArms] (varILoopIqFiltRef)";
static const VISIBLE_STRING  sOdDs_3133_ff[]="Brake Unit: Power Stage input phases monitoring status (varPStageVacStatus)";
static const VISIBLE_STRING  sOdDs_3134_ff[]="Brake Unit: Power Stage Vr-Vs AC phases [1/10 V] (varPStageVacRS)";
static const VISIBLE_STRING  sOdDs_3135_ff[]="Brake Unit: Power Stage Vs-Vt AC phases [1/10 V] (varPStageVacST)";
static const VISIBLE_STRING  sOdDs_3136_ff[]="Brake Unit: Power Stage Vt-Vr AC phases [1/10 V] (varPStageVacTR)";
static const VISIBLE_STRING  sOdDs_3140_ff[]="Brake Unit: Brake Low Voltage Threshold [1/10 V] (parPStage.VBrakeLow)";
static const VISIBLE_STRING  sOdDs_3141_ff[]="Brake Unit: Brake High Voltage Threshold [1/10 V] (parPStage.VBrakeHigh)";
static const VISIBLE_STRING  sOdDs_3142_ff[]="Brake Unit: Drive Under Voltage Threshold Detection [1/10 V] (parPStage.UnderVoltageThreshold)";
static const VISIBLE_STRING  sOdDs_3143_ff[]="Brake Unit: Drive Over Voltage Threshold Detection [1/10 V] (parPStage.OverVoltageThreshold)";
static const VISIBLE_STRING  sOdDs_3144_ff[]="Torque Loop: Drive Over Current Threshold Detection [1/10 mArms] (parPStage.OverCurrentThreshold)";
static const VISIBLE_STRING  sOdDs_3146_ff[]="Torque Loop: Bridge layout assignment (parPStage.BridgeLayout)";
static const VISIBLE_STRING  sOdDs_3150_ff[]="Iq Filter 0: Filter type selector (parILoop.IqFiltPars[0].Type)";
static const VISIBLE_STRING  sOdDs_3151_ff[]="Iq Filter 0: Frequency main, filter dependant [rad/s] (parILoop.IqFiltPars[0].MainFrequency)";
static const VISIBLE_STRING  sOdDs_3152_ff[]="Iq Filter 0: Damping factor main, filter dependant (parILoop.IqFiltPars[0].MainDampFactor)";
static const VISIBLE_STRING  sOdDs_3153_ff[]="Iq Filter 0: Frequency secondary, filter dependant [rad/s] (parILoop.IqFiltPars[0].SecFrequency)";
static const VISIBLE_STRING  sOdDs_3154_ff[]="Iq Filter 0: Damping factor secondary, filter dependant (parILoop.IqFiltPars[0].SecDampFactor)";
static const VISIBLE_STRING  sOdDs_3158_ff[]="Iq Filter 1: Filter type selector (parILoop.IqFiltPars[1].Type)";
static const VISIBLE_STRING  sOdDs_3159_ff[]="Iq Filter 1: Frequency main, filter dependant [rad/s] (parILoop.IqFiltPars[1].MainFrequency)";
static const VISIBLE_STRING  sOdDs_315A_ff[]="Iq Filter 1: Damping factor main, filter dependant (parILoop.IqFiltPars[1].MainDampFactor)";
static const VISIBLE_STRING  sOdDs_315B_ff[]="Iq Filter 1: Frequency secondary, filter dependant [rad/s] (parILoop.IqFiltPars[1].SecFrequency)";
static const VISIBLE_STRING  sOdDs_315C_ff[]="Iq Filter 1: Damping factor secondary, filter dependant (parILoop.IqFiltPars[1].SecDampFactor)";
static const VISIBLE_STRING  sOdDs_3160_ff[]="Iq Filter 2: Filter type selector (parILoop.IqFiltPars[2].Type)";
static const VISIBLE_STRING  sOdDs_3161_ff[]="Iq Filter 2: Frequency main, filter dependant [rad/s] (parILoop.IqFiltPars[2].MainFrequency)";
static const VISIBLE_STRING  sOdDs_3162_ff[]="Iq Filter 2: Damping factor main, filter dependant (parILoop.IqFiltPars[2].MainDampFactor)";
static const VISIBLE_STRING  sOdDs_3163_ff[]="Iq Filter 2: Frequency secondary, filter dependant [rad/s] (parILoop.IqFiltPars[2].SecFrequency)";
static const VISIBLE_STRING  sOdDs_3164_ff[]="Iq Filter 2: Damping factor secondary, filter dependant (parILoop.IqFiltPars[2].SecDampFactor)";
static const VISIBLE_STRING  sOdDs_3168_ff[]="Iq Filter 3: Filter type selector (parILoop.IqFiltPars[3].Type)";
static const VISIBLE_STRING  sOdDs_3169_ff[]="Iq Filter 3: Frequency main, filter dependant [rad/s] (parILoop.IqFiltPars[3].MainFrequency)";
static const VISIBLE_STRING  sOdDs_316A_ff[]="Iq Filter 3: Damping factor main, filter dependant (parILoop.IqFiltPars[3].MainDampFactor)";
static const VISIBLE_STRING  sOdDs_316B_ff[]="Iq Filter 3: Frequency secondary, filter dependant [rad/s] (parILoop.IqFiltPars[3].SecFrequency)";
static const VISIBLE_STRING  sOdDs_316C_ff[]="Iq Filter 3: Damping factor secondary, filter dependant (parILoop.IqFiltPars[3].SecDampFactor)";
#ifdef _HW_DC
static const VISIBLE_STRING  sOdDs_3188_ff[]="Torque Loop: Bridge 2 Iu feedback [1/10 mApk] (varILoopBr2IuFb)";
static const VISIBLE_STRING  sOdDs_3189_ff[]="Torque Loop: Bridge 2 Iv feedback [1/10 mApk] (varILoopBr2IvFb)";
static const VISIBLE_STRING  sOdDs_318A_ff[]="Torque Loop: Bridge 2 Iw feedback [1/10 mApk] (varILoopBr2IwFb)";
static const VISIBLE_STRING  sOdDs_318B_ff[]="Torque Loop: Bridge 3 Iu feedback [1/10 mApk] (varILoopBr3IuFb)";
static const VISIBLE_STRING  sOdDs_318C_ff[]="Torque Loop: Bridge 3 Iv feedback [1/10 mApk] (varILoopBr3IvFb)";
static const VISIBLE_STRING  sOdDs_318D_ff[]="Torque Loop: Bridge 3 Iw feedback [1/10 mApk] (varILoopBr3IwFb)";
#endif
static const VISIBLE_STRING  sOdDs_31a0_ff[]="Field Weakening: Maximum speed requirement (parILFWeak.MaxSpeed)";
static const VISIBLE_STRING  sOdDs_31a1_ff[]="Field Weakening: DC-bus security margin [1/1000] (parILFWeak.VoltageMargin)";
static const VISIBLE_STRING  sOdDs_31a2_ff[]="Field Weakening: Factors computation fixed DC-bus value [1/10 V] (parILFWeak.FixVdcBus)";
static const VISIBLE_STRING  sOdDs_31a8_ff[]="Field Weakening: Field weakening enabled (varILFWeakActive)";
static const VISIBLE_STRING  sOdDs_31a9_ff[]="Field Weakening: Maximum requested speed reachable (varILFWeakMaxSpeedReachable)";
static const VISIBLE_STRING  sOdDs_31aa_ff[]="Field Weakening: Weakening speed starting point (varILFWeakKneeSpeed)";
static const VISIBLE_STRING  sOdDs_31ab_ff[]="Field Weakening: Computed short circuit current [1/10 mArms] (varILFWeakShortCurrent)";
static const VISIBLE_STRING  sOdDs_3200_ff[]="Thermal Model: Power Module NTC Temperature [1/10 ℃]";
static const VISIBLE_STRING  sOdDs_3201_ff[]="Thermal Model: Power Module Maximum Junction Temperature [1/10 ℃]";
static const VISIBLE_STRING  sOdDs_3202_ff[]="Thermal Model: Motor Temperature [1/10 ℃]";
static const VISIBLE_STRING  sOdDs_3203_ff[]="Thermal Model: Thermal Current Limit Iq Max [1/10 mArms] (varThModelIqLimMax)";
static const VISIBLE_STRING  sOdDs_3204_ff[]="Thermal Model: Thermal Current Limit Iq Min [1/10 mArms] (varThModelIqLimMin)";
static const VISIBLE_STRING  sOdDs_3205_ff[]="Thermal Model: Thermal Current Limit Id Max [1/10 mArms] (varThModelIdLimMax)";
static const VISIBLE_STRING  sOdDs_3206_ff[]="Thermal Model: Thermal Current Limit Id Min [1/10 mArms] (varThModelIdLimMin)";
static const VISIBLE_STRING  sOdDs_3207_ff[]="Brake Unit: Power dissipated by Brake Resistor [1/10 W] (varThModelRBrakePower)";
static const VISIBLE_STRING  sOdDs_3208_ff[]="Thermal Model: HeatSink Temperature measured (model) [1/10 ℃] (varThModelTHeatSink)";
static const VISIBLE_STRING  sOdDs_3209_ff[]="Thermal Model: Motor temperature [℃] (varThModelTMotor)";
static const VISIBLE_STRING  sOdDs_320A_ff[]="Brake Unit: Brake Resistor Max Energy [J] (varThModelRBrakeEnergy)";
#ifdef _HW_DC
static const VISIBLE_STRING  sOdDs_320B_ff[]="Thermal Model: Motor PTC sensor [℃] (varThModelTMotorPTC)";
static const VISIBLE_STRING  sOdDs_320C_ff[]="Thermal Model: Motor KTY sensor [℃] (varThModelTMotorKTY)";
static const VISIBLE_STRING  sOdDs_320D_ff[]="Thermal Model: Bridge 1 Temperature [℃] (varThModelBr1Temp)";
static const VISIBLE_STRING  sOdDs_320E_ff[]="Thermal Model: Bridge 2 Temperature [℃] (varThModelBr2Temp)";
static const VISIBLE_STRING  sOdDs_320F_ff[]="Thermal Model: Bridge 3 Temperature [℃] (varThModelBr3Temp)";
#endif
#ifdef _HW_CT
static const VISIBLE_STRING  sOdDs_3210_ff[]="Thermal Model: Controlboard Temperature [1/10℃] (varThModelCtrlbrdTemp)";
#endif
static const VISIBLE_STRING  sOdDs_3220_ff[]="Brake Unit: Brake Resistor Value [1/10 Ohm] (parThermalModel.BrakeResistorValue)";
static const VISIBLE_STRING  sOdDs_3221_ff[]="Brake Unit: Brake Resistor Max Power [1/10 W] (parThermalModel.BrakeResistorPower)";
static const VISIBLE_STRING  sOdDs_3222_ff[]="Thermal Model: Start cooling at [1/10 ℃] (parThermalModel.CoolingTempOn)";
static const VISIBLE_STRING  sOdDs_3223_ff[]="Thermal Model: Stop cooling at [1/10 ℃] (parThermalModel.CoolingTempOff)";
static const VISIBLE_STRING  sOdDs_3224_ff[]="Thermal Model: Motor Temperature limit [1/10 ℃] (parThermalModel.MotorOverTemp)";
static const VISIBLE_STRING  sOdDs_3225_ff[]="Brake Unit: Brake Resistor Max Energy [J] (parThermalModel.BrakeResistorEnergy)";
static const VISIBLE_STRING  sOdDs_3226_ff[]="Thermal Model: Disable fault reaction on current derating (parThermalModel.Flags.DisFltReactOnLimit)";
static const VISIBLE_STRING  sOdDs_3227_ff[]="Brake Unit: No brake drive disable on overpower (parThermalModel.Flags.NoBrakeDisableOnFlt)";
static const VISIBLE_STRING  sOdDs_3228_ff[]="Brake Unit: No fatal fault on brake overpower (parThermalModel.Flags.NoFatalOnBrakeFlt)";
#ifdef _HW_DC
static const VISIBLE_STRING  sOdDs_3250_ff[]="Thermal Model: PTC sensor linearization (parThermailModel.PTCCoeff)";
#endif
static const VISIBLE_STRING  sOdDs_3251_ff[]="Thermal Model: KTY sensor linearization (parThermailModel.KTYCoeff)";
static const VISIBLE_STRING  sOdDs_3250_01[]="K0";
static const VISIBLE_STRING  sOdDs_3250_02[]="K1";
static const VISIBLE_STRING  sOdDs_3250_03[]="K2";
static const VISIBLE_STRING  sOdDs_3250_04[]="K3";

static const VISIBLE_STRING  sOdDs_3300_ff[]="Space Speed Loop: Use different Speed Kp (parSSCntrLp.Flags.UseDifferentKp)";
static const VISIBLE_STRING  sOdDs_330A_ff[]="Space Speed Loop: Maximum Torque limit [1/10 mArms] (parSSCntrLp.ILimit.Max)";
static const VISIBLE_STRING  sOdDs_330B_ff[]="Space Speed Loop: Minimum Torque limit [1/10 mArms] (parSSCntrLp.ILimit.Min)";
static const VISIBLE_STRING  sOdDs_330C_ff[]="Space Speed Loop: Gains";
static const VISIBLE_STRING  sOdDs_330C_01[]="Integral gain [Hz] (parSSCntrLp.fKi)";
static const VISIBLE_STRING  sOdDs_330C_02[]="Position gain [Arms/rad] (parSSCntrLp.fPosKp)";
static const VISIBLE_STRING  sOdDs_330C_03[]="Speed reference gain [Arms/(rad/s)] (parSSCntrLp.fSpdKpRef)";
static const VISIBLE_STRING  sOdDs_330C_04[]="Speed feedback gain[Arms/(rad/s)] (parSSCntrLp.fSpdKpFbk)";
static const VISIBLE_STRING  sOdDs_330C_05[]="Acceleration reference gain [Arms/(rad/s^2)] (parSSCntrLp.fAccKpRef)";
static const VISIBLE_STRING  sOdDs_330C_06[]="Acceleration feedback gain [Arms/(rad/s^2)] (parSSCntrLp.fAccKpFbk)";
static const VISIBLE_STRING  sOdDs_330D_ff[]="Space Speed Loop: Enable acceleration reference filter (parSSCntrLp.Flags.AccRefFiltEnable)";
static const VISIBLE_STRING  sOdDs_330E_ff[]="Space Speed Loop: K filter for acceleration reference (0=no filter) (parSSCntrLp.AccRefKFilt)";
static const VISIBLE_STRING  sOdDs_3320_ff[]="Space Speed Loop: Current reference request monitor [1/10 Arms] (varSSCntrLpIqRef)";
static const VISIBLE_STRING  sOdDs_3321_ff[]="Space Speed Loop: Auto calc err limit Min (varSSCntrLpAutoErrMin)";
static const VISIBLE_STRING  sOdDs_3322_ff[]="Space Speed Loop: Auto calc err limit Max (varSSCntrLpAutoErrMax)";
static const VISIBLE_STRING  sOdDs_3323_ff[]="Space Speed Loop: Filtered Reference Acceleration [rad/s^2] (varSSCntrLpFiltAccRef)";
static const VISIBLE_STRING  sOdDs_3340_ff[]="Positioner: Max Position Error allowed (parPositioner.PositionErrorMax)";
static const VISIBLE_STRING  sOdDs_3341_ff[]="Positioner: Threshold below which motor is considered standstill (parPositioner.ZeroSpeedThreshold)";
static const VISIBLE_STRING  sOdDs_3342_ff[]="Positioner: Time-out for motor blocked alarm [msec] (parPositioner.MotorBlockedTimeout)";
static const VISIBLE_STRING  sOdDs_3343_ff[]="Positioner: Enable S-Ramp (parPositioner.Flags.SRampEnable)";
static const VISIBLE_STRING  sOdDs_3344_ff[]="Positioner: S-Ramp filter K1 (parPositioner.SRampK1)";
static const VISIBLE_STRING  sOdDs_3345_ff[]="Positioner: S-Ramp filter K2 (parPositioner.SRampK2)";
static const VISIBLE_STRING  sOdDs_3346_ff[]="Positioner: S-Ramp position error limit (parPositioner.SRampErrorMax)";
static const VISIBLE_STRING  sOdDs_3352_ff[]="Positioner: Demand Acceleration (varPosRGDemandAccel)";
static const VISIBLE_STRING  sOdDs_3800_ff[]="Encoder: Encoder Supply Voltage (parEncMgr.PowerVoltage)";
static const VISIBLE_STRING  sOdDs_3801_ff[]="Encoder: Main Absolute Encoder Selection (parEncMgr.MainAbsSelection)";
static const VISIBLE_STRING  sOdDs_3802_ff[]="Encoder: Auxiliary Encoder Type Selection (parEncMgr.AuxSelection)";
static const VISIBLE_STRING  sOdDs_3803_ff[]="Encoder: Position feedback source (parEncMgr.Flags.CntrlLoopPosMain)";
static const VISIBLE_STRING  sOdDs_3804_ff[]="Encoder: Speed feedback source (parEncMgr.Flags.CntrlLoopSpeedMain)";
static const VISIBLE_STRING  sOdDs_3805_ff[]="Encoder: Acceleration feedback source (parEncMgr.Flags.CntrlLoopAccelMain)";
static const VISIBLE_STRING  sOdDs_3806_ff[]="Encoder: Electrical angle source (parEncMgr.Flags.CntrlLoopElecAngleMain)";
static const VISIBLE_STRING  sOdDs_3807_ff[]="Encoder: Disable power if relative fail, by default switch to absolute track (parEncMgr.Flags.DisableIfRelFail)";
static const VISIBLE_STRING  sOdDs_3808_ff[]="Encoder: Delay after applying encoder power supply [msec] (parEncMgr.PowerStartupDelay)";
static const VISIBLE_STRING  sOdDs_3809_ff[]="Encoder: Main Relative Encoder Type Selection (parEncMgr.MainRelSelection)";
static const VISIBLE_STRING  sOdDs_380A_ff[]="Encoder: Max angle difference between Abs and Rel tracks for redundancy control (parEncMgr.MaxAbsRelDiff)";
static const VISIBLE_STRING  sOdDs_380B_ff[]="Encoder: Disable reading of electronic plate at startup (parEncMgr.Flags.DisableEPlate)";
static const VISIBLE_STRING  sOdDs_380C_ff[]="Encoder: Enable restore of electronic plate at startup (parEncMgr.Flags.RestoreEPlate)";
static const VISIBLE_STRING  sOdDs_380D_ff[]="Encoder: Disable absolute track processing after valid position (parEncMgr.Flags.DisableAbsAfterValid)";
static const VISIBLE_STRING  sOdDs_380E_ff[]="Encoder: Maximum allowed motor speed (parEncMgr.MaxSpeed)";
static const VISIBLE_STRING  sOdDs_380F_ff[]="Encoder: Electrical Angle feed forward time [usec] (parEncMgr.ElecAngleFeedForwardTime)";
static const VISIBLE_STRING  sOdDs_3810_ff[]="Endat Main: Clock frequency selector [kHz] (parEncMEndat.ClockFreq)";
static const VISIBLE_STRING  sOdDs_3811_ff[]="Endat Main: At reset, if multiturn position is equal or above this number, then abspos became negative (parEncMEndat.MTurnStartPos)";
static const VISIBLE_STRING  sOdDs_3818_ff[]="Endat Main: Overall CRC error counter (varEncMEndatCrcErrors)";
static const VISIBLE_STRING  sOdDs_3819_ff[]="Endat Main: Data Propagation Delay [nsec] (varEncMEndatPropDelay)";
static const VISIBLE_STRING  sOdDs_381a_ff[]="Endat Main: Maximum clock frequency allowed [kHz] (varEncMEndatMaxClockFreq)";
static const VISIBLE_STRING  sOdDs_381b_ff[]="Endat Main: Endat protocol version (varEncMEndatProtocolVer)";
static const VISIBLE_STRING  sOdDs_381c_ff[]="Endat Main: No. of bits of revolution resolution (varEncMEndatStepPerRevBits)";
static const VISIBLE_STRING  sOdDs_381d_ff[]="Endat Main: No. of bits for revolution counting (varEncMEndatRevNumBits)";
static const VISIBLE_STRING  sOdDs_3820_ff[]="Incremental Main: Encoder line counts per turn (parEncMInc.LineCounts)";
static const VISIBLE_STRING  sOdDs_3821_ff[]="Incremental Main: Enable incremental analog tracks interpolation (parEncMInc.Flags.EnableAnalogInterp)";
static const VISIBLE_STRING  sOdDs_3822_ff[]="Incremental Main: Disable Index Error (parEncMInc.Flags.DisableIndexError)";
static const VISIBLE_STRING  sOdDs_3823_ff[]="Incremental Main: Disable Analog Tracks Levels Error (parEncMInc.Flags.DisableAnalogError)";
static const VISIBLE_STRING  sOdDs_3824_ff[]="Incremental Main: Analog Tracks Level Alarm Threshold (parEncMInc.AnalogAlarmThreshold)";
static const VISIBLE_STRING  sOdDs_3825_ff[]="Incremental Main: Index Error tolerance (parEncMInc.IndexErrorTolerance)";
static const VISIBLE_STRING  sOdDs_3826_ff[]="Incremental Main: Enable Index Track (parEncMInc.Flags.EnableIndexTrack)";
static const VISIBLE_STRING  sOdDs_3830_ff[]="Incremental Main: SIN channel from ADC (varEncIncMainChannelSin)";
static const VISIBLE_STRING  sOdDs_3831_ff[]="Incremental Main: COS channel from ADC (varEncIncMainChannelCos)";
static const VISIBLE_STRING  sOdDs_3832_ff[]="Incremental Main: Sin^2+Cos^2 Reading (varEncIncMainChannelLevels)";
static const VISIBLE_STRING  sOdDs_3827_ff[]="Incremental Main: Swap input tracks A and B (parEncMInc.Flags.SwapTracks)";
static const VISIBLE_STRING  sOdDs_3828_ff[]="Incremental Main: Enable step pulse (track A) and direction (track B) mode (parEncMInc.Flags.EnableStepDir)";
static const VISIBLE_STRING  sOdDs_3829_ff[]="Incremental Main: Enable up pulse (track A) and down pulse (track B) mode (parEncMInc.Flags.EnableUpDown)";
static const VISIBLE_STRING  sOdDs_382b_ff[]="Incremental Main: Maximum encoder frequency [kHz] (parEncMInc.MaxFrequency)";
static const VISIBLE_STRING  sOdDs_382c_ff[]="Incremental Main: Disable offset from index track for electrical angle adjustment (parEncMInc.Flags.DisableIndexForElecAngle)";
static const VISIBLE_STRING  sOdDs_382d_ff[]="Incremental Main: Disable offset from index track for mechanical position adjustment (parEncMInc.Flags.DisableIndexForMechPos)";
static const VISIBLE_STRING  sOdDs_382e_ff[]="Incremental Main: Disable index synchronization with track A and B (parEncMInc.Flags.DisableIndexTrackSync)";
static const VISIBLE_STRING  sOdDs_382f_ff[]="Incremental Main: Disable period meter digital interpolation (parEncMInc.Flags.DisableDigitalInterp)";
static const VISIBLE_STRING  sOdDs_3833_ff[]="Incremental Main: Disable filter watchdog error on digital tracks (parEncMInc.Flags.DisableFilterWDTError)";
static const VISIBLE_STRING  sOdDs_3834_ff[]="Incremental Main: Enable capturing of all indexes, unregarding index tolerance (parEncMInc.Flags.CaptureAllIndexes)";
#ifdef _HW_DC
static const VISIBLE_STRING  sOdDs_3835_ff[]="Encoder: Aux Encoder Supply Voltage (parEncMgr.AuxPowerVoltage)";
#endif
static const VISIBLE_STRING  sOdDs_3836_ff[]="Encoder: Simulation Encoder Type Selection (parEncMgr.SimSelection)";
static const VISIBLE_STRING  sOdDs_3837_ff[]="Incremental Main: Frequency above which interpolation switch from analog to digital [kHz] (parEncMInc.InterpSwitchFrequency)";
static const VISIBLE_STRING  sOdDs_3840_ff[]="Absolute Analogue: Encoder number of poles (parEncAn.PoleCounts)";
static const VISIBLE_STRING  sOdDs_3841_ff[]="Absolute Analogue: If enabled, Angle = 360° - arctan(Sin/Cos), otherwise  Angle = arctan(Sin/Cos) (parEncAn.Flags.ReverseSignal)";
static const VISIBLE_STRING  sOdDs_3842_ff[]="Absolute Analogue: High for SinCos, low for resolver (parEncAn.Flags.AnalogGain)";
static const VISIBLE_STRING  sOdDs_3843_ff[]="Absolute Analogue: SinCos Level Alarm threshold (parEncAn.AnalogAlarmThreshold)";
static const VISIBLE_STRING  sOdDs_3844_ff[]="Absolute Analogue: Adjust Sin channel gain [1/1000] (parEncAn.GainSin)";
static const VISIBLE_STRING  sOdDs_3845_ff[]="Absolute Analogue: Adjust Cos channel gain [1/1000] (parEncAn.GainCos)";
static const VISIBLE_STRING  sOdDs_3846_ff[]="Absolute Analogue: Enable runtime automatic calibration of the analog channels (parEncAn.Flags.AutoCalibration)";
static const VISIBLE_STRING  sOdDs_3848_ff[]="Absolute Analogue: SIN channel from ADC (varEncAnChannelSin)";
static const VISIBLE_STRING  sOdDs_3849_ff[]="Absolute Analogue: COS channel from ADC (varEncAnChannelCos)";
static const VISIBLE_STRING  sOdDs_384A_ff[]="Absolute Analogue: Sin^2+Cos^2 Reading (varEncAnChannelLevels)";
static const VISIBLE_STRING  sOdDs_384E_ff[]="Absolute Analogue: Adjust Sin channel offset [1/10000] (parEncAn.OffsetSin)";
static const VISIBLE_STRING  sOdDs_384F_ff[]="Absolute Analogue: Adjust Cos channel offset [1/10000] (parEncAn.OffsetCos)";
static const VISIBLE_STRING  sOdDs_38E0_ff[]="Absolute Analogue: Adjust excitation frequency generator offset [1/10 usec] (parEncAn.FrequencyOffset)";
static const VISIBLE_STRING  sOdDs_3850_ff[]="Sensorless: Open Loop only (parEncBEmf.Flags.OpenLoopOnly)";
static const VISIBLE_STRING  sOdDs_3851_ff[]="Sensorless: Use dynamic Iq limit (parEncBEmf.Flags.DynamicIqLimit)";
static const VISIBLE_STRING  sOdDs_3852_ff[]="Sensorless: Use dynamic Id limit (parEncBEmf.Flags.DynamicIdLimit)";
static const VISIBLE_STRING  sOdDs_3853_ff[]="Sensorless: Back on the fly: % of nominal motor speed [1/100 %] (parEncBEmf.BackOnTheFlySpd)";
static const VISIBLE_STRING  sOdDs_3854_ff[]="Sensorless: Sensorless speed: % of nominal motor speed [1/100 %] (parEncBEmf.SensorlessSpd)";
static const VISIBLE_STRING  sOdDs_3855_ff[]="Sensorless: Threshold1 speed: % of nominal motor speed [1/100 %] (parEncBEmf.Threshold1Spd)";
static const VISIBLE_STRING  sOdDs_3856_ff[]="Sensorless: Threshold0 speed: % of nominal motor speed [1/100 %] (parEncBEmf.Threshold0Spd)";
static const VISIBLE_STRING  sOdDs_3857_ff[]="Sensorless: Antiglitch threshold speed: % of nominal motor speed [1/100 %] (parEncBEmf.AGlitchSpd)";
static const VISIBLE_STRING  sOdDs_3858_ff[]="Sensorless: Antiglitch fault limit (parEncBEmf.AGlitchFaultLim)";
static const VISIBLE_STRING  sOdDs_3859_ff[]="Sensorless: Use antiglitch filter (parEncBEmf.Flags.EnableAGlitchFilt)";
static const VISIBLE_STRING  sOdDs_385A_ff[]="Sensorless: Use Delta Angle Speed in full sensorless (parEncBEmf.Flags.EnableDeltaAngle)";
static const VISIBLE_STRING  sOdDs_385B_ff[]="Sensorless: Sensorless Alarms Disable Mask (parEncBEmf.DisableAlarmMask)";
static const VISIBLE_STRING  sOdDs_385C_ff[]="Sensorless: Value motor Kt refresh period [sec] (parEncBEmf.KtRefreshPeriod)";
static const VISIBLE_STRING  sOdDs_385D_ff[]="Sensorless: Direct current used when in open loop (% of peak motor current) [1/10 %] (parEncBEmf.IdOpenLoop)";
static const VISIBLE_STRING  sOdDs_385E_ff[]="Sensorless: Disable motor Kt evaluation (parEncBEmf.Flags.DisableKtEval)";
static const VISIBLE_STRING  sOdDs_3861_ff[]="Sensorless: Sensorless state (varEncBEmfState)";
static const VISIBLE_STRING  sOdDs_3862_ff[]="Sensorless: Antiglitch counter (varEncBEmfAGlitchCounter)";
static const VISIBLE_STRING  sOdDs_3863_ff[]="Sensorless: Speed constant conversion (varEncBEmfSpeedKConversion)";
static const VISIBLE_STRING  sOdDs_3864_ff[]="Sensorless: Mechanical speed as delta angles (varEncBEmfMechSpeedDelta)";
static const VISIBLE_STRING  sOdDs_3865_ff[]="Sensorless: Value motor Kt (varEncBEmfValuedKT)";
static const VISIBLE_STRING  sOdDs_3870_ff[]="Hall sensors: Encoder number of poles (zero takes motor poles number) (parEncHall.PoleCounts)";
static const VISIBLE_STRING  sOdDs_3871_ff[]="Hall sensors: 4 wire type enable (parEncHall.Flags.Enable4Wire)";
static const VISIBLE_STRING  sOdDs_3872_ff[]="Hall sensors: SIN channel from ADC (varEncHallChannelSin)";
static const VISIBLE_STRING  sOdDs_3873_ff[]="Hall sensors: COS channel from ADC (varEncHallChannelCos)";
static const VISIBLE_STRING  sOdDs_38F0_ff[]="EFS: Procedure type (parEncEFS.ProcedureType)";
static const VISIBLE_STRING  sOdDs_38F1_ff[]="EFS: Force procedure also if feedback is valid (parEncEFS.Flags.Force)";
static const VISIBLE_STRING  sOdDs_38F2_ff[]="EFS: Wait for speed below this threshold before begin seeking (parEncEFS.SpeedThreshold)";
static const VISIBLE_STRING  sOdDs_38F3_ff[]="EFS: Time for Id ramp [msec] (parEncEFS.IdRampTime)";
static const VISIBLE_STRING  sOdDs_38F4_ff[]="EFS: Current to direct the rotor (% of peak motor current) [1/10 %] (parEncEFS.IdRampCurrent)";
static const VISIBLE_STRING  sOdDs_38F8_ff[]="Encoder: Calculated electrical angle encoder phase offset (varEncMainDeltaElecAngle)";
static const VISIBLE_STRING  sOdDs_38F9_ff[]="EFS: Electrical angle feed ratio from space control loop output [1/65536 %] (parEncEFS.ElecAngleFeed)";
static const VISIBLE_STRING  sOdDs_38FA_ff[]="EFS: Time to keep Id steady after ramping [msec] (parEncEFS.IdSteadyTime)";
static const VISIBLE_STRING  sOdDs_38FB_ff[]="EFS: Integral gain (parEncEFS.PiKi)";
static const VISIBLE_STRING  sOdDs_38FC_ff[]="EFS: Position gain (parEncEFS.PiKp)";
static const VISIBLE_STRING  sOdDs_38FD_ff[]="EFS: Max limitation error (parEncEFS.PiErrMax)";
static const VISIBLE_STRING  sOdDs_38FE_ff[]="EFS: Gain shift multiplier (parEncEFS.PiGlobalShift)";
static const VISIBLE_STRING  sOdDs_38FF_ff[]="EFS: Output limitation (parEncEFS.PiOutValLimit)";
static const VISIBLE_STRING  sOdDs_3900_ff[]="EFS: Monitor PI Quality (16384 = 90 elec deg) (varEncEfsOut.Quality)";
static const VISIBLE_STRING  sOdDs_3901_ff[]="EFS: Monitor PI Error (varEncEfsOut.PIErr)";
static const VISIBLE_STRING  sOdDs_3902_ff[]="EFS: Monitor PI Output (varEncEfsOut.PiOutVal)";
static const VISIBLE_STRING  sOdDs_3903_ff[]="EFS: Monitor PI Error Correction (varEncEfsOut.PiCorrection)";
static const VISIBLE_STRING  sOdDs_3904_ff[]="EFS: Monitor PI Procedure Step (varEncEfsOut.PiStatus)";
static const VISIBLE_STRING  sOdDs_3905_ff[]="EFS: Monitor PI Elec Angle Correction (varEncEfsOut.PiElecAngleCorrection)";
static const VISIBLE_STRING  sOdDs_3906_ff[]="EFS: Monitor PI Automatic Ki Gain (varEncEfsOut.PiAutoKi)";
static const VISIBLE_STRING  sOdDs_3907_ff[]="EFS: Monitor PI Automatic Kp Gain (varEncEfsOut.PiAutoKp)";
static const VISIBLE_STRING  sOdDs_3908_ff[]="EFS: Monitor PI Automatic Shift (varEncEfsOut.PiAutoShift)";
#ifndef _HW_DC
static const VISIBLE_STRING  sOdDs_3910_ff[]="Endat Aux: Clock frequency selector [kHz] (parEncAEndat.ClockFreq)";
static const VISIBLE_STRING  sOdDs_3911_ff[]="Endat Aux: At reset, if multiturn position is equal or above this number, then abspos became negative (parEncAEndat.MTurnStartPos)";
static const VISIBLE_STRING  sOdDs_3918_ff[]="Endat Aux: Overall CRC error counter (varEncAEndatCrcErrors)";
static const VISIBLE_STRING  sOdDs_3919_ff[]="Endat Aux: Data Propagation Delay [nsec] (varEncAEndatPropDelay)";
static const VISIBLE_STRING  sOdDs_391a_ff[]="Endat Aux: Maximum clock frequency allowed [kHz] (varEncAEndatMaxClockFreq)";
static const VISIBLE_STRING  sOdDs_391b_ff[]="Endat Aux: Endat protocol version (varEncAEndatProtocolVer)";
static const VISIBLE_STRING  sOdDs_391c_ff[]="Endat Aux: No. of bits of revolution resolution (varEncAEndatStepPerRevBits)";
static const VISIBLE_STRING  sOdDs_391d_ff[]="Endat Aux: No. of bits for revolution counting (varEncAEndatRevNumBits)";
#endif
static const VISIBLE_STRING  sOdDs_3920_ff[]="Incremental Aux: Encoder line counts per turn (parEncAInc.LineCounts)";
static const VISIBLE_STRING  sOdDs_3922_ff[]="Incremental Aux: Disable Index Error (parEncAInc.Flags.DisableIndexError)";
static const VISIBLE_STRING  sOdDs_3925_ff[]="Incremental Aux: Index Error tolerance (parEncAInc.IndexErrorTolerance)";
static const VISIBLE_STRING  sOdDs_3926_ff[]="Incremental Aux: Enable Index Track (parEncAInc.Flags.EnableIndexTrack)";
static const VISIBLE_STRING  sOdDs_3928_ff[]="Encoder Simulation: Index Line Counts (parEncSimInc.IndexLineCounts)";
static const VISIBLE_STRING  sOdDs_3929_ff[]="Encoder Simulation: Index Position Offset (parEncSimInc.IndexOffset)";
static const VISIBLE_STRING  sOdDs_392A_ff[]="Encoder Simulation: Max Tolerance (parEncSimInc.MaxPosErrTolerance)";
static const VISIBLE_STRING  sOdDs_392B_ff[]="Incremental Aux: Swap input tracks A and B (parEncAInc.Flags.SwapTracks)";
static const VISIBLE_STRING  sOdDs_392C_ff[]="Incremental Aux: Enable step pulse (track A) and direction (track B) mode (parEncAInc.Flags.EnableStepDir)";
static const VISIBLE_STRING  sOdDs_392D_ff[]="Incremental Aux: Enable up pulse (track A) and down pulse (track B) mode (parEncAInc.Flags.EnableUpDown)";
static const VISIBLE_STRING  sOdDs_392e_ff[]="Incremental Aux: Maximum encoder frequency [kHz] (parEncAInc.MaxFrequency)";
static const VISIBLE_STRING  sOdDs_392f_ff[]="Incremental Aux: Disable offset from index track for electrical angle adjustment (parEncAInc.Flags.DisableIndexForElecAngle)";
static const VISIBLE_STRING  sOdDs_3930_ff[]="Incremental Aux: Disable offset from index track for mechanical position adjustment (parEncAInc.Flags.DisableIndexForMechPos)";
static const VISIBLE_STRING  sOdDs_3931_ff[]="Incremental Aux: Disable index synchronization with track A and B (parEncAInc.Flags.DisableIndexTrackSync)";
static const VISIBLE_STRING  sOdDs_3932_ff[]="Incremental Aux: Disable period meter digital interpolation (parEncAInc.Flags.DisableDigitalInterp)";
static const VISIBLE_STRING  sOdDs_3933_ff[]="Incremental Aux: Disable filter watchdog error on digital tracks (parEncAInc.Flags.DisableFilterWDTError)";
static const VISIBLE_STRING  sOdDs_3934_ff[]="Incremental Aux: Enable capturing of all indexes, unregarding index tolerance (parEncAInc.Flags.CaptureAllIndexes)";
static const VISIBLE_STRING  sOdDs_3935_ff[]="Encoder Simulation: Line counts per turn (parEncSimInc.LineCounts)";
static const VISIBLE_STRING  sOdDs_3936_ff[]="Encoder Simulation: Output frequency limit [kHz] (parEncSimInc.MaxFrequency)";
static const VISIBLE_STRING  sOdDs_3937_ff[]="Incremental Aux: Frequency above which interpolation switch from analog to digital [kHz] (parEncAInc.InterpSwitchFrequency)";
static const VISIBLE_STRING  sOdDs_3938_ff[]="Encoder Simulation: Enable fast simulation (parEncSimInc.Flags.EnableFastSimulation)";
static const VISIBLE_STRING  sOdDs_3939_ff[]="Encoder Simulation: Blind AuxSim when ElecAngle not valid (parEncSimInc.Flags.BlindAuxSim)";
static const VISIBLE_STRING  sOdDs_3940_ff[]="Hiperface: At reset, if multiturn position is equal or above this number, then abspos became negative (parEncHiperface.MTurnStartPos)";
static const VISIBLE_STRING  sOdDs_3950_ff[]="Nikon encoder: At reset, if multiturn position is equal or above this number, then abspos became negative (parEncMNikon.MTurnStartPos)";
static const VISIBLE_STRING  sOdDs_3958_ff[]="Nikon encoder: communication baudrate (varEncMNikonBaudrate)";
static const VISIBLE_STRING  sOdDs_3959_ff[]="Nikon encoder: battery line (varEncMNikonBatteryLine)";
static const VISIBLE_STRING  sOdDs_395A_ff[]="Nikon encoder: No. of bits of revolution resolution (varEncMNikonStepPerRevBits)";
static const VISIBLE_STRING  sOdDs_395B_ff[]="Nikon encoder: No. of bits for revolution counting (varEncMNikonRevNumBits)";
static const VISIBLE_STRING  sOdDs_3960_ff[]="Tamagawa encoder: At reset, if multiturn position is equal or above this number, then abspos became negative (parEncMTamagawa.MTurnStartPos)";
static const VISIBLE_STRING  sOdDs_3968_ff[]="Tamagawa encoder: No. of bits for revolution counting (varEncMTamagawaRevNumBits)";
static const VISIBLE_STRING  sOdDs_3969_ff[]="Tamagawa encoder: No. of bits of revolution resolution (varEncMTamagawaStepPerRevBits)";
static const VISIBLE_STRING  sOdDs_396A_ff[]="Tamagawa encoder: communication baudrate (varEncMTamagawaBaudrate)";
static const VISIBLE_STRING  sOdDs_3A02_ff[]="Encoder Feedback: Mechanical Speed (varEncMainMechSpeed)";
static const VISIBLE_STRING  sOdDs_3A03_ff[]="Encoder Feedback: Mechanical Acceleration (varEncMainMechAccel)";
static const VISIBLE_STRING  sOdDs_3A04_ff[]="Encoder Main: Electrical Angle (varEncMainElecAngle)";
static const VISIBLE_STRING  sOdDs_3A05_ff[]="Encoder Main: Status (varEncMainStatus)";
static const VISIBLE_STRING  sOdDs_3A06_ff[]="Encoder Main: Feedback";
static const VISIBLE_STRING  sOdDs_3A07_ff[]="Encoder Main: Absolute position offset";
static const VISIBLE_STRING  sOdDs_3A12_ff[]="Encoder Abs Track: Mechanical Speed (varEncAbsMechSpeed)";
static const VISIBLE_STRING  sOdDs_3A13_ff[]="Encoder Abs Track: Mechanical Acceleration (varEncAbsMechAccel)";
static const VISIBLE_STRING  sOdDs_3A14_ff[]="Encoder Abs Track: Electrical Angle (varEncAbsElecAngle)";
static const VISIBLE_STRING  sOdDs_3A15_ff[]="Encoder Abs Track: Status (varEncAbsStatus)";
static const VISIBLE_STRING  sOdDs_3A22_ff[]="Encoder Rel Track: Mechanical Speed (varEncRelMechSpeed)";
static const VISIBLE_STRING  sOdDs_3A23_ff[]="Encoder Rel Track: Mechanical Acceleration (varEncRelMechAccel)";
static const VISIBLE_STRING  sOdDs_3A24_ff[]="Encoder Rel Track: Electrical Angle (varEncRelElecAngle)";
static const VISIBLE_STRING  sOdDs_3A25_ff[]="Encoder Rel Track: Status (varEncRelStatus)";
static const VISIBLE_STRING  sOdDs_3A32_ff[]="Encoder Aux: Mechanical Speed (varEncAuxMechSpeed)";
static const VISIBLE_STRING  sOdDs_3A33_ff[]="Encoder Aux: Mechanical Acceleration (varEncAuxMechAccel)";
static const VISIBLE_STRING  sOdDs_3A34_ff[]="Encoder Aux: Electrical Angle (varEncAuxElecAngle)";
static const VISIBLE_STRING  sOdDs_3A35_ff[]="Encoder Aux: Status (varEncAuxStatus)";
static const VISIBLE_STRING  sOdDs_3A43_ff[]="Encoder Feedback: Mechanical Acceleration (varEncFbMechAccel)";
static const VISIBLE_STRING  sOdDs_3A44_ff[]="Encoder Feedback: Electrical Angle (varEncFbElecAngle)";
static const VISIBLE_STRING  sOdDs_3A45_ff[]="Encoder Feedback: Status (varEncFbStatus)";
static const VISIBLE_STRING  sOdDs_3F00_ff[]="Motor: Motor Stator Resistance [Ohm] (parMotorData.Resistance)";
static const VISIBLE_STRING  sOdDs_3F01_ff[]="Motor: Motor Stator Inductance [H] (parMotorData.Inductance)";
static const VISIBLE_STRING  sOdDs_3F02_ff[]="Motor: Motor KT (parMotorData.KT)";
static const VISIBLE_STRING  sOdDs_3F03_ff[]="Motor: Motor Peak Current [Arms] (parMotorData.CurrentPeak)";
static const VISIBLE_STRING  sOdDs_3F04_ff[]="Motor: Motor Nominal Speed [rad/sec] (parMotorData.SpeedNominal)";
static const VISIBLE_STRING  sOdDs_3F05_ff[]="Encoder: Electrical angle encoder phase offset (parMotorData.PhaseOffset)";
static const VISIBLE_STRING  sOdDs_3F06_ff[]="Motor: Electrical poles count (parMotorData.PoleNumbers)";
static const VISIBLE_STRING  sOdDs_3F07_ff[]="Motor: Motor Synchronous or Direct Inductance [H] (parMotorData.DirectInductance)";
#ifdef _HW_CT
static const VISIBLE_STRING  sOdDs_3F10_00[]="Senesor: Acclerometer Data x (varSensor.AccDataX)";
static const VISIBLE_STRING  sOdDs_3F10_01[]="Senesor: Acclerometer Data y (varSensor.AccDataY)";
static const VISIBLE_STRING  sOdDs_3F10_02[]="Senesor: Acclerometer Data z (varSensor.AccDataZ)";
static const VISIBLE_STRING  sOdDs_3F11_00[]="Senesor: Temperature (varSensor.Temperature)";
static const VISIBLE_STRING  sOdDs_3F11_01[]="Senesor: Humidity (varSensor.Humidity)";
#endif
static const VISIBLE_STRING  sOdDs_5209_ff[]="Core: Drive Life Time [sec] (varSysLifeTime)";
static const VISIBLE_STRING  sOdDs_5700_ff[]="Core: System Status Flags";
static const VISIBLE_STRING  sOdDs_5701_ff[]="Core: System Boot Error Code (varSysBootErrorCode)";
static const VISIBLE_STRING  sOdDs_5702_ff[]="Core: System Active Warnings (varSysWarnings)";
static const VISIBLE_STRING  sOdDs_5703_ff[]="Core: Wrong value parameter code (varSysWrongParCode)";
static const VISIBLE_STRING  sOdDs_5709_ff[]="Core: Session Max RealTime task exec time [1/10 usec]";
static const VISIBLE_STRING  sOdDs_570A_ff[]="Core: Session Average RealTime task exec time [1/10 usec] (varSysAvgRTExecTime)";
static const VISIBLE_STRING  sOdDs_5710_ff[]="Core: Disable PLC program execution at startup (parPlcExeDisable)";
static const VISIBLE_STRING  sOdDs_5711_ff[]="Core: Drive Mode (parSysDriveMode)";
static const VISIBLE_STRING  sOdDs_5712_ff[]="Core: CPU Speed Selection (parSysCoreSpeed)";
static const VISIBLE_STRING  sOdDs_5713_ff[]="Core: CPU frequency [MHz] (varSysCoreFrequency)";
static const VISIBLE_STRING  sOdDs_5720_ff[]="Serial link: Standard used for IPA numbering (parSerialZeroBaseAddress)";
static const VISIBLE_STRING  sOdDs_5721_ff[]="Serial link: Answer to broadcast requests (parSerialAnswerToBCast)";
static const VISIBLE_STRING  sOdDs_5722_ff[]="Serial link: Baudrate selection (parSerialBaudrate)";
static const VISIBLE_STRING  sOdDs_5723_ff[]="Serial link: Databits selection (parSerialDataBits)";
static const VISIBLE_STRING  sOdDs_5724_ff[]="Serial link: Parity mode (parSerialParity)";
static const VISIBLE_STRING  sOdDs_5725_ff[]="Serial link: Stop bits (parSerialStopBits)";
static const VISIBLE_STRING  sOdDs_5726_ff[]="Serial link: Duplex selection (parSerialDuplex)";
static const VISIBLE_STRING  sOdDs_5727_ff[]="Serial link: Delay after complete TX frame [usec] (parSerialEndDelay)";
static const VISIBLE_STRING  sOdDs_5728_ff[]="Serial link: Delay between complete RX frame and begin of TX frame [usec] (parSerialRxToTxDelay)";
static const VISIBLE_STRING  sOdDs_5729_ff[]="Serial link: Slave address (parSerialSlaveAddress)";
static const VISIBLE_STRING  sOdDs_572A_ff[]="Serial link: Disable modbus over CAN (free aux CAN port) (parSerialDisModbusOverCAN)";
static const VISIBLE_STRING  sOdDs_5730_ff[]="CANOpen: Can Alarms Disable Mask (parCAN.DisableAlarmMask)";
static const VISIBLE_STRING  sOdDs_5780_ff[]="Sync Manager Parameters";
static const VISIBLE_STRING  sOdDs_5780_01[]="K filter for timestamp (0=no filter) (parSyncMgr.TSFilter)";
static const VISIBLE_STRING  sOdDs_5780_02[]="time shift for sync point [nsec] (parSyncMgr.ReSyncDelta)";
static const VISIBLE_STRING  sOdDs_5780_03[]="number of consecutive discardable sample (parSyncMgr.PeakNDiscard)";
static const VISIBLE_STRING  sOdDs_5780_04[]="threshold for peak detection (parSyncMgr.PeakNDiscard)";
static const VISIBLE_STRING  sOdDs_5783_ff[]="Sync Manager Monitoring";
static const VISIBLE_STRING  sOdDs_5783_01[]="Sync time period [nsec] (varSyncMgr_SyncTime)";
static const VISIBLE_STRING  sOdDs_5783_02[]="Max sync time period [nsec] (varSyncMgr_SyncMax)";
static const VISIBLE_STRING  sOdDs_5783_03[]="Min sync time period [nsec] (varSyncMgr_SyncMin)";
static const VISIBLE_STRING  sOdDs_5783_04[]="Sync valid and system synchronized (varSyncMgr_Valid)";

//***************************************************************************
// Link table

const CANOPENCOMDB_INFODESCR  hpsCanOpenParamInfo[]=
{ 
    {0x1000, 0xff, sOdDs_1000_ff},
    {0x1001, 0xff, sOdDs_1001_ff},
    {0x1002, 0xff, sOdDs_1002_ff},
    {0x1005, 0xff, sOdDs_1005_ff},
    {0x1008, 0xff, sOdDs_1008_ff},
    {0x100A, 0xff, sOdDs_100A_ff},
    {0x100C, 0xff, sOdDs_100C_ff},
    {0x100D, 0xff, sOdDs_100D_ff},
    {0x1010, 0x00, sOdDs_NoOfEntries},
    {0x1010, 0x01, sOdDs_1010_01},
    {0x1010, 0xff, sOdDs_1010_ff},
    {0x1011, 0x00, sOdDs_NoOfEntries},
    {0x1011, 0x01, sOdDs_1011_01},
    {0x1011, 0xff, sOdDs_1011_ff},
    {0x1014, 0xff, sOdDs_1014_ff},
    {0x1015, 0xff, sOdDs_1015_ff},
    {0x1017, 0xff, sOdDs_1017_ff},
    {0x1018, 0x00, sOdDs_NoOfEntries},
    {0x1018, 0x01, sOdDs_1018_01},
    {0x1018, 0x02, sOdDs_1018_02},
    {0x1018, 0x03, sOdDs_1018_03},
    {0x1018, 0x04, sOdDs_1018_04},
    {0x1018, 0xff, sOdDs_1018_ff},
    {0x10F0, 0x00, sOdDs_NoOfMappedObj},
    {0x10F0, 0x01, sOdDs_10F0_01},
    {0x10F0, 0xff, sOdDs_10F0_ff},
    {0x1400, 0x00, sOdDs_NoOfEntries},
    {0x1400, 0x01, sOdDs_CobIDPdo},
    {0x1400, 0x02, sOdDs_TxType},
    {0x1400, 0xff, sOdDs_1400_ff},
    {0x1401, 0x00, sOdDs_NoOfEntries},
    {0x1401, 0x01, sOdDs_CobIDPdo},
    {0x1401, 0x02, sOdDs_TxType},
    {0x1401, 0xff, sOdDs_1401_ff},
    {0x1402, 0x00, sOdDs_NoOfEntries},
    {0x1402, 0x01, sOdDs_CobIDPdo},
    {0x1402, 0x02, sOdDs_TxType},
    {0x1402, 0xff, sOdDs_1402_ff},
    {0x1403, 0x00, sOdDs_NoOfEntries},
    {0x1403, 0x01, sOdDs_CobIDPdo},
    {0x1403, 0x02, sOdDs_TxType},
    {0x1403, 0xff, sOdDs_1403_ff},
    {0x1404, 0x00, sOdDs_NoOfEntries},
    {0x1404, 0x01, sOdDs_CobIDPdo},
    {0x1404, 0x02, sOdDs_TxType},
    {0x1404, 0xff, sOdDs_1404_ff},
    {0x1405, 0x00, sOdDs_NoOfEntries},
    {0x1405, 0x01, sOdDs_CobIDPdo},
    {0x1405, 0x02, sOdDs_TxType},
    {0x1405, 0xff, sOdDs_1405_ff},
    {0x1406, 0x00, sOdDs_NoOfEntries},
    {0x1406, 0x01, sOdDs_CobIDPdo},
    {0x1406, 0x02, sOdDs_TxType},
    {0x1406, 0xff, sOdDs_1406_ff},
    {0x1407, 0x00, sOdDs_NoOfEntries},
    {0x1407, 0x01, sOdDs_CobIDPdo},
    {0x1407, 0x02, sOdDs_TxType},
    {0x1407, 0xff, sOdDs_1407_ff},
    {0x1600, 0x00, sOdDs_NoOfMappedObj},
    {0x1600, 0x01, sOdDs_MapObj1},
    {0x1600, 0x02, sOdDs_MapObj2},
    {0x1600, 0x03, sOdDs_MapObj3},
    {0x1600, 0x04, sOdDs_MapObj4},
    {0x1600, 0x05, sOdDs_MapObj5},
    {0x1600, 0x06, sOdDs_MapObj6},
    {0x1600, 0x07, sOdDs_MapObj7},
    {0x1600, 0x08, sOdDs_MapObj8},
    {0x1600, 0xff, sOdDs_1600_ff},
    {0x1601, 0x00, sOdDs_NoOfMappedObj},
    {0x1601, 0x01, sOdDs_MapObj1},
    {0x1601, 0x02, sOdDs_MapObj2},
    {0x1601, 0x03, sOdDs_MapObj3},
    {0x1601, 0x04, sOdDs_MapObj4},
    {0x1601, 0x05, sOdDs_MapObj5},
    {0x1601, 0x06, sOdDs_MapObj6},
    {0x1601, 0x07, sOdDs_MapObj7},
    {0x1601, 0x08, sOdDs_MapObj8},
    {0x1601, 0xff, sOdDs_1601_ff},
    {0x1602, 0x00, sOdDs_NoOfMappedObj},
    {0x1602, 0x01, sOdDs_MapObj1},
    {0x1602, 0x02, sOdDs_MapObj2},
    {0x1602, 0x03, sOdDs_MapObj3},
    {0x1602, 0x04, sOdDs_MapObj4},
    {0x1602, 0x05, sOdDs_MapObj5},
    {0x1602, 0x06, sOdDs_MapObj6},
    {0x1602, 0x07, sOdDs_MapObj7},
    {0x1602, 0x08, sOdDs_MapObj8},
    {0x1602, 0xff, sOdDs_1602_ff},
    {0x1603, 0x00, sOdDs_NoOfMappedObj},
    {0x1603, 0x01, sOdDs_MapObj1},
    {0x1603, 0x02, sOdDs_MapObj2},
    {0x1603, 0x03, sOdDs_MapObj3},
    {0x1603, 0x04, sOdDs_MapObj4},
    {0x1603, 0x05, sOdDs_MapObj5},
    {0x1603, 0x06, sOdDs_MapObj6},
    {0x1603, 0x07, sOdDs_MapObj7},
    {0x1603, 0x08, sOdDs_MapObj8},
    {0x1603, 0xff, sOdDs_1603_ff},
    {0x1604, 0x00, sOdDs_NoOfMappedObj},
    {0x1604, 0x01, sOdDs_MapObj1},
    {0x1604, 0x02, sOdDs_MapObj2},
    {0x1604, 0x03, sOdDs_MapObj3},
    {0x1604, 0x04, sOdDs_MapObj4},
    {0x1604, 0x05, sOdDs_MapObj5},
    {0x1604, 0x06, sOdDs_MapObj6},
    {0x1604, 0x07, sOdDs_MapObj7},
    {0x1604, 0x08, sOdDs_MapObj8},
    {0x1604, 0xff, sOdDs_1604_ff},
    {0x1605, 0x00, sOdDs_NoOfMappedObj},
    {0x1605, 0x01, sOdDs_MapObj1},
    {0x1605, 0x02, sOdDs_MapObj2},
    {0x1605, 0x03, sOdDs_MapObj3},
    {0x1605, 0x04, sOdDs_MapObj4},
    {0x1605, 0x05, sOdDs_MapObj5},
    {0x1605, 0x06, sOdDs_MapObj6},
    {0x1605, 0x07, sOdDs_MapObj7},
    {0x1605, 0x08, sOdDs_MapObj8},
    {0x1605, 0xff, sOdDs_1605_ff},
    {0x1606, 0x00, sOdDs_NoOfMappedObj},
    {0x1606, 0x01, sOdDs_MapObj1},
    {0x1606, 0x02, sOdDs_MapObj2},
    {0x1606, 0x03, sOdDs_MapObj3},
    {0x1606, 0x04, sOdDs_MapObj4},
    {0x1606, 0x05, sOdDs_MapObj5},
    {0x1606, 0x06, sOdDs_MapObj6},
    {0x1606, 0x07, sOdDs_MapObj7},
    {0x1606, 0x08, sOdDs_MapObj8},
    {0x1606, 0xff, sOdDs_1606_ff},
    {0x1607, 0x00, sOdDs_NoOfMappedObj},
    {0x1607, 0x01, sOdDs_MapObj1},
    {0x1607, 0x02, sOdDs_MapObj2},
    {0x1607, 0x03, sOdDs_MapObj3},
    {0x1607, 0x04, sOdDs_MapObj4},
    {0x1607, 0x05, sOdDs_MapObj5},
    {0x1607, 0x06, sOdDs_MapObj6},
    {0x1607, 0x07, sOdDs_MapObj7},
    {0x1607, 0x08, sOdDs_MapObj8},
    {0x1607, 0xff, sOdDs_1607_ff},
    {0x1800, 0x00, sOdDs_NoOfEntries},
    {0x1800, 0x01, sOdDs_CobIDPdo},
    {0x1800, 0x02, sOdDs_TxType},
    {0x1800, 0x03, sOdDs_InhibitTime},
    {0x1800, 0xff, sOdDs_1800_ff},
    {0x1801, 0x00, sOdDs_NoOfEntries},
    {0x1801, 0x01, sOdDs_CobIDPdo},
    {0x1801, 0x02, sOdDs_TxType},
    {0x1801, 0x03, sOdDs_InhibitTime},
    {0x1801, 0xff, sOdDs_1801_ff},
    {0x1802, 0x00, sOdDs_NoOfEntries},
    {0x1802, 0x01, sOdDs_CobIDPdo},
    {0x1802, 0x02, sOdDs_TxType},
    {0x1802, 0x03, sOdDs_InhibitTime},
    {0x1802, 0xff, sOdDs_1802_ff},
    {0x1803, 0x00, sOdDs_NoOfEntries},
    {0x1803, 0x01, sOdDs_CobIDPdo},
    {0x1803, 0x02, sOdDs_TxType},
    {0x1803, 0x03, sOdDs_InhibitTime},
    {0x1803, 0xff, sOdDs_1803_ff},
    {0x1804, 0x00, sOdDs_NoOfEntries},
    {0x1804, 0x01, sOdDs_CobIDPdo},
    {0x1804, 0x02, sOdDs_TxType},
    {0x1804, 0x03, sOdDs_InhibitTime},
    {0x1804, 0xff, sOdDs_1804_ff},
    {0x1805, 0x00, sOdDs_NoOfEntries},
    {0x1805, 0x01, sOdDs_CobIDPdo},
    {0x1805, 0x02, sOdDs_TxType},
    {0x1805, 0x03, sOdDs_InhibitTime},
    {0x1805, 0xff, sOdDs_1805_ff},
    {0x1806, 0x00, sOdDs_NoOfEntries},
    {0x1806, 0x01, sOdDs_CobIDPdo},
    {0x1806, 0x02, sOdDs_TxType},
    {0x1806, 0x03, sOdDs_InhibitTime},
    {0x1806, 0xff, sOdDs_1806_ff},
    {0x1807, 0x00, sOdDs_NoOfEntries},
    {0x1807, 0x01, sOdDs_CobIDPdo},
    {0x1807, 0x02, sOdDs_TxType},
    {0x1807, 0x03, sOdDs_InhibitTime},
    {0x1807, 0xff, sOdDs_1807_ff},
    {0x1A00, 0x00, sOdDs_NoOfMappedObj},
    {0x1A00, 0x01, sOdDs_MapObj1},
    {0x1A00, 0x02, sOdDs_MapObj2},
    {0x1A00, 0x03, sOdDs_MapObj3},
    {0x1A00, 0x04, sOdDs_MapObj4},
    {0x1A00, 0x05, sOdDs_MapObj5},
    {0x1A00, 0x06, sOdDs_MapObj6},
    {0x1A00, 0x07, sOdDs_MapObj7},
    {0x1A00, 0x08, sOdDs_MapObj8},
    {0x1A00, 0xff, sOdDs_1A00_ff},
    {0x1A01, 0x00, sOdDs_NoOfMappedObj},
    {0x1A01, 0x01, sOdDs_MapObj1},
    {0x1A01, 0x02, sOdDs_MapObj2},
    {0x1A01, 0x03, sOdDs_MapObj3},
    {0x1A01, 0x04, sOdDs_MapObj4},
    {0x1A01, 0x05, sOdDs_MapObj5},
    {0x1A01, 0x06, sOdDs_MapObj6},
    {0x1A01, 0x07, sOdDs_MapObj7},
    {0x1A01, 0x08, sOdDs_MapObj8},
    {0x1A01, 0xff, sOdDs_1A01_ff},
    {0x1A02, 0x00, sOdDs_NoOfMappedObj},
    {0x1A02, 0x01, sOdDs_MapObj1},
    {0x1A02, 0x02, sOdDs_MapObj2},
    {0x1A02, 0x03, sOdDs_MapObj3},
    {0x1A02, 0x04, sOdDs_MapObj4},
    {0x1A02, 0x05, sOdDs_MapObj5},
    {0x1A02, 0x06, sOdDs_MapObj6},
    {0x1A02, 0x07, sOdDs_MapObj7},
    {0x1A02, 0x08, sOdDs_MapObj8},
    {0x1A02, 0xff, sOdDs_1A02_ff},
    {0x1A03, 0x00, sOdDs_NoOfMappedObj},
    {0x1A03, 0x01, sOdDs_MapObj1},
    {0x1A03, 0x02, sOdDs_MapObj2},
    {0x1A03, 0x03, sOdDs_MapObj3},
    {0x1A03, 0x04, sOdDs_MapObj4},
    {0x1A03, 0x05, sOdDs_MapObj5},
    {0x1A03, 0x06, sOdDs_MapObj6},
    {0x1A03, 0x07, sOdDs_MapObj7},
    {0x1A03, 0x08, sOdDs_MapObj8},
    {0x1A03, 0xff, sOdDs_1A03_ff},
    {0x1A04, 0x00, sOdDs_NoOfMappedObj},
    {0x1A04, 0x01, sOdDs_MapObj1},
    {0x1A04, 0x02, sOdDs_MapObj2},
    {0x1A04, 0x03, sOdDs_MapObj3},
    {0x1A04, 0x04, sOdDs_MapObj4},
    {0x1A04, 0x05, sOdDs_MapObj5},
    {0x1A04, 0x06, sOdDs_MapObj6},
    {0x1A04, 0x07, sOdDs_MapObj7},
    {0x1A04, 0x08, sOdDs_MapObj8},
    {0x1A04, 0xff, sOdDs_1A04_ff},
    {0x1A05, 0x00, sOdDs_NoOfMappedObj},
    {0x1A05, 0x01, sOdDs_MapObj1},
    {0x1A05, 0x02, sOdDs_MapObj2},
    {0x1A05, 0x03, sOdDs_MapObj3},
    {0x1A05, 0x04, sOdDs_MapObj4},
    {0x1A05, 0x05, sOdDs_MapObj5},
    {0x1A05, 0x06, sOdDs_MapObj6},
    {0x1A05, 0x07, sOdDs_MapObj7},
    {0x1A05, 0x08, sOdDs_MapObj8},
    {0x1A05, 0xff, sOdDs_1A05_ff},
    {0x1A06, 0x00, sOdDs_NoOfMappedObj},
    {0x1A06, 0x01, sOdDs_MapObj1},
    {0x1A06, 0x02, sOdDs_MapObj2},
    {0x1A06, 0x03, sOdDs_MapObj3},
    {0x1A06, 0x04, sOdDs_MapObj4},
    {0x1A06, 0x05, sOdDs_MapObj5},
    {0x1A06, 0x06, sOdDs_MapObj6},
    {0x1A06, 0x07, sOdDs_MapObj7},
    {0x1A06, 0x08, sOdDs_MapObj8},
    {0x1A06, 0xff, sOdDs_1A06_ff},
    {0x1A07, 0x00, sOdDs_NoOfMappedObj},
    {0x1A07, 0x01, sOdDs_MapObj1},
    {0x1A07, 0x02, sOdDs_MapObj2},
    {0x1A07, 0x03, sOdDs_MapObj3},
    {0x1A07, 0x04, sOdDs_MapObj4},
    {0x1A07, 0x05, sOdDs_MapObj5},
    {0x1A07, 0x06, sOdDs_MapObj6},
    {0x1A07, 0x07, sOdDs_MapObj7},
    {0x1A07, 0x08, sOdDs_MapObj8},
    {0x1A07, 0xff, sOdDs_1A07_ff},
    {0x1C00, 0x00, sOdDs_NoOfEntries},
    {0x1C00, 0x01, sOdDs_1C00_01},
    {0x1C00, 0x02, sOdDs_1C00_02},
    {0x1C00, 0x03, sOdDs_1C00_03},
    {0x1C00, 0x04, sOdDs_1C00_04},
    {0x1C00, 0xff, sOdDs_1C00_ff},
    {0x1C02, 0x00, sOdDs_NoOfEntries},
    {0x1C02, 0x01, sOdDs_1C02_01},
    {0x1C02, 0x02, sOdDs_1C02_02},
    {0x1C02, 0x03, sOdDs_1C02_03},
    {0x1C02, 0x04, sOdDs_1C02_04},
    {0x1C02, 0xff, sOdDs_1C02_ff},
    {0x1C12, 0x00, sOdDs_1C12_00},
    {0x1C12, 0x01, sOdDs_1C12_01},
    {0x1C12, 0x02, sOdDs_1C12_02},
    {0x1C12, 0x03, sOdDs_1C12_03},
    {0x1C12, 0x04, sOdDs_1C12_04},
    {0x1C12, 0x05, sOdDs_1C12_05},
    {0x1C12, 0x06, sOdDs_1C12_06},
    {0x1C12, 0x07, sOdDs_1C12_07},
    {0x1C12, 0x08, sOdDs_1C12_08},
    {0x1C12, 0xff, sOdDs_1C12_ff},
    {0x1C13, 0x00, sOdDs_1C13_00},
    {0x1C13, 0x01, sOdDs_1C12_01},
    {0x1C13, 0x02, sOdDs_1C12_02},
    {0x1C13, 0x03, sOdDs_1C12_03},
    {0x1C13, 0x04, sOdDs_1C12_04},
    {0x1C13, 0x05, sOdDs_1C12_05},
    {0x1C13, 0x06, sOdDs_1C12_06},
    {0x1C13, 0x07, sOdDs_1C12_07},
    {0x1C13, 0x08, sOdDs_1C12_08},
    {0x1C13, 0xff, sOdDs_1C13_ff},
    {0x1C32, 0x00, sOdDs_NoOfEntries},
    {0x1C32, 0x01, sOdDs_1C32_01},
    {0x1C32, 0x02, sOdDs_1C32_02},
    {0x1C32, 0x03, sOdDs_1C32_03},
    {0x1C32, 0x04, sOdDs_1C32_04},
    {0x1C32, 0x05, sOdDs_1C32_05},
    {0x1C32, 0x06, sOdDs_1C32_06},
    {0x1C32, 0x09, sOdDs_1C32_09},
    {0x1C32, 0x0A, sOdDs_1C32_0A},
    {0x1C32, 0x0B, sOdDs_1C32_0B},
    {0x1C32, 0x0C, sOdDs_1C32_0C},
    {0x1C32, 0x20, sOdDs_1C32_20},
    {0x1C32, 0xff, sOdDs_1C32_ff},
    {0x1C33, 0x00, sOdDs_NoOfEntries},
    {0x1C33, 0x01, sOdDs_1C32_01},
    {0x1C33, 0x02, sOdDs_1C32_02},
    {0x1C33, 0x03, sOdDs_1C32_03},
    {0x1C33, 0x04, sOdDs_1C32_04},
    {0x1C33, 0x05, sOdDs_1C32_05},
    {0x1C33, 0x06, sOdDs_1C32_06},
    {0x1C33, 0x09, sOdDs_1C32_09},
    {0x1C33, 0x0A, sOdDs_1C32_0A},
    {0x1C33, 0x0B, sOdDs_1C32_0B},
    {0x1C33, 0x0C, sOdDs_1C32_0C},
    {0x1C33, 0x20, sOdDs_1C32_20},
    {0x1C33, 0xff, sOdDs_1C33_ff},
    {0x3000, 0xff, sOdDs_3000_ff},
    {0x3001, 0xff, sOdDs_3001_ff},
    {0x3002, 0xff, sOdDs_3002_ff},
    {0x3003, 0xff, sOdDs_3003_ff},
    {0x3008, 0xff, sOdDs_3008_ff},
    {0x3009, 0xff, sOdDs_3009_ff},
    {0x300A, 0xff, sOdDs_300A_ff},
    {0x300B, 0xff, sOdDs_300B_ff},
    {0x3100, 0xff, sOdDs_3100_ff},
    {0x3101, 0xff, sOdDs_3101_ff},
    {0x3102, 0xff, sOdDs_3102_ff},
    {0x3103, 0xff, sOdDs_3103_ff},
    {0x3104, 0xff, sOdDs_3104_ff},
    {0x3105, 0xff, sOdDs_3105_ff},
    {0x3106, 0xff, sOdDs_3106_ff},
    {0x3107, 0xff, sOdDs_3107_ff},
    {0x3108, 0xff, sOdDs_3108_ff},
    {0x3109, 0xff, sOdDs_3109_ff},
    {0x310A, 0xff, sOdDs_310A_ff},
    {0x310B, 0xff, sOdDs_310B_ff},
    {0x310C, 0xff, sOdDs_310C_ff},
    {0x310D, 0xff, sOdDs_310D_ff},
    {0x310E, 0xff, sOdDs_310E_ff},
    {0x310F, 0xff, sOdDs_310F_ff},
    {0x3110, 0xff, sOdDs_3110_ff},
    {0x3111, 0xff, sOdDs_3111_ff},
    {0x3112, 0xff, sOdDs_3112_ff},
    {0x3113, 0xff, sOdDs_3113_ff},
    {0x3114, 0xff, sOdDs_3114_ff},
    {0x3115, 0xff, sOdDs_3115_ff},
    {0x3116, 0xff, sOdDs_3116_ff},
    {0x3117, 0xff, sOdDs_3117_ff},
    {0x3118, 0xff, sOdDs_3118_ff},
    {0x3119, 0xff, sOdDs_3119_ff},
    {0x311A, 0xff, sOdDs_311a_ff},
    {0x311B, 0xff, sOdDs_311b_ff},
    {0x311C, 0xff, sOdDs_311c_ff},
    {0x311D, 0xff, sOdDs_311d_ff},
    {0x311E, 0xff, sOdDs_311e_ff},
    {0x311F, 0xff, sOdDs_311f_ff},
    {0x3120, 0xff, sOdDs_3120_ff},
    {0x3121, 0xff, sOdDs_3121_ff},
    {0x3122, 0xff, sOdDs_3122_ff},
    {0x3123, 0xff, sOdDs_3123_ff},
    {0x3124, 0xff, sOdDs_3124_ff},
    {0x3125, 0xff, sOdDs_3125_ff},
    {0x3126, 0xff, sOdDs_3126_ff},
    {0x3127, 0xff, sOdDs_3127_ff},
    {0x3128, 0xff, sOdDs_3128_ff},
    {0x3129, 0xff, sOdDs_3129_ff},
#ifdef _HW_DC
    {0x312A, 0xff, sOdDs_312a_ff},
    {0x312B, 0xff, sOdDs_312b_ff},
#endif
    {0x312C, 0xff, sOdDs_312c_ff},
    {0x3130, 0xff, sOdDs_3130_ff},
    {0x3133, 0xff, sOdDs_3133_ff},
    {0x3134, 0xff, sOdDs_3134_ff},
    {0x3135, 0xff, sOdDs_3135_ff},
    {0x3136, 0xff, sOdDs_3136_ff},
    {0x3140, 0xff, sOdDs_3140_ff},
    {0x3141, 0xff, sOdDs_3141_ff},
    {0x3142, 0xff, sOdDs_3142_ff},
    {0x3143, 0xff, sOdDs_3143_ff},
    {0x3144, 0xff, sOdDs_3144_ff},
    {0x3146, 0xff, sOdDs_3146_ff},
    {0x3150, 0xff, sOdDs_3150_ff},
    {0x3151, 0xff, sOdDs_3151_ff},
    {0x3152, 0xff, sOdDs_3152_ff},
    {0x3153, 0xff, sOdDs_3153_ff},
    {0x3154, 0xff, sOdDs_3154_ff},
    {0x3158, 0xff, sOdDs_3158_ff},
    {0x3159, 0xff, sOdDs_3159_ff},
    {0x315A, 0xff, sOdDs_315A_ff},
    {0x315B, 0xff, sOdDs_315B_ff},
    {0x315C, 0xff, sOdDs_315C_ff},
    {0x3160, 0xff, sOdDs_3160_ff},
    {0x3161, 0xff, sOdDs_3161_ff},
    {0x3162, 0xff, sOdDs_3162_ff},
    {0x3163, 0xff, sOdDs_3163_ff},
    {0x3164, 0xff, sOdDs_3164_ff},
    {0x3168, 0xff, sOdDs_3168_ff},
    {0x3169, 0xff, sOdDs_3169_ff},
    {0x316A, 0xff, sOdDs_316A_ff},
    {0x316B, 0xff, sOdDs_316B_ff},
    {0x316C, 0xff, sOdDs_316C_ff},
#ifdef _HW_DC
    {0x3188, 0xff, sOdDs_3188_ff},
    {0x3189, 0xff, sOdDs_3189_ff},
    {0x318A, 0xff, sOdDs_318A_ff},
    {0x318B, 0xff, sOdDs_318B_ff},
    {0x318C, 0xff, sOdDs_318C_ff},
    {0x318D, 0xff, sOdDs_318D_ff},
#endif
    {0x31A0, 0xff, sOdDs_31a0_ff},
    {0x31A1, 0xff, sOdDs_31a1_ff},
    {0x31A2, 0xff, sOdDs_31a2_ff},
    {0x31A8, 0xff, sOdDs_31a8_ff},
    {0x31A9, 0xff, sOdDs_31a9_ff},
    {0x31AA, 0xff, sOdDs_31aa_ff},
    {0x31AB, 0xff, sOdDs_31ab_ff},
    {0x3200, 0xff, sOdDs_3200_ff},
    {0x3201, 0xff, sOdDs_3201_ff},
    {0x3202, 0xff, sOdDs_3202_ff},
    {0x3203, 0xff, sOdDs_3203_ff},
    {0x3204, 0xff, sOdDs_3204_ff},
    {0x3205, 0xff, sOdDs_3205_ff},
    {0x3206, 0xff, sOdDs_3206_ff},
    {0x3207, 0xff, sOdDs_3207_ff},
    {0x3208, 0xff, sOdDs_3208_ff},
    {0x3209, 0xff, sOdDs_3209_ff},
    {0x320A, 0xff, sOdDs_320A_ff},
#ifdef _HW_DC
    {0x320B, 0xff, sOdDs_320B_ff},
    {0x320C, 0xff, sOdDs_320C_ff},
    {0x320D, 0xff, sOdDs_320D_ff},
    {0x320E, 0xff, sOdDs_320E_ff},
    {0x320F, 0xff, sOdDs_320F_ff},
#endif
#ifdef _HW_CT
    {0x3210, 0xff, sOdDs_3210_ff},
#endif
    {0x3220, 0xff, sOdDs_3220_ff},
    {0x3221, 0xff, sOdDs_3221_ff},
    {0x3222, 0xff, sOdDs_3222_ff},
    {0x3223, 0xff, sOdDs_3223_ff},
    {0x3224, 0xff, sOdDs_3224_ff},
    {0x3225, 0xff, sOdDs_3225_ff},
    {0x3226, 0xff, sOdDs_3226_ff},
    {0x3227, 0xff, sOdDs_3227_ff},
    {0x3228, 0xff, sOdDs_3228_ff},
#ifdef _HW_DC
    {0x3250, 0x00, sOdDs_NoOfEntries},
    {0x3250, 0x01, sOdDs_3250_01},
    {0x3250, 0x02, sOdDs_3250_02},
    {0x3250, 0x03, sOdDs_3250_03},
    {0x3250, 0x04, sOdDs_3250_04},
    {0x3250, 0xff, sOdDs_3250_ff},
#endif
    {0x3251, 0x00, sOdDs_NoOfEntries},
    {0x3251, 0x01, sOdDs_3250_01},
    {0x3251, 0x02, sOdDs_3250_02},
    {0x3251, 0x03, sOdDs_3250_03},
    {0x3251, 0x04, sOdDs_3250_04},
    {0x3251, 0xff, sOdDs_3251_ff},

    {0x3300, 0xff, sOdDs_3300_ff},
    {0x330A, 0xff, sOdDs_330A_ff},
    {0x330B, 0xff, sOdDs_330B_ff},
    {0x330C, 0x00, sOdDs_NoOfEntries},
    {0x330C, 0x01, sOdDs_330C_01},
    {0x330C, 0x02, sOdDs_330C_02},
    {0x330C, 0x03, sOdDs_330C_03},
    {0x330C, 0x04, sOdDs_330C_04},
    {0x330C, 0x05, sOdDs_330C_05},
    {0x330C, 0x06, sOdDs_330C_06},
    {0x330C, 0xff, sOdDs_330C_ff},
    {0x330D, 0xff, sOdDs_330D_ff},
    {0x330E, 0xff, sOdDs_330E_ff},
    {0x3320, 0xff, sOdDs_3320_ff},
    {0x3321, 0xff, sOdDs_3321_ff},
    {0x3322, 0xff, sOdDs_3322_ff},
    {0x3323, 0xff, sOdDs_3323_ff},
    {0x3340, 0xff, sOdDs_3340_ff},
    {0x3341, 0xff, sOdDs_3341_ff},
    {0x3342, 0xff, sOdDs_3342_ff},
    {0x3343, 0xff, sOdDs_3343_ff},
    {0x3344, 0xff, sOdDs_3344_ff},
    {0x3345, 0xff, sOdDs_3345_ff},
    {0x3346, 0xff, sOdDs_3346_ff},
    {0x3350, 0x00, sOdDs_NoOfEntries},
    {0x3350, 0x01, sOdDs_Angle},
    {0x3350, 0x02, sOdDs_Turns},
    {0x3350, 0xff, sOdDs_DemandPos},
    {0x3352, 0xff, sOdDs_3352_ff},
    {0x3356, 0xff, sOdDs_DemandPos},
    {0x3800, 0xff, sOdDs_3800_ff},
    {0x3801, 0xff, sOdDs_3801_ff},
    {0x3802, 0xff, sOdDs_3802_ff},
    {0x3803, 0xff, sOdDs_3803_ff},
    {0x3804, 0xff, sOdDs_3804_ff},
    {0x3805, 0xff, sOdDs_3805_ff},
    {0x3806, 0xff, sOdDs_3806_ff},
    {0x3807, 0xff, sOdDs_3807_ff},
    {0x3808, 0xff, sOdDs_3808_ff},
    {0x3809, 0xff, sOdDs_3809_ff},
    {0x380A, 0xff, sOdDs_380A_ff},
    {0x380B, 0xff, sOdDs_380B_ff},
    {0x380C, 0xff, sOdDs_380C_ff},
    {0x380D, 0xff, sOdDs_380D_ff},
    {0x380E, 0xff, sOdDs_380E_ff},
    {0x380F, 0xff, sOdDs_380F_ff},
    {0x3810, 0xff, sOdDs_3810_ff},
    {0x3811, 0xff, sOdDs_3811_ff},
    {0x3818, 0xff, sOdDs_3818_ff},
    {0x3819, 0xff, sOdDs_3819_ff},
    {0x381A, 0xff, sOdDs_381a_ff},
    {0x381B, 0xff, sOdDs_381b_ff},
    {0x381C, 0xff, sOdDs_381c_ff},
    {0x381D, 0xff, sOdDs_381d_ff},
    {0x3820, 0xff, sOdDs_3820_ff},
    {0x3821, 0xff, sOdDs_3821_ff},
    {0x3822, 0xff, sOdDs_3822_ff},
    {0x3823, 0xff, sOdDs_3823_ff},
    {0x3824, 0xff, sOdDs_3824_ff},
    {0x3825, 0xff, sOdDs_3825_ff},
    {0x3826, 0xff, sOdDs_3826_ff},
    {0x3827, 0xff, sOdDs_3827_ff},
    {0x3828, 0xff, sOdDs_3828_ff},
    {0x3829, 0xff, sOdDs_3829_ff},
    {0x382b, 0xff, sOdDs_382b_ff},
    {0x382c, 0xff, sOdDs_382c_ff},
    {0x382d, 0xff, sOdDs_382d_ff},
    {0x382e, 0xff, sOdDs_382e_ff},
    {0x382f, 0xff, sOdDs_382f_ff},
    {0x3830, 0xff, sOdDs_3830_ff},
    {0x3831, 0xff, sOdDs_3831_ff},
    {0x3832, 0xff, sOdDs_3832_ff},
    {0x3833, 0xff, sOdDs_3833_ff},
    {0x3834, 0xff, sOdDs_3834_ff},
#ifdef _HW_DC
    {0x3835, 0xff, sOdDs_3835_ff},
#endif
    {0x3836, 0xff, sOdDs_3836_ff},
    {0x3837, 0xff, sOdDs_3837_ff},
    {0x3840, 0xff, sOdDs_3840_ff},
    {0x3841, 0xff, sOdDs_3841_ff},
    {0x3842, 0xff, sOdDs_3842_ff},
    {0x3843, 0xff, sOdDs_3843_ff},
    {0x3844, 0xff, sOdDs_3844_ff},
    {0x3845, 0xff, sOdDs_3845_ff},
    {0x3846, 0xff, sOdDs_3846_ff},
    {0x3848, 0xff, sOdDs_3848_ff},
    {0x3849, 0xff, sOdDs_3849_ff},
    {0x384A, 0xff, sOdDs_384A_ff},
    {0x384E, 0xff, sOdDs_384E_ff},
    {0x384F, 0xff, sOdDs_384F_ff},
    {0x3850, 0xff, sOdDs_3850_ff},
    {0x3851, 0xff, sOdDs_3851_ff},
    {0x3852, 0xff, sOdDs_3852_ff},
    {0x3853, 0xff, sOdDs_3853_ff},
    {0x3854, 0xff, sOdDs_3854_ff},
    {0x3855, 0xff, sOdDs_3855_ff},
    {0x3856, 0xff, sOdDs_3856_ff},
    {0x3857, 0xff, sOdDs_3857_ff},
    {0x3858, 0xff, sOdDs_3858_ff},
    {0x3859, 0xff, sOdDs_3859_ff},
    {0x385A, 0xff, sOdDs_385A_ff},
    {0x385B, 0xff, sOdDs_385B_ff},
    {0x385C, 0xff, sOdDs_385C_ff},
    {0x385D, 0xff, sOdDs_385D_ff},
    {0x385E, 0xff, sOdDs_385E_ff},
    {0x3861, 0xff, sOdDs_3861_ff},
    {0x3862, 0xff, sOdDs_3862_ff},
    {0x3863, 0xff, sOdDs_3863_ff},
    {0x3864, 0xff, sOdDs_3864_ff},
    {0x3865, 0xff, sOdDs_3865_ff},
    {0x3870, 0xff, sOdDs_3870_ff},
    {0x3871, 0xff, sOdDs_3871_ff},
    {0x3872, 0xff, sOdDs_3872_ff},
    {0x3873, 0xff, sOdDs_3873_ff},
    {0x38E0, 0xff, sOdDs_38E0_ff},
    {0x38F0, 0xff, sOdDs_38F0_ff},
    {0x38F1, 0xff, sOdDs_38F1_ff},
    {0x38F2, 0xff, sOdDs_38F2_ff},
    {0x38F3, 0xff, sOdDs_38F3_ff},
    {0x38F4, 0xff, sOdDs_38F4_ff},
    {0x38F8, 0xff, sOdDs_38F8_ff},
    {0x38F9, 0xff, sOdDs_38F9_ff},
    {0x38FA, 0xff, sOdDs_38FA_ff},
    {0x38FB, 0xff, sOdDs_38FB_ff},
    {0x38FC, 0xff, sOdDs_38FC_ff},
    {0x38FD, 0xff, sOdDs_38FD_ff},
    {0x38FE, 0xff, sOdDs_38FE_ff},
    {0x38FF, 0xff, sOdDs_38FF_ff},
    {0x3900, 0xff, sOdDs_3900_ff},
    {0x3901, 0xff, sOdDs_3901_ff},
    {0x3902, 0xff, sOdDs_3902_ff},
    {0x3903, 0xff, sOdDs_3903_ff},
    {0x3904, 0xff, sOdDs_3904_ff},
    {0x3905, 0xff, sOdDs_3905_ff},
    {0x3906, 0xff, sOdDs_3906_ff},
    {0x3907, 0xff, sOdDs_3907_ff},
    {0x3908, 0xff, sOdDs_3908_ff},
#ifndef _HW_DC
    {0x3910, 0xff, sOdDs_3910_ff},
    {0x3911, 0xff, sOdDs_3911_ff},
    {0x3918, 0xff, sOdDs_3918_ff},
    {0x3919, 0xff, sOdDs_3919_ff},
    {0x391A, 0xff, sOdDs_391a_ff},
    {0x391B, 0xff, sOdDs_391b_ff},
    {0x391C, 0xff, sOdDs_391c_ff},
    {0x391D, 0xff, sOdDs_391d_ff},
#endif
    {0x3920, 0xff, sOdDs_3920_ff},
    {0x3922, 0xff, sOdDs_3922_ff},
    {0x3925, 0xff, sOdDs_3925_ff},
    {0x3926, 0xff, sOdDs_3926_ff},
    {0x3928, 0xff, sOdDs_3928_ff},
    {0x3929, 0xff, sOdDs_3929_ff},
    {0x392A, 0xff, sOdDs_392A_ff},
    {0x392B, 0xff, sOdDs_392B_ff},
    {0x392C, 0xff, sOdDs_392C_ff},
    {0x392D, 0xff, sOdDs_392D_ff},
    {0x392e, 0xff, sOdDs_392e_ff},
    {0x392f, 0xff, sOdDs_392f_ff},
    {0x3930, 0xff, sOdDs_3930_ff},
    {0x3931, 0xff, sOdDs_3931_ff},
    {0x3932, 0xff, sOdDs_3932_ff},
    {0x3933, 0xff, sOdDs_3933_ff},
    {0x3934, 0xff, sOdDs_3934_ff},
    {0x3935, 0xff, sOdDs_3935_ff},
    {0x3936, 0xff, sOdDs_3936_ff},
    {0x3937, 0xff, sOdDs_3937_ff},
    {0x3938, 0xff, sOdDs_3938_ff},
    {0x3939, 0xff, sOdDs_3939_ff},
    {0x3940, 0xff, sOdDs_3940_ff},
    {0x3950, 0xff, sOdDs_3950_ff},
    {0x3958, 0xff, sOdDs_3958_ff},
    {0x3959, 0xff, sOdDs_3959_ff},
    {0x395A, 0xff, sOdDs_395A_ff},
    {0x395B, 0xff, sOdDs_395B_ff},
    {0x3960, 0xff, sOdDs_3960_ff},
    {0x3968, 0xff, sOdDs_3968_ff},
    {0x3969, 0xff, sOdDs_3969_ff},
    {0x396A, 0xff, sOdDs_396A_ff},
    {0x3A00, 0x00, sOdDs_NoOfEntries},
    {0x3A00, 0x01, sOdDs_Angle},
    {0x3A00, 0x02, sOdDs_Turns},
    {0x3A00, 0xff, sOdDs_FbFb},
    {0x3A01, 0x00, sOdDs_NoOfEntries},
    {0x3A01, 0x01, sOdDs_Angle},
    {0x3A01, 0x02, sOdDs_Turns},
    {0x3A01, 0xff, sOdDs_FbPosOffset},
    {0x3A02, 0xff, sOdDs_3A02_ff},
    {0x3A03, 0xff, sOdDs_3A03_ff},
    {0x3A04, 0xff, sOdDs_3A04_ff},
    {0x3A05, 0xff, sOdDs_3A05_ff},
    {0x3A06, 0xff, sOdDs_3A06_ff},
    {0x3A07, 0xff, sOdDs_3A07_ff},
    {0x3A10, 0x00, sOdDs_NoOfEntries},
    {0x3A10, 0x01, sOdDs_Angle},
    {0x3A10, 0x02, sOdDs_Turns},
    {0x3A10, 0xff, sOdDs_AbsFb},
    {0x3A11, 0x00, sOdDs_NoOfEntries},
    {0x3A11, 0x01, sOdDs_Angle},
    {0x3A11, 0x02, sOdDs_Turns},
    {0x3A11, 0xff, sOdDs_AbsPosOffset},
    {0x3A12, 0xff, sOdDs_3A12_ff},
    {0x3A13, 0xff, sOdDs_3A13_ff},
    {0x3A14, 0xff, sOdDs_3A14_ff},
    {0x3A15, 0xff, sOdDs_3A15_ff},
    {0x3A16, 0xff, sOdDs_AbsFb},
    {0x3A17, 0xff, sOdDs_AbsPosOffset},
    {0x3A20, 0x00, sOdDs_NoOfEntries},
    {0x3A20, 0x01, sOdDs_Angle},
    {0x3A20, 0x02, sOdDs_Turns},
    {0x3A20, 0xff, sOdDs_RelFb},
    {0x3A21, 0x00, sOdDs_NoOfEntries},
    {0x3A21, 0x01, sOdDs_Angle},
    {0x3A21, 0x02, sOdDs_Turns},
    {0x3A21, 0xff, sOdDs_RelPosOffset},
    {0x3A22, 0xff, sOdDs_3A22_ff},
    {0x3A23, 0xff, sOdDs_3A23_ff},
    {0x3A24, 0xff, sOdDs_3A24_ff},
    {0x3A25, 0xff, sOdDs_3A25_ff},
    {0x3A26, 0xff, sOdDs_RelFb},
    {0x3A27, 0xff, sOdDs_RelPosOffset},
    {0x3A30, 0x00, sOdDs_NoOfEntries},
    {0x3A30, 0x01, sOdDs_Angle},
    {0x3A30, 0x02, sOdDs_Turns},
    {0x3A30, 0xff, sOdDs_AuxFb},
    {0x3A31, 0x00, sOdDs_NoOfEntries},
    {0x3A31, 0x01, sOdDs_Angle},
    {0x3A31, 0x02, sOdDs_Turns},
    {0x3A31, 0xff, sOdDs_AuxPosOffset},
    {0x3A32, 0xff, sOdDs_3A32_ff},
    {0x3A33, 0xff, sOdDs_3A33_ff},
    {0x3A34, 0xff, sOdDs_3A34_ff},
    {0x3A35, 0xff, sOdDs_3A35_ff},
    {0x3A36, 0xff, sOdDs_AuxFb},
    {0x3A37, 0xff, sOdDs_AuxPosOffset},
    {0x3A40, 0x00, sOdDs_NoOfEntries},
    {0x3A40, 0x01, sOdDs_Angle},
    {0x3A40, 0x02, sOdDs_Turns},
    {0x3A40, 0xff, sOdDs_FbFb},
    {0x3A41, 0x00, sOdDs_NoOfEntries},
    {0x3A41, 0x01, sOdDs_Angle},
    {0x3A41, 0x02, sOdDs_Turns},
    {0x3A41, 0xff, sOdDs_FbPosOffset},
    {0x3A43, 0xff, sOdDs_3A43_ff},
    {0x3A44, 0xff, sOdDs_3A44_ff},
    {0x3A45, 0xff, sOdDs_3A45_ff},
    {0x3A46, 0xff, sOdDs_FbFb},
    {0x3A47, 0xff, sOdDs_FbPosOffset},
    {0x3F00, 0xff, sOdDs_3F00_ff},
    {0x3F01, 0xff, sOdDs_3F01_ff},
    {0x3F02, 0xff, sOdDs_3F02_ff},
    {0x3F03, 0xff, sOdDs_3F03_ff},
    {0x3F04, 0xff, sOdDs_3F04_ff},
    {0x3F05, 0xff, sOdDs_3F05_ff},
    {0x3F06, 0xff, sOdDs_3F06_ff},
    {0x3F07, 0xff, sOdDs_3F07_ff},
#ifdef _HW_CT
    {0x3F10, 0x00, sOdDs_3F10_00},
    {0x3F10, 0x01, sOdDs_3F10_01},
    {0x3F10, 0x02, sOdDs_3F10_02},
    {0x3F11, 0x00, sOdDs_3F11_00},
    {0x3F11, 0x01, sOdDs_3F11_01},
#endif
    {0x5209, 0xff, sOdDs_5209_ff},
    {0x5700, 0xff, sOdDs_5700_ff},
    {0x5701, 0xff, sOdDs_5701_ff},
    {0x5702, 0xff, sOdDs_5702_ff},
    {0x5703, 0xff, sOdDs_5703_ff},
    {0x5709, 0xff, sOdDs_5709_ff},
    {0x570A, 0xff, sOdDs_570A_ff},
    {0x5710, 0xff, sOdDs_5710_ff},
    {0x5711, 0xff, sOdDs_5711_ff},
    {0x5712, 0xff, sOdDs_5712_ff},
    {0x5713, 0xff, sOdDs_5713_ff},
    {0x5720, 0xff, sOdDs_5720_ff},
    {0x5721, 0xff, sOdDs_5721_ff},
    {0x5722, 0xff, sOdDs_5722_ff},
    {0x5723, 0xff, sOdDs_5723_ff},
    {0x5724, 0xff, sOdDs_5724_ff},
    {0x5725, 0xff, sOdDs_5725_ff},
    {0x5726, 0xff, sOdDs_5726_ff},
    {0x5727, 0xff, sOdDs_5727_ff},
    {0x5728, 0xff, sOdDs_5728_ff},
    {0x5729, 0xff, sOdDs_5729_ff},
    {0x572A, 0xff, sOdDs_572A_ff},
    {0x5730, 0xff, sOdDs_5730_ff},
    {0x5780, 0x00, sOdDs_NoOfEntries},
    {0x5780, 0x01, sOdDs_5780_01},
    {0x5780, 0x02, sOdDs_5780_02},
    {0x5780, 0x03, sOdDs_5780_03},
    {0x5780, 0x04, sOdDs_5780_04},
    {0x5780, 0xff, sOdDs_5780_ff},
    {0x5783, 0x00, sOdDs_NoOfEntries},
    {0x5783, 0x01, sOdDs_5783_01},
    {0x5783, 0x02, sOdDs_5783_02},
    {0x5783, 0x03, sOdDs_5783_03},
    {0x5783, 0x04, sOdDs_5783_04},
    {0x5783, 0xff, sOdDs_5783_ff},
    {0x603f, 0xff, sOdDs_603f_ff},
    {0x6040, 0xff, sOdDs_6040_ff},
    {0x6041, 0xff, sOdDs_6041_ff},
    {0x605a, 0xff, sOdDs_605a_ff},
    {0x605b, 0xff, sOdDs_605b_ff},
    {0x605c, 0xff, sOdDs_605c_ff},
    {0x605d, 0xff, sOdDs_605d_ff},
    {0x605e, 0xff, sOdDs_605e_ff},
    {0x6060, 0xff, sOdDs_6060_ff},
    {0x6061, 0xff, sOdDs_6061_ff},
    {0x6064, 0xff, sOdDs_6064_ff},
    {0x6065, 0xff, sOdDs_6065_ff},
    {0x6066, 0xff, sOdDs_6066_ff},
    {0x6067, 0xff, sOdDs_6067_ff},
    {0x6068, 0xff, sOdDs_6068_ff},
    {0x6069, 0xff, sOdDs_6069_ff},
    {0x606b, 0xff, sOdDs_606b_ff},
    {0x606c, 0xff, sOdDs_606c_ff},
    {0x606d, 0xff, sOdDs_606d_ff},
    {0x606e, 0xff, sOdDs_606e_ff},
    {0x606f, 0xff, sOdDs_606f_ff},
    {0x6070, 0xff, sOdDs_6070_ff},
    {0x6071, 0xff, sOdDs_6071_ff},
    {0x6076, 0xff, sOdDs_6076_ff},
    {0x6077, 0xff, sOdDs_6077_ff},
    {0x607a, 0xff, sOdDs_607a_ff},
    {0x607c, 0xff, sOdDs_607c_ff},
    {0x6081, 0xff, sOdDs_6081_ff},
    {0x6082, 0xff, sOdDs_6082_ff},
    {0x6083, 0xff, sOdDs_6083_ff},
    {0x6084, 0xff, sOdDs_6084_ff},
    {0x6085, 0xff, sOdDs_6085_ff},
    {0x6098, 0xff, sOdDs_6098_ff},
    {0x6099, 0X00, sOdDs_NoOfMappedObj},
    {0x6099, 0x01, sOdDs_6099_01},
    {0x6099, 0x02, sOdDs_6099_02},
    {0x6099, 0xff, sOdDs_6099_ff},
    {0x609A, 0xff, sOdDs_609A_ff},
    {0x60c1, 0x00, sOdDs_NoOfMappedObj},
    {0x60c1, 0x01, sOdDs_60c1_01},
    {0x60c1, 0xff, sOdDs_60c1_ff},
    {0x60c2, 0x00, sOdDs_NoOfMappedObj},
    {0x60c2, 0x01, sOdDs_60c2_01},
    {0x60c2, 0x02, sOdDs_60c2_02},
    {0x60c2, 0xff, sOdDs_60c2_ff},
    {0x60f4, 0xff, sOdDs_60f4_ff},
    {0x60ff, 0xff, sOdDs_60ff_ff},
    {0x6502, 0xff, sOdDs_6502_ff},
};

const UWORD uwCanOpenParamInfoCount=sizeof(hpsCanOpenParamInfo)/sizeof(CANOPENCOMDB_INFODESCR);
