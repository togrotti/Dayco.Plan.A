/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : AxM-E-Defines.h                                            */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

/////////////////////////////////////////////////////////////////////////////
//

#ifndef _AXM_E_DEFINES_H
 #define _AXM_E_DEFINES_H

/////////////////////////////////////////////////////////////////////////////
//

#include "core\Gpio.h"
#include "xparameters.h"

/////////////////////////////////////////////////////////////////////////////
//

#ifdef _AXX_SYSAPP
#ifdef _APP_XC
#define SYSPERF_SUPPORTED
#endif
#endif

/////////////////////////////////////////////////////////////////////////////
//

#define XFAMILYIDENT_XE167x   0x26
#define XFAMILYIDENT_XC2788X  0x48
#define XFAMILYIDENT_ZYNQ     XPLAT_ZYNQ

/////////////////////////////////////////////////////////////////////////////
//

#define SYSTEMCLOCK_COMP     (XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ/1000000)       // MHz (667MHz)
#define SYSTEMCLOCK_PERF     (777)       // MHz (777MHz)

/////////////////////////////////////////////////////////////////////////////
//

#define REALTIME_TASK_FREQ    8000      // = 1 sec in fast task ticks = 1000000us / 125
#define SPEED_K_CONVERSION         (((FLOAT)ULONG_MAX_VALUE + 1.0) / ((FLOAT)REALTIME_TASK_FREQ * 60.0)) // [d.u.]=K*[rpm]; K = 8947.85
#define SPEED_RADS_K_CONVERSION    (((FLOAT)ULONG_MAX_VALUE + 1.0) / ((FLOAT)REALTIME_TASK_FREQ * (2.0 * FLOAT_PI))) // [d.u.]=K*[rad/s]; K = 85445.7

//#define CC2_T8_RES_COMP       100
//#define CC2_T8_RES_PERF       128

/////////////////////////////////////////////////////////////////////////////
//

#define SW_RESET_DELAY        200       // msec

/////////////////////////////////////////////////////////////////////////////
//

#ifdef _INFINEON_

#define XE167_DIAG_BUTN1    !P5_IN_P8
#define XE167_DIAG_BUTN2    !P5_IN_P9

#define XE167_PONL_DISABLE  P6_OUT_P3

#define XE167_RESET_BUTTON  (!P2_IN_P4)

#else

#ifndef _HW_AXS
#define XE167_DIAG_BUTN1    (!GPIO_IN(MIO_PIN_BASE+18))
#define XE167_DIAG_BUTN2    (!GPIO_IN(MIO_PIN_BASE+45))

#define RESET_BUTTON_PIN    (MIO_PIN_BASE+14)
#define XE167_RESET_BUTTON  (!GPIO_IN(RESET_BUTTON_PIN))
#endif // _HW_AXS

#endif


// XE167_AD_BOARD_TYPE    ==> AN0
// XE167_AD_15V           ==> AN1
// XE167_AD_VENC          ==> AN3
// XE167_AD_3V3           ==> AN5
// XE167_AD_2V5           ==> AN6
// XE167_AD_1V2           ==> AN7
// XE167_AD_24VAUX        ==> AN12
// XE167_AD_BOARD_TEMP    ==> AN13
// XE167_AD_PWB_THERM     ==> AN14
// XE167_AD_MOTOR_PTC     ==> AN15

/////////////////////////////////////////////////////////////////////////////
//


#ifndef _HW_AXS
#define POWER_FAIL_PIN     (MIO_PIN_BASE+16)

#else // _HW_AXS

#if defined(_HW_AXS_SAIETTA)
  // check SAIETTA ZQ_PFAIL before using POWER_FAIL_IN
  #define POWER_FAIL_PIN     (MIO_PIN_BASE+14)

#elif defined(_HW_AXS_CABI35KW)
  // check CABI ZQ_PFAIL before using POWER_FAIL_IN
  #define POWER_FAIL_PIN     (MIO_PIN_BASE+14)

#elif defined(_HW_AXS_DAYCO22KW)
  #define POWER_FAIL_PIN     (MIO_PIN_BASE+14)

#endif

#endif // _HW_AXS

#define POWER_FAIL_SETMODE Gpio_SetMode(POWER_FAIL_PIN, GPIO_DIR_IN)
#define POWER_FAIL_IN      GPIO_IN(POWER_FAIL_PIN)

/////////////////////////////////////////////////////////////////////////////
//

#ifdef _INFINEON_
#define XE167_FPGA_SYNA   P4_IN_P6
#endif

/////////////////////////////////////////////////////////////////////////////
//

#ifdef _INFINEON_

#define LEDS_OUTPUT( by ) { P3_OUT = by; P4_OUT_P5 = by&0x01; P4_OUT_P7 = by&0x02; P2_OUT_P10 = by&0x20; P7_OUT_P1 = by&0x40;}

#else

#define CAN0_EN_PIN    (MIO_PIN_BASE+44)
#define CAN1_EN_PIN    (MIO_PIN_BASE+39)

#define LEDS_0_PIN    (MIO_PIN_BASE+46)
#define LEDS_1_PIN    (MIO_PIN_BASE+47)
#define LEDS_2_PIN    (MIO_PIN_BASE+48)
#define LEDS_3_PIN    (MIO_PIN_BASE+49)
#define LEDS_4_PIN    (MIO_PIN_BASE+50)
#define LEDS_5_PIN    (MIO_PIN_BASE+51)
#define LEDS_6_PIN    (MIO_PIN_BASE+52)
#define LEDS_7_PIN    (MIO_PIN_BASE+53)

#define LEDS_OUTPUT( by ) { GPIO_OUT(LEDS_0_PIN,(((by)&0x01)==0x01)); \
                            GPIO_OUT(LEDS_1_PIN,(((by)&0x02)==0x02)); \
                            GPIO_OUT(LEDS_2_PIN,(((by)&0x04)==0x04)); \
                            GPIO_OUT(LEDS_3_PIN,(((by)&0x08)==0x08)); \
                            GPIO_OUT(LEDS_4_PIN,(((by)&0x10)==0x10)); \
                            GPIO_OUT(LEDS_5_PIN,(((by)&0x20)==0x20)); \
                            GPIO_OUT(LEDS_6_PIN,(((by)&0x40)==0x40)); \
                            GPIO_OUT(LEDS_7_PIN,(((by)&0x80)==0x80)); }

#endif
/////////////////////////////////////////////////////////////////////////////
// For the AxN-DC defines and/or re-defines hardware configuration

#ifdef _AXX_SYSAPP
#ifdef _HW_DC
#include "AxN-DC-Defines.h"
#endif
#endif

#endif // _AXM_E_DEFINES_H
