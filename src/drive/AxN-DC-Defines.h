/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : AxN-DC-Defines.h                                           */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : additional (re)definitions controlboard-hw dependant       */
/*                                                                          */
/****************************************************************************/

#ifndef _AXM_DC_DEFINES_H
#define _AXM_DC_DEFINES_H

//***************************************************************************
// Definitions

#ifdef LEDS_OUTPUT
#undef LEDS_OUTPUT
#endif

#ifndef _HW_AXS // AxN-DC defaults
  #define LEDS_GRN_PIN    (MIO_PIN_BASE+48)
  #define LEDS_YEL_PIN    (MIO_PIN_BASE+43)
  #define LEDS_RED_PIN    (MIO_PIN_BASE+8)

#else

#if defined(_HW_AXS_SAIETTA)
  #define LEDS_GRN_PIN    (MIO_PIN_BASE+46)
  #define LEDS_YEL_PIN    (MIO_PIN_BASE+47)
  #define LEDS_RED_PIN    (MIO_PIN_BASE+48)
#elif defined(_HW_AXS_CABI35KW)
  #define LEDS_GRN_PIN    (MIO_PIN_BASE+7)
  #define LEDS_YEL_PIN    (MIO_PIN_BASE+8)
  #define LEDS_RED_PIN    (MIO_PIN_BASE+9)
#elif defined(_HW_AXS_DAYCO22KW)
  #define LEDS_GRN_PIN    (MIO_PIN_BASE+48)
  #define LEDS_YEL_PIN    (MIO_PIN_BASE+43)
  #define LEDS_RED_PIN    (MIO_PIN_BASE+8)
#endif

#endif

#define LEDS_OUTPUT(by) { GPIO_OUT(LEDS_GRN_PIN,(((by)&0x01)==0x01));GPIO_OUT(LEDS_YEL_PIN,(((by)&0x02)==0x02));GPIO_OUT(LEDS_RED_PIN,(((by)&0x04)==0x04)); }

#define LEDS_OUT_GRN    (0x01)
#define LEDS_OUT_YEL    (0x02)
#define LEDS_OUT_RED    (0x04)


#if defined(_HW_AXS_SAIETTA)
#define CAN0_EN_PIN    (MIO_PIN_BASE+xxx)
#elif defined(_HW_AXS_CABI35KW)
#define CAN0_EN_PIN    (MIO_PIN_BASE+0)
#elif defined(_HW_AXS_DAYCO22KW)
#undef CAN0_EN_PIN
#endif


#endif
