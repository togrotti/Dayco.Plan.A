/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2021 Ningbo Physis Technology Co.,Ltd. All Rights Reserved.  */
/*                                                                          */
/* File        : UserIO.h                                                   */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/* Description : User analog/digital IO                                     */
/*                                                                          */
/****************************************************************************/

#ifndef _USERIO_H
#define _USERIO_H

#include "common\DefineExternals.h"
#ifndef _INFINEON_
#include "core\Gpio.h"
#endif
//***************************************************************************
// Defines
#define USRIO_AIN_CHANNELS          7
#define USRIO_AOUT_CHANNELS         6
#define USRIO_DIN_CHANNELS          16
#define USRIO_DOUT_CHANNELS         10

#ifndef _INFINEON_

/******* GPIO BANK *******/
/* 0 - 31,   Bank 0
 * 32 - 53,  Bank 1
 * 54 - 85,  Bank 2
 * 86 - 117, Bank 3 */
/*************************/

#ifndef _HW_AXS

#ifndef _HW_DC

#define USRIO_DI0_PIN               (MIO_PIN_BASE+27)
#define USRIO_DI1_PIN               (MIO_PIN_BASE+28)
#define USRIO_DI2_PIN               (MIO_PIN_BASE+29)
#define USRIO_DI3_PIN               (MIO_PIN_BASE+30)
#define USRIO_DI4_PIN               (MIO_PIN_BASE+31)

#define USRIO_DI_BANK0              XGPIOPS_BANK0
#define USRIO_DI_BANK0_BASE         (27)

#define USRIO_DI5_PIN               (MIO_PIN_BASE+32)
#define USRIO_DI6_PIN               (MIO_PIN_BASE+33)
#define USRIO_DI7_PIN               (MIO_PIN_BASE+34)

#define USRIO_DI_BANK1              XGPIOPS_BANK1
#define USRIO_DI_BANK1_BASE         (5)

#define USRIO_DI_IN             ((~(GPIO_BANK_IN(USRIO_DI_BANK0)>>USRIO_DI_BANK0_BASE) & 0x1F) | \
                                 (~(GPIO_BANK_IN(USRIO_DI_BANK1)<<USRIO_DI_BANK1_BASE) & 0xE0))

#define USRIO_DO_BANK               XGPIOPS_BANK0
#define USRIO_DO_BANK_MASK          (0x03C00000)
#define USRIO_DO_BANK_BASE          (22)

#define USRIO_DO0_PIN               (MIO_PIN_BASE+22)
#define USRIO_DO1_PIN               (MIO_PIN_BASE+23)
#define USRIO_DO2_PIN               (MIO_PIN_BASE+24)
#define USRIO_DO3_PIN               (MIO_PIN_BASE+25)

#define USRIO_DO_OUT(d)             GPIO_BANK_OUT(USRIO_DO_BANK,USRIO_DO_BANK_MASK,(((ULONG)d<<USRIO_DO_BANK_BASE)&USRIO_DO_BANK_MASK))

#else
#define USRIO_DI0_PIN               (MIO_PIN_BASE+24)
#define USRIO_DI1_PIN               (MIO_PIN_BASE+25)
#define USRIO_DI2_PIN               (MIO_PIN_BASE+26)
#define USRIO_DI3_PIN               (MIO_PIN_BASE+27)
#define USRIO_DI4_PIN               (MIO_PIN_BASE+28)
#define USRIO_DI5_PIN               (MIO_PIN_BASE+29)
#define USRIO_DI6_PIN               (MIO_PIN_BASE+30)
#define USRIO_DI7_PIN               (MIO_PIN_BASE+31)

#define USRIO_DI_BANK0              XGPIOPS_BANK0
#define USRIO_DI_BANK0_BASE         (24)

#define USRIO_DI_IN             ((GPIO_BANK_IN(USRIO_DI_BANK0)>>USRIO_DI_BANK0_BASE)&0xFF)//((~(GPIO_BANK_IN(USRIO_DI_BANK0)>>USRIO_DI_BANK0_BASE))&0xFF)

#define USRIO_DO_BANK               XGPIOPS_BANK0
#define USRIO_DO_BANK_MASK          (0xF00000)
#define USRIO_DO_BANK_BASE          (20)

#define USRIO_DO0_PIN               (MIO_PIN_BASE+20)
#define USRIO_DO1_PIN               (MIO_PIN_BASE+21)
#define USRIO_DO2_PIN               (MIO_PIN_BASE+22)
#define USRIO_DO3_PIN               (MIO_PIN_BASE+23)

#define USRIO_DO_OUT(d)             GPIO_BANK_OUT(USRIO_DO_BANK,USRIO_DO_BANK_MASK,(((ULONG)d<<USRIO_DO_BANK_BASE)&USRIO_DO_BANK_MASK))
#endif

#else // _HW_AXS

#if defined(_HW_AXS_SAIETTA)
  #define USRIO_DI_IN                 (~(0x0000))

  #define USRIO_DO0_PIN               (MIO_PIN_BASE+39)
  #define USRIO_DO1_PIN               (MIO_PIN_BASE+44)

  #define USRIO_DO_OUT(d)             GPIO_BANK_OUT(XGPIOPS_BANK1, 0x00002080, (((ULONG)d << 12) & 0x00002000) | (((ULONG)d << 7) & 0x00000080))

#elif defined(_HW_AXS_CABI35KW)
  #define USRIO_DI0_PIN               (MIO_PIN_BASE+34)
  #define USRIO_DI1_PIN               (MIO_PIN_BASE+35)
  #define USRIO_DI2_PIN               (MIO_PIN_BASE+36)

  #define USRIO_DI_IN                 ((GPIO_BANK_IN(XGPIOPS_BANK1) >> 2) & 0x0007)

  #define USRIO_DO0_PIN               (MIO_PIN_BASE+37)
  #define USRIO_DO1_PIN               (MIO_PIN_BASE+38)
  #define USRIO_DO2_PIN               (MIO_PIN_BASE+39)

  #define USRIO_DO_OUT(d)             GPIO_BANK_OUT(XGPIOPS_BANK1, 0x000000e0, (((ULONG)d << 5) & 0x000000e0))

#elif defined(_HW_AXS_DAYCO22KW)
  #define USRIO_DI0_PIN               (MIO_PIN_BASE+24)
  #define USRIO_DI1_PIN               (MIO_PIN_BASE+25)

  #define USRIO_DI_IN                 ((GPIO_BANK_IN(XGPIOPS_BANK0) >> 24) & 0x0003)

// USRIO_DO1_PIN, USRIO_DO2_PIN       sono su EMIO
  #define USRIO_DO0_PIN               (MIO_PIN_BASE+21)

  #define USRIO_DO_OUT(d)             GPIO_BANK_OUT(XGPIOPS_BANK0, 0x00200000, (((ULONG)d << 21) & 0x00200000))

#endif

#endif // _HW_AXS

#endif // _INFINEON_

//***************************************************************************
// Digital I/O

typedef struct
{
        // (0-7 Physical Inputs, 8-15 Exp Physical Inputs)
    UWORD uwInputs;
        // (0-3 Physical Outputs, 4-7 Control Panel Logical Outputs, 8-9 Exp Physical Outputs)
    UWORD uwOutputs;
} USRIO_DIGITALIO;

typedef struct
{
        // (0-7 Physical Inputs, 8-15 Exp Physical Inputs)
    BOOL  bInputs[USRIO_DIN_CHANNELS];
        // (0-3 Physical Outputs, 4-7 Control Panel Logical Outputs, 8-9 Exp Physical Outputs)
    BOOL  bOutputs[USRIO_DOUT_CHANNELS];
} USRIO_PLC_DIGITALIO;

//***************************************************************************
// Analog I/O

typedef struct
{
        // 0-1 Physical Inputs, 2-3 Control Panel Logical or physical, 4-6 Exp Physical Inputs 
    SWORD swInputs[USRIO_AIN_CHANNELS];             // [mV]
        // 0-1 Physical Outputs, 2-3  Control Panel Logical, 4-5 Exp Physical Outputs
    SWORD swOutputs[USRIO_AOUT_CHANNELS];           // [mV]
} USRIO_ANALOGIO;

//***************************************************************************
// Control panel interaction

typedef struct
{
        // basic structure
    USRIO_DIGITALIO sDIO;

        // (0-7 Physical Inputs, 8-15 Exp Physical Inputs)
    UWORD uwDgInForceOn;
    UWORD uwDgInForceOff;

        // (0-3 Physical Outputs, 4-7 Control Panel Logical Outputs, 8-9 Exp Physical Outputs)
    UWORD uwDgOutForceOn;
    UWORD uwDgOutForceOff;

        // 0-1 Physical Inputs, 2-3 Control Panel Logical or physical, 4-6 Exp Physical Inputs 
    UWORD uwAnInSimulation;
        // 0-1 Physical Outputs, 2-3  Control Panel Logical, 4-5 Exp Physical Outputs
    UWORD uwAnOutSimulation;
} USRIO_CPANEL_BASE;


#ifdef _INFINEON_
EXTERN USRIO_CPANEL_BASE bdata sUsrIOCPBase;
#else
extern USRIO_CPANEL_BASE sUsrIOCPBase;
extern USRIO_PLC_DIGITALIO sPlcDIO;
#endif

typedef struct
{
        // basic structure shared on plc
    USRIO_ANALOGIO sAIO;

        // 0-1 Physical Outputs, 2-3  Control Panel Logical, 4-5 Exp Physical Outputs
    SWORD swOutSimulation[USRIO_AOUT_CHANNELS];     // [mV]
} USRIO_CPANEL_DATA;

EXTERN USRIO_CPANEL_DATA sUsrIOCPData;

#ifdef _INFINEON_
typedef struct
{
        // four nibbles, from hi to lo:
        // # AO, # AI, # DO, # DI
    UWORD uwSizes;

        // addresses for data structures readback
    void huge * hpsAdrBase;
    void huge * hpsAdrData;
} USRIO_CPANEL_DEFS;

EXTERN const USRIO_CPANEL_DEFS huge sUserIOCPDefs;
#else
typedef struct
{
        // four nibbles, from hi to lo:
        // # AO, # AI, # DO, # DI
    UWORD uwSizes;

        // addresses for data structures readback
    void * hpsAdrBase;
    void * hpsAdrData;
} USRIO_CPANEL_DEFS;

extern const USRIO_CPANEL_DEFS sUserIOCPDefs;
#endif

//***************************************************************************
// Hardware configuration
#ifdef _INFINEON_
extern UWORD uwUsrIOFlags;
extern UWORD uwUsrIOHwDetect;
#else
typedef struct {
#ifndef _HW_DC
    BOOL b[4];
#else
    BOOL b[1];
#endif // _hw_dc
} USRIOFLAGS;
extern USRIOFLAGS sUsrIOFlags;
typedef struct {
#ifndef _HW_DC
    BOOL b[5];
#else
    BOOL b[1];
#endif // _hw_dc
} USRIOHWDETECT;
extern USRIOHWDETECT sUsrIOHwDetect;
#endif // _infineon_

#ifndef _HW_DC

#ifdef _INFINEON_
DEFINE_BIT( bUsrIOFlgAnInputsSE,    uwUsrIOFlags,  0 );
DEFINE_BIT( bUsrIOFlgAnInput01LowG, uwUsrIOFlags,  1 );
DEFINE_BIT( bUsrIOFlgAnInput23LowG, uwUsrIOFlags,  2 );
DEFINE_BIT( bUsrIOFlgDisableIOExp , uwUsrIOFlags,  3 );
DEFINE_BIT( bUsrIOHwDetSEAvailable,     uwUsrIOHwDetect, 0 );
DEFINE_BIT( bUsrIOHwDetLowGAvailable,   uwUsrIOHwDetect, 1 );
DEFINE_BIT( bUsrIOHwDetIOExpAvailable,  uwUsrIOHwDetect, 2 );
DEFINE_BIT( bUsrIOHwAnInput01LowGEn,    uwUsrIOHwDetect, 3 );
DEFINE_BIT( bUsrIOHwAnInput23LowGEn,    uwUsrIOHwDetect, 4 );
#else
#define bUsrIOFlgAnInputsSE       sUsrIOFlags.b[0]
#define bUsrIOFlgAnInput01LowG    sUsrIOFlags.b[1]
#define bUsrIOFlgAnInput23LowG    sUsrIOFlags.b[2]
#define bUsrIOFlgDisableIOExp     sUsrIOFlags.b[3]
#define bUsrIOHwDetSEAvailable    sUsrIOHwDetect.b[0]
#define bUsrIOHwDetLowGAvailable  sUsrIOHwDetect.b[1]
#define bUsrIOHwDetIOExpAvailable sUsrIOHwDetect.b[2]
#define bUsrIOHwAnInput01LowGEn   sUsrIOHwDetect.b[3]
#define bUsrIOHwAnInput23LowGEn   sUsrIOHwDetect.b[4]
#endif // _infineon_

#else

#ifdef _INFINEON_
DEFINE_BIT( bUsrIOFlgRequest,       uwUsrIOFlags,  0 );
DEFINE_BIT( bUsrIOHwAvailable,      uwUsrIOHwDetect, 0 );
#else
#define bUsrIOFlgRequest  sUsrIOFlags.b[0]
#define bUsrIOHwAvailable sUsrIOHwDetect.b[0]
#endif // _infineon

#endif // _hw_dc

//***************************************************************************
// Initialization entry point

void UserIO_GetHwOpt(UWORD, ULONG *, ULONG *);
BOOL UserIO_Init(UWORD);

BOOL UserIO_RT_task(void);
void UserIO_RT_aninput_2chanstd(void);
void UserIO_RT_aninput_2chandiff(void);
void UserIO_RT_aninput_4chanse(void);

#endif
