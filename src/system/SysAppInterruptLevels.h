/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : SysAppInterruptLevels.h                                    */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Interrupt levels for all interrupt defined in the          */
/*               application, as care has to be taken in assigning each one */
/*               different priority                                         */
/*                                                                          */
/****************************************************************************/

#ifndef _SYSAPPINTERRUPTLEVELS_H
#define _SYSAPPINTERRUPTLEVELS_H

#ifdef _INFINEON_
//****************************************************************************
// Macro

#define SYSAPPIL_ICDEF(ilvl,gpx,glvl)           (((gpx)<<8)|((ilvl)<<2)|(glvl))

//****************************************************************************
// Os

#define SYSAPPIL_OS_GPT12E_T5                   SYSAPPIL_ICDEF(1,0,0)
#define SYSAPPIL_OS_GPT12E_T6                   SYSAPPIL_ICDEF(2,0,0)

//****************************************************************************
// SerialPorts

#define SYSAPPIL_SERIALPORTS_U0C0_0             SYSAPPIL_ICDEF(8,0,0)
#define SYSAPPIL_SERIALPORTS_U0C0_1             SYSAPPIL_ICDEF(8,0,1)
#define SYSAPPIL_SERIALPORTS_CC2_CC16           SYSAPPIL_ICDEF(8,0,2)

//****************************************************************************
// ModBusOverSerial

#define SYSAPPIL_MODBUSOVERSERIAL_CC20          SYSAPPIL_ICDEF(8,0,3)

//****************************************************************************
// TaskScheduler

#define SYSAPPIL_TASKSCHEDULER_CC2_CC30_LVL     10
#define SYSAPPIL_TASKSCHEDULER_CC2_CC30         SYSAPPIL_ICDEF(SYSAPPIL_TASKSCHEDULER_CC2_CC30_LVL,0,0)

//****************************************************************************
// MultiCANController

#define SYSAPPIL_MULTICANCTRL_CC24              SYSAPPIL_ICDEF(7,1,1)
#define SYSAPPIL_MULTICANCTRL_NODE0_RXVHI       SYSAPPIL_ICDEF(11,0,3)
#define SYSAPPIL_MULTICANCTRL_NODE0_RXHI        SYSAPPIL_ICDEF(9,0,3)
#define SYSAPPIL_MULTICANCTRL_NODE0_RXLO        SYSAPPIL_ICDEF(7,1,0)
#define SYSAPPIL_MULTICANCTRL_NODE0_TX          SYSAPPIL_ICDEF(7,0,3)
#define SYSAPPIL_MULTICANCTRL_NODE1_RXVHI       SYSAPPIL_ICDEF(11,0,2)
#define SYSAPPIL_MULTICANCTRL_NODE1_RXHI        SYSAPPIL_ICDEF(9,0,2)
#define SYSAPPIL_MULTICANCTRL_NODE1_RXLO        SYSAPPIL_ICDEF(7,0,2)
#define SYSAPPIL_MULTICANCTRL_NODE1_TX          SYSAPPIL_ICDEF(7,0,1)

//****************************************************************************
// Plc

#define SYSAPPIL_PLC_SLOWTASK_CC25              SYSAPPIL_ICDEF(4,0,0)

#else

//****************************************************************************
// Os

// #define SYSAPPIL_OS_GPT12E_T5                   SYSAPPIL_ICDEF(1,0,0)

//****************************************************************************
// SerialPorts

#define SYSAPPIL_SERIALPORTS                    (8<<portPRIORITY_SHIFT)

//****************************************************************************
// ModBusOverSerial

#define SYSAPPIL_MODBUSOVERSERIAL               (8<<portPRIORITY_SHIFT)

//****************************************************************************
// MultiCANController

#define SYSAPPIL_MULTICANCTRL                   (7<<portPRIORITY_SHIFT)

//****************************************************************************
// TaskScheduler

#define SYSAPPIL_TASKSCHEDULER                  ((configMAX_API_CALL_INTERRUPT_PRIORITY)<<portPRIORITY_SHIFT)

//****************************************************************************
// Plc

#define SYSAPPIL_PLC_SLOWTASK                   ((configMAX_API_CALL_INTERRUPT_PRIORITY+1)<<portPRIORITY_SHIFT)

#endif
#endif
