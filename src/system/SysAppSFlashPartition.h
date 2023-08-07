/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppSFlashPartition.h                                    */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Partitioning of serial flash                               */
/*                                                                          */
/****************************************************************************/

#ifndef _SYSAPPSFLASHPARTITION_H
#define _SYSAPPSFLASHPARTITION_H

#ifdef _INFINEON
#ifdef _APP_XC
#include "SysAppSFlashPartXC.h"
#else
#include "SysAppSFlashPartXE.h"
#endif
#else
#include "SysAppSFlashPartZYNQ.h"
#endif

#endif
