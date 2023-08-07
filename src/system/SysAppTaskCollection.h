/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppTaskCollection.h                                     */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Collection of all applications that will be initialized    */
/*				 and run both in background and realtime					*/
/*                                                                          */
/****************************************************************************/

#ifndef _SYSAPPTASKCOLLECTION_H
#define _SYSAPPTASKCOLLECTION_H

#include "common\TaskScheduler.h"

extern const TASKSCHEDULER_TASK_INIT psSysAppTaskCollection[];
extern const TASKSCHEDULER_TASK_INIT psSysAppTaskAltCollection[];
extern const TASKSCHEDULER_TASK_INIT psSysAppTaskAltCollectWPars[];
extern const TASKSCHEDULER_TASK_INIT psSysAppTaskAltCollectFullCfg[];

#endif
