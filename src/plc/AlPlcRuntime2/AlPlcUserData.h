/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : AlPlcUserData.h                                            */
/* Author      : Fabio Terrile                                              */
/*               Axel                                                       */
/*                                                                          */
/* Description : Configuration of PLC and declaration of its resources      */
/*                                                                          */
/****************************************************************************/

#ifndef _PLCUSERDATA_H
#define _PLCUSERDATA_H

//****************************************************************************
// Globals

extern const PLCIEC_TASKS plcTasksFunctions[];
extern const PLCIEC_TASKDEF plcTasksDefs[];
extern const PLCIEC_DBREC plcDataBlocks[];
extern const PLCIEC_FCNREC plcEmbeddedFunctions[];

#ifdef _APP_DEBUG
extern const UWORD plcCntTasksFunctions;
extern const UWORD plcCntTasksDefs;
extern const UWORD plcCntDataBlocks;
extern const UWORD plcCntEmbeddedFunctions;
#endif

#endif
