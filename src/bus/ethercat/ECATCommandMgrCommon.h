/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2010, Phase Motion Control. All Rights Reserved.             */
/*                                                                          */
/* File        : ECATCommandMgrCommon.h                                     */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : ECAT specific command manager                              */
/*                                                                          */
/****************************************************************************/

#ifndef _ECATCOMMANDMGRCOMMON_H
#define _ECATCOMMANDMGRCOMMON_H

//***************************************************************************
// Slave app callbacks

UWORD APPL_StartMailboxHandler(void);
UWORD APPL_StopMailboxHandler(void);
UWORD APPL_GenerateMapping(void);
UWORD APPL_StartInputHandler(UWORD *pIntMask);
UWORD APPL_StopInputHandler(void);
UWORD APPL_StartOutputHandler(void);
UWORD APPL_StopOutputHandler(void);
void APPL_StartFreeRunTimer(void);

#endif
