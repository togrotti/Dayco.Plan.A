/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2010, Phase Motion Control. All Rights Reserved.             */
/*                                                                          */
/* File        : ecatappl.h                                                 */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : ECAT specific command manager                              */
/*                                                                          */
/****************************************************************************/

#ifndef _MOD_ECATAPPL
#define _MOD_ECATAPPL

//***************************************************************************
// Prototipi

UINT16 APPL_StartMailboxHandler(void);
UINT16 APPL_StopMailboxHandler(void);
UINT16 APPL_GenerateMapping(void);
UINT16 APPL_StartInputHandler(UINT16 *pIntMask);
UINT16 APPL_StopInputHandler(void);
UINT16 APPL_StartOutputHandler(void);
UINT16 APPL_StopOutputHandler(void);
void APPL_StartFreeRunTimer(void);

#endif
