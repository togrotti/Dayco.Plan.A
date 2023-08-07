/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2021 Ningbo Physis Technology Co.,Ltd. All Rights Reserved.  */
/*                                                                          */
/* File        : UserIORT.h                                                 */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/* Description : User analog/digital IO                                     */
/*                                                                          */
/****************************************************************************/

#ifndef _USERIORT_H
#define _USERIORT_H

//***************************************************************************
// Globals for RT

#ifndef _APP_XC
extern bit bUsrIOboardge102;
extern bit bUsrIOboardge200;
extern bit bUsrIOboardge400;
#else
#define bUsrIOboardge102 (TRUE)
#define bUsrIOboardge200 (TRUE)
#define bUsrIOboardge400 (TRUE)
#endif
extern BOOL bUsrIOaninputse;
extern BOOL bUsrIOaninput01lowg;
extern BOOL bUsrIOaninput23lowg;

extern SWORD swUsrIOAnalogInValue[4];
extern UWORD uwUsrMotorBrakeOut;


extern UMCONV_CONV32TO16 sUsrIOAnalogOut0Conv;
extern SWORD swUsrIOAnalogOut0Offset;
extern UMCONV_CONV32TO16 sUsrIOAnalogOut1Conv;
extern SWORD swUsrIOAnalogOut1Offset;

extern void (* pfUsrIOAnInpProcessing)(void);
extern void (* pfUsrIOIOExpProcess)(void);

//***************************************************************************
// Prototypes

BOOL UserIO_RT_task(void);
void UserIO_RT_aninput_2chanstd(void);
void UserIO_RT_aninput_2chandiff(void);
void UserIO_RT_aninput_4chanse(void);

#endif
