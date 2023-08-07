/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : DefineExternals.h                                          */
/* Author      : Fabio Terrile                                              */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Common define for external symbols                         */
/*                                                                          */
/****************************************************************************/

#undef EXTERN
#undef DEFINE_BIT

#ifndef DEFINE_EXTERNALS
    #define EXTERN extern
    #define DEFINE_BIT( Name, Variable, Position ) extern BOOL Name
#else
    #define EXTERN
    #define DEFINE_BIT( Name, Variable, Position ) BOOL Name = Variable ^ Position
#endif
