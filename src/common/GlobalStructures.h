/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : GlobalStructures.h                                         */
/* Author      : Cristiano Tognetti                                         */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _GLOBAL_STRUCTURES_H
 #define _GLOBAL_STRUCTURES_H

 
typedef struct {
  SLONG slIdRef ; /* Id Reference */
  SLONG slIqRef ; /* Iq Reference */
} GLB_IREF ;
 
typedef struct {
  SQWRD sqPostn ; /* Position     (64bit) */
  SLONG slSpeed ; /* Speed        (32bit) */
  SLONG slAccel ; /* Acceleration (32bit) */
} GLB_KINEMATIC_DATA ; 

typedef struct {
  SLONG slMax ;
  SLONG slMin ;
} GLB_TORQUE_LIMIT ;
#endif