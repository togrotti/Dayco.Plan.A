/****************************************************************************/
/* Project: Zynq development                                         		*/
/*                                                                          */
/* Copyright © 2016-2022, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : stgap4s.h                                       	        */
/* Author      : Stefano Martino                                            */
/*                                                                          */
/* Description : ST STGAP4S functions	                    				*/
/*                                                                          */
/****************************************************************************/

#include "xuartps.h"

#ifndef BOOL
  typedef unsigned char  BOOL;
#endif



/// <summary>
///
///</summary>
typedef struct
{
union {
	  struct {
		u8	CFG1;
		u8	CFG2;
		u8	CFG3;
		u8	CFG4;
		u8	CFG5;
		u8	CFG6;
		u8	CFG7;

		u8	DIAG1CFGA;
		u8	DIAG1CFGB;
		u8	DIAG2CFGA;
		u8	DIAG2CFGB;
	  } Reg;
	  u8	Regs[11];
  };
	u8 Dummy[1];
} STGAP4S_CONFIG_REGISTER_PARAMS;


/// <summary>
///
///</summary>
typedef struct
{
  union {
	  struct {
		u8	STATUS1;
		u8	STATUS2;
		u8	STATUS3;
		u8	STATUS4;
		u8	STATUS5;
	  } Reg;
	  u8	Regs[5];
  };

} STGAP4S_STATUS_REGISTER_VALUES;

/// <summary>
///
///</summary>
typedef struct
{
    union {
        struct {
        	STGAP4S_STATUS_REGISTER_VALUES	U_H;
        	STGAP4S_STATUS_REGISTER_VALUES	U_L;
        	STGAP4S_STATUS_REGISTER_VALUES	V_H;
        	STGAP4S_STATUS_REGISTER_VALUES	V_L;
        	STGAP4S_STATUS_REGISTER_VALUES	W_H;
        	STGAP4S_STATUS_REGISTER_VALUES	W_L;
        } Phase;
        STGAP4S_STATUS_REGISTER_VALUES Phases[6];
    };

} STGAP4S_SYSTEM_STATUS_REGISTER_VALUES;


/// <summary>
///
///</summary>
extern u8 stgap4sSoftReset;
extern u8 stgap4sResetStatus;
extern u8 stgap4sConfigDrivers;
extern u32 stgap4sTotalBytesTransferred;
extern u32 stgap4sTotalWrongRxCRCerrors;
extern u8 stgap4sConfigValuesIndex;


extern const STGAP4S_CONFIG_REGISTER_PARAMS stgap4sCONFIG_PARAMSdefault;
extern STGAP4S_CONFIG_REGISTER_PARAMS stgap4sCONFIG_PARAMS;
extern STGAP4S_CONFIG_REGISTER_PARAMS stgap4sCONFIG_VALUES;
extern STGAP4S_SYSTEM_STATUS_REGISTER_VALUES stgap4sSTATUS_VALUES;


/// <summary>
///
///</summary>
BOOL STgap4s_Initialize(void);
BOOL STgap4s_SoftReset(void);
BOOL STgap4s_ConfigDrivers(void);
BOOL STgap4s_ReadStatusRegisters(void);
BOOL STgap4s_ReadConfigRegisters(u8 gdidx);
BOOL STgap4s_ResetStatus(void);


