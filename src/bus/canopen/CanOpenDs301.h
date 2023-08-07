/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CanOpenDs301.h                                             */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : CiA CANOpen DS301, DS302 slave only                        */
/*                                                                          */
/****************************************************************************/

#ifndef _CANOPENDS301_H
#define _CANOPENDS301_H

#include "common\CommonDefines.h"
#include "common\CommonParamDB.h"
#include "system\SysAppConfig.h"
#include "CanOpenDs301Externals.h"

//***************************************************************************
// Definizione costanti/configurazione
#ifdef CFG_DS301_DEFTIMING
#define	DS301_DEFTIMING				CFG_DS301_DEFTIMING
#endif
#ifdef CFG_DS301_DEFNODEID
#define	DS301_DEFNODEID				CFG_DS301_DEFNODEID
#endif
#define	DS301_ERRPROT_MINTIME		4			// msec
#define	DS301_ERRPROT_MAXTIME		32000		// msec

//***************************************************************************
// Definizioni COB-ID

typedef UNSIGNED32					DS301_COBIDENTRY;

#define	DS301_COBIDMASK					        (0x000007ff)
#define	DS301_COBIDRESET29BIT				    (0x1ffff800)
#define	DS301_CREATECOBIDENTRY(vl,id)		    (((UNSIGNED32)(vl)<<31)|(id))
#define	DS301_GETCANIDS(us)				        ((UNSIGNED16)((us)&(DS301_COBIDMASK)))
#define	DS301_GETCANID(us)				        ((us)&(DS301_COBIDMASK))
#define	DS301_COBIDVALID(us)				    (!((us)&(1l<<31)))

#define	DS301_CREATEPDOCOBIDENTRY(vl,rtr,id)    (((UNSIGNED32)(vl)<<31)|((UNSIGNED32)(rtr)<<30)|(id))
#define	DS301_COBIDRTR(us)				        (!((us)&(1l<<30)))
#define	DS301_COBID29BITID(us)			        ((us)&(1l<<29))

//***************************************************************************
// COB-ID predefiniti

#define	DS301_COBID_NMT				0x0000
#define	DS301_COBID_SYNC			0X0080
#define	DS301_COBID_TIME_STAMP		0X0100
#define	DS301_COBID_EMERGENCY		0X0080
#define	DS301_COBID_PDO_TX			0x0180
#define	DS301_COBID_PDO_RX			0x0200
#define	DS301_COBID_PDO_ID_LOW	    0x0180
#define	DS301_COBID_PDO_ID_HIGH	    0x057F
#define	DS301_COBID_SDO_TX			0X0580
#define	DS301_COBID_SDO_RX			0X0600
#define	DS301_COBID_NMT_ERRCTRL		0X0700
#define	DS301_COBID_LSSRX			0x07e5
#define	DS301_COBID_LSSTX			0x07e4

//***************************************************************************
// NMT commands/status

#define	DS301_NMT_NULL						0
#define	DS301_NMT_START						1
#define	DS301_NMT_STOP						2
#define	DS301_NMT_ENTER_PREOPERATIONAL		128
#define	DS301_NMT_RESET_NODE				129
#define	DS301_NMT_RESET_COMMUNICATION		130

#define	DS301_NMT_STATUS_BOOTUP				0
#define	DS301_NMT_STATUS_STOPPED			4
#define	DS301_NMT_STATUS_PREOPERATIONAL		127
#define	DS301_NMT_STATUS_OPERATIONAL		5

//***************************************************************************
// SDO abort code: i primi due sono utilizzati nelle funzioni dispatch

#define DS301_SDOOK											0x00000000l

#define	DS301_SDOABORT_TOGGLEBIT							0x05030000
#define	DS301_SDOABORT_TIMEOUT								0x05040000
#define	DS301_SDOABORT_INVALIDCOMMAND						0x05040001
#define	DS301_SDOABORT_INVALIDBLKSIZE						0x05040002
#define	DS301_SDOABORT_INVALIDSEQNUM						0x05040003
#define	DS301_SDOABORT_CRCERROR								0x05040004
#define	DS301_SDOABORT_OUTOFMEMORY							0x05040005
#define	DS301_SDOABORT_UNSUPPORTEDACCESS					0x06010000
#define	DS301_SDOABORT_DENYREAD								0x06010001
#define	DS301_SDOABORT_DENYWRITE							0x06010002
#define	DS301_SDOABORT_OBJECTNOTFOUND						0x06020000
#define	DS301_SDOABORT_OBJECTNOTMAPPABLEINPDO				0x06040041
#define	DS301_SDOABORT_PDOLENGTHEXCEEDED					0x06040042
#define	DS301_SDOABORT_GENERALPARAMINCOMPATIBILITY			0x06040043
#define	DS301_SDOABORT_GENERALDEVINCOMPATIBILITY			0x06040047
#define	DS301_SDOABORT_HARDWAREFAIL							0x06060000
#define	DS301_SDOABORT_PARAMLENGHTUNMATCHED					0x06070010
#define	DS301_SDOABORT_PARAMLENGHTTOOHIGH					0x06070012
#define	DS301_SDOABORT_PARAMLENGHTTOOLOW					0x06070013
#define	DS301_SDOABORT_INVALIDSUBINDEX						0x06090011
#define	DS301_SDOABORT_PARAMVALUEEXCEEDED					0x06090030
#define	DS301_SDOABORT_PARAMVALUETOOHIGH					0x06090031
#define	DS301_SDOABORT_PARAMVALUETOOLOW						0x06090032
#define	DS301_SDOABORT_MINGREATERTHANMAX					0x06090036
#define	DS301_SDOABORT_GENERALERROR							0x08000000
#define	DS301_SDOABORT_DATACANNOTSTORED						0x08000020
#define	DS301_SDOABORT_DATACANNOTSTOREDFORLOCAL				0x08000021
#define	DS301_SDOABORT_DATACANNOTSTOREDFORSTATE				0x08000022
#define	DS301_SDOABORT_DYNAMICOBJDICTGENERATIONFAIL			0x08000023

//***************************************************************************
// Data types

#define DS301_ODC_NULL                                      0x0000
#define DS301_ODC_BOOLEAN                                   0x0001
#define DS301_ODC_INTEGER8                                  0x0002
#define DS301_ODC_INTEGER16                                 0x0003
#define DS301_ODC_INTEGER32                                 0x0004
#define DS301_ODC_INTEGER64                                 0x0015
#define DS301_ODC_UNSIGNED8                                 0x0005
#define DS301_ODC_UNSIGNED16                                0x0006
#define DS301_ODC_UNSIGNED32                                0x0007
#define DS301_ODC_UNSIGNED64                                0x001b
#define DS301_ODC_REAL32                                    0x0008
#define DS301_ODC_VISIBLE_STRING                            0x0009
#define DS301_ODC_OCTET_STRING                              0x000a
#define DS301_ODC_DOMAIN                                    0x000f
#define DS301_ODC_MANUFACTURER                              0x0040

//***************************************************************************
// Costanti unita' di misura

#define	DS301_UM_POS_LIN									0x01	// [m]
#define	DS301_UM_POS_ANGLE_RAD								0x10	// [rad]
#define	DS301_UM_POS_ANGLE_DEG								0x41	// [°]
#define	DS301_UM_POS_INTERNAL								0xff	// app specific

#define	DS301_UM_VEL_LIN_MS									0xa6	// [m/s]
#define	DS301_UM_VEL_LIN_MM									0xa7	// [m/min]
#define	DS301_UM_VEL_ANG_RS									0xa3	// [rev/s]
#define	DS301_UM_VEL_ANG_RM									0xa4	// [rev/min]
#define	DS301_UM_VEL_INTERNAL								0xff	// app specific

#define	DS301_UM_ACC_LIN									0xa6	// [m/s^2]
#define	DS301_UM_ACC_ANGLE									0xa3	// [rev/s^2]
#define	DS301_UM_ACC_INTERNAL								0xff	// app specific

//***************************************************************************
// Codici errori definiti nell'error register (1001h)

#define	DS301_REGERR_GENERIC								0x01
#define	DS301_REGERR_CURRENT								0x02
#define	DS301_REGERR_VOLTAGE								0x04
#define	DS301_REGERR_TEMPERATURE							0x08
#define	DS301_REGERR_COMMUNICATION							0x10
#define	DS301_REGERR_DEVICESPECIFIC							0x20
#define	DS301_REGERR_MANUFACTURERSPECIFIC					0x80

//***************************************************************************
// Emergency error codes base

#define	DS301_EMCYCODE_RESET								0x0000
#define	DS301_EMCYCODE_GENERIC								0x1000
#define	DS301_EMCYCODE_CURRENT								0x2000
#define	DS301_EMCYCODE_VOLTAGE								0x3000
#define	DS301_EMCYCODE_TEMPERATURE							0x4000
#define	DS301_EMCYCODE_DEVICEHW								0x5000
#define	DS301_EMCYCODE_DEVICESW								0x6000
#define	DS301_EMCYCODE_ADDITIONALMODULES					0x7000
#define	DS301_EMCYCODE_MONITORING							0x8000
#define	DS301_EMCYCODE_MON_COMMUNICATION					0x8100
#define	DS301_EMCYCODE_MON_CANHWOVERRUN						0x8110
#define	DS301_EMCYCODE_MON_CANSWOVERRUN						0x8111
#define	DS301_EMCYCODE_MON_CANTXOVERRUN						0x8112
#define	DS301_EMCYCODE_MON_CANERRPASV						0x8120
#define	DS301_EMCYCODE_MON_LIFEHEARTBEAT					0x8130
#define	DS301_EMCYCODE_MON_CANBUSOFFRECOVER					0x8140
#define	DS301_EMCYCODE_MON_TRANSMITCOBID					0x8150
#define	DS301_EMCYCODE_PROTOCOL								0x8200
#define	DS301_EMCYCODE_PROT_PDONOTPROCESSEDDUELENGTH		0x8210
#define	DS301_EMCYCODE_PROT_PDOLENGTH						0x8220
#define	DS301_EMCYCODE_COMMUNICATIONERROR					0x8700
#define	DS301_EMCYCODE_EXTERNAL								0x9000
#define	DS301_EMCYCODE_ADDITIONALFUNCTION					0xf000
#define	DS301_EMCYCODE_DEVICESPECIFIC						0xff00

//***************************************************************************
// Emergency error codes application specific

#define	DS301_EMCYCODE_GUARDHEARTBEAT_WRONGPARAMS			0x8201
#define	DS301_EMCYCODE_EMCY_WRONGCOBID  					0x8202
#define	DS301_EMCYCODE_SYNC_WRONGCOBID  					0x8203

//***************************************************************************
// Codici errori relativi alla comunicazione

#define	DS301_COMMERR_CAN_HWOVERRUN 						0x0001
#define	DS301_COMMERR_CAN_PASSIVE    						0x0002
#define	DS301_COMMERR_CAN_BUSOFF    						0x0004
#define	DS301_COMMERR_GUARDHEARTBEAT_WRONGPARAMS 			0x0008
#define	DS301_COMMERR_GUARDHEARTBEAT						0x0010
#define	DS301_COMMERR_PDOLENGHTERROR						0x0020
#define	DS301_COMMERR_CANOVERRUN							0x0080
#define DS301_COMMERR_EMCY_WRONGCOBID                       0x0100
#define	DS301_COMMERR_CAN_EXITFROMBUSOFF    				0x0200
#define DS301_COMMERR_SYNC_WRONGCOBID                       0x0400
#define	DS301_COMMERR_CAN_TXOVERRUN 						0x0800
#define	DS301_COMMERR_SYNCERROR      						0x1000

//***************************************************************************
// Definizioni gestione PDO

#define	DS301_PDO_VALIDSDOINDEX								0x1000
#define	DS301_PDO_DUMMY_LBOUND								ODC_INTEGER8
#define	DS301_PDO_DUMMY_HBOUND								ODC_UNSIGNED32

#define	DS301_PDO_TYPE_ACYCLIC								0
#define	DS301_PDO_TYPE_CYCLIC_LBOUND						1
#define	DS301_PDO_TYPE_CYCLIC_UBOUND						240
#define	DS301_PDO_TYPE_SYNCHRONOUS_RTR						252
#define	DS301_PDO_TYPE_ASYNCHRONOUS_RTR						253
#define	DS301_PDO_TYPE_ASYNCHRONOUS_MANUF					254
#define	DS301_PDO_TYPE_ASYNCHRONOUS_STD						255

#define DS301_PDO_RXMAP_BASEIDX								0x1600
#define DS301_PDO_TXMAP_BASEIDX								0x1A00

//***************************************************************************
// Definizioni gestione LSS

#define	DS301_LSS_NODEID_BEGIN								0x01
#define	DS301_LSS_NODEID_END								0x7f
#define	DS301_LSS_UNCONFIGUREDNODEID						0xff

//***************************************************************************
// Codici di errore di ritorno dall'init

#define DS301_INITERR_NONE                                  0
#define DS301_INITERR_WRONGBAUDRATE                         1
#define DS301_INITERR_WRONGNODEID                           2
#define DS301_INITERR_OTHERS                                3

//***************************************************************************
// Strutture dati globali

    // parametri globali
typedef struct
{
    INTEGER8                        bDummy;

	UNSIGNED8						bLssNodeId;
	UNSIGNED8						bLssTimingIndex;

	DS301_COBIDENTRY 				tSyncCob;
	DS301_COBIDENTRY 				tEmcyCob;
	UNSIGNED16						wEmcyInhibitTime;   // 100 usec

	UNSIGNED16						wGuardTime;         // msec
	UNSIGNED16						wHeartbeatTime;     // msec
	UNSIGNED8 						bLifeTimeFactor;

	UNSIGNED32						dCommCyclePeriod;	// usec
} DS301_PARAM;

//***************************************************************************
// Globals

extern DS301_PARAM DS301_MEMQ_PARAM     tDs301Param;
extern const DS301_PARAM DS301_MEMQ_PARAM tDs301DefParam;

#ifdef CFG_DS301_EMCY
extern volatile UNSIGNED8               bDs301ErrorRegister;
extern volatile UNSIGNED32              dDs301ErrRegManufacturer;
extern volatile UNSIGNED16              wDs301ErrCodeLast;
#endif

//***************************************************************************
// Functions

BOOLEAN     ds301_init(UNSIGNED8 selcanctrl, PUNSIGNED8 errorcode);
void        ds301_loop(void);

BOOL        ds301_check_cob_id(DS301_COBIDENTRY cobid);
BOOL        ds301_check_node_id(UNSIGNED8 nodeid);

void        ds301_faultclear(void);

#ifdef CFG_DS301_NMT
UNSIGNED8   ds301_get_nmt_state(void);
void        ds301_set_nmt_state(UNSIGNED8 bNewStat);
#endif

#endif
