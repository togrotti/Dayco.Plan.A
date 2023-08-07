/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CanOpenDs301Externals.h                                    */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : External callbacks and data structures for handling        */
/*               platform-specific tasks                                    */
/*                                                                          */
/****************************************************************************/

#ifndef _CANOPENDS301EXTERNALS_H
#define _CANOPENDS301EXTERNALS_H

#include "common\CommonUtility.h"
#include "system\SysAppConfig.h"

//***************************************************************************
// Data types between DS301 and local

typedef BOOL					BOOLEAN;
typedef SBYTE					INTEGER8;
typedef UBYTE       			UNSIGNED8;
typedef	SWORD					INTEGER16;
typedef	UWORD					UNSIGNED16;
typedef	SLONG					INTEGER32;
typedef	ULONG					UNSIGNED32;
typedef SQWRD                   INTEGER64;
typedef UQWRD                   UNSIGNED64;
typedef	FLOAT					REAL32;
typedef	char					VISIBLE_STRING;
typedef	unsigned char			OCTET_STRING;

typedef void *					PVOID;
typedef SBYTE *					PINTEGER8;
typedef UBYTE *			        PUNSIGNED8;
typedef	SWORD *					PINTEGER16;
typedef	UWORD *					PUNSIGNED16;
typedef	SLONG *					PINTEGER32;
typedef	ULONG *					PUNSIGNED32;
typedef SQWRD *                 PINTEGER64;
typedef UQWRD *                 PUNSIGNED64;
typedef	REAL32 *				PREAL32;
typedef	char *					PVISIBLE_STRING;
typedef	unsigned char *			POCTET_STRING;

//***************************************************************************
// Data structures

#ifndef _CANDRV_EXTD
typedef UWORD CANDRV_ID;    // ID in bit0:10
#else
#define CANDRV_ID_EXTD      (0x40000000)
typedef ULONG CANDRV_ID;    // ID in bit0:10 oppure bit0:28
#endif

typedef struct
{
    struct
    {
    	BOOL bNewRxCob;
    	BOOL bOverRun;
    	BOOL bRTR;
    	BOOL bRxHiPriority;
    	BOOL bTxOK;
    	BOOL bTxBlock;
    	BOOL bInhibited;
    	BOOL bTxReq;
    	BOOL bTxBlockErr;
    	BOOL bTxUpdating;
    	BOOL bTxQueuing;
    	BOOL bRxVHPR;
    	BOOL bTxWCallback;
    } __attribute__((aligned(4)))flags;

	//	UWORD	    flags;	    // vedi flags COB
	CANDRV_ID   id;			// ID + extended flag
	UWORD	    len;		// lunghezza messaggio in byte (0-8)
	UWORD	    d[4];		// dati, da accedere solo tramite f. conv. dati
	UWORD	    inhibit;	// inhibit time (100 usec risoluz), solo x TX COB
							// ts_hires (can bit time), solo x RX COB senza TSTAMP
	ULONG	    usrdata;	// dati utente
} CANDRV_COB;

typedef CANDRV_COB *        CANDRV_PCOB;

//***************************************************************************
// Memory qualifier

#define DS301_MEMQ_PARAM                

//***************************************************************************
// Macros

#define DS301_MON_SETBIT_8(x,y)         fast_atomic_set_bits(x,y)
#define DS301_MON_CLEARBIT_8(x,y)       fast_atomic_clear_bits(x,y)

#define DS301_MON_SETBIT_16(x,y)        fast_atomic_set_bits(x,y)
#define DS301_MON_CLEARBIT_16(x,y)      fast_atomic_clear_bits(x,y)

#define DS301_MON_SETBIT_32(x,y)        atomic_long_set_bits(x,y)
#define DS301_MON_CLEARBIT_32(x,y)      atomic_long_clear_bits(x,y)

#define DS301_MON_MOVE_8(x,y)           (x)=(y)
#define DS301_MON_MOVE_16(x,y)          (x)=(y)
#define DS301_MON_MOVE_32(x,y)          (atomic_move(&(x), &(y), sizeof(ULONG)))

//***************************************************************************
// SDO Transaction data structure

typedef struct
{
    union
    {
        struct
        {
            UNSIGNED16  bDownload:1;
            UNSIGNED16  bUpload:1;
            UNSIGNED16  bInit:1;
            UNSIGNED16  bData:1;
            UNSIGNED16  bLast:1;
            UNSIGNED16  bEnd:1;
            UNSIGNED16  bAbort:1;
        } b;
        UNSIGNED16      w;
    } f;
    UNSIGNED16  uwIndex;
    UNSIGNED8   ubSubIndex;
    PUNSIGNED8  pubDataBuffer;
    UNSIGNED32  ulDataSize;
    UNSIGNED8   ubContext[16];
} DS301_SDOTRANSACTION;

//***************************************************************************
// System status return codes

#define DS301_FULLYOPERATIVE            (1)
#define DS301_NOTOPERATIVE              (0)

//***************************************************************************
// PDO defines

#ifdef	CFG_DS301_PDO
#define	DS301_PDO_TX_TOT			            CFG_DS301_PDO_TX_TOT
#define	DS301_PDO_RX_TOT			            CFG_DS301_PDO_RX_TOT
#define	DS301_PDO_MAXDATAOBJECT		            CFG_DS301_MAXDATAOBJECT
#endif

//***************************************************************************
// Callbacks data structure

typedef struct
{
    ULONG   (* pfSdoTransactions)(DS301_SDOTRANSACTION *);
    void    (* pfWdtClear)(void);
    UWORD   (* pfSysStatus)(void);
    void    (* pfNmtEvent)(UBYTE);
    BOOL    (* pfGetAlarmEntry)(ULONG *, UWORD *, UBYTE *);
    void    (* pfCommFault)(UWORD);
    BOOL    (* pfExecLssStore)(void);
    void    (* pfSyncEvent)(CANDRV_COB *);
    BOOL    (* pfPdoCreate)(UNSIGNED8);
    void    (* pfPdoDestroy)(UNSIGNED8);
    void    (* pfSetDefaultPdoCobID)(UNSIGNED8, UNSIGNED8);
} DS301_CALLBACKS;

//***************************************************************************
// Identity data structure

#ifdef CFG_DS301_IDENTITY
typedef struct
{
    UNSIGNED32       ulDeviceType;
    UNSIGNED32       ulVendorID;
    UNSIGNED32       ulProductCode;
    UNSIGNED32       ulRevision;
    UNSIGNED32       ulSerialNumber;
    PVISIBLE_STRING  psManufacturerName;
    PVISIBLE_STRING  psDeviceNameBase;
    PVISIBLE_STRING  psSWVersion;
} DS301_IDENTITY;
#endif

//***************************************************************************
// PDO parameters data structures

#ifdef	CFG_DS301_PDO
typedef union
{
    UNSIGNED32                      dMap;
    struct
    {
        UNSIGNED8                   bLength;
        UNSIGNED8                   bSubIndex;
        UNSIGNED16                  wIndex;
    } f;
} DS301_PDO_MAP;


typedef struct
{
    UNSIGNED32                      tCobId;
    UNSIGNED8                       bType;
    UNSIGNED8                       bNoMapped;
	UNSIGNED16						wInhibitTime;   // 100 usec
    UNSIGNED8                       bSyncStartValue;
    DS301_PDO_MAP                   tMap[DS301_PDO_MAXDATAOBJECT];
} DS301_PDO_PARAM;
#endif

//***************************************************************************
// Globals

    // those must be externally defined from main application
extern const DS301_CALLBACKS tDs301CallBacks;

#ifdef CFG_DS301_IDENTITY
extern DS301_IDENTITY tDs301Identity;
#endif

#ifdef	CFG_DS301_PDO
extern DS301_PDO_PARAM DS301_MEMQ_PARAM tDs301PdoParamTx[CFG_DS301_PDO_TX_TOT];
extern const DS301_PDO_PARAM DS301_MEMQ_PARAM tDs301PdoDefParamTx[CFG_DS301_PDO_TX_TOT];
extern DS301_PDO_PARAM DS301_MEMQ_PARAM tDs301PdoParamRx[CFG_DS301_PDO_RX_TOT];
extern const DS301_PDO_PARAM DS301_MEMQ_PARAM tDs301PdoDefParamRx[CFG_DS301_PDO_RX_TOT];
#endif
                                             
#endif
