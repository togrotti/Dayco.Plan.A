/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2016, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : MultiCANController.h                                       */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Driver for Infineon MultiCAN Controller, targeted for      */
/*               CanOpen; support both std 11bit and ext 29bit identifier   */
/*                                                                          */
/****************************************************************************/

#ifndef _MULTICANCONTROLLER_H
#define _MULTICANCONTROLLER_H

//***************************************************************************
// Module configuration
#include "CanOpenDs301Externals.h"


//***************************************************************************
// Defines

#ifndef _HW_AXS
#define CANDRV_MAX_CANNODE  2
#else
#define CANDRV_MAX_CANNODE  1
#endif

//***************************************************************************
// Baud rates (DS301)

#define	CANDRV_10KBPS		8
#define	CANDRV_20KBPS		7
#define	CANDRV_50KBPS		6
#define	CANDRV_100KBPS		5
#define	CANDRV_125KBPS		4
#define	CANDRV_250KBPS		3
#define	CANDRV_500KBPS		2
#define	CANDRV_800KBPS		1
#define	CANDRV_1000KBPS		0

//***************************************************************************
// Flags COBs
/*
#define	CANDRV_F_DEFAULT	0x0000

#define	CANDRV_F_NEWCOB		0x0001		// new RX'ed COB 
#define	CANDRV_F_OVERRUN	0x0002		// rx overrun, previous data lost
#define	CANDRV_F_RTR		0x0004		// RTR RX cob
#define	CANDRV_F_RXHIPRIOR	0x0008		// hi priority rx cob
#define	CANDRV_F_TXOK		0x0010		// tx sent
#define	CANDRV_F_TXBLOCK	0x0020		// wait tx sent before returning
#define	CANDRV_F_INHIBITED	0x0040		// COB inhibited, on inhibited list
#define	CANDRV_F_TXREQ		0x0080		// TX req pending
#define	CANDRV_F_TXBLOCKERR	0x0100		// wait tx also if errors
#define	CANDRV_F_TXUPDATING	0x0200      // TX updating, avoid processing from
                                        // ihnibit list
#define CANDRV_F_TXQUEUING  0x0400      // being processed by can controller
#define	CANDRV_F_RX_VHPR	0x0800		// very hi priority rx cob, only one
                                        // COB admitted per can controller
#define CANDRV_F_TXWCB      0x1000      // callback registered
*/
//***************************************************************************
// Global flags

#define	CANDRV_GF_DEFAULT	0x0000

#define	CANDRV_GF_RUN		0x0001		// Can enabled and running
#define	CANDRV_GF_ERRPASV	0x0002		// error passive mode
#define	CANDRV_GF_BUSOFF	0x0004		// bus-off
#define	CANDRV_GF_HWOVERRUN	0x0008		// hw overrun

#define	CANDRV_PE_BIT1_ERR	0x8000
#define	CANDRV_PE_BIT0_ERR	0x4000
#define	CANDRV_PE_ACK_ERR	0x2000
#define	CANDRV_PE_CRC_ERR	0x1000
#define	CANDRV_PE_FORM_ERR	0x0800
#define	CANDRV_PE_STUFF_ERR	0x0400
#define	CANDRV_PE_TX_WARN	0x0200
#define	CANDRV_PE_RX_WARN	0x0100

//***************************************************************************
// Data structures
/*
#ifndef _CANDRV_EXTD
typedef UWORD CANDRV_ID;    // ID in bit0:10
#else
#define CANDRV_ID_EXTD      (0x40000000)
typedef ULONG CANDRV_ID;    // ID in bit0:10 oppure bit0:28
#endif

typedef struct
{
	UWORD	    flags;	    // vedi flags COB
	CANDRV_ID   id;			// ID + extended flag
	UWORD	    len;		// lunghezza messaggio in byte (0-8)
	UWORD	    d[4];		// dati, da accedere solo tramite f. conv. dati
	UWORD	    inhibit;	// inhibit time (100 usec risoluz), solo x TX COB
							// ts_hires (can bit time), solo x RX COB senza TSTAMP
	ULONG	    usrdata;	// dati utente
} CANDRV_COB;

typedef CANDRV_COB *  CANDRV_PCOB;
*/
//***************************************************************************
// Globals

extern volatile UWORD uwCanDrvGlobalFlags[CANDRV_MAX_CANNODE];
extern volatile UWORD uwCanDrvTimer100us;

//***************************************************************************
// Module access

    // CAN Initialization
BOOL candrv_init(void);

    // Test selected baudrate to check if it's supported here
BOOL candrv_testspeed(SWORD speed);

    // Setup baudrate
BOOL candrv_setspeed(UWORD cannode, SWORD speed);

    // Add RX cob for rx event support
#ifndef _CANDRV_EXTD
BOOL candrv_addrxcob(UWORD cannode, CANDRV_COB * psRxCob, void (* pfCallBack)(CANDRV_COB *));
#else
#define candrv_addrxcob(a,b,c) candrv_addmrxcob(a,b,0x3FFFFFFF,c)
BOOL candrv_addmrxcob(UWORD cannode, CANDRV_COB * psRxCob, ULONG mask, void (* pfCallBack)(CANDRV_COB *));
#endif

    // Delete RX cob for rx event support
BOOL candrv_deleterxcob(UWORD cannode, CANDRV_COB * psRxCob);

    // Request RTR from the actual RX object
BOOL candrv_requestrxcob(UWORD cannode, CANDRV_COB * psRxCob);

    // Node enable
BOOL candrv_run(UWORD cannode);

    // Node disable
BOOL candrv_stop(UWORD cannode);

    // Can status
BOOL candrv_isrunning(UWORD cannode);

    // send cob and/or update data info if inhibited
BOOL candrv_sendcob(UWORD cannode, volatile CANDRV_COB * psTxCob);

    // send cob with callback, inhibit time not managed
BOOL candrv_sendcobwcb(UWORD cannode, volatile CANDRV_COB * psTxCob, void (* pfCallBack)(CANDRV_COB *));

    // set periodic (automatic) and highest priority sending of COB (period = [usec])
BOOL candrv_setperiodiccob(UWORD cannode, CANDRV_ID id, ULONG period);

    // install callback for periodic sending of COB
BOOL candrv_instcallbackperiodiccob(void (* pfCallBack)(UWORD));

//***************************************************************************
// Avoids warning C177: unreachable code - for the data r/w macros

//#pragma warning disable = 177

//***************************************************************************
// Macro for data reading

#define candrv_get_unsigned8(cob,sp)    ((sp)%8?(UBYTE)candrv_get_bitfield(cob,sp,8):\
                                            ((UBYTE *)(cob)->d)[(sp)/8])

//#define candrv_get_integer8(cob,sp)     (SBYTE)(candrv_get_unsigned8(cob,sp))

#define candrv_get_unsigned16(cob,sp)   ((sp)%8?(UWORD)candrv_get_bitfield(cob,sp,16):\
                                            ((sp)%16?(UWORD)(((UBYTE *)((cob)->d))[(sp)/8])|((UWORD)(((UBYTE *)((cob)->d))[(sp)/8+1])<<8):\
                                            (UWORD)((cob)->d)[(sp)/16]))

//#define candrv_get_integer16(cob,sp)    (SWORD)(candrv_get_unsigned16(cob,sp))

#define candrv_get_unsigned32(cob,sp)   ((sp)%8?(ULONG)candrv_get_bitfield(cob,sp,32):\
                                            ((sp)%16?(ULONG)(((UBYTE *)((cob)->d))[(sp)/8])|((ULONG)(((UBYTE *)((cob)->d))[(sp)/8+1])<<8)|\
                                            ((ULONG)(((UBYTE *)((cob)->d))[(sp)/8+2])<<16)|((ULONG)(((UBYTE *)((cob)->d))[(sp)/8+3])<<24):\
                                            *((ULONG *)&(((UWORD *)((cob)->d))[(sp)/16]))))

//#define candrv_get_integer32(cob,sp)    (SLONG)(candrv_get_unsigned32(cob,sp))

ULONG candrv_get_bitfield(CANDRV_PCOB pc,UWORD st,UWORD sz);

//***************************************************************************
// Macro for data writing

#define candrv_put_unsigned8(cob,sp,val)    { if((sp)%8) candrv_put_bitfield(cob,sp,8,(ULONG)(val)); \
                                                else ((UBYTE *)(cob)->d)[(sp)/8]=(UBYTE)(val); }

#define candrv_put_integer8(cob,sp,val)     candrv_put_unsigned8(cob,sp,val)

#define candrv_put_unsigned16(cob,sp,val)   { if((sp)%8) candrv_put_bitfield(cob,sp,16,(ULONG)(val)); \
                                                else if((sp)%16) { ((UBYTE *)(cob)->d)[(sp)/8]=(UBYTE)((val)&0xff); \
                                                ((UBYTE *)(cob)->d)[(sp)/8+1]=(UBYTE)((val)>>8);} \
                                                else ((UWORD *)(cob)->d)[(sp)/16]=(UWORD)(val); }

#define candrv_put_integer16(cob,sp,val)    candrv_put_unsigned16(cob,sp,val)

#define candrv_put_unsigned32(cob,sp,val)   { if((sp)%8) candrv_put_bitfield(cob,sp,32,(ULONG)(val)); \
                                                else if((sp)%16) { ((UBYTE *)(cob)->d)[(sp)/8]=(UBYTE)((val)&0xff); \
                                                ((UBYTE *)(cob)->d)[(sp)/8+1]=(UBYTE)(((val)>>8)&0xff); \
                                                ((UBYTE *)(cob)->d)[(sp)/8+2]=(UBYTE)(((val)>>16)&0xff); \
                                                ((UBYTE *)(cob)->d)[(sp)/8+3]=(UBYTE)((val)>>24);} \
                                                else *((ULONG *)&(((UWORD *)((cob)->d))[(sp)/16]))=(ULONG)(val); }

#define candrv_put_integer32(cob,sp,val)    candrv_put_unsigned32(cob,sp,val)

void candrv_put_bitfield(CANDRV_PCOB pc, UWORD st, UWORD sz, ULONG src);

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
// PLC wrappers

BOOL candrv_plc_setspeed(UWORD cannode, SWORD speed);
#ifndef _CANDRV_EXTD
BOOL candrv_plc_addrxcob(UWORD cannode, CANDRV_COB * psRxCob);
#else
BOOL candrv_plc_addrxcob(UWORD cannode, CANDRV_COB * psRxCob, ULONG mask);
#endif
BOOL candrv_plc_deleterxcob(UWORD cannode, CANDRV_COB * psRxCob);
BOOL candrv_plc_requestrxcob(UWORD cannode, CANDRV_COB * psRxCob);
BOOL candrv_plc_run(UWORD cannode);
BOOL candrv_plc_stop(UWORD cannode);
BOOL candrv_plc_sendcob(UWORD cannode, CANDRV_COB * psTxCob);
BOOL candrv_plc_setperiodiccob(UWORD cannode, CANDRV_ID id, ULONG period);

#endif
