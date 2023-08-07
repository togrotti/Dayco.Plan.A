/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright � 2005,2009, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : MultiCANControllerDefs.h                                   */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Driver for Infineon MultiCAN Controller, targeted for      */
/*               CanOpen                                                    */
/*                                                                          */
/****************************************************************************/
/* SYSTEM CONFIG:                                                           */
/* - Module clock 40MHz                                                     */
/* - Two can controller supported, each with one low priority rx queue and  */
/*   one high priority (hi priority has higher ILVL but do not save MAC)    */
/* - tx queue                                                               */
/* - inhibit time manager with automatic tx for elapsed timer               */
/* - interrupt pointers allocation:                                         */
/*      0: node 0 RX hi priority                                            */
/*      1: node 0 RX lo priority                                            */
/*      2: node 0 TX                                                        */
/*      3: node 0 RX very hi priority                                       */
/*      4: node 1 RX hi priority                                            */
/*      5: node 1 RX lo priority                                            */
/*      6: node 1 TX                                                        */
/*      7: node 1 RX very hi priority                                       */
/*      15: exceptions                                                      */
/****************************************************************************/

#ifndef _MULTICANCONTROLLERDEFS_H
#define _MULTICANCONTROLLERDEFS_H

//***************************************************************************
// Structuring all nodes and all messages for hardware array access
//***************************************************************************
// Data structures

typedef struct
{
    volatile CANDRV_COB  *  psRxCob;
    void                    (* pfCallBack)(CANDRV_COB *);
} RXMBOXPARAMS;
               
typedef struct
{
    UWORD                   uwInhibitTimer;
    volatile CANDRV_COB  *  psTxCob;
} TXMBOXPARAMS;

typedef struct
{
    UWORD                   uwCounter;
    UWORD                   uwRestart;
} SYNCCOBSTAT;
               
//***************************************************************************
// Macros

    // max rx list, for optimized rx irq handler
#define MAX_RXLIST_COBS         16

    // max tx list, for optimized inhibit time management
#define MAX_TXLIST_COBS         16

    // max tx callbacks
#define MAX_TXCALLBACKS         4

#define is_softrescan(cannode)     ((XCanPs_GetStatus(&Can[cannode])&XCANPS_SR_NORMAL_MASK)==0)    //1=CAN不处于激活状态 0=使能
#define	entersoftres(node)	       XCanPs_EnterMode(&Can[cannode],XCANPS_MODE_CONFIG);/*XCanPs_IntrDisable(&Can[cannode], XCANPS_IXR_ALL);*/
#define exitsoftres(cannode)       XCanPs_EnterMode(&Can[cannode], XCANPS_MODE_NORMAL);/*XCanPs_IntrEnable(&Can[cannode], XCANPS_IXR_ALL);*/
    // Panel Busy Flag
#define CAN_PANCTR_BUSY         0x0100

    // inhibit timer
#define TIMER_RELOAD            1000    // every 100us
/*
    // interrupt vectors
#define INHIBIT_TIMER_MANAGER   0x18    // CC2_CC24INT
#define NODE0_RX_HI             0x40    // CAN_0INT
#define NODE0_RX_LO             0x41    // CAN_1INT
#define NODE0_TX                0x42    // CAN_2INT
#define NODE0_RX_VHI            0x43    // CAN_3INT
#define NODE1_RX_HI             0x44    // CAN_4INT
#define NODE1_RX_LO             0x45    // CAN_5INT
#define NODE1_TX                0x46    // CAN_6INT
#define NODE1_RX_VHI            0x47    // CAN_7INT
#define EXCEPTION               0x4f    // CAN_15INT

    // interrupt nodes
#define IN_NODE0_RX_HI          0
#define IN_NODE0_RX_LO          1
#define IN_NODE0_TX             2
#define IN_NODE0_RX_VHI         3
#define IN_NODE1_RX_HI          4
#define IN_NODE1_RX_LO          5
#define IN_NODE1_TX             6
#define IN_NODE1_RX_VHI         7

    // message pending registers
#define MP_NODE0_RX_HI          0
#define MP_NODE0_RX_LO          1
#define MP_NODE0_TX             2
#define MP_NODE0_RX_VHI         3
#define MP_NODE1_RX_HI          4
#define MP_NODE1_RX_LO          5
#define MP_NODE1_TX             6
#define MP_NODE1_RX_VHI         7
*/
    // node selectors
#define NODE0                   0
#define NODE1                   1

    // local flags
#define	CAN_SHUTTINGDOWN	    0x0001
#define CAN_INHIBITENABLED      0x0002
#define CAN_PORTAVAILABLE       0x0004

//***************************************************************************
// Locals (only between multcancontroller modules)

extern RXMBOXPARAMS sCanDrvRxList[CANDRV_MAX_CANNODE][MAX_RXLIST_COBS];
extern SYNCCOBSTAT sCanDrvSyncCobStat[CANDRV_MAX_CANNODE];
extern void (* pfCanDrvSyncCobCallBack)(UWORD);

BOOL candrv_RT_periodiccobprocessing(void);

#endif
