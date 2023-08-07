/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright $)Ao??2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCNetProtocolMaster.c                                  */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : EtherPMC Network Protocol Master Handler                   */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

#include "common\CommonDefines.h"
#include "common\commontypedef.h"
#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "EthPMCHw.h"
#include "EthPMCMboxMaster.h"
#include "EthPMCNetProtocolMaster.h"

#if CFG_ETHPMC
//***************************************************************************
// Locals
volatile static UBYTE ubEPMCNetProtlMaster_NetState = EPMCNETPROTL_STATE_INIT;

//***************************************************************************
// Node Ident
BOOL EPMCNetProtlMaster_NodeIdent(UBYTE ubNodeNum,HPULONG hpulUniqueID)
{
    UWORD uwDataIn[EPMCNETPROTL_MASTER_NODEIDENT_DATASIZE/2]={0};
    UWORD uwDataOut[EPMCHw_MBOX_SIZE]={0};
    UWORD uwSizeOut=0;
    
    // prepare command data
    uwDataIn[0] = EPMCNETPROTL_COMMAND_NODEIDENT;
    // start transaction
    if(EPMCMboxMaster_Transaction(ubNodeNum,                    // node number
                                  EPMCMBOX_COMMCTRL_PROTL_EPMC, // protocol
                                  uwDataIn,                     // data in
                                  EPMCNETPROTL_MASTER_NODEIDENT_DATASIZE, // size in
                                  uwDataOut,                    // data out
                                  &uwSizeOut))                  // size out
    {
        // check DataOut & SizeOut
        if(uwSizeOut==EPMCNETPROTL_SLAVE_NODEIDENT_DATASIZE)
        {
            *hpulUniqueID = ((ULONG)uwDataOut[1]<<16)+uwDataOut[2]; // get unique ID
            return TRUE;
        }
    }
    else if(ubNodeNum==EPMCMBOX_COMMCTRL_NODE_BST) // if breadcast command, always return TRUE after timeout
        return TRUE;

    return FALSE;
}

//***************************************************************************
// Set Node Number
BOOL EPMCNetProtlMaster_SetNodeNum(UBYTE ubNodeNum, UBYTE ubNewNodeNum)
{
    UWORD uwDataIn[EPMCNETPROTL_MASTER_SETNODENUM_DATASIZE/2]={0};
    UWORD uwDataOut[EPMCHw_MBOX_SIZE]={0};
    UWORD uwSizeOut=0;
    
    // prepare command data
    uwDataIn[0] = EPMCNETPROTL_COMMAND_SETNODENUM;
    uwDataIn[1] = (UWORD)ubNewNodeNum;
    // start transaction
    if(EPMCMboxMaster_Transaction(ubNodeNum,                   // node number
                                  EPMCMBOX_COMMCTRL_PROTL_EPMC,// protocol
                                  uwDataIn,                    // data in
                                  EPMCNETPROTL_MASTER_SETNODENUM_DATASIZE, // size in
                                  uwDataOut,                   // data out
                                  &uwSizeOut))                 // size out
    {
        // check DataOut & SizeOut
        if(uwSizeOut==EPMCNETPROTL_SLAVE_SETNODENUM_DATASIZE &&
           ubNodeNum==uwDataOut[3]) // reply old node number
           return TRUE;
    }
    else if(ubNodeNum==EPMCMBOX_COMMCTRL_NODE_BST) // if breadcast command, always return TRUE after timeout
        return TRUE;

    return FALSE;
}

//***************************************************************************
// Configure Realtime data Slot
BOOL EPMCNetProtlMaster_ConfigSlot(UBYTE ubNodeNum, UWORD uwSlotOffset, UWORD uwSlotSize)
{
    UWORD uwDataIn[EPMCNETPROTL_MASTER_CONFIGSLOT_DATASIZE/2]={0};
    UWORD uwDataOut[EPMCHw_MBOX_SIZE]={0};
    UWORD uwSizeOut=0;
    
    // prepare command data
    uwDataIn[0] = EPMCNETPROTL_COMMAND_CONFIGSLOT;
    uwDataIn[1] = (UWORD)uwSlotOffset;
    uwDataIn[2] = (UWORD)uwSlotSize;
    // start transaction
    if(EPMCMboxMaster_Transaction(ubNodeNum,                   // node number
                                  EPMCMBOX_COMMCTRL_PROTL_EPMC,// protocol
                                  uwDataIn,                    // data in
                                  EPMCNETPROTL_MASTER_CONFIGSLOT_DATASIZE, // size in
                                  uwDataOut,                   // data out
                                  &uwSizeOut))                 // size out
    {
        // check DataOut & SizeOut
        if(uwSizeOut==EPMCNETPROTL_SLAVE_CONFIGSLOT_DATASIZE &&
           uwSlotOffset==uwDataOut[1] && uwSlotSize==uwDataOut[2])
           return TRUE;
    }
    else if(ubNodeNum==EPMCMBOX_COMMCTRL_NODE_BST) // if breadcast command, always return TRUE after timeout
        return TRUE;

    return FALSE;
}

//***************************************************************************
// Enable/Disable Output Port
BOOL EPMCNetProtlMaster_EnOutPort(UBYTE ubNodeNum, BOOL bFlag)
{
    UWORD uwDataIn[EPMCNETPROTL_MASTER_ENOUTPORT_DATASIZE/2]={0};
    UWORD uwDataOut[EPMCHw_MBOX_SIZE]={0};
    UWORD uwSizeOut=0;
    
    // prepare command data
    uwDataIn[0] = EPMCNETPROTL_COMMAND_ENOUTPORT;
    uwDataIn[1] = (UWORD)bFlag;
    // start transaction
    if(EPMCMboxMaster_Transaction(ubNodeNum,                   // node number
                                  EPMCMBOX_COMMCTRL_PROTL_EPMC,// protocol
                                  uwDataIn,                    // data in
                                  EPMCNETPROTL_MASTER_ENOUTPORT_DATASIZE, // size in
                                  uwDataOut,                   // data out
                                  &uwSizeOut))                 // size out
    {
        // check DataOut & SizeOut
        if(uwSizeOut==EPMCNETPROTL_SLAVE_ENOUTPORT_DATASIZE &&
           bFlag==uwDataOut[1])
           return TRUE;
    }
    else if(ubNodeNum==EPMCMBOX_COMMCTRL_NODE_BST) // if breadcast command, always return TRUE after timeout
        return TRUE;

    return FALSE;
}

//***************************************************************************
// Get Output Port Link Status
BOOL EPMCNetProtlMaster_GetLinkStatus(UBYTE ubNodeNum, HPUWORD hpuLinkStatus)
{
    UWORD uwDataIn[EPMCNETPROTL_MASTER_GETLINKST_DATASIZE/2]={0};
    UWORD uwDataOut[EPMCHw_MBOX_SIZE]={0};
    UWORD uwSizeOut=0;
    
    // prepare command data
    uwDataIn[0] = EPMCNETPROTL_COMMAND_GETLINKST;
    // start transaction
    if(EPMCMboxMaster_Transaction(ubNodeNum,                   // node number
                                  EPMCMBOX_COMMCTRL_PROTL_EPMC,// protocol
                                  uwDataIn,                    // data in
                                  EPMCNETPROTL_MASTER_GETLINKST_DATASIZE, // size in
                                  uwDataOut,                   // data out
                                  &uwSizeOut))                 // size out
    {
        // check DataOut & SizeOut
        if(uwSizeOut==EPMCNETPROTL_SLAVE_GETLINKST_DATASIZE)
        {
            *hpuLinkStatus = uwDataOut[1];
            return TRUE;
        }
    }
    else if(ubNodeNum==EPMCMBOX_COMMCTRL_NODE_BST) // if breadcast command, always return TRUE after timeout
        return TRUE;

    return FALSE;
}

//***************************************************************************
// Network state request
BOOL EPMCNetProtlMaster_NetStateReq(UBYTE ubNodeNum,UBYTE ubNetState)
{
    UWORD uwDataIn[EPMCNETPROTL_MASTER_NETSTATEREQ_DATASIZE/2]={0};
    UWORD uwDataOut[EPMCHw_MBOX_SIZE]={0};
    UWORD uwSizeOut=0;
    
    // prepare command data
    uwDataIn[0] = EPMCNETPROTL_COMMAND_NETSTATEREQ;
    uwDataIn[1] = (UWORD)ubNetState;
    // start transaction
    if(EPMCMboxMaster_Transaction(ubNodeNum,                   // node number
                                  EPMCMBOX_COMMCTRL_PROTL_EPMC,// protocol
                                  uwDataIn,                    // data in
                                  EPMCNETPROTL_MASTER_NETSTATEREQ_DATASIZE, // size in
                                  uwDataOut,                   // data out
                                  &uwSizeOut))                 // size out
    {
        // check DataOut & SizeOut
        if(uwSizeOut==EPMCNETPROTL_SLAVE_NETSTATEREQ_DATASIZE &&
           ubNetState==uwDataOut[1])
           return TRUE;
    }
    else if(ubNodeNum==EPMCMBOX_COMMCTRL_NODE_BST) // if breadcast command, always return TRUE after timeout
        return TRUE;

    return FALSE;
}

//***************************************************************************
// Network frame timeout
BOOL EPMCNetProtlMaster_FrameTimeout(UBYTE ubNodeNum,UWORD uwTimeout)
{
    UWORD uwDataIn[EPMCNETPROTL_MASTER_FRAMETIMEOUT_DATASIZE/2]={0};
    UWORD uwDataOut[EPMCHw_MBOX_SIZE]={0};
    UWORD uwSizeOut=0;
    
    // prepare command data
    uwDataIn[0] = EPMCNETPROTL_COMMAND_FRAMETIMEOUT;
    uwDataIn[1] = uwTimeout;
    // start transaction
    if(EPMCMboxMaster_Transaction(ubNodeNum,                   // node number
                                  EPMCMBOX_COMMCTRL_PROTL_EPMC,// protocol
                                  uwDataIn,                    // data in
                                  EPMCNETPROTL_MASTER_FRAMETIMEOUT_DATASIZE, // size in
                                  uwDataOut,                   // data out
                                  &uwSizeOut))                 // size out
    {
        // check DataOut & SizeOut
        if(uwSizeOut==EPMCNETPROTL_SLAVE_FRAMETIMEOUT_DATASIZE &&
           uwTimeout==uwDataOut[1])
           return TRUE;
    }
    else if(ubNodeNum==EPMCMBOX_COMMCTRL_NODE_BST) // if breadcast command, always return TRUE after timeout
        return TRUE;

    return FALSE;
}
#endif
