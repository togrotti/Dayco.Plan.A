/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCMboxMaster.c                                         */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Mailbox protocol handler, master implementation            */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

#include "common\CommonDefines.h"
#include "common\commontypedef.h"
#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "common\TaskScheduler.h"
#include "EthPMCHw.h"
#include "EthPMCCommandMgr.h"
#include "EthPMCMboxMaster.h"

#if CFG_ETHPMC

//***************************************************************************
// define

#define EPMCMBOXMASTER_WAITREPLY_TIMEOUT    20

//***************************************************************************
// Locals

// Wait for reply from slave
static BOOL EPMCMboxMaster_WaitReply(UWORD uwWaitingCC);

// Set Master status to idle
static void EPMCMboxMaster_Idle(void);

//***************************************************************************
// Master Init
void EPMCMboxMaster_Init(void)
{
    UWORD uwSAData[3]=EPMCHw_SA_INITDATA;
    UWORD uwDAData[3]=EPMCHw_DA_BROADCAST;
    
    // Set Device as Master, Enable Ouput
    EPMCHw_Device_Set(EPMCHw_DEVICEMODE_MNT);
    
    // Set Frame Length
    EPMCHw_FrameLen_Set(EPMCHw_FLEN_MIN);
    
    // Set Mbox RST flag into frame
    EPMCHw_Mbox_Set(TRUE);
    
    // Init Ethernet Header of Frame
    EPMCHw_EthHeader_Set((HPUWORD)&uwDAData[0],(HPUWORD)&uwSAData,EPMCHw_ETP_DATA);
        
    // Enable Auto Trigger
    EPMCHw_FrameTriggerMode_Set(EPMCHw_TRIGGERMODE_AUTO,TRUE);
}

//***************************************************************************
// Set Master status to IDLE
static void EPMCMboxMaster_Idle(void)
{
    UWORD uwCommControl=EPMCMBOX_SET(0,0,0,EPMCMBOX_COMMCTRL_CMD_IDLE);
    EPMCHw_Mbox_WriteWord(&uwCommControl,0,1);
}

//***************************************************************************
// Wait for Reply
static BOOL EPMCMboxMaster_WaitReply(UWORD uwWaitingCC)
{
    UWORD uwTimer=0;
    UWORD bTimeout=TRUE;
    
    uwTimer = timer_settimeout(uwSysTimers1ms, EPMCMBOXMASTER_WAITREPLY_TIMEOUT); // Set timeout timer

    for(;;)
    {
        if(EPMCHw_RxFrame_Valid()&&!EPMCHw_RxFrame_Error()) // rx frame detected
        {
            UWORD uwReplyCommCtrl;
            EPMCHw_Mbox_ReadWord(&uwReplyCommCtrl,0,1); // read command control from rx frame
    
            // TRUE if request CommControl Word match with the reply CommControl Word
            if(((uwReplyCommCtrl&EPMCMBOX_COMMCTRL_COMM_MASK)==EPMCMBOX_COMMCTRL_CMD_RPLY) && // check command
               ((uwReplyCommCtrl&EPMCMBOX_COMMCTRL_NODE_MASK)==(uwWaitingCC&EPMCMBOX_COMMCTRL_NODE_MASK)) && // check node number
               ((uwReplyCommCtrl&EPMCMBOX_COMMCTRL_PROTL_MASK)==(uwWaitingCC&EPMCMBOX_COMMCTRL_PROTL_MASK))) // check protocol
            {
                bTimeout = FALSE;
                break;
            }
        }
        
        if(timer_istimedout(uwSysTimers1ms, uwTimer)) // wait until timeout
            break;
    }
 
    return (!bTimeout); // false when timeout
}

//***************************************************************************
// Mailbox Master Transaction
BOOL EPMCMboxMaster_Transaction(UBYTE ubNodeNum, UBYTE ubProtocol, HPUWORD hpuDataIn, UWORD uwSizeIn, HPUWORD hpuDataOut, HPUWORD hpuSizeOut)
{
    UWORD uwCommControl;
    BOOL  bGetReply=FALSE;
    
    // Reset mbox RST flag into Header
    EPMCHw_Mbox_Set(FALSE);
    
    // Prepare&Write mailbox data            
    EPMCHw_Mbox_WriteWord(hpuDataIn,1,uwSizeIn/sizeof(UWORD));  // write data to mailbox, with zero byte 0&1
    uwCommControl = EPMCMBOX_SET(ubProtocol,ubNodeNum,(UBYTE)uwSizeIn,EPMCMBOX_COMMCTRL_CMD_REQ); // Set CommControl word
    EPMCHw_Mbox_WriteWord(&uwCommControl,0,1);                   // write CommControl word to mailbox
    
    // Wait for reply
    if(EPMCMboxMaster_WaitReply(uwCommControl))
    {
        bGetReply = TRUE;
        
        EPMCHw_Mbox_ReadWord((HPUWORD)&uwCommControl,0,1);  // read CommControl word
        *hpuSizeOut = EPMCMBOX_GET_DSIZE(uwCommControl);    // size out
        
        EPMCHw_Mbox_ReadWord(hpuDataOut,1,(*hpuSizeOut)/sizeof(UWORD)); // data out
    }
    
    // Set Mailbox status to IDLE
    EPMCMboxMaster_Idle();
    
    // Set mbox RST flag into Header
    EPMCHw_Mbox_Set(TRUE);
    
    // Refresh mailbox data
    timer_wait(uwSysTimers125us,3);
    
    return bGetReply;
}
#endif
