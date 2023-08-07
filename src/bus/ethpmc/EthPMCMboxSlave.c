/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCMboxSlave.c                                          */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Mailbox protocol handler, slave implementation             */
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
#include "EthPMCNetProtocol.h"
#include "EthPMCCommandMgr.h"
#include "EthPMCMboxSlave.h"

#if CFG_ETHPMC

//***************************************************************************
// Local variables
static const EPMCMBOXSL_PROTOCOLS * psProtHandlers = NULL;
static UBYTE ubEPMCMboxSlave_NodeNum = EPMCMBOX_COMMCTRL_NODE_BST;
static void (*pfLateAction)(void) = NULL;

static UWORD uwEPMCMboxSlave_WDTimeout;
BOOL  bEPMCMboxSlave_SyncEnable=FALSE;
static BOOL bEPMCMboxSlave_NodeNumLock = FALSE;

//***************************************************************************
// Slave Init
void EPMCMboxSlave_Init(void)
{
    // Set default node number
    EPMCMboxSlave_SetNodeNum(EPMCMBOX_COMMCTRL_NODE_BST); 
    
    // Set Device as Slave, Disable Ouput
    EPMCHw_Device_Set(EPMCHw_DEVICEMODE_ST);
    
    bEPMCMboxSlave_NodeNumLock = FALSE;
    bEPMCMboxSlave_SyncEnable=FALSE; // internal PWM synchronization
    uwEPMCMboxSlave_WDTimeout=0;
}

//***************************************************************************
// Slave state active
void EPMCMboxSlave_Active(void)
{
    bEPMCMboxSlave_SyncEnable=TRUE; // internal PWM synchronization
}

//***************************************************************************
// Enable/disable slave protocol processing
void EPMCMboxSlave_SetProtocol(const EPMCMBOXSL_PROTOCOLS * psProt)
{
    psProtHandlers = psProt;
}

//***************************************************************************
// Processing loop
void EPMCMboxSlave_Process(void)
{
    // if not NULL then protocol slave processing is enabled
    if(psProtHandlers)
        if(EPMCHw_RxFrame_Valid()&&!EPMCHw_RxFrame_Error()) // rx frame detected
        {
            UWORD uwCommControl;
            UWORD uwReqNodeNum;

            EPMCHw_Mbox_ReadWord(&uwCommControl,0,1); // read command control from rx frame
            uwReqNodeNum = EPMCMBOX_GET_NODE(uwCommControl);

            // check if it's request command, node broadcast or equal to local
            if(EPMCMBOX_GET_COMM(uwCommControl)==EPMCMBOX_COMMCTRL_CMD_REQ &&
              (uwReqNodeNum==EPMCMboxSlave_GetNodeNum() || 
               uwReqNodeNum==EPMCMBOX_COMMCTRL_NODE_BST))
            {
                UWORD uwReqProt=EPMCMBOX_GET_PROTL(uwCommControl);
                // find protocol entry point
                const EPMCMBOXSL_PROTOCOLS * psProt=psProtHandlers;
                for(;psProt->pfHandler!=NULL;psProt++)
                    if(psProt->ubProtocol==uwReqProt)
                        break;

                // if found
                if(psProt->pfHandler)
                {
                    UWORD uwPayLoadIn[EPMCMBOXL_MAX_PAYLOADSIZE/sizeof(UWORD)];
                    UWORD uwPayLoadOut[EPMCMBOXL_MAX_PAYLOADSIZE/sizeof(UWORD)];
                    UWORD uwSizeOut=EPMCMBOXL_MAX_PAYLOADSIZE;
                    BOOL bRetVal;

                    UWORD ubReqDSize=EPMCMBOX_GET_DSIZE(uwCommControl);
                    EPMCHw_Mbox_ReadWord(uwPayLoadIn,1,ubReqDSize/2);     // get payload

                    bRetVal = (*psProt->pfHandler)(uwPayLoadIn,ubReqDSize,uwPayLoadOut,&uwSizeOut);

                    // if valid and not broadcast address reply to the master
                    if(bRetVal && uwReqNodeNum!=EPMCMBOX_COMMCTRL_NODE_BST)
                    {
                        EPMCHw_Mbox_WriteWord(uwPayLoadOut,1,uwSizeOut/sizeof(UWORD));  // write payload out
            
                        uwCommControl=EPMCMBOX_SET(uwReqProt,EPMCMboxSlave_GetNodeNum(),(UWORD)uwSizeOut,EPMCMBOX_COMMCTRL_CMD_RPLY);
                        EPMCHw_Mbox_WriteWord(&uwCommControl,0,1);      // write CommControl word
            
                        EPMCHw_Mbox_Set(TRUE); // Enable Mailbox Re-writing(Slave)
                    }
                }
            }
        }

    // if late actions are required
    if(pfLateAction)
    {
        // wait mailbox reply refresh
        timer_wait(uwSysTimers125us,2);

        // and execute it
        (*pfLateAction)();

        pfLateAction=NULL;
    }
}

//***************************************************************************
// Check Node Number
BOOL EPMCMboxSlave_CheckNodeNum(UBYTE ubNodeNum)
{
    if(ubNodeNum==EPMCMBOX_COMMCTRL_NODE_NULL) // invalid number
        return FALSE;
    else
        return TRUE;    
}

//***************************************************************************
// Set Slave Node Number
BOOL EPMCMboxSlave_SetNodeNum(UBYTE ubNodeNum)
{
    if(!bEPMCMboxSlave_NodeNumLock && EPMCMboxSlave_CheckNodeNum(ubNodeNum)) // check node number
    {
        bEPMCMboxSlave_NodeNumLock = TRUE;
        ubEPMCMboxSlave_NodeNum = ubNodeNum;
        return TRUE;
    }
    else
        return FALSE;
}

//***************************************************************************
// Get Slave Node Number
UBYTE EPMCMboxSlave_GetNodeNum(void)
{
    return ubEPMCMboxSlave_NodeNum;
}

//***************************************************************************
// Set timeout for watchdog of incoming frame
BOOL EPMCMboxSlave_SetTimeout(UWORD uwTimeout)
{
    uwEPMCMboxSlave_WDTimeout = uwTimeout;
    return TRUE;
}

// Get timeout for watchdog of incoming frame
UWORD EPMCMboxSlave_GetTimeout()
{
    return uwEPMCMboxSlave_WDTimeout;
}

//***************************************************************************
// Require late action to be performed after master has received the reply
void EPMCMboxSlave_RunLateAction(void (*pfAction)(void))
{
    pfLateAction=pfAction;
}

#endif
