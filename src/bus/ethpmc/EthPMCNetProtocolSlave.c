/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright $)AB) 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCNetProtocolSlave.c                                   */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : EtherPMC Network Protocol Slave Handler                    */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

#include "common\CommonDefines.h"
#include "common\commontypedef.h"
#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "drive\HardwareConfig.h"
#include "EthPMCHw.h"
#include "EthPMCMboxSlave.h"
#include "EthPMCNetProtocolSlave.h"
#include <string.h>

#if CFG_ETHPMC
//***************************************************************************
// Local variables
volatile static UBYTE ubEPMCNetProtlSlave_NetState = EPMCNETPROTL_STATE_INIT;

//***************************************************************************
// Local functions
static BOOL EPMCNetProtlSlave_NetState(UBYTE ubNetState);
static BOOL EPMCNetProtlSlave_GetLinkStatus(HPUWORD hpuLinkStatus);

//***************************************************************************
// Network Protocol Slave Process
BOOL EPMCNetProtlSlave_Process(HPUWORD hpuwNetProtlData,UWORD uwNetProtlSize,HPUWORD hpuDataOut,HPUWORD puwSizeOut)
{
    ULONG ulUniqueID;
    BOOL bValid=FALSE;

    // Decode Command
    switch(hpuwNetProtlData[0])
    {
        case EPMCNETPROTL_COMMAND_NODEIDENT:
            if(uwNetProtlSize==EPMCNETPROTL_MASTER_NODEIDENT_DATASIZE) // check data size
            {
                // Execute Command: get unique ID
                ulUniqueID = HwGetUID32();
                hpuDataOut[0] = EPMCNETPROTL_COMMAND_NODEIDENT;
                hpuDataOut[1] = (UWORD)(ulUniqueID>>16);
                hpuDataOut[2] = (UWORD)(ulUniqueID&((1l<<16)-1));
                *puwSizeOut = EPMCNETPROTL_SLAVE_NODEIDENT_DATASIZE;
                bValid=TRUE;
            }
            break;

        case EPMCNETPROTL_COMMAND_SETNODENUM:
            if(uwNetProtlSize==EPMCNETPROTL_MASTER_SETNODENUM_DATASIZE) // check data size
                if(EPMCMboxSlave_CheckNodeNum(hpuwNetProtlData[1])) // check node number validity
                {
                    // Execute Command:
                    ulUniqueID = HwGetUID32(); // get unique ID
                    // Prepare data
                    hpuDataOut[0] = EPMCNETPROTL_COMMAND_SETNODENUM;
                    hpuDataOut[1] = (UWORD)(ulUniqueID>>16);
                    hpuDataOut[2] = (UWORD)(ulUniqueID&((1l<<16)-1));
                    hpuDataOut[3] = EPMCMboxSlave_GetNodeNum(); // reply old node number
                    *puwSizeOut = EPMCNETPROTL_SLAVE_SETNODENUM_DATASIZE;
                    bValid=TRUE;
                    
                    EPMCMboxSlave_SetNodeNum(hpuwNetProtlData[1]); // set new node number after send transaction
                 }
            break;

        case EPMCNETPROTL_COMMAND_CONFIGSLOT:
            if(uwNetProtlSize==EPMCNETPROTL_MASTER_CONFIGSLOT_DATASIZE) // check data size
                if(EPMCHw_Slot_Set(hpuwNetProtlData[1],hpuwNetProtlData[2])) // set slot offset&size
                {
                    // copy data for reply
                    memcpy(hpuDataOut, hpuwNetProtlData, EPMCNETPROTL_SLAVE_CONFIGSLOT_DATASIZE);
                    *puwSizeOut = EPMCNETPROTL_SLAVE_CONFIGSLOT_DATASIZE;
                    bValid=TRUE;
                }
            break;

        case EPMCNETPROTL_COMMAND_ENOUTPORT:
            if(uwNetProtlSize==EPMCNETPROTL_MASTER_ENOUTPORT_DATASIZE) // check data size
            {
                // Execute Command:
                EPMCHw_TerminalEn(!(BOOL)hpuwNetProtlData[1]); // enable/disable output port

                // copy data for reply
                memcpy(hpuDataOut, hpuwNetProtlData, EPMCNETPROTL_SLAVE_ENOUTPORT_DATASIZE);
                *puwSizeOut = EPMCNETPROTL_SLAVE_ENOUTPORT_DATASIZE;
                bValid=TRUE;
            }
            break;

        case EPMCNETPROTL_COMMAND_GETLINKST:
            if(uwNetProtlSize==EPMCNETPROTL_MASTER_GETLINKST_DATASIZE) // check data size
            {
                UWORD uwLinkStatus;
                if(EPMCNetProtlSlave_GetLinkStatus(&uwLinkStatus)) // get output port link status
                {
                    // Prepare data
                    hpuDataOut[0] = EPMCNETPROTL_COMMAND_GETLINKST;
                    hpuDataOut[1] = uwLinkStatus;
                    *puwSizeOut = EPMCNETPROTL_SLAVE_GETLINKST_DATASIZE;
                    bValid=TRUE;
                }
            }
            break;

        case EPMCNETPROTL_COMMAND_NETSTATEREQ:
            if(uwNetProtlSize==EPMCNETPROTL_MASTER_NETSTATEREQ_DATASIZE) // check data size
                if(hpuwNetProtlData[1]==EPMCNETPROTL_STATE_INIT || hpuwNetProtlData[1]==EPMCNETPROTL_STATE_SLAVE) // check network state
                {
                    // Execute Command: 
                    EPMCNetProtlSlave_NetState((UBYTE)hpuwNetProtlData[1]);

                    // copy data for reply
                    memcpy(hpuDataOut, hpuwNetProtlData, EPMCNETPROTL_SLAVE_NETSTATEREQ_DATASIZE);
                    *puwSizeOut = EPMCNETPROTL_SLAVE_NETSTATEREQ_DATASIZE;
                    bValid=TRUE;
                }
            break;
        case EPMCNETPROTL_COMMAND_FRAMETIMEOUT: 
            if(uwNetProtlSize==EPMCNETPROTL_MASTER_FRAMETIMEOUT_DATASIZE) // check data size
                // Execute Command:
                if(EPMCMboxSlave_SetTimeout(hpuwNetProtlData[1])) 
                {
                    // copy data for reply
                    memcpy(hpuDataOut, hpuwNetProtlData, EPMCNETPROTL_SLAVE_FRAMETIMEOUT_DATASIZE);
                    *puwSizeOut = EPMCNETPROTL_SLAVE_FRAMETIMEOUT_DATASIZE;
                    bValid=TRUE;
                }
            break;

        default:
            break;
    }

    return bValid;
}

//***************************************************************************
// Network State
static BOOL EPMCNetProtlSlave_NetState(UBYTE ubNetState)
{
    if(ubNetState==EPMCNETPROTL_STATE_INIT)
    {
        EPMCMboxSlave_RunLateAction(&EPMCMboxSlave_Init);
        ubEPMCNetProtlSlave_NetState = EPMCNETPROTL_STATE_INIT;
        return TRUE;
    }
    else if(ubNetState==EPMCNETPROTL_STATE_SLAVE)
    {
        EPMCMboxSlave_RunLateAction(&EPMCMboxSlave_Active);
        ubEPMCNetProtlSlave_NetState = EPMCNETPROTL_STATE_SLAVE;
        return TRUE;
    }
    else
        return FALSE;
}

//***************************************************************************
// Get Network State
UBYTE EPMCNetProtlSlave_GetNetState(void)
{
    return ubEPMCNetProtlSlave_NetState;
}

//***************************************************************************
// Get Link Status
static BOOL EPMCNetProtlSlave_GetLinkStatus(HPUWORD hpuLinkStatus)
{
    /* | 3 |   2  |  1  | 0  | */
    /*  MDI Duplex Speed Link  */
    UWORD uwPhyData;
    UWORD uwStatus=0;
    
    // Read register 0h
    if(EPMCHw_MIIMRead(EPMCHw_OPORT_PHYADDR,0x00,&uwPhyData))
    {
        uwStatus |= ((uwPhyData&(1<<8))>>(8-EPMCNETPROTL_LINKSTATUS_DUPLEX))|  // Bit 8 in Register 0h: Duplex Mode
                    ((uwPhyData&(1<<13))>>(13-EPMCNETPROTL_LINKSTATUS_SPEED)); // Bit 13 in Register 0h: Speed
    }
    else
        return FALSE; // MII read error

    EPMCHw_MIIMRead(EPMCHw_OPORT_PHYADDR,0x03,&uwPhyData);

    if((uwPhyData&MC_MODELNUMBER_MASK)==MC_MODELNUMBER_KSZ8081)
    {
        // Read register 1Eh
        if(EPMCHw_MIIMRead(EPMCHw_OPORT_PHYADDR,0x1E,&uwPhyData))
        {
            uwStatus |= ((uwPhyData&(1<<8))>>(8-EPMCNETPROTL_LINKSTATUS_LINK))| // Bit 8 in Register 1Eh: Link Status
                        ((uwPhyData&(1<<5))>>(5-EPMCNETPROTL_LINKSTATUS_MDI));  // Bit 5 in Register 1Eh: MDI/MDI-X State
        }
        else
            return FALSE; // MII read error
    }
    else if((uwPhyData&MC_MODELNUMBER_MASK)==MC_MODELNUMBER_KSZ8041)
    {
        // Read register 1Eh
        if(EPMCHw_MIIMRead(EPMCHw_OPORT_PHYADDR,0x1E,&uwPhyData))
        {
            uwStatus |= ((uwPhyData&(1<<11))>>(11-EPMCNETPROTL_LINKSTATUS_MDI));  // Bit 11 in Register 1Eh: MDI/MDI-X State
        }
        else
            return FALSE; // MII read error

        // Read register 01h
        if(EPMCHw_MIIMRead(EPMCHw_OPORT_PHYADDR,0x01,&uwPhyData))
        {
            uwStatus |= ((uwPhyData&(1<<2))>>(2-EPMCNETPROTL_LINKSTATUS_LINK));  // Bit 2 in Register 1Eh: Link Status
        }
        else
            return FALSE; // MII read error
    }
    else if((uwPhyData&MC_MODELNUMBER_MASK)==MC_MODELNUMBER_YT8521H)//YT8512H
    {
		// Read register 0x11
        if(EPMCHw_MIIMRead(EPMCHw_OPORT_PHYADDR,0x11,&uwPhyData))
        {
            uwStatus |= ((uwPhyData&(1<<10))>>(10-EPMCNETPROTL_LINKSTATUS_LINK))| // Bit 10 in Register 11h: Link Status
                        ((uwPhyData&(1<<6))>>(6-EPMCNETPROTL_LINKSTATUS_MDI));  // Bit 6 in Register 11h: MDI/MDI-X State
        }
        else
            return FALSE; // MII read error
    }
    
    *hpuLinkStatus = uwStatus;
    return TRUE;
}
#endif
