/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright $)AB) 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCCommandMgr.c                                         */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : EtherPMC specific command manager                          */
/*                                                                          */
/****************************************************************************/


#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\TaskScheduler.h"
#include "system\Os.h"
#include "plc\plc.h"
#include "EthPMCHw.h"
#include "EthPMCMboxMaster.h"
#include "EthPMCMboxSlave.h"
#include "EthPMCNetProtocolMaster.h"
#include "EthPMCNetProtocolSlave.h"
#include "EthPMCCommandMgr.h"
#include "fpga\FpgaHandler.h"
#include "drive\HardwareUnitID.h"
#include "system\SysAppFpgaOptCodes.h"
#include "bus\SyncManager.h"
#include "system\SysLogManagement.h"
#include "bus\ethercat\ECATCommandMgr.h"
#include "drive\HardwareConfig.h"

#if CFG_ETHPMC
//***************************************************************************
// Definition 
#define EPMC_CONTROL_SCAN           0
#define EPMC_CONTROL_SCANSTART      1

#define EPMC_STATUS_NETSTATE        0
#define EPMC_STATUS_SCANACK         2
#define EPMC_STATUS_SCANEND         3
#define EPMC_STATUS_SACNERR         4
#define EPMC_STATUS_NODE            8

#define EPMC_STATUS_NETSTATE_MASK   (((1u<<EPMC_STATUS_SCANACK)-1)-((1u<<EPMC_STATUS_NETSTATE)-1))

#define EPMC_STATUS_SET(n,r,d,k,s)  ((((UWORD)s)<<EPMC_STATUS_NETSTATE)|\
                                     (((UWORD)k)<<EPMC_STATUS_SCANACK) |\
                                     (((UWORD)d)<<EPMC_STATUS_SCANEND) |\
                                     (((UWORD)r)<<EPMC_STATUS_SACNERR) |\
                                     (((UWORD)n)<<EPMC_STATUS_NODE))

#define SLOWTASK_STACKSIZE                      (OS_DEFAULTUSRSTATICSTACK+0x0100)

#define EPMC_FRAMEWDT_TIMEOUT       8
#define EPMC_PHYREAD_TIMEOUT        5000
#define EPMC_SCAN_TIMEOUT           15000                  

#define EPMC_A_LED1_PIN             (MIO_PIN_BASE+52)
#define EPMC_B_LED1_PIN             (MIO_PIN_BASE+39)

//***************************************************************************
// Data Structure

typedef struct
{
    UWORD uwSlaves; // number of connected slaves
    struct
    {
        UWORD uwOf; // offset from beginning of realtime slot
        UWORD uwSz; // size of the slot available to the slave
    }sSlots[64];
}EPMC_REALTIMESLOTS;

//****************************************************************************
// Globals

EPMC_PARAMS sEpmcCM_Params;
const EPMC_PARAMS sEpmcCM_DefParams={0,{0}};

EPMC_INOUT sEpmcCMInOut;
EPMC_DIAG sEpmcCMDiag;

#ifdef _INFINEON_
//***************************************************************************
// Avoids warning C37: nonstandard extension - unnamed parameter

#pragma warning disable = 37
#endif // _infineon_


//***************************************************************************
// Locals

static BOOL bReSyncEnabled=FALSE;
// managed protocols
static const EPMCMBOXSL_PROTOCOLS sProtocols[]=
{
    {&EPMCNetProtlSlave_Process, EPMCMBOX_COMMCTRL_PROTL_EPMC},
    {NULL, 0},  // terminator
};

volatile static UWORD uwEpmcCM_PrevScanStart;
volatile static UWORD uwEpmcCM_PrevScanState;

static EPMC_REALTIMESLOTS * hpsSlaveConfig=NULL;
static BOOL bStartMaster=FALSE;
static void slowtask(void);
static BOOL EpmcCM_IterateProgram(EPMC_NETCONNDEVICES* psEPMCNetConnDev,BOOL bIDCheck);
static BOOL EpmcCM_EPStartScan(void);
static BOOL EpmcCM_EPStartMaster(void);
static void EpmcCM_ErrorCheck(EPMC_DIAG * psDiagOut);

//***************************************************************************
// Init handler

BOOL EpmcCM_Init(UWORD uwOption)
{
	UWORD uwRData = 0;
    switch(uwOption)
    {
        case EPMCCM_INIT_ALWAYS:
                // if EthPMC hardware is available
            if(sGlbOptReq.ulHardwareOpt&HWUNITID_ETH_DBLRMII_0)
            {
                    // set ports as output and set leds when enabled
#ifdef _INFINEON_
                P7_IOCR01=0x0080;
                P7_OUT_P1=TRUE;
                P7_IOCR02=0x0080;
                P7_OUT_P2=TRUE;
#else
#ifndef _HW_VERSION_C
                Gpio_SetMode(EPMC_A_LED1_PIN, GPIO_DIR_OUT);
                Gpio_SetMode(EPMC_B_LED1_PIN, GPIO_DIR_OUT);
                GPIO_OUT(EPMC_A_LED1_PIN, TRUE);
                GPIO_OUT(EPMC_B_LED1_PIN, TRUE);
#endif // !_hw_version_c
#endif // _infineon_
            }
            else
            {
#ifndef _HW_VERSION_C

                Gpio_SetMode(EPMC_A_LED1_PIN, GPIO_DIR_OUT);
                Gpio_SetMode(EPMC_B_LED1_PIN, GPIO_DIR_OUT);
                GPIO_OUT(EPMC_A_LED1_PIN, FALSE);
                GPIO_OUT(EPMC_B_LED1_PIN, FALSE);
#endif // !_hw_version_c

            }
            break;
        case EPMCCM_INIT_CORE:
                // if module disabled
            if(!sEpmcCM_Params.flags.f.bEnableModule)
            {
                EPMCHw_PhyEnable(FALSE);
                return TRUE;
            }
                        // only for hardware version C board,use the phy chip YT8512
#ifdef _HW_VERSION_C 
			// enable phy
			EPMCHw_PhyEnable(FALSE);
			timer_wait(uwSysTimers1ms,200);
			EPMCHw_PhyEnable(TRUE);
			timer_wait(uwSysTimers1ms,200);
			//select the 50MHZ 
			EPMCHw_MIIMWrite(EPMCHw_IPORT_PHYADDR, 0x1E, 0x0050); // Read ext reg 0x50 select the 50MHZ clock
			EPMCHw_MIIMRead(EPMCHw_IPORT_PHYADDR, 0x1F, &uwRData); // Read ext_reg 0x50 value to uwRData
			EPMCHw_MIIMWrite(0, 0x1E, 0x0050); // Write ext reg 0x50 bit6 to 1
			EPMCHw_MIIMWrite(0, 0x1F, uwRData|0x0040); // Write ext reg 0x50 bit6 to 1

			EPMCHw_MIIMWrite(EPMCHw_IPORT_PHYADDR, 0x1E, 0x4001); // Read ext reg 0x50 select the 50MHZ clock
			EPMCHw_MIIMRead(EPMCHw_IPORT_PHYADDR, 0x1F, &uwRData); // Read ext_reg 0x50 value to uwRData
			EPMCHw_MIIMWrite(0, 0x1E, 0x4001); // Write ext reg 0x50 bit6 to 1
			EPMCHw_MIIMWrite(0, 0x1F, uwRData|0x0030); // Write ext reg 0x50 bit6 to 1

			//EPMCHw_MIIMRead(EPMCHw_OPORT_PHYADDR, 0x10, &uwRData); // Read ext_reg 0x50 value to uwRData
			//EPMCHw_MIIMWrite(EPMCHw_OPORT_PHYADDR, 0x10, uwRData|0x0008); // Write reg 0x0000 bit15 to 1

			EPMCHw_MIIMRead(EPMCHw_IPORT_PHYADDR, 0x00, &uwRData); // Read reg 0x0000 set to soft reset
			EPMCHw_MIIMWrite(0, 0x00, uwRData|0x8000); // Write reg 0x0000 bit15 to 1
			//disable the phy address 0 broadcast
			timer_wait(uwSysTimers1ms,50);
			EPMCHw_MIIMWrite(EPMCHw_IPORT_PHYADDR, 0x1E, 0x0000); // Read ext reg 0
			EPMCHw_MIIMRead(EPMCHw_IPORT_PHYADDR, 0x1F, &uwRData); // Read ext_reg 0 value to uwRData
			EPMCHw_MIIMWrite(0, 0x1E, 0x0000); // Write ext reg 0x50 bit6 to 1
			EPMCHw_MIIMWrite(0, 0x1F, uwRData&0xFFBF); // Write ext reg 0 bit6 to 0
			
#endif // _hw_version_c

            EPMCHw_PhyEnable(TRUE);

                // add slow task
            if(!Os_TaskCreateEx(&slowtask,SLOWTASK_STACKSIZE,"EpmcSlowTask"))
                return FALSE;
            break;
        case EPMCCM_INIT_SYNCEVENT:
            if(!sEpmcCM_Params.flags.f.bEnableModule)
                return TRUE;

            bSysStatSyncMgrRequired=TRUE;
            bReSyncEnabled = !(tEcatCMParam.flags.f.bEnableModule & tEcatCMParam.flags.f.bReSyncEnable);
            if(!TaskSched_AddRTTask(&EpmcCM_SyncEvent, TASKSCHEDULER_FLAG_NONE, 0, SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING), 0))
                return FALSE;
            break;
        default:
            assert(FALSE);
    }

    return TRUE; 
}

//***************************************************************************
// Hardware option callback

void EpmcCM_GetHwOpt(UWORD T, ULONG * pulHwOpt, ULONG * pulFPGAOpt)
{
    if(sEpmcCM_Params.flags.f.bEnableModule)
    {
        *pulHwOpt  |=HWUNITID_ETH_DBLRMII_0;
        *pulFPGAOpt|=FPGA_HW_ETHERPMC;
    }
}

//***************************************************************************
// Setup slaves configuration and implicit switch on as a master

BOOL EpmcCM_MasterParams(HPUBYTE hpubBin, UWORD T)
{
        // must be executed only at preboot
    if(!bSysStatBooting)
        return FALSE;

        // set configuration address
    hpsSlaveConfig=(EPMC_REALTIMESLOTS *)hpubBin;
    
    if(sEpmcCM_Params.flags.f.bEnableModule)
        bStartMaster=TRUE;

    return TRUE;
}

//***************************************************************************
// Slow task

static void slowtask(void)
{
    UWORD uwTimer=0;
    UWORD uwRData,uwWData;
    UBYTE ubIPortAdr=EPMCHw_IPORT_PHYADDR;
    UBYTE ubOPortAdr=EPMCHw_OPORT_PHYADDR;
     
    // PHY Initialization
    // 1ms to let phy autoconfig
    timer_wait(uwSysTimers100ns, 10000);

    uwTimer = timer_settimeout(uwSysTimers1ms,EPMC_PHYREAD_TIMEOUT); // setup initialization timeout timer
    for(;;)
    {   
        if(timer_istimedout(uwSysTimers1ms, uwTimer))
        {
            SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_ETHERPMC_FAIL, SYSTEMALARMS_SUBCODE_ETHERPMC_PHYERROR, FALSE);
        
            break;
        }

        if(!EPMCHw_MIIMRead(ubOPortAdr,0x03,&uwRData))
        {
            timer_wait(uwSysTimers100ns, 10000);
            continue;
        }
            
        if((uwRData&MC_MODELNUMBER_MASK)==MC_MODELNUMBER_KSZ8081)
        {
            EPMCHw_DisableBroadcastAddr(); // diable broadcast of PHY
            
            break;
        }
        else if((uwRData&MC_MODELNUMBER_MASK)==MC_MODELNUMBER_KSZ8041)
        {
            // Register 14h [7:6]: Preamble Restore
            //                   1 = Restore received preamble to
            //                       MII output (random latency)
            //                   0 = Consume 1-byte preamble
            //                       before sending frame to MII
            //                       output for fixed latency
            EPMCHw_MIIMRead(ubIPortAdr,0x14,&uwRData);
            uwWData=(uwRData&~(0x0003<<6))|(0x0003<<6);
            EPMCHw_MIIMWrite(ubIPortAdr,0x14,uwWData);
            EPMCHw_MIIMRead(ubIPortAdr,0x14,&uwRData);
            if(uwRData!=uwWData)
            {
				timer_wait(uwSysTimers100ns, 10000);
				continue;
			}
            
            EPMCHw_MIIMRead(ubOPortAdr,0x14,&uwRData);
            uwWData=(uwRData&~(0x0003<<6))|(0x0003<<6);
            EPMCHw_MIIMWrite(ubOPortAdr,0x14,uwWData);
            EPMCHw_MIIMRead(ubOPortAdr,0x14,&uwRData);
            if(uwRData!=uwWData)
            {
				timer_wait(uwSysTimers100ns, 10000);
				continue;
			}
            
            // register = 0x1E, [15:14]: LED Mode
            //                      00 = LED1 : Speed, LED0 : Link/Activity
            //                      01 = LED1 : Activity, LED0 : Link
            EPMCHw_MIIMRead(ubIPortAdr, 0x1E, &uwRData);
            uwWData=(uwRData&~0xC000)|0x4000;
            EPMCHw_MIIMWrite(ubIPortAdr, 0x1E, uwWData);
            // readback data for checking
            EPMCHw_MIIMRead(ubIPortAdr, 0x1E, &uwRData);
            if(uwRData!=uwWData)
            {
				timer_wait(uwSysTimers100ns, 10000);
				continue;
			}

            EPMCHw_MIIMRead(ubOPortAdr, 0x1E, &uwRData);
            uwWData=(uwRData&~0xC000)|0x4000;
            EPMCHw_MIIMWrite(ubOPortAdr, 0x1E, uwWData);
            // readback data for checking
            EPMCHw_MIIMRead(ubOPortAdr, 0x1E, &uwRData);
            if(uwRData!=uwWData)
            {
				timer_wait(uwSysTimers100ns, 10000);
				continue;
			}

            break;
        }
        else if((uwRData&MC_MODELNUMBER_MASK)==MC_MODELNUMBER_YT8521H)//YT8512H
    	{
	    	//set led0 turn on when link led1 blink when active 
			EPMCHw_MIIMWrite(ubOPortAdr, 0x1E, 0x40C0); // write ext reg 0x40C0 to set the led0 work model
			EPMCHw_MIIMWrite(ubOPortAdr, 0x1F, 0x0030); //  Set led0 turn on when linked or active
			EPMCHw_MIIMWrite(ubOPortAdr, 0x1E, 0x40C3); // Write ext reg 0x40C2 to set the led1 work model
			EPMCHw_MIIMWrite(ubOPortAdr, 0x1F, 0X1300); // Write ext reg 0x50 bit6 to 1

	        // readback data for checking
	        EPMCHw_MIIMWrite(ubOPortAdr, 0x1E, 0x40C0); // Read ext reg 0x40C0 
			EPMCHw_MIIMRead(ubOPortAdr, 0x1F, &uwRData); // 
	        if(uwRData!=0x0030)
	        {
	            return FALSE;
	        }

	        EPMCHw_MIIMWrite(ubOPortAdr, 0x1E, 0x40C3); // Read ext reg 0x40C0 
			EPMCHw_MIIMRead(ubOPortAdr, 0x1F, &uwRData); // 
	        if(uwRData!=0x1300)
	        {
	            return FALSE;
	        }

	        //set led0 turn on when link led1 blink when active 
			EPMCHw_MIIMWrite(ubIPortAdr, 0x1E, 0x40C0); // write ext reg 0x40C0 to set the led0 work model
			EPMCHw_MIIMWrite(ubIPortAdr, 0x1F, 0x0030); //  Set led0 turn on when linked or active
			EPMCHw_MIIMWrite(ubIPortAdr, 0x1E, 0x40C3); // Write ext reg 0x40C2 to set the led1 work model
			EPMCHw_MIIMWrite(ubIPortAdr, 0x1F, 0X1300); // Write ext reg 0x50 bit6 to 1

	        // readback data for checking
	        EPMCHw_MIIMWrite(ubIPortAdr, 0x1E, 0x40C0); // Read ext reg 0x40C0 
			EPMCHw_MIIMRead(ubIPortAdr, 0x1F, &uwRData); // 
	        if(uwRData!=0x0030)
	        {
	            return FALSE;
	        }

	        EPMCHw_MIIMWrite(ubIPortAdr, 0x1E, 0x40C3); // Read ext reg 0x40C0 
			EPMCHw_MIIMRead(ubIPortAdr, 0x1F, &uwRData); // 
	        if(uwRData!=0x1300)
	        {
	            return FALSE;
	        }
	        break;
    	}
    }
    
    // at startup, set protocol to null
    EPMCMboxSlave_SetProtocol(NULL);
    
    // set mac address
    UBYTE ubMacAddr[3];
    HwGetUID24(ubMacAddr);
    EPMCHw_MacAddr_Set(ubMacAddr);

    for(;;)
    {
        // EtherPMC SCAN Handler
        if(!EpmcCM_EPStartScan())
            EPMCMboxSlave_SetProtocol(sProtocols);
            
        // EtherPMC Master Handler
        if(bStartMaster)
        {
            bStartMaster=FALSE;
            uwTimer = timer_settimeout(uwSysTimers1ms,EPMC_SCAN_TIMEOUT); // setup timeout timer
            for(;;)
            {
                if(EpmcCM_EPStartMaster())
                    break;
                if(timer_istimedout(uwSysTimers1ms, uwTimer)) // wait until timeout
                {
                    EPMCMboxSlave_SetProtocol(sProtocols);            
                    SysLogMgm_PostAlarm(SYSTEMALARMS_BIT_EN_ETHERPMC_FAIL, SYSTEMALARMS_SUBCODE_ETHERPMC_STARTMASTER, FALSE);
                    break;
                }
            }
        }
        
        // always run slave processing protocol
        EPMCMboxSlave_Process();

        // diagnostics
        sEpmcCMDiag.ulLineDelay=EPMCHw_SyncMgr_Delay();
        sEpmcCMDiag.sFlags.b.bLink0=!((BOOL)EPMCHw_GetLinkUp(ubIPortAdr));
        sEpmcCMDiag.sFlags.b.bLink1=!((BOOL)EPMCHw_GetLinkUp(ubOPortAdr));

        //Os_Sleep(0);
        vTaskDelay(0);
    }
}

//***************************************************************************
// Start SCAN Processing
static BOOL EpmcCM_EPStartScan(void)
{
    UWORD uwScanStart = (sEpmcCMInOut.uwControl&(1<<EPMC_CONTROL_SCANSTART));
    UWORD uwScanState = (sEpmcCMInOut.uwControl&(1<<EPMC_CONTROL_SCAN));
    BOOL bRetVal=FALSE;
    
    if(uwScanState==(1<<EPMC_CONTROL_SCAN)) // SCAN
    {
        bRetVal = TRUE;
        
        if(uwEpmcCM_PrevScanState==0) // previous state is INIT, first switch to SCAN state
        {
            EPMCMboxMaster_Init();
            sEpmcCMInOut.uwStatus = EPMC_STATUS_SET(0,0,0,0,EPMCNETPROTL_STATE_SCAN); // when SCAN, Update EPStatus
        }
        
        // StartScan: bit changes from 0 --> 1
        if(uwEpmcCM_PrevScanStart==0 && uwScanStart==(1<<EPMC_CONTROL_SCANSTART)) 
        {
            sEpmcCMInOut.uwStatus = EPMC_STATUS_SET(0,0,0,1,EPMCNETPROTL_STATE_SCAN); // start scan, update EPStatus
            
            // Broadcast Command: Network state request all connected slave to INIT state
            if(!EPMCNetProtlMaster_NetStateReq(EPMCMBOX_COMMCTRL_NODE_BST,EPMCNETPROTL_STATE_INIT))
            {
                sEpmcCMInOut.uwStatus = EPMC_STATUS_SET(0,1,1,0,EPMCNETPROTL_STATE_SCAN); // scan end with error, update EPStatus
                bRetVal = FALSE;
            }
            
            // Iterate programing all connected slaves
            if(bRetVal && !EpmcCM_IterateProgram(&(sEpmcCM_Params.dev),FALSE))
            {
                sEpmcCMInOut.uwStatus = EPMC_STATUS_SET(0,1,1,0,EPMCNETPROTL_STATE_SCAN); // scan end with error, update EPStatus
                bRetVal = FALSE;
            }
            
            if(bRetVal)
                sEpmcCMInOut.uwStatus = EPMC_STATUS_SET(0,0,1,0,EPMCNETPROTL_STATE_SCAN); // scan end without error, update EPStatus
            else
                sEpmcCM_Params.dev.uwSlaves = 0; // if error, reset the number of slaves
        }
    }
    else // INIT or Slave
    {
        if((sEpmcCMInOut.uwStatus&EPMC_STATUS_NETSTATE_MASK) != EPMCNETPROTL_STATE_MASTER)
            sEpmcCMInOut.uwStatus = EPMC_STATUS_SET(EPMCMboxSlave_GetNodeNum(),0,0,0,EPMCNetProtlSlave_GetNetState()); // when SLAVE or INIT, update EPStatus    
    }
    
    uwEpmcCM_PrevScanStart = uwScanStart; // keep status of SCANSTART bit for next posedge detecting
    uwEpmcCM_PrevScanState = uwScanState; // keep statuc of SCAN
    return bRetVal;
}

//***************************************************************************
// Start Master Processing
static BOOL EpmcCM_EPStartMaster(void)
{
    UWORD uwSlaveN;
    BOOL  bRetVal=FALSE;
    UWORD uwPayloadSize=0;
    
    // Check if EtherPMC module is enabled
    if(sEpmcCM_Params.flags.f.bEnableModule)
        bRetVal = TRUE;
        
    // Check device in INIT state
    if(bRetVal &&((sEpmcCMInOut.uwStatus&EPMC_STATUS_NETSTATE_MASK) == EPMCNETPROTL_STATE_INIT))
    {        
        // Master Initial
        EPMCMboxMaster_Init();
            
        // Check number of connected slave
        if(sEpmcCM_Params.dev.uwSlaves != hpsSlaveConfig->uwSlaves)
            bRetVal = FALSE;
        
        // Broadcast Command: Network state request all connected slave to INIT state
        if(bRetVal && !EPMCNetProtlMaster_NetStateReq(EPMCMBOX_COMMCTRL_NODE_BST,EPMCNETPROTL_STATE_INIT))
            bRetVal = FALSE;
        
        // Start interate programing connected slaves
        if(bRetVal && !EpmcCM_IterateProgram(&(sEpmcCM_Params.dev),TRUE)) // iterating programing all connected slaves
            bRetVal = FALSE;
        
        if(bRetVal)
            for(uwSlaveN=0;uwSlaveN<sEpmcCM_Params.dev.uwSlaves;uwSlaveN++)
            {
                // Command: configure real time data slot
                if(!EPMCNetProtlMaster_ConfigSlot(uwSlaveN+1,hpsSlaveConfig->sSlots[uwSlaveN].uwOf,hpsSlaveConfig->sSlots[uwSlaveN].uwSz))
                {
                    bRetVal = FALSE;
                    break;
                }
                    
                // Command: configure frame timeout
                if(!EPMCNetProtlMaster_FrameTimeout(uwSlaveN+1,EPMC_FRAMEWDT_TIMEOUT))
                {
                    bRetVal = FALSE;
                    break;
                }
            }
        
        if(bRetVal)
        {
            // Calculate Frame Length and Set
            for(uwSlaveN=0;uwSlaveN<hpsSlaveConfig->uwSlaves;uwSlaveN++)
            {
                if(hpsSlaveConfig->sSlots[uwSlaveN].uwOf>=uwPayloadSize)
                    uwPayloadSize = hpsSlaveConfig->sSlots[uwSlaveN].uwOf+hpsSlaveConfig->sSlots[uwSlaveN].uwSz;
            }
            EPMCHw_PayloadLen_Set(uwPayloadSize<<1); // Set payload size to suite the system
            
            sEpmcCMInOut.uwStatus = EPMC_STATUS_SET(0,0,0,0,EPMCNETPROTL_STATE_MASTER); // switch to MASTER state
        }
        
        // Broadcast Command: network state request all connected slave to SLAVE state
        if(bRetVal && !EPMCNetProtlMaster_NetStateReq(EPMCMBOX_COMMCTRL_NODE_BST,EPMCNETPROTL_STATE_SLAVE))
            bRetVal = FALSE;
    }
    
    return bRetVal;
}

//***************************************************************************
// Itering Programing all connected slaves
static BOOL  EpmcCM_IterateProgram(EPMC_NETCONNDEVICES* psEPMCNetConnDev,BOOL bCheck)
{
    UBYTE uwIterNode;
    UWORD uwLinkStatus;
    ULONG ulUniqueID;
    
    // Start scan iterating
    for(uwIterNode=1;uwIterNode<EPMCMBOX_COMMCTRL_NODE_BST;uwIterNode++)
    {
        // Broadcast Command: set node number(itering 1 ~ 3Eh) to node 3Fh
        if(!EPMCNetProtlMaster_SetNodeNum(EPMCMBOX_COMMCTRL_NODE_BST,uwIterNode))
            return FALSE;
            
        // Command: node ident, get unique ID
        if(!EPMCNetProtlMaster_NodeIdent(uwIterNode,&ulUniqueID))
            return FALSE;
        // Check unique ID
        if(bCheck) 
        {
            if(ulUniqueID!=(psEPMCNetConnDev->ulID[uwIterNode-1]))
                return FALSE;
        }
        else
            psEPMCNetConnDev->ulID[uwIterNode-1] = ulUniqueID;
       
        // Command: get output port link status
        if(!EPMCNetProtlMaster_GetLinkStatus(uwIterNode,&uwLinkStatus))
            return FALSE;
        if((uwLinkStatus&(1<<EPMCNETPROTL_LINKSTATUS_LINK))==(1<<EPMCNETPROTL_LINKSTATUS_LINK)) // if Link is up
        {
            // Command: enable output port
            if(!EPMCNetProtlMaster_EnOutPort(uwIterNode,TRUE)) // enable output port, then next node
                return FALSE;
        }
        else // if Link is down, stop the itering
            break;
    }
    
    // Check number of slaves
    if(bCheck)
    {
        if(psEPMCNetConnDev->uwSlaves != uwIterNode)
            return FALSE;
    }
    else
        psEPMCNetConnDev->uwSlaves = uwIterNode; // record the number of connected node
        
    return TRUE;
}

//***************************************************************************
// EtherPMC Sync Management
BOOL EpmcCM_SyncEvent(void)
{
    static BOOL bSysStatEtherPMCSync=FALSE;
    static UBYTE ubSyncCnt=0;
    BOOL bFrameReceived;
    
    bFrameReceived = EPMCHw_RxFrame_Status();          // update realtime frame status for all state
           
    // Only slave trigger the event
    if((bReSyncEnabled && bEPMCMboxSlave_SyncEnable && (sEpmcCMInOut.uwStatus&EPMC_STATUS_NETSTATE_MASK)==EPMCNETPROTL_STATE_SLAVE))
    {            
        bSysStatFieldbusTSReSync = bSysStatEtherPMCSync;   // trigger resync calculation after the fbus sync cycle
        bSysStatEtherPMCSync=bSysStatFieldbusSyncing=FALSE;// reset syncing status

        if(bFrameReceived)
        {
            if(ubSyncCnt==7)
            {
                ubSyncCnt=0;
                // signal epmc resyncing
                bSysStatEtherPMCSync=bSysStatFieldbusSyncing=TRUE;
            }
            else
                ubSyncCnt = ubSyncCnt+1;            
        }
        
        // Execute Synchronization, every 8 realtime cycle
        if(bSysStatEtherPMCSync)
        {        
            union
            {
                UWORD w[2];
                ULONG l;
            } fbus,freerun;
            UWORD uwActualReload=SYNCMGR_FULLRELOAD_GET();
                             
            // get right snapshot timer
            fbus.l=FPGA_EPMCREGS_SPSOP_LO;
    
            // get actual freerun
            freerun.l=FPGA_EPMCREGS_FREERUNTMR_LO;
    
#ifdef _APP_XC
            // correct hi-res part to match local timer
            fbus.w[0]=_sint16_mpy_16_q14(fbus.w[0],uwSyncMgrAdjRTScaling);
#endif // _app_xc
            // split and correct timings
            if(fbus.w[0]>=(0xffff-uwActualReload))
                uwSysFieldbusSampleTS=0xffff;
            else
                uwSysFieldbusSampleTS=fbus.w[0]+uwActualReload;
            uwSysFieldbusSampleRT=fbus.w[1]-freerun.w[1]+uwSysFieldbusRTTimer;
        }        
    }

    // After initialization, update frame status every fast cycle
    EpmcCM_ErrorCheck(&sEpmcCMDiag);
        
    return TRUE;
}

//***************************************************************************
// EtherPMC Error Check
static void EpmcCM_ErrorCheck(EPMC_DIAG * psDiagOut)
{
    static UWORD uwWdtCounter=0;
    UWORD uwWdtTimeout=0;

    if((sEpmcCMInOut.uwStatus&EPMC_STATUS_NETSTATE_MASK)==EPMCNETPROTL_STATE_MASTER)
        uwWdtTimeout=EPMC_FRAMEWDT_TIMEOUT;
    else if((sEpmcCMInOut.uwStatus&EPMC_STATUS_NETSTATE_MASK)==EPMCNETPROTL_STATE_SLAVE)
        uwWdtTimeout=EPMCMboxSlave_GetTimeout();

    // Check watchdog expired , record in missing frame counter
    if(EPMCHw_RxFrame_Valid())
        uwWdtCounter=0; // if rx frame is reveived, feed the dog
    else
        uwWdtCounter++;  // watchdog is active
    if(uwWdtTimeout!=0)
    {
        if ( uwWdtCounter >= uwWdtTimeout)
        {
            // Watchdog is expired
            uwWdtCounter = 0;
            psDiagOut->uwMissingPacket++;
        }
    }
        
    // Check CRC Error, record in CRC error counter
    if(EPMCHw_RxFrame_Error())
        psDiagOut->uwCRCErrorCounter++;
}
#endif
