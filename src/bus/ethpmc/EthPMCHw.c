/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCHw.c                                                 */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Driver for EtherPMC Controller                             */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

#include "common\CommonDefines.h"
#include "common\commontypedef.h"
#include "common\CommonUtility.h"
#include "system\SysAppGlobals.h"
#include "fpga\FpgaHandler.h"
#include "EthPMCHw.h"

//***************************************************************************
// Definition

#define  EPMCHw_WBUF_MAXSIZE     512  // word number
#define  EPMCHw_RBUF_MAXSIZE     512  // word number

#define  EPMCHw_FLEN_MBOXREWR    15   // Bit 15 of FLEN WORD: mBox Re-writing

#define  EPMCHw_MIIM_OP_TIMEOUT  2    // unit msec
#define  EPMCHw_MIIM_OP_READ     2
#define  EPMCHw_MIIM_OP_WRITE    1

typedef union {
    struct{
        UWORD FrameEop: 1;
        UWORD FrameError: 1;
        UWORD ALLink: 1;
        UWORD BLLink: 1;
    } flags;
    UWORD w;
} EPMCHw_RXSTATUS;

#if CFG_ETHPMC
//***************************************************************************
// Local Variables
volatile static UWORD uwEPMCHw_FrameLength=0; // Frame length
volatile static BOOL  bEPMCHw_MboxRst=FALSE;  // Mbox RST bit in Frame
volatile static BOOL  bEPMCHw_DevMode=FALSE;   // Device Mode: TRUE - Master, FALSE - Slave

volatile static EPMCHw_RXSTATUS sEPMCHw_RxStatus;

//***************************************************************************
// Locals Functions
static void EPMCHw_MasterEn(BOOL bEn);

static void EPMCHw_AutoTriggerEN(BOOL bEn);
static void EPMCHw_ManualTrigger(BOOL bEn);

static void EPMCHw_WBUF_WriteWord(HPUWORD hpuData, UWORD uwOffset, UWORD uwSize);
static void EPMCHw_RBUF_ReadWord(HPUWORD hpuData, UWORD uwOffset, UWORD uwSize);

//***************************************************************************
// MII Management Interface Read
BOOL EPMCHw_MIIMRead(UWORD uwPhyAddr,UWORD uwRegAddr,HPUWORD hpuRegRdata)
{
    UWORD uwStatus;
    BOOL bReadBusy=FALSE;
    BOOL bReadError=FALSE;
    UWORD uwTimer=0;
    
    // Set PHY address & register address & Read OP
    FPGA_EPMCREGS_MIIM_ST = (uwRegAddr<<FPGA_EPMCREGS_MIIM_ST_REGADDR) + (uwPhyAddr<<FPGA_EPMCREGS_MIIM_ST_PHYADDR) + (EPMCHw_MIIM_OP_READ<<FPGA_EPMCREGS_MIIM_ST_OP);

    // Waiting for operation start
    uwTimer = timer_settimeout(uwSysTimers100ns, 10);
    for(;;)
    {
        bReadBusy  = FPGA_REG16_GETBIT(FPGA_EPMCREGS_MIIM_ST,FPGA_EPMCREGS_MIIM_ST_BUSY);
        if(timer_istimedout(uwSysTimers100ns, uwTimer) || bReadBusy)
            break;
    }
        
    // Wait for operation done    
    uwTimer = timer_settimeout(uwSysTimers1ms, EPMCHw_MIIM_OP_TIMEOUT);
    for(;;)
    {
        uwStatus   = FPGA_EPMCREGS_MIIM_ST;
        bReadBusy  = FPGA_REG16_GETBIT(uwStatus,FPGA_EPMCREGS_MIIM_ST_BUSY);
        bReadError = FPGA_REG16_GETBIT(uwStatus,FPGA_EPMCREGS_MIIM_ST_ERROR);
        
        // When read data is ready or timeout, stop reading
        if(timer_istimedout(uwSysTimers1ms, uwTimer))
            return FALSE; // timeout
        else if(!bReadBusy)
        {
            // if no read error and not timeout, set actual data
            if(!bReadBusy && !bReadError)
            {
                *hpuRegRdata = FPGA_EPMCREGS_MIIM_DT;
                return TRUE;
            }
            else
                return FALSE; // read error
        }
    }
}

//***************************************************************************
// MII Management Interface Write
BOOL EPMCHw_MIIMWrite(UWORD uwPhyAddr, UWORD uwRegAddr, UWORD uwRegData)
{
    BOOL bWriteBusy = FALSE;
    UWORD uwTimer = 0;

    // Set wrtie data
    FPGA_EPMCREGS_MIIM_DT = uwRegData;

    // Set PHY address & register address & Write OP
    FPGA_EPMCREGS_MIIM_ST = (uwRegAddr<<FPGA_EPMCREGS_MIIM_ST_REGADDR) + (uwPhyAddr<<FPGA_EPMCREGS_MIIM_ST_PHYADDR) + (EPMCHw_MIIM_OP_WRITE<<FPGA_EPMCREGS_MIIM_ST_OP);

    // Set timeout timer
    uwTimer = timer_settimeout(uwSysTimers1ms, EPMCHw_MIIM_OP_TIMEOUT);
    
    // Waiting for operation start
    for(;;)
    {
        bWriteBusy  = FPGA_REG16_GETBIT(FPGA_EPMCREGS_MIIM_ST,FPGA_EPMCREGS_MIIM_ST_BUSY);
        if(bWriteBusy || timer_istimedout(uwSysTimers1ms, uwTimer))
            break;
    }
    
    // Waiting for operation done
    while(bWriteBusy)
    {
        bWriteBusy  = FPGA_REG16_GETBIT(FPGA_EPMCREGS_MIIM_ST,FPGA_EPMCREGS_MIIM_ST_BUSY);
        if(!bWriteBusy || timer_istimedout(uwSysTimers1ms, uwTimer))
        {
            // if timeout, return error
            if(bWriteBusy)
                return FALSE;
            else
                return TRUE;
        }
    }

    return FALSE;
}

//***************************************************************************
// Disable broadcast PHY address
void EPMCHw_DisableBroadcastAddr(void)
{
    UWORD uwPhyData;
    if(EPMCHw_MIIMRead(0x00,0x16,&uwPhyData))
        EPMCHw_MIIMWrite(0x00,0x16,uwPhyData|(1<<9)); // set 1 to bit 9 of register 16h, disable broadcast PHY address
}

//***************************************************************************
// Get Link Up Status
BOOL EPMCHw_GetLinkUp(UBYTE ubPhyAddr)
{
    BOOL  bLinkup=FALSE;

#ifdef _INFINEON_
    UWORD uwPhyData;
    
    // Read register 01h
    if(EPMCHw_MIIMRead(ubPhyAddr,0x01,&uwPhyData))
        bLinkup = ((uwPhyData&((1<<2)|(1<<5))) == ((1<<2)|(1<<5))); // Register 01h: Bit 2 - Link Status, Bit 5 - Auto-negotiation complete
    else
        return FALSE; // MII read error
#else
    if(ubPhyAddr==EPMCHw_IPORT_PHYADDR)
        bLinkup = sEPMCHw_RxStatus.flags.ALLink;
    else if(ubPhyAddr==EPMCHw_OPORT_PHYADDR)
        bLinkup = sEPMCHw_RxStatus.flags.BLLink;
    else
        bLinkup = FALSE;
#endif
    
    return bLinkup;
}

//***************************************************************************
// Master Enable
static void EPMCHw_MasterEn(BOOL bEn)
{
    FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_MATEREN,bEn);
    bEPMCHw_DevMode = bEn;
}

//***************************************************************************
// Terminal Enable/Output Port Enable
void EPMCHw_TerminalEn(BOOL bEn)
{
    // bEn:      HIGH     LOW
    // Terminal: TRUE    FALSE
    // Output:   Diable  Enable
    FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_ENDDEVEN, bEn ? FALSE : TRUE);
}

//***************************************************************************
// Device Set: Master&Slave, Terminal, Default Slot
void EPMCHw_Device_Set(UWORD uwMode)
{
    switch(uwMode)
    {
        case EPMCHw_DEVICEMODE_MT:  // master & terminal
            EPMCHw_MasterEn(TRUE);  // master enable
            EPMCHw_Slot_Set(0,EPMCHw_SLOT_MAXSIZE); // set slot offset&size
            EPMCHw_TerminalEn(TRUE);
            break;
        case EPMCHw_DEVICEMODE_MNT: // master & not terminal
            EPMCHw_MasterEn(TRUE);  // master enable
            EPMCHw_Slot_Set(0,EPMCHw_SLOT_MAXSIZE); // set slot offset&size
            EPMCHw_TerminalEn(FALSE);
            break;
        case EPMCHw_DEVICEMODE_ST:  // slave & terminal
            EPMCHw_MasterEn(FALSE); // master enable
            EPMCHw_Slot_Set(0,0); // set slot offset&size
            EPMCHw_TerminalEn(TRUE);
            break;
        case EPMCHw_DEVICEMODE_SNT: // slave & not terminal 
            EPMCHw_MasterEn(FALSE); // master enable
            EPMCHw_Slot_Set(0,0); // set slot offset&size
            EPMCHw_TerminalEn(FALSE);
            break;
        default:                    // default: master & terminal
            EPMCHw_MasterEn(TRUE);  // master enable
            EPMCHw_Slot_Set(0,EPMCHw_SLOT_MAXSIZE); // set slot offset&size
            EPMCHw_TerminalEn(TRUE);
    }
}

//***************************************************************************
// Get Device Mode
BOOL    EPMCHw_DeviceMode_Get(void)
{
    return bEPMCHw_DevMode;
}


//***************************************************************************
// Frame Auto Trigger Enable
static void EPMCHw_AutoTriggerEN(BOOL bEn)
{
    FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_AUTOTRIGGEREN,bEn);
}

//***************************************************************************
// Frame Manual Trigger
static void EPMCHw_ManualTrigger(BOOL bEn)
{
    FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_MANUALTRIGGER,bEn);
}

//***************************************************************************
// Set Frame Trigger Mode Of Master
void EPMCHw_FrameTriggerMode_Set(UWORD uwTriggerMode, BOOL bEn)
{
    /*  Trigger Mode:
        0x0: Auto trigger,   EN: 1 - trigger start, 0 - trigger stop 
        0x1: Manual trigger, EN: 1 - trigger once, 0 - no trigger
        0x2 ~ 0x3: Reserved */
    if(uwTriggerMode==EPMCHw_TRIGGERMODE_AUTO)
    {
        EPMCHw_AutoTriggerEN(bEn);
    }
    else if(uwTriggerMode==EPMCHw_TRIGGERMODE_MANU)
    {
        EPMCHw_ManualTrigger(FALSE);
        EPMCHw_ManualTrigger(bEn);   // if EN is HIGH, 0 -> 1 -> 0, manual trigger once
        EPMCHw_ManualTrigger(FALSE);
    }
}

//***************************************************************************
// PHY Enable/Disable
void EPMCHw_PhyEnable(BOOL bEn)
{
    FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_PHY_EN,bEn);
}

///////////////////////////////////////////////  BUFFER STRUCTURE /////////////////////////////////////////////////
//                                                                                                               //
// | Preamble |  DA  | SA  |  ETP  |  LEN  |  mBox  |  blank  |      slot      |      blank       |  CRC  |      //
// 0          4      7     10      11      12       20        offset           offset+size        1019    (1024) //
//                                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//***************************************************************************
// Write Words to WRITE BUFFER in FPGA
static void EPMCHw_WBUF_WriteWord(HPUWORD hpuData, UWORD uwOffset, UWORD uwSize)
{
    UWORD i;
    UWORD uwCnt;
    
    // Number is up to EPMCHw_WBUF_MAXSIZE
    if(uwOffset<EPMCHw_WBUF_MAXSIZE)
        uwCnt = uwOffset+uwSize<EPMCHw_WBUF_MAXSIZE ? uwSize : EPMCHw_WBUF_MAXSIZE;
    else
        uwCnt = 0;
    
    // Wirte Data
    for(i=0;i<uwCnt;i++)
        FPGA_BASEOFF_16(FPGA_EPMCREGS_BUF_BASEADDR,((uwOffset+i)<<1)) = hpuData[i];    
}

//***************************************************************************
// Read Words from READ BUFFER in FPGA
static void EPMCHw_RBUF_ReadWord(HPUWORD hpuData, UWORD uwOffset, UWORD uwSize)
{
    UWORD i;
    UWORD uwCnt;
    
    // Number is up to EPMCHw_RBUF_MAXSIZE
    if(uwOffset<EPMCHw_RBUF_MAXSIZE)
        uwCnt = uwOffset+uwSize<EPMCHw_RBUF_MAXSIZE ? uwSize : EPMCHw_RBUF_MAXSIZE;
    else
        uwCnt = 0;
    
    // Read Data
    for(i=0;i<uwCnt;i++)
        hpuData[i] = FPGA_BASEOFF_16(FPGA_EPMCREGS_BUF_BASEADDR,((uwOffset+i)<<1));
}

//***************************************************************************
// Set Ethernet Header: preamble, destination addres, source adress and ethernet type
void EPMCHw_EthHeader_Set(HPUWORD hpuwDA,HPUWORD hpuwSA,UWORD uwEthType)
{     
    const UWORD uwPreambleData[EPMCHw_PREAMBLE_SIZE]={0x5555,0x5555,0x5555,0x55D5};

    // Enable Ethernet Header Single Buffer in FPGA
    FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_HEADERSBUF,TRUE);
   
    // Initial Data in the Preamble Area
    EPMCHw_WBUF_WriteWord((HPUWORD)uwPreambleData,EPMCHw_PREAMBLE_BASE,EPMCHw_PREAMBLE_SIZE);    
    
    // Set destination address
    EPMCHw_WBUF_WriteWord(hpuwDA,EPMCHw_DA_BASE,EPMCHw_DA_SIZE);
 
    // Set Source Address
    // EPMCHw_WBUF_WriteWord(hpuwSA,EPMCHw_SA_BASE,EPMCHw_SA_SIZE);
 
     // Set Ethernet Type
    EPMCHw_WBUF_WriteWord((HPUWORD)&uwEthType,EPMCHw_ETP_BASE,EPMCHw_ETP_SIZE);

    // Disable Ethernet Header Single Buffer in FPGA
    FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_HEADERSBUF,FALSE);
}

//***************************************************************************
// Set mac source adress
void EPMCHw_MacAddr_Set(HPUBYTE hpubSA)
{     
    UWORD uwMacAddr[EPMCHw_SA_SIZE]=EPMCHw_SA_INITDATA;

    uwMacAddr[1] |= hpubSA[0];
    uwMacAddr[2] |= *(HPUWORD)&hpubSA[1];

    // Enable Ethernet Header Single Buffer in FPGA
    FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_HEADERSBUF,TRUE);
    
     // Set Source Address
    EPMCHw_WBUF_WriteWord(uwMacAddr,EPMCHw_SA_BASE,EPMCHw_SA_SIZE);

    // Disable Ethernet Header Single Buffer in FPGA
    FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_HEADERSBUF,FALSE);
}

//***************************************************************************
// Set Frame Length
void EPMCHw_FrameLen_Set(UWORD uwFrameSize)
{
    UWORD uwData;
    if(uwFrameSize<EPMCHw_FLEN_MIN)
        uwEPMCHw_FrameLength = EPMCHw_FLEN_MIN;
    else if(uwFrameSize>EPMCHw_FLEN_MAX)
        uwEPMCHw_FrameLength = EPMCHw_FLEN_MAX;
    else
        uwEPMCHw_FrameLength = uwFrameSize;
    
    uwData = (bEPMCHw_MboxRst ? (1<<EPMCHw_FLEN_MBOXREWR) : 0) | // The flag of mBox Rewriting is in the same word of FrameLength
             (uwEPMCHw_FrameLength-(EPMCHw_CRC_SIZE<<1));        // Frame length without CRC 
    
    EPMCHw_WBUF_WriteWord((HPUWORD)&uwData,EPMCHw_FLEN_BASE,EPMCHw_FLEN_SIZE);
}

//***************************************************************************
// Set Frame Length
void EPMCHw_PayloadLen_Set(UWORD uwPayloadSize)
{
    UWORD uwData;
    uwEPMCHw_FrameLength = uwPayloadSize+((EPMCHw_SLOT_BASE-EPMCHw_PREAMBLE_SIZE+EPMCHw_CRC_SIZE)<<1);
    if(uwEPMCHw_FrameLength<EPMCHw_FLEN_MIN)
        uwEPMCHw_FrameLength = EPMCHw_FLEN_MIN;
    else if(uwEPMCHw_FrameLength>EPMCHw_FLEN_MAX)
        uwEPMCHw_FrameLength = EPMCHw_FLEN_MAX;
    else
        uwEPMCHw_FrameLength = uwEPMCHw_FrameLength;
    
    uwData = (bEPMCHw_MboxRst ? (1<<EPMCHw_FLEN_MBOXREWR) : 0) | // The flag of mBox Rewriting is in the same word of FrameLength
             (uwEPMCHw_FrameLength-(EPMCHw_CRC_SIZE<<1));        // Frame length without CRC 
    
    EPMCHw_WBUF_WriteWord((HPUWORD)&uwData,EPMCHw_FLEN_BASE,EPMCHw_FLEN_SIZE);
}

//***************************************************************************
// Set Slot Offset & Size
BOOL EPMCHw_Slot_Set(UWORD uwOffset,UWORD uwSize)
{
    if(uwOffset+uwSize<=EPMCHw_SLOT_MAXSIZE)
    {
        FPGA_EPMCREGS_OFFSET = (EPMCHw_SLOT_BASE+uwOffset)<<1; // set slot offset(unit: BYTE) in FPGA
        FPGA_EPMCREGS_SIZE   = uwSize<<1; // set slot size(unit: BYTE) in FPGA
        return TRUE;
    }
    else
        return FALSE;
}

//***************************************************************************
// Write Mailbox Data
void EPMCHw_Mbox_WriteWord(HPUWORD hpudata, UWORD uwOffset, UWORD uwSize)
{
    UWORD uwMboxSize = uwOffset+uwSize<EPMCHw_MBOX_SIZE ? uwSize : EPMCHw_MBOX_SIZE-uwOffset;
    if(uwMboxSize>0)
        EPMCHw_WBUF_WriteWord(hpudata,EPMCHw_MBOX_BASE+uwOffset,uwMboxSize);
}

//***************************************************************************
// Read Mailbox Data
void EPMCHw_Mbox_ReadWord(HPUWORD hpudata,UWORD uwOffset,UWORD uwSize)
{
    UWORD uwMboxSize = uwOffset+uwSize<EPMCHw_MBOX_SIZE ? uwSize : EPMCHw_MBOX_SIZE-uwOffset;
    if(uwMboxSize>0)
        EPMCHw_RBUF_ReadWord(hpudata,EPMCHw_MBOX_BASE+uwOffset,uwMboxSize);
}

//***************************************************************************
// Mailbox Set: Set Mailbox RST(Master) or Enable Mailbox Re-writing(Slave)
void EPMCHw_Mbox_Set(BOOL bSet)
{
    UWORD uwData;

    if(bEPMCHw_DevMode) // When master, set re-writing rst bit in Frame Length WORD
    {
        bEPMCHw_MboxRst = bSet;
        uwData = (bEPMCHw_MboxRst ? (1<<EPMCHw_FLEN_MBOXREWR) : 0) |// The flag of mBox Rewriting is in the same word FrameLength
                 (uwEPMCHw_FrameLength-(EPMCHw_CRC_SIZE<<1));       // frame length without CRC 
        EPMCHw_WBUF_WriteWord((HPUWORD)&uwData,EPMCHw_FLEN_BASE,EPMCHw_FLEN_SIZE);    
    }
    else // When slave, set re-writing bit in FPGA SET register
    {
        if(bSet)
        {
            FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_MBOXREWR,FALSE);
            FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_MBOXREWR,TRUE);
            FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_MBOXREWR,TRUE);
        }
        else
            FPGA_REG16_SETBIT(FPGA_EPMCREGS_SET,FPGA_EPMCREGS_SET_MBOXREWR,FALSE);
    }
}

//***************************************************************************
// RX Frame Status
BOOL EPMCHw_RxFrame_Status(void)
{
    sEPMCHw_RxStatus.w = FPGA_EPMCREGS_STATUS; // read clear register, so only read once
    
    if(sEPMCHw_RxStatus.flags.FrameEop&~sEPMCHw_RxStatus.flags.FrameError)
        return TRUE; // only when end of frame without error, return true
    else
        return FALSE;
}

BOOL EPMCHw_RxFrame_Error(void)
{
    return sEPMCHw_RxStatus.flags.FrameError; // when frame received with crc error
}

BOOL EPMCHw_RxFrame_Valid(void)
{
    return sEPMCHw_RxStatus.flags.FrameEop; // when frame received without crc error
}

//***************************************************************************
// Sync Counter
ULONG EPMCHw_SyncMgr_Delay(void)
{
    return (ULONG)((float)FPGA_EPMCREGS_SPSOP_LO*12.5); // convert to nsec
}
#endif
