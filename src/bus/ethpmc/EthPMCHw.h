/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright $)AB) 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : EthPMCHw.h                                                 */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Driver for EtherPMC Controller                             */
/*                                                                          */
/****************************************************************************/

#ifndef _ETHPMCHW_H
#define _ETHPMCHW_H

//***************************************************************************
// MII Management Interface Operation
BOOL EPMCHw_MIIMRead(UWORD uwPhyAddr,UWORD uwRegAddr,HPUWORD hpuRegRdata);
BOOL EPMCHw_MIIMWrite(UWORD uwPhyAddr, UWORD uwRegAddr, UWORD uwRegData);
BOOL EPMCHw_GetLinkUp(UBYTE ubPhyAddr);
void EPMCHw_DisableBroadcastAddr(void);

#ifdef _INFINEON_
#define EPMCHw_IPORT_PHYADDR    0x00
#define EPMCHw_OPORT_PHYADDR    0x03 
#else
#ifdef _HW_VERSION_C 
#define EPMCHw_IPORT_PHYADDR    0x02
#define EPMCHw_OPORT_PHYADDR    0x03
#else
#define EPMCHw_IPORT_PHYADDR    0x01
#define EPMCHw_OPORT_PHYADDR    0x03
#endif
#endif

#define MC_MODELNUMBER_MASK     0x03F0
#define MC_MODELNUMBER_KSZ8081  0x0160
#define MC_MODELNUMBER_KSZ8061  0x0170
#define MC_MODELNUMBER_KSZ8041  0x0110
#define MC_MODELNUMBER_YT8521H  0x0120


//***************************************************************************
// Device Mode
#define EPMCHw_DEVICEMODE_MT  0 // master, terminal
#define EPMCHw_DEVICEMODE_MNT 1 // master, not terminal
#define EPMCHw_DEVICEMODE_ST  2 // slave, terminal
#define EPMCHw_DEVICEMODE_SNT 3 // slave, not ternimal

//***************************************************************************
// Device Setting: master or slave
void    EPMCHw_Device_Set(UWORD uwMode);
BOOL    EPMCHw_DeviceMode_Get(void);

//***************************************************************************
// Terminal, Enable/Disable output port
void    EPMCHw_TerminalEn(BOOL bEn);

//***************************************************************************
// Frame Trigger Mode Setting: Auto or Manual
#define EPMCHw_TRIGGERMODE_AUTO     0
#define EPMCHw_TRIGGERMODE_MANU     1
void    EPMCHw_FrameTriggerMode_Set(UWORD uwTriggerMode, BOOL bEn);

//***************************************************************************
// PHY Enable/Disable
void EPMCHw_PhyEnable(BOOL bEn);

//***************************************************************************
// Frame Data
#define EPMCHw_DA_BROADCAST {0xFFFF,0xFFFF,0xFFFF} // Destination address: FF:FF:FF:FF:FF:FF broadcast
#define EPMCHw_SA_INITDATA  {0x1C8F,0x8A00,0x0000} // Source address 1C:8F:8A:XX:XX:XX is OUI owned by Phase Motion Control SpA
#define EPMCHw_ETP_DATA      0x8A5F

//***************************************************************************
// Set Ethernet Header: preamble, destination addres, source adress and ethernet type
void    EPMCHw_EthHeader_Set(HPUWORD hpuwDA,HPUWORD hpuwSA,UWORD uwEthType);
void    EPMCHw_MacAddr_Set(HPUBYTE hpubSA);

//***************************************************************************
// Frame Length
#define EPMCHw_FLEN_MAX         1020 // Total 1024 Bytes in RAM: 8(Preamble)+14(Header)+2(FLen)+16(Mbox)+984(Slot)
#define EPMCHw_FLEN_MIN         64

#define EPMCHw_PREAMBLE_SIZE    4    // word number
#define EPMCHw_DA_SIZE          3    // word number
#define EPMCHw_SA_SIZE          3    // word number
#define EPMCHw_ETP_SIZE         1    // word number
#define EPMCHw_FLEN_SIZE        1    // word number
#define EPMCHw_CRC_SIZE         2    // word number

#define EPMCHw_PREAMBLE_BASE    0    // word address
#define EPMCHw_DA_BASE          4    // word address
#define EPMCHw_SA_BASE          7    // word address
#define EPMCHw_ETP_BASE         10   // word address
#define EPMCHw_FLEN_BASE        11   // word address

void    EPMCHw_FrameLen_Set(UWORD uwFrameSize);
void    EPMCHw_PayloadLen_Set(UWORD uwPayloadSize);

//***************************************************************************
// Mailbox
#define EPMCHw_MBOX_SIZE    8    // word number
#define EPMCHw_MBOX_BASE    12   // word address
void    EPMCHw_Mbox_Set(BOOL bSet);
void    EPMCHw_Mbox_WriteWord(HPUWORD hpudata,UWORD uwOffset,UWORD uwSize);
void    EPMCHw_Mbox_ReadWord(HPUWORD hpudata,UWORD uwOffset,UWORD uwSize);

//***************************************************************************
// Slot
#define EPMCHw_SLOT_MAXSIZE 492  // word number
#define EPMCHw_SLOT_BASE    20   // word address
BOOL    EPMCHw_Slot_Set(UWORD uwOffset,UWORD uwSize);

//***************************************************************************
// RX Frame
BOOL    EPMCHw_RxFrame_Status(void);
BOOL    EPMCHw_RxFrame_Error(void);
BOOL    EPMCHw_RxFrame_Valid(void);

//***************************************************************************
// Sync Counter
ULONG EPMCHw_SyncMgr_Delay(void);

#endif
