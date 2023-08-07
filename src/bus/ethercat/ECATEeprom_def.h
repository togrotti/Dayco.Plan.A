/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ECATEeprom_def.c                                           */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : EEPROM Data Structures                                     */
/*                                                                          */
/****************************************************************************/

#ifndef _ECATEEPROM_DEF_H
#define _ECATEEPROM_DEF_H

//***************************************************************************
// fixed MANDATORY structure from UWORD 0x0000 to UWORD 0x0007 with CRC

#define ECATE2P_ADDR_FIX_ESCINFO            0x0000

typedef struct // packed
{                           // WordAddr
    UWORD PDI_Control;        // 0x0000 -> Init value for register pEsc->PDI.PDI_Control (0x0140 - 0x0141)
    UWORD PDI_Cfg;            // 0x0001 -> Init value for register pEsc->PDI.PDI_Status  (0x0150 - 0x0153)
    UWORD SyncImpLen;         // 0x0002 -> Sync Impulse in multiples of 10ns
    UWORD ResW_0x0003;        // 0x0003 -> reserved for extended PDI configuration
    UWORD ConfStatAlias;      // 0x0004 -> AliasAddress x pEsc->STATION_ADDR.ConfStatAlias (0x0012 - 0x0013)
    UWORD ResW_0x0005;        // 0x0005 -> reserved (shall be 0)
    UWORD ResW_0x0006;        // 0x0006 -> reserved (shall be 0)
    UWORD Cksum;              // 0x0007 -> checksum // for debugging purposes it is possible to disable the cksum validation with a value 0x88A4
} ECATE2P_FIX_ESCINFO;

#define ECATE2P_SZ_FIX_ESCINFO              (sizeof(ECATE2P_FIX_ESCINFO) - sizeof(UWORD))

//***************************************************************************
// fixed OPTIONAL structure from UWORD 0x0008 to UWORD 0x003F

#define ECATE2P_ADDR_FIX_SLAVEINFO          0x0010

typedef struct // packed
{                           // WordAddr
    ULONG VendorID;           // 0x0008 -> Obj 0x1018 0x01
    ULONG ProductCode;        // 0x000A -> Obj 0x1018 0x02
    ULONG RevisionNumber;     // 0x000C -> Obj 0x1018 0x03
    ULONG SerialNumber;       // 0x000E -> Obj 0x1018 0x04
    SWORD ExecDelay;          // 0x0010 -> Correction Factor for line delay in 100ps to be added if this is the last station
    SWORD Port0Delay;         // 0x0011 -> Correction Factor for line delay in 100ps to be added if master is behind Port0
    SWORD Port1Delay;         // 0x0012 -> Correction Factor for line delay in 100ps to be added if master is behind Port1
    UWORD ResW_0x0013;        // 0x0013 -> reserved (shall be 0)
    UWORD BootRxMbxOffs;      // 0x0014 -> Receive MailBox Offset for Bootstrap State (Master to Slave)
    UWORD BootRxMbxSize;      // 0x0015 -> Receive MailBox Size   for Bootstrap State (Master to Slave)
    UWORD BootTxMbxOffs;      // 0x0016 -> Send    MailBox Offset for Bootstrap State (Slave to Master)
    UWORD BootTxMbxSize;      // 0x0017 -> Send    MailBox Size   for Bootstrap State (Slave to Master)
    UWORD StdRxMbxOffs;       // 0x0018 -> Receive MailBox Offset for Standard  State (Master to Slave)
    UWORD StdRxMbxSize;       // 0x0019 -> Receive MailBox Size   for Standard  State (Master to Slave)
    UWORD StdTxMbxOffs;       // 0x001A -> Send    MailBox Offset for Standard  State (Slave to Master)
    UWORD StdTxMbxSize;       // 0x001B -> Send    MailBox Size   for Standard  State (Slave to Master)
    UWORD MbxProtocol;        // 0x001C -> MailBox Protocols supported
        #define ECATE2P_B_AOE  0x0001
        #define ECATE2P_B_EOE  0x0002
        #define ECATE2P_B_COE  0x0004
        #define ECATE2P_B_FOE  0x0008
        #define ECATE2P_B_SOE  0x0010
        #define ECATE2P_B_VOE  0x0020
    UWORD ResW_0x001D[66/2];  // 0x001D -> reserved (shall be 0)
    UWORD Size;               // 0x003E -> Size of E2Prom in Kbit-1
    UWORD Version;            // 0x003F -> This version is 1
    
} ECATE2P_FIX_SLAVEINFO;

//***************************************************************************
// header fisso x category type

#define ECATE2P_ADDR_CATEGORIES             0x0080

typedef struct // packed
{
    UWORD     CategoryType;
        #define CATTYPE_NOP         0       // 0x0000 -> no info
        #define CATTYPE_STRINGS     10      // 0x000A -> string repository for other categories
        #define CATTYPE_DATATYPE    20      // 0x0014 -> data types for future use
        #define CATTYPE_GENERAL     30      // 0x001E -> general info
        #define CATTYPE_FMMU        40      // 0x0028 -> FMMUs to be used
        #define CATTYPE_SYNCM       41      // 0x0029 -> Sync Manager Configuration
        #define CATTYPE_TXPDO       50      // 0x0032 -> TxPDO Description
        #define CATTYPE_RXPDO       51      // 0x0033 -> RxPDO Description
        #define CATTYPE_DC          60      // 0x003C -> DC Description
        #define CATTYPE_END         0xFFFF
    UWORD     WordSize;       // UWORD size of data area
} CATEGORY_HEADER;

//***************************************************************************
// STRING category (variable length)

typedef struct // packed
{
    CATEGORY_HEADER         Header;                     // category header
    UBYTE                   nStrings;                   // numero di stringhe
} CATEGORY_STRINGS_HEADER;


//***************************************************************************
// GENERAL INFO category

typedef struct // packed
{
    CATEGORY_HEADER     Header;                     // category header
    UBYTE      GroupIdx;           // Group Information       (vendor specific) - Index to Strings
    UBYTE      ImgIdx;             // Image Name              (vendor specific) - Index to Strings
    UBYTE      OrderIdx;           // Device Order Number     (vendor specific) - Index to Strings
    UBYTE      NameIdx;            // Device Name Information (vendor specific) - Index to Strings
    UBYTE      PhysicalLayerPort;  // Mask Physical Layer Ports
        #define PORT0_E_BUS                 0x00
        #define PORT0_100BASE_TX            0x01
        #define PORT0_100BASE_FX            0x02
        #define PORT1_E_BUS                 0x00
        #define PORT1_100BASE_TX            0x04
        #define PORT1_100BASE_FX            0x08
        #define PORT2_E_BUS                 0x00
        #define PORT2_100BASE_TX            0x10
        #define PORT2_100BASE_FX            0x20
        #define PORT3_E_BUS                 0x00
        #define PORT3_100BASE_TX            0x40
        #define PORT3_100BASE_FX            0x80
    UBYTE      COE_Details;
        #define B_EnaSdo                    0x01
        #define B_EnaSdoInfo                0x02
        #define B_EnaPdoAssign              0x04
        #define B_EnaPdoConfig              0x08
        #define B_EnaUpLoadAtStartUp        0x10
        #define B_EnaSdoCompleteAccess      0x20
    UBYTE      FOE_Details;
        #define B_EnableFOE                 0x01
    UBYTE      EOE_Details;
        #define B_EnableEOE                 0x01
    UBYTE      SOE_Channels;       // reserved
    UBYTE      DS402_Channels;     // reserved
    UBYTE      SysmanClass;        // reserved
    UBYTE      Flags;
        #define B_EnableSafeOp              0x01
        #define B_EnableNotLRW              0x02
    UWORD     CurrentOnEBus;      // EBus Current Consumption in mA, negative values means feeding in current
    UBYTE      PAD_Byte1[2];      // reserved
    UWORD      PhysicalPort;
        #define PPORT_NOT_USE               0x00
        #define PPORT_MII                   0x01
        #define PPORT_RESERVED              0x02
        #define PPORT_EBUS                  0x03
    UBYTE      PAD_Byte2[14];     // reserved
} CATEGORY_GENERAL;

//***************************************************************************
// FMMU Info Category

typedef struct // packed
{
    CATEGORY_HEADER     Header;                     // category header
    UBYTE      FMMU[2];
        #define FMMU_NotUsed            0x00
        #define FMMU_UsedForOutputs     0x01
        #define FMMU_UsedForInputs      0x02
        #define FMMU_UsedForSyncMStatus 0x03        // Read MailBox
} CATEGORY_FMMU;

//***************************************************************************
// Sync Manager Configuration Category

typedef struct // packed
{
  UWORD PhyStartAddr;           // 0x00 - 0x01
  UWORD Length;                 // 0x02 - 0x03
  UBYTE Ctrl;                   // 0x04
    #define SM_WRITESETTINGS            0x04                
    #define SM_READSETTINGS             0x00                
    #define SM_PDIEVENT                 0x20                
    #define THREE_BUFFER                0x00
    #define ONE_BUFFER                  0x02
    #define WATCHDOG_TRIGGER            0x40
  UBYTE Sts;                    // 0x05
    #define SM_WRITEEVENT               0x01
    #define SM_READEVENT                0x02
    #define SM_BUFFERWRITTEN            0x08
  UBYTE EnSyncMgr;              // 0x06
    #define SM_ENABLE                   0x01
    #define SM_FIXEDCONTENT             0X02
    #define SM_VIRTUALSYNCMGR           0X04
    #define SM_OPONLY                   0X08
  UBYTE SyncMgrType;            // 0x07
    #define SM_TYPE_NOTUSED             0X00
    #define SM_TYPE_MBOXOUT             0X01
    #define SM_TYPE_MBOXIN              0X02
    #define SM_TYPE_PDOUT               0X03
    #define SM_TYPE_PDIN                0X04
} SYNCMAN;

typedef struct // packed
{
    CATEGORY_HEADER     Header;                // category header

    SYNCMAN             SyncMan[4];
} CATEGORY_SYNCM;

//***************************************************************************
// DC Configuration Category

typedef struct // packed
{
    CATEGORY_HEADER     Header;                     // category header

    ULONG cycleTime0;
    ULONG shiftTime0;
    ULONG shiftTime1;
    SWORD sync1CycleFactor;
    UWORD assignActivate;
    SWORD sync0CycleFactor;
    UBYTE nameIdx;
    UBYTE descIdx;
    UBYTE reserved[4];
} CATEGORY_DC;

//***************************************************************************
// End Category

typedef struct // packed
{
    UWORD     CategoryType;                     // category header
} CATEGORY_END;

#endif