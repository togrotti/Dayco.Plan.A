/****************************************************************************/
/* Project: AxM-E Control Board                                             */
/*                                                                          */
/* Copyright © 2005,2012, Phase Motion Control. All Rights Reserved.        */
/*                                                                          */
/* File        : PlcComDBMgr.c                                              */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Common DB parameter manager for PLC user parameters        */
/*                                                                          */
/****************************************************************************/

#include <string.h>

#include "system\SysAppGlobals.h"
#include "bus\modbus\ModBusComDB.h"
#include "bus\canopen\CanOpenComDB.h"
#include "PlcComDBMgr.h"
#include "common\CommonUtility.h"
#include "plc\Plc.h"
#include "plc\PlcRT.h"
#include "common\ProgramFlashHandler.h"
#include "plc\SoftScope.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)


//***************************************************************************
// Globals

UBYTE  ubPlcComDbErrCode=0;
UWORD  uwPlcComParDefCRC=0;
PARMGM_PTRSOURCE sPlcComParCurrCheck={NULL,0};
PARMGM_PTRSOURCE sPlcComParStorCheck={NULL,0};
ULONG ulPlcComParCodeStart=0l;
ULONG ulPlcComParCodeSize=0l;

//***************************************************************************
// Defines

#define PLCDB_F_PDOMAP          0x10
#define PLCDB_F_RESETREQ        0x20
#define PLCDB_F_FLASHPAR        0x40
#define PLCDB_F_RW              0x80

#define TAG_SUBIDX              0x8000
#define SEL_SUBIDX              0x7fff

#define CRC_SEED                0xbeef

//***************************************************************************
// Data structures

typedef struct
{
    UWORD uwParWordCount;
    UWORD uwParBoolCount;
} LOCPARCHECKHEADER;

typedef struct
{
    UWORD uwCrc;

    UWORD uwDBModbusCount;
    MODBUSCOMDB_ENTRY * hpsDBModbusApp;

    UWORD uwDBCanOpenCount;
    CANOPENCOMDB_ENTRY * hpsDBCanOpenApp;

    LOCPARCHECKHEADER * hpsParCheckHeader;
} LOCDBHEADER;

typedef struct
{
    UWORD uwIECAddress;
    UWORD uwIECOffset;
    UBYTE ubType;
    UBYTE ubAddPar;
    UWORD uwNElements;
    UWORD uwModbusIPA;
    UWORD uwCOEIndex;
    UBYTE ubCOESubIndex;
    UBYTE ubDummy;
    UWORD uwNameOffset;
    UWORD uwDescrOffset;
} LOCPLCDB;

//***************************************************************************
// Locals

static const UWORD uwIECParWord[]={PLCCOMDBMGR_IEC_PAR_WS, PLCCOMDBMGR_IEC_PAR_WI};
static const UWORD uwIECParBool[]={PLCCOMDBMGR_IEC_PAR_BS, PLCCOMDBMGR_IEC_PAR_BI};

static UWORD CalcParChk(ULONG ulBaseAddress, LOCPLCDB * hpsElem);

//***************************************************************************
// Calculate unique code for the selected entry

static UWORD CalcParChk(ULONG ulBaseAddress, LOCPLCDB * hpsElem)
{
    UWORD uwCrc=CRC_SEED;
    HPUBYTE hpubString;

        // add parameter name
    hpubString=(HPUBYTE)ulBaseAddress+hpsElem->uwNameOffset;
    uwCrc=crc16(uwCrc, hpubString, strlen(hpubString));

        // add parameter type
    return crc16(uwCrc, &hpsElem->ubType, 1);
}

//***************************************************************************
// Callback from PLC in order to install the new parameter table, must be
// called at preboot; this will use an temporary memory area, this is
// taken from the sample area of the softscope
ULONG hpsPlcHeader_test =0;

UBYTE PlcComDBAppSet(ULONG ulAddress, ULONG ulRecNum)
{
    LOCDBHEADER * hpsHeader;
    const LOCDBHEADER * hpsPlcHeader;
    LOCPLCDB * hpsPlcDb;
    COMMONPARAMDB_ENTRY * hpsComDb;
    COMMONPARAMDB_ENTRY * hpsComTop;
    MODBUSCOMDB_ENTRY * hpsModbusDb;
    CANOPENCOMDB_ENTRY * hpsCanOpenDb;
    LOCPARCHECKHEADER * hpsParChkHead;
    UWORD * hpuwParChkData;
    ULONG ulTmpAreaTop,ulCt,ulCtNI;
    ULONG ulAdrOffset,ulTotRecNum;
    UWORD uwEntrySz;

        // must be executed only at preboot
    if(!bSysStatBooting)
    {
        ubPlcComDbErrCode=1;
        return 0;
    }

        // check code area definition initialization
    if(ulPlcComParCodeStart==0l || ulPlcComParCodeSize==0l)
    {
        ubPlcComDbErrCode=8;
        return 0;
    }

        // get header pointer, that is at next free segment of PLC code flash
    hpsPlcHeader=(const LOCDBHEADER *)((ulPlcComParCodeStart+ulPlcFileSize+PROGRAMFLASH_SECTOR_SIZE-1)&~(PROGRAMFLASH_SECTOR_SIZE-1));

        // initialize pointers
    hpsHeader=(LOCDBHEADER *)ulScoMemory;
    ulTmpAreaTop=(ULONG)ulScoMemory+sizeof(ulScoMemory);

        // first scan is to create modbus specific entries, scan all area
        // and add for every IPA>0 found
    hpsHeader->uwDBModbusCount=0;
    hpsHeader->hpsDBModbusApp=hpsModbusDb=(MODBUSCOMDB_ENTRY *)&hpsHeader[1];

    for(ulCt=0,hpsPlcDb=(LOCPLCDB *)ulAddress;ulCt<ulRecNum;ulCt++,hpsPlcDb++)
        if(hpsPlcDb->uwModbusIPA>0)
        {
                // setup new element and temporarily setup address as index in the table
            hpsModbusDb->uwIndex=hpsPlcDb->uwModbusIPA;
            hpsModbusDb->hpsComDBEntry=(const COMMONPARAMDB_ENTRY *)ulCt;

                // and increment location
            (hpsHeader->uwDBModbusCount)++;
            hpsModbusDb++;

                // area check
            if((ULONG)hpsModbusDb>=ulTmpAreaTop)
            {
                ubPlcComDbErrCode=2;
                return 0;
            }
        }

        // second scan is to create canopen specific entries, scan all area
        // and add for every index>0 found
    hpsHeader->uwDBCanOpenCount=0;
    hpsHeader->hpsDBCanOpenApp=hpsCanOpenDb=(CANOPENCOMDB_ENTRY *)hpsModbusDb;

    for(ulCt=0,hpsPlcDb=(LOCPLCDB *)ulAddress;ulCt<ulRecNum;ulCt++,hpsPlcDb++)
        if(hpsPlcDb->uwCOEIndex>0)
        {
            UWORD uwFlags;

                // PDO mappable is compatible only with some types
            if(hpsPlcDb->ubAddPar&PLCDB_F_PDOMAP)
                if(!( hpsPlcDb->ubType==COMMONPARAMDB_TYPE_UBYTE || hpsPlcDb->ubType==COMMONPARAMDB_TYPE_SBYTE || \
                      hpsPlcDb->ubType==COMMONPARAMDB_TYPE_UWORD || hpsPlcDb->ubType==COMMONPARAMDB_TYPE_SWORD || \
                      hpsPlcDb->ubType==COMMONPARAMDB_TYPE_ULONG || hpsPlcDb->ubType==COMMONPARAMDB_TYPE_SLONG || \
                      hpsPlcDb->ubType==COMMONPARAMDB_TYPE_UQWRD || hpsPlcDb->ubType==COMMONPARAMDB_TYPE_SQWRD || \
                      hpsPlcDb->ubType==COMMONPARAMDB_TYPE_FLOAT || hpsPlcDb->ubType==COMMONPARAMDB_TYPE_DOUBL ) )
                {
                    ubPlcComDbErrCode=6;
                    return 0;
                }

                // setup new element and temporarily setup address as index in the table
            hpsCanOpenDb->uwIndex=hpsPlcDb->uwCOEIndex;
            hpsCanOpenDb->uwSubIndex=hpsPlcDb->ubCOESubIndex;

            uwFlags=CANOPENCOMDB_F_HIDDEN|CANOPENCOMDB_F_DS301_VALID|CANOPENCOMDB_F_ECATCOE_VALID;
            if(hpsPlcDb->ubAddPar&PLCDB_F_PDOMAP)
                uwFlags|=CANOPENCOMDB_F_PDOMAPPABLE;
            if(hpsPlcDb->ubAddPar&PLCDB_F_FLASHPAR)
                uwFlags|=CANOPENCOMDB_F_PARAM;
            hpsCanOpenDb->uwFlags=uwFlags;

            hpsCanOpenDb->hpsComDBEntry=(const COMMONPARAMDB_ENTRY *)ulCt;

                // and increment location
            (hpsHeader->uwDBCanOpenCount)++;
            hpsCanOpenDb++;

                // area check
            if((ULONG)hpsCanOpenDb>=ulTmpAreaTop)
            {
                ubPlcComDbErrCode=2;
                return 0;
            }
        }

        // record actual canopen entries count for further usage
    ulCtNI=hpsHeader->uwDBCanOpenCount;

        // third scan is for subindex 0 checking, if some record with subindex != 0 without corresponding
        // subindex 0 (mandatory), then entry is created
    ulCt=0;
    while(ulCt<hpsHeader->uwDBCanOpenCount)
    {
            // for every not-zero subindex rescan all to find if already exist
        if(hpsHeader->hpsDBCanOpenApp[ulCt].uwSubIndex>0)
        {
            CANOPENCOMDB_ENTRY * hpsLDb=hpsHeader->hpsDBCanOpenApp;

                // find item with same index and zero as subindex        
            for(;hpsLDb<hpsCanOpenDb;hpsLDb++)
                if(hpsLDb->uwIndex==hpsHeader->hpsDBCanOpenApp[ulCt].uwIndex && hpsLDb->uwSubIndex==0)
                    break;

                // if not found add item and restart scan from zero
            if(hpsLDb>=hpsCanOpenDb)
            {
                hpsCanOpenDb->uwIndex=hpsHeader->hpsDBCanOpenApp[ulCt].uwIndex;
                hpsCanOpenDb->uwSubIndex=0;
                hpsCanOpenDb->uwFlags=CANOPENCOMDB_F_HIDDEN|CANOPENCOMDB_F_DS301_VALID|CANOPENCOMDB_F_ECATCOE_VALID;
                    // temporarily mark as item to be created later
                hpsCanOpenDb->hpsComDBEntry=(const COMMONPARAMDB_ENTRY *)(TAG_SUBIDX+hpsHeader->hpsDBCanOpenApp[ulCt].uwSubIndex);

                    // and increment location
                (hpsHeader->uwDBCanOpenCount)++;
                hpsCanOpenDb++;
    
                    // area check
                if((ULONG)hpsCanOpenDb>=ulTmpAreaTop)
                {
                    ubPlcComDbErrCode=2;
                    return 0;
                }

                ulCt=-1l;
            }
            else
                    // if found adjust max subindex if necessary
                if(((ULONG)hpsLDb->hpsComDBEntry&TAG_SUBIDX) && (((ULONG)hpsLDb->hpsComDBEntry&SEL_SUBIDX)<hpsHeader->hpsDBCanOpenApp[ulCt].uwSubIndex))
                    hpsLDb->hpsComDBEntry=(const COMMONPARAMDB_ENTRY *)(TAG_SUBIDX+hpsHeader->hpsDBCanOpenApp[ulCt].uwSubIndex);
        }
        ulCt++;
    }

        // fourth scan is for common db creation, process each element
    hpsComDb=hpsComTop=(COMMONPARAMDB_ENTRY *)hpsCanOpenDb;

    for(ulCt=0,hpsPlcDb=(LOCPLCDB *)ulAddress;ulCt<ulRecNum;ulCt++,hpsPlcDb++,hpsComDb++)
    {
        UBYTE ubFlags;

            // area check
        if((ULONG)hpsComDb>=ulTmpAreaTop)
        {
            ubPlcComDbErrCode=2;
            return 0;
        }

            // index not used, just dummy value
        hpsComDb->uwIndex=(UWORD)ulCt;

            // flags
        ubFlags=COMMONPARAMDB_FLAG_RD;
        if(hpsPlcDb->ubAddPar&PLCDB_F_RW)
            ubFlags|=COMMONPARAMDB_FLAG_WR;
        if(hpsPlcDb->ubAddPar&PLCDB_F_RESETREQ)
            ubFlags|=COMMONPARAMDB_FLAG_RESETREQ;
        hpsComDb->ubFlags=ubFlags;

            // type
        hpsComDb->ubType=hpsPlcDb->ubType;

            // more type
        if(hpsPlcDb->ubType==COMMONPARAMDB_TYPE_BITW)
            hpsComDb->swMoreType=(SWORD)hpsPlcDb->uwIECOffset&0xf;
        else
            hpsComDb->swMoreType=0;

            // No. of elements
        hpsComDb->uwNElements=1;

            // write denial mask selection
        hpsComDb->uWrDeny=(SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_RESETTING));

            // data pointer
        hpsComDb->hpvData=PlcIECDBEntrySearch(0,hpsPlcDb->uwIECAddress,hpsPlcDb->uwIECOffset);
        if(hpsComDb->hpvData==NULL)
        {
            ubPlcComDbErrCode=4;
            return 0;
        }

            // callback
        hpsComDb->uf.fpfHook=NULL;
    }

        // calculate offset to be added to modbus and canopen tables in order to adjust
        // pointer to common db element when the image will be in flash
    ulAdrOffset=(ULONG)hpsPlcHeader-(ULONG)hpsHeader;

        // fifth scan is for adding newly created subindex 0 elements to common db
    hpsCanOpenDb=&(hpsHeader->hpsDBCanOpenApp[ulCtNI]);

    ulTotRecNum=ulRecNum;
    for(ulCt=ulCtNI;ulCt<hpsHeader->uwDBCanOpenCount;ulCt++,hpsCanOpenDb++)
        if((ULONG)hpsCanOpenDb->hpsComDBEntry&TAG_SUBIDX)
        {
                // index not used, just dummy value
            hpsComDb->uwIndex=(UWORD)ulCt;

            hpsComDb->ubFlags=COMMONPARAMDB_FLAG_RD;
            hpsComDb->ubType=COMMONPARAMDB_TYPE_C_UBYTE;
            hpsComDb->swMoreType=0;
            hpsComDb->uwNElements=1;
            hpsComDb->uWrDeny=(SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_BOOTING)|SYSTEMSTATUS_MASK(SYSTEMSTATUS_BIT_RESETTING));
            hpsComDb->uf.fpfHook=NULL;
    
                // data pointer for constant entries is the returned value
            hpsComDb->hpvData=(HPVOID)((ULONG)hpsCanOpenDb->hpsComDBEntry&SEL_SUBIDX);

                // update location for canopen entry
            hpsCanOpenDb->hpsComDBEntry=(const COMMONPARAMDB_ENTRY *)ulTotRecNum;
    
                // and increment location
            hpsComDb++;
            ulTotRecNum++;

                // area check
            if((ULONG)hpsComDb>=ulTmpAreaTop)
            {
                ubPlcComDbErrCode=2;
                return 0;
            }
        }
    
        // sixth scan is to adjust pointer to common db for modbus entries
    hpsModbusDb=hpsHeader->hpsDBModbusApp;

    for(ulCt=0;ulCt<hpsHeader->uwDBModbusCount;ulCt++,hpsModbusDb++)
        hpsModbusDb->hpsComDBEntry=(const COMMONPARAMDB_ENTRY *)((ULONG)&hpsComTop[(ULONG)hpsModbusDb->hpsComDBEntry]+ulAdrOffset);
        
        // seventh scan is to adjust pointer to common db for canopen entries
    hpsCanOpenDb=hpsHeader->hpsDBCanOpenApp;

    for(ulCt=0;ulCt<hpsHeader->uwDBCanOpenCount;ulCt++,hpsCanOpenDb++)
        hpsCanOpenDb->hpsComDBEntry=(const COMMONPARAMDB_ENTRY *)((ULONG)&hpsComTop[(ULONG)hpsCanOpenDb->hpsComDBEntry]+ulAdrOffset);

        // eighth scan is to find and calculate check code for each word-based flash parameter
    hpsHeader->hpsParCheckHeader=hpsParChkHead=(LOCPARCHECKHEADER *)hpsComDb;
    hpuwParChkData=(UWORD *)&hpsParChkHead[1];
    hpsParChkHead->uwParWordCount=0;
    hpsParChkHead->uwParBoolCount=0;

        // zeroes remaining area in order to have holes already setup
    memset(hpuwParChkData, 0, ulTmpAreaTop-(ULONG)hpuwParChkData);

    for(ulCt=0,hpsPlcDb=(LOCPLCDB *)ulAddress;ulCt<ulRecNum;ulCt++,hpsPlcDb++)
    {
            // area check
        if((ULONG)hpuwParChkData+hpsParChkHead->uwParWordCount*sizeof(UWORD)>=ulTmpAreaTop)
        {
            ubPlcComDbErrCode=2;
            return 0;
        }

            // if IEC address match then add
        for(ulCtNI=0;ulCtNI<sizeof(uwIECParWord)/sizeof(UWORD);ulCtNI++)
            if(uwIECParWord[ulCtNI]==hpsPlcDb->uwIECAddress)
            {
                UWORD uwLocCt,uwChk;

                    // calc entry size
                switch(hpsPlcDb->ubType)
                {
                    case COMMONPARAMDB_TYPE_UBYTE:
                    case COMMONPARAMDB_TYPE_SBYTE:
                    case COMMONPARAMDB_TYPE_UWORD:
                    case COMMONPARAMDB_TYPE_SWORD:
                        uwEntrySz=1;
                        break;

                    case COMMONPARAMDB_TYPE_ULONG:
                    case COMMONPARAMDB_TYPE_SLONG:
                    case COMMONPARAMDB_TYPE_FLOAT:
                    case COMMONPARAMDB_TYPE_DOUBL:
                        uwEntrySz=2;
                        break;

// unsupported from LL environment right now                        
//                    case COMMONPARAMDB_TYPE_UQWRD:
//                    case COMMONPARAMDB_TYPE_SQWRD:
//                        uwEntrySz=4;
//                        break;
//
                    default:
                        ubPlcComDbErrCode=7;
                        return 0;
                }

                    // calculate code
                uwChk=CalcParChk(ulAddress, hpsPlcDb);

                    // then add
                for(uwLocCt=0;uwLocCt<uwEntrySz;uwLocCt++)
                    hpuwParChkData[hpsPlcDb->uwIECOffset+uwLocCt]=uwChk;

                    // update counter
                if(hpsPlcDb->uwIECOffset+uwEntrySz>=hpsParChkHead->uwParWordCount)
                    hpsParChkHead->uwParWordCount=hpsPlcDb->uwIECOffset+uwEntrySz;

                break;
            }
    }

        // ninth scan is to find and calculate check code for each bool-based flash parameter
    hpuwParChkData=(UWORD *)&hpuwParChkData[hpsParChkHead->uwParWordCount];
    for(ulCt=0,hpsPlcDb=(LOCPLCDB *)ulAddress;ulCt<ulRecNum;ulCt++,hpsPlcDb++)
    {
            // area check
        if((ULONG)hpuwParChkData+hpsParChkHead->uwParBoolCount*sizeof(UWORD)>=ulTmpAreaTop)
        {
            ubPlcComDbErrCode=2;
            return 0;
        }

            // if IEC address match then add
        for(ulCtNI=0;ulCtNI<sizeof(uwIECParBool)/sizeof(UWORD);ulCtNI++)
            if(uwIECParBool[ulCtNI]==hpsPlcDb->uwIECAddress)
            {
                    // get max index
                if(hpsPlcDb->uwIECOffset>=hpsParChkHead->uwParBoolCount)
                    hpsParChkHead->uwParBoolCount=hpsPlcDb->uwIECOffset+1;

                hpuwParChkData[hpsPlcDb->uwIECOffset]=CalcParChk(ulAddress, hpsPlcDb);
                break;
            }
    }

        // adjust main pointers and calc overall size
    hpsHeader->hpsDBModbusApp=(MODBUSCOMDB_ENTRY *)((ULONG)hpsHeader->hpsDBModbusApp+ulAdrOffset);
    hpsHeader->hpsDBCanOpenApp=(CANOPENCOMDB_ENTRY *)((ULONG)hpsHeader->hpsDBCanOpenApp+ulAdrOffset);
    hpsHeader->hpsParCheckHeader=(LOCPARCHECKHEADER *)((ULONG)hpsHeader->hpsParCheckHeader+ulAdrOffset);
    ulCtNI=(ULONG)&hpuwParChkData[hpsParChkHead->uwParBoolCount]-(ULONG)hpsHeader;

    hpsPlcHeader_test = (ULONG)hpsPlcHeader; //for test

        // check if size fit in the remaining plc program space
//    if(ulPlcComParCodeSize-((ULONG)hpsPlcHeader-ulPlcComParCodeStart)<ulCtNI)
//    {
//        ubPlcComDbErrCode=3;
//        return 0;
//    }

        // calculate CRC for flash parameters consistency checking
    hpsHeader->uwCrc=0;
    hpsHeader->uwCrc=crc16(CRC_SEED,(const UBYTE *)hpsHeader,(UWORD)ulCtNI);

        // check if flash has same, otherwise re-flash it
    if(memcmp(hpsHeader,hpsPlcHeader,(UWORD)ulCtNI))
    {
        if(ProgramFlashErase(hpsPlcHeader,ulCtNI))
        {
            ubPlcComDbErrCode=5;
            return 0;
        }
 
        if(ProgramFlashWrite(hpsPlcHeader,hpsHeader,ulCtNI))
        {
            ubPlcComDbErrCode=5;
            return 0;
        }
    }

        // setup global pointers
    uwModBusDynParamCount=hpsHeader->uwDBModbusCount;
    atomic_write(&hpsModBusDynParamTable,&hpsHeader->hpsDBModbusApp,sizeof(HPVOID));
    uwCanOpenDynParamCount=hpsHeader->uwDBCanOpenCount;       
    atomic_write(&hpsCanOpenDynParamTable,&hpsHeader->hpsDBCanOpenApp,sizeof(HPVOID));
    sPlcComParCurrCheck.hpvData=hpsHeader->hpsParCheckHeader;
    sPlcComParCurrCheck.uwDataSize=sizeof(LOCPARCHECKHEADER)+(hpsParChkHead->uwParWordCount+hpsParChkHead->uwParBoolCount)*sizeof(UWORD);

        // set par definition CRC
    uwPlcComParDefCRC=hpsHeader->uwCrc;

    return 0;
}

//***************************************************************************
// Void application parameters area

UBYTE PlcComDBAppUnset(void)
{
    HPVOID hpNul=NULL;

    atomic_write(&hpsModBusDynParamTable,&hpNul,sizeof(HPVOID));
    uwModBusDynParamCount=0;
    atomic_write(&hpsCanOpenDynParamTable,&hpNul,sizeof(HPVOID));
    uwCanOpenDynParamCount=0;

    return 0;
}

//***************************************************************************
// Triggered by restore parameters, setup saved parameters check code

SWORD PlcComDBAppRestParChk(HPVOID pvSrc, UWORD uwSize)
{
    sPlcComParStorCheck.hpvData=pvSrc;
    sPlcComParStorCheck.uwDataSize=uwSize;

    return 0;
}

//***************************************************************************
// Remap user app parameters, sizes are referred to WORD array size

UBYTE PlcComDBAppRemapPar(UWORD * puwWord, UWORD uwWordSz, UWORD * puwBool, UWORD uwBoolSz)
{
    LOCPARCHECKHEADER * hpsPCStorHead;
    LOCPARCHECKHEADER * hpsPCCurrHead;
    HPUWORD hpuTmpArea=(HPUWORD)ulScoMemory;
    HPUWORD hpuStorMap;
    HPUWORD hpuCurrMap;
    ULONG ulSz;
    UWORD uwCt,uwCtp;
    UWORD uwMsk,uwMskp;

        // check headers consistency, if one or both doesn't exist
        // then no parameter remapping is necessary; return without error
    if(sPlcComParStorCheck.hpvData==NULL || sPlcComParCurrCheck.hpvData==NULL)
        return 0;

    hpsPCStorHead=(LOCPARCHECKHEADER *)sPlcComParStorCheck.hpvData;
    hpsPCCurrHead=(LOCPARCHECKHEADER *)sPlcComParCurrCheck.hpvData;

    ulSz=sizeof(LOCPARCHECKHEADER)+((ULONG)hpsPCStorHead->uwParWordCount+(ULONG)hpsPCStorHead->uwParBoolCount)*sizeof(UWORD);
    if(ulSz!=(ULONG)sPlcComParStorCheck.uwDataSize)
        return 11;

    ulSz=sizeof(LOCPARCHECKHEADER)+((ULONG)hpsPCCurrHead->uwParWordCount+(ULONG)hpsPCCurrHead->uwParBoolCount)*sizeof(UWORD);
    if(ulSz!=(ULONG)sPlcComParCurrCheck.uwDataSize)
        return 12;

    if(hpsPCCurrHead->uwParWordCount>uwWordSz)
        return 13;
    if(hpsPCCurrHead->uwParBoolCount>uwBoolSz*16)
        return 14;

        // copy actual restored WORD data that belong to previous application
        // to the temporary memory area, then erase it
    memcpy(hpuTmpArea, puwWord, uwWordSz*sizeof(UWORD));
    memset(puwWord, 0, uwWordSz*sizeof(UWORD));

        // scan list and copy matching parameters
    for(uwCt=0, hpuCurrMap=(HPUWORD)&hpsPCCurrHead[1]; uwCt<hpsPCCurrHead->uwParWordCount; uwCt++, hpuCurrMap++)
        for(uwCtp=0, hpuStorMap=(HPUWORD)&hpsPCStorHead[1]; uwCtp<hpsPCStorHead->uwParWordCount; uwCtp++, hpuStorMap++)
            if(*hpuCurrMap==*hpuStorMap)
            {
                puwWord[uwCt]=hpuTmpArea[uwCtp];

                    // check if following have same code, if same then is 32bit value
                if(uwCt+1<hpsPCCurrHead->uwParWordCount)
                    if(hpuCurrMap[0]==hpuCurrMap[1])
                    {
                            // if current size match but stored doesn't
                        if(uwCtp+1>=hpsPCStorHead->uwParWordCount)
                            return 15;

                            // if current state 32bit but stored doesn't
                        if(hpuCurrMap[1]!=hpuStorMap[1])
                            return 16;

                        puwWord[++uwCt]=hpuTmpArea[++uwCtp];
                        hpuCurrMap++;
                        hpuStorMap++;
                    }

                break;
            }

        // copy actual restored BOOL data that belong to previous application
        // to the temporary memory area, then erase it
    memcpy(hpuTmpArea, puwBool, uwBoolSz*sizeof(UWORD));
    memset(puwBool, 0, uwBoolSz*sizeof(UWORD));

        // scan list and copy matching parameters (default is false, then copy only if true)
    for(uwCt=0, uwMsk=0x0001,hpuCurrMap=&((HPUWORD)&hpsPCCurrHead[1])[hpsPCCurrHead->uwParWordCount]; 
        uwCt<hpsPCCurrHead->uwParBoolCount; uwCt++, hpuCurrMap++, uwMsk=uwMsk==0x8000?0x0001:uwMsk<<1)
        for(uwCtp=0, uwMskp=0x0001, hpuStorMap=&((HPUWORD)&hpsPCStorHead[1])[hpsPCStorHead->uwParWordCount];
            uwCtp<hpsPCStorHead->uwParBoolCount; uwCtp++, hpuStorMap++, uwMskp=uwMskp==0x8000?0x0001:uwMskp<<1)
            if(*hpuCurrMap==*hpuStorMap)
            {
                if(hpuTmpArea[uwCtp>>4]&uwMskp)
                    puwBool[uwCt>>4]|=uwMsk;
                break;
            }

    return 0;
}
