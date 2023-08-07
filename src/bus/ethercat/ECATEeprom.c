/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : ECATEeprom.c                                               */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : ecat EEPROM management                                     */
/*                                                                          */
/****************************************************************************/

#include "common\CommonDefines.h"
#include "ECATEeprom.h"
#include "ECATHw.h"
#include "ecat_def.h"
#include "ECATEeprom_def.h"
#include "ECATCommandMgr.h"
#include "bus\canopen\CanOpenDs301Externals.h"
#include "system\SysAppIdentification.h"
#include <string.h>

//***************************************************************************
// Defines

#define EEPROM_SIZE             384     // bytes

//***************************************************************************
// Globals

BOOL bECATEESaveRequested=FALSE;

//***************************************************************************
// AxM-II SII

static const ECATE2P_FIX_ESCINFO tEscInfo=
{
	0xfe08,	// Init value for register pEsc->PDI.PDI_Control (0x0140 - 0x0141)	
	0xee00,	// Init value for register pEsc->PDI.PDI_Status  (0x0150 - 0x0153)
	0x000a,	// Sync Impulse in multiples of 10ns
	0x0000,	// reserved for extended PDI configuration
	0x0000,	// AliasAddress x pEsc->STATION_ADDR.ConfStatAlias (0x0012 - 0x0013)
	0x0000,	// reserved (shall be 0)
	0x0000	// reserved (shall be 0)
};

#define DEVICE_GROUP		"Drives"
#define DEVICE_NAME			"Drive"
#define DEVICE_IMGNAME		"DRIVE"

#define DC0_NAME            "Sync"
#define DC0_DESC            "SM Synchronous"
#define DC1_NAME            "DC250μs"
#define DC1_DESC            "DC Synchronous 250 μs"

static const CATEGORY_GENERAL  tCatGeneral=
{
	{CATTYPE_GENERAL,(sizeof(CATEGORY_GENERAL)-sizeof(CATEGORY_HEADER))/2},
	2,4,1,1,
	PORT0_100BASE_TX|PORT1_100BASE_TX,
	B_EnaSdo|B_EnaSdoInfo|B_EnaPdoAssign|B_EnaPdoConfig|B_EnaUpLoadAtStartUp,
	FOE_SUPPORTED,
	EOE_SUPPORTED,
	0,
	1,
	0,
	0,
	0,
	{0,0},
	PPORT_MII|(PPORT_MII<<4)
};

static const CATEGORY_FMMU  tCatFMMU=
{
	{CATTYPE_FMMU,(sizeof(CATEGORY_FMMU)-sizeof(CATEGORY_HEADER))/2},
	{FMMU_UsedForOutputs,FMMU_UsedForInputs}
};

static const CATEGORY_SYNCM  tCatSyncM=
{
	{CATTYPE_SYNCM,(sizeof(CATEGORY_SYNCM)-sizeof(CATEGORY_HEADER))/2},
	{
		{		// Sync Mgr 0
			DEF_MBX_WRITE_ADDRESS,
			MAX_MBX_SIZE,
			SM_PDIEVENT|SM_WRITESETTINGS|ONE_BUFFER,
			0,
			SM_ECATENABLE,
			SM_TYPE_MBOXOUT
		},
		{		// Sync Mgr 1
			DEF_MBX_READ_ADDRESS,
			MAX_MBX_SIZE,
			SM_PDIEVENT|SM_READSETTINGS|ONE_BUFFER,
			0,
			SM_ECATENABLE,
			SM_TYPE_MBOXIN
		},
		{		// Sync Mgr 2
			MIN_PD_WRITE_ADDRESS,
			0,
			SM_PDIEVENT|SM_WRITESETTINGS|THREE_BUFFER,
			0,
			SM_ECATENABLE,
			SM_TYPE_PDOUT
		},
		{		// Sync Mgr 3
			MIN_PD_WRITE_ADDRESS+MAX_MBX_SIZE*3,
			0,
			SM_PDIEVENT|SM_READSETTINGS|THREE_BUFFER,
			0,
			SM_ECATENABLE,
			SM_TYPE_PDIN
		}
	}
};

static const CATEGORY_DC  tCatDC0=
{
	{CATTYPE_DC,(sizeof(CATEGORY_DC)-sizeof(CATEGORY_HEADER))/2},
    {
        0,0,0,
        -1,0x0000,0,
        5,6,
    },
};

static const CATEGORY_DC  tCatDC1=
{
	{CATTYPE_DC,(sizeof(CATEGORY_DC)-sizeof(CATEGORY_HEADER))/2},
    {
        250000,0,0,
        0,0x0300,0,
        7,8,
    },
};

//***************************************************************************
// Locals

static UBYTE  sEEPromImage[EEPROM_SIZE];

static UWORD  * puwPDIControl;
static UWORD  * puwConfStatAlias;
static SWORD  * pswExecDelay;
static SWORD  * pswPort0Delay;
static SWORD  * pswPort1Delay;

//***************************************************************************
// Local prototypes

static ULONG crcbitbybitfast(void  *, ULONG);
static SWORD escinfocheck(UBYTE  **);
static SWORD slaveinfocheck(UBYTE  **);
static SWORD catstringscheck(UBYTE  **);
static SWORD catgeneralinfocheck(UBYTE  **);
static SWORD catfmmucheck(UBYTE  **);
static SWORD catsyncmgrcheck(UBYTE  **);
static SWORD catdccheck(UBYTE  **);
static SWORD catendcheck(UBYTE  **);

//***************************************************************************
// Macrolist for EEPROM data setup

static SWORD (* const pfE2PCheckMacro[])(UBYTE  **)=
{
	&escinfocheck,
	&slaveinfocheck,
	&catstringscheck,
	&catgeneralinfocheck,
	&catfmmucheck,
	&catsyncmgrcheck,
	&catdccheck,
	&catendcheck,
};

//***************************************************************************
// CRC calc per escinfo

static ULONG crcbitbybitfast(void  * p, ULONG len)
{
	ULONG i, j, c, selbit;
	ULONG crc = 0xFF;
	ULONG crchighbit = 0x80;
	ULONG polynom = 0x07;
    UBYTE * bp = (UBYTE *)p;

	for (i=0; i<len; i++)
	{
		c = (ULONG)*bp++;

		for (j=0x80; j; j>>=1)
		{
			selbit = crc & crchighbit;
			crc <<= 1;
			if (c & j) selbit ^= crchighbit;
			if (selbit) crc ^= polynom;
		}
	}   

	return crc;
}

//***************************************************************************
// Controllo ESC info area

static SWORD escinfocheck(UBYTE  ** e2paddress)
{
	ECATE2P_FIX_ESCINFO  * tValid=(ECATE2P_FIX_ESCINFO  *)*e2paddress;
	ULONG crc;

		// copia dati predefiniti
	*tValid=tEscInfo;

        // imposto AliasAddress
    tValid->ConfStatAlias=tEcatCMParam.uwConfStatAlias;
	
		// calcolo CRC struttura valida
	crc=crcbitbybitfast(tValid, ECATE2P_SZ_FIX_ESCINFO);
	tValid->Cksum=(UWORD)(crc&0x000000ff);

        // aggiorno puntatori di riferimento
    puwPDIControl=&tValid->PDI_Control;
    puwConfStatAlias=&tValid->ConfStatAlias;
	
        // aggiorno indirizzo
	*e2paddress=&(*e2paddress)[sizeof(ECATE2P_FIX_ESCINFO)];
	return TRUE;
}

//***************************************************************************
// Controllo slave info area

static SWORD slaveinfocheck(UBYTE  ** e2paddress)
{
	ECATE2P_FIX_SLAVEINFO  * tValid=(ECATE2P_FIX_SLAVEINFO  *)*e2paddress;

		// preparo dati predefiniti
	memset(tValid, 0, sizeof(ECATE2P_FIX_SLAVEINFO));
	
	tValid->VendorID=tDs301Identity.ulVendorID;
	tValid->ProductCode=tDs301Identity.ulProductCode;
	tValid->RevisionNumber=tDs301Identity.ulRevision;
	tValid->SerialNumber=tDs301Identity.ulSerialNumber;

    tValid->ExecDelay=tEcatCMParam.swExecDelay;
    tValid->Port0Delay=tEcatCMParam.swPort0Delay;
    tValid->Port1Delay=tEcatCMParam.swPort1Delay;

	tValid->StdRxMbxOffs=DEF_MBX_WRITE_ADDRESS;
	tValid->StdRxMbxSize=MAX_MBX_SIZE;
	tValid->StdTxMbxOffs=DEF_MBX_READ_ADDRESS;
	tValid->StdTxMbxSize=MAX_MBX_SIZE;
#if EOE_SUPPORTED
	tValid->MbxProtocol|=ECATE2P_B_EOE;
#endif
#if FOE_SUPPORTED
	tValid->MbxProtocol|=ECATE2P_B_FOE;
#endif
#if COE_SUPPORTED
	tValid->MbxProtocol|=ECATE2P_B_COE;
#endif
	tValid->Size=(UWORD)(((ULONG)EEPROM_SIZE*8)/1024-1);
	tValid->Version=1;

        // aggiorno puntatori di riferimento
    pswExecDelay=&tValid->ExecDelay;
    pswPort0Delay=&tValid->Port0Delay;
    pswPort1Delay=&tValid->Port1Delay;
	
        // aggiorno indirizzo
	*e2paddress=&(*e2paddress)[sizeof(ECATE2P_FIX_SLAVEINFO)];
    return TRUE;
}

//***************************************************************************
// Controllo strings category area

#define _catstr_add(x)  	tValid->sb[spos++]=(CHARS)strlen(x); \
	                        strcpy(&tValid->sb[spos],x); spos+=tValid->sb[spos-1]; \
                            numstr++;

static SWORD catstringscheck(UBYTE  ** e2paddress)
{
	struct __cats
	{
		CATEGORY_HEADER Header;
		CHARS sb[192];
	}  * tValid = (struct __cats  *)*e2paddress;
	UWORD spos=1,numstr=0;

	tValid->Header.CategoryType=CATTYPE_STRINGS;

    _catstr_add(IDENT_FAMILY_NAME);
    _catstr_add(DEVICE_NAME);
    _catstr_add(DEVICE_GROUP);
    _catstr_add(DEVICE_IMGNAME);
    _catstr_add(DC0_NAME);
    _catstr_add(DC0_DESC);
    _catstr_add(DC1_NAME);
    _catstr_add(DC1_DESC);

        // # di stringhe
	tValid->sb[0]=(UBYTE)numstr;
	
	if(spos%2)
		spos++;
	tValid->Header.WordSize=spos/sizeof(UWORD);
	spos+=sizeof(tValid->Header);

        // aggiorno indirizzo
	*e2paddress=&(*e2paddress)[spos];
	return TRUE;
}

//***************************************************************************
// Controllo general info category area

static SWORD catgeneralinfocheck(UBYTE  ** e2paddress)
{
	CATEGORY_GENERAL  * tValid = (CATEGORY_GENERAL  *)*e2paddress;

		// preparo dati predefiniti
	*tValid=tCatGeneral;
	
        // aggiorno indirizzo
	*e2paddress=&(*e2paddress)[sizeof(CATEGORY_GENERAL)];
	return TRUE;
}

//***************************************************************************
// Controllo FMMU category area

static SWORD catfmmucheck(UBYTE  ** e2paddress)
{
	CATEGORY_FMMU  * tValid = (CATEGORY_FMMU  *)*e2paddress;

		// preparo dati predefiniti
	*tValid=tCatFMMU;
	
        // aggiorno indirizzo
	*e2paddress=&(*e2paddress)[sizeof(CATEGORY_FMMU)];
	return TRUE;
}

//***************************************************************************
// Controllo sync manager category area

static SWORD catsyncmgrcheck(UBYTE  ** e2paddress)
{
	CATEGORY_SYNCM  * tValid = (CATEGORY_SYNCM  *)*e2paddress;

		// preparo dati predefiniti
	*tValid=tCatSyncM;
	
        // aggiorno indirizzo
	*e2paddress=&(*e2paddress)[sizeof(CATEGORY_SYNCM)];
	return TRUE;
}

//***************************************************************************
// Controllo DC category area

static SWORD catdccheck(UBYTE  ** e2paddress)
{
		// dati DC 0 ed aggiorno indirizzo
    *((CATEGORY_DC  *)*e2paddress)=tCatDC0;
    *e2paddress=&(*e2paddress)[sizeof(CATEGORY_DC)];

		// dati DC 1 ed aggiorno indirizzo
    *((CATEGORY_DC  *)*e2paddress)=tCatDC1;
    *e2paddress=&(*e2paddress)[sizeof(CATEGORY_DC)];

	return TRUE;
}

//***************************************************************************
// Controllo end category area

static SWORD catendcheck(UBYTE  ** e2paddress)
{
	CATEGORY_END  * tValid = (CATEGORY_END  *)*e2paddress;

		// preparo dati predefiniti
	tValid->CategoryType=CATTYPE_END;
	
        // aggiorno indirizzo
	*e2paddress=&(*e2paddress)[sizeof(CATEGORY_END)];
	return TRUE;
}

//***************************************************************************
// Init

BOOL ECATEE_DataInit(void)
{
    UWORD cnt;
    UBYTE  * e2paddress=sEEPromImage;

        // setup EEPROM default state
    memset(sEEPromImage, (char)0xFF, sizeof(sEEPromImage));

        // fillup with data structures
	for(cnt=0;cnt<sizeof(pfE2PCheckMacro)/sizeof(pfE2PCheckMacro[0]);cnt++)
    {
		if(!(*pfE2PCheckMacro[cnt])(&e2paddress))
			return FALSE;

        if((ULONG)e2paddress > (ULONG)&sEEPromImage[sizeof(sEEPromImage)])
            return FALSE;
    }

	return TRUE;
}

//***************************************************************************
// Emulation entry point

void ECATEE_Emulation(void)
{
    USERIALEEPROMCONTROL tCtrl;
    ULONG ulAddress;
    BOOL bValid=TRUE;

        // get command register
    HW_EscReadAccess((UINT8 *)&tCtrl, ESC_EEPROM_CONTROL, sizeof(tCtrl));

        // if read/write command get and check address
    if(tCtrl.Word[0]&(ESCEE_READ|ESCEE_WRITE))
    {
            // get address (word address)
        HW_EscReadAccess((UINT8 *)&ulAddress, ESC_EEPROM_ADDRESS, sizeof(ulAddress));

            // check validity
        if(ulAddress>=(EEPROM_SIZE/sizeof(UWORD)))
        {
            tCtrl.Word[0]|=ESCEE_ERROR_ACKMISSING;
            bValid=FALSE;
        }
        else
            ulAddress*=sizeof(UWORD);
    }

        // if previous checking are good
    if(bValid)
    {
            // put requested data on ESC
        if(tCtrl.Word[0]&ESCEE_READ)
            HW_EscWriteAccess(&sEEPromImage[ulAddress], ESC_EEPROM_DATA, 4);
        
            // get requested data from ESC
        if(tCtrl.Word[0]&ESCEE_WRITE)
        {
            HW_EscReadAccess(&sEEPromImage[ulAddress], ESC_EEPROM_DATA, 4);

                // check data for NV storage
            if( *puwConfStatAlias   != tEcatCMParam.uwConfStatAlias    ||
                *pswExecDelay       != tEcatCMParam.swExecDelay        ||
                *pswPort0Delay      != tEcatCMParam.swPort0Delay       ||
                *pswPort1Delay      != tEcatCMParam.swPort1Delay )
            {
                tEcatCMParam.uwConfStatAlias=*puwConfStatAlias;
                tEcatCMParam.swExecDelay=*pswExecDelay;
                tEcatCMParam.swPort0Delay=*pswPort0Delay;
                tEcatCMParam.swPort1Delay=*pswPort1Delay;

                bECATEESaveRequested=TRUE;
            }
        }

            // reload config
        if(tCtrl.Word[0]&ESCEE_RELOAD)
        {
            UWORD reload[2];

            reload[0]=tEcatCMParam.uwConfStatAlias;
            reload[1]=((*puwPDIControl&0x0200)>>9)|((*puwPDIControl&0xf000)>>11);
                
            HW_EscWriteAccess((UINT8 *)reload, ESC_EEPROM_DATA, 4);
        }
    }

        // ackknowledge command register
    HW_EscWriteAccess((UINT8 *)&tCtrl, ESC_EEPROM_CONTROL, sizeof(tCtrl));
}
