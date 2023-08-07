/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : AnProcessor.c                                              */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description : Analog processor manager                                   */
/*                                                                          */
/****************************************************************************/

#include "fpga\AnProcessor.h"
#include "system\SysAppGlobals.h"
#include "drive\AxM-E-Defines.h"
#include "common\CommonDefines.h"
#include "common\TaskScheduler.h"
#include "drive\HardwareConfig.h"
#include "common\CommonUtility.h"
#include "common\Int64Functions.h"
#include "fpga\FpgaHandler.h"
#include "common\UnitMeasureConversion.h"

/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

//***************************************************************************
// Defines

#define CHANSETUP_SIZE              (ANPROC_CHANNEL_SIZE)
#define CHANEXT_SIZE                (ANPROC_CHANNEL_SIZE)

//***************************************************************************
// Data structures

#define PRE_AVGSCALE_B              0
#define PRE_AVGSCALE_S              16
#define PRE_AVGSAMPLES_B            (PRE_AVGSCALE_B+PRE_AVGSCALE_S)
#define PRE_AVGSAMPLES_S            7
#define PRE_AVGTYPE_B               (PRE_AVGSAMPLES_B+PRE_AVGSAMPLES_S)
#define PRE_AVGTYPE_S               2
#define PRE_BLANKSTRATEGY_B         (PRE_AVGTYPE_B+PRE_AVGTYPE_S)
#define PRE_BLANKSTRATEGY_S         1
#define PRE_OFFSET_B                (PRE_BLANKSTRATEGY_B+PRE_BLANKSTRATEGY_S)
#define PRE_OFFSET_S                12
#define PRE_BLANKENABLE_B           (PRE_OFFSET_B+PRE_OFFSET_S)
#define PRE_BLANKENABLE_S           1
#define PRE_INSTENABLE_B            (PRE_BLANKENABLE_B+PRE_BLANKENABLE_S)
#define PRE_INSTENABLE_S            1
#define PRE_UNSCHAN_B               (PRE_INSTENABLE_B+PRE_INSTENABLE_S)
#define PRE_UNSCHAN_S               1

typedef struct
{
    UWORD uwRaw[3];
} AN_PRE;

#define SCA_EXTSHIFT_B              0
#define SCA_EXTSHIFT_S              6
#define SCA_EXTSCALER_B             (SCA_EXTSHIFT_B+SCA_EXTSHIFT_S)
#define SCA_EXTSCALER_S             16
#define SCA_EXTADDRESS_B            (SCA_EXTSCALER_B+SCA_EXTSCALER_S)
#define SCA_EXTADDRESS_S            6
#define SCA_EXTENABLE_B             (SCA_EXTADDRESS_B+SCA_EXTADDRESS_S)
#define SCA_EXTENABLE_S             1
#define SCA_INTSCALER_B             (SCA_EXTENABLE_B+SCA_EXTENABLE_S)
#define SCA_INTSCALER_S             16
#define SCA_INTADDRESS_B            (SCA_INTSCALER_B+SCA_INTSCALER_S)
#define SCA_INTADDRESS_S            6
#define SCA_INTENABLE_B             (SCA_INTADDRESS_B+SCA_INTADDRESS_S)
#define SCA_INTENABLE_S             1

typedef struct
{
	UWORD uwRaw[4];
} AN_SCALER;

#define rawset(src,sel,dst)         bitfield_set((src), sel##_B, sel##_S, (dst).uwRaw);

//****************************************************************************
// Locals

static ANPROC_CHAN  hpsChanSetup[CHANSETUP_SIZE];
static ANPROC_RGOUT hpsRgOutSetup[CHANSETUP_SIZE*2];
static AN_PRE       hpsPreSetup[CHANSETUP_SIZE];
static AN_SCALER    hpsScalerSetup[CHANSETUP_SIZE*4];
static UWORD        uwShortSz=0;
static UWORD        uwLongSz=0;
static ANPROC_TABLE hpsRefreshTable[CHANEXT_SIZE];

//****************************************************************************
// Local functions

static BOOL compiler(void);
static BOOL run(void);

//***************************************************************************
// Init handler

BOOL AnProc_Init(UWORD uwTaskParam)
{
//    UBYTE * ptr=(UBYTE *)((HPVOID)&RAMMOD_FCODE_DYN_ANPROC);
    UWORD i;

    switch(uwTaskParam)
    {
        case ANPROC_INIT_CORE:
                // by default all channels are disabled
            for(i=0;i<CHANSETUP_SIZE;i++)
                hpsChanSetup[i].ubOpt=0x00;
            for(i=0;i<CHANSETUP_SIZE*2;i++)
                hpsRgOutSetup[i].pvDst=NULL;
            for(i=0;i<CHANEXT_SIZE;i++)
                hpsRefreshTable[i].pvDst=NULL;

                // install reader before any other drive and I/O task
            return TaskSched_AddRTTask(&AnProc_ForceRefresh, TASKSCHEDULER_FLAG_NONE, 0, 0, 0);

            break;

        case ANPROC_INIT_RUN:
                // compile all channels (pointer for rt task already on the right location)
            if(!compiler())
                return FALSE;
                // and run on analog processor
            if(!run())
                return FALSE;

                // wait for a refresh of all channels before beginning realtime task
            timer_wait(uwSysTimers1ms, 1);
            break;

        default:;
            assert(FALSE);
    }

    return TRUE;
}

//***************************************************************************
// Add/modify channel setup to local list

BOOL AnProc_Set(UBYTE ubSrcTag, ANPROC_CHAN * hpsChannel)
{
        // //assert tag validity
    assert(ubSrcTag<CHANSETUP_SIZE);

        // and store setup
    memcpy(&hpsChanSetup[ubSrcTag], hpsChannel, sizeof(ANPROC_CHAN));

    return TRUE;
}

//***************************************************************************
// Get actual channel setup

BOOL AnProc_Get(UBYTE ubSrcTag, ANPROC_CHAN * hpsChannel)
{
        // //assert tag validity
    assert(ubSrcTag<CHANSETUP_SIZE);

        // and get from store
    memcpy(hpsChannel, &hpsChanSetup[ubSrcTag], sizeof(ANPROC_CHAN));

    return TRUE;
}

//***************************************************************************
// Add/modify channel output setup to local list

BOOL AnProc_RGOutSet(UBYTE ubSrcTag, ANPROC_RGOUT * hpsRgOut)
{
        // //assert tag validity
    assert(ubSrcTag>=ANPROC_RGO_TAGOFFSET && ubSrcTag<ANPROC_RGO_TAGOFFSET+CHANSETUP_SIZE*2);

        // and store setup
    ubSrcTag-=CHANSETUP_SIZE*2;
    memcpy(&hpsRgOutSetup[ubSrcTag], hpsRgOut, sizeof(ANPROC_RGOUT));

    return TRUE;
}

//***************************************************************************
// Remove channel setup from local list

BOOL AnProc_UnSet(UBYTE ubSrcTag)
{
        // //assert tag validity
    assert(ubSrcTag<CHANSETUP_SIZE);

        // disable channel
    hpsChanSetup[ubSrcTag].ubOpt=0x00;

    return TRUE;
}

//***************************************************************************
// Adjust channel offset for dynamic recalc

BOOL AnProc_AdjustOffset(UBYTE ubSrcTag, UWORD uwOffset)
{
    AN_PRE sTmp;

        // //assert tag validity
    assert(ubSrcTag<CHANSETUP_SIZE);

        // and adjust offset, both internally and on compiled data structures
    hpsChanSetup[ubSrcTag].sCal.uwOffst=uwOffset;
    memcpy(&sTmp, &hpsPreSetup[ubSrcTag], sizeof(sTmp));
    rawset(hpsChanSetup[ubSrcTag].sCal.uwOffst>>4, PRE_OFFSET, sTmp);
    memcpy(&hpsPreSetup[ubSrcTag], &sTmp, sizeof(sTmp));

    return TRUE;
}

//***************************************************************************
// Compile code for analog processor

static BOOL compiler()
{
    FLOAT f;
    UWORD i,j,k;
    UWORD uwShortIdx,uwLongIdx;
    AN_SCALER sScal;
    AN_PRE sPre;
    SLONG slMult;
    SWORD swShft,swScale;
    BOOL bValid;
    ANPROC_TABLE * psRefreshTableList=hpsRefreshTable;

        // reset istruction list arrays
    memset(hpsPreSetup, 0, sizeof(hpsPreSetup));
    memset(hpsScalerSetup, 0, sizeof(hpsScalerSetup));

        // count ext ptr short/long destinations
    for(i=0,uwShortSz=uwLongSz=0;i<CHANSETUP_SIZE;i++)
    {
        if(hpsChanSetup[i].ubOpt==0x00)
            continue;
        if(hpsChanSetup[i].ubOpt&ANPROC_OF_DST_LONG)
        {
            if(hpsChanSetup[i].pvDstExtImm)
            {
                uwLongSz++;
                psRefreshTableList->ubOpt = ANPROC_OF_DST_LONG;
                psRefreshTableList->pvDst = hpsChanSetup[i].pvDstExtImm;
                psRefreshTableList++;
            }
            if(hpsChanSetup[i].pvDstExtAvg)
            {
                uwLongSz++;
                psRefreshTableList->ubOpt = ANPROC_OF_DST_LONG;
                psRefreshTableList->pvDst = hpsChanSetup[i].pvDstExtAvg;
                psRefreshTableList++;
            }
        }
        else
        {
            if(hpsChanSetup[i].pvDstExtImm)
            {
                uwShortSz++;
                psRefreshTableList->ubOpt = ANPROC_OF_DST_SHORT;
                psRefreshTableList->pvDst = hpsChanSetup[i].pvDstExtImm;
                psRefreshTableList++;
            }
            if(hpsChanSetup[i].pvDstExtAvg)
            {
                uwShortSz++;
                psRefreshTableList->ubOpt = ANPROC_OF_DST_SHORT;
                psRefreshTableList->pvDst = hpsChanSetup[i].pvDstExtAvg;
                psRefreshTableList++;
            }
        }
    }
    for(i=0;i<CHANSETUP_SIZE*2;i++)
    {

        if(hpsRgOutSetup[i].pvDst==NULL)
            continue;
        if(hpsRgOutSetup[i].ubOpt&ANPROC_OF_DST_LONG)
        {
            uwLongSz++;
            psRefreshTableList->ubOpt = ANPROC_OF_DST_LONG;
            psRefreshTableList->pvDst = hpsRgOutSetup[i].pvDst;
            psRefreshTableList++;
        }
        else
        {
            uwShortSz++;
            psRefreshTableList->ubOpt = ANPROC_OF_DST_SHORT;
            psRefreshTableList->pvDst = hpsRgOutSetup[i].pvDst;
            psRefreshTableList++;
        }
    }
    assert(uwLongSz+uwShortSz<=CHANEXT_SIZE);

        // prepare channels
    for(i=0,uwShortIdx=0,uwLongIdx=uwShortSz;i<CHANSETUP_SIZE;i++)
    {
            // if disabled
		if (hpsChanSetup[i].ubOpt==0x00)
            continue;

            // setup analog precondition
            // options
        memset(&sPre, 0, sizeof(sPre));

        rawset(hpsChanSetup[i].ubOpt&(ANPROC_OF_AVG_1SHT|ANPROC_OF_AVG_CONT), PRE_AVGTYPE, sPre);

        rawset(0, PRE_BLANKENABLE, sPre);
        rawset(0, PRE_BLANKSTRATEGY, sPre);
        rawset(0, PRE_UNSCHAN, sPre);

        if(hpsChanSetup[i].ubOpt&(ANPROC_OF_BLK_INST|ANPROC_OF_BLK_AVG))
        {
            rawset(1, PRE_BLANKENABLE, sPre);
            if(hpsChanSetup[i].ubOpt&ANPROC_OF_BLK_AVG)
                rawset(1, PRE_BLANKSTRATEGY, sPre);
        }

        if(hpsChanSetup[i].ubOpt&ANPROC_OF_UNSIGNED)
        {
            rawset(1, PRE_UNSCHAN, sPre);
            swScale=hpsChanSetup[i].sCal.swScale*2;
        }
        else
            swScale=hpsChanSetup[i].sCal.swScale;

        rawset((hpsChanSetup[i].uwDstIntImm!=ANPROC_ADR_DISABLE||hpsChanSetup[i].pvDstExtImm)?1:0, PRE_INSTENABLE, sPre);

            // avg num sample
        assert(hpsChanSetup[i].ubNumSample<128);
        rawset(hpsChanSetup[i].ubNumSample, PRE_AVGSAMPLES, sPre);

            // offset
        rawset(hpsChanSetup[i].sCal.uwOffst>>4, PRE_OFFSET, sPre);

            // avg correction multiplier
        for(j=1;j<hpsChanSetup[i].ubNumSample;j<<=1);
        f=16384.0*j/hpsChanSetup[i].ubNumSample;
        rawset((UWORD)f, PRE_AVGSCALE, sPre);

            // copy back
        memcpy(&hpsPreSetup[i], &sPre, sizeof(sPre));

            // setup analog scaling
            // compute scalers for both immediate and average
        memset(&sScal, 0, sizeof(sScal));
        rawset(swScale, SCA_INTSCALER, sScal);

        assert(!UmConv_MultShiftCalc(hpsChanSetup[i].flScale*(FLOAT)swScale/8192.0, &slMult, &swShft, 16));
        assert(swShft>-32&&swShft<32);
        rawset((UWORD)slMult, SCA_EXTSCALER, sScal);
        if(hpsChanSetup[i].ubOpt&ANPROC_OF_DST_LONG)
            swShft-=16;
        rawset(-swShft, SCA_EXTSHIFT, sScal);

            // setup for immediate, if needed
        bValid=FALSE;
        if(hpsChanSetup[i].uwDstIntImm!=ANPROC_ADR_DISABLE)
        {
            bValid=TRUE;
            rawset(1, SCA_INTENABLE, sScal);
            rawset(hpsChanSetup[i].uwDstIntImm, SCA_INTADDRESS, sScal);
        }
        if(hpsChanSetup[i].pvDstExtImm)
        {
            bValid=TRUE;
            if(hpsChanSetup[i].ubOpt&ANPROC_OF_DST_LONG)
                k=uwLongIdx++;
            else
                k=uwShortIdx++;

//            psExtPtr[k]=hpsChanSetup[i].pvDstExtImm;
            rawset(1, SCA_EXTENABLE, sScal);
            rawset(k, SCA_EXTADDRESS, sScal);
        }
        if(bValid)
            memcpy(&hpsScalerSetup[i],&sScal,sizeof(sScal));

            // setup for average, if needed
        bValid=FALSE;
        rawset(0, SCA_INTENABLE, sScal);
        rawset(0, SCA_EXTENABLE, sScal);
        if(hpsChanSetup[i].uwDstIntAvg!=ANPROC_ADR_DISABLE)
        {
            bValid=TRUE;
            rawset(1, SCA_INTENABLE, sScal);
            rawset(hpsChanSetup[i].uwDstIntAvg, SCA_INTADDRESS, sScal);
        }
        if(hpsChanSetup[i].pvDstExtAvg)
        {
            bValid=TRUE;
            if(hpsChanSetup[i].ubOpt&ANPROC_OF_DST_LONG)
                k=uwLongIdx++;
            else
                k=uwShortIdx++;

//            psExtPtr[k]=hpsChanSetup[i].pvDstExtAvg;
            rawset(1, SCA_EXTENABLE, sScal);
            rawset(k, SCA_EXTADDRESS, sScal);
        }
        if(bValid)
            memcpy(&hpsScalerSetup[CHANSETUP_SIZE+i],&sScal,sizeof(sScal));
    }

        // prepare rgout
    for(i=0;i<CHANSETUP_SIZE*2;i++)
    {
            // if disabled
        if(hpsRgOutSetup[i].pvDst==NULL)
            continue;

            // setup analog scaling
        memset(&sScal, 0, sizeof(sScal));

        assert(!UmConv_MultShiftCalc(hpsRgOutSetup[i].flScale, &slMult, &swShft, 16));
        assert(swShft>-32&&swShft<32);
        rawset((UWORD)slMult, SCA_EXTSCALER, sScal);
        if(hpsRgOutSetup[i].ubOpt&ANPROC_OF_DST_LONG)
            swShft-=16;
        rawset(-swShft, SCA_EXTSHIFT, sScal);

            // setup for immediate
        if(hpsRgOutSetup[i].ubOpt&ANPROC_OF_DST_LONG)
            k=uwLongIdx++;
        else
            k=uwShortIdx++;

//        psExtPtr[k]=hpsRgOutSetup[i].pvDst;
        rawset(1, SCA_EXTENABLE, sScal);
        rawset(k, SCA_EXTADDRESS, sScal);

        memcpy(&hpsScalerSetup[i+CHANSETUP_SIZE*2],&sScal,sizeof(sScal));
    }

    return TRUE;
}

//***************************************************************************
// load configuration on analog processor

static BOOL run(void)
{
    UWORD i;
    UWORD * hpuwSrc;

        // load both precondition and scaler as a word stream
    hpuwSrc=(UWORD *)hpsPreSetup;
    for(i=0;i<sizeof(hpsPreSetup)/sizeof(UWORD);i++)
        FPGA_ANCOND_INS=*hpuwSrc++;

    hpuwSrc=(UWORD *)hpsScalerSetup;
    for(i=0;i<sizeof(hpsScalerSetup)/sizeof(UWORD);i++)
        FPGA_ANSCALE_INS=*hpuwSrc++;

    return TRUE;
}

//***************************************************************************
// reload just precondition module for offsets

BOOL AnProc_ReloadOffsets(void)
{
    UWORD i;
    UWORD * hpuwSrc;

        // unlock module for re-writing
    FPGA_ANCOND_INS=FPGA_ANCOND_UNLOCK_TOKEN;
    hpuwSrc=(UWORD *)hpsPreSetup;
    for(i=0;i<sizeof(hpsPreSetup)/sizeof(UWORD);i++)
        FPGA_ANCOND_INS=*hpuwSrc++;

    return TRUE;
}

//***************************************************************************
// force immediate data refresh

BOOL AnProc_ForceRefresh(void)
{
    ANPROC_TABLE * psRefreshTableList;
    ULONG          ulRefreshData[CHANSETUP_SIZE*4];
    HPULONG        hpulRefreshData;

    bSysStatAnalogRefreshing = TRUE;

    // 16-bit channel

    // Upload data from analog processor
    for(UWORD uwCnt=0;uwCnt<uwShortSz;uwCnt+=4)
        memcpy(&ulRefreshData[uwCnt], (HPUWORD)(FPGA_ANSCALE_EXTBUF_2W_BASE+uwCnt*sizeof(ULONG)), 4*sizeof(ULONG));

    hpulRefreshData=ulRefreshData;
    psRefreshTableList=hpsRefreshTable;
    while(psRefreshTableList->pvDst)
    {
        if(psRefreshTableList->ubOpt==ANPROC_OF_DST_SHORT)
            *(HPSWORD)(psRefreshTableList->pvDst) = (UWORD)*hpulRefreshData++;

        psRefreshTableList++;
    }

    // 32-bit channel

    // Upload data from analog processor
    for(UWORD uwCnt=0;uwCnt<uwLongSz*2;uwCnt+=4)
        memcpy(&ulRefreshData[uwCnt], (HPUWORD)(FPGA_ANSCALE_EXTBUF_1W_BASE+(uwCnt+uwShortSz*2)*sizeof(ULONG)), 4*sizeof(ULONG));

    hpulRefreshData=ulRefreshData;
    psRefreshTableList=hpsRefreshTable;
    while(psRefreshTableList->pvDst)
    {
        if(psRefreshTableList->ubOpt==ANPROC_OF_DST_LONG)
            *(HPSLONG)(psRefreshTableList->pvDst) = MAKELONG((UWORD)*hpulRefreshData++,(UWORD)*hpulRefreshData++);

        psRefreshTableList++;
    }

    bSysStatAnalogRefreshing = FALSE;

    return TRUE;
}

