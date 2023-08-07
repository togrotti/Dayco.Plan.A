/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : FpgaHandler.h    		                                    */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/

#ifndef _FPGAHANDLER_H
#define _FPGAHANDLER_H

#include "xparameters.h"
#include "xil_io.h"

//////////////////////////////////////////////////////////////////////////////
//                  ZYNQ PS <-> PL Programming Interface                    //
//////////////////////////////////////////////////////////////////////////////
#ifdef _INFINEON_
#define XE167_FPGA_CONFIG     P11_OUT_P0    // ==> FPGA nCONFIG
#define XE167_FPGA_STATUS     P11_IN_P1     // <== FPGA nSTATUS
#define XE167_FPGA_CONFDONE   P11_IN_P2     // <== FPGA CONF_DONE
#define XE167_FPGA_INITDONE   P11_IN_P3     // <== FPGA INIT_DONE
#define XE167_FPGA_CRCERROR   P11_IN_P4     // <== FPGA CRC_ERROR
#endif

#define FPGA_RST_CTRL_REG 0xF8000240
#define FPGA_RST_MASK     0x01


/////////////////////////////////////////////////////////////////////////////
//

SWORD FpgaInit(void) ;
SWORD FpgaLoad(ULONG  * ulLoadedBytes, void (* yield)(void)) ;
SWORD FpgaTest(void) ;
void  FpgaReset( void );
void  FpgaSafeUnLock( UWORD );
void  FpgaSafeLock( void );
BOOL  FpgaCRCService( void );
BOOL  FpgaIdentVerify( void );

/////////////////////////////////////////////////////////////////////////////
//

#define FPGA_LOAD_SUCCESSFULLY        0
#define FPGA_LOAD_ERROR_STATUS_A     -1
#define FPGA_LOAD_ERROR_STATUS_B     -2
#define FPGA_LOAD_ERROR_CONFDONE     -3
#define FPGA_LOAD_ERROR_INITDONE     -4
#define FPGA_LOAD_INVALIDDATABLOCK   -5
#define FPGA_LOAD_NOHWMATCHINGFOUND  -6
#define FPGA_LOAD_NOHWOPTREQMATCH    -7


/////////////////////////////////////////////////////////////////////////////
//

#define FPGA_TEST_SUCCESSFULLY        0
#define FPGA_TEST_ERROR_BUILD_TYPE   -1
#define FPGA_TEST_ERROR_DATA_BUS     -2
#define FPGA_TEST_ERROR_ADRS_BUS     -3
#define FPGA_TEST_ERROR_DEVICE       -4

//////////////////////////////////////////////////////////////////////////////
//

#define FPGA_WDTCHECK_NOERROR         0
#define FPGA_WDTCHECK_WDT_EXPIRED     1
#define FPGA_WDTCHECK_SAFE_LOCKED     2
#define FPGA_WDTCHECK_PLL_LOSSOFLOCK  3

//////////////////////////////////////////////////////////////////////////////
// Tokens, same as Verilog HDL macro definition

#define FPGA_SAFE_LOCK_TOKEN_RESET    0x0000
#define FPGA_SAFE_LOCK_TOKEN_0        0x9540
#define FPGA_SAFE_LOCK_TOKEN_1        0x8C20

//////////////////////////////////////////////////////////////////////////////
// Default mult/offset for ADCs, same as Verilog HDL macro definition

#define FPGA_ADCDEF_GENERIC_MUL       0x2000
#define FPGA_ADCDEF_GENERIC_OFF       0x8000

#define FPGA_ADCDEF_PWI_II_MUL        0x2000
#define FPGA_ADCDEF_PWI_II_OFF        0x8000

#define FPGA_ADCDEF_DCBUS_OFF         0x0000
#define FPGA_ADCDEF_PSPL_OFF          0x8000

//////////////////////////////////////////////////////////////////////////////
//                           FPGA I/O Registers                             //
//////////////////////////////////////////////////////////////////////////////
#ifdef XPAR_AXI_ADAPTER_0_S00_AXI_BASEADDR
#define FPGA_REGISTER_BASE_ADDRESS    XPAR_AXI_ADAPTER_0_S00_AXI_BASEADDR
#else
#define FPGA_REGISTER_BASE_ADDRESS    XPAR_AXI_ADAPTER_LITE_0_S00_AXI_BASEADDR
#endif

#define FPGA_SYSREG_BASEADDR          0x0000 /* SYS */
#define FPGA_ADCREG_BASEADDR          0x0020 /* ADC */
#define FPGA_ANCOND_BASEADDR          0x0028 /* Analog precondition */
#define FPGA_ANSCALE_BASEADDR         0x0030 /* Analog scaling */
#define FPGA_ANSCALE_BUF1W_BASEADDR   0x1400
#define FPGA_ANSCALE_BUF2W_BASEADDR   0x1500
#define FPGA_IQFLT_BASEADDR           0x0040 /* Iq filtering */
#define FPGA_PWMSETUP_BASEADDR        0x0048 /* PWM modulator */
#define FPGA_SCGEN_BASEADDR           0x0050 /* LUT Sin&Cos Generator */
#ifndef _INFINEON_
#define FPGA_CHIPID_BASEADDR          0x0060
#endif
#define FPGA_PWMREG_BASEADDR          0x0080 /* PWM */
#define FPGA_ENC_ABS_BASEADDR         0x00B8 /* Abs Encoder */
#define FPGA_USRIO_BASEADDR           0x00C0 /* User I/O */
#define FPGA_MENDAT_BASEADDR          0x0140 /* Main Endat */
#define FPGA_AENDAT_BASEADDR          0x0160 /* Auxiliary Endat */
#define FPGA_DIGENC_BASEADDR          0x0180 /* Digital Encoder */
#define FPGA_INCAUX_BASEADDR          0x01A0 /* Auxiliary Incremental Encoder */
#define FPGA_INCOUT_BASEADDR          0x01C0 /* Encoder Out */
#define FPGA_ECATREGS_BASEADDR        0x0380
#define FPGA_CPUH_BASEADDR            0x0400
#define RGAD_CPUH_DATRAM_BASEADDR     0x0500
#define RGAD_CPUH_RGURAM_BASEADDR     0x0C00
#ifdef _HW_DC
#define FPGA_EPMCREGS_BASEADDR        0x03A0 /* EtherPMC */
#define FPGA_EPMCREGS_BUF_BASEADDR    0x1000
#endif
#define FPGA_ENETREGS_BASEADDR        0x03C0 /* Ethernet/IP */

#define FPGA_CUSTOMAPP_BASEADDR       0x1000
#define FPGA_CURCHMINMAX_BASEADDR     0x0240
#define FPGA_HIPERFACE_BASEADDR       0x0280
#define FPGA_MNIKON_BASEADDR          0x02A0 /* Main Nikon Encoder */
#define FPGA_MTMGW_BASEADDR           0x02B0
#define FPGA_MBISS_BASEADDR           0x02C0 /* Main BiSS Encoder */
#define FPGA_ABISS_BASEADDR           0x02E0 /* Auxiliary BiSS Encoder */
#define FPGA_XADC_BASEADDR            0x1600
#define FPGA_VEST_BASEADDR            0x1700 /* Voltage Estimator */
#define FPGA_STANDARD_SIZE            0x1000
#define FPGA_CUSTOMAPP_SIZE           0x0400
#define FPGA_MODEXT_SIZE              0x0040
#define FPGA_CURCHMINMAX_SIZE         0x000C
#define RGAD_CPUH_DATRAM_SIZE         0x0200
#define RGAD_CPUH_RGURAM_SIZE         0x0400

#define FPGA_REGISTER_16( offset )    (*((UWORD volatile  *)(FPGA_REGISTER_BASE_ADDRESS + offset)))
#define FPGA_REGISTER_32( offset )    (*((ULONG volatile  *)(FPGA_REGISTER_BASE_ADDRESS + offset)))

#define FPGA_BASEOFF_16( base, offset ) (*((UWORD volatile  *)(FPGA_REGISTER_BASE_ADDRESS + base + offset)))
#ifdef _INFINEON_
#define FPGA_BASEOFF_32( base, offset ) (*((ULONG volatile  *)(FPGA_REGISTER_BASE_ADDRESS + base + offset)))
#else
#define FPGA_BASEOFF_32( base, offset ) MAKELONG(FPGA_BASEOFF_16(base,offset),FPGA_BASEOFF_16(base,offset+2))
#endif

#define FPGA_AT_16( adr, offset )     (*((UWORD volatile  *)((ULONG)&(adr) + (offset))))
#define FPGA_AT_32( adr, offset )     (*((ULONG volatile  *)((ULONG)&(adr) + (offset))))

#define FPGA_REG16_GETBIT( reg, bit ) (((reg)&(1u<<(bit)))==(1u<<(bit)))
#define FPGA_REG16_SETBIT( reg, bit, val ) {if(val) (reg)|=(1u<<(bit)); else (reg)&=~(1u<<(bit));}
#define FPGA_REG16_ATSETBIT( reg, bit, val ) atomic_write_bits(&(reg),(val)?(1u<<(bit)):0,(1u<<(bit)));

#define FPGA_MEMORYTEST_BASE          (FPGA_REGISTER_BASE_ADDRESS+RGAD_CPUH_DATRAM_BASEADDR)

//////////////////////////////////////////////////////////////////////////////
//                                Registers                                 //
//////////////////////////////////////////////////////////////////////////////
/* SYS: FPGA_SYSREG_BASEADDR                              0x300000 + 0x0000 */

#define FPGA_GENERIC_FAULTS           FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x0002) )
#define FPGA_RESET                    FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x0004) )
#define FPGA_OPTIONS                  FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x0006) )
#define FPGA_GENERIC_IO               FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x0008) )
#define FPGA_VENC_SETUP               FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x000A) )
#define FPGA_SAFE_LOCK                FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x000C) )
#define FPGA_WDT_PERIOD               FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x000E) )
#define FPGA_WDT_KEY                  FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x0010) )
#ifndef _HW_DC
#define FPGA_FANFLT_SET               FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x0012) )
#define FPGA_FANPWM_PERIOD            FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x0014) )
#else
#define FPGA_VAUX_SETUP               FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x0012) )
#endif
#define FPGA_EXTOPTIONS               FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x0016) )

#ifndef _HW_DC
#define FPGA_BUILD_APP                FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x001A) )
#endif
#define FPGA_BUILD_TYPE               FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x001C) )
#define FPGA_BUILD_NUMBER             FPGA_REGISTER_16( (FPGA_SYSREG_BASEADDR + 0x001E) )


//////////////////////////////////////////////////////////////////////////////
//                           FPGA ADC Registers                             //
//////////////////////////////////////////////////////////////////////////////

#define FPGA_ADC_PWI_DEL              FPGA_REGISTER_16( (FPGA_ADCREG_BASEADDR + 0x0000) )
#define FPGA_ADC_PWB_DEL              FPGA_REGISTER_16( (FPGA_ADCREG_BASEADDR + 0x0002) )


//////////////////////////////////////////////////////////////////////////////
//                        FPGA Analog Precondition Registers                //
//////////////////////////////////////////////////////////////////////////////

#define FPGA_ANCOND_INS               FPGA_REGISTER_16( (FPGA_ANCOND_BASEADDR + 0x0000) )
#define FPGA_ANCOND_UNLOCK_TOKEN      0xBEEF


//////////////////////////////////////////////////////////////////////////////
//                        FPGA Analog Scaling Registers                     //
//////////////////////////////////////////////////////////////////////////////

#define FPGA_ANSCALE_INS              FPGA_REGISTER_16( (FPGA_ANSCALE_BASEADDR + 0x0000) )
#define FPGA_ANSCALE_EXTBUF_1W        FPGA_REGISTER_16( (FPGA_ANSCALE_BASEADDR + 0x0002) )
#define FPGA_ANSCALE_EXTBUF_2W        FPGA_REGISTER_16( (FPGA_ANSCALE_BASEADDR + 0x0004) )

#define FPGA_ANSCALE_EXTBUF_1W_BASE  (FPGA_REGISTER_BASE_ADDRESS + FPGA_ANSCALE_BUF1W_BASEADDR)
#define FPGA_ANSCALE_EXTBUF_2W_BASE  (FPGA_REGISTER_BASE_ADDRESS + FPGA_ANSCALE_BUF2W_BASEADDR)

//////////////////////////////////////////////////////////////////////////////
//                           FPGA PWM Setup Registers                       //
//////////////////////////////////////////////////////////////////////////////

#define FPGA_PWMS_SET                 FPGA_REGISTER_16( (FPGA_PWMSETUP_BASEADDR + 0x0000) )

// L ==> three switches, H ==> two switches
#define FPGA_PWMS_B_2SWITCH           (0)
// L ==> two switches standard, H ==> two switches with floating fixed switch
#define FPGA_PWMS_B_2SWITCH_HL        (1)
// L ==> two switches standard, H ==> two switches low only
#define FPGA_PWMS_B_2SWITCH_LOWONLY   (2)
// L ==> as above, H ==> straight, directly drive PWM from voltages requests
#define FPGA_PWMS_B_STRAIGHT          (3)


//////////////////////////////////////////////////////////////////////////////
//                           FPGA PWM Registers                             //
//////////////////////////////////////////////////////////////////////////////
/* PWM: FPGA_PWMREG_BASEADDR                              0x300000 + 0x0080 */
#define FPGA_PWM_SET                  FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0000) )
#define FPGA_PWM_SETEX                FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0002) )
#define FPGA_PWM_MODVAL               FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0004) )
#define RGAD_PWM_OUTSHORT_VAL         FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0006) )
#define FPGA_PWM_DEADTIME             FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0008) )
#define FPGA_PWM_STATUS               FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x000A) )
#define FPGA_PWM_HOLDTIME             FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x000E) )
#define FPGA_PWM_FLT_CURRLIMIT        FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0010) )
#define FPGA_PWM_FLT_DCBUSLIMIT       FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0012) )
#define FPGA_PWM_BRAKE_LOW            FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0014) )
#define FPGA_PWM_BRAKE_HIGH           FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0016) )
#define FPGA_PWM_BLANKTIME            FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0018) )
#define FPGA_PWM_BRIDGE_LAYOUT        FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x001A) )
#define FPGA_PWM_DTCOMP_SET           FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x001C) )
#define FPGA_PWM_BRK_COUNT            FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x001E) )
#define FPGA_PWM_PHV_OFFSET           FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0020) )
#define FPGA_PWM_PHW_OFFSET           FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0022) )
#define FPGA_PWM_OC_CNTLIMIT          FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0024) )
#define FPGA_PWM_PHV_MODVAL           FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0026) )
#define FPGA_PWM_PHW_MODVAL           FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x0028) )

#define FPGA_PWM_ASC_CTRL             FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x002A) ) // ASC ConTRoL Dayco Mgmt

#define FPGA_PWM_SHIFT_1_DELAY        FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x002C) ) // Bridge 1 - PWM Shift delay (1 = 6.25ns)
#define FPGA_PWM_SHIFT_2_DELAY        FPGA_REGISTER_16( (FPGA_PWMREG_BASEADDR + 0x002E) ) // Bridge 2 - PWM Shift delay (1 = 6.25ns)

//////////////////////////////////////////////////////////////////////////////
//                        FPGA User I/O  Registers                          //
//////////////////////////////////////////////////////////////////////////////
/* User I/O:  FPGA_USRIO_BASEADDR                        0x300000 + 0x000C0 */
#define FPGA_USRIO_ANA_OUT0           FPGA_REGISTER_16( (FPGA_USRIO_BASEADDR + 0x0000) )
#define FPGA_USRIO_ANA_OUT1           FPGA_REGISTER_16( (FPGA_USRIO_BASEADDR + 0x0002) )


//////////////////////////////////////////////////////////////////////////////
//                        FPGA I/O Register bits                            //
//////////////////////////////////////////////////////////////////////////////
#define FPGA_GENERIC_FAULTS_WDT_EXPIRED     HBITW( FPGA_GENERIC_FAULTS )->bit0
#define FPGA_GENERIC_FAULTS_SAFE_LOCKED     HBITW( FPGA_GENERIC_FAULTS )->bit1
#define FPGA_GENERIC_FAULTS_PLL_LOSSOFLOCK  HBITW( FPGA_GENERIC_FAULTS )->bit2

#define FPGA_GENERIC_IO_FAN_DRIVE           HBITW( FPGA_GENERIC_IO )->bit0
#define FPGA_GENERIC_IO_RLY_DRIVE           HBITW( FPGA_GENERIC_IO )->bit1
#define FPGA_GENERIC_IO_FAN_FAULT           HBITW( FPGA_GENERIC_IO )->bit2
#define FPGA_GENERIC_IOEXP_CONFDONE         HBITW( FPGA_GENERIC_IO )->bit3


//////////////////////////////////////////////////////////////////////////////
//                        PWM Setup and Status                              //
//////////////////////////////////////////////////////////////////////////////
// L ==> Disable PWM, H ==> Enable PWM
#define FPGA_PWM_B_SETUP_ENABLE                (0)
// L-H-L == > Reset Faults
#define FPGA_PWM_B_SETUP_RESET_FAULT           (1)
// L ==> Direct PWM, H ==> Use Modulator
#define FPGA_PWM_B_SETUP_USE_MODULATOR         (2)
// L ==> Normal Outputs, H ==> Inverted Outputs (Direct PWM Only)
#define FPGA_PWM_B_SETUP_INVERT_OUT_U          (3)
#define FPGA_PWM_B_SETUP_INVERT_OUT_V          (4)
#define FPGA_PWM_B_SETUP_INVERT_OUT_W          (5)
#define FPGA_PWM_B_BRIDGE2_OVERCURR            (6)
// L ==> Disabled, H ==> Enabled
#define FPGA_PWM_B_SETUP_BRAKE_ENABLE          (7)
// L ==> Disabled, H ==> Enabled
#define FPGA_PWM_B_4KHZ_2SWITCH                (8)
// L ==> Brake active LOW, H ==> Brake active HIGH
#define FPGA_PWM_B_BRAKE_ACTIVE_HIGH           (9)
// L ==> standard op, H ==> output L IGBT short in place of overvoltage fault
#define FPGA_PWM_B_OV_TO_OUTSHORT              (10)

#define FPGA_PWM_B_BRIDGE1_OVERCURR            (11)
#define FPGA_PWM_B_ENABLE_OUTPADS              (12)
#define FPGA_PWM_B_DTCOMP_CUR_SIGN             (13)
#define FPGA_PWM_B_ENABLE_HOLD_SIGNALS         (14)
#define FPGA_PWM_B_EN_ACTIVESHORTCIRCUIT       (15) // ASC ConTRoL Dayco Mgmt

#define FPGA_PWM_B_SETEX_ENERGYPROT            (0)
#define FPGA_PWM_B_SETEX_TRANSFMOD             (1)
#define FPGA_PWM_B_SETEX_SELDISOUT             (2)

#define FPGA_PWM_B_SETEX_EN_FREQSEL_BASE       (4)
#define FPGA_PWM_B_SETEX_EN_FREQSEL_MASK       (0x0070)

#define FPGA_PWM_B_SETEX_PHASESHIFT            (7)
#define FPGA_PWM_B_SETEX_DISONCHANGE           (8)
#define FPGA_PWM_B_SETEX_EN_DISOCPWM           (9)
#define FPGA_PWM_B_SETEX_EN_CHARGERMODE        (10)

#define FPGA_PWM_STATUS_FAULT_ACTIVE(x)        HBITW( (x) )->bit0
#define FPGA_PWM_STATUS_ENABLED(x)             HBITW( (x) )->bit1
#define FPGA_PWM_STATUS_FULLY_ACTIVE(x)        HBITW( (x) )->bit2
#define FPGA_PWM_STATUS_OVER_CURRENT(x)        HBITW( (x) )->bit3
#define FPGA_PWM_STATUS_OVER_VOLTAGE(x)        HBITW( (x) )->bit4
#define FPGA_PWM_STATUS_DESATURATION(x)        HBITW( (x) )->bit5
#define FPGA_PWM_STATUS_BRAKE_DESAT(x)         HBITW( (x) )->bit6
#define FPGA_PWM_STATUS_BRAKE_ACTIVE(x)        HBITW( (x) )->bit7
#define FPGA_PWM_STATUS_FLT_LAYOUTERROR(x)     HBITW( (x) )->bit8
#define FPGA_PWM_STATUS_EXT_FAULT(x)           HBITW( (x) )->bit9
#define FPGA_PWM_STATUS_OUTSHORT_ACTIVE(x)     HBITW( (x) )->bit10
#define FPGA_PWM_STATUS_ADC_WDT_EXP(x)         HBITW( (x) )->bit12

#define FPGA_PWM_STATUS_B_FREQSEL_BASE         (13)
#define FPGA_PWM_STATUS_B_FREQSEL_MASK         (0xE000)

#define FPGA_PWM_FREQ_8KHZ                     (0)
#define FPGA_PWM_FREQ_2KHZ                     (1)
#define FPGA_PWM_FREQ_4KHZ                     (2)
#define FPGA_PWM_FREQ_16KHZ                    (4)
#define FPGA_PWM_FREQ_32KHZ                    (5)
#define FPGA_PWM_FREQ_64KHZ                    (6)

#define FPGA_PWM_FREQMODULO                    (2500)

// ASC ConTRoL Dayco Mgmt
#define FPGA_PWM_ASC_CTRL_ENABLE                (0) // 1: enable Antonio ASC mgmt
#define FPGA_PWM_ASC_CTRL_USE_HIGH              (1) // 1: use high IGBTs; 0: use low IGBTs
#define FPGA_PWM_ASC_CTRL_STS_ACTIVE(x)         HBITW( (x) )->bit2
#define FPGA_PWM_ASC_CTRL_RELEASE_ASC           (3) // L->H: release ASC
#define FPGA_PWM_ASC_CTRL_STS                   (0x00F0) // 0: Idle; 1: Normal_Hi; 2: Normal_Lo; 3: ASC_Hi; 4: ASC_Lo
//////////////////////////////////////////////////////////////////////////////
//                        Iq filter module                                  //
//////////////////////////////////////////////////////////////////////////////

#define FPGA_IQFLT_SET                         FPGA_REGISTER_16( (FPGA_IQFLT_BASEADDR + 0x0000) )
#define FPGA_IQFLT_IQREF                       FPGA_REGISTER_16( (FPGA_IQFLT_BASEADDR + 0x0002) )

// Iq Ref filter selection / no. of filters
#define FPGA_IQFLT_MASK_FLT_SEL                (0x0003)
// Iq Ref filter configuration mode
#define FPGA_IQFLT_MASK_FLT_CFG                (0x0004)

//////////////////////////////////////////////////////////////////////////////
//                        LUT Sin&Cos Generator                             //
//////////////////////////////////////////////////////////////////////////////

#define FPGA_SCGEN_ANGLEBASE                   FPGA_REGISTER_16( (FPGA_SCGEN_BASEADDR + 0x0000) )
#define FPGA_SCGEN_ANGLEINC                    FPGA_REGISTER_16( (FPGA_SCGEN_BASEADDR + 0x0002) )

#ifndef _INFINEON_
//////////////////////////////////////////////////////////////////////////////
//                        Chip ID                                           //
//////////////////////////////////////////////////////////////////////////////
#define FPGA_CHIPID_LL                         FPGA_REGISTER_16( (FPGA_CHIPID_BASEADDR + 0x0000) )
#define FPGA_CHIPID_LH                         FPGA_REGISTER_16( (FPGA_CHIPID_BASEADDR + 0x0002) )
#define FPGA_CHIPID_HL                         FPGA_REGISTER_16( (FPGA_CHIPID_BASEADDR + 0x0004) )
#define FPGA_CHIPID_HH                         FPGA_REGISTER_16( (FPGA_CHIPID_BASEADDR + 0x0006) )
#endif

//////////////////////////////////////////////////////////////////////////////
//                               FPGA Options                               //
//////////////////////////////////////////////////////////////////////////////
/* Encoder Absolute Channel Gain (1=LOW/0=HIGH) */
#define FPGA_ENCODER_OPTIONS_GAIN     HBITW( FPGA_OPTIONS )->bit0

/* Encoder Supply (0=OFF/1=ON) */
#define FPGA_ENCODER_OPTIONS_SUPPLY   HBITW( FPGA_OPTIONS )->bit1

#ifndef _HW_DC
/* PWB ADC CLK / DATA / CS Negated Lines (0=New Board / 1=Old Board) Default New Board */
#define FPGA_PWB_ADC_NEGATE_SIGNALS   HBITW( FPGA_OPTIONS )->bit2

#ifndef _APP_XC
/* Enable I/O for hw rev 2.00 */
#define FPGA_CONTROLBOARD_REV2        HBITW( FPGA_OPTIONS )->bit3

#else
/* Enable I/O for hw rev < 6.00 */
#define FPGA_CONTROLBOARD_LT_REV6     HBITW( FPGA_OPTIONS )->bit3
#endif

/* AUX_ENC_ENA (AUX_ENC_REN) */
#define FPGA_ENCODER_OPTIONS_AUXENA   HBITW( FPGA_OPTIONS )->bit4

/* AUX_ENC_ENB (AUX_ENC_DEN) */
#define FPGA_ENCODER_OPTIONS_AUXENB   HBITW( FPGA_OPTIONS )->bit5

/* Enable desat from instant currents */
#define FPGA_ENABLE_DESATFROMCURR     HBITW( FPGA_OPTIONS )->bit6

/* IO Expansion board nCONFIG control */
#define FPGA_IOEXP_NCONFIG            HBITW( FPGA_OPTIONS )->bit7

/* Main Endat Encoder Enable */
#define FPGA_ENCODER_OPTIONS_MENDEN   HBITW( FPGA_OPTIONS )->bit8

/* Aux Endat Encoder Enable */
#define FPGA_ENCODER_OPTIONS_AENDEN   HBITW( FPGA_OPTIONS )->bit9

// Negated to PWB_BOARD_TYPE (PWB_ADC_CK and PWB_ADC_PSPL_U enable)
#define FPGA_OPTIONS_POWER_BOARD_TYPE HBITW( FPGA_OPTIONS )->bit10

// Select old-type analog board and activate outputs
#define FPGA_OPTIONS_PWB_ANALOG_BRD   HBITW( FPGA_OPTIONS )->bit11

// Select new interface board and activate outputs
#define FPGA_OPTIONS_PWB_DIGITAL_BRD  HBITW( FPGA_OPTIONS )->bit12

// Select external ADC for currents (valid only with new interface)
#define FPGA_OPTIONS_PWB_EXT_IADC     HBITW( FPGA_OPTIONS )->bit13

// Select true Iw reading and mixing rather than sum
#define FPGA_OPTIONS_PWB_SEL_TRUE_IW  HBITW( FPGA_OPTIONS )->bit14

#else

// Auxiliary Encoder Supply (0=OFF/1=ON)
#define FPGA_AUXENC_OPTIONS_SUPPLY    HBITW( FPGA_OPTIONS )->bit2

// Main Endat Encoder Enable
#define FPGA_ENCODER_OPTIONS_MENDEN   HBITW( FPGA_OPTIONS )->bit3

// Auxiliary encoder output enable
#define FPGA_ENCODER_OPTIONS_AUXOUTEN HBITW( FPGA_OPTIONS )->bit4

// Enable BRIDGE #1
#define FPGA_BRIDGE1_ENABLE           HBITW( FPGA_OPTIONS )->bit5

// Enable BRIDGE #2
#define FPGA_BRIDGE2_ENABLE           HBITW( FPGA_OPTIONS )->bit6

#endif

// Hiperface Encoder Enable
#define FPGA_ENCODER_OPTIONS_HIPFC    HBITW( FPGA_EXTOPTIONS )->bit0
#define FPGA_ENCODER_OPTIONS_MNIKON   HBITW( FPGA_EXTOPTIONS )->bit1
#define FPGA_ENCODER_OPTIONS_TMGW     HBITW( FPGA_EXTOPTIONS )->bit3
#define FPGA_ENCODER_OPTIONS_MBISS    HBITW( FPGA_EXTOPTIONS )->bit4
#define FPGA_ENCODER_OPTIONS_ABISS    HBITW( FPGA_EXTOPTIONS )->bit5

//////////////////////////////////////////////////////////////////////////////
//                               Encoders                                   //
//////////////////////////////////////////////////////////////////////////////
/* Abs Encoder: FPGA_ENC_ABS_BASEADDR                     0x300000 + 0x00A0 */
#define FPGA_ENCODER_RESOLVERDELAY  FPGA_REGISTER_16( (FPGA_ENC_ABS_BASEADDR + 0x0000) )


//////////////////////////////////////////////////////////////////////////////
//                         Endat Encoder                                    //
//////////////////////////////////////////////////////////////////////////////

#define FPGA_ENDAT_SET                  (0x0000)
#define FPGA_ENDAT_STATUS               (0x0002)
#define FPGA_ENDAT_CMDADR               (0x0004)
#define FPGA_ENDAT_PARAM                (0x0006)
#define FPGA_ENDAT_CLOCKDIV             (0x0008)
#define FPGA_ENDAT_PROPCOMP             (0x000A)
#define FPGA_ENDAT_POSFORMAT            (0x000C)
#define FPGA_ENDAT_STARTDELAY           (0x000E)
#define FPGA_ENDAT_EARLYSTOP            (0x0010)
#define FPGA_ENDAT_PROPCOMPMEAS         (0x0012)
#define FPGA_ENDAT_DATAOUT_LLSW         (0x0014)
#define FPGA_ENDAT_DATAOUT_LHSW         (0x0016)
#define FPGA_ENDAT_DATAOUT_HLSW         (0x0018)
#define FPGA_ENDAT_DATAOUT_HHSW         (0x001A)

// 0 = command mode; 1 = position mode (continuous, SYNC triggered)
#define FPGA_ENDAT_SET_MODE(base)       HBITW(FPGA_BASEOFF_16(base, FPGA_ENDAT_SET))->bit0

// only for command mode, begin transaction
#define FPGA_ENDAT_SET_START(base)      HBITW(FPGA_BASEOFF_16(base, FPGA_ENDAT_SET))->bit1

// module enable
#define FPGA_ENDAT_SET_ENABLE(base)     HBITW(FPGA_BASEOFF_16(base, FPGA_ENDAT_SET))->bit2

// configuration register write protection
#define FPGA_ENDAT_SET_WRPROT(base)     HBITW(FPGA_BASEOFF_16(base, FPGA_ENDAT_SET))->bit3

// 0 = compatibility mode; 1 = endat 2.2 commands set
#define FPGA_ENDAT_SET_EXT(base)        HBITW(FPGA_BASEOFF_16(base, FPGA_ENDAT_SET))->bit4

// enable SYNC triggered transactions for all MODE/EXT combinations
#define FPGA_ENDAT_SET_SYNCTRAN(base)   HBITW(FPGA_BASEOFF_16(base, FPGA_ENDAT_SET))->bit5

// module busy, transaction in progress
#define FPGA_ENDAT_STAT_BUSY(x)         HBITW( (x) )->bit0

// valid transaction, data out valid
#define FPGA_ENDAT_STAT_VALID(x)        HBITW( (x) )->bit1

// last transaction crc error
#define FPGA_ENDAT_STAT_CRCERR(x)       HBITW( (x) )->bit2

// received alarm bit (position mode)
#define FPGA_ENDAT_STAT_ALARM(x)        HBITW( (x) )->bit3

// position mode overtime (detected new SYNC before end of transaction)
#define FPGA_ENDAT_STAT_OVERTIME(x)     HBITW( (x) )->bit4

// last ADINFO transaction crc error
#define FPGA_ENDAT_STAT_ADICRCERR(x)    HBITW( (x) )->bit5

// enhanced endat module available
#define FPGA_ENDAT_STAT_ENDATEX(x)      HBITW( (x) )->bit15

//////////////////////////////////////////////////////////////////////////////
//                         Biss Encoder                                     //
//////////////////////////////////////////////////////////////////////////////

#define FPGA_BISS_SET                  (0x0000)
#define FPGA_BISS_STATUS               (0x0002)
#define FPGA_BISS_DATA                 (0x0004)
#define FPGA_BISS_ADDR                 (0x0006)
#define FPGA_BISS_PRETRG               (0x000A)
#define FPGA_BISS_CLKDIV               (0x0010)
#define FPGA_BISS_DATALEN              (0x0012)
#define FPGA_BISS_DATA_LL              (0x0014)
#define FPGA_BISS_DATA_LH              (0x0016)
#define FPGA_BISS_DATA_HL              (0x0018)
#define FPGA_BISS_DATA_HH              (0x001A)

// 1 = command mode; 0 = position mode (continuous, SYNC triggered)
#define FPGA_BISS_SET_MODE(base)       HBITW(FPGA_BASEOFF_16(base, FPGA_BISS_SET))->bit0

// Start, only for command mode, begin transaction
#define FPGA_BISS_SET_START(base)      HBITW(FPGA_BASEOFF_16(base, FPGA_BISS_SET))->bit1

// Read commnad
#define FPGA_BISS_SET_READ(base)       HBITW(FPGA_BASEOFF_16(base, FPGA_BISS_SET))->bit2

// Write command
#define FPGA_BISS_SET_WRITE(base)      HBITW(FPGA_BASEOFF_16(base, FPGA_BISS_SET))->bit3

// MO enable
#define FPGA_BISS_SET_STOP(base)       HBITW(FPGA_BASEOFF_16(base, FPGA_BISS_SET))->bit4

// module busy, transaction in progress
#define FPGA_BISS_STAT_BUSY(x)         HBITW( (x) )->bit0

// valid transaction, data out valid
#define FPGA_BISS_STAT_VALID(x)        HBITW( (x) )->bit1

// last transaction error
#define FPGA_BISS_STAT_ERROR(x)        HBITW( (x) )->bit2

// position mode overtime (detected new SYNC before end of transaction)
#define FPGA_BISS_STAT_OVERTIME(x)     HBITW( (x) )->bit3

// error
#define FPGA_BISS_STAT_ALARM(x)        HBITW( (x) )->bit4

//////////////////////////////////////////////////////////////////////////////
//                          Digital Encoder                                 //
//////////////////////////////////////////////////////////////////////////////

/* present position */
#define FPGA_INCENC_CNT_LSW             (0x0000)
#define FPGA_INCENC_CNT_MSW             (0x0002)
/* last index found: if 0x8000_0000, index never found */
#define FPGA_INCENC_CNTID_LSW           (0x0004)
#define FPGA_INCENC_CNTID_MSW           (0x0006)
/* sin cos values */
#define FPGA_INCENC_SIN                 (0x0008)
#define FPGA_INCENC_COS                 (0x000A)
#define FPGA_INCENC_M_PERIOD            (0x000C)
#define FPGA_INCENC_M_FRACT             (0x000E)
/* filter of the analog input: 0x0000-0x7fff */
#define FPGA_INCENC_K_FILTER            (0x0014)

#define FPGA_INCENC_SET                 (0x0016)
#define FPGA_INCENC_IGNORE_SINCOS(base) HBITW(FPGA_BASEOFF_16(base, FPGA_INCENC_SET ))->bit0 /* 1 = ignore sincos signals; 0 = use sincos signals */
#define FPGA_INCENC_ENABLE_ADFLT(base)  HBITW(FPGA_BASEOFF_16(base, FPGA_INCENC_SET ))->bit1
#define FPGA_INCENC_RESET(base)         HBITW(FPGA_BASEOFF_16(base, FPGA_INCENC_SET ))->bit2 /* 1 = reset (startup); 0 = normal conditions  */
#define FPGA_INCENC_SWAPINPUTS(base)    HBITW(FPGA_BASEOFF_16(base, FPGA_INCENC_SET ))->bit3
#define FPGA_INCENC_EN_STEPDIR(base)    HBITW(FPGA_BASEOFF_16(base, FPGA_INCENC_SET ))->bit4
#define FPGA_INCENC_EN_UPDOWN(base)     HBITW(FPGA_BASEOFF_16(base, FPGA_INCENC_SET ))->bit5
#define FPGA_INCENC_DIS_IDXABSYNC(base) HBITW(FPGA_BASEOFF_16(base, FPGA_INCENC_SET ))->bit6
#define FPGA_INCENC_DIS_WDTFAULT(base)  HBITW(FPGA_BASEOFF_16(base, FPGA_INCENC_SET ))->bit7
#define FPGA_INCENC_DIS_IDXDYNFILTER(base)  HBITW(FPGA_BASEOFF_16(base, FPGA_INCENC_SET ))->bit8

#define FPGA_INCENC_STATUS              (0x0018)
#define FPGA_INCENC_S_FAULT(x)          HBITW( (x) )->bit0 /* 1 = fault; 0 = no fault */
#define FPGA_INCENC_S_FLT_DIG(x)        HBITW( (x) )->bit1
#define FPGA_INCENC_S_FLT_ANDIG(x)      HBITW( (x) )->bit2
#define FPGA_INCENC_S_FLT_WDT(x)        HBITW( (x) )->bit3
#define FPGA_INCENC_S_IDX_EVCAP(x)      HBITW( (x) )->bit14
#define FPGA_INCENC_S_M_DIRECTION(x)    HBITW( (x) )->bit15

#define FPGA_INCENC_DIG_FILTER          (0x001A)

#define FPGA_INCENC_CNTS_LSW            (0x001C)
#define FPGA_INCENC_CNTS_MSW            (0x001E)

#define FPGA_INCENC_COUNT32             (0x0000)
#define FPGA_INCENC_INDEX32             (0x0004)
#define FPGA_INCENC_CNTSNAPSHOT32       (0x001C)

//////////////////////////////////////////////////////////////////////////////
//                             Inc Out Encoder                              //
//////////////////////////////////////////////////////////////////////////////
/* Encoder Out: FPGA_INCOUT_BASEADDR                      0x300000 + 0x01C0 */
/* repeatition digital incremental encoder: when enabled by the means of ENA *
 * and ENB, its output is linked to the input of the auxuliary encoder       */

/* present position generated */
#define FPGA_IOUTAUX_CNT_LSW        FPGA_REGISTER_16( (FPGA_INCOUT_BASEADDR + 0x0000) )
#define FPGA_IOUTAUX_CNT_MSW        FPGA_REGISTER_16( (FPGA_INCOUT_BASEADDR + 0x0002) )
/* position generated by the index */
#define FPGA_IOUTAUX_POSID_LSW      FPGA_REGISTER_16( (FPGA_INCOUT_BASEADDR + 0x0004) )
#define FPGA_IOUTAUX_POSID_MSW      FPGA_REGISTER_16( (FPGA_INCOUT_BASEADDR + 0x0006) )
#define FPGA_IOUTAUX_STEPS          FPGA_REGISTER_16( (FPGA_INCOUT_BASEADDR + 0x0008) )  /* bit   15: 1: generation enabled, 0: generation disabled *
                                                                                          * bit   14: 1: negative counts, 0: positive counts        *
                                                                                          * bit 13-0: counts period n pulses of 12.5ns each         *
                                                                                          *           example: 80 = 1 encoder pulse every 1us       */
#ifdef _INFINEON_
#define FPGA_IOUTAUX_COUNT32        FPGA_REGISTER_32( (FPGA_INCOUT_BASEADDR + 0x0000) )
#define FPGA_IOUTAUX_INDEX32        FPGA_REGISTER_32( (FPGA_INCOUT_BASEADDR + 0x0004) )
#else
#define FPGA_IOUTAUX_COUNT32        MAKELONG(FPGA_IOUTAUX_CNT_LSW,FPGA_IOUTAUX_CNT_MSW)
#define FPGA_IOUTAUX_INDEX32        MAKELONG(FPGA_IOUTAUX_POSID_LSW,FPGA_IOUTAUX_POSID_MSW)
#endif
#define FPGA_IOUTAUX_FASTSIM_EN     HBITW( FPGA_EXTOPTIONS )->bit2                       /* fast simulation */

//////////////////////////////////////////////////////////////////////////////
//                       Hiperface Encoder                                  //
//////////////////////////////////////////////////////////////////////////////

#define FPGA_HIPFC_SET                  (0x0000)    // WR-Only register
#define FPGA_HIPFC_STATUS               (0x0002)
#define FPGA_HIPFC_DATA                 (0x0004)

#define FPGA_HIPFC_SET_PARITY_EN(x)     HBITW( (x) )->bit12
#define FPGA_HIPFC_SET_PARITY_ODD(x)    HBITW( (x) )->bit13
#define FPGA_HIPFC_SET_EXECUTE(x)       HBITW( (x) )->bit15

#define FPGA_HIPFC_STAT_PARITYERROR(x)  HBITW( FPGA_BASEOFF_16(x, FPGA_HIPFC_STATUS ) )->bit15

//////////////////////////////////////////////////////////////////////////////
//                       Nikon Encoder                                      //
//////////////////////////////////////////////////////////////////////////////

#define FPGA_NIKON_SET                  0x0000
#define FPGA_NIKON_STATUS               0x0002
#define FPGA_NIKON_DATA                 0x0004
#define FPGA_NIKON_PRETRG               0x0006
#define FPGA_NIKON_POS_LL               0x0008
#define FPGA_NIKON_POS_LH               0x000A
#define FPGA_NIKON_POS_HL               0x000C
#define FPGA_NIKON_POS_HH               0x000E

#define FPGA_NIKON_SET_WR(base)         HBITW( (FPGA_REGISTER_16( (base + FPGA_NIKON_SET) )) )->bit6
#define FPGA_NIKON_SET_RD(base)         HBITW( (FPGA_REGISTER_16( (base + FPGA_NIKON_SET) )) )->bit7
#define FPGA_NIKON_SET_CMDMODE(base)    HBITW( (FPGA_REGISTER_16( (base + FPGA_NIKON_SET) )) )->bit8

#define FPGA_NIKON_STAT_EOF(x)          HBITW( x )->bit0
#define FPGA_NIKON_STAT_BUSY(x)         HBITW( x )->bit1
#define FPGA_NIKON_STAT_TMOUT(x)        HBITW( x )->bit2
#define FPGA_NIKON_STAT_IOSEL(x)        HBITW( x )->bit3
#define FPGA_NIKON_STAT_TRGFAULT(x)     HBITW( x )->bit4
#define FPGA_NIKON_STAT_ALARM(x)        HBITW( x )->bit5

//////////////////////////////////////////////////////////////////////////////
//                       TAMAGAWA Encoder                                   //
//////////////////////////////////////////////////////////////////////////////

#define FPGA_TMGW_SET                   0x0000
#define FPGA_TMGW_STATUS                0x0002
#define FPGA_TMGW_DATA_0                0x0004
#define FPGA_TMGW_DATA_1                0x0006
#define FPGA_TMGW_DATA_2                0x0008
#define FPGA_TMGW_PRETRG                0x000A

#define FPGA_TMGW_SET_REQ(base)         HBITW( (FPGA_REGISTER_16( (base + FPGA_TMGW_SET) )) )->bit0
#define FPGA_TMGW_SET_RQC0(base)        HBITW( (FPGA_REGISTER_16( (base + FPGA_TMGW_SET) )) )->bit1
#define FPGA_TMGW_SET_RQC1(base)        HBITW( (FPGA_REGISTER_16( (base + FPGA_TMGW_SET) )) )->bit2
#define FPGA_TMGW_SET_DS40(base)        HBITW( (FPGA_REGISTER_16( (base + FPGA_TMGW_SET) )) )->bit3
#define FPGA_TMGW_SET_ESEL(base)        HBITW( (FPGA_REGISTER_16( (base + FPGA_TMGW_SET) )) )->bit4
#define FPGA_TMGW_SET_SELRT(base)       HBITW( (FPGA_REGISTER_16( (base + FPGA_TMGW_SET) )) )->bit5
#define FPGA_TMGW_SET_CMD(base)         HBITW( (FPGA_REGISTER_16( (base + FPGA_TMGW_SET) )) )->bit6

#define FPGA_TMGW_STAT_BUSY(x)          HBITW( x )->bit0
#define FPGA_TMGW_STAT_TIMEOUT(x)       HBITW( x )->bit1
#define FPGA_TMGW_STAT_EOF(x)           HBITW( x )->bit2
#define FPGA_TMGW_STAT_TRGFAULT(x)      HBITW( x )->bit3
#define FPGA_TMGW_STAT_ALARM(x)         HBITW( x )->bit4

//////////////////////////////////////////////////////////////////////////////
//                  Current Channels Min/Max Measurements                   //
//////////////////////////////////////////////////////////////////////////////
/* FPGA_CURCHMINMAX_BASEADDR    0x300000 + 0x0240 */

#define    FPGA_CURCHMINMAX_IU_MIN     FPGA_REGISTER_16( (FPGA_CURCHMINMAX_BASEADDR + 0x0000) )
#define    FPGA_CURCHMINMAX_IU_MAX     FPGA_REGISTER_16( (FPGA_CURCHMINMAX_BASEADDR + 0x0002) )
#define    FPGA_CURCHMINMAX_IV_MIN     FPGA_REGISTER_16( (FPGA_CURCHMINMAX_BASEADDR + 0x0004) )
#define    FPGA_CURCHMINMAX_IV_MAX     FPGA_REGISTER_16( (FPGA_CURCHMINMAX_BASEADDR + 0x0006) )
#define    FPGA_CURCHMINMAX_IW_MIN     FPGA_REGISTER_16( (FPGA_CURCHMINMAX_BASEADDR + 0x0008) )
#define    FPGA_CURCHMINMAX_IW_MAX     FPGA_REGISTER_16( (FPGA_CURCHMINMAX_BASEADDR + 0x000A) )

#define    FPGA_CURCHMINMAX_ADDR_IUIV  FPGA_REGISTER_16( (FPGA_CURCHMINMAX_BASEADDR + 0x000C) )
#define    FPGA_CURCHMINMAX_ADDR_IW    FPGA_REGISTER_16( (FPGA_CURCHMINMAX_BASEADDR + 0x000E) )

//////////////////////////////////////////////////////////////////////////////
//                           FPGA Ethernet Registers                        //
//////////////////////////////////////////////////////////////////////////////

#define FPGA_ECATREGS_SET           FPGA_REGISTER_16( (FPGA_ECATREGS_BASEADDR + 0x0000) )

#define FPGA_ECATREGS_SET_ENMAC     HBITW( FPGA_ECATREGS_SET )->bit0
#define FPGA_ECATREGS_SET_ENPHY     HBITW( FPGA_ECATREGS_SET )->bit1
#define FPGA_ECATREGS_SET_LATCHSYN0 HBITW( FPGA_ECATREGS_SET )->bit2
#define FPGA_ECATREGS_STAT_IRQLATCH HBITW( FPGA_ECATREGS_SET )->bit15

#ifdef _INFINEON
#define    RGAD_ECAT_FREERUNTMR        FPGA_REGISTER_32( (FPGA_ECATREGS_BASEADDR + 0x0004) )
#define    RGAD_ECAT_SPIRQ             FPGA_REGISTER_32( (FPGA_ECATREGS_BASEADDR + 0x0008) )
#define    RGAD_ECAT_SPSYNC0           FPGA_REGISTER_32( (FPGA_ECATREGS_BASEADDR + 0x000C) )
#else
#define    RGAD_ECAT_FREERUNTMR        MAKELONG(FPGA_REGISTER_16(FPGA_ECATREGS_BASEADDR + 0x0004),FPGA_REGISTER_16(FPGA_ECATREGS_BASEADDR + 0x0006))
#define    RGAD_ECAT_SPIRQ             MAKELONG(FPGA_REGISTER_16(FPGA_ECATREGS_BASEADDR + 0x0008),FPGA_REGISTER_16(FPGA_ECATREGS_BASEADDR + 0x000A))
#define    RGAD_ECAT_SPSYNC0           MAKELONG(FPGA_REGISTER_16(FPGA_ECATREGS_BASEADDR + 0x000C),FPGA_REGISTER_16(FPGA_ECATREGS_BASEADDR + 0x000E))
#endif

#define FPGA_ETHERNET_BASE_ADDRESS  XPAR_AXI_ETHERCAT_0_BASEADDR
#define FPGA_ETHERNET_SIZE          0x2000

//////////////////////////////////////////////////////////////////////////////
//                  CPUH                                                    //
//////////////////////////////////////////////////////////////////////////////
/* FPGA_CPUH_BASEADDR    0x300000 + 0x0400 */

#define    FPGA_CPUH_INSRAM_CODE    FPGA_REGISTER_16( (FPGA_CPUH_BASEADDR + 0x0000) )
#define    FPGA_CPUH_INSRAM_LEN     FPGA_REGISTER_16( (FPGA_CPUH_BASEADDR + 0x0002) )
#define    FPGA_DSPH_INSRAML_OVERTIME   FPGA_REG16_GETBIT( FPGA_CPUH_INSRAM_LEN, 15 )
#define    FPGA_DSPH_INSRAML_HALTACK    FPGA_REG16_GETBIT( FPGA_CPUH_INSRAM_LEN, 14 )
#define    FPGA_DSPH_INSRAML_TIMEMASK   (0x00FF)

#define    FPGA_CPUH_INSRAM_STROBE  FPGA_REGISTER_16( (FPGA_CPUH_BASEADDR + 0x0004) )
#define    FPGA_CPUH_INSRAM_CMPLEV  FPGA_REGISTER_16( (FPGA_CPUH_BASEADDR + 0x0006) )

#define    FPGA_CPUH_DSPMAXCYLE     (160)

// DSPH Registers
#define    FPGA_DSPH_HALT           FPGA_REGISTER_16( (FPGA_CPUH_BASEADDR + 0x0010) )
#define    FPGA_DSPH_SET_HALT        	FPGA_DSPH_HALT

// RG UPDATE
#define    FPGA_CPUH_RGUPD_MODE     FPGA_REGISTER_16( (FPGA_CPUH_BASEADDR + 0x0020) )
#define    FPGA_CPUH_RGUPD_SET_QUEUE    FPGA_CPUH_RGUPD_MODE

// access to DATRAM
#define    FPGA_CPUH_DRAM_BASE      (FPGA_REGISTER_BASE_ADDRESS+RGAD_CPUH_DATRAM_BASEADDR)
#define    FPGA_CPUH_DRAM_WR_SW(a,d)    { ((HPUWORD)((a)+FPGA_CPUH_DRAM_BASE))[0]=0;\
                                          ((HPSWORD)((a)+FPGA_CPUH_DRAM_BASE))[1]=(d);\
                                          ((HPSWORD)((a)+FPGA_CPUH_DRAM_BASE))[2]=(SWORD)(d)>=0?0:-1;}
#define    FPGA_CPUH_DRAM_WR_SL(a,d)    { ((HPUWORD)((a)+FPGA_CPUH_DRAM_BASE))[0]=0;\
                                          ((HPSLONG)&(((HPUWORD)((a)+FPGA_CPUH_DRAM_BASE))[1]))[0]=(d);}

// access to RGURAM
#define    FPGA_CPUH_URAM_BASE      (FPGA_REGISTER_BASE_ADDRESS+RGAD_CPUH_RGURAM_BASEADDR)
#define    FPGA_CPUH_URAM_WR_SW(a,d)    { ((HPSWORD)(((a)<<3)+FPGA_CPUH_URAM_BASE+RGAD_CPUH_RGURAM_SIZE/2))[1]=(d);}
#define    FPGA_CPUH_URAM_WR_SL(a,d)    { ((HPUWORD)(((a)<<3)+FPGA_CPUH_URAM_BASE))[0]=0;\
                                          ((HPSLONG)(((a)<<3)+FPGA_CPUH_URAM_BASE+sizeof(UWORD)))[0]=(d);}
#define    FPGA_CPUH_URAM_WR_SQ(a,d)    { ((HPULONG)(((a)<<3)+FPGA_CPUH_URAM_BASE))[0]=(d).lo;\
                                          ((HPSWORD)(((a)<<3)+FPGA_CPUH_URAM_BASE))[2]=(SWORD)((d).hi);}


//////////////////////////////////////////////////////////////////////////////
//                  ADC TAGS                                                //
//////////////////////////////////////////////////////////////////////////////

#ifndef _HW_DC
#define FPGA_ADTAGS_PWI_ISENSEU         0x00
#define FPGA_ADTAGS_PWI_ISENSEV         0x01
#define FPGA_ADTAGS_PWI_DCBUS           0x02
#define FPGA_ADTAGS_PWB_PSPLU           0x03
#define FPGA_ADTAGS_PWB_PSPLV           0x04
#define FPGA_ADTAGS_PWB_DCBUS           0x05
#define FPGA_ADTAGS_ABSENC_SIN          0x06
#define FPGA_ADTAGS_ABSENC_COS          0x07
#define FPGA_ADTAGS_INCENC_SIN          0x08
#define FPGA_ADTAGS_INCENC_COS          0x09
#define FPGA_ADTAGS_USRIO_AIN0          0x0A
#define FPGA_ADTAGS_USRIO_AIN1          0x0B
#define FPGA_ADTAGS_USRIO_AIN2          0x0C
#define FPGA_ADTAGS_USRIO_AIN3          0x0D
#define FPGA_ADTAGS_VIRTUAL_IW          0x0E
#define FPGA_ADTAGS_VIRTUAL_IQR_FLT     0x0F
#define FPGA_ADTAGS_LUT_SIN             0x10
#define FPGA_ADTAGS_LUT_COS             0x11

#else
#define FPGA_ADTAGS_PWI_BR0_ISENSEU     0x00
#define FPGA_ADTAGS_PWI_BR0_ISENSEV     0x01
#define FPGA_ADTAGS_PWI_BR0_ISENSEW     0x02
#define FPGA_ADTAGS_PWI_BR1_ISENSEU     0x03
#define FPGA_ADTAGS_PWI_BR1_ISENSEV     0x04
#define FPGA_ADTAGS_PWI_BR1_ISENSEW     0x05
#define FPGA_ADTAGS_PWI_BR2_ISENSEU     0x06
#define FPGA_ADTAGS_PWI_BR2_ISENSEV     0x07
#define FPGA_ADTAGS_PWI_BR2_ISENSEW     0x08
// #define FPGA_ADTAGS_PWB_PSPLU           0x09
// #define FPGA_ADTAGS_PWB_PSPLV           0x0A
#define FPGA_ADTAGS_PWB_DCBUS           0x09
#define FPGA_ADTAGS_ABSENC_SIN          0x0A
#define FPGA_ADTAGS_ABSENC_COS          0x0B
#define FPGA_ADTAGS_INCENC_SIN          0x0C
#define FPGA_ADTAGS_INCENC_COS          0x0D
#define FPGA_ADTAGS_USRIO_AIN0          0x0E
#define FPGA_ADTAGS_USRIO_AIN1          0x0F
#define FPGA_ADTAGS_USRIO_AIN2          0x10
#define FPGA_ADTAGS_USRIO_AIN3          0x11
#define FPGA_ADTAGS_VIRTUAL_IQR_FLT     0x12
#define FPGA_ADTAGS_LUT_SIN             0x13
#define FPGA_ADTAGS_LUT_COS             0x14
#endif

//////////////////////////////////////////////////////////////////////////////
//                          EtherPMC                                        //
//////////////////////////////////////////////////////////////////////////////


/* EtherPMC: FPGA_EPMCREGS_BASEADDR        0x03A0 */
#define FPGA_EPMCREGS_SET               FPGA_REGISTER_16( (FPGA_EPMCREGS_BASEADDR + 0x0000) )
#define FPGA_EPMCREGS_SET_MATEREN       (0)
#define FPGA_EPMCREGS_SET_AUTOTRIGGEREN (1)
#define FPGA_EPMCREGS_SET_MANUALTRIGGER (2)
#define FPGA_EPMCREGS_SET_ENDDEVEN      (3)
#define FPGA_EPMCREGS_SET_MBOXREWR      (4)
#define FPGA_EPMCREGS_SET_HEADERSBUF    (5)
#define FPGA_EPMCREGS_SET_PHY_EN        (6)

#define FPGA_EPMCREGS_OFFSET            FPGA_REGISTER_16( (FPGA_EPMCREGS_BASEADDR + 0x0002) )
#define FPGA_EPMCREGS_SIZE              FPGA_REGISTER_16( (FPGA_EPMCREGS_BASEADDR + 0x0004) )

#define FPGA_EPMCREGS_STATUS            FPGA_REGISTER_16( (FPGA_EPMCREGS_BASEADDR + 0x000A) )
#define FPGA_EPMCREGS_STATUS_EOP        (0)
#define FPGA_EPMCREGS_STATUS_ERROR      (1)
#define FPGA_EPMCREGS_STATUS_ALLINK     (2)
#define FPGA_EPMCREGS_STATUS_BLLINK     (3)

#define FPGA_EPMCREGS_MIIM_ST           FPGA_REGISTER_16( (FPGA_EPMCREGS_BASEADDR + 0x0010) )
#define FPGA_EPMCREGS_MIIM_ST_REGADDR   (0)
#define FPGA_EPMCREGS_MIIM_ST_PHYADDR   (5)
#define FPGA_EPMCREGS_MIIM_ST_OP        (10)
#define FPGA_EPMCREGS_MIIM_ST_ERROR     (14)
#define FPGA_EPMCREGS_MIIM_ST_BUSY      (15)

#define FPGA_EPMCREGS_MIIM_DT           FPGA_REGISTER_16( (FPGA_EPMCREGS_BASEADDR + 0x0012) )

#define FPGA_EPMCREGS_FREERUNTMR_LO     FPGA_REGISTER_16( (FPGA_EPMCREGS_BASEADDR + 0x0014) )
#define FPGA_EPMCREGS_FREERUNTMR_HI     FPGA_REGISTER_16( (FPGA_EPMCREGS_BASEADDR + 0x0016) )
#define FPGA_EPMCREGS_SPSOP_LO          FPGA_REGISTER_16( (FPGA_EPMCREGS_BASEADDR + 0x0018) )
#define FPGA_EPMCREGS_SPSOP_HI          FPGA_REGISTER_16( (FPGA_EPMCREGS_BASEADDR + 0x001A) )

#define FPGA_EPMCREGS_SLOT_BASE         (FPGA_REGISTER_BASE_ADDRESS+FPGA_EPMCREGS_BUF_BASEADDR+0x0028)

#endif

//////////////////////////////////////////////////////////////////////////////
//                          Ethernet/IP                                     //
//////////////////////////////////////////////////////////////////////////////

/* Ethernet: FPGA_ENETREGS_BASEADDR        0x03C0 */
#define FPGA_ENETREGS_SET                   FPGA_REGISTER_16(FPGA_ENETREGS_BASEADDR + 0x0000)
#define FPGA_ENETREGS_SET_RESET             HBITW( FPGA_ENETREGS_SET )->bit0
#define FPGA_ENETREGS_SET_PROMISEN          HBITW( FPGA_ENETREGS_SET )->bit2
#define FPGA_ENETREGS_SET_PORTX12_EN        HBITW( FPGA_ENETREGS_SET )->bit3
#ifdef _HW_DC
#define FPGA_ENETREGS_SET_PORTX34_EN        HBITW( FPGA_ENETREGS_SET )->bit4
#endif

#define FPGA_ENETREGS_STATUS                FPGA_REGISTER_16(FPGA_ENETREGS_BASEADDR + 0x0002)
#define FPGA_ENETREGS_STATUS_RXCACHED(x)    HBITW( x )->bit0
#define FPGA_ENETREGS_STATUS_LINK(x)        HBITW( x )->bit1

#define FPGA_ENETREGS_CTRL                  FPGA_REGISTER_16(FPGA_ENETREGS_BASEADDR + 0x0004)
#define FPGA_ENETREGS_CTRL_RXFLUSH          HBITW( FPGA_ENETREGS_CTRL )->bit0
#define FPGA_ENETREGS_CTRL_TXTRANSMIT       HBITW( FPGA_ENETREGS_CTRL )->bit1
#define FPGA_ENETREGS_CTRL_RDBUF            HBITW( FPGA_ENETREGS_CTRL )->bit2
#define FPGA_ENETREGS_CTRL_WRBUF            HBITW( FPGA_ENETREGS_CTRL )->bit3

#define FPGA_ENETREGS_RXLENGTH              FPGA_REGISTER_16(FPGA_ENETREGS_BASEADDR + 0x0006)
#define FPGA_ENETREGS_TXLENGTH              FPGA_REGISTER_16(FPGA_ENETREGS_BASEADDR + 0x0008)
#define FPGA_ENETREGS_BUF_DATA              FPGA_REGISTER_16(FPGA_ENETREGS_BASEADDR + 0x0012)
#define FPGA_ENETREGS_BUF_OFFSET            FPGA_REGISTER_16(FPGA_ENETREGS_BASEADDR + 0x0014)

//////////////////////////////////////////////////////////////////////////////
//                            Voltage Estimator                             //
//////////////////////////////////////////////////////////////////////////////
/* Voltage Estimator: FPGA_VEST_BASEADDR                    0x1700 + 0x0000 */

#define FPGA_VEST_U                         FPGA_REGISTER_16( (FPGA_VEST_BASEADDR + 0x0000) )
#define FPGA_VEST_V                         FPGA_REGISTER_16( (FPGA_VEST_BASEADDR + 0x0002) )
#define FPGA_VEST_W                         FPGA_REGISTER_16( (FPGA_VEST_BASEADDR + 0x0004) )


