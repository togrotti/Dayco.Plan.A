/****************************************************************************/
/* AUTO-GENERATED FILE - DO NOT MODIFY IT!                                  */
/****************************************************************************/

/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : SysAppModBusParamTable.c                                   */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : System application parameter table for ModBus              */
/*                                                                          */
/****************************************************************************/

#include "bus\modbus\ModBusComDB.h"
#include "system\SysAppConfig.h"

//****************************************************************************
// Table def
// #%n psCommonParamTable

const MODBUSCOMDB_ENTRY hpsModBusParamTable[]=
{
    {  100 /*0064*/, &psCommonParamTable[582] },
    {  101 /*0065*/, &psCommonParamTable[583] },
    {  102 /*0066*/, &psCommonParamTable[584] },
    {  103 /*0067*/, &psCommonParamTable[585] },
    {  104 /*0068*/, &psCommonParamTable[586] },
    {  105 /*0069*/, &psCommonParamTable[587] },
    {  106 /*006A*/, &psCommonParamTable[588] },
    {  107 /*006B*/, &psCommonParamTable[589] },

#ifdef CFG_EN_MODBUSOVERCAN
//     {  108 /*006C*/, #%p Requested index 0x7CA0 not found in the common database },
//     {  109 /*006D*/, #%p Requested index 0x7CA1 not found in the common database },
//     {  110 /*006E*/, #%p Requested index 0x7CA2 not found in the common database },
//     {  111 /*006F*/, #%p Requested index 0x7CA3 not found in the common database },
//     {  112 /*0070*/, #%p Requested index 0x7CA4 not found in the common database },
//     {  113 /*0071*/, #%p Requested index 0x7CA5 not found in the common database },

//     {  120         , #%p Requested index 0x7CA6 not found in the common database },
#endif

    {  200         , &psCommonParamTable[599] },
    {  201         , &psCommonParamTable[600] },
    {  202         , &psCommonParamTable[601] },
    {  203         , &psCommonParamTable[602] },
    {  204         , &psCommonParamTable[603] },
    {  205         , &psCommonParamTable[604] },
    {  206         , &psCommonParamTable[605] },
    {  207         , &psCommonParamTable[606] },
    {  208         , &psCommonParamTable[607] },
    {  209         , &psCommonParamTable[608] },

    {13800 /*35E8*/, &psCommonParamTable[597] },
    {13801 /*35E9*/, &psCommonParamTable[598] },
    {14200         , &psCommonParamTable[593] },
    {14700         , &psCommonParamTable[594] },
    {15200         , &psCommonParamTable[595] },
    {15700         , &psCommonParamTable[596] },

    {18001 /*4651*/, &psCommonParamTable[610] },
    {18002 /*4652*/, &psCommonParamTable[611] },
    {18003 /*4653*/, &psCommonParamTable[612] },
    {18004 /*4654*/, &psCommonParamTable[613] },
    {18005 /*4655*/, &psCommonParamTable[614] },
    {18006 /*4656*/, &psCommonParamTable[615] },
    {18007 /*4657*/, &psCommonParamTable[616] },
    {18008 /*4658*/, &psCommonParamTable[617] },
    {18009 /*4659*/, &psCommonParamTable[618] },

    {18010 /*465A*/, &psCommonParamTable[30] },
    {18011 /*465B*/, &psCommonParamTable[31] },
    {18013 /*465D*/, &psCommonParamTable[33] },
    {18014 /*465E*/, &psCommonParamTable[34] },
    {18015 /*465F*/, &psCommonParamTable[35] },
    {18016 /*4660*/, &psCommonParamTable[38] },
#ifdef _APP_XC
    {18017 /*4661*/, &psCommonParamTable[36] },
#endif

    {18019 /*4663*/, &psCommonParamTable[619] },

#ifndef _HW_DC
//     {18020 /*4664*/, #%p Requested index 0x01A0 not found in the common database },
//     {18021 /*4665*/, #%p Requested index 0x01A1 not found in the common database },
//     {18028 /*466C*/, #%p Requested index 0x01A8 not found in the common database },
//     {18029 /*466D*/, #%p Requested index 0x01A9 not found in the common database },
//     {18031 /*466F*/, #%p Requested index 0x01AB not found in the common database },
//     {18032 /*4670*/, #%p Requested index 0x01AC not found in the common database },
//     {18033 /*4671*/, #%p Requested index 0x01AD not found in the common database },
//     {18034 /*4672*/, #%p Requested index 0x01AE not found in the common database },
//     {18035 /*4673*/, #%p Requested index 0x01AF not found in the common database },
//     {18036 /*4674*/, #%p Requested index 0x01B0 not found in the common database },
//     {18037 /*4675*/, #%p Requested index 0x01B1 not found in the common database },
#endif
//     {18038 /*4675*/, #%p Requested index 0x01C0 not found in the common database },
//     {18042 /*4676*/, #%p Requested index 0x01C4 not found in the common database },
//     {18046 /*4680*/, #%p Requested index 0x01C8 not found in the common database },
//     {18047 /*4681*/, #%p Requested index 0x01C9 not found in the common database },
//     {18048 /*4682*/, #%p Requested index 0x01CA not found in the common database },

    {18050 /*4682*/, &psCommonParamTable[590] },
    {18058 /*468A*/, &psCommonParamTable[591] },
    {18059 /*468B*/, &psCommonParamTable[592] },

    {18060 /*468C*/, &psCommonParamTable[50] },
    {18061 /*468D*/, &psCommonParamTable[51] },
    {18062 /*468E*/, &psCommonParamTable[54] },
    {18063 /*468F*/, &psCommonParamTable[55] },

    {18068 /*4694*/, &psCommonParamTable[27] },

#ifndef _HW_DC
//     {18070 /*4696*/, #%p Requested index 0x01AA not found in the common database },
#endif

    {18076 /*469C*/, &psCommonParamTable[11] },
    {18077 /*469D*/, &psCommonParamTable[12] },

    {18080 /*46A0*/, &psCommonParamTable[10] },
    {18081 /*46A1*/, &psCommonParamTable[1] },
    {18082 /*46A2*/, &psCommonParamTable[2] },
                                                        
    {18083 /*46A3*/, &psCommonParamTable[4] },
    {18084 /*46A4*/, &psCommonParamTable[1] },
    {18085 /*46A5*/, &psCommonParamTable[2] },
    {18086 /*46A6*/, &psCommonParamTable[5] },

    {18088 /*46A8*/, &psCommonParamTable[53] },
    {18089 /*46A9*/, &psCommonParamTable[49] },
    {18090 /*46AA*/, &psCommonParamTable[52] },

    {18094 /*46AE*/, &psCommonParamTable[6] },
    {18095 /*46AF*/, &psCommonParamTable[7] },
#ifndef _HW_DC
//     {18096 /*46B0*/, #%p Requested index 0x010A not found in the common database },
#endif
    {18097 /*46B1*/, &psCommonParamTable[8] },
    {18098 /*46B2*/, &psCommonParamTable[40] },

    {19000 /*4A38*/, &psCommonParamTable[42] },
    {19001 /*4A39*/, &psCommonParamTable[43] },
    {19002 /*4A3A*/, &psCommonParamTable[44] },
    {19003 /*4A3B*/, &psCommonParamTable[45] },
    {19004 /*4A3C*/, &psCommonParamTable[46] },
    {19005 /*4A3D*/, &psCommonParamTable[41] },
    {19006 /*4A3E*/, &psCommonParamTable[48] },

    {19010 /*4A42*/, &psCommonParamTable[3] },
    {19020 /*4A4C*/, &psCommonParamTable[28] },
    {19030 /*4A56*/, &psCommonParamTable[29] },

    {19050,          &psCommonParamTable[64] },
    {19051,          &psCommonParamTable[65] },
    {19052,          &psCommonParamTable[66] },

    {19100,          &psCommonParamTable[56] },

    {19200,          &psCommonParamTable[57] },
    {19201,          &psCommonParamTable[58] },
    {19202,          &psCommonParamTable[59] },
    {19203,          &psCommonParamTable[60] },
    {19204,          &psCommonParamTable[61] },
    {19205,          &psCommonParamTable[62] },
    {19207,          &psCommonParamTable[63] },

    {19300,          &psCommonParamTable[39] },

    {19340,          &psCommonParamTable[25] },
    {19341,          &psCommonParamTable[26] },
//     {19342,          #%p Requested index 0x0152 not found in the common database },

    {19350,          &psCommonParamTable[13] },
    {19351,          &psCommonParamTable[14] },
    {19352,          &psCommonParamTable[15] },
    {19353,          &psCommonParamTable[16] },
    {19354,          &psCommonParamTable[17] },
    {19355,          &psCommonParamTable[18] },
    {19356,          &psCommonParamTable[19] }, // array di 8 x 16bit

    {19370,          &psCommonParamTable[20] },
    {19371,          &psCommonParamTable[21] },
    {19372,          &psCommonParamTable[22] },
    {19373,          &psCommonParamTable[23] },
    {19374,          &psCommonParamTable[24] },

    {19400,          &psCommonParamTable[620] },
    {19401,          &psCommonParamTable[621] },
    {19402,          &psCommonParamTable[622] },
    {19403,          &psCommonParamTable[623] },
    {19404,          &psCommonParamTable[624] },
    {19410,          &psCommonParamTable[625] },
    {19426,          &psCommonParamTable[626] },

    {19451,          &psCommonParamTable[627] },
    {19452,          &psCommonParamTable[628] },
    {19453,          &psCommonParamTable[629] },
    {19454,          &psCommonParamTable[630] },

    {19460,          &psCommonParamTable[631] },
    {19461,          &psCommonParamTable[632] },
    {19462,          &psCommonParamTable[633] },
    {19463,          &psCommonParamTable[634] },
    {19464,          &psCommonParamTable[635] },
    {19465,          &psCommonParamTable[636] },
    {19466,          &psCommonParamTable[637] },
    {19467,          &psCommonParamTable[638] },

    {19480,          &psCommonParamTable[639] },
    {19481,          &psCommonParamTable[640] },
    {19482,          &psCommonParamTable[641] },

    {20010 /*4E2A*/, &psCommonParamTable[67] },
    {20011 /*4E2B*/, &psCommonParamTable[68] },

    {20049 /*4E51*/, &psCommonParamTable[69] },
#ifdef _HW_DC
    {20050 /*4E52*/, &psCommonParamTable[70] },
    {20052 /*4E54*/, &psCommonParamTable[71] },
    {20054 /*4E56*/, &psCommonParamTable[72] },
    {20056 /*4E58*/, &psCommonParamTable[73] },
#endif

    {20058 /*4E5A*/, &psCommonParamTable[74] },
    {20059 /*4E5A*/, &psCommonParamTable[75] },

    {20900 /*51A4*/, &psCommonParamTable[76] },
    {20910 /*51AE*/, &psCommonParamTable[77] },
    {20920 /*51B8*/, &psCommonParamTable[78] },

    {21999         , &psCommonParamTable[37] },
                                  
        //****************************************************************************
        // Drive Task Configuration
        //****************************************************************************
    {22000 /*55F0*/, &psCommonParamTable[581]}, /* Tipo di drive */
        // do not use 22001-22009



        //****************************************************************************
        // Encoders
        //****************************************************************************
    {26000 /*6590*/, &psCommonParamTable[79] }, /* Encoder supply voltage (RW) */ 
    {26001 /*6591*/, &psCommonParamTable[80] }, /* Main Encoder choosen (RW) */
    {26002 /*6592*/, &psCommonParamTable[81] }, /* Auxiliary Encoder choosen (RW) */
    {26003 /*6593*/, &psCommonParamTable[82] }, /* boolean to select which position-information will be used by the control loop (primary/auxiliary encoder) (RW) */
    {26004 /*6594*/, &psCommonParamTable[83] }, /* boolean to select which speed-information will be used by the control loop (primary/auxiliary encoder) (RW) */    
    {26005 /*6595*/, &psCommonParamTable[84] }, /* boolean to select which acceleration-information will be used by the control loop (primary/auxiliary encoder) (RW) */
    {26006 /*6596*/, &psCommonParamTable[85] }, /* boolean to select which electrical angle-information will be used by the FPGA (primary/auxiliary encoder) (RW) */
    {26007 /*6597*/, &psCommonParamTable[86] },
    {26008 /*6598*/, &psCommonParamTable[87] }, /* Delay after applying encoder supply [msec] */
    {26009 /*6599*/, &psCommonParamTable[88] },

    /* ====== Main ====== */
    {26010 /*65A0*/, &psCommonParamTable[95] }, /* Main Encoder mechanical turns number (RO) */
    {26011 /*65A1*/, &psCommonParamTable[96] }, /* Main Encoder mechanical angle (RO) */
    {26012 /*65A2*/, &psCommonParamTable[97] }, /* Main sensor mechanical Abs position offset HI (RO) */
    {26013 /*65A3*/, &psCommonParamTable[98] }, /* Main sensor mechanical Abs position offset LO (RO) */
    {26014 /*65A4*/, &psCommonParamTable[99] }, /* Main Encoder mechanical speed (RO) */
    {26015 /*65A5*/, &psCommonParamTable[100] }, /* Main Encoder mechanical acceleration (RO) */
    {26016 /*65A6*/, &psCommonParamTable[101] }, /* Main Encoder electrical angle (RO) */
    {26017 /*65A7*/, &psCommonParamTable[102] }, /* Abs Pos 32 bit (RO) */
    {26018 /*65A8*/, &psCommonParamTable[93] }, /* Status (RO) */

    {26020 /*65A4*/, &psCommonParamTable[111] }, /* Endat clock frequency (RW) */
    {26021 /*65A5*/, &psCommonParamTable[112] }, /* Endat CRC error counter (RO) */
    {26022 /*65A6*/, &psCommonParamTable[113] }, /* Propagation delay time [nsec] (RO) */
    {26023 /*65A7*/, &psCommonParamTable[114] },
    {26024 /*65A8*/, &psCommonParamTable[115] },
    {26025 /*65A9*/, &psCommonParamTable[116] },
    {26026 /*65AA*/, &psCommonParamTable[117] },
    {26027 /*65AB*/, &psCommonParamTable[118] },

    {26030 /*65C0*/, &psCommonParamTable[126] }, /* Incremental encoder line counts (RW) */
// //    {26031 /*65C2*/, #%p Requested index 0x0831 not found in the common database }, /* Incremental encoder mechanical offset (RW) */
    {26032 /*65C2*/, &psCommonParamTable[127] }, /* Incremental encoder poles number (RW) */ 
    {26033 /*65C3*/, &psCommonParamTable[128] }, /* 0 => DO NOT USE SinCosInterpolation; 1 => USE SinCosInterpolation (RW) */
    {26034 /*65C4*/, &psCommonParamTable[129] }, // 0 => Index Error Enabled; 1 => Index Error Disabled (RW)
    {26035 /*65C5*/, &psCommonParamTable[130] }, // 0 => sin^2 + cos^2 Error Enabled; 1 => sin^2 + cos^2 Error Disabled (RW)
    {26036 /*65C6*/, &psCommonParamTable[131] }, // sin^2 + cos^2 Error threshold (RW)
    {26037 /*65C7*/, &psCommonParamTable[132] }, // Index Error Tolerance (RW)
    {26038 /*65C8*/, &psCommonParamTable[133] }, // Has Index (RW)
    {26039 /*65C9*/, &psCommonParamTable[134] }, // SIN channel as read from ADC
    {26040 /*65CA*/, &psCommonParamTable[135] }, // COS channel as read from ADC
    {26041 /*65CB*/, &psCommonParamTable[136] }, // SIN^2+COS^2

    {26042 /*65D2*/, &psCommonParamTable[141] }, /* Hall encoder poles number (RW) */
    {26043 /*65D3*/, &psCommonParamTable[142] }, /* Hall encoder type (RW) */
    {26044 /*65D4*/, &psCommonParamTable[143] }, /* SIN channel as read from ADC (RO) */
    {26045 /*65D5*/, &psCommonParamTable[144] }, /* COS channel as read from ADC (RO) */

    {26048 /*65C8*/, &psCommonParamTable[140] }, // Incremental encoder disable internal calibration

    {26049 /*65C1*/, &psCommonParamTable[137] }, // Incremental encoder Swap Tracks
    {26050 /*65C2*/, &psCommonParamTable[138] }, // Incremental encoder enable step/dir
    {26051 /*65C3*/, &psCommonParamTable[139] }, // Incremental encoder enable up/down

    {26052 /*65E2*/, &psCommonParamTable[145] }, /* SinCos encoder poles number (RW) */ 
    {26053 /*65E3*/, &psCommonParamTable[146] }, /* flag Reverse Signals: 0 => Angle = Atan(Sin/Cos); 1 => Angle = 0xffff - Atan(Sin/Cos) (RW) */
    {26054 /*65E4*/, &psCommonParamTable[147] }, /* flag Gain2Use       : 0 => HIGH (SinCos);         1 => LOW (Resolver) (RW) */
    {26055 /*65E5*/, &psCommonParamTable[148] }, /* SIN channel as read from ADC (RO) */
    {26056 /*65E6*/, &psCommonParamTable[149] }, /* COS channel as read from ADC (RO) */
    {26057 /*65E7*/, &psCommonParamTable[150] }, /* SinCos Level Alarm Threshold (RW) */
    {26058 /*65E8*/, &psCommonParamTable[151] }, /* SIN^2+COS^2 */
    {26059 /*65E9*/, &psCommonParamTable[152] }, /* SinCos SIN Gain adjust */
    {26060 /*65EA*/, &psCommonParamTable[153] }, /* SinCos COS Gain adjust */
    {26061 /*65EB*/, &psCommonParamTable[154] }, /* flag: Enable resolver autocalibration */
    {26062 /*65EC*/, &psCommonParamTable[155] }, /* SinCos SIN Offset adjust */
    {26063 /*65ED*/, &psCommonParamTable[156] }, /* SinCos COS Offset adjust */
    {26064 /*65EE*/, &psCommonParamTable[157] }, /* resolver frequency offset */

    {26066 /*65F0*/, &psCommonParamTable[158], }, // flag: Open Loop only (RW) 
    {26067 /*65F1*/, &psCommonParamTable[159], }, // flag: use dynamic Iq limit (RW)     
    {26068 /*65F2*/, &psCommonParamTable[160], }, // flag: use dynamic Id limit (RW) 
    {26069 /*65F3*/, &psCommonParamTable[161], }, // % per la velocita' di rientro al volo (RW) 
    {26070 /*65F4*/, &psCommonParamTable[162], }, // % per la velocita' di sensorless (RW) 
    {26071 /*65F5*/, &psCommonParamTable[163], }, // % per la velocita' di soglia_1 (RW) 
    {26072 /*65F6*/, &psCommonParamTable[164], }, // % per la velocita' di soglia_0 (RW) 
    {26073 /*65F7*/, &psCommonParamTable[165], }, // % per l'intervento dell'antiglitch (RW) 
    {26074 /*65F8*/, &psCommonParamTable[166], }, // limite di fault antiglitch (RW) 
    {26075 /*65F8*/, &psCommonParamTable[167], }, // usa filtro antiglitch (RW) 
    {26076 /*65F8*/, &psCommonParamTable[168], }, // usa emf speed in full sensorless (RW) 
    {26077 /*65F8*/, &psCommonParamTable[169], }, // disabilita errori sensorless (RW) 
    {26078 /*65FA*/, &psCommonParamTable[170], }, // tempo di refresh per il Kt motore stimato (RW) 
    {26079 /*65FB*/, &psCommonParamTable[171], },
    {26080 /*65FC*/, &psCommonParamTable[172], },

    {26083 /*6601*/, &psCommonParamTable[173], }, // Stato Sensorless (RO) 
    {26084 /*6602*/, &psCommonParamTable[174], }, // contatore intervento antiglitch (RO) 
    {26085 /*6603*/, &psCommonParamTable[175], }, // numero magico (derivato da Kt Motore): se zero => Alarm (RO) 
    {26086 /*6604*/, &psCommonParamTable[176], }, // velocita' ottenuta come delta di angoli (RO) 
    {26087 /*6605*/, &psCommonParamTable[177], }, // Kt motore stimato (RO) 

    /* ==== Main Biss Encoder ==== */
    {26088 /*6606*/, &psCommonParamTable[119] }, /* Biss clock frequency (RW) */
    {26089 /*6607*/, &psCommonParamTable[120] }, /* Biss CRC error counter (RO) */
    {26090 /*6609*/, &psCommonParamTable[121] },
    {26091 /*6609*/, &psCommonParamTable[122] },
    {26092 /*660A*/, &psCommonParamTable[123] },
    {26094 /*660C*/, &psCommonParamTable[124] },
    {26095 /*660D*/, &psCommonParamTable[125] },

    {26100 /*65F4*/, &psCommonParamTable[89] },
    {26101 /*65F5*/, &psCommonParamTable[90] },
    {26102 /*65F6*/, &psCommonParamTable[91] },
    {26103 /*65F7*/, &psCommonParamTable[92] },
    {26104 /*65F8*/, &psCommonParamTable[267] },
    {26105 /*65F9*/, &psCommonParamTable[268] },
#ifdef _HW_DC
    {26106 /*65FA*/, &psCommonParamTable[269] },
#endif
    {26107 /*65FB*/, &psCommonParamTable[270] },

    /* ==== Auxiliary ==== */
    {26110 /*6610*/, &psCommonParamTable[178] }, /* Auxiliary sensor mechanical turns number (RO) */
    {26111 /*6611*/, &psCommonParamTable[179] }, /* Auxiliary sensor mechanical angle (RO) */
    {26112 /*65A2*/, &psCommonParamTable[180] }, /* Auxiliary sensor mechanical Abs position offset HI (RO) */
    {26113 /*65A3*/, &psCommonParamTable[181] }, /* Auxiliary sensor mechanical Abs position offset LO (RO) */
    {26114 /*6612*/, &psCommonParamTable[182] }, /* Auxiliary sensor mechanical speed (RO) */
    {26115 /*6613*/, &psCommonParamTable[183] }, /* Auxiliary sensor mechanical acceleration (RO) */
    {26116 /*6614*/, &psCommonParamTable[184] }, /* Auxiliary sensor electrical angle (RO) */
    {26117 /*6615*/, &psCommonParamTable[185] }, /* Abs Pos 32 bit (RO) */
    {26118 /*6616*/, &psCommonParamTable[186] }, /* Status (RO) */

#ifndef _HW_DC
//     {26120 /*6608*/, #%p Requested index 0x0890 not found in the common database }, /* Endat clock frequency (RW) */
//     {26121 /*6609*/, #%p Requested index 0x0891 not found in the common database }, /* Endat CRC error counter (RO) */
//     {26122 /*660A*/, #%p Requested index 0x0892 not found in the common database }, /* Propagation delay time [nsec] (RO) */
//     {26123 /*660B*/, #%p Requested index 0x0893 not found in the common database },
//     {26124 /*660C*/, #%p Requested index 0x0894 not found in the common database },
//     {26125 /*660D*/, #%p Requested index 0x0895 not found in the common database },
//     {26126 /*660E*/, #%p Requested index 0x0896 not found in the common database },
//     {26127 /*660F*/, #%p Requested index 0x0897 not found in the common database },
#endif

    {26130 /*6630*/, &psCommonParamTable[194] }, /* Incremental encoder line counts (RW) */
// //    {26131 /*6631*/, #%p Requested index 0x08A1 not found in the common database }, /* Incremental encoder mechanical offset (RW) */
    {26132 /*6632*/, &psCommonParamTable[195] }, /* Incremental encoder poles number (RW) */   
    {26134 /*6634*/, &psCommonParamTable[196] }, // 0 => Index Error Enabled; 1 => Index Error Disabled (RW)
    {26135 /*6635*/, &psCommonParamTable[197] }, // Index Error Tolerance (RW)
    {26136 /*6636*/, &psCommonParamTable[198] }, // Has Index (RW)
    {26137 /*6637*/, &psCommonParamTable[199] }, // Incremental encoder Swap Tracks
    {26138 /*6638*/, &psCommonParamTable[200] }, // Incremental encoder enable step/dir
    {26139 /*6639*/, &psCommonParamTable[201] }, // Incremental encoder enable up/down

    /* ==== Simulated Encoder ==== */
    {26146 /*6640*/, &psCommonParamTable[202] }, // Simulated Encoder: Simulated Encoder Index Line Counts
    {26147 /*6641*/, &psCommonParamTable[203] }, // Simulated Encoder: index position offset
    {26148 /*6642*/, &psCommonParamTable[204] }, // Simulated Encoder: max angle diff before alarm
    {26149 /*6643*/, &psCommonParamTable[205] },
    {26150 /*6644*/, &psCommonParamTable[206] },
    {26151 /*6645*/, &psCommonParamTable[207] },
    {26152 /*6646*/, &psCommonParamTable[208] },
    {26153 /*6647*/, &psCommonParamTable[209] },
    {26154 /*6648*/, &psCommonParamTable[210] },

    /* ==== Hiperface Encoder ==== */
    {26160         , &psCommonParamTable[211] }, // Hiperface Encoder

    /* ==== Nikon Encoder ==== */
    {26200         , &psCommonParamTable[212] },
    {26201         , &psCommonParamTable[213] },
    {26210         , &psCommonParamTable[214] },
    {26211         , &psCommonParamTable[215] },
    {26212         , &psCommonParamTable[216] },
    {26213         , &psCommonParamTable[217] },

    /* ==== Tamagawa Encoder ==== */
    {26240         , &psCommonParamTable[218] },
    {26241         , &psCommonParamTable[219] },
    {26250         , &psCommonParamTable[220] },
    {26251         , &psCommonParamTable[221] },
    {26252         , &psCommonParamTable[222] },

    /* ==== Aux Biss Encoder ==== */
    {26260         , &psCommonParamTable[187] }, /* Biss clock frequency (RW) */
    {26261         , &psCommonParamTable[188] }, /* Biss CRC error counter (RO) */
    {26262         , &psCommonParamTable[189] },
    {26263         , &psCommonParamTable[190] },
    {26264         , &psCommonParamTable[191] },
    {26266         , &psCommonParamTable[192] },
    {26267         , &psCommonParamTable[193] },

    /* ==== Electrical Field Orientation Parameters ======== */
    {26302 /*66BE*/, &psCommonParamTable[225], },
    {26303 /*66BF*/, &psCommonParamTable[226], },
    {26304 /*66C0*/, &psCommonParamTable[227], },
    {26305 /*66C1*/, &psCommonParamTable[223], },
    {26306 /*66C2*/, &psCommonParamTable[224], },
    {26307 /*66C3*/, &psCommonParamTable[228], },
    {26308 /*66C4*/, &psCommonParamTable[229], },
    {26309 /*66C5*/, &psCommonParamTable[230], },
    {26310         , &psCommonParamTable[231], },
    {26311         , &psCommonParamTable[232], },
    {26312         , &psCommonParamTable[233], },
    {26313         , &psCommonParamTable[234], },
    {26314         , &psCommonParamTable[235], },
    {26330         , &psCommonParamTable[287], },
    {26331         , &psCommonParamTable[288], },
    {26332         , &psCommonParamTable[289], },
    {26333         , &psCommonParamTable[290], },
    {26334         , &psCommonParamTable[291], },
    {26335         , &psCommonParamTable[292], },
    {26336         , &psCommonParamTable[293], },
    {26337         , &psCommonParamTable[294], },
    {26338         , &psCommonParamTable[295], },

    /* ====== Main Abs ====== */
    {26400 /*65A0*/, &psCommonParamTable[236] }, /* mechanical turns number (RO) */
    {26401 /*65A1*/, &psCommonParamTable[237] }, /* mechanical angle (RO) */
    {26402 /*65A2*/, &psCommonParamTable[238] }, /* mechanical Abs position offset HI (RO) */
    {26403 /*65A3*/, &psCommonParamTable[239] }, /* mechanical Abs position offset LO (RO) */
    {26404 /*65A4*/, &psCommonParamTable[240] }, /* mechanical speed (RO) */
    {26405 /*65A5*/, &psCommonParamTable[241] }, /* mechanical acceleration (RO) */
    {26406 /*65A6*/, &psCommonParamTable[242] }, /* electrical angle (RO) */
    {26407 /*65A7*/, &psCommonParamTable[243] }, /* Abs Pos 32 bit (RO) */
    {26408 /*65A8*/, &psCommonParamTable[244] }, /* Status (RO) */

    /* ====== Main Rel ====== */
    {26410 /*65A0*/, &psCommonParamTable[245] }, /* mechanical turns number (RO) */
    {26411 /*65A1*/, &psCommonParamTable[246] }, /* mechanical angle (RO) */
    {26412 /*65A2*/, &psCommonParamTable[247] }, /* mechanical Abs position offset HI (RO) */
    {26413 /*65A3*/, &psCommonParamTable[248] }, /* mechanical Abs position offset LO (RO) */
    {26414 /*65A4*/, &psCommonParamTable[249] }, /* mechanical speed (RO) */
    {26415 /*65A5*/, &psCommonParamTable[250] }, /* mechanical acceleration (RO) */
    {26416 /*65A6*/, &psCommonParamTable[251] }, /* electrical angle (RO) */
    {26417 /*65A7*/, &psCommonParamTable[252] }, /* Abs Pos 32 bit (RO) */
    {26418 /*65A8*/, &psCommonParamTable[253] }, /* Status (RO) */

    /* ######## SpaceSpeed Control Loop ######## */
    {27000 /*6978*/, &psCommonParamTable[296] }, /* Control loop configuration bit (RW) */                                                                        
    {27001 /*6979*/, &psCommonParamTable[297] }, /* Position: proportional gain value (RW) */                                                  
    {27002 /*697A*/, &psCommonParamTable[298] }, /* Speed: reference proportional gain value (RW) */                                            
    {27003 /*697B*/, &psCommonParamTable[299] }, /* Speed: feedback  proportional gain value (RW) */                                            
    {27004 /*697C*/, &psCommonParamTable[300] }, /* Acceleration: reference proportional gain value (RW) */                                     
    {27005 /*697D*/, &psCommonParamTable[301] }, /* Acceleration: feedback  proportional gain value (RW) */                                     
    {27006 /*697E*/, &psCommonParamTable[302] }, /* integral gain value (RW) */                                                                 
    {27007 /*697F*/, &psCommonParamTable[303] }, /* Position: global gain shift => ONLY shift RIGHT, max +16 (RW) */                            
    {27008 /*6980*/, &psCommonParamTable[304] }, /* Acceleration: gains shift => ONLY shift LEFT, max -15 (RW) */                               
    {27009 /*6981*/, &psCommonParamTable[305] }, /* gains global shift: max shift left = -8; max shift right = +16 (RW) */                                                           
    {27012 /*6984*/, &psCommonParamTable[306] }, /* User: Max Positive Torque limit (RW) */                                                    
    {27013 /*6985*/, &psCommonParamTable[307] }, /* User: Max Negative Torque limit (RW) */                                                     
    {27014         , &psCommonParamTable[308] },
    {27015         , &psCommonParamTable[309] },
    {27016         , &psCommonParamTable[310] },
    {27017         , &psCommonParamTable[311] },
    {27018         , &psCommonParamTable[312] },
    {27019         , &psCommonParamTable[313] },
                                                                                                                             
        /* ============== Monitor ============= */                                                                               
        /* -------------- Input  -------------- */
    {27020 /* */, &psCommonParamTable[103] }, // Mechanical turns number used by control loop (Actual Position) (RO) 
    {27021 /* */, &psCommonParamTable[104] }, // Mechanical angle to control loop (Actual Position) (RO)                      
    {27022 /* */, &psCommonParamTable[105] }, // Mechanical Abs position offset HI used by control loop (RO)                        
    {27023 /* */, &psCommonParamTable[106] }, // Mechanical Abs position offset LO used by control loop (RO)                        
    {27024 /* */, &psCommonParamTable[107] }, // Mechanical speed used by control loop (Actual Speed) (RO)                       
    {27025 /* */, &psCommonParamTable[108] }, // Mechanical acceleration used by control loop (Actual Acceleration) (RO)                       
    {27026 /* */, &psCommonParamTable[109] }, // Electrical angle used by control loop (Actual Acceleration) (RO)
    {27027 /* */, &psCommonParamTable[110] }, /* Abs Pos 32 bit (RO) */
    {27028 /* */, &psCommonParamTable[94] }, /* Status (RO) */
                      
        /* -------------- Output -------------- */
    {27029 /*6995*/, &psCommonParamTable[317] }, /* Torque reference to FPGA (1e-4A) (RO) */
    {27030         , &psCommonParamTable[318] },
    {27031         , &psCommonParamTable[319] },

    {27032         , &psCommonParamTable[315] },
    {27033         , &psCommonParamTable[316] },

    {27034         , &psCommonParamTable[320] },

    {27040 /* */, &psCommonParamTable[266] }, // Filtered mechanical speed used by control loop (Actual Speed) (RO)                       
  
    /* ######## PI DcBus Control Loop ######## */
    {27050 /*69AA*/, &psCommonParamTable[321] }, /* DcBusRef (1e-1V) (RW) */
    {27051 /*69AB*/, &psCommonParamTable[322] }, /* proportional gain (RW) */
    {27052 /*69AC*/, &psCommonParamTable[323] }, /* integral gain (RW) */
    {27053 /*69AD*/, &psCommonParamTable[324] }, /* gains global shift: max shift left = -8; max shift right = +16 (RW) */  
    {27054 /*69AE*/, &psCommonParamTable[325] }, /* Max Error allowed (RW) */
    {27055 /*69AF*/, &psCommonParamTable[326] }, /* Min Error allowed (RW) */
    {27056 /*69B0*/, &psCommonParamTable[327] }, /* Limite sulla Max Corrente richiesta al DcBus (RW) */
    {27057 /*69B1*/, &psCommonParamTable[328] }, /* Limite sulla Min Corrente richiesta al DcBus (RW) */
// //    {27058 /*69B2*/, #%p Requested index 0x0B08 not found in the common database }, /* Limite sulla Max Corrente richiesta all'FPGA (RW) */
// //    {27059 /*69B3*/, #%p Requested index 0x0B09 not found in the common database }, /* Limite sulla Min Corrente richiesta all'FPGA (RW) */
    {27060 /*69B4*/, &psCommonParamTable[329] }, /* Use only PI part (RW) */

        /* ============== Monitor ============= */                                                                               
        /* -------------- Output -------------- */
    {27070 /*69BE*/, &psCommonParamTable[330] }, /* error measured (RO) */
    {27071 /*69BF*/, &psCommonParamTable[331] }, /* DcBus Current (1e-4A) (RO) */
    {27072 /*69C0*/, &psCommonParamTable[332] }, /* Current reference to FPGA (1e-4A) (RO) */



    /* ######## Positioner ######## */
    {27100 /*69DC*/, &psCommonParamTable[333] }, /* Profile Velocity (RW) */ 
    {27101 /*69DD*/, &psCommonParamTable[334] }, /* Profile Acceleration (RW) */     
    {27102 /*69DE*/, &psCommonParamTable[335] }, /* Profile Deceleration (RW) */          
    {27103 /*69DF*/, &psCommonParamTable[336] }, /* QuickStop Deceleration (RW) */ 
    {27104 /*69E0*/, &psCommonParamTable[337] }, /* End Speed (in modalita' posizionatore) (RW) */
    {27105 /*69E1*/, &psCommonParamTable[338] }, /* Max Position Error allowed (RW) */ 
    {27106 /*69E2*/, &psCommonParamTable[339] }, /* Threshold to consider speed = zero (RW) */                                     
    {27107 /*69E3*/, &psCommonParamTable[340] }, /* Motor Blocked Timeout (RW) */                                     
    {27108         , &psCommonParamTable[341] },
    {27109         , &psCommonParamTable[342] },
    {27110         , &psCommonParamTable[343] },
    {27111         , &psCommonParamTable[344] },
    {27112         , &psCommonParamTable[345] },

        /* ============== Monitor ============== */
        /* --------------- Input --------------- */
    {27118 /*69EE*/, &psCommonParamTable[347] }, /* TargetVelocity (in modalita velocita') (RW) */

        /* --------------- Output --------------- */
    {27120 /*69F0*/, &psCommonParamTable[348] }, /* Flags status modulo sw posizionatore (RO) */
    {27122 /*69F2*/, &psCommonParamTable[349] }, /* posizione di riferimento da mandare in anello: turns (RO) */
    {27123 /*69F3*/, &psCommonParamTable[350] }, /* posizione di riferimento da mandare in anello: angle (RO) */ 
    {27124 /*69F4*/, &psCommonParamTable[351] }, /* velocita' di riferimento da mandare in anello (RO) */
    {27125 /*69F5*/, &psCommonParamTable[352] }, /* accelerazione di riferimento da mandare in anello (RO) */
    {27126 /*69F6*/, &psCommonParamTable[353] }, /* Position error measured (RO) */

    /* ====== Incremental MAIN Extras ====== */
    {27300         , &psCommonParamTable[271] },
    {27301         , &psCommonParamTable[272] },
    {27302         , &psCommonParamTable[273] },
    {27303         , &psCommonParamTable[274] },
    {27304         , &psCommonParamTable[275] },
    {27305         , &psCommonParamTable[276] },
    {27306         , &psCommonParamTable[277] },
    {27307         , &psCommonParamTable[278] },

    /* ====== Incremental AUX Extras ====== */
    {27320         , &psCommonParamTable[279] },
    {27321         , &psCommonParamTable[280] },
    {27322         , &psCommonParamTable[281] },
    {27323         , &psCommonParamTable[282] },
    {27324         , &psCommonParamTable[283] },
    {27325         , &psCommonParamTable[284] },
    {27326         , &psCommonParamTable[285] },
    {27327         , &psCommonParamTable[286] },

        /* ######## Motor Handler ######## */
    {27400 /*6B08*/, &psCommonParamTable[357] }, // tensione di spegnimento R di frenatura (RW)
    {27401 /*6B09*/, &psCommonParamTable[358] }, // tensione di accensiono R di frenatura (RW)
    {27402 /*6B0A*/, &psCommonParamTable[359] }, // soglia fault undervoltage (RW)
    {27403 /*6B0B*/, &psCommonParamTable[360] }, // soglia fault overvoltage (utente) (RW)
    {27404 /*6B0C*/, &psCommonParamTable[361] }, // soglia fault overcurrent (utente) (RW)
    {27407 /*6B0F*/, &psCommonParamTable[362] }, // UserModKi (RW)
    {27408 /*6B10*/, &psCommonParamTable[363] }, // UserModKp (RW)
    {27409 /*6B11*/, &psCommonParamTable[364] }, // Id min (RW)
    {27410 /*6B12*/, &psCommonParamTable[365] }, // Id max (RW)
    {27411 /*6B13*/, &psCommonParamTable[366] }, // Iq min (RW)
    {27412 /*6B14*/, &psCommonParamTable[367] }, // Iq max (RW)
    {27432 /*6B28*/, &psCommonParamTable[378] }, // Id feedback (RO)
    {27433 /*6B29*/, &psCommonParamTable[379] }, // Iq feedback (RO)
    {27434 /*6B2A*/, &psCommonParamTable[380] }, // Iu feedback (RO)
    {27435 /*6B2B*/, &psCommonParamTable[381] }, // Iv feedback (RO)
    {27436 /*6B2C*/, &psCommonParamTable[382] }, // BackEmf Alpha (RO)
    {27437 /*6B2D*/, &psCommonParamTable[383] }, // BackEmf Beta (RO)
    {27438 /*6B2E*/, &psCommonParamTable[384] }, // VuEstimated (RO)
    {27439 /*6B2F*/, &psCommonParamTable[385] }, // VvEstimated (RO)
    {27440 /*6B30*/, &psCommonParamTable[386] }, // DcBusValue (RO)
    {27441 /*6B31*/, &psCommonParamTable[387] }, // VMotor (RO)
    {27442 /*6B32*/, &psCommonParamTable[388] }, // AtanAngle (RO)
    {27443 /*6B33*/, &psCommonParamTable[389] }, // IdRef2Fpga (RO)
    {27444 /*6B34*/, &psCommonParamTable[390] }, // IqRef2Fpga (RO)
    {27445 /*6B35*/, &psCommonParamTable[391] }, // Iw feedback (RO)
    {27446 /*6B36*/, &psCommonParamTable[392] }, // Iq Filtered Ref (RO)
    {27449 /*6B39*/, &psCommonParamTable[395] }, // Vac input phases status (RO)
    {27450 /*6B3A*/, &psCommonParamTable[393] }, // Vac RS (RO)
    {27451 /*6B3B*/, &psCommonParamTable[394] }, // Vac ST (RO)
    {27452 /*6B3C*/, &psCommonParamTable[396] }, // Vac TR (RO)
    {27453 /*6B3D*/, &psCommonParamTable[397] }, // Vd Out (RO)
    {27454 /*6B3E*/, &psCommonParamTable[398] }, // Vq Out (RO)
    {27464 /*6B48*/, &psCommonParamTable[399] }, // UsrIdRef (Torque mode) (RW)
    {27465 /*6B49*/, &psCommonParamTable[400] }, // UsrIqRef (Torque mode) (RW)
    {27480 /*6B58*/, &psCommonParamTable[402] }, // AutoModKp (RO)
    {27481 /*6B59*/, &psCommonParamTable[403] }, // AutoModKi (RO)
    {27482 /*6B5A*/, &psCommonParamTable[404] },
    {27483 /*6B5B*/, &psCommonParamTable[405] },
    {27484 /*6B5C*/, &psCommonParamTable[406] },
    {27485 /*6B5D*/, &psCommonParamTable[407] },
    {27489 /*6B61*/, &psCommonParamTable[109] }, // Main sensor electrical angle used by FPGA (RO)
    {27490 /*6B62*/, &psCommonParamTable[371] }, // user bridge layout (RW)                 
    {27496         , &psCommonParamTable[408] },
    {27497         , &psCommonParamTable[409] },
    {27498         , &psCommonParamTable[410] },
    {27499         , &psCommonParamTable[411] },
    {27500         , &psCommonParamTable[412] },
    {27501         , &psCommonParamTable[413] },
    {27502         , &psCommonParamTable[414] },
    {27503         , &psCommonParamTable[415] },
    {27504         , &psCommonParamTable[416] },
    {27505         , &psCommonParamTable[372] },
    {27506         , &psCommonParamTable[373] },
    {27507         , &psCommonParamTable[374] },
    {27508         , &psCommonParamTable[375] },
    {27509         , &psCommonParamTable[376] },
    {27510         , &psCommonParamTable[377] },
    {27511         , &psCommonParamTable[417] },
    {27512         , &psCommonParamTable[401] },
#ifdef _HW_DC
    {27513         , &psCommonParamTable[370] },
    {27514         , &psCommonParamTable[369] },
#endif
    {27515         , &psCommonParamTable[368] },
#if CFG_VMOTOR_READ
    {27516 /*6B7C*/, &psCommonParamTable[438] }, // Vu Effective (RO)
    {27517 /*6B7D*/, &psCommonParamTable[439] }, // Vv Effective (RO)
    {27518 /*6B7E*/, &psCommonParamTable[440] }, // Vw Effective (RO)
#endif

    {27600         , &psCommonParamTable[418] },
    {27601         , &psCommonParamTable[419] },
    {27602         , &psCommonParamTable[420] },
    {27603         , &psCommonParamTable[421] },
    {27604         , &psCommonParamTable[422] },
    {27610         , &psCommonParamTable[423] },
    {27611         , &psCommonParamTable[424] },
    {27612         , &psCommonParamTable[425] },
    {27613         , &psCommonParamTable[426] },
    {27614         , &psCommonParamTable[427] },
    {27620         , &psCommonParamTable[428] },
    {27621         , &psCommonParamTable[429] },
    {27622         , &psCommonParamTable[430] },
    {27623         , &psCommonParamTable[431] },
    {27624         , &psCommonParamTable[432] },
    {27630         , &psCommonParamTable[433] },
    {27631         , &psCommonParamTable[434] },
    {27632         , &psCommonParamTable[435] },
    {27633         , &psCommonParamTable[436] },
    {27634         , &psCommonParamTable[437] },

#ifdef _HW_DC
    {27648         , &psCommonParamTable[441] },
    {27649         , &psCommonParamTable[442] },
    {27650         , &psCommonParamTable[443] },
    {27651         , &psCommonParamTable[444] },
    {27652         , &psCommonParamTable[445] },
    {27653         , &psCommonParamTable[446] },
#endif

#ifdef _HW_CT
//     {27654         , #%p Requested index 0x0E8E not found in the common database },
#endif

    {27660         , &psCommonParamTable[447] },
    {27661         , &psCommonParamTable[448] },
    {27662         , &psCommonParamTable[449] },
    {27670         , &psCommonParamTable[450] },
    {27671         , &psCommonParamTable[451] },
    {27672         , &psCommonParamTable[452] },
    {27673         , &psCommonParamTable[453] },

        /* ######## Motor Parameters ######## */
    {27800 /*6C98*/, &psCommonParamTable[507] },  // SerialNumber (RO)
    {27801 /*6C99*/, &psCommonParamTable[508] },  // ProductionDate (RO)
    {27802 /*6C9A*/, &psCommonParamTable[509] },  // Model (RO)
    {27803 /*6C9B*/, &psCommonParamTable[510] },  // Resistance (RW)
    {27804 /*6C9C*/, &psCommonParamTable[511] },  // Inductance (RW)
    {27805 /*6C9D*/, &psCommonParamTable[512] },  // KT (RW)
    {27806 /*6C9E*/, &psCommonParamTable[513] },  // CurrentNominalZeroSpeed (RW)
    {27807 /*6C9F*/, &psCommonParamTable[514] },  // CurrentNominal (RW)
    {27808 /*6CA0*/, &psCommonParamTable[515] },  // CurrentPeak (RW)
    {27809 /*6CA1*/, &psCommonParamTable[516] },  // SpeedNominal (RW)
    {27810 /*6CA2*/, &psCommonParamTable[517] },  // ThermalConstant (RW)
    {27811 /*6CA3*/, &psCommonParamTable[518] },  // StatorInertia (RW)
    {27812 /*6CA4*/, &psCommonParamTable[519] },  // PhaseOffset(RW)
    {27813 /*6CA5*/, &psCommonParamTable[520] },  // Type (RW)
    {27814 /*6CA6*/, &psCommonParamTable[521] },  // PoleNumbers (RW)
    {27815 /*6CA7*/, &psCommonParamTable[522] },  // CoolingTempOn (RW)
    {27816 /*6CA8*/, &psCommonParamTable[523] },  // CoolingTempOff (RW)
    {27817 /*6CA9*/, &psCommonParamTable[524] },  // MaximumTemp (RW)
    {27818 /*6CAA*/, &psCommonParamTable[525] },  // Synchronous or direct inductance (RW)



    /* ######### Thermal Model ######### */
    {28100 /*6C98*/, &psCommonParamTable[526] }, /* R Brake Value (RW)     */
    {28101 /*6C99*/, &psCommonParamTable[527] }, /* R Brake Max Power (RW) */
    {28104 /*6C9C*/, &psCommonParamTable[528] }, /* Cooling Temp ON (RW)   */
    {28105 /*6C9D*/, &psCommonParamTable[529] }, /* Cooling Temp OFF (RW)  */ 
    {28106 /*6C9E*/, &psCommonParamTable[530] }, // motor temp (RW)
    {28107 /*6C9F*/, &psCommonParamTable[531] }, // R Brake Max Energy (RW)
    {28108 /*6CA0*/, &psCommonParamTable[532] },
    {28109 /*6CA1*/, &psCommonParamTable[533] },
    {28110 /*6CA2*/, &psCommonParamTable[534] },

#ifdef _HW_DC
    {28130         , &psCommonParamTable[535] },
    {28131         , &psCommonParamTable[536] },
    {28132         , &psCommonParamTable[537] },
    {28133         , &psCommonParamTable[538] },
#endif
    {28135         , &psCommonParamTable[539] },
    {28136         , &psCommonParamTable[540] },
    {28137         , &psCommonParamTable[541] },
    {28138         , &psCommonParamTable[542] },

    // Output values (RO)
    {28150 /*6C9E*/, &psCommonParamTable[544] }, /* DcBus Average Value        (1e-1 V) */
    {28151 /*6C9F*/, &psCommonParamTable[545] }, /* Iu Average Value           (1e-4 A) */
    {28152 /*6CA0*/, &psCommonParamTable[546] }, /* Iv Average Value           (1e-4 A) */
    {28153 /*6CA1*/, &psCommonParamTable[547] }, /* Iw Average Value           (1e-4 A) */
    {28154 /*6CA2*/, &psCommonParamTable[548] }, /* Temperature from NTC       (1e-1 C) */
    {28155 /*6CA3*/, &psCommonParamTable[549] }, /* Commutation Losses Phase U (1e-3 W) */
    {28156 /*6CA4*/, &psCommonParamTable[550] }, /* Commutation Losses Phase V (1e-3 W) */
    {28157 /*6CA5*/, &psCommonParamTable[551] }, /* Commutation Losses Phase W (1e-3 W) */
    {28158 /*6CA6*/, &psCommonParamTable[552] }, /* Conduction Losses Phase U  (1e-3 W) */
    {28159 /*6CA7*/, &psCommonParamTable[553] }, /* Conduction Losses Phase V  (1e-3 W) */
    {28160 /*6CA8*/, &psCommonParamTable[554] }, /* Conduction Losses Phase W  (1e-3 W) */
    {28161 /*6CA9*/, &psCommonParamTable[555] }, /* Total Losses               (1e-3 W) */
    {28162 /*6C98*/, &psCommonParamTable[556] }, /* Max TJunction for the IGBT (1e-1 C) */
    {28163 /*6C99*/, &psCommonParamTable[557] }, /* TJunction U (model)        (1e-1 C) */
    {28164 /*6C9A*/, &psCommonParamTable[558] }, /* TJunction V (model)        (1e-1 C) */
    {28165 /*6C9B*/, &psCommonParamTable[559] }, /* TJunction W (model)        (1e-1 C) */
    {28166 /*6C9C*/, &psCommonParamTable[560] }, /* Max TJunction   (model)    (1e-1 C) */
    {28167 /*6C9D*/, &psCommonParamTable[561] }, /* Power Dissipated by RBrake (1e-3 W) */
    {28168 /*6C9E*/, &psCommonParamTable[562] }, /* T HeatSink (model)         (1e-1 C) */
    {28169 /*6C9F*/, &psCommonParamTable[563] }, /* IqLimitMax */
    {28170 /*6CA0*/, &psCommonParamTable[564] }, /* IqLimitMin */
    {28171 /*6CA1*/, &psCommonParamTable[565] }, /* IdLimitMax */
    {28172 /*6CA2*/, &psCommonParamTable[566] }, /* IdLimitMin */
    {28173 /*6CA3*/, &psCommonParamTable[567] }, /* ThermalModel Duration (1e-3 s) */
    {28174 /*6CA4*/, &psCommonParamTable[568] }, // Motor temperature (C)
    {28176 /*6CA6*/, &psCommonParamTable[570] },
    {28177 /*6CA7*/, &psCommonParamTable[571] },
    {28178 /*6CA8*/, &psCommonParamTable[572] },
    {28179 /*6CA9*/, &psCommonParamTable[573] },
#ifdef _HW_DC
    {28180 /*6CAA*/, &psCommonParamTable[574] },
    {28181 /*6CAB*/, &psCommonParamTable[575] },
    {28182 /*6CAC*/, &psCommonParamTable[576] },
    {28183 /*6CAD*/, &psCommonParamTable[577] },
    {28184 /*6CAE*/, &psCommonParamTable[578] },
#endif
#ifdef _HW_CT
//     {28185 /*6CAA*/, #%p Requested index 0x312F not found in the common database }, /* T Control Board (C)     */
//     {28186 /*6CAB*/, #%p Requested index 0x3130 not found in the common database }, /* T Control Board (1e-1 C) */
#endif
#ifdef _HW_CT
//     {28187 /*6CAC*/, #%p Requested index 0x4010 not found in the common database }, /* Sensor Accelerometer Data x */
//     {28188 /*6CAD*/, #%p Requested index 0x4011 not found in the common database }, /* Sensor Accelerometer Data y */
//     {28189 /*6CAE*/, #%p Requested index 0x4012 not found in the common database }, /* Sensor Accelerometer Data x */
//     {28190 /*6CAF*/, #%p Requested index 0x4013 not found in the common database }, /* Sensor Temperature          */
//     {28191 /*6CB0*/, #%p Requested index 0x4014 not found in the common database }, /* Sensor Humidity             */
//     {28192 /*6CB1*/, #%p Requested index 0x4000 not found in the common database }, 
//     {28193 /*6CB2*/, #%p Requested index 0x4001 not found in the common database }, 
//     {28194 /*6CB3*/, #%p Requested index 0x4002 not found in the common database }, 
#endif

#ifndef _INFINEON
// _crs_dbg_
    {28195 /*6E23*/, &psCommonParamTable[579] }, /* T ZynQ (Celsius) */
    {28196 /*6E24*/, &psCommonParamTable[580] }, /* T controlboard NTC (Celsius) */
// _crs_dbg_
#endif
    // 28300 to 28354: do not use

    /* ######### DS402 ######### */
    {29000 /*7148*/, &psCommonParamTable[454] }, // ControlWord (RW)
    {29001 /*7149*/, &psCommonParamTable[455] }, // StatusWord (RO)
    {29002 /*7149*/, &psCommonParamTable[459] }, // Tgt Pos HI (RW)
    {29003 /*7149*/, &psCommonParamTable[460] }, // TGT Pos LO (RW)
    {29007 /*714F*/, &psCommonParamTable[466] }, // HM Soft Positive Switch input (RW)
    {29008 /*7150*/, &psCommonParamTable[467] }, // HM Soft Negative Switch input (RW)
    {29009 /*7151*/, &psCommonParamTable[468] }, // HM Soft Home Switch input (RW)

    {29010 /*7152*/, &psCommonParamTable[469] }, // IP Quota monitor (RO)

    {29016 /*7158*/, &psCommonParamTable[457] }, // Mode of Operation (RW)
    {29017 /*7159*/, &psCommonParamTable[458] }, // Mode of Operation Display (RO)

    {29020 /*715C*/, &psCommonParamTable[473] }, // quickstop option code (RW)
    {29021 /*715D*/, &psCommonParamTable[474] }, // shutdown option code (RW)
    {29022 /*715E*/, &psCommonParamTable[475] }, // disable operation option code (RW)
    {29023 /*715F*/, &psCommonParamTable[476] }, // halt option code (RW)
    {29024 /*7160*/, &psCommonParamTable[477] }, // fault reaction option code (RW)
    {29025 /*7161*/, &psCommonParamTable[478] }, // following error window (RW)
    {29026 /*7162*/, &psCommonParamTable[479] }, // following error timeout (RW)
    {29027 /*7163*/, &psCommonParamTable[480] }, // target position window (RW)
    {29028 /*7164*/, &psCommonParamTable[481] }, // target position timeout (RW)
    {29029 /*7165*/, &psCommonParamTable[482] }, // velocity window (RW)
    {29030 /*7166*/, &psCommonParamTable[483] }, // velocity timeout (RW)
    {29031 /*7167*/, &psCommonParamTable[484] }, // velocity threshold (RW)
    {29032 /*7168*/, &psCommonParamTable[485] }, // velocity threshold timeout (RW)
    {29033 /*7169*/, &psCommonParamTable[492] }, // IP time period units
    {29034 /*716A*/, &psCommonParamTable[493] }, // IP time period index
    {29035 /*716B*/, &psCommonParamTable[495] }, // HM Method
    {29036 /*716C*/, &psCommonParamTable[496] }, // HM positive switch source
    {29037 /*716D*/, &psCommonParamTable[497] }, // HM negative switch source
    {29038 /*716E*/, &psCommonParamTable[498] }, // HM home switch source
    {29039 /*716F*/, &psCommonParamTable[499] }, // HM speed for switch search
    {29040 /*7170*/, &psCommonParamTable[500] }, // HM speed for zero search
    {29041 /*7171*/, &psCommonParamTable[501] }, // HM acceleration
    {29042 /*7172*/, &psCommonParamTable[505] }, // HM user offset
    {29043 /*7173*/, &psCommonParamTable[506] }, // HM user offset
    {29044 /*7174*/, &psCommonParamTable[504] }, // CS motor rated torque

    /* ######### CanOpen ######### */
    {30000 /*7530*/, &psCommonParamTable[642] },
    {30001 /*7531*/, &psCommonParamTable[643] },
    {30002 /*7532*/, &psCommonParamTable[644] },
    {30003 /*7533*/, &psCommonParamTable[645] },
    {30004 /*7534*/, &psCommonParamTable[646] },
    {30005 /*7535*/, &psCommonParamTable[647] },
    {30006 /*7536*/, &psCommonParamTable[648] },
    {30007 /*7537*/, &psCommonParamTable[649] },
    {30008 /*7538*/, &psCommonParamTable[650] },
    {30009 /*7539*/, &psCommonParamTable[651] },
    {30010 /*753A*/, &psCommonParamTable[652] },
    {30011 /*753B*/, &psCommonParamTable[653] },
    {30012 /*753C*/, &psCommonParamTable[654] },
    {30013 /*753D*/, &psCommonParamTable[655] },

    {30100         , &psCommonParamTable[660] },
    {30110         , &psCommonParamTable[661] },
    {30120         , &psCommonParamTable[662] },
    {30130         , &psCommonParamTable[663] },
    {30140         , &psCommonParamTable[664] },
    {30150         , &psCommonParamTable[665] },
    {30160         , &psCommonParamTable[666] },
    {30170         , &psCommonParamTable[667] },
    {30200         , &psCommonParamTable[668] },
    {30210         , &psCommonParamTable[669] },
    {30220         , &psCommonParamTable[670] },
    {30230         , &psCommonParamTable[671] },
    {30240         , &psCommonParamTable[672] },
    {30250         , &psCommonParamTable[673] },
    {30260         , &psCommonParamTable[674] },
    {30270         , &psCommonParamTable[675] },
    {30300         , &psCommonParamTable[676] },
    {30310         , &psCommonParamTable[677] },
    {30320         , &psCommonParamTable[678] },
    {30330         , &psCommonParamTable[679] },
    {30340         , &psCommonParamTable[680] },
    {30350         , &psCommonParamTable[681] },
    {30360         , &psCommonParamTable[682] },
    {30370         , &psCommonParamTable[683] },
    {30400         , &psCommonParamTable[684] },
    {30410         , &psCommonParamTable[685] },
    {30420         , &psCommonParamTable[686] },
    {30430         , &psCommonParamTable[687] },
    {30440         , &psCommonParamTable[688] },
    {30450         , &psCommonParamTable[689] },
    {30460         , &psCommonParamTable[690] },
    {30470         , &psCommonParamTable[691] },

    /* ######### EtherCAT ######### */
//     {30800         , #%p Requested index 0x8108 not found in the common database },
//     {30801         , #%p Requested index 0x8109 not found in the common database },
//     {30802         , #%p Requested index 0x810A not found in the common database },

//     {30810         , #%p Requested index 0x8180 not found in the common database },
//     {30811         , #%p Requested index 0x8181 not found in the common database },
//     {30812         , #%p Requested index 0x8182 not found in the common database },
//     {30813         , #%p Requested index 0x8183 not found in the common database },
//     {30814         , #%p Requested index 0x8184 not found in the common database },
//     {30815         , #%p Requested index 0x8185 not found in the common database },
//     {30816         , #%p Requested index 0x8186 not found in the common database },
//     {30817         , #%p Requested index 0x8187 not found in the common database },
//     {30818         , #%p Requested index 0x8188 not found in the common database },
//     {30819         , #%p Requested index 0x8189 not found in the common database },
//     {30820         , #%p Requested index 0x818A not found in the common database },
//     {30821         , #%p Requested index 0x818B not found in the common database },
//     {30822         , #%p Requested index 0x818C not found in the common database },
//     {30823         , #%p Requested index 0x818D not found in the common database },
//     {30824         , #%p Requested index 0x818E not found in the common database },
//     {30825         , #%p Requested index 0x811C not found in the common database },
//     {30826         , #%p Requested index 0x818F not found in the common database },
//     {30827         , #%p Requested index 0x8190 not found in the common database },

    /* ####### EtherPMC ####### */
#ifdef _HW_DC
//     {30900         , #%p Requested index 0x8280 not found in the common database },
//     {30908         , #%p Requested index 0x8288 not found in the common database },
//     {30909         , #%p Requested index 0x8289 not found in the common database },
//     {30910         , #%p Requested index 0x828A not found in the common database },
//     {30920         , #%p Requested index 0x8290 not found in the common database },
//     {30921         , #%p Requested index 0x8291 not found in the common database },
//     {30922         , #%p Requested index 0x8292 not found in the common database },
//     {30923         , #%p Requested index 0x8293 not found in the common database },
//     {30924         , #%p Requested index 0x8294 not found in the common database },
#endif

    /* ####### Sync Manager ####### */
    {31000         , &psCommonParamTable[701] },
    {31001         , &psCommonParamTable[702] },
    {31002         , &psCommonParamTable[703] },
    {31003         , &psCommonParamTable[704] },
    {31004         , &psCommonParamTable[705] },
    {31005         , &psCommonParamTable[706] },

    {31010         , &psCommonParamTable[707] },
    {31011         , &psCommonParamTable[708] },
    {31012         , &psCommonParamTable[709] },
    {31013         , &psCommonParamTable[710] },
    {31014         , &psCommonParamTable[711] },

    /* ####### SS PLC requested memory ####### */
    {32768         , &psCommonParamTable[609] },  // array of 16384 words

#ifdef _HW_AXS_CABI35KW
	/* ============== STGAP4S System Values ============== */
//     {50000 /*    */, #%p Requested index 0x4100 not found in the common database }, 
//     {50001 /*    */, #%p Requested index 0x4101 not found in the common database }, 
//     {50002 /*    */, #%p Requested index 0x4102 not found in the common database }, 
//     {50020 /*    */, #%p Requested index 0x4120 not found in the common database }, 
//     {50021 /*    */, #%p Requested index 0x4121 not found in the common database }, 
	/* ============== STGAP4S Configuration Register Params ============== */
//     {50030 /*    */, #%p Requested index 0x4130 not found in the common database }, 
//     {50031 /*    */, #%p Requested index 0x4131 not found in the common database }, 
//     {50032 /*    */, #%p Requested index 0x4132 not found in the common database }, 
//     {50033 /*    */, #%p Requested index 0x4133 not found in the common database }, 
//     {50034 /*    */, #%p Requested index 0x4134 not found in the common database }, 
//     {50035 /*    */, #%p Requested index 0x4135 not found in the common database }, 
//     {50036 /*    */, #%p Requested index 0x4136 not found in the common database }, 
//     {50037 /*    */, #%p Requested index 0x4137 not found in the common database }, 
//     {50038 /*    */, #%p Requested index 0x4138 not found in the common database }, 
//     {50039 /*    */, #%p Requested index 0x4139 not found in the common database }, 
//     {50040 /*    */, #%p Requested index 0x4140 not found in the common database }, 
	/* ============== STGAP4S Phase U Status Registers ============== */
//     {50050 /*    */, #%p Requested index 0x4150 not found in the common database }, 
//     {50051 /*    */, #%p Requested index 0x4151 not found in the common database }, 
//     {50052 /*    */, #%p Requested index 0x4152 not found in the common database }, 
//     {50053 /*    */, #%p Requested index 0x4153 not found in the common database }, 
//     {50054 /*    */, #%p Requested index 0x4154 not found in the common database }, 
//     {50055 /*    */, #%p Requested index 0x4155 not found in the common database }, 
//     {50056 /*    */, #%p Requested index 0x4156 not found in the common database }, 
//     {50057 /*    */, #%p Requested index 0x4157 not found in the common database }, 
//     {50058 /*    */, #%p Requested index 0x4158 not found in the common database }, 
//     {50059 /*    */, #%p Requested index 0x4159 not found in the common database }, 
	/* ============== STGAP4S Phase V Status Registers ============== */
//     {50060 /*    */, #%p Requested index 0x4160 not found in the common database }, 
//     {50061 /*    */, #%p Requested index 0x4161 not found in the common database }, 
//     {50062 /*    */, #%p Requested index 0x4162 not found in the common database }, 
//     {50063 /*    */, #%p Requested index 0x4163 not found in the common database }, 
//     {50064 /*    */, #%p Requested index 0x4164 not found in the common database }, 
//     {50065 /*    */, #%p Requested index 0x4165 not found in the common database }, 
//     {50066 /*    */, #%p Requested index 0x4166 not found in the common database }, 
//     {50067 /*    */, #%p Requested index 0x4167 not found in the common database }, 
//     {50068 /*    */, #%p Requested index 0x4168 not found in the common database }, 
//     {50069 /*    */, #%p Requested index 0x4169 not found in the common database }, 
	/* ============== STGAP4S Phase W Status Registers ============== */
//     {50070 /*    */, #%p Requested index 0x4170 not found in the common database }, 
//     {50071 /*    */, #%p Requested index 0x4171 not found in the common database }, 
//     {50072 /*    */, #%p Requested index 0x4172 not found in the common database }, 
//     {50073 /*    */, #%p Requested index 0x4173 not found in the common database }, 
//     {50074 /*    */, #%p Requested index 0x4174 not found in the common database }, 
//     {50075 /*    */, #%p Requested index 0x4175 not found in the common database }, 
//     {50076 /*    */, #%p Requested index 0x4176 not found in the common database }, 
//     {50077 /*    */, #%p Requested index 0x4177 not found in the common database }, 
//     {50078 /*    */, #%p Requested index 0x4178 not found in the common database }, 
//     {50079 /*    */, #%p Requested index 0x4179 not found in the common database }, 
	/* ============== STGAP4S System Values ============== */
//     {50080 /*    */, #%p Requested index 0x4180 not found in the common database }, 
	/* ============== STGAP4S Configuration Register Values ============== */
//     {50090 /*    */, #%p Requested index 0x4190 not found in the common database }, 
//     {50091 /*    */, #%p Requested index 0x4191 not found in the common database }, 
//     {50092 /*    */, #%p Requested index 0x4192 not found in the common database }, 
//     {50093 /*    */, #%p Requested index 0x4193 not found in the common database }, 
//     {50094 /*    */, #%p Requested index 0x4194 not found in the common database }, 
//     {50095 /*    */, #%p Requested index 0x4195 not found in the common database }, 
//     {50096 /*    */, #%p Requested index 0x4196 not found in the common database }, 
//     {50097 /*    */, #%p Requested index 0x4197 not found in the common database }, 
//     {50098 /*    */, #%p Requested index 0x4198 not found in the common database }, 
//     {50099 /*    */, #%p Requested index 0x4199 not found in the common database }, 
//     {50100 /*    */, #%p Requested index 0x4200 not found in the common database }, 
#endif

};

const UWORD uwModBusParamCount=sizeof(hpsModBusParamTable)/sizeof(MODBUSCOMDB_ENTRY);

//****************************************************************************
// Coils table def
// #%n psCommonParamTable

const MODBUSCOMDB_ENTRY hpsModBusCoilsTable[]=
{
    /* ####### User digital I/O ############## */
    {16384         , &psCommonParamTable[610] },
    {16416         , &psCommonParamTable[611] },

    /* ####### SS PLC requested memory ####### */
    {32768         , &psCommonParamTable[609] },
};

const UWORD uwModBusCoilsCount=sizeof(hpsModBusCoilsTable)/sizeof(MODBUSCOMDB_ENTRY);
