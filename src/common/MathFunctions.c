/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : MathFunctions.c                                            */
/* Author      : Stefano Martino                                            */
/*               Hu Xiaokai                                                 */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/****************************************************************************/
#pragma GCC optimize (2)

#include <stdio.h>                   // standard I/O .h-file
#include <stdlib.h>


#include "CommonDefines.h"

#ifndef _RD
//#include "AxM-E-Defines.h"
#else
#include "RemDisp-Defines.h"
#endif


/////////////////////////////////////////////////////////////////////////////
// Arc Tangent Tables

// AxM... VECON/Bertocci
#if 0
UWORD const wTabArcTan32A[]  = {  0x0000, 0x0145, 0x028a, 0x03ce, 0x0510,
                                  0x0650, 0x078d, 0x08c5, 0x09fa, 0x0b2b,
                                  0x0c56, 0x0d7d, 0x0e9d, 0x0fb8, 0x10cd,
                                  0x11db, 0x12e3, 0x13e4, 0x14df, 0x15d4,
                                  0x16c1, 0x17a8, 0x1889, 0x1963, 0x1a37,
                                  0x1b04, 0x1bcb, 0x1c8d, 0x1d48, 0x1dfe,
                                  0x1eae, 0x1f59, 0x1fff
                               };

//#if 0
// VC++
UWORD const wTabArcTan32B[]  = {  0x0000, 0x0146, 0x028B, 0x03CF, 0x0511, 0x0650, 0x078D, 0x08C6, 
                                  0x09FB, 0x0B2B, 0x0C57, 0x0D7D, 0x0E9E, 0x0FB8, 0x10CD, 0x11DB, 
                                  0x12E3, 0x13E5, 0x14E0, 0x15D4, 0x16C2, 0x17A9, 0x1889, 0x1963, 
                                  0x1A37, 0x1B05, 0x1BCC, 0x1C8D, 0x1D49, 0x1DFF, 0x1EAF, 0x1F59, 
                                  0x1FFF
                               };
#endif

// VC++
UWORD const wTabArcTan128[]  = {  0x0000, 0x0051, 0x00A3, 0x00F4, 0x0146, 0x0197, 0x01E9, 0x023A, 
                                  0x028B, 0x02DC, 0x032D, 0x037E, 0x03CF, 0x0420, 0x0470, 0x04C1, 
                                  0x0511, 0x0561, 0x05B1, 0x0601, 0x0650, 0x06A0, 0x06EF, 0x073E, 
                                  0x078D, 0x07DC, 0x082A, 0x0878, 0x08C6, 0x0914, 0x0961, 0x09AE, 
                                  0x09FB, 0x0A47, 0x0A94, 0x0AE0, 0x0B2B, 0x0B77, 0x0BC2, 0x0C0C, 
                                  0x0C57, 0x0CA1, 0x0CEB, 0x0D34, 0x0D7D, 0x0DC6, 0x0E0E, 0x0E56, 
                                  0x0E9E, 0x0EE5, 0x0F2C, 0x0F72, 0x0FB8, 0x0FFE, 0x1044, 0x1088, 
                                  0x10CD, 0x1111, 0x1155, 0x1198, 0x11DB, 0x121E, 0x1260, 0x12A2, 
                                  0x12E3, 0x1324, 0x1365, 0x13A5, 0x13E5, 0x1424, 0x1463, 0x14A2, 
                                  0x14E0, 0x151D, 0x155B, 0x1598, 0x15D4, 0x1610, 0x164C, 0x1687, 
                                  0x16C2, 0x16FC, 0x1736, 0x1770, 0x17A9, 0x17E2, 0x181A, 0x1852, 
                                  0x1889, 0x18C0, 0x18F7, 0x192D, 0x1963, 0x1999, 0x19CE, 0x1A03, 
                                  0x1A37, 0x1A6B, 0x1A9F, 0x1AD2, 0x1B05, 0x1B37, 0x1B69, 0x1B9B, 
                                  0x1BCC, 0x1BFD, 0x1C2D, 0x1C5E, 0x1C8D, 0x1CBD, 0x1CEC, 0x1D1A, 
                                  0x1D49, 0x1D77, 0x1DA4, 0x1DD2, 0x1DFF, 0x1E2B, 0x1E57, 0x1E83, 
                                  0x1EAF, 0x1EDA, 0x1F05, 0x1F2F, 0x1F59, 0x1F83, 0x1FAD, 0x1FD6, 
                                  0x1FFF
                               };
#if 0
// VC++
UWORD const wTabArcTan512[]  = {  0x0000, 0x0014, 0x0029, 0x003D, 0x0051, 0x0066, 0x007A, 0x008F, 
                                  0x00A3, 0x00B7, 0x00CC, 0x00E0, 0x00F4, 0x0109, 0x011D, 0x0131, 
                                  0x0146, 0x015A, 0x016E, 0x0183, 0x0197, 0x01AC, 0x01C0, 0x01D4, 
                                  0x01E9, 0x01FD, 0x0211, 0x0225, 0x023A, 0x024E, 0x0262, 0x0277, 
                                  0x028B, 0x029F, 0x02B4, 0x02C8, 0x02DC, 0x02F0, 0x0305, 0x0319, 
                                  0x032D, 0x0341, 0x0356, 0x036A, 0x037E, 0x0392, 0x03A6, 0x03BB, 
                                  0x03CF, 0x03E3, 0x03F7, 0x040B, 0x0420, 0x0434, 0x0448, 0x045C, 
                                  0x0470, 0x0484, 0x0498, 0x04AD, 0x04C1, 0x04D5, 0x04E9, 0x04FD, 
                                  0x0511, 0x0525, 0x0539, 0x054D, 0x0561, 0x0575, 0x0589, 0x059D, 
                                  0x05B1, 0x05C5, 0x05D9, 0x05ED, 0x0601, 0x0615, 0x0629, 0x063D, 
                                  0x0650, 0x0664, 0x0678, 0x068C, 0x06A0, 0x06B4, 0x06C8, 0x06DB, 
                                  0x06EF, 0x0703, 0x0717, 0x072A, 0x073E, 0x0752, 0x0766, 0x0779, 
                                  0x078D, 0x07A1, 0x07B4, 0x07C8, 0x07DC, 0x07EF, 0x0803, 0x0816, 
                                  0x082A, 0x083E, 0x0851, 0x0865, 0x0878, 0x088C, 0x089F, 0x08B3, 
                                  0x08C6, 0x08D9, 0x08ED, 0x0900, 0x0914, 0x0927, 0x093A, 0x094E, 
                                  0x0961, 0x0974, 0x0988, 0x099B, 0x09AE, 0x09C1, 0x09D5, 0x09E8, 
                                  0x09FB, 0x0A0E, 0x0A21, 0x0A34, 0x0A47, 0x0A5B, 0x0A6E, 0x0A81, 
                                  0x0A94, 0x0AA7, 0x0ABA, 0x0ACD, 0x0AE0, 0x0AF3, 0x0B06, 0x0B18, 
                                  0x0B2B, 0x0B3E, 0x0B51, 0x0B64, 0x0B77, 0x0B89, 0x0B9C, 0x0BAF, 
                                  0x0BC2, 0x0BD4, 0x0BE7, 0x0BFA, 0x0C0C, 0x0C1F, 0x0C32, 0x0C44, 
                                  0x0C57, 0x0C69, 0x0C7C, 0x0C8E, 0x0CA1, 0x0CB3, 0x0CC6, 0x0CD8, 
                                  0x0CEB, 0x0CFD, 0x0D0F, 0x0D22, 0x0D34, 0x0D46, 0x0D59, 0x0D6B, 
                                  0x0D7D, 0x0D8F, 0x0DA1, 0x0DB4, 0x0DC6, 0x0DD8, 0x0DEA, 0x0DFC, 
                                  0x0E0E, 0x0E20, 0x0E32, 0x0E44, 0x0E56, 0x0E68, 0x0E7A, 0x0E8C, 
                                  0x0E9E, 0x0EB0, 0x0EC1, 0x0ED3, 0x0EE5, 0x0EF7, 0x0F08, 0x0F1A, 
                                  0x0F2C, 0x0F3D, 0x0F4F, 0x0F61, 0x0F72, 0x0F84, 0x0F95, 0x0FA7, 
                                  0x0FB8, 0x0FCA, 0x0FDB, 0x0FED, 0x0FFE, 0x1010, 0x1021, 0x1032, 
                                  0x1044, 0x1055, 0x1066, 0x1077, 0x1088, 0x109A, 0x10AB, 0x10BC, 
                                  0x10CD, 0x10DE, 0x10EF, 0x1100, 0x1111, 0x1122, 0x1133, 0x1144, 
                                  0x1155, 0x1166, 0x1177, 0x1188, 0x1198, 0x11A9, 0x11BA, 0x11CB, 
                                  0x11DB, 0x11EC, 0x11FD, 0x120D, 0x121E, 0x122F, 0x123F, 0x1250, 
                                  0x1260, 0x1271, 0x1281, 0x1292, 0x12A2, 0x12B2, 0x12C3, 0x12D3, 
                                  0x12E3, 0x12F4, 0x1304, 0x1314, 0x1324, 0x1335, 0x1345, 0x1355, 
                                  0x1365, 0x1375, 0x1385, 0x1395, 0x13A5, 0x13B5, 0x13C5, 0x13D5, 
                                  0x13E5, 0x13F5, 0x1405, 0x1414, 0x1424, 0x1434, 0x1444, 0x1453, 
                                  0x1463, 0x1473, 0x1482, 0x1492, 0x14A2, 0x14B1, 0x14C1, 0x14D0, 
                                  0x14E0, 0x14EF, 0x14FF, 0x150E, 0x151D, 0x152D, 0x153C, 0x154B, 
                                  0x155B, 0x156A, 0x1579, 0x1588, 0x1598, 0x15A7, 0x15B6, 0x15C5, 
                                  0x15D4, 0x15E3, 0x15F2, 0x1601, 0x1610, 0x161F, 0x162E, 0x163D, 
                                  0x164C, 0x165B, 0x1669, 0x1678, 0x1687, 0x1696, 0x16A4, 0x16B3, 
                                  0x16C2, 0x16D0, 0x16DF, 0x16EE, 0x16FC, 0x170B, 0x1719, 0x1728, 
                                  0x1736, 0x1744, 0x1753, 0x1761, 0x1770, 0x177E, 0x178C, 0x179B, 
                                  0x17A9, 0x17B7, 0x17C5, 0x17D3, 0x17E2, 0x17F0, 0x17FE, 0x180C, 
                                  0x181A, 0x1828, 0x1836, 0x1844, 0x1852, 0x1860, 0x186E, 0x187B, 
                                  0x1889, 0x1897, 0x18A5, 0x18B3, 0x18C0, 0x18CE, 0x18DC, 0x18EA, 
                                  0x18F7, 0x1905, 0x1912, 0x1920, 0x192D, 0x193B, 0x1948, 0x1956, 
                                  0x1963, 0x1971, 0x197E, 0x198C, 0x1999, 0x19A6, 0x19B4, 0x19C1, 
                                  0x19CE, 0x19DB, 0x19E8, 0x19F6, 0x1A03, 0x1A10, 0x1A1D, 0x1A2A, 
                                  0x1A37, 0x1A44, 0x1A51, 0x1A5E, 0x1A6B, 0x1A78, 0x1A85, 0x1A92, 
                                  0x1A9F, 0x1AAB, 0x1AB8, 0x1AC5, 0x1AD2, 0x1ADF, 0x1AEB, 0x1AF8, 
                                  0x1B05, 0x1B11, 0x1B1E, 0x1B2A, 0x1B37, 0x1B44, 0x1B50, 0x1B5D, 
                                  0x1B69, 0x1B75, 0x1B82, 0x1B8E, 0x1B9B, 0x1BA7, 0x1BB3, 0x1BC0, 
                                  0x1BCC, 0x1BD8, 0x1BE4, 0x1BF1, 0x1BFD, 0x1C09, 0x1C15, 0x1C21, 
                                  0x1C2D, 0x1C39, 0x1C45, 0x1C52, 0x1C5E, 0x1C69, 0x1C75, 0x1C81, 
                                  0x1C8D, 0x1C99, 0x1CA5, 0x1CB1, 0x1CBD, 0x1CC8, 0x1CD4, 0x1CE0, 
                                  0x1CEC, 0x1CF7, 0x1D03, 0x1D0F, 0x1D1A, 0x1D26, 0x1D32, 0x1D3D, 
                                  0x1D49, 0x1D54, 0x1D60, 0x1D6B, 0x1D77, 0x1D82, 0x1D8E, 0x1D99, 
                                  0x1DA4, 0x1DB0, 0x1DBB, 0x1DC6, 0x1DD2, 0x1DDD, 0x1DE8, 0x1DF3, 
                                  0x1DFF, 0x1E0A, 0x1E15, 0x1E20, 0x1E2B, 0x1E36, 0x1E41, 0x1E4C, 
                                  0x1E57, 0x1E62, 0x1E6D, 0x1E78, 0x1E83, 0x1E8E, 0x1E99, 0x1EA4, 
                                  0x1EAF, 0x1EBA, 0x1EC4, 0x1ECF, 0x1EDA, 0x1EE5, 0x1EEF, 0x1EFA, 
                                  0x1F05, 0x1F0F, 0x1F1A, 0x1F25, 0x1F2F, 0x1F3A, 0x1F44, 0x1F4F, 
                                  0x1F59, 0x1F64, 0x1F6E, 0x1F79, 0x1F83, 0x1F8E, 0x1F98, 0x1FA3, 
                                  0x1FAD, 0x1FB7, 0x1FC2, 0x1FCC, 0x1FD6, 0x1FE0, 0x1FEB, 0x1FF5, 
                                  0x1FFF
                               };
#endif

/////////////////////////////////////////////////////////////////////////////
// Sine Tables

// VC++
UWORD const wTabSine128[]  = {  0x0000, 0x0192, 0x0324, 0x04B6, 0x0648, 0x07D9, 0x096A, 0x0AFB,
                                0x0C8C, 0x0E1C, 0x0FAB, 0x113A, 0x12C8, 0x1455, 0x15E2, 0x176E,
                                0x18F9, 0x1A82, 0x1C0B, 0x1D93, 0x1F1A, 0x209F, 0x2223, 0x23A6,
                                0x2528, 0x26A8, 0x2826, 0x29A3, 0x2B1F, 0x2C99, 0x2E11, 0x2F87,
                                0x30FB, 0x326E, 0x33DF, 0x354D, 0x36BA, 0x3824, 0x398C, 0x3AF2,
                                0x3C56, 0x3DB8, 0x3F17, 0x4073, 0x41CE, 0x4325, 0x447A, 0x45CD,
                                0x471C, 0x4869, 0x49B4, 0x4AFB, 0x4C3F, 0x4D81, 0x4EBF, 0x4FFB,
                                0x5133, 0x5268, 0x539B, 0x54C9, 0x55F5, 0x571D, 0x5842, 0x5964,
                                0x5A82, 0x5B9C, 0x5CB3, 0x5DC7, 0x5ED7, 0x5FE3, 0x60EB, 0x61F0,
                                0x62F1, 0x63EE, 0x64E8, 0x65DD, 0x66CF, 0x67BC, 0x68A6, 0x698B,
                                0x6A6D, 0x6B4A, 0x6C23, 0x6CF8, 0x6DC9, 0x6E96, 0x6F5E, 0x7022,
                                0x70E2, 0x719D, 0x7254, 0x7307, 0x73B5, 0x745F, 0x7504, 0x75A5,
                                0x7641, 0x76D8, 0x776B, 0x77FA, 0x7884, 0x7909, 0x7989, 0x7A05,
                                0x7A7C, 0x7AEE, 0x7B5C, 0x7BC5, 0x7C29, 0x7C88, 0x7CE3, 0x7D39,
                                0x7D89, 0x7DD5, 0x7E1D, 0x7E5F, 0x7E9C, 0x7ED5, 0x7F09, 0x7F37,
                                0x7F61, 0x7F86, 0x7FA6, 0x7FC1, 0x7FD8, 0x7FE9, 0x7FF5, 0x7FFD,
                                0x7FFF
                             };


#if 0
// VC++
UWORD const wTabSine128B[]  = { 0x0000, 0x0192, 0x0324, 0x04B6, 0x0648, 0x07D9, 0x096A, 0x0AFB,
                                0x0C8C, 0x0E1C, 0x0FAB, 0x113A, 0x12C8, 0x1455, 0x15E2, 0x176E,
                                0x18F9, 0x1A82, 0x1C0B, 0x1D93, 0x1F1A, 0x209F, 0x2223, 0x23A6,
                                0x2528, 0x26A8, 0x2826, 0x29A3, 0x2B1F, 0x2C99, 0x2E11, 0x2F87,
                                0x30FB, 0x326E, 0x33DF, 0x354D, 0x36BA, 0x3824, 0x398C, 0x3AF2,
                                0x3C56, 0x3DB8, 0x3F17, 0x4073, 0x41CE, 0x4325, 0x447A, 0x45CD,
                                0x471C, 0x4869, 0x49B4, 0x4AFB, 0x4C3F, 0x4D81, 0x4EBF, 0x4FFB,
                                0x5133, 0x5268, 0x539B, 0x54C9, 0x55F5, 0x571D, 0x5842, 0x5964,
                                0x5A82, 0x5B9C, 0x5CB3, 0x5DC7, 0x5ED7, 0x5FE3, 0x60EB, 0x61F0,
                                0x62F1, 0x63EE, 0x64E8, 0x65DD, 0x66CF, 0x67BC, 0x68A6, 0x698B,
                                0x6A6D, 0x6B4A, 0x6C23, 0x6CF8, 0x6DC9, 0x6E96, 0x6F5E, 0x7022,
                                0x70E2, 0x719D, 0x7254, 0x7307, 0x73B5, 0x745F, 0x7504, 0x75A5,
                                0x7641, 0x76D8, 0x776B, 0x77FA, 0x7884, 0x7909, 0x7989, 0x7A05,
                                0x7A7C, 0x7AEE, 0x7B5C, 0x7BC5, 0x7C29, 0x7C88, 0x7CE3, 0x7D39,
                                0x7D89, 0x7DD5, 0x7E1D, 0x7E5F, 0x7E9C, 0x7ED5, 0x7F09, 0x7F37,
                                0x7F61, 0x7F86, 0x7FA6, 0x7FC1, 0x7FD8, 0x7FE9, 0x7FF5, 0x7FFD,
                                0x7FFF,
                                0x7FFD, 0x7FF5, 0x7FE9, 0x7FD8, 0x7FC1, 0x7FA6, 0x7F86, 0x7F61,
                                0x7F37, 0x7F09, 0x7ED5, 0x7E9C, 0x7E5F, 0x7E1D, 0x7DD5, 0x7D89,
                                0x7D39, 0x7CE3, 0x7C88, 0x7C29, 0x7BC5, 0x7B5C, 0x7AEE, 0x7A7C,
                                0x7A05, 0x7989, 0x7909, 0x7884, 0x77FA, 0x776B, 0x76D8, 0x7641,
                                0x75A5, 0x7504, 0x745F, 0x73B5, 0x7307, 0x7254, 0x719D, 0x70E2,
                                0x7022, 0x6F5E, 0x6E96, 0x6DC9, 0x6CF8, 0x6C23, 0x6B4A, 0x6A6D,
                                0x698B, 0x68A6, 0x67BC, 0x66CF, 0x65DD, 0x64E8, 0x63EE, 0x62F1,
                                0x61F0, 0x60EB, 0x5FE3, 0x5ED7, 0x5DC7, 0x5CB3, 0x5B9C, 0x5A82,
                                0x5964, 0x5842, 0x571D, 0x55F5, 0x54C9, 0x539B, 0x5268, 0x5133,
                                0x4FFB, 0x4EBF, 0x4D81, 0x4C3F, 0x4AFB, 0x49B4, 0x4869, 0x471C,
                                0x45CD, 0x447A, 0x4325, 0x41CE, 0x4073, 0x3F17, 0x3DB8, 0x3C56,
                                0x3AF2, 0x398C, 0x3824, 0x36BA, 0x354D, 0x33DF, 0x326E, 0x30FB,
                                0x2F87, 0x2E11, 0x2C99, 0x2B1F, 0x29A3, 0x2826, 0x26A8, 0x2528,
                                0x23A6, 0x2223, 0x209F, 0x1F1A, 0x1D93, 0x1C0B, 0x1A82, 0x18F9,
                                0x176E, 0x15E2, 0x1455, 0x12C8, 0x113A, 0x0FAB, 0x0E1C, 0x0C8C,
                                0x0AFB, 0x096A, 0x07D9, 0x0648, 0x04B6, 0x0324, 0x0192, 0x0000
                             };


// VC++
UWORD const wTabSine256[]  = {  0x0000, 0x00C9, 0x0192, 0x025B, 0x0324, 0x03ED, 0x04B6, 0x057F,
                                0x0648, 0x0711, 0x07D9, 0x08A2, 0x096A, 0x0A33, 0x0AFB, 0x0BC4,
                                0x0C8C, 0x0D54, 0x0E1C, 0x0EE3, 0x0FAB, 0x1072, 0x113A, 0x1201,
                                0x12C8, 0x138F, 0x1455, 0x151C, 0x15E2, 0x16A8, 0x176E, 0x1833,
                                0x18F9, 0x19BE, 0x1A82, 0x1B47, 0x1C0B, 0x1CCF, 0x1D93, 0x1E57,
                                0x1F1A, 0x1FDD, 0x209F, 0x2161, 0x2223, 0x22E5, 0x23A6, 0x2467,
                                0x2528, 0x25E8, 0x26A8, 0x2767, 0x2826, 0x28E5, 0x29A3, 0x2A61,
                                0x2B1F, 0x2BDC, 0x2C99, 0x2D55, 0x2E11, 0x2ECC, 0x2F87, 0x3041,
                                0x30FB, 0x31B5, 0x326E, 0x3326, 0x33DF, 0x3496, 0x354D, 0x3604,
                                0x36BA, 0x376F, 0x3824, 0x38D9, 0x398C, 0x3A40, 0x3AF2, 0x3BA5,
                                0x3C56, 0x3D07, 0x3DB8, 0x3E68, 0x3F17, 0x3FC5, 0x4073, 0x4121,
                                0x41CE, 0x427A, 0x4325, 0x43D0, 0x447A, 0x4524, 0x45CD, 0x4675,
                                0x471C, 0x47C3, 0x4869, 0x490F, 0x49B4, 0x4A58, 0x4AFB, 0x4B9D,
                                0x4C3F, 0x4CE0, 0x4D81, 0x4E20, 0x4EBF, 0x4F5D, 0x4FFB, 0x5097,
                                0x5133, 0x51CE, 0x5268, 0x5302, 0x539B, 0x5432, 0x54C9, 0x5560,
                                0x55F5, 0x568A, 0x571D, 0x57B0, 0x5842, 0x58D3, 0x5964, 0x59F3,
                                0x5A82, 0x5B0F, 0x5B9C, 0x5C28, 0x5CB3, 0x5D3E, 0x5DC7, 0x5E4F,
                                0x5ED7, 0x5F5D, 0x5FE3, 0x6068, 0x60EB, 0x616E, 0x61F0, 0x6271,
                                0x62F1, 0x6370, 0x63EE, 0x646C, 0x64E8, 0x6563, 0x65DD, 0x6656,
                                0x66CF, 0x6746, 0x67BC, 0x6832, 0x68A6, 0x6919, 0x698B, 0x69FD,
                                0x6A6D, 0x6ADC, 0x6B4A, 0x6BB7, 0x6C23, 0x6C8E, 0x6CF8, 0x6D61,
                                0x6DC9, 0x6E30, 0x6E96, 0x6EFB, 0x6F5E, 0x6FC1, 0x7022, 0x7083,
                                0x70E2, 0x7140, 0x719D, 0x71F9, 0x7254, 0x72AE, 0x7307, 0x735E,
                                0x73B5, 0x740A, 0x745F, 0x74B2, 0x7504, 0x7555, 0x75A5, 0x75F3,
                                0x7641, 0x768D, 0x76D8, 0x7722, 0x776B, 0x77B3, 0x77FA, 0x783F,
                                0x7884, 0x78C7, 0x7909, 0x794A, 0x7989, 0x79C8, 0x7A05, 0x7A41,
                                0x7A7C, 0x7AB6, 0x7AEE, 0x7B26, 0x7B5C, 0x7B91, 0x7BC5, 0x7BF8,
                                0x7C29, 0x7C59, 0x7C88, 0x7CB6, 0x7CE3, 0x7D0E, 0x7D39, 0x7D62,
                                0x7D89, 0x7DB0, 0x7DD5, 0x7DFA, 0x7E1D, 0x7E3E, 0x7E5F, 0x7E7E,
                                0x7E9C, 0x7EB9, 0x7ED5, 0x7EEF, 0x7F09, 0x7F21, 0x7F37, 0x7F4D,
                                0x7F61, 0x7F74, 0x7F86, 0x7F97, 0x7FA6, 0x7FB4, 0x7FC1, 0x7FCD,
                                0x7FD8, 0x7FE1, 0x7FE9, 0x7FF0, 0x7FF5, 0x7FF9, 0x7FFD, 0x7FFE,
                                0x7FFF
                             };



// VC++
UWORD const wTabSineAxM[]  = {      0,   1607,   3211,   4808,   6392,   7961,   9512,  11039,
                                12539,  14010,  15446,  16846,  18204,  19519,  20787,  22005, 
                                23170,  24279,  25329,  26319,  27245,  28106,  28898,  29621, 
                                30273,  30852,  31357,  31785,  32138,  32413,  32610,  32728, 
                                32767,  
                                32728,  32610,  32413,  32138,  31785,  31357,  30852,  30273,
                                29621,  28898,  28106,  27245,  26319,  25329,  24279,  23170,
                                22005,  20787,  19519,  18204,  16846,  15446,  14010,  12539,
                                11039,   9512,   7961,   6392,   4808,   3211,   1607,      0
                             };


#endif
/////////////////////////////////////////////////////////////////////////////
// Square Root Table
#if 0
UWORD const uwSqrtTbl128[] = {  0x0000, 0x16A0, 0x1FFF, 0x2730, 0x2D41, 0x3298, 0x376C, 0x3BDD,
                                0x3FFF, 0x43E1, 0x478D, 0x4B0B, 0x4E61, 0x5195, 0x54A9, 0x57A2,
                                0x5A82, 0x5D4B, 0x5FFF, 0x62A1, 0x6530, 0x67B0, 0x6A21, 0x6C83,
                                0x6ED9, 0x7122, 0x7360, 0x7592, 0x77BB, 0x79D9, 0x7BEE, 0x7DFB,
                                0x7FFF, 0x81FB, 0x83EF, 0x85DD, 0x87C3, 0x89A2, 0x8B7B, 0x8D4E,
                                0x8F1B, 0x90E2, 0x92A3, 0x9460, 0x9617, 0x97C9, 0x9976, 0x9B1F,
                                0x9CC3, 0x9E63, 0x9FFF, 0xA196, 0xA32A, 0xA4BA, 0xA646, 0xA7CE,
                                0xA953, 0xAAD4, 0xAC52, 0xADCD, 0xAF44, 0xB0B9, 0xB22A, 0xB398,
                                0xB504, 0xB66C, 0xB7D2, 0xB935, 0xBA96, 0xBBF4, 0xBD4F, 0xBEA8,
                                0xBFFF, 0xC153, 0xC2A5, 0xC3F4, 0xC542, 0xC68D, 0xC7D6, 0xC91D,
                                0xCA61, 0xCBA4, 0xCCE5, 0xCE24, 0xCF61, 0xD09C, 0xD1D5, 0xD30D,
                                0xD442, 0xD576, 0xD6A8, 0xD7D9, 0xD907, 0xDA35, 0xDB60, 0xDC8A,
                                0xDDB2, 0xDED9, 0xDFFF, 0xE122, 0xE245, 0xE366, 0xE485, 0xE5A3,
                                0xE6C0, 0xE7DB, 0xE8F5, 0xEA0E, 0xEB25, 0xEC3B, 0xED50, 0xEE64,
                                0xEF76, 0xF087, 0xF197, 0xF2A5, 0xF3B3, 0xF4BF, 0xF5CA, 0xF6D5,
                                0xF7DD, 0xF8E5, 0xF9EC, 0xFAF2, 0xFBF6, 0xFCFA, 0xFDFC, 0xFEFE,
                                0xFFFF
                             } ;
#endif

//const UWORD  uwSqrtTbl1024[] =
//  {
//    #include "sqroot_1024.dat"
//  } ;

const UWORD  uwSqrtTbl4096[] =
  {
    #include "sqroot_4096.dat"
  } ;

//const UWORD  uwSqrtTbl8192[] =
//  {
//    #include "sqroot_8192.dat"
//  } ;

/////////////////////////////////////////////////////////////////////////////
//



UWORD ATan16( SWORD swSIN, SWORD swCOS )
{
  BOOL  bNegSIN, bNegCOS, bOctant;
  UWORD uwTangent, uwOffset, uwAlpha;
  
  // SIN absolute value saving sign
  if ( swSIN < 0 ) 
  {
    bNegSIN = 1; 
    swSIN   = -swSIN;
  } 
  else 
  {
    bNegSIN = 0;
  }

  // COS absolute value saving sign
  if ( swCOS < 0 ) 
  {
    bNegCOS = 1; 
    swCOS   = -swCOS;
  } 
  else 
  {
    bNegCOS = 0;
  }

  // SIN = COS : Octant = 1
  if ( swSIN == swCOS )
  {
      bOctant = 1;
      uwTangent = 0xFFFF;
  }
  // SIN > COS : Octant = 1
  else if ( swSIN > swCOS )
  {
    bOctant = 1;
    uwTangent = (UWORD)( ( (ULONG)swCOS << 16 ) / swSIN );
  // COS > SIN : Octant = 0
  } 
  else 
  {
    bOctant = 0;
    uwTangent = (UWORD)( ( (ULONG)swSIN << 16 ) / swCOS );
  }

  // Table lookup with interpolation
//  // uso tabella di 32 elementi
//  uwOffset = uwTangent >> 11;
//  uwAlpha = wTabArcTan32A[ uwOffset ] + (UWORD)( ( (ULONG)( wTabArcTan32A[ uwOffset + 1 ] - wTabArcTan32A[ uwOffset ] ) * ( uwTangent & 0x001F ) ) / 0x0020 ); // bacata!
//  uwAlpha = wTabArcTan32A[ uwOffset ] + (UWORD)( ( (ULONG)( wTabArcTan32A[ uwOffset + 1 ] - wTabArcTan32A[ uwOffset ] ) * ( uwTangent & 0x07FF ) ) / 0x0800 ); // corretta!

  // uso tabella da 128 elementi
  uwOffset = uwTangent >> 9;
  uwAlpha = wTabArcTan128[ uwOffset ] + (UWORD)( ( (ULONG)( wTabArcTan128[ uwOffset + 1 ] - wTabArcTan128[ uwOffset ] ) * ( uwTangent & 0x01FF ) ) / 0x0200 );

  // Adjust quadrants
  if ( bNegSIN ) 
  {
    if ( !bNegCOS ) 
    {
      if ( bOctant ) 
        uwAlpha = 0xC000 + uwAlpha;
      else
        uwAlpha = 0xFFFF - uwAlpha;
    } 
    else 
    {
      if ( bOctant ) 
        uwAlpha = 0xBFFF - uwAlpha;
      else
        uwAlpha = 0x8000 + uwAlpha;
    }
  } 
  else 
  {
    if ( !bNegCOS )
    {
      if ( bOctant ) 
        uwAlpha = 0x3FFF - uwAlpha;
      else
        uwAlpha = 0x0000 + uwAlpha;
    } 
    else
    {
      if ( bOctant ) 
        uwAlpha = 0x4000 + uwAlpha;
      else
        uwAlpha = 0x7fff - uwAlpha;
    }
  }
  return uwAlpha;
}


SWORD Sin16( UWORD uwAngle );

SWORD Cos16( UWORD uwAngle )
{
  return Sin16( 0x4000 - uwAngle );
}

SWORD Sin16( UWORD uwAngle )
{
  BOOL  bBit15, bBit14;
  UWORD uwOffset, uwSIN;

  // retrieve quadrant informations
  bBit15 = (uwAngle & 0x8000) == 0x8000;
  bBit14 = (uwAngle & 0x4000) == 0x4000;

  // 14 bits quadrant angle
  uwAngle &= 0x3FFF;

  // Adjust quadrant
  if ( bBit14 )
    uwAngle = 0x4000 - uwAngle;

  // Table lookup with interpolation
  uwOffset = uwAngle >> 7; // 0-127: since angle is 14 bit and NOT 16
  uwSIN = wTabSine128[ uwOffset ] + (UWORD)( ( (ULONG)( wTabSine128[ uwOffset + 1 ] - wTabSine128[ uwOffset ] ) *
                                                      ( uwAngle & 0x007F ) ) / 0x0080 );



  // Adjust semiplan
  if ( !bBit15 )
    return uwSIN;
  else
    return 0x0000 - uwSIN;

}


#if 0
/////////////////////////////////////////////////////////////////////////////
// nMode: 0 = Use 5 bits (32 elements) VECON/AxM Table
//        1 = Use 5 bits (32 elements) VC++ Table
//        2 = Use 7 bits (128 elements) VC++ Table
//        3 = Use 9 bits (512 elements) VC++ Table


UWORD xxxComputeAngle( SWORD swSIN, SWORD swCOS, SWORD swMode )
{
  bit  bNegSIN, bNegCOS, bOctant;
  UWORD uwTangent, uwOffset, uwAlpha;
  
//  DIAGNOSTIC_OUTPUT = 1;

  // SIN absolute value saving sign
  if ( swSIN < 0 ) {
    bNegSIN = 1; swSIN = -swSIN;
  } else {
    bNegSIN = 0;
  }

  // COS absolute value saving sign
  if ( swCOS < 0 ) {
    bNegCOS = 1; swCOS = -swCOS;
  } else {
    bNegCOS = 0;
  }

  // SIN > COS : Octant = 1
  if ( swSIN >= swCOS ) {
    bOctant = 1;
    uwTangent = (UWORD)( ( (ULONG)swCOS << 16 ) / swSIN );
  // COS > SIN : Octant = 0
  } else {
    bOctant = 0;
    uwTangent = (UWORD)( ( (ULONG)swSIN << 16 ) / swCOS );
  }

  // Table lookup with interpolation
  if ( swMode == 0 ) {
    UWORD uwW1, uwW2, uwW3, uwW4;
    
    uwOffset = uwTangent >> 11;


    uwW1 = wTabArcTan32A[ wOffset ];
    uwW2 = wTabArcTan32A[ wOffset + 1 ];
    uwW3 = uwW2 - uwW1;
    uwW4 = (UWORD)( ( (ULONG)uwW3 * ( uwTangent & 0x07FF ) ) / 0x0800 );

    uwAlpha = uwW1 + uwW4;

  } else if ( swMode == 1 ) {
    uwOffset = uwTangent >> 11;
    uwAlpha  = wTabArcTan32B[ uwOffset ] + (UWORD)( ( (ULONG)( wTabArcTan32B[ wOffset + 1 ] - wTabArcTan32B[ wOffset ] ) *
                                                   ( uwTangent & 0x07FF ) ) / 0x0800 );
  } else if ( swMode == 2 ) {
    uwOffset = uwTangent >> 9;
    uwAlpha  = wTabArcTan128[ uwOffset ] + ( ( wTabArcTan128[ uwOffset + 1 ] - wTabArcTan128[ uwOffset ] ) *
                                             ( uwTangent & 0x01FF ) ) / 0x0200;
  } else if ( nMode == 3 ) {
    uwOffset = uwTangent >> 7;
    uwAlpha  = wTabArcTan512[ uwOffset ] + ( ( wTabArcTan512[ uwOffset + 1 ] - wTabArcTan512[ uwOffset ] ) *
                                             ( uwTangent & 0x007F ) ) / 0x0080;
  } 

  // Adjust quadrants
  if ( !bNegSIN ) {
    if ( bNegCOS ) {
      if ( bOctant ) 
        uwAlpha = 0xC000 + uwAlpha;
      else
        uwAlpha = 0xFFFF - uwAlpha;
    } else {
      if ( bOctant ) 
        uwAlpha = 0xBFFF - uwAlpha;
      else
        uwAlpha = 0x8000 + uwAlpha;
    }
  } else {
    if ( bNegCOS ) {
      if ( bOctant ) 
        uwAlpha = 0x3FFF - uwAlpha;
      else
        uwAlpha = 0x0000 + uwAlpha;
    } else {
      if ( bOctant ) 
        uwAlpha = 0x4000 + uwAlpha;
      else
        uwAlpha = 0x7fff - uwAlpha;
    }
  }

//  DIAGNOSTIC_OUTPUT = 0;

  return uwAlpha;
}


/////////////////////////////////////////////////////////////////////////////
// nMode: 0 = 
//        1 = 


SWORD xxxComputeSIN( UWORD uwAngle, SWORD swMode )
{
  bit  bBit15, bBit14;
  UWORD uwOffset, uwSIN;

  if ( swMode == 0 ) 
  {
    UWORD uwIndice, uwIndex; 
    SWORD  swX, swY, swZ;
    
    /* Considera l'angolo da 0 a 127 */    
    uwIndice = ( uwAngle & 0xfe00 ) >> 9;
    
    /* Divide l'angolo in 64 settori */ 
    uwIndex = uwIndice & 0x003F;
    
    swX = wTabSineAxM[ uwIndex + 1 ];
    
    if ( ( uwIndice + 1 ) & 0x0040 ) 
    swX = -swX;
    swZ = wTabSineAxM[ uwIndex ];
    if ( ( uwIndice ) & 0x0040 )
    swZ = -swZ;
    swY = (SWORD)( (SLONG)( swX - swZ ) * (SLONG)( uwAngle & 0x01ff ) >> 9 );
    swY += swZ;
    return y;

  } 
  else if ( swMode == 1 ) 
  {
    // retrieve quadrant informations
    bBit15 = uwAngle & 0x8000;
    bBit14 = uwAngle & 0x4000;
  
    // 14 bits quadrant angle
    uwAngle &= 0x3FFF;
  
    // Adjust quadrant
    if ( bBit14 )
      uwAngle = 0x4000 - uwAngle;

    // Table lookup with interpolation
    uwOffset = uwAngle >> 7; // 0-127
    uwSIN = wTabSine128[ uwOffset ] + ( ( wTabSine128[ uwOffset + 1 ] - wTabSine128[ uwOffset ] ) *
                                        ( uwAngle & 0x007F ) ) / 0x0080;
  }

  // Adjust semiplan
  if ( !bBit15 )
    return uwSIN;
  else
    return 0x0000 - uwSIN;

}


SWORD xxComputeSIN( UWORD uwAngle, SWORD swMode )
{
  bit  bBit15, bBit14;
  UWORD uwOffset, uwSIN;
  SWORD swSIN;

  if ( swMode == 0 ) {

    // retrieve quadrant informations
    bBit15 = uwAngle & 0x8000;
    bBit14 = uwAngle & 0x4000;
  
    // 14 bits quadrant angle
    uwAngle &= 0x3FFF;
  
    if ( bBit14 )
      uwAngle = 0x3fff - uwAngle;

    // Table lookup with interpolation
    uwOffset = uwAngle >> 7; // 0-127
    uwSIN = wTabSine128[ uwOffset ] + ( ( wTabSine128[ uwOffset + 1 ] - wTabSine128[ uwOffset ] ) *
                                        ( uwAngle & 0x007F ) ) / 0x0080;

  } else if ( swMode == 1 ) {

    // retrieve quadrant informations
    bBit15 = uwAngle & 0x8000;

    // 15 bits quadrant angle
    uwAngle &= 0x7FFF;

    uwOffset = uwAngle >> 7;
    uwSIN = wTabSine128B[ uwOffset ] + ( ( wTabSine128B[ uwOffset + 1 ] - wTabSine128B[ uwOffset ] ) *
                                         ( wAngle & 0x00FF ) ) / 0x0100;

  }


  // Adjust quadrants
  if ( !bBit15 ) {
    if ( !bBit14 )
      swSIN = 0x0000 + uwSIN;
    else
      swSIN = 0x0000 + uwSIN;
  } else {
    if ( !bBit14 )
      swSIN = 0x0000 + uwSIN;
    else
      swSIN = 0x0000 + uwSIN;
  }

  return uwSIN;
}
#endif


/* /////////////////////////////////////////////////////////////////////// */
/* ******************************************************************* * 
 * Locate the position of the highest bit set.                         * 
 * A binary search is used.  The result is an approximation of log2(n) *
 * [the integer part]. Source from:                                    *
 * http://groups.google.com/group/comp.lang.c/msg/52820a5d19679089     *
 * ******************************************************************* */
SWORD _ilog2(ULONG ulArgument)
{
  SWORD swHighestBitPosition = (-1);

  /* Is there a bit on in the high word? */
  /* Else, all the high bits are already zero. */
  if (ulArgument & 0xffff0000) 
  {
    swHighestBitPosition += 16; /* Update our search position */
    ulArgument >>= 16;          /* Shift out lower (irrelevant) bits */
  }
  /* Is there a bit on in the high byte of the current word? */
  /* Else, all the high bits are already zero. */
  if (ulArgument & 0xff00) 
  {
    swHighestBitPosition += 8; /* Update our search position */
    ulArgument >>= 8;          /* Shift out lower (irrelevant) bits */
  }
  /* Is there a bit on in the current nybble? */
  /* Else, all the high bits are already zero. */
  if (ulArgument & 0xf0) 
  {
    swHighestBitPosition += 4; /* Update our search position */
    ulArgument >>= 4;          /* Shift out lower (irrelevant) bits */
  }

  /* Is there a bit on in the high 2 bits of the current nybble? */
  /* 0xc is 1100 in binary... */
  /* Else, all the high bits are already zero. */
  if (ulArgument & 0xc) 
  {
    swHighestBitPosition += 2; /* Update our search position */
    ulArgument >>= 2;          /* Shift out lower (irrelevant) bits */
  }
  /* Is the 2nd bit on? [ 0x2 is 0010 in binary...] */
  /* Else, all the 2nd bit is already zero. */
  if (ulArgument & 0x2) 
  {
    swHighestBitPosition++;    /* Update our search position */
    ulArgument >>= 1;          /* Shift out lower (irrelevant) bit */
  }
  /* Is the lowest bit set? */
  if (ulArgument)
    swHighestBitPosition++;    /* Update our search position */

//original  return swHighestBitPosition ;        // approssimato per difetto
//crs       return (swHighestBitPosition + 1) ;  // approssimato per eccesso
  return (swHighestBitPosition + 1) ;  // approssimato per eccesso
} 
/* /////////////////////////////////////////////////////////////////////// */
///* Square Root with look-up table */
//UWORD _SquareRoot16_LookUpTable(UWORD uwValue)
//{
//  UWORD uwSqrt = 0, uwIdx = 0 ;
//  /* Table lookup with interpolation */
////  /* 128 (= 2^7) elements table: 2^16 - 2^7 = 2^9 = 0x200 */
////  uwIdx  = uwValue >> 9 ;
////  uwSqrt = uwSqrtTbl128[uwIdx] + (UWORD)((((ULONG)uwSqrtTbl128[uwIdx + 1] - uwSqrtTbl128[uwIdx]) * (uwValue & 0x01FF))/0x0200) ;
//
////  /* 4096 (= 2^12) elements table: 2^16 - 2^12 = 2^4 = 0x10 */
////  uwIdx  = uwValue >> 4 ;  
////  uwSqrt = uwSqrtTbl4096[uwIdx] + (UWORD)((((ULONG)uwSqrtTbl4096[uwIdx + 1] - uwSqrtTbl4096[uwIdx]) * (uwValue & 0x000F))/0x0010) ;  
//
////  /* 8192 (= 2^13) elements table: 2^16 - 2^13 = 2^3 = 0x8 */
////  uwIdx  = uwValue >> 3 ;  
////  uwSqrt = uwSqrtTbl8192[uwIdx] + (UWORD)((((ULONG)uwSqrtTbl8192[uwIdx + 1] - uwSqrtTbl8192[uwIdx]) * (uwValue & 0x0007))/0x0008) ;  
//
//  return uwSqrt ;
//}

/* /////////////////////////////////////////////////////////////////////// */
/* Square Root with 4096 (= 2^12) elements table */
UWORD _SquareRoot16_4096(ULONG ulValue)
{ 
  UWORD uwValueHi, uwValueLo, uwIdx, uwSqrt ;
  ULONG ulValueFull ;

  ulValueFull = ulValue << 2 ; /* 0x3fffffff << 2 = 0xfffffffc */
  uwValueHi = HIWORD(ulValueFull) ;
  uwValueLo = LOWORD(ulValueFull) ;

  if (uwValueHi & 0xf000)
  { /* >> 4 */
    uwIdx  = HIWORD(ulValueFull >> 4) ;
    uwSqrt = (uwSqrtTbl4096[uwIdx]) ;
  }
  else if (uwValueHi & 0x0f00)
  { /* >> 0 */
    uwIdx  = HIWORD(ulValueFull)  ;
    uwSqrt = (uwSqrtTbl4096[uwIdx]) >> 2 ;
  }
  else if (uwValueHi & 0x00f0)
  { /* << 4 */
    uwIdx  = HIWORD(ulValueFull << 4)   ;
    uwSqrt = (uwSqrtTbl4096[uwIdx]) >> 4 ;
  }
  else if (uwValueHi & 0x000f)
  { /* << 8 */
    uwIdx  = HIWORD(ulValueFull << 8)    ;
    uwSqrt = (uwSqrtTbl4096[uwIdx]) >> 6 ;
  }
  else if (uwValueLo & 0xf000)
  { /* << 12 */
    uwIdx  = HIWORD(ulValueFull << 12)   ;
    uwSqrt = (uwSqrtTbl4096[uwIdx]) >> 8 ;
  }
  else
    uwSqrt = 0 ;

  return uwSqrt ;
}


/* ######################################################################### */
UWORD CalculateSquareRoot(SWORD swSin, SWORD swCos)
{
  ULONG ulSin2PlusCos2 ;
  UWORD uwSQRoot ;

  ulSin2PlusCos2 = (ULONG)((SLONG)swSin * swSin + (SLONG)swCos * swCos) ;

  /* since sin^2 and cos^2 are 30bit, sin^2+cos^2 could only maximum be 30bit */
  if (ulSin2PlusCos2 > 0x3fffffff) 
    ulSin2PlusCos2 = 0x3fffffff ; 

  uwSQRoot = _SquareRoot16_4096(ulSin2PlusCos2) ;
     
  return uwSQRoot ;
}


/* ######################################################################### */
// result = sqrt(X^2+Y^2) 
//  Value_RMS = result / 1.646760258121 = result * 0.60725293500890546440666072887872
SWORD RootOfSquareSum16(SWORD swX, SWORD swY)
{
  UWORD uwCount ;
  SWORD swDx = 0 ;

  for (uwCount = 0U; uwCount < 12; uwCount++)
  {
    swDx = swX >> uwCount;
    if ( swY > 0 )
    {
      swX += swY >> uwCount;
      swY -= swDx;
    }
    else
    {
      swX -= swY >> uwCount;
      swY += swDx;
    }
  }
  return swX ; 
}
