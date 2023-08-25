/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CommonUtility.c                                            */
/* Author      : Hu Xiaokai                                                 */
/*                                                                          */
/*                                                                          */
/* Description : Common utilities                                           */
/*                                                                          */
/****************************************************************************/
/////////////////////////////////////////////////////////////////////////////
// Compiler Option
#pragma GCC optimize (2)

/////////////////////////////////////////////////////////////////////////////
//

//#include "common\CommonDefines.h"
#include "common\CommonUtility.h"
#include <string.h> // to use memcpy

/////////////////////////////////////////////////////////////////////////////
//
// CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1)

static const UWORD crc16_table[ 256 ] = {
      0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
      0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
      0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
      0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
      0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
      0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
      0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
      0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
      0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
      0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
      0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
      0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
      0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
      0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
      0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
      0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
      0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
      0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
      0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
      0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
      0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
      0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
      0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
      0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
      0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
      0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
      0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
      0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
      0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
      0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
      0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
      0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/////////////////////////////////////////////////////////////////////////////
//

static __inline UWORD crc16_byte( UWORD uwCRC, const UBYTE ubData )
{
  return ( uwCRC >> 8 ) ^ crc16_table[ ( uwCRC ^ ubData ) & 0xff ];
}


/////////////////////////////////////////////////////////////////////////////
//
// Compute the CRC-16 for the data buffer
//    uwCRC       previous CRC value
//    hpbyBuffer  data pointer
//    uwLength    number of bytes in the buffer
//    return      the updated CRC value

UWORD crc16( UWORD uwCRC, const UBYTE * hpbyBuffer, UWORD uwLength )
{
  while ( uwLength-- )
    uwCRC = crc16_byte( uwCRC, *hpbyBuffer++ );
  return uwCRC;
}

/////////////////////////////////////////////////////////////////////////////
// Atomic read up to 8 byte

void atomic_read(HPVOID dst, const HPVOID src, const UWORD count)
{
#ifdef _INFINEON_
    __asm
    {
        exts    r11,#4
        movb    rl1,[r10+]
        movb    rh1,[r10+]
        movb    rl2,[r10+]
        exts    r11,#4
        movb    rh2,[r10+]
        movb    rl3,[r10+]
        movb    rh3,[r10+]
        exts    r11,#2
        movb    rl4,[r10+]
        movb    rh4,[r10+]

        add     r8,r12
        sub     r12, #1

        cmpd1   r12,#0
        jmpr    cc_Z,copy1
        cmpd1   r12,#0
        jmpr    cc_Z,copy2
        cmpd1   r12,#0
        jmpr    cc_Z,copy3
        cmpd1   r12,#0
        jmpr    cc_Z,copy4
        cmpd1   r12,#0
        jmpr    cc_Z,copy5
        cmpd1   r12,#0
        jmpr    cc_Z,copy6
        cmpd1   r12,#0
        jmpr    cc_Z,copy7

        exts    r9,#1
        movb    [-r8],rh4
copy7:
        exts    r9,#1
        movb    [-r8],rl4
copy6:
        exts    r9,#1
        movb    [-r8],rh3
copy5:
        exts    r9,#1
        movb    [-r8],rl3
copy4:
        exts    r9,#1
        movb    [-r8],rh2
copy3:
        exts    r9,#1
        movb    [-r8],rl2
copy2:
        exts    r9,#1
        movb    [-r8],rh1
copy1:
        exts    r9,#1
        movb    [-r8],rl1
    }
#else
    memcpy(dst, src, count);
#endif
}

/////////////////////////////////////////////////////////////////////////////
// Atomic write up to 8 byte

void atomic_write(HPVOID dst, const HPVOID src, const UWORD count)
{
#ifdef _INFINEON_
    __asm
    {
        exts    r11,#4
        movb    rl1,[r10+]
        movb    rh1,[r10+]
        movb    rl2,[r10+]
        movb    rh2,[r10+]
        exts    r11,#4
        movb    rl3,[r10+]
        movb    rh3,[r10+]
        movb    rl4,[r10+]
        movb    rh4,[r10+]

        add     r8,r12
        sub     r12, #1

        cmpd1   r12,#0
        jmpr    cc_Z,copy1
        cmpd1   r12,#0
        jmpr    cc_Z,copy2
        cmpd1   r12,#0
        jmpr    cc_Z,copy3
        cmpd1   r12,#0
        jmpr    cc_Z,copy4
        cmpd1   r12,#0
        jmpr    cc_Z,copy5
        cmpd1   r12,#0
        jmpr    cc_Z,copy6
        cmpd1   r12,#0
        jmpr    cc_Z,copy7

        exts    r9,#2
        movb    [-r8],rh4
copy7:
        exts    r9,#2
        movb    [-r8],rl4
copy6:
        exts    r9,#2
        movb    [-r8],rh3
copy5:
        exts    r9,#2
        movb    [-r8],rl3
copy4:
        exts    r9,#2
        movb    [-r8],rh2
copy3:
        exts    r9,#2
        movb    [-r8],rl2
copy2:
        exts    r9,#2
        movb    [-r8],rh1
copy1:
        exts    r9,#1
        movb    [-r8],rl1
    }
#else
    memcpy(dst, src, count);
#endif
}

/////////////////////////////////////////////////////////////////////////////
// Atomic move up to 8 byte

void atomic_move(HPVOID dst, const HPVOID src, const UWORD count)
{
//    __asm
//    {
//        exts    r11,#4
//        movb    rl1,[r10+]
//        movb    rh1,[r10+]
//        movb    rl2,[r10+]
//        exts    r11,#4
//        movb    rh2,[r10+]
//        movb    rl3,[r10+]
//        movb    rh3,[r10+]
//        exts    r11,#2
//        movb    rl4,[r10+]
//        movb    rh4,[r10+]
//
//        add     r8,r12
//        sub     r12, #1
//
//        cmpd1   r12,#0
//        jmpr    cc_Z,copy1
//        cmpd1   r12,#0
//        jmpr    cc_Z,copy2
//        cmpd1   r12,#0
//        jmpr    cc_Z,copy3
//        cmpd1   r12,#0
//        jmpr    cc_Z,copy4
//        cmpd1   r12,#0
//        jmpr    cc_Z,copy5
//        cmpd1   r12,#0
//        jmpr    cc_Z,copy6
//        cmpd1   r12,#0
//        jmpr    cc_Z,copy7
//
//        exts    r9,#2
//        movb    [-r8],rh4
//copy7:
//        exts    r9,#2
//        movb    [-r8],rl4
//copy6:
//        exts    r9,#2
//        movb    [-r8],rh3
//copy5:
//        exts    r9,#2
//        movb    [-r8],rl3
//copy4:
//        exts    r9,#2
//        movb    [-r8],rh2
//copy3:
//        exts    r9,#2
//        movb    [-r8],rl2
//copy2:
//        exts    r9,#2
//        movb    [-r8],rh1
//copy1:
//        exts    r9,#1
//        movb    [-r8],rl1
//    }
	memcpy(dst, src, count);
}

/////////////////////////////////////////////////////////////////////////////
// Atomic read chunk of 16 bytes

void atomic_read_chunk16(HPVOID dst, const HPVOID src)
{
//    __asm
//    {
//        exts    r11,#4
//        mov     r1,[r10+]
//        mov     r2,[r10+]
//        mov     r3,[r10+]
//        exts    r11,#4
//        mov     r4,[r10+]
//        mov     r5,[r10+]
//        mov     r6,[r10+]
//        exts    r11,#2
//        mov     r7,[r10+]
//        mov     r12,[r10+]
//
//        add     r8,#0x10
//
//        exts    r9,#2
//        mov     [-r8],r12
//        mov     [-r8],r7
//        exts    r9,#2
//        mov     [-r8],r6
//        mov     [-r8],r5
//        exts    r9,#2
//        mov     [-r8],r4
//        mov     [-r8],r3
//        exts    r9,#2
//        mov     [-r8],r2
//        mov     [-r8],r1
//    }
    memcpy(dst,src,16);
}

/////////////////////////////////////////////////////////////////////////////
// Atomic byte write bits

UWORD atomic_byte_write_bits(HPUBYTE dst, UBYTE data, UBYTE mask)
{
//    __asm
//    {
//        and     r10,r11
//        mov     r2,r11
//        xor     r2,#0xffff
//
//        exts    r9,#4
//        movb    rl4,[r8]
//        and     r4,r2
//        or      r4,r10
//        movb    [r8],rl4
//    }
}

/////////////////////////////////////////////////////////////////////////////
// Atomic write bits

UWORD atomic_write_bits(HPUWORD dst, UWORD data, UWORD mask)
{
//    __asm
//    {
//        and     r10,r11
//        mov     r2,r11
//        xor     r2,#0xffff
//
//        exts    r9,#4
//        mov     r4,[r8]
//        and     r4,r2
//        or      r4,r10
//        mov     [r8],r4
//    }
	*dst = ((*dst)&(~mask))|data;
}

/////////////////////////////////////////////////////////////////////////////
// Atomic long set bits

ULONG atomic_long_set_bits(HPULONG dst, ULONG data)
{
//    __asm
//    {
//        exts    r9,#4
//        mov     r4,[r8]
//        or      r4,r10
//        mov     [r8],r4
//        exts    r9,#3
//        mov     r5,[r8+#2]
//        or      r5,r11
//        mov     [r8+#2],r5
//    }
	*dst = *dst | data;
	return (*dst);
}

/////////////////////////////////////////////////////////////////////////////
// Atomic long clear bits

ULONG atomic_long_clear_bits(HPULONG dst, ULONG data)
{
//    __asm
//    {
//        xor     r10,#0xffff
//        xor     r11,#0xffff
//        exts    r9,#4
//        mov     r4,[r8]
//        and     r4,r10
//        mov     [r8],r4
//        exts    r9,#3
//        mov     r5,[r8+#2]
//        and     r5,r11
//        mov     [r8+#2],r5
//    }
	*dst = *dst & ~data;
	return (*dst);
}

/////////////////////////////////////////////////////////////////////////////
// Bitfield read

ULONG bitfield_get(void * pvSrc, UWORD uwStart, UWORD uwSize)
{
	UWORD ns,ct,nt,*pd;
	UWORD *pld;
    ULONG dest=0l;
	
	pd=(UWORD *)&dest;
	pld=(UWORD *)pvSrc;
	ns=uwStart&15;
	for(ct=(uwStart>>4),nt=uwStart;nt<=(uwStart+uwSize);ct++,nt+=16)
        if(ns>0)
		*pd++=(pld[ct]>>ns)|(pld[ct+1]<<(16-ns));
        else
		    *pd++=pld[ct];

	*--pd&=(1<<(uwSize&15))-1;

    return dest;
}

/////////////////////////////////////////////////////////////////////////////
// Bitfield write

void bitfield_set(ULONG ulSrc, UWORD uwStart, UWORD uwSize, void * pvDst)
{
	UWORD ns,ct,nt,*pd,tmp;
	UWORD *pld;
	
	pd=(UWORD *)&ulSrc;
	pld=(UWORD *)pvDst;
	ns=uwStart&15;
	ct=uwStart>>4;
	nt=(uwStart+uwSize-1)>>4;
	
	if(nt>ct)
	{
		tmp=pld[ct]&((1<<ns)-1)|(pd[0]<<ns);
		for(;ct<nt;ct++,pd++)
		{
			pld[ct]=tmp;
            if(ns>0)
			tmp=(pd[0]>>(16-ns))|(pd[1]<<ns);
            else
    			tmp=pd[1];
		}
		nt=(UWORD)(1<<((uwStart+uwSize)&15))-1;
		pld[ct]=tmp&nt|pld[ct]&(0xffff^nt);
	}
	else
	{
		nt=(0xffff>>(16-uwSize))<<ns;
		pld[ct]=pld[ct]&(nt^0xffff)|(pd[0]<<ns)&nt;
	}
}
