/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* File        : CommonUtility.h                                            */
/* Author      : Stefano Martino                                            */
/* Author      : Fabio Terrile                                              */
/*                                                                          */
/* Description : Common utilities                                           */
/*                                                                          */
/****************************************************************************/

#ifndef _COMMONUTILITY_H
 #define _COMMONUTILITY_H

#include "CommonDefines.h"
//#include "DspFunctions.h"
//#include "FatalErrorCodes.h"
//#include <intrins.h>
#include "xplatform_info.h" //BSP: \zynq_som\freertos\bspinclude\include

/////////////////////////////////////////////////////////////////////////////
//
// Device family ident macros

#ifdef _INFINEON_
#define XFAMILY_GETID()                         ((SCU_IDCHIP)>>8)
#else
#define XFAMILY_GETID()                         XGetPlatform_Info()
#endif

/////////////////////////////////////////////////////////////////////////////
//
// General timing macros
#ifdef _INFINEON_
#define TIMER_VAR                               UWORD

// Wait for # waittime ticks of the timer_sel timer
#define timer_wait(timer_sel,waittime)          {SWORD loccnt; loccnt=(SWORD volatile)timer_sel+(SWORD)waittime; \
                                                    while((SWORD volatile)timer_sel-loccnt<0); }

// setup a local timer and non-blocking wait elapsing timer
#define	timer_settimeout(timer_sel,waittime)    ((TIMER_VAR)(timer_sel)+(waittime))
#define	timer_istimedout(timer_sel,waitvar)	    ((SWORD volatile)(timer_sel)-(SWORD)(waitvar)>=0)

// time profiling
#define	timer_profiler_start(timer_sel)         ((TIMER_VAR)timer_sel)
#define	timer_profiler_end(timer_sel,profvar)   ((TIMER_VAR)((UWORD volatile)timer_sel-(UWORD)profvar))

// check if time period is elapsed
#define	timer_deltatimedout(now,ts,delta)   	((SWORD volatile)(now)-(SWORD volatile)(ts)>=(SWORD volatile)delta)

// apply timing correction
#ifdef  _APP_XC
#define timer_100nscorrect(x)                   (_sint16_mpy_16_q14((x),uwSystemTmr100Corr))
#define timer_set100nstmr(x)                    (_sint16_mpy_16_q14((x),uwSystemSetTmr100))
#else
#define timer_100nscorrect(x)                   (x)
#define timer_set100nstmr(x)                    (x)
#endif
#else
#define TIMER_VAR                               UWORD

// Wait for # waittime ticks of the timer_sel timer
#define timer_wait(timer_sel,waittime)          {SWORD loccnt; loccnt=(SWORD volatile)timer_sel+(SWORD)waittime; \
                                                    while((SWORD volatile)timer_sel-loccnt<0); }

// setup a local timer and non-blocking wait elapsing timer
#define	timer_settimeout(timer_sel,waittime)    ((TIMER_VAR)(timer_sel)+(waittime))
#define	timer_istimedout(timer_sel,waitvar)	    ((SWORD)((UWORD volatile)(timer_sel)-(UWORD)(waitvar))>=0)

// time profiling
#define	timer_profiler_start(timer_sel)         ((TIMER_VAR)timer_sel)
#define	timer_profiler_end(timer_sel,profvar)   ((TIMER_VAR)((UWORD volatile)timer_sel-(UWORD)profvar))

// check if time period is elapsed
#define	timer_deltatimedout(now,ts,delta)   	((SWORD volatile)(now)-(SWORD volatile)(ts)>=(SWORD volatile)delta)

// apply timing correction
#define timer_100nscorrect(x)                   (x)
#define timer_set100nstmr(x)                    (x)
#endif

/////////////////////////////////////////////////////////////////////////////
//

UWORD crc16( UWORD uwCRC, const UBYTE * hpbyBuffer, UWORD uwLength );

/////////////////////////////////////////////////////////////////////////////
// Atomic read/write up to 8 byte

#define ATOMIC_RW_MAX_SIZE                      8

void atomic_read(HPVOID dst, const HPVOID src, const UWORD count);
void atomic_write(HPVOID dst, const HPVOID src, const UWORD count);
void atomic_move(HPVOID dst, const HPVOID src, const UWORD count);

/////////////////////////////////////////////////////////////////////////////
// Atomic read chunk of 16 bytes

#define ATOMIC_CHUCK16_SIZE                     16

void atomic_read_chunk16(HPVOID dst, const HPVOID src);

/////////////////////////////////////////////////////////////////////////////
// Atomic byte write bits

UWORD atomic_byte_write_bits(HPUBYTE dst, UBYTE data, UBYTE mask);

/////////////////////////////////////////////////////////////////////////////
// Atomic write n bits

UWORD atomic_write_bits(HPUWORD dst, UWORD data, UWORD mask);

/////////////////////////////////////////////////////////////////////////////
// Atomic long set bits

ULONG atomic_long_set_bits(HPULONG dst, ULONG data);

/////////////////////////////////////////////////////////////////////////////
// Atomic long clear bits

ULONG atomic_long_clear_bits(HPULONG dst, ULONG data);

/////////////////////////////////////////////////////////////////////////////
// Fast atomic set/clear bits
#warning "TODO: atomic & endatomic"
#ifdef _INFINEON_
#define fast_atomic_set_bits(x,y)               { \
                                                    _atomic_(0); \
                                                    (x)|=(y); \
                                                    _endatomic_(); \
                                                }

#define fast_atomic_clear_bits(x,y)             { \
                                                    _atomic_(0); \
                                                    (x)&=~(y); \
                                                    _endatomic_(); \
                                                }
#else // _infineon_
#define fast_atomic_set_bits(x,y)               { \
                                                    (x)|=(y); \
                                                }

#define fast_atomic_clear_bits(x,y)             { \
                                                    (x)&=~(y); \
                                                }
#endif // _infineon_
/////////////////////////////////////////////////////////////////////////////
// Fast atomic set/clear bits

#ifdef _INFINEON_
#define fast_atomic_set_flag(x)                 { \
                                                    _atomic_(0); \
                                                    (x)=TRUE; \
                                                    _endatomic_(); \
                                                }

#define fast_atomic_clear_flag(x)               { \
                                                    _atomic_(0); \
                                                    (x)=FALSE; \
                                                    _endatomic_(); \
                                                }
#else // _infineon_
#define fast_atomic_set_flag(x)                 { \
                                                    (x)=TRUE; \
                                                }

#define fast_atomic_clear_flag(x)               { \
                                                    (x)=FALSE; \
                                                }
#endif // _infineon_

/////////////////////////////////////////////////////////////////////////////
// Bitfield manipulation

ULONG bitfield_get(void * pvSrc, UWORD uwStart, UWORD uwSize);
void bitfield_set(ULONG ulSrc, UWORD uwStart, UWORD uwSize, void * pvDst);

/////////////////////////////////////////////////////////////////////////////
// Assertion macro

//#define assert(condition)               ((condition) ? ((void) 0) : ShowFatalErrorAndHalt(FATAL_ERROR_ASSERTION_FAILED))

#endif // _COMMON_UTILITY_H
