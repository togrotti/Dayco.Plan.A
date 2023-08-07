/****************************************************************************/
/* Project: Ax-Zynq Control Board                                           */
/*                                                                          */
/* Copyright © 2021, Ningbo Physis Technology Co.,Ltd. All Rights Reserved. */
/*                                                                          */
/* Version     : 1.0                                                        */
/* File        : OneWireHandler.c                                           */
/* Author      : Hu Xiaokai		                                              */
/*                                                                          */
/* Description :                                                            */
/*                                                                          */
/* References  : MAXIM AN126 : 1-Wire Communication Through Software        */
/*                     AN187 : 1-Wire Search Algorithm                      */
/*                                                                          */
/****************************************************************************/


#include <stdio.h>

#include "OneWireHandler.h"
#include "AxM-E-Defines.h"

#include "xtime_l.h"

/////////////////////////////////////////////////////////////////////////////
//

#ifdef _AXX_SYSAPP
#include "system\SysAppGlobals.h"
#else
#include "system\BootBlockGlobals.h"
#endif
//#define __PERFSEL

/////////////////////////////////////////////////////////////////////////////
// Compiler Option

#pragma GCC optimize (2)

/////////////////////////////////////////////////////////////////////////////
//

#define INP_FILTER       12

/////////////////////////////////////////////////////////////////////////////
//
// Standard Speed (resolution 1us)

#define OW_DELAY_A      6  //6uS
#define OW_DELAY_B     64  //64uS
#define OW_DELAY_C     60  //60uS
#define OW_DELAY_D     10  //10uS
#define OW_DELAY_E     5   // 90   //9uS
#define OW_DELAY_F     55  //55uS
#define OW_DELAY_G      0  //0uS
#define OW_DELAY_H    480  //480uS
#define OW_DELAY_I     70  //70uS
#define OW_DELAY_J    410  //410uS



#ifndef _HW_AXS

#ifndef _HW_DC

//-----------------------------------------------------------------------
// MIO[9] : Internal 1-Wire Bus
//-----------------------------------------------------------------------
#define OWBUS_INT_PIN  (MIO_PIN_BASE+9)
//-----------------------------------------------------------------------
// MIO[11] : External 1-Wire Bus
//-----------------------------------------------------------------------
#define OWBUS_EXT_PIN  (MIO_PIN_BASE+11)
//-----------------------------------------------------------------------
// MIO[8] : IO Expansion 1-Wire Bus
//-----------------------------------------------------------------------
#define OWBUS_IOE_PIN  (MIO_PIN_BASE+8)

#else

//-----------------------------------------------------------------------
// MIO[9] : CTB 1-Wire Bus
//-----------------------------------------------------------------------
#define OWBUS_INT_PIN  (MIO_PIN_BASE+9)

//-----------------------------------------------------------------------
// MIO[11] : PWB 1-Wire Bus
//-----------------------------------------------------------------------

#define OWBUS_EXT_PIN  (MIO_PIN_BASE+11)

//-----------------------------------------------------------------------
// MIO[15] : EXT 1-Wire Bus
//-----------------------------------------------------------------------
#define OWBUS_IOE_PIN  (MIO_PIN_BASE+15)

#endif

#else // _HW_AXS

#if defined(_HW_AXS_SAIETTA)
  // CTB 1-Wire Bus / PWB 1-Wire Bus / EXT 1-Wire Bus
  #define OWBUS_INT_PIN  (MIO_PIN_BASE+9)
  #define OWBUS_EXT_PIN  (MIO_PIN_BASE+9)
  #define OWBUS_IOE_PIN  (MIO_PIN_BASE+9)

#elif defined(_HW_AXS_CABI35KW)
  // CTB 1-Wire Bus
  #define OWBUS_INT_PIN  (MIO_PIN_BASE+48)
  // PWB 1-Wire Bus / EXT 1-Wire Bus
  #define OWBUS_EXT_PIN  (MIO_PIN_BASE+49)
  #define OWBUS_IOE_PIN  (MIO_PIN_BASE+49)

#elif defined(_HW_AXS_DAYCO22KW)
  // CTB 1-Wire Bus
  #define OWBUS_INT_PIN  (MIO_PIN_BASE+9)
  // PWB 1-Wire Bus / EXT 1-Wire Bus
  #define OWBUS_EXT_PIN  (MIO_PIN_BASE+11)
  #define OWBUS_IOE_PIN  (MIO_PIN_BASE+11)

#endif

#endif // _HW_AXS


/////////////////////////////////////////////////////////////////////////////
//
// Pause for exactly 'tick' number of ticks = 1us

static void tickStdDelay( int tick )
{
  usleep(tick);
}

#ifdef __PERFSEL
static void tickPerfDelay( int tick )
{
  while ( tick-- ) {
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
  }
}

static void (* pTickDelay)(int) = &tickStdDelay;

#define tickDelay(x)        (*pTickDelay)(x)

#else
#define tickDelay(x)        tickStdDelay(x)
#endif

/////////////////////////////////////////////////////////////////////////////
//
int OWHandlerInit( void )
{
  // setup proper delay routine depending on system clock
#ifdef __PERFSEL
  if(uwSystemMainClockFreq==SYSTEMCLOCK_PERF)
    pTickDelay = &tickPerfDelay;
#endif

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
//
static void outbit( int bus, int val )
{
  if ( bus == OW_INTERNAL_BUS )
  {
    Gpio_SetMode(OWBUS_INT_PIN, GPIO_DIR_OUT);
    GPIO_OUT(OWBUS_INT_PIN, val);
  }
  else if ( bus == OW_EXTERNAL_BUS )
  {
    Gpio_SetMode(OWBUS_EXT_PIN, GPIO_DIR_OUT);
    GPIO_OUT(OWBUS_EXT_PIN, val);
  }
  else if ( bus == OW_IOEXP_BUS )
  {
    Gpio_SetMode(OWBUS_IOE_PIN, GPIO_DIR_OUT);
    GPIO_OUT(OWBUS_IOE_PIN, val);
  }
}

/////////////////////////////////////////////////////////////////////////////
//
static int inpbit( int bus )
{
  unsigned int uwFilter = 0, uwTmp ;

  if ( bus == OW_INTERNAL_BUS )
    Gpio_SetMode(OWBUS_INT_PIN, GPIO_DIR_IN);
  else if ( bus == OW_EXTERNAL_BUS )
    Gpio_SetMode(OWBUS_EXT_PIN, GPIO_DIR_IN);
  else if ( bus == OW_IOEXP_BUS )
    Gpio_SetMode(OWBUS_IOE_PIN, GPIO_DIR_IN);

    // filtering the port read (to avoid glitches due to noise)
  for (uwTmp = 0; uwTmp < INP_FILTER; uwTmp++)
    if ( bus == OW_INTERNAL_BUS )
      uwFilter += (unsigned int)(GPIO_IN(OWBUS_INT_PIN)) ;
    else if ( bus == OW_EXTERNAL_BUS )
      uwFilter += (unsigned int)(GPIO_IN(OWBUS_EXT_PIN)) ;
    else if ( bus == OW_IOEXP_BUS )
      uwFilter += (unsigned int)(GPIO_IN(OWBUS_IOE_PIN)) ;

  if (uwFilter > INP_FILTER/2)
    return 1;
  else
    return 0;
}

//--------------------------------------------------------------------------
// Reset the 1-Wire bus and return the presence of any device
// Return TRUE if device present, 0 if no device present
//
int OWReset( int bus )
{
  int result;

  tickDelay( OW_DELAY_G );
  outbit( bus, 0 );                 // Drives DQ low
  tickDelay( OW_DELAY_H );
  outbit( bus, 1 );                 // Releases the bus
  tickDelay( OW_DELAY_I );
  result = inpbit( bus );           // Sample for presence pulse from slave
  tickDelay( OW_DELAY_J );          // Complete the reset sequence recovery

  return !result;                   // Return sample presence pulse result
}

//-----------------------------------------------------------------------------
// Send a 1-Wire write bit. Provide 10us recovery time.
//
void OWWriteBit( int bus, int val )
{
  if ( val ) {
    // Write '1' bit
    outbit( bus, 0 );               // Drives DQ low
    tickDelay( OW_DELAY_A );
    outbit( bus, 1 );               // Releases the bus
    tickDelay( OW_DELAY_B );        // Complete the time slot and 10us recovery
  } else {
    // Write '0' bit
    outbit( bus, 0 );               // Drives DQ low
    tickDelay( OW_DELAY_C );
    outbit( bus, 1 );               // Releases the bus
    tickDelay( OW_DELAY_D );
  }
}


//-----------------------------------------------------------------------------
// Read a bit from the 1-Wire bus and return it. Provide 10us recovery time.
//
int OWReadBit( int bus ) 
{
  int result;

  outbit( bus, 0 );                 // Drives DQ low
  tickDelay( OW_DELAY_A );
  outbit( bus, 1 );                 // Releases the bus
  tickDelay( OW_DELAY_E );
  result = inpbit( bus );           // Sample the bit value from the slave
  tickDelay( OW_DELAY_F );          // Complete the time slot and 10us recovery
  
  return result;
}

//-----------------------------------------------------------------------------
// Write 1-Wire data byte
//
void OWWriteByte( int bus, int data )
{
  int loop;

  // Loop to write each bit in the byte, LS-bit first
  for ( loop = 0; loop < 8; loop++ ) {
    //
    OWWriteBit( bus, data & 0x01 );
    // shift the data byte for the next bit
    data >>= 1;
  }
}


//-----------------------------------------------------------------------------
// Read 1-Wire data byte and return it
//
int OWReadByte( int bus )
{
  int loop, result = 0;

  for ( loop = 0; loop < 8; loop++ ) {
    // shift the result to get it ready for the next bit
    result >>= 1;
    // if result is one, then set MS bit
    if ( OWReadBit( bus ) )
      result |= 0x80;
  }
  return result;
}

//-----------------------------------------------------------------------------
// Write a 1-Wire data byte and return the sampled result.
//
int OWTouchByte( int bus, int data )
{
  int loop, result = 0;

  for ( loop = 0; loop < 8; loop++ ) {
    // shift the result to get it ready for the next bit
    result >>= 1;

    // If sending a '1' then read a bit else write a '0'
    if ( data & 0x01 ) {
      if ( OWReadBit( bus ) )
        result |= 0x80;
    } else
      OWWriteBit( bus, 0 );

    // shift the data byte for the next bit
    data >>= 1;
  }
  return result;
}


//-----------------------------------------------------------------------------
// Write a block 1-Wire data bytes and return the sampled result in the same buffer.
//

void OWBlock( int bus, unsigned char * data, int length, void (* yield)(void) )
{
  int loop;
  
  for ( loop = 0; loop < length; loop++ ) {
    data[ loop ] = (unsigned char)OWTouchByte( bus, data[ loop ] );
    if (yield)
      (*yield)();
  }
}



/////////////////////////////////////////////////////////////////////////////
//

static const unsigned char dscrc_table[] = 
{
	  0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
	157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
	 35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
	190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
	 70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
	219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
	101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
	248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
	140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
	 17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
	175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
	 50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
	202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
	 87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
	233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
	116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
};


//--------------------------------------------------------------------------
// Calculate the CRC8 of the byte value provided with the current
// global 'crc8' value.
// Returns current global crc8 value
//
static unsigned char docrc8( unsigned char prev, unsigned char value )
{
  return dscrc_table[ prev ^ value ];
}


//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE : device found, ROM number in ROM_NO buffer
// FALSE : device not found, end of search
//
static int OWSearch( int bus, OW_SEARCH_STATE * state, void (* yield)(void) )
{
  int id_bit_number;
  int last_zero, rom_byte_number, search_result;
  int id_bit, cmp_id_bit;
  unsigned char rom_byte_mask;
  int search_direction;

  // initialize for search
  id_bit_number = 1;
  last_zero = 0;
  rom_byte_number = 0;
  rom_byte_mask = 1;
  search_result = 0;
  state->crc8 = 0;

  // if the last call was not the last one
  if ( !state->LastDeviceFlag ) {

    // 1-Wire reset
    if ( !OWReset( bus ) ) {
      // reset the search
      state->LastDiscrepancy = 0;
      state->LastDeviceFlag = FALSE;
      state->LastFamilyDiscrepancy = 0;
      return FALSE;
    }

    if(yield)
      (*yield)();

    // issue the search command
    OWWriteByte( bus, 0xF0 );

    // loop to do the search
    do {
      if(yield)
        (*yield)();

      // read a bit and its complement
      id_bit = OWReadBit( bus );
      cmp_id_bit = OWReadBit( bus );

      // check for no devices on 1-wire
      if ( ( id_bit == 1 ) && ( cmp_id_bit == 1 ) )
        break;
      else {
        // all devices coupled have 0 or 1
        if ( id_bit != cmp_id_bit )
          search_direction = id_bit; // bit write value for search
        else {
          // if this discrepancy if before the Last Discrepancy on a previous next then pick the same as last time
          if ( id_bit_number < state->LastDiscrepancy )
            search_direction = ( ( state->ROM_NO[ rom_byte_number ] & rom_byte_mask ) > 0 );
          else
            // if equal to last pick 1, if not then pick 0
            search_direction = ( id_bit_number == state->LastDiscrepancy );

          // if 0 was picked then record its position in LastZero
          if ( search_direction == 0 ) {
            last_zero = id_bit_number;
            // check for Last discrepancy in family
            if ( last_zero < 9 )
              state->LastFamilyDiscrepancy = last_zero;
          }
        }

        // set or clear the bit in the ROM byte rom_byte_number with mask rom_byte_mask
        if ( search_direction == 1 )
          state->ROM_NO[ rom_byte_number ] |= rom_byte_mask;
        else
          state->ROM_NO[ rom_byte_number ] &= ~rom_byte_mask;

        // serial number search direction write bit
        OWWriteBit( bus, search_direction );

        // increment the byte counter id_bit_number and shift the mask rom_byte_mask
        id_bit_number++;
        rom_byte_mask <<= 1;

        // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
        if ( rom_byte_mask == 0 ) {
          state->crc8 = docrc8( state->crc8, state->ROM_NO[ rom_byte_number ] ); // accumulate the CRC
          rom_byte_number++;
          rom_byte_mask = 1;
        }
      }
    } while( rom_byte_number < 8 ); // loop until through all ROM bytes 0-7

    // if the search was successful then
    if ( !( ( id_bit_number < 65 ) || ( state->crc8 != 0 ) ) ) {
      // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
      state->LastDiscrepancy = last_zero;
      // check for last device
      if ( state->LastDiscrepancy == 0 )
        state->LastDeviceFlag = TRUE;
      search_result = TRUE;
    }
  }

  // if no device found then reset counters so next 'search' will be like a first
  if ( !search_result || !state->ROM_NO[ 0 ] ) {
    state->LastDiscrepancy = 0;
    state->LastDeviceFlag = FALSE;
    state->LastFamilyDiscrepancy = 0;
    search_result = FALSE;
  }
  return search_result;
}

//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire bus
// Return TRUE : device found, ROM number in ROM_NO buffer
// FALSE : no device present
//
int OWSearchFirst( int bus, OW_SEARCH_STATE * state, void (* yield)(void) )
{
  // reset the search state
  state->LastDiscrepancy = 0;
  state->LastDeviceFlag  = FALSE;
  state->LastFamilyDiscrepancy = 0;
  return OWSearch( bus, state, yield );
}

//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire bus with the device type 'family_code'
// Return TRUE : device found, ROM number in ROM_NO buffer
// FALSE : no device present
//
int OWSearchFamily( int bus, OW_SEARCH_STATE  * state, unsigned char family_code, void (* yield)(void) )
{
  int i;

  // set the ROM_NO 'family_code'
  state->ROM_NO[ 0 ] = family_code;
  for ( i = 1; i < 8; i++ )
    state->ROM_NO[ i ] = 0;
  // set the search state to find 'family_code' type devices
  state->LastDiscrepancy = 64;
  state->LastDeviceFlag  = FALSE;
  state->LastFamilyDiscrepancy = 0;
  return OWSearch( bus, state, yield );
}


//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire bus
// Return TRUE : device found, ROM number in ROM_NO buffer
// FALSE : device not found, end of search
//
int OWSearchNext( int bus, OW_SEARCH_STATE  * state, void (* yield)(void) )
{
  // leave the search state alone
  return OWSearch( bus, state, yield );
}


//--------------------------------------------------------------------------
// Read Memory
// Return TRUE : device found, memory read
// FALSE : device not found
//
int OWReadMemory( int bus, unsigned char * romno, unsigned char * buffer, int length, void (* yield)(void) )
{
  // Reset 1-Wire 
  if ( OWReset( bus ) ) {
    unsigned char sendpacket[ 9 ]; int i;
  
    // Issue “Match ROM” command
    sendpacket[ 0 ] = 0x55;
    // copy ROM code
    for ( i = 0; i < 8; i++ )
      sendpacket[ i + 1 ] = romno[ i ];
    // Send command sequence
    OWBlock( bus, sendpacket, 9, yield );
  
    // Issue “Read Memory” command
    sendpacket[ 0 ] = 0xF0;
    // TA1, beginning offset = 00h
    sendpacket[ 1 ] = 0x00;
    // TA2, address = 0000h
    sendpacket[ 2 ] = 0x00;
    // Send command sequence
    OWBlock( bus, sendpacket, 3, yield );
  
    // Read the entire memory
    for ( i = 0; i <= length; i++ )
    {
      buffer[ i ] = (unsigned char)OWReadByte( bus );
      if(yield)
        (*yield)();
    }

    return 1;
  } else
    return 0;
}

