/*
These routines are used to configure and control the AD9850 Module which includes a crystal clock that drives the DDS

The AD9850 datasheet describes how to communicate with the AD9850 DDS chip. The module does not change this information.

The following pins are used to control/configure the AD9850 chip (regardless of module)

FQ_UD:
Frequency Update. On the rising edge of this clock, the DDS updates to the frequency (or phase)
loaded in the data input register; it then resets the pointer to Word 0ncy/phase/control words.

W_CLK:
Word Load Clock. This clock is used to load the parallel or serial frequency/phase/control words.

DATA (on module) or D7 (chip)
D7 (Pin 25) is part of parallel data load or serves as the input data pin for serial communications

RESET:
This is the master reset function; when set high, it clears all registers (except the input register), and
the DAC output goes to cosine 0 after additional clock cycles

The AD9850 expects a 40 bin frame which is made of a 32 bit frequency tuning word and 8 bit control word.
Bits W0 to W31 Tuning Word (W0 LSB, W31 MSB)
Bits W32 to W39 Control 

Tuning_Word = Fout x 2^32 / CLOCK

Conrol Word
W32 - Reserved must be 0
W33 - Reserved must be 0
W34 - Power up/Power down, 0=Power Up, 1=Power down
W35 - Phase LSB
W36 - Phase
W37 - Phase
W38 - Phase
W39 - Phase MSB

e.g. Power Down control word i.e. 00100000 = 0x20


Here are the steps to configure the AD9850.
1) Reset the chip by pulsing the RESET line (i.e. pulse is rising and falling edge of pin). For 125 Mhz module no delay requried between 
rising edge and falling edge

2) Set the chip in serial communicaiton mode. Drop the DATA to 0 then Pulse W_CLK and then pulse FQ_UD.  No delay required

3) Calculate the frequecy tuning word using the above formula

4) Shift out 32 bits of the tuning work using LSB first.  This means set DATA base on value of the bit being transmitted then 
pulse the W_CLK line to indicated a bit is present

5) Shift out 8 bits of the control word. For normal operation this should be 0.  Note the high order 2 bits MUST be 0.

*/


#include "Arduino.h"

#include <stdint.h>
#include <avr/eeprom.h>         // Needed for storeing calibration to Arduino EEPROM

#include "UART.h"              // VE3OOI Serial Interface Routines (TTY Commands)
#include "AD9850_v1.h"           // VE3OOI AD9850 Routines
#include "AD9850_Controller.h"  // Defines for this program

AD9850_def ad9850params;

void DDS60Setup (void) 
{
  pinMode (_DATA, OUTPUT); // sets pin 10 as OUPUT
  pinMode (_W_CLK, OUTPUT); // sets pin 9 as OUTPUT
  pinMode (_FQ_UD, OUTPUT); // sets pin 8 as OUTPUT
  pinMode (_RESET, OUTPUT);
  pinMode (LED, OUTPUT);
}

void ADInit ( void )
{
// Drop All Lines to 0
  digitalWrite(_RESET, LOW);
  digitalWrite(_W_CLK, LOW);
  digitalWrite(_FQ_UD, LOW);
  digitalWrite(_DATA, LOW);

// Pulse RESET for 5 CLKIN (Cycles i.e. 5x 1/125Mhz ~ 40ns)
  PulseAD9850Pin(_RESET);
  
// Set to serial mode. Need rising edge of _W_CLK followed by rising edge of _FQ_UD
  PulseAD9850Pin(_W_CLK);            // Pulse _W_CLK
  PulseAD9850Pin(_FQ_UD);            // Pulse _FQ_UD
}





void SetFrequency(unsigned long frequency)
{
  unsigned long ulongword;
  double temp;
  
  temp = (frequency * 0x100000000);
  temp /= ad9850params.CorrectedClock;

  ulongword = (unsigned long) temp;

  SendDataFrame (ulongword, 0);

}

void SetFrequencyPhase(unsigned long frequency, unsigned char phase)
{
/*
      case  %00000 PHASE_WORD = "     0"
      case  %00001 PHASE_WORD = " 11.25"
      case  %00010 PHASE_WORD = "  22.5"
      case  %00011 PHASE_WORD = " 33.75"
      case  %00100 PHASE_WORD = "    45"
      case  %00101 PHASE_WORD = " 56.25"
      case  %00110 PHASE_WORD = "  67.5"
      case  %00111 PHASE_WORD = " 78.75"
      case  %01000 PHASE_WORD = "    90"
      case  %01001 PHASE_WORD = "101.25"
      case  %01010 PHASE_WORD = " 112.5"
      case  %01011 PHASE_WORD = "123.75"
      case  %01100 PHASE_WORD = "   135"
      case  %01101 PHASE_WORD = "146.25"
      case  %01110 PHASE_WORD = " 157.5"
      case  %01111 PHASE_WORD = "168.75"
      case  %10000 PHASE_WORD = "   180"
      case  %10001 PHASE_WORD = "191.25"
      case  %10010 PHASE_WORD = " 202.5"
      case  %10011 PHASE_WORD = "213.75"
      case  %10100 PHASE_WORD = "   225"
      case  %10101 PHASE_WORD = "236.25"
      case  %10110 PHASE_WORD = " 247.5"
      case  %10111 PHASE_WORD = "258.75"
      case  %11000 PHASE_WORD = "   270"
      case  %11001 PHASE_WORD = "281.25"
      case  %11010 PHASE_WORD = " 292.5"
      case  %11011 PHASE_WORD = "303.75"
      case  %11100 PHASE_WORD = "   315"
      case  %11101 PHASE_WORD = "326.25"
      case  %11110 PHASE_WORD = " 337.5"
      case  %11111 PHASE_WORD = "348.75"
*/


  unsigned long ulongword;
  double temp;
  
  temp = (frequency * 0x100000000);
  temp /= ad9850params.CorrectedClock;

  ulongword = (unsigned long) temp;

// Pulse RESET for 5 CLKIN (Cycles i.e. 5x 1/125Mhz ~ 40ns)
  SendDataFrame (ulongword, phase);

}

/*
The device also provides five bits of digitally controlled phase modulation, which enables phase
shifting of its output in increments of 180°, 90°, 45°, 22.5°, 11.25°

0x01 180°
0x02 90°
0x04 45°
0x14 60°
0x1A 120°
*/


unsigned char SwapPhaseBits (unsigned char phasein) 
// This routines swaps bits in phase word so that LSB becomes MSB
{
  unsigned int i;
  unsigned char phaseout;

  phaseout = phasein;
  phaseout &= 0xE0;

  for (i=0; i<5 ; i++) {
    if (phasein & 0x1) {
      phaseout |= 1<<(4-i);
    } 
    phasein >>= 1;
  }
  return phaseout;
 
}

void SendDataFrame (unsigned long tword, unsigned char cword)
{

  // Shift out tuning word LSB first
  ShiftOut ( (unsigned char)(tword & 0xFF));
      
  tword >>= 8;
  ShiftOut ( (unsigned char)(tword & 0xFF));
            
  tword >>= 8;
  ShiftOut ( (unsigned char)(tword & 0xFF));

  tword >>= 8;
  ShiftOut ( (unsigned char)(tword & 0xFF));

  // Send Control Word MSB first, ie left shift
  // But need to swap bits of phase because Phase bits shifted LSB first
  cword = SwapPhaseBits (cword);
  ShiftOutMSB ( cword ); // Normally this is zero, for power up and phase 0

  // Tell AD9850 to Load 40 bits just transferred by pulsing FQ_UD
  digitalWrite(_DATA, LOW);
  PulseAD9850Pin(_FQ_UD);
}

void ShiftOut ( unsigned char value )
// Shift LSB out first (right shift)
{
  unsigned char i;
  for (i=0; i<8; i++) {
    if (value & 0x1) {
      digitalWrite(_DATA, HIGH);
    } else {
      digitalWrite(_DATA, LOW);
    }
    PulseAD9850Pin(_W_CLK);       // Indicate bin ready on Data pin to read    
    value >>= 1;
  }
}

void ShiftOutMSB ( unsigned char value )
// Shift MSB out first (left shift)
{
  unsigned char i;
  for (i=0; i<8; i++) {
    if (value & 0x80) {
      digitalWrite(_DATA, HIGH);
    } else {
      digitalWrite(_DATA, LOW);
    }
    PulseAD9850Pin(_W_CLK);       // Indicate bin ready on Data pin to read    
    value <<= 1;
  }
}


void PulseAD9850Pin (unsigned char ppin)
// Generate a rising and falling edge for a pin 
{
  digitalWrite(ppin, LOW);
  digitalWrite(ppin, HIGH);
  digitalWrite(ppin, LOW);
}


void PowerDown (void)
// This routine does a power down on the module
{

// Setup for serial Load
  digitalWrite(_DATA, LOW);
  
// Set to serial mode. Need rising edge of _W_CLK followed by rising edge of _FQ_UD
  PulseAD9850Pin(_W_CLK);            // Pulse _W_CLK
  PulseAD9850Pin(_FQ_UD);            // Pulse _FQ_UD

  // don't need to send frequency tuning work...powering down
  ShiftOut ( 0 );
  ShiftOut ( 0 );
  ShiftOut ( 0 );
  ShiftOut ( 0 );

  // Send Control Word
  // Power Down control word is 00100000 = 0x20
  ShiftOut ( POWERDOWN_CONTROL_WORD );    // Send power down control work

  digitalWrite(_DATA, LOW);
  PulseAD9850Pin(_FQ_UD);  
}




