/*

Program Written by Dave Rajnauth, VE3OOI to control the AD9850. 
It can output different frequencies from 1Mhz to 40Mhz on AD9850
It also provides functions to calibrate arduino and store the calibration value in EEPROM
*/

#include "Arduino.h"

#include <stdint.h>
//#include <avr/eeprom.h>         // Needed for storeing calibration to Arduino EEPROM

#include <EEPROM.h>
#include "UART.h"              // VE3OOI Serial Interface Routines (TTY Commands)
#include "AD9850_v1.h"           // VE3OOI AD9850 Routines
#include "AD9850_Controller.h"  // Defines for this program

// These variables are defined in UART.cpp and used for Serial interface
// rbuff is used to store all keystrokes which is parsed by Execute() 
// commands[] and numbers[] store all characters or numbers entered provided they
// are separated by spaces.  
// ctr is counter used to process entries
// command_entries contains the total number of charaters/numbers entered
extern char rbuff[RBUFF]; 
extern char commands[MAX_COMMAND_ENTRIES];
extern unsigned char command_entries;
extern unsigned long numbers[MAX_COMMAND_ENTRIES];
extern unsigned char ctr;

// This defines the various parameter used to program AD9850 (See Silicon Labs AN619 Note)
// multisynch defines specific parameters used to determine AD9850 registers
// clk0ctl, clk1ctl, clk2ctl defined specific parameters used to control each clock
extern AD9850_def ad9850params;

// the setup function runs once when you press reset or power the board
void setup() {

  // define the baud rate for TTY communications. Note CR and LF must be sent by  terminal program
  Serial.begin(9600);
  Serial.println ("VE3OOI AD9850 Controller v1.0"); 
  Serial.write ("\r\nRDY> ");
  Serial.flush();
  ResetSerial ();

  // Configure AD9850 pins, reset the module and set for serial communications
  DDS60Setup ();
  ADInit ();

  // Read XTAl correction value from Arduino eeprom memory
  EEPROMRead (0, (char *)&ad9850params, sizeof (ad9850params)); 

  // Check if calibrated and if not then set default parameters and write
  // Bascially there is byte which has a marker, check if marker defined
  if (ad9850params.header != EEPROM_HEADER) {
    Serial.println ("Init EEPROM");
    // Define default Crystal frequency used by AD9850
    ad9850params.CorrectedClock = DDS_CLOCK;

    // Define default correction
    ad9850params.correction = 0;

    // Define header
    ad9850params.header = EEPROM_HEADER;
    EEPROMWrite (0, (char *)&ad9850params, sizeof (ad9850params));
  }
}



// the loop function runs over and over again forever
void loop() {

// Look for characters entered from the keyboard and process them
// This function is part of the UART package.
  ProcessSerial ();
  
}

void ExecuteSerial (char *str)
{
  
// num defined the actual number of entries process from the serial buffer
// i is a generic counter
  unsigned char num;
  unsigned long i;
  double ftemp;
  
// This function called when serial input in present in the serial buffer
// The serial buffer is parsed and characters and numbers are scraped and entered
// in the commands[] and numbers[] variables.
  num = ParseSerial (str);

// Process the commands
// Note: Whenever a parameter is stated as [CLK] the square brackets are not entered. The square brackets means
// that this is a command line parameter entered after the command.
// E.g. F [CLK] [FREQ] would be mean "F 0 7000000" is entered (no square brackets entered)
  switch (commands[0]) {
    
    // Calibrate the AD9850.
    // Syntax: C [CAL] [FREQ], where CAL is the new Calibration value and FREQ is the frequency to output
    // Syntax: C , If no parameters specified, it will display current calibration value
    // Bascially you can set the initial CAL to 100 and check fequency accurate. Adjust up/down as needed
    // numbers[0] will contain the correction, numbers[1] will be the frequency in Hz
    case 'C':             // Calibrate
   
      // First, Check inputs to validate
      if (numbers[0] == 0 && numbers[1] == 0) {
        EEPROMRead (0, (char *)&ad9850params, sizeof (ad9850params));
        Serial.print ("Correction: ");
        Serial.println (ad9850params.correction);
        break;
      } else if (numbers[0] == 0) {
        Serial.println ("Bad Calibration");
        break;
      } else if (numbers[1] < AD_MIN_OUT_FREQ || numbers[1] > AD_MAX_OUT_FREQ) {
        Serial.println ("Bad Frequency");
        break;
      }
      
      // New value defined so read the old values and display what will be done
      EEPROMRead (0, (char *)&ad9850params, sizeof (ad9850params));
      Serial.print ("Old Correction: ");
      Serial.println (ad9850params.correction);
      Serial.print ("New Correction: ");
      Serial.println (numbers[0]);

      // Store the new value entered, reset the AD9850 and then display frequency based on new setting     
      ad9850params.correction = numbers[0];
      ftemp = ad9850params.correction * DDS_CLOCK; 
      ftemp /= 10000000.0;
      ad9850params.CorrectedClock = DDS_CLOCK + (unsigned long) ftemp;

      EEPROMWrite (0, (char *)&ad9850params, sizeof (ad9850params));
  
      SetFrequency (numbers[1]);
      break;

    // This command power downs the AD9850.  
    case 'D':             // Shutdown or powerdown
      PowerDown ();
      break;


    // This command is used to quickly set the frequency. 
    // Syntax: F [FREQ] FREQ is output frequency between 1Mhz to 40Mhz (above 40Mhz the sign is not stable or clean)
    case 'F':             // Set Frequency
      // Validate inputs
      if (numbers[0] < AD_MIN_OUT_FREQ || numbers[1] > AD_MAX_OUT_FREQ) {
        Serial.println ("Frequency out of range");
        break;
      }

      SetFrequency (numbers[0]);
      break;

    // Help Screen. This consumes a ton of memory but necessary for those
    // without much computer or programming experience.
    case 'H':             // Help
      Serial.println ("C - Disp Calib");
      Serial.println ("\tE.g. C");
      Serial.println ("C [CAL] [F] - Set Calib for Freq F");
      Serial.println ("\tE.g. C 100 10000000");
      Serial.println ("D - Power down the module");
      Serial.println ("\tE.g. D");
      Serial.println ("F [F] - Set Freq F and Phase 0");
      Serial.println ("\tE.g. F 10000000");
      Serial.println ("P [F] [P] - Set Freq F with phase P. Check Documentation");
      Serial.println ("\tE.g. F 10000000");
      Serial.println ("S [S] [E] [I] [delay]- Sweep Start to End with Increment with delay ms on all Channels");
      Serial.println ("\tE.g. S 7000000 73000000 100");
      Serial.println ("T [F] [P] [C] - Generate test PSK at Freq F with phase shift P for duation C (Cx64 ms)");
      Serial.println ("\tE.g. P 16 99 (99x64ms duration)");
      Serial.println ("R - Reset");
      break;


    // This command is used to change the phase of a frequency. 
    // Syntax: P [FREQ] [PHASE], FREQ is output frequency between 1Mhz to 40Mhz (above 40Mhz the sign is not stable or clean), PHASE is the phase
    case 'P':             // Set Frequency and phase
      // Validate inputs
      if (numbers[0] < AD_MIN_OUT_FREQ || numbers[1] > AD_MAX_OUT_FREQ) {
        Serial.println ("Frequency out of range");
        break;
      }
      
      // Check is reserved bits set or powerdown bits set
      if (numbers [1] & 0xE0) {
        Serial.println ("Bad Phase");
        break;
      }
      SetFrequencyPhase (numbers[0], numbers[1]);
      break;
     
    // This command reset the AD9850.  
    case 'R':             // Reset
      ADInit ();
      break;

    // This command is used to sweep between two frequencies. 
    // Syntax: S [START] [STOP] {INC] [DELAY] where START is the starting frequency, stop is the ending frequency
    // INC is the amount to increment the frequency by and [DELAY] is the pause in ms before changing frequeny
    // E.g. S 1000000 10000000 1000000 2000 , Sweep from 1 Mhz to 10 Mhz and increment by 1 Mhz.  Pause for 2 seconds before changing frequncy
    // The output is fixed to PLLA and all clocks display the frequencies
    case 'S':             // Scan Frequencies      
      // Validate the inputs
      if (numbers[0] < AD_MIN_OUT_FREQ || numbers[0] > AD_MAX_OUT_FREQ || numbers[1] < AD_MIN_OUT_FREQ || 
          numbers[1] > AD_MAX_OUT_FREQ) {
        Serial.println ("Bad Freq");
        break;
      }
      if (numbers[2] == 0 || numbers[2] > numbers[1]) {
        Serial.println ("Bad Inc");
        break;
      }
      if (numbers[3] == 0 || numbers[3] > 3000) {
        numbers[3] = 500;
      }
      Serial.print ("Sweep Freq: ");
      Serial.print (numbers[0]);
      Serial.print (" to: ");
      Serial.print (numbers[1]);
      Serial.print (" Inc: ");
      Serial.println (numbers[2]); 
      Serial.flush();
     
     for (i=numbers[0]; i<=numbers[1]; i+=numbers[2]) { 
      SetFrequency (i);
      Serial.println (i);
      delay (numbers[3]);
     }
      break;

    // This command sends psk signal as a test.  
    case 'T':             // PSK Test
      // Validate frequency
      if (numbers[0] < AD_MIN_OUT_FREQ || numbers[1] > AD_MAX_OUT_FREQ) {
        Serial.println ("Frequency out of range");
        break;
      }
      // Check is reserved bits set or powerdown bits set
      if (numbers [1] & 0xE0) {
        Serial.println ("Bad Phase");
        break;
      }
      if (numbers[2] > 100) {
        Serial.println ("Bad Duration");
        break;
      } else if (!numbers[2]) {
        numbers[2] = 30;
      }
      for (i=0; i<numbers[2]; i++) {
         SetFrequencyPhase (numbers[0], numbers [1]);
         delay (32);
         SetFrequencyPhase (numbers[0], 0);
         delay (32);
      }
      PowerDown ();
      break;

     
    // If an undefined command is entered, display an error message
    default:
      ErrorOut ();
  }
  
}


// Note that some arduino do not have eeprom and would generate an error during compile.
// If you plan to use a Arduino without eeprom then you need to hard code a calibration value.
void EEPROMWrite (unsigned int memAddr, char *cptr, unsigned int memlen)
// write the static parameter values to Arduino eeprom
{
  unsigned int i;
  for (i=0; i<memlen; i++) {
    EEPROM.write((memAddr+i), (unsigned char)*cptr);
    cptr++;
  }
}

void EEPROMRead (unsigned int memAddr, char *cptr, unsigned int memlen)
// read the static parameter values from Arduino eeprom
{
  unsigned int i;
  for (i=0; i<memlen; i++) {
    *cptr =  EEPROM.read(memAddr+i);
    cptr++;
  }
}


