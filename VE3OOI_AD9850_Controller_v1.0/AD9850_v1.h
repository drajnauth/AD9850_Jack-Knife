

#ifndef _DDS_H_
#define _DDS_H_



typedef struct {
  long int correction;   // can be + or -
  unsigned long CorrectedClock;
  unsigned char header;
} AD9850_def;

#define DDS_CLOCK 125000000
#define EEPROM_HEADER 0xAA

// These pins can be changed to any other pin. Its not static
#define _FQ_UD  9                     // Pin 9 - connect to AD9850 module update pin
#define _W_CLK  8                     // Pin 8 - connect to AD9850 module clock pin
#define _DATA 10                      // Pin 10 - connect to AD9850 moduledata pin
#define _RESET 11                     // Pin 11 - connect to AD9850 modulereset pin.
#define LED 13                        // Internal LED - Used for signalling


#define AD_MIN_OUT_FREQ 1000000
#define AD_MAX_OUT_FREQ 40000000

#define DEFAULT_CONTROL_WORD  0
#define POWERDOWN_CONTROL_WORD  0x20    // Power Down control word i.e. 00100000 = 0x20


// Define functions
void DDS60Setup (void); 
void SetFrequency(unsigned long frequency);
void SetFrequencyPhase(unsigned long frequency, unsigned char phase);
void SendDataFrame (unsigned long tword, unsigned char cword);
void ExecuteSerial (char *str);
void ADInit ( void );
void ShiftOut ( unsigned char value );
void ShiftOutMSB ( unsigned char value );
unsigned char SwapPhaseBits (unsigned char phasein) ;
void PulseAD9850Pin (unsigned char ppin);
void PowerDown (void);

#endif // _DDS_H_
