/* 
    Manchester - Enhanced MasterPlay Clone
    Daniel Jose Viana, July-September 2020 - danjovic@hotmail.com
   
   
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
    
    Planned features:
    - Autofire
    - Support for Sega 6 Button controllers (for MODE key)
    - Detection of NES/SNES controller and automatic reorder of A/B buttons
	
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                         ///
///                                    MODULOS E DEFINICOES EXTERNOS                                        ///
///                                                                                                         ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <pic14regs.h>
#include <stdint.h>
#include <stdbool.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                         ///
///                                    CHIP CONFIGURATION                                                   ///
///                                                                                                         ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint16_t __at _CONFIG configWord = _INTRC_OSC_NOCLKOUT & // internal RC oscillator, IO on pins RA4/5
                                   _CPD_OFF &            // data protecion off
                                   _MCLRE_OFF &          // pin MCLR as IO, internal reset
                                   _CP_OFF   &           // code protection off
                                   _WDT_OFF &            // watchdog off
                                   _PWRTE_ON ;           // powerup timer on



///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                         ///
///                                    DEFINITIONS AND CONSTANTS                                            ///
///                                                                                                         ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
   PIC16F628 pinout
                +--___--+                            
  BOTTOM  RA2 --|1    18|-- RA1  DATA                
     TOP  RA3 --|2    17|-- RA0  START                     
   PAUSE  RA4 --|3    16|-- RA7  C_ST                     
     B_A  RA5*--|4    15|-- RA6  SEL/LAT                    
          VSS --|5    14|-- VDD                      
    POTY  RB0 --|6    13|-- RB7  RIGHT       
    POTX  RB1 --|7    12|-- RB6  LEFT                     
   HALFX  RB2 --|8    11|-- RB5  DOWN         
   HALFY  RB3 --|9    10|-- RB4  UP                             
                +-------+

*/

#define TRIS_POTY   TRISB0
#define TRIS_POTX   TRISB1
#define TRIS_HALFX  TRISB2
#define TRIS_HALFY  TRISB3
#define TRIS_UP     TRISB4
#define TRIS_DOWN   TRISB5
#define TRIS_LEFT   TRISB6
#define TRIS_RIGHT  TRISB7

#define TRIS_START  TRISA0
#define TRIS_DATA   TRISA1
#define TRIS_BOTTOM TRISA2
#define TRIS_TOP    TRISA3
#define TRIS_PAUSE  TRISA4
#define TRIS_BA     TRISA5
#define TRIS_SELLAT TRISA6
#define TRIS_CSTART TRISA7
					
#define PIN_START   RA0
#define PIN_DATAIN  RA1
#define PIN_BOTTOM	RA2			
#define PIN_TOP     RA3
#define PIN_PAUSE   RA4
#define PIN_B_A     RA5
#define PIN_C_ST    RA6
#define PIN_SEL_LAT RA7
					
#define PIN_POTY    RB0
#define PIN_POTX    RB1
#define PIN_HALFX   RB2
#define PIN_HALFY   RB3
#define PIN_UP      RB4
#define PIN_DOWN    RB5
#define PIN_LEFT    RB6
#define PIN_RIGHT   RB7


#define setLatClkLow()  PIN_SEL_LAT=0
#define setLatClkHigh() PIN_SEL_LAT=1

#define dataIn() ( PIN_DATAIN == 0 )

#define _TRGA  (1<<0)  //      _____                                                     _____
#define _TRGB  (1<<1)  // LAT       \___________________________________________________/
#define _SEL   (1<<2)  //      _____                                                     ______
#define _STRT  (1<<3)  // CLK       \_______/\____/\____/\____/\____/\____/\____/\____/\/
#define _UP    (1<<4)  // 
#define _DOWN  (1<<5)  // DATA XXXXXX   A   X  B  X Sel X Sta X  UP X  DW X  LF X  RG XXXXXXXXXXX
#define _LEFT  (1<<6)  // 
#define _RIGHT (1<<7)  //

#define pinUP()     ( PIN_UP    == 0 )
#define pinDOWN()   ( PIN_DOWN  == 0 )
#define pinLEFT()   ( PIN_LEFT  == 0 )
#define pinRIGHT()  ( PIN_RIGHT == 0 )
#define pinBA()     ( PIN_B_A   ==0  )
#define pinCSTART() ( PIN_C_ST  == 0 )

#define fire1Low()  do { TRIS_TOP    = 0; PIN_TOP    = 0; } while (0)
#define fire1Open() do { TRIS_TOP    = 1; PIN_TOP    = 1; } while (0)
#define fire2Low()  do { TRIS_BOTTOM = 0; PIN_BOTTOM = 0; } while (0)
#define fire2Open() do { TRIS_BOTTOM = 1; PIN_BOTTOM = 1; } while (0)

#define setKeySel()       PIN_PAUSE = 1
#define releaseKeySel()   PIN_PAUSE = 0
#define setKeyStart()     PIN_START = 1
#define releaseKeyStart() PIN_START = 0


#define set_X_left()      do { TRIS_HALFX = 1; TRIS_POTX = 0; PIN_HALFX = 0; PIN_POTX = 1; } while (0)  // minimum resistance
#define set_X_middle()    do { TRIS_HALFX = 0; TRIS_POTX = 1; PIN_HALFX = 1; PIN_POTX = 0; } while (0)
#define set_X_right()     do { TRIS_HALFX = 1; TRIS_POTX = 1; PIN_HALFX = 0; PIN_POTX = 0; } while (0)  

#define set_Y_north()     do { TRIS_HALFY = 1; TRIS_POTY = 0; PIN_HALFY = 0; PIN_POTY = 1; } while (0)  // minumum resistance
#define set_Y_middle()    do { TRIS_HALFY = 0; TRIS_POTY = 1; PIN_HALFY = 1; PIN_POTY = 0; } while (0)
#define set_Y_south()     do { TRIS_HALFY = 1; TRIS_POTY = 1; PIN_HALFY = 0; PIN_POTY = 0; } while (0)

#define delay5us()        do { __asm__("nop\n nop\n nop\n nop\n nop"); } while (0)
#define delay4us()        do { __asm__("nop\n nop\n nop\n nop"); } while (0)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                         ///
///                                               FUNCTIONS                                                 ///
///                                                                                                         ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Scan Sega Genesis controller 
//  Outpug bit 0  1  2  3  4  5  6  7  
//             C  B  A  St UP Dw Lf Rg Sega 
//             A  B  Sl St Up Dw Lf Rg nes
uint8_t ScanSega(void) {
   uint8_t i,data;
   setLatClkLow();  // Low level on SEL line
   delay5us(); 
   
   data=0;
   // check for SEGA Genesis controller
   if ( pinLEFT() && pinRIGHT() ) {  // Genesis controller
      if ( pinBA()     ) data |= _SEL;	  
      if ( pinCSTART() ) data |= _STRT;   
   } 
   
   setLatClkHigh();
   for (i=20;i>0;i--){  // wait 100us for RC network to settle
      delay5us();   
   }
   if ( pinUP()     ) data |= _UP;
   if ( pinDOWN()   ) data |= _DOWN;	  
   if ( pinLEFT()   ) data |= _LEFT;	  
   if ( pinRIGHT()  ) data |= _RIGHT;  
   if ( pinBA()     ) data |= _TRGA;	  
   if ( pinCSTART() ) data |= _TRGB;    
   
   return data;
} // ScanSega()

//
// Scan NES/SNES controller 
//  bit in  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
//  NES     A  B  Sl St Up Dw Lf Rg 0  0  0  0  0  0  0  0  (original)
//  NES     A  B  Sl St Up Dw Lf Rg 1  1  1  1  1  1  1  1  (clone)
// SNES     B  Y  Sl St Up Dw Lf Rg A  X  L  R  0  0  0  0  (original/clone)
// 
uint8_t ScanNES( void) {

   uint8_t i,j,data;
   // bring LATCH/CLOCK line down by 100us
   setLatClkLow();

   // delay 100us approximately
   for (j=12;j>0;j--)
      __asm__ ("nop") ;

   data = 0;
   for (i=0;i<8;i++) {
      data >>=1;
      if (dataIn() ) // test for zero
         data |= (1<<7);

   // PulseClock
   setLatClkHigh();
   delay5us();
   setLatClkLow();
}

   // restore clock line level
   setLatClkHigh();

   return data;
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                         ///
///                                          MAIN PROGRAM                                                   ///
///                                                                                                         ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void main (void)
{
   uint8_t stick;  // store controller directionals / buttons
   uint8_t dly;     // delay time


  // Initialize Hardware
 #ifdef __PIC16F688_H__  // Compatibility with 16F630
   OSCCON = 0x61;                     // Initialize clock, 4MHz, 
   ANSEL = 0;                         // All pins as digital i/o
   SPEN=0;                            // Disable serial port (enable RC3/4 ad digital IO) 
#endif 
   CMCON = ( _CM2 | _CM1 | _CM0);     // Analog comparators OFF


  // Initialize interface pins with Genesis Controller
  TRIS_START  = 0; // Key Start as output
  TRIS_DATA   = 1; // NES/SNES Data as input
  TRIS_BOTTOM = 1; // 
  TRIS_TOP    = 1; // 
  TRIS_PAUSE  = 0; // Key Pause as output
  TRIS_BA     = 1; // 
  TRIS_SELLAT = 0; // LATCH/CLOCK as output
  TRIS_CSTART = 1; // 				

  PIN_START   = 0; // Key Start unactive
  PIN_DATAIN  = 0; // 
  PIN_BOTTOM  = 1; // 			
  PIN_TOP     = 1; // 
  PIN_PAUSE   = 0; // Key Pause unactive
  PIN_B_A     = 0; // 
  PIN_SEL_LAT = 1; // Set LATCH/CLOCK in "HIGH"
  PIN_C_ST    = 0; // 

  TRIS_POTY   = 1; // Y axis at middle
  TRIS_HALFY  = 0; // 
 
  TRIS_POTX   = 1; // X axis at middle
  TRIS_HALFX  = 0; // 

  TRIS_UP     = 1; // Genesis UP as input
  TRIS_DOWN   = 1; // Genesis DOWN as input
  TRIS_LEFT   = 1; // Genesis LEFT as input
  TRIS_RIGHT  = 1; // Genesis RIGHT as input
 
  PIN_POTY    = 0; // Y axis at middle
  PIN_HALFY   = 1; //   
  PIN_POTX    = 0; // X axis at middle
  PIN_HALFX   = 1; // 

  PIN_UP      = 0; // No pullups
  PIN_DOWN    = 0; //  
  PIN_LEFT    = 0; //  
  PIN_RIGHT   = 0; //    
  
  
  
/*  
   // Initialize Hardware - Summary
   TRISA = (1<<5) | (1<<4) | (1<<3) | (1<<2) | (1<<1) | (0<<0);
   TRISB = (1<<5) | (1<<4) | (1<<3) | (0<<2) | (1<<1) | (0<<0);

   PORTA = (1<<5) | (1<<4) | (1<<3) | (1<<2) | (1<<1) | (1<<0);
   PORTB = (1<<5) | (0<<4) | (0<<3) | (1<<2) | (1<<1) | (1<<0);
*/   
   
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main loop
//
  for (;;) {
	  
   stick = ScanSega(); // Scan DB9 - Sega/Master/Atari
   stick |= ScanNES(); // Scan NES/SNES controller   
    
  
   // Activate 5200 port accordingly 
   if ((stick & _UP) && (stick & _DOWN) ) // if both UP and DOWN go to middle
      set_Y_middle();
   else 
      if ( stick & _UP    ) 
         set_Y_north();
   else if ( stick & _DOWN  ) 
      set_Y_south();
   else set_Y_middle();

   if ((stick & _LEFT) && (stick & _RIGHT) ) // if both LEFT and RIGHT go to middle
      set_X_middle();  
   else if ( stick & _LEFT    ) 
      set_X_left();
   else if ( stick & _RIGHT  ) 
      set_X_right();
   else set_X_middle(); 
   
   
   // Activate fire buttons accordingly
   if ( stick & _TRGA ) fire1Low(); else fire1Open();
   if ( stick & _TRGB ) fire2Low(); else fire2Open();
   
   // Activate keypad functions accordingly
   if ( stick & _SEL )  setKeySel();   else releaseKeySel(); 
   if ( stick & _STRT ) setKeyStart(); else releaseKeyStart();  

   // Give some time for RC trap on SNES controller to recharge
   dly=100;
   do { delay4us(); } while (--dly); // 10 cycles each iteration, a total of ~ 1ms

  } // for
} // main