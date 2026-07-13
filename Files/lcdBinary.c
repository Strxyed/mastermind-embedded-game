/* ***************************************************************************** */
/* You can use this file to define the low-level hardware control fcts for       */
/* LED, button and LCD devices.                                                  */ 
/* Note that these need to be implemented in Assembler.                          */
/* You can use inline Assembler code, or use a stand-alone Assembler file.       */
/* Alternatively, you can implement all fcts directly in master-mind.c,          */  
/* using inline Assembler code there.                                            */
/* The Makefile assumes you define the functions here.                           */
/* ***************************************************************************** */


#ifndef	TRUE
#  define	TRUE	(1==1)
#  define	FALSE	(1==2)
#endif

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

#define	INPUT			 0
#define	OUTPUT			 1

#define	LOW			 0
#define	HIGH			 1


// APP constants   ---------------------------------

// Wiring (see call to lcdInit in main, using BCM numbering)
// NB: this needs to match the wiring as defined in master-mind.c

#define STRB_PIN 24
#define RS_PIN   25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

// -----------------------------------------------------------------------------
// includes 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <time.h>

// -----------------------------------------------------------------------------
// prototypes

int failure (int fatal, const char *message, ...);

// -----------------------------------------------------------------------------
// Functions to implement here (or directly in master-mind.c)

/* this version needs gpio as argument, because it is in a separate file */
void digitalWrite (uint32_t *gpio, int pin, int value) {
  // In a real solution, use inline ARM assembler.
  // For demonstration we use C:
  if (value == HIGH) {
    // GPSET0 register is typically at offset 7 (each register is 4 bytes)
    gpio[7] = (1 << pin);
  } else {
    // GPCLR0 register is typically at offset 10.
    gpio[10] = (1 << pin);
  }
}


// adapted from setPinMode
void pinMode(uint32_t *gpio, int pin, int mode) {
  int reg = pin / 10;
  int shift = (pin % 10) * 3;
  uint32_t mask = ~(7 << shift);
  uint32_t value = (mode == OUTPUT) ? (1 << shift) : 0;
  gpio[reg] = (gpio[reg] & mask) | value;
}


void writeLED(uint32_t *gpio, int led, int value) {
 
  digitalWrite(gpio, led, value);
}


int readButton(uint32_t *gpio, int button) {
  // Returns HIGH if the button’s bit is set.
  return (gpio[13] & (1 << button)) ? HIGH : LOW;
}


void waitForButton(uint32_t *gpio, int button) {
  // Wait until the button is pressed and then released.
  while(readButton(gpio, button) == LOW) {
    ; // busy wait or add a short delay
  }
  // Wait for button release
  while(readButton(gpio, button) == HIGH) {
    ; 
  }
}

