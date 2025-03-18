
#ifndef VGAGLOBALS_H
#define VGAGLOBALS_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "stdlib.h"
#include "hardware/pio.h"
#include "VGASignals.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include "hardware/timer.h"

/* 
Notes on Horizontal Cycles:

HSYNC sm clock is divided by 4. With a 126 MHz system clock the PIOASM cycles
for the horizontal line are:

Active Region = 801 cycles (640 pixels)
Front Porch = 20 cycles (16 pixels)
Horz. Pulse = 120 cycles (96 pixels)
Back Porch = 60 cycles (48 pixels)

The cycle timings are based on the following:
    time_req = num_pixels / 25.175 MHz
    time_per_cycle = 4 / 126 MHz
    num_cycles = time_req / time_per_cycle

A divisor of 4 was selected because it gives under a 10 ns error for each
horizontal region and just over 100 ns of error for the entire vertical 
active region

*/


/* 
   overclocking to 126 MHz improves timing since 126000000 Hz is close to 125875000 Hz (5x pixel)
   Each pixel is outputed every 5 cycles in the PIOASM which gives room for other commands.
   The timming error is under 0.04 ns per pixels (25 ns total for the entire active line)
*/

#define SYSCLK 126000000     

#define HSYNC_PIN 15            // pico pin 20 
#define VSYNC_PIN 13            // pico pin 19, gpio 14 used for enabling pixel output 
#define PIXEL_PIN  12           // pico pin 16

#define V_ACTIVE_LINES 480 
#define HORZ_PIXELS 640 

#define V_FRONTP_LINES 10       // not using these line values in code    
#define V_SYNCP_LINES 2         
#define V_BACKP_LINES 33 

#define FRMBUFFSIZE 9600        // number of 32 bit words 
#define CHRMEMSIZE 4800 
#define COLS 80

#define CLK 16                  // rising edge trigger. Enabled by CS interrupt handler
#define CS_ENABLE 17            // falling edge trigger. Sets read/write mode and clokk interrupt
#define RW_ENABLE 18            // active low write enable

#define D0 0                    // data pins 
#define D1 1
#define D2 2 
#define D3 3                
#define D4 4
#define D5 5 
#define D6 6              
#define D7 7

#define R0 8                  // register select lines 
#define R1 9
#define R2 10 
 
#define CMDREG 0x00           // registers
#define CHRREG 0x01

#define CURSORIRQ 0
#define CURSORSPD 500000      // blink speed in microseconds 


/* 

Registers:
                   R2  R1  R0
Command Register    0   0   0
Character Register  0   0   1


Commands
---------------------------------------------------------------------------------------------------------------------------------------------------------

output control:
     
    0x00    auto increment on character print. Default is on when controller starts. Write any byte to turn on    
    0x01    view cursor. Default is on when controller starts. Write any byte to turn on
    0x02    set 8x16 charset. Default is on when controller starts. Write any byte to set to 8x16 chars    
    0x03    invert character set pixels (tested)        
    0x04    set auto-warp. Default is on when controller starts. Write any byte to set on (VT100: ^[[?7h)
    
    NEED output control commands to turn off !!!


    0x??    save cursor position, relative to home position, not impacted by change in row offset (VT100: ^[7)
    0x??    restore cursor position, relative to home position, not impacted by change in row offset (VT100: ^[8)
    
    0x??    set scroll region, only rows between top and bottom margins scroll. 
        

move cursor:

    0x09    horizontal tab right (tested) (ascii)
    0x0?    horizontal tab left (to do!!!)         
    0x05    move cursor to top left (home) (tested)  (VT100: ^[[H)  
    0x0A    linefeed (tested) (ascii)  
    0x0D    carriage return (tested) (ascii)
  
    0x??    move cursor up one line, bounded at row 0 (VT100:
    0x??    move cursor down one line, bounded at max rows (VT100:
    0x??    move cursor left one column, bounded at col 0 (VT100:
    0x??    move cursor right one column, bounded at max col (VT100:

    0x??    move cursor up n lines, bounded at row 0 (VT100: ^[[<n>A)
    0x??    move cursor down n lines, bounded at max rows (VT100: ^[[<n>B)
    0x??    move cursor left n columns, bounded at col 0 (VT100: ^[[<n>D)
    0x??    move cursor right n columns, bounded at max cols (VT100: ^[[<n>C)

    0x??    move cursor to screen location col=x row=y (top left corner is 0,0) (VT100: ^[[<v>;<h>H)

delete:
    0x08    backspace (tested) (ascii)
    0x0C    clear screen (tested) (ascii)
    0x7F    delete (tested) (ascii)
    0x10    clear line from cursor right (tested) (VT100: ^[[0K)
    0x11    clear line from cursor left (tested) (VT100: ^[[1K)
    0x12    clear entire line (tested)  (VT100: ^[[2K)
    0x20    clear screen from cursor down (tested) (VT100: ^[[0J) 
    0x21    clear screen from cursor up (tested) (VT100: ^[[1J)

scroll/rotate:
    0x30    rotate down one pixel (tested, there is some shimmering on screen)
    0x3?    rotate up one pixel 
    0x3?    scroll window up one line    (VT100: ^[D)
    0x3?    scroll window down one line    (VT100: ^[M)

*/


// ------------------------------- Global variable declarations for all files --------------------------------

extern unsigned char** TEXTCHARS;           // pointer that can switch between 8x16 and 8x8 char arrays
extern unsigned char** CHRS8x16;            // pointer for dynamic memory allocation for 8x16 fonts (can be used for custom fonts)
extern unsigned char** CHRS8x8;             // pointer for dynamic memory allocation for 8x8 fonts (can be used for custom fonts)

extern uint ROWS;
extern uint CHRHEIGHT;

extern unsigned char charMem[];             // character memory: rows by columns flat array  
extern uint frameBuffer[];                  // 640x480 pixels as a flat array of 32-bit words

extern uint nextCharCol;                    // next free char location (cursor position)
extern uint nextCharRow;

extern uint charRowOffset;                  // row offset for line scrolling
extern uint pixOffset;                      // row offset for pixel scrolling.  Tested on 3 monitors and need to start on row 1  

extern uint nextCharBufferWord;             // frame buffer word that contains pixel row 0 for next character. set by set_nextCharBufferWord
extern int nextCharWordOffset;              // the offset within frame buffer word (little endian format). set by set_nextCharBufferWord 

extern bool ChrIncrement;                   // auto increment to next character  
extern bool lineWrap;                       // auto linefeed if end of line                    

extern volatile bool ThreadNotDone;         // cleared when second thread is done. 

extern volatile uint regSelect;             // register select on latched data (set by interrupt handler). bit 4 set high on reads

extern volatile bool cursorOn;              // cursor control, status of cursor (true if currently displayed)
extern bool cursorEnabled;                  // show the cursor
extern unsigned char* cursor16;             // cursor 8x16  
extern unsigned char* cursor8;              // cursor 8x8  
extern unsigned char* cursor;               // cursor pointer (either to cursor8 or cursor16)  

extern uint bufferClearRowStartWord;        // buffer clear function starting word (first word of character row)
extern uint bufferClearWordUBound;          // buffer clear function < upper bound 


// --------------------------------------- function declarations ------------------------------------

// config commnds common to all PIO init routines
void Standard_PIO_Config(PIO pio, uint sm, uint offset, uint pin, uint num_pins, pio_sm_config* config_ptr);

// initializations for PIO state machines
void HSYNC_Pin_Init(PIO pio, uint sm, uint offset, uint pin); 
void VSYNC_Pin_Init(PIO pio, uint sm, uint offset, uint pin); 
void PIXEL_Pin_Init(PIO pio, uint sm, uint offset, uint pin); 

// initialization for DMA
void PIXEL_DMA_Init(PIO pio, uint sm); 

// cursor 
void CURSOR_Init(void);

// interrupt handlers
void INDATA_IRQ_handler(void);
void PIXEL_DMA_handler(void);
void CURSOR_blink_handler(void);

// frame buffer functions  
void FrameBuffer_WriteAll(void);
void FrameBuffer_WriteNextChar(uint8_t charData, bool increment);
void set_nextCharBufferWord(void);
void charArraysInit(void);
void ClearBufferThread(void);
void InvertBufferThread(void); 
void charBufferWrite(void);
void saveCursor(void); 
void restoreCursor(void); 
void clearCurLineRange(uint startCol, uint colUBound);

 // cursor commands
void cmdLineFeed(void);                
void cmdCarriageReturn(void);
void cmdBackspace(void);
void cmdTab(void);
void cmdHome (void);

// output control
void cmdAutoIncChar(void);              
void cmdSetCursor(void);
void cmdUse8x16Charset(void);
void cmdInvertCharPix(void); 
void cmdAutoWrap (void);

// delete commands
void cmdClearCursorRight(void);         
void cmdClearCursorLeft(void);
void cmdClearLine(void);
void cmdClearDown(void);
void cmdDelete(void);
void cmdClearScreen(void);

// smooth scroll commands
void cmdRotateDown(void);
void cmdClearUp (void); 

void dummy(void);



#endif // VGAGLOBALS_H

