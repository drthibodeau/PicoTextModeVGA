#include "VGAglobals.h"


// display controller command functions

void cmdLineFeed(void) {

    uint bottomRow; 
    if(charRowOffset>0) bottomRow = charRowOffset-1;
    else bottomRow = ROWS-1;
     
    if (nextCharRow==bottomRow) { 
       
        uint memClearStart = COLS*charRowOffset;
        for(int i=0; i<COLS; i++) charMem[memClearStart+i]=0;                       // clear new bottom line
     
        charRowOffset++;
        if(charRowOffset==ROWS) charRowOffset=0;    
                
        FrameBuffer_WriteAll();
    } 
    else {
        nextCharBufferWord+=20*CHRHEIGHT;                                           // 20 bytes per pixel row
        if (nextCharBufferWord>=FRMBUFFSIZE) nextCharBufferWord-=FRMBUFFSIZE;
    }

    nextCharRow++;
    if(nextCharRow==ROWS) nextCharRow=0;
    
}

void cmdBackspace (void) {  
    
    if( nextCharRow==charRowOffset && nextCharCol==0);
    else {
        nextCharWordOffset++;
        if (nextCharWordOffset>3) {
            nextCharWordOffset=0;
            nextCharBufferWord--;                    
        }

        nextCharCol--;
        if (nextCharCol>COLS) {
            nextCharCol=COLS-1;
            nextCharRow--;
            nextCharBufferWord = nextCharBufferWord - 20*(CHRHEIGHT-1);                                         // jump to top of previous line
            if (nextCharBufferWord>FRMBUFFSIZE) nextCharBufferWord = FRMBUFFSIZE - 1 - 20*(CHRHEIGHT-1);  
        }
        if (nextCharRow>ROWS) nextCharRow=ROWS-1;
        FrameBuffer_WriteNextChar(0, false);
    }
}

void cmdCarriageReturn (void) {  
    nextCharCol = 0;    
    uint BufferCol = nextCharBufferWord % 20;       // 20 bytes per row
    nextCharBufferWord -= BufferCol;    
    nextCharWordOffset = 3;
}

void cmdTab (void) {           
    if(nextCharCol<(COLS-4)) {                      // fixed tab stop at start of each word. Go to next word if not at last word of row 
        nextCharBufferWord++;  
        nextCharCol = (nextCharCol/4)*4 + 4;
        nextCharWordOffset=3;
    }
}

void cmdAutoIncChar (void) { 
    ChrIncrement = true;      
}

void cmdAutoWrap (void) { 
    lineWrap = true;      
}

void cmdSetCursor(void) {
   cursorEnabled = true;
   irq_set_enabled(CURSORIRQ, true);
}

void cmdUse8x16Charset(void) {   
    
    TEXTCHARS = CHRS8x16;
    ROWS = 30;
    CHRHEIGHT = 16;
    cursor = cursor16; 
            
    nextCharCol = 0;        // reset indexes and screen
    nextCharRow = 0;
    charRowOffset = 0;         
    pixOffset = 0;             
    set_nextCharBufferWord();    
    FrameBuffer_WriteAll();
}

void cmdInvertCharPix(void) {
        
    ThreadNotDone = true; 
    multicore_launch_core1(InvertBufferThread);   
    
    for(int i=0; i<256; i++) 
        for(int j=0; j<CHRHEIGHT; j++) TEXTCHARS[i][j] = ~TEXTCHARS[i][j];        
    
    while(ThreadNotDone);                                                           // wait for thread to finish 
    multicore_reset_core1();                                                        // reset the core for next launch
}

void cmdHome (void) {   
    nextCharRow = charRowOffset;    
    nextCharCol = 0;    
    nextCharBufferWord = 0;
    nextCharWordOffset = 3;
}

void cmdClearScreen (void) {
  
    ThreadNotDone = true; 
    bufferClearRowStartWord = 0;                                    // clear entire buffer
    bufferClearWordUBound = FRMBUFFSIZE; 
    multicore_launch_core1(ClearBufferThread);   
    
    for(int j=0; j<CHRMEMSIZE; j++) {
        charMem[j] = 0;        
    }

    while(ThreadNotDone);                                           // wait for thread to finish 
    multicore_reset_core1();                                        // reset the core for next launch
                         
}

void cmdDelete (void) {  
    FrameBuffer_WriteNextChar(0, false);
}

void cmdClearCursorRight(void) {     
    clearCurLineRange(nextCharCol,COLS); 
}

void cmdClearCursorLeft(void) {
    clearCurLineRange(0,nextCharCol+1); 
}

void cmdClearLine(void) {
    clearCurLineRange(0,COLS); 
}

void cmdClearDown (void) {
  
    ThreadNotDone = true;    
    cmdClearCursorRight();                                                              // clear remainder of line
    saveCursor();

    uint bottomRow = charRowOffset-1;
    if(bottomRow>ROWS) bottomRow = ROWS-1;                                              // above calc only works for offset > 0. Underflows when offset = 0; 

    if(nextCharRow!=bottomRow) {                                                        // if not at bottom start clearing remaining lines
        
        cmdLineFeed();                                                                  // move to start of next line
        cmdCarriageReturn();

        bufferClearRowStartWord = nextCharBufferWord;                                   // clear remaining frame buffer
        bufferClearWordUBound = FRMBUFFSIZE;                     
        multicore_launch_core1(ClearBufferThread); 

        for(int j=COLS*nextCharRow+nextCharCol; j<CHRMEMSIZE; j++) charMem[j] = 0;     // clear remaining char memory 
    }
    else ThreadNotDone=false;
    
    while(ThreadNotDone);                                                               // wait for thread to finish     
    restoreCursor();
    multicore_reset_core1();                                                            // reset the core for next launch          
}

void cmdClearUp (void) {
  
    ThreadNotDone = true;    
    cmdClearCursorLeft();                                                              // clear current line to cursor
    saveCursor();

    if(nextCharRow!=charRowOffset) {                                                    // not at top row then start clearing remaining lines
        
        cmdCarriageReturn();                                                            // move to start of row to get upper bound

        bufferClearRowStartWord = 0;                                                    // clear top to current frame buffer word
        bufferClearWordUBound = nextCharBufferWord;                     
        multicore_launch_core1(ClearBufferThread); 

        for(int j=COLS*charRowOffset; j<COLS*nextCharRow; j++) charMem[j] = 0;          // clear char rows above  
    }
    else ThreadNotDone=false;
    
    while(ThreadNotDone);                                                               // wait for thread to finish     
    restoreCursor();
    multicore_reset_core1();                                                            // reset the core for next launch          
}

void cmdRotateDown(void) {
    
    pixOffset++;
    nextCharBufferWord -=20;
    
    if(pixOffset==CHRHEIGHT) {
        charRowOffset++;
        pixOffset = 0;
    }    
    
    if(charRowOffset==ROWS) charRowOffset=0; 
    if(nextCharBufferWord>FRMBUFFSIZE) nextCharBufferWord = FRMBUFFSIZE - 20 + 4*nextCharCol/4;         // wrap around to bottom 
    
    FrameBuffer_WriteAll();
    
};
