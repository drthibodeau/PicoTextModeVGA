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
    if(regSelect==CMDREG_R) ChrIncrement = false; else ChrIncrement = true;      
}

void cmdAutoWrap (void) { 
    if(regSelect==CMDREG_R) lineWrap = false; else lineWrap = true;      
}

void cmdSetCursor(void) {
    if(regSelect==CMDREG_W) {
        cursorEnabled = true;
        irq_set_enabled(CURSORIRQ, true);
    }
    else {     
        cursorEnabled = false;
        irq_set_enabled(CURSORIRQ, false);
    }
}

void cmdUse8x16Charset(void) {
    if (regSelect==CMDREG_W) {       
        TEXTCHARS = CHRS8x16;
        ROWS = 30;
        CHRHEIGHT = 16;
        cursor = cursor16; 
    }
    else {
        TEXTCHARS = CHRS8x8;
        ROWS = 60;
        CHRHEIGHT = 8;
        cursor = cursor8;
    }
    
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

        for(int j=COLS*nextCharRow+nextCharCol; j<CHRMEMSIZE; j++) charMem[j] = 0; 

        uint pixRow = pixOffset;                                                        // starting pix offset for first row (0-15)        
        uint charCol = 0;                       
        uint8_t* frameBytePtr = (uint8_t*) &frameBuffer[nextCharBufferWord];            // byte pointer for frame buffer 
    
        for(int j=nextCharBufferWord; j<FRMBUFFSIZE; j++) {

            *(frameBytePtr+3) = TEXTCHARS[0][pixRow];                                   // fill buffer with null character with pixel offset 
            *(frameBytePtr+2) = TEXTCHARS[0][pixRow];
            *(frameBytePtr+1) = TEXTCHARS[0][pixRow];
            *(frameBytePtr+0) = TEXTCHARS[0][pixRow];

            frameBytePtr += 4;
            charCol += 4;
            if (charCol==COLS) {
                charCol=0;
                pixRow++;
                if (pixRow==CHRHEIGHT) pixRow=0;               
            }
        }
    }

   // while(ThreadNotDone);        // wait for thread to finish     
    restoreCursor();
   // multicore_reset_core1();     // reset the core for next launch          
}


