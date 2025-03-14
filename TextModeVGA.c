#include "VGAglobals.h"


#include "textchars.c"                  // 2D arrays of fonts (8x8 and 8x16). 1st dimension corresponds to ascii/cp437 code, 2nd dimension is pixel row
                                        // character 0 is the null character (same as ascii). Used for screen background 

unsigned char** TEXTCHARS;              // pointer that can switch between 8x16 and 8x8 char arrays
unsigned char** CHRS8x16;               // pointer for dynamic memory allocation for 8x16 fonts (can be used for custom fonts)
unsigned char** CHRS8x8;                // pointer for dynamic memory allocation for 8x8 fonts (can be used for custom fonts)

uint ROWS;                              // screen rows
uint CHRHEIGHT;                         // pixel height (8 or 16)

unsigned char charMem[CHRMEMSIZE];      // allocating enough space for 80 cols x 60 rows = 4800 bytes 
uint frameBuffer[FRMBUFFSIZE];          // 640x480 pixels as 9600 32-bit values

PIO pioSYNC = pio0;                     // sync signal PIO 
uint smVSYNC;                           // vsync state machines

uint dmaPIXL;                           // dma channel for writing pixel data

uint VSYNC_code_cnt = 0;                // for updating VSYNC PIOASM loop counters 

uint nextCharCol = 0;                   // next free char location (cursor position)
uint nextCharRow = 0;

uint charRowOffset = 0;                 // row offset for line scrolling
uint pixOffset = 0;                     // row offset for pixel scrolling.  

uint nextCharBufferWord;                // frame buffer word that contains pixel row 0 for next character. set by set_nextCharBufferWord
int nextCharWordOffset;                 // the offset within frame buffer word (little endian format). set by set_nextCharBufferWord 

uint saveCharCol;                       // saved cursor position 
uint saveCharRow;
uint saveCharBufferWord;       
int saveCharWordOffset;         

bool ChrIncrement = true;               // auto increment to next character  
bool lineWrap = true;                   // auto linefeed if end of line                    

volatile bool ThreadNotDone;            // cleared when second thread is done. 

volatile uint regSelect = 0;            // register select on latched data (set by interrupt handler). bit 4 set high on reads

typedef struct {                        // input buffer (circular), modified by ISR
    volatile int buffer[256];
    volatile uint8_t write;             // write index
    volatile uint8_t read;              //  read index    
} InputBuffer;

InputBuffer InBuffer; 

volatile bool cursorOn = false;         // status of cursor (true if currently displayed)
bool cursorEnabled = true;              // visible 
unsigned char* cursor16;                // cursor 8x16  
unsigned char* cursor8;                 // cursor 8x8  
unsigned char* cursor;                  // cursor pointer (either to cursor8 or cursor16)  


int main() {

    set_sys_clock_hz(SYSCLK, true);                 // adjust system clock to give better VGA timings 
    charArraysInit();                               // init 8x8 and 8x16 character sets
    CURSOR_Init();                                  // init cursor
    
    InBuffer.write = 0;                             // setup the input buffer
    InBuffer.read = 0; 
    
    void (*cmdList[0xff])();                        // pointers to command functions

    for(int j=0; j<255; j++) cmdList[j]=&dummy;     // load some dummy "nop" functions. Prevents system crash on bad command input
    
    cmdList[0x00] = &cmdAutoIncChar;                // setting command function pointers 
    cmdList[0x01] = &cmdSetCursor; 
    cmdList[0x02] = &cmdUse8x16Charset;  
    cmdList[0x03] = &cmdInvertCharPix;   
    cmdList[0x04] = &cmdAutoWrap;   
    cmdList[0x05] = &cmdHome;   
    cmdList[0x08] = &cmdBackspace;
    cmdList[0x0A] = &cmdLineFeed;   
      
    cmdList[0x0D] = &cmdCarriageReturn;       
    cmdList[0x09] = &cmdTab;    

    cmdList[0x7F] = &cmdDelete;
    cmdList[0x0C] = &cmdClearScreen;  
    cmdList[0x10] = &cmdClearCursorRight;
    cmdList[0x11] = &cmdClearCursorLeft;
    cmdList[0x12] = &cmdClearLine;
    cmdList[0x20] = &cmdClearDown;

    set_nextCharBufferWord();                       // set the pointer to frame buffer word for next character and offset for byte level data writing 


    // ------------------------TEST CHARS
   
    int i = 0;
    for(int j=0; j<ROWS*COLS; j++) {
        charMem[j] = i;
        i++;
        if (i==256) i=0;
    }
    // init test frame buffer
    FrameBuffer_WriteAll();

    // ------------------------- END TEST   

    
    // set GPIO pins for data bus and register select as inputs 
    uint pinMask = (1u<<D0)|(1u<<D1)|(1u<<D2)|(1u<<D3)|(1u<<D4)|(1u<<D5)|(1u<<D6)|(1u<<D7)|(1u<<R0)|(1u<<R1)|(1u<<R2);  
    gpio_init_mask(pinMask);               
    gpio_set_dir_in_masked(pinMask);  
    
    gpio_init(CS_ENABLE);                                               // set Chip Select pin as input
    gpio_set_dir(CS_ENABLE,false);

    volatile uint8_t latchedData = 0;                                   // input from data bus (set by interrupt handler)

    gpio_init(CLK);                                                     // set Clock pin as input with falling edge IRQ triggger  
    gpio_set_dir(CLK,false);
    gpio_set_irq_enabled(CLK,GPIO_IRQ_EDGE_FALL,true);
    irq_set_exclusive_handler(IO_IRQ_BANK0,DATA_IRQ_handler);
    irq_set_enabled(IO_IRQ_BANK0,true);

    gpio_init(RW_ENABLE);                                               // set RW pin as input
    gpio_set_dir(RW_ENABLE,false);
 
    PIO pioPIXL = pio1;                                                 // PIXEL PIO setup
    uint offsetPIXL = pio_add_program(pioPIXL,&PIXEL_program);    
    uint smPIXL = pio_claim_unused_sm(pioPIXL,true);
    PIXEL_Pin_Init(pioPIXL, smPIXL, offsetPIXL, PIXEL_PIN);

    uint offsetSYNC = pio_add_program(pioSYNC,&SYNC_program);           // SYNC PIO setup
    
    smVSYNC = pio_claim_unused_sm(pioSYNC,true);
    VSYNC_Pin_Init(pioSYNC, smVSYNC, offsetSYNC, VSYNC_PIN);            // VSYNC
    
    uint smHSYNC = pio_claim_unused_sm(pioSYNC,true);
    HSYNC_Pin_Init(pioSYNC, smHSYNC, offsetSYNC, HSYNC_PIN);            // HSYNC
           
    pio_enable_sm_mask_in_sync(pioPIXL, (1<<smPIXL) );                  // enable pixel state machine first and wait for gpio signal from sync program 

    PIXEL_DMA_Init(pioPIXL,smPIXL);                                     // initialize DMA to fill PIXEL tx fifo
    
    pio_enable_sm_mask_in_sync(pioSYNC, (1<<smVSYNC) | (1<<smHSYNC) );  // enabling sync signals
      

    // TEST !!!!!
    stdio_init_all();                                                   // Initialize standard input/output (for USB serial) 


    while(true) {
    
        if((InBuffer.write-InBuffer.read)>0) {                                                  // input buffer not empty
            
            InBuffer.read++;
            regSelect = InBuffer.buffer[InBuffer.read];
            InBuffer.read++;
            latchedData = InBuffer.buffer[InBuffer.read];  
            
            if (cursorEnabled) {
                irq_set_enabled(CURSORIRQ, false);                                              // pause cursor blink 
                charBufferWrite();                                                              // restore char (remove the cursor if visible)
            }

            if(regSelect==CMDREG_W || regSelect==CMDREG_R) (*cmdList[latchedData])();           // process command  
            if(regSelect==CHRREG_W)  FrameBuffer_WriteNextChar(latchedData,ChrIncrement);       // or process new character

            if (cursorEnabled) {
                irq_set_enabled(CURSORIRQ, true);
                CURSOR_blink_handler();                                                         // restart cursor 
            }

            // TEST !!!!!!!
            printf("data: %d  register: %d \n",latchedData, regSelect);            

        }        
    }
}




// TEST !!! FIX roate down one pixel command
void RotateDown(void) {
    
    pixOffset++;
    
    if(pixOffset==CHRHEIGHT) {
        charRowOffset++;
        pixOffset = 0;
    }    
    if(charRowOffset==ROWS) charRowOffset=0;    
};



// threads:
void InvertBufferThread(void) {
    for(int j=0; j<FRMBUFFSIZE; j++) frameBuffer[j] = ~frameBuffer[j];
   ThreadNotDone = false;  
}

void ClearBufferThread(void) {

    uint pixRow = pixOffset;                                // starting pix offset for first row (0-15)
    uint charCol = 0;                       
    uint8_t* frameBytePtr = (uint8_t*) &frameBuffer[0];     // create a byte pointer for frame buffer. Casting as uint8_t allows byte level manipulation;
    
    for(int j=0; j<FRMBUFFSIZE; j++) {

        *(frameBytePtr+3) = TEXTCHARS[0][pixRow];           // fill buffer with null character with pixel offset 
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

   ThreadNotDone = false;    
} 


// FIX !!!!!!
void ClearBuffDownFromWordThread(void) {

} 


// Frame Buffer functions:

void FrameBuffer_WriteAll() {
    
    uint pixRow = pixOffset;                                            // starting pix offset for first row (0-15)
    uint charRow = charRowOffset;                                       // starting screen row offset for first character row (0 to 29)
    uint charCol = 0;                       
    uint cmOffset;                                                      // offset for char memory 
                    
    uint8_t* frameBytePtr = (uint8_t*) &frameBuffer[0];                 // create a byte pointer for frame buffer. Casting as uint8_t allows byte level manipulation;
    
    for(int j=0; j<FRMBUFFSIZE; j++) {

        cmOffset = COLS*charRow + charCol; 
         
        *(frameBytePtr+3) = TEXTCHARS[charMem[cmOffset]][pixRow];       // little endian. 
        *(frameBytePtr+2) = TEXTCHARS[charMem[cmOffset+1]][pixRow];
        *(frameBytePtr+1) = TEXTCHARS[charMem[cmOffset+2]][pixRow];
        *(frameBytePtr+0) = TEXTCHARS[charMem[cmOffset+3]][pixRow];

        frameBytePtr += 4;
        charCol += 4;

        if (charCol==COLS) {                                            // inc pixel/row if end of line/pixel
            charCol=0;
            pixRow++;
            if (pixRow==CHRHEIGHT) {
                pixRow=0;
                charRow++;
            }
            if (charRow==ROWS) charRow=0;
        }
    }
}

void FrameBuffer_WriteNextChar(uint8_t charData, bool increment) {
    
    charMem[COLS*nextCharRow+nextCharCol] = charData;                               // character memory update
    charBufferWrite();                                                              // write to frame buffer
      
    if(increment == true) {

        nextCharWordOffset--;                                                       // move byte offset
        if (nextCharWordOffset<0) {
            nextCharWordOffset=3;
            nextCharBufferWord++;                    
        }
        
        nextCharCol++;
        if (nextCharCol==COLS) {                                                    // if end of line go to next line if linewrap==true
            
            if(lineWrap) {
                
                nextCharCol=0;                
                uint bottomRow = charRowOffset-1;
                if(bottomRow>ROWS) bottomRow = ROWS-1;                              // above calc only works for offset>0. Underflows when offset = 0; 
                
                if(nextCharRow==bottomRow) {
                    cmdLineFeed();                                                  // new line if at bottm. cmdLineFeed does the char row increment 
                    nextCharBufferWord -= 20;                                       // back to start of line after line feed                 
                }
                else {                                                              
                    nextCharBufferWord += 20*(CHRHEIGHT-1);                         // jump to top of next line without a line feed (20 words per line) 
                    if (nextCharBufferWord>=FRMBUFFSIZE) nextCharBufferWord=0;      // wrap around for frame buffer and row index             
                    nextCharRow++;
                    if (nextCharRow==ROWS) nextCharRow=0;
                }               
            }
            else {   
                nextCharCol--;                                                      // undo the increment if no line wrap
                nextCharBufferWord--;
                nextCharWordOffset=0;
            }
        }
    }
}

void charBufferWrite(void) {

    uint cmOffset = COLS*nextCharRow + nextCharCol;                                                                                 // offset for charMem     
    
    uint8_t* frameBytePtr = (uint8_t*) &frameBuffer[nextCharBufferWord];                                                            // byte pointer for frame buffer at current word. 

    for(uint pixRow=0; pixRow<CHRHEIGHT; pixRow++) {                                                                                // write character
        *(frameBytePtr+nextCharWordOffset) = TEXTCHARS[charMem[cmOffset]][pixRow];
        frameBytePtr+=COLS;                                                                                                         // 80 bytes per row        
        if (frameBytePtr>=(uint8_t*) &frameBuffer[FRMBUFFSIZE]) frameBytePtr = (uint8_t*) &frameBuffer[0] + 4*(nextCharCol/4);      // wrap around to zero and inc to next word
    }
}

void set_nextCharBufferWord(void) {
    // calculate frame buffer word that contains pixel row 0 for next character. Called at startup
    // set frame buffer word for next character
    uint bufferCol = nextCharCol/4;                                                               // updating 4 bytes at a time
      
    uint rowOffset;
    if(nextCharRow<charRowOffset) rowOffset = ROWS-(charRowOffset-nextCharRow); 
    else rowOffset = nextCharRow-charRowOffset;
                                                                         
    if(pixOffset>0 && rowOffset==0) nextCharBufferWord = FRMBUFFSIZE - 20*pixOffset + bufferCol;     // need to adjust first row for pixel offset
    else nextCharBufferWord = rowOffset*20*CHRHEIGHT - 20*pixOffset + bufferCol;                              // 320 = 20 bytes per pix row x 16 pix per char row. 4 bytes per block  
   
    // set offset within the word to write the next char byte
    nextCharWordOffset = 3 - nextCharCol % 4;       
}

void saveCursor(void) {
    saveCharBufferWord = nextCharBufferWord;          
    saveCharWordOffset = nextCharWordOffset;
    saveCharCol = nextCharCol;
    saveCharRow = nextCharRow;
}

void restoreCursor(void) {
    nextCharBufferWord = saveCharBufferWord;          
    nextCharWordOffset = saveCharWordOffset;
    nextCharCol = saveCharCol;
    nextCharRow = saveCharRow;
}

void clearCurLineRange(uint startCol, uint colUBound) {

    saveCursor();    
    
    nextCharBufferWord = nextCharBufferWord - nextCharCol/4  + startCol/4;          //  same as diff of startCol/4 - nextCharCol/4 but does not go negative    
    nextCharWordOffset = 3 - startCol % 4;  
    nextCharCol = startCol;
    
    bool tempLineWrap = lineWrap;                                                  // pause line wrapping if set 
    lineWrap = false;
    
    for(; startCol<colUBound; startCol++) FrameBuffer_WriteNextChar(0x00,true);    // write nulls from new cursor position to ipper bound
    
    restoreCursor();
    lineWrap = tempLineWrap;    
}


// Interrupt handlers:

void CURSOR_blink_handler(void) {    
    
    hw_clear_bits(&timer_hw->intr, 1u << CURSORIRQ);                                                                                    // Clear the timer interrupt 0 flag 
    
    // toggle cursor 
    if(cursorOn) {
        charBufferWrite();                                                                                                              // write the char at cursor position
        cursorOn = false;
    }
    else {                                                                                                                              // else: show cursor
        uint cmOffset = COLS*nextCharRow + nextCharCol;                                                                                 // offset for charMem
        uint8_t* frameBytePtr = (uint8_t*) &frameBuffer[nextCharBufferWord];                                                            // byte pointer at current word 

        for(uint pixRow=0; pixRow<CHRHEIGHT; pixRow++) {                                                                                // write char xor'd with cursor  
            *(frameBytePtr+nextCharWordOffset) = TEXTCHARS[charMem[cmOffset]][pixRow] ^ cursor[pixRow] ;                          
            frameBytePtr+=COLS;                                                                                                         // 80 bytes per row        
            if (frameBytePtr>=(uint8_t*) &frameBuffer[FRMBUFFSIZE]) frameBytePtr = (uint8_t*) &frameBuffer[0] + 4*(nextCharCol/4);      // wrap around to zero and inc to next word
        } 
        cursorOn = true;
    }

    timer_hw->alarm[CURSORIRQ] = timer_hw->timerawl + CURSORSPD;                                                                         // restart timer to current time + cursor speed
}

void PIXEL_DMA_handler(void) { 
    dma_hw->ints0 = 1u << dmaPIXL;                              // clear interrupt. ints0 has bits set by any active channel that triggered interrupt
   dma_channel_set_read_addr(dmaPIXL, frameBuffer, true);       // point to frame buffer start address and enable (true)
}

void DATA_IRQ_handler(void) {

    gpio_acknowledge_irq(CLK,GPIO_IRQ_EDGE_FALL);

    if (gpio_get(CS_ENABLE)==0) {                                                                                               // write to input buffer register select followed by data       
        InBuffer.write++;  
        InBuffer.buffer[InBuffer.write] = (gpio_get(RW_ENABLE)<<4) | (gpio_get(R2)<<2) | (gpio_get(R1)<<1) | gpio_get(R0);      // Register. bit 4 set to RW signal
        InBuffer.write++;  
        InBuffer.buffer[InBuffer.write] = (gpio_get(D7)<<7) | (gpio_get(D6)<<6) | (gpio_get(D5)<<5) | (gpio_get(D4)<<4) | (gpio_get(D3)<<3) | (gpio_get(D2)<<2) | (gpio_get(D1)<<1) | gpio_get(D0);  
    }

}

void VSYNC_IRQ_handler() {
    
    // interrupt triggered one time in PIOASM    
   
    irq_set_enabled(PIO0_IRQ_0, false);                                                 // disable interrupt 
    pio_interrupt_clear(pioSYNC,0);                                                     // clear PIO interrupt 
        
    while (!pio_sm_is_tx_fifo_full(pioSYNC, smVSYNC)) {                                 // TX buffer filled until no space in buffer
        
        if (VSYNC_code_cnt==0) pioSYNC->txf[smVSYNC] = V_ACTIVE_LINES-1;
        else if (VSYNC_code_cnt==1) pioSYNC->txf[smVSYNC] = V_FRONTP_LINES-1;
        else pioSYNC->txf[smVSYNC] = V_BACKP_LINES-1;
        
        VSYNC_code_cnt++; 
        if (VSYNC_code_cnt==3) VSYNC_code_cnt=0;
    }

     irq_set_enabled(PIO0_IRQ_0, true);                                                 // enable interrupt
}


// Config functions: 

void PIXEL_DMA_Init(PIO pio, uint sm) {
    
    dmaPIXL = dma_claim_unused_channel(true);                                               // claim channel for writing frame buffer data to pio  
    dma_channel_config dmaConfig = dma_channel_get_default_config(dmaPIXL);
    channel_config_set_transfer_data_size(&dmaConfig,DMA_SIZE_32);      
    channel_config_set_read_increment(&dmaConfig,true);                                     // auto increment through frame buffer
    channel_config_set_write_increment(&dmaConfig,false);                                   // always writing to same TX buffer location
    channel_config_set_dreq(&dmaConfig,DREQ_PIO1_TX0+sm);                                   // flow control based on sm's TX buffer
    
    dma_channel_configure(dmaPIXL,&dmaConfig,&pio->txf[sm],NULL,FRMBUFFSIZE,false);          // init DMA. Interrupt handler will set read address and start transfer

    dma_channel_set_irq0_enabled(dmaPIXL, true);                                             // set DMA interrupt (DMA_IRQ_0). Interrupt occurs on transfer complete 
    irq_set_exclusive_handler(DMA_IRQ_0, PIXEL_DMA_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    PIXEL_DMA_handler();                                                                    // manual call to handler to start DMA

}

void PIXEL_Pin_Init(PIO pio, uint sm, uint offset, uint pin) {
           
    // xxx_get_default_config() function is a helper function generated by the PIO assembler.  
    pio_sm_config config = PIXEL_program_get_default_config(offset);                                            // PIO config structure.     
    Standard_PIO_Config(pio, sm, offset, pin, 1, &config);                                                      // standard PIO config options
    pio_sm_init(pio, sm, offset, &config);                                                                      // init state machine with a specific program (offset) and config

    // auto pull with 32 bit threshold (BIT 17) and shift out to the right (bit 19 == 0)
    // join RX to TX to double buffer size *BIT 30)
    pio->sm[sm].shiftctrl = (1u << PIO_SM0_SHIFTCTRL_FJOIN_TX_LSB) | ( 1u << PIO_SM0_SHIFTCTRL_AUTOPULL_LSB) | (0u << PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_LSB) ; 
  
    // shift out direction is to left but a full 32 bit pull from fifo does not have bits mirrored 
    pio->txf[sm] = HORZ_PIXELS-1;                                                                               // load hozixontal pixel count (less one for zeroth interation)
                                                                                
}

void HSYNC_Pin_Init(PIO pio, uint sm, uint offset, uint pin) {

    // xxx_get_default_config() function is a helper function generated by the PIO assembler.  
    pio_sm_config config = SYNC_program_get_default_config(offset);                                             // PIO config structure. 
    Standard_PIO_Config(pio, sm, offset, pin, 1, &config);                                                      // standard PIO config options

    float div = 4;
    sm_config_set_clkdiv(&config, div);
 
    pio_sm_init(pio, sm, offset+SYNC_offset_hsync_start, &config);                                              // init state machine with a specific program (offset) and config.
        
}

void VSYNC_Pin_Init(PIO pio, uint sm, uint offset, uint pin) {
           
    // xxx_get_default_config() function is a helper function generated by the PIO assembler.  
    pio_sm_config config = SYNC_program_get_default_config(offset);                                             // PIO config structure. 
    Standard_PIO_Config(pio, sm, offset, pin, 2, &config);                                                      // standard PIO config options

    float div = 4;
    sm_config_set_clkdiv(&config, div);

    // enabling system PIOx_IRQ_0 source to be PIO's interrupt
    // pis_interrupt0 is PIOs interrupt 0
    // x will be based on pio's number (0 in this case)
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, VSYNC_IRQ_handler);                                                   // adding handler for PIO0_IRQ_0
    irq_set_enabled(PIO0_IRQ_0, true);                                                                          // enable PIO0_IRQ_0
   
    pio_sm_init(pio, sm, offset+SYNC_offset_vsync_start, &config);                                              // init state machine with a specific program (offset) and config.

    pio->sm[sm].shiftctrl = (1u << PIO_SM0_SHIFTCTRL_AUTOPULL_LSB);                                             // auto pull with 32 bit threshold

    pio->txf[sm] = V_ACTIVE_LINES-1;                                                                            // line counts for vertical regions -1 for the 0th iteration in PIOASM
    pio->txf[sm] = V_FRONTP_LINES-1;  
    pio->txf[sm] = V_BACKP_LINES-1;

}

void Standard_PIO_Config(PIO pio, uint sm, uint offset, uint pin, uint num_pins, pio_sm_config* config_ptr) {

        for(int i=0; i<num_pins; i++) pio_gpio_init(pio, pin+i);               // connects GPIO "pin" to the PIO
        pio_sm_set_consecutive_pindirs(pio, sm, pin, num_pins, true);          // set one or more consecutive pin directions
        sm_config_set_set_pins(config_ptr, pin, num_pins);                     // modifies state machine config to specify which pin(s) out and set instructions affect.
        sm_config_set_out_pins(config_ptr, pin, num_pins);           
   
}
 
void charArraysInit () {
   
    CHRS8x8 = (unsigned char**) malloc(256 * sizeof(unsigned char*));                                        // memory allcoation for 256x8 array
    for (int i = 0; i < 256; i++) CHRS8x8[i] = (unsigned char*)malloc(8 * sizeof(unsigned char));
        
    for (int i = 0; i < 256; i++)                                                                            // copy default 8x8 chars
        for (int j = 0; j < 8; j++) CHRS8x8[i][j] = TEXTCHARS8[i][j];

    CHRS8x16 = (unsigned char**) malloc(256 * sizeof(unsigned char*));                                       // memory allcoation for 256x16 array
    for (int i = 0; i < 256; i++) CHRS8x16[i] = (unsigned char*)malloc(16 * sizeof(unsigned char));
        
    for (int i = 0; i < 256; i++)                                                                            // copy default 8x16 chars 
        for (int j = 0; j < 16; j++) CHRS8x16[i][j] = TEXTCHARS16[i][j];

    TEXTCHARS = CHRS8x16;                                                                                    // default character set
    ROWS = 30;
    CHRHEIGHT = 16;

}

void CURSOR_Init() {
    
    cursor8 = (unsigned char*) malloc(8 * sizeof(unsigned char));           // 8x8 cursor setup 
    for (int j = 0; j < 8; j++) cursor8[j] = TEXTCHARS8[95][j];

    cursor16 = (unsigned char*) malloc(16 * sizeof(unsigned char));         // 8x16 cursor setup 
    for (int j = 0; j < 16; j++) cursor16[j] = TEXTCHARS16[95][j];

    cursor = cursor16;                                                      // default cursor 
        
    timer_hw->alarm[CURSORIRQ] = timer_hw->timerawl + CURSORSPD;             // set time and alarm for cursor irq
    timer_hw->inte = 1u << CURSORIRQ; 
    irq_set_exclusive_handler(CURSORIRQ, CURSOR_blink_handler);
    irq_set_enabled(CURSORIRQ, true);
}


// dummy function to prevent crashes from invalid commands
void dummy(void) {};



   