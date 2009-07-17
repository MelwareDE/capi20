/*-------------------------- SPORT FUNCTIONS ----------------------------*/

/* Init serial port but don't enable just yet, we need to set up DMA first 

   Note SPORT0 is used for the BF533 STAMP and SPORT1 for the BF537 due to
   the physical alignment of the 4fx cards on the STAMP boards. 

   Note that a better way to write init code that works for both sports is
   in uClinux-dist /linux-2.6.x/sound/blackfin/bf53x_sport.c.  A
   structure is set up with the SPORT register addresses referenced to
   the base ptr of the structure.  This means one function can be used
   to init both SPORTs, just by changing the base addr of the ptr. 

	
   This version of bfsi is based on the David Rowe <david@rowetel.com> code
   

*/


//===================================================================================================================
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/bfin5xx_spi.h>
#include <asm/blackfin.h>
#include <linux/delay.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/version.h>


#include "bfsi.h"


//====================================================================================================================
//Static variables
static u8 *iTxBuffer1;					 //See the init_dma_wc() for the structure of the data in those buffers
static u8 *iRxBuffer1;					 


static u8 *fxsfxo_tx_buffer;				 //Buffer where the squeezed FXOFXS data is  placed as
static u8 *fxsfxo_rx_buffer;                             //the layers above insist on 8 channels (we have 9)




 
static int samples_per_chunk;				 //Specify how many frames we get per chunk. In our case 8 frames

static int init = 0;

static int bfsi_init = 0;

static int baud;

							 //there are two callbacks which can be installed
							 // for the FXS/FXO driver and for the FWFXS driver  
static void (*bfsi_fxs_fxo_callback)(u8 *read_samples, u8 *write_samples) = NULL;


#define BYTES_PER_FRAME	9				 //We have 18 bytes in our DMA buffer for each frame. Initially it was 16 but
//#define BYTES_PER_FRAME	8			 //we need 9th time slot as TI codec needs MFD =1 and we use MFD=0 because of 
							 //the FXS/FXO See init_dma_wc()
				 
//------------------------------------------------------//Debug variables
static int bfsi_debug = 1;                              //Debug is off by default.Use debug mode to see the bfsi proc entry

static int readchunk_first = 0;					
static int readchunk_second = 0;
static int readchunk_didntswap = 0;
static u8* lastreadchunk;

static int writechunk_first = 0;
static int writechunk_second = 0;
static int writechunk_didntswap = 0;
static u8* lastwritechunk;
static int softinterrupt = 0;


//------------------------------------------------------// previous and worst case number of cycles we took to process an interrupt
static u32 isr_cycles_last = 0;				
static u32 isr_cycles_worst = 0;
static u32 isr_cycles_average = 0; 			// scaled up by 2x 
static u32 echo_sams = 0;


//------------------------------------------------------//constants for isr cycle averaging 
#define TC    1024 					// time constant    
#define LTC   10   					// base 2 log of TC 



//------------------------------------------------------//structures to support the SOFT irq mechanism
 
struct soft_isr_entry {
	void(*soft_isr)(void *);	/* Soft ISR function */
	void(*soft_dma_isr)(u8 *, u8 *);/* Soft DMA buffer processing */
	void *data;			/* Soft ISR private context */
	int isr_enabled;		/* 0 = ISR processing disabled */
	int dma_enabled;		/* 0 = DMA processing disabled */
	unsigned int divider;		/* Call this ISR once every 'divider'*/
};

static spinlock_t isr_table_lock;
static struct soft_isr_entry soft_isr_table[] =
	{
		{(void(*)(void*))NULL, (void(*)(u8*, u8*))NULL, NULL, 0}
	};



//-------------------- SPI related functions ------------//
//#undef BFIN_SPI_DEBUG
#define BFIN_SPI_DEBUG 

#ifdef BFIN_SPI_DEBUG
#define PRINTK(args...) printk(args)
#else
#define PRINTK(args...)
#endif
/* 
   I found these macros from the bfin5xx_spi.c driver by Luke Yang 
   useful - thanks Luke :-) 
*/

#define DEFINE_SPI_REG(reg, off) \
static inline u16 read_##reg(void) \
            { return *(volatile unsigned short*)(SPI0_REGBASE + off); } \
static inline void write_##reg(u16 v) \
            {*(volatile unsigned short*)(SPI0_REGBASE + off) = v;\
             __builtin_bfin_ssync();}

DEFINE_SPI_REG(CTRL, 0x00)
DEFINE_SPI_REG(FLAG, 0x04)
DEFINE_SPI_REG(STAT, 0x08)
DEFINE_SPI_REG(TDBR, 0x0C)
DEFINE_SPI_REG(RDBR, 0x10)
DEFINE_SPI_REG(BAUD, 0x14)
DEFINE_SPI_REG(SHAW, 0x18)






/* Bit on port G to be used as nCSB */
#define SPI_NCSB_PG_BIT 2


//-------------------- SPI related variables ------------//
static u16 chip_select_mask = 0;





//====================================================================================================================
static void init_sport0(void)
{

	/* set up FSYNC and optionally SCLK using Blackfin Serial port */
  
	/* Note: internalclock option not working at this stage - Tx side
	   appears not to work, e.g. TFS pin never gets asserted. Not a 
	   huge problem as the BF internal clock is not at quite the
	   right frequency (re-crystal of STAMP probably required), so 
	   we really need an external clock anyway.  However it would
	   be nice to know why it doesn't work! */



  
  /* Register SPORTx_TCR1 ( relative details pls refer 12-12 of hardware reference ) 
	        TCKFE ( Clock Falling Edge Select )   ( Bit14 ) 
          	LTFS ( Bit11) :  0 - Active high TFS; 1 - Active low TFS
		ITFS ( Bit9 ) :  0 - External TFS used; 1 - Internal TFS used 
		TFSR: 0 - Dose not require TFS for every data word;  1 - Requires TFS for every data word
		TLSBIT: 0 - Transmit MSB first ;  1 - Transmit LSB first
		ITCLK: 0 - External transmit clock selected; 1 - Internal transmit clock selected
	


		Note that the codec actually use a-law codding but the encoded 8 bit sample is passed to zaptel directly


		transmit disable, internal tx clk, frame settings ignored in multichanel mode, 
	        MSB first, linear, no a/u-law codding, 
		
  */
  
  //bfin_write_SPORT0_TCR1(ITFS | ITCLK);
        printk("SPORT0 using external clock\n");
	bfin_write_SPORT0_TCR1(0);
	__builtin_bfin_ssync(); 
	//bfin_write_SPORT0_TCR1(0x2);
	

        //bfin_write_SPORT0_TCR2(0x0107);               // 8 bit word length, enable secondary SPORT0 transmit channel
	bfin_write_SPORT0_TCR2(0x7);                    // 8 bit word length, disable secondary SPORT0 transmit channel

	

	

	// Setup clock divider  

	//bfin_write_SPORT0_TCLKDIV(0x17);
	//bfin_write_SPORT0_RCLKDIV(0x17);





 	// Frame syn divider 
							
	bfin_write_SPORT0_RFSDIV(255); 			//2048kHz external clock is applied to both RCLK and TCLK  
	bfin_write_SPORT0_TFSDIV(255);			//we internaly divide it by (255+1) to get 8 kHz FSYNC 


	
	
	//
	//bfin_write_SPORT0_TCR1(bfin_read_SPORT0_TCR1() | TSPEN); //enable tx
	//

	
	// receive -------------------------------------
							// receive disable, MSB first, linear, no a/u-law codding. Note that the 
							// codec use a-law codding  but the codded 8 bit sample is passed to zaptel directly
                                                        // doesn't require RFS for each word, actuve high TFS, Early frame sync, 
							// sample by falling edge of clk
	//bfin_write_SPORT0_RCR1(IRFS);			// Internal frame the clock is external
	
	bfin_write_SPORT0_RCR1(0x0);
	__builtin_bfin_ssync(); 
	bfin_write_SPORT0_RCR2(0x7);                    // 8 bit word length, disable secondary SPORT0 receive channel


	
	bfin_write_SPORT0_MTCS0(0x000001ff);		// Enable MCM 9 transmit & receive channels
	bfin_write_SPORT0_MRCS0(0x000001ff);

	

	bfin_write_SPORT0_MCMC1(0x1000);		// MCM window size of 16 with 0 offset. Should be 8 multiple
	

        // Multichannel Frame mode enabled 4
	// Multi channel DMA RCV packing bit 3
	// Multichannel DMA TXT packing bit 2

	bfin_write_SPORT0_MCMC2(0x101c);		// 1 bit delay between FS pulse and first data bit,
        //bfin_write_SPORT0_MCMC2(0x001c);              // 0 bit delay between FS pulse and first data bit,
	
							

	__builtin_bfin_ssync(); 

	udelay(2000);

	

}

//====================================================================
static void init_dma_wc(void)
{

  //Structure of our frames is:(having 16 bytes in total)
  //
  //  Primary channel:
  //		byte    0                            31
  //     	      |bri1-1|bri1-2|bri2-1|bri2-1|bri3-1|bri3-2|bri4-1|bri4-2|fxS0|
  //
  //  Secondory channel:
  //
  //   		 Not usedbyte   0                              	                  31 
  //    	      |reg00|reg01|reg10|reg11|00|codec0|00|codec1|-- // |--|
  //
  // The codec is set in a-law codding so only the most significant
  // byte is used (the LSB is zero) 
  //
  //  In our DMA buffer we colect only the first 9 time slots from 
  //  both primary port channels 
  

  // Set up DMA3 to receive, map DMA3 to Sport0 RX  ----------------------------------
  bfin_write_DMA3_PERIPHERAL_MAP(0x3000); 			   		//DMA3 -> SPORT0 rx
  bfin_write_DMA3_IRQ_STATUS(bfin_read_DMA3_IRQ_STATUS() | 0x2);   		//Clear  DMA error flag  by writing 1 
  

  iRxBuffer1 = (u8 *)l1_data_sram_alloc(2*samples_per_chunk*BYTES_PER_FRAME); //allocates two buffers for 'samples_per_chunk' TDM frames, 
										//each BYTES_PER_FRAME bytes long   

  if (bfsi_debug)  printk(KERN_INFO "iRxBuffer1 = 0x%x\n", (int)iRxBuffer1);

  if (!iRxBuffer1){
    printk("bfsi : Error allocating iRxBuffer1");
    //return -ENOMEM;
    return ;
  }


  // Start address of data buffer 
  bfin_write_DMA3_START_ADDR(iRxBuffer1); // start adress

  // DMA inner loop count 
  bfin_write_DMA3_X_COUNT(samples_per_chunk*BYTES_PER_FRAME); 			//one frame brings info for one data sample * BYTES_PER_FRAME

  // Inner loop address increment 
  bfin_write_DMA3_X_MODIFY(1); 							// Inner loop modifites with 1 bytes 
  bfin_write_DMA3_Y_MODIFY(1); 							// Outer loop modifites with 1 byte 	
  bfin_write_DMA3_Y_COUNT(2);  							// the DMA uses 2D mode because they do dowble buffering 				
  // Configure DMA3
  // 8-bit transfers, Interrupt on completion, Autobuffer mode 
  bfin_write_DMA3_CONFIG(WNR | WDSIZE_8| DI_EN | 0x1000 | DI_SEL | DMA2D); 	// DMA read, 8 bit trasfer, auto buffer, 
										// interupt after completion of inner loop 
										// two dimentional DMA
  
  // Set up DMA4 to transmit, map DMA4 to Sport0 TX  -----------------------------------
  bfin_write_DMA4_PERIPHERAL_MAP(0x4000);
  // Configure DMA4 8-bit transfers, Autobuffer mode 
  bfin_write_DMA4_CONFIG(WDSIZE_8| 0x1000 | DMA2D);


  iTxBuffer1 = (u8 *)l1_data_sram_alloc(2*samples_per_chunk*BYTES_PER_FRAME); //allocates two buffers for 'samples_per_chunk' TDM frames, 
										 //each BYTES_PER_FRAME bytes long 

  if (bfsi_debug) printk(KERN_INFO "iTxBuffer1 = 0x%x\n", (int)iTxBuffer1);
  
  if (!iTxBuffer1){
    printk("bfsi : Error allocating iTxBuffer1");
    //return -ENOMEM;
    return ;
  }

  // Start address of data buffer 
  bfin_write_DMA4_START_ADDR(iTxBuffer1); 

  // DMA inner loop count 
  bfin_write_DMA4_X_COUNT(samples_per_chunk*BYTES_PER_FRAME); 

  // Inner loop address increment 
  bfin_write_DMA4_X_MODIFY(1);
  bfin_write_DMA4_Y_MODIFY(1);
  bfin_write_DMA4_Y_COUNT(2);


  lastreadchunk = &iRxBuffer1[BYTES_PER_FRAME*samples_per_chunk];  	// init test variables  
  lastwritechunk = &iTxBuffer1[BYTES_PER_FRAME*samples_per_chunk];
}


//==================================================================================================================================
// works out which read buffer is available for writting
static u8 *isr_write_processing(void) {
	u8 *writechunk;
	int x;

	// select which ping-pong buffer to write to 
	x = (int)(bfin_read_DMA4_CURR_ADDR()) - (int)iTxBuffer1;

	// Because of the DMA pipelining  x for tx tends to be 32 and samples_per_chunk*16+32
	//   32 sampels shift in respect to rx current adress	
	//   x for rx is exact 0 or samples_per_chunk*16 because it is interrupt syncronized 	
	

	if (x >= BYTES_PER_FRAME*samples_per_chunk) {
		writechunk = iTxBuffer1;
		writechunk_first++;
	}
	else {
		writechunk = iTxBuffer1 + samples_per_chunk*BYTES_PER_FRAME;
		writechunk_second++;
	}

	// make sure writechunk actually ping pongs 
	if (writechunk == lastwritechunk) writechunk_didntswap++;

	lastwritechunk = writechunk;

	return writechunk;
}
//===================================================================================================================================
// works out which read buffer is available for reading 
static u8 *isr_read_processing(void) {
	u8 *readchunk;
	int x;

	// select which ping-pong buffer to write to 
	x = (int)bfin_read_DMA3_CURR_ADDR() - (int)iRxBuffer1;

	// possible values for x are 16*samples_per_chunk=0x60 at the
	//   end of the first row and 2*16*samples_per_chunk=0x80 at the
	//   end of the second row 
	if (x == BYTES_PER_FRAME*samples_per_chunk) {
		readchunk = iRxBuffer1;
		readchunk_first++;
	}
	else {
		readchunk = iRxBuffer1 + samples_per_chunk*BYTES_PER_FRAME;
		readchunk_second++;
	}

	// make sure readchunk actually ping pongs 
	if (readchunk == lastreadchunk) readchunk_didntswap++;

	lastreadchunk = readchunk;

	return readchunk;
}


//=================================================================================================================
// sample cycles register of Blackfin 

static inline unsigned int cycles(void) {
  int ret;

   __asm__ __volatile__
   (
   "%0 = CYCLES;\n\t"
   : "=&d" (ret)
   :
   : "R1"
   );

   return ret;
}

//=================================================================================================================
// called each time the DMA finishes one "line" 

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t sport_rx_isr(int irq, void *dev_id, struct pt_regs * regs)
#else
static irqreturn_t sport_rx_isr(int irq, void *dev_id)
#endif
{
  unsigned int  start_cycles = cycles();
  u8           *read_samples;
  u8           *write_samples;
  int		i;

   
  // confirm interrupt handling, write 1 to DMA_DONE bit 
  bfin_write_DMA3_IRQ_STATUS(0x0001);


  __builtin_bfin_ssync(); 
  __builtin_bfin_ssync();
  
  read_samples = isr_read_processing();
  write_samples = isr_write_processing();
  

  if (bfsi_fxs_fxo_callback != NULL) {

    //Extract read FXS/FXO data from teh DMA buffer

    // We are only using a single FXS channel located at channel 8. However
    // it appears the upper zaptel layers only support 8 channels and also channel
    // number is tied to card. Rather than rewrite the upper layers and end up with
    // Zap/9, we rewrite the buffer so it's 8 channels as the upper layer expects and
    // move the timeslot to simulate a Zap/1


    int i;
    for(i=0;i<samples_per_chunk;i++){
      fxsfxo_rx_buffer[8*i]=read_samples[0+i*BYTES_PER_FRAME + 8];
      //fxsfxo_rx_buffer[8*i+1]=read_samples[1+i*BYTES_PER_FRAME];
      //fxsfxo_rx_buffer[8*i+2]=read_samples[2+i*BYTES_PER_FRAME];
      //fxsfxo_rx_buffer[8*i+3]=read_samples[3+i*BYTES_PER_FRAME];
      
    }

    bfsi_fxs_fxo_callback(fxsfxo_rx_buffer,fxsfxo_tx_buffer );
    

        
    //bfsi_fxs_fxo_callback(read_samples,write_samples );

    

    for(i=0;i<samples_per_chunk;i++){
          //Spread the write FXS/FXO data in the DMA buffer
      write_samples[0+i*BYTES_PER_FRAME + 8]=fxsfxo_tx_buffer[8*i+0];
      //write_samples[1+i*BYTES_PER_FRAME]=fxsfxo_tx_buffer[8*i+1];
      //write_samples[2+i*BYTES_PER_FRAME]=fxsfxo_tx_buffer[8*i+2];
      //write_samples[3+i*BYTES_PER_FRAME]=fxsfxo_tx_buffer[8*i+3];

    }



  }


  __builtin_bfin_ssync();
	/* Call other soft irq */
  for (i = 0; i < ARRAY_SIZE(soft_isr_table); i++) {
    if (likely(soft_isr_table[i].isr_enabled)) {
			if (likely(soft_isr_table[i].soft_isr != NULL))
			  soft_isr_table[i].soft_isr(soft_isr_table[i].data);
    }
    //isr_cycles_3 = cycles() - start_cycles;
    if (likely(soft_isr_table[i].dma_enabled)) {
      if (likely(soft_isr_table[i].soft_dma_isr != NULL)) {


	softinterrupt++;	
	soft_isr_table[i].soft_dma_isr(read_samples,
				       write_samples);
	
	

      }
      
		
    }
  }
  



  // Some stats to help monitor the cycles used by ISR processing 
  //   Simple IIR averager: 
  //
  //     	y(n) = (1 - 1/TC)*y(n) + (1/TC)*x(n)
  //
  //   After conversion to fixed point:
  //
  //    	2*y(n) = ((TC-1)*2*y(n) + 2*x(n) + half_lsb ) >> LTC 
  





  isr_cycles_average = ( (u32)(TC-1)*isr_cycles_average + (((u32)isr_cycles_last)<<1) + TC) >> LTC;
  
  if (isr_cycles_last > isr_cycles_worst) isr_cycles_worst = isr_cycles_last;
  
  
  // we sample right at the end to make sure we count cycles used to measure cycles!
  isr_cycles_last = cycles() - start_cycles;
  
  return IRQ_HANDLED;
}

//===================================================================================================

static int init_sport_interrupts(void)
{
  	
	if(request_irq(IRQ_SPORT0_RX, sport_rx_isr, IRQF_DISABLED, "sport0 rx", NULL) != 0) { // connect the ISR to given interrupt
    
		return -EBUSY;
	}

	if (bfsi_debug) printk(KERN_INFO "ISR installed OK\n");

	// enable DMA3 sport0 Rx interrupt 
	// bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() | 0x00000080); //enable DMA5, sport 1 rx  which is exectued each 8 sample chunk! 
	
	// enable DMA3 sport0 Rx interrupt 
	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() | 0x00000020); //enable DMA3, sport 0 rx  which is exectued each 8 sample chunk! 

	__builtin_bfin_ssync();

	return 0;


}

//=========================================================================================================

static void enable_dma_sport(void)
{
	// enable DMAs 
	bfin_write_DMA4_CONFIG(bfin_read_DMA4_CONFIG() | DMAEN);  // enable DMA 4 for rx
	bfin_write_DMA3_CONFIG(bfin_read_DMA3_CONFIG() | DMAEN);  // enable DMA 5 for tx 
	__builtin_bfin_ssync();

        if(bfsi_debug)  printk(KERN_INFO "DMA3/DMA4 enabled\n");


		
	// enable sport0 Tx and Rx 
	bfin_write_SPORT0_TCR1(bfin_read_SPORT0_TCR1() | TSPEN); //enable tx
	bfin_write_SPORT0_RCR1(bfin_read_SPORT0_RCR1() | RSPEN); //enable rx
		

	__builtin_bfin_ssync();

        if(bfsi_debug) printk(KERN_INFO "SPORT0 enabled\n");
}
//==========================================================================================================

static void disable_sport(void)
{
        printk("disable sport called\n");
	
	// disable sport1 Tx and Rx 
	bfin_write_SPORT0_TCR1(bfin_read_SPORT0_TCR1() & (~TSPEN)); // disable tx
	bfin_write_SPORT0_RCR1(bfin_read_SPORT0_RCR1() & (~RSPEN)); // disable rx
	__builtin_bfin_ssync();

	// disable DMA3 and DMA4 
	bfin_write_DMA4_CONFIG(bfin_read_DMA4_CONFIG() & (~DMAEN)); // disable dma4
	bfin_write_DMA3_CONFIG(bfin_read_DMA3_CONFIG() & (~DMAEN)); // disable dma3 
	__builtin_bfin_ssync();
	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() & (~0x00000020));
	__builtin_bfin_ssync();

}

//======================================================================================================
// shut down SPORT operation cleanly 
void bfsi_sport0_close(void)
{
  disable_sport();

  if (init==2) {
    free_irq(IRQ_SPORT0_RX, NULL);
  }
  if(init){

    l1_data_sram_free(iTxBuffer1);
    l1_data_sram_free(iRxBuffer1);
    
    
    l1_data_sram_free(fxsfxo_tx_buffer);
    l1_data_sram_free(fxsfxo_rx_buffer);


    remove_proc_entry("bfsi", NULL);
  }	
}


//======================================================================================================
int bfsi_proc_read(char *buf, char **start, off_t offset,
                    int count, int *eof, void *data)
{
        int len;

        len = sprintf(buf,
                      "readchunk_first.....: %d\n"
                      "readchunk_second....: %d\n"
                      "readchunk_didntswap.: %d\n"
                      "writechunk_first....: %d\n"
                      "writechunk_second...: %d\n"
                      "writechunk_didntswap: %d\n"
                      "isr_cycles_last.....: %d\n"
                      "isr_cycles_worst....: %d\n"
                      "isr_cycles_average..: %d\n"
                      "echo_sams...........: %d\n"
		      "DMA3_IRQ_STATUS.....: %d\n"
		      "DMA4_IRQ_STATUS.....: %d\n"
		      "SPORT0_STATUS.......: %d\n"
		      "softinterrupt.......: %d\n"
,
		      readchunk_first,
                      readchunk_second,
                      readchunk_didntswap,
                      writechunk_first,
                      writechunk_second,
                      writechunk_didntswap,
                      isr_cycles_last,
                      isr_cycles_worst,
                      isr_cycles_average>>1,
                      echo_sams,
		      bfin_read_DMA3_IRQ_STATUS(),
		      bfin_read_DMA4_IRQ_STATUS(),
		      bfin_read_SPORT0_STAT(),
		      softinterrupt		      		      
		      );

        *eof=1;
        return len;
}
//======================================================================================================

/* 
   Wrapper for entire SPORT setup, returns 1 for success, 0 for failure.

   The SPORT code is designed to deliver small arrays of size samples
   every (125us * samples).  A ping-pong arrangement is used, so the
   address of the buffer will alternate every call between two possible
   values.

   The callback functions privide to the address of the current buffer
   for the read and write channels.  Read means the data was just
   read from the SPORT, so this is the "receive" PCM samples.  Write
   is the PCM data to be written to the SPORT.
   
   The callbacks are called in the context of an interrupt service
   routine, so treat any code them like an ISR.

   Once this function returns successfully the SPORT/DMA will be up
   and running, and calls to the isr callback will start.  For testing
   it is OK to set the callback function pointer to NULL, say if you
   just want to look at the debug information.
   
   If debug==1 then "cat /proc/bfsi" will display some debug
   information, something like:

     readchunk_first.....: 9264
     readchunk_second....: 9264
     readchunk_didntswap.: 0
     writechunk_first....: 9264
     writechunk_second...: 9264
     writechunk_didntswap: 0

   If all is well then "readchunk_didntswap" and "writechunk_didntswap"
   will be static and some very small number.  The first and second
   values should be at most one value different.  These variables
   indicate sucessful ping-pong operation.

   The numbers are incremented ever interrupt, for example if samples=8
   (typical for zaptel), then we get one interrupt every ms, or 1000
   interrupts per second.  This means the values for each first/second
   entry should go up 500 times per second.

   8 channels are sampled at once, so the size of the samples buffers
   is 8*samples (typically 64 bytes for zaptel).

   init = 0 	//no initialization is done yet
   init = 1 	//buffers are allocated
   init = 2	//SPORT is started and interrupts are driven 
			
*/

int bfsi_sport0_init(int samples, int debug)
{
  int i;

  bfsi_debug=debug;
  samples_per_chunk = samples;

	
  if(init){
	printk(KERN_INFO "BFSI already initialized\n");	
	return(0);
  }

  

  if (debug) create_proc_read_entry("bfsi", 0, NULL, bfsi_proc_read, NULL);
 	

  /* First thing we need to do is take the ISDN chip out of reset as it may be driving the 
     PCM bus
  */

  
  //bfsi_hfc_reset(10000);

  /* Zaptel always expects 8 channels. We have 9 so we need to pack the data to make zaptel happy */

  fxsfxo_tx_buffer = (u8 *)l1_data_sram_alloc(samples_per_chunk*2*8); //buffer for the fxofxs we have two codecs and we have,
  fxsfxo_rx_buffer = (u8 *)l1_data_sram_alloc(samples_per_chunk*2*8); //samples_per_chunk frames x 8 channels

  if ( !(fxsfxo_tx_buffer) ||  !(fxsfxo_rx_buffer)) {
    printk("bfsi : Error allocating Tx and Rx Buffers");
    return -ENOMEM;
  }


  init_sport0();

  init_dma_wc();
  enable_dma_sport();

  for(i=0;i<BYTES_PER_FRAME*samples_per_chunk*2;i++) iTxBuffer1[i]=0; //Clean the transmit DMA buffer


  if (init_sport_interrupts())
  init = 1;
  else
    init = 2;

  return(!(init==2));
  
}
//========================================================================================================
//There are two kind of clients which hook callback functions to the sport1. -fxs/fxo and fwfxs client drivers
int bfsi_hook_callback(void (*isr_callback)(u8 *read_samples, u8 *write_samples), int client_type){

	if(client_type==FXS_FXO_CLIENT)
		bfsi_fxs_fxo_callback = isr_callback;		
       
	return(0);
}




/*-------------------------- SPI FUNCTIONS ----------------------------*/

/* May want to  move these out to there own files as part of a cleanup. */
  


/* 
   new_chip_select_mask: the logical OR of all the chip selects we wish
   to use for SPI, for example if we wish to use SPISEL2 and SPISEL3
   chip_select_mask = (1<<2) | (1<<3).

   baud:  The SPI clk divider value, see Blackfin Hardware data book,
   maximum speed when baud = 2, minimum when baud = 0xffff (0 & 1
   disable SPI port).

   The maximum SPI clk for the Si Labs 3050 is 16.4MHz.  On a 
   100MHz system clock Blackfin this means baud=4 minimum (12.5MHz).

   For the IP04 some extra code needed to be added to the three SPI
   routines to handle the use of PF12 as nCSB.  It's starting to 
   look a bit messy and is perhaps inefficient.
*/

void bfsi_spi_init(int mybaud, u16 new_chip_select_mask) 
{
	u16 ctl_reg, flag;
	int cs, bit;

	baud=mybaud;

  	if (baud < 4) {
    		printk("baud = %d may mean SPI clock too fast for Si labs 3050"
	   		"consider baud == 4 or greater", baud);
  	}

	
	PRINTK("bfsi_spi_init\n");
	PRINTK("  new_chip_select_mask = 0x%04x\n", new_chip_select_mask);
	//PRINTK("  FIOD_DIR = 0x%04x\n", bfin_read_FIO_DIR());

	/* grab SPISEL/GPIO pins for SPI, keep level of SPISEL pins H */
	chip_select_mask |= new_chip_select_mask;

	flag = 0xff00 | (chip_select_mask & 0xff);



	/* we need to work thru each bit in mask and set the MUX regs */

	for(bit=0; bit<8; bit++) {
	  if (chip_select_mask & (1<<bit)) {
	    PRINTK("SPI CS bit: %d enabled\n", bit);
	    cs = bit;
	    if (cs == 1) {
	      PRINTK("set for chip select 1\n");
	      bfin_write_PORTF_FER(bfin_read_PORTF_FER() | 0x3c00);
	      __builtin_bfin_ssync();

	    } else if (cs == 2 || cs == 3) {
	      PRINTK("set for chip select 2\n");
	      bfin_write_PORT_MUX(bfin_read_PORT_MUX() | PJSE_SPI);
	      __builtin_bfin_ssync();
	      bfin_write_PORTF_FER(bfin_read_PORTF_FER() | 0x3800);
	      __builtin_bfin_ssync();

	    } else if (cs == 4) {
	      PRINTK("set for chip select 4\n");
	      bfin_write_PORT_MUX(bfin_read_PORT_MUX() | PFS4E_SPI);
	      __builtin_bfin_ssync();
	      bfin_write_PORTF_FER(bfin_read_PORTF_FER() | 0x3840);
	      __builtin_bfin_ssync();

	    } else if (cs == 5) {
	       PRINTK("set for chip select 5\n");
	      bfin_write_PORT_MUX(bfin_read_PORT_MUX() | PFS5E_SPI);
	      __builtin_bfin_ssync();
	      bfin_write_PORTF_FER(bfin_read_PORTF_FER() | 0x3820);
	      __builtin_bfin_ssync();

	    } else if (cs == 6) {
	      bfin_write_PORT_MUX(bfin_read_PORT_MUX() | PFS6E_SPI);
	      __builtin_bfin_ssync();
	      bfin_write_PORTF_FER(bfin_read_PORTF_FER() | 0x3810);
	      __builtin_bfin_ssync();

	    } else if (cs == 7) {
	      bfin_write_PORT_MUX(bfin_read_PORT_MUX() | PJCE_SPI);
	      __builtin_bfin_ssync();
	      bfin_write_PORTF_FER(bfin_read_PORTF_FER() | 0x3800);
	      __builtin_bfin_ssync();
	    }
	  }
	}
	
  	/* note TIMOD = 00 - reading SPI_RDBR kicks off transfer */
  	ctl_reg = SPE | MSTR | CPOL | CPHA | SZ;
  	write_FLAG(flag);
  	write_BAUD(baud);
  	write_CTRL(ctl_reg);
	PRINTK("  After PORT_MUX = 0x%04x\n",bfin_read_PORT_MUX());
	PRINTK("  After PORTF_FER = 0x%04x\n",bfin_read_PORTF_FER());

}


/* 
   We create a new function that will enable the SPI only once. Currently both the wcfxs and the xhf driver 
   want to initialize it. We will create a new function that will only initialize it once to prevent them
   from stepping on each other.

   We will assume that FXS uses SPSEL 5 and that the XHFC chip uses SPSEL 4. Careful this now couples the bfsi
   with the higher level drivers

*/


void bfsi_spi_init_global(void){

  u8 SPI_CS_FXS = 5;
  u8 SPI_CS_HFC = 4;
  u8 SPI_BAUDS = 8;


  if ( bfsi_init==0) {
    bfsi_spi_init( SPI_BAUDS, ( (1<<SPI_CS_FXS) | (1<<SPI_CS_HFC)  )) ;
    printk("Initializing SPI\n"); 
  }
}




/* 
   After much experimentation I found that (i) TIMOD=00 (i.e. using
   read_RDBR() to start transfer) was the best way to start transfers
   and (ii) polling RXS was the best way to end transfers, see p10-30
   and p10-31 of BF533 data book.

   chip_select is the _number_ of the chip select line, e.g. to use
   SPISEL2 chip_select = 2.
*/

void bfsi_spi_write_8_bits(u16 chip_select, u8 bits)
{
  u16 flag_enable, flag = 0;

  if ((chip_select < 8) && (chip_select != 0))  {
    flag = read_FLAG();
    flag_enable = flag & ~(1 << (chip_select + 8));
    //PRINTK("chip_select: %d write: flag: 0x%04x flag_enable: 0x%04x \n", 
    //	   chip_select, flag, flag_enable);

    /* drop SPISEL */
    write_FLAG(flag_enable); 
  }


 
 else {
   printk("ERROR - Invalid Chip select, %x\n",chip_select);
   //bfin_write_FIO_FLAG_C((1<<chip_select));
   
 
 }

  /* read kicks off transfer, detect end by polling RXS */
  write_TDBR(bits);
  read_RDBR(); __builtin_bfin_ssync();
  do {} while (!(read_STAT() & RXS) );

  
  write_FLAG(flag); 
  
  
}

u8 bfsi_spi_read_8_bits(u16 chip_select)
{
  u16 flag_enable = 0, flag = 0, ret;

  if ((chip_select < 8) && (chip_select != 0)) {
    flag = read_FLAG();
    flag_enable = flag & ~(1 << (chip_select + 8));
    //PRINTK("read: flag: 0x%04x flag_enable: 0x%04x \n", 
    //	   flag, flag_enable);
  }
  else {
    printk("ERROR - Invalid Chip select, %x\n",chip_select);
      //bfin_write_FIO_FLAG_C((1<<chip_select));
    
  }

  /* Problem here ? If chip_select >= 8  flag_enable never gets set */

  /* drop SPISEL */
  write_FLAG(flag_enable); 

  /* read kicks off transfer, detect end by polling RXS, we
     read the shadow register to prevent another transfer
     being started 

     While reading we write a dummy tx value, 0xff.  For
     the MMC card, a 0 bit indicates the start of a command 
     sequence therefore an all 1's sequence keeps the MMC
     card in the current state.
  */
  //write_TDBR(0xff);
  read_RDBR();
  __builtin_bfin_ssync();
  do {} while (!(read_STAT() & RXS) );
  ret = bfin_read_SPI_SHADOW();

//printk("SPI Reading %x\n",ret);
  /* raise SPISEL */
  
  write_FLAG(flag); 
  
  return ret;
}



/**
 * bfsi_spi_array_write - write an array of values to an SPI device
 * @chip_select: The _number_ of the chip select line, e.g. to use 
 *               SPISEL2 chip_select = 2.
 * @buf: Transmit buffer
 * @buf_size: Transmit buffer size
 * 
 * Write buf_size octects of data on the specified SPI bus. SPI select signal 
 * is activated before first octect is sent and deactivated after last octect 
 * is sent.
 *
 */
void bfsi_spi_array_write(u16 chip_select, u8 *buf, int buf_size)
{
	u16 flag_enable, flag;
	int i;
	

	if (unlikely((buf_size <= 0) || (buf == NULL))){
		BUG();
		return;
	}


	flag = bfin_read_SPI_FLG();
	flag_enable = flag & ~(1 << (chip_select + 8));
	
	/* Set SPI bus speed */
	bfin_write_SPI_BAUD(baud);
	
	/* drop SPISEL */
	bfin_write_SPI_FLG(flag_enable);

	/* read kicks off transfer, detect end by polling RXS */
	for (i = 0; i < buf_size; i++) {
		bfin_write_SPI_TDBR(buf[i]);
		bfin_read_SPI_RDBR();
		__builtin_bfin_ssync();
		do {} while (!(bfin_read_SPI_STAT() & RXS));
	}

	udelay(2); //Makes the SPI communication in the first BR4-appliance more stable

	/* raise SPISEL */
	bfin_write_SPI_FLG(flag); 

	udelay(2); //Makes the SPI communication in the first BR4-appliance more stable

}

/**
 * bfsi_spi_array_read - Read an array of values from a SPI device
 * @chip_select: The _number_ of the chip select line, e.g. to use 
 *               SPISEL2 chip_select = 2.
 * @buf: Receive buffer
 * @buf_size: Receive buffer size
 * 
 * Read buf_size octects of data from the specified SPI bus. SPI select signal 
 * is activated before first octect is received and deactivated after last 
 * octect is received.
 *
 */
void bfsi_spi_array_read(u16 chip_select, u8 *buf, int buf_size)
{
	u16 flag_enable, flag;
	int i;
	
	if (unlikely((buf_size <= 0) || (buf == NULL) )){
	    
		BUG();
		return;
	}

	flag = bfin_read_SPI_FLG();
	flag_enable = flag & ~(1 << (chip_select + 8));
	
	/* Set SPI bus speed */
	bfin_write_SPI_BAUD(baud);
	
	/* drop SPISEL */
	bfin_write_SPI_FLG(flag_enable);

	/* read kicks off transfer, detect end by polling RXS */
	for (i = 0; i < buf_size; i++) {
		bfin_write_SPI_TDBR(0xFF);
		bfin_read_SPI_RDBR();
		__builtin_bfin_ssync();
		do {} while (!(bfin_read_SPI_STAT() & RXS));
		buf[i] = bfin_read_SPI_SHADOW();
	}

	udelay(2); //Makes the SPI communication in the first BR4-appliance more stable

	/* raise SPISEL */
	bfin_write_SPI_FLG(flag); 
	
	udelay(2); //Makes the SPI communication in the first BR4-appliance more stable
}


/**
 * bfsi_spi_u8_write_array_read - Perform an array write then read transaction
 * @chip_select: The _number_ of the chip select line, SPISEL2 => chip_select=2.
 * @buf: (pre-alloacted) buf-size wide receive buffer
 * @buf_size: Receive buffer size
 * 
 * Perform a transaction of two accesses: Write one byte first, than read 
 * buf_size byte from device. Specifically designed to implement Cologne chip 
 * XHFC driver's '40bit optimized access'.
 *
 */
void bfsi_spi_u8_write_array_read(u16 chip_select, u8 write_buf, u8 *buf, 
								int buf_size)
{
	u16 flag_enable, flag;
	int i;
	
	if (unlikely((buf_size <= 0) || (buf == NULL))) {
		BUG();
		return;
	}

	flag = bfin_read_SPI_FLG();
	flag_enable = flag & ~(1 << (chip_select + 8));
	
	/* Set SPI bus speed */
	bfin_write_SPI_BAUD(baud);
	
	/* drop SPISEL */
	bfin_write_SPI_FLG(flag_enable); 
	
	/* Write first (command) byte */
	bfin_write_SPI_TDBR(write_buf);
	bfin_read_SPI_RDBR();
	__builtin_bfin_ssync();
	do {} while (!(bfin_read_SPI_STAT() & RXS) );

	/* Read n bytes from SPI  */
	for (i = 0; i < buf_size; i++) {
		bfin_write_SPI_TDBR(0xff);
		bfin_read_SPI_RDBR(); 
		__builtin_bfin_ssync();
		do {} while (!(bfin_read_SPI_STAT() & RXS) );
		buf[i] = bfin_read_SPI_SHADOW();
	}
	
	udelay(2); //Makes the SPI communication in the first BR4-appliance more stable

	/* raise SPISEL */
	bfin_write_SPI_FLG(flag); 


	udelay(2); //Makes the SPI communication in the first BR4-appliance more stable
}



/**
 * bfsi_spi_disable - Finalize SPI support on Blackfin processor
 * @chip_select_mask: the logical OR of all CS to be disabled
 * 
 * Notes:
 * chip_select_mask should be set to the logical OR of all the chip selects used
 * with the SPI interface to be removed. For example, to remove an interface 
 * which used SPISEL2 and SPISEL3, set chip_select_mask = (1 << 2) | (1 << 3).
 */
void bfsi_spi_disable(u16 cs_mask_to_remove) 
{
	u16 flag;
	int cs, bit;

	/* Remove SPISEL/GPIO pin usage */
	chip_select_mask &= ~(cs_mask_to_remove);
	flag = 0xff00 | chip_select_mask;

#if defined(CONFIG_BF537)

	/* we need to work thru each bit in mask and set the MUX regs */
	for(bit = 0; bit < 8; bit++) {
		if (cs_mask_to_remove & (1 << bit)) {
			u16 port_f_fer = bfin_read_PORTF_FER();
			u16 port_mux   = bfin_read_PORT_MUX();

			PRINTK("Removing SPI CS bit: %d\n", bit);
			cs = bit;
			//spi_baud[bit] = 0;
			
			switch (cs) {
			case 1:
				bfin_write_PORTF_FER(port_f_fer & ~(0x400));
				__builtin_bfin_ssync();
				break;
			case 2:
			case 3:
				if (!(chip_select_mask & ((1 << 2)|(1 << 3)))) {
					bfin_write_PORT_MUX(port_mux & 
								~(PJSE_SPI));
					__builtin_bfin_ssync();
				}
				break;
				
			case 4:
				bfin_write_PORT_MUX(port_mux & ~(PFS4E_SPI));
				__builtin_bfin_ssync();
				bfin_write_PORTF_FER(port_f_fer & ~(0x0040));
				__builtin_bfin_ssync();
				break;
			case 5:
				bfin_write_PORT_MUX(port_mux & ~(PFS5E_SPI));
				__builtin_bfin_ssync();
				bfin_write_PORTF_FER(port_f_fer & ~(0x0020));
				__builtin_bfin_ssync();
				break;
			case 6:
				bfin_write_PORT_MUX(port_mux & ~(PFS6E_SPI));
				__builtin_bfin_ssync();
				bfin_write_PORTF_FER(port_f_fer & ~(0x0010));
				__builtin_bfin_ssync();
				break;
			case 7:
				bfin_write_PORT_MUX(port_mux & ~(PJCE_SPI));
				__builtin_bfin_ssync();
				break;
			}
		}
	}
	if (chip_select_mask == 0x0) {
		bfin_write_PORTF_FER(bfin_read_PORTF_FER() & ~(0x3800));
		bfin_write_SPI_CTL(0);
	}
#endif
}



#if 0
void fxs_workaround(void){

  printk("Starting Fake burst\n");
  
  /* Set the function of the pin as a GPIO. We clear the bit  */
  /* Set bits 5,6 and 12

     0010 0000 0110 0000
  */


  bfin_write_PORTF_FER(bfin_read_PORTF_FER() & ~(0x2060));
  __builtin_bfin_ssync();
  
  /* Now set them as outputs */
  
  bfin_write_PORTFIO_DIR(bfin_read_PORTFIO_DIR() | (0x2060) ); 
  __builtin_bfin_ssync();
  
  /*Set to it's default state (SPISEL disabled - HIGH  */
  
  udelay(1);
  bfin_write_PORTFIO_CLEAR(1 << 5);
  bfin_write_PORTFIO_CLEAR(1 << 6);
  __builtin_bfin_ssync();

  udelay(1);
  bfin_write_PORTFIO_CLEAR(1 << 13);
   __builtin_bfin_ssync();

  udelay(1);
  bfin_write_PORTFIO_SET(1 << 13);
  __builtin_bfin_ssync();

  udelay(1);
  bfin_write_PORTFIO_CLEAR(1 << 13);
   __builtin_bfin_ssync();

  udelay(1);
  bfin_write_PORTFIO_SET(1 << 13);
  __builtin_bfin_ssync();

  udelay(1);
  bfin_write_PORTFIO_SET(1 << 5);
  bfin_write_PORTFIO_SET(1 << 6);

  udelay(1000);


  printk("Finished fake burst\n");

}
#endif
		
/*-------------------------- RESET FUNCTIONS ----------------------------*/

/**
 * bfsi_reset - Reset line drivers using Blackfin GPIO
 * @reset_line: GPIO line hardware RESET is hooked to
 * @delay_us: RESET activation time (us).
 */
void bfsi_hfc_reset(int delay_us) {
	PRINTK("toggle reset hfc reset\n");
	bfin_write_PORTFIO_INEN(bfin_read_PORTFIO_INEN() & 0xFDFF);
	__builtin_bfin_ssync();
	PRINTK("PORTFIO_INEN is now %x\n",bfin_read_PORTFIO_INEN());   
	bfin_write_PORTF_FER(bfin_read_PORTF_FER() & 0xFDFF);
	__builtin_bfin_ssync();
	bfin_write_PORTFIO_DIR(bfin_read_PORTFIO_DIR() | 0x0200);
	__builtin_bfin_ssync();
	bfin_write_PORTFIO_CLEAR(1<<9);
	__builtin_bfin_ssync();
	udelay(delay_us);
	bfin_write_PORTFIO_SET(1<<9);
	__builtin_bfin_ssync();
}



/**
 * bfsi_soft_irq_register - Register a soft ISR pair called by real DMA ISR
 * @isr: ISR iteself: registered ISR will be called by real SPORT ISR
 * @dma_isr: DMA service routine to manage PCM data received from TDM bus
 * @data: context passed to isr() upon call.
 * @isr_enabled: if true, activate function at the end of registration
 * @dma_enabled: if true, activate function at the end of registration
 *
 * This function allows other kernel modules, specifically analog and ISDN line
 * interface drivers, to register a 'hook' function called by Blackfin SPORT
 * DMA interrupt. Two functions can be registered and both will be called at
 * regular intervals of 1ms. 'isr' function is passed 'data' context while
 * 'dma_isr' is passed receive and transmit PCM audio data buffers.
 *
 */
int bfsi_soft_irq_register(void(*isr)(void*), void(*dma_isr)(u8*, u8*),
				void *data, int isr_enabled, int dma_enabled)
{
	int i;
	u_long flags = 0;

	if (isr == NULL)
		return -1;

	spin_lock_irqsave(&isr_table_lock, flags);
	for (i = 0; i < ARRAY_SIZE(soft_isr_table); i++) {
		if (soft_isr_table[i].soft_isr == NULL) {
			soft_isr_table[i].soft_isr = isr;
			soft_isr_table[i].soft_dma_isr = dma_isr;
			soft_isr_table[i].data = data;
			soft_isr_table[i].divider = 1;
			soft_isr_table[i].isr_enabled = isr_enabled;
			soft_isr_table[i].dma_enabled = dma_enabled;
			spin_unlock_irqrestore(&isr_table_lock, flags);
			return i;
		}
	}
	spin_unlock_irqrestore(&isr_table_lock, flags);
	return -1;

} /* bfsi_soft_irq_register() */

/**
 * bfsi_soft_irq_free - release a soft ISR registration
 *
 */
void bfsi_soft_irq_free (int nr)
{
	u_long flags = 0;

	if (nr >= ARRAY_SIZE(soft_isr_table))
		return;

	spin_lock_irqsave(&isr_table_lock, flags);
	soft_isr_table[nr].isr_enabled = 0;
	soft_isr_table[nr].dma_enabled = 0;
	soft_isr_table[nr].soft_isr = NULL;
	soft_isr_table[nr].data = NULL;
	spin_unlock_irqrestore(&isr_table_lock, flags);
}

/**
 * bfsi_soft_irq_enable -
 *
 */
void bfsi_soft_irq_enable (int nr)
{
	u_long flags = 0;

	if (nr >= ARRAY_SIZE(soft_isr_table))
		return;

	spin_lock_irqsave(&isr_table_lock, flags);
	soft_isr_table[nr].isr_enabled = 1;
	spin_unlock_irqrestore(&isr_table_lock, flags);
}

/**
 * bfsi_soft_irq_disable -
 *
 */
void bfsi_soft_irq_disable (int nr)
{
	u_long flags = 0;

	if (nr >= ARRAY_SIZE(soft_isr_table))
		return;

	spin_lock_irqsave(&isr_table_lock, flags);
	soft_isr_table[nr].isr_enabled = 0;
	spin_unlock_irqrestore(&isr_table_lock, flags);
}

/**
 * bfsi_soft_dma_enable -
 *
 */
void bfsi_soft_dma_enable (int nr)
{
	u_long flags = 0;

	if (nr >= ARRAY_SIZE(soft_isr_table))
		return;

	spin_lock_irqsave(&isr_table_lock, flags);
	soft_isr_table[nr].dma_enabled = 1;
	spin_unlock_irqrestore(&isr_table_lock, flags);
}

/**
 * bfsi_soft_dma_disable -
 *
 */
void bfsi_soft_dma_disable (int nr)
{
	u_long flags = 0;

	if (nr >= ARRAY_SIZE(soft_isr_table))
		return;

	spin_lock_irqsave(&isr_table_lock, flags);
	soft_isr_table[nr].dma_enabled = 0;
	spin_unlock_irqrestore(&isr_table_lock, flags);
}



//========================================================================================================
//Exported symbols
MODULE_LICENSE("GPL");
EXPORT_SYMBOL(bfsi_sport0_init);
EXPORT_SYMBOL(bfsi_sport0_close);
EXPORT_SYMBOL(bfsi_hook_callback);
EXPORT_SYMBOL(bfsi_hfc_reset);



// SPI functions
EXPORT_SYMBOL(bfsi_spi_write_8_bits);
EXPORT_SYMBOL(bfsi_spi_read_8_bits);
EXPORT_SYMBOL(bfsi_spi_init);
EXPORT_SYMBOL(bfsi_spi_init_global);
EXPORT_SYMBOL(bfsi_spi_array_write);
EXPORT_SYMBOL(bfsi_spi_array_read);
EXPORT_SYMBOL(bfsi_spi_u8_write_array_read);
EXPORT_SYMBOL(bfsi_spi_disable);


// Soft functions required by xhfc driver

EXPORT_SYMBOL(bfsi_soft_irq_register);
EXPORT_SYMBOL(bfsi_soft_irq_free);
EXPORT_SYMBOL(bfsi_soft_irq_enable);
EXPORT_SYMBOL(bfsi_soft_irq_disable);
EXPORT_SYMBOL(bfsi_soft_dma_enable);
EXPORT_SYMBOL(bfsi_soft_dma_disable);
