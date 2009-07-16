/*
  bfsi_tlg.c

  Copyright (C) 2006 David Rowe

  Modified for Tlg boards by Diego Serafin 24 november 2006

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.


  Functions for Linux device drivers on the Blackfin that
  support interfacing the Blackfin to Silicon Labs chips.

  These functions are in a separate file from the target wcfxs driver
  so they can be re-used with different drivers, for example unit
  test software.

  For various reasons the CPHA=1 (sofware controlled SPISEL)
  mode needs to be used, for example the SiLabs chip expects
  SPISEL to go high between 8 bit transfers and the timing
  the Si3050 expects (Figs 3 & 38 of 3050 data sheet) are
  closest to Fig 10-12 of the BF533 hardware reference manual.

  See also unittest/spi for several unit tests.


  About TLG boards.

  Tlg embedded asterisk line interface boards are based on Si3220 FXS and
  Si3050 FXO components by SiLabs. Both chip could be intermixed on the same
  SPI bus sharing the same chip select and must be connected in a daisy chain
  fashion.

  Module parameters:

  - tdm_slave 1, if clock AND frame sync are provided externally, 0 if only
                 clock signal is provided externally, while fsync is generated
                 by Blackfin SPORT hardware.

  - tdm_clkX2  1, TDM bus will support double rate clock (C4, i.e. 2 clock
               cycles per bit)
               0, TDM bus will support single rate clock (default)

  Updates:
  ========
  Oct 22 / 07 - Added support for PF14 to be a reset :: Pawel Pastuszak <pawel@astfin.org>
  Nov 1 / 07  - Support for sport0 on BF537; mark@astfing.org
  Mar 28 / 08 - Applied David Rowe's DMA buffer selection bugfix: bug led to lose one DMA
                buffer from time to time, making audible clikcs on TDM device audio signal.
                Added L1 SRAM buffer allocation check. Removed unused tasklet code for DMA ISR.
                Added BF532 processor support. Added timeslot number configurability via
                N_SLOT macro; Diego Serafin <diego.serafin@gmail.com>

*/

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/bfin5xx_spi.h>
#include <linux/delay.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <asm/blackfin.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/bfsi_tlg.h>
//#include "bfsi_spi.h"


#define BFSI_PARAMS_SANITY_CHECK

#ifdef BFSI_PARAMS_SANITY_CHECK
#define NULL_CHECK(p)	if(p == NULL) { \
				printk(KERN_ERR "%s: %s is NULL\n", \
					__FUNCTION__, #p);}
#else
#define NULL_CHECK(p)	{/* NOP */}
#endif

/* Default for BF537 STAMP is SPORT1 */

#ifdef CONFIG_BR4
#define BFSI_SPORT0
#else
#define BFSI_SPORT1
#endif


/* This should always be 8, at least to work with zaptel */
#ifndef SAMPLES_PER_CHUNK
#define SAMPLES_PER_CHUNK 8
#endif


/* enable this define to get verbose debugging info */
#define BFIN_SI_DEBUG  1

#if defined(BFIN_SI_DEBUG)
#define PRINTK(args...) printk(args)
#else
#define PRINTK(args...) do {} while(0)
#endif

/* constants for isr cycle averaging */

#define TC    1024 /* time constant    */
#define LTC   10   /* base 2 log of TC */

/* ping-pong buffer states for debug */

#define PING 0
#define PONG 1

/* misc statics */

static const char * svn_version = "$Id: bfsi_tlg.c 1049 2008-03-28 10:34:09Z diego $";

static u8 *iTxBuffer1;
static u8 *iRxBuffer1;

static int internalclock = 0;
static int bfsi_debug = 1;
static int debug = 1;
static int init_ok = 0;
static unsigned int tdm_slave = 1;
static unsigned int tdm_clkX2 = 1;

#ifdef MODULE
module_param(tdm_slave, uint, S_IRUGO | S_IWUSR);
module_param(tdm_clkX2, uint, S_IRUGO | S_IWUSR);
module_param(debug, int, S_IRUGO | S_IWUSR);
#else
#error bfsi_tlg MUST be compiled as a module
#endif

/* isr callback installed by user */

static void (*bfsi_isr_callback)(u8 *read_samples, u8 *write_samples) = NULL;

/* debug variables */

static int readchunk_first = 0;
static int readchunk_second = 0;
static int readchunk_didntswap = 0;
static int lastreadpingpong;

static int bad_x[5];
static int last_x[5];
static u8 *log_readchunk;

static int writechunk_first = 0;
static int writechunk_second = 0;
static int writechunk_didntswap = 0;
static int lastwritepingpong;

/* previous and worst case number of cycles we took to process an
   interrupt */

static u32 isr_cycles_last = 0;
static u32 isr_cycles_worst = 0;
static u32 isr_cycles_average = 0; /* scaled up by 2x */
static u32 echo_sams = 0;

/* monitor cycles between ISRs */

static u32 isr_between_prev = 0;
static u32 isr_between_worst = 0;
static u32 isr_between_diff = 0;
static u32 isr_between_difflastskip = 0;
static u32 isr_between_skip = 0;

/* freeze ISR activity for test purposes */

static int bfsi_freeze = 0;

static u32 isr_cycles_1 = 0;
static u32 isr_cycles_2 = 0;
static u32 isr_cycles_3 = 0;

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

/* sample cycles register of Blackfin */

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


/*-------------------------- EXTERNAL ISR FUNCTIONS --------------------------*/

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

/*-------------------------- RESET FUNCTIONS ----------------------------*/

/**
 * bfsi_reset - Reset line drivers using Blackfin GPIO
 * @reset_line: GPIO line hardware RESET is hooked to
 * @delay_us: RESET activation time (us).
 */
void bfsi_reset(int reset_line, int delay_us) {
	PRINTK("toggle reset\n");

#if (defined(CONFIG_BF533) || defined(CONFIG_BF532))
       	PRINTK("set reset to PF%d\n", reset_line);
  	bfin_write_FIO_DIR(bfin_read_FIO_DIR() | (1 << reset_line));
  	__builtin_bfin_ssync();

	bfin_write_FIO_FLAG_C((1 << reset_line));
	__builtin_bfin_ssync();
	udelay(delay_us);

	bfin_write_FIO_FLAG_S((1 << reset_line));
	__builtin_bfin_ssync();
#endif

#if defined(CONFIG_BF537)
	switch (reset_line) {
	case 1:
       		PRINTK("set reset to PF10\n");
                bfin_write_PORTF_FER(bfin_read_PORTF_FER() & 0xFBFF);
		__builtin_bfin_ssync();
		bfin_write_PORTFIO_DIR(bfin_read_PORTFIO_DIR() | 0x0400);
		__builtin_bfin_ssync();
		bfin_write_PORTFIO_CLEAR(1<<10);
		__builtin_bfin_ssync();
		udelay(delay_us);
		bfin_write_PORTFIO_SET(1<<10);
		__builtin_bfin_ssync();
		break;
	case 2:
                PRINTK("Error: cannot set reset to PJ11\n");
                break;
	case 3:
                PRINTK("Error: cannot set reset to PJ10\n");
                break;
	case 4:
                PRINTK("set reset to PF6\n");
                bfin_write_PORTF_FER(bfin_read_PORTF_FER() & 0xFFBF);
                __builtin_bfin_ssync();
		bfin_write_PORTFIO_DIR(bfin_read_PORTFIO_DIR() | 0x0040);
		__builtin_bfin_ssync();
		bfin_write_PORTFIO_CLEAR(1<<6);
		__builtin_bfin_ssync();
		udelay(delay_us);
		bfin_write_PORTFIO_SET(1<<6);
		__builtin_bfin_ssync();
		break;
	case 5:
                PRINTK("set reset to PF5\n");
                bfin_write_PORTF_FER(bfin_read_PORTF_FER() & 0xFFDF);
                __builtin_bfin_ssync();
		bfin_write_PORTFIO_DIR(bfin_read_PORTFIO_DIR() | 0x0020);
		__builtin_bfin_ssync();
		bfin_write_PORTFIO_CLEAR(1<<5);
		__builtin_bfin_ssync();
		udelay(delay_us);
		bfin_write_PORTFIO_SET(1<<5);
		__builtin_bfin_ssync();
		break;
	case 6:
                PRINTK("set reset to PF4\n");
                bfin_write_PORTF_FER(bfin_read_PORTF_FER() & 0xFFEF);
                __builtin_bfin_ssync();
		bfin_write_PORTFIO_DIR(bfin_read_PORTFIO_DIR() | 0x0010);
		__builtin_bfin_ssync();
		bfin_write_PORTFIO_CLEAR(1<<4);
		__builtin_bfin_ssync();
		udelay(delay_us);
		bfin_write_PORTFIO_SET(1<<4);
		__builtin_bfin_ssync();
		break;
	case 7:
                PRINTK("Error: cannot set reset to PJ5\n");
                break;
	case 8:
		PRINTK("set reset to PG8\n");
                bfin_write_PORTG_FER(bfin_read_PORTG_FER() & 0xFEFF);
                __builtin_bfin_ssync();
		bfin_write_PORTGIO_DIR(bfin_read_PORTGIO_DIR() | 0x0100);
		__builtin_bfin_ssync();
		bfin_write_PORTGIO_CLEAR(1 << 8);
		__builtin_bfin_ssync();
		udelay(delay_us);
		bfin_write_PORTGIO_SET(1 << 8);
		__builtin_bfin_ssync();
		break;
	case 9:
		PRINTK("set reset to PG9\n");
                bfin_write_PORTG_FER(bfin_read_PORTG_FER() & 0xFDFF);
                __builtin_bfin_ssync();
		bfin_write_PORTGIO_DIR(bfin_read_PORTGIO_DIR() | 0x0200);
		__builtin_bfin_ssync();
		bfin_write_PORTGIO_CLEAR(1 << 9);
		__builtin_bfin_ssync();
		udelay(delay_us);
		bfin_write_PORTGIO_SET(1 << 9);
		__builtin_bfin_ssync();
		break;
	case 14:
		PRINTK("set reset to PF14\n");
		bfin_write_PORTF_FER(bfin_read_PORTF_FER() & 0xBFFF);
                __builtin_bfin_ssync();
                bfin_write_PORTFIO_DIR(bfin_read_PORTFIO_DIR() | 0x4000);
                __builtin_bfin_ssync();
                bfin_write_PORTFIO_CLEAR(1<<14);
                __builtin_bfin_ssync();
                udelay(delay_us);
                bfin_write_PORTFIO_SET(1<<14);
                __builtin_bfin_ssync();
		break;

	default:
		PRINTK("Error: cannot set reset to %#x\n", reset_line);
        } /* switch (reset_line) */
#endif

	/*
	 * p24 3050 data sheet, allow 1ms for PLL lock, with
	 * less than 1ms (1000us) I found register 2 would have
	 * a value of 0 rather than 3, indicating a bad reset.
	 */
	udelay(2000);
}

/*-------------------------- SPORT FUNCTIONS ----------------------------*/

/* Init serial port but dont enable just yet, we need to set up DMA first */

/* Note a better way to write init code that works for both sports is
   in uClinux-dist /linux-2.6.x/sound/blackfin/bf53x_sport.c.  A
   structure is set up with the SPORT register addresses referenced to
   the base ptr of the structure.  This means one function can be used
   to init both SPORTs, just by changing the base addr of the ptr. */

#if defined(BFSI_SPORT0)
static void init_sport0(void)
{
	/* set up FSYNC and optionally SCLK using Blackfin Serial port */

	/* Note: internalclock option not working at this stage - Tx side
	   appears not to work, e.g. TFS pin never gets asserted. Not a
	   huge problem as the BF internal clock is not at quite the
	   right frequency (re-crystal of STAMP probably required), so
	   we really need an external clock anyway.  However it would
	   be nice to know why it doesnt work! */

	if (internalclock) {
		/* approx 2.048MHz PCLK */
		bfin_write_SPORT0_RCLKDIV(24);
		/* 8 kHz FSYNC with 2.048MHz PCLK */
		bfin_write_SPORT0_RFSDIV(255);
	} else {
		/* 8 kHz FSYNC with 2.048MHz PCLK  */
		bfin_write_SPORT0_RFSDIV(255);
	}

	/* external tx clk, not data dependant, MSB first */
	/* 8 bit word length */
	bfin_write_SPORT0_TCR2(7);
	bfin_write_SPORT0_TCR1(0);

	/* rx enabled, MSB first, internal frame sync */
	/* 8 bit word length */
	bfin_write_SPORT0_RCR2(7);
	if (internalclock) {
		bfin_write_SPORT0_RCR1(IRFS | IRCLK);
	}
	else {
		if (!tdm_slave)
			bfin_write_SPORT0_RCR1(IRFS);
	}

	/* Enable MCM 8 transmit & receive channels */
	bfin_write_SPORT0_MTCS0(0xFFFFFFFF >> (32 - (N_SLOTS)));
	bfin_write_SPORT0_MRCS0(0xFFFFFFFF >> (32 - (N_SLOTS)));

	/* MCM window size of 8 with 0 offset */
	bfin_write_SPORT0_MCMC1((((N_SLOTS) + 7) / 8 - 1) << 12);

	/*
	 * 0 bit delay between FS pulse and first data bit,
	 * multichannel frame mode enabled,
	 * multichannel tx and rx DMA packing enabled
	 */
	bfin_write_SPORT0_MCMC2(0x001c | (tdm_clkX2 ? REC_2FROM4 : REC_BYPASS));
}
#endif

#if defined(BFSI_SPORT1)
static void init_sport1(void)
{

        /* BF537 specific pin muxing configuration */
        bfin_write_PORT_MUX(bfin_read_PORT_MUX() | PGTE | PGRE /* | PGSE */);
        __builtin_bfin_ssync();
        bfin_write_PORTG_FER(bfin_read_PORTG_FER() | 0xFC00); /* PG10 to PG15 */
        __builtin_bfin_ssync();

	if (internalclock) {
		bfin_write_SPORT1_RCLKDIV(24);
		bfin_write_SPORT1_RFSDIV(255);
	} else {
		bfin_write_SPORT1_RFSDIV(255);
	}

	/* 8 bit word length */
	bfin_write_SPORT1_TCR2(7);
	bfin_write_SPORT1_TCR1(0);

	/* rx enabled, MSB first, internal frame sync */
	/* 8 bit word length */
	bfin_write_SPORT1_RCR2(7);
	if (internalclock) {
		bfin_write_SPORT1_RCR1(IRFS | IRCLK);
	} else {
		if (!tdm_slave)
			bfin_write_SPORT1_RCR1(IRFS);
	}

	/* Enable MCM 8 transmit & receive channels */
	bfin_write_SPORT1_MTCS0(0xFFFFFFFF >> (32 - (N_SLOTS)));
	bfin_write_SPORT1_MRCS0(0xFFFFFFFF >> (32 - (N_SLOTS)));

	/* MCM window size of N_SLOTS/8 with 0 offset */
	bfin_write_SPORT1_MCMC1(((N_SLOTS + 7) / 8 - 1) << 12);

	PRINTK("MTCS0=MRCS0=%#x, MCMC1=%#x\n", bfin_read_SPORT1_MTCS0(),
		bfin_read_SPORT1_MCMC1());

	/*
	 * 0 bit delay between FS pulse and first data bit,
	 * multichannel frame mode enabled,
	 * multichannel tx and rx DMA packing enabled
	 */
	bfin_write_SPORT1_MCMC2(0x001c | (tdm_clkX2 ? REC_2FROM4 : REC_BYPASS));
}
#endif

/**
 * init_dma_wc -
 *
 *  init DMA for autobuffer mode, but dont enable yet
 */
static int init_dma_wc(void)
{
#if (defined(CONFIG_BF533) || defined(CONFIG_BF532))
	/* Set up DMA1 to receive, map DMA1 to Sport0 RX */
	bfin_write_DMA1_PERIPHERAL_MAP(0x1000);
	bfin_write_DMA1_IRQ_STATUS(bfin_read_DMA1_IRQ_STATUS() | 0x2);
#endif
#if defined(CONFIG_BF537)
#if defined(BFSI_SPORT0)
	/* Set up DMA3 to receive, map DMA3 to Sport0 RX */
	bfin_write_DMA3_PERIPHERAL_MAP(0x3000);
	bfin_write_DMA3_IRQ_STATUS(bfin_read_DMA3_IRQ_STATUS() | 0x2);
#endif
#if defined(BFSI_SPORT1)
	/* Set up DMA5 to receive, map DMA5 to Sport1 RX */
	bfin_write_DMA5_PERIPHERAL_MAP(0x5000);
	bfin_write_DMA5_IRQ_STATUS(bfin_read_DMA5_IRQ_STATUS() | 0x2);
#endif
#endif
	/* Alloc: (2 bufs: ping + pong) * (samples per chunk) * N_SLOTS */
#if L1_DATA_A_LENGTH != 0
	iRxBuffer1 = (char *)l1_data_sram_alloc(2 * (samples_per_chunk) *
						(N_SLOTS));
#else
	{
	dma_addr_t addr;
	iRxBuffer1 = (char*)dma_alloc_coherent(NULL, 2 * (samples_per_chunk) *
					       (N_SLOTS), &addr, 0);
	}
#endif
	if (!iRxBuffer1) {
		printk(KERN_ERR "iRxBuffer allocation error.\n");
		return -1;
	}
	if (bfsi_debug) {
		PRINTK("iRxBuffer1 = 0x%x - size 2 * %d * %d = %dB\n",
			(int)iRxBuffer1, (samples_per_chunk), (N_SLOTS),
			2 * (samples_per_chunk) * (N_SLOTS));
	}

#if (defined(CONFIG_BF533) || defined(CONFIG_BF532))
	/* Start address of data buffer */
	bfin_write_DMA1_START_ADDR(iRxBuffer1);

	/* DMA inner loop count */
	bfin_write_DMA1_X_COUNT((samples_per_chunk) * (N_SLOTS));

	/* Inner loop address increment */
	bfin_write_DMA1_X_MODIFY(1);
	bfin_write_DMA1_Y_MODIFY(1);
	bfin_write_DMA1_Y_COUNT(2);

	/* Configure DMA1:
	 * 8-bit transfers, Interrupt on completion, Autobuffer mode */
	bfin_write_DMA1_CONFIG(WNR | WDSIZE_8 | DI_EN | 0x1000 | DI_SEL |DMA2D);

	/* Set up DMA2 to transmit, map DMA2 to Sport0 TX */
	bfin_write_DMA2_PERIPHERAL_MAP(0x2000);
	/* Configure DMA2 8-bit transfers, Autobuffer mode */
	bfin_write_DMA2_CONFIG(WDSIZE_8 | 0x1000 | DMA2D);
#endif
#if defined(CONFIG_BF537)

#if defined(BFSI_SPORT0)
	/* Start address of data buffer */
	bfin_write_DMA3_START_ADDR(iRxBuffer1);

	/* DMA inner loop count */
	bfin_write_DMA3_X_COUNT((samples_per_chunk) * (N_SLOTS));

	/* Inner loop address increment */
	bfin_write_DMA3_X_MODIFY(1);
	bfin_write_DMA3_Y_MODIFY(1);
	bfin_write_DMA3_Y_COUNT(2);

	/* Configure DMA3
	 * 8-bit transfers, Interrupt on completion, Autobuffer mode */
	bfin_write_DMA3_CONFIG(WNR | WDSIZE_8 | DI_EN | 0x1000 | DI_SEL |DMA2D);
	/* Set up DMA4 to transmit, map DMA4 to Sport0 TX */
	bfin_write_DMA4_PERIPHERAL_MAP(0x4000);
	/* Configure DMA4 8-bit transfers, Autobuffer mode */
	bfin_write_DMA4_CONFIG(WDSIZE_8 | 0x1000 | DMA2D);
#endif

#if defined(BFSI_SPORT1)
	/* Start address of data buffer */
	bfin_write_DMA5_START_ADDR((u32)iRxBuffer1);

	/* DMA inner loop count */
	bfin_write_DMA5_X_COUNT((samples_per_chunk) * (N_SLOTS));

	/* Inner loop address increment */
	bfin_write_DMA5_X_MODIFY(1);
	bfin_write_DMA5_Y_MODIFY(1);
	bfin_write_DMA5_Y_COUNT(2);

	/* Configure DMA5
	 * 8-bit transfers, Interrupt on completion, Autobuffer mode */
	bfin_write_DMA5_CONFIG(WNR | WDSIZE_8 | DI_EN | 0x1000 | DI_SEL |DMA2D);
	/* Set up DMA6 to transmit, map DMA6 to Sport1 TX */
	bfin_write_DMA6_PERIPHERAL_MAP(0x6000);
	/* Configure DMA2 8-bit transfers, Autobuffer mode */
	bfin_write_DMA6_CONFIG(WDSIZE_8 | 0x1000 | DMA2D);
#endif
#endif

/* Alloc: (2 bufs: ping + pong) * (samples per chunk) * N_SLOTS */
#if ((L1_DATA_A_LENGTH != 0) || (L1_DATA_B_LENGTH != 0))
	iTxBuffer1 = (char *)l1_data_sram_alloc(2 * (samples_per_chunk) *
						(N_SLOTS));
#else
	{
	dma_addr_t addr;
	iTxBuffer1 = (char *)dma_alloc_coherent(NULL, 2 * (samples_per_chunk) *
						(N_SLOTS), &addr, 0);
	}
#endif
	if (!iTxBuffer1) {
		printk(KERN_ERR "iTxBuffer allocation error.\n");
		l1_data_A_sram_free(iRxBuffer1);
		return -1;
	}

	if (bfsi_debug){
		PRINTK("iTxBuffer1 = 0x%x - size 2 * %d * %d = %dB\n",
			(int)iTxBuffer1, (samples_per_chunk), (N_SLOTS),
			2 *(samples_per_chunk) * (N_SLOTS));
	}

#if (defined(CONFIG_BF533) || defined(CONFIG_BF532))
	/* Start address of data buffer */
	bfin_write_DMA2_START_ADDR(iTxBuffer1);

	/* DMA inner loop count */
	bfin_write_DMA2_X_COUNT((samples_per_chunk) * (N_SLOTS));

	/* Inner loop address increment */
	bfin_write_DMA2_X_MODIFY(1);
	bfin_write_DMA2_Y_MODIFY(1);
	bfin_write_DMA2_Y_COUNT(2);
#endif

#if defined(CONFIG_BF537)

#if defined(BFSI_SPORT0)
	/* Start address of data buffer */
	bfin_write_DMA4_START_ADDR(iTxBuffer1);

	/* DMA inner loop count */
	bfin_write_DMA4_X_COUNT((samples_per_chunk) * (N_SLOTS));

	/* Inner loop address increment */
	bfin_write_DMA4_X_MODIFY(1);
	bfin_write_DMA4_Y_MODIFY(1);
	bfin_write_DMA4_Y_COUNT(2);
#endif

#if defined(BFSI_SPORT1)
	/* Start address of data buffer */
	bfin_write_DMA6_START_ADDR((u32)iTxBuffer1);

	/* DMA inner loop count */
	bfin_write_DMA6_X_COUNT((samples_per_chunk) * (N_SLOTS));

	/* Inner loop address increment */
	bfin_write_DMA6_X_MODIFY(1);
	bfin_write_DMA6_Y_MODIFY(1);
	bfin_write_DMA6_Y_COUNT(2);
#endif

#endif
	return 0;
} /* init_dma_wc () */

/**
 * isr_write_processing - works out which write buffer is available for writing
 *
 */
static inline u8 *isr_write_processing(void)
{
	u8 *writechunk;
	int writepingpong;
	int tx;

	/* select which ping-pong buffer to write to */
#if (defined(CONFIG_BF533) || defined(CONFIG_BF532))
	tx = (int)(bfin_read_DMA2_CURR_ADDR()) - (int)iTxBuffer1;
#endif
#if defined(CONFIG_BF537)
#if defined(BFSI_SPORT0)
	tx = (int)(bfin_read_DMA4_CURR_ADDR()) - (int)iTxBuffer1;
#endif
#if defined(BFSI_SPORT1)
	tx = (int)(bfin_read_DMA6_CURR_ADDR()) - (int)iTxBuffer1;
#endif
#endif

	/*
	 * for some reason x for tx tends to be 0xe and 0x4e, whereas
	 * x for rx is 0x40 and 0x80.  Note sure why they would be
	 * different.  We could perhaps consider having
	 * different interrupts for tx and rx side.  Hope this
	 * offset doesn't kill the echo cancellation, e.g. if we
	 * get echo samples in rx before tx has sent them!
	 *
	 */
	if (tx >= (N_SLOTS) * (samples_per_chunk)) {
		writechunk = (unsigned char *)iTxBuffer1;
		writechunk_first++;
		writepingpong = PING;
	} else {
		writechunk = (unsigned char *)iTxBuffer1 + (samples_per_chunk) *
			(N_SLOTS);
		writechunk_second++;
		writepingpong = PONG;
	}

	/* make sure writechunk actually ping pongs */
	if (writepingpong == lastwritepingpong) {
		writechunk_didntswap++;
	}
	lastwritepingpong = writepingpong;

	return writechunk;
} /* isr_write_processing () */

/**
 * isr_read_processing - works out which read buffer is available for reading
 *
 */
static inline u8 *isr_read_processing(void) {
	u8 *readchunk;
	int readpingpong;
	int rx, i;

	/* select which ping-pong buffer to read from */
#if (defined(CONFIG_BF533) || defined(CONFIG_BF532))
	rx = (int)bfin_read_DMA1_CURR_ADDR() - (int)iRxBuffer1;
#endif
#if defined(CONFIG_BF537)
#if defined(BFSI_SPORT0)
	rx = (int)bfin_read_DMA3_CURR_ADDR() - (int)iRxBuffer1;
#endif
#if defined(BFSI_SPORT1)
	rx = (int)bfin_read_DMA5_CURR_ADDR() - (int)iRxBuffer1;
#endif
#endif


	/* possible values for x are:
	 * e.g. N_SLOTS = 8
	 * 8 * samples_per_chunk = 0x40 at the end of the first row and
	 * 2 * 8 * samples_per_chunk = 0x80 at the end of the second row
	 * Rx address could be slightly higher than exact buffer boundary 0x40,
	 * so to identify PING buffer completion accept values ranging
	 * from PING end included to PONG end excluded, e.g., for N_SLOTS = 8
	 * consider PING if 0x40 <= rx < 0x80, otherwise consider PONG
	 *
	 * 0                0x40             0x80
	 * |______PING______|______PONG______|
	 */
	if ((rx >= (N_SLOTS) * (samples_per_chunk)) &&
	    (rx < 2 * (N_SLOTS) * (samples_per_chunk))) {
		readchunk = iRxBuffer1;
		readchunk_first++;
		readpingpong = PING;
	} else {
		readchunk = iRxBuffer1 + (samples_per_chunk) * (N_SLOTS);
		readchunk_second++;
		readpingpong = PONG;
	}

	log_readchunk = readchunk;

	/* memory of x for debug */

	for(i = 0; i < 4; i++)
	    last_x[i] = last_x[i+1];

	last_x[4] = rx;

	/* make sure readchunk actually ping pongs */

	if (readpingpong == lastreadpingpong) {
		readchunk_didntswap++;
		memcpy(bad_x, last_x, sizeof(bad_x));
	}

	lastreadpingpong = readpingpong;

	return readchunk;
} /* isr_read_processing () */

/**
 * irqreturn_t sport_rx_isr - called each time the DMA finishes one "line"
 *
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t sport_rx_isr(int irq, void *dev_id, struct pt_regs * regs)
#else
static irqreturn_t sport_rx_isr(int irq, void *dev_id)
#endif
{
	unsigned int 	start_cycles = cycles();
	int		i;
	u8 *            read_samples, *write_samples;

	/* confirm interrupt handling, write 1 to DMA_DONE bit */
#if (defined(CONFIG_BF533) || defined(CONFIG_BF532))
	bfin_write_DMA1_IRQ_STATUS(0x0001);
#endif
#if defined(CONFIG_BF537)
#if defined(BFSI_SPORT0)
	bfin_write_DMA3_IRQ_STATUS(0x0001);
#endif
#if defined(BFSI_SPORT1)
	bfin_write_DMA5_IRQ_STATUS(0x0001);
#endif
#endif
	__builtin_bfin_ssync();
	__builtin_bfin_ssync();

	/* Measure clock cycles since last interrupt.  This should always be
	 * less than 1ms.
	 */

	isr_between_diff = start_cycles - isr_between_prev;
	isr_between_prev = start_cycles;
	if (isr_between_diff > isr_between_worst)
		isr_between_worst = isr_between_diff;
	/* note hard coded for 500MHz clock, 500000 cycles between ISRs, so
	 * 1000000 means we didnt process within 2ms
	 */
	if (isr_between_diff > 1000000) {
		isr_between_skip++;
		isr_between_difflastskip = isr_between_diff;
	}

	/* read and write sample callbacks */

	read_samples = isr_read_processing();
	write_samples = isr_write_processing();

	isr_cycles_1 = cycles() - start_cycles;


	if (bfsi_isr_callback != NULL) {
		bfsi_isr_callback(read_samples, write_samples);
	}

	__builtin_bfin_ssync();

	isr_cycles_2 = cycles() - start_cycles;

	/* Call other soft irq */
  	for (i = 0; i < ARRAY_SIZE(soft_isr_table); i++) {
		if (likely(soft_isr_table[i].isr_enabled)) {
			if (likely(soft_isr_table[i].soft_isr != NULL))
			     soft_isr_table[i].soft_isr(soft_isr_table[i].data);
		}
		isr_cycles_3 = cycles() - start_cycles;
		if (likely(soft_isr_table[i].dma_enabled)) {
			if (likely(soft_isr_table[i].soft_dma_isr != NULL))
				soft_isr_table[i].soft_dma_isr(read_samples,
								write_samples);
		}
	}



	/*
	 * 	Simple IIR averager:
	 *
	 * 	y(n) = (1 - 1/TC)*y(n) + (1/TC)*x(n)
	 *
	 * 	After conversion to fixed point:
	 *
	 * 	2*y(n) = ((TC-1)*2*y(n) + 2*x(n) + half_lsb ) >> LTC
	 */

	isr_cycles_average = ((u32)(TC-1) * isr_cycles_average +
				(((u32)isr_cycles_last) << 1) + TC) >> LTC;

	if (isr_cycles_last > isr_cycles_worst)
		isr_cycles_worst = isr_cycles_last;

	/* we sample right at the end to make sure we count cycles used to
	 * measure cycles!
	 */
	isr_cycles_last = cycles() - start_cycles;

	return IRQ_HANDLED;
} /* sport_rx_isr () */

static int init_sport_interrupts(void)
{
#if defined(BFSI_SPORT0)
  	if(request_irq(IRQ_SPORT0_RX, sport_rx_isr,
		       IRQF_DISABLED, "sport rx", NULL) != 0) {
    		return -EBUSY;
	}
#endif
#if defined(BFSI_SPORT1)
  	if(request_irq(IRQ_SPORT1_RX, sport_rx_isr,
		       IRQF_DISABLED, "sport rx", NULL) != 0) {
    		return -EBUSY;
	}
#endif
	if (bfsi_debug) {
		PRINTK("ISR installed OK\n");
	}
#if (defined(CONFIG_BF533) || defined(CONFIG_BF532))
	/* enable DMA1 sport0 Rx interrupt */
	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() | 0x00000200);
	__builtin_bfin_ssync();
#endif
#if defined(CONFIG_BF537)
#if defined(BFSI_SPORT0)
	/* enable DMA3 sport0 Rx interrupt */
	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() | 0x00000020);
	__builtin_bfin_ssync();
#endif
#if defined(BFSI_SPORT1)
	/* enable DMA5 sport1 Rx interrupt */
	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() | 0x00000080);
	__builtin_bfin_ssync();
#endif
#endif
	return 0;
}

static void enable_dma_sport(void)
{
	/* enable DMAs */
#if (defined(CONFIG_BF533) || defined(CONFIG_BF532))
	bfin_write_DMA2_CONFIG(bfin_read_DMA2_CONFIG() | DMAEN);
	bfin_write_DMA1_CONFIG(bfin_read_DMA1_CONFIG() | DMAEN);
	__builtin_bfin_ssync();
	if(debug)
		PRINTK(KERN_INFO "DMA1/DMA2 enabled\n");

	/* enable sport0 Tx and Rx */

	bfin_write_SPORT0_TCR1(bfin_read_SPORT0_TCR1() | TSPEN);
	bfin_write_SPORT0_RCR1(bfin_read_SPORT0_RCR1() | RSPEN);
	__builtin_bfin_ssync();
	if(debug)
		PRINTK(KERN_INFO "SPORT0 enabled\n");
#endif
#if defined(CONFIG_BF537)
#if defined(BFSI_SPORT0)
	bfin_write_DMA4_CONFIG(bfin_read_DMA4_CONFIG() | DMAEN);
	bfin_write_DMA3_CONFIG(bfin_read_DMA3_CONFIG() | DMAEN);
	__builtin_bfin_ssync();
	if(debug)
		PRINTK(KERN_INFO "DMA3/DMA4 enabled\n");

	/* enable sport0 Tx and Rx */
	bfin_write_SPORT0_TCR1(bfin_read_SPORT0_TCR1() | TSPEN);
	bfin_write_SPORT0_RCR1(bfin_read_SPORT0_RCR1() | RSPEN);
	__builtin_bfin_ssync();
	if(debug)
		PRINTK(KERN_INFO "SPORT0 enabled\n");
#endif
#if defined(BFSI_SPORT1)
	bfin_write_DMA6_CONFIG(bfin_read_DMA6_CONFIG() | DMAEN);
	bfin_write_DMA5_CONFIG(bfin_read_DMA5_CONFIG() | DMAEN);
	__builtin_bfin_ssync();
	if(debug)
		PRINTK(KERN_INFO "DMA5/DMA6 enabled\n");

	/* enable sport1 Tx and Rx */

	bfin_write_SPORT1_TCR1(bfin_read_SPORT1_TCR1() | TSPEN);
	bfin_write_SPORT1_RCR1(bfin_read_SPORT1_RCR1() | RSPEN);
	__builtin_bfin_ssync();
	if(debug)
		PRINTK(KERN_INFO "SPORT1 enabled\n");
#endif
#endif

}

static void disable_sport(void)
{
#if (defined(CONFIG_BF533) || defined(CONFIG_BF532))
	/* disable sport0 Tx and Rx */
	bfin_write_SPORT0_TCR1(bfin_read_SPORT0_TCR1() & (~TSPEN));
	bfin_write_SPORT0_RCR1(bfin_read_SPORT0_RCR1() & (~RSPEN));
	__builtin_bfin_ssync();
	if(debug)
		PRINTK(KERN_INFO "SPORT0 disabled\n");

	/* disable DMA1 and DMA2 */
	bfin_write_DMA2_CONFIG(bfin_read_DMA2_CONFIG() & (~DMAEN));
	bfin_write_DMA1_CONFIG(bfin_read_DMA1_CONFIG() & (~DMAEN));
	__builtin_bfin_ssync();
	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() & (~0x00000200));
	__builtin_bfin_ssync();
	if(debug)
		PRINTK(KERN_INFO "DMA1/DMA2 disabled\n");
#endif

#if defined(CONFIG_BF537)
#if defined(BFSI_SPORT0)
	/* disable sport0 Tx and Rx */
	bfin_write_SPORT0_TCR1(bfin_read_SPORT0_TCR1() & (~TSPEN));
	bfin_write_SPORT0_RCR1(bfin_read_SPORT0_RCR1() & (~RSPEN));
	__builtin_bfin_ssync();
	if(debug)
		PRINTK(KERN_INFO "SPORT0 disabled\n");

	/* disable DMA3 and DMA4 */
	bfin_write_DMA4_CONFIG(bfin_read_DMA4_CONFIG() & (~DMAEN));
	bfin_write_DMA3_CONFIG(bfin_read_DMA3_CONFIG() & (~DMAEN));
	__builtin_bfin_ssync();
	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() & (~0x00000020));
	__builtin_bfin_ssync();
	if(debug)
		PRINTK(KERN_INFO "DMA3/DMA4 disabled\n");

#endif
#if defined(BFSI_SPORT1)
	/* disable sport1 Tx and Rx */
	bfin_write_SPORT1_TCR1(bfin_read_SPORT1_TCR1() & (~TSPEN));
	bfin_write_SPORT1_RCR1(bfin_read_SPORT1_RCR1() & (~RSPEN));
	__builtin_bfin_ssync();
	if(debug)
		PRINTK(KERN_INFO "SPORT1 disabled\n");

	/* disable DMA3 and DMA4 */
	bfin_write_DMA6_CONFIG(bfin_read_DMA6_CONFIG() & (~DMAEN));
	bfin_write_DMA5_CONFIG(bfin_read_DMA5_CONFIG() & (~DMAEN));
	__builtin_bfin_ssync();
	bfin_write_SIC_IMASK(bfin_read_SIC_IMASK() & (~0x00000080));
	__builtin_bfin_ssync();
	if(debug)
		PRINTK(KERN_INFO "DMA5/DMA6 disabled\n");
#endif
#endif
}

int bfsi_proc_read(char *buf, char **start, off_t offset,
		    int count, int *eof, void *data)
{
	int len;

	len = sprintf(buf,
		      "readchunk_first.........: %d\n"
		      "readchunk_second........: %d\n"
		      "readchunk_didntswap.....: %d\n"
		      "bad_x...................: %d %d %d %d %d\n"
		      "log_readchunk...........: %p\n"
		      "writechunk_first........: %d\n"
		      "writechunk_second.......: %d\n"
		      "writechunk_didntswap....: %d\n"
		      "isr_cycles_last.........: %d (%d %d %d)\n"
		      "isr_cycles_worst........: %d\n"
		      "isr_cycles_average......: %d\n"
		      "echo_sams...............: %d\n"
		      "isr_between_diff........: %d\n"
		      "isr_between_worst.......: %d\n"
		      "isr_between_skip........: %d\n"
		      "isr_between_difflastskip: %d\n"
		      ,
		      readchunk_first,
		      readchunk_second,
		      readchunk_didntswap,
		      bad_x[0],bad_x[1],bad_x[2],bad_x[3],bad_x[4],
		      log_readchunk,
		      writechunk_first,
		      writechunk_second,
		      writechunk_didntswap,
		      isr_cycles_last, isr_cycles_1, isr_cycles_2, isr_cycles_3,
		      isr_cycles_worst,
		      isr_cycles_average >> 1,
		      echo_sams,
		      isr_between_diff,
		      isr_between_worst,
		      isr_between_skip,
		      isr_between_difflastskip
		      );

	*eof = 1;
	return len;
}

int bfsi_proc_isr_stat(char *buf, char **start, off_t offset,
		    int count, int *eof, void *data)
{
	int len;

	len = sprintf(buf, "%d %d %d %d\n",
		      isr_cycles_1, isr_cycles_2, isr_cycles_3, isr_cycles_last
		      );
	*eof=1;
	return len;
}

static int proc_read_bfsi_freeze(char *buf, char **start, off_t offset,
                                 int count, int *eof, void *data)
{
	int len;

	*eof = 1;

	len = sprintf(buf, "%d\n", bfsi_freeze);

	return len;
}

static int proc_write_bfsi_freeze(struct file *file, const char *buffer,
                                  unsigned long count, void *data)
{
	int   new_freeze;
	char *endbuffer;

	new_freeze = simple_strtol (buffer, &endbuffer, 10);
	bfsi_freeze = new_freeze;

	return count;
}

static int proc_write_bfsi_reset(struct file *file, const char *buffer,
				 unsigned long count, void *data)
{
	int i;

	isr_cycles_worst = 0;
	isr_between_worst = 0;
	isr_between_skip = 0;
	isr_between_difflastskip = 0;
	readchunk_first = 0;
	readchunk_second = 0;
	readchunk_didntswap = 0;
	writechunk_first = 0;
	writechunk_second = 0;
	writechunk_didntswap = 0;
	for(i=0; i<5; i++)
		bad_x[i] = 0;

	return count;
}


/*
 * Wrapper for entire SPORT setup, returns 1 for success, 0 for failure.
 *
 *
 * The SPORT code is designed to deliver small arrays of size samples
 * every (125us * samples).  A ping-pong arrangement is used, so the
 * address of the buffer will alternate every call between two possible
 * values.
 *
 * The callback functions provide to the address of the current buffer
 * for the read and write channels.  Read means the data was just
 * read from the SPORT, so this is the "receive" PCM samples.  Write
 * is the PCM data to be written to the SPORT.
 *
 * The callbacks are called in the context of an interrupt service
 * routine, so treat any code them like an ISR.
 *
 * Once this function returns successfully the SPORT/DMA will be up
 * and running, and calls to the isr callback will start.  For testing
 * it is OK to set the callback function pointer to NULL, say if you
 * just want to look at the debug information.
 *
 * If debug==1 then "cat /proc/bfsi" will display some debug
 * information, something like:
 *
 *   readchunk_first.....: 9264
 *   readchunk_second....: 9264
 *   readchunk_didntswap.: 0
 *   writechunk_first....: 9264
 *   writechunk_second...: 9264
 *   writechunk_didntswap: 0
 *
 * If all is well then "readchunk_didntswap" and "writechunk_didntswap"
 * will be static and some very small number.  The first and second
 * values should be at most one value different.  These variables
 * indicate sucessful ping-pong operation.
 *
 * The numbers are incremented ever interrupt, for example if samples=8
 * (typical for zaptel), then we get one interrupt every ms, or 1000
 * interrupts per second.  This means the values for each first/second
 * entry should go up 500 times per second.
 *
 * 8 channels are sampled at once, so the size of the samples buffers
 * is 8*samples (typically 64 bytes for zaptel).
 *
 * TODO:
 *
 * 1/ It might be nice to modify this function allow user defined
 *    SPORT control reg settings, for example to change clock
 *    dividers and frame sync sources.  Or posible provide
 *    a bfsi_sport_set() function.
 *
 * 2/ Modify the callbacks to provide user-dfine context information.
 *
 */

static int bfsi_sport_init (void)
{


	/* DMA Interrupt Activation */
#if (defined(CONFIG_BF533) || defined(CONFIG_BF532))
	init_sport0();
#endif
#if defined(CONFIG_BF537)
#if defined(BFSI_SPORT0)
	init_sport0();
#endif
#if defined(BFSI_SPORT1)
	init_sport1();
#endif
#endif
	if (init_dma_wc()) {
		init_ok = 0;
		return init_ok;
	}

	enable_dma_sport();

	if (init_sport_interrupts())
		init_ok = 0;
	else
		init_ok = 1;

	return init_ok;

}

int bfsi_sport_register(
  void (*isr_callback)(u8 *read_samples, u8 *write_samples),
  int samples,
  int debug
)
{
	u32 flags;

	spin_lock_irqsave(&isr_table_lock, flags);
	bfsi_isr_callback = isr_callback;
	spin_unlock_irqrestore(&isr_table_lock, flags);

	return 0;
} /* bfsi_sport_register () */

/* Remove analog line ISR nd leave DMA ISR up and running */

void bfsi_sport_unregister(void)
{
	u32 flags;

	spin_lock_irqsave(&isr_table_lock, flags);
	bfsi_isr_callback = NULL;
	spin_unlock_irqrestore(&isr_table_lock, flags);
}

/* shut down SPORT operation cleanly */

static void bfsi_sport_close(void)
{
	disable_sport();

	if (init_ok) {
#if (defined(CONFIG_BF533) || defined(CONFIG_BF532))
		free_irq(IRQ_SPORT0_RX, NULL);
#endif
#if defined(CONFIG_BF537)
#if defined(BFSI_SPORT0)
		free_irq(IRQ_SPORT0_RX, NULL);
#endif
#if defined(BFSI_SPORT1)
		free_irq(IRQ_SPORT1_RX, NULL);
#endif
#endif
	}
#if ((L1_DATA_A_LENGTH != 0) || (L1_DATA_B_LENGTH != 0))
	l1_data_sram_free(iTxBuffer1);
	l1_data_sram_free(iRxBuffer1);
#else
	dma_free_coherent(NULL, 2 * (samples_per_chunk) * (N_SLOTS), iTxBuffer1, 0);
	dma_free_coherent(NULL, 2 * (samples_per_chunk) * (N_SLOTS), iRxBuffer1, 0);
#endif
	remove_proc_entry("bfsi", NULL);
	remove_proc_entry("bfsi_isr_stat", NULL);
	remove_proc_entry("bfsi_freeze", NULL);
	remove_proc_entry("bfsi_reset", NULL);
}


static int __init bfsi_init(void)
{
	struct proc_dir_entry *freeze, *reset;

	spin_lock_init(&isr_table_lock);
	printk(KERN_INFO "BFSI TLG Driver (%s). PCM slots:%d\n", svn_version, N_SLOTS);



	if (debug) {
		create_proc_read_entry("bfsi", 0, NULL, bfsi_proc_read, NULL);
		create_proc_read_entry("bfsi_isr_stat", 0, NULL,
				       bfsi_proc_isr_stat, NULL);
		freeze = create_proc_read_entry("bfsi_freeze", 0, NULL, proc_read_bfsi_freeze, NULL);
		freeze->write_proc = proc_write_bfsi_freeze;
		reset = create_proc_read_entry("bfsi_reset", 0, NULL, NULL, NULL);
		reset->write_proc = proc_write_bfsi_reset;

		bfsi_debug = debug;
	}

	init_ok = bfsi_sport_init();

	if (init_ok)
		return 0;

	return -1;
}

static void __exit bfsi_cleanup(void)
{
	/* DMA Interrupt De-activation */
	bfsi_sport_close();
	printk("%s: ...so long and thanks for all the fish\n", __FUNCTION__);
}

MODULE_LICENSE("GPL");
EXPORT_SYMBOL(bfsi_spi_init);
EXPORT_SYMBOL(bfsi_spi_u8_write);
EXPORT_SYMBOL(bfsi_spi_u8_read);
EXPORT_SYMBOL(bfsi_reset);
EXPORT_SYMBOL(bfsi_sport_register);
EXPORT_SYMBOL(bfsi_sport_unregister);
EXPORT_SYMBOL(bfsi_spi_disable);
EXPORT_SYMBOL(bfsi_spi_array_write);
EXPORT_SYMBOL(bfsi_spi_array_read);
EXPORT_SYMBOL(bfsi_spi_u8_write_array_read);
EXPORT_SYMBOL(bfsi_soft_irq_register);
EXPORT_SYMBOL(bfsi_soft_irq_free);
EXPORT_SYMBOL(bfsi_soft_irq_enable);
EXPORT_SYMBOL(bfsi_soft_irq_disable);
EXPORT_SYMBOL(bfsi_soft_dma_enable);
EXPORT_SYMBOL(bfsi_soft_dma_disable);

MODULE_DESCRIPTION("Blackfin Serial Interface Driver");
MODULE_AUTHOR("David Rowe");

module_init(bfsi_init);
module_exit(bfsi_cleanup);
