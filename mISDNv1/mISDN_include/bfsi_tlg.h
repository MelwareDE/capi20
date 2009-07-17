/*
  bfsi_tlg.h
  Diego Serafin 
  Dec 14 2006
 
  Functions for Linux device drivers on the Blackfin that
  support interfacing the Blackfin to Silicon Labs chips.
*/

#ifndef __BFSI_TLG_H_
#define __BFSI_TLG_H_

/* Blackfin Serial Interface Configuration variables */

#ifndef ENABLE_128MS
#define N_SLOTS         8
#else
#define N_SLOTS         16 //DPN: 128 ms support, only even time slots are used
#endif

#define SAMPLES_PER_CHUNK	8  /* Must be equal to ZT_CHUNKSIZE */
#define FX_MAX_PORTS    	16 /* max FX(s|o) ports number per SPI bus */

#define RESET_I			14  /* ISDN chips reset: PF14 */
#define RESET_A			9  /* Analog SLICs chips reset: PG9 */

int  bfsi_sport_register
	(void (*isr_callback)(u8 *read_samples, u8 *write_samples), int samples,
	int debug);
void bfsi_sport_unregister(void);
void bfsi_reset(int reset_bit, int delay_us);
int  bfsi_soft_irq_register 
	(void(*isr)(void*), void(*dma_isr)(u8*, u8*), void *data, 
	int isr_enabled, int dma_enabled);
void bfsi_soft_irq_free(int);
void bfsi_soft_irq_enable(int);
void bfsi_soft_irq_disable(int);
void bfsi_soft_dma_enable(int);
void bfsi_soft_dma_disable(int);

/* SPI functions */
void bfsi_spi_init(int baud, u16 new_chip_select_mask);
u8   bfsi_spi_u8_read(u16 chip_select);
void bfsi_spi_u8_write(u16 chip_select, u8 tx_data);
void bfsi_spi_disable(u16 cs_mask_to_remove);
void bfsi_spi_array_write(u16 chip_select, u8 *buf, int buf_size);
void bfsi_spi_array_read(u16 chip_select, u8 *buf, int buf_size);
void bfsi_spi_u8_write_array_read(u16 chip_select, u8 write_buf, u8 *buf, 
	int buf_size);


#endif /* __BFSI_TLG_H_ */
