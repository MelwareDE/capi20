/*
  bfsi.h
  David Rowe 
  Dec 1 2006
 
  Functions for Linux device drivers on the Blackfin that
  support interfacing the Blackfin to Silicon Labs chips.
*/

#ifndef __BFSI__

#define __BFSI__
					//There are two types of clients hooking callbacks to the bfsi driver 
#define FXS_FXO_CLIENT	0		//Driver which controls the FXO/FXS interfaces
#define FWFXS_CLIENT    1              	//Driver which controls the TI codec four wire interfaces 


/* Bit on port F to be used as reset */

#define FXO_FXS_RESET 15


void bfsi_hfc_reset(int delay_us);

void bfsi_spi_init(int baud, u16 chip_select_mask);
void bfsi_spi_init_global(void);
void bfsi_spi_array_write(u16 chip_select, u8 *buf, int buf_size);
void bfsi_spi_array_read(u16 chip_select, u8 *buf, int buf_size);
void bfsi_spi_u8_write_array_read(u16 chip_select, u8 write_buf, u8 *buf,int buf_size);
void bfsi_spi_disable(u16 cs_mask_to_remove);



int bfsi_sport0_init(int samples, int debug);

int bfsi_hook_callback(void (*isr_callback)(u8 *read_samples, u8 *write_samples), int client_type);

void bfsi_sport0_close(void);
void fxs_workaround(void);


int bfsi_soft_irq_register(void(*isr)(void*), void(*dma_isr)(u8*, u8*),
			   void *data, int isr_enabled, int dma_enabled);
void bfsi_soft_irq_free (int nr);
void bfsi_soft_irq_enable (int nr);

void bfsi_soft_irq_disable (int nr);
void bfsi_soft_dma_enable (int nr);
void bfsi_soft_dma_disable (int nr);





#endif

