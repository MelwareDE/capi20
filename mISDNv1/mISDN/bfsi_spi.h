/*
  bfsi_spi.h
  Diego Serafin 
  Jan 24 2007
 
  Functions for Linux device drivers on the Blackfin that implement SPI bus
  interfacing.
  
  svn_version: $Id: bfsi_spi.h 40 2007-05-07 09:54:39Z diego $
*/
#ifndef BFSI_SPI_H_
#define BFSI_SPI_H_

#include <asm/blackfin.h>
#include <linux/delay.h>

/* Static variables */

static u16 chip_select_mask = 0;
static u16 spi_baud[8] = {0, 0, 0, 0, 0, 0, 0, 0};

/* enable this define to get verbose debugging info */
#define BFIN_SPI_DEBUG  1

#ifdef BFIN_SPI_DEBUG
#undef PRINTK
#define PRINTK(args...) printk(args)
#else
#define PRINTK(args...)
#endif

/**
 * bfsi_spi_u8_write - Write one octect of data on the specified SPI bus
 * @chip_select: The _number_ of the chip select line, e.g. to use 
 *               SPISEL2 chip_select = 2.
 * @tx_data: Octect to be transmitted on SPI bus
 *
 */
void bfsi_spi_u8_write(u16 chip_select, u8 tx_data)
{
	u16 flag_enable, flag;
	
	if (unlikely(chip_select >= ARRAY_SIZE(spi_baud))) {
		BUG();
		return;
	}

	flag = bfin_read_SPI_FLG();
	flag_enable = flag & ~(1 << (chip_select + 8));
	
	/* Set SPI bus speed */
	bfin_write_SPI_BAUD(spi_baud[chip_select]);

	
	/* drop SPISEL */
	bfin_write_SPI_FLG(flag_enable);

	/* read kicks off transfer, detect end by polling RXS */
	bfin_write_SPI_TDBR(tx_data);
	bfin_read_SPI_RDBR();
	__builtin_bfin_ssync();
	do {} while (!(bfin_read_SPI_STAT() & RXS) );

	udelay(2); //Makes the SPI communication in the first BR4-appliance more stable 

	/* raise SPISEL */
	bfin_write_SPI_FLG(flag); 

	udelay(2); //Makes the SPI communication in the first BR4-appliance more stable
}

/**
 * bfsi_spi_u8_read - Read one octect of data from the specified SPI bus
 * @chip_select: The _number_ of the chip select line, e.g. to use 
 *               SPISEL2 chip_select = 2.
 *	
 * Returns an u8 read from SPI device.
 *
 */
u8 bfsi_spi_u8_read(u16 chip_select)
{
	u16 flag_enable, flag, ret;
	
	if (unlikely(chip_select >= ARRAY_SIZE(spi_baud))) {
		BUG();
		return 0;
	}

	flag = bfin_read_SPI_FLG();
	flag_enable = flag & ~(1 << (chip_select + 8));

	/* Set SPI bus speed */
	bfin_write_SPI_BAUD(spi_baud[chip_select]);

	/* drop SPISEL */
	bfin_write_SPI_FLG(flag_enable); 

	/* 
	 * read kicks off transfer, detect end by polling RXS, we read the 
	 * shadow register to prevent another transfer being started.
	 * While reading we write a dummy tx value, 0xff. For the MMC card, 
	 * a 0 bit indicates the start of a command sequence therefore an 
	 * all 1's sequence keeps the MMC card in the current state.
	 */
	bfin_write_SPI_TDBR(0xff);
	bfin_read_SPI_RDBR(); 
	__builtin_bfin_ssync();
	do {} while (!(bfin_read_SPI_STAT() & RXS) );
	ret = bfin_read_SPI_SHADOW();
	__builtin_bfin_ssync();

	udelay(2); //Makes the SPI communication in the first BR4-appliance more stable
	
	/* raise SPISEL */
	bfin_write_SPI_FLG(flag); 

	udelay(2); //Makes the SPI communication in the first BR4-appliance more stable

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
	
	if (unlikely((buf_size <= 0) || (buf == NULL) || 
	    (chip_select >= ARRAY_SIZE(spi_baud)))) {
		BUG();
		return;
	}

	flag = bfin_read_SPI_FLG();
	flag_enable = flag & ~(1 << (chip_select + 8));
	
	/* Set SPI bus speed */
	bfin_write_SPI_BAUD(spi_baud[chip_select]);
	
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
	
	if (unlikely((buf_size <= 0) || (buf == NULL) || 
	    (chip_select >= ARRAY_SIZE(spi_baud)))) {
		BUG();
		return;
	}

	flag = bfin_read_SPI_FLG();
	flag_enable = flag & ~(1 << (chip_select + 8));
	
	/* Set SPI bus speed */
	bfin_write_SPI_BAUD(spi_baud[chip_select]);
	
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
	
	if (unlikely((buf_size <= 0) || (buf == NULL) || 
	    (chip_select >= ARRAY_SIZE(spi_baud)))) {
		BUG();
		return;
	}

	flag = bfin_read_SPI_FLG();
	flag_enable = flag & ~(1 << (chip_select + 8));
	
	/* Set SPI bus speed */
	bfin_write_SPI_BAUD(spi_baud[chip_select]);
	
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
 * bfsi_spi_init - Initialize SPI support on Blackfin processor
 * @baud: The SPI clk divider value, see Blackfin Hardware data book.
 * @new_chip_select_mask: the logical OR of all the chip selects to configure
 *
 * Notes:
 * new_chip_select_mask is the logical OR of all the chip selects we wish
 * to use for SPI, for example if we wish to use SPISEL2 and SPISEL3
 * chip_select_mask = (1<<2) | (1<<3).
 * baud:  The SPI clk divider value, see Blackfin Hardware data book,
 * maximum speed when baud = 2, minimum when baud = 0xffff (0 & 1 disable 
 * SPI port).
 * The maximum SPI clk for the Si Labs 3050 is 16.4MHz.  On a
 * 100MHz system clock Blackfin this means baud=4 minimum (12.5MHz).
 */
void bfsi_spi_init(int baud, u16 new_chip_select_mask) 
{
	u16 ctl_reg, flag;
	int cs, bit;

  	if (baud < 4) {
    		printk("baud = %d may mean SPI clock too fast for Si labs 3050"
	   		"consider baud == 4 or greater", baud);
  	}

	/* grab SPISEL/GPIO pins for SPI, keep level of SPISEL pins H */
	chip_select_mask |= new_chip_select_mask;
	flag = 0xff00 | chip_select_mask;

#if defined(CONFIG_BF537)
	/* we need to work thru each bit in mask and set the MUX regs */
	for(bit = 0; bit < 8; bit++) {
		if (chip_select_mask & (1 << bit)) {
			u16 port_f_fer = bfin_read_PORTF_FER();
			u16 port_mux   = bfin_read_PORT_MUX();
			PRINTK("SPI CS bit: %d enabled\n", bit);
			cs = bit;
			spi_baud[bit] = baud;
			switch (cs) {
			case 1:
				bfin_write_PORTF_FER(port_f_fer | 0x3c00);
				__builtin_bfin_ssync();
				break;
			case 2:
			case 3:
				bfin_write_PORT_MUX(port_mux | PJSE_SPI);
				__builtin_bfin_ssync();
				bfin_write_PORTF_FER(port_f_fer | 0x3800);
				__builtin_bfin_ssync();
				break;
			case 4:
			      bfin_write_PORT_MUX(port_mux | PFS4E_SPI);
				__builtin_bfin_ssync();
				bfin_write_PORTF_FER(port_f_fer | 0x3840);
				__builtin_bfin_ssync();
				break;
			case 5:
				bfin_write_PORT_MUX(port_mux | PFS5E_SPI);
				__builtin_bfin_ssync();
				bfin_write_PORTF_FER(port_f_fer | 0x3820);
				__builtin_bfin_ssync();
				break;
			case 6:
				bfin_write_PORT_MUX(port_mux | PFS6E_SPI);
				__builtin_bfin_ssync();
				bfin_write_PORTF_FER(port_f_fer | 0x3810);
				__builtin_bfin_ssync();
				break;
			case 7:
				bfin_write_PORT_MUX(port_mux | PJCE_SPI);
				__builtin_bfin_ssync();
				bfin_write_PORTF_FER(port_f_fer | 0x3800);
				__builtin_bfin_ssync();
				break;
			}
		}
	}
#endif
  	/* note TIMOD = 00 - reading SPI_RDBR kicks off transfer */
  	ctl_reg = SPE | MSTR | CPOL | CPHA | SZ;
  	bfin_write_SPI_FLG(flag);
  	bfin_write_SPI_CTL(ctl_reg);
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
			spi_baud[bit] = 0;
			
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

#undef PRINTK
#endif /*BFSI_SPI_H_*/
