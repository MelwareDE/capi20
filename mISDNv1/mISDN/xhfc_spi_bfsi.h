/* $Id:$
 *
 * Blackfin uClinux SPI Bus support for xhfc_su.c
 *
 * Authors : Diego Serafin
 * Contact : diego.serafin@gmail.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef _XHFC_SPI_BFSI_H_
#define _XHFC_SPI_BFSI_H_

//#include <linux/bfsi_spi.h>
#include "xhfc_su.h"

#undef XHFC_SPI_DEBUG

#ifdef XHFC_SPI_DEBUG
#define PRINTK(args...) printk(args)
#else
#define PRINTK(args...)
#endif

/* Default SPI configuration value for SPI BFSI hardware */

#define XHFC_SPI_SPEED_DEFAULT	8
#define XHFC_SPI_SEL_DEFAULT	5


/*---------------------------------------------------------------------------*/

/*
 * I/O INTERFACE between SPI protocol driver and XHFC chip
 *
 * XHFC SPI register access consists of two transactions: an address write
 * transaction first and a data read or write transaction afterwards.
 *
 * SPI transactions have either a length of 16 bit for single byte access or
 * 40 bit for high performance accesses. The first byte is a control byte
 * in both cases, whereas the following bits are either one data byte or
 * four data bytes. Control and data bytes are transmitted msb first.
 * between the controller and memory buffers.
 *
 * SPI control byte.
 *
 * See tables below for control byte construction. Bit 'R' and 'A' are used to
 * specify Read/write and Address/data transaction types. The meaning of M' bit
 * in position 5 depends on 'A' bit value. Broadcasting can be enabled with an
 * address transaction and single or multiple data bytes is selected with a data
 * transaction.
 *
 * Up to 16 microchips of the XHFC series can be connected to the SPI interface
 * and can operate with the same SPISEL# signal. The desired chip is selected
 * with the device address bits C3..C0 within an address transaction. Every
 * XHFC microchip must specify its address by connecting DN3..DN0 pins (Device
 * Number) to ground or power supply. A microchip is selected when [C3 C2 C1 C0]
 * = [DN3 DN2 DN1 DN0] (all numbers in the range 0..15 are allowed).
 *
 * In addiction to the chip selection, an SPI write access writes its data into
 * ALL connected XHFC microchips if broadcast is used. SPI read access with
 * broadcast bit enabled execute the register read access in ALL connected XHFC
 * microchips, but only the specified chip delivers its data to the SPI master.
 *
 * ADDRESS transaction (A = 1) control byte:
 * +-----------------------------+
 * |Control bit nr               |
 * | 7 | 6 | 5 | 4 |  3  2  1  0 |
 * +---+---+---+---+-------------+
 * | R | A | B | 0 | C3 C2 C1 C0 |
 * +---+---+---+---+-------------+
 *
 * R = 0 for WRITE and 1 for READ
 * A = 1 (ADDRESS transaction)
 * B = 0 for unicast ad 1 for broadcast to all devices
 * bit at position 4 must always be 0.
 * C3..C0 Device address (to identify specific chip on a SPI bus)
 *
 * DATA transaction (A = 0) control byte:
 * +----------------------------+
 * |Control bit nr              |
 * | 7 | 6 | 5 | 4 | 3  2  1  0 |
 * +---+---+---+---+------------+
 * | R | A | M | 0 | 0  0  0  0 |
 * +---+---+---+---+------------+
 *
 * R = 0 for WRITE and 1 for READ
 * A = 0 (DATA transaction)
 * M = 0 for SINGLE data byte transfer or 1 for 4 data byte transfer
 * bit from position 4 to position 0 must be always zero in data transaction
 *
 * (freely adapted from XHFC-2SU4SU/XHFC-4SU Data Sheet, March 2006 edition)
 */

/*
 * XHFC SPI control byte definitions
 *
 */

#define SPI_ADDR	0x40
#define SPI_DATA	0x00
#define SPI_RD		0x80
#define SPI_WR		0x00
#define SPI_BROAD	0x20
#define SPI_MULTI	0x20

/* Nr of devices that could share the same spi select */
#define SPI_MAX_XHFC	16

/**
 * write_xhfcregptr_nolock -- write the XHFC register pointer without spinlock
 * @xhfc: the device descriptor
 * @value: new register value
 *
 * Write the XHFC register pointer to XHFC chip. It does not use spinlock to
 * protect from interrupt access to the same code. Use it only inside other
 * spinlock-protected functions.
 */
static inline void write_xhfcregptr_nolock(xhfc_t * xhfc, u8 reg_addr)
{
	u8 tx_buf[] = {
		/* SPI write: register address (control byte '01X0 CCCC') */
		SPI_ADDR | SPI_WR | xhfc->chipidx,
		reg_addr
	};

	bfsi_spi_array_write(xhfc->pi->spi_sel, tx_buf, sizeof(tx_buf));

	PRINTK(KERN_INFO "%s: written regptr = %#x\n", __FUNCTION__, reg_addr);
}

/* Writes the XHFC register address pointer */

/**
 * write_xhfcregptr -- write the XHFC register pointer to XHFC
 * @xhfc: the device descriptor
 * @value: new register value
 *
 * Write the XHFC register pointer to XHFC chip.
 */
static inline void write_xhfcregptr(xhfc_t * xhfc, u8 reg_addr)
{
	u32 flags;

	spin_lock_irqsave(xhfc->lock, flags);
	write_xhfcregptr_nolock(xhfc, reg_addr);
	spin_unlock_irqrestore(xhfc->lock, flags);

	PRINTK(KERN_INFO "%s: written regptr = %#x\n", __FUNCTION__, reg_addr);
}

/*
 * Register address readback.
 *
 * When the non-multiplexed modes are used, the address read access can be
 * executed to readback the address of the currently selected register.
 *
 * The address read-back capability is useful for interrupt procedures, e.g.,
 * to save and restore the previous state. XHFC ISR should be coded this way:
 *
 *    - interrupt procedure: - execute address read access and store the address
 *    - ... (execute the interrupt service routine)
 *    - address write access to restore the previous address
 *
 * This procedure is important to avoid data read or write to an unexpected
 * register address after a register read or write access has been split by an
 * interrupt service routine which executes any access to the XHFC-2S4U / 4SU.
 *
 */

/**
 * read_xhfcregptr -- read the currently selected register from XHFC
 * @xhfc: the device descriptor
 *
 * Read the currently selected register from XHFC chip by executing a SPI
 * address read access.
 */
static inline u8 read_xhfcregptr(xhfc_t * xhfc)
{
	u8 rx_buf = 0;
	u32 flags;

	spin_lock_irqsave(xhfc->lock, flags);
	/* Set register address read command (control byte '11X0 CCCC') */
	bfsi_spi_u8_write_array_read(xhfc->pi->spi_sel, SPI_ADDR | SPI_RD,
		&rx_buf, sizeof(rx_buf));
	spin_unlock_irqrestore(xhfc->lock, flags);

	PRINTK(KERN_INFO "%s: read SPI ADDR = %x\n", __FUNCTION__, rx_buf);

	return rx_buf;
}


/*---------------------------------------------------------------------------*/

/*
 * Register read access.
 *
 * Register read consist always of a transaction sequence with an address write
 * transaction first, and one or several data read transaction afterwards.
 * XHFC-2SU/4SU offers four ways of executing register read accesses:
 *
 * + A register read access is a sequence of one SPI write address transaction
 *   and one SPI read data transaction. The first transaction specifies the
 *   register address (control byte 01X0 CCCC), second transaction transfers
 *   register value to SPI master (control byte 1000 0000). X=1 enables
 *   broadcast and CCCC is ignored, X=0 disables it and CCCC must specify chip
 *   address.
 *
 * + It is allowed to execute multiple data read transactions to the same
 *   register address without address write transactions in between (typically
 *   used for receiving FIFO data).
 *
 * + Handling multiple bytes from the same register is available with the 40bit
 *   read transaction. With the first 16bit transaction the SPI master specifies
 *   register address (control byte 01X0 CCCC), then four register bytes are
 *   transferred to the SPI master (control byte 1010 0000).
 *   (Broadcasting is handled in the same way as explained in the first point).
 *
 * + A combination of second and third points can be used to read a multiple
 *   of four bytes. First the control byte '01X0 CCCC' executes the address
 *   write transaction, and afterwards four data bytes can be read several times
 *   with the control byte '1010 0000' in each 40bit read trasaction.
 *
 *
 */

/**
 * read_xhfc -- read from XHFC chip
 * @xhfc: the device descriptor
 * @reg_addr: register to be accessed
 *
 * Read a single byte from a register.
 */
static inline u8 read_xhfc(xhfc_t * xhfc, u8 reg_addr)
{
	u8 rx_buf = 0;
	u32 flags;

	spin_lock_irqsave(xhfc->lock, flags);
	write_xhfcregptr_nolock(xhfc, reg_addr);
	bfsi_spi_u8_write_array_read(xhfc->pi->spi_sel, SPI_DATA | SPI_RD,
		&rx_buf, sizeof(rx_buf));
	spin_unlock_irqrestore(xhfc->lock, flags);

	PRINTK(KERN_INFO "%s: read [%#x] = %x-%x\n", __FUNCTION__, reg_addr,
		rx_buf);

	return rx_buf;
}

/**
 * read4u8_xhfc -- read four bytes from XHFC chip storing data in a 4B buffer
 * @xhfc: the device descriptor
 * @reg_addr: register to be accessed
 * @rx_buf: a read buffer allocated by the caller at least 4 bytes wide.
 *
 * read four bytes from the same register address.
 */
static inline u8 * read4u8_xhfc(xhfc_t * xhfc, u8 reg_addr, u8* rx_buf)
{
	u32 flags;

	spin_lock_irqsave(xhfc->lock, flags);
	write_xhfcregptr_nolock(xhfc, reg_addr);
	bfsi_spi_u8_write_array_read(xhfc->pi->spi_sel,
		SPI_DATA | SPI_RD | SPI_MULTI, rx_buf, 4);
	spin_unlock_irqrestore(xhfc->lock, flags);

	PRINTK(KERN_INFO "%s: read32ptr [%#x] = %x%x%x%x\n", __FUNCTION__,
		reg_addr, rx_buf[3], rx_buf[2], rx_buf[1], rx_buf[0]);
	return rx_buf;
}

/**
 * read32_xhfc -- read four bytes from XHFC chip
 * @xhfc: the device descriptor
 * @reg_addr: register to be accessed
 *
 * read four bytes from the same register address.
 */
static inline __u32 read32_xhfc(xhfc_t * xhfc, u8 reg_addr)
{
	__u32 rx_buf = 0;

	read4u8_xhfc (xhfc, reg_addr, (u8 *)&rx_buf);

	PRINTK(KERN_INFO "%s: read32 [%#x] = %#x\n", __FUNCTION__, reg_addr,
		rx_buf);
	return rx_buf;
}

/*
 * Register write access.
 *
 * Register read consist always of a transaction sequence with an address write
 * transaction first, and one or several data write transaction afterwards.
 * XHFC-2SU/4SU offers four ways of executing register read accesses:
 *
 * + A register write access is a sequence of two SPI write transactions.
 *   With the first transaction the SPI master specifies the register address
 *   (control byte 01X0 CCCC), afterwards the new register value is transfered
 *   to XHFC chip (control byte 0000 0000).
 *   X=1 enables broadcast and CCCC is ignored, X=0 disables it and CCCC must
 *   specify chip address.
 *
 * + It is allowed to execute multiple data write transactions to the same
 *   register address without address write transactions in between (typically
 *   used for transmitting FIFO data).
 *
 * + Writing multiple bytes into the same register is available with the 40bit
 *   wriet transaction. With the first 16bit transaction the SPI master specifies
 *   register address (control byte 01X0 CCCC), then four new register bytes are
 *   transferred from the SPI master to XHFC chip (control byte 0010 0000).
 *   (Broadcasting is handled in the same way as explained in the first point).
 *
 * + A combination of second and third points can be used to write a multiple
 *   of four bytes. First the control byte '01X0 CCCC' executes the address
 *   write transaction, and afterwards four data bytes can be written several
 *   times with the control byte '0010 0000' in each 40bit write trasaction.
 *
 *
 */

/**
 * write_xhfc -- write to XHFC chip
 * @xhfc: the device descriptor
 * @reg_addr: register to be updated
 * @value: new register value
 *
 * Write a single byte from a register.
 */
static inline void write_xhfc(xhfc_t * xhfc, u8 reg_addr, u8 value)
{
	u32 flags;
	u8 tx_buf[] = {
		/* Set register address SPI write (control byte '01X0 CCCC') */
		SPI_ADDR | SPI_WR | xhfc->chipidx,
		reg_addr,
		/* Set register write */
		SPI_DATA | SPI_WR,
		value
	};

	spin_lock_irqsave(xhfc->lock, flags);
	bfsi_spi_array_write(xhfc->pi->spi_sel, tx_buf, sizeof(tx_buf));
	spin_unlock_irqrestore(xhfc->lock, flags);

	PRINTK(KERN_INFO "%s: write [%#x] = %#x\n", __FUNCTION__, reg_addr, value);
}

/**
 * write4u8_xhfc -- write four bytes to XHFC chip
 * @xhfc: the device descriptor
 * @reg_addr: register to be updated
 * @value: new register values
 *
 * write four bytes to the same register address.
 */
static inline void write4u8_xhfc(xhfc_t * xhfc, u8 reg_addr, u8 *value)
{
	u32 flags;
	u8 tx_buf[] = {
		/* Set register address SPI write (control byte '01X0 CCCC') */
		SPI_ADDR | SPI_WR | xhfc->chipidx,
		reg_addr,
		/* Set register write */
		SPI_DATA | SPI_WR | SPI_MULTI,
		value[0], value[1], value[2], value[3]
	};

	spin_lock_irqsave(xhfc->lock, flags);
	bfsi_spi_array_write(xhfc->pi->spi_sel, tx_buf, sizeof(tx_buf));
	spin_unlock_irqrestore(xhfc->lock, flags);

	PRINTK(KERN_INFO "%s: write [%#x] = MSB(last tx)-%x-%x-%x-%x-LSB(first tx)\n",
		__FUNCTION__, reg_addr, value[3], value[2], value[1], value[0]);
}

/**
 * write32_xhfc -- write an u32 to XHFC chip
 * @xhfc: the device descriptor
 * @reg_addr: register to be updated
 * @value: new register values laid on a single u32
 *
 * write four bytes to the same register address.
 */
static inline void write32_xhfc(xhfc_t * xhfc, u8 reg_addr, u32 value)
{
	write4u8_xhfc(xhfc, reg_addr, (u8 *)&value);
}


/*
 * Read* register access.
 *
 * Some register must be read with an indirect method, called Read*, written as
 * 'r*' in register tables and here translated into 'sread'. This refers to all
 * readable registers in the range 0xC0..0xFF called 'target registers' in the
 * manual.
 *
 * The Read* access performs two consecutive accesses to XHFC chip:
 *
 *   * First, a read access to target register must be executed and read
 *     value discarded.
 *
 *   * Then the actual register value could be read from register R_INT_DATA
 *
 * Read* method SHOULD be used to access R_RAM_DATA, A_SL_CFG, A_CH_MSK,
 * A_CON_HDLC, A_SUBCH_CFG, A_CHANNEL, A_FIFO_SEQ and A_FIFO_CTRL registers.
 */

/**
 * sread_xhfc -- read from XHFC chip using Read* method
 * @xhfc: the device descriptor
 * @reg_addr: register to be accessed
 *
 * Read a single byte from a target register using 'Read*' method.
 */
static inline u8 sread_xhfc(xhfc_t * xhfc, u8 reg_addr)
{
	/* SPI read: data to be discarded */
	//read_xhfc(xhfc, reg_addr);
	u32 flags;
	u8 rx_buf = 0;
	u8 tx_buf[] = {
		/* SPI write: register address (control byte '01X0 CCCC') */
		SPI_ADDR | SPI_WR | xhfc->chipidx,
		R_INT_DATA
	};

	spin_lock_irqsave(xhfc->lock, flags);
	write_xhfcregptr_nolock(xhfc, reg_addr);
	bfsi_spi_u8_write_array_read(xhfc->pi->spi_sel, SPI_DATA | SPI_RD,
		&rx_buf, sizeof(rx_buf));

	/* SPI read: valid data to be returned */
	bfsi_spi_array_write(xhfc->pi->spi_sel, tx_buf, sizeof(tx_buf));
	bfsi_spi_u8_write_array_read(xhfc->pi->spi_sel, SPI_DATA | SPI_RD,
		&rx_buf, 1);

	spin_unlock_irqrestore(xhfc->lock, flags);

	return rx_buf;
}

/*
 * SPI initalization wrappers
 *
 */
static int xhfc_spi_init(xhfc_pi *pi)
{
	bfsi_spi_init_global();
	return 0;
}

static void xhfc_spi_disable(u16 spi_sel)
{
	bfsi_spi_disable(1 << spi_sel);
}

#endif /* _XHFC_SPI_BFSI_H_ */
