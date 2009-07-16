/* $Id:$
 *
 * SPI Bus support for CologneChip XHFC
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

#ifndef XHFC_SPI_H_
#define XHFC_SPI_H_

#define CONFIG_SPI_BLACKFIN

#ifdef CONFIG_SPI_BLACKFIN
	#include "xhfc_spi_bfsi.h"
#else
	#error Unsupported SPI architecture: check xhfc_spi.h
#endif

#endif /*XHFC_SPI_H_*/
