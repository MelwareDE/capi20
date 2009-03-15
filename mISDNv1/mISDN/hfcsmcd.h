/* hfcsmcd.h
 * HFC-S mini register definitions
 * (C) 2007 Copyright Cologne Chip AG
 * (support@CologneChip.com)
 *
 * Dual-license
 * ------------
 * Cologne Chip AG, Eintrachtstr. 113, 50668 Koeln, Germany, provides this header 
 * file (software) under a dual-license.
 * The licensee can choose from the following two licensing models:
 *   * For GPL (free) distributions, see the 'License - GPL'
 *   * For commercial distributions, see the 'License - Commercial'
 *
 * 
 * License - GPL
 * -------------
 * This software is free software; you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software 
 * Foundation; either version 2, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with 
 * this software; if not, write to the Free Software Foundation, Inc., 675 Mass 
 * Ave, Cambridge, MA 02139, USA.
 *
 *
 * License - Commercial
 * --------------------
 * (C) 2007 Copyright Cologne Chip AG
 * All rights reserved.
 * Contact: support@CologneChip.com
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright notice, 
 *       this list of conditions and the following disclaimer.
 *     * Redistributions of source code must mark all modifications explicitly as 
 *       such, if any are made.
 *     * For redistributing and use of this software in binary form, none of the 
 *       provisions above applies.
 *
 * This software is provided by Cologne Chip AG "as is" and any express or implied 
 * warranties, including, but not limited to, the implied warranties of 
 * merchantability and fitness for a particular purpose are disclaimed. In no event 
 * shall Cologne Chip AG be liable for any direct, indirect, incidental, special, 
 * exemplary, or consequential damages (including, but not limited to, procurement 
 * of substitute goods or services; loss of use, data, or profits; or business 
 * interruption) however caused and on any theory of liability, whether in contract, 
 * strict liability, or tort (including negligence or otherwise) arising in any way 
 * out of the use of this software, even if advised of the possibility of such 
 * damage.
 * __________________________________________________________________________________
 *
 *   File name:     hfcsmcd.h
 *   File content:  This file contains the HFC-S mini register definitions.
 *   Creation date: 18.06.2007 15:49
 *   Creator:       Genero 3.6
 *   Data base:     HFC XML 1.6 for HFC-S mini and HFC-S USB
 *   Address range: 0x00 - 0xFC
 * 
 *   The information presented can not be considered as assured characteristics.
 *   Data can change without notice. Please check version numbers in case of doubt.
 * 
 *   For further information or questions please contact support@CologneChip.com
 * __________________________________________________________________________________
 * 
 *   WARNING: This file has been generated automatically and should not be
 *            changed to maintain compatibility with later versions.
 */

#ifndef _HFCSMCD_H_
#define _HFCSMCD_H_


/*
 *  Common chip information:
 */

	#define CHIP_NAME		"HFC-S mini"
	#define CHIP_TITLE		"ISDN HDLC FIFO controller with S/T interface and integrated FIFOs"
	#define CHIP_MANUFACTURER	"Cologne Chip"
	#define CHIP_ID			0x05
	#define CHIP_REGISTER_COUNT	71
	#define CHIP_DATABASE		"Version HFC-XMLHFC XML 1.6 for HFC-S mini and HFC-S USB - GeneroGenero 3.6 "


/*
 *  Begin of HFC-S mini register definitions.
 */

#define R_CIRM 0x00 // register address, write only
	#define M_SRES  0x08  // mask bit 3
	#define SET_V_SRES(R,V)  (R = (__u8)((R & (__u8)(M_SRES ^ 0xFF)) | (__u8)((V & 0x01) << 3)))
	#define GET_V_SRES(R)    (__u8)((R & M_SRES) >> 3)


#define A_Z1 0x04 // register address, read only
	#define M_Z1  0xFF  // mask bits 0..7
	#define GET_V_Z1(R)    (__u8)(R & M_Z1)


#define A_Z2 0x06 // register address, read only
	#define M_Z2  0xFF  // mask bits 0..7
	#define GET_V_Z2(R)    (__u8)(R & M_Z2)


#define R_RAM_ADDR0 0x08 // register address, write only
	#define M_RAM_ADDR0  0xFF  // mask bits 0..7
	#define SET_V_RAM_ADDR0(R,V)  (R = (__u8)((R & (__u8)(M_RAM_ADDR0 ^ 0xFF)) | (__u8)V))
	#define GET_V_RAM_ADDR0(R)    (__u8)(R & M_RAM_ADDR0)


#define R_RAM_ADDR1 0x09 // register address, write only
	#define M_RAM_ADDR1  0x07  // mask bits 0..2
	#define SET_V_RAM_ADDR1(R,V)  (R = (__u8)((R & (__u8)(M_RAM_ADDR1 ^ 0xFF)) | (__u8)(V & 0x07)))
	#define GET_V_RAM_ADDR1(R)    (__u8)(R & M_RAM_ADDR1)

	#define M_ADDR_RES  0x40  // mask bit 6
	#define SET_V_ADDR_RES(R,V)  (R = (__u8)((R & (__u8)(M_ADDR_RES ^ 0xFF)) | (__u8)((V & 0x01) << 6)))
	#define GET_V_ADDR_RES(R)    (__u8)((R & M_ADDR_RES) >> 6)

	#define M_ADDR_INC  0x80  // mask bit 7
	#define SET_V_ADDR_INC(R,V)  (R = (__u8)((R & (__u8)(M_ADDR_INC ^ 0xFF)) | (__u8)((V & 0x01) << 7)))
	#define GET_V_ADDR_INC(R)    (__u8)((R & M_ADDR_INC) >> 7)


#define R_FIFO_REV 0x0B // register address, write only
	#define M_FIFO0_TX_REV  0x01  // mask bit 0
	#define SET_V_FIFO0_TX_REV(R,V)  (R = (__u8)((R & (__u8)(M_FIFO0_TX_REV ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_FIFO0_TX_REV(R)    (__u8)(R & M_FIFO0_TX_REV)

	#define M_FIFO0_RX_REV  0x02  // mask bit 1
	#define SET_V_FIFO0_RX_REV(R,V)  (R = (__u8)((R & (__u8)(M_FIFO0_RX_REV ^ 0xFF)) | (__u8)((V & 0x01) << 1)))
	#define GET_V_FIFO0_RX_REV(R)    (__u8)((R & M_FIFO0_RX_REV) >> 1)

	#define M_FIFO1_TX_REV  0x04  // mask bit 2
	#define SET_V_FIFO1_TX_REV(R,V)  (R = (__u8)((R & (__u8)(M_FIFO1_TX_REV ^ 0xFF)) | (__u8)((V & 0x01) << 2)))
	#define GET_V_FIFO1_TX_REV(R)    (__u8)((R & M_FIFO1_TX_REV) >> 2)

	#define M_FIFO1_RX_REV  0x08  // mask bit 3
	#define SET_V_FIFO1_RX_REV(R,V)  (R = (__u8)((R & (__u8)(M_FIFO1_RX_REV ^ 0xFF)) | (__u8)((V & 0x01) << 3)))
	#define GET_V_FIFO1_RX_REV(R)    (__u8)((R & M_FIFO1_RX_REV) >> 3)

	#define M_FIFO2_TX_REV  0x10  // mask bit 4
	#define SET_V_FIFO2_TX_REV(R,V)  (R = (__u8)((R & (__u8)(M_FIFO2_TX_REV ^ 0xFF)) | (__u8)((V & 0x01) << 4)))
	#define GET_V_FIFO2_TX_REV(R)    (__u8)((R & M_FIFO2_TX_REV) >> 4)

	#define M_FIFO2_RX_REV  0x20  // mask bit 5
	#define SET_V_FIFO2_RX_REV(R,V)  (R = (__u8)((R & (__u8)(M_FIFO2_RX_REV ^ 0xFF)) | (__u8)((V & 0x01) << 5)))
	#define GET_V_FIFO2_RX_REV(R)    (__u8)((R & M_FIFO2_RX_REV) >> 5)

	#define M_FIFO3_TX_REV  0x40  // mask bit 6
	#define SET_V_FIFO3_TX_REV(R,V)  (R = (__u8)((R & (__u8)(M_FIFO3_TX_REV ^ 0xFF)) | (__u8)((V & 0x01) << 6)))
	#define GET_V_FIFO3_TX_REV(R)    (__u8)((R & M_FIFO3_TX_REV) >> 6)

	#define M_FIFO3_RX_REV  0x80  // mask bit 7
	#define SET_V_FIFO3_RX_REV(R,V)  (R = (__u8)((R & (__u8)(M_FIFO3_RX_REV ^ 0xFF)) | (__u8)((V & 0x01) << 7)))
	#define GET_V_FIFO3_RX_REV(R)    (__u8)((R & M_FIFO3_RX_REV) >> 7)


#define A_F1 0x0C // register address, read only
	#define M_F1  0xFF  // mask bits 0..7
	#define GET_V_F1(R)    (__u8)(R & M_F1)


#define R_FIFO_THRES 0x0C // register address, write only
	#define M_THRES_TX  0x0F  // mask bits 0..3
	#define SET_V_THRES_TX(R,V)  (R = (__u8)((R & (__u8)(M_THRES_TX ^ 0xFF)) | (__u8)(V & 0x0F)))
	#define GET_V_THRES_TX(R)    (__u8)(R & M_THRES_TX)

	#define M_THRES_RX  0xF0  // mask bits 4..7
	#define SET_V_THRES_RX(R,V)  (R = (__u8)((R & (__u8)(M_THRES_RX ^ 0xFF)) | (__u8)((V & 0x0F) << 4)))
	#define GET_V_THRES_RX(R)    (__u8)((R & M_THRES_RX) >> 4)


#define A_F2 0x0D // register address, read only
	#define M_F2  0xFF  // mask bits 0..7
	#define GET_V_F2(R)    (__u8)(R & M_F2)


#define R_DF_MD 0x0D // register address, write only
	#define M_CSM  0x80  // mask bit 7
	#define SET_V_CSM(R,V)  (R = (__u8)((R & (__u8)(M_CSM ^ 0xFF)) | (__u8)((V & 0x01) << 7)))
	#define GET_V_CSM(R)    (__u8)((R & M_CSM) >> 7)


#define A_INC_RES_FIFO 0x0E // register address, write only
	#define M_INC_F  0x01  // mask bit 0
	#define SET_V_INC_F(R,V)  (R = (__u8)((R & (__u8)(M_INC_F ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_INC_F(R)    (__u8)(R & M_INC_F)

	#define M_RES_FIFO  0x02  // mask bit 1
	#define SET_V_RES_FIFO(R,V)  (R = (__u8)((R & (__u8)(M_RES_FIFO ^ 0xFF)) | (__u8)((V & 0x01) << 1)))
	#define GET_V_RES_FIFO(R)    (__u8)((R & M_RES_FIFO) >> 1)


#define R_FIFO 0x0F // register address, write only
	#define M_FIFO_DIR  0x01  // mask bit 0
	#define SET_V_FIFO_DIR(R,V)  (R = (__u8)((R & (__u8)(M_FIFO_DIR ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_FIFO_DIR(R)    (__u8)(R & M_FIFO_DIR)

	#define M_FIFO_NUM  0x06  // mask bits 1..2
	#define SET_V_FIFO_NUM(R,V)  (R = (__u8)((R & (__u8)(M_FIFO_NUM ^ 0xFF)) | (__u8)((V & 0x03) << 1)))
	#define GET_V_FIFO_NUM(R)    (__u8)((R & M_FIFO_NUM) >> 1)


#define R_FIFO_IRQ 0x10 // register address, read only
	#define M_FIFO0_TX_IRQ  0x01  // mask bit 0
	#define GET_V_FIFO0_TX_IRQ(R)    (__u8)(R & M_FIFO0_TX_IRQ)

	#define M_FIFO0_RX_IRQ  0x02  // mask bit 1
	#define GET_V_FIFO0_RX_IRQ(R)    (__u8)((R & M_FIFO0_RX_IRQ) >> 1)

	#define M_FIFO1_TX_IRQ  0x04  // mask bit 2
	#define GET_V_FIFO1_TX_IRQ(R)    (__u8)((R & M_FIFO1_TX_IRQ) >> 2)

	#define M_FIFO1_RX_IRQ  0x08  // mask bit 3
	#define GET_V_FIFO1_RX_IRQ(R)    (__u8)((R & M_FIFO1_RX_IRQ) >> 3)

	#define M_FIFO2_TX_IRQ  0x10  // mask bit 4
	#define GET_V_FIFO2_TX_IRQ(R)    (__u8)((R & M_FIFO2_TX_IRQ) >> 4)

	#define M_FIFO2_RX_IRQ  0x20  // mask bit 5
	#define GET_V_FIFO2_RX_IRQ(R)    (__u8)((R & M_FIFO2_RX_IRQ) >> 5)

	#define M_FIFO3_TX_IRQ  0x40  // mask bit 6
	#define GET_V_FIFO3_TX_IRQ(R)    (__u8)((R & M_FIFO3_TX_IRQ) >> 6)

	#define M_FIFO3_RX_IRQ  0x80  // mask bit 7
	#define GET_V_FIFO3_RX_IRQ(R)    (__u8)((R & M_FIFO3_RX_IRQ) >> 7)


#define R_MISC_IRQ 0x11 // register address, read only
	#define M_ST_IRQ  0x01  // mask bit 0
	#define GET_V_ST_IRQ(R)    (__u8)(R & M_ST_IRQ)

	#define M_TI_IRQ  0x02  // mask bit 1
	#define GET_V_TI_IRQ(R)    (__u8)((R & M_TI_IRQ) >> 1)

	#define M_PROC_IRQ  0x04  // mask bit 2
	#define GET_V_PROC_IRQ(R)    (__u8)((R & M_PROC_IRQ) >> 2)

	#define M_CI_IRQ  0x08  // mask bit 3
	#define GET_V_CI_IRQ(R)    (__u8)((R & M_CI_IRQ) >> 3)

	#define M_MON_RX_IRQ  0x10  // mask bit 4
	#define GET_V_MON_RX_IRQ(R)    (__u8)((R & M_MON_RX_IRQ) >> 4)


#define R_PCM_MD0 0x14 // register address, write only
	#define M_PCM_MD  0x01  // mask bit 0
	#define SET_V_PCM_MD(R,V)  (R = (__u8)((R & (__u8)(M_PCM_MD ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_PCM_MD(R)    (__u8)(R & M_PCM_MD)

	#define M_C4_POL  0x02  // mask bit 1
	#define SET_V_C4_POL(R,V)  (R = (__u8)((R & (__u8)(M_C4_POL ^ 0xFF)) | (__u8)((V & 0x01) << 1)))
	#define GET_V_C4_POL(R)    (__u8)((R & M_C4_POL) >> 1)

	#define M_F0_NEG  0x04  // mask bit 2
	#define SET_V_F0_NEG(R,V)  (R = (__u8)((R & (__u8)(M_F0_NEG ^ 0xFF)) | (__u8)((V & 0x01) << 2)))
	#define GET_V_F0_NEG(R)    (__u8)((R & M_F0_NEG) >> 2)

	#define M_F0_LEN  0x08  // mask bit 3
	#define SET_V_F0_LEN(R,V)  (R = (__u8)((R & (__u8)(M_F0_LEN ^ 0xFF)) | (__u8)((V & 0x01) << 3)))
	#define GET_V_F0_LEN(R)    (__u8)((R & M_F0_LEN) >> 3)

	#define M_SL_CODECA  0x30  // mask bits 4..5
	#define SET_V_SL_CODECA(R,V)  (R = (__u8)((R & (__u8)(M_SL_CODECA ^ 0xFF)) | (__u8)((V & 0x03) << 4)))
	#define GET_V_SL_CODECA(R)    (__u8)((R & M_SL_CODECA) >> 4)

	#define M_SL_CODECB  0xC0  // mask bits 6..7
	#define SET_V_SL_CODECB(R,V)  (R = (__u8)((R & (__u8)(M_SL_CODECB ^ 0xFF)) | (__u8)((V & 0x03) << 6)))
	#define GET_V_SL_CODECB(R)    (__u8)((R & M_SL_CODECB) >> 6)


#define R_PCM_MD1 0x15 // register address, write only
	#define M_AUX1_MIR  0x01  // mask bit 0
	#define SET_V_AUX1_MIR(R,V)  (R = (__u8)((R & (__u8)(M_AUX1_MIR ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_AUX1_MIR(R)    (__u8)(R & M_AUX1_MIR)

	#define M_AUX2_MIR  0x02  // mask bit 1
	#define SET_V_AUX2_MIR(R,V)  (R = (__u8)((R & (__u8)(M_AUX2_MIR ^ 0xFF)) | (__u8)((V & 0x01) << 1)))
	#define GET_V_AUX2_MIR(R)    (__u8)((R & M_AUX2_MIR) >> 1)

	#define M_PLL_ADJ  0x0C  // mask bits 2..3
	#define SET_V_PLL_ADJ(R,V)  (R = (__u8)((R & (__u8)(M_PLL_ADJ ^ 0xFF)) | (__u8)((V & 0x03) << 2)))
	#define GET_V_PLL_ADJ(R)    (__u8)((R & M_PLL_ADJ) >> 2)

	#define M_PCM_DR  0x30  // mask bits 4..5
	#define SET_V_PCM_DR(R,V)  (R = (__u8)((R & (__u8)(M_PCM_DR ^ 0xFF)) | (__u8)((V & 0x03) << 4)))
	#define GET_V_PCM_DR(R)    (__u8)((R & M_PCM_DR) >> 4)

	#define M_PCM_LOOP  0x40  // mask bit 6
	#define SET_V_PCM_LOOP(R,V)  (R = (__u8)((R & (__u8)(M_PCM_LOOP ^ 0xFF)) | (__u8)((V & 0x01) << 6)))
	#define GET_V_PCM_LOOP(R)    (__u8)((R & M_PCM_LOOP) >> 6)

	#define M_GCI_EN  0x80  // mask bit 7
	#define SET_V_GCI_EN(R,V)  (R = (__u8)((R & (__u8)(M_GCI_EN ^ 0xFF)) | (__u8)((V & 0x01) << 7)))
	#define GET_V_GCI_EN(R)    (__u8)((R & M_GCI_EN) >> 7)


#define R_PCM_MD2 0x16 // register address, write only
	#define M_OKI_CODECA  0x01  // mask bit 0
	#define SET_V_OKI_CODECA(R,V)  (R = (__u8)((R & (__u8)(M_OKI_CODECA ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_OKI_CODECA(R)    (__u8)(R & M_OKI_CODECA)

	#define M_OKI_CODECB  0x02  // mask bit 1
	#define SET_V_OKI_CODECB(R,V)  (R = (__u8)((R & (__u8)(M_OKI_CODECB ^ 0xFF)) | (__u8)((V & 0x01) << 1)))
	#define GET_V_OKI_CODECB(R)    (__u8)((R & M_OKI_CODECB) >> 1)

	#define M_SYNC_SRC  0x04  // mask bit 2
	#define SET_V_SYNC_SRC(R,V)  (R = (__u8)((R & (__u8)(M_SYNC_SRC ^ 0xFF)) | (__u8)((V & 0x01) << 2)))
	#define GET_V_SYNC_SRC(R)    (__u8)((R & M_SYNC_SRC) >> 2)

	#define M_SYNC_OUT  0x08  // mask bit 3
	#define SET_V_SYNC_OUT(R,V)  (R = (__u8)((R & (__u8)(M_SYNC_OUT ^ 0xFF)) | (__u8)((V & 0x01) << 3)))
	#define GET_V_SYNC_OUT(R)    (__u8)((R & M_SYNC_OUT) >> 3)

	#define M_SL_BL  0x30  // mask bits 4..5
	#define SET_V_SL_BL(R,V)  (R = (__u8)((R & (__u8)(M_SL_BL ^ 0xFF)) | (__u8)((V & 0x03) << 4)))
	#define GET_V_SL_BL(R)    (__u8)((R & M_SL_BL) >> 4)

	#define M_PLL_ICR  0x40  // mask bit 6
	#define SET_V_PLL_ICR(R,V)  (R = (__u8)((R & (__u8)(M_PLL_ICR ^ 0xFF)) | (__u8)((V & 0x01) << 6)))
	#define GET_V_PLL_ICR(R)    (__u8)((R & M_PLL_ICR) >> 6)

	#define M_PLL_MAN  0x80  // mask bit 7
	#define SET_V_PLL_MAN(R,V)  (R = (__u8)((R & (__u8)(M_PLL_MAN ^ 0xFF)) | (__u8)((V & 0x01) << 7)))
	#define GET_V_PLL_MAN(R)    (__u8)((R & M_PLL_MAN) >> 7)


#define R_CHIP_ID 0x16 // register address, read only
	#define M_CHIP_ID  0xF0  // mask bits 4..7
	#define GET_V_CHIP_ID(R)    (__u8)((R & M_CHIP_ID) >> 4)


#define R_F0_CNTL 0x18 // register address, read only
	#define M_F0_CNTL  0xFF  // mask bits 0..7
	#define GET_V_F0_CNTL(R)    (__u8)(R & M_F0_CNTL)


#define R_F0_CNTH 0x19 // register address, read only
	#define M_F0_CNTH  0xFF  // mask bits 0..7
	#define GET_V_F0_CNTH(R)    (__u8)(R & M_F0_CNTH)


#define A_USAGE 0x1A // register address, read only
	#define M_USAGE  0xFF  // mask bits 0..7
	#define GET_V_USAGE(R)    (__u8)(R & M_USAGE)


#define R_FIFO_IRQMSK 0x1A // register address, write only
	#define M_FIFO0_TX_IRQMSK  0x01  // mask bit 0
	#define SET_V_FIFO0_TX_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_FIFO0_TX_IRQMSK ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_FIFO0_TX_IRQMSK(R)    (__u8)(R & M_FIFO0_TX_IRQMSK)

	#define M_FIFO0_RX_IRQMSK  0x02  // mask bit 1
	#define SET_V_FIFO0_RX_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_FIFO0_RX_IRQMSK ^ 0xFF)) | (__u8)((V & 0x01) << 1)))
	#define GET_V_FIFO0_RX_IRQMSK(R)    (__u8)((R & M_FIFO0_RX_IRQMSK) >> 1)

	#define M_FIFO1_TX_IRQMSK  0x04  // mask bit 2
	#define SET_V_FIFO1_TX_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_FIFO1_TX_IRQMSK ^ 0xFF)) | (__u8)((V & 0x01) << 2)))
	#define GET_V_FIFO1_TX_IRQMSK(R)    (__u8)((R & M_FIFO1_TX_IRQMSK) >> 2)

	#define M_FIFO1_RX_IRQMSK  0x08  // mask bit 3
	#define SET_V_FIFO1_RX_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_FIFO1_RX_IRQMSK ^ 0xFF)) | (__u8)((V & 0x01) << 3)))
	#define GET_V_FIFO1_RX_IRQMSK(R)    (__u8)((R & M_FIFO1_RX_IRQMSK) >> 3)

	#define M_FIFO2_TX_IRQMSK  0x10  // mask bit 4
	#define SET_V_FIFO2_TX_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_FIFO2_TX_IRQMSK ^ 0xFF)) | (__u8)((V & 0x01) << 4)))
	#define GET_V_FIFO2_TX_IRQMSK(R)    (__u8)((R & M_FIFO2_TX_IRQMSK) >> 4)

	#define M_FIFO2_RX_IRQMSK  0x20  // mask bit 5
	#define SET_V_FIFO2_RX_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_FIFO2_RX_IRQMSK ^ 0xFF)) | (__u8)((V & 0x01) << 5)))
	#define GET_V_FIFO2_RX_IRQMSK(R)    (__u8)((R & M_FIFO2_RX_IRQMSK) >> 5)

	#define M_FIFO3_TX_IRQMSK  0x40  // mask bit 6
	#define SET_V_FIFO3_TX_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_FIFO3_TX_IRQMSK ^ 0xFF)) | (__u8)((V & 0x01) << 6)))
	#define GET_V_FIFO3_TX_IRQMSK(R)    (__u8)((R & M_FIFO3_TX_IRQMSK) >> 6)

	#define M_FIFO3_RX_IRQMSK  0x80  // mask bit 7
	#define SET_V_FIFO3_RX_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_FIFO3_RX_IRQMSK ^ 0xFF)) | (__u8)((V & 0x01) << 7)))
	#define GET_V_FIFO3_RX_IRQMSK(R)    (__u8)((R & M_FIFO3_RX_IRQMSK) >> 7)


#define R_MISC_IRQMSK 0x1B // register address, write only
	#define M_ST_IRQMSK  0x01  // mask bit 0
	#define SET_V_ST_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_ST_IRQMSK ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_ST_IRQMSK(R)    (__u8)(R & M_ST_IRQMSK)

	#define M_TI_IRQMSK  0x02  // mask bit 1
	#define SET_V_TI_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_TI_IRQMSK ^ 0xFF)) | (__u8)((V & 0x01) << 1)))
	#define GET_V_TI_IRQMSK(R)    (__u8)((R & M_TI_IRQMSK) >> 1)

	#define M_PROC_IRQMSK  0x04  // mask bit 2
	#define SET_V_PROC_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_PROC_IRQMSK ^ 0xFF)) | (__u8)((V & 0x01) << 2)))
	#define GET_V_PROC_IRQMSK(R)    (__u8)((R & M_PROC_IRQMSK) >> 2)

	#define M_CI_IRQMSK  0x08  // mask bit 3
	#define SET_V_CI_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_CI_IRQMSK ^ 0xFF)) | (__u8)((V & 0x01) << 3)))
	#define GET_V_CI_IRQMSK(R)    (__u8)((R & M_CI_IRQMSK) >> 3)

	#define M_MON_IRQMSK  0x10  // mask bit 4
	#define SET_V_MON_IRQMSK(R,V)  (R = (__u8)((R & (__u8)(M_MON_IRQMSK ^ 0xFF)) | (__u8)((V & 0x01) << 4)))
	#define GET_V_MON_IRQMSK(R)    (__u8)((R & M_MON_IRQMSK) >> 4)

	#define M_IRQ_REV  0x40  // mask bit 6
	#define SET_V_IRQ_REV(R,V)  (R = (__u8)((R & (__u8)(M_IRQ_REV ^ 0xFF)) | (__u8)((V & 0x01) << 6)))
	#define GET_V_IRQ_REV(R)    (__u8)((R & M_IRQ_REV) >> 6)

	#define M_IRQ_EN  0x80  // mask bit 7
	#define SET_V_IRQ_EN(R,V)  (R = (__u8)((R & (__u8)(M_IRQ_EN ^ 0xFF)) | (__u8)((V & 0x01) << 7)))
	#define GET_V_IRQ_EN(R)    (__u8)((R & M_IRQ_EN) >> 7)


#define R_FILL 0x1B // register address, read only
	#define M_FIFO0_TX_FILL  0x01  // mask bit 0
	#define GET_V_FIFO0_TX_FILL(R)    (__u8)(R & M_FIFO0_TX_FILL)

	#define M_FIFO0_RX_FILL  0x02  // mask bit 1
	#define GET_V_FIFO0_RX_FILL(R)    (__u8)((R & M_FIFO0_RX_FILL) >> 1)

	#define M_FIFO1_TX_FILL  0x04  // mask bit 2
	#define GET_V_FIFO1_TX_FILL(R)    (__u8)((R & M_FIFO1_TX_FILL) >> 2)

	#define M_FIFO1_RX_FILL  0x08  // mask bit 3
	#define GET_V_FIFO1_RX_FILL(R)    (__u8)((R & M_FIFO1_RX_FILL) >> 3)

	#define M_FIFO2_TX_FILL  0x10  // mask bit 4
	#define GET_V_FIFO2_TX_FILL(R)    (__u8)((R & M_FIFO2_TX_FILL) >> 4)

	#define M_FIFO2_RX_FILL  0x20  // mask bit 5
	#define GET_V_FIFO2_RX_FILL(R)    (__u8)((R & M_FIFO2_RX_FILL) >> 5)

	#define M_FIFO3_TX_FILL  0x40  // mask bit 6
	#define GET_V_FIFO3_TX_FILL(R)    (__u8)((R & M_FIFO3_TX_FILL) >> 6)

	#define M_FIFO3_RX_FILL  0x80  // mask bit 7
	#define GET_V_FIFO3_RX_FILL(R)    (__u8)((R & M_FIFO3_RX_FILL) >> 7)


#define R_TI 0x1C // register address, write only
	#define M_EV_TS  0x0F  // mask bits 0..3
	#define SET_V_EV_TS(R,V)  (R = (__u8)((R & (__u8)(M_EV_TS ^ 0xFF)) | (__u8)(V & 0x0F)))
	#define GET_V_EV_TS(R)    (__u8)(R & M_EV_TS)


#define R_STATUS 0x1C // register address, read only
	#define M_BUSY  0x01  // mask bit 0
	#define GET_V_BUSY(R)    (__u8)(R & M_BUSY)

	#define M_PROC  0x02  // mask bit 1
	#define GET_V_PROC(R)    (__u8)((R & M_PROC) >> 1)

	#define M_AWAKE_IN  0x08  // mask bit 3
	#define GET_V_AWAKE_IN(R)    (__u8)((R & M_AWAKE_IN) >> 3)

	#define M_SYNC_IN  0x10  // mask bit 4
	#define GET_V_SYNC_IN(R)    (__u8)((R & M_SYNC_IN) >> 4)

	#define M_MISC_IRQSTA  0x40  // mask bit 6
	#define GET_V_MISC_IRQSTA(R)    (__u8)((R & M_MISC_IRQSTA) >> 6)

	#define M_FIFO_IRQSTA  0x80  // mask bit 7
	#define GET_V_FIFO_IRQSTA(R)    (__u8)((R & M_FIFO_IRQSTA) >> 7)


#define R_B1_TX_SL 0x20 // register address, write only
	#define M_B1_TX_SL  0x1F  // mask bits 0..4
	#define SET_V_B1_TX_SL(R,V)  (R = (__u8)((R & (__u8)(M_B1_TX_SL ^ 0xFF)) | (__u8)(V & 0x1F)))
	#define GET_V_B1_TX_SL(R)    (__u8)(R & M_B1_TX_SL)

	#define M_B1_TX_ROUT  0xC0  // mask bits 6..7
	#define SET_V_B1_TX_ROUT(R,V)  (R = (__u8)((R & (__u8)(M_B1_TX_ROUT ^ 0xFF)) | (__u8)((V & 0x03) << 6)))
	#define GET_V_B1_TX_ROUT(R)    (__u8)((R & M_B1_TX_ROUT) >> 6)


#define R_B2_TX_SL 0x21 // register address, write only
	#define M_B2_TX_SL  0x1F  // mask bits 0..4
	#define SET_V_B2_TX_SL(R,V)  (R = (__u8)((R & (__u8)(M_B2_TX_SL ^ 0xFF)) | (__u8)(V & 0x1F)))
	#define GET_V_B2_TX_SL(R)    (__u8)(R & M_B2_TX_SL)

	#define M_B2_TX_ROUT  0xC0  // mask bits 6..7
	#define SET_V_B2_TX_ROUT(R,V)  (R = (__u8)((R & (__u8)(M_B2_TX_ROUT ^ 0xFF)) | (__u8)((V & 0x03) << 6)))
	#define GET_V_B2_TX_ROUT(R)    (__u8)((R & M_B2_TX_ROUT) >> 6)


#define R_AUX1_TX_SL 0x22 // register address, write only
	#define M_AUX1_TX_SL  0x1F  // mask bits 0..4
	#define SET_V_AUX1_TX_SL(R,V)  (R = (__u8)((R & (__u8)(M_AUX1_TX_SL ^ 0xFF)) | (__u8)(V & 0x1F)))
	#define GET_V_AUX1_TX_SL(R)    (__u8)(R & M_AUX1_TX_SL)

	#define M_AUX1_TX_ROUT  0xC0  // mask bits 6..7
	#define SET_V_AUX1_TX_ROUT(R,V)  (R = (__u8)((R & (__u8)(M_AUX1_TX_ROUT ^ 0xFF)) | (__u8)((V & 0x03) << 6)))
	#define GET_V_AUX1_TX_ROUT(R)    (__u8)((R & M_AUX1_TX_ROUT) >> 6)


#define R_AUX2_TX_SL 0x23 // register address, write only
	#define M_AUX2_TX_SL  0x1F  // mask bits 0..4
	#define SET_V_AUX2_TX_SL(R,V)  (R = (__u8)((R & (__u8)(M_AUX2_TX_SL ^ 0xFF)) | (__u8)(V & 0x1F)))
	#define GET_V_AUX2_TX_SL(R)    (__u8)(R & M_AUX2_TX_SL)

	#define M_AUX2_TX_ROUT  0xC0  // mask bits 6..7
	#define SET_V_AUX2_TX_ROUT(R,V)  (R = (__u8)((R & (__u8)(M_AUX2_TX_ROUT ^ 0xFF)) | (__u8)((V & 0x03) << 6)))
	#define GET_V_AUX2_TX_ROUT(R)    (__u8)((R & M_AUX2_TX_ROUT) >> 6)


#define R_B1_RX_SL 0x24 // register address, write only
	#define M_B1_RX_SL  0x1F  // mask bits 0..4
	#define SET_V_B1_RX_SL(R,V)  (R = (__u8)((R & (__u8)(M_B1_RX_SL ^ 0xFF)) | (__u8)(V & 0x1F)))
	#define GET_V_B1_RX_SL(R)    (__u8)(R & M_B1_RX_SL)

	#define M_B1_RX_ROUT  0xC0  // mask bits 6..7
	#define SET_V_B1_RX_ROUT(R,V)  (R = (__u8)((R & (__u8)(M_B1_RX_ROUT ^ 0xFF)) | (__u8)((V & 0x03) << 6)))
	#define GET_V_B1_RX_ROUT(R)    (__u8)((R & M_B1_RX_ROUT) >> 6)


#define R_B2_RX_SL 0x25 // register address, write only
	#define M_B2_RX_SL  0x1F  // mask bits 0..4
	#define SET_V_B2_RX_SL(R,V)  (R = (__u8)((R & (__u8)(M_B2_RX_SL ^ 0xFF)) | (__u8)(V & 0x1F)))
	#define GET_V_B2_RX_SL(R)    (__u8)(R & M_B2_RX_SL)

	#define M_B2_RX_ROUT  0xC0  // mask bits 6..7
	#define SET_V_B2_RX_ROUT(R,V)  (R = (__u8)((R & (__u8)(M_B2_RX_ROUT ^ 0xFF)) | (__u8)((V & 0x03) << 6)))
	#define GET_V_B2_RX_ROUT(R)    (__u8)((R & M_B2_RX_ROUT) >> 6)


#define R_AUX1_RX_SL 0x26 // register address, write only
	#define M_AUX1_RX_SL  0x1F  // mask bits 0..4
	#define SET_V_AUX1_RX_SL(R,V)  (R = (__u8)((R & (__u8)(M_AUX1_RX_SL ^ 0xFF)) | (__u8)(V & 0x1F)))
	#define GET_V_AUX1_RX_SL(R)    (__u8)(R & M_AUX1_RX_SL)

	#define M_AUX1_RX_ROUT  0xC0  // mask bits 6..7
	#define SET_V_AUX1_RX_ROUT(R,V)  (R = (__u8)((R & (__u8)(M_AUX1_RX_ROUT ^ 0xFF)) | (__u8)((V & 0x03) << 6)))
	#define GET_V_AUX1_RX_ROUT(R)    (__u8)((R & M_AUX1_RX_ROUT) >> 6)


#define R_AUX2_RX_SL 0x27 // register address, write only
	#define M_AUX2_RX_SL  0x1F  // mask bits 0..4
	#define SET_V_AUX2_RX_SL(R,V)  (R = (__u8)((R & (__u8)(M_AUX2_RX_SL ^ 0xFF)) | (__u8)(V & 0x1F)))
	#define GET_V_AUX2_RX_SL(R)    (__u8)(R & M_AUX2_RX_SL)

	#define M_AUX2_RX_ROUT  0xC0  // mask bits 6..7
	#define SET_V_AUX2_RX_ROUT(R,V)  (R = (__u8)((R & (__u8)(M_AUX2_RX_ROUT ^ 0xFF)) | (__u8)((V & 0x03) << 6)))
	#define GET_V_AUX2_RX_ROUT(R)    (__u8)((R & M_AUX2_RX_ROUT) >> 6)


#define R_CI_TX 0x28 // register address, write only
	#define M_GCI_C  0x0F  // mask bits 0..3
	#define SET_V_GCI_C(R,V)  (R = (__u8)((R & (__u8)(M_GCI_C ^ 0xFF)) | (__u8)(V & 0x0F)))
	#define GET_V_GCI_C(R)    (__u8)(R & M_GCI_C)


#define R_CI_RX 0x28 // register address, read only
	#define M_GCI_I  0x0F  // mask bits 0..3
	#define GET_V_GCI_I(R)    (__u8)(R & M_GCI_I)


#define R_PCM_GCI_STA 0x29 // register address, read only
	#define M_MON_RXR  0x01  // mask bit 0
	#define GET_V_MON_RXR(R)    (__u8)(R & M_MON_RXR)

	#define M_MON_TXR  0x02  // mask bit 1
	#define GET_V_MON_TXR(R)    (__u8)((R & M_MON_TXR) >> 1)

	#define M_STIO2_IN  0x40  // mask bit 6
	#define GET_V_STIO2_IN(R)    (__u8)((R & M_STIO2_IN) >> 6)

	#define M_STIO1_IN  0x80  // mask bit 7
	#define GET_V_STIO1_IN(R)    (__u8)((R & M_STIO1_IN) >> 7)


#define R_MON1_TX 0x2A // register address, write only
	#define M_MON1_TX  0xFF  // mask bits 0..7
	#define SET_V_MON1_TX(R,V)  (R = (__u8)((R & (__u8)(M_MON1_TX ^ 0xFF)) | (__u8)V))
	#define GET_V_MON1_TX(R)    (__u8)(R & M_MON1_TX)


#define R_MON1_RX 0x2A // register address, read only
	#define M_MON1_RX  0xFF  // mask bits 0..7
	#define GET_V_MON1_RX(R)    (__u8)(R & M_MON1_RX)


#define R_MON2_TX 0x2B // register address, write only
	#define M_MON2_TX  0xFF  // mask bits 0..7
	#define SET_V_MON2_TX(R,V)  (R = (__u8)((R & (__u8)(M_MON2_TX ^ 0xFF)) | (__u8)V))
	#define GET_V_MON2_TX(R)    (__u8)(R & M_MON2_TX)


#define R_MON2_RX 0x2B // register address, read only
	#define M_MON2_RX  0xFF  // mask bits 0..7
	#define GET_V_MON2_RX(R)    (__u8)(R & M_MON2_RX)


#define R_B1_TX 0x2C // register address, write only
	#define M_B1_TX  0xFF  // mask bits 0..7
	#define SET_V_B1_TX(R,V)  (R = (__u8)((R & (__u8)(M_B1_TX ^ 0xFF)) | (__u8)V))
	#define GET_V_B1_TX(R)    (__u8)(R & M_B1_TX)


#define R_B1_RX 0x2C // register address, read only
	#define M_B1_RX  0xFF  // mask bits 0..7
	#define GET_V_B1_RX(R)    (__u8)(R & M_B1_RX)


#define R_B2_TX 0x2D // register address, write only
	#define M_B2_TX  0xFF  // mask bits 0..7
	#define SET_V_B2_TX(R,V)  (R = (__u8)((R & (__u8)(M_B2_TX ^ 0xFF)) | (__u8)V))
	#define GET_V_B2_TX(R)    (__u8)(R & M_B2_TX)


#define R_B2_RX 0x2D // register address, read only
	#define M_B2_RX  0xFF  // mask bits 0..7
	#define GET_V_B2_RX(R)    (__u8)(R & M_B2_RX)


#define R_AUX1_TX 0x2E // register address, write only
	#define M_AUX1_TX  0xFF  // mask bits 0..7
	#define SET_V_AUX1_TX(R,V)  (R = (__u8)((R & (__u8)(M_AUX1_TX ^ 0xFF)) | (__u8)V))
	#define GET_V_AUX1_TX(R)    (__u8)(R & M_AUX1_TX)


#define R_AUX1_RX 0x2E // register address, read only
	#define M_AUX1_RX  0xFF  // mask bits 0..7
	#define GET_V_AUX1_RX(R)    (__u8)(R & M_AUX1_RX)


#define R_AUX2_TX 0x2F // register address, write only
	#define M_AUX2_TX  0xFF  // mask bits 0..7
	#define SET_V_AUX2_TX(R,V)  (R = (__u8)((R & (__u8)(M_AUX2_TX ^ 0xFF)) | (__u8)V))
	#define GET_V_AUX2_TX(R)    (__u8)(R & M_AUX2_TX)


#define R_AUX2_RX 0x2F // register address, read only
	#define M_AUX2_RX  0xFF  // mask bits 0..7
	#define GET_V_AUX2_RX(R)    (__u8)(R & M_AUX2_RX)


#define R_ST_WR_STA 0x30 // register address, write only
	#define M_ST_SET_STA  0x0F  // mask bits 0..3
	#define SET_V_ST_SET_STA(R,V)  (R = (__u8)((R & (__u8)(M_ST_SET_STA ^ 0xFF)) | (__u8)(V & 0x0F)))
	#define GET_V_ST_SET_STA(R)    (__u8)(R & M_ST_SET_STA)

	#define M_ST_LD_STA  0x10  // mask bit 4
	#define SET_V_ST_LD_STA(R,V)  (R = (__u8)((R & (__u8)(M_ST_LD_STA ^ 0xFF)) | (__u8)((V & 0x01) << 4)))
	#define GET_V_ST_LD_STA(R)    (__u8)((R & M_ST_LD_STA) >> 4)

	#define M_ST_ACT  0x60  // mask bits 5..6
	#define SET_V_ST_ACT(R,V)  (R = (__u8)((R & (__u8)(M_ST_ACT ^ 0xFF)) | (__u8)((V & 0x03) << 5)))
	#define GET_V_ST_ACT(R)    (__u8)((R & M_ST_ACT) >> 5)

	#define M_SET_G2_G3  0x80  // mask bit 7
	#define SET_V_SET_G2_G3(R,V)  (R = (__u8)((R & (__u8)(M_SET_G2_G3 ^ 0xFF)) | (__u8)((V & 0x01) << 7)))
	#define GET_V_SET_G2_G3(R)    (__u8)((R & M_SET_G2_G3) >> 7)


#define R_ST_RD_STA 0x30 // register address, read only
	#define M_ST_STA  0x0F  // mask bits 0..3
	#define GET_V_ST_STA(R)    (__u8)(R & M_ST_STA)

	#define M_FR_SYNC  0x10  // mask bit 4
	#define GET_V_FR_SYNC(R)    (__u8)((R & M_FR_SYNC) >> 4)

	#define M_T2_EXP  0x20  // mask bit 5
	#define GET_V_T2_EXP(R)    (__u8)((R & M_T2_EXP) >> 5)

	#define M_INFO0  0x40  // mask bit 6
	#define GET_V_INFO0(R)    (__u8)((R & M_INFO0) >> 6)

	#define M_G2_G3  0x80  // mask bit 7
	#define GET_V_G2_G3(R)    (__u8)((R & M_G2_G3) >> 7)


#define R_ST_CTRL0 0x31 // register address, write only
	#define M_B1_EN  0x01  // mask bit 0
	#define SET_V_B1_EN(R,V)  (R = (__u8)((R & (__u8)(M_B1_EN ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_B1_EN(R)    (__u8)(R & M_B1_EN)

	#define M_B2_EN  0x02  // mask bit 1
	#define SET_V_B2_EN(R,V)  (R = (__u8)((R & (__u8)(M_B2_EN ^ 0xFF)) | (__u8)((V & 0x01) << 1)))
	#define GET_V_B2_EN(R)    (__u8)((R & M_B2_EN) >> 1)

	#define M_ST_MD  0x04  // mask bit 2
	#define SET_V_ST_MD(R,V)  (R = (__u8)((R & (__u8)(M_ST_MD ^ 0xFF)) | (__u8)((V & 0x01) << 2)))
	#define GET_V_ST_MD(R)    (__u8)((R & M_ST_MD) >> 2)

	#define M_D_PRIO  0x08  // mask bit 3
	#define SET_V_D_PRIO(R,V)  (R = (__u8)((R & (__u8)(M_D_PRIO ^ 0xFF)) | (__u8)((V & 0x01) << 3)))
	#define GET_V_D_PRIO(R)    (__u8)((R & M_D_PRIO) >> 3)

	#define M_SQ_EN  0x10  // mask bit 4
	#define SET_V_SQ_EN(R,V)  (R = (__u8)((R & (__u8)(M_SQ_EN ^ 0xFF)) | (__u8)((V & 0x01) << 4)))
	#define GET_V_SQ_EN(R)    (__u8)((R & M_SQ_EN) >> 4)

	#define M_96KHZ  0x20  // mask bit 5
	#define SET_V_96KHZ(R,V)  (R = (__u8)((R & (__u8)(M_96KHZ ^ 0xFF)) | (__u8)((V & 0x01) << 5)))
	#define GET_V_96KHZ(R)    (__u8)((R & M_96KHZ) >> 5)

	#define M_TX_LI  0x40  // mask bit 6
	#define SET_V_TX_LI(R,V)  (R = (__u8)((R & (__u8)(M_TX_LI ^ 0xFF)) | (__u8)((V & 0x01) << 6)))
	#define GET_V_TX_LI(R)    (__u8)((R & M_TX_LI) >> 6)

	#define M_ST_STOP  0x80  // mask bit 7
	#define SET_V_ST_STOP(R,V)  (R = (__u8)((R & (__u8)(M_ST_STOP ^ 0xFF)) | (__u8)((V & 0x01) << 7)))
	#define GET_V_ST_STOP(R)    (__u8)((R & M_ST_STOP) >> 7)


#define R_ST_CTRL1 0x32 // register address, write only
	#define M_G2_G3_EN  0x01  // mask bit 0
	#define SET_V_G2_G3_EN(R,V)  (R = (__u8)((R & (__u8)(M_G2_G3_EN ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_G2_G3_EN(R)    (__u8)(R & M_G2_G3_EN)

	#define M_D_RES  0x04  // mask bit 2
	#define SET_V_D_RES(R,V)  (R = (__u8)((R & (__u8)(M_D_RES ^ 0xFF)) | (__u8)((V & 0x01) << 2)))
	#define GET_V_D_RES(R)    (__u8)((R & M_D_RES) >> 2)

	#define M_E_IGNO  0x08  // mask bit 3
	#define SET_V_E_IGNO(R,V)  (R = (__u8)((R & (__u8)(M_E_IGNO ^ 0xFF)) | (__u8)((V & 0x01) << 3)))
	#define GET_V_E_IGNO(R)    (__u8)((R & M_E_IGNO) >> 3)

	#define M_E_LO  0x10  // mask bit 4
	#define SET_V_E_LO(R,V)  (R = (__u8)((R & (__u8)(M_E_LO ^ 0xFF)) | (__u8)((V & 0x01) << 4)))
	#define GET_V_E_LO(R)    (__u8)((R & M_E_LO) >> 4)

	#define M_B12_SWAP  0x80  // mask bit 7
	#define SET_V_B12_SWAP(R,V)  (R = (__u8)((R & (__u8)(M_B12_SWAP ^ 0xFF)) | (__u8)((V & 0x01) << 7)))
	#define GET_V_B12_SWAP(R)    (__u8)((R & M_B12_SWAP) >> 7)


#define R_ST_CTRL2 0x33 // register address, write only
	#define M_B1_RX_EN  0x01  // mask bit 0
	#define SET_V_B1_RX_EN(R,V)  (R = (__u8)((R & (__u8)(M_B1_RX_EN ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_B1_RX_EN(R)    (__u8)(R & M_B1_RX_EN)

	#define M_B2_RX_EN  0x02  // mask bit 1
	#define SET_V_B2_RX_EN(R,V)  (R = (__u8)((R & (__u8)(M_B2_RX_EN ^ 0xFF)) | (__u8)((V & 0x01) << 1)))
	#define GET_V_B2_RX_EN(R)    (__u8)((R & M_B2_RX_EN) >> 1)

	#define M_FO_INFO0  0x40  // mask bit 6
	#define SET_V_FO_INFO0(R,V)  (R = (__u8)((R & (__u8)(M_FO_INFO0 ^ 0xFF)) | (__u8)((V & 0x01) << 6)))
	#define GET_V_FO_INFO0(R)    (__u8)((R & M_FO_INFO0) >> 6)


#define R_ST_SQ_WR 0x34 // register address, write only
	#define M_ST_SQ_WR  0x0F  // mask bits 0..3
	#define SET_V_ST_SQ_WR(R,V)  (R = (__u8)((R & (__u8)(M_ST_SQ_WR ^ 0xFF)) | (__u8)(V & 0x0F)))
	#define GET_V_ST_SQ_WR(R)    (__u8)(R & M_ST_SQ_WR)


#define R_ST_SQ_RD 0x34 // register address, read only
	#define M_ST_SQ_RD  0x0F  // mask bits 0..3
	#define GET_V_ST_SQ_RD(R)    (__u8)(R & M_ST_SQ_RD)

	#define M_MF_RX_RDY  0x10  // mask bit 4
	#define GET_V_MF_RX_RDY(R)    (__u8)((R & M_MF_RX_RDY) >> 4)

	#define M_MF_TX_RDY  0x80  // mask bit 7
	#define GET_V_MF_TX_RDY(R)    (__u8)((R & M_MF_TX_RDY) >> 7)


#define R_ST_CLK_DLY 0x37 // register address, write only
	#define M_ST_CLK_DLY  0x0F  // mask bits 0..3
	#define SET_V_ST_CLK_DLY(R,V)  (R = (__u8)((R & (__u8)(M_ST_CLK_DLY ^ 0xFF)) | (__u8)(V & 0x0F)))
	#define GET_V_ST_CLK_DLY(R)    (__u8)(R & M_ST_CLK_DLY)

	#define M_ST_SMPL  0x70  // mask bits 4..6
	#define SET_V_ST_SMPL(R,V)  (R = (__u8)((R & (__u8)(M_ST_SMPL ^ 0xFF)) | (__u8)((V & 0x07) << 4)))
	#define GET_V_ST_SMPL(R)    (__u8)((R & M_ST_SMPL) >> 4)


#define R_ST_B1_TX 0x3C // register address, write only
	#define M_ST_B1_TX  0xFF  // mask bits 0..7
	#define SET_V_ST_B1_TX(R,V)  (R = (__u8)((R & (__u8)(M_ST_B1_TX ^ 0xFF)) | (__u8)V))
	#define GET_V_ST_B1_TX(R)    (__u8)(R & M_ST_B1_TX)


#define R_ST_B1_RX 0x3C // register address, read only
	#define M_ST_B1_RX  0xFF  // mask bits 0..7
	#define GET_V_ST_B1_RX(R)    (__u8)(R & M_ST_B1_RX)


#define R_ST_B2_TX 0x3D // register address, write only
	#define M_ST_B2_TX  0xFF  // mask bits 0..7
	#define SET_V_ST_B2_TX(R,V)  (R = (__u8)((R & (__u8)(M_ST_B2_TX ^ 0xFF)) | (__u8)V))
	#define GET_V_ST_B2_TX(R)    (__u8)(R & M_ST_B2_TX)


#define R_ST_B2_RX 0x3D // register address, read only
	#define M_ST_B2_RX  0xFF  // mask bits 0..7
	#define GET_V_ST_B2_RX(R)    (__u8)(R & M_ST_B2_RX)


#define R_ST_D_TX 0x3E // register address, write only
	#define M_ST_D_TX  0xC0  // mask bits 6..7
	#define SET_V_ST_D_TX(R,V)  (R = (__u8)((R & (__u8)(M_ST_D_TX ^ 0xFF)) | (__u8)((V & 0x03) << 6)))
	#define GET_V_ST_D_TX(R)    (__u8)((R & M_ST_D_TX) >> 6)


#define R_ST_D_RX 0x3E // register address, read only
	#define M_ST_D_RX  0xC0  // mask bits 6..7
	#define GET_V_ST_D_RX(R)    (__u8)((R & M_ST_D_RX) >> 6)


#define R_ST_E_RX 0x3F // register address, read only
	#define M_ST_E_RX  0xC0  // mask bits 6..7
	#define GET_V_ST_E_RX(R)    (__u8)((R & M_ST_E_RX) >> 6)


#define A_FIFO_DATA 0x80 // register address, read/write
	#define M_FIFO_DATA  0xFF  // mask bits 0..7
	#define SET_V_FIFO_DATA(R,V)  (R = (__u8)((R & (__u8)(M_FIFO_DATA ^ 0xFF)) | (__u8)V))
	#define GET_V_FIFO_DATA(R)    (__u8)(R & M_FIFO_DATA)


#define A_FIFO_DATA_NOINC 0x84 // register address, write only
	#define M_FIFO_DATA_NOINC  0xFF  // mask bits 0..7
	#define SET_V_FIFO_DATA_NOINC(R,V)  (R = (__u8)((R & (__u8)(M_FIFO_DATA_NOINC ^ 0xFF)) | (__u8)V))
	#define GET_V_FIFO_DATA_NOINC(R)    (__u8)(R & M_FIFO_DATA_NOINC)


#define R_RAM_DATA 0xC0 // register address, read/write
	#define M_RAM_DATA  0xFF  // mask bits 0..7
	#define SET_V_RAM_DATA(R,V)  (R = (__u8)((R & (__u8)(M_RAM_DATA ^ 0xFF)) | (__u8)V))
	#define GET_V_RAM_DATA(R)    (__u8)(R & M_RAM_DATA)


#define A_CH_MSK 0xF4 // register address, write only
	#define M_CH_MSK  0xFF  // mask bits 0..7
	#define SET_V_CH_MSK(R,V)  (R = (__u8)((R & (__u8)(M_CH_MSK ^ 0xFF)) | (__u8)V))
	#define GET_V_CH_MSK(R)    (__u8)(R & M_CH_MSK)


#define A_CON_HDLC 0xFA // register address, write only
	#define M_IFF  0x01  // mask bit 0
	#define SET_V_IFF(R,V)  (R = (__u8)((R & (__u8)(M_IFF ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_IFF(R)    (__u8)(R & M_IFF)

	#define M_HDLC_TRP  0x02  // mask bit 1
	#define SET_V_HDLC_TRP(R,V)  (R = (__u8)((R & (__u8)(M_HDLC_TRP ^ 0xFF)) | (__u8)((V & 0x01) << 1)))
	#define GET_V_HDLC_TRP(R)    (__u8)((R & M_HDLC_TRP) >> 1)

	#define M_TRP_IRQ  0x0C  // mask bits 2..3
	#define SET_V_TRP_IRQ(R,V)  (R = (__u8)((R & (__u8)(M_TRP_IRQ ^ 0xFF)) | (__u8)((V & 0x03) << 2)))
	#define GET_V_TRP_IRQ(R)    (__u8)((R & M_TRP_IRQ) >> 2)

	#define M_DATA_FLOW  0xE0  // mask bits 5..7
	#define SET_V_DATA_FLOW(R,V)  (R = (__u8)((R & (__u8)(M_DATA_FLOW ^ 0xFF)) | (__u8)((V & 0x07) << 5)))
	#define GET_V_DATA_FLOW(R)    (__u8)((R & M_DATA_FLOW) >> 5)


#define A_HDLC_PAR 0xFB // register address, write only
	#define M_BIT_CNT  0x07  // mask bits 0..2
	#define SET_V_BIT_CNT(R,V)  (R = (__u8)((R & (__u8)(M_BIT_CNT ^ 0xFF)) | (__u8)(V & 0x07)))
	#define GET_V_BIT_CNT(R)    (__u8)(R & M_BIT_CNT)

	#define M_START_BIT  0x38  // mask bits 3..5
	#define SET_V_START_BIT(R,V)  (R = (__u8)((R & (__u8)(M_START_BIT ^ 0xFF)) | (__u8)((V & 0x07) << 3)))
	#define GET_V_START_BIT(R)    (__u8)((R & M_START_BIT) >> 3)

	#define M_LOOP_FIFO  0x40  // mask bit 6
	#define SET_V_LOOP_FIFO(R,V)  (R = (__u8)((R & (__u8)(M_LOOP_FIFO ^ 0xFF)) | (__u8)((V & 0x01) << 6)))
	#define GET_V_LOOP_FIFO(R)    (__u8)((R & M_LOOP_FIFO) >> 6)

	#define M_INV_DATA  0x80  // mask bit 7
	#define SET_V_INV_DATA(R,V)  (R = (__u8)((R & (__u8)(M_INV_DATA ^ 0xFF)) | (__u8)((V & 0x01) << 7)))
	#define GET_V_INV_DATA(R)    (__u8)((R & M_INV_DATA) >> 7)


#define A_CHANNEL 0xFC // register address, write only
	#define M_CH_DIR  0x01  // mask bit 0
	#define SET_V_CH_DIR(R,V)  (R = (__u8)((R & (__u8)(M_CH_DIR ^ 0xFF)) | (__u8)(V & 0x01)))
	#define GET_V_CH_DIR(R)    (__u8)(R & M_CH_DIR)

	#define M_CH_NUM  0x06  // mask bits 1..2
	#define SET_V_CH_NUM(R,V)  (R = (__u8)((R & (__u8)(M_CH_NUM ^ 0xFF)) | (__u8)((V & 0x03) << 1)))
	#define GET_V_CH_NUM(R)    (__u8)((R & M_CH_NUM) >> 1)


#endif /* _HFCSMCD_H_ */
