/* xhfc_su.c v1.23 2007/05/30
 * mISDN driver for Cologne Chip' XHFC
 *
 * (C) 2007 Copyright Cologne Chip AG
 * Authors : Martin Bachem, Joerg Ciesielski
 * Contact : info@colognechip.com
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
 *
 *
 * MODULE PARAMETERS:
 * (NOTE: layermask and protocol must be given for all ports,
 *  not for the number of cards.)
 *
 * - protocol=<p1>[,p2,p3...]
 *   Values:
 *      <bit  3 -  0>  D-channel protocol id
 *      <bit  4 - 31>  Flags for special features
 *
 *      D-channel protocol ids
 *        - 1       1TR6 (not released yet)
 *        - 2       DSS1
 *
 *      Flags for special features
 *        <bit 4>   0x0010  Net side stack (NT mode)
 *        <bit 5>   0x0020  Line Interface Mode (0=S0, 1=Up)
 *        <bit 6>   0x0040  st line polarity (1=exchanged)
 *        <bit 7>   0x0080  B1 channel loop ST-RX -> XHFC PCM -> ST-TX
 *        <bit 8>   0x0100  B2 channel loop ST-RX -> XHFC PCM -> ST-TX
 *        <bit 9>   0x0200  D channel loop  ST-RX -> XHFC PCM -> ST-TX
 *       <bit 10>   0x0400  Select PTP (DDI) Mode
 *
 * - layermask=<l1>[,l2,l3...] (32bit):
 *        mask of layers to be used for D-channel stack
 *
 * - pcm:
 *      PCM (TDM) bus number. If pcm > -1 pcm audio is transmitted over XHFC
 * 	TDM bus. TDM bus configuration is fixed (by now) and depends on
 * 	pcm_config bitfield. PCM bus number is unique for all cards, i.e. all
 * 	XHFC chips must be connected to the same phisical TDM bus.
 *
 *      Note: Only one XHFC on the PCM bus could be master, i,e, could transmit
 * 	clock and sync signals, all the others must be configured as slaves.
 *      -1 means no support of PCM bus and is the default value. B-channels
 * 	audio (or data) will flow through microprocessor bus interface.
 *
 * - pcm_config=<cfg1>[,cfg2,...]
 *
 *      PCM audio TDM (Time Division Multiplex) bus configuration (one value for
 *      each XHFC installed on TDM).
 *
 *      <bit 0  -  7>  First B-channel Time Slot offset.
 *
 *      Select the first XHFC B-channel timeslot allocation offfset, i.e. the
 * 	first timeslot to be used for transmitting B-channel audio on the TDM
 * 	bus. This TS number will be used to configure the port 1 B1 channel.
 * 	All other XHFC B (audio) channels will be allocated following a natural
 * 	series. Time slot number is calculated using the following equation:
 *
 *      ts_nr = xhfc_ch % 2 + (port - 1) * 2 + pcm_config & 0xFF
 *
 *      Example: time slot allocation table if chip_id = 1 and
 *               pcm_config & 0xFF = 10 (first board), pcm_config & 0xFF = 18
 *               (second board)
 *
 *      XHFC chip_id = 0 (ch offset = 10)      XHFC chip_id = 1 (ch offset = 18)
 *      Port  B1  B2                           Port  B1  B2
 *         1  10  11                              1  18  19
 *         2  12  13                              2  20  21
 *         3  14  15                              3  22  23
 *         4  16  17                              4  24  25
 *
 *      Default is pcm_config[0..7] = 0
 *
 *      <bit 8  - 15>  reserved.
 *
 *      <bit 16 - 17>  PCM Master mode speed:
 *         0 = PCM32 mode: C4IO @ 4096KHz (C2IO @ 2048KHz) => 32 ch (2Mbit/s)
 *         1 = PCM64 mode: C4IO @ 8192KHz => 64 ch (4Mbit/s)
 *         2 = PCM128 mode: C4IO @ 16384 KHz => 128 ch (8Mbit/s)
 *         3 = T1 mode: C4IO @ 1536 KHz => 12 ch (0.75Mbit/s)
 *
 *      <bit 18> Bus master/slave mode selection
 * 	   1 = Set PCM into SLAVE MODE, 0 = Set PCM into MASTER mode
 *             Note: when in slave mode, PCM speed is automatically set.
 *
 *      <bit 19> F0IO pulse length
 * 	   1 = Use Long F0IO pulse (two C4IO or one C2IO clock cycles),
 *         0 = use normal F0IO pulse (one C4IO clock long)
 *
 *      <bit 20 - 22>  Synchronization port:
 *         0 = Auto synchronize on the first available TE port or on SYNC_I
 *         1 = Synchronize always on port 1
 *         2 = Synchronize always on port 2
 *         3 = Synchronize always on port 3
 *         4 = Synchronize always on port 4
 *         5 = Synchronize always on SYNC_I
 *
 *      <bit 23>  1 = Enable C2I (single clock), 0 = enable C4I (double clock)
 *      <bit 24>  1 = Enable C2O single clock output, 0 = Disable C2O.
 *      <bit 25>  1 = F0IO pin is sampled on positive C4IO clock transition,
 *                0 = F0IO pin is sampled on negative C4IO clock transition
 *
 * 	<bit 26>  1 = F0 frame sync signal is active low (STBUS/MVIP type),
 *                0 = F0 frame sync signal is active high
 *
 *      <bit 27>  1 = SYNC_I input is activated as ISDN synchronization source
 *                0 = SYNC_I input is disabled
 *
 * - debug:
 *        enable debugging (see xhfc_su.h for debug options)
 *
 *  NOTE:  If your ISDN line is set up by the telco to be PTP aka DDI
 *         nothing will work unless you took care to set b10 in the protocol
 *         field as shown above
 */

#include <linux/mISDNif.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/proc_fs.h>
#include <asm/timex.h>
#include "core.h"
#include "helper.h"
#include "debug.h"
#include "xhfc_su.h"
#include "xhfc24sucd.h"



#ifdef CONFIG_BR4
#define I2C_LED
#include <linux/i2c-pca9539.h>
#define CONFIG_EC_ZL38065
#ifndef ENABLE_128MS
#define N_SLOTS         8
#else
#define N_SLOTS         16 //DPN: 128 ms support, only even time slots are used
#endif
#else
#define N_SLOTS         9 // Required for CaesarBR4 1FXS + 4BRI (8 channels)
#endif


#if BRIDGE == BRIDGE_PCI2PI
#warning "Selected BRIDGE = PCI2PI"
#define MAX_CARDS	8
#include <linux/pci.h>
#include "xhfc_pci2pi.h"
#endif

#if BRIDGE == BRIDGE_SIMPLE_MEMORY_MAPPED
#define MAX_CARDS	8
#warning "Selected BRIDGE = Simple memory mapped"
#include "xhfc_mem.h"
static xhfc_pi * apiKnown[MAX_CARDS];
static __u32 * pAIC = NULL;
__u32 PCI2PI_XHFC_OFFSETS[PCI2PI_MAX_XHFC];
#endif

#if BRIDGE == BRIDGE_SPI
#warning "Selected BRIDGE = SPI"
#define MAX_CARDS	2
#include "xhfc_spi.h"
static xhfc_pi *pi = NULL;
static unsigned int spi_sel = XHFC_SPI_SEL_DEFAULT;
static unsigned int spi_speed = XHFC_SPI_SPEED_DEFAULT;
/* Transparent packet size to upper layers (bytes) */
static unsigned int trans_packet_size = 256;
#endif

#include "dsp.h"
/* Hardware Echo canceller support */
#if defined(CONFIG_EC_ZL38065)
extern int zl38065_echocan(int channel, int tail_ms);
#define XHFC_HW_EC 1
#else
#undef XHFC_HW_EC
#endif

#ifdef XHFC_HW_EC
static int(*xhfc_ec_driver)(int,int) = NULL;
#endif

static const char xhfc_rev[] = "v1.23 2007/05/30";

static int card_cnt;
static u_int protocol[MAX_CARDS * MAX_PORT];
static int layermask[MAX_CARDS * MAX_PORT];
static int pcm = -1;
static int pcm_config[MAX_CARDS];
static unsigned char sh0h_shape = 0x07;
static unsigned char sh0l_shape = 0x80;
static int fifo_rev = 0;

static u_int dma_rx_bytes = 0;
static u_int dma_tx_bytes = 0;
static u_int misdn_bch_frames_sent = 0;
static u_int misdn_bch_frames_received = 0;

static mISDNobject_t hw_mISDNObj;
static unsigned int debug = 0;


#ifdef MODULE
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
#ifdef OLD_MODULE_PARAM
MODULE_PARM(debug, "1i");
#define MODULE_PARM_T   "1-4i"
MODULE_PARM(protocol, MODULE_PARM_T);
MODULE_PARM(layermask, MODULE_PARM_T);
#if BRIDGE == BRIDGE_SPI
MODULE_PARM(spi_sel, "1i");
MODULE_PARM(spi_speed, "1i");
MODULE_PARM(trans_packet_size, "1i");
#endif
MODULE_PARM(pcm, "1i");
MODULE_PARM(pcm_config, MODULE_PARM_T);
#else
module_param(debug, uint, S_IRUGO | S_IWUSR);
#if BRIDGE == BRIDGE_SPI
module_param(spi_sel, uint, S_IRUGO | S_IWUSR);
module_param(spi_speed, uint, S_IRUGO | S_IWUSR);
module_param(trans_packet_size, uint, S_IRUGO | S_IWUSR);
#endif
module_param(pcm, uint, S_IRUGO | S_IWUSR);
module_param(sh0h_shape, byte, S_IRUGO | S_IWUSR);
module_param(sh0l_shape, byte, S_IRUGO | S_IWUSR);
module_param(fifo_rev, int, S_IRUGO | S_IWUSR);
#ifdef OLD_MODULE_PARAM_ARRAY
static int num_protocol = 0, num_layermask = 0, num_pcm_config = 0;
module_param_array(protocol, uint, num_protocol, S_IRUGO | S_IWUSR);
module_param_array(layermask, uint, num_layermask, S_IRUGO | S_IWUSR);
module_param_array(pcm_config, uint, num_pcm_config, S_IRUGO | S_IWUSR);
#else
static uint num_pcm_config = MAX_CARDS;
module_param_array(protocol, uint, NULL, S_IRUGO | S_IWUSR);
module_param_array(layermask, uint, NULL, S_IRUGO | S_IWUSR);
module_param_array(pcm_config, uint, &num_pcm_config, S_IRUGO | S_IWUSR);
#endif
#endif
#endif

/* static function prototypes */
static void release_card(xhfc_pi * pi);
static void setup_fifo(xhfc_t * xhfc, __u8 fifo, __u8 conhdlc, __u8 subcfg,
    __u8 fifoctrl, __u8 enable);
static void setup_su(xhfc_t * xhfc, __u8 pt, __u8 bc, __u8 enable);
static int  setup_channel(xhfc_t * xhfc, __u8 channel, int protocol);
static int  xhfc_proc_read(char *, char **, off_t, int, int *, void *);
static int  xhfc_proc_write(struct file *, const char *, unsigned long, void *);


/*
 * Physical S/U commands to control Line Interface
 */
static char *HFC_PH_COMMANDS[] = {
	"HFC_L1_ACTIVATE_TE",
	"HFC_L1_FORCE_DEACTIVATE_TE",
	"HFC_L1_ACTIVATE_NT",
	"HFC_L1_DEACTIVATE_NT",
	"HFC_L1_TESTLOOP_B1",
	"HFC_L1_TESTLOOP_B2",
	"HFC_L1_TESTLOOP_D"
};

static void jiffie_delay_unint(int d)
{
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(d);
}

#ifdef XHFC_HW_EC
static int xhfc_echocan(xhfc_chan_t *chan, int eclen)
{
	if(debug)
		printk("echocan: Channel is %d length %d\n",
		       chan->slot_tx, eclen);

	if (xhfc_ec_driver)
		xhfc_ec_driver(chan->slot_tx, eclen);
	else
		printk("echocan: No Hardware driver available\n");

	return 0;
}
#endif

#ifdef I2C_LED
static int led_call_off(int time_slot)
{
	u8 value;
#ifdef ENABLE_128MS
	time_slot=time_slot/2;
#endif
	/* Turning LEDs off */
	if (  time_slot< 4) {
		pca9539_get_byte( &value, PCA9539_OUTPUT_0);
        	pca9539_set_byte(value |= (1 << (time_slot *2) ), PCA9539_OUTPUT_0);
        	pca9539_set_byte(value & ~(1 << (time_slot *2 +1) ), PCA9539_OUTPUT_0);
	} else {
		pca9539_get_byte( &value, PCA9539_OUTPUT_1);
        	pca9539_set_byte(value |= (1 << (((time_slot -4)* -2)+6) ), PCA9539_OUTPUT_1);
        	pca9539_set_byte(value & ~(1 << (((time_slot -4)* -2)+7) ), PCA9539_OUTPUT_1);
	}
}
#endif

#ifdef I2C_LED
static int led_call_on(int time_slot)
{
        u8 value;
#ifdef ENABLE_128MS
        time_slot=time_slot/2;
#endif

/* Turning LEDs on */
	if ( time_slot < 4) {
        	pca9539_get_byte( &value, PCA9539_OUTPUT_0);
                pca9539_set_byte(value &= ~(1 << (time_slot *2) ), PCA9539_OUTPUT_0);
                pca9539_set_byte(value |  (1 << (time_slot *2 +1) ), PCA9539_OUTPUT_0);
        } else {
                pca9539_get_byte( &value, PCA9539_OUTPUT_1);
                pca9539_set_byte(value &= ~(1 << (((time_slot -4)* -2)+6) ), PCA9539_OUTPUT_1);
                pca9539_set_byte(value |  (1 << (((time_slot -4)* -2)+7) ), PCA9539_OUTPUT_1);
         }
}
#endif


static void
xhfc_ph_command(xhfc_port_t * port, u_char command)
{
	xhfc_t * xhfc = port->xhfc;

	if (debug & DEBUG_HFC_S0_STATES)
		printk(KERN_INFO "%s %s: %s (%i)\n",
		    __FUNCTION__, port->name,
		    HFC_PH_COMMANDS[command], command);

	switch (command) {
	case HFC_L1_ACTIVATE_TE:
		write_xhfc(xhfc, R_SU_SEL, port->idx);
		write_xhfc(xhfc, A_SU_WR_STA, STA_ACTIVATE);
		break;

	case HFC_L1_FORCE_DEACTIVATE_TE:
		write_xhfc(xhfc, R_SU_SEL, port->idx);
/* WAS:			write_xhfc(xhfc, A_SU_WR_STA, STA_DEACTIVATE); */
/* TE _IS NOT ALLOWED_ to shut down L1: change to F3 */
			write_xhfc(xhfc, A_SU_WR_STA, M_SU_LD_STA | 0x03);
			udelay(6);
			write_xhfc(xhfc, A_SU_WR_STA, 0);
		break;

	case HFC_L1_ACTIVATE_NT:
		write_xhfc(xhfc, R_SU_SEL, port->idx);
		write_xhfc(xhfc, A_SU_WR_STA,
				STA_ACTIVATE | M_SU_SET_G2_G3);
		break;

	case HFC_L1_DEACTIVATE_NT:
		write_xhfc(xhfc, R_SU_SEL, port->idx);
		write_xhfc(xhfc, A_SU_WR_STA, STA_DEACTIVATE);
		break;

	case HFC_L1_TESTLOOP_B1:
		setup_fifo(xhfc, port->idx*8,   0xC6, 0, 0, 0);
			/* connect B1-SU RX with PCM TX */
		setup_fifo(xhfc, port->idx*8+1, 0xC6, 0, 0, 0);
			/* connect B1-SU TX with PCM RX */

		write_xhfc(xhfc, R_SLOT, port->idx*8);
			/* PCM timeslot B1 TX */
		write_xhfc(xhfc, A_SL_CFG, port->idx*8 + 0x80);
			/* enable B1 TX timeslot on STIO1 */

		write_xhfc(xhfc, R_SLOT, port->idx*8+1);
			/* PCM timeslot B1 RX */
		write_xhfc(xhfc, A_SL_CFG, port->idx*8+1 + 0xC0);
			/* enable B1 RX timeslot on STIO1 */

		setup_su(xhfc, port->idx, 0, 1);
		break;

	case HFC_L1_TESTLOOP_B2:
		setup_fifo(xhfc, port->idx*8+2, 0xC6, 0, 0, 0);
			/* connect B2-SU RX with PCM TX */
		setup_fifo(xhfc, port->idx*8+3, 0xC6, 0, 0, 0);
			/* connect B2-SU TX with PCM RX */

		write_xhfc(xhfc, R_SLOT, port->idx*8+2);
			/* PCM timeslot B2 TX */
		write_xhfc(xhfc, A_SL_CFG, port->idx*8+2 + 0x80);
			/* enable B2 TX timeslot on STIO1 */

		write_xhfc(xhfc, R_SLOT, port->idx*8+3);
			/* PCM timeslot B2 RX */
		write_xhfc(xhfc, A_SL_CFG, port->idx*8+3 + 0xC0);
			/* enable B2 RX timeslot on STIO1 */

		setup_su(xhfc, port->idx, 1, 1);
		break;

	case HFC_L1_TESTLOOP_D:
		setup_fifo(xhfc, port->idx*8+4, 0xC4, 2, M_FR_ABO, 1);
			/* connect D-SU RX with PCM TX */
		setup_fifo(xhfc, port->idx*8+5, 0xC4, 2,
			M_FR_ABO | M_FIFO_IRQMSK, 1);
			/* connect D-SU TX with PCM RX */

		write_xhfc(xhfc, R_SLOT, port->idx*8+4);
			/* PCM timeslot D TX */
		write_xhfc(xhfc, A_SL_CFG, port->idx*8 + 4 + 0x80);
			/* enable D TX timeslot on STIO1 */

		write_xhfc(xhfc, R_SLOT, port->idx*8 + 5);
			/* PCM timeslot D RX */
		write_xhfc(xhfc, A_SL_CFG, port->idx*8 + 5 + 0xC0);
			/* enable D RX timeslot on STIO1 */
		break;
	}
}


static void
l1_timer_start_t3(xhfc_port_t * port)
{
	if (!timer_pending(&port->t3_timer)) {
		if (debug & DEBUG_HFC_S0_STATES)
			printk(KERN_INFO "%s %s\n", __FUNCTION__, port->name);
		port->t3_timer.expires = jiffies + (XHFC_TIMER_T3 * HZ) / 1000;
		add_timer(&port->t3_timer);
	}
}

static void
l1_timer_stop_t3(xhfc_port_t * port)
{
	clear_bit(HFC_L1_ACTIVATING, &port->l1_flags);
	if (timer_pending(&port->t3_timer)) {
		if (debug & DEBUG_HFC_S0_STATES)
			printk(KERN_INFO "%s %s\n", __FUNCTION__, port->name);
		del_timer(&port->t3_timer);
	}
}

/*
 * called when timer t3 expires
 * -> activation failed
 *    force clean L1 deactivation
 */
static void
l1_timer_expire_t3(xhfc_port_t * port)
{
	channel_t * dch = &port->xhfc->chan[D_CH_IDX(port->idx)].ch;

	if (debug & DEBUG_HFC_S0_STATES)
		printk(KERN_INFO "%s %s\n", __FUNCTION__, port->name);

	clear_bit(HFC_L1_ACTIVATING, &port->l1_flags),
	xhfc_ph_command(port, HFC_L1_FORCE_DEACTIVATE_TE);

	mISDN_queue_data(&dch->inst, FLG_MSG_UP,
		(PH_DEACTIVATE | INDICATION),
		0, 0, NULL, 0);
	mISDN_queue_data(&dch->inst, dch->inst.id | MSG_BROADCAST,
		MGR_SHORTSTATUS | INDICATION, SSTATUS_L1_DEACTIVATED,
		0, NULL, 0);
}

static void
l1_timer_start_t4(xhfc_port_t * port)
{
	set_bit(HFC_L1_DEACTTIMER, &port->l1_flags);
	if (!timer_pending(&port->t4_timer)) {
		if (debug & DEBUG_HFC_S0_STATES)
			printk(KERN_INFO "%s %s\n", __FUNCTION__, port->name);

		port->t4_timer.expires =
		    jiffies + (XHFC_TIMER_T4 * HZ) / 1000;
		add_timer(&port->t4_timer);
	}
}

static void
l1_timer_stop_t4(xhfc_port_t * port)
{
	clear_bit(HFC_L1_DEACTTIMER, &port->l1_flags);
	if (timer_pending(&port->t4_timer)) {
		if (debug & DEBUG_HFC_S0_STATES)
			printk(KERN_INFO "%s %s\n", __FUNCTION__, port->name);
		del_timer(&port->t4_timer);
	}
}

/*
 * called when timer t4 expires
 * send (PH_DEACTIVATE | INDICATION) to upper layer
 */
static void
l1_timer_expire_t4(xhfc_port_t * port)
{
	channel_t * dch = &port->xhfc->chan[D_CH_IDX(port->idx)].ch;

	if (debug & DEBUG_HFC_S0_STATES)
		printk(KERN_INFO "%s %s\n", __FUNCTION__, port->name);

	clear_bit(HFC_L1_DEACTTIMER, &port->l1_flags);
	mISDN_queue_data(&dch->inst, FLG_MSG_UP,
		(PH_DEACTIVATE | INDICATION), 0, 0, NULL, 0);
	mISDN_queue_data(&dch->inst, dch->inst.id | MSG_BROADCAST,
		MGR_SHORTSTATUS | INDICATION, SSTATUS_L1_DEACTIVATED,
		0, NULL, 0);
}

/*
 * Line Interface State handler
 */
static void
su_new_state(xhfc_port_t * port)
{
	channel_t * dch = &port->xhfc->chan[D_CH_IDX(port->idx)].ch;
	xhfc_t * xhfc = port->xhfc;
	u_int prim = 0;

	if (port->mode & PORT_MODE_TE) {
		if (debug & DEBUG_HFC_S0_STATES)
			printk(KERN_INFO "%s %s: TE F%d\n",
				__FUNCTION__, port->name, dch->state);

		if ((dch->state <= 3) || (dch->state >= 7))
			l1_timer_stop_t3(port);

		switch (dch->state) {
		case (3):
			if (test_and_clear_bit(HFC_L1_ACTIVATED,
			    &port->l1_flags))
				l1_timer_start_t4(port);
			return;

		case (7):
			if (timer_pending(&port->t4_timer))
				l1_timer_stop_t4(port);

			if (test_and_clear_bit(HFC_L1_ACTIVATING,
			    &port->l1_flags)) {
				if (debug & DEBUG_HFC_S0_STATES)
					printk(KERN_INFO
					    "%s %s: l1->l2 "
					    "(PH_ACTIVATE | CONFIRM)\n",
					    __FUNCTION__, port->name);

				set_bit(HFC_L1_ACTIVATED, &port->l1_flags);
				prim = PH_ACTIVATE | CONFIRM;
			} else {
				if (!(test_and_set_bit(HFC_L1_ACTIVATED,
				    &port->l1_flags))) {
					if (debug & DEBUG_HFC_S0_STATES)
						printk(KERN_INFO
						    "%s %s: l1->l2 "
						    "(PH_ACTIVATE | "
						    "INDICATION)\n",
						    __FUNCTION__, port->name);
					prim = PH_ACTIVATE | INDICATION;
				} else {
					// L1 was already activated
					// (e.g. F8->F7)
					return;
				}
			}
			mISDN_queue_data(&dch->inst, dch->inst.id |
			    MSG_BROADCAST, MGR_SHORTSTATUS | INDICATION,
			    SSTATUS_L1_ACTIVATED, 0, NULL, 0);
			break;

		case (8):
			l1_timer_stop_t4(port);
			return;
		default:
			return;
		}

	} else if (port->mode & PORT_MODE_NT) {

		if (debug & DEBUG_HFC_S0_STATES)
			printk(KERN_INFO "%s %s: NT G%d\n",
				__FUNCTION__, port->name, dch->state);

		switch (dch->state) {
		case (1):
			clear_bit(FLG_ACTIVE, &dch->Flags);
			port->nt_timer = 0;
			port->mode &= ~NT_TIMER;
			prim = (PH_DEACTIVATE | INDICATION);
			if (debug & DEBUG_HFC_S0_STATES)
				printk(KERN_INFO "%s %s: l1->l2 "
				    "(PH_DEACTIVATE | INDICATION)\n",
				    __FUNCTION__, port->name);
			break;
		case (2):
			if (port->nt_timer < 0) {
				port->nt_timer = 0;
				port->mode &= ~NT_TIMER;
				xhfc_ph_command(port, HFC_L1_DEACTIVATE_NT);
			} else {
				port->nt_timer = NT_T1_COUNT;
				port->mode |= NT_TIMER;

				write_xhfc(xhfc, R_SU_SEL, port->idx);
				write_xhfc(xhfc, A_SU_WR_STA, M_SU_SET_G2_G3);
			}
			return;
		case (3):
			set_bit(FLG_ACTIVE, &dch->Flags);
			port->nt_timer = 0;
			port->mode &= ~NT_TIMER;
			prim = (PH_ACTIVATE | INDICATION);

			if (debug & DEBUG_HFC_S0_STATES)
				printk(KERN_INFO "%s %s: l1->l2 "
				    "(PH_ACTIVATE | INDICATION)\n",
				    __FUNCTION__, port->name);
			break;
		case (4):
			port->nt_timer = 0;
			port->mode &= ~NT_TIMER;
			return;
		default:
			break;
		}
		mISDN_queue_data(&dch->inst, dch->inst.id | MSG_BROADCAST,
		    MGR_SHORTSTATUS | INDICATION,
		    test_bit(FLG_ACTIVE, &dch->Flags) ?
		    SSTATUS_L1_ACTIVATED : SSTATUS_L1_DEACTIVATED,
		    0, NULL, 0);
	}

	mISDN_queue_data(&dch->inst, FLG_MSG_UP, prim, 0, 0, NULL, 0);
}

/*
 * Layer 1 D-channel hardware access
 */
static int
handle_dmsg(channel_t *dch, struct sk_buff *skb)
{
	xhfc_t * xhfc = dch->hw;
	xhfc_port_t * port = xhfc->chan[dch->channel].port;

	int		ret = 0;
	mISDN_head_t *hh = mISDN_HEAD_P(skb);

	switch (hh->prim) {
	case (PH_ACTIVATE | REQUEST):
		if ((dch->debug) & (debug & DEBUG_HFC_S0_STATES))
			mISDN_debugprint(&dch->inst,
					"l2->l1 (PH_ACTIVATE | REQUEST)");

		if (port->mode & PORT_MODE_TE) {
			if (test_bit(HFC_L1_ACTIVATED, &port->l1_flags)) {

				if ((dch->debug) &
				    (debug & DEBUG_HFC_S0_STATES))
					mISDN_debugprint(&dch->inst,
					    "l1->l2 (PH_ACTIVATE | CONFIRM)");

				mISDN_queue_data(&dch->inst, FLG_MSG_UP,
							PH_ACTIVATE | CONFIRM,
							0, 0, NULL, 0);
			} else {
				set_bit(HFC_L1_ACTIVATING,
				    &port->l1_flags);

				xhfc_ph_command(port, HFC_L1_ACTIVATE_TE);
				l1_timer_start_t3(port);
			}
		} else {
			if (dch->state == 3) {
				mISDN_queue_data(&dch->inst, FLG_MSG_UP,
						PH_ACTIVATE | INDICATION,
						0, 0, NULL, 0);
			} else {
				xhfc_ph_command(port, HFC_L1_ACTIVATE_NT);
			}
		}
		break;

	case (PH_DEACTIVATE | REQUEST):
		if (port->mode & PORT_MODE_TE) {
			// no deact request in TE mode !
			ret = -EINVAL;
		} else {
			xhfc_ph_command(port, HFC_L1_DEACTIVATE_NT);
		}
		break;

	case (MDL_FINDTEI | REQUEST):
		return (mISDN_queue_up(&dch->inst, 0, skb));
		break;
	}

	return (ret);
}

/*
 * Layer 1 B-channel hardware access
 */
static int
handle_bmsg(channel_t *bch, struct sk_buff *skb)
{
	xhfc_t 	            *xhfc = bch->hw;
	int		    ret = 0;
	mISDN_head_t	    *hh = mISDN_HEAD_P(skb);
	u_long		    flags;
	struct dsp_features features;
	struct sk_buff      *nskb = skb;
#ifdef XHFC_HW_EC
	u32                 cont, taps;
	xhfc_chan_t         *xhfc_ch = container_of(bch, xhfc_chan_t, ch);
#endif

	if ((hh->prim == (PH_ACTIVATE | REQUEST)) ||
	    (hh->prim == (DL_ESTABLISH | REQUEST))) {
#ifdef I2C_LED
		led_call_on(xhfc_ch->slot_tx);
#endif
		if (!test_and_set_bit(FLG_ACTIVE, &bch->Flags)) {
			spin_lock_irqsave(&xhfc->lock, flags);
			if (bch->inst.pid.protocol[2] == ISDN_PID_L2_B_TRANS)
				set_bit(FLG_L2DATA, &bch->Flags);
			ret = setup_channel(xhfc, bch->channel,
			    bch->inst.pid.protocol[1]);
			spin_unlock_irqrestore(&xhfc->lock, flags);
		}
#ifdef FIXME
		if (bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV)
			if (bch->dev)
				if_link(&bch->dev->rport.pif,
					hh->prim | CONFIRM, 0, 0, NULL, 0);
#endif
		skb_trim(skb, 0);
		return (mISDN_queueup_newhead(&bch->inst, 0, hh->prim |
		    CONFIRM, ret, skb));

	} else if ((hh->prim == (PH_DEACTIVATE | REQUEST)) ||
	    (hh->prim == (DL_RELEASE | REQUEST)) ||
	    ((hh->prim == (PH_CONTROL | REQUEST) &&
	    (hh->dinfo == HW_DEACTIVATE)))) {
#ifdef I2C_LED
		if (hh->prim == (DL_RELEASE | REQUEST))
			led_call_off(xhfc_ch->slot_tx);
			
#endif

		spin_lock_irqsave(&xhfc->lock, flags);
		if (test_and_clear_bit(FLG_TX_NEXT, &bch->Flags)) {
			dev_kfree_skb(bch->next_skb);
			bch->next_skb = NULL;
		}
		if (bch->tx_skb) {
			dev_kfree_skb(bch->tx_skb);
			bch->tx_skb = NULL;
		}
		bch->tx_idx = 0;
		if (bch->rx_skb) {
			dev_kfree_skb(bch->rx_skb);
			bch->rx_skb = NULL;
		}
		clear_bit(FLG_L2DATA, &bch->Flags);
		clear_bit(FLG_TX_BUSY, &bch->Flags);
		setup_channel(xhfc, bch->channel, ISDN_PID_NONE);
		clear_bit(FLG_ACTIVE, &bch->Flags);
		spin_unlock_irqrestore(&xhfc->lock, flags);
		skb_trim(skb, 0);
		if (hh->prim != (PH_CONTROL | REQUEST)) {
#ifdef FIXME
			if (bch->inst.pid.protocol[2] == ISDN_PID_L2_B_RAWDEV)
				if (bch->dev)
					if_link(&bch->dev->rport.pif,
						hh->prim | CONFIRM, 0, 0, NULL, 0);
#endif
			if (!mISDN_queueup_newhead(&bch->inst, 0, hh->prim |
						   CONFIRM, 0, skb))
				return (0);
		}
	} else if (hh->prim == (PH_CONTROL | REQUEST)) {

		if (debug)
			printk(KERN_INFO "%s %s: Received prim(%x %s|%s) "
			       "dinfo=%#x\n", xhfc->name, __FUNCTION__,
			       FRIENDLY_PRIM_ARG(hh->prim), hh->dinfo);

		spin_lock_irqsave(&xhfc->lock, flags);

		switch (hh->dinfo) {
#ifdef XHFC_HW_EC
		case 0:
			/* Intercept HW EC commands sent to DSP module when DSP
			 * module is not available (embedded applications)
			 */
			memcpy(&cont, skb->data, sizeof(cont));
			switch(cont) {
			case HW_ECHOCAN_ON:
				if (skb->len < 2 * sizeof(u32)) {
					printk(KERN_WARNING
					       "%s: HW_ECHOCAN_ON lacks "
					       "parameters\n", __FUNCTION__);
				}

				memcpy(&taps, skb->data + sizeof(cont), sizeof(taps));
				xhfc_echocan(xhfc_ch, taps);
				ret = 0;
				break;

			case HW_ECHOCAN_OFF:
				xhfc_echocan(xhfc_ch, 0);
				ret = 0;
				break;
			default:
				if (debug)
					printk(KERN_DEBUG "%s: unknown "
					       "PH_CONTROL info 0 - cont %x\n",
					       __FUNCTION__, cont);
				ret = -EINVAL;
			}
			break;
#endif
		case HW_FEATURES: /* fill features structure */
			if (debug)
				printk(KERN_DEBUG "%s: HW_FEATURE request\n",
				       __FUNCTION__);
			/* create confirm */
			memset(&features, 0, sizeof(features));
			features.hfc_id = xhfc->chipidx;
#ifdef XHFC_HW_EC
			features.hfc_echocanhw = 1;
#endif /* XHFC_HW_EC */
			nskb = create_link_skb(PH_CONTROL | CONFIRM, HW_FEATURES,
					       sizeof(features), &features, 0);
			if (!nskb) {
				printk(KERN_ERR "%s: missed skb allocation!\n",
				       __FUNCTION__);
				break;
			}
			ret = 0;
			/* send confirm */
			if (mISDN_queue_up(&bch->inst, 0, nskb))
				dev_kfree_skb(nskb);
			break;
#ifdef XHFC_HW_EC
		case HW_ECHOCAN_ON:
			if (skb->len < sizeof(u32)) {
				printk(KERN_WARNING
				       "%s: HW_ECHOCAN_ON lacks parameters\n",
				       __FUNCTION__);
			}

			memcpy(&taps, skb->data, sizeof(taps));
			xhfc_echocan(xhfc_ch, taps);

			ret = 0;
			break;

		case HW_ECHOCAN_OFF:
			xhfc_echocan(xhfc_ch, 0);

			ret = 0;
			break;
#endif /* XHFC_HW_EC */
		default:
			if (debug)
				printk(KERN_DEBUG
				       "%s: unknown PH_CONTROL info %x\n",
				       __FUNCTION__, hh->dinfo);
			ret = -EINVAL;
		}

		spin_unlock_irqrestore(&xhfc->lock, flags);

	} else {
		printk(KERN_WARNING "%s %s: unknown prim(%x %s|%s)\n",
		       xhfc->name, __FUNCTION__, FRIENDLY_PRIM_ARG(hh->prim));
	}
	if (!ret)
		dev_kfree_skb(skb);
	return (ret);
}

/*
 * handle Layer2 -> Layer 1 messages
 * (PH_DATA is handled unified for B and D channel, all other
 * messages are redirected to handle_dmsg or handle_bmsg)
 */
static int
xhfc_l2l1(mISDNinstance_t *inst, struct sk_buff *skb)
{
	channel_t	*chan = container_of(inst, channel_t, inst);
	mISDN_head_t	*hh = mISDN_HEAD_P(skb);
	xhfc_t		*xhfc = inst->privat;
	int		ret = 0;
	u_long		flags;

	if ((hh->prim == PH_DATA_REQ) || (hh->prim == DL_DATA_REQ)) {
		spin_lock_irqsave(inst->hwlock, flags);
		ret = channel_senddata(chan, hh->dinfo, skb);
		if (ret > 0) { /* direct TX */
			tasklet_schedule(&xhfc->tasklet);
			// printk ("PH_DATA_REQ: %i bytes in channel(%i)\n",
			//    ret, chan->channel);
			ret = 0;
		}
		spin_unlock_irqrestore(inst->hwlock, flags);
		return (ret);
	}
	if (test_bit(FLG_DCHANNEL, &chan->Flags)) {
		ret = handle_dmsg(chan, skb);
		if (ret != -EAGAIN)
			return (ret);
		ret = -EINVAL;
	}
	if (test_bit(FLG_BCHANNEL, &chan->Flags)) {
		ret = handle_bmsg(chan, skb);
		if (ret != -EAGAIN)
			return (ret);
		ret = -EINVAL;
	}
	if (!ret)
		dev_kfree_skb(skb);
	return (ret);
}

static int
xhfc_manager(void *data, u_int prim, void *arg)
{
	xhfc_t *xhfc = NULL;
	mISDNinstance_t *inst = data;
	struct sk_buff *skb;
	int channel = -1;
	int i;
	channel_t *chan = NULL;
	u_long flags;

	if (!data) {
		MGR_HASPROTOCOL_HANDLER(prim, arg, &hw_mISDNObj)
		    printk(KERN_ERR "%s: no data prim %x %s|%s arg %p\n",
			   __FUNCTION__, FRIENDLY_PRIM_ARG(prim), arg);
		return (-EINVAL);
	}

	spin_lock_irqsave(&hw_mISDNObj.lock, flags);

	/* find channel and card */
	list_for_each_entry(xhfc, &hw_mISDNObj.ilist, list) {
		i = 0;
		while (i < MAX_CHAN) {
			if (xhfc->chan[i].ch.Flags &&
				&xhfc->chan[i].ch.inst == inst) {
				channel = i;
				chan = &xhfc->chan[i].ch;
				break;
			}
			i++;
		}
		if (channel >= 0)
			break;
	}
	spin_unlock_irqrestore(&hw_mISDNObj.lock, flags);

	if (channel < 0) {
		printk(KERN_ERR
		    "%s: no card/channel found  data %p prim %x %s|%s arg %p\n",
		    __FUNCTION__, data, FRIENDLY_PRIM_ARG(prim), arg);
		return (-EINVAL);
	}

	switch (prim) {
		case MGR_REGLAYER | CONFIRM:
			mISDN_setpara(chan, &inst->st->para);
			break;
		case MGR_UNREGLAYER | REQUEST:
			if ((skb = create_link_skb(PH_CONTROL | REQUEST,
				HW_DEACTIVATE, 0, NULL, 0))) {
				if (xhfc_l2l1(inst, skb))
					dev_kfree_skb(skb);
			} else
				printk(KERN_WARNING
				    "no SKB in %s MGR_UNREGLAYER | REQUEST\n",
				    __FUNCTION__);
			mISDN_ctrl(inst, MGR_UNREGLAYER | REQUEST, NULL);
			break;
		case MGR_CLRSTPARA | INDICATION:
			arg = NULL;
		case MGR_ADDSTPARA | INDICATION:
			mISDN_setpara(chan, arg);
			break;
		case MGR_RELEASE | INDICATION:
			if (channel != 2)
				hw_mISDNObj.refcnt--;

			break;
		case MGR_SETSTACK | INDICATION:
			if ((channel != 2) && (inst->pid.global == 2)) {
				if ((skb = create_link_skb(
				    PH_ACTIVATE | REQUEST,
				    0, 0, NULL, 0))) {
					if (xhfc_l2l1(inst, skb))
						dev_kfree_skb(skb);
				}
				if (inst->pid.protocol[2] ==
				    ISDN_PID_L2_B_TRANS)
					mISDN_queue_data(inst, FLG_MSG_UP,
					    DL_ESTABLISH | INDICATION,
					    0, 0, NULL, 0);
				else
					mISDN_queue_data(inst, FLG_MSG_UP,
					    PH_ACTIVATE | INDICATION,
					    0, 0, NULL, 0);
			}
			break;
		case MGR_GLOBALOPT | REQUEST:
			if (arg) {
				// FIXME: detect cards with HEADSET
				u_int *gopt = arg;
				*gopt = GLOBALOPT_INTERNAL_CTRL |
				    GLOBALOPT_EXTERNAL_EQUIPMENT |
				    GLOBALOPT_HANDSET;
			} else
				return (-EINVAL);
			break;
		case MGR_SELCHANNEL | REQUEST:
			// no special procedure
			return (-EINVAL);
			PRIM_NOT_HANDLED(MGR_CTRLREADY | INDICATION);
		default:
			printk(KERN_WARNING
			    "%s %s: prim %x %s|%s "
			    "not handled\n",
			    xhfc->name, __FUNCTION__, FRIENDLY_PRIM_ARG(prim));
			return (-EINVAL);
	}
	return (0);
}

/*
 * check if new buffer for channel
 * is waiting in transmit queue
 */
static int
next_tx_frame(xhfc_t * xhfc, __u8 channel)
{
	channel_t *ch = &xhfc->chan[channel].ch;

	if (ch->tx_skb)
		dev_kfree_skb(ch->tx_skb);
	if (test_and_clear_bit(FLG_TX_NEXT, &ch->Flags)) {
		ch->tx_skb = ch->next_skb;
		if (ch->tx_skb) {
			mISDN_head_t *hh = mISDN_HEAD_P(ch->tx_skb);
			ch->next_skb = NULL;
			clear_bit(FLG_TX_NEXT, &ch->Flags);
			ch->tx_idx = 0;
			queue_ch_frame(ch, CONFIRM, hh->dinfo, NULL);
			return (1);
		} else {
			printk(KERN_WARNING
			    "%s channel(%i) TX_NEXT without skb\n",
			    xhfc->name, channel);
			clear_bit(FLG_TX_NEXT, &ch->Flags);
		}
	} else
		ch->tx_skb = NULL;
	clear_bit(FLG_TX_BUSY, &ch->Flags);
	return (0);
}

static inline void
xhfc_waitbusy(xhfc_t * xhfc)
{
	while (read_xhfc(xhfc, R_STATUS) & M_BUSY);
}

static inline void
xhfc_selfifo(xhfc_t * xhfc, __u8 fifo)
{
	write_xhfc(xhfc, R_FIFO, fifo);
	xhfc_waitbusy(xhfc);
}

static inline void
xhfc_inc_f(xhfc_t * xhfc)
{
	write_xhfc(xhfc, A_INC_RES_FIFO, M_INC_F);
	xhfc_waitbusy(xhfc);
}

static inline void
xhfc_resetfifo(xhfc_t * xhfc)
{
	write_xhfc(xhfc, A_INC_RES_FIFO, M_RES_FIFO | M_RES_FIFO_ERR);
	xhfc_waitbusy(xhfc);
}

/*
 * fill fifo with TX data
 */
static void
xhfc_write_fifo(xhfc_t * xhfc, __u8 channel)
{
	__u8		fcnt, tcnt, i;
	__u8		free;
	__u8		f1, f2;
	__u8		fstat;
	__u8		*data;
	int		remain;
	channel_t	*ch = &xhfc->chan[channel].ch;


send_buffer:
	if (!ch->tx_skb)
		return;
	remain = ch->tx_skb->len - ch->tx_idx;
	if (remain <= 0)
		return;

	xhfc_selfifo(xhfc, (channel * 2));

	fstat = read_xhfc(xhfc, A_FIFO_STA);
	free = (xhfc->max_z - (read_xhfc(xhfc, A_USAGE)));
	tcnt = (free >= remain) ? remain : free;

	f1 = read_xhfc(xhfc, A_F1);
	f2 = read_xhfc(xhfc, A_F2);

	fcnt = 0x07 - ((f1 - f2) & 0x07); /* free frame count in tx fifo */

	if (debug & DEBUG_HFC_FIFO) {
		mISDN_debugprint(&ch->inst,
		    "%s channel(%i) len(%i) idx(%i) f1(%i) f2(%i) "
		    "fcnt(%i) tcnt(%i) free(%i) fstat(%i)",
		    __FUNCTION__, channel, ch->tx_skb->len, ch->tx_idx,
		    f1, f2, fcnt, tcnt, free, fstat);
	}

	/* check for fifo underrun during frame transmission */
	fstat = read_xhfc(xhfc, A_FIFO_STA);
	if (fstat & M_FIFO_ERR) {
		if (debug & DEBUG_HFC_FIFO_ERR) {
			mISDN_debugprint(&ch->inst,
			    "%s transmit fifo channel(%i) underrun idx(%i), "
			    "A_FIFO_STA(0x%02x)",
			    __FUNCTION__, channel, ch->tx_idx, fstat);
		}

		write_xhfc(xhfc, A_INC_RES_FIFO, M_RES_FIFO_ERR);

		/* restart frame transmission */
		if ((test_bit(FLG_HDLC, &ch->Flags)) && ch->tx_idx) {
			ch->tx_idx = 0;
			goto send_buffer;
		}
	}

	if (free && fcnt && tcnt) {
		if (ch->tx_skb == NULL) {
			BUG();
			goto send_buffer;
		}
		data = ch->tx_skb->data + ch->tx_idx;
		ch->tx_idx += tcnt;

		if (debug & DEBUG_HFC_FIFO) {
			printk("%s channel(%i) writing: ",
			    xhfc->name, channel);

			i = 0;
			while (i < tcnt)
				printk("%02x ", *(data+(i++)));
			printk("\n");
		}

		/* write data to FIFO */
		i = 0;
		while (i < tcnt) {
			if ((tcnt-i) >= 4) {
#ifdef CONFIG_BLACKFIN
				write32_xhfc(xhfc, A_FIFO_DATA,
					(((u32)*(data + i + 0)) << 0) |
					(((u32)*(data + i + 1)) << 8) |
					(((u32)*(data + i + 2)) << 16) |
					(((u32)*(data + i + 3)) << 24));
#else
				write32_xhfc(xhfc, A_FIFO_DATA,
				    *((__u32 *) (data+i)));
#endif
				i += 4;
			} else {
				write_xhfc(xhfc, A_FIFO_DATA, *(data+i));
				i++;
			}
		}

		if (ch->tx_idx == ch->tx_skb->len) {
			if (test_bit(FLG_HDLC, &ch->Flags)) {
				/* terminate frame */
				xhfc_inc_f(xhfc);
			} else {
				xhfc_selfifo(xhfc, (channel * 2));
			}

			/* check for fifo underrun during frame transmission */
			fstat = read_xhfc(xhfc, A_FIFO_STA);
			if (fstat & M_FIFO_ERR) {
				if (debug & DEBUG_HFC_FIFO_ERR) {
					mISDN_debugprint(&ch->inst,
					    "%s transmit fifo channel(%i) "
					    "underrun during transmission, "
					    "A_FIFO_STA(0x%02x)\n",
					    __FUNCTION__,
					    channel,
					    fstat);
				}
				write_xhfc(xhfc, A_INC_RES_FIFO,
				    M_RES_FIFO_ERR);

				if (test_bit(FLG_HDLC, &ch->Flags)) {
					// restart frame transmission
					ch->tx_idx = 0;
					goto send_buffer;
				}
			}

			if (next_tx_frame(xhfc, channel)) {
				if (debug & DEBUG_HFC_BTRACE)
					mISDN_debugprint(&ch->inst,
					    "channel(%i) has next_tx_frame",
					    channel);
				if ((free - tcnt) > 8) {
					if (debug & DEBUG_HFC_BTRACE)
						mISDN_debugprint(&ch->inst,
						    "channel(%i) continue "
						    "B-TX immediately",
						    channel);
					goto send_buffer;
				}
			}

		} else {
			/* tx buffer not complete, but fifo filled to maximum */
			xhfc_selfifo(xhfc, (channel * 2));
		}
	}
}

/*
 * read RX data out of fifo
 */
static void
xhfc_read_fifo(xhfc_t * xhfc, __u8 channel)
{
	__u8	f1 = 0, f2 = 0, z1 = 0, z2 = 0;
	__u8	fstat = 0;
	int	i;
	int	rcnt;		/* read rcnt bytes out of fifo */
	__u8	*data;		/* new data pointer */
	struct sk_buff	*skb;	/* data buffer for upper layer */
	channel_t	*ch = &xhfc->chan[channel].ch;

receive_buffer:

	xhfc_selfifo(xhfc, (channel * 2) + 1);

	fstat = read_xhfc(xhfc, A_FIFO_STA);
	if (fstat & M_FIFO_ERR) {
		if (debug & DEBUG_HFC_FIFO_ERR)
			mISDN_debugprint(&ch->inst,
			    "RX fifo overflow channel(%i), "
			    "A_FIFO_STA(0x%02x) f0cnt(%i)",
			    channel, fstat, xhfc->f0_akku);
		write_xhfc(xhfc, A_INC_RES_FIFO, M_RES_FIFO_ERR);
	}

	if (test_bit(FLG_HDLC, &ch->Flags)) {
		/* hdlc rcnt */
		f1 = read_xhfc(xhfc, A_F1);
		f2 = read_xhfc(xhfc, A_F2);
		z1 = read_xhfc(xhfc, A_Z1);
		z2 = read_xhfc(xhfc, A_Z2);

		rcnt = (z1 - z2) & xhfc->max_z;
		if (f1 != f2)
			rcnt++;

	} else {
		/* transparent rcnt */
		rcnt = read_xhfc(xhfc, A_USAGE) - 1;
	}

	if (debug & DEBUG_HFC_FIFO) {
		if (ch->rx_skb)
			i = ch->rx_skb->len;
		else
			i = 0;
		mISDN_debugprint(&ch->inst,
		    "reading %i bytes channel(%i) "
		    "irq_cnt(%i) fstat(%i) idx(%i) f1(%i) f2(%i) z1(%i) z2(%i)",
		    rcnt, channel, xhfc->irq_cnt, fstat, i, f1, f2, z1, z2);
	}

	if (ch->rx_skb) {
		if (skb_tailroom(ch->rx_skb) < (rcnt + ch->up_headerlen + 3)) {
			queue_ch_frame(ch, INDICATION, MISDN_ID_ANY,
			    ch->rx_skb);
			ch->rx_skb = NULL;
		}
	}

	if (rcnt > 0) {
		if (!ch->rx_skb) {
			ch->rx_skb = alloc_stack_skb(
			    ch->maxlen + 3, ch->up_headerlen);
			if (!ch->rx_skb) {
				printk(KERN_DEBUG "%s: No mem for rx_skb\n",
				    __FUNCTION__);
				return;
			}
		}
		data = skb_put(ch->rx_skb, rcnt);

		/* read data from FIFO */
		i = 0;
		while (i < rcnt) {
			if ((rcnt - i) >= 4) {
#ifdef CONFIG_BLACKFIN
				u32 __rd_data = read32_xhfc(xhfc, A_FIFO_DATA);
				*(data + i + 0) =
					(u8)((__rd_data & 0x000000FF) >> 0);
				*(data + i + 1) =
					(u8)((__rd_data & 0x0000FF00) >> 8);
				*(data + i + 2) =
					(u8)((__rd_data & 0x00FF0000) >> 16);
				*(data + i + 3) =
					(u8)((__rd_data & 0xFF000000) >> 24);
#else
				*((__u32 *) (data + i)) = read32_xhfc(xhfc,
				    A_FIFO_DATA);
#endif
				i += 4;
			} else {
				*(data + i) = read_xhfc(xhfc, A_FIFO_DATA);
				i++;
			}
		}
	} else
		return;

	if (test_bit(FLG_HDLC, &ch->Flags)) {
		if (f1 != f2) {
			xhfc_inc_f(xhfc);

			if ((ch->debug) && (debug & DEBUG_HFC_DTRACE)) {
				mISDN_debugprint(&ch->inst,
					"channel(%i) new RX len(%i): ",
					channel, ch->rx_skb->len);
				i = 0;
				printk("  ");
				while (i < ch->rx_skb->len)
					printk("%02x ", ch->rx_skb->data[i++]);
				printk("\n");
			}

			/* check minimum frame size */
			if (ch->rx_skb->len < 4) {
				if (debug & DEBUG_HFC_FIFO_ERR)
					mISDN_debugprint(&ch->inst,
					    "%s: frame in channel(%i) < "
					    "minimum size",
					    __FUNCTION__, channel);
				goto read_exit;
			}

			/* check crc */
			if (ch->rx_skb->data[ch->rx_skb->len - 1]) {
				if (debug & DEBUG_HFC_FIFO_ERR)
					mISDN_debugprint(&ch->inst,
					    "%s: channel(%i) CRC-error",
					    __FUNCTION__, channel);
				goto read_exit;
			}

			/* remove cksum */
			skb_trim(ch->rx_skb, ch->rx_skb->len - 3);

			if (ch->rx_skb->len < MISDN_COPY_SIZE) {

				skb = alloc_stack_skb(ch->rx_skb->len,
				    ch->up_headerlen);
				if (skb) {

					memcpy(skb_put(skb, ch->rx_skb->len),
					    ch->rx_skb->data, ch->rx_skb->len);

					skb_trim(ch->rx_skb, 0);

				} else {
					skb = ch->rx_skb;
					ch->rx_skb = NULL;
				}
			} else {
				skb = ch->rx_skb;
				ch->rx_skb = NULL;
			}

			queue_ch_frame(ch, INDICATION, MISDN_ID_ANY, skb);

		read_exit:
			if (ch->rx_skb)
				skb_trim(ch->rx_skb, 0);
			if (read_xhfc(xhfc, A_USAGE) > 8) {
				if (debug & DEBUG_HFC_FIFO)
					mISDN_debugprint(&ch->inst,
					    "%s: channel(%i) continue "
					    "xhfc_read_fifo",
					    __FUNCTION__, channel);
				goto receive_buffer;
			}
			return;


		} else {
			xhfc_selfifo(xhfc, (channel * 2) + 1);
		}
	} else {
		xhfc_selfifo(xhfc, (channel * 2) + 1);
		if (ch->rx_skb->len >= TRANSP_PACKET_SIZE) {
			/* deliver transparent data to layer2 */
			queue_ch_frame(ch, INDICATION, MISDN_ID_ANY,
			    ch->rx_skb);
			ch->rx_skb = NULL;
		}
	}
}

/*
 * bottom half handler for interrupt
 */
static void
xhfc_bh_handler(unsigned long ul_hw)
{
	xhfc_t 	*xhfc = (xhfc_t *) ul_hw;
	int		i;
	__u8		su_state;
	channel_t	*dch;

	/* timer interrupt */
	if (GET_V_TI_IRQ(xhfc->misc_irq)) {
		xhfc->misc_irq &= (__u8)(~M_TI_IRQ);

		/* Handle tx Fifos */
		for (i = 0; i < xhfc->max_fifo; i++) {
			if ((1 << (i * 2)) & (xhfc->fifo_irqmsk)) {
				xhfc->fifo_irq &= ~(1 << (i * 2));
				if (test_bit(FLG_TX_BUSY,
				    &xhfc->chan[i].ch.Flags)) {
					xhfc_write_fifo(xhfc, i);
				}
			}
		}

		/* handle NT Timer */
		for (i = 0; i < xhfc->num_ports; i++) {
			if ((xhfc->port[i].mode & PORT_MODE_NT) &&
			    (xhfc->port[i].mode & NT_TIMER)) {
				if ((--xhfc->port[i].nt_timer) < 0)
					su_new_state(&xhfc->port[i]);
			}
		}
	}

	/* set fifo_irq when RX data over treshold */
	for (i = 0; i < xhfc->num_ports; i++) {
		xhfc->fifo_irq |= read_xhfc(xhfc, R_FILL_BL0 + i) << (i * 8);
	}

	/* Handle rx Fifos */
	if ((xhfc->fifo_irq & xhfc->fifo_irqmsk) & FIFO_MASK_RX) {
		for (i = 0; i < xhfc->max_fifo; i++) {
			if ((xhfc->fifo_irq & (1 << (i * 2 + 1)))
			    & (xhfc->fifo_irqmsk)) {

				xhfc->fifo_irq &= ~(1 << (i * 2 + 1));
				xhfc_read_fifo(xhfc, i);
			}
		}
	}

	/* su interrupt */
	if (xhfc->su_irq & xhfc->su_irqmsk) {
		xhfc->su_irq = 0;
		for (i = 0; i < xhfc->num_ports; i++) {
			write_xhfc(xhfc, R_SU_SEL, i);
			su_state = read_xhfc(xhfc, A_SU_RD_STA);

			dch = &xhfc->chan[D_CH_IDX(i)].ch;
			if (GET_V_SU_STA(su_state) != dch->state) {
				dch->state = GET_V_SU_STA(su_state);
				su_new_state(&xhfc->port[i]);
			}
		}
	}
}

/*
 * Interrupt handler
 */
static irqreturn_t
#ifdef OLD_IRQ_CALL
xhfc_interrupt(int intno, void *dev_id, struct pt_regs *regs)
#else
xhfc_interrupt(int intno, void *dev_id)
#endif
{
	xhfc_pi *pi = dev_id;
	xhfc_t * xhfc = NULL;
	__u8 i, j;
	__u32 xhfc_irqs, xhfc_with_irqs;
#ifdef USE_F0_COUNTER
	__u32 f0_cnt;
#endif
	xhfc_with_irqs = 0;

	for (i = 0; i < pi->driver_data.num_xhfcs; i++) {
		xhfc = &pi->xhfc[i];
		if (GET_V_GLOB_IRQ_EN(xhfc->irq_ctrl) && (read_xhfc(xhfc, R_IRQ_OVIEW)))
			/* mark this xhfc possibly had irq */
			xhfc_with_irqs |= (1 << i);
	}
	if (!xhfc_with_irqs) {
		if (debug & DEBUG_HFC_IRQ)
			printk(KERN_INFO
			    "%s %s NOT M_GLOB_IRQ_EN or R_IRQ_OVIEW \n",
			    xhfc->name, __FUNCTION__);
		return IRQ_NONE;
	}

	xhfc_irqs = 0;
	for (i = 0; (i < pi->driver_data.num_xhfcs) && xhfc_with_irqs; i++) {

		if (!(xhfc_with_irqs & (1 << i)))
			continue;

		xhfc_with_irqs &= ~(1 << i);

		xhfc = &pi->xhfc[i];

		xhfc->misc_irq |= read_xhfc(xhfc, R_MISC_IRQ);
		xhfc->su_irq |= read_xhfc(xhfc, R_SU_IRQ);

		/* get fifo IRQ states in bundle */
		for (j = 0; j < 4; j++) {
			xhfc->fifo_irq |=
			    (read_xhfc(xhfc, R_FIFO_BL0_IRQ + j) << (j * 8));
		}

		/*
		 * call bottom half at events
		 *   - Timer Interrupt (or other misc_irq sources)
		 *   - SU State change
		 *   - Fifo FrameEnd interrupts (only at rx fifos enabled)
		 */
		if ((xhfc->misc_irq & xhfc->misc_irqmsk)
		      || (xhfc->su_irq & xhfc->su_irqmsk)
		      || (xhfc->fifo_irq & xhfc->fifo_irqmsk)) {

		      	/* mark this xhfc really had irq */
			xhfc_irqs |= (1 << i);

			/* queue bottom half */
			if (!(xhfc->testirq))
				tasklet_schedule(&xhfc->tasklet);

			/* count irqs */
			xhfc->irq_cnt++;

#ifdef USE_F0_COUNTER
			/* accumulate f0 counter diffs */
			f0_cnt = read_xhfc(xhfc, R_F0_CNTL);
			f0_cnt += read_xhfc(xhfc, R_F0_CNTH) << 8;
			xhfc->f0_akku += (f0_cnt - xhfc->f0_cnt);
			if ((f0_cnt - xhfc->f0_cnt) < 0)
				xhfc->f0_akku += 0xFFFF;
			xhfc->f0_cnt = f0_cnt;
#endif
		}
	}

#if BRIDGE == BRIDGE_SIMPLE_MEMORY_MAPPED
	pAIC[0x130 >>2] = 0;
#endif

	return ((xhfc_irqs) ? IRQ_HANDLED : IRQ_NONE);
}

#if defined(XHFC_BFSI_IRQ_PROCESSING)
/*
 * Blackfin BFSI interrupt / DMA management functions
 *
 */

/* Call once every 4 ISR calls (i.e. every 4ms) */
//#define XHFC_SOFT_IRQ_DIVIDER 	4
#define XHFC_SOFT_IRQ_DIVIDER 	1
/*
 * Soft Interrupt handler
 * This ISR gets called from bfsi DMA service interrupt (INTERRUPT CONTEXT)
 */
static void
xhfc_soft_irq_process(void* dev_id)
{
	xhfc_pi *pi = (xhfc_pi *)dev_id;

	if (unlikely(pi == NULL)) {
		BUG();
		return;
	}
	pi->irq_nr++;

	if ((pi->irq_nr % (XHFC_SOFT_IRQ_DIVIDER)) == 0) {
#ifdef	OLD_IRQ_CALL
		xhfc_interrupt(pi->irq_nr, dev_id, NULL);
#else
		xhfc_interrupt(pi->irq_nr, dev_id);
#endif
	}
}

/* sample cycles register of Blackfin */

static inline unsigned int bf_cycles(void) {
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

/*
 * PCM audio: Blackfin Serial Port DMA buffer processing.
 * Transmit direction (Blackfin --> TDM TX line)
 */
static inline void xhfc_tx_prep(xhfc_t *xhfc, channel_t *ch, u16 b_ch,
	u8 *writechunk)
{
	u16 chunk, remain, t_slot, tx_samples;
	u8 *data = NULL;

	remain = ch->tx_skb->len - ch->tx_idx;
	data = ch->tx_skb->data + ch->tx_idx;
	t_slot = xhfc->chan[b_ch].slot_tx;
	tx_samples = min((u16)SAMPLES_PER_CHUNK, remain);
	for (chunk = 0; chunk < tx_samples; chunk++) {
		writechunk[N_SLOTS * chunk + t_slot] = *(data++);
	}
	dma_tx_bytes += tx_samples;
	ch->tx_idx += tx_samples;
	if (ch->tx_idx == ch->tx_skb->len) {
		if (next_tx_frame(xhfc, b_ch)) {
			misdn_bch_frames_received++;
			if (debug & DEBUG_HFC_BTRACE)
				mISDN_debugprint(&ch->inst,
					"channel(%i) has next_tx_frame", b_ch);
		}
	}
}

/*
 * PCM audio: Blackfin Serial Port DMA buffer processing.
 * Receive direction (Blackfin <-- TDM RX line)
 */
static inline void xhfc_rx_prep(xhfc_t *xhfc, channel_t *ch, u16 b_ch,
	u8 *readchunk)
{
	u16 chunk, t_slot;
	u8 *data = NULL;

	data = skb_put(ch->rx_skb, SAMPLES_PER_CHUNK);
	t_slot = xhfc->chan[b_ch].slot_rx;
	for (chunk = 0; chunk < SAMPLES_PER_CHUNK; chunk++) {
		*(data++) = readchunk[N_SLOTS * chunk + t_slot];
	}
	dma_rx_bytes += SAMPLES_PER_CHUNK;
	if (ch->rx_skb->len >= trans_packet_size) {
		/* deliver transparent data to layer2 */
		queue_ch_frame(ch, INDICATION, MISDN_ID_ANY, ch->rx_skb);
		ch->rx_skb = NULL;
		misdn_bch_frames_sent++;
	}
}

/*
 * PCM audio: Blackfin Serial Port DMA buffer processing.
 */
static inline void xhfc_txrx_prep(xhfc_t *xhfc, u8 *writechunk, u8 *readchunk)
{
	u16 b_ch, port;
	channel_t *ch = NULL;
#ifdef XHFC_BFSI_DMA_DEBUG
	int cycles_before;
#endif

	BUG_ON(xhfc == NULL);

	// TODO: cleanup
	if (unlikely(xhfc->testirq))
		return;

	BUG_ON((xhfc->chan == NULL) || (xhfc->port == NULL));

#ifdef XHFC_BFSI_DMA_DEBUG
	cycles_before = bf_cycles();
#endif

	for (port = 0; port < xhfc->num_ports; port++) {
		for (b_ch = port * 4; b_ch <= port * 4 + 1; b_ch++) {
			ch = &xhfc->chan[b_ch].ch;

			/* TX Prep */
			if (test_bit(FLG_TX_BUSY, &ch->Flags)) {
				if (likely(ch->tx_skb != NULL)) {
					xhfc_tx_prep(xhfc, ch, b_ch,writechunk);
				}
			}
#ifdef XHFC_BFSI_DMA_DEBUG
			xhfc->cycles_txprep = bf_cycles() - cycles_before;
#endif
			/* RX Prep */
			if (test_bit(FLG_ACTIVE, &ch->Flags)) {
				if (likely(ch->rx_skb == NULL)) {
					ch->rx_skb = alloc_stack_skb(ch->maxlen
						+ 3, ch->up_headerlen);
					if (!ch->rx_skb) {
						printk(KERN_WARNING "%s: No mem "
						       "for rx_skb\n",
						       __FUNCTION__);
						return;
					}
				}
				xhfc_rx_prep(xhfc, ch, b_ch, readchunk);
			}
#ifdef XHFC_BFSI_DMA_DEBUG
			xhfc->cycles_rxprep = bf_cycles() - cycles_before -
				xhfc->cycles_txprep;
#endif
		}
	}
}

static void
xhfc_pcm_dma_process(u8 *read_samples, u8 *write_samples)
{
	int i;

	for (i = 0; i < pi->driver_data.num_xhfcs; i++) {
		xhfc_txrx_prep(&pi->xhfc[i], write_samples, read_samples);
	}
}

static void
disable_dma(xhfc_t * xhfc)
{
	if (debug & DEBUG_HFC_IRQ)
		printk(KERN_INFO "%s %s\n", xhfc->name, __FUNCTION__);

	bfsi_soft_dma_disable(xhfc->pi->irq);
}

static void
enable_dma(xhfc_t * xhfc)
{
	if (debug & DEBUG_HFC_IRQ)
		printk(KERN_INFO "%s %s\n", xhfc->name, __FUNCTION__);

	bfsi_soft_dma_enable(xhfc->pi->irq);
}
#endif /* #if defined(XHFC_BFSI_IRQ_PROCESSING) */


/*
 * disable all interrupts by disabling M_GLOB_IRQ_EN
 */
static void
disable_interrupts(xhfc_t * xhfc)
{
	if (debug & DEBUG_HFC_IRQ)
		printk(KERN_INFO "%s %s\n", xhfc->name, __FUNCTION__);

#ifdef XHFC_BFSI_IRQ_PROCESSING
	if (xhfc->testirq)
		disable_dma(xhfc);
	bfsi_soft_irq_disable(xhfc->pi->irq);
#endif
        SET_V_GLOB_IRQ_EN(xhfc->irq_ctrl, 0);
        write_xhfc(xhfc, R_IRQ_CTRL, xhfc->irq_ctrl);
}

/*
 * start interrupt and set interrupt mask
 */
static void
enable_interrupts(xhfc_t * xhfc)
{
	if (debug & DEBUG_HFC_IRQ)
		printk(KERN_INFO "%s %s\n", xhfc->name, __FUNCTION__);
	
	write_xhfc(xhfc, R_SU_IRQMSK, xhfc->su_irqmsk);

	/* use defined timer interval */
	write_xhfc(xhfc, R_TI_WD, xhfc->ti_wd);
	SET_V_TI_IRQMSK(xhfc->misc_irqmsk, 1);
	write_xhfc(xhfc, R_MISC_IRQMSK, xhfc->misc_irqmsk);

	/* clear all pending interrupts bits */
	read_xhfc(xhfc, R_MISC_IRQ);
	read_xhfc(xhfc, R_SU_IRQ);
	read_xhfc(xhfc, R_FIFO_BL0_IRQ);
	read_xhfc(xhfc, R_FIFO_BL1_IRQ);
	read_xhfc(xhfc, R_FIFO_BL2_IRQ);
	read_xhfc(xhfc, R_FIFO_BL3_IRQ);

	/* enable global interrupts */
	SET_V_GLOB_IRQ_EN(xhfc->irq_ctrl, 1);
	SET_V_FIFO_IRQ_EN(xhfc->irq_ctrl, 1);
	write_xhfc(xhfc, R_IRQ_CTRL, xhfc->irq_ctrl);

#ifdef XHFC_BFSI_IRQ_PROCESSING
	if (xhfc->testirq)
		enable_dma(xhfc);
	bfsi_soft_irq_enable(xhfc->pi->irq);
#endif

}

/*
 * initialise the PCM TDM bus
 * return 0 on success.
 */
static int
xhfc_pcm_init(xhfc_t * xhfc)
{
	char * sync_ports[] = {"AUTO", "1", "2", "3", "4", "SYNC_I"};
	__u8 sync_port = XHFC_PCM_SYNC_PORT_GET(xhfc->pcm_config);
	__u8 sl_sel0;
	__u8 sh0l;
	__u8 sh0h;

	if (debug & DEBUG_HFC_INIT) {
		printk(KERN_INFO
		"%s %s: Setting up PCM audio over TDM: pcm_config=%#x\n"
		"TDM Bus mode: %s\n"
		"TDM Bit rate: %s Mbit/s\n"
		"C2I clock input:%s\n"
		"C2O clock output:%s\n"
		"C4POL: F0IO sampled on %s clock transition\n"
		"F0IO signal is ACTIVE %s\n"
		"ISDN Sync port: %s\n"
		"SYNC_I is %s\n",
		__FUNCTION__, xhfc->name,
		xhfc->pcm_config,
		xhfc->pcm_config & XHFC_PCM_SLAVE_MODE ? "SLAVE":"MASTER",
		XHFC_PCM_MASTER_SPEED_GET(xhfc->pcm_config) == 0 ? "2":
		XHFC_PCM_MASTER_SPEED_GET(xhfc->pcm_config) == 1 ? "4":
		XHFC_PCM_MASTER_SPEED_GET(xhfc->pcm_config) == 2 ? "8":
		XHFC_PCM_MASTER_SPEED_GET(xhfc->pcm_config) == 3 ? "0.75":"ERR",
		xhfc->pcm_config & XHFC_PCM_C2I_EN ? "ON": "OFF",
		xhfc->pcm_config & XHFC_PCM_C2O_EN ? "ON": "OFF",
		xhfc->pcm_config & XHFC_PCM_C4_POL ? "POS" : "NEG",
		xhfc->pcm_config & XHFC_PCM_F0IO_A_LOW ? "LOW" : "HIGH",
		XHFC_PCM_SYNC_PORT_GET(xhfc->pcm_config) <
			ARRAY_SIZE(sync_ports) ?
		sync_ports[XHFC_PCM_SYNC_PORT_GET(xhfc->pcm_config)] : "ERROR",
		xhfc->pcm_config & XHFC_PCM_SYNC_I_ENABLE ? "ENABLED":"DISABLED"
		);
	}


	/* Set TDM bus master/slave mode. Register: R_PCM_MD0 */
	SET_V_PCM_MD(xhfc->pcm_md0,
		xhfc->pcm_config & XHFC_PCM_SLAVE_MODE ? 0 : 1);
	SET_V_F0_LEN(xhfc->pcm_md0,
		xhfc->pcm_config & XHFC_PCM_LONG_F0IO ? 1 : 0);
	SET_V_C4_POL(xhfc->pcm_md0,
		xhfc->pcm_config & XHFC_PCM_C4_POL ? 1 : 0);
	SET_V_F0_NEG(xhfc->pcm_md0,
		xhfc->pcm_config & XHFC_PCM_F0IO_A_LOW ? 1 : 0);

	write_xhfc(xhfc, R_PCM_MD0, xhfc->pcm_md0);

	/* TDM Master-only setup */
	if (!(xhfc->pcm_config & XHFC_PCM_SLAVE_MODE)) {
		if (debug & DEBUG_HFC_INIT) {
			printk(KERN_INFO "%s: Setting TDM master speed = %d\n",
				__FUNCTION__, GET_V_PCM_DR(xhfc->pcm_md1));
		}
		/* PCM: set pll adjust, TDM data rate. Register: R_PCM_MD1 */
		SET_V_PCM_IDX(xhfc->pcm_md0, IDX_PCM_MD1);
		xhfc->pcm_md1 = 0;
		SET_V_PLL_ADJ(xhfc->pcm_md1, 3);
		SET_V_PCM_DR(xhfc->pcm_md1,
			XHFC_PCM_MASTER_SPEED_GET(xhfc->pcm_config));
		write_xhfc(xhfc, R_PCM_MD0, xhfc->pcm_md0);
		write_xhfc(xhfc, R_PCM_MD1, xhfc->pcm_md1);

		/* PCM: TDM clock signal setup: Enable C20/C2I or C4O/C4I.
		 * Register: R_PCM_MD2 */
		SET_V_PCM_IDX(xhfc->pcm_md0, IDX_PCM_MD2);
		xhfc->pcm_md2 = 0x0;
		SET_V_C2I_EN(xhfc->pcm_md2,
			xhfc->pcm_config & XHFC_PCM_C2I_EN ? 1 : 0);
		SET_V_C2O_EN(xhfc->pcm_md2,
			xhfc->pcm_config & XHFC_PCM_C2O_EN ? 1 : 0);

		if (debug & DEBUG_HFC_INIT) {
			printk(KERN_INFO "%s: Setting TDM clock: C2I=%s, "
				"C2O=%s\n", __FUNCTION__,
				GET_V_C2I_EN(xhfc->pcm_md2) ? "ON" : "OFF",
				GET_V_C2O_EN(xhfc->pcm_md2) ? "ON" : "OFF");
		}

		write_xhfc(xhfc, R_PCM_MD0, xhfc->pcm_md0);
		write_xhfc(xhfc, R_PCM_MD2, xhfc->pcm_md2);

		/* PCM: TDM frame sync signal. Use F1_0 to shape signal */
		SET_V_PCM_IDX(xhfc->pcm_md0, IDX_SL_SEL0);
		sl_sel0 = 0;
		SET_V_SL_SEL0(sl_sel0, 0x1F); /* Time slot 0 => 0x1F + 1*/
		SET_V_SH_SEL0(sl_sel0, 0);    /* Select Shape type 0 */
		write_xhfc(xhfc, R_PCM_MD0, xhfc->pcm_md0);
		write_xhfc(xhfc, R_SL_SEL0, sl_sel0);

		/* Shape register 0 setup */
		SET_V_PCM_IDX(xhfc->pcm_md0, IDX_SH0L);
		sh0l = 0;
		/* Reversed bitmask: see XHFC manual par 6.7 */
		SET_V_SH0L(sh0l, sh0l_shape);
		write_xhfc(xhfc, R_PCM_MD0, xhfc->pcm_md0);
		write_xhfc(xhfc, R_SH0L, sh0l);

		SET_V_PCM_IDX(xhfc->pcm_md0, IDX_SH0H);
		sh0h = 0;
		/* Reversed bitmask: see XHFC manual par 6.7 */
		SET_V_SH0H(sh0h, sh0h_shape);
		write_xhfc(xhfc, R_PCM_MD0, xhfc->pcm_md0);
		write_xhfc(xhfc, R_SH0H, sh0h);
	}

	
	/* ST/Up synchronization source
	 *    0 = Auto synchronize on the first available TE port or on SYNC_I
	 *    1 = Synchronize always on port 1
	 *    2 = Synchronize always on port 2
	 *    3 = Synchronize always on port 3
	 *    4 = Synchronize always on port 4
	 *    5 = Synchronize always on SYNC_I
	 */
	
	
	xhfc->su_sync = 0x0;
	
	switch(sync_port) {
	case 0:
		if (debug & DEBUG_HFC_INIT) {
			printk(KERN_INFO "%s: Setting SU sync=AUTO\n",
				__FUNCTION__);
		}
		SET_V_MAN_SYNC(xhfc->su_sync, 0);
		if (xhfc->pcm_config & XHFC_PCM_SYNC_I_ENABLE)
			SET_V_AUTO_SYNCI(xhfc->su_sync, 1);
		break;
	case 1:
	case 2:
	case 3:
	case 4:
		if (debug & DEBUG_HFC_INIT) {
			printk(KERN_INFO "%s: Setting SU sync=PORT_%d\n",
				__FUNCTION__, sync_port);
		}
		SET_V_MAN_SYNC(xhfc->su_sync, 1);
		SET_V_SYNC_SEL(xhfc->su_sync, (sync_port - 1));
		break;
	case 5:
		if (debug & DEBUG_HFC_INIT) {
			printk(KERN_INFO "%s: Setting SU sync=SYNC_I\n",
				__FUNCTION__);
		}
		SET_V_MAN_SYNC(xhfc->su_sync, 1);
		SET_V_SYNC_SEL(xhfc->su_sync, 4);
		break;

	default:
		if (debug & DEBUG_HFC_INIT)
			printk(KERN_ERR "%s %s: Wrong ST/Up sync port"
				"value %d\n",
			       xhfc->name, __FUNCTION__, sync_port);
		return (-EINVAL);
	}

	write_xhfc(xhfc, R_SU_SYNC, xhfc->su_sync);
	
		
	return 0;
}

static void xhfc_pcm_ch_setup(xhfc_t * xhfc, __u8 channel)
{
	__u8     r_slot;
	__u8   a_sl_cfg;
	__u8 a_con_hdlc;

	r_slot = a_sl_cfg = a_con_hdlc = 0;

	/* Setup TX timeslot */
	SET_V_SL_DIR(r_slot, 0); /* TX */
	SET_V_SL_NUM(r_slot, xhfc->chan[channel].slot_tx);
	write_xhfc(xhfc, R_SLOT, r_slot);

	SET_V_CH_SDIR(a_sl_cfg, 0);
	SET_V_CH_SNUM(a_sl_cfg, channel);
	SET_V_ROUT(a_sl_cfg, 0x2); /* TX on STIO1 */
	write_xhfc(xhfc, A_SL_CFG, a_sl_cfg);

	/* Setup RX timeslot */
	SET_V_SL_DIR(r_slot, 1); /* RX */
	SET_V_SL_NUM(r_slot, xhfc->chan[channel].slot_rx);
	write_xhfc(xhfc, R_SLOT, r_slot);

	SET_V_CH_SDIR(a_sl_cfg, 1);
	SET_V_CH_SNUM(a_sl_cfg, channel);
	SET_V_ROUT(a_sl_cfg, 0x2); /* RX from STIO2 */
	write_xhfc(xhfc, A_SL_CFG, a_sl_cfg);

	/* SET Transparent mode */
	SET_V_HDLC_TRP(a_con_hdlc, 1);
	/* SET FIFO ON (mandatory) - IRQ off */
	SET_V_FIFO_IRQ(a_con_hdlc, 7);
	/* Connect ST/Up straight to PCM */
	SET_V_DATA_FLOW(a_con_hdlc, 0x6);
	/* B-TX Fifo (M_REV will invert bit processing msb first) */
	setup_fifo(xhfc, (channel << 1) | (fifo_rev ? M_REV : 0),
		a_con_hdlc, 0, 0, 1);
	/* B-RX Fifo */
	setup_fifo(xhfc, ((channel << 1) + 1) | (fifo_rev ? M_REV : 0),
		a_con_hdlc, 0, 0, 1);

	if (debug & DEBUG_HFC_MODE)
		mISDN_debugprint(&xhfc->chan[channel].ch.inst,
			"ISDN_PID_L1_B_64TRANS on TDM bus: slots: tx=%d, rx=%d",
			xhfc->chan[channel].slot_tx,
			xhfc->chan[channel].slot_rx);
}

static void xhfc_pcm_ts_setup(xhfc_t * xhfc, int b, int pt)
{
	int ch_idx = B_CH_IDX(pt, b);
	u16 ts_nr;
#ifdef I2C_LED
	u16 led_nr;
	u8 value;
#endif

	if (xhfc == NULL) {
		BUG();
		return;
	}

#ifndef ENABLE_128MS
	
        ts_nr = b + pt * 2 + XHFC_PCM_FIRST_TS_GET(xhfc->pcm_config);
#else
        //DPN:
        //Change the B channels assignment on the TDM bus so only even time slots are used.
        //This is done in relation with the 128ms tail length support 
        ts_nr = b * 2 + pt * 4 + XHFC_PCM_FIRST_TS_GET(xhfc->pcm_config);
#endif
	
	/* Check for channel availability on TDM */
	if (ts_nr >= 32 * (GET_V_PCM_DR(xhfc->pcm_md1) + 1)) {
		/* Stop channel config here */
		xhfc->chan[ch_idx].slot_rx =
		xhfc->chan[ch_idx].slot_tx = -1;
		printk("Error, timeslot not available on bus\n");
		return;
	}

	xhfc->chan[ch_idx].slot_rx =
	xhfc->chan[ch_idx].slot_tx = ts_nr;

#ifdef I2C_LED
	led_call_off(ts_nr);
#endif

	printk(KERN_DEBUG
	       "%s %s: Registering B-channel timeslot, card(%d) ch(%d) port(%d)"
	       " slot_rx(%d) slot_tx(%d)\n", xhfc->name, __FUNCTION__,
	       xhfc->chipnum, ch_idx, pt,
	       xhfc->chan[ch_idx].slot_rx, xhfc->chan[ch_idx].slot_tx);
}

/*
 * initialise the XHFC ISDN Chip
 * return 0 on success.
 */
static int
init_xhfc(xhfc_t * xhfc)
{
	int err = 0;
	int timeout = 0x2000;
	__u8 chip_id;

#if BRIDGE == BRIDGE_SIMPLE_MEMORY_MAPPED
	int timeout_pll = 10;
	/* system clock is xtal */

#define M1_PLL_M 0x40

	timeout = 0x20000;
	/* software reset to enable R_FIFO_MD setting */
	write_xhfc(xhfc, R_CIRM, M_SRES | M_SU_RES);
	udelay(5);
	write_xhfc(xhfc, R_CIRM, 0);

	write_xhfc(xhfc, R_CLK_CFG, /* M_PCM_CLK */ 0);
	write_xhfc(xhfc, R_PLL_CTRL, (M1_PLL_M * 3)); // x2, RESET

	/* confirm data bus is connected and reliable */

	{
		int n, t;

		for (n = 0; n < 256 ; n++) {
			write_xhfc(xhfc, R_PLL_P, n);
			t = read_xhfc(xhfc, R_PLL_P);
			if (t != n) {
				printk(KERN_ERR
				    "%s %s: initialization sequence "
				    "Data bus fail: 0x%02x -> 0x%02x\n",
				    xhfc->name, __FUNCTION__, n, t);
				return -ENODEV;
			}
		}
	}

#endif

	chip_id = read_xhfc(xhfc, R_CHIP_ID);
	switch (chip_id) {
		case CHIP_ID_1SU:
			xhfc->num_ports = 1;
			xhfc->max_fifo = 4;
			xhfc->max_z = 0xFF;
			SET_V_EV_TS(xhfc->ti_wd, 6); /* timer irq interval 16 ms */
			write_xhfc(xhfc, R_FIFO_MD, 2);
			SET_V_SU0_IRQMSK(xhfc->su_irqmsk, 1);
			sprintf(xhfc->name, "%s_PI%d_%i",
			        CHIP_NAME_1SU,
			        xhfc->pi->cardnum,
			        xhfc->chipidx);
			break;

		case CHIP_ID_2SU:
			xhfc->num_ports = 2;
			xhfc->max_fifo = 8;
			xhfc->max_z = 0x7F;
			SET_V_EV_TS(xhfc->ti_wd, 5);	/* timer irq interval 8 ms */
			write_xhfc(xhfc, R_FIFO_MD, 1);
			SET_V_SU0_IRQMSK(xhfc->su_irqmsk, 1);
			SET_V_SU1_IRQMSK(xhfc->su_irqmsk, 1);
			sprintf(xhfc->name, "%s_PI%d_%i",
			        CHIP_NAME_2SU,
			        xhfc->pi->cardnum,
			        xhfc->chipidx);
			break;

		case CHIP_ID_2S4U:
			xhfc->num_ports = 4;
			xhfc->max_fifo = 16;
			xhfc->max_z = 0x3F;
			SET_V_EV_TS(xhfc->ti_wd, 4);	/* timer irq interval 4 ms */
			write_xhfc(xhfc, R_FIFO_MD, 0);
			SET_V_SU0_IRQMSK(xhfc->su_irqmsk, 1);
			SET_V_SU1_IRQMSK(xhfc->su_irqmsk, 1);
			SET_V_SU2_IRQMSK(xhfc->su_irqmsk, 1);
			SET_V_SU3_IRQMSK(xhfc->su_irqmsk, 1);
			sprintf(xhfc->name, "%s_PI%d_%i",
			        CHIP_NAME_2S4U,
			        xhfc->pi->cardnum,
			        xhfc->chipidx);
		break;

		case CHIP_ID_4SU:
			xhfc->num_ports = 4;
			xhfc->max_fifo = 16;
			xhfc->max_z = 0x3F;
			SET_V_EV_TS(xhfc->ti_wd, 4);	/* timer irq interval 4 ms */
			write_xhfc(xhfc, R_FIFO_MD, 0);
			SET_V_SU0_IRQMSK(xhfc->su_irqmsk, 1);
			SET_V_SU1_IRQMSK(xhfc->su_irqmsk, 1);
			SET_V_SU2_IRQMSK(xhfc->su_irqmsk, 1);
			SET_V_SU3_IRQMSK(xhfc->su_irqmsk, 1);
			sprintf(xhfc->name, "%s_PI%d_%i",
			        CHIP_NAME_4SU,
			        xhfc->pi->cardnum,
			        xhfc->chipidx);
			break;
		default:
			err = -ENODEV;
	}

	if (err) {
		if (debug & DEBUG_HFC_INIT)
			printk(KERN_ERR "%s %s: unknown Chip ID 0x%x\n",
			       xhfc->name, __FUNCTION__, chip_id);
		return (err);
	} else {
		if (debug & DEBUG_HFC_INIT)
			printk(KERN_INFO "%s ChipID: 0x%x\n",
			       xhfc->name, chip_id);
	}

#if BRIDGE == BRIDGE_PCI2PI
	/* software reset to enable R_FIFO_MD setting */
	write_xhfc(xhfc, R_CIRM, M_SRES);
	udelay(5);
	write_xhfc(xhfc, R_CIRM, 0);

	/* amplitude */
	write_xhfc(xhfc, R_PWM_MD, 0x80);
	write_xhfc(xhfc, R_PWM1, 0x18);

#endif

#if BRIDGE == BRIDGE_SIMPLE_MEMORY_MAPPED

	/* set PLL ratios (for 7.68MHz -> 24.576MHz) */

	write_xhfc(xhfc, R_PLL_CTRL, (M1_PLL_M * 3)); // x4, RESET
	write_xhfc(xhfc, R_PLL_CTRL, (M1_PLL_M * 3) |
	    M_PLL_NRES); // x4, no reset

	write_xhfc(xhfc, R_PLL_P, 5);
	write_xhfc(xhfc, R_PLL_N, 16);
	write_xhfc(xhfc, R_PLL_S, 1);

	/* confirm PLL lock */

	while ((!read_xhfc(xhfc, R_PLL_STA) & M_PLL_LOCK) && timeout_pll) {
		timeout_pll--;
		mdelay(1);
	}

	if (!timeout_pll) {
		printk(KERN_ERR
			"%s %s: initialization sequence "
			"PLL did not lock after 10ms\n",
			xhfc->name, __FUNCTION__);
		return -ENODEV;
	} else {
		printk(KERN_INFO "%s %s: PLL locked\n",
			xhfc->name, __FUNCTION__);
	}

	/* accept PLL clock as system clock*/

	write_xhfc(xhfc, R_CLK_CFG, /* M_PCM_CLK | */ M_CLKO_PLL | M_CLK_PLL);


	/* amplitude */
//	write_xhfc(xhfc, R_PWM_MD, 0x80);
//	write_xhfc(xhfc, R_PWM1, 0x18);
#endif

	write_xhfc(xhfc, R_FIFO_THRES, 0x11);

	while ((read_xhfc(xhfc, R_STATUS) & (M_BUSY | M_PCM_INIT))
	       && (timeout))
		timeout--;

	if (!(timeout)) {
		if (debug & DEBUG_HFC_INIT)
			printk(KERN_ERR
			    "%s %s: initialization sequence "
			    "could not finish, status=0x%02X\n",
			    xhfc->name, __FUNCTION__,
			    read_xhfc(xhfc, R_STATUS));
		return (-ENODEV);
	}

#ifdef I2C_LED
        /* set GPIOs for LEDs */
        pca9539_set_byte(0x0, PCA9539_DIRECTION_0);
        pca9539_set_byte(0x0, PCA9539_DIRECTION_1);
        pca9539_set_byte(PCA9539_MASK_0, PCA9539_OUTPUT_0);
        pca9539_set_byte(PCA9539_MASK_1, PCA9539_OUTPUT_1);
#endif

	/* PCM master/slave configuration */
	if (xhfc->pcm > -1) {
		err = xhfc_pcm_init(xhfc);
		if (err)
			return err;

	} else {
		if (debug & DEBUG_HFC_INIT) {
			printk(KERN_INFO "%s: Set PCM default to master mode\n",
				__FUNCTION__);
		}
		/* set PCM master mode as default configuration */
		SET_V_PCM_MD(xhfc->pcm_md0, 1);
		write_xhfc(xhfc, R_PCM_MD0, xhfc->pcm_md0);

		/* set pll adjust */
		SET_V_PCM_IDX(xhfc->pcm_md0, IDX_PCM_MD1);
		SET_V_PLL_ADJ(xhfc->pcm_md1, 3);
		write_xhfc(xhfc, R_PCM_MD0, xhfc->pcm_md0);
		write_xhfc(xhfc, R_PCM_MD1, xhfc->pcm_md1);
	}


	// Enable the SPORT 
	// ZT_CHUNKSIZE = 8;
	bfsi_sport0_init(8, debug);

	// Give time for things to settle
	printk("About to perform IRQ test, delaying 1s\n");
	printk("N_SLOTS is now %x\n",N_SLOTS);
	mdelay(1000);

	/* perfom short irq test */
	xhfc->testirq = 1;
	enable_interrupts(xhfc);
	mdelay(1 << GET_V_EV_TS(xhfc->ti_wd));
	disable_interrupts(xhfc);

	if (xhfc->irq_cnt > 2) {
		/* Leave the board in testirq mode until configuration is finished */

	  printk("Passed intrrupt test\n");
	  return (0);
	} else {
		if (debug & DEBUG_HFC_INIT)
			printk(KERN_INFO
			    "%s %s: ERROR getting IRQ (irq_cnt %i)\n",
			    xhfc->name, __FUNCTION__, xhfc->irq_cnt);
		return (-EIO);
	}
}

/*
 * free memory for all used channels
 */
static void
release_channels(xhfc_t * xhfc)
{
	int i = 0;

	while (i < MAX_CHAN) {
		if (xhfc->chan[i].ch.Flags) {
			if (debug & DEBUG_HFC_INIT)
				printk(KERN_DEBUG "%s %s: free channel %d\n",
				    xhfc->name, __FUNCTION__, i);
			mISDN_freechannel(&xhfc->chan[i].ch);
			mISDN_ctrl(&xhfc->chan[i].ch.inst,
			    MGR_UNREGLAYER | REQUEST, NULL);
		}
		i++;
	}

	if (xhfc->chan)
		kfree(xhfc->chan);
	if (xhfc->port)
		kfree(xhfc->port);
}

/*
 * setup port (line interface) with SU_CRTLx
 */
static void
init_su(xhfc_t * xhfc, __u8 pt)
{
	xhfc_port_t *port = &xhfc->port[pt];

	if (debug & DEBUG_HFC_MODE)
		printk(KERN_INFO "%s %s port(%i)\n", xhfc->name,
		    __FUNCTION__, pt);

	write_xhfc(xhfc, R_SU_SEL, pt);

	if (port->mode & PORT_MODE_NT)
		SET_V_SU_MD(port->su_ctrl0, 1);

	if (port->mode & PORT_MODE_EXCH_POL)
		port->su_ctrl2 = M_SU_EXCHG;

	if (port->mode & PORT_MODE_UP) {
		SET_V_ST_SEL(port->st_ctrl3, 1);
		write_xhfc(xhfc, A_MS_TX, 0x0F);
		SET_V_ST_SQ_EN(port->su_ctrl0, 1);
	}

	/* configure end of pulse control for ST mode (TE & NT) */
	if (port->mode & PORT_MODE_S0) {
		SET_V_ST_PU_CTRL(port->su_ctrl0, 1);
		port->st_ctrl3 = 0xf8;
	}

	if (debug & DEBUG_HFC_MODE)
		printk(KERN_INFO "%s %s su_ctrl0(0x%02x) "
		       "su_ctrl1(0x%02x) "
		       "su_ctrl2(0x%02x) "
		       "st_ctrl3(0x%02x)\n",
		       xhfc->name, __FUNCTION__,
		       port->su_ctrl0,
		       port->su_ctrl1,
		       port->su_ctrl2,
		       port->st_ctrl3);
	write_xhfc(xhfc, A_ST_CTRL3, port->st_ctrl3);
	write_xhfc(xhfc, A_SU_CTRL0, port->su_ctrl0);
	write_xhfc(xhfc, A_SU_CTRL1, port->su_ctrl1);
	write_xhfc(xhfc, A_SU_CTRL2, port->su_ctrl2);

	if (port->mode & PORT_MODE_TE)
		write_xhfc(xhfc, A_SU_CLK_DLY, CLK_DLY_TE);
	else
		write_xhfc(xhfc, A_SU_CLK_DLY, CLK_DLY_NT);

	write_xhfc(xhfc, A_SU_WR_STA, 0);
}

/*
 * Setup Fifo using A_CON_HDLC, A_SUBCH_CFG, A_FIFO_CTRL
 */
static void
setup_fifo(xhfc_t * xhfc, __u8 fifo, __u8 conhdlc, __u8 subcfg,
    __u8 fifoctrl, __u8 enable)
{
	xhfc_selfifo(xhfc, fifo);
	write_xhfc(xhfc, A_CON_HDLC, conhdlc);
	write_xhfc(xhfc, A_SUBCH_CFG, subcfg);
	write_xhfc(xhfc, A_FIFO_CTRL, fifoctrl);

	if (enable) {
		/* Enable FIFO IRQ if mode is not ST/Up->PCM direct connect. */
		if ((conhdlc & M_DATA_FLOW) != 0xC0) {
			xhfc->fifo_irqmsk |= (1 << (fifo & ~M_REV));
		}
	} else {
		xhfc->fifo_irqmsk &= ~(1 << (fifo & ~M_REV));
	}

	xhfc_resetfifo(xhfc);
	xhfc_selfifo(xhfc, fifo);

#if BRIDGE == BRIDGE_PCI2PI
	if (debug & DEBUG_HFC_MODE) {
		printk(KERN_INFO
		    "%s %s: fifo(%i) conhdlc(0x%02x) "
		    "subcfg(0x%02x) fifoctrl(0x%02x)\n",
		    xhfc->name, __FUNCTION__, fifo,
		    sread_xhfc(xhfc, A_CON_HDLC),
		    sread_xhfc(xhfc, A_SUBCH_CFG),
		    sread_xhfc(xhfc,  A_FIFO_CTRL));
	}
#endif
}

/*
 * Setup S/U interface, enable/disable B-Channels
 */
static void
setup_su(xhfc_t * xhfc, __u8 pt, __u8 bc, __u8 enable)
{
	xhfc_port_t *port = &xhfc->port[pt];

	if (!((bc == 0) || (bc == 1))) {
		printk(KERN_INFO "%s %s: pt(%i) ERROR: bc(%i) unvalid!\n",
		    xhfc->name, __FUNCTION__, pt, bc);
		return;
	}

	if (debug & DEBUG_HFC_MODE)
		printk(KERN_INFO "%s %s %s pt(%i) bc(%i)\n",
		    xhfc->name, __FUNCTION__,
		    (enable) ? ("enable") : ("disable"), pt, bc);

	if (bc) {
		SET_V_B2_RX_EN(port->su_ctrl2, (enable?1:0));
		SET_V_B2_TX_EN(port->su_ctrl0, (enable?1:0));
	} else {
		SET_V_B1_RX_EN(port->su_ctrl2, (enable?1:0));
		SET_V_B1_TX_EN(port->su_ctrl0, (enable?1:0));
	}

	if (xhfc->port[pt].mode & PORT_MODE_NT)
		SET_V_SU_MD(xhfc->port[pt].su_ctrl0, 1);

	write_xhfc(xhfc, R_SU_SEL, pt);
	write_xhfc(xhfc, A_SU_CTRL0, xhfc->port[pt].su_ctrl0);
	write_xhfc(xhfc, A_SU_CTRL2, xhfc->port[pt].su_ctrl2);
}

/*
 * (dis-) connect D/B-Channel using protocol
 */
static int
setup_channel(xhfc_t * xhfc, __u8 channel, int protocol)
{
	xhfc_port_t *port = xhfc->chan[channel].port;

	if (test_bit(FLG_BCHANNEL, &xhfc->chan[channel].ch.Flags)) {
		if (debug & DEBUG_HFC_MODE)
			mISDN_debugprint(&xhfc->chan[channel].ch.inst,
			    "channel(%i) protocol %x-->%x",
			    channel, xhfc->chan[channel].ch.state,
			    protocol);

		switch (protocol) {
		case (-1):	/* used for init */
			xhfc->chan[channel].ch.state = -1;
			xhfc->chan[channel].ch.channel = channel;
			/* fall trough */
		case (ISDN_PID_NONE):
			if (debug & DEBUG_HFC_MODE)
				mISDN_debugprint(&xhfc->chan[channel].ch.inst,
				    "ISDN_PID_NONE");
			if (xhfc->chan[channel].ch.state == ISDN_PID_NONE)
				return (0);	/* already in idle state */
			xhfc->chan[channel].ch.state = ISDN_PID_NONE;

			setup_fifo(xhfc, (channel << 1),
				4, 0, 0, 0);	/* B-TX fifo */
			setup_fifo(xhfc, (channel << 1) + 1,
				4, 0, 0, 0);	/* B-RX fifo */

			setup_su(xhfc, port->idx,
				(channel % 4) ? 1 : 0, 0);

			clear_bit(FLG_HDLC,
				&xhfc->chan[channel].ch.Flags);
			clear_bit(FLG_TRANSPARENT,
				&xhfc->chan[channel].ch.Flags);

			break;

		case (ISDN_PID_L1_B_64TRANS):
			if (debug & DEBUG_HFC_MODE)
				mISDN_debugprint(&xhfc->chan[channel].ch.inst,
				    "ISDN_PID_L1_B_64TRANS");
				/* Timeslot setup */
				if (xhfc->pcm > -1) {
					xhfc_pcm_ch_setup(xhfc, channel);
				} else {
					setup_fifo(xhfc, (channel << 1), 6, 0,
						0, 1);	/* B-TX Fifo */
					setup_fifo(xhfc, (channel << 1) + 1, 6,
						0, 0, 1);/* B-RX Fifo */
				}

			setup_su(xhfc, port->idx, (channel % 4) ? 1 : 0, 1);

			xhfc->chan[channel].ch.state = ISDN_PID_L1_B_64TRANS;
			set_bit(FLG_TRANSPARENT,
			    &xhfc->chan[channel].ch.Flags);

			break;

		case (ISDN_PID_L1_B_64HDLC):
			if (debug & DEBUG_HFC_MODE)
				mISDN_debugprint(&xhfc->chan[channel].ch.inst,
				    "ISDN_PID_L1_B_64HDLC");
			setup_fifo(xhfc, (channel << 1),
			    4, 0, M_FR_ABO, 1);	// TX Fifo
			setup_fifo(xhfc, (channel << 1) + 1,
			    4, 0, M_FR_ABO | M_FIFO_IRQMSK, 1);	// RX Fifo

			setup_su(xhfc, port->idx, (channel % 4) ? 1 : 0, 1);

			xhfc->chan[channel].ch.state = ISDN_PID_L1_B_64HDLC;
			set_bit(FLG_HDLC,
			    &xhfc->chan[channel].ch.Flags);

			break;
		default:
			mISDN_debugprint(&xhfc->chan[channel].ch.inst,
			    "prot not known %x", protocol);
			return (-ENOPROTOOPT);
		}
		return (0);
	} else if (test_bit(FLG_DCHANNEL, &xhfc->chan[channel].ch.Flags)) {
		if (debug & DEBUG_HFC_MODE)
			mISDN_debugprint(&xhfc->chan[channel].ch.inst,
			    "channel(%i) protocol(%i)",
			    channel, protocol);

		setup_fifo(xhfc, (channel << 1),
		    5, 2, M_FR_ABO, 1);	/* D TX fifo */
		setup_fifo(xhfc, (channel << 1) + 1,
		    5, 2, M_FR_ABO | M_FIFO_IRQMSK, 1);	/* D RX fifo */

		return (0);
	}

	printk(KERN_INFO
	    "%s %s ERROR: channel(%i) is NEITHER B nor D !!!\n",
	    xhfc->name, __FUNCTION__, channel);

	return (-1);
}

/*
 * register ISDN stack for one XHFC card
 *   - register all ports and channels
 *   - set param_idx
 *
 *  channel mapping in mISDN in xhfc->chan[MAX_CHAN]:
 *    1st line interf:  0=B1,  1=B2,  2=D,  3=PCM
 *    2nd line interf:  4=B1,  5=B2,  6=D,  7=PCM
 *    3rd line interf:  8=B1,  9=B2, 10=D, 11=PCM
 *    4th line interf; 12=B1, 13=B2, 14=D, 15=PCM
 */
static int
init_mISDN_channels(xhfc_t * xhfc)
{
	int err;
	int pt;		/* ST/U port index */
	int ch_idx;	/* channel index */
	int b;
	channel_t *ch;
	mISDN_pid_t pid;
	u_long flags;

	for (pt = 0; pt < xhfc->num_ports; pt++) {
		/* init D channels */
		ch_idx = D_CH_IDX(pt);
		if (debug & DEBUG_HFC_INIT)
			printk(KERN_INFO
			    "%s %s: Registering D-channel, card(%d) "
			    "ch(%d) port(%d) protocol(%x)\n",
			    xhfc->name, __FUNCTION__, xhfc->chipnum,
			    ch_idx, pt, xhfc->port[pt].dpid);

		xhfc->port[pt].idx = pt;
		xhfc->port[pt].xhfc = xhfc;
		xhfc->chan[ch_idx].port = &xhfc->port[pt];
		ch = &xhfc->chan[ch_idx].ch;

		memset(ch, 0, sizeof(channel_t));
		ch->channel = ch_idx;
		ch->debug = debug;
		ch->inst.obj = &hw_mISDNObj;
		ch->inst.hwlock = &xhfc->lock;
		ch->inst.class_dev.dev =
#if BRIDGE == BRIDGE_PCI2PI
		    &xhfc->pi->pdev->dev;
#else
		    NULL;
#endif
		mISDN_init_instance(&ch->inst, &hw_mISDNObj, xhfc, xhfc_l2l1);
		ch->inst.pid.layermask = ISDN_LAYER(0);
		sprintf(ch->inst.name, "%s_%d_D", xhfc->name, pt);
		err = mISDN_initchannel(ch, MSK_INIT_DCHANNEL,
		    MAX_DFRAME_LEN_L1);
		if (err)
			goto free_channels;
		ch->hw = xhfc;

		/* init t3 timer */
		init_timer(&xhfc->port[pt].t3_timer);
		xhfc->port[pt].t3_timer.data = (long) &xhfc->port[pt];
		xhfc->port[pt].t3_timer.function = (void *) l1_timer_expire_t3;

		/* init t4 timer */
		init_timer(&xhfc->port[pt].t4_timer);
		xhfc->port[pt].t4_timer.data = (long) &xhfc->port[pt];
		xhfc->port[pt].t4_timer.function = (void *) l1_timer_expire_t4;

		/* init B channels */
		for (b = 0; b < 2; b++) {
			ch_idx = B_CH_IDX(pt, b);
			if (debug & DEBUG_HFC_INIT)
				printk(KERN_INFO
				    "%s %s: Registering B-channel, card(%d) "
				    "ch(%d) port(%d)\n", xhfc->name,
				    __FUNCTION__, xhfc->chipnum, ch_idx, pt);

			xhfc->chan[ch_idx].port = &xhfc->port[pt];

			/*
			 * PCM timeslot setup if TDM bus is supported:
			 * ts_nr = xhfc_ch % 2 + (port - 1) + pcm_config & 0xFF
			 */
			if (xhfc->pcm > -1) {
				xhfc_pcm_ts_setup(xhfc, b, pt);
			}
			ch = &xhfc->chan[ch_idx].ch;

			memset(ch, 0, sizeof(channel_t));
			ch->channel = ch_idx;
			ch->debug = debug;
			mISDN_init_instance(&ch->inst, &hw_mISDNObj, xhfc,
			    xhfc_l2l1);
			ch->inst.pid.layermask = ISDN_LAYER(0);
			ch->inst.hwlock = &xhfc->lock;
			ch->inst.class_dev.dev =
#if BRIDGE == BRIDGE_PCI2PI
			    &xhfc->pi->pdev->dev;
#else
			    NULL;
#endif

			sprintf(ch->inst.name, "%s_%d_B%d",
				xhfc->name, pt, b + 1);

			if (mISDN_initchannel(ch, MSK_INIT_BCHANNEL,
			    MAX_DATA_MEM)) {
				err = -ENOMEM;
				goto free_channels;
			}
			ch->hw = xhfc;
		}

		/* clear PCM */
		memset(&xhfc->chan[PCM_CH_IDX(pt)], 0, sizeof(channel_t));

		mISDN_set_dchannel_pid(&pid, xhfc->port[pt].dpid,
		    layermask[xhfc->param_idx + pt]);

		/* register D Channel */
		ch = &xhfc->chan[D_CH_IDX(pt)].ch;

		/* set protocol for NT/TE */
		if (xhfc->port[pt].mode & PORT_MODE_NT) {
			/* NT-mode */
			xhfc->port[xhfc->param_idx + pt].mode |= NT_TIMER;
			xhfc->port[xhfc->param_idx + pt].nt_timer = 0;

			ch->inst.pid.protocol[0] = ISDN_PID_L0_NT_S0;
			ch->inst.pid.protocol[1] = ISDN_PID_L1_NT_S0;
			pid.protocol[0] = ISDN_PID_L0_NT_S0;
			pid.protocol[1] = ISDN_PID_L1_NT_S0;
			ch->inst.pid.layermask |= ISDN_LAYER(1);
			pid.layermask |= ISDN_LAYER(1);
			if (layermask[xhfc->param_idx + pt] & ISDN_LAYER(2))
				pid.protocol[2] = ISDN_PID_L2_LAPD_NET;
		} else {
			/* TE-mode */
			xhfc->port[xhfc->param_idx + pt].mode |= PORT_MODE_TE;
			ch->inst.pid.protocol[0] = ISDN_PID_L0_TE_S0;
			ch->inst.pid.protocol[1] = ISDN_PID_L1_TE_S0;
			pid.protocol[0] = ISDN_PID_L0_TE_S0;
			pid.protocol[1] = ISDN_PID_L1_TE_S0;
		}

		if (debug & DEBUG_HFC_INIT)
			printk(KERN_INFO
			    "%s %s: registering Stack for Port %i\n",
			    xhfc->name, __FUNCTION__, pt);

		/* register stack */
		err = mISDN_ctrl(NULL, MGR_NEWSTACK | REQUEST, &ch->inst);
		if (err) {
			printk(KERN_ERR
			    "%s %s: MGR_NEWSTACK | REQUEST  err(%d)\n",
			    xhfc->name, __FUNCTION__, err);
			goto free_channels;
		}
		ch->state = 0;

		/* attach two BChannels to this DChannel (ch) */
		for (b = 0; b < 2; b++) {
			err = mISDN_ctrl(ch->inst.st, MGR_NEWSTACK | REQUEST,
				&xhfc->chan[B_CH_IDX(pt,b)].ch.inst);
			if (err) {
				printk(KERN_ERR
				    "%s %s: MGR_ADDSTACK bchan error %d\n",
				    xhfc->name, __FUNCTION__, err);
				goto free_stack;
			}
		}

		err = mISDN_ctrl(ch->inst.st, MGR_SETSTACK | REQUEST, &pid);

		if (err) {
			printk(KERN_ERR
			    "%s %s: MGR_SETSTACK REQUEST dch err(%d)\n",
			    xhfc->name, __FUNCTION__, err);
			mISDN_ctrl(ch->inst.st, MGR_DELSTACK | REQUEST, NULL);
			goto free_stack;
		}

		/* initial setup of each channel */
		spin_lock_irqsave(&xhfc->lock, flags);
		setup_channel(xhfc, ch->channel, -1);
		for (b = 0; b < 2; b++)
			setup_channel(xhfc, B_CH_IDX(pt,b), -1);
		spin_unlock_irqrestore(&xhfc->lock, flags);

		jiffie_delay_unint((100 * HZ) / 1000 /* 100ms */);

		mISDN_ctrl(ch->inst.st, MGR_CTRLREADY | INDICATION, NULL);
	}
	return (0);

free_stack:
	mISDN_ctrl(ch->inst.st, MGR_DELSTACK | REQUEST, NULL);
free_channels:
	spin_lock_irqsave(&hw_mISDNObj.lock, flags);
	release_channels(xhfc);
	list_del(&xhfc->list);
	spin_unlock_irqrestore(&hw_mISDNObj.lock, flags);

	return (err);
}

/*
 * parse module paramaters like
 * NE/TE and S0/Up port mode
 */
static void
parse_module_params(xhfc_t * xhfc)
{
	__u8 pt;
#if defined(CONFIG_XHFC_PORT_MODE_AUTOSENSE)
	__u8 is_NT;
	__u8 gpio_in = read_xhfc(xhfc, R_GPIO_IN0);
#endif

	/* parse module parameters */
	for (pt = 0; pt < xhfc->num_ports; pt++) {
		/* D-Channel protocol: (2=DSS1) */
		xhfc->port[pt].dpid = (protocol[xhfc->param_idx + pt] & 0x0F);
		if (xhfc->port[pt].dpid == 0) {
			printk(KERN_INFO
			    "%s %s: WARNING: wrong value for protocol[%i], "
			    "assuming 0x02 (DSS1)...\n",
			    xhfc->name, __FUNCTION__,
			    xhfc->param_idx + pt);
			xhfc->port[pt].dpid = 0x02;
		}

		/* tunnel PTP mode */
		if (protocol[xhfc->param_idx + pt] & 0x0400)
			xhfc->port[pt].dpid |= 0x20;

		/* Line Interface TE or NT */
#if defined(CONFIG_XHFC_PORT_MODE_AUTOSENSE)
		switch (pt) {
		case 0:
			is_NT = GET_V_GPIO_IN0(gpio_in);
			break;
		case 1:
			is_NT = GET_V_GPIO_IN1(gpio_in);
			break;
		case 2:
			is_NT = GET_V_GPIO_IN7(gpio_in);
			break;
		case 3:
			is_NT = GET_V_GPIO_IN3(gpio_in);
			break;
		default:
			BUG();
		}

		if (is_NT) {
			xhfc->port[pt].mode |= PORT_MODE_NT;
			if (!(protocol[xhfc->param_idx + pt] & 0x10)) {
				printk(KERN_WARNING "%s %s: WARNING: port %d "
				       "mode configuration mismatch: protocol "
				       "set to TE, hw set to NT\n",
				       xhfc->name, __FUNCTION__, pt);
			}
		} else {
			xhfc->port[pt].mode |= PORT_MODE_TE;
			if (protocol[xhfc->param_idx + pt] & 0x10) {
				printk(KERN_WARNING "%s %s: WARNING: port %d "
				       "mode configuration mismatch: protocol "
				       "set to NT, hw set to TE\n",
				       xhfc->name, __FUNCTION__, pt);
			}
		}
#else
		if (protocol[xhfc->param_idx + pt] & 0x10)
			xhfc->port[pt].mode |= PORT_MODE_NT;
		else
			xhfc->port[pt].mode |= PORT_MODE_TE;
#endif

		/* Line Interface in S0 or Up mode */
		if (protocol[xhfc->param_idx + pt] & 0x20)
			xhfc->port[pt].mode |= PORT_MODE_UP;
		else
			xhfc->port[pt].mode |= PORT_MODE_S0;

		/* st line polarity */
		if (protocol[xhfc->param_idx + pt] & 0x40)
			xhfc->port[pt].mode |= PORT_MODE_EXCH_POL;

		/* get layer1 loop-config */
		if (protocol[xhfc->param_idx + pt] & 0x80)
			xhfc->port[pt].mode |= PORT_MODE_LOOP_B1;

		if (protocol[xhfc->param_idx + pt] & 0x0100)
			xhfc->port[pt].mode |= PORT_MODE_LOOP_B2;

		if (protocol[xhfc->param_idx + pt] & 0x0200)
			xhfc->port[pt].mode |= PORT_MODE_LOOP_D;

		if (debug & DEBUG_HFC_INIT)
			printk(
			    "%s %s: protocol[%i]=0x%02x, dpid=%d, "
			    "mode:%s,%s %s, Loops:0x%x\n",
			    xhfc->name, __FUNCTION__, xhfc->param_idx+pt,
			    protocol[xhfc->param_idx + pt],
			    xhfc->port[pt].dpid,
			    (xhfc->port[pt].mode & PORT_MODE_TE)?"TE":"NT",
			    (xhfc->port[pt].mode & PORT_MODE_S0)?"S0":"Up",
			    (xhfc->port[pt].mode & PORT_MODE_EXCH_POL) ?
			    "SU_EXCH":"",
			    (xhfc->port[pt].mode & PORT_MODE_LOOPS));
	}
}

/*
 * initialise the XHFC hardware
 * return 0 on success.
 */
static int __devinit
setup_instance(xhfc_t * xhfc)
{
	int err;
	int pt;
	xhfc_t *previous_hw;
	xhfc_port_t * port = NULL;
	xhfc_chan_t * chan = NULL;
	u_long flags;


	spin_lock_init(&xhfc->lock);
	tasklet_init(&xhfc->tasklet, xhfc_bh_handler, (unsigned long) xhfc);

	/* search previous instances to index protocol[] array */
	list_for_each_entry(previous_hw, &hw_mISDNObj.ilist, list) {
		xhfc->param_idx += previous_hw->num_ports;
	}

	spin_lock_irqsave(&hw_mISDNObj.lock, flags);
	/* add this instance to hardware list */
	list_add_tail(&xhfc->list, &hw_mISDNObj.ilist);
	spin_unlock_irqrestore(&hw_mISDNObj.lock, flags);

	err = init_xhfc(xhfc);
	if (err)
		goto out;

	/* alloc mem for all ports and channels */
	err = -ENOMEM;
	port = kzalloc(sizeof(xhfc_port_t) * xhfc->num_ports, GFP_KERNEL);
	if (port) {
		xhfc->port = port;
		chan = kzalloc(sizeof(xhfc_chan_t) * xhfc->num_ports *
		    CHAN_PER_PORT, GFP_KERNEL);
		if (chan) {
			xhfc->chan = chan;
			err = 0;
		} else {
			printk(KERN_ERR "%s %s: No kmem for xhfc_chan_t*%i \n",
			    xhfc->name, __FUNCTION__, xhfc->num_ports *
			    CHAN_PER_PORT);
			goto out;
		}
	} else {
		printk(KERN_ERR "%s %s: No kmem for xhfc_port_t*%i \n",
		       xhfc->name, __FUNCTION__, xhfc->num_ports);
		goto out;
	}

#if defined(CONFIG_XHFC_PORT_MODE_AUTOSENSE)

	/* Setup GPIO input to read ISDN NT/TE port mode from HW jumpers */
	write_xhfc(xhfc, R_GPIO_SEL, M_GPIO_SEL0 | M_GPIO_SEL1 | M_GPIO_SEL3 |
		   M_GPIO_SEL7);

	printk(KERN_INFO "%s %s: Enabled port mode NT/TE autosensing\n",
	       xhfc->name, __FUNCTION__);
#endif

	parse_module_params(xhfc);

	/* init line interfaces (ports) */
	for (pt = 0; pt < xhfc->num_ports; pt++) {
		sprintf(xhfc->port[pt].name, "%s_%i", xhfc->name, pt);
		init_su(xhfc, pt);
	}

	/* register all channels at ISDN procol stack */
	err = init_mISDN_channels(xhfc);
	if (err)
		goto out;

	jiffie_delay_unint((100 * HZ) / 1000 /* 100ms */);

	enable_interrupts(xhfc);

#if defined(XHFC_BFSI_IRQ_PROCESSING)
	enable_dma(xhfc);
#endif

	/* force initial layer1 statechanges */
	xhfc->su_irq = xhfc->su_irqmsk;

	/* init loops if desired */
	for (pt = 0; pt < xhfc->num_ports; pt++) {
		if (xhfc->port[pt].mode & PORT_MODE_LOOP_B1)
			xhfc_ph_command(&xhfc->port[pt], HFC_L1_TESTLOOP_B1);
		if (xhfc->port[pt].mode & PORT_MODE_LOOP_B2)
			xhfc_ph_command(&xhfc->port[pt], HFC_L1_TESTLOOP_B2);
		if (xhfc->port[pt].mode & PORT_MODE_LOOP_D)
			xhfc_ph_command(&xhfc->port[pt], HFC_L1_TESTLOOP_D);
	}

	return (0);

out:
	if (xhfc->chan)
		kfree(xhfc->chan);
	if (xhfc->port)
		kfree(xhfc->port);
	return (err);
}

/*
 * release single card
 */
static void
release_card(xhfc_pi * pi)
{
	u_long	flags;
	__u8 i;

	for (i = 0; i < pi->driver_data.num_xhfcs; i++) {
#if defined(XHFC_BFSI_IRQ_PROCESSING)
		disable_dma(&pi->xhfc[i]);
#endif
		disable_interrupts(&pi->xhfc[i]);
	}

#ifdef XHFC_BFSI_IRQ_PROCESSING
	printk("%s: freeing BFSI soft IRQ\n", __FUNCTION__);
	bfsi_soft_irq_free(pi->irq);
#else
	free_irq(pi->irq, pi);
#endif

	/* wait for pending tasklet to finish */
	jiffie_delay_unint((100 * HZ) / 1000 /* 100ms */);

	spin_lock_irqsave(&hw_mISDNObj.lock, flags);
	for (i = 0; i < pi->driver_data.num_xhfcs; i++) {
		release_channels(&pi->xhfc[i]);
		list_del(&pi->xhfc[i].list);
	}
	spin_unlock_irqrestore(&hw_mISDNObj.lock, flags);

#if BRIDGE == BRIDGE_SIMPLE_MEMORY_MAPPED

	printk(KERN_INFO "release_card()\n");

	pi->pdwPioBaseA[0x34>>2] =
	    0x10000000; // set intercept low (relays off)

	if (PCI2PI_XHFC_OFFSETS[0])
		iounmap((unsigned long *)PCI2PI_XHFC_OFFSETS[0]);

	PCI2PI_XHFC_OFFSETS[0] = 0;

	if (pi) {

		if (pi->pdwPioBaseA)
			iounmap((unsigned long *)pi->pdwPioBaseA);

		if (pi->pdwPioBaseB)
			iounmap((unsigned long *)pi->pdwPioBaseB);

	}
#endif
	kfree(pi->xhfc);
	kfree(pi);
}

/*
 * Issue an hardware reset to all XHFC hardware
 * (implementation is strictly platform dependent)
 */
static void xhfc_reset_hw (int delay_us) {

#ifdef XHFC_BFSI_IRQ_PROCESSING
	bfsi_hfc_reset(delay_us);
#endif
}


#if BRIDGE == BRIDGE_PCI2PI

/*
 * PCI hotplug interface: probe new card
 */
static int __devinit
xhfc_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	pi_params * driver_data = (pi_params *) ent->driver_data;
	xhfc_pi * pi = NULL;
	__u8 i;

	int err = -ENOMEM;

	/* alloc mem for ProcessorInterface xhfc_pi */
	if (!(pi = kzalloc(sizeof(xhfc_pi), GFP_KERNEL))) {
		printk(KERN_ERR "%s: No kmem for XHFC card\n",
		       __FUNCTION__);
		goto out;
	}

	pi->cardnum = card_cnt;

	sprintf(pi->name, "%s_PI%d", DRIVER_NAME, pi->cardnum);
	printk(KERN_INFO "%s %s: adapter '%s' found on PCI bus "
	    "%02x dev %02x, using %i XHFC controllers\n",
	    pi->name, __FUNCTION__, driver_data->device_name,
	    pdev->bus->number, pdev->devfn, driver_data->num_xhfcs);

	/* alloc mem for all XHFCs (xhfc_t) */
	if (!(pi->xhfc = kzalloc(sizeof(xhfc_t) * driver_data->num_xhfcs,
	    GFP_KERNEL))) {
		printk(KERN_ERR "%s %s: No kmem for sizeof(xhfc_t)*%i \n",
		    pi->name, __FUNCTION__, driver_data->num_xhfcs);
		goto out;
	}

	pi->pdev = pdev;
	err = pci_enable_device(pdev);
	if (err) {
		printk(KERN_ERR "%s %s: error with pci_enable_device\n",
		    pi->name, __FUNCTION__);
		goto out;
	}

	if (driver_data->num_xhfcs > PCI2PI_MAX_XHFC) {
		printk(KERN_ERR
		    "%s %s: max number og adressable XHFCs aceeded\n",
		    pi->name, __FUNCTION__);
		goto out;
	}

	pi->driver_data = *driver_data;
	pi->irq = pdev->irq;
	pi->hw_membase = (u_char *) pci_resource_start(pdev, 1);
	pi->membase = ioremap((ulong) pi->hw_membase, 4096);
	pci_set_drvdata(pdev, pi);

	err = init_pci_bridge(pi);
	if (err) {
		printk(KERN_ERR "%s %s: init_pci_bridge failed!\n",
		    pi->name, __FUNCTION__);
		goto out;
	}

	/* init interrupt engine */
	if (request_irq(pi->irq, xhfc_interrupt, __IRQF_SHARED, "XHFC", pi)) {
		printk(KERN_WARNING "%s %s: couldn't get interrupt %d\n",
		    pi->name, __FUNCTION__, pi->irq);
		pi->irq = 0;
		err = -EIO;
		goto out;
	}

	err = 0;
	for (i = 0; i < pi->driver_data.num_xhfcs; i++) {
		pi->xhfc[i].pi = pi;
		pi->xhfc[i].chipidx = i;
		err |= setup_instance(&pi->xhfc[i]);
	}

	if (!err) {
		card_cnt++;
		return (0);
	} else {
		goto out;
	}

out:
	if (pi->xhfc)
		kfree(pi->xhfc);
	if (pi)
		kfree(pi);
	return (err);
};

/*
 * PCI hotplug interface: remove card
 */
static void __devexit
xhfc_pci_remove(struct pci_dev *pdev)
{
	xhfc_pi *pi = pci_get_drvdata(pdev);
	printk(KERN_INFO "%s %s: removing card\n", pi->name,
	    __FUNCTION__);
	release_card(pi);
	card_cnt--;
	pci_disable_device(pdev);
	return;
};


static struct pci_device_id xhfc_ids[] = {
	{
		.vendor = PCI_VENDOR_ID_CCD,
		.device = 0xA003,
		.subvendor = 0x1397,
		.subdevice = 0xA003,
		.driver_data = (unsigned long) &((pi_params) {
		    1, "XHFC Evaluation Board"}),
	},
	{}
};

/*
 * Module init
 */
static struct pci_driver xhfc_driver = {
	name:DRIVER_NAME,
	probe:xhfc_pci_probe,
	remove:__devexit_p(xhfc_pci_remove),
	id_table:xhfc_ids,
};


MODULE_DEVICE_TABLE(pci, xhfc_ids);

#endif // BRIDGE_PCI2PI

#if BRIDGE == BRIDGE_SIMPLE_MEMORY_MAPPED

/*
 * no PCI on device -- simple memory mapped
 */

int SimpleMemoryMappedConfigurePlatform(void)
{
	xhfc_pi * pi = NULL;
	__u8 i;
	int err = -ENOMEM;

	ulong * pdwPioBaseStaticMemoryController =
	    ioremap_nocache(0xffffff00, 0x100);


	/* alloc mem for ProcessorInterface xhfc_pi */
	if (!(pi = kmalloc(sizeof(xhfc_pi), GFP_KERNEL))) {
		printk(KERN_ERR "%s %s: No kmem for XHFC card\n",
		    pi->name, __FUNCTION__);
		goto out;
	}
	memset(pi, 0, sizeof(xhfc_pi));

	pi->cardnum = card_cnt;
	pi->driver_data.num_xhfcs = 1;
	apiKnown[card_cnt] = pi;

	sprintf(pi->name, "%s_PI%d", DRIVER_NAME, pi->cardnum);
	printk(KERN_INFO "%s %s: found  "
	    "using %i XHFC controllers\n",
	    pi->name, __FUNCTION__,
	    pi->driver_data.num_xhfcs);

	/* alloc mem for all XHFCs (xhfc_t) */
	if (!(pi->xhfc = kmalloc(sizeof(xhfc_t) * pi->driver_data.num_xhfcs,
	    GFP_KERNEL))) {
		printk(KERN_ERR "%s %s: No kmem for sizeof(xhfc_t)*%i \n",
		    pi->name, __FUNCTION__, pi->driver_data.num_xhfcs);
		goto out;
	}
	memset(pi->xhfc, 0, sizeof(xhfc_t) * pi->driver_data.num_xhfcs);


	/* allocate struct holding platform instance */

	printk("Configuring: Platform ISDN-2 4ch\n");

	// CS4 needs to be quite slow for Echo Canceller

	pdwPioBaseStaticMemoryController[0x80>>2] =
		0x00004286; // set CS4 for Echo Canceller
	pdwPioBaseStaticMemoryController[0x84>>2] =
		0x7701428f; // set CS5 for ISDN chip
	iounmap((unsigned long *)pdwPioBaseStaticMemoryController);


	/*
	 * allocate memory mapped on to the isdn and echo canceller chips
	 */

//	if ((pxhfcplatform->m_instances[0].vmmioEchoCanceller =
//	    ioremap_nocache(0x50000000, 0x20000)) == NULL) { // CS4
//		printk(KERN_ERR "vpbox: Unable to ioremap_nocache\n");
//		goto bail2;
//	}

	if ((PCI2PI_XHFC_OFFSETS[0] = (__u32)ioremap_nocache(0x60000000,
	    0x20000)) == 0) { // CS5
		printk(KERN_ERR "vpbox: Unable to ioremap_nocache\n");
		goto bail0;
	}

	if (!(pi->pdwPioBaseB = ioremap_nocache(0xfffff600,0x200)))
		goto bail1;
	if (!(pi->pdwPioBaseA = ioremap_nocache(0xfffff400,0x200)))
		goto bail2;


	pAIC = (__u32 *)ioremap_nocache(0xfffff000, 0x200);
	if (!pAIC)
		goto bail3;

	// LEDs + PA28 = "intercept"

	pi->pdwPioBaseA[0x00>>2] =
	    0x1900007b; // enable PIO ctrl
	pi->pdwPioBaseA[0x10>>2] =
	    0x1900007b; // enable PIO output
	pi->pdwPioBaseA[0x30>>2] =
	    0x0900007b | 0x10000000; // set them high (leds off, relay on)
	pi->pdwPioBaseA[0x74>>2] = // Select Peripheral B control
	    0x00000002; // PA2 -> IRQ4

	mdelay(100); // relay

	/*
	 *  init the two SSC channels
	 */

/*
	pxhfcplatform->vmmioSSC0[0x00>>2] = 0x8000; // software reset
	pxhfcplatform->vmmioSSC1[0x00>>2] = 0x8000; // software reset
	pxhfcplatform->vmmioSSC0[0x00>>2] = 0x0000; // ...
	pxhfcplatform->vmmioSSC0[0x00>>2] = 0x0000; // ...
	pxhfcplatform->vmmioSSC0[0x00>>2] = 0x0000; // ...
	pxhfcplatform->vmmioSSC1[0x00>>2] = 0x0000; // software reset

		// SSC0

		// "DSP mode" rx clock and frame mode
	pfad->vmmioSSC0[0x10>>2] = 0x00000101;
	    // Rx clk mode: use TX input clock, high frame
	pfad->vmmioSSC0[0x14>>2] = 0x0000019f;
	    // Rx frame: 32-bit words, 2 of them per frame event
	pfad->vmmioSSC0[0x00>>2] = 0x2; // RX disable

		// "DSP Mode" tx clock and frame mode
	pfad->vmmioSSC0[0x18>>2] = 0x7f000322;
	    // Tx clk mode: input clock, high frame
	pfad->vmmioSSC0[0x1c>>2] = 0x0020019f;
	    // Tx frame: 32-bit words, 2 of them per frame event
	pfad->vmmioSSC0[0x00>>2] = 0x200;
	    // TX disable

		// SSC1

		// "DSP mode" rx clock and frame mode
	pfad->vmmioSSC1[0x10>>2] = 0x00000101;
	    // Rx clk mode: use TX input clock, high frame
	pfad->vmmioSSC1[0x14>>2] = 0x0000019f;
	    // Rx frame: 32-bit words, 8 of them per frame event
	pfad->vmmioSSC1[0x00>>2] = 0x2; // RX disable

		// "DSP Mode" tx clock and frame mode
	pfad->vmmioSSC1[0x18>>2] = 0x00000322;
	    // Tx clk mode: input clock, high frame
	pfad->vmmioSSC1[0x1c>>2] = 0x0000019f;
	    // Tx frame: 32-bit words, 8 of them per frame event
	pfad->vmmioSSC1[0x00>>2] = 0x200; // TX disable
*/

	// pb 0,5,6,11,28 are frame sigs

	pi->pdwPioBaseB[0x00>>2] = 0x24000400; // enable periph ctl
	pi->pdwPioBaseB[0x04>>2] = 0x18000bff; // disable PIO control
	pi->pdwPioBaseB[0x14>>2] = 0x3c000fff; // disable PIO output
	pi->pdwPioBaseB[0x70>>2] = 0x18000bff; // Peripheral A

	if (pi->driver_data.num_xhfcs > PCI2PI_MAX_XHFC) {
		printk(KERN_ERR
		    "%s %s: max number og adressable XHFCs aceeded\n",
		    pi->name, __FUNCTION__);
		goto out;
	}

//	pi->driver_data = *driver_data;
	pi->irq = 29; // IRQ4
//	pi->hw_membase = (u_char *) pci_resource_start(pdev, 1);
//	pi->membase = ioremap((ulong) pi->hw_membase, 4096);
//	pci_set_drvdata(pdev, pi);

//	err = init_pci_bridge(pi);
//	if (err) {
//		printk(KERN_ERR "%s %s: init_pci_bridge failed!\n",
//		    pi->name, __FUNCTION__);
//		goto out;
//	}

	/* init interrupt engine */

	if (request_irq(pi->irq, xhfc_interrupt, IRQF_TRIGGER_FALLING,
	    "XHFC", pi)) {
		printk(KERN_WARNING "%s %s: couldn't get interrupt %d\n",
		    pi->name, __FUNCTION__, pi->irq);
		pi->irq = 0;
		err = -EIO;
		goto out;
	}

	err = 0;
	for (i = 0; i < pi->driver_data.num_xhfcs; i++) {
		pi->xhfc[i].pi = pi;
		pi->xhfc[i].chipidx = i;
		err |= setup_instance(&pi->xhfc[i]);
	}

	if (!err) {
		card_cnt++;
		return (0);
	} else {
		goto out1;
	}

 out1:
	free_irq(pi->irq, pi);

 out:
	if (pi->xhfc)
		kfree(pi->xhfc);
	if (pi)
		kfree(pi);
	return (err);







	return 0;

bail3:
	iounmap((unsigned long *)pi->pdwPioBaseA);

bail2:
	iounmap((unsigned long *)pi->pdwPioBaseB);

bail1:
	iounmap((unsigned long *)PCI2PI_XHFC_OFFSETS[0]);

bail0:
	return -ENOMEM;
}




#endif // SIMPLE_MEMORY_MAPPED

#if BRIDGE == BRIDGE_SPI
/**
 * spi_xhfc_isdevice
 *
 * Checks if a device is present at a given bus address: if a recognized device
 * is present is returned 0 otherwise -1.
 *
 */
static int spi_xhfc_isdevice(xhfc_t *xhfc)
{
	int chip_id;

	chip_id = read_xhfc(xhfc, R_CHIP_ID);

	switch (chip_id) {
		case CHIP_ID_1SU:
		case CHIP_ID_2SU:
		case CHIP_ID_2S4U:
		case CHIP_ID_4SU:
			break;
		case 0:
			printk(KERN_DEBUG "%s: No device at SPI address %#x\n",
				 __FUNCTION__, xhfc->chipidx);
			return -1;
		default:
			printk(KERN_INFO "%s: Unrecognized chip ID %#x at SPI"
				" address %#x\n", __FUNCTION__, chip_id,
				xhfc->chipidx);
			return -1;
	}

	printk(KERN_DEBUG "%s: Identified XHFC device %#x at SPI address %#x\n",
		__FUNCTION__, chip_id,	xhfc->chipidx);

	return 0;
}

/**
 * spi_xhfc_count
 *
 * Count/config how many cascaded XHFC devices are attached to SPI bus.
 *
 */
static int spi_xhfc_count(xhfc_pi *pi)
{
	xhfc_t 	tmp_xhfc;
	int	i, chip_id;
	int	chip_count = 0;
	char *	chip_name = NULL;

	if (!pi)
		return -1;

	memset (&tmp_xhfc, 0x00, sizeof(tmp_xhfc));

	tmp_xhfc.pi = pi;

	for (i = 0; i < min(MAX_CARDS, SPI_MAX_XHFC); i++) {
		tmp_xhfc.chipidx = i;

		chip_id = read_xhfc(&tmp_xhfc, R_CHIP_ID);
		chip_name = NULL;

		switch (chip_id) {
		case CHIP_ID_1SU:
			chip_name = CHIP_NAME_1SU;
			break;
		case CHIP_ID_2SU:
			chip_name = CHIP_NAME_2SU;
			break;
		case CHIP_ID_2S4U:
			chip_name = CHIP_NAME_2S4U;
			break;
		case CHIP_ID_4SU:
			chip_name = CHIP_NAME_4SU;
			break;
		case 0x0:
			printk(KERN_DEBUG "%s: No XHFCs @ SPI address %d\n",
				 __FUNCTION__, i);
			break;
		default:
			printk(KERN_DEBUG "%s: Unrecognized chip ID %#x @ SPI "
				"address %d\n", __FUNCTION__, chip_id, i);
		}

		if (chip_name != NULL) {
			chip_count++;
			printk(KERN_INFO "%s: Found %s @ SPI address %d\n",
				 __FUNCTION__, chip_name, i);
		}
	}

	return chip_count;
} /* spi_xhfc_count () */


/**
 * xhfc_spi_probe -
 *
 *
 */
static int __devinit xhfc_spi_probe (void)
{
	int xnr, xnr_ok, xnr_found = 0;
	int err = -ENODEV;
	unsigned short have_master = 0;

	printk(/*KERN_DEBUG*/ "%s: entered\n", __FUNCTION__);

	/* alloc mem for Processor Interface xhfc_pi */
	pi = kzalloc(sizeof(xhfc_pi), GFP_KERNEL);
	if (pi == NULL) {
		printk(KERN_ERR "%s: No kmem for XHFC card\n", __FUNCTION__);
		err = -ENOMEM;
		goto err0;
	}

	pi->cardnum = card_cnt;

	/* SPI parameters */
	pi->spi_sel = spi_sel;
	pi->spi_speed = spi_speed;
	spin_lock_init(&pi->lock);

	if (xhfc_spi_init(pi)) {
		printk(KERN_ERR "%s: SPI registration failed\n", __FUNCTION__);
		err = -ENODEV;
		goto err1;
	}

	/* Reset XHFCs */
	xhfc_reset_hw(1000 /* us - reset delay */);

	pi->driver_data.num_xhfcs = 0;

	/* Count/config how many cascaded XHFC devices are attached to SPI bus */
	xnr_found = spi_xhfc_count(pi);

	printk(KERN_INFO "%s: %d device%s found\n", __FUNCTION__, xnr_found,
	       xnr_found == 1 ? "" : "s");

	if (xnr_found <= 0) {
		err = -ENODEV;
		goto err2;
	}

	/* Check if PCM parameters are provided properly */
	if (num_pcm_config < xnr_found) {
		printk(KERN_ERR "Incomplete PCM configuration: %d chip%s found"
		       " %d configuration%s provided. Check pcm_config!\n",
		       xnr_found, xnr_found > 1 ? "s" : "", num_pcm_config,
		       num_pcm_config > 1 ? "s" : "");
		err = -EPROTO;
		goto err2;
	}
	pi->driver_data.device_name = "XHFC:SPI";

#ifdef XHFC_BFSI_IRQ_PROCESSING
	pi->irq = bfsi_soft_irq_register(xhfc_soft_irq_process,
		xhfc_pcm_dma_process, pi, 0, 0);
	if (pi->irq < 0) {
		printk(KERN_ERR "%s %s: couldn't register to BFSI IRQ\n",
		       pi->name, __FUNCTION__);
		pi->irq = -1;
		err = -EIO;
		goto err2;
	}

	if (debug)
		printk(KERN_INFO "%s %s: Registered to BFSI IRQ %#x\n",
			pi->name, __FUNCTION__, pi->irq);
#endif
	err = 0;

	/* alloc mem for all XHFCs (xhfc_t) */
	if (!(pi->xhfc = kzalloc(sizeof(xhfc_t) * xnr_found, GFP_KERNEL))) {
		printk(KERN_ERR "%s: No kmem for sizeof(xhfc_t) * %d\n",
			__FUNCTION__, xnr_found);
		err = -ENOMEM;
		goto err3;
	}

	/* Setup pi ptr now to allow irq test to function properly */
	for (xnr = 0; xnr < xnr_found; xnr++)
		pi->xhfc[xnr].pi = pi;

	/* Search for installed XHFCs on SPI bus */
	for (xnr_ok = xnr = 0; (xnr_ok < xnr_found) && (xnr < SPI_MAX_XHFC);
	     xnr++) {
		pi->xhfc[xnr_ok].pi = pi;
		pi->xhfc[xnr_ok].chipidx = xnr;
		pi->xhfc[xnr_ok].chipnum = xnr_ok;
		pi->xhfc[xnr_ok].pcm = pcm;
		/* Force testirq mode on unconfigured cards to avoid BH call
		 * with unconfigured context, but allow for IRQ test to be
		 * performed on still unconfigured boards
		 */
		pi->xhfc[xnr_ok].testirq = 1;
		if (pi->xhfc[xnr_ok].pcm > -1) {
			/* Allow only one bus master */
			if (!(pcm_config[xnr_ok] & XHFC_PCM_SLAVE_MODE)) {
				if (have_master) {
					printk (KERN_ERR "Only one bus master"
						" allowed! Check pcm_config "
						"parameters\n");
					err = -EPROTO;
					goto err4;
				}
				have_master = 1;
			}
			pi->xhfc[xnr_ok].pcm_config = pcm_config[xnr_ok];
			printk(KERN_INFO "%s %s: xhfc[%d].pcm_config = %#x\n",
				pi->name, __FUNCTION__, xnr_ok,
				pi->xhfc[xnr_ok].pcm_config);
		}
		sprintf(pi->name, "%s_PI%d", DRIVER_NAME, pi->cardnum);

		if (spi_xhfc_isdevice(&pi->xhfc[xnr_ok]) == -1)
			continue;

		pi->driver_data.num_xhfcs++;

		err = setup_instance(&pi->xhfc[xnr_ok]);

		if (err) {
			printk(KERN_ERR "%s %s: Errors setting up instance %d\n",
			       pi->name, __FUNCTION__, xnr);
			err = -ENODEV;
			goto err4;
		}
		printk(KERN_INFO "%s %s: adapter(%d) '%s' found on SPI bus\n",
		       pi->name, __FUNCTION__, xnr_ok, pi->driver_data.device_name);

		/* Take out of testirq mode the board: now BH could be called safely */
		pi->xhfc[xnr_ok].testirq = 0;
		xnr_ok++;
	}

	if (xnr_ok == xnr_found) {
		card_cnt++;
		return (0);
	}

 err4:
	kfree(pi->xhfc);
 err3:
#if defined(XHFC_BFSI_IRQ_PROCESSING)
	bfsi_soft_irq_free(pi->irq);
#else
	free_irq(IRQ_PF0 + pi->irq_pfx, pi);
#endif

 err2:
	xhfc_spi_disable(pi->spi_sel);
 err1:
	kfree(pi);
 err0:
	return (err);
}

#endif /* BRIDGE_SPI */


/*
 * kernel module init
 */
static int __init
xhfc_init(void)
{
	int err;
	struct proc_dir_entry *reset;

	printk(KERN_INFO "XHFC: %s driver Rev. %s (debug=%i)\n",
	    __FUNCTION__, mISDN_getrev(xhfc_rev), debug);

#ifdef CONFIG_EC_ZL38065
	xhfc_ec_driver = symbol_get(zl38065_echocan);
	if (!xhfc_ec_driver) {
		printk(KERN_WARNING "%s: missing ZL38065 echo canceller module!\n",
		       __FUNCTION__);
	}
	
	//{//dpn
	//int ch;
	//for(ch=0;ch<32;ch++)
	//	xhfc_ec_driver(ch, 0);
	//}

#endif

#ifdef MODULE
	hw_mISDNObj.owner = THIS_MODULE;
#endif

	INIT_LIST_HEAD(&hw_mISDNObj.ilist);
	spin_lock_init(&hw_mISDNObj.lock);
	hw_mISDNObj.name = DRIVER_NAME;
	hw_mISDNObj.own_ctrl = xhfc_manager;

	hw_mISDNObj.DPROTO.protocol[0] =
	    ISDN_PID_L0_TE_S0 | ISDN_PID_L0_NT_S0;
	hw_mISDNObj.DPROTO.protocol[1] =
	    ISDN_PID_L1_TE_S0 | ISDN_PID_L1_NT_S0;
	hw_mISDNObj.BPROTO.protocol[1] =
	    ISDN_PID_L1_B_64TRANS | ISDN_PID_L1_B_64HDLC;
	hw_mISDNObj.BPROTO.protocol[2] =
	    ISDN_PID_L2_B_TRANS | ISDN_PID_L2_B_RAWDEV;
	card_cnt = 0;

	if ((err = mISDN_register(&hw_mISDNObj))) {
		printk(KERN_ERR "XHFC: can't register xhfc error(%d)\n", err);
		goto out1;
	}

#if BRIDGE == BRIDGE_PCI2PI
	err = pci_register_driver(&xhfc_driver);
	if (err < 0) {
		goto out2;
	}

#if !defined(CONFIG_HOTPLUG)
	if (err == 0) {
		err = -ENODEV;
		pci_unregister_driver(&xhfc_driver);
		goto out2;
	}
#endif
#endif

#if BRIDGE == BRIDGE_SIMPLE_MEMORY_MAPPED
	SimpleMemoryMappedConfigurePlatform();
#endif

#if BRIDGE == BRIDGE_SPI

	if ((err = xhfc_spi_probe()))
		goto out2;
#endif

	printk(KERN_INFO "XHFC: %d card%s installed\n", card_cnt,
		card_cnt == 1 ? "": "s");

	mISDN_module_register(THIS_MODULE);
	create_proc_read_entry("xhfc", 0, NULL, xhfc_proc_read, NULL);
	reset = create_proc_read_entry("xhfc_reset", 0, NULL, NULL, NULL);
	reset->write_proc = xhfc_proc_write;

	return 0;

 out2:
	mISDN_unregister(&hw_mISDNObj);
 out1:
	return (err);
}
static void __exit
xhfc_cleanup(void)
{
	int err;

	printk("xhfc_cleanup... %d\n", card_cnt);

	remove_proc_entry("xhfc", NULL);
	remove_proc_entry("xhfc_reset", NULL);

	mISDN_module_unregister(THIS_MODULE);

#if BRIDGE == BRIDGE_PCI2PI
	pci_unregister_driver(&xhfc_driver);
#endif

#if BRIDGE == BRIDGE_SIMPLE_MEMORY_MAPPED
	{
		int n = 0;
		while (n < card_cnt) {
			release_card(apiKnown[n]);
			n++;
		}
	}

	if (pAIC)
		iounmap(pAIC);

#endif

#if BRIDGE == BRIDGE_SPI
	release_card(pi);
	xhfc_spi_disable(pi->spi_sel);
#endif

	if ((err = mISDN_unregister(&hw_mISDNObj))) {
		printk(KERN_ERR "XHFC: can't unregister xhfc, error(%d)\n",
		    err);
	}

#ifdef CONFIG_EC_ZL38065
	if (xhfc_ec_driver)
		symbol_put(zl38065_echocan);
#endif

	printk(KERN_INFO "%s: driver removed\n", __FUNCTION__);
}

static int xhfc_proc_read(char *buf, char **start, off_t offset,
		    int count, int *eof, void *data)
{
	int len = 0;
	int i;
	int tslots = -1;
	volatile __u8 bert_sta;
#if BRIDGE == BRIDGE_SIMPLE_MEMORY_MAPPED
	for (i = 0; i < card_cnt; i++) {
		xhfc_pi * pi = apiKnown[i];
#else
	for (i = 0; i < pi->driver_data.num_xhfcs; i++) {
#endif


		len += sprintf(buf+len, "Card[%d]--------------\n", i + 1);
		bert_sta = read_xhfc(&pi->xhfc[i], R_BERT_STA);
		tslots = read_xhfc(&pi->xhfc[i], R_SL_MAX);
		len += sprintf(buf+len,
			       "ISDN Sync Source ....: %s%c\n"
			       "TDM timeslot number..: %d\n"
#ifdef XHFC_BFSI_DMA_DEBUG
			       "RX prep: %d cycles\n"
			       "TX prep: %d cycles\n"
#endif
			       ,
			       GET_V_RD_SYNC_SRC(bert_sta) == 4 ? "SYNC_I" : "PORT_",
			       GET_V_RD_SYNC_SRC(bert_sta) == 4 ? ' ' : '1' +
			       GET_V_RD_SYNC_SRC(bert_sta),
			       tslots + 1
#ifdef XHFC_BFSI_DMA_DEBUG
			       ,
			       (&pi->xhfc[i])->cycles_rxprep,
			       (&pi->xhfc[i])->cycles_txprep
#endif
			       );
	}

	len += sprintf(buf+len,
		       "Common ---------------\n"
		       "                       RX / TX\n"
		       "PCM DMA samples......: %d/%d\n"
		       "B-ch audio frames....: %d/%d\n"
		       "Average packet size..: %d/%d\n",
		       dma_rx_bytes, dma_tx_bytes,
		       misdn_bch_frames_received, misdn_bch_frames_sent,
		       dma_rx_bytes / misdn_bch_frames_received,
		       dma_tx_bytes / misdn_bch_frames_sent
		       );

	*eof=1;
	return len;
}

static int xhfc_proc_write(struct file *file, const char *buffer,
				   unsigned long count, void *data)
{
	dma_rx_bytes = 0;
	dma_tx_bytes = 0;
	misdn_bch_frames_received = 0;
	misdn_bch_frames_sent = 0;

	return count;
}


module_init(xhfc_init);
module_exit(xhfc_cleanup);
