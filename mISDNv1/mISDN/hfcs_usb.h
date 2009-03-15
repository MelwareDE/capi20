/*
 * hfcs_usb.h, HFC-S USB mISDN driver
 */

#ifndef __HFCS_USB_H__
#define __HFCS_USB_H__

#define DRIVER_AUTHOR   "Martin Bachem <m.bachem@gmx.de>"
#define DRIVER_DESC     "HFC-S USB based mISDN driver"

/* DEBUG flags, use combined value for module parameter debug=x */
#define DEBUG_HFC_INIT		0x0001
#define DEBUG_HFC_MODE		0x0002
#define DEBUG_HFC_S0_STATES	0x0004
#define DEBUG_HFC_IRQ		0x0008
#define DEBUG_HFC_FIFO_ERR	0x0010
#define DEBUG_HFC_DTRACE	0x2000
#define DEBUG_HFC_BTRACE	0x4000	/* very(!) heavy messageslog load */
#define DEBUG_HFC_FIFO		0x8000	/* very(!) heavy messageslog load */


/*
 * when ISO URB completes with -EXDEV, have a look
 * at the urb->start_frame history of a few URBs
 * if ISO_FRAME_START_DEBUG is enabled
 */
// #define ISO_FRAME_START_DEBUG
#ifdef ISO_FRAME_START_DEBUG
	#define ISO_FRAME_START_RING_COUNT 16
#endif

/*
 * debug all USB packets (INT-RX or ISO-RX)
 * containing D-Channel Data
 */
// #define FULL_DCHAN_DEBUG

#define HFC_CTRL_TIMEOUT	20	/* 5ms timeout writing/reading regs */
#define CLKDEL_TE		0x0f	/* CLKDEL in TE mode */
#define CLKDEL_NT		0x6c	/* CLKDEL in NT mode */

/* hfcsusb Layer1 commands */
#define HFC_L1_ACTIVATE_TE		0x01
#define HFC_L1_ACTIVATE_NT		0x02
#define HFC_L1_DEACTIVATE_NT		0x03
#define HFC_L1_FORCE_DEACTIVATE_TE	0x04

/* cmd FLAGS in HFCUSB_STATES register */
#define HFCUSB_LOAD_STATE	0x10
#define HFCUSB_ACTIVATE		0x20
#define HFCUSB_DO_ACTION	0x40
#define HFCUSB_NT_G2_G3		0x80

/* bits in hw_mode */
#define PORT_MODE_TE		0x01
#define PORT_MODE_NT		0x02
#define NT_ACTIVATION_TIMER	0x04	/* enables NT mode activation Timer */
#define NT_T1_COUNT		10

#define MAX_BCH_SIZE 		2048	/* allowed B-channel packet size */

#define HFCUSB_RX_THRESHOLD 	64	/* threshold for fifo report bit rx */
#define HFCUSB_TX_THRESHOLD 	64	/* threshold for fifo report bit tx */

#define HFCUSB_CHIP_ID		0x16	/* Chip ID register index */
#define HFCUSB_CIRM		0x00	/* cirm register index */
#define HFCUSB_USB_SIZE		0x07	/* int length register */
#define HFCUSB_USB_SIZE_I	0x06	/* iso length register */
#define HFCUSB_F_CROSS		0x0b	/* bit order register */
#define HFCUSB_CLKDEL		0x37	/* bit delay register */
#define HFCUSB_CON_HDLC		0xfa	/* channel connect register */
#define HFCUSB_HDLC_PAR		0xfb
#define HFCUSB_SCTRL		0x31	/* S-bus control register (tx) */
#define HFCUSB_SCTRL_E		0x32	/* same for E and special funcs */
#define HFCUSB_SCTRL_R		0x33	/* S-bus control register (rx) */
#define HFCUSB_F_THRES		0x0c	/* threshold register */
#define HFCUSB_FIFO		0x0f	/* fifo select register */
#define HFCUSB_F_USAGE		0x1a	/* fifo usage register */
#define HFCUSB_MST_MODE0	0x14
#define HFCUSB_MST_MODE1	0x15
#define HFCUSB_P_DATA		0x1f
#define HFCUSB_INC_RES_F	0x0e
#define HFCUSB_B1_SSL		0x20
#define HFCUSB_B2_SSL		0x21
#define HFCUSB_B1_RSL		0x24
#define HFCUSB_B2_RSL		0x25
#define HFCUSB_STATES		0x30


#define HFCUSB_CHIPID		0x40	/* ID value of HFC-S USB */

/* fifo registers */
#define HFCUSB_NUM_FIFOS	8	/* maximum number of fifos */
#define HFCUSB_B1_TX		0	/* index for B1 transmit bulk/int */
#define HFCUSB_B1_RX		1	/* index for B1 receive bulk/int */
#define HFCUSB_B2_TX		2
#define HFCUSB_B2_RX		3
#define HFCUSB_D_TX		4
#define HFCUSB_D_RX		5
#define HFCUSB_PCM_TX		6
#define HFCUSB_PCM_RX		7

/* Chan idx  */
#define B1	0
#define B2	1
#define D	2
#define PCM	3
#define MAX_CHAN 4

/*
 * used to switch snd_transfer_mode for different TA modes e.g. the Billion USB TA just
 * supports ISO out, while the Cologne Chip EVAL TA just supports BULK out
 */
#define USB_INT		0
#define USB_BULK	1
#define USB_ISOC	2

#define ISOC_PACKETS_D	8
#define ISOC_PACKETS_B	8
#define ISO_BUFFER_SIZE	128

/* defines how much ISO packets are handled in one URB */
static int iso_packets[8] =
    { ISOC_PACKETS_B, ISOC_PACKETS_B, ISOC_PACKETS_B, ISOC_PACKETS_B,
	ISOC_PACKETS_D, ISOC_PACKETS_D, ISOC_PACKETS_D, ISOC_PACKETS_D
};


/* Fifo flow Control for TX ISO */
#define SINK_MAX	68
#define SINK_MIN	48
#define SINK_DMIN	12
#define SINK_DMAX	18
#define BITLINE_INF	(-64*8)

/* HFC-S USB register access by Control-URSs */
#define write_usb(a,b,c)usb_control_msg((a)->dev,(a)->ctrl_out_pipe,0,0x40,(c),(b),0,0,HFC_CTRL_TIMEOUT)
#define read_usb(a,b,c) usb_control_msg((a)->dev,(a)->ctrl_in_pipe,1,0xC0,0,(b),(c),1,HFC_CTRL_TIMEOUT)
#define HFC_CTRL_BUFSIZE 32
typedef struct {
	__u8 hfcs_reg;		/* register number */
	__u8 reg_val;		/* value to be written (or read) */
} ctrl_buft;

/*
 * URB error codes
 * Used to represent a list of values and their respective symbolic names 
 */
struct hfcusb_symbolic_list {
	const int num;
	const char *name;
};

static struct hfcusb_symbolic_list urb_errlist[] = {
	{-ENOMEM, "No memory for allocation of internal structures"},
	{-ENOSPC, "The host controller's bandwidth is already consumed"},
	{-ENOENT, "URB was canceled by unlink_urb"},
	{-EXDEV, "ISO transfer only partially completed"},
	{-EAGAIN, "Too match scheduled for the future"},
	{-ENXIO, "URB already queued"},
	{-EFBIG, "Too much ISO frames requested"},
	{-ENOSR, "Buffer error (overrun)"},
	{-EPIPE, "Specified endpoint is stalled (device not responding)"},
	{-EOVERFLOW, "Babble (bad cable?)"},
	{-EPROTO, "Bit-stuff error (bad cable?)"},
	{-EILSEQ, "CRC/Timeout"},
	{-ETIMEDOUT, "NAK (device does not respond)"},
	{-ESHUTDOWN, "Device unplugged"},
	{-1, NULL}
};

static inline const char *
symbolic(struct hfcusb_symbolic_list list[], const int num)
{
	int i;
	for (i = 0; list[i].name != NULL; i++)
		if (list[i].num == num)
			return (list[i].name);
	return "<unkown USB Error>";
}

/* USB descriptor need to contain one of the following EndPoint combination: */
#define CNF_4INT3ISO	1	// 4 INT IN, 3 ISO OUT
#define CNF_3INT3ISO	2	// 3 INT IN, 3 ISO OUT
#define CNF_4ISO3ISO	3	// 4 ISO IN, 3 ISO OUT
#define CNF_3ISO3ISO	4	// 3 ISO IN, 3 ISO OUT

#define EP_NUL 1		// Endpoint at this position not allowed
#define EP_NOP 2		// all type of endpoints allowed at this position
#define EP_ISO 3		// Isochron endpoint mandatory at this position
#define EP_BLK 4		// Bulk endpoint mandatory at this position
#define EP_INT 5		// Interrupt endpoint mandatory at this position


/*
 * List of all supported enpoints configiration sets, used to find the
 * best matching endpoint configuration within a devices' USB descriptor.
 * We need at least 3 RX endpoints, and 3 TX endpoints, either
 * INT-in and ISO-out, or ISO-in and ISO-out)
 * with 4 RX endpoints even E-Channel logging is possible
 */
static int
validconf[][19] = {
	// INT in, ISO out config
	{EP_NUL, EP_INT, EP_NUL, EP_INT, EP_NUL, EP_INT, EP_NOP, EP_INT,
	 EP_ISO, EP_NUL, EP_ISO, EP_NUL, EP_ISO, EP_NUL, EP_NUL, EP_NUL,
	 CNF_4INT3ISO, 2, 1},
	{EP_NUL, EP_INT, EP_NUL, EP_INT, EP_NUL, EP_INT, EP_NUL, EP_NUL,
	 EP_ISO, EP_NUL, EP_ISO, EP_NUL, EP_ISO, EP_NUL, EP_NUL, EP_NUL,
	 CNF_3INT3ISO, 2, 0},
	// ISO in, ISO out config
	{EP_NUL, EP_NUL, EP_NUL, EP_NUL, EP_NUL, EP_NUL, EP_NUL, EP_NUL,
	 EP_ISO, EP_ISO, EP_ISO, EP_ISO, EP_ISO, EP_ISO, EP_NOP, EP_ISO,
	 CNF_4ISO3ISO, 2, 1},
	{EP_NUL, EP_NUL, EP_NUL, EP_NUL, EP_NUL, EP_NUL, EP_NUL, EP_NUL,
	 EP_ISO, EP_ISO, EP_ISO, EP_ISO, EP_ISO, EP_ISO, EP_NUL, EP_NUL,
	 CNF_3ISO3ISO, 2, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}	// EOL element
};

/* string description of chosen config */
char *conf_str[] = {
	"4 Interrupt IN + 3 Isochron OUT",
	"3 Interrupt IN + 3 Isochron OUT",
	"4 Isochron IN + 3 Isochron OUT",
	"3 Isochron IN + 3 Isochron OUT"
};


#define LED_OFF		0	// no LED support
#define LED_SCHEME1	1	// LED standard scheme
#define LED_SCHEME2	2	// not used yet...

#define LED_POWER_ON	1
#define LED_POWER_OFF	2
#define LED_S0_ON	3
#define LED_S0_OFF	4
#define LED_B1_ON	5
#define LED_B1_OFF	6
#define LED_B1_DATA	7
#define LED_B2_ON	8
#define LED_B2_OFF	9
#define LED_B2_DATA	10

#define LED_NORMAL	0	// LEDs are normal
#define LED_INVERTED 	1	// LEDs are inverted

/* time in ms to perform a Flashing LED when B-Channel has traffic */
#define LED_TIME      250

#endif	/* __HFCS_USB_H__ */
