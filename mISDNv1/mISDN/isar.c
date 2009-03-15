/* $Id: isar.c,v 1.22 2006/06/27 13:24:07 keil Exp $
 *
 * isar.c   ISAR (Siemens PSB 7110) specific routines
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 *
 * This file is (c) under GNU PUBLIC LICENSE
 *
 */

#include <linux/delay.h>
#include <linux/vmalloc.h>
#include "layer1.h"
#include "channel.h"
#include "isar.h"
#include "debug.h"

#define DBG_LOADFIRM	0
#define DUMP_MBOXFRAME	2

#define MIN(a,b) ((a<b)?a:b)

static char *ISAR_revision = "$Revision: 1.22 $";

const u_char faxmodulation_s[] = "3,24,48,72,73,74,96,97,98,121,122,145,146";
const u_char faxmodulation[] = {3,24,48,72,73,74,96,97,98,121,122,145,146};
#define FAXMODCNT 13

void isar_setup(channel_t *);
static void isar_pump_cmd(channel_t *, int, u_char);

static int firmwaresize = 0;
static u_char *firmware;
static u_char *fw_p;

static inline int
waitforHIA(channel_t *bch, int timeout)
{

	while ((bch->read_reg(bch->inst.privat, ISAR_HIA) & 1) && timeout) {
		udelay(1);
		timeout--;
	}
	if (!timeout)
		printk(KERN_WARNING "mISDN: ISAR waitforHIA timeout\n");
	return(timeout);
}


int
sendmsg(channel_t *bch, u_char his, u_char creg, u_char len,
	u_char *msg)
{
	int i;
	
	if (!waitforHIA(bch, 4000))
		return(0);
#if DUMP_MBOXFRAME
	if (bch->debug & L1_DEB_HSCX)
		mISDN_debugprint(&bch->inst, "sendmsg(%02x,%02x,%d)", his, creg, len);
#endif
	bch->write_reg(bch->inst.privat, ISAR_CTRL_H, creg);
	bch->write_reg(bch->inst.privat, ISAR_CTRL_L, len);
	bch->write_reg(bch->inst.privat, ISAR_WADR, 0);
	if (msg && len) {
		bch->write_fifo(bch->inst.privat, msg, len);
#if DUMP_MBOXFRAME>1
		if (bch->debug & L1_DEB_HSCX_FIFO) {
			char *t;
			
			i = len;
			while (i>0) {
				t = bch->log;
				t += sprintf(t, "sendmbox cnt %d", len);
				mISDN_QuickHex(t, &msg[len-i], (i>64) ? 64:i);
				mISDN_debugprint(&bch->inst, bch->log);
				i -= 64;
			}
		}
#endif
	}
	bch->write_reg(bch->inst.privat, ISAR_HIS, his);
	waitforHIA(bch, 10000);
	return(1);
}

/* Call only with IRQ disabled !!! */
inline void
rcv_mbox(channel_t *bch, isar_reg_t *ireg, u_char *msg)
{
	int i;

	bch->write_reg(bch->inst.privat, ISAR_RADR, 0);
	if (msg && ireg->clsb) {
		bch->read_fifo(bch->inst.privat, msg, ireg->clsb);
#if DUMP_MBOXFRAME>1
		if (bch->debug & L1_DEB_HSCX_FIFO) {
			char *t;
			
			i = ireg->clsb;
			while (i>0) {
				t = bch->log;
				t += sprintf(t, "rcv_mbox cnt %d", ireg->clsb);
				mISDN_QuickHex(t, &msg[ireg->clsb-i], (i>64) ? 64:i);
				mISDN_debugprint(&bch->inst, bch->log);
				i -= 64;
			}
		}
#endif
	}
	bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
}

/* Call only with IRQ disabled !!! */
inline void
get_irq_infos(channel_t *bch, isar_reg_t *ireg)
{
	ireg->iis = bch->read_reg(bch->inst.privat, ISAR_IIS);
	ireg->cmsb = bch->read_reg(bch->inst.privat, ISAR_CTRL_H);
	ireg->clsb = bch->read_reg(bch->inst.privat, ISAR_CTRL_L);
#if DUMP_MBOXFRAME
	if (bch->debug & L1_DEB_HSCX)
		mISDN_debugprint(&bch->inst, "rcv_mbox(%02x,%02x,%d)", ireg->iis, ireg->cmsb,
			ireg->clsb);
#endif
}

int
waitrecmsg(channel_t *bch, u_char *len,
	u_char *msg, int maxdelay)
{
	int timeout = 0;
	isar_hw_t *ih = bch->hw;
	
	
	while((!(bch->read_reg(bch->inst.privat, ISAR_IRQBIT) & ISAR_IRQSTA)) &&
		(timeout++ < maxdelay))
		udelay(1);
	if (timeout >= maxdelay) {
		printk(KERN_WARNING"isar recmsg IRQSTA timeout\n");
		return(0);
	}
	get_irq_infos(bch, ih->reg);
	rcv_mbox(bch, ih->reg, msg);
	*len = ih->reg->clsb;
	return(1);
}

int
ISARVersion(channel_t *bch, char *s)
{
	int ver;
	u_char msg[] = ISAR_MSG_HWVER;
	u_char tmp[64];
	u_char len;
	isar_hw_t *ih = bch->hw;
	int debug;

//	bch->cardmsg(bch->inst.privat, CARD_RESET,  NULL);
	/* disable ISAR IRQ */
	bch->write_reg(bch->inst.privat, ISAR_IRQBIT, 0);
	debug = bch->debug;
	bch->debug &= ~(L1_DEB_HSCX | L1_DEB_HSCX_FIFO);
	if (!sendmsg(bch, ISAR_HIS_VNR, 0, 3, msg))
		return(-1);
	if (!waitrecmsg(bch, &len, tmp, 100000))
		 return(-2);
	bch->debug = debug;
	if (ih->reg->iis == ISAR_IIS_VNR) {
		if (len == 1) {
			ver = tmp[0] & 0xf;
			printk(KERN_INFO "%s ISAR version %d\n", s, ver);
			return(ver);
		}
		return(-3);
	}
	return(-4);
}

int
isar_load_firmware(channel_t *bch, u_char *buf, int size)
{
	int		ret, cnt, debug;
	u_char		len, nom, noc;
	u_short		sadr, left, *sp;
	u_char		*p = buf;
	u_char		*msg, *tmpmsg, *mp, tmp[64];
	isar_hw_t	*ih = bch->hw;
	u_long		flags;
	
	struct {u_short sadr;
		u_short len;
		u_short d_key;
	} *blk_head;
		
	spin_lock_irqsave(bch->inst.hwlock, flags);
#define	BLK_HEAD_SIZE 6
	if (1 != (ret = ISARVersion(bch, "Testing"))) {
		printk(KERN_ERR"isar_load_firmware wrong isar version %d\n", ret);
		spin_unlock_irqrestore(bch->inst.hwlock, flags);
		return(1);
	}
	debug = bch->debug;
#if DBG_LOADFIRM<2
	bch->debug &= ~(L1_DEB_HSCX | L1_DEB_HSCX_FIFO);
#endif
	printk(KERN_DEBUG"isar_load_firmware buf %#lx\n", (u_long)buf);
	printk(KERN_DEBUG"isar_load_firmware size: %d\n", size);
	cnt = 0;
	/* disable ISAR IRQ */
	bch->write_reg(bch->inst.privat, ISAR_IRQBIT, 0);
	if (!(msg = kmalloc(256, GFP_ATOMIC))) {
		printk(KERN_ERR"isar_load_firmware no buffer\n");
		spin_unlock_irqrestore(bch->inst.hwlock, flags);
		return (1);
	}
	while (cnt < size) {
		blk_head = (void *)p;
#ifdef __BIG_ENDIAN
		sadr = (blk_head->sadr & 0xff)*256 + blk_head->sadr/256;
		blk_head->sadr = sadr;
		sadr = (blk_head->len & 0xff)*256 + blk_head->len/256;
		blk_head->len = sadr;
		sadr = (blk_head->d_key & 0xff)*256 + blk_head->d_key/256;
		blk_head->d_key = sadr;
#endif /* __BIG_ENDIAN */
		cnt += BLK_HEAD_SIZE;
		p += BLK_HEAD_SIZE;
		printk(KERN_DEBUG"isar firmware block (%#x,%5d,%#x)\n",
			blk_head->sadr, blk_head->len, blk_head->d_key & 0xff);
		sadr = blk_head->sadr;
		left = blk_head->len;
		if (cnt+left > size) {
			printk(KERN_ERR"isar: firmware size error have %d need %d bytes\n",
				size, cnt+left);
			ret = 1;goto reterror;
		}
		if (!sendmsg(bch, ISAR_HIS_DKEY, blk_head->d_key & 0xff, 0, NULL)) {
			printk(KERN_ERR"isar sendmsg dkey failed\n");
			ret = 1;goto reterror;
		}
		if (!waitrecmsg(bch, &len, tmp, 100000)) {
			printk(KERN_ERR"isar waitrecmsg dkey failed\n");
			ret = 1;goto reterror;
		}
		if ((ih->reg->iis != ISAR_IIS_DKEY) || ih->reg->cmsb || len) {
			printk(KERN_ERR"isar wrong dkey response (%x,%x,%x)\n",
				ih->reg->iis, ih->reg->cmsb, len);
			ret = 1;goto reterror;
		}
		while (left>0) {
			noc = MIN(126, left);
			nom = 2*noc;
			mp  = msg;
			*mp++ = sadr / 256;
			*mp++ = sadr % 256;
			left -= noc;
			*mp++ = noc;
			tmpmsg = p;
			p += nom;
			cnt += nom;
			nom += 3;
			sp = (u_short *)tmpmsg;
#if DBG_LOADFIRM
			printk(KERN_DEBUG"isar: load %3d words at %04x\n",
				 noc, sadr);
#endif
			sadr += noc;
			while(noc) {
#ifdef __BIG_ENDIAN
				*mp++ = *sp % 256;
				*mp++ = *sp / 256;
#else
				*mp++ = *sp / 256;
				*mp++ = *sp % 256;
#endif /* __BIG_ENDIAN */
				sp++;
				noc--;
			}
			if (!sendmsg(bch, ISAR_HIS_FIRM, 0, nom, msg)) {
				printk(KERN_ERR"isar sendmsg prog failed\n");
				ret = 1;goto reterror;
			}
			if (!waitrecmsg(bch, &len, tmp, 100000)) {
				printk(KERN_ERR"isar waitrecmsg prog failed\n");
				ret = 1;goto reterror;
			}
			if ((ih->reg->iis != ISAR_IIS_FIRM) || ih->reg->cmsb || len) {
				printk(KERN_ERR"isar wrong prog response (%x,%x,%x)\n",
					ih->reg->iis, ih->reg->cmsb, len);
				ret = 1;goto reterror;
			}
		}
		printk(KERN_DEBUG"isar firmware block %5d words loaded\n",
			blk_head->len);
	}
	/* 10ms delay */
	cnt = 10;
	while (cnt--)
		udelay(1000);
	msg[0] = 0xff;
	msg[1] = 0xfe;
	ih->reg->bstat = 0;
	if (!sendmsg(bch, ISAR_HIS_STDSP, 0, 2, msg)) {
		printk(KERN_ERR"isar sendmsg start dsp failed\n");
		ret = 1;goto reterror;
	}
	if (!waitrecmsg(bch, &len, tmp, 100000)) {
		printk(KERN_ERR"isar waitrecmsg start dsp failed\n");
		ret = 1;goto reterror;
	}
	if ((ih->reg->iis != ISAR_IIS_STDSP) || ih->reg->cmsb || len) {
		printk(KERN_ERR"isar wrong start dsp response (%x,%x,%x)\n",
			ih->reg->iis, ih->reg->cmsb, len);
		ret = 1;goto reterror;
	} else
		printk(KERN_DEBUG"isar start dsp success\n");
	/* NORMAL mode entered */
	/* Enable IRQs of ISAR */
	bch->write_reg(bch->inst.privat, ISAR_IRQBIT, ISAR_IRQSTA);
	spin_unlock_irqrestore(bch->inst.hwlock, flags);
	cnt = 1000; /* max 1s */
	while ((!ih->reg->bstat) && cnt) {
		mdelay(1);
		cnt--;
	}
 	if (!cnt) {
		printk(KERN_ERR"isar no general status event received\n");
		ret = 1;
		goto reterrflg;
	} else {
		printk(KERN_DEBUG"isar general status event %x\n",
			ih->reg->bstat);
	}
	/* 10ms delay */
	cnt = 10;
	while (cnt--)
		mdelay(1);
	ih->reg->iis = 0;
	spin_lock_irqsave(bch->inst.hwlock, flags);
	if (!sendmsg(bch, ISAR_HIS_DIAG, ISAR_CTRL_STST, 0, NULL)) {
		printk(KERN_ERR"isar sendmsg self tst failed\n");
		ret = 1;goto reterror;
	}
	spin_unlock_irqrestore(bch->inst.hwlock, flags);
	cnt = 10000; /* max 100 ms */
	while ((ih->reg->iis != ISAR_IIS_DIAG) && cnt) {
		udelay(10);
		cnt--;
	}
	mdelay(1);
	if (!cnt) {
		printk(KERN_ERR"isar no self tst response\n");
		ret = 1;goto reterrflg;
	}
	if ((ih->reg->cmsb == ISAR_CTRL_STST) && (ih->reg->clsb == 1)
		&& (ih->reg->par[0] == 0)) {
		printk(KERN_DEBUG"isar selftest OK\n");
	} else {
		printk(KERN_DEBUG"isar selftest not OK %x/%x/%x\n",
			ih->reg->cmsb, ih->reg->clsb, ih->reg->par[0]);
		ret = 1;goto reterrflg;
	}
	spin_lock_irqsave(bch->inst.hwlock, flags);
	ih->reg->iis = 0;
	if (!sendmsg(bch, ISAR_HIS_DIAG, ISAR_CTRL_SWVER, 0, NULL)) {
		printk(KERN_ERR"isar RQST SVN failed\n");
		ret = 1;goto reterror;
	}
	spin_unlock_irqrestore(bch->inst.hwlock, flags);
	cnt = 30000; /* max 300 ms */
	while ((ih->reg->iis != ISAR_IIS_DIAG) && cnt) {
		udelay(10);
		cnt--;
	}
	mdelay(1);
	if (!cnt) {
		printk(KERN_ERR"isar no SVN response\n");
		ret = 1;goto reterrflg;
	} else {
		if ((ih->reg->cmsb == ISAR_CTRL_SWVER) && (ih->reg->clsb == 1))
			printk(KERN_DEBUG"isar software version %#x\n",
				ih->reg->par[0]);
		else {
			
			printk(KERN_ERR"isar wrong swver response (%x,%x) cnt(%d)\n",
				ih->reg->cmsb, ih->reg->clsb, cnt);
			ret = 1;goto reterrflg;
		}
	}
	bch->debug = debug;
	spin_lock_irqsave(bch->inst.hwlock, flags);
	isar_setup(bch);
	spin_unlock_irqrestore(bch->inst.hwlock, flags);
	bch->inst.obj->own_ctrl(&bch->inst, MGR_LOADFIRM | CONFIRM, NULL);
	ret = 0;
reterrflg:
	spin_lock_irqsave(bch->inst.hwlock, flags);
reterror:
	bch->debug = debug;
	if (ret)
		/* disable ISAR IRQ */
		bch->write_reg(bch->inst.privat, ISAR_IRQBIT, 0);
	spin_unlock_irqrestore(bch->inst.hwlock, flags);
	kfree(msg);
	return(ret);
}

#ifdef OBSOLETE
#define B_LL_READY	8
#define B_LL_NOCARRIER	9
#define B_LL_CONNECT	10
#define B_LL_OK		11
#define B_LL_FCERROR	12
#define B_TOUCH_TONE	13
#endif

static inline void
deliver_status(channel_t *bch, int status)
{
	if (bch->debug & L1_DEB_HSCX)
		mISDN_debugprint(&bch->inst, "HL->LL FAXIND %x", status);
	mISDN_queue_data(&bch->inst, FLG_MSG_UP, PH_STATUS | INDICATION, status, 0, NULL, 0);
}

#ifdef OBSOLETE
static void
isar_bh(channel_t *bch)
{
	int	tt;

	if (test_and_clear_bit(B_LL_READY, &bch->event))
		deliver_status(bch, HW_MOD_READY);
	if (test_and_clear_bit(B_LL_NOCARRIER, &bch->event))
		deliver_status(bch, HW_MOD_NOCARR);
	if (test_and_clear_bit(B_LL_CONNECT, &bch->event))
		deliver_status(bch, HW_MOD_CONNECT);
	if (test_and_clear_bit(B_LL_OK, &bch->event))
		deliver_status(bch, HW_MOD_OK);
	if (test_and_clear_bit(B_LL_FCERROR, &bch->event))
		deliver_status(bch, HW_MOD_FCERROR);
	if (test_and_clear_bit(B_TOUCH_TONE, &bch->event)) {
		tt = bch->conmsg[0] | 0x30;
		if (tt == 0x3e)
			tt = '*';
		else if (tt == 0x3f)
			tt = '#';
		else if (tt > '9')
			tt += 7;
		tt |= DTMF_TONE_VAL;
		mISDN_queue_data(&bch->inst, FLG_MSG_UP, PH_CONTROL | INDICATION,
			0, sizeof(int), &tt, 0);
	}
}
#endif

static inline void
isar_rcv_frame(channel_t *bch)
{
	u_char		*ptr;
	struct sk_buff	*skb;
	isar_hw_t	*ih = bch->hw;
	
	if (!ih->reg->clsb) {
		mISDN_debugprint(&bch->inst, "isar zero len frame");
		bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
		return;
	}
	switch (bch->state) {
	    case ISDN_PID_NONE:
		mISDN_debugprint(&bch->inst, "isar protocol 0 spurious IIS_RDATA %x/%x/%x",
			ih->reg->iis, ih->reg->cmsb, ih->reg->clsb);
		printk(KERN_WARNING"isar protocol 0 spurious IIS_RDATA %x/%x/%x\n",
			ih->reg->iis, ih->reg->cmsb, ih->reg->clsb);
		bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
		break;
	    case ISDN_PID_L1_B_64TRANS:
	    case ISDN_PID_L2_B_TRANSDTMF:
	    case ISDN_PID_L1_B_MODEM_ASYNC:
	    	if (!bch->rx_skb) {
	    		bch->rx_skb = alloc_stack_skb(ih->reg->clsb, bch->up_headerlen);
	    		if (unlikely(!bch->rx_skb)) {
	    			printk(KERN_WARNING "mISDN: skb out of memory\n");
	    			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
	    			break;
			}
		}
		rcv_mbox(bch, ih->reg, (u_char *)skb_put(bch->rx_skb, ih->reg->clsb));
		queue_ch_frame(bch, INDICATION, MISDN_ID_ANY, bch->rx_skb);
		bch->rx_skb = NULL;
		break;
	    case ISDN_PID_L1_B_64HDLC:
	    	if (!bch->rx_skb) {
	    		bch->rx_skb = alloc_stack_skb(bch->maxlen + 2, bch->up_headerlen);
	    		if (unlikely(!bch->rx_skb)) {
	    			printk(KERN_WARNING "mISDN: skb out of memory\n");
	    			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
	    			break;
			}
		}
		if ((bch->rx_skb->len + ih->reg->clsb) > (bch->maxlen + 2)) {
			if (bch->debug & L1_DEB_WARN)
				mISDN_debugprint(&bch->inst, "isar_rcv_frame: incoming packet too large");
			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
			skb_trim(bch->rx_skb, 0);
			break;
		}
		if (ih->reg->cmsb & HDLC_ERROR) {
			if (bch->debug & L1_DEB_WARN)
				mISDN_debugprint(&bch->inst, "isar frame error %x len %d",
					ih->reg->cmsb, ih->reg->clsb);
#ifdef ERROR_STATISTIC
			if (ih->reg->cmsb & HDLC_ERR_RER)
				bch->err_inv++;
			if (ih->reg->cmsb & HDLC_ERR_CER)
				bch->err_crc++;
#endif
			skb_trim(bch->rx_skb, 0);
			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
			break;
		}
		if (ih->reg->cmsb & HDLC_FSD)
			skb_trim(bch->rx_skb, 0);
		ptr = skb_put(bch->rx_skb, ih->reg->clsb);
		rcv_mbox(bch, ih->reg, ptr);
		if (ih->reg->cmsb & HDLC_FED) {
			if (bch->rx_skb->len < 3) { /* last 2 bytes are the FCS */
				if (bch->debug & L1_DEB_WARN)
					mISDN_debugprint(&bch->inst, "isar frame to short %d",
						bch->rx_skb->len);
				skb_trim(bch->rx_skb, 0);
				break;
			}
			skb_trim(bch->rx_skb, bch->rx_skb->len - 2);
			if (bch->rx_skb->len < MISDN_COPY_SIZE) {
				skb = alloc_stack_skb(bch->rx_skb->len, bch->up_headerlen);
				if (skb) {
					memcpy(skb_put(skb, bch->rx_skb->len),
						bch->rx_skb->data, bch->rx_skb->len);
					skb_trim(bch->rx_skb, 0);
				} else {
					skb = bch->rx_skb;
					bch->rx_skb = NULL;
				}
			} else {
				skb = bch->rx_skb;
				bch->rx_skb = NULL;
			}
			queue_ch_frame(bch, INDICATION, MISDN_ID_ANY, skb);
		}
		break;
	case ISDN_PID_L1_B_T30FAX:
		if (ih->state != STFAX_ACTIV) {
			if (bch->debug & L1_DEB_WARN)
				mISDN_debugprint(&bch->inst, "isar_rcv_frame: not ACTIV");
			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
			if (bch->rx_skb)
				skb_trim(bch->rx_skb, 0);
			break;
		}
		if (ih->cmd == PCTRL_CMD_FRM) {
		    	if (!bch->rx_skb) {
		    		bch->rx_skb = alloc_stack_skb(ih->reg->clsb, bch->up_headerlen);
	    			if (unlikely(!bch->rx_skb)) {
	    				printk(KERN_WARNING "mISDN: skb out of memory\n");
		    			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
	    				break;
				}
			}
			rcv_mbox(bch, ih->reg, skb_put(bch->rx_skb, ih->reg->clsb));
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "isar_rcv_frame: %d",
					bch->rx_skb->len);
			if (ih->reg->cmsb & SART_NMD) { /* ABORT */
				if (bch->debug & L1_DEB_WARN)
					mISDN_debugprint(&bch->inst, "isar_rcv_frame: no more data");
				bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
				sendmsg(bch, SET_DPS(ih->dpath) |
					ISAR_HIS_PUMPCTRL, PCTRL_CMD_ESC,
					0, NULL);
				ih->state = STFAX_ESCAPE;
//				set_skb_flag(skb, DF_NOMOREDATA);
			}
			queue_ch_frame(bch, INDICATION, MISDN_ID_ANY, bch->rx_skb);
			bch->rx_skb = NULL;
			if (ih->reg->cmsb & SART_NMD)
				deliver_status(bch, HW_MOD_NOCARR);
			break;
		}
		if (ih->cmd != PCTRL_CMD_FRH) {
			if (bch->debug & L1_DEB_WARN)
				mISDN_debugprint(&bch->inst, "isar_rcv_frame: unknown fax mode %x",
					ih->cmd);
			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
			if (bch->rx_skb)
				skb_trim(bch->rx_skb, 0);
			break;
		}
		/* PCTRL_CMD_FRH */
	    	if (!bch->rx_skb) {
	    		bch->rx_skb = alloc_stack_skb(bch->maxlen + 2, bch->up_headerlen);
	    		if (unlikely(!bch->rx_skb)) {
	    			printk(KERN_WARNING "mISDN: skb out of memory\n");
	    			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
	    			break;
			}
		}
		if ((bch->rx_skb->len + ih->reg->clsb) > (bch->maxlen + 2)) {
			if (bch->debug & L1_DEB_WARN)
				mISDN_debugprint(&bch->inst, "isar_rcv_frame: incoming packet too large");
			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
			skb_trim(bch->rx_skb, 0);
			break;
		}  else if (ih->reg->cmsb & HDLC_ERROR) {
			if (bch->debug & L1_DEB_WARN)
				mISDN_debugprint(&bch->inst, "isar frame error %x len %d",
					ih->reg->cmsb, ih->reg->clsb);
			skb_trim(bch->rx_skb, 0);
			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
			break;
		}
		if (ih->reg->cmsb & HDLC_FSD)
			skb_trim(bch->rx_skb, 0);
		ptr = skb_put(bch->rx_skb, ih->reg->clsb);
		rcv_mbox(bch, ih->reg, ptr);
		if (ih->reg->cmsb & HDLC_FED) {
			if (bch->rx_skb->len < 3) { /* last 2 bytes are the FCS */
				if (bch->debug & L1_DEB_WARN)
					mISDN_debugprint(&bch->inst, "isar frame to short %d",
						bch->rx_skb->len);
				skb_trim(bch->rx_skb, 0);
				break;
			}
			skb_trim(bch->rx_skb, bch->rx_skb->len - 2);
			if (bch->rx_skb->len < MISDN_COPY_SIZE) {
				skb = alloc_stack_skb(bch->rx_skb->len, bch->up_headerlen);
				if (skb) {
					memcpy(skb_put(skb, bch->rx_skb->len),
						bch->rx_skb->data, bch->rx_skb->len);
					skb_trim(bch->rx_skb, 0);
				} else {
					skb = bch->rx_skb;
					bch->rx_skb = NULL;
				}
			} else {
				skb = bch->rx_skb;
				bch->rx_skb = NULL;
			}
			queue_ch_frame(bch, INDICATION, MISDN_ID_ANY, skb);
		}
		if (ih->reg->cmsb & SART_NMD) { /* ABORT */
			if (bch->debug & L1_DEB_WARN)
				mISDN_debugprint(&bch->inst, "isar_rcv_frame: no more data");
			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
			if (bch->rx_skb)
				skb_trim(bch->rx_skb, 0);
			sendmsg(bch, SET_DPS(ih->dpath) | ISAR_HIS_PUMPCTRL,
				PCTRL_CMD_ESC, 0, NULL);
			ih->state = STFAX_ESCAPE;
			deliver_status(bch, HW_MOD_NOCARR);
		}
		break;
	default:
		printk(KERN_ERR"isar_rcv_frame protocol (%x)error\n", bch->state);
		bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
		break;
	}
}

void
isar_fill_fifo(channel_t *bch)
{
	isar_hw_t	*ih = bch->hw;
	int count;
	u_char msb;
	u_char *ptr;

	if ((bch->debug & L1_DEB_HSCX) && !(bch->debug & L1_DEB_HSCX_FIFO))
		mISDN_debugprint(&bch->inst, "%s", __FUNCTION__);
	if (!bch->tx_skb)
		return;
	count = bch->tx_skb->len - bch->tx_idx;
	if (count <= 0)
		return;
	if (!(ih->reg->bstat &
		(ih->dpath == 1 ? BSTAT_RDM1 : BSTAT_RDM2)))
		return;
	if (count > ih->mml) {
		msb = 0;
		count = ih->mml;
	} else {
		msb = HDLC_FED;
	}
	ptr = bch->tx_skb->data + bch->tx_idx;
	if (!bch->tx_idx) {
		if (bch->debug & L1_DEB_HSCX)
			mISDN_debugprint(&bch->inst, "frame start");
		if ((bch->state == ISDN_PID_L1_B_T30FAX) &&
			(ih->cmd == PCTRL_CMD_FTH)) {
			if (count > 1) {
				if ((ptr[0]== 0xff) && (ptr[1] == 0x13)) {
					/* last frame */
					test_and_set_bit(FLG_LASTDATA, &bch->Flags);
					if (bch->debug & L1_DEB_HSCX)
						mISDN_debugprint(&bch->inst, "set LASTDATA");
					if (msb == HDLC_FED)
						test_and_set_bit(FLG_DLEETX, &bch->Flags);
				}
			}
		}
		msb |= HDLC_FST;
	}
	bch->tx_idx += count;
	switch (bch->state) {
		case ISDN_PID_NONE:
			printk(KERN_ERR "%s: wrong protocol 0\n", __FUNCTION__);
			break;
		case ISDN_PID_L1_B_64TRANS:
		case ISDN_PID_L2_B_TRANSDTMF:
		case ISDN_PID_L1_B_MODEM_ASYNC:
			sendmsg(bch, SET_DPS(ih->dpath) | ISAR_HIS_SDATA,
				0, count, ptr);
			break;
		case ISDN_PID_L1_B_64HDLC:
			sendmsg(bch, SET_DPS(ih->dpath) | ISAR_HIS_SDATA,
				msb, count, ptr);
			break;
		case ISDN_PID_L1_B_T30FAX:
			if (ih->state != STFAX_ACTIV) {
				if (bch->debug & L1_DEB_WARN)
					mISDN_debugprint(&bch->inst, "%s: not ACTIV",
						__FUNCTION__);
			} else if (ih->cmd == PCTRL_CMD_FTH) {
				sendmsg(bch, SET_DPS(ih->dpath) | ISAR_HIS_SDATA,
					msb, count, ptr);
			} else if (ih->cmd == PCTRL_CMD_FTM) {
				sendmsg(bch, SET_DPS(ih->dpath) | ISAR_HIS_SDATA,
					0, count, ptr);
			} else {
				if (bch->debug & L1_DEB_WARN)
					mISDN_debugprint(&bch->inst, "%s: not FTH/FTM",
						__FUNCTION__);
			}
			break;
		default:
			if (bch->debug)
				mISDN_debugprint(&bch->inst, "%s: protocol(%x) error",
					__FUNCTION__, bch->state);
			printk(KERN_ERR "%s: protocol(%x) error\n",
				__FUNCTION__, bch->state);
			break;
	}
}

inline
channel_t *sel_bch_isar(channel_t *bch, u_char dpath)
{

	if ((!dpath) || (dpath == 3))
		return(NULL);
	
	if (((isar_hw_t *)bch[0].hw)->dpath == dpath)
		return(&bch[0]);
	if (((isar_hw_t *)bch[1].hw)->dpath == dpath)
		return(&bch[1]);
	return(NULL);
}

inline void
send_frames(channel_t *bch)
{
	isar_hw_t	*ih = bch->hw;

	if (bch->tx_skb && (bch->tx_skb->len > bch->tx_idx)) {
		isar_fill_fifo(bch);
	} else {
		if (bch->state == ISDN_PID_L1_B_T30FAX) {
			if (ih->cmd == PCTRL_CMD_FTH) {
				if (test_bit(FLG_LASTDATA, &bch->Flags)) {
					printk(KERN_WARNING "set NMD_DATA\n");
					test_and_set_bit(FLG_NMD_DATA, &bch->Flags);
				}
			} else if (ih->cmd == PCTRL_CMD_FTM) {
				if (test_bit(FLG_DLEETX, &bch->Flags)) {
					test_and_set_bit(FLG_LASTDATA, &bch->Flags);
					test_and_set_bit(FLG_NMD_DATA, &bch->Flags);
				}
			}
		}
		if (bch->tx_skb)
			dev_kfree_skb(bch->tx_skb);
		bch->tx_idx = 0;
		if (test_bit(FLG_TX_NEXT, &bch->Flags)) {
			bch->tx_skb = bch->next_skb;
			if (bch->tx_skb) {
				mISDN_head_t	*hh = mISDN_HEAD_P(bch->tx_skb);

				bch->next_skb = NULL;
				test_and_clear_bit(FLG_TX_NEXT, &bch->Flags);
				queue_ch_frame(bch, CONFIRM, hh->dinfo, NULL);
				isar_fill_fifo(bch);
			} else {
				printk(KERN_WARNING "hdlc tx irq TX_NEXT without skb\n");
				test_and_clear_bit(FLG_TX_NEXT, &bch->Flags);
				test_and_clear_bit(FLG_TX_BUSY, &bch->Flags);
			}
		} else {
			bch->tx_skb = NULL;
			if (test_and_clear_bit(FLG_DLEETX, &bch->Flags)) {
				if (test_and_clear_bit(FLG_LASTDATA, &bch->Flags)) {
					if (test_and_clear_bit(FLG_NMD_DATA, &bch->Flags)) {
						u_char dummy = 0;
						sendmsg(bch, SET_DPS(ih->dpath) |
							ISAR_HIS_SDATA, 0x01, 1, &dummy);
					}
					test_and_set_bit(FLG_LL_OK, &bch->Flags);
				} else {
					deliver_status(bch, HW_MOD_CONNECT);
				}
			}
			test_and_clear_bit(FLG_TX_BUSY, &bch->Flags);
//			bch_sched_event(bch, B_XMTBUFREADY);
		}
	}
}

inline void
check_send(channel_t *bch, u_char rdm)
{
	channel_t *bc;
	
	if (rdm & BSTAT_RDM1) {
		if ((bc = sel_bch_isar(bch, 1))) {
			if (test_bit(FLG_ACTIVE, &bc->Flags)) {
				send_frames(bc);
			}
		}
	}
	if (rdm & BSTAT_RDM2) {
		if ((bc = sel_bch_isar(bch, 2))) {
			if (test_bit(FLG_ACTIVE, &bc->Flags)) {
				send_frames(bc);
			}
		}
	}
	
}

const char *dmril[] = {"NO SPEED", "1200/75", "NODEF2", "75/1200", "NODEF4",
			"300", "600", "1200", "2400", "4800", "7200",
			"9600nt", "9600t", "12000", "14400", "WRONG"};
const char *dmrim[] = {"NO MOD", "NO DEF", "V32/V32b", "V22", "V21",
			"Bell103", "V23", "Bell202", "V17", "V29", "V27ter"};

static void
isar_pump_status_rsp(channel_t *bch, isar_reg_t *ireg) {
	isar_hw_t	*ih = bch->hw;
	u_char ril = ireg->par[0];
	u_char rim;

	if (!test_and_clear_bit(ISAR_RATE_REQ, &ireg->Flags))
		return;
	if (ril > 14) {
		if (bch->debug & L1_DEB_WARN)
			mISDN_debugprint(&bch->inst, "wrong pstrsp ril=%d",ril);
		ril = 15;
	}
	switch(ireg->par[1]) {
		case 0:
			rim = 0;
			break;
		case 0x20:
			rim = 2;
			break;
		case 0x40:
			rim = 3;
			break;
		case 0x41:
			rim = 4;
			break;
		case 0x51:
			rim = 5;
			break;
		case 0x61:
			rim = 6;
			break;
		case 0x71:
			rim = 7;
			break;
		case 0x82:
			rim = 8;
			break;
		case 0x92:
			rim = 9;
			break;
		case 0xa2:
			rim = 10;
			break;
		default:
			rim = 1;
			break;
	}
	sprintf(ih->conmsg,"%s %s", dmril[ril], dmrim[rim]);
	if (bch->debug & L1_DEB_HSCX)
		mISDN_debugprint(&bch->inst, "pump strsp %s %s", ih->conmsg);
}

static void
isar_pump_statev_modem(channel_t *bch, u_char devt) {
	isar_hw_t	*ih = bch->hw;
	u_char dps = SET_DPS(ih->dpath);

	switch(devt) {
		case PSEV_10MS_TIMER:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev TIMER");
			break;
		case PSEV_CON_ON:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev CONNECT");
			deliver_status(bch, HW_MOD_CONNECT);
			break;
		case PSEV_CON_OFF:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev NO CONNECT");
			sendmsg(bch, dps | ISAR_HIS_PSTREQ, 0, 0, NULL);
			deliver_status(bch, HW_MOD_NOCARR);
			break;
		case PSEV_V24_OFF:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev V24 OFF");
			break;
		case PSEV_CTS_ON:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev CTS ON");
			break;
		case PSEV_CTS_OFF:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev CTS OFF");
			break;
		case PSEV_DCD_ON:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev CARRIER ON");
			test_and_set_bit(ISAR_RATE_REQ, &ih->reg->Flags);
			sendmsg(bch, dps | ISAR_HIS_PSTREQ, 0, 0, NULL);
			break;
		case PSEV_DCD_OFF:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev CARRIER OFF");
			break;
		case PSEV_DSR_ON:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev DSR ON");
			break;
		case PSEV_DSR_OFF:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev DSR_OFF");
			break;
		case PSEV_REM_RET:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev REMOTE RETRAIN");
			break;
		case PSEV_REM_REN:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev REMOTE RENEGOTIATE");
			break;
		case PSEV_GSTN_CLR:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev GSTN CLEAR", devt);
			break;
		default:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "unknown pump stev %x", devt);
			break;
	}
}

static void
isar_pump_statev_fax(channel_t *bch, u_char devt) {
	isar_hw_t	*ih = bch->hw;
	u_char dps = SET_DPS(ih->dpath);
	u_char p1;

	switch(devt) {
		case PSEV_10MS_TIMER:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev TIMER");
			break;
		case PSEV_RSP_READY:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev RSP_READY");
			ih->state = STFAX_READY;
			deliver_status(bch, HW_MOD_READY);
//			if (test_bit(BC_FLG_ORIG, &bch->Flags)) {
//				isar_pump_cmd(bch, HW_MOD_FRH, 3);
//			} else {
//				isar_pump_cmd(bch, HW_MOD_FTH, 3);
//			}
			break;
		case PSEV_LINE_TX_H:
			if (ih->state == STFAX_LINE) {
				if (bch->debug & L1_DEB_HSCX)
					mISDN_debugprint(&bch->inst, "pump stev LINE_TX_H");
				ih->state = STFAX_CONT;
				sendmsg(bch, dps | ISAR_HIS_PUMPCTRL, PCTRL_CMD_CONT, 0, NULL);
			} else {
				if (bch->debug & L1_DEB_WARN)
					mISDN_debugprint(&bch->inst, "pump stev LINE_TX_H wrong st %x",
						ih->state);
			}
			break;
		case PSEV_LINE_RX_H:
			if (ih->state == STFAX_LINE) {
				if (bch->debug & L1_DEB_HSCX)
					mISDN_debugprint(&bch->inst, "pump stev LINE_RX_H");
				ih->state = STFAX_CONT;
				sendmsg(bch, dps | ISAR_HIS_PUMPCTRL, PCTRL_CMD_CONT, 0, NULL);
			} else {
				if (bch->debug & L1_DEB_WARN)
					mISDN_debugprint(&bch->inst, "pump stev LINE_RX_H wrong st %x",
						ih->state);
			}
			break;
		case PSEV_LINE_TX_B:
			if (ih->state == STFAX_LINE) {
				if (bch->debug & L1_DEB_HSCX)
					mISDN_debugprint(&bch->inst, "pump stev LINE_TX_B");
				ih->state = STFAX_CONT;
				sendmsg(bch, dps | ISAR_HIS_PUMPCTRL, PCTRL_CMD_CONT, 0, NULL);
			} else {
				if (bch->debug & L1_DEB_WARN)
					mISDN_debugprint(&bch->inst, "pump stev LINE_TX_B wrong st %x",
						ih->state);
			}
			break;
		case PSEV_LINE_RX_B:
			if (ih->state == STFAX_LINE) {
				if (bch->debug & L1_DEB_HSCX)
					mISDN_debugprint(&bch->inst, "pump stev LINE_RX_B");
				ih->state = STFAX_CONT;
				sendmsg(bch, dps | ISAR_HIS_PUMPCTRL, PCTRL_CMD_CONT, 0, NULL);
			} else {
				if (bch->debug & L1_DEB_WARN)
					mISDN_debugprint(&bch->inst, "pump stev LINE_RX_B wrong st %x",
						ih->state);
			}
			break;
		case PSEV_RSP_CONN:
			if (ih->state == STFAX_CONT) {
				if (bch->debug & L1_DEB_HSCX)
					mISDN_debugprint(&bch->inst, "pump stev RSP_CONN");
				ih->state = STFAX_ACTIV;
				test_and_set_bit(ISAR_RATE_REQ, &ih->reg->Flags);
				sendmsg(bch, dps | ISAR_HIS_PSTREQ, 0, 0, NULL);
				if (ih->cmd == PCTRL_CMD_FTH) {
					int delay = (ih->mod == 3) ? 1000 : 200;
					/* 1s (200 ms) Flags before data */
					if (test_and_set_bit(FLG_FTI_RUN, &bch->Flags))
						del_timer(&ih->ftimer);
					ih->ftimer.expires =
						jiffies + ((delay * HZ)/1000);
					test_and_set_bit(FLG_LL_CONN,
						&bch->Flags);
					add_timer(&ih->ftimer);
				} else {
					deliver_status(bch, HW_MOD_CONNECT);
				}
			} else {
				if (bch->debug & L1_DEB_WARN)
					mISDN_debugprint(&bch->inst, "pump stev RSP_CONN wrong st %x",
						ih->state);
			}
			break;
		case PSEV_FLAGS_DET:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev FLAGS_DET");
			break;
		case PSEV_RSP_DISC:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev RSP_DISC state(%d)", ih->state);
			if (ih->state == STFAX_ESCAPE) {
				p1 = 5;
				switch(ih->newcmd) {
					case 0:
						ih->state = STFAX_READY;
						break;
					case PCTRL_CMD_FTM:
						p1 = 2;
					case PCTRL_CMD_FTH:
						sendmsg(bch, dps | ISAR_HIS_PUMPCTRL,
							PCTRL_CMD_SILON, 1, &p1);
						ih->state = STFAX_SILDET;
						break;
					case PCTRL_CMD_FRH:
					case PCTRL_CMD_FRM:
						p1 = ih->mod = ih->newmod;
						ih->newmod = 0;
						ih->cmd = ih->newcmd;
						ih->newcmd = 0;
						sendmsg(bch, dps | ISAR_HIS_PUMPCTRL,
							ih->cmd, 1, &p1);
						ih->state = STFAX_LINE;
						ih->try_mod = 3;
						break;
					default:
						if (bch->debug & L1_DEB_HSCX)
							mISDN_debugprint(&bch->inst, "RSP_DISC unknown newcmd %x", ih->newcmd);
						break;
				}
			} else if (ih->state == STFAX_ACTIV) {
				if (test_and_clear_bit(FLG_LL_OK, &bch->Flags)) {
					deliver_status(bch, HW_MOD_OK);
				} else if (ih->cmd == PCTRL_CMD_FRM) {
					deliver_status(bch, HW_MOD_NOCARR);
				} else {
					deliver_status(bch, HW_MOD_FCERROR);
				}
				ih->state = STFAX_READY;
			} else if (ih->state != STFAX_SILDET) { // ignore in STFAX_SILDET
				ih->state = STFAX_READY;
				deliver_status(bch, HW_MOD_FCERROR);
			}
			break;
		case PSEV_RSP_SILDET:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev RSP_SILDET");
			if (ih->state == STFAX_SILDET) {
				p1 = ih->mod = ih->newmod;
				ih->newmod = 0;
				ih->cmd = ih->newcmd;
				ih->newcmd = 0;
				sendmsg(bch, dps | ISAR_HIS_PUMPCTRL,
					ih->cmd, 1, &p1);
				ih->state = STFAX_LINE;
				ih->try_mod = 3;
			}
			break;
		case PSEV_RSP_SILOFF:
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev RSP_SILOFF");
			break;
		case PSEV_RSP_FCERR:
			if (ih->state == STFAX_LINE) {
				if (bch->debug & L1_DEB_HSCX)
					mISDN_debugprint(&bch->inst, "pump stev RSP_FCERR try %d",
						ih->try_mod);
				if (ih->try_mod--) {
					sendmsg(bch, dps | ISAR_HIS_PUMPCTRL,
						ih->cmd, 1,
						&ih->mod);
					break;
				}
			}
			if (bch->debug & L1_DEB_HSCX)
				mISDN_debugprint(&bch->inst, "pump stev RSP_FCERR");
			ih->state = STFAX_ESCAPE;
			sendmsg(bch, dps | ISAR_HIS_PUMPCTRL, PCTRL_CMD_ESC, 0, NULL);
			deliver_status(bch, HW_MOD_FCERROR);
			break;
		default:
			break;
	}
}

static char debbuf[128];

void
isar_int_main(channel_t *bch)
{
	isar_hw_t	*ih = bch->hw;
	channel_t *bc;

	get_irq_infos(bch, ih->reg);
	switch (ih->reg->iis & ISAR_IIS_MSCMSD) {
		case ISAR_IIS_RDATA:
			if ((bc = sel_bch_isar(bch, ih->reg->iis >> 6))) {
				isar_rcv_frame(bc);
			} else {
				mISDN_debugprint(&bch->inst, "isar spurious IIS_RDATA %x/%x/%x",
					ih->reg->iis, ih->reg->cmsb, ih->reg->clsb);
				bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
			}
			break;
		case ISAR_IIS_GSTEV:
			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
			ih->reg->bstat |= ih->reg->cmsb;
			check_send(bch, ih->reg->cmsb);
			break;
		case ISAR_IIS_BSTEV:
#ifdef ERROR_STATISTIC
			if ((bc = sel_bch_isar(bch, ih->reg->iis >> 6))) {
				if (ih->reg->cmsb == BSTEV_TBO)
					bc->err_tx++;
				if (ih->reg->cmsb == BSTEV_RBO)
					bc->err_rdo++;
			}
#endif
			if (bch->debug & L1_DEB_WARN)
				mISDN_debugprint(&bch->inst, "Buffer STEV dpath%d msb(%x)",
					ih->reg->iis>>6, ih->reg->cmsb);
			bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
			break;
		case ISAR_IIS_PSTEV:
			if ((bc = sel_bch_isar(bch, ih->reg->iis >> 6))) {
				rcv_mbox(bc, ih->reg, (u_char *)ih->reg->par);
				if (bc->state == ISDN_PID_L1_B_MODEM_ASYNC) {
					isar_pump_statev_modem(bc, ih->reg->cmsb);
				} else if (bc->state == ISDN_PID_L1_B_T30FAX) {
					isar_pump_statev_fax(bc, ih->reg->cmsb);
				} else if (bc->state == ISDN_PID_L2_B_TRANSDTMF) {
					int	tt;
					tt = ih->reg->cmsb | 0x30;
					if (tt == 0x3e)
						tt = '*';
					else if (tt == 0x3f)
						tt = '#';
					else if (tt > '9')
						tt += 7;
					tt |= DTMF_TONE_VAL;
					mISDN_queue_data(&bch->inst, FLG_MSG_UP, PH_CONTROL | INDICATION,
						0, sizeof(int), &tt, 0);
				} else {
					if (bch->debug & L1_DEB_WARN)
						mISDN_debugprint(&bch->inst, "isar IIS_PSTEV pmode %d stat %x",
							bc->state, ih->reg->cmsb);
				}
			} else {
				mISDN_debugprint(&bch->inst, "isar spurious IIS_PSTEV %x/%x/%x",
					ih->reg->iis, ih->reg->cmsb, ih->reg->clsb);
				bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
			}
			break;
		case ISAR_IIS_PSTRSP:
			if ((bc = sel_bch_isar(bch, ih->reg->iis >> 6))) {
				rcv_mbox(bc, ih->reg, (u_char *)ih->reg->par);
				isar_pump_status_rsp(bc, ih->reg);
			} else {
				mISDN_debugprint(&bch->inst, "isar spurious IIS_PSTRSP %x/%x/%x",
					ih->reg->iis, ih->reg->cmsb, ih->reg->clsb);
				bch->write_reg(bch->inst.privat, ISAR_IIA, 0);
			}
			break;
		case ISAR_IIS_DIAG:
		case ISAR_IIS_BSTRSP:
		case ISAR_IIS_IOM2RSP:
			rcv_mbox(bch, ih->reg, (u_char *)ih->reg->par);
			if ((bch->debug & (L1_DEB_HSCX | L1_DEB_HSCX_FIFO))
				== L1_DEB_HSCX) {
				u_char *tp=debbuf;

				tp += sprintf(debbuf, "msg iis(%x) msb(%x)",
					ih->reg->iis, ih->reg->cmsb);
				mISDN_QuickHex(tp, (u_char *)ih->reg->par, ih->reg->clsb);
				mISDN_debugprint(&bch->inst, debbuf);
			}
			break;
		case ISAR_IIS_INVMSG:
			rcv_mbox(bch, ih->reg, debbuf);
			if (bch->debug & L1_DEB_WARN)
				mISDN_debugprint(&bch->inst, "invalid msg his:%x",
					ih->reg->cmsb);
			break;
		default:
			rcv_mbox(bch, ih->reg, debbuf);
			if (bch->debug & L1_DEB_WARN)
				mISDN_debugprint(&bch->inst, "unhandled msg iis(%x) ctrl(%x/%x)",
					ih->reg->iis, ih->reg->cmsb, ih->reg->clsb);
			break;
	}
}

static void
ftimer_handler(channel_t *bch) {
	if (bch->debug)
		mISDN_debugprint(&bch->inst, "ftimer flags %04x",
			bch->Flags);
	test_and_clear_bit(FLG_FTI_RUN, &bch->Flags);
	if (test_and_clear_bit(FLG_LL_CONN, &bch->Flags)) {
		deliver_status(bch, HW_MOD_CONNECT);
	}
}

static void
setup_pump(channel_t *bch) {
	isar_hw_t	*ih = bch->hw;
	u_char dps = SET_DPS(ih->dpath);
	u_char ctrl, param[6];

	switch (bch->state) {
		case ISDN_PID_NONE:
		case ISDN_PID_L1_B_64TRANS:
		case ISDN_PID_L1_B_64HDLC:
			sendmsg(bch, dps | ISAR_HIS_PUMPCFG, PMOD_BYPASS, 0, NULL);
			break;
		case ISDN_PID_L2_B_TRANSDTMF:
			if (test_bit(FLG_DTMFSEND, &bch->Flags)) {
				param[0] = 5; /* TOA 5 db */
				sendmsg(bch, dps | ISAR_HIS_PUMPCFG, PMOD_DTMF_TRANS, 1, param);
			} else {
				param[0] = 40; /* REL -46 dbm */
				sendmsg(bch, dps | ISAR_HIS_PUMPCFG, PMOD_DTMF, 1, param);
			}
		case ISDN_PID_L1_B_MODEM_ASYNC:
			ctrl = PMOD_DATAMODEM;
			if (test_bit(FLG_ORIGIN, &bch->Flags)) {
				ctrl |= PCTRL_ORIG;
				param[5] = PV32P6_CTN;
			} else {
				param[5] = PV32P6_ATN;
			}
			param[0] = 6; /* 6 db */
			param[1] = PV32P2_V23R | PV32P2_V22A | PV32P2_V22B |
				   PV32P2_V22C | PV32P2_V21 | PV32P2_BEL;
			param[2] = PV32P3_AMOD | PV32P3_V32B | PV32P3_V23B;
			param[3] = PV32P4_UT144;
			param[4] = PV32P5_UT144;
			sendmsg(bch, dps | ISAR_HIS_PUMPCFG, ctrl, 6, param);
			break;
		case ISDN_PID_L1_B_T30FAX:
			ctrl = PMOD_FAX;
			if (test_bit(FLG_ORIGIN, &bch->Flags)) {
				ctrl |= PCTRL_ORIG;
				param[1] = PFAXP2_CTN;
			} else {
				param[1] = PFAXP2_ATN;
			}
			param[0] = 6; /* 6 db */
			sendmsg(bch, dps | ISAR_HIS_PUMPCFG, ctrl, 2, param);
			ih->state = STFAX_NULL;
			ih->newcmd = 0;
			ih->newmod = 0;
			test_and_set_bit(FLG_FTI_RUN, &bch->Flags);
			break;
	}
	udelay(1000);
	sendmsg(bch, dps | ISAR_HIS_PSTREQ, 0, 0, NULL);
	udelay(1000);
}

static void
setup_sart(channel_t *bch) {
	isar_hw_t	*ih = bch->hw;
	u_char dps = SET_DPS(ih->dpath);
	u_char ctrl, param[2];
	
	switch (bch->state) {
		case ISDN_PID_NONE:
			sendmsg(bch, dps | ISAR_HIS_SARTCFG, SMODE_DISABLE, 0,
				NULL);
			break;
		case ISDN_PID_L1_B_64TRANS:
		case ISDN_PID_L2_B_TRANSDTMF:
			sendmsg(bch, dps | ISAR_HIS_SARTCFG, SMODE_BINARY, 2,
				"\0\0");
			break;
		case ISDN_PID_L1_B_64HDLC:
		case ISDN_PID_L1_B_T30FAX:
			param[0] = 0;
			sendmsg(bch, dps | ISAR_HIS_SARTCFG, SMODE_HDLC, 1,
				param);
			break;
		case ISDN_PID_L1_B_MODEM_ASYNC:
			ctrl = SMODE_V14 | SCTRL_HDMC_BOTH;
			param[0] = S_P1_CHS_8;
			param[1] = S_P2_BFT_DEF;
			sendmsg(bch, dps | ISAR_HIS_SARTCFG, ctrl, 2,
				param);
			break;
	}
	udelay(1000);
	sendmsg(bch, dps | ISAR_HIS_BSTREQ, 0, 0, NULL);
	udelay(1000);
}

static void
setup_iom2(channel_t *bch) {
	isar_hw_t	*ih = bch->hw;
	u_char dps = SET_DPS(ih->dpath);
	u_char cmsb = IOM_CTRL_ENA, msg[5] = {IOM_P1_TXD,0,0,0,0};
	
	if (bch->channel)
		msg[1] = msg[3] = 1;
	switch (bch->state) {
		case ISDN_PID_NONE:
			cmsb = 0;
			/* dummy slot */
			msg[1] = msg[3] = ih->dpath + 2;
			break;
		case ISDN_PID_L1_B_64TRANS:
		case ISDN_PID_L1_B_64HDLC:
			break;
		case ISDN_PID_L1_B_MODEM_ASYNC:
		case ISDN_PID_L1_B_T30FAX:
			cmsb |= IOM_CTRL_RCV;
		case ISDN_PID_L2_B_TRANSDTMF:
			if (test_bit(FLG_DTMFSEND, &bch->Flags))
				cmsb |= IOM_CTRL_RCV;
			cmsb |= IOM_CTRL_ALAW;
			break;
	}
	sendmsg(bch, dps | ISAR_HIS_IOM2CFG, cmsb, 5, msg);
	udelay(1000);
	sendmsg(bch, dps | ISAR_HIS_IOM2REQ, 0, 0, NULL);
	udelay(1000);
}

static int
modeisar(channel_t *bch, int channel, u_int bprotocol, u_char *param)
{
	isar_hw_t	*ih = bch->hw;

	/* Here we are selecting the best datapath for requested protocol */
	if(bch->state == ISDN_PID_NONE) { /* New Setup */
		bch->channel = channel;
		switch (bprotocol) {
			case ISDN_PID_NONE: /* init */
				if (!ih->dpath)
					/* no init for dpath 0 */
					return(0);
				break;
			case ISDN_PID_L1_B_64TRANS:
			case ISDN_PID_L1_B_64HDLC:
				/* best is datapath 2 */
				if (!test_and_set_bit(ISAR_DP2_USE, &ih->reg->Flags))
					ih->dpath = 2;
				else if (!test_and_set_bit(ISAR_DP1_USE,
					&ih->reg->Flags))
					ih->dpath = 1;
				else {
					printk(KERN_WARNING"isar modeisar both pathes in use\n");
					return(-EINVAL);
				}
				break;
			case ISDN_PID_L1_B_MODEM_ASYNC:
			case ISDN_PID_L1_B_T30FAX:
			case ISDN_PID_L2_B_TRANSDTMF:
				/* only datapath 1 */
				if (!test_and_set_bit(ISAR_DP1_USE,
					&ih->reg->Flags))
					ih->dpath = 1;
				else {
					printk(KERN_WARNING"isar modeisar analog funktions only with DP1\n");
					mISDN_debugprint(&bch->inst, "isar modeisar analog funktions only with DP1");
					return(-EBUSY);
				}
				break;
		}
	}
	if (bch->debug & L1_DEB_HSCX)
		mISDN_debugprint(&bch->inst, "isar dp%d protocol %x->%x ichan %d",
			ih->dpath, bch->state, bprotocol, channel);
	bch->state = bprotocol;
	setup_pump(bch);
	setup_iom2(bch);
	setup_sart(bch);
	if (bch->state == ISDN_PID_NONE) {
		/* Clear resources */
		if (ih->dpath == 1)
			test_and_clear_bit(ISAR_DP1_USE, &ih->reg->Flags);
		else if (ih->dpath == 2)
			test_and_clear_bit(ISAR_DP2_USE, &ih->reg->Flags);
		ih->dpath = 0;
	}
	return(0);
}

static void
isar_pump_cmd(channel_t *bch, int cmd, u_char para)
{
	isar_hw_t	*ih = bch->hw;
	u_char		dps = SET_DPS(ih->dpath);
	u_char		ctrl = 0, nom = 0, p1 = 0;

	if (bch->debug & L1_DEB_HSCX)
		mISDN_debugprint(&bch->inst, "isar_pump_cmd %x/%x state(%x)",
					cmd, para, ih->state);
	switch(cmd) {
		case HW_MOD_FTM:
			if (ih->state == STFAX_READY) {
				p1 = para;
				ctrl = PCTRL_CMD_FTM;
				nom = 1;
				ih->state = STFAX_LINE;
				ih->cmd = ctrl;
				ih->mod = para;
				ih->newmod = 0;
				ih->newcmd = 0;
				ih->try_mod = 3;
			} else if ((ih->state == STFAX_ACTIV) &&
				(ih->cmd == PCTRL_CMD_FTM) &&
				(ih->mod == para)) {
				deliver_status(bch, HW_MOD_CONNECT);
			} else {
				ih->newmod = para;
				ih->newcmd = PCTRL_CMD_FTM;
				nom = 0;
				ctrl = PCTRL_CMD_ESC;
				ih->state = STFAX_ESCAPE;
			}
			break;
		case HW_MOD_FTH:
			if (ih->state == STFAX_READY) {
				p1 = para;
				ctrl = PCTRL_CMD_FTH;
				nom = 1;
				ih->state = STFAX_LINE;
				ih->cmd = ctrl;
				ih->mod = para;
				ih->newmod = 0;
				ih->newcmd = 0;
				ih->try_mod = 3;
			} else if ((ih->state == STFAX_ACTIV) &&
				(ih->cmd == PCTRL_CMD_FTH) &&
				(ih->mod == para)) {
				deliver_status(bch, HW_MOD_CONNECT);
			} else {
				ih->newmod = para;
				ih->newcmd = PCTRL_CMD_FTH;
				nom = 0;
				ctrl = PCTRL_CMD_ESC;
				ih->state = STFAX_ESCAPE;
			}
			break;
		case HW_MOD_FRM:
			if (ih->state == STFAX_READY) {
				p1 = para;
				ctrl = PCTRL_CMD_FRM;
				nom = 1;
				ih->state = STFAX_LINE;
				ih->cmd = ctrl;
				ih->mod = para;
				ih->newmod = 0;
				ih->newcmd = 0;
				ih->try_mod = 3;
			} else if ((ih->state == STFAX_ACTIV) &&
				(ih->cmd == PCTRL_CMD_FRM) &&
				(ih->mod == para)) {
				deliver_status(bch, HW_MOD_CONNECT);
			} else {
				ih->newmod = para;
				ih->newcmd = PCTRL_CMD_FRM;
				nom = 0;
				ctrl = PCTRL_CMD_ESC;
				ih->state = STFAX_ESCAPE;
			}
			break;
		case HW_MOD_FRH:
			if (ih->state == STFAX_READY) {
				p1 = para;
				ctrl = PCTRL_CMD_FRH;
				nom = 1;
				ih->state = STFAX_LINE;
				ih->cmd = ctrl;
				ih->mod = para;
				ih->newmod = 0;
				ih->newcmd = 0;
				ih->try_mod = 3;
			} else if ((ih->state == STFAX_ACTIV) &&
				(ih->cmd == PCTRL_CMD_FRH) &&
				(ih->mod == para)) {
				deliver_status(bch, HW_MOD_CONNECT);
			} else {
				ih->newmod = para;
				ih->newcmd = PCTRL_CMD_FRH;
				nom = 0;
				ctrl = PCTRL_CMD_ESC;
				ih->state = STFAX_ESCAPE;
			}
			break;
		case PCTRL_CMD_TDTMF:
			p1 = para;
			nom = 1;
			ctrl = PCTRL_CMD_TDTMF;
			break;
	}
	if (ctrl)
		sendmsg(bch, dps | ISAR_HIS_PUMPCTRL, ctrl, nom, &p1);
}

void
isar_setup(channel_t *bch)
{
	u_char msg;
	int i;
	
	/* Dpath 1, 2 */
	msg = 61;
	for (i=0; i<2; i++) {
		isar_hw_t	*ih = bch[i].hw;
		/* Buffer Config */
		sendmsg(bch, (i ? ISAR_HIS_DPS2 : ISAR_HIS_DPS1) |
			ISAR_HIS_P12CFG, 4, 1, &msg);
		ih->mml = msg;
		bch[i].state = 0;
		ih->dpath = i + 1;
		modeisar(&bch[i], i, 0, NULL);
	}
}

int
isar_down(mISDNinstance_t *inst, struct sk_buff *skb)
{
	channel_t	*bch = container_of(inst, channel_t, inst);
	int		ret = 0;
	mISDN_head_t	*hh = mISDN_HEAD_P(skb);
	u_long		flags;

	if ((hh->prim == PH_DATA_REQ) ||
		(hh->prim == (DL_DATA | REQUEST))) {
		spin_lock_irqsave(inst->hwlock, flags);
		ret = channel_senddata(bch, hh->dinfo, skb);
		if (ret > 0) { /* direct TX */
			isar_fill_fifo(bch);
			ret = 0;
		}
		spin_unlock_irqrestore(inst->hwlock, flags);
		return(ret);
	}
	if ((hh->prim == (PH_ACTIVATE | REQUEST)) ||
		(hh->prim == (DL_ESTABLISH  | REQUEST))) {
		if (!test_and_set_bit(FLG_ACTIVE, &bch->Flags)) {
			u_int	bp = bch->inst.pid.protocol[1];

			if (bch->inst.pid.global == 1)
				test_and_set_bit(FLG_ORIGIN, &bch->Flags);
			if ((bp == ISDN_PID_L1_B_64TRANS) &&
				(bch->inst.pid.protocol[2] == ISDN_PID_L2_B_TRANSDTMF))
				bp = ISDN_PID_L2_B_TRANSDTMF;
			spin_lock_irqsave(inst->hwlock, flags);
			ret = modeisar(bch, bch->channel, bp, NULL);
			spin_unlock_irqrestore(inst->hwlock, flags);
		}
		skb_trim(skb, 0);
		return(mISDN_queueup_newhead(inst, 0, hh->prim | CONFIRM, ret, skb));
	} else if ((hh->prim == (PH_DEACTIVATE | REQUEST)) ||
		(hh->prim == (DL_RELEASE | REQUEST)) ||
		((hh->prim == (PH_CONTROL | REQUEST) && (hh->dinfo == HW_DEACTIVATE)))) {
		spin_lock_irqsave(inst->hwlock, flags);
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
		test_and_clear_bit(FLG_TX_BUSY, &bch->Flags);
		test_and_clear_bit(FLG_L2DATA, &bch->Flags);
		modeisar(bch, bch->channel, 0, NULL);
		test_and_clear_bit(FLG_ACTIVE, &bch->Flags);
		spin_unlock_irqrestore(inst->hwlock, flags);
		skb_trim(skb, 0);
		if (hh->prim != (PH_CONTROL | REQUEST))
			if (!mISDN_queueup_newhead(inst, 0, hh->prim | CONFIRM, 0, skb))
				return(0);
	} else if (hh->prim == (PH_CONTROL | REQUEST)) {
		int  *val;
		int  len;

		val = (int *)skb->data;
		if (bch->debug & L1_DEB_HSCX)
			mISDN_debugprint(&bch->inst, "PH_CONTROL | REQUEST %x/%x",
					hh->dinfo, *val);
		if ((hh->dinfo == 0) && ((*val & ~DTMF_TONE_MASK) == DTMF_TONE_VAL)) {
			if (bch->state == ISDN_PID_L2_B_TRANSDTMF) {
				char tt = *val & DTMF_TONE_MASK;
				
				if (tt == '*')
					tt = 0x1e;
				else if (tt == '#')
					tt = 0x1f;
				else if (tt > '9')
					tt -= 7;
				tt &= 0x1f;
				spin_lock_irqsave(inst->hwlock, flags);
				isar_pump_cmd(bch, PCTRL_CMD_TDTMF, tt);
				spin_unlock_irqrestore(inst->hwlock, flags);
				skb_trim(skb, 0);
				if (!mISDN_queueup_newhead(inst, 0, PH_CONTROL | CONFIRM, 0, skb))
					return(0);
			} else {
				printk(KERN_WARNING "isar_down TOUCH_TONE_SEND wrong protocol %x\n",
					bch->state);
				return(-EINVAL);
			}
		} else if ((hh->dinfo == HW_MOD_FRM) || (hh->dinfo == HW_MOD_FRH) ||
			(hh->dinfo == HW_MOD_FTM) || (hh->dinfo == HW_MOD_FTH)) {
			u_int i;

			for (i=0; i<FAXMODCNT; i++)
				if (faxmodulation[i] == *val)
					break;
			if ((FAXMODCNT > i) && test_bit(FLG_INITIALIZED, &bch->Flags)) {
				printk(KERN_WARNING "isar: new mod\n");
				isar_pump_cmd(bch, hh->dinfo, *val);
				ret = 0;
			} else {
				int_errtxt("wrong modulation");
				/* wrong modulation or not activ */
				// TODO
				ret = -EINVAL;
			}
		} else if (hh->dinfo == HW_MOD_LASTDATA) {
			test_and_set_bit(FLG_DLEETX, &bch->Flags);
		} else if (hh->dinfo == HW_FIRM_START) {
			firmwaresize = *val;
			if (!(firmware = vmalloc(firmwaresize))) {
				firmwaresize = 0;
				return(-ENOMEM);
			}
			fw_p = firmware;
			skb_trim(skb, 0);
			if(!mISDN_queueup_newhead(inst, 0, PH_CONTROL | CONFIRM, 0, skb))
				return(0);
		} else if (hh->dinfo == HW_FIRM_DATA) {
			len = *val++;
			if (!fw_p)
				return(-EINVAL);
			memcpy(fw_p, val, len);
			fw_p += len;
			skb_trim(skb, 0);
			if(!mISDN_queueup_newhead(inst, 0, PH_CONTROL | CONFIRM, 0, skb))
				return(0);
		} else if (hh->dinfo == HW_FIRM_END) {
			if (!fw_p)
				return(-EINVAL);
			len = (fw_p - firmware) & 0xffffffff;
			if (len == firmwaresize)
				ret = isar_load_firmware(bch, firmware, firmwaresize);
			else {
				printk(KERN_WARNING "wrong firmware size %d/%d\n",
					len, firmwaresize);
				ret = -EINVAL;
			}
			vfree(firmware);
			fw_p = firmware = NULL;
			firmwaresize = 0;
			skb_trim(skb, 0);
			if(!mISDN_queueup_newhead(inst, 0, PH_CONTROL | CONFIRM, 0, skb))
				return(0);
		} else {
			printk(KERN_WARNING "isar_down unknown (PH_CONTROL | REQUEST) %x\n",
				hh->dinfo);
			ret = -EINVAL;
		}
	} else {
		printk(KERN_WARNING "isar_down unknown prim(%x)\n", hh->prim);
		ret = -EINVAL;
	}
	if (!ret)
		dev_kfree_skb(skb);
	return(ret);
}

void
free_isar(channel_t *bch)
{
	isar_hw_t *ih = bch->hw;

	modeisar(bch, bch->channel, 0, NULL);
	del_timer(&ih->ftimer);
	test_and_clear_bit(FLG_INITIALIZED, &bch->Flags);
}


int init_isar(channel_t *bch)
{
	isar_hw_t *ih = bch->hw;

	printk(KERN_INFO "mISDN: ISAR driver Rev. %s\n", mISDN_getrev(ISAR_revision));
	ih->ftimer.function = (void *) ftimer_handler;
	ih->ftimer.data = (long) bch;
	init_timer(&ih->ftimer);
	test_and_set_bit(FLG_INITIALIZED, &bch->Flags);
	return (0);
}
