/* $Id: l3_ndss1.c,v 1.17 2008-12-18 20:32:43 armin Exp $
 *
 * EURO/DSS1 D-channel protocol NT
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 * Adapted for NT mode    Armin Schindler (armin@melware.de)
 *
 *		This file is (c) under GNU  PUBLIC LICENSE
 *		For changes and modifications please read
 *		../../../Documentation/isdn/mISDN.cert
 *
 * Thanks to    Jan den Ouden
 *              Fritz Elfert
 *
 */

#include <linux/module.h>

#include "core.h"
#include "layer3_net.h"
#include "helper.h"
#include "debug.h"
#include "dss1.h"

#define l3_debug(x...) l3_debug_net(x)

static int debug = 0;
static mISDNobject_t n_dss1;

static const char *dss1_revision = "$Revision: 1.17 $";

struct _l3_msg {
	int mt;
	struct sk_buff *skb;
};

static int l3_msg(layer3_t *l3, u_int pr, int dinfo, void *arg);

static int comp_required[] = {1,2,3,5,6,7,9,10,11,14,15,-1};

static void doprintdata(char *buf, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		printk("%02x ", buf[i]);
	}
}

static void show_ddata(char *prefix, struct sk_buff *skb)
{
	Q931_info_t	*qi;

	if (!(debug & 0x4))
		return;

	if (!skb) {
		printk(KERN_DEBUG "%s no skb.\n", prefix); 
		return;
	}
	if (skb->len <= L3_EXTRA_SIZE) {
		printk(KERN_DEBUG "%s no qi: ", prefix);
		doprintdata(skb->data, skb->len);
		printk("\n");
		return;
	}
	qi = (Q931_info_t *)skb->data;
	printk(KERN_DEBUG "%s ", prefix);
	if (qi->channel_id.off) {
		printk("CHANNEL_ID-off=%d ", qi->channel_id.off);
	}
	doprintdata(skb->data + L3_EXTRA_SIZE, skb->len - L3_EXTRA_SIZE);
	printk("\n");
}

static int
parseQ931(struct sk_buff *skb) {
	Q931_info_t	*qi;
	int		l, codeset, maincodeset;
	int		len, iep, pos = 0, cnt = 0, eidx = -1;
	u16		cr;
	ie_info_t	*ie, *old;
	u_char		t, *p = skb->data;

	if (skb->len < 3)
		return(-1);
	p++;
	l = (*p++) & 0xf;
	if (l>2)
		return(-2);
	if (l)
		cr = *p++;
	else
		cr = 0;
	if (l == 2) {
		cr <<= 8;
		cr |= *p++;
	} else if (l == 1)
		if (cr & 0x80) {
			cr |= 0x8000;
			cr &= 0xFF7F;
		}
	t = *p;
	if ((u_long)p & 1)
		pos = 1;
	else
		pos = 0;
	skb_pull(skb, (p - skb->data) - pos);
	len = skb->len;
	p = skb->data;
	if (skb_headroom(skb) < (int)L3_EXTRA_SIZE) {
		int_error();
		return(-3);
	}
	qi = (Q931_info_t *)skb_push(skb, L3_EXTRA_SIZE);
	mISDN_initQ931_info(qi);
	qi->type = t;
	qi->crlen = l;
	qi->cr = cr;
	pos++;
	codeset = maincodeset = 0;
	ie = &qi->bearer_capability;
	while (pos < len) {

		
		if ((p[pos] & 0xf0) == 0x90) {
			codeset = p[pos] & 0x07;
			if (!(p[pos] & 0x08))
				maincodeset = codeset;
			if (eidx >= 0) {
				qi->ext[eidx].cs.len = pos - qi->ext[eidx].ie.off;
				eidx = -1;
			}
			pos++;
			continue;
		}
		if (codeset == 0) {
			if (p[pos] & 0x80) { /* single octett IE */
				if (p[pos] == IE_MORE_DATA)
					qi->more_data.off = pos;
				else if (p[pos] == IE_COMPLETE) {
					qi->sending_complete.off = pos;
				}
				else if ((p[pos] & 0xf0) == IE_CONGESTION)
					qi->congestion_level.off = pos;
				else {
					printk("parseQ931: Unknown Single Oct IE [%x]\n",p[pos]);
				}
				cnt++;
				pos++;
			} else {
				t = p[pos];
				iep = mISDN_l3_ie2pos(t);
				if ((pos+1) >= len)
					return(-4);
				l = p[pos+1];
				if ((pos+l+1) >= len)
					return(-5);
				if (iep>=0) {
					if (!ie[iep].off) { /* IE not detected before */
						ie[iep].off = pos;
					} else { /* IE is repeated */
						old = &ie[iep];
						if (old->repeated)
							old = mISDN_get_last_repeated_ie(qi, old);
						if (!old) {
							int_error();
							return(-6);
						}
						eidx = mISDN_get_free_ext_ie(qi);
						if (eidx < 0) {
							int_error();
							return(-7);
						}
						old->ridx = eidx;
						old->repeated = 1;
						qi->ext[eidx].ie.off = pos;
						qi->ext[eidx].v.codeset = 0;
						qi->ext[eidx].v.val = t;
						eidx = -1;
					}
				} else {
					int i;
					for (i=0; comp_required[i] > 0; i++) {
						if ( p[pos] == comp_required[i] && l==1 ) {
							qi->comprehension_required.off = pos;
						} 
					}
					if (!qi->comprehension_required.off)
						printk(" ie not handled ie [%x] l [%x]\n", p[pos],l);
				}
				pos += l + 2;
				cnt++;
			}
		} else { /* codeset != 0 */
			if (eidx < 0) {
				eidx = mISDN_get_free_ext_ie(qi);
				if (eidx < 0) {
					int_error();
					return(-8);
				}
				qi->ext[eidx].cs.codeset = codeset;
				qi->ext[eidx].ie.off = pos;
				qi->ext[eidx].ie.cs_flg = 1;
				if (codeset == maincodeset) { /* locked shift */
					qi->ext[eidx].cs.locked = 1;
				}
			}
			if (p[pos] & 0x80) { /* single octett IE */
				cnt++;
				pos++;
			} else {
				if ((pos+1) >= len)
					return(-4);
				l = p[pos+1];
				if ((pos+l+1) >= len)
					return(-5);
				pos += l + 2;
				cnt++;
			}
			if (qi->ext[eidx].cs.locked == 0) {/* single IE codeset shift */
				qi->ext[eidx].cs.len = pos - qi->ext[eidx].ie.off;
				eidx = -1;
			}
		}
		codeset = maincodeset;
	}
	if (eidx >= 0)
		qi->ext[eidx].cs.len = pos - qi->ext[eidx].ie.off;
	return(cnt);
}

static int
getcallref(u_char *p)
{
	int l, cr = 0;

	p++;            /* prot discr */
	l = 0xf & *p++;     /* callref length */
	if (l > 2)      /* wrong callref only 1 or 2 octet*/
		return(-2);
	if (!l)         /* dummy CallRef */
		return(-1);
	if (l == 1) {       /* BRI */
		cr = *p & 0x7f;
		cr += (*p & 0x80) << 8;
	} else {        /* PRI */
		cr = *p++ << 8;
		cr += *p;
	}
	return (cr);
}

static l3_process_t *
find_proc4l4id(struct list_head *listhead, int id)
{
	l3_process_t   *p;
	l3_process_t   *cp;

	if (debug & 0x2)
		printk(KERN_DEBUG "find_proc4l4id NT: for id %x\n", id);

	list_for_each_entry(p, listhead, list) {
		if (p->l4id == id)
			return p;
		if (!list_empty(&p->childlist)) {
			cp = find_proc4l4id(&p->childlist, id);
			if (cp)
				return(cp);
		}
	}
	return NULL;
}

static l3_process_t *
find_proc(struct list_head *listhead, int ces, int cr)
{
	l3_process_t   *p;
	l3_process_t   *cp;

	if (debug & 0x2)
		printk(KERN_DEBUG "find_proc NT: for ces=%x cr=%x\n", ces, cr);

	list_for_each_entry(p, listhead, list) {
		if ((p->ces == ces) && (p->callref == cr))
			return p;
		if (!list_empty(&p->childlist)) {
			cp = find_proc(&p->childlist, ces, cr);
			if (cp)
				return(cp);
		}
		if (((p->ces & 0xffffff00) == 0xff00) && (p->callref == cr))
			return p;
	}
	return NULL;
}

static l3_process_t *
create_proc(layer3_t *l3, int ces, int cr, l3_process_t *master)
{
	l3_process_t  *l3p;

	l3p = kzalloc(sizeof(l3_process_t), GFP_ATOMIC);
	if (l3p) {
		INIT_LIST_HEAD(&l3p->childlist);
		l3p->l3 = l3;
		l3p->ces = ces;
		l3p->callref = cr;
		l3p->master = master;
		L3InitTimer_net(l3p, &l3p->timer1);
		L3InitTimer_net(l3p, &l3p->timer2);
		if (master) {
			list_add_tail(&l3p->list, &master->childlist);
		}
		if (debug & 0x2)
			l3_debug(l3, "create_proc NT: ces=%x cr=%x master=%p",
				ces, cr, master);
	} else {
		printk(KERN_ERR "mISDN ndss1 can't get memory for cr %d\n", cr);
	}
	return(l3p);
}

static struct sk_buff *prep_l3data_msg(u_int prim, int dinfo, int ssize, int dsize, struct sk_buff *old)
{
	struct sk_buff *new;

	if (!old) {
		new = alloc_skb(ssize + dsize, GFP_ATOMIC);
		if (!new) {
			return(NULL);
		}
		memset(skb_put(new, ssize), 0, ssize);
	} else {
		new = skb_copy(old, GFP_ATOMIC);
		if (!new) {
			return(NULL);
		}
	}
	mISDN_sethead(prim, dinfo, new);
	return(new);
}

static void
send_squeue(layer3_t *l3)
{
	struct sk_buff *skb;

	while ((skb = skb_dequeue(&l3->squeue))) {
		if (mISDN_queue_down(&l3->inst, 0, skb))
			dev_kfree_skb(skb);
	}
}

static int
mISDN_l3upu(l3_process_t *l3p, u_int prim, struct sk_buff *skb)
{
	layer3_t *l3;
	int err = -EINVAL;

	if (!l3p)
		return (-EINVAL);
	l3 = l3p->l3;
	if (!skb) {
		err = mISDN_queue_data(&l3->inst, FLG_MSG_UP, prim,
		    l3p->l4id, 0, NULL, 0);
	} else {
		show_ddata("toL4:", skb);
		err = mISDN_queueup_newhead(&l3->inst, 0, prim, l3p->l4id, skb);
	}
	return (err);
}

static int
mISDN_l3up(l3_process_t *l3p, struct sk_buff *skb)
{
	layer3_t *l3;
	int err = -EINVAL;

	if (!l3p)
		return (-EINVAL);
	l3 = l3p->l3;
	if (!skb) {
		printk(KERN_ERR "mISDN_l3up NT: no skb!\n");
	} else {
		show_ddata("toL4:", skb);
		err = mISDN_queue_up(&l3->inst, 0, skb);
	}
	return (err);
}

static int
l3down(layer3_t *l3, u_int prim, int dinfo, struct sk_buff *skb)
{
	int err = -EINVAL;

	if (!skb) {
		err = mISDN_queue_data(&l3->inst, FLG_MSG_DOWN, prim, dinfo, 0, NULL, 0);
	} else {
		if ((debug & 0x4))
			printk(KERN_DEBUG "l3down: prim=0x%x dinfo=0x%x\n", prim, dinfo);
		show_ddata("toL2:", skb);
		err = mISDN_queuedown_newhead(&l3->inst, 0, prim, dinfo, skb);
	}
	return (err);
}

static int
calc_msg_len(Q931_info_t *qi)
{
	int		i, cnt = 0;
	u_char		*buf = (u_char *)qi;
	ie_info_t	*ie;

	buf += L3_EXTRA_SIZE;
	if (qi->more_data.off)
		cnt++;
	if (qi->sending_complete.off)
		cnt++;
	if (qi->congestion_level.off)
		cnt++;
	ie = &qi->bearer_capability;
	while (ie <= &qi->comprehension_required) {
		if (ie->off)
			cnt += buf[ie->off + 1] + 2;
		ie++;
	}
	for (i = 0; i < 8; i++) {
		if (qi->ext[i].ie.off) {
			if (qi->ext[i].ie.cs_flg == 1) { /* other codset info chunk */
				cnt++; /* codeset shift IE */
				cnt += qi->ext[i].cs.len;
			} else { /* repeated IE */
				cnt += buf[qi->ext[i].ie.off + 1] + 2;
			}
		}
	}
	return(cnt);
}

static int
compose_msg(struct sk_buff *skb, Q931_info_t *qi)
{
	int		i, l, ri;
	u_char		*p, *buf = (u_char *)qi;
	ie_info_t	*ie;

	buf += L3_EXTRA_SIZE;

	if (qi->more_data.off) {
		p = skb_put(skb, 1);
		*p = buf[qi->more_data.off];
	}
	if (qi->sending_complete.off) {
		p = skb_put(skb, 1);
		*p = buf[qi->sending_complete.off];
	}
	if (qi->congestion_level.off) {
		p = skb_put(skb, 1);
		*p = buf[qi->congestion_level.off];
	}
	ie = &qi->bearer_capability;
	for (i=0; i<33; i++) {
		if (ie[i].off) {
			l = buf[ie[i].off + 1] +1;
			p = skb_put(skb, l + 1);
			*p++ = mISDN_l3_pos2ie(i);
			memcpy(p, &buf[ie[i].off + 1], l);
			if (ie[i].repeated) {
				ri = ie[i].ridx;
				while(ri >= 0) {
					l = buf[qi->ext[ri].ie.off + 1] +1;
					p = skb_put(skb, l + 1);
					if (mISDN_l3_pos2ie(i) != qi->ext[ri].v.val)
						int_error();
					*p++ = qi->ext[ri].v.val;
					memcpy(p, &buf[qi->ext[ri].ie.off + 1], l);
					if (qi->ext[ri].ie.repeated)
						ri = qi->ext[ri].ie.ridx;
					else
						ri = -1;
				}
			}
		}
	}
	for (i=0; i<8; i++) {
		/* handle other codeset elements */
		if (qi->ext[i].ie.cs_flg == 1) {
			p = skb_put(skb, 1); /* shift codeset IE */
			if (qi->ext[i].cs.locked == 1)
				*p = 0x90 | qi->ext[i].cs.codeset;
			else /* non-locking shift */
				*p = 0x98 | qi->ext[i].cs.codeset;
			p = skb_put(skb, qi->ext[i].cs.len);
			memcpy(p, &buf[qi->ext[i].ie.off], qi->ext[i].cs.len);
		}
	}
	return(0);
}

static struct sk_buff
*MsgStart(l3_process_t *pc, u_char mt, int len)
{
	struct sk_buff	*skb;
	int		lx = 4;
	u_char		*p;

	if (test_bit(FLG_CRLEN2, &pc->l3->Flag))
		lx++;
	if (pc->callref == -1) /* dummy cr */
		lx = 3;
	if (!(skb = alloc_stack_skb(len + lx, pc->l3->down_headerlen)))
		return(NULL);
	p = skb_put(skb, lx);
	*p++ = 8;
	if (lx == 3)
		*p++ = 0;
	else if (lx == 5) {
		*p++ = 2;
		*p++ = (pc->callref >> 8)  ^ 0x80;
		*p++ = pc->callref & 0xff;
	} else {
		*p++ = 1;
		*p = pc->callref & 0x7f;
		if (!(pc->callref & 0x8000))
			*p |= 0x80;
		p++;
	}
	*p = mt;
	return(skb);
}

static struct sk_buff *MakeMsgMT(l3_process_t *pc, struct sk_buff *skb, u_char mt)
{
	int	l;
	struct sk_buff *nskb;
	Q931_info_t *qi;

	if (!skb)
		return NULL;

	qi = (Q931_info_t *)skb->data;
	l = calc_msg_len(qi);
	if (!(nskb = MsgStart(pc, mt, l))) {
		return NULL;
	}
	if (l)
		compose_msg(nskb, qi);
	return(nskb);
}

static struct sk_buff *MakeMsg(l3_process_t *pc, struct sk_buff *skb)
{
	int	l;
	struct sk_buff *nskb;
	Q931_info_t *qi;

	if (!skb)
		return NULL;

	qi = (Q931_info_t *)skb->data;
	l = calc_msg_len(qi);
	if (!(nskb = MsgStart(pc, qi->type, l))) {
		return NULL;
	}
	if (l)
		compose_msg(nskb, qi);
	return(nskb);
}

static void
newl3state(l3_process_t *pc, int state)
{
	if (pc->l3 && pc->l3->debug & L3_DEB_STATE)
		l3_debug(pc->l3, "newstate cr %d %d%s --> %d%s", 
			 pc->callref & 0x7FFF,
			 pc->state, pc->master ? "i" : "",
			 state, pc->master ? "i" : "");
	pc->state = state;
}

static void copy_to_obuf(l3_process_t *pc, struct sk_buff *skb)
{
	if (skb) {
		pc->obuflen = skb->len;
		if (skb->len) {
			memcpy(&pc->obuf[0], skb->data, skb->len);
		}
	} else {
		pc->obuflen = 0;
	}
}

static int SendMsgDirect(l3_process_t *pc, struct sk_buff *skb, int state)
{
	int ret;

	if (state != -1)
		newl3state(pc, state);
	copy_to_obuf(pc, skb);
	if ((ret = l3_msg(pc->l3, DL_DATA | REQUEST, pc->ces, skb)))
		kfree_skb(skb);
	return(ret);
}

static int SendMsg(l3_process_t *pc, struct sk_buff *skb, int state)
{
	struct sk_buff *nskb;

	if (!(nskb = MakeMsg(pc, skb)))
		return -EINVAL;

	return(SendMsgDirect(pc, nskb, state));
}

#if 0
static int ie_ALERTING[] = {IE_BEARER, IE_CHANNEL_ID | IE_MANDATORY_1,
		IE_FACILITY, IE_PROGRESS, IE_DISPLAY, IE_SIGNAL, IE_REDIR_DN,
		IE_HLC, IE_USER_USER, -1};
static int ie_CALL_PROCEEDING[] = {IE_BEARER, IE_CHANNEL_ID | IE_MANDATORY_1,
		IE_FACILITY, IE_PROGRESS, IE_DISPLAY, IE_REDIR_DN, IE_HLC, -1};
static int ie_CONNECT[] = {IE_BEARER, IE_CHANNEL_ID | IE_MANDATORY_1,
		IE_FACILITY, IE_PROGRESS, IE_DISPLAY, IE_DATE, IE_SIGNAL,
		IE_CONNECT_PN, IE_CONNECT_SUB, IE_LLC, IE_HLC, IE_USER_USER, -1};
static int ie_CONNECT_ACKNOWLEDGE[] = {IE_CHANNEL_ID, IE_DISPLAY, IE_SIGNAL, -1};
static int ie_DISCONNECT[] = {IE_CAUSE | IE_MANDATORY, IE_FACILITY,
		IE_PROGRESS, IE_DISPLAY, IE_SIGNAL, IE_USER_USER, -1};
static int ie_INFORMATION[] = {IE_COMPLETE, IE_DISPLAY, IE_KEYPAD, IE_SIGNAL,
		IE_CALLED_PN, -1};
static int ie_NOTIFY[] = {IE_BEARER, IE_NOTIFY | IE_MANDATORY, IE_DISPLAY, IE_REDIR_DN, -1};
static int ie_PROGRESS[] = {IE_BEARER, IE_CAUSE, IE_FACILITY, IE_PROGRESS |
		IE_MANDATORY, IE_DISPLAY, IE_HLC, IE_USER_USER, -1};
static int ie_RELEASE[] = {IE_CAUSE | IE_MANDATORY_1, IE_FACILITY, IE_DISPLAY,
		IE_SIGNAL, IE_USER_USER, -1};
/* a RELEASE_COMPLETE with errors don't require special actions
static int ie_RELEASE_COMPLETE[] = {IE_CAUSE | IE_MANDATORY_1, IE_FACILITY,
		IE_DISPLAY, IE_SIGNAL, IE_USER_USER, -1};
*/
static int ie_RESUME_ACKNOWLEDGE[] = {IE_CHANNEL_ID| IE_MANDATORY, IE_FACILITY,
		IE_DISPLAY, -1};
static int ie_RESUME_REJECT[] = {IE_CAUSE | IE_MANDATORY, IE_DISPLAY, -1};
#endif
static int ie_SETUP[] = {IE_COMPLETE, IE_BEARER  | IE_MANDATORY,
		IE_CHANNEL_ID| IE_MANDATORY, IE_FACILITY, IE_PROGRESS,
		IE_NET_FAC, IE_DISPLAY, IE_KEYPAD, IE_SIGNAL, IE_CALLING_PN,
		IE_CALLING_SUB, IE_CALLED_PN, IE_CALLED_SUB, IE_REDIR_NR,
		IE_LLC, IE_HLC, IE_USER_USER, -1};
#if 0
static int ie_SETUP_ACKNOWLEDGE[] = {IE_CHANNEL_ID | IE_MANDATORY, IE_FACILITY,
		IE_PROGRESS, IE_DISPLAY, IE_SIGNAL, -1};
static int ie_STATUS[] = {IE_CAUSE | IE_MANDATORY, IE_CALL_STATE |
		IE_MANDATORY, IE_DISPLAY, -1};
static int ie_STATUS_ENQUIRY[] = {IE_DISPLAY, -1};
static int ie_SUSPEND_ACKNOWLEDGE[] = {IE_FACILITY, IE_DISPLAY, -1};
static int ie_SUSPEND_REJECT[] = {IE_CAUSE | IE_MANDATORY, IE_DISPLAY, -1};
static int ie_HOLD[] = {IE_DISPLAY, -1};
static int ie_HOLD_ACKNOWLEDGE[] = {IE_DISPLAY, -1};
static int ie_HOLD_REJECT[] = {IE_CAUSE | IE_MANDATORY, IE_DISPLAY, -1};
static int ie_RETRIEVE[] = {IE_CHANNEL_ID| IE_MANDATORY, IE_DISPLAY, -1};
static int ie_RETRIEVE_ACKNOWLEDGE[] = {IE_CHANNEL_ID| IE_MANDATORY, IE_DISPLAY, -1};
static int ie_RETRIEVE_REJECT[] = {IE_CAUSE | IE_MANDATORY, IE_DISPLAY, -1};
#endif
/* not used
 * static int ie_CONGESTION_CONTROL[] = {IE_CONGESTION | IE_MANDATORY,
 *		IE_CAUSE | IE_MANDATORY, IE_DISPLAY, -1};
 * static int ie_USER_INFORMATION[] = {IE_MORE_DATA, IE_USER_USER | IE_MANDATORY, -1};
 * static int ie_RESTART[] = {IE_CHANNEL_ID, IE_DISPLAY, IE_RESTART_IND |
 *		IE_MANDATORY, -1};
 */
static int ie_FACILITY[] = {IE_FACILITY | IE_MANDATORY, IE_DISPLAY, -1};

struct ie_len {
	int ie;
	int len;
};

static
struct ie_len max_ie_len[] = {
	{IE_SEGMENT, 4},
	{IE_BEARER, 12},
	{IE_CAUSE, 32},
	{IE_CALL_ID, 10},
	{IE_CALL_STATE, 3},
	{IE_CHANNEL_ID,	34},
	{IE_FACILITY, 255},
	{IE_PROGRESS, 4},
	{IE_NET_FAC, 255},
	{IE_NOTIFY, 255}, /* 3-* Q.932 Section 9 */
	{IE_DISPLAY, 82},
	{IE_DATE, 8},
	{IE_KEYPAD, 34},
	{IE_SIGNAL, 3},
	{IE_INFORATE, 6},
	{IE_E2E_TDELAY, 11},
	{IE_TDELAY_SEL, 5},
	{IE_PACK_BINPARA, 3},
	{IE_PACK_WINSIZE, 4},
	{IE_PACK_SIZE, 4},
	{IE_CUG, 7},
	{IE_REV_CHARGE, 3},
	{IE_CALLING_PN, 24},
	{IE_CALLING_SUB, 23},
	{IE_CALLED_PN, 24},
	{IE_CALLED_SUB, 23},
	{IE_REDIR_NR, 255},
	{IE_REDIR_DN, 255},
	{IE_TRANS_SEL, 255},
	{IE_RESTART_IND, 3},
	{IE_LLC, 18},
	{IE_HLC, 5},
	{IE_USER_USER, 131},
	{-1,0},
};

static int
getmax_ie_len(u_char ie) {
	int i = 0;
	while (max_ie_len[i].ie != -1) {
		if (max_ie_len[i].ie == ie)
			return(max_ie_len[i].len);
		i++;
	}
	return(255);
}

static int
ie_in_set(l3_process_t *pc, u_char ie, int *checklist) {
	int ret = 1;

	while (*checklist != -1) {
		if ((*checklist & 0xff) == ie) {
			if (ie & 0x80)
				return(-ret);
			else
				return(ret);
		}
		ret++;
		checklist++;
	}
	return(0);
}

static int
check_infoelements(l3_process_t *pc, struct sk_buff *skb, int *checklist)
{
	Q931_info_t	*qi = (Q931_info_t *)skb->data;
	int		*cl = checklist;
	u_char		*p, ie;
	ie_info_t	*iep;
	int		i, l, newpos, oldpos;
	int		err_seq = 0, err_len = 0, err_compr = 0, err_ureg = 0;

	p = skb->data;
	p += L3_EXTRA_SIZE;
	iep = &qi->bearer_capability;
	oldpos = -1;

	for (i=0; i<33; i++) {
		if (iep[i].off) {
			ie = mISDN_l3_pos2ie(i);
			if ((newpos = ie_in_set(pc, ie, cl))) {
				if (newpos > 0) {
					if (newpos < oldpos)
						err_seq++;
					else
						oldpos = newpos;
				} else {
					printk(KERN_NOTICE "ie_in_set returned <0  [%d] ie:[%x]\n",newpos,ie);
				}
			} else {
				if (debug) printk(KERN_NOTICE "Found ie in set which we do not support ie [%x]\n",ie);
				if (ie_in_set(pc, ie, comp_required))
					err_compr++;
				else
					err_ureg++;
			}
			l = p[iep[i].off +1];
			if (l > getmax_ie_len(ie))
				err_len++;
		}
	}

	if (qi->comprehension_required.off) {
		if ( ! (p[qi->comprehension_required.off +2] &0xf) ) {
			err_compr++;
		}
	}
	
	if (err_compr | err_ureg | err_len | err_seq) {
		if (pc->l3->debug & L3_DEB_CHECK)
			l3_debug(pc->l3, "check IE MT(%x) %d/%d/%d/%d",
				qi->type, err_compr, err_ureg, err_len, err_seq);
		if (err_compr)
			return(ERR_IE_COMPREHENSION);
		if (err_ureg)
			return(ERR_IE_UNRECOGNIZED);
		if (err_len)
			return(ERR_IE_LENGTH);
		if (err_seq)
			return(ERR_IE_SEQUENCE);
	}
	return(0);
}

static int
l3dss1_message(l3_process_t *pc, unsigned char mt)
{
	struct sk_buff  *skb;
	unsigned char *p;
	int ret;
	int crlen = 1;

	if (test_bit(FLG_CRLEN2, &pc->l3->Flag))
		crlen = 2;

	if (!(skb = alloc_stack_skb(crlen+3, pc->l3->down_headerlen)))
		return(-ENOMEM);
	p = skb_put(skb, crlen+3);
	*p++ = 8;
	*p++ = crlen;
	if (crlen == 2) {
		*p++ = (pc->callref >> 8) ^ 0x80;
		*p++ = pc->callref & 0xff;
	} else {
		*p = pc->callref & 0x7f;
		if (!(pc->callref & 0x8000))
			*p |= 0x80;
		p++;
	}
	*p++ = mt;
	copy_to_obuf(pc, skb);
	if ((ret=l3_msg(pc->l3, DL_DATA | REQUEST, pc->ces, skb)))
		dev_kfree_skb(skb);
	return(ret);
}

#if 0
static unsigned char *
findie(unsigned char *p, int size, unsigned char ie, int wanted_set)
{
	int l, codeset, maincodeset;
	unsigned char *pend = p + size;

	/* skip protocol discriminator, callref and message type */
	p++;
	l = (*p++) & 0xf;
	p += l;
	p++;
	codeset = 0;
	maincodeset = 0;
	/* while there are bytes left... */
	while (p < pend) {
		if ((*p & 0xf0) == 0x90) {
			codeset = *p & 0x07;
			if (!(*p & 0x08))
				maincodeset = codeset;
		}
		if (codeset == wanted_set) {
			if (*p == ie) {
				/* improved length check (Werner Cornelius) */
				if (!(*p & 0x80)) {
					if ((pend - p) < 2)
						return(NULL);
					if (*(p+1) > (pend - (p+2)))
						return(NULL);
					p++; /* points to len */
				}
				return (p);
			} else if ((*p > ie) && !(*p & 0x80))
				return (NULL);
		}
		if (!(*p & 0x80)) {
			p++;
			l = *p;
			p += l;
			codeset = maincodeset;
		}
		p++;
	}
	return (NULL);
}
#endif

static void
l3dss1_message_cause(l3_process_t *pc, unsigned char mt, unsigned char cause)
{
	static struct sk_buff *skb;
	u_char		*p;
	Q931_info_t *qi;

	skb = MsgStart(pc, mt, 4);
	if (!skb)
		return;

	if (cause) {
		qi = (Q931_info_t *)skb->data;
		qi->cause.off = skb->len - L3_EXTRA_SIZE;
		p = skb_put(skb, 4);
		*p++ = IE_CAUSE;
		*p++ = 2;
		*p++ = 0x80 | CAUSE_LOC_PNET_LOCUSER;
		*p++ = 0x80 | cause;
	}
	SendMsgDirect(pc, skb, -1); 
}

static void
l3dss1_status_send(l3_process_t *pc, unsigned char cause)
{
	static struct sk_buff *skb;
	u_char		*p;
	Q931_info_t *qi;

	skb = MsgStart(pc, MT_STATUS, 4);
	qi = (Q931_info_t *)skb->data;

	qi->cause.off = skb->len - L3_EXTRA_SIZE;
	p = skb_put(skb, 7);

	*p++ = IE_CAUSE;
	*p++ = 2;
	*p++ = 0x80 | CAUSE_LOC_USER;
	*p++ = 0x80 | cause;

	qi->call_state.off = qi->cause.off + 4;
	*p++ = IE_CALL_STATE;
	*p++ = 1;
	*p++ = pc->state & 0x3f;

	SendMsgDirect(pc, skb, -1); 
}

static void
l3dss1_msg_without_setup(l3_process_t *pc, unsigned char cause)
{
	/* This routine is called if here was no SETUP made (checks in dss1up and in
	 * l3dss1_setup) and a RELEASE_COMPLETE have to be sent with an error code
	 * MT_STATUS_ENQUIRE in the NULL state is handled too
	 */
	switch (cause) {
		case 81:	/* invalid callreference */
		case 88:	/* incomp destination */
		case 96:	/* mandory IE missing */
		case 100:       /* invalid IE contents */
		case 101:	/* incompatible Callstate */
			l3dss1_message_cause(pc, MT_RELEASE_COMPLETE, cause);
			break;
		default:
			break;
	}
	send_proc(pc, IMSG_END_PROC, NULL);
}

static void
l3dss1_std_ie_err(l3_process_t *pc, int ret) {

	if (pc->l3->debug & L3_DEB_CHECK)
		l3_debug(pc->l3, "check_infoelements ret %d", ret);
	switch(ret) {
		case 0: 
			break;
		case ERR_IE_COMPREHENSION:
			l3dss1_status_send(pc, CAUSE_MANDATORY_IE_MISS);
			break;
		case ERR_IE_UNRECOGNIZED:
			l3dss1_status_send(pc, CAUSE_IE_NOTIMPLEMENTED);
			break;
		case ERR_IE_LENGTH:
			l3dss1_status_send(pc, CAUSE_INVALID_CONTENTS);
			break;
		case ERR_IE_SEQUENCE:
		default:
			break;
	}
}

static unsigned char *
get_channel_id(l3_process_t *pc, struct sk_buff *skb) {
	Q931_info_t	*qi = (Q931_info_t *)skb->data;
	unsigned char	*p;

	if (qi->channel_id.off) {
		p = skb->data;
		p += L3_EXTRA_SIZE + qi->channel_id.off;
		p++;
	} else
		return(NULL);
	return(p);
}

static unsigned char *
l3dss1_get_channel_id(l3_process_t *pc, struct sk_buff *omsg) {
	Q931_info_t	*qi = (Q931_info_t *)omsg->data;
	unsigned char	*sp, *p;
	int	l;

	if (!qi->channel_id.off) {
		pc->err = -1;
		return(NULL);
	}
	p = omsg->data;
	p += L3_EXTRA_SIZE + qi->channel_id.off;
	sp = p;
	

		l = *p++;
		if (test_bit(FLG_EXTCID, &pc->l3->Flag)) { /* PRI */
			if (l < 3) {
				if (pc->l3->debug & L3_DEB_WARN)
					l3_debug(pc->l3, "wrong chid len %d", *p);
				pc->err = -2;
				return (NULL);
			}
			if ((*p & 0x60) != 0x20) {
				if (pc->l3->debug & L3_DEB_WARN)
					l3_debug(pc->l3, "wrong chid %x (for PRI interface)", *p);
				pc->err = -3;
				return (NULL);
			}
			p++;
			if (*p & 0x10) {
				if (pc->l3->debug & L3_DEB_WARN)
					l3_debug(pc->l3, "wrong chid %x (channel map not supported)", *p);
				pc->err = -4;
				return (NULL);
			}
			p++;
			pc->bc = *p & 0x7f;
		} else { /* BRI */
			if (l < 1) {
				if (pc->l3->debug & L3_DEB_WARN)
					l3_debug(pc->l3, "wrong chid len %d", *p);
				pc->err = -2;
				return (NULL);
			}
			if (*p & 0x60) {
				if (pc->l3->debug & L3_DEB_WARN)
					l3_debug(pc->l3, "wrong chid %x", *p);
				pc->err = -3;
				return (NULL);
			}
			pc->bc = *p & 3;
		}
		p = sp;
	
	return(sp);
}

static int
l3dss1_get_cause(l3_process_t *pc, struct sk_buff *skb) {
	Q931_info_t	*qi = (Q931_info_t *)skb->data;
	u_char		l;
	u_char		*p;

	if (qi->cause.off) {
		p = skb->data;
		p += L3_EXTRA_SIZE + qi->cause.off;
		p++;
		l = *p++;
		if (l>30) {
			return(-30);
		}
		if (l)
			l--;
		else {
			return(-2);
		}
		if (l && !(*p & 0x80)) {
			l--;
			p++; /* skip recommendation */
		}
		p++;
		if (l) {
			if (!(*p & 0x80)) {
				return(-3);
			}
			pc->err = *p & 0x7F;
		} else {
			return(-4);
		}
	} else
		return(-1);
	return(0);
}

static void
l3dss1_status_enq(l3_process_t *proc, int pr, void *arg)
{
}

static void
l3dss1_userinfo(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;

	skb = prep_l3data_msg(CC_USER_INFORMATION | INDICATION, pc->l4id /* pc->ces |
		(pc->callref << 16) */, 0, 0, arg);
	if (!skb)
		return;
	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);
}

static void
l3dss1_facility(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff  *skb;
	Q931_info_t	*qi;
	int		ret;

	skb = prep_l3data_msg(CC_FACILITY | INDICATION,
		pc->callref>0? pc->l4id /* pc->ces | (pc->callref << 16) */ :-1, 
		0, 0, arg);
	if (!skb)
		return;

	qi = (Q931_info_t *)skb->data;
	ret = check_infoelements(pc, skb, ie_FACILITY);
	l3dss1_std_ie_err(pc, ret);
	if (!qi->facility.off) {
		if (pc->l3->debug & L3_DEB_WARN)
			l3_debug(pc->l3, "FACILITY without IE_FACILITY");
		dev_kfree_skb(skb);
		return;
	}

	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);
}

static void
l3dss1_restart_req(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff  *skb;

	if (!arg) return ;

	skb = MakeMsgMT(pc, arg, MT_RESTART);
	SendMsgDirect(pc, skb, -1);
}

static void
l3dss1_restart(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;

	skb = prep_l3data_msg(CC_RESTART | INDICATION,
		pc->callref>0? pc->l4id /* pc->ces | (pc->callref << 16) */ :-1,
		0, 0, arg);
	if (!skb)
		return;

	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);

	skb = MakeMsgMT(pc, arg, MT_RESTART_ACKNOWLEDGE);
	SendMsgDirect(pc, skb, -1);
}

static void
l3dss1_setup(l3_process_t *pc, int pr, void *arg)
{
	unsigned char	*p;
	int	bcfound = 0;
	struct sk_buff *skb;
	int	err = 0;
	Q931_info_t	*qi;
	
	skb = prep_l3data_msg(CC_SETUP | INDICATION, pc->l4id /* pc->ces |
		(pc->callref << 16) */, 0, 0, arg);
	if (!skb)
		return;
	qi = (Q931_info_t *)skb->data;
	p = skb->data;

	/*
	 * Bearer Capabilities
	 */
	/* only the first occurence 'll be detected ! */
	if (qi->bearer_capability.off) {
		p += L3_EXTRA_SIZE + qi->bearer_capability.off;
		p++;
		if ((p[0] < 2) || (p[0] > 11))
			err = 1;
		else {
			switch (p[1] & 0x7f) {
				case 0x00: /* Speech */
				case 0x10: /* 3.1 Khz audio */
				case 0x08: /* Unrestricted digital information */
				case 0x09: /* Restricted digital information */
				case 0x11:
					/* Unrestr. digital information  with 
					 * tones/announcements ( or 7 kHz audio
					 */
				case 0x18: /* Video */
					break;
				default:
					err = 2;
					break;
			}
			switch (p[2] & 0x7f) {
				case 0x40: /* packed mode */
				case 0x10: /* 64 kbit */
				case 0x11: /* 2*64 kbit */
				case 0x13: /* 384 kbit */
				case 0x15: /* 1536 kbit */
				case 0x17: /* 1920 kbit */
					break;
				default:
					err = 3;
					break;
			}
		}
		if (err) {
			if (pc->l3->debug & L3_DEB_WARN)
				l3_debug(pc->l3, "setup with wrong bearer(l=%d:%x,%x)",
					p[0], p[1], p[2]);
			l3dss1_msg_without_setup(pc, CAUSE_INVALID_CONTENTS);
			dev_kfree_skb(skb);
			return;
		} 
	} else {
		if (pc->l3->debug & L3_DEB_WARN)
			l3_debug(pc->l3, "setup without bearer capabilities");
		/* ETS 300-104 1.3.3 */
		l3dss1_msg_without_setup(pc, CAUSE_MANDATORY_IE_MISS);
		dev_kfree_skb(skb);
		return;
	}
	/*
	 * Channel Identification
	 */
	if ((l3dss1_get_channel_id(pc, skb))) {
		if (pc->bc) {
			bcfound++;
		} else {
			if (pc->l3->debug & L3_DEB_WARN)
				l3_debug(pc->l3, "setup without bchannel, call waiting");
			bcfound++;
		} 
	} else if (pc->err != -1) {
		if (pc->l3->debug & L3_DEB_WARN)
			l3_debug(pc->l3, "setup with wrong chid ret %d", pc->err);
	}
	/* Now we are on none mandatory IEs */
	err = check_infoelements(pc, skb, ie_SETUP);
	if (ERR_IE_COMPREHENSION == err) {
		l3dss1_msg_without_setup(pc, CAUSE_MANDATORY_IE_MISS);
		dev_kfree_skb(skb);
		return;
	}

	newl3state(pc, 1);
	L3DelTimer_net(&pc->timer2);
	L3AddTimer_net(&pc->timer2, T_CTRL, 0x31f);
	if (err) /* STATUS for none mandatory IE errors after actions are taken */
		l3dss1_std_ie_err(pc, err);
	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);
}

static void
l3dss1_disconnect(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;
	int ret;

	if (pc->state == 19) {
	//	printf("We're in State 19, receive disconnect, so we stay here\n");
		return ;
	}

	skb = prep_l3data_msg(CC_DISCONNECT | INDICATION, pc->l4id /* pc->ces |
		(pc->callref << 16) */, 0, 0, arg);
	if (!skb)
		return;

	StopAllL3Timer_net(pc);
	newl3state(pc, 11);

	if ((ret = l3dss1_get_cause(pc, skb))) {
		if (pc->l3->debug & L3_DEB_WARN)
			l3_debug(pc->l3, "DISC get_cause ret(%d)", ret);
	}  else {
		pc->cause = pc->err;
	}
	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);
}

static void
l3dss1_disconnect_i(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;
	unsigned char		cause = 0;
	int ret;

	skb = prep_l3data_msg(CC_DISCONNECT | INDICATION, pc->l4id /* pc->ces |
		(pc->callref << 16) */ , 0, 0, arg);
	if (!skb)
		return;
	StopAllL3Timer_net(pc);
	if ((ret = l3dss1_get_cause(pc, skb))) {
		if (pc->l3->debug & L3_DEB_WARN)
			l3_debug(pc->l3, "DISC get_cause ret(%d)", pc->err);
		if (ret == -1)
			cause = CAUSE_MANDATORY_IE_MISS;
		else
			cause = CAUSE_INVALID_CONTENTS;
	} else {
		cause = pc->err;
		pc->cause = cause;
	}
	if (cause)
		l3dss1_message_cause(pc, MT_RELEASE, cause);
	else
		l3dss1_message(pc, MT_RELEASE);
	newl3state(pc, 19);
	test_and_clear_bit(FLG_L3P_TIMER308_1, &pc->Flags);
	L3AddTimer_net(&pc->timer1, T308, 0x308);
	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);
}

static void
l3dss1_information(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;

	skb = prep_l3data_msg(CC_INFORMATION | INDICATION, pc->l4id /* pc->ces |
		(pc->callref << 16) */ , 0, 0, arg);
	if (!skb)
		return;
	if (pc->state == 2) { /* overlap receiving */
		L3DelTimer_net(&pc->timer1);
		L3AddTimer_net(&pc->timer1, T302, 0x302);
	}
	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);
}

static void
l3dss1_release(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;
	int 	cause=0;
	int ret;

	skb = prep_l3data_msg(CC_RELEASE | INDICATION, pc->l4id /* pc->ces |
		(pc->callref << 16) */ , 0, 0, arg);
	if (!skb)
		return;
	StopAllL3Timer_net(pc);
	if ((ret = l3dss1_get_cause(pc, skb))) {
		if (pc->state != 12) {
			if (pc->l3->debug & L3_DEB_WARN)
				l3_debug(pc->l3, "REL get_cause ret(%d)",
					pc->err);
			if ((ret == -1) && (pc->state != 12))
				cause = CAUSE_MANDATORY_IE_MISS;
			else 
				cause = CAUSE_INVALID_CONTENTS;
		}
	} else
		cause = pc->err;
	if (cause)
		l3dss1_message_cause(pc, MT_RELEASE_COMPLETE, cause);
	else
		l3dss1_message(pc, MT_RELEASE_COMPLETE);
	/* not in state 12 
	if (mISDN_l3up(pc, skb))
	*/
		dev_kfree_skb(skb);

	newl3state(pc, 0);
	send_proc(pc, IMSG_END_PROC_M, NULL);
}

static void
l3dss1_release_i(l3_process_t *pc, int pr, void *arg)
{
	l3dss1_message(pc, MT_RELEASE_COMPLETE);
	newl3state(pc, 0);
	send_proc(pc, IMSG_END_PROC_M, NULL);
}

static void
l3dss1_release_cmpl(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;
	int ret;

	skb = prep_l3data_msg(CC_RELEASE_COMPLETE | INDICATION, pc->l4id /* pc->ces |
		(pc->callref << 16) */ , 0, 0, arg);
	if (!skb)
		return;

	StopAllL3Timer_net(pc);
	newl3state(pc, 0);
	if ((ret = l3dss1_get_cause(pc, skb))) {
		if (pc->l3->debug & L3_DEB_WARN)
			l3_debug(pc->l3, "RELCMPL get_cause err(%d)",
				ret);
	} else {
		pc->cause = pc->err;
	}
	/* not in state 7
	if (mISDN_l3up(pc, skb))
	*/
		dev_kfree_skb(skb);
	send_proc(pc, IMSG_END_PROC_M, NULL);
}

/*
static void
l3dss1_release_cmpl_i(l3_process_t *pc, int pr, void *arg)
{
	send_proc(pc, IMSG_END_PROC_M, NULL);
}
*/

static void
l3dss1_connect_ack(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff	*skb;

	skb = prep_l3data_msg(CC_CONNECT_ACKNOWLEDGE | INDICATION, pc->l4id 
		/* pc->master->ces | (pc->master->callref << 16) */, 0, 0, arg);
	if (!skb)
		return;

	L3DelTimer_net(&pc->timer1);	/* T310 */
	newl3state(pc, 8);
	if (!mISDN_l3up(pc, skb))
		return;
	dev_kfree_skb(skb);
}

static void
l3dss1_setup_acknowledge_i(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;
	l3_process_t *master = pc;

	if (!test_bit(FLG_PTP, &pc->l3->Flag)) {
		if (!pc->master) {
			L3DelTimer_net(&pc->timer1);
			newl3state(pc, 25);
			return;
		}
		master = pc->master;
	}
	skb = prep_l3data_msg(CC_SETUP_ACKNOWLEDGE | INDICATION, master->l4id 
		/* pc->master->ces | (pc->master->callref << 16) */, 0, 0, arg);
	if (!skb)
		return;
	L3DelTimer_net(&pc->timer1);	/* T304 */
	newl3state(pc, 25);
	if (!mISDN_l3up(master, skb))
		return;
	dev_kfree_skb(skb);
}

static void
l3dss1_proceeding_i(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;
	l3_process_t *master = pc;

	if (!test_bit(FLG_PTP, &pc->l3->Flag)) {
		if (!pc->master) {
			L3DelTimer_net(&pc->timer1);
			newl3state(pc, 9);
			return;
		}
		master = pc->master;
	}
	skb = prep_l3data_msg(CC_PROCEEDING | INDICATION, master->l4id
		/* pc->master->ces | (pc->master->callref << 16) */, 0, 0, arg);
	if (!skb)
		return;
	L3DelTimer_net(&pc->timer1);	/* T304 */
	newl3state(pc, 9);
	if (!mISDN_l3up(master, skb))
		return;
	dev_kfree_skb(skb);
}

static void
l3dss1_alerting_i(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;
	l3_process_t *master = pc;

	if (!test_bit(FLG_PTP, &pc->l3->Flag)) {
		if (!pc->master) {
			L3DelTimer_net(&pc->timer1);
			newl3state(pc, 7);
			return;
		}
		master = pc->master;
	}
	skb = prep_l3data_msg(CC_ALERTING | INDICATION, master->l4id
		/* pc->master->ces | (pc->master->callref << 16) */, 0, 0, arg);
	if (!skb)
		return;
	L3DelTimer_net(&pc->timer1);	/* T304 */
	newl3state(pc, 7);
	if (!mISDN_l3up(master, skb))
		return;
	dev_kfree_skb(skb);
}

static void
l3dss1_connect_i(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;
	l3_process_t *master = pc;
	u_char		*p;
	Q931_info_t *qi;

	if (!test_bit(FLG_PTP, &pc->l3->Flag)) {
		if (!pc->master) {
			L3DelTimer_net(&pc->timer1);
			newl3state(pc, 8);
			return;
		}
		master = pc->master;
	}
	skb = prep_l3data_msg(CC_CONNECT | INDICATION, master->l4id 
		/* pc->master->ces | (pc->master->callref << 16) */, 0, 3, arg);
	if (!skb)
		return;

	if (!test_bit(FLG_PTP, &pc->l3->Flag)) {
		qi = (Q931_info_t *)skb->data;
		qi->channel_id.off = skb->len - L3_EXTRA_SIZE;
		p = skb_put(skb, 4);
		*p++ = IE_CHANNEL_ID;
		*p++ = 0x01;
		*p++ = 0x88 | pc->bc;
	}

	L3DelTimer_net(&pc->timer1);	/* T310 */
	newl3state(pc, 8);
	l3dss1_message(pc, MT_CONNECT_ACKNOWLEDGE);
	if (test_bit(FLG_PTP, &pc->l3->Flag)) {
		if (mISDN_l3up(pc, skb))
			dev_kfree_skb(skb);
		return;
	}
	if (send_proc(pc, IMSG_CONNECT_IND, skb))
		dev_kfree_skb(skb);
}

static void
l3dss1_hold(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;
/*
	if (!(pc->l3->nst->feature & FEATURE_NET_HOLD)) {
		l3dss1_message_cause(pc, MT_HOLD_REJECT, CAUSE_MT_NOTIMPLEMENTED);
		return;
	}
*/
	if (pc->hold_state == HOLDAUX_HOLD_IND)
		return;
	if (pc->hold_state != HOLDAUX_IDLE) {
		l3dss1_message_cause(pc, MT_HOLD_REJECT, CAUSE_NOTCOMPAT_STATE);
		return;
	}
	pc->hold_state = HOLDAUX_HOLD_IND; 

	skb = prep_l3data_msg(CC_HOLD | INDICATION, pc->l4id /* pc->ces |
		(pc->callref << 16) */ , 0, 0, arg);
	if (!skb)
		return;
	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);
}

static void
l3dss1_retrieve(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;
/*
	if (!(pc->l3->nst->feature & FEATURE_NET_HOLD)) {
		l3dss1_message_cause(pc, MT_RETRIEVE_REJECT, CAUSE_MT_NOTIMPLEMENTED);
		return;
	}
*/
	if (pc->hold_state == HOLDAUX_RETR_IND)
		return;
	if (pc->hold_state != HOLDAUX_HOLD) {
		l3dss1_message_cause(pc, MT_RETRIEVE_REJECT, CAUSE_NOTCOMPAT_STATE);
		return;
	}
	pc->hold_state = HOLDAUX_RETR_IND;

	skb = prep_l3data_msg(CC_RETRIEVE | INDICATION, pc->l4id /* pc->ces |
		(pc->callref << 16) */, 0, 0, arg);
	if (!skb)
		return;
	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);
}

static void
l3dss1_suspend(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;

	skb = prep_l3data_msg(CC_SUSPEND | INDICATION, pc->l4id /* pc->ces |
		(pc->callref << 16) */ , 0, 0, arg);
	if (!skb)
		return;
	newl3state(pc, 15);
	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);
}

static void
l3dss1_resume(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;

	skb = prep_l3data_msg(CC_RESUME | INDICATION, pc->l4id /* pc->ces |
		(pc->callref << 16) */ , 0, 0, arg);
	if (!skb)
		return;
	newl3state(pc, 17);
	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);
}

static struct stateentry datastatelist[] =
{
	{ALL_STATES,
		MT_STATUS_ENQUIRY, l3dss1_status_enq},
	{ALL_STATES,
		MT_FACILITY, l3dss1_facility},
	{SBIT(19),
		MT_STATUS, l3dss1_release_cmpl},
	{SBIT(0),
		MT_SETUP, l3dss1_setup},
	{SBIT(6) | SBIT(7)  | SBIT(9) | SBIT(25),
		MT_CALL_PROCEEDING, l3dss1_proceeding_i},
	{SBIT(6) | SBIT(7)  | SBIT(9) | SBIT(25),
		MT_SETUP_ACKNOWLEDGE, l3dss1_setup_acknowledge_i},
	{SBIT(6) | SBIT(7)  | SBIT(9) | SBIT(25),
		MT_ALERTING, l3dss1_alerting_i},
	{SBIT(2) | SBIT(3) | SBIT(4) | SBIT(7) | SBIT(8) | SBIT(9) | SBIT(10) |
	 SBIT(11) | SBIT(12) | SBIT(15) | SBIT(17) | SBIT(19) | SBIT(25),
		MT_INFORMATION, l3dss1_information},
	{SBIT(0) | SBIT(1) | SBIT(2) | SBIT(3) | SBIT(4) | SBIT(10) |
	 SBIT(11) | SBIT(12) | SBIT(15) | SBIT(17) | SBIT(19),
		MT_RELEASE_COMPLETE, l3dss1_release_cmpl},
	{SBIT(6) | SBIT(7) | SBIT(8) | SBIT(9) | SBIT(25),
	/*
		MT_RELEASE_COMPLETE, l3dss1_release_cmpl_i},
	*/
		MT_RELEASE_COMPLETE, l3dss1_release_cmpl},

	{SBIT(1) | SBIT(2) | SBIT(3) | SBIT(4) | SBIT(10) |
	 SBIT(11) | SBIT(12) | SBIT(15) | SBIT(17),
		MT_RELEASE, l3dss1_release},
	{SBIT(6) | SBIT(7) | SBIT(8) | SBIT(9) | SBIT(19) | SBIT(25),
		MT_RELEASE, l3dss1_release_i},
	{SBIT(6) | SBIT(7)  | SBIT(9) | SBIT(25),
		MT_CONNECT, l3dss1_connect_i},
	{SBIT(10),
	 MT_CONNECT_ACKNOWLEDGE, l3dss1_connect_ack},
	{SBIT(1) | SBIT(2) | SBIT(3) | SBIT(4) | SBIT(10) | SBIT(19),
		MT_DISCONNECT, l3dss1_disconnect},
	{SBIT(7) | SBIT(8) | SBIT(9) | SBIT(25),
		MT_DISCONNECT, l3dss1_disconnect_i},
	{SBIT(4) | SBIT(7) | SBIT(10),
		MT_USER_INFORMATION, l3dss1_userinfo},
	{SBIT(3) | SBIT(4) | SBIT(10),
		MT_HOLD, l3dss1_hold},
	{SBIT(3) | SBIT(4) | SBIT(10) | SBIT(12),
		MT_RETRIEVE, l3dss1_retrieve},
	{SBIT(10),
		MT_SUSPEND, l3dss1_suspend},
	{SBIT(0),
		MT_RESUME, l3dss1_resume},
};

#define DATASLLEN \
	(sizeof(datastatelist) / sizeof(struct stateentry))

static l3_process_t 
*create_child_proc(l3_process_t *pc, int mt, struct sk_buff *msg, int state) {
	mISDN_head_t	*hh;
	struct _l3_msg	l3m;
	l3_process_t	*p3i;

	if (!test_bit(FLG_PTP, &pc->l3->Flag)) {
		hh = mISDN_HEAD_P(msg);
		p3i = create_proc(pc->l3, hh->dinfo, pc->callref, pc);
		if (!p3i) {
			l3_debug(pc->l3, "cannot create child");
			return(NULL);
		}
		p3i->l4id = pc->l4id;
		p3i->state = pc->state;
		p3i->bc = pc->bc;
		if (pc->state != -1)
			newl3state(pc, state);
	} else {
		p3i = pc;
	}
	l3m.mt = mt;
	l3m.skb = msg;
	send_proc(p3i, IMSG_L2_DATA, &l3m);
	return(p3i);
}                                                   

static void
l3dss1_setup_acknowledge_m(l3_process_t *pc, int pr, void *arg)
{
	L3DelTimer_net(&pc->timer1);
	create_child_proc(pc, pr, arg, 25);
}

static void
l3dss1_proceeding_m(l3_process_t *pc, int pr, void *arg)
{
	L3DelTimer_net(&pc->timer1);
	create_child_proc(pc, pr, arg, 9);
}

static void
l3dss1_alerting_m(l3_process_t *pc, int pr, void *arg)
{
	L3DelTimer_net(&pc->timer1);
	create_child_proc(pc, pr, arg, 7);
}

static void
l3dss1_connect_m(l3_process_t *pc, int pr, void *arg)
{
	L3DelTimer_net(&pc->timer1);
	create_child_proc(pc, pr, arg, 8);
}

static void
l3dss1_release_m(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *msg = arg;
	l3dss1_release_i(pc, pr, msg);
}

static void
l3dss1_release_mx(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *msg = arg;

	l3dss1_release(pc, pr, msg);
}

static void
l3dss1_release_cmpl_m(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *msg = arg;

	if ((pc->state == 6) || (pc->state == 7)) {
		if (!l3dss1_get_cause(pc, msg)) {
			switch(pc->cause) {
				case CAUSE_USER_BUSY:
					break;
				case CAUSE_CALL_REJECTED:
					if (pc->err == CAUSE_USER_BUSY)
						pc->cause = pc->err;
					break;
				default:
					pc->cause = pc->err;
			}
		}
		test_and_set_bit(FLG_L3P_GOTRELCOMP, &pc->Flags);
		send_proc(pc, IMSG_END_PROC_M, NULL);
	}
}

static void
l3dss1_release_cmpl_mx(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *msg = arg;

	l3dss1_release_cmpl(pc, pr, msg);
}

static void
l3dss1_information_mx(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *msg = arg;

	l3dss1_information(pc, pr, msg);
}

static struct stateentry mdatastatelist[] =
{
	{SBIT(6) | SBIT(7) | SBIT(9) | SBIT(25),
		MT_SETUP_ACKNOWLEDGE, l3dss1_setup_acknowledge_m},
	{SBIT(6) | SBIT(7) | SBIT(9) | SBIT(25),
		MT_CALL_PROCEEDING, l3dss1_proceeding_m},
	{SBIT(6) | SBIT(7) | SBIT(9) | SBIT(25),
		MT_ALERTING, l3dss1_alerting_m},
	{SBIT(6) | SBIT(7) | SBIT(9) | SBIT(25),
		MT_CONNECT, l3dss1_connect_m},
	{SBIT(2) | SBIT(3) | SBIT(4) | SBIT(7) | SBIT(8) | SBIT(9) | SBIT(10) |
	 SBIT(11) | SBIT(12) | SBIT(15) | SBIT(17) | SBIT(19) | SBIT(25),
		MT_INFORMATION, l3dss1_information_mx},
	{SBIT(1) | SBIT(2) | SBIT(3) | SBIT(4) | SBIT(10) | SBIT(11) |
	 SBIT(12) | SBIT(15) | SBIT(17),
		MT_RELEASE, l3dss1_release_mx},
	{SBIT(6) | SBIT(7) | SBIT(8) | SBIT(9) | SBIT(22) | SBIT(25),
		MT_RELEASE, l3dss1_release_m},
	{SBIT(19),  MT_RELEASE, l3dss1_release_cmpl},
	{SBIT(0) | SBIT(1) | SBIT(2) | SBIT(3) | SBIT(4) | SBIT(10) |
	 SBIT(11) | SBIT(12) | SBIT(15) | SBIT(17) | SBIT(19),
		MT_RELEASE_COMPLETE, l3dss1_release_cmpl_mx},
	{SBIT(6) | SBIT(7) | SBIT(8) | SBIT(9) | SBIT(22) | SBIT(25),
		MT_RELEASE_COMPLETE, l3dss1_release_cmpl_m},
};

#define MDATASLLEN \
	(sizeof(mdatastatelist) / sizeof(struct stateentry))
 
static void
l3dss1_setup_ack_req(l3_process_t *pc, int pr, void *arg)
{
	unsigned char *p;

	if (arg) {
		p = get_channel_id(pc, arg);
		if (p) {
			if (p[0] == 1)
				pc->bc = p[1] & 3;
		}
		SendMsg(pc, arg, 2);
	} else {
		newl3state(pc, 2);
		l3dss1_message(pc, MT_SETUP_ACKNOWLEDGE);
	}
	L3DelTimer_net(&pc->timer1);
	L3AddTimer_net(&pc->timer1, T302, 0x302);
}

static void
l3dss1_proceed_req(l3_process_t *pc, int pr, void *arg)
{
	unsigned char *p;

	L3DelTimer_net(&pc->timer1);
	if (arg) {
		p = get_channel_id(pc, arg);
		if (p) {
			if (p[0] == 1)
				pc->bc = p[1] & 3;
		}
		SendMsg(pc, arg, 3);
	} else {
		newl3state(pc, 3);
		l3dss1_message(pc, MT_CALL_PROCEEDING);
	}
}

static void
l3dss1_alert_req(l3_process_t *pc, int pr, void *arg)
{
	unsigned char *p;

	if (arg) {
		p = get_channel_id(pc, arg);
		if (p) {
			if (p[0] == 1)
				pc->bc = p[1] & 3;
		}
		SendMsg(pc, arg, 4);
	} else {
		newl3state(pc, 4);
		l3dss1_message(pc, MT_ALERTING);
	}
	L3DelTimer_net(&pc->timer1);
}

static void
l3dss1_setup_req(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb = MakeMsg(pc, arg);
	unsigned char *p;

	if (!skb)
		return;

	p = get_channel_id(pc, (struct sk_buff *)arg);
	if (p) {
		if (p[0] == 1)
			pc->bc = p[1] & 3;
		if (debug & 0x04)
			l3_debug(pc->l3, "%s: channel id 0x%x 0x%x", __FUNCTION__, p[0], p[1]);
	}
	newl3state(pc, 6);
	copy_to_obuf(pc, skb);
	if (test_bit(FLG_PTP, &pc->l3->Flag)) {
		if (l3_msg(pc->l3, DL_DATA | REQUEST, 0, skb))
			dev_kfree_skb(skb);
	} else {
		if (l3_msg(pc->l3, DL_UNITDATA | REQUEST, 127, skb))
			dev_kfree_skb(skb);
	}
	L3DelTimer_net(&pc->timer1);
	test_and_clear_bit(FLG_L3P_TIMER303_1, &pc->Flags);
	L3AddTimer_net(&pc->timer1, T303, 0x303);
	L3DelTimer_net(&pc->timer2);
	if (!test_bit(FLG_PTP, &pc->l3->Flag)) {
		test_and_set_bit(FLG_L3P_TIMER312, &pc->Flags);
		L3AddTimer_net(&pc->timer2, T312, 0x312);
	}
}

static void
l3dss1_connect_req(l3_process_t *pc, int pr, void *arg)
{
	unsigned char *p;

	L3DelTimer_net(&pc->timer1);

	if (arg) {
		p = get_channel_id(pc, arg);
		if (p) {
			if (p[0] == 1)
				pc->bc = p[1] & 3;
		}
		SendMsg(pc, arg, 10);
	} else {
		newl3state(pc, 10);
		l3dss1_message(pc, MT_CONNECT);
	}
}

static void
l3dss1_connect_res(l3_process_t *pc, int pr, void *arg)
{
	unsigned char *p;
	int			cause;

	L3DelTimer_net(&pc->timer1);
	send_proc(pc, IMSG_SEL_PROC, NULL);

	if (arg) {
		p = get_channel_id(pc, arg);
		if (p) {
			if (p[0] == 1)
				pc->bc = p[1] & 3;
		}
		SendMsg(pc, arg, 10);
	} else {
		newl3state(pc, 10);
		l3dss1_message(pc, MT_CONNECT_ACKNOWLEDGE);
	}
	cause = CAUSE_NONSELECTED_USER;
	send_proc(pc, IMSG_RELEASE_CHILDS, &cause);
}

static void
l3dss1_disconnect_req(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff	*skb = arg;
	Q931_info_t	*qi;
	u_char		*p;

	StopAllL3Timer_net(pc);
	if (arg) {
		qi = (Q931_info_t *)skb->data;
		if (!qi->cause.off) {
			qi->cause.off = skb->len - L3_EXTRA_SIZE;
			p = skb_put(skb, 4);
			*p++ = IE_CAUSE;
			*p++ = 2;
			*p++ = 0x80 | CAUSE_LOC_PNET_LOCUSER;
			*p++ = 0x80 | CAUSE_NORMALUNSPECIFIED;
			pc->cause=CAUSE_NORMALUNSPECIFIED;
		} else {
			p=skb->data;
			p += L3_EXTRA_SIZE + qi->cause.off;
			pc->cause = (*(p+3) & 0x7f);
		}
		SendMsg(pc, arg, 12);
	} else {
		newl3state(pc, 12);
		l3dss1_message_cause(pc, MT_DISCONNECT, CAUSE_NORMALUNSPECIFIED);
		pc->cause=CAUSE_NORMALUNSPECIFIED;
	}
	L3AddTimer_net(&pc->timer1, T305, 0x305);
}

static void
l3dss1_facility_req(l3_process_t *pc, int pr, void *arg)
{
	if (arg) {
		SendMsg(pc, arg, -1);
	}
}

static void
l3dss1_userinfo_req(l3_process_t *pc, int pr, void *arg)
{
	if (arg) {
		SendMsg(pc, arg, -1);
	}
}

static void
l3dss1_information_req(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb;

	if (pc->state == 25 && !(test_bit(FLG_PTP, &pc->l3->Flag)))
		return;
	
	if (arg) {
		if (pc->state != 25)
			SendMsg(pc, arg , -1);
		else {
			if (!(skb = MakeMsg(pc, arg)))
				return;
			copy_to_obuf(pc, skb);
			if (l3_msg(pc->l3, DL_DATA | REQUEST, 0, skb))
				kfree_skb(skb);
		}
	}
}

static void
l3dss1_progress_req(l3_process_t *pc, int pr, void *arg)
{
	if (arg) {
		SendMsg(pc, arg, -1);
	}
}

static void
l3dss1_notify_req(l3_process_t *pc, int pr, void *arg)
{
	if (arg) {
		SendMsg(pc, arg, -1);
	}
}

static void
l3dss1_disconnect_req_out(l3_process_t *pc, int pr, void *arg)
{
	struct sk_buff *skb = (struct sk_buff *)arg;
	Q931_info_t	*qi;
	int		cause = CAUSE_NORMALUNSPECIFIED;
	u_char		*p;

	if (test_bit(FLG_PTP, &pc->l3->Flag)) {
		l3dss1_disconnect_req(pc, pr, arg);
		return;
	}

	if (pc->master) { /* child */
		l3dss1_disconnect_req_out(pc->master, pr, arg);
		return;
	}
	L3DelTimer_net(&pc->timer1);
	if (arg) {
		qi = (Q931_info_t *)skb->data;
		if (!qi->cause.off) {
			cause = CAUSE_NORMALUNSPECIFIED;
		} else {
			p=skb->data;
			p += L3_EXTRA_SIZE + qi->cause.off;
			cause = (*(p+3) & 0x7f);
		}
	}
	send_proc(pc, IMSG_RELEASE_CHILDS, &cause);
	if (test_bit(FLG_L3P_TIMER312, &pc->Flags)) {
		newl3state(pc, 22);
	} else {
		mISDN_queue_data(&pc->l3->inst, FLG_MSG_UP,
			CC_RELEASE | CONFIRM, pc->l4id /* pc->ces | (pc->callref << 16) */,
			0, NULL, 0);
		newl3state(pc, 0);
		if (list_empty(&pc->childlist))
			send_proc(pc, IMSG_END_PROC_M, NULL);
	}
}

static void
l3dss1_release_req(l3_process_t *pc, int pr, void *arg)
{
	StopAllL3Timer_net(pc);
	if (arg) {
		SendMsg(pc, arg, 19);
	} else {
		newl3state(pc, 19);
		l3dss1_message(pc, MT_RELEASE);
	}
	test_and_clear_bit(FLG_L3P_TIMER308_1, &pc->Flags);
	L3AddTimer_net(&pc->timer1, T308, 0x308);
}

static void
l3dss1_release_cmpl_req(l3_process_t *pc, int pr, void *arg)
{
	StopAllL3Timer_net(pc);
	if (arg) {
		SendMsg(pc, arg, 0);
	} else {
		newl3state(pc, 0);
		l3dss1_message(pc, MT_RELEASE_COMPLETE);
	}
	mISDN_l3upu(pc, CC_RELEASE_COMPLETE | CONFIRM, NULL);
	send_proc(pc, IMSG_END_PROC_M, NULL);
}

static void
l3dss1_t302(l3_process_t *pc, int pr, void *arg)
{
	StopAllL3Timer_net(pc);
#if 0
	mISDN_queue_data(&pc->l3->inst, FLG_MSG_UP,
		CC_TIMEOUT | INDICATION, pc->l4id /* pc->ces | (pc->callref << 16) */,
		sizeof(int), &t, 0);
#endif
	pc->cause = CAUSE_TIMER_EXPIRED;
	l3dss1_message_cause(pc, MT_RELEASE_COMPLETE, CAUSE_TIMER_EXPIRED);
	send_proc(pc, IMSG_END_PROC_M, NULL);
}

static void
send_release_ind(l3_process_t *pc)
{
	struct sk_buff *skb, *nskb;
	u_char		*p;
	Q931_info_t *qi;

	nskb = mISDN_alloc_l3msg(4, MT_RELEASE);

	skb = prep_l3data_msg(CC_RELEASE | INDICATION,
		pc->l4id /* pc->ces | (pc->callref << 16) */,
		0, 4, nskb);
	dev_kfree_skb(nskb);
	if (!skb)
		return;
	qi = (Q931_info_t *)skb->data;
	qi->cause.off = skb->len - L3_EXTRA_SIZE;
	p = skb_put(skb, 4);
	*p++ = IE_CAUSE;
	*p++ = 2;
	*p++ = 0x80;
	if (pc->cause)
		*p++ = 0x80 | pc->cause;
	else
		*p++ = 0x80 | CAUSE_NORMALUNSPECIFIED;

	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);
}

static void
l3dss1_t303(l3_process_t *pc, int pr, void *arg)
{
	int			l;
	struct sk_buff *skb;
	u_char		*p;
	Q931_info_t *qi;

	L3DelTimer_net(&pc->timer1);
	if (test_bit(FLG_L3P_GOTRELCOMP, &pc->Flags)) {
		StopAllL3Timer_net(pc);
		skb = prep_l3data_msg(CC_RELEASE_COMPLETE | INDICATION,
			pc->l4id /* pc->ces | (pc->callref << 16) */,
			0, 4, arg);
		if (!skb)
			return;
		newl3state(pc, 0);
		qi = (Q931_info_t *)skb->data;
		qi->cause.off = skb->len - L3_EXTRA_SIZE;
		p = skb_put(skb, 4);
		*p++ = IE_CAUSE;
		*p++ = 2;
		*p++ = 0x80;
		if (pc->cause)
			*p++ = 0x80 | pc->cause;
		else
			*p++ = 0x80 | CAUSE_NORMALUNSPECIFIED;

		if (mISDN_l3up(pc, skb))
			dev_kfree_skb(skb);
		send_proc(pc, IMSG_END_PROC_M, NULL);
		return;
	}
	if (!test_and_set_bit(FLG_L3P_TIMER303_1, &pc->Flags)) {
		l = pc->obuflen;
		if ((l) && (pc->obuf[3] == MT_SETUP)) {
			if ((skb = alloc_stack_skb(pc->obuflen, pc->l3->down_headerlen))) {
				memcpy(skb_put(skb, l), &pc->obuf[0], l);

				if (test_bit(FLG_PTP, &pc->l3->Flag)) {
					if (l3_msg(pc->l3, DL_DATA | REQUEST, 0, skb))
						dev_kfree_skb(skb);
				} else {
					if (l3_msg(pc->l3, DL_UNITDATA | REQUEST, 127, skb))
						dev_kfree_skb(skb);
				}
			}

			L3DelTimer_net(&pc->timer2);
			if (!(test_bit(FLG_PTP, &pc->l3->Flag))) {
				L3AddTimer_net(&pc->timer2, T312, 0x312);
				test_and_set_bit(FLG_L3P_TIMER312, &pc->Flags);
			}

			L3AddTimer_net(&pc->timer1, T303, 0x303);
			return;
		}
	}
#if 0
	skb = prep_l3data_msg(CC_RELEASE_COMPLETE | INDICATION,
		pc->l4id /* pc->ces | (pc->callref << 16) */,
		0, 4, arg);
	if (!skb)
		return;
	qi = (Q931_info_t *)skb->data;
	qi->cause.off = skb->len - L3_EXTRA_SIZE;
	p = skb_put(skb, 4);
	*p++ = IE_CAUSE;
	*p++ = 2;
	*p++ = 0x85;
	*p++ = CAUSE_NOUSER_RESPONDING | 0x80;
	if (mISDN_l3up(pc, skb))
		dev_kfree_skb(skb);
//	newl3state(pc, 22);
#endif
	newl3state(pc, 0);
	send_proc(pc, IMSG_END_PROC_M, NULL);
}

static void
l3dss1_t305(l3_process_t *pc, int pr, void *arg)
{
	// mut we dat sendn? :  int t = 0x305;

	StopAllL3Timer_net(pc);

	newl3state(pc, 19);
	l3dss1_message(pc, MT_RELEASE);
	test_and_clear_bit(FLG_L3P_TIMER308_1, &pc->Flags);
	L3AddTimer_net(&pc->timer1, T308, 0x308);
}

static void
l3dss1_t308(l3_process_t *pc, int pr, void *arg)
{
	if (!test_and_set_bit(FLG_L3P_TIMER308_1, &pc->Flags)) {
		newl3state(pc, 19);
		L3DelTimer_net(&pc->timer1);
		l3dss1_message(pc, MT_RELEASE);
		L3AddTimer_net(&pc->timer1, T308, 0x308);
	} else {
		StopAllL3Timer_net(pc);
		newl3state(pc, 0);
#if 0
		mISDN_queue_data(&pc->l3->inst, FLG_MSG_UP,
			CC_TIMEOUT | INDICATION, pc->l4id /* pc->ces | (pc->callref << 16) */,
			sizeof(int), &t, 0);
#endif
		send_proc(pc, IMSG_END_PROC_M, NULL);
	}
}

static void
l3dss1_t312(l3_process_t *pc, int pr, void *arg)
{
	test_and_clear_bit(FLG_L3P_TIMER312, &pc->Flags);
	L3DelTimer_net(&pc->timer2);
	if (debug)
		l3_debug(pc->l3, "%s: state %d", __FUNCTION__, pc->state);
	if (pc->state == 22 || pc->state == 25 || pc->state == 9 || pc->state == 7) {
		StopAllL3Timer_net(pc);
		if (list_empty(&pc->childlist)) {
#if 0
			mISDN_queue_data(&pc->l3->inst, FLG_MSG_UP,
				CC_TIMEOUT | INDICATION, pc->l4id /* pc->ces | (pc->callref << 16) */,
				sizeof(int), &t, 0);
#endif
			send_proc(pc, IMSG_END_PROC_M, NULL);
		}
	}
}

static void
l3dss1_holdack_req(l3_process_t *pc, int pr, void *arg)
{
	if (pc->hold_state != HOLDAUX_HOLD_IND)
		return;
	pc->hold_state = HOLDAUX_HOLD; 
	if (arg) {
		SendMsg(pc, arg, -1);
	} else {
		l3dss1_message(pc, MT_HOLD_ACKNOWLEDGE);
	}
}

static void
l3dss1_holdrej_req(l3_process_t *pc, int pr, void *arg)
{
	if (pc->hold_state != HOLDAUX_HOLD_IND)
		return;
	pc->hold_state = HOLDAUX_IDLE; 
	if (arg) {
		SendMsg(pc, arg, -1);
	}
}

static void
l3dss1_retrack_req(l3_process_t *pc, int pr, void *arg)
{
	if (pc->hold_state != HOLDAUX_RETR_IND)
		return;
	pc->hold_state = HOLDAUX_IDLE;
	if (arg) {
		SendMsg(pc, arg, -1);
	} else {
		l3dss1_message(pc, MT_RETRIEVE_ACKNOWLEDGE);
	}
}

static void
l3dss1_retrrej_req(l3_process_t *pc, int pr, void *arg)
{
	if (pc->hold_state != HOLDAUX_RETR_IND)
		return;
	pc->hold_state = HOLDAUX_HOLD; 
	if (arg) {
		SendMsg(pc, arg, -1);
	}
}

static void
l3dss1_suspack_req(l3_process_t *pc, int pr, void *arg)
{
	StopAllL3Timer_net(pc);
	if (arg) {
		SendMsg(pc, arg, 0);
	} else {
		l3dss1_message(pc, MT_SUSPEND_ACKNOWLEDGE);
	}
	newl3state(pc, 0);
	send_proc(pc, IMSG_END_PROC_M, NULL);
}

static void
l3dss1_susprej_req(l3_process_t *pc, int pr, void *arg)
{
	if (arg) {
		SendMsg(pc, arg, -1);
	}
	newl3state(pc, 10);
}

static void
l3dss1_resack_req(l3_process_t *pc, int pr, void *arg)
{
	StopAllL3Timer_net(pc);
	if (arg) {
		SendMsg(pc, arg, 0);
	} else {
		l3dss1_message(pc, MT_RESUME_ACKNOWLEDGE);
	}
	newl3state(pc, 10);
}

static void
l3dss1_resrej_req(l3_process_t *pc, int pr, void *arg)
{
	if (arg) {
		SendMsg(pc, arg, -1);
	}
	newl3state(pc, 0);
	send_proc(pc, IMSG_END_PROC_M, NULL);
}

static struct stateentry downstatelist[] =
{
	{SBIT(2),
	 CC_T302, l3dss1_t302},
	{SBIT(12),
	 CC_T305, l3dss1_t305},
	{SBIT(6),
	 CC_T303, l3dss1_t303},
	{SBIT(19),
	 CC_T308, l3dss1_t308},
	{ALL_STATES,
	 CC_T312, l3dss1_t312},
	{ALL_STATES,
		CC_RELEASE_COMPLETE | REQUEST, l3dss1_release_cmpl_req},
	{SBIT(0),
	 CC_SETUP | REQUEST, l3dss1_setup_req},
	{SBIT(2) | SBIT(3) | SBIT(4) | SBIT(7) | SBIT(8) | SBIT(9) |
		SBIT(10) | SBIT(11) | SBIT(12) | SBIT(15) | SBIT(25),
	 CC_INFORMATION | REQUEST, l3dss1_information_req},
	{ALL_STATES,
	 CC_NOTIFY | REQUEST, l3dss1_notify_req},
	{SBIT(2) | SBIT(3) | SBIT(4),
	 CC_PROGRESS | REQUEST, l3dss1_progress_req},
	{SBIT(1) | SBIT(2) | SBIT(3) | SBIT(4) | SBIT(10),
	 CC_DISCONNECT | REQUEST, l3dss1_disconnect_req},
	{ SBIT(2) | SBIT(6) | SBIT(7) | SBIT(8) | SBIT(9) | SBIT(25),
	 CC_DISCONNECT | REQUEST, l3dss1_disconnect_req_out},
	{ SBIT(2) | SBIT(11)
	| SBIT(12) | SBIT(7) | SBIT(8) | SBIT(9) | SBIT(25)
	 ,CC_RELEASE | REQUEST, l3dss1_release_req},
	{ALL_STATES,
	 CC_RESTART | REQUEST, l3dss1_restart_req},
	{SBIT(6) | SBIT(25),
	 CC_SETUP | RESPONSE, l3dss1_release_cmpl_req},
	{SBIT(1) | SBIT(2),
	 CC_PROCEEDING | REQUEST, l3dss1_proceed_req},
	{SBIT(1),
	 CC_SETUP_ACKNOWLEDGE | REQUEST, l3dss1_setup_ack_req},
	{SBIT(2) | SBIT(3),
	 CC_ALERTING | REQUEST, l3dss1_alert_req},
	{SBIT(2) | SBIT(3) | SBIT(4),
	 CC_CONNECT | REQUEST, l3dss1_connect_req},
	{SBIT(8),
	 CC_CONNECT | REQUEST, l3dss1_connect_res},
	{SBIT(8),
	 CC_CONNECT | RESPONSE, l3dss1_connect_res},
	{ALL_STATES,
	 CC_FACILITY | REQUEST, l3dss1_facility_req},
	{SBIT(4) | SBIT(7) | SBIT(8) | SBIT(10),
	 CC_USER_INFORMATION | REQUEST, l3dss1_userinfo_req},
	{SBIT(3) | SBIT(4) | SBIT(10) | SBIT(12),
	 CC_HOLD_ACKNOWLEDGE | REQUEST, l3dss1_holdack_req},
	{SBIT(3) | SBIT(4) | SBIT(10) | SBIT(12),
	 CC_HOLD_REJECT | REQUEST, l3dss1_holdrej_req},
	{SBIT(3) | SBIT(4) | SBIT(10) | SBIT(12),
	 CC_RETRIEVE_ACKNOWLEDGE | REQUEST, l3dss1_retrack_req},
	{SBIT(3) | SBIT(4) | SBIT(10) | SBIT(12),
	 CC_RETRIEVE_REJECT | REQUEST, l3dss1_retrrej_req},
	{SBIT(15),
	 CC_SUSPEND_ACKNOWLEDGE | REQUEST, l3dss1_suspack_req},
	{SBIT(15),
	 CC_SUSPEND_REJECT | REQUEST, l3dss1_susprej_req},
	{SBIT(17),
	 CC_RESUME_ACKNOWLEDGE | REQUEST, l3dss1_resack_req},
	{SBIT(17),
	 CC_RESUME_REJECT | REQUEST, l3dss1_resrej_req},
};

#define DOWNSLLEN \
	(sizeof(downstatelist) / sizeof(struct stateentry))


static int
dl_data_mux(layer3_t *l3, mISDN_head_t *hh, struct sk_buff *skb)
{
	l3_process_t	*proc;
	int		ret = -EINVAL;
	int		cr;
	struct _l3_msg	l3m;

	if (!l3)
		return(ret);

	if (l3->debug)
		l3_debug(l3, "dl_data_mux NT: pr=%x dinfo=%x skb=%p len=%d",
			hh->prim, hh->dinfo, skb, skb->len);

	if (skb->len < 3) {
		l3_debug(l3, "ndss1 frame too short(%d)", skb->len);
		dev_kfree_skb(skb);
		return(0);
	}
	if (skb->data[0] != PROTO_DIS_EURO) { 
		if (l3->debug & L3_DEB_PROTERR) {
			l3_debug(l3, "ndss1%sunexpected discriminator %x message len %d",
				(hh->prim == (DL_DATA | INDICATION)) ? " " : "(broadcast) ",
				skb->data[0], skb->len);
		}
		dev_kfree_skb(skb);
		return(0);
	}
	cr = getcallref(skb->data);
	if (skb->len < ((skb->data[1] & 0x0f) + 3)) {
		l3_debug(l3, "ndss1 frame too short(%d)", skb->len);
		dev_kfree_skb(skb);
		return(0);
	}
	l3m.mt = skb->data[skb->data[1] + 2];
	if ((ret = parseQ931(skb)) < 0) {
		if (l3->debug & L3_DEB_PROTERR)
			l3_debug(l3, "dss1up: parse IE error %d", ret);
		printk(KERN_WARNING "dss1up: parse IE error %d\n", ret);
		dev_kfree_skb(skb);
		return(0);
	}
	l3m.skb = skb;
	if (cr == -2) {  /* wrong Callref */
		if (l3->debug & L3_DEB_WARN)
			l3_debug(l3, "ndss1 wrong Callref");
		dev_kfree_skb(skb);
		return(0);
	} else if (cr == -1) {  /* Dummy Callref */
		if (l3m.mt == MT_FACILITY) {
			l3_process_t dummy;
			memset( &dummy, 0, sizeof(l3_process_t));
			dummy.l3 = l3;
			dummy.ces = 0;
			dummy.callref = -1;
			l3dss1_facility(&dummy, hh->prim, skb);
		}
		else if (l3->debug & L3_DEB_WARN)
			l3_debug(l3, "dss1 dummy Callref (no facility msg)");
		dev_kfree_skb(skb);
		return(0);
	} else if ((((skb->data[1] & 0x0f) == 1) && (0==(cr & 0x7f))) ||
		(((skb->data[1] & 0x0f) == 2) && (0==(cr & 0x7fff)))) {
		/* Global CallRef */
		if (l3->debug & L3_DEB_STATE)
			l3_debug(l3, "ndss1 Global CallRef");
//		global_handler(l3, l3m.mt, msg);

		if (l3m.mt == MT_RESTART)  {
			l3_process_t dummy;
			memset( &dummy, 0, sizeof(l3_process_t));
			dummy.l3 = l3;
			dummy.ces = 0;
			dummy.callref = 0;
			l3dss1_restart(&dummy, hh->prim, skb);
		}
		dev_kfree_skb(skb);
		return(0);
	}
	proc = find_proc(&l3->proclist, hh->dinfo, cr);
	if (!proc) {
		l3_debug(l3, "dl_data_mux: no proc found for MT=%x", l3m.mt);
		if (l3m.mt == MT_SETUP || l3m.mt == MT_RESUME) {
			/* Setup/Resume creates a new transaction process */
			if (skb->data[2] & 0x80) {
				/* Setup/Resume with wrong CREF flag */
				if (l3->debug & L3_DEB_STATE)
					l3_debug(l3, "ndss1 wrong CRef flag");
				dev_kfree_skb(skb);
				return(0);
			}
			if (!(proc = create_proc(l3, hh->dinfo, cr, NULL))) {
				/* May be to answer with RELEASE_COMPLETE and
				 * CAUSE 0x2f "Resource unavailable", but this
				 * need a new_l3_process too ... arghh
				 */
				dev_kfree_skb(skb);
				return(0);
			}
			proc->l4id = proc->ces | (proc->callref << 16);
			list_add_tail(&proc->list, &l3->proclist);
			/* register this ID in L4 */
			ret = mISDN_queue_data(&proc->l3->inst, FLG_MSG_UP,
				CC_NEW_CR | INDICATION, proc->l4id, 0, NULL, 0);
			if (ret) {
				printk(KERN_WARNING "ndss1up: cannot register ID(%x)\n",
					proc->l4id);
				dev_kfree_skb(skb);
				send_proc(proc, IMSG_END_PROC, NULL);
				return(0);
			}
		} else {
			// it happens that a response to an outgoing setup is received after connect of another terminal. in this case we must release.
			dev_kfree_skb(skb);
			return(0);
		}
	}
	if (((proc->ces & 0xffffff00) == 0xff00) &&
		(!test_bit(FLG_PTP, &l3->Flag))) { 
		send_proc(proc, IMSG_MASTER_L2_DATA, &l3m);
	} else {
		send_proc(proc, IMSG_L2_DATA, &l3m);
	}
	dev_kfree_skb(skb);
	return(0);
}

static int
remove_proc(struct list_head *listhead, int ces)
{
	int found = 1;
	int any = 0;
	l3_process_t *proc;
	l3_process_t *nproc;

	if (ces > 126)
		return(0);

	while(found) {
		found = 0;

		list_for_each_entry_safe(proc, nproc, listhead, list) {
			if (proc->ces == ces) {
				if (proc->master)
					send_proc(proc, IMSG_END_PROC_M, NULL);
				else
					send_proc(proc, IMSG_END_PROC, NULL);
				any = 1;
				found = 1;
				break;
			}
			if (!list_empty(&proc->childlist)) {
				if (remove_proc(&proc->childlist, ces)) {
					any = 1;
					found = 1;
					break;
				}
			}
		}
	}
	return(any);
}

static int
l3_msg(layer3_t *l3, u_int pr, int dinfo, void *arg)
{
	struct sk_buff	*skb = arg;
	int	ces = dinfo & 0xffff;

	if (test_bit(FLG_PTP, &l3->Flag))
		dinfo=0;

	switch (pr) {
		case (DL_UNITDATA | REQUEST):
			return(l3down(l3, pr, dinfo, arg));
		case (DL_DATA | REQUEST):
			if (l3->l2_state0 == ST_L3_LC_ESTAB || ces > 0) {
				return(l3down(l3, pr, dinfo, arg));
			} else {
				if (ces == 0) {
					mISDN_sethead(pr, dinfo, skb);
					skb_queue_tail(&l3->squeue, skb);
					l3->l2_state0 = ST_L3_LC_ESTAB_WAIT;
					l3down(l3, DL_ESTABLISH | REQUEST, dinfo, NULL);
					return(0);
				}
			}
			break;
		case (DL_DATA | CONFIRM):
			break;
		case (DL_ESTABLISH | REQUEST):
			if (ces == 0) {
				if (l3->l2_state0 != ST_L3_LC_ESTAB) {
					l3down(l3, pr, dinfo, NULL);
					l3->l2_state0 = ST_L3_LC_ESTAB_WAIT;
				}
			}
			break;
		case (DL_ESTABLISH | CONFIRM):
			if (l3->l2_state0 != ST_L3_LC_REL_WAIT) {
				l3->l2_state0 = ST_L3_LC_ESTAB;
				send_squeue(l3);
			}
			mISDN_queue_data(&l3->inst, FLG_MSG_UP, pr, dinfo, 0, NULL, 0);
			break;
		case (DL_ESTABLISH | INDICATION):
			if (ces == 0) {
				if (l3->l2_state0 == ST_L3_LC_REL) {
					l3->l2_state0 = ST_L3_LC_ESTAB;
					send_squeue(l3);
				}
			}
			mISDN_queue_data(&l3->inst, FLG_MSG_UP, pr, dinfo, 0, NULL, 0);
			break;
		case (DL_RELEASE | INDICATION):
			if (ces == 0) {
				if (l3->l2_state0 == ST_L3_LC_ESTAB) {
					l3->l2_state0 = ST_L3_LC_REL;
				}
			}
			mISDN_queue_data(&l3->inst, FLG_MSG_UP, pr, dinfo, 0, NULL, 0);

			if ( ! (
#if 0
				l3->nst->feature& FEATURE_NET_KEEPCALLS &&
#endif
				(test_bit(FLG_PTP, &l3->Flag))) ) {
				remove_proc(&l3->proclist, dinfo);
			}
			break;
		case (DL_RELEASE | CONFIRM):
			if (ces == 0) {
				if (l3->l2_state0 == ST_L3_LC_REL_WAIT) {
					l3->l2_state0 = ST_L3_LC_REL;
				}
			}
			mISDN_queue_data(&l3->inst, FLG_MSG_UP, pr, dinfo, 0, NULL, 0);
			remove_proc(&l3->proclist, dinfo);
			break;
		case (DL_RELEASE | REQUEST):
			if (ces == 0) {
				if (l3->l2_state0 == ST_L3_LC_ESTAB) {
					l3down(l3, pr, dinfo, NULL);
					l3->l2_state0 = ST_L3_LC_REL_WAIT;
				}
			}
			break;
	}
	if (skb)
		dev_kfree_skb(skb);
	return(0);
}

static int
ndss1_fromdown(layer3_t *l3, struct sk_buff *skb, mISDN_head_t *hh)
{
	int ret = -EINVAL;

	if (hh->prim == (DL_DATA | INDICATION)) {
		ret = dl_data_mux(l3, hh, skb);
	} else {
		ret = l3_msg(l3, hh->prim, hh->dinfo, skb);
	}

	if (ret)
		dev_kfree_skb(skb);

	return(0);
}

static int
ndss1_fromup(layer3_t *l3, struct sk_buff *skb, mISDN_head_t *hh)
{
	l3_process_t	*proc;
	struct _l3_msg	l3m;

	proc = find_proc4l4id(&l3->proclist, hh->dinfo);

	if (debug & 0x2)
		l3_debug(l3, "ndss1_fromup: dinfo=0x%x proc=%p", hh->dinfo, proc);

	if (!proc) {
		switch (hh->prim) {
			case CC_RESTART | REQUEST:
				{
					l3_process_t dummy;
					memset( &dummy, 0, sizeof(l3_process_t));
					dummy.l3 = l3;
					dummy.ces = 0;
					dummy.callref = 0;
					l3dss1_restart_req(&dummy, hh->prim, skb); 
					dev_kfree_skb(skb);
					return(0);
				}
			break;
			case CC_SETUP | REQUEST:
			{
				l3->next_cr++;
				if (test_bit(FLG_CRLEN2, &l3->Flag)) {
					if (l3->next_cr>32766)
						l3->next_cr = 1;
				} else {
					if (l3->next_cr>126)
						l3->next_cr = 1;
				}
				proc = create_proc(l3, /* hh->dinfo & */ 0xffff,
					l3->next_cr | 0x8000, NULL);
				if (!proc) {
					dev_kfree_skb(skb);
					return(0);
				}
				proc->l4id = hh->dinfo;
				list_add_tail(&proc->list, &l3->proclist);
				/*
				l4id = proc->ces | (proc->callref << 16);
				mISDN_queue_data(&proc->l3->inst, FLG_MSG_UP,
					CC_SETUP | CONFIRM, hh->dinfo, sizeof(int), &l4id, 0);
				*/
			}
			break;
			case DL_ESTABLISH | REQUEST: 
				if (test_bit(FLG_PTP, &l3->Flag)) {
					l3down(l3, DL_ESTABLISH | REQUEST, 0, NULL);
					dev_kfree_skb(skb);
					return 0;
				}
			break;
			default:
			break;
		}
	}
	if (!proc) {
		dev_kfree_skb(skb);
		return(0);
	}
	l3m.mt = hh->prim;
	if (skb->len)
		l3m.skb = skb;
	else {
		l3m.skb = NULL;
	}
	send_proc(proc, IMSG_L4_DATA, &l3m);
	dev_kfree_skb(skb);
	return(0);
}

static int
imsg_intrelease(l3_process_t *master, l3_process_t *child)
{
	int	cause;

	if ((!master) || (!child))
		return(-EINVAL);

	if (master->l3->debug & L3_DEB_STATE) {
		l3_debug(master->l3, "imsg_intrelease: state=%d",
			master->state);
	}

	switch (master->state) {
		case 0:
			if (list_empty(&master->childlist)) {
				send_proc(master, IMSG_END_PROC, NULL);
			}
			break;
		case 6:
		case 10:
			break;
		case 19:
			send_proc(master, IMSG_END_PROC, NULL);
			break;
		case 7:
		case 9:
		case 25:
			if (!list_empty(&master->childlist) ||
				test_bit(FLG_L3P_TIMER312, &master->Flags)) {
			} else {
				send_proc(master, IMSG_END_PROC, NULL);
			}
			break;
		case 8:
			if (master->selces == child->ces) {
				cause = CAUSE_NONSELECTED_USER;
				send_proc(master, IMSG_RELEASE_CHILDS, &cause);
				if (test_bit(FLG_L3P_TIMER312, &master->Flags)) {
					newl3state(master, 22);
				} else {
					if (list_empty(&master->childlist))
						send_proc(master, IMSG_END_PROC, NULL);
				}
			}
			break;
		case 22:
			if (list_empty(&master->childlist))
				send_proc(master, IMSG_END_PROC, NULL);
			break;
	}
	return(0);
}

int send_proc(l3_process_t *proc, int op, void *arg)
{
	int		i;
	l3_process_t	*selp;
	struct _l3_msg	*l3m = arg;
	struct _l3_msg	l3msg;

	if (proc->l3 && proc->l3->debug & L3_DEB_PROC) {
		l3_debug(proc->l3, "%s: proc(%x,%d) op(%d)", __FUNCTION__,
			proc->ces, proc->callref, op);  
	}

	switch(op) {
		case IMSG_END_PROC:
		case IMSG_END_PROC_M:
			StopAllL3Timer_net(proc);
			if (!proc->master && !arg) {
				send_release_ind(proc);
			}
			while (!list_empty(&proc->childlist)) {
				send_proc(list_first_entry(&proc->childlist, l3_process_t, list), IMSG_END_PROC, NULL);
			}
			list_del(&proc->list);
			if (proc->master) {
				if (op == IMSG_END_PROC_M)
					imsg_intrelease(proc->master, proc);
			}
			kfree(proc);
			break;
		case IMSG_L2_DATA:
			for (i = 0; i < DATASLLEN; i++)
				if ((l3m->mt == datastatelist[i].primitive) &&
					((1 << proc->state) & datastatelist[i].state))
				break;
			if (i == DATASLLEN) {
				if (proc->l3->debug & L3_DEB_STATE) {
					l3_debug(proc->l3, "dss1 state %d mt %#x unhandled",
						proc->state, l3m->mt);
				}
				if ((MT_RELEASE_COMPLETE != l3m->mt) && (MT_RELEASE != l3m->mt)) {
		//			l3dss1_status_send(proc, CAUSE_NOTCOMPAT_STATE);
				}
			} else {
				if (proc->l3->debug & L3_DEB_STATE) {
					l3_debug(proc->l3, "dss1 state %d mt %x",
						proc->state, l3m->mt);
				}
				show_ddata("fromL2:", l3m->skb);
				datastatelist[i].rout(proc, l3m->mt, l3m->skb);
			}
			break;
		case IMSG_MASTER_L2_DATA:
			for (i = 0; i < MDATASLLEN; i++)
				if ((l3m->mt == mdatastatelist[i].primitive) &&
					((1 << proc->state) & mdatastatelist[i].state))
				break;
			if (i == MDATASLLEN) {
				if (proc->l3->debug & L3_DEB_STATE) {
					l3_debug(proc->l3, "dss1 state %d mt %#x unhandled",
						proc->state, l3m->mt);
				}
				if ((MT_RELEASE_COMPLETE != l3m->mt) && (MT_RELEASE != l3m->mt)) {
		//			l3dss1_status_send(proc, CAUSE_NOTCOMPAT_STATE);
				}
			} else {
				if (proc->l3->debug & L3_DEB_STATE) {
					l3_debug(proc->l3, "dss1 state %d mt %x",
						proc->state, l3m->mt);
				}
				show_ddata("fromL2M:", l3m->skb);
				mdatastatelist[i].rout(proc, l3m->mt, l3m->skb);
			}
			break;
		case IMSG_TIMER_EXPIRED:
			i = *((int *)arg);
			l3_debug(proc->l3, "%s: timer %x", __FUNCTION__, i);
			l3m = &l3msg;
			l3m->mt = CC_TIMER | (i<<8);
			l3m->skb = NULL;
		case IMSG_L4_DATA:
			for (i = 0; i < DOWNSLLEN; i++)
				if ((l3m->mt == downstatelist[i].primitive) &&
					((1 << proc->state) & downstatelist[i].state))
				break;
			if (i == DOWNSLLEN) {
				if (proc->l3->debug & L3_DEB_STATE) {
					l3_debug(proc->l3, "dss1 state %d L4 %#x unhandled",
						proc->state, l3m->mt);
				}
			} else {
				if (proc->l3->debug & L3_DEB_STATE) {
					l3_debug(proc->l3, "dss1 state %d L4 %x",
						proc->state, l3m->mt);
				}
				if (l3m->skb) {
					show_ddata("fromL4:", l3m->skb);
					downstatelist[i].rout(proc, l3m->mt, l3m->skb);
				} else {
					downstatelist[i].rout(proc, l3m->mt, NULL);
				}
			}
			break;
		case IMSG_CONNECT_IND:
			selp = proc;
			proc = proc->master;
			if (!proc)
				return(-EINVAL);
			proc->selces = selp->ces;
			newl3state(proc, 8);
			return(mISDN_l3up(proc, arg));
		case IMSG_SEL_PROC:
			selp = find_proc(&proc->childlist, proc->selces, proc->callref);
			i = proc->selces | (proc->callref << 16);
			mISDN_queue_data(&proc->l3->inst, FLG_MSG_UP,
				CC_NEW_CR | INDICATION, proc->l4id /* proc->ces |
				(proc->callref << 16) */, sizeof(int), &i, 0);
			proc->ces = proc->selces;
			send_proc(selp, IMSG_END_PROC, NULL);
			break;
		case IMSG_RELEASE_CHILDS:
			{
				u_char cause[4];

				l3msg.mt = CC_RELEASE | REQUEST;
				l3msg.skb = mISDN_alloc_l3msg(10, MT_RELEASE);
				if (!l3msg.skb)
					return(-ENOMEM);
				cause[0] = 2;
				cause[1] = 0x80 | CAUSE_LOC_PNET_LOCUSER;
				cause[2] = 0x80 | *((int *)arg);
				mISDN_AddIE(l3msg.skb, IE_CAUSE, cause);
				
				list_for_each_entry(selp, &proc->childlist, list) {
					send_proc(selp, IMSG_L4_DATA, &l3msg);
				}
				dev_kfree_skb(l3msg.skb);
			}
			break;
	}
	return(0);
}

static int
ndss1_function(mISDNinstance_t *inst, struct sk_buff *skb)
{
	layer3_t	*l3;
	int		ret = -EINVAL;
	mISDN_head_t	*hh;

	l3 = inst->privat;
	hh = mISDN_HEAD_P(skb);
	if (debug)
		l3_debug(l3, "%s: addr(%08x) prim(%x) dinfo(%x)", __FUNCTION__,
			hh->addr, hh->prim, hh->dinfo);
	if (!l3)
		return(ret);

	switch(hh->addr & MSG_DIR_MASK) {
		case FLG_MSG_DOWN:
			ret = ndss1_fromup(l3, skb, hh);
			break;
		case FLG_MSG_UP:
			ret = ndss1_fromdown(l3, skb, hh);
			break;
		case MSG_TO_OWNER:
			/* FIXME: must be handled depending on type */
			int_errtxt("not implemented yet");
			break;
		default:
			/* FIXME: broadcast must be handled depending on type */
			if ((hh->prim & MISDN_CMD_MASK) == MGR_SHORTSTATUS) {
				ret = -EOPNOTSUPP;
				break;
			}
			int_errtxt("not implemented yet");
			break;
	}
	return(ret);
}

static void
release_ndss1(layer3_t *l3)
{
	mISDNinstance_t  *inst = &l3->inst;
	u_long		flags;

	if (debug)	
		l3_debug(l3, "release_ndss1 refcnt %d l3(%p) inst(%p)",
			n_dss1.refcnt, l3, inst);
	release_l3_net(l3);

	spin_lock_irqsave(&n_dss1.lock, flags);
	list_del(&l3->list);
	spin_unlock_irqrestore(&n_dss1.lock, flags);

	mISDN_ctrl(inst, MGR_UNREGLAYER | REQUEST, NULL);
	if (l3->entity != MISDN_ENTITY_NONE)
		mISDN_ctrl(inst, MGR_DELENTITY | REQUEST, (void *)((u_long)l3->entity));
	kfree(l3);
}

static int
new_ndss1(mISDNstack_t *st, mISDN_pid_t *pid)
{
	layer3_t	*nl3;
	int		err;
	u_long		flags;

	if (!st || !pid)
		return(-EINVAL);
	if (!(nl3 = kzalloc(sizeof(layer3_t), GFP_ATOMIC))) {
		printk(KERN_ERR "kmalloc layer3 failed\n");
		return(-ENOMEM);
	}
	memcpy(&nl3->inst.pid, pid, sizeof(mISDN_pid_t));
	nl3->debug = debug;
	nl3->instp = &nl3->inst;
	mISDN_init_instance(&nl3->inst, &n_dss1, nl3, ndss1_function);
	if (!mISDN_SetHandledPID(&n_dss1, &nl3->inst.pid)) {
		int_error();
		return(-ENOPROTOOPT);
	}
	if ((pid->protocol[3] & ~ISDN_PID_FEATURE_MASK) != ISDN_PID_L3_DSS1NET) {
		printk(KERN_ERR "ndss1 create failed prt %x\n",
			pid->protocol[3]);
		kfree(nl3);
		return(-ENOPROTOOPT);
	}
	init_l3_net(nl3);
	if (pid->protocol[3] & ISDN_PID_L3_DF_PTP)
		test_and_set_bit(FLG_PTP, &nl3->Flag);
	if (pid->protocol[3] & ISDN_PID_L3_DF_EXTCID)
		test_and_set_bit(FLG_EXTCID, &nl3->Flag);
	if (pid->protocol[3] & ISDN_PID_L3_DF_CRLEN2)
		test_and_set_bit(FLG_CRLEN2, &nl3->Flag);

	sprintf(nl3->inst.name, "NDSS1 %x", st->id >> 8);
	spin_lock_irqsave(&n_dss1.lock, flags);
	list_add_tail(&nl3->list, &n_dss1.ilist);
	spin_unlock_irqrestore(&n_dss1.lock, flags);
	err = mISDN_ctrl(&nl3->inst, MGR_NEWENTITY | REQUEST, NULL);
	if (err) {
		printk(KERN_WARNING "mISDN %s: MGR_NEWENTITY REQUEST failed err(%d)\n",
			__FUNCTION__, err);
	}
	err = mISDN_ctrl(st, MGR_REGLAYER | INDICATION, &nl3->inst);
	if (err) {
		release_l3_net(nl3);
		list_del(&nl3->list);
		kfree(nl3);
	} else {
		mISDN_stPara_t	stp;

	    	if (st->para.down_headerlen)
		    	nl3->down_headerlen = st->para.down_headerlen;
		stp.maxdatalen = 0;
		stp.up_headerlen = L3_EXTRA_SIZE;
		stp.down_headerlen = 0;
		mISDN_ctrl(st, MGR_ADDSTPARA | REQUEST, &stp);
	}
	if (debug & 0x1)
		l3_debug(nl3, "new_ndss1: %s", nl3->inst.name);
	return(err);
}

static char MName[] = "NDSS1";

#ifdef MODULE
MODULE_AUTHOR("Armin Schindler");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
#ifdef OLD_MODULE_PARAM
MODULE_PARM(debug, "1i");
#else
module_param(debug, uint, S_IRUGO | S_IWUSR);
#endif
#endif

static int
ndss1_manager(void *data, u_int prim, void *arg) {
	mISDNinstance_t *inst = data;
	layer3_t	*l3l = NULL;
	u_long		flags;

	if (debug & MISDN_DEBUG_MANAGER)
		printk(KERN_DEBUG "ndss1_manager data:%p prim:%x arg:%p\n", data, prim, arg);
	if (!data)
		return(-EINVAL);
	spin_lock_irqsave(&n_dss1.lock, flags);
	if (!list_empty(&n_dss1.ilist)) {
		list_for_each_entry(l3l, &n_dss1.ilist, list) {
			if (&l3l->inst == inst)
				break;
		}
	}
	spin_unlock_irqrestore(&n_dss1.lock, flags);
	if (prim == (MGR_NEWLAYER | REQUEST))
		return(new_ndss1(data, arg));
	if (!l3l) {
		if (debug & 0x1)
			printk(KERN_WARNING "ndss1_manager prim(%x) no instance\n", prim);
		return(-EINVAL);
	}
	switch(prim) {
	    case MGR_NEWENTITY | CONFIRM:
		l3l->entity = (u_long)arg & 0xffffffff;
		break;
	    case MGR_ADDSTPARA | INDICATION:
	    	l3l->down_headerlen = ((mISDN_stPara_t *)arg)->down_headerlen;
	    case MGR_CLRSTPARA | INDICATION:
		break;
#ifdef FIXME
	    case MGR_CONNECT | REQUEST:
		return(mISDN_ConnectIF(inst, arg));
	    case MGR_SETIF | REQUEST:
	    case MGR_SETIF | INDICATION:
		return(mISDN_SetIF(inst, arg, prim, ndss1_fromup, ndss1_fromdown, l3l));
	    case MGR_DISCONNECT | REQUEST:
	    case MGR_DISCONNECT | INDICATION:
		return(mISDN_DisConnectIF(inst, arg));
#endif
	    case MGR_RELEASE | INDICATION:
	    case MGR_UNREGLAYER | REQUEST:
	    	if (debug & MISDN_DEBUG_MANAGER)
			printk(KERN_DEBUG "release_ndss1 id %x\n", l3l->inst.st->id);
	    	release_ndss1(l3l);
	    	break;
	    PRIM_NOT_HANDLED(MGR_CTRLREADY | INDICATION);
	    default:
	    	if (debug & 0x1)
				printk(KERN_DEBUG "ndss1 prim %x not handled\n", prim);
		return(-EINVAL);
	}
	return(0);
}

int NDSS1Init(void)
{
	int err;
	char tmp[32];

	strcpy(tmp, dss1_revision);
	printk(KERN_INFO "mISDN: NT DSS1 Rev. %s\n", mISDN_getrev(tmp));
#ifdef MODULE
	n_dss1.owner = THIS_MODULE;
#endif
	spin_lock_init(&n_dss1.lock);
	INIT_LIST_HEAD(&n_dss1.ilist);
	n_dss1.name = MName;
	n_dss1.DPROTO.protocol[3] = ISDN_PID_L3_DSS1NET |
		ISDN_PID_L3_DF_PTP |
		ISDN_PID_L3_DF_EXTCID |
		ISDN_PID_L3_DF_CRLEN2;
	n_dss1.own_ctrl = ndss1_manager;
	if ((err = mISDN_register(&n_dss1))) {
		printk(KERN_ERR "Can't register %s error(%d)\n", MName, err);
	} else
		mISDN_module_register(THIS_MODULE);
	return(err);
}

#ifdef MODULE
void NDSS1_cleanup(void)
{
	int err;
	layer3_t	*l3, *next;

	mISDN_module_unregister(THIS_MODULE);

	if ((err = mISDN_unregister(&n_dss1))) {
		printk(KERN_ERR "Can't unregister NT DSS1 error(%d)\n", err);
	}
	if (!list_empty(&n_dss1.ilist)) {
		printk(KERN_WARNING "mISDNl3 n_dss1 list not empty\n");
		list_for_each_entry_safe(l3, next, &n_dss1.ilist, list)
			release_ndss1(l3);
	}
}

module_init(NDSS1Init);
module_exit(NDSS1_cleanup);
#endif
