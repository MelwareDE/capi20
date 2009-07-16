/*
 * $Id: layer2_net.c,v 1.10 2008-12-18 17:04:49 armin Exp $
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 * Adapted for NT mode    Armin Schindler (armin@melware.de)
 *
 * This file is released under the GPLv2
 *
 */
#include <linux/module.h>
#include "core.h"
#include "layer2_net.h"
#include "helper.h"
#include "debug.h"

static char *l2_revision = "$Revision: 1.10 $";

static void l2m_debug(struct FsmInst *fi, char *fmt, ...);

static int debug = 0;
static mISDNobject_t isdnl2;

static
struct Fsm l2fsm = {NULL, 0, 0, NULL, NULL};

static layer2_t *new_dl2_net(mISDNinstance_t *ninst, int tei);

enum {
	ST_L2_1,
	ST_L2_2,
	ST_L2_3,
	ST_L2_4,
	ST_L2_5,
	ST_L2_6,
	ST_L2_7,
	ST_L2_8,
};

#define L2_STATE_COUNT (ST_L2_8+1)

static char *strL2State[] =
{
	"ST_L2_1",
	"ST_L2_2",
	"ST_L2_3",
	"ST_L2_4",
	"ST_L2_5",
	"ST_L2_6",
	"ST_L2_7",
	"ST_L2_8",
};

enum {
	EV_L2_UI,
	EV_L2_SABME,
	EV_L2_DISC,
	EV_L2_DM,
	EV_L2_UA,
	EV_L2_FRMR,
	EV_L2_SUPER,
	EV_L2_I,
	EV_L2_DL_DATA,
	EV_L2_ACK_PULL,
	EV_L2_DL_UNITDATA,
	EV_L2_DL_ESTABLISH_REQ,
	EV_L2_DL_RELEASE_REQ,
	EV_L2_MDL_ASSIGN,
	EV_L2_MDL_REMOVE,
	EV_L2_MDL_ERROR,
	EV_L1_DEACTIVATE,
	EV_L2_T200,
	EV_L2_T203,
	EV_L2_SET_OWN_BUSY,
	EV_L2_CLEAR_OWN_BUSY,
	EV_L2_FRAME_ERROR,
};

#define L2_EVENT_COUNT (EV_L2_FRAME_ERROR+1)

static char *strL2Event[] =
{
	"EV_L2_UI",
	"EV_L2_SABME",
	"EV_L2_DISC",
	"EV_L2_DM",
	"EV_L2_UA",
	"EV_L2_FRMR",
	"EV_L2_SUPER",
	"EV_L2_I",
	"EV_L2_DL_DATA",
	"EV_L2_ACK_PULL",
	"EV_L2_DL_UNITDATA",
	"EV_L2_DL_ESTABLISH_REQ",
	"EV_L2_DL_RELEASE_REQ",
	"EV_L2_MDL_ASSIGN",
	"EV_L2_MDL_REMOVE",
	"EV_L2_MDL_ERROR",
	"EV_L1_DEACTIVATE",
	"EV_L2_T200",
	"EV_L2_T203",
	"EV_L2_SET_OWN_BUSY",
	"EV_L2_CLEAR_OWN_BUSY",
	"EV_L2_FRAME_ERROR",
};

inline u_int
l2headersize(layer2_t *l2, int ui)
{
	return ((test_bit(FLG_MOD128, &l2->flag) && (!ui)) ? 2 : 1) +
		(test_bit(FLG_LAPD, &l2->flag) ? 2 : 1);
}

inline u_int
l2addrsize(layer2_t *l2)
{
	return test_bit(FLG_LAPD, &l2->flag) ? 2 : 1;
}

static int
l2up(layer2_t *l2, u_int prim, int dinfo, struct sk_buff *skb)
{
	if (debug & 0x02)
		printk(KERN_DEBUG "l2up NT: %s prim=%x dinfo=%x len=%d\n",
			l2->instp->name, prim, dinfo, skb->len);
	return mISDN_queueup_newhead(l2->instp, 0, prim, dinfo, skb);
}

static int
l2up_create(layer2_t *l2, u_int prim, int dinfo, int len, void *arg)
{
	return mISDN_queue_data(l2->instp,
	    FLG_MSG_UP, prim, dinfo, len, arg, 0);
}

static int
l2down_skb(layer2_t *l2, struct sk_buff *skb) {
	int ret;

	ret = mISDN_queue_down(l2->instp, 0, skb);
	if (ret && l2->debug)
		printk(KERN_DEBUG "l2down_skb NT: ret(%d)\n", ret);
	return ret;
}

static int
l2down_raw(layer2_t *l2, struct sk_buff *skb)
{
#if 0
	/* do we need L1_BUSY in NT mode? */
	mISDN_head_t *hh = mISDN_HEAD_P(skb);

	if (hh->prim == PH_DATA_REQ) {
		if (test_and_set_bit(FLG_L1_BUSY, &l2->flag)) {
			skb_queue_tail(&l2->down_queue, skb);
			return 0;
		}
		l2->down_id = mISDN_HEAD_DINFO(skb);
	}
#endif
	return l2down_skb(l2, skb);
}

static int
l2down(layer2_t *l2, u_int prim, int dinfo, struct sk_buff *skb)
{
	mISDN_sethead(prim, dinfo, skb);
	return l2down_raw(l2, skb);
}

static int
l2down_create(layer2_t *l2, u_int prim, int dinfo, int len, void *arg)
{
	struct sk_buff	*skb;
	int		err;

	skb = create_link_skb(prim, dinfo, len, arg, 0);
	if (!skb)
		return -ENOMEM;
	err = l2down_raw(l2, skb);
	if (err)
		dev_kfree_skb(skb);
	return err;
}

static int
l2mgr(layer2_t *l2, u_int prim, void *arg) {
	long c = (long)arg;

	printk(KERN_WARNING
	    "l2mgr NT: addr:%x prim %x %c\n", l2->instp->id, prim, (char)c);

	return 0;
}

static void
set_peer_busy(layer2_t *l2) {
	test_and_set_bit(FLG_PEER_BUSY, &l2->flag);
	if (skb_queue_len(&l2->i_queue) || skb_queue_len(&l2->ui_queue))
		test_and_set_bit(FLG_L2BLOCK, &l2->flag);
}

static void
clear_peer_busy(layer2_t *l2) {
	if (test_and_clear_bit(FLG_PEER_BUSY, &l2->flag))
		test_and_clear_bit(FLG_L2BLOCK, &l2->flag);
}

static void
InitWin(layer2_t *l2)
{
	int i;

	for (i = 0; i < MAX_WINDOW; i++)
		l2->windowar[i] = NULL;
}

static int
freewin(layer2_t *l2)
{
	int i, cnt = 0;

	for (i = 0; i < MAX_WINDOW; i++) {
		if (l2->windowar[i]) {
			cnt++;
			dev_kfree_skb(l2->windowar[i]);
			l2->windowar[i] = NULL;
		}
	}
	return cnt;
}

static void
ReleaseWin(layer2_t *l2)
{
	int cnt;

	if ((cnt = freewin(l2)))
		printk(KERN_WARNING
		    "isdnl2 freed %d skbuffs in release\n", cnt);
}

inline unsigned int
cansend(layer2_t *l2)
{
	unsigned int p1;

	if (test_bit(FLG_MOD128, &l2->flag))
		p1 = (l2->vs - l2->va) % 128;
	else
		p1 = (l2->vs - l2->va) % 8;
	return ((p1 < l2->window) && !test_bit(FLG_PEER_BUSY, &l2->flag));
}

inline void
clear_exception(layer2_t *l2)
{
	test_and_clear_bit(FLG_ACK_PEND, &l2->flag);
	test_and_clear_bit(FLG_REJEXC, &l2->flag);
	test_and_clear_bit(FLG_OWN_BUSY, &l2->flag);
	clear_peer_busy(l2);
}

static int
sethdraddr(layer2_t *l2, u_char *header, int rsp)
{
	u_char *ptr = header;
	int crbit = rsp;

	if (test_bit(FLG_LAPD, &l2->flag)) {
		if (test_bit(FLG_LAPD_NET, &l2->flag))
			crbit = !crbit;
		*ptr++ = (l2->sapi << 2) | (crbit ? 2 : 0);
		*ptr++ = (l2->tei << 1) | 1;
		return 2;
	} else {
		if (test_bit(FLG_ORIG, &l2->flag))
			crbit = !crbit;
		if (crbit)
			*ptr++ = l2->addr.B;
		else
			*ptr++ = l2->addr.A;
		return 1;
	}
}

inline static void
enqueue_super(layer2_t *l2, struct sk_buff *skb)
{
	if (l2down(l2, PH_DATA | REQUEST, DINFO_SKB, skb))
		dev_kfree_skb(skb);
}

#define enqueue_ui(a, b) enqueue_super(a, b)

inline int
IsUI(u_char * data)
{
	return ((data[0] & 0xef) == UI);
}

inline int
IsUA(u_char * data)
{
	return ((data[0] & 0xef) == UA);
}

inline int
IsDM(u_char * data)
{
	return ((data[0] & 0xef) == DM);
}

inline int
IsDISC(u_char * data)
{
	return ((data[0] & 0xef) == DISC);
}

inline int
IsRR(u_char * data, layer2_t *l2)
{
	if (test_bit(FLG_MOD128, &l2->flag))
		return (data[0] == RR);
	else
		return ((data[0] & 0xf) == 1);
}

inline int
IsSFrame(u_char * data, layer2_t *l2)
{
	register u_char d = *data;

	if (!test_bit(FLG_MOD128, &l2->flag))
		d &= 0xf;
	return (((d & 0xf3) == 1) && ((d & 0x0c) != 0x0c));
}

inline int
IsSABME(u_char * data, layer2_t *l2)
{
	u_char d = data[0] & ~0x10;

	return (test_bit(FLG_MOD128, &l2->flag) ? d == SABME : d == SABM);
}

inline int
IsREJ(u_char * data, layer2_t *l2)
{
	return (test_bit(FLG_MOD128, &l2->flag) ?
	    data[0] == REJ : (data[0] & 0xf) == REJ);
}

inline int
IsFRMR(u_char * data)
{
	return ((data[0] & 0xef) == FRMR);
}

inline int
IsRNR(u_char * data, layer2_t *l2)
{
	return (test_bit(FLG_MOD128, &l2->flag) ?
	    data[0] == RNR : (data[0] & 0xf) == RNR);
}

static
int
iframe_error(layer2_t *l2, struct sk_buff *skb)
{
	u_int	i = l2addrsize(l2) + (test_bit(FLG_MOD128, &l2->flag) ?
	    2 : 1);
	int	rsp = *skb->data & 0x2;

	if (test_bit(FLG_ORIG, &l2->flag))
		rsp = !rsp;
	if (rsp)
		return 'L';
	if (skb->len < i)
		return 'N';
	if ((skb->len - i) > l2->maxlen)
		return 'O';
	return 0;
}

static
int
super_error(layer2_t *l2, struct sk_buff *skb)
{
	if (skb->len != l2addrsize(l2) +
	    (test_bit(FLG_MOD128, &l2->flag) ? 2 : 1))
		return 'N';
	return 0;
}

static
int
unnum_error(layer2_t *l2, struct sk_buff *skb, int wantrsp)
{
	int rsp = (*skb->data & 0x2) >> 1;
	if (test_bit(FLG_ORIG, &l2->flag))
		rsp = !rsp;
	if (rsp != wantrsp)
		return 'L';
	if (skb->len != l2addrsize(l2) + 1)
		return 'N';
	return 0;
}

static
int
UI_error(layer2_t *l2, struct sk_buff *skb)
{
	int rsp = *skb->data & 0x2;
	if (test_bit(FLG_ORIG, &l2->flag))
		rsp = !rsp;
	if (rsp)
		return 'L';
	if (skb->len > l2->maxlen + l2addrsize(l2) + 1)
		return 'O';
	return 0;
}

static
int
FRMR_error(layer2_t *l2, struct sk_buff *skb)
{
	u_int	headers = l2addrsize(l2) + 1;
	u_char	*datap = skb->data + headers;
	int	rsp = *skb->data & 0x2;

	if (test_bit(FLG_ORIG, &l2->flag))
		rsp = !rsp;
	if (!rsp)
		return 'L';
	if (test_bit(FLG_MOD128, &l2->flag)) {
		if (skb->len < headers + 5)
			return 'N';
		else if (l2->debug)
			l2m_debug(&l2->l2m,
			    "FRMR information %2x %2x %2x %2x %2x",
			    datap[0], datap[1], datap[2], datap[3], datap[4]);
	} else {
		if (skb->len < headers + 3)
			return 'N';
		else if (l2->debug)
			l2m_debug(&l2->l2m,
			    "FRMR information %2x %2x %2x",
			    datap[0], datap[1], datap[2]);
	}
	return 0;
}

static unsigned int
legalnr(layer2_t *l2, unsigned int nr)
{
	if (test_bit(FLG_MOD128, &l2->flag))
		return ((nr - l2->va) % 128) <= ((l2->vs - l2->va) % 128);
	else
		return ((nr - l2->va) % 8) <= ((l2->vs - l2->va) % 8);
}

static void
setva(layer2_t *l2, unsigned int nr)
{
	struct sk_buff	*skb;

	while (l2->va != nr) {
		l2->va++;
		if (test_bit(FLG_MOD128, &l2->flag))
			l2->va %= 128;
		else
			l2->va %= 8;
		if (l2->windowar[l2->sow]) {
			skb_trim(l2->windowar[l2->sow], 0);
			skb_queue_tail(&l2->tmp_queue, l2->windowar[l2->sow]);
			l2->windowar[l2->sow] = NULL;
		}
		l2->sow = (l2->sow + 1) % l2->window;
	}
	while ((skb = skb_dequeue(&l2->tmp_queue))) {
		if (l2up(l2, DL_DATA | CONFIRM, mISDN_HEAD_DINFO(skb), skb))
			dev_kfree_skb(skb);
	}
}

static void
send_uframe(layer2_t *l2, struct sk_buff *skb, u_char cmd, u_char cr)
{
	u_char tmp[MAX_HEADER_LEN];
	int i;

	i = sethdraddr(l2, tmp, cr);

	if (l2->debug)
		l2m_debug(&l2->l2m, "send_uframe: cmd=%x cr=%x i=%x",
			cmd, cr, i);

	tmp[i++] = cmd;
	if (skb)
		skb_trim(skb, 0);
	else if (!(skb = alloc_skb(i, GFP_ATOMIC))) {
		printk(KERN_WARNING "%s: can't alloc skbuff\n", __FUNCTION__);
		return;
	}
	memcpy(skb_put(skb, i), tmp, i);
	enqueue_super(l2, skb);
}


inline u_char
get_PollFlag(layer2_t *l2, struct sk_buff * skb)
{
	return (skb->data[l2addrsize(l2)] & 0x10);
}

inline u_char
get_PollFlagFree(layer2_t *l2, struct sk_buff *skb)
{
	u_char PF;

	PF = get_PollFlag(l2, skb);
	dev_kfree_skb(skb);
	return (PF);
}

inline void
start_t200(layer2_t *l2, int i)
{
	mISDN_FsmAddTimer(&l2->t200, l2->T200, EV_L2_T200, NULL, i);
	test_and_set_bit(FLG_T200_RUN, &l2->flag);
}

inline void
restart_t200(layer2_t *l2, int i)
{
	mISDN_FsmRestartTimer(&l2->t200, l2->T200, EV_L2_T200, NULL, i);
	test_and_set_bit(FLG_T200_RUN, &l2->flag);
}

inline void
stop_t200(layer2_t *l2, int i)
{
	if (test_and_clear_bit(FLG_T200_RUN, &l2->flag))
		mISDN_FsmDelTimer(&l2->t200, i);
}

inline void
st5_dl_release_l2l3(layer2_t *l2)
{
	int pr;

	if (test_and_clear_bit(FLG_PEND_REL, &l2->flag)) {
		pr = DL_RELEASE | CONFIRM;
	} else {
		pr = DL_RELEASE | INDICATION;
	}
	l2up_create(l2, pr, CES(l2), 0, NULL);
}

inline void
lapb_dl_release_l2l3(layer2_t *l2, int f)
{
	if (test_bit(FLG_LAPB, &l2->flag))
		l2down_create(l2, PH_DEACTIVATE | REQUEST, 0, 0, NULL);
	l2up_create(l2, DL_RELEASE | f, CES(l2), 0, NULL);
}

static void
establishlink(struct FsmInst *fi)
{
	layer2_t *l2 = fi->userdata;
	u_char cmd;

	clear_exception(l2);
	l2->rc = 0;
	cmd = (test_bit(FLG_MOD128, &l2->flag) ? SABME : SABM) | 0x10;
	send_uframe(l2, NULL, cmd, CMD);
	mISDN_FsmDelTimer(&l2->t203, 1);
	restart_t200(l2, 1);
	test_and_clear_bit(FLG_PEND_REL, &l2->flag);
	freewin(l2);
	mISDN_FsmChangeState(fi, ST_L2_5);
}

static void
l2_mdl_error_ua(struct FsmInst *fi, int event, void *arg)
{
	struct sk_buff *skb = arg;
	layer2_t *l2 = fi->userdata;

	if (get_PollFlagFree(l2, skb))
		l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'C');
	else
		l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'D');

}

static void
l2_mdl_error_dm(struct FsmInst *fi, int event, void *arg)
{
	struct sk_buff *skb = arg;
	layer2_t *l2 = fi->userdata;

	if (get_PollFlagFree(l2, skb))
		l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'B');
	else {
		l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'E');
		establishlink(fi);
		test_and_clear_bit(FLG_L3_INIT, &l2->flag);
	}
}

static void
l2_st8_mdl_error_dm(struct FsmInst *fi, int event, void *arg)
{
	struct sk_buff *skb = arg;
	layer2_t *l2 = fi->userdata;

	if (get_PollFlagFree(l2, skb))
		l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'B');
	else {
		l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'E');
	}
	establishlink(fi);
	test_and_clear_bit(FLG_L3_INIT, &l2->flag);
}

static void
l2_go_st3(struct FsmInst *fi, int event, void *arg)
{
	dev_kfree_skb((struct sk_buff *)arg);
	mISDN_FsmChangeState(fi, ST_L2_3);
}

static void
l2_mdl_assign(struct FsmInst *fi, int event, void *arg)
{
	layer2_t	*l2 = fi->userdata;
	struct sk_buff	*skb = arg;
	mISDN_head_t	*hh;

	mISDN_FsmChangeState(fi, ST_L2_3);
	skb_trim(skb, 0);
	hh = mISDN_HEAD_P(skb);
	hh->prim = MDL_ASSIGN | INDICATION;
	hh->dinfo = 0;
	if (l2_tei_net(l2->tm, skb))
		dev_kfree_skb(skb);
}

static void
l2_queue_ui_assign(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	skb_queue_tail(&l2->ui_queue, skb);
	mISDN_FsmChangeState(fi, ST_L2_2);
	if ((skb = create_link_skb(MDL_ASSIGN | INDICATION, 0, 0, NULL, 0))) {
		if (l2_tei_net(l2->tm, skb))
			dev_kfree_skb(skb);
	}
}

static void
l2_queue_ui(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	skb_queue_tail(&l2->ui_queue, skb);
}

static void
tx_ui(layer2_t *l2)
{
	struct sk_buff *skb;
	u_char header[MAX_HEADER_LEN];
	int i;

	i = sethdraddr(l2, header, CMD);
	if (test_bit(FLG_LAPD_NET, &l2->flag))
		header[1] = 0xff; /* tei 127 */
	header[i++] = UI;
	while ((skb = skb_dequeue(&l2->ui_queue))) {
		memcpy(skb_push(skb, i), header, i);
		enqueue_ui(l2, skb);
	}
}

static void
l2_send_ui(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	skb_queue_tail(&l2->ui_queue, skb);
	tx_ui(l2);
}

static void
l2_got_ui(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	skb_pull(skb, l2headersize(l2, 1));
/*
 *		in states 1-3 for broadcast
 */
	if (l2up(l2, DL_UNITDATA | INDICATION, CES(l2), skb))
		dev_kfree_skb(skb);
}

static void
l2_establish(struct FsmInst *fi, int event, void *arg)
{
	struct sk_buff *skb = arg;
	layer2_t *l2 = fi->userdata;

	establishlink(fi);
	test_and_set_bit(FLG_L3_INIT, &l2->flag);
	dev_kfree_skb(skb);
}

static void
l2_discard_i_setl3(struct FsmInst *fi, int event, void *arg)
{
	struct sk_buff *skb = arg;
	layer2_t *l2 = fi->userdata;

	discard_queue(&l2->i_queue);
	test_and_set_bit(FLG_L3_INIT, &l2->flag);
	test_and_clear_bit(FLG_PEND_REL, &l2->flag);
	dev_kfree_skb(skb);
}

static void
l2_l3_reestablish(struct FsmInst *fi, int event, void *arg)
{
	struct sk_buff *skb = arg;
	layer2_t *l2 = fi->userdata;

	discard_queue(&l2->i_queue);
	establishlink(fi);
	test_and_set_bit(FLG_L3_INIT, &l2->flag);
	dev_kfree_skb(skb);
}

static void
l2_release(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	skb_trim(skb, 0);
	if (l2up(l2, DL_RELEASE | CONFIRM, CES(l2), skb))
		dev_kfree_skb(skb);
}

static void
l2_pend_rel(struct FsmInst *fi, int event, void *arg)
{
	struct sk_buff *skb = arg;
	layer2_t *l2 = fi->userdata;

	test_and_set_bit(FLG_PEND_REL, &l2->flag);
	dev_kfree_skb(skb);
}

static void
l2_disconnect(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	discard_queue(&l2->i_queue);
	freewin(l2);
	mISDN_FsmChangeState(fi, ST_L2_6);
	l2->rc = 0;
	send_uframe(l2, NULL, DISC | 0x10, CMD);
	mISDN_FsmDelTimer(&l2->t203, 1);
	restart_t200(l2, 2);
	if (skb)
		dev_kfree_skb(skb);
}

static void
l2_start_multi(struct FsmInst *fi, int event, void *arg)
{
	layer2_t	*l2 = fi->userdata;
	struct sk_buff	*skb = arg;

	send_uframe(l2, NULL, UA | get_PollFlag(l2, skb), RSP);

	clear_exception(l2);
	l2->vs = 0;
	l2->va = 0;
	l2->vr = 0;
	l2->sow = 0;
	mISDN_FsmChangeState(fi, ST_L2_7);
	mISDN_FsmAddTimer(&l2->t203, l2->T203, EV_L2_T203, NULL, 3);
	skb_trim(skb, 0);
	if (l2up(l2, DL_ESTABLISH | INDICATION, CES(l2), skb))
		dev_kfree_skb(skb);

#ifdef OLD_L2
	mISDN_queue_data(&l2->inst, l2->inst.id | MSG_BROADCAST,
		MGR_SHORTSTATUS | INDICATION, SSTATUS_L2_ESTABLISHED,
		0, NULL, 0);
#endif
}

static void
l2_send_UA(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	send_uframe(l2, skb, UA | get_PollFlag(l2, skb), RSP);
}

static void
l2_send_DM(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	send_uframe(l2, skb, DM | get_PollFlag(l2, skb), RSP);
}

static void
l2_restart_multi(struct FsmInst *fi, int event, void *arg)
{
	layer2_t	*l2 = fi->userdata;
	struct sk_buff	*skb = arg;
	int		est = 0;

	send_uframe(l2, skb, UA | get_PollFlag(l2, skb), RSP);

	l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'F');

	if (l2->vs != l2->va) {
		discard_queue(&l2->i_queue);
		est = 1;
	}

	clear_exception(l2);
	l2->vs = 0;
	l2->va = 0;
	l2->vr = 0;
	l2->sow = 0;
	mISDN_FsmChangeState(fi, ST_L2_7);
	stop_t200(l2, 3);
	mISDN_FsmRestartTimer(&l2->t203, l2->T203, EV_L2_T203, NULL, 3);

	if (est) {
		l2up_create(l2, DL_ESTABLISH | INDICATION, CES(l2), 0, NULL);
#ifdef OLD_L2
		mISDN_queue_data(&l2->inst, l2->inst.id | MSG_BROADCAST,
		    MGR_SHORTSTATUS | INDICATION, SSTATUS_L2_ESTABLISHED,
		    0, NULL, 0);
#endif
	}

	if (skb_queue_len(&l2->i_queue) && cansend(l2))
		mISDN_FsmEvent(fi, EV_L2_ACK_PULL, NULL);
}

static void
l2_stop_multi(struct FsmInst *fi, int event, void *arg)
{
	layer2_t	*l2 = fi->userdata;
	struct sk_buff	*skb = arg;

	mISDN_FsmChangeState(fi, ST_L2_4);
	mISDN_FsmDelTimer(&l2->t203, 3);
	stop_t200(l2, 4);

	send_uframe(l2, skb, UA | get_PollFlag(l2, skb), RSP);
	discard_queue(&l2->i_queue);
	freewin(l2);
	lapb_dl_release_l2l3(l2, INDICATION);

#ifdef OLD_L2
	mISDN_queue_data(&l2->inst, l2->inst.id | MSG_BROADCAST,
	    MGR_SHORTSTATUS | INDICATION, SSTATUS_L2_RELEASED,
	    0, NULL, 0);
#endif
}

static void
l2_connected(struct FsmInst *fi, int event, void *arg)
{
	layer2_t	*l2 = fi->userdata;
	struct sk_buff	*skb = arg;
	int pr = -1;

	if (!get_PollFlag(l2, skb)) {
		l2_mdl_error_ua(fi, event, arg);
		return;
	}
	dev_kfree_skb(skb);
	if (test_and_clear_bit(FLG_PEND_REL, &l2->flag))
		l2_disconnect(fi, event, NULL);
	if (test_and_clear_bit(FLG_L3_INIT, &l2->flag)) {
		pr = DL_ESTABLISH | CONFIRM;
	} else if (l2->vs != l2->va) {
		discard_queue(&l2->i_queue);
		pr = DL_ESTABLISH | INDICATION;
	}
	stop_t200(l2, 5);
	l2->vr = 0;
	l2->vs = 0;
	l2->va = 0;
	l2->sow = 0;
	mISDN_FsmChangeState(fi, ST_L2_7);
	mISDN_FsmAddTimer(&l2->t203, l2->T203, EV_L2_T203, NULL, 4);
	if (pr != -1)
		l2up_create(l2, pr, CES(l2), 0, NULL);

	if (skb_queue_len(&l2->i_queue) && cansend(l2))
		mISDN_FsmEvent(fi, EV_L2_ACK_PULL, NULL);

#ifdef OLD_L2
	mISDN_queue_data(&l2->inst, l2->inst.id | MSG_BROADCAST,
		MGR_SHORTSTATUS | INDICATION, SSTATUS_L2_ESTABLISHED,
		0, NULL, 0);
#endif
}

static void
l2_released(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	if (!get_PollFlag(l2, skb)) {
		l2_mdl_error_ua(fi, event, arg);
		return;
	}
	dev_kfree_skb(skb);
	stop_t200(l2, 6);
	lapb_dl_release_l2l3(l2, CONFIRM);
	mISDN_FsmChangeState(fi, ST_L2_4);

#ifdef OLD_L2
	mISDN_queue_data(&l2->inst, l2->inst.id | MSG_BROADCAST,
		MGR_SHORTSTATUS | INDICATION, SSTATUS_L2_RELEASED,
		0, NULL, 0);
#endif
}

static void
l2_reestablish(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	if (!get_PollFlagFree(l2, skb)) {
		establishlink(fi);
		test_and_set_bit(FLG_L3_INIT, &l2->flag);
	}
}

static void
l2_st5_dm_release(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	if (get_PollFlagFree(l2, skb)) {
		stop_t200(l2, 7);
		if (!test_bit(FLG_L3_INIT, &l2->flag))
			discard_queue(&l2->i_queue);
		if (test_bit(FLG_LAPB, &l2->flag))
			l2down_create(l2, PH_DEACTIVATE | REQUEST, 0, 0, NULL);
		st5_dl_release_l2l3(l2);
		mISDN_FsmChangeState(fi, ST_L2_4);
	}
}

static void
l2_st6_dm_release(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	if (get_PollFlagFree(l2, skb)) {
		stop_t200(l2, 8);
		lapb_dl_release_l2l3(l2, CONFIRM);
		mISDN_FsmChangeState(fi, ST_L2_4);
#ifdef OLD_L2
		mISDN_queue_data(&l2->inst, l2->inst.id | MSG_BROADCAST,
			MGR_SHORTSTATUS | INDICATION, SSTATUS_L2_RELEASED,
			0, NULL, 0);
#endif
	}
}

void
enquiry_cr(layer2_t *l2, u_char typ, u_char cr, u_char pf)
{
	struct sk_buff *skb;
	u_char tmp[MAX_HEADER_LEN];
	int i;

	i = sethdraddr(l2, tmp, cr);
	if (test_bit(FLG_MOD128, &l2->flag)) {
		tmp[i++] = typ;
		tmp[i++] = (l2->vr << 1) | (pf ? 1 : 0);
	} else
		tmp[i++] = (l2->vr << 5) | typ | (pf ? 0x10 : 0);
	if (!(skb = alloc_skb(i, GFP_ATOMIC))) {
		printk(KERN_WARNING
		    "isdnl2 can't alloc sbbuff for enquiry_cr\n");
		return;
	}
	memcpy(skb_put(skb, i), tmp, i);
	enqueue_super(l2, skb);
}

inline void
enquiry_response(layer2_t *l2)
{
	if (test_bit(FLG_OWN_BUSY, &l2->flag))
		enquiry_cr(l2, RNR, RSP, 1);
	else
		enquiry_cr(l2, RR, RSP, 1);
	test_and_clear_bit(FLG_ACK_PEND, &l2->flag);
}

inline void
transmit_enquiry(layer2_t *l2)
{
	if (test_bit(FLG_OWN_BUSY, &l2->flag))
		enquiry_cr(l2, RNR, CMD, 1);
	else
		enquiry_cr(l2, RR, CMD, 1);
	test_and_clear_bit(FLG_ACK_PEND, &l2->flag);
	start_t200(l2, 9);
}


static void
nrerrorrecovery(struct FsmInst *fi)
{
	layer2_t *l2 = fi->userdata;

	l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'J');
	establishlink(fi);
	test_and_clear_bit(FLG_L3_INIT, &l2->flag);
}

static void
invoke_retransmission(layer2_t *l2, unsigned int nr)
{
	u_int	p1;

	if (l2->vs != nr) {
		while (l2->vs != nr) {
			(l2->vs)--;
			if (test_bit(FLG_MOD128, &l2->flag)) {
				l2->vs %= 128;
				p1 = (l2->vs - l2->va) % 128;
			} else {
				l2->vs %= 8;
				p1 = (l2->vs - l2->va) % 8;
			}
			p1 = (p1 + l2->sow) % l2->window;
			if (l2->windowar[p1])
				skb_queue_head(&l2->i_queue, l2->windowar[p1]);
			l2->windowar[p1] = NULL;
		}
		mISDN_FsmEvent(&l2->l2m, EV_L2_ACK_PULL, NULL);
	}
}

static void
l2_st7_got_super(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;
	int PollFlag, rsp, typ = RR;
	unsigned int nr;

	rsp = *skb->data & 0x2;
	if (test_bit(FLG_ORIG, &l2->flag))
		rsp = !rsp;

	skb_pull(skb, l2addrsize(l2));
	if (IsRNR(skb->data, l2)) {
		set_peer_busy(l2);
		typ = RNR;
	} else
		clear_peer_busy(l2);
	if (IsREJ(skb->data, l2))
		typ = REJ;

	if (test_bit(FLG_MOD128, &l2->flag)) {
		PollFlag = (skb->data[1] & 0x1) == 0x1;
		nr = skb->data[1] >> 1;
	} else {
		PollFlag = (skb->data[0] & 0x10);
		nr = (skb->data[0] >> 5) & 0x7;
	}
	dev_kfree_skb(skb);

	if (PollFlag) {
		if (rsp)
			l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'A');
		else
			enquiry_response(l2);
	}
	if (legalnr(l2, nr)) {
		if (typ == REJ) {
			setva(l2, nr);
			invoke_retransmission(l2, nr);
			stop_t200(l2, 10);
			if (mISDN_FsmAddTimer(&l2->t203, l2->T203,
					EV_L2_T203, NULL, 6))
				l2m_debug(&l2->l2m, "Restart T203 ST7 REJ");
		} else if ((nr == l2->vs) && (typ == RR)) {
			setva(l2, nr);
			stop_t200(l2, 11);
			mISDN_FsmRestartTimer(&l2->t203, l2->T203,
					EV_L2_T203, NULL, 7);
		} else if ((l2->va != nr) || (typ == RNR)) {
			setva(l2, nr);
			if (typ != RR)
				mISDN_FsmDelTimer(&l2->t203, 9);
			restart_t200(l2, 12);
		}
		if (skb_queue_len(&l2->i_queue) && (typ == RR)) {
			mISDN_FsmEvent(fi, EV_L2_ACK_PULL, NULL);
		}
	} else
		nrerrorrecovery(fi);
}

static void
l2_feed_i_if_reest(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	if (!test_bit(FLG_L3_INIT, &l2->flag))
		skb_queue_tail(&l2->i_queue, skb);
	else
		dev_kfree_skb(skb);
}

static void
l2_feed_i_pull(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	skb_queue_tail(&l2->i_queue, skb);
	mISDN_FsmEvent(fi, EV_L2_ACK_PULL, NULL);
}

static void
l2_feed_iqueue(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	skb_queue_tail(&l2->i_queue, skb);
}

static void
l2_got_iframe(struct FsmInst *fi, int event, void *arg)
{
	layer2_t	*l2 = fi->userdata;
	struct sk_buff	*skb = arg;
	int		PollFlag, i;
	u_int		ns, nr;

	i = l2addrsize(l2);
	if (test_bit(FLG_MOD128, &l2->flag)) {
		PollFlag = ((skb->data[i + 1] & 0x1) == 0x1);
		ns = skb->data[i] >> 1;
		nr = (skb->data[i + 1] >> 1) & 0x7f;
	} else {
		PollFlag = (skb->data[i] & 0x10);
		ns = (skb->data[i] >> 1) & 0x7;
		nr = (skb->data[i] >> 5) & 0x7;
	}
	if (test_bit(FLG_OWN_BUSY, &l2->flag)) {
		dev_kfree_skb(skb);
		if (PollFlag)
			enquiry_response(l2);
	} else if (l2->vr == ns) {
		(l2->vr)++;
		if (test_bit(FLG_MOD128, &l2->flag))
			l2->vr %= 128;
		else
			l2->vr %= 8;
		test_and_clear_bit(FLG_REJEXC, &l2->flag);
		if (PollFlag)
			enquiry_response(l2);
		else
			test_and_set_bit(FLG_ACK_PEND, &l2->flag);
		skb_pull(skb, l2headersize(l2, 0));
		if (l2up(l2, DL_DATA | INDICATION, CES(l2), skb))
				dev_kfree_skb(skb);
	} else {
		/* n(s)!=v(r) */
		dev_kfree_skb(skb);
		if (test_and_set_bit(FLG_REJEXC, &l2->flag)) {
			if (PollFlag)
				enquiry_response(l2);
		} else {
			enquiry_cr(l2, REJ, RSP, PollFlag);
			test_and_clear_bit(FLG_ACK_PEND, &l2->flag);
		}
	}
	if (legalnr(l2, nr)) {
		if (!test_bit(FLG_PEER_BUSY, &l2->flag) &&
		    (fi->state == ST_L2_7)) {
			if (nr == l2->vs) {
				stop_t200(l2, 13);
				mISDN_FsmRestartTimer(&l2->t203, l2->T203,
						EV_L2_T203, NULL, 7);
			} else if (nr != l2->va)
				restart_t200(l2, 14);
		}
		setva(l2, nr);
	} else {
		nrerrorrecovery(fi);
		return;
	}
	if (skb_queue_len(&l2->i_queue) && (fi->state == ST_L2_7))
		mISDN_FsmEvent(fi, EV_L2_ACK_PULL, NULL);
	if (test_and_clear_bit(FLG_ACK_PEND, &l2->flag))
		enquiry_cr(l2, RR, RSP, 0);
}

static void
l2_got_tei(struct FsmInst *fi, int event, void *arg)
{
	layer2_t	*l2 = fi->userdata;
	struct sk_buff	*skb = arg;
	mISDN_head_t	*hh = mISDN_HEAD_P(skb);

	l2->tei = hh->dinfo;
	dev_kfree_skb(skb);
	if (fi->state == ST_L2_3) {
		establishlink(fi);
		test_and_set_bit(FLG_L3_INIT, &l2->flag);
	} else
		mISDN_FsmChangeState(fi, ST_L2_4);
	if (skb_queue_len(&l2->ui_queue))
		tx_ui(l2);
}

static void
l2_st5_tout_200(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;

	if (test_bit(FLG_LAPD, &l2->flag) &&
		test_bit(FLG_DCHAN_BUSY, &l2->flag)) {
		mISDN_FsmAddTimer(&l2->t200, l2->T200, EV_L2_T200, NULL, 9);
	} else if (l2->rc == l2->N200) {
		mISDN_FsmChangeState(fi, ST_L2_4);
		test_and_clear_bit(FLG_T200_RUN, &l2->flag);
		discard_queue(&l2->i_queue);
		l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'G');
		if (test_bit(FLG_LAPB, &l2->flag))
			l2down_create(l2, PH_DEACTIVATE | REQUEST, 0, 0, NULL);
		st5_dl_release_l2l3(l2);
	} else {
		l2->rc++;
		mISDN_FsmAddTimer(&l2->t200, l2->T200, EV_L2_T200, NULL, 9);
		send_uframe(l2, NULL, (test_bit(FLG_MOD128, &l2->flag) ?
			SABME : SABM) | 0x10, CMD);
	}
}

static void
l2_st6_tout_200(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;

	if (test_bit(FLG_LAPD, &l2->flag) &&
		test_bit(FLG_DCHAN_BUSY, &l2->flag)) {
		mISDN_FsmAddTimer(&l2->t200, l2->T200, EV_L2_T200, NULL, 9);
	} else if (l2->rc == l2->N200) {
		mISDN_FsmChangeState(fi, ST_L2_4);
		test_and_clear_bit(FLG_T200_RUN, &l2->flag);
		l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'H');
		lapb_dl_release_l2l3(l2, CONFIRM);
	} else {
		l2->rc++;
		mISDN_FsmAddTimer(&l2->t200, l2->T200, EV_L2_T200,
			    NULL, 9);
		send_uframe(l2, NULL, DISC | 0x10, CMD);
	}
}

static void
l2_st7_tout_200(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;

	if (test_bit(FLG_LAPD, &l2->flag) &&
		test_bit(FLG_DCHAN_BUSY, &l2->flag)) {
		mISDN_FsmAddTimer(&l2->t200, l2->T200, EV_L2_T200, NULL, 9);
		return;
	}
	test_and_clear_bit(FLG_T200_RUN, &l2->flag);
	l2->rc = 0;
	mISDN_FsmChangeState(fi, ST_L2_8);
	transmit_enquiry(l2);
	l2->rc++;
}

static void
l2_st8_tout_200(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;

	if (test_bit(FLG_LAPD, &l2->flag) &&
		test_bit(FLG_DCHAN_BUSY, &l2->flag)) {
		mISDN_FsmAddTimer(&l2->t200, l2->T200, EV_L2_T200, NULL, 9);
		return;
	}
	test_and_clear_bit(FLG_T200_RUN, &l2->flag);
	if (l2->rc == l2->N200) {
		l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'I');
		establishlink(fi);
		test_and_clear_bit(FLG_L3_INIT, &l2->flag);
	} else {
		transmit_enquiry(l2);
		l2->rc++;
	}
}

static void
l2_st7_tout_203(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;

	if (test_bit(FLG_LAPD, &l2->flag) &&
		test_bit(FLG_DCHAN_BUSY, &l2->flag)) {
		mISDN_FsmAddTimer(&l2->t203, l2->T203, EV_L2_T203, NULL, 9);
		return;
	}
	mISDN_FsmChangeState(fi, ST_L2_8);
	transmit_enquiry(l2);
	l2->rc = 0;
}

static void
l2_pull_iqueue(struct FsmInst *fi, int event, void *arg)
{
	layer2_t	*l2 = fi->userdata;
	struct sk_buff	*skb, *nskb, *oskb;
	u_char		header[MAX_HEADER_LEN];
	u_int		i, p1;

	if (!cansend(l2))
		return;

	skb = skb_dequeue(&l2->i_queue);
	if (!skb)
		return;

	if (test_bit(FLG_MOD128, &l2->flag))
		p1 = (l2->vs - l2->va) % 128;
	else
		p1 = (l2->vs - l2->va) % 8;
	p1 = (p1 + l2->sow) % l2->window;
	if (l2->windowar[p1]) {
		printk(KERN_WARNING "isdnl2 NT try overwrite ack queue entry %d\n",
		    p1);
		dev_kfree_skb(l2->windowar[p1]);
	}
	l2->windowar[p1] = skb;
	i = sethdraddr(l2, header, CMD);


	if (test_bit(FLG_MOD128, &l2->flag)) {
		header[i++] = l2->vs << 1;
		header[i++] = l2->vr << 1;
		l2->vs = (l2->vs + 1) % 128;
	} else {
		header[i++] = (l2->vr << 5) | (l2->vs << 1);
		l2->vs = (l2->vs + 1) % 8;
	}

	nskb = skb_clone(skb, GFP_ATOMIC);
	p1 = skb_headroom(nskb);
	if (p1 >= i)
		memcpy(skb_push(nskb, i), header, i);
	else {
		printk(KERN_WARNING
		    "isdnl2 pull_iqueue skb header(%d/%d) too short\n", i, p1);
		oskb = nskb;
		nskb = alloc_skb(oskb->len + i, GFP_ATOMIC);
		if (!nskb) {
			dev_kfree_skb(oskb);
			printk(KERN_WARNING "%s: no skb mem\n", __FUNCTION__);
			return;
		}
		memcpy(skb_put(nskb, i), header, i);
		memcpy(skb_put(nskb, oskb->len), oskb->data, oskb->len);
		dev_kfree_skb(oskb);
	}
	l2down(l2, PH_DATA_REQ, DINFO_SKB, nskb);
	test_and_clear_bit(FLG_ACK_PEND, &l2->flag);
	if (!test_and_set_bit(FLG_T200_RUN, &l2->flag)) {
		mISDN_FsmDelTimer(&l2->t203, 13);
		mISDN_FsmAddTimer(&l2->t200, l2->T200, EV_L2_T200, NULL, 11);
	}
}

static void
l2_st8_got_super(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;
	int PollFlag, rsp, rnr = 0;
	unsigned int nr;

	rsp = *skb->data & 0x2;
	if (test_bit(FLG_ORIG, &l2->flag))
		rsp = !rsp;

	skb_pull(skb, l2addrsize(l2));

	if (IsRNR(skb->data, l2)) {
		set_peer_busy(l2);
		rnr = 1;
	} else
		clear_peer_busy(l2);

	if (test_bit(FLG_MOD128, &l2->flag)) {
		PollFlag = (skb->data[1] & 0x1) == 0x1;
		nr = skb->data[1] >> 1;
	} else {
		PollFlag = (skb->data[0] & 0x10);
		nr = (skb->data[0] >> 5) & 0x7;
	}
	dev_kfree_skb(skb);
	if (rsp && PollFlag) {
		if (legalnr(l2, nr)) {
			if (rnr) {
				restart_t200(l2, 15);
			} else {
				stop_t200(l2, 16);
				mISDN_FsmAddTimer(&l2->t203, l2->T203,
					    EV_L2_T203, NULL, 5);
				setva(l2, nr);
			}
			invoke_retransmission(l2, nr);
			mISDN_FsmChangeState(fi, ST_L2_7);
			if (skb_queue_len(&l2->i_queue) && cansend(l2))
				mISDN_FsmEvent(fi, EV_L2_ACK_PULL, NULL);
		} else
			nrerrorrecovery(fi);
	} else {
		if (!rsp && PollFlag)
			enquiry_response(l2);
		if (legalnr(l2, nr)) {
			setva(l2, nr);
		} else
			nrerrorrecovery(fi);
	}
}

static void
l2_got_FRMR(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	skb_pull(skb, l2addrsize(l2) + 1);

	if (!(skb->data[0] & 1) || ((skb->data[0] & 3) == 1) || /* I or S */
	    (IsUA(skb->data) && (fi->state == ST_L2_7))) {
		l2mgr(l2, MDL_ERROR | INDICATION, (void *) 'K');
		establishlink(fi);
		test_and_clear_bit(FLG_L3_INIT, &l2->flag);
	}
	dev_kfree_skb(skb);
}

static void
l2_st24_tei_remove(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	discard_queue(&l2->ui_queue);
	l2->tei = -1;
	mISDN_FsmChangeState(fi, ST_L2_1);
	dev_kfree_skb(skb);
}

static void
l2_st3_tei_remove(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	discard_queue(&l2->ui_queue);
	l2->tei = -1;
	skb_trim(skb, 0);
	if (l2up(l2, DL_RELEASE | INDICATION, CES(l2), skb))
		dev_kfree_skb(skb);
	mISDN_FsmChangeState(fi, ST_L2_1);
}

static void
l2_st5_tei_remove(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	discard_queue(&l2->i_queue);
	discard_queue(&l2->ui_queue);
	freewin(l2);
	l2->tei = -1;
	stop_t200(l2, 17);
	st5_dl_release_l2l3(l2);
	mISDN_FsmChangeState(fi, ST_L2_1);
	dev_kfree_skb(skb);
}

static void
l2_st6_tei_remove(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	discard_queue(&l2->ui_queue);
	l2->tei = -1;
	stop_t200(l2, 18);
	if (l2up(l2, DL_RELEASE | CONFIRM, CES(l2), skb))
		dev_kfree_skb(skb);
	mISDN_FsmChangeState(fi, ST_L2_1);
}

static void
l2_tei_remove(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	discard_queue(&l2->i_queue);
	discard_queue(&l2->ui_queue);
	freewin(l2);
	l2->tei = -1;
	stop_t200(l2, 17);
	mISDN_FsmDelTimer(&l2->t203, 19);
	if (l2up(l2, DL_RELEASE | INDICATION, 0, skb))
		dev_kfree_skb(skb);
#ifdef OLD_L2
	mISDN_queue_data(&l2->inst, l2->inst.id | MSG_BROADCAST,
		MGR_SHORTSTATUS | INDICATION, SSTATUS_L2_RELEASED,
		0, NULL, 0);
#endif
	mISDN_FsmChangeState(fi, ST_L2_1);
}

static void
l2_st14_persistant_da(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	discard_queue(&l2->i_queue);
	discard_queue(&l2->ui_queue);
	if (test_and_clear_bit(FLG_ESTAB_PEND, &l2->flag))
		if (!l2up(l2, DL_RELEASE | INDICATION, CES(l2), skb))
			return;
	dev_kfree_skb(skb);
}

static void
l2_st5_persistant_da(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	discard_queue(&l2->i_queue);
	discard_queue(&l2->ui_queue);
	freewin(l2);
	stop_t200(l2, 19);
	st5_dl_release_l2l3(l2);
	mISDN_FsmChangeState(fi, ST_L2_4);
	dev_kfree_skb(skb);
}

static void
l2_st6_persistant_da(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	discard_queue(&l2->ui_queue);
	stop_t200(l2, 20);
	if (l2up(l2, DL_RELEASE | CONFIRM, CES(l2), skb))
		dev_kfree_skb(skb);
	mISDN_FsmChangeState(fi, ST_L2_4);
}

static void
l2_persistant_da(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	discard_queue(&l2->i_queue);
	discard_queue(&l2->ui_queue);
	freewin(l2);
	stop_t200(l2, 19);
	mISDN_FsmDelTimer(&l2->t203, 19);
	if (l2up(l2, DL_RELEASE | INDICATION, CES(l2), skb))
		dev_kfree_skb(skb);
#ifdef OLD_L2
	mISDN_queue_data(&l2->inst, l2->inst.id | MSG_BROADCAST,
		MGR_SHORTSTATUS | INDICATION, SSTATUS_L2_RELEASED,
		0, NULL, 0);
#endif
	mISDN_FsmChangeState(fi, ST_L2_4);
}

static void
l2_set_own_busy(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	if (!test_and_set_bit(FLG_OWN_BUSY, &l2->flag)) {
		enquiry_cr(l2, RNR, RSP, 0);
		test_and_clear_bit(FLG_ACK_PEND, &l2->flag);
	}
	if (skb)
		dev_kfree_skb(skb);
}

static void
l2_clear_own_busy(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;
	struct sk_buff *skb = arg;

	if (!test_and_clear_bit(FLG_OWN_BUSY, &l2->flag)) {
		enquiry_cr(l2, RR, RSP, 0);
		test_and_clear_bit(FLG_ACK_PEND, &l2->flag);
	}
	if (skb)
		dev_kfree_skb(skb);
}

static void
l2_frame_error(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;

	l2mgr(l2, MDL_ERROR | INDICATION, arg);
}

static void
l2_frame_error_reest(struct FsmInst *fi, int event, void *arg)
{
	layer2_t *l2 = fi->userdata;

	l2mgr(l2, MDL_ERROR | INDICATION, arg);
	establishlink(fi);
	test_and_clear_bit(FLG_L3_INIT, &l2->flag);
}

static struct FsmNode L2FnList[] =
{
	{ST_L2_1, EV_L2_DL_ESTABLISH_REQ, l2_mdl_assign},
	{ST_L2_2, EV_L2_DL_ESTABLISH_REQ, l2_go_st3},
	{ST_L2_4, EV_L2_DL_ESTABLISH_REQ, l2_establish},
	{ST_L2_5, EV_L2_DL_ESTABLISH_REQ, l2_discard_i_setl3},
	{ST_L2_7, EV_L2_DL_ESTABLISH_REQ, l2_l3_reestablish},
	{ST_L2_8, EV_L2_DL_ESTABLISH_REQ, l2_l3_reestablish},
	{ST_L2_4, EV_L2_DL_RELEASE_REQ, l2_release},
	{ST_L2_5, EV_L2_DL_RELEASE_REQ, l2_pend_rel},
	{ST_L2_7, EV_L2_DL_RELEASE_REQ, l2_disconnect},
	{ST_L2_8, EV_L2_DL_RELEASE_REQ, l2_disconnect},
	{ST_L2_5, EV_L2_DL_DATA, l2_feed_i_if_reest},
	{ST_L2_7, EV_L2_DL_DATA, l2_feed_i_pull},
	{ST_L2_8, EV_L2_DL_DATA, l2_feed_iqueue},
	{ST_L2_1, EV_L2_DL_UNITDATA, l2_queue_ui_assign},
	{ST_L2_2, EV_L2_DL_UNITDATA, l2_queue_ui},
	{ST_L2_3, EV_L2_DL_UNITDATA, l2_queue_ui},
	{ST_L2_4, EV_L2_DL_UNITDATA, l2_send_ui},
	{ST_L2_5, EV_L2_DL_UNITDATA, l2_send_ui},
	{ST_L2_6, EV_L2_DL_UNITDATA, l2_send_ui},
	{ST_L2_7, EV_L2_DL_UNITDATA, l2_send_ui},
	{ST_L2_8, EV_L2_DL_UNITDATA, l2_send_ui},
	{ST_L2_1, EV_L2_MDL_ASSIGN, l2_got_tei},
	{ST_L2_2, EV_L2_MDL_ASSIGN, l2_got_tei},
	{ST_L2_3, EV_L2_MDL_ASSIGN, l2_got_tei},
	{ST_L2_2, EV_L2_MDL_ERROR, l2_st24_tei_remove},
	{ST_L2_3, EV_L2_MDL_ERROR, l2_st3_tei_remove},
	{ST_L2_4, EV_L2_MDL_REMOVE, l2_st24_tei_remove},
	{ST_L2_5, EV_L2_MDL_REMOVE, l2_st5_tei_remove},
	{ST_L2_6, EV_L2_MDL_REMOVE, l2_st6_tei_remove},
	{ST_L2_7, EV_L2_MDL_REMOVE, l2_tei_remove},
	{ST_L2_8, EV_L2_MDL_REMOVE, l2_tei_remove},
	{ST_L2_4, EV_L2_SABME, l2_start_multi},
	{ST_L2_5, EV_L2_SABME, l2_send_UA},
	{ST_L2_6, EV_L2_SABME, l2_send_DM},
	{ST_L2_7, EV_L2_SABME, l2_restart_multi},
	{ST_L2_8, EV_L2_SABME, l2_restart_multi},
	{ST_L2_4, EV_L2_DISC, l2_send_DM},
	{ST_L2_5, EV_L2_DISC, l2_send_DM},
	{ST_L2_6, EV_L2_DISC, l2_send_UA},
	{ST_L2_7, EV_L2_DISC, l2_stop_multi},
	{ST_L2_8, EV_L2_DISC, l2_stop_multi},
	{ST_L2_4, EV_L2_UA, l2_mdl_error_ua},
	{ST_L2_5, EV_L2_UA, l2_connected},
	{ST_L2_6, EV_L2_UA, l2_released},
	{ST_L2_7, EV_L2_UA, l2_mdl_error_ua},
	{ST_L2_8, EV_L2_UA, l2_mdl_error_ua},
	{ST_L2_4, EV_L2_DM, l2_reestablish},
	{ST_L2_5, EV_L2_DM, l2_st5_dm_release},
	{ST_L2_6, EV_L2_DM, l2_st6_dm_release},
	{ST_L2_7, EV_L2_DM, l2_mdl_error_dm},
	{ST_L2_8, EV_L2_DM, l2_st8_mdl_error_dm},
	{ST_L2_1, EV_L2_UI, l2_got_ui},
	{ST_L2_2, EV_L2_UI, l2_got_ui},
	{ST_L2_3, EV_L2_UI, l2_got_ui},
	{ST_L2_4, EV_L2_UI, l2_got_ui},
	{ST_L2_5, EV_L2_UI, l2_got_ui},
	{ST_L2_6, EV_L2_UI, l2_got_ui},
	{ST_L2_7, EV_L2_UI, l2_got_ui},
	{ST_L2_8, EV_L2_UI, l2_got_ui},
	{ST_L2_7, EV_L2_FRMR, l2_got_FRMR},
	{ST_L2_8, EV_L2_FRMR, l2_got_FRMR},
	{ST_L2_7, EV_L2_SUPER, l2_st7_got_super},
	{ST_L2_8, EV_L2_SUPER, l2_st8_got_super},
	{ST_L2_7, EV_L2_I, l2_got_iframe},
	{ST_L2_8, EV_L2_I, l2_got_iframe},
	{ST_L2_5, EV_L2_T200, l2_st5_tout_200},
	{ST_L2_6, EV_L2_T200, l2_st6_tout_200},
	{ST_L2_7, EV_L2_T200, l2_st7_tout_200},
	{ST_L2_8, EV_L2_T200, l2_st8_tout_200},
	{ST_L2_7, EV_L2_T203, l2_st7_tout_203},
	{ST_L2_7, EV_L2_ACK_PULL, l2_pull_iqueue},
	{ST_L2_7, EV_L2_SET_OWN_BUSY, l2_set_own_busy},
	{ST_L2_8, EV_L2_SET_OWN_BUSY, l2_set_own_busy},
	{ST_L2_7, EV_L2_CLEAR_OWN_BUSY, l2_clear_own_busy},
	{ST_L2_8, EV_L2_CLEAR_OWN_BUSY, l2_clear_own_busy},
	{ST_L2_4, EV_L2_FRAME_ERROR, l2_frame_error},
	{ST_L2_5, EV_L2_FRAME_ERROR, l2_frame_error},
	{ST_L2_6, EV_L2_FRAME_ERROR, l2_frame_error},
	{ST_L2_7, EV_L2_FRAME_ERROR, l2_frame_error_reest},
	{ST_L2_8, EV_L2_FRAME_ERROR, l2_frame_error_reest},
	{ST_L2_1, EV_L1_DEACTIVATE, l2_st14_persistant_da},
	{ST_L2_2, EV_L1_DEACTIVATE, l2_st24_tei_remove},
	{ST_L2_3, EV_L1_DEACTIVATE, l2_st3_tei_remove},
	{ST_L2_4, EV_L1_DEACTIVATE, l2_st14_persistant_da},
	{ST_L2_5, EV_L1_DEACTIVATE, l2_st5_persistant_da},
	{ST_L2_6, EV_L1_DEACTIVATE, l2_st6_persistant_da},
	{ST_L2_7, EV_L1_DEACTIVATE, l2_persistant_da},
	{ST_L2_8, EV_L1_DEACTIVATE, l2_persistant_da},
};

#define L2_FN_COUNT (sizeof(L2FnList)/sizeof(struct FsmNode))

layer2_t *new_tei_req_net(mISDNinstance_t *inst)
{
	layer2_t    *l2, *l2t;
	int     tei;
	u_long		flags;
	int found;

	spin_lock_irqsave(&isdnl2.lock, flags);
	for (tei = 64; tei < 127; tei++) {
		found = 0;
		list_for_each_entry_safe(l2, l2t, &isdnl2.ilist, list) {
			if ((l2->tei == tei) && (inst == l2->instp)) {
				found = 1;
				break;
			}
		}
		if (!found)
			break;
	}
	spin_unlock_irqrestore(&isdnl2.lock, flags);

	if (tei == 127) /* all tei in use */
		return(NULL);
	l2 = new_dl2_net(inst, tei);
	return(l2);
}

layer2_t *
find_tei_net(mISDNinstance_t *inst, int tei)
{
	layer2_t	*l2, *l2t;
	u_long		flags;
	int found = 0;

	spin_lock_irqsave(&isdnl2.lock, flags);
	list_for_each_entry_safe(l2, l2t, &isdnl2.ilist, list) {
		if ((l2->tei == tei) && (inst == l2->instp)) {
			found = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&isdnl2.lock, flags);
	return((found) ? l2 : NULL);
}

static layer2_t *
select_l2(mISDNinstance_t *inst, int sapi, int tei) {
	layer2_t    *l2, *l2t;
	u_long		flags;
	int found = 0;

	if (debug)
		printk(KERN_DEBUG "select_l2: sapi=%x tei=%x\n", sapi, tei);

	spin_lock_irqsave(&isdnl2.lock, flags);
	list_for_each_entry_safe(l2, l2t, &isdnl2.ilist, list) {
		if ((l2->sapi == sapi) && (l2->tei == tei) &&
				(inst == l2->instp)) {
			found = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&isdnl2.lock, flags);
	return((found) ? l2 : NULL);
}

static int
ph_data_mux_net(mISDNinstance_t *inst, mISDN_head_t *hh, struct sk_buff *skb)
{
	layer2_t	*l2 = inst->privat;
	u_char		*datap = skb->data;
	int		ret = -EINVAL;
	int		psapi, ptei;
	int		c = 0;

	if (l2->debug)
		l2m_debug(&l2->l2m, "ph_data_mux_net: len %d", skb->len);

	if (skb->len <= 2) {
		mISDN_FsmEvent(&l2->l2m, EV_L2_FRAME_ERROR, (void *) 'N');
		return(ret);
	}
	psapi = *datap++;
	ptei = *datap++;
	if ((psapi & 1) || !(ptei & 1)) {
		printk(KERN_WARNING "l2 NT D-channel frame wrong EA0/EA1\n");
		return(ret);
	}
	psapi >>= 2;
	ptei >>= 1;
	if (ptei == GROUP_TEI) {
		if (psapi == TEI_SAPI) {
			hh->prim = MDL_UNITDATA | INDICATION;
			if (inst->pid.protocol[2] & ISDN_PID_L2_DF_PTP) {
				printk(KERN_WARNING "%s: tei (%d) management not enabled for PTP\n", 
					__FUNCTION__, ptei);
				return(-EINVAL);
			}
			return(tei_mux_net(inst, skb));
		} else {
			printk(KERN_WARNING "%s NT: unknown tei(%d) msg\n", __FUNCTION__,
				ptei);
		}
	}
	l2 = select_l2(inst, psapi, ptei);
	if (!l2) {
		if (debug)
			printk(KERN_DEBUG "%s NT: no l2 for sapi(%d) tei(%d)\n", __FUNCTION__,
				psapi, ptei);
		return(-ENXIO);
	}
	if (!(*datap & 1)) {	/* I-Frame */
		if(!(c = iframe_error(l2, skb)))
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_I, skb);
	} else if (IsSFrame(datap, l2)) {	/* S-Frame */
		if(!(c = super_error(l2, skb)))
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_SUPER, skb);
	} else if (IsUI(datap)) {
		if(!(c = UI_error(l2, skb)))
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_UI, skb);
	} else if (IsSABME(datap, l2)) {
		if(!(c = unnum_error(l2, skb, CMD)))
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_SABME, skb);
	} else if (IsUA(datap)) {
		if(!(c = unnum_error(l2, skb, RSP)))
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_UA, skb);
	} else if (IsDISC(datap)) {
		if(!(c = unnum_error(l2, skb, CMD)))
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_DISC, skb);
	} else if (IsDM(datap)) {
		if(!(c = unnum_error(l2, skb, RSP)))
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_DM, skb);
	} else if (IsFRMR(datap)) {
		if(!(c = FRMR_error(l2, skb)))
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_FRMR, skb);
	} else {
		c = 'L';
	}
	if (c) {
		printk(KERN_DEBUG "l2 NT D-channel frame error %c\n",c);
		mISDN_FsmEvent(&l2->l2m, EV_L2_FRAME_ERROR, (void *)(long)c);
	}
	if (ret)
		dev_kfree_skb(skb);
	return(0);
}


static int
msg_mux(mISDNinstance_t *inst, mISDN_head_t *hh, struct sk_buff *skb)
{
	layer2_t    *l2, *l2t;
	int		ret = -EINVAL;
	struct sk_buff *nskb;

	list_for_each_entry_safe(l2, l2t, &isdnl2.ilist, list) {
		if (l2->instp != inst)
			continue;

		if (debug & 0x02)
			printk(KERN_DEBUG "msg_mux NT: %s (tei %d)\n",
				l2->instp->name, l2->tei);

		if (hh->prim == (PH_CONTROL | INDICATION)) {
			if (hh->dinfo == HW_D_BLOCKED)
				test_and_set_bit(FLG_DCHAN_BUSY, &l2->flag);
			else if (hh->dinfo == HW_D_NOBLOCKED)
				test_and_clear_bit(FLG_DCHAN_BUSY, &l2->flag);
			continue;
		}
		nskb = skb_copy(skb, GFP_ATOMIC);
		ret = -EINVAL;
		switch (hh->prim) {
			case (PH_ACTIVATE | CONFIRM):
			case (PH_ACTIVATE | INDICATION):
				test_and_set_bit(FLG_L1_ACTIV, &l2->flag);
				if (test_and_clear_bit(FLG_ESTAB_PEND, &l2->flag))
					ret = mISDN_FsmEvent(&l2->l2m,
						EV_L2_DL_ESTABLISH_REQ, nskb);
				break;
			case (PH_DEACTIVATE | INDICATION):
			case (PH_DEACTIVATE | CONFIRM):
				test_and_clear_bit(FLG_L1_ACTIV, &l2->flag);
				ret = mISDN_FsmEvent(&l2->l2m, EV_L1_DEACTIVATE, nskb);
				break;
			default:
				l2m_debug(&l2->l2m, "l2 unknown pr %x", hh->prim);
				break;
		}
		if (ret)
			dev_kfree_skb(nskb);
		ret = 0;
	}

	dev_kfree_skb(skb);
	return(0);
}

static int
ph_data_confirm(mISDNinstance_t *inst, mISDN_head_t *hh, struct sk_buff *skb) {
	layer2_t *l2 = inst->privat;
	struct sk_buff *nskb = skb;
//	mISDNif_t *next = up->clone;
	int ret = -EAGAIN;

	if (test_bit(FLG_L1_BUSY, &l2->flag)) {
		if (hh->dinfo == l2->down_id) {
			if ((nskb = skb_dequeue(&l2->down_queue))) {
				l2->down_id = mISDN_HEAD_DINFO(nskb);
				if (l2down_skb(l2, nskb)) {
					dev_kfree_skb(nskb);
					l2->down_id = MISDN_ID_NONE;
				}
			} else
				l2->down_id = MISDN_ID_NONE;
#ifdef FIXME
			if (next)
				ret = next->func(next, skb);
#endif
			if (ret) {
				dev_kfree_skb(skb);
				ret = 0;
			}
			if (l2->down_id == MISDN_ID_NONE) {
				test_and_clear_bit(FLG_L1_BUSY, &l2->flag);
				mISDN_FsmEvent(&l2->l2m, EV_L2_ACK_PULL, NULL);
			}
		}
	}
#ifdef FIXME
	if (ret && next)
		ret = next->func(next, skb);
#endif
	if (!test_and_set_bit(FLG_L1_BUSY, &l2->flag)) {
		if ((nskb = skb_dequeue(&l2->down_queue))) {
			l2->down_id = mISDN_HEAD_DINFO(nskb);
			if (l2down_skb(l2, nskb)) {
				dev_kfree_skb(nskb);
				l2->down_id = MISDN_ID_NONE;
				test_and_clear_bit(FLG_L1_BUSY, &l2->flag);
			}
		} else
			test_and_clear_bit(FLG_L1_BUSY, &l2->flag);
	}
	return ret;
}

static int
l2from_down(mISDNinstance_t *inst, struct sk_buff *skb, mISDN_head_t *hh)
{
	layer2_t	*l2 = inst->privat;
	int 		ret = -EINVAL;

	if (l2->debug)
		l2m_debug(&l2->l2m, "l2from_down NT prim %x", hh->prim);

	switch (hh->prim) {
		case (PH_DATA_IND):
			ret = ph_data_mux_net(inst, hh, skb);
			break;
		case (PH_DATA | CONFIRM):
			ret = ph_data_confirm(inst, hh, skb);
			break;
		case (PH_ACTIVATE | CONFIRM):
		case (PH_ACTIVATE | INDICATION):
		case (PH_CONTROL | INDICATION):
		case (PH_DEACTIVATE | INDICATION):
		case (PH_DEACTIVATE | CONFIRM):
			ret = msg_mux(inst, hh, skb);
			break;
		default:
			if (l2->debug)
				l2m_debug(&l2->l2m, "l2 unknown pr %x",
				    hh->prim);
			ret = -EINVAL;
			break;
	}
	return ret;
}

static int
l2from_up_net(mISDNinstance_t *inst, struct sk_buff *skb, mISDN_head_t *hh) {
	layer2_t    *l2 = inst->privat;
	int		ret = -EINVAL;

	if (hh->addr & FLG_MSG_CLONED)
		return l2down_raw(l2, skb);

	l2 = select_l2(inst, SAPITEI(hh->dinfo));
	if (!l2) {
		printk(KERN_WARNING "%s: no l2 for sapi(%d) tei(%d)\n", __FUNCTION__,
			SAPITEI(hh->dinfo));
		return -ENXIO;
	}

	switch (hh->prim) {
		case (DL_DATA | REQUEST):
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_DL_DATA, skb);
			break;
		case (DL_UNITDATA | REQUEST):
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_DL_UNITDATA, skb);
			break;
		case (DL_ESTABLISH | REQUEST):
			if (test_bit(FLG_L1_ACTIV, &l2->flag)) {
				if (test_bit(FLG_LAPD, &l2->flag) ||
					test_bit(FLG_ORIG, &l2->flag)) {
					ret = mISDN_FsmEvent(&l2->l2m,
						EV_L2_DL_ESTABLISH_REQ, skb);
				}
			} else {
				if (test_bit(FLG_LAPD, &l2->flag) ||
					test_bit(FLG_ORIG, &l2->flag)) {
					test_and_set_bit(FLG_ESTAB_PEND,
						&l2->flag);
				}
				ret = l2down(l2, PH_ACTIVATE | REQUEST, 0, skb);
			}
			break;
		case (DL_RELEASE | REQUEST):
			if (test_bit(FLG_LAPB, &l2->flag))
				l2down_create(l2, PH_DEACTIVATE | REQUEST,
					0, 0, NULL);
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_DL_RELEASE_REQ,
			    skb);
			break;
		case (MDL_ASSIGN | REQUEST):
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_MDL_ASSIGN, skb);
			break;
		case (MDL_REMOVE | REQUEST):
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_MDL_REMOVE, skb);
			break;
		case (MDL_ERROR | RESPONSE):
			ret = mISDN_FsmEvent(&l2->l2m, EV_L2_MDL_ERROR, skb);
		case (MDL_STATUS | REQUEST):
			l2up_create(l2, MDL_STATUS | CONFIRM, hh->dinfo, 1,
				(void *)((u_long)l2->tei));
			break;
		default:
			if (l2->debug)
				l2m_debug(&l2->l2m, "l2 unknown pr %04x",
				    hh->prim);
	}
	return ret;
}

static int
l2_shortstatus(layer2_t *l2, struct sk_buff *skb, mISDN_head_t *hh)
{
	u_int	temp;

	if (hh->prim == (MGR_SHORTSTATUS | REQUEST)) {
		temp = hh->dinfo & SSTATUS_ALL;
		if (temp == SSTATUS_ALL || temp == SSTATUS_L2) {
			skb_trim(skb, 0);
			if (hh->dinfo & SSTATUS_BROADCAST_BIT)
				temp = l2->inst.id | MSG_BROADCAST;
			else
				temp = hh->addr | FLG_MSG_TARGET;
			switch (l2->l2m.state) {
			case ST_L2_7:
			case ST_L2_8:
				hh->dinfo = SSTATUS_L2_ESTABLISHED;
				break;
			default:
				hh->dinfo = SSTATUS_L2_RELEASED;
			}
			hh->prim = MGR_SHORTSTATUS | CONFIRM;
			return mISDN_queue_message(&l2->inst, temp, skb);
		}
	}
	return -EOPNOTSUPP;
}

static int
l2_function_net(mISDNinstance_t *inst, struct sk_buff *skb)
{
	layer2_t	*l2 = inst->privat;
	mISDN_head_t	*hh = mISDN_HEAD_P(skb);
	int		ret = -EINVAL;

	if (debug)
		printk(KERN_DEBUG  "%s: addr(%08x) prim(%x) dinfo(%x)\n",
			__FUNCTION__,  hh->addr, hh->prim, hh->dinfo);
	if (!l2)
		return ret;

	if (unlikely((hh->prim & MISDN_CMD_MASK) == MGR_SHORTSTATUS))
		return l2_shortstatus(l2, skb, hh);

	switch (hh->addr & MSG_DIR_MASK) {
	case FLG_MSG_DOWN:
		ret = l2from_up_net(inst, skb, hh);
		break;
	case FLG_MSG_UP:
		ret = l2from_down(inst, skb, hh);
		break;
	case MSG_TO_OWNER:
		/* FIXME: must be handled depending on type */
		int_errtxt("not implemented yet");
		break;
	default:
		/* FIXME: broadcast must be handled depending on type */
		int_errtxt("not implemented yet");
		break;
	}
	return ret;
}

int
tei_l2_net(layer2_t *l2, struct sk_buff *skb)
{
	mISDN_head_t	*hh;
	int		ret = -EINVAL;

	if (!l2 || !skb)
		return ret;
	hh = mISDN_HEAD_P(skb);
	if (l2->debug)
		printk(KERN_DEBUG "%s: prim(%x)\n", __FUNCTION__, hh->prim);
	switch (hh->prim) {
	case (MDL_UNITDATA | REQUEST):
		ret = l2down(l2, PH_DATA_REQ, hh->dinfo, skb);
		break;
	case (MDL_ASSIGN | REQUEST):
		ret = mISDN_FsmEvent(&l2->l2m, EV_L2_MDL_ASSIGN, skb);
		break;
	case (MDL_REMOVE | REQUEST):
		ret = mISDN_FsmEvent(&l2->l2m, EV_L2_MDL_REMOVE, skb);
		break;
	case (MDL_ERROR | RESPONSE):
		ret = mISDN_FsmEvent(&l2->l2m, EV_L2_MDL_ERROR, skb);
		break;
	case (MDL_FINDTEI | REQUEST):
		ret = l2down_skb(l2, skb);
		break;
	}
	return ret;
}

static void
l2m_debug(struct FsmInst *fi, char *fmt, ...)
{
	layer2_t *l2 = fi->userdata;
	logdata_t log;

	va_start(log.args, fmt);
	log.fmt = fmt;
	log.head = l2->instp->name;
	mISDN_ctrl(&l2->inst, MGR_DEBUGDATA | REQUEST, &log);
	va_end(log.args);
}

static void
release_l2(layer2_t *l2)
{
	mISDNinstance_t	*inst = l2->instp;
	u_long		flags;

	if (debug & 0x10)
		printk(KERN_WARNING "release l2 net: tei %d\n", l2->tei);

	mISDN_FsmDelTimer(&l2->t200, 21);
	mISDN_FsmDelTimer(&l2->t203, 16);
	discard_queue(&l2->i_queue);
	discard_queue(&l2->ui_queue);
	discard_queue(&l2->down_queue);
	ReleaseWin(l2);
	if (test_bit(FLG_LAPD, &l2->flag))
		release_tei_net(l2->tm);
	spin_lock_irqsave(&isdnl2.lock, flags);
	list_del(&l2->list);
	spin_unlock_irqrestore(&isdnl2.lock, flags);
	mISDN_ctrl(inst, MGR_UNREGLAYER | REQUEST, NULL);
	if (l2->entity != MISDN_ENTITY_NONE)
		mISDN_ctrl(inst, MGR_DELENTITY | REQUEST,
		    (void *)((u_long)l2->entity));
	kfree(l2);
}

static layer2_t *new_dl2_net(mISDNinstance_t *ninst, int tei)
{
	layer2_t	*nl2;
	u_long		flags;

	if (!(nl2 = kzalloc(sizeof(layer2_t), GFP_ATOMIC))) {
		printk(KERN_ERR "kmalloc layer2 NT failed\n");
		return NULL;
	}
	nl2->debug = debug;
	if (ninst == NULL) {
		nl2->instp = &nl2->inst;
	} else {
		nl2->instp = ninst;
	}
	nl2->next_id = 1;
	nl2->down_id = MISDN_ID_NONE;

	test_and_set_bit(FLG_LAPD, &nl2->flag);
	test_and_set_bit(FLG_LAPD_NET, &nl2->flag);
	test_and_set_bit(FLG_MOD128, &nl2->flag);
	nl2->sapi = 0;
	nl2->tei = tei;
	nl2->maxlen = MAX_DFRAME_LEN;
	nl2->window = 1;
	nl2->T200 = 1000;
	nl2->N200 = 3;
	nl2->T203 = 10000;
	if (create_teimgr_net(nl2)) {
		kfree(nl2);
		return NULL;
	}
	skb_queue_head_init(&nl2->i_queue);
	skb_queue_head_init(&nl2->ui_queue);
	skb_queue_head_init(&nl2->down_queue);
	skb_queue_head_init(&nl2->tmp_queue);
	InitWin(nl2);
	nl2->l2m.fsm = &l2fsm;
	nl2->l2m.state = ST_L2_4;
	nl2->l2m.debug = debug;
	nl2->l2m.userdata = nl2;
	nl2->l2m.userint = 0;
	nl2->l2m.printdebug = l2m_debug;
	mISDN_FsmInitTimer(&nl2->l2m, &nl2->t200);
	mISDN_FsmInitTimer(&nl2->l2m, &nl2->t203);
	spin_lock_irqsave(&isdnl2.lock, flags);
	list_add_tail(&nl2->list, &isdnl2.ilist);
	spin_unlock_irqrestore(&isdnl2.lock, flags);
	if (debug & 0x10)
		printk(KERN_DEBUG "new_dl2_net: created l2 tei %d\n", tei);
	return(nl2);
}

static int
new_l2_net(mISDNstack_t *st, mISDN_pid_t *pid) {
	struct sk_buff	*skb;
	layer2_t	*nl2, *nl2a;
	int		err;

	if (!st || !pid)
		return -EINVAL;

	if ((nl2 = new_dl2_net(NULL, 0)) == NULL) {
		printk(KERN_WARNING "mISDN L2 NT: failed to create L2 with TEI 0\n");
		return -ENOMEM;
	}

	if ((nl2a = new_dl2_net(&nl2->inst, 127)) == NULL) {
		printk(KERN_WARNING "mISDN L2 NT: failed to create L2 with TEI 127\n");
		return -ENOMEM;
	}	
	if (pid->protocol[2] & ISDN_PID_L2_DF_PTP) {
		if (debug & 0x1)
			printk(KERN_WARNING "new_l2_net: set to PTP\n");
		test_and_set_bit(FLG_PTP, &nl2->flag);
	}

	mISDN_init_instance(&nl2->inst, &isdnl2, nl2, l2_function_net);
	nl2->inst.extentions = EXT_INST_CLONE;
	memcpy(&nl2->inst.pid, pid, sizeof(mISDN_pid_t));
	if (!mISDN_SetHandledPID(&isdnl2, &nl2->inst.pid)) {
		int_error();
		return -ENOPROTOOPT;
	}
	sprintf(nl2->inst.name, "lapdn %x", st->id>>8);

	err = mISDN_ctrl(&nl2->inst, MGR_NEWENTITY | REQUEST, NULL);
	if (err) {
		printk(KERN_WARNING
		    "mISDN %s: MGR_NEWENTITY REQUEST failed err(%d)\n",
		    __FUNCTION__, err);
	}
	err = mISDN_ctrl(st, MGR_REGLAYER | INDICATION, &nl2->inst);
	if (err) {
		mISDN_FsmDelTimer(&nl2->t200, 0);
		mISDN_FsmDelTimer(&nl2->t203, 0);
		list_del(&nl2->list);
		kfree(nl2);
		nl2 = NULL;
	} else {
		mISDN_stPara_t	stp;

		if (st->para.maxdatalen)
			nl2->maxlen = st->para.maxdatalen;
		stp.maxdatalen = 0;
		stp.up_headerlen = 0;
		stp.down_headerlen = l2headersize(nl2, 0);
		mISDN_ctrl(st, MGR_ADDSTPARA | REQUEST, &stp);
	}
	if (!(pid->protocol[2] & ISDN_PID_L2_DF_PTP)) {
		if ((skb = create_link_skb(MDL_REMOVE | INDICATION, 127, 0, NULL, 0))) {
			if (l2_tei_net(nl2->tm, skb))
				dev_kfree_skb(skb);
		}
	}
	if (debug & 0x1)
		printk(KERN_WARNING "new_l2_net: %s\n", nl2->inst.name);

	return err;
}

static int
l2_status(layer2_t *l2, status_info_l2_t *si)
{
	if (!si)
		return -EINVAL;
	memset(si, 0, sizeof(status_info_l2_t));
	si->len = sizeof(status_info_l2_t) - 2*sizeof(int);
	si->typ = STATUS_INFO_L2;
	si->protocol = l2->instp->pid.protocol[2];
	si->state = l2->l2m.state;
	si->sapi = l2->sapi;
	si->tei = l2->tei;
	si->addr = l2->addr;
	si->maxlen = l2->maxlen;
	si->flag = l2->flag;
	si->vs = l2->vs;
	si->va = l2->va;
	si->vr = l2->vr;
	si->rc = l2->rc;
	si->window = l2->window;
	si->sow = l2->sow;
	si->T200 = l2->T200;
	si->N200 = l2->N200;
	si->T203 = l2->T203;
	si->len_i_queue = skb_queue_len(&l2->i_queue);
	si->len_ui_queue = skb_queue_len(&l2->ui_queue);
	si->len_d_queue = skb_queue_len(&l2->down_queue);
	si->debug = l2->debug;
	if (l2->tm) {
		si->tei_state = l2->tm->tei_m.state;
		si->tei_ri = l2->tm->ri;
		si->T201 = l2->tm->T201;
		si->tei_debug = l2->tm->debug;
	}
	return 0;
}

static char MName[] = "ISDNL2NT";

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
l2_manager_net(void *data, u_int prim, void *arg) {
	mISDNinstance_t	*inst = data;
	layer2_t	*l2l, *l2t;
	int		err = -EINVAL;
	u_long		flags;

	if (debug & 0x1000)
		printk(KERN_DEBUG "%s: data:%p prim:%x arg:%p\n", __FUNCTION__,
			data, prim, arg);
	if (!data)
		return err;
	spin_lock_irqsave(&isdnl2.lock, flags);
	list_for_each_entry_safe(l2l, l2t, &isdnl2.ilist, list) {
		if (&l2l->inst == inst) {
			err = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&isdnl2.lock, flags);
	if (prim == (MGR_NEWLAYER | REQUEST))
		return new_l2_net(data, arg);
	if (err) {
		if (debug & 0x1)
			printk(KERN_WARNING
			    "l2_manager_net prim(%x) l2 no instance\n", prim);
		return err;
	}
	switch (prim) {
	case MGR_NEWENTITY | CONFIRM:
		l2l->entity = (u_long)arg & 0xffffffff;
		break;
	case MGR_ADDSTPARA | INDICATION:
		if (((mISDN_stPara_t *)arg)->maxdatalen)
			l2l->maxlen = ((mISDN_stPara_t *)arg)->maxdatalen;
	case MGR_CLRSTPARA | INDICATION:
		break;
	case MGR_RELEASE | INDICATION:
	case MGR_UNREGLAYER | REQUEST:
		release_l2(l2l);
		break;
	case MGR_STATUS | REQUEST:
		return l2_status(l2l, arg);
	default:
		if (debug & 0x1)
			printk(KERN_WARNING
			    "l2_manager_net prim %x not handled\n", prim);
		return -EINVAL;
	}
	return 0;
}

int
Isdnl2_net_Init(void)
{
	int err;

	printk(KERN_INFO
	    "ISDN L2 NT driver version %s (Melware)\n", mISDN_getrev(l2_revision));
#ifdef MODULE
	isdnl2.owner = THIS_MODULE;
#endif
	isdnl2.name = MName;
	isdnl2.DPROTO.protocol[2] = ISDN_PID_L2_LAPD_NET | ISDN_PID_L2_DF_PTP;
	isdnl2.BPROTO.protocol[2] = ISDN_PID_L2_B_X75SLP;
	isdnl2.own_ctrl = l2_manager_net;
	spin_lock_init(&isdnl2.lock);
	INIT_LIST_HEAD(&isdnl2.ilist);
	l2fsm.state_count = L2_STATE_COUNT;
	l2fsm.event_count = L2_EVENT_COUNT;
	l2fsm.strEvent = strL2Event;
	l2fsm.strState = strL2State;
	mISDN_FsmNew(&l2fsm, L2FnList, L2_FN_COUNT);
	TEIInit_net();
	if ((err = mISDN_register(&isdnl2))) {
		printk(KERN_ERR "Can't register %s error(%d)\n", MName, err);
		mISDN_FsmFree(&l2fsm);
	} else
		mISDN_module_register(THIS_MODULE);
	return err;
}

void
Isdnl2_net_cleanup(void)
{
	int		err;
	layer2_t	*l2, *nl2;

	mISDN_module_unregister(THIS_MODULE);

	if ((err = mISDN_unregister(&isdnl2))) {
		printk(KERN_ERR
		    "Can't unregister ISDN layer 2 NT error(%d)\n", err);
	}
	if (!list_empty(&isdnl2.ilist)) {
		printk(KERN_WARNING "mISDNl2 NT l2 list not empty\n");
		list_for_each_entry_safe(l2, nl2, &isdnl2.ilist, list)
			release_l2(l2);
	}
	TEIFree_net();
	mISDN_FsmFree(&l2fsm);
}

module_init(Isdnl2_net_Init);
module_exit(Isdnl2_net_cleanup);

