/* $Id: tei_net.c,v 1.3 2008-11-10 22:57:47 armin Exp $
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 * Adapted for NT mode    Armin Schindler (armin@melware.de)
 *
 *		This file is (c) under GNU PUBLIC LICENSE
 *		For changes and modifications please read
 *		../../../Documentation/isdn/mISDN.cert
 *
 */
#include "layer2_net.h"
#include "helper.h"
#include "debug.h"
#include <linux/random.h>

const char *tei_revision = "$Revision: 1.3 $";

#define ID_REQUEST	1
#define ID_ASSIGNED	2
#define ID_DENIED	3
#define ID_CHK_REQ	4
#define ID_CHK_RES	5
#define ID_REMOVE	6
#define ID_VERIFY	7

#define TEI_ENTITY_ID	0xf

static
struct Fsm teifsm =
{NULL, 0, 0, NULL, NULL};

enum {
	ST_TEI_NOP,
	ST_TEI_REMOVE,
	ST_TEI_IDVERIFY,
};

#define TEI_STATE_COUNT (ST_TEI_IDVERIFY+1)

static char *strTeiState[] =
{
	"ST_TEI_NOP",
	"ST_TEI_REMOVE",
	"ST_TEI_IDVERIFY",
};

enum {
	EV_IDREQ,
	EV_ASSIGN,
	EV_ASSIGN_REQ,
	EV_CHECK_RES,
	EV_CHECK_REQ,
	EV_REMOVE,
	EV_VERIFY,
	EV_T201,
};

#define TEI_EVENT_COUNT (EV_T201+1)

static char *strTeiEvent[] =
{
	"EV_IDREQ",
	"EV_ASSIGN",
	"EV_ASSIGN_REQ",
	"EV_CHECK_RES",
	"EV_CHECK_REQ",
	"EV_REMOVE",
	"EV_VERIFY",
	"EV_T201",
};

static void
put_tei_msg(teimgr_t *tm, u_char m_id, unsigned int ri, u_char tei)
{
	struct sk_buff *skb;
	u_char bp[8];

	bp[0] = (TEI_SAPI << 2);
	if (test_bit(FLG_LAPD_NET, &tm->l2->flag))
		bp[0] |= 2; /* CR:=1 for net command */
	bp[1] = (GROUP_TEI << 1) | 0x1;
	bp[2] = UI;
	bp[3] = TEI_ENTITY_ID;
	bp[4] = ri >> 8;
	bp[5] = ri & 0xff;
	bp[6] = m_id;
	bp[7] = (tei << 1) | 1;
	skb = create_link_skb(MDL_UNITDATA | REQUEST, DINFO_SKB, 8, bp, 0);
	if (!skb) {
		printk(KERN_WARNING "mISDN: No skb for TEI NT manager\n");
		return;
	}
	if (tei_l2_net(tm->l2, skb))
		dev_kfree_skb(skb);
}

static void
tei_assign_req(struct FsmInst *fi, int event, void *arg)
{
	teimgr_t *tm = fi->userdata;
	u_char *dp = arg;

	if (tm->l2->tei == -1) {
		tm->tei_m.printdebug(&tm->tei_m,
			"net tei assign request without tei");
		return;
	}
	tm->ri = ((unsigned int) *dp++ << 8);
	tm->ri += *dp++;
	if (tm->debug)
		tm->tei_m.printdebug(&tm->tei_m,
			"net assign request ri %d teim %d", tm->ri, *dp);
	put_tei_msg(tm, ID_ASSIGNED, tm->ri, tm->l2->tei);
	mISDN_FsmChangeState(fi, ST_TEI_NOP);
}

static void
tei_id_chk_res(struct FsmInst *fi, int event, void *arg)
{
	teimgr_t *tm = fi->userdata;
	int *ri = arg;

	if (tm->debug)
		tm->tei_m.printdebug(fi, "identity %d check response ri %x/%x",
			tm->l2->tei, *ri, tm->ri);
	if (tm->ri != -1) {
		mISDN_FsmDelTimer(&tm->t201, 4);
		tm->tei_m.printdebug(fi, "duplicat %d response", tm->l2->tei);
		tm->val = tm->l2->tei;
		put_tei_msg(tm, ID_REMOVE, 0, tm->val);
		mISDN_FsmAddTimer(&tm->t201, tm->T201, EV_T201, NULL, 2);
		mISDN_FsmChangeState(&tm->tei_m, ST_TEI_REMOVE);
	} else
		tm->ri = *ri;
}

static void
tei_id_remove(struct FsmInst *fi, int event, void *arg)
{
	teimgr_t *tm = fi->userdata;
	int *tei = arg;

	if (tm->debug)
		tm->tei_m.printdebug(fi, "identity remove tei %d/%d", *tei, tm->l2->tei);
	tm->val = *tei;
	put_tei_msg(tm, ID_REMOVE, 0, tm->val);
	mISDN_FsmAddTimer(&tm->t201, tm->T201, EV_T201, NULL, 2);
	mISDN_FsmChangeState(&tm->tei_m, ST_TEI_REMOVE);
}

static void
tei_id_verify(struct FsmInst *fi, int event, void *arg)
{
	teimgr_t *tm = fi->userdata;

	if (tm->debug)
		tm->tei_m.printdebug(fi, "id verify request for tei NT %d",
			tm->l2->tei);
	tm->ri = -1;
	put_tei_msg(tm, ID_CHK_REQ, 0, tm->l2->tei);
	mISDN_FsmChangeState(&tm->tei_m, ST_TEI_IDVERIFY);
	test_and_set_bit(FLG_TEI_T201_1, &tm->l2->flag);
	mISDN_FsmAddTimer(&tm->t201, tm->T201, EV_T201, NULL, 2);
}

static void
tei_id_remove_tout(struct FsmInst *fi, int event, void *arg)
{
	teimgr_t *tm = fi->userdata;

	if (tm->debug)
		tm->tei_m.printdebug(fi, "remove req(2) tei NT %d",
			tm->l2->tei);
	put_tei_msg(tm, ID_REMOVE, 0, tm->val);
	mISDN_FsmChangeState(fi, ST_TEI_NOP);
}

static void
tei_id_ver_tout(struct FsmInst *fi, int event, void *arg)
{
	teimgr_t *tm = fi->userdata;

	if (tm->debug)
		tm->tei_m.printdebug(fi, "verify tout tei NT %d",
			tm->l2->tei);
	if (test_and_clear_bit(FLG_TEI_T201_1, &tm->l2->flag)) {
		put_tei_msg(tm, ID_CHK_REQ, 0, tm->l2->tei);
		tm->ri = -1;
		mISDN_FsmAddTimer(&tm->t201, tm->T201, EV_T201, NULL, 3);
	} else {
		mISDN_FsmChangeState(fi, ST_TEI_NOP);
		if (tm->ri == -1) {
			tm->tei_m.printdebug(fi, "tei NT %d check no response",
				tm->l2->tei);
			// remove tei
		} else
			tm->tei_m.printdebug(fi, "tei %d check ok",
				tm->l2->tei);
	}
}

int
l2_tei_net(teimgr_t *tm, struct sk_buff *skb)
{
	mISDN_head_t	*hh;
	int		ret = -EINVAL;

	if (!tm || !skb)
		return(ret);
	hh = mISDN_HEAD_P(skb);
	if (tm->debug)
		printk(KERN_DEBUG "%s: prim(%x)\n", __FUNCTION__, hh->prim);
	switch(hh->prim) {
		case (MDL_REMOVE | INDICATION):
			mISDN_FsmEvent(&tm->tei_m, EV_REMOVE, &hh->dinfo);
			break;
		case (MDL_ERROR | REQUEST):
			if (!test_bit(FLG_FIXED_TEI, &tm->l2->flag))
				mISDN_FsmEvent(&tm->tei_m, EV_VERIFY, NULL);
		break;
	}
	dev_kfree_skb(skb);
	return(0);
}

static void
tei_debug(struct FsmInst *fi, char *fmt, ...)
{
	teimgr_t	*tm = fi->userdata;
	logdata_t	log;
	char		head[29];

	va_start(log.args, fmt);
	sprintf(head,"tei NT %s", tm->l2->inst.name);
	log.fmt = fmt;
	log.head = head;
	mISDN_ctrl(&tm->l2->inst, MGR_DEBUGDATA | REQUEST, &log);
	va_end(log.args);
}

static struct FsmNode TeiFnList[] =
{
	{ST_TEI_NOP, EV_ASSIGN_REQ, tei_assign_req},
	{ST_TEI_NOP, EV_VERIFY, tei_id_verify},
	{ST_TEI_NOP, EV_REMOVE, tei_id_remove},
	{ST_TEI_REMOVE, EV_T201, tei_id_remove_tout},
	{ST_TEI_IDVERIFY, EV_T201, tei_id_ver_tout},
	{ST_TEI_IDVERIFY, EV_REMOVE, tei_id_remove},
	{ST_TEI_IDVERIFY, EV_CHECK_RES, tei_id_chk_res},
};

#define TEI_FN_COUNT (sizeof(TeiFnList)/sizeof(struct FsmNode))

void
release_tei_net(teimgr_t *tm)
{
	mISDN_FsmDelTimer(&tm->t201, 1);
	kfree(tm);
}

int
create_teimgr_net(layer2_t *l2) {
	teimgr_t *ntei;

	if (!l2) {
		printk(KERN_ERR "create_tei_net no layer2\n");
		return(-EINVAL);
	}
	if (!(ntei = kzalloc(sizeof(teimgr_t), GFP_ATOMIC))) {
		printk(KERN_ERR "kmalloc teimgr NT failed\n");
		return(-ENOMEM);
	}
	ntei->l2 = l2;
	ntei->T201 = 1000;	/* T201  1000 milliseconds */
	ntei->debug = l2->debug;
	ntei->tei_m.debug = l2->debug;
	ntei->tei_m.userdata = ntei;
	ntei->tei_m.printdebug = tei_debug;
	ntei->tei_m.fsm = &teifsm;
	ntei->tei_m.state = ST_TEI_NOP;
	mISDN_FsmInitTimer(&ntei->tei_m, &ntei->t201);
	l2->tm = ntei;
	return(0);
}

int
tei_mux_net(mISDNinstance_t *inst, struct sk_buff *skb)
{
	mISDN_head_t	*hh;
	u_char		*dp;
	int 		mt;
	layer2_t	*l2;
	unsigned int	ri, ai;

	hh = mISDN_HEAD_P(skb);
#if 0
		printk(KERN_DEBUG "%s: prim(%x) len(%d)\n", __FUNCTION__,
			hh->prim, skb->len);
#endif

	if (hh->prim != (MDL_UNITDATA | INDICATION)) {
		printk(KERN_WARNING "%s: prim(%x) unhandled\n", __FUNCTION__,
			hh->prim);
		return(-EINVAL);
	}
	if (skb->len < 8) {
		printk(KERN_WARNING "short tei NT mgr frame %d/8\n", skb->len);
		return(-EINVAL);
	}
	dp = skb->data + 2;
	if ((*dp & 0xef) != UI) {
		printk(KERN_WARNING "tei NT mgr frame is not ui %x\n", *dp);
		return(-EINVAL);
	}
	dp++;
	if (*dp++ != TEI_ENTITY_ID) {
		/* wrong management entity identifier, ignore */
		dp--;
		printk(KERN_WARNING "tei NT handler wrong entity id %x\n", *dp);
		return(-EINVAL);
	} else {
		mt = *(dp+2);
		ri = ((unsigned int) *dp++ << 8);
		ri += *dp++;
		dp++;
		ai = (unsigned int) *dp++;
		ai >>= 1;
#if 0
			printk(KERN_DEBUG "tei NT handler mt %x ri(%x) ai(%d)\n",
				mt, ri, ai);
#endif
		if (mt == ID_REQUEST) {
			if (ai != 127) {
				printk(KERN_WARNING "%s: ID_REQUEST ai(%d) not 127\n", __FUNCTION__,
					ai);
				return(-EINVAL);
			}
			l2 = new_tei_req_net(inst);
			if (!l2) {
				printk(KERN_WARNING "%s: no free tei\n", __FUNCTION__);
				return(-EBUSY);
			}
			l2->tm->ri = ri;
			put_tei_msg(l2->tm, ID_ASSIGNED, ri, l2->tei);
			dev_kfree_skb(skb);
			return(0);
		}
		l2 = find_tei_net(inst, ai);
		if (mt == ID_VERIFY) {
			if (l2) {
				mISDN_FsmEvent(&l2->tm->tei_m, EV_VERIFY, &ai);
			} else {
				l2 = find_tei_net(inst, 127);
				if (!l2) {
					printk(KERN_WARNING "%s: no 127 manager\n", __FUNCTION__);
					return(-EINVAL);
				}
				mISDN_FsmEvent(&l2->tm->tei_m, EV_REMOVE, &ai);
			}
		} else if (mt == ID_CHK_RES) {
			if (l2) {
				mISDN_FsmEvent(&l2->tm->tei_m, EV_CHECK_RES, &ri);
			} else {
				l2 = find_tei_net(inst, 127);
				if (!l2) {
					printk(KERN_WARNING "%s: no 127 manager\n", __FUNCTION__);
					return(-EINVAL);
				}
				mISDN_FsmEvent(&l2->tm->tei_m, EV_REMOVE, &ai);
			}
		} else {
			printk(KERN_WARNING "%s: wrong mt %x\n", __FUNCTION__, mt);
			return(-EINVAL);
		}
	}
	dev_kfree_skb(skb);
	return(0);
}
int TEIInit_net(void)
{
	teifsm.state_count = TEI_STATE_COUNT;
	teifsm.event_count = TEI_EVENT_COUNT;
	teifsm.strEvent = strTeiEvent;
	teifsm.strState = strTeiState;
	mISDN_FsmNew(&teifsm, TeiFnList, TEI_FN_COUNT);
	return(0);
}

void TEIFree_net(void)
{
	mISDN_FsmFree(&teifsm);
}

