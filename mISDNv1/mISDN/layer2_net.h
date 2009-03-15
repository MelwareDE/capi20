/* $Id: layer2_net.h,v 1.1 2008-11-04 11:52:19 armin Exp $
 *
 * Layer 2 NT defines
 *
 * This file is (c) under GNU PUBLIC LICENSE
 *
 */

#include <linux/mISDNif.h>
#include <linux/skbuff.h>
#include "fsm.h"
#ifdef MISDN_MEMDEBUG
#include "memdbg.h"
#endif

#define DINFO_SKB       -1

#define MAX_WINDOW	8


typedef struct _teimgr {
	int		ri;
	struct FsmInst	tei_m;
	struct FsmTimer	t201;
	int		T201;
	int		debug;
	int		val;
	struct _layer2	*l2;
} teimgr_t;

typedef struct _laddr {
	u_char	A;
	u_char	B;
} laddr_t;

typedef struct _layer2 {
	struct list_head	list;
	int			sapi;
	int			tei;
	laddr_t			addr;
	u_int			maxlen;
	teimgr_t		*tm;
	u_long			flag;
	u_int			vs, va, vr;
	int			rc;
	u_int			window;
	u_int			sow;
	int			entity;
	struct FsmInst		l2m;
	struct FsmTimer		t200, t203;
	int			T200, N200, T203;
	int			debug;
	mISDNinstance_t		inst;
	mISDNinstance_t		*instp;
//	mISDNif_t		*cloneif;
	int			next_id;
	u_int			down_id;
	struct sk_buff		*windowar[MAX_WINDOW];
	struct sk_buff_head	i_queue;
	struct sk_buff_head	ui_queue;
	struct sk_buff_head	down_queue;
	struct sk_buff_head	tmp_queue;
} layer2_t;

#define SAPITEI(ces)    (ces>>8)&0xff, ces&0xff
static inline int CES(layer2_t *l2) {
	return(l2->tei | (l2->sapi << 8));
}

/* l2 status_info */
typedef struct _status_info_l2 {
	int	len;
	int	typ;
	int	protocol;
	int	state;
	int	sapi;
	int	tei;
	laddr_t addr;
	u_int	maxlen;
	u_long	flag;
	u_int	vs;
	u_int	va;
	u_int	vr;
	int	rc;
	u_int	window;
	u_int	sow;
	int	T200;
	int	N200;
	int	T203;
	int	len_i_queue;
	int	len_ui_queue;
	int	len_d_queue;
	int	debug;
	int	tei_state;
	int	tei_ri;
	int	T201;
	int	tei_debug;
} status_info_l2_t;

/* from mISDN_l2.c */
extern int tei0_active_net(layer2_t *l2);
extern int tei_l2_net(layer2_t *l2, struct sk_buff *skb);
extern layer2_t *find_tei_net(mISDNinstance_t *inst, int tei);
extern layer2_t *new_tei_req_net(mISDNinstance_t *inst);

/* from tei_net.c */
extern int tei_mux_net(mISDNinstance_t *inst, struct sk_buff *skb);
extern int l2_tei_net(teimgr_t *tm, struct sk_buff *skb);
extern int create_teimgr_net(layer2_t *l2);
extern void release_tei_net(teimgr_t *tm);
extern int TEIInit_net(void);
extern void TEIFree_net(void);

#define GROUP_TEI	127
#define TEI_SAPI	63
#define CTRL_SAPI	0

#define RR	0x01
#define RNR	0x05
#define REJ	0x09
#define SABME	0x6f
#define SABM	0x2f
#define DM	0x0f
#define UI	0x03
#define DISC	0x43
#define UA	0x63
#define FRMR	0x87
#define XID	0xaf

#define CMD	0
#define RSP	1

#define LC_FLUSH_WAIT 1

#define FLG_LAPB	0
#define FLG_LAPD	1
#define FLG_ORIG	2
#define FLG_MOD128	3
#define FLG_PEND_REL	4
#define FLG_L3_INIT	5
#define FLG_T200_RUN	6
#define FLG_ACK_PEND	7
#define FLG_REJEXC	8
#define FLG_OWN_BUSY	9
#define FLG_PEER_BUSY	10
#define FLG_DCHAN_BUSY	11
#define FLG_L1_ACTIV	12
#define FLG_ESTAB_PEND	13
#define FLG_PTP		14
#define FLG_FIXED_TEI	15
#define FLG_L2BLOCK	16
#define FLG_L1_BUSY	17
#define FLG_LAPD_NET	18
#define FLG_TEI_T201_1  19

