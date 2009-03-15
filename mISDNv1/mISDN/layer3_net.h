/* $Id: layer3_net.h,v 1.4 2008-11-12 17:54:20 armin Exp $
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

#define SBIT(state) (1<<state)
#define ALL_STATES  0x03ffffff

#define PROTO_DIS_EURO	0x08

#define L3_DEB_WARN	0x01
#define L3_DEB_PROTERR	0x02
#define L3_DEB_STATE	0x04
#define L3_DEB_CHARGE	0x08
#define L3_DEB_PROC		0x08
#define L3_DEB_CHECK	0x10
#define L3_DEB_SI	0x20
#define L3_DEB_MSG	0x80000000

#define FLG_L2BLOCK	1
#define FLG_PTP		2
#define FLG_EXTCID	3
#define FLG_CRLEN2	4

/* NT */
#define FLG_L3P_TIMER312    1
#define FLG_L3P_TIMER303_1  2
#define FLG_L3P_TIMER308_1  3
#define FLG_L3P_GOTRELCOMP  4

enum {
	IMSG_END_PROC,
	IMSG_END_PROC_M,
	IMSG_L2_DATA,
	IMSG_L4_DATA,
	IMSG_TIMER_EXPIRED,
	IMSG_MASTER_L2_DATA,
	IMSG_PROCEEDING_IND,
	IMSG_ALERTING_IND,
	IMSG_CONNECT_IND,
	IMSG_SEL_PROC,
	IMSG_RELEASE_CHILDS,
};

enum {
	ST_L3_LC_REL,
	ST_L3_LC_ESTAB_WAIT,
	ST_L3_LC_REL_DELAY,
	ST_L3_LC_REL_WAIT,
	ST_L3_LC_ESTAB,
};

typedef struct _L3Timer {
	struct _l3_process	*pc;
	struct timer_list	tl;
	int			nr;
} L3Timer_t;

typedef struct _l3_process {
	struct list_head	list;
	struct list_head    childlist;
	struct _l3_process *master;
	struct _layer3		*l3;
	int			callref;
	int			state;
	int cause;
	u_int			id;
	int			bc;
	int			err;
	/* NT extra */
	int		ces;
	int		selces;
	unsigned long	Flags;
	L3Timer_t		timer1;
	L3Timer_t		timer2;
	int		hold_state;
	unsigned int l4id;
	u_char      obuf[MAX_DFRAME_LEN];
	int			obuflen;
} l3_process_t;

typedef struct _layer3 {
	struct list_head	list;
	int			entity;
	struct list_head	proclist;
	int			down_headerlen;
	u_int			id;
	int			debug;
	u_long			Flag;
	mISDNinstance_t		inst;
	struct sk_buff_head	squeue;
	/* NT */
	mISDNinstance_t *instp;
	int		l2_state0;
	int		next_cr;
} layer3_t;

struct stateentry {
	int	state;
	unsigned int	primitive;
	void (*rout) (l3_process_t *, int, void *);
};

extern int		send_proc(l3_process_t *proc, int op, void *arg);
extern void		L3InitTimer_net(l3_process_t *, L3Timer_t *);
extern void		L3DelTimer_net(L3Timer_t *);
extern int		L3AddTimer_net(L3Timer_t *, int, int);
extern void		StopAllL3Timer_net(l3_process_t *);
extern void		init_l3_net(layer3_t *);
extern void		release_l3_net(layer3_t *);
extern void		l3_debug_net(layer3_t *, char *, ...);

