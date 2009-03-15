/*
 * $Id: layer3_net.c,v 1.4 2008-11-15 15:46:02 armin Exp $
 *
 * Author       Karsten Keil (keil@isdn4linux.de)
 * Adapted for NT mode    Armin Schindler (armin@melware.de)
 *
 *		This file is (c) under GNU PUBLIC LICENSE
 *		For changes and modifications please read
 *		../../../Documentation/isdn/mISDN.cert
 *
 * Thanks to    Jan den Ouden
 *              Fritz Elfert
 *
 */
#include "layer3_net.h"
#include "helper.h"
#include "dss1.h"

void
l3_debug_net(layer3_t *l3, char *fmt, ...)
{
	logdata_t log;

	va_start(log.args, fmt);
	log.fmt = fmt;
	log.head = l3->inst.name;
	mISDN_ctrl(&l3->inst, MGR_DEBUGDATA | REQUEST, &log);
	va_end(log.args);
}

void
L3DelTimer_net(L3Timer_t *t)
{
	if (t->pc->l3->debug & L3_DEB_STATE)
		l3_debug_net(t->pc->l3, "L3DelTimer_net %x proc=%p", t->nr, t->pc);

	del_timer(&t->tl);
}

int
L3AddTimer_net(L3Timer_t *t, int millisec, int timer_nr)
{
	if (t->pc->l3->debug & L3_DEB_STATE)
		l3_debug_net(t->pc->l3, "L3AddTimer_net %x proc=%p", timer_nr, t->pc);

	if (timer_pending(&t->tl)) {
		return -1;
	}
	init_timer(&t->tl);
	t->nr = timer_nr;
	t->tl.expires = jiffies + (millisec * HZ) / 1000;
	add_timer(&t->tl);
	return 0;
}

static void
L3ExpireTimer(L3Timer_t *t)
{
	if (t->pc->l3->debug & L3_DEB_STATE)
		l3_debug_net(t->pc->l3, "timerNT %p nr %x expired", t, t->nr);
	send_proc(t->pc, IMSG_TIMER_EXPIRED, &t->nr);
}

void
L3InitTimer_net(l3_process_t *pc, L3Timer_t *t)
{
	t->pc = pc;
	t->tl.function = (void *) L3ExpireTimer;
	t->tl.data = (long) t;
	init_timer(&t->tl);
}

void
StopAllL3Timer_net(l3_process_t *pc)
{
	if (pc->l3->debug & L3_DEB_STATE)
		l3_debug_net(pc->l3, "StopAllL3Timer_net proc=%p", pc);

	L3DelTimer_net(&pc->timer1);
	L3DelTimer_net(&pc->timer2);

	test_and_clear_bit(FLG_L3P_TIMER303_1, &pc->Flags);
	test_and_clear_bit(FLG_L3P_TIMER308_1, &pc->Flags);
	test_and_clear_bit(FLG_L3P_TIMER312, &pc->Flags);
}

void
init_l3_net(layer3_t *l3)
{
	INIT_LIST_HEAD(&l3->proclist);
	l3->entity = MISDN_ENTITY_NONE;
	skb_queue_head_init(&l3->squeue);
}

void
release_l3_net(layer3_t *l3)
{
	l3_process_t *p, *np;

	list_for_each_entry_safe(p, np, &l3->proclist, list) {
		send_proc(p, IMSG_END_PROC, NULL);
	}
	discard_queue(&l3->squeue);
}

