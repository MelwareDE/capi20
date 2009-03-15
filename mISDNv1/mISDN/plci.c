/* $Id: plci.c,v 1.12 2006/03/06 12:52:07 keil Exp $
 *
 */

#include "m_capi.h"
#include "dss1.h"
#include "helper.h"
#include "debug.h"

#define plciDebug(plci, lev, fmt, args...) \
        capidebug(lev, fmt, ## args)


void plciInit(Controller_t *contr)
{
	Plci_t	*plci = contr->plcis;
	int	i;

	for (i = 0; i < contr->maxplci; i++) {
		memset(plci, 0, sizeof(Plci_t));
		plci->addr = ((i + 1) << 8) | contr->addr;
		plci->l3id = MISDN_ID_NONE;
		INIT_LIST_HEAD(&plci->AppPlcis);
		plci->contr = contr;
		if (contr->debug & CAPI_DBG_PLCI)
			printk(KERN_DEBUG "%s: %p PLCI(%x) l3id(%x)\n",
				__FUNCTION__, plci, plci->addr, plci->l3id);
		plci++;
	}
}

void plciHandleSetupInd(Plci_t *plci, int pr, Q931_info_t *qi)
{
	__u16			CIPValue;
	Application_t		*appl;
	AppPlci_t		*aplci;
	struct list_head	*item, *next;
	int afound = 0;
	u_char causebyte = 0xd8; /* incompatible destination */
	int chid = 0, chwanted = 0x0;
	u_char		*ie;

	if (!qi || !plci->contr) {
		int_error();
		return;
	}

	CIPValue = q931CIPValue(qi);

	if (plci->contr->nt) {
		if (qi->channel_id.off) {
			ie = (u_char *)qi;
			ie += L3_EXTRA_SIZE + qi->channel_id.off;
			chwanted = plci_parse_channel_id(ie);
		}
		if (!(chid = ContrAllocFreeChannel(plci->contr, chwanted))) {
			causebyte = 0xa2; /* no channel */
			goto nochannel;
		}
	}

	list_for_each_safe(item, next, &plci->contr->Applications) {
		appl = (Application_t *)item;
		if (test_bit(APPL_STATE_RELEASE, &appl->state))
			continue;
		if (listenHandle(appl, CIPValue)) {
			aplci = ApplicationNewAppPlci(appl, plci);
			if (!aplci) {
				int_error();
				break;
			}
			AppPlci_l3l4(aplci, pr, qi);
			if (afound == 0) {
				afound = 1;
				if (aplci->contr->nt) {
					struct sk_buff *skb = mISDN_alloc_l3msg(10, MT_SETUP_ACKNOWLEDGE);
					u_char cid[3] = {IE_CHANNEL_ID,1,0x00};
					aplci->bch = chid;
					cid[2] = 0x88 | chid;
					aplci->channel = cid[2];
					if (skb) {
						mISDN_AddvarIE(skb,cid);
						plciL4L3(plci, CC_SETUP_ACKNOWLEDGE | REQUEST, skb);
					}
				}
			}
		}
	}
nochannel:
	if (plci->nAppl == 0) {
		struct sk_buff *skb = mISDN_alloc_l3msg(10, MT_RELEASE_COMPLETE);
		u_char cause[4] = {IE_CAUSE,2,0x80,0x00};

		if (skb) {
			cause[3] = causebyte;
			mISDN_AddvarIE(skb,cause);
			plciL4L3(plci, CC_RELEASE_COMPLETE | REQUEST, skb);
		}
		if (plci->contr->nt) {
			ContrFreeChannel(plci->contr, chid);
		}
		ControllerReleasePlci(plci);
	}
}

int plci_l3l4(Plci_t *plci, int pr, struct sk_buff *skb)
{
	AppPlci_t		*aplci;
	Q931_info_t		*qi;
	struct list_head	*item, *next;

	if (skb->len)
		qi = (Q931_info_t *)skb->data;
	else
		qi = NULL;
	switch (pr) {
		case CC_SETUP | INDICATION:
			plciHandleSetupInd(plci, pr, qi);
			break;
		/*
		// no extra treatment for CC_RELEASE_CR | INDICATION ! 
		case CC_RELEASE_CR | INDICATION:
			break;
		*/
		default:
			list_for_each_safe(item, next, &plci->AppPlcis) {
				aplci = (AppPlci_t *)item;
				AppPlci_l3l4(aplci, pr, qi);
			}
			break;
	}
	dev_kfree_skb(skb);
	return(0);
}

AppPlci_t *
getAppPlci4Id(Plci_t *plci, __u16 appId) {
	struct list_head	*item;
	AppPlci_t		*aplci;

	list_for_each(item, &plci->AppPlcis) {
		aplci = (AppPlci_t *)item;
		if (appId == aplci->appl->ApplId)
			return(aplci);
	}
	return(NULL);
}

void plciAttachAppPlci(Plci_t *plci, AppPlci_t *aplci)
{
	AppPlci_t	*test = getAppPlci4Id(plci, aplci->appl->ApplId);

	if (test) {
		int_error();
		return;
	}
	list_add(&aplci->head, &plci->AppPlcis);
	plci->nAppl++;
}

void
plciDetachAppPlci(Plci_t *plci, AppPlci_t *aplci)
{
	aplci->plci = NULL;
	list_del_init(&aplci->head);
	plci->nAppl--;
	if (!plci->nAppl)
		ControllerReleasePlci(plci);
}

void plciNewCrReq(Plci_t *plci)
{
	plciL4L3(plci, CC_NEW_CR | REQUEST, NULL);
}

int
plciL4L3(Plci_t *plci, __u32 prim, struct sk_buff *skb)
{
#define	MY_RESERVE	8
	int	err;

	if (!skb) {
		if (!(skb = alloc_skb(MY_RESERVE, GFP_ATOMIC))) {
			printk(KERN_WARNING "%s: no skb size %d\n",
				__FUNCTION__, MY_RESERVE);
			return(-ENOMEM);
		} else
			skb_reserve(skb, MY_RESERVE);
	}
	err = ControllerL4L3(plci->contr, prim, plci->l3id, skb);
	if (err)
		dev_kfree_skb(skb);
	return(err);
}
