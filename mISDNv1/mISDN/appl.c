/*
 *  $Id: appl.c,v 1.14 2006/03/06 12:52:07 keil Exp $
 *
 *  Applications are owned by the controller and only
 *  handle this controller, multiplexing multiple
 *  controller with one application is done in the higher
 *  driver independ CAPI driver. The application contain
 *  the Listen state machine.
 *
 */

#include "m_capi.h"
#include "helper.h"
#include "debug.h"
#include "mISDNManufacturer.h"

#define applDebug(appl, lev, fmt, args...) \
	capidebug(lev, fmt, ## args)

static struct list_head	garbage_applications =
    LIST_HEAD_INIT(garbage_applications);

int
ApplicationConstr(Controller_t *contr, __u16 ApplId, capi_register_params *rp)
{
	Application_t	*appl = kzalloc(sizeof(Application_t), GFP_ATOMIC);

	if (!appl)
		return -ENOMEM;

	INIT_LIST_HEAD(&appl->head);
	appl->contr = contr;
	appl->maxplci = contr->maxplci;
	appl->AppPlcis  = kzalloc(appl->maxplci * sizeof(AppPlci_t *),
	    GFP_ATOMIC);
	if (!appl->AppPlcis) {
		kfree(appl);
		return -ENOMEM;
	}
	appl->ApplId = ApplId;
	appl->MsgId = 1;
	appl->NotificationMask = 0;
	memcpy(&appl->reg_params, rp, sizeof(capi_register_params));
	listenConstr(appl);
	list_add(&appl->head, &contr->Applications);
	test_and_set_bit(APPL_STATE_ACTIV, &appl->state);
	return 0;
}

/*
 * Destroy the Application
 *
 * depending who initiate this we cannot release imediatly, if
 * any AppPlci is still in use.
 *
 * @who:   0 - a AppPlci is released in state APPL_STATE_RELEASE
 *         1 - Application is released from CAPI application
 *         2 - the controller is resetted
 *         3 - the controller is removed
 *         4 - the CAPI module will be unload
 */
int
ApplicationDestr(Application_t *appl, int who)
{
	int		i, used = 0;
	AppPlci_t	**aplci_p = appl->AppPlcis;

	if (test_and_set_bit(APPL_STATE_DESTRUCTOR, &appl->state)) {
		/* we are already in this function */
		return -EBUSY;
	}
	applDebug(appl, CAPI_DBG_APPL_INFO,
		"ApplicationDestr: appl=%p who=%d aplci_p=%p", appl, who, aplci_p);
	test_and_set_bit(APPL_STATE_RELEASE, &appl->state);
	test_and_clear_bit(APPL_STATE_ACTIV, &appl->state);
	listenDestr(appl);
	if (who > 2) {
		appl->contr = NULL;
	}
	if (aplci_p) {
		for (i = 0; i < appl->maxplci; i++) {
			if (*aplci_p) {
				switch (who) {
				case 4:
					AppPlciDestr(*aplci_p);
					*aplci_p = NULL;
					break;
				case 1:
				case 2:
				case 3:
					AppPlciRelease(*aplci_p);
				case 0:
					if ((volatile AppPlci_t *)(*aplci_p))
						used++;
					break;
				}
			}
			aplci_p++;
		}
	}
	if (used) {
		if (who == 3) {
			list_del_init(&appl->head);
			list_add(&appl->head, &garbage_applications);
		}
		test_and_clear_bit(APPL_STATE_DESTRUCTOR, &appl->state);
		applDebug(appl, CAPI_DBG_APPL_INFO,
			"ApplicationDestr: appl=%p still un use (%d)", appl, used);
		return -EBUSY;
	}
	list_del_init(&appl->head);
	appl->maxplci = 0;
	kfree(appl->AppPlcis);
	appl->AppPlcis = NULL;
	kfree(appl);
	return 0;
}

AppPlci_t *
getAppPlci4addr(Application_t *appl, __u32 addr)
{
	int plci_idx = (addr >> 8) & 0xff;

	if ((plci_idx < 1) || (plci_idx >= appl->maxplci)) {
		int_error();
		return NULL;
	}
	return appl->AppPlcis[plci_idx - 1];
}

static void send_hw_control(AppPlci_t *aplci, u32 message, u32 param1, u32 param2,
	u32 param3, u32 param4)
{
	struct sk_buff *nskb;
	u32 param[4];

	if (aplci->link) {
		param[0] = param1;
		param[1] = param2;
		param[2] = param3;
		param[3] = param4;
		nskb = create_link_skb(PH_CONTROL | REQUEST, message,
		    sizeof(param), param, 0);
		if (!nskb) {
			printk(KERN_ERR "%s: No mem for skb.\n", __FUNCTION__);
			return;
		}
		if (mISDN_queue_down(&aplci->link->inst, 0, nskb))
			dev_kfree_skb(nskb);
	}
}

static void
LineInterconnectFacilityReq(Application_t *appl, _cmsg *cmsg)
{
	u16	func = 0xff;
	u_char	*p = cmsg->FacilityRequestParameter;
	int len;
	unsigned int plci1, plci2=0, path1=0, path2=0;
	AppPlci_t	*aplci1;
	AppPlci_t	*aplci2;
	int conf_id;
	int send_ind = 0;
	unsigned char para[4];
	u32 pcm1, pcm2;

	plci1 = cmsg->adr.adrPLCI & 0xffff;
	conf_id = plci1;
	capi_cmsg_answer(cmsg);
	cmsg->Info = CapiIllMessageParmCoding;

	aplci1 = getAppPlci4addr(appl, plci1);
	if (!aplci1) {
		cmsg->Info = CapiIllContrPlciNcci;
		goto endli;
	}
	if (!p) {
		goto endli;
	}
	len = *p++;
	if (len > 1) {
		func = CAPIMSG_U16(p, 0);
		len -= 2;
		p += 2;
		if (len > 4) {
			path1 = CAPIMSG_U32(p, 1);
			p += 5;
			len -= 5;
			if (len > 9) {
				plci2 = CAPIMSG_U32(p, 2);
				path2 = CAPIMSG_U32(p, 6);
			}
		}
	}
	applDebug(appl, CAPI_DBG_APPL_INFO,
	    "LineInterconnectFacilityReq: func=%d plci1=%x plci2=%x path1=%x path2=%x",
	    func, plci1, plci2, path1, path2);
	if (func == 1) { /* connect */
		if (!plci2) {
			cmsg->Info = CapiIllContrPlciNcci;
			goto endli;
		}
		aplci2 = getAplci4plciId(plci2);
		if (!aplci2) {
			cmsg->Info = CapiIllContrPlciNcci;
			goto endli;
		}
		pcm1 = (((aplci1->channel & 0x3) - 1) | (((plci1 & 0xff) - 1) << 1));
		pcm2 = (((aplci2->channel & 0x3) - 1) | (((plci2 & 0xff) - 1) << 1));

		applDebug(appl, CAPI_DBG_APPL_INFO,
	    	"LineInterconnectFacilityReq: pcm1=%d pcm2=%d", pcm1, pcm2);

		if ((pcm1 > 31) || (pcm2 > 31)) {
			cmsg->Info = CapiMessageNotSupportedInCurrentState;
			goto endli;
		}
#if 0
		printk(KERN_DEBUG "LI: %d %d %d-%d %d %d\n",
			aplci1->features.pcm_id, aplci1->features.pcm_slots, aplci1->features.pcm_banks,
			aplci2->features.pcm_id, aplci2->features.pcm_slots, aplci2->features.pcm_banks);
#endif
		test_and_set_bit(APLCI_STATE_RX_OFF, &aplci1->state);
		test_and_set_bit(APLCI_STATE_RX_OFF, &aplci2->state);
		send_hw_control(aplci1, HW_PCM_CONN, pcm1, 0, pcm2, 0);
		send_hw_control(aplci2, HW_PCM_CONN, pcm2, 0, pcm1, 0);

		cmsg->FacilityRequestParameter[0] = 2;
		cmsg->Info = CAPI_NOERROR;
		send_ind = 1;
	} else if (func == 2) { /* disconnect */
		plci2 = path1;
		if (!plci2) {
			cmsg->Info = CapiIllContrPlciNcci;
			goto endli;
		}
		aplci2 = getAplci4plciId(plci2);
		if (!aplci2) {
			cmsg->Info = CapiIllContrPlciNcci;
			goto endli;
		}

		send_hw_control(aplci1, HW_PCM_DISC, 0, 0, 0, 0); 
		send_hw_control(aplci2, HW_PCM_DISC, 0, 0, 0, 0); 
		test_and_clear_bit(APLCI_STATE_RX_OFF, &aplci1->state);
		test_and_clear_bit(APLCI_STATE_RX_OFF, &aplci2->state);

		cmsg->FacilityRequestParameter[0] = 2;
		cmsg->Info = CAPI_NOERROR;
		send_ind = 1;
	}

endli:
	SendCmsg2Application(appl, cmsg);
	if (cmsg->Info == CAPI_NOERROR) {
		CMSG_ALLOC(cmsg);
		capi_cmsg_header(cmsg, appl->ApplId, CAPI_FACILITY, CAPI_IND, appl->MsgId++, plci1);
		cmsg->FacilitySelector = 0x0005;
		para[0] = 2;
		para[1] = (unsigned char)func;
		para[2] = 0;
		cmsg->FacilityIndicationParameter = para;
		SendCmsg2Application(appl, cmsg);
	}
}

static void
FacilityReq(Application_t *appl, struct sk_buff *skb)
{
	_cmsg		*cmsg;
	AppPlci_t	*aplci;
	Ncci_t		*ncci;

	cmsg = cmsg_alloc();
	if (!cmsg) {
		int_error();
		dev_kfree_skb(skb);
		return;
	}
	capi_message2cmsg(cmsg, skb->data);

	switch (cmsg->FacilitySelector) {
	case 0x0000: // Handset
	case 0x0001: // DTMF
		aplci = getAppPlci4addr(appl, CAPIMSG_CONTROL(skb->data));
		if (aplci) {
			ncci = getNCCI4addr(aplci, CAPIMSG_NCCI(skb->data),
			    GET_NCCI_PLCI);
			if (ncci) {
				ncciGetCmsg(ncci, cmsg);
				break;
			}
		}
		SendCmsgAnswer2Application(appl, cmsg, CapiIllContrPlciNcci);
		break;
	case 0x0003: // SupplementaryServices
		SupplementaryFacilityReq(appl, cmsg);
		break;
	case 0x0005: // Line-Interconnect
		LineInterconnectFacilityReq(appl, cmsg);
		break;
	default:
		int_error();
		SendCmsgAnswer2Application(appl, cmsg,
		    CapiFacilityNotSupported);
		break;
	}

	dev_kfree_skb(skb);
}

void
ApplicationSendMessage(Application_t *appl, struct sk_buff *skb)
{
	Plci_t		*plci;
	AppPlci_t	*aplci;
	__u16		ret;

	switch (CAPICMD(CAPIMSG_COMMAND(skb->data),
	    CAPIMSG_SUBCOMMAND(skb->data))) { // new NCCI
	case CAPI_CONNECT_B3_REQ:
		aplci = getAppPlci4addr(appl, CAPIMSG_CONTROL(skb->data));
		if (!aplci) {
			AnswerMessage2Application(appl, skb,
			    CapiIllContrPlciNcci);
			goto free;
		}
		ConnectB3Request(aplci, skb);
		break;
	// maybe already down NCCI
	case CAPI_DISCONNECT_B3_RESP:
		aplci = getAppPlci4addr(appl, CAPIMSG_CONTROL(skb->data));
		if (!aplci) {
			AnswerMessage2Application(appl, skb,
			    CapiIllContrPlciNcci);
			goto free;
		}
		DisconnectB3Request(aplci, skb);
		break;
	// for PLCI state machine
	case CAPI_INFO_REQ:
	case CAPI_ALERT_REQ:
	case CAPI_CONNECT_RESP:
	case CAPI_CONNECT_ACTIVE_RESP:
	case CAPI_DISCONNECT_REQ:
	case CAPI_DISCONNECT_RESP:
	case CAPI_SELECT_B_PROTOCOL_REQ:
		aplci = getAppPlci4addr(appl, CAPIMSG_CONTROL(skb->data));
		if (!aplci) {
			AnswerMessage2Application(appl, skb,
			    CapiIllContrPlciNcci);
			goto free;
		}
		ret = AppPlciSendMessage(aplci, skb);
		if (ret) {
			int_error();
		}
		break;
	case CAPI_CONNECT_REQ:
		if (ControllerNewPlci(appl->contr, &plci, MISDN_ID_ANY)) {
			AnswerMessage2Application(appl, skb,
			    CapiNoPlciAvailable);
			goto free;
		}
		aplci = ApplicationNewAppPlci(appl, plci);
		if (!aplci) {
			AnswerMessage2Application(appl, skb,
			    CapiNoPlciAvailable);
			goto free;
		}
		ret = AppPlciSendMessage(aplci, skb);
		if (ret)
			int_error();

		break;

	// for LISTEN state machine
	case CAPI_LISTEN_REQ:
		ret = listenSendMessage(appl, skb);
		if (ret) {
			int_error();
		}
		break;

	// other
	case CAPI_FACILITY_REQ:
		FacilityReq(appl, skb);
		break;
	case CAPI_FACILITY_RESP:
		goto free;
	case CAPI_MANUFACTURER_REQ:
		applManufacturerReq(appl, skb);
		break;
	case CAPI_INFO_RESP:
		goto free;
	default:
		applDebug(appl, CAPI_DBG_WARN,
		    "applSendMessage: %#x %#x not handled!",
		    CAPIMSG_COMMAND(skb->data), CAPIMSG_SUBCOMMAND(skb->data));
		break;
	}
	return;
free:
	dev_kfree_skb(skb);
}

AppPlci_t *
ApplicationNewAppPlci(Application_t *appl, Plci_t *plci)
{
	AppPlci_t	*aplci;
	int		plci_idx = (plci->addr >> 8) & 0xff;

	if (test_bit(APPL_STATE_RELEASE, &appl->state))
		return NULL;
	if ((plci_idx < 1) || (plci_idx >= appl->maxplci)) {
		int_error();
		return NULL;
	}
	if (appl->AppPlcis[plci_idx - 1]) {
		int_error();
		return NULL;
	}
	if (AppPlciConstr(&aplci, appl, plci)) {
		int_error();
		return NULL;
	}
	applDebug(appl, CAPI_DBG_APPL_INFO,
	    "ApplicationNewAppPlci: idx(%d) aplci(%p) appl(%p) plci(%p)",
	    plci_idx, aplci, appl, plci);
	appl->AppPlcis[plci_idx - 1] = aplci;
	plciAttachAppPlci(plci, aplci);
	return aplci;
}

void
ApplicationDelAppPlci(Application_t *appl, AppPlci_t *aplci)
{
	int	plci_idx = (aplci->addr >> 8) & 0xff;

	if ((plci_idx < 1) || (plci_idx >= appl->maxplci)) {
		int_error();
		return;
	}
	if (appl->AppPlcis[plci_idx - 1] != aplci) {
		int_error();
		return;
	}
	appl->AppPlcis[plci_idx - 1] = NULL;
	if (test_bit(APPL_STATE_RELEASE, &appl->state) &&
		!test_bit(APPL_STATE_DESTRUCTOR, &appl->state))
		ApplicationDestr(appl, 0);
}

void
SendCmsg2Application(Application_t *appl, _cmsg *cmsg)
{
	struct sk_buff	*skb;
	unsigned int debugval = CAPI_DBG_APPL_MSG;

	if (test_bit(APPL_STATE_RELEASE, &appl->state)) {
		/*
		 * Application is released and cannot receive messages
		 * anymore. To avoid stalls in the state machines we
		 * must answer INDICATIONS.
		 */
		AppPlci_t	*aplci;
		Ncci_t		*ncci;

		if (CAPI_IND != cmsg->Subcommand)
			goto free;
		switch (cmsg->Command) {
		// for NCCI state machine
		case CAPI_CONNECT_B3:
			cmsg->Reject = 2;
		case CAPI_CONNECT_B3_ACTIVE:
		case CAPI_DISCONNECT_B3:
			aplci = getAppPlci4addr(appl,
				(cmsg->adr.adrNCCI & 0xffff));
			if (!aplci)
				goto free;
			ncci = getNCCI4addr(aplci,
				cmsg->adr.adrNCCI, GET_NCCI_EXACT);
			if (!ncci) {
				int_error();
				goto free;
			}
			capi_cmsg_answer(cmsg);
			ncciGetCmsg(ncci, cmsg);
			break;
		// for PLCI state machine
		case CAPI_CONNECT:
			cmsg->Reject = 2;
		case CAPI_CONNECT_ACTIVE:
		case CAPI_DISCONNECT:
			aplci = getAppPlci4addr(appl,
			    (cmsg->adr.adrPLCI & 0xffff));
			if (!aplci)
				goto free;
			capi_cmsg_answer(cmsg);
			AppPlciGetCmsg(aplci, cmsg);
			break;
		case CAPI_FACILITY:
		case CAPI_MANUFACTURER:
		case CAPI_INFO:
			goto free;
		default:
			int_error();
			goto free;
		}
		return;
	}
	if (!(skb = alloc_skb(CAPI_MSG_DEFAULT_LEN, GFP_ATOMIC))) {
		printk(KERN_WARNING "%s: no mem for %d bytes\n",
		    __FUNCTION__, CAPI_MSG_DEFAULT_LEN);
		int_error();
		goto free;
	}
	capi_cmsg2message(cmsg, skb->data);
	if (cmsg->Command == CAPI_DATA_B3)
		debugval = CAPI_DBG_NCCI_L3;
	applDebug(appl, debugval,
	    "%s: len(%d) applid(%x) %s msgnr(%d) addr(%08x)",
	    __FUNCTION__, CAPIMSG_LEN(skb->data), cmsg->ApplId,
	    capi_cmd2str(cmsg->Command, cmsg->Subcommand),
		cmsg->Messagenumber, cmsg->adr.adrController);
	if (CAPI_MSG_DEFAULT_LEN < CAPIMSG_LEN(skb->data)) {
		printk(KERN_ERR
		    "%s: CAPI_MSG_DEFAULT_LEN overrun (%d/%d)\n",
		    __FUNCTION__, CAPIMSG_LEN(skb->data),
		    CAPI_MSG_DEFAULT_LEN);
		int_error();
		dev_kfree_skb(skb);
		goto free;
	}
	skb_put(skb, CAPIMSG_LEN(skb->data));
#ifdef OLDCAPI_DRIVER_INTERFACE
	appl->contr->ctrl->handle_capimsg(appl->contr->ctrl, cmsg->ApplId, skb);
#else
	capi_ctr_handle_message(appl->contr->ctrl, cmsg->ApplId, skb);
#endif
free:
	cmsg_free(cmsg);
}

void
SendCmsgAnswer2Application(Application_t *appl, _cmsg *cmsg, __u16 Info)
{
	capi_cmsg_answer(cmsg);
	cmsg->Info = Info;
	SendCmsg2Application(appl, cmsg);
}

void
AnswerMessage2Application(Application_t *appl, struct sk_buff *skb, __u16 Info)
{
	_cmsg	*cmsg;

	CMSG_ALLOC(cmsg);
	capi_message2cmsg(cmsg, skb->data);
	SendCmsgAnswer2Application(appl, cmsg, Info);
}

#define AVM_MANUFACTURER_ID	0x214D5641 /* "AVM!" */
#define CLASS_AVM		0x00
#define FUNCTION_AVM_D2_TRACE	0x01

struct AVMD2Trace {
	__u8 Length;
	__u8 data[4];
};

void applManufacturerReqAVM(Application_t *appl, _cmsg *cmsg,
    struct sk_buff *skb)
{
	struct AVMD2Trace *at;

	if (cmsg->Class != CLASS_AVM) {
		applDebug(appl, CAPI_DBG_APPL_INFO,
		    "CAPI: unknown class %#x\n", cmsg->Class);
		cmsg_free(cmsg);
		dev_kfree_skb(skb);
		return;
	}
	switch (cmsg->Function) {
	case FUNCTION_AVM_D2_TRACE:
		at = (struct AVMD2Trace *)cmsg->ManuData;
		if (!at || at->Length != 4) {
			int_error();
			break;
		}
		if (memcmp(at->data, "\200\014\000\000", 4) == 0) {
			test_and_set_bit(APPL_STATE_D2TRACE, &appl->state);
		} else if (memcmp(at->data, "\000\000\000\000", 4) == 0) {
			test_and_clear_bit(APPL_STATE_D2TRACE, &appl->state);
		} else {
			int_error();
		}
		break;
	default:
		applDebug(appl, CAPI_DBG_APPL_INFO,
		    "CAPI: unknown function %#x\n", cmsg->Function);
	}
	cmsg_free(cmsg);
	dev_kfree_skb(skb);
}

void applManufacturerReqmISDN(Application_t *appl, _cmsg *cmsg,
    struct sk_buff *skb)
{
	AppPlci_t	*aplci;
	Ncci_t		*ncci;

	switch (cmsg->Class) {
	case mISDN_MF_CLASS_HANDSET:
		/*
		 *  Note normally MANUFATURER messages are only defined for
		 * controller address we extent it here to PLCI/NCCI
		 */
		aplci = getAppPlci4addr(appl, CAPIMSG_CONTROL(skb->data));
		if (aplci) {
			ncci = getNCCI4addr(aplci, CAPIMSG_NCCI(skb->data),
			    GET_NCCI_PLCI);
			if (ncci) {
				cmsg_free(cmsg);
				ncciSendMessage(ncci, skb);
				return;
			}
		}
		SendCmsgAnswer2Application(appl, cmsg, CapiIllContrPlciNcci);
		break;
	default:
		cmsg_free(cmsg);
		break;
	}
	dev_kfree_skb(skb);
}

void
applManufacturerReq(Application_t *appl, struct sk_buff *skb)
{
	_cmsg	*cmsg;

	if (skb->len < 16 + 8) {
		dev_kfree_skb(skb);
		return;
	}
	cmsg = cmsg_alloc();
	if (!cmsg) {
		int_error();
		dev_kfree_skb(skb);
		return;
	}
	capi_message2cmsg(cmsg, skb->data);
	switch (cmsg->ManuID) {
	case mISDN_MANUFACTURER_ID:
		applManufacturerReqmISDN(appl, cmsg, skb);
		break;
	case AVM_MANUFACTURER_ID:
		applManufacturerReqAVM(appl, cmsg, skb);
		break;
	default:
		applDebug(appl, CAPI_DBG_APPL_INFO,
			"CAPI: unknown ManuID %#x\n", cmsg->ManuID);
		cmsg_free(cmsg);
		dev_kfree_skb(skb);
		break;
	}
}

void
applD2Trace(Application_t *appl, u_char *buf, int len)
{
	_cmsg	*cmsg;
	__u8	manuData[255];

	if (!test_bit(APPL_STATE_D2TRACE, &appl->state))
		return;

	CMSG_ALLOC(cmsg);
	capi_cmsg_header(cmsg, appl->ApplId, CAPI_MANUFACTURER, CAPI_IND,
	    appl->MsgId++, appl->contr->addr);
	cmsg->ManuID = AVM_MANUFACTURER_ID;
	cmsg->Class = CLASS_AVM;
	cmsg->Function = FUNCTION_AVM_D2_TRACE;
	cmsg->ManuData = (_cstruct) &manuData;
	manuData[0] = 2 + len; // length
	manuData[1] = 0x80;
	manuData[2] = 0x0f;
	memcpy(&manuData[3], buf, len);

	SendCmsg2Application(appl, cmsg);
}

void
free_Application(void)
{
	struct list_head	*item, *next;
	int			n = 0;

	if (list_empty(&garbage_applications)) {
		printk(KERN_DEBUG "%s: no garbage\n", __FUNCTION__);
		return;
	}
	list_for_each_safe(item, next, &garbage_applications) {
		ApplicationDestr((Application_t *)item, 4);
		n++;
	}
	printk(KERN_WARNING"%s: %d garbage items\n", __FUNCTION__, n);
}
