#include <stdio.h>
#include <capi20.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <linux/isdn.h>
#include <errno.h>

#define __u32 unsigned int
#define __u16 unsigned short
#define __u8  unsigned char

#include "capi_enc.c"

char           *progname;

#define error(fmt, args...) do { \
        fprintf(stderr, fmt, ## args); \
	exit (1); \
} while (0)

void check_put_cmsg(_cmsg *cmsg)
{
	int err;

	err = capi_put_cmsg(cmsg);
	if (err != CapiNoError) 
		error("put cmsgcmd %#x subcmd %#x err %#x", 
		      cmsg->Command, cmsg->Subcommand, err);
}

void check_wait_get_cmsg(_cmsg *cmsg, unsigned ApplId)
{
	int err;

	err = capi20_waitformessage(ApplId, 0);
	if (err != CapiNoError)
		error("WAITING - (%#x)\n", err);
	err = capi_get_cmsg(cmsg, ApplId);
	if (err != CapiNoError)
		error("GET CMSG - (%#x)\n", err);
}

char *proc2string[] = {
	[0] "cfu",
	[1] "cfb",
	[2] "cfnr",
};

char *bs2string[43] = {
	[0 ] "allServices",
	[1 ] "speech",
	[2 ] "unrestrictedDigitialInformation",
	[3 ] "audio3100Hz",
	[4 ] "unrestrictedDigitalInformationWithTonesAndAnnouncements",
	[5 ] "multirate",
	[32] "telephony",
	[33] "teletex",
	[34] "telefaxGroup4Class1",
	[35] "videotexSyntaxBased",
	[36] "videotelephony",
	[37] "telefaxGroup2-3",
	[38] "telephony7kHz",
	[39] "euroFileTransfer",
	[40] "fileTransferAndAccessManagement",
	[41] "videoconference",
	[42] "audioGraphicConference",
};

char *procedure2string(int procedure)
{
	if (procedure < 0 || procedure > 42)
		return "?";
	return proc2string[procedure];
}

char *basicService2string(int bs)
{
	if (bs < 0 || bs > 42)
		return "?";
	if (bs2string[bs])
		return bs2string[bs];
	else
		return "?";
}

int parseFacilityPartyNumber(__u8 *msg)
{
	char number[30];

	memcpy(number, msg+4, msg[0]-3);
	number[msg[0]-3] = 0;
	printf("%s ", number);

	return msg[0] + 1;
}

int parseInterrogateResponse(__u8 *msg)
{
	__u8 *p;
	int procedure;
	int basicService;
	
	p = &msg[1];
	procedure = *p++;
	procedure |= *p++ << 8;
	basicService = *p++;
	basicService |= *p++ << 8;
	printf("%s %s ", procedure2string(procedure), 
	       basicService2string(basicService));
	p += parseFacilityPartyNumber(p);
	p += parseFacilityPartyNumber(p);
	printf("\n");
	// FIXME subaddress

	return msg[0] + 1;
}

int parseInterrogateNumbers(__u8 *msg)
{
	__u8 *end = msg + msg[0];
	__u8 *p = &msg[1];

	printf("Call forwarding activated on: ");
	while (p < end) {
		p += parseFacilityPartyNumber(p);
	}
	printf("\n");
	return msg[0] + 1;
}

int parseInterrogateParameters(__u8 *msg)
{
	__u8 *end = msg + msg[0];
	__u8 *p = &msg[1];

	while (p < end) {
		p += parseInterrogateResponse(p);
	}
	return msg[0] + 1;
}

void usage()
{
	int i;

	fprintf(stderr, "usage: %s [a|d|i|I] ...\n", 
		progname);
	fprintf(stderr, "       a <procedure> <bs> <msn> <diverted_to>\n");
	fprintf(stderr, "         activate diversion\n");
	fprintf(stderr, "       d <procedure> <bs> <msn>\n");
	fprintf(stderr, "         deactivate diversion\n");
	fprintf(stderr, "       i\n");
	fprintf(stderr, "         interrogate diverted numbers\n");
	fprintf(stderr, "       I <procedure> <bs> <msn>\n");
	fprintf(stderr, "         interrogate diversion details\n");
	fprintf(stderr, "   \n");
	fprintf(stderr, "   <procedure>  : cfu|cfb|cfnr (call forwarding unconditional, busy, no reply)\n");
	fprintf(stderr, "   <bs>         : Basic Service,\n");
	for (i = 0; i <= 42; i++) {
		if (bs2string[i]) {
			fprintf(stderr, "             %2d : %s\n",
				i, bs2string[i]);
		}
	}
	fprintf(stderr, "   <msn>        : MSN to be forwarded\n");
	fprintf(stderr, "   <diverted_to>: forwarded to number\n");
	exit(-1);
}

int main(int argc, char *argv[])
{
	__u16 Procedure;
	__u16 Function;
	__u16 BasicService;
	__u8 *ServedUserNumber;
	__u8 *DivertedToNumber;
	__u16 Reason, Info;

	unsigned ApplId;
	unsigned err;
	unsigned MsgId = 0;
	unsigned controller = 1;
	__u8 tmp[128], *p;
	_cmsg cmsg;

	if ((progname = strrchr(argv[0], '/')))
		progname++;
	else
		progname = argv[0];
  
	if (argc < 2)
		usage();

	if (argv[1][0] == 'a') {
		if (argc != 6)
			usage();
		Function = 0x0009;

	} else if (argv[1][0] == 'd') {
		if (argc != 5)
			usage();
		Function = 0x000a;
	} else if (argv[1][0] == 'i') {
		if (argc != 2)
			usage();
		Function = 0x000c;
	} else if (argv[1][0] == 'I') {
		if (argc != 5)
			usage();
		Function = 0x000b;
	} else {
		usage();
	}
	
	if (Function != 0x000c) {
		if (strcmp(argv[2], "cfu") == 0)
			Procedure = 0;
		else if (strcmp(argv[2], "cfb") == 0)
			Procedure = 1;
		else if (strcmp(argv[2], "cfnr") == 0)
			Procedure = 2;
		else
			usage();
		
		BasicService = strtol(argv[3],NULL,0);
		
		ServedUserNumber = argv[4];
		
		if (Function == 0x00009) 
			DivertedToNumber = argv[5];
	}
	
	if (capi20_isinstalled() != CapiNoError) {
		fprintf(stderr, "capi not installed - %s (%d)\n", strerror(errno), errno);
		return 1;
	}
		
	err = capi20_register(2, 7, 2048, &ApplId);
	if (err != CapiNoError) {
		fprintf(stderr, "could not register - (%#x)\n", err);
		return 1;
	}

	FACILITY_REQ_HEADER(&cmsg, ApplId, MsgId++, controller);
	cmsg.FacilitySelector = 0x0003;
	
	p = &tmp[1];
	p += capiEncodeWord(p, Function);
	switch (Function) {
	case 0x0009:
		p += capiEncodeFacReqCFact(p, Procedure, BasicService, 
					   ServedUserNumber, DivertedToNumber);
		break;
	case 0x000a:
		p += capiEncodeFacReqCFdeact(p, Procedure, BasicService,
					     ServedUserNumber);
		break;
	case 0x000b:
		p += capiEncodeFacReqCFinterParameters(p, Procedure, BasicService,
						       ServedUserNumber);
		break;
	case 0x000c:
		p += capiEncodeFacReqCFinterNumbers(p);
		break;
	}
	tmp[0] = p - &tmp[1];

	cmsg.FacilityRequestParameter = tmp;

	check_put_cmsg(&cmsg); // FACILITY REQ

	check_wait_get_cmsg(&cmsg, ApplId); // FACILITY CONF
	if (cmsg.Info != 0x0000) 
		error("FACILITY CONF: Info %#x\n", cmsg.Info);
	Info = cmsg.FacilityConfirmationParameter[4];
	Info |= cmsg.FacilityConfirmationParameter[5] << 8;
	if (Info != 0x0000)
		error("FACILITY CONF: SuppInfo %#x\n", Info);

	check_wait_get_cmsg(&cmsg, ApplId); // FACILITY IND
	if (cmsg.Info != 0x0000) 
		error("FACILITY IND: Info %#x\n", cmsg.Info);
	Reason = cmsg.FacilityIndicationParameter[4];
	Reason |= cmsg.FacilityIndicationParameter[5] << 8;

	if (Reason == 0x0000) {
		switch (Function) {
		case 0x0009:
		case 0x000a:
			printf("succeeded.\n");
			break;
		case 0x000b:
			parseInterrogateParameters(&cmsg.FacilityIndicationParameter[10]);
			break;
		case 0x000c:
			parseInterrogateNumbers(&cmsg.FacilityIndicationParameter[10]);
			break;
		}
	} else {
		printf("failed, Reason %#04x\n", Reason);
	}

	capi_cmsg_answer(&cmsg);
	check_put_cmsg(&cmsg); // FACILITY RESP

	return 0;
}
