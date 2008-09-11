/* 
 *
 * A CAPI application to get infomation about installed controllers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#define _LINUX_LIST_H
#include <capi20.h>
#include <linux/capi.h>

struct bittext {
   __u32 bit;
   char *text;
};

#define GET_WORD(p)    (((unsigned char*)p)[0] + (((unsigned char*)p)[1] << 8))
#define GET_DWORD(p)   (((unsigned char*)p)[0] + (((unsigned char*)p)[1] << 8) + \
			(((unsigned char*)p)[2] << 16) + (((unsigned char*)p)[3] << 24))

struct bittext goptions[] = {
/*  0 */ { 0x0001, "internal controller supported" },
/*  1 */ { 0x0002, "external equipment supported"},
/*  2 */ { 0x0004, "handset supported" },
/*  3 */ { 0x0008, "DTMF supported" },
/*  4 */ { 0x0010, "Supplementary Services supported" },
/*  5 */ { 0x0020, "channel allocation supported (leased lines)" },
 { 0, 0 }
};

struct bittext b1support[] = {
/*  0 */ { 0x0001, "64 kbit/s with HDLC framing" },
/*  1 */ { 0x0002, "64 kbit/s bit-transparent operation" },
/*  2 */ { 0x0004, "V.110 asynconous operation with start/stop byte framing" },
/*  3 */ { 0x0008, "V.110 synconous operation with HDLC framing" },
/*  4 */ { 0x0010, "T.30 modem for fax group 3" },
/*  5 */ { 0x0020, "64 kbit/s inverted with HDLC framing" },
/*  6 */ { 0x0040, "56 kbit/s bit-transparent operation" },
/*  7 */ { 0x0080, "Modem with all negotiations" },
/*  8 */ { 0x0100, "Modem asyncronous operation with start/stop byte framing" },
/*  9 */ { 0x0200, "Modem syncronous operation with HDLC framing" },
 { 0, 0 }
};
struct bittext b2support[] = {
/*  0 */ { 0x0001, "ISO 7776 (X.75 SLP)" },
/*  1 */ { 0x0002, "Transparent" },
/*  2 */ { 0x0004, "SDLC" },
/*  3 */ { 0x0008, "LAPD with Q.921 for D channel X.25 (SAPI 16)" },
/*  4 */ { 0x0010, "T.30 for fax group 3" },
/*  5 */ { 0x0020, "Point-to-Point Protocol (PPP)" },
/*  6 */ { 0x0040, "Transparent (ignoring framing errors of B1 protocol)" },
/*  7 */ { 0x0080, "Modem error correction and compression (V.42bis or MNP5)" },
/*  8 */ { 0x0100, "ISO 7776 (X.75 SLP) with V.42bis compression" },
/*  9 */ { 0x0200, "V.120 asyncronous mode" },
/* 10 */ { 0x0400, "V.120 asyncronous mode with V.42bis compression" },
/* 11 */ { 0x0800, "V.120 bit-transparent mode" },
/* 12 */ { 0x1000, "LAPD with Q.921 including free SAPI selection" },
 { 0, 0 }
};
struct bittext b3support[] = {
/*  0 */ { 0x0001, "Transparent" },
/*  1 */ { 0x0002, "T.90NL, T.70NL, T.90" },
/*  2 */ { 0x0004, "ISO 8208 (X.25 DTE-DTE)" },
/*  3 */ { 0x0008, "X.25 DCE" },
/*  4 */ { 0x0010, "T.30 for fax group 3" },
/*  5 */ { 0x0020, "T.30 for fax group 3 with extensions" },
/*  6 */ { 0x0040, "reserved" },
/*  7 */ { 0x0080, "Modem" },
 { 0, 0 }
};

struct bittext SupportedServices[] = {
/*  0 */ { 0x0001, "Hold / Retrieve" },
/*  1 */ { 0x0002, "Terminal Portability" },
/*  2 */ { 0x0004, "ECT" },
/*  3 */ { 0x0008, "3PTY" },
/*  4 */ { 0x0010, "Call Forwarding" },
/*  5 */ { 0x0020, "Call Deflection" },
/*  6 */ { 0x0040, "MCID" },
/*  7 */ { 0x0080, "CCBS" },
 { 0, 0 }
};

static void showbitvalues(struct bittext *p, __u32 value)
{
   while (p->text) {
      if (value & p->bit) printf("   %s\n", p->text);
      p++;
   }
}

int main(int argc, char **argv)
{
   struct capi_profile cprofile;
   unsigned char buf[64];
   unsigned int *vbuf;
   unsigned char *s;
   int ncontr, i;
   unsigned j;
   int isAVM;
   unsigned err, ApplId, MsgId = 1, SSInfo, SuppServices;
   _cmsg cmsg;

   if (CAPI20_ISINSTALLED() != CapiNoError) {
      fprintf(stderr, "capi not installed - %s (%d)\n", strerror(errno), errno);
      return 2;
   }

   CAPI20_GET_PROFILE(0, (CAPI_MESSAGE)&cprofile);
   ncontr = GET_WORD(&cprofile.ncontroller);
   printf("Number of Controllers : %d\n", ncontr);

   err = CAPI20_REGISTER(1, 1, 2048, &ApplId);
   if (err != CapiNoError) {
       fprintf(stderr, "could not register - %s (%#x)\n", capi_info2str(err), err);
       return 1;
   }

   for (i = 1; i <= ncontr; i++) {
       isAVM = 0;
       printf("Controller %d:\n", i);
       if (!CAPI20_GET_MANUFACTURER (i, buf)) {
           fprintf(stderr, "could not get manufacturer info for controller %d\n", i);
           return 1;
       }
       printf("Manufacturer: %s\n", buf);
       if (strstr((char *)buf, "AVM") != 0) isAVM = 1;
       if (!CAPI20_GET_VERSION (i, buf)) {
           fprintf(stderr, "could not get capi version info for controller %d\n", i);
           return 1;
       }
       vbuf = (unsigned int *)buf;
       printf("CAPI Version: %u.%u\n", GET_DWORD(&vbuf[0]), GET_DWORD(&vbuf[1]));
       if (isAVM) {
          printf("Manufacturer Version: %u.%01x%01x-%02u  (%u.%u)\n",
                  (GET_DWORD(&vbuf[2])>>4) & 0x0f,
                  ((GET_DWORD(&vbuf[2])<<4) & 0xf0),
		  ((GET_DWORD(&vbuf[3])>>4) & 0x0f),
                  (GET_DWORD(&vbuf[3]) & 0x0f),
                  GET_DWORD(&vbuf[2]), GET_DWORD(&vbuf[3]) );
       } else {
          printf("Manufacturer Version: %u.%u\n", GET_DWORD(&vbuf[2]), GET_DWORD(&vbuf[3]));
       }
       if (!CAPI20_GET_SERIAL_NUMBER (i, buf)) {
           fprintf(stderr, "could not get serial number info for controller %d\n", i);
           return 1;
       }
       printf("Serial Number: %s\n", (char *)buf);
       err = CAPI20_GET_PROFILE(i, (CAPI_MESSAGE)&cprofile);
       if (err != CapiNoError) {
           fprintf(stderr, "could not get profile info for controller %d - %s (%#x)\n", i, capi_info2str(err), err);
           return 1;
       }
       printf("BChannels: %u\n", GET_WORD(&cprofile.nbchannel));
       printf("Global Options: 0x%08x\n", GET_DWORD(&cprofile.goptions));
       showbitvalues(goptions, GET_DWORD(&cprofile.goptions));
       printf("B1 protocols support: 0x%08x\n", GET_DWORD(&cprofile.support1));
       showbitvalues(b1support, GET_DWORD(&cprofile.support1));
       printf("B2 protocols support: 0x%08x\n", GET_DWORD(&cprofile.support2));
       showbitvalues(b2support, GET_DWORD(&cprofile.support2));
       printf("B3 protocols support: 0x%08x\n", GET_DWORD(&cprofile.support3));
       showbitvalues(b3support, GET_DWORD(&cprofile.support3));
       for (j=0, s = (unsigned char *)&cprofile; j < sizeof(cprofile); j++) {
           switch (j) {
	      case 0: printf("\n  "); break;
	      case 2: printf("\n  "); break;
	      case 4: printf("\n  "); break;
	      case 8: printf("\n  "); break;
	      case 12: printf("\n  "); break;
	      case 16: printf("\n  "); break;
	      case 20: printf("\n  "); break;
	      case 44: printf("\n  "); break;
	      case 64: printf("\n  "); break;
              default: if ((j % 4) == 0) printf(" ");
	   }
           printf("%02x", s[j]);
       }

       printf("\n");

       if ( (cprofile.support1 & 0x10000000) &&
	    (cprofile.support2 & 0x40000000) &&
	    (cprofile.support3 & 0x40000000) ) {
	        printf("\n");
	        continue;
	}

       FACILITY_REQ_HEADER(&cmsg, ApplId, MsgId++, i);
       s = (unsigned char *) &cmsg.FacilitySelector;
       s[0] = 0x03;
       s[1] = 0x00;
       cmsg.FacilityRequestParameter = (unsigned char *)"\x03""\x00\x00""\x00"; /* GetSupportedServices */

       err = CAPI_PUT_CMSG(&cmsg);
       if (err != CapiNoError) {
	   fprintf(stderr, "FAC REQ - %s (%#x)\n", capi_info2str(err), err);
	   continue;
       }
       err = capi20_waitformessage(ApplId, 0);
       if (err != CapiNoError) {
	   fprintf(stderr, "FAC WAIT - %s (%#x)\n", capi_info2str(err), err);
	   continue;
       }
       err = CAPI_GET_CMSG(&cmsg, ApplId);
       if (err != CapiNoError) {
	   fprintf(stderr, "FAC GET - %s (%#x)\n", capi_info2str(err), err);
	   continue;
       }
       if (cmsg.Info != 0x0000) {
	   fprintf(stderr, "FAC GET - Info: %s (%#x)\n", capi_info2str(cmsg.Info), cmsg.Info);
	   continue;
       }
       if (cmsg.FacilityConfirmationParameter[0] != 0x09) {
	   fprintf(stderr, "FAC GET - (len)\n");
	   continue;
       }
       SSInfo = GET_WORD(&cmsg.FacilityConfirmationParameter[4]);

       SuppServices = GET_DWORD(&cmsg.FacilityConfirmationParameter[6]);
       
       printf("\nSupplementary services support: 0x%08x\n", SuppServices);
       showbitvalues(SupportedServices, SuppServices);
       printf("\n");
   }
   return 0;
}
