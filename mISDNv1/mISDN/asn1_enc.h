/* $Id: asn1_enc.h,v 1.1 2006/03/06 12:52:07 keil Exp $
 *
 */

#include "asn1.h"

int encodeNull(__u8 *dest);
int encodeBoolean(__u8 *dest, __u32 i);
int encodeInt(__u8 *dest, __u32 i);
int encodeEnum(__u8 *dest, __u32 i);
int encodeNumberDigits(__u8 *dest, __u8 *nd, __u8 len);
int encodePublicPartyNumber(__u8 *dest, __u8 *facilityPartyNumber);
int encodePartyNumber(__u8 *dest, __u8 *facilityPartyNumber);
int encodeServedUserNumber(__u8 *dest, __u8 *servedUserNumber);
int encodeAddress(__u8 *dest, __u8 *facilityPartyNumber,
    __u8 *calledPartySubaddress);
int encodeActivationDiversion(__u8 *dest, struct FacReqCFActivate *CFActivate);
int encodeDeactivationDiversion(__u8 *dest,
    struct FacReqCFDeactivate *CFDeactivate);
int encodeInterrogationDiversion(__u8 *dest,
    struct FacReqCFInterrogateParameters *params);
int encodeInvokeDeflection(__u8 *dest, struct FacReqCDeflection *CD);
