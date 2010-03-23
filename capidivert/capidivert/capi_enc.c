int capiEncodeWord(__u8 *p, __u16 i)
{
	*p++ = i;
	*p++ = i >> 8;
	return 2;
}

int capiEncodeDWord(__u8 *p, __u32 i)
{
	*p++ = i;
	*p++ = i >> 8;
	*p++ = i >> 16;
	*p++ = i >> 24;
	return 4;
}

int capiEncodeServedUserNumber(__u8 *dest, __u8 *ServedUserNumber)
{
	__u8 *p;

	p = &dest[1];

	*p++ = 0x01;
	*p++ = 0xc1;
	*p++ = 0x00;
	strcpy(p, ServedUserNumber); p += strlen(ServedUserNumber);

	dest[0] = p - &dest[1];
	return p - dest;
}

int capiEncodeDivertedToNumber(__u8 *dest, __u8 *DivertedToNumber)
{
	__u8 *p;

	p = &dest[1];

	*p++ = 0x01;
	*p++ = 0x80;
	*p++ = 0x00;
	strcpy(p, DivertedToNumber); p += strlen(DivertedToNumber);

	dest[0] = p - &dest[1];
	return p - dest;
}

int capiEncodeFacReqCFact(__u8 *dest, __u16 Procedure, __u16 BasicService,
			  __u8 *ServedUserNumber, __u8 *DivertedToNumber)
{
	__u8 *p;

	p = &dest[1];
	p += capiEncodeDWord(p, 0x12345678);
	p += capiEncodeWord(p, Procedure);
	p += capiEncodeWord(p, BasicService);
	p += capiEncodeServedUserNumber(p, ServedUserNumber);
	p += capiEncodeDivertedToNumber(p, DivertedToNumber);
	*p++ = 0x00; // Subaddress
	dest[0] = p - &dest[1];
	return p - dest;
}

int capiEncodeFacReqCFdeact(__u8 *dest, __u16 Procedure, __u16 BasicService,
			    __u8 *ServedUserNumber)
{
	__u8 *p;

	p = &dest[1];
	p += capiEncodeDWord(p, 0x12345678);
	p += capiEncodeWord(p, Procedure);
	p += capiEncodeWord(p, BasicService);
	p += capiEncodeServedUserNumber(p, ServedUserNumber);
	dest[0] = p - &dest[1];
	return p - dest;
}

int capiEncodeFacReqCFinterParameters(__u8 *dest, __u16 Procedure, __u16 BasicService,
				      __u8 *ServedUserNumber)
{
	__u8 *p;

	p = &dest[1];
	p += capiEncodeDWord(p, 0x12345678);
	p += capiEncodeWord(p, Procedure);
	p += capiEncodeWord(p, BasicService);
	p += capiEncodeServedUserNumber(p, ServedUserNumber);
	dest[0] = p - &dest[1];
	return p - dest;
}

int capiEncodeFacReqCFinterNumbers(__u8 *dest)
{
	__u8 *p;

	p = &dest[1];
	p += capiEncodeDWord(p, 0x12345678);
	dest[0] = p - &dest[1];
	return p - dest;
}

int capiEncodeFacReqListen(__u8 *dest, __u32 NotificationMask)
{
	__u8 *p;

	p = &dest[1];
	p += capiEncodeDWord(p, NotificationMask);
	dest[0] = p - &dest[1];
	return p - dest;
}

