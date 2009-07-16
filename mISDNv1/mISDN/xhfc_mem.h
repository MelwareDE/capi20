
#define PCI2PI_MAX_XHFC 8
extern __u32 PCI2PI_XHFC_OFFSETS[PCI2PI_MAX_XHFC];

/*
 * defines which address bit is hooked through to chip to select
 * between Address/Data access
 */

/*
 * FIXME  currently uses A0, moving hardware to use A2 will allow the
 * bus interface unit to eprform 4 x byte accesses for the 32-bit mode
 * in hardware, as the PCI interface does
 */


#define ADDR_ALINE_VAL 1


/*
functions for non multiplexed access
*/

static inline __u8
read_xhfc(xhfc_t * xhfc, __u8 reg_addr)
{
	*((volatile __u8 *)(PCI2PI_XHFC_OFFSETS[xhfc->chipidx] +
	    ADDR_ALINE_VAL)) = reg_addr;
	return (*(volatile __u8 *) (PCI2PI_XHFC_OFFSETS[xhfc->chipidx]));
}


static inline __u32
read32_xhfc(xhfc_t * xhfc, __u8 reg_addr)
{
	return
	    read_xhfc(xhfc, reg_addr) |
	    (read_xhfc(xhfc, reg_addr) << 8) |
	    (read_xhfc(xhfc, reg_addr) << 16) |
	    (read_xhfc(xhfc, reg_addr) << 24);
}

static inline void
write_xhfc(xhfc_t * xhfc, __u8 reg_addr, __u8 value)
{
	*((volatile __u8 *)(PCI2PI_XHFC_OFFSETS[xhfc->chipidx] +
	    ADDR_ALINE_VAL)) = reg_addr;
	*((volatile __u8 *)(PCI2PI_XHFC_OFFSETS[xhfc->chipidx])) = value;
}


static inline void
write32_xhfc(xhfc_t * xhfc, __u8 reg_addr, __u32 value)
{
	write_xhfc(xhfc, reg_addr, value & 0xff );
	write_xhfc(xhfc, reg_addr+1, (value >> 8) & 0xff );
	write_xhfc(xhfc, reg_addr+2, (value >> 16) & 0xff );
	write_xhfc(xhfc, reg_addr+3, (value >> 24) & 0xff );
}

