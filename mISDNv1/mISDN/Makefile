# Makefile for the modular ISDN driver
#
EXTRA_CFLAGS += -ggdb
#

ifdef MINCLUDES
EXTRA_CFLAGS += -I$(MINCLUDES) -g
endif
ifdef CONFIG_MISDN_MEMDEBUG
	EXTRA_CFLAGS += -DMISDN_MEMDEBUG
endif

ifdef CONFIG_MISDN_NETDEV
	EXTRA_CFLAGS += -DCONFIG_MISDN_NETDEV 
endif

EXTRA_CFLAGS += -DMISDNVERSION=\"$(MISDNVERSION)\"

obj-$(CONFIG_MISDN_DRV) += mISDN_core.o
obj-$(CONFIG_MISDN_DRV) += mISDN_isac.o
obj-$(CONFIG_MISDN_DRV) += mISDN_l1.o
obj-$(CONFIG_MISDN_DRV) += mISDN_l2.o
obj-$(CONFIG_MISDN_DRV) += mISDN_l2_net.o
obj-$(CONFIG_MISDN_DRV) += mISDN_x25dte.o
obj-$(CONFIG_MISDN_DRV) += l3udss1.o
obj-$(CONFIG_MISDN_DRV) += l3ndss1.o
obj-$(CONFIG_MISDN_DRV) += mISDN_capi.o
obj-$(CONFIG_MISDN_DRV) += mISDN_dtmf.o

ifdef CONFIG_MISDN_AVM_FRITZ
obj-$(CONFIG_MISDN_DRV) += avmfritz.o
endif

ifdef CONFIG_MISDN_NETJET
obj-$(CONFIG_MISDN_DRV) += netjetpci.o
endif

ifdef CONFIG_MISDN_HFCPCI
obj-$(CONFIG_MISDN_DRV) += hfcpci.o
endif

ifdef CONFIG_MISDN_HFCUSB
obj-$(CONFIG_MISDN_DRV) += hfcsusb.o
endif

ifdef CONFIG_MISDN_HFCMINI
obj-$(CONFIG_MISDN_DRV) += hfcsmini.o
endif

ifdef CONFIG_MISDN_SPEEDFAX
obj-$(CONFIG_MISDN_DRV) += sedlfax.o
# obj-$(CONFIG_MISDN_DRV) += faxl3.o
endif

ifdef CONFIG_MISDN_W6692
obj-$(CONFIG_MISDN_DRV) += w6692pci.o
endif

ifdef CONFIG_MISDN_HFCMULTI
obj-$(CONFIG_MISDN_DRV) += hfcmulti.o
endif

ifdef CONFIG_MISDN_XHFC
obj-$(CONFIG_MISDN_DRV) += xhfc.o
endif

ifdef CONFIG_MISDN_DSP
obj-$(CONFIG_MISDN_DRV) += mISDN_dsp.o mISDN_dsp_mec2.o mISDN_dsp_kb1ec.o mISDN_dsp_mg2ec.o mISDN_dsp_oslec.o

endif

ifdef CONFIG_MISDN_L1OIP
obj-$(CONFIG_MISDN_DRV) += l1oip.o
endif

ifdef CONFIG_I4L_CAPI_LAYER
obj-$(CONFIG_MISDN_DRV) += I4LmISDN.o
endif

# multi objects

sedlfax-objs := sedl_fax.o isar.o
avmfritz-objs := avm_fritz.o
netjetpci-objs := netjet.o
hfcpci-objs := hfc_pci.o
hfcsusb-objs := hfcs_usb.o
hfcsmini-objs := hfcs_mini.o
w6692pci-objs := w6692.o
hfcmulti-objs := hfc_multi.o
xhfc-objs := xhfc_su.o xhfc_pci2pi.o
mISDN_isac-objs := isac.o arcofi.o
mISDN_core-objs := core.o stack.o udevice.o helper.o debug.o fsm.o \
			channel.o l3helper.o \
			sysfs_obj.o sysfs_inst.o sysfs_st.o 

ifdef CONFIG_MISDN_NETDEV			
mISDN_core-objs += netdev.o
endif

ifdef CONFIG_MISDN_MEMDEBUG
mISDN_core-objs += memdbg.o
endif

ifdef CONFIG_MISDN_DEBUGTOOL
obj-$(CONFIG_MISDN_DEBUGTOOL) += mISDN_debugtool.o
endif

mISDN_l1-objs := layer1.o
mISDN_l2-objs := layer2.o tei.o
mISDN_l2_net-objs := layer2_net.o tei_net.o
l3udss1-objs := layer3.o l3_udss1.o
l3ndss1-objs := layer3_net.o l3_ndss1.o
mISDN_capi-objs := capi.o contr.o listen.o appl.o plci.o app_plci.o ncci.o asn1.o \
                   asn1_aoc.o asn1_comp.o asn1_generic.o asn1_diversion.o \
                   asn1_basic_service.o asn1_address.o asn1_enc.o capi_enc.o \
                   supp_serv.o
mISDN_dtmf-objs := dtmf.o
mISDN_dsp-objs := dsp_core.o dsp_cmx.o dsp_tones.o dsp_dtmf.o dsp_audio.o dsp_blowfish.o dsp_pipeline.o dsp_hwec.o
mISDN_dsp_mec2-objs := dsp_mec2.o
mISDN_dsp_kb1ec-objs := dsp_kb1ec.o
mISDN_dsp_mg2ec-objs := dsp_mg2ec.o
mISDN_dsp_oslec-objs := dsp_oslec.o oslec_wrap.o oslec_echo.o
mISDN_dsp_octwareec-objs := dsp_octwareec.o
l1oip-objs := l1oip_core.o l1oip_codec.o
mISDN_x25dte-objs := x25_dte.o x25_l3.o
I4LmISDN-objs := i4l_mISDN.o
mISDN_debugtool-objs := debugtool.o

