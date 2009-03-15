
# location of the configured kernel
KDIR := /usr/src/linux

# which modules to build
CONFIG_MISDN_DRV = m
CONFIG_MISDN_DSP = y
CONFIG_MISDN_HFCMULTI = y
CONFIG_MISDN_HFCMINI = y
CONFIG_MISDN_HFCUSB = y
CONFIG_MISDN_HFCPCI = y
CONFIG_MISDN_AVM_FRITZ = y

HERE = $(shell pwd)

MISDNVERSION = 1-1.0

MINCLUDES = $(HERE)/mISDN 

export MINCLUDES MISDNVERSION CONFIG_MISDN_DRV CONFIG_MISDN_HFCMULTI CONFIG_MISDN_DSP CONFIG_MISDN_HFCMINI \
	CONFIG_MISDN_HFCUSB CONFIG_MISDN_HFCPCI CONFIG_MISDN_AVM_FRITZ

all: $(HERE)/mISDN/linux
	make -C $(KDIR) M=$(HERE)/mISDN modules

$(HERE)/mISDN/linux:
	ln -s $(HERE)/mISDN_include $(HERE)/mISDN/linux

clean:
	make -C $(KDIR) M=$(HERE)/mISDN clean
	rm -f $(HERE)/mISDN/linux
	rm -f $(HERE)/mISDN/Module.symvers

