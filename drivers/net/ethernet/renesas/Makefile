#
# Makefile for the Renesas device drivers.
#

obj-$(CONFIG_SH_ETH) += sh_eth.o
obj-$(CONFIG_RAVB) += ravb.o
ravb-objs := ravb_main.o
ifeq ($(CONFIG_RAVB_PTP_1588_CLOCK),y)
	ravb-objs += ravb_ptp.o
endif
