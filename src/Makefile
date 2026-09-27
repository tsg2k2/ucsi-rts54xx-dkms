# SPDX-License-Identifier: GPL-2.0-only
# Out-of-tree build of ucsi_ite; the kbuild side lives in Kbuild.

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

.PHONY: all clean
