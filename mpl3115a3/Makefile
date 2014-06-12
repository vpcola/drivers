obj-m += mpl3115.o
mpl3115-objs := bartemp.o mpl3115a.o

KDIR = /opt/cross/raspberry/linux

PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean

