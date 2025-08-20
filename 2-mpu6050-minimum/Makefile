obj-m += mpu6050_min.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

RESULT = mpu6050_minimum
SRC    = $(RESULT).c

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
