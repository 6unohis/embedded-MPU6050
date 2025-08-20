obj-m += mpu6050_min_drv.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

RESULT = mpu6050_min_user
SRC    = $(RESULT).c

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	$(CC) -o $(RESULT) $(SRC)
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f $(RESULT)
