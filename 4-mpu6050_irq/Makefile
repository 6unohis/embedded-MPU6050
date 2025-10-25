# Makefile
KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

obj-m += mpu6050_irq_drv.o

ccflags-y += -Wall -Wextra

all: module user

module:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

user: user_2

user_2: mpu6050_irq_usr.c mpu6050_irq.h
	$(CC) -O2 -Wall -Wextra -o $@ mpu6050_irq_usr.c

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	$(RM) user_2
