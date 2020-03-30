obj-m := main.o
PWD=$(shell pwd)

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	rm -rf *.o *mod.c *.symvers *.order
