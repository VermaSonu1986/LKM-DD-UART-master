obj-m+=Device_Driver_UART.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
	#make COPTS=-g
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean

