This sample uses interrupt from the gpio pin 23 on the raspberry pi headers.

When triggered, the interrupt service routine updates that status on the leds
out on the i2c bus. Basically it will just increment the value of the data
sent out on the led. The attributes chip_led and chip_switch provides an easy
way of accessing the leds.

Note that before you can load this driver, the module must first be identified
by the board setup routine. Details on how this was done is described in my
blog at : 

http://lightsurge2.blogspot.sg/2014/05/writing-linux-kernel-device-driver-for.html

This experiment is a learning experience on how to handle device interrupts
that communicates with hardware on Linux.
