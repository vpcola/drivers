This sample uses interrupt from the gpio pin 23 on the raspberry pi headers.

When triggered, the interrupt service routine updates that status on the leds
out on the i2c bus. Basically it will just increment the value of the data
sent out on the led. The attributes chip_led and chip_switch provides an easy
way of accessing the leds.

This experiment is a learning experience on how to handle device interrupts
that communicates with hardware on Linux.
