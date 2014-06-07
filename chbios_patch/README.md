This is patch file to get the I2C bus 1 working
on ChibiOS/RT.

Download ChibiOS/RT for the raspberry pi at 

git clone https://github.com/steve-bate/ChibiOS-RPi

Once downloaded, go to the ChibiOS-Rpi directory and
apply the patch:

patch -p0 < patchfile

You should now be able to use I2C bus 1 on the 
Raspberry Pi with the ChibiOS/RT (RTOS).
