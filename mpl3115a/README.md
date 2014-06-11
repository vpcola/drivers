This is a quick device driver for the raspberry pi I made to experiment 
with the XTRINSIC MEMS Sensor board that contains 3 sensors - A temperature
and barometer sensor (using the MPL3115A2 chip), a 3-axis accelerometer, 
and the digital compass sensor.

I'm currently interested to get the temperature and barometric pressure
only for the moment, I would have to write the other drivers later.
So this driver is named mpl3115 from the temperature/barometer sensor
on the board.

The device driver works by handling the interrupt lines from the sensor
and reading them only when there is available data (sensor raises data
ready interrupt). 

The driver exposes 3 kernel attributes to userland in /sys. Once the
driver is loaded, it exposes the following attributes on 
/sys/bus/i2c/drivers/mpl3115/1-0060 directory:

<b>altbarmode</b> - The current mode of the driver whether it uses Barometric
    pressure (B) measurement or Altitude (A). By default the driver
    starts with Barometric pressure mode.

<b>altbarvalue</b> - The last update time from the driver (time barometer/alti-
    meter was updated) and the altimeter/barometric pressure value, 
    depending on the mode. The format of this is given in the example
    below:
    
    <hours:min:sec:micro sec>|<altimeter/barometer value>

    Example below shows how readings are taken and to switch modes:
    
```
pi@raspberrypi:~$ cat /sys/bus/i2c/drivers/mpl3115/1-0060/altbarmode
B
pi@raspberrypi:~$ cat /sys/bus/i2c/drivers/mpl3115/1-0060/altbarvalue
10:39:8:107541|6426032
pi@raspberrypi:~$ echo 'A' > /sys/bus/i2c/drivers/mpl3115/1-0060/altbarmode
pi@raspberrypi:~$ cat /sys/bus/i2c/drivers/mpl3115/1-0060/altbarvalue
10:39:49:688672|19632
pi@raspberrypi:~$
```

Notice the difference in reading altimeter mode and barometer mode above.
Also note that the value is the raw readings of the sensor, user
applications must convert this value to meters/feet/etc.

<b>tempvalue</b> - The value of the temperature sensor. This value is a 16 bit value
    containing the numbers part in the MSB, and the decimal part in the LSB.
    Since we are prohibited from using floating point in kernel code, user 
    applications must convert this value to Fahrenheit and to Celcuius 
    using the formula:

```
    double fahrenheit = 0;

    MSB = (value >> 8) & 0xFF;
    LSB = value & 0xFF;

    LSB = (LSB > 99) (LSB / 1000) : (LSB / 100);
   
    fahrenheit = MSB + LSB;
```

So for the readings below:

```
pi@raspberrypi:~$ cat /sys/bus/i2c/drivers/mpl3115/1-0060/tempvalue
10:57:2:914445|7856
pi@raspberrypi:~$
```
7856 is 0x1EB0 (MSB = 1E or 30, LSB = B0 or ), to get the reading in fahrenheit

F = 30.176 ( That's how hot Singapore is! )

If you put your finger on the MPL3115 chip, notice the temperature increase.

Right now only the last read values are taken by this driver, theres currently
no kernel FIFO to store all the values when an interrupt is triggered, what was
last ready will be reflected. In the future I will expand this driver to store
all read values from the sensor in a kernel fifo (or circular buffer) and implements
a char driver.




    
