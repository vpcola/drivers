This is the third update to the original mpl3115 device driver for the 
raspberry pi. 

I'm currently interested just to get the temperature and barometric pressure
for the moment, I would have to write the other drivers later.
So this driver is named mpl3115 from the temperature/barometer sensor
on the board.

The device driver works by handling the interrupt lines from the sensor
and reading them only when there is available data (sensor raises data
ready interrupt). 

The driver exposes 4 kernel attributes to userland in /sys. Once the
driver is loaded, it exposes the following attributes on 
/sys/bus/i2c/drivers/mpl3115/1-0060 directory:

<b>steptime</b> - Sets and gets the current step time. Step time
    is expressed in 2^x seconds. The step time also dictates the interrupt 
    time interval. The smallest value being 0, which translates to 2^0 = 1
    second interval. 2 ^ 1 is 2 seconds ... and so on.

<b>altbarmode</b> - The current mode of the driver whether it uses Barometric
    pressure (B) measurement or Altitude (A). By default the driver
    starts with Barometric pressure mode.

<b>altbarvalue</b> - The last update time from the driver (time barometer/alti-
    meter was updated) and the altimeter/barometric pressure value, 
    depending on the mode. The format of this is given in the example
    below:
    
    <altimeter/barometer value> <P|M>

    Barometric pressure is expressed in pascals, while altimeter is 
    expressed in meters.

    Example below shows how readings are taken and to switch modes:
    
```
pi@raspberrypi ~ cat /sys/bus/i2c/drivers/mpl3115/1-0060/altbarvalue
50.9375 M
pi@raspberrypi ~ $ cat /sys/bus/i2c/drivers/mpl3115/1-0060/tempvalue
29.50 C
pi@raspberrypi ~ $ cat /sys/bus/i2c/drivers/mpl3115/1-0060/altbarmode
A
pi@raspberrypi ~ $ sudo cat /dev/mpl3115
29.37 C|100711.75 P
29.37 C|100711.25 P
29.37 C|100711.50 P
29.37 C|100712.25 P
29.37 C|100711.25 P
29.37 C|100712.75 P
29.37 C|100709.25 P
^Cpi@raspberrypi ~ echo "A" > /sys/bus/i2c/drivers/mpl3115/1-0060/altbarmode
pi@raspberrypi ~ $ sudo cat /dev/mpl3115
29.37 C|100711.00 P
29.37 C|100712.25 P
29.37 C|100712.00 P
29.37 C|100714.75 P
29.37 C|100709.75 P
29.37 C|100713.75 P
29.37 C|100712.50 P
29.37 C|100710.75 P
29.37 C|100714.00 P
29.37 C|100712.75 P
29.37 C|100714.75 P
29.37 C|100717.75 P
29.37 C|100714.25 P
29.37 C|51.0000 M
29.37 C|51.3750 M
29.37 C|51.0000 M
29.37 C|51.0625 M
29.37 C|51.0625 M
29.37 C|51.0625 M
29.37 C|51.1875 M
29.37 C|51.0625 M
^Cpi@raspberrypi ~ $
```

Notice the difference in reading altimeter mode and barometer mode above.

<b>tempvalue</b> - The value of the temperature sensor. Like the altermeter
    readings above, this is prefixed with the last update time.

If you put your finger on the MPL3115 chip, notice the temperature increase.

Right now only the last read values are taken by this driver, theres currently
no kernel FIFO to store all the values when an interrupt is triggered, what was
last ready will be reflected. In the future I will expand this driver to store
all read values from the sensor in a kernel fifo (or circular buffer) and implements
a char driver.

<b>FIFO Capability and Character driver</b>

This driver also adds a fifo capability in /dev/mpl3115. The driver will push the
temperature and altimeter readings to a fifo which can be read by userland
drivers. 

Do note however that the driver doesn't make corrections when the fifo is full, 
so the application in user space needs to be aware of this. The fifo must be 
emptied by continiously being read.

Example of reading the fifo below:
```
pi@raspberrypi ~ $ sudo cat /dev/mpl3115
30.00 C|49.6250 M
29.93 C|49.8750 M
29.87 C|49.9375 M
29.81 C|50.1875 M
29.81 C|49.8125 M
29.81 C|50.5625 M
30.06 C|49.4375 M
30.18 C|48.5000 M
30.18 C|48.8125 M
30.25 C|48.6250 M
30.25 C|48.7500 M
30.31 C|48.7500 M
30.25 C|49.2500 M
30.12 C|49.3125 M
30.00 C|49.7500 M
```

   Data from the fifo above reflects the last update time, 
   the temperature readings in deg. Centigrade, and the raw
   Altimeter values.
