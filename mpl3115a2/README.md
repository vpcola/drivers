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

<b>tempvalue</b> - The value of the temperature sensor. Like the altermeter
    readings above, this is prefixed with the last update time.

If you put your finger on the MPL3115 chip, notice the temperature increase.

Right now only the last read values are taken by this driver, theres currently
no kernel FIFO to store all the values when an interrupt is triggered, what was
last ready will be reflected. In the future I will expand this driver to store
all read values from the sensor in a kernel fifo (or circular buffer) and implements
a char driver.

<b>FIFO Capability</b>

This driver also adds a fifo capability in /proc. The driver will push the
temperature and altimeter readings to a fifo in /proc/mpl3115_fifo. Do note however
that the driver doesn't make corrections when the fifo is full, so the application
in user space needs to be aware of this. The fifo must be emptied by continiously
being read.

Example of reading the fifo below:
```
pi@raspberrypi:/proc$ cat mpl3115_fifo
08:00:49:713592|30.68|6426704
08:00:50:681850|30.62|6426368
08:00:51:650218|30.68|6426560
08:00:52:618559|30.68|6426720
08:00:53:586796|30.68|6426576
08:00:54:555169|30.68|6426576
08:00:55:523500|30.68|6426320
08:00:56:491792|30.68|6426320
08:00:57:460101|30.68|6426432
08:00:58:428426|30.68|6426528
08:00:59:396771|30.68|6426384
08:01:00:365137|30.68|6426624
08:01:01:333447|30.68|6426848
08:01:02:301896|30.68|6426288
08:01:03:270228|30.68|6426496
08:01:04:238576|30.68|6426320
08:01:05:206866|30.68|6426656
08pi@raspberrypi:/proc$ cat mpl3115_fifo
08:31:19:840098|30.75|6424304
08:31:20:808452|30.75|6424160
pi@raspberrypi:/proc$ cat mpl3115_fifo
08:31:21:776680|30.75|6424128
08:31:22:745041|30.75|6424128
08:31:23:713354|30.75|6424256
08:31:24:681684|30.75|6424112
08:31:25:650005|30.75|6424240
```

   Data from the fifo above reflects the last update time, 
   the temperature readings in deg. Centigrade, and the raw
   Altimeter values.
