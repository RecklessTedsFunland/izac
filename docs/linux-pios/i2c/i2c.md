# i2c

Raspberry Pi OS 64-bit

Change `/boot/config.txt` to the following and reboot.

```
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000
```

```bash
#!/bin/bash
# Print current maximum i2c rate
var="$(xxd /sys/class/i2c-adapter/i2c-1/of_node/clock-frequency | awk -F': ' '{print $2}')"
var=${var//[[:blank:].\}]/}
printf "I2C Clock Rate: %d Hz\n" 0x$var
```

```
pi@cm4:~ $ ./i2c_baudrate.sh
I2C Clock Rate: 400000 Hz
```

## See i2c Devices

```
sudo apt install i2c-tools
```

```
pi@cm4:~ $ i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```