
### Debugging if Permission Denied

```
2025/04/15 21:26:14 Failed to open serial port: failed to open serial port: Permission denied
exit status 1
```

```
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

Add your user to the dialout group:

```
sudo usermod -a -G dialout $USER
```

Log out and log back in for the group changes to take effect, or run:

```
newgrp dialout
```

Verify your user is now part of the dialout group:

```
groups
```

Check the permissions of the serial port:

```
sudo ls -la /dev/ttyUSB0
```

RJ-45

- 2 - Organge Ground - 2
- 3 - green-white stripe = RXD+
- 4 - Blue - TXD- 
- 5 - Blue/White - TXD+
- 6 - Green - RXD-
- 7 - Brown/White - GND 