---
title: Ubuntu Bluetooth and iMac
date: 5 Apr 2020
---

Unfortunately 19.10 and my iMac don't do great with 
bluetooth. Every now and then my touchpad disconnects
and I have to shut it off and re-attch it.

I you shutoff bluetooth (you somehow go straight into 
Airplane mode) and it won't come back on. Try:

1. `sudo modprobe -r btusb`
1. `sudo modprobe btusb`

Might also have to do:

1. `bluetoothctl`
1. `scan on`
1. `exit`
