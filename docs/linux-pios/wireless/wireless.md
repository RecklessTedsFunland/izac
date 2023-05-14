# WiFi and Bluetooth

Here both are off

```
~ $ sudo rfkill list
0: phy0: Wireless LAN
	Soft blocked: yes
	Hard blocked: no
1: hci0: Bluetooth
	Soft blocked: yes
	Hard blocked: no
```

Turn them on/off with `unblock`/`block`:

- `sudo rfkill block wifi`
- `rfkill unblock wifi`
- `rfkill block bluetooth`
- `rfkill unblock bluetooth`
