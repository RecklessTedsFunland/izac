# `raspi-config`

Ubuntu mounts the `boot` directory differently than PiOS, so find the
current `/boot/firmware` location and remount it to `boot` so
`raspi-config` works.

```bash
$ df -h
Filesystem      Size  Used Avail Use% Mounted on
tmpfs           180M  3.1M  177M   2% /run
/dev/mmcblk0p2  7.1G  2.6G  4.2G  38% /
tmpfs           898M     0  898M   0% /dev/shm
tmpfs           5.0M     0  5.0M   0% /run/lock
/dev/mmcblk0p1  253M   74M  179M  30% /boot/firmware <== This one
tmpfs           180M  4.0K  180M   1% /run/user/1000

$ sudo mount /dev/mmcblk0p1 /boot
```

```bash
sudo apt install raspi-config
sudo raspi-config
```
