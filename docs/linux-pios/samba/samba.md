# Samba

## External Drive

Plug in the drive and do:

```
sudo fdisk -l

... lots of stuff
Disk /dev/sda: 1.82 TiB, 2000365289472 bytes, 3906963456 sectors
Disk model: My Passport 25E1
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes
Disklabel type: gpt
Disk identifier: 44BBE1A2-E9A0-439C-A786-8D0F1CC48E06

Device     Start        End    Sectors  Size Type
/dev/sda1   2048 3906961407 3906959360  1.8T Linux filesystem
```
So the system sees the drive, but you can also do:

```
sudo blkid /dev/sda1
/dev/sda1: LABEL="slurm" UUID="f5a3c4cf-6761-4a3e-a0b8-6acaa06cb05a" BLOCK_SIZE="4096" TYPE="ext4" PARTUUID="95ce4e5b-6862-4177-b513-7a8890ed1dd9"
```

Okay, now we need to mount the drive:

```
sudo mount -t ext4 -o defaults /dev/sda1 /mnt/usb
```

This assumes a drive formated in `ext4`. 

Add the following to `/etc/samba/smd.conf` so the drive shows up on the network:

```
[USB]
  comment = USB Drive
  path = "/mnt/usb"
  writeable = yes
  create mask = 0777
  directory mask = 0777
  force user = pi
```

Restart `samba` with: `sudo systemctl restart`.

When you are all done, do:

```
sudo umount /dev/sda1
```
