---
title: Setting Swap Space on Raspberry Pi for Package Building
date: 8 Oct 2019
---

Setting up a swapfile on your SD card has some issues:

- it's slow as dead monkeys floating in molasses
- SD cards have slow read/write and limited number of writes, so you are 
losing lifetime with this option

But, I have run up against memory issues compiling `colcon` packages (the
freaking `examples_rclcpp_minimal` which should be simple), so you may have
to do this and accept slowness/lifetime issues.

Steps:

1. `sudo fallocate -l 4G /swapfile`
1. `sudo chmod 600 /swapfile`
1. `sudo mkswap /swapfile`
1. `sudo swapon /swapfile`

Now to double check everything worked:

```
$ sudo swapon --show
NAME      TYPE SIZE  USED PRIO
/swapfile file   4G 48.5M   -2
```

## Make Permanent

Add to `/etc/fstab`: `/swapfile swap swap defaults 0 0`

# References

- [How To Add Swap Space on Ubuntu 18.04](https://linuxize.com/post/how-to-add-swap-space-on-ubuntu-18-04/)
- [stackexchange comment about "slow as dead monkeys"](https://raspberrypi.stackexchange.com/questions/70/how-to-set-up-swap-space)
