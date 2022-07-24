---
title: Disabling the Ubuntu Auto-Update System
date: 6 Feb 2021
image: "https://images.pexels.com/photos/171198/pexels-photo-171198.jpeg?auto=compress&cs=tinysrgb&h=750&w=1260"
---

Just lobotomize the system

```
sudo mv /usr/bin/gnome-software ~/
sudo mv /usr/bin/update-manager ~/ 
sudo mv /usr/bin/update-notifier ~/ 
```



# None of this works from here down

Disable auto updates from GUI desktop

![](https://linuxconfig.org/images/03-disable-automatic-updates-on-ubuntu-20-04-focal-fossa-linux.png)

Edit `/etc/apt/apt.conf.d/20auto-upgrades` to:

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Download-Upgradeable-Packages "0";
APT::Periodic::AutocleanInterval "0";
APT::Periodic::Unattended-Upgrade "1";
```

# References

- [Disable Automatic Updates on Ubuntu 20.04 Focal Fossa Linux](https://linuxconfig.org/disable-automatic-updates-on-ubuntu-20-04-focal-fossa-linux)
