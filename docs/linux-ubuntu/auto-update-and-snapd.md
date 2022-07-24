---
title: Stopping the Auto Updater
date: 3 Nov 2019
---

The auto updater is a POS built by morons who can't even code. 
When you tell it to not update in the GUI application, it still
does because the programmers suck!

So here is how to kill this POS:

```
systemctl stop apt-daily.timer
systemctl disable apt-daily.timer
systemctl disable apt-daily.service
systemctl stop apt-daily-upgrade.timer
systemctl disable apt-daily-upgrade.timer
systemctl disable apt-daily-upgrade.service
```

## `snapd`

```
kevin@olubuntu2010:~$ snap list
Name               Version             Rev   Tracking         Publisher   Notes
core18             20200724            1885  latest/stable    canonical✓  base
gnome-3-34-1804    0+git.3556cb3       60    latest/stable/…  canonical✓  -
gtk-common-themes  0.1-36-gc75f853     1506  latest/stable/…  canonical✓  -
snap-store         3.36.0-82-g80486d0  481   latest/stable/…  canonical✓  -
snapd              2.47.1              9721  latest/stable    canonical✓  snapd
```

```
sudo snap remove snap-store
sudo snap remove gtk-common-themes
sudo snap remove gnome-3-34-1804
sudo snap remove core18
sudo snap remove snapd
```

```
sudo apt purge snapd
```

- [reference](https://www.kevin-custer.com/blog/disabling-snaps-in-ubuntu-20-04/)
