---
title: Install Gnome-Software
date: 7 June 2020
---

Somehow I lost `gnome-software` when I upgraded to Ubuntu 20.04. To reinstall
it, just do:

```
# first, clean list of cached packages so Ubuntu Software can read them
sudo apt clean 
sudo apt update && sudo apt upgrade
sudo apt autoremove gnome-software && sudo apt install gnome-software gnome-software-plugin-flatpak
```

Now I have the little Ubuntu software briefcase in my apps.

## References

- askubuntu.com: [Software Doesn't Open](https://askubuntu.com/questions/1231932/ubuntu-software-doesnt-open-in-ubuntu-20-04)
