# Uninstalling

Following the instructions [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#uninstall), all you need to do is:

```bash
sudo apt remove ~nros-humble-* && sudo apt autoremove
```

```bash
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt autoremove
# Consider upgrading for packages previously shadowed.
sudo apt upgrade
```
