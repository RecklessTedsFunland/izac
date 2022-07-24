---
title: Ubuntu Install RPi3
date: 6 Oct 2019
---

As of this, ROS2 [release](https://index.ros.org/doc/ros2/Releases/)

- [Desktop Ubuntu](https://docs.ros.org/en/humble/Installation.html)
- [Raspberry Pi Instructions (Humble)](https://docs.ros.org/en/humble/How-To-Guides/Installing-on-Raspberry-Pi.html)
- [ROS 2 Varients](https://www.ros.org/reps/rep-2001.html#id22)

## Install

The arm 64-bit version has `ssh` setup by default and requires a password reset.

- Use the `Raspberry Pi Imager`
    - Select Ubuntu Server 22.04 LTS (64-bit) from Other general-purpose OS
    - From the *gear icon* set up `username`, `wifi`, and anything else

- OR get [server image](https://ubuntu.com/download/raspberry-pi)
version 22.04 LTS or later
    - username: `ubuntu`  
    - password: `ubuntu` (which you have to change on login or ssh)

- Ubuntu Linux - Jammy Jellyfish (22.04)
    - `sudo apt install ros-humble-desktop` which give GUI stuff
    - `sudo apt install ros-humble-ros-base`
    - Now do `source /opt/ros/humble/setup.zsh` to setup your `env` properly
    - [Ubuntu Instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## `python3`

```bash
sudo apt install python3-colcon-zsh python3-colcon-cmake python3-colcon-ros
```

## Setup

Unfortunately Ubuntu is setup my an idiot who doesn't use computers, especially
headless servers.

- Change hostname: `sudo hostnamectl set-hostname <new name>`
    ```
    ~$ hostnamectl
       Static hostname: ros
             Icon name: computer
            Machine ID: 32f369a042bd4d3e9f1f793aa46ad688
               Boot ID: f1856754c054460394bdba31faa79e0f
      Operating System: Ubuntu 18.04.3 LTS
                Kernel: Linux 4.15.0-1048-raspi2
          Architecture: arm64
    ```
- Avahi: `sudo apt-get install avahi-daemon`
- Remove stupid cloud init stuff:
    ```
    sudo dpkg-reconfigure cloud-init
    sudo apt-get purge cloud-init
    sudo mv /etc/cloud/ ~/; sudo mv /var/lib/cloud/ ~/cloud-lib
    ```
- Disable services that wanted that
    - First figure them out with: `sudo systemctl show -p WantedBy network-online.target`
    - Disable with: `sudo systemctl disable <service>`
        - I stopped `iscsid.service` and `open-iscsi.service` which are san service things
        I don't use
        - I kept `nmbd.service` for avahi
- samba:
```
sudo apt install samba
sudo ufw allow 'Samba'
sudo cp /etc/samba/smb.conf{,.backup}
# update smb.conf
sudo systemctl restart nmbd
```
- add user with: `sudo smbpasswd -a ubuntu`
- add user home:
```
[homes]
   comment = Home Directories
   browseable = no
   read only = no
   create mask = 0755
   directory mask = 0755
   valid users = %S
   path=/home/%S
```

### Networking

Create the following with: `sudo nano /etc/netplan/01-netcfg.yaml`

```yaml
network:
 version: 2
 renderer: networkd
 ethernets:
   eth0:
     dhcp4: yes
     dhcp6: yes
     optional: true
 wifis:
   wlan0:
     dhcp4: yes
     dhcp6: yes
     access-points:
       "your-wifi-name":
         password: "your-wifi-password"
```

Enable with `sudo netplan apply` and reboot

If there is boot delay, you can do: `systemctl mask systemd-networkd-wait-online.service`

# ROS2

- Setup locale:
    ```
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```
- Setup sources:
    ```
    sudo apt update && sudo apt install curl gnupg2 lsb-release
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
    ```
- Install ROS packages:
    ```
    sudo apt update
    sudo apt install ros-humble-ros-base
    ```

## Cross-Compiling for ARM

There are [directions](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation/)
for using docker to cross-compile for ARM.

# References

- [turtlebot3 emanual](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/#setup)
- [ROS2 install dasing on linux](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)
- [Add samba server to ubuntu](https://linuxize.com/post/how-to-install-and-configure-samba-on-ubuntu-18-04/)
- [Default `umask` permissions](https://www.computernetworkingnotes.com/linux-tutorials/how-to-change-default-umask-permission-in-linux.html)
