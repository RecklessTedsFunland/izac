---
title: Dropbox Install on Ubuntu
date: 27 July 2020
---

## Package Manager

Install the appropriate package if you want to use Dropbox on your Linux Desktop.

		Ubuntu 14.04 or higher (.deb)	 	64-bit	 	32-bit

Note: These packages install an open-source helper application. The version of this 
application does not change as frequently as the main Dropbox application. These 
packages will always install the latest version of Dropbox for Linux.

## Dropbox Headless Install via command line

The Dropbox daemon works fine on all 32-bit and 64-bit Linux servers. To install, 
run the following command in your Linux terminal.

- 32-bit: `cd ~ && wget -O - "https://www.dropbox.com/download?plat=lnx.x86" | tar xzf -`
- 64-bit: `cd ~ && wget -O - "https://www.dropbox.com/download?plat=lnx.x86_64" | tar xzf -`

Next, run the Dropbox daemon from the newly created `.dropbox-dist` folder: `~/.dropbox-dist/dropboxd`

## Systemd

You can setup dropbox to run automatically with `systemd` by using [this](https://github.com/joeroback/dropbox) service file.

```
# this is the dropbox@.service file

[Unit]
Description=Dropbox as a system service
After=local-fs.target network.target

[Service]
Type=simple
ExecStart=/usr/bin/env "/home/%i/.dropbox-dist/dropboxd"
Restart=on-failure
RestartSec=1
User=%i

[Install]
WantedBy=multi-user.target
```

From the `README.md`, it says:

---

This is a systemd service file to start up dropbox on a per user basis on boot.

This assumes each user installed dropbox headless via the command line: https://www.dropbox.com/en/install?os=lnx and has setup his/her account such that running dropboxd manually works.

* Place the dropbox@.service file into /etc/systemd/system
* Reload the daemons: systemctl daemon-reload
* Enable the service for users: systemctl enable dropbox@username
* Start service: systemctl start dropbox@username

---

## References

- dropbox: [Install](https://www.dropbox.com/install)
- linuxbabe: [How to Install Dropbox on Ubuntu 18.04 From Official Repository](https://www.linuxbabe.com/ubuntu/install-dropbox-ubuntu-18-04)
- linoxide: [How to Install Dropbox on Ubuntu 18.04 (Terminal/GUI)](https://linoxide.com/linux-how-to/install-dropbox-ubuntu/#:~:text=Install%20Dropbox%20On%20Ubuntu%20Desktop,select%20Open%20With%20Software%20Install.)
- github: [dropbox systemd service setup](https://github.com/joeroback/dropbox)
