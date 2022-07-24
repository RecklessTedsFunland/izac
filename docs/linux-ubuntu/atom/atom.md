---
title: Install Atom.io
date: 16 Mar 2021
image: "atom.webp"
---


Update the packages list, install the dependencies, import the 
repository GPG key, enable the Apt repository and install:

```
sudo apt update
sudo apt install software-properties-common apt-transport-https wget
wget -q https://packagecloud.io/AtomEditor/atom/gpgkey -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packagecloud.io/AtomEditor/atom/any/ any main"
sudo apt install atom
```

- [referemce](https://linuxize.com/post/how-to-install-atom-text-editor-on-ubuntu-20-04/)
