---
title: Useful Software on Ubuntu
date: 3 Aug 2019
---

Here are some of the key steps to installing some useful engineering
software on Ubuntu.

## KiCad

https://kicad.org/download/ubuntu/

```
sudo add-apt-repository --yes ppa:kicad/kicad-5.1-releases
sudo apt update
sudo apt install --install-suggests kicad
```

## OpenSCAD

https://www.openscad.org/downloads.html

```
sudo apt update
sudo apt install openscad
```

Also developmental AppImages available from the download link above

## Chrome

```
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
echo 'deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main' | sudo tee /etc/apt/sources.list.d/google-chrome.list
sudo apt update 
sudo apt install google-chrome-stable
```

## Handbrake

```
sudo add-apt-repository ppa:stebbins/handbrake-releases
sudo apt install handbrake-gtk handbrake-cli
```

## Atom.io

https://flight-manual.atom.io/getting-started/sections/installing-atom/

```
sudo apt install software-properties-common apt-transport-https wget
wget -q https://packagecloud.io/AtomEditor/atom/gpgkey -O- | sudo apt-key add -

sudo sh -c 'echo "deb [arch=amd64] https://packagecloud.io/AtomEditor/atom/any/ any main" > /etc/apt/sources.list.d/atom.list'

sudo apt update
sudo apt install atom
```

## Node.js

```
curl -sL https://deb.nodesource.com/setup_15.x | sudo -E bash -
sudo apt-get install -y nodejs
sudo npm install -g npm
```

## FeeCAD

flatpack: https://www.freecadweb.org/downloads.php

