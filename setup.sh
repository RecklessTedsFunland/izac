#!/bin/bash
# -------------------------------------------------------------------------------
# MIT License
#
# Copyright (c) 2020 Reckless Ted's Funland
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# -------------------------------------------------------------------------------

# exit on error
set -e

# USEAGE="setup.sh <HOSTNAME>"

# check if we are root
if [[ "${EUID}" -eq 0 || "${USER}" != "ubuntu" ]]; then
    echo "Please run as user ubuntu"
    exit 1
fi

if [[ $# -eq 1 ]]; then
    HOST=$1
else
    HOST="ros"
fi

echo "================================"
echo " Host: ${HOST}"
echo "================================"

ZONE="America/New_York"
# ZONE="America/Denver"

# checks to see if a command exists
# if exists ls; then echo "ls found!"; fi
exists()
{
  command -v "$1" >/dev/null 2>&1
}

echo ""
echo ">>> START <<<"
echo ""

# echo ""
# echo ">>> Executed as ROOT <<<"
# echo ""

echo ">> APT installs ========================================================"
sudo apt update
sudo apt install -y cmake pkg-config build-essential clang \
    git git-lfs \
    figlet \
    python3 python3-pip python3-dev

echo ">> Setting hostname to ================================================="
sudo hostnamectl set-hostname ${HOST}
hostnamectl

echo ">> Setting timezone ===================================================="
sudo timedatectl set-timezone America/New_York
timedatectl

echo ">> Changing MOTD ======================================================="
sudo chmod a-x /etc/update-motd.d/10-help-text
# echo "figlet `uname -n`" >> /etc/update-motd.d/00-header
# file_change "/etc/update-motd.d/00-header" "figlet" "figlet `uname -n`"
cat "/etc/update-motd.d/00-header" | grep figlet
if [[ "$?" == 1 ]]; then
    echo "start"
    echo "figlet \`uname -n\`" | sudo tee --append /etc/update-motd.d/00-header > /dev/null
fi

echo ">> Setup I2C for 400kHz ================================================"
sudo apt install -y i2c-tools

cat "/boot/firmware/usercfg.txt" | grep i2c
if [[ "$?" == 1 ]]; then
    echo "dtparam=i2c_arm=on,i2c_arm_baudrate=400000" | sudo tee --append /boot/firmware/usercfg.txt
fi


echo ">> Set local to en_US.UTF-8 ============================================"
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo ">> Setup Avahi and Samba"
echo "  - enable samba user with: sudo smbpasswd -a ubuntu"
sudo apt install -y samba avahi-daemon
sudo cp /etc/samba/smb.conf{,.backup}
# ufw allow 'Samba'  # already done?

cat "/etc/samba/smb.conf" | grep ubuntu
if [[ "$?" == 1 ]]; then
    sudo cat <<EOF >"/etc/samba/smb.conf"
[ubuntu]
   comment = Home Directories
   browseable = yes
   read only = no
   create mask = 0700
   directory mask = 0700
   valid users = %S
   path=/home/%S"
EOF
    echo "---------------------------------------------------------"
    echo "** Finish SAMBA install with: sudo smbpasswd -a ubuntu"
    echo "---------------------------------------------------------"
fi

echo ">> Setup NodeJS ========================================================"
if [[ ! -f "/etc/apt/sources.list.d/nodesource.list" ]]; then
    sudo apt purge -y nodejs
    curl -sL https://deb.nodesource.com/setup_14.x | sudo -E bash -
    sudo apt install -y nodejs
    sudo npm install npm@latest -g
fi

if [[ ! -f "/etc/systemd/system/archeyjs.service" ]]; then
    sudo npm install -g httpserver archeyjs
    ARCHEYJS=`command -v archeyjs`
    sudo touch /etc/systemd/system/archeyjs.service
    sudo cat <<EOF >/etc/systemd/system/archeyjs.service
[Service]
ExecStart=${ARCHEYJS}
Restart=always
StandardOutput=syslog
StandardError=syslog
SyslogIdentifier=archeyjs
User=pi
Group=pi
Environment=NODE_ENV=production
[Install]
WantedBy=multi-user.target
EOF
    sudo systemctl --no-pager enable archeyjs
    sudo systemctl --no-pager start archeyjs
fi

echo ">> Remove cloud crap and san disk stuff ================================"
# dpkg-reconfigure cloud-init # interactive!!
sudo apt purge -y cloud-init
# mv -f /etc/cloud/     /tmp
# mv -f /var/lib/cloud/ /tmp

sudo systemctl disable iscsid.service
sudo systemctl disable open-iscsi.service

echo ">> Remove Snapd ========================================================"
if exists snap; then
    echo "  - snap list | awk '{print \$1}' to see packages"
    echo "snap remove lxd"
    echo "snap remove core18"
    echo "snap remove snapd"
    echo "apt purge snapd"
    sudo snap list | sed "1 d" | awk '{print $1}' | while read pkg; do sudo snap remove ${pkg}; done
    sudo apt -y purge snapd
fi

echo ">> Clean up sudo stuff ================================================="
sudo chown -R ${USER}:${USER} /home/${USER}
sudo apt autoremove -y

# ==========================================================================

echo ""
echo ">>> Executed as ${USER} <<<"
echo ""

echo ">> Setup Git ============================================================"
NAME="walchko"
git config --global user.name ${NAME}
git config --global user.email ${NAME}@users.noreply.github.com
git config --global push.default simple
mkdir -p /home/${USER}/github
cd /home/${USER}/github

if [[ ! -d "/home/${USER}/github/dotfiles" ]]; then
    echo ">> Cloning dotfiles ..."
    git clone git@github.com:walchko/dotfiles.git
    ln -s dotfiles/git-pull.sh .
    mv ~/.bashrc ~/.bashrc.orig
    ln -s ~/github/dotfiles/bashrc ~/.bashrc
else
    echo ">> Github dotfiles already installed"
fi

cd ~

echo ">> Install python packages ============================================="
pip3 install -U twine numpy psutil pytest simplejson colorama pyserial picamera[array]

echo ">> Install poetry ======================================================"
if [[ ! -d "/home/${USER}/.poetry" ]]; then
    curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python3
else
    echo ">> Poetry already installed"
fi
