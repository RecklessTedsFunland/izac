![](izac.png)

# Setting Up Ubuntu on RPi

## `git`

To prevent source from detecting `0644` files changed to `0755`:

```
umask 022
git-clone='git clone --config core.filemode=false '
```

Local `.git/config` needs to have `filemode = false` so permission changes
are not tracked. `git` by default doesn't track file modes.

## other

```bash
echo ">> APT installs ========================================================"
sudo apt update
sudo apt install -y cmake pkg-config build-essential clang \
    git git-lfs \
    figlet
```

```bash
echo ">> Setting hostname to ================================================="
sudo hostnamectl set-hostname ${HOST}
hostnamectl

echo ">> Setting timezone ===================================================="
sudo timedatectl set-timezone America/New_York
timedatectl
```

```bash
echo ">> Changing MOTD ======================================================="
sudo chmod a-x /etc/update-motd.d/10-help-text
# echo "figlet `uname -n`" >> /etc/update-motd.d/00-header
# file_change "/etc/update-motd.d/00-header" "figlet" "figlet `uname -n`"
cat "/etc/update-motd.d/00-header" | grep figlet
if [[ "$?" == 1 ]]; then
    echo "start"
    echo "\"$(figlet \`uname -n\`)\"" | sudo tee --append /etc/update-motd.d/00-header > /dev/null
fi
```

```bash
echo ">> Setup I2C for 400kHz ================================================"
sudo apt install -y i2c-tools

cat "/boot/firmware/usercfg.txt" | grep i2c
if [[ "$?" == 1 ]]; then
    echo "dtparam=i2c_arm=on,i2c_arm_baudrate=400000" | sudo tee --append /boot/firmware/usercfg.txt
fi
```

```bash
echo ">> Remove cloud crap and san disk stuff ================================"
# dpkg-reconfigure cloud-init # interactive!!
sudo apt purge -y cloud-init
# mv -f /etc/cloud/     /tmp
# mv -f /var/lib/cloud/ /tmp

sudo systemctl disable iscsid.service
sudo systemctl disable open-iscsi.service

echo ">> Remove Snapd ========================================================"

exists()
{
  command -v "$1" >/dev/null 2>&1
}

if exists snap; then
    echo "  - snap list | awk '{print \$1}' to see packages"
    echo "snap remove lxd"
    echo "snap remove core20"
    echo "snap remove snapd"
    echo "apt purge snapd"
    # sudo snap list | sed "1 d" | awk '{print $1}' | while read pkg; do sudo snap remove ${pkg}; done
    sudo snap remove lxd
    sudo snap remove core20
    sudo snap remove snapd
    sudo apt -y purge snapd
fi
```

```bash
echo ">> Install python packages ============================================="
pip3 install -U twine numpy psutil pytest simplejson colorama pyserial picamera[array] poetry
```

## Script to Automate

**WARNING:** This is still under development

```bash
curl -sSL https://raw.githubusercontent.com/RecklessTedsFunland/izac/master/setup.sh | bash -
```

## References

- linuxconfig: [Modifying MOTD on Ubuntu 20.04](https://linuxconfig.org/disable-dynamic-motd-and-news-on-ubuntu-20-04-focal-fossa-linux)

# MIT License

**Copyright (c) 2020 Reckless Ted's Funland**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
