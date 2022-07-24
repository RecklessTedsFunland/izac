---
title: Home Libraries and Binaries
---

In order to not polute my system libraries with binaries/libraries
I build from source. Life would be great if everyone always built packages
for their software, but that isn't easy.

## Setup

So I created a hidden folder `~/.local` in my home directory. In
there is the standard `/usr/local` layout of folders and files.
I added the following to my `~/.bashrc` to point programs at what
is installed there:

```
export PATH=$HOME/.local/bin:$PATH
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
```
## Building with CMake

```
cmake -DCMAKE_INSTALL_PREFIX=$ENV{HOME}/.local
```
