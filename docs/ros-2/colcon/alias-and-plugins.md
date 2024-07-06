# Aliases

Added some aliases to make life simplier:

```bash
alias colcon-build='colcon build --executor sequential'
alias colcon-build-pkg='colcon build --executor sequential --packages-select'
alias refresh='. install/setup.bash'
```

> **WARNING:** Raspberry Pi 4 kept crashing when building a lot of packaages
> in parallel. Fixed it by doing one package at a time with arg:
> `--executor sequential`

```bash
colcon-build
refresh
colcon-build-pkg my_cool_pkg
```

## Colcon Clean

This is a plugin that gives you `make clean` like abilities ... should be included.

```bash
sudo apt install python3-colcon-clean
```
