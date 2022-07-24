---
title: Stop Terminal Exit Warning
---

If you have a program running like `ssh` and try to close a window, it
will warn you the program is still running. To stop the stupid issue:

```
gsettings set org.gnome.Terminal.Legacy.Settings confirm-close false
```

I have no idea why this isn't easier to do, Ubuntu developers must
be stupid I guess. Every other OS is easy to change this option.
