---
title: How to Fix Gazebo Certificate Errors
date: 24 Sept 2019
---

You need to change the url associated with osrf:

```
$ nano /home/kevin/.ignition/fuel/config.yaml 
---
# The list of servers.
servers:
  -
    name: osrf
    url: https://api.ignitionrobotics.org

  # -
    # name: another_server
    # url: https://myserver

# Where are the assets stored in disk.
# cache:
#   path: /tmp/ignition/fuel
```

# References

- [bitbucket issue](https://bitbucket.org/osrf/gazebo/issues/2607/error-restcc-205-during-startup-gazebo)
