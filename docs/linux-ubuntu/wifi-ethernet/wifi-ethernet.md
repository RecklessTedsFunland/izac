# WiFi and Ethernet Setup Ubuntu

## Networking

```yaml
network:
    version: 2
    ethernets:
         eth0:
             dhcp4: true
             optional: true
    wifis:
        renderer: networkd
        wlan0:
            access-points:
                hoth:
                    password: 6523d5a141e6abfe852e23
            dhcp4: true
            optional: true
```

## I don't know what this is

```
apt purge network-manager-config-connectivity-ubuntu
```
