---
title: ROS2 and Multicast Testing
date: 6 Oct 2019
image: "https://i.pinimg.com/564x/40/2a/f7/402af7419092815df8164080f54a9505.jpg"
image-height: "300px"
---

Ros nodes use multicast to find each other. If you have problems, you can test
connectivity with `iperf`. Although other addresses worked on linux-to-linux (i.e.,
244.1.1.1), macOS had issues. The address `224.0.0.1` worked the best for both.

- macOS: `brew install iperf`
- linux: `sudo apt install iperf`

**note:** firewalls on macos and linux (ufw) didn't seem to be an issue.

## Client (send)

- u: udp
- p: port
- i: time between reports to stdout
- c: client address
- T: time to live (TTL)

```
kevin@Logan multicast $ iperf -c 224.0.0.1 -p 11311 -u -T 2  -i 1
------------------------------------------------------------
Client connecting to 224.0.0.1, UDP port 11311
Sending 1470 byte datagrams, IPG target: 11215.21 us (kalman adjust)
Setting multicast TTL to 2
UDP buffer size: 9.00 KByte (default)
------------------------------------------------------------
[  4] local 10.0.1.143 port 58107 connected with 224.0.0.1 port 11311
[ ID] Interval       Transfer     Bandwidth
[  4]  0.0- 1.0 sec   131 KBytes  1.07 Mbits/sec
[  4]  1.0- 2.0 sec   128 KBytes  1.05 Mbits/sec
[  4]  2.0- 3.0 sec   128 KBytes  1.05 Mbits/sec
[  4]  3.0- 4.0 sec   128 KBytes  1.05 Mbits/sec
[  4]  4.0- 5.0 sec   128 KBytes  1.05 Mbits/sec
[  4]  5.0- 6.0 sec   128 KBytes  1.05 Mbits/sec
[  4]  6.0- 7.0 sec   129 KBytes  1.06 Mbits/sec
[  4]  7.0- 8.0 sec   128 KBytes  1.05 Mbits/sec
[  4]  8.0- 9.0 sec   128 KBytes  1.05 Mbits/sec
[  4]  9.0-10.0 sec   128 KBytes  1.05 Mbits/sec
[  4]  0.0-10.0 sec  1.25 MBytes  1.05 Mbits/sec
[  4] Sent 893 datagrams
```

## Server (receive)

- s: server
- B: bind address
- u: udp
- p: port
- i: time between reports to stdout

```
kevin@Logan multicast $ iperf -s -u -B 224.0.0.1 -p 11311 -i 1
------------------------------------------------------------
Server listening on UDP port 11311
Binding to local address 224.0.0.1
Joining multicast group  224.0.0.1
Receiving 1470 byte datagrams
UDP buffer size:  768 KByte (default)
------------------------------------------------------------
[  4] local 224.0.0.1 port 11311 connected with 10.0.1.19 port 57429
[ ID] Interval       Transfer     Bandwidth        Jitter   Lost/Total Datagrams
[  4]  0.0- 1.0 sec  54.6 KBytes   447 Kbits/sec  32.696 ms  184/  222 (83%)
[  4]  1.0- 2.0 sec  14.4 KBytes   118 Kbits/sec  30.254 ms  101/  111 (91%)
[  4]  2.0- 3.0 sec  14.4 KBytes   118 Kbits/sec  41.592 ms  124/  134 (93%)
[  4]  3.0- 4.0 sec  14.4 KBytes   118 Kbits/sec  57.411 ms  145/  155 (94%)
[  4]  4.0- 5.0 sec  12.9 KBytes   106 Kbits/sec  78.279 ms   97/  106 (92%)
[  4]  5.0- 6.0 sec  14.4 KBytes   118 Kbits/sec  72.326 ms   58/   68 (85%)
[  4]  6.0- 7.0 sec  11.5 KBytes  94.1 Kbits/sec  53.250 ms   63/   71 (89%)
[  4]  0.0- 7.4 sec   142 KBytes   157 Kbits/sec  49.576 ms  794/  893 (89%)
```

# Simple Python

Here is a horrible little python script to test with:

```python
#!/usr/bin/env python

import sys
import socket
import struct

DEFAULT_GROUP = '224.0.0.1'
DEFAULT_PORT = 49150


def send(data, *, group=DEFAULT_GROUP, port=DEFAULT_PORT, ttl=None):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    if ttl is not None:
        packed_ttl = struct.pack('b', ttl)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, packed_ttl)
    try:
        s.sendto(data, (group, port))
    finally:
        s.close()


def receive(*, group=DEFAULT_GROUP, port=DEFAULT_PORT, timeout=None):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            # not available on Windows
            pass
        s.bind(('', port))

        s.settimeout(timeout)

        mreq = struct.pack('4sl', socket.inet_aton(group), socket.INADDR_ANY)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        try:
            data, sender_addr = s.recvfrom(4096)
        finally:
            s.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
    finally:
        s.close()
    return data, sender_addr


if len(sys.argv) == 2:
    cmd = sys.argv[1]
    if cmd == "send":
        send(b"hello")
    elif cmd == "listen":
        print(receive())
    else:
        print("unknown command:", cmd)
```  

# Firewalls

You might have to poke holes in your firewall to make it work:

```
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```

# References

- [pymotw multicast article](https://pymotw.com/2/socket/multicast.html)
- [python socket communications](https://medium.com/python-pandemonium/python-socket-communication-e10b39225a4c)
- [iperf gist](https://gist.github.com/jayjanssen/5697813)
- [python udp comm](https://wiki.python.org/moin/UdpCommunication)
