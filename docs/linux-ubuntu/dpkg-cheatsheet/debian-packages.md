# Debian Package Cheatsheet


| Syntax                      | Description                 | Exmaple                        |
|-----------------------------|-----------------------------|--------------------------------|
| `dpkg -i {.deb package}`	  | Install the package                              |	`dpkg -i zip_2.31-3_i386.deb` |
| `dpkg -i {.deb package}`	  | Upgrade package if it is installed else install a fresh copy of package	| `dpkg -i zip_2.31-3_i386.deb` |
| `dpkg -R {Directory-name}`	| Install all packages recursively from directory	 | `dpkg -R /tmp/downloads` |
| `dpkg -r {package}`       	| Remove/Delete an installed package except configuration files |	`dpkg -r zip` |
| `dpkg -P {package}`         | Remove/Delete everything including configuration files	| `dpkg -P apache-perl` |
| `dpkg -l`	                  | List all installed packages, along with package version and short description	| `dpkg -l` `dpkg -l \| less`  `dpkg -l '*apache*'` `dpkg -l \| grep -i 'sudo'` |
| `dpkg -l {package}`         | List individual installed packages, along with package version and short description | `dpkg -l apache-perl` |
| `dpkg -L {package}`         | Find out files are provided by the installed package i.e. list where files were installed	| `dpkg -L apache-perl` `dpkg -L perl` |
| `dpkg -c {.Deb package}`    | List files provided (or owned) by the package i.e. List all files inside debian .deb package file, very useful to find where files would be installed	| `dpkg -c dc_1.06-19_i386.deb` |
| `dpkg -S {/path/to/file}`   | Find what package owns the file i.e. find out what package does file belong	| `dpkg -S /bin/netstat` `dpkg -S /sbin/ippool` |
| `dpkg -p {package}`	        | Display details about package package group, version, maintainer, Architecture, display depends packages, description etc |	`dpkg -p lsof` |
| `dpkg -s {package} \| grep`  | Status	Find out if Debian package is installed or not (status)	| `dpkg -s lsof | grep Status` |

Here I have built a test package that installs everything to `./tmp`

```
kevin@dalek:~/github/gecko/build$ dpkg -c gecko-0.2.0-Linux.deb
drwxr-xr-x root/root         0 2019-04-06 10:10 ./tmp/
drwxr-xr-x root/root         0 2019-04-06 10:10 ./tmp/bin/
-rwxr-xr-x root/root     32280 2019-04-06 10:10 ./tmp/bin/echo
-rwxr-xr-x root/root     32280 2019-04-06 10:10 ./tmp/bin/logserver
drwxr-xr-x root/root         0 2019-04-06 10:10 ./tmp/include/
drwxr-xr-x root/root         0 2019-04-06 10:10 ./tmp/include/gecko/
-rw-r--r-- root/root       439 2019-04-01 20:18 ./tmp/include/gecko/ascii.hpp
-rw-r--r-- root/root      2265 2019-04-01 20:18 ./tmp/include/gecko/color.hpp
-rw-r--r-- root/root      1294 2019-04-01 20:18 ./tmp/include/gecko/event.hpp
-rw-r--r-- root/root       281 2019-03-17 16:56 ./tmp/include/gecko/exceptions.hpp
-rw-r--r-- root/root       934 2019-04-01 19:28 ./tmp/include/gecko/gecko.hpp
-rw-r--r-- root/root      1267 2019-04-01 20:18 ./tmp/include/gecko/geckocpp.hpp
-rw-r--r-- root/root       209 2019-04-01 20:18 ./tmp/include/gecko/helpers.hpp
-rw-r--r-- root/root      1351 2019-04-01 20:18 ./tmp/include/gecko/log.hpp
drwxr-xr-x root/root         0 2019-04-06 10:10 ./tmp/include/gecko/msgpack/
-rw-r--r-- root/root      4608 2019-04-01 20:27 ./tmp/include/gecko/msgpack/common.hpp
-rw-r--r-- root/root      1360 2019-04-02 18:17 ./tmp/include/gecko/msgpack/msgpack_pub_sub.hpp
-rw-r--r-- root/root       710 2019-03-31 16:50 ./tmp/include/gecko/msgpack/msgs.hpp
-rw-r--r-- root/root     11369 2019-04-01 20:49 ./tmp/include/gecko/msgpack/serialization.hpp
-rw-r--r-- root/root      4096 2019-04-01 20:30 ./tmp/include/gecko/msgpack/stamped.hpp
-rw-r--r-- root/root       788 2019-04-01 20:18 ./tmp/include/gecko/msocket.hpp
-rw-r--r-- root/root       586 2019-04-01 20:17 ./tmp/include/gecko/network.hpp
-rw-r--r-- root/root      1082 2019-04-01 20:17 ./tmp/include/gecko/node.hpp
-rw-r--r-- root/root       424 2019-04-01 20:17 ./tmp/include/gecko/signals.hpp
-rw-r--r-- root/root      2406 2019-04-01 20:17 ./tmp/include/gecko/time.hpp
-rw-r--r-- root/root      2713 2019-04-01 21:02 ./tmp/include/gecko/transport.hpp
-rw-r--r-- root/root       751 2019-04-01 20:17 ./tmp/include/gecko/zmq_rep_req.hpp
-rw-r--r-- root/root      1646 2019-04-01 20:14 ./tmp/include/gecko/zmq_sub_pub.hpp
drwxr-xr-x root/root         0 2019-04-06 10:10 ./tmp/lib/
-rw-r--r-- root/root    220968 2019-04-06 10:10 ./tmp/lib/libgecko.so
```


# References

- [dpkg command](https://www.cyberciti.biz/howto/question/linux/dpkg-cheat-sheet.php)
