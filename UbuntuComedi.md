# Introduction #

Installation of Comedi on a non-standard ubuntu kernel is similar to installing it on any arbitrary linux system.

# Details #

```
git clone git://comedi.org/git/comedi/comedi.git
cd comedi
./autogen.sh
```
(it's ok if autogen yells at you)
```
./configure
make
sudo make install
sudo depmod -a
```
Then restart, and when you come back up, everything should be configured. You may still need to add everyone to the "iocard" group in /etc/group, and reboot again.