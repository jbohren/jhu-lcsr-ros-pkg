# Introduction #

<font color='#FF0000'><b><i>Notice: debian package support for CISST is experimental and unstable</i></b></font>

The CISST libraries and some SAW components can be installed via debian packages on Ubuntu 10.04 LTS.

# Details #

At this time the CISST debian packages are being hosted on the personal domain of developer Jonathan Bohren. To add this APT repository to your system, run:
```
sudo sh -c 'echo "deb http://pkg.jbohren.com/repos/apt/debian lucid main" > /etc/apt/sources.list.d/jbohren.list'
```

Then pull down and set up the appropriate key:
```
wget -O - http://pkg.jbohren.com/repos/apt/conf/jbohren.gpg.key | sudo apt-key add -
```

Update the package manager to get the available packages:
```
sudo apt-get update
```

Now install the CISST core libraries by simply running:
```
sudo apt-get install cisst
```