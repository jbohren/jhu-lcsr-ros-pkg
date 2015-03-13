# Introduction #

These are notes on installing the Kinect / OpenNI libraries on Ubuntu linux, partially from source, and partially from binary distributions.

# Details #

The current version of ps-engine (PrimeSense Sensor) in the software channels is 5.0.3.3, but there is a bug in even as late as 5.1.0.25 in which an unnecessary USB set interface command causes timeout problems on Linux.

```
Build dependencies:
sudo apt-get install libusb-1.0-0-dev
```

```
export ONI_SRC_ROOT=/path/to/openni/source/dir
export AVIN2_SK_SRC_ROOT=/path/to/avin2/source/dir
```

First, build and install the OpenNI libraries:
```
cd $ONI_SRC_ROOT/Platform/Linux/CreateRedist
chmod +x RedistMaker
./RedistMaker
cd ../Redist/OpenNI-Bin*
chmod +x install.sh
sudo ./install.sh
```

Next, build and install the avin2 SensorKinect libraries:
```
cd $AVIN2_SK_SRC_ROOT/Platform/Linux/CreateRedist
chmod +x RedistMaker
./RedistMaker
cd ../Redist/Sensor-Bin-Linux*
chmod +x install.sh
sudo ./install.sh
```

Finally, copy the header files for development:
```
sudo mkdir /usr/include/openni
sudo cp -R $ONI_SRC_ROOT/Include/* /usr/include/openni
sudo cp -R $AVIN2_SK_SRC_ROOT/Include/* /usr/include/openni
```