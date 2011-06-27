#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_devices)/lib
	cp $(shell rospack find cisst)/lib/libcisstDevices.a $(shell rospack find cisst_devices)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
