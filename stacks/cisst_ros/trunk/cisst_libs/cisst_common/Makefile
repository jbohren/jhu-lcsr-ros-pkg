#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_common)/lib
	cp $(shell rospack find cisst)/lib/libcisstCommon.a $(shell rospack find cisst_common)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
