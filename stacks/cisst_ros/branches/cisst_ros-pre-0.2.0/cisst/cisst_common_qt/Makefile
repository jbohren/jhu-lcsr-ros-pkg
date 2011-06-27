#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_common_qt)/lib
	cp $(shell rospack find cisst)/lib/libcisstCommonQt.a $(shell rospack find cisst_common_qt)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
