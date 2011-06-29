#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_numerical)/lib
	cp $(shell rospack find cisst)/lib/libcisstNumerical.a $(shell rospack find cisst_numerical)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
