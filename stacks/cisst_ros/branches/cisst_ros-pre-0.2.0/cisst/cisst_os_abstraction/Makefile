#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_os_abstraction)/lib
	cp $(shell rospack find cisst)/lib/libcisstOSAbstraction.a $(shell rospack find cisst_os_abstraction)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
