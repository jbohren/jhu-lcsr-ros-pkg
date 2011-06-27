#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_interactive)/lib
	cp $(shell rospack find cisst)/lib/libcisstInteractive.a $(shell rospack find cisst_interactive)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
