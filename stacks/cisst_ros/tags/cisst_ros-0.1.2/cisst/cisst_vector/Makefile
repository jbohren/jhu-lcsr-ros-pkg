#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_vector)/lib
	cp $(shell rospack find cisst)/lib/libcisstVector.a $(shell rospack find cisst_vector)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
