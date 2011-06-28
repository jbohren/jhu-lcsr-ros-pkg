#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_parameter_types)/lib
	cp $(shell rospack find cisst)/lib/libcisstParameterTypes.a $(shell rospack find cisst_parameter_types)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
