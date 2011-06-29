#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_multi_task)/lib
	cp $(shell rospack find cisst)/lib/libcisstMultiTask.a $(shell rospack find cisst_multi_task)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
