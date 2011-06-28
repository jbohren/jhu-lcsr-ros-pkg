#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_robot)/lib
	cp $(shell rospack find cisst)/lib/libcisstRobot.a $(shell rospack find cisst_robot)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
