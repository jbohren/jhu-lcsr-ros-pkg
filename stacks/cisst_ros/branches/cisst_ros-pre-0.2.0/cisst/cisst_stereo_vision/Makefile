#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_stereo_vision)/lib
	cp $(shell rospack find cisst)/lib/libcisstStereoVision.a $(shell rospack find cisst_stereo_vision)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
