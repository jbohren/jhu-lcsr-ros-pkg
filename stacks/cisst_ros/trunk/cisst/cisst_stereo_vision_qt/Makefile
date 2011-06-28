#include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p $(shell rospack find cisst_stereo_vision_qt)/lib
	cp $(shell rospack find cisst)/lib/libcisstStereoVisionQt.a $(shell rospack find cisst_stereo_vision_qt)/lib
	touch ROS_NOBUILD

clean:
	rm -rf lib ROS_NOBUILD
